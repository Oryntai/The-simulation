//! Прежде чем верить контактам в комнате, сверяем их с ручным расчётом: вес на опоре,
//! порог трения, скольжение, удар тяжёлого о лёгкое, отскок, цепочка сил в стопке.

use sim_mujoco::contact::restitution_solref;
use sim_mujoco::Sim;

const G: f64 = 9.81;
const OPTS: &str = r#"cone="elliptic" impratio="10""#;

fn run_to(s: &mut Sim, t: f64) {
    while s.time() < t {
        s.step();
    }
}

fn normal_sum(s: &Sim) -> f64 {
    s.contacts().iter().map(|c| c.normal_force).sum()
}

fn close(name: &str, got: f64, want: f64, rel: f64) {
    println!("{name}: симуляция {got:.5}, расчёт {want:.5}, ошибка {:.3}%", 100.0 * (got - want).abs() / want.abs());
    assert!((got - want).abs() <= rel * want.abs(), "{name}: {got} против {want} (допуск {:.1}%)", rel * 100.0);
}

#[test]
fn resting_box_presses_with_its_weight() {
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.001" gravity="0 0 -{G}" {OPTS}/><worldbody>
        <geom type="plane" size="5 5 0.1"/>
        <body pos="0 0 0.11"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="7"/></body>
        </worldbody></mujoco>"#
    ))
    .unwrap();
    run_to(&mut s, 2.0);
    assert!(s.numerically_ok());
    close("вес ящика 7 кг на полу, Н", normal_sum(&s), 7.0 * G, 0.005);
}

fn incline(alpha_deg: f64, mu: f64) -> Sim {
    let a = alpha_deg.to_radians();
    Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.001" gravity="{gx} 0 {gz}" {OPTS}/><worldbody>
        <geom type="plane" size="50 50 0.1" friction="{mu} 0.005 0.0001"/>
        <body pos="0 0 0.1"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="2" friction="{mu} 0.005 0.0001"/></body>
        </worldbody></mujoco>"#,
        gx = G * a.sin(),
        gz = -G * a.cos(),
    ))
    .unwrap()
}

#[test]
fn friction_threshold_matches_tan_alpha() {
    let mu = 0.5_f64;
    let critical = mu.atan().to_degrees();
    let mut stuck = incline(critical - 3.0, mu);
    run_to(&mut stuck, 3.0);
    let creep = stuck.qpos()[0].abs();
    println!("ниже порога на 3°: сползло {:.3} мм за 3 с", creep * 1000.0);
    assert!(stuck.numerically_ok());
    assert!(creep < 0.002, "ящик не должен ехать ниже порога: {creep} м");

    let mut sliding = incline(critical + 3.0, mu);
    run_to(&mut sliding, 3.0);
    assert!(sliding.qpos()[0] > 0.05, "выше порога ящик обязан поехать: {} м", sliding.qpos()[0]);
}

#[test]
fn sliding_acceleration_matches_formula() {
    let (alpha, mu) = (35.0_f64, 0.4_f64);
    let mut s = incline(alpha, mu);
    run_to(&mut s, 0.5);
    let v1 = s.qvel()[0];
    run_to(&mut s, 1.5);
    let v2 = s.qvel()[0];
    let a = alpha.to_radians();
    assert!(s.numerically_ok());
    close("ускорение на склоне, м/с²", v2 - v1, G * (a.sin() - mu * a.cos()), 0.02);
}

fn collision(m1: f64, m2: f64, e: f64) -> (f64, f64) {
    let (k, b) = restitution_solref(e, 0.01);
    let geom = format!(r#"type="sphere" size="0.05" solref="{k} {b}" solimp="0.99 0.99 0.001 0.5 2""#);
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.00025" gravity="0 0 0" {OPTS}/><worldbody>
        <body pos="0 0 1"><freejoint/><geom {geom} mass="{m1}"/></body>
        <body pos="0.2 0 1"><freejoint/><geom {geom} mass="{m2}"/></body>
        </worldbody></mujoco>"#
    ))
    .unwrap();
    s.qvel_mut()[0] = 1.0;
    run_to(&mut s, 0.5);
    assert!(s.numerically_ok());
    let (v1, v2) = (s.qvel()[0], s.qvel()[6]);
    let p = m1 * v1 + m2 * v2;
    close(&format!("импульс {m1} кг → {m2} кг, кг·м/с"), p, m1, 1e-6);
    (v1, v2)
}

#[test]
fn heavy_hits_light_and_light_hits_heavy() {
    for (m1, m2) in [(10.0, 0.5), (0.5, 10.0), (3.0, 3.0)] {
        let e = 0.5;
        let (v1, v2) = collision(m1, m2, e);
        let want1 = (m1 - e * m2) / (m1 + m2);
        let want2 = (1.0 + e) * m1 / (m1 + m2);
        println!("{m1} кг → {m2} кг: v1 {v1:.4} (расчёт {want1:.4}), v2 {v2:.4} (расчёт {want2:.4})");
        close("скорость лёгкого/ударенного после удара, м/с", v2, want2, 0.05);
        assert!((v1 - want1).abs() < 0.05, "скорость ударившего: {v1} против {want1}");
    }
}

#[test]
fn bounce_height_matches_restitution() {
    for e in [0.2, 0.4, 0.6, 0.8] {
        bounce(e);
    }
}

/// Упругость отдельно от силы тяжести: шар в невесомости бьётся о неподвижную стену.
#[test]
fn restitution_against_a_wall_without_gravity() {
    let wall = |e: f64, dt: f64| {
        let (k, b) = restitution_solref(e, 0.01);
        let params = format!(r#"solref="{k} {b}" solimp="0.99 0.99 0.001 0.5 2""#);
        let mut s = Sim::from_xml(&format!(
            r#"<mujoco><option timestep="{dt}" gravity="0 0 0" {OPTS}/><worldbody>
            <geom type="box" size="0.05 1 1" pos="0.3 0 1" {params}/>
            <body pos="0 0 1"><freejoint/><geom type="sphere" size="0.05" mass="1" {params}/></body>
            </worldbody></mujoco>"#
        ))
        .unwrap();
        s.qvel_mut()[0] = 2.0;
        run_to(&mut s, 0.5);
        assert!(s.numerically_ok());
        -s.qvel()[0] / 2.0
    };
    for e in [0.2, 0.4, 0.6, 0.8] {
        let runs: Vec<f64> = [0.00025, 0.0001, 0.00005, 0.00002].iter().map(|dt| wall(e, *dt)).collect();
        println!("сходимость при e = {e}: шаг 0.25/0.1/0.05/0.02 мс → {runs:.4?}");
        close(&format!("восстановление у стены при e = {e}"), *runs.last().unwrap(), e, 0.03);
    }
}

/// Удар короткий: за 10 мс сила тяжести отняла бы ~0.1 м/с, чего формула e²·h не учитывает.
fn bounce(e: f64) {
    let h = 0.5_f64;
    let (k, b) = restitution_solref(e, 0.002);
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.00005" gravity="0 0 -{G}" {OPTS}/><worldbody>
        <geom type="plane" size="5 5 0.1" solref="{k} {b}" solimp="0.99 0.99 0.001 0.5 2"/>
        <body pos="0 0 {z}"><freejoint/><geom type="sphere" size="0.05" mass="1" solref="{k} {b}" solimp="0.99 0.99 0.001 0.5 2"/></body>
        </worldbody></mujoco>"#,
        z = h + 0.05,
    ))
    .unwrap();
    let mut touched = false;
    let mut peak = f64::NEG_INFINITY;
    while s.time() < 2.0 {
        s.step();
        touched |= !s.contacts().is_empty();
        if touched && s.contacts().is_empty() && s.qvel()[2] > 0.0 {
            peak = peak.max(s.qpos()[2] - 0.05);
        }
        if peak > 0.0 && s.qvel()[2] < 0.0 {
            break;
        }
    }
    assert!(s.numerically_ok());
    close(&format!("высота отскока при e = {e}, м"), peak, e * e * h, 0.08);
}

#[test]
fn force_chain_through_a_stack() {
    let (heavy, light) = (20.0, 0.3);
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.001" gravity="0 0 -{G}" {OPTS}/><worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"/>
        <body pos="0 0 0.011"><freejoint/><geom name="plate" type="box" size="0.2 0.2 0.01" mass="{light}"/></body>
        <body pos="0 0 0.125"><freejoint/><geom name="weight" type="box" size="0.1 0.1 0.1" mass="{heavy}"/></body>
        </worldbody></mujoco>"#
    ))
    .unwrap();
    run_to(&mut s, 3.0);
    assert!(s.numerically_ok());
    let floor = s.id(sim_mujoco::ffi::mjtObj::mjOBJ_GEOM, "floor").unwrap() as i32;
    let on = |g: i32, c: &sim_mujoco::contact::ContactInfo| c.geom[0] == g || c.geom[1] == g;
    let contacts = s.contacts();
    let on_floor: f64 = contacts.iter().filter(|c| on(floor, c)).map(|c| c.normal_force).sum();
    let between: f64 = contacts.iter().filter(|c| !on(floor, c)).map(|c| c.normal_force).sum();
    close("пол держит всю стопку, Н", on_floor, (heavy + light) * G, 0.005);
    close("лёгкая пластина несёт вес гири, Н", between, heavy * G, 0.005);
}

#[test]
fn numerical_blowup_is_reported() {
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.001" gravity="0 0 -{G}"/><worldbody>
        <body pos="0 0 1"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body>
        </worldbody></mujoco>"#
    ))
    .unwrap();
    assert!(s.numerically_ok());
    s.qvel_mut()[2] = f64::NAN;
    s.step();
    assert!(!s.numerically_ok(), "расхождение должно быть видно, MuJoCo сбрасывает сцену молча");
    s.reset();
    assert!(s.numerically_ok());
}
