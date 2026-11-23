use sim_mujoco::Sim;
use sim_mujoco::contact::ContactWrench;
use sim_mujoco::state::{BodyWrench, EqualitySwitch};

// Настройки синтетического стенда. Прямой solref и конус заданы по
// справочнику MuJoCo 4.0.1. Частота 4 кГц разрешает контактную моду.
const DT: f64 = 0.00025;
const SOLVER: &str =
    r#"cone="elliptic" impratio="10" solver="Newton" iterations="60" tolerance="1e-12""#;
const NORMAL: &str = r#"solref="-9000 -120" solimp="0.9 0.9 0.001 0.5 2""#;

fn sim(gravity: [f64; 3], bodies: &str, contact: &str) -> Sim {
    let [gx, gy, gz] = gravity;
    Sim::from_xml(&format!(
        r#"<mujoco><option timestep="{DT}" gravity="{gx} {gy} {gz}" {SOLVER}>
        <flag diagexact="enable"/></option><worldbody>{bodies}</worldbody>{contact}</mujoco>"#
    ))
    .unwrap()
}

fn on(contact: &ContactWrench, geom: &str) -> f64 {
    if contact.geometries[1] == geom {
        1.0
    } else {
        assert_eq!(contact.geometries[0], geom);
        -1.0
    }
}

fn close(got: f64, expected: f64, absolute: f64) {
    assert!(
        (got - expected).abs() <= absolute,
        "{got:.12} против {expected:.12}, допуск {absolute}"
    );
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}

#[test]
fn прямой_solref_не_зависит_от_шага_при_той_же_позе_и_нагрузке() {
    // Блок 2 кг вдавлен в опору на 1 мм. Один нормальный ряд, постоянный
    // импеданс 0,95, ускорительная жёсткость 9000 с⁻², нулевое гашение.
    // Все параметры заданы для аналитически проверяемого численного стенда.
    for hz in [40_000, 80_000, 160_000] {
        let dt = 1.0 / f64::from(hz);
        let xml = format!(
            r#"<mujoco><option timestep="{dt}" gravity="0 0 0">
              <flag diagexact="enable"/>
            </option><worldbody>
              <geom name="опора" type="plane" size="1 1 .1"/>
              <body name="блок" pos="0 0 .099">
                <joint type="slide" axis="0 0 1"/>
                <geom name="поверхность" type="sphere" size=".1" mass="2"/>
              </body>
            </worldbody><contact>
              <pair geom1="опора" geom2="поверхность" condim="1"
                    solref="-9000 0" solimp=".95 .95 .001 .5 2"/>
            </contact></mujoco>"#,
        );
        let mut sim = Sim::from_xml(&xml).unwrap();
        for load in [0.0, -10.0] {
            sim.qfrc_applied_mut()[0] = load;
            sim.forward();
            let contacts = sim.contact_wrenches().unwrap();
            assert_eq!(contacts.len(), 1);
            let force = dot(contacts[0].force_n, contacts[0].frame[0]);
            // Из (A+R)·F = aref − J·qacc_smooth при J=1, A=1/m,
            // R=(1−d)/(m·d), aref=k·глубина/d (MuJoCo 4.0.1).
            close(force, 2.0 * 9000.0 * 0.001 - 0.95 * load, 1e-10);
        }
    }
}

#[test]
fn rolling_resistance_matches_work_and_inertia() {
    // Шар 2 кг радиуса 0.1 м, I=2mR²/5, N=mg=20 Н.
    // Коэффициент сопротивления качению — плечо 0.002 м.
    let radius = 0.1;
    let mass = 2.0;
    let inertia = 2.0 / 5.0 * mass * radius * radius;
    let mut s = sim(
        [0.0, 0.0, -10.0],
        r#"
        <geom name="опора" type="plane" size="5 5 .1"/>
        <body name="блок" pos="0 0 .1"><freejoint/>
            <geom name="образец" type="sphere" size=".1" mass="2"/>
        </body>"#,
        r#"<contact><pair geom1="опора" geom2="образец" condim="6"
         friction=".6 .6 .001 .002 .002" solref="-15000 -50"
         solreffriction="0 -20" solimp=".99 .99 .001 .5 2"/></contact>"#,
    );
    for _ in 0..1600 {
        s.step();
    }
    s.qvel_mut()[0] = 0.5;
    s.qvel_mut()[4] = 0.5 / radius;
    let energy = |s: &Sim| {
        let v = s.qvel();
        0.5 * mass * dot(v[..3].try_into().unwrap(), v[..3].try_into().unwrap())
            + 0.5 * inertia * dot(v[3..6].try_into().unwrap(), v[3..6].try_into().unwrap())
            + mass * 10.0 * s.qpos()[2]
    };
    let begin = energy(&s);
    let mut work = 0.0;
    let mut torque_work = 0.0;
    let mut roll_angle = 0.0;
    let mut rolling_torque: f64 = 0.0;
    let mut normal_impulse = 0.0;
    let mut expected_rolling_work = 0.0;
    let mut contact_steps = 0;
    let start_normal_velocity = s.qvel()[2];
    // Полсекунды: независимая проверка работы и замедления до остановки.
    for _ in 0..2000 {
        let v: [f64; 6] = s.qvel()[..6].try_into().unwrap();
        let center: [f64; 3] = s.qpos()[..3].try_into().unwrap();
        let step = s.step_with_contacts().unwrap();
        assert!(step.contacts.len() <= 1);
        let Some(point) = step.contacts.first() else {
            continue;
        };
        let v: [f64; 6] = std::array::from_fn(|i| (v[i] + s.qvel()[i]) / 2.0);
        contact_steps += 1;
        let sign = on(point, "образец");
        let rx = point.point_m[0] - center[0];
        let rz = point.point_m[2] - center[2];
        let v_contact = [v[0] + v[4] * rz, v[1], v[2] - v[4] * rx];
        let moment = sign * point.torque_nm[1];
        assert!(moment < 0.0);
        normal_impulse += sign * point.force_n[2] * DT;
        expected_rolling_work -= 0.002 * sign * point.force_n[2] * v[4] * DT;
        torque_work += moment * v[4] * DT;
        work += (sign * dot(point.force_n, v_contact) + moment * v[4]) * DT;
        roll_angle += v[4] * DT;
        rolling_torque += moment * DT;
    }
    let lost = energy(&s) - begin;
    let expected_torque = -0.002 * 20.0;
    let final_v = 0.5 + 0.5 * expected_torque / (mass * radius + inertia / radius);
    println!(
        "качение: v={:.9} против {final_v:.9} м/с, ΔE={lost:.9}, ∫Pdt={work:.9}, Wкач={torque_work:.9} против {expected_rolling_work:.9} Дж, Jn={normal_impulse:.9}, тактов контакта={contact_steps}",
        s.qvel()[0]
    );
    assert!(contact_steps >= 1900); // Не менее 95% окна имеет ненулевую опору.
    // Работа рассчитана со средней скоростью шага, а Euler обновляет
    // координату конечной скоростью. Поправка потенциальной энергии
    // суммируется в mg*h*Δv_z/2.
    let gravity_discretization = mass * 10.0 * DT * (s.qvel()[2] - start_normal_velocity) / 2.0;
    close(lost, work + gravity_discretization, 1e-8); // Запас на поворот базиса за шаг в работе момента.
    close(
        normal_impulse,
        20.0 * 0.5 + mass * (s.qvel()[2] - start_normal_velocity),
        1e-6, // Разрешение интеграла до 0.1 ppm веса за окно опыта.
    );
    close(torque_work, expected_rolling_work, 0.0003); // Слабая связь направлений в конусе.
    close(torque_work, expected_torque * roll_angle, 0.002); // Независимая проверка по весу.
    close(rolling_torque / 0.5, expected_torque, 0.001);
    close(s.qvel()[0], final_v, 0.007);
}

#[test]
fn rod_clearance_matches_collision_and_axial_reaction() {
    // Стержень шириной 20 мм проверяется в проходах 22 и 18 мм.
    // Передняя грань начинается в z=0.04 м, опора начинается в z=0.08 м,
    // свободный ход при v=0.05 м/с за 2.2 с даёт 0.11 м. Малая скорость
    // держит тормозной путь мягкой опоры меньше бокового нахлёста 1 мм.
    let run = |gap: f64| {
        let wall = 0.02 + gap / 2.0;
        let mut s = sim(
            [0.0; 3],
            &format!(
                r#"
            <geom name="левая" type="box" pos="-{wall} 0 .12" size=".02 .1 .04"/>
            <geom name="правая" type="box" pos="{wall} 0 .12" size=".02 .1 .04"/>
            <body name="стержень"><joint name="ось" type="slide" axis="0 0 1"/>
                <geom name="проба" type="box" size=".01 .01 .04" mass="1"/>
            </body>"#
            ),
            &format!(
                r#"<contact>
            <pair geom1="левая" geom2="проба" condim="1" {NORMAL}/>
            <pair geom1="правая" geom2="проба" condim="1" {NORMAL}/>
            </contact>"#
            ),
        );
        s.qvel_mut()[0] = 0.05;
        let mut impulse = 0.0;
        let mut contacts = 0;
        for _ in 0..8800 {
            for c in s.step_with_contacts().unwrap().contacts {
                impulse += on(&c, "проба") * c.force_n[2] * DT;
                contacts += 1;
            }
        }
        (s.qpos()[0], s.qvel()[0], impulse, contacts)
    };
    let (wide, v_wide, j_wide, contacts_wide) = run(0.022);
    let (narrow, v_narrow, j_narrow, contacts_narrow) = run(0.018);
    println!("зазор ±2 мм: ход {wide:.9}/{narrow:.9} м, импульс {j_wide:.9}/{j_narrow:.9} Н·с");
    close(wide, 0.11, 1e-12);
    close(v_wide, 0.05, 1e-12);
    assert_eq!(contacts_wide, 0);
    assert!(contacts_narrow > 0 && narrow < 0.041 && j_narrow < -0.03);
    close(j_narrow, v_narrow - 0.05, 1e-10); // m=1 кг, проекция импульса на ось.
}

#[test]
fn wedge_force_balances_axis_and_transverse_support() {
    // Нормаль плоскости наклонена на 30° от Z против движения оси X.
    // Осевое усилие 10 Н даёт N=F/sin(30°)=20 Н и боковую реакцию 10√3 Н.
    let angle = 30.0_f64.to_radians();
    let z = 0.05 / angle.cos();
    let mut s = sim(
        [0.0; 3],
        &format!(
            r#"
        <geom name="клин" type="plane" size="2 2 .1" quat="{} 0 {} 0"/>
        <body name="каретка" pos="0 0 {z}"><joint name="ось" type="slide" axis="1 0 0"/>
            <geom name="образец" type="sphere" size=".05" mass="2"/>
        </body>"#,
            (angle / 2.0).cos(),
            -(angle / 2.0).sin()
        ),
        &format!(r#"<contact><pair geom1="клин" geom2="образец" condim="1" {NORMAL}/></contact>"#),
    );
    s.qfrc_applied_mut()[0] = 10.0;
    for _ in 0..8000 {
        s.step();
    }
    let step = s.step_with_contacts().unwrap();
    assert_eq!(step.contacts.len(), 1);
    let point = &step.contacts[0];
    let [x, _, z] = point.force_n.map(|f| on(point, "образец") * f);
    println!(
        "клин: Fx={x:.9}, Fz={z:.9} Н против −10 и {:.9}",
        10.0 * 3.0_f64.sqrt()
    );
    close(x, -10.0, 0.01); // 0.1% приложенного осевого усилия.
    close(z, 10.0 * 3.0_f64.sqrt(), 0.02);
    assert!(point.separation_m < 0.0 && s.qvel()[0].abs() < 1e-5);
}

#[test]
fn raw_explicit_pairs_bypass_mask_filter() {
    // Одна и та же пара с непересекающимися битами. Явная запись обходит
    // их в движке. Поэтому сборщик проверяет обе маски до записи пары.
    let bodies = r#"<geom name="опора" type="plane" size="1 1 .1" contype="1" conaffinity="1"/>
        <body pos="0 0 .099"><freejoint/><geom name="блок" type="sphere"
            size=".1" mass="1" contype="2" conaffinity="2"/></body>"#;
    let mut filtered = sim([0.0; 3], bodies, "");
    assert!(filtered.step_with_contacts().unwrap().contacts.is_empty());
    let mut explicit = sim(
        [0.0; 3],
        bodies,
        &format!(r#"<contact><pair geom1="опора" geom2="блок" condim="1" {NORMAL}/></contact>"#),
    );
    assert!(!explicit.contact_pair_allowed(["опора", "блок"]).unwrap());
    let contacts = explicit.step_with_contacts().unwrap().contacts;
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].force_n[2] > 0.0);
}

#[test]
fn body_wrench_and_switch_are_atomic_and_replayable() {
    // Свободный шар m=2 кг, r=0.1 м, инерция 0.008 кг·м².
    // Сдвижная опора испытывает 4 Н и активируется на границе такта.
    let mut s = sim(
        [0.0; 3],
        r#"
        <body name="каретка"><joint name="ось" type="slide" axis="1 0 0"/>
            <geom name="опора" type="sphere" size=".1" mass="2" contype="0" conaffinity="0"/></body>
        <body name="блок" pos="0 0 1"><freejoint/><geom name="круг"
            type="sphere" size=".1" mass="2" contype="0" conaffinity="0"/></body>"#,
        r#"<equality><joint name="фиксация" joint1="ось" polycoef="0 0 0 0 0"
            active="false" solref="-9000 -120" solimp=".9 .9 .001 .5 2"/></equality>"#,
    );
    let body = BodyWrench {
        body: "блок",
        force_n: [0.0, 4.0, 0.0],
        torque_nm: [0.0, 1.0, 0.0],
    };
    let axis = BodyWrench {
        body: "каретка",
        force_n: [4.0, 0.0, 0.0],
        torque_nm: [0.0; 3],
    };
    let turn_on = EqualitySwitch {
        name: "фиксация",
        active: true,
    };
    let start = s.state_hash();
    let snap = s.snapshot();
    // Точные параметры синтетической среды. Меняем только модель, до шага.
    s.set_medium(2.0, 0.01, [0.0, 0.0, 1.0]);
    assert_ne!(s.state_hash(), start);
    let mut medium_branch = s.fork().unwrap();
    assert_eq!(medium_branch.state_hash(), s.state_hash());
    s.restore(&snap).unwrap();
    assert_eq!(s.state_hash(), start);
    assert_eq!(s.model().opt.density, 0.0);
    assert_eq!(s.model().opt.viscosity, 0.0);
    assert_eq!(s.model().opt.wind, [0.0; 3]);
    medium_branch.set_medium(0.0, 0.0, [0.0; 3]);
    assert_eq!(medium_branch.state_hash(), start);
    assert!(s.apply_step_effects(&[body, body], &[turn_on]).is_err());
    assert!(s.apply_step_effects(&[body], &[turn_on, turn_on]).is_err());
    assert!(
        s.apply_step_effects(
            &[body],
            &[EqualitySwitch {
                name: "нет",
                active: true
            }]
        )
        .is_err()
    );
    assert!(
        s.apply_step_effects(
            &[BodyWrench {
                torque_nm: [f64::INFINITY; 3],
                ..body
            }],
            &[]
        )
        .is_err()
    );
    assert_eq!(s.state_hash(), start);
    s.apply_step_effects(&[body], &[]).unwrap();
    assert_ne!(s.state_hash(), start); // Нагрузка влияет на отпечаток даже до шага.
    let with_load = s.state_hash();
    s.apply_step_effects(&[body], &[turn_on]).unwrap();
    assert_ne!(s.state_hash(), with_load); // Независимый вклад переключения.
    assert!(s.equality_active("фиксация").unwrap());
    assert!(s.equality_reaction("фиксация").unwrap().is_none());
    s.apply_step_effects(&[body, axis], &[turn_on]).unwrap();
    let state_with_effects = s.snapshot();
    let start_step = s.step_with_contacts().unwrap();
    assert_eq!((start_step.start_s, start_step.end_s), (0.0, DT));
    // Индекс 0 — поступательная ось, затем 6 компонент свободного тела.
    close(s.qvel()[2], DT * 4.0 / 2.0, 1e-14);
    close(s.qvel()[5], DT / 0.008, 1e-14);
    let before_fork = s.state_hash();
    let mut fork = s.fork().unwrap();
    assert_eq!(fork.state_hash(), before_fork);
    fork.apply_step_effects(
        &[],
        &[EqualitySwitch {
            active: false,
            ..turn_on
        }],
    )
    .unwrap();
    assert!(!fork.equality_active("фиксация").unwrap());
    assert_eq!(s.state_hash(), before_fork);
    assert!(s.restore(&fork.snapshot()).is_err());
    s.restore(&state_with_effects).unwrap();
    s.step_with_contacts().unwrap();
    assert_eq!(s.state_hash(), before_fork);
    s.apply_step_effects(&[axis], &[]).unwrap();
    for _ in 0..4000 {
        s.step();
    }
    let reaction = s.equality_reaction("фиксация").unwrap().unwrap();
    println!("фиксация оси: реакция {:.9} Н против −4 Н", reaction[0]);
    close(reaction[0], -4.0, 0.01); // 0.25% от приложенного усилия.
    s.restore(&snap).unwrap();
    assert_eq!(s.state_hash(), start);
    assert!(!s.equality_active("фиксация").unwrap());
}

#[test]
fn malformed_step_cannot_produce_integrated_reactions() {
    let mut s = sim(
        [0.0; 3],
        r#"<body><freejoint/><geom name="блок" type="sphere" size=".1" mass="1"/></body>"#,
        "",
    );
    let time = s.time();
    s.qvel_mut()[0] = f64::NAN;
    assert!(s.step_with_contacts().is_err());
    assert_eq!(s.time(), time);
    s.reset();
    s.set_medium(0.0, 0.0, [f64::INFINITY, 0.0, 0.0]);
    assert!(s.step_with_contacts().is_err());
    assert_eq!(s.time(), time);
    s.set_medium(0.0, 0.0, [0.0; 3]);
    assert_eq!(s.step_with_contacts().unwrap().end_s, DT);
    let mut rk = Sim::from_xml(r#"<mujoco><option integrator="RK4"/></mujoco>"#).unwrap();
    assert!(rk.step_with_contacts().is_err());
    assert_eq!(rk.time(), 0.0);
}

#[test]
fn несовпавший_weld_не_разгоняет_покоящиеся_каретки() {
    // Две каретки по 1 кг, без внешних сил. Целевая конфигурация weld
    // совпадает с исходной. Перед включением одна каретка смещена на 1 мм.
    let mut sim = Sim::from_xml(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/>
        <worldbody><body name="a"><joint type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
        <body name="b"><joint type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body></worldbody>
        <equality><weld name="стык" body1="a" body2="b" active="false"/></equality>
        </mujoco>"#,
    )
    .unwrap();
    sim.qpos_mut()[0] = 0.001;
    let activation = sim.apply_step_effects(
        &[],
        &[EqualitySwitch {
            name: "стык",
            active: true,
        }],
    );
    sim.step();
    let energy_j = sim.qvel().iter().map(|v| v * v / 2.0).sum::<f64>();
    eprintln!("кинетическая энергия после захвата: {energy_j:.12e} Дж");
    assert!(activation.is_err());
    assert!(!sim.equality_active("стык").unwrap());
    assert_eq!(energy_j, 0.0);
}
