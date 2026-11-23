//! Модели тела открываются, числа в них правдоподобны.
use sim_mujoco::{GeomKind, Sim, compose_myoarm, models, version};

fn describe(name: &str, s: &Sim) {
    let (lo, hi) = s.z_extent();
    let muscles = s.muscles();
    let fsum: f64 = muscles.iter().map(|m| m.f_max).sum();
    println!(
        "{name}: MuJoCo {} | тел {} | суставов {} | DoF {} | мышц {} (ΣF₀ = {:.0} Н) | сухожилий {} | мешей {} | масса {:.2} кг | геометрия по Z {:.3}…{:.3} м | ключевых поз {}",
        version(),
        s.nbody(),
        s.njnt(),
        s.nv(),
        muscles.len(),
        fsum,
        s.ntendon(),
        s.nmesh(),
        s.total_mass(),
        lo,
        hi,
        s.nkey()
    );
}

#[test]
fn ms_human_700_full_body_loads() {
    let s = Sim::load(&models::ms_human_700()).expect("MS-Human-700");
    describe("MS-Human-700", &s);
    assert!(
        s.muscles().len() >= 690,
        "ожидалось ~700 мышц, есть {}",
        s.muscles().len()
    );
    assert!(s.total_mass() > 40.0 && s.total_mass() < 100.0);
    assert!(s.geoms().iter().any(|g| g.kind == GeomKind::Mesh));
}

#[test]
fn ms_human_700_arm_loads() {
    let s = Sim::load(&models::ms_human_700_arm()).expect("MS-Human-700 Manipulation");
    describe("MS-Human-700 рука", &s);
    assert!(s.muscles().len() >= 60);
}

#[test]
fn myoarm_loads() {
    let s = compose_myoarm().expect("MyoArm");
    describe("MyoArm", &s);
    assert!(s.muscles().len() >= 60);
    for m in s.muscles().iter().take(5) {
        println!(
            "  {} F₀ = {:.1} Н, длина {:.3}…{:.3} м",
            m.name, m.f_max, m.length_range[0], m.length_range[1]
        );
    }
}
