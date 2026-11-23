use sim_scenarios::elbow_plant::*;

#[test]
fn anatomy_vs_winter_anthropometry() {
    let a = ElbowParams::from_anatomy().unwrap();
    // MS-Human-700 — человек ~75 кг (модель 60 кг без мышц-оболочек? берём массу модели).
    let w = ElbowParams::winter(75.0, 1.75);
    println!(
        "анатомия: m {:.3} кг, r_c {:.3} м, кисть {:.3} м, I {:.4} кг·м², предел {:.0}°",
        a.mass,
        a.com,
        a.hand,
        a.inertia,
        a.max_angle.to_degrees()
    );
    println!(
        "Winter:   m {:.3} кг, r_c {:.3} м, кисть {:.3} м, I {:.4} кг·м²",
        w.mass, w.com, w.hand, w.inertia
    );
    // Разные люди и разные способы — ждём совпадения в пределах ~35 %.
    for (x, y, what) in [
        (a.mass, w.mass, "масса"),
        (a.com, w.com, "центр масс"),
        (a.inertia, w.inertia, "инерция"),
    ] {
        let r = x / y;
        assert!(r > 0.65 && r < 1.35, "{what}: {r}");
    }
}

#[test]
fn small_oscillation_period_matches_physical_pendulum() {
    let p = ElbowParams::winter(75.0, 1.75);
    let mut e = ElbowPlant::new(
        ElbowParams {
            damping: 0.0,
            ..p.clone()
        },
        0.05,
    );
    let dt = 1e-4;
    let mut t = 0.0;
    let mut last = e.theta;
    let mut crossings = Vec::new();
    while crossings.len() < 5 && t < 10.0 {
        e.step(dt, 0.0, 0.0);
        t += dt;
        if last > 0.0 && e.theta <= 0.0 {
            crossings.push(t);
        }
        last = e.theta;
    }
    // Упор при θ < 0 мешает — сдвинем: качаем вокруг 60° под постоянным моментом.
    let _ = crossings;
    let theta0 = 60f64.to_radians();
    let mut e = ElbowPlant::new(
        ElbowParams {
            damping: 0.0,
            ..p.clone()
        },
        theta0 + 0.02,
    );
    let hold = e.holding_torque(theta0);
    let (mut t, mut last, mut cr) = (0.0, 0.02, Vec::new());
    while cr.len() < 4 && t < 10.0 {
        e.step(dt, hold, 0.0);
        t += dt;
        let d = e.theta - theta0;
        if last > 0.0 && d <= 0.0 {
            cr.push(t);
        }
        last = d;
    }
    let period = (cr[3] - cr[0]) / 3.0;
    let k = p.mass * p.com * G * theta0.cos();
    let analytic = 2.0 * std::f64::consts::PI * (p.inertia / k).sqrt();
    println!("период малых колебаний вокруг 60°: {period:.4} с, аналитика {analytic:.4} с");
    assert!((period - analytic).abs() / analytic < 0.01);
}

#[test]
fn energy_is_conserved_without_damping() {
    // Качаем вокруг 60° под постоянным моментом, не касаясь упоров:
    // сохраняется E = T + V_тяжести − τ·θ.
    let p = ElbowParams::winter(75.0, 1.75);
    let mut e = ElbowPlant::new(ElbowParams { damping: 0.0, ..p }, 75f64.to_radians());
    e.load.payload = 5.0;
    let tau = e.holding_torque(60f64.to_radians());
    let total = |e: &ElbowPlant| e.energy(0.01) - tau * e.theta;
    let e0 = total(&e);
    let mut worst: f64 = 0.0;
    for _ in 0..50_000 {
        e.step(1e-4, tau, 0.01);
        assert_eq!(e.stop_torque, 0.0, "задели упор");
        worst = worst.max((total(&e) - e0).abs());
    }
    println!("энергия: наибольший уход {worst:.2e} Дж из {e0:.3}");
    assert!(worst / e0.abs() < 1e-3);
}

#[test]
fn water_reduces_holding_torque() {
    let p = ElbowParams::winter(75.0, 1.75);
    let mut e = ElbowPlant::new(p, 1.0);
    let dry = e.holding_torque(std::f64::consts::FRAC_PI_2);
    e.load.arm_buoyancy = 0.88;
    let wet = e.holding_torque(std::f64::consts::FRAC_PI_2);
    assert!((wet / dry - 0.12).abs() < 1e-9);
}
