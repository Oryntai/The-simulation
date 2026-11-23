use sim_process::work::rotary::{AngularStop, RotaryMedium};

#[test]
fn угловой_упор_сверен_по_энергии_и_чувствительности() {
    // Синтетический стенд: 2 м плеча, 3 Н/м и четверть радиана хода.
    let q = 0.25_f64;
    let baseline = AngularStop::new(0.0, 2.0, 3.0, 0.0).unwrap();
    let changed = AngularStop::new(0.0, 2.0, 5.0, 0.0).unwrap();
    let (f, m) = baseline.evaluate(q, 2.0).unwrap();
    let (f_new, m_new) = changed.evaluate(q, 2.0).unwrap();
    // Подъём энергии линейной пружины: U = ½ k r² q².
    let energy = |angle: f64| 6.0 * angle.powi(2);
    // Двоичный шаг исключает потерю точности при вычитании в этой квадратичной задаче.
    let h = 1.0 / 1024.0;
    assert_eq!(m, -(energy(q + h) - energy(q - h)) / (2.0 * h));
    assert_eq!(f_new - f, 1.0);
    assert_eq!(m_new - m, -2.0);
    assert_eq!(baseline.evaluate(-q, 2.0).unwrap(), (0.0, 0.0));
    assert!(baseline.evaluate(-q, f64::NAN).is_err());
    assert!(AngularStop::new(0.0, 0.0, 3.0, 0.0).is_err());
}

#[test]
fn нагрузка_среды_сверена_по_моменту_и_мощности() {
    // Синтетический стенд: гармоника 7 Н·м, сопротивление 2 Н·м·с²/рад².
    let model = RotaryMedium::new(7.0, 2.0).unwrap();
    assert_eq!(model.moment(std::f64::consts::FRAC_PI_2, 0.0).unwrap(), 7.0);
    for v in [-3.0, 3.0] {
        assert_eq!(model.moment(0.0, v).unwrap() * v, -54.0);
    }
    assert!(model.moment(f64::INFINITY, 0.0).is_err());
    assert!(RotaryMedium::new(7.0, -1.0).is_err());
}
