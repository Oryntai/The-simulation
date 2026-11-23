//! Электромагнит: решатель цепи, сила, обмотка.
use sim_em::magcircuit::*;

#[test]
fn holding_force_grows_then_saturates() {
    let mc = example_holding_magnet(CoreMaterial::Nonlinear(BhCurve::aisi_1010()), 0.2e-3);
    let mut last = 0.0;
    let mut gains = Vec::new();
    for i in [0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0] {
        let f = mc.force_maxwell_n(i);
        assert!(f > last);
        if last > 0.0 {
            gains.push(f / last);
        }
        last = f;
    }
    // в линейной зоне удвоение тока даёт ×4, в насыщении — заметно меньше
    assert!(gains[0] > 3.5, "{gains:?}");
    assert!(*gains.last().unwrap() < 1.5, "{gains:?}");
}

#[test]
fn fringing_lowers_gap_flux_density_but_energy_force_stays_consistent() {
    let mut mc = example_holding_magnet(CoreMaterial::Nonlinear(BhCurve::aisi_1010()), 1e-3);
    let b_no = mc.solve(2.0).b_gap_t;
    mc.fringing = true;
    let b_fr = mc.solve(2.0).b_gap_t;
    assert!(b_fr < b_no);
    // с выпучиванием сила по энергии меньше, чем по формуле Максвелла (∂A_g/∂g > 0)
    assert!(mc.force_virtual_work_n(2.0) < mc.force_maxwell_n(2.0));
}

#[test]
fn coil_helpers() {
    let coil = Coil {
        turns: 400.0,
        awg: 24,
        mean_turn_length_m: 0.09,
        cooling_area_m2: 0.012,
        h_w_per_m2k: 12.0,
    };
    let r20 = coil.resistance_ohm(20.0);
    assert!((coil.resistance_ohm(120.0) / r20 - (1.0 + 0.393)).abs() < 1e-12);
    let t = coil.steady_temperature_c(0.5, 25.0).unwrap();
    assert!(t > 25.0 && t < 200.0);
    assert!(
        coil.steady_temperature_c(50.0, 25.0).is_none(),
        "тепловой разгон"
    );
    assert!(turns_in_window(1e-4, 0.55e-3, 0.55) > 200.0);
}

#[test]
fn circuit_checks_pass() {
    for c in sim_em::magcircuit::checks() {
        c.assert_pass();
    }
}
