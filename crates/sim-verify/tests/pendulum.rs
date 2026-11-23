//! Период маятника против аналитики (малые и большие амплитуды) и сохранение энергии.
use sim_verify::cases;

#[test]
fn small_amplitude_period_matches_2pi_sqrt_l_over_g() {
    let c = cases::pendulum_small_angle();
    println!("{c}");
    c.assert_pass();
}

#[test]
fn large_amplitude_period_matches_elliptic_integral() {
    for amp in [90.0, 170.0] {
        let c = cases::pendulum_large_amplitude(amp);
        println!("{c}");
        c.assert_pass();
    }
}

#[test]
fn physical_pendulum_period() {
    let c = cases::physical_pendulum_rod();
    println!("{c}");
    c.assert_pass();
}

#[test]
fn energy_is_conserved_with_rk4() {
    let c = cases::pendulum_energy_rk4();
    println!("{c}");
    c.assert_pass();
}
