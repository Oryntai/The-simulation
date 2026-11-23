//! Свободное падение против z = z₀ − g·t²/2, порядок сходимости схемы, вращение свободного тела.
use sim_verify::cases;

#[test]
fn position_and_velocity_match_analytic() {
    for c in [cases::free_fall_position(), cases::free_fall_velocity()] {
        println!("{c}");
        c.assert_pass();
    }
}

#[test]
fn semi_implicit_euler_error_is_known_and_first_order() {
    for c in [
        cases::free_fall_euler_error(),
        cases::free_fall_convergence_order(),
    ] {
        println!("{c}");
        c.assert_pass();
    }
}

#[test]
fn torque_free_rotation_conserves_momentum_and_energy() {
    for c in [cases::torque_free_momentum(), cases::torque_free_energy()] {
        println!("{c}");
        c.assert_pass();
    }
}
