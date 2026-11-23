//! Прогиб консольной балки: МКЭ и многотельная цепочка против формул сопромата.
use sim_verify::cases;

#[test]
fn fem_cantilever_matches_euler_bernoulli() {
    for c in [
        cases::cantilever_fem_tip(),
        cases::cantilever_fem_slope(),
        cases::cantilever_fem_shape(),
        cases::cantilever_fem_udl(),
    ] {
        println!("{c}");
        c.assert_pass();
    }
}

#[test]
fn multibody_bar_chain_matches_beam_theory() {
    for c in [
        cases::bar_chain_vs_discrete(),
        cases::bar_chain_vs_continuum(),
        cases::bar_chain_vs_elastica(),
    ] {
        println!("{c}");
        c.assert_pass();
    }
}

#[test]
fn bar_chain_converges_as_one_over_n_squared() {
    // Второй способ должен сходиться к первому с известной скоростью, иначе в движке ошибка.
    let exact = sim_verify::analytic::cantilever_tip_point(0.1, cases::BEAM_L, cases::beam_ei());
    let e10 = (cases::bar_chain_tip_deflection(10, 0.1) - exact).abs();
    let e20 = (cases::bar_chain_tip_deflection(20, 0.1) - exact).abs();
    let ratio = e10 / e20;
    println!("ошибка N=10: {e10:.3e}, N=20: {e20:.3e}, отношение {ratio:.4} (ожидается 4)");
    assert!(
        (ratio - 4.0).abs() < 0.01,
        "порядок сходимости цепочки не 2: {ratio}"
    );
}
