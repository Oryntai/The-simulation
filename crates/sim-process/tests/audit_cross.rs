use std::num::NonZeroU32;

use sim_process::work::{
    heat::solder_network::{SolderContact, SolderNetworkInput, SolderNetworkSpec},
    solder_window::{
        SolderBounds, SolderBridgeLimit, SolderCriterion, SolderWindowInput, SolderWindowLimits,
        SolderWindowStatus, assess_solder_window,
    },
};
use sim_process::{Analysis, SourcedRange};
use sim_world::Provenance;

include!("fixtures/solder_window.rs");

#[test]
fn hot_end_keeps_upper_limits_open_and_preserves_failure() {
    use SolderCriterion::{MaxAboveLiquidus, MaxComponentTemperature};

    let mut input = bench(35.0, 1.45, 0.375);
    // Та же аналитическая мода, что и в стенде Т5. В конце оба узла
    // горячее ликвидуса, хотя наблюдаемый пик вывода ниже 520 К.
    let decay = (-1.45_f64).exp();
    let liquidus = input.limits.liquidus_k.as_ref().unwrap().min;
    let end_pad = 528.0 - 70.0 * decay;
    let end_lead = 528.0 - 105.0 * decay;
    assert!(end_pad > liquidus && end_lead > liquidus);
    assert!(end_lead < input.limits.max_component_k.as_ref().unwrap().min);
    let report = assess_solder_window(&input, steps(1024)).unwrap();
    let thermal = analyzed(report.thermal.clone());
    includes(thermal.peak_component_k, end_lead, 1e-8);
    for criterion in [MaxAboveLiquidus, MaxComponentTemperature] {
        let check = report.check(criterion);
        assert!(
            matches!(
                &check.status,
                SolderWindowStatus::NotAnalyzed { reason } if reason.contains("нет фазы остывания")
            ),
            "{check:?}"
        );
    }

    // Наблюдаемое время 1,45 − ln(70/(528 − 493,15)) уже больше 0,6 с.
    input.limits.max_above_liquidus_s = point(0.6);
    let report = assess_solder_window(&input, steps(1024)).unwrap();
    let check = report.check(MaxAboveLiquidus);
    assert_eq!(check.status, SolderWindowStatus::Fail);
    assert!(check.margin.unwrap().upper < 0.0);
}
