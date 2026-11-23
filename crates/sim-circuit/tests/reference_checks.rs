//! Все сверки блока с эталоном должны проходить; при провале печатается сама проверка.

use sim_circuit::checks;
use sim_verify::Check;

fn assert_all(name: &str, checks: &[Check]) {
    let mut failed = Vec::new();
    for c in checks {
        println!("{c}");
        if !c.passed() {
            failed.push(c.to_string());
        }
    }
    assert!(
        failed.is_empty(),
        "{name}: не пройдено {} из {}:\n{}",
        failed.len(),
        checks.len(),
        failed.join("\n")
    );
}

#[test]
fn circuit_solver_matches_analytic_solutions() {
    assert_all("схемы", &checks::circuit::all());
}

#[test]
fn battery_model_matches_analytic_and_published_data() {
    assert_all("аккумулятор", &checks::battery::all());
}

#[test]
fn pcb_calculations_match_published_values() {
    assert_all("плата", &checks::pcb::all());
}

#[test]
fn ngspice_cross_check() {
    match sim_circuit::ngspice::find_ngspice() {
        None => println!("ngspice не найден — сверка пропущена"),
        Some(ng) => {
            println!("ngspice: {}", ng.display());
            assert_all("ngspice", &checks::ngspice_cmp::all(&ng));
        }
    }
}

#[test]
fn reference_check_ids_are_unique_and_namespaced() {
    let all = sim_circuit::reference_checks();
    let mut ids: Vec<&str> = all.iter().map(|c| c.id).collect();
    let n = ids.len();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), n, "повторяющиеся id сверок");
    for id in ids {
        assert!(
            ["circuit.", "battery.", "pcb.", "ngspice."]
                .iter()
                .any(|p| id.starts_with(p)),
            "id без префикса блока: {id}"
        );
    }
    assert!(n >= 40, "сверок слишком мало: {n}");
}
