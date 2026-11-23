//! Отчёт по всем эталонным проверкам мастерской: `cargo verify`.

fn main() {
    let groups = sim_scenarios::checks::all_reference_checks();
    let (mut total, mut failed) = (0, 0);
    for g in &groups {
        let bad = g.checks.iter().filter(|c| !c.passed()).count();
        println!(
            "\n=== {} — {} из {} ===\n",
            g.name,
            g.checks.len() - bad,
            g.checks.len()
        );
        for c in &g.checks {
            println!("{c}");
        }
        total += g.checks.len();
        failed += bad;
    }
    let known = sim_scenarios::checks::known_discrepancies();
    println!("\n=== Известные расхождения модели с измерениями людей: {} ===", known.len());
    println!("Не засчитаны как пройденные. Вердикт опирается на сами измерения.\n");
    for c in &known {
        println!("{c}");
    }
    println!("\nИтого: пройдено {} из {}.", total - failed, total);
    if failed > 0 {
        std::process::exit(1);
    }
}
