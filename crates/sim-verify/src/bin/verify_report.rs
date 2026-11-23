//! Отчёт по всем эталонным проверкам: `cargo run -p sim-verify --release --bin verify-report`.

fn main() {
    let checks = sim_verify::cases::run_all();
    let failed = checks.iter().filter(|c| !c.passed()).count();
    println!(
        "Эталонные проверки физики мастерской: {} шт.\n",
        checks.len()
    );
    for c in &checks {
        println!("{c}");
    }
    println!(
        "Итого: пройдено {} из {}.",
        checks.len() - failed,
        checks.len()
    );
    if failed > 0 {
        std::process::exit(1);
    }
}
