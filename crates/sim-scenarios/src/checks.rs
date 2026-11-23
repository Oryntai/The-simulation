//! Все эталонные сверки мастерской по блокам: механика, прочность, радио, схемы,
//! электромагнетизм, среда, микроконтроллер. Окно показывает их сводкой, `cargo verify` —
//! полным отчётом.

use sim_verify::Check;

pub struct CheckGroup {
    pub name: &'static str,
    pub checks: Vec<Check>,
}

/// Где модель расходится с измерениями людей. Эти числа не считаются пройденными и не
/// участвуют в «прошёл»: вердикт для них опирается на сами измерения.
pub fn known_discrepancies() -> Vec<Check> {
    sim_human::known_discrepancies()
}

pub fn all_reference_checks() -> Vec<CheckGroup> {
    vec![
        CheckGroup {
            name: "Механика и прочность (наше ядро)",
            checks: sim_verify::cases::run_all(),
        },
        CheckGroup {
            name: "Радио: Wi-Fi и Bluetooth",
            checks: sim_radio::reference_checks(),
        },
        CheckGroup {
            name: "Среда и человек: воздух, вода, ветер, холод, жара",
            checks: sim_env::reference_checks(),
        },
        CheckGroup {
            name: "Схемы, батарея, платы (в т. ч. против ngspice)",
            checks: sim_circuit::reference_checks(),
        },
        CheckGroup {
            name: "Микроконтроллер и прошивка (RISC-V в контуре)",
            checks: sim_mcu::reference_checks(),
        },
        CheckGroup {
            name: "Электромагнетизм: поля, магниты, электромагнит, мотор",
            checks: sim_em::reference_checks(),
        },
        CheckGroup {
            name: "Материалы, крепёж и печать",
            checks: sim_materials::reference_checks(),
        },
        CheckGroup {
            name: "Безопасность батареи",
            checks: sim_battery::reference_checks(),
        },
        CheckGroup {
            name: "Приводы и питание",
            checks: sim_actuators::reference_checks(),
        },
        CheckGroup {
            name: "Платы и соединения",
            checks: sim_boards::reference_checks(),
        },
        CheckGroup {
            name: "Мышцы, нагрузка и отказ привода",
            checks: sim_human::reference_checks(),
        },
    ]
}
