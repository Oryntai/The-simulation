use std::collections::BTreeMap;

use sim_circuit::battery::{Cell, CellParams, Table};
use sim_process::schematic::mcu_energy::*;
use sim_process::schematic::sleep_drain::*;
use sim_process::schematic::{Mode, Scope, Section, Status};
use sim_process::{Analysis, ProcessError, SourcedRange};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «блок»: два цикла [8 с сна, 2 с CPU, 1 с TX, 1 с RX] и 2 с сна.
// Интеграл на выводах МК: 18·5 мкА + 4·22 мА + 2·335 мА + 2·84 мА = 0,92609 Кл.
// Независимые токи со стороны батареи: 10/100/300/80 мА МК, плюс 20 или 10 мА утечки.
// При Qёмк = 10 Кл, Uoc = 3 + SOC, R0 = 0,5 Ом и без RC или саморазряда
// к концу второго TX (23 с) ушло 1,70 / 1,47 Кл. U = 4 − Q/10 − I·0,5
// даёт 3,670 / 3,698 В. Относительно 3,680 В запас −10 / +18 мВ.
// За 26 с расход 1,86 / 1,60 Кл, зарядовый запас 8,14 / 8,40 Кл.
const COULOMBS_PER_AH: f64 = 3600.0; // Кулонов в А·ч.
const NUMERICAL_TOL: f64 = 1e-9; // Допуск десятичных вычислений стенда, меньше сдвига порога.
const CATALOG: &str = include_str!("../../../catalog/boards/electrical/esp32-c3-power.toml");
const STATES: [&str; 4] = [
    "deep_sleep",
    "modem_sleep_80_mhz_running_clocks_on",
    "wifi_tx_b_1_mbps_21_dbm",
    "wifi_rx_bgn_ht20",
];

fn seconds(s: f64) -> Time {
    Time::from_secs(s).unwrap()
}

// Тестовый читатель простых строк этого файла данных. Штатный загрузчик TOML относится к Т2.22.
fn fields(block: &str) -> BTreeMap<&str, &str> {
    let mut fields = BTreeMap::new();
    for line in block.lines().map(str::trim) {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (key, value) = line.split_once(" = ").unwrap();
        assert!(fields.insert(key, value.trim_matches('"')).is_none());
    }
    fields
}

fn catalog() -> McuPowerProfile {
    let mut blocks = CATALOG.split("\n[[states]]\n");
    let header = fields(blocks.next().unwrap());
    assert_eq!(header["format"], "1");
    assert_eq!(header.len(), 3);
    let rows = blocks.map(|block| {
        let f = fields(block);
        assert_eq!(f.len(), 6);
        let kind = match f["kind"] {
            "typical" => McuCurrentKind::Typical,
            "peak" => McuCurrentKind::Peak,
            kind => panic!("неизвестный вид тока: {kind}"),
        };
        (
            f["id"].into(),
            McuCurrent {
                current_a: f["current_a"].parse().unwrap(),
                kind,
                source: Provenance::Datasheet(format!(
                    "{}, §{}, табл. {}",
                    header["datasheet"], f["section"], f["table"]
                )),
                conditions: f["conditions"].into(),
            },
        )
    });
    McuPowerProfile::new(header["board"].into(), rows).unwrap()
}

fn analyzed<T>(result: &Analysis<T>) -> &T {
    match result {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("расчёт открыт: {reason}"),
    }
}

fn near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 2.0 * NUMERICAL_TOL,
        "{actual} вместо {expected}"
    );
}

#[test]
fn all_passport_rows_match_the_original_units_and_conditions() {
    let power = catalog();
    assert_eq!(power.board(), "esp32-c3");
    let count = CATALOG.lines().filter(|line| *line == "[[states]]").count();
    // §5.6.2, табл. 5-8 в мА и табл. 5-9 в мкА; §5.6.1, табл. 5-7 в мА.
    // Сверка переноса в СИ в обратную сторону, включая различия TX/RX и HT20/HT40.
    let groups: [(&str, McuCurrentKind, f64, &[(&str, f64)]); 3] = [
        (
            "§5.6.2, табл. 5-8",
            McuCurrentKind::Typical,
            1000.0,
            &[
                ("modem_sleep_160_mhz_running_clocks_off", 23.0),
                ("modem_sleep_160_mhz_running_clocks_on", 28.0),
                ("modem_sleep_160_mhz_idle_clocks_off", 16.0),
                ("modem_sleep_160_mhz_idle_clocks_on", 21.0),
                ("modem_sleep_80_mhz_running_clocks_off", 17.0),
                ("modem_sleep_80_mhz_running_clocks_on", 22.0),
                ("modem_sleep_80_mhz_idle_clocks_off", 13.0),
                ("modem_sleep_80_mhz_idle_clocks_on", 18.0),
            ],
        ),
        (
            "§5.6.2, табл. 5-9",
            McuCurrentKind::Typical,
            1_000_000.0,
            &[
                ("light_sleep", 130.0),
                ("deep_sleep", 5.0),
                ("power_off", 1.0),
            ],
        ),
        (
            "§5.6.1, табл. 5-7",
            McuCurrentKind::Peak,
            1000.0,
            &[
                ("wifi_tx_b_1_mbps_21_dbm", 335.0),
                ("wifi_tx_g_54_mbps_19_dbm", 285.0),
                ("wifi_tx_n_ht20_mcs7_18_5_dbm", 276.0),
                ("wifi_tx_n_ht40_mcs7_18_5_dbm", 278.0),
                ("wifi_rx_bgn_ht20", 84.0),
                ("wifi_rx_n_ht40", 87.0),
            ],
        ),
    ];
    assert_eq!(count, groups.iter().map(|group| group.3.len()).sum());
    for (citation, kind, scale, values) in groups {
        for &(id, expected) in values {
            let point = *analyzed(&power.current(id));
            near(point.current_a * scale, expected);
            assert_eq!(point.kind, kind);
            assert!(matches!(&point.source, Provenance::Datasheet(text)
                if text.contains("v2.5") && text.ends_with(citation)));
        }
    }
    for id in ["active_40_mhz", "wifi_tx", "ble_rx", "wake_transition"] {
        assert!(matches!(power.current(id), Analysis::NotAnalyzed { .. }));
    }
}

fn exact(value: f64, source: &Provenance) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source.clone(),
    }
}

struct Bench {
    source: Provenance,
    params: CellParams,
    branch: Section,
    leakage_branch: Section,
    controller: [SourcedRange; 4],
    leakage: SourcedRange,
    capacity: SourcedRange,
    soc: SourcedRange,
    temp: SourcedRange,
    self_discharge: SourcedRange,
    cutoff: SourcedRange,
    voltage_error: SourcedRange,
}

impl Bench {
    fn new(leakage: f64, cutoff: f64, source: Provenance) -> Self {
        // Электрическая модель стенда задаётся уравнениями выше. Прочие поля
        // структуры CellParams унаследованы из существующего конструктора.
        let mut params = CellParams::lg_m50();
        params.capacity_ah = 10.0 / COULOMBS_PER_AH;
        params.ocv = Table(vec![(0.0, 3.0), (1.0, 4.0)]);
        params.entropic = Table(vec![(0.0, 0.0), (1.0, 0.0)]);
        params.r0 = Table(vec![(0.0, 0.5), (1.0, 0.5)]);
        params.rc.clear();
        params.capacity_factor = Table(vec![(25.0, 1.0)]);
        params.v_min = cutoff;
        let range = |value| exact(value, &source);
        Self {
            branch: Section::Connection("питание блока".into()),
            leakage_branch: Section::Connection("утечка блока".into()),
            controller: [0.01, 0.10, 0.30, 0.08].map(range),
            leakage: range(leakage),
            capacity: range(params.capacity_ah),
            soc: range(1.0),
            temp: range(25.0),
            self_discharge: range(0.0),
            cutoff: range(cutoff),
            voltage_error: range(NUMERICAL_TOL),
            params,
            source,
        }
    }

    fn reference(leakage: f64, cutoff: f64) -> Self {
        Self::new(
            leakage,
            cutoff,
            Provenance::Derived("Уравнения стенда блока".into()),
        )
    }

    fn loads(&self) -> [SleepDrainLoad<'_>; 1] {
        [SleepDrainLoad {
            branch: &self.leakage_branch,
            battery_current_a: Some(&self.leakage),
        }]
    }

    fn states<'a>(&'a self, loads: &'a [SleepDrainLoad<'a>]) -> [SleepDrainState<'a>; 4] {
        std::array::from_fn(|index| SleepDrainState {
            id: STATES[index],
            duration: seconds([8.0, 2.0, 1.0, 1.0][index]),
            controller_a: Some(&self.controller[index]),
            loads: Some(loads),
        })
    }

    fn input<'a>(&'a self, states: &'a [SleepDrainState<'a>]) -> SleepDrainInput<'a> {
        SleepDrainInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.branch,
            required: Some(SleepDrainRequirement {
                duration: seconds(26.0),
                source: &self.source,
            }),
            profile: Some(SleepDrainProfile {
                states,
                repeat: true,
                source: &self.source,
            }),
            battery: Some(SleepDrainBattery {
                params: &self.params,
                applicability: &self.source,
                capacity_ah: Some(&self.capacity),
                initial_soc: Some(&self.soc),
                initial_polarization_v: Some(&[]),
                temperature_c: Some(&self.temp),
                self_discharge_a: Some(&self.self_discharge),
                cutoff_v: Some(&self.cutoff),
                voltage_error_v: Some(&self.voltage_error),
            }),
        }
    }
}

#[test]
fn leakage_pair_and_cutoff_threshold_match_charge_balance_and_cell() {
    let power = catalog();
    // Сдвиг порога на 10 мкВ в обе стороны от 3,670 В превосходит допуск стенда.
    for (leakage, cutoff, charge_c, voltage_v, status) in [
        (0.020, 3.680, 1.86, 3.670, Status::Fail),
        (0.010, 3.680, 1.60, 3.698, Status::Pass),
        (0.020, 3.670_01, 1.86, 3.670, Status::Fail),
        (0.020, 3.669_99, 1.86, 3.670, Status::Pass),
        (0.020, 3.670, 1.86, 3.670, Status::NotAnalyzed),
    ] {
        let bench = Bench::reference(leakage, cutoff);
        let loads = bench.loads();
        let states = bench.states(&loads);
        let input = bench.input(&states);
        let result = assess_mcu_energy(Some(&power), &input).unwrap();
        let chip = analyzed(&result.chip_charge);
        assert_eq!(chip.window.end, seconds(26.0));
        assert!(chip.includes_peak_held);
        near(chip.charge_c, 0.92609);
        // Второй способ интегрирования для этой постановки: развёрнутые циклы на сетке 1 с.
        let mut cell = Cell::new(bench.params.clone());
        let mut min_v = f64::INFINITY;
        let mut total_chip_c = 0.0;
        for second in 0..26 {
            let (chip_a, terminal_a) = match second % 12 {
                0..=7 => (0.000005, 0.01),
                8..=9 => (0.022, 0.10),
                10 => (0.335, 0.30),
                _ => (0.084, 0.08),
            };
            total_chip_c += chip_a;
            cell.step(terminal_a + leakage, 1.0, 25.0);
            min_v = min_v.min(cell.terminal_voltage());
        }
        near(chip.charge_c, total_chip_c);
        near(min_v, voltage_v);
        assert_eq!(result.sleep_drain, assess_sleep_drain(&input).unwrap());
        assert_eq!(result.sleep_drain.status, status, "{result:?}");
        let evidence = &result.sleep_drain.evidence;
        assert_eq!(evidence.code, "sleep_drain");
        let total = analyzed(&evidence.consumption).total_ah;
        let charge_margin = analyzed(&evidence.charge_margin_ah);
        let cutoff_margin = analyzed(&evidence.cutoff_margin_v);
        for (value, expected) in [
            (total.min * COULOMBS_PER_AH, charge_c),
            (total.max * COULOMBS_PER_AH, charge_c),
            (charge_margin.min * COULOMBS_PER_AH, 10.0 - charge_c),
            (charge_margin.max * COULOMBS_PER_AH, 10.0 - charge_c),
            (cutoff_margin.min, min_v - cutoff),
            (cutoff_margin.max, min_v - cutoff),
        ] {
            near(value, expected);
        }
        match (&evidence.cutoff_violation, status) {
            (Some(witness), Status::Fail) => {
                assert_eq!(witness.state, STATES[2]);
                assert_eq!(witness.time, seconds(23.0));
            }
            (None, Status::Pass | Status::NotAnalyzed) => {}
            pair => panic!("неожиданное свидетельство отсечки: {pair:?}"),
        }
    }
}

#[test]
fn only_a_complete_profile_and_sourced_battery_can_close_sleep_drain() {
    let power = catalog();
    let bench = Bench::reference(0.010, 3.680);
    let loads = bench.loads();
    let mut states = bench.states(&loads);
    for state in &mut states {
        state.controller_a = None;
    }
    let result = assess_mcu_energy(Some(&power), &bench.input(&states)).unwrap();
    near(analyzed(&result.chip_charge).charge_c, 0.92609);
    assert_eq!(result.sleep_drain.status, Status::NotAnalyzed);
    assert!(matches!(
        result.sleep_drain.evidence.consumption,
        Analysis::NotAnalyzed { .. }
    ));

    let states = bench.states(&loads);
    let mut input = bench.input(&states);
    let estimate = exact(3.680, &Provenance::Estimate("Порог будет замерен".into()));
    input.battery.as_mut().unwrap().cutoff_v = Some(&estimate);
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    assert_eq!(result.sleep_drain.status, Status::NotAnalyzed);
    assert!(matches!(
        result.sleep_drain.evidence.cutoff_margin_v,
        Analysis::NotAnalyzed { .. }
    ));

    input.battery = None;
    input.profile.as_mut().unwrap().repeat = false;
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    assert_eq!(result.sleep_drain.status, Status::NotAnalyzed);
    assert!(matches!(result.chip_charge, Analysis::NotAnalyzed { .. }));
    input.required = None;
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    near(analyzed(&result.chip_charge).charge_c, 0.46304); // 8·5 мкА + 2·22 + 335 + 84 мКл.

    let mut states = states;
    states[1].id = "ble_rx";
    let mut input = bench.input(&states);
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    assert!(
        matches!(result.chip_charge, Analysis::NotAnalyzed { ref reason } if reason.contains("ble_rx"))
    );
    // Неизвестное состояние за правой границей не участвует в интеграле.
    input.required.as_mut().unwrap().duration = seconds(8.0);
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    let chip = analyzed(&result.chip_charge);
    near(chip.charge_c, 0.000040); // 8 с · 5 мкА.
    assert!(!chip.includes_peak_held);
    assert!(matches!(
        assess_mcu_energy(None, &input).unwrap().chip_charge,
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn invalid_points_and_battery_inputs_are_errors() {
    let power = catalog();
    let point = (*analyzed(&power.current(STATES[0]))).clone();
    let make = |rows| McuPowerProfile::new("блок".into(), rows);
    assert!(make(vec![(STATES[0].into(), point.clone()); 2]).is_err());
    let bench = Bench::reference(0.010, 3.680);
    let loads = bench.loads();
    let states = bench.states(&loads);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -point.current_a] {
        assert!(matches!(
            make(vec![(
                STATES[0].into(),
                McuCurrent {
                    current_a: invalid,
                    ..point.clone()
                }
            )]),
            Err(ProcessError::InvalidInput(_))
        ));
        let cutoff = exact(invalid, &bench.source);
        let mut input = bench.input(&states);
        input.battery.as_mut().unwrap().cutoff_v = Some(&cutoff);
        input.required = None;
        assert!(matches!(
            assess_mcu_energy(None, &input),
            Err(ProcessError::InvalidInput(_))
        ));
    }
    let mut states = states;
    states[0].duration = Time(u64::MAX);
    let mut input = bench.input(&states);
    input.required = None;
    assert!(matches!(
        assess_mcu_energy(Some(&power), &input),
        Err(ProcessError::InvalidInput(_))
    ));
}

#[test]
fn assembly_requires_a_measured_load_profile_for_a_positive_result() {
    let power = catalog();
    // Источники здесь синтетические и проверяют ветвление по виду записи.
    let bench = Bench::new(
        0.010,
        3.680,
        Provenance::Measured("Тестовая запись стенда".into()),
    );
    let loads = bench.loads();
    let states = bench.states(&loads);
    let mut input = bench.input(&states);
    input.scope = Scope::Assembly;
    assert_eq!(
        assess_mcu_energy(Some(&power), &input)
            .unwrap()
            .sleep_drain
            .status,
        Status::Pass
    );

    let paper = Provenance::Paper("Тестовая запись расчётного режима".into());
    input.profile.as_mut().unwrap().source = &paper;
    let result = assess_mcu_energy(Some(&power), &input).unwrap();
    assert_eq!(result.sleep_drain, assess_sleep_drain(&input).unwrap());
    assert_eq!(result.sleep_drain.status, Status::NotAnalyzed);
    assert!(result.sleep_drain.reason.contains("замер"));
}
