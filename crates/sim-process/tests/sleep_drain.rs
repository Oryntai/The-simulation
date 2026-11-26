use sim_circuit::battery::{Cell, CellParams, RcParams, Table, discharge_to_cutoff};
use sim_process::schematic::sleep_drain::*;
use sim_process::schematic::{Mode, Scope, Section, Status};
use sim_process::{Analysis, ProcessError, SourcedRange};
use sim_signal::Time;
use sim_world::Provenance;

// «Блок»: 10 циклов по 90 с сна и 10 с пробуждения, требование 1000 с.
// Ток МК 0,005 / 0,105 А. Постоянные токи регулятора, делителя, подтяжки
// и модуля: 0,010 + 0,005 + 0,005 + (0,050 / 0,005) А.
// Поэтому на клеммах 0,075 / 0,175 А с утечкой и 0,030 / 0,130 А
// после исправления. Саморазряд 0,002 А. Суммы I·Δt за 1000 с:
// МК = 15 Кл, прочие = 70 / 25 Кл, саморазряд = 2 Кл. Итого 87 / 42 Кл.
// При Q = 100 Кл и 25 °C зарядовый запас +13 / +58 Кл. Для OCV = 3 + SOC,
// R0 = 0,5 Ом последний импульс даёт 3,0425 / 3,515 В и запас к 3,2 В
// −0,1575 / +0,315 В. С отсечкой 3,2 В на токе импульса заряд до отсечки
// 100·(3 + 1 − 3,2 − I·0,5) = 71,25 / 73,5 Кл.
const SECOND: u64 = 1_000_000_000; // Наносекунд в секунде.
const COULOMBS_PER_AH: f64 = 3600.0; // Кулонов в А·ч.
const TOL: f64 = 1e-8; // Допуск вычислений и десятичных чисел этого стенда.

fn seconds(s: u64) -> Time {
    Time(s * SECOND)
}

fn source() -> Provenance {
    Provenance::Derived("Аналитические параметры и полная схема стенда блока".into())
}

fn exact(x: f64) -> SourcedRange {
    SourcedRange {
        min: x,
        nominal: x,
        max: x,
        provenance: source(),
    }
}

fn element(id: &str) -> Section {
    Section::Element {
        component: "блок".into(),
        element: id.into(),
    }
}

struct Bench {
    source: Provenance,
    params: CellParams,
    battery: Section,
    load_branches: [Section; 4],
    load_currents: [SourcedRange; 4],
    controller_currents: [SourcedRange; 2],
    capacity: SourcedRange,
    soc: SourcedRange,
    temperature: SourcedRange,
    self_discharge: SourcedRange,
    cutoff: SourcedRange,
    voltage_error: SourcedRange,
}

impl Bench {
    fn new(module_a: f64) -> Self {
        // Паспортные поля метаданных остаются из имеющегося типа. Электрические
        // параметры стенда задаются формулами выше и не описывают серийную ячейку.
        let mut params = CellParams::lg_m50();
        params.capacity_ah = 100.0 / COULOMBS_PER_AH;
        params.v_min = 3.2;
        params.ocv = Table(vec![(0.0, 3.0), (1.0, 4.0)]);
        params.entropic = Table(vec![(0.0, 0.0), (1.0, 0.0)]);
        params.r0 = Table(vec![(0.0, 0.5), (1.0, 0.5)]);
        params.rc.clear();
        params.capacity_factor = Table(vec![(25.0, 1.0)]);
        Self {
            source: source(),
            battery: element("питание"),
            load_branches: ["регулятор", "делитель", "подтяжка", "модуль"].map(element),
            load_currents: [0.010, 0.005, 0.005, module_a].map(exact),
            controller_currents: [0.005, 0.105].map(exact),
            capacity: exact(params.capacity_ah),
            soc: exact(1.0),
            temperature: exact(25.0),
            self_discharge: exact(0.002),
            cutoff: exact(params.v_min),
            voltage_error: exact(TOL),
            params,
        }
    }

    fn loads(&self) -> [SleepDrainLoad<'_>; 4] {
        std::array::from_fn(|i| SleepDrainLoad {
            branch: &self.load_branches[i],
            battery_current_a: Some(&self.load_currents[i]),
        })
    }

    fn states<'a>(&'a self, loads: &'a [SleepDrainLoad<'a>]) -> [SleepDrainState<'a>; 2] {
        std::array::from_fn(|i| SleepDrainState {
            id: ["сон", "пробуждение"][i],
            duration: seconds([90, 10][i]),
            controller_a: Some(&self.controller_currents[i]),
            loads: Some(loads),
        })
    }

    fn input<'a>(&'a self, states: &'a [SleepDrainState<'a>]) -> SleepDrainInput<'a> {
        SleepDrainInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.battery,
            required: Some(SleepDrainRequirement {
                duration: seconds(1000),
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
                temperature_c: Some(&self.temperature),
                self_discharge_a: Some(&self.self_discharge),
                cutoff_v: Some(&self.cutoff),
                voltage_error_v: Some(&self.voltage_error),
            }),
        }
    }
}

fn analyzed<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("не получен результат: {reason}"),
    }
}

fn near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 2.0 * TOL,
        "{actual} вместо {expected}"
    );
}

#[test]
fn sleep_drain_leakage_pair_matches_charge_and_loaded_cutoff() {
    for (module_a, wake_a, other_q, q_cutoff, charge_margin, voltage_margin, expected) in [
        (0.050, 0.175, 70.0, 71.25, 13.0, -0.1575, Status::Fail),
        (0.005, 0.130, 25.0, 73.5, 58.0, 0.315, Status::Pass),
    ] {
        let bench = Bench::new(module_a);
        let loads = bench.loads();
        let states = bench.states(&loads);
        let result = assess_sleep_drain(&bench.input(&states)).unwrap();
        assert_eq!(result.status, expected, "{result:?}");
        assert_eq!(result.evidence.code, "sleep_drain");
        assert_eq!(result.evidence.branch, bench.battery);
        assert_eq!(result.evidence.window.unwrap().end, seconds(1000));
        let charge = analyzed(&result.evidence.consumption);
        near(charge.controller_ah.max * COULOMBS_PER_AH, 15.0);
        near(charge.other_loads_ah.max * COULOMBS_PER_AH, other_q);
        near(charge.self_discharge_ah.max * COULOMBS_PER_AH, 2.0);
        // Независимая сумма по каждому потребителю и состоянию, вручную по схеме.
        let independent_q =
            900.0 * 0.005 + 100.0 * 0.105 + 1000.0 * (0.010 + 0.005 + 0.005 + module_a + 0.002);
        near(charge.total_ah.min * COULOMBS_PER_AH, independent_q);
        near(charge.total_ah.max * COULOMBS_PER_AH, independent_q);
        let margin = analyzed(&result.evidence.charge_margin_ah);
        near(margin.min * COULOMBS_PER_AH, charge_margin);
        near(margin.max * COULOMBS_PER_AH, charge_margin);
        let margin = analyzed(&result.evidence.cutoff_margin_v);
        near(margin.min, voltage_margin);
        near(margin.max, voltage_margin);

        // Без RC граница по заряду при заданном токе не зависит от предыстории.
        // Решатель с постоянным током независимо даёт заряд отсечки для этой фазы.
        // Шаг 0,25 с выбран для сверки линейной модели; пересечение внутри шага.
        let (cutoff_ah, _) = discharge_to_cutoff(&bench.params, wake_a, 25.0, 0.25);
        near(cutoff_ah * COULOMBS_PER_AH, q_cutoff);
        assert_eq!(
            independent_q > cutoff_ah * COULOMBS_PER_AH,
            expected == Status::Fail
        );
        assert_eq!(
            result.evidence.cutoff_violation.is_some(),
            expected == Status::Fail
        );
    }
}

#[test]
fn sleep_drain_straddles_loaded_threshold() {
    // По линейной кривой выше минимум 3,515 В на левом пределе конца импульса.
    // Смещение 10 мкВ по обе стороны больше заявленной погрешности вычислений.
    for (cutoff, expected) in [
        (3.515_01, Status::Fail),
        (3.514_99, Status::Pass),
        (3.515, Status::NotAnalyzed),
    ] {
        let mut bench = Bench::new(0.005);
        bench.cutoff = exact(cutoff);
        let loads = bench.loads();
        let states = bench.states(&loads);
        let result = assess_sleep_drain(&bench.input(&states)).unwrap();
        assert_eq!(result.status, expected, "{result:?}");
        let margin = analyzed(&result.evidence.cutoff_margin_v);
        near(margin.min, 3.515 - cutoff);
        near(margin.max, 3.515 - cutoff);
    }
}

#[test]
fn sleep_drain_partial_wake_propagates_rc_history() {
    let mut bench = Bench::new(0.005);
    // R1 = 2 Ом, C1 = 5 Ф, τ = 10 с. 10 полных циклов, очередной сон
    // и половина пробуждения. Без саморазряда Q = 990·0,03 + 105·0,13 = 43,35 Кл.
    // Ожидаемый итог напряжения вычисляется вторым способом пошагово ниже.
    bench.params.rc = vec![RcParams {
        r: Table(vec![(0.0, 2.0), (1.0, 2.0)]),
        c: 5.0,
    }];
    bench.self_discharge = exact(0.0);
    // При пороге 3,35 В десятое полное пробуждение уже даёт отсечку.
    bench.cutoff = exact(3.35);
    let loads = bench.loads();
    let states = bench.states(&loads);
    let rest = [exact(0.0)];
    let mut input = bench.input(&states);
    input.required.as_mut().unwrap().duration = seconds(1095);
    input.battery.as_mut().unwrap().initial_polarization_v = Some(&rest);
    let result = assess_sleep_drain(&input).unwrap();
    assert_eq!(result.status, Status::Fail, "{result:?}");
    near(
        analyzed(&result.evidence.consumption).total_ah.max * COULOMBS_PER_AH,
        43.35,
    );

    let mut cell = Cell::new(bench.params.clone());
    let mut min_v = cell.ocv();
    for (current, dt) in [(0.03, 90.0), (0.13, 10.0)]
        .into_iter()
        .cycle()
        .take(20)
        .chain([(0.03, 90.0), (0.13, 5.0)])
    {
        min_v = min_v.min(cell.terminal_voltage_at(current));
        cell.step(current, dt, 25.0);
        min_v = min_v.min(cell.terminal_voltage());
    }
    assert!(min_v < bench.cutoff.min);
    let margin = analyzed(&result.evidence.cutoff_margin_v);
    near(margin.max, min_v - bench.cutoff.min);
    assert!(margin.min <= min_v - bench.cutoff.min);
    let witness = result.evidence.cutoff_violation.unwrap();
    assert_eq!(witness.time, seconds(1000));
    assert_eq!(witness.state, "пробуждение");
}

#[test]
fn sleep_drain_resolves_wake_peak_inside_one_period() {
    let mut bench = Bench::new(0.005);
    bench.cutoff = exact(3.3);
    let loads = bench.loads();
    let [sleep, wake] = bench.states(&loads);
    let peak = exact(1.5); // 1 нс между соседними отсчётами; R0 даёт падение >0,75 В.
    let states = [
        SleepDrainState {
            duration: seconds(5),
            ..sleep
        },
        SleepDrainState {
            duration: Time(1),
            controller_a: Some(&peak),
            ..wake
        },
        SleepDrainState {
            duration: seconds(5),
            ..sleep
        },
    ];
    let mut input = bench.input(&states);
    input.required.as_mut().unwrap().duration = seconds(10);
    input.profile.as_mut().unwrap().repeat = false;
    let result = assess_sleep_drain(&input).unwrap();
    assert_eq!(result.status, Status::Fail, "{result:?}");
    let witness = result.evidence.cutoff_violation.unwrap();
    assert_eq!(witness.time, seconds(5));
    near(
        witness.margin_v.max,
        4.0 - (5.0 * 0.032) / 100.0 - 1.525 * 0.5 - 3.3,
    );
    assert!(analyzed(&result.evidence.charge_margin_ah).min > 0.0);
}

#[test]
fn sleep_drain_applies_temperature_and_capacity_uncertainty() {
    let mut bench = Bench::new(0.005);
    // Холодный стенд: доступно 40% от 100 Кл, а приложенный профиль требует 42 Кл.
    bench.params.capacity_factor = Table(vec![(0.0, 0.4), (25.0, 1.0)]);
    bench.temperature = exact(0.0);
    let loads = bench.loads();
    let states = bench.states(&loads);
    let input = bench.input(&states);
    let result = assess_sleep_drain(&input).unwrap();
    assert_eq!(result.status, Status::Fail, "{result:?}");
    near(
        analyzed(&result.evidence.charge_margin_ah).max * COULOMBS_PER_AH,
        -2.0,
    );

    let temperature = SourcedRange {
        min: 0.0,
        nominal: 25.0,
        max: 25.0,
        provenance: source(),
    };
    let mut uncertain = input;
    uncertain.battery.as_mut().unwrap().temperature_c = Some(&temperature);
    let result = assess_sleep_drain(&uncertain).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed, "{result:?}");
    let margin = analyzed(&result.evidence.charge_margin_ah);
    near(margin.min * COULOMBS_PER_AH, -2.0);
    near(margin.max * COULOMBS_PER_AH, 58.0);
}

#[test]
fn sleep_drain_missing_or_estimated_evidence_stays_open() {
    let bench = Bench::new(0.005);
    let loads = bench.loads();
    let states = bench.states(&loads);
    let complete = bench.input(&states);
    let mut no_peak = states;
    no_peak[1].controller_a = None;
    let mut no_load = loads;
    no_load[0].battery_current_a = None;
    let unobserved_load = bench.states(&no_load);
    let mut no_inventory = states;
    no_inventory[0].loads = None;
    let estimate = Provenance::Estimate("Оценка без данных".into());
    let estimated_cutoff = SourcedRange {
        provenance: estimate.clone(),
        ..bench.cutoff.clone()
    };
    let model = complete.battery.unwrap();
    for input in [
        SleepDrainInput {
            required: None,
            ..complete
        },
        SleepDrainInput {
            profile: None,
            ..complete
        },
        SleepDrainInput {
            battery: None,
            ..complete
        },
        SleepDrainInput {
            scope: Scope::Assembly,
            ..complete
        },
        bench.input(&no_peak),
        bench.input(&unobserved_load),
        bench.input(&no_inventory),
        SleepDrainInput {
            profile: complete
                .profile
                .map(|p| SleepDrainProfile { repeat: false, ..p }),
            ..complete
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                cutoff_v: Some(&estimated_cutoff),
                ..model
            }),
            ..complete
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                applicability: &estimate,
                ..model
            }),
            ..complete
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                self_discharge_a: None,
                ..model
            }),
            ..complete
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                temperature_c: None,
                ..model
            }),
            ..complete
        },
    ] {
        let result = assess_sleep_drain(&input).unwrap();
        assert_eq!(result.status, Status::NotAnalyzed, "{result:?}");
        assert_eq!(result.evidence.code, "sleep_drain");
    }
}

#[test]
fn sleep_drain_rejects_nonfinite_even_with_missing_claim() {
    let bench = Bench::new(0.005);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut loads = bench.loads();
        let bad_range = exact(invalid);
        loads[0].battery_current_a = Some(&bad_range);
        let states = bench.states(&loads);
        let input = bench.input(&states);
        let no_claim = SleepDrainInput {
            required: None,
            ..input
        };
        assert!(matches!(
            assess_sleep_drain(&no_claim),
            Err(ProcessError::InvalidInput(_))
        ));

        let clean_loads = bench.loads();
        let clean_states = bench.states(&clean_loads);
        let mut input = bench.input(&clean_states);
        let mut params = bench.params.clone();
        params.ocv.0[0].1 = invalid;
        input.battery.as_mut().unwrap().params = &params;
        input.profile = None;
        assert!(matches!(
            assess_sleep_drain(&input),
            Err(ProcessError::InvalidInput(_))
        ));
    }
}
