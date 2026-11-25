//! Приёмка Э2 на точных синтетических цепях. Формулы ниже независимы от MNA и ngspice.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use serde::Deserialize;
use sim_cli::electrical::{SchematicInput, load_catalog, load_schematic};
use sim_process::{Analysis, Interval, SourcedRange, schematic::*, work::HazardCheck};
use sim_runtime::electrical::{PinRail, RailEvent};
use sim_task::{
    Outcome,
    verdict::{Finding, HazardClass, SimulationVerdict, VerdictReport},
};

#[path = "fixtures/electrical_verdict.rs"]
mod electrical_verdict;
use electrical_verdict::ElectricalStand;
use sim_signal::Time;
use sim_world::Provenance;

const CLASSES: [&str; 12] = [
    "assembly_mismatch",
    "missing_ground",
    "reverse_polarity",
    "pin_overvoltage",
    "ground_switch_backfeed",
    "source_conflict",
    "rail_sag",
    "startup_actuation",
    "inductive_spike",
    "switch_underdrive",
    "connection_overcurrent",
    "regulator_overload",
];

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    format: u32,
    pairs: BTreeMap<String, [PathBuf; 2]>,
}

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn corpus() -> PathBuf {
    root().join("devices/electrical/corpus")
}

fn manifest() -> Manifest {
    let m: Manifest = toml::from_str(&fs::read_to_string(corpus().join("manifest.toml")).unwrap())
        .expect("неверный манифест Э2");
    assert_eq!(m.format, 1, "версия манифеста");
    assert_eq!(
        m.pairs.keys().map(String::as_str).collect::<BTreeSet<_>>(),
        BTreeSet::from(CLASSES),
    );
    assert_eq!(
        m.pairs.values().flatten().collect::<BTreeSet<_>>().len(),
        2 * CLASSES.len()
    );
    for (class, pair) in &m.pairs {
        assert_eq!(
            *pair,
            ["unsafe.toml", "fixed.toml"].map(|file| Path::new(class).join(file))
        );
    }
    let mut paths = BTreeSet::new();
    for entry in fs::read_dir(corpus()).unwrap() {
        let dir = entry.unwrap().path();
        if dir.is_dir() && dir.file_name().unwrap() != "catalog" {
            paths.extend(fs::read_dir(dir).unwrap().map(|file| {
                file.unwrap()
                    .path()
                    .strip_prefix(corpus())
                    .unwrap()
                    .to_owned()
            }));
        }
    }
    assert_eq!(paths, m.pairs.values().flatten().cloned().collect());
    m
}

fn catalog() -> Catalog {
    let base = load_catalog(&root().join("catalog/electrical"))
        .unwrap()
        .catalog;
    let fixture = load_catalog(&corpus().join("catalog")).unwrap().catalog;
    let mut parts = base.parts().clone();
    for (id, part) in fixture.parts() {
        assert!(parts.insert(id.clone(), part.clone()).is_none(), "{id}");
    }
    Catalog::new(parts).unwrap()
}

fn load(class: &str, fixed: bool, catalog: &Catalog) -> SchematicInput {
    let file = if fixed { "fixed.toml" } else { "unsafe.toml" };
    load_schematic(&corpus().join(class).join(file), catalog).unwrap()
}

fn analyzed<T>(analysis: Analysis<T>) -> T {
    match analysis {
        Analysis::Analyzed { value, scope } => {
            assert!(!scope.trim().is_empty());
            value
        }
        Analysis::NotAnalyzed { reason } => panic!("нет результата: {reason}"),
    }
}

fn end(path: &str) -> Endpoint {
    let (component, pin) = path.split_once('/').unwrap();
    Endpoint {
        component: component.into(),
        pin: pin.into(),
    }
}

fn volts(a: &str, b: &str) -> Probe {
    Probe::Voltage(end(a), end(b))
}

fn link(id: &str) -> Section {
    Section::Connection(id.into())
}

fn branch(component: &str, element: &str) -> Section {
    Section::Element {
        component: component.into(),
        element: element.into(),
    }
}

fn point_limit(value: f64, source: &str) -> SourcedRange {
    // Паспортом искусственного стенда служит спецификация его входных данных.
    assert!(value.is_finite() && value > 0.0 && !source.trim().is_empty());
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: Provenance::Datasheet(source.into()),
    }
}

fn near(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        error <= tolerance,
        "{label}: MNA {actual:.12e}, вручную {expected:.12e}, Δ {error:.3e} > {tolerance:.3e}"
    );
    println!(
        "  {label} · MNA {actual:.9e}, вручную {expected:.9e}, Δ {error:.2e}, допуск {tolerance:.2e}"
    );
}

// Допуск 1 нВ/нА/нВт для линейных DC-формул с точными параметрами корпуса.
const LINEAR: f64 = 1e-9;

fn prepare(input: &SchematicInput, plan: &ModePlan, mode: &Mode) -> PreparedMode {
    analyzed(input.compiled.prepare_mode(plan, mode).unwrap())
}

fn series(prepared: &PreparedMode, case: &ModeCase) -> ElectricalSeries {
    let trace = analyzed(prepared.solve());
    let mut expected = BTreeSet::from([Time::ZERO]);
    if let Some(w) = &case.window {
        expected.extend(
            (0..w.end.0)
                .step_by(usize::try_from(w.step.0).unwrap())
                .map(Time),
        );
        expected.insert(w.end);
        expected.extend(
            case.sources
                .values()
                .flat_map(|s| s.points.iter().map(|(t, _)| *t)),
        );
    }
    assert_eq!(trace.time, expected.into_iter().collect::<Vec<_>>());
    for probe in &case.probes {
        assert!(trace.values.contains_key(probe), "нет наблюдения {probe:?}");
    }
    for values in trace.values.values() {
        assert_eq!(values.len(), trace.time.len());
        assert!(values.iter().all(|v| v.is_finite()));
    }
    trace
}

fn compare(
    prepared: &PreparedMode,
    trace: &ElectricalSeries,
    checks: BTreeMap<Probe, ReferenceCheck>,
) -> BTreeMap<Probe, ReferenceResult> {
    let spice = prepared.compare_ngspice(&checks).unwrap();
    assert_eq!(
        checks.keys().collect::<Vec<_>>(),
        spice.keys().collect::<Vec<_>>()
    );
    for (probe, result) in &spice {
        if let ReferenceCheck::Transient(w) = checks[probe] {
            assert!(result.rows >= trace.time.len() && result.settling_s.is_some());
            let peak = trace
                .time
                .iter()
                .zip(&trace.values[probe])
                .filter(|(time, _)| **time >= w.start && **time <= w.end)
                .max_by(|(_, a), (_, b)| a.total_cmp(b))
                .unwrap()
                .0;
            // В относительном сравнении нулевой эталон допускает только точный ноль.
            let relative_pct = |d: ReferenceDifference| {
                if d.reference == 0.0 {
                    assert_eq!(d.error, 0.0);
                    0.0
                } else {
                    100.0 * d.error / d.reference.abs()
                }
            };
            let dt = result.settling_s.unwrap();
            println!(
                "  ngspice {probe:?} · {} строк, пик MNA {} нс, Δmin {:.3e}, Δmax {:.3e} ({:.6}%), Δt {:.3e} с ({:.6}%)",
                result.rows,
                peak.0,
                result.min.error,
                result.max.error,
                relative_pct(result.max),
                dt.error,
                relative_pct(dt),
            );
        } else {
            assert_eq!(result.rows, 1);
            assert!(result.settling_s.is_none());
            assert_eq!(result.max.measured, trace.values[probe][0]);
            println!("  ngspice {probe:?} · DC t=0, Δ {:.3e} В", result.max.error);
        }
        assert!(result.matches(), "{probe:?}: {result:?}");
    }
    spice
}

fn dc(
    input: &SchematicInput,
    mode: Mode,
    extra: BTreeSet<Probe>,
) -> (PreparedMode, ElectricalSeries) {
    let mut plan = input.modes.clone();
    let case = plan.cases.entry(mode.clone()).or_insert_with(|| ModeCase {
        source: Provenance::Derived("Измеряемые DC-разности исходной пары Э1".into()),
        window: None,
        probes: BTreeSet::new(),
        sources: BTreeMap::new(),
        pins: BTreeMap::new(),
        sampling: None,
    });
    case.probes.extend(extra);
    // Для постоянных резистивных нагрузок отдельно сверяется рабочая точка.
    assert!(case.sources.is_empty());
    case.window = None;
    let checks = case
        .probes
        .iter()
        .filter(|p| matches!(p, Probe::Voltage(..)))
        .map(|p| (p.clone(), ReferenceCheck::DcVoltage))
        .collect::<BTreeMap<_, _>>();
    assert!(!checks.is_empty());
    println!("  режим {mode:?}");
    let prepared = prepare(input, &plan, &mode);
    let trace = series(&prepared, &plan.cases[&mode]);
    compare(&prepared, &trace, checks);
    (prepared, trace)
}

fn topology(input: &SchematicInput, class: StaticClass, fixed: bool) -> Option<f64> {
    let report = input.compiled.check_static();
    let probe;
    match class {
        StaticClass::AssemblyMismatch => {
            let connectors = report
                .assembly_mismatch
                .iter()
                .filter(|f| f.evidence.kind == JointKind::Plug)
                .collect::<Vec<_>>();
            let good = connectors
                .iter()
                .filter(|f| {
                    let [a, b] = &f.evidence.contacts;
                    [a.kind, b.kind].contains(&ContactKind::Plug)
                        && [a.kind, b.kind].contains(&ContactKind::Socket)
                        && a.family == b.family
                })
                .count();
            assert_eq!(
                (good, connectors.len()),
                if fixed { (2, 2) } else { (0, 1) }
            );
            assert!(
                connectors
                    .iter()
                    .all(|f| f.evidence.branch_connected == fixed)
            );
            probe = volts("load/input", "load/return");
            let (_, trace) = dc(input, Mode::Normal, BTreeSet::from([probe.clone()]));
            // Э1: 5 В, 1 Ом, 100 Ом, жила 0,2 Ом и три стыка по 0,1 Ом.
            let current = if fixed {
                5.0 / (1.0 + 100.0 + 0.2 + 3.0 * 0.1)
            } else {
                0.0
            };
            near(
                "5/(1+100+0,2+3·0,1) или разрыв, А",
                trace.values[&probe][0] / 100.0,
                current,
                LINEAR,
            );
            None
        }
        StaticClass::MissingGround => {
            let [finding] = &report.missing_ground[..] else {
                panic!("нет единственного пути возврата")
            };
            assert_eq!(
                finding.evidence.references,
                [end("sender/minus"), end("receiver/return")]
            );
            assert_eq!(finding.evidence.signal[0].section, link("signal"));
            if fixed {
                let path = finding.evidence.return_path.as_ref().unwrap();
                assert_eq!(path.len(), 1);
                assert_eq!(path[0].section, link("return"));
                assert_eq!(
                    [path[0].from.clone(), path[0].to.clone()],
                    finding.evidence.references
                );
            } else {
                assert!(finding.evidence.return_path.is_none());
            }
            probe = volts("receiver/input", "receiver/return");
            let (_, trace) = dc(
                input,
                Mode::Normal,
                BTreeSet::from([
                    probe.clone(),
                    volts("sender/plus", "sender/minus"),
                    Probe::Current(link("signal")),
                ]),
            );
            // Э1: 5 В, 1 Ом, вход 1 кОм и два стыка по 0,1 Ом.
            let current = if fixed {
                5.0 / (1.0 + 1000.0 + 2.0 * 0.1)
            } else {
                0.0
            };
            near(
                "5/(1+1000+2·0,1) или разрыв, А",
                trace.values[&Probe::Current(link("signal"))][0],
                current,
                LINEAR,
            );
            None
        }
        StaticClass::ReversePolarity => {
            let [finding] = &report.reverse_polarity[..] else {
                panic!("нет единственной маркированной пары")
            };
            assert_eq!(
                finding.evidence.ends,
                [end("capacitor/plus"), end("capacitor/minus")]
            );
            probe = volts("capacitor/plus", "capacitor/minus");
            let (_, trace) = dc(input, Mode::Normal, BTreeSet::from([probe.clone()]));
            // Э1: в DC ток идеального конденсатора равен нулю, знак задаёт ориентация.
            let expected = if fixed { 5.0 } else { -5.0 };
            near(
                "полярность V(+) − V(−), В",
                finding.evidence.voltage_v.unwrap(),
                expected,
                LINEAR,
            );
            near(
                "полярность V(+) − V(−), В",
                trace.values[&probe][0],
                expected,
                LINEAR,
            );
            Some(expected)
        }
    }
}

fn board() -> BoardContact {
    BoardContact {
        board: "fixture".into(),
        pin: "p0".into(),
    }
}

fn thermal_v() -> f64 {
    // Точные константы СИ-2019, T = 27 °C (TNOM обеих схем).
    1.380_649e-23 * 300.15 / 1.602_176_634e-19
}

fn bisect(mut lo: f64, mut hi: f64, increasing: impl Fn(f64) -> f64) -> f64 {
    assert!(increasing(lo) <= 0.0 && increasing(hi) >= 0.0);
    // 52 деления достаточно для мантиссы f64 на интервалах этих стендов.
    for _ in 0..52 {
        let mid = lo.midpoint(hi);
        if increasing(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo.midpoint(hi)
}

fn pin_overvoltage(input: &SchematicInput, fixed: bool) -> f64 {
    let (mode, trace) = dc(input, Mode::MainOnly, BTreeSet::new());
    let stamp = &mode.pins[&board()];
    let clamp = stamp.upper_clamp.clone().unwrap();
    let pad = &trace.values[&volts("board/io", "board/g")][0];
    let current = trace.values[&Probe::Current(clamp)][0];
    // Пара Э2: Is = 1 пА; верхний диод имеет анод на входе. KCL с явной 1 МОм утечкой.
    let diode = |v: f64| 1e-12 * ((v - 3.3) / thermal_v()).exp_m1();
    let from_source = if fixed { 1000.0 + 2.0 * 0.1 } else { 100.0 };
    let to_return = if fixed {
        1.0 / (1800.0 + 2.0 * 0.1)
    } else {
        0.0
    };
    let expected = bisect(0.0, 5.0, |v| {
        (v - 5.0) / (from_source + 0.1) + v * (1e-6 + to_return) + diode(v)
    });
    // 10 мкВ / 0,1 мкА покрывают критерий сходимости нелинейной рабочей точки.
    near("KCL входа с диодом Шокли, В", *pad, expected, 1e-5);
    near(
        "Is·expm1((Vpad − 3,3)/Vt), А",
        current,
        diode(expected),
        1e-7,
    );
    diode(expected).max(0.0)
}

fn source_conflict(input: &SchematicInput, fixed: bool) -> f64 {
    let (_, trace) = dc(input, Mode::Both, BTreeSet::new());
    let usb = branch("usb", "series");
    // Две ЭДС 5/4 В, два внутренних 1 Ом и два стыка 0,1 Ом.
    let expected = if fixed {
        0.0
    } else {
        -(5.0 - 4.0) / (2.0 + 2.0 * 0.1)
    };
    near(
        "−ΔV/ΣR обратного тока, А",
        trace.values[&Probe::Current(usb.clone())][0],
        expected,
        LINEAR,
    );
    near(
        "Uusb = 4 В − Iusb·1 Ом",
        trace.values[&volts("usb/p", "usb/g")][0],
        4.0 - expected,
        LINEAR,
    );
    expected.abs()
}

fn backfeed(input: &SchematicInput, fixed: bool) -> f64 {
    let (_, trace) = dc(input, Mode::LoadOff, BTreeSet::new());
    // 100 Ом возврата через сигнальную часть, по 0,1 Ом на два стыка.
    // Ключ выключен (1 МОм). В исправлении он перенесён в плюс и добавлен общий провод 0,1 Ом.
    let signal_path: f64 = 100.0 + 2.0 * 0.1;
    let (feed, direct): (f64, f64) = if fixed {
        (10.0 + 1e6, 0.1)
    } else {
        (10.0, 1e6)
    };
    let parallel = (direct.recip() + signal_path.recip()).recip();
    let local = 5.0 * parallel / (feed + parallel);
    near(
        "Vвозврата/ΣR сигнальной петли, А",
        trace.values[&Probe::Current(link("signal"))][0],
        local / signal_path,
        LINEAR,
    );
    near(
        "потенциал местного возврата, В",
        trace.values[&volts("block/ret", "block/g")][0],
        local,
        LINEAR,
    );
    local / signal_path
}

fn startup(input: &SchematicInput, fixed: bool) -> f64 {
    let (prepared, trace) = dc(input, Mode::Reset, BTreeSet::new());
    assert_eq!(prepared.pins[&board()].state, PinState::InputPull(Pull::Up));
    // 10 кОм вверх, 1 МОм вход; исправление добавляет 1 кОм и четыре стыка 0,1 Ом.
    let external: f64 = 1000.0 + 4.0 * 0.1;
    let to_ground = 1e-6 + if fixed { external.recip() } else { 0.0 };
    let pad = 3.3 / (1.0 + 10000.0 * to_ground);
    let gate = pad - if fixed { pad / external * 0.2 } else { 0.0 };
    let gate_probe = volts("load/gate", "load/g");
    near(
        "делитель подтяжек Vgate, В",
        trace.values[&gate_probe][0],
        gate,
        LINEAR,
    );
    // По паспорту эквивалента порог ключа 1,65 В, Rвкл = 1 Ом, Rвыкл = 1 МОм.
    let expected = 5.0 / (10.0 + if gate > 1.65 { 1.0 } else { 1e6 });
    let load = trace.values[&Probe::Current(branch("load", "load"))][0];
    near("5/(10+Rключа), А", load, expected, LINEAR);
    let boot = prepare(input, &input.modes, &Mode::Boot(Time::ZERO));
    assert_eq!(boot.pins[&board()].state, prepared.pins[&board()].state);
    near(
        "начало загрузки, А",
        analyzed(boot.solve()).values[&Probe::Current(branch("load", "load"))][0],
        load,
        LINEAR,
    );
    expected
}

fn transistor(input: &SchematicInput, fixed: bool) -> f64 {
    let (_, trace) = dc(input, Mode::Normal, BTreeSet::new());
    // Level 1 без модуляции длины, β = 1 А/В², Vt = 2 В, Rнагрузки = 5 Ом.
    // В триодной области: (5 − Vds)/5 = β·((Vgs − Vt)·Vds − Vds²/2).
    let vgs = if fixed { 5.0 } else { 3.3 };
    let a = vgs - 2.0 + 1.0 / 5.0;
    let vds = 1.0 / (a + f64::sqrt(a * a - 2.0 * 5.0 / 5.0)) * 2.0;
    assert!(vds > 0.0 && vds < vgs - 2.0);
    let amps = trace.values[&Probe::Current(branch("block", "channel"))][0];
    let measured_vds = trace.values[&volts("block/drain", "block/g")][0];
    near(
        "фактическое Vgs, В",
        trace.values[&volts("block/gate", "block/g")][0],
        vgs,
        LINEAR,
    );
    near(
        "корень квадратного уравнения канала Vds, В",
        measured_vds,
        vds,
        1e-6,
    );
    let expected_a = (5.0 - vds) / 5.0;
    near("ток канала из KVL, А", amps, expected_a, 1e-6);
    let rds = vds / expected_a;
    near(
        "I²·Rds(on) при фактическом Vgs, Вт",
        amps * measured_vds,
        expected_a.powi(2) * rds,
        1e-6,
    );
    expected_a.powi(2) * rds
}

fn joint_limit(input: &SchematicInput, id: &str) -> SourcedRange {
    input
        .compiled
        .assembly_sheet()
        .inspections
        .into_iter()
        .find_map(|i| match i.expected {
            Expected::CurrentLimitA(parameter) if i.subject == id => parameter.range,
            _ => None,
        })
        .expect("монтажный лист не связал предел с участком")
}

fn connection_current(input: &SchematicInput, fixed: bool) -> f64 {
    let (_, trace) = dc(input, Mode::MainOnly, BTreeSet::new());
    let feed = link("feed");
    // ЭДС 5 В, 1 Ом источника, 2 Ом нагрузки, 0,1 Ом возврата и 0,1/0,05 Ом стыка.
    let resistance = if fixed { 0.05 } else { 0.1 };
    let amps: f64 = 5.0 / (1.0 + 2.0 + 0.1 + resistance);
    near(
        "5/ΣR на стыке, А",
        trace.values[&Probe::Current(feed.clone())][0],
        amps,
        LINEAR,
    );
    near(
        "I²·R тепловыделение стыка, Вт",
        trace.values[&Probe::Power(feed.clone())][0],
        amps.powi(2) * resistance,
        LINEAR,
    );
    let limit = joint_limit(input, "feed");
    assert_eq!(limit.nominal, if fixed { 2.0 } else { 1.0 });
    amps
}

fn regulator(input: &SchematicInput, fixed: bool, catalog: &Catalog) -> f64 {
    dc(input, Mode::Normal, BTreeSet::new());
    let id = input
        .compiled
        .assembly_sheet()
        .parts
        .into_iter()
        .find(|part| part.component == "regulator")
        .unwrap()
        .part;
    let rating = &catalog.part(&id).unwrap().ratings["output_domain"];
    assert_eq!(rating.quantity, Quantity::CurrentA);
    assert_eq!(rating.nodes, ["p", "g"]);
    assert!(rating.conditions.contains("Vin 10 В") && rating.conditions.contains("25 °C"));
    let limit = rating.limit.range.as_ref().unwrap();
    let case = &input.modes.cases[&Mode::Normal];
    let mode = prepare(input, &input.modes, &Mode::Normal);
    let trace = series(&mode, case);
    let feed = link("feed");
    // Паспортные окна 4,8…5,2 В при 25 °C. В корпусе 3 Ом нагрузки и два стыка 0,1 Ом.
    let r = 3.0 + 2.0 * 0.1;
    let demand = 5.0 / r;
    for &a in &trace.values[&Probe::Current(feed.clone())] {
        assert!((a - demand).abs() <= LINEAR);
    }
    near(
        "5/(3+2·0,1) требуемый ток регулирования, А",
        trace.values[&Probe::Current(feed.clone())][0],
        demand,
        LINEAR,
    );
    assert_eq!(limit.nominal, if fixed { 3.0 } else { 1.0 });
    assert_eq!(fixed, 5.2 / r < limit.min);
    assert_eq!(!fixed, 4.8 / r > limit.max);
    demand
}

fn dynamic(
    input: &SchematicInput,
    mode: Mode,
    checks: BTreeMap<Probe, ComparisonWindow>,
) -> ElectricalSeries {
    let mut plan = input.modes.clone();
    let case = &plan.cases[&mode];
    for source in std::iter::once(&case.source).chain(case.sources.values().map(|s| &s.source)) {
        assert!(
            matches!(source, Provenance::Derived(description) if !description.trim().is_empty())
        );
    }
    let window = case.window.as_ref().unwrap();
    let report = input
        .compiled
        .run_modes(&plan)
        .unwrap()
        .into_iter()
        .find_map(|row| match row {
            ModeCoverage::Case(report) if report.mode == mode => Some(report),
            _ => None,
        })
        .expect("режим отсутствует в расписании");
    assert_eq!(report.window.as_ref(), Some(window));
    assert_eq!(report.source_fronts, case.sources);
    assert!(report.seed.is_none() && report.samples.is_empty());
    let prepared = prepare(input, &plan, &mode);
    let coarse = series(&prepared, case);
    assert_eq!(analyzed(report.nominal), coarse);
    let checks = checks
        .into_iter()
        .map(|(p, w)| (p, ReferenceCheck::Transient(w)))
        .collect::<BTreeMap<_, _>>();
    assert_eq!(
        checks.keys().collect::<BTreeSet<_>>(),
        case.probes.iter().collect()
    );
    let first = compare(&prepared, &coarse, checks.clone());

    let fine_case = plan.cases.get_mut(&mode).unwrap();
    let fine_window = fine_case.window.as_mut().unwrap();
    // Вложенная сетка с половинным шагом. Моменты PWL сохраняются.
    assert_eq!(fine_window.step.0 % 2, 0);
    fine_window.step.0 /= 2;
    let prepared = prepare(input, &plan, &mode);
    let fine = series(&prepared, &plan.cases[&mode]);
    let second = compare(&prepared, &fine, checks);
    for (probe, a) in &first {
        let b = &second[probe];
        for (x, y) in [
            (a.min, b.min),
            (a.max, b.max),
            (a.settling_s.unwrap(), b.settling_s.unwrap()),
        ] {
            // Архитектура §8.2: тот же критерий 5% для контроля сходимости.
            assert!(
                (x.measured - y.measured).abs() <= 0.05 * y.measured.abs(),
                "{probe:?}: разные сетки {x:?} / {y:?}"
            );
        }
    }
    fine
}

fn window(start_ns: u64, end_ns: u64, target: f64, band: f64) -> ComparisonWindow {
    ComparisonWindow {
        start: Time(start_ns),
        end: Time(end_ns),
        target,
        band,
    }
}

fn rail_sag(input: &SchematicInput, fixed: bool) -> f64 {
    let voltage = volts("block/p", "block/g");
    let demand = Probe::Current(branch("block", "demand"));
    dc(input, Mode::MainOnly, BTreeSet::new());
    // Из схемы: 5 В, 2/0,4 Ом, 10 мкФ, линейный пуск 0,1→1 А за 50 мкс.
    let r = if fixed { 0.4 } else { 2.0 };
    let target = 5.0 - r;
    let trace = dynamic(
        input,
        Mode::Start,
        BTreeMap::from([
            // Полоса установки ±10 мВ/10 мА фиксируется для импульса корпуса.
            (voltage.clone(), window(0, 800000, target, 0.010)),
            (demand.clone(), window(0, 800000, 1.0, 0.010)),
        ]),
    );
    let front = |ns| trace.time.binary_search(&Time(ns)).unwrap();
    near(
        "начальный и конечный ток фронта, А",
        trace.values[&demand][front(50000)],
        0.1,
        LINEAR,
    );
    near(
        "начальный и конечный ток фронта, А",
        trace.values[&demand][front(100000)],
        1.0,
        LINEAR,
    );
    let tau: f64 = r * 10e-6;
    let ramp: f64 = 50e-6;
    let at_end = 5.0 - r * (1.0 - 0.9 / ramp * tau * (1.0 - (-ramp / tau).exp()));
    // Решение dV/dt = [(5 − V)/R − I(t)]/C, затем экспонента на плато.
    // 0,5 мВ покрывает дискретизацию в точке излома; зазор до порога больше 0,5 В.
    near(
        "I·R и ΔQ/C к концу фронта, В",
        trace.values[&voltage][front(100000)],
        at_end,
        0.0005,
    );
    near(
        "5−IR на плато, В",
        *trace.values[&voltage].last().unwrap(),
        target,
        LINEAR,
    );
    assert!(
        trace.values[&voltage]
            .windows(2)
            .all(|v| v[1] <= v[0] + LINEAR)
    );

    let reset: toml::Value =
        toml::from_str(&fs::read_to_string(corpus().join("rail-reset.toml")).unwrap()).unwrap();
    let parameter = |field: &str| {
        let data = &reset["reset"][field];
        let range = &data["range"];
        let value = range["nominal"].as_float().unwrap();
        assert_eq!(range["min"].as_float(), Some(value));
        assert_eq!(range["max"].as_float(), Some(value));
        Parameter {
            range: Some(point_limit(
                value,
                range["source"]["datasheet"].as_str().unwrap(),
            )),
            measurement: data["measurement"].as_str().unwrap().into(),
        }
    };
    let profile = ResetProfile {
        falling_v: parameter("falling_v"),
        rising_v: parameter("rising_v"),
        assert_delay_s: parameter("assert_delay_s"),
        release_delay_s: parameter("release_delay_s"),
    };
    let threshold = profile.falling_v.range.as_ref().unwrap().nominal;
    let delay = Time::from_secs(profile.assert_delay_s.range.as_ref().unwrap().nominal).unwrap();
    let mut rail = PinRail::new(profile, false).unwrap();
    // На плато аналитическая RC-кривая монотонна. Края плюс погрешность
    // дискретизации ограничивают всё окно, включая время между наблюдениями.
    let start = Time(400000);
    let end = Time(800000);
    let lo = trace.values[&voltage][front(end.0)] - 0.0005;
    let hi = trace.values[&voltage][front(start.0)] + 0.0005;
    let event = analyzed(rail.advance(Interval { start, end }, [lo, hi]).unwrap());
    if fixed {
        assert_eq!(event.event, None);
        assert!(!event.in_reset);
        // Минимум на всём монотонном пуске выше порога.
        assert!(lo > threshold);
    } else {
        assert!(event.in_reset && hi < threshold);
        assert_eq!(
            event.event,
            Some(RailEvent::Asserted(Time(start.0 + delay.0)))
        );
    }
    println!(
        "  шина [{lo:.6}, {hi:.6}] В, порог {} В, выдержка {} нс, {:?}",
        threshold, delay.0, event.event
    );
    target
}

fn inductive_spike(input: &SchematicInput, fixed: bool) -> f64 {
    let voltage = volts("block/g", "block/p");
    let coil = Probe::Current(branch("block", "coil"));
    let (_, dc_trace) = dc(input, Mode::Normal, BTreeSet::new());
    near(
        "установившийся ток L до отключения, А",
        dc_trace.values[&coil][0],
        1.0,
        LINEAR,
    );
    let trace = dynamic(
        input,
        Mode::LoadOff,
        BTreeMap::from([
            // Полосы корпуса вокруг нуля: 1 В для хвоста RL; 0,532 В для спада через диод.
            (
                voltage.clone(),
                window(21000, 400000, 0.0, if fixed { 0.532 } else { 1.0 }),
            ),
            // Полоса 0,95 А фиксирует начало затухания тока в обеих постановках.
            (coil.clone(), window(21000, 400000, 0.0, 0.95)),
        ]),
    );
    let at = trace.time.binary_search(&Time(21000)).unwrap();
    let peak = trace.envelopes()[&voltage].max.clone();
    assert_eq!(
        peak.time,
        Time(21000),
        "максимум на конце фронта отключения"
    );
    let current = trace.values[&coil][at];
    assert!(current > 0.0, "ток катушки при выключении сохранил знак");
    let expected = if fixed {
        // Во фронте 1 мкс при L = 1 мГн и V < 1 В ток падает менее чем на 1 мА.
        // Стационарное распределение тока в момент фронта: I_L = V/20 + Id(V).
        assert!((1.0 - 0.001..=1.0).contains(&current));
        bisect(0.0, 1.0, |v| {
            v / 20.0 + 1e-9 * (v / thermal_v()).exp_m1() - 1.0
        })
    } else {
        // Точный ответ параллельной RL-ветви на снятие тока линейным фронтом.
        let normalized: f64 = 20.0 * 1e-6 / 0.001;
        20.0 * (-(-normalized).exp_m1() / normalized)
    };
    // 2 мВ покрывают 1 мА неопределённости тока и шаг интегрирования около фронта.
    near(
        "−L·di/dt или ограничение диодом, В",
        peak.value,
        expected,
        0.002,
    );
    expected
}

fn check_verdict<'a>(report: &'a VerdictReport, class: &str, fixed: bool) -> Vec<&'a Finding> {
    let expected = if fixed { Outcome::Pass } else { Outcome::Fail };
    let targeted: Vec<_> = report.findings.iter().filter(|f| f.code == class).collect();
    assert!(
        !report
            .inapplicable
            .iter()
            .any(|item| item.code.as_deref() == Some(class))
    );
    assert_eq!(
        report.simulation,
        if fixed {
            SimulationVerdict::NotProven
        } else {
            SimulationVerdict::Rejected
        }
    );
    assert!(report.findings.iter().any(|f| {
        f.key.class == HazardClass::Execution
            && f.electrical.is_some()
            && f.outcome == Outcome::NotAnalyzed
            && f.reason.contains("фактического состава")
    }));
    if fixed {
        assert!(
            targeted.iter().all(|f| f.outcome != Outcome::Fail),
            "{class}: {targeted:?}"
        );
    }
    let mut witnesses = Vec::new();
    let mut check = |object: &str, mode: Option<Mode>, probe: Option<Probe>, reason: &str| {
        let mode = mode.map(|value| format!("{value:?}"));
        let probe = probe.map(|value| format!("{value:?}"));
        let finding = targeted
            .iter()
            .copied()
            .find(|finding| {
                finding.key.object == format!("блок/{object}")
                    && finding.outcome == expected
                    && finding.reason.contains(reason)
                    && finding.electrical.as_ref().is_some_and(|detail| {
                        detail.mode == mode
                            && probe
                                .as_ref()
                                .is_none_or(|p| detail.probe.as_ref() == Some(p))
                    })
            })
            .unwrap_or_else(|| panic!("{class}/{object} {expected:?}: {targeted:?}"));
        assert!(finding.journal.is_some(), "{finding:?}");
        assert_eq!(finding.confirmed_hazard, !fixed);
        if fixed {
            assert!(
                finding
                    .model_scope
                    .as_deref()
                    .unwrap()
                    .contains("эталонный стенд")
            );
        }
        if let Some(limit) = &finding.limit {
            assert!(!finding.limit_sources.is_empty());
            let [low, high] = finding.measurement.unwrap();
            assert!(low.is_finite() && high.is_finite() && low <= high);
            if matches!(class, "rail_sag" | "reverse_polarity") {
                let [min, max] = limit.lower.unwrap();
                assert!(limit.upper.is_none());
                assert!(if fixed { low > max } else { high < min });
            } else {
                let [min, max] = limit.upper.unwrap();
                assert!(limit.lower.is_none());
                assert!(if fixed { high < min } else { low > max });
            }
            println!(
                "  вердикт {expected:?} · {} · {:?} {limit:?}",
                finding.reason, finding.measurement
            );
        } else {
            println!("  вердикт {expected:?} · {}", finding.reason);
        }
        witnesses.push(finding);
    };
    let abs_current = |component, element| Some(Probe::Current(branch(component, element)));
    let reason = |failure, success| if fixed { success } else { failure };
    match class {
        "assembly_mismatch" => {
            let plugs: Vec<_> = targeted.iter().filter(|f| is_plug(f)).collect();
            assert_eq!(plugs.len(), if fixed { 2 } else { 1 });
            for f in plugs {
                check(
                    f.key.object.strip_prefix("блок/").unwrap(),
                    None,
                    None,
                    reason("несовместимые контакты", "сопрягаются"),
                );
            }
        }
        "missing_ground" => check(
            "sender/plus → receiver/input",
            None,
            None,
            reason(
                "нет штатного пути возврата",
                "найден штатный физический путь возврата",
            ),
        ),
        "reverse_polarity" => check(
            "capacitor/reverse",
            None,
            Some(volts("capacitor/plus", "capacitor/minus")),
            reason(
                "обратное напряжение превышает",
                "внутри предела обратной полярности",
            ),
        ),
        "pin_overvoltage" => {
            let clamp = |name| Some(Probe::Current(branch("fixture/p0", name)));
            check(
                "fixture/p0",
                Some(Mode::MainOnly),
                clamp("upper"),
                "ток защитного диода",
            );
        }
        "ground_switch_backfeed" => check(
            "block/signal_path",
            Some(Mode::LoadOff),
            abs_current("block", "signal_path"),
            reason("при выключенном ключе", "пик тока"),
        ),
        "source_conflict" if fixed => {
            for source in ["main", "usb"] {
                check(
                    &format!("{source}/emf"),
                    Some(Mode::Both),
                    abs_current(source, "emf"),
                    "внутри применимого предела",
                );
            }
        }
        "source_conflict" => check(
            "main → usb",
            Some(Mode::Both),
            abs_current("usb", "emf"),
            "прямое параллельное включение подаёт ток",
        ),
        "rail_sag" => check(
            "block/minimum_supply_voltage",
            Some(Mode::Start),
            Some(volts("block/p", "block/g")),
            reason("минимум напряжения шины ниже", "на двух сетках"),
        ),
        "startup_actuation" => {
            for mode in [Mode::Reset, Mode::Boot(Time::ZERO)] {
                check(
                    "load/load",
                    Some(mode),
                    abs_current("load", "load"),
                    reason("предел непреднамеренного включения", "пик тока"),
                );
            }
        }
        "inductive_spike" => check(
            "block/turnoff_voltage",
            Some(Mode::LoadOff),
            Some(volts("block/g", "block/p")),
            reason("импульс напряжения при отключении", "на двух сетках"),
        ),
        "switch_underdrive" => check(
            "block/channel",
            Some(Mode::Normal),
            Some(Probe::Power(branch("block", "channel"))),
            reason("потери канала", "внутри применимого предела"),
        ),
        "connection_overcurrent" => check(
            "feed",
            Some(Mode::MainOnly),
            Some(Probe::Current(link("feed"))),
            reason("при пределе", "пик тока"),
        ),
        "regulator_overload" => check(
            "regulator/regulation",
            Some(Mode::Normal),
            abs_current("regulator", "regulation"),
            reason("при пределе", "пик тока"),
        ),
        _ => unreachable!("неизвестный класс {class}"),
    }
    witnesses
}

fn is_plug(finding: &Finding) -> bool {
    finding
        .electrical
        .as_ref()
        .and_then(|f| f.witness.as_ref())
        .is_some_and(|w| w.contains("kind: Plug"))
}

#[test]
fn electrical_corpus() {
    // Изолируем настройку синтаксического адаптера от других тестов и процессов.
    const REAL_SPICE: &str = "SIM_CORPUS_REAL_NGSPICE";
    if std::env::var_os(REAL_SPICE).is_none() {
        let engine = std::env::var_os("NGSPICE")
            .map_or_else(
                || root().join("assets/third_party/ngspice/bin/ngspice"),
                PathBuf::from,
            )
            .canonicalize()
            .expect("обязательный ngspice не найден");
        let child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "electrical_corpus", "--nocapture"])
            .env("NGSPICE", corpus().join("ngspice.sh"))
            .env(REAL_SPICE, engine)
            .output()
            .unwrap();
        print!("{}", String::from_utf8_lossy(&child.stdout));
        assert!(
            child.status.success(),
            "{}",
            String::from_utf8_lossy(&child.stderr)
        );
        return;
    }
    let m = manifest();
    let catalog = catalog();
    let stand = ElectricalStand::new("корпус");
    for class in CLASSES {
        for fixed in [false, true] {
            let path = corpus().join(&m.pairs[class][usize::from(fixed)]);
            let input = load_schematic(&path, &catalog).unwrap();
            assert_eq!(input.path, path);
            assert_eq!(input.bytes, fs::read(&input.path).unwrap());
            let verdict = stand.assess(&input);
            let witnesses = check_verdict(&verdict, class, fixed);

            println!("{class} · {}", path.file_name().unwrap().to_string_lossy());
            let reference = match class {
                "assembly_mismatch" => topology(&input, StaticClass::AssemblyMismatch, fixed)
                    .map(|value| (value, LINEAR)),
                "missing_ground" => {
                    topology(&input, StaticClass::MissingGround, fixed).map(|value| (value, LINEAR))
                }
                "reverse_polarity" => topology(&input, StaticClass::ReversePolarity, fixed)
                    .map(|value| (value, LINEAR)),
                "pin_overvoltage" => Some((pin_overvoltage(&input, fixed), 1e-7)),
                "ground_switch_backfeed" => Some((backfeed(&input, fixed), LINEAR)),
                "source_conflict" => Some((source_conflict(&input, fixed), LINEAR)),
                "rail_sag" => Some((rail_sag(&input, fixed), LINEAR)),
                "startup_actuation" => Some((startup(&input, fixed), LINEAR)),
                "inductive_spike" => Some((inductive_spike(&input, fixed), 0.002)),
                "switch_underdrive" => Some((transistor(&input, fixed), 1e-6)),
                "connection_overcurrent" => Some((connection_current(&input, fixed), LINEAR)),
                "regulator_overload" => Some((regulator(&input, fixed, &catalog), LINEAR)),
                _ => unreachable!("неизвестный класс {class}"),
            };
            if let Some((value, tolerance)) = reference {
                for witness in witnesses {
                    for bound in witness.measurement.unwrap() {
                        assert!(
                            (bound - value).abs() <= tolerance,
                            "{class}: граница вердикта {bound}, вручную {value}, допуск {tolerance}"
                        );
                    }
                }
            }
        }
    }
    println!("Э2 · 12 пар, 24 схемы, 12 ручных сверок, 24 DC-сверки, 4 перехода на двух сетках");
}

#[test]
fn без_подтверждённого_предела_продуктовый_свидетель_остаётся_открытым() {
    let catalog = catalog();
    let stand = ElectricalStand::new("пределы");
    for (class, part, rating) in [
        (
            "ground_switch_backfeed",
            "backfeed_low",
            "off_signal_current",
        ),
        ("rail_sag", "rail_weak", "minimum_supply_voltage"),
        ("startup_actuation", "load_gate", "uncommanded_load_current"),
        ("inductive_spike", "coil_open", "turnoff_voltage"),
        ("switch_underdrive", "gate_logic", "channel_loss"),
    ] {
        let unsafe_case = load(class, false, &catalog);
        check_verdict(&stand.assess(&unsafe_case), class, false);
        for omit in [false, true] {
            let mut parts = catalog.parts().clone();
            let ratings = &mut parts.get_mut(part).unwrap().ratings;
            if omit {
                ratings.remove(rating).unwrap();
            } else {
                ratings
                    .get_mut(rating)
                    .unwrap()
                    .limit
                    .range
                    .as_mut()
                    .unwrap()
                    .provenance = Provenance::Estimate("неподтверждённое предположение".into());
            }
            let altered = Catalog::new(parts).unwrap();
            let input = load(class, false, &altered);
            let verdict = stand.assess(&input);
            let mut target = verdict
                .findings
                .iter()
                .filter(|f| f.code == class)
                .peekable();
            assert!(target.peek().is_some(), "{class}: потеряно обязательство");
            assert!(
                target.all(|f| f.outcome == Outcome::NotAnalyzed),
                "{class}: {verdict:?}"
            );
        }
    }

    let mut case = load("rail_sag", true, &catalog);
    let original = stand.assess(&case);
    check_verdict(&original, "rail_sag", true);
    for omit in [false, true] {
        let sources = &mut case.modes.cases.get_mut(&Mode::Start).unwrap().sources;
        if omit {
            sources.clear();
        } else {
            for trace in sources.values_mut() {
                let initial = trace.points[0].1;
                for (_, value) in &mut trace.points {
                    *value = initial;
                }
            }
        }
        let verdict = stand.assess(&case);
        let mut findings = verdict
            .findings
            .iter()
            .filter(|f| f.code == "rail_sag")
            .peekable();
        assert!(findings.peek().is_some());
        assert!(findings.all(|f| f.outcome == Outcome::NotAnalyzed));
    }
}

#[test]
fn rejects_unknown_limits_and_invalid_data() {
    let catalog = catalog();
    let pin = load("pin_overvoltage", false, &catalog);
    let source = std::str::from_utf8(&pin.bytes).unwrap();
    for (from, to, error) in [
        ("nominal = 100.0", "nominal = nan", "resistance_ohm"),
        ("nominal = 100.0", "nominal = inf", "resistance_ohm"),
        (
            "ends = [\"board/io\"]",
            "ends = [\"board/unknown\"]",
            "неизвестный вывод",
        ),
    ] {
        let changed = source.replacen(from, to, 1);
        assert_ne!(changed, source);
        let reason = SchematicInput::parse(&pin.path, changed.into_bytes(), &catalog).unwrap_err();
        assert!(reason.contains(error), "{reason}");
    }

    let current_input = load("connection_overcurrent", true, &catalog);
    let mode = prepare(&current_input, &current_input.modes, &Mode::MainOnly);
    let mut rating = joint_limit(&current_input, "feed");
    rating.provenance = Provenance::Estimate("неподтверждённый предел".into());
    let result = mode
        .check_current(&link("feed"), CurrentDirection::Absolute, Some(&rating))
        .unwrap();
    assert!(
        matches!(result, HazardCheck::NotAnalyzed { reason } if reason.contains("не подтверждён"))
    );

    let linked = load("missing_ground", false, &catalog);
    let (isolated, _) = std::str::from_utf8(&linked.bytes)
        .unwrap()
        .split_once("[connections.signal]")
        .unwrap();
    let float =
        SchematicInput::parse(&linked.path, isolated.as_bytes().to_vec(), &catalog).unwrap();
    let probes = BTreeSet::from([volts("receiver/return", "sender/minus")]);
    let mut plan = ModePlan::default();
    plan.cases.insert(
        Mode::Normal,
        ModeCase {
            source: Provenance::Derived(
                "Волна 1, проверка неопределённости между островами".into(),
            ),
            probes,
            window: None,
            sources: BTreeMap::new(),
            pins: BTreeMap::new(),
            sampling: None,
        },
    );
    assert!(matches!(
        float.compiled.prepare_mode(&plan, &Mode::Normal).unwrap(),
        Analysis::NotAnalyzed { reason } if reason.contains("плавающий") || reason.contains("острова")
    ));
}

#[test]
fn rejects_incomplete_spice_trace() {
    use std::os::unix::fs::PermissionsExt;

    // Запуск в дочернем процессе изолирует настройку NGSPICE от остальных тестов.
    const CASE_ENV: &str = "SIM_CORPUS_SPICE_NEGATIVE";
    if let Ok(expected) = std::env::var(CASE_ENV) {
        let input = load("rail_sag", false, &catalog());
        let mode = prepare(&input, &input.modes, &Mode::Start);
        let probe = volts("block/p", "block/g");
        let error = mode
            .compare_ngspice(&BTreeMap::from([(
                probe,
                ReferenceCheck::Transient(window(0, 800000, 3.0, 0.010)),
            )]))
            .unwrap_err();
        assert!(error.contains(&expected), "{error}");
        return;
    }

    struct Temp(PathBuf);
    impl Drop for Temp {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }
    let dir = std::env::temp_dir().join(format!("sim-corpus-spice-{}", std::process::id()));
    fs::create_dir(&dir).unwrap();
    let temp = Temp(dir);
    let fake = temp.0.join("ngspice");
    for (permissions, script, expected) in [
        (0o600, "", "не удалось запустить ngspice"),
        (0o700, "cat >/dev/null\nexit 9", "ошибка расчёта ngspice"),
        (0o700, "cat >/dev/null\n", "нет таблицы"),
        (
            0o700,
            r#"columns=$(sed -n 's/^print //p')
printf '%s\n' 'No. of Data Rows : 3' "Index time $columns" '0 0 5' '1 0.0000001 5'"#,
            "неполная таблица",
        ),
        (
            0o700,
            r#"columns=$(sed -n 's/^print //p')
printf '%s\n' 'No. of Data Rows : 2' "Index time $columns" '0 0 5' '1 0.0008 3'"#,
            "неполная или немонотонная шкала",
        ),
        (
            0o700,
            r#"columns=$(sed -n 's/^print //p')
printf '%s\n' 'No. of Data Rows : 1' 'Index time' '0 0'
"#,
            "отсутствуют запрошенные векторы",
        ),
        (
            0o700,
            r#"columns=$(sed -n 's/^print //p')
printf '%s\n' 'No. of Data Rows : 1' "Index time $columns" '0 0 NaN'"#,
            "неконечная или нечитаемая строка",
        ),
        (
            0o700,
            // Шаг 200 нс со смещением 1 нс строже предела 250 нс, но пропускает фронты.
            r#"columns=$(sed -n 's/^print //p')
printf '%s\n' 'No. of Data Rows : 4002' "Index time $columns" '0 0 5'
awk 'BEGIN {
    for (i = 1; i <= 4000; i++) printf "%d %.17g 5\n", i, (1+(i-1)*200)*1e-9
    print "4001 0.0008 3"
}'"#,
            "метки PWL покрыты неполностью",
        ),
    ] {
        fs::write(&fake, format!("#!/bin/sh\n{script}\n")).unwrap();
        fs::set_permissions(&fake, fs::Permissions::from_mode(permissions)).unwrap();
        let child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "rejects_incomplete_spice_trace", "--nocapture"])
            .env("NGSPICE", &fake)
            .env(CASE_ENV, expected)
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{expected}\n{}\n{}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        assert!(String::from_utf8_lossy(&child.stdout).contains("1 passed"));
    }
}
