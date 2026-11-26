//! Приёмка Т6: клеммные модели, живой журнал и вердикт кампании.

use std::{
    path::{Path, PathBuf},
    process::Command,
};

use esd_event::*;
use serde::Deserialize;
use serde_json::Value;
use sim_cli::electrical::{SchematicInput, load_catalog, load_schematic, parse_catalog};
use sim_process::{
    Analysis, ProcessError, SourcedRange, schematic,
    schematic::{
        Endpoint, Parameter, ResetProfile, Scope, Status,
        emi_reset::{
            EmiBand, EmiCriterion, EmiInput, EmiInterval, EmiOutcome, EmiReturn, EmiSourceKind,
            EmiTransfer, EmiTrial, prepare_emi_reset,
        },
        sim_circuit::{
            emc::loop_model::{LumpedDomain, SeriesRl},
            netlist,
        },
    },
};
use sim_runtime::{
    LogKind, Runtime,
    control::{self, ControlAssets, ControlConfig, EventKind, RunSpec},
    electrical::{BoardConnection, BoardCurrents, ElectricalSetup, TransientCircuit},
};
use sim_signal::{Rate, Time};
use sim_task::{
    Fingerprint, Outcome,
    verdict::{Finding, SimulationVerdict, VerdictReport},
};
use sim_world::{CompiledWorld, Provenance};

// До экспорта модуля Т6.6 используется тот же компилируемый оценщик,
// что и в sim-process/tests/esd_event.rs.
extern crate self as sim_circuit;
pub use sim_process::schematic::sim_circuit::ngspice;
#[allow(dead_code)]
// Остальные уровни контактного импульса покрывает Т6.6.
#[path = "../../sim-process/src/schematic/esd_event.rs"]
mod esd_event;

mod verdict {
    include!("fixtures/electrical_verdict.rs");

    impl ElectricalStand {
        pub(super) fn with_emc(
            &self,
            scheme: &SchematicInput,
            record: impl FnOnce(&mut ElectricalEvidence),
        ) -> VerdictReport {
            fs::write(self.directory.0.join("scheme.toml"), &scheme.bytes).unwrap();
            let mut evidence = ElectricalEvidence::evaluate(
                "блок",
                &scheme.bytes,
                &scheme.compiled,
                &scheme.modes,
                self.inventory.build.unwrap(),
            )
            .unwrap();
            record(&mut evidence);
            let requirements = derive_requirements_with_electrical(
                &self.inventory,
                &self.plan,
                &BTreeMap::from([("блок".into(), evidence)]),
            )
            .unwrap();
            assess_campaign(&requirements, &CampaignEvidence::new(&requirements)).unwrap()
        }

        pub(super) fn cli(
            &self,
            scheme: &SchematicInput,
            catalog: &std::path::Path,
            profile: Option<&str>,
        ) -> serde_json::Value {
            let dir = &self.directory.0;
            fs::write(dir.join("scheme.toml"), &scheme.bytes).unwrap();
            let parts = dir.join("catalog/electrical");
            fs::create_dir_all(&parts).unwrap();
            fs::copy(catalog.join("reference.toml"), parts.join("reference.toml")).unwrap();
            if let Some(profile) = profile {
                fs::write(dir.join("emc.toml"), profile).unwrap();
            }
            let manifest = dir.join("electrical.toml");
            fs::write(
                &manifest,
                format!(
                    "electrical_format = 1\ncatalog = \"catalog\"\n[assemblies.block]\nworld = \"world.toml\"\n{}",
                    if profile.is_some() { "emc = \"emc.toml\"\n" } else { "" },
                ),
            )
            .unwrap();
            let output = std::process::Command::new(env!("CARGO_BIN_EXE_sim-run"))
                .arg("--campaign")
                .arg(manifest)
                .arg("--json")
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "{}\n{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
            assert_eq!(report["version"], 1);
            report["assemblies"]["block"].clone()
        }
    }
}
use verdict::ElectricalStand;

const MANIFEST: &str = include_str!("../../../devices/electrical/depth_t6/acceptance.toml");
const ROUNDING: f64 = 1e-12; // Округление ручных расчётов U/I и направленные границы.
const NS: f64 = 1e-9; // Секунд в наносекунде.

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Acceptance {
    format: u32,
    spectrum: Spectrum,
    esd: Esd,
    reset: Reset,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Spectrum {
    reference: PathBuf,
    rl_deck: PathBuf,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Esd {
    source: String,
    catalog: String,
    scheme: String,
    ports: [String; 3],
    start_ns: u64,
    bandwidth_hz: f64,
    length_m: f64,
    wave_speed_m_s: f64,
    max_length_wavelengths: f64,
    max_terminal_v: f64,
    samples_ns_a: Vec<[f64; 2]>,
    series_ohm: f64,
    pin_ohm: f64,
    voltage_limit_v: f64,
    current_limit_a: f64,
    tvs: Tvs,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Tvs {
    source: String,
    knee_v: f64,
    dynamic_ohm: f64,
    max_injected_peak_a: f64,
    min_rise_ns: f64,
    max_pulse_ns: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reset {
    catalog: PathBuf,
    scheme: PathBuf,
    source: String,
    board: String,
    switch: String,
    input: String,
    reference: String,
    start_ns: u64,
    end_ns: u64,
    horizon_ns: u64,
    bias_v: f64,
    falling_v: f64,
    rising_v: f64,
    assert_ns: u64,
    release_ns: u64,
    current_a: f64,
    unsafe_ohm: f64,
    fixed_ohm: f64,
    bandwidth_hz: f64,
    model_max_hz: f64,
    length_m: f64,
    wave_speed_m_s: f64,
    max_length_wavelengths: f64,
}

fn acceptance() -> Acceptance {
    let data: Acceptance = toml::from_str(MANIFEST).unwrap();
    assert_eq!(data.format, 1);
    data
}

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/depth_t6")
}

fn exact(value: f64, source: &Provenance) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source.clone(),
    }
}

fn endpoint(address: &str) -> Endpoint {
    let (component, pin) = address.split_once('/').unwrap();
    Endpoint {
        component: component.into(),
        pin: pin.into(),
    }
}

fn calculated<T>(analysis: Analysis<T>) -> T {
    match analysis {
        Analysis::Analyzed { value, scope } => {
            assert!(!scope.is_empty());
            value
        }
        Analysis::NotAnalyzed { reason } => panic!("NotAnalyzed: {reason}"),
    }
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.14e} вместо {expected:.14e}, допуск {tolerance:e}"
    );
}

fn finding<'a>(
    report: &'a VerdictReport,
    code: &str,
    variant: Fingerprint,
    expected: Outcome,
) -> &'a Finding {
    let target = report
        .findings
        .iter()
        .filter(|item| item.code == code)
        .collect::<Vec<_>>();
    assert_eq!(target.len(), 1, "{code}");
    let item = target[0];
    assert_eq!(item.key.class.spec().id, code);
    assert_eq!(item.outcome, expected, "{item:?}");
    assert_eq!(item.confirmed_hazard, expected == Outcome::Fail);
    assert!(
        item.measurement_sources
            .iter()
            .any(|source| { source.contains(&format!("{:016x}", variant.as_u64())) })
    );
    assert_eq!(
        item.electrical.as_ref().unwrap().build,
        report.build.unwrap()
    );
    assert!(
        report
            .findings
            .iter()
            .all(|other| other.code == code || other.outcome != Outcome::Fail)
    );
    assert_eq!(
        report.simulation,
        if expected == Outcome::Fail {
            SimulationVerdict::Rejected
        } else {
            SimulationVerdict::NotProven
        }
    );
    if expected == Outcome::NotAnalyzed {
        assert!(!item.missing_data.is_empty());
    }
    item
}

fn esd_report(
    stand: &ElectricalStand,
    scheme: &SchematicInput,
    input: &EsdInput<'_>,
    variant: Fingerprint,
) -> (VerdictReport, schematic::Finding<EsdEvidence>) {
    let mut result = None;
    let report = stand.with_emc(scheme, |evidence| {
        result = Some(
            evidence
                .record_esd_event(
                    &scheme.compiled,
                    &scheme.modes,
                    "контакт",
                    variant,
                    |compiled| assess_esd_event(compiled, input),
                )
                .unwrap(),
        );
    });
    (report, result.unwrap())
}

#[test]
fn контактный_импульс_через_вердикт_и_ngspice() {
    // До сравнения с моделью: при 7,5 А, R = 3 + 2 Ом без защиты Uвх = 15 В,
    // Iвх = 7,5 А; запас относительно 4 В / 3 А равен −11 В / −4,5 А.
    // С TVS: 7,5 = Uкл/5 + (Uкл−6,5)/0,5. Тогда Uкл = 205/22 В,
    // Uвх = 41/11 В, Iвх = 41/22 А, запас +3/11 В и +25/22 А.
    let config = acceptance().esd;
    let path = Path::new("блок-контакт.toml");
    let catalog = parse_catalog(path, config.catalog.as_bytes()).unwrap();
    let scheme = SchematicInput::parse(path, config.scheme.into_bytes(), &catalog).unwrap();
    let stand = ElectricalStand::new("t69-esd");
    assert_eq!(
        stand.assess(&scheme).simulation,
        SimulationVerdict::NotProven
    );
    let source = Provenance::Derived(config.source);
    let tvs_source = Provenance::Derived(config.tvs.source);
    let samples: Vec<_> = config
        .samples_ns_a
        .into_iter()
        .map(|[ns, current_abs_a]| EsdSample {
            offset_s: ns * NS,
            current_abs_a,
        })
        .collect();
    let [exposed, protected, return_] = config.ports.map(|port| endpoint(&port));
    let port = EsdPort {
        exposed,
        protected,
        return_,
    };
    let limits = EsdPinLimits {
        voltage_abs_v: exact(config.voltage_limit_v, &source),
        current_abs_a: exact(config.current_limit_a, &source),
    };
    let ng = ngspice::find_ngspice().expect("NotAnalyzed: для независимой сверки нужен ngspice");
    for polarity in [EsdPolarity::Positive, EsdPolarity::Negative] {
        let sign = if polarity == EsdPolarity::Positive {
            1.0
        } else {
            -1.0
        };
        let mut input = EsdInput {
            port: &port,
            start: Time(config.start_ns),
            exposure: Some(&source),
            pulse: Some(EsdPulse {
                level: EsdLevel::Kv2,
                polarity,
                samples: &samples,
                bandwidth_hz: config.bandwidth_hz,
                source: Some(&source),
            }),
            network: Some(EsdNetwork {
                series_ohm: config.series_ohm,
                pin_ohm: config.pin_ohm,
                domain: EsdDomain {
                    bandwidth_hz: config.bandwidth_hz,
                    length_m: config.length_m,
                    wave_speed_m_s: config.wave_speed_m_s,
                    max_length_wavelengths: config.max_length_wavelengths,
                    max_terminal_abs_v: config.max_terminal_v,
                    // Точная резистивная модель синтетического блока.
                    voltage_error_v: 0.0,
                    current_error_a: 0.0,
                    source: Some(&source),
                },
                protection: Some(EsdProtection::Absent(&source)),
            }),
            limits: Some(&limits),
        };
        for (fixed, [volts, amps, margin_v, margin_a], criterion, status, outcome) in [
            (
                false,
                [15.0, 7.5, -11.0, -4.5],
                EsdCriterion::Exceeded,
                Status::Fail,
                Outcome::Fail,
            ),
            (
                true,
                [41.0 / 11.0, 41.0 / 22.0, 3.0 / 11.0, 25.0 / 22.0],
                EsdCriterion::WithinBench,
                Status::NotAnalyzed,
                Outcome::NotAnalyzed,
            ),
        ] {
            if fixed {
                input.network.as_mut().unwrap().protection = Some(EsdProtection::Tvs(EsdTvs {
                    polarity,
                    knee_v: config.tvs.knee_v,
                    dynamic_ohm: config.tvs.dynamic_ohm,
                    max_injected_peak_a: config.tvs.max_injected_peak_a,
                    min_rise_s: config.tvs.min_rise_ns * NS,
                    max_pulse_s: config.tvs.max_pulse_ns * NS,
                    datasheet: Some(&tvs_source),
                }));
            }
            let variant = Fingerprint::of(format!("{MANIFEST}\n{polarity:?} {fixed}").as_bytes());
            let (report, result) = esd_report(&stand, &scheme, &input, variant);
            assert_eq!(result.status, status);
            assert_eq!(result.evidence.criterion, Some(criterion));
            let item = finding(&report, "esd_event", variant, outcome);
            assert_eq!(
                item.electrical.as_ref().unwrap().probe.as_deref(),
                Some("block/terminal")
            );
            let response = calculated(result.evidence.response);
            close(response.peak.pin_v, sign * volts, ROUNDING);
            close(response.peak.pin_a, sign * amps, ROUNDING);
            close(response.electrical_length_wavelengths, 0.01, ROUNDING);
            let margins = calculated(result.evidence.margins);
            close(margins.voltage_v.lower, margin_v, ROUNDING);
            close(margins.current_a.upper, margin_a, ROUNDING);

            // Т6.6: 16 и 32 шага на фронт 0,8 нс. Допуск сверки 1 мкВ / 1 мкА.
            const SPICE: f64 = 1e-6;
            for max_step_s in [0.05 * NS, 0.025 * NS] {
                let reference = calculated(
                    cross_check_esd_event(
                        &scheme.compiled,
                        &input,
                        &ng,
                        EsdSpiceOptions {
                            max_step_s,
                            voltage_tolerance_v: SPICE,
                            current_tolerance_a: SPICE,
                        },
                    )
                    .unwrap(),
                );
                assert!(reference.within_tolerance, "{reference:?}");
                assert!(
                    reference.rows
                        >= (samples.last().unwrap().offset_s / max_step_s).floor() as usize
                );
                close(reference.pin_voltage_peak_abs_v, volts, SPICE);
                close(reference.pin_current_peak_abs_a, amps, SPICE);
                println!("ESD {polarity:?}, TVS {fixed}, шаг {max_step_s:e}: {reference:?}");
            }
        }

        let variant = Fingerprint::of("контакт: отрицательные постановки".as_bytes());
        let mut absent = input;
        absent.limits = None;
        let mut outside = input;
        outside.pulse.as_mut().unwrap().bandwidth_hz = config.bandwidth_hz * 2.0;
        let mut unknown = input;
        unknown.network.as_mut().unwrap().domain.source = None;
        let mut unsourced_limits = limits.clone();
        unsourced_limits.voltage_abs_v.provenance =
            Provenance::Estimate("предел без подтверждения".into());
        let mut unsourced = input;
        unsourced.limits = Some(&unsourced_limits);
        for open in [absent, outside, unknown, unsourced] {
            let (report, result) = esd_report(&stand, &scheme, &open, variant);
            assert_eq!(result.status, Status::NotAnalyzed);
            assert_eq!(result.evidence.criterion, None);
            finding(&report, "esd_event", variant, Outcome::NotAnalyzed);
        }
        // В защищённом варианте ручные пики 41/11 В и 41/22 А.
        // Сдвигаем каждый предел отдельно на ±1 мВ или ±1 мА.
        const OFFSET: f64 = 1e-3;
        for (voltage_offset, current_offset) in
            [(-OFFSET, OFFSET), (OFFSET, -OFFSET), (OFFSET, OFFSET)]
        {
            let shifted = EsdPinLimits {
                voltage_abs_v: exact(41.0 / 11.0 + voltage_offset, &source),
                current_abs_a: exact(41.0 / 22.0 + current_offset, &source),
            };
            let mut near = input;
            near.limits = Some(&shifted);
            let (report, result) = esd_report(&stand, &scheme, &near, variant);
            let expected = if voltage_offset < 0.0 || current_offset < 0.0 {
                Outcome::Fail
            } else {
                Outcome::NotAnalyzed
            };
            finding(&report, "esd_event", variant, expected);
            let margins = calculated(result.evidence.margins);
            close(margins.voltage_v.nominal, voltage_offset, ROUNDING);
            close(margins.current_a.nominal, current_offset, ROUNDING);
        }
        for non_finite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut invalid = input;
            invalid.network.as_mut().unwrap().pin_ohm = non_finite;
            stand.with_emc(&scheme, |evidence| {
                assert!(
                    evidence
                        .record_esd_event(
                            &scheme.compiled,
                            &scheme.modes,
                            "контакт",
                            variant,
                            |compiled| assess_esd_event(compiled, &invalid),
                        )
                        .is_err()
                );
                assert!(
                    evidence
                        .findings()
                        .iter()
                        .all(|item| item.code != "esd_event")
                );
            });
        }
    }
}

impl Reset {
    fn scheme(&self) -> SchematicInput {
        let parts = load_catalog(&data_dir().join(&self.catalog)).unwrap();
        load_schematic(&data_dir().join(&self.scheme), &parts.catalog).unwrap()
    }

    fn parameter(&self, value: f64) -> Parameter {
        Parameter {
            range: Some(exact(value, &Provenance::Derived(self.source.clone()))),
            measurement: "синхронный замер на клеммах блока".into(),
        }
    }

    fn input(&self, resistance_ohm: f64) -> EmiInput {
        let source = Provenance::Derived(self.source.clone());
        EmiInput {
            scope: Scope::Reference,
            source: self.switch.clone(),
            source_kind: EmiSourceKind::Switch,
            board: self.board.clone(),
            input: endpoint(&self.input),
            reference: endpoint(&self.reference),
            binding: Some(source.clone()),
            start: Time(self.start_ns),
            end: Time(self.end_ns),
            bias_v: self.parameter(self.bias_v),
            detector: Some(ResetProfile {
                falling_v: self.parameter(self.falling_v),
                rising_v: self.parameter(self.rising_v),
                assert_delay_s: self.parameter(Time(self.assert_ns).as_secs()),
                release_delay_s: self.parameter(Time(self.release_ns).as_secs()),
            }),
            transfer: Some(EmiTransfer::SharedReturn(EmiReturn {
                geometry: "один резистивный общий возврат блока".into(),
                domain: LumpedDomain {
                    max_frequency_hz: self.model_max_hz,
                    longest_path_m: self.length_m,
                    min_wave_speed_m_s: self.wave_speed_m_s,
                    max_wavelength_fraction: self.max_length_wavelengths,
                },
                rl: SeriesRl {
                    resistance_ohm,
                    inductance_h: 0.0,
                },
                highest_frequency_hz: self.bandwidth_hz,
                model_source: Some(source),
                remainder_v: self.parameter(0.0),
                intervals: vec![EmiInterval {
                    end: Time(self.end_ns),
                    value: (EmiBand::point(self.current_a), EmiBand::point(0.0)),
                }],
            })),
        }
    }

    fn profile(&self) -> String {
        // Тот же вход и окно, дельта на клеммах рассчитана по Ому для варианта без исправления.
        let delta_v = -self.unsafe_ohm * self.current_a;
        let source = format!(r#"{{kind = "derived", reference = {:?}}}"#, self.source);
        let range = |v: f64| {
            format!(r#"{{ min = {v:e}, nominal = {v:e}, max = {v:e}, source = {source} }}"#)
        };
        format!(
            r#"
format = 1
[instances."блок".emi_reset]
source = {switch:?}
source_kind = "switch"
board = {board:?}
input = {input:?}
reference = {reference:?}
binding = {source}
start_ns = {start}
end_ns = {end}
bias_v = {bias}

[instances."блок".emi_reset.detector]
falling_v = {falling}
rising_v = {rising}
assert_delay_s = {hold}
release_delay_s = {release}

[instances."блок".emi_reset.transfer]
kind = "measured"
source = {source}
intervals = [{{ end_ns = {end}, delta_v = [{delta_v:e}, {delta_v:e}, {delta_v:e}] }}]
"#,
            switch = self.switch,
            board = self.board,
            input = self.input,
            reference = self.reference,
            start = self.start_ns,
            end = self.end_ns,
            bias = range(self.bias_v),
            falling = range(self.falling_v),
            rising = range(self.rising_v),
            hold = range(Time(self.assert_ns).as_secs()),
            release = range(Time(self.release_ns).as_secs()),
        )
    }
}

struct Assets;

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        match name {
            "образ" => Ok(include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec()),
            // Контракт vmcu_ports: баннер R, watchdog выключен, ШИМ 1 кГц.
            "данные" => Ok([vec![b'R', 0, 0, 0], 1000_u32.to_le_bytes().to_vec()].concat()),
            _ => Err("неизвестный ресурс блока".into()),
        }
    }

    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        Err("блок исполняет образ платы".into())
    }
}

fn run_reset(
    scheme: &SchematicInput,
    trial: &EmiTrial,
    coupled: bool,
    horizon: Time,
    until: Time,
) -> Runtime {
    // Т6.7: физика и цепь через 10 мкс, плата через 20 мкс.
    // Идеальный источник 3,3 В не проседает при работе платы.
    let step = Time(10_000);
    let mut runtime = Runtime::load(CompiledWorld {
        name: "блок".into(),
        seed: 0,
        physics: Rate::hz(100_000).unwrap(),
        mjcf: r#"<mujoco><option timestep="0.00001"/><worldbody/></mujoco>"#.into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    })
    .unwrap();
    runtime
        .register_observation("постоянный вход блока", "V")
        .unwrap();
    let input = trial.input();
    let circuit = netlist::parse("блок\nVrail p 0 3.3\nIload p 0 EXT 0\n.end\n").unwrap();
    let zero = exact(
        0.0,
        &Provenance::Derived("идеальная шина стенда Т6.7".into()),
    );
    runtime
        .install_electrical(
            ElectricalSetup {
                max_step: step,
                scope: input.scope,
                sources: vec![],
                probes: Default::default(),
                boards: vec![BoardConnection {
                    board: input.board.clone(),
                    plus: "p".into(),
                    minus: "0".into(),
                    current_source: "iload".into(),
                    reset: input.detector.clone().unwrap(),
                    pins: vec![],
                    initially_in_reset: false,
                    boot_duration_s: Parameter {
                        range: Some(zero.clone()),
                        measurement: "задержка модели платы".into(),
                    },
                    currents: BoardCurrents {
                        unpowered_a: zero.clone(),
                        reset_a: zero.clone(),
                        boot_a: zero.clone(),
                        working_a: zero,
                    },
                }],
            },
            &scheme.bytes,
            TransientCircuit::new(circuit, step, vec![]).unwrap(),
        )
        .unwrap();
    runtime.bind_emi_reset(trial.clone(), coupled).unwrap();
    let cfg = format!(
        r#"
version = 1
[[board]]
name = {board:?}
catalog = "vmcu-c3"
image = "образ"
config = "данные"
rate_hz = 50000
source = "синтетический стенд Т6.7"
[[board_input]]
name = "постоянный вход образа"
board = {board:?}
signal = "постоянный вход блока"
unit = "V"
kind = "adc"
port = 0
max_age_ns = 60000
scale = 1
offset = 0
fallback = 1.65
source = "половина шины 3,3 В для входа тестового образа"
"#,
        board = input.board
    );
    control::install(
        &mut runtime,
        &ControlConfig::parse(&cfg).unwrap(),
        &Assets,
        RunSpec {
            world: b"block",
            catalog: &scheme.bytes,
            task: MANIFEST.as_bytes(),
            criteria: b"emi_reset",
            mode: "сброс блока",
            horizon,
            model_ports: &[],
        },
    )
    .unwrap();
    assert_eq!(
        runtime.run_until(until).unwrap(),
        sim_runtime::Status::Running
    );
    runtime
}

fn reset_edges(runtime: &Runtime, board: &str) -> Vec<(u64, bool)> {
    control::run(runtime)
        .unwrap()
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::ElectricalReset {
                board: target,
                asserted,
            } if target == board => Some((event.at.0, *asserted)),
            _ => None,
        })
        .collect()
}

fn journal(runtime: &Runtime) -> u64 {
    control::run(runtime).unwrap().log().fingerprint()
}

fn input_voltage(runtime: &Runtime, at: Time) -> f64 {
    let log = runtime.log();
    let channel = log
        .header()
        .truth
        .iter()
        .position(|spec| spec.unit == "V" && spec.name.starts_with("ЭМС "))
        .unwrap();
    log.entries()
        .iter()
        .find(|entry| {
            entry.kind == LogKind::Truth
                && entry.id as usize == channel
                && entry.at == at
                && entry.valid
        })
        .unwrap()
        .value
}

#[test]
fn сброс_через_живую_пару_и_вердикт() {
    // До реализации: при I = 2 А и R = 0,6 / 0,4 Ом вход 1,8 / 2,2 В.
    // Для порога 2 В запас −0,2 / +0,2 В. При окне [11,22) мкс и
    // выдержке 8 мкс запас времени −3 / +8 мкс. Срабатывание в 19 мкс,
    // отпускание в 25 мкс (3 мкс после возврата входа к 3 В).
    let setup = acceptance().reset;
    let scheme = setup.scheme();
    let stand = ElectricalStand::new("t69-reset");
    let horizon = Time(setup.horizon_ns);
    for (ohm, margin, dwell, reset, criterion, outcome) in [
        (
            setup.unsafe_ohm,
            -0.2,
            -3000,
            Some(Time(19_000)),
            EmiCriterion::Violated,
            Outcome::Fail,
        ),
        (
            setup.fixed_ohm,
            0.2,
            8000,
            None,
            EmiCriterion::WithinBench,
            Outcome::NotAnalyzed,
        ),
    ] {
        let input = setup.input(ohm);
        let trial = calculated(prepare_emi_reset(input.clone()).unwrap());
        let active = run_reset(&scheme, &trial, true, horizon, horizon);
        let quiet = run_reset(&scheme, &trial, false, horizon, horizon);
        let expected_edges =
            reset.map_or_else(Vec::new, |time| vec![(time.0, true), (25_000, false)]);
        assert_eq!(reset_edges(&active, &setup.board), expected_edges);
        assert!(reset_edges(&quiet, &setup.board).is_empty());
        assert_ne!(journal(&active), journal(&quiet));
        close(input_voltage(&active, input.start), 2.0 + margin, ROUNDING);
        close(input_voltage(&quiet, input.start), 3.0, ROUNDING);
        close(input_voltage(&active, input.end), 3.0, ROUNDING);
        let variant = Fingerprint::of(format!("{MANIFEST}\n{ohm:e}").as_bytes());
        let report = stand.with_emc(&scheme, |evidence| {
            let result = calculated(
                evidence
                    .record_emi_reset(
                        &scheme.compiled,
                        &scheme.modes,
                        "сброс",
                        variant,
                        input.clone(),
                        Some((&active, &quiet)),
                        None,
                    )
                    .unwrap(),
            );
            assert_eq!(result.criterion, Some(criterion));
            close(result.margins.voltage_v.nominal, margin, ROUNDING);
            assert_eq!(result.margins.dwell_ns, [dwell; 3]);
            let observation = result.observation.unwrap();
            assert!(observation.complete);
            assert_eq!(observation.observed_until, horizon);
            assert_eq!(observation.reset_at, reset);
            assert_eq!(observation.external_asserted_at, reset);
            assert_eq!(observation.baseline_reset_at, None);
            if let Some(at) = reset {
                assert_eq!(result.outcome, EmiOutcome::Fail { at });
            } else {
                assert!(matches!(result.outcome, EmiOutcome::NotAnalyzed { .. }));
            }
        });
        let item = finding(&report, "emi_reset", variant, outcome);
        assert_eq!(item.key.window_ns, Some([11_000, 22_000]));
        assert_eq!(item.event_ns, reset.map(|at| at.0));
        assert_eq!(
            item.journal,
            Some(Fingerprint::from_fnv1a(journal(&active)))
        );
        assert!(
            item.measurement_sources
                .iter()
                .any(|source| source.contains(&format!("{:016x}", journal(&quiet))))
        );
        println!("EMI {ohm} Ом: запас {margin:+} В / {dwell:+} нс, события {expected_edges:?}");

        if reset.is_some() {
            // До окна воздействия: даже валидный частичный журнал не доказывает сброс.
            let partial = run_reset(&scheme, &trial, true, horizon, input.start);
            let report = stand.with_emc(&scheme, |evidence| {
                let open = calculated(
                    evidence
                        .record_emi_reset(
                            &scheme.compiled,
                            &scheme.modes,
                            "сброс",
                            variant,
                            input.clone(),
                            Some((&partial, &quiet)),
                            None,
                        )
                        .unwrap(),
                );
                assert_eq!(open.criterion, None);
                assert!(!open.observation.unwrap().complete);
            });
            finding(&report, "emi_reset", variant, Outcome::NotAnalyzed);
            stand.with_emc(&scheme, |evidence| {
                assert!(
                    evidence
                        .record_emi_reset(
                            &scheme.compiled,
                            &scheme.modes,
                            "сброс",
                            variant,
                            input,
                            Some((&quiet, &active)),
                            None,
                        )
                        .is_err()
                );
            });
        }
    }
}

#[test]
fn неполная_или_неконечная_огибающая_сброса() {
    let setup = acceptance().reset;
    let scheme = setup.scheme();
    let stand = ElectricalStand::new("t69-open");
    let variant = Fingerprint::of(MANIFEST.as_bytes());
    let mut unknown = setup.input(setup.unsafe_ohm);
    unknown.detector.as_mut().unwrap().falling_v.range = None;
    let mut unsourced = setup.input(setup.unsafe_ohm);
    let Some(EmiTransfer::SharedReturn(path)) = &mut unsourced.transfer else {
        unreachable!()
    };
    path.model_source = None;
    let mut missing = setup.input(setup.unsafe_ohm);
    let Some(EmiTransfer::SharedReturn(path)) = &mut missing.transfer else {
        unreachable!()
    };
    path.intervals.clear();
    for input in [unknown, unsourced, missing] {
        let report = stand.with_emc(&scheme, |evidence| {
            assert!(matches!(
                evidence
                    .record_emi_reset(
                        &scheme.compiled,
                        &scheme.modes,
                        "сброс",
                        variant,
                        input,
                        None,
                        None
                    )
                    .unwrap(),
                Analysis::NotAnalyzed { .. }
            ));
        });
        finding(&report, "emi_reset", variant, Outcome::NotAnalyzed);
    }
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        stand.with_emc(&scheme, |evidence| {
            assert!(
                evidence
                    .record_emi_reset(
                        &scheme.compiled,
                        &scheme.modes,
                        "сброс",
                        variant,
                        setup.input(invalid),
                        None,
                        None,
                    )
                    .is_err()
            );
            assert!(
                evidence
                    .findings()
                    .iter()
                    .all(|item| item.code != "emi_reset")
            );
        });
    }
}

#[test]
fn отчёт_cli_оставляет_проверку_без_журналов_открытой() {
    let setup = acceptance().reset;
    let scheme = setup.scheme();
    let stand = ElectricalStand::new("t69-cli");
    let catalog = data_dir().join(&setup.catalog);
    let old = stand.cli(&scheme, &catalog, None);
    assert!(old["assemblies"]["блок"].get("emc_checks").is_none());
    let profile = setup.profile();
    let report = stand.cli(&scheme, &catalog, Some(&profile));
    assert_eq!(report, stand.cli(&scheme, &catalog, Some(&profile)));
    assert_eq!(report["simulation"], "NotProven");
    let checks = &report["assemblies"]["блок"]["emc_checks"];
    assert_eq!(checks.as_object().unwrap().len(), 2);
    for code in ["esd_event", "emi_reset"] {
        let row = &checks[code];
        assert_eq!(row["status"], "not_analyzed");
        assert_eq!(
            row["profile"],
            serde_json::to_value(Fingerprint::of(profile.as_bytes())).unwrap()
        );
        assert!(!row["required_measurement"].as_str().unwrap().is_empty());
        let findings: Vec<_> = report["findings"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|item| item["code"] == code)
            .collect();
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0]["outcome"], "not_analyzed");
    }
    let reset = &checks["emi_reset"];
    assert_eq!(reset["observation"], Value::Null);
    assert_eq!(reset["bench_criterion"], Value::Null);
    close(
        reset["voltage_margin_v"][1].as_f64().unwrap(),
        -0.2,
        ROUNDING,
    );
    assert_eq!(reset["dwell_margin_ns"][1], "-3000");
}

#[test]
fn спектр_rl_комплексным_делителем_и_ngspice() {
    let data = data_dir();
    let spectrum = acceptance().spectrum;
    assert!(
        data.join(spectrum.reference).is_file(),
        "нет полевого стенда Т6.5"
    );
    let solver =
        ngspice::find_ngspice().expect("NotAnalyzed: для независимого спектра нужен ngspice");
    let output = Command::new(solver)
        .arg("-b")
        .arg(data.join(spectrum.rl_deck))
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    let rows: Vec<[f64; 3]> = stdout
        .lines()
        .filter_map(|line| {
            let mut cells = line.split_whitespace();
            cells.next()?.parse::<usize>().ok()?;
            let row = std::array::from_fn(|_| {
                cells
                    .next()
                    .unwrap()
                    .trim_end_matches(',')
                    .parse::<f64>()
                    .unwrap()
            });
            assert!(cells.next().is_none());
            assert!(row.iter().all(|value| value.is_finite()));
            Some(row)
        })
        .collect();
    // Т6.5: геометрия 24×16 мм, r=1 мм, L по формуле Розы — Коэна.
    // Сам netlist использует независимую квадратуру Неймана 35,738699160 нГн.
    // Для разности L около 0,13 фГн и печати ngspice достаточны 2 нВ/В.
    const INDUCTANCE_H: f64 = 35.738_699_289e-9;
    const TOLERANCE: f64 = 2e-9;
    let frequencies = [100e6, 150e6, 200e6, 250e6, 300e6]; // Полоса и сетка петли Т6.5.
    assert_eq!(rows.len(), frequencies.len());
    let mut max_error: f64 = 0.0;
    for ([frequency, re, im], expected) in rows.into_iter().zip(frequencies) {
        assert_eq!(frequency, expected);
        let x = std::f64::consts::TAU * frequency * INDUCTANCE_H;
        let r = 25.0_f64; // Два последовательных окончания по 25 Ом в стенде Т6.5.
        let denominator = (2.0 * r).powi(2) + x.powi(2);
        let exact_re = r * (2.0 * r) / denominator;
        let exact_im = -r * x / denominator;
        max_error = max_error.max((exact_re - re).hypot(exact_im - im));
    }
    assert!(max_error <= TOLERANCE, "невязка {max_error:e} В/В");
    println!("RL: максимум комплексной ошибки {max_error:e} В/В по 5 частотам");
}
