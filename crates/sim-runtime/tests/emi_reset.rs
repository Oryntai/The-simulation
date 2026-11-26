use std::{collections::BTreeMap, path::PathBuf};

use sim_boards::{BoardOutput, ResetCause};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        Endpoint, Parameter, PinState, ResetProfile, Scope,
        emi_reset::{
            EmiBand, EmiCriterion, EmiInput, EmiInterval, EmiOutcome, EmiPacketProbe, EmiReturn,
            EmiSourceKind, EmiTransfer, EmiTrial, prepare_emi_reset,
        },
        sim_circuit::emc::loop_model::{LumpedDomain, SeriesRl},
    },
};
use sim_runtime::{
    LogKind, Runtime, Status,
    control::{self, ControlAssets, ControlConfig, ControlRecording, EventKind, RunSpec},
    electrical::{BoardConnection, BoardCurrents, CircuitModel, ElectricalSetup, SourceStep},
};
use sim_signal::{Rate, Time};
use sim_world::{CompiledWorld, Provenance};

// Стенд: VDD = 3,3 В, вход сброса в покое 3 В, пороги 2 и 2,5 В.
// Общий возврат 0,6/0,4 Ом при 2 А даёт 1,8/2,2 В, запас ∓0,2 В.
// 11–22 мкс, задержки 8/3 мкс, поэтому фронты на 19 и 25 мкс.
// Основной шаг 10 мкс, обмен платы 20 мкс. Все проверяемые границы между ними.
const US: u64 = 1_000;
const END: Time = Time(60 * US);
const BOARD: &str = "блок";

fn source() -> Provenance {
    Provenance::Derived("синтетическая схема стенда".into())
}

fn point(nominal: f64) -> SourcedRange {
    SourcedRange {
        min: nominal,
        nominal,
        max: nominal,
        provenance: source(),
    }
}

fn parameter(value: f64) -> Parameter {
    Parameter {
        range: Some(point(value)),
        measurement: "замерить на стенде".into(),
    }
}

fn detector() -> ResetProfile {
    ResetProfile {
        falling_v: parameter(2.0),
        rising_v: parameter(2.5),
        assert_delay_s: parameter(Time(8 * US).as_secs()),
        release_delay_s: parameter(Time(3 * US).as_secs()),
    }
}

fn trial(delta: f64, until: Time, board: &str) -> EmiTrial {
    let input = EmiInput {
        scope: Scope::Reference,
        source: "коммутация".into(),
        source_kind: EmiSourceKind::Motor,
        board: board.into(),
        input: Endpoint {
            component: board.into(),
            pin: "сброс".into(),
        },
        reference: Endpoint {
            component: board.into(),
            pin: "опора".into(),
        },
        binding: Some(source()),
        start: Time(11 * US),
        end: until,
        bias_v: parameter(3.0),
        detector: Some(detector()),
        transfer: Some(EmiTransfer::SharedReturn(EmiReturn {
            geometry: "резистивный общий провод синтетического стенда".into(),
            // Область эквивалента из постановки стенда: 0,1 м при фазовой
            // скорости 2e8 м/с и доле длины волны 0,01, до 2 МГц.
            domain: LumpedDomain {
                max_frequency_hz: 2e6,
                longest_path_m: 0.1,
                min_wave_speed_m_s: 2e8,
                max_wavelength_fraction: 0.01,
            },
            rl: SeriesRl {
                resistance_ohm: -delta / 2.0,
                inductance_h: 0.0,
            },
            highest_frequency_hz: 1e6,
            intervals: vec![EmiInterval {
                end: until,
                value: (EmiBand::point(2.0), EmiBand::point(0.0)),
            }],
            model_source: Some(source()),
            remainder_v: parameter(0.0),
        })),
    };
    let Analysis::Analyzed { value, .. } = prepare_emi_reset(input).unwrap() else {
        panic!("нет огибающей испытания");
    };
    value
}

// Независимый источник постоянной шины. Ток платы течёт в идеальный возврат,
// изменение фазы платы не влияет на напряжение.
#[derive(Clone)]
struct Supply {
    voltage: f64,
    next: f64,
    current: f64,
}

impl CircuitModel for Supply {
    fn validate_board(&self, board: &BoardConnection) -> Result<(), String> {
        if (&*board.plus, &*board.minus, &*board.current_source) != ("p", "g", "load") {
            return Err("контакты стенда не совпали".into());
        }
        Ok(())
    }
    fn advance_to(&mut self, _: Time) -> Result<(), String> {
        self.voltage = self.next;
        Ok(())
    }
    fn voltage(&self, plus: &str, minus: &str) -> Result<f64, String> {
        if (plus, minus) != ("p", "g") {
            return Err("неизвестный узел".into());
        }
        Ok(self.voltage)
    }
    fn current(&self, name: &str) -> Result<f64, String> {
        if name != "load" {
            return Err("неизвестная ветвь".into());
        }
        Ok(self.current)
    }
    fn set_source_step(&mut self, name: &str, value: f64) -> Result<(), String> {
        match name {
            "load" => self.current = value,
            "supply" => self.next = value,
            _ => return Err("неизвестная ветвь".into()),
        }
        Ok(())
    }
    fn set_pin(&mut self, _: &str, _: PinState) -> Result<(), String> {
        Err("в установке нет выходов в цепь".into())
    }
}

struct Assets;
impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        match name {
            "образ" => Ok(include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec()),
            // Схема конфигурации vmcu_ports: баннер R, watchdog выключен, PWM 1 кГц.
            "данные" => Ok([vec![b'R', 0, 0, 0], 1000_u32.to_le_bytes().to_vec()].concat()),
            _ => Err("нет ресурса".into()),
        }
    }
    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        Err("в стенде используется образ платы".into())
    }
}

fn unloaded(sag: bool) -> Runtime {
    let mut rt = Runtime::load(CompiledWorld {
        name: BOARD.into(),
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
    rt.register_observation("уровень стенда", "V").unwrap();
    let sources = if sag {
        // Независимое обесточивание начинается на правой стороне 10 мкс.
        vec![SourceStep {
            at: Time(10 * US),
            source: "supply".into(),
            value: 0.0,
        }]
    } else {
        vec![]
    };
    rt.install_electrical(
        ElectricalSetup {
            max_step: Time(10 * US),
            scope: Scope::Reference,
            sources,
            probes: BTreeMap::new(),
            boards: vec![BoardConnection {
                board: BOARD.into(),
                plus: "p".into(),
                minus: "g".into(),
                current_source: "load".into(),
                reset: detector(),
                pins: vec![],
                initially_in_reset: false,
                boot_duration_s: parameter(0.0),
                // Идеальная шина стенда, потребление задано одинаковым во всех фазах.
                currents: BoardCurrents {
                    unpowered_a: point(0.0),
                    reset_a: point(0.0),
                    boot_a: point(0.0),
                    working_a: point(0.0),
                },
            }],
        },
        b"emi-stand",
        Supply {
            voltage: 3.3,
            next: 3.3,
            current: 0.0,
        },
    )
    .unwrap();
    rt
}

fn build(
    case: EmiTrial,
    coupled: bool,
    sag: bool,
    replay: Option<ControlRecording>,
) -> Result<Runtime, String> {
    let mut rt = unloaded(sag);
    rt.bind_emi_reset(case, coupled)?;
    let config = ControlConfig::parse(
        r#"
version=1
[[board]]
name='блок'
catalog='vmcu-c3'
image='образ'
config='данные'
rate_hz=50000
source='стенд'
[[board_input]]
name='постоянный вход образа'
board='блок'
signal='уровень стенда'
unit='V'
kind='adc'
port=0
max_age_ns=60000
scale=1
offset=0
fallback=1.65
source='половина шины 3,3 В для чтения АЦП тестовым образом'
[[packet]]
name='входящий'
kind='uart'
to_board='блок'
to_port=0
delay_ns=1000
capacity=4
max_age_ns=16000
source='стенд'
[[transmission]]
route='входящий'
at_ns=18000
bytes=[42]
source='стенд'
"#,
    )?;
    let spec = RunSpec {
        world: b"block",
        catalog: b"stand",
        task: b"emi",
        criteria: b"reset",
        mode: "стенд",
        horizon: END,
        model_ports: &[],
    };
    match replay {
        Some(recording) => control::install_replay(&mut rt, &config, &Assets, spec, recording)?,
        None => control::install(&mut rt, &config, &Assets, spec)?,
    }
    Ok(rt)
}

fn go(rt: &mut Runtime, at: Time) {
    assert_eq!(
        rt.run_until(at).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
}

fn resets(rt: &Runtime) -> Vec<(Time, bool)> {
    control::run(rt)
        .unwrap()
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::ElectricalReset { asserted, .. } => Some((event.at, *asserted)),
            _ => None,
        })
        .collect()
}

#[test]
fn помеха_на_адресованном_входе_перезапускает_плату_и_срывает_доставку() {
    let case = trial(-1.2, Time(22 * US), BOARD);
    let mut active = build(case.clone(), true, false, None).unwrap();
    let mut quiet = build(case, false, false, None).unwrap();
    go(&mut active, END);
    go(&mut quiet, END);
    assert_eq!(
        resets(&active),
        [(Time(19 * US), true), (Time(25 * US), false)]
    );
    assert!(resets(&quiet).is_empty());
    let run = control::run(&active).unwrap();
    assert!(run.events().iter().any(|event| matches!(&event.kind,
        EventKind::BoardOutput { event, produced_at, .. }
        if event.output == BoardOutput::Reset(ResetCause::Brownout) && *produced_at == Time(19 * US)
    )));
    assert!(run.events().iter().any(|event| matches!(&event.kind,
        EventKind::BoardOutput { event, produced_at, .. }
        if matches!(event.output, BoardOutput::Uart { byte: b'R', .. }) && *produced_at > Time(25 * US)
    )));
    assert!(run.events().iter().all(|event| match &event.kind {
        EventKind::ElectricalRail { voltage_v, .. } => *voltage_v == 3.3,
        _ => true,
    }));
    let packet = EmiPacketProbe {
        route: "входящий".into(),
        sent_at: Time(18 * US),
        // Требование стенда: доставка до 24 мкс, до отпускания сброса.
        deadline: Time(24 * US),
        source: Some(source()),
    };
    let evidence = active.compare_emi_reset(&quiet, Some(&packet)).unwrap();
    assert_eq!(evidence.outcome, EmiOutcome::Fail { at: Time(19 * US) });
    let delivery = evidence.packet.unwrap();
    assert_eq!(delivery.baseline_received_at, Time(20 * US));
    assert_eq!(delivery.disturbed_received_at, None);
    assert!(run.events().iter().any(|event| matches!(&event.kind,
        EventKind::Rejected { path, id: Some(id), reason: control::Rejection::Disconnected }
        if path == "входящий" && *id == delivery.id && event.at == Time(20 * US)
    )));
    let mut unspecified = packet;
    unspecified.source = None;
    let open = active
        .compare_emi_reset(&quiet, Some(&unspecified))
        .unwrap();
    assert_eq!(open.outcome, evidence.outcome);
    assert!(open.packet.is_none() && open.packet_open.is_some());
    let row = |at| {
        active
            .log()
            .entries()
            .iter()
            .find(|row| row.at == at && row.kind == LogKind::Truth)
            .unwrap()
            .value
    };
    // Закон Ома независимо от выдержек детектора.
    assert!((row(Time(11 * US)) - (3.0_f64 - 0.6 * 2.0)).abs() <= f64::EPSILON);
    assert_eq!(row(Time(22 * US)), 3.0);
}

#[test]
fn исправленная_пара_короткий_фронт_и_обесточенный_контроль() {
    for (delta, end, sag, criterion) in [
        (-0.8, 22, false, Some(EmiCriterion::WithinBench)),
        (-1.2, 18, false, Some(EmiCriterion::WithinBench)),
        (-1.2, 19, false, None), // Нулевой запас при совпадении фронта и срока выдержки.
        (-1.2, 22, true, None),
    ] {
        let case = trial(delta, Time(end * US), BOARD);
        let mut active = build(case.clone(), true, sag, None).unwrap();
        let mut quiet = build(case, false, sag, None).unwrap();
        go(&mut active, END);
        go(&mut quiet, END);
        assert_eq!(resets(&active), resets(&quiet));
        let evidence = active.compare_emi_reset(&quiet, None).unwrap();
        assert_eq!(evidence.criterion, criterion);
        assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
    }
}

#[test]
fn повтор_проверяет_воздействие_и_сохраняет_выдержку_из_снимка() {
    let case = trial(-1.2, Time(22 * US), BOARD);
    let mut run = build(case.clone(), true, false, None).unwrap();
    go(&mut run, Time(15 * US));
    let snapshot = run.snapshot();
    let mut fork = run.fork().unwrap();
    go(&mut run, END);
    let live = control::run(&run).unwrap();
    let fingerprint = live.fingerprint();
    let expected = live.recording();
    assert!(build(case.clone(), false, false, Some(expected.clone())).is_err());
    let mut replay = build(case, true, false, Some(expected.clone())).unwrap();
    run.restore(&snapshot).unwrap();
    for runtime in [&mut run, &mut fork, &mut replay] {
        go(runtime, END);
        assert_eq!(control::run(runtime).unwrap().fingerprint(), fingerprint);
    }
    control::verify_replay(&replay).unwrap();
}

#[test]
fn неизвестная_плата_и_несовпадающие_опыты_не_дают_доказательство() {
    let mut rt = unloaded(false);
    assert!(
        rt.bind_emi_reset(trial(-1.2, Time(22 * US), "нет платы"), true)
            .is_err()
    );
    let mut active = build(trial(-1.2, Time(22 * US), BOARD), true, false, None).unwrap();
    let mut quiet = build(trial(-0.8, Time(22 * US), BOARD), false, false, None).unwrap();
    go(&mut active, END);
    go(&mut quiet, END);
    assert!(active.compare_emi_reset(&quiet, None).is_err());
}

#[test]
fn неполная_запись_не_закрывает_критерий() {
    let case = trial(-0.8, Time(22 * US), BOARD);
    let mut active = build(case.clone(), true, false, None).unwrap();
    let mut quiet = build(case, false, false, None).unwrap();
    go(&mut active, Time(15 * US));
    go(&mut quiet, END);
    let evidence = active.compare_emi_reset(&quiet, None).unwrap();
    assert_eq!(evidence.criterion, None);
    assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
}
