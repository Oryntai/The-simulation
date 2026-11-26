use std::{collections::BTreeMap, path::PathBuf};

use sim_boards::{BoardOutput, ResetCause};
use sim_process::{
    SourcedRange,
    schematic::{
        BootStage, ClampPath, Parameter, PinModel, PinProfile, PinState, Pull, ResetProfile, Scope,
        SourcedPinState,
    },
};
use sim_runtime::{
    Forces, LogKind, Object, Runtime, Status, Truth, TruthId,
    control::{self, ControlAssets, ControlConfig, ControlRecording, EventKind, RunSpec},
    electrical::{
        BoardConnection, BoardCurrents, CircuitModel, ElectricalProbe, ElectricalSetup,
        GpioConnection, LoadConnection, SourceStep,
    },
};
use sim_signal::{ChannelId, Command, Rate, Rng, Time};
use sim_world::{CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Provenance};

// Преобразование единиц SI. Все электрические числа ниже задают синтетический стенд.
const US: u64 = 1_000;
const END: Time = Time(120 * US);
const BOARD: &str = "блок";

fn source() -> Provenance {
    Provenance::Measured("заданные параметры синтетического стенда".into())
}

fn point(value: f64) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source(),
    }
}

fn parameter(value: f64) -> Parameter {
    Parameter {
        range: Some(point(value)),
        measurement: "снять соответственный ряд на стенде".into(),
    }
}

fn pin(state: PinState) -> SourcedPinState {
    SourcedPinState {
        state,
        source: source(),
    }
}

fn setup() -> ElectricalSetup {
    // Стенд: ЭДС 3,5/1,7 В, последовательное R 1 Ом, общий потенциал 1,2 В.
    // Плата: токи 0/50/150/200 мА в четырёх фазах. Пороги 2 и 2,8 В.
    // Длительности подавления 10 и 15 мкс, один этап загрузки 4 мкс.
    let switches = [(10, 1.7), (12, 3.5), (40, 1.7), (80, 3.5)];
    ElectricalSetup {
        max_step: Time(2 * US),
        scope: Scope::Reference,
        boards: vec![BoardConnection {
            board: BOARD.into(),
            plus: "vdd".into(),
            minus: "vss".into(),
            current_source: "i_board".into(),
            currents: BoardCurrents {
                unpowered_a: point(0.0),
                reset_a: point(0.05),
                boot_a: point(0.15),
                working_a: point(0.2),
            },
            reset: ResetProfile {
                falling_v: parameter(2.0),
                rising_v: parameter(2.8),
                assert_delay_s: parameter(Time(10 * US).as_secs()),
                release_delay_s: parameter(Time(15 * US).as_secs()),
            },
            initially_in_reset: false,
            boot_duration_s: parameter(Time(4 * US).as_secs()),
            pins: vec![GpioConnection {
                // GPIO3 подтверждён картой и ассемблером тестового образа vmcu_ports.
                pin: 3,
                contact: "затвор".into(),
                profile: PinProfile {
                    // В схеме стенда резисторы драйвера 10 Ом, внутренние подтяжки
                    // 1 кОм, утечки 1 МОм. Защитные ветви в стенде отсутствуют.
                    model: PinModel {
                        high_ohm: parameter(10.0),
                        low_ohm: parameter(10.0),
                        pull_up_ohm: parameter(1_000.0),
                        pull_down_ohm: parameter(1_000.0),
                        input_ohm: parameter(1_000_000.0),
                        high_z_ohm: parameter(1_000_000.0),
                        upper_clamp: ClampPath::Absent(source()),
                        lower_clamp: ClampPath::Absent(source()),
                    },
                    unpowered: Some(pin(PinState::HighZ)),
                    reset: Some(pin(PinState::HighZ)),
                    boot: vec![BootStage {
                        duration_s: parameter(Time(4 * US).as_secs()),
                        pin: pin(PinState::InputPull(Pull::Down)),
                    }],
                },
            }],
        }],
        sources: switches
            .into_iter()
            .map(|(us, value)| SourceStep {
                at: Time(us * US),
                source: "v_source".into(),
                value,
            })
            .collect(),
        probes: BTreeMap::from([
            (
                "ток ключа".into(),
                ElectricalProbe::Current {
                    branch: "ключ".into(),
                },
            ),
            (
                "шина платы".into(),
                ElectricalProbe::Voltage {
                    plus: "vdd".into(),
                    minus: "vss".into(),
                },
            ),
        ]),
    }
}

/// Независимый эталон связи на резистивном стенде с кусочно-постоянными входами.
/// Параллельные нагрузки — ток платы и управляемая ветвь 10 Ом. При разомкнутом
/// ключе ток ветви равен нулю; порог затвора стенда 1 В. Подтяжка затвора 10 кОм.
#[derive(Clone)]
struct OhmStand {
    source: f64,
    load: f64,
    pin: PinState,
    vdd: f64,
    branch_a: f64,
    board_a: f64,
    next_load_a: f64,
    load_a: f64,
}

impl Default for OhmStand {
    fn default() -> Self {
        Self {
            source: 3.5,
            vdd: 3.5,
            load: 0.0,
            pin: PinState::HighZ,
            branch_a: 0.0,
            board_a: 0.0,
            next_load_a: 0.0,
            load_a: 0.0,
        }
    }
}

impl CircuitModel for OhmStand {
    fn validate_board(&self, board: &BoardConnection) -> Result<(), String> {
        if (&*board.plus, &*board.minus, &*board.current_source) != ("vdd", "vss", "i_board")
            || board.pins.iter().any(|pin| pin.contact != "затвор")
        {
            return Err("разрыв привязки стенда".into());
        }
        Ok(())
    }

    fn validate_current_source(&self, plus: &str, minus: &str, branch: &str) -> Result<(), String> {
        if (plus, minus, branch) != ("vdd", "vss", "i_load") {
            return Err("неверная ориентация нагрузки стенда".into());
        }
        Ok(())
    }

    fn advance_to(&mut self, at: Time) -> Result<(), String> {
        if at == Time::ZERO {
            return Ok(());
        }
        // Учитываем ток делителя затвора и нагрузку после замыкания.
        let gate_r = match self.pin {
            PinState::High => Some(10.0 + 10_000.0),
            PinState::InputPull(Pull::Up) => Some(1_000.0 + 10_000.0),
            _ => None,
        };
        let gate_g = gate_r.map_or(0.0, f64::recip);
        let load_g = if gate_r
            .is_some_and(|r| (self.source - self.load) / (1.0 + gate_g) * 10_000.0 / r > 1.0)
        {
            0.1
        } else {
            0.0
        };
        self.vdd = (self.source - self.load - self.next_load_a) / (1.0 + gate_g + load_g);
        self.branch_a = self.vdd * load_g;
        self.board_a = self.load;
        self.load_a = self.next_load_a;
        Ok(())
    }

    fn voltage(&self, plus: &str, minus: &str) -> Result<f64, String> {
        let node = |name| match name {
            "vss" => Ok(1.2),
            "vdd" => Ok(self.vdd + 1.2),
            _ => Err("узел стенда не найден"),
        };
        Ok(node(plus)? - node(minus)?)
    }

    fn current(&self, branch: &str) -> Result<f64, String> {
        match branch {
            "i_board" => Ok(self.board_a),
            "i_load" => Ok(self.load_a),
            "ключ" => Ok(self.branch_a),
            _ => Err("ветвь стенда не найдена".into()),
        }
    }

    fn set_source_step(&mut self, name: &str, value: f64) -> Result<(), String> {
        match name {
            "v_source" => self.source = value,
            "i_board" => self.load = value,
            "i_load" => self.next_load_a = value,
            _ => return Err("источник стенда не найден".into()),
        }
        Ok(())
    }

    fn set_pin(&mut self, contact: &str, state: PinState) -> Result<(), String> {
        if contact != "затвор" {
            return Err("затвор стенда не найден".into());
        }
        self.pin = state;
        Ok(())
    }
}

struct Assets;

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        match name {
            "образ" => Ok(include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec()),
            // Формат vmcu_ports: ASCII S, выключенный watchdog, PWM 1000 Гц.
            "данные" => Ok([vec![b'S', 0, 0, 0], 1000_u32.to_le_bytes().to_vec()].concat()),
            _ => Err("ресурс стенда не найден".into()),
        }
    }

    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        Err("стенд использует образ платы".into())
    }
}

fn stand_world() -> CompiledWorld {
    // Пустая механическая сцена и шаг 10 мкс. Электрические события находятся и между тактами физики.
    CompiledWorld {
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
    }
}

fn unloaded() -> Runtime {
    let mut rt = Runtime::load(stand_world()).unwrap();
    rt.register_observation("напряжение стенда", "V").unwrap();
    rt
}

fn config(ending: bool) -> ControlConfig {
    let ending = if ending { "true" } else { "false" };
    // Обмен платы каждые 20 мкс; вход 0 включает выход 3, АЦП получает вход установки.
    ControlConfig::parse(&format!(
        r#"
version=1
[[board]]
name='блок'
catalog='vmcu-c3'
image='образ'
config='данные'
rate_hz=50000
end_sample={ending}
source='стенд'
[[board_action]]
id='вход'
board='блок'
at_ns=0
kind='gpio'
pin=0
high=true
source='стенд'
[[board_input]]
name='АЦП'
board='блок'
signal='напряжение стенда'
unit='V'
kind='adc'
port=0
max_age_ns=120000
scale=1
offset=0
fallback=1.65
source='стенд'
"#
    ))
    .unwrap()
}

fn spec() -> RunSpec<'static> {
    RunSpec {
        world: b"block",
        catalog: b"stand",
        task: b"rail",
        criteria: b"reset",
        mode: BOARD,
        horizon: END,
        model_ports: &[],
    }
}

fn build(
    setup: ElectricalSetup,
    key: &[u8],
    replay: Option<ControlRecording>,
) -> Result<Runtime, String> {
    let mut rt = unloaded();
    rt.install_electrical(setup, key, OhmStand::default())?;
    let config = config(false);
    match replay {
        Some(replay) => control::install_replay(&mut rt, &config, &Assets, spec(), replay)?,
        None => control::install(&mut rt, &config, &Assets, spec())?,
    }
    Ok(rt)
}

fn go(rt: &mut Runtime, time: Time) {
    assert_eq!(
        rt.run_until(time).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
}

fn journal(rt: &Runtime) -> Vec<u8> {
    let mut bytes = Vec::new();
    rt.log().write_binary(&mut bytes).unwrap();
    bytes
}

fn mna_dc(emf: f64, board_a: f64, closed: bool) -> (f64, f64) {
    use sim_process::{Analysis, schematic::*};

    // Та же физическая сеть в штатном MNA. Для установившегося положения
    // ключа заменяем его проводящую ветвь сопротивлением нагрузки стенда.
    let element = |kind, nodes: [&str; 2], key: &str, value| Primitive {
        kind,
        nodes: nodes.map(str::to_owned).into(),
        parameters: BTreeMap::from([(key.into(), parameter(value))]),
        continuity: false,
    };
    let mut elements = BTreeMap::from([
        (
            "эдс".into(),
            element(PrimitiveKind::Voltage, ["src", "vss"], "volt", emf),
        ),
        (
            "последовательный".into(),
            element(PrimitiveKind::Resistor, ["src", "vdd"], "ohm", 1.0),
        ),
        (
            "плата".into(),
            element(PrimitiveKind::Current, ["vdd", "vss"], "ampere", board_a),
        ),
    ]);
    if closed {
        elements.insert(
            "делитель".into(),
            element(
                PrimitiveKind::Resistor,
                ["vdd", "vss"],
                "ohm",
                10.0 + 10_000.0,
            ),
        );
        elements.insert(
            "ключ".into(),
            element(PrimitiveKind::Resistor, ["vdd", "vss"], "ohm", 10.0),
        );
    }
    let names = ["vdd", "vss"];
    let part = Part {
        kind: PartKind::Other,
        source: "заданные параметры синтетического стенда".into(),
        pins: names
            .into_iter()
            .map(|name| {
                (
                    name.into(),
                    Pin {
                        contact: Contact {
                            kind: ContactKind::Bare,
                            family: None,
                            key: None,
                        },
                        role: PinRole::Passive,
                    },
                )
            })
            .collect(),
        internal_nodes: ["src".into()].into(),
        elements,
        ratings: BTreeMap::new(),
        model_gap: None,
        reference_only: true,
    };
    let catalog = Catalog::new(BTreeMap::from([(BOARD.into(), part)])).unwrap();
    let scheme = Schematic {
        name: BOARD.into(),
        scope: Scope::Reference,
        nets: names.map(str::to_owned).into(),
        components: BTreeMap::from([(
            BOARD.into(),
            Component {
                part: BOARD.into(),
                nets: names.into_iter().map(|n| (n.into(), n.into())).collect(),
            },
        )]),
        connections: BTreeMap::new(),
        rails: BTreeMap::new(),
        bindings: Vec::new(),
    };
    let dc = compile(scheme, &catalog).unwrap().dc();
    let end = |pin: &str| Endpoint {
        component: BOARD.into(),
        pin: pin.into(),
    };
    let Analysis::Analyzed { value, .. } = dc.voltage(&end("vdd"), &end("vss")) else {
        panic!("MNA не вернул разность клемм стенда");
    };
    let current = if closed {
        dc.branches()
            .iter()
            .find(|row| {
                row.section
                    == Section::Element {
                        component: BOARD.into(),
                        element: "ключ".into(),
                    }
            })
            .unwrap()
            .current_a
    } else {
        0.0 // Разомкнутая ветвь отсутствует в нетлисте DC.
    };
    (value, current)
}

#[test]
fn ток_ключа_следует_за_выводом_и_просадка_перезапускает_образ() {
    let mut rt = build(setup(), b"stand-a", None).unwrap();
    go(&mut rt, END);
    let run = control::run(&rt).unwrap();
    let reset = run
        .events()
        .iter()
        .filter_map(|event| match event.kind {
            EventKind::ElectricalReset { asserted, .. } => Some((event.at.0, asserted)),
            _ => None,
        })
        .collect::<Vec<_>>();
    // Просадка впервые видна на 42 мкс, восстановление на 82 мкс. Короткая
    // просадка из 10–12 мкс отменяет выдержку и не запускает образ повторно.
    assert_eq!(reset, [(52 * US, true), (97 * US, false)]);
    let mut uart = Vec::new();
    let mut brownouts = Vec::new();
    for event in run.events() {
        if let EventKind::BoardOutput {
            event, produced_at, ..
        } = &event.kind
        {
            match event.output {
                BoardOutput::Uart { byte: b'S', .. } => uart.push(*produced_at),
                BoardOutput::Reset(ResetCause::Brownout) => brownouts.push(*produced_at),
                _ => (),
            }
        }
    }
    assert_eq!(brownouts, [Time(52 * US)]);
    assert_eq!(uart.len(), 2);
    assert_eq!(uart[1].0 - uart[0].0, 97 * US);
    assert!(uart[0] < Time(4 * US));
    let pins = run
        .events()
        .iter()
        .filter_map(|event| match event.kind {
            EventKind::ElectricalPin { pin: 3, state, .. } => Some((event.at.0, state)),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(
        pins,
        [
            (0, PinState::InputPull(Pull::Down)),
            (4 * US, PinState::High),
            (52 * US, PinState::HighZ),
            (97 * US, PinState::InputPull(Pull::Down)),
            (101 * US, PinState::High)
        ]
    );

    let id = |name| {
        rt.log()
            .header()
            .truth
            .iter()
            .position(|spec| spec.name == name)
            .unwrap() as u32
    };
    let current = id("ток ключа");
    let voltage = id("шина платы");
    let reading = |id, us| {
        rt.log()
            .entries()
            .iter()
            .find(|row| {
                row.kind == sim_runtime::LogKind::Truth && row.id == id && row.at == Time(us * US)
            })
            .unwrap()
            .value
    };
    // Независимая сверка узлового баланса (3,5 − V) / 1 Ом = 0,2 + I + V/10010.
    let rail = reading(voltage, 6);
    let load = reading(current, 6);
    assert!((load - 0.3).abs() < 0.001); // Разница с 0,3 А вызвана делителем затвора.
    assert!(((3.5 - rail) - (0.2 + load + rail / 10010.0)).abs() < 2e-15);
    assert_eq!(reading(current, 4), 0.0);
    assert!(reading(current, 52) > 0.1);
    assert_eq!(reading(current, 54), 0.0);
    assert!((reading(voltage, 54) - 1.65).abs() < 2e-15);
    assert!((reading(voltage, 97) - 3.45).abs() < 2e-15);
    assert_eq!(reading(current, 101), 0.0);
    assert!(reading(current, 102) > 0.29);
    for (us, emf, board_a, closed) in [
        (6, 3.5, 0.2, true),
        (52, 1.7, 0.2, true),
        (54, 1.7, 0.05, false),
        (97, 3.5, 0.05, false),
        (102, 3.5, 0.2, true),
    ] {
        let (v, a) = mna_dc(emf, board_a, closed);
        // Абсолютный допуск 10 мВ из §5 электрического ТЗ. Для ветви 10 Ом
        // он соответствует 1 мА по закону Ома.
        assert!((reading(voltage, us) - v).abs() < 0.01, "{us} мкс");
        assert!((reading(current, us) - a).abs() < 0.001, "{us} мкс");
    }
    assert!(
        rt.simplifications()
            .iter()
            .any(|item| item.blocks_proof && item.subject.contains("электрическая"))
    );
}

#[test]
fn снимок_ветка_повтор_и_независимый_запуск_дают_одинаковый_журнал() {
    let mut rt = build(setup(), b"stand-a", None).unwrap();
    go(&mut rt, Time(49 * US)); // Выдержка просадки уже отсчитывается, сброс впереди.
    let snapshot = rt.snapshot();
    let mut branch = rt.fork().unwrap();
    go(&mut rt, END);
    let original = control::run(&rt).unwrap();
    let hash = original.fingerprint();
    let bytes = journal(&rt);
    let recording = original.recording();
    let mut replay = build(setup(), b"stand-a", Some(recording.clone())).unwrap();
    let mut fresh = build(setup(), b"stand-a", None).unwrap();
    rt.restore(&snapshot).unwrap();
    for run in [&mut rt, &mut branch, &mut replay, &mut fresh] {
        go(run, END);
        assert_eq!(journal(run), bytes);
        assert_eq!(control::run(run).unwrap().fingerprint(), hash);
    }
    control::verify_replay(&replay).unwrap();
    assert!(build(setup(), b"stand-b", Some(recording)).is_err());
}

#[test]
fn повтор_обрыва_между_тактами_не_добавляет_отсчёт() {
    let stop = Time(49 * US);
    let mut rt = build(setup(), b"stand-a", None).unwrap();
    go(&mut rt, stop);
    let recording = control::run(&rt).unwrap().recording();
    let mut replay = build(setup(), b"stand-a", Some(recording)).unwrap();
    go(&mut replay, stop);
    control::verify_replay(&replay).unwrap();
    assert_eq!(journal(&rt), journal(&replay));
}

#[test]
fn неизвестный_порог_конфликт_и_nan_отклоняются_до_прогона() {
    let mut missing = setup();
    missing.boards[0].reset.falling_v.range = None;
    assert!(build(missing, b"stand-a", None).is_err());
    let mut nan = setup();
    nan.sources[0].value = f64::NAN;
    assert!(build(nan, b"stand-a", None).is_err());
    let mut collision = setup();
    collision.sources[0].source = "i_board".into();
    assert!(build(collision, b"stand-a", None).is_err());
    let mut rt = unloaded();
    rt.install_electrical(setup(), b"stand-a", OhmStand::default())
        .unwrap();
    assert!(rt.run_until(END).is_err());
    assert!(control::install(&mut rt, &config(true), &Assets, spec()).is_err());
    control::install(&mut rt, &config(false), &Assets, spec()).unwrap();
    go(&mut rt, END);
}

#[test]
fn нулевая_начальная_шина_не_даёт_прошивке_проработать_выдержку() {
    let mut rt = unloaded();
    rt.install_electrical(
        setup(),
        b"stand-off",
        OhmStand {
            source: 0.0,
            vdd: 0.0,
            ..OhmStand::default()
        },
    )
    .unwrap();
    control::install(&mut rt, &config(false), &Assets, spec()).unwrap();
    // 10 мкс ещё не истекли; выключенная шина сразу требует обесточенного GPIO.
    go(&mut rt, Time(8 * US));
    let run = control::run(&rt).unwrap();
    let events = run.events();
    assert!(events.iter().any(|event| event.at == Time::ZERO
        && matches!(
            event.kind,
            EventKind::ElectricalPin {
                state: PinState::HighZ,
                ..
            }
        )));
    assert!(events.iter().any(|event| event.at == Time::ZERO
        && matches!(
            event.kind,
            EventKind::ElectricalReset { asserted: true, .. }
        )));
    assert!(!events.iter().any(|event| matches!(event.kind, EventKind::BoardOutput { ref event, .. } if matches!(event.output, BoardOutput::Uart { .. }))));
}

#[derive(Clone)]
struct Load {
    input: ChannelId,
    voltage: TruthId,
    current: TruthId,
    missing_at: Option<Time>,
    tick: usize,
}

impl Object for Load {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        let voltage = command
            .targets
            .iter()
            .find_map(|&(channel, value)| (channel == self.input).then_some(value))
            .ok_or("нет напряжения нагрузки")?;
        out.publish(self.voltage, voltage);
        // Профиль стенда: потребление 0,5 А, возврат 0,25 А, затем разрыв.
        let profile = [0.5, -0.25, 0.0];
        if self.missing_at != Some(truth.time()) {
            out.publish(self.current, profile[self.tick % profile.len()]);
        }
        self.tick += 1;
        Ok(())
    }
}

fn load_stand(missing_at: Option<Time>, replay: Option<ControlRecording>) -> Runtime {
    // Шаг каретки 10 мкс, обмен со схемой каждые 7 мкс. Механическое усилие
    // не подаётся: вход нагрузки используется только для напряжения шины.
    let mut rt = Runtime::load(CompiledWorld {
        name: "каретка".into(),
        mjcf: r#"<mujoco><option timestep="0.00001" gravity="0 0 0"/>
            <worldbody><body name="блок">
                <joint name="ход" type="slide" axis="1 0 0"/>
                <geom type="sphere" size="0.1" mass="1"/>
            </body></worldbody></mujoco>"#
            .into(),
        joints: vec![CompiledJoint {
            name: "ход".into(),
            owner: "каретка".into(),
            kind: CompiledJointKind::Slide,
        }],
        efforts: vec![EffortChannel {
            name: "питание".into(),
            joint: "ход".into(),
            dof: 0,
        }],
        ..stand_world()
    })
    .unwrap();
    let input = rt.channel("питание").unwrap();
    let voltage = rt.register_truth("напряжение нагрузки", "V").unwrap();
    let current = rt.register_truth("следующий ток нагрузки", "A").unwrap();
    rt.register_object(
        "нагрузка",
        rt.physics_rate(),
        &[input],
        Load {
            input,
            voltage,
            current,
            missing_at,
            tick: 0,
        },
    )
    .unwrap();
    rt.install_electrical(
        ElectricalSetup {
            max_step: Time(7 * US),
            scope: Scope::Reference,
            boards: vec![],
            sources: vec![],
            probes: BTreeMap::from([(
                "ток цепи".into(),
                ElectricalProbe::Current {
                    branch: "i_load".into(),
                },
            )]),
        },
        b"stand-load",
        OhmStand::default(),
    )
    .unwrap();
    rt.bind_electrical_loads(vec![LoadConnection {
        plus: "vdd".into(),
        minus: "vss".into(),
        current_source: "i_load".into(),
        voltage_input: input,
        next_current: current,
    }])
    .unwrap();
    let config = ControlConfig::parse("version=1").unwrap();
    match replay {
        Some(recording) => control::install_replay(&mut rt, &config, &Assets, spec(), recording),
        None => control::install(&mut rt, &config, &Assets, spec()),
    }
    .unwrap();
    rt
}

fn load_reading(rt: &Runtime, name: &str, us: u64) -> f64 {
    let id = rt
        .log()
        .header()
        .truth
        .iter()
        .position(|spec| spec.name == name)
        .unwrap() as u32;
    let mut rows = rt
        .log()
        .entries()
        .iter()
        .filter(|row| row.kind == LogKind::Truth && row.id == id && row.at == Time(us * US));
    let row = rows
        .next()
        .unwrap_or_else(|| panic!("нет «{name}» на {us} мкс"));
    assert!(row.valid && rows.next().is_none());
    row.value
}

#[test]
fn общий_такт_удерживает_знаковый_ток_во_всех_путях() {
    let mut whole = load_stand(None, None);
    go(&mut whole, END);
    assert_eq!(whole.time(), END);

    let mut electrical = load_stand(None, None);
    assert_eq!(
        electrical.run_electrical_until(END).unwrap(),
        Status::Running
    );
    let mut stepped = load_stand(None, None);
    for index in 1..=whole.steps() {
        assert_eq!(stepped.step().unwrap(), Status::Running);
        assert_eq!(stepped.time(), stepped.physics_rate().tick(index));
    }
    for run in [&whole, &electrical, &stepped] {
        // Ток на границе относится к закрытому интервалу. Фронты удержания
        // появляются и на внефизических отсчётах сетки 7 мкс.
        for (us, expected) in [
            (0, 0.0),
            (7, 0.5),
            (10, 0.5),
            (14, -0.25),
            (20, -0.25),
            (21, 0.0),
        ] {
            assert_eq!(load_reading(run, "ток цепи", us), expected, "{us} мкс");
        }
        // Независимый баланс стенда: V = 3,5 В − I × 1 Ом.
        // Четыре epsilon учитывают сдвиг опоры на 1,2 В и обратное вычитание.
        for (us, expected) in [(0, 3.5_f64), (10, 3.0), (20, 3.75), (30, 3.5)] {
            let voltage = load_reading(run, "напряжение нагрузки", us);
            assert!((voltage - expected).abs() <= 4.0 * f64::EPSILON * expected);
        }
    }
    assert_eq!(journal(&stepped), journal(&whole));
    assert_eq!(journal(&electrical), journal(&whole));
}

#[test]
fn отсутствующий_ток_останавливает_опыт_на_первом_пропуске() {
    for missing_at in [Time::ZERO, Time(10 * US)] {
        let mut whole = load_stand(Some(missing_at), None);
        let mut electrical = load_stand(Some(missing_at), None);
        let mut stepped = load_stand(Some(missing_at), None);
        assert_eq!(whole.run_until(END).unwrap(), Status::ComponentFailure);
        assert_eq!(
            electrical.run_electrical_until(END).unwrap(),
            Status::ComponentFailure
        );
        assert_eq!(stepped.step().unwrap(), Status::ComponentFailure);
        for run in [&whole, &electrical, &stepped] {
            assert_eq!(run.time(), missing_at);
            assert_eq!(
                run.failure().unwrap(),
                format!(
                    "«i_load»: нагрузка не опубликовала ток шины на такте {} нс",
                    missing_at.0
                )
            );
            assert!(run.log().entries().iter().all(|row| row.at <= missing_at));
            if missing_at != Time::ZERO {
                assert_eq!(load_reading(run, "ток цепи", 10), 0.5);
            }
        }
        assert_eq!(journal(&stepped), journal(&whole));
        assert_eq!(journal(&electrical), journal(&whole));
    }
}

#[test]
fn связанная_нагрузка_сохраняет_журнал_после_снимка_и_при_повторе() {
    let mut original = load_stand(None, None);
    // На 17 мкс в схеме удерживается обратный ток с такта 10 мкс.
    go(&mut original, Time(17 * US));
    let snapshot = original.try_snapshot().unwrap();
    let mut fork = original.fork().unwrap();
    go(&mut original, END);
    let bytes = journal(&original);
    let recording = control::run(&original).unwrap().recording();
    let mut replay = load_stand(None, Some(recording));

    original.restore(&snapshot).unwrap();
    for run in [&mut original, &mut fork, &mut replay] {
        assert_eq!(run.step().unwrap(), Status::Running);
        assert_eq!(run.run_electrical_until(END).unwrap(), Status::Running);
        assert_eq!(journal(run), bytes);
    }
    control::verify_replay(&replay).unwrap();

    let mut audit = load_stand(None, None);
    audit.audit_snapshot(Time(17 * US), END).unwrap();
    assert_eq!(journal(&audit), bytes);
}
