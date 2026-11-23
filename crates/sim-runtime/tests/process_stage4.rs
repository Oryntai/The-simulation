use std::{collections::BTreeMap, path::PathBuf};

use sim_process::{Analysis, BodyLoad, ProcessError, SourcedRange, attachments::*};
use sim_runtime::{
    Forces, Log, Object, Runtime, Status, Truth,
    control::{self, ControlAssets, ControlConfig, EventKind, RunSpec},
    process::{
        AttachmentAdapter, ProcessFrame, ProcessModel, ProcessOut, RadioEnvironment, RadioPath,
    },
};
use sim_signal::{ChannelId, Command, Rate, Rng, Time};
use sim_world::{
    CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, MountKind, Provenance,
};

const MS: u64 = 1_000_000;
const END: Time = Time(6 * MS);

fn source(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Measured("заданные параметры синтетической каретки".into()),
    }
}

fn exact(x: f64) -> SourcedRange {
    source(x, x, x)
}

fn known<T>(value: T) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: "контрольный расчёт каретки".into(),
    }
}

fn scene() -> CompiledWorld {
    // Две независимые каретки по 2 кг, шаг 1 мс. Связь в общей исходной позе.
    CompiledWorld {
        name: "каретка".into(),
        seed: 31,
        physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco><option gravity="0 0 0" timestep="0.001" integrator="Euler"/>
        <worldbody>
          <body name="а"><joint name="ось-а" type="slide" axis="1 0 0"/>
            <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
          <body name="б"><joint name="ось-б" type="slide" axis="1 0 0"/>
            <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
        </worldbody>
        <equality><weld name="стык" body1="а" body2="б" active="false"/></equality></mujoco>"#
            .into(),
        bodies: vec![],
        joints: ["ось-а", "ось-б"]
            .map(|axis| CompiledJoint {
                name: axis.into(),
                owner: "каретка".into(),
                kind: CompiledJointKind::Slide,
            })
            .into(),
        efforts: vec![EffortChannel {
            name: "сила".into(),
            joint: "ось-б".into(),
            dof: 0,
        }],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }
}

struct Assets;
impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        Ok(match name {
            "образ" => include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec(),
            "параметры" => {
                // Формат тестовой прошивки: режим S, без watchdog, ШИМ 1000 Гц.
                [b'S', 0, 0, 0]
                    .into_iter()
                    .chain(1000_u32.to_le_bytes())
                    .collect()
            }
            "слоты" => b"accepts_through_ns,delivers_at_ns\n1000000,2000000\n".to_vec(),
            _ => return Err(format!("нет ресурса {name}")),
        })
    }
    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        Err("нет внешнего процесса".into())
    }
}

fn spec() -> RunSpec<'static> {
    RunSpec {
        world: b"carriage",
        catalog: b"fixture",
        task: b"fixture",
        criteria: b"fixture",
        mode: "каретка",
        horizon: END,
        model_ports: &[],
    }
}

#[derive(Clone)]
struct Layer {
    dry_at: Time,
}

impl ProcessModel for Layer {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        use sim_radio::dielectric::Medium;
        // Синтетическая трасса: 10 см пресной воды до заданной границы.
        let (medium, depth_m) = if frame.next.start < self.dry_at {
            (Medium::FreshWater { t_c: 22.0 }, 0.1)
        } else {
            (Medium::Air, 0.0)
        };
        let mut out = ProcessOut::default();
        out.radio.insert(
            "радиолиния".into(),
            known(RadioEnvironment { medium, depth_m }),
        );
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        self.dry_at.0
    }
}

#[test]
fn мокрая_линия_повторяет_передачу_живой_плате() {
    use sim_radio::timed::Outcome;
    for (dry_at, terminal) in [
        (Time(2 * MS), Outcome::Delivered { rssi_dbm: -60 }),
        (Time(7 * MS), Outcome::Lost),
    ] {
        let mut rt = radio_runtime();
        let config = packet_config("wifi", "attempt_ns=1500000\nloss=0\nattempts=3");
        control::install(&mut rt, &config, &Assets, spec()).unwrap();
        // Испытательный бюджет с заведомо разным исходом в воде и в воздухе.
        rt.bind_process_radio(
            "радиолиния",
            RadioPath {
                frequency_hz: exact(2.4e9),
                sensitivity_dbm: exact(-85.0),
            },
        )
        .unwrap();
        rt.register_process("слой", Layer { dry_at }).unwrap();
        assert_eq!(
            rt.run_until(END).unwrap(),
            Status::Running,
            "{:?}",
            rt.failure()
        );
        let run = control::run(&rt).unwrap();
        let events: Vec<_> = run
            .events()
            .iter()
            .filter_map(|event| match &event.kind {
                EventKind::Packet { route, event } if route == "радиолиния" => {
                    Some(event)
                }
                _ => None,
            })
            .collect();
        assert_eq!(events[0].at, Time(MS / 4 + 3 * MS / 2));
        assert_eq!(events[0].outcome, Outcome::Retry);
        let end = events.last().unwrap();
        assert_eq!(end.outcome, terminal);
        assert_eq!(end.at.0, MS / 4 + u64::from(end.attempt) * 3 * MS / 2);
        assert_eq!(end.attempt, if terminal == Outcome::Lost { 3 } else { 2 });
        assert_eq!(run.events().iter().any(|e| matches!(&e.kind,
            EventKind::PacketDelivered { route, bytes, .. } if route == "радиолиния" && bytes == &[1]
        )), terminal != Outcome::Lost);
    }
}

#[derive(Clone)]
struct Load {
    switch_at: Time,
    force: f64,
    body: &'static str,
    generalized: Option<f64>,
    noisy: bool,
}

impl ProcessModel for Load {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        if frame.next.start >= self.switch_at {
            out.body_loads.push(BodyLoad {
                body: self.body.into(),
                force_n: [self.force, 0.0, 0.0],
                torque_nm: [0.0; 3],
            });
            if let Some(force) = self.generalized {
                out.efforts.push((ChannelId(0), force));
            }
        }
        if self.noisy {
            out.measure(frame.next.start, "проба", "1", known(rng.normal()))?;
        }
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        self.force.to_bits()
    }
}

#[derive(Clone)]
struct Drive;
impl Object for Drive {
    fn update(
        &mut self,
        _: &Truth<'_>,
        _: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        // Известное постоянное усилие испытательного привода.
        out.set(ChannelId(0), 2.0);
        Ok(())
    }
}

fn build(controlled: bool) -> Runtime {
    let mut rt = Runtime::load(scene()).unwrap();
    rt.register_object("привод", Rate::hz(1000).unwrap(), &[ChannelId(0)], Drive)
        .unwrap();
    rt.register_process(
        "нагрузка",
        Load {
            switch_at: Time::ZERO,
            force: 4.0,
            body: "б",
            generalized: Some(-1.0),
            noisy: false,
        },
    )
    .unwrap();
    if controlled {
        control::install(
            &mut rt,
            &ControlConfig::parse("version=1").unwrap(),
            &Assets,
            spec(),
        )
        .unwrap();
    }
    rt
}

#[test]
fn привод_и_две_реакции_суммируются_ровно_один_раз() {
    for controlled in [false, true] {
        let mut rt = build(controlled);
        assert_eq!(
            rt.run_until(END).unwrap(),
            Status::Running,
            "{:?}",
            rt.failure()
        );
        let journal = rt.process_journal();
        assert_eq!(journal.len(), 7);
        let actual = journal.last().unwrap().qvel[1];
        // Ньютон: импульс 5 Н × 6 мс для 2 кг.
        let expected = (2.0 + 4.0 - 1.0) * END.as_secs() / 2.0;
        assert!(
            (actual - expected).abs() < 16.0 * f64::EPSILON,
            "{actual} vs {expected}"
        );
        assert_eq!(journal.last().unwrap().qvel[0], 0.0);
    }
}

fn attachment() -> AttachmentModel {
    let ports = ["а", "б"].map(|body| MountPort {
        name: format!("{body}/посадка"),
        body: body.into(),
        kind: MountKind::Mechanical,
        interface: "плоскость".into(),
    });
    AttachmentModel::new(
        AttachmentBinding {
            name: "крепление".into(),
            mounts: ports.clone().map(|p| p.name),
            kind: AttachmentKind::Adhesive,
        },
        ports,
        // Геометрический захват и условия отверждения контрольного стенда.
        CaptureLimits {
            gap_m: exact(0.001),
            lateral_m: exact(0.001),
            axis_error_rad: exact(0.01),
            speed_m_s: exact(0.01),
            angular_speed_rad_s: exact(0.01),
            required_travel_m: exact(0.001),
            release_force_n: exact(0.5),
            release_torque_nm: exact(0.5),
            capacity: WrenchCapacity {
                force_n: exact(1.0),
                torque_nm: exact(1.0),
            },
        },
        AttachmentLaw::Adhesive(AdhesiveLaw {
            cure_s: exact(0.001),
            compression_n: source(3.0, 5.0, 7.0),
            temperature_k: source(290.0, 295.0, 300.0),
        }),
    )
    .unwrap()
}

fn input(frame: &ProcessFrame<'_>) -> Result<AttachmentInput, ProcessError> {
    let mounts = ["ось-а", "ось-б"].map(|axis| {
        let (q, v) = frame.truth.joint(axis).unwrap();
        MountMotion {
            point_m: [q[0], 0.0, 0.0],
            com_m: [q[0], 0.0, 0.0],
            axis: [1.0, 0.0, 0.0],
            velocity_m_s: [v[0], 0.0, 0.0],
            angular_velocity_rad_s: [0.0; 3],
        }
    });
    // На каждой каретке ровно одна поступательная степень свободы вдоль X.
    // Поэтому второй компонент Jᵀλ здесь равен осевой силе в ньютонах.
    let measured = frame
        .completed
        .and_then(|step| step.equality_reactions["стык"].as_ref())
        .map_or(0.0, |v| v[1]);
    Ok(AttachmentInput {
        mounts,
        available_travel_m: known(0.01),
        measured_wrench: known(JointWrench {
            force_n: [measured, 0.0, 0.0],
            torque_nm: [0.0; 3],
        }),
        latch_contact_n: known(0.0),
        thread: None,
        // Контрольный стенд задаёт отдельное измерение температуры и прижима.
        cure: known(CureObservation {
            compression_n: [4.0, 6.0],
            temperature_k: [294.0, 296.0],
            gap_m: 0.0,
            lateral_m: 0.0,
            axis_error_rad: 0.0,
            speed_m_s: 0.0,
            angular_speed_rad_s: 0.0,
        }),
        weld_residual: Some(WeldResidual {
            translation_m: (mounts[0].point_m[0] - mounts[1].point_m[0]).abs(),
            rotation_rad: 0.0,
        }),
        command: None,
    })
}

fn with_attachment(extra: Option<f64>) -> Runtime {
    let mut rt = Runtime::load(scene()).unwrap();
    // Запрос между тактами, затем выдержка 1 мс, затем внешние 4 Н.
    let at = Time(MS / 2);
    let request = AttachmentRequest {
        at,
        name: "крепление".into(),
        action: AttachmentAction::Attach,
    };
    rt.register_attachment(AttachmentAdapter {
        model: attachment(),
        equality: Some("стык".into()),
        requests: BTreeMap::from([(at, request)]),
        input,
    })
    .unwrap();
    rt.register_process(
        "нагрузка",
        Load {
            switch_at: Time(2 * MS),
            force: 4.0,
            body: "б",
            generalized: None,
            noisy: true,
        },
    )
    .unwrap();
    if let Some(force) = extra {
        rt.register_process(
            "проверка пакета",
            Load {
                switch_at: Time(2 * MS),
                force,
                body: "а",
                generalized: None,
                noisy: false,
            },
        )
        .unwrap();
    }
    rt
}

fn bytes(rt: &Runtime) -> Vec<u8> {
    let mut bytes = Vec::new();
    rt.log().write_binary(&mut bytes).unwrap();
    bytes
}

#[test]
fn смена_связи_и_процесс_побитово_в_снимке_ветке_и_воспроизведении() {
    let mut direct = with_attachment(None);
    assert_eq!(
        direct.run_until(END).unwrap(),
        Status::Running,
        "{:?}",
        direct.failure()
    );
    let rows = direct.process_journal();
    let changes: Vec<_> = rows
        .iter()
        .flat_map(|row| row.outputs["крепление"].attachment_records())
        .filter_map(|r| r.event.as_ref())
        .collect();
    assert_eq!(changes.len(), 3);
    assert_eq!(
        (changes[0].request.at, changes[0].evaluated_at),
        (Time(MS / 2), Time(MS))
    );
    assert_eq!(changes[1].evaluated_at, Time(2 * MS));
    assert_eq!(changes[2].evaluated_at, Time(3 * MS));
    assert_eq!(rows[2].qvel, [0.0; 2]);
    assert_eq!(
        rows[2].outputs["крепление"].attachment_records()[0].applied_limit,
        Some(("стык".into(), true))
    );
    assert_eq!(
        rows[3].outputs["крепление"].attachment_records()[0].applied_limit,
        Some(("стык".into(), false))
    );
    let reaction = rows[3].completed.as_ref().unwrap().equality_reactions["стык"]
        .as_ref()
        .unwrap();
    assert!(reaction[0] > 1.0 && reaction[1] < -1.0);
    assert_eq!(reaction[0], -reaction[1]);
    let momentum: f64 = rows[3].qvel.iter().map(|v| 2.0 * v).sum();
    assert!((momentum - 4.0 * Time(MS).as_secs()).abs() < 16.0 * f64::EPSILON);
    assert!(rows[4].completed.as_ref().unwrap().equality_reactions["стык"].is_none());

    let expected = bytes(&direct);
    let mut audited = with_attachment(None);
    audited.audit_snapshot(Time(2 * MS), END).unwrap();
    assert_eq!(bytes(&audited), expected);
    let mut split = with_attachment(None);
    split.run_until(Time(2 * MS)).unwrap();
    let snapshot = split.try_snapshot().unwrap();
    let mut fork = split.fork().unwrap();
    fork.run_until(END).unwrap();
    assert_eq!(split.time(), Time(2 * MS));
    assert_eq!(bytes(&fork), expected);
    split.run_until(Time(4 * MS)).unwrap();
    split.restore(&snapshot).unwrap();
    split.run_until(END).unwrap();
    assert_eq!(bytes(&split), expected);

    let log = Log::read_binary(expected.as_slice()).unwrap();
    assert!(
        log.process_entries()
            .iter()
            .any(|(_, line)| line.contains("Overload"))
    );
    let mut replay = with_attachment(None);
    replay.replay(log).unwrap();
    replay.run_until(END).unwrap();
    replay.verify_replay().unwrap();
    assert_eq!(bytes(&replay), expected);
}

#[test]
fn неконечный_или_конфликтующий_пакет_эффектов_не_применяется_частично() {
    let mut good = with_attachment(None);
    good.run_until(Time(MS)).unwrap();
    for value in [f64::NAN, f64::INFINITY] {
        let mut rt = with_attachment(Some(value));
        assert_eq!(rt.run_until(END).unwrap(), Status::ComponentFailure);
        assert_eq!(rt.time(), Time(2 * MS));
        assert_eq!(
            rt.process_journal(),
            good.process_journal()
                .iter()
                .cloned()
                .map(|mut tick| {
                    tick.outputs
                        .insert("проверка пакета".into(), ProcessOut::default());
                    tick
                })
                .collect::<Vec<_>>()
        );
        assert!(rt.failure().unwrap().contains("неконечная нагрузка"));
    }

    // Два писателя одного состояния. В журнале отсутствуют оба выхода и переключение связи.
    #[derive(Clone)]
    struct Duplicate;
    impl ProcessModel for Duplicate {
        fn update(
            &mut self,
            frame: &ProcessFrame<'_>,
            _: &mut Rng,
        ) -> Result<ProcessOut, ProcessError> {
            let mut out = ProcessOut::default();
            if frame.next.start == Time(2 * MS) {
                out.states.insert("крепление".into(), "Free".into());
            }
            Ok(out)
        }
        fn state_fingerprint(&self) -> u64 {
            0
        }
    }
    let mut rt = with_attachment(None);
    rt.register_process("конфликт", Duplicate).unwrap();
    assert_eq!(rt.run_until(END).unwrap(), Status::ComponentFailure);
    assert_eq!(
        rt.failure(),
        Some("недопустимое или повторное состояние «крепление»")
    );
    assert_eq!(
        rt.process_journal().last().unwrap().outputs["крепление"].attachment_records()[0].state,
        AttachmentState::Engaging
    );
}

fn packet_config(kind: &str, options: &str) -> ControlConfig {
    ControlConfig::parse(&format!(
        r#"version=1
[[board]]
name='плата'
catalog='vmcu-c3'
image='образ'
config='параметры'
rate_hz=4000
source='искусственный стенд'
[[board_input]]
name='вход'
board='плата'
signal='измерение'
unit='m'
kind='adc'
port=0
max_age_ns=2000000
scale=1
offset=0
fallback=0
source='искусственный стенд'
[[packet]]
name='радиолиния'
kind='{kind}'
to_board='плата'
to_port=0
capacity=2
max_age_ns=3000000
rssi_dbm=-60
max_bytes=20
{options}
source='искусственный стенд'
[[transmission]]
route='радиолиния'
at_ns=250000
bytes=[1]
source='искусственный стенд'
"#
    ))
    .unwrap()
}

fn radio_runtime() -> Runtime {
    let mut rt = Runtime::load(scene()).unwrap();
    rt.register_observation("измерение", "m").unwrap();
    rt
}

#[test]
fn записанные_слоты_доставляют_пакет_без_доказательства() {
    let mut rt = radio_runtime();
    let config = packet_config("scheduled_radio", "slots_asset='слоты'");
    control::install(&mut rt, &config, &Assets, spec()).unwrap();
    assert_eq!(
        rt.run_until(END).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(|e| matches!(&e.kind,
        EventKind::PacketDelivered { route, bytes, .. } if route == "радиолиния" && bytes == &[1]
    ) && e.at == Time(2 * MS)));
    assert_eq!(
        run.recorded_radio_routes().collect::<Vec<_>>(),
        ["радиолиния"]
    );
    assert!(
        run.simplifications()
            .iter()
            .any(|l| l.subject == "channel:радиолиния"
                && l.blocks_proof
                && l.text.contains("NotAnalyzed"))
    );
    let mut replay = radio_runtime();
    control::install_replay(&mut replay, &config, &Assets, spec(), run.recording()).unwrap();
    replay.run_until(END).unwrap();
    control::verify_replay(&replay).unwrap();

    let mut live = radio_runtime();
    let config = packet_config("ble", "interval_ns=1000000\nloss=0\nattempts=1");
    control::install(&mut live, &config, &Assets, spec()).unwrap();
    live.run_until(END).unwrap();
    let run = control::run(&live).unwrap();
    assert!(run.recorded_radio_routes().next().is_none());
    assert!(
        run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::PacketDelivered { .. }) && e.at == Time(MS))
    );
}

#[test]
fn тепловой_процесс_переживает_откат_и_сверяется_с_экспонентой() {
    use sim_process::work::heat::{HeatInput, HeatSpec, HeatStep, HeatTransfer, ThermalContact};
    let mut rt = Runtime::load(scene()).unwrap();
    // Две ёмкости по 2 Дж/К, контактная проводимость 5 Вт/К.
    // Начальный перепад 20 К, внешнего источника энергии нет.
    let model = HeatTransfer::new(HeatSpec {
        source_capacity_j_k: exact(2.0),
        target_capacity_j_k: exact(2.0),
        source_initial_k: exact(310.0),
        target_initial_k: exact(290.0),
        contact_coefficient_w_m2_k: exact(5.0),
        calibrated_area_m2: exact(1.0),
    })
    .unwrap();
    rt.register_step_model(
        "тепло",
        model,
        |frame| {
            Ok(frame.completed.map(|last| {
                (
                    last.interval,
                    HeatInput {
                        power_into_source_w: 0.0,
                        contact: Some(ThermalContact::Touching {
                            area_m2: Some(exact(1.0)),
                        }),
                    },
                )
            }))
        },
        |at, result| {
            let mut out = ProcessOut::default();
            for (name, select) in [
                (
                    "источник",
                    (|step: HeatStep| step.source_k) as fn(HeatStep) -> f64,
                ),
                ("цель", |step: HeatStep| step.target_k),
            ] {
                let measure = match &result {
                    Analysis::Analyzed { value, scope } => Analysis::Analyzed {
                        value: select(*value),
                        scope: scope.clone(),
                    },
                    Analysis::NotAnalyzed { reason } => Analysis::NotAnalyzed {
                        reason: reason.clone(),
                    },
                };
                out.measure(at, name, "K", measure)?;
            }
            Ok(out)
        },
    )
    .unwrap();
    rt.run_until(Time(2 * MS)).unwrap();
    let snap = rt.try_snapshot().unwrap();
    assert_eq!(rt.run_until(END).unwrap(), Status::Running);
    let proven = bytes(&rt);
    let row = rt.process_journal().last().unwrap();
    let temps: Vec<f64> = row.outputs["тепло"]
        .measurements
        .iter()
        .map(|m| match m.result {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { .. } => panic!("тепло не рассчитано"),
        })
        .collect();
    // Сохраняется средняя температура 300 K. Перепад затухает с λ=5 с⁻¹.
    let expected = 300.0 - 10.0 * (-5.0 * END.as_secs()).exp();
    assert!((temps[1] - expected).abs() < 4.0 * f64::EPSILON * expected);
    assert_eq!(temps[0] + temps[1], 600.0);
    rt.restore(&snap).unwrap();
    rt.run_until(END).unwrap();
    assert_eq!(bytes(&rt), proven);
}

#[test]
fn погружение_передаёт_нагрузку_тепло_и_глубину_в_модель_радио() {
    use sim_process::work::immersion::*;
    use sim_runtime::process::{ImmersionAdapter, RadioPath};
    let mut world = scene();
    world.mjcf = world
        .mjcf
        .replace("gravity=\"0 0 0\"", "gravity=\"0 0 -10\"");
    let mut rt = Runtime::load(world.clone()).unwrap();
    rt.register_observation("измерение", "m").unwrap();
    let cfg = packet_config("ble", "interval_ns=1000000\nloss=0\nattempts=1");
    control::install(&mut rt, &cfg, &Assets, spec()).unwrap();
    // Контрольная радиолиния −60 дБм, порог −80 дБм в диапазоне 2,4 ГГц.
    rt.bind_process_radio(
        "радиолиния",
        RadioPath {
            frequency_hz: exact(2.4e9),
            sensitivity_dbm: exact(-80.0),
        },
    )
    .unwrap();
    rt.register_immersion(ImmersionAdapter {
        name: "жидкость".into(),
        drive: Some(ChannelId(0)),
        radio: vec!["радиолиния".into()],
        model: ImmersionSpec {
            body: "б".into(),
            half_extent_m: [0.5; 3].map(exact),
            region: ImmersionRegion {
                surface_point_m: [0.0; 3].map(exact),
                outward_normal: [0.0, 0.0, 1.0].map(exact),
                density_kg_m3: exact(1000.0),
                viscosity_pa_s: exact(0.001),
                flow_m_s: [0.0; 3].map(exact),
                temperature_k: exact(293.15),
            },
            drag: None,
            heat: Some(WetHeatSpec {
                coefficient_w_m2_k: exact(2.0),
                calibrated_speed_m_s: source(0.0, 0.5, 1.0),
                calibrated_area_m2: source(1.0, 3.0, 6.0),
            }),
            radio: Some(RadioFluid {
                water: RadioWater::Fresh,
                composition_source: exact(0.0).provenance,
                calibrated_temperature_k: source(283.15, 293.15, 303.15),
            }),
        },
        input: |frame: &ProcessFrame<'_>| {
            let block = frame.body_point("б", [0.0; 3])?;
            Ok(ImmersionInput {
                box_center_m: block.position_m,
                box_axes: std::array::from_fn(|i| block.rotation.map(|row| row[i])),
                body_com_m: block.com_m,
                velocity_at_com_m_s: block.velocity_m_s,
                angular_velocity_rad_s: block.angular_velocity_rad_s,
                gravity_m_s2: frame.truth.engine().gravity(),
                engine_density_kg_m3: 0.0,
                engine_viscosity_pa_s: 0.0,
                housing_temperature_k: Some(294.15),
                antenna_m: Some([0.0, 0.0, -3.0]),
            })
        },
    })
    .unwrap();
    assert_eq!(
        rt.run_until(END).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let row = &rt.process_journal()[0].outputs["жидкость"];
    // Половина куба в жидкости даёт 0,5 м³ и 5 кН по Архимеду.
    assert!((row.body_loads[0].force_n[2] - 5000.0).abs() < 5000.0 * f64::EPSILON * 8.0);
    let Analysis::Analyzed {
        value: ThermalBoundary::Wet { heat_out_w, .. },
        ..
    } = row.thermal[&ChannelId(0)]
    else {
        panic!("нет тепловой границы");
    };
    // Дно и четыре половины боков дают 3 м², перепад 1 K.
    assert_eq!(heat_out_w, 6.0);
    let Analysis::Analyzed {
        value: environment, ..
    } = row.radio["радиолиния"]
    else {
        panic!("нет глубины");
    };
    assert_eq!(environment.depth_m, 3.0);
    let material = environment.medium.permittivity(2.4e9);
    assert!(material.attenuation_db_per_m(2.4e9) * environment.depth_m > 20.0);
    let run = control::run(&rt).unwrap();
    assert!(
        run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::Packet { event, .. }
        if event.outcome == control::PacketOutcome::Lost))
    );
    assert!(
        !run.events()
            .iter()
            .any(|e| matches!(e.kind, EventKind::PacketDelivered { .. }))
    );
}
