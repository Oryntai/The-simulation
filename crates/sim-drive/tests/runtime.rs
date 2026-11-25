use std::collections::BTreeMap;
use std::path::PathBuf;

use sim_drive::{DriveCatalog, install};
use sim_runtime::{LogKind, Runtime, Status};
use sim_signal::{ChannelId, Command, Rate, Time};
use sim_world::{CompiledDrive, CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel};

struct Temp(PathBuf);

impl Temp {
    fn new() -> Self {
        for suffix in 0_u64.. {
            let dir = std::env::temp_dir()
                .join(format!("sim-drive-test-{}-{suffix}", std::process::id()));
            match std::fs::create_dir(&dir) {
                Ok(()) => return Self(dir),
                Err(err) if err.kind() == std::io::ErrorKind::AlreadyExists => {}
                Err(err) => panic!("{err}"),
            }
        }
        unreachable!()
    }

    fn write(&self, id: &str, data: &str) {
        std::fs::write(self.0.join(format!("{id}.toml")), data).unwrap();
    }
}

impl Drop for Temp {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

fn position_file(id: &str, kind: &str, mode: &str, internal: bool, delay: f64) -> String {
    // Числа синтетического стенда: R=2, L=0.5, K=1, J ротора=0.2.
    // Синтетические границы позиционки ±2 рад, зазор регулятора ±0.05 рад.
    let inertia = if internal { "internal" } else { "mjcf" };
    let mut text = format!(
        r#"
id = "{id}"
name = "Привод стенда"
kind = "{kind}"
mode = "{mode}"
inertia = "{inertia}"
backdrive = "reversible"
brake = "none"
stop = "none"

[parameters]
"#
    );
    let mut values = BTreeMap::from([
        ("supply_v", 8.0),
        ("voltage_min_v", 0.0),
        ("voltage_max_v", 8.0),
        ("current_limit_a", 100.0),
        ("resistance_ohm", 2.0),
        ("inductance_h", 0.5),
        ("motor_constant", 1.0),
        ("gear_ratio", 1.0),
        ("rotor_inertia_kg_m2", 0.2),
        ("gear_input_inertia_kg_m2", 0.0),
        ("gear_efficiency", 1.0),
        ("backward_efficiency", 1.0),
        ("coulomb_nm", 0.0),
        ("viscous_nm_per_rad_s", 0.0),
        ("static_friction_nm", 0.0),
        ("stribeck_rad_s", 1.0),
        ("friction_band_rad_s", 1.0),
        ("backlash", 0.0),
        ("control_hz", 1000.0),
        ("bus_delay_s", delay),
    ]);
    if mode == "position" {
        values.extend([
            ("position_min", -2.0),
            ("position_max", 2.0),
            ("position_deadband", 0.05),
            ("speed_limit", 1.0),
            ("position_gain", 10.0),
            ("speed_kp", 2.0),
            ("speed_ki", 0.0),
        ]);
    }
    if internal {
        values.extend([
            ("backlash", 0.4),
            ("coupling_stiffness", 100.0),
            ("coupling_damping", 2.0),
        ]);
    }
    if kind == "linear" {
        values.extend([
            // Коэффициент 10 рад/м. J_rotor × 10² = 0.2 кг в MJCF.
            ("linear_travel_m_per_rev", std::f64::consts::TAU / 10.0),
            ("rotor_inertia_kg_m2", 0.002),
        ]);
    }
    use std::fmt::Write;
    for (key, value) in values {
        writeln!(text, r#"{key} = {{ value = {value:?}, source = "derived", reference = "Синтетический стенд с заданными константами" }}"#).unwrap();
    }
    text
}

fn world(part: &str, kind: CompiledJointKind, armature: f64) -> CompiledWorld {
    // Нейтральный блок. Поворот: J = 0.8 + armature. Поступательно:
    // масса 2 кг + armature. На стенде отсутствуют тяжесть и внешние силы.
    let (joint, axis) = if kind == CompiledJointKind::Slide {
        ("slide", "1 0 0")
    } else {
        ("hinge", "0 0 1")
    };
    CompiledWorld {
        name: "блок".into(),
        seed: 17,
        physics: Rate::hz(10_000).unwrap(),
        mjcf: format!(
            r#"<mujoco model="блок">
          <option timestep="0.0001" gravity="0 0 0" integrator="Euler"/>
          <worldbody><body name="блок">
            <joint name="ось" type="{joint}" axis="{axis}" armature="{armature}"/>
            <inertial pos="0 0 0" mass="2" diaginertia="0.8 0.8 0.8"/>
          </body></worldbody></mujoco>"#
        ),
        bodies: vec![],
        joints: vec![CompiledJoint {
            name: "ось".into(),
            owner: "блок".into(),
            kind,
        }],
        efforts: vec![EffortChannel {
            name: "команда".into(),
            joint: "ось".into(),
            dof: 0,
        }],
        drives: vec![CompiledDrive {
            name: "блок/привод".into(),
            owner: "блок".into(),
            part: part.into(),
            effort: "команда".into(),
            joint: "ось".into(),
            params: BTreeMap::new(),
        }],
        sensors: vec![],
        simplifications: vec![],
    }
}

fn go(runtime: &mut Runtime, t: f64) {
    assert_eq!(
        runtime.run_until(Time::from_secs(t).unwrap()).unwrap(),
        Status::Running,
        "{:?}",
        runtime.failure()
    );
}

fn command(runtime: &mut Runtime, id: ChannelId, value: f64) {
    runtime
        .act(&Command {
            targets: vec![(id, value)],
        })
        .unwrap();
}

fn trace(runtime: &Runtime, name: &str) -> Vec<(Time, f64)> {
    let header = runtime.log().header();
    let index = header.truth.iter().position(|s| s.name == name).unwrap();
    runtime
        .log()
        .entries()
        .iter()
        .filter(|e| e.id as usize == index && e.kind == LogKind::Truth)
        .map(|e| (e.at, e.value))
        .collect()
}

fn last(runtime: &Runtime, name: &str) -> f64 {
    trace(runtime, name).last().unwrap().1
}

fn bytes(runtime: &Runtime) -> Vec<u8> {
    let mut data = Vec::new();
    runtime.log().write_binary(&mut data).unwrap();
    data
}

#[test]
fn new_catalog_file_runs_second_order_response_in_mujoco() {
    let dir = Temp::new();
    let id = "новая-позиция"; // Проверка строгости идентификатора файла отдельно от физики.
    dir.write(id, &position_file(id, "dc", "pwm", false, 0.0));
    assert!(
        DriveCatalog::load(&dir.0)
            .unwrap_err()
            .contains("<id>.toml")
    );
    std::fs::remove_file(dir.0.join(format!("{id}.toml"))).unwrap();
    let id = "test-only-block";
    assert!(DriveCatalog::load(&dir.0).unwrap().get(id).is_none());
    dir.write(id, &position_file(id, "dc", "pwm", false, 0.0));
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let w = world(id, CompiledJointKind::Hinge, 0.2);
    let mut r = Runtime::load(w.clone()).unwrap();
    let handles = install(&mut r, &w, &catalog).unwrap();
    assert_eq!(handles.len(), 1);
    assert_eq!(
        r.truth_id("блок/привод/current_a"),
        Some(handles[0].current_a)
    );
    r.record_joint_state("координаты").unwrap();
    command(&mut r, handles[0].command, 1.0);
    let discriminant = 2.0_f64.sqrt();
    let a = -2.0 + discriminant;
    let b = -2.0 - discriminant;
    let mut max_error = [0.0_f64; 2];
    for t in [0.1, 0.2, 0.4, 0.8] {
        go(&mut r, t);
        // Jω' = Ki, L i' = V − Ri − Kω; ω∞=V/K=8. Уравнение
        // характеристик s²+4s+2=0, условия ω(0)=ω'(0)=0.
        let speed = 8.0 * (1.0 + (b * (a * t).exp() - a * (b * t).exp()) / (a - b));
        let current = 8.0 * a * b * ((a * t).exp() - (b * t).exp()) / (a - b);
        max_error[0] = max_error[0].max((last(&r, "qvel[0]") - speed).abs());
        max_error[1] = max_error[1].max((last(&r, "блок/привод/current_a") - current).abs());
    }
    // Погрешность разделённого шага MuJoCo/привода первого порядка при Δt=0.1 мс.
    assert!(max_error[0] < 5e-4 && max_error[1] < 5e-4, "{max_error:?}");
    assert!(last(&r, "qvel[0]") > 2.0);
    println!(
        "Второй порядок: max Δω={:.9e} рад/с, max Δi={:.9e} А",
        max_error[0], max_error[1]
    );
}

#[test]
fn delay_deadband_snapshot_and_duplicate_runs_are_checked_in_mujoco() {
    let dir = Temp::new();
    // Команда появляется между тактами регулятора. Задержка 3.05 мс
    // округлится вверх до 3.1 мс на сетке физики 0.1 мс.
    let id = "bench-servo";
    dir.write(
        id,
        &position_file(id, "hobby_servo", "position", false, 0.00305),
    );
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let w = world(id, CompiledJointKind::Hinge, 0.2);
    let start = || {
        let mut r = Runtime::load(w.clone()).unwrap();
        let p = install(&mut r, &w, &catalog).unwrap().remove(0);
        r.record_joint_state("координаты").unwrap();
        command(&mut r, p.command, 0.03);
        go(&mut r, 0.010);
        assert_eq!(last(&r, "qvel[0]"), 0.0);
        assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
        assert_eq!(
            trace(&r, "блок/привод/accepted_command")[0],
            (Time(3_100_000), 0.03)
        );
        command(&mut r, p.command, 0.50);
        go(&mut r, 0.012);
        (r, p.command)
    };
    let finish = |r: &mut Runtime, id| {
        go(r, 0.060);
        let arrival = trace(r, "блок/привод/accepted_command")
            .into_iter()
            .find(|e| e.1 == 0.50)
            .unwrap()
            .0;
        // 0.1 мс от вызова act после уже завершившегося такта, затем 3.05 мс на шине.
        assert_eq!(arrival, Time(13_200_000));
        let first_current = trace(r, "блок/привод/current_a")
            .into_iter()
            .find(|e| e.1 > 0.0)
            .unwrap()
            .0;
        assert_eq!(first_current, Time(14_100_000));
        assert!(last(r, "qvel[0]") > 0.0);
        command(r, id, -0.50);
        go(r, 0.130);
        assert!(last(r, "блок/привод/current_a") < 0.0);
    };
    let (mut first, id) = start();
    let snapshot = first.snapshot();
    finish(&mut first, id);
    let hash = first.log().end().state_hash;
    let baseline = bytes(&first);
    assert!(first.log().entries().len() > 1000);
    first.restore(&snapshot).unwrap();
    finish(&mut first, id);
    assert_eq!(first.log().end().state_hash, hash);
    assert_eq!(bytes(&first), baseline);
    let (mut other, other_id) = start();
    finish(&mut other, other_id);
    assert_eq!(bytes(&other), baseline);
}

#[test]
fn compliant_rotor_and_linear_force_reach_the_physics_engine() {
    let dir = Temp::new();
    dir.write(
        "coupled",
        &position_file("coupled", "bldc", "pwm", true, 0.0),
    );
    dir.write(
        "linear",
        &position_file("linear", "linear", "current", false, 0.0),
    );
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let w = world("coupled", CompiledJointKind::Hinge, 0.0);
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    assert!(p.backlash_rad.is_some());
    r.record_joint_state("координаты").unwrap();
    command(&mut r, p.command, 1.0);
    go(&mut r, 0.1);
    assert_eq!(last(&r, "qpos[0]"), 0.0);
    assert_eq!(last(&r, "блок/привод/joint_torque_nm"), 0.0);
    assert!(last(&r, "блок/привод/motor_rad") > 0.01);
    go(&mut r, 0.4);
    assert!(last(&r, "qvel[0]") > 0.1);
    let residual = last(&r, "блок/привод/motor_energy_residual_j");
    assert!(residual.abs() < 5e-4, "{residual:.9e} Дж");

    let w = world("linear", CompiledJointKind::Slide, 0.2);
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    assert!(p.joint_torque_nm.is_none() && p.joint_force_n.is_some());
    command(&mut r, p.command, 0.50);
    go(&mut r, 0.08);
    // k=1 Н·м/А, 0.5 А, десять радиан ротора на метр выхода.
    assert!((last(&r, "блок/привод/joint_force_n") - 5.0).abs() < 1e-9);
    assert!(last(&r, "блок/привод/motor_rad_s") > 0.0);
}

#[test]
fn invalid_catalog_world_and_instance_inputs_are_rejected_without_installing() {
    let dir = Temp::new();
    let id = "bench-block";
    let standard = position_file(id, "dc", "pwm", true, 0.0);
    for (encoding, message) in [
        (
            standard.replace("stop = \"none\"", "stop = \"none\"\nextra = true"),
            "extra",
        ),
        (
            standard.replace("resistance_ohm", "unknown_parameter"),
            "unknown_parameter",
        ),
        (
            standard.replace("value = 2.0,", "value = nan,"),
            "resistance_ohm",
        ),
        (
            standard.replace("value = 2.0,", "value = inf,"),
            "resistance_ohm",
        ),
    ] {
        dir.write(id, &encoding);
        assert!(DriveCatalog::load(&dir.0).unwrap_err().contains(message));
    }
    let unknown = standard.lines().map(|line| {
        if line.starts_with("inductance_h") {
            r#"inductance_h = { unknown = true, bench = "подать импульс при застопоренном валу" }"#
        } else {
            line
        }
    }).collect::<Vec<_>>().join("\n");
    dir.write(id, &unknown);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world("missing-position", CompiledJointKind::Hinge, 0.0);
    let mut r = Runtime::load(w.clone()).unwrap();
    assert!(
        install(&mut r, &w, &catalog)
            .unwrap_err()
            .contains("неизвестная позиция")
    );
    w.drives[0].part = id.into();
    let err = install(&mut r, &w, &catalog).unwrap_err();
    assert!(
        err.contains("inductance_h") && err.contains("застопоренном валу"),
        "{err}"
    );
    assert!(r.log().header().truth.is_empty());
    for bad in [f64::NAN, f64::INFINITY] {
        w.drives[0].params.insert("inductance_h".into(), bad);
        assert!(
            install(&mut r, &w, &catalog)
                .unwrap_err()
                .contains("конечное")
        );
    }
    w.drives[0].params.insert("inductance_h".into(), 0.5);
    w.drives[0].params.insert("unknown_parameter".into(), 1.0);
    assert!(
        install(&mut r, &w, &catalog)
            .unwrap_err()
            .contains("unknown_parameter")
    );
    w.drives[0].params.remove("unknown_parameter");
    let w2 = world(id, CompiledJointKind::Hinge, 0.2);
    let mut r2 = Runtime::load(w2.clone()).unwrap();
    assert!(
        install(&mut r2, &w2, &catalog)
            .unwrap_err()
            .contains("двойной учёт")
    );
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    assert!(p.winding_c.is_none());
    assert!(
        r.simplifications()
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("inductance_h"))
    );
    assert!(
        r.simplifications()
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("нагрев"))
    );
    assert!(
        r.act(&Command {
            targets: vec![(p.command, f64::NAN)]
        })
        .is_err()
    );
    let count = r.log().header().truth.len();
    assert!(install(&mut r, &w, &catalog).is_err());
    assert_eq!(r.log().header().truth.len(), count);
}

#[test]
fn estimates_block_proof_and_registration_rolls_back_on_collision() {
    let dir = Temp::new();
    let id = "estimated-block";
    dir.write(
        id,
        &position_file(id, "dc", "pwm", false, 0.0)
            .replace("source = \"derived\"", "source = \"estimate\""),
    );
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    let mut second = w.drives[0].clone();
    second.name = "блок/резерв".into();
    second.effort = "резерв".into();
    w.efforts.push(EffortChannel {
        name: second.effort.clone(),
        ..w.efforts[0].clone()
    });
    w.drives.push(second);
    let mut r = Runtime::load(w.clone()).unwrap();
    r.register_truth("блок/резерв/motor_rad", "rad").unwrap();
    assert!(install(&mut r, &w, &catalog).is_err());
    assert_eq!(r.log().header().truth.len(), 1);
    assert!(r.truth_id("блок/привод/current_a").is_none());
    assert!(r.simplifications().is_empty());
    w.drives.pop();
    install(&mut r, &w, &catalog).unwrap();
    assert!(
        r.simplifications()
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("resistance_ohm: оценка"))
    );
}

#[test]
fn cutting_power_engages_brake_after_motion_in_mujoco() {
    let dir = Temp::new();
    let id = "braked-block";
    // Стенд: тормоз 100 Н·м/рад, предельный момент 3 Н·м.
    let text = position_file(id, "dc", "pwm", false, 0.0).replace(
        "brake = \"none\"",
        "brake = \"holding\"\npower_input = true",
    ) + r#"
brake_effort = { value = 3.0, source = "derived", reference = "Синтетический стенд" }
brake_stiffness = { value = 100.0, source = "derived", reference = "Синтетический стенд" }
"#;
    dir.write(id, &text);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    let mut r = Runtime::load(w.clone()).unwrap();
    assert!(
        install(&mut r, &w, &catalog)
            .unwrap_err()
            .contains("power_input")
    );
    w.efforts.push(EffortChannel {
        name: "команда-power".into(),
        ..w.efforts[0].clone()
    });
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    r.record_joint_state("координаты").unwrap();
    let power = p.power.unwrap();
    command(&mut r, p.command, 1.0);
    command(&mut r, power, 1.0);
    go(&mut r, 0.10);
    assert!(last(&r, "qvel[0]") > 0.05);
    assert!(last(&r, "блок/привод/current_a") > 1.0);
    command(&mut r, power, 0.0);
    go(&mut r, 0.1001);
    let anchor = last(&r, "qpos[0]");
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    assert_eq!(last(&r, "блок/привод/joint_torque_nm"), 0.0);
    go(&mut r, 0.12);
    // Euler даёт x_mid=q+h²F/(4J) в координате q−hv/2.
    // Независимо решаем линейное уравнение F=−K(x_mid−anchor).
    let h = 1.0 / r.physics_rate().as_hz();
    let expected =
        (-100.0 * (last(&r, "qpos[0]") - anchor) / (1.0 + 100.0 * h * h / 4.0)).clamp(-3.0, 3.0);
    assert!(expected < -0.05);
    assert!((last(&r, "блок/привод/joint_torque_nm") - expected).abs() < 1e-12);
    assert!(last(&r, "блок/привод/motor_energy_residual_j").abs() < 1e-9);
    command(&mut r, power, 1.0);
    go(&mut r, 0.14);
    assert!(last(&r, "блок/привод/current_a") > 0.0);
    command(&mut r, power, 2.0);
    assert_eq!(
        r.run_until(Time(141_000_000)).unwrap(),
        Status::ComponentFailure
    );
    assert!(r.failure().unwrap().contains("0 или 1"));
}

#[test]
fn velocity_mode_limits_requested_speed_at_fractional_update_rate() {
    let dir = Temp::new();
    // Синтетическая уставка: частота 1000.5 Гц, ограничение скорости 0.5 рад/с
    // и Kp=2 дают ток 1 А на неподвижном валу.
    let text = position_file("controlled-block", "bus_servo", "velocity", false, 0.0)
        .replace(
            "control_hz = { value = 1000.0",
            "control_hz = { value = 1000.5",
        )
        .replace(
            "inductance_h = { value = 0.5",
            "inductance_h = { value = 0.0",
        )
        + r#"
speed_limit = { value = 0.5, source = "derived", reference = "Синтетический стенд" }
speed_kp = { value = 2.0, source = "derived", reference = "Синтетический стенд" }
speed_ki = { value = 0.0, source = "derived", reference = "Синтетический стенд" }
"#;
    dir.write("controlled-block", &text);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let w = world("controlled-block", CompiledJointKind::Hinge, 0.2);
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    r.record_joint_state("координаты").unwrap();
    command(&mut r, p.command, 4.0);
    go(&mut r, 0.0);
    assert_eq!(last(&r, "блок/привод/current_a"), 1.0);
    // Номинальный второй тик 1 / 1000.5 с впервые попадает на 1.0 мс.
    go(&mut r, 0.0009);
    assert_eq!(last(&r, "блок/привод/current_a"), 1.0);
    go(&mut r, 0.001);
    let expected = 2.0 * (0.5 - last(&r, "qvel[0]"));
    assert!(expected < 1.0);
    assert!((last(&r, "блок/привод/current_a") - expected).abs() < 1e-13);
}

#[derive(Clone, Copy, Debug)]
enum BenchFeedback {
    Coupling,
    StopSpring,
    StopDamper,
    Lock,
    Brake,
    BackEmf,
    Friction,
}

impl BenchFeedback {
    fn setup(self, hz: u64) -> (Temp, DriveCatalog, CompiledWorld) {
        let dir = Temp::new();
        let id = "feedback-block";
        let internal = matches!(self, Self::Coupling);
        let armature = if internal { 0.0 } else { 0.5e-6 };
        let mut w = world(id, CompiledJointKind::Hinge, armature);
        w.physics = Rate::hz(hz).unwrap();
        let h = 1.0 / w.physics.as_hz();
        let body_inertia = 1e-6 - armature;
        w.mjcf = w.mjcf.replace("0.0001", &h.to_string()).replace(
            "0.8 0.8 0.8",
            &format!("{body_inertia} {body_inertia} {body_inertia}"),
        );
        w.efforts.push(EffortChannel {
            name: "толчок".into(),
            ..w.efforts[0].clone()
        });
        let mut file = position_file(id, "dc", "pwm", internal, 0.0).replace(
            "rotor_inertia_kg_m2 = { value = 0.2",
            if internal {
                "rotor_inertia_kg_m2 = { value = 0.2"
            } else {
                "rotor_inertia_kg_m2 = { value = 0.0000005"
            },
        );
        // J=10⁻⁶, h=1 мс. Настраиваем a=k·h²/J=5 либо b=c·h/J=5.
        // Амплитуды удержания и полоса трения шире первого тестового толчка.
        let extra = match self {
            Self::Coupling => {
                file = file
                    .replace(
                        "coupling_stiffness = { value = 100.0",
                        "coupling_stiffness = { value = 5.0",
                    )
                    .replace(
                        "coupling_damping = { value = 2.0",
                        "coupling_damping = { value = 0.0",
                    )
                    .replace("backlash = { value = 0.4", "backlash = { value = 0.0");
                vec![]
            }
            Self::StopSpring | Self::StopDamper => {
                file = file.replace("stop = \"none\"", "stop = \"compliant\"");
                let (k, c) = if matches!(self, Self::StopSpring) {
                    (5.0, 0.0)
                } else {
                    (0.001, 0.005)
                };
                vec![
                    ("stop_min", -1.0),
                    ("stop_max", 0.0),
                    ("stop_stiffness", k),
                    ("stop_damping", c),
                ]
            }
            Self::Lock => {
                file = file.replace("backdrive = \"reversible\"", "backdrive = \"self_locking\"");
                vec![("hold_effort", 3.0), ("hold_stiffness", 5.0)]
            }
            Self::Brake => {
                file = file.replace("brake = \"none\"", "brake = \"holding\"");
                vec![("brake_effort", 3.0), ("brake_stiffness", 5.0)]
            }
            Self::BackEmf => {
                file = file
                    .replace(
                        "inductance_h = { value = 0.5",
                        "inductance_h = { value = 0.0",
                    )
                    .replace(
                        "motor_constant = { value = 1.0",
                        "motor_constant = { value = 0.05",
                    );
                vec![]
            }
            Self::Friction => {
                file = file
                    .replace(
                        "static_friction_nm = { value = 0.0",
                        "static_friction_nm = { value = 0.0025",
                    )
                    .replace(
                        "coulomb_nm = { value = 0.0",
                        "coulomb_nm = { value = 0.0025",
                    )
                    .replace(
                        "friction_band_rad_s = { value = 1.0",
                        "friction_band_rad_s = { value = 2.0",
                    );
                vec![]
            }
        };
        if matches!(self, Self::BackEmf | Self::Friction) {
            // N=2. Armature=0.5·10⁻⁶ остаётся прежней, J_rotor уменьшается в N² раз.
            file = file
                .replace("gear_ratio = { value = 1.0", "gear_ratio = { value = 2.0")
                .replace(
                    "rotor_inertia_kg_m2 = { value = 0.0000005",
                    "rotor_inertia_kg_m2 = { value = 0.000000125",
                );
        }
        use std::fmt::Write;
        for (key, value) in extra {
            writeln!(file, r#"{key} = {{ value = {value:?}, source = "derived", reference = "Уравнение тестового блока" }}"#).unwrap();
        }
        dir.write(id, &file);
        let catalog = DriveCatalog::load(&dir.0).unwrap();
        (dir, catalog, w)
    }
}

const FEEDBACK_CASES: [BenchFeedback; 7] = [
    BenchFeedback::Coupling,
    BenchFeedback::StopSpring,
    BenchFeedback::StopDamper,
    BenchFeedback::Lock,
    BenchFeedback::Brake,
    BenchFeedback::BackEmf,
    BenchFeedback::Friction,
];

impl BenchFeedback {
    fn frequency(self) -> (f64, &'static str) {
        // Независимая подстановка коэффициентов стенда на выходе.
        let j = 1e-6;
        match self {
            Self::Coupling => (
                (5.0 * (1.0 / j + 1.0 / 0.2_f64)).sqrt(),
                "coupling_stiffness",
            ),
            Self::StopSpring => ((5.0 / j).sqrt(), "stop_stiffness"),
            Self::StopDamper => ((0.001 / j).sqrt() + 0.005 / j, "stop_damping"),
            Self::Lock => ((5.0 / j).sqrt(), "hold_stiffness"),
            Self::Brake => ((5.0 / j).sqrt(), "brake_stiffness"),
            Self::BackEmf => (0.005 / j, "противо-ЭДС"),
            Self::Friction => (0.005 / j, "static_friction_nm"),
        }
    }

    fn accuracy_limit(self) -> f64 {
        let (frequency, _) = self.frequency();
        // Допуск 1%: интегратор средней точки даёт ошибку относительной
        // фазы ≤ (ωh)²/12, абсолютную ошибку множителя затухания ≤ (bh)³/12.
        match self {
            Self::Friction | Self::BackEmf => frequency / 0.12_f64.cbrt(),
            Self::StopDamper => (0.005 / 1e-6) / 0.12_f64.cbrt(),
            _ => frequency / 0.12_f64.sqrt(),
        }
    }

    fn runtime(self, hz: u64) -> Runtime {
        let (_dir, catalog, w) = self.setup(hz);
        let mut r = Runtime::load(w.clone()).unwrap();
        let p = install(&mut r, &w, &catalog).unwrap().remove(0);
        r.record_joint_state("координаты").unwrap();
        if matches!(self, Self::BackEmf) {
            command(&mut r, p.command, 0.0);
        }
        r
    }

    fn trajectory(self, hz: u64, steps: u64) -> Vec<(f64, f64)> {
        self.trajectory_with_kick(hz, steps, 1)
    }

    fn trajectory_with_kick(self, hz: u64, steps: u64, kick_steps: u64) -> Vec<(f64, f64)> {
        let mut r = self.runtime(hz);
        let rate = r.physics_rate();
        let kick = r.channel("толчок").unwrap();
        // Прямоугольный толчок с импульсом J·1 рад/с, J=10⁻⁶.
        command(&mut r, kick, 1e-6 * hz as f64 / kick_steps as f64);
        assert_eq!(
            r.run_until(rate.tick(kick_steps)).unwrap(),
            Status::Running,
            "{:?}",
            r.failure()
        );
        command(&mut r, kick, 0.0);
        assert_eq!(
            r.run_until(rate.tick(steps)).unwrap(),
            Status::Running,
            "{:?}",
            r.failure()
        );
        trace(&r, "qpos[0]")
            .into_iter()
            .zip(trace(&r, "qvel[0]"))
            .map(|((_, q), (_, v))| (q, v))
            .collect()
    }
}

#[test]
fn sampled_drive_feedback_rejects_coarse_steps_before_publishing_force() {
    for case in FEEDBACK_CASES {
        let (_, parameter) = case.frequency();
        let required = case.accuracy_limit();
        // Исходно неустойчивая частота и сетка на 1% ниже нового порога.
        let below = (0.99 * required).floor() as u64;
        println!(
            "{case:?}: отказ при 1000 и {below} Гц, требуется ≥ {:.0} Гц",
            required.ceil()
        );
        for hz in [1000, below] {
            let mut r = case.runtime(hz);
            assert_eq!(
                r.run_until(Time::ZERO).unwrap(),
                Status::ComponentFailure,
                "{case:?}"
            );
            let error = r.failure().unwrap();
            assert!(
                error.contains("блок/привод") && error.contains(parameter),
                "{error}"
            );
            assert!(error.contains("Mᵢᵢ=1.000000000e-6"), "{error}");
            let requested = error
                .split("≥ ")
                .nth(1)
                .unwrap()
                .split(' ')
                .next()
                .unwrap()
                .parse::<f64>()
                .unwrap();
            assert_eq!(requested, required.ceil(), "{error}");
            assert_eq!(r.steps(), 0);
            assert!(trace(&r, "блок/привод/joint_torque_nm").is_empty());
        }
    }
}

#[test]
fn unexpected_axis_impulse_stops_before_publishing_the_next_effort() {
    let mut r = BenchFeedback::Friction.runtime(12_000);
    // Внешнее усилие меняет скорость на 1 рад/с за один такт при J=10⁻⁶.
    // Без него пробный средний момент трения нулевой. На первом шаге
    // средняя скорость 0.5 рад/с даёт |ΔF|=0.0025 Н·м и |Δp|=ΔF·h.
    let kick = r.channel("толчок").unwrap();
    command(&mut r, kick, 1e-6 * 12_000.0);
    assert_eq!(r.step().unwrap(), Status::ComponentFailure);
    assert_eq!(r.steps(), 1);
    assert_eq!(
        trace(&r, "блок/привод/joint_torque_nm"),
        vec![(Time::ZERO, 0.0)]
    );
    let reason = r.failure().unwrap();
    assert!(reason.contains("невязка импульса 2.083333e-7"), "{reason}");
    assert!(reason.contains("≥ 24000 Гц"), "{reason}");
}

#[test]
fn previously_verified_grid_agrees_with_a_finer_mujoco_run() {
    // Сохраняем прежнюю частоту этой сверки и сетку в 32 раза мельче.
    // Ошибки нормированы на толчок 1 рад/с и его путь за характерное время.
    let refine = 32;
    for case in FEEDBACK_CASES {
        let (frequency, _) = case.frequency();
        let hz = (1.01 * 64.0 * frequency).ceil() as u64;
        let steps = (hz as f64 / frequency).floor() as u64;
        let coarse = case.trajectory(hz, steps);
        let fine = case.trajectory(hz * refine, steps * refine);
        assert_eq!(coarse.len(), steps as usize + 1);
        assert_eq!(fine.len(), (steps * refine) as usize + 1);
        assert!(coarse.last().unwrap().0 > 0.0 && coarse.last().unwrap().1 < 0.95);
        // Первые интервалы содержат толчки разной длительности; сравнение
        // начинается после окончания обоих толчков.
        let mut error = [0.0_f64; 2];
        for ((q, v), (qr, vr)) in coarse
            .iter()
            .zip(fine.iter().step_by(refine as usize))
            .skip(1)
        {
            error[0] = error[0].max((q - qr).abs() * frequency);
            error[1] = error[1].max((v - vr).abs());
        }
        println!(
            "{case:?}: {hz} Гц, max Δq·ω={:.6} и Δv={:.6} рад/с",
            error[0], error[1]
        );
        assert!(error.into_iter().all(|e| e < 0.02), "{case:?}: {error:?}");
    }
}

#[test]
fn seven_feedback_modes_converge_against_closed_form() {
    // Отклик покоя на единичное постоянное ускорение. Вычислен из корней
    // характеристического многочлена; импульс строится разностью двух ступеней.
    let step = |t: f64, damping: f64, stiffness: f64| {
        if damping == 0.0 {
            let omega = stiffness.sqrt();
            (
                2.0 * (0.5 * omega * t).sin().powi(2) / stiffness,
                (omega * t).sin() / omega,
            )
        } else if stiffness == 0.0 {
            let velocity = -(-damping * t).exp_m1() / damping;
            ((t - velocity) / damping, velocity)
        } else {
            let r_fast = -0.5 * (damping + (damping * damping - 4.0 * stiffness).sqrt());
            let r_slow = stiffness / r_fast;
            (
                ((r_slow * t).exp_m1() / r_slow - (r_fast * t).exp_m1() / r_fast)
                    / (r_slow - r_fast),
                ((r_slow * t).exp() - (r_fast * t).exp()) / (r_slow - r_fast),
            )
        }
    };
    for case in FEEDBACK_CASES {
        let (scale, _) = case.frequency();
        // Та же шкала, что у прежней проверки точности. При сгущении
        // длительность и амплитуда одного физического толчка совпадают.
        let hz = (64.0 * scale).ceil() as u64;
        let duration = 1.0 / hz as f64;
        let steps = (hz as f64 / scale).floor() as u64;
        let analytic = |t: f64| {
            let j = 1e-6;
            match case {
                BenchFeedback::Coupling => {
                    let rotor = 0.2;
                    let (z, dz) = step(t, 0.0, 5.0 * (1.0 / j + 1.0 / rotor));
                    // Движение центра инерции плюс относительная мода.
                    let sum = j + rotor;
                    (
                        (j * t * t / 2.0 + rotor * z) / sum,
                        (j * t + rotor * dz) / sum,
                    )
                }
                BenchFeedback::StopDamper => step(t, 0.005 / j, 0.001 / j),
                BenchFeedback::StopSpring | BenchFeedback::Lock | BenchFeedback::Brake => {
                    step(t, 0.0, 5.0 / j)
                }
                BenchFeedback::BackEmf | BenchFeedback::Friction => step(t, 0.005 / j, 0.0),
            }
        };
        let mut error = Vec::new();
        for refine in [1, 2, 4, 8] {
            let trace = case.trajectory_with_kick(hz * refine, steps * refine, refine);
            assert_eq!(trace.len(), (steps * refine + 1) as usize);
            let mut worst = [0.0_f64; 2];
            for (index, &(q, v)) in trace.iter().step_by(refine as usize).enumerate().skip(1) {
                let at = index as f64 * duration;
                let (a, b) = analytic(at);
                let (c, d) = analytic(at - duration);
                worst[0] = worst[0].max(((a - c) / duration - q).abs() * scale);
                worst[1] = worst[1].max(((b - d) / duration - v).abs());
            }
            error.push(worst);
        }
        println!("{case:?}: сгущение 1,2,4,8, [Δq·ω, Δv] {error:.7?}");
        for axis in 0..2 {
            assert!((0.0..0.02).contains(&error[0][axis]));
            for pair in error.windows(2) {
                assert!(pair[1][axis] < pair[0][axis], "{case:?}: {error:?}");
            }
        }
    }
}

#[test]
fn a_command_arriving_after_a_controller_tick_waits_for_the_next_tick() {
    let dir = Temp::new();
    let id = "timed-block";
    // При физике 2 кГц такт регулятора на ⅔ мс обрабатывается в 1 мс.
    // Сообщение на 0.9 мс должно дождаться такта на 1⅓ мс.
    let file = position_file(id, "dc", "pwm", false, 0.0009)
        .replace(
            "control_hz = { value = 1000.0",
            "control_hz = { value = 1500.0",
        )
        .replace(
            "inductance_h = { value = 0.5",
            "inductance_h = { value = 0.0",
        );
    dir.write(id, &file);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    w.physics = Rate::hz(2000).unwrap();
    w.mjcf = w.mjcf.replace("0.0001", "0.0005");
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    command(&mut r, p.command, 1.0);
    go(&mut r, 0.001);
    assert_eq!(last(&r, "блок/привод/accepted_command"), 1.0);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    go(&mut r, 0.0015);
    assert_eq!(last(&r, "блок/привод/current_a"), 4.0);
}

#[test]
fn drive_energy_diagnostic_uses_work_actually_applied_to_the_world() {
    let (_dir, catalog, w) = BenchFeedback::Friction.setup(1_000_000);
    let mut r = Runtime::load(w.clone()).unwrap();
    install(&mut r, &w, &catalog).unwrap();
    r.record_joint_state("координаты").unwrap();
    let kick = r.channel("толчок").unwrap();
    command(&mut r, kick, 1.0);
    go(&mut r, 0.000001);
    assert_eq!(last(&r, "qvel[0]"), 1.0);
    // За первый шаг привод передал нулевой момент. В самом приводе
    // при средней скорости 0.5 модель учла потерю F·v·h = 1.25 нДж.
    // Расхождение с переданной работой должно попасть в невязку.
    assert_eq!(last(&r, "блок/привод/motor_energy_residual_j"), 0.0);
    let actual = last(&r, "блок/привод/world_energy_residual_j");
    assert!((actual - (-1.25e-9)).abs() < 1e-19, "{actual:.12e} Дж");
}

#[test]
fn combined_feedback_on_one_axis_consumes_the_same_step_budget() {
    let (_dir, catalog, mut w) = BenchFeedback::Friction.setup(12_000);
    // По множителю затухания одной ветви нужно 10138 Гц, двум 20275 Гц.
    // Выбираем сетку между порогами без изменений массы и armature.
    let second = CompiledDrive {
        name: "блок/второй привод".into(),
        effort: "резерв".into(),
        ..w.drives[0].clone()
    };
    w.efforts.push(EffortChannel {
        name: second.effort.clone(),
        ..w.efforts[0].clone()
    });
    let mut single = Runtime::load(w.clone()).unwrap();
    install(&mut single, &w, &catalog).unwrap();
    go(&mut single, 0.0);

    w.drives.push(second);
    let mut double = Runtime::load(w.clone()).unwrap();
    install(&mut double, &w, &catalog).unwrap();
    assert_eq!(
        double.run_until(Time::ZERO).unwrap(),
        Status::ComponentFailure
    );
    assert!(
        double.failure().unwrap().contains("≥ 20275 Гц"),
        "{:?}",
        double.failure()
    );
}

#[test]
fn new_pose_can_fail_the_bound_without_sensors_refreshing_the_engine() {
    let dir = Temp::new();
    let id = "mobile-block";
    // Аналитически J_eff(0)=1.375, J_eff(π)=0.7083̅. Порог
    // для k=122880 Н·м/рад при 1 кГц равен J_eff≥1.024 кг·м².
    // k = 12·0.01·f²·J_eff, из бюджета фазы средней точки.
    let file = position_file(id, "dc", "pwm", false, 0.0)
        .replace(
            "rotor_inertia_kg_m2 = { value = 0.2",
            "rotor_inertia_kg_m2 = { value = 0.25",
        )
        .replace("stop = \"none\"", "stop = \"compliant\"")
        + r#"
stop_min = { value = -100.0, source = "derived", reference = "Далёкий упор стенда" }
stop_max = { value = 100.0, source = "derived", reference = "Далёкий упор стенда" }
stop_stiffness = { value = 122880.0, source = "derived", reference = "Порог J в этом тесте" }
stop_damping = { value = 0.0, source = "derived", reference = "Только упругость" }
"#;
    dir.write(id, &file);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.25);
    w.physics = Rate::hz(1000).unwrap();
    w.mjcf = r#"<mujoco><option timestep="0.001" gravity="0 0 0"/>
        <worldbody><body name="блок">
            <joint name="ось" axis="0 0 1" armature="0.25"/>
            <inertial pos="0 0 0" mass="1" diaginertia="0.25 0.25 0.25"/>
            <body name="стержень" pos="1 0 0">
                <joint name="вторая ось" axis="0 0 1" armature="0.25"/>
                <inertial pos="1 0 0" mass="1" diaginertia="0.25 0.25 0.25"/>
            </body></body></worldbody></mujoco>"#
        .into();
    w.joints.push(CompiledJoint {
        name: "вторая ось".into(),
        owner: "стержень".into(),
        kind: CompiledJointKind::Hinge,
    });
    w.efforts.push(EffortChannel {
        name: "поворот".into(),
        joint: "вторая ось".into(),
        dof: 0,
    });
    let mut r = Runtime::load(w.clone()).unwrap();
    install(&mut r, &w, &catalog).unwrap();
    go(&mut r, 0.0);
    // Достаточный импульс для поворота второго стержня через область отказа.
    let turn = r.channel("поворот").unwrap();
    command(&mut r, turn, 10.0);
    go(&mut r, 0.05);
    let mut fork = r.fork().unwrap();
    for run in [&mut r, &mut fork] {
        assert_eq!(
            run.run_until(Time(2_000_000_000)).unwrap(),
            Status::ComponentFailure
        );
        assert!(
            run.failure().unwrap().contains("stop_stiffness"),
            "{:?}",
            run.failure()
        );
        assert!(run.steps() >= 51);
    }
    println!(
        "После изменения позы, {} шагов. {}",
        r.steps(),
        r.failure().unwrap()
    );
    assert_eq!(r.failure(), fork.failure());
    assert_eq!(bytes(&r), bytes(&fork));
}

#[test]
fn held_spring_is_rejected_with_rk4_even_at_a_fine_step() {
    let (_dir, catalog, mut w) = BenchFeedback::Coupling.setup(1_000_000);
    w.mjcf = w.mjcf.replace("Euler", "RK4");
    let mut r = Runtime::load(w.clone()).unwrap();
    install(&mut r, &w, &catalog).unwrap();
    assert_eq!(r.run_until(Time::ZERO).unwrap(), Status::ComponentFailure);
    assert!(
        r.failure().unwrap().contains("RK4") && r.failure().unwrap().contains("coupling_stiffness")
    );
    assert_eq!(r.steps(), 0);
}

#[test]
fn free_coupling_conserves_momentum_and_bounds_energy_over_many_periods() {
    // Две одинаковые инерции J=10⁻⁶, k=4.8: ω=√(2k/J).
    // Сетка прежней проверки 200 кГц. Без потерь суммарный
    // импульс V₁+V₂ равен 1, а нормированная энергия
    // V₁²+V₂²+(k/J)(q₁−q₂)² равна 1.
    let (_dir, catalog, mut w) = BenchFeedback::Coupling.setup(200_000);
    w.drives[0]
        .params
        .insert("rotor_inertia_kg_m2".into(), 1e-6);
    w.drives[0].params.insert("coupling_stiffness".into(), 4.8);
    let rate = w.physics;
    let omega = (2.0 * 4.8 / 1e-6_f64).sqrt();
    let end = Time::from_secs(300.0 * std::f64::consts::TAU / omega).unwrap();
    let mut r = Runtime::load(w.clone()).unwrap();
    install(&mut r, &w, &catalog).unwrap();
    r.record_joint_state("координаты").unwrap();
    let kick = r.channel("толчок").unwrap();
    command(&mut r, kick, 1e-6 * rate.as_hz());
    assert_eq!(r.run_until(rate.tick(1)).unwrap(), Status::Running);
    command(&mut r, kick, 0.0);
    assert_eq!(r.run_until(end).unwrap(), Status::Running);
    let h = 1.0 / rate.as_hz();
    let torque = trace(&r, "блок/привод/joint_torque_nm");
    let world_speed = trace(&r, "qvel[0]");
    let rotor_speed = trace(&r, "блок/привод/motor_rad_s");
    let mut impulse_error = 0.0_f64;
    for ((transmitted, world), rotor) in torque
        .iter()
        .zip(world_speed.windows(2))
        .zip(rotor_speed.windows(2))
        .skip(1)
    {
        let impulse = transmitted.1 * h;
        let world_error = (1e-6 * (world[1].1 - world[0].1) - impulse).abs();
        let rotor_error = (1e-6 * (rotor[1].1 - rotor[0].1) + impulse).abs();
        impulse_error = impulse_error.max(world_error).max(rotor_error);
    }
    println!("Невязка импульса по опубликованному усилию: {impulse_error:.9e} Н·м·с");
    assert!(impulse_error < 16.0 * f64::EPSILON * 1e-6);
    // После внешнего толчка Δx в координате x=q−hv/2 равно h/2.
    let initial_shadow_energy = 1.0 + (4.8 / 1e-6) * (h / 2.0).powi(2);
    let mut errors = [0.0_f64; 3];
    for ((((.., q), (.., v)), (.., qm)), (.., vm)) in trace(&r, "qpos[0]")
        .into_iter()
        .zip(trace(&r, "qvel[0]"))
        .zip(trace(&r, "блок/привод/motor_rad"))
        .zip(trace(&r, "блок/привод/motor_rad_s"))
        .skip(1)
    {
        errors[0] = errors[0].max((v + vm - 1.0).abs());
        errors[1] = errors[1].max((v * v + vm * vm + 4.8 / 1e-6 * (q - qm).powi(2) - 1.0).abs());
        let x = (q - qm) - 0.5 * h * (v - vm);
        let shadow = v * v + vm * vm + 4.8 / 1e-6 * x * x;
        errors[2] = errors[2].max((shadow / initial_shadow_energy - 1.0).abs());
    }
    println!(
        "300 периодов: max Δp/p={:.9e}, max ΔE/E={:.9e}, max ΔE(x)/E(x)={:.9e}",
        errors[0], errors[1], errors[2]
    );
    // Округление сумм импульса и колебание энергии симплектического шага.
    assert!(
        errors[0] < 1e-10 && errors[1] < 0.01 && errors[2] < 1e-9,
        "{errors:?}"
    );
}

#[test]
fn compliant_rotor_with_a_joint_limit_restores_forks_and_replays_bitwise() {
    let (_dir, catalog, mut w) = BenchFeedback::Coupling.setup(20_000);
    // Верхний предел синтетического блока, до него доезжает ротор под питанием.
    // Мост 1 МГц по тому же сценарию, его напряжение усредняется.
    w.mjcf = w.mjcf.replace(
        r#"armature="0""#,
        r#"armature="0" limited="true" range="-1 0.01""#,
    );
    w.drives[0].params.insert("control_hz".into(), 1_000_000.0);
    let load = || {
        let mut runtime = Runtime::load(w.clone()).unwrap();
        install(&mut runtime, &w, &catalog).unwrap();
        runtime.record_joint_state("координаты").unwrap();
        runtime
    };
    let mut r = load();
    let channel = r.channel("команда").unwrap();
    command(&mut r, channel, 1.0);
    go(&mut r, 0.03);
    // Угол в MJCF задан в градусах. Предел 0.01° пройден,
    // ограничение MuJoCo создаёт конечную реакцию пробному шагу.
    assert!(last(&r, "qpos[0]") > 0.01_f64.to_radians());
    assert!(last(&r, "блок/привод/joint_torque_nm") > 0.0);
    let snapshot = r.snapshot();
    let mut fork = r.fork().unwrap();
    let end = Time::from_secs(0.05).unwrap();
    for run in [&mut fork, &mut r] {
        command(run, channel, -1.0);
        assert_eq!(
            run.run_until(end).unwrap(),
            Status::Running,
            "{:?}",
            run.failure()
        );
    }
    let expected = bytes(&r);
    assert_eq!(bytes(&fork), expected);
    r.restore(&snapshot).unwrap();
    command(&mut r, channel, -1.0);
    assert_eq!(r.run_until(end).unwrap(), Status::Running);
    assert_eq!(bytes(&r), expected);
    let mut replay = load();
    replay.replay(r.log().clone()).unwrap();
    assert_eq!(replay.run_until(end).unwrap(), Status::Running);
    replay.verify_replay().unwrap();
    assert_eq!(bytes(&replay), expected);
}

#[test]
fn restoring_power_does_not_run_a_controller_tick_from_before_restore() {
    let dir = Temp::new();
    let id = "switched-block";
    let file = position_file(id, "dc", "pwm", false, 0.0)
        .replace(
            "control_hz = { value = 1000.0",
            "control_hz = { value = 1500.0",
        )
        .replace(
            "inductance_h = { value = 0.5",
            "inductance_h = { value = 0.0",
        )
        .replace("brake = \"none\"", "brake = \"none\"\npower_input = true");
    dir.write(id, &file);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    w.physics = Rate::hz(2000).unwrap();
    w.mjcf = w.mjcf.replace("0.0001", "0.0005");
    w.efforts.push(EffortChannel {
        name: "команда-power".into(),
        ..w.efforts[0].clone()
    });
    let mut r = Runtime::load(w.clone()).unwrap();
    let p = install(&mut r, &w, &catalog).unwrap().remove(0);
    let power = p.power.unwrap();
    command(&mut r, p.command, 1.0);
    go(&mut r, 0.0005);
    command(&mut r, power, 1.0);
    // Питание впервые видно на такте физики 1 мс. Тик регулятора
    // на ⅔ мс был раньше, следующий, на 1⅓ мс, виден в 1.5 мс.
    go(&mut r, 0.001);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    go(&mut r, 0.0015);
    assert_eq!(last(&r, "блок/привод/current_a"), 4.0);
    command(&mut r, power, 0.0);
    go(&mut r, 0.002);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    assert_eq!(last(&r, "блок/привод/joint_torque_nm"), 0.0);
}

fn with_inputs(world: &mut CompiledWorld, suffixes: &[&str]) {
    let output = world.efforts[0].clone();
    world
        .efforts
        .extend(suffixes.iter().map(|suffix| EffortChannel {
            name: format!("{}{suffix}", output.name),
            ..output.clone()
        }));
}

#[test]
fn bus_voltage_scales_held_duty_and_rejects_invalid_inputs() {
    let dir = Temp::new();
    let id = "block-bus";
    dir.write(id, &position_file(id, "dc", "pwm", false, 0.0));
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    with_inputs(&mut w, &["-bus-v"]);
    let mut r = Runtime::load(w.clone()).unwrap();
    let h = install(&mut r, &w, &catalog).unwrap().remove(0);
    r.record_joint_state("координаты").unwrap();

    let bus = h.bus_voltage.unwrap();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            r.act(&Command {
                targets: vec![(bus, bad)]
            })
            .is_err()
        );
    }
    // Стенд задаёт скважность 1/2. До записи на шине номинальные 8 В.
    command(&mut r, h.command, 0.5);
    go(&mut r, 0.0002);
    assert_eq!(last(&r, "блок/привод/voltage_v"), 4.0);
    let speed_before = last(&r, "qvel[0]");
    let current_before = last(&r, "блок/привод/current_a");
    command(&mut r, bus, 2.0);
    go(&mut r, 0.0003);
    assert_eq!(last(&r, "блок/привод/voltage_v"), 4.0);
    let speed_start = last(&r, "qvel[0]");
    let current_start = last(&r, "блок/привод/current_a");
    assert!(speed_start > speed_before && current_start > current_before);
    go(&mut r, 0.0004);
    // Изменение шины действует до следующего такта обновления скважности (1 мс).
    assert_eq!(last(&r, "блок/привод/duty"), 0.5);
    assert_eq!(last(&r, "блок/привод/bus_voltage_v"), 2.0);
    assert_eq!(last(&r, "блок/привод/voltage_v"), 1.0);
    let emf = 0.5 * (speed_start + last(&r, "qvel[0]"));
    let steady = (1.0 - emf) / 2.0;
    let expected = steady + (current_start - steady) * (-0.0004_f64).exp();
    assert!((last(&r, "блок/привод/current_a") - expected).abs() < 1e-14);
    // У жёсткой передачи стенда K=1, N=1, трение отсутствует.
    assert!((last(&r, "блок/привод/end_joint_torque_nm") - expected).abs() < 1e-14);

    let snapshot = r.snapshot();
    for bad in [-0.001, 8.001] {
        r.restore(&snapshot).unwrap();
        command(&mut r, bus, bad);
        assert_ne!(r.run_until(Time(500_000)).unwrap(), Status::Running);
        assert!(format!("{:?}", r.failure()).contains("вне диапазона"));
    }
    // Верхняя и нижняя границы замкнутого диапазона допустимы.
    r.restore(&snapshot).unwrap();
    command(&mut r, bus, 0.0);
    go(&mut r, 0.0006);
    assert_eq!(last(&r, "блок/привод/voltage_v"), 0.0);

    let source = position_file(id, "dc", "pwm", false, 0.0);
    let without_range = source
        .lines()
        .filter(|line| !line.starts_with("voltage_min_v"))
        .collect::<Vec<_>>()
        .join("\n");
    dir.write(id, &without_range);
    let missing = DriveCatalog::load(&dir.0).unwrap();
    let mut r = Runtime::load(w.clone()).unwrap();
    let error = install(&mut r, &w, &missing).unwrap_err();
    assert!(error.contains("voltage_min_v") && error.contains("на стенде"));
}

#[test]
fn bridge_commands_select_diode_coast_and_short_in_mujoco() {
    use sim_drive::BridgeCommand::{Coast, Drive, Short};

    let dir = Temp::new();
    let id = "block-bridge";
    let source = position_file(id, "dc", "pwm", false, 0.0)
        .replace("brake = \"none\"", "brake = \"none\"\npower_input = true");
    dir.write(id, &source);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    with_inputs(&mut w, &["-power", "-bridge"]);
    let mut r = Runtime::load(w.clone()).unwrap();
    let h = install(&mut r, &w, &catalog).unwrap().remove(0);
    r.record_joint_state("координаты").unwrap();
    let mode = h.bridge_mode.unwrap();
    let power = h.power.unwrap();
    command(&mut r, power, 1.0);
    command(&mut r, h.command, 1.0);
    go(&mut r, 0.001);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    command(&mut r, mode, Drive.value());
    go(&mut r, 0.101);
    assert!(last(&r, "блок/привод/current_a") > 1.0);
    assert!(last(&r, "блок/привод/bus_power_w") > 8.0);
    let snapshot = r.snapshot();
    let initial_bus = last(&r, "блок/привод/bus_energy_j");
    command(&mut r, mode, Coast.value());
    go(&mut r, 0.102);
    assert_eq!(last(&r, "блок/привод/bridge_mode"), Coast.value());
    assert_eq!(last(&r, "блок/привод/voltage_v"), -8.0);
    assert!(last(&r, "блок/привод/current_a") > 1.0);
    assert!(last(&r, "блок/привод/bus_energy_j") < initial_bus);
    assert_eq!(last(&r, "блок/привод/bridge_dump_j"), 0.0);
    go(&mut r, 0.2);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);
    assert_eq!(last(&r, "блок/привод/bus_power_w"), 0.0);
    assert!(last(&r, "блок/привод/motor_energy_residual_j").abs() < 1e-10);

    r.restore(&snapshot).unwrap();
    command(&mut r, mode, Short.value());
    go(&mut r, 0.102);
    assert_eq!(last(&r, "блок/привод/bridge_mode"), Short.value());
    assert_eq!(last(&r, "блок/привод/voltage_v"), 0.0);
    let short_bus = last(&r, "блок/привод/bus_energy_j");
    let initial_copper = last(&r, "блок/привод/copper_loss_j");
    go(&mut r, 1.4);
    assert!(last(&r, "блок/привод/current_a") < 0.0);
    assert!(last(&r, "блок/привод/joint_torque_nm") < 0.0);
    assert_eq!(last(&r, "блок/привод/bus_energy_j"), short_bus);
    assert!(last(&r, "блок/привод/copper_loss_j") > initial_copper);
    assert!(last(&r, "блок/привод/motor_energy_residual_j").abs() < 1e-10);
    // Для J=1 правый Euler делает на F²h²/(2J) больше работы,
    // чем средняя скорость внутреннего расчёта. Последнее усилие ещё не приложено.
    let h_step = 1.0 / w.physics.as_hz();
    let forces = trace(&r, "блок/привод/joint_torque_nm");
    let expected = -0.5
        * h_step
        * h_step
        * forces[..forces.len() - 1]
            .iter()
            .map(|(_, f)| f * f)
            .sum::<f64>();
    let measured = last(&r, "блок/привод/world_energy_residual_j");
    assert!(
        expected < 0.0 && (measured - expected).abs() < 1e-10,
        "{measured:.12e} против {expected:.12e}"
    );

    r.restore(&snapshot).unwrap();
    command(&mut r, mode, Short.value());
    command(&mut r, power, 0.0);
    go(&mut r, 0.102);
    assert_eq!(last(&r, "блок/привод/bridge_mode"), Coast.value());
    command(&mut r, mode, 3.0);
    assert_ne!(r.run_until(Time(102_100_000)).unwrap(), Status::Running);
    assert!(format!("{:?}", r.failure()).contains("режим моста"));
}

#[test]
fn initial_temperatures_set_resistance_and_have_instance_provenance() {
    let dir = Temp::new();
    let id = "block-thermal";
    dir.write(id, &position_file(id, "dc", "pwm", false, 0.0));
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    with_inputs(&mut w, &["-bus-v"]);
    // Синтетическая сеть с двумя различными начальными температурами.
    let params = &mut w.drives[0].params;
    *params = [
        ("initial_winding_temp_c", 60.0),
        ("initial_housing_temp_c", 40.0),
        ("ambient_c", 20.0),
        ("resistance_ref_c", 20.0),
        ("resistance_alpha_per_k", 0.004),
        ("r_th_winding_housing_k_per_w", 2.0),
        ("r_th_housing_ambient_k_per_w", 4.0),
        ("tau_winding_s", 2.0),
        ("tau_housing_s", 4.0),
        ("winding_loss_fraction", 0.5),
        ("gear_loss_to_housing_fraction", 0.0),
    ]
    .into_iter()
    .map(|(key, value)| (key.into(), value))
    .collect();
    let mut r = Runtime::load(w.clone()).unwrap();
    let h = install(&mut r, &w, &catalog).unwrap().remove(0);
    assert!(h.winding_c.is_some() && h.housing_c.is_some());
    for name in ["initial_winding_temp_c", "initial_housing_temp_c"] {
        assert!(
            r.simplifications().iter().any(|s| s.blocks_proof
                && s.text.contains(name)
                && s.text.contains("происхождение"))
        );
    }
    go(&mut r, 0.0);
    assert_eq!(last(&r, "блок/привод/winding_c"), 60.0);
    assert_eq!(last(&r, "блок/привод/housing_c"), 40.0);
    assert!((last(&r, "блок/привод/resistance_ohm") - 2.32).abs() < 1e-14);
    go(&mut r, 0.001);
    assert!(last(&r, "блок/привод/winding_c") < 60.0);
    assert!(last(&r, "блок/привод/housing_c") > 40.0);

    for (key, value) in [
        ("initial_housing_temp_c", -274.0),
        ("initial_winding_temp_c", f64::NAN),
        ("initial_winding_temp_c", f64::INFINITY),
        ("initial_winding_temp_c", -250.0),
    ] {
        let mut invalid = w.clone();
        invalid.drives[0].params.insert(key.into(), value);
        let mut r = if value.is_finite() {
            Runtime::load(invalid.clone()).unwrap()
        } else {
            assert!(matches!(
                Runtime::load(invalid.clone()),
                Err(reason) if reason.contains("неконечные параметры")
            ));
            Runtime::load(w.clone()).unwrap()
        };
        assert!(install(&mut r, &invalid, &catalog).is_err());
        assert!(r.truth_id("блок/привод/current_a").is_none());
    }
    w.drives[0].params.remove("tau_housing_s");
    let mut r = Runtime::load(w.clone()).unwrap();
    let err = install(&mut r, &w, &catalog).unwrap_err();
    assert!(err.contains("tau_housing_s") && err.contains("на стенде"));
}

#[test]
fn pulse_frames_refresh_timeout_and_reject_out_of_calibration_without_reusing_targets() {
    let dir = Temp::new();
    let id = "block-pulse";
    let mut source = position_file(id, "hobby_servo", "position", false, 0.002);
    // Синтетический стенд с обратным направлением координаты.
    source.push_str(r#"
pulse_period_min_s = { value = 0.005, source = "derived", reference = "Синтетический стенд" }
pulse_period_max_s = { value = 0.015, source = "derived", reference = "Синтетический стенд" }
pulse_width_min_s = { value = 0.001, source = "derived", reference = "Синтетический стенд" }
pulse_width_max_s = { value = 0.003, source = "derived", reference = "Синтетический стенд" }
pulse_position_at_min_width = { value = 1.0, source = "derived", reference = "Синтетический стенд" }
pulse_position_at_max_width = { value = -1.5, source = "derived", reference = "Синтетический стенд" }
pulse_timeout_s = { value = 0.02, source = "derived", reference = "Синтетический стенд" }
"#);
    dir.write(id, &source);
    let catalog = DriveCatalog::load(&dir.0).unwrap();
    let mut w = world(id, CompiledJointKind::Hinge, 0.2);
    with_inputs(&mut w, &["-pulse-period", "-pulse-width", "-pulse-seq"]);
    w.drives[0]
        .params
        .insert("pulse_position_at_max_width".into(), -1.0);
    let mut r = Runtime::load(w.clone()).unwrap();
    let h = install(&mut r, &w, &catalog).unwrap().remove(0);
    let ids = h.pulse.unwrap();
    let frame = |r: &mut Runtime, period, width, sequence| {
        r.act(&Command {
            targets: vec![
                (ids.period_s, period),
                (ids.width_s, width),
                (ids.sequence, sequence),
            ],
        })
        .unwrap();
    };
    let present = |r: &Runtime| last(r, "блок/привод/command_present");
    let accepted = |r: &Runtime| last(r, "блок/привод/accepted_command");
    frame(&mut r, 0.01, 0.0015, 0.0);
    go(&mut r, 0.0019);
    assert_eq!(present(&r), 0.0);
    go(&mut r, 0.002);
    assert_eq!(present(&r), 1.0);
    assert!((accepted(&r) - 0.5).abs() < 1e-14);
    go(&mut r, 0.01);
    frame(&mut r, 0.01, 0.0015, 1.0);
    go(&mut r, 0.025);
    assert_eq!(present(&r), 1.0);
    let snapshot = r.snapshot();
    let mut branch = r.fork().unwrap();
    go(&mut r, 0.0301);
    assert_eq!(present(&r), 1.0);
    go(&mut r, 0.0302);
    go(&mut branch, 0.0302);
    assert_eq!(bytes(&r), bytes(&branch));
    assert_eq!(present(&r), 0.0);
    assert_eq!(last(&r, "блок/привод/current_a"), 0.0);

    // Кадр с допустимым периодом оказывается в задержке. Следующий,
    // с шириной вне диапазона, удаляет его из очереди.
    frame(&mut r, 0.01, 0.003, 2.0);
    go(&mut r, 0.0303);
    frame(&mut r, 0.01, 0.004, 3.0);
    go(&mut r, 0.034);
    assert_eq!(present(&r), 0.0);
    assert_eq!(accepted(&r), 0.5);
    frame(&mut r, 0.01, 0.003, 4.0);
    go(&mut r, 0.037);
    assert_eq!(present(&r), 1.0);
    assert_eq!(accepted(&r), -1.0);
    frame(&mut r, 0.020, 0.003, 5.0);
    go(&mut r, 0.0371);
    assert_eq!(present(&r), 0.0);

    r.restore(&snapshot).unwrap();
    frame(&mut r, 0.01, 0.002, 0.0);
    assert_ne!(r.run_until(Time(25_100_000)).unwrap(), Status::Running);
    assert!(format!("{:?}", r.failure()).contains("номер импульса уменьшился"));

    r.restore(&snapshot).unwrap();
    frame(&mut r, 0.01, 0.011, 2.0);
    assert_ne!(r.run_until(Time(25_100_000)).unwrap(), Status::Running);
    assert!(format!("{:?}", r.failure()).contains("ширина ≤ период"));

    let source_without_timeout = source
        .lines()
        .filter(|line| !line.starts_with("pulse_timeout_s"))
        .collect::<Vec<_>>()
        .join("\n");
    dir.write(id, &source_without_timeout);
    let missing = DriveCatalog::load(&dir.0).unwrap();
    let mut r = Runtime::load(w.clone()).unwrap();
    let error = install(&mut r, &w, &missing).unwrap_err();
    assert!(error.contains("pulse_timeout_s") && error.contains("на стенде"));
    w.efforts.pop();
    let mut r = Runtime::load(w.clone()).unwrap();
    assert!(
        install(&mut r, &w, &catalog)
            .unwrap_err()
            .contains("-pulse-seq")
    );
}

#[derive(Clone, Copy, Debug)]
enum CircuitPath {
    Common,
    Sampled,
}

impl CircuitPath {
    fn setup(self, initial_speed: f64) -> (Runtime, ChannelId, ChannelId) {
        self.setup_inertia(initial_speed, 8e8)
    }

    fn setup_inertia(
        self,
        initial_speed: f64,
        body_inertia: f64,
    ) -> (Runtime, ChannelId, ChannelId) {
        // Синтетический стенд: R=2 Ом, L=2 мГн, k=1, N=1, без потерь передачи.
        // Инерция зажима 8·10⁸ кг·м² удерживает скорость с изменением < 10⁻⁹ рад/с
        // при моменте до 10 Н·м за проверяемые интервалы до 20 мс.
        let id = "circuit-block";
        let mut w = world(id, CompiledJointKind::Hinge, 0.2);
        w.mjcf = w.mjcf.replace(
            "0.8 0.8 0.8",
            &format!("{body_inertia} {body_inertia} {body_inertia}"),
        );
        with_inputs(&mut w, &["-bus-v", "-power"]);
        w.efforts[2].name = "блок/разрешение".into();
        if matches!(self, Self::Common) {
            w.efforts[2].name = "команда-power".into();
        }
        let mut r = Runtime::load(w.clone()).unwrap();
        r.set_initial_velocities(&BTreeMap::from([(
            "ось".into(),
            BTreeMap::from([(0, initial_speed)]),
        )]))
        .unwrap();
        let enable = r.channel(&w.efforts[2].name).unwrap();
        let (bus, current) = match self {
            Self::Common => {
                let dir = Temp::new();
                let f = position_file(id, "dc", "pwm", false, 0.0)
                    .replace(
                        "inductance_h = { value = 0.5",
                        "inductance_h = { value = 0.002",
                    )
                    .replace("brake = \"none\"", "brake = \"none\"\npower_input = true");
                dir.write(id, &f);
                let catalog = DriveCatalog::load(&dir.0).unwrap();
                let error = sim_drive::DriveHandle::install_with_circuit(
                    &mut r,
                    &w,
                    &catalog,
                    &["блок/привод", "блок/привод"],
                )
                .unwrap_err();
                assert!(error.contains("повторно"));
                let h = sim_drive::DriveHandle::install_with_circuit(
                    &mut r,
                    &w,
                    &catalog,
                    &["блок/привод"],
                )
                .unwrap()
                .remove(0);
                assert_eq!(h.power, Some(enable));
                h.circuit_port().unwrap()
            }
            Self::Sampled => {
                // Большие синтетические теплоёмкости C=τ/R=10⁶ Дж/К
                // сохраняют сопротивление стенда с точностью лучше 10⁻⁸ Ом.
                let spec: sim_drive::sampled::Spec = toml::from_str(
                    r#"
source = "Искусственный контур R–L и жёсткая передача"
bridge_effort = "разрешение"
ambient_c = 20
fixture_enabled = false
[motor]
nominal_v = 8
r_ohm = 2
r_ref_c = 20
l_h = 0.002
k = 1
j = 0.2
coulomb = 0
viscous = 0
band = 1
max_a = 100
[gear]
ratio = 1
eta_forward = 1
eta_backward = 1
friction = 0
j = 0
rated = 1
repeated = 2
peak = 3
mass = 1
[thermal]
r_wh = 1
r_ha = 1
tau_w = 1e6
tau_h = 1e6
max_c = 100
fraction = 0
"#,
                )
                .unwrap();
                use sim_drive::sampled::Measurement::SupplyVoltage;
                assert!(spec.initial_measurement(SupplyVoltage).is_err());
                assert_eq!(
                    spec.initial_measurement_with_circuit(SupplyVoltage, 4.0)
                        .unwrap(),
                    4.0
                );
                for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
                    assert!(
                        spec.initial_measurement_with_circuit(SupplyVoltage, bad)
                            .is_err()
                    );
                }
                sim_drive::sampled::install_with_circuit(&mut r, &w, "блок/привод", spec)
                    .unwrap()
                    .1
            }
        };
        assert_eq!(r.truth_id("блок/привод/next_bus_current_a"), Some(current));
        (r, bus, enable)
    }
}

const CIRCUIT_H: f64 = 0.0001; // Шаг стенда world(), 10 кГц.
const CIRCUIT_R: f64 = 2.0; // Ом, одинаково в обоих профилях.
const CIRCUIT_L: f64 = 0.002; // Гн, одинаково в обоих профилях.

fn motor_signal(runtime: &Runtime, name: &str) -> f64 {
    last(runtime, &format!("блок/привод/{name}"))
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.12e} вместо {expected:.12e}, допуск {tolerance:.1e}"
    );
}

fn rl_reference(initial: f64, terminal: f64, emf: f64, seconds: f64) -> (f64, f64) {
    // Независимое ручное решение: экспонента и первообразная R–L.
    let tau = CIRCUIT_L / CIRCUIT_R;
    let dc = (terminal - emf) / CIRCUIT_R;
    let a = (-seconds / tau).exp();
    (
        dc + (initial - dc) * a,
        dc * seconds + (initial - dc) * tau * (1.0 - a),
    )
}

struct ElectricalBench {
    circuit: sim_circuit::Transient,
    charge: f64,
    work: f64,
    mna_work: f64,
    voltage: f64,
    bus: ChannelId,
    wire: &'static str,
}

impl ElectricalBench {
    fn new(runtime: &mut Runtime, bus: ChannelId, feed_ohm: f64) -> Self {
        // Локальная опора на +2 В проверяет разность клемм.
        // Источник 4 В, регулируемое сопротивление проводки из параметра опыта.
        let (wire, branch) = if feed_ohm == 0.0 {
            ("Vwire", "Vwire src plus 0".into())
        } else {
            ("Rwire", format!("Rwire src plus {feed_ohm}"))
        };
        let netlist = format!(
            "Блок\nVref minus 0 2\nVfeed src minus EXT 4\n{branch}\nIload plus minus EXT 0\n"
        );
        let circuit = sim_circuit::Circuit::parse(&netlist).unwrap();
        let circuit = sim_circuit::Transient::new(&circuit, CIRCUIT_H).unwrap();
        let voltage = circuit.voltage("plus").unwrap() - circuit.voltage("minus").unwrap();
        // Численная проводимость MNA сдвигает рабочую точку на несколько пВ.
        close(voltage, 4.0, 1e-10);
        command(runtime, bus, voltage);
        go(runtime, 0.0);
        Self {
            circuit,
            charge: 0.0,
            work: 0.0,
            mna_work: 0.0,
            voltage,
            bus,
            wire,
        }
    }

    fn advance(&mut self, r: &mut Runtime, tick: u32, source: f64) {
        let load = motor_signal(r, "next_bus_current_a");
        let time = f64::from(tick) * CIRCUIT_H;
        self.circuit.set_source_step("Iload", load).unwrap();
        self.circuit.set_source_step("Vfeed", source).unwrap();
        self.circuit.advance_to(time).unwrap();
        let i_source = self.circuit.current("Vfeed").unwrap();
        let i_wire = self.circuit.current(self.wire).unwrap();
        let v_bus = self.circuit.voltage("plus").unwrap() - self.circuit.voltage("minus").unwrap();
        // Допуск узлового решения для токов стенда порядка единиц ампер.
        close(i_wire, load, 1e-9);
        close(i_source, -load, 1e-9);
        close(
            self.circuit.power("Vfeed").unwrap() + self.circuit.power(self.wire).unwrap(),
            -self.circuit.power("Iload").unwrap(),
            1e-9,
        );
        self.charge += i_wire * CIRCUIT_H;
        self.mna_work += self.circuit.power("Iload").unwrap() * CIRCUIT_H;
        self.work += self.voltage * load * CIRCUIT_H;
        self.voltage = v_bus;
        command(r, self.bus, self.voltage);
        go(r, time);
        // Погрешность суммы 200 шагов и MNA при двух выводах.
        close(motor_signal(r, "bus_charge_c"), self.charge, 2e-11);
        close(motor_signal(r, "bus_energy_j"), self.work, 2e-11);
        close(motor_signal(r, "motor_energy_residual_j"), 0.0, 2e-11);
    }
}

#[test]
fn circuit_drive_start_stall_and_release_match_rl_and_mna() {
    for path in [CircuitPath::Common, CircuitPath::Sampled] {
        let (mut r, bus, enable) = path.setup(0.0);
        let duty = r.channel("команда").unwrap();
        // Обратная скважность −1/2: ток обмотки отрицателен, ток источника
        // при пуске и заклинивании положителен. Vш = 4 В, Uобм = −2 В.
        command(&mut r, duty, -0.5);
        command(&mut r, enable, 1.0);
        // 1 мОм — сопротивление тестовой перемычки; затем 1 Ом.
        let mut bench = ElectricalBench::new(&mut r, bus, 0.001);
        let launch = motor_signal(&r, "next_bus_current_a");
        let (_, q_first) = rl_reference(0.0, -2.0, 0.0, CIRCUIT_H);
        close(launch, -0.5 * q_first / CIRCUIT_H, 1e-12);
        let mut predicted_current = 0.0;
        let mut predicted_charge = 0.0;
        let mut greatest_delta = 0.0_f64;
        for tick in 1..=120 {
            let terminal = -0.5 * bench.voltage;
            let (end, q) = rl_reference(predicted_current, terminal, 0.0, CIRCUIT_H);
            predicted_current = end;
            predicted_charge += -0.5 * q;
            bench.advance(&mut r, tick, 4.0);
            greatest_delta = greatest_delta.max((bench.work - bench.mna_work).abs());
        }
        // Каталожная четверть ВАХ: Iш = D²V/(R + D²Rпровода) при ω=0.
        let stall = 0.25 * 4.0 / (CIRCUIT_R + 0.25 * 0.001);
        close(motor_signal(&r, "current_a"), predicted_current, 1e-5);
        close(motor_signal(&r, "next_bus_current_a"), stall, 4e-6);
        close(bench.charge, predicted_charge, 2e-11);
        assert!(r.joint_velocity("ось", 0).unwrap().abs() < 1e-9);
        assert!(greatest_delta < 3e-8);
        println!(
            "{path:?}: пуск {launch:.9} А, заклинивание {:.9} А, Q {:.9} Кл, ∆E удержания {greatest_delta:.3e} Дж",
            motor_signal(&r, "next_bus_current_a"),
            bench.charge
        );

        // Снимаем разрешение при наличии напряжения. Начинается отдача
        // магнитной энергии через диоды, после нуля цепь обмотки размыкается.
        command(&mut r, enable, 0.0);
        bench.advance(&mut r, 121, 4.0);
        let at_release = bench.charge;
        let initial = rl_reference(
            predicted_current,
            -0.5 * (4.0 - stall * 0.001),
            0.0,
            CIRCUIT_H,
        )
        .0;
        let until_zero = (CIRCUIT_L / CIRCUIT_R) * (CIRCUIT_R * initial.abs() / 4.0).ln_1p();
        let (_, returned) = rl_reference(initial, 4.0, 0.0, until_zero);
        assert!(motor_signal(&r, "next_bus_current_a") < 0.0);
        let saved = r.snapshot();
        let mut fork = r.fork().unwrap();
        let circuit_saved = bench.circuit.snapshot();
        let saved_balances = (bench.voltage, bench.charge, bench.work, bench.mna_work);
        let mut branched = ElectricalBench {
            circuit: sim_circuit::Transient::new(
                &sim_circuit::Circuit::parse("Блок\nV1 x 0 1\nR1 x 0 1\n").unwrap(),
                CIRCUIT_H,
            )
            .unwrap(),
            ..bench
        };
        branched.circuit.restore(&circuit_saved);
        for tick in 122..=135 {
            bench.advance(&mut r, tick, 4.0);
            branched.advance(&mut fork, tick, 4.0);
        }
        assert_eq!(bytes(&r), bytes(&fork));
        let log = bytes(&r);
        r.restore(&saved).unwrap();
        bench.circuit.restore(&circuit_saved);
        (bench.voltage, bench.charge, bench.work, bench.mna_work) = saved_balances;
        for tick in 122..=135 {
            bench.advance(&mut r, tick, 4.0);
        }
        assert_eq!(bytes(&r), log);
        assert_eq!(motor_signal(&fork, "next_bus_current_a"), 0.0);
        close(bench.charge - at_release, returned, 2e-7);
        println!(
            "{path:?}: гашение, Q {:.9} Кл, t₀ {until_zero:.9} с",
            bench.charge - at_release
        );

        let (mut weak, bus, enable) = path.setup(0.0);
        let duty = weak.channel("команда").unwrap();
        command(&mut weak, duty, -0.5);
        command(&mut weak, enable, 1.0);
        let mut weak_bench = ElectricalBench::new(&mut weak, bus, 1.0);
        for tick in 1..=120 {
            weak_bench.advance(&mut weak, tick, 4.0);
        }
        let weak_stall = 0.25 * 4.0 / (CIRCUIT_R + 0.25);
        close(motor_signal(&weak, "next_bus_current_a"), weak_stall, 4e-6);
        close(weak_bench.voltage, 4.0 - weak_stall, 4e-6);
        let energy_difference = (weak_bench.work - weak_bench.mna_work).abs();
        println!(
            "{path:?}: 1 Ом, {:.9} В, {:.9} А, ∆E удержания {energy_difference:.3e} Дж",
            weak_bench.voltage,
            motor_signal(&weak, "next_bus_current_a")
        );
        assert!(energy_difference < 1.5e-5);
    }
}

#[test]
fn circuit_drive_regenerates_at_zero_bus_voltage_and_refuses_bad_voltage() {
    for path in [CircuitPath::Common, CircuitPath::Sampled] {
        let (mut r, bus, enable) = path.setup(10.0);
        let duty = r.channel("команда").unwrap();
        command(&mut r, duty, 1.0);
        command(&mut r, enable, 1.0);
        // E=10 В > Vш=4 В. При замкнутом мосте источник принимает ток.
        let mut bench = ElectricalBench::new(&mut r, bus, 0.0);
        let (_, q) = rl_reference(0.0, 4.0, 10.0, CIRCUIT_H);
        close(motor_signal(&r, "next_bus_current_a"), q / CIRCUIT_H, 1e-9);
        bench.advance(&mut r, 1, 4.0);
        assert!(bench.charge < 0.0 && bench.work < 0.0);
        // Источник опускает напряжение до нуля. После доставки нуля
        // на следующей границе остаётся только диодный путь.
        bench.advance(&mut r, 2, 0.0);
        assert_eq!(bench.voltage, 0.0);
        let (_, zero_q_after_front) = rl_reference(
            rl_reference(0.0, 4.0, 10.0, 2.0 * CIRCUIT_H).0,
            0.0,
            10.0,
            CIRCUIT_H,
        );
        close(
            motor_signal(&r, "next_bus_current_a"),
            zero_q_after_front / CIRCUIT_H,
            1e-9,
        );
        let energy_at_front = motor_signal(&r, "bus_energy_j");
        bench.advance(&mut r, 3, 0.0);
        assert_eq!(
            motor_signal(&r, "bus_energy_j").to_bits(),
            energy_at_front.to_bits()
        );

        // Идеальный зажим шины в ноль. Команда остаётся единичной,
        // отключение активного моста вызвано напряжением, а ток диодов сохраняется.
        let (mut zero, zero_bus, enable) = path.setup(10.0);
        let duty = zero.channel("команда").unwrap();
        command(&mut zero, duty, 1.0);
        command(&mut zero, enable, 1.0);
        command(&mut zero, zero_bus, 0.0);
        go(&mut zero, 0.0);
        let (_, zero_q) = rl_reference(0.0, 0.0, 10.0, CIRCUIT_H);
        let zero_i = motor_signal(&zero, "next_bus_current_a");
        close(zero_i, zero_q / CIRCUIT_H, 1e-9);
        go(&mut zero, CIRCUIT_H);
        close(motor_signal(&zero, "bus_charge_c"), zero_q, 1e-13);
        assert_eq!(motor_signal(&zero, "bus_energy_j"), 0.0);
        println!(
            "{path:?}: E=10 В, Iш при 4 В {:.9} А и при 0 В {zero_i:.9} А",
            q / CIRCUIT_H
        );

        let (mut invalid, bus, enable) = path.setup(0.0);
        command(&mut invalid, enable, 1.0);
        let saved = invalid.snapshot();
        assert_eq!(
            invalid.run_until(Time::ZERO).unwrap(),
            Status::ComponentFailure
        );
        assert!(
            invalid
                .failure()
                .unwrap()
                .contains("не поступило напряжение")
        );
        invalid.restore(&saved).unwrap();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                invalid
                    .act(&Command {
                        targets: vec![(bus, bad)]
                    })
                    .is_err()
            );
        }
        command(&mut invalid, bus, -1.0);
        assert_eq!(
            invalid.run_until(Time::ZERO).unwrap(),
            Status::ComponentFailure
        );
        assert!(invalid.failure().unwrap().contains("напряжение шины"));
    }
}

#[test]
fn circuit_drive_load_matches_moving_block_emf_on_the_same_interval() {
    for path in [CircuitPath::Common, CircuitPath::Sampled] {
        // J = 0.8 + 0.2 = 1 кг·м². Противо-ЭДС начинается с 10 В
        // и падает, когда регенерация тормозит свободный блок.
        let (mut r, bus, enable) = path.setup_inertia(10.0, 0.8);
        let duty = r.channel("команда").unwrap();
        command(&mut r, duty, 1.0);
        command(&mut r, enable, 1.0);
        let mut bench = ElectricalBench::new(&mut r, bus, 0.0);
        let mut predicted_current = 0.0;
        let mut mechanical = 0.0;
        for tick in 1..=100 {
            let start = r.joint_velocity("ось", 0).unwrap();
            let amps = motor_signal(&r, "next_bus_current_a");
            bench.advance(&mut r, tick, 4.0);
            let stop = r.joint_velocity("ось", 0).unwrap();
            let emf = match path {
                CircuitPath::Common => 0.5 * (start + stop),
                CircuitPath::Sampled => start,
            };
            let (current, charge) = rl_reference(predicted_current, 4.0, emf, CIRCUIT_H);
            predicted_current = current;
            // Допуск включает изменение R при нагреве дискретной обмотки.
            close(amps * CIRCUIT_H, charge, 2e-12);
            mechanical += emf * charge;
        }
        assert!(r.joint_velocity("ось", 0).unwrap() < 9.98);
        assert!(mechanical < 0.0);
        let storage = 0.5 * CIRCUIT_L * predicted_current.powi(2);
        let copper = motor_signal(&r, "copper_loss_j");
        close(bench.work, copper + mechanical + storage, 5e-10);
        close(bench.mna_work, bench.work, 2e-12);
    }
}
