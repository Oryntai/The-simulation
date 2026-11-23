use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use sim_mujoco::{Sim, ffi::mjtObj};
use sim_world::*;

struct Fixture(PathBuf);

impl Fixture {
    fn new(name: &str) -> Self {
        // Путь принадлежит только данному запуску теста; удаляем его при выходе.
        let path = std::env::temp_dir().join(format!("sim-world-{name}-{}", std::process::id()));
        fs::create_dir(&path).unwrap();
        Self(path)
    }

    fn write(&self, file: &str, data: impl AsRef<[u8]>) -> PathBuf {
        let path = self.0.join(file);
        fs::write(&path, data).unwrap();
        path
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn placement(file: &str) -> World {
    World {
        format: FORMAT,
        name: "Стенд".into(),
        seed: 19,
        physics_hz: 500,
        gravity_m_s2: [0.0, 0.0, -9.81],
        environment: Environment::default(),
        devices: vec![Placement {
            name: "стенд".into(),
            file: file.into(),
            pose: Pose::default(),
        }],
        objects: Vec::new(),
        attachments: Vec::new(),
        sensors: Vec::new(),
    }
}

fn compile(d: Device, file: &str) -> CompiledWorld {
    let input = DeviceInput {
        path: file.into(),
        device: d,
    };
    compile_world(
        Path::new("мир.toml"),
        &placement(file),
        &BTreeMap::from([("стенд".into(), input)]),
    )
    .unwrap()
}

fn near(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() <= eps, "получено {a}, ожидалось {b}")
}

#[test]
fn loader_resolves_world_and_device_relative_paths() {
    let tmp = Fixture::new("loader");
    fs::create_dir(tmp.0.join("models")).unwrap();
    let mesh = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";
    tmp.write("models/блок.obj", mesh);
    let scene = r#"
format = 3
name = "Мир с блоком"
seed = 19
physics_hz = 500
gravity_m_s2 = [0.0, 0.0, -9.81]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "стенд"
file = "models/блок.toml"
[devices.pose]
position_m = [1.0, 2.0, 3.0]
"#;
    let definition = r#"
format = 3
name = "Блок"
[[links]]
name = "блок"
[links.joint]
kind = "free"
[links.mass]
kind = "geometry"
[links.mass.material]
name = "условный материал"
density_kg_m3 = 6.0
provenance = { measured = "плотность синтетической сцены" }
[[links.collision]]
[links.collision.shape]
kind = "mesh"
file = "блок.obj"
"#;
    let path = tmp.write("мир.toml", scene);
    tmp.write("models/блок.toml", definition);
    let c = load_and_compile(&path).unwrap();
    assert_eq!(c, load_and_compile(&path).unwrap());
    assert_eq!(c.bodies[0].name, "стенд/блок");
    // Тетраэдр с рёбрами 1 м имеет объём 1/6 м³, барицентр (1/4,1/4,1/4).
    near(c.bodies[0].mass_kg, 1.0, 1e-14);
    assert_eq!(c.bodies[0].com_m, [0.25; 3]);
    // Для единичного тетраэдра Var(x)=3/80, Cov(x,y)=−1/80.
    // Тензор с диагональю 6/80 и внедиагональными 1/80 даёт {5,5,8}/80.
    for (a, b) in c.bodies[0]
        .inertia_kg_m2
        .into_iter()
        .zip([5.0 / 80.0, 5.0 / 80.0, 8.0 / 80.0])
    {
        near(a, b, 1e-14);
    }
    let report = verify_compiled(&c);
    assert_eq!(report.inertial, CheckStatus::Verified);
    assert!(matches!(report.dynamics, CheckStatus::NotAnalyzed(_)));
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    let id = sim.id(mjtObj::mjOBJ_BODY, "стенд/блок").unwrap();
    assert_eq!(sim.body_pose(id).0, [1.0, 2.0, 3.0]);
    tmp.write(
        "models/блок.toml",
        definition.replace("file = \"блок.obj\"", "file = \"нет.obj\""),
    );
    let e = load_and_compile(&path).unwrap_err();
    assert!(
        e.contains("мир.toml") && e.contains("блок.toml") && e.contains("collision[0].shape.file"),
        "{e}"
    );
}

#[test]
fn stl_binary_ascii_and_obj_are_read_without_external_parser() {
    let tmp = Fixture::new("mesh");
    let ascii = b"solid block\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid block\n";
    let a = load_mesh(&tmp.write("a.stl", ascii), [2.0, 3.0, 4.0]).unwrap();
    // 80 байт заголовка, 4 байта количества граней, 50 байт одной грани.
    let mut binary = vec![0; 80];
    binary[..5].copy_from_slice(b"solid");
    binary.extend(1_u32.to_le_bytes());
    for value in [0_f32, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
        binary.extend(value.to_le_bytes());
    }
    binary.extend(0_u16.to_le_bytes());
    let b = load_mesh(&tmp.write("b.stl", &binary), [2.0, 3.0, 4.0]).unwrap();
    let obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n";
    let c = load_mesh(&tmp.write("c.obj", obj), [2.0, 3.0, 4.0]).unwrap();
    assert_eq!(a, b);
    assert_eq!(b, c);
    assert_eq!(c.vertices_m, [[0.0; 3], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]]);
    assert_eq!(c.faces, [[0, 1, 2]]);
    binary[96..100].copy_from_slice(&f32::NAN.to_le_bytes());
    assert!(
        load_mesh(&tmp.write("bad.stl", binary), [1.0; 3])
            .unwrap_err()
            .contains("неконечная")
    );
    assert!(
        load_mesh(
            &tmp.write("bad.obj", b"v 0 0 inf\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
            [1.0; 3]
        )
        .unwrap_err()
        .contains("строка 1")
    );
    let nonconvex = b"v 0 0 0\nv 2 0 0\nv 1 1 0\nv 2 2 0\nv 0 2 0\nf 1 2 3 4 5\n";
    assert!(
        load_mesh(&tmp.write("concave.obj", nonconvex), [1.0; 3])
            .unwrap_err()
            .contains("невыпуклая")
    );
}

const URDF: &str = r#"<robot name="Каретка" version="1.0">
  <link name="стенд"/>
  <link name="каретка">
    <inertial>
      <origin xyz="0.2 0 0" rpy="0 0 1.5707963267948966"/>
      <mass value="2"/>
      <inertia ixx="0.2" iyy="0.3" izz="0.4" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision><geometry><box size="0.2 0.4 0.6"/></geometry></collision>
  </link>
  <joint name="ход" type="prismatic">
    <parent link="стенд"/><child link="каретка"/>
    <origin xyz="0 1 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/>
    <limit lower="0.1" upper="0.6" effort="10" velocity="0.8"/>
    <dynamics damping="0.2" friction="0.1"/>
  </joint>
</robot>"#;

#[test]
fn import_urdf_retains_joints_inertial_frame_and_limits() {
    let path = Path::new("каретка.urdf");
    let d = import_urdf(path, URDF).unwrap();
    let l = &d.links[1];
    assert_eq!(l.parent.as_deref(), Some("стенд"));
    assert_eq!(l.joint.limits_m, Some([0.1, 0.6]));
    assert_eq!(l.joint.effort_limit_si, Some(10.0));
    let Mass::Explicit { inertia_kg_m2, .. } = l.mass else {
        panic!("неявная масса")
    };
    // Поворот на π/2 меняет местами главные моменты X и Y.
    near(inertia_kg_m2[0], 0.3, 1e-14);
    near(inertia_kg_m2[1], 0.2, 1e-14);
    let c = compile(d, "каретка.urdf");
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    let id = sim.id(mjtObj::mjOBJ_BODY, "стенд/каретка").unwrap();
    let p = sim.body_pose(id).0;
    // Смещение 0,1 м вдоль локального X после поворота идёт в мировую Y.
    near(p[1], 1.1, 1e-14);
    assert!(
        c.simplifications
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("паспортные"))
    );
    for (suffix, reason) in [
        ("<sensor/>", "sensor"),
        ("<link name=\"стенд\"/>", "задано дважды"),
    ] {
        let invalid = URDF.replace("</robot>", &format!("{suffix}</robot>"));
        let e = import_urdf(path, &invalid).unwrap_err();
        assert!(e.contains("каретка.urdf") && e.contains(reason), "{e}");
    }
    let invalid = URDF.replace("value=\"2\"", "value=\"NaN\"");
    assert!(
        import_urdf(path, &invalid)
            .unwrap_err()
            .contains("/inertial/mass")
    );
}

const MJCF: &str = r#"<mujoco model="Маятник">
  <compiler angle="degree"/>
  <worldbody>
    <body name="маятник" pos="0.1 0.2 0.3" euler="20 30 40">
      <joint name="угол" type="hinge" axis="0 0 1" pos="0.02 0 0" ref="30" range="-45 90" damping="0.2"/>
      <geom type="box" size="0.1 0.2 0.3" pos="0.3 0.05 0" mass="2" contype="0" conaffinity="0"/>
      <geom type="sphere" size="0.1" pos="0.2 0 0" density="800" contype="0" conaffinity="0"/>
    </body>
    <body name="стержень" pos="0 1 0">
      <joint name="угол2" type="hinge"/>
      <inertial mass="1" pos="0 0 0" quat="0.7071067811865476 0 0 0.7071067811865476" diaginertia="0.13 0.14 0.17"/>
    </body>
  </worldbody>
  <equality><joint name="пара" joint1="угол2" joint2="угол" polycoef="0 -2 0 0 0"/></equality>
</mujoco>"#;

#[test]
fn import_mjcf_matches_the_directly_loaded_source_model() {
    let path = Path::new("маятник.xml");
    let d = import_mjcf(path, MJCF).unwrap();
    assert!(matches!(
        d.transmissions[0],
        Transmission::Gear { ratio: -2.0, .. }
    ));
    assert!(d.links[0].collision.is_empty() && d.links[0].visual.len() == 2);
    let c = compile(d, "маятник.xml");
    let actual = Sim::from_xml(&c.mjcf).unwrap();
    let reference = Sim::from_xml(MJCF).unwrap();
    for name in ["маятник", "стержень"] {
        let b = |s: &Sim, name: &str| s.id(mjtObj::mjOBJ_BODY, name).unwrap();
        let i = b(&actual, &format!("стенд/{name}"));
        let j = b(&reference, name);
        near(actual.body_mass(i), reference.body_mass(j), 1e-12);
        let mut a = actual.body_inertia(i);
        let mut r = reference.body_inertia(j);
        a.sort_by(f64::total_cmp);
        r.sort_by(f64::total_cmp);
        for (a, b) in a.into_iter().zip(r) {
            near(a, b, 1e-12);
        }
        let (p, q) = actual.body_pose(i);
        let (rp, rq) = reference.body_pose(j);
        for (a, b) in p.into_iter().zip(rp) {
            near(a, b, 1e-12);
        }
        near(
            q.into_iter().zip(rq).map(|(a, b)| a * b).sum::<f64>().abs(),
            1.0,
            1e-12,
        );
    }
    near(actual.qpos()[0], std::f64::consts::PI / 6.0, 1e-14);
    for (a, b) in actual
        .full_mass_matrix()
        .into_iter()
        .zip(reference.full_mass_matrix())
    {
        near(a, b, 1e-12);
    }
    let invalid = MJCF.replace(
        "</mujoco>",
        "<actuator><motor joint=\"угол\"/></actuator></mujoco>",
    );
    let e = import_mjcf(path, &invalid).unwrap_err();
    assert!(e.contains("маятник.xml") && e.contains("<actuator>"), "{e}");
    let invalid = MJCF.replace("damping=\"0.2\"", "damping=\"inf\"");
    assert!(import_mjcf(path, &invalid).unwrap_err().contains("damping"));
}

#[test]
fn migration_preserves_mechanics_and_original_text_through_format_three() {
    use sim_device::{
        Axis, BodyRegion, Direction, Electronics, JointKind, JointSpec, LinkSpec, Section,
        UseProfile,
    };
    let legacy = sim_device::Device {
        name: "Каретка со стержнем".into(),
        template: "Стенд".into(),
        mount: BodyRegion::None,
        links: vec![LinkSpec {
            name: "стержень".into(),
            parent: None,
            offset_mm: [100.0, 200.0, 300.0],
            direction: Direction::Left,
            length_mm: 300.0,
            section: Section::Round { d_mm: 20.0 },
            material: "al6061-t6".into(),
            print: None,
            hole_mm: 0.0,
            end_mass_kg: 0.3,
            near_body: BodyRegion::None,
            contact_lever_mm: 0.0,
            joint: JointSpec {
                kind: JointKind::Hinge,
                axis: Axis::X,
                min_deg: -90.0,
                max_deg: 45.0,
                ..JointSpec::fixed()
            },
            settings: Default::default(),
        }],
        electronics: Electronics {
            board: "контроллер-1".into(),
            firmware: "сборка.elf".into(),
            ..Electronics::default()
        },
        usage: UseProfile::default(),
        settings: Default::default(),
        measurements: vec![sim_device::measure::Measurement {
            quantity: sim_device::measure::Quantity::TotalMass,
            link: None,
            value: 0.55,
            note: "первая строка\nвторая строка".into(),
            design: Some(42),
        }],
    };
    let tmp = Fixture::new("legacy");
    let text = sim_device::file::to_text(&legacy) + "\n# Комментарий владельца\n";
    let path = tmp.write("стержень.device", &text);
    let actual = migrate_v2(&path, &text, &LegacyCatalog).unwrap();
    let v3 = toml::to_string(&actual).unwrap();
    let read_back = parse_device(&path, &v3).unwrap();
    assert_eq!(extract_v2(&read_back), Some(text.as_str()));
    assert_eq!(
        sim_device::file::from_text(extract_v2(&read_back).unwrap()).unwrap(),
        legacy
    );
    assert_eq!(actual.links[0].joint.axis, Some([1.0, 0.0, 0.0]));
    let c = compile(actual.clone(), "стержень.device");
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    let id = sim.id(mjtObj::mjOBJ_BODY, "стенд/стержень").unwrap();
    for (a, b) in sim.body_pose(id).0.into_iter().zip([0.1, 0.2, 0.3]) {
        near(a, b, 1e-14);
    }
    let rho = sim_materials::MaterialId::Al6061T6.material().density;
    let beam = rho * std::f64::consts::PI * 0.01 * 0.01 * 0.3;
    // Стержень вдоль Y; его центр в 0,15 м, груз в 0,30 м от шарнира.
    near(c.bodies[0].mass_kg, beam + 0.3, 1e-14);
    near(
        c.bodies[0].com_m[1],
        (beam * 0.15 + 0.3 * 0.3) / (beam + 0.3),
        1e-14,
    );
    assert!(
        c.simplifications
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("legacy_v2"))
    );
    let world = tmp.write(
        "мир.toml",
        toml::to_string(&placement("стержень.device")).unwrap(),
    );
    assert_eq!(load_and_compile(&world).unwrap(), c);
    tmp.write("стержень.device", format!("# Файл после миграции\n{v3}"));
    assert_eq!(load_device(&path, &LegacyCatalog).unwrap(), actual);
}

#[test]
fn import_mjcf_world_keeps_global_settings_and_fixed_world_sites() {
    let path = Path::new("маятник.mjcf");
    let text = r#"<mujoco model="Маятник">
      <option timestep="0.004" gravity="0 0 -2" density="1.2" wind="1 0 0"/>
      <worldbody>
        <site name="опора" pos="0 1 0"/>
        <body name="блок" pos="0 1 0">
          <freejoint/>
          <site name="стык"/>
          <inertial mass="2" pos="0 0 0" diaginertia="0.2 0.3 0.4"/>
        </body>
      </worldbody>
      <equality><weld site1="опора" site2="стык"/></equality>
    </mujoco>"#;
    let (mut world, input) = import_mjcf_world(path, text, 42).unwrap();
    assert_eq!(world.seed, 42);
    assert_eq!(world.physics_hz, 250);
    assert_eq!(world.gravity_m_s2, [0.0, 0.0, -2.0]);
    // Перенос должен переместить и неподвижную опору, и свободное тело.
    world.devices[0].pose.position_m = [2.0, 0.0, 0.0];
    let c = compile_world(
        Path::new("мир.toml"),
        &world,
        &BTreeMap::from([("mjcf".into(), input)]),
    )
    .unwrap();
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    assert_eq!(
        sim.body_pose(sim.id(mjtObj::mjOBJ_BODY, "mjcf/блок").unwrap())
            .0,
        [2.0, 1.0, 0.0]
    );
    near(sim.timestep(), 0.004, 0.0);
    near(sim.model().opt.density, 1.2, 0.0);
    assert_eq!(sim.model().neq, 1);
    assert!(import_mjcf(path, text).unwrap_err().contains("<option>"));
    assert!(
        import_mjcf_world(path, &text.replace("0.004", "0.0041"), 42)
            .unwrap_err()
            .contains("option.timestep")
    );
    assert!(
        import_mjcf_world(path, &text.replace("wind=", "integrator=\"RK4\" wind="), 42)
            .unwrap_err()
            .contains("integrator")
    );
}
