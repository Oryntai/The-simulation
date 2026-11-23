use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use sim_mujoco::{Sim, ffi::mjtObj};
use sim_world::*;

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "получено {actual}, ожидалось {expected}, допуск {tolerance}"
    );
}

fn explicit() -> Mass {
    // Один килограмм, положительно определённая инерция с внедиагональным элементом.
    Mass::Explicit {
        mass_kg: 1.0,
        com_m: [0.2, 0.0, 0.0],
        inertia_kg_m2: [0.1, 0.15, 0.2, 0.025, 0.0, 0.0],
        provenance: Provenance::Measured("исходные числа синтетической сцены".into()),
    }
}

fn position(position_m: [f64; 3]) -> Pose {
    Pose {
        position_m,
        ..Pose::default()
    }
}

fn link(name: &str, kind: JointType, axis: Option<[f64; 3]>) -> Link {
    let mut joint = Joint::new(kind);
    joint.axis = axis;
    if kind != JointType::Fixed {
        joint.damping = Some(0.0);
        joint.friction_loss = Some(0.0);
        joint.armature = Some(0.0);
    }
    Link {
        name: name.into(),
        parent: None,
        pose: Pose::default(),
        joint,
        mass: explicit(),
        collision: Vec::new(),
        visual: Vec::new(),
        mounts: Vec::new(),
    }
}

fn device(links: Vec<Link>) -> Device {
    Device {
        format: FORMAT,
        name: "Стержень".into(),
        links,
        constraints: Vec::new(),
        transmissions: Vec::new(),
        drives: Vec::new(),
        sensors: Vec::new(),
        legacy_v2: None,
    }
}

fn world() -> World {
    // 1 мс на шаг и нулевая гравитация изолируют кинематические тесты.
    World {
        format: FORMAT,
        name: "Лаборатория".into(),
        seed: 29,
        physics_hz: 1000,
        gravity_m_s2: [0.0; 3],
        environment: Environment::default(),
        devices: vec![Placement {
            name: "стенд".into(),
            file: "стержень.toml".into(),
            pose: Pose::default(),
        }],
        objects: Vec::new(),
        attachments: Vec::new(),
        sensors: Vec::new(),
    }
}

fn compile(w: &World, d: Device) -> Result<CompiledWorld, String> {
    let input = w
        .devices
        .iter()
        .map(|p| {
            (
                p.name.clone(),
                DeviceInput {
                    path: p.file.clone(),
                    device: d.clone(),
                },
            )
        })
        .collect();
    compile_world(Path::new("мир.toml"), w, &input)
}

fn body(sim: &Sim, name: &str) -> usize {
    sim.id(mjtObj::mjOBJ_BODY, name).unwrap()
}
fn joint(sim: &Sim, name: &str) -> sim_mujoco::JointInfo {
    sim.joints().into_iter().find(|j| j.name == name).unwrap()
}

#[test]
fn chain_slide_hinge_ball_uses_si_and_agrees_with_second_engine() {
    let mut w = world();
    w.gravity_m_s2 = [0.0, 0.0, -9.81];
    w.devices[0].pose = Pose {
        position_m: [1.0, 2.0, 3.0],
        rpy_deg: Some([0.0, 0.0, 90.0]),
        quat_wxyz: None,
    };
    let mut slide = link("каретка", JointType::Slide, Some([2.0, 0.0, 0.0]));
    slide.joint.initial_m = Some(0.25);
    slide.joint.limits_m = Some([0.0, 2.0]);
    let mut hinge = link("маятник", JointType::Hinge, Some([0.0, 0.0, 4.0]));
    hinge.parent = Some(slide.name.clone());
    hinge.pose = position([1.0, 0.0, 0.0]);
    hinge.joint.initial_deg = Some(90.0);
    hinge.joint.limits_deg = Some([-90.0, 120.0]);
    hinge.joint.armature = Some(0.01);
    let mut ball = link("блок", JointType::Ball, None);
    ball.parent = Some(hinge.name.clone());
    ball.pose = position([1.0, 0.0, 0.0]);
    ball.joint.ball_limit_deg = Some(60.0);
    ball.joint.initial_rpy_deg = Some([90.0, 0.0, 0.0]);
    ball.joint.efforts.push(Effort {
        name: "момент".into(),
        dof: 2,
    });
    // Проверяем ссылки вперёд в том же документе.
    let d = device(vec![ball, hinge, slide]);
    let c = compile(&w, d.clone()).unwrap();
    assert_eq!(c, compile(&w, d).unwrap());
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    let (p, q) = sim.body_pose(body(&sim, "стенд/блок"));
    // После поворота мира на четверть оборота каретка идёт по +Y. Второй
    // поворот на четверть оборота направляет вынос блока по −X.
    for (actual, expected) in p.into_iter().zip([0.0, 3.25, 3.0]) {
        near(actual, expected, 1e-12);
    }
    let expected = [0.0, 0.0, 2.0_f64.sqrt() / 2.0, 2.0_f64.sqrt() / 2.0];
    near(
        q.iter()
            .zip(expected)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .abs(),
        1.0,
        1e-12,
    );
    let hinge = joint(&sim, "стенд/маятник");
    near(
        sim.qpos()[hinge.qpos_adr],
        std::f64::consts::FRAC_PI_2,
        1e-15,
    );
    near(hinge.range[0], -std::f64::consts::FRAC_PI_2, 1e-15);
    near(hinge.range[1], 2.0 * std::f64::consts::PI / 3.0, 1e-15);
    let ball = joint(&sim, "стенд/блок");
    near(ball.range[1], std::f64::consts::PI / 3.0, 1e-15);
    assert_eq!(
        c.efforts,
        vec![EffortChannel {
            name: "стенд/момент".into(),
            joint: "стенд/блок".into(),
            dof: 2
        }]
    );
    let report = verify_compiled(&c);
    assert_eq!(report.inertial, CheckStatus::Verified);
    assert_eq!(report.dynamics, CheckStatus::Verified, "{report:?}");
    let engine = report.engine.unwrap();
    assert!(engine.samples > 0 && engine.bias_scale > 0.0);
    assert!(
        engine.max_mass_rel < 1e-10 && engine.max_bias_abs < 1e-10,
        "{engine:?}"
    );
}

fn crate_material(density: f64) -> Material {
    Material {
        name: "однородный образец".into(),
        density_kg_m3: density,
        provenance: Provenance::Measured("условная плотность сцены с известной массой".into()),
    }
}

#[test]
fn steiner_matches_hand_calculation_and_mujoco() {
    let mut part = link("блок", JointType::Free, None);
    part.mass = Mass::Geometry {
        material: crate_material(1.0),
    };
    let mut a = Geometry::new(Shape::Box { size_m: [1.0; 3] });
    a.pose = position([-1.0, 0.0, 0.0]);
    a.material = Some(crate_material(2.0));
    let mut b = Geometry::new(Shape::Box { size_m: [1.0; 3] });
    b.pose = position([2.0, 0.0, 0.0]);
    part.collision = vec![a, b];
    part.visual = vec![Geometry::new(Shape::Sphere { radius_m: 5.0 })];
    let c = compile(&world(), device(vec![part])).unwrap();
    let value = &c.bodies[0];
    // Два единичных куба массами 2 и 1 кг. Центр (−2+2)/3=0.
    // Вдоль X: 3/6. Поперёк: 3/6 + 2·1² + 1·2².
    near(value.mass_kg, 3.0, 1e-14);
    assert!(value.com_m.iter().all(|x| x.abs() < 1e-14));
    for (a, b) in value.inertia_kg_m2.into_iter().zip([0.5, 6.5, 6.5]) {
        near(a, b, 1e-14);
    }
    assert_eq!(verify_compiled(&c).inertial, CheckStatus::Verified);
    let sim = Sim::from_xml(&c.mjcf).unwrap();
    let mut actual = sim.body_inertia(body(&sim, "стенд/блок"));
    actual.sort_by(f64::total_cmp);
    for (a, b) in actual.into_iter().zip([0.5, 6.5, 6.5]) {
        near(a, b, 1e-13);
    }
    // Искажение метаданных должно ломать проверку, даже если MJCF рабочий.
    for field in ["mass_kg", "com_m", "inertia_kg_m2"] {
        let mut invalid = c.clone();
        let b = &mut invalid.bodies[0];
        match field {
            "mass_kg" => b.mass_kg *= 2.0,
            "com_m" => b.com_m[1] = 1.0,
            _ => b.inertia_kg_m2[0] = 0.25,
        }
        assert!(
            matches!(verify_compiled(&invalid).inertial, CheckStatus::Mismatch(e) if e.contains(field))
        );
    }
    assert!(matches!(
        verify_compiled(&c).dynamics,
        CheckStatus::NotAnalyzed(_)
    ));
}

#[test]
fn all_primitives_have_independently_checked_inertia() {
    let shapes = [
        Shape::Sphere { radius_m: 0.2 },
        Shape::Capsule {
            radius_m: 0.2,
            length_m: 0.6,
        },
        Shape::Cylinder {
            radius_m: 0.1,
            length_m: 0.3,
        },
        Shape::Ellipsoid {
            radii_m: [0.1, 0.2, 0.3],
        },
    ];
    let mut links = Vec::new();
    for (i, shape) in shapes.into_iter().enumerate() {
        let mut part = link(&format!("блок-{i}"), JointType::Fixed, None);
        part.mass = Mass::Geometry {
            material: crate_material(1.0),
        };
        let mut g = Geometry::new(shape);
        g.pose.rpy_deg = Some([23.0, 13.0, 38.0]);
        part.collision.push(g);
        links.push(part);
    }
    let c = compile(&world(), device(links)).unwrap();
    assert_eq!(verify_compiled(&c).inertial, CheckStatus::Verified);
    // Объём эллипсоида: якобиан отображения единичного шара равен abc.
    near(
        c.bodies[3].mass_kg,
        4.0 * std::f64::consts::PI * 0.1 * 0.2 * 0.3 / 3.0,
        1e-15,
    );
}

fn anchor(link: Option<&str>, xyz: [f64; 3]) -> Anchor {
    Anchor {
        link: link.map(str::to_owned),
        pose: position(xyz),
    }
}

fn driven(c: &CompiledWorld, input: &str, output: &str) -> (Sim, f64, f64) {
    let mut sim = Sim::from_xml(&c.mjcf).unwrap();
    let i = joint(&sim, input);
    let o = joint(&sim, output);
    let start = [sim.qpos()[i.qpos_adr], sim.qpos()[o.qpos_adr]];
    // Постоянное усилие за 0,4 с, близкие инерции и мягкие связи MuJoCo.
    sim.qfrc_applied_mut()[i.dof_adr] = 1.0;
    for _ in 0..400 {
        sim.step();
    }
    let di = sim.qpos()[i.qpos_adr] - start[0];
    let dout = sim.qpos()[o.qpos_adr] - start[1];
    assert!(di > 1e-3, "сцена должна сдвинуться под усилием, Δ={di}");
    (sim, di, dout)
}

#[test]
fn gear_belt_screw_run_with_signed_ratios() {
    for (tx, output_kind, expected) in [
        (
            Transmission::Gear {
                name: "связь".into(),
                input: "вход".into(),
                output: "выход".into(),
                ratio: -2.0,
            },
            JointType::Hinge,
            -2.0,
        ),
        (
            Transmission::Belt {
                name: "связь".into(),
                input: "вход".into(),
                output: "выход".into(),
                ratio: 0.5,
            },
            JointType::Hinge,
            0.5,
        ),
        (
            Transmission::Screw {
                name: "связь".into(),
                input: "вход".into(),
                output: "выход".into(),
                lead_m_per_turn: 0.5,
            },
            JointType::Slide,
            0.5 / std::f64::consts::TAU,
        ),
    ] {
        let mut d = device(vec![
            link("вход", JointType::Hinge, Some([0.0, 0.0, 1.0])),
            link("выход", output_kind, Some([0.0, 0.0, 1.0])),
        ]);
        d.transmissions.push(tx);
        let c = compile(&world(), d).unwrap();
        let (_, di, dout) = driven(&c, "стенд/вход", "стенд/выход");
        // Допуск 1% на мягкую связь, ошибка масштаба и знак требуют настоящего движения.
        near(dout / di, expected, expected.abs() * 0.01);
        let report = verify_compiled(&c);
        assert!(matches!(report.dynamics, CheckStatus::NotAnalyzed(_)));
        assert!(report.engine.is_some());
    }
}

#[test]
fn cable_transfers_motion_through_branches_and_pulley() {
    let mut a = link("вход", JointType::Slide, Some([1.0, 0.0, 0.0]));
    a.pose = position([1.0, 0.0, 0.0]);
    let mut b = link("выход", JointType::Slide, Some([1.0, 0.0, 0.0]));
    b.pose = position([1.0, 1.0, 0.0]);
    let mut d = device(vec![a, b]);
    d.transmissions.push(Transmission::Cable {
        name: "трос".into(),
        max_length_m: 1.5,
        branches: vec![
            CableBranch {
                divisor: 1.0,
                points: vec![
                    anchor(None, [0.0, 0.0, 0.0]),
                    anchor(Some("вход"), [0.0; 3]),
                ],
            },
            CableBranch {
                divisor: 2.0,
                points: vec![
                    anchor(None, [0.0, 1.0, 0.0]),
                    anchor(Some("выход"), [0.0; 3]),
                ],
            },
        ],
    });
    let c = compile(&world(), d).unwrap();
    let (sim, di, dout) = driven(&c, "стенд/вход", "стенд/выход");
    // L=(1+Δx)+(1+Δy)/2, откуда Δy/Δx=−2 после натяжения.
    near(dout / di, -2.0, 0.02);
    near(sim.tendon_length(0), 1.5, 1e-4);
    assert_eq!(sim.tendon_path_objects(0), 5);
}

#[test]
fn connect_and_weld_close_a_chain() {
    for weld in [false, true] {
        let a = link("вход", JointType::Slide, Some([1.0, 0.0, 0.0]));
        let mut b = link("выход", JointType::Slide, Some([1.0, 0.0, 0.0]));
        b.pose = position([0.0, 1.0, 0.0]);
        let mut d = device(vec![a, b]);
        let a = anchor(Some("вход"), [0.0, 0.5, 0.0]);
        let b = anchor(Some("выход"), [0.0, -0.5, 0.0]);
        d.constraints.push(if weld {
            Constraint::Weld {
                name: "замыкание".into(),
                first: a,
                second: b,
                active: true,
            }
        } else {
            Constraint::Connect {
                name: "замыкание".into(),
                first: a,
                second: b,
                active: true,
            }
        });
        let c = compile(&world(), d).unwrap();
        let (sim, di, dout) = driven(&c, "стенд/вход", "стенд/выход");
        assert_eq!(sim.model().neq, 1);
        near(dout / di, 1.0, 0.01);
    }
}

fn mount(kind: MountKind) -> Mount {
    Mount {
        name: "стык".into(),
        kind,
        interface: "посадка".into(),
        pose: Pose::default(),
    }
}

#[test]
fn multiple_devices_free_objects_and_named_attachment() {
    let mut w = world();
    w.devices.push(Placement {
        name: "опора".into(),
        file: "стержень.toml".into(),
        pose: position([0.0, 2.0, 0.0]),
    });
    for (name, y) in [("предмет", 0.0), ("груз", 3.0)] {
        w.objects.push(Object {
            name: name.into(),
            pose: position([0.0, y, 0.0]),
            mass: explicit(),
            visual: Vec::new(),
            collision: Vec::new(),
            mounts: vec![mount(MountKind::Mechanical)],
        });
    }
    w.attachments.push(Attachment {
        name: "зацеп".into(),
        first: "стенд/блок/стык".into(),
        second: "предмет/предмет/стык".into(),
        mode: AttachmentMode::Runtime,
    });
    let mut part = link("блок", JointType::Fixed, None);
    part.mounts.push(mount(MountKind::Mechanical));
    let d = device(vec![part]);
    let c = compile(&w, d.clone()).unwrap();
    assert_eq!(c.bodies.len(), 4);
    let mut sim = Sim::from_xml(&c.mjcf).unwrap();
    assert_eq!(sim.nv(), 12);
    let eq = sim.id(mjtObj::mjOBJ_EQUALITY, "attachment/зацеп").unwrap();
    // Читаем флаг непосредственно из независимой структуры движка.
    let active = |s: &Sim| unsafe { *s.data().eq_active.add(eq) };
    assert!(!active(&sim));
    set_attachment_active(&mut sim, "зацеп", true).unwrap();
    assert!(active(&sim));
    set_attachment_active(&mut sim, "зацеп", false).unwrap();
    assert!(!active(&sim));
    assert!(set_attachment_active(&mut sim, "нет", true).is_err());
    w.attachments[0].mode = AttachmentMode::Design;
    let c = compile(&w, d.clone()).unwrap();
    assert!(active(&Sim::from_xml(&c.mjcf).unwrap()));
    w.objects[0].pose.position_m[0] = 0.1;
    assert!(
        compile(&w, d.clone())
            .unwrap_err()
            .contains("совместите позы")
    );
    w.objects[0].mounts[0].kind = MountKind::Signal;
    assert!(compile(&w, d).unwrap_err().contains("несовместимые типы"));
}

#[test]
fn parser_and_compiler_refuse_corrupt_data_with_paths() {
    let w = world();
    let mut d = device(vec![link("блок", JointType::Fixed, None)]);
    let text = toml::to_string(&d).unwrap();
    let path = Path::new("проверка.toml");
    assert_eq!(parse_device(path, &text).unwrap(), d);
    let typo = text.replace("mass_kg", "msas_kg");
    let e = parse_device(path, &typo).unwrap_err();
    assert!(
        e.contains("проверка.toml") && e.contains("links[0].mass") && e.contains("msas_kg"),
        "{e}"
    );
    let nan = text.replace("mass_kg = 1.0", "mass_kg = nan");
    let e = parse_device(path, &nan).unwrap_err();
    assert!(
        e.contains("проверка.toml") && e.contains("links[0].mass.mass_kg"),
        "{e}"
    );
    let mut wt = toml::to_string(&w).unwrap();
    wt = wt.replace("viscosity_pa_s = 0.0", "viscosity_pa_s = +inf");
    assert!(
        parse_world(path, &wt)
            .unwrap_err()
            .contains("environment.viscosity_pa_s")
    );
    let target = compile(&w, d.clone()).unwrap();
    assert_eq!(target.bodies.len(), 1);
    d.links.push(d.links[0].clone());
    assert!(
        compile(&w, d.clone())
            .unwrap_err()
            .contains("links[1].name")
    );
    d.links[1].name = "опора".into();
    d.links[0].parent = Some("отсутствует".into());
    assert!(
        compile(&w, d.clone())
            .unwrap_err()
            .contains("links[0].parent")
    );
    d.links[0].parent = Some("опора".into());
    d.links[1].parent = Some("блок".into());
    assert!(compile(&w, d.clone()).unwrap_err().contains("цикл"));
    d.links[0].parent = None;
    let Mass::Explicit { mass_kg, .. } = &mut d.links[0].mass else {
        unreachable!()
    };
    *mass_kg = -1.0;
    assert!(
        compile(&w, d)
            .unwrap_err()
            .contains("links[0].mass.mass_kg")
    );
    let mut l = link("стержень", JointType::Slide, Some([0.0; 3]));
    assert!(
        compile(&w, device(vec![l.clone()]))
            .unwrap_err()
            .contains("joint.axis")
    );
    l.joint.axis = Some([1.0, 0.0, 0.0]);
    l.joint.efforts.push(Effort {
        name: "сила".into(),
        dof: 1,
    });
    assert!(
        compile(&w, device(vec![l]))
            .unwrap_err()
            .contains("efforts[0].dof")
    );
}

#[test]
fn sectional_model_is_only_selected_for_constant_supported_sections() {
    let s = Shape::Box {
        size_m: [0.02, 0.03, 0.4],
    };
    let StrengthSection::Applicable { section, length_m } =
        strength_section(&s, SectionAxis::Z).unwrap()
    else {
        panic!("нет прямоугольного сечения")
    };
    near(length_m, 0.4, 0.0);
    near(section.area(), 0.02 * 0.03, 1e-16);
    let s = Shape::Mesh {
        file: PathBuf::from("блок.obj"),
        scale: [1.0; 3],
    };
    assert!(matches!(
        strength_section(&s, SectionAxis::Z).unwrap(),
        StrengthSection::NotAnalyzed { .. }
    ));
    assert!(
        strength_section(
            &Shape::Cylinder {
                radius_m: f64::NAN,
                length_m: 1.0
            },
            SectionAxis::Z
        )
        .is_err()
    );
}

#[test]
fn map_must_match_world_instances() {
    assert!(
        compile_world(Path::new("мир.toml"), &world(), &BTreeMap::new())
            .unwrap_err()
            .contains("devices[0].file")
    );
}

#[test]
fn full_tensor_check_detects_rotated_principal_axes() {
    let l = link("блок", JointType::Free, None);
    let mut c = compile(&world(), device(vec![l])).unwrap();
    let good = verify_compiled(&c);
    assert_eq!(good.inertial, CheckStatus::Verified);
    assert_eq!(good.dynamics, CheckStatus::Verified, "{good:?}");
    let original = c
        .mjcf
        .lines()
        .find(|line| line.starts_with("<inertial "))
        .unwrap();
    let changed = original.replace("0.025", "-0.025");
    assert_ne!(original, changed);
    c.mjcf = c.mjcf.replace(original, &changed);
    assert!(
        matches!(verify_compiled(&c).inertial,CheckStatus::Mismatch(e) if e.contains("inertia_tensor"))
    );
}

mod parts {
    use super::*;

    fn with_channel() -> Device {
        let mut l = link("блок", JointType::Hinge, Some([0.0, 1.0, 0.0]));
        l.joint.efforts.push(Effort {
            name: "момент".into(),
            dof: 0,
        });
        device(vec![l])
    }

    fn drive(effort: &str) -> DriveUse {
        DriveUse {
            name: "привод".into(),
            part: "деталь".into(),
            effort: effort.into(),
            params: BTreeMap::new(),
        }
    }

    fn sensor() -> SensorUse {
        SensorUse {
            name: "датчик".into(),
            part: "деталь".into(),
            link: Some("блок".into()),
            pose: position([0.0, 0.0, 0.1]),
            joint: Some("блок".into()),
            drive: Some("привод".into()),
            params: BTreeMap::new(),
        }
    }

    #[test]
    fn drive_and_sensor_resolve_to_full_names() {
        let mut d = with_channel();
        d.drives.push(drive("момент"));
        d.sensors.push(sensor());
        let c = compile(&world(), d).unwrap();
        assert_eq!(c.drives[0].effort, "стенд/момент");
        assert_eq!(c.drives[0].joint, "стенд/блок");
        assert_eq!(c.sensors[0].body.as_deref(), Some("стенд/блок"));
        assert_eq!(c.sensors[0].drive.as_deref(), Some("стенд/привод"));
        assert_eq!(c.sensors[0].position_m, [0.0, 0.0, 0.1]);
    }

    #[test]
    fn broken_references_are_refused() {
        let cases: Vec<(&str, fn(&mut Device))> = vec![
            ("effort", |d| d.drives[0].effort = "нет".into()),
            ("part", |d| d.drives[0].part = " ".into()),
            ("params", |d| {
                d.drives[0].params.insert("люфт".into(), f64::NAN);
            }),
            ("link", |d| d.sensors[0].link = Some("нет".into())),
            ("joint", |d| d.sensors[0].joint = Some("нет".into())),
            ("drive", |d| d.sensors[0].drive = Some("нет".into())),
            ("name", |d| {
                let copy = d.sensors[0].clone();
                d.sensors.push(copy);
            }),
            ("занят", |d| {
                let mut copy = d.drives[0].clone();
                copy.name = "второй".into();
                d.drives.push(copy);
            }),
        ];
        for (what, edit) in cases {
            let mut d = with_channel();
            d.drives.push(drive("момент"));
            d.sensors.push(sensor());
            edit(&mut d);
            let e = compile(&world(), d).expect_err(what);
            assert!(e.contains(what) || what == "params" && e.contains("люфт"), "{what}: {e}");
        }
    }

    #[test]
    fn world_sensor_refers_to_compiled_body() {
        let mut w = world();
        let mut s = sensor();
        s.name = "камера".into();
        s.link = Some("стенд/блок".into());
        s.joint = None;
        s.drive = None;
        w.sensors.push(s.clone());
        let c = compile(&w, with_channel()).unwrap();
        assert_eq!(c.sensors[0].body.as_deref(), Some("стенд/блок"));
        w.sensors[0].link = Some("стенд/нет".into());
        assert!(compile(&w, with_channel()).is_err());
    }
}
