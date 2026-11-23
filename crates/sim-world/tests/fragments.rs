use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use sim_mujoco::ffi::mjtObj as Obj;
use sim_world::*;

// Синтетическая каретка: масса 2 кг, ЦМ на 0,25 м от оси.
// Тензор получен поворотом главных осей с моментами [0,12; 0,08; 0,15]
// на 45° вокруг Z. Две оси находятся на одном теле.
const MODEL: &str = r#"<mujoco model="каретка">
    <compiler angle="degree" fusestatic="false" alignfree="false"/>
    <worldbody>
        <body name="блок">
            <inertial pos="0.25 0 0" mass="2" fullinertia="0.1 0.1 0.15 0.02 0 0"/>
            <joint name="ход" type="slide" axis="0 0 1"/>
            <joint name="угол" axis="0 0 1" range="-90 90"/>
            <site name="центр" pos="0 0 0"/>
            <site name="край" pos="0.25 0 0"/>
        </body>
    </worldbody>
    <sensor><jointpos name="отсчёт" joint="ход"/></sensor>
</mujoco>"#;
const WORLD: &str = "стенд.toml";

fn scene() -> World {
    World {
        format: FORMAT,
        name: "Каретка".into(),
        seed: 6,
        physics_hz: 1000,
        gravity_m_s2: [0.0, 0.0, -10.0],
        environment: Environment::default(),
        devices: vec![],
        objects: vec![],
        attachments: vec![],
        sensors: vec![],
    }
}

fn fragment(name: &str) -> ModelFragment {
    ModelFragment {
        name: name.into(),
        source: PathBuf::from("каретка.xml"),
        mjcf: Some(MODEL.into()),
        pose: Pose::default(),
        inertia: BTreeMap::from([(
            "блок".into(),
            FragmentInertia {
                mass_kg: 2.0,
                com_m: [0.25, 0.0, 0.0],
                tensor_kg_m2: [0.1, 0.1, 0.15, 0.02, 0.0, 0.0],
                provenance: Provenance::Derived(
                    "Синтетический тензор с поворотом главных осей".into(),
                ),
            },
        )]),
        ports: ["центр", "край"]
            .into_iter()
            .map(|site| {
                (
                    format!("порт_{site}"),
                    FragmentPort {
                        site: site.into(),
                        kind: MountKind::Mechanical,
                        interface: "стык".into(),
                    },
                )
            })
            .collect(),
        surfaces: BTreeMap::new(),
        efforts: ["ход", "угол"]
            .into_iter()
            .map(|joint| FragmentEffort {
                name: format!("усилие_{joint}"),
                joint: joint.into(),
                dof: 0,
            })
            .collect(),
        drives: vec![],
    }
}

fn compile(
    world: &World,
    fragments: &[ModelFragment],
    contacts: &ContactBuildOptions,
) -> Result<(CompiledWorld, FragmentBindings), String> {
    compile_with_fragment_bindings(Path::new(WORLD), world, &[], fragments, contacts)
}

fn close(a: f64, b: f64) {
    // Запас округления для тензора, смены базиса и одного шага.
    assert!((a - b).abs() < 1e-11, "{a:.17} != {b:.17}");
}

#[test]
fn verifies_tensor_and_unconstrained_acceleration() {
    let world = scene();
    let input = fragment("первая");
    let (compiled, bindings) = compile(&world, &[input], &ContactBuildOptions::default()).unwrap();
    let report = verify_compiled(&compiled);
    assert_eq!(report.inertial, CheckStatus::Verified);
    assert_eq!(report.dynamics, CheckStatus::Verified);
    let check = report.engine.unwrap();
    assert_eq!(check.samples, 32);
    assert!(check.max_mass_rel < 1e-11, "{check:?}");
    assert!(check.max_bias_abs < 1e-11, "{check:?}");

    let mut sim = compiled.open_contact_sim().unwrap();
    let matrix = sim.full_mass_matrix();
    // Теорема Штейнера и ортогональные направления движения ЦМ в нейтральной позе.
    let i = 0.15 + 2.0 * 0.25_f64.powi(2);
    for (actual, expected) in matrix.iter().copied().zip([2.0, 0.0, 0.0, i]) {
        close(actual, expected);
    }
    assert_eq!(sim.bias_forces(), [20.0, 0.0]);
    assert_eq!(compiled.efforts[0].name, "первая/усилие_ход");
    assert_eq!(compiled.efforts[1].joint, "первая/угол");
    assert_eq!(
        bindings
            .mechanical_port("первая/порт_край")
            .unwrap()
            .local_pose
            .position_m,
        [0.25, 0.0, 0.0]
    );
    sim.qfrc_applied_mut().copy_from_slice(&[26.0, 0.55]);
    sim.step();
    // Неразрывная динамика при q̇=0: (26−20)/2 = 3 м/с², 0,55/I = 2 рад/с².
    let observed = [
        sim.qvel()[0] / sim.timestep(),
        sim.qvel()[1] / sim.timestep(),
    ];
    close(observed[0], 3.0);
    close(observed[1], 2.0);
    println!(
        "Mx={:.15}, I={:.15}, a={observed:?}, сверка={check:?}",
        matrix[0], matrix[3]
    );
}

fn contacts() -> ContactBuildOptions {
    // Синтетические параметры мягкой нормали по модели MuJoCo 4.0.1.
    ContactBuildOptions {
        solver: Some(ContactSolver {
            cone: FrictionCone::Elliptic,
            solver: ConstraintSolver::Newton,
            impratio: 1.0,
            tolerance: 1e-10,
            iterations: 40,
            line_search_tolerance: 1e-10,
            line_search_iterations: 30,
            noslip_tolerance: 1e-9,
            noslip_iterations: 0,
            ccd_tolerance: 1e-8,
            ccd_iterations: 60,
        }),
        pairs: vec![ContactPair {
            geometries: [
                "первая/поверхность".into(),
                "environment/collision/0".into(),
            ],
            settings: ContactSettings {
                dim: 1,
                friction: [0.2; 5],
                solref: [-4000.0, -80.0],
                solreffriction: [0.0; 2],
                solimp: [0.9, 0.9, 0.001, 0.5, 2.0],
                margin_m: 0.0,
                gap_m: 0.0,
                adhesion_n: 0.0,
            },
        }],
        ..ContactBuildOptions::default()
    }
}

fn experiment() -> (World, [ModelFragment; 2], ContactBuildOptions) {
    let mut world = scene();
    let mut floor = Geometry::new(Shape::Box {
        size_m: [2.0, 2.0, 0.2],
    });
    floor.pose.position_m[2] = -0.1;
    world.environment.collision.push(floor);
    let mut first = fragment("первая");
    first.pose.position_m[2] = 0.099;
    let xml = MODEL.replace(
        "<site name=\"центр\"",
        "<geom name=\"нижняя\" type=\"sphere\" size=\"0.1\"/><site name=\"центр\"",
    );
    first.mjcf = Some(xml);
    first.surfaces.insert("поверхность".into(), "нижняя".into());
    let mut second = fragment("вторая");
    second.pose.position_m = [0.25, 0.0, 1.099];
    second.pose.rpy_deg = Some([0.0, 0.0, 90.0]);
    world.attachments.push(Attachment {
        name: "стык".into(),
        first: "первая/порт_край".into(),
        second: "вторая/порт_центр".into(),
        mode: AttachmentMode::Runtime,
    });
    world.sensors.push(SensorUse {
        name: "отсчёт".into(),
        part: "датчик".into(),
        link: Some("вторая/блок".into()),
        joint: Some("вторая/ход".into()),
        drive: None,
        pose: Pose::default(),
        params: BTreeMap::new(),
    });
    (world, [first, second], contacts())
}

#[test]
fn binds_cross_placement_weld_trace_sensor_and_contact_alias() {
    let (world, fragments, options) = experiment();
    let (compiled, bindings) = compile(&world, &fragments, &options).unwrap();
    assert_eq!(
        (
            compiled.bodies.len(),
            compiled.joints.len(),
            compiled.efforts.len()
        ),
        (2, 4, 4)
    );
    assert_eq!(compiled.sensors[0].joint.as_deref(), Some("вторая/ход"));
    assert_eq!(compiled.sensors[0].body.as_deref(), Some("вторая/блок"));
    assert_eq!(bindings.surfaces["первая/поверхность"], "первая/нижняя");
    assert!(
        compiled
            .contact_geometry_names()
            .unwrap()
            .contains("первая/нижняя")
    );
    assert_eq!(compile(&world, &fragments, &options).unwrap().1, bindings);
    let mut sim = compiled.open_contact_sim().unwrap();
    let body_ids =
        ["первая/блок", "вторая/блок"].map(|name| sim.id(Obj::mjOBJ_BODY, name).unwrap());
    assert_eq!(sim.weld_bodies("attachment/стык").unwrap(), body_ids);
    assert!(!sim.equality_active("attachment/стык").unwrap());
    let points = bindings
        .tension_path(&[
            "первая/порт_край".into(),
            "вторая/порт_центр".into(),
            "вторая/порт_край".into(),
        ])
        .unwrap();
    let positions: Vec<_> = points
        .iter()
        .map(|p| {
            let site = sim.id(Obj::mjOBJ_SITE, &p.site).unwrap();
            // SAFETY: ID сайта получен из той же модели, массив имеет 3*nsite элементов.
            unsafe { *sim.data().site_xpos.add(3 * site).cast::<[f64; 3]>() }
        })
        .collect();
    // Точка края первой каретки стоит строго под центром второй. Второй отрезок
    // после четверти оборота идёт вдоль Y. Полная длина 1,25 м.
    let distance = |a: [f64; 3], b: [f64; 3]| {
        a.into_iter()
            .zip(b)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    close(
        distance(positions[0], positions[1]) + distance(positions[1], positions[2]),
        1.25,
    );
    close(positions[2][1], 0.25);
    assert_eq!(
        sim.explicit_contact_settings(["первая/нижняя", "environment/collision/0"])
            .unwrap(),
        Some(options.pairs[0].settings)
    );
    assert!(!sim.step_with_contacts().unwrap().contacts.is_empty());
    // Установившийся баланс каретки в вертикальной направляющей: N=mg=20 Н.
    for _ in 0..2000 {
        sim.step();
    }
    let normal: f64 = sim.contacts().iter().map(|c| c.normal_force).sum();
    // Допуск установления по нормальной силе в этой синтетической постановке.
    assert!((normal - 20.0).abs() < 1e-8, "{normal:.12} Н");
    println!("трасса 1,25 м, реакция {normal:.12} Н");
    let report = verify_compiled(&compiled);
    assert_eq!(report.inertial, CheckStatus::Verified);
    assert!(matches!(report.dynamics, CheckStatus::NotAnalyzed(_)));
    assert!(report.engine.is_some());
    // В этом опыте точки разнесены. Замыкание создало бы работу без модели захвата.
    assert!(set_attachment_active(&mut sim, "стык", true).is_err());
    assert!(!sim.equality_active("attachment/стык").unwrap());
}

#[test]
fn rejects_disabled_pair_unknown_port_and_wrong_inertia() {
    let (mut world, mut fragments, options) = experiment();
    let mut excluded = options.clone();
    excluded
        .excluded_bodies
        .push(["первая/блок".into(), "world".into()]);
    assert!(
        compile(&world, &fragments, &excluded)
            .unwrap_err()
            .contains("запрещают пару")
    );
    let original = fragments.clone();
    fragments[0].mjcf = fragments[0].mjcf.take().map(|xml| {
        xml.replace(
            "name=\"нижняя\"",
            "name=\"нижняя\" contype=\"0\" conaffinity=\"0\"",
        )
    });
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("запрещают пару")
    );
    fragments = original;
    world.attachments[0].second = "вторая/неизвестно".into();
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("неизвестная точка")
    );
    world.attachments.clear();
    fragments[0].inertia.get_mut("блок").unwrap().tensor_kg_m2[3] *= -1.0;
    // Главные моменты совпадают: проверка обязана сравнить тензор в кадре звена.
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("inertia_tensor")
    );
}

#[test]
fn rejects_duplicate_prefix_nonfinite_and_unresolved_export() {
    let (world, mut fragments, options) = experiment();
    fragments[1].name = fragments[0].name.clone();
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("дважды")
    );
    fragments[1].name = "вторая".into();
    fragments[1].efforts[0].dof = 1;
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("степени свободы")
    );
    fragments[1].efforts[0].dof = 0;
    fragments[1].mjcf = Some(MODEL.replace("0.25 0 0", "&#78;aN 0 0"));
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("неконечное")
    );
    fragments[1].mjcf = Some(MODEL.into());
    fragments[1].pose.position_m[2] = f64::INFINITY;
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("конечные")
    );
}

#[test]
fn missing_passport_and_nonmechanical_port_do_not_prove_loads() {
    let mut input = fragment("первая");
    input.inertia.clear();
    input.ports.get_mut("порт_центр").unwrap().kind = MountKind::Signal;
    let (compiled, bindings) =
        compile(&scene(), &[input], &ContactBuildOptions::default()).unwrap();
    assert!(matches!(
        verify_compiled(&compiled).inertial,
        CheckStatus::NotAnalyzed(_)
    ));
    assert!(!compiled.bodies[0].provenance.can_prove());
    assert!(
        bindings
            .tension_path(&["первая/порт_центр".into(), "первая/порт_край".into()])
            .unwrap_err()
            .contains("не механический")
    );
    assert!(
        bindings
            .tension_path(&["первая/порт_край".into(); 1])
            .is_err()
    );
}

#[test]
fn resolves_includes_and_external_mesh_relative_to_source() {
    // Уникальное имя для параллельных процессов тестов. Детерминизм физики от него не зависит.
    let dir = std::env::temp_dir().join(format!("sim-world-fragments-{}", std::process::id()));
    std::fs::create_dir(&dir).unwrap();
    struct Cleanup(PathBuf);
    impl Drop for Cleanup {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    let _cleanup = Cleanup(dir.clone());
    let local = dir.join("модель");
    std::fs::create_dir_all(local.join("сетки")).unwrap();
    std::fs::create_dir_all(local.join("части")).unwrap();
    let asset = local.join("сетки").join("грань.obj");
    // Тетраэдр с единичными рёбрами вдоль трёх координат и объёмом 1/6 м³.
    let mesh = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";
    std::fs::write(&asset, mesh).unwrap();
    let include = local.join("части").join("ресурс.xml");
    std::fs::write(
        &include,
        r#"<mujocoinclude><asset><mesh name="грань" file="грань.obj"/></asset></mujocoinclude>"#,
    )
    .unwrap();
    let model = local.join("каретка.xml");
    let xml = MODEL
        .replace("angle=\"degree\"", "angle=\"degree\" meshdir=\"сетки\"")
        .replace("<worldbody>", "<include file=\"части/ресурс.xml\"/><worldbody>")
        .replace("<site name=\"центр\"", r#"<geom name="сетка" mesh="грань" type="mesh" contype="0" conaffinity="0" group="1"/><site name="центр""#);
    std::fs::write(&model, xml).unwrap();
    let mut a = fragment("первая");
    a.mjcf = None;
    a.source = "модель/каретка.xml".into();
    let b = ModelFragment {
        name: "вторая".into(),
        ..a.clone()
    };
    let assemble = || {
        compile_with_fragment_bindings(
            &dir.join(WORLD),
            &scene(),
            &[],
            &[a.clone(), b.clone()],
            &ContactBuildOptions::default(),
        )
    };
    let (world, bindings) = assemble().unwrap();
    assert_eq!(bindings.resources.len(), 3);
    assert!(
        bindings
            .resources
            .contains_key(&std::path::absolute(&asset).unwrap())
    );
    let sim = world.open_contact_sim().unwrap();
    for name in ["первая", "вторая"] {
        let geom = sim
            .geoms()
            .into_iter()
            .find(|g| g.name == format!("{name}/сетка"))
            .unwrap();
        let mesh = sim.mesh(geom.mesh.unwrap());
        assert_eq!((mesh.positions.len(), mesh.indices.len()), (4, 12));
        let volume: f32 = mesh
            .indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|face| {
                let [a, b, c] = face.map(|i| nalgebra::Vector3::from(mesh.positions[i as usize]));
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum();
        // MuJoCo хранит вершины в float32, запас на поворот базиса и 4 тройных произведения.
        assert!((volume.abs() - 1.0 / 6.0).abs() < 16.0 * f32::EPSILON);
    }
    std::fs::write(&asset, format!("# поправка в источнике\n{mesh}")).unwrap();
    assert!(
        bindings
            .verify_resources()
            .unwrap_err()
            .contains("изменился после сборки")
    );
    assert_ne!(assemble().unwrap().1.fingerprint, bindings.fingerprint);
    std::fs::remove_file(asset).unwrap();
    assert!(assemble().unwrap_err().contains("ресурс"));
    std::fs::write(
        include,
        r#"<mujocoinclude><include file="../каретка.xml"/></mujocoinclude>"#,
    )
    .unwrap();
    assert!(assemble().unwrap_err().contains("цикл включений"));
}

#[test]
fn checks_design_weld_at_the_composed_site_frames() {
    let (mut world, mut fragments, options) = experiment();
    world.attachments[0].mode = AttachmentMode::Design;
    assert!(
        compile(&world, &fragments, &options)
            .unwrap_err()
            .contains("совместите")
    );
    fragments[1].pose.position_m[2] = fragments[0].pose.position_m[2];
    fragments[1].mjcf = Some(MODEL.replace("name=\"центр\"", "name=\"центр\" euler=\"0 0 -90\""));
    let (compiled, _) = compile(&world, &fragments, &options).unwrap();
    assert!(
        compiled
            .open_contact_sim()
            .unwrap()
            .equality_active("attachment/стык")
            .unwrap()
    );
}

#[test]
fn exports_a_selected_ball_coordinate() {
    let mut f = fragment("блок");
    f.mjcf = Some(
        MODEL
            .replace(r#"<joint name="ход" type="slide" axis="0 0 1"/>"#, "")
            .replace(
                r#"<joint name="угол" axis="0 0 1" range="-90 90"/>"#,
                r#"<joint name="угол" type="ball"/>"#,
            )
            .replace(
                r#"<sensor><jointpos name="отсчёт" joint="ход"/></sensor>"#,
                "",
            ),
    );
    f.efforts = vec![FragmentEffort {
        name: "момент".into(),
        joint: "угол".into(),
        dof: 2,
    }];
    let (compiled, _) = compile(&scene(), &[f.clone()], &ContactBuildOptions::default()).unwrap();
    assert_eq!(compiled.joints[0].kind, CompiledJointKind::Ball);
    assert_eq!(compiled.efforts[0].dof, 2);
    assert_eq!(compiled.open_contact_sim().unwrap().nv(), 3);
    f.efforts[0].dof = 3;
    assert!(
        compile(&scene(), &[f], &ContactBuildOptions::default())
            .unwrap_err()
            .contains("нет степени свободы")
    );
}
