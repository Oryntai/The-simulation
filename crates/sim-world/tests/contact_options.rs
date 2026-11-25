use std::{collections::BTreeMap, path::Path};

use sim_signal::Rate;
use sim_world::*;

const FLOOR: &str = "environment/collision/0";
const BLOCK: &str = "блок&/блок&/collision/0";

// Синтетическая стендовая сцена: блок 2 кг, грань 0.2 м, опора 1 м,
// начальное вдавливание 1 мм и ускорение 10 м/с².
fn fixture() -> World {
    let mut floor = Geometry::new(Shape::Box {
        size_m: [1.0, 1.0, 0.2],
    });
    floor.pose.position_m[2] = -0.1;
    World {
        format: FORMAT,
        name: "Блок и опора".into(),
        seed: 0,
        physics_hz: 1000,
        gravity_m_s2: [0.0, 0.0, -10.0],
        environment: Environment {
            collision: vec![floor],
            ..Environment::default()
        },
        objects: vec![Object {
            name: "блок&".into(),
            pose: Pose {
                position_m: [0.0, 0.0, 0.099],
                ..Pose::default()
            },
            mass: Mass::Geometry {
                material: Material {
                    name: "проба".into(),
                    density_kg_m3: 2.0 / 0.2_f64.powi(3),
                    provenance: Provenance::Measured("Синтетическая масса стенда 2 кг".into()),
                },
            },
            collision: vec![Geometry::new(Shape::Box { size_m: [0.2; 3] })],
            visual: vec![],
            mounts: vec![],
        }],
        devices: vec![],
        attachments: vec![],
        sensors: vec![],
    }
}

fn options() -> ContactBuildOptions {
    // Нижние границы и формат solref/solimp заданы по справочнику MuJoCo
    // 4.0.1. Остальное — численные входы стенда для проверки передачи 5
    // разных коэффициентов и ненулевого отклика.
    ContactBuildOptions {
        physics: Rate::hz(2000),
        solver: Some(ContactSolver {
            cone: FrictionCone::Elliptic,
            solver: ConstraintSolver::Newton,
            impratio: 2.0,
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
            geometries: [FLOOR.into(), BLOCK.into()],
            settings: ContactSettings {
                dim: 6,
                friction: [0.2, 0.35, 0.01, 0.005, 0.015],
                solref: [-4000.0, -80.0],
                solreffriction: [0.0, -60.0],
                solimp: [0.9, 0.9, 0.001, 0.5, 2.0],
                margin_m: 0.0,
                gap_m: 0.0,
                adhesion_n: 0.0,
            },
        }],
        ..ContactBuildOptions::default()
    }
}

fn compile(world: &World, opts: &ContactBuildOptions) -> Result<CompiledWorld, String> {
    compile_world_with_contact_options(Path::new("стенд.toml"), world, &BTreeMap::new(), opts)
}

#[test]
fn explicit_pair_is_canonical_and_reaches_active_contacts() {
    let opts = options();
    let world = compile(&fixture(), &opts).unwrap();
    assert_eq!(world.physics, Rate::hz(2000).unwrap());
    assert!(world.mjcf.contains("блок&amp;/блок&amp;/collision/0"));
    let mut swapped = opts.clone();
    swapped.pairs[0].geometries.swap(0, 1);
    assert_eq!(compile(&fixture(), &swapped).unwrap(), world);

    let mut sim = world.open_contact_sim().unwrap();
    assert_eq!(sim.timestep(), 0.0005);
    let expected = opts.pairs[0].settings;
    assert_eq!(
        sim.explicit_contact_settings([BLOCK, FLOOR]).unwrap(),
        Some(expected)
    );
    let first = sim.step_with_contacts().unwrap();
    assert!(!first.contacts.is_empty());
    for point in first.contacts {
        assert_eq!(point.dim, expected.dim);
        assert_eq!(point.friction, expected.friction);
        assert_eq!(point.solref, expected.solref);
        assert_eq!(point.solreffriction, expected.solreffriction);
        assert_eq!(point.solimp, expected.solimp);
    }
    // После установления вертикальный баланс даёт N = mg = 20 Н.
    for _ in 0..2000 {
        sim.step();
    }
    let actual: f64 = sim
        .contact_wrenches()
        .unwrap()
        .iter()
        .map(|c| {
            if c.geometries[1] == BLOCK {
                c.force_n[2]
            } else {
                -c.force_n[2]
            }
        })
        .sum();
    println!("собранная опора: N={actual:.9} Н, mg=20 Н");
    assert!((actual - 20.0).abs() < 0.01); // 0.05% в опыте установившегося равновесия.
    assert!(
        world
            .simplifications
            .iter()
            .any(|x| x.subject == "contact" && x.blocks_proof)
    );
}

#[test]
fn explicit_pair_cannot_bypass_masks_or_body_exclusions() {
    let mut world = fixture();
    world.environment.collision[0].contact_mask = Some([1, 1]);
    world.objects[0].collision[0].contact_mask = Some([2, 2]);
    let opts = options();
    let error = compile(&world, &opts).unwrap_err();
    assert!(error.contains("запрещают пару"), "{error}");

    let mut excluded = opts.clone();
    excluded
        .excluded_bodies
        .push(["world".into(), "блок&/блок&".into()]);
    let error = compile(&fixture(), &excluded).unwrap_err();
    assert!(error.contains("запрещают пару"), "{error}");
    excluded.pairs.clear();
    assert!(
        compile(&fixture(), &excluded)
            .unwrap()
            .contact_candidates()
            .unwrap()
            .is_empty()
    );
}

#[test]
fn достижимая_исключённая_пара_блокирует_доказательство() {
    let mut opts = options();
    opts.pairs.clear();
    opts.excluded_bodies
        .push(["world".into(), "блок&/блок&".into()]);
    let world = compile(&fixture(), &opts).unwrap();
    let sim = world.open_contact_sim().unwrap();
    assert!(!sim.contact_pair_allowed([BLOCK, FLOOR]).unwrap());
    assert!(sim.contact_wrenches().unwrap().is_empty());
    // В исходном опыте грань уже пересекает опору на 1 мм.
    let mut original = compile(&fixture(), &options())
        .unwrap()
        .open_contact_sim()
        .unwrap();
    assert!(!original.step_with_contacts().unwrap().contacts.is_empty());
    assert!(
        world
            .simplifications
            .iter()
            .any(|note| note.subject == "contact/exclusions"
                && note.blocks_proof
                && note.text.contains(BLOCK)
                && note.text.contains(FLOOR))
    );
}

#[test]
fn жёсткая_группа_проверяет_соседние_ветви_и_геометрический_зазор() {
    let mut world = fixture();
    world.environment.collision.clear();
    world.objects.clear();
    world.devices.push(Placement {
        name: "стенд".into(),
        file: "block.toml".into(),
        pose: Pose::default(),
    });
    // Кубы со стороной 0,2 м. У смещения 0,15 м есть пересечение 0,05 м,
    // у смещения 1 м зазор заведомо больше диагонали куба.
    let link = |name: &str, parent: &str, x: f64| {
        format!(
            r#"
[[links]]
name = "{name}"
{parent}
pose = {{ position_m = [{x:?}, 0.0, 0.0] }}
joint = {{ kind = "fixed" }}
mass = {{ kind = "massless" }}
[[links.collision]]
shape = {{ kind = "box", size_m = [0.2, 0.2, 0.2] }}
"#
        )
    };
    for (offset, blocked) in [(0.15, true), (1.0, false)] {
        let data = format!(
            "format = 3\nname = \"Блок\"\n{}{}{}",
            link("основание", "", 0.0),
            link("первый", "parent = \"основание\"", 0.0),
            link("второй", "parent = \"основание\"", offset),
        );
        let device = toml::from_str(&data).unwrap();
        let devices = BTreeMap::from([(
            "стенд".into(),
            DeviceInput {
                path: "block.toml".into(),
                device,
            },
        )]);
        let compiled = compile_world(Path::new("стенд.toml"), &world, &devices).unwrap();
        let blockers: Vec<_> = compiled
            .simplifications
            .iter()
            .filter(|note| note.subject == "contact/exclusions" && note.blocks_proof)
            .collect();
        assert_eq!(blockers.len(), usize::from(blocked));
        if let [note] = blockers.as_slice() {
            assert!(note.text.contains("стенд/первый") && note.text.contains("стенд/второй"));
            assert!(!note.text.contains("стенд/основание"));
        }
    }
}

#[test]
fn фильтр_шарнира_оставляет_риск_защемления_в_огибающей() {
    let mut world = fixture();
    world.environment.collision.clear();
    world.objects.clear();
    world.devices.push(Placement {
        name: "стенд".into(), file: "block.toml".into(), pose: Pose::default(),
    });
    let first = "стенд/основание/collision/0";
    let second = "стенд/блок/collision/0";
    // У кубов со стороной 0,2 м при смещении 0,15 м пересечение 0,05 м.
    // Якорь посередине между центрами: поворот на 180° совмещает центры.
    // При смещении 1 м и ходе 5° номинальная хорда меньше зазора.
    // Мягкий предел не исключает перебег, поэтому пара остаётся в аудите.
    // При якоре в центре второго куба внешние сферы разделены при любом угле.
    for (offset, limit, anchor, blocked) in [
        (0.15_f64, 5.0, -0.15 / 2.0, true),
        (1.0, 180.0, -0.5, true),
        (1.0, 5.0, -0.5, true),
        (1.0, 5.0, 0.0, false),
    ] {
        let data = format!(r#"
format = 3
name = "Шарнир"
[[links]]
name = "основание"
joint = {{ kind = "free" }}
mass = {{ kind = "explicit", mass_kg = 2.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0], provenance = {{ measured = "Инерция численного стенда." }} }}
[[links.collision]]
shape = {{ kind = "box", size_m = [0.2, 0.2, 0.2] }}
[[links]]
name = "блок"
parent = "основание"
pose = {{ position_m = [{offset:?}, 0.0, 0.0] }}
joint = {{ kind = "hinge", axis = [0.0, 0.0, 1.0], anchor_m = [{anchor:?}, 0.0, 0.0], limits_deg = [0.0, {limit:?}] }}
mass = {{ kind = "explicit", mass_kg = 2.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0], provenance = {{ measured = "Инерция численного стенда." }} }}
[[links.collision]]
shape = {{ kind = "box", size_m = [0.2, 0.2, 0.2] }}
"#);
        let devices = BTreeMap::from([("стенд".into(), DeviceInput {
            path: "block.toml".into(), device: toml::from_str(&data).unwrap(),
        })]);
        let compiled = compile_world(Path::new("стенд.toml"), &world, &devices).unwrap();
        let sim = compiled.open_contact_sim().unwrap();
        assert!(!sim.contact_pair_allowed([first, second]).unwrap());
        assert_eq!(compiled.simplifications.iter().any(|note| note.blocks_proof
            && note.subject == "contact/exclusions" && note.text.contains(first)
            && note.text.contains(second)), blocked, "{offset}/{limit}/{anchor}");
    }
}

#[test]
fn плечо_сечения_учитывает_позу_и_точку_нагрузки() {
    // Стержень 2 м, локальная X после поворота стала Y. Его торцы лежат
    // в (1, 1, 0) и (1, 3, 0). От точки (4, 7, 0) до дальнего торца √45 м.
    let mut geometry = Geometry::new(Shape::Box {
        size_m: [2.0, 0.2, 0.2],
    });
    geometry.pose.position_m = [1.0, 2.0, 0.0];
    geometry.pose.rpy_deg = Some([0.0, 0.0, 90.0]);
    let arm = geometry
        .section_load_arm(SectionAxis::X, &[[4.0, 7.0, 0.0]])
        .unwrap()
        .unwrap();
    assert!((arm - 45_f64.sqrt()).abs() <= arm * f64::EPSILON);
    assert_eq!(
        geometry.section_load_arm(SectionAxis::X, &[]).unwrap(),
        Some(2.0)
    );
    assert!(
        geometry
            .section_load_arm(SectionAxis::X, &[[f64::NAN, 0.0, 0.0]])
            .is_err()
    );
}

#[test]
fn rejects_duplicates_unknown_pairs_and_engine_clipping() {
    let mut opts = options();
    let mut reversed = opts.pairs[0].clone();
    reversed.geometries.swap(0, 1);
    opts.pairs.push(reversed);
    assert!(
        compile(&fixture(), &opts)
            .unwrap_err()
            .contains("повторная пара")
    );

    let mut opts = options();
    opts.pairs[0].geometries[1] = "неизвестная геометрия".into();
    assert!(
        compile(&fixture(), &opts)
            .unwrap_err()
            .contains("неизвестная геометрия")
    );

    let mut opts = options();
    opts.pairs.clear();
    assert!(
        compile(&fixture(), &opts)
            .unwrap_err()
            .contains("отсутствует явная модель")
    );
    for bad in [f64::NAN, f64::INFINITY, f64::MAX, 0.0] {
        let mut opts = options();
        opts.pairs[0].settings.friction[0] = bad;
        assert!(compile(&fixture(), &opts).is_err());
    }
    let mut opts = options();
    opts.pairs[0].settings.solimp[1] = 1.0; // MuJoCo обрезал бы до mjMAXIMP.
    assert!(compile(&fixture(), &opts).is_err());
    let mut opts = options();
    opts.pairs[0].settings.solref = [-40_000_000.0, -80.0]; // h²ω² > 4.
    assert!(
        compile(&fixture(), &opts)
            .unwrap_err()
            .contains("шаг не разрешает")
    );
    let mut opts = options();
    opts.solver.as_mut().unwrap().cone = FrictionCone::Pyramidal;
    assert!(
        compile(&fixture(), &opts)
            .unwrap_err()
            .contains("игнорирует")
    );
}

#[test]
fn package_adds_normal_structural_elements_and_updates_clock() {
    let mut opts = ContactBuildOptions {
        physics: Rate::hz(4000),
        ..Default::default()
    };
    let mut world = fixture();
    world.objects[0].mounts.push(Mount {
        name: "стык".into(),
        kind: MountKind::Mechanical,
        interface: "контрольная плоскость".into(),
        pose: Pose::default(),
    });
    let mut node = world.objects[0].clone();
    node.name = "узел".into();
    node.pose.position_m[0] = 2.0;
    node.collision[0].contact_mask = Some([0, 0]);
    opts.added_objects.push(node);
    opts.added_attachments.push(Attachment {
        name: "узел".into(),
        first: "блок&/блок&/стык".into(),
        second: "узел/узел/стык".into(),
        mode: AttachmentMode::Runtime,
    });
    let built = compile(&world, &opts).unwrap();
    assert_eq!(built.physics.tick(4).0, 1_000_000);
    let sim = built.open_contact_sim().unwrap();
    assert_eq!(sim.timestep(), 0.00025);
    assert!(!sim.equality_active("attachment/узел").unwrap());
    assert_eq!(built.bodies.len(), 2);
    assert!(
        built
            .bodies
            .iter()
            .any(|x| x.name == "узел/узел" && (x.mass_kg - 2.0).abs() < 1e-14)
    );
    opts.added_objects[0].name = fixture().objects.remove(0).name;
    assert!(compile(&world, &opts).is_err());
}

#[test]
fn loader_rebuilds_with_options_and_preserves_strict_parsing() {
    use std::{fs, path::PathBuf};
    struct TemporaryFile(PathBuf);
    impl Drop for TemporaryFile {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.0);
        }
    }
    let file = TemporaryFile(
        std::env::temp_dir().join(format!("contact-options-{}.toml", std::process::id())),
    );
    let text = toml::to_string(&fixture()).unwrap();
    fs::write(&file.0, &text).unwrap();
    assert_eq!(
        load_and_compile_with_contact_options(&file.0, &options()).unwrap(),
        compile(&fixture(), &options()).unwrap()
    );
    fs::write(&file.0, format!("unknown = true\n{text}")).unwrap();
    assert!(
        load_and_compile_with_contact_options(&file.0, &options())
            .unwrap_err()
            .contains("unknown")
    );
}

#[test]
fn отключённая_пара_блокирует_доказательство_до_встречи_и_при_пересечении() {
    for height in [0.099, 0.2] {
        // Первая проба использует исходное вдавливание стенда, вторая начинает над опорой.
        let mut scene = fixture();
        scene.objects[0].pose.position_m[2] = height;
        for geom in [&mut scene.environment.collision[0], &mut scene.objects[0].collision[0]] {
            geom.contact_mask = Some([0, 0]);
            geom.friction = Some([0.0; 3]);
        }
        let defaults = ContactBuildOptions::default();
        let compiled = compile(&scene, &defaults).unwrap();
        assert!(compiled.simplifications.iter().any(|note| {
            note.subject == "contact" && note.blocks_proof
                && note.text.contains(BLOCK) && note.text.contains(FLOOR)
        }));
        let mut masked = compiled.open_contact_sim().unwrap();
        let enabled = compile(&scene, &options()).unwrap_err();
        assert!(enabled.contains("запрещают пару"));
        scene.objects[0].collision[0].contact_mask = None;
        scene.environment.collision[0].contact_mask = None;
        let mut physical = compile(&scene, &defaults).unwrap().open_contact_sim().unwrap();
        let mut touched = false;
        // 250 шагов по 1 мс, свободное падение пересекает опору в обеих пробах.
        for _ in 0..250 {
            masked.step();
            physical.step();
            assert!(masked.contacts().is_empty());
            touched |= !physical.contacts().is_empty();
        }
        let z = height - 10.0 * 0.001_f64.powi(2) * 250.0 * 251.0 / 2.0;
        assert!((masked.qpos()[2] - z).abs() < 1e-12);
        assert!(touched && physical.qpos()[2] > masked.qpos()[2]);
    }
}
