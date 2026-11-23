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
