use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use sim_human::{Sex, reference_checks, scene::*};
use sim_mujoco::{Sim, ffi::mjtObj as Obj};

type V = Vector3<f64>;

fn spec() -> ParticipantSpec {
    marginal_matrix(Sex::Male).remove(4)
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.12e} вместо {expected:.12e}, допуск {tolerance:.2e}"
    );
}

fn q([w, x, y, z]: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

fn region(model: &ParticipantModel, label: &str, side: Side) -> RegionId {
    model
        .regions
        .iter()
        .find(|(_, surface)| surface.label == label && surface.side == side)
        .unwrap_or_else(|| panic!("не найден участок {label}"))
        .0
        .clone()
}

fn segment<'a>(model: &'a ParticipantModel, name: &str) -> &'a Segment {
    model
        .mass
        .segments
        .iter()
        .find(|s| s.name == name)
        .expect("сегмент паспорта")
}

fn world(model: &ParticipantModel, exclude: Option<&str>) -> Sim {
    // Независимые параметры испытательной каретки. Радиус зонда 0,5 мм,
    // масса 0,02 кг. Блок 2 кг не относится к участнику.
    let mut xml = model.mjcf.replace(
        "</worldbody>",
        r#"
      <body name="probe" pos="10 0 0"><freejoint name="probe_dof"/>
        <geom name="probe_geom" type="sphere" size="0.0005" mass="0.02"
              contype="1" conaffinity="0" condim="1"/>
      </body>
      <body name="block" pos="20 0 0">
        <geom name="block_geom" type="box" size="0.1 0.1 0.1" mass="2" contype="0" conaffinity="0"/>
      </body></worldbody>"#,
    );
    let end = xml.find('>').unwrap() + 1;
    // Формат начинается с корневого элемента, без декларации.
    xml.insert_str(end, r#"<option gravity="0 0 0" timestep="0.0001"/>"#);
    if let Some(body) = exclude {
        xml = xml.replace(
            "</contact>",
            &format!("<exclude body1=\"probe\" body2=\"{body}\"/></contact>"),
        );
    }
    Sim::from_xml(&xml).expect("MJCF зонда и участника")
}

fn place_probe(sim: &mut Sim, point: V) {
    let joint = sim
        .joints()
        .into_iter()
        .find(|joint| joint.name == "probe_dof")
        .unwrap();
    sim.qpos_mut()[joint.qpos_adr..joint.qpos_adr + 3].copy_from_slice(point.as_slice());
    sim.forward();
}

fn probe_points(sim: &Sim, port: &Port) -> (V, V) {
    let body = sim.id(Obj::mjOBJ_BODY, &port.body).unwrap();
    let (origin, rotation) = sim.body_pose(body);
    let rotation = q(rotation);
    (
        V::from(origin) + rotation * V::from(port.point_m),
        rotation * q(port.orientation) * V::z(),
    )
}

fn known_pair(sim: &Sim, geometry: &str) -> Option<sim_mujoco::contact::ContactInfo> {
    let probe = sim.id(Obj::mjOBJ_GEOM, "probe_geom").unwrap() as i32;
    let target = sim.id(Obj::mjOBJ_GEOM, geometry).unwrap() as i32;
    sim.contacts()
        .into_iter()
        .find(|contact| contact.geom.contains(&probe) && contact.geom.contains(&target))
}

#[test]
fn live_surface_inventory_includes_both_hands_and_all_phalanges() {
    let model = build(&spec()).expect("полнотелая сцена");
    // Два звена большого пальца, по три у остальных.
    for side in [Side::Right, Side::Left] {
        for n in 1..=5 {
            assert_eq!(
                model
                    .regions
                    .values()
                    .filter(|s| s.side == side && s.digit == Some(n))
                    .count(),
                if n == 1 { 2 } else { 3 }
            );
        }
    }
    assert_eq!(model.ports.len(), 6 * model.regions.len());
    assert!(model.resources.iter().all(|path| path.is_file()));
    let mut sim = world(&model, None);
    let mut covered = BTreeSet::new();
    let mut max_error: f64 = 0.0;
    let mut min_force = f64::INFINITY;
    for (id, surface) in &model.regions {
        assert!(surface.matches_masks(1, 0));
        assert!(!surface.matches_masks(0, 1));
        assert!(surface.effective_area_m2.is_none());
        let geom = sim.id(Obj::mjOBJ_GEOM, &surface.geometry).unwrap();
        let actual = sim.geoms().into_iter().find(|g| g.id == geom).unwrap();
        assert_eq!(sim.name(Obj::mjOBJ_BODY, actual.body), surface.body);
        let candidates = model.ports.iter().filter(|p| p.region == *id);
        for port in candidates {
            let (point, outward) = probe_points(&sim, port);
            // Аналитическое перекрытие аналитической поверхности сферой — 0,1 мм.
            place_probe(&mut sim, point + outward * 0.0004);
            let Some(hit) = known_pair(&sim, &surface.geometry) else {
                continue;
            };
            if hit.normal_force <= 0.0 {
                continue;
            }
            let expected = -0.0001;
            // Эллипсоиды используют GJK MuJoCo. 1 мкм — численное окно для этого опыта.
            close(hit.dist, expected, 1e-6);
            max_error = max_error.max((hit.dist - expected).abs());
            min_force = min_force.min(hit.normal_force);
            let normal = V::from(hit.normal);
            assert!(normal.dot(&outward).abs() >= 0.98, "{id:?}: {hit:?}");
            assert!(sim.numerically_ok());
            covered.insert(id.clone());
            // Та же точка с положительным зазором не даёт активного контакта пары.
            place_probe(&mut sim, point + outward * 0.001);
            assert!(known_pair(&sim, &surface.geometry).is_none(), "{id:?}");
            break;
        }
    }
    let missing: Vec<_> = model
        .regions
        .keys()
        .filter(|id| !covered.contains(*id))
        .collect();
    assert!(
        missing.is_empty(),
        "зонд не обнаружил геометрии {missing:?}"
    );
    eprintln!(
        "Зонд: {} участков, ошибка расстояния ≤ {:.3e} м, реакция ≥ {:.5} Н",
        covered.len(),
        max_error,
        min_force
    );
}

#[test]
fn moved_pose_ports_exclusions_and_disabled_geometry_are_distinct() {
    let model = build(&spec()).unwrap();
    let head = region(&model, "Голова", Side::Midline);
    let surface = &model.regions[&head];
    let port = model
        .ports
        .iter()
        .find(|p| p.region == head && p.direction == "z+")
        .unwrap();
    assert_eq!(model.mount(&port.site, &head).unwrap().body, surface.body);
    assert!(
        model
            .mount(&port.site, &RegionId("missing".into()))
            .is_err()
    );
    let head_axis = model
        .axes
        .iter()
        .find(|axis| axis.label == "T1_head_neck_AR")
        .unwrap();
    // Поворот 0,2 рад внутри исходной огибающей ±0,3 рад.
    let angle = 0.2;
    let pose = BTreeMap::from([(head_axis.joint.clone(), angle)]);
    model.validate_pose(&pose).unwrap();
    assert!(
        model
            .validate_pose(&BTreeMap::from([(head_axis.joint.clone(), 0.31)]))
            .is_err()
    );
    assert!(
        model
            .validate_pose(&BTreeMap::from([(head_axis.joint.clone(), f64::NAN)]))
            .is_err()
    );
    assert!(
        model
            .validate_pose(&BTreeMap::from([("missing".into(), 0.0)]))
            .is_err()
    );
    let dependent = model.axes.iter().find(|axis| axis.dependent).unwrap();
    assert!(
        model
            .validate_pose(&BTreeMap::from([(dependent.joint.clone(), 0.0)]))
            .is_err()
    );

    for excluded in [false, true] {
        let mut sim = world(&model, excluded.then_some(surface.body.as_str()));
        let joint = sim
            .joints()
            .into_iter()
            .find(|j| j.name == head_axis.joint)
            .unwrap();
        let (p0, _) = probe_points(&sim, port);
        let anchor = V::from(sim.joint_anchor(joint.id));
        let axis = V::from(sim.joint_axis(joint.id));
        sim.qpos_mut()[joint.qpos_adr] = angle;
        sim.forward();
        let (point, outward) = probe_points(&sim, port);
        let delta = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
        let analytic = anchor + delta * (p0 - anchor);
        close((point - analytic).norm(), 0.0, 1e-9);
        assert!((point - p0).norm() > 0.01);
        place_probe(&mut sim, point + outward * 0.0004);
        assert_eq!(known_pair(&sim, &surface.geometry).is_some(), !excluded);
        place_probe(&mut sim, V::new(10.0, 0.0, 0.0));
        assert!(sim.contacts().is_empty());
    }
    // Визуальная сетка головы по-прежнему существует, её маски выключены.
    let sim = Sim::from_xml(&model.mjcf).unwrap();
    let m = sim.model();
    let meshes: Vec<_> = sim
        .geoms()
        .into_iter()
        .filter(|g| g.mesh.is_some() && sim.name(Obj::mjOBJ_BODY, g.body) == surface.body)
        .collect();
    assert!(!meshes.is_empty());
    for mesh in meshes {
        let masks = unsafe {
            (
                *m.geom_contype.add(mesh.id),
                *m.geom_conaffinity.add(mesh.id),
            )
        };
        assert_eq!(masks, (0, 0));
    }
    assert!(!model.excluded_body_pairs.is_empty());
    for [first, second] in &model.excluded_body_pairs {
        assert!(model.excludes(second, first));
        assert!(sim.id(Obj::mjOBJ_BODY, first).is_some());
        assert!(sim.id(Obj::mjOBJ_BODY, second).is_some());
    }
}

#[test]
fn mirrors_rotate_with_axial_sign_and_preserve_wrapping_paths() {
    let model = build(&spec()).unwrap();
    let mut sim = Sim::from_xml(&model.mjcf).unwrap();
    let right = model
        .axes
        .iter()
        .find(|axis| axis.label == "elbow_flexion_r")
        .unwrap();
    let left = model
        .axes
        .iter()
        .find(|axis| axis.label == "mirror_elbow_flexion_r")
        .unwrap();
    // В родительской рамке отражение меняет Z для точек и X,Y для псевдовектора.
    let [x, y, z] = right.direction;
    assert_eq!(left.direction, [-x, -y, z]);
    let r = region(&model, "Дистальная фаланга, палец 1", Side::Right);
    let l = region(&model, "Дистальная фаланга, палец 1", Side::Left);
    let r_body = sim.id(Obj::mjOBJ_BODY, &model.regions[&r].body).unwrap();
    let l_body = sim.id(Obj::mjOBJ_BODY, &model.regions[&l].body).unwrap();

    let source_names = include_str!("../data/stage5/names.tsv");
    let mut tendons = Vec::new();
    let names: BTreeMap<_, _> = source_names
        .lines()
        .filter_map(|line| {
            let mut cols = line.split('\t');
            (cols.next() == Some("spatial")).then(|| {
                let id = cols.next().unwrap();
                (cols.next().unwrap(), id)
            })
        })
        .collect();
    for (&name, &id) in names.iter().filter(|(n, _)| n.starts_with("mirror_")) {
        let other = names[name.trim_start_matches("mirror_")];
        tendons.push((
            sim.id(Obj::mjOBJ_TENDON, id).unwrap(),
            sim.id(Obj::mjOBJ_TENDON, other).unwrap(),
        ));
    }
    assert_eq!(tendons.len(), 67); // Tendon_Arm_Hand_r.xml, все пути.
    for angle in [0.0, 0.67] {
        for axis in [right, left] {
            let dof = sim
                .joints()
                .into_iter()
                .find(|j| j.name == axis.joint)
                .unwrap();
            sim.qpos_mut()[dof.qpos_adr] = angle;
        }
        sim.forward();
        let [x, y, z] = sim.body_pose(r_body).0;
        close(
            (V::from(sim.body_pose(l_body).0) - V::new(x, -y, z)).norm(),
            0.0,
            1e-8,
        );
        assert!(x.is_finite() && y < 0.0);
        for &(l_id, r_id) in &tendons {
            let r_len = sim.tendon_length(r_id);
            let l_len = sim.tendon_length(l_id);
            assert!(r_len > 0.0 && l_len > 0.0);
            close(l_len, r_len, 2e-8);
            assert_eq!(sim.tendon_path_objects(l_id), sim.tendon_path_objects(r_id));
        }
    }
}

#[test]
fn marginals_and_uniform_mass_tensor_scale_independently() {
    // CDC, NHANES 2015–2018, таблицы 3, 5, 9 и 11, строка «20 и старше».
    for (population, stature, mass) in [
        (Sex::Female, [1.498, 1.613, 1.725], [49.8, 73.1, 119.6]),
        (Sex::Male, [1.628, 1.754, 1.874], [61.7, 87.4, 130.3]),
    ] {
        let matrix = marginal_matrix(population);
        assert_eq!(matrix.len(), 9);
        let mid = build(&matrix[4]).unwrap();
        for (i, config) in matrix.iter().enumerate() {
            assert_eq!(config.stature_m, stature[i / 3]);
            assert_eq!(config.mass_kg, mass[i % 3]);
            let model = if i == 4 {
                mid.clone()
            } else {
                build(config).unwrap()
            };
            let s = config.stature_m / matrix[4].stature_m;
            let k = config.mass_kg / matrix[4].mass_kg;
            assert!(
                model.anthropometry.stature_in_marginal_range
                    && model.anthropometry.mass_in_marginal_range
            );
            assert!(!model.anthropometry.joint_distribution_known);
            assert!(
                model.anthropometry.strength_coverage.is_none()
                    && model.anthropometry.soft_tissue_coverage.is_none()
            );
            close(
                model.anthropometry.neutral_proxy_height_m,
                config.stature_m,
                1e-12,
            );
            close(model.mass.mass_kg, config.mass_kg, 1e-10);
            close(
                model.mass.segments.iter().map(|seg| seg.mass_kg).sum(),
                config.mass_kg,
                1e-10,
            );
            close(
                (V::from(model.mass.com_m) - V::from(mid.mass.com_m) * s).norm(),
                0.0,
                1e-12,
            );
            let tensor = Matrix3::from(model.mass.inertia_kg_m2);
            close(
                (tensor - Matrix3::from(mid.mass.inertia_kg_m2) * (k * s * s)).norm(),
                0.0,
                1e-10,
            );
        }
    }
}

#[test]
fn affine_measurement_moves_attachment_and_recomputes_tensor() {
    let config = spec();
    let baseline = build(&config).unwrap();
    let id = region(&baseline, "Предплечье", Side::Right);
    let surface = &baseline.regions[&id];
    let SurfaceShape::Ellipsoid { semi_axes_m } = surface.shape else {
        panic!("ожидается эллипсоид")
    };
    // Испытательная размерная матрица: 1,2× ширина, 1,1× длина, 0,9× глубина.
    let dimensions = [
        2.4 * semi_axes_m[0],
        2.2 * semi_axes_m[1],
        1.8 * semi_axes_m[2],
    ];
    let corrected = build(&ParticipantSpec {
        proportions: vec![Proportion {
            region: id.clone(),
            measure: ProportionMeasure::LocalDimensions(dimensions),
            source: "контрольные размеры образца".into(),
        }],
        ..config
    })
    .unwrap();
    assert!(corrected.anthropometry.uniform_geometric_scale.is_none());
    close(
        corrected.anthropometry.neutral_proxy_height_m,
        config.stature_m,
        1e-12,
    );
    let SurfaceShape::Ellipsoid { semi_axes_m } = corrected.regions[&id].shape else {
        panic!("ожидается эллипсоид")
    };
    for (a, d) in semi_axes_m.into_iter().zip(dimensions) {
        close(2.0 * a, d, 1e-12);
    }
    let old = segment(&baseline, &surface.body);
    let new = segment(&corrected, &surface.body);
    close(new.mass_kg, old.mass_kg, 1e-12);
    let diagonal = V::new(1.2, 1.1, 0.9);
    close(
        (V::from(new.com_m) - diagonal.component_mul(&V::from(old.com_m))).norm(),
        0.0,
        1e-12,
    );
    let [[ixx, _, _], [_, iyy, _], [_, _, izz]] = old.inertia_kg_m2;
    // ∫y²dm = (Ixx + Izz − Iyy)/2, ∫z²dm = Ixx − ∫y²dm.
    let y2 = (ixx + izz - iyy) / 2.0;
    let expected = y2 * 1.21 + (ixx - y2) * 0.81;
    close(new.inertia_kg_m2[0][0], expected, 1e-10);
    let old_port = baseline
        .ports
        .iter()
        .find(|p| p.region == id && p.direction == "y-")
        .unwrap();
    let new_port = corrected.mount(&old_port.site, &id).unwrap();
    close(
        (V::from(new_port.point_m) - diagonal.component_mul(&V::from(old_port.point_m))).norm(),
        0.0,
        1e-10,
    );
    assert!((V::from(new_port.point_m) - V::from(old_port.point_m)).norm() > 0.005);
    // Площадь 20–40 см² задана калиброванной парой на исходном масштабе.
    let area = Interval {
        min: 0.002,
        max: 0.004,
    };
    let source = Source {
        reference: "условия подобия пары",
        locator: "измерение пятна A",
    };
    let uniform = baseline
        .area_from_similarity(&id, area, source)
        .unwrap()
        .unwrap();
    let scale = config.stature_m / baseline.anthropometry.reference_stature_m;
    close(uniform.min, area.min * scale.powi(2), 1e-15);
    close(uniform.max, area.max * scale.powi(2), 1e-15);
    assert!(
        corrected
            .area_from_similarity(&id, area, source)
            .unwrap()
            .is_none()
    );
    assert!(
        baseline
            .area_from_similarity(&id, Interval::point(f64::NAN), source)
            .is_err()
    );
    assert!(
        baseline
            .area_from_similarity(
                &id,
                area,
                Source {
                    locator: "",
                    ..source
                }
            )
            .unwrap()
            .is_none()
    );
    let sim = world(&corrected, None);
    let m = sim.model();
    let body = sim.id(Obj::mjOBJ_BODY, &new.name).unwrap();
    let local = unsafe { *m.body_iquat.cast::<[f64; 4]>().add(body) };
    let r = q(local).to_rotation_matrix();
    let exported = r.matrix()
        * Matrix3::from_diagonal(&sim.body_inertia(body).into())
        * r.matrix().transpose();
    close(exported[(0, 0)], expected, 1e-8);
    close(
        sim.body_mass(sim.id(Obj::mjOBJ_BODY, "block").unwrap()),
        2.0,
        1e-12,
    );
    close(
        sim.total_mass() - corrected.exported_mass.mass_kg,
        2.02,
        1e-10,
    );
    let loss = (corrected.exported_mass.mass_kg - corrected.mass.mass_kg).abs();
    assert!(loss < 0.0001, "округление в MJCF: {loss} кг");
    eprintln!("Округление суммы масс при записи: {loss:.8} кг");
    eprintln!(
        "Изменённое предплечье: Ixx {:.8} кг·м², аналитика {:.8}",
        exported[(0, 0)],
        expected
    );
}

#[test]
fn nonfinite_inputs_and_unknown_measurements_are_rejected() {
    for (stature_m, mass_kg) in [(f64::NAN, 70.0), (1.7, f64::INFINITY), (1.7, -1.0)] {
        assert!(
            build(&ParticipantSpec {
                stature_m,
                mass_kg,
                ..spec()
            })
            .is_err()
        );
    }
    let unknown = RegionId("missing".into());
    assert!(limits(&unknown).is_none());
    assert!(
        build(&ParticipantSpec {
            proportions: vec![Proportion {
                region: unknown,
                measure: ProportionMeasure::LocalDimensions([0.1; 3]),
                source: "опыт".into()
            }],
            ..spec()
        })
        .is_err()
    );
    let outside = build(&ParticipantSpec {
        stature_m: 1.55,
        mass_kg: 150.0,
        ..spec()
    })
    .unwrap();
    assert!(
        !outside.anthropometry.stature_in_marginal_range
            && !outside.anthropometry.mass_in_marginal_range
    );
}

#[test]
fn exposure_area_limits_and_fail_closed_are_checked() {
    use AssessmentIssue as Issue;
    use ExposureOutcome as Outcome;
    // Идентификатор сверяется с публикуемым паспортом. Источник испытания — аналитический стенд.
    let id = RegionId("r0008".into());
    let source = Source {
        reference: "испытание блока",
        locator: "интервал 1",
    };
    let area = Interval {
        min: 0.002,
        max: 0.004,
    };
    let mut context = ExposureContext {
        direction: Direction::Normal,
        mode: LoadMode::Sustained,
        patch: Patch::Circumferential,
        duration_s: 2.0,
        repetitions: 1,
        population: "образец".into(),
        geometry: "пятно A".into(),
        area_m2: Some((area, source)),
        complete: true,
        converged: true,
    };
    let mut limit = ExposureLimit {
        metric: Metric::MeanPressure,
        range: Some(Interval::point(21000.0)),
        source,
        applicability: Applicability {
            direction: context.direction,
            mode: context.mode,
            patch: context.patch,
            area_m2: Some(area),
            duration_s: Some(Interval { min: 1.0, max: 3.0 }),
            repetitions: Some([1, 1]),
            population: "образец",
            geometry: "пятно A",
        },
        status: LimitStatus::Derived,
    };
    let force = |n| Exposure {
        metric: Metric::MeanPressure,
        value: Interval::point(n),
    };
    let check =
        |n, context: &ExposureContext, limit| assess_exposure(&id, force(n), context, limit);
    let result = check(40.0, &context, Some(limit));
    assert_eq!(
        result.measurement,
        Some(Interval {
            min: 10000.0,
            max: 20000.0
        })
    );
    assert_eq!(result.tissue, Outcome::NotAnalyzed(Issue::MissingLimit));
    assert_eq!(result.design, Outcome::WithinLimit);
    assert_eq!(result.exceeded_orientations.len(), 2);
    assert_eq!(
        check(45.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::Indeterminate)
    );
    assert_eq!(check(100.0, &context, Some(limit)).design, Outcome::Fail);
    context.complete = false;
    assert_eq!(check(100.0, &context, Some(limit)).design, Outcome::Fail);
    assert_eq!(
        check(40.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::Incomplete)
    );
    context.complete = true;
    context.duration_s = 4.0;
    assert_eq!(
        check(100.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::OutsideApplicability)
    );
    context.duration_s = 2.0;
    context.converged = false;
    let unconverged = check(100.0, &context, Some(limit));
    assert_eq!(unconverged.design, Outcome::NotAnalyzed(Issue::Unconverged));
    assert_eq!(
        unconverged.measurement,
        Some(Interval {
            min: 25000.0,
            max: 50000.0
        })
    );
    context.converged = true;
    context.area_m2 = None;
    assert_eq!(
        check(100.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::MissingArea)
    );
    context.area_m2 = Some((area, source));
    limit.status = LimitStatus::Orientation;
    assert_eq!(
        check(100.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::MissingLimit)
    );
    limit.status = LimitStatus::Derived;
    limit.source.locator = "";
    assert_eq!(
        check(100.0, &context, Some(limit)).design,
        Outcome::NotAnalyzed(Issue::MissingLimit)
    );
    for input in [f64::NAN, f64::INFINITY, -1.0] {
        assert_eq!(
            check(input, &context, None).tissue,
            Outcome::NotAnalyzed(Issue::InvalidInput)
        );
    }
    assert_eq!(
        assess_exposure(
            &RegionId("missing".into()),
            force(40.0),
            &context,
            Some(limit)
        )
        .tissue,
        Outcome::NotAnalyzed(Issue::UnknownRegion)
    );
    assert_eq!(limits(&id).unwrap()[0].range, Some(Interval::point(150.0)));
}

#[test]
fn legacy_discrepancies_and_protective_checks_keep_numeric_evidence() {
    let model = build(&spec()).unwrap();
    let discrepancies = model.reference_discrepancies();
    assert_eq!(discrepancies.len(), 5);
    for discrepancy in discrepancies {
        assert!(discrepancy.simulated.is_finite() && discrepancy.reference.is_finite());
        assert!(discrepancy.abs_error() > 0.0 && !discrepancy.note.is_empty());
        eprintln!(
            "{}: {:.5} / {:.5}, {}",
            discrepancy.id,
            discrepancy.simulated,
            discrepancy.reference,
            discrepancy.tolerance_text()
        );
    }
    let checks = reference_checks();
    let guards: Vec<_> = checks
        .iter()
        .filter(|c| c.id.starts_with("human_verdict_"))
        .collect();
    assert_eq!(guards.len(), 5);
    assert!(guards.iter().all(|c| c.passed()));
}

#[test]
fn participant_joins_general_mjcf_composition() {
    use sim_mujoco::composition::{MjcfFragment, compose_mjcf};
    let model = build(&spec()).unwrap();
    assert_eq!(build(&spec()).unwrap().mjcf, model.mjcf);
    let base = r#"<mujoco><worldbody><body name="block" pos="20 0 0"><geom name="block_geom" type="box" size="0.1 0.1 0.1" mass="2"/></body></worldbody></mujoco>"#;
    let joined = compose_mjcf(
        base,
        &[MjcfFragment {
            prefix: "participant",
            source: &model.source,
            xml: &model.mjcf,
            position_m: [0.1, 0.2, 0.3],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }],
    )
    .unwrap();
    let sim = Sim::from_xml(&joined.xml).unwrap();
    let body = sim
        .id(Obj::mjOBJ_BODY, &format!("participant/{}", model.root_body))
        .unwrap();
    assert!(sim.body_pose(body).0[2] > 0.3);
    close(
        sim.body_mass(sim.id(Obj::mjOBJ_BODY, "block").unwrap()),
        2.0,
        1e-12,
    );
    assert_eq!(joined.resources.len(), model.resources.len());
    close(sim.total_mass(), 2.0 + model.exported_mass.mass_kg, 1e-10);
    for surface in model.regions.values() {
        assert!(
            sim.id(
                Obj::mjOBJ_GEOM,
                &format!("participant/{}", surface.geometry)
            )
            .is_some()
        );
    }
}

#[test]
fn independently_check_aggregate_inertia_with_spatial_mass_matrix() {
    let model = build(&ParticipantSpec {
        base: BaseVariant::MsHuman700Free,
        ..spec()
    })
    .unwrap();
    let sim = Sim::from_xml(&model.mjcf).unwrap();
    let root = sim
        .joints()
        .into_iter()
        .find(|j| Some(&j.name) == model.root_joint.as_ref())
        .unwrap();
    let (point, rotation) = sim.body_pose(root.body);
    let n = sim.nv();
    let h = sim.full_mass_matrix();
    let m = model.mass.mass_kg;
    let local_axis = q(rotation).to_rotation_matrix();
    let r = V::from(model.mass.com_m) - V::from(point);
    let tensor = Matrix3::from(model.mass.inertia_kg_m2);
    // 0.5 m|v + ω×r|² + 0.5 ω·I_cm·ω. Движок независимо собирает матрицу CRB.
    let energy = |speed: [f64; 6]| {
        let [vx, vy, vz, wx, wy, wz] = speed;
        let v = V::new(vx, vy, vz);
        let w = local_axis * V::new(wx, wy, wz);
        0.5 * (m * (v + w.cross(&r)).norm_squared() + w.dot(&(tensor * w)))
    };
    let mut max_relative: f64 = 0.0;
    for i in 0..6 {
        let mut speed = [0.0; 6];
        speed[i] = 1.0;
        let diag = h[(root.dof_adr + i) * n + root.dof_adr + i];
        let expected = 2.0 * energy(speed);
        max_relative = max_relative.max((diag - expected).abs() / expected);
        close(diag, expected, 2e-5 * expected);
        for j in i + 1..6 {
            speed[j] = 1.0;
            let expected_mix = 2.0 * energy(speed);
            let ii = (root.dof_adr + i) * n + root.dof_adr;
            let jj = (root.dof_adr + j) * n + root.dof_adr;
            let mix = h[ii + i] + h[jj + j] + 2.0 * h[ii + j];
            max_relative = max_relative.max((mix - expected_mix).abs() / expected_mix);
            close(mix, expected_mix, 2e-5 * expected_mix);
            speed[j] = 0.0;
        }
    }
    eprintln!(
        "Матрица масс: {} кг, I_cm={:?} кг·м²; относительная ошибка энергии ≤ {:.3e}",
        m, model.mass.inertia_kg_m2, max_relative
    );
}

#[test]
fn inactive_visual_surface_does_not_count_as_covered_patch() {
    let model = build(&spec()).unwrap();
    let head = region(&model, "Голова", Side::Midline);
    let mut sim = world(&model, None);
    let body = sim.id(Obj::mjOBJ_BODY, &model.regions[&head].body).unwrap();
    let visuals: Vec<_> = sim
        .geoms()
        .into_iter()
        .filter(|g| g.body == body && g.mesh.is_some())
        .collect();
    let points: Vec<V> = visuals
        .iter()
        .flat_map(|g| {
            let (xyz, axes) = sim.geom_pose(g.id);
            let rotation = Matrix3::from_row_slice(&axes);
            let origin = V::from(xyz);
            sim.mesh(g.mesh.unwrap())
                .positions
                .into_iter()
                .step_by(17)
                .map(move |p| origin + rotation * V::from(p.map(f64::from)))
        })
        .collect();
    assert!(!points.is_empty());
    let hole = points
        .into_iter()
        .find(|point| {
            // Центр сферы помещён ровно в вершину существующей визуальной сетки.
            place_probe(&mut sim, *point);
            sim.contacts().is_empty()
        })
        .expect("проверенная точка сетки вне внешнего покрытия");
    assert!((hole - V::from(sim.body_pose(body).0)).norm() < 0.4);
    assert!(sim.numerically_ok());
    assert!(
        model
            .regions
            .values()
            .all(|s| !visuals.iter().any(|v| v.name == s.geometry))
    );
}

#[test]
fn measured_length_and_circumference_keep_independent_values() {
    let config = spec();
    let baseline = build(&config).unwrap();
    let id = region(&baseline, "Плечо", Side::Right);
    // Измерения геометрического образца для испытания, в метрах.
    let length = 0.300;
    let circumference = 0.330;
    let mut config = ParticipantSpec {
        proportions: vec![Proportion {
            region: id.clone(),
            measure: ProportionMeasure::LengthAndCircumference {
                length_m: length,
                circumference_m: circumference,
            },
            source: "синтетический обмер образца".into(),
        }],
        ..config
    };
    let result = build(&config).unwrap();
    let SurfaceShape::Ellipsoid {
        semi_axes_m: [a, b, c],
    } = result.regions[&id].shape
    else {
        panic!("прокси")
    };
    // Независимая формула Рамануджана для периметра эллипса.
    let h = ((a - c) / (a + c)).powi(2);
    let contour =
        std::f64::consts::PI * (a + c) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
    close(2.0 * b, length, 1e-12);
    close(contour, circumference, 1e-7);
    eprintln!(
        "Локальный обмер: длина {:.6} м, обхват {:.6} м",
        2.0 * b,
        contour
    );
    config.proportions[0].measure = ProportionMeasure::LocalDimensions([f64::INFINITY, 0.2, 0.1]);
    assert!(build(&config).is_err());
    config.proportions[0].measure = ProportionMeasure::LocalDimensions([0.1, 0.2, f64::NAN]);
    assert!(build(&config).is_err());
}

#[test]
fn applicable_tissue_evidence_requires_region_scope_and_validation() {
    use AssessmentIssue as Issue;
    use ExposureOutcome as Outcome;
    let id = RegionId("r0008".into());
    // Данные синтетического опыта для проверки типизированного интерфейса.
    let source = Source {
        reference: "синтетический блок",
        locator: "таблица прогона A",
    };
    let area = Interval::point(0.004);
    let context = ExposureContext {
        direction: Direction::Tangential,
        mode: LoadMode::Quasistatic,
        patch: Patch::Local,
        duration_s: 2.0,
        repetitions: 1,
        population: "образец".into(),
        geometry: "B".into(),
        area_m2: Some((area, source)),
        complete: true,
        converged: true,
    };
    let mut criterion = TissueCriterion {
        region: id.clone(),
        limit: ExposureLimit {
            metric: Metric::MeanShear,
            range: Some(Interval::point(500.0)),
            source,
            applicability: Applicability {
                direction: Direction::Tangential,
                mode: LoadMode::Quasistatic,
                patch: Patch::Local,
                area_m2: Some(area),
                duration_s: Some(Interval::point(2.0)),
                repetitions: Some([1, 1]),
                population: "образец",
                geometry: "B",
            },
            status: LimitStatus::Measurement,
        },
        validation: Source {
            reference: "синтетический блок",
            locator: "отдельная калибровка B",
        },
    };
    let assess = |n, criterion: &TissueCriterion<'static>| {
        assess_exposure_with_tissue(
            &id,
            Exposure {
                metric: Metric::MeanShear,
                value: Interval::point(n),
            },
            &context,
            None,
            Some(criterion),
        )
    };
    let low = assess(1.0, &criterion);
    assert_eq!(low.measurement, Some(Interval::point(250.0)));
    assert_eq!(low.tissue, Outcome::WithinLimit);
    let high = assess(3.0, &criterion);
    assert_eq!(high.measurement, Some(Interval::point(750.0)));
    assert_eq!(high.tissue, Outcome::Fail);
    assert_eq!(high.design, Outcome::NotAnalyzed(Issue::MissingLimit));
    criterion.validation.locator = "";
    assert_eq!(
        assess(3.0, &criterion).tissue,
        Outcome::NotAnalyzed(Issue::MissingLimit)
    );
    criterion.validation = source;
    criterion.region = RegionId("r0001".into());
    assert_eq!(
        assess(3.0, &criterion).tissue,
        Outcome::NotAnalyzed(Issue::OutsideApplicability)
    );
}

#[test]
fn articulated_phalanges_contact_in_a_second_pose() {
    let model = build(&spec()).unwrap();
    let mut sim = world(&model, None);
    // Команда 0,35 рад лежит в исходном диапазоне IP. Проверяем обе стороны.
    let angle = 0.35;
    let mut pose = BTreeMap::new();
    let mut probes = Vec::new();
    for (side, label) in [
        (Side::Right, "ip_flexion"),
        (Side::Left, "mirror_ip_flexion"),
    ] {
        let axis = model.axes.iter().find(|a| a.label == label).unwrap();
        let joint = sim
            .joints()
            .into_iter()
            .find(|j| j.name == axis.joint)
            .unwrap();
        let id = region(&model, "Дистальная фаланга, палец 1", side);
        let anchor = V::from(sim.joint_anchor(joint.id));
        let direction = nalgebra::Unit::new_normalize(V::from(sim.joint_axis(joint.id)));
        let lever = |port: &Port| {
            (probe_points(&sim, port).0 - anchor)
                .cross(&direction)
                .norm()
        };
        let port = model
            .ports
            .iter()
            .filter(|p| p.region == id)
            .max_by(|a, b| lever(a).total_cmp(&lever(b)))
            .unwrap();
        let (before, _) = probe_points(&sim, port);
        let expected =
            anchor + UnitQuaternion::from_axis_angle(&direction, angle) * (before - anchor);
        pose.insert(axis.joint.clone(), angle);
        sim.qpos_mut()[joint.qpos_adr] = angle;
        probes.push((id, port, expected, before));
    }
    model.validate_pose(&pose).unwrap();
    sim.forward();
    for (_, port, expected, before) in &probes {
        let (point, _) = probe_points(&sim, port);
        close((point - expected).norm(), 0.0, 1e-9);
        assert!((point - before).norm() > 0.001);
    }
    for (id, port, _, _) in probes {
        let (point, outward) = probe_points(&sim, port);
        place_probe(&mut sim, point + outward * 0.0004);
        let geometry = &model.regions[&id].geometry;
        close(known_pair(&sim, geometry).unwrap().dist, -0.0001, 1e-6);
        sim.step();
        assert!(sim.numerically_ok());
        assert!(known_pair(&sim, geometry).unwrap().normal_force > 0.0);
        sim.qvel_mut().fill(0.0);
    }
}
