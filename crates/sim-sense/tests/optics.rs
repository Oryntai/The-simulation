mod common;

use std::collections::BTreeMap;
use std::path::Path;
use std::ptr;

use sim_mujoco::ffi;
use sim_runtime::{Evaluator, LogKind, Runtime, Truth, TruthId, TruthOut};
use sim_sense::{
    Detection, FrameTruth, FrameTruthReader, Intrinsics, Pose, SensorKind, checkerboard,
    compare_detections, install,
};
use sim_signal::{ImageFrame, PixelFormat, Rng, Time};
use sim_world::{Environment, SensorUse, World, compile_world, parse_world};

use common::{CatalogDir, near, run, sensor, world};

const BROWN: [(&str, f64); 5] = [
    ("k1", 0.14),
    ("k2", -0.04),
    ("k3", 0.02),
    ("p1", 0.010),
    ("p2", -0.007),
];

fn analytic(point: [f64; 3], lens: &Intrinsics) -> [f64; 2] {
    let x = point[0] / point[2];
    let y = point[1] / point[2];
    let radius2 = x.powi(2) + y.powi(2);
    let [a, b, c] = lens.radial;
    let [p, q] = lens.tangential;
    let radial = 1.0 + a * radius2 + b * radius2.powi(2) + c * radius2.powi(3);
    [
        lens.cx + lens.fx * (x * radial + 2.0 * p * x * y + q * (3.0 * x.powi(2) + y.powi(2))),
        lens.cy + lens.fy * (y * radial + p * (x.powi(2) + 3.0 * y.powi(2)) + 2.0 * q * x * y),
    ]
}

fn component(frame: &ImageFrame, x: usize, y: usize, channel: usize) -> f64 {
    f64::from(frame.data[(y * frame.width as usize + x) * 3 + channel]) / f64::from(u8::MAX)
}

#[test]
fn projected_sphere_centers_agree_to_a_tenth_of_a_pixel() {
    let dir = CatalogDir::new("сферы");
    let mut fields = vec![("spatial_samples", 8.0), ("cx", 31.37), ("cy", 23.79)];
    dir.add("pinhole", SensorKind::ColorCamera, &fields);
    fields.extend(BROWN);
    dir.add("distorted", SensorKind::ColorCamera, &fields);
    let world = world(
        r#"<geom name="сфера-А" size="0.10" pos="-0.42 0.28 3" rgba="1 0 0 1"/>
                         <geom name="сфера-Б" size="0.10" pos="0.50 -0.29 3" rgba="0 1 0 1"/>"#,
        "0 0 0",
        vec![sensor("первая", "pinhole"), sensor("вторая", "distorted")],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    let handles = install(&mut rt, &world, &dir.catalog()).unwrap();
    assert_eq!(
        rt.simplifications()
            .iter()
            .filter(|s| s.blocks_proof)
            .count(),
        2
    );
    run(&mut rt, 0.0);
    let obs = rt.observe();
    for h in handles {
        let frame = obs.frame(h.signal("image").unwrap()).unwrap();
        assert!(frame.valid);
        assert_eq!(frame.format, PixelFormat::Rgb8);
        let lens = h.intrinsics().unwrap();
        for (channel, xyz) in [[-0.42, 0.28, 3.0], [0.50, -0.29, 3.0]]
            .into_iter()
            .enumerate()
        {
            let mut area = 0.0;
            let mut centroid = [0.0; 2];
            for y in 0..frame.height as usize {
                for x in 0..frame.width as usize {
                    let weight = component(frame, x, y, channel);
                    area += weight;
                    centroid[0] += (x as f64 + 0.5) * weight;
                    centroid[1] += (y as f64 + 0.5) * weight;
                }
            }
            assert!(area > 15.0); // π(80·0.10/3)² ≈ 22 пикселя.
            let expected = analytic(xyz, lens);
            let from_api = lens.project(xyz).unwrap().unwrap();
            for i in 0..2 {
                near(centroid[i] / area, expected[i], 0.1);
                near(from_api[i], expected[i], 1e-12);
            }
        }
    }
}

fn edge_at(samples: &[f64], origin: usize) -> f64 {
    assert!(samples[0] != *samples.last().unwrap());
    let white = samples.iter().sum::<f64>();
    origin as f64
        + if samples[0] > samples[4] {
            white
        } else {
            5.0 - white
        }
}

fn fit(points: Vec<(f64, f64)>) -> (f64, f64) {
    let n = points.len() as f64;
    let x = points.iter().map(|p| p.0).sum::<f64>() / n;
    let y = points.iter().map(|p| p.1).sum::<f64>() / n;
    let slope = points.iter().map(|p| (p.0 - x) * (p.1 - y)).sum::<f64>()
        / points.iter().map(|p| (p.0 - x).powi(2)).sum::<f64>();
    (y - slope * x, slope)
}

fn locate_corner(frame: &ImageFrame, around: [f64; 2]) -> [f64; 2] {
    // Находим покрытие по суммарной яркости в окне шириной пять пикселей.
    // Точки на четырёх соседних строках/столбцах дают пересечение двух кромок.
    let x = around[0].floor() as usize;
    let y = around[1].floor() as usize;
    let neighbors = |c| [c - 3, c - 2, c + 2, c + 3];
    let (a, b) = fit(neighbors(y)
        .into_iter()
        .map(|row| {
            let samples: Vec<_> = (x - 2..=x + 2)
                .map(|column| component(frame, column, row, 0))
                .collect();
            (row as f64 + 0.5, edge_at(&samples, x - 2))
        })
        .collect());
    let (c, d) = fit(neighbors(x)
        .into_iter()
        .map(|column| {
            let samples: Vec<_> = (y - 2..=y + 2)
                .map(|row| component(frame, column, row, 0))
                .collect();
            (column as f64 + 0.5, edge_at(&samples, y - 2))
        })
        .collect());
    let px = (a + b * c) / (1.0 - b * d);
    [px, c + d * px]
}

#[test]
fn checkerboard_roundtrips_world_file_and_recovers_subpixel_corners() {
    let dir = CatalogDir::new("мишень");
    let mut fields = vec![("spatial_samples", 16.0), ("cx", 31.63), ("cy", 22.92)];
    dir.add("pinhole", SensorKind::ColorCamera, &fields);
    fields.extend(BROWN);
    dir.add("distorted", SensorKind::ColorCamera, &fields);
    let angle: f64 = 0.075;
    let (sin, cos) = angle.sin_cos();
    let transform = |x: f64, y: f64| [0.087 + x * cos - y * sin, 0.064 + x * sin + y * cos, 2.0];
    let cells = checkerboard(4, 4, 0.25, 0.002, [0.0, 0.0, 0.0, 1.0], [1.0; 4]).unwrap();
    assert_eq!(cells.len(), 16);
    assert!(checkerboard(1, 1, f64::NAN, 0.002, [0.0; 4], [1.0; 4]).is_err());
    let visual = cells
        .into_iter()
        .map(|mut cell| {
            let [x, y, z] = cell.pose.position_m;
            cell.pose.position_m = transform(x, y);
            cell.pose.position_m[2] += z;
            cell.pose.rpy_deg = Some([0.0, 0.0, angle.to_degrees()]);
            cell
        })
        .collect();
    let world = World {
        format: 3,
        name: "мишень".into(),
        seed: 417,
        physics_hz: 1000,
        gravity_m_s2: [0.0; 3],
        environment: Environment {
            density_kg_m3: 0.0,
            viscosity_pa_s: 0.0,
            wind_m_s: [0.0; 3],
            collision: Vec::new(),
            visual,
        },
        devices: Vec::new(),
        objects: Vec::new(),
        attachments: Vec::new(),
        sensors: ["pinhole", "distorted"]
            .into_iter()
            .map(|name| SensorUse {
                name: name.into(),
                part: name.into(),
                link: None,
                pose: Default::default(),
                joint: None,
                drive: None,
                params: BTreeMap::new(),
            })
            .collect(),
    };
    let path = Path::new("мишень.toml");
    let world = parse_world(path, &toml::to_string(&world).unwrap()).unwrap();
    let world = compile_world(path, &world, &BTreeMap::new()).unwrap();
    let mut rt = Runtime::load(world.clone()).unwrap();
    let handles = install(&mut rt, &world, &dir.catalog()).unwrap();
    run(&mut rt, 0.0);
    let obs = rt.observe();
    for h in handles {
        let frame = obs.frame(h.signal("image").unwrap()).unwrap();
        assert!(frame.valid);
        for (x, y) in [(0.0, 0.0), (0.25, 0.0), (0.0, 0.25), (-0.25, -0.25)] {
            let expected = analytic(transform(x, y), h.intrinsics().unwrap());
            let measured = locate_corner(frame, expected);
            near(measured[0], expected[0], 0.1);
            near(measured[1], expected[1], 0.1);
        }
    }
}

#[derive(Clone)]
pub struct Inspect<F> {
    pub reader: FrameTruthReader,
    pub check: F,
    pub counter: TruthId,
    pub count: u32,
}

impl<F> Evaluator for Inspect<F>
where
    F: Fn(&FrameTruth, &Truth<'_>) + Clone,
{
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        for data in self.reader.advance(truth)? {
            (self.check)(&data, truth);
            self.count += 1;
            out.write(self.counter, f64::from(self.count));
        }
        Ok(())
    }
}

fn value_mm(frame: &ImageFrame, x: usize, y: usize) -> u16 {
    let index = (y * frame.width as usize + x) * 2;
    u16::from_le_bytes(frame.data[index..index + 2].try_into().unwrap())
}

#[test]
fn depth_ids_and_poses_match_analytic_surfaces_and_mujoco_rays() {
    let dir = CatalogDir::new("глубина");
    let settings = [
        ("width", 81.0),
        ("height", 49.0),
        ("fx", 75.0),
        ("fy", 75.0),
        ("cx", 40.5),
        ("cy", 24.5),
        ("latency_s", 0.0175),
    ];
    dir.add("depth", SensorKind::DepthCamera, &settings);
    let mut holes = settings.to_vec();
    holes.push(("hole_probability", 1.0));
    dir.add("holes", SensorKind::DepthCamera, &holes);
    let mut world = world(
        r#"
        <geom name="плоскость" type="plane" size="3 3 .1" quat="0 1 0 0" pos="0 0 4"/>
        <geom name="сфера" type="sphere" size=".16" pos="-1.2 0 3"/>
        <geom name="капсула" type="capsule" size=".16 .25" pos="-.8 0 3"/>
        <geom name="эллипсоид" type="ellipsoid" size=".2 .2 .21" pos="-.4 0 3"/>
        <geom name="цилиндр" type="cylinder" size=".16 .30" pos="0 0 3"/>
        <geom name="блок" type="box" size=".16 .17 .20" pos=".4 0 3"/>
        <geom name="сетка" type="mesh" mesh="тетраэдр" pos="1.2 0 3"/>
        <geom name="дальняя" type="plane" size="3 3 .1" quat="0 1 0 0" pos="0 0 6"/>
        <geom name="невидимая" type="box" size="3 3 .05" pos="0 0 1" rgba="0 0 0 0"/>"#,
        "0 0 0",
        vec![sensor("глубина", "depth"), sensor("дыры", "holes")],
    );
    let mesh = r#"<asset><mesh name="тетраэдр" vertex="-.18 -.17 0 .18 -.17 0 0 .2 0 0 0 .35"
                                      face="0 2 1 0 1 3 1 2 3 2 0 3"/></asset>"#;
    world.mjcf = world
        .mjcf
        .replace("<worldbody>", &format!("{mesh}<worldbody>"));
    let mut rt = Runtime::load(world.clone()).unwrap();
    let handles = install(&mut rt, &world, &dir.catalog()).unwrap();
    let counter = rt.register_truth("проверено", "1").unwrap();
    let output = handles[0].signal("depth").unwrap();
    let check = move |frame: &FrameTruth, truth: &Truth<'_>| {
        assert_eq!(frame.signal, output);
        assert_eq!(frame.available, Time(frame.captured.0 + 17_500_000));
        assert_eq!(frame.row_times.as_ref(), vec![frame.captured; 49]);
        assert_eq!(frame.camera_pose.position_m, [0.0; 3]);
        let world_ray = |x: usize| [(x as f64 - 40.0) / 75.0, 0.0, 1.0];
        let names = [
            (10, "сфера"),
            (20, "капсула"),
            (30, "эллипсоид"),
            (40, "цилиндр"),
            (50, "блок"),
            (60, "плоскость"),
            (70, "сетка"),
        ];
        for (x, name) in names {
            let index = 24 * 81 + x;
            let ray = world_ray(x);
            let mut id = -1;
            let distance = unsafe {
                ffi::mj_ray(
                    truth.engine().model(),
                    truth.engine().data(),
                    [0.0; 3].as_ptr(),
                    ray.as_ptr(),
                    ptr::null(),
                    true,
                    -1,
                    &mut id,
                    ptr::null_mut(),
                )
            };
            assert!(distance > 0.0 && id >= 0);
            near(frame.depth_m[index].unwrap(), distance, 1e-7);
            assert_eq!(frame.object_ids[index], Some(id as u32));
            assert_eq!(
                frame
                    .objects
                    .iter()
                    .find(|o| o.id == id as u32)
                    .unwrap()
                    .name,
                name
            );
        }
        // Центральная ось цилиндра, свободная плоскость и луч через центр сферы.
        near(frame.depth_m[24 * 81 + 40].unwrap(), 2.7, 1e-12);
        near(frame.depth_m[24 * 81 + 60].unwrap(), 4.0, 1e-12);
        let expected_sphere = 3.0 - 0.16 / (1.0 + 0.4_f64.powi(2)).sqrt();
        near(frame.depth_m[24 * 81 + 10].unwrap(), expected_sphere, 1e-12);
        let sphere = frame.objects.iter().find(|o| o.name == "сфера").unwrap();
        assert_eq!(sphere.pose.position_m, [-1.2, 0.0, 3.0]);
        let block = frame.objects.iter().find(|o| o.name == "блок").unwrap();
        let q = (0.3_f64 / 2.0).sin_cos();
        let detection = Detection {
            object_id: sphere.id,
            pose: Pose {
                position_m: [-1.2, 0.4, 3.0],
                quat_wxyz: [q.1, 0.0, q.0, 0.0],
            },
        };
        let unexpected = Detection {
            object_id: u32::MAX,
            pose: block.pose,
        };
        let result = compare_detections(
            frame,
            &[sphere.id, block.id],
            &[detection.clone(), detection, unexpected],
            1,
        )
        .unwrap();
        assert_eq!(result.missed, [block.id]);
        assert_eq!(result.false_positives, [sphere.id, u32::MAX]);
        assert_eq!(result.pose_errors.len(), 1);
        near(result.pose_errors[0].translation_m, 0.4, 1e-12);
        near(result.pose_errors[0].rotation_rad, 0.3, 1e-12);
        assert!(compare_detections(frame, &[u32::MAX], &[], 1).is_err());
        let invalid = Detection {
            object_id: sphere.id,
            pose: Pose {
                position_m: [f64::NAN; 3],
                ..sphere.pose
            },
        };
        assert!(compare_detections(frame, &[sphere.id], &[invalid], 1).is_err());
    };
    rt.register_evaluator(
        "геометрическая сверка",
        world.physics,
        Inspect {
            reader: handles[0].frame_truth_reader().unwrap(),
            check,
            counter,
            count: 0,
        },
    )
    .unwrap();
    run(&mut rt, 0.017499999);
    assert!(rt.observe().frame(output).is_none());
    run(&mut rt, 0.0175);
    let obs = rt.observe();
    let frame = obs.frame(output).unwrap();
    assert!(frame.valid);
    assert_eq!(frame.at, Time(17_500_000));
    assert_eq!(frame.format, PixelFormat::DepthMm16);
    assert_eq!(value_mm(frame, 40, 24), 2700);
    assert_eq!(value_mm(frame, 60, 24), 4000);
    assert_eq!(value_mm(frame, 10, 24), 2851);
    let holes = obs.frame(handles[1].signal("depth").unwrap()).unwrap();
    assert!(holes.valid && holes.data.iter().all(|b| *b == 0));
    assert_eq!(
        rt.log()
            .entries()
            .iter()
            .filter(|e| e.kind == LogKind::Truth)
            .count(),
        2
    );
}

#[test]
fn checker_texture_is_a_material_on_an_ordinary_scene_geometry() {
    let dir = CatalogDir::new("материал");
    dir.add("camera", SensorKind::ColorCamera, &[]);
    let mut scene = world(
        r#"<geom type="plane" size=".6 .6 .05" pos="0 0 2" material="клетки"/>"#,
        "0 0 0",
        vec![sensor("кадр", "camera")],
    );
    scene.mjcf = scene.mjcf.replace("<worldbody>", r#"<asset>
        <texture name="рисунок" type="2d" builtin="checker" rgb1="1 1 1" rgb2="0 0 0" width="8" height="8"/>
        <material name="клетки" texture="рисунок" texrepeat="1 1" rgba="1 1 1 1"/>
        </asset><worldbody>"#);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let id = install(&mut rt, &scene, &dir.catalog()).unwrap()[0]
        .signal("image")
        .unwrap();
    run(&mut rt, 0.0);
    let obs = rt.observe();
    let frame = obs.frame(id).unwrap();
    let top_left = component(frame, 25, 17, 0);
    assert!(top_left == 0.0 || top_left == 1.0);
    assert_eq!(component(frame, 38, 30, 0), top_left);
    assert_eq!(component(frame, 25, 30, 0), 1.0 - top_left);
    assert_eq!(component(frame, 38, 17, 0), 1.0 - top_left);
}
