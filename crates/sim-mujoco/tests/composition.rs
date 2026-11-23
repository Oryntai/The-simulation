use std::path::Path;

use sim_mujoco::composition::{MjcfFragment, compose_mjcf};
use sim_mujoco::{Sim, ffi::mjtObj as Obj};

// Аналитическая постановка: два независимых блока на составных осях.
// Моменты центра масс выведены из формулы прямоугольного параллелепипеда.
const MODEL: &str = r#"
<mujoco model="блок">
    <compiler angle="degree" fusestatic="false" alignfree="false" usethread="false"/>
    <option gravity="0 0 0" timestep="0.01" solver="PGS"/>
    <default><joint damping="0.125"/><geom contype="0" conaffinity="0"/></default>
    <worldbody>
        <body name="опора" pos="0 0 0.5"><site name="вход"/></body>
        <body name="блок" pos="0 0 1">
            <inertial pos="0 0 0" mass="3" diaginertia="0.0425 0.0125 0.05"/>
            <joint name="ход" type="slide" axis="1 0 0"/>
            <joint name="угол" axis="0 0 1" range="-90 90"/>
            <geom name="поверхность" type="box" size="0.1 0.2 0.05"/>
            <site name="выход" pos="0 0 0.25"/>
        </body>
    </worldbody>
    <tendon><spatial name="трасса"><site site="вход"/><site site="выход"/></spatial></tendon>
    <equality><joint name="связь" joint1="угол" active="false"/></equality>
    <sensor><jointpos name="отсчёт" joint="ход"/></sensor>
</mujoco>
"#;
const BASE: &str = r#"<mujoco model="стержень">
<compiler angle="radian" fusestatic="false" alignfree="false" usethread="false"/>
<option gravity="0 0 -10" timestep="0.000819128062993" solver="CG"/>
<worldbody><body name="неподвижный" pos="1 2 3"/></worldbody></mujoco>"#;

fn block(prefix: &str) -> MjcfFragment<'_> {
    MjcfFragment {
        prefix,
        source: Path::new("блок.xml"),
        xml: MODEL,
        position_m: [0.0; 3],
        quat_wxyz: [1.0, 0.0, 0.0, 0.0],
    }
}

fn approximate(a: f64, b: f64) {
    // Допуск для поворота и округления литералов аналитического опыта.
    assert!((a - b).abs() < 1e-12, "{a} != {b}");
}

#[test]
fn copies_mechanism_and_keeps_world_options() {
    let a = block("первый");
    let b = MjcfFragment {
        position_m: [2.0, -1.0, 3.0],
        // Четверть оборота вокруг Z.
        quat_wxyz: [1.0, 0.0, 0.0, 1.0],
        ..block("второй")
    };
    let compiled = compose_mjcf(BASE, &[a, b]).unwrap();
    let mut sim = Sim::from_xml(&compiled.xml).unwrap();
    assert_eq!((sim.nbody(), sim.nv(), sim.ntendon()), (6, 4, 2));
    assert_eq!(sim.total_mass(), 6.0);
    assert_eq!(
        sim.body_pose(sim.id(Obj::mjOBJ_BODY, "неподвижный").unwrap())
            .0,
        [1.0, 2.0, 3.0]
    );
    assert_eq!(
        sim.body_pose(sim.id(Obj::mjOBJ_BODY, "первый/блок").unwrap())
            .0,
        [0.0, 0.0, 1.0]
    );
    assert_eq!(
        sim.body_pose(sim.id(Obj::mjOBJ_BODY, "второй/блок").unwrap())
            .0,
        [2.0, -1.0, 4.0]
    );
    let axis = sim.id(Obj::mjOBJ_JOINT, "второй/ход").unwrap();
    for (x, target) in sim.joint_axis(axis).into_iter().zip([0.0, 1.0, 0.0]) {
        approximate(x, target);
    }
    let range = sim.joints()[sim.id(Obj::mjOBJ_JOINT, "второй/угол").unwrap()].range;
    approximate(range[1], std::f64::consts::FRAC_PI_2);
    approximate(
        sim.tendon_length(sim.id(Obj::mjOBJ_TENDON, "второй/трасса").unwrap()),
        0.75,
    );
    assert!(sim.id(Obj::mjOBJ_EQUALITY, "второй/связь").is_some());
    let sensor = sim.id(Obj::mjOBJ_SENSOR, "первый/отсчёт").unwrap();
    let axis = sim.id(Obj::mjOBJ_JOINT, "первый/ход").unwrap();
    let qpos = sim.joints()[axis].qpos_adr;
    sim.qpos_mut()[qpos] = 0.125;
    sim.forward();
    // Датчик измерил ход, а длина первого пути изменилась по теореме Пифагора.
    // SAFETY: адрес отсчёта взят из таблицы датчиков той же модели.
    let reading = unsafe {
        *sim.data()
            .sensordata
            .add(*sim.model().sensor_adr.add(sensor) as usize)
    };
    approximate(reading, 0.125);
    let first = sim.id(Obj::mjOBJ_TENDON, "первый/трасса").unwrap();
    let second = sim.id(Obj::mjOBJ_TENDON, "второй/трасса").unwrap();
    approximate(sim.tendon_length(first), 0.75_f64.hypot(0.125));
    approximate(sim.tendon_length(second), 0.75);
    assert_eq!(
        sim.model().opt.solver as u32,
        sim_mujoco::ffi::mjtSolver::mjSOL_CG
    );
    assert_eq!(sim.gravity(), [0.0, 0.0, -10.0]);
    assert_eq!(sim.timestep(), 0.000819128062993);
    assert_eq!(Sim::from_xml(MODEL).unwrap().total_mass(), 3.0);
    assert_eq!(compiled, compose_mjcf(BASE, &[a, b]).unwrap());
}

#[test]
fn rejects_conflict_missing_reference_and_nonfinite_pose() {
    assert!(
        compose_mjcf(BASE, &[block("блок"), block("блок")])
            .unwrap_err()
            .contains("конфликт")
    );
    let broken = MODEL.replace("<site site=\"вход\"/>", "<site site=\"отсутствует\"/>");
    assert!(
        compose_mjcf(
            BASE,
            &[MjcfFragment {
                xml: &broken,
                ..block("блок")
            }]
        )
        .is_err()
    );
    assert!(
        compose_mjcf(
            BASE,
            &[MjcfFragment {
                position_m: [f64::NAN, 0.0, 0.0],
                ..block("блок")
            }]
        )
        .unwrap_err()
        .contains("конечное")
    );
}

#[test]
fn rejects_explicit_nonfinite_values_and_unsupported_sections() {
    let fragment = block("блок");
    for parameter in ["NaN", "&#x4e;aN", "-Inf", "1e999"] {
        let xml = MODEL.replace("damping=\"0.125\"", &format!("damping=\"{parameter}\""));
        assert!(
            compose_mjcf(
                BASE,
                &[MjcfFragment {
                    xml: &xml,
                    ..fragment
                }]
            )
            .unwrap_err()
            .contains("неконечное")
        );
    }
    let xml = MODEL.replace(
        "</mujoco>",
        "<keyframe><key name=\"кадр\"/></keyframe></mujoco>",
    );
    assert!(
        compose_mjcf(
            BASE,
            &[MjcfFragment {
                xml: &xml,
                ..fragment
            }]
        )
        .unwrap_err()
        .contains("keyframe")
    );
    let xml = MODEL.replace("fusestatic=\"false\"", "fusestatic=\"true\"");
    assert!(
        compose_mjcf(
            BASE,
            &[MjcfFragment {
                xml: &xml,
                ..fragment
            }]
        )
        .unwrap_err()
        .contains("сохранять тела")
    );
}

#[test]
fn keeps_explicit_inertials_when_local_inertia_compiler_differs() {
    let xml = MODEL.replace(
        "angle=\"degree\"",
        "angle=\"degree\" inertiafromgeom=\"false\"",
    );
    let fragment = MjcfFragment {
        xml: &xml,
        ..block("блок")
    };
    let joined = compose_mjcf(BASE, &[fragment]).unwrap();
    let engine = Sim::from_xml(&joined.xml).unwrap();
    let body = engine.id(Obj::mjOBJ_BODY, "блок/блок").unwrap();
    approximate(engine.body_mass(body), 3.0);
    approximate(engine.body_inertia(body)[2], 0.05);
    assert_eq!(engine.timestep(), 0.000819128062993);
}
