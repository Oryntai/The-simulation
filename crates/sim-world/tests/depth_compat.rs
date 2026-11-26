use std::fs;
use std::path::{Path, PathBuf};

use sim_device::Catalog;
use sim_mujoco::Sim;
use sim_world::{LegacyCatalog, extract_v2, load_and_compile_with_schematics, load_device};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../sim-cli/tests/fixtures/depth_compat")
        .canonicalize()
        .unwrap()
}

#[test]
fn прежний_device_мигрирует_с_сохранением_байтов_и_массы() {
    let path = fixtures().join("block.device");
    let source = fs::read_to_string(&path).unwrap();
    let device = load_device(&path, &LegacyCatalog).unwrap();
    assert_eq!(device.format, 3);
    assert_eq!(extract_v2(&device), Some(source.as_str()));

    // Берём окружение старой каретки, меняя только ссылку в обёртке стенда.
    let world = fixtures().join("stage6/corpus/world.toml");
    let mut scene = sim_world::parse_world(&world, &fs::read_to_string(&world).unwrap()).unwrap();
    scene.devices[0].file = path.clone();
    let instance = scene.devices[0].name.clone();
    let compiled = sim_world::compile_world(
        &world,
        &scene,
        &[(instance, sim_world::DeviceInput { path, device })].into(),
    )
    .unwrap();

    // Три сплошных прямоугольных звена. Размеры в мм из block.device, плотности из
    // прежнего каталога. 1 мм³ = 10⁻⁹ м³. Допуск 1 нг на округление умножений.
    let aluminium = LegacyCatalog.density_kg_m3("al6061-t6").unwrap();
    let polymer = LegacyCatalog.density_kg_m3("petg").unwrap();
    let expected = [
        aluminium * 4.0 * 8.0 * 140.0e-9,
        polymer * 12.0 * 30.0 * 130.0e-9,
        polymer * 12.0 * 30.0 * 130.0e-9,
    ];
    assert_eq!(compiled.bodies.len(), expected.len());
    for (body, mass) in compiled.bodies.iter().zip(expected) {
        assert!((body.mass_kg - mass).abs() < 1e-12, "{body:?}");
    }
    let mut sim = Sim::from_xml(&compiled.mjcf).unwrap();
    sim.step();
    assert!((sim.total_mass() - expected.iter().sum::<f64>()).abs() < 1e-12);
    assert_eq!(sim.nv(), 0, "в прежней модели все соединения неподвижны");
}

#[test]
fn прежний_мир_со_ссылкой_и_без_неё_сохраняет_тип_загрузки() {
    let path = fixtures().join("electric/devices/block/world.toml");
    let with = load_and_compile_with_schematics(&path).unwrap();
    assert_eq!(
        with.schematics.into_iter().collect::<Vec<_>>(),
        [(
            "устройство".into(),
            path.with_file_name("electrical-scheme.toml")
        )]
    );
    assert!(with.world.simplifications.iter().any(|s| s.blocks_proof));

    let path = fixtures().join("stage6/corpus/world.toml");
    let without = load_and_compile_with_schematics(&path).unwrap();
    assert!(without.schematics.is_empty());
    assert_eq!(without.world.bodies.len(), 1);
    assert_eq!(without.world.bodies[0].mass_kg, 2.0); // rod.toml.
}

#[test]
fn неизвестный_формат_устройства_отклоняется() {
    let path = fixtures().join("stage6/corpus/rod.toml");
    let source = fs::read_to_string(&path).unwrap();
    let invalid = source.replacen("format = 3", "format = 999", 1);
    assert_ne!(source, invalid);
    assert!(sim_world::parse_device(&path, &invalid).is_err());
}
