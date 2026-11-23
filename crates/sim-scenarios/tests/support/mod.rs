//! Общие заготовки приёмочных тестов конструктора.
#![allow(dead_code)]

use std::path::{Path, PathBuf};

use sim_boards::{Connection, SupplyRail};
use sim_device::*;
use sim_scenarios::device_templates::template;

pub struct TempDir(pub PathBuf);

impl TempDir {
    pub fn new() -> Self {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static ID: AtomicUsize = AtomicUsize::new(0);
        let path = std::env::temp_dir().join(format!(
            "sim-project-test-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir(&path).unwrap();
        Self(path)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

pub fn device() -> Device {
    let mut d = template(0);
    d.name = "Стенд конструктора".into();
    d.mount = BodyRegion::None;
    d.links[0].near_body = BodyRegion::None;
    d.links[0].joint = JointSpec::fixed();
    d.links[0].length_mm = 300.0;
    d.links[0].end_mass_kg = 0.2;
    d.links[0].settings = LinkSettings {
        bolt_size: Some("M4".into()),
        bolt_grade: Some("8.8".into()),
        bolt_count: Some(2),
        thread: Some("through".into()),
        plate_mm: Some(12.0),
        edge_mm: Some(10.0),
        bolt_lever_mm: Some(15.0),
        ..Default::default()
    };
    d.usage = UseProfile {
        ambient_c: 30.0,
        hours_per_day: 0.25,
        long_wear: false,
        cycles: 10_000.0,
    };
    d.settings = DeviceSettings {
        cold_c: Some(15.0),
        accessory_mass_kg: Some(0.2),
        inventory_source: Some("Протокол масс и монтажа стенда".into()),
        contact_source: Some("Протокол ограждения стенда".into()),
        ..Default::default()
    };
    let e = &mut d.electronics;
    e.series = 1;
    e.parallel = 2;
    e.regulator.clear();
    e.cell = "LG INR21700-M50".into();
    e.wire_m = 0.4;
    e.bms = true;
    e.connections = vec![
        Wiring(Connection::Supply {
            rail: SupplyRail::Vsys,
            volts: 3.6,
        }),
        Wiring(Connection::Ground { shared: Some(true) }),
        Wiring(Connection::DigitalInput {
            pin: 3,
            low_v: 0.0,
            high_v: 3.3,
        }),
        Wiring(Connection::Output { pin: 2, net: 1 }),
        Wiring(Connection::AnalogInput {
            pin: 26,
            volts: 1.65,
            reference_v: Some(3.3),
        }),
        Wiring(Connection::Load {
            pin: 2,
            milliamps: 0.0,
            sinking: false,
        }),
    ];
    // Параметры контрольной батареи из sim-battery/tests/safety.rs.
    use sim_battery::{BmsSpec, Bq769x0Config, MosfetSpec};
    let b = BmsSpec::bq769x0(&Bq769x0Config {
        r_sense_ohm: 0.002,
        scd_mv: 44.0,
        scd_delay_us: 70,
        ocd_mv: 20.0,
        ocd_delay_ms: 160.0,
        cell_ovp_v: Some(4.25),
        cell_uvp_v: Some(2.8),
        mosfet: MosfetSpec {
            vds_max_v: 40.0,
            id_pulse_a: 500.0,
            id_pulse_width_s: 2e-4,
            rds_on_ohm: 0.002,
            avalanche_energy_j: Some(0.1),
        },
    })
    .unwrap();
    e.settings = ElectricalSettings {
        resistance_spread: Some(0.1),
        group_spread_v: Some(0.02),
        interconnect_ohm: Some(0.002),
        contacts_ohm: Some(0.003),
        wire_kind: Some("awg".into()),
        awg: Some(16),
        insulation: Some("silicone".into()),
        cooling: Some("bundled".into()),
        fuse_position_m: Some(0.0),
        bms_name: Some(b.name),
        bms_scd_min_a: Some(b.scd_trip_min_a),
        bms_scd_max_a: Some(b.scd_trip_max_a),
        bms_scd_delay_s: Some(b.scd_delay_max_s),
        bms_ocd_min_a: Some(b.ocd_trip_min_a),
        bms_ocd_max_a: Some(b.ocd_trip_max_a),
        bms_ocd_min_s: Some(b.ocd_delay_min_s),
        bms_ocd_max_s: Some(b.ocd_delay_max_s),
        bms_ovp_v: b.cell_ovp_v,
        bms_uvp_v: b.cell_uvp_v,
        mosfet_v: Some(b.mosfet.vds_max_v),
        mosfet_a: Some(b.mosfet.id_pulse_a),
        mosfet_s: Some(b.mosfet.id_pulse_width_s),
        mosfet_ohm: Some(b.mosfet.rds_on_ohm),
        mosfet_j: b.mosfet.avalanche_energy_j,
        enclosure_area_m2: Some(0.06),
        enclosure_height_mm: Some(100.0),
        enclosure_wall_mm: Some(2.0),
        enclosure_k: Some(0.18),
        enclosure_h: Some(3.0),
        enclosure_emissivity: Some(0.85),
        enclosure_body_fraction: Some(0.0),
        spacer_mm: Some(5.0),
        spacer_k: Some(0.05),
        skin_s: Some(900.0),
        skin_class: Some("plastic".into()),
        skin_fraction: Some(0.03),
        charger_a: Some(2.0),
        charger_v: Some(4.2),
        charge_min_c: Some(10.0),
        charge_max_c: Some(30.0),
        board_avg_a: Some(0.08),
        board_peak_a: Some(0.12),
        board_thermal_kw: Some(30.0),
        board_temp_limit_c: Some(70.0),
        board_source: Some("Тепловой стенд платы".into()),
        sensors_declared: Some(true),
        firmware_s: Some(0.0065),
        sequencing_source: Some(
            "Осциллограммы контрольного стенда при включении и отключении".into(),
        ),
        ..Default::default()
    };
    d
}

pub fn moving() -> Device {
    let mut d = device();
    d.mount = BodyRegion::UpperArm;
    d.links[0].near_body = BodyRegion::Hand;
    d.links[0].contact_lever_mm = 300.0;
    let driver = template(0).links.remove(0);
    d.links[0].joint = driver.joint;
    d.links[0].joint.pinch_gap_mm = 110.0;
    d.links[0].joint.torque_limit_nm = 10.0;
    d.links[0].joint.speed_limit_dps = 20.0;
    let s = &mut d.links[0].settings;
    // Границы испытательного сценария, включая подложенную известную инерцию.
    s.inertia_kg_m2 = Some(0.03);
    s.inertia_source = Some("Эталонная инерция маятника".into());
    s.acceleration_s = Some(0.25);
    s.move_s = Some(0.5);
    s.rest_s = Some(2.0);
    d.electronics.series = 6;
    d.electronics.connections.push(Wiring(Connection::Motor {
        pin: 2,
        direct: false,
    }));
    d
}

pub fn printed() -> Device {
    let mut d = device();
    let l = &mut d.links[0];
    l.material = "pla".into();
    l.length_mm = 60.0;
    l.end_mass_kg = 4.0;
    l.section = Section::Rect {
        b_mm: 20.0,
        h_mm: 10.0,
    };
    l.print = Some(PrintSpec {
        layer_mm: 0.2,
        walls: 4,
        infill_pct: 100.0,
        across_layers: false,
    });
    l.settings.bbox_x_mm = Some(60.0);
    l.settings.bbox_y_mm = Some(20.0);
    l.settings.bbox_z_mm = Some(10.0);
    l.settings.wall_mm = Some(2.0);
    l.settings.overhang_deg = Some(30.0);
    l.settings.bridge_mm = Some(5.0);
    l.settings.print_hole_mm = Some(0.0);
    d.settings.volume_x_mm = Some(256.0);
    d.settings.volume_y_mm = Some(256.0);
    d.settings.volume_z_mm = Some(256.0);
    d.settings.nozzle_mm = Some(0.4);
    d.settings.enclosed = Some(true);
    d.settings.supports = Some(false);
    d.usage.ambient_c = sim_materials::catalog::ROOM_C;
    d
}

pub fn firmware(d: &mut Device, dir: &Path) {
    // Записанный образ эталонного теста RP2040 из sim-boards.
    std::fs::write(
        dir.join("pico.elf"),
        include_bytes!("../../../sim-boards/testdata/pico.elf"),
    )
    .unwrap();
    d.electronics.firmware = "pico.elf".into();
    d.electronics.firmware_cases.push(FirmwareSettings {
        boot: Some(true),
        input_pin: Some(3),
        input_high: Some(true),
        adc: Some(0),
        adc_v: Some(1.65),
        output_pin: Some(2),
        pwm_min: Some(0.25),
        pwm_max: Some(0.25),
        reason: Some("Эталонный ШИМ платы".into()),
        ..Default::default()
    });
}
