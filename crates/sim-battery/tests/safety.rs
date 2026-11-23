//! Негативные и сквозные тесты оценки батареи. Базовая сборка обязана пройти всё без
//! `NotAnalyzed`: иначе негативные тесты могли бы падать по чужой причине или проходить вхолостую.

use sim_battery::*;
use sim_circuit::battery::CellParams;

fn mosfet() -> MosfetSpec {
    MosfetSpec {
        vds_max_v: 40.0,
        id_pulse_a: 500.0,
        id_pulse_width_s: 2e-4,
        rds_on_ohm: 0.002,
        avalanche_energy_j: Some(0.1),
    }
}

fn bms() -> BmsSpec {
    BmsSpec::bq769x0(&Bq769x0Config {
        r_sense_ohm: 0.002,
        scd_mv: 44.0,
        scd_delay_us: 70,
        ocd_mv: 20.0,
        ocd_delay_ms: 160.0,
        cell_ovp_v: Some(4.25),
        cell_uvp_v: Some(2.8),
        mosfet: mosfet(),
    })
    .expect("настройки BMS допустимы")
}

fn good_pack() -> PackDesign {
    PackDesign {
        cell: CellSpec::lg_m50(),
        series: 4,
        parallel: 2,
        cell_resistance_spread: 0.1,
        group_voltage_spread_v: 0.02,
        interconnect_r_ohm: 0.002,
        contact_r_ohm: 0.003,
        wire: WireRun {
            conductor: Conductor::Awg(16),
            insulation: Insulation::Silicone,
            length_m: 0.4,
            cooling: WireCooling::Bundled,
            outer_diameter_mm: None,
        },
        fuse: FuseSpec::ato(10),
        bms: Some(bms()),
        enclosure: Some(EnclosureDesign {
            outer_area_m2: 0.06,
            height_m: 0.1,
            wall_thickness_m: 0.002,
            wall_k_w_mk: 0.18,
            h_inner_w_m2k: 3.0,
            emissivity: 0.85,
            body_side_fraction: 0.3,
        }),
        skin: Some(SkinInterface {
            spacer_thickness_m: 0.005,
            spacer_k_w_mk: 0.05,
            contact_time_s: 28_800.0,
            surface_class: ContactClass::Plastics,
            contact_body_fraction: 0.03,
        }),
        worn_on_body: true,
    }
}

fn good_load() -> LoadProfile {
    LoadProfile {
        steps: vec![
            LoadStep {
                current_a: 3.0,
                duration_s: 600.0,
            },
            LoadStep {
                current_a: 6.0,
                duration_s: 10.0,
            },
        ],
        cycles: 48,
        ambient_min_c: 15.0,
        ambient_max_c: 30.0,
        load_v_min: Some(10.0),
        load_v_max: Some(20.0),
        charge: Some(ChargeProfile {
            current_a: 2.0,
            voltage_v: 16.8,
            ambient_min_c: 10.0,
            ambient_max_c: 30.0,
        }),
    }
}

fn failed(f: &[BatteryFinding], kind: BatteryCheck) -> Vec<&BatteryFinding> {
    f.iter()
        .filter(|x| x.kind == kind && x.outcome == Outcome::Fail)
        .collect()
}

/// Сборка получила `Fail` именно по `kind`, и в пояснении названа причина.
fn assert_fail(f: &[BatteryFinding], kind: BatteryCheck, reason: &str) {
    assert_eq!(overall(f), Outcome::Fail);
    let hits = failed(f, kind);
    assert!(!hits.is_empty(), "нет Fail по {kind:?}: {f:#?}");
    assert!(
        hits.iter().any(|x| x.detail.contains(reason)),
        "в Fail по {kind:?} нет причины «{reason}»: {hits:#?}"
    );
}

#[test]
fn good_pack_passes_every_check() {
    let f = assess_pack(&good_pack(), &good_load());
    for x in &f {
        assert_eq!(x.outcome, Outcome::Pass, "{x:#?}");
        assert!(x.value.is_finite() && x.limit.is_finite(), "{x:#?}");
        assert!(!x.detail.is_empty());
    }
    for k in ALL_CHECKS {
        assert!(f.iter().any(|x| x.kind == k), "нет проверки {k:?}");
    }
    assert_eq!(overall(&f), Outcome::Pass);
}

#[test]
fn no_fuse_fails() {
    let mut p = good_pack();
    p.fuse = None;
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::Protection, "нет плавкого предохранителя");
    assert_fail(&f, BatteryCheck::ShortCircuit, "предохранителя нет");
}

#[test]
fn no_bms_fails() {
    let mut p = good_pack();
    p.bms = None;
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::Protection, "нет BMS");
    assert_fail(&f, BatteryCheck::VoltageRange, "нет BMS");
}

#[test]
fn thin_wire_fails() {
    let mut p = good_pack();
    p.wire.conductor = Conductor::Awg(30);
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::WireHeating, "перегрузка");
}

#[test]
fn wire_destroyed_before_fuse_opens_fails() {
    let mut p = good_pack();
    p.wire.conductor = Conductor::Awg(26);
    p.wire.insulation = Insulation::Pvc;
    p.fuse = FuseSpec::ato(40);
    let f = assess_pack(&p, &good_load());
    assert_fail(
        &f,
        BatteryCheck::WireHeating,
        "раньше, чем сработает защита",
    );
}

#[test]
fn fuse_with_low_voltage_rating_fails() {
    let mut p = good_pack();
    if let Some(x) = p.fuse.as_mut() {
        x.voltage_dc_v = Some(12.0);
    }
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::Protection, "номинал 12 В DC");
}

#[test]
fn ac_only_fuse_fails() {
    let mut p = good_pack();
    if let Some(x) = p.fuse.as_mut() {
        x.voltage_dc_v = None;
    }
    let f = assess_pack(&p, &good_load());
    assert_fail(
        &f,
        BatteryCheck::Protection,
        "нет номинала по постоянному току",
    );
}

#[test]
fn pack_above_fuse_voltage_fails() {
    let mut p = good_pack();
    p.series = 8;
    let mut l = good_load();
    l.load_v_max = Some(40.0);
    l.load_v_min = Some(20.0);
    if let Some(c) = l.charge.as_mut() {
        c.voltage_v = 33.6;
    }
    let f = assess_pack(&p, &l);
    assert_fail(&f, BatteryCheck::Protection, "номинал 32 В DC");
}

#[test]
fn fault_above_breaking_capacity_fails() {
    let mut p = good_pack();
    p.cell = CellSpec::molicel_p42a();
    p.series = 7;
    p.parallel = 8;
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::ShortCircuit, "отключающая способность");
}

#[test]
fn fuse_down_the_wire_leaves_terminals_unprotected() {
    let mut p = good_pack();
    if let Some(x) = p.fuse.as_mut() {
        x.position_m = 0.2;
    }
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::ShortCircuit, "мимо предохранителя");
}

#[test]
fn fuse_too_big_for_weak_fault_current_fails() {
    let mut p = good_pack();
    p.fuse = FuseSpec::ato(40);
    p.contact_r_ohm = 0.3;
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::Protection, "гарантированное срабатывание");
}

#[test]
fn charging_in_frost_fails() {
    let mut l = good_load();
    if let Some(c) = l.charge.as_mut() {
        c.ambient_min_c = -5.0;
    }
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::ChargeWindow, "металлический литий");
}

#[test]
fn every_catalog_cell_refuses_charge_below_zero() {
    for cell in catalog() {
        let mut p = good_pack();
        p.cell = cell;
        let mut l = good_load();
        if let Some(c) = l.charge.as_mut() {
            c.ambient_min_c = -1.0;
        }
        let f = assess_pack(&p, &l);
        assert!(
            !failed(&f, BatteryCheck::ChargeWindow).is_empty(),
            "{}",
            p.cell.name
        );
    }
}

#[test]
fn charge_current_step_inside_range_is_seen() {
    // диапазон 20…40 °C пересекает ступень 1.455 А: 3 А на ячейку разрешены только выше 25 °C
    let mut l = good_load();
    if let Some(c) = l.charge.as_mut() {
        c.current_a = 5.5;
        c.ambient_min_c = 26.0;
        c.ambient_max_c = 40.0;
    }
    let ok = assess_pack(&good_pack(), &l);
    assert!(
        failed(&ok, BatteryCheck::ChargeWindow)
            .iter()
            .all(|x| !x.detail.contains("ток заряда")),
        "{ok:#?}"
    );
    if let Some(c) = l.charge.as_mut() {
        c.ambient_min_c = 20.0;
    }
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::ChargeWindow, "ток заряда");
}

#[test]
fn self_heating_during_charge_fails() {
    // воздух 40 °C в окне заряда, ток в пределе, но закрытый корпус греет ячейки выше 45 °C
    let mut l = good_load();
    if let Some(c) = l.charge.as_mut() {
        c.current_a = 5.5;
        c.ambient_min_c = 30.0;
        c.ambient_max_c = 40.0;
    }
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::ChargeWindow, "при заряде");
}

#[test]
fn fault_between_bms_thresholds_is_not_a_pass() {
    // наименьший ток КЗ попадает между порогами: ключ держит его до задержки перегрузки,
    // а область безопасной работы ключа не задана
    let mut p = good_pack();
    p.contact_r_ohm = 0.3;
    let f = assess_pack(&p, &good_load());
    assert!(
        f.iter().any(|x| x.kind == BatteryCheck::Protection
            && x.outcome == Outcome::NotAnalyzed
            && x.detail.contains("область безопасной работы")),
        "{f:#?}"
    );
    assert_eq!(overall(&f), Outcome::NotAnalyzed, "{f:#?}");
}

#[test]
fn overloaded_cell_fails() {
    let mut l = good_load();
    l.steps = vec![LoadStep {
        current_a: 40.0,
        duration_s: 600.0,
    }];
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::CellCurrent, "длительн");
}

#[test]
fn cold_derating_of_m50_is_applied() {
    let mut l = good_load();
    l.ambient_min_c = 5.0;
    l.steps = vec![LoadStep {
        current_a: 6.0,
        duration_s: 600.0,
    }];
    // 3.3 А на худшую ячейку: при 25 °C допустимо, ниже 10 °C предел 2.425 А
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::CellCurrent, "2.42");
}

#[test]
fn long_step_below_peak_cannot_hide_behind_short_pulse() {
    let mut p = good_pack();
    p.cell = CellSpec::a123_anr26650();
    p.series = 1;
    p.parallel = 1;
    p.fuse = FuseSpec::ato(40);
    let mut l = good_load();
    l.charge = None;
    l.load_v_min = None;
    l.steps = vec![
        LoadStep {
            current_a: 100.0,
            duration_s: 1.0,
        },
        LoadStep {
            current_a: 60.0,
            duration_s: 60.0,
        },
    ];
    let f = assess_pack(&p, &l);
    assert_fail(&f, BatteryCheck::CellCurrent, "разрешён только");
}

#[test]
fn sealed_hot_enclosure_fails_temperature_and_runaway() {
    let mut p = good_pack();
    if let Some(e) = p.enclosure.as_mut() {
        e.outer_area_m2 = 0.015;
        e.h_inner_w_m2k = 2.0;
        e.body_side_fraction = 0.8;
    }
    let mut l = good_load();
    l.ambient_max_c = 40.0;
    l.steps = vec![LoadStep {
        current_a: 7.0,
        duration_s: 3600.0,
    }];
    let f = assess_pack(&p, &l);
    assert_fail(&f, BatteryCheck::CellTemperature, "верхняя граница");
    assert_fail(&f, BatteryCheck::ThermalRunaway, "саморазогрева");
}

#[test]
fn hot_surface_on_skin_fails() {
    let mut p = good_pack();
    if let Some(s) = p.skin.as_mut() {
        s.spacer_thickness_m = 0.0;
    }
    let mut l = good_load();
    l.steps[0].current_a = 4.5;
    let f = assess_pack(&p, &l);
    assert_fail(&f, BatteryCheck::SkinContact, "порог ожога");
}

#[test]
fn large_skin_area_is_not_passed() {
    let mut p = good_pack();
    if let Some(s) = p.skin.as_mut() {
        s.contact_body_fraction = 0.15;
    }
    let f = assess_pack(&p, &good_load());
    assert!(
        f.iter()
            .any(|x| x.kind == BatteryCheck::SkinContact && x.outcome == Outcome::NotAnalyzed)
    );
    assert_ne!(overall(&f), Outcome::Pass);
}

#[test]
fn charger_overvoltage_fails() {
    let mut l = good_load();
    if let Some(c) = l.charge.as_mut() {
        c.voltage_v = 17.6;
    }
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::VoltageRange, "зарядное устройство");
}

#[test]
fn bms_overvoltage_threshold_too_high_fails() {
    let mut p = good_pack();
    if let Some(b) = p.bms.as_mut() {
        b.cell_ovp_v = Some(4.35);
    }
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::VoltageRange, "порог перезаряда");
}

#[test]
fn pack_above_load_limit_fails() {
    let mut l = good_load();
    l.load_v_max = Some(15.0);
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::VoltageRange, "нагрузка допускает");
}

#[test]
fn group_imbalance_fails() {
    let mut p = good_pack();
    p.group_voltage_spread_v = 0.3;
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::VoltageRange, "разбалансе");
}

#[test]
fn weak_mosfet_fails() {
    let mut p = good_pack();
    if let Some(b) = p.bms.as_mut() {
        b.mosfet.id_pulse_a = 100.0;
    }
    let f = assess_pack(&p, &good_load());
    assert_fail(&f, BatteryCheck::Protection, "импульсный предел");
}

#[test]
fn bms_that_trips_on_normal_load_fails() {
    let mut l = good_load();
    l.steps[1].current_a = 9.0;
    let f = assess_pack(&good_pack(), &l);
    assert_fail(&f, BatteryCheck::Protection, "вызовет отключение");
}

#[test]
fn unknown_avalanche_energy_is_not_a_pass() {
    let mut p = good_pack();
    if let Some(b) = p.bms.as_mut() {
        b.mosfet.avalanche_energy_j = None;
    }
    let f = assess_pack(&p, &good_load());
    assert_eq!(overall(&f), Outcome::NotAnalyzed, "{f:#?}");
}

#[test]
fn missing_enclosure_passes_only_under_adiabatic_bound() {
    let mut p = good_pack();
    p.enclosure = None;
    // за 8 часов без теплоотвода предел превышен: вывода нет, но и «годно» нет
    let f = assess_pack(&p, &good_load());
    assert!(
        f.iter()
            .any(|x| x.kind == BatteryCheck::CellTemperature && x.outcome == Outcome::NotAnalyzed)
    );
    assert_ne!(overall(&f), Outcome::Pass);
    // короткий профиль не успевает нагреть даже без теплоотвода
    let mut l = good_load();
    l.cycles = 1;
    l.steps = vec![LoadStep {
        current_a: 3.0,
        duration_s: 60.0,
    }];
    let g = assess_pack(&p, &l);
    assert!(
        g.iter()
            .filter(|x| x.kind == BatteryCheck::CellTemperature)
            .all(|x| x.outcome == Outcome::Pass),
        "{g:#?}"
    );
}

#[test]
fn empty_lipo_form_is_never_a_pass() {
    let mut p = good_pack();
    p.cell = CellSpec::custom_lipo(&LipoInput::default());
    let f = assess_pack(&p, &good_load());
    assert_ne!(overall(&f), Outcome::Pass);
    assert!(
        f.iter()
            .filter(|x| x.kind == BatteryCheck::ShortCircuit)
            .all(|x| x.outcome == Outcome::NotAnalyzed),
        "{f:#?}"
    );
    for x in &f {
        assert!(x.value.is_finite() && x.limit.is_finite(), "{x:#?}");
    }
}

#[test]
fn filled_lipo_form_is_computed() {
    let input = LipoInput {
        name: "LiPo из формы".into(),
        chemistry: Chemistry::Lco,
        capacity_ah: Some(5.0),
        v_nominal: Some(3.7),
        v_charge_max: Some(4.2),
        v_cutoff: Some(3.0),
        r_dc_ohm: Some(0.02),
        i_cont_a: Some(10.0),
        i_pulse_a: Some(20.0),
        pulse_s: Some(5.0),
        i_charge_a: Some(2.5),
        t_charge_min_c: Some(0.0),
        t_charge_max_c: Some(45.0),
        t_discharge_min_c: Some(-10.0),
        t_discharge_max_c: Some(60.0),
        mass_g: Some(100.0),
        length_mm: Some(100.0),
        width_mm: Some(50.0),
        thickness_mm: Some(10.0),
    };
    let mut p = good_pack();
    p.cell = CellSpec::custom_lipo(&input);
    p.parallel = 1;
    let f = assess_pack(&p, &good_load());
    assert!(
        f.iter()
            .filter(|x| x.kind == BatteryCheck::ShortCircuit)
            .all(|x| x.outcome != Outcome::NotAnalyzed),
        "{f:#?}"
    );
    let at = short_circuit(&p, &good_load(), FaultPoint::Terminals).expect("данные есть");
    // нижняя оценка 0.6·20 мОм на 4 ячейки; внешние сопротивления в наибольший ток не входят
    let hand = 16.8 / (4.0 * 0.012);
    assert!((at.i_max_a - hand).abs() < 1e-9, "{}", at.i_max_a);
}

type Mutator = (&'static str, fn(&mut PackDesign, &mut LoadProfile, f64));

fn mutators() -> Vec<Mutator> {
    vec![
        ("ёмкость", |p, _, x| {
            p.cell.capacity_typ_ah.value = Some(x)
        }),
        ("v_nom", |p, _, x| p.cell.v_nominal.value = Some(x)),
        ("v_max", |p, _, x| p.cell.v_charge_max.value = Some(x)),
        ("v_cut", |p, _, x| p.cell.v_cutoff.value = Some(x)),
        ("r_low", |p, _, x| p.cell.r_low_ohm.value = Some(x)),
        ("r_high", |p, _, x| p.cell.r_dc_high_ohm.value = Some(x)),
        ("i_cont", |p, _, x| {
            p.cell.i_discharge_cont_a.value = Some(x)
        }),
        ("t_chg", |p, _, x| p.cell.t_charge_min_c.value = Some(x)),
        ("t_dis", |p, _, x| p.cell.t_discharge_max_c.value = Some(x)),
        ("масса", |p, _, x| p.cell.mass_kg.value = Some(x)),
        ("cp", |p, _, x| p.cell.cp_j_kg_k.value = Some(x)),
        ("ea", |p, _, x| p.cell.ea_r_j_mol.value = Some(x)),
        ("предел заряда", |p, _, x| {
            p.cell.charge_limits[0].current_a = x
        }),
        ("разброс", |p, _, x| p.cell_resistance_spread = x),
        ("разбаланс", |p, _, x| p.group_voltage_spread_v = x),
        ("перемычки", |p, _, x| p.interconnect_r_ohm = x),
        ("контакты", |p, _, x| p.contact_r_ohm = x),
        ("сечение", |p, _, x| {
            p.wire.conductor = Conductor::Mm2(x)
        }),
        ("длина", |p, _, x| p.wire.length_m = x),
        ("диаметр", |p, _, x| {
            p.wire.outer_diameter_mm = Some(x)
        }),
        ("номинал", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.rating_a = x;
            }
        }),
        ("напряжение пр.", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.voltage_dc_v = Some(x);
            }
        }),
        ("отключение", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.breaking_a = Some(x);
            }
        }),
        ("I²t", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.melt_i2t_a2s = Some(x);
            }
        }),
        ("R пр.", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.cold_resistance_ohm = Some(x);
            }
        }),
        ("положение", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.position_m = x;
            }
        }),
        ("характеристика", |p, _, x| {
            if let Some(f) = p.fuse.as_mut() {
                f.tcc[1].t_max_s = Some(x);
            }
        }),
        ("порог КЗ", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.scd_trip_max_a = x;
            }
        }),
        ("задержка КЗ", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.scd_delay_max_s = x;
            }
        }),
        ("порог OCD", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.ocd_trip_max_a = x;
            }
        }),
        ("OVP", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.cell_ovp_v = Some(x);
            }
        }),
        ("Vds", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.mosfet.vds_max_v = x;
            }
        }),
        ("Idm", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.mosfet.id_pulse_a = x;
            }
        }),
        ("Rds", |p, _, x| {
            if let Some(b) = p.bms.as_mut() {
                b.mosfet.rds_on_ohm = x;
            }
        }),
        ("площадь", |p, _, x| {
            if let Some(e) = p.enclosure.as_mut() {
                e.outer_area_m2 = x;
            }
        }),
        ("стенка k", |p, _, x| {
            if let Some(e) = p.enclosure.as_mut() {
                e.wall_k_w_mk = x;
            }
        }),
        ("h внутри", |p, _, x| {
            if let Some(e) = p.enclosure.as_mut() {
                e.h_inner_w_m2k = x;
            }
        }),
        ("доля у тела", |p, _, x| {
            if let Some(e) = p.enclosure.as_mut() {
                e.body_side_fraction = x;
            }
        }),
        ("прокладка", |p, _, x| {
            if let Some(s) = p.skin.as_mut() {
                s.spacer_thickness_m = x;
            }
        }),
        ("прокладка k", |p, _, x| {
            if let Some(s) = p.skin.as_mut() {
                s.spacer_k_w_mk = x;
            }
        }),
        ("время контакта", |p, _, x| {
            if let Some(s) = p.skin.as_mut() {
                s.contact_time_s = x;
            }
        }),
        ("доля тела", |p, _, x| {
            if let Some(s) = p.skin.as_mut() {
                s.contact_body_fraction = x;
            }
        }),
        ("ток шага", |_, l, x| l.steps[0].current_a = x),
        ("длительность", |_, l, x| {
            l.steps[1].duration_s = x
        }),
        ("среда мин", |_, l, x| l.ambient_min_c = x),
        ("среда макс", |_, l, x| l.ambient_max_c = x),
        ("нагрузка мин", |_, l, x| l.load_v_min = Some(x)),
        ("нагрузка макс", |_, l, x| {
            l.load_v_max = Some(x)
        }),
        ("ток заряда", |_, l, x| {
            if let Some(c) = l.charge.as_mut() {
                c.current_a = x;
            }
        }),
        ("напряжение заряда", |_, l, x| {
            if let Some(c) = l.charge.as_mut() {
                c.voltage_v = x;
            }
        }),
        ("заряд мин", |_, l, x| {
            if let Some(c) = l.charge.as_mut() {
                c.ambient_min_c = x;
            }
        }),
        ("заряд макс", |_, l, x| {
            if let Some(c) = l.charge.as_mut() {
                c.ambient_max_c = x;
            }
        }),
    ]
}

#[test]
fn nan_and_infinity_in_any_input_is_fail_for_all_checks() {
    for (name, m) in mutators() {
        for x in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let (mut p, mut l) = (good_pack(), good_load());
            m(&mut p, &mut l, x);
            let f = assess_pack(&p, &l);
            assert_eq!(f.len(), ALL_CHECKS.len(), "{name} = {x}");
            assert!(
                f.iter().all(|y| y.outcome == Outcome::Fail),
                "{name} = {x}: {f:#?}"
            );
            assert_eq!(overall(&f), Outcome::Fail, "{name} = {x}");
        }
    }
}

#[test]
fn zero_series_or_parallel_is_fail() {
    for (s, pp) in [(0, 2), (4, 0), (2000, 1), (1, 2000)] {
        let mut p = good_pack();
        p.series = s;
        p.parallel = pp;
        let f = assess_pack(&p, &good_load());
        assert!(f.iter().all(|y| y.outcome == Outcome::Fail), "{s}S{pp}P");
    }
}

#[test]
fn overall_never_passes_on_empty_or_partial() {
    assert_eq!(overall(&[]), Outcome::NotAnalyzed);
    let mut f = assess_pack(&good_pack(), &good_load());
    f[0].outcome = Outcome::NotAnalyzed;
    assert_eq!(overall(&f), Outcome::NotAnalyzed);
    f[1].outcome = Outcome::Fail;
    assert_eq!(overall(&f), Outcome::Fail);
}

/// Простой воспроизводимый генератор: случайные сборки не должны ронять оценку, а всякий
/// `Pass` обязан подтверждаться независимым пересчётом ключевых неравенств.
#[test]
fn random_designs_never_get_a_false_pass() {
    let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let cells = catalog();
    let mut passes = 0;
    for _ in 0..3000 {
        let mut p = good_pack();
        let mut l = good_load();
        p.cell = cells[(rnd() * cells.len() as f64) as usize % cells.len()].clone();
        p.series = 1 + (rnd() * 7.0) as u32;
        p.parallel = 1 + (rnd() * 4.0) as u32;
        p.wire.conductor = Conductor::Awg(10 + (rnd() * 20.0) as u8);
        p.wire.length_m = 0.1 + rnd() * 2.0;
        p.fuse = if rnd() < 0.1 {
            None
        } else {
            FuseSpec::ato([10, 20, 30, 40][(rnd() * 4.0) as usize % 4])
        };
        if rnd() < 0.1 {
            p.bms = None;
        }
        l.steps[0].current_a = rnd() * 12.0;
        l.steps[1].current_a = rnd() * 12.0;
        l.ambient_min_c = -25.0 + rnd() * 40.0;
        l.ambient_max_c = l.ambient_min_c + rnd() * 30.0;
        l.load_v_min = None;
        l.load_v_max = Some(40.0);
        let v_full = p.cell.v_charge_max.value.expect("есть") * p.series as f64;
        if let Some(c) = l.charge.as_mut() {
            c.voltage_v = v_full;
        }
        let f = assess_pack(&p, &l);
        for x in &f {
            if x.outcome == Outcome::Pass {
                assert!(x.value.is_finite() && x.limit.is_finite(), "{x:#?}");
            }
        }
        if overall(&f) != Outcome::Pass {
            continue;
        }
        passes += 1;
        let fuse = p.fuse.as_ref().expect("Pass без предохранителя");
        assert!(p.bms.is_some(), "Pass без BMS");
        assert!(v_full <= fuse.voltage_dc_v.expect("номинал"));
        let open = fuse.guaranteed_open_a().expect("характеристика");
        for x in fault_table(&p, &l) {
            assert!(x.fuse_in_path);
            assert!(x.i_max_a <= fuse.breaking_a.expect("отключение"));
            assert!(x.i_min_a >= open, "{x:?}");
            let t = fuse.max_clear_time_s(x.i_max_a).expect("сработает");
            let theta = p
                .wire
                .temperature_after_c(x.i_max_a, t, l.ambient_max_c, l.ambient_max_c);
            if x.wire_loop_m > 0.0 {
                assert!(theta <= p.wire.insulation.short_circuit_c(), "{theta}");
            }
        }
        assert!(p.wire.ampacity_a(l.ambient_max_c) >= open);
    }
    assert!(
        passes > 0,
        "ни одна случайная сборка не прошла: тест пустой"
    );
}

#[test]
fn catalog_m50_matches_circuit_model() {
    let sim = CellParams::lg_m50();
    let c = CellSpec::lg_m50();
    let v = |p: Param| p.value.expect("параметр M50");
    assert_eq!(v(c.capacity_typ_ah), sim.capacity_ah);
    assert_eq!(v(c.v_nominal), sim.nominal_v);
    assert_eq!(v(c.v_charge_max), sim.v_max);
    assert_eq!(v(c.v_cutoff), sim.v_min);
    assert_eq!(v(c.mass_kg), sim.mass_kg);
    assert_eq!(v(c.t_charge_min_c), sim.charge_temp_c.0);
    assert_eq!(v(c.t_charge_max_c), sim.charge_temp_c.1);
    assert_eq!(v(c.t_discharge_min_c), sim.discharge_temp_c.0);
    assert_eq!(v(c.t_discharge_max_c), sim.discharge_temp_c.1);
    assert_eq!(v(c.ea_r_j_mol), sim.ea_r0);
    assert!((v(c.mass_kg) * v(c.cp_j_kg_k) - sim.heat_capacity_j_k).abs() < 1e-9);
    assert_eq!(v(c.discharge_cold_below_c), sim.max_discharge_a[0].0);
    assert_eq!(v(c.i_discharge_cold_a), sim.max_discharge_a[0].1);
    assert_eq!(v(c.i_discharge_cont_a), sim.max_discharge_a[1].1);
    assert_eq!(c.charge_limits.len(), sim.max_charge_a.len());
    for (a, b) in c.charge_limits.iter().zip(&sim.max_charge_a) {
        assert_eq!((a.up_to_c, a.current_a), *b);
    }
    // нижняя оценка сопротивления не выше R0 модели на всём SOC, верхняя с концом службы — выше
    for k in 0..=100 {
        let z = k as f64 / 100.0;
        let r0 = sim.r0_at(z, 25.0);
        assert!(v(c.r_low_ohm) <= r0, "SOC {z}: {r0}");
        assert!(
            v(c.r_dc_high_ohm) * 2.0 >= r0 + sim.rc_r_at(0, z, 25.0),
            "SOC {z}"
        );
    }
}

#[test]
fn ampacity_model_never_exceeds_nec_free_air_table() {
    // NEC табл. 310.15(B)(17), медь, свободный воздух 30 °C, колонка 90 °C
    for (awg, nec) in [(18u8, 18.0), (16, 24.0), (14, 35.0), (12, 40.0), (10, 55.0)] {
        let w = WireRun {
            conductor: Conductor::Awg(awg),
            insulation: Insulation::Silicone,
            length_m: 1.0,
            cooling: WireCooling::FreeAir,
            outer_diameter_mm: None,
        };
        let model = (w.cooling_w_per_m_k(30.0) * 60.0 / w.resistance_ohm_per_m(90.0)).sqrt();
        assert!(model < nec, "AWG {awg}: модель {model:.1} А, NEC {nec} А");
        assert!(
            model > 0.5 * nec,
            "AWG {awg}: модель {model:.1} А, NEC {nec} А"
        );
    }
}

#[test]
fn reference_checks_all_pass_and_ids_are_unique() {
    let checks = reference_checks();
    assert!(checks.len() >= 25, "сверок {}", checks.len());
    let mut ids: Vec<&str> = checks.iter().map(|c| c.id).collect();
    ids.sort_unstable();
    let n = ids.len();
    ids.dedup();
    assert_eq!(ids.len(), n, "повторяются идентификаторы сверок");
    for c in &checks {
        assert!(c.id.starts_with("battery."), "{}", c.id);
        assert!(c.simulated.is_finite() && c.reference.is_finite(), "{c}");
        c.assert_pass();
    }
}
