//! Негативные и сквозные тесты единой оценки детали. Главное, что здесь проверяется: заведомо
//! опасная деталь получает `Fail` **по своей причине**, а та же деталь без дефекта — проходит
//! (иначе тест прошёл бы вхолостую на детали, которая падает всегда).

use sim_materials::catalog::MM;
use sim_materials::fatigue::Surface;
use sim_materials::*;

fn get(f: &[PartFinding], kind: FindingKind) -> &PartFinding {
    f.iter()
        .find(|x| x.kind == kind)
        .unwrap_or_else(|| panic!("нет записи {kind:?}"))
}

fn outcome(f: &[PartFinding], kind: FindingKind) -> Outcome {
    get(f, kind).outcome
}

fn geometry() -> PrintGeometry {
    PrintGeometry {
        bbox: [0.06, 0.02, 0.02],
        build_volume: [0.256, 0.256, 0.256],
        min_wall: 2.0 * MM,
        max_overhang_deg: 30.0,
        max_bridge: 5.0 * MM,
        min_hole: 0.0,
        supports: false,
        enclosed: true,
    }
}

fn print_spec(angle: f64) -> PrintSpec {
    PrintSpec {
        process: PrintProcess {
            infill: 1.0,
            load_angle_deg: angle,
            ..PrintProcess::default()
        },
        geometry: geometry(),
    }
}

fn printed(material: MaterialId, angle: f64) -> PartSpec {
    let mut p = PartSpec::new(
        "печатная деталь",
        material,
        Section::Rect { b: 0.02, h: 0.01 },
        0.06,
    );
    p.print = Some(print_spec(angle));
    p.fastening = Fastening::NoFasteners;
    p
}

fn metal(material: MaterialId) -> PartSpec {
    let mut p = PartSpec::new(
        "металлическая деталь",
        material,
        Section::Rect { b: 0.03, h: 0.006 },
        0.1,
    );
    p.fastening = Fastening::NoFasteners;
    p
}

fn room(load: LoadCase) -> LoadCase {
    LoadCase {
        temperature_c: 23.0,
        ..load
    }
}

// --- Положительный контроль ----------------------------------------------------------------

#[test]
fn sound_metal_part_passes_everything() {
    let p = metal(MaterialId::Al6061T6);
    let l = room(LoadCase {
        axial: 500.0,
        bending: 1.0,
        cycles: 1e6,
        load_ratio: 0.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    assert_eq!(overall(&f), Outcome::Pass, "{f:#?}");
}

#[test]
fn every_report_has_every_check() {
    for id in MaterialId::ALL {
        let mut p = PartSpec::new("x", id, Section::Round { d: 0.01 }, 0.1);
        if id.material().is_printed() {
            p.print = Some(print_spec(90.0));
        }
        for l in [
            LoadCase::default(),
            room(LoadCase {
                axial: -100.0,
                cycles: 1e5,
                ..LoadCase::default()
            }),
        ] {
            let f = assess_part(&p, &l);
            for kind in FindingKind::CHECKS {
                assert_eq!(
                    f.iter().filter(|x| x.kind == kind).count(),
                    1,
                    "{id:?} {kind:?}"
                );
            }
        }
    }
}

#[test]
fn all_reference_checks_pass() {
    let checks = reference_checks();
    assert!(checks.len() >= 40, "сверок всего {}", checks.len());
    println!("сверок: {}", checks.len());
    for c in &checks {
        println!(
            "{:<44} {:>12.5} / {:>12.5}  ошибка {}",
            c.id,
            c.simulated,
            c.reference,
            c.error_text()
        );
    }
    let mut ids: Vec<_> = checks.iter().map(|c| c.id).collect();
    ids.sort_unstable();
    let n = ids.len();
    ids.dedup();
    assert_eq!(ids.len(), n, "повторяющиеся id");
    for c in &checks {
        c.assert_pass();
    }
}

// --- Статика и слои ------------------------------------------------------------------------

#[test]
fn pla_across_layers_fails_where_along_passes() {
    // Растяжение 2 кН на 200 мм² = 10 МПа: вдоль слоёв запас 3.1, поперёк 1.96.
    let l = room(LoadCase {
        axial: 2000.0,
        ..LoadCase::default()
    });
    let along = assess_part(&printed(MaterialId::Pla, 0.0), &l);
    let across = assess_part(&printed(MaterialId::Pla, 90.0), &l);
    assert_eq!(
        outcome(&along, FindingKind::Static),
        Outcome::Pass,
        "{along:#?}"
    );
    assert_eq!(
        outcome(&across, FindingKind::Static),
        Outcome::Fail,
        "{across:#?}"
    );
    assert!(get(&across, FindingKind::Static).value < get(&along, FindingKind::Static).value);
}

#[test]
fn every_printed_material_is_weaker_across_layers() {
    let l = room(LoadCase {
        axial: 100.0,
        ..LoadCase::default()
    });
    for id in MaterialId::ALL {
        if !id.material().is_printed() {
            continue;
        }
        let a = get(&assess_part(&printed(id, 0.0), &l), FindingKind::Static).value;
        let c = get(&assess_part(&printed(id, 90.0), &l), FindingKind::Static).value;
        assert!(c <= 0.75 * a, "{id:?}: вдоль {a:.1}, поперёк {c:.1}");
    }
}

#[test]
fn low_infill_part_is_weaker() {
    let l = room(LoadCase {
        bending: 2.0,
        ..LoadCase::default()
    });
    let solid = printed(MaterialId::Petg, 0.0);
    let mut sparse = printed(MaterialId::Petg, 0.0);
    sparse.section = Section::Rect { b: 0.02, h: 0.01 };
    sparse.print.as_mut().unwrap().process.infill = 0.15;
    sparse.print.as_mut().unwrap().process.walls = 2;
    let a = get(&assess_part(&solid, &l), FindingKind::Static).value;
    let b = get(&assess_part(&sparse, &l), FindingKind::Static).value;
    assert!(b < a, "сплошная {a}, редкая {b}");
}

#[test]
fn printed_without_settings_is_not_a_pass() {
    let mut p = printed(MaterialId::Abs, 0.0);
    p.print = None;
    let l = room(LoadCase {
        axial: 10.0,
        cycles: 100.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::NotAnalyzed);
    assert_eq!(outcome(&f, FindingKind::Printability), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
}

#[test]
fn stress_raiser_counts_statically_for_printed() {
    let l = room(LoadCase {
        axial: 500.0,
        ..LoadCase::default()
    });
    let plain = printed(MaterialId::Pla, 0.0);
    let mut holed = printed(MaterialId::Pla, 0.0);
    holed.raiser = StressRaiser::Hole {
        d: 4.0 * MM,
        orientation: HoleOrientation::OutOfPlane,
        centered: true,
    };
    let a = get(&assess_part(&plain, &l), FindingKind::Static).value;
    let b = get(&assess_part(&holed, &l), FindingKind::Static).value;
    assert!(b < a / 2.0, "без отверстия {a}, с отверстием {b}");
}

#[test]
fn composite_compression_weaker_than_tension() {
    let mut p = PartSpec::new(
        "труба",
        MaterialId::CfrpTube,
        Section::Tube {
            od: 0.02,
            wall: 0.002,
        },
        0.05,
    );
    p.fastening = Fastening::NoFasteners;
    let t = room(LoadCase {
        axial: 1000.0,
        ..LoadCase::default()
    });
    let c = room(LoadCase {
        axial: -1000.0,
        ..LoadCase::default()
    });
    let nt = get(&assess_part(&p, &t), FindingKind::Static).value;
    let nc = get(&assess_part(&p, &c), FindingKind::Static).value;
    assert!((nt / nc - 2.0).abs() < 0.05, "растяжение {nt}, сжатие {nc}");
}

// --- Температура ---------------------------------------------------------------------------

#[test]
fn part_near_softening_fails() {
    let p = printed(MaterialId::Pla, 0.0);
    let l = LoadCase {
        axial: 200.0,
        temperature_c: 50.0,
        ..LoadCase::default()
    };
    let f = assess_part(&p, &l);
    assert_eq!(
        outcome(&f, FindingKind::Temperature),
        Outcome::Fail,
        "{f:#?}"
    );
    // Та же деталь при комнатной температуре — без отказа по температуре и статике.
    let ok = assess_part(&p, &room(l.clone()));
    assert_eq!(outcome(&ok, FindingKind::Temperature), Outcome::Pass);
    assert_eq!(outcome(&ok, FindingKind::Static), Outcome::Pass);
}

#[test]
fn at_softening_part_carries_nothing() {
    let p = printed(MaterialId::Pla, 0.0);
    let l = LoadCase {
        axial: 1.0,
        cycles: 10.0,
        temperature_c: 54.0,
        ..LoadCase::default()
    };
    let f = assess_part(&p, &l);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::Fail);
    assert_eq!(outcome(&f, FindingKind::Fatigue), Outcome::Fail);
    assert_eq!(get(&f, FindingKind::Static).value, 0.0);
}

#[test]
fn cold_printed_part_not_analyzed() {
    let p = printed(MaterialId::Abs, 0.0);
    let l = LoadCase {
        axial: 10.0,
        temperature_c: -10.0,
        ..LoadCase::default()
    };
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Temperature),
        Outcome::NotAnalyzed
    );
}

// --- Усталость -----------------------------------------------------------------------------

#[test]
fn hole_without_fatigue_margin_fails() {
    // Одна и та же сила: гладкая полоса проходит, полоса с отверстием — нет.
    let l = room(LoadCase {
        axial: 3200.0,
        cycles: 1e7,
        load_ratio: -1.0,
        ..LoadCase::default()
    });
    let mut plain = metal(MaterialId::SteelS235);
    plain.section = Section::Rect { b: 0.03, h: 0.005 };
    let mut holed = plain.clone();
    holed.raiser = StressRaiser::Hole {
        d: 10.0 * MM,
        orientation: HoleOrientation::OutOfPlane,
        centered: true,
    };
    let a = assess_part(&plain, &l);
    let b = assess_part(&holed, &l);
    assert_eq!(outcome(&a, FindingKind::Fatigue), Outcome::Pass, "{a:#?}");
    assert_eq!(outcome(&b, FindingKind::Fatigue), Outcome::Fail, "{b:#?}");
    // Статика у обеих проходит: отказ именно усталостный.
    assert_eq!(outcome(&b, FindingKind::Static), Outcome::Pass, "{b:#?}");
}

#[test]
fn printed_fatigue_is_never_guessed() {
    let l = room(LoadCase {
        axial: 50.0,
        cycles: 1e5,
        ..LoadCase::default()
    });
    for id in MaterialId::ALL {
        if !id.material().is_printed() {
            continue;
        }
        let f = assess_part(&printed(id, 0.0), &l);
        assert_eq!(
            outcome(&f, FindingKind::Fatigue),
            Outcome::NotAnalyzed,
            "{id:?}"
        );
        assert_ne!(overall(&f), Outcome::Pass, "{id:?}");
    }
}

#[test]
fn printed_fatigue_overload_is_a_fail_not_a_shrug() {
    // Пик цикла выше прочности: данных не нужно, деталь сломается.
    let l = room(LoadCase {
        axial: 20_000.0,
        cycles: 100.0,
        ..LoadCase::default()
    });
    let f = assess_part(&printed(MaterialId::Pla, 90.0), &l);
    assert_eq!(outcome(&f, FindingKind::Fatigue), Outcome::Fail);
}

#[test]
fn measured_sn_can_prove_printed_fatigue_only_within_its_range() {
    let mut p = printed(MaterialId::Petg, 0.0);
    p.measured_sn = Some(MeasuredSn {
        a: 60e6,
        b: -0.08,
        max_cycles: 1e6,
    });
    let inside = room(LoadCase {
        axial: 100.0,
        cycles: 1e5,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &inside), FindingKind::Fatigue),
        Outcome::Pass
    );
    let beyond = LoadCase {
        cycles: 1e7,
        ..inside.clone()
    };
    assert_eq!(
        outcome(&assess_part(&p, &beyond), FindingKind::Fatigue),
        Outcome::NotAnalyzed
    );
    let heavy = room(LoadCase {
        axial: 5000.0,
        cycles: 1e5,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &heavy), FindingKind::Fatigue),
        Outcome::Fail
    );
}

#[test]
fn aluminium_beyond_data_base_not_analyzed() {
    let p = metal(MaterialId::Al6061T6);
    let inside = room(LoadCase {
        axial: 100.0,
        cycles: 1e8,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &inside), FindingKind::Fatigue),
        Outcome::Pass
    );
    let beyond = LoadCase {
        cycles: 1e9,
        ..inside
    };
    assert_eq!(
        outcome(&assess_part(&p, &beyond), FindingKind::Fatigue),
        Outcome::NotAnalyzed
    );
}

#[test]
fn composite_fatigue_not_guessed() {
    let mut p = PartSpec::new(
        "труба",
        MaterialId::CfrpTube,
        Section::Tube {
            od: 0.02,
            wall: 0.002,
        },
        0.05,
    );
    p.fastening = Fastening::NoFasteners;
    let l = room(LoadCase {
        axial: 10.0,
        cycles: 1e4,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Fatigue),
        Outcome::NotAnalyzed
    );
}

#[test]
fn spectrum_uses_miner_and_more_cycles_hurt() {
    let p = metal(MaterialId::Al6061T6);
    let base = |n: f64| {
        room(LoadCase {
            axial: 8000.0,
            load_ratio: -1.0,
            spectrum: vec![
                LoadBlock {
                    fraction: 1.0,
                    cycles: n,
                },
                LoadBlock {
                    fraction: 0.5,
                    cycles: 10.0 * n,
                },
            ],
            ..LoadCase::default()
        })
    };
    let few = get(&assess_part(&p, &base(1e3)), FindingKind::Fatigue).value;
    let many = get(&assess_part(&p, &base(1e6)), FindingKind::Fatigue).value;
    assert!(many < few, "{few} → {many}");
}

#[test]
fn surface_and_reliability_lower_margin() {
    let l = room(LoadCase {
        axial: 1000.0,
        cycles: 1e6,
        load_ratio: -1.0,
        ..LoadCase::default()
    });
    let ground = {
        let mut p = metal(MaterialId::SteelS235);
        p.surface = Surface::Ground;
        p.reliability = Reliability::R50;
        p
    };
    let rough = {
        let mut p = metal(MaterialId::SteelS235);
        p.surface = Surface::AsForged;
        p.reliability = Reliability::R9999;
        p
    };
    let a = get(&assess_part(&ground, &l), FindingKind::Fatigue).value;
    let b = get(&assess_part(&rough, &l), FindingKind::Fatigue).value;
    assert!(b < a, "{a} / {b}");
}

// --- Устойчивость --------------------------------------------------------------------------

#[test]
fn slender_rod_buckles_short_one_does_not() {
    let mut long = PartSpec::new(
        "стержень",
        MaterialId::Al6061T6,
        Section::Round { d: 0.006 },
        1.0,
    );
    long.fastening = Fastening::NoFasteners;
    let mut short = long.clone();
    short.length = 0.05;
    let l = room(LoadCase {
        axial: -500.0,
        ..LoadCase::default()
    });
    let fl = assess_part(&long, &l);
    let fs = assess_part(&short, &l);
    assert_eq!(
        outcome(&fl, FindingKind::Buckling),
        Outcome::Fail,
        "{fl:#?}"
    );
    assert_eq!(
        outcome(&fs, FindingKind::Buckling),
        Outcome::Pass,
        "{fs:#?}"
    );
    // Статика длинного стержня сама по себе проходит: ловит именно устойчивость.
    assert_eq!(outcome(&fl, FindingKind::Static), Outcome::Pass);
}

#[test]
fn thin_wall_tube_local_buckling_not_passed() {
    let mut p = PartSpec::new(
        "тонкая труба",
        MaterialId::Al6061T6,
        Section::Tube {
            od: 0.05,
            wall: 0.0005,
        },
        0.1,
    );
    p.fastening = Fastening::NoFasteners;
    let l = room(LoadCase {
        axial: -100.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Buckling),
        Outcome::NotAnalyzed
    );
}

#[test]
fn tall_thin_bar_buckles_sideways_in_bending() {
    // Полоса 2×40 мм на ребре, 1 м: по статике запас есть, но она ляжет набок.
    let mut p = PartSpec::new(
        "полоса на ребре",
        MaterialId::SteelS235,
        Section::Rect { b: 0.002, h: 0.04 },
        1.0,
    );
    p.fastening = Fastening::NoFasteners;
    let l = room(LoadCase {
        bending: 40.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::Pass, "{f:#?}");
    assert_eq!(outcome(&f, FindingKind::Buckling), Outcome::Fail, "{f:#?}");
    // Та же полоса плашмя боковой формы не имеет.
    let mut flat = p.clone();
    flat.section = Section::Rect { b: 0.04, h: 0.002 };
    let light = room(LoadCase {
        bending: 1.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&flat, &light), FindingKind::Buckling),
        Outcome::Pass
    );
}

#[test]
fn thin_tube_in_bending_or_torsion_is_not_a_pass() {
    let mut p = PartSpec::new(
        "тонкая труба",
        MaterialId::Al6061T6,
        Section::Tube {
            od: 0.05,
            wall: 0.001,
        }, // D/t = 50 > 0.07·E/Fy ≈ 20
        0.2,
    );
    p.fastening = Fastening::NoFasteners;
    for l in [
        room(LoadCase {
            bending: 1.0,
            ..LoadCase::default()
        }),
        room(LoadCase {
            torsion: 1.0,
            ..LoadCase::default()
        }),
    ] {
        assert_eq!(
            outcome(&assess_part(&p, &l), FindingKind::Buckling),
            Outcome::NotAnalyzed
        );
    }
    // Толстостенная проходит.
    p.section = Section::Tube {
        od: 0.02,
        wall: 0.003,
    };
    let l = room(LoadCase {
        bending: 1.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Buckling),
        Outcome::Pass
    );
}

#[test]
fn compression_plus_bending_interacts() {
    // Каждая нагрузка отдельно проходит, вместе — нет.
    let mut p = PartSpec::new(
        "полоса на ребре",
        MaterialId::SteelS235,
        Section::Rect { b: 0.004, h: 0.03 },
        0.6,
    );
    p.fastening = Fastening::NoFasteners;
    let (e, sy) = (210e9, 235e6);
    let pcr = sim_materials::section::critical_load(&p.section, e, sy, 0.6, p.end).critical_load;
    let mn = sim_materials::section::rect_lateral_buckling(0.03, 0.004, e, sy, 0.6).unwrap();
    let only_p = room(LoadCase {
        axial: -0.7 * pcr / 3.0,
        ..LoadCase::default()
    });
    let only_m = room(LoadCase {
        bending: 0.7 * mn / 3.0,
        ..LoadCase::default()
    });
    let both = room(LoadCase {
        axial: only_p.axial,
        bending: only_m.bending,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &only_p), FindingKind::Buckling),
        Outcome::Pass
    );
    assert_eq!(
        outcome(&assess_part(&p, &only_m), FindingKind::Buckling),
        Outcome::Pass
    );
    assert_eq!(
        outcome(&assess_part(&p, &both), FindingKind::Buckling),
        Outcome::Fail
    );
}

#[test]
fn s235_below_zero_not_a_pass() {
    let p = metal(MaterialId::SteelS235);
    let l = LoadCase {
        axial: 100.0,
        temperature_c: -10.0,
        ..LoadCase::default()
    };
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Temperature),
        Outcome::NotAnalyzed
    );
    // Алюминий хладноломкости не имеет.
    let al = metal(MaterialId::Al6061T6);
    assert_eq!(
        outcome(&assess_part(&al, &l), FindingKind::Temperature),
        Outcome::Pass
    );
}

#[test]
fn compression_amplifies_bending() {
    let mut p = PartSpec::new(
        "балка-колонна",
        MaterialId::Al6061T6,
        Section::Round { d: 0.01 },
        0.6,
    );
    p.fastening = Fastening::NoFasteners;
    let no_axial = room(LoadCase {
        bending: 2.0,
        ..LoadCase::default()
    });
    let with = room(LoadCase {
        bending: 2.0,
        axial: -1000.0,
        ..LoadCase::default()
    });
    let a = get(&assess_part(&p, &no_axial), FindingKind::Static).value;
    let b = get(&assess_part(&p, &with), FindingKind::Static).value;
    let pure_add = 1.0 / (1.0 / a + (1000.0 / Section::Round { d: 0.01 }.area()) / 240e6);
    assert!(b < pure_add, "усиления изгиба нет: {b} против {pure_add}");
}

#[test]
fn tpu_buckling_not_analyzed() {
    let p = printed(MaterialId::Tpu95a, 0.0);
    let l = room(LoadCase {
        axial: -5.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Buckling),
        Outcome::NotAnalyzed
    );
}

// --- Ползучесть ----------------------------------------------------------------------------

#[test]
fn sustained_load_above_working_level_fails() {
    let p = printed(MaterialId::Pla, 0.0);
    // 2 кН / 200 мм² = 10 МПа: статика проходит, а постоянно — выше 25% от 31 МПа.
    let l = room(LoadCase {
        axial: 2000.0,
        sustained_fraction: 1.0,
        sustained_hours: 100.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::Pass);
    assert_eq!(outcome(&f, FindingKind::Creep), Outcome::Fail, "{f:#?}");
    let short_light = LoadCase {
        sustained_fraction: 0.3,
        ..l
    };
    assert_eq!(
        outcome(&assess_part(&p, &short_light), FindingKind::Creep),
        Outcome::Pass
    );
}

#[test]
fn creep_beyond_data_is_not_a_pass() {
    let p = printed(MaterialId::Asa, 0.0);
    let l = room(LoadCase {
        axial: 100.0,
        sustained_fraction: 1.0,
        sustained_hours: 5000.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Creep),
        Outcome::NotAnalyzed
    );
}

#[test]
fn nylon_above_tg_at_body_temperature_not_a_pass() {
    let p = printed(MaterialId::Pa12, 0.0);
    let l = LoadCase {
        axial: 100.0,
        temperature_c: 37.0,
        sustained_fraction: 1.0,
        sustained_hours: 10.0,
        ..LoadCase::default()
    };
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Creep),
        Outcome::NotAnalyzed
    );
}

// --- Крепёж --------------------------------------------------------------------------------

fn bolted(engagement: ThreadEngagement) -> FastenerSpec {
    FastenerSpec {
        size: BoltSize::M3,
        grade: BoltGrade::G8_8,
        count: 1,
        engagement,
        plate_thickness: 6.0 * MM,
        edge_distance: 6.0 * MM,
    }
}

#[test]
fn thread_in_plastic_pulls_out_before_bolt_breaks() {
    let mut p = printed(MaterialId::Pla, 0.0);
    p.fastening = Fastening::Bolted(bolted(ThreadEngagement::Tapped { length: 4.0 * MM }));
    let l = room(LoadCase {
        axial: 10.0,
        fastener_tension: 150.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    let x = get(&f, FindingKind::Fasteners);
    assert_eq!(x.outcome, Outcome::Fail, "{x:#?}");
    assert!(x.detail.contains("вырыв резьбы"), "{}", x.detail);
    // Сквозной болт с той же силой держит.
    p.fastening = Fastening::Bolted(bolted(ThreadEngagement::ThroughBolt));
    let through = assess_part(&p, &l);
    assert_eq!(
        outcome(&through, FindingKind::Fasteners),
        Outcome::Pass,
        "{through:#?}"
    );
}

#[test]
fn bolt_head_crushes_plastic() {
    let mut p = printed(MaterialId::Pla, 0.0);
    p.fastening = Fastening::Bolted(bolted(ThreadEngagement::ThroughBolt));
    let l = room(LoadCase {
        axial: 10.0,
        fastener_tension: 600.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    let x = get(&f, FindingKind::Fasteners);
    assert_eq!(x.outcome, Outcome::Fail, "{x:#?}");
    assert!(x.detail.contains("под головкой"), "{}", x.detail);
}

#[test]
fn bearing_in_plastic_fails_where_metal_holds() {
    let spec = bolted(ThreadEngagement::ThroughBolt);
    let l = room(LoadCase {
        axial: 10.0,
        fastener_shear: 600.0,
        ..LoadCase::default()
    });
    let mut plastic = printed(MaterialId::Pla, 0.0);
    plastic.fastening = Fastening::Bolted(spec);
    let mut steel = metal(MaterialId::SteelS235);
    steel.fastening = Fastening::Bolted(spec);
    assert_eq!(
        outcome(&assess_part(&plastic, &l), FindingKind::Fasteners),
        Outcome::Fail
    );
    assert_eq!(
        outcome(&assess_part(&steel, &l), FindingKind::Fasteners),
        Outcome::Pass
    );
}

#[test]
fn hole_too_close_to_edge_fails() {
    let mut p = metal(MaterialId::SteelS235);
    p.fastening = Fastening::Bolted(FastenerSpec {
        edge_distance: 3.5 * MM,
        ..bolted(ThreadEngagement::ThroughBolt)
    });
    let l = room(LoadCase {
        axial: 10.0,
        fastener_shear: 10.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    let x = get(&f, FindingKind::Fasteners);
    assert_eq!(x.outcome, Outcome::Fail);
    assert!(x.detail.contains("кромк"), "{}", x.detail);
}

#[test]
fn unspecified_fastening_is_not_a_pass() {
    let mut p = metal(MaterialId::Al6061T6);
    p.fastening = Fastening::NotSpecified;
    let l = room(LoadCase {
        axial: 100.0,
        cycles: 1e4,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    assert_eq!(outcome(&f, FindingKind::Fasteners), Outcome::NotAnalyzed);
    assert_eq!(overall(&f), Outcome::NotAnalyzed);
}

#[test]
fn force_on_missing_fastener_is_a_fail() {
    let p = metal(MaterialId::Al6061T6);
    let l = room(LoadCase {
        axial: 100.0,
        fastener_shear: 50.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Fasteners),
        Outcome::Fail
    );
}

// --- Печать --------------------------------------------------------------------------------

#[test]
fn unprintable_overhang_fails_for_that_reason() {
    let mut p = printed(MaterialId::Pla, 0.0);
    p.print.as_mut().unwrap().geometry.max_overhang_deg = 80.0;
    let l = room(LoadCase {
        axial: 10.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    let x = get(&f, FindingKind::Printability);
    assert_eq!(x.outcome, Outcome::Fail);
    assert!(x.detail.contains("нависание"), "{}", x.detail);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::Pass);
    p.print.as_mut().unwrap().geometry.supports = true;
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Printability),
        Outcome::Pass
    );
}

#[test]
fn oversized_part_fails_printability() {
    let mut p = printed(MaterialId::Petg, 0.0);
    p.print.as_mut().unwrap().geometry.bbox = [0.3, 0.02, 0.02];
    let l = room(LoadCase {
        axial: 10.0,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Printability),
        Outcome::Fail
    );
}

#[test]
fn long_abs_without_enclosure_fails_printability() {
    let mut p = printed(MaterialId::Abs, 0.0);
    {
        let g = &mut p.print.as_mut().unwrap().geometry;
        g.bbox = [0.2, 0.02, 0.02];
        g.enclosed = false;
    }
    let l = room(LoadCase {
        axial: 10.0,
        ..LoadCase::default()
    });
    let f = assess_part(&p, &l);
    let x = get(&f, FindingKind::Printability);
    assert_eq!(x.outcome, Outcome::Fail);
    assert!(x.detail.contains("камеры"), "{}", x.detail);
}

// --- Запасы --------------------------------------------------------------------------------

#[test]
fn body_worn_margin_cannot_be_lowered() {
    let mut p = metal(MaterialId::Al6061T6);
    p.margins.static_yield = 1.1;
    p.margins.fatigue = 1.1;
    // σ = 8 кН / 180 мм² = 44 МПа — запас большой; подбираем запас ≈ 1.5.
    let force = 240e6 / 1.5 * p.section.area();
    let l = room(LoadCase {
        axial: force,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Static),
        Outcome::Fail
    );
    p.body_worn = false;
    assert_eq!(
        outcome(&assess_part(&p, &l), FindingKind::Static),
        Outcome::Pass
    );
}

#[test]
fn default_margins_are_two() {
    let m = Margins::default();
    assert_eq!((m.static_yield, m.fatigue), (2.0, 2.0));
    assert_eq!(BODY_WORN_MIN_MARGIN, 2.0);
}

#[test]
fn margin_boundary_is_exact() {
    let p = metal(MaterialId::Al6061T6);
    let at = 240e6 / 2.0 * p.section.area();
    let just_over = room(LoadCase {
        axial: at * 1.0001,
        ..LoadCase::default()
    });
    let just_under = room(LoadCase {
        axial: at * 0.9999,
        ..LoadCase::default()
    });
    assert_eq!(
        outcome(&assess_part(&p, &just_over), FindingKind::Static),
        Outcome::Fail
    );
    assert_eq!(
        outcome(&assess_part(&p, &just_under), FindingKind::Static),
        Outcome::Pass
    );
}

// --- Некорректный вход ---------------------------------------------------------------------

fn assert_all_fail(p: &PartSpec, l: &LoadCase, what: &str) {
    let f = assess_part(p, l);
    assert_eq!(overall(&f), Outcome::Fail, "{what}");
    assert_eq!(outcome(&f, FindingKind::Input), Outcome::Fail, "{what}");
    assert!(
        f.iter().all(|x| x.outcome == Outcome::Fail),
        "{what}: есть не-Fail запись"
    );
    assert!(
        f.iter().all(|x| x.value.is_finite() && x.limit.is_finite()),
        "{what}: нечисловое значение в отчёте"
    );
}

#[test]
fn nan_and_infinity_in_any_load_field_fail() {
    let p = printed(MaterialId::Petg, 0.0);
    let base = room(LoadCase {
        axial: 10.0,
        cycles: 100.0,
        ..LoadCase::default()
    });
    type Set = fn(&mut LoadCase, f64);
    let fields: [(&str, Set); 11] = [
        ("axial", |l, v| l.axial = v),
        ("shear", |l, v| l.shear = v),
        ("bending", |l, v| l.bending = v),
        ("torsion", |l, v| l.torsion = v),
        ("temperature", |l, v| l.temperature_c = v),
        ("cycles", |l, v| l.cycles = v),
        ("load_ratio", |l, v| l.load_ratio = v),
        ("sustained_fraction", |l, v| l.sustained_fraction = v),
        ("sustained_hours", |l, v| l.sustained_hours = v),
        ("fastener_shear", |l, v| l.fastener_shear = v),
        ("fastener_tension", |l, v| l.fastener_tension = v),
    ];
    for (name, set) in fields {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut l = base.clone();
            set(&mut l, bad);
            assert_all_fail(&p, &l, &format!("{name} = {bad}"));
        }
    }
    for bad in [f64::NAN, f64::INFINITY, -1.0, 2.0] {
        let mut l = base.clone();
        l.spectrum = vec![LoadBlock {
            fraction: bad,
            cycles: 10.0,
        }];
        assert_all_fail(&p, &l, &format!("доля блока {bad}"));
    }
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        let mut l = base.clone();
        l.spectrum = vec![LoadBlock {
            fraction: 0.5,
            cycles: bad,
        }];
        assert_all_fail(&p, &l, &format!("циклы блока {bad}"));
    }
}

#[test]
fn nan_and_infinity_in_any_part_field_fail() {
    let l = room(LoadCase {
        axial: 10.0,
        cycles: 100.0,
        ..LoadCase::default()
    });
    let base = printed(MaterialId::Petg, 0.0);
    type Set = fn(&mut PartSpec, f64);
    let fields: [(&str, Set); 20] = [
        ("b", |p, v| p.section = Section::Rect { b: v, h: 0.01 }),
        ("h", |p, v| p.section = Section::Rect { b: 0.02, h: v }),
        ("d", |p, v| p.section = Section::Round { d: v }),
        ("od", |p, v| {
            p.section = Section::Tube { od: v, wall: 0.001 }
        }),
        ("wall", |p, v| {
            p.section = Section::Tube { od: 0.02, wall: v }
        }),
        ("length", |p, v| p.length = v),
        ("hole", |p, v| {
            p.raiser = StressRaiser::Hole {
                d: v,
                orientation: HoleOrientation::InPlane,
                centered: true,
            }
        }),
        ("fillet big", |p, v| {
            p.raiser = StressRaiser::Fillet { big: v, r: 0.001 }
        }),
        ("fillet r", |p, v| {
            p.raiser = StressRaiser::Fillet { big: 0.02, r: v }
        }),
        ("margin static", |p, v| p.margins.static_yield = v),
        ("margin fatigue", |p, v| p.margins.fatigue = v),
        ("margin buckling", |p, v| p.margins.buckling = v),
        ("nozzle", |p, v| {
            p.print.as_mut().unwrap().process.nozzle = v
        }),
        ("layer", |p, v| p.print.as_mut().unwrap().process.layer = v),
        ("infill", |p, v| {
            p.print.as_mut().unwrap().process.infill = v
        }),
        ("angle", |p, v| {
            p.print.as_mut().unwrap().process.load_angle_deg = v
        }),
        ("min_wall", |p, v| {
            p.print.as_mut().unwrap().geometry.min_wall = v
        }),
        ("overhang", |p, v| {
            p.print.as_mut().unwrap().geometry.max_overhang_deg = v
        }),
        ("bridge", |p, v| {
            p.print.as_mut().unwrap().geometry.max_bridge = v
        }),
        ("sn a", |p, v| {
            p.measured_sn = Some(MeasuredSn {
                a: v,
                b: -0.1,
                max_cycles: 1e6,
            })
        }),
    ];
    for (name, set) in fields {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut p = base.clone();
            set(&mut p, bad);
            assert_all_fail(&p, &l, &format!("{name} = {bad}"));
        }
    }
}

#[test]
fn nan_in_fastener_fails() {
    let l = room(LoadCase {
        axial: 10.0,
        fastener_shear: 10.0,
        ..LoadCase::default()
    });
    let mut p = metal(MaterialId::SteelS235);
    p.fastening = Fastening::Bolted(FastenerSpec {
        plate_thickness: f64::NAN,
        ..bolted(ThreadEngagement::ThroughBolt)
    });
    assert_all_fail(&p, &l, "толщина под болтом");
}

#[test]
fn millimetres_in_metre_fields_rejected() {
    let l = room(LoadCase {
        axial: 100.0,
        ..LoadCase::default()
    });
    // Сечение «20×10» вместо 0.02×0.01 дало бы гигантский ложный запас.
    let p = PartSpec::new(
        "мм вместо м",
        MaterialId::Al6061T6,
        Section::Rect { b: 20.0, h: 10.0 },
        0.1,
    );
    assert_all_fail(&p, &l, "сечение в мм");
    let p = PartSpec::new(
        "длина в мм",
        MaterialId::Al6061T6,
        Section::Round { d: 0.01 },
        100.0,
    );
    assert_all_fail(&p, &l, "длина в мм");
}

#[test]
fn impossible_geometry_rejected() {
    let l = room(LoadCase {
        axial: 100.0,
        ..LoadCase::default()
    });
    let mut p = metal(MaterialId::SteelS235);
    p.raiser = StressRaiser::Hole {
        d: 0.05,
        orientation: HoleOrientation::OutOfPlane,
        centered: true,
    };
    assert_all_fail(&p, &l, "отверстие шире полосы");
    let mut p = metal(MaterialId::SteelS235);
    p.raiser = StressRaiser::Fillet {
        big: 0.003,
        r: 0.001,
    };
    assert_all_fail(&p, &l, "галтель наоборот");
    let mut p = metal(MaterialId::SteelS235);
    p.margins.static_yield = 0.5;
    assert_all_fail(&p, &l, "запас меньше 1");
}

#[test]
fn overall_rules() {
    let f = |outcome, value| PartFinding {
        kind: FindingKind::Static,
        outcome,
        value,
        limit: 1.0,
        unit: "×",
        detail: String::new(),
    };
    assert_eq!(overall(&[]), Outcome::Fail);
    assert_eq!(overall(&[f(Outcome::Pass, 2.0)]), Outcome::Pass);
    assert_eq!(
        overall(&[f(Outcome::Pass, 2.0), f(Outcome::NotAnalyzed, 0.0)]),
        Outcome::NotAnalyzed
    );
    assert_eq!(
        overall(&[f(Outcome::NotAnalyzed, 0.0), f(Outcome::Fail, 0.0)]),
        Outcome::Fail
    );
    // Подделанный Pass с NaN всё равно отказ.
    assert_eq!(overall(&[f(Outcome::Pass, f64::NAN)]), Outcome::Fail);
    assert_eq!(overall(&[f(Outcome::Pass, f64::INFINITY)]), Outcome::Fail);
}

#[test]
fn zero_load_is_not_a_pass() {
    let p = metal(MaterialId::Al6061T6);
    let f = assess_part(&p, &room(LoadCase::default()));
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::NotAnalyzed);
    assert_eq!(outcome(&f, FindingKind::Fatigue), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
}

#[test]
fn stress_raiser_on_composite_or_round_not_guessed() {
    let l = room(LoadCase {
        axial: 100.0,
        cycles: 1e4,
        ..LoadCase::default()
    });
    let mut round = PartSpec::new(
        "вал",
        MaterialId::SteelS235,
        Section::Round { d: 0.02 },
        0.1,
    );
    round.raiser = StressRaiser::Hole {
        d: 0.004,
        orientation: HoleOrientation::InPlane,
        centered: true,
    };
    round.fastening = Fastening::NoFasteners;
    let f = assess_part(&round, &l);
    assert_eq!(outcome(&f, FindingKind::Static), Outcome::NotAnalyzed);
    assert_eq!(outcome(&f, FindingKind::Fatigue), Outcome::NotAnalyzed);

    let mut off = metal(MaterialId::SteelS235);
    off.raiser = StressRaiser::Hole {
        d: 0.004,
        orientation: HoleOrientation::OutOfPlane,
        centered: false,
    };
    assert_eq!(
        outcome(&assess_part(&off, &l), FindingKind::Static),
        Outcome::NotAnalyzed
    );
}

// --- Свойства на переборе ------------------------------------------------------------------

/// Простой детерминированный генератор: тест воспроизводим без внешних зависимостей.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }

    fn log(&mut self, a: f64, b: f64) -> f64 {
        (self.range(a.ln(), b.ln())).exp()
    }
}

fn random_case(rng: &mut Lcg) -> (PartSpec, LoadCase) {
    let id = MaterialId::ALL[(rng.next() * 13.0) as usize % 13];
    let section = match (rng.next() * 3.0) as u32 {
        0 => Section::Rect {
            b: rng.log(2e-3, 0.05),
            h: rng.log(2e-3, 0.05),
        },
        1 => Section::Round {
            d: rng.log(2e-3, 0.05),
        },
        _ => {
            let od = rng.log(4e-3, 0.05);
            Section::Tube {
                od,
                wall: od * rng.range(0.02, 0.45),
            }
        }
    };
    let mut p = PartSpec::new("перебор", id, section, rng.log(0.01, 1.0));
    p.fastening = Fastening::NoFasteners;
    if id.material().is_printed() {
        p.print = Some(PrintSpec {
            process: PrintProcess {
                nozzle: rng.range(0.2e-3, 0.8e-3),
                layer: rng.range(0.08e-3, 0.3e-3),
                walls: (rng.next() * 6.0) as u32 + 1,
                infill: rng.next(),
                load_angle_deg: rng.range(0.0, 90.0),
            },
            geometry: geometry(),
        });
    }
    let sign = if rng.next() < 0.5 { -1.0 } else { 1.0 };
    let l = LoadCase {
        axial: sign * rng.log(1.0, 2e4),
        shear: rng.log(0.1, 2e3),
        bending: rng.log(0.01, 100.0),
        torsion: rng.log(0.01, 50.0),
        temperature_c: rng.range(0.0, 120.0),
        cycles: rng.log(10.0, 1e8),
        load_ratio: rng.range(-1.0, 1.0),
        sustained_fraction: rng.next(),
        sustained_hours: rng.log(0.1, 2000.0),
        ..LoadCase::default()
    };
    (p, l)
}

#[test]
fn random_parts_never_report_nonfinite_pass() {
    let mut rng = Lcg(0x5EED);
    for _ in 0..20_000 {
        let (p, l) = random_case(&mut rng);
        let f = assess_part(&p, &l);
        assert_eq!(f.len(), FindingKind::CHECKS.len(), "{p:?}");
        for x in &f {
            assert!(
                x.value.is_finite() && x.limit.is_finite(),
                "{x:#?}\n{p:#?}\n{l:#?}"
            );
            if x.outcome == Outcome::Pass && x.unit == "×" && x.limit > 0.0 {
                assert!(x.value >= x.limit, "Pass без запаса: {x:#?}");
            }
            if x.outcome == Outcome::Pass && matches!(x.unit, "Н" | "МПа" | "°C") && x.limit > 0.0
            {
                assert!(x.value <= x.limit, "Pass выше предела: {x:#?}");
            }
        }
    }
}

#[test]
fn more_load_never_turns_fail_into_pass() {
    let mut rng = Lcg(0xFACE);
    for _ in 0..5_000 {
        let (p, l) = random_case(&mut rng);
        let heavier = LoadCase {
            axial: l.axial * 1.5,
            shear: l.shear * 1.5,
            bending: l.bending * 1.5,
            torsion: l.torsion * 1.5,
            ..l.clone()
        };
        let (a, b) = (assess_part(&p, &l), assess_part(&p, &heavier));
        for kind in [
            FindingKind::Static,
            FindingKind::Fatigue,
            FindingKind::Buckling,
            FindingKind::Creep,
        ] {
            if outcome(&a, kind) == Outcome::Fail {
                assert_eq!(
                    outcome(&b, kind),
                    Outcome::Fail,
                    "{kind:?}: при большей нагрузке отказ исчез\n{p:#?}\n{l:#?}\n{:#?}",
                    get(&b, kind)
                );
            }
        }
    }
}

#[test]
fn hotter_part_never_turns_fail_into_pass() {
    let mut rng = Lcg(0xBEEF);
    for _ in 0..5_000 {
        let (p, l) = random_case(&mut rng);
        let hotter = LoadCase {
            temperature_c: l.temperature_c + 15.0,
            ..l.clone()
        };
        let (a, b) = (assess_part(&p, &l), assess_part(&p, &hotter));
        for kind in [FindingKind::Static, FindingKind::Temperature] {
            if outcome(&a, kind) == Outcome::Fail {
                assert_eq!(outcome(&b, kind), Outcome::Fail, "{kind:?}\n{p:#?}\n{l:#?}");
            }
        }
    }
}
