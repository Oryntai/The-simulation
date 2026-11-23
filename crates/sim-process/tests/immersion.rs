//! Независимые геометрические и физические эталоны для синтетического блока.

use std::f64::consts::{FRAC_1_SQRT_2, TAU};

use sim_process::work::immersion::*;
use sim_process::{Analysis, SourcedRange};
use sim_radio::dielectric::Medium;
use sim_world::Provenance;

fn band(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Measured("синтетический стенд с аналитической геометрией".into()),
    }
}

fn r(x: f64) -> SourcedRange {
    band(x, x, x)
}

fn spec() -> ImmersionSpec {
    ImmersionSpec {
        body: "блок".into(),
        half_extent_m: [r(0.5), r(1.0), r(0.5)],
        region: ImmersionRegion {
            surface_point_m: [0.0; 3].map(r),
            outward_normal: [0.0, 0.0, 1.0].map(r),
            density_kg_m3: r(1000.0),
            viscosity_pa_s: r(0.001),
            flow_m_s: [1.0, 0.0, 0.0].map(r),
            temperature_k: r(293.15),
        },
        drag: Some(DragSpec {
            coefficient: r(0.4),
            reference_length_m: r(1.0),
            calibrated_reynolds: band(1e5, 1e6, 2e6),
        }),
        heat: Some(WetHeatSpec {
            coefficient_w_m2_k: r(50.0),
            calibrated_speed_m_s: band(0.0, 1.0, 2.0),
            calibrated_area_m2: band(0.01, 5.0, 10.01),
        }),
        radio: Some(RadioFluid {
            water: RadioWater::Fresh,
            composition_source: r(0.0).provenance,
            calibrated_temperature_k: band(283.15, 293.15, 303.15),
        }),
    }
}

fn input() -> ImmersionInput {
    ImmersionInput {
        box_center_m: [0.0; 3],
        box_axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        body_com_m: [0.1, 0.0, 0.2],
        velocity_at_com_m_s: [0.0; 3],
        angular_velocity_rad_s: [0.0; 3],
        gravity_m_s2: [0.0, 0.0, -10.0],
        engine_density_kg_m3: 0.0,
        engine_viscosity_pa_s: 0.0,
        housing_temperature_k: Some(313.15),
        antenna_m: Some([0.0, 0.0, -0.2]),
    }
}

fn value<T: std::fmt::Debug>(analysis: Analysis<T>) -> T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        missing => panic!("ожидался расчёт: {missing:?}"),
    }
}

fn unknown<T>(analysis: &Analysis<T>) -> bool {
    matches!(analysis, Analysis::NotAnalyzed { .. })
}

fn close(actual: f64, expected: f64) {
    // Зазор на округление двойной точности при суммировании тетраэдров.
    assert!(
        (actual - expected).abs() <= 2e-12 * expected.abs().max(1.0),
        "{actual} ≠ {expected}"
    );
}

fn vector(actual: [f64; 3], expected: [f64; 3]) {
    for (a, e) in actual.into_iter().zip(expected) {
        close(a, e);
    }
}

#[test]
fn dry_partial_and_full_block_match_geometry_buoyancy_and_heat() {
    let spec = spec();
    let input = input();
    let dry = spec
        .evaluate(&ImmersionInput {
            box_center_m: [0.0, 0.0, 0.5],
            antenna_m: Some([0.0, 0.0, 0.5]),
            ..input
        })
        .unwrap();
    assert_eq!(dry.volume_m3, 0.0);
    assert_eq!(dry.wetted_area_m2, 0.0);
    assert_eq!(dry.center_of_buoyancy_m, None);
    assert_eq!(value(dry.body_load).force_n, [0.0; 3]);
    assert_eq!(value(dry.drive_thermal), ThermalBoundary::Dry);
    assert_eq!(
        value(dry.radio),
        RadioEnvironment {
            medium: Medium::Air,
            depth_m: 0.0
        }
    );
    let half = spec.evaluate(&input).unwrap();
    // 1 × 2 × 0,5 м. Смочены дно и нижние половины боков.
    close(half.volume_m3, 1.0);
    close(half.wetted_area_m2, 5.0);
    vector(half.center_of_buoyancy_m.unwrap(), [0.0, 0.0, -0.25]);
    vector(half.buoyancy.force_n, [0.0, 0.0, 10_000.0]);
    vector(half.buoyancy.torque_nm, [0.0, 1000.0, 0.0]);
    // Поток +X. Силу даёт ρCd·(2×0,5)·1²/2, плечо по Z −0,45 м.
    vector(value(half.drag).force_n, [200.0, 0.0, 0.0]);
    let load = value(half.body_load);
    vector(load.force_n, [200.0, 0.0, 10_000.0]);
    vector(load.torque_nm, [0.0, 910.0, 0.0]);
    let ThermalBoundary::Wet {
        fluid_temperature_k,
        conductance_w_k,
        heat_out_w,
    } = value(half.drive_thermal)
    else {
        panic!("пропущена мокрая граница")
    };
    close(fluid_temperature_k, 293.15);
    close(conductance_w_k, 250.0);
    close(heat_out_w, 5000.0);
    let full = spec
        .evaluate(&ImmersionInput {
            box_center_m: [0.0, 0.0, -0.75],
            body_com_m: [0.0, 0.0, -0.75],
            housing_temperature_k: Some(283.15),
            ..input
        })
        .unwrap();
    close(full.volume_m3, 2.0);
    close(full.wetted_area_m2, 10.0);
    vector(full.center_of_buoyancy_m.unwrap(), [0.0, 0.0, -0.75]);
    vector(full.buoyancy.force_n, [0.0, 0.0, 20_000.0]);
    vector(value(full.drag).force_n, [400.0, 0.0, 0.0]);
    assert!(
        matches!(value(full.drive_thermal), ThermalBoundary::Wet { heat_out_w, .. }
        if (heat_out_w + 5000.0).abs() < 2e-12)
    );
    let tangent = spec
        .evaluate(&ImmersionInput {
            box_center_m: [0.0, 0.0, -0.5],
            ..input
        })
        .unwrap();
    close(tangent.volume_m3, 2.0);
    close(tangent.wetted_area_m2, 8.0);
}

#[test]
fn rotated_block_clips_wedges_and_computes_drag_in_both_directions() {
    let spec = spec();
    let input = ImmersionInput {
        box_axes: [
            [FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2],
            [0.0, 1.0, 0.0],
            [FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2],
        ],
        ..input()
    };
    let half = spec.evaluate(&input).unwrap();
    // Сечение XZ — ромб. Нижняя половина треугольная, центр по Z на трети высоты.
    close(half.volume_m3, 1.0);
    close(half.wetted_area_m2, 5.0);
    vector(
        half.center_of_buoyancy_m.unwrap(),
        [0.0, 0.0, -FRAC_1_SQRT_2 / 3.0],
    );
    let drag = 400.0 * FRAC_1_SQRT_2;
    vector(value(half.drag).force_n, [drag, 0.0, 0.0]);
    let reversed = spec
        .evaluate(&ImmersionInput {
            velocity_at_com_m_s: [2.0, 0.0, 0.0],
            ..input
        })
        .unwrap();
    vector(value(reversed.drag).force_n, [-drag, 0.0, 0.0]);
    let still = spec
        .evaluate(&ImmersionInput {
            velocity_at_com_m_s: [1.0, 0.0, 0.0],
            ..input
        })
        .unwrap();
    vector(value(still.drag).force_n, [0.0; 3]);
    let mut lower = spec.clone();
    lower.region.surface_point_m[2] = r(-FRAC_1_SQRT_2 / 2.0);
    let wedge = lower.evaluate(&input).unwrap();
    // Половина высоты треугольника даёт четверть площади сечения.
    close(wedge.volume_m3, 0.25);
    close(wedge.wetted_area_m2, 2.25);
    vector(
        wedge.center_of_buoyancy_m.unwrap(),
        [0.0, 0.0, -2.0 * FRAC_1_SQRT_2 / 3.0],
    );
}

#[test]
fn wet_radio_depth_feeds_catalog_medium_without_treating_it_as_link_budget() {
    // Частота контрольной радиолинии 2,4 ГГц; c — точное значение СИ.
    const F_HZ: f64 = 2.4e9;
    const C_M_S: f64 = 299_792_458.0;
    let signal = value(spec().evaluate(&input()).unwrap().radio);
    assert_eq!(signal.medium, Medium::FreshWater { t_c: 20.0 });
    close(signal.depth_m, 0.2);
    let eps = signal.medium.permittivity(F_HZ);
    let attenuation = eps.attenuation_db_per_m(F_HZ) * signal.depth_m;
    // Из k = ω/c·sqrt(ε*): Im(sqrt(ε*)) = sqrt((|ε*| − Re ε*)/2).
    let imaginary = ((eps.re.hypot(eps.im) - eps.re) / 2.0).sqrt();
    let expected = TAU * F_HZ / C_M_S * imaginary * 20.0 / 10.0_f64.ln() * 0.2;
    assert!(attenuation > 0.0);
    close(attenuation, expected);
    let mut no_fluid = spec();
    no_fluid.radio = None;
    assert!(unknown(&no_fluid.evaluate(&input()).unwrap().radio));
}

#[test]
fn immersion_refuses_double_drag_invalid_values_and_uncovered_regimes() {
    let spec = spec();
    let input = input();
    for bad in [
        ImmersionInput {
            engine_density_kg_m3: 1.2,
            ..input
        },
        ImmersionInput {
            engine_viscosity_pa_s: 1e-5,
            ..input
        },
        ImmersionInput {
            velocity_at_com_m_s: [f64::INFINITY, 0.0, 0.0],
            ..input
        },
        ImmersionInput {
            housing_temperature_k: Some(f64::NAN),
            ..input
        },
        ImmersionInput {
            gravity_m_s2: [1.0, 0.0, -10.0],
            ..input
        },
        ImmersionInput {
            box_axes: [[1.0, 0.0, 0.0]; 3],
            ..input
        },
    ] {
        assert!(spec.evaluate(&bad).is_err(), "{bad:?}");
    }
    let uncalibrated = spec
        .evaluate(&ImmersionInput {
            velocity_at_com_m_s: [20.0, 0.0, 0.0],
            ..input
        })
        .unwrap();
    assert!(unknown(&uncalibrated.drag));
    assert!(unknown(&uncalibrated.body_load));
    assert!(unknown(&uncalibrated.drive_thermal));
    close(uncalibrated.buoyancy.force_n[2], 10_000.0);
    let mut incomplete = spec.clone();
    incomplete.drag = None;
    incomplete.heat = None;
    let result = incomplete.evaluate(&input).unwrap();
    assert!(unknown(&result.body_load));
    assert!(unknown(&result.drive_thermal));
    incomplete.region.viscosity_pa_s.provenance = Provenance::Measured(" ".into());
    assert!(incomplete.evaluate(&input).is_err());
}

#[test]
fn oblique_surface_cuts_tetrahedron_with_origin_independent_center() {
    let mut spec = spec();
    let mut input = input();
    spec.half_extent_m = [0.5; 3].map(r);
    spec.region.flow_m_s = [0.0; 3].map(r);
    spec.region.outward_normal = [1.0; 3].map(r);
    // Для блока в (2,3,4) относительная плоскость x+y+z = −1. Нижний угол
    // отсекает тетраэдр с тремя ортогональными рёбрами по 0,5 м.
    spec.region.surface_point_m = [1.0, 3.0, 4.0].map(r);
    input.gravity_m_s2 = [-10.0 / 3.0_f64.sqrt(); 3];
    input.box_center_m = [2.0, 3.0, 4.0];
    input.body_com_m = input.box_center_m;
    let result = spec.evaluate(&input).unwrap();
    close(result.volume_m3, 0.5_f64.powi(3) / 6.0);
    close(result.wetted_area_m2, 3.0 * 0.5 * 0.5 * 0.5);
    vector(result.center_of_buoyancy_m.unwrap(), [1.625, 2.625, 3.625]);
    vector(
        result.buoyancy.force_n,
        [10_000.0 / 48.0 / 3.0_f64.sqrt(); 3],
    );
    vector(result.buoyancy.torque_nm, [0.0; 3]);
}
