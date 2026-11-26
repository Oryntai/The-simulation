fn range(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Datasheet("математическая постановка стенда в этом тесте".into()),
    }
}

fn point(value: f64) -> Option<SourcedRange> {
    Some(range(value, value, value))
}

fn bench(amplitude: f64, duration: f64, dose_mm3: f64) -> SolderWindowInput {
    // Постановка из devices/electrical/depth_t5/expected.md. Для заданных C и G
    // вектор [3, 2, 3, 1] — тепловая мода с постоянной времени 1 с.
    // При 21,6 Вт баланс даёт T∞ = [708, 528, 528, 408] К.
    let steady = [708.0, 528.0, 528.0, 408.0];
    let mode = [3.0, 2.0, 3.0, 1.0];
    let initial_k = std::array::from_fn(|i| point(steady[i] - amplitude * mode[i]));
    SolderWindowInput {
        network: SolderNetworkSpec {
            capacity_j_k: [0.04, 0.015, 0.01, 0.02].map(point).into(),
            initial_k: initial_k.into(),
            pad_lead_w_k: point(0.03),
            pad_copper_w_k: point(0.18),
            copper_polygon_w_k: point(0.20),
            polygon_k: point(300.0),
            max_power_w: point(21.6),
        },
        thermal: SolderNetworkInput {
            contact: Some(SolderContact::Touching {
                conductance_w_k: point(0.12),
            }),
            requested_power_w: point(27.0), // Запрос 5/4 лимита.
        },
        force_n: point(1.0), // Проводимость калибрована в точке 1 Н.
        duration_s: point(duration),
        pitch_m: point(0.001),
        dose_m3: point(dose_mm3 * 1e-9), // Перевод мм³ в м³.
        limits: SolderWindowLimits {
            wetting_k: point(500.0),
            max_time_to_wet_s: point(1.0),
            liquidus_k: point(493.15), // 220 °C + 273,15 по паспорту сплава стенда.
            min_above_liquidus_s: point(0.5),
            max_above_liquidus_s: point(1.2),
            max_pad_k: point(515.0),
            max_component_k: point(520.0),
            min_volume_m3: point(2.5e-10),
            bridge: Some(SolderBridgeLimit {
                pitch_m: point(0.001).unwrap(),
                max_volume_m3: point(5e-10).unwrap(),
            }),
        },
    }
}

fn steps(n: u32) -> NonZeroU32 {
    NonZeroU32::new(n).unwrap()
}

fn analyzed<T>(result: Analysis<T>) -> T {
    match result {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("{reason}"),
    }
}

fn includes(bounds: SolderBounds, expected: f64, tolerance: f64) {
    assert!(
        bounds.lower <= expected + tolerance,
        "{bounds:?}, {expected}"
    );
    assert!(
        bounds.upper >= expected - tolerance,
        "{bounds:?}, {expected}"
    );
}
