// Подключение к общему модулю схемы выполняет задача 3.7.
#[path = "../src/schematic/fuse_coordination.rs"]
mod fuse_coordination;

use fuse_coordination::*;
use sim_process::schematic::{Mode, Scope, Section, Status};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange, schematic};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «блок», одиночный импульс, 12 В, условно без отвода тепла. За первые 10 мс
// провод несёт 10 А, затем 20 А. Ключ на всём окне несёт 20 А при 2 В.
// I²t(10 мс) = 1 А²·с. При пределах провода 4…5 А²·с границы достижения
// (10 + (4−1)/400·1000) … (10 + (5−1)/400·1000) = 17,5…20 мс.
// Заданный SOA стенда при 300 К: 21…22 мс для указанного прямоугольного импульса.
// TCC 23…25 мс даёт запасы провода −7,5…−3 мс и ключа −4…−1 мс.
// TCC 15…17 мс даёт запасы провода +0,5…+5 мс и ключа +4…+7 мс.
// Близкая граница: 17,49 мс даёт +10 мкс, 17,50/17,51 мс касаются/перекрываются.
// При 20,01 мс минимального времени отключения весь интервал повреждения провода
// уже пройден, верхний запас −10 мкс. Полный интеграл до 30 мс равен 1 + 8 = 9 А²·с.
const SOURCE: &str = "Расчётная постановка стенда блока, одиночный импульс";
const START: Time = Time(1_000_000_000);
const MID: Time = Time(1_010_000_000);
const END: Time = Time(1_030_000_000);
const WINDOW: Interval = Interval {
    start: START,
    end: END,
};
const TOLERANCE: f64 = 1e-12; // С запасом на направленное округление коротких сумм f64.

fn range(min: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        max,
        nominal: min,
        provenance: Provenance::Derived(SOURCE.into()),
    }
}

fn exact(x: f64) -> SourcedRange {
    range(x, x)
}

fn point(x: f64) -> FuseBounds {
    FuseBounds { min: x, max: x }
}

fn analyzed<T>(analysis: &Analysis<T>) -> &T {
    let Analysis::Analyzed { value, .. } = analysis else {
        panic!("нет расчёта");
    };
    value
}

fn close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < TOLERANCE,
        "{actual} вместо {expected}"
    );
}

struct Bench {
    fuse: Section,
    wire: Section,
    switch: Section,
    path: Provenance,
    prospective_a: SourcedRange,
    recovery_v: SourcedRange,
    tau_s: SourcedRange,
    temperature_k: SourcedRange,
    wire_a: [SourcedRange; 2],
    switch_a: SourcedRange,
    switch_v: SourcedRange,
    breaking_a: SourcedRange,
    breaking_v: SourcedRange,
    clearing_s: SourcedRange,
    wire_i2t: SourcedRange,
    soa_s: SourcedRange,
}

impl Bench {
    fn new(clearing_min_s: f64, clearing_max_s: f64) -> Self {
        Self {
            fuse: Section::Element {
                component: "блок".into(),
                element: "f".into(),
            },
            wire: Section::Connection("провод".into()),
            switch: Section::Element {
                component: "блок".into(),
                element: "q".into(),
            },
            path: Provenance::Derived(SOURCE.into()),
            prospective_a: exact(20.0),
            recovery_v: exact(12.0),
            tau_s: exact(0.0), // Чисто резистив, L/R = 0.
            temperature_k: exact(300.0),
            wire_a: [exact(10.0), exact(20.0)],
            switch_a: exact(20.0),
            switch_v: exact(2.0),
            breaking_a: exact(30.0), // Условная отключающая способность стенда.
            breaking_v: exact(24.0), // Условный паспорт стенда, DC.
            clearing_s: range(clearing_min_s, clearing_max_s),
            wire_i2t: range(4.0, 5.0),
            soa_s: range(0.021, 0.022),
        }
    }

    fn trace(&self) -> [FuseFaultStep<'_>; 2] {
        [
            Interval {
                start: START,
                end: MID,
            },
            Interval {
                start: MID,
                end: END,
            },
        ]
        .map(|interval| FuseFaultStep {
            interval,
            wire_current_a: Some(&self.wire_a[usize::from(interval.start == MID)]),
            switch_current_a: Some(&self.switch_a),
            switch_voltage_v: Some(&self.switch_v),
        })
    }

    fn input<'a>(&'a self, trace: &'a [FuseFaultStep<'a>]) -> FuseCoordinationInput<'a> {
        FuseCoordinationInput {
            scope: Scope::Reference,
            mode: &Mode::Stall,
            fuse: &self.fuse,
            wire: &self.wire,
            switch: &self.switch,
            window: WINDOW,
            protection_path: Some(&self.path),
            trace,
            prospective_current_a: Some(&self.prospective_a),
            recovery_voltage_v: Some(&self.recovery_v),
            dc_time_constant_s: Some(&self.tau_s),
            fuse_temperature_k: Some(&self.temperature_k),
            wire_start_temperature_k: Some(&self.temperature_k),
            switch_start_temperature_k: Some(&self.temperature_k),
            prospective_waveform: Some(FuseWaveform::DcStep),
            switch_waveform: Some(FuseWaveform::DcStep),
        }
    }

    fn ratings(&self) -> FuseCoordinationRatings<'_> {
        let conditions = FuseDcConditions {
            temperature_k: point(300.0),
            time_constant_s: point(0.0),
            waveform: FuseWaveform::DcStep,
        };
        FuseCoordinationRatings {
            interrupt: Some(FuseInterruptRating {
                conditions,
                max_prospective_current_a: &self.breaking_a,
                max_recovery_voltage_v: &self.breaking_v,
            }),
            tcc: Some(FuseTcc {
                conditions,
                recovery_voltage_v: point(12.0),
                stage: FuseTimeStage::TotalClearing,
                bands: &[],
            }),
            wire: Some(FuseWireLimit {
                start_temperature_k: point(300.0),
                damage_i2t_a2s: &self.wire_i2t,
            }),
            switch: Some(FuseSwitchSoa {
                start_temperature_k: point(300.0),
                current_a: point(20.0),
                voltage_v: point(2.0),
                waveform: FuseWaveform::DcStep,
                damage_time_s: &self.soa_s,
            }),
        }
    }

    fn band(&self) -> FuseTccBand<'_> {
        FuseTccBand {
            prospective_current_a: point(20.0),
            time_s: Some(&self.clearing_s),
        }
    }

    fn assess(
        &self,
        adjust: impl FnOnce(&mut FuseCoordinationInput<'_>, &mut FuseCoordinationRatings<'_>),
    ) -> Result<schematic::Finding<FuseCoordinationEvidence>, ProcessError> {
        let trace = self.trace();
        let bands = [self.band()];
        let mut input = self.input(&trace);
        let mut ratings = self.ratings();
        ratings.tcc.as_mut().unwrap().bands = &bands;
        adjust(&mut input, &mut ratings);
        assess_fuse_coordination(&input, &ratings)
    }
}

#[test]
fn unsafe_and_corrected_fuse_have_signed_margins() {
    for (t_min, t_max, status, wire, switch) in [
        (
            0.023,
            0.025,
            Status::Fail,
            [-0.0075, -0.003],
            [-0.004, -0.001],
        ),
        (0.015, 0.017, Status::Pass, [0.0005, 0.005], [0.004, 0.007]),
    ] {
        let hit = Bench::new(t_min, t_max).assess(|_, _| {}).unwrap();
        assert_eq!(hit.status, status, "{hit:?}");
        assert_eq!(hit.evidence.code, "fuse_coordination");
        assert_eq!(hit.evidence.mode, Mode::Stall);
        assert_eq!(hit.evidence.window, WINDOW);
        assert_eq!(hit.subject, "блок/f");
        let e = &hit.evidence;
        assert_eq!(
            *analyzed(&e.clearing_time_s),
            FuseBounds {
                min: t_min,
                max: t_max
            }
        );
        let dose = analyzed(&e.wire).window_i2t_a2s;
        close(dose.min, 9.0);
        close(dose.max, 9.0);
        let damage = analyzed(&e.wire).damage_time_s;
        close(damage.earliest_s, 0.0175);
        close(damage.latest_s.unwrap(), 0.020);
        let wm = e.wire_margin_s.unwrap();
        let sm = e.switch_margin_s.unwrap();
        close(wm.min, wire[0]);
        close(wm.max.unwrap(), wire[1]);
        close(sm.min, switch[0]);
        close(sm.max.unwrap(), switch[1]);
    }
}

#[test]
fn crossing_or_touching_either_time_threshold_never_passes() {
    for (t_min, t_max, expected) in [
        (0.015, 0.01749, Status::Pass),
        (0.015, 0.01750, Status::NotAnalyzed),
        (0.015, 0.01751, Status::NotAnalyzed),
        (0.01999, 0.02005, Status::NotAnalyzed),
        (0.02000, 0.02005, Status::NotAnalyzed),
        (0.02001, 0.02005, Status::Fail),
    ] {
        let hit = Bench::new(t_min, t_max).assess(|_, _| {}).unwrap();
        assert_eq!(hit.status, expected, "{hit:?}");
    }
    for (t_min, t_max, expected) in [
        (0.015, 0.02099, Status::Pass),
        (0.015, 0.02100, Status::NotAnalyzed),
        (0.015, 0.02101, Status::NotAnalyzed),
        (0.02200, 0.02300, Status::NotAnalyzed),
        (0.02201, 0.02300, Status::Fail),
    ] {
        let mut bench = Bench::new(t_min, t_max);
        bench.wire_i2t = exact(10.0); // Выше 9 А²·с, в этом окне решает только SOA.
        let hit = bench.assess(|_, _| {}).unwrap();
        assert_eq!(hit.status, expected, "{hit:?}");
        assert!(
            analyzed(&hit.evidence.wire)
                .damage_time_s
                .latest_s
                .is_none()
        );
    }
}

#[test]
fn a_short_window_does_not_cover_latest_total_clearing() {
    let mut bench = Bench::new(0.015, 0.031);
    bench.wire_i2t = exact(10.0);
    bench.soa_s = exact(0.040);
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
}

#[test]
fn melting_only_missing_and_inapplicable_parameters_are_rejected_as_evidence() {
    let bench = Bench::new(0.015, 0.017);
    for hit in [
        bench.assess(|_, r| r.tcc.as_mut().unwrap().stage = FuseTimeStage::PreArcing),
        bench.assess(|_, r| r.tcc.as_mut().unwrap().bands = &[]),
        bench.assess(|_, r| r.interrupt = None),
        bench.assess(|_, r| r.wire = None),
        bench.assess(|_, r| r.switch = None),
        bench.assess(|i, _| i.protection_path = None),
        bench.assess(|i, _| i.prospective_current_a = None),
        bench.assess(|i, _| i.switch_start_temperature_k = None),
        bench.assess(|i, _| i.fuse_temperature_k = None),
        bench.assess(|i, _| i.wire_start_temperature_k = None),
        bench.assess(|i, _| i.prospective_waveform = None),
        bench.assess(|i, _| {
            i.prospective_waveform = Some(FuseWaveform::Characterized("иной импульс"))
        }),
        bench.assess(|i, _| i.trace = &[]),
        bench.assess(|i, _| i.scope = Scope::Assembly),
        bench.assess(|_, r| r.tcc.as_mut().unwrap().recovery_voltage_v = point(10.0)),
        bench.assess(|_, r| r.switch.as_mut().unwrap().voltage_v = point(1.0)),
    ] {
        let hit = hit.unwrap();
        assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    }
    let mut bench = bench;
    bench.clearing_s.provenance = Provenance::Estimate("время по аналогии".into());
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    bench.clearing_s = range(0.015, 0.017);
    bench.soa_s.provenance = Provenance::Estimate("SOA предполагается".into());
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
}

#[test]
fn breaking_current_and_recovery_voltage_are_independent_constraints() {
    let mut bench = Bench::new(0.015, 0.017);
    bench.breaking_a = range(19.0, 21.0);
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    bench.breaking_a = exact(19.0);
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::Fail, "{hit:?}");
    assert!(hit.reason.contains("отключающ"), "{hit:?}");
    bench.breaking_a = exact(20.0);
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::Pass, "{hit:?}");
    bench.breaking_v = exact(11.0);
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::Fail, "{hit:?}");
    assert!(hit.reason.contains("напряжение"), "{hit:?}");
}

#[test]
fn uncertain_interstep_bounds_widen_the_wire_damage_interval() {
    let mut bench = Bench::new(0.015, 0.017);
    // I²t за всё окно: 8²·0,01 + 18²·0,02 = 7,12 и 12²·0,01 + 22²·0,02 = 11,12.
    // Пересечения с 4 и 5 А²·с происходят после первых 10 мс.
    bench.wire_a = [range(8.0, 12.0), range(18.0, 22.0)];
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    let wire = analyzed(&hit.evidence.wire);
    close(wire.window_i2t_a2s.min, 7.12);
    close(wire.window_i2t_a2s.max, 11.12);
    close(wire.damage_time_s.earliest_s, 0.010 + (4.0 - 1.44) / 484.0);
    close(
        wire.damage_time_s.latest_s.unwrap(),
        0.010 + (5.0 - 0.64) / 324.0,
    );
}

#[test]
fn a_gap_or_an_unbounded_peak_blocks_a_pass() {
    let mut bench = Bench::new(0.015, 0.017);
    let bands = [bench.band()];
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    let mut trace = bench.trace();
    trace[1].interval.start = Time(MID.0 + 1); // Один непокрытый наносекундой участок.
    let hit = assess_fuse_coordination(&bench.input(&trace), &ratings).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");

    let mut trace = bench.trace();
    trace[1].switch_voltage_v = None;
    let hit = assess_fuse_coordination(&bench.input(&trace), &ratings).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    assert!(matches!(hit.evidence.switch, Analysis::NotAnalyzed { .. }));

    bench.wire_a[1].provenance = Provenance::Estimate("между тактами ток неизвестен".into());
    let hit = bench.assess(|_, _| {}).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    assert!(matches!(hit.evidence.wire, Analysis::NotAnalyzed { .. }));
}

#[test]
fn tcc_uses_all_covered_bands_and_refuses_to_interpolate_across_a_gap() {
    let mut bench = Bench::new(0.015, 0.017);
    // Синтетические соседние полосы одной TCC для разброса ступени 19…21 А.
    bench.prospective_a = range(19.0, 21.0);
    let upper_current_time = range(0.014, 0.016);
    let mut bands = [
        FuseTccBand {
            prospective_current_a: FuseBounds {
                min: 20.0,
                max: 21.0,
            },
            time_s: Some(&upper_current_time),
        },
        FuseTccBand {
            prospective_current_a: FuseBounds {
                min: 19.0,
                max: 20.0,
            },
            ..bench.band()
        },
    ];
    let trace = bench.trace();
    let input = bench.input(&trace);
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::Pass, "{hit:?}");
    assert_eq!(
        *analyzed(&hit.evidence.clearing_time_s),
        FuseBounds {
            min: 0.014,
            max: 0.017
        }
    );

    bands[0].prospective_current_a.min = 20.0_f64.next_up();
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
    assert!(matches!(
        hit.evidence.clearing_time_s,
        Analysis::NotAnalyzed { .. }
    ));

    bands[0].prospective_current_a.min = 20.0;
    bands[0].time_s = None; // Строка паспорта без обеих границ.
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");
}

#[test]
fn characterized_switch_profile_must_match_soa_conditions() {
    let bench = Bench::new(0.015, 0.017);
    let mut trace = bench.trace();
    let peak = exact(3.0);
    trace[1].switch_voltage_v = Some(&peak);
    let mut input = bench.input(&trace);
    let bands = [bench.band()];
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    ratings.switch.as_mut().unwrap().voltage_v = FuseBounds { min: 2.0, max: 3.0 };

    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::NotAnalyzed, "{hit:?}");

    // SOA синтетического стенда для одиночной двухступенчатой последовательности U.
    let shape = FuseWaveform::Characterized("двухступенчатый импульс стенда блока");
    ratings.switch.as_mut().unwrap().waveform = shape;
    input.switch_waveform = Some(shape);
    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::Pass, "{hit:?}");
}

#[test]
fn nonfinite_input_is_an_error_even_outside_the_applicable_band() {
    for nonfinite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let bench = Bench::new(0.015, 0.017);
        for result in [
            bench.assess(|_, r| r.tcc.as_mut().unwrap().recovery_voltage_v.min = nonfinite),
            bench.assess(|_, r| {
                r.interrupt.as_mut().unwrap().conditions.time_constant_s.max = nonfinite
            }),
            bench.assess(|_, r| r.switch.as_mut().unwrap().current_a.max = nonfinite),
            bench.assess(|_, r| r.wire.as_mut().unwrap().start_temperature_k.min = nonfinite),
        ] {
            assert!(
                matches!(result, Err(ProcessError::InvalidInput(_))),
                "{result:?}"
            );
        }
        let bad = exact(nonfinite);
        let mut trace = bench.trace();
        trace[1].wire_current_a = Some(&bad);
        let result = assess_fuse_coordination(&bench.input(&trace), &bench.ratings());
        assert!(
            matches!(result, Err(ProcessError::InvalidInput(_))),
            "{result:?}"
        );

        let mut bench = bench;
        bench.soa_s.nominal = nonfinite;
        let result = bench.assess(|_, _| {});
        assert!(
            matches!(result, Err(ProcessError::InvalidInput(_))),
            "{result:?}"
        );
    }
}

#[test]
fn datasheet_tcc_nodes_match_existing_battery_reference_rows() {
    // Littelfuse ATOF 0287010, таблица Opening Time (стр. 3).
    // Эти же верхние времена проверяет sim-battery в battery.ato_200 и battery.ato_350.
    // Здесь проверяется ввод паспортных строк в расчётный стенд. Условия его
    // применения к реальному монтажу и фактическое гашение дуги не проверяются.
    const DATASHEET: &str = "Littelfuse ATOF 0287010, Time-Current Characteristic Data, https://www.littelfuse.com/assetdocs/littelfuse-datasheet-287-atof?assetguid=43dcdce8-8ca2-426f-8998-7e566f048d40";
    for (current, t_min, t_max) in [(20.0, 0.15, 5.0), (35.0, 0.08, 0.5)] {
        let mut bench = Bench::new(t_min, t_max);
        bench.prospective_a = exact(current);
        bench.clearing_s.provenance = Provenance::Datasheet(DATASHEET.into());
        let bands = [FuseTccBand {
            prospective_current_a: point(current),
            ..bench.band()
        }];
        let trace = bench.trace();
        let mut ratings = bench.ratings();
        ratings.tcc.as_mut().unwrap().bands = &bands;
        let hit = assess_fuse_coordination(&bench.input(&trace), &ratings).unwrap();
        assert_eq!(
            *analyzed(&hit.evidence.clearing_time_s),
            FuseBounds {
                min: t_min,
                max: t_max
            }
        );
    }
}

#[test]
fn wire_threshold_matches_the_existing_iec_pvc_reference() {
    // IEC 60364-5-54 табл. A.54, переход ПВХ 70 → 160 °C, k = 115.
    // Одно и то же k независимо проверяет sim-battery как battery.k_pvc_160.
    // Адиабатический эталон с S = 1 мм²: I²t = (kS)² = 13 225 А²·с.
    // Источник 600 А на 50 мс (импульс из battery.adiabatic_vs_rk4) даёт 18 000 А²·с,
    // предел достигается через 13 225 / 360 000 ≈ 36,736 мс.
    let mut bench = Bench::new(0.030, 0.035);
    bench.prospective_a = exact(600.0);
    bench.breaking_a = exact(1_000.0); // Условная отключающая способность этого стенда.
    bench.wire_a = [exact(600.0), exact(600.0)];
    bench.wire_i2t = exact(13_225.0);
    bench.switch_a = exact(600.0);
    bench.soa_s = range(0.045, 0.048); // Синтетический предел ключа после предела провода.
    let initial = exact(343.15); // 70 °C в абсолютной шкале.
    let mut trace = bench.trace();
    trace[1].interval.end = Time(START.0 + 50_000_000);
    let bands = [FuseTccBand {
        prospective_current_a: point(600.0),
        ..bench.band()
    }];
    let mut input = bench.input(&trace);
    input.window.end = trace[1].interval.end;
    input.wire_start_temperature_k = Some(&initial);
    let mut ratings = bench.ratings();
    ratings.tcc.as_mut().unwrap().bands = &bands;
    ratings.wire.as_mut().unwrap().start_temperature_k = point(343.15);
    ratings.switch.as_mut().unwrap().current_a = point(600.0);
    let hit = assess_fuse_coordination(&input, &ratings).unwrap();
    assert_eq!(hit.status, Status::Pass, "{hit:?}");
    let wire = analyzed(&hit.evidence.wire);
    for integral in [wire.window_i2t_a2s.min, wire.window_i2t_a2s.max] {
        assert!((integral - 18_000.0).abs() < 18_000.0 * TOLERANCE);
    }
    close(wire.damage_time_s.earliest_s, 13_225.0 / 360_000.0);
    close(wire.damage_time_s.latest_s.unwrap(), 13_225.0 / 360_000.0);
}
