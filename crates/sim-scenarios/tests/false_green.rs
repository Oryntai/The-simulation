//! Главное требование к мастерской: она не имеет права сказать «годно» без оснований.
//! Ложное «годно» на опасном билде дороже любой другой ошибки симулятора, поэтому здесь
//! проверяются не формулы, а сами зелёные выводы.

use std::collections::BTreeMap;
use std::panic::{catch_unwind, AssertUnwindSafe};

use sim_scenarios::beam::BeamStrut;
use sim_scenarios::exo_arm::ExoArm;
use sim_scenarios::human_limits::FLEXION_MEASURED;
use sim_scenarios::{catalog, Level, Scenario, Status};
use sim_verify::{Check, Tolerance};

/// «Совпало» при расхождении больше этой доли считается ложным зелёным.
const OK_MAX_REL_DIFF: f64 = 0.05;
/// Кнопки-переключатели отказов, которые обход включает и выключает.
const FAULT_ACTIONS: [&str; 5] = ["estop", "sensor", "jam", "radio", "hang"];
const ONE_SHOT_FAULT: &str = "hang";

fn check(simulated: f64, reference: f64) -> Check {
    Check {
        id: "false_green",
        title: "проверка вердикта".into(),
        simulated,
        reference,
        unit: "",
        method: String::new(),
        reference_method: String::new(),
        tolerance: Tolerance::Relative(0.01),
        note: String::new(),
    }
}

#[test]
fn broken_reference_never_passes() {
    assert!(!check(1.0, f64::INFINITY).passed(), "эталон = ∞ дал «пройдено»");
    assert!(!check(1.0, f64::NEG_INFINITY).passed(), "эталон = −∞ дал «пройдено»");
    assert!(!check(1.0, f64::NAN).passed(), "эталон = NaN дал «пройдено»");
    assert!(!check(f64::NAN, 1.0).passed(), "симуляция = NaN дала «пройдено»");
    assert!(!check(f64::INFINITY, 1.0).passed(), "симуляция = ∞ дала «пройдено»");
}

#[test]
fn load_above_measured_human_strength_is_not_green() {
    let mut sc = ExoArm::default();
    sc.set_param("payload", 15.0);
    sc.set_param("angle", 90.0);
    sc.reset();
    assert!(sc.run_until_settled(20.0), "симуляция не установилась");
    let tau = sc.sim_elbow_torque().abs();
    let women_90 = FLEXION_MEASURED[3].2;
    assert!(tau > women_90, "условие теста: {tau:.1} Н·м должно быть выше {women_90} Н·м");
    let green: Vec<_> = sc.notes().into_iter().filter(|n| n.level == Level::Good).collect();
    assert!(
        green.is_empty(),
        "{tau:.1} Н·м это больше максимума средней женщины ({women_90} Н·м), но вывод зелёный: {:?}",
        green.iter().map(|n| &n.text).collect::<Vec<_>>()
    );
}

#[test]
fn strut_without_fatigue_check_is_not_green() {
    let mut sc = BeamStrut::default();
    sc.apply_preset(1);
    sc.set_param("load", 5.0);
    sc.reset();
    sc.advance(0.1);
    let green: Vec<_> = sc.notes().into_iter().filter(|n| n.level == Level::Good).collect();
    assert!(
        green.is_empty(),
        "усталость и концентраторы не считаются, но вывод зелёный: {:?}",
        green.iter().map(|n| &n.text).collect::<Vec<_>>()
    );
}

#[derive(Default)]
struct Findings {
    ok_but_differs: BTreeMap<String, (f64, String)>,
    green_with_mismatch: BTreeMap<String, String>,
    panics: BTreeMap<String, String>,
}

fn inspect(s: &dyn Scenario, cfg: &str, f: &mut Findings) {
    let cmp = s.comparisons();
    for c in &cmp {
        if c.status != Status::Ok {
            continue;
        }
        let rel = if !c.sim.is_finite() || !c.reference.is_finite() {
            f64::INFINITY
        } else if c.reference.abs() > 1e-9 {
            ((c.sim - c.reference) / c.reference).abs()
        } else {
            (c.sim - c.reference).abs()
        };
        if rel > OK_MAX_REL_DIFF {
            let key = format!("{} / {}", s.id(), c.title);
            let e = f.ok_but_differs.entry(key).or_insert((0.0, String::new()));
            if rel > e.0 {
                *e = (rel, format!("{cfg}: {:.4} ↔ {:.4} {}", c.sim, c.reference, c.unit));
            }
        }
    }
    if let Some(bad) = cmp.iter().find(|c| matches!(c.status, Status::Mismatch | Status::OverLimit)) {
        if let Some(good) = s.notes().into_iter().find(|n| n.level == Level::Good) {
            f.green_with_mismatch
                .entry(format!("{} / {}", s.id(), bad.title))
                .or_insert(format!("{cfg}: зелёный вывод «{}»", good.text));
        }
    }
}

fn fault_active(s: &dyn Scenario, id: &str) -> bool {
    s.actions().iter().any(|a| a.id == id && a.active)
}

/// Кнопки отказов — переключатели, поэтому нажимаем только если состояние другое.
fn set_fault(s: &mut dyn Scenario, id: &str, on: bool) {
    if fault_active(&*s, id) != on {
        s.run_action(id);
    }
}

fn run_config(s: &mut dyn Scenario, cfg: &str, f: &mut Findings, apply: impl FnOnce(&mut dyn Scenario)) {
    let id = s.id();
    let res = catch_unwind(AssertUnwindSafe(|| {
        apply(&mut *s);
        s.reset();
        for _ in 0..8 {
            s.advance(0.1);
        }
        inspect(&*s, cfg, f);
    }));
    if res.is_err() {
        f.panics.insert(format!("{id} / {cfg}"), "паника".into());
    }
}

/// Обход всех сценариев: значения по умолчанию, готовые варианты, все пункты списков и
/// края всех ползунков.
#[test]
fn green_status_is_backed_by_numbers_everywhere() {
    let mut f = Findings::default();
    let mut configs = 0usize;
    let mut faults = 0usize;
    for mut s in catalog() {
        let s: &mut dyn Scenario = s.as_mut();
        run_config(s, "по умолчанию", &mut f, |_| {});
        configs += 1;
        for (i, name) in s.presets().into_iter().enumerate() {
            run_config(s, &format!("вариант «{name}»"), &mut f, |s| s.apply_preset(i));
            configs += 1;
        }
        for ch in s.choices() {
            for (i, opt) in ch.options.iter().enumerate() {
                run_config(s, &format!("{}={opt}", ch.id), &mut f, |s| s.set_choice(ch.id, i));
                configs += 1;
            }
            run_config(s, "возврат", &mut f, |s| s.set_choice(ch.id, ch.selected));
        }
        for p in s.params() {
            for v in [p.min, p.max] {
                run_config(s, &format!("{}={v}", p.id), &mut f, |s| s.set_param(p.id, v));
                configs += 1;
            }
            run_config(s, "возврат", &mut f, |s| s.set_param(p.id, p.value));
        }
        // Отказы: включаем на ходу, смотрим выводы, выключаем. Пересборку прошивки не трогаем.
        let actions: Vec<_> = s.actions().into_iter().map(|a| a.id).collect();
        for id in FAULT_ACTIONS.iter().copied().filter(|a| actions.contains(a)) {
            let res = catch_unwind(AssertUnwindSafe(|| {
                set_fault(s, id, false);
                s.reset();
                for _ in 0..3 {
                    s.advance(0.1);
                }
                // Зависание — разовое событие, у него нет состояния «включено».
                let toggle = id != ONE_SHOT_FAULT;
                if toggle {
                    set_fault(s, id, true);
                    assert!(fault_active(s, id), "отказ {id} не включился");
                } else {
                    s.run_action(id);
                }
                for _ in 0..5 {
                    s.advance(0.1);
                }
                if toggle {
                    assert!(fault_active(s, id), "отказ {id} сам выключился");
                }
                inspect(&*s, &format!("отказ {id}, на ходу"), &mut f);
                if toggle {
                    set_fault(s, id, false);
                }
                s.reset();
            }));
            configs += 1;
            faults += 1;
            if res.is_err() {
                f.panics.insert(format!("{} / отказ {id}", s.id()), "паника".into());
            }
        }
    }

    println!("проверено конфигураций: {configs}, из них с включённым отказом: {faults}");
    assert_eq!(faults, FAULT_ACTIONS.len(), "обход должен пройти все отказы");
    println!("\n«совпало», но числа расходятся больше {:.0}%: {}", OK_MAX_REL_DIFF * 100.0, f.ok_but_differs.len());
    for (k, (rel, how)) in &f.ok_but_differs {
        println!("  {k}: {:.1}% ({how})", rel * 100.0);
    }
    println!("\nзелёный вывод при «верить нельзя»: {}", f.green_with_mismatch.len());
    for (k, how) in &f.green_with_mismatch {
        println!("  {k}: {how}");
    }
    println!("\nпадения на краях ползунков: {}", f.panics.len());
    for k in f.panics.keys() {
        println!("  {k}");
    }
    assert!(
        f.ok_but_differs.is_empty() && f.green_with_mismatch.is_empty() && f.panics.is_empty(),
        "найдены ложные зелёные или падения, список выше"
    );
}
