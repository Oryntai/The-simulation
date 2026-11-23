//! Приёмка вердикта: заведомо опасный билд обязан быть отклонён именно по своей опасности,
//! а аккуратный билд не должен получать ложных тревог и не должен получать «годен»,
//! пока не все опасности посчитаны.

use sim_scenarios::blueprint::*;

fn failed(findings: &[Finding]) -> Vec<&'static str> {
    findings.iter().filter(|f| f.outcome == Outcome::Fail).map(|f| f.hazard).collect()
}

#[test]
fn every_dangerous_build_is_rejected_for_the_right_reason() {
    for (build, hazard) in dangerous_builds() {
        let r = assess(&build);
        let bad = failed(&r);
        println!("{}: {:?}, нарушено {bad:?}", build.name, verdict(&r));
        assert_eq!(verdict(&r), BuildVerdict::Rejected, "{} не отклонён", build.name);
        assert!(bad.contains(&hazard), "{}: ожидали провал по «{hazard}», а нарушено {bad:?}", build.name);
    }
}

#[test]
fn reference_build_has_no_false_alarms_but_is_not_proven() {
    let r = assess(&ExoBuild::reference());
    for f in &r {
        println!("{:?} {}: {:.2} / {:.2} {} ({})", f.outcome, f.hazard, f.value, f.limit, f.unit, f.detail);
    }
    assert!(failed(&r).is_empty(), "ложная тревога: {:?}", failed(&r));
    for h in [H_FUNCTION, H_IMPACT, H_CLAMP, H_JAM] {
        let f = r.iter().find(|f| f.hazard == h).unwrap();
        assert_eq!(f.outcome, Outcome::Pass, "{h} должна быть реально проверена: {}", f.detail);
    }
    assert_eq!(verdict(&r), BuildVerdict::NotProven);
}

/// Предел скорости считается под груз: тяжёлая кисть не должна бить сильнее предела.
#[test]
fn impact_stays_within_limit_for_every_payload() {
    for payload_kg in [0.0, 2.0, 5.0, 8.0] {
        let b = ExoBuild { payload_kg, ..ExoBuild::reference() };
        let r = assess(&b);
        let hit = r.iter().find(|f| f.hazard == H_IMPACT).unwrap();
        let clamp = r.iter().find(|f| f.hazard == H_CLAMP).unwrap();
        println!("груз {payload_kg} кг: удар {:.0}/{:.0} Н, зажим {:.0}/{:.0} Н", hit.value, hit.limit, clamp.value, clamp.limit);
        assert_eq!(hit.outcome, Outcome::Pass, "удар при {payload_kg} кг: {}", hit.detail);
        assert_eq!(clamp.outcome, Outcome::Pass, "зажим при {payload_kg} кг: {}", clamp.detail);
    }
}

#[test]
fn build_temperature_reaches_the_firmware_rig() {
    let hot = ExoBuild { ambient_c: 40.0, ..ExoBuild::reference() };
    let r = assess(&hot);
    let jam = r.iter().find(|f| f.hazard == H_JAM).unwrap();
    println!("заклинивание при 40 °C: пик обмотки {:.2} °C", jam.value);
    assert!(jam.value >= 40.0, "стенд считал не при температуре билда: {:.2} °C", jam.value);
}

#[test]
fn board_is_checked_at_the_current_firmware_allows() {
    let declared = ExoBuild { trace_mm: 1.5, copper_2oz: false, ..ExoBuild::reference() };
    let unlocked = ExoBuild { firmware_edit: "i_max_a = 30\ni_trip_a = 35\n", ..declared.clone() };
    let rise = |b: &ExoBuild| assess(b).iter().find(|f| f.hazard == H_TRACES).unwrap().value;
    let (a, b) = (rise(&declared), rise(&unlocked));
    println!("перегрев дорожки: заявлено {a:.1} К, по пределу прошивки {b:.1} К");
    assert!(b > a * 2.0, "предел тока из прошивки не дошёл до расчёта платы: {a:.1} → {b:.1} К");
}

fn all_pass() -> Vec<Finding> {
    REQUIRED_HAZARDS.iter().map(|h| Finding::passed(h, "")).collect()
}

#[test]
fn verdict_requires_positive_proof_for_every_hazard() {
    assert_eq!(verdict(&[]), BuildVerdict::NotProven);
    assert_eq!(verdict(&all_pass()), BuildVerdict::PassedSimulation);

    let mut missing = all_pass();
    missing.pop();
    assert_eq!(verdict(&missing), BuildVerdict::NotProven, "пропущенная опасность");

    let mut unknown = all_pass();
    unknown[3] = Finding::not_analyzed(unknown[3].hazard, "");
    assert_eq!(verdict(&unknown), BuildVerdict::NotProven, "непосчитанная опасность");

    let mut contradicted = all_pass();
    contradicted.push(Finding::not_analyzed(H_JAM, "второй расчёт не выполнен"));
    assert_eq!(verdict(&contradicted), BuildVerdict::NotProven, "часть проверок опасности не выполнена");

    let mut bad = all_pass();
    bad.push(Finding::failed(H_JAM, ""));
    assert_eq!(verdict(&bad), BuildVerdict::Rejected, "провал важнее всех «пройдено»");
}

#[test]
fn broken_numbers_are_a_failure() {
    for (value, limit) in [(f64::NAN, 1.0), (1.0, f64::NAN), (f64::INFINITY, f64::INFINITY), (f64::NEG_INFINITY, 1.0)] {
        assert_eq!(Finding::at_most(H_JAM, value, limit, "", "").outcome, Outcome::Fail, "≤: {value} / {limit}");
    }
    for (value, limit) in [(f64::NAN, 1.0), (1.0, f64::NAN), (f64::INFINITY, 1.0), (f64::INFINITY, f64::INFINITY)] {
        assert_eq!(Finding::at_least(H_STRUCT, value, limit, "", "").outcome, Outcome::Fail, "≥: {value} / {limit}");
    }
}
