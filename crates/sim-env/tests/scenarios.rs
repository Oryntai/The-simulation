//! Сценарии мастерской: экзоскелет на руке в холоде, жаре, воде и на ветру.

use sim_env::contact::{self, Material, RiskLevel};
use sim_env::convection::{self, Medium};
use sim_env::effects::{self, HandFunction};
use sim_env::indices::{self, WindChillRisk};
use sim_env::two_node::{TwoNodeInputs, TwoNodeWarning};
use sim_env::{air, immersion, strain, two_node};

#[test]
fn aluminium_cuff_in_frost_vs_padded_cuff() {
    // голая алюминиевая манжета на морозе −20 °C — обморожение сразу
    let bare = contact::contact_risk(Material::Aluminium6061, -20.0, 2.0);
    assert_eq!(bare.level, RiskLevel::FrostnipRisk);
    assert!(bare.contact_temp_c < 0.0);
    // вспененный неопрен — максимум холодовая боль, контакт далеко выше 0 °C
    let padded = contact::contact_risk(Material::NeopreneFoam, -20.0, 2.0);
    assert!(padded.level < RiskLevel::Numbness, "{padded:?}");
    assert!(padded.contact_temp_c > 15.0);
}

#[test]
fn thin_plastic_on_metal_protects_only_seconds() {
    let skin = Material::Skin.props();
    let pla = Material::Pla.props();
    let al = Material::Aluminium6061.props();
    let t = contact::layer_breakthrough_time_s(&pla, 1e-3, &al, 30.0, -20.0);
    assert!((5.0..60.0).contains(&t), "{t}");
    // толще слой — дольше защищает (время ~ L²)
    let t2 = contact::layer_breakthrough_time_s(&pla, 2e-3, &al, 30.0, -20.0);
    assert!((t2 / t - 4.0).abs() < 0.2, "{t} {t2}");
    let early = contact::layered_contact_temperature(&skin, 30.0, &pla, 1e-3, &al, -20.0, 0.5);
    let late = contact::layered_contact_temperature(&skin, 30.0, &pla, 1e-3, &al, -20.0, 600.0);
    assert!(early > late + 10.0);
}

#[test]
fn black_motor_housing_in_sun_can_burn() {
    let black = &effects::SURFACES[0].0;
    let t = effects::solar_device_temperature(black, 1000.0, 35.0, 1.0, 0.1);
    assert!(t > 65.0, "{t}");
    let r = contact::contact_risk(Material::Aluminium6061, t, 1.0);
    assert_eq!(r.level, RiskLevel::BurnRisk);
    // белое покрытие — ниже порога ожога даже при касании 10 с
    let white = &effects::SURFACES[3].0;
    let tw = effects::solar_device_temperature(white, 1000.0, 35.0, 1.0, 0.1);
    assert!(tw < contact::burn_threshold_c(contact::ContactClass::PaintedMetal, 10.0));
}

#[test]
fn forearm_heat_loss_wind_vs_water() {
    let air = Medium::air(60.0);
    let calm = convection::h_cylinder(&air, 0.08, 0.0, 33.0, 0.0);
    let wind = convection::h_cylinder(&air, 0.08, 10.0, 33.0, 0.0);
    let water = convection::h_cylinder(&Medium::sea(), 0.08, 0.2, 33.0, 10.0);
    assert!(calm > 2.0 && calm < 8.0, "{calm}");
    assert!(wind > 4.0 * calm, "{wind}");
    assert!(water > 20.0 * wind, "{water}");
    let hr = convection::h_radiation(0.98, 33.0, 0.0);
    assert!((4.0..6.0).contains(&hr), "{hr}");
}

#[test]
fn cold_device_brought_inside_fogs() {
    let c = air::condensation_check(23.0, 55.0, -15.0);
    assert!(c.condenses && c.frost);
    assert!(c.margin_k < -20.0);
}

#[test]
fn wind_chill_and_frostbite() {
    let w = indices::wind_chill(-25.0, 30.0 / 3.6);
    assert!((w.index_c + 39.0).abs() < 1.0, "{w:?}");
    assert!(w.risk >= WindChillRisk::High);
    assert!(w.frostbite_minutes.is_some_and(|m| m <= 15.0));
    assert_eq!(indices::beaufort(12.0).number, 6);
}

#[test]
fn heat_index_categories() {
    let hi = indices::heat_index_c(35.0, 60.0);
    assert!(hi > 45.0, "{hi}");
    assert_eq!(
        indices::HeatIndexCategory::from_index_c(hi),
        indices::HeatIndexCategory::Danger
    );
}

#[test]
fn worker_in_heat_hits_core_limit_and_flags_it() {
    let inp = TwoNodeInputs::ashrae55(38.0, 45.0, 0.5, 40.0, 3.0, 0.6)
        .for_person(80.0, 1.80)
        .with_exposure(120.0, 1.0);
    let out = two_node::two_node(&inp);
    assert!(out.t_core_c > strain::CORE_LIMIT_UNACCLIMATISED_C);
    assert!(out.warnings.contains(&TwoNodeWarning::CoreAboveLimit));
    assert!(out.water_loss_g > 500.0, "{}", out.water_loss_g);
    let e = out.energy;
    assert!(e.residual_j().abs() < 1e-6 * e.metabolic_j);
}

#[test]
fn immersion_timeline_cold_sea() {
    let tl = immersion::immersion_timeline(8.0);
    assert!(tl.cold_shock);
    let t30 = tl.time_to_core_30_min.unwrap();
    assert!((120.0..240.0).contains(&t30), "{t30}");
    assert_eq!(immersion::stage(1.0), immersion::ImmersionStage::ColdShock);
}

#[test]
fn hands_and_muscles_in_cold() {
    assert_eq!(effects::hand_function(12.0), HandFunction::ReducedDexterity);
    let (power, _, _, in_range) = effects::muscle_power_factor(33.0);
    assert!(power < 0.9 && in_range);
}
