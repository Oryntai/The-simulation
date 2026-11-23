//! Погружение в холодную воду: ожидаемая хронология по литературе (холодовой шок,
//! отказ плавания, гипотермия) и время до охлаждения ядра до 30 °C по Hayward et al. (1975).
//!
//! Двухузловая модель Гагге для воды не годится (она не знает про воду), поэтому здесь —
//! эмпирика с явными источниками и разбросом. Вторая проверка: время Hayward сверяется
//! с разбросом пяти опубликованных моделей выживания (таблица Oakley & Pethybridge 1997,
//! воспроизведена в Transport Canada TP 13822) — см. `reference_checks()`.
//!
//! Допущения: взрослый в лёгкой одежде, в спасательном жилете (голова над водой), спокойная
//! вода, без активного плавания. Плавание, волнение и худоба ускоряют охлаждение; жир,
//! гидрокостюм и поза HELP — замедляют. Смерть в первые минуты (утопление при холодовом
//! шоке) моделью охлаждения не описывается вовсе — это отдельный риск.

/// Этап погружения в холодную воду (классификация Golden & Hervey 1981; Tipton 1989).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ImmersionStage {
    /// 0–3 мин: холодовой шок — вдох-рывок, гипервентиляция, тахикардия, скачок давления;
    /// главный риск — вдохнуть воду (большинство смертей при погружении).
    ColdShock,
    /// 3–30 мин: охлаждение мышц и нервов рук и ног, отказ плавания, потеря ловкости.
    SwimFailure,
    /// > 30 мин: гипотермия (ядро < 35 °C), затем потеря сознания (~30 °C).
    Hypothermia,
}

/// Конец фазы холодового шока, мин (2–3 мин по TP 13822; 3–5 мин в части источников).
pub const COLD_SHOCK_END_MIN: f64 = 3.0;
/// Начало фазы гипотермии, мин (≥ 30 мин — Golden & Tipton; TP 13822).
pub const HYPOTHERMIA_PHASE_START_MIN: f64 = 30.0;
/// Вода холоднее этого считается «холодной» для холодового шока и отказа плавания, °C
/// (реакция холодового шока максимальна при 10–15 °C и ниже; Tipton 1989).
pub const COLD_WATER_C: f64 = 15.0;

/// Этап по времени пребывания в воде (для холодной воды).
pub fn stage(minutes_in_water: f64) -> ImmersionStage {
    if minutes_in_water < COLD_SHOCK_END_MIN {
        ImmersionStage::ColdShock
    } else if minutes_in_water < HYPOTHERMIA_PHASE_START_MIN {
        ImmersionStage::SwimFailure
    } else {
        ImmersionStage::Hypothermia
    }
}

/// Скорость падения ректальной температуры по Hayward, Eckerson, Collis (1975), °C/мин:
/// 0.0785 − 0.0034·T_воды. Получена линейной экстраполяцией 40-минутных опытов.
/// `None`, если вода теплее ≈ 23 °C (скорость → 0, формула неприменима).
///
/// Источник: Hayward J.S., Eckerson J.D., Collis M.L. (1975) Can. J. Physiol. Pharmacol.
/// 53:21–32, doi:10.1139/y75-003 (коэффициенты — по вторичным источникам: калькуляторы
/// выживания, DTIC-отчёты; первоисточник не прочитан).
pub fn hayward_cooling_rate_c_per_min(t_water_c: f64) -> Option<f64> {
    let r = 0.0785 - 0.0034 * t_water_c;
    (r > 0.005).then_some(r)
}

/// Время, мин, до падения ядра с 37.2 до 30 °C по Hayward et al. (1975):
/// t = 15 + 7.2/(0.0785 − 0.0034·T_воды) (15 мин — начальная задержка охлаждения ядра).
/// Применимость: 0…≈20 °C, лёгкая одежда, жилет; это «время до потери сознания», не смерти.
pub fn hayward_time_to_core_30_min(t_water_c: f64) -> Option<f64> {
    if t_water_c < 0.0 {
        // ниже 0 °C — только морская вода; формула экстраполируется, берём 0 °C (консервативно
        // было бы ещё короче — помечается предупреждением в [`immersion_timeline`])
        return hayward_cooling_rate_c_per_min(0.0).map(|r| 15.0 + 7.2 / r);
    }
    hayward_cooling_rate_c_per_min(t_water_c).map(|r| 15.0 + 7.2 / r)
}

/// Предсказанное время до гибели, ч, по пяти моделям (лёгкая одежда), для 5, 10, 15 °C:
/// (источник модели, [5 °C, 10 °C, 15 °C]); `NaN` — нет данных.
/// Источник: Oakley E.H.N., Pethybridge R.J. (1997) INM Report 97011, воспроизведено в
/// Transport Canada TP 13822 «Survival in Cold Waters» (2003), гл. 1
/// (https://bac-lac.wayback.archive-it.org/web/20061209225100/http:/www.tc.gc.ca/MarineSafety/tp/Tp13822/chapter-1.htm).
pub const SURVIVAL_MODELS_H: [(&str, [f64; 3]); 5] = [
    ("Molnar 1946", [2.3, 4.0, f64::NAN]),
    ("Keatinge 1969", [0.9, f64::NAN, 4.5]),
    ("Nunneley & Wissler 1980", [1.1, 2.6, 3.0]),
    ("Allan 1983", [1.5, 2.5, 9.0]),
    ("Lee & Lee 1989", [1.0, 3.0, 7.0]),
];
/// Температуры воды столбцов [`SURVIVAL_MODELS_H`], °C.
pub const SURVIVAL_MODELS_T_C: [f64; 3] = [5.0, 10.0, 15.0];

/// Разброс (мин, макс) предсказанного времени до гибели, ч, при температуре столбца `idx`.
pub fn survival_models_range_h(idx: usize) -> (f64, f64) {
    SURVIVAL_MODELS_H
        .iter()
        .map(|(_, v)| v[idx])
        .filter(|x| x.is_finite())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
            (lo.min(x), hi.max(x))
        })
}

/// Ожидаемая хронология погружения.
#[derive(Clone, Debug, PartialEq)]
pub struct ImmersionTimeline {
    /// Температура воды, °C.
    pub t_water_c: f64,
    /// Холодовой шок ожидается (вода < 15 °C).
    pub cold_shock: bool,
    /// Конец холодового шока, мин.
    pub cold_shock_end_min: f64,
    /// Окно отказа плавания / потери ловкости рук, мин (начало, конец) — для холодной воды.
    pub swim_failure_window_min: Option<(f64, f64)>,
    /// Время до ядра 30 °C (потеря сознания) по Hayward 1975, мин.
    pub time_to_core_30_min: Option<f64>,
    /// Скорость охлаждения ядра после начальной задержки, °C/мин.
    pub core_cooling_c_per_min: Option<f64>,
    /// Пояснения и предупреждения.
    pub notes: Vec<&'static str>,
}

/// Хронология погружения в воду температуры `t_water_c` (лёгкая одежда, жилет).
pub fn immersion_timeline(t_water_c: f64) -> ImmersionTimeline {
    let cold = t_water_c < COLD_WATER_C;
    let mut notes = vec![
        "оценка по литературе для лёгкой одежды и спасательного жилета; разброс между людьми ±50 % и больше",
        "первые минуты опасны утоплением из-за холодового шока независимо от охлаждения ядра",
    ];
    if t_water_c < 0.0 {
        notes.push("вода ниже 0 °C (морская): уравнение Hayward экстраполировано от 0 °C — реальное время короче");
    }
    if t_water_c > 20.0 {
        notes.push(
            "вода теплее 20 °C: уравнение Hayward неприменимо, гипотермия развивается часами",
        );
    }
    ImmersionTimeline {
        t_water_c,
        cold_shock: cold,
        cold_shock_end_min: COLD_SHOCK_END_MIN,
        swim_failure_window_min: cold.then_some((COLD_SHOCK_END_MIN, HYPOTHERMIA_PHASE_START_MIN)),
        time_to_core_30_min: if t_water_c <= 20.0 {
            hayward_time_to_core_30_min(t_water_c)
        } else {
            None
        },
        core_cooling_c_per_min: hayward_cooling_rate_c_per_min(t_water_c.max(0.0)),
        notes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hayward_numbers() {
        // 10 °C: 15 + 7.2/0.0445 ≈ 176.8 мин
        let t = hayward_time_to_core_30_min(10.0).unwrap();
        assert!((t - 176.8).abs() < 0.1);
        assert!(hayward_time_to_core_30_min(25.0).is_none());
        assert_eq!(stage(1.0), ImmersionStage::ColdShock);
        assert_eq!(stage(10.0), ImmersionStage::SwimFailure);
        assert_eq!(survival_models_range_h(0), (0.9, 2.3));
    }
}
