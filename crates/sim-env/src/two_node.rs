//! Двухузловая модель терморегуляции человека Гагге (Gagge, Fobelets, Berglund 1986) в
//! редакции ASHRAE 55-2020 (Normative Appendix D) и стандартная эффективная температура SET.
//!
//! Тело — два концентрических узла: ядро и кожа. Управление: сосудорасширение/сужение
//! (кровоток кожи), потоотделение, дрожь. Нестационарный расчёт, шаг 1 мин (как в
//! эталонном коде ASHRAE) или мельче.
//!
//! Порт с Python-кода pythermalcomfort `two_nodes_gagge` (версия 4.8.1, MIT License,
//! © 2019 Federico Tartarini, https://github.com/CenterForTheBuiltEnvironment/pythermalcomfort),
//! который сам повторяет код ASHRAE 55. Отличия порта:
//! * внешняя работа учитывается один раз и в Вт/м² (в коде ASHRAE она вычитается ещё раз
//!   «в метах»); при W = 0 (все эталонные таблицы) результаты совпадают;
//! * масса тела и площадь поверхности — входные параметры (в ASHRAE 70 кг и 1.8258 м²);
//! * добавлены: журнал энергии, предупреждения о выходе из области применимости,
//!   непрерывная постановка той же модели с интегрированием РК4 (второй численный способ).
//!
//! Область применимости. Модель подогнана к опытам в помещении: 10–40 °C воздух и
//! радиационная температура, скорость воздуха до 2 м/с, 1–4 мет, 0–1.5 кло
//! (ASHRAE 55-2020, табл. 7.3.4, — пределы для SET). Сильный холод (дрожь, сужение сосудов
//! рук и ног), жара выше 40 °C, ветер выше 2 м/с — экстраполяция: модель выдаёт
//! предупреждение. Погружение в воду моделью НЕ описывается (теплообмен в воде в 50–100 раз
//! сильнее, а модель не знает про воду) — для воды см. [`crate::immersion`].
//!
//! Источники: Gagge A.P., Fobelets A.P., Berglund L.G. (1986) ASHRAE Trans. 92(2B):709–731;
//! Gagge A.P., Stolwijk J.A.J., Nishi Y. (1971) ASHRAE Trans. 77(1):247–262;
//! ANSI/ASHRAE Standard 55-2020, Appendix D.

use crate::convection::STEFAN_BOLTZMANN;

/// 1 мет в Вт/м² (как в коде ASHRAE 55 для SET).
pub const MET_W_M2: f64 = 58.2;
/// Удельная теплоёмкость тканей тела в модели, Вт·ч/(кг·К) (= 3492 Дж/(кг·К)).
pub const BODY_HEAT_CAPACITY_WH: f64 = 0.97;
/// Нейтральная температура кожи, °C.
pub const T_SKIN_NEUTRAL: f64 = 33.7;
/// Нейтральная температура ядра, °C.
pub const T_CORE_NEUTRAL: f64 = 36.8;
/// Нейтральный кровоток кожи, л/(ч·м²).
pub const SKIN_BLOOD_FLOW_NEUTRAL: f64 = 6.3;
/// Удельная теплота испарения пота в модели, Вт·ч/г.
pub const LATENT_WH_PER_G: f64 = 0.68;

/// Поза (влияет на долю излучающей поверхности тела).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Posture {
    /// Сидя: A_r/A_D = 0.70.
    Sitting,
    /// Стоя: A_r/A_D = 0.73.
    Standing,
}

/// Площадь поверхности тела по Дюбуа, м²: A = 0.202·m^0.425·h^0.725 (m — кг, h — м).
/// Источник: DuBois D., DuBois E.F. (1916) Arch. Intern. Med. 17:863–871.
pub fn body_surface_area_dubois(mass_kg: f64, height_m: f64) -> f64 {
    0.202 * mass_kg.powf(0.425) * height_m.powf(0.725)
}

/// Давление насыщения в модели Гагге, мм рт. ст. (формула Антуана из кода ASHRAE).
pub fn p_sat_torr(t_c: f64) -> f64 {
    (18.6686 - 4030.183 / (t_c + 235.0)).exp()
}

/// Входные данные двухузловой модели.
#[derive(Clone, Debug, PartialEq)]
pub struct TwoNodeInputs {
    /// Температура воздуха, °C.
    pub t_air_c: f64,
    /// Средняя радиационная температура, °C.
    pub t_mrt_c: f64,
    /// Скорость воздуха относительно тела, м/с.
    pub air_speed: f64,
    /// Относительная влажность, %.
    pub rh_percent: f64,
    /// Метаболизм, мет.
    pub met: f64,
    /// Теплоизоляция одежды, кло.
    pub clo: f64,
    /// Внешняя механическая работа, мет.
    pub work_met: f64,
    /// Масса тела, кг (в модели — теплоёмкость узлов).
    pub body_mass_kg: f64,
    /// Площадь поверхности тела, м².
    pub body_surface_area: f64,
    /// Атмосферное давление, Па.
    pub p_atm: f64,
    /// Поза.
    pub posture: Posture,
    /// Длительность пребывания, мин.
    pub exposure_min: f64,
    /// Шаг явной схемы, мин (в эталоне ASHRAE — 1 мин).
    pub dt_min: f64,
    /// Предел кровотока кожи, л/(ч·м²) (ASHRAE: 90).
    pub max_skin_blood_flow: f64,
    /// Предел потоотделения, г/(ч·м²) (ASHRAE: 500).
    pub max_sweating: f64,
    /// Критическая влажность кожи; `None` — по скорости воздуха и одежде, как в ASHRAE.
    pub w_max: Option<f64>,
    /// Начальное состояние; `None` — нейтральное (33.7/36.8 °C).
    pub initial: Option<TwoNodeState>,
    /// Как ограничивать долю регуляторного пота, когда влажность кожи достигла w_max.
    pub wettedness_cap: WettednessCap,
}

/// Вариант ограничения испарения при w = w_max.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WettednessCap {
    /// Как у Gagge et al. (1986) и в pythermalcomfort: p_rsw = (w_max − 0.06)/0.94 —
    /// обращение w = 0.06 + 0.94·p_rsw, SET непрерывна (по умолчанию).
    Gagge1986,
    /// Как в коде ASHRAE 55-2020: p_rsw = w_max/0.94 — даёт скачок SET на пределе
    /// (pythermalcomfort issue #489, исправлено в 4.7.0); нужен, чтобы воспроизвести таблицу
    /// ASHRAE для 4 кло.
    Ashrae55Code,
}

impl TwoNodeInputs {
    /// Условия эталонного расчёта ASHRAE 55 / pythermalcomfort: 70 кг, 1.8258 м², стоя,
    /// 101 325 Па, 59 шагов по 1 мин (цикл эталонного кода `while n < 60`, начиная с n = 1).
    pub fn ashrae55(
        t_air_c: f64,
        t_mrt_c: f64,
        air_speed: f64,
        rh_percent: f64,
        met: f64,
        clo: f64,
    ) -> Self {
        Self {
            t_air_c,
            t_mrt_c,
            air_speed,
            rh_percent,
            met,
            clo,
            work_met: 0.0,
            body_mass_kg: 70.0,
            body_surface_area: 1.8258,
            p_atm: 101_325.0,
            posture: Posture::Standing,
            exposure_min: 59.0,
            dt_min: 1.0,
            max_skin_blood_flow: 90.0,
            max_sweating: 500.0,
            w_max: None,
            initial: None,
            wettedness_cap: WettednessCap::Gagge1986,
        }
    }

    /// Те же условия для конкретного человека: масса и рост → площадь по Дюбуа.
    pub fn for_person(mut self, mass_kg: f64, height_m: f64) -> Self {
        self.body_mass_kg = mass_kg;
        self.body_surface_area = body_surface_area_dubois(mass_kg, height_m);
        self
    }

    /// Задать длительность и шаг.
    pub fn with_exposure(mut self, exposure_min: f64, dt_min: f64) -> Self {
        self.exposure_min = exposure_min;
        self.dt_min = dt_min;
        self
    }
}

/// Состояние узлов.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoNodeState {
    /// Температура ядра, °C.
    pub t_core_c: f64,
    /// Средняя температура кожи, °C.
    pub t_skin_c: f64,
    /// Кровоток кожи, л/(ч·м²).
    pub skin_blood_flow: f64,
    /// Доля массы тела в узле «кожа».
    pub alpha: f64,
    /// Испарение с кожи, Вт/м².
    pub e_skin: f64,
    /// Полный метаболизм (с дрожью), Вт/м².
    pub metabolic_w_m2: f64,
}

impl TwoNodeState {
    /// Нейтральное начальное состояние кода ASHRAE.
    pub fn neutral(met: f64) -> Self {
        Self {
            t_core_c: T_CORE_NEUTRAL,
            t_skin_c: T_SKIN_NEUTRAL,
            skin_blood_flow: SKIN_BLOOD_FLOW_NEUTRAL,
            alpha: 0.1,
            e_skin: 0.1 * met,
            metabolic_w_m2: met * MET_W_M2,
        }
    }
}

/// Отсчёт истории.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoNodeSample {
    /// Время от начала, мин.
    pub t_min: f64,
    /// Температура ядра, °C.
    pub t_core_c: f64,
    /// Температура кожи, °C.
    pub t_skin_c: f64,
    /// Влажность кожи (доля 0…1).
    pub wettedness: f64,
    /// Регуляторное потоотделение, г/(ч·м²).
    pub sweat_g_h_m2: f64,
    /// Дрожь, Вт/м².
    pub shivering_w_m2: f64,
    /// Кровоток кожи, л/(ч·м²).
    pub skin_blood_flow: f64,
}

/// Предупреждения об области применимости и о состоянии человека.
#[derive(Clone, Debug, PartialEq)]
pub enum TwoNodeWarning {
    /// Параметр вне пределов ASHRAE 55 для SET — результат — экстраполяция.
    OutsideValidity {
        /// Имя параметра.
        parameter: &'static str,
        /// Значение.
        value: f64,
        /// Допустимый диапазон.
        range: (f64, f64),
    },
    /// Холод: включилась дрожь > 0.5 мет — модель грубая (нет сосудистой реакции конечностей).
    ColdExtrapolation,
    /// Ядро ниже 35 °C — гипотермия (за пределами калибровки модели).
    Hypothermia,
    /// Ядро выше 38 °C — предел ISO 7933/ACGIH для неакклиматизированных.
    CoreAboveLimit,
    /// Достигнута критическая влажность кожи — пот стекает, испарение ограничено.
    WettednessCapped,
    /// Потоотделение упёрлось в предел.
    SweatingCapped,
}

/// Журнал энергии за расчёт, Дж на всё тело. Проверка: накопление = приход − расход.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyLedger {
    /// Теплопродукция (метаболизм с дрожью).
    pub metabolic_j: f64,
    /// Внешняя работа.
    pub work_j: f64,
    /// Сухие потери с поверхности одежды (конвекция + излучение), пересчитанные по T_cl.
    pub dry_j: f64,
    /// Испарение с кожи.
    pub evaporative_j: f64,
    /// Дыхание (явное + скрытое).
    pub respiratory_j: f64,
    /// Накопление по изменению температур узлов (c·m·[α·ΔT_sk + (1−α)·ΔT_cr]).
    pub storage_j: f64,
}

impl EnergyLedger {
    /// Невязка: приход − расход − накопление, Дж.
    pub fn residual_j(&self) -> f64 {
        self.metabolic_j
            - self.work_j
            - self.dry_j
            - self.evaporative_j
            - self.respiratory_j
            - self.storage_j
    }
}

/// Результат расчёта.
#[derive(Clone, Debug, PartialEq)]
pub struct TwoNodeOutput {
    /// Конечное состояние.
    pub state: TwoNodeState,
    /// Температура ядра в конце, °C.
    pub t_core_c: f64,
    /// Средняя температура кожи в конце, °C.
    pub t_skin_c: f64,
    /// Средняя температура тела, °C.
    pub t_body_c: f64,
    /// Влажность кожи w (0.06…w_max).
    pub wettedness: f64,
    /// Критическая влажность кожи w_max.
    pub w_max: f64,
    /// Регуляторное потоотделение (испаряемое), г/(ч·м²).
    pub sweat_g_h_m2: f64,
    /// Потоотделение всего тела, г/ч.
    pub sweat_g_h: f64,
    /// Дрожь, Вт/м².
    pub shivering_w_m2: f64,
    /// Кровоток кожи, л/(ч·м²).
    pub skin_blood_flow: f64,
    /// Испарение с кожи, Вт/м².
    pub e_skin: f64,
    /// Испарение регуляторного пота, Вт/м².
    pub e_rsw: f64,
    /// Максимально возможное испарение, Вт/м².
    pub e_max: f64,
    /// Сухие потери (конвекция + излучение), Вт/м².
    pub q_sensible: f64,
    /// Потери дыханием (явные + скрытые), Вт/м².
    pub q_respiration: f64,
    /// Стандартная эффективная температура SET, °C.
    pub set_c: f64,
    /// Испарённая вода за весь расчёт (кожа: пот + диффузия; дыхание), г. Стекающий
    /// (неиспарившийся) пот модель не считает — при w = w_max потеря воды занижена.
    pub water_loss_g: f64,
    /// История по шагам.
    pub history: Vec<TwoNodeSample>,
    /// Журнал энергии.
    pub energy: EnergyLedger,
    /// Предупреждения.
    pub warnings: Vec<TwoNodeWarning>,
}

/// Постоянные одного расчёта.
struct Consts {
    t_a: f64,
    t_r: f64,
    vp_torr: f64,
    met: f64,
    rm: f64,
    w_ext: f64,
    area: f64,
    mass: f64,
    r_clo: f64,
    f_a_cl: f64,
    lr: f64,
    i_cl: f64,
    w_max: f64,
    h_cc: f64,
    a_r: f64,
    max_bl: f64,
    max_sw: f64,
    cap: WettednessCap,
    t_body_neutral: f64,
}

/// Сухой теплообмен при данной температуре кожи.
#[derive(Clone, Copy, Debug)]
struct Dry {
    q_sensible: f64,
    t_cl: f64,
    h_r: f64,
}

/// Управляющие сигналы и испарение при данном состоянии.
#[derive(Clone, Copy, Debug)]
struct Controls {
    m_bl: f64,
    alpha: f64,
    m_rsw: f64,
    e_rsw: f64,
    e_max: f64,
    e_skin: f64,
    w: f64,
    shiver: f64,
    capped_w: bool,
    capped_sw: bool,
}

impl Consts {
    fn new(inp: &TwoNodeInputs) -> Self {
        let p_atm_rel = inp.p_atm / 101_325.0;
        let v = inp.air_speed.max(0.1);
        let clo = inp.clo.max(0.0);
        let i_cl = if clo > 0.0 { 0.45 } else { 1.0 };
        let w_max = match inp.w_max {
            Some(w) if w > 0.0 => w,
            _ => {
                if clo > 0.0 {
                    0.59 * v.powf(-0.08)
                } else {
                    0.38 * v.powf(-0.29)
                }
            }
        };
        let mut h_cc = (3.0 * p_atm_rel.powf(0.53)).max(8.600_001 * (v * p_atm_rel).powf(0.53));
        if inp.met > 0.85 {
            h_cc = h_cc.max(5.66 * (inp.met - 0.85).powf(0.39));
        }
        let vp_torr = inp.rh_percent / 100.0 * p_sat_torr(inp.t_air_c);
        Consts {
            t_a: inp.t_air_c,
            t_r: inp.t_mrt_c,
            vp_torr,
            met: inp.met,
            rm: (inp.met - inp.work_met) * MET_W_M2,
            w_ext: inp.work_met * MET_W_M2,
            area: inp.body_surface_area,
            mass: inp.body_mass_kg,
            r_clo: 0.155 * clo,
            f_a_cl: 1.0 + 0.15 * clo,
            lr: 2.2 / p_atm_rel,
            i_cl,
            w_max,
            h_cc,
            a_r: match inp.posture {
                Posture::Sitting => 0.70,
                Posture::Standing => 0.73,
            },
            max_bl: inp.max_skin_blood_flow,
            max_sw: inp.max_sweating,
            cap: inp.wettedness_cap,
            t_body_neutral: 0.1 * T_SKIN_NEUTRAL + 0.9 * T_CORE_NEUTRAL,
        }
    }

    /// Дыхание (явное и скрытое), Вт/м², как в коде ASHRAE: считается один раз от
    /// исходного метаболизма.
    fn respiration(&self) -> (f64, f64) {
        let m = self.met * MET_W_M2;
        (
            0.0023 * m * (44.0 - self.vp_torr),
            0.0014 * m * (34.0 - self.t_a),
        )
    }

    fn h_r(&self, t_cl: f64) -> f64 {
        4.0 * 0.95 * STEFAN_BOLTZMANN * ((t_cl + self.t_r) / 2.0 + 273.15).powi(3) * self.a_r
    }

    /// Итерация температуры поверхности одежды (допуск `tol`, К) и сухие потери.
    fn dry(&self, t_skin: f64, tol: f64) -> Dry {
        let mut h_r = 4.7;
        let h_t = h_r + self.h_cc;
        let mut r_a = 1.0 / (self.f_a_cl * h_t);
        let mut t_op = (h_r * self.t_r + self.h_cc * self.t_a) / h_t;
        let mut t_cl = (r_a * t_skin + self.r_clo * t_op) / (r_a + self.r_clo);
        for _ in 0..200 {
            h_r = self.h_r(t_cl);
            let h_t = h_r + self.h_cc;
            r_a = 1.0 / (self.f_a_cl * h_t);
            t_op = (h_r * self.t_r + self.h_cc * self.t_a) / h_t;
            let t_new = (r_a * t_skin + self.r_clo * t_op) / (r_a + self.r_clo);
            let done = (t_new - t_cl).abs() <= tol;
            t_cl = t_new;
            if done {
                break;
            }
        }
        Dry {
            q_sensible: (t_skin - t_op) / (r_a + self.r_clo),
            t_cl,
            h_r,
        }
    }

    /// Сухие потери, пересчитанные «снаружи» — по поверхности одежды:
    /// f_cl·[h_c·(T_cl − T_a) + h_r·(T_cl − T_r)]. Независимая от сопротивлений проверка.
    fn dry_at_clothing(&self, d: &Dry) -> f64 {
        self.f_a_cl * (self.h_cc * (d.t_cl - self.t_a) + d.h_r * (d.t_cl - self.t_r))
    }

    /// Управление при данном состоянии. `alpha_body` — доля массы кожи, по которой считается
    /// средняя температура тела (в коде ASHRAE — значение с прошлого шага).
    fn controls(&self, t_skin: f64, t_core: f64, alpha_body: f64) -> Controls {
        const C_SW: f64 = 170.0;
        const C_DIL: f64 = 120.0;
        const C_STR: f64 = 0.5;
        let t_body = alpha_body * t_skin + (1.0 - alpha_body) * t_core;
        let sk_sig = t_skin - T_SKIN_NEUTRAL;
        let warm_sk = sk_sig.max(0.0);
        let colds = (-sk_sig).max(0.0);
        let c_reg = t_core - T_CORE_NEUTRAL;
        let c_warm = c_reg.max(0.0);
        let c_cold = (-c_reg).max(0.0);
        let warm_b = (t_body - self.t_body_neutral).max(0.0);
        let m_bl = ((SKIN_BLOOD_FLOW_NEUTRAL + C_DIL * c_warm) / (1.0 + C_STR * colds))
            .min(self.max_bl)
            .max(0.5);
        let mut m_rsw = C_SW * warm_b * (warm_sk / 10.7).exp();
        let capped_sw = m_rsw > self.max_sw;
        m_rsw = m_rsw.min(self.max_sw);
        let mut e_rsw = LATENT_WH_PER_G * m_rsw;
        let r_ea = 1.0 / (self.lr * self.f_a_cl * self.h_cc);
        let r_ecl = self.r_clo / (self.lr * self.i_cl);
        let mut e_max = (p_sat_torr(t_skin) - self.vp_torr) / (r_ea + r_ecl);
        if e_max == 0.0 {
            e_max = 0.001;
        }
        let mut p_rsw = e_rsw / e_max;
        let mut w = 0.06 + 0.94 * p_rsw;
        let mut e_diff = w * e_max - e_rsw;
        let mut capped_w = false;
        if w > self.w_max {
            capped_w = true;
            w = self.w_max;
            p_rsw = match self.cap {
                WettednessCap::Gagge1986 => (self.w_max - 0.06) / 0.94,
                WettednessCap::Ashrae55Code => self.w_max / 0.94,
            };
            e_rsw = p_rsw * e_max;
            e_diff = 0.06 * (1.0 - p_rsw) * e_max;
        }
        if e_max < 0.0 {
            e_diff = 0.0;
            e_rsw = 0.0;
            w = self.w_max;
        }
        let e_skin = e_rsw + e_diff;
        m_rsw = e_rsw / LATENT_WH_PER_G;
        let shiver = 19.4 * colds * c_cold;
        Controls {
            m_bl,
            alpha: 0.041_773_7 + 0.745_183_3 / (m_bl + 0.585_417),
            m_rsw,
            e_rsw,
            e_max,
            e_skin,
            w,
            shiver,
            capped_w,
            capped_sw,
        }
    }

    /// Теплоёмкости узлов, Вт·ч/К.
    fn capacities(&self, alpha: f64) -> (f64, f64) {
        (
            BODY_HEAT_CAPACITY_WH * alpha * self.mass,
            BODY_HEAT_CAPACITY_WH * (1.0 - alpha) * self.mass,
        )
    }

    /// Мощности накопления в коже и ядре, Вт/м².
    fn storage(
        &self,
        t_skin: f64,
        t_core: f64,
        c: &Controls,
        m_total: f64,
        q_sens: f64,
    ) -> (f64, f64) {
        let (q_res, c_res) = self.respiration();
        let hf_cs = (t_core - t_skin) * (5.28 + 1.163 * c.m_bl);
        let s_core = m_total - self.w_ext - hf_cs - q_res - c_res;
        let s_skin = hf_cs - q_sens - c.e_skin;
        (s_skin, s_core)
    }
}

/// Допуск итерации T_cl, К (в коде ASHRAE — 0.01 К; берём строже, разница в SET < 0.01 К).
const T_CL_TOL: f64 = 1e-6;

/// Двухузловая модель: нестационарный расчёт явной схемой с шагом `dt_min`
/// (при dt = 1 мин и 59 шагах — ровно эталонный алгоритм ASHRAE 55).
pub fn two_node(inp: &TwoNodeInputs) -> TwoNodeOutput {
    let k = Consts::new(inp);
    let (q_res, c_res) = k.respiration();
    let mut st = inp
        .initial
        .unwrap_or_else(|| TwoNodeState::neutral(inp.met));
    assert!(inp.dt_min > 0.0, "шаг dt_min должен быть > 0");
    let steps = (inp.exposure_min / inp.dt_min).round().max(0.0) as usize;
    let dt_h = inp.dt_min / 60.0;
    let mut ledger = EnergyLedger::default();
    let mut history = Vec::with_capacity(steps + 1);
    let mut warnings = Vec::new();
    let mut last = k.controls(st.t_skin_c, st.t_core_c, st.alpha);
    // в коде ASHRAE до первого шага действуют начальные значения, а не функции состояния
    last.m_bl = st.skin_blood_flow;
    last.e_skin = st.e_skin;
    last.alpha = st.alpha;
    last.shiver = (st.metabolic_w_m2 - k.met * MET_W_M2).max(0.0);
    let mut m_total = st.metabolic_w_m2;
    let mut q_sens = 0.0;
    let mut dry = k.dry(st.t_skin_c, T_CL_TOL);
    let (mut any_cap_w, mut any_cap_sw, mut any_cold) = (false, false, false);
    let mut water_loss_g = 0.0;
    history.push(sample(0.0, &st, &last));
    for i in 0..steps {
        dry = k.dry(st.t_skin_c, T_CL_TOL);
        q_sens = dry.q_sensible;
        let (s_skin, s_core) = k.storage(st.t_skin_c, st.t_core_c, &last, m_total, q_sens);
        let (c_sk, c_cr) = k.capacities(st.alpha);
        let d_sk = s_skin * k.area * dt_h / c_sk;
        let d_cr = s_core * k.area * dt_h / c_cr;
        // журнал: потоки шага (Дж на тело) и накопление по изменению температур
        let sec = inp.dt_min * 60.0 * k.area;
        ledger.metabolic_j += m_total * sec;
        ledger.work_j += k.w_ext * sec;
        ledger.dry_j += k.dry_at_clothing(&dry) * sec;
        ledger.evaporative_j += last.e_skin * sec;
        ledger.respiratory_j += (q_res + c_res) * sec;
        ledger.storage_j += (c_sk * d_sk + c_cr * d_cr) * 3600.0;
        // испарённая вода: кожа (пот + диффузия) и скрытая часть дыхания
        water_loss_g += (last.e_skin + q_res) / LATENT_WH_PER_G * dt_h * k.area;
        st.t_skin_c += d_sk;
        st.t_core_c += d_cr;
        let c = k.controls(st.t_skin_c, st.t_core_c, st.alpha);
        any_cap_w |= c.capped_w;
        any_cap_sw |= c.capped_sw;
        any_cold |= c.shiver > 0.5 * MET_W_M2;
        m_total = k.rm + k.w_ext + c.shiver;
        st.skin_blood_flow = c.m_bl;
        st.alpha = c.alpha;
        st.e_skin = c.e_skin;
        st.metabolic_w_m2 = m_total;
        last = c;
        history.push(sample((i + 1) as f64 * inp.dt_min, &st, &last));
    }
    let set_c = standard_effective_temperature(&k, inp, &st, &last, q_sens, dry.h_r);
    collect_warnings(inp, &st, any_cap_w, any_cap_sw, any_cold, &mut warnings);
    TwoNodeOutput {
        state: st,
        t_core_c: st.t_core_c,
        t_skin_c: st.t_skin_c,
        t_body_c: st.alpha * st.t_skin_c + (1.0 - st.alpha) * st.t_core_c,
        wettedness: last.w,
        w_max: k.w_max,
        sweat_g_h_m2: last.m_rsw,
        sweat_g_h: last.m_rsw * k.area,
        shivering_w_m2: last.shiver,
        skin_blood_flow: last.m_bl,
        e_skin: last.e_skin,
        e_rsw: last.e_rsw,
        e_max: last.e_max,
        q_sensible: q_sens,
        q_respiration: q_res + c_res,
        set_c,
        water_loss_g,
        history,
        energy: ledger,
        warnings,
    }
}

fn sample(t_min: f64, st: &TwoNodeState, c: &Controls) -> TwoNodeSample {
    TwoNodeSample {
        t_min,
        t_core_c: st.t_core_c,
        t_skin_c: st.t_skin_c,
        wettedness: c.w,
        sweat_g_h_m2: c.m_rsw,
        shivering_w_m2: c.shiver,
        skin_blood_flow: c.m_bl,
    }
}

fn collect_warnings(
    inp: &TwoNodeInputs,
    st: &TwoNodeState,
    cap_w: bool,
    cap_sw: bool,
    cold: bool,
    w: &mut Vec<TwoNodeWarning>,
) {
    let mut check = |parameter: &'static str, value: f64, range: (f64, f64)| {
        if value < range.0 || value > range.1 {
            w.push(TwoNodeWarning::OutsideValidity {
                parameter,
                value,
                range,
            });
        }
    };
    check("температура воздуха, °C", inp.t_air_c, VALIDITY_T_AIR);
    check("радиационная температура, °C", inp.t_mrt_c, VALIDITY_T_MRT);
    check("скорость воздуха, м/с", inp.air_speed, VALIDITY_AIR_SPEED);
    check("метаболизм, мет", inp.met, VALIDITY_MET);
    check("одежда, кло", inp.clo, VALIDITY_CLO);
    if cold {
        w.push(TwoNodeWarning::ColdExtrapolation);
    }
    if st.t_core_c < crate::strain::HYPOTHERMIA_C {
        w.push(TwoNodeWarning::Hypothermia);
    }
    if st.t_core_c > crate::strain::CORE_LIMIT_UNACCLIMATISED_C {
        w.push(TwoNodeWarning::CoreAboveLimit);
    }
    if cap_w {
        w.push(TwoNodeWarning::WettednessCapped);
    }
    if cap_sw {
        w.push(TwoNodeWarning::SweatingCapped);
    }
}

/// Пределы применимости SET по ASHRAE 55-2020 (табл. 7.3.4): воздух, °C.
pub const VALIDITY_T_AIR: (f64, f64) = (10.0, 40.0);
/// То же: радиационная температура, °C.
pub const VALIDITY_T_MRT: (f64, f64) = (10.0, 40.0);
/// То же: скорость воздуха, м/с.
pub const VALIDITY_AIR_SPEED: (f64, f64) = (0.0, 2.0);
/// То же: метаболизм, мет.
pub const VALIDITY_MET: (f64, f64) = (1.0, 4.0);
/// То же: одежда, кло.
pub const VALIDITY_CLO: (f64, f64) = (0.0, 1.5);

/// SET: температура стандартной среды (RH 50 %, v ≈ 0.1 м/с, T_r = T_a, стандартная
/// одежда для данного метаболизма), в которой человек с той же температурой и влажностью
/// кожи теряет то же тепло. Алгоритм кода ASHRAE 55.
fn standard_effective_temperature(
    k: &Consts,
    inp: &TwoNodeInputs,
    st: &TwoNodeState,
    c: &Controls,
    q_sens: f64,
    h_r_last: f64,
) -> f64 {
    let p_atm_rel = inp.p_atm / 101_325.0;
    let q_skin = q_sens + c.e_skin;
    let p_s_sk = p_sat_torr(st.t_skin_c);
    let h_r_s = h_r_last;
    let mut h_c_s = 3.0 * p_atm_rel.powf(0.53);
    if inp.met > 0.85 {
        h_c_s = h_c_s.max(5.66 * (inp.met - 0.85).powf(0.39));
    }
    h_c_s = h_c_s.max(3.0);
    let h_t_s = h_c_s + h_r_s;
    // в коде ASHRAE: (MET − WME/METFACTOR) — сохраняем как есть
    let r_clo_s = 1.52 / ((inp.met - inp.work_met / MET_W_M2) + 0.6944) - 0.1835;
    let r_cl_s = 0.155 * r_clo_s;
    let f_a_cl_s = 1.0 + 0.25 * r_clo_s;
    let f_cl_s = 1.0 / (1.0 + 0.155 * f_a_cl_s * h_t_s * r_clo_s);
    let i_m_s = 0.45;
    let i_cl_s = i_m_s * h_c_s / h_t_s * (1.0 - f_cl_s) / (h_c_s / h_t_s - f_cl_s * i_m_s);
    let r_a_s = 1.0 / (f_a_cl_s * h_t_s);
    let r_ea_s = 1.0 / (k.lr * f_a_cl_s * h_c_s);
    let r_ecl_s = r_cl_s / (k.lr * i_cl_s);
    let h_d_s = 1.0 / (r_a_s + r_cl_s);
    let h_e_s = 1.0 / (r_ea_s + r_ecl_s);
    let f = |set: f64| {
        q_skin - h_d_s * (st.t_skin_c - set) - c.w * h_e_s * (p_s_sk - 0.5 * p_sat_torr(set))
    };
    // секущая с малым приращением, как в ASHRAE, но до сходимости 1e-8
    let delta = 1e-4;
    let mut set = st.t_skin_c - q_skin / h_d_s;
    for _ in 0..100 {
        let e1 = f(set);
        let e2 = f(set + delta);
        let next = set - delta * e1 / (e2 - e1);
        let dx = next - set;
        set = next;
        if dx.abs() < 1e-8 {
            break;
        }
    }
    set
}

/// Та же модель в непрерывной постановке (сигналы управления — функции текущего состояния),
/// проинтегрированная классическим РК4 с шагом `dt_min`. Второй численный способ для явной
/// схемы ASHRAE (шаг 1 мин): различие показывает ошибку дискретизации эталонного кода.
/// Возвращает (T_ядра, T_кожи) в конце.
pub fn two_node_rk4(inp: &TwoNodeInputs, dt_min: f64) -> (f64, f64) {
    let k = Consts::new(inp);
    let rhs = |t_sk: f64, t_cr: f64| -> (f64, f64) {
        let c = k.controls(t_sk, t_cr, alpha_fixed_point(&k, t_sk, t_cr));
        let m_total = k.rm + k.w_ext + c.shiver;
        let q = k.dry(t_sk, 1e-10).q_sensible;
        let (s_sk, s_cr) = k.storage(t_sk, t_cr, &c, m_total, q);
        let (c_sk, c_cr) = k.capacities(c.alpha);
        // К/мин
        (s_sk * k.area / c_sk / 60.0, s_cr * k.area / c_cr / 60.0)
    };
    let init = inp
        .initial
        .unwrap_or_else(|| TwoNodeState::neutral(inp.met));
    let (mut sk, mut cr) = (init.t_skin_c, init.t_core_c);
    assert!(dt_min > 0.0, "шаг dt_min должен быть > 0");
    let steps = (inp.exposure_min / dt_min).round().max(0.0) as usize;
    let h = dt_min;
    for _ in 0..steps {
        let k1 = rhs(sk, cr);
        let k2 = rhs(sk + 0.5 * h * k1.0, cr + 0.5 * h * k1.1);
        let k3 = rhs(sk + 0.5 * h * k2.0, cr + 0.5 * h * k2.1);
        let k4 = rhs(sk + h * k3.0, cr + h * k3.1);
        sk += h / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
        cr += h / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
    }
    (cr, sk)
}

/// α как функция состояния: α = f(m_bl), а m_bl зависит только от сигналов ядра и кожи.
fn alpha_fixed_point(k: &Consts, t_sk: f64, t_cr: f64) -> f64 {
    // m_bl не зависит от α, поэтому одной подстановки достаточно
    k.controls(t_sk, t_cr, 0.1).alpha
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neutral_room_is_near_neutral() {
        let out = two_node(&TwoNodeInputs::ashrae55(25.0, 25.0, 0.15, 50.0, 1.0, 0.5));
        assert!((out.set_c - 23.8).abs() < 0.2, "{}", out.set_c);
        assert!((out.t_core_c - 36.8).abs() < 0.3);
        assert!(out.warnings.is_empty(), "{:?}", out.warnings);
    }

    #[test]
    fn dubois_area() {
        // 70 кг, 1.75 м → ≈ 1.85 м²
        assert!((body_surface_area_dubois(70.0, 1.75) - 1.846).abs() < 0.005);
    }

    #[test]
    fn cold_flags_extrapolation() {
        let mut inp = TwoNodeInputs::ashrae55(-10.0, -10.0, 5.0, 50.0, 1.2, 1.0);
        inp.exposure_min = 120.0;
        let out = two_node(&inp);
        assert!(
            out.warnings
                .iter()
                .any(|w| matches!(w, TwoNodeWarning::OutsideValidity { .. }))
        );
        assert!(out.shivering_w_m2 > 0.0);
    }
}
