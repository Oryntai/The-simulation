//! Печатная плата: сопротивление и нагрев дорожек, переходные отверстия, волновое
//! сопротивление микрополоска, температура переходов компонентов, свойства FR-4 и простое
//! описание платы с проверкой всех дорожек и деталей.
//!
//! Единицы — СИ (метры, амперы, °C для температур); формулы IPC-2221 внутри работают в милах,
//! как в стандарте (1 mil = 25.4 мкм).

use std::fmt;

pub mod clearance;
pub mod impedance;
pub mod kicad;
pub mod layout;
pub mod ratings;
pub mod return_path;

/// 1 mil = 0.001 дюйма, м.
pub const MIL: f64 = 25.4e-6;

/// Удельное сопротивление меди при 20 °C, Ом·м: 1.68e-8 — чистая медь (CRC Handbook of
/// Chemistry and Physics: 1.678 мкОм·см). Отожжённая медь по стандарту IACS (IEC 60028,
/// NBS Handbook 100 «Copper Wire Tables», 1966,
/// <https://nvlpubs.nist.gov/nistpubs/Legacy/hb/nbshandbook100.pdf>) — 1.7241e-8 (на 2.6% больше);
/// гальваническая медь печатных плат — до ~1.9e-8 (значение по умолчанию многих калькуляторов).
pub const CU_RHO_20C: f64 = 1.68e-8;

/// Температурный коэффициент сопротивления меди при 20 °C, 1/К (NBS Handbook 100, IEC 60028).
pub const CU_ALPHA: f64 = 0.00393;

/// Номинальная толщина фольги на 1 oz/ft², м: IPC-4562A — ½ oz 17.1 мкм, 1 oz 34.3 мкм,
/// 2 oz 68.6 мкм (минимум — на 10% меньше). Калькуляторы IPC-2221 обычно берут 1.378 mil =
/// 35 мкм. Внешние слои после металлизации толще (IPC-6012: 1 oz → ≥ 47.9 мкм, класс 2).
pub const FOIL_M_PER_OZ: f64 = 34.3e-6;

/// Толщина фольги, м, по массе на площадь в унциях на кв. фут.
pub fn oz_to_m(oz: f64) -> f64 {
    oz * FOIL_M_PER_OZ
}

/// Удельное сопротивление меди при температуре, Ом·м (линейная зависимость).
pub fn copper_resistivity(temp_c: f64) -> f64 {
    CU_RHO_20C * (1.0 + CU_ALPHA * (temp_c - 20.0))
}

/// Сопротивление прямоугольной дорожки, Ом.
pub fn trace_resistance(length: f64, width: f64, thickness: f64, temp_c: f64) -> f64 {
    copper_resistivity(temp_c) * length / (width * thickness)
}

/// Слой дорожки для формулы IPC-2221.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IpcLayer {
    External,
    Internal,
}

impl IpcLayer {
    /// Коэффициент k формулы IPC-2221 (§6.2): 0.048 — наружные, 0.024 — внутренние слои.
    pub fn k(self) -> f64 {
        match self {
            IpcLayer::External => 0.048,
            IpcLayer::Internal => 0.024,
        }
    }
}

/// IPC-2221 (по графикам NBS 1956 г.): допустимый ток, А, при сечении `area_mil2` (mil²)
/// и перегреве `delta_t` (К): I = k·ΔT^0.44·A^0.725.
///
/// Применимость графиков: ток до 35 А, ширина до 400 mil, ΔT 10–100 К, медь 0.5–3 oz;
/// за их пределами формула — экстраполяция. Известные расхождения с IPC-2152 (2009, новые
/// измерения; J. Olson, «Determining Circuit Board Current Carrying Capability»;
/// M. Jouppi, «The New IPC-2152», PCD&F, 2009):
/// * внутренний график 2221 — не измерение, а наружный, сниженный вдвое; реальные внутренние
///   дорожки греются почти как наружные или меньше → 2221 для внутренних сильно консервативен;
/// * наружный график 2221 мерился на проводнике в спокойном воздухе без учёта платы и
///   полигонов; на маленькой плате без полигонов он НЕ консервативен, с медным полигоном под
///   дорожкой — консервативен (сверки `pcb.ipc2221_*_vs_2152`).
pub fn ipc2221_current(area_mil2: f64, delta_t: f64, layer: IpcLayer) -> f64 {
    layer.k() * delta_t.powf(0.44) * area_mil2.powf(0.725)
}

/// Обратная формула: нужное сечение, mil².
pub fn ipc2221_area_mil2(current: f64, delta_t: f64, layer: IpcLayer) -> f64 {
    (current / (layer.k() * delta_t.powf(0.44))).powf(1.0 / 0.725)
}

/// Обратная формула: перегрев дорожки, К.
pub fn ipc2221_delta_t(current: f64, area_mil2: f64, layer: IpcLayer) -> f64 {
    (current / (layer.k() * area_mil2.powf(0.725))).powf(1.0 / 0.44)
}

/// Нужная ширина дорожки, м, при толщине меди `thickness` (м).
pub fn ipc2221_width(current: f64, delta_t: f64, thickness: f64, layer: IpcLayer) -> f64 {
    ipc2221_area_mil2(current, delta_t, layer) * MIL * MIL / thickness
}

/// Сопротивление металлизированного отверстия (полый цилиндр), Ом:
/// R = ρ·L / (π·(D − t)·t), D — диаметр отверстия (сверла), t — толщина металлизации.
/// Металлизация по IPC-6012: класс 2 — в среднем 20 мкм, класс 3 — 25 мкм.
pub fn via_resistance(drill: f64, plating: f64, length: f64, temp_c: f64) -> f64 {
    copper_resistivity(temp_c) * length / (std::f64::consts::PI * (drill - plating) * plating)
}

/// Результат расчёта микрополоска.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Microstrip {
    /// Волновое сопротивление, Ом.
    pub z0: f64,
    /// Эффективная диэлектрическая проницаемость (для IPC-2141 — не вычисляется, NaN).
    pub eps_eff: f64,
    /// Внутри ли диапазона применимости формулы.
    pub valid: bool,
}

/// IPC-2141: Z0 = 87/√(εr + 1.41)·ln(5.98h/(0.8w + t)).
/// Применимость: 0.1 < w/h < 2.0, 1 < εr < 15 (S. Hall, G. Hall, J. McCall, «High-Speed
/// Digital System Design», Wiley, 2000); в этом диапазоне для 50 Ом расхождение с
/// Хаммерстадом–Йенсеном < ~2%, вне его — до десятков процентов.
pub fn microstrip_ipc2141(w: f64, h: f64, t: f64, er: f64) -> Microstrip {
    let z0 = 87.0 / (er + 1.41).sqrt() * (5.98 * h / (0.8 * w + t)).ln();
    let u = w / h;
    Microstrip {
        z0,
        eps_eff: f64::NAN,
        valid: (0.1..2.0).contains(&u) && er > 1.0 && er < 15.0,
    }
}

/// Волновое сопротивление свободного пространства μ₀·c, Ом.
const ETA0: f64 = 376.730_313_412;

fn hj_z01(u: f64) -> f64 {
    let f = 6.0 + (std::f64::consts::TAU - 6.0) * (-(30.666 / u).powf(0.7528)).exp();
    ETA0 / std::f64::consts::TAU * (f / u + (1.0 + (2.0 / u).powi(2)).sqrt()).ln()
}

fn hj_eps_eff(u: f64, er: f64) -> f64 {
    let a = 1.0
        + ((u.powi(4) + (u / 52.0).powi(2)) / (u.powi(4) + 0.432)).ln() / 49.0
        + (1.0 + (u / 18.1).powi(3)).ln() / 18.7;
    let b = 0.564 * ((er - 0.9) / (er + 3.0)).powf(0.053);
    (er + 1.0) / 2.0 + (er - 1.0) / 2.0 * (1.0 + 10.0 / u).powf(-a * b)
}

/// Хаммерстад и Йенсен, «Accurate Models for Microstrip Computer-Aided Design», IEEE MTT-S
/// 1980, DOI 10.1109/MWSYM.1980.1124303 (квазистатика): точность Z0 в воздухе лучше 0.01%
/// при w/h ≤ 1 и 0.03% при w/h ≤ 1000, εeff — лучше 0.2% при εr < 128 и 0.01 ≤ w/h ≤ 100.
/// Поправка на толщину проводника — по тем же авторам (в изложении технической документации
/// Qucs, «Microstrip lines», <https://qucs.sourceforge.net/tech/node75.html>).
pub fn microstrip_hammerstad_jensen(w: f64, h: f64, t: f64, er: f64) -> Microstrip {
    let u = w / h;
    let (ur, u1) = if t > 0.0 {
        let tn = t / h;
        let coth = 1.0 / (6.517 * u).sqrt().tanh();
        let du1 =
            tn / std::f64::consts::PI * (1.0 + 4.0 * std::f64::consts::E / (tn * coth * coth)).ln();
        let sech = 1.0 / (er - 1.0).sqrt().cosh();
        let dur = 0.5 * du1 * (1.0 + sech);
        (u + dur, u + du1)
    } else {
        (u, u)
    };
    let e_r = hj_eps_eff(ur, er);
    let z0 = hj_z01(ur) / e_r.sqrt();
    let eps_eff = e_r * (hj_z01(u1) / hj_z01(ur)).powi(2);
    Microstrip {
        z0,
        eps_eff,
        valid: (0.01..=100.0).contains(&u) && er < 128.0,
    }
}

/// Температура перехода по JEDEC JESD51: Tj = Ta + P·θJA, °C. θJA зависит от платы
/// (до ±50% между тестовыми платами 1s и 2s2p — TI SPRA953D «Semiconductor and IC Package
/// Thermal Metrics», <https://www.ti.com/lit/an/spra953d/spra953d.pdf>), поэтому это оценка.
pub fn junction_temperature(t_ambient: f64, power: f64, theta_ja: f64) -> f64 {
    t_ambient + power * theta_ja
}

/// Корпус и его типовое θJA (неподвижный воздух, тестовая плата JEDEC, если не сказано иное).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Package {
    pub name: &'static str,
    /// °C/Вт.
    pub theta_ja: f64,
    pub source: &'static str,
}

pub const PACKAGES: &[Package] = &[
    Package {
        name: "SOT-23-3",
        theta_ja: 206.0,
        source: "TI TL431 SLVS543S §6.3 (DBZ)",
    },
    Package {
        name: "SOT-23-5",
        theta_ja: 206.0,
        source: "TI TL431 SLVS543S §6.3 (DBV)",
    },
    Package {
        name: "SOT-23-6 (силовая ИС)",
        theta_ja: 92.6,
        source: "TI TPS563201 SLVSD90B §5.4 (DDC)",
    },
    Package {
        name: "SOT-89",
        theta_ja: 52.0,
        source: "TI TL431 SLVS543S §6.3 (PK)",
    },
    Package {
        name: "SOIC-8",
        theta_ja: 97.0,
        source: "TI TL431 SLVS543S §6.3 (D)",
    },
    Package {
        name: "SOT-223",
        theta_ja: 61.6,
        source: "TI LM1117 SNOS412Q §7.4 (DCY)",
    },
    Package {
        name: "TO-252 (DPAK)",
        theta_ja: 45.1,
        source: "TI LM1117 SNOS412Q §7.4 (NDP)",
    },
    Package {
        name: "TO-263 (D2PAK)",
        theta_ja: 41.3,
        source: "TI LM1117 SNOS412Q §7.4 (KTT)",
    },
    Package {
        name: "TO-220, без радиатора",
        theta_ja: 62.0,
        source: "Infineon IRL540NPbF, Rev 1.0 (свободный воздух)",
    },
    Package {
        name: "VQFN-16 3×3",
        theta_ja: 45.0,
        source: "TI TPS62130 SLVSAG7F §7.4 (RGT)",
    },
    Package {
        name: "PowerPAK SO-8 (5×6)",
        theta_ja: 70.0,
        source: "Vishay SiRA10BDP, doc 76396: 70 °C/Вт max установившийся, плата 1\"×1\" FR-4",
    },
];

/// Найти корпус по имени (начало строки, без учёта регистра).
pub fn package(name: &str) -> Option<&'static Package> {
    let n = name.to_lowercase();
    PACKAGES
        .iter()
        .find(|p| p.name.to_lowercase().starts_with(&n))
}

/// Свойства стеклотекстолита.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Laminate {
    pub name: &'static str,
    /// Температура стеклования (DSC), °C.
    pub tg_c: f64,
    /// Температура разложения (5% потери массы), °C.
    pub td_c: f64,
    /// Диэлектрическая проницаемость на 1 ГГц.
    pub er_1ghz: f64,
    /// Тангенс угла потерь на 1 ГГц.
    pub loss_tangent_1ghz: f64,
    /// Теплопроводность поперёк слоёв, Вт/(м·К).
    pub k_through: f64,
    /// КТР по оси Z ниже/выше Tg, ppm/К.
    pub cte_z_ppm: (f64, f64),
    /// Предельная рабочая температура (UL 796 RTI), °C.
    pub max_operating_c: f64,
    pub source: &'static str,
}

/// Стандартный FR-4 (IPC-4101/21, Tg ≈ 135 °C): Kingboard KB-6160A,
/// <https://intectiv.si/wp-content/uploads/2023/08/Technical-data-sheet-KB-6160A-min.pdf>;
/// теплопроводность поперёк слоёв 0.29 Вт/(м·К) — Azar & Graebner, IEEE SEMI-THERM 1996;
/// RTI 130 °C — типовое значение UL 796 для FR-4 (даташиты Isola IS400/FR406/370HR).
pub const FR4_STANDARD: Laminate = Laminate {
    name: "FR-4 стандартный (KB-6160A, IPC-4101/21)",
    tg_c: 135.0,
    td_c: 305.0,
    er_1ghz: 4.25,
    loss_tangent_1ghz: 0.018,
    k_through: 0.29,
    cte_z_ppm: (60.0, 300.0),
    max_operating_c: 130.0,
    source: "Kingboard KB-6160A TDS; Azar & Graebner 1996; UL 796 RTI",
};

/// FR-4 с высокой Tg: Isola 370HR (IPC-4101/98/99/101/126),
/// <https://www.isola-group.com/wp-content/uploads/data-sheets/370hr.pdf>.
pub const FR4_HIGH_TG: Laminate = Laminate {
    name: "FR-4 high-Tg (Isola 370HR)",
    tg_c: 180.0,
    td_c: 340.0,
    er_1ghz: 4.17,
    loss_tangent_1ghz: 0.0161,
    k_through: 0.4,
    cte_z_ppm: (45.0, 230.0),
    max_operating_c: 130.0,
    source: "Isola 370HR data sheet, 04/2021",
};

// ───────────────────────────── Плата ─────────────────────────────

/// Слой платы.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TraceLayer {
    Top,
    Bottom,
    /// Внутренний слой с номером (1 — ближайший к верху).
    Inner(u8),
}

impl TraceLayer {
    pub fn ipc(self) -> IpcLayer {
        match self {
            TraceLayer::Top | TraceLayer::Bottom => IpcLayer::External,
            TraceLayer::Inner(_) => IpcLayer::Internal,
        }
    }
}

/// Дорожка: ломаная (точки в метрах в системе координат платы), ширина, слой, цепь, ток.
#[derive(Clone, Debug, PartialEq)]
pub struct Trace {
    pub points: Vec<[f64; 2]>,
    pub width: f64,
    pub layer: TraceLayer,
    pub net: String,
    /// Длительный (действующий) ток, А.
    pub current: f64,
}

impl Trace {
    pub fn length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
            .sum()
    }
}

/// Деталь: прямоугольный посадочный размер, центр, рассеиваемая мощность, тепловые данные.
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    pub name: String,
    /// Размер посадочного места (ширина, высота), м.
    pub footprint: [f64; 2],
    /// Центр, м.
    pub pos: [f64; 2],
    pub power_w: f64,
    /// °C/Вт.
    pub theta_ja: f64,
    /// Предельная температура перехода, °C (по даташиту детали; типично 125–150).
    pub tj_max_c: f64,
}

/// Плата.
#[derive(Clone, Debug, PartialEq)]
pub struct Board {
    pub width: f64,
    pub height: f64,
    pub thickness: f64,
    /// Медь наружных и внутренних слоёв, oz.
    pub copper_oz_outer: f64,
    pub copper_oz_inner: f64,
    pub laminate: Laminate,
    pub traces: Vec<Trace>,
    pub parts: Vec<Part>,
}

impl Board {
    /// Пустая двухслойная плата 1.6 мм, медь 1 oz, стандартный FR-4.
    pub fn new(width: f64, height: f64) -> Board {
        Board {
            width,
            height,
            thickness: 1.6e-3,
            copper_oz_outer: 1.0,
            copper_oz_inner: 0.5,
            laminate: FR4_STANDARD,
            traces: Vec::new(),
            parts: Vec::new(),
        }
    }

    fn inside(&self, p: [f64; 2]) -> bool {
        (0.0..=self.width).contains(&p[0]) && (0.0..=self.height).contains(&p[1])
    }
}

/// Правила проверки платы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DesignRules {
    /// Допустимый перегрев дорожки, К (частая практика — 10–20 К; оценка).
    pub max_trace_rise_c: f64,
    /// Допустимое падение напряжения на дорожке, В.
    pub max_trace_drop_v: f64,
}

impl Default for DesignRules {
    fn default() -> Self {
        DesignRules {
            max_trace_rise_c: 20.0,
            max_trace_drop_v: 0.1,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct TraceReport {
    pub index: usize,
    pub net: String,
    pub length: f64,
    /// Сечение, mil².
    pub area_mil2: f64,
    /// Перегрев по IPC-2221, К.
    pub delta_t: f64,
    pub temp_c: f64,
    /// Сопротивление при рабочей температуре, Ом.
    pub resistance: f64,
    pub voltage_drop: f64,
    pub power_w: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PartReport {
    pub name: String,
    pub tj_c: f64,
    pub margin_c: f64,
}

/// Замечание проверки.
#[derive(Clone, Debug, PartialEq)]
pub struct BoardWarning {
    /// «Дорожка N (цепь)» или имя детали.
    pub item: String,
    pub message: String,
}

impl fmt::Display for BoardWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.item, self.message)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoardReport {
    pub traces: Vec<TraceReport>,
    pub parts: Vec<PartReport>,
    pub warnings: Vec<BoardWarning>,
}

/// Проверить все дорожки (сопротивление, падение, перегрев по IPC-2221) и все детали
/// (температура перехода) при температуре окружающего воздуха `ambient_c`.
///
/// Допущения: каждая дорожка греется независимо (IPC-2221 не учитывает соседей и полигоны);
/// температура детали — по θJA, без взаимного нагрева деталей через плату.
pub fn evaluate(board: &Board, ambient_c: f64, rules: &DesignRules) -> BoardReport {
    let mut warnings = Vec::new();
    let mut traces = Vec::with_capacity(board.traces.len());
    for (k, tr) in board.traces.iter().enumerate() {
        let item = format!("дорожка {} ({})", k + 1, tr.net);
        let mut warn = |m: String| {
            warnings.push(BoardWarning {
                item: item.clone(),
                message: m,
            })
        };
        let oz = match tr.layer {
            TraceLayer::Top | TraceLayer::Bottom => board.copper_oz_outer,
            TraceLayer::Inner(_) => board.copper_oz_inner,
        };
        let thickness = oz_to_m(oz);
        let area_mil2 = tr.width * thickness / (MIL * MIL);
        let i = tr.current.abs();
        let delta_t = if i > 0.0 {
            ipc2221_delta_t(i, area_mil2, tr.layer.ipc())
        } else {
            0.0
        };
        let temp_c = ambient_c + delta_t;
        let length = tr.length();
        let resistance = trace_resistance(length, tr.width, thickness, temp_c);
        let voltage_drop = i * resistance;
        if tr.points.len() < 2 {
            warn("в дорожке меньше двух точек".into());
        }
        if tr.points.iter().any(|&p| !board.inside(p)) {
            warn("дорожка выходит за край платы".into());
        }
        if delta_t > rules.max_trace_rise_c {
            warn(format!(
                "перегрев {delta_t:.1} К > {:.0} К по IPC-2221: нужна ширина ≥ {:.2} мм",
                rules.max_trace_rise_c,
                ipc2221_width(i, rules.max_trace_rise_c, thickness, tr.layer.ipc()) * 1e3
            ));
        }
        if temp_c > board.laminate.max_operating_c {
            warn(format!(
                "температура дорожки {temp_c:.0} °C выше предельной для {} ({:.0} °C)",
                board.laminate.name, board.laminate.max_operating_c
            ));
        }
        if voltage_drop > rules.max_trace_drop_v {
            warn(format!(
                "падение напряжения {:.0} мВ > {:.0} мВ",
                voltage_drop * 1e3,
                rules.max_trace_drop_v * 1e3
            ));
        }
        if i > 35.0 || tr.width > 400.0 * MIL || delta_t > 100.0 {
            warn(
                "вне диапазона графиков IPC-2221 (ток ≤ 35 А, ширина ≤ 400 mil, ΔT ≤ 100 К) — \
                 экстраполяция"
                    .into(),
            );
        }
        traces.push(TraceReport {
            index: k,
            net: tr.net.clone(),
            length,
            area_mil2,
            delta_t,
            temp_c,
            resistance,
            voltage_drop,
            power_w: i * voltage_drop,
        });
    }
    let mut parts = Vec::with_capacity(board.parts.len());
    for p in &board.parts {
        let tj = junction_temperature(ambient_c, p.power_w, p.theta_ja);
        let margin = p.tj_max_c - tj;
        let mut warn = |m: String| {
            warnings.push(BoardWarning {
                item: p.name.clone(),
                message: m,
            })
        };
        if margin < 0.0 {
            warn(format!(
                "Tj = {tj:.0} °C выше предельной {:.0} °C: нужен радиатор, полигон или меньшая мощность",
                p.tj_max_c
            ));
        } else if margin < 15.0 {
            warn(format!(
                "Tj = {tj:.0} °C — запас до предела всего {margin:.0} К"
            ));
        }
        let half = [p.footprint[0] / 2.0, p.footprint[1] / 2.0];
        let corners = [
            [p.pos[0] - half[0], p.pos[1] - half[1]],
            [p.pos[0] + half[0], p.pos[1] + half[1]],
        ];
        if !corners.iter().all(|&c| board.inside(c)) {
            warn("деталь выходит за край платы".into());
        }
        parts.push(PartReport {
            name: p.name.clone(),
            tj_c: tj,
            margin_c: margin,
        });
    }
    BoardReport {
        traces,
        parts,
        warnings,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ipc2221_roundtrip() {
        for layer in [IpcLayer::External, IpcLayer::Internal] {
            let a = ipc2221_area_mil2(3.0, 10.0, layer);
            assert!((ipc2221_current(a, 10.0, layer) - 3.0).abs() < 1e-12);
            assert!((ipc2221_delta_t(3.0, a, layer) - 10.0).abs() < 1e-9);
        }
    }

    #[test]
    fn resistance_and_via() {
        // 1 oz, 1 мм × 100 мм при 20 °C: 1.68e-8·0.1/(1e-3·34.3e-6) ≈ 49 мОм.
        let r = trace_resistance(0.1, 1e-3, oz_to_m(1.0), 20.0);
        assert!((r - 1.68e-8 * 0.1 / (1e-3 * 34.3e-6)).abs() < 1e-15);
        let r100 = trace_resistance(0.1, 1e-3, oz_to_m(1.0), 100.0);
        assert!((r100 / r - (1.0 + 0.00393 * 80.0)).abs() < 1e-12);
        // Сверло 0.3 мм, металлизация 25 мкм, плата 1.6 мм: ≈ 1.25 мОм.
        let rv = via_resistance(0.3e-3, 25e-6, 1.6e-3, 20.0);
        assert!((rv - 1.245e-3).abs() < 0.01e-3, "{rv}");
    }

    #[test]
    fn board_evaluation_flags_problems() {
        let mut b = Board::new(0.05, 0.04);
        b.traces.push(Trace {
            points: vec![[0.005, 0.005], [0.045, 0.005], [0.045, 0.035]],
            width: 0.3e-3,
            layer: TraceLayer::Top,
            net: "VMOT".into(),
            current: 5.0,
        });
        b.traces.push(Trace {
            points: vec![[0.005, 0.01], [0.02, 0.01]],
            width: 2.0e-3,
            layer: TraceLayer::Top,
            net: "GND".into(),
            current: 1.0,
        });
        b.parts.push(Part {
            name: "U1 LDO".into(),
            footprint: [6.5e-3, 7e-3],
            pos: [0.02, 0.02],
            power_w: 1.5,
            theta_ja: package("SOT-223").unwrap().theta_ja,
            tj_max_c: 125.0,
        });
        let rep = evaluate(&b, 40.0, &DesignRules::default());
        assert!((rep.traces[0].length - 0.07).abs() < 1e-12);
        assert!(
            rep.warnings
                .iter()
                .any(|w| w.item.contains("VMOT") && w.message.contains("перегрев"))
        );
        assert!(
            !rep.warnings.iter().any(|w| w.item.contains("GND")),
            "{:?}",
            rep.warnings
        );
        assert!((rep.parts[0].tj_c - (40.0 + 1.5 * 61.6)).abs() < 1e-9);
        assert!(rep.warnings.iter().any(|w| w.item == "U1 LDO"));
    }
}
