//! Контакт кожи с материалами прибора: температура контакта, ожог от горячего,
//! боль, онемение и обморожение от холодного. Главное для экзоскелета: металлическая рама
//! и манжеты касаются кожи на морозе или на солнце.
//!
//! Три способа расчёта температуры контакта:
//! 1. **Два полубесконечных тела** (тепловая активность e = √(k·ρ·c)):
//!    T_c = (e_s·T_s + e_m·T_m)/(e_s + e_m) — устанавливается мгновенно и не меняется.
//! 2. **Слой на подложке** (тонкий пластик на алюминии) — точное аналитическое решение
//!    (метод преобразования Лапласа, ряд по отражениям):
//!    T_c(t) − T_m = (T_s − T_m)·e_s/(e_s + e_1)·[1 + (1 + β)·Σ_{n≥1} β^{n−1}·Γ^n·erfc(n·L/√(α₁t))],
//!    Γ = (e_1 − e_2)/(e_1 + e_2), β = (e_1 − e_s)/(e_1 + e_s). При t → 0 — как у слоя,
//!    при t → ∞ — как у подложки. Вывод — в комментарии к [`layered_contact_temperature`].
//! 3. **Численно**: одномерная нестационарная теплопроводность (метод конечных объёмов,
//!    Кранк–Николсон, старт Раннахера), многослойная, с перфузией кожи (Pennes) по желанию.
//!    Сверяется со способами 1 и 2 и по сходимости на сетках.
//!
//! Допущения: идеальный тепловой контакт (без контактного сопротивления — для холодного
//! металла это консервативно), сухая кожа, одномерная задача (площадь контакта много больше
//! глубины прогрева: за 10 с прогрев кожи ≈ 1 мм), постоянные свойства.

use crate::air::bisect;

/// Свойства материала.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialProps {
    /// Название.
    pub name: &'static str,
    /// Теплопроводность, Вт/(м·К).
    pub k: f64,
    /// Плотность, кг/м³.
    pub rho: f64,
    /// Удельная теплоёмкость, Дж/(кг·К).
    pub c: f64,
    /// Класс материала по ISO 13732-1 (для порогов ожога).
    pub class: ContactClass,
    /// Источник данных.
    pub source: &'static str,
}

impl MaterialProps {
    /// Тепловая активность (effusivity) e = √(k·ρ·c), Вт·с^½/(м²·К).
    pub fn effusivity(&self) -> f64 {
        (self.k * self.rho * self.c).sqrt()
    }
    /// Температуропроводность α = k/(ρ·c), м²/с.
    pub fn diffusivity(&self) -> f64 {
        self.k / (self.rho * self.c)
    }
}

/// Классы поверхностей ISO 13732-1 / EN 13202 (пороги ожога различаются по классам).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactClass {
    /// Металл без покрытия.
    UncoatedMetal,
    /// Окрашенный (лакированный) металл.
    PaintedMetal,
    /// Эмалированный металл.
    EnamelledMetal,
    /// Керамика, стекло, камень.
    CeramicGlassStone,
    /// Пластмассы.
    Plastics,
    /// Дерево.
    Wood,
    /// Живая ткань (кожа) — порогов ожога «по материалу» нет.
    Tissue,
}

/// Материалы мастерской.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Material {
    /// Алюминий 6061-T6.
    Aluminium6061,
    /// Нержавеющая сталь AISI 304.
    StainlessSteel304,
    /// Титан Ti-6Al-4V.
    Titanium6Al4V,
    /// Углепластик (поперёк слоёв).
    CarbonFibreComposite,
    /// ABS.
    Abs,
    /// PLA (сплошная печать).
    Pla,
    /// PETG (сплошная печать).
    Petg,
    /// Полиамид PA12.
    Pa12,
    /// Термопластичный полиуретан TPU.
    Tpu,
    /// Силиконовая резина.
    SiliconeRubber,
    /// Неопрен сплошной.
    NeopreneSolid,
    /// Неопрен вспененный (как в гидрокостюмах).
    NeopreneFoam,
    /// Дерево (среднее).
    Wood,
    /// Стекло.
    Glass,
    /// Лёд (0 °C).
    Ice,
    /// Вода.
    Water,
    /// Кожа человека.
    Skin,
}

/// Все материалы (для интерфейса и проверок).
pub const ALL_MATERIALS: [Material; 17] = [
    Material::Aluminium6061,
    Material::StainlessSteel304,
    Material::Titanium6Al4V,
    Material::CarbonFibreComposite,
    Material::Abs,
    Material::Pla,
    Material::Petg,
    Material::Pa12,
    Material::Tpu,
    Material::SiliconeRubber,
    Material::NeopreneSolid,
    Material::NeopreneFoam,
    Material::Wood,
    Material::Glass,
    Material::Ice,
    Material::Water,
    Material::Skin,
];

const ASM: &str = "ASM Aerospace Specification Metals (MatWeb), паспорт сплава";
const CENELEC: &str = "CENELEC Guide 29 (2007) / IEC Guide 117, табл. C.1";
const ESTIMATE: &str = "оценка: типичные паспортные данные производителей, разброс ±30 %";

impl Material {
    /// Теплофизические свойства.
    pub fn props(self) -> MaterialProps {
        use ContactClass::*;
        let m = |name, k, rho, c, class, source| MaterialProps {
            name,
            k,
            rho,
            c,
            class,
            source,
        };
        match self {
            Material::Aluminium6061 => {
                m("алюминий 6061-T6", 167.0, 2700.0, 896.0, UncoatedMetal, ASM)
            }
            Material::StainlessSteel304 => m(
                "нержавеющая сталь 304",
                16.2,
                8000.0,
                500.0,
                UncoatedMetal,
                ASM,
            ),
            Material::Titanium6Al4V => m("титан Ti-6Al-4V", 6.7, 4430.0, 526.0, UncoatedMetal, ASM),
            Material::CarbonFibreComposite => m(
                "углепластик (поперёк слоёв)",
                0.8,
                1550.0,
                1000.0,
                // по тепловой активности (~1100) ближе к стеклу, чем к пластмассам —
                // по CENELEC Guide 29 §6.2 берём класс с той же активностью (консервативно)
                CeramicGlassStone,
                "оценка: поперёк слоёв k = 0.5–1.0, вдоль 5–10 Вт/(м·К)",
            ),
            Material::Abs => m("ABS", 0.18, 1040.0, 1510.0, Plastics, CENELEC),
            Material::Pla => m(
                "PLA (100 % заполнение)",
                0.13,
                1240.0,
                1800.0,
                Plastics,
                "Farah S. et al. (2016) Adv. Drug Deliv. Rev. 107:367, doi:10.1016/j.addr.2016.06.012; c — оценка",
            ),
            Material::Petg => m(
                "PETG (100 % заполнение)",
                0.20,
                1270.0,
                1200.0,
                Plastics,
                ESTIMATE,
            ),
            Material::Pa12 => m(
                "полиамид PA12",
                0.21,
                1010.0,
                2100.0,
                Plastics,
                "k и c — для полиамидов по CENELEC Guide 29 табл. C.1; ρ — PA12",
            ),
            Material::Tpu => m("TPU", 0.19, 1200.0, 1800.0, Plastics, ESTIMATE),
            Material::SiliconeRubber => m(
                "силиконовая резина",
                0.25,
                1150.0,
                1300.0,
                Plastics,
                ESTIMATE,
            ),
            Material::NeopreneSolid => {
                m("неопрен сплошной", 0.19, 1230.0, 2140.0, Plastics, ESTIMATE)
            }
            Material::NeopreneFoam => m(
                "неопрен вспененный",
                0.05,
                170.0,
                1800.0,
                Plastics,
                ESTIMATE,
            ),
            Material::Wood => m("дерево (среднее)", 0.18, 660.0, 1720.0, Wood, CENELEC),
            Material::Glass => m("стекло", 0.88, 2600.0, 670.0, CeramicGlassStone, CENELEC),
            Material::Ice => m(
                "лёд, 0 °C",
                2.22,
                917.0,
                2050.0,
                CeramicGlassStone,
                "справочник CRC Handbook of Chemistry and Physics",
            ),
            // CENELEC Guide 29 / ISO 13732-1: порог ожога горячей водой — нижняя граница
            // разброса для металла без покрытия
            Material::Water => m("вода", 0.60, 1000.0, 4190.0, UncoatedMetal, CENELEC),
            Material::Skin => m(
                "кожа",
                0.37,
                1109.0,
                3391.0,
                Tissue,
                "IT'IS Foundation Tissue Properties Database v4.1, doi:10.13099/VIP21000-04-1",
            ),
        }
    }
}

/// Нормальная температура кожи кисти при контакте с горячим, °C (как у термоэстезиометра
/// ISO 13732-1 / Marzetta 1974).
pub const SKIN_T_FOR_HOT_C: f64 = 33.0;
/// Температура кожи пальцев при контакте с холодным, °C («оценка»: лёгкое охлаждение,
/// сужение сосудов — как у участников опытов Geng et al. 2006).
pub const SKIN_T_FOR_COLD_C: f64 = 30.0;

/// Температура контакта двух полубесконечных тел, °C.
pub fn contact_temperature(e1: f64, t1: f64, e2: f64, t2: f64) -> f64 {
    (e1 * t1 + e2 * t2) / (e1 + e2)
}

/// Дополнительная функция ошибок erfc(x), x ≥ 0 — по Чебышёвской аппроксимации
/// (Numerical Recipes, `erfcc`), относительная ошибка < 1.2·10⁻⁷.
pub fn erfc(x: f64) -> f64 {
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let ans = t
        * (-z * z - 1.265_512_23
            + t * (1.000_023_68
                + t * (0.374_091_96
                    + t * (0.096_784_18
                        + t * (-0.186_288_06
                            + t * (0.278_868_07
                                + t * (-1.135_203_98
                                    + t * (1.488_515_87
                                        + t * (-0.822_152_23 + t * 0.170_872_77)))))))))
            .exp();
    if x >= 0.0 { ans } else { 2.0 - ans }
}

/// Температура контакта кожи со слоем толщиной `thickness` на полубесконечной подложке,
/// оба при `t_material_c`, через время `time_s` после касания; кожа — полубесконечная при
/// `t_skin_c`. Точное решение.
///
/// Вывод (Лаплас по t, u = T − T_m в материале, v = T − T_s в коже): в слое
/// u = A(e^{−qx} + Γ·E·e^{qx}), E = e^{−2qL}, q = √(p/α₁); поток в материал
/// e₁√p·(1 − ΓE)/(1 + ΓE)·u(0); в коже поток −e_s√p·v(0); скачок u(0) − v(0) = ΔT/p.
/// Отсюда u(0) = ΔT/p · e_s/(e_s + e₁) · (1 + ΓE)/(1 − βΓE); разложение в ряд по E и
/// L⁻¹{E^n/p} = erfc(nL/√(α₁t)) дают формулу из описания модуля.
pub fn layered_contact_temperature(
    skin: &MaterialProps,
    t_skin_c: f64,
    layer: &MaterialProps,
    thickness: f64,
    substrate: &MaterialProps,
    t_material_c: f64,
    time_s: f64,
) -> f64 {
    let (es, e1, e2) = (
        skin.effusivity(),
        layer.effusivity(),
        substrate.effusivity(),
    );
    let gamma = (e1 - e2) / (e1 + e2);
    let beta = (e1 - es) / (e1 + es);
    let a1 = layer.diffusivity();
    let mut sum = 0.0;
    let mut coef = gamma; // β^{n−1}·Γ^n
    for n in 1..=400 {
        let arg = n as f64 * thickness / (a1 * time_s).sqrt();
        let term = coef * erfc(arg);
        sum += term;
        if arg > 6.0 || coef.abs() < 1e-14 {
            break;
        }
        coef *= beta * gamma;
    }
    t_material_c + (t_skin_c - t_material_c) * es / (es + e1) * (1.0 + (1.0 + beta) * sum)
}

// ---------------------------------------------------------------------------
// Численная модель: 1D, конечные объёмы, Кранк–Николсон
// ---------------------------------------------------------------------------

/// Перфузия кровью по Пеннесу: источник w·ρ_b·c_b·(T_a − T) в единице объёма.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Perfusion {
    /// Объёмный кровоток на объём ткани, 1/с (кожа в покое ≈ 0.5–2·10⁻³ 1/с).
    pub rate_per_s: f64,
    /// Температура артериальной крови, °C.
    pub t_arterial_c: f64,
}

/// Слой расчётной области.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layer {
    /// Материал.
    pub props: MaterialProps,
    /// Толщина, м.
    pub thickness: f64,
    /// Начальная температура, °C.
    pub t_init_c: f64,
    /// Число ячеек.
    pub cells: usize,
    /// Перфузия (только для ткани).
    pub perfusion: Option<Perfusion>,
}

/// Граничное условие на внешних концах области.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Boundary {
    /// Нет потока.
    Adiabatic,
    /// Заданная температура, °C.
    Fixed(f64),
}

/// Задача контакта: слои материала (от тыльной стороны к коже), затем слои ткани
/// (от поверхности кожи вглубь). Контакт — между последним слоем материала и первым слоем
/// ткани.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactProblem {
    /// Слои материала: [тыльный, …, касающийся кожи].
    pub material: Vec<Layer>,
    /// Слои ткани: [поверхностный, …, глубокий].
    pub tissue: Vec<Layer>,
    /// Граница за материалом.
    pub back: Boundary,
    /// Граница в глубине ткани.
    pub deep: Boundary,
}

struct Grid {
    dx: Vec<f64>,
    k: Vec<f64>,
    cap: Vec<f64>,   // ρ·c·dx, Дж/(м²·К)
    perf: Vec<f64>,  // w·ρ_b·c_b·dx, Вт/(м²·К)
    t_art: Vec<f64>, // температура артериальной крови, °C
    t0: Vec<f64>,
    contact: usize, // индекс первой ячейки ткани
}

/// Объёмная теплоёмкость крови, Дж/(м³·К) (ρ_b = 1050 кг/м³, c_b = 3617 Дж/(кг·К), IT'IS).
pub const BLOOD_VOLUMETRIC_HEAT: f64 = 1050.0 * 3617.0;

impl ContactProblem {
    fn grid(&self) -> Grid {
        let mut g = Grid {
            dx: vec![],
            k: vec![],
            cap: vec![],
            perf: vec![],
            t_art: vec![],
            t0: vec![],
            contact: 0,
        };
        let mut push = |l: &Layer| {
            let dx = l.thickness / l.cells as f64;
            for _ in 0..l.cells {
                g.dx.push(dx);
                g.k.push(l.props.k);
                g.cap.push(l.props.rho * l.props.c * dx);
                let (p, ta) = l
                    .perfusion
                    .map(|p| (p.rate_per_s * BLOOD_VOLUMETRIC_HEAT * dx, p.t_arterial_c))
                    .unwrap_or((0.0, 0.0));
                g.perf.push(p);
                g.t_art.push(ta);
                g.t0.push(l.t_init_c);
            }
        };
        for l in &self.material {
            push(l);
        }
        let contact: usize = self.material.iter().map(|l| l.cells).sum();
        for l in &self.tissue {
            push(l);
        }
        g.contact = contact;
        g
    }

    /// Расчёт: возвращает температуру контакта в моменты `times_s` (по возрастанию).
    /// Шаг по времени `dt`; первые два шага заменены четырьмя полушагами неявного Эйлера
    /// (старт Раннахера — гасит колебания Кранка–Николсона от разрывного начального поля).
    pub fn contact_temperatures(&self, times_s: &[f64], dt: f64) -> Vec<f64> {
        self.run(times_s, dt).0
    }

    /// То же, плюс поле температур в конце (центры ячеек: координата от контакта, м,
    /// отрицательная — в материале; температура, °C).
    pub fn run(&self, times_s: &[f64], dt: f64) -> (Vec<f64>, Vec<(f64, f64)>) {
        let g = self.grid();
        let n = g.dx.len();
        // проводимости между соседними ячейками
        let mut gface = vec![0.0; n + 1];
        for i in 1..n {
            gface[i] = 1.0 / (g.dx[i - 1] / (2.0 * g.k[i - 1]) + g.dx[i] / (2.0 * g.k[i]));
        }
        let (mut g_left, mut t_left) = (0.0, 0.0);
        if let Boundary::Fixed(t) = self.back {
            g_left = 2.0 * g.k[0] / g.dx[0];
            t_left = t;
        }
        let (mut g_right, mut t_right) = (0.0, 0.0);
        if let Boundary::Fixed(t) = self.deep {
            g_right = 2.0 * g.k[n - 1] / g.dx[n - 1];
            t_right = t;
        }
        gface[0] = g_left;
        gface[n] = g_right;
        let interface = |t: &[f64]| {
            let c = g.contact;
            let gl = 2.0 * g.k[c - 1] / g.dx[c - 1];
            let gr = 2.0 * g.k[c] / g.dx[c];
            (gl * t[c - 1] + gr * t[c]) / (gl + gr)
        };
        let mut temp = g.t0.clone();
        let mut out = Vec::with_capacity(times_s.len());
        let mut time = 0.0;
        let mut next = 0;
        let mut step_count = 0usize;
        // A·T + b: поток в ячейку i
        let apply = |t: &[f64], i: usize| -> f64 {
            let tl = if i == 0 { t_left } else { t[i - 1] };
            let tr = if i == n - 1 { t_right } else { t[i + 1] };
            gface[i] * (tl - t[i]) + gface[i + 1] * (tr - t[i]) + g.perf[i] * (g.t_art[i] - t[i])
        };
        let mut a = vec![0.0; n];
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];
        while next < times_s.len() {
            let target = times_s[next];
            if time >= target - 1e-12 {
                out.push(interface(&temp));
                next += 1;
                continue;
            }
            // шаг: Раннахер на первых двух шагах
            let (h, theta) = if step_count < 4 {
                (0.5 * dt, 1.0)
            } else {
                (dt, 0.5)
            };
            let h = h.min(target - time);
            for i in 0..n {
                let diag = g.cap[i] / h + theta * (gface[i] + gface[i + 1] + g.perf[i]);
                a[i] = if i == 0 { 0.0 } else { -theta * gface[i] };
                c[i] = if i == n - 1 {
                    0.0
                } else {
                    -theta * gface[i + 1]
                };
                b[i] = diag;
                let mut rhs = g.cap[i] / h * temp[i] + (1.0 - theta) * apply(&temp, i);
                rhs += theta * g.perf[i] * g.t_art[i];
                if i == 0 {
                    rhs += theta * gface[0] * t_left;
                }
                if i == n - 1 {
                    rhs += theta * gface[n] * t_right;
                }
                d[i] = rhs;
            }
            thomas(&a, &mut b, &c, &mut d);
            temp.copy_from_slice(&d);
            time += h;
            step_count += 1;
        }
        // профиль
        let mut x = -(0..g.contact).map(|i| g.dx[i]).sum::<f64>();
        let mut profile = Vec::with_capacity(n);
        for i in 0..n {
            profile.push((x + 0.5 * g.dx[i], temp[i]));
            x += g.dx[i];
        }
        (out, profile)
    }
}

/// Прогонка (алгоритм Томаса): решение трёхдиагональной системы; результат в `d`.
fn thomas(a: &[f64], b: &mut [f64], c: &[f64], d: &mut [f64]) {
    let n = d.len();
    for i in 1..n {
        let w = a[i] / b[i - 1];
        b[i] -= w * c[i - 1];
        d[i] -= w * d[i - 1];
    }
    d[n - 1] /= b[n - 1];
    for i in (0..n - 1).rev() {
        d[i] = (d[i] - c[i] * d[i + 1]) / b[i];
    }
}

/// Готовая задача «два полубесконечных тела»: материал при `t_material_c` и кожа при
/// `t_skin_c`, длина каждой стороны 8·√(α·t_end), `cells` ячеек на сторону, концы
/// адиабатические (на таком удалении возмущение < 10⁻⁸).
pub fn semi_infinite_problem(
    material: &MaterialProps,
    t_material_c: f64,
    skin: &MaterialProps,
    t_skin_c: f64,
    t_end: f64,
    cells: usize,
) -> ContactProblem {
    let lm = 8.0 * (material.diffusivity() * t_end).sqrt();
    let ls = 8.0 * (skin.diffusivity() * t_end).sqrt();
    ContactProblem {
        material: vec![Layer {
            props: *material,
            thickness: lm,
            t_init_c: t_material_c,
            cells,
            perfusion: None,
        }],
        tissue: vec![Layer {
            props: *skin,
            thickness: ls,
            t_init_c: t_skin_c,
            cells,
            perfusion: None,
        }],
        back: Boundary::Adiabatic,
        deep: Boundary::Adiabatic,
    }
}

/// Точное решение для двух полубесконечных тел в точке `x` (x < 0 — материал, x > 0 — кожа).
pub fn semi_infinite_exact(
    material: &MaterialProps,
    t_material_c: f64,
    skin: &MaterialProps,
    t_skin_c: f64,
    x: f64,
    time_s: f64,
) -> f64 {
    let tc = contact_temperature(
        material.effusivity(),
        t_material_c,
        skin.effusivity(),
        t_skin_c,
    );
    let (ti, a) = if x < 0.0 {
        (t_material_c, material.diffusivity())
    } else {
        (t_skin_c, skin.diffusivity())
    };
    let s = x.abs() / (2.0 * (a * time_s).sqrt());
    tc + (ti - tc) * (1.0 - erfc(s))
}

// ---------------------------------------------------------------------------
// Пороги: ожог (ISO 13732-1), холод (ISO 13732-3)
// ---------------------------------------------------------------------------

/// Время контакта для таблицы порогов ожога, с.
pub const BURN_TIMES_S: [f64; 6] = [1.0, 4.0, 10.0, 60.0, 600.0, 28_800.0];

/// Пороги ожога (температура ПОВЕРХНОСТИ), °C, для времён [`BURN_TIMES_S`]:
/// 1, 4, 10 с — EN 13202:2000 (по EN 563 = ISO 13732-1, нижние границы разбросов;
/// таблица воспроизведена в Arora, Medora, Pinnangudi, «Accessible hot surfaces and burn
/// hazards», IEEE PSES Chicago, 2010); 1 мин, 10 мин, 8 ч — ISO 13732-1:2006 табл. 1
/// (= CENELEC Guide 29:2007, табл. A.1). Значение 43 °C при 8 ч — только если касается
/// малая часть тела (< 10 % поверхности).
pub fn burn_threshold_table(class: ContactClass) -> [f64; 6] {
    match class {
        ContactClass::UncoatedMetal | ContactClass::Tissue => [65.0, 58.0, 55.0, 51.0, 48.0, 43.0],
        ContactClass::PaintedMetal => [83.0, 64.0, 55.0, 51.0, 48.0, 43.0],
        ContactClass::EnamelledMetal => [74.0, 60.0, 56.0, 51.0, 48.0, 43.0],
        ContactClass::CeramicGlassStone => [80.0, 70.0, 66.0, 56.0, 48.0, 43.0],
        ContactClass::Plastics => [85.0, 74.0, 70.0, 60.0, 48.0, 43.0],
        ContactClass::Wood => [110.0, 93.0, 89.0, 60.0, 48.0, 43.0],
    }
}

/// Порог ожога для класса и времени контакта, °C (температура поверхности). Между узлами
/// таблицы — линейно по lg t (ISO 13732-1 предписывает интерполяцию); короче 1 с — значение
/// для 1 с (консервативно: при 0.5 с порог выше); дольше 8 ч — 43 °C.
pub fn burn_threshold_c(class: ContactClass, contact_time_s: f64) -> f64 {
    let v = burn_threshold_table(class);
    let t = contact_time_s.max(BURN_TIMES_S[0]);
    if t >= BURN_TIMES_S[5] {
        return v[5];
    }
    let i = BURN_TIMES_S.iter().rposition(|&x| x <= t).unwrap_or(0);
    let (t0, t1) = (BURN_TIMES_S[i].log10(), BURN_TIMES_S[i + 1].log10());
    let s = (t.log10() - t0) / (t1 - t0);
    v[i] + s * (v[i + 1] - v[i])
}

/// Порог тепловой боли по температуре кожи, °C («ориентир»: таблица ощущений ASTM C1055-03 —
/// порог боли при тканевой температуре 40–44 °C; берём 43 °C).
pub const HEAT_PAIN_CONTACT_C: f64 = 43.0;

/// Критерий холодовой боли — температура контакта 15 °C (ISO 13732-3:2005; Havenith et al. 1992).
pub const COLD_PAIN_CONTACT_C: f64 = 15.0;
/// Критерий онемения — 7 °C (ISO 13732-3:2005; Provins & Morton 1960).
pub const COLD_NUMBNESS_CONTACT_C: f64 = 7.0;
/// Критерий риска обморожения (frostnip) — 0 °C (ISO 13732-3:2005; Danielsson 1996:
/// риск замерзания кожи растёт с 5 до 95 % при поверхности кожи от −4.8 до −7.8 °C).
pub const FROSTNIP_CONTACT_C: f64 = 0.0;

/// Время, с, до температуры контакта 15 °C при касании пальцем поверхности с контактным
/// фактором `f_c_kilo` (e в 10³ Вт·с^½/(м²·К)) и температурой `t_surface_c`:
/// t(15) = 13.7/F_C^1.09·exp(0.108·T_s) (Geng et al. 2006, ур. 5; R² = 0.93; нейлон,
/// сталь, алюминий; нижний квартиль — защищает 75 % людей). Применимость: F_C 0.78…21.2,
/// T_s −40…+2 °C. Вне — `None`.
///
/// Источник: Geng Q., Holmér I. et al. (2006) «Temperature limit values for touching cold
/// surfaces with the fingertip», Ann. Occup. Hyg. 50(8):851–862, doi:10.1093/annhyg/mel030
/// (основа ISO 13732-3). Показатели степеней прочитаны из рукописи (Loughborough repository,
/// https://repository.lboro.ac.uk/articles/journal_contribution/9349373); ур. 4 (онемение)
/// из рукописи однозначно не восстанавливается и не используется.
pub fn geng_time_to_pain_s(f_c_kilo: f64, t_surface_c: f64) -> Option<f64> {
    if !(0.7..=22.0).contains(&f_c_kilo) || !(-40.0..=2.0).contains(&t_surface_c) {
        return None;
    }
    Some(13.7 / f_c_kilo.powf(1.09) * (0.108 * t_surface_c).exp())
}

/// Время, с, до температуры контакта 0 °C (риск обморожения) для металлов:
/// t(0) = 980.5/F_C^1.03·exp(0.21·T_s) (Geng et al. 2006, ур. 3; R² = 0.99; только сталь
/// и алюминий, F_C 7.3…21.2). Применимость T_s −40…0 °C. Вне — `None`.
pub fn geng_time_to_frostnip_s(f_c_kilo: f64, t_surface_c: f64) -> Option<f64> {
    if !(5.0..=22.0).contains(&f_c_kilo) || !(-40.0..=0.0).contains(&t_surface_c) {
        return None;
    }
    Some(980.5 / f_c_kilo.powf(1.03) * (0.21 * t_surface_c).exp())
}

/// Уровень риска при касании.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RiskLevel {
    /// Безопасно.
    Safe,
    /// Горячо: кожа выше порога тепловой боли, но ниже порога ожога.
    HeatPain,
    /// Холодовая боль (контакт ≤ 15 °C).
    ColdPain,
    /// Онемение (контакт ≤ 7 °C).
    Numbness,
    /// Риск обморожения (контакт ≤ 0 °C).
    FrostnipRisk,
    /// Риск ожога (поверхность ≥ порога ISO 13732-1 для этого времени).
    BurnRisk,
}

/// Оценка риска касания.
#[derive(Clone, Debug, PartialEq)]
pub struct Risk {
    /// Температура контакта (полубесконечные тела, мгновенная), °C.
    pub contact_temp_c: f64,
    /// Уровень.
    pub level: RiskLevel,
    /// Порог, с которым сравнивали, °C: порог ожога (температура поверхности) или
    /// критерий холода (температура контакта).
    pub threshold_c: Option<f64>,
    /// Время до достижения критерия холода по Geng et al. 2006, с (если модель применима).
    pub time_to_threshold_s: Option<f64>,
    /// Источник порога.
    pub threshold_source: &'static str,
    /// Пояснения.
    pub notes: Vec<&'static str>,
}

const SRC_HOT: &str =
    "ISO 13732-1:2006 (табл. 1) и EN 13202:2000: пороги ожога по классу материала и времени";
const SRC_COLD: &str =
    "ISO 13732-3:2005: контакт 15/7/0 °C — боль/онемение/обморожение; время — Geng et al. 2006";

/// Риск касания кожей материала `material` с температурой поверхности `t_surface_c` в течение
/// `contact_time_s`. Кожа: 33 °C для горячего, 30 °C для холодного.
pub fn contact_risk(material: Material, t_surface_c: f64, contact_time_s: f64) -> Risk {
    contact_risk_props(&material.props(), t_surface_c, contact_time_s)
}

/// То же для произвольного материала.
pub fn contact_risk_props(m: &MaterialProps, t_surface_c: f64, contact_time_s: f64) -> Risk {
    let skin = Material::Skin.props();
    let es = skin.effusivity();
    let em = m.effusivity();
    if t_surface_c >= SKIN_T_FOR_HOT_C {
        let tc = contact_temperature(es, SKIN_T_FOR_HOT_C, em, t_surface_c);
        let thr = burn_threshold_c(m.class, contact_time_s);
        let level = if t_surface_c >= thr {
            RiskLevel::BurnRisk
        } else if tc >= HEAT_PAIN_CONTACT_C {
            RiskLevel::HeatPain
        } else {
            RiskLevel::Safe
        };
        let mut notes =
            vec!["порог ожога задан для температуры поверхности (свойства материала уже учтены)"];
        if contact_time_s < 1.0 {
            notes.push("контакт короче 1 с: взят порог для 1 с (консервативно)");
        }
        if contact_time_s >= 28_800.0 {
            notes.push("43 °C при 8 ч справедливо, только если касается < 10 % поверхности тела");
        }
        return Risk {
            contact_temp_c: tc,
            level,
            threshold_c: Some(thr),
            time_to_threshold_s: None,
            threshold_source: SRC_HOT,
            notes,
        };
    }
    let tc = contact_temperature(es, SKIN_T_FOR_COLD_C, em, t_surface_c);
    let fc = em / 1000.0;
    let t15 = geng_time_to_pain_s(fc, t_surface_c);
    let t0 = geng_time_to_frostnip_s(fc, t_surface_c);
    let mut notes = vec![
        "мгновенная температура контакта двух полубесконечных тел, кожа 30 °C",
        "без контактного сопротивления и влаги; инеевая плёнка ускоряет охлаждение",
    ];
    let (level, thr, time) = if tc <= FROSTNIP_CONTACT_C || t0.is_some_and(|t| contact_time_s >= t)
    {
        (RiskLevel::FrostnipRisk, FROSTNIP_CONTACT_C, t0)
    } else if tc <= COLD_NUMBNESS_CONTACT_C {
        (RiskLevel::Numbness, COLD_NUMBNESS_CONTACT_C, None)
    } else if tc <= COLD_PAIN_CONTACT_C || t15.is_some_and(|t| contact_time_s >= t) {
        (RiskLevel::ColdPain, COLD_PAIN_CONTACT_C, t15)
    } else {
        (RiskLevel::Safe, COLD_PAIN_CONTACT_C, t15)
    };
    if t15.is_none() && t_surface_c < COLD_PAIN_CONTACT_C {
        notes.push("время до боли по Geng 2006 вне области (дерево, пена, или T вне −40…+2 °C) — только мгновенная оценка");
    }
    Risk {
        contact_temp_c: tc,
        level,
        threshold_c: Some(thr),
        time_to_threshold_s: time,
        threshold_source: SRC_COLD,
        notes,
    }
}

/// Температура поверхности, °C, при которой касание `material` на `contact_time_s` достигает
/// порога ожога — просто порог ISO 13732-1 для класса (обёртка для интерфейса).
pub fn max_touch_temperature_c(material: Material, contact_time_s: f64) -> f64 {
    burn_threshold_c(material.props().class, contact_time_s)
}

/// Время, с, за которое температура контакта со слоем `thickness` на подложке сместится
/// от «как у слоя» к «как у подложки» на 10 % разницы (численно по точному решению).
/// Показывает, когда тонкий пластик на металле перестаёт защищать.
pub fn layer_breakthrough_time_s(
    layer: &MaterialProps,
    thickness: f64,
    substrate: &MaterialProps,
    t_skin_c: f64,
    t_material_c: f64,
) -> f64 {
    let skin = Material::Skin.props();
    let t_layer = contact_temperature(
        skin.effusivity(),
        t_skin_c,
        layer.effusivity(),
        t_material_c,
    );
    let t_sub = contact_temperature(
        skin.effusivity(),
        t_skin_c,
        substrate.effusivity(),
        t_material_c,
    );
    let target = t_layer + 0.1 * (t_sub - t_layer);
    let f = |log_t: f64| {
        let tc = layered_contact_temperature(
            &skin,
            t_skin_c,
            layer,
            thickness,
            substrate,
            t_material_c,
            10f64.powf(log_t),
        );
        (tc - target) * (t_sub - t_layer).signum()
    };
    10f64.powf(bisect(f, -4.0, 6.0, 1e-9))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn erfc_values() {
        assert!((erfc(0.0) - 1.0).abs() < 1e-7);
        assert!((erfc(1.0) - 0.157_299_207).abs() < 1e-7);
        assert!((erfc(2.0) - 0.004_677_735).abs() < 1e-8);
    }

    #[test]
    fn layered_limits() {
        let skin = Material::Skin.props();
        let pla = Material::Pla.props();
        let al = Material::Aluminium6061.props();
        let early = layered_contact_temperature(&skin, 33.0, &pla, 1e-3, &al, -20.0, 1e-3);
        let late = layered_contact_temperature(&skin, 33.0, &pla, 1e-3, &al, -20.0, 1e9);
        let t_pla = contact_temperature(skin.effusivity(), 33.0, pla.effusivity(), -20.0);
        let t_al = contact_temperature(skin.effusivity(), 33.0, al.effusivity(), -20.0);
        assert!((early - t_pla).abs() < 1e-6, "{early} {t_pla}");
        assert!((late - t_al).abs() < 0.1, "{late} {t_al}");
    }

    #[test]
    fn risk_examples() {
        // алюминий на морозе −20 °C — обморожение
        assert_eq!(
            contact_risk(Material::Aluminium6061, -20.0, 5.0).level,
            RiskLevel::FrostnipRisk
        );
        // дерево −20 °C, 5 с — максимум боль
        assert!(contact_risk(Material::Wood, -20.0, 5.0).level <= RiskLevel::ColdPain);
        // чёрный металл на солнце 70 °C, 1 с — ожог (порог 65 °C)
        assert_eq!(
            contact_risk(Material::Aluminium6061, 70.0, 1.0).level,
            RiskLevel::BurnRisk
        );
        // пластик 70 °C, 1 с — не ожог (порог 85 °C)
        assert_ne!(
            contact_risk(Material::Abs, 70.0, 1.0).level,
            RiskLevel::BurnRisk
        );
        assert!((burn_threshold_c(ContactClass::UncoatedMetal, 60.0) - 51.0).abs() < 1e-9);
    }
}
