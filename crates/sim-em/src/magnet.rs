//! Постоянные магниты NdFeB.
//!
//! Модель: «жёсткий» магнит с однородной намагниченностью M = Br/μ₀ и μ_r = 1
//! (у спечённого NdFeB μ_rec ≈ 1.05 — пренебрегаем, ошибка порядка нескольких %).
//! Эквивалент — поверхностный ток K = M × n:
//! * осевой цилиндр → азимутальный слой K = M на боковой поверхности (как соленоид),
//!   поле — замкнутая формула Derby–Olbert, сверка со стопкой витков и формулой на оси;
//! * диаметральный диск → стопка прямоугольных рамок (срезы, намагниченные поперёк),
//!   сверка с моделью магнитных зарядов σ = M·n (другая эквивалентная модель).
//!
//! Сила между соосными магнитами — интеграл точных сил «виток–виток» (K, E) по обоим
//! слоям; сверка — интеграл Ганкеля с функциями Бесселя (Vokoun et al., JMMM 321 (2009)
//! 3758, doi:10.1016/j.jmmm.2009.07.030; та же формула выводится из взаимной индуктивности
//! соосных витков, Smythe 1950) и дипольный предел 3μ₀m₁m₂/(2πz⁴).
//!
//! Магнит на толстой стальной плите — метод изображений (μ_стали → ∞): сила равна
//! притяжению к зеркальной копии на расстоянии 2·зазор. Это ВЕРХНЯЯ оценка: реальная
//! сталь насыщается (B_s ≈ 2 Т), имеет конечную μ и шероховатость, покрытие магнита
//! (Ni-Cu-Ni) даёт паразитный зазор — измеренная сила ниже.

use sim_verify::{Check, Tolerance};
use std::f64::consts::PI;

use crate::MU0;
use crate::field::{FieldSource, Vec3, loop_field_local, segment_field};
use crate::field::{sheet_cylinder_field_by_loops, sheet_cylinder_field_local};
use crate::math::{GaussLegendre, bessel_jn, tanh_sinh};

// ---------------------------------------------------------------------------------------
// Марки и температура
// ---------------------------------------------------------------------------------------

/// Марка спечённого NdFeB (класс по температуре — без суффикса, т.е. «N»).
#[derive(Clone, Copy, Debug)]
pub struct NdfebGrade {
    pub name: &'static str,
    /// Остаточная индукция при 20 °C, диапазон, Тл.
    pub br_min_t: f64,
    pub br_max_t: f64,
    /// Минимальная коэрцитивная сила по намагниченности HcJ, кА/м.
    pub hcj_min_ka_per_m: f64,
    /// Диапазон (BH)max, кДж/м³.
    pub bh_max_min_kj_per_m3: f64,
    pub bh_max_max_kj_per_m3: f64,
}

impl NdfebGrade {
    /// Середина диапазона Br — «типовое» значение для расчёта.
    pub fn br_typical_t(&self) -> f64 {
        0.5 * (self.br_min_t + self.br_max_t)
    }
}

/// Источник таблицы марок.
pub const GRADES_SOURCE: &str = "magnets4you GmbH, «Technical specifications», \
https://www.magnet-shop.com/technical-specifications (таблица марок N35…N52, Br в Тл, \
HcJ ≥ 955 кА/м для класса N); сверено с Arnold Magnetic Technologies, \
https://www.arnoldmagnetics.com/products/neodymium-iron-boron-magnets/ (типовые Br: \
N42 1315 мТ, N52 1450 мТ).";

/// Марки N35…N52 (класс «N», ≤ 80 °C по magnet-shop; Arnold для N52 даёт 60 °C).
pub const NDFEB_GRADES: [NdfebGrade; 8] = [
    grade("N35", 1.17, 1.21, 263.0, 279.0),
    grade("N38", 1.22, 1.26, 287.0, 303.0),
    grade("N40", 1.26, 1.29, 303.0, 318.0),
    grade("N42", 1.29, 1.32, 318.0, 334.0),
    grade("N45", 1.32, 1.37, 342.0, 358.0),
    grade("N48", 1.37, 1.42, 358.0, 382.0),
    grade("N50", 1.40, 1.46, 374.0, 406.0),
    grade("N52", 1.42, 1.47, 380.0, 422.0),
];

const fn grade(name: &'static str, bmin: f64, bmax: f64, e1: f64, e2: f64) -> NdfebGrade {
    NdfebGrade {
        name,
        br_min_t: bmin,
        br_max_t: bmax,
        hcj_min_ka_per_m: 955.0,
        bh_max_min_kj_per_m3: e1,
        bh_max_max_kj_per_m3: e2,
    }
}

/// Найти марку по имени («N42»).
pub fn ndfeb_grade(name: &str) -> Option<NdfebGrade> {
    NDFEB_GRADES.iter().copied().find(|g| g.name == name)
}

/// Температурный класс (суффикс марки) и максимальная рабочая температура.
#[derive(Clone, Copy, Debug)]
pub struct TempClass {
    pub suffix: &'static str,
    pub max_operating_c: f64,
    /// Минимальная HcJ, кА/м (magnet-shop; для AH — по Arnold не приводится, NaN).
    pub hcj_min_ka_per_m: f64,
}

/// Максимальные рабочие температуры по суффиксу: N 80, M 100, H 120, SH 150, UH 180,
/// EH 200 °C — magnet-shop.com (см. [`GRADES_SOURCE`]); AH 220 °C — Arnold (N28AH…N35AH).
/// ВАЖНО: это предел для магнита с «нормальной» рабочей точкой (B/μ₀H ≈ −1…−2);
/// тонкий диск (малый коэффициент проводимости) начинает необратимо терять поток раньше.
pub const TEMP_CLASSES: [TempClass; 7] = [
    TempClass {
        suffix: "",
        max_operating_c: 80.0,
        hcj_min_ka_per_m: 955.0,
    },
    TempClass {
        suffix: "M",
        max_operating_c: 100.0,
        hcj_min_ka_per_m: 1114.0,
    },
    TempClass {
        suffix: "H",
        max_operating_c: 120.0,
        hcj_min_ka_per_m: 1353.0,
    },
    TempClass {
        suffix: "SH",
        max_operating_c: 150.0,
        hcj_min_ka_per_m: 1592.0,
    },
    TempClass {
        suffix: "UH",
        max_operating_c: 180.0,
        hcj_min_ka_per_m: 1990.0,
    },
    TempClass {
        suffix: "EH",
        max_operating_c: 200.0,
        hcj_min_ka_per_m: 2388.0,
    },
    TempClass {
        suffix: "AH",
        max_operating_c: 220.0,
        hcj_min_ka_per_m: f64::NAN,
    },
];

/// Класс по полному имени марки («N42SH» → SH).
pub fn temp_class_of(grade_name: &str) -> Option<TempClass> {
    let digits_end = grade_name
        .char_indices()
        .skip(1)
        .find(|(_, c)| !c.is_ascii_digit())
        .map(|(i, _)| i)
        .unwrap_or(grade_name.len());
    let suffix = &grade_name[digits_end..];
    TEMP_CLASSES.iter().copied().find(|t| t.suffix == suffix)
}

/// Обратимый температурный коэффициент Br спечённого NdFeB, 1/К: −0.12 %/°C
/// (Arnold Magnetic Technologies, таблица марок N42…N35AH — одинаково −0.12 %/°C).
pub const BR_TEMP_COEFF_PER_K: f64 = -0.0012;

/// Br при температуре `t_c` (обратимая часть, от 20 °C). Не учитывает необратимые потери
/// выше максимальной рабочей температуры класса.
pub fn br_at_temperature(br_20c: f64, t_c: f64) -> f64 {
    br_20c * (1.0 + BR_TEMP_COEFF_PER_K * (t_c - 20.0))
}

// ---------------------------------------------------------------------------------------
// Осевой цилиндрический магнит
// ---------------------------------------------------------------------------------------

/// Цилиндр, намагниченный вдоль своей оси (направление `axis` = север).
#[derive(Clone, Copy, Debug)]
pub struct AxialCylinderMagnet {
    pub center: Vec3,
    pub axis: Vec3,
    pub radius: f64,
    pub length: f64,
    /// Остаточная индукция, Тл (знак «−» — намагничен против `axis`).
    pub br: f64,
}

impl AxialCylinderMagnet {
    pub fn new(center: Vec3, axis: Vec3, radius: f64, length: f64, br: f64) -> Self {
        Self {
            center,
            axis: axis.normalize(),
            radius,
            length,
            br,
        }
    }
    pub fn magnetization(&self) -> f64 {
        self.br / MU0
    }
    pub fn volume(&self) -> f64 {
        PI * self.radius * self.radius * self.length
    }
    /// Магнитный момент m = M·V, А·м².
    pub fn moment(&self) -> f64 {
        self.magnetization() * self.volume()
    }
    /// Поле в локальных (ρ, z от центра): (B_ρ, B_z) — замкнутая формула слоя.
    pub fn field_local(&self, rho: f64, z: f64) -> (f64, f64) {
        sheet_cylinder_field_local(self.radius, 0.5 * self.length, self.magnetization(), rho, z)
    }
}

impl FieldSource for AxialCylinderMagnet {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        let d = r - self.center;
        let z = d.dot(&self.axis);
        let radial = d - self.axis * z;
        let rho = radial.norm();
        let (br, bz) = self.field_local(rho, z);
        let mut b = self.axis * bz;
        if rho > 0.0 {
            b += radial * (br / rho);
        }
        b
    }
}

/// Поле на оси осевого цилиндра на расстоянии `z` от торца (снаружи):
/// B(z) = Br/2·[(z + L)/√(R² + (z + L)²) − z/√(R² + z²)].
pub fn on_axis_field_from_face(br: f64, radius: f64, length: f64, z: f64) -> f64 {
    0.5 * br
        * ((z + length) / (radius * radius + (z + length).powi(2)).sqrt()
            - z / (radius * radius + z * z).sqrt())
}

/// Поле точечного диполя m (А·м², вектор) в точке r относительно диполя:
/// B = μ₀/(4π)·(3(m·r̂)r̂ − m)/r³.
pub fn dipole_field(m: &Vec3, r: &Vec3) -> Vec3 {
    let d = r.norm();
    let u = r / d;
    (3.0 * m.dot(&u) * u - m) * (MU0 / (4.0 * PI * d.powi(3)))
}

// ---------------------------------------------------------------------------------------
// Силы
// ---------------------------------------------------------------------------------------

/// Осевая сила на виток 2 (радиус b, ток i2) от соосного витка 1 (радиус a, ток i1),
/// виток 2 выше на `z`. F_z = −2π·b·i2·B_ρ⁽¹⁾(b, z). Отрицательная — притяжение.
/// Эквивалентна замкнутой формуле F = μ₀i₁i₂z/√((a+b)²+z²)·[K − (a²+b²+z²)/((a−b)²+z²)·E].
pub fn loop_loop_axial_force(a: f64, i1: f64, b: f64, i2: f64, z: f64) -> f64 {
    let (brho, _) = loop_field_local(a, i1, b, z);
    -2.0 * PI * b * i2 * brho
}

/// Пара соосных осевых цилиндрических магнитов: магнит 1 снизу, магнит 2 над ним,
/// `gap` — зазор между ближними торцами. Br со знаком: одинаковые знаки — магниты
/// намагничены в одну сторону (N к S — притягиваются).
#[derive(Clone, Copy, Debug)]
pub struct CoaxialPair {
    pub br1: f64,
    pub r1: f64,
    pub l1: f64,
    pub br2: f64,
    pub r2: f64,
    pub l2: f64,
    pub gap: f64,
}

impl CoaxialPair {
    /// Магнит и его зеркальное изображение в идеальной стали (μ → ∞) при зазоре `gap`
    /// до плиты: изображение имеет ту же намагниченность, расстояние между торцами 2·gap.
    pub fn with_image(br: f64, radius: f64, length: f64, gap: f64) -> Self {
        Self {
            br1: br,
            r1: radius,
            l1: length,
            br2: br,
            r2: radius,
            l2: length,
            gap: 2.0 * gap,
        }
    }
    pub fn moments(&self) -> (f64, f64) {
        (
            self.br1 / MU0 * PI * self.r1 * self.r1 * self.l1,
            self.br2 / MU0 * PI * self.r2 * self.r2 * self.l2,
        )
    }
    /// Расстояние между центрами.
    pub fn center_distance(&self) -> f64 {
        self.gap + 0.5 * (self.l1 + self.l2)
    }
}

/// Сила на магнит 2 вдоль оси (Н), отрицательная — притяжение. Метод: двойной интеграл
/// сил «виток–виток» по слоям K₁ = M₁, K₂ = M₂. Сила зависит только от u = z₂ − z₁,
/// поэтому двойной интеграл сводится к одномерному: F = M₁M₂·∫ f(u)·w(u) du, где w(u) —
/// длина пересечения (свёртка двух «ступенек», кусочно-линейная). Каждый кусок —
/// квадратура tanh-sinh (держит особенность f ~ 1/u при касании торцов, gap = 0).
pub fn coaxial_force_loops(p: &CoaxialPair) -> f64 {
    let (m1, m2) = (p.br1 / MU0, p.br2 / MU0);
    let d = p.gap;
    let (lmin, lmax) = if p.l1 < p.l2 {
        (p.l1, p.l2)
    } else {
        (p.l2, p.l1)
    };
    let w = |u: f64| -> f64 {
        // z1 ∈ [−l1, 0], z2 = z1 + u ∈ [d, d + l2]
        let lo = (-p.l1).max(d - u);
        let hi = 0.0_f64.min(d + p.l2 - u);
        (hi - lo).max(0.0)
    };
    let f = |u: f64| loop_loop_axial_force(p.r1, 1.0, p.r2, 1.0, u) * w(u);
    let breaks = [d, d + lmin, d + lmax, d + p.l1 + p.l2];
    let mut s = 0.0;
    for k in 0..3 {
        if breaks[k + 1] > breaks[k] {
            s += tanh_sinh(breaks[k], breaks[k + 1], 7, f);
        }
    }
    m1 * m2 * s
}

/// Та же сила через интеграл Ганкеля (второй, независимый способ):
/// F = −μ₀π·R₁R₂·M₁M₂·∫₀^∞ J₁(kR₁)J₁(kR₂)(1 − e^{−kL₁})(1 − e^{−kL₂})e^{−kd}/k dk.
/// Вывод: взаимная индуктивность соосных витков M(z) = μ₀πab∫J₁(ka)J₁(kb)e^{−k|z|}dk
/// (Smythe 1950, §7.13), F = I₁I₂·∂M/∂z, интегрирование по слоям — в элементарных
/// функциях. Совпадает с формулой Vokoun et al. (2009). Требует gap > 0 (иначе хвост
/// интеграла затухает лишь как 1/k²).
pub fn coaxial_force_bessel(p: &CoaxialPair) -> f64 {
    assert!(p.gap > 0.0, "интеграл Ганкеля нужен с зазором > 0");
    let (m1, m2) = (p.br1 / MU0, p.br2 / MU0);
    let d = p.gap;
    let kmax = 40.0 / d; // e^{−40} ≈ 4e-18
    let rmax = p.r1.max(p.r2);
    // панель ≈ четверть периода осцилляций J₁(kR)
    let panel = 0.5 * PI / rmax;
    let panels = ((kmax / panel).ceil() as usize).max(16);
    let gl = GaussLegendre::new(10);
    let g = |k: f64| {
        if k == 0.0 {
            return 0.0;
        }
        bessel_jn(1, k * p.r1)
            * bessel_jn(1, k * p.r2)
            * (-(-k * p.l1).exp_m1())
            * (-(-k * p.l2).exp_m1())
            * (-k * d).exp()
            / k
    };
    let integral = gl.integrate_composite(0.0, kmax, panels, g);
    -MU0 * PI * p.r1 * p.r2 * m1 * m2 * integral
}

/// Дипольный предел: два соосных диполя на расстоянии z (центр–центр):
/// F = −3μ₀m₁m₂/(2πz⁴) (отрицательная — притяжение при одинаковой ориентации).
pub fn dipole_dipole_coaxial_force(m1: f64, m2: f64, z: f64) -> f64 {
    -3.0 * MU0 * m1 * m2 / (2.0 * PI * z.powi(4))
}

/// Сила притяжения осевого цилиндра к толстой плите из идеальной стали (μ → ∞) при зазоре
/// `gap` между торцом и плитой (Н, положительная — притяжение). Метод изображений.
/// Это верхняя оценка — см. описание модуля.
pub fn pull_force_ideal_steel(br: f64, radius: f64, length: f64, gap: f64) -> f64 {
    -coaxial_force_loops(&CoaxialPair::with_image(br, radius, length, gap))
}

/// Верхняя граница силы отрыва при касании: F ≤ Br²·A/(2μ₀) (давление Максвелла при
/// индукции Br на всей площади торца — предел бесконечно длинного магнита).
pub fn contact_pull_upper_bound(br: f64, radius: f64) -> f64 {
    br * br * PI * radius * radius / (2.0 * MU0)
}

// ---------------------------------------------------------------------------------------
// Диаметрально намагниченный диск (магнит датчика угла)
// ---------------------------------------------------------------------------------------

/// Цилиндр (диск), намагниченный поперёк оси — типовой магнит датчика угла на оси
/// сустава. Локальная СК: z — ось цилиндра (`axis`), x — направление намагниченности
/// (`mag_dir`, проецируется на плоскость, перпендикулярную оси).
#[derive(Clone, Copy, Debug)]
pub struct DiametricDiscMagnet {
    pub center: Vec3,
    pub axis: Vec3,
    pub mag_dir: Vec3,
    pub radius: f64,
    pub thickness: f64,
    pub br: f64,
    /// Число узлов квадратуры по срезам (по умолчанию 48).
    pub slices: usize,
}

impl DiametricDiscMagnet {
    pub fn new(
        center: Vec3,
        axis: Vec3,
        mag_dir: Vec3,
        radius: f64,
        thickness: f64,
        br: f64,
    ) -> Self {
        let w = axis.normalize();
        let u = (mag_dir - w * mag_dir.dot(&w)).normalize();
        Self {
            center,
            axis: w,
            mag_dir: u,
            radius,
            thickness,
            br,
            slices: 48,
        }
    }

    fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let u = self.mag_dir;
        let w = self.axis;
        (u, w.cross(&u), w)
    }

    /// Поле моделью тока: срез толщиной dx, намагниченный по x, эквивалентен прямоугольной
    /// рамке по его контуру (ширина 2√(R² − x²), высота h) с током I = M·dx.
    /// Интеграл по x = R·sin θ (снимает корневую особенность у краёв) — Гаусс–Лежандр.
    pub fn b_current_model(&self, r: &Vec3) -> Vec3 {
        let (u, v, w) = self.basis();
        let m = self.br / MU0;
        let hh = 0.5 * self.thickness;
        let gl = GaussLegendre::new(self.slices.max(4));
        let mut b = Vec3::zeros();
        let half_pi = 0.5 * PI;
        for (xi, wi) in gl.x.iter().zip(&gl.w) {
            let th = half_pi * xi;
            let (s, c) = th.sin_cos();
            let x = self.radius * s;
            let dx = self.radius * c * half_pi * wi;
            let yw = self.radius * c;
            let p = |y: f64, z: f64| self.center + u * x + v * y + w * z;
            let corners = [p(-yw, -hh), p(yw, -hh), p(yw, hh), p(-yw, hh)];
            let cur = m * dx;
            for k in 0..4 {
                b += segment_field(&corners[k], &corners[(k + 1) % 4], cur, r);
            }
        }
        b
    }

    /// Поле моделью магнитных зарядов (второй способ): σ = M·cos φ на боковой
    /// поверхности, H = (1/4π)∮σ(r − r')/|r − r'|³ dA', B = μ₀H вне магнита.
    /// Интеграл по высоте — аналитически, по φ — правило трапеций (`n_phi` узлов).
    pub fn b_charge_model(&self, r: &Vec3, n_phi: usize) -> Vec3 {
        let (u, v, w) = self.basis();
        let d = r - self.center;
        let (x, y, z) = (d.dot(&u), d.dot(&v), d.dot(&w));
        let m = self.br / MU0;
        let hh = 0.5 * self.thickness;
        let rr = self.radius;
        let h = 2.0 * PI / n_phi as f64;
        let (mut hx, mut hy, mut hz) = (0.0, 0.0, 0.0);
        for k in 0..n_phi {
            let phi = k as f64 * h;
            let (sp, cp) = phi.sin_cos();
            let (dx, dy) = (x - rr * cp, y - rr * sp);
            let rho2 = dx * dx + dy * dy;
            let fz = |s: f64| s / (rho2 * (rho2 + s * s).sqrt());
            let gz = |s: f64| -1.0 / (rho2 + s * s).sqrt();
            let i1 = fz(z + hh) - fz(z - hh);
            let i2 = gz(z + hh) - gz(z - hh);
            hx += cp * dx * i1;
            hy += cp * dy * i1;
            hz += cp * i2;
        }
        let k = m * rr * h / (4.0 * PI);
        (u * hx + v * hy + w * hz) * (k * MU0)
    }

    /// Дипольный момент (вектор), А·м².
    pub fn moment(&self) -> Vec3 {
        self.mag_dir * (self.br / MU0 * PI * self.radius * self.radius * self.thickness)
    }
}

impl FieldSource for DiametricDiscMagnet {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        self.b_current_model(r)
    }
}

/// Магнитный датчик угла на эффекте Холла: допустимое окно амплитуды B_z на окружности
/// элементов Холла.
#[derive(Clone, Copy, Debug)]
pub struct HallAngleSensor {
    pub name: &'static str,
    pub bz_min_t: f64,
    pub bz_max_t: f64,
    /// Радиус окружности элементов Холла на кристалле, м.
    pub hall_radius_m: f64,
    pub source: &'static str,
}

/// ams-OSRAM AS5600: B_z 30…90 мТ на окружности радиусом 1 мм (поверхность кристалла),
/// рекомендуемый зазор 0.5–3 мм «в зависимости от магнита». Даташит AS5600 v1-06,
/// https://www.mouser.com/pdfdocs/AMS_AS5600_Datasheet_EN.PDF (табл. «Magnetic
/// Characteristics», рис. «Magnet placement»).
pub const AS5600: HallAngleSensor = HallAngleSensor {
    name: "ams-OSRAM AS5600",
    bz_min_t: 0.030,
    bz_max_t: 0.090,
    hall_radius_m: 1.0e-3,
    source: "ams AS5600 datasheet v1-06 (2018), Magnetic Characteristics: Bz 30–90 mT; airgap 0.5–3 mm",
};

/// Амплитуда B_z в плоскости датчика на расстоянии `gap` (м) под торцом диаметрального
/// диска, на радиусе элементов Холла в направлении намагниченности (там B_z максимальна
/// по углу; B_z ∝ cos(φ − φ_магнита)). Магнит соосен с датчиком.
pub fn sensor_bz_amplitude(
    radius: f64,
    thickness: f64,
    br: f64,
    gap: f64,
    hall_radius: f64,
) -> f64 {
    let m = DiametricDiscMagnet::new(Vec3::zeros(), Vec3::z(), Vec3::x(), radius, thickness, br);
    let p = Vec3::new(hall_radius, 0.0, -0.5 * thickness - gap);
    m.b_current_model(&p).z.abs()
}

/// Окно зазоров (мин, макс), в котором амплитуда B_z попадает в окно датчика.
/// None — если даже вплотную поле меньше минимума. Предполагается, что B_z(g) убывает
/// с зазором (так для дисков с R ≫ радиуса элементов Холла).
///
/// Пример (модель): Ø6×2.5 мм N35 для AS5600 — окно ≈ 0.09…2.5 мм от плоскости элементов
/// Холла (N42 — до 2.7 мм); даташит говорит «0.5–3 мм в зависимости от магнита» —
/// согласуется качественно. Кривой B_z(зазор) от производителя для количественной сверки
/// не нашли; зазор «кристалл ↔ верх корпуса» (~0.2–0.5 мм, зависит от корпуса) нужно
/// прибавить к нашему при сравнении с «airgap» по даташиту.
pub fn sensor_gap_window(
    radius: f64,
    thickness: f64,
    br: f64,
    sensor: &HallAngleSensor,
) -> Option<(f64, f64)> {
    let b = |g: f64| sensor_bz_amplitude(radius, thickness, br, g, sensor.hall_radius_m);
    let gmin_probe = 0.05e-3;
    if b(gmin_probe) < sensor.bz_min_t {
        return None;
    }
    let solve = |target: f64| {
        // B_z(g) убывает с зазором на интересующем участке — бисекция
        let (mut lo, mut hi) = (gmin_probe, 50e-3);
        if b(lo) <= target {
            return lo;
        }
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if b(mid) > target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    };
    Some((solve(sensor.bz_max_t), solve(sensor.bz_min_t)))
}

// ---------------------------------------------------------------------------------------
// Сверки
// ---------------------------------------------------------------------------------------

/// Сила отрыва K&J Magnetics D88-N52 (Ø12.7 × 12.7 мм, N52, осевой) от толстой
/// шлифованной стальной плиты («Pull Force, Case 1»): 18.08 lb = 80.42 Н, среднее по
/// 5 образцам. https://www.kjmagnetics.com/proddetail.asp?prod=D88-N52 ;
/// методика: https://www.kjmagnetics.com/blog/testing-magnet-strength (плита ≥ 1″).
pub const KJ_D88_N52_PULL_N: f64 = 18.08 * 4.448_221_615_260_5;

/// Все сверки модуля магнитов.
pub fn checks() -> Vec<Check> {
    let mut v = Vec::new();
    let n42 = ndfeb_grade("N42").expect("N42");
    let br = n42.br_typical_t();
    let (rr, ll) = (5e-3, 10e-3);
    let mag = AxialCylinderMagnet::new(Vec3::zeros(), Vec3::z(), rr, ll, br);

    // 1. Поле на оси: модель слоя (Derby–Olbert) против формулы Br/2[...].
    {
        let z = 2e-3; // 2 мм от торца
        let sim = mag.b_at(&Vec3::new(0.0, 0.0, 0.5 * ll + z)).z;
        v.push(Check {
            id: "em_magnet_axis_formula",
            title: "Осевой цилиндр Ø10×10 N42: B на оси в 2 мм от торца, модель тока против формулы на оси".into(),
            simulated: sim,
            reference: on_axis_field_from_face(br, rr, ll, z),
            unit: "Тл",
            method: "поверхностный ток K = Br/μ₀, замкнутая формула Derby–Olbert".into(),
            reference_method: "B(z) = Br/2·[(z+L)/√(R²+(z+L)²) − z/√(R²+z²)]".into(),
            tolerance: Tolerance::Relative(1e-12),
            note: format!("Br = {br:.3} Тл (середина диапазона N42)."),
        });
    }
    // 2. Вне оси: замкнутая формула против стопки витков (у края торца).
    {
        let (rho, z) = (0.9 * rr, 0.5 * ll + 0.5e-3);
        let (dr, dz) = mag.field_local(rho, z);
        let (lr, lz) = sheet_cylinder_field_by_loops(rr, 0.5 * ll, br / MU0, rho, z, 128);
        v.push(Check {
            id: "em_magnet_offaxis_loops",
            title: "Осевой цилиндр: |B| у края торца (ρ = 0.9R, 0.5 мм над торцом), формула против стопки витков".into(),
            simulated: (dr * dr + dz * dz).sqrt(),
            reference: (lr * lr + lz * lz).sqrt(),
            unit: "Тл",
            method: "Derby–Olbert (cel Булирша)".into(),
            reference_method: "сумма полей витков (K, E), Гаусс–Лежандр 128×16 по высоте".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: format!("B_ρ {dr:.6} / {lr:.6} Тл, B_z {dz:.6} / {lz:.6} Тл."),
        });
    }
    // 3. Дальнее поле → диполь.
    {
        let rdist = 40.0 * ll;
        let p = Vec3::new(0.6, 0.0, 0.8) * rdist;
        let sim = mag.b_at(&p);
        let dip = dipole_field(&Vec3::new(0.0, 0.0, mag.moment()), &p);
        v.push(Check {
            id: "em_magnet_far_dipole",
            title: "Осевой цилиндр: дальнее поле (r = 40L) против точечного диполя m = Br·V/μ₀"
                .into(),
            simulated: sim.norm(),
            reference: dip.norm(),
            unit: "Тл",
            method: "модель поверхностного тока".into(),
            reference_method: "B = μ₀/(4π)·(3(m·r̂)r̂ − m)/r³".into(),
            tolerance: Tolerance::Relative(1e-3),
            note: "Поправка следующего (октупольного) порядка ~ (R/r)², (L/r)² ≈ 6e-4·коэф.".into(),
        });
    }

    // 4–5. Сила между соосными магнитами: витки против Ганкеля.
    for (id, gap) in [
        ("em_force_loops_vs_bessel_1mm", 1e-3),
        ("em_force_loops_vs_bessel_5mm", 5e-3),
    ] {
        let pair = CoaxialPair {
            br1: br,
            r1: rr,
            l1: ll,
            br2: br,
            r2: 4e-3,
            l2: 3e-3,
            gap,
        };
        v.push(Check {
            id,
            title: format!(
                "Сила Ø10×10 N42 на Ø8×3 N42 при зазоре {:.0} мм: витки (K, E) против интеграла Ганкеля",
                gap * 1e3
            ),
            simulated: coaxial_force_loops(&pair),
            reference: coaxial_force_bessel(&pair),
            unit: "Н",
            method: "∫∫ сил «виток–виток» по слоям (свёртка, tanh-sinh)".into(),
            reference_method: "−μ₀πR₁R₂M₁M₂∫J₁J₁(1−e^{−kL₁})(1−e^{−kL₂})e^{−kd}/k dk (Vokoun 2009)".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: "Разные радиусы — проверяется общий случай. Знак «−» — притяжение.".into(),
        });
    }
    // 6. Дипольный предел силы.
    {
        let pair = CoaxialPair {
            br1: br,
            r1: rr,
            l1: ll,
            br2: br,
            r2: rr,
            l2: ll,
            gap: 0.39, // центр–центр 0.4 м = 40L
        };
        let (m1, m2) = pair.moments();
        v.push(Check {
            id: "em_force_dipole_limit",
            title:
                "Сила двух Ø10×10 N42 на расстоянии 40L: модель против диполь–диполь 3μ₀m₁m₂/(2πz⁴)"
                    .into(),
            simulated: coaxial_force_loops(&pair),
            reference: dipole_dipole_coaxial_force(m1, m2, pair.center_distance()),
            unit: "Н",
            method: "∫∫ сил «виток–виток»".into(),
            reference_method: "два точечных диполя на оси".into(),
            tolerance: Tolerance::Relative(2e-3),
            note: "Отличие — мультипольные поправки ~ (L/z)² ≈ 6e-4·коэф.".into(),
        });
    }
    // 7. Идеальная сталь, длинный магнит при касании → Br²A/(2μ₀).
    {
        let (r, l) = (2e-3, 60e-3);
        let f = pull_force_ideal_steel(br, r, l, 0.0);
        let bound = contact_pull_upper_bound(br, r);
        v.push(Check {
            id: "em_pull_long_magnet_bound",
            title: "Длинный магнит Ø4×60 (L/R = 30) на идеальной стали при касании → Br²A/(2μ₀)".into(),
            simulated: f,
            reference: bound,
            unit: "Н",
            method: "метод изображений: магнит + зеркальная копия, витки (K, E)".into(),
            reference_method: "давление Максвелла B²/(2μ₀) при B = Br на всём торце".into(),
            tolerance: Tolerance::Relative(0.03),
            note: "Для конечного магнита индукция на торце ниже Br (краевой эффект ~R/L) — модель должна быть чуть ниже границы.".into(),
        });
        // короткий магнит: модель строго ниже границы
        let fs = pull_force_ideal_steel(br, 5e-3, 2e-3, 0.0);
        let bs = contact_pull_upper_bound(br, 5e-3);
        v.push(Check {
            id: "em_pull_short_below_bound",
            title: "Диск Ø10×2 на идеальной стали: доля от Br²A/(2μ₀) (должна быть < 1)".into(),
            simulated: fs / bs,
            reference: 0.5,
            unit: "—",
            method: format!("метод изображений: F = {fs:.2} Н, граница {bs:.2} Н"),
            reference_method: "граница Br²A/(2μ₀); для тонкого диска рабочая точка далеко от Br"
                .into(),
            tolerance: Tolerance::Absolute(0.5),
            note: "Проверка неравенства 0 < F/F_гр < 1 (эталон 0.5 ± 0.5).".into(),
        });
    }
    // 8. K&J D88-N52 — измерение производителя.
    {
        let grade = ndfeb_grade("N52").expect("N52");
        let br52 = grade.br_typical_t();
        let (r, l) = (6.35e-3, 12.7e-3);
        let ideal = pull_force_ideal_steel(br52, r, l, 0.0);
        let coat = pull_force_ideal_steel(br52, r, l, 20e-6);
        v.push(Check {
            id: "em_pull_kj_d88_n52",
            title: "Сила отрыва D88-N52 (Ø12.7×12.7) от толстой стали: идеальная сталь против замера K&J".into(),
            simulated: ideal,
            reference: KJ_D88_N52_PULL_N,
            unit: "Н",
            method: format!(
                "метод изображений, μ_стали → ∞, Br = {br52:.3} Тл; с паразитным зазором 20 мкм — {coat:.1} Н"
            ),
            reference_method: "K&J Magnetics, Pull Force Case 1 = 18.08 lb (среднее 5 образцов)".into(),
            tolerance: Tolerance::Relative(0.15),
            note: format!(
                "ОБЪЯСНЁННОЕ РАСХОЖДЕНИЕ (модель выше на {:.0} %): идеальная сталь — верхняя оценка; \
                 у реальной стали конечная μ и насыщение у торца (B ≈ 1.3 Т в пятне контакта), \
                 покрытие Ni-Cu-Ni и шероховатость дают зазор 10–30 мкм, μ_rec магнита ≈ 1.05, \
                 Br партии может быть у нижней границы марки (−3.4 % силы при Br_min). Бюджет: зазор −3 %, \
                 Br −3.4 %, сталь (FEA с μ ≈ 1000 у QuickField/K&J — до 10 %) ⇒ допуск 15 %, не подгонка.",
                (ideal / KJ_D88_N52_PULL_N - 1.0) * 100.0
            ),
        });
    }
    // 9. Таблица марок: два независимых источника согласуются.
    for (id, name, arnold_t) in [
        ("em_grade_n42_sources", "N42", 1.315),
        ("em_grade_n52_sources", "N52", 1.450),
    ] {
        let g = ndfeb_grade(name).expect("grade");
        v.push(Check {
            id,
            title: format!(
                "Br марки {name}: середина диапазона magnet-shop против типового Arnold"
            ),
            simulated: g.br_typical_t(),
            reference: arnold_t,
            unit: "Тл",
            method: "magnet-shop.com, Technical specifications (диапазон Br)".into(),
            reference_method: "Arnold Magnetic Technologies, таблица марок (typical Br)".into(),
            tolerance: Tolerance::Relative(0.02),
            note: "Разброс марки по Br ≈ ±1.5 % — допуск по ширине диапазона.".into(),
        });
    }
    // 10. Диаметральный диск: модель тока против модели зарядов.
    {
        let dm = DiametricDiscMagnet::new(Vec3::zeros(), Vec3::z(), Vec3::x(), 3e-3, 2.5e-3, 1.19);
        let p = Vec3::new(1e-3, 0.3e-3, -1.25e-3 - 1.5e-3);
        let bc = dm.b_current_model(&p);
        let bq = dm.b_charge_model(&p, 720);
        v.push(Check {
            id: "em_diametric_current_vs_charge",
            title: "Диаметральный диск Ø6×2.5: |B| под торцом (зазор 1.5 мм) — модель тока против модели зарядов".into(),
            simulated: bc.norm(),
            reference: bq.norm(),
            unit: "Тл",
            method: "стопка прямоугольных рамок (Био–Савар отрезков), 48 срезов".into(),
            reference_method: "поверхностные заряды σ = M·cos φ, интеграл по высоте аналитически, по φ — трапеции 720".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: format!("Компоненты (ток / заряды): Bz {:.6} / {:.6} Тл.", bc.z, bq.z),
        });
        let far = Vec3::new(0.3, 0.2, 0.25);
        v.push(Check {
            id: "em_diametric_far_dipole",
            title: "Диаметральный диск: дальнее поле (r ≈ 0.44 м) против диполя".into(),
            simulated: dm.b_current_model(&far).norm(),
            reference: dipole_field(&dm.moment(), &far).norm(),
            unit: "Тл",
            method: "стопка рамок".into(),
            reference_method: "точечный диполь m = Br·V/μ₀ вдоль x".into(),
            tolerance: Tolerance::Relative(1e-4),
            note: String::new(),
        });
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_form_loop_force_matches() {
        let (a, b, z) = (0.03_f64, 0.02_f64, 0.011_f64);
        let (i1, i2) = (2.0, 3.0);
        let (k, e) = crate::math::ellip_ke(4.0 * a * b / ((a + b).powi(2) + z * z));
        let cf = MU0 * i1 * i2 * z / ((a + b).powi(2) + z * z).sqrt()
            * (k - (a * a + b * b + z * z) / ((a - b).powi(2) + z * z) * e);
        let f = loop_loop_axial_force(a, i1, b, i2, z);
        assert!((f - cf).abs() < 1e-12 * cf.abs(), "{f} {cf}");
        assert!(f < 0.0, "одинаково направленные токи притягиваются");
    }

    #[test]
    fn temp_class_parsing() {
        assert_eq!(temp_class_of("N42SH").unwrap().max_operating_c, 150.0);
        assert_eq!(temp_class_of("N52").unwrap().max_operating_c, 80.0);
        assert_eq!(temp_class_of("N35AH").unwrap().max_operating_c, 220.0);
        assert!((br_at_temperature(1.3, 80.0) - 1.3 * (1.0 - 0.072)).abs() < 1e-12);
    }

    #[test]
    fn antiparallel_magnets_repel() {
        let p = CoaxialPair {
            br1: 1.3,
            r1: 5e-3,
            l1: 5e-3,
            br2: -1.3,
            r2: 5e-3,
            l2: 5e-3,
            gap: 2e-3,
        };
        assert!(coaxial_force_loops(&p) > 0.0);
    }
}
