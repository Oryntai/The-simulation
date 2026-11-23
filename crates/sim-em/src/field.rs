//! Поля постоянных токов (магнитостатика в вакууме, μ = μ₀ всюду).
//!
//! * прямой отрезок — точная формула Био–Савара для конечного отрезка в устойчивой форме
//!   Hanson & Hirshman, Phys. Plasmas 9 (2002) 4410, doi:10.1063/1.1507589;
//! * круговой виток — точное поле в любой точке через полные эллиптические интегралы K, E
//!   (Smythe, «Static and Dynamic Electricity», 1950, §7.10; в удобной записи —
//!   Simpson et al., NASA/TM-2013-217919);
//! * соленоид / цилиндрический токовый слой — на оси аналитика, вне оси — замкнутая формула
//!   Derby & Olbert, Am. J. Phys. 78 (2010) 229, doi:10.1119/1.3256157 (через `cel` Булирша);
//! * силовые линии — RK4 по длине дуги вдоль B/|B|.
//!
//! Допущения: нити бесконечно тонкие, ток постоянный (магнитостатика), среда — вакуум.
//! На самой нити поле сингулярно: функции возвращают NaN/∞ — не подставляйте точки на проводе.

use nalgebra::Vector3;
use sim_verify::{Check, Tolerance};
use std::f64::consts::PI;

use crate::MU0;
use crate::math::{GaussLegendre, cel, ellip_ke_mc};

pub type Vec3 = Vector3<f64>;

/// Любой источник поля: даёт индукцию B (Тл) в точке `r` (м, мировая СК).
pub trait FieldSource {
    fn b_at(&self, r: &Vec3) -> Vec3;
}

/// Набор источников — поле складывается (принцип суперпозиции).
#[derive(Default)]
pub struct FieldScene {
    pub sources: Vec<Box<dyn FieldSource + Send + Sync>>,
}

impl FieldScene {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn push(&mut self, s: impl FieldSource + Send + Sync + 'static) {
        self.sources.push(Box::new(s));
    }
}

impl FieldSource for FieldScene {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        self.sources.iter().map(|s| s.b_at(r)).sum()
    }
}

/// Ортонормированный базис (u, v, w) с w = `axis` (нормируется).
pub fn frame_from_axis(axis: &Vec3) -> (Vec3, Vec3, Vec3) {
    let w = axis.normalize();
    let helper = if w.x.abs() < 0.9 {
        Vec3::x()
    } else {
        Vec3::y()
    };
    let u = helper.cross(&w).normalize();
    let v = w.cross(&u);
    (u, v, w)
}

// ---------------------------------------------------------------------------------------
// Прямой отрезок и ломаная
// ---------------------------------------------------------------------------------------

/// Поле конечного прямого отрезка тока `current` (А), текущего от `p1` к `p2`, в точке `r`.
/// Формула Hanson & Hirshman (2002), ур. (5):
/// B = μ₀I/(4π) · (ê × Rᵢ) · 2L(Rᵢ + R_f) / (Rᵢ·R_f·((Rᵢ + R_f)² − L²)),
/// где ê — орт отрезка, Rᵢ = r − p1, R_f = r − p2, L — длина. Эквивалентна классической
/// μ₀I/(4πd)·(cos θ₁ − cos θ₂), но без вычитания близких чисел.
/// На прямой, содержащей отрезок, возвращает 0 вне отрезка и NaN на нём.
pub fn segment_field(p1: &Vec3, p2: &Vec3, current: f64, r: &Vec3) -> Vec3 {
    let l_vec = p2 - p1;
    let l = l_vec.norm();
    if l == 0.0 {
        return Vec3::zeros();
    }
    let e = l_vec / l;
    let ri = r - p1;
    let rf = r - p2;
    let (ri_n, rf_n) = (ri.norm(), rf.norm());
    let s = ri_n + rf_n;
    let denom = ri_n * rf_n * (s * s - l * l);
    let cross = e.cross(&ri);
    if denom <= 0.0 {
        return Vec3::repeat(f64::NAN);
    }
    if cross.norm_squared() == 0.0 {
        return Vec3::zeros();
    }
    cross * (MU0 * current / (4.0 * PI) * 2.0 * l * s / denom)
}

/// Ломаная с током: последовательность точек; `closed` — замкнуть последним отрезком.
/// Разомкнутая ломаная нарушает непрерывность тока (∇·J ≠ 0 на концах) — для неё закон
/// Ампера не выполняется; физичны только замкнутые контуры или их части в сумме.
#[derive(Clone, Debug)]
pub struct Polyline {
    pub points: Vec<Vec3>,
    pub current: f64,
    pub closed: bool,
}

impl Polyline {
    pub fn new(points: Vec<Vec3>, current: f64, closed: bool) -> Self {
        Self {
            points,
            current,
            closed,
        }
    }

    /// Правильный N-угольник, вписанный в окружность радиуса `radius` с центром `center`
    /// и осью `axis`; ток течёт против часовой стрелки, если смотреть с конца оси.
    pub fn regular_polygon(center: Vec3, axis: Vec3, radius: f64, n: usize, current: f64) -> Self {
        let (u, v, _) = frame_from_axis(&axis);
        let pts = (0..n)
            .map(|k| {
                let t = 2.0 * PI * k as f64 / n as f64;
                center + radius * (u * t.cos() + v * t.sin())
            })
            .collect();
        Self::new(pts, current, true)
    }

    /// Прямоугольный контур в плоскости (u, v) со сторонами `a` (вдоль u) и `b` (вдоль v).
    pub fn rectangle(center: Vec3, u: Vec3, v: Vec3, a: f64, b: f64, current: f64) -> Self {
        let (hu, hv) = (u.normalize() * (0.5 * a), v.normalize() * (0.5 * b));
        Self::new(
            vec![
                center - hu - hv,
                center + hu - hv,
                center + hu + hv,
                center - hu + hv,
            ],
            current,
            true,
        )
    }

    /// Отрезки (начало, конец).
    pub fn segments(&self) -> impl Iterator<Item = (&Vec3, &Vec3)> {
        let n = self.points.len();
        let m = if self.closed { n } else { n.saturating_sub(1) };
        (0..m).map(move |k| (&self.points[k], &self.points[(k + 1) % n]))
    }
}

impl FieldSource for Polyline {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        self.segments()
            .map(|(a, b)| segment_field(a, b, self.current, r))
            .sum()
    }
}

// ---------------------------------------------------------------------------------------
// Круговой виток
// ---------------------------------------------------------------------------------------

/// Поле кругового витка радиуса `a` с током `i` в его собственных цилиндрических
/// координатах (виток в плоскости z = 0, центр в начале, ток против часовой стрелки с +z).
/// Возвращает (B_ρ, B_z). Точная формула (Smythe 1950; Simpson et al. 2013):
/// α² = (a−ρ)² + z², β² = (a+ρ)² + z², k² = 1 − α²/β², C = μ₀I/π,
/// B_z = C/(2α²β)·[(a² − ρ² − z²)·E + α²·K],
/// B_ρ = C·z/(2α²βρ)·[(a² + ρ² + z²)·E − α²·K].
/// Около оси (ρ → 0) B_ρ считается рядом B_ρ ≈ 3μ₀I a² z ρ / (4(a²+z²)^{5/2}),
/// чтобы не терять точность на взаимном сокращении K и E.
pub fn loop_field_local(a: f64, i: f64, rho: f64, z: f64) -> (f64, f64) {
    let rho = rho.abs();
    let alpha2 = (a - rho) * (a - rho) + z * z;
    let beta2 = (a + rho) * (a + rho) + z * z;
    if alpha2 == 0.0 {
        return (f64::NAN, f64::NAN);
    }
    let beta = beta2.sqrt();
    let (k, e) = ellip_ke_mc(alpha2 / beta2);
    let c = MU0 * i / PI;
    let bz = c / (2.0 * alpha2 * beta) * ((a * a - rho * rho - z * z) * e + alpha2 * k);
    let s2 = a * a + z * z;
    let brho = if rho < 3e-6 * s2.sqrt() {
        3.0 * MU0 * i * a * a * z * rho / (4.0 * s2.powf(2.5))
    } else {
        c * z / (2.0 * alpha2 * beta * rho) * ((a * a + rho * rho + z * z) * e - alpha2 * k)
    };
    (brho, bz)
}

/// Азимутальная компонента векторного потенциала витка A_φ(ρ, z) (Т·м):
/// A_φ = μ₀I/(πk)·√(a/ρ)·[(1 − k²/2)K − E] (Smythe 1950, §7.10; Jackson, 3-е изд., (5.37)).
/// Магнитный поток через круг радиуса ρ на высоте z: Ψ = 2πρ·A_φ — постоянен вдоль
/// силовой линии осесимметричного поля (используется для сверки трассировки).
pub fn loop_vector_potential_local(a: f64, i: f64, rho: f64, z: f64) -> f64 {
    let rho = rho.abs();
    if rho == 0.0 {
        return 0.0;
    }
    let alpha2 = (a - rho) * (a - rho) + z * z;
    let beta2 = (a + rho) * (a + rho) + z * z;
    let mc = alpha2 / beta2;
    let m = 4.0 * a * rho / beta2;
    let bracket = if m < 1e-3 {
        // ряд (1 − m/2)K − E = (π/2)(m²/16 + 3m³/64 + 75m⁴/2048 + …)
        0.5 * PI * (m * m / 16.0 + 3.0 * m.powi(3) / 64.0 + 75.0 * m.powi(4) / 2048.0)
    } else {
        let (k, e) = ellip_ke_mc(mc);
        (1.0 - 0.5 * m) * k - e
    };
    MU0 * i / (PI * m.sqrt()) * (a / rho).sqrt() * bracket
}

/// Круговой виток в пространстве.
#[derive(Clone, Copy, Debug)]
pub struct CircularLoop {
    pub center: Vec3,
    /// Ось (нормаль); ток против часовой стрелки, если смотреть с конца оси.
    pub axis: Vec3,
    pub radius: f64,
    pub current: f64,
}

impl CircularLoop {
    pub fn new(center: Vec3, axis: Vec3, radius: f64, current: f64) -> Self {
        Self {
            center,
            axis: axis.normalize(),
            radius,
            current,
        }
    }

    /// Поле на оси (замкнутая формула): B = μ₀I a² / (2(a² + z²)^{3/2}).
    pub fn on_axis_closed_form(radius: f64, current: f64, z: f64) -> f64 {
        MU0 * current * radius * radius / (2.0 * (radius * radius + z * z).powf(1.5))
    }

    /// Магнитный момент m = I·π a² (А·м²).
    pub fn moment(&self) -> f64 {
        self.current * PI * self.radius * self.radius
    }
}

/// Перевод осесимметричного поля (B_ρ, B_z), заданного функцией в локальных (ρ, z),
/// в мировой вектор.
fn axisym_to_world(
    center: &Vec3,
    axis: &Vec3,
    r: &Vec3,
    f: impl Fn(f64, f64) -> (f64, f64),
) -> Vec3 {
    let d = r - center;
    let z = d.dot(axis);
    let radial = d - axis * z;
    let rho = radial.norm();
    let (brho, bz) = f(rho, z);
    let mut b = axis * bz;
    if rho > 0.0 {
        b += radial * (brho / rho);
    }
    b
}

impl FieldSource for CircularLoop {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        axisym_to_world(&self.center, &self.axis, r, |rho, z| {
            loop_field_local(self.radius, self.current, rho, z)
        })
    }
}

// ---------------------------------------------------------------------------------------
// Цилиндрический токовый слой (соленоид, эквивалент осевого магнита)
// ---------------------------------------------------------------------------------------

/// Поле однородного азимутального токового слоя на цилиндре радиуса `a`, длины 2·`b`
/// (центр в начале, ось z), линейная плотность тока `k_sheet` (А/м; для соленоида n·I,
/// для магнита M = Br/μ₀). Возвращает (B_ρ, B_z).
///
/// Формула Derby & Olbert (2010), ур. (6)–(7): B₀ = μ₀K/π, z± = z ± b,
/// α± = a/√(z±² + (ρ+a)²), β± = z±/√(z±² + (ρ+a)²), γ = (a−ρ)/(a+ρ),
/// k± = √((z±² + (a−ρ)²)/(z±² + (a+ρ)²)),
/// B_ρ = B₀·[α₊·cel(k₊,1,1,−1) − α₋·cel(k₋,1,1,−1)],
/// B_z = B₀·a/(a+ρ)·[β₊·cel(k₊,γ²,1,γ) − β₋·cel(k₋,γ²,1,γ)].
/// Особые точки — рёбра слоя (ρ = a, z = ±b): возвращается NaN.
pub fn sheet_cylinder_field_local(a: f64, b: f64, k_sheet: f64, rho: f64, z: f64) -> (f64, f64) {
    let rho = rho.abs();
    let b0 = MU0 * k_sheet / PI;
    let gamma = (a - rho) / (a + rho);
    let term = |zz: f64| {
        let den = (zz * zz + (rho + a) * (rho + a)).sqrt();
        let alpha = a / den;
        let beta = zz / den;
        let kc = ((zz * zz + (a - rho) * (a - rho)) / (zz * zz + (a + rho) * (a + rho))).sqrt();
        (
            alpha * cel(kc, 1.0, 1.0, -1.0),
            beta * cel(kc, gamma * gamma, 1.0, gamma),
        )
    };
    let (rp, zp) = term(z + b);
    let (rm, zm) = term(z - b);
    (b0 * (rp - rm), b0 * a / (a + rho) * (zp - zm))
}

/// Поле того же слоя на оси (аналитика):
/// B_z(z) = μ₀K/2·[(z + b)/√((z+b)² + a²) − (z − b)/√((z−b)² + a²)].
pub fn sheet_cylinder_on_axis(a: f64, b: f64, k_sheet: f64, z: f64) -> f64 {
    0.5 * MU0
        * k_sheet
        * ((z + b) / ((z + b).powi(2) + a * a).sqrt() - (z - b) / ((z - b).powi(2) + a * a).sqrt())
}

/// Тот же слой, посчитанный как стопка круговых витков (квадратура Гаусса–Лежандра по z) —
/// второй, независимый способ для сверки замкнутой формулы.
pub fn sheet_cylinder_field_by_loops(
    a: f64,
    b: f64,
    k_sheet: f64,
    rho: f64,
    z: f64,
    panels: usize,
) -> (f64, f64) {
    let gl = GaussLegendre::new(16);
    let br = gl.integrate_composite(-b, b, panels, |zs| {
        loop_field_local(a, k_sheet, rho, z - zs).0
    });
    let bz = gl.integrate_composite(-b, b, panels, |zs| {
        loop_field_local(a, k_sheet, rho, z - zs).1
    });
    (br, bz)
}

/// Конечный идеальный соленоид: тонкий токовый слой, n = turns/length.
/// Допущение: шаг намотки много меньше радиуса и расстояния до точки (дискретность витков
/// и спиральность не учитываются).
#[derive(Clone, Copy, Debug)]
pub struct Solenoid {
    pub center: Vec3,
    pub axis: Vec3,
    pub radius: f64,
    pub length: f64,
    pub turns: f64,
    pub current: f64,
}

impl Solenoid {
    pub fn new(
        center: Vec3,
        axis: Vec3,
        radius: f64,
        length: f64,
        turns: f64,
        current: f64,
    ) -> Self {
        Self {
            center,
            axis: axis.normalize(),
            radius,
            length,
            turns,
            current,
        }
    }
    /// Линейная плотность тока n·I (А/м).
    pub fn sheet_current(&self) -> f64 {
        self.turns / self.length * self.current
    }
    /// B на оси на расстоянии `z` от центра (аналитика).
    pub fn on_axis(&self, z: f64) -> f64 {
        sheet_cylinder_on_axis(self.radius, 0.5 * self.length, self.sheet_current(), z)
    }
    /// Поле бесконечного соленоида μ₀·n·I — предел для сверки.
    pub fn infinite_limit(&self) -> f64 {
        MU0 * self.sheet_current()
    }
}

impl FieldSource for Solenoid {
    fn b_at(&self, r: &Vec3) -> Vec3 {
        axisym_to_world(&self.center, &self.axis, r, |rho, z| {
            sheet_cylinder_field_local(self.radius, 0.5 * self.length, self.sheet_current(), rho, z)
        })
    }
}

// ---------------------------------------------------------------------------------------
// Силовые линии
// ---------------------------------------------------------------------------------------

/// Параметры трассировки силовой линии.
#[derive(Clone, Copy, Debug)]
pub struct TraceOptions {
    /// Шаг по длине дуги, м.
    pub step: f64,
    /// Максимум шагов в каждую сторону.
    pub max_steps: usize,
    /// Линия обрывается, если ушла дальше этого радиуса от `bounds_center`.
    pub bounds_center: Vec3,
    pub bounds_radius: f64,
    /// Линия обрывается при |B| ниже этого порога (Тл) или при NaN (точка на проводе).
    pub min_field: f64,
    /// Трассировать в обе стороны от затравки (по полю и против).
    pub both_directions: bool,
}

impl Default for TraceOptions {
    fn default() -> Self {
        Self {
            step: 1e-3,
            max_steps: 5000,
            bounds_center: Vec3::zeros(),
            bounds_radius: 1.0,
            min_field: 1e-12,
            both_directions: true,
        }
    }
}

/// Одна силовая линия B от затравки `seed`: интегрирование dr/ds = ±B/|B| методом RK4
/// с постоянным шагом по длине дуги. Возвращает ломаную (для 3D-отрисовки).
/// Замкнутая линия (вернулась к затравке ближе 0.6 шага после пробега > 10 шагов)
/// замыкается и обрывается.
pub fn trace_field_line(src: &dyn FieldSource, seed: Vec3, opt: &TraceOptions) -> Vec<Vec3> {
    let forward = trace_one_way(src, seed, opt, 1.0);
    if !opt.both_directions || forward.closed {
        return forward.points;
    }
    let backward = trace_one_way(src, seed, opt, -1.0);
    let mut pts: Vec<Vec3> = backward.points.into_iter().rev().collect();
    pts.pop(); // затравка повторяется
    pts.extend(forward.points);
    pts
}

/// Несколько линий от набора затравок.
pub fn trace_field_lines(
    src: &dyn FieldSource,
    seeds: &[Vec3],
    opt: &TraceOptions,
) -> Vec<Vec<Vec3>> {
    seeds
        .iter()
        .map(|s| trace_field_line(src, *s, opt))
        .collect()
}

/// Затравки на окружности (удобно для витков, соленоидов и магнитов).
pub fn seeds_on_circle(center: Vec3, axis: Vec3, radius: f64, n: usize) -> Vec<Vec3> {
    let (u, v, _) = frame_from_axis(&axis);
    (0..n)
        .map(|k| {
            let t = 2.0 * PI * k as f64 / n as f64;
            center + radius * (u * t.cos() + v * t.sin())
        })
        .collect()
}

struct Traced {
    points: Vec<Vec3>,
    closed: bool,
}

fn trace_one_way(src: &dyn FieldSource, seed: Vec3, opt: &TraceOptions, sign: f64) -> Traced {
    let dir = |p: &Vec3| -> Option<Vec3> {
        let b = src.b_at(p);
        let n = b.norm();
        if !n.is_finite() || n < opt.min_field {
            None
        } else {
            Some(b * (sign / n))
        }
    };
    let h = opt.step;
    let mut pts = vec![seed];
    let mut p = seed;
    for k in 0..opt.max_steps {
        let Some(k1) = dir(&p) else { break };
        let Some(k2) = dir(&(p + k1 * (0.5 * h))) else {
            break;
        };
        let Some(k3) = dir(&(p + k2 * (0.5 * h))) else {
            break;
        };
        let Some(k4) = dir(&(p + k3 * h)) else { break };
        p += (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (h / 6.0);
        if (p - opt.bounds_center).norm() > opt.bounds_radius {
            pts.push(p);
            break;
        }
        if k > 10 && (p - seed).norm() < 0.6 * h {
            pts.push(seed);
            return Traced {
                points: pts,
                closed: true,
            };
        }
        pts.push(p);
    }
    Traced {
        points: pts,
        closed: false,
    }
}

// ---------------------------------------------------------------------------------------
// Численные операторы для сверок
// ---------------------------------------------------------------------------------------

/// Дивергенция B центральными разностями с шагом h.
pub fn divergence(src: &dyn FieldSource, r: &Vec3, h: f64) -> f64 {
    let mut d = 0.0;
    for k in 0..3 {
        let mut e = Vec3::zeros();
        e[k] = h;
        d += (src.b_at(&(r + e))[k] - src.b_at(&(r - e))[k]) / (2.0 * h);
    }
    d
}

/// Циркуляция ∮B·dl по окружности (центр, ось, радиус) правилом трапеций (для периодической
/// гладкой функции сходится экспоненциально). Обход — против часовой стрелки с конца оси.
pub fn circulation_on_circle(
    src: &dyn FieldSource,
    center: Vec3,
    axis: Vec3,
    radius: f64,
    n: usize,
) -> f64 {
    let (u, v, _) = frame_from_axis(&axis);
    let h = 2.0 * PI / n as f64;
    (0..n)
        .map(|k| {
            let t = k as f64 * h;
            let p = center + radius * (u * t.cos() + v * t.sin());
            let tangent = (-u * t.sin() + v * t.cos()) * radius;
            src.b_at(&p).dot(&tangent)
        })
        .sum::<f64>()
        * h
}

// ---------------------------------------------------------------------------------------
// Сверки
// ---------------------------------------------------------------------------------------

/// Все сверки модуля полей токов.
pub fn checks() -> Vec<Check> {
    let mut v = Vec::new();
    let (a, i) = (0.05, 10.0);
    let lp = CircularLoop::new(Vec3::zeros(), Vec3::z(), a, i);

    // 1. Виток на оси: замкнутая формула против эллиптической при ρ = 0 и ρ → 0.
    {
        let z = 0.7 * a;
        let ell = lp.b_at(&Vec3::new(1e-9 * a, 0.0, z)).z;
        let cf = CircularLoop::on_axis_closed_form(a, i, z);
        v.push(Check {
            id: "em_loop_axis_elliptic",
            title: "Виток: эллиптическая формула на оси = замкнутой μ₀Ia²/(2(a²+z²)^{3/2})".into(),
            simulated: ell,
            reference: cf,
            unit: "Тл",
            method: "K, E через AGM в точке ρ = 1e-9·a, z = 0.7a".into(),
            reference_method: "закон Био–Савара на оси, интеграл в элементарных функциях".into(),
            tolerance: Tolerance::Relative(1e-12),
            note: "Проверяет и AGM, и предел ρ → 0 (ряд для B_ρ около оси).".into(),
        });
    }

    // 2. Многоугольник из N отрезков (точный Био–Савар отрезков) → окружность: сходимость O(1/N²).
    {
        let p_off = Vec3::new(0.6 * a, 0.2 * a, 0.3 * a);
        let p_axis = Vec3::new(0.0, 0.0, 0.4 * a);
        let exact_off = lp.b_at(&p_off);
        let exact_axis = lp.b_at(&p_axis);
        let err = |n: usize, p: &Vec3, ex: &Vec3| {
            let poly = Polyline::regular_polygon(Vec3::zeros(), Vec3::z(), a, n, i);
            (poly.b_at(p) - ex).norm() / ex.norm()
        };
        let (e1, e2) = (err(256, &p_off, &exact_off), err(512, &p_off, &exact_off));
        let poly = Polyline::regular_polygon(Vec3::zeros(), Vec3::z(), a, 2048, i);
        v.push(Check {
            id: "em_polygon_loop_offaxis",
            title: "Ломаная из 2048 отрезков против эллиптической формулы витка (вне оси)".into(),
            simulated: poly.b_at(&p_off).norm(),
            reference: exact_off.norm(),
            unit: "Тл",
            method: "сумма точных полей отрезков (Hanson–Hirshman), вписанный 2048-угольник".into(),
            reference_method: "поле витка через K, E (Smythe)".into(),
            tolerance: Tolerance::Relative(1e-5),
            note: "Точка (0.6a, 0.2a, 0.3a). Вписанный многоугольник занижает площадь на ~(2π²/3)/N² — отсюда остаток.".into(),
        });
        v.push(Check {
            id: "em_polygon_loop_axis",
            title: "Ломаная из 2048 отрезков против эллиптической формулы витка (на оси)".into(),
            simulated: poly.b_at(&p_axis).z,
            reference: exact_axis.z,
            unit: "Тл",
            method: "сумма точных полей отрезков, вписанный 2048-угольник".into(),
            reference_method: "поле витка через K, E".into(),
            tolerance: Tolerance::Relative(1e-5),
            note: String::new(),
        });
        v.push(Check {
            id: "em_polygon_convergence_order",
            title: "Порядок сходимости ломаной к витку: log₂(e₂₅₆/e₅₁₂) ≈ 2".into(),
            simulated: (e1 / e2).log2(),
            reference: 2.0,
            unit: "—",
            method: format!("ошибки e₂₅₆ = {e1:.3e}, e₅₁₂ = {e2:.3e}"),
            reference_method: "ошибка вписанного многоугольника ∝ 1/N² (разложение sin(π/N))"
                .into(),
            tolerance: Tolerance::Absolute(0.05),
            note:
                "Сходимость по N показывает, что расхождение — дискретизация, а не ошибка формул."
                    .into(),
        });
    }

    // 3. Длинный соленоид: центр против μ₀nI.
    {
        let s = Solenoid::new(Vec3::zeros(), Vec3::z(), 0.01, 2.0, 4000.0, 1.5);
        let b = s.b_at(&Vec3::zeros()).z;
        v.push(Check {
            id: "em_long_solenoid_centre",
            title: "Длинный соленоид (L = 200R): B в центре против μ₀·n·I".into(),
            simulated: b,
            reference: s.infinite_limit(),
            unit: "Тл",
            method: "формула Derby–Olbert (cel Булирша), точка ρ = 0, z = 0".into(),
            reference_method: "бесконечный соленоид, закон Ампера".into(),
            tolerance: Tolerance::Relative(1e-4),
            note: "Ожидаемое отличие конечной длины 1 − L/√(L²+4R²) ≈ 5e-5.".into(),
        });
        // на оси: Derby–Olbert против аналитики на оси
        let z = 0.9;
        v.push(Check {
            id: "em_solenoid_axis_formula",
            title: "Соленоид: Derby–Olbert на оси против аналитики на оси (у торца)".into(),
            simulated: s.b_at(&Vec3::new(0.0, 0.0, z)).z,
            reference: s.on_axis(z),
            unit: "Тл",
            method: "Derby–Olbert при ρ = 0".into(),
            reference_method: "μ₀nI/2·[(z+b)/√((z+b)²+a²) − (z−b)/√((z−b)²+a²)]".into(),
            tolerance: Tolerance::Relative(1e-12),
            note: String::new(),
        });
    }

    // 4. Соленоид вне оси: замкнутая формула против стопки витков.
    {
        let (ra, hb, kk) = (0.02, 0.05, 1.0e4);
        for (id, rho, z) in [
            ("em_solenoid_offaxis_inside", 0.5 * ra, 0.2 * hb),
            ("em_solenoid_offaxis_outside", 1.6 * ra, 1.3 * hb),
        ] {
            let (dr, dz) = sheet_cylinder_field_local(ra, hb, kk, rho, z);
            let (lr, lz) = sheet_cylinder_field_by_loops(ra, hb, kk, rho, z, 64);
            v.push(Check {
                id,
                title: format!(
                    "Соленоид вне оси (ρ = {:.1}R, z = {:.1}·L/2): |B| Derby–Olbert против стопки витков",
                    rho / ra,
                    z / hb
                ),
                simulated: (dr * dr + dz * dz).sqrt(),
                reference: (lr * lr + lz * lz).sqrt(),
                unit: "Тл",
                method: "замкнутая формула Derby–Olbert (cel)".into(),
                reference_method: "интеграл полей витков (K, E) по длине, Гаусс–Лежандр 64×16".into(),
                tolerance: Tolerance::Relative(1e-8),
                note: format!("Компоненты: B_ρ {dr:.6e} / {lr:.6e}, B_z {dz:.6e} / {lz:.6e}."),
            });
        }
    }

    // 5. Закон Ампера: ∮B·dl вокруг стороны квадратной рамки = μ₀I.
    {
        let side = 0.2;
        let cur = 3.0;
        let sq = Polyline::rectangle(Vec3::zeros(), Vec3::x(), Vec3::y(), side, side, cur);
        // сторона вдоль +x при y = −side/2; контур — окружность в плоскости yz вокруг её середины
        let circ = circulation_on_circle(
            &sq,
            Vec3::new(0.0, -0.5 * side, 0.0),
            Vec3::x(),
            0.3 * side,
            400,
        );
        v.push(Check {
            id: "em_ampere_circulation",
            title: "Закон Ампера: ∮B·dl вокруг провода квадратной рамки = μ₀I".into(),
            simulated: circ,
            reference: MU0 * cur,
            unit: "Тл·м",
            method: "Био–Савар ломаной, правило трапеций по окружности (400 узлов)".into(),
            reference_method: "закон Ампера ∮B·dl = μ₀I_охв".into(),
            tolerance: Tolerance::Relative(1e-9),
            note: "Контур радиусом 0.3·сторона охватывает только одну сторону рамки.".into(),
        });
    }

    // 6. div B = 0 численно (виток и ломаная).
    {
        let p = Vec3::new(0.4 * a, -0.3 * a, 0.25 * a);
        let h = 1e-4 * a;
        let b = lp.b_at(&p).norm();
        let dl = divergence(&lp, &p, h) * a / b;
        v.push(Check {
            id: "em_div_b_loop",
            title: "∇·B = 0: виток (центральные разности), безразмерно a·∇·B/|B|".into(),
            simulated: dl,
            reference: 0.0,
            unit: "—",
            method: "центральные разности h = 1e-4·a".into(),
            reference_method: "уравнение Максвелла ∇·B = 0".into(),
            tolerance: Tolerance::Absolute(1e-6),
            note: "Остаток — ошибка разностей O(h²) и округления.".into(),
        });
        let poly = Polyline::regular_polygon(Vec3::zeros(), Vec3::new(0.2, 0.1, 1.0), a, 7, i);
        let bp = poly.b_at(&p).norm();
        v.push(Check {
            id: "em_div_b_polyline",
            title: "∇·B = 0: семиугольная рамка под наклоном".into(),
            simulated: divergence(&poly, &p, h) * a / bp,
            reference: 0.0,
            unit: "—",
            method: "центральные разности h = 1e-4·a".into(),
            reference_method: "∇·B = 0".into(),
            tolerance: Tolerance::Absolute(1e-6),
            note: String::new(),
        });
    }

    // 7. Трассировка силовой линии: поток Ψ = 2πρA_φ постоянен вдоль линии.
    {
        let opt = TraceOptions {
            step: 0.004 * a,
            max_steps: 20000,
            bounds_center: Vec3::zeros(),
            bounds_radius: 20.0 * a,
            min_field: 1e-15,
            both_directions: false,
        };
        let seed = Vec3::new(0.6 * a, 0.0, 0.0);
        let line = trace_field_line(&lp, seed, &opt);
        let psi = |p: &Vec3| {
            let rho = (p.x * p.x + p.y * p.y).sqrt();
            2.0 * PI * rho * loop_vector_potential_local(a, i, rho, p.z)
        };
        let closed = line.last() == Some(&seed);
        let psi0 = psi(&seed);
        let max_dev = line
            .iter()
            .map(|p| (psi(p) - psi0).abs())
            .fold(0.0, f64::max);
        v.push(Check {
            id: "em_fieldline_flux_invariant",
            title: "Силовая линия витка (RK4): поток Ψ = 2πρA_φ вдоль линии постоянен".into(),
            simulated: psi0 + max_dev,
            reference: psi0,
            unit: "Вб",
            method: format!(
                "RK4 по длине дуги, шаг 0.004a, {} точек, линия {}",
                line.len(),
                if closed { "замкнулась" } else { "НЕ замкнулась" }
            ),
            reference_method: "аналитический A_φ витка через K, E (Smythe/Jackson 5.37)".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: "В осесимметричном поле линия B лежит на поверхности Ψ = const — независимый инвариант.".into(),
        });
    }

    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_matches_cos_form() {
        let p1 = Vec3::new(0.1, -0.2, 0.3);
        let p2 = Vec3::new(-0.4, 0.5, 0.05);
        let r = Vec3::new(0.3, 0.2, -0.1);
        let b = segment_field(&p1, &p2, 2.0, &r);
        let u = (p2 - p1).normalize();
        let r1 = r - p1;
        let r2 = r - p2;
        let c = u.cross(&r1);
        let cl =
            c / c.norm_squared() * (u.dot(&r1) / r1.norm() - u.dot(&r2) / r2.norm()) * MU0 * 2.0
                / (4.0 * PI);
        assert!((b - cl).norm() < 1e-12 * cl.norm());
    }

    #[test]
    fn loop_field_matches_numeric_biot_savart_near_wire() {
        // близко к проводу (m → 1): сверка с мелкой ломаной
        let a = 0.05;
        let lp = CircularLoop::new(Vec3::zeros(), Vec3::z(), a, 1.0);
        let p = Vec3::new(a * 1.01, 0.0, 0.002 * a);
        let poly = Polyline::regular_polygon(Vec3::zeros(), Vec3::z(), a, 20000, 1.0);
        let (b1, b2) = (lp.b_at(&p), poly.b_at(&p));
        assert!((b1 - b2).norm() < 2e-4 * b1.norm(), "{b1:?} {b2:?}");
    }
}
