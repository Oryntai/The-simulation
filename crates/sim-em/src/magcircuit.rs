//! Электромагнит с ферромагнитным сердечником: магнитная цепь (закон полного тока по
//! средней линии), нелинейная кривая B–H стали, сила удержания якоря; помощники по обмотке
//! (калибр провода, сопротивление меди с температурой, нагрев).
//!
//! Допущения цепи (стандартный «сосредоточенный» подход, напр. Fitzgerald & Kingsley,
//! «Electric Machinery», гл. 1): поток один и тот же по всему пути (рассеяние вне зазоров
//! не учитывается), индукция в стали однородна по сечению, гистерезис не учитывается
//! (берётся начальная/средняя кривая), зазоры плоские. Выпучивание поля в зазоре —
//! по желанию: площадь полюса увеличивается на длину зазора по каждой стороне
//! (инженерное правило, точность ~10–20 % при g ≪ размера полюса; не сверено).

use sim_verify::{Check, Tolerance};
use std::f64::consts::PI;

use crate::MU0;
use crate::math::GaussLegendre;

// ---------------------------------------------------------------------------------------
// Кривая намагничивания
// ---------------------------------------------------------------------------------------

/// Кривая B(H) стали: кусочно-линейная интерполяция по таблице, нечётное продолжение
/// на H < 0; за последней точкой — B = J_s + μ₀H (поляризация насыщена).
#[derive(Clone, Debug)]
pub struct BhCurve {
    pub name: &'static str,
    pub source: &'static str,
    h: Vec<f64>,
    b: Vec<f64>,
}

impl BhCurve {
    /// Таблица (H, А/м; B, Тл), начинается с (0, 0), обе колонки строго возрастают.
    pub fn from_table(name: &'static str, source: &'static str, h: &[f64], b: &[f64]) -> Self {
        assert_eq!(h.len(), b.len());
        assert!(h.len() >= 2 && h[0] == 0.0 && b[0] == 0.0);
        for k in 1..h.len() {
            assert!(
                h[k] > h[k - 1] && b[k] > b[k - 1],
                "кривая B–H должна возрастать"
            );
        }
        Self {
            name,
            source,
            h: h.to_vec(),
            b: b.to_vec(),
        }
    }

    /// Малоуглеродистая сталь AISI 1010: таблица из A. Vorozhtsov, «Permanent Magnet Dipole
    /// for DIRAC», CERN DIRAC note 2012-02, разд. 5.3, табл. 3; воспроизведена в глоссарии
    /// QuickField: https://quickfield.com/glossary/soft_steel.htm . Сама марка стали не
    /// задаёт кривую однозначно (зависит от плавки и отжига) — это одна опубликованная кривая.
    pub fn aisi_1010() -> Self {
        const H_KA: [f64; 34] = [
            0.0, 0.166, 0.199, 0.240, 0.289, 0.347, 0.418, 0.502, 0.604, 0.726, 0.872, 1.049,
            1.261, 1.823, 2.192, 3.167, 4.579, 6.619, 9.568, 11.499, 13.831, 16.624, 19.990,
            24.032, 28.893, 34.736, 41.762, 50.209, 60.375, 72.575, 87.257, 104.907, 126.122,
            151.595,
        ];
        const B_T: [f64; 34] = [
            0.0, 0.576, 0.680, 0.792, 0.895, 0.992, 1.082, 1.164, 1.237, 1.302, 1.359, 1.407,
            1.449, 1.517, 1.545, 1.595, 1.646, 1.702, 1.768, 1.805, 1.843, 1.883, 1.924, 1.964,
            2.002, 2.038, 2.071, 2.100, 2.125, 2.146, 2.165, 2.187, 2.214, 2.246,
        ];
        let h: Vec<f64> = H_KA.iter().map(|x| x * 1e3).collect();
        Self::from_table(
            "AISI 1010",
            "Vorozhtsov, CERN DIRAC note 2012-02, табл. 3 (через QuickField glossary «Soft steel»)",
            &h,
            &B_T,
        )
    }

    /// Поляризация насыщения J_s = B − μ₀H в последней точке таблицы.
    pub fn saturation_polarization(&self) -> f64 {
        let n = self.h.len() - 1;
        self.b[n] - MU0 * self.h[n]
    }

    /// Начальная относительная проницаемость (наклон первого отрезка).
    pub fn mu_r_initial(&self) -> f64 {
        self.b[1] / (MU0 * self.h[1])
    }

    /// Последняя табличная напряжённость, А/м.
    pub fn h_table_end(&self) -> f64 {
        *self.h.last().expect("таблица")
    }

    pub fn b_of_h(&self, h: f64) -> f64 {
        let s = h.signum();
        let h = h.abs();
        let n = self.h.len() - 1;
        if h >= self.h[n] {
            return s * (self.saturation_polarization() + MU0 * h);
        }
        let k = self.h.partition_point(|&x| x <= h).max(1) - 1;
        let t = (h - self.h[k]) / (self.h[k + 1] - self.h[k]);
        s * (self.b[k] + t * (self.b[k + 1] - self.b[k]))
    }

    /// H(B) и dH/dB — обратная кусочно-линейная функция.
    pub fn h_of_b(&self, b: f64) -> (f64, f64) {
        let s = b.signum();
        let b = b.abs();
        let n = self.b.len() - 1;
        if b >= self.b[n] {
            return (s * (b - self.saturation_polarization()) / MU0, 1.0 / MU0);
        }
        let k = self.b.partition_point(|&x| x <= b).max(1) - 1;
        let slope = (self.h[k + 1] - self.h[k]) / (self.b[k + 1] - self.b[k]);
        (s * (self.h[k] + slope * (b - self.b[k])), slope)
    }

    /// Плотность энергии ∫₀^B H dB (Дж/м³) — точно для кусочно-линейной кривой.
    pub fn energy_density(&self, b: f64) -> f64 {
        let b = b.abs();
        let mut w = 0.0;
        for k in 0..self.b.len() - 1 {
            if b <= self.b[k] {
                return w;
            }
            let b_hi = b.min(self.b[k + 1]);
            let (h_hi, _) = self.h_of_b(b_hi);
            w += 0.5 * (self.h[k] + h_hi) * (b_hi - self.b[k]);
            if b <= self.b[k + 1] {
                return w;
            }
        }
        let js = self.saturation_polarization();
        let bl = *self.b.last().expect("таблица");
        w + ((b - js).powi(2) - (bl - js).powi(2)) / (2.0 * MU0)
    }
}

/// Материал сердечника.
#[derive(Clone, Debug)]
pub enum CoreMaterial {
    /// μ → ∞ (сталь не требует МДС) — предел для аналитики.
    Ideal,
    /// Линейный материал с постоянной μ_r.
    Linear { mu_r: f64 },
    /// Нелинейная кривая B–H.
    Nonlinear(BhCurve),
}

impl CoreMaterial {
    fn h_of_b(&self, b: f64) -> (f64, f64) {
        match self {
            CoreMaterial::Ideal => (0.0, 0.0),
            CoreMaterial::Linear { mu_r } => (b / (MU0 * mu_r), 1.0 / (MU0 * mu_r)),
            CoreMaterial::Nonlinear(c) => c.h_of_b(b),
        }
    }
    fn energy_density(&self, b: f64) -> f64 {
        match self {
            CoreMaterial::Ideal => 0.0,
            CoreMaterial::Linear { mu_r } => b * b / (2.0 * MU0 * mu_r),
            CoreMaterial::Nonlinear(c) => c.energy_density(b),
        }
    }
}

/// Форма полюса (сечение зазора без учёта выпучивания).
#[derive(Clone, Copy, Debug)]
pub enum PoleFace {
    Rect { a: f64, b: f64 },
    Round { d: f64 },
}

/// Электромагнит: сердечник (+ якорь) со средней длиной пути `core_path_m` и сечением
/// `core_area_m2`, `n_gaps` одинаковых зазоров `gap_m` (U-образный магнит с якорем — 2),
/// обмотка `turns` витков.
#[derive(Clone, Debug)]
pub struct MagneticCircuit {
    pub material: CoreMaterial,
    pub core_path_m: f64,
    pub core_area_m2: f64,
    pub pole: PoleFace,
    pub gap_m: f64,
    pub n_gaps: u32,
    pub turns: f64,
    pub fringing: bool,
}

/// Решение цепи при заданном токе.
#[derive(Clone, Copy, Debug)]
pub struct CircuitState {
    pub flux_wb: f64,
    pub b_core_t: f64,
    pub h_core_a_per_m: f64,
    pub b_gap_t: f64,
    pub gap_area_m2: f64,
    /// МДС, приходящаяся на сталь и на зазоры, А.
    pub mmf_core_a: f64,
    pub mmf_gaps_a: f64,
    pub iterations: u32,
}

impl MagneticCircuit {
    /// Площадь зазора: площадь полюса, при `fringing` — расширенная на g по каждой стороне.
    pub fn gap_area(&self, g: f64) -> f64 {
        let e = if self.fringing { g } else { 0.0 };
        match self.pole {
            PoleFace::Rect { a, b } => (a + e) * (b + e),
            PoleFace::Round { d } => 0.25 * PI * (d + e) * (d + e),
        }
    }

    fn mmf_of_flux(&self, phi: f64, g: f64) -> (f64, f64, f64) {
        let bc = phi / self.core_area_m2;
        let (hc, dh) = self.material.h_of_b(bc);
        let ag = self.gap_area(g);
        let gap_coef = self.n_gaps as f64 * g / (MU0 * ag);
        let mmf = hc * self.core_path_m + gap_coef * phi;
        let dmmf = dh * self.core_path_m / self.core_area_m2 + gap_coef;
        (mmf, dmmf, hc)
    }

    /// Решение при токе `current` и текущем зазоре.
    pub fn solve(&self, current: f64) -> CircuitState {
        self.solve_at_gap(current, self.gap_m)
    }

    /// Решение N·I = H(Φ/A_c)·l_c + n·g·Φ/(μ₀A_g) относительно Φ: метод Ньютона,
    /// защищённый бисекцией (вилка [0, Φ_max], Φ_max — поток при идеальной стали).
    pub fn solve_at_gap(&self, current: f64, g: f64) -> CircuitState {
        let ni = self.turns * current;
        let s = ni.signum();
        let ni = ni.abs();
        let ag = self.gap_area(g);
        let n = self.n_gaps as f64;
        let phi_max = if g > 0.0 && n > 0.0 {
            MU0 * ag * ni / (n * g)
        } else {
            // без зазора: верхняя граница из насыщения (B ≤ J_s + μ₀·NI/l_c) с запасом
            (3.0 + MU0 * ni / self.core_path_m) * self.core_area_m2
        };
        let (mut lo, mut hi) = (0.0, phi_max);
        // Старт справа: МДС(Φ) выпукла (кривая B–H загибается), и Ньютон с правого края
        // идёт к корню монотонно; где выпуклость нарушена (округление таблицы) — бисекция.
        let mut phi = phi_max;
        let mut it = 0;
        for _ in 0..200 {
            it += 1;
            let (f, df, _) = self.mmf_of_flux(phi, g);
            let r = f - ni;
            if r > 0.0 {
                hi = phi;
            } else {
                lo = phi;
            }
            let mut next = phi - r / df;
            if !(next > lo && next < hi) || !next.is_finite() {
                next = 0.5 * (lo + hi);
            }
            let done = (next - phi).abs() <= 1e-15 * phi_max || (hi - lo) <= 1e-15 * phi_max;
            phi = next;
            if done {
                break;
            }
        }
        let (_, _, hc) = self.mmf_of_flux(phi, g);
        let mmf_core = hc * self.core_path_m;
        CircuitState {
            flux_wb: s * phi,
            b_core_t: s * phi / self.core_area_m2,
            h_core_a_per_m: s * hc,
            b_gap_t: s * phi / ag,
            gap_area_m2: ag,
            mmf_core_a: s * mmf_core,
            mmf_gaps_a: s * (ni - mmf_core),
            iterations: it,
        }
    }

    /// Сила притяжения якоря по натяжению Максвелла: F = n·B_g²·A_g/(2μ₀), Н.
    pub fn force_maxwell_n(&self, current: f64) -> f64 {
        let st = self.solve(current);
        self.n_gaps as f64 * st.b_gap_t * st.b_gap_t * st.gap_area_m2 / (2.0 * MU0)
    }

    /// Коэнергия W'(I, g) = ∫₀ᴵ N·Φ di = Φ·N·I − W, где энергия поля
    /// W = V_стали·∫₀^{B_c} H dB + n·g·Φ²/(2μ₀A_g) (точно для кусочно-линейной B–H).
    pub fn coenergy_j(&self, current: f64, g: f64) -> f64 {
        let st = self.solve_at_gap(current, g);
        let w_core =
            self.core_path_m * self.core_area_m2 * self.material.energy_density(st.b_core_t);
        let w_gap =
            self.n_gaps as f64 * g * st.flux_wb * st.flux_wb / (2.0 * MU0 * self.gap_area(g));
        st.flux_wb * self.turns * current - (w_core + w_gap)
    }

    /// Коэнергия прямым интегрированием потокосцепления по току (Гаусс–Лежандр) —
    /// для сверки полуаналитической формулы.
    pub fn coenergy_by_current_integral(&self, current: f64, g: f64, panels: usize) -> f64 {
        let gl = GaussLegendre::new(8);
        gl.integrate_composite(0.0, current, panels, |i| {
            self.turns * self.solve_at_gap(i, g).flux_wb
        })
    }

    /// Сила по принципу виртуальной работы: F = −∂W'/∂g|_I (центральная разность),
    /// положительная — притяжение (зазор стремится закрыться).
    pub fn force_virtual_work_n(&self, current: f64) -> f64 {
        let g = self.gap_m;
        let dg = 1e-4 * g;
        -(self.coenergy_j(current, g + dg) - self.coenergy_j(current, g - dg)) / (2.0 * dg)
    }

    /// Секущая индуктивность L = N·Φ/I, Гн.
    pub fn inductance_h(&self, current: f64) -> f64 {
        self.turns * self.solve(current).flux_wb / current
    }
}

// ---------------------------------------------------------------------------------------
// Обмотка
// ---------------------------------------------------------------------------------------

/// Удельное сопротивление отожжённой меди при 20 °C (стандарт IACS 100 %, IEC 60028:
/// 1/58 Ом·мм²/м), Ом·м.
pub const RHO_CU_20C: f64 = 1.724_14e-8;
/// Температурный коэффициент сопротивления меди при 20 °C, 1/К (IACS: 0.00393).
pub const ALPHA_CU: f64 = 0.003_93;

/// Диаметр голой проволоки по калибру AWG: d = 0.127 мм · 92^((36 − n)/39)
/// (определение ASTM B258). Для 0/0 … 0000 передавайте −1 … −3.
pub fn awg_diameter_m(awg: i32) -> f64 {
    0.127e-3 * 92f64.powf((36 - awg) as f64 / 39.0)
}

/// Сечение проволоки по калибру, м².
pub fn awg_area_m2(awg: i32) -> f64 {
    let d = awg_diameter_m(awg);
    0.25 * PI * d * d
}

/// Сопротивление медного проводника длиной `length_m`, сечением `area_m2` при `t_c`:
/// R = ρ₂₀·l/A·(1 + α(T − 20)).
pub fn copper_resistance_ohm(length_m: f64, area_m2: f64, t_c: f64) -> f64 {
    RHO_CU_20C * length_m / area_m2 * (1.0 + ALPHA_CU * (t_c - 20.0))
}

/// Сколько витков провода диаметром (с изоляцией) `d_insulated_m` помещается в окно
/// площадью `window_m2` при коэффициенте заполнения `fill` (рядовая намотка ~0.5–0.6,
/// «оценка»).
pub fn turns_in_window(window_m2: f64, d_insulated_m: f64, fill: f64) -> f64 {
    (fill * window_m2 / (0.25 * PI * d_insulated_m * d_insulated_m)).floor()
}

/// Катушка: число витков, калибр, средняя длина витка, площадь охлаждаемой поверхности.
#[derive(Clone, Copy, Debug)]
pub struct Coil {
    pub turns: f64,
    pub awg: i32,
    pub mean_turn_length_m: f64,
    /// Наружная поверхность, отдающая тепло, м².
    pub cooling_area_m2: f64,
    /// Коэффициент теплоотдачи, Вт/(м²·К): естественная конвекция + излучение ≈ 10–15
    /// («оценка», учебник Incropera, гл. 9 — порядок величины для вертикальной стенки).
    pub h_w_per_m2k: f64,
}

impl Coil {
    pub fn wire_length_m(&self) -> f64 {
        self.turns * self.mean_turn_length_m
    }
    pub fn resistance_ohm(&self, t_c: f64) -> f64 {
        copper_resistance_ohm(self.wire_length_m(), awg_area_m2(self.awg), t_c)
    }
    /// Плотность тока, А/мм² (медные обмотки без принудительного охлаждения — обычно ≤ 3–5).
    pub fn current_density_a_per_mm2(&self, current: f64) -> f64 {
        current / (awg_area_m2(self.awg) * 1e6)
    }
    /// Установившаяся температура при постоянном токе с учётом роста R(T):
    /// T = T_a + I²R(T)/(hA) → T = (T_a + k(1 − 20α))/(1 − kα), k = I²R₂₀/(hA).
    /// None — тепловой разгон (kα ≥ 1: нагрев растёт быстрее отвода).
    pub fn steady_temperature_c(&self, current: f64, t_ambient_c: f64) -> Option<f64> {
        let k = current * current * self.resistance_ohm(20.0)
            / (self.h_w_per_m2k * self.cooling_area_m2);
        let den = 1.0 - k * ALPHA_CU;
        (den > 0.0).then(|| (t_ambient_c + k * (1.0 - 20.0 * ALPHA_CU)) / den)
    }
    /// Мощность потерь при токе и температуре, Вт.
    pub fn power_w(&self, current: f64, t_c: f64) -> f64 {
        current * current * self.resistance_ohm(t_c)
    }
}

// ---------------------------------------------------------------------------------------
// Сверки
// ---------------------------------------------------------------------------------------

/// Пример: U-образный электромагнит удержания с якорем (два зазора).
pub fn example_holding_magnet(material: CoreMaterial, gap_m: f64) -> MagneticCircuit {
    MagneticCircuit {
        material,
        core_path_m: 0.16,
        core_area_m2: 4e-4,
        pole: PoleFace::Rect { a: 0.02, b: 0.02 },
        gap_m,
        n_gaps: 2,
        turns: 400.0,
        fringing: false,
    }
}

/// Все сверки модуля магнитной цепи и обмоток.
pub fn checks() -> Vec<Check> {
    let mut v = Vec::new();
    let steel = BhCurve::aisi_1010();
    let g = 0.5e-3;

    // 1. Идеальный сердечник: B = μ₀NI/(2g).
    {
        let mc = example_holding_magnet(CoreMaterial::Ideal, g);
        let i = 1.0;
        let st = mc.solve(i);
        v.push(Check {
            id: "em_circuit_ideal_core",
            title: "Электромагнит, сталь μ → ∞: B в зазоре против μ₀·N·I/(2g)".into(),
            simulated: st.b_gap_t,
            reference: MU0 * mc.turns * i / (2.0 * g),
            unit: "Тл",
            method: "решатель цепи (Ньютон + бисекция)".into(),
            reference_method: "закон полного тока, вся МДС на двух зазорах".into(),
            tolerance: Tolerance::Relative(1e-12),
            note: "N = 400, g = 0.5 мм ×2, I = 1 А.".into(),
        });
    }
    // 2. AISI 1010, начальный линейный участок таблицы (B < 0.576 Т).
    {
        let mc = example_holding_magnet(CoreMaterial::Nonlinear(steel.clone()), g);
        let i = 0.5;
        let st = mc.solve(i);
        let mu_r = steel.mu_r_initial();
        let reference = MU0 * mc.turns * i / (2.0 * g + mc.core_path_m / mu_r);
        let naive = MU0 * mc.turns * i / (2.0 * g);
        v.push(Check {
            id: "em_circuit_linear_region",
            title: "AISI 1010, линейный участок: B в зазоре против μ₀NI/(2g + l_c/μ_r0)".into(),
            simulated: st.b_gap_t,
            reference,
            unit: "Тл",
            method: format!("нелинейный решатель по таблице B–H ({} итер.)", st.iterations),
            reference_method: format!("аналитика с μ_r0 = {mu_r:.0} (наклон первого отрезка таблицы)"),
            tolerance: Tolerance::Relative(1e-10),
            note: format!(
                "Без учёта стали μ₀NI/(2g) = {naive:.5} Тл — на {:.1} % выше: МДС стали в линейном режиме.",
                (naive / reference - 1.0) * 100.0
            ),
        });
    }
    // 3. Глубокое насыщение: аналитика B = J_s + μ₀H.
    {
        let mc = example_holding_magnet(CoreMaterial::Nonlinear(steel.clone()), g);
        let i = 120.0; // NI = 48 кА — далеко за таблицей
        let st = mc.solve(i);
        let js = steel.saturation_polarization();
        let ni = mc.turns * i;
        let h = (ni - 2.0 * g * js / MU0) / (mc.core_path_m + 2.0 * g);
        let reference = js + MU0 * h;
        v.push(Check {
            id: "em_circuit_saturation",
            title: "AISI 1010, насыщение (NI = 48 кА): B против аналитики J_s + μ₀H".into(),
            simulated: st.b_core_t,
            reference,
            unit: "Тл",
            method: "нелинейный решатель".into(),
            reference_method: format!(
                "за таблицей B = J_s + μ₀H, J_s = {js:.4} Тл; H = (NI − 2gJ_s/μ₀)/(l_c + 2g)"
            ),
            tolerance: Tolerance::Relative(1e-10),
            note: format!(
                "H стали = {:.0} кА/м > конца таблицы {:.0} кА/м. Плато: сила растёт только за счёт μ₀H.",
                h / 1e3,
                steel.h_table_end() / 1e3
            ),
        });
        // плато: сила при 48 кА против силы при 12 кА — рост мал по сравнению с ростом тока
        let f_hi = mc.force_maxwell_n(i);
        let f_lo = mc.force_maxwell_n(i / 4.0);
        v.push(Check {
            id: "em_circuit_saturation_plateau",
            title: "Плато насыщения: рост силы при увеличении тока в 4 раза (12 → 48 кА)".into(),
            simulated: f_hi / f_lo,
            reference: 1.0,
            unit: "—",
            method: format!("F(48 кА) = {f_hi:.0} Н, F(12 кА) = {f_lo:.0} Н"),
            reference_method: "в насыщении сила ≈ const (в линейном режиме выросла бы в 16 раз)"
                .into(),
            tolerance: Tolerance::Absolute(0.35),
            note: "Допуск отражает остаточный рост B за счёт μ₀H (≈ 0.1–0.2 Тл), а не подгонку."
                .into(),
        });
    }
    // 4. Сила: натяжение Максвелла против виртуальной работы — три режима.
    for (id, label, i) in [
        ("em_force_maxwell_vs_energy_linear", "линейный режим", 0.5),
        ("em_force_maxwell_vs_energy_knee", "колено кривой", 3.0),
        ("em_force_maxwell_vs_energy_sat", "насыщение", 60.0),
    ] {
        let mc = example_holding_magnet(CoreMaterial::Nonlinear(steel.clone()), g);
        let st = mc.solve(i);
        v.push(Check {
            id,
            title: format!(
                "Сила удержания, {label} (B_стали = {:.2} Тл): −∂W'/∂g против n·B²A/(2μ₀)",
                st.b_core_t
            ),
            simulated: mc.force_virtual_work_n(i),
            reference: mc.force_maxwell_n(i),
            unit: "Н",
            method: "виртуальная работа: численная производная коэнергии по зазору при I = const"
                .into(),
            reference_method: "натяжение Максвелла на поверхностях полюсов".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: "Равны тождественно для цепи без выпучивания; при выпучивании (A_g(g)) \
                   формула Максвелла теряет член ∂A_g/∂g — расхождение растёт с g."
                .into(),
        });
    }
    // 5. Коэнергия двумя путями.
    {
        let mc = example_holding_magnet(CoreMaterial::Nonlinear(steel.clone()), g);
        let i = 3.0;
        v.push(Check {
            id: "em_coenergy_two_ways",
            title: "Коэнергия на колене кривой: ∫NΦ di (квадратура по току) против ΦNI − W (энергия поля)".into(),
            simulated: mc.coenergy_by_current_integral(i, g, 400),
            reference: mc.coenergy_j(i, g),
            unit: "Дж",
            method: "Гаусс–Лежандр 400×8 по току, в каждом узле — решение цепи".into(),
            reference_method: "Φ·N·I − (V·∫H dB + nΦ²g/(2μ₀A)), точно для кусочно-линейной B–H".into(),
            tolerance: Tolerance::Relative(1e-5),
            note: "Остаток — изломы Φ(i) на узлах таблицы (квадратура O(h²) на изломе).".into(),
        });
    }
    // 6. Калибры AWG против таблицы ASTM B258.
    for (id, awg, d_in) in [
        ("em_awg10_diameter", 10, 0.1019),
        ("em_awg20_diameter", 20, 0.0320),
        ("em_awg30_diameter", 30, 0.0100),
    ] {
        v.push(Check {
            id,
            title: format!("Диаметр AWG {awg}: формула против таблицы ASTM B258"),
            simulated: awg_diameter_m(awg) * 1e3,
            reference: d_in * 25.4,
            unit: "мм",
            method: "d = 0.127 мм·92^((36−n)/39)".into(),
            reference_method: format!("ASTM B258, табл. 1: {d_in} дюйма"),
            tolerance: Tolerance::Relative(0.005),
            note: "Таблица округлена до 0.1 мил (0.0001″) — отсюда остаток.".into(),
        });
    }
    // 7. Сопротивление при 75 °C против NEC Chapter 9 Table 8.
    for (id, awg, ohm_kft) in [
        ("em_awg10_resistance_75c", 10, 1.21),
        ("em_awg14_resistance_75c", 14, 3.07),
    ] {
        let r = copper_resistance_ohm(1000.0 * 0.3048, awg_area_m2(awg), 75.0);
        v.push(Check {
            id,
            title: format!(
                "Сопротивление AWG {awg} (одножильный) при 75 °C: ρ₂₀(1 + αΔT)·l/A против NEC"
            ),
            simulated: r,
            reference: ohm_kft,
            unit: "Ом/1000 фут",
            method: "ρ₂₀ = 1.72414e-8 Ом·м (IACS), α = 0.00393 1/К".into(),
            reference_method: "NFPA 70 (NEC), Chapter 9, Table 8, uncoated solid copper, 75 °C"
                .into(),
            tolerance: Tolerance::Relative(0.01),
            note: "Проверяет и сечение по AWG, и температурный коэффициент меди.".into(),
        });
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bh_inverse_roundtrip() {
        let c = BhCurve::aisi_1010();
        for &h in &[50.0, 300.0, 2000.0, 40_000.0, 400_000.0] {
            let b = c.b_of_h(h);
            let (h2, _) = c.h_of_b(b);
            assert!((h2 - h).abs() < 1e-9 * h, "{h} {h2}");
        }
        assert!(c.mu_r_initial() > 2000.0);
    }

    #[test]
    fn energy_density_matches_quadrature() {
        let c = BhCurve::aisi_1010();
        let b = 2.3;
        let gl = GaussLegendre::new(8);
        let q = gl.integrate_composite(0.0, b, 20000, |x| c.h_of_b(x).0);
        assert!(
            (q - c.energy_density(b)).abs() < 1e-6 * q,
            "{q} {}",
            c.energy_density(b)
        );
    }

    #[test]
    fn coil_temperature_self_consistent() {
        let coil = Coil {
            turns: 400.0,
            awg: 22,
            mean_turn_length_m: 0.1,
            cooling_area_m2: 0.01,
            h_w_per_m2k: 12.0,
        };
        let t = coil.steady_temperature_c(0.8, 25.0).unwrap();
        let p = coil.power_w(0.8, t);
        assert!((t - 25.0 - p / (12.0 * 0.01)).abs() < 1e-9);
    }
}
