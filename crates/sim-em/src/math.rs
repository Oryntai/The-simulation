//! Численные «кирпичи»: полные эллиптические интегралы (AGM), обобщённый эллиптический
//! интеграл Булирша `cel`, функция Бесселя J₁, квадратуры Гаусса–Лежандра и tanh-sinh.
//!
//! Всё реализовано по учебникам/статьям (ссылки у каждой функции), чужой код не переносился.

use std::f64::consts::PI;

/// Полные эллиптические интегралы K(m) и E(m) через дополнительный параметр `mc = 1 − m`
/// (m = k²). Метод арифметико-геометрического среднего (Abramowitz & Stegun, 17.6;
/// DLMF 19.8.1–19.8.6): a₀ = 1, b₀ = √mc, c₀² = m; K = π/(2·a_N), E = K·(1 − Σ 2ⁿ⁻¹·cₙ²).
///
/// Параметр берётся как `mc`, чтобы вблизи особенности (m → 1, точка у провода)
/// не терять точность на вычитании 1 − m: геометрия даёт mc напрямую.
/// При `mc ≤ 0` возвращает (∞, 1).
pub fn ellip_ke_mc(mc: f64) -> (f64, f64) {
    if mc <= 0.0 {
        return (f64::INFINITY, 1.0);
    }
    let mut a = 1.0_f64;
    let mut b = mc.sqrt();
    let mut c2 = 1.0 - mc; // c₀²
    let mut pow2 = 0.5_f64; // 2^(n−1) при n = 0
    let mut sum = pow2 * c2;
    for _ in 0..40 {
        let an = 0.5 * (a + b);
        let bn = (a * b).sqrt();
        let cn = 0.5 * (a - b);
        a = an;
        b = bn;
        pow2 *= 2.0;
        c2 = cn * cn;
        sum += pow2 * c2;
        if cn.abs() <= 1e-17 * a {
            break;
        }
    }
    let k = PI / (2.0 * a);
    (k, k * (1.0 - sum))
}

/// K(m) и E(m) по параметру m = k² (обёртка над [`ellip_ke_mc`]).
pub fn ellip_ke(m: f64) -> (f64, f64) {
    ellip_ke_mc(1.0 - m)
}

/// Обобщённый полный эллиптический интеграл Булирша
/// cel(kc, p, c, s) = ∫₀^{π/2} (c·cos²φ + s·sin²φ) / ((cos²φ + p·sin²φ)·√(cos²φ + kc²·sin²φ)) dφ.
///
/// Алгоритм: R. Bulirsch, «Numerical calculation of elliptic integrals and elliptic
/// functions III», Numer. Math. 13 (1969) 305–315, doi:10.1007/BF02165405; в виде,
/// приведённом в приложении N. Derby, S. Olbert, Am. J. Phys. 78 (2010) 229,
/// doi:10.1119/1.3256157. Частные случаи: cel(kc,1,1,1) = K, cel(kc,1,1,kc²) = E.
/// При kc = 0 интеграл расходится — возвращает NaN.
pub fn cel(kc: f64, p: f64, c: f64, s: f64) -> f64 {
    if kc == 0.0 {
        return f64::NAN;
    }
    // Сходимость квадратичная: при условии останова 1e-8 ошибка ~1e-16.
    let errtol = 1e-8;
    let mut k = kc.abs();
    let mut pp = p;
    let mut cc = c;
    let mut ss = s;
    let mut em = 1.0_f64;
    if p > 0.0 {
        pp = p.sqrt();
        ss = s / pp;
    } else {
        let mut f = kc * kc;
        let mut q = 1.0 - f;
        let g = 1.0 - pp;
        f -= pp;
        q *= ss - c * pp;
        pp = (f / g).sqrt();
        cc = (c - ss) / g;
        ss = -q / (g * g * pp) + cc * pp;
    }
    let mut f = cc;
    cc += ss / pp;
    let mut g = k / pp;
    ss = 2.0 * (ss + f * g);
    pp += g;
    g = em;
    em += k;
    let mut kk = k;
    let mut iter = 0;
    while (g - k).abs() > g * errtol && iter < 60 {
        k = 2.0 * kk.sqrt();
        kk = k * em;
        f = cc;
        cc += ss / pp;
        g = kk / pp;
        ss = 2.0 * (ss + f * g);
        pp += g;
        g = em;
        em += k;
        iter += 1;
    }
    0.5 * PI * (ss + cc * em) / (em * (em + pp))
}

/// Функция Бесселя первого рода целого порядка Jₙ(x) по интегралу Бесселя
/// Jₙ(x) = (1/π)·∫₀^π cos(nτ − x·sin τ) dτ (DLMF 10.9.2).
/// Подынтегральная функция чётная и 2π-периодическая, поэтому правило трапеций сходится
/// экспоненциально, как только число узлов заметно больше |x| (Trefethen & Weideman,
/// SIAM Review 56 (2014) 385, doi:10.1137/130932132). Берём N = |x| + n + 40.
pub fn bessel_jn(n: u32, x: f64) -> f64 {
    let nodes = (x.abs() + n as f64 + 40.0).ceil() as usize;
    let h = PI / nodes as f64;
    let nf = n as f64;
    // концы τ = 0 и τ = π (sin π = 0 точно по смыслу)
    let mut sum = 0.5 * (1.0 + (nf * PI).cos());
    for j in 1..nodes {
        let t = j as f64 * h;
        sum += (nf * t - x * t.sin()).cos();
    }
    sum * h / PI
}

/// Узлы и веса квадратуры Гаусса–Лежандра на [−1, 1].
/// Корни Pₙ ищутся методом Ньютона от приближения cos(π(i − ¼)/(n + ½))
/// (Golub & Welsch / стандартный учебный алгоритм; Abramowitz & Stegun 25.4.29).
#[derive(Clone, Debug)]
pub struct GaussLegendre {
    pub x: Vec<f64>,
    pub w: Vec<f64>,
}

impl GaussLegendre {
    pub fn new(n: usize) -> Self {
        assert!(n >= 1);
        let mut x = vec![0.0; n];
        let mut w = vec![0.0; n];
        let nf = n as f64;
        for i in 0..n.div_ceil(2) {
            let mut z = (PI * (i as f64 + 0.75) / (nf + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                // Рекуррентность для P_n(z) и производной.
                let (mut p0, mut p1) = (1.0_f64, z);
                for k in 2..=n {
                    let kf = k as f64;
                    let p2 = ((2.0 * kf - 1.0) * z * p1 - (kf - 1.0) * p0) / kf;
                    p0 = p1;
                    p1 = p2;
                }
                // p1 = P_n(z), p0 = P_{n−1}(z)
                dp = nf * (z * p1 - p0) / (z * z - 1.0);
                let dz = p1 / dp;
                z -= dz;
                if dz.abs() < 1e-16 {
                    break;
                }
            }
            x[i] = -z;
            x[n - 1 - i] = z;
            let wi = 2.0 / ((1.0 - z * z) * dp * dp);
            w[i] = wi;
            w[n - 1 - i] = wi;
        }
        if n % 2 == 1 {
            x[n / 2] = 0.0;
        }
        Self { x, w }
    }

    /// ∫ₐᵇ f — одна панель.
    pub fn integrate(&self, a: f64, b: f64, mut f: impl FnMut(f64) -> f64) -> f64 {
        let hm = 0.5 * (b - a);
        let c = 0.5 * (a + b);
        let mut s = 0.0;
        for (xi, wi) in self.x.iter().zip(&self.w) {
            s += wi * f(c + hm * xi);
        }
        s * hm
    }

    /// ∫ₐᵇ f — составное правило на `panels` равных панелях.
    pub fn integrate_composite(
        &self,
        a: f64,
        b: f64,
        panels: usize,
        mut f: impl FnMut(f64) -> f64,
    ) -> f64 {
        let h = (b - a) / panels as f64;
        let mut s = 0.0;
        for p in 0..panels {
            let lo = a + p as f64 * h;
            s += self.integrate(lo, lo + h, &mut f);
        }
        s
    }
}

/// Квадратура tanh-sinh (двойная экспонента, Takahasi & Mori, Publ. RIMS 9 (1974) 721,
/// doi:10.2977/prims/1195192451) на [a, b]. Устойчива к интегрируемым особенностям
/// на концах (логарифм, 1/√x). `level` — число удвоений сетки (6–8 обычно хватает).
pub fn tanh_sinh(a: f64, b: f64, level: u32, mut f: impl FnMut(f64) -> f64) -> f64 {
    let hw = 0.5 * (b - a);
    let h = 2.0_f64.powi(-(level as i32));
    // t = 4: расстояние до конца ~e^{-86} — хвост даже для 1/√x меньше 1e-18
    let tmax = 4.0;
    let n = (tmax / h).ceil() as i64;
    let mut sum = 0.0;
    for j in -n..=n {
        let t = j as f64 * h;
        let u = 0.5 * PI * t.sinh();
        let ch = u.cosh();
        // x = tanh(u); 1 − |x| считаем без вычитания: 1/(e^{|u|}·ch)
        let x = u.tanh();
        let one_minus = 1.0 / ((u.abs()).exp() * ch);
        if one_minus < 1e-300 {
            continue;
        }
        let w = 0.5 * PI * t.cosh() / (ch * ch);
        // точка рядом с концом: считаем расстояние до конца точно
        let xp = if x >= 0.0 {
            b - hw * one_minus
        } else {
            a + hw * one_minus
        };
        let fx = f(xp);
        if fx.is_finite() {
            sum += w * fx;
        }
    }
    sum * h * hw
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ke_known_values() {
        // K(0) = E(0) = π/2; K(0.5) = 1.8540746773013719, E(0.5) = 1.3506438810476755 (DLMF/A&S табл. 17.1)
        let (k0, e0) = ellip_ke(0.0);
        assert!((k0 - PI / 2.0).abs() < 1e-15 && (e0 - PI / 2.0).abs() < 1e-15);
        let (k, e) = ellip_ke(0.5);
        assert!((k - 1.854_074_677_301_372).abs() < 1e-14, "{k}");
        assert!((e - 1.350_643_881_047_675_5).abs() < 1e-14, "{e}");
        // Тождество Лежандра: E·K' + E'·K − K·K' = π/2
        let m = 0.3;
        let (k1, e1) = ellip_ke(m);
        let (k2, e2) = ellip_ke(1.0 - m);
        assert!((e1 * k2 + e2 * k1 - k1 * k2 - PI / 2.0).abs() < 1e-14);
    }

    #[test]
    fn cel_reduces_to_k_and_e() {
        for &kc in &[0.05, 0.3, 0.7, 0.999, 1.0] {
            let mc = kc * kc;
            let (k, e) = ellip_ke_mc(mc);
            assert!((cel(kc, 1.0, 1.0, 1.0) - k).abs() < 1e-13 * k, "K kc={kc}");
            assert!((cel(kc, 1.0, 1.0, mc) - e).abs() < 1e-13 * e, "E kc={kc}");
        }
        // Третий род, p ≠ 1, сверка с прямой квадратурой.
        for &(kc, p, c, s) in &[
            (0.4, 0.3, 1.0, 0.2),
            (0.8, 2.5, 0.7, -1.0),
            (0.6, 0.0, 1.0, 0.0),
        ] {
            let gl = GaussLegendre::new(40);
            let direct = gl.integrate_composite(0.0, PI / 2.0, 8, |phi: f64| {
                let (sn, cs) = phi.sin_cos();
                (c * cs * cs + s * sn * sn)
                    / ((cs * cs + p * sn * sn) * (cs * cs + kc * kc * sn * sn).sqrt())
            });
            let v = cel(kc, p, c, s);
            assert!(
                (v - direct).abs() < 1e-11 * direct.abs().max(1.0),
                "{v} vs {direct}"
            );
        }
    }

    #[test]
    fn bessel_j1_known_values() {
        // DLMF табл. 10.?: J1(1) = 0.4400505857449335, J1(10) = 0.04347274616886144
        assert!((bessel_jn(1, 1.0) - 0.440_050_585_744_933_5).abs() < 1e-14);
        assert!((bessel_jn(1, 10.0) - 0.043_472_746_168_861_44).abs() < 1e-14);
        assert!((bessel_jn(0, 2.404_825_557_695_773).abs()) < 1e-14);
        // Асимптотика при больших x: J1(x) ≈ √(2/(πx))·cos(x − 3π/4) с поправкой O(1/x)
        let x = 300.0;
        let asym = (2.0 / (PI * x)).sqrt() * (x - 0.75 * PI).cos();
        assert!((bessel_jn(1, x) - asym).abs() < 5e-4);
    }

    #[test]
    fn quadratures() {
        let gl = GaussLegendre::new(12);
        let v = gl.integrate(0.0, 1.0, |x| x.powi(20));
        assert!((v - 1.0 / 21.0).abs() < 1e-15);
        // ∫₀¹ ln x dx = −1 (особенность на конце)
        let v = tanh_sinh(0.0, 1.0, 7, |x| x.ln());
        assert!((v + 1.0).abs() < 1e-12, "{v}");
        // ∫₀¹ 1/√x = 2
        let v = tanh_sinh(0.0, 1.0, 7, |x| 1.0 / x.sqrt());
        assert!((v - 2.0).abs() < 1e-10, "{v}");
    }
}
