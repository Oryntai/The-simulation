//! Аналитические решения — «второй независимый способ». Каждая формула — со ссылкой и допущениями.

use std::f64::consts::PI;

/// Арифметико-геометрическое среднее (Гаусс). Сходится квадратично.
pub fn agm(mut a: f64, mut b: f64) -> f64 {
    for _ in 0..64 {
        let (an, bn) = (0.5 * (a + b), (a * b).sqrt());
        if (an - bn).abs() <= 1e-16 * an {
            return an;
        }
        a = an;
        b = bn;
    }
    a
}

/// Полный эллиптический интеграл 1-го рода K(k) через AGM: K = π / (2·AGM(1, √(1−k²))).
/// (Abramowitz & Stegun, 17.6.)
pub fn elliptic_k(k: f64) -> f64 {
    PI / (2.0 * agm(1.0, (1.0 - k * k).sqrt()))
}

/// Период математического маятника при малых углах: T = 2π√(L/g).
pub fn pendulum_period_small(l: f64, g: f64) -> f64 {
    2.0 * PI * (l / g).sqrt()
}

/// Точный период математического маятника при амплитуде θ₀ (любой, < π):
/// T = 4√(L/g)·K(sin(θ₀/2)).
pub fn pendulum_period_exact(l: f64, g: f64, theta0: f64) -> f64 {
    4.0 * (l / g).sqrt() * elliptic_k((theta0 / 2.0).sin())
}

/// Период физического маятника при малых углах: T = 2π√(I_оси / (m·g·d)).
pub fn physical_pendulum_period_small(i_pivot: f64, m: f64, d: f64, g: f64) -> f64 {
    2.0 * PI * (i_pivot / (m * g * d)).sqrt()
}

/// Консоль, сила P на конце: прогиб конца δ = P·L³ / (3·E·I).
pub fn cantilever_tip_point(p: f64, l: f64, ei: f64) -> f64 {
    p * l.powi(3) / (3.0 * ei)
}

/// Консоль, сила P на конце: угол поворота конца θ = P·L² / (2·E·I).
pub fn cantilever_tip_slope_point(p: f64, l: f64, ei: f64) -> f64 {
    p * l * l / (2.0 * ei)
}

/// Консоль, сила P на конце: прогиб в точке x: w(x) = P·x²·(3L − x) / (6·E·I).
pub fn cantilever_deflection_point(x: f64, p: f64, l: f64, ei: f64) -> f64 {
    p * x * x * (3.0 * l - x) / (6.0 * ei)
}

/// Консоль под равномерной нагрузкой q: прогиб конца δ = q·L⁴ / (8·E·I).
pub fn cantilever_tip_udl(q: f64, l: f64, ei: f64) -> f64 {
    q * l.powi(4) / (8.0 * ei)
}

/// Цепочка Хенки (N жёстких звеньев, шарниры с пружинами k = EI/h в серединах ячеек):
/// точный прогиб дискретной модели δ = P·L³/(3EI)·(1 − 1/(4N²)).
pub fn hencky_chain_tip(p: f64, l: f64, ei: f64, n: usize) -> f64 {
    let n = n as f64;
    cantilever_tip_point(p, l, ei) * (1.0 - 1.0 / (4.0 * n * n))
}

/// Большой прогиб консоли под вертикальной силой на конце — эластика Эйлера:
/// EI·θ''(s) = −P·cos θ, θ(0) = 0, θ'(L) = 0 (s — длина дуги, θ — угол наклона оси).
/// Решаем методом стрельбы: RK4 по s и бисекция по кривизне в заделке.
/// Возвращает вертикальный прогиб конца. Независим от многотельного движка и от МКЭ.
pub fn elastica_tip_deflection(p: f64, l: f64, ei: f64) -> f64 {
    let a = p / ei;
    let n = 4000;
    let h = l / n as f64;
    let rhs = |th: f64, k: f64| (k, -a * th.cos(), th.sin());
    let shoot = |k0: f64| {
        let (mut th, mut k, mut y) = (0.0_f64, k0, 0.0_f64);
        for _ in 0..n {
            let (a1, b1, c1) = rhs(th, k);
            let (a2, b2, c2) = rhs(th + 0.5 * h * a1, k + 0.5 * h * b1);
            let (a3, b3, c3) = rhs(th + 0.5 * h * a2, k + 0.5 * h * b2);
            let (a4, b4, c4) = rhs(th + h * a3, k + h * b3);
            th += h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
            k += h / 6.0 * (b1 + 2.0 * b2 + 2.0 * b3 + b4);
            y += h / 6.0 * (c1 + 2.0 * c2 + 2.0 * c3 + c4);
        }
        (k, y)
    };
    // θ'(L) < 0 при k0 = 0 и > 0 при k0 = P·L/EI (момент в заделке не больше линейного).
    let (mut lo, mut hi) = (0.0, a * l);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if shoot(mid).0 < 0.0 {
            lo = mid
        } else {
            hi = mid
        }
    }
    shoot(0.5 * (lo + hi)).1
}

/// Свободное падение без сопротивления воздуха: z(t) = z₀ + v₀·t − g·t²/2.
pub fn free_fall_z(z0: f64, v0: f64, g: f64, t: f64) -> f64 {
    z0 + v0 * t - 0.5 * g * t * t
}

/// v(t) = v₀ − g·t.
pub fn free_fall_v(v0: f64, g: f64, t: f64) -> f64 {
    v0 - g * t
}

/// Время падения с высоты h из покоя: t = √(2h/g).
pub fn free_fall_time(h: f64, g: f64) -> f64 {
    (2.0 * h / g).sqrt()
}

/// Скорость удара после падения с высоты h: v = √(2gh).
pub fn free_fall_impact_speed(h: f64, g: f64) -> f64 {
    (2.0 * g * h).sqrt()
}

/// Ошибка полунеявного Эйлера при постоянном ускорении: Δz = g·t·dt/2 (тело «отстаёт»).
pub fn semi_implicit_euler_fall_error(g: f64, t: f64, dt: f64) -> f64 {
    0.5 * g * t * dt
}

/// Статический момент от груза: M = m·g·L (плечо L — по горизонтали).
pub fn static_moment(m: f64, g: f64, lever: f64) -> f64 {
    m * g * lever
}

/// Напряжение изгиба σ = M·c / I.
pub fn bending_stress(moment: f64, c: f64, i: f64) -> f64 {
    moment * c / i
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elliptic_k_known_values() {
        // K(0) = π/2; K(sin 45°) = 1.854074677301372 (A&S, табл. 17.1).
        assert!((elliptic_k(0.0) - PI / 2.0).abs() < 1e-15);
        assert!((elliptic_k((PI / 4.0).sin()) - 1.854_074_677_301_372).abs() < 1e-13);
    }

    #[test]
    fn elastica_small_load_is_linear_and_large_load_is_stiffer() {
        let (l, ei) = (1.0, 100.0);
        let small = elastica_tip_deflection(1e-3, l, ei);
        assert!((small / cantilever_tip_point(1e-3, l, ei) - 1.0).abs() < 1e-6);
        // Bisshopp & Drucker (1945): при P·L²/EI = 1 прогиб конца ≈ 0.3017·L (линейная теория — 0.3333).
        let big = elastica_tip_deflection(100.0, l, ei);
        assert!((big - 0.3017).abs() < 1e-3, "эластика: {big}");
    }

    #[test]
    fn exact_period_tends_to_small_angle() {
        let t0 = pendulum_period_small(1.0, 9.81);
        let t = pendulum_period_exact(1.0, 9.81, 1e-4);
        assert!((t / t0 - 1.0).abs() < 1e-9);
        // Ряд: T ≈ T₀(1 + θ₀²/16 + 11θ₀⁴/3072)
        let th: f64 = 0.2;
        let series = t0 * (1.0 + th * th / 16.0 + 11.0 * th.powi(4) / 3072.0);
        assert!((pendulum_period_exact(1.0, 9.81, th) / series - 1.0).abs() < 1e-6);
    }
}
