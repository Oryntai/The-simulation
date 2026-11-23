//! Сверки с эталоном: каждая — число нашей модели против числа, полученного другим,
//! независимым способом (аналитика, таблица/стандарт/даташит, внешняя программа).
//!
//! * [`circuit`] — решатель схем против аналитических решений;
//! * [`battery`] — модель ячейки против аналитики и опубликованных измерений;
//! * [`pcb`] — расчёты плат против опубликованных калькуляторов, стандарта IPC-2152, учебника;
//! * [`ngspice_cmp`] — те же нетлисты в ngspice (если программа найдена).

pub mod battery;
pub mod circuit;
pub mod ngspice_cmp;
pub mod pcb;

use sim_verify::{Check, Tolerance};

/// Собрать проверку (сокращение для конструктора).
#[allow(clippy::too_many_arguments)]
pub(crate) fn mk(
    id: &'static str,
    title: &str,
    simulated: f64,
    reference: f64,
    unit: &'static str,
    method: &str,
    reference_method: &str,
    tolerance: Tolerance,
    note: &str,
) -> Check {
    Check {
        id,
        title: title.to_string(),
        simulated,
        reference,
        unit,
        method: method.to_string(),
        reference_method: reference_method.to_string(),
        tolerance,
        note: note.to_string(),
    }
}

/// Проверка, которую не удалось даже посчитать (ошибка решателя): видна как провал с причиной.
pub(crate) fn failed(id: &'static str, title: &str, unit: &'static str, why: String) -> Check {
    Check {
        id,
        title: title.to_string(),
        simulated: f64::NAN,
        reference: 0.0,
        unit,
        method: format!("расчёт не выполнен: {why}"),
        reference_method: String::new(),
        tolerance: Tolerance::Relative(0.0),
        note: String::new(),
    }
}

/// Моменты пересечения нуля (линейная интерполяция). `rising`: снизу вверх, иначе сверху вниз.
pub(crate) fn zero_crossings(t: &[f64], y: &[f64], rising: bool) -> Vec<f64> {
    let mut out = Vec::new();
    for k in 1..t.len() {
        let (a, b) = (y[k - 1], y[k]);
        let hit = if rising {
            a < 0.0 && b >= 0.0
        } else {
            a > 0.0 && b <= 0.0
        };
        if hit {
            out.push(t[k - 1] + (t[k] - t[k - 1]) * a / (a - b));
        }
    }
    out
}

/// Линейная интерполяция ряда (t, y) в момент `tq`.
pub(crate) fn interp(t: &[f64], y: &[f64], tq: f64) -> f64 {
    let k = t.partition_point(|&x| x < tq).clamp(1, t.len() - 1);
    let (t0, t1) = (t[k - 1], t[k]);
    y[k - 1] + (y[k] - y[k - 1]) * (tq - t0) / (t1 - t0)
}

/// Решение уравнения w + ln w = l (w > 0), то есть W(eᴸ) — главная ветвь функции Ламберта
/// для аргумента eᴸ. Через логарифм, чтобы не переполняться при больших l. Метод Галлея.
pub fn lambert_w_of_exp(l: f64) -> f64 {
    let mut w = if l > 1.0 {
        l - l.ln()
    } else {
        l.exp().max(1e-300)
    };
    for _ in 0..100 {
        let f = w + w.ln() - l;
        let f1 = 1.0 + 1.0 / w;
        let f2 = -1.0 / (w * w);
        let step = f / (f1 - 0.5 * f * f2 / f1);
        let next = (w - step).max(w * 1e-3);
        if (next - w).abs() <= 1e-15 * next.abs() {
            return next;
        }
        w = next;
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lambert_w_identity() {
        for l in [-20.0, -2.0, 0.0, 1.0, 5.0, 50.0, 700.0, 5000.0] {
            let w = lambert_w_of_exp(l);
            assert!(
                (w + w.ln() - l).abs() < 1e-12 * l.abs().max(1.0),
                "l={l}, w={w}"
            );
        }
        // W(1) = Ω ≈ 0.5671432904097838.
        assert!((lambert_w_of_exp(0.0) - 0.567_143_290_409_783_8).abs() < 1e-15);
    }
}
