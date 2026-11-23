use crate::config::{Electrical, Thermal};
use crate::input::BridgeCommand;

/// Цель моста. Ток каждой фазы задаётся отдельно, у коллекторного двигателя фаза одна.
#[derive(Clone, Copy)]
pub(crate) enum Bridge {
    Open,
    Coast,
    Short,
    Duty(f64),
    Current([f64; 2]),
    /// Напряжение проводящих диодов, без активного ограничения тока.
    Diode(f64),
}

impl Bridge {
    pub(crate) fn mode(self) -> BridgeCommand {
        match self {
            Self::Open | Self::Coast | Self::Diode(_) => BridgeCommand::Coast,
            Self::Short => BridgeCommand::Short,
            Self::Duty(_) | Self::Current(_) => BridgeCommand::Drive,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Winding {
    pub current: f64,
    pub voltage: f64,
    pub mean: f64,
    pub electrical_j: f64,
    pub copper_j: f64,
    pub limit_exceeded: bool,
}

pub(crate) fn winding(
    p: Electrical,
    resistance: f64,
    initial: f64,
    emf: f64,
    bridge: Bridge,
    phase: usize,
    dt: f64,
) -> Result<Winding, String> {
    if matches!(bridge, Bridge::Open) {
        // Разомкнутый мост: энергия остаточного поля рассеивается в цепи гашения.
        return Ok(Winding::default());
    }
    if matches!(bridge, Bridge::Coast) {
        return coast(p, resistance, initial, emf, dt);
    }
    let x = resistance * dt / p.inductance;
    let gain = if p.inductance == 0.0 {
        1.0 / resistance
    } else if x == 0.0 {
        dt / p.inductance
    } else {
        -(-x).exp_m1() / resistance
    };
    if !gain.is_finite() || gain <= 0.0 {
        return Err("переход R–L вне численного диапазона".into());
    }
    let hold = initial.mul_add(resistance, emf);
    let inverse = |target: f64| (target - initial).mul_add(1.0 / gain, hold);
    let mut voltage = match bridge {
        Bridge::Duty(duty) => duty * p.supply,
        Bridge::Current(target) => inverse(target[phase].clamp(-p.limit, p.limit)),
        Bridge::Short => 0.0,
        Bridge::Diode(rail) => rail,
        Bridge::Open | Bridge::Coast => unreachable!(),
    }
    .clamp(-p.supply, p.supply);
    let prediction = (voltage - hold).mul_add(gain, initial);
    if !matches!(bridge, Bridge::Short | Bridge::Diode(_)) && prediction.abs() > p.limit {
        // Если ЭДС вышла за возможности моста, ограничить ток таким напряжением
        // невозможно. Ток продолжает эволюцию по R–L, публикуется признак выхода.
        voltage = inverse(prediction.clamp(-p.limit, p.limit)).clamp(-p.supply, p.supply);
    }
    let delta = (voltage - hold) * gain;
    let current = initial + delta;
    let (mean, square) = if p.inductance == 0.0 {
        (current, current * current)
    } else if x < 0.05 {
        // Вблизи нуля разность экспоненциальных моментов теряет точность.
        // Трёхточечная квадратура Гаусса на [0, 1], точная до степени 5.
        let offset = (3.0_f64 / 5.0).sqrt() / 2.0;
        let points = [0.5 - offset, 0.5, 0.5 + offset];
        let weights = [5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0];
        let mut mean = 0.0;
        let mut square = 0.0;
        for (point, weight) in points.into_iter().zip(weights) {
            let progress = if x == 0.0 {
                point
            } else {
                (-x * point).exp_m1() / (-x).exp_m1()
            };
            let i = delta.mul_add(progress, initial);
            mean += weight * i;
            square += weight * i * i;
        }
        (mean, square)
    } else {
        // Среднее и дисперсия exp(−Rt/L) на интервале.
        let beta = -(-x).exp_m1() / x;
        let gamma = -(-2.0 * x).exp_m1() / (2.0 * x);
        let amplitude = (voltage - hold) / resistance;
        let mean = amplitude.mul_add(1.0 - beta, initial);
        (
            mean,
            mean * mean + amplitude * amplitude * (gamma - beta * beta),
        )
    };
    let electrical_j = dt * voltage * mean;
    let copper_j = dt * resistance * square;
    if ![current, mean, electrical_j, copper_j]
        .into_iter()
        .all(f64::is_finite)
        || copper_j < 0.0
    {
        return Err("ток или энергия обмотки вне численного диапазона".into());
    }
    // Запас на обратную подстановку уставки в экспоненту и округление.
    let tolerance = 8.0 * f64::EPSILON * p.limit.max(initial.abs());
    Ok(Winding {
        current,
        voltage,
        mean,
        electrical_j,
        copper_j,
        limit_exceeded: initial.abs().max(current.abs()) - p.limit > tolerance,
    })
}

/// Идеальные обратные диоды моста. При ненулевом токе проводит противоположная
/// шина. В нуле обмотка разомкнута, пока модуль ЭДС не превысит напряжение шины.
fn coast(p: Electrical, r: f64, initial: f64, emf: f64, dt: f64) -> Result<Winding, String> {
    if !emf.is_finite() {
        return Err("противо-ЭДС вне численного диапазона".into());
    }
    let from_zero = |dt| {
        if emf.abs() > p.supply {
            winding(p, r, 0.0, emf, Bridge::Diode(p.supply.copysign(emf)), 0, dt)
        } else {
            Ok(Winding {
                voltage: emf,
                ..Winding::default()
            })
        }
    };
    if initial == 0.0 || p.inductance == 0.0 {
        return from_zero(dt);
    }
    let rail = -p.supply.copysign(initial);
    let gap = (emf - rail) * initial.signum();
    if gap <= 0.0 {
        return winding(p, r, initial, emf, Bridge::Diode(rail), 0, dt);
    }

    // i(t) = i∞ + (i₀ − i∞) exp(−Rt/L). Время до нуля
    // равно (L/R) ln(1 + R|i₀|/|V−E|). После нуля решается другой контур.
    let to_zero = (p.inductance / r) * (r * initial.abs() / gap).ln_1p();
    if to_zero.is_nan() || to_zero <= 0.0 {
        return Err("момент гашения тока вне численного диапазона".into());
    }
    if to_zero > dt {
        return winding(p, r, initial, emf, Bridge::Diode(rail), 0, dt);
    }
    let first = winding(p, r, initial, emf, Bridge::Diode(rail), 0, to_zero)?;
    let after = dt - to_zero;
    if after == 0.0 {
        return Ok(Winding {
            current: 0.0,
            ..first
        });
    }
    let second = from_zero(after)?;
    let weight = to_zero / dt;
    Ok(Winding {
        current: second.current,
        voltage: first.voltage * weight + second.voltage * (after / dt),
        mean: first.mean * weight + second.mean * (after / dt),
        electrical_j: first.electrical_j + second.electrical_j,
        copper_j: first.copper_j + second.copper_j,
        limit_exceeded: first.limit_exceeded || second.limit_exceeded,
    })
}

impl Thermal {
    /// Трапеции для линейной тепловой цепи из двух RC-узлов, с замороженными
    /// на подшаг мощностями. Система 2×2 решается непосредственно.
    pub(crate) fn advance(
        self,
        [w, h]: [f64; 2],
        dt: f64,
        winding_j: f64,
        housing_j: f64,
    ) -> Result<[f64; 2], String> {
        let a = dt / (2.0 * self.c_w * self.r_wh);
        let b = dt / (2.0 * self.c_h * self.r_wh);
        let c = dt / (2.0 * self.c_h * self.r_ha);
        let delta = w - h;
        let rhs_w = winding_j / self.c_w - 2.0 * a * delta;
        let rhs_h = housing_j / self.c_h + 2.0 * (b * delta - c * (h - self.ambient));
        let determinant = 1.0 + a + b + c + a * c;
        let pair = [
            w + ((1.0 + b + c) * rhs_w + a * rhs_h) / determinant,
            h + (b * rhs_w + (1.0 + a) * rhs_h) / determinant,
        ];
        if pair.into_iter().all(f64::is_finite) {
            Ok(pair)
        } else {
            Err("температуры двухузловой модели вне численного диапазона".into())
        }
    }
}
