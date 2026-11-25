use crate::config::{Electrical, Thermal};
use crate::input::BridgeCommand;
use sim_signal::{Clock, Time};

/// Длительность удержания заряда в цепи задаётся целыми наносекундами.
pub(crate) fn bus_interval(clock: &mut Clock, at: Time) -> Result<f64, String> {
    const NS_PER_SECOND: u64 = 1_000_000_000;
    let index = clock
        .index()
        .checked_add(1)
        .ok_or("исчерпана сетка нагрузки")?;
    let rate = clock.rate();
    let estimate = index as f64 / rate.as_hz() * NS_PER_SECOND as f64;
    // Те же границы переполнения u128/u64 и округления, что в sim-runtime::time_at.
    if index > u64::MAX / NS_PER_SECOND || estimate >= u64::MAX as f64 * (1.0 - 4.0 * f64::EPSILON)
    {
        return Err("исчерпана сетка нагрузки схемы".into());
    }
    if clock.due() != at {
        return Err("пропущен такт нагрузки схемы".into());
    }
    clock.advance();
    Ok(Time(clock.due().0 - at.0).as_secs())
}

/// Цель моста. Ток каждой фазы задаётся отдельно, у коллекторного двигателя фаза одна.
#[derive(Clone, Copy)]
pub(crate) enum Bridge {
    Open,
    Coast,
    Short,
    Duty(f64),
    /// Дискретный мост без токового регулятора. Предел остаётся диагностическим.
    UnregulatedDuty(f64),
    Current([f64; 2]),
    /// Напряжение проводящих диодов, без активного ограничения тока.
    Diode(f64),
}

impl Bridge {
    pub(crate) fn mode(self) -> BridgeCommand {
        match self {
            Self::Open | Self::Coast | Self::Diode(_) => BridgeCommand::Coast,
            Self::Short => BridgeCommand::Short,
            Self::Duty(_) | Self::UnregulatedDuty(_) | Self::Current(_) => BridgeCommand::Drive,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Winding {
    pub current: f64,
    pub voltage: f64,
    pub mean: f64,
    pub bus_charge_c: f64,
    pub electrical_j: f64,
    pub copper_j: f64,
    pub limit_exceeded: bool,
    /// Момент перехода через ноль и напряжения до/после него.
    pub commutation: Option<(f64, f64, f64)>,
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
    if ![p.supply, resistance, initial, emf, dt]
        .into_iter()
        .all(f64::is_finite)
        || p.supply < 0.0
        || resistance <= 0.0
        || dt < 0.0
    {
        return Err("некорректный вход расчёта обмотки".into());
    }
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
        Bridge::Duty(duty) | Bridge::UnregulatedDuty(duty) => duty * p.supply,
        Bridge::Current(target) => inverse(target[phase].clamp(-p.limit, p.limit)),
        Bridge::Short => 0.0,
        Bridge::Diode(rail) => rail,
        Bridge::Open | Bridge::Coast => unreachable!(),
    }
    .clamp(-p.supply, p.supply);
    let prediction = (voltage - hold).mul_add(gain, initial);
    if !matches!(
        bridge,
        Bridge::Short | Bridge::Diode(_) | Bridge::UnregulatedDuty(_)
    ) && prediction.abs() > p.limit
    {
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
    // Для идеального моста i_ш = m·i_обм. Диоды соединяют обмотку с
    // шиной с коэффициентом ±1 даже при нуле вольт. В ветви регулятора
    // ниже делится только ограниченное напряжение команды на напряжение
    // шины, чтобы найти безразмерную скважность, а не мощность на вольты.
    let transfer = match bridge {
        Bridge::Diode(rail) => rail.signum(),
        Bridge::Duty(duty) | Bridge::UnregulatedDuty(duty) if voltage == duty * p.supply => duty,
        Bridge::Short => 0.0,
        _ if p.supply == 0.0 => 0.0,
        _ if voltage.abs() == p.supply => voltage.signum(),
        _ => voltage / p.supply,
    };
    let bus_charge_c = dt * (transfer * mean);
    let electrical_j = dt * voltage * mean;
    let copper_j = dt * resistance * square;
    if ![current, mean, bus_charge_c, electrical_j, copper_j]
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
        bus_charge_c,
        electrical_j,
        copper_j,
        limit_exceeded: initial.abs().max(current.abs()) - p.limit > tolerance,
        commutation: None,
    })
}

/// Идеальные обратные диоды моста. При ненулевом токе проводит противоположная
/// шина. В нуле обмотка разомкнута, пока модуль ЭДС не превысит напряжение шины.
fn coast(p: Electrical, r: f64, initial: f64, emf: f64, dt: f64) -> Result<Winding, String> {
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
        bus_charge_c: first.bus_charge_c + second.bus_charge_c,
        electrical_j: first.electrical_j + second.electrical_j,
        copper_j: first.copper_j + second.copper_j,
        limit_exceeded: first.limit_exceeded || second.limit_exceeded,
        commutation: Some((to_zero, first.voltage, second.voltage)),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn заряд_моста_учитывает_ограничение_тока_и_знаки_фаз() {
        // Синтетическая обмотка: R=2 Ом, L=2 мГн, Vш=8 В, предел 1 А.
        // На шаге τ ln 2 ток меняется на половину разности с установившимся.
        let p = Electrical {
            supply: 8.0,
            resistance: 2.0,
            inductance: 0.002,
            k: 1.0,
            limit: 1.0,
        };
        let tau = p.inductance / p.resistance;
        let dt = tau * std::f64::consts::LN_2;
        let limited = winding(p, p.resistance, 0.0, 0.0, Bridge::Duty(0.75), 0, dt).unwrap();
        // U=4 В выводит ток на 1 А ровно к границе. Интеграл тока в
        // обмотке = (U/R)(dt−τ/2). Ток второй фазы течёт обратно,
        // но её отрицательная скважность также даёт потребление шины.
        let expected = 0.5 * 2.0 * (dt - tau / 2.0);
        let bridge = Bridge::Current([1.0, -0.5]);
        let a = winding(p, p.resistance, 0.0, 0.0, bridge, 0, dt).unwrap();
        let b = winding(p, p.resistance, 0.0, 0.0, bridge, 1, dt).unwrap();
        let shorted = winding(p, p.resistance, 0.0, 1.0, Bridge::Short, 0, dt).unwrap();
        for result in [limited, a] {
            assert_eq!(result.current, 1.0);
            assert_eq!(result.voltage, 4.0);
            // Допуск на округление логарифма и экспоненциального момента.
            assert!((result.bus_charge_c - expected).abs() < 1e-17);
        }
        assert_eq!(b.current, -0.5);
        assert!((a.bus_charge_c + b.bus_charge_c - 1.25 * expected).abs() < 1e-17);
        assert_eq!(shorted.bus_charge_c, 0.0);
        assert!(shorted.copper_j > 0.0);
    }
}
