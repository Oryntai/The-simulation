//! Ограниченные механические модели для численных свидетелей.

use crate::{Interval, ProcessError, SourcedRange, contact::ContactFrame};
use sim_signal::Time;

use super::{
    HazardCheck,
    math::*,
    trace::{matches_pair, validate_frame, validate_pair},
};

/// Длина осевой линии и максимум её кривизны. Каждый внутренний узел
/// заменён круговой дугой, касательной к двум отрезкам на заданном отступе
/// от вершины. Наложение зон скругления и разворот на π не задают такой трассы.
pub(super) fn rounded_route(
    points_m: &[[f64; 3]],
    setbacks_m: &[f64],
) -> Result<Option<[f64; 2]>, ProcessError> {
    let mut spans = Vec::with_capacity(points_m.len() - 1);
    for points in points_m.windows(2) {
        let delta = sub(points[1], points[0]);
        let length = norm(delta);
        computed(&[length])?;
        if length == 0.0 {
            return Ok(None);
        }
        spans.push((length, delta.map(|x| x / length)));
    }
    let mut length = 0.0;
    for (index, &(span_length, _)) in spans.iter().enumerate() {
        // У концов трассы скруглений нет. Внутри отрезка остаётся только
        // длина между двумя точками касания.
        let left = index.checked_sub(1).map_or(0.0, |i| setbacks_m[i]);
        let right = setbacks_m.get(index).copied().unwrap_or(0.0);
        let straight = span_length - left - right;
        if straight < 0.0 {
            return Ok(None);
        }
        length += straight;
    }
    let mut peak_curvature: f64 = 0.0;
    for (spans, &setback) in spans.windows(2).zip(setbacks_m) {
        let incoming = spans[0].1;
        let outgoing = spans[1].1;
        // Для единичных касательных отношение норм разности и суммы —
        // tan(θ/2). Из треугольника радиус равен setback / tan(θ/2).
        let sine = norm(sub(incoming, outgoing));
        let cosine = norm(add(incoming, outgoing));
        if cosine == 0.0 {
            return Ok(None);
        }
        if sine == 0.0 {
            length += 2.0 * setback; // Предел длины дуги при θ → 0.
            continue;
        }
        let tangent = sine / cosine;
        let curvature = tangent / setback;
        let angle = 2.0 * sine.atan2(cosine);
        let arc_length = setback * (angle / tangent);
        computed(&[curvature, arc_length])?;
        if curvature == 0.0 || arc_length == 0.0 {
            return Ok(None);
        }
        length += arc_length;
        peak_curvature = peak_curvature.max(curvature);
    }
    computed(&[length, peak_curvature])?;
    Ok(Some([length, peak_curvature]))
}

/// Сравнение фактического значения с огибающей допустимого максимума.
pub fn upper_check(
    value: f64,
    limit: &SourcedRange,
    scope: &str,
) -> Result<HazardCheck, ProcessError> {
    finite(&[value])?;
    name(scope)?;
    range(limit, "верхний предел", |x| x >= 0.0)?;
    Ok(if !limit.can_prove() {
        HazardCheck::NotAnalyzed {
            reason: "источник предела не подтверждён".into(),
        }
    } else if value > limit.max {
        HazardCheck::Violation {
            reason: format!("{}. Превышен верхний предел", scope.trim_end_matches('.')),
        }
    } else if value <= limit.min {
        HazardCheck::WithinLimit {
            scope: scope.into(),
        }
    } else {
        HazardCheck::NotAnalyzed {
            reason: "измерение внутри диапазона предела".into(),
        }
    })
}

/// Суммарная величина сжимающих нормальных составляющих одной пары.
pub fn normal_load(
    interval: Interval,
    frame: &ContactFrame,
    pair: &[String; 2],
) -> Result<f64, ProcessError> {
    validate_pair(pair)?;
    validate_frame(frame, interval)?;
    let value = pair_load(frame, pair, None);
    computed(&[value])?;
    Ok(value)
}

/// Одновременное сжатие общего тела, минимальная осевая реакция двух сторон.
/// Вторые геометрии обеих пар должны принадлежать этому телу; первый контакт
/// направлен вдоль оси, второй — против неё.
pub fn opposed_load(
    interval: Interval,
    frame: &ContactFrame,
    pairs: &[[String; 2]; 2],
    axis: [f64; 3],
) -> Result<f64, ProcessError> {
    for pair in pairs {
        validate_pair(pair)?;
    }
    if pairs[0][0] == pairs[1][0] || pairs[0].iter().eq(pairs[1].iter().rev()) {
        return Err(invalid("для сжатия нужны две разные стороны"));
    }
    validate_frame(frame, interval)?;
    let axis = unit(axis)?;
    let positive = pair_load(frame, &pairs[0], Some(axis));
    let negative = pair_load(frame, &pairs[1], Some(scale(axis, -1.0)));
    computed(&[positive, negative])?;
    Ok(positive.min(negative))
}

fn pair_load(frame: &ContactFrame, pair: &[String; 2], axis: Option<[f64; 3]>) -> f64 {
    frame
        .contacts
        .iter()
        // Реакция уже подтверждена решателем. Податливый контакт с зазором
        // может передавать силу до совпадения номинальных поверхностей.
        .filter(|sample| matches_pair(sample, pair))
        .map(|sample| {
            let length = norm(sample.normal);
            let normal = sample.normal.map(|value| value / length);
            let load = dot(sample.force_n, normal).max(0.0);
            let projection = axis.map_or(1.0, |axis| {
                let sign = if sample.geometries[1] == pair[1] {
                    1.0
                } else {
                    -1.0
                };
                (sign * dot(normal, axis)).max(0.0)
            });
            load * projection
        })
        .sum()
}

/// Интеграл модуля нормальной силы за непрерывные физические интервалы.
#[derive(Clone, Debug)]
pub struct NormalImpulse {
    end: Time,
    value: f64,
}

impl NormalImpulse {
    pub fn new(start: Time) -> Self {
        Self {
            end: start,
            value: 0.0,
        }
    }

    pub fn advance(&mut self, interval: Interval, force_n: f64) -> Result<f64, ProcessError> {
        let dt = continuous(Some(self.end), interval)?;
        finite(&[force_n])?;
        if force_n < 0.0 {
            return Err(invalid("нормальная сила должна быть неотрицательной"));
        }
        let value = self.value + force_n * dt;
        computed(&[value])?;
        self.end = interval.end;
        self.value = value;
        Ok(value)
    }
}

/// Скорость сближения двух материальных точек вдоль соединяющей их прямой.
pub fn closing_speed(
    positions_m: [[f64; 3]; 2],
    velocities_m_s: [[f64; 3]; 2],
) -> Result<Option<(f64, f64)>, ProcessError> {
    for vector in positions_m.into_iter().chain(velocities_m_s) {
        finite(&vector)?;
    }
    let delta = sub(positions_m[1], positions_m[0]);
    let distance = norm(delta);
    computed(&[distance])?;
    if distance == 0.0 {
        return Ok(None);
    }
    let direction = delta.map(|component| component / distance);
    let speed = dot(sub(velocities_m_s[0], velocities_m_s[1]), direction);
    computed(&[speed])?;
    Ok(Some((distance, speed.max(0.0))))
}

/// Амплитуда изгибного напряжения по трём выдержанным фазам A→B→A.
/// Работа в упругой области и применимость точки S–N проверяются в постановке.
pub fn reversed_bending_stress(
    moments_nm: [f64; 3],
    section_modulus_m3: &SourcedRange,
    reversal_error_nm: &SourcedRange,
) -> Result<Option<[f64; 2]>, ProcessError> {
    finite(&moments_nm)?;
    range(section_modulus_m3, "модуль сечения", |x| {
        x > 0.0
    })?;
    range(reversal_error_nm, "невязка симметрии цикла", |x| x >= 0.0)?;
    let [a, b, c] = moments_nm;
    let residuals = [a + b, b + c, a - c];
    computed(&residuals)?;
    if b == 0.0
        || b.is_sign_positive() == a.is_sign_positive()
        || b.is_sign_positive() == c.is_sign_positive()
        || a == 0.0
        || c == 0.0
        || residuals
            .into_iter()
            .any(|r| r.abs() > reversal_error_nm.min)
    {
        return Ok(None);
    }
    // Полуразмахи двух наблюдаемых переходов и крайние значения модуля.
    let first = (a * 0.5 - b * 0.5).abs();
    let second = (c * 0.5 - b * 0.5).abs();
    let value = [
        first.min(second) / section_modulus_m3.max,
        first.max(second) / section_modulus_m3.min,
    ];
    computed(&value)?;
    Ok(Some(value))
}

fn unit(vector: [f64; 3]) -> Result<[f64; 3], ProcessError> {
    finite(&vector)?;
    let length = norm(vector);
    computed(&[length])?;
    if length == 0.0 {
        return Err(invalid("нулевая ось сжатия"));
    }
    Ok(vector.map(|x| x / length))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact::ContactSample;
    use sim_world::Provenance;

    fn point(nominal: f64) -> SourcedRange {
        SourcedRange {
            min: nominal,
            nominal,
            max: nominal,
            provenance: Provenance::Datasheet("Условия численного опыта".into()),
        }
    }

    #[test]
    fn сжатие_требует_двух_встречных_реакций() {
        let step = Interval {
            start: Time::ZERO,
            end: Time(1_000_000),
        };
        let pairs = [
            ["опора".into(), "блок".into()],
            ["каретка".into(), "блок".into()],
        ];
        let sample = |geometries, normal, force_n| ContactSample {
            geometries,
            normal,
            force_n,
            point_m: [0.0; 3],
            separation_m: 0.0,
            torque_nm: [0.0; 3],
        };
        // Полярность второй пары обратна порядку геометрий в журнале.
        let mut frame = ContactFrame {
            at: step.end,
            contacts: vec![
                sample(pairs[0].clone(), [1.0, 0.0, 0.0], [3.0, 10.0, 0.0]),
                sample(
                    ["блок".into(), "каретка".into()],
                    [1.0, 0.0, 0.0],
                    [2.0, 0.0, 0.0],
                ),
            ],
        };
        assert_eq!(normal_load(step, &frame, &pairs[0]).unwrap(), 3.0);
        assert_eq!(
            opposed_load(step, &frame, &pairs, [1.0, 0.0, 0.0]).unwrap(),
            2.0
        );
        frame.contacts[1].normal[0] = -1.0;
        frame.contacts[1].force_n[0] = -2.0;
        assert_eq!(
            opposed_load(step, &frame, &pairs, [1.0, 0.0, 0.0]).unwrap(),
            0.0
        );
        frame.contacts[0].separation_m = 0.001; // Положительный зазор при нагруженной связи.
        assert_eq!(normal_load(step, &frame, &pairs[0]).unwrap(), 3.0);
        frame.contacts[1].force_n[0] = f64::NAN;
        assert!(opposed_load(step, &frame, &pairs, [1.0, 0.0, 0.0]).is_err());
    }

    #[test]
    fn импульс_не_переступает_через_пропуски() {
        let mut sum = NormalImpulse::new(Time::ZERO);
        let first = Interval {
            start: Time::ZERO,
            end: Time(1_000_000),
        };
        // 2 Н на 1 мс, затем 3 Н на 2 мс: 0,008 Н·с.
        assert_eq!(sum.advance(first, 2.0).unwrap(), 0.002);
        let gap = Interval {
            start: Time(2_000_000),
            end: Time(3_000_000),
        };
        assert!(sum.advance(gap, 3.0).is_err());
        let next = Interval {
            start: first.end,
            end: gap.end,
        };
        assert!(sum.advance(next, f64::INFINITY).is_err());
        assert_eq!(sum.advance(next, 3.0).unwrap(), 0.008);
    }

    #[test]
    fn сближение_зависит_от_относительной_скорости() {
        let points = [[0.0; 3], [0.0, 0.0, 3.0]];
        // Скорость 6 м/с поперёк прямой не влияет на её длину.
        assert_eq!(
            closing_speed(points, [[6.0, 0.0, 2.0], [0.0, 0.0, 1.0]]).unwrap(),
            Some((3.0, 1.0))
        );
        assert_eq!(
            closing_speed(points, [[6.0, 0.0, 1.0], [0.0, 0.0, 2.0]]).unwrap(),
            Some((3.0, 0.0))
        );
        assert_eq!(closing_speed([[0.0; 3]; 2], [[0.0; 3]; 2]).unwrap(), None);
        assert!(closing_speed(points, [[f64::NAN; 3]; 2]).is_err());
    }

    #[test]
    fn изгиб_требует_доказанной_смены_знака_и_замыкания() {
        // ±8 Н·м, крайние модули 1 и 2 м³: амплитуда от 4 до 8 Па.
        let mut section = point(2.0);
        section.min = 1.0;
        let tolerance = point(0.1);
        assert_eq!(
            reversed_bending_stress([8.0, -8.0, 8.0], &section, &tolerance).unwrap(),
            Some([4.0, 8.0])
        );
        assert_eq!(
            reversed_bending_stress([8.0, 8.0, 8.0], &section, &tolerance).unwrap(),
            None
        );
        assert_eq!(
            reversed_bending_stress([8.0, -8.0, 6.0], &section, &tolerance).unwrap(),
            None
        );
        assert!(reversed_bending_stress([8.0, f64::INFINITY, 8.0], &section, &tolerance).is_err());
        let mut limit = point(5.0);
        assert!(matches!(
            upper_check(8.0, &limit, "нагрузка").unwrap(),
            HazardCheck::Violation { .. }
        ));
        limit.provenance = Provenance::Estimate("грубый ориентир".into());
        assert!(matches!(
            upper_check(8.0, &limit, "нагрузка").unwrap(),
            HazardCheck::NotAnalyzed { .. }
        ));
    }
}
