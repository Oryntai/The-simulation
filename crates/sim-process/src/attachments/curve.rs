use crate::{ProcessError, SourcedRange};

use super::math::{finite, invalid, range};

/// Узел измеренной кусочно-линейной характеристики. За крайними узлами экстраполяции нет.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePoint {
    pub displacement_m: SourcedRange,
    pub force_n: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ForceCurve {
    points: Vec<CurvePoint>,
    areas_j: Vec<f64>,
}

impl ForceCurve {
    pub fn new(points: Vec<CurvePoint>) -> Result<Self, ProcessError> {
        if points.len() < 2 || points[0].displacement_m.max != 0.0 {
            return Err(invalid(
                "кривая должна начинаться с нулевого хода и содержать отрезок",
            ));
        }
        for point in &points {
            range(&point.displacement_m, 0.0, f64::MAX)?;
            range(&point.force_n, 0.0, f64::MAX)?;
        }
        let mut areas_j = vec![0.0];
        for pair in points.windows(2) {
            let [a, b] = [&pair[0], &pair[1]];
            if a.displacement_m.max >= b.displacement_m.min {
                return Err(invalid("диапазоны соседних узлов кривой перекрываются"));
            }
            let area = areas_j.last().unwrap()
                + trapezoid(
                    b.displacement_m.nominal - a.displacement_m.nominal,
                    a.force_n.nominal,
                    b.force_n.nominal,
                );
            finite([area])?;
            areas_j.push(area);
        }
        Ok(Self { points, areas_j })
    }

    pub fn points(&self) -> &[CurvePoint] {
        &self.points
    }

    pub fn maximum_m(&self) -> f64 {
        self.points[self.points.len() - 1].displacement_m.nominal
    }

    /// Сила и интеграл силы от нуля до заданного хода на номинальной характеристике.
    pub fn sample(&self, x_m: f64) -> Result<Option<(f64, f64)>, ProcessError> {
        finite([x_m])?;
        if x_m < 0.0 || x_m > self.maximum_m() {
            return Ok(None);
        }
        let i = self
            .points
            .partition_point(|p| p.displacement_m.nominal <= x_m)
            - 1;
        let a = &self.points[i];
        let dx = x_m - a.displacement_m.nominal;
        let force = self.points.get(i + 1).map_or(a.force_n.nominal, |b| {
            let t = dx / (b.displacement_m.nominal - a.displacement_m.nominal);
            (1.0 - t) * a.force_n.nominal + t * b.force_n.nominal
        });
        let area = self.areas_j[i] + trapezoid(dx, a.force_n.nominal, force);
        finite([force, area])?;
        Ok(Some((force, area)))
    }

    pub(super) fn at(&self, x_m: f64) -> Result<(f64, f64), ProcessError> {
        self.sample(x_m)?
            .ok_or_else(|| invalid("ход вне области характеристики"))
    }

    pub(super) fn compression(&self) -> Result<(), ProcessError> {
        if self.points[0].force_n.max != 0.0
            || self
                .points
                .windows(2)
                .any(|pair| pair[0].force_n.max > pair[1].force_n.min)
            || self.points.last().unwrap().force_n.min <= 0.0
        {
            return Err(invalid("кривая сжатия должна расти от нулевой силы"));
        }
        Ok(())
    }
}

fn trapezoid(dx: f64, a: f64, b: f64) -> f64 {
    // Площадь под линейным отрезком равна среднему двух концов, умноженному на длину.
    dx * (a * 0.5 + b * 0.5)
}
