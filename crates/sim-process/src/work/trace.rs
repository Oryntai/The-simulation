//! След от контакта с плоской поверхностью в системе координат мира.

use sim_signal::{Rng, Time};

use crate::contact::{ContactFrame, ContactSample};
use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::math::*;

#[derive(Clone, Debug, PartialEq)]
pub struct TraceSpec {
    /// Имена поверхности и оставляющей след геометрии.
    pub pair: [String; 2],
    pub plane_point_m: [SourcedRange; 3],
    pub plane_normal: [SourcedRange; 3],
    /// Максимальное расстояние контактной точки до поверхности, включая вдавливание.
    pub plane_tolerance_m: SourcedRange,
    /// Эталонная ломаная на поверхности в системе мира.
    pub reference_m: Vec<[SourcedRange; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TraceStep {
    NoContact,
    Mark {
        point_m: [f64; 3],
        plane_error_m: f64,
        path_error_m: f64,
        /// Полилинейная длина до этого отсчёта по соседним шагам с контактом.
        sampled_length_m: f64,
    },
}

#[derive(Clone, Debug)]
pub struct SurfaceTrace {
    spec: TraceSpec,
    origin: Vec3,
    normal: Vec3,
    reference: Vec<Vec3>,
    previous: Option<Vec3>,
    sampled_length_m: f64,
    last: Option<Time>,
}

impl SurfaceTrace {
    pub fn new(spec: TraceSpec) -> Result<Self, ProcessError> {
        validate_pair(&spec.pair)?;
        let origin = point(&spec.plane_point_m)?;
        let normal = direction(point(&spec.plane_normal)?)?;
        let tolerance = range(
            &spec.plane_tolerance_m,
            "отклонение от плоскости",
            |x| x >= 0.0,
        )?;
        let mut reference = Vec::new();
        for entry in &spec.reference_m {
            let offset = sub(point(entry)?, origin);
            let height = dot(offset, normal);
            finite(&[height])?;
            if height.abs() > tolerance {
                return Err(invalid("эталонная линия за пределами допуска плоскости"));
            }
            let projected = sub(offset, scale(normal, height));
            finite(&projected)?;
            if let Some(&previous) = reference.last() {
                direction(sub(projected, previous))?;
            }
            reference.push(projected);
        }
        if reference.len() < 2 {
            return Err(invalid("для эталонного пути нужны отрезки"));
        }
        Ok(Self {
            spec,
            origin,
            normal,
            reference,
            previous: None,
            sampled_length_m: 0.0,
            last: None,
        })
    }
}

impl StepModel for SurfaceTrace {
    type Input = Option<ContactFrame>;
    type Output = Analysis<TraceStep>;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        continuous(self.last, interval)?;
        let mut next_point = None;
        let mut length = self.sampled_length_m;
        let result = if let Some(frame) = input {
            validate_frame(frame, interval)?;
            let mut touching = frame.contacts.iter().filter(|c| {
                matches_pair(c, &self.spec.pair)
                    && c.separation_m <= 0.0
                    && dot(c.force_n, c.normal) > 0.0
            });
            match (touching.next(), touching.next()) {
                (None, _) => calculated(
                    TraceStep::NoContact,
                    "за интервал нет нагруженного контакта пары",
                ),
                (Some(_), Some(_)) => {
                    unknown("несколько контактных точек: единственный след не определён")
                }
                (Some(c), None) => {
                    let offset = sub(c.point_m, self.origin);
                    let height = dot(offset, self.normal);
                    computed(&[height])?;
                    if height.abs() > self.spec.plane_tolerance_m.nominal {
                        unknown("контакт за пределами допуска плоскости")
                    } else {
                        let p = sub(offset, scale(self.normal, height));
                        computed(&p)?;
                        let mut distance = f64::INFINITY;
                        for segment in self.reference.windows(2) {
                            let edge = sub(segment[1], segment[0]);
                            let dir = direction(edge)?;
                            let offset = sub(p, segment[0]);
                            let along = dot(offset, dir);
                            computed(&[along])?;
                            let nearest = scale(dir, along.clamp(0.0, norm(edge)));
                            let error = norm(sub(offset, nearest));
                            computed(&[error])?;
                            distance = distance.min(error);
                        }
                        if let Some(previous) = self.previous {
                            length += norm(sub(p, previous));
                        }
                        let mark = add(self.origin, p);
                        computed(&mark)?;
                        computed(&[length, distance])?;
                        next_point = Some(p);
                        calculated(
                            TraceStep::Mark {
                                point_m: mark,
                                plane_error_m: height.abs(),
                                path_error_m: distance,
                                sampled_length_m: length,
                            },
                            "отсчёт единственного нагруженного контакта, длина ломаной между соседними отсчётами",
                        )
                    }
                }
            }
        } else {
            unknown("отсутствует запись контактов физического интервала")
        };
        self.previous = next_point;
        self.sampled_length_m = length;
        self.last = Some(interval.end);
        Ok(result)
    }
}

pub(super) fn validate_pair(pair: &[String; 2]) -> Result<(), ProcessError> {
    name(&pair[0])?;
    name(&pair[1])?;
    if pair[0] == pair[1] {
        return Err(invalid("пара содержит одну и ту же геометрию"));
    }
    Ok(())
}

pub(super) fn validate_frame(frame: &ContactFrame, interval: Interval) -> Result<(), ProcessError> {
    if frame.at != interval.end {
        return Err(invalid("кадр контактов относится к другому интервалу"));
    }
    for sample in &frame.contacts {
        validate_pair(&sample.geometries)?;
        finite(&sample.point_m)?;
        let normal = direction(sample.normal)?;
        finite(&sample.force_n)?;
        finite(&sample.torque_nm)?;
        finite(&[sample.separation_m])?;
        computed(&[
            dot(sample.force_n, normal),
            dot(sample.force_n, sample.normal),
        ])?;
    }
    Ok(())
}

pub(super) fn matches_pair(c: &ContactSample, pair: &[String; 2]) -> bool {
    c.geometries == *pair || c.geometries[0] == pair[1] && c.geometries[1] == pair[0]
}
