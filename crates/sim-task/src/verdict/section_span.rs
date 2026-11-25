//! Внутренние усилия в плоских сечениях прямой призмы.

use sim_world::{
    Geometry, Link, Mass, Material, SectionAxis, Shape, StrengthSection, strength_section,
};

use super::section_reaction::{Wrench, cross, sub};

type Vector = [f64; 3];

pub(super) fn add(a: Vector, b: Vector) -> Vector {
    std::array::from_fn(|i| a[i] + b[i])
}

pub(super) fn scale(a: Vector, factor: f64) -> Vector {
    a.map(|x| x * factor)
}

pub(super) fn norm([x, y, z]: Vector) -> f64 {
    x.hypot(y).hypot(z)
}

#[derive(Clone, Copy)]
pub(super) struct Motion {
    pub speed: f64,
    pub acceleration: f64,
    pub gravity: Vector,
    pub mass: f64,
}

#[derive(Clone, Copy)]
pub(super) enum Distribution<'a> {
    Uniform,
    Stepped {
        material: &'a Material,
        volumes: &'a [Geometry],
    },
    Translation {
        com: Vector,
        inertia_trace: f64,
    },
}

fn same_volume(a: &Geometry, b: &Geometry) -> bool {
    a.shape == b.shape && a.pose == b.pose
}

fn homogeneous<'a>(material: &'a Material, volumes: &'a [Geometry]) -> Option<Distribution<'a>> {
    match volumes {
        [] => None,
        [_] => Some(Distribution::Uniform),
        _ => Some(Distribution::Stepped { material, volumes }),
    }
}

pub(super) fn distribution(link: &Link, sliding: bool) -> Option<Distribution<'_>> {
    match &link.mass {
        Mass::Geometry { material } => homogeneous(material, &link.collision),
        Mass::Volumes { material, volumes } => {
            let mut unmatched: Vec<_> = link.collision.iter().collect();
            for volume in volumes {
                let index = unmatched
                    .iter()
                    .position(|collision| same_volume(collision, volume))?;
                unmatched.swap_remove(index);
            }
            unmatched
                .is_empty()
                .then(|| homogeneous(material, volumes))
                .flatten()
        }
        Mass::Explicit {
            com_m,
            inertia_kg_m2: [ix, iy, iz, ..],
            ..
        } if sliding && link.collision.len() == 1 => Some(Distribution::Translation {
            com: *com_m,
            inertia_trace: ix + iy + iz,
        }),
        _ => None,
    }
}

pub(super) struct Prism {
    mass: f64,
    center: Vector,
    variance: Vector,
}

fn volume_mass(geometry: &Geometry, material: &Material) -> f64 {
    if let Some(mass) = &geometry.explicit_mass {
        return mass.mass_kg;
    }
    let density = geometry.material.as_ref().unwrap_or(material).density_kg_m3;
    match geometry.shape {
        // Те же равномерные объёмы, по которым sim-world считает массу тела.
        Shape::Box { size_m: [x, y, z] } => density * x * y * z,
        Shape::Cylinder {
            radius_m: r,
            length_m: l,
        } => density * std::f64::consts::PI * r * r * l,
        _ => unreachable!("объём должен быть призмой постоянного сечения"),
    }
}

/// Соосные призмы со стыками торец к торцу. У каждой призмы свои размеры и масса.
/// Считаются односторонние сечения у каждого стыка, концентрация остаётся открытой.
pub(super) fn adjoining_prisms(
    current: &Geometry,
    axis: SectionAxis,
    i: usize,
    material: &Material,
    volumes: &[Geometry],
    local: impl Fn(Vector) -> Vector,
) -> Result<Option<(f64, Vec<Prism>)>, String> {
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut intervals = Vec::with_capacity(volumes.len());
    let mut adjacent = Vec::with_capacity(volumes.len());
    let mut own_mass = None;
    for volume in volumes {
        let StrengthSection::Applicable { length_m, .. } = strength_section(&volume.shape, axis)?
        else {
            return Ok(None);
        };
        for direction in axes {
            if volume.local_load_direction(direction)? != local(direction) {
                return Ok(None);
            }
        }
        let center = local(sub(volume.pose.position_m, current.pose.position_m));
        let [left, right] = [-1.0, 1.0].map(|sign| center[i] + sign * length_m / 2.0);
        let mass = volume_mass(volume, material);
        if !center
            .into_iter()
            .chain([left, right, mass])
            .all(f64::is_finite)
        {
            return Err("неконечная геометрия или масса ступенчатого профиля".into());
        }
        if (0..3).any(|j| j != i && center[j] != 0.0) || left >= right {
            return Ok(None);
        }
        intervals.push([left, right]);
        if same_volume(volume, current) {
            own_mass = Some(mass);
        } else {
            adjacent.push(Prism {
                mass,
                center,
                variance: variances(volume),
            });
        }
    }
    intervals.sort_by(|a, b| a[0].total_cmp(&b[0]));
    if !intervals.windows(2).all(|pair| pair[0][1] == pair[1][0]) {
        return Ok(None);
    }
    Ok(own_mass.map(|mass| (mass, adjacent)))
}

pub(super) fn variances(geometry: &Geometry) -> Vector {
    match geometry.shape {
        // Интеграл x² по равномерному отрезку длины l даёт l²/12.
        Shape::Box { size_m } => size_m.map(|l| l * l / 12.0),
        // Интеграл x² по равномерному кругу радиуса r даёт r²/4.
        Shape::Cylinder { radius_m, length_m } => [
            radius_m * radius_m / 4.0,
            radius_m * radius_m / 4.0,
            length_m * length_m / 12.0,
        ],
        _ => unreachable!("сечение должно быть постоянным"),
    }
}

/// Коэффициенты Бернштейна силы и момента между торцом и ближайшей
/// границей со стороны опоры. На плоскости опоры берётся односторонний предел.
pub(super) fn overhangs(
    axis: usize,
    length: f64,
    anchor: Vector,
    variance: Vector,
    direction: Vector,
    motion: Motion,
    sliding: bool,
    adjacent: &[Prism],
) -> impl Iterator<Item = [Wrench; 4]> + '_ {
    let half = length / 2.0;
    let omega = scale(direction, if sliding { 0.0 } else { motion.speed });
    let alpha = scale(direction, if sliding { 0.0 } else { motion.acceleration });
    let linear = if sliding {
        scale(direction, motion.acceleration)
    } else {
        [0.0; 3]
    };
    let gradient = move |r| add(cross(alpha, r), cross(omega, cross(omega, r)));
    let effective = move |point| add(sub(linear, motion.gravity), gradient(sub(point, anchor)));
    let rotational_moment = move |variance: Vector| {
        let inertia: Vector =
            std::array::from_fn(|i| variance[(i + 1) % 3] + variance[(i + 2) % 3]);
        let apply = |v: Vector| std::array::from_fn(|i| inertia[i] * v[i]);
        add(apply(alpha), cross(omega, apply(omega)))
    };
    [-1.0, 1.0].into_iter().filter_map(move |sign| {
        let depth = half - sign * anchor[axis].clamp(-half, half);
        if depth <= 0.0 {
            return None;
        }
        let mut tip = [0.0; 3];
        tip[axis] = sign * half;
        let mut lever = [0.0; 3];
        lever[axis] = sign * depth / 2.0;
        let mass = motion.mass * (depth / length);
        // Отрезок без опоры. Его длина t·depth, центр масс отстоит
        // от торца на −t·lever, а от сечения на +t·lever.
        // На дополнительной части внешняя реакция опоры даёт равное
        // по модулю внутреннее усилие по балансу всего тела.
        let force_linear = scale(effective(tip), mass);
        let force_quadratic = scale(gradient(lever), -mass);
        let mut cross_variance = variance;
        cross_variance[axis] = 0.0;
        let mut axial_variance = [0.0; 3];
        axial_variance[axis] = depth * depth / 12.0;
        let mut outboard = Wrench {
            force: [0.0; 3],
            moment: [0.0; 3],
        };
        for prism in adjacent
            .iter()
            .filter(|prism| sign * prism.center[axis] > 0.0)
        {
            let force = scale(effective(prism.center), prism.mass);
            let spin = scale(rotational_moment(prism.variance), prism.mass);
            outboard.force = add(outboard.force, force);
            outboard.moment = add(
                outboard.moment,
                add(spin, cross(sub(prism.center, tip), force)),
            );
        }
        // Масса за торцом постоянна. При движении плоскости сечения
        // её плечо меняется линейно на 2·t·lever.
        let moment_linear = add(
            scale(rotational_moment(cross_variance), mass),
            cross(scale(lever, 2.0), outboard.force),
        );
        let moment_quadratic = cross(lever, force_linear);
        let moment_cubic = add(
            cross(lever, force_quadratic),
            scale(rotational_moment(axial_variance), mass),
        );
        // Для полинома d + a·t + b·t² + c·t³ контрольные точки
        // Бернштейна степени 3 получаются добавлением d к (0, a/3, (2a+b)/3, a+b+c).
        Some([
            outboard,
            Wrench {
                force: add(outboard.force, scale(force_linear, 1.0 / 3.0)),
                moment: add(outboard.moment, scale(moment_linear, 1.0 / 3.0)),
            },
            Wrench {
                force: add(
                    outboard.force,
                    scale(add(scale(force_linear, 2.0), force_quadratic), 1.0 / 3.0),
                ),
                moment: add(
                    outboard.moment,
                    scale(add(scale(moment_linear, 2.0), moment_quadratic), 1.0 / 3.0),
                ),
            },
            Wrench {
                force: add(outboard.force, add(force_linear, force_quadratic)),
                moment: add(
                    outboard.moment,
                    add(add(moment_linear, moment_quadratic), moment_cubic),
                ),
            },
        ])
    })
}

/// Верхняя граница для отсечения произвольного подмножества массы при
/// поступательном движении. Напряжение в конкретном сечении неизвестно.
pub(super) fn translation_bound(
    axis: usize,
    length: f64,
    com: Vector,
    inertia_trace: f64,
    acceleration: Vector,
    mass: f64,
) -> Result<Wrench, String> {
    let force = scale(acceleration, mass);
    let end_moments = [-length / 2.0, length / 2.0].map(|end| {
        let mut offset = com;
        offset[axis] -= end;
        cross(offset, force)
    });
    if !end_moments.into_iter().flatten().all(f64::is_finite) {
        return Err("неконечный момент от поступательного ускорения".into());
    }
    // Для любой отсечённой части m' ≤ m. По Коши — Шварцу
    // ∫ |r−COM| dm' ≤ √(m·∫ |r−COM|² dm) = √(m·tr(I)/2).
    // Момент от смещения COM аффинен по положению сечения.
    let uncertainty = norm(acceleration) * mass.sqrt() * (inertia_trace / 2.0).sqrt();
    Ok(Wrench {
        force,
        moment: std::array::from_fn(|i| {
            end_moments[0][i].abs().max(end_moments[1][i].abs()) + uncertainty
        }),
    })
}

impl Wrench {
    pub(super) fn midpoint(control: [Self; 4]) -> Self {
        // Значение кубического полинома Бернштейна при t = 1/2.
        let [a, b, c, d] = control;
        let middle = |a, b, c, d| scale(add(add(a, d), scale(add(b, c), 3.0)), 1.0 / 8.0);
        Self {
            force: middle(a.force, b.force, c.force, d.force),
            moment: middle(a.moment, b.moment, c.moment, d.moment),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn баланс_отрезка_сходится_с_квадратурой_по_объёму() {
        // Параметры синтетического прямоугольного блока и наклонного шарнира.
        let lengths = [2.0, 0.6, 0.4];
        let anchor = [-0.3, 0.1, -0.1];
        let direction = [2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0];
        let motion = Motion {
            speed: 7.0,
            acceleration: -3.0,
            gravity: [1.0, -2.0, -9.0],
            mass: 6.0,
        };
        let omega = scale(direction, motion.speed);
        let alpha = scale(direction, motion.acceleration);
        let spans = overhangs(
            0,
            lengths[0],
            anchor,
            lengths.map(|l| l * l / 12.0),
            direction,
            motion,
            false,
            &[],
        );
        for (sign, controls) in [-1.0, 1.0].into_iter().zip(spans) {
            for (fraction, actual) in [(0.5, Wrench::midpoint(controls)), (1.0, controls[3])] {
                let tip = sign * lengths[0] / 2.0;
                let cut = tip + fraction * (anchor[0] - tip);
                let half_depth = (tip - cut).abs() / 2.0;
                let center = [(tip + cut) / 2.0, 0.0, 0.0];
                // Два узла Гаусса ±1/√3 на каждой оси точно интегрируют
                // полиномы до кубической степени, включая r×a(r).
                let nodes = [-1.0 / 3.0_f64.sqrt(), 1.0 / 3.0_f64.sqrt()];
                let node_mass = motion.mass * (2.0 * half_depth / lengths[0]) / 8.0;
                let mut force = [0.0; 3];
                let mut moment = [0.0; 3];
                for x in nodes {
                    for y in nodes {
                        for z in nodes {
                            let point = add(
                                center,
                                [half_depth * x, lengths[1] * y / 2.0, lengths[2] * z / 2.0],
                            );
                            let r = sub(point, anchor);
                            let acceleration = sub(
                                add(cross(alpha, r), cross(omega, cross(omega, r))),
                                motion.gravity,
                            );
                            let df = scale(acceleration, node_mass);
                            force = add(force, df);
                            moment = add(moment, cross(sub(point, [cut, 0.0, 0.0]), df));
                        }
                    }
                }
                let tolerance = f64::EPSILON.sqrt();
                assert!(norm(sub(actual.force, force)) <= tolerance * norm(force));
                assert!(norm(sub(actual.moment, moment)) <= tolerance * norm(moment));
            }
        }
    }
}
