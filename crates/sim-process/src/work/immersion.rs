//! Погружение прямоугольного объёма в жидкость с плоской границей.

use sim_env::air::T0;
use sim_radio::dielectric::Medium;
use sim_signal::Rng;
use sim_world::Provenance;

use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange, StepModel};

use super::math::*;

#[derive(Clone, Debug, PartialEq)]
pub struct ImmersionRegion {
    pub surface_point_m: [SourcedRange; 3],
    /// Нормаль из жидкости, в системе мира. Направление нормируется.
    pub outward_normal: [SourcedRange; 3],
    pub density_kg_m3: SourcedRange,
    pub viscosity_pa_s: SourcedRange,
    pub flow_m_s: [SourcedRange; 3],
    pub temperature_k: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct DragSpec {
    pub coefficient: SourcedRange,
    pub reference_length_m: SourcedRange,
    pub calibrated_reynolds: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct WetHeatSpec {
    pub coefficient_w_m2_k: SourcedRange,
    pub calibrated_speed_m_s: SourcedRange,
    pub calibrated_area_m2: SourcedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioWater {
    Fresh,
    SeaAtCatalogSalinity,
}

#[derive(Clone, Debug)]
pub struct RadioFluid {
    pub water: RadioWater,
    pub composition_source: Provenance,
    pub calibrated_temperature_k: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct ImmersionSpec {
    pub body: String,
    pub half_extent_m: [SourcedRange; 3],
    pub region: ImmersionRegion,
    pub drag: Option<DragSpec>,
    pub heat: Option<WetHeatSpec>,
    pub radio: Option<RadioFluid>,
}

#[derive(Clone, Copy, Debug)]
pub struct ImmersionInput {
    pub box_center_m: [f64; 3],
    /// Правый ортонормированный базис из трёх осей блока в системе мира.
    pub box_axes: [[f64; 3]; 3],
    pub body_com_m: [f64; 3],
    pub velocity_at_com_m_s: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
    pub gravity_m_s2: [f64; 3],
    /// Фактические параметры встроенной среды из скомпилированной модели MuJoCo.
    /// Глобальное сопротивление нужно отключить до применения локальной модели.
    pub engine_density_kg_m3: f64,
    pub engine_viscosity_pa_s: f64,
    pub housing_temperature_k: Option<f64>,
    pub antenna_m: Option<[f64; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ThermalBoundary {
    Dry,
    Wet {
        /// Температура резервуара, а не корпуса.
        fluid_temperature_k: f64,
        /// Добавочный проводящий путь от смоченной поверхности к жидкости.
        conductance_w_k: f64,
        /// Положительный поток направлен от корпуса в жидкость.
        heat_out_w: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadioEnvironment {
    pub medium: Medium,
    /// Глубина антенны по нормали, для радиолинии по нормали к границе.
    pub depth_m: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ImmersionStep {
    pub volume_m3: f64,
    pub center_of_buoyancy_m: Option<[f64; 3]>,
    pub wetted_area_m2: f64,
    pub buoyancy: BodyLoad,
    pub drag: Analysis<BodyLoad>,
    /// Сумма для применения рантаймом. При неизвестном сопротивлении эффект неполон.
    pub body_load: Analysis<BodyLoad>,
    pub drive_thermal: Analysis<ThermalBoundary>,
    pub radio: Analysis<RadioEnvironment>,
}

impl StepModel for ImmersionSpec {
    type Input = ImmersionInput;
    type Output = ImmersionStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        duration(interval)?;
        self.evaluate(input)
    }
}

impl ImmersionSpec {
    pub fn evaluate(&self, input: &ImmersionInput) -> Result<ImmersionStep, ProcessError> {
        name(&self.body)?;
        let extents = point(&self.half_extent_m)?;
        if self
            .half_extent_m
            .iter()
            .any(|dimension| dimension.min <= 0.0)
        {
            return Err(invalid("полуразмер блока должен быть положительным"));
        }
        let surface = point(&self.region.surface_point_m)?;
        let normal = direction(point(&self.region.outward_normal)?)?;
        let density = range(
            &self.region.density_kg_m3,
            "плотность жидкости",
            |x| x > 0.0,
        )?;
        let viscosity = range(
            &self.region.viscosity_pa_s,
            "вязкость жидкости",
            |x| x > 0.0,
        )?;
        let flow = point(&self.region.flow_m_s)?;
        let fluid_k = range(
            &self.region.temperature_k,
            "температура жидкости",
            |x| x > 0.0,
        )?;
        self.validate_optional()?;
        for vector in [
            input.box_center_m,
            input.body_com_m,
            input.velocity_at_com_m_s,
            input.angular_velocity_rad_s,
            input.gravity_m_s2,
        ] {
            finite(&vector)?;
        }
        finite(&[input.engine_density_kg_m3, input.engine_viscosity_pa_s])?;
        if input.engine_density_kg_m3 != 0.0 || input.engine_viscosity_pa_s != 0.0 {
            return Err(invalid(
                "локальная гидродинамика несовместима с глобальным сопротивлением MuJoCo",
            ));
        }
        if let Some(temperature) = input.housing_temperature_k {
            finite(&[temperature])?;
            if temperature <= 0.0 {
                return Err(invalid("температура корпуса должна быть положительной"));
            }
        }
        if let Some(p) = input.antenna_m {
            finite(&p)?;
        }
        let axes = input.box_axes;
        for axis in axes {
            finite(&axis)?;
        }
        // Допуск представленности ортонормированного базиса в f64, без физического допуска.
        let roundoff = f64::EPSILON.sqrt();
        if axes.iter().any(|&v| (norm(v) - 1.0).abs() > roundoff)
            || norm(sub(cross(axes[0], axes[1]), axes[2])) > roundoff
            || dot(axes[0], axes[1]).abs() > roundoff
        {
            return Err(invalid(
                "оси блока не образуют правый ортонормированный базис",
            ));
        }
        if norm(sub(direction(input.gravity_m_s2)?, scale(normal, -1.0))) > roundoff {
            return Err(invalid(
                "свободная поверхность не перпендикулярна гравитации",
            ));
        }
        let center_depth = dot(sub(surface, input.box_center_m), normal);
        computed(&[center_depth])?;
        let solid = clip_box(extents, axes, normal, center_depth)?;
        let radio = self.radio_environment(input.antenna_m, normal, surface, fluid_k)?;
        let load = |force_n: Vec3, arm: Vec3| {
            let torque_nm = cross(arm, force_n);
            computed(&force_n)?;
            computed(&torque_nm)?;
            Ok(BodyLoad {
                body: self.body.clone(),
                force_n,
                torque_nm,
            })
        };
        let Some(solid) = solid else {
            let zero = load([0.0; 3], [0.0; 3])?;
            let dry = "геометрия вне жидкости";
            return Ok(ImmersionStep {
                volume_m3: 0.0,
                center_of_buoyancy_m: None,
                wetted_area_m2: 0.0,
                buoyancy: zero.clone(),
                drag: calculated(zero.clone(), dry),
                body_load: calculated(zero, dry),
                drive_thermal: calculated(ThermalBoundary::Dry, dry),
                radio,
            });
        };
        let center = add(input.box_center_m, solid.center);
        let arm = add(sub(input.box_center_m, input.body_com_m), solid.center);
        computed(&center)?;
        computed(&arm)?;
        // Архимед: вытесненная масса жидкости, умноженная на вектор против гравитации.
        let buoyancy = load(scale(input.gravity_m_s2, -density * solid.volume), arm)?;
        let relative = sub(
            add(
                input.velocity_at_com_m_s,
                cross(input.angular_velocity_rad_s, arm),
            ),
            flow,
        );
        computed(&relative)?;
        let speed = norm(relative);
        computed(&[speed])?;
        let drag = if speed == 0.0 && norm(input.angular_velocity_rad_s) == 0.0 {
            calculated(load([0.0; 3], arm)?, "нет относительного движения")
        } else if let Some(spec) = &self.drag
            && speed > 0.0
        {
            let reynolds = density * speed * spec.reference_length_m.nominal / viscosity;
            computed(&[reynolds])?;
            if in_calibration(reynolds, &spec.calibrated_reynolds) {
                let direction = relative.map(|x| x / speed);
                // Для замкнутого выпуклого объёма сумма модулей проекций граней
                // равна удвоенной площади силуэта. Срез замыкает вытесненный объём.
                let projected_area = solid
                    .face_areas
                    .iter()
                    .map(|&a| dot(a, direction).abs())
                    .sum::<f64>()
                    / 2.0;
                let factor = -0.5 * density * spec.coefficient.nominal * projected_area * speed;
                computed(&[projected_area, factor])?;
                calculated(
                    load(scale(relative, factor), arm)?,
                    "квадратичное сопротивление по силуэту, приложено в центре объёма, вращательное сопротивление отдельно не учтён",
                )
            } else {
                unknown("число Рейнольдса вне калибровки сопротивления")
            }
        } else {
            unknown("нет применимой модели сопротивления для движения тела")
        };
        let body_load = match &drag {
            Analysis::Analyzed { value, .. } => calculated(
                load(add(buoyancy.force_n, value.force_n), arm)?,
                "плавучесть и сопротивление по номиналам, сила в центре вытесненного объёма",
            ),
            Analysis::NotAnalyzed { reason } => unknown(reason),
        };
        let drive_thermal = if let (Some(heat), Some(housing_k)) =
            (&self.heat, input.housing_temperature_k)
            && in_calibration(speed, &heat.calibrated_speed_m_s)
            && in_calibration(solid.wetted_area, &heat.calibrated_area_m2)
        {
            // Закон Ньютона для конвективного потока: Q̇ = h A (T_корпуса − T_жидкости).
            let conductance_w_k = heat.coefficient_w_m2_k.nominal * solid.wetted_area;
            let heat_out_w = conductance_w_k * (housing_k - fluid_k);
            computed(&[conductance_w_k, heat_out_w])?;
            if conductance_w_k <= 0.0 {
                return Err(execution("тепловая проводимость жидкости не представима"));
            }
            calculated(
                ThermalBoundary::Wet {
                    fluid_temperature_k: fluid_k,
                    conductance_w_k,
                    heat_out_w,
                },
                "номинальная конвекция по фактической мокрой площади, смоченная сторона корпуса",
            )
        } else {
            unknown("нет температуры корпуса или применимой калибровки теплообмена")
        };
        Ok(ImmersionStep {
            volume_m3: solid.volume,
            center_of_buoyancy_m: Some(center),
            wetted_area_m2: solid.wetted_area,
            buoyancy,
            drag,
            body_load,
            drive_thermal,
            radio,
        })
    }

    fn validate_optional(&self) -> Result<(), ProcessError> {
        if let Some(drag) = &self.drag {
            range(&drag.coefficient, "коэффициент сопротивления", |x| x > 0.0)?;
            range(
                &drag.reference_length_m,
                "характерный размер для Reynolds",
                |x| x > 0.0,
            )?;
            range(&drag.calibrated_reynolds, "диапазон Reynolds", |x| x >= 0.0)?;
        }
        if let Some(heat) = &self.heat {
            range(
                &heat.coefficient_w_m2_k,
                "коэффициент конвекции",
                |x| x > 0.0,
            )?;
            range(
                &heat.calibrated_speed_m_s,
                "допустимая скорость для конвекции",
                |x| x >= 0.0,
            )?;
            range(
                &heat.calibrated_area_m2,
                "допустимая мокрая площадь",
                |x| x > 0.0,
            )?;
        }
        if let Some(radio) = &self.radio {
            source(&radio.composition_source)?;
            range(
                &radio.calibrated_temperature_k,
                "температурный диапазон радио",
                |x| x > 0.0,
            )?;
        }
        Ok(())
    }

    fn radio_environment(
        &self,
        antenna: Option<Vec3>,
        normal: Vec3,
        surface: Vec3,
        liquid_k: f64,
    ) -> Result<Analysis<RadioEnvironment>, ProcessError> {
        let Some(antenna) = antenna else {
            return Ok(unknown("не задано положение антенны"));
        };
        let depth_m = dot(sub(surface, antenna), normal);
        computed(&[depth_m])?;
        if depth_m <= 0.0 {
            return Ok(calculated(
                RadioEnvironment {
                    medium: Medium::Air,
                    depth_m: 0.0,
                },
                "антенна вне жидкости; среда в точке антенны",
            ));
        }
        let Some(radio) = &self.radio else {
            return Ok(unknown("нет модели диэлектрических свойств жидкости"));
        };
        if !in_calibration(liquid_k, &radio.calibrated_temperature_k) {
            return Ok(unknown("температура жидкости вне диапазона модели радио"));
        }
        let t_c = liquid_k - T0;
        let medium = match radio.water {
            RadioWater::Fresh => Medium::FreshWater { t_c },
            RadioWater::SeaAtCatalogSalinity => Medium::SeaWater { t_c },
        };
        Ok(calculated(
            RadioEnvironment { medium, depth_m },
            "модель воды sim-radio, глубина по нормали, потери и геометрия линии считаются отдельно",
        ))
    }
}

fn in_calibration(value: f64, range: &SourcedRange) -> bool {
    (range.min..=range.max).contains(&value)
}

struct Submerged {
    volume: f64,
    center: Vec3,
    wetted_area: f64,
    face_areas: Vec<Vec3>,
}

fn clip_box(
    extents: Vec3,
    axes: [Vec3; 3],
    normal: Vec3,
    center_depth: f64,
) -> Result<Option<Submerged>, ProcessError> {
    // Биты индекса выбирают знак вдоль каждой оси. Грани перечислены против
    // часовой стрелки при взгляде снаружи соответствующей грани.
    const FACES: [[usize; 4]; 6] = [
        [0, 4, 6, 2],
        [1, 3, 7, 5],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 2, 3, 1],
        [4, 5, 7, 6],
    ];
    let vertices: [Vec3; 8] = std::array::from_fn(|i| {
        axes.into_iter()
            .enumerate()
            .fold([0.0; 3], |p, (axis, basis)| {
                add(
                    p,
                    scale(
                        basis,
                        if i & (1 << axis) == 0 {
                            -extents[axis]
                        } else {
                            extents[axis]
                        },
                    ),
                )
            })
    });
    let depths = vertices.map(|p| center_depth - dot(p, normal));
    computed(&depths)?;
    if depths.iter().all(|&d| d <= 0.0) {
        return Ok(None);
    }
    let mut cap = Vec::new();
    let mut faces = Vec::new();
    for face in FACES {
        let mut polygon = Vec::new();
        for (&a, &b) in face.iter().zip(face.iter().cycle().skip(1)) {
            if (depths[a] >= 0.0) != (depths[b] >= 0.0) {
                // Одинаковый порядок концов у двух граней даёт побитово одну вершину.
                let low = a.min(b);
                let high = a.max(b);
                let span = depths[low] - depths[high];
                computed(&[span])?;
                let fraction = depths[low] / span;
                let p = if fraction == 0.0 {
                    vertices[low]
                } else if fraction == 1.0 {
                    vertices[high]
                } else {
                    add(
                        vertices[low],
                        scale(sub(vertices[high], vertices[low]), fraction),
                    )
                };
                computed(&p)?;
                polygon.push(p);
                if !cap.contains(&p) {
                    cap.push(p);
                }
            }
            if depths[b] >= 0.0 {
                polygon.push(vertices[b]);
            }
        }
        if polygon.len() >= 3 {
            faces.push((!face.iter().all(|&i| depths[i] == 0.0), polygon));
        }
    }
    if cap.len() >= 3 {
        let center = mean(cap.iter().copied(), cap.len());
        computed(&center)?;
        let least = (0..3)
            .min_by(|&i, &j| normal[i].abs().total_cmp(&normal[j].abs()))
            .unwrap();
        let axis = std::array::from_fn(|i| f64::from(i == least));
        let u = direction(cross(axis, normal))?;
        let v = cross(normal, u);
        let angle = |p: &Vec3| {
            let offset = sub(*p, center);
            dot(offset, v).atan2(dot(offset, u))
        };
        cap.sort_by(|a, b| angle(a).total_cmp(&angle(b)));
        faces.push((false, cap));
    }
    let count = faces.iter().map(|(_, p)| p.len()).sum();
    if count == 0 {
        return Err(execution("смоченный объём вырожден в пределах точности"));
    }
    let center = mean(faces.iter().flat_map(|(_, p)| p.iter().copied()), count);
    computed(&center)?;
    let mut volume = 0.0;
    let mut centroid_offset = [0.0; 3];
    let mut wetted_area = 0.0;
    let mut face_areas = Vec::new();
    for (wet, polygon) in faces {
        let mut area = [0.0; 3];
        let a = polygon[0];
        for pair in polygon[1..].windows(2) {
            let b = pair[0];
            let c = pair[1];
            let twice_area = cross(sub(b, a), sub(c, a));
            area = add(area, scale(twice_area, 0.5));
            // Разбиваем выпуклый объём на тетраэдры от внутренней точки.
            // Объём = |смешанное произведение| / 6, центр = среднее четырёх вершин.
            let offset = sub(a, center);
            let weight = dot(offset, twice_area).abs() / 6.0;
            let tetra_center = scale(add(add(offset, sub(b, center)), sub(c, center)), 0.25);
            volume += weight;
            centroid_offset = add(centroid_offset, scale(tetra_center, weight));
        }
        computed(&area)?;
        if wet {
            wetted_area += norm(area);
        }
        face_areas.push(area);
    }
    computed(&[volume, wetted_area])?;
    if volume <= 0.0 || wetted_area <= 0.0 {
        return Err(execution(
            "смоченный объём или площадь вырождены в пределах точности",
        ));
    }
    let center = add(center, centroid_offset.map(|x| x / volume));
    computed(&center)?;
    Ok(Some(Submerged {
        volume,
        center,
        wetted_area,
        face_areas,
    }))
}

fn mean(points: impl Iterator<Item = Vec3>, count: usize) -> Vec3 {
    points.fold([0.0; 3], |sum, p| add(sum, scale(p, 1.0 / count as f64)))
}
