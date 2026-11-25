use std::path::Path;

use nalgebra::{Matrix3, UnitQuaternion};
use sim_materials::Section;

use crate::{Geometry, Material, Shape, physics};

pub const STRENGTH_REQUIRES_BENCH: &str =
    "расчётная прочность не закрывает опасность без стендового замера";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentrifugalStress {
    /// Среднее нормальное напряжение на срезе либо по объёму сетки, Па.
    pub normal_pa: f64,
    /// Пики радиального и окружного напряжений дисковой модели при справочном ν, Па.
    pub disk_reference_pa: Option<[f64; 2]>,
    /// Верхняя огибающая пиков дисковой модели по допустимому ν, Па.
    pub disk_envelope_pa: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct CentrifugalModel {
    rotation: UnitQuaternion<f64>,
    density: f64,
    normal: NormalStress,
    disk_radius: Option<f64>,
}

#[derive(Clone, Debug)]
enum NormalStress {
    Section([f64; 3]),
    Volume(Matrix3<f64>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionAxis {
    X,
    Y,
    Z,
}

impl SectionAxis {
    fn direction(self) -> physics::Vec3 {
        match self {
            Self::X => physics::Vec3::x(),
            Self::Y => physics::Vec3::y(),
            Self::Z => physics::Vec3::z(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum StrengthSection {
    Applicable { section: Section, length_m: f64 },
    NotAnalyzed { reason: String },
}

impl Geometry {
    /// Подготавливает отсев по центробежной нагрузке однородной формы.
    /// `asset_base` — каталог файла, задающего форму. Без плотности или
    /// замкнутого объёма результат отсутствует.
    pub fn centrifugal_model(
        &self,
        material: Option<&Material>,
        asset_base: &Path,
    ) -> Result<Option<CentrifugalModel>, String> {
        use std::f64::consts::PI;

        let frame = physics::frame(&self.pose)?;
        let material_density = self.material.as_ref().or(material).map(|m| m.density_kg_m3);
        if let Some(density) = material_density {
            physics::positive(&[density], "плотность")?;
        }
        if let Some(mass) = &self.explicit_mass {
            physics::nonnegative(&[mass.mass_kg], "масса объёма")?;
        }
        // Коэффициент S/A, где S — первый момент половины объёма,
        // A — площадь центрального среза. Из баланса сил среднее
        // нормальное напряжение равно ρ·ω_поперечная²·S/A.
        let section = |arms: [f64; 3], volume| -> Result<_, String> {
            physics::positive(&arms, "геометрические коэффициенты центробежной нагрузки")?;
            Ok((NormalStress::Section(arms), volume))
        };
        let (normal, volume) = match &self.shape {
            Shape::Box { size_m } => {
                physics::positive(size_m, "size_m")?;
                section(size_m.map(|d| d.powi(2) / 8.0), size_m.iter().product())?
            }
            Shape::Sphere { radius_m } => {
                physics::positive(&[*radius_m], "radius_m")?;
                section([radius_m.powi(2) / 4.0; 3], 4.0 / 3.0 * PI * radius_m.powi(3))?
            }
            Shape::Ellipsoid { radii_m } => {
                physics::positive(radii_m, "radii_m")?;
                section(radii_m.map(|r| r.powi(2) / 4.0), 4.0 / 3.0 * PI * radii_m.iter().product::<f64>())?
            }
            Shape::Cylinder { radius_m: r, length_m: l } => {
                physics::positive(&[*r, *l], "radius_m/length_m")?;
                section([r.powi(2) / 3.0, r.powi(2) / 3.0, l.powi(2) / 8.0], PI * r.powi(2) * l)?
            }
            Shape::Capsule { radius_m: r, length_m: l } => {
                physics::positive(&[*r, *l], "radius_m/length_m")?;
                // Продольный срез: прямоугольник плюс круг. Для нормали Z:
                // полцилиндра и полусфера с центром масс в 3r/8 от торца.
                let radial = (2.0 * r.powi(3) * l / 3.0 + PI * r.powi(4) / 4.0)
                    / (2.0 * r * l + PI * r.powi(2));
                let axial = l.powi(2) / 8.0 + 2.0 * r / 3.0 * (l / 2.0 + 3.0 * r / 8.0);
                section([radial, radial, axial], PI * r.powi(2) * (l + 4.0 / 3.0 * r))?
            }
            Shape::Mesh { .. } | Shape::Triangles { .. } => {
                let geometry = physics::Geom::new(self, asset_base)?;
                let mesh = geometry.mesh.as_ref().expect("сетка загружена");
                if mesh.require_closed().is_err() {
                    return Ok(None);
                }
                // Плотность 1 для геометрического тензора: масса численно
                // равна объёму. Из I = V·(tr(C)·1 − C) восстанавливаем
                // ковариацию координат C относительно центра объёма.
                let inertia = physics::mesh_inertia(mesh, 1.0)?;
                let tensor = inertia.tensor / inertia.kg;
                let covariance = Matrix3::identity() * (tensor.trace() / 2.0) - tensor;
                physics::finite(covariance.as_slice(), "второй момент объёма сетки")?;
                (NormalStress::Volume(covariance), inertia.kg)
            }
        };
        physics::positive(&[volume], "объём формы")?;
        let density = match &self.explicit_mass {
            Some(mass) => {
                if mass.mass_kg == 0.0 {
                    return Ok(None);
                }
                mass.mass_kg / volume
            }
            None => match material_density {
                Some(density) => density,
                None => return Ok(None),
            },
        };
        physics::positive(&[density], "плотность")?;
        let disk_radius = match self.shape {
            Shape::Cylinder { radius_m, .. } => Some(radius_m),
            _ => None,
        };
        Ok(Some(CentrifugalModel { rotation: frame.rotation, density, normal, disk_radius }))
    }

    /// Единичное направление нагрузки в локальном кадре формы. На входе
    /// направление задано в кадре звена, как и ось сочленения.
    pub fn local_load_direction(&self, direction: [f64; 3]) -> Result<[f64; 3], String> {
        let frame = physics::frame(&self.pose)?;
        let direction = physics::axis(direction)?;
        Ok(frame.rotation.inverse_transform_vector(&direction).into())
    }

    /// Огибающая плеча для постоянного сечения. Точки нагрузки заданы в кадре
    /// звена. Крайние сечения определяются длиной, позой и локальной осью формы.
    pub fn section_load_arm(
        &self,
        axis: SectionAxis,
        load_points_m: &[[f64; 3]],
    ) -> Result<Option<f64>, String> {
        let frame = physics::frame(&self.pose)?;
        for point in load_points_m {
            physics::finite(point, "точка приложения нагрузки")?;
        }
        let StrengthSection::Applicable { length_m, .. } = strength_section(&self.shape, axis)?
        else {
            return Ok(None);
        };
        let end = axis.direction() * (length_m / 2.0);
        let ends = [-end, end].map(|point| frame.transform_point(&point.into()));
        // При неизвестной точке на стержне максимальное расстояние вдоль
        // стержня равно его длине. Известные точки вне стержня расширяют оценку.
        let mut arm = length_m;
        for &point in load_points_m {
            for end in ends {
                let delta = end.coords - physics::Vec3::from(point);
                arm = arm.max(delta.x.hypot(delta.y).hypot(delta.z));
            }
        }
        physics::finite(&[arm], "плечо сечения")?;
        Ok(Some(arm))
    }

    /// Верхняя граница эксцентриситета силы относительно оси сечения.
    /// Охватывает весь профиль и точки нагрузки в кадре звена.
    pub fn section_radial_arm(
        &self,
        axis: SectionAxis,
        load_points_m: &[[f64; 3]],
    ) -> Result<Option<f64>, String> {
        let frame = physics::frame(&self.pose)?;
        for point in load_points_m {
            physics::finite(point, "точка приложения нагрузки")?;
        }
        let StrengthSection::Applicable { section, .. } = strength_section(&self.shape, axis)?
        else {
            return Ok(None);
        };
        // Наиболее удалённая точка прямоугольника находится на полудиагонали.
        let mut arm = match section {
            Section::Rect { b, h } => (b / 2.0).hypot(h / 2.0),
            Section::Round { d } => d / 2.0,
            Section::Tube { od, .. } => od / 2.0,
        };
        for &point in load_points_m {
            let local = frame.inverse_transform_point(&physics::Vec3::from(point).into());
            let delta = axis.direction().cross(&local.coords);
            let radius = delta.x.hypot(delta.y).hypot(delta.z);
            physics::finite(&[radius], "эксцентриситет силы")?;
            arm = arm.max(radius);
        }
        physics::finite(&[arm], "эксцентриситет силы")?;
        Ok(Some(arm))
    }
}

impl CentrifugalModel {
    /// Угловая скорость в кадре звена, рад/с.
    pub fn stress(&self, angular_velocity_rad_s: [f64; 3]) -> Result<CentrifugalStress, String> {
        physics::finite(&angular_velocity_rad_s, "угловая скорость")?;
        let omega = self.rotation.inverse_transform_vector(&angular_velocity_rad_s.into());
        physics::finite(omega.as_slice(), "угловая скорость в кадре формы")?;
        let normal_pa = match &self.normal {
            NormalStress::Section(arms) => {
                let mut peak = 0.0_f64;
                for (i, arm) in arms.iter().enumerate() {
                    let transverse = omega[(i + 1) % 3].hypot(omega[(i + 2) % 3]);
                    let stress = self.density * (transverse * arm.sqrt()).powi(2);
                    physics::finite(&[stress], "центробежное растяжение сечения")?;
                    peak = peak.max(stress);
                }
                peak
            }
            NormalStress::Volume(covariance) => {
                let speed = omega.x.hypot(omega.y).hypot(omega.z);
                if speed == 0.0 {
                    return Ok(CentrifugalStress { normal_pa: 0.0, disk_reference_pa: None, disk_envelope_pa: None });
                }
                physics::finite(&[speed], "модуль угловой скорости")?;
                let axis = omega / speed;
                // Интегрируем уравнение равновесия свободного объёма
                // с весом n·x. Для n ⟂ ω получаем среднее σ_nn =
                // ρ|ω|² E[(n·x)²]. Среднее по двум ортогональным n
                // ограничивает снизу максимум по поперечным направлениям.
                let variance = (covariance.trace() - axis.dot(&(covariance * axis))) / 2.0;
                physics::positive(&[variance], "поперечный второй момент объёма сетки")?;
                self.density * (speed * variance.sqrt()).powi(2)
            }
        };
        physics::finite(&[normal_pa], "центробежное напряжение")?;
        let (disk_reference_pa, disk_envelope_pa) = if let Some(radius) = self.disk_radius.filter(|_| omega.z != 0.0) {
            // NACA TN 2217, ур. (39): максимум σᵣ в центре;
            // максимум σθ в центре либо на краю. Справочное ν = 0,3
            // взято из постановки NACA TN 1667, не из материала звена.
            // Без измеренного ν охватываем пределы -1 и 1/2 из
            // положительности объёмного модуля и модуля сдвига.
            let scale = self.density * (omega.z * radius).powi(2);
            let peaks = |nu: f64| {
                let center = scale * ((3.0 + nu) / 8.0);
                let rim = scale * ((1.0 - nu) / 4.0);
                [center, center.max(rim)]
            };
            let reference = peaks(0.3);
            let envelope = peaks(-1.0)[1].max(peaks(0.5)[1]);
            physics::finite(&[reference[0], reference[1], envelope], "напряжение диска")?;
            (Some(reference), Some(envelope))
        } else {
            (None, None)
        };
        Ok(CentrifugalStress { normal_pa, disk_reference_pa, disk_envelope_pa })
    }
}

/// Поперечное сечение вдоль локальной оси. Для прямоугольника возвращает
/// ориентацию с наименьшим моментом сопротивления изгибу.
pub fn strength_section(shape: &Shape, axis: SectionAxis) -> Result<StrengthSection, String> {
    let (section, length_m) = match shape {
        Shape::Box { size_m: [x, y, z] } => {
            physics::positive(&[*x, *y, *z], "size_m")?;
            let (b, h, l) = match axis {
                SectionAxis::X => (*y, *z, *x),
                SectionAxis::Y => (*z, *x, *y),
                SectionAxis::Z => (*x, *y, *z),
            };
            // W = b·h²/6, поэтому при перестановке сторон минимум даёт меньшее h.
            (Section::Rect { b: b.max(h), h: b.min(h) }, l)
        }
        Shape::Cylinder { radius_m, length_m } if axis == SectionAxis::Z => {
            physics::positive(&[*radius_m, *length_m], "radius_m/length_m")?;
            (Section::Round { d: 2.0 * radius_m }, *length_m)
        }
        shape => {
            let reason = match shape {
                Shape::Mesh { scale, .. } => {
                    physics::positive(scale, "scale")?;
                    "сетка требует отдельной проверки сечения или расчёта напряжений"
                }
                Shape::Triangles { vertices_m, .. } => {
                    for v in vertices_m {
                        physics::finite(v, "vertices_m")?;
                    }
                    "сетка требует отдельной проверки сечения или расчёта напряжений"
                }
                Shape::Sphere { radius_m } => {
                    physics::positive(&[*radius_m], "radius_m")?;
                    "сечение меняется вдоль оси"
                }
                Shape::Ellipsoid { radii_m } => {
                    physics::positive(radii_m, "radii_m")?;
                    "сечение меняется вдоль оси"
                }
                Shape::Capsule { radius_m, length_m } | Shape::Cylinder { radius_m, length_m } => {
                    physics::positive(&[*radius_m, *length_m], "radius_m/length_m")?;
                    "сечение меняется вдоль оси"
                }
                Shape::Box { .. } => unreachable!(),
            };
            return Ok(StrengthSection::NotAnalyzed {
                reason: reason.into(),
            });
        }
    };
    section.validate()?;
    Ok(StrengthSection::Applicable { section, length_m })
}
