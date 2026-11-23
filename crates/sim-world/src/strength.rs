use sim_materials::Section;

use crate::{Shape, physics};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionAxis {
    X,
    Y,
    Z,
}

#[derive(Clone, Debug, PartialEq)]
pub enum StrengthSection {
    Applicable { section: Section, length_m: f64 },
    NotAnalyzed { reason: String },
}

/// Определяет поперечное сечение вдоль заданной локальной оси. Проверка прочности
/// требует материала, нагрузки, закрепления и проверки применимости стержневой модели.
pub fn strength_section(shape: &Shape, axis: SectionAxis) -> Result<StrengthSection, String> {
    let (section, length_m) = match shape {
        Shape::Box { size_m: [x, y, z] } => {
            physics::positive(&[*x, *y, *z], "size_m")?;
            let (b, h, l) = match axis {
                SectionAxis::X => (*y, *z, *x),
                SectionAxis::Y => (*z, *x, *y),
                SectionAxis::Z => (*x, *y, *z),
            };
            (Section::Rect { b, h }, l)
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
