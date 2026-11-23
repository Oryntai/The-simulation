//! Прочность напечатанной детали: толщина слоя, стенки, заполнение, сопло, направление
//! нагрузки относительно слоёв, температура.
//!
//! Паспортные значения меряются на образцах со 100% заполнением. Модель только снижает их:
//!
//! 1. **Межслойная связь.** Паспортный образец «стоя» — маленький, напечатан на подобранном
//!    режиме, часто отожжён. В настоящей детали есть шов, перерывы между слоями и перепад
//!    температуры, поэтому прочность поперёк слоёв дополнительно умножается на
//!    [`INTERLAYER_BOND_FACTOR`]. Ahn et al. (2002, Rapid Prototyping J. 8(4), 248–257,
//!    DOI 10.1108/13552540210441166): 65–72% от литьевого ABS при лучшем режиме вдоль нитей и
//!    существенно меньше поперёк.
//! 2. **Направление.** От 0° (вдоль слоёв) прочность линейно падает и уже при
//!    [`OFF_AXIS_FULL_DROP_DEG`] считается как поперёк: косая нагрузка раскрывает слой сдвигом.
//! 3. **Заполнение.** Стенки считаются сплошными, сердцевина — ячеистой (закон Гибсона–Эшби).
//! 4. **Высота слоя.** Выше половины сопла валики хуже сплавляются.
//! 5. **Температура** — [`crate::catalog::temperature_factor`].

use crate::catalog::{Material, temperature_factor};
use crate::section::Section;

/// Коэффициент межслойной связи реальной детали относительно паспортного образца.
/// **Оценка в опасную сторону.** Ориентиры: ZX/XZ у Stratasys ABS-M30 F770 = 23.1/32.5 = 0.71;
/// старые паспорта PolyLite PETG V5: Z/XY = 13.4/31.9 = 0.42.
pub const INTERLAYER_BOND_FACTOR: f64 = 0.7;

/// Угол между нагрузкой и плоскостью слоёв, начиная с которого деталь считается как нагруженная
/// поперёк слоёв, градусы. **Оценка в опасную сторону.**
pub const OFF_AXIS_FULL_DROP_DEG: f64 = 30.0;

/// Отношение слой/сопло, при котором меряются паспорта (0.2 мм на сопле 0.4 мм).
pub const REFERENCE_LAYER_RATIO: f64 = 0.5;

/// Наибольшее отношение слой/сопло, при котором валик ещё надёжно прижимается
/// (распространённое правило «слой ≤ 75–80% сопла»); выше — блокер в проверке печати.
pub const MAX_LAYER_RATIO: f64 = 0.8;

/// Коэффициент перехода от прочности при растяжении к прочности на сдвиг: 1/√3 (Мизес).
pub const SHEAR_FROM_TENSION: f64 = 0.577;

/// Параметры печати, влияющие на прочность.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintProcess {
    /// Диаметр сопла, м.
    pub nozzle: f64,
    /// Высота слоя, м.
    pub layer: f64,
    /// Число стенок (периметров).
    pub walls: u32,
    /// Заполнение, доля 0..1.
    pub infill: f64,
    /// Угол между нагрузкой и плоскостью слоёв, градусы: 0 — вдоль слоёв, 90 — поперёк.
    pub load_angle_deg: f64,
}

impl Default for PrintProcess {
    /// Типичная печать соплом 0.4. Направление по умолчанию — **поперёк слоёв**: пока
    /// ориентация не задана, считаем худший случай.
    fn default() -> Self {
        Self {
            nozzle: 0.4e-3,
            layer: 0.2e-3,
            walls: 4,
            infill: 0.4,
            load_angle_deg: 90.0,
        }
    }
}

impl PrintProcess {
    /// Проверка диапазонов.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.nozzle.is_finite() && self.nozzle > 0.0 && self.nozzle <= 5e-3) {
            return Err(format!("диаметр сопла вне диапазона: {} м", self.nozzle));
        }
        if !(self.layer.is_finite() && self.layer > 0.0 && self.layer <= 5e-3) {
            return Err(format!("высота слоя вне диапазона: {} м", self.layer));
        }
        if !(self.infill.is_finite() && (0.0..=1.0).contains(&self.infill)) {
            return Err(format!("заполнение вне диапазона 0..1: {}", self.infill));
        }
        if !(self.load_angle_deg.is_finite() && (0.0..=90.0).contains(&self.load_angle_deg)) {
            return Err(format!(
                "угол к слоям вне диапазона 0..90°: {}",
                self.load_angle_deg
            ));
        }
        Ok(())
    }

    /// Толщина стенки из периметров, м (ширина линии принята равной соплу).
    pub fn shell_thickness(&self) -> f64 {
        self.walls as f64 * self.nozzle
    }
}

/// Доля прочности ячеистой сердцевины: 0.3·ρ^1.5 при ρ ≤ 0.3 (Gibson & Ashby, «Cellular Solids»,
/// 2-е изд., 1997, гл. 5, пластическое разрушение открытых ячеек), дальше линейно до 1 при
/// ρ = 1. Линейный участок — **оценка**: закон выведен для малых плотностей.
pub fn infill_strength_fraction(infill: f64) -> f64 {
    let rho = infill.clamp(0.0, 1.0);
    const KNEE: f64 = 0.3;
    if rho <= KNEE {
        0.3 * rho.powf(1.5)
    } else {
        let at_knee = 0.3 * KNEE.powf(1.5);
        at_knee + (1.0 - at_knee) * (rho - KNEE) / (1.0 - KNEE)
    }
}

/// Доля жёсткости ячеистой сердцевины: ρ² (Gibson & Ashby, гл. 5, открытые ячейки).
pub fn infill_stiffness_fraction(infill: f64) -> f64 {
    infill.clamp(0.0, 1.0).powi(2)
}

/// Снижение прочности от высоты слоя `(вдоль, поперёк)`. До половины сопла — без снижения
/// (при таком режиме сняты паспорта), к 0.75 сопла — 0.9 вдоль и 0.8 поперёк, дальше тем же
/// наклоном, но не ниже 0.5. **Оценка в опасную сторону**: надёжной зависимости нет.
pub fn layer_height_factors(layer: f64, nozzle: f64) -> (f64, f64) {
    let ratio = layer / nozzle;
    if !ratio.is_finite() || ratio <= REFERENCE_LAYER_RATIO {
        return if ratio.is_finite() {
            (1.0, 1.0)
        } else {
            (0.5, 0.5)
        };
    }
    let x = (ratio - REFERENCE_LAYER_RATIO) / 0.25;
    ((1.0 - 0.1 * x).max(0.5), (1.0 - 0.2 * x).max(0.5))
}

/// Доли несущего материала в сечении из-за неполного заполнения.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FillFractions {
    /// Для растяжения и сдвига (по площади).
    pub area: f64,
    /// Для изгиба (по моменту инерции), прочность.
    pub bending: f64,
    /// Для изгиба и устойчивости, жёсткость.
    pub stiffness: f64,
}

/// Считает доли: сплошные стенки толщиной `shell` по контуру и сердцевина с заполнением.
/// Верх и низ (сплошные слои) не учитываются — это в безопасную сторону.
pub fn fill_fractions(section: &Section, shell: f64, infill: f64) -> FillFractions {
    let ks = infill_strength_fraction(infill);
    let ke = infill_stiffness_fraction(infill);
    // Площадь и момент инерции сердцевины.
    let (core_a, core_i) = match *section {
        Section::Rect { b, h } => {
            let (bc, hc) = (b - 2.0 * shell, h - 2.0 * shell);
            if bc > 0.0 && hc > 0.0 {
                (bc * hc, bc * hc.powi(3) / 12.0)
            } else {
                (0.0, 0.0)
            }
        }
        Section::Round { d } => {
            let dc = d - 2.0 * shell;
            if dc > 0.0 {
                let c = Section::Round { d: dc };
                (c.area(), c.inertia())
            } else {
                (0.0, 0.0)
            }
        }
        Section::Tube { od, wall } => {
            // Периметры идут и по наружной, и по внутренней поверхности.
            let core_wall = wall - 2.0 * shell;
            if core_wall > 0.0 {
                let c = Section::Tube {
                    od: od - 2.0 * shell,
                    wall: core_wall,
                };
                (c.area(), c.inertia())
            } else {
                (0.0, 0.0)
            }
        }
    };
    let (a, i) = (section.area(), section.inertia());
    FillFractions {
        area: 1.0 - (1.0 - ks) * core_a / a,
        bending: 1.0 - (1.0 - ks) * core_i / i,
        stiffness: 1.0 - (1.0 - ke) * core_i / i,
    }
}

/// Эффективные свойства напечатанной детали при заданной температуре и направлении нагрузки.
/// Напряжения от номинального сечения делятся на доли [`FillFractions`] вызывающей стороной.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintedStrength {
    /// Расчётный предел текучести в направлении нагрузки, Па.
    pub yield_strength: f64,
    /// Расчётный предел прочности в направлении нагрузки, Па.
    pub ultimate: f64,
    /// Расчётная прочность поперёк слоёв (с межслойной связью), Па.
    pub across: f64,
    /// Прочность межслойного сдвига, Па.
    pub interlayer_shear: f64,
    /// Модуль упругости в направлении нагрузки, Па (без учёта заполнения).
    pub modulus: f64,
    /// Доли несущего материала.
    pub fill: FillFractions,
    /// Множитель температуры.
    pub k_temperature: f64,
}

/// Линейный переход от `along` к `across` по углу с насыщением на [`OFF_AXIS_FULL_DROP_DEG`].
fn off_axis(along: f64, across: f64, angle_deg: f64) -> f64 {
    let t = (angle_deg.abs() / OFF_AXIS_FULL_DROP_DEG).min(1.0);
    along + (across - along) * t
}

/// Прочность печатной детали с учётом всех снижений.
pub fn printed_strength(
    m: &Material,
    p: &PrintProcess,
    section: &Section,
    t_c: f64,
) -> PrintedStrength {
    let k_t = temperature_factor(m, t_c);
    let (k_along, k_across) = layer_height_factors(p.layer, p.nozzle);
    let sy_across = m.sy_across * INTERLAYER_BOND_FACTOR * k_across * k_t;
    let su_across = m.su_across * INTERLAYER_BOND_FACTOR * k_across * k_t;
    let sy_along = m.sy_along * k_along * k_t;
    let su_along = m.su_along * k_along * k_t;
    // Вдоль слоёв деталь не может оказаться слабее, чем поперёк: страховка от странных данных.
    let yield_strength = off_axis(sy_along.max(sy_across), sy_across, p.load_angle_deg);
    let ultimate = off_axis(su_along.max(su_across), su_across, p.load_angle_deg);
    let modulus = off_axis(m.e_along, m.e_across, p.load_angle_deg) * k_t;
    PrintedStrength {
        yield_strength,
        ultimate,
        across: sy_across,
        interlayer_shear: sy_across * SHEAR_FROM_TENSION,
        modulus,
        fill: fill_fractions(section, p.shell_thickness(), p.infill),
        k_temperature: k_t,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::catalog::{MaterialId, PLA};

    const S: Section = Section::Rect { b: 0.02, h: 0.01 };

    fn solid(angle: f64) -> PrintProcess {
        PrintProcess {
            infill: 1.0,
            load_angle_deg: angle,
            ..PrintProcess::default()
        }
    }

    #[test]
    fn across_layers_clearly_weaker() {
        for id in MaterialId::ALL {
            let m = id.material();
            if !m.is_printed() {
                continue;
            }
            let along = printed_strength(m, &solid(0.0), &S, 23.0);
            let across = printed_strength(m, &solid(90.0), &S, 23.0);
            assert!(
                across.yield_strength <= 0.75 * along.yield_strength,
                "{}: поперёк {:.1} МПа, вдоль {:.1} МПа",
                m.name,
                across.yield_strength / 1e6,
                along.yield_strength / 1e6
            );
        }
    }

    #[test]
    fn strength_falls_with_angle_and_saturates() {
        let mut prev = f64::INFINITY;
        for a in 0..=90 {
            let s = printed_strength(&PLA, &solid(a as f64), &S, 23.0).yield_strength;
            assert!(s <= prev + 1e-6, "угол {a}");
            prev = s;
        }
        let at30 = printed_strength(&PLA, &solid(30.0), &S, 23.0).yield_strength;
        let at90 = printed_strength(&PLA, &solid(90.0), &S, 23.0).yield_strength;
        assert_eq!(at30, at90);
    }

    #[test]
    fn solid_part_has_full_section() {
        let f = fill_fractions(&S, 0.0016, 1.0);
        assert_eq!((f.area, f.bending, f.stiffness), (1.0, 1.0, 1.0));
    }

    #[test]
    fn walls_filling_the_section_make_it_solid() {
        // 4 стенки по 0.4 мм с двух сторон = 3.2 мм > сечения 3×3 мм.
        let f = fill_fractions(&Section::Rect { b: 0.003, h: 0.003 }, 0.0016, 0.1);
        assert_eq!((f.area, f.bending), (1.0, 1.0));
    }

    #[test]
    fn less_infill_and_fewer_walls_are_weaker() {
        let section = Section::Rect { b: 0.03, h: 0.02 };
        let f40 = fill_fractions(&section, 0.0016, 0.4);
        let f15 = fill_fractions(&section, 0.0016, 0.15);
        let thin = fill_fractions(&section, 0.0008, 0.15);
        assert!(f15.area < f40.area && f15.bending < f40.bending);
        assert!(thin.area < f15.area && thin.bending < f15.bending);
        // Изгиб держат наружные волокна: доля по изгибу больше доли по площади.
        assert!(f40.bending > f40.area);
    }

    #[test]
    fn tube_core_between_two_shells() {
        let tube = Section::Tube {
            od: 0.03,
            wall: 0.006,
        };
        let f = fill_fractions(&tube, 0.0012, 0.0);
        // Сердцевина без заполнения: остаются два кольца по 1.2 мм.
        let core = Section::Tube {
            od: 0.03 - 0.0024,
            wall: 0.006 - 0.0024,
        };
        let expected = 1.0 - core.area() / tube.area();
        assert!((f.area - expected).abs() < 1e-12);
    }

    #[test]
    fn infill_fractions_monotonic() {
        let mut prev = -1.0;
        for i in 0..=100 {
            let k = infill_strength_fraction(i as f64 / 100.0);
            assert!(k > prev, "{i}");
            prev = k;
        }
        assert_eq!(infill_strength_fraction(1.0), 1.0);
        assert_eq!(infill_strength_fraction(0.0), 0.0);
    }

    #[test]
    fn coarse_layer_is_weaker() {
        let (a, c) = layer_height_factors(0.3e-3, 0.4e-3);
        assert!((a - 0.9).abs() < 1e-12 && (c - 0.8).abs() < 1e-12);
        assert_eq!(layer_height_factors(0.2e-3, 0.4e-3), (1.0, 1.0));
        assert_eq!(layer_height_factors(f64::NAN, 0.4e-3), (0.5, 0.5));
    }

    #[test]
    fn hot_part_loses_strength() {
        let cold = printed_strength(&PLA, &solid(0.0), &S, 23.0).yield_strength;
        let warm = printed_strength(&PLA, &solid(0.0), &S, 40.0).yield_strength;
        let soft = printed_strength(&PLA, &solid(0.0), &S, 54.0).yield_strength;
        assert!(warm < 0.6 * cold);
        assert_eq!(soft, 0.0);
    }

    #[test]
    fn invalid_process_rejected() {
        for p in [
            PrintProcess {
                nozzle: f64::NAN,
                ..PrintProcess::default()
            },
            PrintProcess {
                layer: 0.0,
                ..PrintProcess::default()
            },
            PrintProcess {
                infill: 1.5,
                ..PrintProcess::default()
            },
            PrintProcess {
                infill: f64::NAN,
                ..PrintProcess::default()
            },
            PrintProcess {
                load_angle_deg: 120.0,
                ..PrintProcess::default()
            },
            PrintProcess {
                nozzle: 0.4,
                ..PrintProcess::default()
            }, // сопло 0.4 м: перепутаны единицы
        ] {
            assert!(p.validate().is_err(), "{p:?}");
        }
    }
}
