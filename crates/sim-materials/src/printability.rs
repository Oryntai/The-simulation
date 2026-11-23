//! Можно ли такую деталь напечатать. Принтер как устройство не моделируется: это одна проверка
//! геометрии и настроек против материала. Итог — блокеры и предупреждения.
//!
//! Базовые правила FDM: стенка ≥ 0.8 мм, элементы и отверстия ≥ 2 мм (Hubs, «DFM tips for 3D
//! printed parts with thin walls»), нависание 45°, мост 10 мм (правила Hubs во вторичных
//! источниках). Предельные значения по материалам — из паспортов Bambu Lab, см. каталог.

use crate::catalog::{Material, MaterialClass};
use crate::printed::{MAX_LAYER_RATIO, PrintProcess};

/// Нависание, которое печатается без оговорок, градусы от вертикали.
pub const SAFE_OVERHANG_DEG: f64 = 45.0;
/// Мост, который печатается без оговорок, м.
pub const SAFE_BRIDGE: f64 = 10e-3;
/// Отверстие меньше этого диаметра печатается неточно, м (Hubs: 2 мм).
pub const MIN_RELIABLE_HOLE: f64 = 2e-3;
/// Стенка в линиях сопла, с которой она считается надёжной (0.8 мм при сопле 0.4 мм).
pub const MIN_WALL_LINES: f64 = 2.0;
/// Длина детали без закрытой камеры, с которой коробление ABS/ASA/PA считается блокером, м.
/// **Оценка**: порог зависит от формы и адгезии, взят по типичному столу.
pub const WARP_BLOCK_LENGTH: f64 = 100e-3;

/// Геометрия и условия печати детали. Все размеры — м.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintGeometry {
    /// Габарит детали в положении печати: X, Y, Z.
    pub bbox: [f64; 3],
    /// Объём печати: X, Y, Z.
    pub build_volume: [f64; 3],
    /// Самая тонкая стенка.
    pub min_wall: f64,
    /// Наибольшее нависание, градусы от вертикали (0 — стенка вертикальна).
    pub max_overhang_deg: f64,
    /// Самый длинный мост (0 — мостов нет).
    pub max_bridge: f64,
    /// Самое маленькое отверстие (0 — отверстий нет).
    pub min_hole: f64,
    /// Включены поддержки.
    pub supports: bool,
    /// Принтер с закрытой камерой.
    pub enclosed: bool,
}

impl PrintGeometry {
    /// Проверка диапазонов.
    pub fn validate(&self) -> Result<(), String> {
        for (name, values) in [("габарит", self.bbox), ("объём печати", self.build_volume)]
        {
            for v in values {
                if !(v.is_finite() && v > 0.0 && v <= 10.0) {
                    return Err(format!("{name} вне диапазона: {v} м"));
                }
            }
        }
        if !(self.min_wall.is_finite() && self.min_wall > 0.0 && self.min_wall <= 1.0) {
            return Err(format!("минимальная стенка некорректна: {}", self.min_wall));
        }
        if !(self.max_overhang_deg.is_finite() && (0.0..=90.0).contains(&self.max_overhang_deg)) {
            return Err(format!(
                "нависание вне диапазона 0..90°: {}",
                self.max_overhang_deg
            ));
        }
        for (name, v) in [("мост", self.max_bridge), ("отверстие", self.min_hole)] {
            if !(v.is_finite() && (0.0..=10.0).contains(&v)) {
                return Err(format!("{name} вне диапазона: {v} м"));
            }
        }
        Ok(())
    }
}

/// Итог проверки печати.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct PrintReport {
    /// Деталь в таком виде не напечатать.
    pub blockers: Vec<String>,
    /// Напечатать можно, но есть риск.
    pub warnings: Vec<String>,
}

impl PrintReport {
    /// Печатается ли деталь.
    pub fn printable(&self) -> bool {
        self.blockers.is_empty()
    }
}

fn mm(v: f64) -> f64 {
    v * 1e3
}

/// Проверка печати. Для непечатного материала — блокер.
pub fn check_printability(m: &Material, p: &PrintProcess, g: &PrintGeometry) -> PrintReport {
    let mut r = PrintReport::default();
    let Some(traits) = m.print.filter(|_| m.class == MaterialClass::Printed) else {
        r.blockers
            .push(format!("{} — не печатный материал", m.name));
        return r;
    };
    if let Err(e) = p.validate() {
        r.blockers.push(e);
        return r;
    }
    if let Err(e) = g.validate() {
        r.blockers.push(e);
        return r;
    }

    // Габарит — в заданном положении: поворот детали меняет направление слоёв и её прочность.
    for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
        if g.bbox[i] > g.build_volume[i] {
            r.blockers.push(format!(
                "деталь не влезает по {axis}: {:.1} мм при объёме {:.1} мм",
                mm(g.bbox[i]),
                mm(g.build_volume[i])
            ));
        }
    }

    // Сопло и слой.
    let ratio = p.layer / p.nozzle;
    if ratio > MAX_LAYER_RATIO {
        r.blockers.push(format!(
            "слой {:.2} мм больше {:.0}% сопла {:.2} мм: валики не сплавятся",
            mm(p.layer),
            MAX_LAYER_RATIO * 100.0,
            mm(p.nozzle)
        ));
    } else if ratio > 0.75 {
        r.warnings.push(format!(
            "слой {:.0}% сопла: на грани сплавления",
            ratio * 100.0
        ));
    }
    if p.walls == 0 {
        r.blockers.push("нет ни одной стенки".into());
    }

    // Минимальная стенка от сопла.
    if g.min_wall < p.nozzle {
        r.blockers.push(format!(
            "стенка {:.2} мм тоньше линии сопла {:.2} мм: не напечатается",
            mm(g.min_wall),
            mm(p.nozzle)
        ));
    } else if g.min_wall < MIN_WALL_LINES * p.nozzle {
        r.warnings.push(format!(
            "стенка {:.2} мм — меньше двух линий ({:.2} мм): непрочная и неточная",
            mm(g.min_wall),
            mm(MIN_WALL_LINES * p.nozzle)
        ));
    }

    // Нависания.
    if g.max_overhang_deg > SAFE_OVERHANG_DEG {
        if g.supports {
            r.warnings.push(format!(
                "нависание {:.0}° держится на поддержках: снизу будет грубая поверхность",
                g.max_overhang_deg
            ));
        } else if g.max_overhang_deg > traits.max_overhang_deg {
            r.blockers.push(format!(
                "нависание {:.0}° без поддержек при пределе {:.0}° для {}",
                g.max_overhang_deg, traits.max_overhang_deg, m.name
            ));
        } else {
            r.warnings.push(format!(
                "нависание {:.0}° без поддержек: выше базовых 45°, возможны дефекты",
                g.max_overhang_deg
            ));
        }
    }

    // Мосты.
    if g.max_bridge > SAFE_BRIDGE {
        if g.supports {
            r.warnings.push(format!(
                "мост {:.0} мм держится на поддержках",
                mm(g.max_bridge)
            ));
        } else if g.max_bridge > traits.max_bridge {
            r.blockers.push(format!(
                "мост {:.0} мм без поддержек при пределе {:.0} мм для {}",
                mm(g.max_bridge),
                mm(traits.max_bridge),
                m.name
            ));
        } else {
            r.warnings.push(format!(
                "мост {:.0} мм без поддержек: длиннее базовых 10 мм, возможно провисание",
                mm(g.max_bridge)
            ));
        }
    }

    // Отверстия.
    if g.min_hole > 0.0 {
        if g.min_hole < 2.0 * p.nozzle {
            r.blockers.push(format!(
                "отверстие Ø{:.2} мм меньше двух линий сопла: заплывёт",
                mm(g.min_hole)
            ));
        } else if g.min_hole < MIN_RELIABLE_HOLE {
            r.warnings.push(format!(
                "отверстие Ø{:.2} мм меньше надёжных 2 мм: рассверлить после печати",
                mm(g.min_hole)
            ));
        }
    }

    // Материал: усадка и закрытая камера.
    let longest = g.bbox.iter().copied().fold(0.0, f64::max);
    if traits.needs_enclosure && !g.enclosed {
        let text = format!(
            "{} без закрытой камеры: усадка и коробление, расслоение при длине {:.0} мм",
            m.name,
            mm(longest)
        );
        if longest >= WARP_BLOCK_LENGTH {
            r.blockers.push(text);
        } else {
            r.warnings.push(text);
        }
    }
    if let Some(cte) = traits.cte {
        // Оценка усадки при остывании от температуры стеклования до комнатной.
        let strain = cte * (traits.set_temp_c - 25.0);
        r.warnings.push(format!(
            "усадка {} ≈ {:.2}%: на длине {:.0} мм это {:.2} мм, заложить в размеры",
            m.name,
            strain * 100.0,
            mm(longest),
            mm(longest * strain)
        ));
    }
    if traits.abrasive {
        r.warnings
            .push("абразивный материал: нужно закалённое сопло".into());
        if p.nozzle < 0.4e-3 {
            r.blockers
                .push("волокнистый материал в сопле меньше 0.4 мм: забьётся".into());
        }
    }
    if traits.hygroscopic {
        r.warnings
            .push(format!("{} впитывает влагу: сушить перед печатью", m.name));
    }
    if traits.flexible {
        r.warnings
            .push("гибкий пруток: нужен прямой экструдер, малая скорость".into());
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::catalog::{ABS, AL_6061_T6, PA_CF, PLA, TPU_95A};

    fn geom() -> PrintGeometry {
        PrintGeometry {
            bbox: [0.05, 0.03, 0.02],
            build_volume: [0.256, 0.256, 0.256],
            min_wall: 2e-3,
            max_overhang_deg: 30.0,
            max_bridge: 5e-3,
            min_hole: 3e-3,
            supports: false,
            enclosed: true,
        }
    }

    fn proc() -> PrintProcess {
        PrintProcess::default()
    }

    #[test]
    fn good_part_is_printable() {
        let r = check_printability(&PLA, &proc(), &geom());
        assert!(r.printable(), "{:?}", r.blockers);
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    }

    #[test]
    fn too_big_is_blocked() {
        let g = PrintGeometry {
            bbox: [0.3, 0.03, 0.02],
            ..geom()
        };
        let r = check_printability(&PLA, &proc(), &g);
        assert!(
            r.blockers.iter().any(|b| b.contains("не влезает по X")),
            "{:?}",
            r.blockers
        );
    }

    #[test]
    fn wall_thinner_than_nozzle_blocked() {
        let g = PrintGeometry {
            min_wall: 0.3e-3,
            ..geom()
        };
        assert!(!check_printability(&PLA, &proc(), &g).printable());
        let thin = PrintGeometry {
            min_wall: 0.6e-3,
            ..geom()
        };
        let r = check_printability(&PLA, &proc(), &thin);
        assert!(r.printable());
        assert!(r.warnings.iter().any(|w| w.contains("двух линий")));
    }

    #[test]
    fn overhang_without_supports_blocked() {
        let g = PrintGeometry {
            max_overhang_deg: 75.0,
            ..geom()
        };
        let r = check_printability(&PLA, &proc(), &g);
        assert!(
            r.blockers.iter().any(|b| b.contains("нависание")),
            "{:?}",
            r.blockers
        );
        // С поддержками — можно, но с предупреждением.
        let g2 = PrintGeometry {
            supports: true,
            ..g
        };
        let r2 = check_printability(&PLA, &proc(), &g2);
        assert!(r2.printable());
        assert!(r2.warnings.iter().any(|w| w.contains("поддержках")));
    }

    #[test]
    fn long_bridge_blocked() {
        let g = PrintGeometry {
            max_bridge: 50e-3,
            ..geom()
        };
        let r = check_printability(&PLA, &proc(), &g);
        assert!(
            r.blockers.iter().any(|b| b.contains("мост")),
            "{:?}",
            r.blockers
        );
    }

    #[test]
    fn tiny_hole_blocked() {
        let g = PrintGeometry {
            min_hole: 0.5e-3,
            ..geom()
        };
        assert!(!check_printability(&PLA, &proc(), &g).printable());
        let g2 = PrintGeometry {
            min_hole: 1.2e-3,
            ..geom()
        };
        let r = check_printability(&PLA, &proc(), &g2);
        assert!(r.printable());
        assert!(r.warnings.iter().any(|w| w.contains("рассверлить")));
    }

    #[test]
    fn coarse_layer_blocked() {
        let p = PrintProcess {
            layer: 0.36e-3,
            ..proc()
        };
        assert!(!check_printability(&PLA, &p, &geom()).printable());
    }

    #[test]
    fn abs_long_part_needs_enclosure() {
        let long = PrintGeometry {
            bbox: [0.2, 0.03, 0.02],
            enclosed: false,
            ..geom()
        };
        let r = check_printability(&ABS, &proc(), &long);
        assert!(
            r.blockers.iter().any(|b| b.contains("камеры")),
            "{:?}",
            r.blockers
        );
        // Короткая — можно, но с предупреждением; в камере — без блокера.
        let short = PrintGeometry {
            enclosed: false,
            ..geom()
        };
        let r = check_printability(&ABS, &proc(), &short);
        assert!(r.printable());
        assert!(r.warnings.iter().any(|w| w.contains("камеры")));
        let closed = PrintGeometry {
            enclosed: true,
            ..long
        };
        assert!(check_printability(&ABS, &proc(), &closed).printable());
    }

    #[test]
    fn shrinkage_reported_for_abs() {
        let r = check_printability(&ABS, &proc(), &geom());
        assert!(
            r.warnings.iter().any(|w| w.contains("усадка")),
            "{:?}",
            r.warnings
        );
    }

    #[test]
    fn abrasive_needs_hard_nozzle_and_large_bore() {
        let r = check_printability(&PA_CF, &proc(), &geom());
        assert!(r.warnings.iter().any(|w| w.contains("закалённое")));
        let small = PrintProcess {
            nozzle: 0.25e-3,
            layer: 0.1e-3,
            ..proc()
        };
        let g = PrintGeometry {
            min_wall: 1e-3,
            ..geom()
        };
        assert!(!check_printability(&PA_CF, &small, &g).printable());
    }

    #[test]
    fn tpu_flexible_warning() {
        let r = check_printability(&TPU_95A, &proc(), &geom());
        assert!(r.warnings.iter().any(|w| w.contains("прямой экструдер")));
    }

    #[test]
    fn metal_is_not_printable_here() {
        assert!(!check_printability(&AL_6061_T6, &proc(), &geom()).printable());
    }

    #[test]
    fn nan_is_a_blocker() {
        for g in [
            PrintGeometry {
                min_wall: f64::NAN,
                ..geom()
            },
            PrintGeometry {
                max_overhang_deg: f64::NAN,
                ..geom()
            },
            PrintGeometry {
                max_bridge: f64::INFINITY,
                ..geom()
            },
            PrintGeometry {
                min_hole: -1.0,
                ..geom()
            },
            PrintGeometry {
                bbox: [f64::NAN, 0.03, 0.02],
                ..geom()
            },
            PrintGeometry {
                build_volume: [0.2, 0.0, 0.2],
                ..geom()
            },
        ] {
            assert!(!check_printability(&PLA, &proc(), &g).printable(), "{g:?}");
        }
        let p = PrintProcess {
            nozzle: f64::NAN,
            ..proc()
        };
        assert!(!check_printability(&PLA, &p, &geom()).printable());
    }
}
