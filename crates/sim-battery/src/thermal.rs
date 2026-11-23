//! Нагрев батареи при профиле нагрузки и температура поверхности у кожи.
//!
//! Модель сосредоточенной теплоёмкости: `C·dT/dt = Q(T) − (T − Ta)/R_th`, где
//! `Q = N·I²·R(T) + I_бат²·R_пути`. Все ячейки считаются худшими: ток с учётом разброса,
//! сопротивление конца службы, ниже 25 °C — рост по Аррениусу, выше 25 °C снижения нет.
//! На шаге `Q` заморожено, и уравнение решается точно (экспонента).
//!
//! Не учтено: градиент внутри ячейки (сердцевина горячее поверхности), обратимое тепло,
//! солнце, тепло соседних узлов устройства, теплоотвод в тело (он только снижал бы нагрев).
//!
//! Без данных о корпусе считается адиабата: это верхняя граница нагрева. Если даже она ниже
//! предела — проверка пройдена; иначе вывода нет (`NotAnalyzed`).
//!
//! Поверхность у кожи — делитель теплового потока между ячейкой и телом:
//! `T_s = T − (T − 37)·R_пр/(R_пр + R_ткани)`. Сопротивление ткани взято большим (слой жира
//! 15 мм): чем оно больше, тем горячее поверхность. Порог ожога — ISO 13732-1 из `sim-env`.

use sim_env::contact::ContactClass;
use sim_env::convection::{Medium, h_radiation, h_vertical_natural};

use crate::pack::{EOL_RESISTANCE_FACTOR, LoadProfile, PackDesign, cold_resistance_factor};

/// Температура ядра тела, °C.
pub const BODY_CORE_C: f64 = 37.0;
/// Теплопроводность жира, Вт/(м·К): оценка, типичное значение; по таблице IT'IS Foundation
/// не подтверждено (страница базы не отдаёт числа). Вместе с толщиной задаёт сопротивление ткани.
pub const FAT_K_W_MK: f64 = 0.21;
/// Толщина слоя ткани: оценка, большая толщина даёт более горячую поверхность.
pub const TISSUE_THICKNESS_M: f64 = 0.015;
/// Внутренняя теплоотдача от ячеек к стенке по умолчанию, Вт/(м²·К): оценка для неподвижного
/// воздуха в закрытом корпусе, нижняя сторона диапазона.
pub const H_INNER_DEFAULT_W_M2K: f64 = 3.0;

const MAX_SUBSTEPS_TOTAL: f64 = 2.0e6;
const MAX_CYCLES: u32 = 100_000;

/// Корпус батареи.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnclosureDesign {
    /// Наружная площадь, м².
    pub outer_area_m2: f64,
    /// Высота (для свободной конвекции), м.
    pub height_m: f64,
    /// Толщина стенки, м.
    pub wall_thickness_m: f64,
    /// Теплопроводность стенки, Вт/(м·К).
    pub wall_k_w_mk: f64,
    /// Внутренняя теплоотдача, Вт/(м²·К).
    pub h_inner_w_m2k: f64,
    /// Излучательная способность наружной поверхности.
    pub emissivity: f64,
    /// Доля площади, прижатая к телу или одежде (не охлаждается), 0…1.
    pub body_side_fraction: f64,
}

impl EnclosureDesign {
    /// Тепловое сопротивление «ячейки — среда», К/Вт. Наружная теплоотдача считается при
    /// перегреве 10 К: там она наименьшая.
    pub fn r_th_k_w(&self, t_ambient_c: f64) -> f64 {
        let ts = t_ambient_c + 10.0;
        let h_out = h_vertical_natural(&Medium::air(50.0), self.height_m, ts, t_ambient_c)
            + h_radiation(self.emissivity, ts, t_ambient_c);
        let free_area = self.outer_area_m2 * (1.0 - self.body_side_fraction);
        1.0 / (self.h_inner_w_m2k * self.outer_area_m2)
            + self.wall_thickness_m / (self.wall_k_w_mk * self.outer_area_m2)
            + 1.0 / (h_out * free_area)
    }
}

/// Прилегание батареи к коже.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkinInterface {
    /// Толщина прокладки между ячейками и кожей, м (0 — вплотную).
    pub spacer_thickness_m: f64,
    /// Теплопроводность прокладки, Вт/(м·К).
    pub spacer_k_w_mk: f64,
    /// Время контакта, с.
    pub contact_time_s: f64,
    /// Класс материала поверхности по ISO 13732-1.
    pub surface_class: ContactClass,
    /// Доля поверхности тела под батареей, 0…1.
    pub contact_body_fraction: f64,
}

impl SkinInterface {
    /// Температура поверхности, касающейся кожи, °C, при температуре ячеек.
    pub fn surface_temp_c(&self, t_cell_c: f64) -> f64 {
        let r_spacer = self.spacer_thickness_m / self.spacer_k_w_mk;
        let r_tissue = TISSUE_THICKNESS_M / FAT_K_W_MK;
        t_cell_c - (t_cell_c - BODY_CORE_C) * r_spacer / (r_spacer + r_tissue)
    }
}

/// Результат теплового расчёта.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalResult {
    /// Наибольшая температура ячеек, °C.
    pub t_max_c: f64,
    /// Температура в конце профиля, °C.
    pub t_end_c: f64,
    /// Считалось без теплоотвода: данных о корпусе нет.
    pub adiabatic: bool,
    /// Наибольшее тепловыделение, Вт.
    pub heat_peak_w: f64,
}

/// Нагрев батареи за профиль. Старт — при наибольшей температуре среды.
/// `None` — нет сопротивления, массы или теплоёмкости ячейки.
pub fn simulate_temperature(pack: &PackDesign, load: &LoadProfile) -> Option<ThermalResult> {
    let cell = &pack.cell;
    let r_high = cell.r_dc_high_ohm.value?;
    let mass = cell.mass_kg.value?;
    let cp = cell.cp_j_kg_k.value?;
    let ea = cell.ea_r_j_mol.value?;
    if load.steps.is_empty() {
        return None;
    }

    let n = pack.cell_count() as f64;
    let c_total = n * mass * cp;
    let t_amb = load.ambient_max_c;
    let r_th = pack.enclosure.as_ref().map(|e| e.r_th_k_w(t_amb));
    let r_path = pack.interconnect_r_ohm + pack.bms.as_ref().map_or(0.0, |b| b.mosfet.rds_on_ohm);

    let cycles = load.cycles.clamp(1, MAX_CYCLES);
    let cycle_time: f64 = load.steps.iter().map(|s| s.duration_s).sum();
    let total_time = cycle_time * cycles as f64;
    let mut dt_max = 5.0_f64;
    if let Some(rt) = r_th {
        let tau20 = c_total * rt / 20.0;
        if tau20 < dt_max {
            dt_max = tau20;
        }
    }
    if total_time / dt_max > MAX_SUBSTEPS_TOTAL {
        dt_max = total_time / MAX_SUBSTEPS_TOTAL;
    }

    let mut t = t_amb;
    let mut t_max = t;
    let mut heat_peak = 0.0_f64;
    for _ in 0..cycles {
        let start = t;
        for step in &load.steps {
            let i_cell = pack.worst_cell_current_a(step.current_a);
            let substeps = (step.duration_s / dt_max).ceil().clamp(1.0, 1.0e6);
            let dt = step.duration_s / substeps;
            for _ in 0..substeps as u64 {
                let r = r_high * EOL_RESISTANCE_FACTOR * cold_resistance_factor(ea, t);
                let q = n * i_cell * i_cell * r + step.current_a * step.current_a * r_path;
                if q > heat_peak {
                    heat_peak = q;
                }
                t = match r_th {
                    Some(rt) => {
                        let t_eq = t_amb + q * rt;
                        t_eq + (t - t_eq) * (-dt / (c_total * rt)).exp()
                    }
                    None => t + q * dt / c_total,
                };
                if !t.is_finite() {
                    return Some(ThermalResult {
                        t_max_c: f64::INFINITY,
                        t_end_c: f64::INFINITY,
                        adiabatic: r_th.is_none(),
                        heat_peak_w: heat_peak,
                    });
                }
                if t > t_max {
                    t_max = t;
                }
            }
        }
        if (t - start).abs() < 1e-6 {
            break;
        }
    }
    Some(ThermalResult {
        t_max_c: t_max,
        t_end_c: t,
        adiabatic: r_th.is_none(),
        heat_peak_w: heat_peak,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cells::CellSpec;
    use crate::pack::LoadStep;
    use crate::wire::{Conductor, Insulation, WireCooling, WireRun};

    fn enclosure() -> EnclosureDesign {
        EnclosureDesign {
            outer_area_m2: 0.06,
            height_m: 0.1,
            wall_thickness_m: 0.002,
            wall_k_w_mk: 0.18,
            h_inner_w_m2k: H_INNER_DEFAULT_W_M2K,
            emissivity: 0.85,
            body_side_fraction: 0.3,
        }
    }

    fn pack(enc: Option<EnclosureDesign>) -> PackDesign {
        PackDesign {
            cell: CellSpec::lg_m50(),
            series: 4,
            parallel: 2,
            cell_resistance_spread: 0.0,
            group_voltage_spread_v: 0.0,
            interconnect_r_ohm: 0.0,
            contact_r_ohm: 0.0,
            wire: WireRun {
                conductor: Conductor::Awg(14),
                insulation: Insulation::Silicone,
                length_m: 0.5,
                cooling: WireCooling::FreeAir,
                outer_diameter_mm: None,
            },
            fuse: None,
            bms: None,
            enclosure: enc,
            skin: None,
            worn_on_body: true,
        }
    }

    fn load(i: f64, d: f64) -> LoadProfile {
        LoadProfile {
            steps: vec![LoadStep {
                current_a: i,
                duration_s: d,
            }],
            cycles: 1,
            ambient_min_c: 25.0,
            ambient_max_c: 25.0,
            load_v_min: None,
            load_v_max: None,
            charge: None,
        }
    }

    #[test]
    fn step_response_matches_closed_form() {
        let p = pack(Some(enclosure()));
        let l = load(4.0, 1800.0);
        let r = simulate_temperature(&p, &l).expect("данные есть");
        let rt = enclosure().r_th_k_w(25.0);
        let c = 8.0 * 0.0685 * 887.0;
        let q = 8.0 * 2.0 * 2.0 * (0.036 * EOL_RESISTANCE_FACTOR);
        let exact = 25.0 + q * rt * (1.0 - (-1800.0 / (c * rt)).exp());
        assert!((r.t_end_c - exact).abs() < 1e-6, "{} {exact}", r.t_end_c);
    }

    #[test]
    fn adiabatic_without_enclosure_is_upper_bound() {
        let l = load(6.0, 1200.0);
        let a = simulate_temperature(&pack(None), &l).expect("данные есть");
        let b = simulate_temperature(&pack(Some(enclosure())), &l).expect("данные есть");
        assert!(a.adiabatic && !b.adiabatic);
        assert!(a.t_max_c > b.t_max_c);
        let c = 8.0 * 0.0685 * 887.0;
        let q = 8.0 * 3.0 * 3.0 * (0.036 * EOL_RESISTANCE_FACTOR);
        assert!((a.t_end_c - (25.0 + q * 1200.0 / c)).abs() < 1e-6);
    }

    #[test]
    fn blocked_body_side_heats_more() {
        let l = load(6.0, 3600.0);
        let mut e = enclosure();
        e.body_side_fraction = 0.0;
        let open = simulate_temperature(&pack(Some(e)), &l)
            .expect("есть")
            .t_max_c;
        e.body_side_fraction = 0.6;
        let blocked = simulate_temperature(&pack(Some(e)), &l)
            .expect("есть")
            .t_max_c;
        assert!(blocked > open + 1.0, "{blocked} {open}");
    }

    #[test]
    fn spacer_lowers_skin_surface() {
        let mut s = SkinInterface {
            spacer_thickness_m: 0.0,
            spacer_k_w_mk: 0.05,
            contact_time_s: 3600.0,
            surface_class: ContactClass::Plastics,
            contact_body_fraction: 0.02,
        };
        assert_eq!(s.surface_temp_c(50.0), 50.0);
        s.spacer_thickness_m = 0.004;
        let t = s.surface_temp_c(50.0);
        assert!(t < 50.0 && t > 37.0, "{t}");
    }

    #[test]
    fn huge_cycle_count_finishes() {
        let p = pack(Some(enclosure()));
        let mut l = load(2.0, 1.0);
        l.cycles = u32::MAX;
        l.steps.push(LoadStep {
            current_a: 0.0,
            duration_s: 1.0,
        });
        let r = simulate_temperature(&p, &l).expect("данные есть");
        assert!(r.t_max_c.is_finite());
    }
}
