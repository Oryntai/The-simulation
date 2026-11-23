//! Импульс через конструкцию, движение резьбы и локальные критерии опасности.

use std::f64::consts::TAU;

use sim_materials::fasteners::{FastenerSpec, MemberStrength, check_fasteners};
use sim_signal::Rng;
use sim_world::Provenance;

use crate::contact::ContactFrame;
use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::trace::{matches_pair, validate_frame, validate_pair};
use super::{HazardCheck, math::*};

#[derive(Clone, Debug)]
pub struct ImpactPath {
    /// Первое тело действует на конструкцию.
    pub input_pair: [String; 2],
    /// Конструкция действует на второе тело.
    pub output_pair: [String; 2],
    pub carrier_mass_kg: SourcedRange,
    pub momentum_tolerance_ns: SourcedRange,
    /// Подтверждает, что указанные геометрии принадлежат нужным телам.
    pub path_source: Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct ImpactInput {
    pub carrier_velocity_start_m_s: [f64; 3],
    pub carrier_velocity_end_m_s: [f64; 3],
    /// Импульс остальных сил на конструкцию, включая вес и реакции опор.
    pub other_impulse_ns: Option<[f64; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactStep {
    pub input_impulse_ns: [f64; 3],
    pub output_impulse_ns: [f64; 3],
    pub reaction_on_carrier_ns: [f64; 3],
    pub carrier_delta_momentum_ns: [f64; 3],
    pub residual_ns: [f64; 3],
}

#[derive(Clone, Debug)]
pub struct ImpactFrameInput {
    pub contacts: Option<ContactFrame>,
    pub kinematics: ImpactInput,
}

impl StepModel for ImpactPath {
    type Input = ImpactFrameInput;
    type Output = Analysis<ImpactStep>;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        self.evaluate(interval, input.contacts.as_ref(), &input.kinematics)
    }
}

impl ImpactPath {
    pub fn evaluate(
        &self,
        interval: Interval,
        frame: Option<&ContactFrame>,
        input: &ImpactInput,
    ) -> Result<Analysis<ImpactStep>, ProcessError> {
        let dt = duration(interval)?;
        validate_pair(&self.input_pair)?;
        validate_pair(&self.output_pair)?;
        if self.input_pair == self.output_pair
            || self.input_pair[0] == self.output_pair[1]
                && self.input_pair[1] == self.output_pair[0]
        {
            return Err(invalid("вход и выход удара используют одну пару"));
        }
        source(&self.path_source)?;
        let mass = range(&self.carrier_mass_kg, "масса конструкции", |x| x > 0.0)?;
        range(&self.momentum_tolerance_ns, "допуск импульса", |x| x >= 0.0)?;
        finite(&input.carrier_velocity_start_m_s)?;
        finite(&input.carrier_velocity_end_m_s)?;
        if let Some(other) = input.other_impulse_ns {
            finite(&other)?;
        }
        if let Some(frame) = frame {
            validate_frame(frame, interval)?;
        }
        let (Some(frame), Some(other)) = (frame, input.other_impulse_ns) else {
            return Ok(unknown(
                "неполный путь удара: отсутствуют контакты или остальные импульсы",
            ));
        };
        if !direct(&self.path_source) {
            return Ok(unknown("путь через конструкцию не подтверждён"));
        }
        let impulse = |pair: &[String; 2]| {
            frame
                .contacts
                .iter()
                .filter(|c| matches_pair(c, pair))
                .fold([0.0; 3], |sum, c| {
                    let sign = if c.geometries == *pair { 1.0 } else { -1.0 };
                    add(sum, scale(c.force_n, sign * dt))
                })
        };
        let carrier_delta_momentum_ns = scale(
            sub(
                input.carrier_velocity_end_m_s,
                input.carrier_velocity_start_m_s,
            ),
            mass,
        );
        let input_impulse_ns = impulse(&self.input_pair);
        let output_impulse_ns = impulse(&self.output_pair);
        let residual_ns = sub(
            sub(add(input_impulse_ns, other), output_impulse_ns),
            carrier_delta_momentum_ns,
        );
        for vector in [
            carrier_delta_momentum_ns,
            input_impulse_ns,
            output_impulse_ns,
            residual_ns,
        ] {
            computed(&vector)?;
        }
        if norm(residual_ns) > self.momentum_tolerance_ns.min {
            return Ok(unknown(
                "баланс количества движения не сходится в заданном допуске",
            ));
        }
        Ok(calculated(
            ImpactStep {
                input_impulse_ns,
                output_impulse_ns,
                reaction_on_carrier_ns: scale(output_impulse_ns, -1.0),
                carrier_delta_momentum_ns,
                residual_ns,
            },
            "интеграл средних контактных сил за физический шаг, проверен линейный баланс по номиналу массы",
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Handedness {
    Positive,
    Negative,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EngagementState {
    Engaged,
    Disengaged,
    Stripped,
}

#[derive(Clone, Debug)]
pub struct ThreadStrength {
    /// Только статическое растяжение стандартизованного болта по sim-materials.
    pub fastener: FastenerSpec,
    pub fastener_source: Provenance,
    pub member_bearing_pa: SourcedRange,
    pub member_shear_pa: SourcedRange,
    pub thread_factor: SourcedRange,
    /// Источник подтверждает применимость модели материала в данном соединении.
    pub member_model_source: Option<Provenance>,
}

#[derive(Clone, Debug)]
pub struct ThreadSpec {
    /// Осевое перемещение за один оборот, включая многозаходность.
    pub lead_m_per_turn: SourcedRange,
    pub handedness: Handedness,
    pub efficiency: SourcedRange,
    pub travel_tolerance_m: SourcedRange,
    pub tensile_strength: Option<ThreadStrength>,
}

#[derive(Clone, Copy, Debug)]
pub struct ThreadInput {
    pub engagement: Option<EngagementState>,
    pub angle_delta_rad: f64,
    pub axial_delta_m: f64,
    /// Средний подведённый момент за угловой ход.
    pub drive_torque_nm: f64,
    /// Измеренная растягивающая составляющая нагрузки группы болтов.
    pub measured_tension_n: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreadStep {
    pub expected_travel_m: f64,
    pub travel_error_m: f64,
    pub applied_torque_nm: f64,
    pub axial_force_n: f64,
    pub input_work_j: f64,
    pub axial_work_j: f64,
    pub loss_j: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ThreadResult {
    pub motion: Analysis<ThreadStep>,
    pub alignment: HazardCheck,
    pub strength: HazardCheck,
}

impl StepModel for ThreadSpec {
    type Input = ThreadInput;
    type Output = ThreadResult;

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

impl ThreadSpec {
    pub fn evaluate(&self, input: &ThreadInput) -> Result<ThreadResult, ProcessError> {
        let lead = range(&self.lead_m_per_turn, "ход за оборот", |x| {
            x > 0.0
        })?;
        let eta = range(&self.efficiency, "КПД резьбы", |x| {
            x > 0.0 && x <= 1.0
        })?;
        range(&self.travel_tolerance_m, "допуск хода", |x| {
            x >= 0.0
        })?;
        finite(&[
            input.angle_delta_rad,
            input.axial_delta_m,
            input.drive_torque_nm,
        ])?;
        if let Some(force) = input.measured_tension_n {
            finite(&[force])?;
            if force < 0.0 {
                return Err(invalid(
                    "растягивающая нагрузка задана отрицательным числом",
                ));
            }
        }
        let strength = if let Some(spec) = &self.tensile_strength {
            if self.lead_m_per_turn.min != spec.fastener.size.pitch()
                || self.lead_m_per_turn.max != spec.fastener.size.pitch()
            {
                return Err(invalid("ход не совпадает с шагом однозаходного крепежа"));
            }
            spec.assess(input.measured_tension_n)?
        } else {
            unresolved("прочность зацепления не задана")
        };
        let incomplete = |reason: &str, alignment| ThreadResult {
            motion: unknown(reason),
            alignment,
            strength: strength.clone(),
        };
        match input.engagement {
            Some(EngagementState::Stripped) => {
                return Ok(incomplete(
                    "резьба сорвана",
                    HazardCheck::Violation {
                        reason: "зафиксирован срыв резьбы".into(),
                    },
                ));
            }
            Some(EngagementState::Engaged) => {}
            _ => {
                return Ok(incomplete(
                    "нет подтверждённого зацепления",
                    unresolved("не подтверждено зацепление"),
                ));
            }
        }
        let sign = match self.handedness {
            Handedness::Positive => 1.0,
            Handedness::Negative => -1.0,
        };
        let rotation = input.angle_delta_rad / TAU * sign;
        let expected_travel_m = rotation * lead;
        let travel_error_m = input.axial_delta_m - expected_travel_m;
        let limits = [
            rotation * self.lead_m_per_turn.min,
            rotation * self.lead_m_per_turn.max,
        ];
        computed(&[expected_travel_m, travel_error_m, limits[0], limits[1]])?;
        let low = limits[0].min(limits[1]);
        let high = limits[0].max(limits[1]);
        let nearest = input.axial_delta_m.clamp(low, high);
        let deviation = (input.axial_delta_m - low)
            .abs()
            .max((input.axial_delta_m - high).abs());
        let definite_error = (input.axial_delta_m - nearest).abs();
        computed(&[deviation, definite_error])?;
        let fit_source =
            direct(&self.lead_m_per_turn.provenance) && direct(&self.travel_tolerance_m.provenance);
        if definite_error > self.travel_tolerance_m.max {
            let reason = "фактический ход расходится с вращением";
            let check = if fit_source {
                HazardCheck::Violation {
                    reason: reason.into(),
                }
            } else {
                unresolved("источник допуска не позволяет подтвердить срыв")
            };
            return Ok(incomplete(reason, check));
        }
        if deviation > self.travel_tolerance_m.min {
            return Ok(incomplete(
                "интервал хода выходит за допуск",
                unresolved("срыв не исключён в диапазоне параметров"),
            ));
        }
        let input_work_j = input.drive_torque_nm * input.angle_delta_rad;
        computed(&[input_work_j])?;
        if input_work_j <= 0.0 {
            return Ok(incomplete(
                "статическое зацепление или обратный ход",
                unresolved("нет прямой передачи работы"),
            ));
        }
        // Виртуальная работа по винтовой кинематике: F·dx = η·M·dθ.
        // Сила и работа относятся к номинальному кинематическому ходу.
        let axial_force_n = input.drive_torque_nm * eta * TAU * sign / lead;
        let axial_work_j = input_work_j * eta;
        let loss_j = input_work_j * (1.0 - eta);
        computed(&[axial_force_n, axial_work_j, loss_j])?;
        let alignment = if fit_source {
            HazardCheck::WithinLimit {
                scope: "проверка фактического хода по оборотам".into(),
            }
        } else {
            unresolved("оценочные данные о ходе не закрывают срыв")
        };
        Ok(ThreadResult {
            motion: calculated(
                ThreadStep {
                    expected_travel_m,
                    travel_error_m,
                    applied_torque_nm: input.drive_torque_nm,
                    axial_force_n,
                    input_work_j,
                    axial_work_j,
                    loss_j,
                },
                "прямой ход в устойчивом зацеплении, виртуальная работа с заданным КПД",
            ),
            alignment,
            strength,
        })
    }
}

impl ThreadStrength {
    fn assess(&self, force: Option<f64>) -> Result<HazardCheck, ProcessError> {
        self.fastener.validate().map_err(invalid)?;
        source(&self.fastener_source)?;
        range(&self.member_bearing_pa, "предел смятия", |x| {
            x > 0.0
        })?;
        range(&self.member_shear_pa, "предел сдвига", |x| {
            x > 0.0
        })?;
        range(
            &self.thread_factor,
            "коэффициент вырыва",
            |x| x > 0.0 && x <= 1.0,
        )?;
        if let Some(src) = &self.member_model_source {
            source(src)?;
        }
        if !direct(&self.fastener_source) {
            return Ok(unresolved("не подтверждена геометрия или класс крепежа"));
        }
        let Some(force) = force.filter(|force| *force > 0.0) else {
            return Ok(unresolved("нет ненулевого замера растягивающей нагрузки"));
        };
        let material_confirmed = self.member_model_source.as_ref().is_some_and(direct)
            && [
                &self.member_bearing_pa,
                &self.member_shear_pa,
                &self.thread_factor,
            ]
            .into_iter()
            .all(|value| direct(&value.provenance));
        let check = |bound: fn(&SourcedRange) -> f64| {
            let member = MemberStrength {
                bearing: bound(&self.member_bearing_pa),
                shear: bound(&self.member_shear_pa),
                thread_factor: bound(&self.thread_factor),
                model_applies: material_confirmed,
            };
            // Один статический режим без поперечной силы и циклов.
            let report = check_fasteners(&self.fastener, &member, 0.0, force, 0.0, 0.0);
            for item in &report.items {
                computed(&[item.factor])?;
            }
            Ok(report)
        };
        // Допустимые силы по sim-materials монотонны по каждому из этих параметров.
        let strongest = check(|range| range.max)?;
        if let Some(reason) = strongest.geometry_faults.first() {
            return Ok(HazardCheck::Violation {
                reason: reason.clone(),
            });
        }
        if let Some(item) = strongest.governing().filter(|item| item.factor < 1.0) {
            return Ok(HazardCheck::Violation {
                reason: format!("{}: запас не выше {:.6}", item.name, item.factor),
            });
        }
        if !strongest.not_analyzed.is_empty() {
            return Ok(unresolved(
                "неполные или неподтверждённые свойства материала крепежа",
            ));
        }
        let weakest = check(|range| range.min)?;
        Ok(match weakest.governing() {
            Some(item) if item.factor > 1.0 => HazardCheck::WithinLimit {
                scope: format!(
                    "только статическое растяжение, {}: запас не ниже {:.6}",
                    item.name, item.factor
                ),
            },
            Some(_) => unresolved("разброс прочности пересекает применимый предел"),
            None => unresolved("нет применимой проверки крепежа"),
        })
    }
}

/// Линейный накопитель: E = коэффициент · координата² / 2.
#[derive(Clone, Debug, PartialEq)]
pub enum EnergyStore {
    Spring {
        stiffness_n_m: SourcedRange,
        deflection_m: SourcedRange,
    },
    TorsionSpring {
        stiffness_nm_rad: SourcedRange,
        deflection_rad: SourcedRange,
    },
    Capacitor {
        capacitance_f: SourcedRange,
        voltage_v: SourcedRange,
    },
    Inductor {
        inductance_h: SourcedRange,
        current_a: SourcedRange,
    },
    Flywheel {
        inertia_kg_m2: SourcedRange,
        speed_rad_s: SourcedRange,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnergyBounds {
    pub min_j: f64,
    pub nominal_j: f64,
    pub max_j: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EnergyResult {
    pub energy: Analysis<EnergyBounds>,
    pub check: HazardCheck,
}

pub fn assess_energy(
    store: Option<&EnergyStore>,
    limit_j: Option<&SourcedRange>,
) -> Result<EnergyResult, ProcessError> {
    if let Some(limit) = limit_j {
        range(limit, "предел запасённой энергии", |x| x >= 0.0)?;
    }
    let Some(store) = store else {
        return Ok(EnergyResult {
            energy: unknown("нет модели накопителя"),
            check: unresolved("нет модели накопителя"),
        });
    };
    let (k, x) = match store {
        EnergyStore::Spring {
            stiffness_n_m,
            deflection_m,
        } => (stiffness_n_m, deflection_m),
        EnergyStore::TorsionSpring {
            stiffness_nm_rad,
            deflection_rad,
        } => (stiffness_nm_rad, deflection_rad),
        EnergyStore::Capacitor {
            capacitance_f,
            voltage_v,
        } => (capacitance_f, voltage_v),
        EnergyStore::Inductor {
            inductance_h,
            current_a,
        } => (inductance_h, current_a),
        EnergyStore::Flywheel {
            inertia_kg_m2,
            speed_rad_s,
        } => (inertia_kg_m2, speed_rad_s),
    };
    range(k, "коэффициент накопителя", |x| {
        x > 0.0
    })?;
    range(x, "координата накопителя", |_| true)?;
    let low = if x.min <= 0.0 && x.max >= 0.0 {
        0.0
    } else {
        x.min.abs().min(x.max.abs())
    };
    let high = x.min.abs().max(x.max.abs());
    let energy = |k, x| 0.5 * k * x * x;
    let bounds = EnergyBounds {
        min_j: energy(k.min, low),
        nominal_j: energy(k.nominal, x.nominal),
        max_j: energy(k.max, high),
    };
    computed(&[bounds.min_j, bounds.nominal_j, bounds.max_j])?;
    let scope = "один линейный накопитель, интервал энергии из пределов параметров";
    let check = match limit_j {
        Some(limit)
            if [&k.provenance, &x.provenance, &limit.provenance]
                .into_iter()
                .all(direct) =>
        {
            if bounds.min_j > limit.max {
                HazardCheck::Violation {
                    reason: format!(
                        "запасено не менее {} Дж при пределе {} Дж",
                        bounds.min_j, limit.max
                    ),
                }
            } else if bounds.max_j < limit.min {
                HazardCheck::WithinLimit {
                    scope: scope.into(),
                }
            } else {
                unresolved("границы энергии перекрываются с пределом")
            }
        }
        _ => unresolved("нет подтверждённого предела или источника параметров энергии"),
    };
    Ok(EnergyResult {
        energy: calculated(bounds, scope),
        check,
    })
}

#[derive(Clone, Debug)]
pub struct EdgeInput {
    pub measured_radius_m: Option<SourcedRange>,
    /// Предел радиуса для проверяемого материала, нагрузки и области контакта.
    pub minimum_radius_m: Option<SourcedRange>,
    pub contact_force_n: Option<SourcedRange>,
    pub calibrated_application: Option<EdgeApplicability>,
    pub scope: String,
}

#[derive(Clone, Debug)]
pub struct EdgeApplicability {
    pub contact_force_n: SourcedRange,
    /// Обоснование предела для материала, формы и контактирующей поверхности.
    pub source: Provenance,
}

pub fn assess_edge(input: &EdgeInput) -> Result<HazardCheck, ProcessError> {
    name(&input.scope)?;
    for limit in [&input.measured_radius_m, &input.minimum_radius_m]
        .into_iter()
        .flatten()
    {
        range(limit, "радиус кромки", |x| x >= 0.0)?;
    }
    if let Some(force) = &input.contact_force_n {
        range(force, "контактная нагрузка на кромку", |x| x >= 0.0)?;
    }
    if let Some(calibration) = &input.calibrated_application {
        source(&calibration.source)?;
        range(
            &calibration.contact_force_n,
            "диапазон нагрузки на кромку",
            |x| x >= 0.0,
        )?;
    }
    match (
        &input.measured_radius_m,
        &input.minimum_radius_m,
        &input.contact_force_n,
        &input.calibrated_application,
    ) {
        (Some(radius), Some(limit), Some(force), Some(calibration))
            if [
                &calibration.source,
                &calibration.contact_force_n.provenance,
                &force.provenance,
                &radius.provenance,
                &limit.provenance,
            ]
            .into_iter()
            .all(direct)
                && force.min >= calibration.contact_force_n.min
                && force.max <= calibration.contact_force_n.max =>
        {
            Ok(if radius.max < limit.min {
                HazardCheck::Violation {
                    reason: format!("радиус кромки меньше допустимого: {}", input.scope),
                }
            } else if radius.min > limit.max {
                HazardCheck::WithinLimit {
                    scope: input.scope.clone(),
                }
            } else {
                unresolved("диапазоны радиуса кромки и порога перекрываются")
            })
        }
        _ => Ok(unresolved(
            "нет замера кромки или применимого критерия под нагрузкой",
        )),
    }
}

fn unresolved(reason: &str) -> HazardCheck {
    HazardCheck::NotAnalyzed {
        reason: reason.into(),
    }
}
