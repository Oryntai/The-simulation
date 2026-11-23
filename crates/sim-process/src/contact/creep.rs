use std::collections::BTreeMap;

use sim_signal::{Rate, Time};
use sim_world::{ContactPair, ContactSolver, Provenance};

use super::CreepReport;
use super::table::{invalid, name, range, source, traceable};
use crate::{Analysis, Interval, ProcessError, SourcedRange};

/// Аналитическая опора: плоское прямоугольное пятно, постоянная нагрузка, сухое
/// изотропное трение. Положительная Z — нормаль, X и Y — оси прямоугольника.
#[derive(Clone, Debug, PartialEq)]
pub struct RigidPlaneSupport {
    /// Отпечаток геометрии, материалов, начала опыта и схемы нагружения.
    pub fixture: String,
    pub provenance: Provenance,
    pub half_extent_m: [f64; 2],
    /// Сила опоры, требуемая для статического равновесия.
    pub required_force_n: [f64; 3],
    /// Момент опоры относительно центра прямоугольника.
    pub required_moment_nm: [f64; 3],
    pub friction: SourcedRange,
    /// Допуск независимого баланса моментов.
    pub moment_residual_nm: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RigidBalance {
    pub center_of_pressure_m: [f64; 2],
    pub torsion_residual_nm: f64,
    /// Запас при минимальном и максимальном коэффициентах в таблице.
    pub reserve_n: [f64; 2],
    pub mode: RigidMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidMode {
    WithinCone,
    OutsideCone,
}

impl RigidPlaneSupport {
    pub fn balance(&self) -> Result<Analysis<RigidBalance>, ProcessError> {
        source(&self.provenance)?;
        range(&self.friction, |x| x >= sim_world::MIN_CONTACT_FRICTION)?;
        range(&self.moment_residual_nm, |x| x >= 0.0)?;
        if !name(&self.fixture)
            || self.required_force_n[2] <= 0.0
            || !self.half_extent_m.iter().all(|x| x.is_finite() && *x > 0.0)
            || !self
                .required_force_n
                .iter()
                .chain(&self.required_moment_nm)
                .all(|x| x.is_finite())
        {
            return Err(invalid("неверная геометрия опоры или нагрузка"));
        }
        if ![
            &self.provenance,
            &self.friction.provenance,
            &self.moment_residual_nm.provenance,
        ]
        .into_iter()
        .all(traceable)
        {
            return Ok(unavailable(
                "не подтверждён источник геометрии, нагрузки или трения",
            ));
        }
        let [fx, fy, fz] = self.required_force_n;
        let [mx, my, mz] = self.required_moment_nm;
        let pressure = [-my / fz, mx / fz];
        let tangent = fx.hypot(fy);
        let reserve = [
            self.friction.min * fz - tangent,
            self.friction.max * fz - tangent,
        ];
        // Распределяем N по четырём углам билинейными весами. Их сумма опирает Fz,
        // центр давления даёт Mx и My. Касательные силы с теми же весами
        // удовлетворяют конусу в каждом углу и дают Mz = x*Fy − y*Fx.
        let residual = (mz - (pressure[0] * fy - pressure[1] * fx)).abs();
        if !pressure
            .into_iter()
            .chain(reserve)
            .chain([residual])
            .all(f64::is_finite)
        {
            return Err(invalid("переполнение баланса опоры"));
        }
        if pressure
            .iter()
            .zip(self.half_extent_m)
            .any(|(p, edge)| p.abs() >= edge)
        {
            return Ok(unavailable(
                "центр давления вышел на границу или за границу опоры",
            ));
        }
        // Пара равных противоположных касательных добавок на углах
        // компенсирует остаточный Mz. Каждая добавка ограничена свободным
        // радиусом конуса этого угла. Выбираем одну поперечную пару направлений.
        let torsion_capacity = reserve[0]
            * (self.half_extent_m[0] - pressure[0].abs())
                .max(self.half_extent_m[1] - pressure[1].abs());
        if !torsion_capacity.is_finite() {
            return Err(invalid("переполнение расчёта запаса момента"));
        }
        let mode = if reserve[1] < 0.0 {
            RigidMode::OutsideCone
        } else if reserve[0] > 0.0 && residual + self.moment_residual_nm.max < torsion_capacity {
            RigidMode::WithinCone
        } else {
            return Ok(unavailable(
                "граница конуса или не подтверждено равновесие моментов",
            ));
        };
        Ok(Analysis::Analyzed {
            value: RigidBalance {
                center_of_pressure_m: pressure,
                torsion_residual_nm: residual,
                reserve_n: reserve,
                mode,
            },
            scope: "жёсткая прямоугольная опора с сухим изотропным трением".into(),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CreepVariant {
    Baseline,
    HalfStep,
    QuarterStep,
    Reference,
    Impedance,
    Cone,
    ImpedanceRatio,
    Tolerance,
    Iterations,
    NoSlip,
    OutsideCone,
}

const VARIANTS: [CreepVariant; 11] = [
    CreepVariant::Baseline,
    CreepVariant::HalfStep,
    CreepVariant::QuarterStep,
    CreepVariant::Reference,
    CreepVariant::Impedance,
    CreepVariant::Cone,
    CreepVariant::ImpedanceRatio,
    CreepVariant::Tolerance,
    CreepVariant::Iterations,
    CreepVariant::NoSlip,
    CreepVariant::OutsideCone,
];

#[derive(Clone, Debug, PartialEq)]
pub struct CreepConfiguration {
    pub engine_version: String,
    pub rate: Rate,
    pub solver: ContactSolver,
    pub pair: ContactPair,
    /// Источник выбранных численных параметров и настройки опыта.
    pub source: Provenance,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldSample {
    pub at: Time,
    /// Относительная позиция пары в плоскости опоры.
    pub tangent_position_m: [f64; 2],
    pub tangent_velocity_m_s: [f64; 2],
    pub contacts: usize,
    pub normal_reaction_n: f64,
    pub max_penetration_m: f64,
    /// Сумма касательных контактных сил и внешней нагрузки.
    pub net_tangent_force_n: [f64; 2],
    /// Модуль суммарного момента относительно центра масс с внешней нагрузкой.
    pub net_moment_nm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CreepMetrics {
    pub final_drift_m: f64,
    pub peak_drift_m: f64,
    pub final_speed_m_s: f64,
    pub final_velocity_m_s: [f64; 2],
    pub peak_speed_m_s: f64,
    pub final_normal_n: f64,
    pub normal_range_n: [f64; 2],
    pub peak_penetration_m: f64,
    pub normal_impulse_n_s: f64,
    pub contact_time_s: f64,
    pub peak_tangent_imbalance_n: f64,
    pub peak_net_moment_nm: f64,
    pub min_friction_reserve_n: f64,
    pub continuously_supported: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CreepTrial {
    variant: CreepVariant,
    config: CreepConfiguration,
    support: RigidPlaneSupport,
    hold: Interval,
    metrics: CreepMetrics,
}

impl CreepTrial {
    pub fn variant(&self) -> CreepVariant {
        self.variant
    }
    pub fn configuration(&self) -> &CreepConfiguration {
        &self.config
    }
    pub fn support(&self) -> &RigidPlaneSupport {
        &self.support
    }
    pub fn metrics(&self) -> &CreepMetrics {
        &self.metrics
    }
    pub fn hold(&self) -> Interval {
        self.hold
    }
}

pub struct CreepTrace {
    trial: CreepTrial,
    origin: [f64; 2],
    tick: u64,
    last_tick: u64,
}

// Длительность выдержки по ТЗ, § 4.6 и плану этапа 4.
const HOLD_S: u64 = 2;
const NS_PER_S: u64 = 1_000_000_000;

impl CreepTrace {
    /// Проверяет временную сетку и принимает отсчёт после установления.
    /// Все последующие такты выдержки должны присутствовать в журнале.
    pub fn new(
        variant: CreepVariant,
        config: CreepConfiguration,
        support: RigidPlaneSupport,
        start_tick: u64,
        start: HoldSample,
    ) -> Result<Self, ProcessError> {
        source(&config.source)?;
        support.balance()?;
        config.solver.validate().map_err(invalid)?;
        config
            .pair
            .validate_step(config.rate, config.solver.cone)
            .map_err(invalid)?;
        if config.engine_version != "4.0.1" {
            return Err(invalid("замер контакта проверен для MuJoCo 4.0.1"));
        }
        if config.pair.settings.dim != 3
            || config.pair.settings.friction[..2] != [support.friction.nominal; 2]
        {
            return Err(invalid(
                "опыт требует плоскую опору с изотропной парой скольжения",
            ));
        }
        // Для этой серии поддержаны целые Гц, включая сетки с дробным числом
        // наносекунд за такт. Считаем время через u128 до вызова Rate::tick.
        let hz = config.rate.as_hz() as u64;
        let last_tick = start_tick
            .checked_add(hz * HOLD_S)
            .ok_or_else(|| invalid("переполнение номера такта"))?;
        if Some(config.rate) != Rate::hz(hz)
            || start_tick == 0
            || u128::from(last_tick) * u128::from(NS_PER_S) / u128::from(hz) > u128::from(u64::MAX)
            || config.rate.tick(start_tick) != start.at
        {
            return Err(invalid("начало выдержки выпало из сетки опыта"));
        }
        let hold = Interval {
            start: start.at,
            end: config.rate.tick(last_tick),
        };
        let metrics = sample_metrics(start, start.tangent_position_m, &support)?;
        Ok(Self {
            trial: CreepTrial {
                variant,
                config,
                support,
                hold,
                metrics,
            },
            origin: start.tangent_position_m,
            tick: start_tick,
            last_tick,
        })
    }

    pub fn record(&mut self, sample: HoldSample) -> Result<(), ProcessError> {
        if self.tick == self.last_tick || sample.at != self.trial.config.rate.tick(self.tick + 1) {
            return Err(invalid("пропуск, дубль или лишний отсчёт в выдержке"));
        }
        let next = sample_metrics(sample, self.origin, &self.trial.support)?;
        let m = &mut self.trial.metrics;
        let dt = Time(sample.at.0 - self.trial.config.rate.tick(self.tick).0).as_secs();
        let impulse = m.normal_impulse_n_s + next.final_normal_n * dt;
        if !impulse.is_finite() {
            return Err(invalid("переполнение интеграла нормальной реакции"));
        }
        m.final_drift_m = next.final_drift_m;
        m.peak_drift_m = m.peak_drift_m.max(next.peak_drift_m);
        m.final_speed_m_s = next.final_speed_m_s;
        m.final_velocity_m_s = next.final_velocity_m_s;
        m.peak_speed_m_s = m.peak_speed_m_s.max(next.peak_speed_m_s);
        m.final_normal_n = next.final_normal_n;
        m.normal_range_n[0] = m.normal_range_n[0].min(next.normal_range_n[0]);
        m.normal_range_n[1] = m.normal_range_n[1].max(next.normal_range_n[1]);
        m.peak_penetration_m = m.peak_penetration_m.max(next.peak_penetration_m);
        m.normal_impulse_n_s = impulse;
        if next.continuously_supported {
            m.contact_time_s += dt;
        }
        m.peak_tangent_imbalance_n = m
            .peak_tangent_imbalance_n
            .max(next.peak_tangent_imbalance_n);
        m.peak_net_moment_nm = m.peak_net_moment_nm.max(next.peak_net_moment_nm);
        m.min_friction_reserve_n = m.min_friction_reserve_n.min(next.min_friction_reserve_n);
        m.continuously_supported &= next.continuously_supported;
        self.tick += 1;
        Ok(())
    }

    pub fn finish(self) -> Result<CreepTrial, ProcessError> {
        if self.tick != self.last_tick {
            return Err(invalid("неполные две секунды выдержки"));
        }
        Ok(self.trial)
    }
}

fn sample_metrics(
    s: HoldSample,
    origin: [f64; 2],
    plane: &RigidPlaneSupport,
) -> Result<CreepMetrics, ProcessError> {
    let [x, y] = s.tangent_position_m;
    let drift = (x - origin[0]).hypot(y - origin[1]);
    let speed = s.tangent_velocity_m_s[0].hypot(s.tangent_velocity_m_s[1]);
    let [fx, fy, _] = plane.required_force_n;
    let reserve = plane.friction.min * s.normal_reaction_n - fx.hypot(fy);
    let tangent_imbalance = s.net_tangent_force_n[0].hypot(s.net_tangent_force_n[1]);
    if ![drift, speed, reserve, tangent_imbalance]
        .into_iter()
        .all(f64::is_finite)
        || ![s.normal_reaction_n, s.max_penetration_m, s.net_moment_nm]
            .into_iter()
            .all(|x| x.is_finite() && x >= 0.0)
        || (s.contacts == 0 && (s.normal_reaction_n != 0.0 || s.max_penetration_m != 0.0))
    {
        return Err(invalid("недопустимый или неконечный отсчёт выдержки"));
    }
    Ok(CreepMetrics {
        final_drift_m: drift,
        peak_drift_m: drift,
        final_speed_m_s: speed,
        peak_speed_m_s: speed,
        final_velocity_m_s: s.tangent_velocity_m_s,
        final_normal_n: s.normal_reaction_n,
        normal_range_n: [s.normal_reaction_n; 2],
        peak_penetration_m: s.max_penetration_m,
        normal_impulse_n_s: 0.0,
        contact_time_s: 0.0,
        peak_net_moment_nm: s.net_moment_nm,
        peak_tangent_imbalance_n: tangent_imbalance,
        min_friction_reserve_n: reserve,
        continuously_supported: s.contacts != 0 && s.normal_reaction_n > 0.0,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct CreepTolerances {
    pub holding_speed_m_s: SourcedRange,
    pub normal_balance_n: SourcedRange,
    pub tangent_balance_n: SourcedRange,
    pub moment_balance_nm: SourcedRange,
    pub step_displacement_m: SourcedRange,
    pub step_speed_m_s: SourcedRange,
    pub step_force_n: SourcedRange,
    pub step_penetration_m: SourcedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LimitComparison {
    BelowLimit,
    ExceedsLimit,
}

fn unavailable<T>(reason: &str) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

fn same_setup(
    variant: CreepVariant,
    trial: &CreepTrial,
    base: &CreepTrial,
    fine: &CreepTrial,
) -> bool {
    use CreepVariant::*;
    let (template, factor) = match variant {
        Baseline => (base, 1),
        HalfStep => (base, 2),
        QuarterStep => (base, 4),
        _ => (fine, 1),
    };
    let rate = Rate::hz(template.config.rate.as_hz() as u64 * factor);
    if trial.hold != base.hold
        || Some(trial.config.rate) != rate
        || trial.config.engine_version != base.config.engine_version
        || (variant != OutsideCone && trial.support != base.support)
    {
        return false;
    }
    let mut pair = template.config.pair.clone();
    let mut solver = template.config.solver;
    match variant {
        Reference => pair.settings.solref = trial.config.pair.settings.solref,
        Impedance => pair.settings.solimp = trial.config.pair.settings.solimp,
        Cone => solver.cone = trial.config.solver.cone,
        ImpedanceRatio => solver.impratio = trial.config.solver.impratio,
        Tolerance => solver.tolerance = trial.config.solver.tolerance,
        Iterations => solver.iterations = trial.config.solver.iterations,
        NoSlip => solver.noslip_iterations = trial.config.solver.noslip_iterations,
        _ => {}
    }
    pair == trial.config.pair
        && solver == trial.config.solver
        && match variant {
            NoSlip => solver.noslip_iterations > 0,
            Reference | Impedance | Cone | ImpedanceRatio | Tolerance | Iterations => {
                pair != template.config.pair || solver != template.config.solver
            }
            _ => true,
        }
}

fn supports(trial: &CreepTrial, t: &CreepTolerances) -> bool {
    let m = &trial.metrics;
    let normal = trial.support.required_force_n[2];
    m.continuously_supported
        && m.min_friction_reserve_n > 0.0
        && m.peak_speed_m_s <= t.holding_speed_m_s.min
        && m.peak_tangent_imbalance_n <= t.tangent_balance_n.min
        && m.peak_net_moment_nm <= t.moment_balance_nm.min
        && m.normal_range_n
            .into_iter()
            .all(|n| (n - normal).abs() <= t.normal_balance_n.min)
}

fn differences(a: &CreepMetrics, b: &CreepMetrics) -> [f64; 4] {
    [
        (a.final_drift_m - b.final_drift_m)
            .abs()
            .max((a.peak_drift_m - b.peak_drift_m).abs()),
        (a.final_speed_m_s - b.final_speed_m_s)
            .abs()
            .max((a.peak_speed_m_s - b.peak_speed_m_s).abs()),
        (a.final_normal_n - b.final_normal_n)
            .abs()
            .max((a.normal_range_n[0] - b.normal_range_n[0]).abs())
            .max((a.normal_range_n[1] - b.normal_range_n[1]).abs()),
        (a.peak_penetration_m - b.peak_penetration_m).abs(),
    ]
}

impl CreepReport {
    pub fn from_trials(
        trials: Vec<CreepTrial>,
        tolerances: &CreepTolerances,
    ) -> Result<Self, ProcessError> {
        use CreepVariant::*;
        let limits = [
            &tolerances.holding_speed_m_s,
            &tolerances.normal_balance_n,
            &tolerances.tangent_balance_n,
            &tolerances.moment_balance_nm,
            &tolerances.step_displacement_m,
            &tolerances.step_speed_m_s,
            &tolerances.step_force_n,
            &tolerances.step_penetration_m,
        ];
        for limit in limits {
            range(limit, |x| x >= 0.0)?;
        }
        let mut indexed = BTreeMap::new();
        for trial in trials {
            if indexed.insert(trial.variant, trial).is_some() {
                return Err(invalid("повтор опыта ползучести"));
            }
        }
        let fine = indexed
            .get(&QuarterStep)
            .ok_or_else(|| invalid("нет опыта на самой мелкой сетке"))?;
        let bound = (|| {
            if !limits.iter().all(|x| traceable(&x.provenance))
                || indexed.values().any(|x| !traceable(&x.config.source))
            {
                return unavailable("источник допуска или конфигурации не подтверждён");
            }
            if VARIANTS.iter().any(|v| !indexed.contains_key(v)) {
                return unavailable("неполная серия ползучести");
            }
            let base = &indexed[&Baseline];
            if base.config.solver.noslip_iterations != 0
                || VARIANTS
                    .iter()
                    .any(|v| !same_setup(*v, &indexed[v], base, fine))
            {
                return unavailable("опыты меняют несколько факторов или окно выдержки");
            }
            let inside = base.support.balance();
            let outside = &indexed[&OutsideCone];
            let [fx, fy, fz] = outside.support.required_force_n;
            let [vx, vy] = outside.metrics.final_velocity_m_s;
            let tangential_load = fx.hypot(fy);
            let slip_speed = vx * (-fx / tangential_load) + vy * (-fy / tangential_load);
            // Режим срыва может включать потерю опоры на отдельных тактах.
            // Требуем контакт в опыте и баланс нормального импульса в окне.
            let mean_normal = outside.metrics.normal_impulse_n_s / HOLD_S as f64;
            let mode = |r: Result<Analysis<RigidBalance>, ProcessError>, expected| matches!(r, Ok(Analysis::Analyzed { value, .. }) if value.mode == expected);
            if !mode(inside, RigidMode::WithinCone)
                || !mode(outside.support.balance(), RigidMode::OutsideCone)
                || outside.support.fixture != base.support.fixture
                || outside.support.half_extent_m != base.support.half_extent_m
                || outside.support.friction != base.support.friction
                || outside.support.required_force_n[2] != base.support.required_force_n[2]
                || outside.metrics.contact_time_s <= 0.0
                || (mean_normal - fz).abs() > tolerances.normal_balance_n.min
                || !slip_speed.is_finite()
                || slip_speed <= tolerances.holding_speed_m_s.min
            {
                return unavailable("не подтверждён жёсткий предел или контрольный срыв");
            }
            if base.support.friction.min != base.support.friction.max {
                return unavailable("замер при номинальном трении не покрывает диапазон пары");
            }
            if indexed
                .values()
                .filter(|x| x.variant != OutsideCone)
                .any(|x| !supports(x, tolerances))
            {
                return unavailable("изменился режим удержания или нарушен баланс реакций");
            }
            let half = &indexed[&HalfStep];
            let first = differences(&base.metrics, &half.metrics);
            let second = differences(&half.metrics, &fine.metrics);
            let allowed = [
                tolerances.step_displacement_m.min,
                tolerances.step_speed_m_s.min,
                tolerances.step_force_n.min,
                tolerances.step_penetration_m.min,
            ];
            if (0..allowed.len()).any(|i| first[i] > allowed[i] || second[i] > allowed[i]) {
                return unavailable("две соседние сетки не укладываются в допуски сходимости");
            }
            // Жёсткая равновесная модель даёт нулевое скольжение. Максимум
            // по всем измеренным точкам и вариантам покрывает весь наблюдавшийся
            // разброс относительно нуля. Добавляем наибольшую разность сеток.
            let peak = indexed
                .values()
                .filter(|x| x.variant != OutsideCone)
                .map(|x| x.metrics.peak_drift_m)
                .fold(0.0, f64::max);
            let bound = peak + first[0].max(second[0]);
            if !bound.is_finite() {
                return unavailable("переполнение оценки погрешности");
            }
            Analysis::Analyzed {
                value: bound,
                scope:
                    "замер выдержки и запас по двум разностям сеток для указанной опоры и нагрузки"
                        .into(),
            }
        })();
        Ok(Self {
            hold: fine.hold,
            drift_m: fine.metrics.final_drift_m,
            peak_drift_m: fine.metrics.peak_drift_m,
            numerical_uncertainty_m: bound,
            tolerances: tolerances.clone(),
            trials: indexed,
        })
    }

    pub fn trials(&self) -> impl Iterator<Item = &CreepTrial> {
        self.trials.values()
    }
    pub fn tolerances(&self) -> &CreepTolerances {
        &self.tolerances
    }
    pub fn numerical_uncertainty_m(&self) -> &Analysis<f64> {
        &self.numerical_uncertainty_m
    }

    /// Расширяет прогноз по модулю смещения для изученной конфигурации.
    /// Порог сохраняется. Неизвестная область или пересечение дают NotAnalyzed.
    pub fn compare_displacement_limit(
        &self,
        config: &CreepConfiguration,
        support: &RigidPlaneSupport,
        predicted_m: [f64; 2],
        limit_m: &SourcedRange,
    ) -> Result<Analysis<([f64; 2], LimitComparison)>, ProcessError> {
        range(limit_m, |x| x >= 0.0)?;
        if !predicted_m.iter().all(|x| x.is_finite() && *x >= 0.0)
            || predicted_m[0] > predicted_m[1]
        {
            return Err(invalid("недопустимый интервал прогнозируемого смещения"));
        }
        let fine = &self.trials[&CreepVariant::QuarterStep];
        let Analysis::Analyzed {
            value: bound,
            scope,
        } = &self.numerical_uncertainty_m
        else {
            return Ok(unavailable("граница численной погрешности недоступна"));
        };
        if config != &fine.config || support != &fine.support || !traceable(&limit_m.provenance) {
            return Ok(unavailable(
                "нет границы для конфигурации, нагрузки или происхождения порога",
            ));
        }
        let expanded = [(predicted_m[0] - bound).max(0.0), predicted_m[1] + bound];
        if !expanded[1].is_finite() {
            return Ok(unavailable("переполнение границы прогнозируемого смещения"));
        }
        let comparison = if expanded[1] < limit_m.min {
            LimitComparison::BelowLimit
        } else if expanded[0] > limit_m.max {
            LimitComparison::ExceedsLimit
        } else {
            return Ok(unavailable("расширенный интервал пересекает порог"));
        };
        Ok(Analysis::Analyzed {
            value: (expanded, comparison),
            scope: scope.clone(),
        })
    }
}
