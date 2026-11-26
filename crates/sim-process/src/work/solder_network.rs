//! Тепловая сеть пайки с границей по температуре полигона.

use sim_signal::{Rng, Time};

use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::super::math::{calculated, computed, continuous, execution, finite, invalid, unknown};
use super::ThermalResponse;

const TIP: usize = 0;
const PAD: usize = 1;
const LEAD: usize = 2;
const COPPER: usize = 3;
const POLYGON: usize = 4;
const NODES: usize = POLYGON + 1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolderNodes<T> {
    pub tip: T,
    pub pad: T,
    pub lead: T,
    pub copper: T,
}

impl<T> SolderNodes<T> {
    fn as_array(&self) -> [&T; 4] {
        [&self.tip, &self.pad, &self.lead, &self.copper]
    }
}

impl<T> From<[T; 4]> for SolderNodes<T> {
    fn from([tip, pad, lead, copper]: [T; 4]) -> Self {
        Self {
            tip,
            pad,
            lead,
            copper,
        }
    }
}

/// Для расчёта нужны точечные параметры с проверяемыми источниками.
/// `None` обозначает пропуск, явно заданный ноль размыкает тепловую ветвь.
#[derive(Clone, Debug, PartialEq)]
pub struct SolderNetworkSpec {
    pub capacity_j_k: SolderNodes<Option<SourcedRange>>,
    pub initial_k: SolderNodes<Option<SourcedRange>>,
    pub pad_lead_w_k: Option<SourcedRange>,
    pub pad_copper_w_k: Option<SourcedRange>,
    pub copper_polygon_w_k: Option<SourcedRange>,
    /// Температура граничного резервуара постоянна весь прогон.
    pub polygon_k: Option<SourcedRange>,
    /// Верхний предел тепловой мощности, поступающей в жало.
    pub max_power_w: Option<SourcedRange>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SolderContact {
    Separated,
    Touching {
        conductance_w_k: Option<SourcedRange>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderNetworkInput {
    /// Контакт жала с площадкой и его проводимость постоянны в пределах шага.
    pub contact: Option<SolderContact>,
    /// Запрошенная тепловая мощность удерживается на всём шаге.
    pub requested_power_w: Option<SourcedRange>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolderNetworkStep {
    pub start_k: SolderNodes<f64>,
    pub end_k: SolderNodes<f64>,
    pub applied_power_w: f64,
    pub supplied_j: f64,
    /// Интеграл теплового потока из меди в полигон за шаг, со знаком.
    pub to_polygon_j: f64,
}

/// Связи жало–площадка, площадка–вывод, площадка–медь и медь–полигон.
/// Тепловые истории с пропуском остаются открытыми до нового прогона.
#[derive(Clone, Debug)]
pub struct SolderNetwork {
    state: Result<ThermalState, String>,
    last: Option<Time>,
}

#[derive(Clone, Debug)]
struct ThermalState {
    capacity_j_k: [f64; 4],
    rates: [[f64; NODES]; NODES],
    polygon_k: f64,
    max_power_w: f64,
    /// Энергии узлов относительно температуры полигона. Накопитель полигона
    /// обнуляется при каждом шаге, поэтому результат содержит отвод за шаг.
    energy_j: [f64; NODES],
}

impl SolderNetwork {
    pub fn new(spec: SolderNetworkSpec) -> Result<Self, ProcessError> {
        let mut unresolved = None;
        let mut check =
            |value, name, zero_allowed| parameter(value, name, zero_allowed, &mut unresolved);
        let [tip_c, pad_c, lead_c, copper_c] = spec.capacity_j_k.as_array();
        let capacities = [
            check(tip_c, "теплоёмкость жала", false)?,
            check(pad_c, "теплоёмкость площадки", false)?,
            check(lead_c, "теплоёмкость вывода", false)?,
            check(copper_c, "теплоёмкость меди", false)?,
        ];
        let [tip_k, pad_k, lead_k, copper_k] = spec.initial_k.as_array();
        let temperatures = [
            check(tip_k, "начальная температура жала", false)?,
            check(pad_k, "начальная температура площадки", false)?,
            check(lead_k, "начальная температура вывода", false)?,
            check(copper_k, "начальная температура меди", false)?,
        ];
        let pad_lead = check(&spec.pad_lead_w_k, "проводимость площадка–вывод", true)?;
        let pad_copper = check(&spec.pad_copper_w_k, "проводимость площадка–медь", true)?;
        let copper_polygon = check(&spec.copper_polygon_w_k, "проводимость медь–полигон", true)?;
        let polygon = check(&spec.polygon_k, "температура полигона", false)?;
        let max_power = check(&spec.max_power_w, "предел мощности жала", true)?;

        let state = if let Some(reason) = unresolved {
            Err(reason)
        } else {
            let capacity_j_k = capacities.map(Option::unwrap);
            let polygon_k = polygon.unwrap();
            let mut state = ThermalState {
                capacity_j_k,
                rates: [[0.0; NODES]; NODES],
                polygon_k,
                max_power_w: max_power.unwrap(),
                energy_j: [0.0; NODES],
            };
            for (i, temperature) in temperatures.into_iter().enumerate() {
                let offset = temperature.unwrap() - polygon_k;
                state.energy_j[i] = capacity_j_k[i] * offset;
                if state.energy_j[i] == 0.0 && offset != 0.0 {
                    return Err(execution("начальная тепловая энергия не представима"));
                }
            }
            computed(&state.energy_j)?;
            state.rates[LEAD][PAD] = rate(pad_lead.unwrap(), capacity_j_k[PAD])?;
            state.rates[PAD][LEAD] = rate(pad_lead.unwrap(), capacity_j_k[LEAD])?;
            state.rates[COPPER][PAD] = rate(pad_copper.unwrap(), capacity_j_k[PAD])?;
            state.rates[PAD][COPPER] = rate(pad_copper.unwrap(), capacity_j_k[COPPER])?;
            state.rates[POLYGON][COPPER] = rate(copper_polygon.unwrap(), capacity_j_k[COPPER])?;
            Ok(state)
        };
        Ok(Self { state, last: None })
    }
}

impl StepModel for SolderNetwork {
    type Input = SolderNetworkInput;
    type Output = Analysis<SolderNetworkStep>;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        let dt = continuous(self.last, interval)?;
        let mut unresolved = None;
        let contact = match &input.contact {
            Some(SolderContact::Separated) => Some(0.0),
            Some(SolderContact::Touching { conductance_w_k }) => parameter(
                conductance_w_k,
                "проводимость жало–площадка",
                false,
                &mut unresolved,
            )?,
            None => {
                unresolved = Some("нет наблюдения контакта жала с площадкой".into());
                None
            }
        };
        let power = parameter(
            &input.requested_power_w,
            "запрошенная мощность жала",
            true,
            &mut unresolved,
        )?;
        let result = match (&mut self.state, contact, power) {
            (Ok(state), Some(contact), Some(power)) => calculated(
                state.advance(dt, contact, power)?,
                "точечная модель четырёх теплоёмкостей с постоянными входами шага",
            ),
            (Err(reason), _, _) => unknown(reason),
            (Ok(_), _, _) => {
                let reason = unresolved.unwrap();
                self.state = Err(format!("тепловая история прервана: {reason}"));
                unknown(&reason)
            }
        };
        self.last = Some(interval.end);
        Ok(result)
    }
}

impl ThermalState {
    fn advance(
        &mut self,
        dt: f64,
        contact: f64,
        requested_power: f64,
    ) -> Result<SolderNetworkStep, ProcessError> {
        let mut rates = self.rates;
        rates[PAD][TIP] = rate(contact, self.capacity_j_k[TIP])?;
        rates[TIP][PAD] = rate(contact, self.capacity_j_k[PAD])?;
        let applied_power_w = requested_power.min(self.max_power_w);
        let supplied_j = applied_power_w * dt;
        computed(&[supplied_j])?;
        if supplied_j == 0.0 && applied_power_w > 0.0 {
            return Err(execution("поданная тепловая энергия не представима"));
        }
        let response = ThermalResponse::new(&rates, TIP, dt)?;
        let mut end_energy = response.apply(self.energy_j, supplied_j);
        computed(&end_energy)?;
        let start_k = self.temperatures(&self.energy_j)?;
        let end_k = self.temperatures(&end_energy)?;
        let to_polygon_j = end_energy[POLYGON];
        end_energy[POLYGON] = 0.0;
        self.energy_j = end_energy;
        Ok(SolderNetworkStep {
            start_k,
            end_k,
            applied_power_w,
            supplied_j,
            to_polygon_j,
        })
    }

    fn temperatures(&self, energy_j: &[f64; NODES]) -> Result<SolderNodes<f64>, ProcessError> {
        let values = std::array::from_fn(|i| self.polygon_k + energy_j[i] / self.capacity_j_k[i]);
        computed(&values)?;
        if values.iter().any(|temperature| *temperature <= 0.0) {
            return Err(execution(
                "температура вышла из представимого физического диапазона",
            ));
        }
        Ok(values.into())
    }
}

fn rate(conductance_w_k: f64, capacity_j_k: f64) -> Result<f64, ProcessError> {
    let rate = conductance_w_k / capacity_j_k;
    computed(&[rate])?;
    if rate == 0.0 && conductance_w_k > 0.0 {
        return Err(execution("скорость теплового перехода не представима"));
    }
    Ok(rate)
}

fn parameter(
    value: &Option<SourcedRange>,
    label: &str,
    zero_allowed: bool,
    unresolved: &mut Option<String>,
) -> Result<Option<f64>, ProcessError> {
    let reason = if let Some(value) = value {
        finite(&[value.min, value.nominal, value.max])?;
        if value.min < 0.0
            || (!zero_allowed && value.min == 0.0)
            || value.min > value.nominal
            || value.nominal > value.max
        {
            return Err(invalid(format!(
                "некорректный диапазон параметра «{label}»"
            )));
        }
        if !value.can_prove() {
            "нет подтверждённого источника"
        } else if value.min != value.max {
            "огибающая для диапазона не рассчитана"
        } else {
            return Ok(Some(value.nominal));
        }
    } else {
        "нет значения"
    };
    unresolved.get_or_insert_with(|| format!("{label}: {reason}"));
    Ok(None)
}
