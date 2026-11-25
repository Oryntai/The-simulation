use std::collections::BTreeMap;

use sim_boards::{BoardCatalog, BoardId, board_pins, board_spec, pin_spec};
use sim_process::schematic::{
    PinFunction, PinLimits, PinProfile, ResetProfile, SourcedPinState, ns_bound,
};
use sim_process::{Analysis, Interval, ProcessError};
use sim_signal::Time;
use sim_world::Provenance;

fn invalid(message: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(message.into())
}

/// Разрешает канал АЦП в единственный доступный контакт. У виртуальной
/// платы аналоговые каналы составляют отдельное пространство имён ABI.
pub fn resolve_live_contact(
    board: &str,
    number: u32,
    function: PinFunction,
) -> Result<String, String> {
    let catalog = BoardCatalog::builtin().map_err(|e| e.to_string())?;
    let entry = catalog
        .get(board)
        .ok_or_else(|| format!("неизвестная плата «{board}»"))?;
    if function != PinFunction::Adc {
        resolve_pin_limits(&catalog, board, number, function).map_err(|e| e.to_string())?;
        return Ok(number.to_string());
    }
    if entry.profile == BoardId::Virtual {
        // Число каналов проверяет установщик BoardSim при установке проводки.
        return Ok(format!("adc:{number}"));
    }
    let pins = board_pins(entry.profile)
        .into_iter()
        .filter(|p| !p.internal_only && p.adc_channel.is_some_and(|adc| u32::from(adc) == number))
        .map(|p| p.id)
        .collect::<Vec<_>>();
    match pins.as_slice() {
        [pin] => Ok(pin.to_string()),
        _ => Err(format!(
            "«{board}»: канал АЦП {number} не разрешается в один контакт"
        )),
    }
}

/// Возможности и пределы привязываются к физическому контакту, уже разрешённому установщиком.
pub fn resolve_pin_limits(
    catalog: &BoardCatalog,
    board: &str,
    pin: u32,
    function: PinFunction,
) -> Result<PinLimits, ProcessError> {
    let entry = catalog
        .get(board)
        .ok_or_else(|| invalid(format!("неизвестная плата «{board}»")))?;
    let part = board_spec(entry.profile);
    let spec = pin_spec(entry.profile, pin)
        .ok_or_else(|| invalid(format!("«{board}»: неизвестный контакт {pin}")))?;
    let supported = match function {
        PinFunction::GpioInput | PinFunction::GpioOutput => spec.gpio,
        PinFunction::Adc => spec.adc_channel.is_some(),
        PinFunction::Pwm => spec.pwm,
        PinFunction::Uart => spec.uart,
        PinFunction::I2c => spec.i2c,
        PinFunction::Spi => spec.spi,
    };
    if spec.internal_only || !supported {
        return Err(invalid(format!(
            "«{board}»: контакт {pin} недоступен для {function:?}"
        )));
    }
    // PinSpec даёт абсолютное напряжение только при номинальном питании. Рабочие
    // токи отдельных профилей содержат типовые/оценочные данные и служат диагностикой.
    Ok(PinLimits {
        nominal_supply_v: part.nominal_io_v,
        abs_input_v: spec.abs_input_v.map(|r| [r.min, r.max]),
        specified_source_a: spec.source_ma.map(milliamps),
        specified_sink_a: spec.sink_ma.map(milliamps),
        abs_pin_a: spec.abs_pin_ma.map(milliamps),
        vil_max_v: part.vil_max_v,
        vih_min_v: part.vih_min_v,
        vol_max_v: part.vol_max_v,
        voh_min_v: part.voh_min_v,
        source: part.source.into(),
    })
}

fn milliamps(value: f64) -> f64 {
    // SI: миллиампер равен 10⁻³ А.
    value / 1000.0
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoardElectricalProfile {
    pub board: String,
    pub reset: ResetProfile,
    pub reset_pins: BTreeMap<u32, SourcedPinState>,
}

impl BoardElectricalProfile {
    pub fn validate_builtin(&self) -> Result<(), ProcessError> {
        let catalog = BoardCatalog::builtin().map_err(|error| invalid(error.to_string()))?;
        self.validate(&catalog)
    }

    pub fn validate(&self, catalog: &BoardCatalog) -> Result<(), ProcessError> {
        let entry = catalog
            .get(&self.board)
            .ok_or_else(|| invalid(format!("неизвестная плата «{}»", self.board)))?;
        self.reset.validate()?;
        for (&id, state) in &self.reset_pins {
            if !pin_spec(entry.profile, id).is_some_and(|pin| pin.gpio && !pin.internal_only) {
                return Err(invalid(format!("«{}»: неизвестный GPIO {id}", self.board)));
            }
            let text = match &state.source {
                Provenance::Datasheet(s)
                | Provenance::Paper(s)
                | Provenance::Measured(s)
                | Provenance::Derived(s)
                | Provenance::Estimate(s) => s,
            };
            if text.trim().is_empty() || text.chars().any(char::is_control) {
                return Err(invalid("для GPIO требуется источник состояния при сбросе"));
            }
        }
        Ok(())
    }

    pub fn apply_reset_default(&self, pin: u32, profile: &mut PinProfile) {
        if profile.reset.is_none() {
            profile.reset = self.reset_pins.get(&pin).cloned();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RailEvent {
    Asserted(Time),
    Released(Time),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RailUpdate {
    pub in_reset: bool,
    pub event: Option<RailEvent>,
    /// Момент последнего подтверждённого выхода из сброса; начало загрузки.
    pub boot_since: Option<Time>,
}

/// Длительность подтверждается только сплошной огибающей на всём интервале.
/// Пробы на концах или типовое паспортное значение такую огибающую не заменяют.
#[derive(Clone, Debug)]
pub struct PinRail {
    profile: ResetProfile,
    in_reset: bool,
    last: Option<Time>,
    since: Option<Time>,
    boot_since: Option<Time>,
    ambiguous: bool,
}

impl PinRail {
    pub fn new(profile: ResetProfile, initially_in_reset: bool) -> Result<Self, ProcessError> {
        profile.validate()?;
        Ok(Self {
            profile,
            in_reset: initially_in_reset,
            last: None,
            since: None,
            boot_since: None,
            ambiguous: false,
        })
    }

    pub fn advance(
        &mut self,
        interval: Interval,
        voltage_v: [f64; 2],
    ) -> Result<Analysis<RailUpdate>, ProcessError> {
        let [low, high] = voltage_v;
        if !low.is_finite()
            || !high.is_finite()
            || low > high
            || interval.start >= interval.end
            || self.last.is_some_and(|last| last != interval.start)
        {
            return Err(invalid(
                "питание платы требует конечную огибающую на непрерывных интервалах",
            ));
        }
        let direction = if self.in_reset {
            (&self.profile.rising_v, &self.profile.release_delay_s)
        } else {
            (&self.profile.falling_v, &self.profile.assert_delay_s)
        };
        let confirmed = |parameter: &sim_process::schematic::Parameter| {
            parameter
                .range
                .as_ref()
                .filter(|range| range.can_prove())
                .cloned()
        };
        let unknown = |reason: String| Analysis::NotAnalyzed { reason };
        let (Some(threshold), Some(delay)) = (confirmed(direction.0), confirmed(direction.1))
        else {
            self.last = Some(interval.end);
            self.since = None;
            self.ambiguous = true;
            return Ok(unknown(format!(
                "питание платы: {}; {}",
                direction.0.measurement, direction.1.measurement
            )));
        };
        let (qualifies, excludes) = if self.in_reset {
            (low > threshold.max, high <= threshold.min)
        } else {
            (high < threshold.min, low >= threshold.max)
        };
        let mut event = None;
        if qualifies {
            let since = self.since.unwrap_or(interval.start);
            let earliest = ns_bound(delay.min, false)?;
            let latest = ns_bound(delay.max, true)?;
            let early = since
                .0
                .checked_add(earliest)
                .map(Time)
                .ok_or_else(|| invalid("выдержка сброса вне шкалы времени"))?;
            let late = since
                .0
                .checked_add(latest)
                .map(Time)
                .ok_or_else(|| invalid("выдержка сброса вне шкалы времени"))?;
            if late <= interval.end {
                let was_ambiguous = self.ambiguous;
                self.in_reset = !self.in_reset;
                self.ambiguous = false;
                self.since = None;
                // Точный момент возможен только с точной выдержкой и непрерывной историей.
                if !was_ambiguous && early == late && early >= interval.start {
                    if self.in_reset {
                        self.boot_since = None;
                        event = Some(RailEvent::Asserted(late));
                    } else {
                        self.boot_since = Some(late);
                        event = Some(RailEvent::Released(late));
                    }
                } else {
                    self.boot_since = None;
                    self.last = Some(interval.end);
                    return Ok(unknown(format!(
                        "момент сброса/восстановления ограничен {}–{} нс",
                        early.0, late.0
                    )));
                }
            } else {
                self.since = Some(since);
                self.ambiguous |= early <= interval.end;
            }
        } else {
            self.since = None;
            self.ambiguous |= !excludes;
        }
        self.last = Some(interval.end);
        Ok(if self.ambiguous {
            unknown("пересечение порога или выполнение выдержки не определено огибающей".into())
        } else {
            Analysis::Analyzed {
                value: RailUpdate {
                    in_reset: self.in_reset,
                    event,
                    boot_since: self.boot_since,
                },
                scope: "локальная шина, огибающая напряжения и паспортные выдержки".into(),
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_process::{SourcedRange, schematic::Parameter};

    fn point(value: f64) -> Parameter {
        Parameter {
            range: Some(SourcedRange {
                min: value,
                nominal: value,
                max: value,
                provenance: Provenance::Measured("синтетический стенд с заданным порогом".into()),
            }),
            measurement: "осциллограмма шины и сброса".into(),
        }
    }

    #[test]
    fn rail_rejects_a_short_dip_and_keeps_exact_boot_epoch() {
        // В стенде минимум 2 В, восстановление 3 В и выдержка 10 нс в обе стороны.
        let profile = ResetProfile {
            falling_v: point(2.0),
            rising_v: point(3.0),
            assert_delay_s: point(Time(10).as_secs()),
            release_delay_s: point(Time(10).as_secs()),
        };
        let mut rail = PinRail::new(profile.clone(), false).unwrap();
        let advance = |rail: &mut PinRail, a, b, v| {
            rail.advance(
                Interval {
                    start: Time(a),
                    end: Time(b),
                },
                [v, v],
            )
            .unwrap()
        };
        let event = |result| match result {
            Analysis::Analyzed {
                value: RailUpdate { event, .. },
                ..
            } => event,
            other => panic!("{other:?}"),
        };
        assert_eq!(event(advance(&mut rail, 0, 9, 1.0)), None);
        assert_eq!(event(advance(&mut rail, 9, 20, 4.0)), None);
        assert_eq!(
            event(advance(&mut rail, 20, 32, 1.0)),
            Some(RailEvent::Asserted(Time(30)))
        );
        assert_eq!(event(advance(&mut rail, 32, 40, 2.5)), None);
        let mut fork = rail.clone();
        let after = advance(&mut rail, 40, 55, 4.0);
        assert_eq!(after, advance(&mut fork, 40, 55, 4.0));
        assert_eq!(event(after), Some(RailEvent::Released(Time(50))));
        let mut unknown = PinRail::new(
            ResetProfile {
                assert_delay_s: Parameter {
                    range: None,
                    measurement: "измерить выдержку".into(),
                },
                ..profile
            },
            false,
        )
        .unwrap();
        assert!(matches!(
            advance(&mut unknown, 0, 100, 1.0),
            Analysis::NotAnalyzed { .. }
        ));
        assert!(
            rail.advance(
                Interval {
                    start: Time(55),
                    end: Time(56)
                },
                [0.0, f64::NAN]
            )
            .is_err()
        );
    }
}
