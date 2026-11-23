use crate::{BoardId, SupplyRail, board_spec, pin_spec, supply_range};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

/// Каждая запись описывает одно ограничение или участника общей цепи.
/// Напряжения и токи задаются в худшем режиме, включая пуск.
#[derive(Clone, Copy, Debug)]
pub enum Connection {
    Supply {
        rail: SupplyRail,
        volts: f64,
    },
    Ground {
        shared: Option<bool>,
    },
    DigitalInput {
        pin: u32,
        low_v: f64,
        high_v: f64,
    },
    AnalogInput {
        pin: u32,
        volts: f64,
        reference_v: Option<f64>,
    },
    /// Все активные двухтактные выходы одной цепи задаются с одинаковым net.
    Output {
        pin: u32,
        net: u32,
    },
    ExternalOutput {
        net: u32,
    },
    Load {
        pin: u32,
        milliamps: f64,
        sinking: bool,
    },
    Led {
        pin: u32,
        resistance_ohm: Option<f64>,
        min_forward_v: f64,
        max_drive_v: f64,
        sinking: bool,
    },
    Motor {
        pin: u32,
        direct: bool,
    },
    Strap {
        pin: u32,
        compatible_at_reset: Option<bool>,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WiringRule {
    Numeric,
    Pin,
    Supply,
    Ground,
    Logic,
    Overvoltage,
    AdcRange,
    Contention,
    PinCurrent,
    TotalCurrent,
    LedResistor,
    DirectMotor,
    Strap,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WiringFinding {
    pub connection: Option<usize>,
    pub pin: Option<u32>,
    pub rule: WiringRule,
    pub outcome: Outcome,
    pub message: &'static str,
}

pub fn check_wiring(board: BoardId, connections: &[Connection]) -> Vec<WiringFinding> {
    use Connection::*;
    use Outcome::*;
    use WiringRule as Rule;

    let mut out = Vec::new();
    let virtual_board = board == BoardId::Virtual;
    let spec = board_spec(board);
    let mut nets = BTreeMap::new();
    let mut rails = BTreeMap::new();
    let mut currents = BTreeMap::<(u32, bool), f64>::new();
    let mcu_supply = connections.iter().find_map(|c| match *c {
        Supply {
            rail: SupplyRail::Mcu,
            volts,
        } if spec.mcu_supply_v.contains(volts) => Some(volts),
        _ => None,
    });
    let mut ground = false;
    let mut power = false;
    let mut incomplete_current = false;

    for (i, &c) in connections.iter().enumerate() {
        let pin = match c {
            DigitalInput { pin, .. }
            | AnalogInput { pin, .. }
            | Output { pin, .. }
            | Load { pin, .. }
            | Led { pin, .. }
            | Motor { pin, .. }
            | Strap { pin, .. } => Some(pin),
            _ => None,
        };
        let mut add = |rule, outcome, message| {
            out.push(WiringFinding {
                connection: Some(i),
                pin,
                rule,
                outcome,
                message,
            })
        };

        let finite = match c {
            Supply { volts, .. } => volts.is_finite(),
            DigitalInput { low_v, high_v, .. } => low_v.is_finite() && high_v.is_finite(),
            AnalogInput {
                volts, reference_v, ..
            } => volts.is_finite() && reference_v.is_none_or(f64::is_finite),
            Load { milliamps, .. } => milliamps.is_finite(),
            Led {
                resistance_ohm,
                min_forward_v,
                max_drive_v,
                ..
            } => {
                resistance_ohm.is_none_or(f64::is_finite)
                    && min_forward_v.is_finite()
                    && max_drive_v.is_finite()
            }
            _ => true,
        };
        if !finite {
            add(Rule::Numeric, Fail, "Число вне конечного диапазона");
            continue;
        }
        let p = pin.and_then(|p| pin_spec(board, p));
        if pin.is_some() && p.is_none() {
            add(Rule::Pin, Fail, "Вывода нет в каталоге платы");
        }
        if let Some(p) = p
            && let Some(reason) = p.reserved
        {
            add(
                Rule::Pin,
                if p.internal_only { Fail } else { NotAnalyzed },
                reason,
            );
        }
        if virtual_board {
            continue;
        }
        if p.is_some_and(|p| p.strap) && !matches!(c, Strap { .. }) {
            let verified = connections.iter().any(|c| matches!(c, Strap { pin: other, compatible_at_reset: Some(true) } if Some(*other) == pin));
            if !verified {
                add(
                    Rule::Strap,
                    NotAnalyzed,
                    "Уровень страпящего вывода в момент сброса не проверен",
                );
            }
        }

        let mut current = None;
        match c {
            Supply { rail, volts } => {
                power = true;
                let key = rail as u8;
                if rails.insert(key, volts).is_some_and(|prior| prior != volts) {
                    add(
                        Rule::Supply,
                        Fail,
                        "На одной шине заданы разные напряжения питания",
                    );
                }
                match supply_range(board, rail) {
                    Some(range) if range.contains(volts) => add(
                        Rule::Supply,
                        Pass,
                        "Напряжение в рабочем диапазоне указанной шины",
                    ),
                    Some(_) => add(
                        Rule::Supply,
                        Fail,
                        "Напряжение вне рабочего диапазона указанной шины",
                    ),
                    None if volts < 0.0 => add(
                        Rule::Supply,
                        Fail,
                        "Отрицательное напряжение на шине питания",
                    ),
                    None => add(
                        Rule::Supply,
                        NotAnalyzed,
                        "Пределы этой шины неизвестны для выбранной платы",
                    ),
                }
            }
            Ground { shared } => {
                ground = true;
                match shared {
                    Some(true) => add(Rule::Ground, Pass, "Общая земля указана"),
                    Some(false) => add(Rule::Ground, Fail, "Нет общей земли сигнальных цепей"),
                    None => add(Rule::Ground, NotAnalyzed, "Топология земли неизвестна"),
                }
            }
            DigitalInput { low_v, high_v, .. } => {
                if let Some(p) = p {
                    if !p.gpio && !p.strap {
                        add(Rule::Pin, Fail, "Вывод не поддерживает цифровой вход");
                    }
                    let max = if p.five_volt_input && board == BoardId::Stm32f103c8 {
                        p.digital_input_max_v
                    } else {
                        p.digital_input_max_v
                            .min(mcu_supply.unwrap_or(spec.nominal_io_v))
                    };
                    if low_v < 0.0 || high_v > max {
                        add(
                            Rule::Overvoltage,
                            Fail,
                            "Уровень за допустимым рабочим диапазоном цифрового входа",
                        );
                    } else if low_v > spec.vil_max_v || high_v < spec.vih_min_v || high_v < low_v {
                        add(
                            Rule::Logic,
                            Fail,
                            "Уровни не достигают порогов логического входа",
                        );
                    } else if high_v > spec.nominal_io_v {
                        add(
                            Rule::Logic,
                            NotAnalyzed,
                            "Нужно подтвердить допустимость уровня при выключении, сбросе и переключении режима",
                        );
                    } else {
                        add(
                            Rule::Logic,
                            NotAnalyzed,
                            "Уровни подходят при номинальном питании; последовательность включения не проверена",
                        );
                    }
                }
            }
            AnalogInput {
                volts, reference_v, ..
            } => {
                if let Some(p) = p {
                    if p.adc_channel.is_none() {
                        add(Rule::Pin, Fail, "У вывода нет канала АЦП");
                    }
                    let ceiling = spec.adc_max_v.min(mcu_supply.unwrap_or(spec.adc_max_v));
                    // DS5319 таблица 9: VDDA >= 2,4 В при использовании АЦП (LQFP48).
                    if volts < 0.0
                        || volts > ceiling
                        || board == BoardId::Stm32f103c8 && mcu_supply.is_some_and(|v| v < 2.4)
                        || reference_v.is_some_and(|r| r <= 0.0 || r > ceiling || volts > r)
                    {
                        add(
                            Rule::AdcRange,
                            Fail,
                            "Вход АЦП вне диапазона опоры или канала",
                        );
                    } else if p.adc_channel.is_none() || p.internal_only {
                        // Ранее уже вынесен вердикт по выводу.
                    } else if reference_v.is_none() {
                        add(
                            Rule::AdcRange,
                            NotAnalyzed,
                            "Не задан верхний предел рабочего диапазона АЦП",
                        );
                    } else {
                        add(Rule::AdcRange, Pass, "Вход АЦП в пределах заданной опоры");
                    }
                }
            }
            Output { net, .. } | ExternalOutput { net } => {
                if pin.is_some_and(|pin| !connections.iter().any(|c| matches!(c,
                    Load { pin: other, .. } | Led { pin: other, .. } | Motor { pin: other, .. } if *other == pin))) {
                    add(Rule::PinCurrent, NotAnalyzed, "Нагрузка выходной цепи не указана");
                }
                if p.is_some_and(|p| !p.gpio) {
                    add(Rule::Pin, Fail, "Вывод не поддерживает цифровой выход");
                }
                if nets.insert(net, i).is_some() {
                    add(
                        Rule::Contention,
                        Fail,
                        "На одной цепи несколько двухтактных выходов",
                    );
                }
            }
            Load {
                milliamps, sinking, ..
            } => current = Some((milliamps, sinking)),
            Led {
                resistance_ohm,
                min_forward_v,
                max_drive_v,
                sinking,
                ..
            } => {
                match resistance_ohm {
                    None => add(
                        Rule::LedResistor,
                        Fail,
                        "Светодиод подключён без токоограничивающего резистора",
                    ),
                    Some(r) if r <= 0.0 || min_forward_v < 0.0 || max_drive_v < 0.0 => {
                        add(
                            Rule::LedResistor,
                            Fail,
                            "Недопустимое сопротивление или напряжение светодиода",
                        );
                    }
                    Some(r) => {
                        // Верхняя граница тока по закону Ома, при минимальном прямом падении.
                        let volts = max_drive_v.max(mcu_supply.unwrap_or(spec.mcu_supply_v.max));
                        current = Some((((volts - min_forward_v).max(0.0)) / r * 1000.0, sinking));
                    }
                }
            }
            Motor { direct, .. } => {
                if direct {
                    add(Rule::DirectMotor, Fail, "Мотор подключён напрямую к выводу");
                } else {
                    add(
                        Rule::DirectMotor,
                        NotAnalyzed,
                        "Драйвер, цепь обратного тока и пусковой режим не проверены",
                    );
                }
            }
            Strap {
                compatible_at_reset,
                ..
            } => match compatible_at_reset {
                Some(false) => add(
                    Rule::Strap,
                    Fail,
                    "Уровень при сбросе несовместим с режимом загрузки",
                ),
                Some(true) if p.is_some_and(|p| p.strap) => {
                    add(Rule::Strap, Pass, "Уровень при сбросе подтверждён")
                }
                _ => add(
                    Rule::Strap,
                    NotAnalyzed,
                    "Режим вывода при сбросе не подтверждён",
                ),
            },
        }
        if let (Some(p), Some((ma, sinking))) = (p, current) {
            let limit = if sinking { p.sink_ma } else { p.source_ma };
            let sum = currents.entry((p.id, sinking)).or_default();
            if !ma.is_finite() || ma < 0.0 {
                add(Rule::Numeric, Fail, "Недопустимый ток нагрузки");
                continue;
            }
            *sum += ma;
            if !sum.is_finite() {
                add(Rule::Numeric, Fail, "Недопустимый ток нагрузки");
            } else if !p.gpio || limit.is_some_and(|limit| *sum > limit) {
                add(
                    Rule::PinCurrent,
                    Fail,
                    "Ток нагрузок превышает рабочий предел вывода",
                );
            } else if limit.is_none()
                || p.internal_only
                || mcu_supply
                    .is_some_and(|volts| volts < spec.nominal_io_v && board != BoardId::Rp2040)
            {
                incomplete_current = true;
                add(
                    Rule::PinCurrent,
                    NotAnalyzed,
                    "Гарантированный рабочий предел тока вывода неизвестен",
                );
            } else {
                add(
                    Rule::PinCurrent,
                    Pass,
                    "Ток нагрузок в пределах проверенного ограничения вывода",
                );
            }
        }
    }

    let mut add = |rule, outcome, message| {
        out.push(WiringFinding {
            connection: None,
            pin: None,
            rule,
            outcome,
            message,
        })
    };
    if virtual_board {
        add(
            Rule::Supply,
            NotAnalyzed,
            "Виртуальный профиль не задаёт электрические пределы реальной платы",
        );
        return out;
    }
    if !power {
        add(Rule::Supply, NotAnalyzed, "Питание не задано");
    }
    if !ground {
        add(Rule::Ground, NotAnalyzed, "Общая земля не указана");
    }
    if rails.len() > 1 {
        add(
            Rule::Supply,
            NotAnalyzed,
            "Совместная работа источников питания не проверена",
        );
    }

    let (source, sink) = currents
        .iter()
        .fold((0.0, 0.0), |(source, sink), ((_, sinking), ma)| {
            if *sinking {
                (source, sink + ma)
            } else {
                (source + ma, sink)
            }
        });
    // 7810D §28.2, примечания 3–4: отдельные группы источников и стоков.
    let group_total = |sinking, pins: std::ops::RangeInclusive<u32>| {
        pins.filter_map(|pin| currents.get(&(pin, sinking)))
            .sum::<f64>()
    };
    let avr_group_over = board == BoardId::Atmega328p
        && (group_total(true, 14..=19) > 100.0
            || group_total(true, 5..=13) > 100.0
            || group_total(true, 0..=4) > 100.0
            || group_total(false, 0..=4) + group_total(false, 14..=19) > 150.0
            || group_total(false, 5..=13) > 150.0);
    if !source.is_finite() || !sink.is_finite() {
        add(Rule::Numeric, Fail, "Недопустимый суммарный ток");
    } else if spec
        .abs_total_ma
        .is_some_and(|limit| source >= limit || sink >= limit)
    {
        add(
            Rule::TotalCurrent,
            Fail,
            "Внешняя нагрузка исчерпывает абсолютный предел суммарного тока",
        );
    } else if spec
        .gpio_total_ma
        .is_some_and(|limit| source >= limit || sink >= limit)
    {
        add(
            Rule::TotalCurrent,
            Fail,
            "Внешние нагрузки исчерпывают суммарный предел GPIO и QSPI",
        );
    } else if avr_group_over {
        add(
            Rule::TotalCurrent,
            Fail,
            "Превышен паспортный ток группы выводов ATmega328P",
        );
    } else if !currents.is_empty() || incomplete_current {
        add(
            Rule::TotalCurrent,
            NotAnalyzed,
            "Токи потребления ядра, групп выводов и тепловой режим не подтверждены",
        );
    }
    out
}

/// Сводный результат для переданных соединений. Неопределённый результат
/// сохраняется, пока хотя бы одно ограничение не проверено.
pub fn wiring_outcome(findings: &[WiringFinding]) -> Outcome {
    if findings.iter().any(|f| f.outcome == Outcome::Fail) {
        Outcome::Fail
    } else if findings.is_empty() || findings.iter().any(|f| f.outcome == Outcome::NotAnalyzed) {
        Outcome::NotAnalyzed
    } else {
        Outcome::Pass
    }
}
