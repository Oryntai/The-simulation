//! Хранение соединений каталога плат в проекте.

use sim_boards::{Connection, SupplyRail};

#[derive(Clone, Copy, Debug)]
pub struct Wiring(pub Connection);

impl PartialEq for Wiring {
    fn eq(&self, other: &Self) -> bool {
        self.to_line() == other.to_line()
    }
}

impl Wiring {
    pub const FORMATS: &'static [(&'static str, &'static str)] = &[
        (
            "supply",
            "Питание: supply шина(mcu/vsys/vin/usb) напряжение_В",
        ),
        ("ground", "Общая земля: ground true/false/—"),
        ("digital", "Цифровой вход: digital вывод низкий_В высокий_В"),
        (
            "analog",
            "Аналоговый вход: analog вывод напряжение_В опора_В/—",
        ),
        ("output", "Выход: output вывод номер_цепи"),
        ("external", "Внешний выход: external номер_цепи"),
        ("load", "Нагрузка GPIO: load вывод ток_мА сток(true/false)"),
        (
            "led",
            "Светодиод: led вывод резистор_Ом/— падение_В питание_В сток(true/false)",
        ),
        (
            "motor",
            "Управление мотором: motor вывод напрямую(true/false)",
        ),
        (
            "strap",
            "Загрузочный вывод: strap вывод допустим_при_сбросе(true/false/—)",
        ),
    ];

    pub fn to_line(self) -> String {
        use Connection::*;
        let opt = |v: Option<f64>| v.map_or("—".into(), |v| v.to_string());
        let flag = |v: Option<bool>| v.map_or("—".into(), |v| v.to_string());
        match self.0 {
            Supply { rail, volts } => {
                let rail = match rail {
                    SupplyRail::Mcu => "mcu",
                    SupplyRail::Vsys => "vsys",
                    SupplyRail::Vin => "vin",
                    SupplyRail::Usb => "usb",
                };
                format!("supply {rail} {volts}")
            }
            Ground { shared } => format!("ground {}", flag(shared)),
            DigitalInput { pin, low_v, high_v } => format!("digital {pin} {low_v} {high_v}"),
            AnalogInput {
                pin,
                volts,
                reference_v,
            } => format!("analog {pin} {volts} {}", opt(reference_v)),
            Output { pin, net } => format!("output {pin} {net}"),
            ExternalOutput { net } => format!("external {net}"),
            Load {
                pin,
                milliamps,
                sinking,
            } => format!("load {pin} {milliamps} {sinking}"),
            Led {
                pin,
                resistance_ohm,
                min_forward_v,
                max_drive_v,
                sinking,
            } => format!(
                "led {pin} {} {min_forward_v} {max_drive_v} {sinking}",
                opt(resistance_ohm)
            ),
            Motor { pin, direct } => format!("motor {pin} {direct}"),
            Strap {
                pin,
                compatible_at_reset,
            } => format!("strap {pin} {}", flag(compatible_at_reset)),
        }
    }

    pub fn from_line(line: &str) -> Result<Self, String> {
        use Connection::*;
        fn value<T: std::str::FromStr>(s: &str) -> Result<T, String> {
            s.parse()
                .map_err(|_| format!("Неверное значение соединения «{s}»"))
        }
        fn opt<T: std::str::FromStr>(s: &str) -> Result<Option<T>, String> {
            if matches!(s, "—" | "-") {
                Ok(None)
            } else {
                value(s).map(Some)
            }
        }
        let tokens: Vec<_> = line.split_whitespace().collect();
        Ok(Self(match tokens.as_slice() {
            ["supply", rail, volts] => Supply {
                rail: match *rail {
                    "mcu" => SupplyRail::Mcu,
                    "vsys" => SupplyRail::Vsys,
                    "vin" => SupplyRail::Vin,
                    "usb" => SupplyRail::Usb,
                    _ => return Err("Неизвестная шина питания платы".into()),
                },
                volts: value(volts)?,
            },
            ["ground", shared] => Ground {
                shared: opt(shared)?,
            },
            ["digital", pin, low, high] => DigitalInput {
                pin: value(pin)?,
                low_v: value(low)?,
                high_v: value(high)?,
            },
            ["analog", pin, volts, reference] => AnalogInput {
                pin: value(pin)?,
                volts: value(volts)?,
                reference_v: opt(reference)?,
            },
            ["output", pin, net] => Output {
                pin: value(pin)?,
                net: value(net)?,
            },
            ["external", net] => ExternalOutput { net: value(net)? },
            ["load", pin, ma, sink] => Load {
                pin: value(pin)?,
                milliamps: value(ma)?,
                sinking: value(sink)?,
            },
            ["led", pin, ohm, forward, drive, sink] => Led {
                pin: value(pin)?,
                resistance_ohm: opt(ohm)?,
                min_forward_v: value(forward)?,
                max_drive_v: value(drive)?,
                sinking: value(sink)?,
            },
            ["motor", pin, direct] => Motor {
                pin: value(pin)?,
                direct: value(direct)?,
            },
            ["strap", pin, reset] => Strap {
                pin: value(pin)?,
                compatible_at_reset: opt(reset)?,
            },
            _ => {
                return Err("Укажите вид соединения и его параметры по формату из подсказки".into());
            }
        }))
    }

    pub fn is_finite(self) -> bool {
        use Connection::*;
        match self.0 {
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
            Ground { .. } | Output { .. } | ExternalOutput { .. } | Motor { .. } | Strap { .. } => {
                true
            }
        }
    }
}
