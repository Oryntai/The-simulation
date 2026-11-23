//! Согласование приводов, нагрузок, питания, батареи и соединений платы.

use sim_actuators as act;
use sim_battery as bat;
use sim_boards::{BoardId, Connection, SupplyRail, WiringRule};
use sim_device::measure::Quantity;
use sim_device::{Analysis, BodyRegion, Device, ElectricalSettings, Electronics, JointLoad};

use super::{DeviceReport, Group, Outcome, specified, upper};

const MM: f64 = sim_materials::catalog::MM;
const SECONDS_PER_HOUR: f64 = 3600.0; // Определение часа.

fn actuator_code(k: act::ActuatorCheck) -> &'static str {
    use act::ActuatorCheck::*;
    match k {
        PeakTorque => "peak_torque",
        ContinuousTorque => "continuous_torque",
        Speed => "motor_speed",
        Voltage => "motor_voltage",
        Current => "motor_current",
        DriverCompatibility => "driver_compatibility",
        DriverCurrent => "driver_current",
        DriverVoltage => "driver_voltage",
        DriverTemperature => "driver_temperature",
        WindingTemperature => "winding_temperature",
        AmbientTemperature => "motor_ambient",
        HoldWhenUnpowered => "unpowered_hold",
    }
}

fn battery_code(k: bat::BatteryCheck) -> &'static str {
    use bat::BatteryCheck::*;
    match k {
        ShortCircuit => "short_circuit",
        WireHeating => "wire_heating",
        Protection => "protection",
        CellCurrent => "cell_current",
        CellTemperature => "cell_temperature",
        ThermalRunaway => "thermal_runaway",
        ChargeWindow => "charge",
        SkinContact => "skin_temperature",
        VoltageRange => "battery_voltage",
    }
}

fn wiring_code(k: WiringRule) -> &'static str {
    match k {
        WiringRule::Overvoltage => "overvoltage",
        WiringRule::DirectMotor => "direct_motor",
        WiringRule::Supply => "board_supply",
        WiringRule::Ground => "ground",
        WiringRule::PinCurrent | WiringRule::TotalCurrent => "gpio_current",
        WiringRule::Pin => "pin",
        WiringRule::Contention => "contention",
        WiringRule::Numeric => "wire_numeric",
        WiringRule::Logic => "logic",
        WiringRule::AdcRange => "adc",
        WiringRule::LedResistor => "led",
        WiringRule::Strap => "strap",
    }
}

fn supply(e: &Electronics) -> Option<(SupplyRail, f64)> {
    let mut supplies = e.connections.iter().filter_map(|w| match w.0 {
        Connection::Supply { rail, volts } => Some((rail, volts)),
        _ => None,
    });
    let rail = supplies.next()?;
    supplies.next().is_none().then_some(rail)
}

fn board_load(
    d: &Device,
    board: BoardId,
    regulated: bool,
    full_v: Option<f64>,
    r: &mut DeviceReport,
) -> Option<act::BoardLoad> {
    let e = &d.electronics;
    let s = &e.settings;
    let (_, declared) = supply(e)?;
    let voltage_v = if regulated {
        declared
    } else {
        full_v.unwrap_or(declared)
    };
    let (Some(current_avg_a), Some(current_peak_a)) = (s.board_avg_a, s.board_peak_a) else {
        r.finding(
            Group::Power,
            "board_current",
            None,
            "Потребление платы",
            Outcome::NotAnalyzed,
            "Укажите верхние границы среднего и пикового токов платы с подключёнными выходами",
        );
        return None;
    };
    if !specified(s.board_source.as_deref()) {
        r.finding(
            Group::Power,
            "board_current",
            None,
            "Потребление платы",
            Outcome::NotAnalyzed,
            "Нужен источник границ тока платы",
        );
    }
    let board_thermal = match (s.board_thermal_kw, s.board_temp_limit_c) {
        (Some(k), Some(limit)) if k > 0.0 && limit > -273.15 => {
            let temp = d.usage.ambient_c + voltage_v * current_peak_a * k;
            let outcome =
                if !specified(s.board_source.as_deref()) && upper(temp, limit) == Outcome::Pass {
                    Outcome::NotAnalyzed
                } else {
                    upper(temp, limit)
                };
            r.predict(Quantity::BoardTemperature, None, temp, Some(limit));
            (outcome, temp, limit)
        }
        (Some(_), Some(_)) => (Outcome::Fail, 0.0, 0.0),
        _ => (Outcome::NotAnalyzed, 0.0, 0.0),
    };
    r.numeric(
        Group::Power,
        "board_temperature",
        None,
        "Нагрев платы",
        board_thermal.0,
        board_thermal.1,
        board_thermal.2,
        "°C",
        "Граница температуры по полной пиковой мощности и тепловому сопротивлению платы в корпусе",
    );
    Some(act::BoardLoad {
        name: sim_boards::board_spec(board).name.into(),
        voltage_v,
        current_avg_a,
        current_peak_a,
        rail: regulated.then_some(0),
    })
}

fn wiring(
    d: &Device,
    board: Option<BoardId>,
    regulated: Option<f64>,
    raw: Option<(f64, f64)>,
    r: &mut DeviceReport,
) {
    let e = &d.electronics;
    let active: Vec<_> = d
        .links
        .iter()
        .filter(|l| !l.joint.actuator.is_empty())
        .collect();
    let wires: Vec<_> = e.connections.iter().map(|w| w.0).collect();
    // Подключение силовой обмотки к GPIO опасно независимо от выбранной платы.
    if board.is_none()
        && wires
            .iter()
            .any(|w| matches!(w, Connection::Motor { direct: true, .. }))
    {
        r.finding(
            Group::Wiring,
            "direct_motor",
            None,
            "Мотор от GPIO",
            Outcome::Fail,
            "Мотор подключён напрямую к выводу платы",
        );
    }
    let Some(board) = board else {
        r.finding(
            Group::Wiring,
            "board",
            None,
            "Плата",
            Outcome::NotAnalyzed,
            "Выберите известную плату",
        );
        return;
    };
    let wf = sim_boards::check_wiring(board, &wires);
    let outcome = sim_boards::wiring_outcome(&wf);
    let zero_gpio_loads = !wires.iter().any(|c| {
        matches!(c,
        Connection::Load { milliamps, .. } if *milliamps != 0.0)
    }) && !wires
        .iter()
        .any(|c| matches!(c, Connection::Led { .. } | Connection::Motor { .. }));
    for f in wf {
        let title = f
            .pin
            .map_or_else(|| "Соединение платы".into(), |pin| format!("Вывод {pin}"));
        let proof = f.outcome == sim_boards::Outcome::NotAnalyzed
            && match f.rule {
                WiringRule::Logic => {
                    specified(e.settings.sequencing_source.as_deref())
                        && f.connection.is_some_and(|i| {
                            matches!(wires[i], Connection::DigitalInput { high_v, .. }
                    if high_v <= sim_boards::board_spec(board).nominal_io_v)
                        })
                }
                WiringRule::TotalCurrent => zero_gpio_loads,
                WiringRule::PinCurrent => {
                    zero_gpio_loads
                        && f.connection.is_some_and(|i| {
                            matches!(wires[i], Connection::Load { milliamps: 0.0, .. })
                        })
                }
                _ => false,
            };
        let detail = if proof {
            if f.rule == WiringRule::Logic {
                "Рабочие уровни в пределах платы. Пуск, сброс и снятие питания подтверждены протоколом".into()
            } else {
                "Ведомость выходов указывает нулевую внешнюю нагрузку в этой цепи".into()
            }
        } else {
            match f.connection {
                Some(i) => format!("Соединение {}. {}", i + 1, f.message),
                None => f.message.into(),
            }
        };
        r.finding(
            Group::Wiring,
            wiring_code(f.rule),
            None,
            title,
            if proof {
                Outcome::Pass
            } else {
                f.outcome.into()
            },
            detail,
        );
    }
    if outcome == sim_boards::Outcome::NotAnalyzed && wires.is_empty() {
        r.finding(
            Group::Wiring,
            "connections",
            None,
            "Схема платы",
            Outcome::NotAnalyzed,
            "Нет соединений",
        );
    }
    let mut motor_pins: Vec<_> = wires
        .iter()
        .filter_map(|c| match c {
            Connection::Motor { direct: false, pin } => Some(*pin),
            _ => None,
        })
        .collect();
    motor_pins.sort_unstable();
    motor_pins.dedup();
    let mut declared: Vec<_> = active
        .iter()
        .filter_map(|l| l.settings.motor_gpio)
        .collect();
    declared.sort_unstable();
    if motor_pins.len() != active.len() || declared != motor_pins {
        r.finding(
            Group::Wiring,
            "motor_wiring",
            None,
            "Связь платы и приводов",
            Outcome::NotAnalyzed,
            "Для каждого привода нужен свой управляющий вывод и описание его силового подключения",
        );
    }
    if !e.sensors.is_empty()
        && !wires.iter().any(|c| {
            matches!(
                c,
                Connection::AnalogInput { .. } | Connection::DigitalInput { .. }
            )
        })
    {
        r.finding(
            Group::Wiring,
            "sensor_wiring",
            None,
            "Соединения датчиков",
            Outcome::NotAnalyzed,
            "Указаны датчики, но нет входных цепей, через которые плата с ними работает",
        );
    }
    let Some((rail, declared)) = supply(e) else {
        r.finding(
            Group::Wiring,
            "supply_path",
            None,
            "Источник питания платы",
            Outcome::NotAnalyzed,
            "Требуется одна питающая шина платы с указанным напряжением",
        );
        return;
    };
    if let Some(range) = sim_boards::supply_range(board, rail) {
        if let Some(v) = regulated {
            r.numeric(
                Group::Wiring,
                "supply_path",
                None,
                "Стабилизатор → плата",
                if v == declared {
                    Outcome::Pass
                } else {
                    Outcome::Fail
                },
                v,
                declared,
                "В",
                "Указанное напряжение на шине должно совпадать с настройкой стабилизатора",
            );
        } else if let Some((lo, hi)) = raw {
            r.numeric(
                Group::Wiring,
                "supply_path",
                None,
                "Батарея → плата",
                upper(hi, range.max),
                hi,
                range.max,
                "В",
                "Наибольшее напряжение батареи на входе платы",
            );
            r.numeric(
                Group::Wiring,
                "supply_path",
                None,
                "Батарея → плата",
                super::lower(lo, range.min),
                lo,
                range.min,
                "В",
                "Наименьшее напряжение батареи без учёта дополнительной просадки",
            );
            if !(lo..=hi).contains(&declared) {
                r.finding(
                    Group::Wiring,
                    "supply_path",
                    None,
                    "Батарея → плата",
                    Outcome::Fail,
                    "Указанное напряжение соединения вне диапазона батареи",
                );
            }
        } else {
            r.finding(
                Group::Wiring,
                "supply_path",
                None,
                "Источник питания платы",
                Outcome::NotAnalyzed,
                "Не определён фактический диапазон напряжения шины",
            );
        }
    }
}

fn duty(d: &Device, j: &JointLoad, v: f64) -> (act::DutyCycle, bool) {
    let s = &d.links[j.link].settings;
    let move_s = s.move_s;
    let rest_s = s.rest_s;
    let complete = move_s.is_some() && rest_s.is_some();
    // При отсутствии данных считаем непрерывное действие худшей точки. Единица
    // времени служит только для нормировки постоянного режима и не подтверждает цикл.
    let mut segments = vec![act::DutySegment {
        torque_nm: j.required_nm,
        speed_rad_s: j.speed_rad_s,
        duration_s: move_s.unwrap_or(1.0),
    }];
    if let Some(duration_s) = rest_s.filter(|t| *t != 0.0) {
        segments.push(act::DutySegment {
            torque_nm: j.gravity_nm,
            speed_rad_s: 0.0,
            duration_s,
        });
    }
    (
        act::DutyCycle {
            segments,
            ambient_c: d.usage.ambient_c,
            supply_voltage_v: v,
            unpowered_hold_torque_nm: Some(j.gravity_nm),
        },
        complete,
    )
}

fn inspect_motor(
    d: &Device,
    j: &JointLoad,
    a: &act::Actuator,
    driver: Option<&act::Driver>,
    duty: &act::DutyCycle,
    complete: bool,
    primary: bool,
    r: &mut DeviceReport,
) {
    let i = Some(j.link);
    let l = &d.links[j.link];
    let findings = act::assess_actuator(a, driver, duty);
    let overall = act::overall(&findings);
    for f in findings {
        if !primary
            && f.outcome != act::Outcome::Fail
            && f.kind != act::ActuatorCheck::WindingTemperature
        {
            continue;
        }
        let thermal = f.kind == act::ActuatorCheck::WindingTemperature;
        let peak = f.kind == act::ActuatorCheck::PeakTorque;
        if thermal && f.unit == "°C" && f.outcome == act::Outcome::Pass {
            r.predict(Quantity::MotorTemperature, i, f.value, Some(f.limit));
        }
        if peak && f.unit == "Н·м" && f.limit > 0.0 {
            r.predict(
                Quantity::ActuatorPeakTorque,
                i,
                f.limit,
                Some(j.required_nm),
            );
        }
        r.numeric(
            Group::Actuators,
            actuator_code(f.kind),
            i,
            format!("{} — {}", l.name, f.kind.title()),
            f.outcome.into(),
            f.value,
            f.limit,
            f.unit,
            f.detail,
        );
    }
    if overall == act::Outcome::Fail
        && !r
            .findings
            .iter()
            .any(|f| f.group == Group::Actuators && f.link == i && f.outcome == Outcome::Fail)
    {
        r.finding(
            Group::Actuators,
            "actuator_result",
            i,
            &l.name,
            Outcome::Fail,
            "Расчёт привода вернул ошибку",
        );
    }
    if primary && !complete {
        r.finding(
            Group::Actuators,
            "duty_cycle",
            i,
            &l.name,
            Outcome::NotAnalyzed,
            "Не задано время движения и удержания за цикл",
        );
    }
    if primary && l.joint.torque_limit_nm > 0.0 {
        r.numeric(
            Group::Actuators,
            "torque_limit",
            i,
            &l.name,
            upper(j.required_nm, l.joint.torque_limit_nm),
            j.required_nm,
            l.joint.torque_limit_nm,
            "Н·м",
            "Момент нагрузки против установленного предела",
        );
    }
}

fn battery_wire(e: &Electronics) -> Option<bat::WireRun> {
    let s = &e.settings;
    let conductor = match s.wire_kind.as_deref()? {
        "mm2" => bat::Conductor::Mm2(e.wire_mm2),
        "awg" => bat::Conductor::Awg(u8::try_from(s.awg?).ok()?),
        _ => return None,
    };
    let insulation = match s.insulation.as_deref()? {
        "pvc" => bat::Insulation::Pvc,
        "silicone" => bat::Insulation::Silicone,
        "ptfe" => bat::Insulation::Ptfe,
        _ => return None,
    };
    let cooling = match s.cooling.as_deref()? {
        "air" => bat::WireCooling::FreeAir,
        "bundled" => bat::WireCooling::Bundled,
        "enclosed" => bat::WireCooling::Enclosed,
        _ => return None,
    };
    Some(bat::WireRun {
        conductor,
        insulation,
        cooling,
        length_m: e.wire_m,
        outer_diameter_mm: s.wire_outer_mm,
    })
}

fn bms(e: &Electronics) -> Option<bat::BmsSpec> {
    if !e.bms {
        return None;
    }
    let s = &e.settings;
    Some(bat::BmsSpec {
        name: s.bms_name.clone()?,
        scd_trip_min_a: s.bms_scd_min_a?,
        scd_trip_max_a: s.bms_scd_max_a?,
        scd_delay_max_s: s.bms_scd_delay_s?,
        ocd_trip_min_a: s.bms_ocd_min_a?,
        ocd_trip_max_a: s.bms_ocd_max_a?,
        ocd_delay_min_s: s.bms_ocd_min_s?,
        ocd_delay_max_s: s.bms_ocd_max_s?,
        cell_ovp_v: s.bms_ovp_v,
        cell_uvp_v: s.bms_uvp_v,
        mosfet: bat::MosfetSpec {
            vds_max_v: s.mosfet_v?,
            id_pulse_a: s.mosfet_a?,
            id_pulse_width_s: s.mosfet_s?,
            rds_on_ohm: s.mosfet_ohm?,
            avalanche_energy_j: s.mosfet_j,
        },
    })
}

fn enclosure(s: &ElectricalSettings) -> Option<bat::EnclosureDesign> {
    Some(bat::EnclosureDesign {
        outer_area_m2: s.enclosure_area_m2?,
        height_m: s.enclosure_height_mm? * MM,
        wall_thickness_m: s.enclosure_wall_mm? * MM,
        wall_k_w_mk: s.enclosure_k?,
        h_inner_w_m2k: s.enclosure_h?,
        emissivity: s.enclosure_emissivity?,
        body_side_fraction: s.enclosure_body_fraction?,
    })
}

fn skin(s: &ElectricalSettings, long: bool) -> Option<bat::SkinInterface> {
    use bat::ContactClass::*;
    let surface_class = match s.skin_class.as_deref()? {
        "metal" => UncoatedMetal,
        "plastic" => Plastics,
        "wood" => Wood,
        "ceramic" => CeramicGlassStone,
        "coated" => PaintedMetal,
        _ => return None,
    };
    Some(bat::SkinInterface {
        spacer_thickness_m: s.spacer_mm? * MM,
        spacer_k_w_mk: s.spacer_k?,
        // ISO 13732-1: асимптота для длительного контакта, 8 часов.
        contact_time_s: if long {
            s.skin_s?.max(8.0 * SECONDS_PER_HOUR)
        } else {
            s.skin_s?
        },
        surface_class,
        contact_body_fraction: s.skin_fraction?,
    })
}

fn charge(s: &ElectricalSettings) -> Option<bat::ChargeProfile> {
    Some(bat::ChargeProfile {
        current_a: s.charger_a?,
        voltage_v: s.charger_v?,
        ambient_min_c: s.charge_min_c?,
        ambient_max_c: s.charge_max_c?,
    })
}

fn pack_design(d: &Device, cell: &bat::CellSpec, r: &mut DeviceReport) -> Option<bat::PackDesign> {
    let e = &d.electronics;
    let s = &e.settings;
    if e.fuse_a == 0.0 {
        r.finding(
            Group::Battery,
            "no_fuse",
            None,
            "Предохранитель",
            Outcome::Fail,
            "Нет предохранителя для отключения короткого замыкания",
        );
    }
    let mut fuse = bat::FuseSpec::ato(e.fuse_a as u32).filter(|f| f.rating_a == e.fuse_a);
    if let (Some(f), Some(position)) = (&mut fuse, s.fuse_position_m) {
        f.position_m = position;
    } else if e.fuse_a > 0.0 {
        r.finding(
            Group::Battery,
            "fuse_data",
            None,
            "Предохранитель",
            Outcome::NotAnalyzed,
            "Нужен номинал Littelfuse ATO из каталога и расстояние от клеммы",
        );
        return None;
    }
    let bms = bms(e);
    if e.bms && bms.is_none() {
        r.finding(
            Group::Battery,
            "bms_data",
            None,
            "Плата защиты",
            Outcome::NotAnalyzed,
            "Нужны паспортные пороги, времена и параметры разрядного ключа",
        );
        return None;
    }
    let (Some(wire), Some(spread), Some(group), Some(interconnect), Some(contacts)) = (
        battery_wire(e),
        s.resistance_spread,
        s.group_spread_v,
        s.interconnect_ohm,
        s.contacts_ohm,
    ) else {
        r.finding(Group::Battery, "pack_data", None, "Сборка и провод", Outcome::NotAnalyzed,
            "Нужны разброс ячеек, сопротивления перемычек и контактов, тип жилы, изоляция и укладка");
        return None;
    };
    Some(bat::PackDesign {
        cell: cell.clone(),
        series: e.series,
        parallel: e.parallel,
        cell_resistance_spread: spread,
        group_voltage_spread_v: group,
        interconnect_r_ohm: interconnect,
        contact_r_ohm: contacts,
        wire,
        fuse,
        bms,
        enclosure: enclosure(s),
        skin: skin(s, d.usage.long_wear),
        worn_on_body: d.mount != BodyRegion::None
            || d.links.iter().any(|l| l.near_body != BodyRegion::None),
    })
}

fn battery_input(
    d: &Device,
    c: &bat::CellSpec,
    pack: Option<&bat::PackDesign>,
) -> Option<act::Battery> {
    let e = &d.electronics;
    if e.series == 0 || e.parallel == 0 {
        return None;
    }
    let series = e.series as f64;
    let groups = f64::from(e.series - 1);
    let imbalance = e.settings.group_spread_v.unwrap_or(0.0) * groups;
    let min_temp = d.settings.cold_c;
    let continuous = pack.and_then(|p| {
        Some(c.discharge_limit_a(min_temp?)? * f64::from(p.parallel) / p.worst_cell_share())
    });
    let resistance = pack.and_then(|p| {
        let temp = min_temp?;
        Some(
            p.cell_r_worst_ohm(temp)? * p.s_over_p() * (1.0 + p.cell_resistance_spread)
                + p.interconnect_r_ohm
                + p.contact_r_ohm
                + p.wire
                    .loop_resistance_ohm(2.0 * p.wire.length_m, p.wire.insulation.continuous_c())
                + p.fuse.as_ref()?.cold_resistance_ohm?
                + p.bms.as_ref().map_or(0.0, |b| b.mosfet.rds_on_ohm),
        )
    });
    Some(act::Battery {
        nominal_v: series * c.v_nominal.value?,
        full_v: series * c.v_charge_max.value?,
        // Плата защиты отключает батарею раньше паспортного минимума ячейки, если так настроена.
        empty_v: series
            * c.v_cutoff
                .value?
                .max(e.settings.bms_uvp_v.filter(|_| e.bms).unwrap_or(0.0))
            + imbalance,
        capacity_ah: c.capacity_min_ah.value? * e.parallel as f64,
        max_continuous_a: continuous,
        // Длительный предел используется и для пиковой оценки, чтобы не засчитывать
        // паспортный импульс без подтверждённой длительности и паузы между импульсами.
        max_peak_a: continuous,
        internal_resistance_ohm: resistance,
    })
}

pub(super) fn evaluate(d: &Device, analysis: Option<&Analysis>, r: &mut DeviceReport) {
    let e = &d.electronics;
    let board = crate::device_catalog::board(&e.board);
    let regulator = act::regulator_by_id(&e.regulator);
    let rail = regulator.zip(e.settings.rail_v);
    if !e.regulator.is_empty() && rail.is_none() {
        r.finding(
            Group::Power,
            "regulator",
            None,
            "Стабилизатор",
            Outcome::NotAnalyzed,
            "Стабилизатор неизвестен или выходное напряжение не задано",
        );
    }
    let cell = bat::find_cell(&e.cell);
    let pack = cell.as_ref().and_then(|c| pack_design(d, c, r));
    let battery = cell
        .as_ref()
        .and_then(|c| battery_input(d, c, pack.as_ref()));
    let board_regulated = e
        .settings
        .board_regulated
        .unwrap_or(!e.regulator.is_empty());
    if !e.regulator.is_empty() && e.settings.board_regulated.is_none() {
        r.finding(
            Group::Power,
            "board_rail",
            None,
            "Питание платы",
            Outcome::NotAnalyzed,
            "При выбранном стабилизаторе укажите, откуда питается плата",
        );
    }
    if board_regulated && rail.is_none() {
        r.finding(
            Group::Power,
            "board_rail",
            None,
            "Питание платы",
            Outcome::NotAnalyzed,
            "Для платы выбрано питание от стабилизатора без заданного выхода",
        );
    }
    let board_voltage = board_regulated.then(|| rail.map(|(_, v)| v)).flatten();
    let raw = (!board_regulated)
        .then(|| battery.as_ref().map(|p| (p.empty_v, p.full_v)))
        .flatten();
    wiring(d, board, board_voltage, raw, r);
    let active = d.links.iter().any(|l| !l.joint.actuator.is_empty());
    if !active {
        r.finding(
            Group::Actuators,
            "no_actuators",
            None,
            "Нет приводов",
            Outcome::Pass,
            "В дереве устройства нет приводных звеньев. Проверка приводов неприменима",
        );
    }
    if cell.is_none() || e.series == 0 || e.parallel == 0 {
        r.finding(
            Group::Battery,
            "cell",
            None,
            "Ячейки и схема S×P",
            Outcome::NotAnalyzed,
            "Не выбрана ячейка из каталога или не указано число ячеек в группах",
        );
    }
    let driver = act::driver_by_id(&e.driver);
    let board = board.and_then(|b| {
        board_load(
            d,
            b,
            board_regulated && rail.is_some(),
            battery.as_ref().map(|b| b.full_v),
            r,
        )
    });
    if board.is_none() {
        r.finding(
            Group::Power,
            "board_power",
            None,
            "Плата и питание",
            Outcome::NotAnalyzed,
            "Нет полного описания питания платы",
        );
    }
    let mut sensors = Vec::new();
    if e.settings.sensors_declared != Some(true) {
        r.finding(
            Group::Power,
            "sensor_inventory",
            None,
            "Ведомость датчиков",
            Outcome::NotAnalyzed,
            "Подтвердите полноту списка датчиков, включая случай без датчиков",
        );
    }
    for s in &e.sensors {
        let sensor = s.id.as_deref().and_then(act::sensor_by_id);
        if let (Some(sensor), Some(count), Some(on_regulator)) = (sensor, s.count, s.on_regulator)
            && count > 0
            && (!on_regulator || rail.is_some())
        {
            sensors.push(act::SensorLoad {
                sensor,
                count,
                rail: on_regulator.then_some(0),
                current_override_a: s.current_a,
            });
        } else {
            r.finding(
                Group::Power,
                "sensor",
                None,
                "Датчик",
                Outcome::NotAnalyzed,
                "Укажите модель датчика, ненулевое количество и существующую питающую шину",
            );
        }
    }
    let mut motors = Vec::new();
    let mut uses_global_driver = false;
    if let Some(analysis) = analysis {
        for j in &analysis.joints {
            let l = &d.links[j.link];
            if l.joint.actuator.is_empty() {
                continue;
            }
            let i = Some(j.link);
            let Some(a) = act::actuator_by_id(&l.joint.actuator) else {
                r.finding(
                    Group::Actuators,
                    "actuator",
                    i,
                    &l.name,
                    Outcome::NotAnalyzed,
                    "Неизвестный привод",
                );
                continue;
            };
            let drv = if let Some(id) = l.settings.driver_id.as_deref() {
                act::driver_by_id(id)
            } else if a.builtin_driver.is_none() {
                uses_global_driver |= driver.is_some();
                driver
            } else {
                None
            };
            if l.settings.driver_id.is_some() && drv.is_none() {
                r.finding(
                    Group::Actuators,
                    "driver_catalog",
                    i,
                    &l.name,
                    Outcome::NotAnalyzed,
                    "Драйвер шарнира отсутствует в каталоге",
                );
            }
            if a.builtin_driver.is_none() && drv.is_none() {
                r.finding(
                    Group::Actuators,
                    "missing_driver",
                    i,
                    &l.name,
                    Outcome::Fail,
                    "У привода нет драйвера",
                );
            }
            let motor_regulated = l.settings.motor_regulated == Some(true);
            if !e.regulator.is_empty() && l.settings.motor_regulated.is_none() {
                r.finding(
                    Group::Power,
                    "motor_rail",
                    i,
                    &l.name,
                    Outcome::NotAnalyzed,
                    "Укажите, питается ли привод от батареи или от стабилизатора",
                );
            }
            let voltage = if motor_regulated {
                rail.map(|(_, v)| (v, v))
            } else {
                battery.as_ref().map(|b| (b.empty_v, b.full_v))
            };
            let Some((lo, hi)) = voltage else {
                if let Some(peak) = a.peak_torque_nm.get() {
                    let out = if j.gravity_nm > peak {
                        Outcome::Fail
                    } else {
                        Outcome::NotAnalyzed
                    };
                    r.numeric(Group::Actuators, "peak_torque", i, &l.name, out, j.gravity_nm, peak, "Н·м",
                        "Статический момент против паспортного пика. Нет диапазона питания для рабочей точки");
                }
                r.finding(
                    Group::Power,
                    "motor_supply",
                    i,
                    &l.name,
                    Outcome::NotAnalyzed,
                    "Не найдено питание привода на указанной шине",
                );
                continue;
            };
            let (low, complete) = duty(d, j, lo);
            inspect_motor(d, j, a, drv, &low, complete, true, r);
            if hi != lo {
                let (high, _) = duty(d, j, hi);
                inspect_motor(d, j, a, drv, &high, complete, false, r);
            }
            motors.push((j, a, drv, low, motor_regulated.then_some(0)));
        }
    }
    if !e.driver.is_empty() && !uses_global_driver {
        r.finding(
            Group::Power,
            "unused_driver",
            None,
            "Внешний драйвер",
            Outcome::NotAnalyzed,
            "Выбранный драйвер не сопоставлен ни одному приводу",
        );
    }
    let current = battery.as_ref().and_then(|battery| {
        if d.usage.hours_per_day <= 0.0 {
            r.finding(Group::Power, "runtime", None, "Продолжительность работы", Outcome::NotAnalyzed,
                "Задайте время работы за день");
        }
        let input = act::PowerInput {
            battery: battery.clone(),
            rails: rail.into_iter().map(|(regulator, output_v)| act::RailInput {
                name: regulator.name.into(), regulator, output_v,
            }).collect(),
            actuators: motors.iter().map(|(_, actuator, driver, duty, rail)| act::ActuatorLoad {
                actuator, driver: *driver, duty, count: 1, rail: *rail,
            }).collect(),
            sensors, boards: board.into_iter().collect(), ambient_c: d.usage.ambient_c,
            required_runtime_s: (d.usage.hours_per_day > 0.0).then_some(d.usage.hours_per_day * SECONDS_PER_HOUR),
        };
        let budget = act::power_budget(&input);
        let direct_board = if !board_regulated && !e.board.is_empty() {
            crate::device_catalog::board(&e.board).zip(supply(e))
                .and_then(|(board, (rail, _))| sim_boards::supply_range(board, rail))
                .zip(battery.internal_resistance_ohm)
                .zip(input.boards.first())
                .filter(|((range, res), _)| budget.total_peak_a > 0.0 && battery.full_v <= range.max
                    && battery.empty_v - res * budget.total_peak_a >= range.min)
        } else { None };
        for f in &budget.findings {
            if f.kind == act::PowerCheck::ConsumerVoltage && f.outcome == act::Outcome::NotAnalyzed
                && let Some(((range, res), board)) = direct_board && f.subject == board.name
            {
                r.numeric(Group::Power, "board_range", None, &f.subject, Outcome::Pass,
                    battery.empty_v - res * budget.total_peak_a, range.min, "В",
                    "Диапазон входа взят из паспорта платы, включая пик под просадкой и полный заряд");
            } else if f.kind == act::PowerCheck::BatterySag && f.outcome == act::Outcome::NotAnalyzed
                && let Some(((range, res), _)) = direct_board
                && !budget.findings.iter().any(|x| x.kind == act::PowerCheck::DataCompleteness && x.outcome != act::Outcome::Pass)
                && f.value > 0.0 && f.value >= f.limit
            {
                r.numeric(Group::Power, "board_sag", None, "Просадка на входе платы", Outcome::Pass,
                    battery.empty_v - res * budget.total_peak_a, f.limit.max(range.min), "В",
                    "Предел прямого входа платы учтён после расчёта просадки");
            } else {
                r.numeric(Group::Power, "power_budget", None, format!("{} — {}", f.subject, f.kind.title()),
                    f.outcome.into(), f.value, f.limit, f.unit, &f.detail);
            }
        }
        if !budget.total_peak_a.is_finite() || !budget.total_avg_a.is_finite()
            || !budget.total_peak_w.is_finite() || !budget.total_avg_w.is_finite()
            || budget.runtime_s.is_some_and(|x| !x.is_finite())
        {
            r.finding(Group::Power, "power_result", None, "Суммарное питание", Outcome::Fail, "Неконечный результат");
            return None;
        }
        let i = budget.total_peak_a;
        // Корень баланса P = I(U−IR) уже найден в бюджете. Дополнительно прогоняем
        // рабочую точку и входы реальных потребителей под пиковой просадкой.
        if i > 0.0 && let Some(res) = battery.internal_resistance_ohm {
            let sagged = battery.empty_v - res * i;
            if sagged > 0.0 {
                for (j, a, drv, prev, _) in motors.iter().filter(|motor| motor.4.is_none()) {
                    let cycle = act::DutyCycle { supply_voltage_v: sagged, ..prev.clone() };
                    inspect_motor(d, j, a, *drv, &cycle, true, false, r);
                }
                if let Some((reg, v)) = rail {
                    let need = reg.required_input_v(v);
                    r.numeric(Group::Power, "loaded_rail", None, "Вход стабилизатора под пиковой нагрузкой",
                        super::lower(sagged, need), sagged, need, "В", "Пиковый ток всех потребителей и потери в сборке");
                }
                if !board_regulated && let Some((p, (rail, _))) = crate::device_catalog::board(&e.board).zip(supply(e))
                    && let Some(range) = sim_boards::supply_range(p, rail)
                {
                    r.numeric(Group::Power, "loaded_board", None, "Питание платы под пиковой нагрузкой",
                        super::lower(sagged, range.min), sagged, range.min, "В", "Пиковый ток всех потребителей и потери в сборке");
                }
            }
        }
        let complete = !budget.findings.iter().any(|f| f.kind == act::PowerCheck::DataCompleteness && f.outcome != act::Outcome::Pass)
            && !input.boards.is_empty() && motors.len() == d.links.iter().filter(|l| !l.joint.actuator.is_empty()).count()
            && input.sensors.len() == e.sensors.len() && e.settings.sensors_declared == Some(true)
            && (e.regulator.is_empty() || rail.is_some()) && !budget.lines.is_empty() && budget.lines.iter().all(|l| l.known);
        if complete && i > 0.0 {
            r.predict(Quantity::BatteryCurrent, None, i, battery.max_peak_a);
        } else {
            r.finding(Group::Power, "power_coverage", None, "Суммарное питание", Outcome::NotAnalyzed,
                "В сумме есть неучтённые потребители или неполные характеристики");
        }
        // Цикл нагрузки: длинный участок движения на пике, остальное время — так, чтобы
        // средний ток совпал с бюджетом. Без полного цикла остаётся постоянный пик.
        let cycle = (!motors.is_empty()).then(|| {
            motors.iter().try_fold((0.0_f64, 0.0_f64), |(peak_s, period), (j, ..)| {
                let s = &d.links[j.link].settings;
                let (mv, rest) = (s.move_s?, s.rest_s?);
                Some((peak_s.max(mv), period.max(mv + rest)))
            })
        }).flatten();
        (i > 0.0).then_some(Demand { peak_a: i, avg_a: budget.total_avg_a, cycle })
    });
    if battery.is_none() {
        r.finding(
            Group::Power,
            "battery_supply",
            None,
            "Источник питания",
            Outcome::NotAnalyzed,
            "Нет паспортного диапазона напряжений и ёмкости батареи",
        );
    }
    let Some(pack) = pack else {
        return;
    };
    battery_findings(d, &pack, current, rail, board_regulated, r);
}

/// Что батарея отдаёт за рабочий интервал.
#[derive(Clone, Copy)]
struct Demand {
    peak_a: f64,
    avg_a: f64,
    /// Длительность пика и период цикла, с.
    cycle: Option<(f64, f64)>,
}

/// Шаги разряда и число их повторов за `duration_s`.
fn discharge_steps(demand: Demand, duration_s: f64) -> (Vec<bat::LoadStep>, u32) {
    const MAX_REPEATS: f64 = 2000.0; // Тепловая постоянная батареи намного больше такого шага.
    let constant = |current_a| {
        (
            vec![bat::LoadStep {
                current_a,
                duration_s,
            }],
            1,
        )
    };
    let Some((peak_s, period)) = demand.cycle.filter(|&(p, t)| p > 0.0 && t > p) else {
        return constant(demand.peak_a);
    };
    let base = ((demand.avg_a * period - demand.peak_a * peak_s) / (period - peak_s))
        .clamp(0.0, demand.peak_a);
    let repeats = (duration_s / period).ceil().clamp(1.0, MAX_REPEATS);
    // Длинный ряд укрупняем с той же скважностью: пик становится длиннее, то есть строже.
    let stretch = (duration_s / period / repeats).max(1.0);
    (
        vec![
            bat::LoadStep {
                current_a: demand.peak_a,
                duration_s: peak_s * stretch,
            },
            bat::LoadStep {
                current_a: base,
                duration_s: (period - peak_s) * stretch,
            },
        ],
        repeats as u32,
    )
}

fn battery_findings(
    d: &Device,
    pack: &bat::PackDesign,
    current: Option<Demand>,
    rail: Option<(&act::Regulator, f64)>,
    board_regulated: bool,
    r: &mut DeviceReport,
) {
    let s = &d.electronics.settings;
    let duration_s = d.usage.hours_per_day * SECONDS_PER_HOUR;
    if d.settings.cold_c.is_none() {
        r.finding(
            Group::Battery,
            "cold",
            None,
            "Наименьшая температура",
            Outcome::NotAnalyzed,
            "Без неё не подтверждены просадка, ток КЗ и рабочее окно на холоде",
        );
    }
    if current.is_none() || duration_s <= 0.0 {
        r.finding(
            Group::Battery,
            "discharge",
            None,
            "Профиль разряда",
            Outcome::NotAnalyzed,
            "Нет расчётного тока устройства или длительности работы",
        );
    }
    if pack.enclosure.is_none() {
        r.finding(
            Group::Battery,
            "enclosure",
            None,
            "Корпус",
            Outcome::NotAnalyzed,
            "Тепловая конструкция корпуса не задана",
        );
    }
    let board_range = crate::device_catalog::board(&d.electronics.board)
        .zip(supply(&d.electronics))
        .and_then(|(b, (s, _))| sim_boards::supply_range(b, s));
    let mut min_v = board_range.filter(|_| !board_regulated).map(|r| r.min);
    let mut max_v = board_range.filter(|_| !board_regulated).map(|r| r.max);
    if let Some((reg, volts)) = rail {
        let min = reg.required_input_v(volts);
        min_v = Some(min_v.map_or(min, |old| old.max(min)));
        if let Some(max) = reg.input_max_v.get() {
            max_v = Some(max_v.map_or(max, |old| old.min(max)));
        }
    }
    for a in d
        .links
        .iter()
        .filter(|l| l.settings.motor_regulated != Some(true))
        .filter_map(|l| act::actuator_by_id(&l.joint.actuator))
    {
        if let Some(v) = a.voltage_min_v.get() {
            min_v = Some(min_v.map_or(v, |p| p.max(v)));
        }
        if let Some(v) = a.voltage_max_v.get() {
            max_v = Some(max_v.map_or(v, |p| p.min(v)));
        }
    }
    let (steps, cycles) = current
        .filter(|_| duration_s > 0.0)
        .map_or((Vec::new(), 1), |demand| discharge_steps(demand, duration_s));
    let profile = bat::LoadProfile {
        steps,
        cycles,
        ambient_min_c: d.settings.cold_c.unwrap_or(d.usage.ambient_c),
        ambient_max_c: d.usage.ambient_c,
        load_v_min: min_v,
        load_v_max: max_v,
        charge: charge(s),
    };
    let findings = bat::assess_pack(pack, &profile);
    let overall = bat::overall(&findings);
    for f in findings {
        let skin_na = f.kind == bat::BatteryCheck::SkinContact
            && !pack.worn_on_body
            && f.outcome == bat::Outcome::NotAnalyzed;
        if !skin_na && f.outcome == bat::Outcome::Pass && f.unit == "°C" {
            match f.kind {
                bat::BatteryCheck::SkinContact => r.predict(
                    Quantity::SkinSurfaceTemperature,
                    None,
                    f.value,
                    Some(f.limit),
                ),
                bat::BatteryCheck::CellTemperature => (),
                _ => (),
            }
        }
        r.numeric(
            Group::Battery,
            battery_code(f.kind),
            None,
            f.kind.name(),
            if skin_na {
                Outcome::Pass
            } else {
                f.outcome.into()
            },
            f.value,
            f.limit,
            f.unit,
            if skin_na {
                "Устройство не носится на теле. Контакт батареи с кожей при носке неприменим".into()
            } else {
                f.detail
            },
        );
    }
    if overall == bat::Outcome::Fail
        && !r
            .findings
            .iter()
            .any(|f| f.group == Group::Battery && f.outcome == Outcome::Fail)
    {
        r.finding(
            Group::Battery,
            "battery_result",
            None,
            "Батарея",
            Outcome::Fail,
            "Расчёт батареи вернул ошибку",
        );
    }
    if bat::validate(pack, &profile).is_empty()
        && let Some(t) = bat::simulate_temperature(pack, &profile)
    {
        if ![t.heat_peak_w, t.t_end_c, t.t_max_c]
            .into_iter()
            .all(f64::is_finite)
        {
            r.finding(
                Group::Battery,
                "thermal_result",
                None,
                "Температура батареи",
                Outcome::Fail,
                "Неконечный тепловой расчёт",
            );
        }
        r.predict(
            Quantity::BatteryTemperature,
            None,
            t.t_max_c,
            pack.cell.t_discharge_max_c.value,
        );
    }
}
