//! Дискретная модель жёсткой передачи с приложением момента на конце шага.
//! Нужна для воспроизведения экспериментов, задающих такую квадратуру явно.

use serde::Deserialize;
use sim_circuit::battery::{CellParams, Pack};
use sim_em::motor::{DatasheetValues, GearParams, Motor, MotorParams, ThermalParams};
use sim_runtime::{Forces, Object, Runtime, Truth, TruthId, control::ModelPort};
use sim_signal::{ChannelId, Command, Rng};
use sim_world::{CompiledJointKind, CompiledWorld, Simplification};

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Spec {
    source: String,
    bridge_effort: String,
    pub ambient_c: f64,
    pub fixture_enabled: bool,
    supply: Supply,
    motor: Electrical,
    gear: Gear,
    thermal: Thermal,
    fixture: Option<Fixture>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Supply {
    cell: Cell,
    series: usize,
    parallel: usize,
    soc: f64,
    electronics_a: f64,
    max_regen_a: f64,
    conversion_floor_v: f64,
}

#[derive(Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Cell {
    LgM50,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Measurement {
    WindingCurrent,
    HousingTemperature,
    SupplyVoltage,
}

impl Measurement {
    pub fn output(self) -> (&'static str, &'static str) {
        match self {
            Self::WindingCurrent => ("current_a", "A"),
            Self::HousingTemperature => ("housing_c", "degC"),
            Self::SupplyVoltage => ("bus_next_v", "V"),
        }
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Electrical {
    nominal_v: f64,
    r_ohm: f64,
    r_ref_c: f64,
    l_h: f64,
    k: f64,
    j: f64,
    coulomb: f64,
    viscous: f64,
    band: f64,
    max_a: f64,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Gear {
    ratio: f64,
    eta_forward: f64,
    eta_backward: f64,
    friction: f64,
    j: f64,
    rated: f64,
    repeated: f64,
    peak: f64,
    mass: f64,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Thermal {
    r_wh: f64,
    r_ha: f64,
    tau_w: f64,
    tau_h: f64,
    max_c: f64,
    fraction: f64,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    damping: f64,
    min: f64,
    max: f64,
    stop_k: f64,
    stop_damping: f64,
    target: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    integral_limit: f64,
    torque_limit: f64,
}

impl Fixture {
    fn valid(&self) -> bool {
        [self.stop_k, self.integral_limit, self.torque_limit]
            .into_iter()
            .all(|value| value.is_finite() && value > 0.0)
            && [self.damping, self.stop_damping, self.kp, self.ki, self.kd]
                .into_iter()
                .all(|value| value.is_finite() && value >= 0.0)
            && [self.min, self.max, self.target]
                .into_iter()
                .all(f64::is_finite)
            && self.min < self.max
    }

    fn moments(&self, q: f64, v: f64, dt: f64, integral: Option<&mut f64>) -> (f64, f64, f64) {
        let stop = if q < self.min {
            -self.stop_k * (q - self.min) - self.stop_damping * v.min(0.0)
        } else if q > self.max {
            -self.stop_k * (q - self.max) - self.stop_damping * v.max(0.0)
        } else {
            0.0
        };
        let load = integral.map_or(0.0, |integral| {
            let error = self.target - q;
            *integral = (*integral + dt * error).clamp(-self.integral_limit, self.integral_limit);
            (self.kp * error + self.ki * *integral - self.kd * v)
                .clamp(-self.torque_limit, self.torque_limit)
        });
        (load, stop, self.damping * v)
    }
}

impl Spec {
    fn pack(&self) -> Pack {
        let supply = &self.supply;
        let cell = match supply.cell {
            Cell::LgM50 => CellParams::lg_m50(),
        };
        Pack::with_soc(cell, supply.series, supply.parallel, supply.soc)
    }

    /// Показание до первого электрического шага из начального состояния модели.
    pub fn initial_measurement(&self, measurement: Measurement) -> Result<f64, String> {
        self.params()?;
        let value = match measurement {
            Measurement::WindingCurrent => 0.0, // Motor::new начинает с обесточенной обмотки.
            Measurement::HousingTemperature => self.ambient_c,
            Measurement::SupplyVoltage => self.pack().terminal_voltage(),
        };
        if !value.is_finite() {
            return Err("неконечное начальное показание привода".into());
        }
        Ok(value)
    }

    fn params(&self) -> Result<MotorParams, String> {
        let (e, g, t, s) = (&self.motor, &self.gear, &self.thermal, &self.supply);
        let positive = [
            e.nominal_v,
            e.r_ohm,
            e.l_h,
            e.k,
            e.j,
            e.band,
            e.max_a,
            g.ratio,
            g.rated,
            g.repeated,
            g.peak,
            g.mass,
            t.r_wh,
            t.r_ha,
            t.tau_w,
            t.tau_h,
            s.conversion_floor_v,
        ];
        let nonnegative = [
            e.coulomb,
            e.viscous,
            g.friction,
            g.j,
            s.electronics_a,
            s.max_regen_a,
        ];
        if !positive.into_iter().all(|v| v.is_finite() && v > 0.0)
            || !nonnegative.into_iter().all(|v| v.is_finite() && v >= 0.0)
            || ![self.ambient_c, e.r_ref_c, t.max_c]
                .into_iter()
                .all(f64::is_finite)
            || ![g.eta_forward, g.eta_backward]
                .into_iter()
                .all(|v| v > 0.0 && v <= 1.0)
            || !(0.0..=1.0).contains(&t.fraction)
            || !(0.0..=1.0).contains(&s.soc)
            || s.series == 0
            || s.parallel == 0
            || s.series.checked_mul(s.parallel).is_none()
            || (self.fixture_enabled && self.fixture.is_none())
            || self.fixture.as_ref().is_some_and(|f| !f.valid())
            || self.source.trim().is_empty()
        {
            return Err("некорректный параметр дискретного привода".into());
        }
        let params = MotorParams {
            name: "жёсткая передача",
            nominal_voltage_v: e.nominal_v,
            resistance_ohm: e.r_ohm,
            resistance_ref_c: e.r_ref_c,
            inductance_h: e.l_h,
            torque_constant_nm_per_a: e.k,
            rotor_inertia_kg_m2: e.j,
            coulomb_friction_nm: e.coulomb,
            viscous_friction_nm_s_per_rad: e.viscous,
            coulomb_band_rad_s: e.band,
            max_current_a: e.max_a,
            gear: GearParams {
                name: "жёсткая передача",
                ratio: g.ratio,
                efficiency_forward: g.eta_forward,
                efficiency_backward: g.eta_backward,
                input_coulomb_nm: g.friction,
                input_inertia_kg_m2: g.j,
                rated_output_torque_nm: g.rated,
                repeated_peak_output_torque_nm: g.repeated,
                momentary_peak_output_torque_nm: g.peak,
                mass_kg: g.mass,
                source: "параметры задачи",
            },
            thermal: ThermalParams {
                r_winding_housing_k_per_w: t.r_wh,
                r_housing_ambient_k_per_w: t.r_ha,
                tau_winding_s: t.tau_w,
                tau_housing_s: t.tau_h,
                max_winding_temp_c: t.max_c,
                no_load_loss_to_winding: t.fraction,
            },
            datasheet: DatasheetValues::default(),
            sources: &[],
        };
        if !params.resistance_at(self.ambient_c).is_finite()
            || params.resistance_at(self.ambient_c) <= 0.0
        {
            return Err("некорректное сопротивление при начальной температуре".into());
        }
        Ok(params)
    }
}

#[derive(Clone)]
struct Plant {
    spec: Spec,
    motor: Motor,
    pack: Pack,
    joint: String,
    command: ChannelId,
    power: ChannelId,
    ids: [TruthId; 12],
    integral: f64,
    previous: Option<u64>,
}

fn mean_output_torque(
    params: &MotorParams,
    speed: f64,
    start: f64,
    steady: f64,
    h: f64,
    tau: f64,
) -> f64 {
    let friction = params.friction_torque_motor(speed);
    let difference = start - steady;
    let impulse = |time: f64| {
        let charge = steady * time + difference * tau * -(-time / tau).exp_m1();
        params.torque_constant_nm_per_a * charge - friction * time
    };
    let total = impulse(h);
    let threshold = friction / params.torque_constant_nm_per_a;
    let ratio = (threshold - steady) / difference;
    if ratio > 0.0 && ratio < 1.0 {
        let crossing = -tau * ratio.ln();
        if crossing < h {
            let before = impulse(crossing);
            // Формула передачи линейна на каждом направлении потока мощности.
            return (params.gear_output_torque(before, speed)
                + params.gear_output_torque(total - before, speed))
                / h;
        }
    }
    params.gear_output_torque(total, speed) / h
}

impl Object for Plant {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        cmd: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        let (q, v) = truth.joint(&self.joint).ok_or("ось модели отсутствует")?;
        let (q, v) = (q[0], v[0]);
        let h = truth.engine().timestep();
        let (thermal_resistance, thermal_ambient) = thermal_boundary(
            &self.spec.thermal,
            self.spec.ambient_c,
            truth.thermal_boundary(self.command),
            h,
        )?;
        let input = |channel| {
            cmd.targets
                .iter()
                .find(|&&(id, _)| id == channel)
                .map(|&(_, value)| value)
        };
        let duty = input(self.command).unwrap_or(0.0);
        if !(-1.0..=1.0).contains(&duty) {
            return Err("скважность должна быть в диапазоне [−1, 1]".into());
        }
        let enabled = match input(self.power) {
            Some(1.0) => true,
            None | Some(0.0) => false,
            _ => return Err("разрешение моста должно равняться 0 или 1".into()),
        };
        let bus_v = self.pack.terminal_voltage();
        if !bus_v.is_finite() || bus_v <= 0.0 {
            return Err("напряжение шины дискретного привода вне рабочего диапазона".into());
        }
        let i0 = self.motor.current();
        let p = &self.motor.params;
        let r = self.motor.winding_resistance_ohm();
        let tau = p.inductance_h / r;
        let old_emf = self.motor.back_emf_v();
        let a = (-h / tau).exp();
        let voltage = if enabled {
            duty * bus_v
        } else {
            // Диодный выбег. Величина подобрана на конечное значение тока 0
            // при сохранении противо-ЭДС предыдущего такта регулятора моста.
            let zero = old_emf + r * (-i0 * a / (1.0 - a));
            if zero.abs() <= bus_v {
                zero
            } else {
                -i0.signum() * bus_v
            }
        };
        let wm = p.gear.ratio * v;
        let steady = (voltage - p.torque_constant_nm_per_a * wm) / r;
        let mean_torque = mean_output_torque(p, wm, i0, steady, h, tau);
        self.motor.step_electrical(voltage, v, h);
        let torque = self.motor.joint_torque();
        let (load, stop, damping) = self
            .spec
            .fixture
            .as_ref()
            .map_or((0.0, 0.0, 0.0), |fixture| {
                fixture.moments(
                    q,
                    v,
                    h,
                    self.spec.fixture_enabled.then_some(&mut self.integral),
                )
            });
        out.set(self.command, torque + load + stop - damping);
        self.motor
            .set_housing_to_ambient_k_per_w(thermal_resistance);
        self.motor.step_thermal(h, thermal_ambient);
        let end_i = self.motor.current();
        let supply = &self.spec.supply;
        let battery_i = if bus_v > supply.conversion_floor_v {
            (voltage * end_i).max(-supply.max_regen_a * bus_v) / bus_v + supply.electronics_a
        } else {
            supply.electronics_a
        };
        self.pack.step(battery_i, h, self.spec.ambient_c);
        for (id, value) in self.ids.into_iter().zip([
            duty,
            end_i,
            voltage,
            bus_v,
            self.pack.terminal_voltage(),
            self.motor.winding_temp_c(),
            self.motor.housing_temp_c(),
            torque,
            mean_torque,
            f64::from(enabled),
            stop,
            load,
        ]) {
            out.publish(id, value);
        }
        if enabled
            && let Some(sequence) = truth.command_sequence(self.command)
            && self.previous.replace(sequence) != Some(sequence)
        {
            out.accept(self.command, sequence, truth.time(), duty);
        }
        Ok(())
    }
}

use sim_runtime::process::{ProcessAnalysis, ThermalBoundary};

fn thermal_boundary(
    thermal: &Thermal,
    air_c: f64,
    boundary: Option<&ProcessAnalysis<ThermalBoundary>>,
    dt: f64,
) -> Result<(f64, f64), String> {
    let dry = thermal.r_ha;
    match boundary {
        None
        | Some(ProcessAnalysis::Analyzed {
            value: ThermalBoundary::Dry,
            ..
        }) => Ok((dry, air_c)),
        Some(ProcessAnalysis::Analyzed {
            value:
                ThermalBoundary::Wet {
                    fluid_temperature_k,
                    conductance_w_k,
                    ..
                },
            ..
        }) => {
            // Две параллельные проводимости и эквивалентная температура
            // из баланса потоков. 273,15 — сдвиг шкал K и °C по СИ.
            let air = dry.recip();
            let total = air + conductance_w_k;
            let resistance = total.recip();
            let fluid_c = fluid_temperature_k - 273.15;
            let ambient = air / total * air_c + conductance_w_k / total * fluid_c;
            // Наружный шаг не должен превышать новую постоянную времени корпуса.
            // Иначе режим за шаг не разрешается и RK4 потребует неограниченного дробления.
            let housing_tau = resistance * (thermal.tau_h / dry);
            if resistance > 0.0
                && ambient.is_finite()
                && housing_tau.is_finite()
                && dt <= housing_tau
            {
                Ok((resistance, ambient))
            } else {
                Err("некорректная эквивалентная тепловая граница".into())
            }
        }
        Some(ProcessAnalysis::NotAnalyzed { reason }) => Err(format!("тепловая граница: {reason}")),
    }
}

/// Установить привод перед сборкой проводки. Инерция передачи включена в armature.
pub fn install(
    runtime: &mut Runtime,
    world: &CompiledWorld,
    drive_name: &str,
    spec: Spec,
) -> Result<[ModelPort; 2], String> {
    let drive = world
        .drives
        .iter()
        .find(|drive| drive.name == drive_name)
        .ok_or("привод из профиля отсутствует в мире")?;
    let valid_axis = world
        .joints
        .iter()
        .any(|joint| joint.name == drive.joint && joint.kind == CompiledJointKind::Hinge);
    if !valid_axis {
        return Err("дискретной модели жёсткой передачи нужна вращательная ось".into());
    }
    let params = spec.params()?;
    let expected = params.reflected_inertia();
    let actual = runtime
        .joint_armature(&drive.joint, 0)
        .ok_or("нет инерции оси")?;
    // Запас округления для сложения и двух умножений при приведении инерции.
    if !expected.is_finite()
        || (expected - actual).abs() > 16.0 * f64::EPSILON * expected.max(actual)
    {
        return Err("armature не соответствует приведённой инерции мотора и передачи".into());
    }
    let pack = spec.pack();
    let motor = Motor::new(params).with_temperatures(spec.ambient_c, spec.ambient_c);
    let command = runtime.channel(&drive.effort).ok_or("нет канала привода")?;
    let power_name = format!("{}/{}", drive.owner, spec.bridge_effort);
    if !world
        .efforts
        .iter()
        .any(|effort| effort.name == power_name && effort.joint == drive.joint && effort.dof == 0)
        || power_name == drive.effort
    {
        return Err("вход моста должен относиться к той же оси".into());
    }
    let power = runtime.channel(&power_name).ok_or("нет входа моста")?;
    let before = runtime.try_snapshot()?;
    let result = (|| {
        let ids: [TruthId; 12] = [
            ("accepted_command", "1"),
            ("current_a", "A"),
            ("voltage_v", "V"),
            ("bus_voltage_v", "V"),
            ("bus_next_v", "V"),
            ("winding_c", "degC"),
            ("housing_c", "degC"),
            ("joint_torque_nm", "N*m"),
            ("mean_torque_nm", "N*m"),
            ("bridge_enabled", "bool"),
            ("stop_torque_nm", "N*m"),
            ("fixture_torque_nm", "N*m"),
        ]
        .map(|(name, unit)| runtime.register_truth(&format!("{drive_name}/{name}"), unit))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .expect("фиксированный список");
        runtime.add_simplification(Simplification {
            subject: drive_name.into(),
            text: format!("{} Механика использует момент по конечному току вместо интеграла импульса. Модель питания и упоров требует стендовой сверки.", spec.source),
            blocks_proof: true,
        })?;
        runtime.register_object(
            &format!("{drive_name}/дискретная модель"),
            world.physics,
            &[command, power],
            Plant {
                spec,
                motor,
                pack,
                joint: drive.joint.clone(),
                command,
                power,
                ids,
                integral: 0.0,
                previous: None,
            },
        )?;
        Ok([
            ModelPort::Drive {
                channel: command,
                accepted: ids[0],
            },
            ModelPort::DrivePower {
                channel: power,
                drive: command,
            },
        ])
    })();
    if result.is_err() {
        runtime.restore(&before)?;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_em::motor::ThermalNet;

    #[test]
    fn тепловая_граница_сохраняет_потоки_и_возвращает_сухой_режим() {
        // Искусственная двухузловая сеть: 2 и 4 K/Вт, 4 и 8 Дж/K.
        // Подключение жидкости 5 °C добавляет канал 0,75 Вт/K.
        let thermal = Thermal {
            r_wh: 2.0,
            r_ha: 4.0,
            tau_w: 8.0,
            tau_h: 32.0,
            max_c: 100.0,
            fraction: 0.0,
        };
        let dt = 0.01;
        let air_c = 25.0;
        let wet = |conductance_w_k| ProcessAnalysis::Analyzed {
            value: ThermalBoundary::Wet {
                fluid_temperature_k: 278.15,
                conductance_w_k,
                heat_out_w: 15.0,
            },
            scope: "аналитическая сеть контрольного стенда".into(),
        };
        let (r_ha, ambient) = thermal_boundary(&thermal, air_c, Some(&wet(0.75)), dt).unwrap();
        assert_eq!(r_ha, 1.0);
        assert_eq!(ambient, 10.0);
        assert_eq!((air_c - ambient) / r_ha, 0.75 * (25.0 - 5.0));
        let net = ThermalNet {
            r_wh: thermal.r_wh,
            r_ha,
            c_w: 4.0,
            c_h: 8.0,
        };
        let rk4 = net.integrate((25.0, 25.0), dt, ambient, |_| (0.0, 0.0));
        let exact = net.exact_constant_power((25.0, 25.0), dt, ambient, 0.0, 0.0);
        // Ошибка одного шага RK4 масштаба (dt/τ)^5 для этой сети.
        assert!((rk4.0 - exact.0).abs() < 1e-11 && (rk4.1 - exact.1).abs() < 1e-11);
        let dry = ProcessAnalysis::Analyzed {
            value: ThermalBoundary::Dry,
            scope: "сухая сеть".into(),
        };
        assert_eq!(
            thermal_boundary(&thermal, air_c, Some(&dry), dt).unwrap(),
            (4.0, 25.0)
        );
        let missing = ProcessAnalysis::NotAnalyzed {
            reason: "нет замера теплоотдачи".into(),
        };
        assert!(thermal_boundary(&thermal, air_c, Some(&missing), dt).is_err());
        assert!(thermal_boundary(&thermal, air_c, Some(&wet(f64::MAX)), dt).is_err());
    }
}
