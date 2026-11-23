//! Привод сустава: двигатель с постоянными магнитами + редуктор + тепловая модель.
//!
//! Электромеханика (эквивалент коллекторного двигателя постоянного тока):
//! V = R(T)·i + L·di/dt + k·ω_м, τ_эм = k·i, где k = K_t = K_e (в СИ это одно число —
//! иначе нарушается закон сохранения энергии). BLDC/PMSM приводится к той же схеме:
//! R и L — «межфазные» (terminal, phase-to-phase), i — эквивалентный ток звена
//! (блочная коммутация) или амплитуда тока q-оси при векторном управлении с i_d = 0,
//! пересчитанная так, чтобы K_t·i давал момент. Так задают данные maxon и CubeMars
//! (у обоих K_m = K_t/√R_межфазн. сходится с их же таблицами). Пульсации момента,
//! коммутация, ослабление поля, насыщение магнитопровода НЕ моделируются.
//!
//! Трение двигателя: Кулоново T_c + вязкое b·ω (по току холостого хода, разделение —
//! «оценка»), Кулоново регуляризовано в полосе |ω| < ω_s (линейный участок) — стандарт
//! для ко-симуляции с заданной скоростью: точное залипание при ω = 0 здесь не описать,
//! это дело многотельного решателя.
//!
//! Редуктор: передаточное число N, КПД зависит от направления потока мощности:
//! двигатель ведёт нагрузку — τ_сустав = N·η_пр·τ_вал; нагрузка ведёт двигатель
//! (торможение, обратный привод) — τ_сустав = N·τ_вал/η_обр. По умолчанию
//! η_обр = 2 − 1/η_пр: так выходит для зацепления с кулоновым трением, если потери
//! в прямом и обратном ходе имеют одну природу (вывод для червячных/планетарных пар:
//! прямой КПД tanλ/tan(λ+φ), обратный tan(λ−φ)/tanλ; при малом φ η_обр ≈ 2 − 1/η_пр).
//! Это допущение; при η_пр < 0.5 редуктор становится самотормозящим (η_обр ≤ 0).
//! Приведённая инерция J·N² отдаётся многотельной модели (как «armature»), КПД на
//! инерционный момент не накладывается — упрощение.
//!
//! Тепло: два узла «обмотка → корпус → среда» (R_th1, R_th2; ёмкости из постоянных
//! времени даташита C = τ/R — трактовка maxon). Потери в меди R(T)·i² (α_Cu = 0.00393 1/К)
//! идут в обмотку, потери трения/железа делятся между обмоткой и корпусом (доля —
//! «оценка»). Сопротивление «корпус → среда» можно заменить (обдув, вода) — ёмкости
//! при этом не меняются.

use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use sim_verify::{Check, Tolerance};
use std::f64::consts::PI;

use crate::magcircuit::ALPHA_CU;

/// Перевод об/мин → рад/с.
pub const RPM: f64 = 2.0 * PI / 60.0;

// ---------------------------------------------------------------------------------------
// Параметры
// ---------------------------------------------------------------------------------------

/// Редуктор.
#[derive(Clone, Debug)]
pub struct GearParams {
    pub name: &'static str,
    pub ratio: f64,
    /// КПД зацепления, когда двигатель ведёт нагрузку.
    pub efficiency_forward: f64,
    /// КПД при обратном потоке мощности (нагрузка ведёт двигатель).
    pub efficiency_backward: f64,
    /// Кулоново трение редуктора, приведённое ко входу, Н·м.
    pub input_coulomb_nm: f64,
    /// Момент инерции входного звена (генератор волн / солнце), кг·м².
    pub input_inertia_kg_m2: f64,
    /// Допустимые моменты на выходе, Н·м: номинальный, повторяющийся пик, мгновенный пик.
    pub rated_output_torque_nm: f64,
    pub repeated_peak_output_torque_nm: f64,
    pub momentary_peak_output_torque_nm: f64,
    pub mass_kg: f64,
    pub source: &'static str,
}

/// Обратный КПД по прямому при допущении «одна природа потерь»: η_обр = 2 − 1/η_пр.
pub fn backward_efficiency_from_forward(eta_f: f64) -> f64 {
    2.0 - 1.0 / eta_f
}

/// Тепловая модель двигателя.
#[derive(Clone, Debug)]
pub struct ThermalParams {
    pub r_winding_housing_k_per_w: f64,
    pub r_housing_ambient_k_per_w: f64,
    pub tau_winding_s: f64,
    pub tau_housing_s: f64,
    pub max_winding_temp_c: f64,
    /// Доля потерь холостого хода (трение, железо), выделяющаяся в обмотке/статоре.
    pub no_load_loss_to_winding: f64,
}

/// Значения из даташита — только для сверки с расчётом из базовых параметров.
#[derive(Clone, Debug, Default)]
pub struct DatasheetValues {
    pub no_load_speed_rpm: Option<f64>,
    pub no_load_current_a: Option<f64>,
    pub nominal_speed_rpm: Option<f64>,
    pub nominal_torque_nm: Option<f64>,
    pub nominal_current_a: Option<f64>,
    pub stall_torque_nm: Option<f64>,
    pub stall_current_a: Option<f64>,
    pub max_efficiency: Option<f64>,
    pub speed_constant_rpm_per_v: Option<f64>,
    pub back_emf_v_per_krpm: Option<f64>,
    pub speed_torque_gradient_rpm_per_mnm: Option<f64>,
    pub mech_time_constant_s: Option<f64>,
    pub elec_time_constant_s: Option<f64>,
    pub motor_constant_nm_per_sqrt_w: Option<f64>,
    /// Для мотор-редукторов: момент/скорость/ток на выходе.
    pub rated_joint_torque_nm: Option<f64>,
    pub peak_joint_torque_nm: Option<f64>,
    pub rated_current_a: Option<f64>,
    pub peak_current_a: Option<f64>,
    pub joint_no_load_speed_rpm: Option<f64>,
    pub backdrive_torque_nm: Option<f64>,
}

/// Параметры привода (двигатель + редуктор + тепло).
#[derive(Clone, Debug)]
pub struct MotorParams {
    pub name: &'static str,
    pub nominal_voltage_v: f64,
    /// Межфазное сопротивление при `resistance_ref_c`, Ом.
    pub resistance_ohm: f64,
    pub resistance_ref_c: f64,
    /// Межфазная индуктивность, Гн.
    pub inductance_h: f64,
    /// K_t = K_e, Н·м/А = В·с/рад.
    pub torque_constant_nm_per_a: f64,
    pub rotor_inertia_kg_m2: f64,
    /// Трение двигателя (сторона двигателя).
    pub coulomb_friction_nm: f64,
    pub viscous_friction_nm_s_per_rad: f64,
    /// Полуширина полосы регуляризации Кулонова трения, рад/с (сторона двигателя).
    pub coulomb_band_rad_s: f64,
    /// Предельный ток драйвера/двигателя, А (для режима токового управления).
    pub max_current_a: f64,
    pub gear: GearParams,
    pub thermal: ThermalParams,
    pub datasheet: DatasheetValues,
    pub sources: &'static [&'static str],
}

impl MotorParams {
    /// maxon EC 90 flat Ø90 мм, 260 Вт, 48 В, с датчиками Холла (арт. 500267) +
    /// Harmonic Drive CSG-25-50-2A-GR (волновая передача 50:1).
    ///
    /// Для локтевого экзоскелета: ~35 Н·м длительно на суставе (номинальный момент 0.964 Н·м
    /// × 50 × η), пик ограничен редуктором (повторяющийся 127 Н·м), скорость сустава при
    /// номинальной скорости двигателя 1670/50 = 33 об/мин = 200 °/с.
    ///
    /// Двигатель — данные maxon (страница изделия, 2026; тепловые данные — там же).
    /// Пусковой момент 13.1 Н·м — каталог maxon EC flat 2022 (текущая страница даёт
    /// 9.41 Н·м — см. [`datasheet_discrepancies`]). Разделение трения холостого хода
    /// пополам на Кулоново и вязкое — «оценка». Редуктор: передаточное число, моменты,
    /// масса, пусковой момент 13 Н·сМ (вход) — страница HD; КПД зацепления 0.85 — «оценка»
    /// (каталожные кривые КПД CSG не удалось прочитать в машинном виде; типовые значения
    /// волновых передач при номинальном моменте 70–85 %); инерция входа 0.413·10⁻⁴ кг·м²
    /// — «оценка» по каталогу CSF/CSG (типоразмер 25), для i = 50 не подтверждена.
    pub fn maxon_ec90flat_260w_hd_csg25_50() -> Self {
        let kt = 0.231;
        let i0 = 0.278;
        let n0 = 1960.0 * RPM;
        let t_noload = kt * i0; // момент трения на холостом ходу
        let eta_f = 0.85;
        Self {
            name: "maxon EC 90 flat 260 Вт 48 В (500267) + Harmonic Drive CSG-25-50",
            nominal_voltage_v: 48.0,
            resistance_ohm: 0.844,
            resistance_ref_c: 25.0,
            inductance_h: 1.07e-3,
            torque_constant_nm_per_a: kt,
            rotor_inertia_kg_m2: 5060e-7,
            coulomb_friction_nm: 0.5 * t_noload,
            viscous_friction_nm_s_per_rad: 0.5 * t_noload / n0,
            coulomb_band_rad_s: 0.5,
            max_current_a: 15.0,
            gear: GearParams {
                name: "Harmonic Drive CSG-25-50-2A-GR",
                ratio: 50.0,
                efficiency_forward: eta_f,
                efficiency_backward: backward_efficiency_from_forward(eta_f),
                input_coulomb_nm: 0.13,
                input_inertia_kg_m2: 0.413e-4,
                rated_output_torque_nm: 51.0,
                repeated_peak_output_torque_nm: 127.0,
                momentary_peak_output_torque_nm: 242.0,
                mass_kg: 0.42,
                source: "https://www.harmonicdrive.net/products/component-sets/cup-type/csg-2a/csg-25-50-2a-gr",
            },
            thermal: ThermalParams {
                r_winding_housing_k_per_w: 1.82,
                r_housing_ambient_k_per_w: 1.78,
                tau_winding_s: 54.3,
                tau_housing_s: 265.0,
                max_winding_temp_c: 125.0,
                no_load_loss_to_winding: 0.5,
            },
            datasheet: DatasheetValues {
                no_load_speed_rpm: Some(1960.0),
                no_load_current_a: Some(0.278),
                nominal_speed_rpm: Some(1670.0),
                nominal_torque_nm: Some(0.964),
                nominal_current_a: Some(4.06),
                stall_torque_nm: Some(13.1),
                stall_current_a: Some(56.9),
                max_efficiency: Some(0.87),
                speed_constant_rpm_per_v: Some(41.3),
                speed_torque_gradient_rpm_per_mnm: Some(0.151),
                mech_time_constant_s: Some(7.99e-3),
                backdrive_torque_nm: Some(8.3),
                ..Default::default()
            },
            sources: &[
                "maxon, EC 90 flat Ø90 mm, brushless, 260 W, part 500267: https://www.maxongroup.com/maxon/view/product/motor/ecmotor/ecflat/ecflat90/500267",
                "maxon EC flat catalog 2022 (stall torque 13100 mNm @48 V): https://maxonjapan.com/wp-content/uploads/2022/10/EC_flat.pdf",
                "Harmonic Drive CSG-25-50-2A-GR: https://www.harmonicdrive.net/products/component-sets/cup-type/csg-2a/csg-25-50-2a-gr",
            ],
        }
    }

    /// CubeMars AK80-64 KV80 — встроенный привод (BLDC + планетарный 64:1 + драйвер),
    /// сделан в т.ч. для экзоскелетов: 48 Н·м номинал, 120 Н·м пик на выходе.
    ///
    /// Из данных производителя: R = 0.22 Ом и L = 133.5 мкГн (межфазные), K_t = 0.136 Н·м/А
    /// (сторона двигателя: подтверждается τ_мех = R·J/K_t² = 0.67 мс из той же таблицы),
    /// J ротора 564.5 г·см², N = 64, номинальный/пиковый ток 7/19 А. Тепловых данных нет:
    /// T_max обмотки 100 °C, R_th и τ — «оценка» (R_th подобрано так, чтобы номинальный ток
    /// 7 А при 25 °C давал T_max: R_th1 + R_th2 = 5.37 К/Вт; τ — по массе 0.85 кг).
    /// КПД планетарного редуктора 0.85 — «оценка» (2 ступени по ~0.92); Кулоново трение
    /// на входе выбрано по «Back drive 4.7 Н·м» (калибровка, не сверка).
    pub fn cubemars_ak80_64() -> Self {
        let n = 64.0;
        let eta_f = 0.85;
        let eta_b = backward_efficiency_from_forward(eta_f);
        let backdrive = 4.7;
        Self {
            name: "CubeMars AK80-64 KV80",
            nominal_voltage_v: 48.0,
            resistance_ohm: 0.22,
            resistance_ref_c: 25.0,
            inductance_h: 133.5e-6,
            torque_constant_nm_per_a: 0.136,
            rotor_inertia_kg_m2: 564.5e-7,
            coulomb_friction_nm: 0.0,
            viscous_friction_nm_s_per_rad: 0.0,
            coulomb_band_rad_s: 0.5,
            max_current_a: 19.0,
            gear: GearParams {
                name: "планетарный 64:1 (встроенный)",
                ratio: n,
                efficiency_forward: eta_f,
                efficiency_backward: eta_b,
                input_coulomb_nm: backdrive * eta_b / n,
                input_inertia_kg_m2: 0.0,
                rated_output_torque_nm: 48.0,
                repeated_peak_output_torque_nm: 120.0,
                momentary_peak_output_torque_nm: 120.0,
                mass_kg: 0.85,
                source: "https://www.cubemars.com/product/ak80-64-kv80-robotic-actuator.html",
            },
            thermal: ThermalParams {
                r_winding_housing_k_per_w: 1.5,
                r_housing_ambient_k_per_w: 3.87,
                tau_winding_s: 60.0,
                tau_housing_s: 1600.0,
                max_winding_temp_c: 100.0,
                no_load_loss_to_winding: 0.5,
            },
            datasheet: DatasheetValues {
                speed_constant_rpm_per_v: Some(80.0),
                back_emf_v_per_krpm: Some(13.7),
                mech_time_constant_s: Some(0.67e-3),
                elec_time_constant_s: Some(0.61e-3),
                motor_constant_nm_per_sqrt_w: Some(0.29),
                rated_joint_torque_nm: Some(48.0),
                peak_joint_torque_nm: Some(120.0),
                rated_current_a: Some(7.0),
                peak_current_a: Some(19.0),
                joint_no_load_speed_rpm: Some(75.0),
                backdrive_torque_nm: Some(backdrive),
                ..Default::default()
            },
            sources: &[
                "CubeMars AK80-64 KV80: https://www.cubemars.com/product/ak80-64-kv80-robotic-actuator.html",
            ],
        }
    }

    /// Сопротивление обмотки при температуре `t_c`.
    pub fn resistance_at(&self, t_c: f64) -> f64 {
        self.resistance_ohm * (1.0 + ALPHA_CU * (t_c - self.resistance_ref_c))
    }

    /// Приведённая к суставу инерция (ротор + вход редуктора)·N², кг·м².
    pub fn reflected_inertia(&self) -> f64 {
        (self.rotor_inertia_kg_m2 + self.gear.input_inertia_kg_m2)
            * self.gear.ratio
            * self.gear.ratio
    }

    /// Трение самого двигателя (подшипники, потери в железе как момент) — его тепло
    /// идёт в тепловую модель двигателя.
    pub fn motor_friction_nm(&self, omega_m: f64) -> f64 {
        let s = (omega_m / self.coulomb_band_rad_s).clamp(-1.0, 1.0);
        self.coulomb_friction_nm * s + self.viscous_friction_nm_s_per_rad * omega_m
    }

    /// Кулоново трение редуктора, приведённое ко входу (его тепло — в редукторе,
    /// в тепловую модель двигателя не идёт).
    pub fn gear_coulomb_nm(&self, omega_m: f64) -> f64 {
        self.gear.input_coulomb_nm * (omega_m / self.coulomb_band_rad_s).clamp(-1.0, 1.0)
    }

    /// Полный момент трения на стороне двигателя (двигатель + Кулоново редуктора).
    pub fn friction_torque_motor(&self, omega_m: f64) -> f64 {
        self.motor_friction_nm(omega_m) + self.gear_coulomb_nm(omega_m)
    }

    /// Момент на выходе редуктора по моменту на его входе τ_вал (после трения двигателя).
    pub fn gear_output_torque(&self, tau_shaft: f64, omega_m: f64) -> f64 {
        let n = self.gear.ratio;
        if tau_shaft * omega_m >= 0.0 {
            n * self.gear.efficiency_forward * tau_shaft
        } else {
            n * tau_shaft / self.gear.efficiency_backward
        }
    }

    // ---------------- установившиеся режимы (сторона двигателя, как в даташитах) ------

    /// Пусковой ток V/R (холодная обмотка, `resistance_ref_c`).
    pub fn stall_current_a(&self, v: f64) -> f64 {
        v / self.resistance_ohm
    }
    /// Пусковой момент k·V/R (без трения — как печатают производители).
    pub fn stall_torque_motor_nm(&self, v: f64) -> f64 {
        self.torque_constant_nm_per_a * v / self.resistance_ohm
    }
    /// Скорость холостого хода двигателя (без редуктора, трение только двигателя):
    /// V = R·i + kω, k·i = T_c + bω → ω = (V − R·T_c/k)/(k + R·b/k).
    pub fn no_load_speed_motor_rad_s(&self, v: f64) -> f64 {
        let k = self.torque_constant_nm_per_a;
        let r = self.resistance_ohm;
        (v - r * self.coulomb_friction_nm / k) / (k + r * self.viscous_friction_nm_s_per_rad / k)
    }
    /// Скорость холостого хода сустава (с трением редуктора), рад/с.
    pub fn no_load_speed_joint_rad_s(&self, v: f64) -> f64 {
        let k = self.torque_constant_nm_per_a;
        let r = self.resistance_ohm;
        let tc = self.coulomb_friction_nm + self.gear.input_coulomb_nm;
        (v - r * tc / k) / (k + r * self.viscous_friction_nm_s_per_rad / k) / self.gear.ratio
    }
    /// Прямая «момент–скорость» двигателя: полезный момент на валу при скорости ω > 0.
    pub fn shaft_torque_at_speed(&self, v: f64, omega_m: f64) -> f64 {
        let k = self.torque_constant_nm_per_a;
        k * (v - k * omega_m) / self.resistance_ohm
            - self.coulomb_friction_nm * omega_m.signum()
            - self.viscous_friction_nm_s_per_rad * omega_m
    }
    /// Характеристика сустава (ω_сустава рад/с, τ_сустава Н·м) от упора до холостого хода.
    pub fn torque_speed_curve_joint(&self, v: f64, points: usize) -> Vec<(f64, f64)> {
        let w0 = self.no_load_speed_joint_rad_s(v);
        (0..points)
            .map(|j| {
                let wj = w0 * j as f64 / (points - 1).max(1) as f64;
                let wm = wj * self.gear.ratio;
                let k = self.torque_constant_nm_per_a;
                let tau_shaft = k * (v - k * wm) / self.resistance_ohm
                    - (self.coulomb_friction_nm + self.gear.input_coulomb_nm)
                    - self.viscous_friction_nm_s_per_rad * wm;
                (wj, self.gear_output_torque(tau_shaft.max(0.0), wm.max(0.0)))
            })
            .collect()
    }
    /// Точка максимального КПД двигателя (без редуктора) при напряжении v:
    /// максимизация η(ω) = τ_вал·ω/(V·i) золотым сечением на (0, ω₀).
    pub fn max_efficiency_point(&self, v: f64) -> EfficiencyPoint {
        let k = self.torque_constant_nm_per_a;
        let r = self.resistance_ohm;
        let eff = |w: f64| {
            let i = (v - k * w) / r;
            let tau = self.shaft_torque_at_speed(v, w);
            if i <= 0.0 { 0.0 } else { tau * w / (v * i) }
        };
        let (mut a, mut b) = (0.0, self.no_load_speed_motor_rad_s(v));
        let gr = (5f64.sqrt() - 1.0) / 2.0;
        let (mut c, mut d) = (b - gr * (b - a), a + gr * (b - a));
        let (mut fc, mut fd) = (eff(c), eff(d));
        for _ in 0..200 {
            if fc > fd {
                b = d;
                d = c;
                fd = fc;
                c = b - gr * (b - a);
                fc = eff(c);
            } else {
                a = c;
                c = d;
                fc = fd;
                d = a + gr * (b - a);
                fd = eff(d);
            }
            if (b - a).abs() < 1e-12 * self.no_load_speed_motor_rad_s(v) {
                break;
            }
        }
        let w = 0.5 * (a + b);
        EfficiencyPoint {
            speed_rad_s: w,
            torque_nm: self.shaft_torque_at_speed(v, w),
            current_a: (v - k * w) / r,
            efficiency: eff(w),
        }
    }

    /// Максимальный длительный ток при температуре среды и скорости двигателя
    /// `omega_m` (с потерями холостого хода), по стационару тепловой модели:
    /// T_max = T_a + P_w·(R₁+R₂) + P_h·R₂, P_w = I²R(T_max) + s·P_тр, P_h = (1−s)·P_тр.
    /// `r_housing_ambient` — текущее сопротивление «корпус → среда».
    pub fn continuous_current_a(
        &self,
        t_ambient_c: f64,
        omega_m: f64,
        r_housing_ambient: f64,
    ) -> f64 {
        let th = &self.thermal;
        let p_fr = self.motor_friction_nm(omega_m) * omega_m;
        let s = th.no_load_loss_to_winding;
        let r_tot = th.r_winding_housing_k_per_w + r_housing_ambient;
        let budget = th.max_winding_temp_c
            - t_ambient_c
            - s * p_fr * r_tot
            - (1.0 - s) * p_fr * r_housing_ambient;
        if budget <= 0.0 {
            return 0.0;
        }
        (budget / (r_tot * self.resistance_at(th.max_winding_temp_c))).sqrt()
    }
}

/// Формула снижения длительного тока maxon при температуре среды T_a ≠ 25 °C
/// (тепловые сопротивления те же, обмотка на пределе T_max):
/// I(T_a) = I_N·√((T_max − T_a)/(T_max − 25 °C)).
pub fn derated_current_maxon(i_nominal: f64, t_max_c: f64, t_ambient_c: f64) -> f64 {
    i_nominal * ((t_max_c - t_ambient_c).max(0.0) / (t_max_c - 25.0)).sqrt()
}

/// Точка максимального КПД.
#[derive(Clone, Copy, Debug)]
pub struct EfficiencyPoint {
    pub speed_rad_s: f64,
    pub torque_nm: f64,
    pub current_a: f64,
    pub efficiency: f64,
}

// ---------------------------------------------------------------------------------------
// Энергетический баланс
// ---------------------------------------------------------------------------------------

/// Накопленные энергии, Дж (знак «+» — по направлению «из сети к суставу»).
#[derive(Clone, Copy, Debug, Default)]
pub struct EnergyLedger {
    /// ∫V·i dt — из источника питания.
    pub electrical_in_j: f64,
    /// ∫R·i² dt.
    pub copper_j: f64,
    /// ∫(τ_тр·ω) dt — трение двигателя и Кулоново редуктора.
    pub friction_j: f64,
    /// Потери в зацеплении редуктора.
    pub gear_j: f64,
    /// ∫τ_сустав·ω_сустав dt — механическая работа на суставе.
    pub mechanical_out_j: f64,
    /// Энергия магнитного поля обмотки ½·L·i² — на начало учёта.
    pub magnetic_start_j: f64,
}

impl EnergyLedger {
    /// Невязка баланса: вход − (медь + трение + редуктор + выход + ΔW_маг).
    pub fn residual_j(&self, magnetic_now_j: f64) -> f64 {
        self.electrical_in_j
            - (self.copper_j
                + self.friction_j
                + self.gear_j
                + self.mechanical_out_j
                + (magnetic_now_j - self.magnetic_start_j))
    }
}

// ---------------------------------------------------------------------------------------
// Двигатель для ко-симуляции
// ---------------------------------------------------------------------------------------

/// Состояние привода для пошаговой ко-симуляции с многотельной моделью.
#[derive(Clone, Debug)]
pub struct Motor {
    pub params: MotorParams,
    current_a: f64,
    omega_motor: f64,
    t_winding_c: f64,
    t_housing_c: f64,
    r_housing_ambient: f64,
    c_winding: f64,
    c_housing: f64,
    pending_copper_j: f64,
    pending_noload_j: f64,
    pending_time_s: f64,
    ledger: EnergyLedger,
}

impl Motor {
    /// Новый привод: ток 0, обмотка и корпус при 25 °C.
    pub fn new(params: MotorParams) -> Self {
        let th = &params.thermal;
        let c_w = th.tau_winding_s / th.r_winding_housing_k_per_w;
        let c_h = th.tau_housing_s / th.r_housing_ambient_k_per_w;
        let r_ha = th.r_housing_ambient_k_per_w;
        Self {
            params,
            current_a: 0.0,
            omega_motor: 0.0,
            t_winding_c: 25.0,
            t_housing_c: 25.0,
            r_housing_ambient: r_ha,
            c_winding: c_w,
            c_housing: c_h,
            pending_copper_j: 0.0,
            pending_noload_j: 0.0,
            pending_time_s: 0.0,
            ledger: EnergyLedger::default(),
        }
    }

    /// Задать начальные температуры обмотки и корпуса.
    pub fn with_temperatures(mut self, winding_c: f64, housing_c: f64) -> Self {
        self.t_winding_c = winding_c;
        self.t_housing_c = housing_c;
        self
    }

    /// Шаг электрической части: напряжение на зажимах `v_terminal` (В) и скорость сустава
    /// `omega_joint` (рад/с, задаёт многотельная модель) постоянны на шаге `dt`.
    /// Ток интегрируется ТОЧНО (экспонента с τ_э = L/R) — устойчиво при любом dt,
    /// в т.ч. dt ≫ L/R (у AK80-64 L/R = 0.6 мс). R берётся при текущей температуре обмотки.
    pub fn step_electrical(&mut self, v_terminal: f64, omega_joint: f64, dt: f64) {
        let p = &self.params;
        let k = p.torque_constant_nm_per_a;
        let wm = omega_joint * p.gear.ratio;
        let r = p.resistance_at(self.t_winding_c);
        let l = p.inductance_h;
        let i0 = self.current_a;
        let i_ss = (v_terminal - k * wm) / r;
        let (i1, int_i, int_i2) = if l > 0.0 {
            let tau = l / r;
            let a = (-dt / tau).exp();
            let one_m_a = -(-dt / tau).exp_m1();
            let one_m_a2 = -(-2.0 * dt / tau).exp_m1();
            let d = i0 - i_ss;
            (
                i_ss + d * a,
                i_ss * dt + d * tau * one_m_a,
                i_ss * i_ss * dt + 2.0 * i_ss * d * tau * one_m_a + d * d * 0.5 * tau * one_m_a2,
            )
        } else {
            (i_ss, i_ss * dt, i_ss * i_ss * dt)
        };
        let p_fric = p.friction_torque_motor(wm) * wm;
        let p_conv = k * wm * int_i / dt; // средняя электромеханическая мощность
        let p_shaft = p_conv - p_fric;
        let (p_out, p_gear) = if p_shaft >= 0.0 {
            (
                p.gear.efficiency_forward * p_shaft,
                (1.0 - p.gear.efficiency_forward) * p_shaft,
            )
        } else {
            (
                p_shaft / p.gear.efficiency_backward,
                (1.0 / p.gear.efficiency_backward - 1.0) * (-p_shaft),
            )
        };
        self.ledger.electrical_in_j += v_terminal * int_i;
        self.ledger.copper_j += r * int_i2;
        self.ledger.friction_j += p_fric * dt;
        self.ledger.gear_j += p_gear * dt;
        self.ledger.mechanical_out_j += p_out * dt;
        self.pending_copper_j += r * int_i2;
        self.pending_noload_j += p.motor_friction_nm(wm) * wm * dt;
        self.pending_time_s += dt;
        self.current_a = i1;
        self.omega_motor = wm;
    }

    /// Шаг с идеальным токовым контуром: драйвер выставляет напряжение, чтобы за шаг
    /// выйти на ток `i_cmd` (ограниченный `max_current_a`), но не больше `v_bus` по модулю.
    /// Возвращает приложенное напряжение.
    pub fn step_current(&mut self, i_cmd: f64, omega_joint: f64, dt: f64, v_bus: f64) -> f64 {
        let p = &self.params;
        let i_cmd = i_cmd.clamp(-p.max_current_a, p.max_current_a);
        let k = p.torque_constant_nm_per_a;
        let wm = omega_joint * p.gear.ratio;
        let r = p.resistance_at(self.t_winding_c);
        let l = p.inductance_h;
        // напряжение, при котором точная экспонента приходит в i_cmd за dt
        let v = if l > 0.0 {
            let a = (-dt * r / l).exp();
            let one_m_a = -(-dt * r / l).exp_m1();
            let i_ss = (i_cmd - self.current_a * a) / one_m_a;
            r * i_ss + k * wm
        } else {
            r * i_cmd + k * wm
        };
        let v = v.clamp(-v_bus, v_bus);
        self.step_electrical(v, omega_joint, dt);
        v
    }

    /// Электромагнитный момент k·i, Н·м (сторона двигателя).
    pub fn motor_torque(&self) -> f64 {
        self.params.torque_constant_nm_per_a * self.current_a
    }

    /// Момент, который привод прикладывает к суставу, Н·м (после трения и КПД редуктора;
    /// БЕЗ инерции ротора — её даёт [`Motor::reflected_inertia`] многотельной модели).
    pub fn joint_torque(&self) -> f64 {
        let p = &self.params;
        let tau_shaft = self.motor_torque() - p.friction_torque_motor(self.omega_motor);
        p.gear_output_torque(tau_shaft, self.omega_motor)
    }

    pub fn current(&self) -> f64 {
        self.current_a
    }
    /// Скорость двигателя последнего шага, рад/с.
    pub fn motor_speed(&self) -> f64 {
        self.omega_motor
    }
    pub fn reflected_inertia(&self) -> f64 {
        self.params.reflected_inertia()
    }
    pub fn winding_resistance_ohm(&self) -> f64 {
        self.params.resistance_at(self.t_winding_c)
    }
    /// Мгновенные потери в меди R(T)·i², Вт.
    pub fn copper_loss_w(&self) -> f64 {
        self.winding_resistance_ohm() * self.current_a * self.current_a
    }
    /// Противо-ЭДС k·ω_м, В.
    pub fn back_emf_v(&self) -> f64 {
        self.params.torque_constant_nm_per_a * self.omega_motor
    }
    /// Энергия магнитного поля обмотки ½Li², Дж.
    pub fn magnetic_energy_j(&self) -> f64 {
        0.5 * self.params.inductance_h * self.current_a * self.current_a
    }
    /// Накопленный энергобаланс с момента создания или [`Motor::reset_energy`].
    pub fn energy(&self) -> &EnergyLedger {
        &self.ledger
    }
    /// Обнулить энергобаланс (начальная энергия поля — текущая ½Li²).
    pub fn reset_energy(&mut self) {
        self.ledger = EnergyLedger {
            magnetic_start_j: self.magnetic_energy_j(),
            ..Default::default()
        };
    }

    /// Шаг тепловой модели на `dt` при температуре среды `t_ambient_c`. Мощности —
    /// средние по электрическим шагам с прошлого вызова (энергетически согласованно);
    /// если электрических шагов не было — мгновенные R(T)i² и трение.
    /// Интегрирование — RK4 с подшагом ≤ min(τ)/20.
    pub fn step_thermal(&mut self, dt: f64, t_ambient_c: f64) {
        let (p_cu, p_nl) = if self.pending_time_s > 0.0 {
            (
                self.pending_copper_j / self.pending_time_s,
                self.pending_noload_j / self.pending_time_s,
            )
        } else {
            let wm = self.omega_motor;
            (self.copper_loss_w(), self.params.motor_friction_nm(wm) * wm)
        };
        self.pending_copper_j = 0.0;
        self.pending_noload_j = 0.0;
        self.pending_time_s = 0.0;
        let s = self.params.thermal.no_load_loss_to_winding;
        let net = ThermalNet {
            r_wh: self.params.thermal.r_winding_housing_k_per_w,
            r_ha: self.r_housing_ambient,
            c_w: self.c_winding,
            c_h: self.c_housing,
        };
        let (tw, th) = net.integrate(
            (self.t_winding_c, self.t_housing_c),
            dt,
            t_ambient_c,
            |_tw| (p_cu + s * p_nl, (1.0 - s) * p_nl),
        );
        self.t_winding_c = tw;
        self.t_housing_c = th;
    }

    /// Заменить сопротивление «корпус → среда» (обдув, вода, крепление к раме), К/Вт.
    pub fn set_housing_to_ambient_k_per_w(&mut self, r: f64) {
        self.r_housing_ambient = r;
    }
    pub fn housing_to_ambient_k_per_w(&self) -> f64 {
        self.r_housing_ambient
    }
    pub fn winding_temp_c(&self) -> f64 {
        self.t_winding_c
    }
    pub fn housing_temp_c(&self) -> f64 {
        self.t_housing_c
    }
    /// Обмотка выше допустимой температуры.
    pub fn is_overheated(&self) -> bool {
        self.t_winding_c > self.params.thermal.max_winding_temp_c
    }
    /// Момент на суставе выше повторяющегося пика редуктора.
    pub fn gear_overloaded(&self) -> bool {
        self.joint_torque().abs() > self.params.gear.repeated_peak_output_torque_nm
    }
}

/// Двухузловая тепловая сеть «обмотка → корпус → среда».
#[derive(Clone, Copy, Debug)]
pub struct ThermalNet {
    pub r_wh: f64,
    pub r_ha: f64,
    pub c_w: f64,
    pub c_h: f64,
}

impl ThermalNet {
    /// RK4 с подшагом ≤ min(R·C)/20; `power(T_w)` → (P_обмотки, P_корпуса).
    pub fn integrate(
        &self,
        state: (f64, f64),
        dt: f64,
        t_amb: f64,
        power: impl Fn(f64) -> (f64, f64),
    ) -> (f64, f64) {
        let tau_min = (self.r_wh * self.c_w)
            .min(self.r_ha * self.c_h)
            .min(self.r_wh * self.c_h);
        let n = ((dt / (tau_min / 20.0)).ceil() as usize).max(1);
        let h = dt / n as f64;
        let f = |tw: f64, th: f64| {
            let (pw, ph) = power(tw);
            let q_wh = (tw - th) / self.r_wh;
            let q_ha = (th - t_amb) / self.r_ha;
            ((pw - q_wh) / self.c_w, (ph + q_wh - q_ha) / self.c_h)
        };
        let (mut tw, mut th) = state;
        for _ in 0..n {
            let k1 = f(tw, th);
            let k2 = f(tw + 0.5 * h * k1.0, th + 0.5 * h * k1.1);
            let k3 = f(tw + 0.5 * h * k2.0, th + 0.5 * h * k2.1);
            let k4 = f(tw + h * k3.0, th + h * k3.1);
            tw += h / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
            th += h / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
        }
        (tw, th)
    }

    /// Точное решение при постоянных мощностях: матричная экспонента
    /// (линейная система ẋ = A·x + b, через расширенную матрицу 3×3).
    pub fn exact_constant_power(
        &self,
        state: (f64, f64),
        t: f64,
        t_amb: f64,
        pw: f64,
        ph: f64,
    ) -> (f64, f64) {
        // переменные — превышения над средой
        let a = Matrix2::new(
            -1.0 / (self.r_wh * self.c_w),
            1.0 / (self.r_wh * self.c_w),
            1.0 / (self.r_wh * self.c_h),
            -(1.0 / self.r_wh + 1.0 / self.r_ha) / self.c_h,
        );
        let b = Vector2::new(pw / self.c_w, ph / self.c_h);
        let mut m = Matrix3::zeros();
        m.fixed_view_mut::<2, 2>(0, 0).copy_from(&(a * t));
        m.fixed_view_mut::<2, 1>(0, 2).copy_from(&(b * t));
        let e = m.exp();
        let x = e * Vector3::new(state.0 - t_amb, state.1 - t_amb, 1.0);
        (x[0] + t_amb, x[1] + t_amb)
    }
}

// ---------------------------------------------------------------------------------------
// Связанный переходный процесс (для сверок)
// ---------------------------------------------------------------------------------------

/// Результат переходного процесса «напряжение на зажимах → разгон инерционной нагрузки».
#[derive(Clone, Debug)]
pub struct TransientResult {
    pub time_s: Vec<f64>,
    pub current_a: Vec<f64>,
    pub joint_speed_rad_s: Vec<f64>,
    pub ledger: EnergyLedger,
    /// Кинетическая энергия ротора и нагрузки в конце, Дж.
    pub kinetic_end_j: f64,
    pub magnetic_end_j: f64,
}

impl TransientResult {
    /// Невязка энергобаланса, Дж.
    pub fn energy_residual_j(&self) -> f64 {
        self.ledger.residual_j(self.magnetic_end_j) - self.kinetic_end_j
    }
}

/// Разгон: двигатель + редуктор + инерция нагрузки `load_inertia_joint` на суставе
/// (без внешнего момента), напряжение `voltage(t)`, холодная обмотка. Интегрирование
/// (i, ω_м) и всех энергий — RK4 с шагом dt. Здесь, в отличие от ко-симуляции,
/// скорость двигателя — переменная состояния.
pub fn simulate_transient(
    p: &MotorParams,
    load_inertia_joint: f64,
    voltage: impl Fn(f64) -> f64,
    t_end: f64,
    dt: f64,
) -> TransientResult {
    let k = p.torque_constant_nm_per_a;
    let r = p.resistance_ohm;
    let l = p.inductance_h;
    let n = p.gear.ratio;
    let jm = p.rotor_inertia_kg_m2 + p.gear.input_inertia_kg_m2;
    // y = [i, ω_м, E_in, E_cu, E_тр, E_ред]
    let rhs = |t: f64, y: &[f64; 6]| -> [f64; 6] {
        let (i, w) = (y[0], y[1]);
        let v = voltage(t);
        let tau_em = k * i;
        let tau_f = p.friction_torque_motor(w);
        let net = tau_em - tau_f;
        // режим: мощность через редуктор от двигателя (ведёт) или к двигателю (тормозит)
        let driving = net * w >= 0.0;
        let (j_eff, eta) = if driving {
            (
                jm + load_inertia_joint / (n * n * p.gear.efficiency_forward),
                p.gear.efficiency_forward,
            )
        } else {
            (
                jm + load_inertia_joint * p.gear.efficiency_backward / (n * n),
                p.gear.efficiency_backward,
            )
        };
        let dw = net / j_eff;
        let tau_in = net - jm * dw; // момент на входе редуктора
        let p_gear = if driving {
            (1.0 - eta) * tau_in * w
        } else {
            (1.0 / eta - 1.0) * (-tau_in * w)
        };
        [
            (v - r * i - k * w) / l,
            dw,
            v * i,
            r * i * i,
            tau_f * w,
            p_gear,
        ]
    };
    let steps = (t_end / dt).round() as usize;
    let mut y = [0.0; 6];
    let mut res = TransientResult {
        time_s: Vec::with_capacity(steps / 10 + 2),
        current_a: Vec::new(),
        joint_speed_rad_s: Vec::new(),
        ledger: EnergyLedger::default(),
        kinetic_end_j: 0.0,
        magnetic_end_j: 0.0,
    };
    for s in 0..=steps {
        let t = s as f64 * dt;
        if s % 10 == 0 {
            res.time_s.push(t);
            res.current_a.push(y[0]);
            res.joint_speed_rad_s.push(y[1] / n);
        }
        if s == steps {
            break;
        }
        let add = |y: &[f64; 6], k: &[f64; 6], h: f64| {
            let mut o = *y;
            for j in 0..6 {
                o[j] += h * k[j];
            }
            o
        };
        let k1 = rhs(t, &y);
        let k2 = rhs(t + 0.5 * dt, &add(&y, &k1, 0.5 * dt));
        let k3 = rhs(t + 0.5 * dt, &add(&y, &k2, 0.5 * dt));
        let k4 = rhs(t + dt, &add(&y, &k3, dt));
        for j in 0..6 {
            y[j] += dt / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
        }
    }
    let (i, w) = (y[0], y[1]);
    res.ledger = EnergyLedger {
        electrical_in_j: y[2],
        copper_j: y[3],
        friction_j: y[4],
        gear_j: y[5],
        mechanical_out_j: 0.0,
        magnetic_start_j: 0.0,
    };
    res.magnetic_end_j = 0.5 * l * i * i;
    res.kinetic_end_j = 0.5 * jm * w * w + 0.5 * load_inertia_joint * (w / n).powi(2);
    res
}

/// Точное решение линейной модели (без Кулонова трения, η = 1, скачок V при t = 0):
/// x = [i, ω_м], ẋ = A·x + b·V — матричная экспонента (Паде, nalgebra). Возвращает ω_м(t).
pub fn linear_step_exact_speed(p: &MotorParams, load_inertia_joint: f64, v: f64, t: f64) -> f64 {
    let k = p.torque_constant_nm_per_a;
    let (r, l) = (p.resistance_ohm, p.inductance_h);
    let n = p.gear.ratio;
    let j = p.rotor_inertia_kg_m2 + p.gear.input_inertia_kg_m2 + load_inertia_joint / (n * n);
    let b = p.viscous_friction_nm_s_per_rad;
    let m = Matrix3::new(
        -r / l * t,
        -k / l * t,
        v / l * t,
        k / j * t,
        -b / j * t,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    (m.exp() * Vector3::new(0.0, 0.0, 1.0))[1]
}

// ---------------------------------------------------------------------------------------
// Сверки
// ---------------------------------------------------------------------------------------

fn ds(x: Option<f64>) -> f64 {
    x.expect("значение даташита задано в пресете")
}

/// Все сверки модуля привода.
pub fn checks() -> Vec<Check> {
    let mut v = Vec::new();
    let mx = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let d = &mx.datasheet;
    let vn = mx.nominal_voltage_v;
    let k = mx.torque_constant_nm_per_a;
    let src = "maxon 500267 (страница изделия)";

    // --- даташит maxon: величины из базовых параметров против напечатанных ---
    v.push(Check {
        id: "em_maxon_kt_kv",
        title: "maxon EC 90 flat: K_t против K_v (K_t·K_v = 60/(2π))".into(),
        simulated: 60.0 / (2.0 * PI * ds(d.speed_constant_rpm_per_v)),
        reference: k,
        unit: "Н·м/А",
        method: "K_t = 60/(2π·K_v), K_v = 41.3 об/мин/В".into(),
        reference_method: format!("{src}: K_t = 231 мН·м/А"),
        tolerance: Tolerance::Relative(0.05),
        note: "В СИ K_t = K_e — следствие сохранения энергии.".into(),
    });
    v.push(Check {
        id: "em_maxon_stall_current",
        title: "maxon EC 90 flat: пусковой ток V/R против даташита".into(),
        simulated: mx.stall_current_a(vn),
        reference: ds(d.stall_current_a),
        unit: "А",
        method: "48 В / 0.844 Ом".into(),
        reference_method: format!("{src}: 56.9 А"),
        tolerance: Tolerance::Relative(0.05),
        note: String::new(),
    });
    v.push(Check {
        id: "em_maxon_stall_torque",
        title: "maxon EC 90 flat: пусковой момент K_t·V/R против каталога 2022".into(),
        simulated: mx.stall_torque_motor_nm(vn),
        reference: ds(d.stall_torque_nm),
        unit: "Н·м",
        method: "линейная модель K_t·V/R".into(),
        reference_method: "каталог maxon EC flat 2022: 13 100 мН·м".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "Текущая страница maxon даёт 9 410 мН·м (−28 %) — вероятно, учтено насыщение; \
               линейная модель этого не описывает, см. datasheet_discrepancies()."
            .into(),
    });
    v.push(Check {
        id: "em_maxon_no_load_speed",
        title: "maxon EC 90 flat: скорость холостого хода модели против даташита".into(),
        simulated: mx.no_load_speed_motor_rad_s(vn) / RPM,
        reference: ds(d.no_load_speed_rpm),
        unit: "об/мин",
        method: "ω₀ = (V − R·T_c/k)/(k + R·b/k), трение по I₀ = 0.278 А".into(),
        reference_method: format!("{src}: 1960 об/мин"),
        tolerance: Tolerance::Relative(0.05),
        note: "Остаток ~0.7 %: округление K_v/K_t и допуск производителя.".into(),
    });
    v.push(Check {
        id: "em_maxon_speed_torque_gradient",
        title: "maxon EC 90 flat: наклон характеристики R/K_t² против даташита".into(),
        simulated: mx.resistance_ohm / (k * k) / RPM / 1000.0,
        reference: ds(d.speed_torque_gradient_rpm_per_mnm),
        unit: "об/мин/мН·м",
        method: "Δn/ΔM = R/k² (из V = Ri + kω)".into(),
        reference_method: format!("{src}: 0.151 об/мин/мН·м"),
        tolerance: Tolerance::Relative(0.05),
        note: String::new(),
    });
    v.push(Check {
        id: "em_maxon_mech_time_constant",
        title: "maxon EC 90 flat: механическая постоянная R·J/K_t² против даташита".into(),
        simulated: mx.resistance_ohm * mx.rotor_inertia_kg_m2 / (k * k),
        reference: ds(d.mech_time_constant_s),
        unit: "с",
        method: "τ_м = R·J_ротора/k²".into(),
        reference_method: format!("{src}: 7.99 мс"),
        tolerance: Tolerance::Relative(0.05),
        note: String::new(),
    });
    {
        let ep = mx.max_efficiency_point(vn);
        v.push(Check {
            id: "em_maxon_max_efficiency",
            title: "maxon EC 90 flat: максимальный КПД модели (поиск по характеристике) против даташита".into(),
            simulated: ep.efficiency,
            reference: ds(d.max_efficiency),
            unit: "—",
            method: format!(
                "золотое сечение по ω: η_max при {:.0} об/мин, {:.3} Н·м",
                ep.speed_rad_s / RPM,
                ep.torque_nm
            ),
            reference_method: format!("{src}: 87 % (округлено до целого)"),
            tolerance: Tolerance::Absolute(0.01),
            note: "Допуск 1 п.п.: округление в даташите ±0.5 + наше разделение трения на Кулоново/вязкое.".into(),
        });
        // та же величина двумя способами: численный поиск против формулы maxon
        let mut coul = mx.clone();
        coul.coulomb_friction_nm = k * ds(d.no_load_current_a);
        coul.viscous_friction_nm_s_per_rad = 0.0;
        let ia = coul.stall_current_a(vn);
        let i0 = ds(d.no_load_current_a);
        v.push(Check {
            id: "em_max_efficiency_formula",
            title: "η_max: численный поиск по характеристике против формулы (1 − √(I₀/I_A))²"
                .into(),
            simulated: coul.max_efficiency_point(vn).efficiency,
            reference: (1.0 - (i0 / ia).sqrt()).powi(2),
            unit: "—",
            method: "золотое сечение, трение только Кулоново T = k·I₀".into(),
            reference_method:
                "аналитический максимум η(I) = (1 − I₀/I)(1 − I/I_A) (maxon, «Formulae Handbook»)"
                    .into(),
            tolerance: Tolerance::Relative(1e-9),
            note: String::new(),
        });
    }
    // длительный ток: модель с потерями холостого хода против даташита (полоса)
    {
        let wn = ds(d.nominal_speed_rpm) * RPM;
        let r_ha = mx.thermal.r_housing_ambient_k_per_w;
        let mid = mx.continuous_current_a(25.0, wn, r_ha);
        let mut all_w = mx.clone();
        all_w.thermal.no_load_loss_to_winding = 1.0;
        let mut all_h = mx.clone();
        all_h.thermal.no_load_loss_to_winding = 0.0;
        let lo = all_w.continuous_current_a(25.0, wn, r_ha);
        let hi = all_h.continuous_current_a(25.0, wn, r_ha);
        let cu_only = mx.continuous_current_a(25.0, 0.0, r_ha);
        v.push(Check {
            id: "em_maxon_nominal_current_thermal",
            title: "maxon EC 90 flat: длительный ток из тепловой модели (T_обм = 125 °C при 25 °C) против даташита".into(),
            simulated: mid,
            reference: ds(d.nominal_current_a),
            unit: "А",
            method: format!(
                "стационар 2-узловой сети, R(125 °C), потери х.х. при 1670 об/мин делятся пополам; \
                 все в обмотку → {lo:.2} А, все в корпус → {hi:.2} А"
            ),
            reference_method: format!("{src}: 4.06 А"),
            tolerance: Tolerance::Absolute(0.5 * (hi - lo)),
            note: format!(
                "ОБЪЯСНЁННОЕ: только медь (без трения/железа) дало бы {cu_only:.2} А (+{:.0} %); \
                 даташит лежит внутри полосы [{lo:.2}; {hi:.2}] А, заданной тем, куда уходят потери х.х. \
                 Допуск — полуширина этой полосы.",
                (cu_only / ds(d.nominal_current_a) - 1.0) * 100.0
            ),
        });
    }

    // --- CubeMars AK80-64 ---
    {
        let ak = MotorParams::cubemars_ak80_64();
        let da = &ak.datasheet;
        let ka = ak.torque_constant_nm_per_a;
        let src = "CubeMars AK80-64 (страница изделия)";
        v.push(Check {
            id: "em_ak80_kt_ke",
            title: "AK80-64: K_t против K_e = 13.7 В/1000 об/мин".into(),
            simulated: ds(da.back_emf_v_per_krpm) / (1000.0 * RPM),
            reference: ka,
            unit: "Н·м/А",
            method: "K_e в СИ: 13.7 В/(1000 об/мин)".into(),
            reference_method: format!("{src}: K_t = 0.136 Н·м/А"),
            tolerance: Tolerance::Relative(0.05),
            note: "Расхождение 3.8 % < 5 %; модель использует K_t для обоих (иначе не сохраняется энергия).".into(),
        });
        v.push(Check {
            id: "em_ak80_mech_time_constant",
            title: "AK80-64: τ_мех = R·J/K_t² против даташита".into(),
            simulated: ak.resistance_ohm * ak.rotor_inertia_kg_m2 / (ka * ka),
            reference: ds(da.mech_time_constant_s),
            unit: "с",
            method: "R = 0.22 Ом (межфазн.), J = 564.5 г·см², K_t = 0.136".into(),
            reference_method: format!("{src}: 0.67 мс"),
            tolerance: Tolerance::Relative(0.05),
            note: "Подтверждает, что K_t и J заданы на стороне двигателя, а R — межфазное.".into(),
        });
        v.push(Check {
            id: "em_ak80_elec_time_constant",
            title: "AK80-64: τ_эл = L/R против даташита".into(),
            simulated: ak.inductance_h / ak.resistance_ohm,
            reference: ds(da.elec_time_constant_s),
            unit: "с",
            method: "133.5 мкГн / 0.22 Ом".into(),
            reference_method: format!("{src}: 0.61 мс"),
            tolerance: Tolerance::Relative(0.05),
            note: String::new(),
        });
        v.push(Check {
            id: "em_ak80_km",
            title: "AK80-64: K_m = K_t/√R против даташита".into(),
            simulated: ka / ak.resistance_ohm.sqrt(),
            reference: ds(da.motor_constant_nm_per_sqrt_w),
            unit: "Н·м/√Вт",
            method: "0.136/√0.22".into(),
            reference_method: format!("{src}: 0.29 Н·м/√Вт"),
            tolerance: Tolerance::Relative(0.05),
            note: String::new(),
        });
        let tau_rated = {
            let i = ds(da.rated_current_a);
            ak.gear_output_torque(ka * i - ak.gear.input_coulomb_nm, 1.0)
        };
        v.push(Check {
            id: "em_ak80_rated_torque",
            title: "AK80-64: момент на выходе при номинальном токе 7 А против номинала 48 Н·м"
                .into(),
            simulated: tau_rated,
            reference: ds(da.rated_joint_torque_nm),
            unit: "Н·м",
            method: "N·η·(K_t·I − T_c), η = 0.85 «оценка», T_c — из «Back drive 4.7 Н·м»".into(),
            reference_method: format!("{src}: rated torque 48 Н·м"),
            tolerance: Tolerance::Relative(0.05),
            note: "КПД — оценка для двухступенчатого планетарного (не подбиралась под 48 Н·м)."
                .into(),
        });
    }

    // --- энергия и переходные процессы ---
    {
        // разгон + реверс напряжения (рекуперация через обратный КПД)
        let jl = 0.3;
        let res = simulate_transient(&mx, jl, |t| if t < 0.06 { vn } else { -vn }, 0.08, 2e-6);
        let sinks = res.ledger.copper_j
            + res.ledger.friction_j
            + res.ledger.gear_j
            + res.kinetic_end_j
            + res.magnetic_end_j;
        v.push(Check {
            id: "em_motor_energy_balance_transient",
            title: "Привод maxon+HD: энергобаланс разгона и реверса (вход = медь + трение + редуктор + ΔW_кин + ΔW_маг)".into(),
            simulated: sinks,
            reference: res.ledger.electrical_in_j,
            unit: "Дж",
            method: format!(
                "RK4 (i, ω, энергии), dt = 2 мкс; медь {:.2}, трение {:.3}, редуктор {:.3}, W_кин {:.3}, W_маг {:.4} Дж",
                res.ledger.copper_j, res.ledger.friction_j, res.ledger.gear_j, res.kinetic_end_j, res.magnetic_end_j
            ),
            reference_method: "∫V·i dt (энергия из источника)".into(),
            tolerance: Tolerance::Relative(1e-7),
            note: "Нагрузка 0.3 кг·м² на суставе (≈ предплечье с кистью), 48 В 60 мс, затем −48 В 20 мс: \
                   торможение через обратный КПД редуктора."
                .into(),
        });
        // установившаяся скорость разгона против статической формулы
        let res2 = simulate_transient(&mx, jl, |_| vn, 0.25, 1e-5);
        let w_end = *res2.joint_speed_rad_s.last().expect("есть точки");
        v.push(Check {
            id: "em_motor_transient_vs_static",
            title:
                "Привод maxon+HD: скорость сустава в конце разгона против формулы холостого хода"
                    .into(),
            simulated: w_end,
            reference: mx.no_load_speed_joint_rad_s(vn),
            unit: "рад/с",
            method: "RK4 переходного процесса, 0.25 с ≈ 25 τ_мех".into(),
            reference_method: "ω₀ = (V − R·T_c/k)/(k + R·b/k)/N (установившийся режим)".into(),
            tolerance: Tolerance::Relative(1e-6),
            note: String::new(),
        });
        // линейная модель: RK4 против матричной экспоненты
        let mut lin = mx.clone();
        lin.coulomb_friction_nm = 0.0;
        lin.gear.input_coulomb_nm = 0.0;
        lin.gear.efficiency_forward = 1.0;
        lin.gear.efficiency_backward = 1.0;
        let t1 = 0.01;
        let rl = simulate_transient(&lin, jl, |_| vn, t1, 1e-6);
        v.push(Check {
            id: "em_motor_step_vs_matrix_exp",
            title:
                "Линейная модель двигателя: скорость через 10 мс, RK4 против матричной экспоненты"
                    .into(),
            simulated: rl.joint_speed_rad_s.last().copied().unwrap_or(f64::NAN) * lin.gear.ratio,
            reference: linear_step_exact_speed(&lin, jl, vn, t1),
            unit: "рад/с",
            method: "RK4, dt = 1 мкс".into(),
            reference_method: "exp(M·t) расширенной системы (Паде, nalgebra)".into(),
            tolerance: Tolerance::Relative(1e-8),
            note: "Без Кулонова трения и потерь редуктора (иначе система нелинейна).".into(),
        });
        // ко-симуляция: точная экспонента тока + бухгалтерия энергии
        let mut m = Motor::new(mx.clone());
        let dt = 1e-3;
        for s in 0..400 {
            let t = s as f64 * dt;
            let w = 2.0 * (2.0 * PI * 1.5 * t).sin(); // качание сустава ±2 рад/с
            let vt = if s < 300 {
                30.0 * (2.0 * PI * t).cos()
            } else {
                -10.0
            };
            m.step_electrical(vt, w, dt);
        }
        let led = *m.energy();
        v.push(Check {
            id: "em_cosim_energy_bookkeeping",
            title: "Ко-симуляция (шаг 1 мс, заданная скорость): вход = медь + трение + редуктор + работа + ΔW_маг".into(),
            simulated: led.electrical_in_j - led.residual_j(m.magnetic_energy_j()),
            reference: led.electrical_in_j,
            unit: "Дж",
            method: "точные интегралы ∫i, ∫i² для экспоненты тока на шаге".into(),
            reference_method: "∫V·i dt".into(),
            tolerance: Tolerance::Relative(1e-10),
            note: "Проверяет согласованность бухгалтерии (в т.ч. смены направления мощности через редуктор).".into(),
        });
    }

    // --- тепловая модель ---
    {
        let th = &mx.thermal;
        let net = ThermalNet {
            r_wh: th.r_winding_housing_k_per_w,
            r_ha: th.r_housing_ambient_k_per_w,
            c_w: th.tau_winding_s / th.r_winding_housing_k_per_w,
            c_h: th.tau_housing_s / th.r_housing_ambient_k_per_w,
        };
        let (pw, ph) = (20.0, 3.0);
        let t = 300.0;
        let num = net.integrate((25.0, 25.0), t, 25.0, |_| (pw, ph));
        let ex = net.exact_constant_power((25.0, 25.0), t, 25.0, pw, ph);
        v.push(Check {
            id: "em_thermal_step_vs_exact",
            title: "Тепловая сеть maxon: T обмотки через 300 с при 20 + 3 Вт, RK4 против матричной экспоненты".into(),
            simulated: num.0,
            reference: ex.0,
            unit: "°C",
            method: "RK4, подшаг ≤ min(RC)/20".into(),
            reference_method: "точное решение линейной системы exp(A·t)".into(),
            tolerance: Tolerance::Absolute(1e-6),
            note: format!("Корпус: {:.6} / {:.6} °C.", num.1, ex.1),
        });
        // стационар с R(T): аналитика
        let i = 3.0;
        let mut mm = Motor::new(mx.clone());
        mm.params.thermal.no_load_loss_to_winding = 1.0;
        let r_tot = th.r_winding_housing_k_per_w + th.r_housing_ambient_k_per_w;
        let kk = i * i * mx.resistance_ohm * r_tot;
        let t_ref = mx.resistance_ref_c;
        let t_exact = (25.0 + kk * (1.0 - ALPHA_CU * t_ref)) / (1.0 - kk * ALPHA_CU);
        for _ in 0..2000 {
            mm.current_a = i;
            mm.step_thermal(10.0, 25.0);
        }
        v.push(Check {
            id: "em_thermal_steady_r_of_t",
            title: "Нагрев при 3 А (стоянка): T обмотки в стационаре против аналитики с R(T)"
                .into(),
            simulated: mm.winding_temp_c(),
            reference: t_exact,
            unit: "°C",
            method: "интегрирование тепловой модели 20 000 с (≈60 медленных τ), шаг 10 с".into(),
            reference_method: "T = (T_a + k(1 − αT₀))/(1 − kα), k = I²R₀(R₁+R₂)".into(),
            tolerance: Tolerance::Absolute(1e-3),
            note: String::new(),
        });
        // снижение тока по температуре среды: формула maxon против моделирования
        let i_n = mx.continuous_current_a(25.0, 0.0, th.r_housing_ambient_k_per_w);
        let ta = 40.0;
        let i40 = derated_current_maxon(i_n, th.max_winding_temp_c, ta);
        let mut m40 = Motor::new(mx.clone()).with_temperatures(ta, ta);
        for _ in 0..2000 {
            m40.current_a = i40;
            m40.step_thermal(10.0, ta);
        }
        v.push(Check {
            id: "em_thermal_derating_maxon",
            title: "Снижение длительного тока при 40 °C: I_N·√((T_max−T_a)/(T_max−25)) → T обмотки = T_max".into(),
            simulated: m40.winding_temp_c(),
            reference: th.max_winding_temp_c,
            unit: "°C",
            method: format!("моделирование 20 000 с при I = {i40:.3} А (из I_N = {i_n:.3} А по формуле maxon)"),
            reference_method: "предел обмотки maxon: 125 °C".into(),
            tolerance: Tolerance::Absolute(0.01),
            note: "Только медь, стоянка; формула точна при неизменных R_th (R(T) одинаково на пределе).".into(),
        });
    }
    v
}

/// Противоречия внутри данных производителей, которые модель (линейный K_t, трение по I₀)
/// не объясняет. Возвращаются как Check: «симуляция» — модель из базовых параметров,
/// «эталон» — напечатанное значение. Многие из них НЕ проходят 5 % — это находка о данных.
pub fn datasheet_discrepancies() -> Vec<Check> {
    let mut v = Vec::new();
    let mx = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let k = mx.torque_constant_nm_per_a;
    let vn = mx.nominal_voltage_v;
    v.push(Check {
        id: "ds_maxon_stall_torque_2026",
        title: "maxon EC 90 flat 500267: K_t·V/R против пускового момента на текущей странице (9410 мН·м)".into(),
        simulated: mx.stall_torque_motor_nm(vn),
        reference: 9.41,
        unit: "Н·м",
        method: "линейная модель K_t·V/R".into(),
        reference_method: "maxongroup.com, 500267, stall torque 9410 mNm (каталог 2022: 13100)".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "Гипотеза: новая цифра учитывает насыщение при 57 А (14×I_N); maxon этого явно не пишет. \
               При токах до ~3·I_N, нужных экзоскелету, линейная модель остаётся рабочей — но это не проверено."
            .into(),
    });
    let i_n = mx.datasheet.nominal_current_a.unwrap_or(f64::NAN);
    let i0 = mx.datasheet.no_load_current_a.unwrap_or(f64::NAN);
    v.push(Check {
        id: "ds_maxon_nominal_torque",
        title: "maxon EC 90 flat 500267: K_t·(I_N − I₀) против номинального момента 964 мН·м".into(),
        simulated: k * (i_n - i0),
        reference: mx.datasheet.nominal_torque_nm.unwrap_or(f64::NAN),
        unit: "Н·м",
        method: "0.231·(4.06 − 0.278)".into(),
        reference_method: "maxon 500267: nominal torque 964 mNm".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "Даже K_t·I_N = 0.938 Н·м < 0.964: напечатанный номинальный момент не следует из K_t и I_N. \
               Возможная причина — разные условия определения (монтаж/теплоотвод), не подтверждено."
            .into(),
    });
    {
        // скорость в номинальной точке с горячей обмоткой
        let tau = mx.datasheet.nominal_torque_nm.unwrap_or(f64::NAN);
        let r = mx.resistance_at(mx.thermal.max_winding_temp_c);
        let mut w = mx.no_load_speed_motor_rad_s(vn);
        for _ in 0..50 {
            let i = (tau + mx.coulomb_friction_nm + mx.viscous_friction_nm_s_per_rad * w) / k;
            w = (vn - r * i) / k;
        }
        v.push(Check {
            id: "ds_maxon_nominal_speed",
            title: "maxon EC 90 flat 500267: скорость при номинальном моменте (обмотка 125 °C) против 1670 об/мин".into(),
            simulated: w / RPM,
            reference: mx.datasheet.nominal_speed_rpm.unwrap_or(f64::NAN),
            unit: "об/мин",
            method: "V = R(125 °C)·i + kω, k·i = M_N + трение".into(),
            reference_method: "maxon 500267: nominal speed 1670 rpm".into(),
            tolerance: Tolerance::Relative(0.05),
            note: "Холодная обмотка дала бы ещё больше. Объяснения в открытых данных maxon не нашли.".into(),
        });
    }
    let ak = MotorParams::cubemars_ak80_64();
    let ka = ak.torque_constant_nm_per_a;
    v.push(Check {
        id: "ds_ak80_kt_kv",
        title: "AK80-64: 60/(2π·K_v) при K_v = 80 об/мин/В против K_t = 0.136".into(),
        simulated: 60.0 / (2.0 * PI * 80.0),
        reference: ka,
        unit: "Н·м/А",
        method: "K_t из K_v".into(),
        reference_method: "CubeMars: K_t = 0.136 Н·м/А".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "−12 %. K_v у производителей BLDC часто задают в другой системе (амплитуда/действующее, \
               фазное/линейное); какую выбрал CubeMars — не указано. K_t согласуется с K_e (3.8 %) — берём K_t."
            .into(),
    });
    v.push(Check {
        id: "ds_ak80_no_load_speed",
        title: "AK80-64: скорость холостого хода сустава при 48 В — модель против 75 об/мин".into(),
        simulated: ak.no_load_speed_joint_rad_s(48.0) / RPM,
        reference: 75.0,
        unit: "об/мин",
        method: "(V − R·T_c/k)/k/N с K_t = K_e = 0.136".into(),
        reference_method: "CubeMars: no-load speed 75 rpm (в другой таблице 37/75 при 24/48 В)".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "Даже через K_v = 80: 48·80/64 = 60 об/мин < 75. Возможная причина — ослабление поля \
               во встроенном драйвере; не подтверждено. Для расчёта скорости сустава берите нижнюю оценку."
            .into(),
    });
    {
        let i_pk = 19.0;
        v.push(Check {
            id: "ds_ak80_peak_torque",
            title: "AK80-64: момент при пиковом токе 19 А (линейный K_t) против пика 120 Н·м".into(),
            simulated: ak.gear_output_torque(ka * i_pk - ak.gear.input_coulomb_nm, 1.0),
            reference: 120.0,
            unit: "Н·м",
            method: "N·η·(K_t·I − T_c)".into(),
            reference_method: "CubeMars: peak torque 120 Н·м".into(),
            tolerance: Tolerance::Relative(0.05),
            note: "Модель выше на ~14 %: насыщение K_t при 2.7·I_ном и/или пик — это предел прочности \
                   редуктора, а не электромагнитный. Не подтверждено."
                .into(),
        });
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backdrive_matches_calibration() {
        let ak = MotorParams::cubemars_ak80_64();
        let mut m = Motor::new(ak.clone());
        // обесточенный привод (V = 0, но противо-ЭДС даёт ток): медленный обратный ход
        m.step_electrical(0.0, 0.001, 1.0);
        // без тока — только трение: момент на суставе = −N·T_c/η_обр
        let tau_f = ak.gear_output_torque(-ak.friction_torque_motor(1.0), 1.0);
        assert!((tau_f.abs() - 4.7).abs() < 1e-9, "{tau_f}");
    }

    #[test]
    fn current_step_is_exact_and_stable() {
        let mut m = Motor::new(MotorParams::cubemars_ak80_64());
        // шаг 5 мс ≫ L/R = 0.6 мс: явный Эйлер бы разошёлся, экспонента — нет
        for _ in 0..20 {
            m.step_electrical(10.0, 0.0, 5e-3);
        }
        let r = m.winding_resistance_ohm();
        assert!((m.current() - 10.0 / r).abs() < 1e-9);
        // токовый режим выходит на заданный ток за один шаг
        let mut m2 = Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50());
        let v = m2.step_current(5.0, 0.1, 1e-3, 48.0);
        assert!(v.abs() <= 48.0);
        assert!((m2.current() - 5.0).abs() < 1e-9, "{}", m2.current());
    }

    #[test]
    fn housing_resistance_is_replaceable() {
        let mut a = Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50());
        let mut b = a.clone();
        b.set_housing_to_ambient_k_per_w(0.5); // обдув
        for _ in 0..600 {
            for m in [&mut a, &mut b] {
                m.step_current(4.0, 0.0, 1e-3, 48.0);
                m.step_thermal(1.0, 25.0);
            }
        }
        assert!(b.winding_temp_c() < a.winding_temp_c());
    }

    #[test]
    fn exo_requirements_are_met_by_presets() {
        for p in [
            MotorParams::maxon_ec90flat_260w_hd_csg25_50(),
            MotorParams::cubemars_ak80_64(),
        ] {
            let w = p
                .no_load_speed_joint_rad_s(p.nominal_voltage_v)
                .to_degrees();
            assert!(w > 200.0, "{}: {w} °/с", p.name);
            let i_cont = p.continuous_current_a(25.0, 0.0, p.thermal.r_housing_ambient_k_per_w);
            let t_cont = p.gear_output_torque(
                p.torque_constant_nm_per_a * i_cont - p.gear.input_coulomb_nm,
                1.0,
            );
            assert!(t_cont > 30.0, "{}: {t_cont} Н·м", p.name);
        }
    }
}
