//! Эталонные сверки вторым независимым способом и найденные противоречия в паспортах.
//!
//! [`reference_checks`] обязаны проходить: это проверка модели и каталога.
//! [`datasheet_discrepancies`] — находки о данных производителей: часть из них не проходит
//! допуск, и это не ошибка модели. Их не прячем; в заметке сказано, какое число взято.

use std::f64::consts::PI;

use sim_verify::{Check, Tolerance};

use crate::actuator::{Actuator, VoltagePoint};
use crate::assess::DutyCycle;
use crate::power::{Battery, BoardLoad, PowerInput, RailInput, power_budget};
use crate::spec::{ALPHA_CU, KGF_CM, RPM, Spec, T_REF_C};
use crate::thermal::{HeatSegment, ThermalNet, periodic_peak_winding_c, steady_winding_c};
use crate::{actuator_by_id, driver_by_id, regulator_by_id};

#[allow(clippy::too_many_arguments)]
fn check(
    id: &'static str,
    title: impl Into<String>,
    simulated: f64,
    reference: f64,
    unit: &'static str,
    method: impl Into<String>,
    reference_method: impl Into<String>,
    tolerance: Tolerance,
    note: impl Into<String>,
) -> Check {
    Check {
        id,
        title: title.into(),
        simulated,
        reference,
        unit,
        method: method.into(),
        reference_method: reference_method.into(),
        tolerance,
        note: note.into(),
    }
}

fn act(id: &str) -> &'static Actuator {
    actuator_by_id(id).expect("привод есть в каталоге")
}

fn val(s: Spec) -> f64 {
    s.get().unwrap_or(f64::NAN)
}

/// Паспорт Pololu #3073 (50:1 HPCB 6 В) — только для сверки модели с напечатанной P_max.
/// https://www.pololu.com/product/3073/specs
static POLOLU_3073: [VoltagePoint; 1] = [VoltagePoint {
    voltage_v: 6.0,
    stall_torque_nm: 0.74 * KGF_CM,
    no_load_rpm: 650.0,
    stall_current_a: Some(1.5),
}];

/// Все эталонные сверки крейта.
pub fn reference_checks() -> Vec<Check> {
    let mut v = Vec::new();
    datasheet_identities(&mut v);
    curve_checks(&mut v);
    driver_checks(&mut v);
    transient_checks(&mut v);
    thermal_checks(&mut v);
    power_checks(&mut v);
    v
}

fn datasheet_identities(v: &mut Vec<Check>) {
    let mx = act("maxon-ec90-hd-csg25-50");
    let (k, r) = (val(mx.kt_motor_nm_per_a), val(mx.resistance_ohm));
    let src = "maxon 500267";
    v.push(check(
        "act_maxon_kt_kv",
        "maxon EC 90: K_t против K_v (K_t·K_v = 60/2π)",
        60.0 / (2.0 * PI * val(mx.kv_rpm_per_v)),
        k,
        "Н·м/А",
        "60/(2π·41.3)",
        format!("{src}: K_t = 231 мН·м/А"),
        Tolerance::Relative(0.01),
        "",
    ));
    v.push(check(
        "act_maxon_stall_current",
        "maxon EC 90: закон Ома V/R против стопового тока",
        val(mx.voltage_nominal_v) / r,
        val(mx.stall_current_a),
        "А",
        "48 В / 0.844 Ом",
        format!("{src}: 56.9 А"),
        Tolerance::Relative(0.01),
        "",
    ));
    v.push(check(
        "act_maxon_no_load_speed",
        "maxon EC 90: скорость холостого хода (V − R·I₀)/K_t против паспорта",
        (48.0 - r * val(mx.no_load_current_a)) / k / RPM,
        1960.0,
        "об/мин",
        "баланс напряжений на холостом ходу, вал двигателя",
        format!("{src}: 1960 об/мин"),
        Tolerance::Relative(0.02),
        "В расчётах вердикта K_e взят больше — по номинальной точке, см. расхождения.",
    ));
    v.push(check(
        "act_maxon_gradient",
        "maxon EC 90: наклон R/K_t² против паспорта",
        r / (k * k) / RPM / 1000.0,
        0.151,
        "об/мин/мН·м",
        "Δn/ΔM = R/k²",
        format!("{src}: 0.151 об/мин/мН·м"),
        Tolerance::Relative(0.02),
        "",
    ));
    v.push(check(
        "act_maxon_tau_mech",
        "maxon EC 90: механическая постоянная R·J/K_t² против паспорта",
        r * val(mx.rotor_inertia_kg_m2) / (k * k),
        7.99e-3,
        "с",
        "τ_м = R·J/k²",
        format!("{src}: 7.99 мс"),
        Tolerance::Relative(0.02),
        "",
    ));
    v.push(check(
        "act_maxon_reflected_inertia",
        "Отражённая инерция maxon + HD против чисел sim-em",
        mx.reflected_inertia_kg_m2().unwrap_or(f64::NAN),
        (5060e-7 + 0.413e-4) * 50.0 * 50.0,
        "кг·м²",
        "Actuator::reflected_inertia_kg_m2",
        "(J_ротора + J_входа)·N² по числам crates/sim-em/src/motor.rs",
        Tolerance::Relative(1e-12),
        "Сверка чтением: зависимости от sim-em у крейта нет.",
    ));

    let ak = act("cubemars-ak80-64");
    let (ka, ra) = (val(ak.kt_motor_nm_per_a), val(ak.resistance_ohm));
    v.push(check(
        "act_ak80_km",
        "AK80-64: K_m = K_t/√R против паспорта",
        ka / ra.sqrt(),
        0.29,
        "Н·м/√Вт",
        "0.136/√0.22",
        "CubeMars: 0.29 Н·м/√Вт",
        Tolerance::Relative(0.05),
        "",
    ));
    v.push(check(
        "act_ak80_tau_elec",
        "AK80-64: L/R против паспорта",
        val(ak.inductance_h) / ra,
        0.61e-3,
        "с",
        "133.5 мкГн / 0.22 Ом",
        "CubeMars: 0.61 мс",
        Tolerance::Relative(0.05),
        "",
    ));
    v.push(check(
        "act_ak80_tau_mech",
        "AK80-64: R·J/K_t² против паспорта",
        ra * val(ak.rotor_inertia_kg_m2) / (ka * ka),
        0.67e-3,
        "с",
        "R = 0.22 Ом, J = 564.5 г·см²",
        "CubeMars: 0.67 мс",
        Tolerance::Relative(0.05),
        "Подтверждает, что K_t и J заданы на валу двигателя, а R — межфазное.",
    ));
    let ak70 = act("cubemars-ak70-10");
    v.push(check(
        "act_ak70_km",
        "AK70-10: K_m = K_t/√R против паспорта",
        val(ak70.kt_motor_nm_per_a) / val(ak70.resistance_ohm).sqrt(),
        0.24,
        "Н·м/√Вт",
        "0.123/√0.272",
        "CubeMars: 0.24 Н·м/√Вт",
        Tolerance::Relative(0.05),
        "",
    ));
    let ak60 = act("cubemars-ak60-6");
    v.push(check(
        "act_ak60_km",
        "AK60-6: K_m = K_t/√R против паспорта",
        val(ak60.kt_motor_nm_per_a) / val(ak60.resistance_ohm).sqrt(),
        0.17,
        "Н·м/√Вт",
        "0.078/√0.202",
        "CubeMars: 0.17 Н·м/√Вт",
        Tolerance::Relative(0.05),
        "",
    ));

    // Отношение момента к току, напечатанное ROBOTIS отдельной графой.
    let dxl: [(&'static str, &str, f64, f64); 9] = [
        ("act_xl330_kt_37", "dynamixel-xl330-m288", 3.7, 0.378),
        ("act_xl330_kt_50", "dynamixel-xl330-m288", 5.0, 0.354),
        ("act_xl330_kt_60", "dynamixel-xl330-m288", 6.0, 0.345),
        ("act_xl430_kt_90", "dynamixel-xl430-w250", 9.0, 1.000),
        ("act_xl430_kt_111", "dynamixel-xl430-w250", 11.1, 1.077),
        ("act_xl430_kt_120", "dynamixel-xl430-w250", 12.0, 1.071),
        ("act_xm430_kt_111", "dynamixel-xm430-w350", 11.1, 1.810),
        ("act_xm430_kt_120", "dynamixel-xm430-w350", 12.0, 1.783),
        ("act_xm430_kt_148", "dynamixel-xm430-w350", 14.8, 1.778),
    ];
    for (id, part, volts, printed) in dxl {
        let a = act(part);
        let k = a
            .motor_line(volts, T_REF_C)
            .and_then(|l| l.k_t_nm_per_a)
            .unwrap_or(f64::NAN);
        v.push(check(
            id,
            format!("{}: K_t модели при {volts} В против графы паспорта", a.name),
            k,
            printed,
            "Н·м/А",
            "стоповый момент / стоповый ток из таблицы",
            "ROBOTIS e-Manual: отдельно напечатанное отношение",
            Tolerance::Relative(0.005),
            "",
        ));
    }
    let sts = act("feetech-sts3215");
    v.push(check(
        "act_sts3215_kt",
        "STS3215: стоповый момент / стоповый ток против K_t паспорта",
        val(sts.peak_torque_nm) / val(sts.stall_current_a) / KGF_CM,
        8.0,
        "кгс·см/А",
        "19.5 кгс·см / 2.5 А",
        "Feetech: K_t = 8 кгс·см/А",
        Tolerance::Relative(0.05),
        "",
    ));
}

fn curve_checks(v: &mut Vec<Check>) {
    let n20 = act("n20-100-6v-hp");
    let line = n20.motor_line(6.0, T_REF_C);
    v.push(check(
        "act_curve_n20_stall",
        "N20 100:1: прямая при ω = 0 проходит через паспортный стоповый момент",
        line.map_or(f64::NAN, |l| l.available_torque(0.0, 6.0)),
        1.7 * KGF_CM,
        "Н·м",
        "MotorLine::available_torque",
        "Pololu #2214: 1.7 кгс·см",
        Tolerance::Relative(1e-9),
        "",
    ));
    v.push(check(
        "act_curve_n20_free",
        "N20 100:1: прямая при τ = 0 проходит через паспортную скорость",
        line.map_or(f64::NAN, |l| l.available_speed(0.0, 6.0)) / RPM,
        310.0,
        "об/мин",
        "MotorLine::available_speed",
        "Pololu #2214: 310 об/мин",
        Tolerance::Relative(1e-9),
        "",
    ));
    let xm = act("dynamixel-xm430-w350");
    v.push(check(
        "act_curve_xm430_point",
        "XM430: интерполяция по напряжению попадает в паспортную точку 12 В",
        xm.point_at(12.0).map_or(f64::NAN, |p| p.stall_torque_nm),
        4.1,
        "Н·м",
        "Actuator::point_at",
        "ROBOTIS: 4.1 Н·м при 12 В",
        Tolerance::Relative(1e-9),
        "",
    ));

    // Максимум механической мощности: перебор против T_s·ω₀/4 и против напечатанной P_max.
    let mut p3073 = n20.clone();
    p3073.voltage_points = &POLOLU_3073;
    p3073.no_load_current_a = Spec::ds(0.15, "Pololu #3073");
    let l = p3073.motor_line(6.0, T_REF_C);
    let scan = l.map_or(f64::NAN, |l| {
        (0..=20_000)
            .map(|j| {
                let w = l.no_load_rad_s * j as f64 / 20_000.0;
                w * l.available_torque(w, 6.0)
            })
            .fold(0.0, f64::max)
    });
    let closed = 0.74 * KGF_CM * 650.0 * RPM / 4.0;
    v.push(check(
        "act_curve_pmax_closed_form",
        "Прямая мотора: максимум мощности перебором против T_s·ω₀/4",
        scan,
        closed,
        "Вт",
        "перебор 20 001 точки по кривой",
        "аналитический максимум линейной характеристики",
        Tolerance::Relative(1e-6),
        "",
    ));
    v.push(check(
        "act_curve_pmax_pololu",
        "Прямая мотора: максимум мощности против напечатанного Pololu",
        scan,
        1.2,
        "Вт",
        "перебор по кривой из стоповой точки и холостого хода",
        "Pololu #3073: Max power 1.2 Вт",
        Tolerance::Relative(0.05),
        "Независимое число паспорта: в модель не входит.",
    ));

    let nema = act("nema17-17hs19-2004s1");
    v.push(check(
        "act_stepper_hold",
        "NEMA 17: удерживающий момент при ω = 0 против паспорта",
        nema.available_torque_nm(12.0, 0.0, T_REF_C)
            .unwrap_or(f64::NAN),
        0.59,
        "Н·м",
        "верхняя граница момента по импедансу при ω = 0",
        "STEPPERONLINE: 0.59 Н·м",
        Tolerance::Relative(1e-9),
        "",
    ));

    let n = 10_000;
    let a = 7.0;
    let duty = (0..n).fold(DutyCycle::new(12.0, 25.0), |d, j| {
        let t = (j as f64 + 0.5) / n as f64;
        d.segment(a * (2.0 * PI * t).sin(), 0.0, 1e-3)
    });
    v.push(check(
        "act_rms_sine",
        "Среднеквадратичный момент цикла: синус против A/√2",
        duty.rms_torque_nm(),
        a / 2f64.sqrt(),
        "Н·м",
        "DutyCycle::rms_torque_nm, 10 000 участков",
        "аналитика для синуса",
        Tolerance::Relative(1e-9),
        "",
    ));
}

fn driver_checks(v: &mut Vec<Check>) {
    v.push(check(
        "act_drv8833_ron",
        "DRV8833: верхний + нижний ключ против заголовка паспорта",
        200.0 + 160.0,
        360.0,
        "мОм",
        "таблица FET: 200 + 160 мОм при 25 °C",
        "TI: HS + LS 360 мОм",
        Tolerance::Relative(1e-9),
        "",
    ));
    v.push(check(
        "act_bts7960_ron",
        "BTS7960: сопротивление пути против заголовка паспорта",
        7.0 + 9.0,
        16.0,
        "мОм",
        "таблица: 7 + 9 мОм при 25 °C",
        "Infineon: path resistance typ. 16 мОм",
        Tolerance::Relative(1e-9),
        "",
    ));
    v.push(check(
        "act_tb6612_ron",
        "TB6612: закон Ома, насыщение при 1 А против заявленного сопротивления",
        0.5 / 1.0,
        0.5,
        "Ом",
        "Vsat(U+L) = 0.5 В тип. при 1 А",
        "Toshiba: 0.5 Ом (верх + низ)",
        Tolerance::Relative(1e-9),
        "",
    ));
    let l298 = driver_by_id("l298n").expect("драйвер есть в каталоге");
    v.push(check(
        "act_l298_drop_sum",
        "L298: падение модели при 2 А против суммы верхнего и нижнего ключа",
        l298.voltage_drop(2.0).unwrap_or(f64::NAN),
        2.7 + 2.3,
        "В",
        "таблица падения: Total Drop 4.9 В",
        "ST: VCEsat(H) 2.7 В + VCEsat(L) 2.3 В, максимумы",
        Tolerance::Relative(0.05),
        "Сумма максимумов чуть больше напечатанного итога: все три максимума разом не достигаются.",
    ));
    v.push(check(
        "act_l298_drop_interp",
        "L298: падение при 1.5 А, линейная интерполяция",
        l298.voltage_drop(1.5).unwrap_or(f64::NAN),
        (3.2 + 4.9) / 2.0,
        "В",
        "Driver::voltage_drop",
        "середина между точками 1 А и 2 А",
        Tolerance::Relative(1e-9),
        "",
    ));
}

fn transient_checks(v: &mut Vec<Check>) {
    // Электрическая цепь обмотки интегрируется отдельно от статической формулы модели.
    let mx = act("maxon-ec90-hd-csg25-50");
    let Some(line) = mx.motor_line(48.0, 60.0) else {
        return;
    };
    let (tau, w) = (20.0, 1.5);
    let r = line.r_ohm.unwrap_or(f64::NAN);
    let l = val(mx.inductance_h);
    let ke = line.k_e_v_s;
    let u = line.required_voltage(tau, w).unwrap_or(f64::NAN);
    let i_model = line.current_for_torque(tau).unwrap_or(f64::NAN);
    let t_end = 40.0 * l / r;
    let steps = 40_000;
    let h = t_end / steps as f64;
    let f = |i: f64| (u - r * i - ke * w) / l;
    let (mut i, mut e_in, mut e_cu, mut e_emf) = (0.0, 0.0, 0.0, 0.0);
    for _ in 0..steps {
        let (k1, k2_i) = (f(i), i + 0.5 * h * f(i));
        let k2 = f(k2_i);
        let k3_i = i + 0.5 * h * k2;
        let k3 = f(k3_i);
        let k4_i = i + h * k3;
        let k4 = f(k4_i);
        // Интегралы по Симпсону на тех же узлах RK4.
        let simpson =
            |g: &dyn Fn(f64) -> f64| h / 6.0 * (g(i) + 2.0 * g(k2_i) + 2.0 * g(k3_i) + g(k4_i));
        e_in += simpson(&|x| u * x);
        e_cu += simpson(&|x| r * x * x);
        e_emf += simpson(&|x| ke * w * x);
        i += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    v.push(check(
        "act_ohm_transient_current",
        "Ток обмотки: разгон цепи L·di/dt до установившегося против статической модели",
        i,
        i_model,
        "А",
        "RK4 уравнения V = R·i + L·di/dt + K_e·ω, 40 постоянных L/R",
        "MotorLine::current_for_torque",
        Tolerance::Relative(1e-9),
        "maxon + HD, 20 Н·м, 1.5 рад/с, обмотка 60 °C.",
    ));
    v.push(check(
        "act_energy_balance_transient",
        "Энергобаланс обмотки: вход = медь + противо-ЭДС + энергия поля",
        e_cu + e_emf + 0.5 * l * i * i,
        e_in,
        "Дж",
        "∫R·i²dt + ∫K_e·ω·i dt + ½·L·i²",
        "∫V·i dt",
        Tolerance::Relative(1e-6),
        "",
    ));
}

fn thermal_checks(v: &mut Vec<Check>) {
    let mx = act("maxon-ec90-hd-csg25-50");
    let r0 = val(mx.resistance_ohm);
    let (r1, r2, t1, t2) = (
        val(mx.r_th_winding_housing_k_per_w),
        val(mx.r_th_housing_ambient_k_per_w),
        val(mx.tau_winding_s),
        val(mx.tau_housing_s),
    );
    let net = ThermalNet::from_datasheet(r1, r2, t1, t2);
    let seg = |i: f64, dt: f64| HeatSegment {
        duration_s: dt,
        copper_w_at_ref: i * i * r0,
        other_winding_w: 0.0,
        other_housing_w: 0.0,
    };
    let cycle = [seg(6.0, 20.0), seg(1.0, 40.0)];
    let exact = periodic_peak_winding_c(&net, &cycle, 25.0, T_REF_C).unwrap_or(f64::NAN);

    // Второй способ: прямое интегрирование RK4 по множеству циклов.
    let rhs = |tw: f64, th: f64, p0: f64| {
        let p = p0 * (1.0 + ALPHA_CU * (tw - T_REF_C));
        let q = (tw - th) / net.r_wh;
        ((p - q) / net.c_w, (q - (th - 25.0) / net.r_ha) / net.c_h)
    };
    let h = 0.02;
    let (mut tw, mut th) = (25.0f64, 25.0f64);
    let mut peak = 0.0f64;
    for c in 0..120 {
        if c == 119 {
            peak = tw;
        }
        for s in &cycle {
            let n = (s.duration_s / h).round() as usize;
            for _ in 0..n {
                let p = s.copper_w_at_ref;
                let k1 = rhs(tw, th, p);
                let k2 = rhs(tw + 0.5 * h * k1.0, th + 0.5 * h * k1.1, p);
                let k3 = rhs(tw + 0.5 * h * k2.0, th + 0.5 * h * k2.1, p);
                let k4 = rhs(tw + h * k3.0, th + h * k3.1, p);
                tw += h / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
                th += h / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
                if c == 119 {
                    peak = peak.max(tw);
                }
            }
        }
    }
    v.push(check(
        "act_thermal_periodic_vs_rk4",
        "Установившийся тепловой цикл: неподвижная точка против RK4 на 120 циклах",
        exact,
        peak,
        "°C",
        "матричная экспонента по участкам, x = Φx + γ",
        "прямое интегрирование RK4, шаг 20 мс, 2 часа модельного времени",
        Tolerance::Absolute(0.02),
        "maxon: 6 А 20 с, 1 А 40 с, воздух 25 °C, R(T) учтено.",
    ));
    let i = 3.0;
    let k = i * i * r0 * (r1 + r2);
    v.push(check(
        "act_thermal_steady_closed_form",
        "Постоянный ток: периодический расчёт против аналитики с R(T)",
        periodic_peak_winding_c(&net, &[seg(i, 100.0)], 25.0, T_REF_C).unwrap_or(f64::NAN),
        (25.0 + k * (1.0 - ALPHA_CU * T_REF_C)) / (1.0 - k * ALPHA_CU),
        "°C",
        "periodic_peak_winding_c",
        "T = (T_a + k(1 − αT₀))/(1 − kα), k = I²R₀(R₁ + R₂)",
        Tolerance::Absolute(1e-6),
        "",
    ));
    v.push(check(
        "act_thermal_steady_single",
        "Одноступенчатая модель против той же аналитики",
        steady_winding_c(i * i * r0, r1 + r2, 25.0, T_REF_C).unwrap_or(f64::NAN),
        (25.0 + k * (1.0 - ALPHA_CU * T_REF_C)) / (1.0 - k * ALPHA_CU),
        "°C",
        "steady_winding_c",
        "T = (T_a + k(1 − αT₀))/(1 − kα)",
        Tolerance::Absolute(1e-9),
        "",
    ));
    let t_max = val(mx.max_winding_temp_c);
    let i_n = ((t_max - 25.0) / ((r1 + r2) * r0 * (1.0 + ALPHA_CU * (t_max - T_REF_C)))).sqrt();
    let i_40 = i_n * ((t_max - 40.0) / (t_max - 25.0)).sqrt();
    v.push(check(
        "act_thermal_derating",
        "Снижение тока при 40 °C по формуле maxon выводит обмотку ровно на предел",
        periodic_peak_winding_c(&net, &[seg(i_40, 100.0)], 40.0, T_REF_C).unwrap_or(f64::NAN),
        t_max,
        "°C",
        "тепловая модель при I = I_N·√((T_max − T_a)/(T_max − 25))",
        "предел обмотки maxon 125 °C",
        Tolerance::Absolute(1e-6),
        "Формула, по которой в вердикте снижается длительный момент.",
    ));
}

fn power_checks(v: &mut Vec<Check>) {
    let ams = regulator_by_id("ams1117-3.3").expect("стабилизатор есть в каталоге");
    let op = ams.operating(5.0, 3.3, 0.5);
    let iq = val(ams.quiescent_a);
    v.push(check(
        "act_ldo_energy_balance",
        "AMS1117-3.3: потери как вход − выход против (V_вх − V_вых)·I + V_вх·I_q",
        op.map_or(f64::NAN, |o| o.input_power_w - 3.3 * 0.5),
        (5.0 - 3.3) * 0.5 + 5.0 * iq,
        "Вт",
        "Regulator::operating",
        "баланс мощности линейного стабилизатора",
        Tolerance::Relative(1e-9),
        "",
    ));
    let lm = regulator_by_id("lm2596-adj").expect("стабилизатор есть в каталоге");
    let op = lm.operating(12.0, 5.0, 2.0);
    let iq = val(lm.quiescent_a);
    v.push(check(
        "act_buck_energy_balance",
        "LM2596: потери как вход − выход против P_вых·(1/η − 1) + V_вх·I_q",
        op.map_or(f64::NAN, |o| o.loss_w),
        10.0 * (1.0 / 0.80 - 1.0) + 12.0 * iq,
        "Вт",
        "Regulator::operating, η = 80 % при 12 → 5 В",
        "баланс мощности понижающего преобразователя",
        Tolerance::Relative(1e-9),
        "",
    ));

    // Сводка питания против ручного сложения.
    let input = PowerInput {
        battery: Battery {
            nominal_v: 7.4,
            full_v: 8.4,
            empty_v: 6.4,
            capacity_ah: 2.0,
            max_continuous_a: Some(10.0),
            max_peak_a: Some(20.0),
            internal_resistance_ohm: Some(0.05),
        },
        rails: vec![RailInput {
            name: "3.3 В".into(),
            regulator: ams,
            output_v: 3.3,
        }],
        actuators: Vec::new(),
        sensors: Vec::new(),
        boards: vec![
            BoardLoad {
                name: "контроллер".into(),
                voltage_v: 3.3,
                current_avg_a: 0.10,
                current_peak_a: 0.30,
                rail: Some(0),
            },
            BoardLoad {
                name: "нагрузка от батареи".into(),
                voltage_v: 7.4,
                current_avg_a: 0.50,
                current_peak_a: 1.00,
                rail: None,
            },
        ],
        ambient_c: 25.0,
        required_runtime_s: None,
    };
    let b = power_budget(&input);
    let iq = val(ams.quiescent_a);
    v.push(check(
        "act_power_budget_avg",
        "Сводка питания: средняя мощность против ручного сложения",
        b.total_avg_w,
        6.4 * (0.10 + iq) + 7.4 * 0.50,
        "Вт",
        "power_budget",
        "V_разр·(I_нагр + I_q) для линейной шины + прямая нагрузка",
        Tolerance::Relative(1e-9),
        "",
    ));
    let p_peak = 6.4 * (0.30 + iq) + 7.4 * 1.00;
    v.push(check(
        "act_power_budget_peak",
        "Сводка питания: пиковый ток батареи против ручного сложения",
        b.total_peak_a,
        p_peak / (3.2 + (3.2_f64.powi(2) - 0.05 * p_peak).sqrt()),
        "А",
        "power_budget",
        "ручной баланс мощности и больший корень U² − U_разр·U + R·P = 0",
        Tolerance::Relative(1e-9),
        "",
    ));
    v.push(check(
        "act_battery_terminal_energy_balance",
        "Батарея: мощность химического источника против нагрузки и потерь",
        6.4 * b.total_peak_a,
        p_peak + 0.05 * b.total_peak_a.powi(2),
        "Вт",
        "U_разр·I по пиковому току сводки",
        "P_нагрузки + I²R по эквиваленту Тевенина",
        Tolerance::Relative(1e-9),
        "Параметры батареи заданы синтетическим тестом.",
    ));
}

/// Противоречия внутри данных производителей. «Симуляция» — альтернативное значение или
/// расчёт из других чисел паспорта, «эталон» — напечатанное значение. Часть не проходит
/// допуск 5 %: это находка о данных; в заметке сказано, что взято в каталог.
pub fn datasheet_discrepancies() -> Vec<Check> {
    let tol = Tolerance::Relative(0.05);
    let mut v = Vec::new();
    let mut add = |id: &'static str,
                   title: &str,
                   sim: f64,
                   reference: f64,
                   unit: &'static str,
                   method: &str,
                   ref_method: &str,
                   note: &str| {
        v.push(check(
            id, title, sim, reference, unit, method, ref_method, tol, note,
        ));
    };

    add(
        "ds_act_maxon_stall_torque",
        "maxon EC 90: K_t·V/R против стопового момента на текущей странице",
        0.231 * 48.0 / 0.844,
        9.41,
        "Н·м",
        "линейная модель",
        "maxon 500267: 9.41 Н·м (каталог 2022: 13.1)",
        "Предел по току в каталоге задан 15 А: до области расхождения расчёт не доходит.",
    );
    add(
        "ds_act_maxon_nominal_torque",
        "maxon EC 90: K_t·(I_N − I₀) против номинального момента",
        0.231 * (4.06 - 0.278),
        0.964,
        "Н·м",
        "0.231·(4.06 − 0.278)",
        "maxon 500267: 964 мН·м",
        "В каталоге — меньшее значение, из K_t и токов.",
    );
    add(
        "ds_act_maxon_nominal_speed",
        "maxon EC 90: (V − R·I_N)/K_t против номинальной скорости",
        (48.0 - 0.844 * 4.06) / 0.231 / RPM,
        1670.0,
        "об/мин",
        "баланс напряжений",
        "maxon 500267: 1670 об/мин",
        "Для вердикта K_e увеличен так, чтобы прямая проходила через более медленную паспортную точку.",
    );
    add(
        "ds_act_ak80_kt_kv",
        "AK80-64: 60/(2π·K_v) против K_t",
        60.0 / (2.0 * PI * 80.0),
        0.136,
        "Н·м/А",
        "из K_v = 80",
        "CubeMars: 0.136 Н·м/А",
        "Для момента берём опубликованные пары момент/ток, для скорости — наибольшую константу.",
    );
    add(
        "ds_act_ak80_no_load",
        "AK80-64: скорость холостого хода из K_t против паспорта",
        48.0 / (0.136 * 64.0) / RPM,
        75.0,
        "об/мин",
        "V/(k·N)",
        "CubeMars: 75 об/мин",
        "Расчёт идёт по меньшей скорости; возможная причина — ослабление поля, не подтверждено.",
    );
    add(
        "ds_act_ak80_peak_torque",
        "AK80-64: N·η·K_t·I_пик против пикового момента",
        64.0 * 0.85 * 0.136 * 19.0,
        120.0,
        "Н·м",
        "линейный K_t, η = 0.85 (оценка)",
        "CubeMars: 120 Н·м",
        "Связь момента с током берётся по паспортным парам, а не по линейному K_t.",
    );
    add(
        "ds_act_ak80_rated_speed",
        "AK80-64: скорость при номинальном токе из K_t против паспорта",
        (48.0 - 0.22 * 7.0) / (0.136 * 64.0) / RPM,
        48.0,
        "об/мин",
        "(V − R·I)/(k·N)",
        "CubeMars: 48 об/мин",
        "В расчёте K_e взят по паспортной точке.",
    );
    add(
        "ds_act_ak70_kt_kv",
        "AK70-10: 60/(2π·K_v) против K_t",
        60.0 / (2.0 * PI * 100.0),
        0.123,
        "Н·м/А",
        "из K_v = 100",
        "CubeMars: 0.123 Н·м/А",
        "Берём наибольшую константу для скорости.",
    );
    add(
        "ds_act_ak70_no_load",
        "AK70-10: скорость холостого хода из K_t против паспорта",
        48.0 / (0.123 * 10.0) / RPM,
        480.0,
        "об/мин",
        "V/(k·N)",
        "CubeMars: 480 об/мин",
        "Расчёт идёт по меньшей скорости.",
    );
    add(
        "ds_act_ak70_rated_speed",
        "AK70-10: скорость при номинальном токе из K_t против паспорта",
        (48.0 - 0.272 * 7.2) / (0.123 * 10.0) / RPM,
        310.0,
        "об/мин",
        "(V − R·I)/(k·N)",
        "CubeMars: 310 об/мин",
        "В расчёте K_e взят по паспортной точке.",
    );
    add(
        "ds_act_ak70_specific_torque",
        "AK70-10: пиковый момент на массу против напечатанного",
        24.8 / 0.621,
        47.6,
        "Н·м/кг",
        "24.8 Н·м / 621 г",
        "CubeMars: 47.6 Н·м/кг",
        "Напечатанное отношение в расчётах не используется.",
    );
    add(
        "ds_act_ak60_kt_kv",
        "AK60-6: 60/(2π·K_v) против K_t",
        60.0 / (2.0 * PI * 140.0),
        0.078,
        "Н·м/А",
        "из K_v = 140",
        "CubeMars: 0.078 Н·м/А",
        "Берём наибольшую константу для скорости.",
    );
    add(
        "ds_act_ak60_no_load",
        "AK60-6: 320 об/мин в разделе Core Data против 560 в таблице",
        320.0,
        560.0,
        "об/мин",
        "страница CubeMars, Core Data (24·80/6 = 320, это KV80)",
        "таблица сравнения, KV140 (24·140/6 = 560)",
        "Страница смешивает варианты KV80 и KV140.",
    );
    add(
        "ds_act_ak60_rated_speed",
        "AK60-6: скорость при номинальном токе из K_t против паспорта",
        (24.0 - 0.202 * 6.5) / (0.078 * 6.0) / RPM,
        420.0,
        "об/мин",
        "(V − R·I)/(k·N)",
        "CubeMars: 420 об/мин",
        "В расчёте K_e взят по паспортной точке.",
    );
    add(
        "ds_act_ak60_mass",
        "AK60-6: масса в тексте против таблицы",
        0.358,
        0.368,
        "кг",
        "CubeMars, список особенностей: ~358 г",
        "CubeMars, таблица: 368 г",
        "Берём большую массу.",
    );
    add(
        "ds_act_sts3215_resistance",
        "STS3215: V/R против стопового тока",
        7.4 / 2.5,
        2.5,
        "А",
        "7.4 В / 2.5 Ом",
        "Feetech: 2.5 А",
        "Расчёт идёт по таблице точек, сопротивление 2.5 Ом для прямой не используется.",
    );
    add(
        "ds_act_sg90_torque",
        "SG90: нижняя заявленная цифра момента против обычной",
        1.2,
        1.8,
        "кгс·см",
        "копия листка TowerPro",
        "большинство листков: 1.8 кгс·см",
        "Берём 1.2 кгс·см.",
    );
    add(
        "ds_act_sg90_stall_current",
        "SG90: измеренный стоповый ток против заявленного",
        0.36,
        0.65,
        "А",
        "Protosupplies, замер",
        "форум Arduino со ссылкой на производителя: 650 ± 80 мА",
        "Для расчёта питания берём 730 мА.",
    );
    add(
        "ds_act_mg90s_speed",
        "MG90S: скорость в таблице против текста того же листка",
        0.10,
        0.12,
        "с/60°",
        "AZ-Delivery, таблица",
        "AZ-Delivery, текст ниже таблицы",
        "Берём более медленную: 0.12 с/60°.",
    );
    add(
        "ds_act_mg996r_stall_current",
        "MG996R: стоповый ток в разных листингах",
        1.4,
        2.5,
        "А",
        "листинг продавца",
        "листок Tower Pro: 2.5 А при 6 В",
        "Для питания берём 2.5 А.",
    );
    add(
        "ds_act_ds3218_torque",
        "DS3218: момент при 5 В у разных продавцов",
        18.0,
        19.0,
        "кгс·см",
        "руководство DIYmall",
        "большинство листингов",
        "Берём 18 кгс·см.",
    );
    add(
        "ds_act_ds3218_stall_current",
        "DS3218: стоповый ток при 6.8 В по паспорту и листингу",
        2.6,
        2.2,
        "А",
        "листинг DS3218 MG",
        "DSSERVO, таблица электрических характеристик",
        "Для пика питания берём 2.6 А, для паспортной точки кривой оставляем 2.2 А.",
    );
    add(
        "ds_act_mg996r_voltage",
        "MG996R: верхнее напряжение из листка и с сайта TowerPro",
        7.2,
        6.6,
        "В",
        "листок MG996R, копия Cornell",
        "страница TowerPro MG996R",
        "Берём меньший верхний предел 6.6 В.",
    );
    add(
        "ds_act_nema17_resistance",
        "17HS19-2004S1: сопротивление в руководстве против чертежа",
        1.4,
        1.6,
        "Ом",
        "руководство продавца",
        "чертёж A0842: 1.60 Ом ± 10 %",
        "Для нагрева берём 1.6 Ом.",
    );
    // STEPPERONLINE, график 24 В / 2 А / полушаг: при 4000 имп/с (600 об/мин)
    // около 33 Н·см. Число считано с графика, поэтому это ориентир.
    let stepper = act("nema17-17hs19-2004s1");
    add(
        "ds_act_nema17_pull_out",
        "17HS19-2004S1: верхняя граница модели против графика при 600 об/мин",
        stepper
            .available_torque_nm(24.0, 600.0 * RPM, T_REF_C)
            .unwrap_or(f64::NAN),
        0.33,
        "Н·м",
        "расчёт по импедансу, без противо-ЭДС",
        "STEPPERONLINE, оценка по графику pull-out, 24 В, 2 А, полушаг",
        "Модель импеданса служит верхней границей. Без настроек драйвера проверка запаса остаётся NotAnalyzed.",
    );
    add(
        "ds_act_n20_298_gearbox",
        "N20 298:1: предел редуктора против стопового момента",
        2.0,
        4.0,
        "кгс·см",
        "Pololu: предел мгновенного момента",
        "Pololu: экстраполированный стоповый момент",
        "Предел в каталоге — 2 кгс·см: мотор способен сломать свой редуктор.",
    );
    add(
        "ds_act_ams1117_current",
        "AMS1117: ток по паспорту AMS против заявленного продавцами",
        0.8,
        1.0,
        "А",
        "AMS: 800 мА",
        "листинги и копии: 1 А",
        "Берём 800 мА.",
    );
    add(
        "ds_act_tb6612_current",
        "TB6612: рабочий ток против заголовка паспорта",
        1.0,
        1.2,
        "А",
        "Toshiba, рабочий диапазон",
        "Toshiba, заголовок: 1.2 А (ср.)",
        "Берём 1.0 А.",
    );
    add(
        "ds_act_bts7960_limit",
        "BTS7960: минимальный порог ограничения против типового «43 А»",
        33.0,
        43.0,
        "А",
        "Infineon: нижний ключ, 150 °C, минимум",
        "Infineon: заголовок, тип.",
        "Пик в каталоге — 33 А.",
    );
    add(
        "ds_act_bts7960_thermal",
        "BTS7960: длительный ток по нагреву без радиатора против предела 40 А",
        ((150.0f64 - 25.0) / (35.0 * 0.0305)).sqrt(),
        40.0,
        "А",
        "√((T_j − T_a)/(R_θJA·R_on)), 6 см² меди",
        "Infineon: 40 А при температуре корпуса до 85 °C",
        "40 А требуют радиатора: без него проверка нагрева даст Fail намного раньше.",
    );
    add(
        "ds_act_l298_total_drop",
        "L298: напечатанный итог падения против суммы максимумов",
        4.9,
        2.7 + 2.3,
        "В",
        "ST: Total Drop при 2 А",
        "ST: VCEsat(H) + VCEsat(L), максимумы",
        "Берём напечатанный итог 4.9 В.",
    );
    v
}
