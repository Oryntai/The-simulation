//! Решатель схем против аналитических решений.

use sim_verify::{Check, Tolerance};

use super::{failed, interp, lambert_w_of_exp, mk, zero_crossings};
use crate::devices::thermal_voltage;
use crate::error::SimError;
use crate::netlist::TranSpec;
use crate::{Circuit, SimOptions, Transient, operating_point, run_tran, run_tran_with};

/// RC-цепь, ступенька 5 В при t = 0 (UIC, конденсатор разряжен), τ = 1 мс.
pub const RC_STEP: &str = "RC ступенька
V1 in 0 DC 5
R1 in out 1k
C1 out 0 1u IC=0
.tran 5u 5m uic
.end
";

/// RL-цепь, ступенька 10 В при t = 0, τ = L/R = 1 мс, установившийся ток 1 А.
pub const RL_STEP: &str = "RL ступенька
V1 in 0 DC 10
R1 in a 10
L1 a 0 10m IC=0
.tran 5u 5m uic
.end
";

/// Последовательный RLC, разряд конденсатора 1 В: α = R/2L = 5000 1/с, ω₀ = 31623 рад/с.
pub const RLC_RINGDOWN: &str = "RLC затухающие колебания
C1 a 0 1u IC=1
R1 a b 10
L1 b 0 1m IC=0
.tran 0.5u 3m uic
.end
";

/// LC-контур без потерь, конденсатор заряжен до 1 В.
pub const LC_TANK: &str = "LC контур
C1 a 0 1u IC=1
L1 a 0 1m IC=0
.tran 0.2u 19.869177m uic
.end
";

/// Диод 1N4148 через резистор от 5 В.
pub const DIODE_R: &str = "Диод и резистор
V1 in 0 DC 5
R1 in a 1k
D1 a 0 D1N4148
.model D1N4148 D(IS=2.52n N=1.752 RS=0.568)
.op
.end
";

/// Делитель напряжения 10 В на 3k/7k.
pub const DIVIDER: &str = "Делитель
V1 in 0 DC 10
R1 in out 3k
R2 out 0 7k
.op
.end
";

/// Неуравновешенный мост Уитстона с гальванометром 500 Ом.
pub const WHEATSTONE: &str = "Мост Уитстона
V1 top 0 DC 10
R1 top l 1k
R2 l 0 2k
R3 top r 1.5k
R4 r 0 2.5k
Rg l r 500
.op
.end
";

/// NMOS в насыщении с резистивной нагрузкой: Vgs = 2 В, Vto = 1 В, KP = 1 мА/В², λ = 0.02 1/В.
pub const NMOS_SAT: &str = "NMOS насыщение
Vdd vdd 0 DC 10
Vg g 0 DC 2
Rd vdd d 1k
M1 d g 0 0 nm W=10u L=10u
.model nm NMOS(VTO=1 KP=1m LAMBDA=0.02)
.op
.end
";

/// PMOS в насыщении: исток на 10 В, затвор 8 В (Vsg = 2 В), нагрузка 1 кОм на землю.
pub const PMOS_SAT: &str = "PMOS насыщение
Vdd vdd 0 DC 10
Vg g 0 DC 8
M1 d g vdd vdd pm W=10u L=10u
Rd d 0 1k
.model pm PMOS(VTO=-1 KP=1m LAMBDA=0.02)
.op
.end
";

/// Синхронный понижающий преобразователь 24 → 5 В, 200 кГц, идеальные ключи без мёртвого
/// времени (управление — взаимно инверсные PULSE), старт близко к установившемуся режиму.
pub const BUCK_SYNC_IDEAL: &str = "Синхронный buck 24-5 В
Vin vin 0 DC 24
Vg1 g1 0 PULSE(0 1 0 10n 10n 1.031667u 5u)
Vg2 g2 0 PULSE(1 0 0 10n 10n 1.031667u 5u)
S1 vin sw g1 0 swm
S2 sw 0 g2 0 swm
.model swm SW(RON=1m ROFF=1meg VT=0.5)
L1 sw out 47u IC=1
C1 out 0 47u IC=5
Rl out 0 5
.tran 20n 4m uic
.end
";

/// H-мост с ключами, которыми управляет хост (ШИМ из «прошивки»), и мотор R+L+противо-ЭДС.
pub const HBRIDGE_EXT_IDEAL: &str = "H-мост, ключи из кода
Vbus vbus 0 DC 12
S1 vbus a EXT RON=0.01 ROFF=1g
S2 a 0 EXT RON=0.01 ROFF=1g
S3 vbus b EXT RON=0.01 ROFF=1g
S4 b 0 EXT RON=0.01 ROFF=1g ON
Rm a m1 1
Lm m1 m2 1m
Vbemf m2 b EXT 4
.end
";

fn guard(
    id: &'static str,
    title: &str,
    unit: &'static str,
    f: impl FnOnce() -> Result<Check, SimError>,
) -> Check {
    f().unwrap_or_else(|e| failed(id, title, unit, e.to_string()))
}

/// Все сверки решателя схем.
pub fn all() -> Vec<Check> {
    vec![
        rc_step(),
        rl_step(),
        rlc_frequency(),
        rlc_decay(),
        lc_energy(),
        trap_order(),
        diode_lambert_w(),
        divider(),
        wheatstone(),
        nmos_saturation(),
        pmos_saturation(),
        buck_vout(),
        buck_ripple(),
        hbridge_ripple(),
        motor_cosim_speed(),
        adc_settling(),
    ]
}

pub fn rc_step() -> Check {
    let (id, title) = ("circuit.rc_step", "RC-цепь: напряжение на C при t = τ");
    guard(id, title, "В", || {
        let wf = run_tran(&Circuit::parse(RC_STEP)?)?;
        let v = wf.at("v(out)", 1e-3).unwrap_or(f64::NAN);
        Ok(mk(
            id,
            title,
            v,
            5.0 * (1.0 - (-1.0f64).exp()),
            "В",
            "MNA, трапеции (1-й шаг — Эйлер), шаг τ/200",
            "V·(1 − e^(−t/RC))",
            Tolerance::Relative(1e-4),
            "R = 1 кОм, C = 1 мкФ, V = 5 В, старт с нуля (UIC)",
        ))
    })
}

pub fn rl_step() -> Check {
    let (id, title) = ("circuit.rl_step", "RL-цепь: ток через L при t = τ");
    guard(id, title, "А", || {
        let wf = run_tran(&Circuit::parse(RL_STEP)?)?;
        let i = wf.at("i(l1)", 1e-3).unwrap_or(f64::NAN);
        Ok(mk(
            id,
            title,
            i,
            1.0 - (-1.0f64).exp(),
            "А",
            "MNA (ток L — неизвестная ветви), трапеции, шаг τ/200",
            "(V/R)·(1 − e^(−tR/L))",
            Tolerance::Relative(1e-4),
            "R = 10 Ом, L = 10 мГн, V = 10 В",
        ))
    })
}

const RLC_R: f64 = 10.0;
const RLC_L: f64 = 1e-3;
const RLC_C: f64 = 1e-6;

fn rlc_alpha_wd() -> (f64, f64) {
    let alpha = RLC_R / (2.0 * RLC_L);
    let w0sq = 1.0 / (RLC_L * RLC_C);
    (alpha, (w0sq - alpha * alpha).sqrt())
}

/// Аналитика разряда: v(t) = e^(−αt)·(cos ωd·t + (α/ωd)·sin ωd·t).
fn rlc_exact(t: f64) -> f64 {
    let (a, wd) = rlc_alpha_wd();
    (-a * t).exp() * ((wd * t).cos() + a / wd * (wd * t).sin())
}

pub fn rlc_frequency() -> Check {
    let (id, title) = ("circuit.rlc_frequency", "RLC: период затухающих колебаний");
    guard(id, title, "с", || {
        let wf = run_tran(&Circuit::parse(RLC_RINGDOWN)?)?;
        let v = wf.get("v(a)").unwrap_or(&[]);
        let c = zero_crossings(&wf.time, v, true);
        let period = if c.len() >= 2 {
            (c[c.len() - 1] - c[0]) / (c.len() - 1) as f64
        } else {
            f64::NAN
        };
        let (_, wd) = rlc_alpha_wd();
        Ok(mk(
            id,
            title,
            period,
            std::f64::consts::TAU / wd,
            "с",
            "MNA, трапеции, шаг 0.5 мкс; переходы v(C) через ноль, линейная интерполяция",
            "2π/√(1/LC − (R/2L)²)",
            Tolerance::Relative(1e-4),
            "R = 10 Ом, L = 1 мГн, C = 1 мкФ; фазовая ошибка трапеций ≈ (ωh)²/12 ≈ 2e-5",
        ))
    })
}

pub fn rlc_decay() -> Check {
    let (id, title) = ("circuit.rlc_decay", "RLC: коэффициент затухания α");
    guard(id, title, "1/с", || {
        let wf = run_tran(&Circuit::parse(RLC_RINGDOWN)?)?;
        let v = wf.get("v(a)").unwrap_or(&[]);
        let i = wf.get("i(l1)").unwrap_or(&[]);
        // Экстремумы v(C) — там, где ток равен нулю (v' ∝ i).
        let mut peaks: Vec<f64> = zero_crossings(&wf.time, i, true);
        peaks.extend(zero_crossings(&wf.time, i, false));
        peaks.sort_by(f64::total_cmp);
        let alpha = if peaks.len() >= 9 {
            let (t0, t1) = (peaks[0], peaks[8]);
            let (v0, v1) = (interp(&wf.time, v, t0), interp(&wf.time, v, t1));
            (v0.abs() / v1.abs()).ln() / (t1 - t0)
        } else {
            f64::NAN
        };
        Ok(mk(
            id,
            title,
            alpha,
            RLC_R / (2.0 * RLC_L),
            "1/с",
            "отношение экстремумов v(C) через 8 полупериодов (моменты i(L) = 0)",
            "α = R/(2L)",
            Tolerance::Relative(1e-3),
            "трапеции не вносят численного затухания; ошибка — интерполяция экстремума",
        ))
    })
}

pub fn lc_energy() -> Check {
    let (id, title) = ("circuit.lc_energy", "LC-контур: энергия через 100 периодов");
    guard(id, title, "Дж", || {
        let ckt = Circuit::parse(LC_TANK)?;
        let wf = run_tran(&ckt)?;
        let (v, i) = (
            *wf.get("v(a)").and_then(|s| s.last()).unwrap_or(&f64::NAN),
            *wf.get("i(l1)").and_then(|s| s.last()).unwrap_or(&f64::NAN),
        );
        let e = 0.5 * RLC_C * v * v + 0.5 * RLC_L * i * i;
        Ok(mk(
            id,
            title,
            e,
            0.5 * RLC_C * 1.0,
            "Дж",
            "MNA, трапеции, шаг T/1000, ½CV² + ½LI² в конце",
            "начальная энергия ½·C·V₀² (контур без потерь)",
            Tolerance::Relative(2e-4),
            "трапеции сохраняют квадратичный инвариант линейного LC-контура; ожидаемая потеря \
             ≈ (ωh)² ≈ 4e-5 — от единственного шага неявного Эйлера на старте",
        ))
    })
}

/// Ошибка RLC-разряда в момент 1 мс при шаге h.
fn rlc_error_at(h: f64) -> Result<f64, SimError> {
    let ckt = Circuit::parse(RLC_RINGDOWN)?;
    let spec = TranSpec {
        tstep: h,
        tstop: 1e-3,
        tstart: 0.0,
        tmax: None,
        uic: true,
    };
    let wf = run_tran_with(&ckt, &spec, SimOptions::default())?;
    let v = *wf.get("v(a)").and_then(|s| s.last()).unwrap_or(&f64::NAN);
    Ok((v - rlc_exact(1e-3)).abs())
}

pub fn trap_order() -> Check {
    let (id, title) = (
        "circuit.trap_order",
        "Порядок сходимости трапеций (RLC, деление шага)",
    );
    guard(id, title, "", || {
        let e1 = rlc_error_at(2e-6)?;
        let e2 = rlc_error_at(1e-6)?;
        let e3 = rlc_error_at(0.5e-6)?;
        let order = 0.5 * ((e1 / e2).log2() + (e2 / e3).log2());
        Ok(mk(
            id,
            title,
            order,
            2.0,
            "",
            &format!(
                "log₂ отношения ошибок при h = 2, 1, 0.5 мкс (ошибки {e1:.2e}, {e2:.2e}, {e3:.2e} В)"
            ),
            "теоретический порядок метода трапеций",
            Tolerance::Absolute(0.1),
            "первый шаг Эйлера даёт локальную ошибку O(h²) один раз — глобальный порядок 2 сохраняется",
        ))
    })
}

/// Ток диода с последовательным сопротивлением через функцию Ламберта:
/// V = I·Rt + n·Vt·ln(I/Is + 1), Rt = R + RS.
pub fn diode_current_lambert(v: f64, r_total: f64, is: f64, n: f64) -> f64 {
    let nvt = n * thermal_voltage();
    let l = (v + is * r_total) / nvt - (nvt / (r_total * is)).ln();
    let w = lambert_w_of_exp(l);
    w * nvt / r_total - is
}

pub fn diode_lambert_w() -> Check {
    let (id, title) = ("circuit.diode_lambert_w", "Диод + резистор: ток");
    guard(id, title, "А", || {
        let op = operating_point(&Circuit::parse(DIODE_R)?)?;
        let i = op.current("D1")?;
        Ok(mk(
            id,
            title,
            i,
            diode_current_lambert(5.0, 1000.0 + 0.568, 2.52e-9, 1.752),
            "А",
            "рабочая точка: Ньютон с ограничителем pnjlim",
            "точное решение через функцию Ламберта W (Banwell & Jayakumar, Electron. Lett. 36(4), 2000, DOI 10.1049/el:20000301)",
            Tolerance::Relative(1e-6),
            "модель 1N4148: IS = 2.52 нА, N = 1.752, RS = 0.568 Ом; ток gmin (1e-12 См) пренебрежимо мал",
        ))
    })
}

pub fn divider() -> Check {
    let (id, title) = ("circuit.divider", "Делитель напряжения 3k/7k от 10 В");
    guard(id, title, "В", || {
        let op = operating_point(&Circuit::parse(DIVIDER)?)?;
        Ok(mk(
            id,
            title,
            op.voltage("out")?,
            10.0 * 7.0 / 10.0,
            "В",
            "MNA, рабочая точка",
            "V·R2/(R1+R2)",
            Tolerance::Relative(1e-8),
            "шунт узлов 1e-12 См даёт ошибку ≈ 2e-9",
        ))
    })
}

pub fn wheatstone() -> Check {
    let (id, title) = ("circuit.wheatstone", "Мост Уитстона: ток гальванометра");
    guard(id, title, "А", || {
        let op = operating_point(&Circuit::parse(WHEATSTONE)?)?;
        let (r1, r2, r3, r4, rg, v) = (1e3, 2e3, 1.5e3, 2.5e3, 500.0, 10.0);
        let vth = v * (r2 / (r1 + r2) - r4 / (r3 + r4));
        let rth = r1 * r2 / (r1 + r2) + r3 * r4 / (r3 + r4);
        Ok(mk(
            id,
            title,
            op.current("Rg")?,
            vth / (rth + rg),
            "А",
            "MNA, рабочая точка",
            "теорема Тевенина для диагонали моста",
            Tolerance::Relative(1e-7),
            "",
        ))
    })
}

/// Ток насыщения NMOS/PMOS с нагрузкой Rd: I = β/2·Vov²·(1 + λ(Vdd − I·Rd)) — решаем линейно.
fn square_law_with_load(beta: f64, vov: f64, lambda: f64, vdd: f64, rd: f64) -> f64 {
    let k = 0.5 * beta * vov * vov;
    k * (1.0 + lambda * vdd) / (1.0 + k * lambda * rd)
}

pub fn nmos_saturation() -> Check {
    let (id, title) = ("circuit.nmos_sat", "NMOS в насыщении: ток стока");
    guard(id, title, "А", || {
        let op = operating_point(&Circuit::parse(NMOS_SAT)?)?;
        Ok(mk(
            id,
            title,
            op.current("M1")?,
            square_law_with_load(1e-3, 1.0, 0.02, 10.0, 1e3),
            "А",
            "рабочая точка: Ньютон с ограничителями fetlim/limvds",
            "квадратичный закон Id = KP/2·W/L·(Vgs−Vto)²·(1+λVds), Vds = Vdd − Id·Rd",
            Tolerance::Relative(1e-7),
            "сверяет решатель, знаки и стыковку с нагрузкой; сама модель — та же формула \
             Шихмана–Ходжеса; утечка перехода подложки (≈1e-11 А) пренебрежима",
        ))
    })
}

pub fn pmos_saturation() -> Check {
    let (id, title) = ("circuit.pmos_sat", "PMOS в насыщении: ток через нагрузку");
    guard(id, title, "А", || {
        let op = operating_point(&Circuit::parse(PMOS_SAT)?)?;
        Ok(mk(
            id,
            title,
            op.current("Rd")?,
            square_law_with_load(1e-3, 1.0, 0.02, 10.0, 1e3),
            "А",
            "рабочая точка (полярность PMOS: Vto = −1 В, Vsg = 2 В)",
            "квадратичный закон с |Vgs|, |Vds|",
            Tolerance::Relative(1e-7),
            "проверка знаков для p-канала",
        ))
    })
}

/// Установившийся режим buck: (среднее Vout за 10 периодов, размах тока L за последний период).
pub fn buck_steady_state() -> Result<(f64, f64), SimError> {
    let wf = run_tran(&Circuit::parse(BUCK_SYNC_IDEAL)?)?;
    let t = &wf.time;
    let (v, i) = (
        wf.get("v(out)").unwrap_or(&[]),
        wf.get("i(l1)").unwrap_or(&[]),
    );
    let t_end = *t.last().unwrap_or(&0.0);
    let period = 5e-6;
    let from10 = t.partition_point(|&x| x < t_end - 10.0 * period - 1e-12);
    // Среднее по трапециям на ровно 10 периодах.
    let mut acc = 0.0;
    for k in from10 + 1..t.len() {
        acc += 0.5 * (v[k] + v[k - 1]) * (t[k] - t[k - 1]);
    }
    let vavg = acc / (t[t.len() - 1] - t[from10]);
    let from1 = t.partition_point(|&x| x < t_end - period - 1e-12);
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &x in &i[from1..] {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    Ok((vavg, hi - lo))
}

const BUCK_VIN: f64 = 24.0;
const BUCK_T: f64 = 5e-6;
/// Эффективная скважность: ключ включён на tr + pw (см. описание в `BUCK_SYNC_IDEAL`).
const BUCK_D: f64 = (10e-9 + 1.031667e-6) / BUCK_T;

pub fn buck_vout() -> Check {
    let (id, title) = (
        "circuit.buck_vout",
        "Buck 24→5 В: среднее выходное напряжение",
    );
    guard(id, title, "В", || {
        let (vavg, _) = buck_steady_state()?;
        let (rl, ron) = (5.0, 1e-3);
        Ok(mk(
            id,
            title,
            vavg,
            BUCK_D * BUCK_VIN * rl / (rl + ron),
            "В",
            "MNA, ключи SW по PULSE, шаг 20 нс, 4 мс; среднее за 10 периодов",
            "усреднение: Vout = D·Vin·Rн/(Rн + Ron) (точно для линейной периодической схемы)",
            Tolerance::Relative(2e-4),
            "200 кГц, L = 47 мкГн, C = 47 мкФ, Rн = 5 Ом, Ron = 1 мОм, без мёртвого времени; \
             остаток переходного процесса LC-фильтра ≈ 1e-4",
        ))
    })
}

pub fn buck_ripple() -> Check {
    let (id, title) = ("circuit.buck_ripple", "Buck 24→5 В: размах тока дросселя");
    guard(id, title, "А", || {
        let (_, ripple) = buck_steady_state()?;
        let l = 47e-6;
        let vout = BUCK_D * BUCK_VIN;
        Ok(mk(
            id,
            title,
            ripple,
            (BUCK_VIN - vout) * BUCK_D / (l * (1.0 / BUCK_T)),
            "А",
            "MNA, размах i(L) за последний период",
            "ΔI = (Vin − Vout)·D/(L·f), режим непрерывного тока",
            Tolerance::Relative(0.01),
            "формула пренебрегает пульсацией Vout (≈6 мВ) и падением на Ron — отсюда допуск 1%",
        ))
    })
}

/// Периодический режим мотора на ШИМ (знак–модуль): (i_max, i_min) аналитически.
pub fn rl_pwm_extremes(vbus: f64, e: f64, r: f64, l: f64, ton: f64, toff: f64) -> (f64, f64) {
    let tau = l / r;
    let (ion, ioff) = ((vbus - e) / r, -e / r);
    let a = (-ton / tau).exp();
    let b = (-toff / tau).exp();
    let imax = (ion * (1.0 - a) + a * ioff * (1.0 - b)) / (1.0 - a * b);
    let imin = ioff * (1.0 - b) + b * imax;
    (imax, imin)
}

/// H-мост на ключах EXT, ШИМ 20 кГц из цикла хоста. Вернуть размах тока мотора в
/// установившемся режиме.
pub fn hbridge_ripple_sim(duty: f64, dt: f64, t_end: f64) -> Result<f64, SimError> {
    let ckt = Circuit::parse(HBRIDGE_EXT_IDEAL)?;
    let mut tr = Transient::new(&ckt, dt)?;
    let steps_per_period = (50e-6 / dt).round() as usize;
    let on_steps = (duty * steps_per_period as f64).round() as usize;
    let n = (t_end / dt).round() as usize;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    tr.set_switch("S3", false)?;
    tr.set_switch("S4", true)?;
    for k in 0..n {
        let on = k % steps_per_period < on_steps;
        tr.set_switch("S1", on)?;
        tr.set_switch("S2", !on)?;
        tr.step()?;
        if k + 1 >= n - 5 * steps_per_period {
            let i = tr.current("Lm")?;
            lo = lo.min(i);
            hi = hi.max(i);
        }
    }
    Ok(hi - lo)
}

pub fn hbridge_ripple() -> Check {
    let (id, title) = (
        "circuit.hbridge_ripple",
        "H-мост, ШИМ 20 кГц: размах тока мотора",
    );
    guard(id, title, "А", || {
        let ripple = hbridge_ripple_sim(0.6, 1e-6, 15e-3)?;
        let (imax, imin) = rl_pwm_extremes(12.0, 4.0, 1.02, 1e-3, 30e-6, 20e-6);
        Ok(mk(
            id,
            title,
            ripple,
            imax - imin,
            "А",
            "пошаговый API: ключи EXT из цикла хоста, шаг 1 мкс, 15 мс",
            "периодическое решение RL-цепи с ЭДС: экспоненты на фазах вкл/выкл",
            Tolerance::Relative(1e-3),
            "мотор: R = 1 Ом, L = 1 мГн, ЭДС 4 В; Vbus = 12 В, D = 0.6, знак–модуль; \
             в контуре всегда два ключа по 0.01 Ом",
        ))
    })
}

/// Параметры мотора для совместного моделирования (оценка — типичный малый щёточный мотор).
pub struct MotorParams {
    pub k: f64,
    pub j: f64,
    pub b: f64,
    pub t_load: f64,
}

pub const MOTOR: MotorParams = MotorParams {
    k: 0.05,
    j: 2e-5,
    b: 1e-5,
    t_load: 0.01,
};

/// Мотор на H-мосту: электрика — наш решатель, механика — явный шаг хоста.
/// Вернуть среднюю скорость за последние 20 периодов ШИМ, рад/с.
pub fn motor_cosim(duty: f64, dt: f64, t_end: f64) -> Result<f64, SimError> {
    let ckt = Circuit::parse(HBRIDGE_EXT_IDEAL)?;
    let mut tr = Transient::new(&ckt, dt)?;
    let m = MOTOR;
    let steps_per_period = (50e-6 / dt).round() as usize;
    let on_steps = (duty * steps_per_period as f64).round() as usize;
    let n = (t_end / dt).round() as usize;
    let avg_from = n - 20 * steps_per_period;
    let mut omega = 0.0;
    let mut acc = 0.0;
    tr.set_switch("S3", false)?;
    tr.set_switch("S4", true)?;
    tr.set_source("Vbemf", 0.0)?;
    for k in 0..n {
        let on = k % steps_per_period < on_steps;
        tr.set_switch("S1", on)?;
        tr.set_switch("S2", !on)?;
        tr.step()?;
        let i = tr.current("Lm")?;
        omega += dt / m.j * (m.k * i - m.b * omega - m.t_load);
        tr.set_source("Vbemf", m.k * omega)?;
        if k >= avg_from {
            acc += omega;
        }
    }
    Ok(acc / (n - avg_from) as f64)
}

pub fn motor_cosim_speed() -> Check {
    let (id, title) = (
        "circuit.motor_cosim",
        "Мотор + H-мост (совместное моделирование): средняя скорость",
    );
    guard(id, title, "рад/с", || {
        let duty = 0.6;
        let w = motor_cosim(duty, 1e-6, 0.3)?;
        let m = MOTOR;
        let r = 1.02;
        let reference = (m.k * duty * 12.0 - r * m.t_load) / (r * m.b + m.k * m.k);
        Ok(mk(
            id,
            title,
            w,
            reference,
            "рад/с",
            "схема — наш решатель (шаг 1 мкс, 0.3 с), механика — шаг хоста, связь через EXT-ЭДС",
            "усреднённая модель: ω = (k·D·Vbus − R·Tн)/(R·b + k²)",
            Tolerance::Relative(1e-3),
            "k = 0.05 Н·м/А, J = 2e-5 кг·м², b = 1e-5 Н·м·с, Tн = 0.01 Н·м (оценка); \
             для линейной периодической системы среднее точно подчиняется усреднённой модели",
        ))
    })
}

/// Вход АЦП: при сопротивлении источника, равном пределу по формуле ST AN2834, конденсатор
/// выборки за время выборки должен недозарядиться ровно на ¼ младшего разряда.
pub fn adc_settling() -> Check {
    let (id, title) = (
        "circuit.adc_settling",
        "АЦП 12 бит: недозаряд выборки при предельном сопротивлении источника",
    );
    guard(id, title, "", || {
        let (ts, cs, rsw, bits) = (1e-6, 5e-12, 1e3, 12);
        let rmax = crate::adc::max_source_resistance(ts, cs, rsw, bits);
        let net = format!(
            "выборка АЦП\nV1 src 0 DC 1\nRs src adc {rmax:e}\nS1 adc smp EXT RON={rsw:e} ROFF=1g ON\n\
             Csh smp 0 {cs:e} IC=0\n.tran 1n {ts:e} uic\n"
        );
        let ckt = Circuit::parse(&net)?;
        let spec = ckt
            .tran_spec()
            .ok_or(SimError::Invalid("нет .tran".into()))?;
        // Плавающих узлов нет — шунт узлов убираем: остаток 6e-5 В чувствителен к нему.
        let opts = SimOptions {
            gshunt: 0.0,
            ..SimOptions::default()
        };
        let wf = run_tran_with(&ckt, &spec, opts)?;
        let v = *wf.get("v(smp)").and_then(|s| s.last()).unwrap_or(&f64::NAN);
        Ok(mk(
            id,
            title,
            1.0 - v,
            2f64.powi(-(bits as i32 + 2)),
            "",
            "схема: источник 1 В → R_max → ключ выборки 1 кОм → 5 пФ, 1 мкс, шаг 1 нс",
            "критерий ST AN2834: e^(−Ts/((R+Rкл)·C)) = 2^−(N+2), т.е. ¼ LSB",
            Tolerance::Relative(1e-3),
            &format!(
                "R_max = {rmax:.0} Ом; расчёт без шунта узлов на землю (иначе 1e-12 См сместили \
                 бы остаток на ≈7e-4 отн.); ошибка трапеций и первого шага Эйлера ≈1e-4 отн."
            ),
        ))
    })
}
