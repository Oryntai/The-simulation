//! Перекрёстная сверка с ngspice: те же нетлисты, независимая программа (своя реализация
//! моделей, свой выбор шага, свой разреженный решатель). Для переходных процессов в
//! проверке показана точка наибольшего расхождения: наше значение и значение ngspice в ней.

use std::path::Path;

use sim_verify::{Check, Tolerance};

use super::circuit as cc;
use super::{failed, interp, mk};
use crate::examples;
use crate::netlist::TranSpec;
use crate::ngspice::{NgAnalysis, run};
use crate::{Circuit, SimOptions, operating_point, run_tran_with};

struct TranCase {
    id: &'static str,
    title: &'static str,
    netlist: &'static str,
    signal: &'static str,
    /// Своё окно расчёта (иначе — `.tran` нетлиста).
    spec: Option<TranSpec>,
    /// Наибольший шаг ngspice (иначе — шаг вывода).
    ng_tmax: Option<f64>,
    tol: f64,
    unit: &'static str,
    note: &'static str,
}

struct OpCase {
    id: &'static str,
    title: &'static str,
    netlist: &'static str,
    signal: &'static str,
    tol: f64,
    unit: &'static str,
}

const TRAN_CASES: &[TranCase] = &[
    TranCase {
        id: "ngspice.rc_step",
        title: "ngspice: RC-ступенька, v(out)",
        netlist: cc::RC_STEP,
        signal: "v(out)",
        spec: None,
        ng_tmax: None,
        tol: 1e-3,
        unit: "В",
        note: "",
    },
    TranCase {
        id: "ngspice.rlc_ringdown",
        title: "ngspice: разряд RLC, v(C)",
        netlist: cc::RLC_RINGDOWN,
        signal: "v(a)",
        spec: None,
        ng_tmax: None,
        tol: 2e-3,
        unit: "В",
        note: "",
    },
    TranCase {
        id: "ngspice.rlc_series",
        title: "ngspice: пример «RLC-контур», v(C)",
        netlist: examples::RLC_SERIES.netlist,
        signal: "v(b)",
        spec: None,
        ng_tmax: None,
        tol: 5e-3,
        unit: "В",
        note: "фронт PULSE 1 мкс",
    },
    TranCase {
        id: "ngspice.rectifier",
        title: "ngspice: пример «выпрямитель», v(out)",
        netlist: examples::HALF_WAVE_RECTIFIER.netlist,
        signal: "v(out)",
        spec: None,
        ng_tmax: None,
        tol: 0.03,
        unit: "В",
        note: "моменты отпирания диода у программ различаются на долю шага",
    },
    TranCase {
        id: "ngspice.motor_lowside",
        title: "ngspice: пример «мотор, ключ в нижнем плече», ток мотора",
        netlist: examples::MOTOR_LOWSIDE_PWM.netlist,
        signal: "i(vbemf)",
        spec: None,
        ng_tmax: None,
        tol: 0.02,
        unit: "А",
        note: "EXT-ЭДС заменена постоянным источником 4 В",
    },
    TranCase {
        id: "ngspice.hbridge_pwm",
        title: "ngspice: пример «H-мост с мёртвым временем», ток мотора",
        netlist: examples::HBRIDGE_PWM.netlist,
        signal: "i(vbemf)",
        spec: Some(TranSpec {
            tstep: 0.5e-6,
            tstop: 3e-3,
            tstart: 0.0,
            tmax: None,
            uic: false,
        }),
        ng_tmax: None,
        tol: 0.02,
        unit: "А",
        note: "ключи SW по PULSE, диоды в мёртвое время; EXT-ЭДС → 4 В",
    },
    TranCase {
        id: "ngspice.buck",
        title: "ngspice: синхронный buck 24→5 В, v(out)",
        netlist: cc::BUCK_SYNC_IDEAL,
        signal: "v(out)",
        spec: Some(TranSpec {
            tstep: 20e-9,
            tstop: 3e-3,
            tstart: 2.5e-3,
            tmax: None,
            uic: true,
        }),
        ng_tmax: Some(5e-9),
        tol: 1e-4,
        unit: "В",
        note: "сравнение на 2.5–3 мс (установившийся режим); ngspice с наибольшим шагом 20 нс \
               ещё не сошёлся (среднее Vout ниже на ≈3.5 мВ), поэтому ему дан шаг 5 нс; наш \
               решатель находит моменты переключения ключей интерполяцией порога и даёт тот же \
               результат при шагах 2–20 нс",
    },
];

const OP_CASES: &[OpCase] = &[
    OpCase {
        id: "ngspice.diode_op",
        title: "ngspice: диод 1N4148 + 1 кОм, напряжение на диоде",
        netlist: cc::DIODE_R,
        signal: "v(a)",
        tol: 1e-5,
        unit: "В",
    },
    OpCase {
        id: "ngspice.led_op",
        title: "ngspice: пример «светодиод», напряжение на светодиоде",
        netlist: examples::LED_RESISTOR.netlist,
        signal: "v(a)",
        tol: 1e-5,
        unit: "В",
    },
    OpCase {
        id: "ngspice.nmos_op",
        title: "ngspice: NMOS в насыщении, напряжение стока",
        netlist: cc::NMOS_SAT,
        signal: "v(d)",
        tol: 1e-5,
        unit: "В",
    },
    OpCase {
        id: "ngspice.pmos_op",
        title: "ngspice: PMOS в насыщении, напряжение стока",
        netlist: cc::PMOS_SAT,
        signal: "v(d)",
        tol: 1e-5,
        unit: "В",
    },
];

/// Все сверки с ngspice (`ng` — путь к программе).
pub fn all(ng: &Path) -> Vec<Check> {
    let mut v: Vec<Check> = OP_CASES.iter().map(|c| op_case(ng, c)).collect();
    v.extend(TRAN_CASES.iter().map(|c| tran_case(ng, c)));
    v
}

fn op_case(ng: &Path, c: &OpCase) -> Check {
    let ours = Circuit::parse(c.netlist)
        .and_then(|ckt| operating_point(&ckt))
        .and_then(|op| op.voltage(c.signal.trim_start_matches("v(").trim_end_matches(')')));
    let theirs = run(ng, c.netlist, NgAnalysis::Op, &[c.signal]);
    match (ours, theirs) {
        (Ok(a), Ok(t)) => mk(
            c.id,
            c.title,
            a,
            t.rows[0][0],
            c.unit,
            "наш решатель, рабочая точка",
            "ngspice-47, тот же нетлист (op)",
            Tolerance::Absolute(c.tol),
            "",
        ),
        (Err(e), _) => failed(c.id, c.title, c.unit, e.to_string()),
        (_, Err(e)) => failed(c.id, c.title, c.unit, e),
    }
}

fn tran_case(ng: &Path, c: &TranCase) -> Check {
    let run_ours = || -> Result<(Vec<f64>, Vec<f64>, TranSpec), String> {
        let ckt = Circuit::parse(c.netlist).map_err(|e| e.to_string())?;
        let spec = c.spec.or(ckt.tran_spec()).ok_or("в нетлисте нет .tran")?;
        let wf = run_tran_with(&ckt, &spec, SimOptions::default()).map_err(|e| e.to_string())?;
        let y = wf.get(c.signal).ok_or("нет такого сигнала")?.to_vec();
        Ok((wf.time, y, spec))
    };
    let (t, y, spec) = match run_ours() {
        Ok(r) => r,
        Err(e) => return failed(c.id, c.title, c.unit, e),
    };
    let ana = NgAnalysis::Tran {
        tstep: spec.tstep,
        tstop: spec.tstop,
        tstart: spec.tstart,
        tmax: c.ng_tmax.or(spec.tmax).unwrap_or(spec.tstep),
        uic: spec.uic,
    };
    let table = match run(ng, c.netlist, ana, &[c.signal]) {
        Ok(t) => t,
        Err(e) => return failed(c.id, c.title, c.unit, e),
    };
    let (Some(nt), Some(ny)) = (table.column("time"), table.column(c.signal)) else {
        return failed(
            c.id,
            c.title,
            c.unit,
            "в таблице ngspice нет time/сигнала".into(),
        );
    };
    let (t0, t1) = (nt[0], nt[nt.len() - 1]);
    let mut worst = (0.0, f64::NAN, f64::NAN);
    let mut n = 0;
    for (&tk, &yk) in t.iter().zip(&y) {
        if tk < t0 || tk > t1 {
            continue;
        }
        n += 1;
        let r = interp(&nt, &ny, tk);
        if worst.1.is_nan() || (yk - r).abs() > (worst.1 - worst.2).abs() {
            worst = (tk, yk, r);
        }
    }
    mk(
        c.id,
        c.title,
        worst.1,
        worst.2,
        c.unit,
        &format!(
            "наш решатель, шаг {:.3e} с; точка наибольшего расхождения t = {:.4e} с",
            spec.tstep, worst.0
        ),
        &format!(
            "ngspice-47, тот же нетлист, наибольший шаг {:.1e} с; линейная интерполяция",
            c.ng_tmax.or(spec.tmax).unwrap_or(spec.tstep)
        ),
        Tolerance::Absolute(c.tol),
        &format!(
            "сравнено {n} точек; допуск — наибольшее отклонение по всей осциллограмме. {}",
            c.note
        ),
    )
}
