//! Сверки VMCU-C3 и прошивок со вторым независимым способом.
//!
//! * Самопроверка: прошивка `selftest` (RISC-V, исполняется эмулятором) против
//!   того же исходника `selftest-core`, скомпилированного нативно для хоста (LLVM
//!   под aarch64/x86_64), и против значений из спецификации RISC-V.
//! * Декодер: кодировки, выданные ассемблером LLVM, против ожидаемых полей.
//! * Таймер: измеренный период прерывания против настроенного.
//! * `exo-ctrl` в замкнутом контуре со стендом [`crate::bench`]: ток
//!   гравитационной поддержки против расчёта «вручную», установка положения,
//!   время реакции каждой защиты против заявленной границы.
//! * Привод maxon с тепловой моделью: время срабатывания защиты от заклинивания,
//!   оценка температуры обмотки прошивкой против тепловой сети стенда (f64, истинный
//!   ток), пик обмотки при заклинивании с перезапусками, ток в полосе снижения.
//!
//! Сценарии — публичные функции: тесты и приложение могут вызывать их отдельно.

use std::collections::BTreeMap;

use sim_verify::{Check, Tolerance};

use crate::bench::{Bench, ExoPlant, PlantParams, PlantThermal};
use crate::cpu::{Decoded, decode};
use crate::exo::{self, ExoCommand, ExoConfig, ExoDiag, ExoMode, ExoTelemetry, fault, state};
use crate::{Mcu, firmware};

/// Тот же исходник, что `firmware/selftest-core`, скомпилированный для хоста.
#[allow(dead_code)]
pub mod native_selftest {
    include!("../../../firmware/selftest-core/src/selftest.rs");
}

// ---------------------------------------------------------------------------
// Вспомогательное

/// Пары (ключ, значение) самопроверки.
pub type KeyValues = Vec<(&'static str, u32)>;
/// Условия (выполнено, описание).
pub type Conditions = Vec<(bool, &'static str)>;

#[allow(clippy::too_many_arguments)]
fn mk(
    id: &'static str,
    title: &str,
    simulated: f64,
    reference: f64,
    unit: &'static str,
    method: &str,
    reference_method: &str,
    tolerance: Tolerance,
    note: &str,
) -> Check {
    Check {
        id,
        title: title.to_string(),
        simulated,
        reference,
        unit,
        method: method.to_string(),
        reference_method: reference_method.to_string(),
        tolerance,
        note: note.to_string(),
    }
}

/// Проверка «значение в интервале [lo; hi]»: эталон — середина, допуск — полуширина.
#[allow(clippy::too_many_arguments)]
fn range(
    id: &'static str,
    title: &str,
    simulated: f64,
    (lo, hi): (f64, f64),
    unit: &'static str,
    method: &str,
    reference_method: &str,
    note: &str,
) -> Check {
    mk(
        id,
        title,
        simulated,
        0.5 * (lo + hi),
        unit,
        method,
        reference_method,
        Tolerance::Absolute(0.5 * (hi - lo)),
        &format!("допуск — интервал [{lo}; {hi}] {unit}. {note}"),
    )
}

/// Проверка «выполнено условий из N».
fn conditions(
    id: &'static str,
    title: &str,
    conds: &[(bool, &str)],
    method: &str,
    reference_method: &str,
) -> Check {
    let ok = conds.iter().filter(|c| c.0).count();
    let failed: Vec<&str> = conds.iter().filter(|c| !c.0).map(|c| c.1).collect();
    let all: Vec<&str> = conds.iter().map(|c| c.1).collect();
    let note = if failed.is_empty() {
        format!("условия: {}", all.join("; "))
    } else {
        format!("НЕ выполнено: {}", failed.join("; "))
    };
    mk(
        id,
        title,
        ok as f64,
        conds.len() as f64,
        "усл.",
        method,
        reference_method,
        Tolerance::Absolute(0.0),
        &note,
    )
}

/// Сценарий не смог выполниться — проверка проваливается с причиной.
fn broken(id: &'static str, title: &str, err: &str) -> Check {
    mk(
        id,
        title,
        f64::NAN,
        0.0,
        "",
        "сценарий не выполнен",
        "—",
        Tolerance::Absolute(0.0),
        err,
    )
}

// ---------------------------------------------------------------------------
// Самопроверка

/// Итог прогона прошивки `selftest`.
#[derive(Clone, Debug, Default)]
pub struct SelftestRun {
    /// Строки `ST ключ значение` (результаты selftest-core).
    pub st: BTreeMap<String, u32>,
    /// Строки `SYS ключ значение` (системные проверки).
    pub sys: BTreeMap<String, u32>,
    /// Число ключей, которое прошивка сообщила в `ST end N`.
    pub end_count: Option<u32>,
    pub exit_code: Option<u32>,
    pub fault: Option<String>,
    /// Байты, отправленные в UART для проверки приёма.
    pub sent: Vec<u8>,
    pub instructions: u64,
    pub uart: String,
}

/// Байты для проверки приёма UART.
pub const SELFTEST_RX: &[u8; 16] = b"VMCU-C3 rx test!";

/// Запуск `selftest` до остановки (не дольше 2 с модели).
pub fn run_selftest() -> Result<SelftestRun, String> {
    let mut m = Mcu::from_elf(firmware::selftest_elf()).map_err(|e| e.to_string())?;
    let mut out = String::new();
    let mut sent = false;
    for _ in 0..2000 {
        m.run_for(1e-3).map_err(|e| e.to_string())?;
        out.push_str(&m.uart_take());
        if !sent && out.contains("SYS rx.ready") {
            m.uart_send(SELFTEST_RX);
            sent = true;
        }
        if m.is_halted() {
            break;
        }
    }
    let mut r = SelftestRun {
        exit_code: m.exit_code(),
        fault: m.fault().map(|f| f.to_string()),
        sent: SELFTEST_RX.to_vec(),
        instructions: m.instret(),
        ..Default::default()
    };
    for line in out.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.as_slice() {
            ["ST", "end", n] => r.end_count = n.parse().ok(),
            ["ST", k, v] => {
                if let Ok(v) = u32::from_str_radix(v, 16) {
                    r.st.insert(k.to_string(), v);
                }
            }
            ["SYS", k, v] => {
                if let Ok(v) = u32::from_str_radix(v, 16) {
                    r.sys.insert(k.to_string(), v);
                }
            }
            _ => {}
        }
    }
    r.uart = out;
    Ok(r)
}

/// Нативный расчёт: (результаты команд по эталонной семантике, результаты алгоритмов).
pub fn native_selftest() -> (KeyValues, KeyValues) {
    let mut isa = Vec::new();
    native_selftest::run_isa(&native_selftest::RefIsa, &mut |k, v| isa.push((k, v)));
    let mut algo = Vec::new();
    native_selftest::run_algo(&mut |k, v| algo.push((k, v)));
    (isa, algo)
}

/// Ожидаемые системные значения (из спецификации RISC-V и описания VMCU-C3).
pub fn selftest_sys_expectations() -> Vec<(&'static str, u32, &'static str)> {
    vec![
        ("misa", crate::cpu::MISA, "misa = RV32 + I, M, C"),
        ("mhartid", 0, "mhartid = 0"),
        ("mstatus.mpp", 3, "MPP = 11 (только M-режим)"),
        ("sysctrl.id", crate::periph::sysctrl::ID, "SYSCTRL.ID"),
        ("reset_reason", 1, "сброс по питанию"),
        ("mcycle.grows", 1, "mcycle растёт"),
        (
            "minstret.grows",
            1,
            "minstret растёт быстрее числа итераций",
        ),
        ("ecall.ret", 43, "ecall: обработчик вернул 2·21+1"),
        ("ecall.mcause", 11, "ecall из M-режима: mcause = 11"),
        ("trap.illegal16.mcause", 2, "c.unimp → mcause = 2"),
        ("trap.illegal16.mtval", 0, "c.unimp → mtval = биты (0)"),
        (
            "trap.illegal32.mcause",
            2,
            "запись в cycle (RO) → mcause = 2",
        ),
        (
            "trap.illegal32.mtval",
            0xc000_1073,
            "mtval = биты инструкции",
        ),
        ("trap.lw_misaligned.mcause", 4, "lw по нечётному адресу → 4"),
        ("trap.lw_misaligned.tval_ok", 1, "mtval = адрес доступа"),
        ("trap.lw_unmapped.mcause", 5, "загрузка из пустоты → 5"),
        ("trap.lw_unmapped.mtval", 0x1000_0000, "mtval = адрес"),
        ("trap.sw_flash.mcause", 7, "запись во флеш → 7"),
        ("trap.sw_flash.mtval", 0x4200_0000, "mtval = адрес"),
        ("trap.count", 6, "6 исключений"),
        (
            "timer.mcause",
            0x8000_0007,
            "прерывание таймера: mcause = 2³¹ + 7",
        ),
        (
            "rx.mcause",
            0x8000_000b,
            "внешнее прерывание: mcause = 2³¹ + 11",
        ),
        (
            "rx.crc",
            native_selftest::crc32(SELFTEST_RX),
            "CRC-32 принятых по UART байт",
        ),
    ]
}

fn selftest_checks(out: &mut Vec<Check>) {
    const T_ISA: &str =
        "Команды RV32IM в эмуляторе против эталонной семантики (совпавших значений)";
    const T_ALGO: &str = "Алгоритмы (CRC-32, FNV, Фибоначчи u64, сортировки, f32/f64, деление u64) — эмулятор против нативной сборки";
    let r = match run_selftest() {
        Ok(r) => r,
        Err(e) => {
            out.push(broken("mcu.selftest.isa", T_ISA, &e));
            out.push(broken("mcu.selftest.algo", T_ALGO, &e));
            return;
        }
    };
    let (isa, algo) = native_selftest();
    let cmp = |list: &[(&'static str, u32)]| -> (usize, Vec<String>) {
        let mut bad = Vec::new();
        let ok = list
            .iter()
            .filter(|(k, v)| {
                let same = r.st.get(*k) == Some(v);
                if !same {
                    bad.push(format!(
                        "{k}: эмулятор {:?} ≠ натив {v:08x}",
                        r.st.get(*k).map(|x| format!("{x:08x}"))
                    ));
                }
                same
            })
            .count();
        (ok, bad)
    };
    let (ok_isa, bad_isa) = cmp(&isa);
    let (ok_algo, bad_algo) = cmp(&algo);
    let status = match (&r.fault, r.exit_code) {
        (Some(f), _) => format!("СБОЙ: {f}. "),
        (None, Some(0)) => String::new(),
        (None, c) => format!("прошивка не завершилась штатно (код {c:?}). "),
    };
    out.push(mk(
        "mcu.selftest.isa",
        T_ISA,
        ok_isa as f64,
        isa.len() as f64,
        "знач.",
        "прошивка selftest (настоящие инструкции через asm!) в эмуляторе, вывод UART",
        "та же функция selftest-core с эталонной семантикой (RefIsa, по Volume I, гл. 2 и 7), собранная нативно",
        Tolerance::Absolute(0.0),
        &format!(
            "{status}18 команд «регистр–регистр» на сетке 16×16 операндов (CRC-32 результатов), граничные случаи деления \
             (на 0, INT_MIN/−1), 15 команд с непосредственным операндом, 6 переходов, 5 загрузок и 3 записи, \
             6 видов CSR-команд, LUI/AUIPC. {}",
            if bad_isa.is_empty() { String::new() } else { format!("Расхождения: {}", bad_isa.join("; ")) }
        ),
    ));
    out.push(mk(
        "mcu.selftest.algo",
        T_ALGO,
        ok_algo as f64,
        algo.len() as f64,
        "знач.",
        "прошивка selftest в эмуляторе (код, сжатые инструкции и soft-float — от компилятора)",
        "тот же исходник selftest-core, собранный LLVM под хост",
        Tolerance::Absolute(0.0),
        &format!(
            "{status}Совпадение побитовое, в т. ч. f32/f64 (+, ×, ÷ по IEEE 754 точно округляются на обеих платформах). {}",
            if bad_algo.is_empty() { String::new() } else { format!("Расхождения: {}", bad_algo.join("; ")) }
        ),
    ));
    out.push(mk(
        "mcu.selftest.crc32_check",
        "CRC-32(\"123456789\") в эмуляторе против опубликованного контрольного значения",
        r.st.get("crc32.check").map_or(f64::NAN, |&v| v as f64),
        0xcbf4_3926u32 as f64,
        "",
        "прошивка selftest в эмуляторе",
        "контрольное значение CRC-32/ISO-HDLC = 0xCBF43926 (каталог CRC RevEng)",
        Tolerance::Absolute(0.0),
        "третий независимый источник для таблицы CRC",
    ));

    let exp = selftest_sys_expectations();
    let conds: Vec<(bool, String)> = exp
        .iter()
        .map(|(k, v, what)| {
            let got = r.sys.get(*k);
            (
                got == Some(v),
                if got == Some(v) {
                    what.to_string()
                } else {
                    format!("{what} (получено {got:?})")
                },
            )
        })
        .collect();
    let conds_ref: Vec<(bool, &str)> = conds.iter().map(|(b, s)| (*b, s.as_str())).collect();
    out.push(conditions(
        "mcu.selftest.traps",
        "Ловушки, CSR и прерывания в эмуляторе против спецификации RISC-V",
        &conds_ref,
        "прошивка selftest: ecall, c.unimp, запись в RO-CSR, невыровненный lw, доступ в пустоту и во флеш, таймер, приём UART",
        "Privileged ISA 20211203: таблица 3.6 (mcause), 3.1.15 (mtval), misa; описание VMCU-C3",
    ));

    let period_cycles = r
        .sys
        .get("timer.period_sum20")
        .map_or(f64::NAN, |&v| v as f64 / 20.0);
    out.push(mk(
        "mcu.timer.period",
        "Период прерывания таймера 10 кГц: среднее по 20 периодам",
        period_cycles / 160.0,
        100.0,
        "мкс",
        "selftest: mcycle на входе в обработчик, mtimecmp += 1600 тиков",
        "настроенный период: 1600 тиков mtime при 16 МГц = 100 мкс",
        Tolerance::Absolute(1.0 / 16.0),
        "допуск — один тик mtime (62.5 нс)",
    ));
    let jitter = match (r.sys.get("timer.period_max"), r.sys.get("timer.period_min")) {
        (Some(&a), Some(&b)) => a as f64 - b as f64,
        _ => f64::NAN,
    };
    out.push(mk(
        "mcu.timer.jitter",
        "Дрожание периода прерывания таймера (макс − мин по 20 периодам)",
        jitter,
        0.0,
        "такт",
        "selftest: mcycle на входе в обработчик; ядро ждёт в wfi",
        "идеальный таймер без дрожания",
        Tolerance::Absolute(10.0),
        "допуск — один тик mtime (10 тактов): прерывание берётся на границе инструкции",
    ));
}

// ---------------------------------------------------------------------------
// Декодер против ассемблера LLVM

fn decoder_check(out: &mut Vec<Check>) {
    let pc = 0x4200_1000u32;
    let mut bad = Vec::new();
    let mut ok = 0;
    for v in crate::llvm_vectors::VECTORS {
        let d: Decoded = decode(v.raw, pc);
        let imm = if v.pc_rel {
            pc.wrapping_add(v.imm as i32 as u32)
        } else {
            v.imm as i32 as u32
        };
        let good = d.op == v.op
            && d.rd == v.rd
            && d.rs1 == v.rs1
            && d.rs2 == v.rs2
            && d.len == v.len
            && d.imm == imm;
        if good {
            ok += 1;
        } else {
            bad.push(format!("{} (0x{:x}) → {:?}", v.asm, v.raw, d));
        }
    }
    out.push(mk(
        "mcu.decode.llvm",
        "Декодер (в т. ч. все виды сжатых команд RV32C) против кодировок ассемблера LLVM",
        ok as f64,
        crate::llvm_vectors::VECTORS.len() as f64,
        "команд",
        "наш декодер: биты → операция, регистры, непосредственное значение",
        "кодировщик LLVM (rustc, riscv32imc) по мнемоникам с известным смыслом",
        Tolerance::Absolute(0.0),
        &if bad.is_empty() {
            "крайние значения смещений и непосредственных операндов каждого формата".to_string()
        } else {
            format!("Расхождения: {}", bad.join("; "))
        },
    ));
}

// ---------------------------------------------------------------------------
// exo-ctrl в замкнутом контуре

fn cmd(mode: ExoMode, assist: u8, target: f64, payload: f64) -> ExoCommand {
    ExoCommand {
        mode,
        assist_pct: assist,
        target_deg: target,
        payload_kg: payload,
        seq: 0,
    }
}

/// Стенд с прошивкой exo-ctrl. `hold` — угол удерживается внешней силой.
pub fn exo_bench(
    cfg: &ExoConfig,
    theta_deg: f64,
    hold: bool,
    payload_kg: f64,
) -> Result<Bench, String> {
    let mut plant = ExoPlant::new(PlantParams::default(), theta_deg);
    plant.payload_kg = payload_kg;
    if hold {
        plant.hold_deg = Some(theta_deg);
    }
    Bench::new(firmware::exo_ctrl_elf(), cfg, plant).map_err(|e| e.to_string())
}

/// Шагать мелким шагом `dt`, пока `cond` не станет истинным (не дольше `max_s`).
/// Возвращает время от начала, с.
fn time_until(
    b: &mut Bench,
    dt: f64,
    max_s: f64,
    mut cond: impl FnMut(&Bench) -> bool,
) -> Option<f64> {
    let saved = b.dt;
    b.dt = dt;
    let t0 = b.time();
    let mut res = None;
    while b.time() - t0 < max_s {
        b.step();
        if cond(b) {
            res = Some(b.time() - t0);
            break;
        }
    }
    b.dt = saved;
    res
}

/// Средний ток мотора за `seconds`.
fn mean_current(b: &mut Bench, seconds: f64) -> f64 {
    let (mut s, mut n) = (0.0, 0usize);
    let end = b.time() + seconds;
    while b.time() < end {
        b.step();
        s += b.plant.current;
        n += 1;
    }
    s / n.max(1) as f64
}

fn enabled(b: &Bench) -> bool {
    b.mcu.gpio_output(exo::pins::GPIO_ENABLE)
}

/// Флаги из телеметрии после момента `t`.
fn flags_since(b: &Bench, t: f64) -> u8 {
    b.telemetry
        .iter()
        .filter(|(tt, _)| *tt >= t)
        .fold(0, |acc, (_, tl)| acc | tl.fault_flags)
}

/// Флаги второго байта из телеметрии после момента `t`.
fn flags2_since(b: &Bench, t: f64) -> u8 {
    b.telemetry
        .iter()
        .filter(|(tt, _)| *tt >= t)
        .fold(0, |acc, (_, tl)| acc | tl.fault_flags2)
}

/// Частота тиков, телеметрии и загрузка ЦП в режиме поддержки (окно 1 с).
#[derive(Clone, Copy, Debug)]
pub struct RateResult {
    pub ticks_per_s: f64,
    pub telemetry_per_s: f64,
    pub cpu_load: f64,
    pub instructions_per_s: f64,
}

pub fn exo_rates() -> Result<RateResult, String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 90.0, true, 2.0)?;
    let c = cmd(ExoMode::GravityAssist, 100, 0.0, 2.0);
    b.send(c);
    b.run(0.5);
    let ticks0 = b.mcu.interrupt_count(crate::cpu::cause::M_TIMER_INT);
    let tl0 = b.telemetry.len();
    let (act0, cyc0, ins0) = (b.mcu.active_cycles(), b.mcu.cycles(), b.mcu.instret());
    for _ in 0..5 {
        b.send(c);
        b.run(0.2);
    }
    let dc = (b.mcu.cycles() - cyc0) as f64;
    Ok(RateResult {
        ticks_per_s: (b.mcu.interrupt_count(crate::cpu::cause::M_TIMER_INT) - ticks0) as f64,
        telemetry_per_s: (b.telemetry.len() - tl0) as f64,
        cpu_load: (b.mcu.active_cycles() - act0) as f64 / dc,
        instructions_per_s: (b.mcu.instret() - ins0) as f64,
    })
}

/// Ток гравитационной поддержки при удержании угла: (измерено, расчёт вручную), А.
pub fn exo_assist_current(theta_deg: f64, payload_kg: f64) -> Result<(f64, f64), String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, theta_deg, true, payload_kg)?;
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, payload_kg));
    b.run(0.15);
    let i = mean_current(&mut b, 0.05);
    Ok((i, cfg.gravity_assist_current(100.0, payload_kg, theta_deg)))
}

/// Установка положения 30° → 90°: (угол в конце, время установления ±1°, перерегулирование).
#[derive(Clone, Copy, Debug)]
pub struct PositionResult {
    pub final_deg: f64,
    pub settle_s: f64,
    pub overshoot_deg: f64,
}

/// Наибольшая скорость руки при переходе `from` → `to` с пределом скорости из конфигурации
/// по умолчанию: (пик скорости °/с, угол в конце).
pub fn exo_speed_limited_move(from: f64, to: f64) -> Result<(f64, f64), String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, from, false, 0.0)?;
    let c = cmd(ExoMode::PositionTrack, 0, to, 0.0);
    let t0 = b.time();
    let travel_s = (to - from).abs() / cfg.speed_limit_dps as f64 + 2.0;
    let mut peak = 0.0_f64;
    let mut prev = b.plant.theta_deg();
    let mut prev_t = t0;
    let mut k = 0;
    while b.time() - t0 < travel_s {
        b.send(c);
        k += 1;
        let end = t0 + 0.05 * k as f64;
        while b.time() < end {
            b.step();
        }
        let (th, t) = (b.plant.theta_deg(), b.time());
        peak = peak.max(((th - prev) / (t - prev_t)).abs());
        prev = th;
        prev_t = t;
    }
    Ok((peak, b.plant.theta_deg()))
}

/// Голый позиционный регулятор: без предела скорости и без реакции на контакт.
pub fn exo_position_step(from: f64, to: f64) -> Result<PositionResult, String> {
    let cfg = ExoConfig {
        speed_limit_dps: 0.0,
        contact_torque_nm: 0.0,
        ..ExoConfig::default()
    };
    let mut b = exo_bench(&cfg, from, false, 0.0)?;
    let c = cmd(ExoMode::PositionTrack, 0, to, 0.0);
    let t0 = b.time();
    let mut last_out = 0.0;
    let mut peak = f64::MIN;
    for k in 0..10 {
        b.send(c);
        let end = t0 + 0.2 * (k + 1) as f64;
        while b.time() < end {
            b.step();
            let th = b.plant.theta_deg();
            peak = peak.max(th);
            if (th - to).abs() > 1.0 {
                last_out = b.time() - t0;
            }
        }
    }
    let mut s = 0.0;
    let mut n = 0;
    let end = b.time() + 0.1;
    while b.time() < end {
        b.step();
        s += b.plant.theta_deg();
        n += 1;
    }
    Ok(PositionResult {
        final_deg: s / n as f64,
        settle_s: last_out,
        overshoot_deg: (peak - to).max(0.0),
    })
}

/// Время реакции защиты: стенд в режиме поддержки (90°, 2 кг), в момент t0
/// `inject` вносит неисправность; ждём выключения драйвера. Возвращает
/// (время, с; флаги телеметрии после неисправности; стенд).
fn reaction(
    cfg: &ExoConfig,
    dt: f64,
    max_s: f64,
    inject: impl FnOnce(&mut Bench),
) -> Result<(f64, u8, Bench), String> {
    let mut b = exo_bench(cfg, 90.0, true, 2.0)?;
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    b.run(0.1);
    if !enabled(&b) {
        return Err("драйвер не включился до внесения неисправности".into());
    }
    // Сдвиг фазы относительно тика 10 кГц, чтобы не попадать ровно на границу.
    b.dt = 37e-6;
    b.step();
    b.dt = 20e-6;
    let t_inj = b.time();
    inject(&mut b);
    let t = time_until(&mut b, dt, max_s, |b| !enabled(b))
        .ok_or_else(|| format!("драйвер не выключился за {max_s} с"))?;
    b.run(0.05);
    let flags = flags_since(&b, t_inj);
    Ok((t, flags, b))
}

pub fn exo_estop_reaction() -> Result<(f64, u8), String> {
    let (t, f, _) = reaction(&ExoConfig::default(), 1e-6, 0.01, |b| {
        b.mcu.set_gpio_input(exo::pins::GPIO_ESTOP, false)
    })?;
    Ok((t, f))
}

pub fn exo_sensor_reaction() -> Result<(f64, u8), String> {
    let (t, f, _) = reaction(&ExoConfig::default(), 1e-6, 0.01, |b| {
        b.plant.angle_volts_override = Some(0.1)
    })?;
    Ok((t, f))
}

/// Перегрузка по току: (время срабатывания, с; условия защёлки).
pub fn exo_overcurrent() -> Result<(f64, Conditions), String> {
    let (t, flags, mut b) = reaction(&ExoConfig::default(), 5e-6, 0.02, |b| {
        b.plant.current_sense_override_a = Some(14.0)
    })?;
    b.plant.current_sense_override_a = None;
    b.run(0.05);
    let still_off = !enabled(&b);
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    b.run(0.02);
    let off_after_mode1 = !enabled(&b);
    b.send(cmd(ExoMode::Off, 0, 0.0, 2.0));
    b.run(0.005);
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    b.run(0.02);
    let on_after_reset = enabled(&b);
    Ok((
        t,
        vec![
            (
                flags & fault::OVERCURRENT != 0,
                "флаг перегрузки в телеметрии",
            ),
            (still_off, "драйвер выключен после снятия перегрузки"),
            (off_after_mode1, "команда режима 1 не снимает защёлку"),
            (
                on_after_reset,
                "команда режима 0, затем 1 — драйвер снова включён",
            ),
        ],
    ))
}

/// Снижение тока по температуре: (ток при T, расчёт), А.
pub fn exo_temp_derate(temp_c: f64) -> Result<(f64, f64, u8), String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 90.0, true, 5.0)?;
    b.plant.temp_c = temp_c;
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 5.0));
    b.run(0.15);
    let i = mean_current(&mut b, 0.05);
    let k = ((cfg.temp_off_c as f64 - temp_c) / (cfg.temp_off_c - cfg.temp_derate_c) as f64)
        .clamp(0.0, 1.0);
    let expected = cfg
        .gravity_assist_current(100.0, 5.0, 90.0)
        .min(cfg.i_max_a as f64 * k);
    let st = b.last_telemetry().map_or(0, |t| t.state);
    Ok((i, expected, st))
}

pub fn exo_overtemp_off() -> Result<(f64, u8), String> {
    let (t, f, _) = reaction(&ExoConfig::default(), 10e-6, 0.01, |b| {
        b.plant.temp_c = 95.0
    })?;
    Ok((t, f))
}

pub fn exo_battery_low() -> Result<(f64, u8), String> {
    let (t, f, _) = reaction(&ExoConfig::default(), 10e-6, 0.05, |b| {
        b.plant.vbat_v = 2.9 * 6.0
    })?;
    Ok((t, f))
}

/// Предупреждение о батарее: условия.
pub fn exo_battery_warn() -> Result<Conditions, String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 90.0, true, 2.0)?;
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    b.run(0.1);
    let t = b.time();
    b.plant.vbat_v = 3.2 * 6.0;
    b.run(0.1);
    let f = flags_since(&b, t);
    Ok(vec![
        (f & fault::BATT_WARN != 0, "флаг «батарея низкая»"),
        (f & fault::BATT_LOW == 0, "нет флага «батарея разряжена»"),
        (enabled(&b), "драйвер остаётся включён"),
    ])
}

/// Потеря связи: время от команды до спада тока до 50 % и до < 0.05 А, с.
pub fn exo_cmd_timeout() -> Result<(f64, f64, u8), String> {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 90.0, true, 2.0)?;
    b.run(0.01);
    let t_cmd = b.time();
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    b.run(0.3);
    let i0 = mean_current(&mut b, 0.1);
    let half = time_until(&mut b, 10e-6, 0.5, |b| b.plant.current < 0.5 * i0)
        .ok_or("ток не спал до 50 %")?;
    let t_half = b.time() - t_cmd;
    let _ = half;
    time_until(&mut b, 10e-6, 0.5, |b| b.plant.current.abs() < 0.05)
        .ok_or("ток не спал до нуля")?;
    let t_zero = b.time() - t_cmd;
    let flags = flags_since(&b, t_cmd + 0.55);
    Ok((t_half, t_zero, flags))
}

/// Мягкий предел угла: угол скачком переводится за предел и удерживается.
/// Возвращает (время до исчезновения тока запрещённого знака, с; установившийся
/// ток, А; флаги). Запрещённый знак: `+` (сгибание) за максимумом, `−` (разгибание)
/// за минимумом. Скачок угла возбуждает D-составляющую — она даёт ток
/// *разрешённого* знака, который спадает с фильтром скорости (10 мс); поэтому
/// время реакции измеряется по току запрещённого знака, а установившийся ток —
/// через 50 мс.
pub fn exo_angle_limit(
    inside_deg: f64,
    beyond_deg: f64,
    target_deg: f64,
) -> Result<(f64, f64, u8), String> {
    // Проверяем сам мягкий предел: траектория и реакция на контакт здесь только мешают.
    let cfg = ExoConfig {
        speed_limit_dps: 0.0,
        contact_torque_nm: 0.0,
        ..ExoConfig::default()
    };
    let mut b = exo_bench(&cfg, inside_deg, true, 0.0)?;
    let c = cmd(ExoMode::PositionTrack, 0, target_deg, 0.0);
    b.send(c);
    b.run(0.1);
    let sign = if beyond_deg > inside_deg { 1.0 } else { -1.0 };
    if sign * b.plant.current < 0.5 {
        return Err(format!(
            "до предела ток запрещённого знака мал ({:.3} А) — сценарий не проверяет предел",
            b.plant.current
        ));
    }
    let t0 = b.time();
    b.plant.hold_deg = Some(beyond_deg);
    let t = time_until(&mut b, 10e-6, 0.05, |b| sign * b.plant.current < 0.05)
        .ok_or("ток запрещённого знака не исчез за 50 мс")?;
    b.send(c);
    b.run(0.05);
    let i = mean_current(&mut b, 0.05);
    Ok((t, i, flags_since(&b, t0)))
}

/// Сторож при зависании основного цикла: (время сброса от включения, с; условия).
pub fn exo_watchdog(hang_ms: u32) -> Result<(f64, Conditions), String> {
    let cfg = ExoConfig {
        test_hang_ms: hang_ms,
        ..ExoConfig::default()
    };
    let mut b = exo_bench(&cfg, 90.0, true, 2.0)?;
    let t0 = b.time();
    b.send(cmd(ExoMode::GravityAssist, 100, 0.0, 2.0));
    let mut t_reset = None;
    b.run_while(1.0, |b| {
        if b.mcu.watchdog_resets() > 0 {
            t_reset = Some(b.time());
            false
        } else {
            true
        }
    });
    let t_reset = t_reset.ok_or("сброса сторожем не было")? - t0;
    let off_right_after = !enabled(&b);
    let n_tl = b.telemetry.len();
    b.run(0.3);
    let after: Vec<&ExoTelemetry> = b.telemetry[n_tl..].iter().map(|(_, t)| t).collect();
    Ok((
        t_reset,
        vec![
            (
                b.mcu.reset_reason() == crate::ResetReason::Watchdog,
                "причина сброса — сторож",
            ),
            (
                b.mcu.watchdog_resets() == 1,
                "после перезапуска зависания нет (сброс один)",
            ),
            (off_right_after, "после сброса драйвер выключен"),
            (
                !after.is_empty() && after.iter().all(|t| t.state & state::AFTER_WATCHDOG != 0),
                "телеметрия после сброса: бит «после сторожа»",
            ),
            (
                !after.is_empty() && after.iter().all(|t| !t.driver_on()),
                "до новой команды драйвер не включается",
            ),
        ],
    ))
}

// ---------------------------------------------------------------------------
// Привод maxon с тепловой моделью: заклинивание и перегрев обмотки

/// Стенд с приводом maxon EC 90 flat + HD 50:1 и двухузловой тепловой моделью;
/// обмотка и корпус в начале при `t_start_c`, среда 25 °C, батарея 24 В, шаг
/// обмена 50 мкс (L/R = 1.27 мс).
pub fn maxon_bench(
    cfg: &ExoConfig,
    theta_deg: f64,
    hold: bool,
    payload_kg: f64,
    t_start_c: f64,
) -> Result<Bench, String> {
    let mut plant = ExoPlant::new(PlantParams::maxon_ec90flat_hd50(), theta_deg).with_thermal(
        PlantThermal::maxon_ec90flat(),
        t_start_c,
        t_start_c,
    );
    plant.payload_kg = payload_kg;
    plant.vbat_v = 24.0;
    if hold {
        plant.hold_deg = Some(theta_deg);
    }
    let mut b = Bench::new(firmware::exo_ctrl_elf(), cfg, plant).map_err(|e| e.to_string())?;
    b.dt = 50e-6;
    Ok(b)
}

/// Заклинивание сустава в позиционном режиме (цель 120°, сустав держится на 60°).
#[derive(Clone, Debug)]
pub struct StallResult {
    /// От момента, когда ток стенда достиг 0.85·i_max, до начала спада тока
    /// (EXO_DIAG.stall_state = 1), с.
    pub trip_s: f64,
    /// От начала спада до выключения драйвера, с.
    pub ramp_s: f64,
    /// Ток перед срабатыванием и в середине спада, А.
    pub current_before_a: f64,
    pub current_mid_a: f64,
    pub conditions: Conditions,
    /// Пик температуры обмотки стенда, °C.
    pub winding_peak_c: f64,
}

pub fn exo_stall() -> Result<StallResult, String> {
    // Защита от заклинивания — второй барьер: проверяем её отдельно, когда реакция на
    // контакт выключена и ток доходит до предела.
    let cfg = ExoConfig {
        speed_limit_dps: 0.0,
        contact_torque_nm: 0.0,
        ..ExoConfig::maxon_ec90flat_hd50()
    };
    let mut b = maxon_bench(&cfg, 60.0, true, 0.0, 25.0)?;
    let c = cmd(ExoMode::PositionTrack, 0, 120.0, 0.0);
    b.send(c);
    let thr = 0.85 * cfg.i_max_a as f64;
    time_until(&mut b, 20e-6, 0.1, |b| b.plant.current >= thr)
        .ok_or("ток не дошёл до 0.85·i_max")?;
    let t0 = b.time();
    b.dt = 20e-6;
    let mut next_cmd = t0 + 0.2;
    let mut peak = b.plant.t_winding_c;
    let stall_state = |b: &Bench| ExoDiag::read(&b.mcu).map_or(0, |d| d.stall_state);
    let mut current_before = b.plant.current;
    while stall_state(&b) == 0 {
        if b.time() - t0 > 3.0 {
            return Err("защита от заклинивания не сработала за 3 с".into());
        }
        if b.time() >= next_cmd {
            b.send(c);
            next_cmd += 0.2;
        }
        current_before = b.plant.current;
        b.step();
        peak = peak.max(b.plant.t_winding_c);
    }
    let t_trip = b.time();
    let mid = t_trip + 0.5 * cfg.stall_ramp_s as f64;
    let mut current_mid = f64::NAN;
    let mut on_during_ramp = true;
    while enabled(&b) {
        if b.time() - t_trip > 1.0 {
            return Err("после срабатывания драйвер не выключился за 1 с".into());
        }
        b.step();
        if current_mid.is_nan() && b.time() >= mid {
            current_mid = b.plant.current;
            on_during_ramp = enabled(&b);
        }
    }
    let ramp_s = b.time() - t_trip;
    b.dt = 50e-6;
    for _ in 0..3 {
        b.send(c);
        b.run(0.1);
    }
    let still_off = !enabled(&b);
    let f2 = flags2_since(&b, t_trip);
    b.send(cmd(ExoMode::Off, 0, 120.0, 0.0));
    b.run(0.005);
    b.send(c);
    b.run(0.05);
    let on_again = enabled(&b);
    Ok(StallResult {
        trip_s: t_trip - t0,
        ramp_s,
        current_before_a: current_before,
        current_mid_a: current_mid,
        conditions: vec![
            (
                f2 & fault::STALL != 0,
                "флаг STALL во втором байте телеметрии",
            ),
            (
                on_during_ramp,
                "во время спада драйвер включён (ток уменьшается, а не обрывается)",
            ),
            (still_off, "команды режима 2 не снимают защёлку (300 мс)"),
            (
                on_again,
                "команда режима 0, затем 2 — драйвер снова включён",
            ),
            (
                peak < 35.0,
                "обмотка за время до срабатывания нагрелась < 10 К",
            ),
        ],
        winding_peak_c: peak,
    })
}

/// Поддержка 100 % с грузом 25 кг при θ = 90° (нужно ≈ 9 А > i_max = 8.1 А):
/// ток у предела, человек держит руку неподвижно `seconds` секунд. Обмотка
/// и корпус в начале при `t_start_c`.
#[derive(Clone, Debug)]
pub struct AssistHoldResult {
    pub conditions: Conditions,
    /// Ток стенда в конце и предел по формуле вручную (по оценке обмотки), А.
    pub current_a: f64,
    pub limit_formula_a: f64,
    pub winding_est_c: f64,
    pub winding_true_c: f64,
}

pub fn exo_assist_saturated_hold(t_start_c: f64, seconds: f64) -> Result<AssistHoldResult, String> {
    let cfg = ExoConfig::maxon_ec90flat_hd50();
    let mut b = maxon_bench(&cfg, 90.0, true, 25.0, t_start_c)?;
    let c = cmd(ExoMode::GravityAssist, 100, 0.0, 25.0);
    b.send(c);
    b.run(0.02);
    let t0 = b.time();
    let (mut off_ms, mut stall_seen) = (0u32, false);
    let mut k = 0;
    while b.time() - t0 < seconds {
        if k % 200 == 0 {
            b.send(c);
        }
        for _ in 0..20 {
            b.step(); // 1 мс
        }
        k += 1;
        if !enabled(&b) {
            off_ms += 1;
        }
        if ExoDiag::read(&b.mcu).is_some_and(|d| d.stall_state != 0) {
            stall_seen = true;
        }
    }
    let d = ExoDiag::read(&b.mcu).ok_or("нет EXO_DIAG")?;
    // Задание без ограничения i_max (формула вручную).
    let need = ExoConfig {
        i_max_a: f32::MAX,
        ..cfg
    }
    .gravity_assist_current(100.0, 25.0, 90.0);
    let f2 = flags2_since(&b, t0);
    let t_max = cfg.t_winding_max_c as f64;
    let k_t = ((cfg.temp_off_c as f64 - d.housing_c) / (cfg.temp_off_c - cfg.temp_derate_c) as f64)
        .clamp(0.0, 1.0);
    let limit = cfg
        .winding_current_limit(d.winding_est_c)
        .min(cfg.i_max_a as f64 * k_t);
    Ok(AssistHoldResult {
        conditions: vec![
            (
                need > cfg.i_max_a as f64,
                "задание поддержки выше i_max (ток у предела)",
            ),
            (off_ms == 0, "драйвер включён всё время"),
            (
                !stall_seen && f2 & fault::STALL == 0,
                "защита от заклинивания не срабатывала",
            ),
            (
                d.winding_est_c > t_max - 25.0 && f2 & fault::WINDING_HOT != 0,
                "дошли до полосы снижения по обмотке (флаг WINDING_HOT)",
            ),
        ],
        current_a: b.plant.current,
        limit_formula_a: limit,
        winding_est_c: d.winding_est_c,
        winding_true_c: b.plant.t_winding_c,
    })
}

/// Оценка температуры обмотки прошивкой против тепловой модели стенда.
#[derive(Clone, Copy, Debug)]
pub struct WindingEstimateResult {
    pub firmware_c: f64,
    pub host_c: f64,
    pub max_dev_k: f64,
    pub housing_c: f64,
    pub current_a: f64,
}

/// Тяжёлое удержание без заклинивания: поддержка 100 %, груз 16 кг, θ = 90°
/// (ток ≈ 5.9 А < 0.85·8.1 А), `seconds` секунд от 25 °C.
pub fn exo_winding_estimate(seconds: f64) -> Result<WindingEstimateResult, String> {
    let cfg = ExoConfig::maxon_ec90flat_hd50();
    let mut b = maxon_bench(&cfg, 90.0, true, 16.0, 25.0)?;
    let c = cmd(ExoMode::GravityAssist, 100, 0.0, 16.0);
    let mut max_dev: f64 = 0.0;
    let steps = (seconds / 0.1).round() as usize;
    let mut last = None;
    for k in 0..steps {
        if k % 2 == 0 {
            b.send(c);
        }
        b.run(0.1);
        let d = ExoDiag::read(&b.mcu).ok_or("нет диагностического блока EXO_DIAG")?;
        max_dev = max_dev.max((d.winding_est_c - b.plant.t_winding_c).abs());
        last = Some(d);
    }
    let d = last.ok_or("пустой прогон")?;
    if !enabled(&b) {
        return Err("драйвер выключился — сценарий не про оценку".into());
    }
    Ok(WindingEstimateResult {
        firmware_c: d.winding_est_c,
        host_c: b.plant.t_winding_c,
        max_dev_k: max_dev,
        housing_c: b.plant.t_housing_c,
        current_a: b.plant.current,
    })
}

/// Заклинивание, но оператор каждый раз сразу перезапускает привод (режим 0 → 2):
/// защита от заклинивания срабатывает раз за разом, обмотку бережёт только
/// тепловая модель.
#[derive(Clone, Copy, Debug)]
pub struct JamResult {
    /// Пик оценки обмотки, пока драйвер включён, °C.
    pub est_peak_enabled_c: f64,
    /// Пик истинной (стенд) температуры обмотки, °C.
    pub true_peak_c: f64,
    pub housing_end_c: f64,
    /// Ток стенда во включённом состоянии и предел по формуле вручную, А (последний замер).
    pub current_a: f64,
    pub limit_formula_a: f64,
    pub restarts: u32,
    /// Доля времени с включённым драйвером.
    pub duty: f64,
}

pub fn exo_jam_restarts(t_start_c: f64, seconds: f64) -> Result<JamResult, String> {
    // Худший случай для обмотки: реакция на контакт выключена, ток идёт до предела,
    // поэтому проверяется сама тепловая защита как отдельный барьер.
    let cfg = ExoConfig {
        speed_limit_dps: 0.0,
        contact_torque_nm: 0.0,
        ..ExoConfig::maxon_ec90flat_hd50()
    };
    let mut b = maxon_bench(&cfg, 60.0, true, 0.0, t_start_c)?;
    let c = cmd(ExoMode::PositionTrack, 0, 120.0, 0.0);
    b.send(c);
    let t0 = b.time();
    let (mut est_peak, mut true_peak) = (f64::MIN, f64::MIN);
    let mut restarts = 0;
    let mut sample = None;
    let mut on_since: Option<f64> = None;
    let (mut on_time, mut total) = (0.0, 0.0);
    let mut next_ctl = t0 + 0.1;
    while b.time() - t0 < seconds {
        for _ in 0..20 {
            b.step(); // 1 мс
        }
        total += 1e-3;
        true_peak = true_peak.max(b.plant.t_winding_c);
        let on = enabled(&b);
        if on {
            on_time += 1e-3;
            if let Some(d) = ExoDiag::read(&b.mcu) {
                est_peak = est_peak.max(d.winding_est_c);
                if d.stall_state == 0 && on_since.is_some_and(|t| b.time() - t > 0.5) {
                    let k_t = ((cfg.temp_off_c as f64 - d.housing_c)
                        / (cfg.temp_off_c - cfg.temp_derate_c) as f64)
                        .clamp(0.0, 1.0);
                    let lim = cfg
                        .winding_current_limit(d.winding_est_c)
                        .min(cfg.i_max_a as f64 * k_t);
                    sample = Some((b.plant.current, lim));
                }
            }
            on_since.get_or_insert(b.time());
        } else {
            on_since = None;
        }
        if b.time() >= next_ctl {
            next_ctl += 0.1;
            if on {
                b.send(c);
            } else {
                b.send(cmd(ExoMode::Off, 0, 120.0, 0.0));
                b.run(0.002);
                b.send(c);
                restarts += 1;
            }
        }
    }
    let (current_a, limit_formula_a) =
        sample.ok_or("не было включённого установившегося участка")?;
    Ok(JamResult {
        est_peak_enabled_c: est_peak,
        true_peak_c: true_peak,
        housing_end_c: b.plant.t_housing_c,
        current_a,
        limit_formula_a,
        restarts,
        duty: on_time / total,
    })
}

fn thermal_checks(out: &mut Vec<Check>) {
    let cfg = ExoConfig::maxon_ec90flat_hd50();

    const T_ST: &str = "exo-ctrl (maxon EC 90 flat + HD 50:1): заклинивание сустава → выключение";
    const T_STL: &str = "exo-ctrl: защёлка заклинивания";
    const T_SR: &str = "exo-ctrl: заклинивание — время спада тока до выключения драйвера";
    const T_SM: &str = "exo-ctrl: заклинивание — ток в середине спада";
    match exo_stall() {
        Ok(r) => {
            out.push(mk(
                "mcu.exo.stall_trip",
                T_ST,
                r.trip_s,
                cfg.stall_time_s as f64,
                "с",
                "стенд: сустав удерживается, цель позиции на 60° дальше; от тока 0.85·i_max до начала спада (EXO_DIAG)",
                "настройка stall_time_s = 1.5 с",
                Tolerance::Absolute(0.005),
                &format!(
                    "счёт ведёт внешний контур 1 кГц по среднему за 1 мс току; пик обмотки {:.1} °C",
                    r.winding_peak_c
                ),
            ));
            out.push(mk(
                "mcu.exo.stall_ramp_time",
                T_SR,
                r.ramp_s,
                cfg.stall_ramp_s as f64,
                "с",
                "стенд: от начала спада до низкого уровня на GPIO2",
                "настройка stall_ramp_s = 0.2 с",
                Tolerance::Absolute(0.005),
                "",
            ));
            out.push(mk(
                "mcu.exo.stall_ramp_mid",
                T_SM,
                r.current_mid_a,
                0.5 * r.current_before_a,
                "А",
                "стенд: ток мотора через stall_ramp_s/2 после начала спада",
                "линейный спад: половина тока перед срабатыванием",
                Tolerance::Absolute(0.15),
                &format!("ток перед срабатыванием {:.3} А", r.current_before_a),
            ));
            out.push(conditions(
                "mcu.exo.stall_latch",
                T_STL,
                &r.conditions,
                "стенд: команды режимов после срабатывания",
                "требование: спад, затем защёлка до команды режима 0",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.stall_trip", T_ST, &e));
            out.push(broken("mcu.exo.stall_ramp_time", T_SR, &e));
            out.push(broken("mcu.exo.stall_ramp_mid", T_SM, &e));
            out.push(broken("mcu.exo.stall_latch", T_STL, &e));
        }
    }

    const T_AH: &str = "exo-ctrl: поддержка у предела тока, рука неподвижна 30 с (груз 25 кг, от 65 °C) — без ложного «заклинивания»";
    const T_AHC: &str = "exo-ctrl: тот же прогон — ток в конце против предела по обмотке";
    match exo_assist_saturated_hold(65.0, 30.0) {
        Ok(r) => {
            out.push(conditions(
                "mcu.exo.assist_hold_no_stall",
                T_AH,
                &r.conditions,
                "стенд: режим 1, угол удерживается, команды каждые 200 мс",
                "требование: неподвижное удержание у предела в режиме поддержки — не заклинивание",
            ));
            out.push(mk(
                "mcu.exo.assist_hold_current",
                T_AHC,
                r.current_a,
                r.limit_formula_a,
                "А",
                "стенд: ток мотора в конце прогона",
                "вручную: i_max·(T_max − T_оценка)/25 — только снижение по обмотке",
                Tolerance::Absolute(0.05),
                &format!(
                    "оценка обмотки {:.1} °C, стенд {:.1} °C",
                    r.winding_est_c, r.winding_true_c
                ),
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.assist_hold_no_stall", T_AH, &e));
            out.push(broken("mcu.exo.assist_hold_current", T_AHC, &e));
        }
    }

    const T_WE: &str = "exo-ctrl: оценка температуры обмотки через 20 с удержания 5.9 А";
    const T_WD: &str = "exo-ctrl: наибольшее расхождение оценки обмотки за 20 с";
    match exo_winding_estimate(20.0) {
        Ok(r) => {
            let note = format!(
                "ток {:.2} А, корпус {:.2} °C; погрешности: квантование АЦП тока (8 мА) и корпуса (0.08 К), f32, шаг 1 мс",
                r.current_a, r.housing_c
            );
            out.push(mk(
                "mcu.exo.winding_estimate",
                T_WE,
                r.firmware_c,
                r.host_c,
                "°C",
                "прошивка: узел «обмотка», I² по 10 отсчётам АЦП, корпус — по ADC2, Эйлер 1 кГц в f32 (символ EXO_DIAG)",
                "стенд: двухузловая сеть «обмотка → корпус → среда» в f64 по истинному току (шаг 50 мкс)",
                Tolerance::Absolute(0.5),
                &note,
            ));
            out.push(mk(
                "mcu.exo.winding_estimate_maxdev",
                T_WD,
                r.max_dev_k,
                0.0,
                "К",
                "замеры каждые 100 мс",
                "тепловая модель стенда",
                Tolerance::Absolute(0.5),
                &note,
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.winding_estimate", T_WE, &e));
            out.push(broken("mcu.exo.winding_estimate_maxdev", T_WD, &e));
        }
    }

    let t_max = cfg.t_winding_max_c as f64;
    const T_JE: &str = "exo-ctrl: заклинивание + немедленные перезапуски, 60 с от 50 °C — пик ОЦЕНКИ обмотки при включённом драйвере";
    const T_JT: &str = "exo-ctrl: тот же прогон — пик ИСТИННОЙ температуры обмотки (стенд)";
    const T_JC: &str = "exo-ctrl: ток в полосе снижения по обмотке против формулы";
    match exo_jam_restarts(50.0, 60.0) {
        Ok(r) => {
            let note = format!(
                "перезапусков {}, драйвер включён {:.0} % времени, корпус в конце {:.1} °C",
                r.restarts,
                100.0 * r.duty,
                r.housing_end_c
            );
            out.push(range(
                "mcu.exo.jam_winding_estimate_peak",
                T_JE,
                r.est_peak_enabled_c,
                (t_max - 25.0, t_max + 5.0),
                "°C",
                "EXO_DIAG каждую 1 мс, пока драйвер включён",
                "требование: не выше T_max + 5 °C; нижняя граница — сценарий дошёл до полосы снижения",
                &note,
            ));
            out.push(range(
                "mcu.exo.jam_winding_true_peak",
                T_JT,
                r.true_peak_c,
                (t_max - 25.0, t_max + 5.0),
                "°C",
                "тепловая модель стенда",
                "T_max обмотки maxon 125 °C",
                &note,
            ));
            out.push(mk(
                "mcu.exo.winding_derate_current",
                T_JC,
                r.current_a,
                r.limit_formula_a,
                "А",
                "стенд: ток мотора через 0.5 с после включения (сустав заклинен — ток у предела)",
                "вручную: i_max·(T_max − T_оценка)/25 (и снижение по корпусу, если есть)",
                Tolerance::Absolute(0.05),
                "",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.jam_winding_estimate_peak", T_JE, &e));
            out.push(broken("mcu.exo.jam_winding_true_peak", T_JT, &e));
            out.push(broken("mcu.exo.winding_derate_current", T_JC, &e));
        }
    }
}

fn exo_checks(out: &mut Vec<Check>) {
    let cfg = ExoConfig::default();

    const T_TICK: &str = "exo-ctrl: прерываний таймера за 1 с модели";
    const T_TLM: &str = "exo-ctrl: пакетов телеметрии за 1 с";
    match exo_rates() {
        Ok(r) => {
            out.push(mk(
                "mcu.exo.tick_rate",
                T_TICK,
                r.ticks_per_s,
                10_000.0,
                "шт.",
                "счётчик взятых прерываний ядра, режим поддержки",
                "настройка прошивки: mtimecmp += 1600 тиков 16 МГц",
                Tolerance::Absolute(1.0),
                &format!(
                    "загрузка ЦП при 160 МГц: {:.1} % («оценка» по модели времени), {:.2} млн инструкций/с",
                    100.0 * r.cpu_load,
                    r.instructions_per_s / 1e6
                ),
            ));
            out.push(mk(
                "mcu.exo.telemetry_rate",
                T_TLM,
                r.telemetry_per_s,
                50.0,
                "шт.",
                "пакеты 0x7E из очереди передачи радио",
                "протокол: телеметрия каждые 20 мс",
                Tolerance::Absolute(1.0),
                "",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.tick_rate", T_TICK, &e));
            out.push(broken("mcu.exo.telemetry_rate", T_TLM, &e));
        }
    }

    let ids: [(&'static str, f64); 4] = [
        ("mcu.exo.assist_current_30", 30.0),
        ("mcu.exo.assist_current_60", 60.0),
        ("mcu.exo.assist_current_90", 90.0),
        ("mcu.exo.assist_current_120", 120.0),
    ];
    for (id, th) in ids {
        let title = format!("exo-ctrl: ток гравитационной поддержки 100 %, груз 2 кг, θ = {th}°");
        match exo_assist_current(th, 2.0) {
            Ok((i, e)) => out.push(mk(
                id,
                &title,
                i,
                e,
                "А",
                "прошивка в замкнутом контуре со стендом (угол удерживается), средний ток мотора за 50 мс",
                "вручную: i = (m_f·g·r_c + m·g·L)·sin θ / (Kt·N·η)",
                Tolerance::Absolute(0.02),
                "допуск ≈ 2.5 младших разряда АЦП тока (8 мА); вклад квантования угла и sin в f32 < 0.1 %",
            )),
            Err(e) => out.push(broken(id, &title, &e)),
        }
    }

    const T_POS: &str = "exo-ctrl: позиционный режим 30° → 90°, угол через 2 с";
    const T_SET: &str = "exo-ctrl: время установления позиции в ±1°";
    match exo_position_step(30.0, 90.0) {
        Ok(p) => {
            out.push(mk(
                "mcu.exo.position_final",
                T_POS,
                p.final_deg,
                90.0,
                "°",
                "прошивка (ПИД + компенсация тяжести) со стендом: J ≈ 0.1 кг·м², вязкое трение 0.05 Н·м·с/рад",
                "задание",
                Tolerance::Absolute(0.5),
                &format!(
                    "перерегулирование {:.2}°; разрешение датчика угла ≈ 0.045° (12 бит)",
                    p.overshoot_deg
                ),
            ));
            out.push(range(
                "mcu.exo.position_settle",
                T_SET,
                p.settle_s,
                (0.0, 1.0),
                "с",
                "последний выход за ±1° от задания",
                "требование к регулятору (заявлено в MEMORY_MAP.md)",
                "",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.position_final", T_POS, &e));
            out.push(broken("mcu.exo.position_settle", T_SET, &e));
        }
    }

    const T_SPEED: &str = "exo-ctrl: рука не быстрее предела скорости 40 °/с";
    const T_SPEED_END: &str = "exo-ctrl: с пределом скорости рука доходит до цели 90°";
    match exo_speed_limited_move(30.0, 90.0) {
        Ok((peak, end)) => {
            out.push(range(
                "mcu.exo.speed_limit",
                T_SPEED,
                peak,
                (30.0, 44.0),
                "°/с",
                "наибольшая скорость руки на стенде, окно 50 мс",
                "предел из конфигурации + 10% на слежение",
                "нижняя граница: рука действительно движется",
            ));
            out.push(mk(
                "mcu.exo.speed_limit_final",
                T_SPEED_END,
                end,
                90.0,
                "°",
                "прошивка с траекторией",
                "задание",
                Tolerance::Absolute(1.0),
                "",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.speed_limit", T_SPEED, &e));
            out.push(broken("mcu.exo.speed_limit_final", T_SPEED_END, &e));
        }
    }

    const T_ESTOP: &str = "exo-ctrl: аварийная кнопка → драйвер выключен";
    match exo_estop_reaction() {
        Ok((t, f)) => out.push(range(
            "mcu.exo.estop_reaction",
            T_ESTOP,
            t * 1e6,
            (0.0, 110.0),
            "мкс",
            "шаг стенда 1 мкс: от низкого уровня на GPIO0 до низкого уровня на GPIO2",
            "граница: один тик 10 кГц (100 мкс) + исполнение обработчика",
            &format!("флаг ESTOP в телеметрии: {}", f & fault::ESTOP != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.estop_reaction", T_ESTOP, &e)),
    }

    const T_SENS: &str = "exo-ctrl: обрыв датчика угла (0.1 В) → драйвер выключен";
    match exo_sensor_reaction() {
        Ok((t, f)) => out.push(range(
            "mcu.exo.sensor_reaction",
            T_SENS,
            t * 1e6,
            (0.0, 110.0),
            "мкс",
            "шаг стенда 1 мкс",
            "граница: один тик 10 кГц + исполнение обработчика",
            &format!("флаг SENSOR в телеметрии: {}", f & fault::SENSOR != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.sensor_reaction", T_SENS, &e)),
    }

    const T_OC: &str = "exo-ctrl: перегрузка по току (датчик показывает 14 А > 12 А) → выключение";
    const T_OCL: &str = "exo-ctrl: защёлка перегрузки по току";
    match exo_overcurrent() {
        Ok((t, conds)) => {
            out.push(range(
                "mcu.exo.overcurrent_trip",
                T_OC,
                t * 1e3,
                (2.0, 2.2),
                "мс",
                "шаг стенда 5 мкс",
                "требование: |I| > i_trip дольше 2 мс; отсчёты каждые 100 мкс → (2.0; 2.1] мс",
                "",
            ));
            out.push(conditions(
                "mcu.exo.overcurrent_latch",
                T_OCL,
                &conds,
                "стенд: снятие перегрузки, команды режимов",
                "требование: защёлка до команды режима 0",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.overcurrent_trip", T_OC, &e));
            out.push(broken("mcu.exo.overcurrent_latch", T_OCL, &e));
        }
    }

    const T_DER: &str = "exo-ctrl: снижение предела тока при 80 °C (груз 5 кг, нужно 4.94 А)";
    match exo_temp_derate(80.0) {
        Ok((i, e, st)) => out.push(mk(
            "mcu.exo.temp_derate",
            T_DER,
            i,
            e,
            "А",
            "стенд, средний ток мотора за 50 мс",
            "вручную: i_max·(T_off − T)/(T_off − T_derate) = 8·10/20 = 4 А",
            Tolerance::Absolute(0.05),
            &format!(
                "квантование АЦП температуры: 80 °C читается как 79.95 °C (+0.02 А); бит «снижение» в телеметрии: {}",
                st & state::DERATING != 0
            ),
        )),
        Err(e) => out.push(broken("mcu.exo.temp_derate", T_DER, &e)),
    }

    const T_OT: &str = "exo-ctrl: перегрев 95 °C > 90 °C → драйвер выключен";
    match exo_overtemp_off() {
        Ok((t, f)) => out.push(range(
            "mcu.exo.overtemp_off",
            T_OT,
            t * 1e3,
            (0.0, 1.2),
            "мс",
            "шаг стенда 10 мкс",
            "граница: внешний контур 1 кГц + один тик",
            &format!("флаг OVERTEMP: {}", f & fault::OVERTEMP != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.overtemp_off", T_OT, &e)),
    }

    const T_BL: &str = "exo-ctrl: батарея 2.9 В/элемент → драйвер выключен";
    match exo_battery_low() {
        Ok((t, f)) => out.push(range(
            "mcu.exo.battery_low",
            T_BL,
            t * 1e3,
            (9.0, 10.2),
            "мс",
            "шаг стенда 10 мкс",
            "требование: 10 отсчётов 1 кГц подряд ниже 3.0 В/элемент (фильтр просадок) → (9.1; 10.1] мс",
            &format!("флаг BATT_LOW: {}", f & fault::BATT_LOW != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.battery_low", T_BL, &e)),
    }

    const T_BW: &str = "exo-ctrl: батарея 3.2 В/элемент → только предупреждение";
    match exo_battery_warn() {
        Ok(c) => out.push(conditions(
            "mcu.exo.battery_warn",
            T_BW,
            &c,
            "стенд, телеметрия за 100 мс",
            "требование: < 3.3 В/элемент — флаг, драйвер работает",
        )),
        Err(e) => out.push(broken("mcu.exo.battery_warn", T_BW, &e)),
    }

    const T_TO50: &str = "exo-ctrl: нет команд — ток поддержки спал до 50 %";
    const T_TO0: &str = "exo-ctrl: нет команд — ток поддержки спал до нуля (< 0.05 А)";
    match exo_cmd_timeout() {
        Ok((th, tz, f)) => {
            out.push(mk(
                "mcu.exo.cmd_timeout_half",
                T_TO50,
                th,
                0.600,
                "с",
                "стенд, время от последней команды, шаг 10 мкс",
                "требование: 500 мс тишины, затем линейный спад за 200 мс → 50 % через 600 мс",
                Tolerance::Absolute(0.003),
                &format!("флаг CMD_TIMEOUT: {}", f & fault::CMD_TIMEOUT != 0),
            ));
            out.push(mk(
                "mcu.exo.cmd_timeout_zero",
                T_TO0,
                tz,
                0.696,
                "с",
                "стенд, время от последней команды",
                "требование: спад до 0 к 700 мс; 0.05 А из 2.36 А — это 2.1 % → 696 мс",
                Tolerance::Absolute(0.005),
                "",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.cmd_timeout_half", T_TO50, &e));
            out.push(broken("mcu.exo.cmd_timeout_zero", T_TO0, &e));
        }
    }

    const T_AMAX: &str = "exo-ctrl: угол 145° > 140° (цель 150°) — ток сгибания отсутствует";
    const T_AMIN: &str = "exo-ctrl: угол 2° < 5° (цель 0°) — ток разгибания отсутствует";
    const T_AREACT: &str =
        "exo-ctrl: реакция мягкого предела угла (ток запрещённого знака < 0.05 А)";
    let amax = exo_angle_limit(130.0, 145.0, 150.0);
    let amin = exo_angle_limit(15.0, 2.0, 0.0);
    match &amax {
        Ok((_, i, f)) => out.push(mk(
            "mcu.exo.angle_max_current",
            T_AMAX,
            *i,
            0.0,
            "А",
            "стенд: угол удерживается за пределом, средний ток за 50 мс (через 50 мс после скачка)",
            "требование: за angle_max момент сгибания запрещён; цель за пределом — ПИД просит сгибание",
            Tolerance::Absolute(0.02),
            &format!("флаг ANGLE_LIMIT: {}", f & fault::ANGLE_LIMIT != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.angle_max_current", T_AMAX, e)),
    }
    match &amin {
        Ok((_, i, f)) => out.push(mk(
            "mcu.exo.angle_min_current",
            T_AMIN,
            *i,
            0.0,
            "А",
            "стенд: угол удерживается за пределом, средний ток за 50 мс (через 50 мс после скачка)",
            "требование: до angle_min момент разгибания запрещён; цель за пределом — ПИД просит разгибание",
            Tolerance::Absolute(0.02),
            &format!("флаг ANGLE_LIMIT: {}", f & fault::ANGLE_LIMIT != 0),
        )),
        Err(e) => out.push(broken("mcu.exo.angle_min_current", T_AMIN, e)),
    }
    match (&amax, &amin) {
        (Ok((a, _, _)), Ok((b, _, _))) => out.push(range(
            "mcu.exo.angle_limit_reaction",
            T_AREACT,
            a.max(*b) * 1e3,
            (0.0, 3.0),
            "мс",
            "стенд: скачок угла за предел, худшее из двух пределов, шаг 10 мкс",
            "граница: внешний контур 1 кГц + спад тока токовым контуром (полоса ≈ 700 Гц)",
            &format!("max: {:.3} мс, min: {:.3} мс", a * 1e3, b * 1e3),
        )),
        (Err(e), _) | (_, Err(e)) => out.push(broken("mcu.exo.angle_limit_reaction", T_AREACT, e)),
    }

    const T_WD: &str = "exo-ctrl: зависание основного цикла в 300 мс → сброс сторожем";
    const T_WDC: &str = "exo-ctrl: состояние после сброса сторожем";
    match exo_watchdog(300) {
        Ok((t, c)) => {
            out.push(mk(
                "mcu.exo.watchdog_reset",
                T_WD,
                t,
                0.300 + cfg.wdt_timeout_ms as f64 * 1e-3,
                "с",
                "стенд; тестовый флаг конфигурации test_hang_ms = 300",
                "последнее «кормление» ≈ 300 мс + тайм-аут сторожа 50 мс",
                Tolerance::Absolute(0.0005),
                "",
            ));
            out.push(conditions(
                "mcu.exo.watchdog_after",
                T_WDC,
                &c,
                "стенд, 300 мс после сброса",
                "требование: безопасный перезапуск",
            ));
        }
        Err(e) => {
            out.push(broken("mcu.exo.watchdog_reset", T_WD, &e));
            out.push(broken("mcu.exo.watchdog_after", T_WDC, &e));
        }
    }
}

/// Все сверки блока.
pub fn all() -> Vec<Check> {
    let mut out = Vec::new();
    selftest_checks(&mut out);
    decoder_check(&mut out);
    exo_checks(&mut out);
    thermal_checks(&mut out);
    out
}
