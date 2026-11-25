//! Запуск ngspice (внешняя программа, лицензия BSD-3-Clause для основной части) в пакетном
//! режиме для перекрёстной сверки. Код ngspice в наш не переносится: мы передаём нетлист на
//! стандартный вход `ngspice -b` и разбираем напечатанные таблицы. Временных файлов нет.
//!
//! Где искать программу: переменная окружения `NGSPICE`, затем
//! `assets/third_party/ngspice/bin/ngspice` относительно каталога `sim/` (от исходников крейта
//! и от текущего каталога).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use crate::Circuit;
use crate::circuit::Kind;
use crate::mna::validate_circuit;
use crate::netlist::{ElementKind, ModelKind, Netlist, Waveform};

/// Найти исполняемый файл ngspice.
pub fn find_ngspice() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("NGSPICE") {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    let rel = Path::new("assets/third_party/ngspice/bin/ngspice");
    let candidates = [
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(rel),
        rel.to_path_buf(),
        Path::new("..").join(rel),
        Path::new("../..").join(rel),
    ];
    candidates.into_iter().find(|p| p.is_file())
}

/// Таблица результатов ngspice: имена столбцов и значения.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NgTable {
    pub columns: Vec<String>,
    pub rows: Vec<Vec<f64>>,
}

impl NgTable {
    pub fn column(&self, name: &str) -> Option<Vec<f64>> {
        let name = name.to_lowercase();
        let k = self.columns.iter().position(|c| *c == name)?;
        Some(self.rows.iter().map(|r| r[k]).collect())
    }
}

/// Анализ для запуска.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NgAnalysis {
    Op,
    /// tstep, tstop, начало вывода, наибольший шаг, uic.
    Tran {
        tstep: f64,
        tstop: f64,
        tstart: f64,
        tmax: f64,
        uic: bool,
    },
}

/// Команда в расписании независимой сверки. Метка задана в секундах.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NgEvent<'a> {
    pub time: f64,
    pub element: &'a str,
    pub value: NgEventValue,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NgEventValue {
    /// Скачок сразу после указанной метки, как у `Transient::set_source_step`.
    SourceStep(f64),
    /// Линейный переход от предыдущей команды этого источника (либо t = 0).
    SourceRamp(f64),
    /// Удержание состояния EXT-ключа справа от метки.
    Switch(bool),
}

/// Подготовленная схема с сохранёнными именами узлов и адресами токов источников.
pub struct NgDeck {
    text: String,
    source_currents: BTreeMap<String, String>,
    events: Vec<(f64, bool)>,
}

impl NgDeck {
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Вектор тока независимого источника, со знаком от первой клеммы ко второй.
    pub fn source_current(&self, element: &str) -> Option<&str> {
        self.source_currents
            .get(&element.to_lowercase())
            .map(String::as_str)
    }

    /// Сверить сформированную схему и проверить, что таблица разрешила метки событий.
    pub fn run(
        &self,
        ngspice: &Path,
        analysis: NgAnalysis,
        signals: &[&str],
    ) -> Result<NgTable, String> {
        let table = run(ngspice, &self.text, analysis, signals)?;
        if let NgAnalysis::Tran { tstart, tstop, .. } = analysis {
            let time = table.column("time").ok_or("нет временной шкалы ngspice")?;
            for (i, &(event, step)) in self.events.iter().enumerate() {
                if event < tstart
                    || event > tstop
                    || (event > 0.0 && !time.iter().any(|&t| same_time(t, event)))
                {
                    return Err(format!("ngspice: отсутствует метка события {event:e} с"));
                }
                if step {
                    let end = self.events.get(i + 1).map_or(tstop, |&(t, _)| t);
                    if !time
                        .iter()
                        .any(|&t| t > event && !same_time(t, event) && t <= end)
                    {
                        return Err(format!(
                            "ngspice: отсутствуют отсчёты после скачка на {event:e} с"
                        ));
                    }
                }
            }
        }
        Ok(table)
    }
}

/// Экспортировать раскрытую схему и команды хоста. У скачков есть точная метка
/// излома и отдельные значения слева и справа. Порядок узлов элементов сохраняется.
pub fn prepare_netlist_with_events(
    netlist: &Netlist,
    events: &[NgEvent<'_>],
) -> Result<NgDeck, String> {
    use ElementKind::*;
    let mut names = BTreeSet::new();
    let mut elements = BTreeMap::new();
    for e in &netlist.elements {
        let (prefix, pins) = match &e.kind {
            Resistor { .. } => ('r', 2),
            Capacitor { .. } => ('c', 2),
            Inductor { .. } => ('l', 2),
            VoltageSource(_) => ('v', 2),
            CurrentSource(_) => ('i', 2),
            Diode { .. } => ('d', 2),
            Switch {
                control: crate::netlist::SwitchControl::External,
                ..
            } => ('s', 2),
            Switch { .. } => ('s', 4),
            Mosfet { .. } => ('m', 4),
            Vcvs { .. } => ('e', 4),
            Vccs { .. } => ('g', 4),
        };
        if e.nodes.len() != pins || !identifier(&e.name) || e.nodes.iter().any(|n| !identifier(n)) {
            return Err(format!("недопустимое имя или выводы элемента «{}»", e.name));
        }
        let lower = e.name.to_lowercase();
        let name = if lower.starts_with(prefix) && !lower.contains('.') {
            lower.clone()
        } else {
            format!("{prefix}.{lower}")
        };
        if elements.insert(lower, name.clone()).is_some() {
            return Err(format!("повтор элемента «{}»", e.name));
        }
        names.extend(e.nodes.iter().map(|n| n.to_lowercase()));
    }
    let mut used_elements = BTreeSet::new();
    for name in elements.values() {
        if !used_elements.insert(name.clone()) {
            return Err(format!("имя «{name}» неоднозначно после раскрытия"));
        }
    }
    names.extend(used_elements);
    for model in &netlist.models {
        if !identifier(&model.name)
            || model
                .params
                .iter()
                .any(|(k, v)| !identifier(k) || !v.is_finite())
        {
            return Err(format!("недопустимые параметры модели «{}»", model.name));
        }
        names.insert(model.name.to_lowercase());
    }
    let circuit = Circuit::from_netlist(netlist).map_err(|e| e.to_string())?;
    validate_circuit(&circuit).map_err(|e| e.to_string())?;
    let mut schedules: BTreeMap<String, Vec<NgEvent<'_>>> = BTreeMap::new();
    for &event in events {
        if !event.time.is_finite() || event.time < 0.0 {
            return Err(format!("недопустимая метка события «{}»", event.element));
        }
        let element = circuit
            .find_elem(event.element)
            .map_err(|e| e.to_string())?;
        let valid = match (&element.kind, event.value) {
            (
                Kind::Vsrc { src, .. } | Kind::Isrc { src, .. },
                NgEventValue::SourceStep(v) | NgEventValue::SourceRamp(v),
            ) => src.ext.is_some() && v.is_finite(),
            (Kind::Switch(s), NgEventValue::Switch(_)) => s.ctrl.is_none(),
            _ => false,
        };
        if !valid {
            return Err(format!("недопустимая команда элемента «{}»", event.element));
        }
        let schedule = schedules.entry(element.name.clone()).or_default();
        let start = schedule.last().map_or(0.0, |prev| prev.time);
        if (!schedule.is_empty() || matches!(event.value, NgEventValue::SourceRamp(_)))
            && event.time <= start
        {
            return Err(format!(
                "метки команд «{}» должны возрастать",
                event.element
            ));
        }
        schedule.push(event);
    }

    let mut text = netlist
        .title
        .lines()
        .next()
        .filter(|s| !s.is_empty())
        .unwrap_or("Схема")
        .to_string();
    text.push('\n');
    let mut source_currents = BTreeMap::new();
    let mut serial = 0;
    let mut new_name = |prefix: &str| loop {
        serial += 1;
        let name = format!("{prefix}sim_event_{serial}");
        if names.insert(name.clone()) {
            break name;
        }
    };
    for (e, resolved) in netlist.elements.iter().zip(&circuit.elems) {
        let name = &elements[&e.name.to_lowercase()];
        let nodes = e.nodes.join(" ");
        let events = schedules.get(&e.name).map_or(&[][..], Vec::as_slice);
        match &e.kind {
            VoltageSource(src) | CurrentSource(src) => {
                let voltage = matches!(e.kind, VoltageSource(_));
                let sense = if voltage { name.clone() } else { new_name("v") };
                source_currents.insert(e.name.to_lowercase(), format!("i({sense})"));
                let (head, pins) = if voltage && events.is_empty() {
                    (name.clone(), nodes)
                } else {
                    let inner = new_name("n");
                    writeln!(text, "{sense} {} {inner} 0", e.nodes[0]).unwrap();
                    (
                        if events.is_empty() {
                            name.clone()
                        } else {
                            new_name("b")
                        },
                        format!("{inner} {}", e.nodes[1]),
                    )
                };
                if !events.is_empty() {
                    let Some(Waveform::Ext { initial }) = src.wave else {
                        unreachable!()
                    };
                    let expr = event_expression(initial, events);
                    writeln!(
                        text,
                        "{head} {pins} {}= {expr}",
                        if voltage { "V" } else { "I" }
                    )
                    .unwrap();
                } else {
                    write!(text, "{head} {pins} ").unwrap();
                    source_text(&mut text, src, netlist)?;
                    text.push('\n');
                }
            }
            Switch { .. } => {
                let Kind::Switch(s) = &resolved.kind else {
                    unreachable!()
                };
                let model = new_name("m");
                let state = if s.initially_on { "ON" } else { "OFF" };
                let (control, vt, vh) = if s.ctrl.is_none() {
                    let ctrl = new_name("n");
                    let init = f64::from(s.initially_on);
                    if events.is_empty() {
                        writeln!(text, "{} {ctrl} 0 {init}", new_name("v")).unwrap();
                    } else {
                        let expr = event_expression(init, events);
                        writeln!(text, "{} {ctrl} 0 V= {expr}", new_name("b")).unwrap();
                    }
                    // Безразмерный сигнал 0/1, порог посередине, без гистерезиса.
                    (format!("{ctrl} 0"), 0.5, 0.0)
                } else {
                    (e.nodes[2..].join(" "), s.vt, s.vh)
                };
                writeln!(
                    text,
                    "{name} {} {} {control} {model} {state}",
                    e.nodes[0], e.nodes[1]
                )
                .unwrap();
                writeln!(
                    text,
                    ".model {model} SW(RON={:e} ROFF={:e} VT={vt:e} VH={vh:e})",
                    s.ron, s.roff
                )
                .unwrap();
            }
            Resistor { r } if r.is_finite() && *r != 0.0 => {
                writeln!(text, "{name} {nodes} {r:e}").unwrap()
            }
            Resistor { .. } => return Err(format!("недопустимое сопротивление «{}»", e.name)),
            Capacitor { c, ic } | Inductor { l: c, ic } => {
                write!(text, "{name} {nodes} {c:e}").unwrap();
                if let Some(ic) = ic {
                    write!(text, " IC={ic:e}").unwrap();
                }
                text.push('\n');
            }
            Diode { model, area } if area.is_finite() && *area > 0.0 => {
                writeln!(text, "{name} {nodes} {model} {area:e}").unwrap();
            }
            Diode { .. } => return Err(format!("недопустимая площадь «{}»", e.name)),
            Mosfet { model, w, l }
                if [w, l]
                    .into_iter()
                    .all(|v| v.is_none_or(|v| v.is_finite() && v > 0.0)) =>
            {
                // Размеры по умолчанию совпадают с моделью MOS уровня 1 в Circuit.
                let (w, l) = (w.unwrap_or(100e-6), l.unwrap_or(100e-6));
                writeln!(text, "{name} {nodes} {model} W={w:e} L={l:e}").unwrap();
            }
            Mosfet { .. } => return Err(format!("недопустимый размер «{}»", e.name)),
            Vcvs { gain } | Vccs { gm: gain } => writeln!(text, "{name} {nodes} {gain:e}").unwrap(),
        }
    }
    for model in &netlist.models {
        let kind = match model.kind {
            ModelKind::Switch => continue,
            ModelKind::Diode => "D",
            ModelKind::Nmos => "NMOS",
            ModelKind::Pmos => "PMOS",
        };
        write!(text, ".model {} {kind}(", model.name).unwrap();
        for (key, value) in &model.params {
            write!(text, " {key}={value:e}").unwrap();
        }
        text.push_str(")\n");
    }
    let mut times: Vec<_> = events
        .iter()
        .map(|e| (e.time, !matches!(e.value, NgEventValue::SourceRamp(_))))
        .collect();
    times.sort_by(|a, b| a.0.total_cmp(&b.0));
    times.dedup_by(|a, b| {
        if a.0 == b.0 {
            b.1 |= a.1;
            true
        } else {
            false
        }
    });
    if times.iter().any(|&(t, _)| t > 0.0) {
        // Изолированный источник PWL принуждает ngspice поставить шаг на метке.
        write!(text, "{} {} 0 PWL(0 0", new_name("v"), new_name("n")).unwrap();
        for &(t, _) in times.iter().filter(|&&(t, _)| t > 0.0) {
            write!(text, " {t:.16e} 0").unwrap();
        }
        text.push_str(")\n");
    }
    Ok(NgDeck {
        text,
        source_currents,
        events: times,
    })
}

fn identifier(s: &str) -> bool {
    !s.is_empty()
        && s.bytes()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, b'_' | b'.' | b'-'))
}

fn event_expression(initial: f64, schedule: &[NgEvent<'_>]) -> String {
    let mut expr = String::new();
    for (i, event) in schedule.iter().enumerate().rev() {
        let value = |event: &NgEvent<'_>| match event.value {
            NgEventValue::SourceStep(v) | NgEventValue::SourceRamp(v) => v,
            NgEventValue::Switch(on) => f64::from(on),
        };
        let time = event.time;
        let to = value(event);
        if matches!(event.value, NgEventValue::SourceRamp(_)) {
            let (start, from) = i
                .checked_sub(1)
                .map_or((0.0, initial), |j| (schedule[j].time, value(&schedule[j])));
            // Взвешенная сумма избегает переполнения разности двух конечных амплитуд.
            write!(expr, "time >= {time:.16e} ? ({to:.16e}) : time > {start:.16e} ? (({from:.16e})*((({time:.16e})-time)/({:.16e}))+({to:.16e})*((time-({start:.16e}))/({:.16e}))) : ", time - start, time - start).unwrap();
        } else {
            write!(expr, "time > {time:.16e} ? ({to:.16e}) : ").unwrap();
        }
    }
    write!(expr, "({initial:.16e})").unwrap();
    expr
}

fn source_text(
    out: &mut String,
    src: &crate::netlist::Source,
    netlist: &Netlist,
) -> Result<(), String> {
    match &src.wave {
        Some(Waveform::Ext { initial }) => write!(out, "DC {initial:e}").unwrap(),
        None => write!(out, "DC {:e}", src.dc.unwrap_or(0.0)).unwrap(),
        Some(wave) => {
            // Отдельный .op использует DC, а переходный расчёт — закон источника.
            if let Some(dc) = src.dc {
                write!(out, "DC {dc:e} ").unwrap();
            }
            match wave {
                Waveform::Pwl(points) => {
                    out.push_str("PWL(");
                    for (t, v) in points {
                        write!(out, " {t:.16e} {v:.16e}").unwrap();
                    }
                    out.push(')');
                }
                Waveform::Sin {
                    vo,
                    va,
                    freq,
                    td,
                    theta,
                    phase_deg,
                } => {
                    write!(
                        out,
                        "SIN({vo:e} {va:e} {freq:e} {td:e} {theta:e} {phase_deg:e})"
                    )
                    .unwrap();
                }
                Waveform::Pulse {
                    v1,
                    v2,
                    td,
                    tr,
                    tf,
                    pw,
                    per,
                } => {
                    let edge = |edge: f64| {
                        if edge > 0.0 {
                            Some(edge)
                        } else {
                            netlist.tran().map(|t| t.tstep)
                        }
                    };
                    let (Some(tr), Some(tf)) = (edge(*tr), edge(*tf)) else {
                        return Err("для неявных фронтов PULSE нужен tstep в .tran".into());
                    };
                    // За горизонтом опыта параметры одиночного импульса не наблюдаемы.
                    let horizon = || {
                        netlist
                            .tran()
                            .map(|t| t.tstop)
                            .filter(|t| t.is_finite() && *t > 0.0)
                            .ok_or("для одиночного PULSE нужен конечный горизонт .tran")
                    };
                    let pw = if pw.is_finite() { *pw } else { horizon()? };
                    let per = if per.is_finite() {
                        *per
                    } else {
                        tr + pw + tf + horizon()?
                    };
                    if !per.is_finite() || !tr.is_finite() || !tf.is_finite() {
                        return Err("неконечные времена PULSE".into());
                    }
                    write!(
                        out,
                        "PULSE({v1:e} {v2:e} {td:e} {tr:e} {tf:e} {pw:e} {per:e})"
                    )
                    .unwrap();
                }
                Waveform::Ext { .. } => unreachable!(),
            }
        }
    }
    Ok(())
}

/// Убрать из нетлиста директивы анализа и `.end`, заменить `EXT v` у источников на `DC v`
/// (ngspice не знает внешних источников; при постоянном значении это одно и то же).
pub fn prepare_netlist(netlist: &str) -> String {
    let mut out = String::new();
    let mut control = false;
    for (k, line) in netlist.lines().enumerate() {
        if k > 0 {
            let mut words = line.split_whitespace();
            let original_head = words.next().unwrap_or("");
            let head = original_head.to_ascii_lowercase();
            if head == ".control" {
                control = true;
            }
            if head == ".endc" {
                control = false;
                continue;
            }
            if control {
                continue;
            }
            if head == ".end" {
                break;
            }
            if matches!(head.as_str(), ".tran" | ".op" | ".print" | ".plot") {
                continue;
            }
            if (head.starts_with('v') || head.starts_with('i'))
                && words.skip(2).any(|word| word.eq_ignore_ascii_case("ext"))
                && let Ok(net) = crate::parse(&format!("Схема\n{line}"))
                && let Some(e) = net.elements.first()
                && let ElementKind::VoltageSource(src) | ElementKind::CurrentSource(src) = &e.kind
                && let Some(Waveform::Ext { initial }) = src.wave
            {
                writeln!(out, "{original_head} {} DC {initial}", e.nodes.join(" ")).unwrap();
                continue;
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// Запустить ngspice и напечатать сигналы `signals`.
pub fn run(
    ngspice: &Path,
    netlist: &str,
    analysis: NgAnalysis,
    signals: &[&str],
) -> Result<NgTable, String> {
    if signals.is_empty()
        || signals
            .iter()
            .any(|s| s.is_empty() || s.chars().any(|c| c.is_whitespace() || c == ';'))
    {
        return Err("нужны имена векторов ngspice без пробелов".into());
    }
    let mut columns: Vec<_> = signals.iter().map(|s| s.to_ascii_lowercase()).collect();
    let mut input = prepare_netlist(netlist);
    // В экспоненциальном выводе цифра перед точкой и 16 после неё сохраняют f64.
    input
        .push_str(".control\nset width=10000\nset height=10000000\nset numdgt=16\nset noaskquit\n");
    match analysis {
        NgAnalysis::Op => input.push_str("op\n"),
        NgAnalysis::Tran {
            tstep,
            tstop,
            tstart,
            tmax,
            uic,
        } => {
            if ![tstep, tstop, tstart, tmax].into_iter().all(f64::is_finite)
                || tstep <= 0.0
                || tmax <= 0.0
                || tstart < 0.0
                || tstart >= tstop
            {
                return Err("недопустимая сетка ngspice".into());
            }
            columns.retain(|c| c != "time");
            columns.insert(0, "time".into());
            writeln!(
                input,
                "tran {tstep:.16e} {tstop:.16e} {tstart:.16e} {tmax:.16e}{}",
                if uic { " uic" } else { "" }
            )
            .unwrap();
        }
    }
    writeln!(input, "print {}\nquit\n.endc\n.end", signals.join(" ")).unwrap();
    let mut child = Command::new(ngspice)
        .args(["-n", "-b"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("не удалось запустить ngspice: {e}"))?;
    let written = child
        .stdin
        .take()
        .ok_or("нет stdin у ngspice")?
        .write_all(input.as_bytes());
    if let Err(e) = written {
        let _ = child.kill();
        let _ = child.wait();
        return Err(format!("ошибка записи в ngspice: {e}"));
    }
    let out = child
        .wait_with_output()
        .map_err(|e| format!("ngspice: {e}"))?;
    let text = String::from_utf8_lossy(&out.stdout);
    let err = String::from_utf8_lossy(&out.stderr);
    let result = if !out.status.success()
        || err.lines().any(|l| {
            let l = l.trim().to_ascii_lowercase();
            l.starts_with("error") || l.contains("simulation(s) aborted")
        }) {
        Err(format!("ошибка расчёта ngspice ({})", out.status))
    } else {
        match analysis {
            NgAnalysis::Op => parse_op(&text, signals),
            NgAnalysis::Tran {
                tstart,
                tstop,
                tmax,
                ..
            } => parse_table(&text)
                .and_then(|table| validate_trace(&text, table, &columns, tstart, tstop, tmax)),
        }
    };
    result.map_err(|e| {
        format!(
            "{e}; stderr ngspice: {}",
            err.lines().take(5).collect::<Vec<_>>().join(" | ")
        )
    })
}

fn same_time(a: f64, b: f64) -> bool {
    // Погрешность округления при разборе десятичных меток в двух программах.
    a >= b.next_down().next_down() && a <= b.next_up().next_up()
}

fn validate_trace(
    text: &str,
    table: NgTable,
    columns: &[String],
    tstart: f64,
    tstop: f64,
    tmax: f64,
) -> Result<NgTable, String> {
    let count = text.lines().find_map(|l| {
        l.trim()
            .strip_prefix("No. of Data Rows :")?
            .trim()
            .parse::<usize>()
            .ok()
    });
    if count != Some(table.rows.len()) || table.columns != columns {
        return Err("ngspice: неполная таблица или отсутствуют запрошенные векторы".into());
    }
    let first = table.rows.first().unwrap()[0];
    let last = table.rows.last().unwrap()[0];
    if (first < tstart && !same_time(first, tstart))
        || first > tstart + tmax
        || !same_time(last, tstop)
        || table.rows.windows(2).any(|w| {
            let (a, b) = (w[0][0], w[1][0]);
            b <= a || (b > a + tmax && !same_time(b, a + tmax))
        })
    {
        return Err(format!(
            "ngspice: неполная или немонотонная шкала времени [{first:e}, {last:e}] с"
        ));
    }
    Ok(table)
}

fn parse_op(text: &str, signals: &[&str]) -> Result<NgTable, String> {
    let mut row = Vec::new();
    for s in signals {
        let s = s.to_lowercase();
        let v = text
            .lines()
            .find_map(|l| {
                let (name, val) = l.split_once('=')?;
                (name.trim().to_lowercase() == s)
                    .then(|| val.trim().parse::<f64>().ok())
                    .flatten()
            })
            .ok_or_else(|| format!("ngspice не напечатал {s}"))?;
        if !v.is_finite() {
            return Err(format!("ngspice: неконечный вектор {s}"));
        }
        row.push(v);
    }
    Ok(NgTable {
        columns: signals.iter().map(|s| s.to_lowercase()).collect(),
        rows: vec![row],
    })
}

fn parse_table(text: &str) -> Result<NgTable, String> {
    let mut columns: Vec<String> = Vec::new();
    let mut rows = Vec::new();
    for line in text.lines() {
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.first() == Some(&"Index") {
            let header: Vec<_> = toks[1..].iter().map(|s| s.to_lowercase()).collect();
            if !columns.is_empty() && columns != header {
                return Err("ngspice: изменился заголовок таблицы".into());
            }
            columns = header;
            continue;
        }
        let Some(index) = toks.first().and_then(|s| s.parse::<usize>().ok()) else {
            continue;
        };
        if columns.is_empty() {
            continue;
        }
        if index != rows.len() || toks.len() != columns.len() + 1 {
            return Err(format!(
                "ngspice: пропуск или неверная длина строки {index}"
            ));
        }
        let values: Option<Vec<_>> = toks[1..]
            .iter()
            .map(|t| t.parse::<f64>().ok().filter(|v| v.is_finite()))
            .collect();
        rows.push(
            values.ok_or_else(|| format!("ngspice: неконечная или нечитаемая строка {index}"))?,
        );
    }
    if rows.is_empty() {
        return Err("в выводе ngspice нет таблицы".into());
    }
    Ok(NgTable { columns, rows })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prepare_replaces_ext_and_drops_analyses() {
        let n = prepare_netlist("t\nVb a 0 EXT 4\nR1 a 0 1\n.tran 1u 1m\n.end\n");
        assert!(n.contains("Vb a 0 DC 4"));
        assert!(!n.contains(".tran") && !n.contains(".end"));
    }

    #[test]
    fn parses_tables() {
        let t = parse_table(
            "junk\nIndex   time            v(out)\n-----\n0\t0.000000e+00\t1.0e-3\t\n1\t1.0e-4\t2.0\n",
        )
        .unwrap();
        assert_eq!(t.columns, vec!["time", "v(out)"]);
        assert_eq!(t.column("v(out)").unwrap(), vec![1.0e-3, 2.0]);
        let o = parse_op("v(a) = 6.5e-01\ni(v1) = -4.3e-03\n", &["v(a)", "i(v1)"]).unwrap();
        assert_eq!(o.rows[0], vec![0.65, -4.3e-3]);
    }

    #[test]
    fn rejects_nonfinite_and_truncated_tables() {
        assert!(parse_op("v(a) = NaN\n", &["v(a)"]).is_err());
        assert!(parse_table("Index time v(a)\n0 0 0\n1 1e-4 inf\n").is_err());
        assert!(parse_table("Index time v(a)\n0 0 0\n2 2e-4 1\n").is_err());
        let text = "No. of Data Rows : 3\nIndex time v(a)\n0 0 0\n1 1e-4 1\n";
        let table = parse_table(text).unwrap();
        let columns = table.columns.clone();
        assert!(validate_trace(text, table, &columns, 0.0, 2e-4, 1e-4).is_err());
    }
}
