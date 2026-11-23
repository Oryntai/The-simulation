//! Разбор нетлиста — подмножество синтаксиса SPICE (Berkeley SPICE3 / ngspice).
//!
//! Правила (как в SPICE):
//! * первая строка — **всегда заголовок** схемы, даже если похожа на элемент;
//! * регистр не важен (всё приводится к нижнему), `*` в начале строки — комментарий,
//!   `;` — комментарий до конца строки, строка с `+` в начале продолжает предыдущую;
//! * `.end` завершает нетлист; узел `0` (или `gnd`) — земля;
//! * числа — с инженерными суффиксами, см. [`crate::units::parse_value`].
//!
//! Элементы: `R C L V I D S M E G X`. Директивы: `.model`, `.subckt/.ends`, `.op`,
//! `.tran tstep tstop [tstart [tmax]] [uic]`, `.end`. Директивы вывода (`.print`, `.plot`,
//! `.options`, `.save`, `.probe`, `.width`, `.temp`) и блок `.control … .endc` игнорируются
//! с предупреждением. Подстановки параметров (`.param`, `{…}`), `.include/.lib`, `.ic/.nodeset`
//! не поддерживаются — это ошибка, а не молчаливый пропуск: иначе схема была бы посчитана не та.
//!
//! Подсхемы раскрываются при разборе: элемент `R1` внутри экземпляра `X1` получает имя
//! `x1.r1`, внутренний узел `n` — имя `x1.n`; узел `0` остаётся глобальной землёй.
//! Модели (`.model`) глобальны, где бы ни стояли.

use std::collections::HashMap;
use std::fmt;

use crate::units::parse_value;

/// Ошибка разбора: номер строки исходного текста (с 1) и сообщение по-русски.
#[derive(Clone, Debug, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "строка {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

fn err<T>(line: usize, message: impl Into<String>) -> Result<T, ParseError> {
    Err(ParseError {
        line,
        message: message.into(),
    })
}

/// Разобранный и «сплющенный» (подсхемы раскрыты) нетлист.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Netlist {
    pub title: String,
    pub elements: Vec<Element>,
    pub models: Vec<ModelCard>,
    pub analyses: Vec<Analysis>,
    /// Что было проигнорировано (директивы вывода, неподдерживаемые параметры моделей).
    pub warnings: Vec<String>,
}

impl Netlist {
    pub fn model(&self, name: &str) -> Option<&ModelCard> {
        let name = name.to_lowercase();
        self.models.iter().find(|m| m.name == name)
    }

    pub fn element(&self, name: &str) -> Option<&Element> {
        let name = name.to_lowercase();
        self.elements.iter().find(|e| e.name == name)
    }

    /// Параметры `.tran`, если директива есть.
    pub fn tran(&self) -> Option<&TranSpec> {
        self.analyses.iter().find_map(|a| match a {
            Analysis::Tran(t) => Some(t),
            Analysis::Op => None,
        })
    }
}

/// Элемент схемы. Узлы — в порядке SPICE для данного типа элемента.
#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    /// Имя в нижнем регистре (`r1`, `x1.r1`).
    pub name: String,
    pub nodes: Vec<String>,
    pub kind: ElementKind,
    /// Строка исходного текста (для сообщений об ошибках).
    pub line: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ElementKind {
    /// `Rxx n+ n- R`
    Resistor { r: f64 },
    /// `Cxx n+ n- C [IC=v0]`
    Capacitor { c: f64, ic: Option<f64> },
    /// `Lxx n+ n- L [IC=i0]`
    Inductor { l: f64, ic: Option<f64> },
    /// `Vxx n+ n- [DC v] [PULSE(…)|SIN(…)|PWL(…)|EXT v0]`; ток ветви течёт из n+ через источник в n-.
    VoltageSource(Source),
    /// `Ixx n+ n- …`; ток течёт из n+ через источник в n- (то есть втекает в узел n-).
    CurrentSource(Source),
    /// `Dxx анод катод модель [площадь]`
    Diode { model: String, area: f64 },
    /// `Sxx n+ n- nc+ nc- модель [ON|OFF]` или `Sxx n+ n- EXT [модель] [ON|OFF] [RON= ROFF=]`.
    Switch {
        control: SwitchControl,
        model: Option<String>,
        initially_on: bool,
        ron: Option<f64>,
        roff: Option<f64>,
    },
    /// `Mxx d g s b модель [W=] [L=]`
    Mosfet {
        model: String,
        w: Option<f64>,
        l: Option<f64>,
    },
    /// `Exx n+ n- nc+ nc- коэффициент` — ИНУН.
    Vcvs { gain: f64 },
    /// `Gxx n+ n- nc+ nc- крутизна` — ИТУН, ток из n+ через источник в n-.
    Vccs { gm: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SwitchControl {
    /// Управляется напряжением между узлами nc+ и nc- (узлы 3 и 4 элемента).
    Voltage,
    /// Управляется из кода хоста: `Transient::set_switch`.
    External,
}

/// Описание независимого источника.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Source {
    /// Значение для расчёта рабочей точки (`DC v` или просто число).
    pub dc: Option<f64>,
    /// Функция времени для переходного процесса.
    pub wave: Option<Waveform>,
}

/// Функции времени источников (смысл параметров — как в SPICE).
#[derive(Clone, Debug, PartialEq)]
pub enum Waveform {
    /// `PULSE(v1 v2 td tr tf pw per)`. Нулевые tr/tf при расчёте заменяются шагом `.tran`
    /// (как в SPICE); отсутствующие pw/per — бесконечность (один импульс).
    Pulse {
        v1: f64,
        v2: f64,
        td: f64,
        tr: f64,
        tf: f64,
        pw: f64,
        per: f64,
    },
    /// `SIN(vo va freq [td [theta [phase°]]])`: vo + va·e^(−θ(t−td))·sin(2πf(t−td) + φ) при t ≥ td.
    Sin {
        vo: f64,
        va: f64,
        freq: f64,
        td: f64,
        theta: f64,
        phase_deg: f64,
    },
    /// `PWL(t1 v1 t2 v2 …)` — кусочно-линейная; до t1 — v1, после последней точки — последнее значение.
    Pwl(Vec<(f64, f64)>),
    /// `EXT v0` — значение задаёт код хоста на каждом шаге (`Transient::set_source`).
    Ext { initial: f64 },
}

/// Карточка `.model`.
#[derive(Clone, Debug, PartialEq)]
pub struct ModelCard {
    pub name: String,
    pub kind: ModelKind,
    pub params: Vec<(String, f64)>,
    pub line: usize,
}

impl ModelCard {
    pub fn get(&self, key: &str) -> Option<f64> {
        self.params.iter().find(|(k, _)| k == key).map(|(_, v)| *v)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelKind {
    Diode,
    Switch,
    Nmos,
    Pmos,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Analysis {
    Op,
    Tran(TranSpec),
}

/// `.tran tstep tstop [tstart [tmax]] [uic]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TranSpec {
    pub tstep: f64,
    pub tstop: f64,
    pub tstart: f64,
    pub tmax: Option<f64>,
    /// Не считать рабочую точку, стартовать с IC= элементов (остальное — нули).
    pub uic: bool,
}

// ───────────────────────────── Лексер ─────────────────────────────

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Word(String),
    Kv(String, String),
}

impl Tok {
    fn word(&self) -> Option<&str> {
        match self {
            Tok::Word(w) => Some(w),
            Tok::Kv(..) => None,
        }
    }
}

fn tokenize(s: &str) -> Vec<Tok> {
    let mut raw: Vec<String> = Vec::new();
    let mut cur = String::new();
    for ch in s.chars() {
        match ch {
            c if c.is_whitespace() || c == '(' || c == ')' || c == ',' => {
                if !cur.is_empty() {
                    raw.push(std::mem::take(&mut cur));
                }
            }
            '=' => {
                if !cur.is_empty() {
                    raw.push(std::mem::take(&mut cur));
                }
                raw.push("=".into());
            }
            c => cur.push(c),
        }
    }
    if !cur.is_empty() {
        raw.push(cur);
    }
    let mut toks = Vec::with_capacity(raw.len());
    let mut i = 0;
    while i < raw.len() {
        if i + 2 < raw.len() && raw[i + 1] == "=" && raw[i] != "=" {
            toks.push(Tok::Kv(raw[i].clone(), raw[i + 2].clone()));
            i += 3;
        } else {
            toks.push(Tok::Word(raw[i].clone()));
            i += 1;
        }
    }
    toks
}

/// Логические строки: (номер первой физической строки, текст в нижнем регистре без комментариев).
fn logical_lines(text: &str) -> (String, Vec<(usize, String)>) {
    let mut lines = text.lines();
    let title = lines.next().unwrap_or("").trim().to_string();
    let mut out: Vec<(usize, String)> = Vec::new();
    for (idx, raw) in lines.enumerate() {
        let lineno = idx + 2;
        let mut s = raw.trim();
        if s.starts_with('*') {
            continue;
        }
        if let Some(p) = s.find(';') {
            s = s[..p].trim();
        }
        if s.is_empty() {
            continue;
        }
        let s = s.to_lowercase();
        if let Some(rest) = s.strip_prefix('+')
            && let Some(last) = out.last_mut()
        {
            last.1.push(' ');
            last.1.push_str(rest);
            continue;
        }
        out.push((lineno, s));
    }
    (title, out)
}

// ───────────────────────────── Разбор ─────────────────────────────

#[derive(Clone, Debug)]
struct Instance {
    name: String,
    nodes: Vec<String>,
    subckt: String,
    line: usize,
}

#[derive(Clone, Debug)]
enum Card {
    Element(Element),
    Instance(Instance),
}

#[derive(Clone, Debug)]
struct SubcktDef {
    ports: Vec<String>,
    cards: Vec<Card>,
}

/// Разобрать текст нетлиста.
pub fn parse(text: &str) -> Result<Netlist, ParseError> {
    let (title, lines) = logical_lines(text);
    let mut net = Netlist {
        title,
        ..Default::default()
    };
    let mut top: Vec<Card> = Vec::new();
    let mut subckts: HashMap<String, SubcktDef> = HashMap::new();
    // Открытая сейчас подсхема: (имя, строка, определение).
    let mut open: Option<(String, usize, SubcktDef)> = None;
    let mut in_control = false;

    for (line, s) in &lines {
        let line = *line;
        let toks = tokenize(s);
        let Some(Tok::Word(head)) = toks.first() else {
            return err(
                line,
                "строка должна начинаться с имени элемента или директивы",
            );
        };
        let head = head.clone();
        if in_control {
            if head == ".endc" {
                in_control = false;
            }
            continue;
        }
        if head.starts_with('.') {
            match head.as_str() {
                ".end" => break,
                ".model" => net
                    .models
                    .push(parse_model(&toks, line, &mut net.warnings)?),
                ".subckt" => {
                    if open.is_some() {
                        return err(line, "вложенное определение .subckt не поддерживается");
                    }
                    let name = toks.get(1).and_then(Tok::word).ok_or(ParseError {
                        line,
                        message: ".subckt: нет имени подсхемы".into(),
                    })?;
                    let mut ports = Vec::new();
                    for t in &toks[2..] {
                        match t {
                            Tok::Word(w) if w == "params:" => {
                                return err(line, "параметры подсхем (params:) не поддерживаются");
                            }
                            Tok::Word(w) => ports.push(norm_node(w)),
                            Tok::Kv(..) => {
                                return err(line, "параметры подсхем не поддерживаются");
                            }
                        }
                    }
                    open = Some((
                        name.to_string(),
                        line,
                        SubcktDef {
                            ports,
                            cards: Vec::new(),
                        },
                    ));
                }
                ".ends" => {
                    let Some((name, _, def)) = open.take() else {
                        return err(line, ".ends без .subckt");
                    };
                    if subckts.insert(name.clone(), def).is_some() {
                        return err(line, format!("подсхема «{name}» определена дважды"));
                    }
                }
                ".op" => net.analyses.push(Analysis::Op),
                ".tran" => net.analyses.push(Analysis::Tran(parse_tran(&toks, line)?)),
                ".control" => {
                    in_control = true;
                    net.warnings.push(format!(
                        "строка {line}: блок .control … .endc проигнорирован"
                    ));
                }
                ".options" | ".option" | ".opt" | ".print" | ".plot" | ".probe" | ".save"
                | ".width" | ".temp" | ".title" => {
                    net.warnings
                        .push(format!("строка {line}: директива {head} проигнорирована"));
                }
                ".param" | ".include" | ".inc" | ".lib" | ".ic" | ".nodeset" | ".func" => {
                    return err(
                        line,
                        format!("директива {head} не поддерживается этим решателем"),
                    );
                }
                _ => return err(line, format!("неизвестная директива {head}")),
            }
            continue;
        }
        let card = parse_card(&head, &toks, line, &mut net.warnings)?;
        match &mut open {
            Some((_, _, def)) => def.cards.push(card),
            None => top.push(card),
        }
    }
    if let Some((name, line, _)) = open {
        return err(line, format!("подсхема «{name}» не закрыта .ends"));
    }

    // Раскрыть подсхемы.
    for card in top {
        expand(card, "", &HashMap::new(), &subckts, 0, &mut net.elements)?;
    }
    // Имена элементов уникальны.
    let mut seen: HashMap<&str, usize> = HashMap::new();
    for e in &net.elements {
        if let Some(prev) = seen.insert(e.name.as_str(), e.line) {
            return err(
                e.line,
                format!(
                    "элемент «{}» уже объявлен в строке {prev}; имена должны быть уникальны",
                    e.name
                ),
            );
        }
    }
    let mut mseen: HashMap<&str, usize> = HashMap::new();
    for m in &net.models {
        if let Some(prev) = mseen.insert(m.name.as_str(), m.line) {
            return err(
                m.line,
                format!("модель «{}» уже объявлена в строке {prev}", m.name),
            );
        }
    }
    Ok(net)
}

fn norm_node(s: &str) -> String {
    if s == "gnd" || s == "0" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

const MAX_SUBCKT_DEPTH: usize = 20;

fn expand(
    card: Card,
    prefix: &str,
    node_map: &HashMap<String, String>,
    subckts: &HashMap<String, SubcktDef>,
    depth: usize,
    out: &mut Vec<Element>,
) -> Result<(), ParseError> {
    let map_node = |n: &str| -> String {
        if n == "0" {
            "0".into()
        } else if let Some(m) = node_map.get(n) {
            m.clone()
        } else if prefix.is_empty() {
            n.to_string()
        } else {
            format!("{prefix}.{n}")
        }
    };
    let map_name = |n: &str| -> String {
        if prefix.is_empty() {
            n.to_string()
        } else {
            format!("{prefix}.{n}")
        }
    };
    match card {
        Card::Element(mut e) => {
            e.nodes = e.nodes.iter().map(|n| map_node(n)).collect();
            e.name = map_name(&e.name);
            out.push(e);
        }
        Card::Instance(inst) => {
            if depth >= MAX_SUBCKT_DEPTH {
                return err(
                    inst.line,
                    "слишком глубокая вложенность подсхем (рекурсия?)",
                );
            }
            let Some(def) = subckts.get(&inst.subckt) else {
                return err(
                    inst.line,
                    format!("подсхема «{}» не определена", inst.subckt),
                );
            };
            if def.ports.len() != inst.nodes.len() {
                return err(
                    inst.line,
                    format!(
                        "у подсхемы «{}» {} выводов, а в экземпляре {} узлов",
                        inst.subckt,
                        def.ports.len(),
                        inst.nodes.len()
                    ),
                );
            }
            let child_prefix = map_name(&inst.name);
            let mut child_map = HashMap::new();
            for (port, node) in def.ports.iter().zip(&inst.nodes) {
                child_map.insert(port.clone(), map_node(node));
            }
            for c in def.cards.clone() {
                expand(c, &child_prefix, &child_map, subckts, depth + 1, out)?;
            }
        }
    }
    Ok(())
}

fn num(tok: Option<&Tok>, what: &str, line: usize) -> Result<f64, ParseError> {
    match tok {
        Some(Tok::Word(w)) => parse_value(w).ok_or(ParseError {
            line,
            message: format!("{what}: «{w}» — не число"),
        }),
        Some(Tok::Kv(k, _)) => err(line, format!("{what}: ожидалось число, а не «{k}=…»")),
        None => err(line, format!("{what}: значение не указано")),
    }
}

fn kv_num(key: &str, val: &str, line: usize) -> Result<f64, ParseError> {
    parse_value(val).ok_or(ParseError {
        line,
        message: format!("параметр {key}: «{val}» — не число"),
    })
}

fn nodes_of(toks: &[Tok], n: usize, name: &str, line: usize) -> Result<Vec<String>, ParseError> {
    let mut nodes = Vec::with_capacity(n);
    for i in 0..n {
        match toks.get(1 + i) {
            Some(Tok::Word(w)) => nodes.push(norm_node(w)),
            _ => {
                return err(
                    line,
                    format!("у элемента «{name}» должно быть {n} узла(ов)"),
                );
            }
        }
    }
    Ok(nodes)
}

fn parse_card(
    name: &str,
    toks: &[Tok],
    line: usize,
    warnings: &mut Vec<String>,
) -> Result<Card, ParseError> {
    let first = name.chars().next().unwrap_or(' ');
    let el = |nodes: Vec<String>, kind: ElementKind| {
        Ok(Card::Element(Element {
            name: name.to_string(),
            nodes,
            kind,
            line,
        }))
    };
    let warn_extra = |warnings: &mut Vec<String>, rest: &[Tok]| {
        for t in rest {
            let what = match t {
                Tok::Word(w) => w.clone(),
                Tok::Kv(k, v) => format!("{k}={v}"),
            };
            warnings.push(format!(
                "строка {line}: у «{name}» параметр «{what}» проигнорирован"
            ));
        }
    };
    match first {
        'r' => {
            let nodes = nodes_of(toks, 2, name, line)?;
            let (r, rest) = match toks.get(3) {
                Some(Tok::Kv(k, v)) if k == "r" => (kv_num(k, v, line)?, &toks[4..]),
                t => (
                    num(t, &format!("сопротивление {name}"), line)?,
                    toks.get(4..).unwrap_or(&[]),
                ),
            };
            if r == 0.0 || !r.is_finite() {
                return err(line, format!("сопротивление {name} должно быть ненулевым"));
            }
            warn_extra(warnings, rest);
            el(nodes, ElementKind::Resistor { r })
        }
        'c' | 'l' => {
            let nodes = nodes_of(toks, 2, name, line)?;
            let what = if first == 'c' {
                "ёмкость"
            } else {
                "индуктивность"
            };
            let value = num(toks.get(3), &format!("{what} {name}"), line)?;
            if value <= 0.0 || !value.is_finite() {
                return err(line, format!("{what} {name} должна быть положительной"));
            }
            let mut ic = None;
            for t in toks.get(4..).unwrap_or(&[]) {
                match t {
                    Tok::Kv(k, v) if k == "ic" => ic = Some(kv_num(k, v, line)?),
                    other => warn_extra(warnings, std::slice::from_ref(other)),
                }
            }
            if first == 'c' {
                el(nodes, ElementKind::Capacitor { c: value, ic })
            } else {
                el(nodes, ElementKind::Inductor { l: value, ic })
            }
        }
        'v' | 'i' => {
            let nodes = nodes_of(toks, 2, name, line)?;
            let src = parse_source(&toks[3..], name, line)?;
            if first == 'v' {
                el(nodes, ElementKind::VoltageSource(src))
            } else {
                el(nodes, ElementKind::CurrentSource(src))
            }
        }
        'd' => {
            let nodes = nodes_of(toks, 2, name, line)?;
            let model = toks.get(3).and_then(Tok::word).ok_or(ParseError {
                line,
                message: format!("у диода {name} не указана модель"),
            })?;
            let mut area = 1.0;
            let mut rest = toks.get(4..).unwrap_or(&[]);
            if let Some(Tok::Word(w)) = rest.first()
                && let Some(a) = parse_value(w)
            {
                if a <= 0.0 {
                    return err(
                        line,
                        format!("площадь диода {name} должна быть положительной"),
                    );
                }
                area = a;
                rest = &rest[1..];
            }
            warn_extra(warnings, rest);
            el(
                nodes,
                ElementKind::Diode {
                    model: model.to_string(),
                    area,
                },
            )
        }
        's' => parse_switch(name, toks, line, warnings),
        'm' => {
            let nodes = nodes_of(toks, 4, name, line)?;
            let model = toks.get(5).and_then(Tok::word).ok_or(ParseError {
                line,
                message: format!("у транзистора {name} не указана модель"),
            })?;
            let (mut w, mut l) = (None, None);
            for t in toks.get(6..).unwrap_or(&[]) {
                match t {
                    Tok::Kv(k, v) if k == "w" => w = Some(kv_num(k, v, line)?),
                    Tok::Kv(k, v) if k == "l" => l = Some(kv_num(k, v, line)?),
                    other => warn_extra(warnings, std::slice::from_ref(other)),
                }
            }
            if w.is_some_and(|x| x <= 0.0) || l.is_some_and(|x| x <= 0.0) {
                return err(
                    line,
                    format!("W и L транзистора {name} должны быть положительными"),
                );
            }
            el(
                nodes,
                ElementKind::Mosfet {
                    model: model.to_string(),
                    w,
                    l,
                },
            )
        }
        'e' | 'g' => {
            let nodes = nodes_of(toks, 4, name, line)?;
            let k = num(toks.get(5), &format!("коэффициент {name}"), line)?;
            warn_extra(warnings, toks.get(6..).unwrap_or(&[]));
            if first == 'e' {
                el(nodes, ElementKind::Vcvs { gain: k })
            } else {
                el(nodes, ElementKind::Vccs { gm: k })
            }
        }
        'x' => {
            let mut words = Vec::new();
            for t in &toks[1..] {
                match t {
                    Tok::Word(w) => words.push(w.clone()),
                    Tok::Kv(..) => {
                        return err(line, "параметры экземпляров подсхем не поддерживаются");
                    }
                }
            }
            let Some(subckt) = words.pop() else {
                return err(line, format!("у экземпляра {name} не указана подсхема"));
            };
            Ok(Card::Instance(Instance {
                name: name.to_string(),
                nodes: words.iter().map(|w| norm_node(w)).collect(),
                subckt,
                line,
            }))
        }
        _ => err(
            line,
            format!("неизвестный тип элемента «{name}» (поддерживаются R C L V I D S M E G X)"),
        ),
    }
}

fn parse_switch(
    name: &str,
    toks: &[Tok],
    line: usize,
    warnings: &mut Vec<String>,
) -> Result<Card, ParseError> {
    let mut nodes = nodes_of(toks, 2, name, line)?;
    let external = matches!(toks.get(3), Some(Tok::Word(w)) if w == "ext");
    let rest_start;
    let (control, mut model) = if external {
        rest_start = 4;
        (SwitchControl::External, None)
    } else {
        let mut all = nodes_of(toks, 4, name, line)?;
        nodes = std::mem::take(&mut all);
        let model = toks.get(5).and_then(Tok::word).ok_or(ParseError {
            line,
            message: format!(
                "у ключа {name} нет модели: «S n+ n- nc+ nc- модель» или «S n+ n- EXT»"
            ),
        })?;
        rest_start = 6;
        (SwitchControl::Voltage, Some(model.to_string()))
    };
    let (mut on, mut ron, mut roff) = (false, None, None);
    for t in toks.get(rest_start..).unwrap_or(&[]) {
        match t {
            Tok::Word(w) if w == "on" => on = true,
            Tok::Word(w) if w == "off" => on = false,
            Tok::Word(w) if external && model.is_none() && parse_value(w).is_none() => {
                model = Some(w.clone())
            }
            Tok::Kv(k, v) if k == "ron" => ron = Some(kv_num(k, v, line)?),
            Tok::Kv(k, v) if k == "roff" => roff = Some(kv_num(k, v, line)?),
            Tok::Word(w) => warnings.push(format!(
                "строка {line}: у «{name}» слово «{w}» проигнорировано"
            )),
            Tok::Kv(k, v) => warnings.push(format!(
                "строка {line}: у «{name}» параметр «{k}={v}» проигнорирован"
            )),
        }
    }
    for r in [ron, roff].into_iter().flatten() {
        if r <= 0.0 {
            return err(
                line,
                format!("RON/ROFF ключа {name} должны быть положительными"),
            );
        }
    }
    Ok(Card::Element(Element {
        name: name.to_string(),
        nodes,
        kind: ElementKind::Switch {
            control,
            model,
            initially_on: on,
            ron,
            roff,
        },
        line,
    }))
}

fn collect_numbers(
    toks: &[Tok],
    i: &mut usize,
    line: usize,
    what: &str,
) -> Result<Vec<f64>, ParseError> {
    let mut v = Vec::new();
    while let Some(Tok::Word(w)) = toks.get(*i) {
        match parse_value(w) {
            Some(x) => v.push(x),
            None => {
                if is_source_keyword(w) {
                    break;
                }
                return err(line, format!("{what}: «{w}» — не число"));
            }
        }
        *i += 1;
    }
    Ok(v)
}

fn is_source_keyword(w: &str) -> bool {
    matches!(w, "dc" | "ac" | "pulse" | "sin" | "pwl" | "ext")
}

fn parse_source(toks: &[Tok], name: &str, line: usize) -> Result<Source, ParseError> {
    let mut src = Source::default();
    let mut i = 0;
    while i < toks.len() {
        let Tok::Word(w) = &toks[i] else {
            let Tok::Kv(k, _) = &toks[i] else {
                unreachable!()
            };
            return err(
                line,
                format!("у источника {name} неизвестный параметр «{k}=»"),
            );
        };
        if let Some(x) = parse_value(w) {
            if src.dc.is_some() {
                return err(line, format!("у источника {name} лишнее число «{w}»"));
            }
            src.dc = Some(x);
            i += 1;
            continue;
        }
        i += 1;
        let set_wave = |src: &mut Source, wave: Waveform| -> Result<(), ParseError> {
            if src.wave.is_some() {
                return err(line, format!("у источника {name} две функции времени"));
            }
            src.wave = Some(wave);
            Ok(())
        };
        match w.as_str() {
            "dc" => {
                src.dc = Some(num(toks.get(i), &format!("DC-значение {name}"), line)?);
                i += 1;
            }
            "ac" => {
                // Малосигнальный анализ не поддерживается — амплитуда/фаза пропускаются.
                let _ = collect_numbers(toks, &mut i, line, "AC")?;
            }
            "pulse" => {
                let p = collect_numbers(toks, &mut i, line, "PULSE")?;
                if p.len() < 2 || p.len() > 7 {
                    return err(line, "PULSE(v1 v2 [td tr tf pw per]): от 2 до 7 чисел");
                }
                let g = |k: usize, d: f64| p.get(k).copied().unwrap_or(d);
                let wave = Waveform::Pulse {
                    v1: p[0],
                    v2: p[1],
                    td: g(2, 0.0),
                    tr: g(3, 0.0),
                    tf: g(4, 0.0),
                    pw: g(5, f64::INFINITY),
                    per: g(6, f64::INFINITY),
                };
                if g(2, 0.0) < 0.0 || g(3, 0.0) < 0.0 || g(4, 0.0) < 0.0 || g(5, 1.0) < 0.0 {
                    return err(line, "PULSE: времена не могут быть отрицательными");
                }
                if g(6, 1.0) <= 0.0 {
                    return err(line, "PULSE: период должен быть положительным");
                }
                set_wave(&mut src, wave)?;
            }
            "sin" => {
                let p = collect_numbers(toks, &mut i, line, "SIN")?;
                if p.len() < 3 || p.len() > 6 {
                    return err(line, "SIN(vo va freq [td theta phase]): от 3 до 6 чисел");
                }
                let g = |k: usize| p.get(k).copied().unwrap_or(0.0);
                set_wave(
                    &mut src,
                    Waveform::Sin {
                        vo: p[0],
                        va: p[1],
                        freq: p[2],
                        td: g(3),
                        theta: g(4),
                        phase_deg: g(5),
                    },
                )?;
            }
            "pwl" => {
                let p = collect_numbers(toks, &mut i, line, "PWL")?;
                if p.len() < 2 || p.len() % 2 != 0 {
                    return err(line, "PWL(t1 v1 t2 v2 …): нужно чётное число значений");
                }
                let pts: Vec<(f64, f64)> = p.chunks(2).map(|c| (c[0], c[1])).collect();
                if pts.windows(2).any(|w| w[1].0 <= w[0].0) {
                    return err(line, "PWL: моменты времени должны строго возрастать");
                }
                set_wave(&mut src, Waveform::Pwl(pts))?;
            }
            "ext" => {
                let p = collect_numbers(toks, &mut i, line, "EXT")?;
                if p.len() > 1 {
                    return err(line, "EXT [начальное значение]: не больше одного числа");
                }
                let initial = p.first().copied().or(src.dc).unwrap_or(0.0);
                set_wave(&mut src, Waveform::Ext { initial })?;
            }
            other => {
                return err(
                    line,
                    format!("у источника {name} непонятное слово «{other}»"),
                );
            }
        }
    }
    Ok(src)
}

fn parse_model(
    toks: &[Tok],
    line: usize,
    warnings: &mut Vec<String>,
) -> Result<ModelCard, ParseError> {
    let name = toks.get(1).and_then(Tok::word).ok_or(ParseError {
        line,
        message: ".model: нет имени модели".into(),
    })?;
    let kind_word = toks.get(2).and_then(Tok::word).ok_or(ParseError {
        line,
        message: format!(".model {name}: нет типа модели"),
    })?;
    let kind = match kind_word {
        "d" => ModelKind::Diode,
        "sw" => ModelKind::Switch,
        "nmos" => ModelKind::Nmos,
        "pmos" => ModelKind::Pmos,
        other => {
            return err(
                line,
                format!("тип модели «{other}» не поддерживается (есть D, SW, NMOS, PMOS)"),
            );
        }
    };
    let mut params = Vec::new();
    for t in &toks[3..] {
        match t {
            Tok::Kv(k, v) => params.push((k.clone(), kv_num(k, v, line)?)),
            Tok::Word(w) => {
                return err(
                    line,
                    format!(".model {name}: «{w}» — ожидалось ИМЯ=значение"),
                );
            }
        }
    }
    let known: &[&str] = match kind {
        ModelKind::Diode => &["is", "n", "rs"],
        ModelKind::Switch => &["ron", "roff", "vt", "vh"],
        ModelKind::Nmos | ModelKind::Pmos => &["vto", "kp", "lambda", "is", "level"],
    };
    for (k, v) in &params {
        if k == "level" && *v != 1.0 {
            return err(
                line,
                format!(".model {name}: поддерживается только LEVEL=1 (Шихман–Ходжес)"),
            );
        }
        if !known.contains(&k.as_str()) {
            warnings.push(format!(
                "строка {line}: параметр {k} модели «{name}» не моделируется и проигнорирован"
            ));
        }
    }
    Ok(ModelCard {
        name: name.to_string(),
        kind,
        params,
        line,
    })
}

fn parse_tran(toks: &[Tok], line: usize) -> Result<TranSpec, ParseError> {
    let mut nums = Vec::new();
    let mut uic = false;
    for t in &toks[1..] {
        match t {
            Tok::Word(w) if w == "uic" => uic = true,
            t => nums.push(num(Some(t), ".tran", line)?),
        }
    }
    if nums.len() < 2 || nums.len() > 4 {
        return err(line, ".tran tstep tstop [tstart [tmax]] [uic]");
    }
    let spec = TranSpec {
        tstep: nums[0],
        tstop: nums[1],
        tstart: nums.get(2).copied().unwrap_or(0.0),
        tmax: nums.get(3).copied(),
        uic,
    };
    if spec.tstep <= 0.0 || spec.tstop <= 0.0 {
        return err(line, ".tran: шаг и время конца должны быть положительными");
    }
    if spec.tstart < 0.0 || spec.tstart >= spec.tstop {
        return err(line, ".tran: tstart должен быть в [0, tstop)");
    }
    if spec.tmax.is_some_and(|m| m <= 0.0) {
        return err(line, ".tran: tmax должен быть положительным");
    }
    Ok(spec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_netlist() {
        let src = "Заголовок\n\
                   * комментарий\n\
                   V1 in 0 DC 5 ; встроенный комментарий\n\
                   R1 in out 1k\n\
                   C1 out GND 10uF IC=1\n\
                   + \n\
                   .tran 1u 1m uic\n\
                   .end\n\
                   R9 a b 1";
        let n = parse(src).unwrap();
        assert_eq!(n.title, "Заголовок");
        assert_eq!(n.elements.len(), 3);
        assert_eq!(n.elements[2].nodes, vec!["out", "0"]);
        assert_eq!(
            n.elements[2].kind,
            ElementKind::Capacitor {
                c: 10e-6,
                ic: Some(1.0)
            }
        );
        let t = n.tran().unwrap();
        assert!(t.uic && t.tstep == 1e-6 && t.tstop == 1e-3);
    }

    #[test]
    fn continuation_and_sources() {
        let src = "t\nV1 a 0 PULSE(0 5\n+ 1u 10n 10n 5u 20u)\nI1 0 b SIN(0 1m 1k)\nV2 c 0 PWL(0 0 1m 1 2m 0)\nV3 d 0 EXT 2.5\n";
        let n = parse(src).unwrap();
        match &n.elements[0].kind {
            ElementKind::VoltageSource(Source {
                wave: Some(Waveform::Pulse { v2, per, .. }),
                ..
            }) => assert!(*v2 == 5.0 && (*per - 20e-6).abs() < 1e-18),
            k => panic!("{k:?}"),
        }
        match &n.elements[3].kind {
            ElementKind::VoltageSource(Source {
                wave: Some(Waveform::Ext { initial }),
                ..
            }) => assert_eq!(*initial, 2.5),
            k => panic!("{k:?}"),
        }
    }

    #[test]
    fn models_and_switches() {
        let src = "t\nD1 a 0 dmod\n.model DMOD D(IS=1e-14 N=1.5 CJO=2p)\nS1 a b c 0 smod\nS2 a b EXT RON=0.01\n.model smod SW(RON=1 ROFF=1meg VT=2.5 VH=0.1)\nM1 d g 0 0 nm W=10u L=1u\n.model nm NMOS(VTO=1 KP=1e-3)\n";
        let n = parse(src).unwrap();
        assert_eq!(n.model("dmod").unwrap().get("n"), Some(1.5));
        assert!(n.warnings.iter().any(|w| w.contains("cjo")));
        match &n.elements[2].kind {
            ElementKind::Switch { control, ron, .. } => {
                assert_eq!(*control, SwitchControl::External);
                assert_eq!(*ron, Some(0.01));
            }
            k => panic!("{k:?}"),
        }
        assert_eq!(n.elements[1].nodes.len(), 4);
    }

    #[test]
    fn subcircuit_expansion() {
        let src = "t\n.subckt div top out\nR1 top out 1k\nR2 out 0 1k\n.ends\n.subckt two a b\nX1 a m div\nX2 m b div\n.ends\nV1 in 0 1\nXA in o two\n";
        let n = parse(src).unwrap();
        let names: Vec<&str> = n.elements.iter().map(|e| e.name.as_str()).collect();
        assert_eq!(
            names,
            vec!["v1", "xa.x1.r1", "xa.x1.r2", "xa.x2.r1", "xa.x2.r2"]
        );
        assert_eq!(n.elements[1].nodes, vec!["in", "xa.m"]);
        assert_eq!(n.elements[2].nodes, vec!["xa.m", "0"]);
        assert_eq!(n.elements[3].nodes, vec!["xa.m", "o"]);
    }

    #[test]
    fn errors_have_line_numbers() {
        let e = parse("t\nR1 a b 1k\nQ1 a b c\n").unwrap_err();
        assert_eq!(e.line, 3);
        let e = parse("t\nR1 a b xyz\n").unwrap_err();
        assert_eq!(e.line, 2);
        assert!(e.to_string().contains("не число"));
        let e = parse("t\nR1 a b 1\nR1 b 0 1\n").unwrap_err();
        assert_eq!(e.line, 3);
        let e = parse("t\n.param x=1\n").unwrap_err();
        assert!(e.message.contains("не поддерживается"));
        let e = parse("t\nX1 a b nosuch\n").unwrap_err();
        assert!(e.message.contains("не определена"));
    }
}
