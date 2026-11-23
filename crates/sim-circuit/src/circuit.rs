//! Схема, готовая к расчёту: узлы пронумерованы, модели подставлены, у каждого элемента —
//! индексы в векторах состояния решателя.
//!
//! Значения по умолчанию параметров моделей — как в SPICE3/ngspice:
//! * диод `D`: IS = 1e-14 А, N = 1, RS = 0;
//! * ключ `SW`: RON = 1 Ом, ROFF = 1e12 Ом, VT = 0, VH = 0 (включён при Vc > VT+VH,
//!   выключен при Vc < VT−VH, между порогами помнит состояние);
//! * МОП `NMOS/PMOS` уровня 1: VTO = 0, KP = 2e-5 А/В², LAMBDA = 0, IS переходов 1e-14 А,
//!   W = L = 100 мкм.
//!
//! Ёмкость перехода диода CJO и все ёмкости МОП не моделируются (предупреждение при разборе):
//! для схем силовой электроники на шагах ≥ 10 нс это обычно приемлемо, но выбросы и звон
//! на фронтах, которые дают эти ёмкости, решатель не покажет.

use std::collections::HashMap;

use crate::devices::{DiodeParams, MosParams, MosType, vcrit};
use crate::error::SimError;
use crate::netlist::{
    ElementKind, ModelCard, ModelKind, Netlist, SwitchControl, TranSpec, Waveform, parse,
};

/// Индекс узла: 0 — земля.
pub(crate) type Node = usize;

/// Источник: значение для рабочей точки, функция времени, индекс EXT-входа.
#[derive(Clone, Debug)]
pub(crate) struct SrcFn {
    pub dc: Option<f64>,
    pub wave: Option<Waveform>,
    pub ext: Option<usize>,
}

impl SrcFn {
    /// Значение в переходном процессе (EXT обрабатывает решатель).
    pub fn tran_value(&self, t: f64) -> f64 {
        match &self.wave {
            Some(w) => w.value(t),
            None => self.dc.unwrap_or(0.0),
        }
    }

    /// Значение для отдельного расчёта `.op` (как в SPICE: DC-значение, если задано).
    pub fn op_value(&self) -> f64 {
        match (self.dc, &self.wave) {
            (Some(v), _) => v,
            (None, Some(w)) => w.value(0.0),
            (None, None) => 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct DiodeDev {
    /// Внешний анод.
    pub a: Node,
    /// Анод самого перехода (внутренний узел при RS > 0).
    pub ja: Node,
    pub k: Node,
    pub p: DiodeParams,
    pub vcrit: f64,
    /// Индекс в векторе состояний переходов.
    pub j: usize,
}

#[derive(Clone, Debug)]
pub(crate) struct MosDev {
    pub d: Node,
    pub g: Node,
    pub s: Node,
    pub b: Node,
    pub p: MosParams,
    /// Ток насыщения переходов подложки, А.
    pub is: f64,
    pub vcrit: f64,
    /// Индекс в векторе состояний МОП.
    pub m: usize,
}

#[derive(Clone, Debug)]
pub(crate) struct SwitchDev {
    pub a: Node,
    pub b: Node,
    /// Узлы управления для ключа, управляемого напряжением; `None` — EXT.
    pub ctrl: Option<(Node, Node)>,
    pub ron: f64,
    pub roff: f64,
    pub vt: f64,
    pub vh: f64,
    pub initially_on: bool,
    /// Индекс в векторе состояний ключей.
    pub s: usize,
}

#[derive(Clone, Debug)]
pub(crate) enum Kind {
    Res {
        a: Node,
        b: Node,
        g: f64,
    },
    Cap {
        a: Node,
        b: Node,
        c: f64,
        ic: Option<f64>,
        k: usize,
    },
    Ind {
        a: Node,
        b: Node,
        l: f64,
        ic: Option<f64>,
        br: usize,
        k: usize,
    },
    Vsrc {
        a: Node,
        b: Node,
        src: SrcFn,
        br: usize,
    },
    Isrc {
        a: Node,
        b: Node,
        src: SrcFn,
    },
    Diode(DiodeDev),
    Switch(SwitchDev),
    Mos(MosDev),
    Vcvs {
        a: Node,
        b: Node,
        ca: Node,
        cb: Node,
        gain: f64,
        br: usize,
    },
    Vccs {
        a: Node,
        b: Node,
        ca: Node,
        cb: Node,
        gm: f64,
    },
}

#[derive(Clone, Debug)]
pub(crate) struct Elem {
    pub name: String,
    pub kind: Kind,
}

/// Схема, готовая к расчёту.
#[derive(Clone, Debug)]
pub struct Circuit {
    pub title: String,
    /// Имена узлов; индекс 0 — земля «0». Внутренние узлы содержат `#`.
    pub(crate) nodes: Vec<String>,
    pub(crate) node_index: HashMap<String, Node>,
    pub(crate) elems: Vec<Elem>,
    pub(crate) elem_index: HashMap<String, usize>,
    pub(crate) n_branches: usize,
    pub(crate) n_caps: usize,
    pub(crate) n_inds: usize,
    pub(crate) n_junctions: usize,
    pub(crate) n_mos: usize,
    pub(crate) n_switches: usize,
    pub(crate) n_ext: usize,
    pub(crate) tran: Option<TranSpec>,
    pub(crate) has_op: bool,
    /// Предупреждения разбора и построения.
    pub warnings: Vec<String>,
}

fn model_param(m: Option<&ModelCard>, key: &str, default: f64) -> f64 {
    m.and_then(|m| m.get(key)).unwrap_or(default)
}

impl Circuit {
    /// Разобрать текст нетлиста и построить схему.
    pub fn parse(text: &str) -> Result<Circuit, SimError> {
        Circuit::from_netlist(&parse(text)?)
    }

    /// Построить схему из разобранного нетлиста.
    pub fn from_netlist(net: &Netlist) -> Result<Circuit, SimError> {
        let mut c = Circuit {
            title: net.title.clone(),
            nodes: vec!["0".to_string()],
            node_index: HashMap::from([("0".to_string(), 0)]),
            elems: Vec::with_capacity(net.elements.len()),
            elem_index: HashMap::new(),
            n_branches: 0,
            n_caps: 0,
            n_inds: 0,
            n_junctions: 0,
            n_mos: 0,
            n_switches: 0,
            n_ext: 0,
            tran: net.tran().copied(),
            has_op: net
                .analyses
                .iter()
                .any(|a| matches!(a, crate::netlist::Analysis::Op)),
            warnings: net.warnings.clone(),
        };
        let mut touches_ground = false;
        for e in &net.elements {
            let fail = |message: String| SimError::Element {
                name: e.name.clone(),
                line: e.line,
                message,
            };
            let nodes: Vec<Node> = e.nodes.iter().map(|n| c.node(n)).collect();
            touches_ground |= nodes.contains(&0);
            let find_model = |kinds: &[ModelKind], name: &str| -> Result<&ModelCard, SimError> {
                let m = net
                    .model(name)
                    .ok_or_else(|| fail(format!("модель «{name}» не найдена (.model)")))?;
                if !kinds.contains(&m.kind) {
                    return Err(fail(format!("модель «{name}» не того типа ({:?})", m.kind)));
                }
                Ok(m)
            };
            let kind = match &e.kind {
                ElementKind::Resistor { r } => Kind::Res {
                    a: nodes[0],
                    b: nodes[1],
                    g: 1.0 / r,
                },
                ElementKind::Capacitor { c: cap, ic } => {
                    c.n_caps += 1;
                    Kind::Cap {
                        a: nodes[0],
                        b: nodes[1],
                        c: *cap,
                        ic: *ic,
                        k: c.n_caps - 1,
                    }
                }
                ElementKind::Inductor { l, ic } => {
                    c.n_inds += 1;
                    c.n_branches += 1;
                    Kind::Ind {
                        a: nodes[0],
                        b: nodes[1],
                        l: *l,
                        ic: *ic,
                        br: c.n_branches - 1,
                        k: c.n_inds - 1,
                    }
                }
                ElementKind::VoltageSource(s) | ElementKind::CurrentSource(s) => {
                    let ext = if matches!(s.wave, Some(Waveform::Ext { .. })) {
                        c.n_ext += 1;
                        Some(c.n_ext - 1)
                    } else {
                        None
                    };
                    let src = SrcFn {
                        dc: s.dc,
                        wave: s.wave.clone(),
                        ext,
                    };
                    if matches!(e.kind, ElementKind::VoltageSource(_)) {
                        c.n_branches += 1;
                        Kind::Vsrc {
                            a: nodes[0],
                            b: nodes[1],
                            src,
                            br: c.n_branches - 1,
                        }
                    } else {
                        Kind::Isrc {
                            a: nodes[0],
                            b: nodes[1],
                            src,
                        }
                    }
                }
                ElementKind::Diode { model, area } => {
                    let m = find_model(&[ModelKind::Diode], model)?;
                    let p = DiodeParams {
                        is: model_param(Some(m), "is", 1e-14) * area,
                        n: model_param(Some(m), "n", 1.0),
                        rs: model_param(Some(m), "rs", 0.0) / area,
                    };
                    if p.is <= 0.0 || p.n <= 0.0 || p.rs < 0.0 {
                        return Err(fail("у диода IS и N должны быть > 0, RS ≥ 0".into()));
                    }
                    let ja = if p.rs > 0.0 {
                        c.node(&format!("{}#rs", e.name))
                    } else {
                        nodes[0]
                    };
                    c.n_junctions += 1;
                    Kind::Diode(DiodeDev {
                        a: nodes[0],
                        ja,
                        k: nodes[1],
                        p,
                        vcrit: vcrit(p.is, p.nvt()),
                        j: c.n_junctions - 1,
                    })
                }
                ElementKind::Switch {
                    control,
                    model,
                    initially_on,
                    ron,
                    roff,
                } => {
                    let m = match model {
                        Some(name) => Some(find_model(&[ModelKind::Switch], name)?),
                        None => None,
                    };
                    let ron = ron.unwrap_or(model_param(m, "ron", 1.0));
                    let roff = roff.unwrap_or(model_param(m, "roff", 1e12));
                    if ron <= 0.0 || roff <= 0.0 {
                        return Err(fail("RON и ROFF должны быть положительными".into()));
                    }
                    c.n_switches += 1;
                    Kind::Switch(SwitchDev {
                        a: nodes[0],
                        b: nodes[1],
                        ctrl: match control {
                            SwitchControl::Voltage => Some((nodes[2], nodes[3])),
                            SwitchControl::External => None,
                        },
                        ron,
                        roff,
                        vt: model_param(m, "vt", 0.0),
                        vh: model_param(m, "vh", 0.0).abs(),
                        initially_on: *initially_on,
                        s: c.n_switches - 1,
                    })
                }
                ElementKind::Mosfet { model, w, l } => {
                    let m = find_model(&[ModelKind::Nmos, ModelKind::Pmos], model)?;
                    let kind = if m.kind == ModelKind::Nmos {
                        MosType::N
                    } else {
                        MosType::P
                    };
                    let w = w.unwrap_or(100e-6);
                    let l = l.unwrap_or(100e-6);
                    let kp = model_param(Some(m), "kp", 2e-5);
                    let is = model_param(Some(m), "is", 1e-14);
                    if kp <= 0.0 || is <= 0.0 {
                        return Err(fail("KP и IS должны быть положительными".into()));
                    }
                    c.n_mos += 1;
                    Kind::Mos(MosDev {
                        d: nodes[0],
                        g: nodes[1],
                        s: nodes[2],
                        b: nodes[3],
                        p: MosParams {
                            kind,
                            vto: model_param(Some(m), "vto", 0.0),
                            beta: kp * w / l,
                            lambda: model_param(Some(m), "lambda", 0.0),
                        },
                        is,
                        vcrit: vcrit(is, crate::devices::thermal_voltage()),
                        m: c.n_mos - 1,
                    })
                }
                ElementKind::Vcvs { gain } => {
                    c.n_branches += 1;
                    Kind::Vcvs {
                        a: nodes[0],
                        b: nodes[1],
                        ca: nodes[2],
                        cb: nodes[3],
                        gain: *gain,
                        br: c.n_branches - 1,
                    }
                }
                ElementKind::Vccs { gm } => Kind::Vccs {
                    a: nodes[0],
                    b: nodes[1],
                    ca: nodes[2],
                    cb: nodes[3],
                    gm: *gm,
                },
            };
            c.elem_index.insert(e.name.clone(), c.elems.len());
            c.elems.push(Elem {
                name: e.name.clone(),
                kind,
            });
        }
        if c.elems.is_empty() {
            return Err(SimError::Invalid("в схеме нет ни одного элемента".into()));
        }
        if !touches_ground {
            return Err(SimError::Invalid(
                "ни один элемент не подключён к земле (узел 0 или gnd)".into(),
            ));
        }
        Ok(c)
    }

    fn node(&mut self, name: &str) -> Node {
        if let Some(&i) = self.node_index.get(name) {
            return i;
        }
        let i = self.nodes.len();
        self.nodes.push(name.to_string());
        self.node_index.insert(name.to_string(), i);
        i
    }

    /// Индекс узла по имени (регистр не важен, `gnd` = `0`).
    pub(crate) fn find_node(&self, name: &str) -> Result<Node, SimError> {
        let n = name.trim().to_lowercase();
        let n = if n == "gnd" { "0".to_string() } else { n };
        self.node_index
            .get(&n)
            .copied()
            .ok_or(SimError::UnknownNode(name.to_string()))
    }

    pub(crate) fn find_elem(&self, name: &str) -> Result<&Elem, SimError> {
        self.elem_index
            .get(&name.trim().to_lowercase())
            .map(|&i| &self.elems[i])
            .ok_or(SimError::UnknownElement(name.to_string()))
    }

    /// Видимые узлы (без земли и внутренних узлов приборов).
    pub fn node_names(&self) -> impl Iterator<Item = &str> {
        self.nodes[1..]
            .iter()
            .map(String::as_str)
            .filter(|n| !n.contains('#'))
    }

    /// Имена элементов в порядке нетлиста.
    pub fn element_names(&self) -> impl Iterator<Item = &str> {
        self.elems.iter().map(|e| e.name.as_str())
    }

    /// Параметры `.tran` из нетлиста.
    pub fn tran_spec(&self) -> Option<TranSpec> {
        self.tran
    }

    /// Есть ли в нетлисте `.op`.
    pub fn has_op(&self) -> bool {
        self.has_op
    }

    /// Есть ли нелинейные приборы (диоды, МОП): тогда на каждом шаге итерации Ньютона.
    pub fn is_nonlinear(&self) -> bool {
        self.n_junctions > 0 || self.n_mos > 0
    }

    /// Число неизвестных MNA (узлы без земли + токи ветвей).
    pub fn unknowns(&self) -> usize {
        self.nodes.len() - 1 + self.n_branches
    }
}
