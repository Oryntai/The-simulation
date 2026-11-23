//! Сценарий «Лаборатория схем»: нетлист (подмножество SPICE) редактируется прямо в окне,
//! считается нашим решателем (`sim-circuit`: узловой анализ, Ньютон, трапеции) и — по
//! кнопке — внешней программой ngspice на том же тексте. Кривые и их расхождение — рядом.
//! На плате в 3D детали светятся по рассеиваемой мощности в момент курсора времени.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use sim_circuit::netlist::ElementKind;
use sim_circuit::{Circuit, Waveforms, examples, operating_point, run_tran};
use sim_core::{Pose, Shape, Vec3};

use crate::scenario::*;

type Slot<T> = Arc<Mutex<Option<T>>>;

pub fn ngspice_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/third_party/ngspice/bin/ngspice")
}

/// Деталь на плате для 3D.
#[derive(Clone, Debug)]
struct Part {
    name: String,
    kind: char,
    nodes: Vec<String>,
    /// Сопротивление резистора, Ом (для мощности Δv²/R).
    r: Option<f64>,
}

/// Результат ngspice: сигнал → (время, значения).
type NgResult = Result<Vec<(String, Vec<f64>, Vec<f64>)>, String>;

pub struct CircuitLab {
    example: usize,
    text: String,
    parts: Vec<Part>,
    result: Option<Result<Waveforms, String>>,
    pending: Option<Slot<Result<Waveforms, String>>>,
    ng: Option<NgResult>,
    ng_pending: Option<Slot<NgResult>>,
    cursor: f64,
    log: Vec<String>,
    epoch: u64,
    t: f64,
}

impl Default for CircuitLab {
    fn default() -> Self {
        let mut s = Self {
            example: 0,
            text: String::new(),
            parts: Vec::new(),
            result: None,
            pending: None,
            ng: None,
            ng_pending: None,
            cursor: 0.0,
            log: Vec::new(),
            epoch: 0,
            t: 0.0,
        };
        s.load_example(0);
        s
    }
}

impl CircuitLab {
    fn load_example(&mut self, i: usize) {
        let ex = examples::all();
        self.example = i.min(ex.len() - 1);
        self.text = ex[self.example].netlist.to_string();
        self.start();
    }

    /// Запустить расчёт текущего текста в фоне.
    fn start(&mut self) {
        self.epoch += 1;
        self.ng = None;
        self.ng_pending = None;
        self.parts = match sim_circuit::parse(&self.text) {
            Ok(n) => n
                .elements
                .iter()
                .map(|e| Part {
                    name: e.name.clone(),
                    nodes: e.nodes.clone(),
                    r: match &e.kind {
                        ElementKind::Resistor { r } => Some(*r),
                        _ => None,
                    },
                    kind: match &e.kind {
                        ElementKind::Resistor { .. } => 'r',
                        ElementKind::Capacitor { .. } => 'c',
                        ElementKind::Inductor { .. } => 'l',
                        ElementKind::VoltageSource(_) => 'v',
                        ElementKind::CurrentSource(_) => 'i',
                        ElementKind::Diode { .. } => 'd',
                        _ => e.name.chars().next().unwrap_or('x'),
                    },
                })
                .collect(),
            Err(_) => Vec::new(),
        };
        let slot: Slot<Result<Waveforms, String>> = Arc::new(Mutex::new(None));
        let out = slot.clone();
        let text = self.text.clone();
        std::thread::spawn(move || {
            let r = Circuit::parse(&text)
                .map_err(|e| e.to_string())
                .and_then(|c| {
                    if c.tran_spec().is_some() {
                        run_tran(&c).map_err(|e| e.to_string())
                    } else {
                        // Только рабочая точка (.op): один «отсчёт» в момент 0.
                        operating_point(&c)
                            .map_err(|e| e.to_string())
                            .map(|op| Waveforms {
                                time: vec![0.0],
                                signals: op
                                    .signals()
                                    .into_iter()
                                    .map(|(n, v)| (n, vec![v]))
                                    .collect(),
                            })
                    }
                });
            if let Ok(mut g) = out.lock() {
                *g = Some(r);
            }
        });
        self.pending = Some(slot);
        self.result = None;
        self.log.push("расчёт запущен…".into());
    }

    fn start_ngspice(&mut self) {
        let Some(Ok(w)) = &self.result else {
            self.log
                .push("сначала нужен успешный расчёт нашим решателем".into());
            return;
        };
        let exe = ngspice_path();
        if !exe.exists() {
            self.ng = Some(Err(format!("ngspice не найден: {}", exe.display())));
            return;
        }
        let signals: Vec<String> = self.interesting_nodes(w).into_iter().take(6).collect();
        let text = self.text.clone();
        let op_only = w.len() == 1;
        let slot: Slot<NgResult> = Arc::new(Mutex::new(None));
        let out = slot.clone();
        std::thread::spawn(move || {
            let r = if op_only {
                run_ngspice_op(&exe, &text)
            } else {
                run_ngspice(&exe, &text, &signals)
            };
            if let Ok(mut g) = out.lock() {
                *g = Some(r);
            }
        });
        self.ng_pending = Some(slot);
        self.log.push("ngspice: запущен на том же нетлисте…".into());
    }

    fn poll(&mut self) {
        if let Some(slot) = &self.pending {
            let done = slot.lock().ok().and_then(|mut g| g.take());
            if let Some(r) = done {
                self.pending = None;
                match &r {
                    Ok(w) => self.log.push(format!(
                        "готово: {} отсчётов, {} сигналов",
                        w.len(),
                        w.signals.len()
                    )),
                    Err(e) => self.log.push(format!("ошибка: {e}")),
                }
                self.result = Some(r);
                self.cursor = 0.0;
            }
        }
        if let Some(slot) = &self.ng_pending {
            let done = slot.lock().ok().and_then(|mut g| g.take());
            if let Some(r) = done {
                self.ng_pending = None;
                match &r {
                    Ok(v) => self
                        .log
                        .push(format!("ngspice: готово, сравниваю {} сигналов", v.len())),
                    Err(e) => self.log.push(format!("ngspice: {e}")),
                }
                self.ng = Some(r);
            }
        }
    }

    fn waves(&self) -> Option<&Waveforms> {
        match &self.result {
            Some(Ok(w)) => Some(w),
            _ => None,
        }
    }

    /// Время курсора в секундах (бегает по длительности расчёта).
    fn cursor_time(&self) -> f64 {
        self.waves()
            .and_then(|w| w.time.last().map(|t| t * self.cursor))
            .unwrap_or(0.0)
    }

    /// Напряжение узла в момент `t` (земля — 0).
    fn node_v(&self, w: &Waveforms, node: &str, t: f64) -> Option<f64> {
        let n = node.to_lowercase();
        if n == "0" || n == "gnd" {
            return Some(0.0);
        }
        w.at(&format!("v({n})"), t)
    }

    /// Мощность детали в момент курсора, Вт: резистор — Δv²/R (рассеивает), источник
    /// напряжения — v·i (по знаку: отдаёт в схему — отрицательная).
    fn part_power(&self, part: &Part) -> Option<f64> {
        let w = self.waves()?;
        let t = self.cursor_time();
        let dv = self.node_v(w, part.nodes.first()?, t)? - self.node_v(w, part.nodes.get(1)?, t)?;
        match (part.kind, part.r) {
            ('r', Some(r)) if r > 0.0 => Some(dv * dv / r),
            ('v', _) => Some(dv * w.at(&format!("i({})", part.name), t)?),
            _ => None,
        }
    }

    /// Напряжения узлов, интересные для графика и сверки: без узлов, которые просто заданы
    /// источником напряжения относительно земли (их значение известно заранее).
    fn interesting_nodes(&self, w: &Waveforms) -> Vec<String> {
        let driven: Vec<String> = self
            .parts
            .iter()
            .filter(|p| p.kind == 'v' && p.nodes.get(1).is_some_and(|n| n == "0" || n == "gnd"))
            .filter_map(|p| p.nodes.first().map(|n| format!("v({})", n.to_lowercase())))
            .collect();
        let all: Vec<String> = w
            .signals
            .iter()
            .map(|(n, _)| n.clone())
            .filter(|n| n.starts_with("v("))
            .collect();
        let inner: Vec<String> = all
            .iter()
            .filter(|n| !driven.contains(n))
            .cloned()
            .collect();
        if inner.is_empty() { all } else { inner }
    }

    /// Это ли сейчас по-прежнему тот пример (для сверок с аналитикой).
    fn is_pristine_example(&self) -> bool {
        self.text == examples::all()[self.example].netlist
    }
}

/// Перевод нашего нетлиста в понятный ngspice: источники `EXT v` → `DC v` (при постоянном
/// значении это одно и то же), ключи `S a b EXT …` → резистор с сопротивлением ключа в его
/// начальном состоянии (ROFF, или RON при флаге ON).
fn ngspice_compatible(text: &str) -> String {
    let mut out = String::new();
    for (k, line) in text.lines().enumerate() {
        let toks: Vec<&str> = line.split_whitespace().collect();
        let low = line.trim().to_lowercase();
        if k > 0 && low.starts_with('s') && toks.len() >= 4 && toks[3].eq_ignore_ascii_case("ext") {
            let mut ron = 1.0;
            let mut roff = 1e12;
            let mut on = false;
            for t in &toks[4..] {
                let tl = t.to_lowercase();
                if let Some(v) = tl.strip_prefix("ron=") {
                    ron = sim_circuit::units::parse_value(v).unwrap_or(ron);
                } else if let Some(v) = tl.strip_prefix("roff=") {
                    roff = sim_circuit::units::parse_value(v).unwrap_or(roff);
                } else if tl == "on" {
                    on = true;
                }
            }
            out += &format!(
                "R{} {} {} {:e}\n",
                toks[0],
                toks[1],
                toks[2],
                if on { ron } else { roff }
            );
        } else if k > 0 && (low.starts_with('v') || low.starts_with('i')) && low.contains(" ext ") {
            let fixed: Vec<String> = toks
                .iter()
                .map(|t| {
                    if t.eq_ignore_ascii_case("ext") {
                        "DC".to_string()
                    } else {
                        t.to_string()
                    }
                })
                .collect();
            out += &fixed.join(" ");
            out.push('\n');
        } else {
            out += line;
            out.push('\n');
        }
    }
    out
}

/// Рабочая точка в ngspice: `op` + `print all`, разбор строк «имя = значение».
fn run_ngspice_op(exe: &PathBuf, text: &str) -> NgResult {
    let dir = std::env::temp_dir().join(format!("workshop-ngspice-op-{}", std::process::id()));
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let cir = dir.join("circuit.cir");
    let text = ngspice_compatible(text);
    let body: String = text
        .lines()
        .filter(|l| {
            let t = l.trim().to_lowercase();
            t != ".end" && t != ".op"
        })
        .map(|l| format!("{l}\n"))
        .collect();
    std::fs::write(
        &cir,
        format!("{body}.control\nop\nprint all\n.endc\n.end\n"),
    )
    .map_err(|e| e.to_string())?;
    let out = std::process::Command::new(exe)
        .arg("-b")
        .arg(&cir)
        .output()
        .map_err(|e| e.to_string())?;
    let _ = std::fs::remove_dir_all(&dir);
    let stdout = String::from_utf8_lossy(&out.stdout);
    let mut v = Vec::new();
    for line in stdout.lines() {
        let Some((name, val)) = line.split_once('=') else {
            continue;
        };
        let (name, val) = (name.trim().to_lowercase(), val.trim());
        let Ok(x) = val.parse::<f64>() else { continue };
        let key = if let Some(src) = name.strip_suffix("#branch") {
            format!("i({src})")
        } else if name.starts_with("v(") || name.starts_with("i(") {
            name
        } else {
            format!("v({name})")
        };
        v.push((key, vec![0.0], vec![x]));
    }
    if v.is_empty() {
        return Err("ngspice не напечатал рабочую точку".into());
    }
    Ok(v)
}

/// Запуск ngspice в пакетном режиме: нетлист + блок .control c `wrdata`.
fn run_ngspice(exe: &PathBuf, text: &str, signals: &[String]) -> NgResult {
    let dir = std::env::temp_dir().join(format!("workshop-ngspice-{}", std::process::id()));
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let cir = dir.join("circuit.cir");
    let data = dir.join("out.txt");
    let text = ngspice_compatible(text);
    let body: String = text
        .lines()
        .filter(|l| !l.trim().eq_ignore_ascii_case(".end"))
        .map(|l| format!("{l}\n"))
        .collect();
    let deck = format!(
        "{body}.control\nset wr_singlescale\nset wr_vecnames\nrun\nwrdata {} {}\n.endc\n.end\n",
        data.display(),
        signals.join(" ")
    );
    std::fs::write(&cir, deck).map_err(|e| e.to_string())?;
    let out = std::process::Command::new(exe)
        .arg("-b")
        .arg(&cir)
        .output()
        .map_err(|e| e.to_string())?;
    let read = std::fs::read_to_string(&data);
    let _ = std::fs::remove_dir_all(&dir);
    let txt = read.map_err(|_| {
        let err = String::from_utf8_lossy(&out.stdout).to_string()
            + &String::from_utf8_lossy(&out.stderr);
        let first = err
            .lines()
            .find(|l| l.to_lowercase().contains("error"))
            .unwrap_or("нет данных")
            .to_string();
        format!("ngspice не дал данных: {first}")
    })?;
    let mut lines = txt.lines();
    let header: Vec<String> = lines
        .next()
        .unwrap_or("")
        .split_whitespace()
        .map(|s| s.to_lowercase())
        .collect();
    let mut cols: Vec<Vec<f64>> = vec![Vec::new(); header.len()];
    for l in lines {
        let vals: Vec<f64> = l
            .split_whitespace()
            .filter_map(|x| x.parse().ok())
            .collect();
        if vals.len() == header.len() {
            for (c, v) in cols.iter_mut().zip(vals) {
                c.push(v);
            }
        }
    }
    if header.len() < 2 || cols[0].is_empty() {
        return Err("пустой вывод ngspice".into());
    }
    let time = cols[0].clone();
    Ok(header
        .iter()
        .zip(cols.iter())
        .skip(1)
        .map(|(n, v)| (n.clone(), time.clone(), v.clone()))
        .collect())
}

/// Линейная интерполяция таблицы (t, y) в момент `t` (None — вне диапазона).
fn interp(ts: &[f64], ys: &[f64], t: f64) -> Option<f64> {
    if ts.is_empty() || t < ts[0] || t > ts[ts.len() - 1] {
        return None;
    }
    let k = ts.partition_point(|&x| x < t);
    if k == 0 {
        return Some(ys[0]);
    }
    let (t0, t1) = (ts[k - 1], ts[k]);
    if t1 <= t0 {
        return Some(ys[k]);
    }
    Some(ys[k - 1] + (ys[k] - ys[k - 1]) * (t - t0) / (t1 - t0))
}

const PLOT_COLORS: [[f32; 4]; 6] = [
    C_YELLOW,
    C_BLUE,
    C_RED,
    C_GREEN,
    [0.8, 0.5, 1.0, 1.0],
    C_GRAY,
];

impl Scenario for CircuitLab {
    fn id(&self) -> &'static str {
        "circuit_lab"
    }

    fn title(&self) -> String {
        "Лаборатория схем: редактор и ngspice".into()
    }

    fn summary(&self) -> String {
        "Схема пишется текстом (подмножество SPICE) прямо здесь и считается нашим решателем; \
         кнопка «Сверить с ngspice» прогоняет тот же текст во внешней программе. На плате \
         детали светятся по мощности в бегущий момент времени."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        Vec::new()
    }

    fn set_param(&mut self, _id: &str, _value: f64) {}

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            id: "example",
            label: "Пример".into(),
            options: examples::all()
                .iter()
                .map(|e| e.title.to_string())
                .collect(),
            selected: self.example,
        }]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "example" {
            self.load_example(index);
        }
    }

    fn text_fields(&self) -> Vec<TextField> {
        vec![TextField {
            id: "netlist",
            label: "Нетлист (SPICE): R, C, L, V, I, D, S, M, E, G, .model, .subckt, .tran".into(),
            text: self.text.clone(),
            rows: 14,
        }]
    }

    fn set_text(&mut self, id: &str, text: String) {
        if id == "netlist" {
            self.text = text;
            self.start();
        }
    }

    fn actions(&self) -> Vec<Action> {
        vec![
            Action::new(
                "ngspice",
                if self.ng_pending.is_some() {
                    "ngspice считает…"
                } else {
                    "Сверить с ngspice"
                },
                "Тот же нетлист во внешней программе ngspice 47 (BSD), сравнение кривых",
                self.ng_pending.is_some(),
            ),
            Action::new(
                "restore",
                "Вернуть пример",
                "Отменить правки нетлиста",
                false,
            ),
        ]
    }

    fn run_action(&mut self, id: &str) {
        match id {
            "ngspice" => {
                if self.ng_pending.is_none() {
                    self.start_ngspice();
                }
            }
            "restore" => self.load_example(self.example),
            _ => {}
        }
    }

    fn reset(&mut self) {
        self.cursor = 0.0;
        self.t = 0.0;
    }

    fn advance(&mut self, dt_real: f64) {
        self.poll();
        self.t += dt_real;
        // Курсор пробегает весь расчёт за 6 с.
        self.cursor = (self.cursor + dt_real / 6.0) % 1.0;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let n = self.parts.len().max(1);
        let cols = 4usize;
        let rows = n.div_ceil(cols);
        let pitch = 0.11;
        let w = cols as f64 * pitch;
        let h = rows as f64 * pitch;
        let z = 0.9;
        let mut v = vec![
            VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(w / 2.0 + 0.03, h / 2.0 + 0.03, 0.004),
                },
                pose: Pose::at(Vec3::new(0.0, 0.0, z)),
                color: [0.08, 0.4, 0.18, 1.0],
            },
            VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.03,
                    length: z - 0.004,
                },
                pose: Pose::at(Vec3::new(0.0, 0.0, (z - 0.004) / 2.0)),
                color: C_GRAY,
            },
        ];
        let pmax = self
            .parts
            .iter()
            .filter_map(|p| self.part_power(p))
            .map(f64::abs)
            .fold(1e-9, f64::max);
        for (k, p) in self.parts.iter().enumerate() {
            let (r, c) = (k / cols, k % cols);
            let pos = Vec3::new(
                -w / 2.0 + pitch * (c as f64 + 0.5),
                -h / 2.0 + pitch * (r as f64 + 0.5),
                z + 0.004,
            );
            let heat = self
                .part_power(p)
                .map(|x| (x.abs() / pmax) as f32)
                .unwrap_or(0.0);
            let base: [f32; 4] = match p.kind {
                'r' => [0.85, 0.75, 0.55, 1.0],
                'c' => [0.25, 0.4, 0.8, 1.0],
                'l' => [0.75, 0.45, 0.2, 1.0],
                'v' => [0.15, 0.15, 0.2, 1.0],
                'd' => [0.1, 0.1, 0.1, 1.0],
                'm' => [0.12, 0.12, 0.12, 1.0],
                _ => [0.5, 0.5, 0.55, 1.0],
            };
            let color = [
                base[0] + (1.0 - base[0]) * heat,
                base[1] * (1.0 - 0.7 * heat),
                base[2] * (1.0 - 0.8 * heat),
                1.0,
            ];
            let (shape, lift) = match p.kind {
                'r' => (
                    Shape::Box {
                        half: Vec3::new(0.03, 0.008, 0.008),
                    },
                    0.008,
                ),
                'c' => (
                    Shape::Cylinder {
                        radius: 0.012,
                        length: 0.03,
                    },
                    0.015,
                ),
                'l' => (
                    Shape::Cylinder {
                        radius: 0.022,
                        length: 0.02,
                    },
                    0.01,
                ),
                'v' => (
                    Shape::Box {
                        half: Vec3::new(0.03, 0.02, 0.02),
                    },
                    0.02,
                ),
                'm' => (
                    Shape::Box {
                        half: Vec3::new(0.012, 0.005, 0.02),
                    },
                    0.02,
                ),
                _ => (
                    Shape::Box {
                        half: Vec3::new(0.012, 0.008, 0.006),
                    },
                    0.006,
                ),
            };
            v.push(VisualInstance {
                shape,
                pose: Pose::at(pos + Vec3::z() * lift),
                color,
            });
        }
        v
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn readouts(&self) -> Vec<Readout> {
        let mut r = Vec::new();
        let Some(w) = self.waves() else {
            return r;
        };
        let t = self.cursor_time();
        r.push(Readout::new("Момент на курсоре", t * 1000.0, "мс", 3));
        for (n, _) in w
            .signals
            .iter()
            .filter(|(n, _)| n.starts_with("v("))
            .take(8)
        {
            if let Some(v) = w.at(n, t) {
                r.push(Readout::new(n, v, "В", 4));
            }
        }
        for p in &self.parts {
            if let Some(pw) = self.part_power(p) {
                r.push(Readout::new(
                    &format!("мощность {}", p.name),
                    pw * 1000.0,
                    "мВт",
                    2,
                ));
            }
        }
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let mut v = Vec::new();
        let (Some(w), Some(Ok(ng))) = (self.waves(), &self.ng) else {
            if let Some(Err(e)) = &self.ng {
                v.push(Comparison {
                    title: "Сверка с ngspice".into(),
                    sim_label: "наш решатель".into(),
                    sim: f64::NAN,
                    ref_label: "ngspice".into(),
                    reference: f64::NAN,
                    unit: "",
                    decimals: 0,
                    status: Status::Pending,
                    explanation: e.clone(),
                });
            }
            return v;
        };
        for (name, nt, nv) in ng {
            let Some(_) = w.get(name) else { continue };
            // Сравниваем на нашей сетке вывода: ngspice шагает мельче (дробит шаг у изломов),
            // поэтому его кривую интерполируем в наши моменты, а не наоборот.
            let ours = w.get(name).unwrap_or(&[]);
            let mut max_err: f64 = 0.0;
            let mut span: f64 = 0.0;
            for (t, y) in w.time.iter().zip(ours) {
                if let Some(theirs) = interp(nt, nv, *t) {
                    max_err = max_err.max((theirs - y).abs());
                    span = span.max(y.abs());
                }
            }
            let rel = max_err / span.max(1e-9);
            let last_t = *nt.last().unwrap_or(&0.0);
            let is_op = w.len() == 1;
            v.push(Comparison {
                title: if is_op {
                    format!("{name}: рабочая точка против ngspice")
                } else {
                    format!("{name}: вся кривая против ngspice")
                },
                sim_label: "наш решатель, в конце".into(),
                sim: w.at(name, last_t).unwrap_or(f64::NAN),
                ref_label: "ngspice, в конце".into(),
                reference: *nv.last().unwrap_or(&f64::NAN),
                unit: if name.starts_with("i(") { "А" } else { "В" },
                decimals: 6,
                status: if rel < 0.01 {
                    Status::Ok
                } else if rel < 0.05 {
                    Status::Explained
                } else {
                    Status::Mismatch
                },
                explanation: format!(
                    "Наибольшее отклонение по всей кривой {:.3} мВ ({:.2} % размаха). Разные шаги \
                     и методы интегрирования дают доли процента на фронтах ШИМ.",
                    max_err * 1000.0,
                    100.0 * rel
                ),
            });
        }
        v
    }

    fn notes(&self) -> Vec<Note> {
        let mut n = Vec::new();
        if self.is_pristine_example() {
            n.push(Note::new(
                Level::Info,
                examples::all()[self.example].description,
            ));
        } else {
            n.push(Note::new(Level::Info, "Нетлист изменён — это ваша схема."));
        }
        match &self.result {
            Some(Err(e)) => n.push(Note::new(Level::Danger, format!("Схема не считается: {e}"))),
            None => n.push(Note::new(Level::Info, "Считаю…")),
            _ => {}
        }
        if self.ng.is_none() && self.ng_pending.is_none() && self.waves().is_some() {
            n.push(Note::new(
                Level::Info,
                "Нажмите «Сверить с ngspice», чтобы прогнать тот же текст во внешнем эталоне.",
            ));
        }
        n
    }

    fn plot(&self) -> Option<Plot> {
        let w = self.waves().filter(|w| w.len() > 1)?;
        let names: Vec<String> = self.interesting_nodes(w).into_iter().take(4).collect();
        let stride = (w.len() / 800).max(1);
        let mut series: Vec<Series> = names
            .iter()
            .enumerate()
            .map(|(k, n)| Series {
                name: n.to_string(),
                points: w
                    .time
                    .iter()
                    .zip(w.get(n).unwrap_or(&[]))
                    .step_by(stride)
                    .map(|(t, y)| [t * 1000.0, *y])
                    .collect(),
                color: PLOT_COLORS[k % PLOT_COLORS.len()],
                style: SeriesStyle::Solid,
            })
            .collect();
        if let Some(Ok(ng)) = &self.ng {
            for (k, n) in names.iter().enumerate() {
                if let Some((_, t, y)) = ng.iter().find(|(m, _, _)| m == n) {
                    let st = (t.len() / 400).max(1);
                    series.push(Series {
                        name: format!("{n} ngspice"),
                        points: t
                            .iter()
                            .zip(y)
                            .step_by(st)
                            .map(|(a, b)| [a * 1000.0, *b])
                            .collect(),
                        color: PLOT_COLORS[k % PLOT_COLORS.len()],
                        style: SeriesStyle::Dashed,
                    });
                }
            }
        }
        let tc = self.cursor_time() * 1000.0;
        let (lo, hi) = series
            .iter()
            .flat_map(|s| s.points.iter().map(|p| p[1]))
            .fold((0.0f64, 0.0f64), |(a, b), y| (a.min(y), b.max(y)));
        series.push(Series {
            name: "курсор".into(),
            points: vec![[tc, lo], [tc, hi]],
            color: [1.0, 1.0, 1.0, 0.5],
            style: SeriesStyle::Dashed,
        });
        Some(Plot {
            title: "Напряжения узлов".into(),
            x_label: "мс".into(),
            y_label: "В".into(),
            series,
            bands: vec![],
        })
    }

    fn console(&self) -> Vec<String> {
        self.log.iter().rev().take(40).rev().cloned().collect()
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.9)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(0.15, -0.55, 0.45)
    }
}
