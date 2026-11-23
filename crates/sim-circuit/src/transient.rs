//! Переходный процесс: пошаговый API для совместного моделирования и пакетный `.tran`.
//!
//! Пошаговый режим (механика/прошивка в цикле хоста):
//! ```
//! use sim_circuit::{Circuit, Transient};
//! let ckt = Circuit::parse("мотор\nV1 vcc 0 12\nS1 vcc m EXT RON=0.01\nRm m e 1\nLm e b 1m\nVbemf b 0 EXT 0\n").unwrap();
//! let mut tr = Transient::new(&ckt, 1e-6).unwrap();
//! tr.set_switch("S1", true).unwrap();
//! for _ in 0..100 {
//!     tr.set_source("Vbemf", 2.0).unwrap(); // противо-ЭДС от механики
//!     tr.step().unwrap();
//! }
//! let i = tr.current("Lm").unwrap();
//! assert!(i > 0.0);
//! ```
//!
//! Семантика внешних входов: значение `set_source` относится к концу следующего шага, внутри
//! шага — линейная интерполяция от прошлого значения (для трапеций это точно соответствует
//! кусочно-линейному входу). `set_switch` — состояние на весь следующий шаг (как ШИМ из
//! прошивки с удержанием), после переключения шаг делается неявным Эйлером.
//!
//! Внутренний шаг: изломы PULSE/PWL/SIN решатель проходит точно; после излома — шаг Эйлера
//! длиной 0.1 от штатного (или от расстояния до следующего излома), затем шаг удваивается
//! до штатного — как в SPICE. Без этого Эйлер на фронте сдвигает отклик на полшага.

use crate::circuit::{Circuit, Kind};
use crate::error::SimError;
use crate::mna::{
    CapState, Engine, IndState, Method, PointCtx, PointFail, SimOptions, singular_error,
};
use crate::netlist::TranSpec;

/// Настройки переходного процесса.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TranOptions {
    /// Не считать рабочую точку: старт с IC= конденсаторов и индуктивностей (остальное — нули).
    pub uic: bool,
    /// Наибольший внутренний шаг (по умолчанию равен шагу `dt`).
    pub max_step: Option<f64>,
    pub sim: SimOptions,
}

/// Счётчики работы решателя.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TranStats {
    /// Принятые внутренние шаги.
    pub steps: usize,
    /// Отвергнутые попытки (несходимость Ньютона → деление шага).
    pub rejected: usize,
    /// Шаги неявным Эйлером (первый, после изломов и переключений).
    pub euler_steps: usize,
}

/// Переходный процесс с фиксированным шагом хоста.
pub struct Transient {
    eng: Engine,
    dt: f64,
    h_max: f64,
    t: f64,
    need_be: bool,
    h_hint: f64,
    sw_pending: Vec<Option<bool>>,
    stats: TranStats,
    /// Предупреждения запуска (например, не удалось согласовать начальные условия UIC).
    pub warnings: Vec<String>,
}

impl Transient {
    /// Создать переходный процесс с шагом `dt`. Начальное состояние — рабочая точка по
    /// постоянному току (источники в момент t = 0), либо IC= элементов, если в `.tran` есть `uic`.
    pub fn new(ckt: &Circuit, dt: f64) -> Result<Transient, SimError> {
        let spec = ckt.tran_spec();
        Transient::with_options(
            ckt,
            dt,
            TranOptions {
                uic: spec.is_some_and(|s| s.uic),
                max_step: spec.and_then(|s| s.tmax),
                sim: SimOptions::default(),
            },
        )
    }

    pub fn with_options(ckt: &Circuit, dt: f64, opts: TranOptions) -> Result<Transient, SimError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(SimError::Invalid(format!(
                "шаг переходного процесса должен быть положительным, а не {dt}"
            )));
        }
        let h_max = opts.max_step.map_or(dt, |m| m.min(dt));
        // Нулевые фронты PULSE → шаг (правило SPICE: «tr, tf по умолчанию = TSTEP»).
        let mut ckt = ckt.clone();
        for e in &mut ckt.elems {
            if let Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } = &mut e.kind
                && let Some(w) = &src.wave
            {
                src.wave = Some(w.with_default_edges(dt));
            }
        }
        let mut eng = Engine::new(&ckt, opts.sim);
        let mut warnings = ckt.warnings.clone();
        if opts.uic {
            init_uic(&mut eng, dt, &mut warnings)?;
        } else {
            eng.solve_dc(false, 0.0)?;
            eng.init_reactive_from_dc();
        }
        let n_sw = eng.sw_on.len();
        Ok(Transient {
            eng,
            dt,
            h_max,
            t: 0.0,
            need_be: true,
            h_hint: h_max,
            sw_pending: vec![None; n_sw],
            stats: TranStats::default(),
            warnings,
        })
    }

    /// Текущее время, с.
    pub fn time(&self) -> f64 {
        self.t
    }

    /// Шаг хоста, с.
    pub fn dt(&self) -> f64 {
        self.dt
    }

    pub fn stats(&self) -> TranStats {
        self.stats
    }

    /// Задать значение EXT-источника (В или А) на конец следующего шага.
    pub fn set_source(&mut self, name: &str, value: f64) -> Result<(), SimError> {
        let e = self.eng.ckt.find_elem(name)?;
        match &e.kind {
            Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } => match src.ext {
                Some(k) => {
                    self.eng.ext[k].to = value;
                    Ok(())
                }
                None => Err(SimError::WrongElement {
                    name: name.into(),
                    message: "источник не внешний: объявите его как «EXT начальное_значение»"
                        .into(),
                }),
            },
            _ => Err(SimError::WrongElement {
                name: name.into(),
                message: "это не источник".into(),
            }),
        }
    }

    /// Включить/выключить EXT-ключ на следующий шаг.
    pub fn set_switch(&mut self, name: &str, on: bool) -> Result<(), SimError> {
        let e = self.eng.ckt.find_elem(name)?;
        match &e.kind {
            Kind::Switch(s) if s.ctrl.is_none() => {
                self.sw_pending[s.s] = Some(on);
                Ok(())
            }
            Kind::Switch(_) => Err(SimError::WrongElement {
                name: name.into(),
                message: "ключ управляется напряжением, а не из кода (нужна форма «S n+ n- EXT»)"
                    .into(),
            }),
            _ => Err(SimError::WrongElement {
                name: name.into(),
                message: "это не ключ".into(),
            }),
        }
    }

    /// Состояние ключа в последнем рассчитанном моменте.
    pub fn switch_state(&self, name: &str) -> Result<bool, SimError> {
        let e = self.eng.ckt.find_elem(name)?;
        match &e.kind {
            Kind::Switch(s) => Ok(self.eng.sw_on[s.s]),
            _ => Err(SimError::WrongElement {
                name: name.into(),
                message: "это не ключ".into(),
            }),
        }
    }

    /// Потенциал узла относительно земли, В.
    pub fn voltage(&self, node: &str) -> Result<f64, SimError> {
        self.eng.node_voltage(node)
    }

    /// Ток элемента, А: из первого узла через элемент во второй (для источника напряжения —
    /// как в SPICE: из n+ через источник в n-, поэтому у отдающего мощность источника ток
    /// отрицательный); для МОП — ток, втекающий в сток.
    pub fn current(&self, element: &str) -> Result<f64, SimError> {
        self.eng.element_current(element, self.t)
    }

    /// Мгновенная поглощаемая мощность элемента, Вт (> 0 — потребляет, < 0 — отдаёт).
    pub fn power(&self, element: &str) -> Result<f64, SimError> {
        self.eng.element_power(element, self.t)
    }

    /// Имена наблюдаемых сигналов: `v(узел)` и `i(ветвь)` для V, L, E.
    pub fn signal_names(&self) -> Vec<String> {
        self.eng.signal_names()
    }

    /// Значения сигналов в порядке [`Transient::signal_names`].
    pub fn signal_values(&self, out: &mut Vec<f64>) {
        self.eng.signal_values(out)
    }

    /// Сделать один шаг хоста `dt` (внутри — подшаги до изломов источников и при делении шага).
    pub fn step(&mut self) -> Result<(), SimError> {
        let target = self.t + self.dt;
        self.advance_to(target)
    }

    /// Дойти до момента `t_end` (внешние входы интерполируются по всему интервалу).
    pub fn advance_to(&mut self, t_end: f64) -> Result<(), SimError> {
        let eps = 1e-9 * self.dt;
        if t_end <= self.t + eps {
            return Ok(());
        }
        for (k, p) in self.sw_pending.iter_mut().enumerate() {
            if let Some(on) = p.take()
                && self.eng.sw_on[k] != on
            {
                self.eng.sw_on[k] = on;
                self.need_be = true;
            }
        }
        let t_start = self.t;
        let span = t_end - t_start;
        while self.t < t_end - eps {
            let mut h = self.h_hint.min(self.h_max).min(t_end - self.t);
            let mut bp_hit = None;
            if let Some(bp) = self.next_breakpoint(self.t, eps)
                && bp <= self.t + h + eps
            {
                h = bp - self.t;
                bp_hit = Some(bp);
            }
            if bp_hit.is_none() && t_end - (self.t + h) < eps {
                h = t_end - self.t;
            }
            let mut halvings = 0;
            let mut hit: bool;
            // Поиск момента переключения ключей, управляемых напряжением: шаг, на котором
            // ключ сменил состояние, переделывается до момента пересечения порога (со старыми
            // состояниями), и ключ переключается точно в этот момент.
            let mut freeze = false;
            let mut event_states: Option<Vec<bool>> = None;
            let min_event = 1e-6 * self.h_max;
            loop {
                let method = if self.need_be {
                    Method::Be
                } else {
                    Method::Trap
                };
                let t_new = self.t + h;
                let ctx = PointCtx {
                    t: t_new,
                    method,
                    h,
                    src_scale: 1.0,
                    gmin_extra: 0.0,
                    ext_frac: ((t_new - t_start) / span).clamp(0.0, 1.0),
                    standalone_dc: false,
                    init_junctions: false,
                    freeze_switches: freeze,
                    max_iter: self.eng.opts.max_iter_tran,
                };
                let mut trial = self.eng.sw_on.clone();
                let before = (!freeze && self.eng.has_vc_switches)
                    .then(|| (self.eng.x.clone(), self.eng.nl_snapshot()));
                match self.eng.solve_point(&ctx, &mut trial) {
                    Ok(()) => {
                        if let Some((x0, snap)) = before
                            && let Some((theta, states)) =
                                self.eng.switch_event(&x0, &self.eng.x, &trial)
                        {
                            if theta * h >= min_event {
                                // Порог пересечён внутри шага: дойти до него со старыми состояниями.
                                self.eng.x = x0;
                                self.eng.nl_restore(snap);
                                h *= theta;
                                bp_hit = None;
                                freeze = true;
                                event_states = Some(states);
                                continue;
                            }
                            if method == Method::Trap {
                                // Переключение в самом начале шага: трапеции через разрыв дали
                                // бы ошибку O(h) — переключаем сейчас и переделываем шаг Эйлером.
                                self.eng.x = x0;
                                self.eng.nl_restore(snap);
                                self.eng.sw_on = states;
                                self.need_be = true;
                                freeze = true;
                                continue;
                            }
                        }
                        let toggled = self.commit(method, h, trial);
                        self.stats.steps += 1;
                        if method == Method::Be {
                            self.stats.euler_steps += 1;
                        }
                        self.t = match bp_hit {
                            Some(bp) => bp,
                            None if (t_new - t_end).abs() < eps => t_end,
                            None => t_new,
                        };
                        self.need_be = bp_hit.is_some() || toggled;
                        hit = bp_hit.is_some();
                        if let Some(states) = event_states.take() {
                            self.eng.sw_on = states;
                            self.need_be = true;
                            hit = true;
                        }
                        break;
                    }
                    Err(PointFail::Singular) => return Err(singular_error()),
                    Err(PointFail::NoConvergence) => {
                        self.stats.rejected += 1;
                        halvings += 1;
                        if halvings > self.eng.opts.max_halvings {
                            return Err(SimError::NoConvergence {
                                time: self.t,
                                message: format!(
                                    "метод Ньютона не сошёлся и после {} делений шага (шаг {:.3e} с)",
                                    halvings - 1,
                                    h
                                ),
                            });
                        }
                        h /= 2.0;
                        bp_hit = None;
                        self.need_be = true;
                    }
                }
            }
            // После неудач — продолжаем меньшим шагом; после излома источника первый шаг
            // (неявный Эйлер) — 0.1 от расстояния до следующего излома или от шага, как в SPICE:
            // Эйлер на изломе первого порядка, и короткий шаг делает его ошибку малой.
            // Дальше шаг удваивается до штатного.
            self.h_hint = if halvings > 0 {
                h
            } else if hit {
                let next = self
                    .next_breakpoint(self.t, eps)
                    .map_or(self.h_max, |b| b - self.t);
                0.1 * self.h_max.min(next)
            } else {
                (self.h_hint * 2.0).min(self.h_max)
            };
        }
        for e in &mut self.eng.ext {
            e.from = e.to;
        }
        Ok(())
    }

    fn next_breakpoint(&self, t: f64, eps: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        for e in &self.eng.ckt.elems {
            if let Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } = &e.kind
                && src.ext.is_none()
                && let Some(w) = &src.wave
                && let Some(bp) = w.next_breakpoint(t, eps)
            {
                best = Some(best.map_or(bp, |b: f64| b.min(bp)));
            }
        }
        best
    }

    /// Принять решение шага: обновить истории C и L, состояния ключей. Вернуть «ключ переключился».
    fn commit(&mut self, method: Method, h: f64, trial: Vec<bool>) -> bool {
        let eng = &mut self.eng;
        let x = &eng.x;
        let nn = eng.nn;
        let v = |n: usize| if n == 0 { 0.0 } else { x[n - 1] };
        for e in &eng.ckt.elems {
            match &e.kind {
                Kind::Cap { a, b, c, k, .. } => {
                    let old = eng.cap[*k];
                    let vn = v(*a) - v(*b);
                    let i = match method {
                        Method::Be => c / h * (vn - old.v),
                        Method::Trap => 2.0 * c / h * (vn - old.v) - old.i,
                        Method::Dc => 0.0,
                    };
                    eng.cap[*k] = CapState { v: vn, i };
                }
                Kind::Ind { a, b, br, k, .. } => {
                    eng.ind[*k] = IndState {
                        i: x[nn + br],
                        v: v(*a) - v(*b),
                    };
                }
                _ => {}
            }
        }
        let toggled = trial != eng.sw_on;
        eng.sw_on = trial;
        toggled
    }
}

/// Старт без рабочей точки: истории C и L — из IC= (иначе ноль); согласованные потенциалы
/// узлов при t = 0 — решение шага Эйлера ничтожной длины h₀ = 1e-6·dt (конденсатор с IC
/// ведёт себя как источник напряжения, индуктивность — как источник тока).
fn init_uic(eng: &mut Engine, dt: f64, warnings: &mut Vec<String>) -> Result<(), SimError> {
    for e in &eng.ckt.elems {
        match &e.kind {
            Kind::Cap { ic, k, .. } => {
                eng.cap[*k] = CapState {
                    v: ic.unwrap_or(0.0),
                    i: 0.0,
                }
            }
            Kind::Ind { ic, k, .. } => {
                eng.ind[*k] = IndState {
                    i: ic.unwrap_or(0.0),
                    v: 0.0,
                }
            }
            _ => {}
        }
    }
    let ctx = PointCtx {
        t: 0.0,
        method: Method::Be,
        h: 1e-6 * dt,
        src_scale: 1.0,
        gmin_extra: 0.0,
        ext_frac: 1.0,
        standalone_dc: false,
        init_junctions: true,
        freeze_switches: false,
        max_iter: eng.opts.max_iter_op,
    };
    let mut trial = eng.sw_on.clone();
    match eng.solve_point(&ctx, &mut trial) {
        Ok(()) => eng.sw_on = trial,
        Err(PointFail::Singular) => return Err(singular_error()),
        Err(PointFail::NoConvergence) => {
            eng.x.fill(0.0);
            warnings.push(
                "UIC: не удалось согласовать потенциалы узлов при t = 0, старт с нулей".into(),
            );
        }
    }
    Ok(())
}

/// Осциллограммы переходного процесса.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Waveforms {
    pub time: Vec<f64>,
    /// (`v(узел)` или `i(ветвь)`, отсчёты).
    pub signals: Vec<(String, Vec<f64>)>,
}

impl Waveforms {
    /// Отсчёты сигнала по имени (`v(out)`, `i(v1)`; регистр не важен).
    pub fn get(&self, name: &str) -> Option<&[f64]> {
        let key = name.trim().to_lowercase();
        self.signals
            .iter()
            .find(|(n, _)| *n == key)
            .map(|(_, v)| v.as_slice())
    }

    /// Значение сигнала в момент `t` (линейная интерполяция между отсчётами).
    pub fn at(&self, name: &str, t: f64) -> Option<f64> {
        let y = self.get(name)?;
        let ts = &self.time;
        if ts.is_empty() {
            return None;
        }
        if t <= ts[0] {
            return Some(y[0]);
        }
        let k = ts.partition_point(|&x| x < t);
        if k >= ts.len() {
            return Some(y[y.len() - 1]);
        }
        let (t0, t1) = (ts[k - 1], ts[k]);
        Some(y[k - 1] + (y[k] - y[k - 1]) * (t - t0) / (t1 - t0))
    }

    pub fn len(&self) -> usize {
        self.time.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time.is_empty()
    }
}

/// Пакетный переходный процесс по директиве `.tran` нетлиста.
pub fn run_tran(ckt: &Circuit) -> Result<Waveforms, SimError> {
    let spec = ckt
        .tran_spec()
        .ok_or_else(|| SimError::Invalid("в нетлисте нет директивы .tran".into()))?;
    run_tran_with(ckt, &spec, SimOptions::default())
}

/// Переходный процесс с явными параметрами. Отсчёты — на сетке k·tstep, начиная с tstart.
pub fn run_tran_with(
    ckt: &Circuit,
    spec: &TranSpec,
    sim: SimOptions,
) -> Result<Waveforms, SimError> {
    let opts = TranOptions {
        uic: spec.uic,
        max_step: spec.tmax,
        sim,
    };
    let mut tr = Transient::with_options(ckt, spec.tstep, opts)?;
    let names = tr.signal_names();
    let n_steps = ((spec.tstop / spec.tstep) - 1e-9).ceil().max(1.0) as usize;
    let mut wf = Waveforms {
        time: Vec::with_capacity(n_steps + 1),
        signals: names
            .into_iter()
            .map(|n| (n, Vec::with_capacity(n_steps + 1)))
            .collect(),
    };
    let mut buf = Vec::new();
    let eps = 1e-9 * spec.tstep;
    let mut record = |tr: &Transient, wf: &mut Waveforms| {
        if tr.time() >= spec.tstart - eps {
            tr.signal_values(&mut buf);
            wf.time.push(tr.time());
            for (s, v) in wf.signals.iter_mut().zip(&buf) {
                s.1.push(*v);
            }
        }
    };
    record(&tr, &mut wf);
    for k in 1..=n_steps {
        let target = (k as f64 * spec.tstep).min(spec.tstop);
        tr.advance_to(target)?;
        record(&tr, &mut wf);
    }
    Ok(wf)
}
