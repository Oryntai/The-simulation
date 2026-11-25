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
//! прошивки с удержанием). `set_source_step` задаёт скачок на левой границе следующего
//! интервала. После скачка и переключения шаг делается неявным Эйлером.
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

// Относительная граница округления шага из прежней схемы интегрирования.
const STEP_ROUNDING: f64 = 1e-9;

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
#[derive(Clone)]
pub struct Transient {
    eng: Engine,
    dt: f64,
    h_max: f64,
    t: f64,
    need_be: bool,
    h_hint: f64,
    sw_pending: Vec<Option<bool>>,
    source_steps: Vec<Option<f64>>,
    stats: TranStats,
    /// Предупреждения разбора и построения схемы.
    pub warnings: Vec<String>,
}

/// Состояние схемы и планировщика на границе шага, вместе с отложенными командами.
#[derive(Clone)]
pub struct TransientSnapshot(Transient);

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
        if opts.max_step.is_some_and(|h| !h.is_finite() || h <= 0.0) {
            return Err(SimError::Invalid(
                "максимальный шаг должен быть конечным и положительным".into(),
            ));
        }
        let h_max = opts.max_step.map_or(dt, |m| m.min(dt));
        let mut eng = Engine::new(ckt, opts.sim)?;
        // Нулевые фронты PULSE → шаг (правило SPICE: «tr, tf по умолчанию = TSTEP»).
        for e in &mut eng.ckt.elems {
            if let Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } = &mut e.kind
                && let Some(w) = &src.wave
            {
                src.wave = Some(w.with_default_edges(dt));
            }
        }
        if opts.uic {
            init_uic(&mut eng, dt)?;
        } else {
            eng.solve_dc(false, 0.0)?;
            eng.init_reactive_from_dc();
        }
        let n_sw = eng.sw_on.len();
        let n_ext = eng.ext.len();
        Ok(Transient {
            eng,
            dt,
            h_max,
            t: 0.0,
            need_be: true,
            h_hint: h_max,
            sw_pending: vec![None; n_sw],
            source_steps: vec![None; n_ext],
            stats: TranStats::default(),
            warnings: ckt.warnings.clone(),
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

    pub fn snapshot(&self) -> TransientSnapshot {
        TransientSnapshot(self.clone())
    }

    /// Восстановить схему, настройки, текущую точку и ещё не применённые команды.
    pub fn restore(&mut self, snapshot: &TransientSnapshot) {
        *self = snapshot.0.clone();
    }

    /// Задать значение EXT-источника (В или А) на конец следующего шага.
    /// После `set_source_step` линейный участок начинается с нового значения.
    pub fn set_source(&mut self, name: &str, value: f64) -> Result<(), SimError> {
        let k = self.external_source(name, value)?;
        self.eng.ext[k].to = value;
        Ok(())
    }

    /// Задать скачок EXT-источника (В или А) в текущий момент для следующего интервала.
    /// Сначала доведите расчёт до метки события через [`Self::advance_to`].
    /// Точка на метке сохраняет значения с левой стороны. Скачок задаёт новое
    /// значение на всём интервале, пока `set_source` не задаст другой его конец.
    pub fn set_source_step(&mut self, name: &str, value: f64) -> Result<(), SimError> {
        let k = self.external_source(name, value)?;
        self.source_steps[k] = Some(value);
        self.eng.ext[k].to = value;
        Ok(())
    }

    fn external_source(&self, name: &str, value: f64) -> Result<usize, SimError> {
        if !value.is_finite() {
            return Err(SimError::Invalid(format!(
                "неконечное значение источника «{name}»"
            )));
        }
        let e = self.eng.ckt.find_elem(name)?;
        match &e.kind {
            Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } => {
                src.ext.ok_or_else(|| SimError::WrongElement {
                    name: name.into(),
                    message: "источник не внешний: объявите его как «EXT начальное_значение»"
                        .into(),
                })
            }
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

    /// Активное состояние ключа. На найденной границе переключения управляемого ключа
    /// это состояние справа, тогда как ток относится к последней принятой точке MNA.
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

    /// Токи из принятого MNA, в порядке выводов нетлиста, положительные в сторону элемента.
    pub fn terminal_currents(&self, element: &str) -> Result<Vec<f64>, SimError> {
        self.eng.terminal_currents(element, self.t)
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
        if target == self.t {
            return Err(SimError::Invalid(
                "шаг меньше разрешения шкалы времени".into(),
            ));
        }
        self.advance_to(target)
    }

    /// Дойти точно до `t_end`. Линейные входы интерполируются по всему интервалу.
    /// При ошибке состояние и команды остаются на входной границе.
    pub fn advance_to(&mut self, t_end: f64) -> Result<(), SimError> {
        if !t_end.is_finite() || t_end < self.t {
            return Err(SimError::Invalid(format!(
                "недопустимая конечная метка {t_end} с"
            )));
        }
        if t_end == self.t {
            return Ok(());
        }
        let saved = self.eng.save_state();
        let timing = (self.t, self.need_be, self.h_hint, self.stats);
        if let Err(error) = self.integrate(t_end) {
            self.eng.restore_state(saved);
            (self.t, self.need_be, self.h_hint, self.stats) = timing;
            return Err(error);
        }
        self.sw_pending.fill(None);
        self.source_steps.fill(None);
        for e in &mut self.eng.ext {
            e.from = e.to;
        }
        Ok(())
    }

    fn integrate(&mut self, t_end: f64) -> Result<(), SimError> {
        for (k, &p) in self.sw_pending.iter().enumerate() {
            if let Some(on) = p
                && self.eng.sw_on[k] != on
            {
                self.eng.sw_on[k] = on;
                self.need_be = true;
            }
        }
        let t_start = self.t;
        let span = t_end - t_start;
        let mut source_jump = false;
        for (e, &step) in self.eng.ext.iter_mut().zip(&self.source_steps) {
            if let Some(value) = step {
                source_jump |= value != e.from;
                e.from = value;
            }
        }
        if source_jump {
            self.need_be = true;
            // Та же стартовая доля шага, что у изломов PWL/PULSE.
            self.h_hint = self.h_hint.min(0.1 * self.h_max.min(span));
        }
        while self.t < t_end {
            let remaining = t_end - self.t;
            let min_progress = self.t.next_up() - self.t;
            let mut h = self.h_hint.max(min_progress).min(self.h_max).min(remaining);
            // Сумма внутренних шагов может оставить остаток округления на правой границе.
            // Достигаем заданной метки этим шагом без отдельного микрошага.
            let proposed = self.t + h;
            let mut reaches_end = proposed > self.t && t_end - proposed <= STEP_ROUNDING * h;
            if reaches_end {
                h = remaining;
            }
            let mut bp_hit = None;
            if let Some(bp) = self.next_breakpoint(self.t) {
                let limit = if reaches_end { t_end } else { proposed };
                if bp <= limit {
                    let coincides = reaches_end && t_end - bp <= STEP_ROUNDING * h;
                    let boundary = if coincides { t_end } else { bp };
                    reaches_end = coincides;
                    h = boundary - self.t;
                    bp_hit = Some(boundary);
                }
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
                let t_new = bp_hit.unwrap_or_else(|| if reaches_end { t_end } else { self.t + h });
                if h <= 0.0 || t_new <= self.t || t_new > t_end {
                    return Err(SimError::NoConvergence {
                        time: self.t,
                        message: "внутренний шаг меньше разрешения шкалы времени".into(),
                    });
                }
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
                                reaches_end = false;
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
                        self.eng.record_inputs(ctx.ext_frac, &trial);
                        let toggled = self.commit(method, h, trial);
                        self.stats.steps += 1;
                        if method == Method::Be {
                            self.stats.euler_steps += 1;
                        }
                        self.t = t_new;
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
                        reaches_end = false;
                        if event_states.take().is_some() {
                            freeze = false;
                        }
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
                    .next_breakpoint(self.t)
                    .map_or(self.h_max, |b| b - self.t);
                0.1 * self.h_max.min(next)
            } else {
                (self.h_hint * 2.0).min(self.h_max)
            };
        }
        Ok(())
    }

    fn next_breakpoint(&self, t: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        for e in &self.eng.ckt.elems {
            if let Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } = &e.kind
                && src.ext.is_none()
                && let Some(w) = &src.wave
            {
                // Суммы длительностей PULSE округляются отдельно от шкалы хоста.
                // Для PWL метки даны напрямую, поэтому ищем их без сдвига.
                let eps = if matches!(w, crate::netlist::Waveform::Pulse { .. }) {
                    STEP_ROUNDING * self.dt
                } else {
                    0.0
                };
                if let Some(bp) = w.next_breakpoint(t, eps) {
                    best = Some(best.map_or(bp, |b: f64| b.min(bp)));
                }
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
fn init_uic(eng: &mut Engine, dt: f64) -> Result<(), SimError> {
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
        Ok(()) => {
            eng.record_inputs(ctx.ext_frac, &trial);
            eng.sw_on = trial;
            for e in &eng.ckt.elems {
                if let Kind::Cap { a, b, c, k, .. } = e.kind {
                    let v = eng.v(&eng.x, a) - eng.v(&eng.x, b);
                    eng.cap[k].i = (c / ctx.h) * (v - eng.cap[k].v);
                }
            }
        }
        Err(PointFail::Singular) => return Err(singular_error()),
        Err(PointFail::NoConvergence) => {
            return Err(SimError::NoConvergence {
                time: 0.0,
                message: "UIC: не удалось согласовать потенциалы узлов".into(),
            });
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
        if !t.is_finite() {
            return None;
        }
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
    if !spec.tstop.is_finite()
        || !spec.tstart.is_finite()
        || spec.tstop <= 0.0
        || spec.tstart < 0.0
        || spec.tstart >= spec.tstop
    {
        return Err(SimError::Invalid(
            "недопустимые границы переходного процесса".into(),
        ));
    }
    let opts = TranOptions {
        uic: spec.uic,
        max_step: spec.tmax,
        sim,
    };
    let mut tr = Transient::with_options(ckt, spec.tstep, opts)?;
    let names = tr.signal_names();
    let steps = (spec.tstop / spec.tstep).ceil().max(1.0);
    if steps >= usize::MAX as f64 {
        return Err(SimError::Invalid(
            "сетка переходного процесса слишком велика".into(),
        ));
    }
    let n_steps = steps as usize;
    let mut wf = Waveforms {
        time: Vec::new(),
        signals: names.into_iter().map(|n| (n, Vec::new())).collect(),
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
        let target = if k == n_steps {
            spec.tstop
        } else {
            k as f64 * spec.tstep
        };
        if target >= spec.tstop && k != n_steps {
            continue;
        }
        tr.advance_to(target)?;
        record(&tr, &mut wf);
    }
    Ok(wf)
}
