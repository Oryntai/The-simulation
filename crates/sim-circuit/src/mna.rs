//! Модифицированный узловой анализ (MNA): сборка системы, метод Ньютона, рабочая точка.
//!
//! Неизвестные: потенциалы узлов (кроме земли), затем токи ветвей (источники напряжения,
//! индуктивности, ИНУН). Система плотная, решается LU-разложением nalgebra с частичным
//! выбором ведущего элемента — для схем до нескольких сотен узлов этого достаточно.
//!
//! Схема интегрирования — трапеции; неявный Эйлер на первом шаге, после изломов источников и
//! переключений ключей (там трапеции дают незатухающий «звон» в токах ёмкостей). Модели-
//! компаньоны (Nagel 1975; Ho, Ruehli, Brennan, «The modified nodal approach to network
//! analysis», IEEE TCAS 22(6):504–509, 1975, DOI 10.1109/TCS.1975.1084079):
//! * C, трапеции: i₁ = (2C/h)(v₁ − v₀) − i₀; Эйлер: i₁ = (C/h)(v₁ − v₀);
//! * L (ток — неизвестная ветви), трапеции: v₁ − (2L/h)i₁ = −(2L/h)i₀ − v₀; Эйлер: v₁ − (L/h)i₁ = −(L/h)i₀.
//!
//! Каждый узел шунтирован на землю проводимостью `gshunt` (по умолчанию 1e-12 См), каждый p-n
//! переход — проводимостью `gmin` (1e-12 См, как в SPICE): так у узлов, связанных с остальной
//! схемой лишь ёмкостями, есть определённый потенциал по постоянному току. Ошибка от шунта —
//! порядка gshunt·R (≈1e-9 для килоомных цепей).

use nalgebra::{DMatrix, DVector, Dyn, LU};

use crate::circuit::{Circuit, Kind, Node};
use crate::devices::{fetlim, junction_iv, limvds, mos_linearize, pnjlim, thermal_voltage};
use crate::error::SimError;

/// Настройки решателя.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimOptions {
    /// Относительный допуск сходимости Ньютона (SPICE RELTOL; у нас строже, чем 1e-3 SPICE).
    pub reltol: f64,
    /// Абсолютный допуск по напряжениям, В.
    pub vntol: f64,
    /// Абсолютный допуск по токам, А.
    pub abstol: f64,
    /// Проводимость параллельно p-n переходам, См.
    pub gmin: f64,
    /// Проводимость каждого узла на землю, См.
    pub gshunt: f64,
    /// Предел итераций Ньютона для рабочей точки.
    pub max_iter_op: usize,
    /// Предел итераций Ньютона на шаге переходного процесса (дальше — деление шага).
    pub max_iter_tran: usize,
    /// Сколько раз можно делить шаг пополам при несходимости.
    pub max_halvings: usize,
}

impl Default for SimOptions {
    fn default() -> Self {
        SimOptions {
            reltol: 1e-6,
            vntol: 1e-9,
            abstol: 1e-12,
            gmin: 1e-12,
            gshunt: 1e-12,
            max_iter_op: 200,
            max_iter_tran: 60,
            max_halvings: 24,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum Method {
    /// Рабочая точка: C — разрыв, L — короткое замыкание.
    Dc,
    /// Неявный Эйлер.
    Be,
    /// Трапеции.
    Trap,
}

/// Контекст расчёта одной точки (момента времени или рабочей точки).
#[derive(Clone, Copy, Debug)]
pub(crate) struct PointCtx {
    pub t: f64,
    pub method: Method,
    pub h: f64,
    /// Множитель всех независимых источников (шаги по источникам).
    pub src_scale: f64,
    /// Добавочная проводимость узлов на землю (шаги по gmin).
    pub gmin_extra: f64,
    /// Доля шага для линейной интерполяции EXT-источников (0 — начало, 1 — конец шага).
    pub ext_frac: f64,
    /// Отдельный `.op` (источники берут DC-значение), а не старт переходного процесса.
    pub standalone_dc: bool,
    /// Первая итерация: переходы — в точке vcrit, затворы — на пороге (SPICE MODEINITJCT).
    pub init_junctions: bool,
    /// Не менять состояния ключей, управляемых напряжением (шаг до момента переключения).
    pub freeze_switches: bool,
    pub max_iter: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct CapState {
    pub v: f64,
    pub i: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct IndState {
    pub i: f64,
    pub v: f64,
}

/// Напряжения последней линеаризации МОП (нормированные на полярность).
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct MosState {
    pub vgs: f64,
    pub vds: f64,
    pub vbs: f64,
    pub vbd: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ExtState {
    /// Значение в начале текущего шага.
    pub from: f64,
    /// Значение в конце шага (его задаёт хост).
    pub to: f64,
}

#[derive(Clone, Debug)]
struct CacheKey {
    method: Method,
    h: u64,
    gmin_extra: u64,
    switches: Vec<bool>,
}

struct CacheEntry {
    key: CacheKey,
    base: DMatrix<f64>,
    lu: Option<LU<f64, Dyn, Dyn>>,
}

const CACHE_SIZE: usize = 16;
/// Через сколько итераций Ньютона «замораживаются» состояния ключей (защита от дребезга).
const SWITCH_FREEZE_ITER: usize = 25;

/// Ошибка расчёта точки: вырожденность — структурная (дробление шага не поможет).
#[derive(Debug)]
pub(crate) enum PointFail {
    NoConvergence,
    Singular,
}

/// Движок: схема, её состояние и кэш матриц. Общий для `.op` и переходного процесса.
pub(crate) struct Engine {
    pub ckt: Circuit,
    pub opts: SimOptions,
    /// Число узлов без земли.
    pub nn: usize,
    pub n: usize,
    pub x: DVector<f64>,
    pub cap: Vec<CapState>,
    pub ind: Vec<IndState>,
    pub sw_on: Vec<bool>,
    pub junc: Vec<f64>,
    pub mos: Vec<MosState>,
    pub ext: Vec<ExtState>,
    cache: Vec<CacheEntry>,
    cache_next: usize,
    pub linear: bool,
    /// Есть ключи, управляемые напряжением (для них ищется момент переключения).
    pub has_vc_switches: bool,
    /// Режим отдельного `.op`: источники берут DC-значения.
    pub standalone_dc: bool,
}

#[inline]
fn idx(n: Node) -> Option<usize> {
    n.checked_sub(1)
}

#[inline]
fn add(m: &mut DMatrix<f64>, r: Option<usize>, c: Option<usize>, v: f64) {
    if let (Some(r), Some(c)) = (r, c) {
        m[(r, c)] += v;
    }
}

#[inline]
fn stamp_g(m: &mut DMatrix<f64>, a: Node, b: Node, g: f64) {
    let (ia, ib) = (idx(a), idx(b));
    add(m, ia, ia, g);
    add(m, ib, ib, g);
    add(m, ia, ib, -g);
    add(m, ib, ia, -g);
}

/// Ток `i` течёт из узла `a` через элемент в узел `b`.
#[inline]
fn stamp_i(rhs: &mut DVector<f64>, a: Node, b: Node, i: f64) {
    if let Some(ia) = idx(a) {
        rhs[ia] -= i;
    }
    if let Some(ib) = idx(b) {
        rhs[ib] += i;
    }
}

#[inline]
fn stamp_branch_incidence(m: &mut DMatrix<f64>, a: Node, b: Node, br: usize) {
    let (ia, ib) = (idx(a), idx(b));
    add(m, ia, Some(br), 1.0);
    add(m, ib, Some(br), -1.0);
    add(m, Some(br), ia, 1.0);
    add(m, Some(br), ib, -1.0);
}

/// Состояние ключа по управляющему напряжению с гистерезисом.
pub(crate) fn switch_decide(vc: f64, vt: f64, vh: f64, prev: bool) -> bool {
    if vc > vt + vh {
        true
    } else if vc < vt - vh {
        false
    } else {
        prev
    }
}

impl Engine {
    pub fn new(ckt: &Circuit, opts: SimOptions) -> Engine {
        let nn = ckt.nodes.len() - 1;
        let n = nn + ckt.n_branches;
        let mut sw_on = vec![false; ckt.n_switches];
        let mut ext = vec![ExtState::default(); ckt.n_ext];
        for e in &ckt.elems {
            match &e.kind {
                Kind::Switch(s) => sw_on[s.s] = s.initially_on,
                Kind::Vsrc { src, .. } | Kind::Isrc { src, .. } => {
                    if let (Some(k), Some(crate::netlist::Waveform::Ext { initial })) =
                        (src.ext, &src.wave)
                    {
                        ext[k] = ExtState {
                            from: *initial,
                            to: *initial,
                        };
                    }
                }
                _ => {}
            }
        }
        let linear = !ckt.is_nonlinear();
        let has_vc_switches = ckt
            .elems
            .iter()
            .any(|e| matches!(&e.kind, Kind::Switch(s) if s.ctrl.is_some()));
        Engine {
            ckt: ckt.clone(),
            opts,
            nn,
            n,
            x: DVector::zeros(n),
            cap: vec![CapState::default(); ckt.n_caps],
            ind: vec![IndState::default(); ckt.n_inds],
            sw_on,
            junc: vec![0.0; ckt.n_junctions],
            mos: vec![MosState::default(); ckt.n_mos],
            ext,
            cache: Vec::new(),
            cache_next: 0,
            linear,
            has_vc_switches,
            standalone_dc: false,
        }
    }

    #[inline]
    pub fn v(&self, x: &DVector<f64>, n: Node) -> f64 {
        match idx(n) {
            Some(i) => x[i],
            None => 0.0,
        }
    }

    // ───────────── Сборка ─────────────

    fn build_base(&self, key: &CacheKey) -> DMatrix<f64> {
        let mut m = DMatrix::zeros(self.n, self.n);
        let gnode = self.opts.gshunt + f64::from_bits(key.gmin_extra);
        for i in 0..self.nn {
            m[(i, i)] += gnode;
        }
        let h = f64::from_bits(key.h);
        let gmin = self.opts.gmin;
        let nn = self.nn;
        for e in &self.ckt.elems {
            match &e.kind {
                Kind::Res { a, b, g } => stamp_g(&mut m, *a, *b, *g),
                Kind::Cap { a, b, c, .. } => match key.method {
                    Method::Dc => {}
                    Method::Be => stamp_g(&mut m, *a, *b, c / h),
                    Method::Trap => stamp_g(&mut m, *a, *b, 2.0 * c / h),
                },
                Kind::Ind { a, b, l, br, .. } => {
                    let r = nn + br;
                    stamp_branch_incidence(&mut m, *a, *b, r);
                    let req = match key.method {
                        Method::Dc => 0.0,
                        Method::Be => l / h,
                        Method::Trap => 2.0 * l / h,
                    };
                    m[(r, r)] -= req;
                }
                Kind::Vsrc { a, b, br, .. } => stamp_branch_incidence(&mut m, *a, *b, nn + br),
                Kind::Isrc { .. } => {}
                Kind::Diode(d) => {
                    if d.ja != d.a {
                        stamp_g(&mut m, d.a, d.ja, 1.0 / d.p.rs);
                    }
                    stamp_g(&mut m, d.ja, d.k, gmin);
                }
                Kind::Switch(s) => {
                    let g = if key.switches[s.s] {
                        1.0 / s.ron
                    } else {
                        1.0 / s.roff
                    };
                    stamp_g(&mut m, s.a, s.b, g);
                }
                Kind::Mos(mo) => {
                    stamp_g(&mut m, mo.b, mo.d, gmin);
                    stamp_g(&mut m, mo.b, mo.s, gmin);
                }
                Kind::Vcvs {
                    a,
                    b,
                    ca,
                    cb,
                    gain,
                    br,
                } => {
                    let r = nn + br;
                    stamp_branch_incidence(&mut m, *a, *b, r);
                    add(&mut m, Some(r), idx(*ca), -gain);
                    add(&mut m, Some(r), idx(*cb), *gain);
                }
                Kind::Vccs { a, b, ca, cb, gm } => {
                    let (ia, ib, ica, icb) = (idx(*a), idx(*b), idx(*ca), idx(*cb));
                    add(&mut m, ia, ica, *gm);
                    add(&mut m, ia, icb, -gm);
                    add(&mut m, ib, ica, -gm);
                    add(&mut m, ib, icb, *gm);
                }
            }
        }
        m
    }

    /// Значение независимого источника в контексте `ctx`.
    pub fn src_value(&self, src: &crate::circuit::SrcFn, ctx: &PointCtx) -> f64 {
        let v = if let Some(k) = src.ext {
            let e = self.ext[k];
            e.from + (e.to - e.from) * ctx.ext_frac
        } else if ctx.standalone_dc {
            src.op_value()
        } else {
            src.tran_value(ctx.t)
        };
        v * ctx.src_scale
    }

    fn build_rhs(&self, rhs: &mut DVector<f64>, ctx: &PointCtx) {
        rhs.fill(0.0);
        let nn = self.nn;
        let h = ctx.h;
        for e in &self.ckt.elems {
            match &e.kind {
                Kind::Vsrc { src, br, .. } => rhs[nn + br] = self.src_value(src, ctx),
                Kind::Isrc { a, b, src } => stamp_i(rhs, *a, *b, self.src_value(src, ctx)),
                Kind::Cap { a, b, c, k, .. } => {
                    let st = self.cap[*k];
                    let ieq = match ctx.method {
                        Method::Dc => 0.0,
                        Method::Be => -(c / h) * st.v,
                        Method::Trap => -(2.0 * c / h) * st.v - st.i,
                    };
                    stamp_i(rhs, *a, *b, ieq);
                }
                Kind::Ind { l, br, k, .. } => {
                    let st = self.ind[*k];
                    rhs[nn + br] = match ctx.method {
                        Method::Dc => 0.0,
                        Method::Be => -(l / h) * st.i,
                        Method::Trap => -(2.0 * l / h) * st.i - st.v,
                    };
                }
                _ => {}
            }
        }
    }

    /// Линеаризовать нелинейные приборы в точке `x`; вернуть «было ограничение шага».
    fn stamp_nonlinear(
        &mut self,
        m: &mut DMatrix<f64>,
        rhs: &mut DVector<f64>,
        x: &DVector<f64>,
        init: bool,
    ) -> bool {
        let mut limited = false;
        let vt = thermal_voltage();
        // Разделяем заимствования: схема читается, состояния переходов пишутся.
        let Engine { ckt, junc, mos, .. } = self;
        let vof = |n: Node| idx(n).map_or(0.0, |i| x[i]);
        for e in &ckt.elems {
            match &e.kind {
                Kind::Diode(d) => {
                    let nvt = d.p.nvt();
                    let raw = vof(d.ja) - vof(d.k);
                    let vd = if init {
                        limited = true;
                        d.vcrit
                    } else {
                        let (v, lim) = pnjlim(raw, junc[d.j], nvt, d.vcrit);
                        limited |= lim;
                        v
                    };
                    junc[d.j] = vd;
                    let (id, gd) = junction_iv(d.p.is, nvt, vd);
                    stamp_g(m, d.ja, d.k, gd);
                    stamp_i(rhs, d.ja, d.k, id - gd * vd);
                }
                Kind::Mos(mo) => {
                    let sg = mo.p.kind.sign();
                    let vto_n = sg * mo.p.vto;
                    let vgs_raw = sg * (vof(mo.g) - vof(mo.s));
                    let vds_raw = sg * (vof(mo.d) - vof(mo.s));
                    let vbs_raw = sg * (vof(mo.b) - vof(mo.s));
                    let vbd_raw = sg * (vof(mo.b) - vof(mo.d));
                    let st = &mut mos[mo.m];
                    let (vgs, vds, vbs, vbd) = if init {
                        limited = true;
                        (vto_n, 0.0, 0.0, 0.0)
                    } else {
                        let vgd_raw = vgs_raw - vds_raw;
                        let (vgs, vds) = if st.vds >= 0.0 {
                            let vgs = fetlim(vgs_raw, st.vgs, vto_n);
                            let vds = limvds(vgs - vgd_raw, st.vds);
                            (vgs, vds)
                        } else {
                            let vgd = fetlim(vgd_raw, st.vgs - st.vds, vto_n);
                            let vds = -limvds(-(vgs_raw - vgd), -st.vds);
                            (vgd + vds, vds)
                        };
                        let (vbs, l1) = pnjlim(vbs_raw, st.vbs, vt, mo.vcrit);
                        let (vbd, l2) = pnjlim(vbd_raw, st.vbd, vt, mo.vcrit);
                        let tol = 1e-12 * (1.0 + vgs_raw.abs() + vds_raw.abs());
                        limited |=
                            l1 || l2 || (vgs - vgs_raw).abs() > tol || (vds - vds_raw).abs() > tol;
                        (vgs, vds, vbs, vbd)
                    };
                    *st = MosState { vgs, vds, vbs, vbd };
                    // Канал: ток сток→исток, линеаризация в точке (vgs, vds), исток = 0.
                    let lin = mos_linearize(&mo.p, vgs, vds);
                    let ieq = lin.ids - lin.dg * sg * vgs - lin.dd * sg * vds;
                    let (id, ig, is) = (idx(mo.d), idx(mo.g), idx(mo.s));
                    add(m, id, ig, lin.dg);
                    add(m, id, id, lin.dd);
                    add(m, id, is, lin.ds);
                    add(m, is, ig, -lin.dg);
                    add(m, is, id, -lin.dd);
                    add(m, is, is, -lin.ds);
                    stamp_i(rhs, mo.d, mo.s, ieq);
                    // Переходы подложки: «анод» — подложка для NMOS, сток/исток для PMOS.
                    for (vj, other) in [(vbd, mo.d), (vbs, mo.s)] {
                        let (an, ca) = if sg > 0.0 {
                            (mo.b, other)
                        } else {
                            (other, mo.b)
                        };
                        let (ij, gj) = junction_iv(mo.is, vt, vj);
                        stamp_g(m, an, ca, gj);
                        stamp_i(rhs, an, ca, ij - gj * vj);
                    }
                }
                _ => {}
            }
        }
        limited
    }

    /// Обновить пробные состояния ключей, управляемых напряжением. Вернуть «что-то изменилось».
    fn update_switches(&self, x: &DVector<f64>, trial: &mut [bool]) -> bool {
        let mut changed = false;
        for e in &self.ckt.elems {
            if let Kind::Switch(s) = &e.kind
                && let Some((cp, cn)) = s.ctrl
            {
                let vc = self.v(x, cp) - self.v(x, cn);
                let st = switch_decide(vc, s.vt, s.vh, self.sw_on[s.s]);
                if st != trial[s.s] {
                    trial[s.s] = st;
                    changed = true;
                }
            }
        }
        changed
    }

    /// Снимок состояний линеаризации нелинейных приборов (для отката принятого решения).
    pub fn nl_snapshot(&self) -> (Vec<f64>, Vec<MosState>) {
        (self.junc.clone(), self.mos.clone())
    }

    pub fn nl_restore(&mut self, snap: (Vec<f64>, Vec<MosState>)) {
        self.junc = snap.0;
        self.mos = snap.1;
    }

    /// Ключи, управляемые напряжением, сменили состояние на шаге x0 → x1: доля шага θ ∈ [0, 1],
    /// на которой управляющее напряжение пересекает порог (линейная интерполяция), и новые
    /// состояния ключей, переключающихся первыми (в пределах 1e-9 от θ).
    pub fn switch_event(
        &self,
        x0: &DVector<f64>,
        x1: &DVector<f64>,
        trial: &[bool],
    ) -> Option<(f64, Vec<bool>)> {
        let mut found: Vec<(f64, usize)> = Vec::new();
        for e in &self.ckt.elems {
            if let Kind::Switch(s) = &e.kind
                && let Some((cp, cn)) = s.ctrl
                && trial[s.s] != self.sw_on[s.s]
            {
                let vc0 = self.v(x0, cp) - self.v(x0, cn);
                let vc1 = self.v(x1, cp) - self.v(x1, cn);
                let level = if trial[s.s] { s.vt + s.vh } else { s.vt - s.vh };
                let theta = if (vc1 - vc0).abs() > 0.0 {
                    ((level - vc0) / (vc1 - vc0)).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                found.push((theta, s.s));
            }
        }
        let theta = found.iter().map(|f| f.0).fold(f64::INFINITY, f64::min);
        if !theta.is_finite() {
            return None;
        }
        let mut states = self.sw_on.clone();
        for (th, k) in found {
            if th <= theta + 1e-9 {
                states[k] = trial[k];
            }
        }
        Some((theta, states))
    }

    fn converged(&self, a: &DVector<f64>, b: &DVector<f64>) -> bool {
        let o = &self.opts;
        a.iter().zip(b.iter()).enumerate().all(|(i, (&p, &q))| {
            let floor = if i < self.nn { o.vntol } else { o.abstol };
            (p - q).abs() <= o.reltol * p.abs().max(q.abs()) + floor
        })
    }

    /// Слот кэша для (метод, шаг, gmin, состояния ключей); ключ без выделения памяти
    /// сравнивается по срезу, новая запись создаётся только при промахе.
    fn cache_slot(&mut self, method: Method, h: u64, gmin_extra: u64, switches: &[bool]) -> usize {
        if let Some(i) = self.cache.iter().position(|c| {
            c.key.method == method
                && c.key.h == h
                && c.key.gmin_extra == gmin_extra
                && c.key.switches == switches
        }) {
            return i;
        }
        let key = CacheKey {
            method,
            h,
            gmin_extra,
            switches: switches.to_vec(),
        };
        let base = self.build_base(&key);
        let entry = CacheEntry {
            key,
            base,
            lu: None,
        };
        if self.cache.len() < CACHE_SIZE {
            self.cache.push(entry);
            self.cache.len() - 1
        } else {
            let i = self.cache_next;
            self.cache[i] = entry;
            self.cache_next = (self.cache_next + 1) % CACHE_SIZE;
            i
        }
    }

    fn solution_ok(x: &DVector<f64>) -> bool {
        x.iter().all(|v| v.is_finite() && v.abs() < 1e30)
    }

    /// Решить одну точку методом Ньютона. При успехе `self.x`, состояния переходов и
    /// пробные состояния ключей (`sw_trial`) обновлены; состояние ключей НЕ принято.
    pub fn solve_point(
        &mut self,
        ctx: &PointCtx,
        sw_trial: &mut Vec<bool>,
    ) -> Result<(), PointFail> {
        let saved_junc = self.junc.clone();
        let saved_mos = self.mos.clone();
        let saved_trial = sw_trial.clone();
        let result = self.newton(ctx, sw_trial);
        if result.is_err() {
            self.junc = saved_junc;
            self.mos = saved_mos;
            *sw_trial = saved_trial;
        }
        result
    }

    fn newton(&mut self, ctx: &PointCtx, sw_trial: &mut [bool]) -> Result<(), PointFail> {
        let mut x = self.x.clone();
        if !ctx.freeze_switches {
            self.update_switches(&x, sw_trial);
        }
        let mut rhs = DVector::zeros(self.n);
        self.build_rhs(&mut rhs, ctx);
        let h_bits = if ctx.method == Method::Dc {
            0
        } else {
            ctx.h.to_bits()
        };
        for iter in 0..ctx.max_iter.max(1) {
            let slot = self.cache_slot(ctx.method, h_bits, ctx.gmin_extra.to_bits(), sw_trial);
            if self.linear {
                // Линейная схема: одно решение кэшированной LU; повтор — только если
                // ключи, управляемые напряжением, сменили состояние.
                let entry = &mut self.cache[slot];
                if entry.lu.is_none() {
                    entry.lu = Some(entry.base.clone().lu());
                }
                let sol = entry
                    .lu
                    .as_ref()
                    .and_then(|lu| lu.solve(&rhs))
                    .filter(Self::solution_ok)
                    .ok_or(PointFail::Singular)?;
                x = sol;
                let sw_changed = !ctx.freeze_switches
                    && iter < SWITCH_FREEZE_ITER
                    && self.update_switches(&x, sw_trial);
                if !sw_changed {
                    self.x = x;
                    return Ok(());
                }
                continue;
            }
            let mut m = self.cache[slot].base.clone();
            let mut r = rhs.clone();
            let limited = self.stamp_nonlinear(&mut m, &mut r, &x, ctx.init_junctions && iter == 0);
            let Some(sol) = m.lu().solve(&r) else {
                return Err(PointFail::Singular);
            };
            if !Self::solution_ok(&sol) {
                // У нелинейной схемы бесконечности — скорее расходимость Ньютона.
                return Err(PointFail::NoConvergence);
            }
            let conv = !limited && iter > 0 && self.converged(&x, &sol);
            x = sol;
            let sw_changed = !ctx.freeze_switches
                && iter < SWITCH_FREEZE_ITER
                && self.update_switches(&x, sw_trial);
            if conv && !sw_changed {
                self.x = x;
                return Ok(());
            }
        }
        Err(PointFail::NoConvergence)
    }

    // ───────────── Рабочая точка ─────────────

    /// Рабочая точка: прямой Ньютон → шаги по gmin → шаги по источникам.
    pub fn solve_dc(&mut self, standalone: bool, t: f64) -> Result<(), SimError> {
        let base = PointCtx {
            t,
            method: Method::Dc,
            h: 0.0,
            src_scale: 1.0,
            gmin_extra: 0.0,
            ext_frac: 1.0,
            standalone_dc: standalone,
            init_junctions: true,
            freeze_switches: false,
            max_iter: self.opts.max_iter_op,
        };
        let mut trial = self.sw_on.clone();
        self.x.fill(0.0);
        match self.solve_point(&base, &mut trial) {
            Ok(()) => {
                self.sw_on = trial;
                return Ok(());
            }
            Err(PointFail::Singular) => return Err(singular_error()),
            Err(PointFail::NoConvergence) => {}
        }
        // Шаги по gmin: большая проводимость узлов на землю делает задачу почти линейной,
        // затем её уменьшаем, каждый раз стартуя с прошлого решения.
        self.x.fill(0.0);
        let mut trial = self.sw_on.clone();
        let mut ok = true;
        let mut first = true;
        let mut g = 1e-2;
        while g >= 1e-13 {
            let ctx = PointCtx {
                gmin_extra: g,
                init_junctions: first,
                ..base
            };
            if self.solve_point(&ctx, &mut trial).is_err() {
                ok = false;
                break;
            }
            first = false;
            g /= 10.0;
        }
        if ok {
            let ctx = PointCtx {
                init_junctions: false,
                ..base
            };
            if self.solve_point(&ctx, &mut trial).is_ok() {
                self.sw_on = trial;
                return Ok(());
            }
        }
        // Шаги по источникам: от нуля до полного значения, шаг адаптивный.
        self.x.fill(0.0);
        let mut trial = self.sw_on.clone();
        let mut scale: f64 = 0.0;
        let mut ds: f64 = 0.1;
        let ctx0 = PointCtx {
            src_scale: 0.0,
            ..base
        };
        if self.solve_point(&ctx0, &mut trial).is_err() {
            return Err(SimError::NoConvergence {
                time: t,
                message: "рабочая точка не найдена даже при нулевых источниках".into(),
            });
        }
        while scale < 1.0 {
            let next = (scale + ds).min(1.0);
            let ctx = PointCtx {
                src_scale: next,
                init_junctions: false,
                ..base
            };
            let saved = self.x.clone();
            if self.solve_point(&ctx, &mut trial).is_ok() {
                scale = next;
                ds = (ds * 1.5).min(0.25);
            } else {
                self.x = saved;
                ds /= 4.0;
                if ds < 1e-6 {
                    return Err(SimError::NoConvergence {
                        time: t,
                        message: format!(
                            "рабочая точка: шаги по gmin и по источникам не помогли (дошли до {:.1}% источников)",
                            scale * 100.0
                        ),
                    });
                }
            }
        }
        self.sw_on = trial;
        Ok(())
    }

    /// После рабочей точки: состояния C и L для старта переходного процесса.
    pub fn init_reactive_from_dc(&mut self) {
        let x = &self.x;
        let v = |n: Node| idx(n).map_or(0.0, |i| x[i]);
        for e in &self.ckt.elems {
            match &e.kind {
                Kind::Cap { a, b, k, .. } => {
                    self.cap[*k] = CapState {
                        v: v(*a) - v(*b),
                        i: 0.0,
                    }
                }
                Kind::Ind { br, k, .. } => {
                    self.ind[*k] = IndState {
                        i: x[self.nn + br],
                        v: 0.0,
                    }
                }
                _ => {}
            }
        }
    }

    // ───────────── Величины элементов ─────────────

    /// Ток через элемент, А: из первого узла через элемент во второй (для МОП — втекающий в сток).
    pub fn element_current(&self, name: &str, ctx_t: f64) -> Result<f64, SimError> {
        let e = self.ckt.find_elem(name)?;
        let x = &self.x;
        let nn = self.nn;
        Ok(match &e.kind {
            Kind::Res { a, b, g } => g * (self.v(x, *a) - self.v(x, *b)),
            Kind::Cap { k, .. } => self.cap[*k].i,
            Kind::Ind { br, .. } | Kind::Vsrc { br, .. } | Kind::Vcvs { br, .. } => x[nn + br],
            Kind::Isrc { src, .. } => self.src_value(src, &self.value_ctx(ctx_t)),
            Kind::Diode(d) => {
                let vj = self.v(x, d.ja) - self.v(x, d.k);
                junction_iv(d.p.is, d.p.nvt(), vj).0 + self.opts.gmin * vj
            }
            Kind::Switch(s) => {
                let r = if self.sw_on[s.s] { s.ron } else { s.roff };
                (self.v(x, s.a) - self.v(x, s.b)) / r
            }
            Kind::Mos(_) => self.mos_terminal_currents(e)[0],
            Kind::Vccs { ca, cb, gm, .. } => gm * (self.v(x, *ca) - self.v(x, *cb)),
        })
    }

    /// Мгновенная поглощаемая мощность элемента, Вт (пассивное правило знаков: > 0 —
    /// элемент потребляет энергию; у отдающего энергию источника — < 0).
    pub fn element_power(&self, name: &str, ctx_t: f64) -> Result<f64, SimError> {
        let e = self.ckt.find_elem(name)?;
        let x = &self.x;
        let (a, b) = match &e.kind {
            Kind::Res { a, b, .. }
            | Kind::Cap { a, b, .. }
            | Kind::Ind { a, b, .. }
            | Kind::Vsrc { a, b, .. }
            | Kind::Isrc { a, b, .. }
            | Kind::Vcvs { a, b, .. }
            | Kind::Vccs { a, b, .. } => (*a, *b),
            Kind::Diode(d) => (d.a, d.k),
            Kind::Switch(s) => (s.a, s.b),
            Kind::Mos(mo) => {
                let [i_d, i_s, i_b] = self.mos_terminal_currents(e);
                return Ok(self.v(x, mo.d) * i_d + self.v(x, mo.s) * i_s + self.v(x, mo.b) * i_b);
            }
        };
        let i = self.element_current(name, ctx_t)?;
        // Ток источника тока течёт из n+ через источник в n-, то есть «вверх» по напряжению.
        Ok((self.v(x, a) - self.v(x, b)) * i)
    }

    /// Токи, втекающие в сток, исток и подложку МОП.
    fn mos_terminal_currents(&self, e: &crate::circuit::Elem) -> [f64; 3] {
        let Kind::Mos(mo) = &e.kind else {
            return [0.0; 3];
        };
        let x = &self.x;
        let sg = mo.p.kind.sign();
        let vt = thermal_voltage();
        let (vg, vd, vs, vb) = (
            self.v(x, mo.g),
            self.v(x, mo.d),
            self.v(x, mo.s),
            self.v(x, mo.b),
        );
        let ids = mos_linearize(&mo.p, sg * (vg - vs), sg * (vd - vs)).ids;
        let gmin = self.opts.gmin;
        let vbd = sg * (vb - vd);
        let vbs = sg * (vb - vs);
        let ibd = junction_iv(mo.is, vt, vbd).0 + gmin * vbd;
        let ibs = junction_iv(mo.is, vt, vbs).0 + gmin * vbs;
        // Ток перехода течёт от «анода» к «катоду»: для NMOS из подложки в сток/исток.
        [ids - sg * ibd, -ids - sg * ibs, sg * (ibd + ibs)]
    }

    pub fn value_ctx(&self, t: f64) -> PointCtx {
        PointCtx {
            t,
            method: Method::Trap,
            h: 0.0,
            src_scale: 1.0,
            gmin_extra: 0.0,
            ext_frac: 1.0,
            standalone_dc: self.standalone_dc,
            init_junctions: false,
            freeze_switches: false,
            max_iter: 0,
        }
    }

    pub fn node_voltage(&self, name: &str) -> Result<f64, SimError> {
        let n = self.ckt.find_node(name)?;
        Ok(self.v(&self.x, n))
    }

    /// Имена и значения наблюдаемых сигналов: `v(узел)`, затем `i(ветвь)` для V, L, E.
    pub fn signal_names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.ckt.node_names().map(|n| format!("v({n})")).collect();
        for e in &self.ckt.elems {
            if matches!(
                e.kind,
                Kind::Vsrc { .. } | Kind::Ind { .. } | Kind::Vcvs { .. }
            ) {
                names.push(format!("i({})", e.name));
            }
        }
        names
    }

    pub fn signal_values(&self, out: &mut Vec<f64>) {
        out.clear();
        for (i, name) in self.ckt.nodes.iter().enumerate().skip(1) {
            if !name.contains('#') {
                out.push(self.x[i - 1]);
            }
        }
        for e in &self.ckt.elems {
            match &e.kind {
                Kind::Vsrc { br, .. } | Kind::Ind { br, .. } | Kind::Vcvs { br, .. } => {
                    out.push(self.x[self.nn + br])
                }
                _ => {}
            }
        }
    }
}

pub(crate) fn singular_error() -> SimError {
    SimError::Singular(
        "проверьте, что у каждого узла есть путь к земле и нет контуров только из источников \
         напряжения и индуктивностей (или параллельно включённых источников напряжения)"
            .into(),
    )
}

/// Результат расчёта рабочей точки (`.op`).
pub struct OperatingPoint {
    eng: Engine,
}

impl OperatingPoint {
    /// Потенциал узла, В.
    pub fn voltage(&self, node: &str) -> Result<f64, SimError> {
        self.eng.node_voltage(node)
    }

    /// Ток элемента, А (см. [`crate::Transient::current`]).
    pub fn current(&self, element: &str) -> Result<f64, SimError> {
        self.eng.element_current(element, 0.0)
    }

    /// Поглощаемая мощность элемента, Вт.
    pub fn power(&self, element: &str) -> Result<f64, SimError> {
        self.eng.element_power(element, 0.0)
    }

    /// Все наблюдаемые сигналы: (`v(узел)` / `i(ветвь)`, значение).
    pub fn signals(&self) -> Vec<(String, f64)> {
        let mut vals = Vec::new();
        self.eng.signal_values(&mut vals);
        self.eng.signal_names().into_iter().zip(vals).collect()
    }
}

/// Рассчитать рабочую точку по постоянному току (источники — DC-значения).
pub fn operating_point(ckt: &Circuit) -> Result<OperatingPoint, SimError> {
    operating_point_with(ckt, SimOptions::default())
}

pub fn operating_point_with(ckt: &Circuit, opts: SimOptions) -> Result<OperatingPoint, SimError> {
    let mut eng = Engine::new(ckt, opts);
    eng.standalone_dc = true;
    eng.solve_dc(true, 0.0)?;
    Ok(OperatingPoint { eng })
}
