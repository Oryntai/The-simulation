//! Функции времени независимых источников: значение и ближайшая точка излома.
//!
//! Точки излома (углы PULSE, узлы PWL, начало SIN) решатель обязан попадать шагом точно,
//! а после них делает шаг неявным Эйлером: трапеции на изломе входа «звенят».

use crate::netlist::Waveform;

impl Waveform {
    /// Значение функции в момент `t` (для EXT — начальное значение; текущее хранит решатель).
    pub fn value(&self, t: f64) -> f64 {
        match *self {
            Waveform::Pulse {
                v1,
                v2,
                td,
                tr,
                tf,
                pw,
                per,
            } => {
                if t < td {
                    return v1;
                }
                let mut tl = t - td;
                if per.is_finite() {
                    tl -= (tl / per).floor() * per;
                }
                if tl < tr {
                    v1 + (v2 - v1) * tl / tr
                } else if tl < tr + pw {
                    v2
                } else if tl < tr + pw + tf {
                    v2 + (v1 - v2) * (tl - tr - pw) / tf
                } else {
                    v1
                }
            }
            Waveform::Sin {
                vo,
                va,
                freq,
                td,
                theta,
                phase_deg,
            } => {
                let ph = phase_deg.to_radians();
                if t < td {
                    vo + va * ph.sin()
                } else {
                    let tl = t - td;
                    vo + va * (-theta * tl).exp() * (std::f64::consts::TAU * freq * tl + ph).sin()
                }
            }
            Waveform::Pwl(ref pts) => {
                let (t0, v0) = pts[0];
                if t <= t0 {
                    return v0;
                }
                for w in pts.windows(2) {
                    let ((ta, va), (tb, vb)) = (w[0], w[1]);
                    if t <= tb {
                        return va + (vb - va) * (t - ta) / (tb - ta);
                    }
                }
                pts[pts.len() - 1].1
            }
            Waveform::Ext { initial } => initial,
        }
    }

    /// Ближайшая точка излома строго позже `t + eps`.
    pub fn next_breakpoint(&self, t: f64, eps: f64) -> Option<f64> {
        let after = t + eps;
        match *self {
            Waveform::Pulse {
                td,
                tr,
                tf,
                pw,
                per,
                ..
            } => {
                let corners = [0.0, tr, tr + pw, tr + pw + tf];
                let k0 = if per.is_finite() && after > td {
                    ((after - td) / per).floor()
                } else {
                    0.0
                };
                let periods = if per.is_finite() { 2 } else { 1 };
                for k in 0..periods {
                    let base = td + (k0 + k as f64) * if per.is_finite() { per } else { 0.0 };
                    for c in corners {
                        let tc = base + c;
                        if tc.is_finite() && tc > after {
                            return Some(tc);
                        }
                    }
                }
                None
            }
            Waveform::Sin { td, .. } => (td > after).then_some(td),
            Waveform::Pwl(ref pts) => pts.iter().map(|p| p.0).find(|&tp| tp > after),
            Waveform::Ext { .. } => None,
        }
    }

    /// Копия с нулевыми фронтами PULSE, заменёнными на `tstep` (правило SPICE).
    pub(crate) fn with_default_edges(&self, tstep: f64) -> Waveform {
        match *self {
            Waveform::Pulse {
                v1,
                v2,
                td,
                tr,
                tf,
                pw,
                per,
            } => Waveform::Pulse {
                v1,
                v2,
                td,
                tr: if tr > 0.0 { tr } else { tstep },
                tf: if tf > 0.0 { tf } else { tstep },
                pw,
                per,
            },
            ref w => w.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pulse() -> Waveform {
        Waveform::Pulse {
            v1: 0.0,
            v2: 5.0,
            td: 1.0,
            tr: 0.5,
            tf: 0.5,
            pw: 2.0,
            per: 5.0,
        }
    }

    #[test]
    fn pulse_values() {
        let p = pulse();
        assert_eq!(p.value(0.5), 0.0);
        assert_eq!(p.value(1.25), 2.5);
        assert_eq!(p.value(2.0), 5.0);
        assert_eq!(p.value(3.75), 2.5);
        assert_eq!(p.value(4.5), 0.0);
        assert_eq!(p.value(6.25), 2.5);
    }

    #[test]
    fn pulse_breakpoints() {
        let p = pulse();
        let mut t = 0.0;
        let mut got = Vec::new();
        while let Some(b) = p.next_breakpoint(t, 1e-12) {
            if b > 12.0 {
                break;
            }
            got.push(b);
            t = b;
        }
        assert_eq!(
            got,
            vec![1.0, 1.5, 3.5, 4.0, 6.0, 6.5, 8.5, 9.0, 11.0, 11.5]
        );
    }

    #[test]
    fn pwl_values_and_breaks() {
        let w = Waveform::Pwl(vec![(1.0, 0.0), (2.0, 2.0), (4.0, 0.0)]);
        assert_eq!(w.value(0.0), 0.0);
        assert_eq!(w.value(1.5), 1.0);
        assert_eq!(w.value(3.0), 1.0);
        assert_eq!(w.value(9.0), 0.0);
        assert_eq!(w.next_breakpoint(1.0, 1e-12), Some(2.0));
        assert_eq!(w.next_breakpoint(4.0, 1e-12), None);
    }
}
