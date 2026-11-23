//! Нелинейные приборы: диод Шокли и МОП-транзистор уровня 1 (Шихман–Ходжес), плюс
//! ограничители шага Ньютона.
//!
//! Ограничители `pnjlim`, `fetlim`, `limvds` реализованы по алгоритмам Berkeley SPICE
//! (L. W. Nagel, «SPICE2: A Computer Program to Simulate Semiconductor Circuits», UCB/ERL
//! M520, 1975, §4 — ограничение напряжения p-n перехода; T. Quarles, «Analysis of Performance
//! and Convergence Issues for Circuit Simulation», UCB/ERL M89/42, 1989 — fetlim/limvds).
//! Эти же алгоритмы лежат в `devsup.c` SPICE3/ngspice (лицензия BSD-3-Clause, Regents of
//! the University of California); здесь — собственная запись по описанию алгоритма.

/// Постоянная Больцмана, Дж/К (точное значение СИ-2019).
pub const BOLTZMANN: f64 = 1.380_649e-23;
/// Элементарный заряд, Кл (точное значение СИ-2019).
pub const CHARGE: f64 = 1.602_176_634e-19;
/// Номинальная температура приборов, К (27 °C — TNOM по умолчанию в SPICE).
pub const T_NOM: f64 = 300.15;

/// Тепловой потенциал kT/q при T_NOM, В (≈ 25.86 мВ).
pub fn thermal_voltage() -> f64 {
    BOLTZMANN * T_NOM / CHARGE
}

/// Ток и дифференциальная проводимость идеального перехода `is·(exp(v/nvt) − 1)`.
/// Выше 80·nvt экспонента продолжается касательной, чтобы не было переполнения при
/// неудачной итерации (на сошедшемся решении так далеко переход не открывается).
pub fn junction_iv(is: f64, nvt: f64, v: f64) -> (f64, f64) {
    const XMAX: f64 = 80.0;
    let x = v / nvt;
    if x > XMAX {
        let em = XMAX.exp();
        (is * (em * (1.0 + x - XMAX) - 1.0), is * em / nvt)
    } else {
        let e = x.exp();
        (is * (e - 1.0), is * e / nvt)
    }
}

/// Критическое напряжение перехода: выше него ограничитель `pnjlim` начинает работать.
pub fn vcrit(is: f64, nvt: f64) -> f64 {
    nvt * (nvt / (std::f64::consts::SQRT_2 * is)).ln()
}

/// Ограничение шага Ньютона для p-n перехода (Nagel 1975). Возвращает новое напряжение и
/// флаг «было ограничено» (тогда итерация не может считаться сошедшейся).
pub fn pnjlim(vnew: f64, vold: f64, nvt: f64, vcrit: f64) -> (f64, bool) {
    if vnew > vcrit && (vnew - vold).abs() > 2.0 * nvt {
        if vold > 0.0 {
            let arg = 1.0 + (vnew - vold) / nvt;
            if arg > 0.0 {
                (vold + nvt * arg.ln(), true)
            } else {
                (vcrit, true)
            }
        } else {
            (nvt * (vnew / nvt).ln(), true)
        }
    } else {
        (vnew, false)
    }
}

/// Ограничение шага по напряжению затвор–исток (SPICE3 `fetlim`, Quarles 1989).
pub fn fetlim(vnew: f64, vold: f64, vto: f64) -> f64 {
    let vtsthi = (2.0 * (vold - vto)).abs() + 2.0;
    let vtstlo = vtsthi / 2.0 + 2.0;
    let vtox = vto + 3.5;
    let delv = vnew - vold;
    let mut v = vnew;
    if vold >= vto {
        if vold >= vtox {
            if delv <= 0.0 {
                if vnew >= vtox {
                    if -delv > vtstlo {
                        v = vold - vtstlo;
                    }
                } else {
                    v = vnew.max(vto + 2.0);
                }
            } else if delv >= vtsthi {
                v = vold + vtsthi;
            }
        } else if delv <= 0.0 {
            v = vnew.max(vto - 0.5);
        } else {
            v = vnew.min(vto + 4.0);
        }
    } else if delv <= 0.0 {
        if -delv > vtsthi {
            v = vold - vtsthi;
        }
    } else {
        let vtemp = vto + 0.5;
        if vnew <= vtemp {
            if delv > vtstlo {
                v = vold + vtstlo;
            }
        } else {
            v = vtemp;
        }
    }
    v
}

/// Ограничение шага по напряжению сток–исток (SPICE3 `limvds`).
pub fn limvds(vnew: f64, vold: f64) -> f64 {
    if vold >= 3.5 {
        if vnew > vold {
            vnew.min(3.0 * vold + 2.0)
        } else if vnew < 3.5 {
            vnew.max(2.0)
        } else {
            vnew
        }
    } else if vnew > vold {
        vnew.min(4.0)
    } else {
        vnew.max(-0.5)
    }
}

/// Параметры диода после учёта площади.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiodeParams {
    /// Ток насыщения, А (SPICE IS, по умолчанию 1e-14).
    pub is: f64,
    /// Коэффициент неидеальности (N, по умолчанию 1).
    pub n: f64,
    /// Последовательное сопротивление, Ом (RS, по умолчанию 0).
    pub rs: f64,
}

impl DiodeParams {
    pub fn nvt(&self) -> f64 {
        self.n * thermal_voltage()
    }
}

/// Полярность МОП-транзистора.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MosType {
    N,
    P,
}

impl MosType {
    pub fn sign(self) -> f64 {
        match self {
            MosType::N => 1.0,
            MosType::P => -1.0,
        }
    }
}

/// Параметры МОП-транзистора уровня 1 (Шихман–Ходжес, IEEE JSSC 3(5):285–289, 1968,
/// DOI 10.1109/JSSC.1968.1049902), в форме SPICE:
/// * насыщение: Id = KP/2·W/L·(Vgs−Vto)²·(1+λVds);
/// * линейная область: Id = KP·W/L·(Vgs−Vto−Vds/2)·Vds·(1+λVds).
///
/// Не моделируются: эффект подложки (GAMMA/PHI — порог постоянен), ёмкости затвора и
/// переходов (переключение мгновенное по входу), подпороговый ток, температура.
/// Диоды подложка–сток и подложка–исток (IS из модели, по умолчанию 1e-14 А) есть — это и
/// «встроенный диод» силового ключа, если подложка соединена с истоком.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MosParams {
    pub kind: MosType,
    /// Порог в «собственной» полярности (для PMOS — отрицательный, как в SPICE).
    pub vto: f64,
    /// KP·W/L, А/В².
    pub beta: f64,
    pub lambda: f64,
}

/// Ток канала и производные в нормальном режиме (vds ≥ 0), в NMOS-нормированных величинах.
pub fn mos_channel(beta: f64, vto_n: f64, lambda: f64, vgs: f64, vds: f64) -> (f64, f64, f64) {
    let vgst = vgs - vto_n;
    if vgst <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    let clm = 1.0 + lambda * vds;
    if vds < vgst {
        let core = vgst * vds - 0.5 * vds * vds;
        let ids = beta * core * clm;
        let gm = beta * vds * clm;
        let gds = beta * (vgst - vds) * clm + beta * lambda * core;
        (ids, gm, gds)
    } else {
        let ids = 0.5 * beta * vgst * vgst * clm;
        let gm = beta * vgst * clm;
        let gds = 0.5 * beta * vgst * vgst * lambda;
        (ids, gm, gds)
    }
}

/// Результат линеаризации канала МОП: ток сток→исток и его частные производные по
/// потенциалам затвора, стока и истока.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MosLinear {
    pub ids: f64,
    pub dg: f64,
    pub dd: f64,
    pub ds: f64,
}

/// Ток канала (сток→исток, реальные знаки) по нормированным vgs/vds (с учётом прямого и
/// инверсного режимов). `vgs_n`, `vds_n` — уже умноженные на знак полярности.
pub fn mos_linearize(p: &MosParams, vgs_n: f64, vds_n: f64) -> MosLinear {
    let s = p.kind.sign();
    let vto_n = s * p.vto;
    if vds_n >= 0.0 {
        let (i, gm, gds) = mos_channel(p.beta, vto_n, p.lambda, vgs_n, vds_n);
        MosLinear {
            ids: s * i,
            dg: gm,
            dd: gds,
            ds: -gm - gds,
        }
    } else {
        // Инверсный режим: сток и исток меняются ролями.
        let vgd_n = vgs_n - vds_n;
        let (i, gm, gds) = mos_channel(p.beta, vto_n, p.lambda, vgd_n, -vds_n);
        MosLinear {
            ids: -s * i,
            dg: -gm,
            ds: -gds,
            dd: gm + gds,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pnjlim_limits_large_steps() {
        let nvt = thermal_voltage();
        let vc = vcrit(1e-14, nvt);
        assert!((0.6..0.75).contains(&vc), "vcrit = {vc}");
        let (v, lim) = pnjlim(5.0, 0.6, nvt, vc);
        assert!(lim && v < 0.8 && v > 0.6);
        let (v, lim) = pnjlim(0.3, 0.2, nvt, vc);
        assert!(!lim && v == 0.3);
    }

    #[test]
    fn mos_derivatives_match_finite_differences() {
        for kind in [MosType::N, MosType::P] {
            let p = MosParams {
                kind,
                vto: kind.sign() * 1.0,
                beta: 2e-3,
                lambda: 0.02,
            };
            for &(vg, vd, vs) in &[
                (3.0, 5.0, 0.0),
                (3.0, 0.5, 0.0),
                (2.0, -1.0, 0.0),
                (-3.0, -5.0, 0.0),
            ] {
                let f = |vg: f64, vd: f64, vs: f64| {
                    let sg = kind.sign();
                    mos_linearize(&p, sg * (vg - vs), sg * (vd - vs)).ids
                };
                let l = {
                    let sg = kind.sign();
                    mos_linearize(&p, sg * (vg - vs), sg * (vd - vs))
                };
                let h = 1e-6;
                let dg = (f(vg + h, vd, vs) - f(vg - h, vd, vs)) / (2.0 * h);
                let dd = (f(vg, vd + h, vs) - f(vg, vd - h, vs)) / (2.0 * h);
                let ds = (f(vg, vd, vs + h) - f(vg, vd, vs - h)) / (2.0 * h);
                assert!((dg - l.dg).abs() < 1e-7, "{kind:?} dg {dg} vs {}", l.dg);
                assert!((dd - l.dd).abs() < 1e-7, "{kind:?} dd {dd} vs {}", l.dd);
                assert!((ds - l.ds).abs() < 1e-7, "{kind:?} ds {ds} vs {}", l.ds);
            }
        }
    }
}
