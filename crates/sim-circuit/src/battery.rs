//! Li-ion ячейка: эквивалентная схема (модель Тевенина) и батарея из одинаковых ячеек.
//!
//! Схема ячейки: источник ЭДС OCV(SOC, T) → R0 → N звеньев RC (поляризация) → клеммы.
//! * SOC — кулоновский счёт относительно ёмкости при 25 °C, `Q25`;
//! * ёмкость при холоде меньше (диффузия и кинетика замедляются): доступная ёмкость
//!   `Q(T) = f(T)·Q25`. Моделируется «сжатием» шкалы: для OCV и сопротивлений берётся
//!   эффективный SOC = 1 − (1 − SOC)·Q25/Q(T). При нагреве ёмкость возвращается — как и у
//!   реальной ячейки (потеря при холоде обратима). Это приближение (оценка): реальная
//!   ячейка теряет ёмкость в основном на высоких токах и у дна разряда;
//! * R0 и R_k зависят от температуры по Аррениусу: R(T) = R(25 °C)·exp(Ea/R·(1/T − 1/298.15 К));
//! * тепло: необратимое I²·R0 + Σ V_k²/R_k и обратимое (энтропийное) −I·T·dU/dT
//!   (Bernardi, Pawlikowski, Newman, J. Electrochem. Soc. 132(1):5–12, 1985,
//!   DOI 10.1149/1.2113792); знак тока: **разряд > 0**.
//!
//! Параметры ячейки — LG INR21700-M50 (NMC811 / графит-SiOx), см. [`CellParams::lg_m50`].
//! Батарея ([`Pack`]) — s последовательно × p параллельно одинаковых ячеек при одной
//! температуре с равным делением тока (разброс ячеек и балансировка не моделируются).

use std::fmt;

/// Универсальная газовая постоянная, Дж/(моль·К).
const R_GAS: f64 = 8.314_462_618;
const T25_K: f64 = 298.15;

/// Кусочно-линейная таблица (x по возрастанию); вне диапазона — крайние значения.
#[derive(Clone, Debug, PartialEq)]
pub struct Table(pub Vec<(f64, f64)>);

impl Table {
    pub fn eval(&self, x: f64) -> f64 {
        let p = &self.0;
        if x <= p[0].0 {
            return p[0].1;
        }
        let k = p.partition_point(|q| q.0 < x);
        if k >= p.len() {
            return p[p.len() - 1].1;
        }
        let (a, b) = (p[k - 1], p[k]);
        a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
    }
}

/// Звено RC поляризации.
#[derive(Clone, Debug, PartialEq)]
pub struct RcParams {
    /// R(SOC) при 25 °C, Ом.
    pub r: Table,
    /// Ёмкость, Ф (не зависит от SOC и T — данных нет).
    pub c: f64,
}

/// Параметры ячейки.
#[derive(Clone, Debug, PartialEq)]
pub struct CellParams {
    pub name: String,
    /// Ёмкость при 25 °C (по OCV-окну 2.5–4.2 В), А·ч.
    pub capacity_ah: f64,
    /// Номинальное напряжение, В.
    pub nominal_v: f64,
    pub v_max: f64,
    pub v_min: f64,
    /// OCV(SOC) при 25 °C, В.
    pub ocv: Table,
    /// dU/dT(SOC), В/К.
    pub entropic: Table,
    /// R0(SOC) при 25 °C, Ом.
    pub r0: Table,
    pub rc: Vec<RcParams>,
    /// Энергия активации R0, Дж/моль.
    pub ea_r0: f64,
    /// Энергия активации R_k, Дж/моль.
    pub ea_rc: f64,
    /// f(T) = Q(T)/Q25 по температуре, °C.
    pub capacity_factor: Table,
    /// Диапазон температур заряда, °C.
    pub charge_temp_c: (f64, f64),
    /// Диапазон температур разряда, °C.
    pub discharge_temp_c: (f64, f64),
    /// Предельный ток заряда по температуре: (до T, °C; ток, А), по возрастанию T.
    pub max_charge_a: Vec<(f64, f64)>,
    /// Предельный длительный ток разряда по температуре: (до T, °C; ток, А).
    pub max_discharge_a: Vec<(f64, f64)>,
    pub mass_kg: f64,
    /// Теплоёмкость ячейки, Дж/К.
    pub heat_capacity_j_k: f64,
}

/// Электродные потенциалы LG M50 по Chen et al. 2020 (J. Electrochem. Soc. 167:080534,
/// DOI 10.1149/1945-7111/ab9050), ур. 9 (графит-SiOx) и ур. 8 (NMC811) — аппроксимации
/// измерений GITT; та же запись есть в наборе параметров Chen2020 библиотеки PyBaMM
/// (BSD-3-Clause), <https://github.com/pybamm-team/PyBaMM>.
fn chen2020_un(x: f64) -> f64 {
    1.9793 * (-39.3631 * x).exp() + 0.2482
        - 0.0909 * (29.8538 * (x - 0.1234)).tanh()
        - 0.04478 * (14.9159 * (x - 0.2769)).tanh()
        - 0.0205 * (30.4444 * (x - 0.6103)).tanh()
}

fn chen2020_up(y: f64) -> f64 {
    -0.8090 * y + 4.4875
        - 0.0428 * (18.5138 * (y - 0.5542)).tanh()
        - 17.7326 * (15.7890 * (y - 0.3117)).tanh()
        + 17.5842 * (15.9308 * (y - 0.3120)).tanh()
}

/// OCV ячейки LG M50 при 25 °C напрямую по Chen 2020: U_p(y) − U_n(x), стехиометрии
/// линейны по SOC между пределами из табл. VII: x 0.0279 → 0.9014, y 0.9084 → 0.2661.
pub fn lg_m50_ocv_chen2020(soc: f64) -> f64 {
    let x = 0.0279 + (0.9014 - 0.0279) * soc;
    let y = 0.9084 + (0.2661 - 0.9084) * soc;
    chen2020_up(y) - chen2020_un(x)
}

impl CellParams {
    /// LG INR21700-M50 (5 А·ч класс, 3.63 В, 18.2 Вт·ч).
    ///
    /// Источники:
    /// * даташит: LG Chem Product Specification INR21700 M50 18.2Wh, LRB-PS-CY18.2Wh-M50,
    ///   2017-09-22, <https://asset.conrad.com/media10/add/160267/c1/-/en/002114679DS01/datenblatt-2114679-lg-chem-inr21700-50t-spezial-akku-21700-flat-top-li-ion-36-v-5000-mah.pdf>:
    ///   4.20/2.50 В, заряд 0…45 °C (до 25 °C не более 0.3C = 1.455 А, 25…45 °C — 0.7C =
    ///   3.395 А), разряд −20…55 °C (§2.7; в §5.1 — до 60 °C), длительный разряд 1.5C =
    ///   7.275 А (при −20…10 °C — 0.5C), DCIR 30 ± 6 мОм (50% SOC, 0.5C, 30 с), масса 68.5 г;
    /// * OCV — Chen et al. 2020 (см. [`lg_m50_ocv_chen2020`]); таблица 0…100% строится из формул;
    /// * R0(z), R1(z), C1 = 2913.1 Ф — Lone, Atlan, Fasolato, Raimondo, Drummond, arXiv:2508.14454
    ///   (2025, препринт, без рецензии), табл. I, HPPC на LG M50T при 25 °C. Полином R1(z)
    ///   выше ~70% SOC уходит к нулю и в минус — нефизично; держим R1 = 10 мОм при SOC ≥ 60%
    ///   (оценка, порядок согласован с медленной частью R(30 с) − R(1 с) ≈ 11 мОм по данным
    ///   Brosa Planella et al. 2021 при ~100% SOC);
    /// * температура: Brosa Planella, Sheikh, Widanage, Electrochim. Acta 388:138524 (2021),
    ///   DOI 10.1016/j.electacta.2021.138524, открытые данные
    ///   <https://github.com/brosaplanella/TEC-reduced-model> (Zenodo 10.5281/zenodo.4864437):
    ///   по скачку напряжения в начале разряда 1C при 0/10/25 °C — Ea(R за 1 с) ≈ 15.0 кДж/моль,
    ///   медленная часть почти не зависит от T (Ea ≈ 1.9 кДж/моль) — наш расчёт по их данным;
    ///   ёмкость C/10: 4.900 А·ч (25 °C), 4.696 (10 °C), 4.425 (0 °C) → f(T) = 1, 0.958, 0.903;
    ///   ниже 0 °C — линейная экстраполяция (оценка), даташит гарантирует ≥ 70% энергии при −10 °C;
    /// * dU/dT — расчёт по аппроксимациям O'Regan et al., Electrochim. Acta 425:140700 (2022),
    ///   DOI 10.1016/j.electacta.2022.140700, ур. 16–17, со стехиометриями Chen 2020;
    /// * теплоёмкость: 0.0685 кг × 887 Дж/(кг·К) (удельная теплоёмкость рулона, O'Regan 2022) ≈
    ///   61 Дж/К — оценка для ячейки целиком.
    pub fn lg_m50() -> CellParams {
        // Узлы таблицы: у дна разряда кривая круто изгибается — шаг 0.25%, дальше 1%.
        let soc_pts: Vec<f64> = (0..20)
            .map(|k| k as f64 * 0.0025)
            .chain((5..=100).map(|k| k as f64 * 0.01))
            .collect();
        let ocv = Table(
            soc_pts
                .iter()
                .map(|&z| (z, lg_m50_ocv_chen2020(z)))
                .collect(),
        );
        let r0 = Table(
            (0..=20)
                .map(|k| {
                    let z = k as f64 / 20.0;
                    (z, -0.056 * z * z * z + 0.116 * z * z - 0.073 * z + 0.0393)
                })
                .collect(),
        );
        let r1 = Table(
            (0..=12)
                .map(|k| {
                    let z = k as f64 / 20.0;
                    (z, -0.02248 * z * z - 0.01228 * z + 0.02551)
                })
                .chain([(1.0, -0.02248 * 0.36 - 0.01228 * 0.6 + 0.02551)])
                .collect(),
        );
        let entropic = Table(
            [
                -0.25, -0.36, -0.09, -0.04, -0.01, 0.02, 0.03, 0.04, 0.05, 0.07, 0.10,
            ]
            .iter()
            .enumerate()
            .map(|(k, mv)| (k as f64 / 10.0, mv * 1e-3))
            .collect(),
        );
        CellParams {
            name: "LG INR21700-M50".into(),
            capacity_ah: 4.900,
            nominal_v: 3.63,
            v_max: 4.20,
            v_min: 2.50,
            ocv,
            entropic,
            r0,
            rc: vec![RcParams { r: r1, c: 2913.1 }],
            ea_r0: 15.0e3,
            ea_rc: 1.9e3,
            capacity_factor: Table(vec![
                (-20.0, 0.793),
                (-10.0, 0.848),
                (0.0, 0.903),
                (10.0, 0.958),
                (25.0, 1.0),
                (60.0, 1.0),
            ]),
            charge_temp_c: (0.0, 45.0),
            discharge_temp_c: (-20.0, 55.0),
            max_charge_a: vec![(25.0, 1.455), (45.0, 3.395)],
            max_discharge_a: vec![(10.0, 2.425), (55.0, 7.275)],
            mass_kg: 0.0685,
            heat_capacity_j_k: 0.0685 * 887.0,
        }
    }

    fn arrhenius(ea: f64, temp_c: f64) -> f64 {
        (ea / R_GAS * (1.0 / (temp_c + 273.15) - 1.0 / T25_K)).exp()
    }

    /// Доступная ёмкость при температуре, А·ч.
    pub fn available_capacity_ah(&self, temp_c: f64) -> f64 {
        self.capacity_ah * self.capacity_factor.eval(temp_c)
    }

    /// R0 при данных эффективном SOC и температуре, Ом.
    pub fn r0_at(&self, soc_eff: f64, temp_c: f64) -> f64 {
        self.r0.eval(soc_eff) * Self::arrhenius(self.ea_r0, temp_c)
    }

    /// R_k при данных эффективном SOC и температуре, Ом.
    pub fn rc_r_at(&self, k: usize, soc_eff: f64, temp_c: f64) -> f64 {
        self.rc[k].r.eval(soc_eff) * Self::arrhenius(self.ea_rc, temp_c)
    }

    /// OCV с энтропийной поправкой, В.
    pub fn ocv_at(&self, soc_eff: f64, temp_c: f64) -> f64 {
        self.ocv.eval(soc_eff) + self.entropic.eval(soc_eff) * (temp_c - 25.0)
    }

    fn limit_for(table: &[(f64, f64)], temp_c: f64) -> f64 {
        table
            .iter()
            .find(|(t, _)| temp_c <= *t)
            .or(table.last())
            .map_or(f64::INFINITY, |p| p.1)
    }
}

/// Предупреждение о выходе за пределы, заданные производителем.
#[derive(Clone, Debug, PartialEq)]
pub enum CellWarning {
    /// Заряд ниже допустимой температуры (литий осаждается на аноде).
    ChargeTooCold {
        temp_c: f64,
        min_c: f64,
    },
    ChargeTooHot {
        temp_c: f64,
        max_c: f64,
    },
    DischargeTooCold {
        temp_c: f64,
        min_c: f64,
    },
    DischargeTooHot {
        temp_c: f64,
        max_c: f64,
    },
    ChargeCurrentHigh {
        current_a: f64,
        max_a: f64,
    },
    DischargeCurrentHigh {
        current_a: f64,
        max_a: f64,
    },
    OverVoltage {
        v: f64,
        max: f64,
    },
    UnderVoltage {
        v: f64,
        min: f64,
    },
}

impl fmt::Display for CellWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CellWarning::ChargeTooCold { temp_c, min_c } => write!(
                f,
                "заряд при {temp_c:.1} °C ниже допустимых {min_c:.0} °C: осаждение металлического \
                 лития на аноде (Waldmann et al., J. Power Sources 384:107, 2018, \
                 DOI 10.1016/j.jpowsour.2018.02.063) — ёмкость теряется необратимо, риск КЗ"
            ),
            CellWarning::ChargeTooHot { temp_c, max_c } => {
                write!(f, "заряд при {temp_c:.1} °C выше допустимых {max_c:.0} °C")
            }
            CellWarning::DischargeTooCold { temp_c, min_c } => {
                write!(f, "разряд при {temp_c:.1} °C ниже допустимых {min_c:.0} °C")
            }
            CellWarning::DischargeTooHot { temp_c, max_c } => {
                write!(f, "разряд при {temp_c:.1} °C выше допустимых {max_c:.0} °C")
            }
            CellWarning::ChargeCurrentHigh { current_a, max_a } => write!(
                f,
                "ток заряда {current_a:.2} А больше допустимых {max_a:.2} А при этой температуре"
            ),
            CellWarning::DischargeCurrentHigh { current_a, max_a } => write!(
                f,
                "ток разряда {current_a:.2} А больше допустимого длительного {max_a:.2} А"
            ),
            CellWarning::OverVoltage { v, max } => {
                write!(
                    f,
                    "напряжение {v:.3} В выше предельного {max:.2} В (перезаряд)"
                )
            }
            CellWarning::UnderVoltage { v, min } => {
                write!(
                    f,
                    "напряжение {v:.3} В ниже конечного {min:.2} В (переразряд)"
                )
            }
        }
    }
}

/// Ячейка: состояние модели.
#[derive(Clone, Debug)]
pub struct Cell {
    p: CellParams,
    /// Отданный заряд от полного, А·ч.
    removed_ah: f64,
    v_rc: Vec<f64>,
    current: f64,
    temp_c: f64,
}

impl Cell {
    /// Полностью заряженная отдохнувшая ячейка при 25 °C.
    pub fn new(params: CellParams) -> Cell {
        Cell::with_soc(params, 1.0)
    }

    pub fn with_soc(params: CellParams, soc: f64) -> Cell {
        let n = params.rc.len();
        Cell {
            removed_ah: (1.0 - soc) * params.capacity_ah,
            p: params,
            v_rc: vec![0.0; n],
            current: 0.0,
            temp_c: 25.0,
        }
    }

    pub fn params(&self) -> &CellParams {
        &self.p
    }

    /// Шаг: ток `current_a` (разряд > 0) постоянен на интервале `dt`, температура `temp_c`.
    /// Звенья RC интегрируются точно (экспонента), SOC — кулоновским счётом.
    /// Возвращает предупреждения о пределах производителя.
    pub fn step(&mut self, current_a: f64, dt: f64, temp_c: f64) -> Vec<CellWarning> {
        self.temp_c = temp_c;
        let z = self.effective_soc();
        for k in 0..self.v_rc.len() {
            let r = self.p.rc_r_at(k, z, temp_c);
            let tau = r * self.p.rc[k].c;
            let a = (-dt / tau).exp();
            self.v_rc[k] = self.v_rc[k] * a + r * current_a * (1.0 - a);
        }
        self.removed_ah += current_a * dt / 3600.0;
        self.current = current_a;
        let mut w = self.check_limits(current_a, temp_c);
        let v = self.terminal_voltage();
        if v > self.p.v_max + 1e-9 {
            w.push(CellWarning::OverVoltage {
                v,
                max: self.p.v_max,
            });
        }
        if v < self.p.v_min - 1e-9 {
            w.push(CellWarning::UnderVoltage {
                v,
                min: self.p.v_min,
            });
        }
        w
    }

    /// Проверка тока и температуры по даташиту (без шага).
    pub fn check_limits(&self, current_a: f64, temp_c: f64) -> Vec<CellWarning> {
        let p = &self.p;
        let mut w = Vec::new();
        if current_a < 0.0 {
            if temp_c < p.charge_temp_c.0 {
                w.push(CellWarning::ChargeTooCold {
                    temp_c,
                    min_c: p.charge_temp_c.0,
                });
            }
            if temp_c > p.charge_temp_c.1 {
                w.push(CellWarning::ChargeTooHot {
                    temp_c,
                    max_c: p.charge_temp_c.1,
                });
            }
            let max = CellParams::limit_for(&p.max_charge_a, temp_c);
            if -current_a > max {
                w.push(CellWarning::ChargeCurrentHigh {
                    current_a: -current_a,
                    max_a: max,
                });
            }
        } else if current_a > 0.0 {
            if temp_c < p.discharge_temp_c.0 {
                w.push(CellWarning::DischargeTooCold {
                    temp_c,
                    min_c: p.discharge_temp_c.0,
                });
            }
            if temp_c > p.discharge_temp_c.1 {
                w.push(CellWarning::DischargeTooHot {
                    temp_c,
                    max_c: p.discharge_temp_c.1,
                });
            }
            let max = CellParams::limit_for(&p.max_discharge_a, temp_c);
            if current_a > max {
                w.push(CellWarning::DischargeCurrentHigh {
                    current_a,
                    max_a: max,
                });
            }
        }
        w
    }

    /// SOC кулоновского счёта относительно ёмкости при 25 °C (0…1; может выйти за пределы
    /// при пере(раз)ряде).
    pub fn soc(&self) -> f64 {
        1.0 - self.removed_ah / self.p.capacity_ah
    }

    /// Эффективный SOC при текущей температуре (для OCV и сопротивлений), 0…1.
    pub fn effective_soc(&self) -> f64 {
        let q = self.p.available_capacity_ah(self.temp_c);
        (1.0 - self.removed_ah / q).clamp(0.0, 1.0)
    }

    /// Отданный от полного заряд, А·ч.
    pub fn removed_ah(&self) -> f64 {
        self.removed_ah
    }

    /// ЭДС (напряжение разомкнутой цепи после полного отдыха), В.
    pub fn ocv(&self) -> f64 {
        self.p.ocv_at(self.effective_soc(), self.temp_c)
    }

    /// Напряжение на клеммах при токе последнего шага, В.
    pub fn terminal_voltage(&self) -> f64 {
        self.terminal_voltage_at(self.current)
    }

    /// Напряжение на клеммах, если в текущем состоянии мгновенно задать ток `current_a`.
    pub fn terminal_voltage_at(&self, current_a: f64) -> f64 {
        self.ocv() - current_a * self.r0() - self.polarization_v()
    }

    /// Сумма напряжений звеньев RC, В.
    pub fn polarization_v(&self) -> f64 {
        self.v_rc.iter().sum()
    }

    /// Внутреннее омическое сопротивление в текущем состоянии, Ом.
    pub fn r0(&self) -> f64 {
        self.p.r0_at(self.effective_soc(), self.temp_c)
    }

    /// Ток последнего шага, А.
    pub fn current(&self) -> f64 {
        self.current
    }

    pub fn temperature_c(&self) -> f64 {
        self.temp_c
    }

    /// Необратимое тепло (джоулево в R0 и в резисторах звеньев RC), Вт.
    pub fn irreversible_heat_w(&self) -> f64 {
        let z = self.effective_soc();
        let mut q = self.current * self.current * self.r0();
        for (k, v) in self.v_rc.iter().enumerate() {
            q += v * v / self.p.rc_r_at(k, z, self.temp_c);
        }
        q
    }

    /// Обратимое (энтропийное) тепло −I·T·dU/dT, Вт (может быть отрицательным).
    pub fn reversible_heat_w(&self) -> f64 {
        -self.current * (self.temp_c + 273.15) * self.p.entropic.eval(self.effective_soc())
    }

    /// Полное тепловыделение, Вт.
    pub fn heat_w(&self) -> f64 {
        self.irreversible_heat_w() + self.reversible_heat_w()
    }

    /// Энергия, запасённая в конденсаторах звеньев RC, Дж.
    pub fn rc_energy_j(&self) -> f64 {
        self.v_rc
            .iter()
            .zip(&self.p.rc)
            .map(|(v, rc)| 0.5 * rc.c * v * v)
            .sum()
    }

    /// Доступная ёмкость при температуре, А·ч.
    pub fn available_capacity_ah(&self, temp_c: f64) -> f64 {
        self.p.available_capacity_ah(temp_c)
    }
}

/// Батарея s×p одинаковых ячеек.
#[derive(Clone, Debug)]
pub struct Pack {
    cell: Cell,
    pub series: usize,
    pub parallel: usize,
}

impl Pack {
    pub fn new(params: CellParams, series: usize, parallel: usize) -> Pack {
        assert!(series > 0 && parallel > 0, "батарея: s и p должны быть ≥ 1");
        Pack {
            cell: Cell::new(params),
            series,
            parallel,
        }
    }

    pub fn with_soc(params: CellParams, series: usize, parallel: usize, soc: f64) -> Pack {
        let mut p = Pack::new(params, series, parallel);
        p.cell = Cell::with_soc(p.cell.p.clone(), soc);
        p
    }

    /// Шаг батареи: ток батареи (разряд > 0) делится поровну между p ветвями.
    pub fn step(&mut self, current_a: f64, dt: f64, temp_c: f64) -> Vec<CellWarning> {
        self.cell.step(current_a / self.parallel as f64, dt, temp_c)
    }

    pub fn terminal_voltage(&self) -> f64 {
        self.series as f64 * self.cell.terminal_voltage()
    }

    pub fn ocv(&self) -> f64 {
        self.series as f64 * self.cell.ocv()
    }

    /// Омическое сопротивление батареи (без межъячеечных перемычек), Ом.
    pub fn r0(&self) -> f64 {
        self.series as f64 * self.cell.r0() / self.parallel as f64
    }

    pub fn soc(&self) -> f64 {
        self.cell.soc()
    }

    pub fn heat_w(&self) -> f64 {
        (self.series * self.parallel) as f64 * self.cell.heat_w()
    }

    pub fn available_capacity_ah(&self, temp_c: f64) -> f64 {
        self.parallel as f64 * self.cell.available_capacity_ah(temp_c)
    }

    /// Номинальная энергия, Вт·ч.
    pub fn nominal_energy_wh(&self) -> f64 {
        let p = &self.cell.p;
        (self.series * self.parallel) as f64 * p.capacity_ah * p.nominal_v
    }

    pub fn mass_kg(&self) -> f64 {
        (self.series * self.parallel) as f64 * self.cell.p.mass_kg
    }

    /// Ячейка-представитель (все ячейки одинаковы).
    pub fn cell(&self) -> &Cell {
        &self.cell
    }
}

/// Разряд постоянным током до конечного напряжения при постоянной температуре:
/// (отданный заряд, А·ч; отданная энергия, Вт·ч).
pub fn discharge_to_cutoff(
    params: &CellParams,
    current_a: f64,
    temp_c: f64,
    dt: f64,
) -> (f64, f64) {
    let mut cell = Cell::new(params.clone());
    let mut energy_wh = 0.0;
    let mut v_prev = cell.terminal_voltage_at(current_a);
    loop {
        let q_before = cell.removed_ah;
        cell.step(current_a, dt, temp_c);
        let v = cell.terminal_voltage();
        if v <= params.v_min {
            // Доля шага до пересечения конечного напряжения (линейно).
            let frac = ((v_prev - params.v_min) / (v_prev - v)).clamp(0.0, 1.0);
            let dq = (cell.removed_ah - q_before) * frac;
            energy_wh += 0.5 * (v_prev + params.v_min) * dq;
            return (q_before + dq, energy_wh);
        }
        energy_wh += 0.5 * (v_prev + v) * (cell.removed_ah - q_before);
        v_prev = v;
        if cell.removed_ah > 2.0 * params.capacity_ah {
            return (cell.removed_ah, energy_wh);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ocv_table_matches_formula() {
        let p = CellParams::lg_m50();
        for k in 0..=200 {
            let z = k as f64 / 200.0;
            let d = (p.ocv.eval(z) - lg_m50_ocv_chen2020(z)).abs();
            assert!(d < 3e-3, "SOC {z}: таблица отличается от формулы на {d} В");
        }
        assert!((lg_m50_ocv_chen2020(0.5) - 3.708).abs() < 1e-3);
    }

    #[test]
    fn charging_cold_warns() {
        let mut c = Cell::with_soc(CellParams::lg_m50(), 0.5);
        let w = c.step(-1.0, 1.0, -5.0);
        assert!(
            w.iter()
                .any(|w| matches!(w, CellWarning::ChargeTooCold { .. }))
        );
        assert!(w[0].to_string().contains("лития"));
        let w = c.step(-1.0, 1.0, 20.0);
        assert!(w.is_empty(), "{w:?}");
        let w = c.step(-2.0, 1.0, 20.0);
        assert!(matches!(w[0], CellWarning::ChargeCurrentHigh { .. }));
    }

    #[test]
    fn pack_scales() {
        let mut pk = Pack::new(CellParams::lg_m50(), 3, 2);
        pk.step(4.0, 10.0, 25.0);
        let c = pk.cell().clone();
        assert!((pk.terminal_voltage() - 3.0 * c.terminal_voltage()).abs() < 1e-12);
        assert!((c.current() - 2.0).abs() < 1e-15);
        assert!((pk.heat_w() - 6.0 * c.heat_w()).abs() < 1e-12);
        assert!((pk.available_capacity_ah(25.0) - 9.8).abs() < 1e-12);
    }
}
