//! Стабилизаторы: вход, выход, ток, КПД или падение, нагрев.
//!
//! Линейный стабилизатор: I_вх = I_вых + I_q, потери (V_вх − V_вых)·I_вых + V_вх·I_q.
//! Понижающий: P_вх = V_вых·I_вых/η + V_вх·I_q. Все потери понижающего условно отнесены
//! к микросхеме, хотя часть выделяется в диоде и дросселе: это запас.

use crate::spec::Spec;

/// Вид стабилизатора.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegulatorKind {
    Linear,
    Buck,
}

impl RegulatorKind {
    pub fn title(self) -> &'static str {
        match self {
            RegulatorKind::Linear => "линейный",
            RegulatorKind::Buck => "понижающий импульсный",
        }
    }
}

/// Стабилизатор из каталога.
#[derive(Clone, Debug)]
pub struct Regulator {
    pub id: &'static str,
    pub name: &'static str,
    pub kind: RegulatorKind,
    pub what_is: &'static str,
    pub input_min_v: Spec,
    pub input_max_v: Spec,
    pub input_abs_max_v: Spec,
    pub output_min_v: Spec,
    pub output_max_v: Spec,
    pub output_current_a: Spec,
    pub current_limit_min_a: Spec,
    /// Минимальная разность вход − выход (у импульсных — насыщение ключа), В.
    pub dropout_v: Spec,
    /// Наибольшая скважность ключа.
    pub max_duty: Spec,
    /// Минимальная нагрузка, без которой напряжение не гарантировано, А.
    pub min_load_a: Spec,
    pub quiescent_a: Spec,
    /// Паспортные точки КПД: (выходное напряжение, В; КПД при полной нагрузке).
    pub efficiency_points: &'static [(f64, f64)],
    /// КПД, если паспортных точек нет.
    pub efficiency_fallback: Spec,
    pub r_theta_ja_k_per_w: Spec,
    pub t_j_max_c: Spec,
    pub t_shutdown_c: Spec,
    pub sources: &'static [&'static str],
    pub caveats: &'static [&'static str],
}

/// Режим стабилизатора в рабочей точке.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegulatorOperation {
    pub input_current_a: f64,
    pub input_power_w: f64,
    pub loss_w: f64,
    /// `false`, если КПД — оценка.
    pub proven: bool,
}

impl Regulator {
    /// Наименьшее входное напряжение для выхода `v_out`, В.
    pub fn required_input_v(&self, v_out: f64) -> f64 {
        let duty = self.max_duty.get().unwrap_or(1.0);
        let drop = self.dropout_v.get().unwrap_or(0.0);
        (v_out / duty + drop).max(self.input_min_v.get().unwrap_or(0.0))
    }

    /// КПД понижающего при выходе `v_out`: ближайшая паспортная точка снизу.
    pub fn efficiency_at(&self, v_out: f64) -> Spec {
        let p = self.efficiency_points;
        match p.iter().rev().find(|(v, _)| *v <= v_out).or(p.first()) {
            Some(&(_, eta)) => Spec::est(
                eta,
                "типовая паспортная точка при полной нагрузке, не гарантированный минимум",
            ),
            None => self.efficiency_fallback,
        }
    }

    /// Токи и потери в рабочей точке. `None` — вход некорректен или нет КПД.
    pub fn operating(&self, v_in: f64, v_out: f64, i_out: f64) -> Option<RegulatorOperation> {
        if !(v_in.is_finite() && v_in > 0.0 && v_out.is_finite() && v_out > 0.0) {
            return None;
        }
        if !(i_out.is_finite() && i_out >= 0.0) {
            return None;
        }
        let iq = self.quiescent_a.get().unwrap_or(0.0);
        let p_out = v_out * i_out;
        let (p_in, proven) = match self.kind {
            RegulatorKind::Linear => (v_in * (i_out + iq), self.quiescent_a.is_proof()),
            RegulatorKind::Buck => {
                let eta = self.efficiency_at(v_out);
                let e = eta.get().filter(|e| *e > 0.0 && *e <= 1.0)?;
                (
                    p_out / e + v_in * iq,
                    eta.is_proof() && self.quiescent_a.is_proof(),
                )
            }
        };
        Some(RegulatorOperation {
            input_current_a: p_in / v_in,
            input_power_w: p_in,
            loss_w: (p_in - p_out).max(0.0),
            proven,
        })
    }

    /// Предельная температура кристалла: меньшая из максимума и порога защиты, °C.
    pub fn junction_limit_c(&self) -> Option<f64> {
        match (self.t_j_max_c.get(), self.t_shutdown_c.get()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    /// Все числовые поля — для проверки на NaN/∞ и знак.
    pub fn numeric_specs(&self) -> Vec<(&'static str, Spec)> {
        vec![
            ("input_min_v", self.input_min_v),
            ("input_max_v", self.input_max_v),
            ("input_abs_max_v", self.input_abs_max_v),
            ("output_min_v", self.output_min_v),
            ("output_max_v", self.output_max_v),
            ("output_current_a", self.output_current_a),
            ("current_limit_min_a", self.current_limit_min_a),
            ("dropout_v", self.dropout_v),
            ("max_duty", self.max_duty),
            ("min_load_a", self.min_load_a),
            ("quiescent_a", self.quiescent_a),
            ("efficiency_fallback", self.efficiency_fallback),
            ("r_theta_ja_k_per_w", self.r_theta_ja_k_per_w),
            ("t_j_max_c", self.t_j_max_c),
            ("t_shutdown_c", self.t_shutdown_c),
        ]
    }

    fn blank(
        id: &'static str,
        name: &'static str,
        kind: RegulatorKind,
        what_is: &'static str,
    ) -> Self {
        Self {
            id,
            name,
            kind,
            what_is,
            input_min_v: Spec::NONE,
            input_max_v: Spec::NONE,
            input_abs_max_v: Spec::NONE,
            output_min_v: Spec::NONE,
            output_max_v: Spec::NONE,
            output_current_a: Spec::NONE,
            current_limit_min_a: Spec::NONE,
            dropout_v: Spec::NONE,
            max_duty: Spec::NONE,
            min_load_a: Spec::NONE,
            quiescent_a: Spec::NONE,
            efficiency_points: &[],
            efficiency_fallback: Spec::NONE,
            r_theta_ja_k_per_w: Spec::NONE,
            t_j_max_c: Spec::NONE,
            t_shutdown_c: Spec::NONE,
            sources: &[],
            caveats: &[],
        }
    }
}

const AMS: &str = "Advanced Monolithic Systems AMS1117: https://files.waveshare.com/upload/b/b8/AMS1117-datasheet.pdf";
const LM: &str = "TI LM2596: https://www.ti.com/lit/ds/symlink/lm2596.pdf";
const MP: &str = "MPS MP1584 Rev. 1.0: https://docs.rs-online.com/b924/A700000013716646.pdf";
const TPS: &str = "TI TPS5430, SLVS632L: https://www.ti.com/lit/ds/symlink/tps5430.pdf";
const TPS60: &str = "TI TPS54360, SLVSBB4G: https://www.ti.com/lit/ds/symlink/tps54360.pdf";

// TI SNVS124G, таблицы 7.5–7.8: типовой КПД при 3 А. 5 и 12 В — фиксированные
// версии, используем их для ориентировочного расчёта регулируемого варианта.
static LM2596_EFF: [(f64, f64); 3] = [(3.0, 0.73), (5.0, 0.80), (12.0, 0.90)];

fn ams1117(
    id: &'static str,
    name: &'static str,
    v_out: f64,
    v_in_min: f64,
    what: &'static str,
) -> Regulator {
    Regulator {
        input_min_v: Spec::ds(
            v_in_min,
            "AMS: нижняя граница таблицы точности выходного напряжения",
        ),
        input_max_v: Spec::ds(12.0, "AMS: характеристики даны при V_вх ≤ 12 В"),
        input_abs_max_v: Spec::ds(15.0, AMS),
        output_min_v: Spec::ds(v_out, AMS),
        output_max_v: Spec::ds(v_out, AMS),
        output_current_a: Spec::ds(0.8, "AMS: 800 мА; продавцы модулей пишут 1 А"),
        current_limit_min_a: Spec::ds(0.9, AMS),
        dropout_v: Spec::ds(1.3, AMS),
        min_load_a: Spec::ds(0.010, AMS),
        quiescent_a: Spec::ds(0.010, AMS),
        r_theta_ja_k_per_w: Spec::ds(
            90.0,
            "AMS: SOT-223 (46–90 °C/Вт в зависимости от меди, берём худшее)",
        ),
        t_j_max_c: Spec::ds(125.0, "AMS: управляющая часть до 125 °C"),
        sources: &[AMS],
        caveats: &[
            "На модулях и в листингах часто «1 А»: паспорт AMS гарантирует 800 мА.",
            "Нужна нагрузка от 10 мА, иначе напряжение на выходе не гарантировано.",
        ],
        ..Regulator::blank(id, name, RegulatorKind::Linear, what)
    }
}

/// Все стабилизаторы каталога.
pub(crate) fn build() -> Vec<Regulator> {
    vec![
        ams1117(
            "ams1117-3.3",
            "AMS1117-3.3",
            3.3,
            4.75, // AMS, диапазон 4.75–12 В для варианта 3.3 В.
            "Линейный стабилизатор на 3.3 В: простой, но всю разницу напряжений превращает в тепло.",
        ),
        ams1117(
            "ams1117-5.0",
            "AMS1117-5.0",
            5.0,
            6.5, // AMS, диапазон 6.5–12 В для варианта 5 В.
            "Линейный стабилизатор на 5 В; от 12 В при заметном токе сильно греется.",
        ),
        Regulator {
            input_min_v: Spec::ds(4.5, LM),
            input_max_v: Spec::ds(40.0, LM),
            input_abs_max_v: Spec::ds(45.0, LM),
            output_min_v: Spec::ds(1.23, LM),
            output_max_v: Spec::ds(37.0, LM),
            output_current_a: Spec::ds(3.0, LM),
            current_limit_min_a: Spec::ds(3.4, "TI: минимум по температуре"),
            dropout_v: Spec::ds(1.5, "TI: насыщение ключа до 1.5 В при 3 А"),
            max_duty: Spec::ds(1.0, LM),
            quiescent_a: Spec::ds(0.010, LM),
            efficiency_points: &LM2596_EFF,
            r_theta_ja_k_per_w: Spec::ds(
                50.0,
                "TI: TO-263 на 0.5 дюйма² меди или TO-220 без радиатора",
            ),
            t_j_max_c: Spec::ds(125.0, "TI: рабочий диапазон до 125 °C"),
            sources: &[LM],
            caveats: &[
                "КПД дан при 3 А и конкретном входе (12 В для 3.3 и 5 В, 25 В для 12 В).",
                "Дешёвые модули часто на копиях микросхемы и держат меньше 3 А.",
            ],
            ..Regulator::blank(
                "lm2596-adj",
                "LM2596 (регулируемый)",
                RegulatorKind::Buck,
                "Понижающий модуль до 3 А с большим входным диапазоном: питание логики и серв от батареи.",
            )
        },
        Regulator {
            input_min_v: Spec::ds(4.5, MP),
            input_max_v: Spec::ds(28.0, MP),
            input_abs_max_v: Spec::ds(30.0, MP),
            output_min_v: Spec::ds(0.8, MP),
            output_max_v: Spec::ds(25.0, MP),
            output_current_a: Spec::ds(3.0, MP),
            quiescent_a: Spec::ds(0.000125, MP),
            efficiency_fallback: Spec::est(0.75, "кривая КПД только картинкой; оценка снизу"),
            r_theta_ja_k_per_w: Spec::ds(50.0, "MPS: SOIC8E, 4-слойная плата JESD51-7"),
            t_j_max_c: Spec::ds(125.0, MP),
            t_shutdown_c: Spec::ds(150.0, MP),
            sources: &[MP],
            caveats: &[
                "MPS: не рекомендован для новых разработок, замена MP2338.",
                "R_θJA дано для 4-слойной платы; крошечные модули греются сильнее.",
                "КПД — оценка: нагрев и ток от батареи можно опровергнуть, но не доказать.",
            ],
            ..Regulator::blank(
                "mp1584",
                "MP1584",
                RegulatorKind::Buck,
                "Крошечный понижающий преобразователь до 28 В: мало места, но слабое охлаждение.",
            )
        },
        Regulator {
            input_min_v: Spec::ds(4.5, TPS60),
            input_max_v: Spec::ds(60.0, TPS60),
            input_abs_max_v: Spec::ds(65.0, TPS60),
            output_min_v: Spec::ds(0.808, "TI: верхняя граница опорного напряжения"),
            output_max_v: Spec::ds(58.8, "TI: страница продукта TPS54360"),
            output_current_a: Spec::ds(3.5, TPS60),
            current_limit_min_a: Spec::ds(4.5, "TI: минимум порога по всему диапазону"),
            max_duty: Spec::ds(0.98, "TI: страница продукта TPS54360"),
            quiescent_a: Spec::ds(0.000175, "TI: наибольший ток без переключения при 25 °C"),
            efficiency_fallback: Spec::est(
                0.75,
                "кривая КПД только картинкой, в расчёт не снята; оценка снизу",
            ),
            r_theta_ja_k_per_w: Spec::ds(42.0, "TI: корпус HSOIC (DDA), стандартная плата"),
            t_j_max_c: Spec::ds(150.0, TPS60),
            t_shutdown_c: Spec::ds(176.0, "TI: типовой порог, гистерезис 12 °C"),
            sources: &[TPS60],
            caveats: &[
                "Нужны внешний диод, дроссель и конденсаторы: схема влияет на КПД.",
                "КПД — оценка: нагрев и ток от батареи можно опровергнуть, но не доказать.",
            ],
            ..Regulator::blank(
                "tps54360",
                "TPS54360",
                RegulatorKind::Buck,
                "Понижающий преобразователь до 60 В: питание логики прямо от батареи 10S.",
            )
        },
        Regulator {
            input_min_v: Spec::ds(5.5, TPS),
            input_max_v: Spec::ds(36.0, TPS),
            input_abs_max_v: Spec::ds(40.0, TPS),
            output_min_v: Spec::ds(1.221, "TI: опорное напряжение 1.221 В"),
            output_current_a: Spec::ds(3.0, TPS),
            current_limit_min_a: Spec::ds(4.0, "TI: минимум порога по верхнему ключу"),
            max_duty: Spec::ds(0.87, "TI: минимум наибольшей скважности"),
            quiescent_a: Spec::ds(0.0044, TPS),
            efficiency_fallback: Spec::est(
                0.75,
                "TI пишет «до 95 %», кривая только картинкой; оценка снизу",
            ),
            r_theta_ja_k_per_w: Spec::ds(45.0, "TI: плата TPS5430EVM"),
            t_j_max_c: Spec::ds(125.0, TPS),
            t_shutdown_c: Spec::ds(135.0, "TI: порог минимум 135 °C"),
            sources: &[TPS],
            caveats: &[
                "Нужен внешний диод, дроссель и конденсаторы: схема влияет на КПД.",
                "КПД — оценка: нагрев и ток от батареи можно опровергнуть, но не доказать.",
            ],
            ..Regulator::blank(
                "tps5430",
                "TPS5430",
                RegulatorKind::Buck,
                "Понижающий преобразователь TI до 36 В и 3 А с хорошо описанными параметрами.",
            )
        },
    ]
}
