//! Питание и общий возврат в установившемся периодическом режиме.
//!
//! Источник постоянного напряжения замыкается через последовательный
//! R–L контур. Нагрузка и ветви развязки подключены между локальными
//! VDD и возвратом. Через общий участок течёт ток контура питания,
//! из которого исключён ток, замыкающийся локально через развязку.
//! Сигнальная опора наблюдается без нагрузки относительно возврата источника.
//! Модель линейна во всей огибающей. Пусковой переход, ограничение регулятора,
//! нелинейная нагрузка и распределённые пути требуют отдельной постановки.

use std::f64::consts::TAU;
use std::num::NonZeroU32;

use nalgebra::Complex;

use super::loop_model::{
    Analysis, LumpedDomain, LumpedPath, Sourced, checked_complex, finite, invalid, nonnegative,
    positive, with_source,
};
use crate::SimError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decoupling {
    pub capacitance_f: f64,
    pub esr_ohm: f64,
    pub esl_h: f64,
}

/// Пиковый фазор, `x(t) = Re(peak · exp(j·2π·order·f₀·t))`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Harmonic {
    pub order: u32,
    pub peak: Complex<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PeriodicLoad<'a> {
    /// Положительный ток направлен от VDD в локальный возврат, А.
    pub dc_a: f64,
    pub fundamental_hz: f64,
    /// Пиковые фазоры тока в амперах, в строго возрастающем порядке.
    pub harmonics: &'a [Harmonic],
    /// Источник нагрузки подтверждает полную периодическую форму.
    /// Усечённый спектр, одиночный фронт и неизвестный хвост оставляют false.
    pub complete_spectrum: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowerInput<'a> {
    /// Полоса обоснованной линейности топологии, источника и ветвей развязки.
    pub domain: Option<Sourced<'a, LumpedDomain>>,
    pub source_v: Option<Sourced<'a, f64>>,
    /// Весь дифференциальный контур от источника, включая общий участок.
    pub feed: Option<LumpedPath>,
    /// Эквивалент физического общего участка. Подтверждённое отсутствие
    /// общего участка задаётся явным путём с R = L = 0.
    pub shared_return: Option<LumpedPath>,
    /// Параллельные ветви с собственными последовательными ESR и ESL.
    /// Пустой список явно задаёт отсутствие локальной развязки.
    pub decoupling: Option<Sourced<'a, &'a [Decoupling]>>,
    pub load: Option<Sourced<'a, PeriodicLoad<'a>>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PowerResponse {
    /// Верхняя частота общей области контура, возврата и развязки, Гц.
    pub max_frequency_hz: f64,
    /// VDD относительно локальной опоры.
    pub rail: VoltageTrace,
    /// Локальная опора относительно возврата источника.
    pub return_shift: VoltageTrace,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoltageTrace {
    dc_v: f64,
    fundamental_hz: f64,
    harmonics: Vec<Harmonic>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Extrema {
    pub min_lower_v: f64,
    pub min_sample_v: f64,
    pub min_phase: f64,
    pub max_sample_v: f64,
    pub max_phase: f64,
    pub max_upper_v: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VoltageLimit {
    Min(f64),
    Max(f64),
}

/// Интервал запаса для расчётной формы, в вольтах. Отрицательный запас
/// означает нарушение. Верхняя граница получена в наблюдаемой фазе.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoltageMargin {
    pub lower_v: f64,
    pub sampled_v: f64,
    pub phase: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VoltageCheck {
    Fail(VoltageMargin),
    NotAnalyzed {
        margin: Option<VoltageMargin>,
        reason: &'static str,
    },
}

impl VoltageTrace {
    pub fn dc_v(&self) -> f64 {
        self.dc_v
    }

    pub fn fundamental_hz(&self) -> f64 {
        self.fundamental_hz
    }

    /// Напряжение в вольтах, фазоры привязаны к фазе нагрузки.
    pub fn harmonics(&self) -> &[Harmonic] {
        &self.harmonics
    }

    /// Напряжение в заданной фазе в долях периода, фаза может быть знаковой.
    pub fn at_phase(&self, phase: f64) -> Result<f64, SimError> {
        let phase = finite(phase)?.fract();
        let mut ac = 0.0;
        for h in &self.harmonics {
            let (sin, cos) = (TAU * (phase * f64::from(h.order)).fract()).sin_cos();
            ac += h.peak.re.mul_add(cos, -h.peak.im * sin);
        }
        finite(self.dc_v + ac)
    }

    /// Внутренние свидетели и внешние границы минимума/максимума за период.
    /// Границы учитывают непопадание между отсчётами для аналитического ряда.
    /// Погрешности параметров и арифметики с плавающей точкой сюда не входят.
    pub fn extrema(&self, samples: NonZeroU32) -> Result<Extrema, SimError> {
        let count = f64::from(samples.get());
        let initial = self.at_phase(0.0)?;
        let (mut min_sample_v, mut max_sample_v) = (initial, initial);
        let (mut min_phase, mut max_phase) = (0.0, 0.0);
        for i in 1..samples.get() {
            let phase = f64::from(i) / count;
            let value = self.at_phase(phase)?;
            if value < min_sample_v {
                (min_sample_v, min_phase) = (value, phase);
            }
            if value > max_sample_v {
                (max_sample_v, max_phase) = (value, phase);
            }
        }
        // В экстремуме производная нулевая. Теорема Тейлора даёт ошибку
        // ≤ max|V''|·(1/(2N))²/2. 2·|Vn| также ограничивает разброс гармоники.
        let mut padding = 0.0;
        for h in &self.harmonics {
            let curvature_factor = (TAU * f64::from(h.order) / count).powi(2) / 8.0;
            padding += h.peak.norm() * curvature_factor.min(2.0);
        }
        Ok(Extrema {
            min_lower_v: finite(min_sample_v - padding)?,
            min_sample_v,
            min_phase,
            max_sample_v,
            max_phase,
            max_upper_v: finite(max_sample_v + padding)?,
        })
    }

    /// Сопоставить модель с применимым пределом стенда. `Fail` содержит
    /// фазу превышения. Для остальных исходов требуется измерение на порту.
    pub fn check(
        &self,
        limit: Option<Sourced<'_, VoltageLimit>>,
        samples: NonZeroU32,
    ) -> Result<VoltageCheck, SimError> {
        if let Some(limit) = limit {
            let (VoltageLimit::Min(v) | VoltageLimit::Max(v)) = limit.value;
            finite(v)?;
        }
        let Some(limit) = with_source(limit) else {
            return Ok(VoltageCheck::NotAnalyzed {
                margin: None,
                reason: "нет предела с источником. Нужен применимый предел для измерительного порта",
            });
        };
        let range = self.extrema(samples)?;
        let (lower_v, sampled_v, phase) = match limit {
            VoltageLimit::Min(v) => (
                range.min_lower_v - v,
                range.min_sample_v - v,
                range.min_phase,
            ),
            VoltageLimit::Max(v) => (
                v - range.max_upper_v,
                v - range.max_sample_v,
                range.max_phase,
            ),
        };
        let margin = VoltageMargin {
            lower_v: finite(lower_v)?,
            sampled_v: finite(sampled_v)?,
            phase,
        };
        if margin.sampled_v < 0.0 {
            return Ok(VoltageCheck::Fail(margin));
        }
        let reason = if margin.lower_v < 0.0 {
            "огибающая пересекает предел. Нужен стендовый замер экстремума с разрешением фронта"
        } else {
            "нужен стендовый замер напряжения на заданном порту во всей полосе и огибающей нагрузки"
        };
        Ok(VoltageCheck::NotAnalyzed {
            margin: Some(margin),
            reason,
        })
    }
}

/// Рассчитать оба порта для каждой гармоники полного спектра нагрузки.
pub fn analyze_power(input: PowerInput<'_>) -> Result<Analysis<PowerResponse>, SimError> {
    let domain_limit = input
        .domain
        .map(|d| d.value.frequency_limit())
        .transpose()?;
    if let Some(v) = input.source_v {
        finite(v.value)?;
    }
    if let Some(caps) = input.decoupling {
        for cap in caps.value {
            if !positive(cap.capacitance_f) || !nonnegative(cap.esr_ohm) || !nonnegative(cap.esl_h)
            {
                return Err(invalid(
                    "C должна быть конечной и положительной, ESR и ESL — конечными и неотрицательными",
                ));
            }
        }
    }
    if let Some(load) = input.load {
        validate_load(load.value)?;
    }
    if let (Some(feed), Some(shared)) = (input.feed, input.shared_return) {
        let (feed, shared) = (feed.series_rl(), shared.series_rl());
        if shared.resistance_ohm > feed.resistance_ohm || shared.inductance_h > feed.inductance_h {
            return Err(invalid(
                "R и L общего последовательного участка не могут превышать R и L всего контура",
            ));
        }
    }
    let (
        Some(_),
        Some(domain_limit),
        Some(source_v),
        Some(feed),
        Some(shared),
        Some(caps),
        Some(load),
    ) = (
        with_source(input.domain),
        domain_limit,
        with_source(input.source_v),
        input.feed,
        input.shared_return,
        with_source(input.decoupling),
        with_source(input.load),
    )
    else {
        return Ok(Analysis::NotAnalyzed(
            "не заданы питание, общий возврат, развязка, область или нагрузка с источником. Нужны схема и стендовые замеры",
        ));
    };
    let Some(last) = load.harmonics.iter().rfind(|h| h.peak != Complex::ZERO) else {
        return Ok(Analysis::NotAnalyzed(
            "нет переменной составляющей нагрузки. Нужен измеренный ряд тока с фронтами",
        ));
    };
    let max_frequency = domain_limit
        .min(feed.max_frequency_hz())
        .min(shared.max_frequency_hz());
    if !load.complete_spectrum || load.fundamental_hz * f64::from(last.order) > max_frequency {
        return Ok(Analysis::NotAnalyzed(
            "не покрыт спектр нагрузки. Нужны полоса импеданса и стендовый ряд тока на тех же портах",
        ));
    }
    let (feed, shared) = (feed.series_rl(), shared.series_rl());
    if caps.iter().any(|cap| cap.esl_h > 0.0 && cap.esr_ohm == 0.0)
        || (feed.inductance_h > 0.0
            && feed.resistance_ohm == 0.0
            && !caps.is_empty()
            && caps.iter().all(|cap| cap.esr_ohm == 0.0))
    {
        return Ok(Analysis::NotAnalyzed(
            "не установлено затухание резонанса. Нужен замер потерь и импеданса контура",
        ));
    }

    let mut rail = VoltageTrace {
        dc_v: finite(source_v - feed.resistance_ohm * load.dc_a)?,
        fundamental_hz: load.fundamental_hz,
        harmonics: Vec::with_capacity(load.harmonics.len()),
    };
    let mut return_shift = VoltageTrace {
        dc_v: finite(shared.resistance_ohm * load.dc_a)?,
        fundamental_hz: load.fundamental_hz,
        harmonics: Vec::with_capacity(load.harmonics.len()),
    };
    for h in load.harmonics.iter().filter(|h| h.peak != Complex::ZERO) {
        let omega = finite(TAU * (load.fundamental_hz * f64::from(h.order)))?;
        let z_feed = feed.at_omega(omega)?;
        let mut y_cap = Complex::ZERO;
        for cap in caps {
            let capacitive = (1.0 / omega) / cap.capacitance_f;
            let z_cap = checked_complex(Complex::new(cap.esr_ohm, omega * cap.esl_h - capacitive))?;
            y_cap = checked_complex(y_cap + complex_div(Complex::ONE, z_cap)?)?;
        }
        // I_feed = I_load + Y_cap Vrail, Vrail = −Z_feed I_feed (AC).
        let denominator = checked_complex(Complex::<f64>::ONE + z_feed * y_cap)?;
        let i_feed = complex_div(h.peak, denominator)?;
        let v_rail = checked_complex(-z_feed * i_feed)?;
        let v_return = checked_complex(shared.at_omega(omega)? * i_feed)?;
        rail.harmonics.push(Harmonic {
            order: h.order,
            peak: v_rail,
        });
        return_shift.harmonics.push(Harmonic {
            order: h.order,
            peak: v_return,
        });
    }
    Ok(Analysis::Calculated(PowerResponse {
        max_frequency_hz: max_frequency,
        rail,
        return_shift,
    }))
}

fn validate_load(load: PeriodicLoad<'_>) -> Result<(), SimError> {
    finite(load.dc_a)?;
    if !positive(load.fundamental_hz) {
        return Err(invalid(
            "основная частота нагрузки должна быть конечной и положительной",
        ));
    }
    let mut previous = 0;
    for h in load.harmonics {
        checked_complex(h.peak)?;
        finite(load.fundamental_hz * f64::from(h.order))?;
        if h.order <= previous {
            return Err(invalid(
                "номера гармоник должны быть положительными и строго возрастающими",
            ));
        }
        previous = h.order;
    }
    Ok(())
}

fn complex_div(a: Complex<f64>, b: Complex<f64>) -> Result<Complex<f64>, SimError> {
    // Нормировка избавляет от переполнения при вычислении квадрата |b|.
    let scale = b.re.abs().max(b.im.abs());
    if scale == 0.0 {
        return Err(invalid(
            "вырожденный импеданс, конечный периодический отклик не определён",
        ));
    }
    let b = Complex::new(b.re / scale, b.im / scale);
    let a = Complex::new(a.re / scale, a.im / scale);
    checked_complex((a * b.conj()) / b.norm_sqr())
}
