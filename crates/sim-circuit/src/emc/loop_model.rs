//! Сосредоточенные оценки петли тока и импеданса физического возврата.

use std::f64::consts::{PI, TAU};

use crate::SimError;
use nalgebra::Complex;

// CODATA 2022, табл. XXXIII. Скорость света зафиксирована определением СИ.
// https://physics.nist.gov/cuu/pdf/RevModPhys.97.025002.pdf
const MU_0: f64 = 1.256_637_061_27e-6;
const C_0: f64 = 299_792_458.0;

/// Исходное значение и описание его происхождения и условий применения.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sourced<'a, T> {
    pub value: T,
    pub source: &'a str,
}

/// `Calculated` содержит расчётную величину в заявленной области модели.
#[derive(Clone, Debug, PartialEq)]
pub enum Analysis<T> {
    Calculated(T),
    NotAnalyzed(&'static str),
}

/// Условия применимости одной конкретной сосредоточенной топологии, от DC.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LumpedDomain {
    /// Верхняя частота, до которой обоснованы параметры и топология, Гц.
    pub max_frequency_hz: f64,
    /// Верхняя граница длины электрического пути, м. Для петли — периметр.
    pub longest_path_m: f64,
    /// Нижняя оценка фазовой скорости во всей полосе, м/с.
    pub min_wave_speed_m_s: f64,
    /// Допустимая доля длины волны из источника области.
    pub max_wavelength_fraction: f64,
}

impl LumpedDomain {
    pub(super) fn frequency_limit(self) -> Result<f64, SimError> {
        // Сосредоточенная модель требует малого фазового набега (MIT 6.013, §14.4).
        // λ/10 отсекает пути с набегом > 36°. Обоснованная рабочая граница
        // может быть существенно ниже и задаётся для конкретной топологии.
        // https://web.mit.edu/6.013_book/www/chapter14/14.html
        const MAX_WAVELENGTH_FRACTION: f64 = 0.1;
        if !positive(self.max_frequency_hz)
            || !positive(self.longest_path_m)
            || !positive(self.min_wave_speed_m_s)
            || self.min_wave_speed_m_s > C_0
            || !positive(self.max_wavelength_fraction)
            || self.max_wavelength_fraction > MAX_WAVELENGTH_FRACTION
        {
            return Err(invalid("некорректная область сосредоточенной модели"));
        }
        let electrical_limit =
            (self.max_wavelength_fraction * self.min_wave_speed_m_s) / self.longest_path_m;
        Ok(self.max_frequency_hz.min(electrical_limit))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeriesRl {
    pub resistance_ohm: f64,
    pub inductance_h: f64,
}

impl SeriesRl {
    pub(super) fn validate(self) -> Result<(), SimError> {
        if !nonnegative(self.resistance_ohm) || !nonnegative(self.inductance_h) {
            return Err(invalid("R и L должны быть конечными и неотрицательными"));
        }
        Ok(())
    }

    pub(super) fn at_omega(self, omega: f64) -> Result<Complex<f64>, SimError> {
        checked_complex(Complex::new(self.resistance_ohm, omega * self.inductance_h))
    }
}

/// Пассивный последовательный R–L путь с проверенной областью модели.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LumpedPath {
    rl: SeriesRl,
    max_frequency_hz: f64,
}

impl LumpedPath {
    pub fn series_rl(self) -> SeriesRl {
        self.rl
    }

    pub fn max_frequency_hz(self) -> f64 {
        self.max_frequency_hz
    }

    /// Импеданс в Омах, соглашение `exp(jωt)`.
    pub fn impedance(self, frequency_hz: f64) -> Result<Analysis<Complex<f64>>, SimError> {
        if !nonnegative(frequency_hz) {
            return Err(invalid("частота должна быть конечной и неотрицательной"));
        }
        if frequency_hz > self.max_frequency_hz {
            return Ok(Analysis::NotAnalyzed(
                "частота вне полосы модели пути. Нужен замер импеданса на тех же портах",
            ));
        }
        Ok(Analysis::Calculated(self.rl.at_omega(TAU * frequency_hz)?))
    }
}

/// Задать эквивалент по измерению или явной модели. Для общего возврата
/// источник должен привязать R и L к общей ветви, портам и пути тока.
pub fn fitted_path(
    rl: Option<Sourced<'_, SeriesRl>>,
    domain: Option<Sourced<'_, LumpedDomain>>,
) -> Result<Analysis<LumpedPath>, SimError> {
    let limit = domain.map(|d| d.value.frequency_limit()).transpose()?;
    if let Some(rl) = rl {
        rl.value.validate()?;
    }
    match (with_source(rl), with_source(domain), limit) {
        (Some(rl), Some(_), Some(max_frequency_hz)) if max_frequency_hz > 0.0 => {
            Ok(Analysis::Calculated(LumpedPath {
                rl,
                max_frequency_hz,
            }))
        }
        _ => Ok(Analysis::NotAnalyzed(
            "нет R–L пути с обоснованной областью. Нужен замер импеданса между заданными портами",
        )),
    }
}

/// Изолированный плоский прямоугольник из однородного круглого провода.
/// Стороны заданы по оси провода. Провод и однородная среда немагнитны.
/// Область включает одиночную петлю вдали от других проводников, плоскостей
/// и магнитных материалов. Потери и геометрия контактов задаются отдельно.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RectangularLoop {
    pub side_a_m: f64,
    pub side_b_m: f64,
    pub wire_radius_m: f64,
    /// Удельное сопротивление при температуре опыта, Ом·м.
    pub resistivity_ohm_m: f64,
    /// Верхняя граница r/min(a,b), заданная источником геометрии.
    pub max_radius_to_side: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoopEstimate {
    pub area_m2: f64,
    /// Весь дифференциальный контур. Импеданс общего участка задаётся отдельно.
    pub path: LumpedPath,
}

/// Оценить площадь, R постоянного тока и полную низкочастотную L петли.
pub fn rectangular_loop(
    geometry: Option<Sourced<'_, RectangularLoop>>,
    domain: Option<Sourced<'_, LumpedDomain>>,
) -> Result<Analysis<LoopEstimate>, SimError> {
    let limit = domain.map(|d| d.value.frequency_limit()).transpose()?;
    if let Some(g) = geometry {
        let g = g.value;
        if ![
            g.side_a_m,
            g.side_b_m,
            g.wire_radius_m,
            g.resistivity_ohm_m,
            g.max_radius_to_side,
        ]
        .into_iter()
        .all(positive)
        {
            return Err(invalid(
                "размеры петли, удельное сопротивление и геометрическая граница должны быть конечными и положительными",
            ));
        }
    }
    let (Some(g), Some(domain), Some(limit)) = (with_source(geometry), with_source(domain), limit)
    else {
        return Ok(Analysis::NotAnalyzed(
            "нет геометрии, материала или области петли с источником. Нужны обмер и замер импеданса",
        ));
    };
    let (a, b, r) = (g.side_a_m, g.side_b_m, g.wire_radius_m);
    let perimeter = 2.0 * (a + b);
    // В (86) отброшен порядок (r/side)². Ограничиваем этот параметр 0,01,
    // сохраняя меньшую геометрическую границу из источника конкретного опыта.
    if r / a.min(b) > g.max_radius_to_side.min(0.1) || perimeter > domain.longest_path_m {
        return Ok(Analysis::NotAnalyzed(
            "геометрия вне области тонкой прямоугольной петли. Нужен замер дифференциального импеданса",
        ));
    }

    // Rosa и Cohen, NBS Bulletin 5(1), (73), (77), (86). Складываем
    // собственные L четырёх сторон и вычитаем удвоенные взаимные L двух
    // противоположных пар. У соседних осей скалярное произведение равно нулю.
    // https://nvlpubs.nist.gov/nistpubs/bulletin/05/nbsbulletinv5n1p1_A2b.pdf
    let self_side = |len: f64| parallel_kernel(len, r) + len / 4.0;
    let inductance_h =
        (MU_0 / PI) * (self_side(a) - parallel_kernel(a, b) + self_side(b) - parallel_kernel(b, a));
    let resistance_ohm = g.resistivity_ohm_m * perimeter / (PI * r * r);
    let area_m2 = a * b;

    // δ = √(ρ / (π f μ₀)). При δ < r приближение равномерного тока
    // не используем. Это отсечение, оценка ошибки внутри полосы нужна отдельно.
    let skin_limit = g.resistivity_ohm_m / (PI * MU_0 * r * r);
    let max_frequency_hz = limit.min(skin_limit);
    if ![area_m2, inductance_h, resistance_ohm, skin_limit]
        .into_iter()
        .all(positive)
    {
        return Err(invalid(
            "параметры петли выходят за численный диапазон модели",
        ));
    }
    if max_frequency_hz <= 0.0 {
        return Ok(Analysis::NotAnalyzed(
            "нет ненулевой полосы сосредоточенной петли. Нужен замер импеданса",
        ));
    }
    Ok(Analysis::Calculated(LoopEstimate {
        area_m2,
        path: LumpedPath {
            rl: SeriesRl {
                resistance_ohm,
                inductance_h,
            },
            max_frequency_hz,
        },
    }))
}

// Половина двойного интеграла Неймана для двух параллельных одинаковых
// отрезков. Разность hypot(len, gap) − gap вычисляем без вычитания близких чисел.
fn parallel_kernel(len: f64, gap: f64) -> f64 {
    let diagonal = len.hypot(gap);
    len * ((len / gap).asinh() - (len / diagonal) / (1.0 + gap / diagonal))
}

pub(super) fn with_source<T>(item: Option<Sourced<'_, T>>) -> Option<T> {
    item.filter(|item| !item.source.trim().is_empty())
        .map(|item| item.value)
}

pub(super) fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

pub(super) fn nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

pub(super) fn finite(value: f64) -> Result<f64, SimError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(invalid("неконечное значение в расчёте ЭМС"))
    }
}

pub(super) fn checked_complex(value: Complex<f64>) -> Result<Complex<f64>, SimError> {
    finite(value.re)?;
    finite(value.im)?;
    Ok(value)
}

pub(super) fn invalid(message: &str) -> SimError {
    SimError::Invalid(message.into())
}
