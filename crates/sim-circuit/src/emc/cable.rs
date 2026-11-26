//! Собственный синфазный ток кабеля в одномодовом эквиваленте.
//!
//! Отрезок — прямой круглый эквипотенциальный проводник над сплошной плоской
//! опорой в однородном немагнитном диэлектрике. Поперечное сечение постоянно.
//! Опора одна для обоих портов и несёт обратный ток. Для пучка эквивалентный
//! радиус допустим при обосновании, что провода представляют одну синфазную
//! моду. Учитываются продольные отражения и заданные распределённые потери.
//! Края и разрывы опоры, переходы между модами, неоднородности, излучение,
//! внешнее возбуждение вдоль кабеля и дисперсия параметров вне этой модели.
//!
//! Расчёт относится к одной частоте. Огибающие произвольного сигнала и ток
//! между портами требуют отдельной постановки.

use std::f64::consts::TAU;

use nalgebra::Complex;

use crate::SimError;

use super::loop_model::{
    Analysis, Sourced, checked_complex, finite, invalid, nonnegative, positive, with_source,
};

// CODATA 2022, таблица XXXIII. Скорость света зафиксирована определением СИ.
// https://physics.nist.gov/cuu/pdf/RevModPhys.97.025002.pdf
const LIGHT_SPEED_M_S: f64 = 299_792_458.0;
const VACUUM_PERMEABILITY_H_M: f64 = 1.256_637_061_27e-6;

// Консервативное отсечение модели установки с идеализированными портами.
// λ/10 соответствует 36° фазового набега по максимальному размеру.
// MIT 6.013, §§14.4–14.5, связь фазового набега с временем пролёта.
// https://web.mit.edu/6.013_book/www/chapter14/14.4.html
const MAX_WAVELENGTH_FRACTION: f64 = 0.1;

/// Геометрия между плоскостями портов. Все размеры в метрах.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableGeometry<'a> {
    pub length_m: f64,
    pub radius_m: f64,
    pub axis_height_m: f64,
    pub relative_permittivity: f64,
    /// Идентификатор физической плоскости возврата из данных установки.
    pub reference: &'a str,
    /// Источник подтверждает однородную геометрию и применимость
    /// приближения плоской опоры и одной синфазной моды.
    pub uniform_over_plane: bool,
}

/// Потери синфазного тракта в заявленной полосе. Нулевые потери задаются явно.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableLosses {
    /// Последовательное сопротивление моды с учётом возврата.
    pub series_resistance_ohm_per_m: f64,
    /// Поперечная проводимость от кабеля к опоре.
    pub shunt_conductance_s_per_m: f64,
}

/// Обоснованная полоса, в которой применима геометрия и приняты параметры.
/// Обе границы включительны. Электрический размер может сузить полосу.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableBand {
    pub min_frequency_hz: f64,
    pub max_frequency_hz: f64,
    /// Допустимая доля λ для длины и поперечного размера. Интервал (0; 0,1].
    pub max_wavelength_fraction: f64,
}

/// Эквивалент Тевенина на ближнем порту при частоте расчёта.
/// Фазоры амплитудные, с соглашением exp(jωt). При DC задаются вещественными.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableSource<'a> {
    pub reference: &'a str,
    pub peak_v: Complex<f64>,
    /// Пассивный выходной импеданс. Идеальный источник задаётся нулём явно.
    pub impedance_ohm: Complex<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CableLoad {
    /// Явное размыкание дальнего порта.
    Open,
    /// Пассивный импеданс на частоте расчёта. Ноль означает закоротку.
    ImpedanceOhm(Complex<f64>),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableTermination<'a> {
    pub reference: &'a str,
    pub load: CableLoad,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CableEnd {
    Near,
    Far,
}

/// Верхний предел амплитуды тока на одном порту стенда.
/// Источник должен обосновать его применимость к постановке и частоте.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableLimit<'a> {
    pub reference: &'a str,
    pub end: CableEnd,
    pub frequency_hz: f64,
    pub max_peak_a: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CableInput<'a> {
    pub geometry: Option<Sourced<'a, CableGeometry<'a>>>,
    pub losses: Option<Sourced<'a, CableLosses>>,
    pub band: Option<Sourced<'a, CableBand>>,
    pub source: Option<Sourced<'a, CableSource<'a>>>,
    pub termination: Option<Sourced<'a, CableTermination<'a>>>,
    pub limit: Option<Sourced<'a, CableLimit<'a>>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CablePortPhasors {
    /// Напряжение проводника относительно заявленной опоры.
    pub voltage_peak_v: Complex<f64>,
    /// Суммарный ток моды. Оба тока ориентированы от ближнего конца к дальнему.
    pub current_peak_a: Complex<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableMargin {
    pub end: CableEnd,
    pub peak_a: f64,
    /// Предел минус модуль фазора тока, в амперах.
    pub margin_a: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CableCheck {
    Fail(CableMargin),
    NotAnalyzed {
        margin: Option<CableMargin>,
        reason: &'static str,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableResponse<'a> {
    pub reference: &'a str,
    pub frequency_hz: f64,
    pub near: CablePortPhasors,
    pub far: CablePortPhasors,
    pub check: CableCheck,
}

/// Амплитуды и фазы на двух портах на одной частоте. Портовый запас
/// относится к этой точке и не ограничивает огибающую по частоте.
pub fn common_mode_current<'a>(
    input: &CableInput<'a>,
    frequency_hz: f64,
) -> Result<Analysis<CableResponse<'a>>, SimError> {
    validate(input, frequency_hz)?;

    let (Some(g), Some(losses), Some(band), Some(source), Some(termination)) = (
        with_source(input.geometry),
        with_source(input.losses),
        with_source(input.band),
        with_source(input.source),
        with_source(input.termination),
    ) else {
        return Ok(Analysis::NotAnalyzed(
            "нужны геометрия, потери, полоса, возбуждение и оконечная нагрузка с источниками.              Нужен стендовый замер тока на заданном кабеле",
        ));
    };
    if g.reference.trim().is_empty()
        || g.reference != source.reference
        || g.reference != termination.reference
    {
        return Ok(Analysis::NotAnalyzed(
            "у кабеля и обоих портов должна быть общая физическая опора.              Нужен стендовый замер возврата и тока на тех же портах",
        ));
    }
    if !g.uniform_over_plane || g.relative_permittivity < 1.0 {
        return Ok(Analysis::NotAnalyzed(
            "геометрия или среда вне одномодового приближения.              Нужен стендовый замер тока с фактической опорой",
        ));
    }

    let wave_speed = LIGHT_SPEED_M_S / g.relative_permittivity.sqrt();
    let extent = g.length_m.max(finite(g.axis_height_m + g.radius_m)?);
    let max_frequency = band
        .max_frequency_hz
        .min(band.max_wavelength_fraction * wave_speed / extent);
    if !(band.min_frequency_hz..=max_frequency).contains(&frequency_hz) {
        return Ok(Analysis::NotAnalyzed(
            "частота вне заявленной полосы или электрического размера кабеля.              Нужен замер тока в заданной полосе на обоих портах",
        ));
    }

    // MIT 6.013, §§4.6–4.7, 8.6 и 14.2. Метод изображений даёт
    // k = acosh(h/r), L' = μ₀ k/(2π), C' = 1/(v² L').
    // https://web.mit.edu/6.013_book/www/chapter4/4.7.html
    // https://web.mit.edu/6.013_book/www/chapter14/14.2.html
    let geometry_factor = finite((g.axis_height_m / g.radius_m).acosh())?;
    let l_per_m = VACUUM_PERMEABILITY_H_M * geometry_factor / TAU;
    let c_per_m = finite(1.0 / (wave_speed.powi(2) * l_per_m))?;
    let omega = finite(TAU * frequency_hz)?;
    let z = checked_complex(Complex::new(
        losses.series_resistance_ohm_per_m * g.length_m,
        omega * l_per_m * g.length_m,
    ))?;
    let y = checked_complex(Complex::new(
        losses.shunt_conductance_s_per_m * g.length_m,
        omega * c_per_m * g.length_m,
    ))?;

    // Решение телеграфных уравнений для однородного отрезка. При одном
    // направлении токов Vб = A Vд + B Iд, Iб = C Vд + A Iд. Участок
    // описывают θ=√(ZY), A=cosh θ, B=Z sinh(θ)/θ, C=Y sinh(θ)/θ.
    let theta_sq = checked_complex(z * y)?;
    let theta = checked_complex(theta_sq.sqrt())?;
    if theta.im.abs() > (TAU * band.max_wavelength_fraction).next_up() {
        return Ok(Analysis::NotAnalyzed(
            "фазовый набег с учётом потерь вне области модели.              Нужен замер тока и фазового набега на кабеле",
        ));
    }
    let (a, sinhc) = if theta_sq.norm() <= f64::EPSILON {
        // Ряды cosh и sinh(θ)/θ в нуле. При |θ²| ≤ ε остаток порядка ε²,
        // так что сохраняется DC-предел и не возникает деления 0/0.
        let one = Complex::new(1.0, 0.0);
        (one + theta_sq / 2.0, one + theta_sq / 6.0)
    } else {
        (theta.cosh(), theta.sinh() / theta)
    };
    let b = checked_complex(z * sinhc)?;
    let c = checked_complex(y * sinhc)?;

    // Нормируем условие на дальнем порту по Vд при обрыве и по Iд
    // при конечном импедансе. Это включает идеальную закоротку без 1/Zн.
    let (far_v, far_i) = match termination.load {
        CableLoad::Open => (Complex::new(1.0, 0.0), 0.0),
        CableLoad::ImpedanceOhm(z_load) => (z_load, 1.0),
    };
    let near_v = checked_complex(a * far_v + b * far_i)?;
    let near_i = checked_complex(c * far_v + a * far_i)?;
    let denominator = checked_complex(near_v + source.impedance_ohm * near_i)?;
    let scale = finite(denominator.norm())?;
    let term_bound = finite(
        (a * far_v).norm()
            + (b * far_i).norm()
            + source.impedance_ohm.norm() * ((c * far_v).norm() + (a * far_i).norm()),
    )?;
    // √ε ограничивает потерю примерно половины значащих разрядов f64
    // при компенсации слагаемых. Включает несовместимые идеальные порты.
    if scale <= f64::EPSILON.sqrt() * term_bound {
        return Ok(Analysis::NotAnalyzed(
            "ток не определён устойчиво при заданных импедансах.              Нужен замер импедансов и тока на портах",
        ));
    }

    // Нормируем числитель и знаменатель до комплексного деления.
    // ЭДС умножаем на коэффициент передачи после деления. Так малое
    // напряжение на большой нагрузке сохраняется при округлении.
    let normalized = denominator / scale;
    let at_source =
        |basis: Complex<f64>| checked_complex((basis / scale) / normalized * source.peak_v);
    let near = CablePortPhasors {
        voltage_peak_v: at_source(near_v)?,
        current_peak_a: at_source(near_i)?,
    };
    let far = CablePortPhasors {
        voltage_peak_v: at_source(far_v)?,
        current_peak_a: at_source(far_i.into())?,
    };
    let near_peak = finite(near.current_peak_a.norm())?;
    let far_peak = finite(far.current_peak_a.norm())?;
    let check = match with_source(input.limit) {
        Some(limit) if limit.reference == g.reference && limit.frequency_hz == frequency_hz => {
            let peak_a = match limit.end {
                CableEnd::Near => near_peak,
                CableEnd::Far => far_peak,
            };
            let margin = CableMargin {
                end: limit.end,
                peak_a,
                margin_a: limit.max_peak_a - peak_a,
            };
            if margin.margin_a < 0.0 {
                CableCheck::Fail(margin)
            } else {
                CableCheck::NotAnalyzed {
                    margin: Some(margin),
                    reason: "нужен стендовый замер тока на указанном порту в рабочей огибающей",
                }
            }
        }
        _ => CableCheck::NotAnalyzed {
            margin: None,
            reason: "нужен предел с источником для этой опоры и частоты, а также стендовый замер",
        },
    };
    Ok(Analysis::Calculated(CableResponse {
        reference: g.reference,
        frequency_hz,
        near,
        far,
        check,
    }))
}

fn validate(input: &CableInput<'_>, frequency_hz: f64) -> Result<(), SimError> {
    if !nonnegative(frequency_hz) {
        return Err(invalid("частота должна быть конечной и неотрицательной"));
    }
    if let Some(g) = input.geometry {
        let g = g.value;
        if !positive(g.length_m)
            || !positive(g.radius_m)
            || !positive(g.axis_height_m)
            || g.axis_height_m <= g.radius_m
            || !positive(g.relative_permittivity)
        {
            return Err(invalid(
                "нужны конечные положительные размеры и проницаемость, ось должна быть выше радиуса",
            ));
        }
    }
    if let Some(losses) = input.losses {
        let losses = losses.value;
        if !nonnegative(losses.series_resistance_ohm_per_m)
            || !nonnegative(losses.shunt_conductance_s_per_m)
        {
            return Err(invalid(
                "погонные сопротивление и проводимость должны быть конечными и неотрицательными",
            ));
        }
    }
    if let Some(band) = input.band {
        let band = band.value;
        if !nonnegative(band.min_frequency_hz)
            || !positive(band.max_frequency_hz)
            || band.min_frequency_hz > band.max_frequency_hz
            || !positive(band.max_wavelength_fraction)
            || band.max_wavelength_fraction > MAX_WAVELENGTH_FRACTION
        {
            return Err(invalid("некорректная полоса или доля длины волны кабеля"));
        }
    }
    if let Some(source) = input.source {
        let source = source.value;
        checked_complex(source.peak_v)?;
        if frequency_hz == 0.0 && source.peak_v.im != 0.0 {
            return Err(invalid("ЭДС при DC должна быть вещественной"));
        }
        validate_impedance(source.impedance_ohm, frequency_hz)?;
    }
    if let Some(termination) = input.termination
        && let CableLoad::ImpedanceOhm(z) = termination.value.load
    {
        validate_impedance(z, frequency_hz)?;
    }
    if let Some(limit) = input.limit
        && (!nonnegative(limit.value.frequency_hz) || !nonnegative(limit.value.max_peak_a))
    {
        return Err(invalid(
            "частота и предел тока должны быть конечными и неотрицательными",
        ));
    }
    Ok(())
}

fn validate_impedance(z: Complex<f64>, frequency_hz: f64) -> Result<(), SimError> {
    checked_complex(z)?;
    if z.re < 0.0 || (frequency_hz == 0.0 && z.im != 0.0) {
        return Err(invalid(
            "импеданс порта должен быть пассивным, при DC — вещественным",
        ));
    }
    Ok(())
}
