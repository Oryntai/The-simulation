//! Квазистатические параметры и передача на однородном участке двух микрополосковых линий.
//!
//! Все четыре порта отнесены к одной непрерывной плоскости возврата. Модель описывает
//! одинаковые прямые, параллельные и полностью совмещённые по длине полоски нулевой
//! толщины на изотропной немагнитной подложке, с воздухом над ними. Потери, дисперсия,
//! краевые поля концов, неоднородности и другие проводники за плоскостями портов
//! требуют отдельного обоснования области, замера или внешнего расчёта.
//! Численный результат предназначен для оценки наведённого напряжения.
//! При компенсации ёмкостного и индуктивного вкладов относительная погрешность
//! наведённого напряжения может сильно возрасти.

use std::f64::consts::TAU;

use nalgebra::{Complex, Matrix2, Vector2};

use super::loop_model::{
    Analysis, Sourced, checked_complex, finite, invalid, nonnegative, positive, with_source,
};
use crate::{SimError, pcb::microstrip_hammerstad_jensen};

// Точное значение скорости света в СИ.
const LIGHT_SPEED_M_S: f64 = 299_792_458.0;
// Отсечение электрически длинных участков: λ/10 даёт фазовый набег 36°.
// MIT 6.013, §14.4, ограничение сосредоточенного приближения по времени полёта.
// https://web.mit.edu/6.013_book/www/chapter14/14.html
const MAX_WAVELENGTH_FRACTION: f64 = 0.1;

/// Геометрия равных полосок, совмещённых по длине. Размеры в метрах.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MicrostripPair {
    /// Длина равномерно связанного участка между плоскостями портов.
    pub length_m: f64,
    /// Ширина каждой полоски.
    pub width_m: f64,
    /// Зазор между ближайшими краями полосок.
    pub gap_m: f64,
    /// Высота над общей опорой и одновременно толщина подложки.
    pub height_m: f64,
    /// Толщина металла. Для идеальных полосок задаётся явно равной нулю.
    pub thickness_m: f64,
    pub relative_permittivity: f64,
    /// Источник геометрии подтверждает указанную топологию и отсутствие
    /// значимых разрывов опоры, близких экранов, переходов и неоднородностей.
    pub uniform_over_plane: bool,
}

/// Полоса с обоснованием применимости статических C/L и пренебрежения потерями.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CouplingBand {
    pub min_frequency_hz: f64,
    pub max_frequency_hz: f64,
    /// Верхняя доля λ для продольного и поперечного размеров, в интервале (0; 0,1].
    pub max_wavelength_fraction: f64,
}

/// Интегральные параметры участка. Диагонали матриц C и L одинаковы на обеих трассах.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledLc {
    /// Диагональ матрицы Максвелла, включая вклад связи с соседней трассой.
    pub self_c_f: f64,
    /// Положительная межтрассовая ёмкость. Внедиагональный элемент матрицы равен −Cm.
    /// Ёмкость каждой трассы непосредственно на плоскость равна self_c_f − mutual_c_f.
    pub mutual_c_f: f64,
    pub self_l_h: f64,
    /// Положительный внедиагональный элемент L при одинаковом направлении токов.
    pub mutual_l_h: f64,
}

/// Явные оконечные параметры. Ёмкость между портом и плоскостью может быть нулевой.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParallelRc {
    /// Конечное положительное сопротивление, Ом.
    pub resistance_ohm: f64,
    pub capacitance_f: f64,
}

/// Все нагрузки относительно общей опоры. Ближний порт источника подключён к ЭДС
/// через свой резистор, его шунтирующий конденсатор остаётся на стороне трассы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CouplingPorts {
    pub aggressor_near: ParallelRc,
    pub victim_near: ParallelRc,
    pub aggressor_far: ParallelRc,
    pub victim_far: ParallelRc,
}

/// Комплексные отношения напряжений портов к ЭДС, с соглашением exp(jωt).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CouplingTransfer {
    pub aggressor_near: Complex<f64>,
    pub victim_near: Complex<f64>,
    pub aggressor_far: Complex<f64>,
    pub victim_far: Complex<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Mode {
    impedance_ohm: f64,
    delay_s: f64,
}

/// Пара трасс в установленной области модели.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledMicrostrip {
    lc: CoupledLc,
    even: Mode,
    odd: Mode,
    frequency_range_hz: [f64; 2],
}

impl CoupledMicrostrip {
    pub fn parameters(self) -> CoupledLc {
        self.lc
    }

    /// Включительные границы фактической полосы после отсечения по электрическим размерам.
    pub fn frequency_range_hz(self) -> [f64; 2] {
        self.frequency_range_hz
    }

    /// Передача на одной частоте. Для физической оценки нужны приложимость модели,
    /// спектр источника и измеренные уровни помехи в заданных плоскостях портов.
    pub fn transfer(
        self,
        ports: Option<Sourced<'_, CouplingPorts>>,
        frequency_hz: f64,
    ) -> Result<Analysis<CouplingTransfer>, SimError> {
        if !nonnegative(frequency_hz) {
            return Err(invalid("частота должна быть конечной и неотрицательной"));
        }
        if let Some(ports) = ports {
            let p = ports.value;
            for port in [
                p.aggressor_near,
                p.victim_near,
                p.aggressor_far,
                p.victim_far,
            ] {
                if !positive(port.resistance_ohm) || !nonnegative(port.capacitance_f) {
                    return Err(invalid("нагрузки требуют конечных R > 0 и C ≥ 0"));
                }
            }
        }
        let Some(p) = with_source(ports) else {
            return Ok(Analysis::NotAnalyzed(
                "нет нагрузок четырёх портов с источником. Нужны импедансы и привязка к опоре",
            ));
        };
        let [min, max] = self.frequency_range_hz;
        if !(min..=max).contains(&frequency_hz) {
            return Ok(Analysis::NotAnalyzed(
                "частота вне области модели пары трасс. Нужен замер передачи на тех же портах",
            ));
        }

        let omega = finite(TAU * frequency_hz)?;
        // MIT 6.013, §14.5. Для каждой моды цепная матрица имеет A = D = cos θ,
        // B = j Z sin θ, C = j sin θ / Z. Ток с обеих сторон направлен вдоль трасс.
        let (sin_even, cos_even) = (omega * self.even.delay_s).sin_cos();
        let (sin_odd, cos_odd) = (omega * self.odd.delay_s).sin_cos();
        let a = modal_matrix(cos_even, cos_odd).map(|x| Complex::new(x, 0.0));
        let b = modal_matrix(
            sin_even * self.even.impedance_ohm,
            sin_odd * self.odd.impedance_ohm,
        )
        .map(|x| Complex::new(0.0, x));
        let c = modal_matrix(
            sin_even / self.even.impedance_ohm,
            sin_odd / self.odd.impedance_ohm,
        )
        .map(|x| Complex::new(0.0, x));

        let far_admittance = |p: ParallelRc| {
            checked_complex(Complex::new(
                1.0 / p.resistance_ohm,
                omega * p.capacitance_f,
            ))
        };
        let y_far = Matrix2::from_diagonal(&Vector2::new(
            far_admittance(p.aggressor_far)?,
            far_admittance(p.victim_far)?,
        ));
        let voltage_in = a + b * y_far;
        let current_in = c + a * y_far;

        // Iд = Yд Vд. На ближней стороне Vб + Rб(Iб + jωCб Vб) = [1 В, 0 В].
        let mut system = voltage_in;
        let mut drive = Vector2::new(Complex::new(1.0, 0.0), Complex::new(0.0, 0.0));
        for (row, port) in [p.aggressor_near, p.victim_near].into_iter().enumerate() {
            let cap_gain = Complex::new(1.0, omega * port.capacitance_f * port.resistance_ohm);
            for col in 0..2 {
                system[(row, col)] = checked_complex(
                    cap_gain * voltage_in[(row, col)]
                        + current_in[(row, col)] * port.resistance_ohm,
                )?;
            }
            let scale = finite(system[(row, 0)].norm().max(system[(row, 1)].norm()))?;
            if scale == 0.0 {
                return Err(invalid("нулевая строка уравнений портов"));
            }
            for col in 0..2 {
                system[(row, col)] /= scale;
            }
            drive[row] = checked_complex(drive[row] / scale)?;
        }
        // После нормировки строк малый детерминант 2×2 усиливает ошибки округления.
        // √ε ограничивает усиление примерно половиной разрядов f64.
        if system.determinant().norm() <= f64::EPSILON.sqrt() {
            return Ok(Analysis::NotAnalyzed(
                "задача передачи плохо обусловлена. Нужен замер передачи с заданными нагрузками",
            ));
        }
        let far = system
            .lu()
            .solve(&drive)
            .ok_or_else(|| invalid("вырожденная система нагрузок пары трасс"))?;
        let near = voltage_in * far;
        Ok(Analysis::Calculated(CouplingTransfer {
            aggressor_near: checked_complex(near[0])?,
            victim_near: checked_complex(near[1])?,
            aggressor_far: checked_complex(far[0])?,
            victim_far: checked_complex(far[1])?,
        }))
    }
}

/// Рассчитать параметры из геометрии с обоснованной полосой.
pub fn coupled_microstrip(
    geometry: Option<Sourced<'_, MicrostripPair>>,
    band: Option<Sourced<'_, CouplingBand>>,
) -> Result<Analysis<CoupledMicrostrip>, SimError> {
    if let Some(g) = geometry {
        let g = g.value;
        if ![
            g.length_m,
            g.width_m,
            g.gap_m,
            g.height_m,
            g.relative_permittivity,
        ]
        .into_iter()
        .all(positive)
            || !nonnegative(g.thickness_m)
        {
            return Err(invalid(
                "размеры и проницаемость пары трасс должны быть конечными и физически допустимыми",
            ));
        }
    }
    if let Some(b) = band {
        let b = b.value;
        if !nonnegative(b.min_frequency_hz)
            || !positive(b.max_frequency_hz)
            || b.min_frequency_hz > b.max_frequency_hz
            || !positive(b.max_wavelength_fraction)
            || b.max_wavelength_fraction > MAX_WAVELENGTH_FRACTION
        {
            return Err(invalid(
                "некорректная полоса или граница электрической длины пары трасс",
            ));
        }
    }

    let (Some(g), Some(b)) = (with_source(geometry), with_source(band)) else {
        return Ok(Analysis::NotAnalyzed(
            "нет геометрии или полосы с источником. Нужны размеры, подложка и замер передачи",
        ));
    };
    let u = finite(g.width_m / g.height_m)?;
    let gap = finite(g.gap_m / g.height_m)?;
    let er = g.relative_permittivity;
    // Киршнинг и Янсен, формула (1), область статических выражений для полосок
    // нулевой толщины. DOI 10.1109/TMTT.1984.1132616.
    if !g.uniform_over_plane
        || g.thickness_m != 0.0
        || !(0.1..=10.0).contains(&u)
        || !(0.1..=10.0).contains(&gap)
        || !(1.0..=18.0).contains(&er)
    {
        return Ok(Analysis::NotAnalyzed(
            "геометрия вне области модели равных тонких трасс. Нужен замер или внешний расчёт пары",
        ));
    }
    let width = finite(2.0 * g.width_m + g.gap_m)?;
    let extent = g.length_m.max(width).max(g.height_m);
    // Для обеих мод εeff ≤ εr, поэтому c/√εr консервативно ограничивает λ снизу.
    let length_limit = finite((b.max_wavelength_fraction * LIGHT_SPEED_M_S / er.sqrt()) / extent)?;
    if length_limit <= 0.0 {
        return Err(invalid(
            "граница электрической длины вышла за численный диапазон",
        ));
    }
    let high = length_limit.min(b.max_frequency_hz);
    if b.min_frequency_hz > high {
        return Ok(Analysis::NotAnalyzed(
            "заявленная полоса выше предела по электрическим размерам. Нужен замер передачи",
        ));
    }

    let (even, odd) = static_modes(u, gap, er, g.length_m);
    if ![
        even.impedance_ohm,
        odd.impedance_ohm,
        even.delay_s,
        odd.delay_s,
    ]
    .into_iter()
    .all(positive)
    {
        return Err(invalid(
            "модальные параметры пары трасс вышли за численный диапазон",
        ));
    }
    // Для моды Z = √(L'/C'), τ = ℓ√(L'C'). Обратный переход в базис трасс
    // даёт промежуточные суммы и разности собственных значений матриц C и L.
    let ce = even.delay_s / even.impedance_ohm;
    let co = odd.delay_s / odd.impedance_ohm;
    let le = even.delay_s * even.impedance_ohm;
    let lo = odd.delay_s * odd.impedance_ohm;
    if ![ce, co, le, lo].into_iter().all(positive) {
        return Err(invalid(
            "ёмкость или индуктивность вышла за численный диапазон",
        ));
    }
    let lc = CoupledLc {
        self_c_f: ce / 2.0 + co / 2.0,
        mutual_c_f: (co - ce) / 2.0,
        self_l_h: le / 2.0 + lo / 2.0,
        mutual_l_h: (le - lo) / 2.0,
    };
    if !(0.0 < lc.mutual_c_f
        && lc.mutual_c_f < lc.self_c_f
        && 0.0 < lc.mutual_l_h
        && lc.mutual_l_h < lc.self_l_h)
    {
        return Ok(Analysis::NotAnalyzed(
            "модель не разрешает пассивные взаимные C/L. Нужен замер параметров пары",
        ));
    }
    Ok(Analysis::Calculated(CoupledMicrostrip {
        lc,
        even,
        odd,
        frequency_range_hz: [b.min_frequency_hz, high],
    }))
}

// Киршнинг и Янсен, статические формулы (3), (4), (8), (9), в записи Qucs
// (11.91–11.99, 11.118–11.129). Коэффициенты ниже принадлежат этим формулам.
// https://qucs.sourceforge.net/tech/node77.html
fn static_modes(u: f64, gap: f64, er: f64, length_m: f64) -> (Mode, Mode) {
    // Для εeff чётной моды формула Хаммерстада–Йенсена применяется
    // к эквивалентной нормированной ширине v. Высота нормирована к единице.
    let single = microstrip_hammerstad_jensen(u, 1.0, 0.0, er);
    let exp_gap = (-gap).exp();
    let v = u * (20.0 + gap * gap) / (10.0 + gap * gap) + gap * exp_gap;
    let eps_even = microstrip_hammerstad_jensen(v, 1.0, 0.0, er).eps_eff;
    let eps_mid = (er + 1.0) / 2.0;
    let a0 = 0.7287 * (single.eps_eff - eps_mid) * -(-0.179 * u).exp_m1();
    let b0 = 0.747 * er / (0.15 + er);
    let c0 = b0 - (b0 - 0.207) * (-0.414 * u).exp();
    let d0 = 0.593 + 0.694 * (-0.562 * u).exp();
    let eps_odd = single.eps_eff + (eps_mid + a0 - single.eps_eff) * (-c0 * gap.powf(d0)).exp();

    let q1 = 0.8695 * u.powf(0.194);
    let q2 = 1.0 + 0.7519 * gap + 0.189 * gap.powf(2.31);
    let q3 = 0.1975
        + (16.6 + (8.4 / gap).powi(6)).powf(-0.387)
        + (gap.powi(10) / (1.0 + (gap / 3.4).powi(10))).ln() / 241.0;
    let q4 = (2.0 * q1 / q2) / (exp_gap * u.powf(q3) + (2.0 - exp_gap) * u.powf(-q3));
    let q5 = 1.794 + 1.14 * (0.638 / (gap + 0.517 * gap.powf(2.43))).ln_1p();
    let q6 = 0.2305
        + (gap.powi(10) / (1.0 + (gap / 5.8).powi(10))).ln() / 281.3
        + (0.598 * gap.powf(1.154)).ln_1p() / 5.1;
    let q7 = (10.0 + 190.0 * gap * gap) / (1.0 + 82.3 * gap.powi(3));
    let q8 = (-6.5 - 0.95 * gap.ln() - (gap / 0.15).powi(5)).exp();
    let q9 = q7.ln() * (q8 + 1.0 / 16.5);
    let q10 = q4 - q5 / q2 * u.powf(q6 * u.powf(-q9));
    let z_air = single.z0 * single.eps_eff.sqrt();
    let mode = |eps: f64, q: f64| Mode {
        impedance_ohm: z_air / (eps.sqrt() * (1.0 - z_air / 377.0 * q)),
        delay_s: length_m * (eps.sqrt() / LIGHT_SPEED_M_S),
    };
    (mode(eps_even, q4), mode(eps_odd, q10))
}

// Смена базиса с чётной (1, 1) и нечётной (1, −1) мод на напряжения трасс.
fn modal_matrix(even: f64, odd: f64) -> Matrix2<f64> {
    let diagonal = (even + odd) / 2.0;
    let mutual = (even - odd) / 2.0;
    Matrix2::new(diagonal, mutual, mutual, diagonal)
}
