//! Заводская калибровка SAR ADC1 ESP32-C3. Преобразование повторяет ESP-IDF v6.1.1
//! для сырого кода, снятого после аппаратной установки `init_code`.
//! Даташит §4.2.2.1 указывает заводскую калибровку только для ADC1.

use std::fmt;

// ESP-IDF components/soc/esp32c3/register/soc/efuse_reg.h: смещения
// EFUSE_RD_WR_DIS_REG, EFUSE_RD_SYS_PART2_DATA7_REG и EFUSE_RD_SYS_PART1_DATA4_REG.
const READBACK_START: usize = 0x2c;
const READBACK_LAST: usize = 0x178;
const BLOCK2_DATA4: usize = 0x6c;
const WORD_BYTES: usize = size_of::<u32>();
const READBACK_WORDS: usize = (READBACK_LAST - READBACK_START) / WORD_BYTES + 1;

// ESP-IDF components/efuse/esp32c3/esp_efuse_table.csv: BLOCK2, поля
// BLK_VERSION_MAJOR, ADC1_INIT_CODE_ATTEN0..3 и ADC1_CAL_VOL_ATTEN0..3.
const BLOCK2_VERSION_BIT: usize = 128;
const BLOCK2_VERSION_WIDTH: usize = 2;
const FIRST_INIT_BIT: usize = 148;
const FIRST_CAL_VOLTAGE_BIT: usize = 188;
const CAL_FIELD_WIDTH: usize = 10;

// ESP-IDF components/efuse/esp32c3/esp_efuse_rtc_calib.c и include/esp_efuse_rtc_calib.h:
// версия, смещения кодов и напряжения заводских опорных точек.
const CAL_VERSION: u8 = 1;
const INIT_BASE: u16 = 1000;
const DIGI_BASE: u16 = 2000;
const REFERENCE_MV: [u16; 4] = [400, 550, 750, 1370];

// TRM v1.4 §34.2.3.2 и ESP-IDF soc/esp32c3/include/soc/soc_caps.h:
// SOC_ADC_RTC_MAX_BITWIDTH и SOC_ADC_DIGI_MAX_BITWIDTH равны 12.
const MAX_RAW: u32 = (1 << 12) - 1;
// ESP-IDF components/esp_adc/adc_cali_curve_fitting.c, coeff_a_scaling.
const SLOPE_SCALE: u64 = 65_536;

// ESP-IDF components/esp_adc/esp32c3/curve_fitting_coefficients.c:
// adc1_error_coef_atten и adc1_error_sign, с объединённым знаком числителя.
const ERROR_TERMS: [&[(i64, u64)]; 4] = [
    &[
        (-225_966_470_500_043, 10u64.pow(15)),
        (-7_265_418_501_948, 10u64.pow(16)),
        (109_410_402_681, 10u64.pow(16)),
    ],
    &[
        (4_229_623_392_600_516, 10u64.pow(16)),
        (-731_527_490_903, 10u64.pow(16)),
        (88_166_562_521, 10u64.pow(16)),
    ],
    &[
        (-1_017_859_239_236_435, 10u64.pow(15)),
        (-97_159_265_299_153, 10u64.pow(16)),
        (149_794_028_038, 10u64.pow(16)),
    ],
    &[
        (-14_912_262_772_850_453, 10u64.pow(16)),
        (-228_549_975_564_099, 10u64.pow(16)),
        (356_391_935_717, 10u64.pow(16)),
        (-179_964_582, 10u64.pow(16)),
        (42_046, 10u64.pow(16)),
    ],
];

/// Коды ATTEN0..3 из ESP-IDF components/esp_hal_ana_conv/include/hal/adc_types.h.
/// Номинальное ослабление соответствует TRM v1.4 §34.2.3.2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EspAdcAttenuation {
    Db0 = 0,
    Db2_5 = 1,
    Db6 = 2,
    Db12 = 3,
}

impl TryFrom<u8> for EspAdcAttenuation {
    type Error = EspAdcError;

    fn try_from(code: u8) -> Result<Self, Self::Error> {
        match code {
            x if x == Self::Db0 as u8 => Ok(Self::Db0),
            x if x == Self::Db2_5 as u8 => Ok(Self::Db2_5),
            x if x == Self::Db6 as u8 => Ok(Self::Db6),
            x if x == Self::Db12 as u8 => Ok(Self::Db12),
            _ => Err(EspAdcError::InvalidAttenuation(code)),
        }
    }
}

/// Эффективный диапазон входа и суммарная погрешность после аппаратной и программной
/// калибровки, ESP32-C3 Series Datasheet v2.5 §5.5, таблица 5-6.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspAdcLimits {
    pub min_input_mv: u16,
    pub max_input_mv: u16,
    pub max_abs_error_mv: u16,
}

impl EspAdcAttenuation {
    pub const fn limits(self) -> EspAdcLimits {
        // ESP32-C3 Series Datasheet v2.5 §5.5, таблица 5-6, строки ATTEN0..3.
        let (max_input_mv, max_abs_error_mv) = match self {
            Self::Db0 => (750, 10),
            Self::Db2_5 => (1050, 10),
            Self::Db6 => (1300, 10),
            Self::Db12 => (2500, 35),
        };
        EspAdcLimits {
            min_input_mv: 0,
            max_input_mv,
            max_abs_error_mv,
        }
    }

    /// Проверяет заданное напряжение на физическом входе в мВ, включая края диапазона.
    pub fn validate_input_mv(self, input_mv: f64) -> Result<(), EspAdcError> {
        if !input_mv.is_finite() {
            return Err(EspAdcError::NonFiniteInput);
        }
        let limits = self.limits();
        let range = f64::from(limits.min_input_mv)..=f64::from(limits.max_input_mv);
        if !range.contains(&input_mv) {
            return Err(EspAdcError::OutsideEffectiveRange(self));
        }
        Ok(())
    }
}

/// Аппаратный код и заводская опорная точка для одного ослабления.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspAdcPoint {
    pub init_code: u16,
    pub reference_raw: u16,
    pub reference_mv: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EspAdcError {
    /// `None` означает отсутствие снимка. Ноль в BLOCK2 означает отсутствие калибровки.
    NotAnalyzed {
        block_version: Option<u8>,
    },
    InvalidReadbackLength {
        expected: usize,
        actual: usize,
    },
    InvalidAttenuation(u8),
    InvalidRaw(u32),
    NonFiniteInput,
    OutsideEffectiveRange(EspAdcAttenuation),
}

impl fmt::Display for EspAdcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotAnalyzed {
                block_version: None,
            } => {
                write!(f, "NotAnalyzed: снимок eFuse для ADC1 отсутствует")
            }
            Self::NotAnalyzed {
                block_version: Some(0),
            } => {
                write!(
                    f,
                    "NotAnalyzed: в eFuse BLOCK2 нет заводской калибровки ADC1"
                )
            }
            Self::NotAnalyzed {
                block_version: Some(version),
            } => {
                write!(
                    f,
                    "NotAnalyzed: версия eFuse BLOCK2 {version} не поддержана для ADC1"
                )
            }
            Self::InvalidReadbackLength { expected, actual } => {
                write!(f, "снимок eFuse: нужно {expected} слов, передано {actual}")
            }
            Self::InvalidAttenuation(code) => write!(f, "неизвестное ослабление ADC1: {code}"),
            Self::InvalidRaw(raw) => write!(f, "код ADC1 {raw} превышает {MAX_RAW}"),
            Self::NonFiniteInput => write!(f, "напряжение входа ADC1 должно быть конечным"),
            Self::OutsideEffectiveRange(atten) => {
                let limits = atten.limits();
                write!(
                    f,
                    "напряжение входа ADC1 вне диапазона {}..={} мВ",
                    limits.min_input_mv, limits.max_input_mv
                )
            }
        }
    }
}

impl std::error::Error for EspAdcError {}

/// Профиль ADC1 версии 1. Все каналы ADC1 используют одни заводские опорные точки.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspAdcCalibration {
    points: [EspAdcPoint; REFERENCE_MV.len()],
}

impl EspAdcCalibration {
    /// Слова от EFUSE_RD_WR_DIS_REG до EFUSE_RD_SYS_PART2_DATA7_REG включительно,
    /// в том же порядке, что у `EspMemory::install_efuse_readback`.
    pub fn from_efuse_readback(words: Option<&[u32]>) -> Result<Self, EspAdcError> {
        let words = words.ok_or(EspAdcError::NotAnalyzed {
            block_version: None,
        })?;
        if words.len() != READBACK_WORDS {
            return Err(EspAdcError::InvalidReadbackLength {
                expected: READBACK_WORDS,
                actual: words.len(),
            });
        }

        // efuse_reg.h: поля калибровки занимают DATA4..=DATA7, младшее слово первым.
        let first_word = (BLOCK2_DATA4 - READBACK_START) / WORD_BYTES;
        let packed = words[first_word..first_word + 4]
            .iter()
            .rev()
            .fold(0_u128, |acc, &word| (acc << u32::BITS) | u128::from(word));
        let field =
            |bit, width| ((packed >> (bit - BLOCK2_VERSION_BIT)) & ((1_u128 << width) - 1)) as u16;

        let version = field(BLOCK2_VERSION_BIT, BLOCK2_VERSION_WIDTH) as u8;
        if version != CAL_VERSION {
            return Err(EspAdcError::NotAnalyzed {
                block_version: Some(version),
            });
        }

        let points = std::array::from_fn(|atten| {
            let shift = atten * CAL_FIELD_WIDTH;
            let init = field(FIRST_INIT_BIT + shift, CAL_FIELD_WIDTH);
            let cal = field(FIRST_CAL_VOLTAGE_BIT + shift, CAL_FIELD_WIDTH);
            // esp_efuse_rtc_calib.c: знак в бите 9, младшие биты хранят модуль.
            let sign_bit = 1 << (CAL_FIELD_WIDTH - 1);
            let reference_raw = if cal & sign_bit == 0 {
                DIGI_BASE + cal
            } else {
                DIGI_BASE - (cal & !sign_bit)
            };
            EspAdcPoint {
                init_code: INIT_BASE + init,
                reference_raw,
                reference_mv: REFERENCE_MV[atten],
            }
        });
        Ok(Self { points })
    }

    pub fn point(&self, attenuation: EspAdcAttenuation) -> EspAdcPoint {
        self.points[attenuation as usize]
    }

    /// Показание в мВ по алгоритму `adc_cali_raw_to_voltage`. Применимость паспортной
    /// погрешности определяется по фактическому входу и выполнению аппаратной калибровки.
    pub fn raw_to_mv(&self, attenuation: EspAdcAttenuation, raw: u32) -> Result<i32, EspAdcError> {
        if raw > MAX_RAW {
            return Err(EspAdcError::InvalidRaw(raw));
        }
        let point = self.point(attenuation);
        // adc_cali_curve_fitting.c: coeff_b = 0, деления по этапам с отбрасыванием остатка.
        let slope = SLOPE_SCALE * u64::from(point.reference_mv) / u64::from(point.reference_raw);
        let first_mv = u64::from(raw) * slope / SLOPE_SCALE;
        if first_mv == 0 {
            return Ok(0);
        }
        // Отбрасывание остатка у каждого члена до применения его знака, как в
        // get_reading_error. При 12-битном raw и опорных точках v1 произведения
        // помещаются в uint64_t: максимальный first_mv равен 3767 при ATTEN3.
        let correction: i32 = ERROR_TERMS[attenuation as usize]
            .iter()
            .enumerate()
            .map(|(degree, &(numerator, denominator))| {
                let magnitude =
                    first_mv.pow(degree as u32) * numerator.unsigned_abs() / denominator;
                magnitude as i32 * numerator.signum() as i32
            })
            .sum();
        Ok(first_mv as i32 - correction)
    }
}
