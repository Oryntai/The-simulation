//! Датчики: питание, ток, диапазон, разрешение, точность, интерфейс.

use crate::spec::Spec;

/// Что измеряет датчик.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorKind {
    Angle,
    Encoder,
    Imu,
    Distance,
    ForceAdc,
    LoadCell,
    ForceResistor,
    PowerMonitor,
}

impl SensorKind {
    pub fn title(self) -> &'static str {
        match self {
            SensorKind::Angle => "датчик угла",
            SensorKind::Encoder => "энкодер",
            SensorKind::Imu => "инерциальный датчик",
            SensorKind::Distance => "дальномер",
            SensorKind::ForceAdc => "АЦП для тензодатчика",
            SensorKind::LoadCell => "тензодатчик",
            SensorKind::ForceResistor => "резистивный датчик силы",
            SensorKind::PowerMonitor => "монитор тока и мощности",
        }
    }
}

/// Датчик из каталога.
#[derive(Clone, Debug)]
pub struct Sensor {
    pub id: &'static str,
    pub name: &'static str,
    pub kind: SensorKind,
    pub what_is: &'static str,
    /// Допустимые диапазоны питания, В (у некоторых датчиков их два).
    pub supply_ranges: &'static [(f64, f64)],
    /// Рабочий ток, А.
    pub current_a: Spec,
    /// Пиковый ток, А.
    pub current_peak_a: Spec,
    /// Диапазон измерения.
    pub range_min: Spec,
    pub range_max: Spec,
    pub range_unit: &'static str,
    pub resolution: Spec,
    pub resolution_unit: &'static str,
    /// Погрешность (модуль).
    pub accuracy: Spec,
    pub accuracy_unit: &'static str,
    pub interface: &'static str,
    pub sources: &'static [&'static str],
    pub caveats: &'static [&'static str],
}

impl Sensor {
    /// Подходит ли напряжение питания.
    pub fn supply_ok(&self, v: f64) -> bool {
        v.is_finite() && self.supply_ranges.iter().any(|&(a, b)| v >= a && v <= b)
    }

    /// Ток для расчёта питания: пиковый, если он опубликован, иначе рабочий.
    pub fn worst_current_a(&self) -> Option<f64> {
        match (self.current_peak_a.get(), self.current_a.get()) {
            (Some(peak), Some(avg)) => Some(peak.max(avg)),
            (peak, avg) => peak.or(avg),
        }
    }

    fn blank(
        id: &'static str,
        name: &'static str,
        kind: SensorKind,
        what_is: &'static str,
        interface: &'static str,
    ) -> Self {
        Self {
            id,
            name,
            kind,
            what_is,
            supply_ranges: &[],
            current_a: Spec::NONE,
            current_peak_a: Spec::NONE,
            range_min: Spec::NONE,
            range_max: Spec::NONE,
            range_unit: "",
            resolution: Spec::NONE,
            resolution_unit: "",
            accuracy: Spec::NONE,
            accuracy_unit: "",
            interface,
            sources: &[],
            caveats: &[],
        }
    }
}

const AS5600: &str = "ams AS5600 v1-06: https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf";
const AS5048: &str =
    "ams AS5048A/B v1-11: https://cdn.sparkfun.com/assets/e/d/e/1/d/AS5048_DS000298_4-00.pdf";
const AMT: &str = "Same Sky AMT10: https://www.sameskydevices.com/product/resource/amt10-v.pdf";
const MPU: &str = "TDK InvenSense PS-MPU-6000A-00: https://invensense.tdk.com/wp-www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf";
const BMI: &str = "Bosch BST-BMI270-DS000-08: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf";
const VL53: &str = "ST VL53L0X DS11555 Rev 6: https://www.st.com/resource/en/datasheet/vl53l0x.pdf";
const HX711: &str =
    "Avia HX711: https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf";
const TAL: &str = "TAL220B, листинг Little Bird: https://littlebirdelectronics.com.au/products/load-cell-5kg-straight-bar-tal220b";
const TAL2: &str =
    "SparkFun SEN-14729: https://www.sparkfun.com/load-cell-5kg-straight-bar-tal220b.html";
const TAL3: &str = "DigiKey, SparkFun 14729: https://www.digikey.cn/en/products/detail/sparkfun-electronics/14729/9555603";
const FSR: &str = "Interlink FSR 400 Series: https://www.teachengineering.org/content/uconn_/activities/uconn-2662-contractions-calculations-force-resistance-activity/uconn-2662-contractions-calculations-FSR400series-datasheet.pdf";
const INA: &str = "TI INA219, SBOS448G: https://www.ti.com/lit/ds/symlink/ina219.pdf";

/// Все датчики каталога.
pub(crate) fn build() -> Vec<Sensor> {
    vec![
        Sensor {
            supply_ranges: &[(3.0, 3.6), (4.5, 5.5)],
            current_a: Spec::ds(0.0065, AS5600),
            range_min: Spec::ds(0.0, AS5600),
            range_max: Spec::ds(360.0, AS5600),
            range_unit: "°",
            resolution: Spec::derived(360.0 / 4096.0, "12 бит на 360°"),
            resolution_unit: "°",
            accuracy: Spec::ds(1.0, "ams: нелинейность ±1° без смещения магнита"),
            accuracy_unit: "°",
            sources: &[AS5600],
            caveats: &["Точность ±1° — без смещения магнита; при смещении хуже."],
            ..Sensor::blank(
                "as5600",
                "AS5600",
                SensorKind::Angle,
                "Магнитный датчик угла: читает поворот магнитика на валу без контакта.",
                "I²C, ШИМ, аналоговый",
            )
        },
        Sensor {
            supply_ranges: &[(3.0, 3.6), (4.5, 5.5)],
            current_a: Spec::ds(0.015, AS5048),
            range_min: Spec::ds(0.0, AS5048),
            range_max: Spec::ds(360.0, AS5048),
            range_unit: "°",
            resolution: Spec::ds(0.0219, AS5048),
            resolution_unit: "°",
            accuracy: Spec::ds(1.2, "ams: ±1.2° при смещении магнита и по всей температуре"),
            accuracy_unit: "°",
            sources: &[AS5048],
            caveats: &["±0.8° только при идеальной установке магнита и 25 °C."],
            ..Sensor::blank(
                "as5048a",
                "AS5048A",
                SensorKind::Encoder,
                "Магнитный абсолютный энкодер 14 бит: знает угол сразу после включения.",
                "SPI, ШИМ",
            )
        },
        Sensor {
            supply_ranges: &[(3.6, 5.5)],
            current_a: Spec::ds(0.006, "Same Sky, типовой ток без нагрузки выходов"),
            range_min: Spec::ds(0.0, AMT),
            range_max: Spec::ds(
                7500.0,
                "Same Sky: 7500/15000 об/мин, зависит от разрешения; берём меньшее",
            ),
            range_unit: "об/мин",
            resolution: Spec::ds(2048.0, AMT),
            resolution_unit: "импульсов/об",
            accuracy: Spec::ds(0.25, AMT),
            accuracy_unit: "°",
            interface: "квадратурный A/B + индекс",
            sources: &[AMT],
            caveats: &[
                "6 мА типовые для выходов без нагрузки. Верхний ток с обвязкой не опубликован.",
                "Энкодер относительный: после включения нужен поиск нуля.",
            ],
            ..Sensor::blank(
                "amt102-v",
                "AMT102-V",
                SensorKind::Encoder,
                "Ёмкостный инкрементальный энкодер на вал мотора, до 2048 импульсов на оборот.",
                "квадратурный A/B + индекс",
            )
        },
        Sensor {
            supply_ranges: &[(2.375, 3.46)],
            current_a: Spec::ds(0.0038, "InvenSense: гироскоп + акселерометр, без DMP"),
            current_peak_a: Spec::NONE,
            range_min: Spec::ds(-2000.0, MPU),
            range_max: Spec::ds(2000.0, MPU),
            range_unit: "°/с (акселерометр до ±16 g)",
            resolution: Spec::derived(1.0 / 16.4, "16 бит, 16.4 LSB/(°/с) при ±2000 °/с"),
            resolution_unit: "°/с",
            accuracy: Spec::ds(
                0.005,
                "InvenSense: шум 0.005 °/с/√Гц, акселерометр 400 мкg/√Гц",
            ),
            accuracy_unit: "°/с/√Гц",
            sources: &[MPU],
            caveats: &[
                "Снят с производства (EOL), на рынке много копий.",
                "Сам чип питается до 3.46 В: на 5 В только через стабилизатор модуля.",
            ],
            ..Sensor::blank(
                "mpu6050",
                "MPU-6050",
                SensorKind::Imu,
                "Шестиосевой датчик движения: гироскоп и акселерометр.",
                "I²C",
            )
        },
        Sensor {
            supply_ranges: &[(1.71, 3.6)],
            current_a: Spec::ds(0.000685, "Bosch: A+G, обычный режим"),
            current_peak_a: Spec::NONE,
            range_min: Spec::ds(-2000.0, BMI),
            range_max: Spec::ds(2000.0, BMI),
            range_unit: "°/с (акселерометр до ±16 g)",
            resolution: Spec::derived(4000.0 / 65536.0, "16 бит на ±2000 °/с"),
            resolution_unit: "°/с",
            accuracy: Spec::ds(0.007, "Bosch: шум 0.007 °/с/√Гц, акселерометр 0.16 мg/√Гц"),
            accuracy_unit: "°/с/√Гц",
            sources: &[BMI],
            caveats: &["Предел питания 3.6 В: от 5 В напрямую питать нельзя."],
            ..Sensor::blank(
                "bmi270",
                "BMI270",
                SensorKind::Imu,
                "Современный экономичный шестиосевой датчик движения.",
                "SPI, I²C",
            )
        },
        Sensor {
            supply_ranges: &[(2.6, 3.5)],
            current_a: Spec::ds(0.019, "ST: среднее при измерении, бюджет 33 мс"),
            current_peak_a: Spec::ds(0.040, "ST: пик вместе с лазером"),
            range_min: Spec::NONE,
            range_max: Spec::ds(2.0, "ST: до 2 м, режим long range, только в темноте"),
            range_unit: "м",
            resolution: Spec::NONE,
            accuracy: Spec::ds(
                12.0,
                "ST, табл. 12: до 12 % для серой цели на улице при 33 мс",
            ),
            accuracy_unit: "%",
            sources: &[VL53],
            caveats: &[
                "Минимальная дальность в паспорте не указана.",
                "Точность сильно зависит от цвета цели и освещения.",
            ],
            ..Sensor::blank(
                "vl53l0x",
                "VL53L0X",
                SensorKind::Distance,
                "Лазерный дальномер по времени пролёта, до 2 м в темноте.",
                "I²C",
            )
        },
        Sensor {
            supply_ranges: &[(2.6, 5.5)],
            current_a: Spec::derived(0.0015, "Avia: аналоговая часть 1400 мкА + цифровая 100 мкА"),
            range_min: Spec::derived(-20.0, "±0.5·AVDD/128 при AVDD = 5 В, мВ"),
            range_max: Spec::derived(20.0, "±0.5·AVDD/128 при AVDD = 5 В, мВ"),
            range_unit: "мВ на входе",
            resolution: Spec::ds(24.0, HX711),
            resolution_unit: "бит",
            accuracy: Spec::ds(
                90.0,
                "Avia: шум 90 нВ rms при 80 Гц, усиление 128 (50 нВ при 10 Гц)",
            ),
            accuracy_unit: "нВ rms",
            sources: &[HX711],
            caveats: &[
                "Ток не включает питание моста тензодатчика: его считает отдельная позиция.",
                "Частота только 10 или 80 отсчётов/с: для быстрого контура силы мало.",
            ],
            ..Sensor::blank(
                "hx711",
                "HX711",
                SensorKind::ForceAdc,
                "24-битный АЦП для тензодатчика: превращает милливольты моста в число.",
                "двухпроводной последовательный (DOUT, SCK)",
            )
        },
        Sensor {
            supply_ranges: &[(3.0, 10.0)],
            current_a: Spec::derived(0.005, "5 В / 1000 Ом входного сопротивления моста"),
            current_peak_a: Spec::derived(0.010, "10 В / 1000 Ом, наибольшее возбуждение"),
            range_min: Spec::ds(0.0, TAL2),
            range_max: Spec::ds(5.0, TAL2),
            range_unit: "кг",
            resolution: Spec::NONE,
            accuracy: Spec::vendor(0.05, TAL3),
            accuracy_unit: "% ПШ",
            sources: &[TAL, TAL2, TAL3],
            caveats: &[
                "Выход 1.0 ± 0.1 мВ/В: в листинге опечатка «1.0V ± 0.1V».",
                "Безопасная перегрузка в доступных данных не указана.",
            ],
            ..Sensor::blank(
                "tal220b-5kg",
                "Тензодатчик TAL220B 5 кг",
                SensorKind::LoadCell,
                "Алюминиевая балка с тензомостом: точно меряет вес и усилие, читается через HX711.",
                "аналоговый мост, через HX711",
            )
        },
        Sensor {
            supply_ranges: &[],
            range_min: Spec::ds(0.2, FSR),
            range_max: Spec::ds(20.0, FSR),
            range_unit: "Н",
            accuracy: Spec::ds(6.0, "Interlink: ±6 % от детали к детали в одной партии"),
            accuracy_unit: "%",
            sources: &[FSR],
            caveats: &[
                "Не прибор для измерения силы: гистерезис +10 %, дрейф под нагрузкой.",
                "Ток задаёт внешний делитель, питание паспорт не нормирует.",
            ],
            ..Sensor::blank(
                "fsr402",
                "FSR 402",
                SensorKind::ForceResistor,
                "Плёночная кнопка-датчик: сопротивление падает при нажатии, годится для «есть касание».",
                "аналоговый делитель",
            )
        },
        Sensor {
            supply_ranges: &[(3.0, 5.5)],
            current_a: Spec::ds(0.0007, INA),
            current_peak_a: Spec::ds(0.001, INA),
            range_min: Spec::ds(0.0, INA),
            range_max: Spec::ds(26.0, INA),
            range_unit: "В на шине (шунт до ±320 мВ)",
            resolution: Spec::ds(12.0, INA),
            resolution_unit: "бит",
            accuracy: Spec::ds(
                1.0,
                "TI, INA219A: до ±1 % по току во всём диапазоне температур",
            ),
            accuracy_unit: "%",
            sources: &[INA],
            caveats: &["Ток в амперах зависит от выбранного шунта."],
            ..Sensor::blank(
                "ina219",
                "INA219",
                SensorKind::PowerMonitor,
                "Измеритель тока и напряжения на шунте: показывает, сколько реально ест устройство.",
                "I²C",
            )
        },
    ]
}
