//! Драйверы моторов: напряжение, ток, падение на ключах, тепловой предел.
//!
//! Падение напряжения берётся по максимальным значениям паспорта (горячий кристалл),
//! потому что именно оно отнимает напряжение у мотора и греет корпус.

use crate::actuator::MotorType;
use crate::spec::Spec;

/// Вид драйвера.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverKind {
    /// H-мост для коллекторного мотора или одной обмотки.
    HBridge,
    /// Драйвер шагового двигателя с токовым чоппером.
    StepperChopper,
    /// Сервоконтроллер (коллекторный и BLDC).
    ServoController,
    /// Встроен в привод, отдельно не подключается.
    Integrated,
}

impl DriverKind {
    pub fn title(self) -> &'static str {
        match self {
            DriverKind::HBridge => "H-мост",
            DriverKind::StepperChopper => "драйвер шагового двигателя",
            DriverKind::ServoController => "сервоконтроллер",
            DriverKind::Integrated => "встроенный драйвер",
        }
    }
}

/// Драйвер из каталога. Токи — на один канал (мост).
#[derive(Clone, Debug)]
pub struct Driver {
    pub id: &'static str,
    pub name: &'static str,
    pub kind: DriverKind,
    pub what_is: &'static str,
    pub supports: &'static [MotorType],

    pub voltage_min_v: Spec,
    /// Верхний предел рабочего напряжения.
    pub voltage_max_v: Spec,
    /// Абсолютный максимум (работать на нём нельзя).
    pub voltage_abs_max_v: Spec,

    pub current_continuous_a: Spec,
    /// Минимальное питание, при котором нормирован указанный длительный ток.
    pub current_rating_min_voltage_v: Spec,
    /// Ниже этого напряжения длительный ток меньше: (порог, В; ток, А).
    pub low_voltage_current: Option<(f64, f64)>,
    pub current_peak_a: Spec,
    /// Сколько секунд допускается ток выше длительного.
    pub peak_max_duration_s: Spec,

    /// Сопротивление пути тока «верхний + нижний ключ», максимум для горячего кристалла, Ом.
    pub r_on_hot_ohm: Spec,
    /// Падение на биполярном мосту: точки (ток, А; максимум падения, В).
    pub drop_table: &'static [(f64, f64)],
    /// Доля напряжения питания на выходе (сервоконтроллеры).
    pub output_fraction: Spec,

    pub r_theta_ja_k_per_w: Spec,
    pub r_theta_jc_k_per_w: Spec,
    pub t_j_max_c: Spec,
    /// Нижняя граница порога тепловой защиты, °C.
    pub t_shutdown_c: Spec,
    pub max_ambient_c: Spec,
    /// Длительный ток нормирован для готового модуля при `max_ambient_c`.
    pub rated_module: bool,
    pub quiescent_current_a: Spec,

    pub sources: &'static [&'static str],
    pub caveats: &'static [&'static str],
}

impl Driver {
    pub fn blank(
        id: &'static str,
        name: &'static str,
        kind: DriverKind,
        what_is: &'static str,
        supports: &'static [MotorType],
    ) -> Self {
        Self {
            id,
            name,
            kind,
            what_is,
            supports,
            voltage_min_v: Spec::NONE,
            voltage_max_v: Spec::NONE,
            voltage_abs_max_v: Spec::NONE,
            current_continuous_a: Spec::NONE,
            current_rating_min_voltage_v: Spec::NONE,
            low_voltage_current: None,
            current_peak_a: Spec::NONE,
            peak_max_duration_s: Spec::NONE,
            r_on_hot_ohm: Spec::NONE,
            drop_table: &[],
            output_fraction: Spec::NONE,
            r_theta_ja_k_per_w: Spec::NONE,
            r_theta_jc_k_per_w: Spec::NONE,
            t_j_max_c: Spec::NONE,
            t_shutdown_c: Spec::NONE,
            max_ambient_c: Spec::NONE,
            rated_module: false,
            quiescent_current_a: Spec::NONE,
            sources: &[],
            caveats: &[],
        }
    }

    /// Числовые поля (кроме температур) — для проверки на NaN/∞ и знак.
    pub fn numeric_specs(&self) -> Vec<(&'static str, Spec)> {
        vec![
            ("voltage_min_v", self.voltage_min_v),
            ("voltage_max_v", self.voltage_max_v),
            ("voltage_abs_max_v", self.voltage_abs_max_v),
            ("current_continuous_a", self.current_continuous_a),
            (
                "current_rating_min_voltage_v",
                self.current_rating_min_voltage_v,
            ),
            ("current_peak_a", self.current_peak_a),
            ("peak_max_duration_s", self.peak_max_duration_s),
            ("r_on_hot_ohm", self.r_on_hot_ohm),
            ("output_fraction", self.output_fraction),
            ("r_theta_ja_k_per_w", self.r_theta_ja_k_per_w),
            ("r_theta_jc_k_per_w", self.r_theta_jc_k_per_w),
            ("quiescent_current_a", self.quiescent_current_a),
        ]
    }

    pub fn temperature_specs(&self) -> [(&'static str, Spec); 3] {
        [
            ("t_j_max_c", self.t_j_max_c),
            ("t_shutdown_c", self.t_shutdown_c),
            ("max_ambient_c", self.max_ambient_c),
        ]
    }

    /// Копия с радиатором: R_θJA = R_θJC + `r_case_ambient_k_per_w`.
    /// `None`, если R_θJC не опубликовано или значение не положительное и конечное.
    pub fn with_heatsink(&self, r_case_ambient_k_per_w: f64) -> Option<Driver> {
        if !(r_case_ambient_k_per_w.is_finite() && r_case_ambient_k_per_w > 0.0) {
            return None;
        }
        let jc = self.r_theta_jc_k_per_w.get()?;
        let mut d = self.clone();
        d.r_theta_ja_k_per_w = Spec::derived(
            jc + r_case_ambient_k_per_w,
            "R_θJC паспорта + заданный радиатор",
        );
        Some(d)
    }

    /// Длительный ток при напряжении питания `v`.
    pub fn continuous_current_at(&self, v: f64) -> Spec {
        match self.low_voltage_current {
            Some((threshold, current)) if v < threshold => Spec {
                value: Some(current),
                ..self.current_continuous_a
            },
            _ if self
                .current_rating_min_voltage_v
                .get()
                .is_some_and(|min| v < min) =>
            {
                self.current_continuous_a.get().map_or(Spec::NONE, |i| {
                    Spec::est(i, "паспортный ток нормирован при более высоком питании")
                })
            }
            _ => self.current_continuous_a,
        }
    }

    /// Падение на ключах при токе `i`, В. `None` — данных нет.
    pub fn voltage_drop(&self, i: f64) -> Option<f64> {
        let i = i.abs();
        if let (Some(&(i0, v0)), Some(&(i1, v1))) =
            (self.drop_table.first(), self.drop_table.last())
        {
            if i <= i0 {
                return Some(v0);
            }
            if i >= i1 {
                if self.drop_table.len() < 2 {
                    return Some(v1);
                }
                let (ia, va) = self.drop_table[self.drop_table.len() - 2];
                return Some(v1 + (i - i1) * (v1 - va) / (i1 - ia));
            }
            return self
                .drop_table
                .windows(2)
                .find(|w| i >= w[0].0 && i <= w[1].0)
                .map(|w| w[0].1 + (i - w[0].0) * (w[1].1 - w[0].1) / (w[1].0 - w[0].0));
        }
        self.r_on_hot_ohm.get().map(|r| r * i)
    }

    /// Напряжение на выходе при питании `v` и токе `i`, В.
    pub fn output_voltage(&self, v: f64, i: f64) -> f64 {
        let f = self.output_fraction.get().unwrap_or(1.0);
        let terminal = v * f - self.voltage_drop(i).unwrap_or(0.0);
        if terminal.is_finite() {
            terminal.max(0.0)
        } else {
            f64::NAN
        }
    }

    /// Мощность потерь в ключах при токе `i` через один мост, Вт. `None` — данных нет.
    pub fn conduction_loss_w(&self, i: f64) -> Option<f64> {
        self.voltage_drop(i).map(|d| d * i.abs())
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
}

const BRUSHED_STEPPER: &[MotorType] = &[MotorType::BrushedDc, MotorType::BipolarStepper];
const BRUSHED: &[MotorType] = &[MotorType::BrushedDc];
const STEPPER: &[MotorType] = &[MotorType::BipolarStepper];
const BLDC_BRUSHED: &[MotorType] = &[MotorType::Bldc, MotorType::BrushedDc];
const BLDC: &[MotorType] = &[MotorType::Bldc];

const DRV8833: &str = "TI DRV8833, SLVSAR1E: https://www.ti.com/lit/ds/symlink/drv8833.pdf";
const TB6612: &str = "Toshiba TB6612FNG: https://tams-www.informatik.uni-hamburg.de/lehre/2019ss/vorlesung/es/doc/tb6612fng.pdf";
const L298: &str = "ST L298, DS0218: https://www.st.com/resource/en/datasheet/l298.pdf";
const BTS: &str = "Infineon BTS 7960, Rev. 1.1: https://www.infineon.com/assets/row/public/documents/10/57/infineon-bts7960-ds-en.pdf";
const DRV8825: &str = "TI DRV8825, SLVSA73F: https://www.ti.com/lit/ds/symlink/drv8825.pdf";
const A4988: &str =
    "Allegro A4988: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf";
const ESCON: &str = "maxon ESCON 50/5 (409510): https://www.maxongroup.com/maxon/view/product/control/4-Q-Servokontroller/409510";
const XL330: &str = "ROBOTIS XL330-M288-T: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/";
const XL430: &str = "ROBOTIS XL430-W250-T: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/";
const XM430: &str = "ROBOTIS XM430-W350-T/R: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/";
const STS: &str = "Feetech STS3215, спецификация A/0: https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf";
const AK80: &str =
    "CubeMars AK80-64 KV80: https://www.cubemars.com/product/ak80-64-kv80-robotic-actuator.html";
const AK70: &str = "CubeMars AK70-10 KV100: https://www.cubemars.com/goods-1031-AK70-10.html";
const AK60: &str = "CubeMars AK60-6 V1.1: https://www.cubemars.com/goods-1142-AK60-6+V11.html";

// ST L298, Total Drop; Toshiba TB6612FNG, верхний и нижний ключ.
static L298_DROP: [(f64, f64); 2] = [(1.0, 3.2), (2.0, 4.9)];
static TB6612_DROP: [(f64, f64); 2] = [(0.3, 0.21), (1.0, 0.7)];

/// Все драйверы каталога.
pub(crate) fn build() -> Vec<Driver> {
    vec![
        Driver {
            voltage_min_v: Spec::ds(2.7, DRV8833),
            voltage_max_v: Spec::ds(10.8, DRV8833),
            voltage_abs_max_v: Spec::ds(11.8, DRV8833),
            current_continuous_a: Spec::ds(
                1.5,
                "TI: 1.5 А RMS на мост, корпуса PWP/RTY, VM = 5 В, 25 °C",
            ),
            current_rating_min_voltage_v: Spec::ds(
                5.0,
                "TI SLVSAR1E, § 6.3, ниже 5 В допустимый ток падает",
            ),
            current_peak_a: Spec::ds(2.0, "TI: 2 А пик; порог защиты от 2 А"),
            r_on_hot_ohm: Spec::est(0.65, "TI: 350 + 300 мОм при 85 °C и 2.7 В, типовые"),
            r_theta_ja_k_per_w: Spec::ds(40.5, "TI: корпус PWP"),
            r_theta_jc_k_per_w: Spec::ds(4.8, "TI: корпус PWP, к нижней площадке"),
            t_j_max_c: Spec::ds(150.0, DRV8833),
            t_shutdown_c: Spec::ds(150.0, "TI: порог защиты минимум 150 °C"),
            quiescent_current_a: Spec::ds(0.003, DRV8833),
            sources: &[DRV8833],
            caveats: &[
                "Ниже 5 В сопротивление растёт, а длительный ток падает: числа TI не приводит.",
                "В корпусе PW длительный ток втрое меньше: см. отдельную позицию.",
            ],
            ..Driver::blank(
                "drv8833",
                "DRV8833 (PWP/RTY)",
                DriverKind::HBridge,
                "Двойной маленький H-мост для моторов до 10.8 В: N20, небольшие мотор-редукторы.",
                BRUSHED_STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(2.7, DRV8833),
            voltage_max_v: Spec::ds(10.8, DRV8833),
            voltage_abs_max_v: Spec::ds(11.8, DRV8833),
            current_continuous_a: Spec::ds(0.5, "TI: 500 мА RMS на мост, корпус PW"),
            current_rating_min_voltage_v: Spec::ds(
                5.0,
                "TI SLVSAR1E, § 6.3, ниже 5 В допустимый ток падает",
            ),
            current_peak_a: Spec::ds(2.0, "TI: 2 А пик"),
            r_on_hot_ohm: Spec::est(0.65, "TI: 350 + 300 мОм при 85 °C и 2.7 В, типовые"),
            r_theta_ja_k_per_w: Spec::ds(103.1, "TI: корпус PW"),
            r_theta_jc_k_per_w: Spec::ds(38.0, "TI: корпус PW, к верху"),
            t_j_max_c: Spec::ds(150.0, DRV8833),
            t_shutdown_c: Spec::ds(150.0, "TI: порог защиты минимум 150 °C"),
            quiescent_current_a: Spec::ds(0.003, DRV8833),
            sources: &[DRV8833],
            caveats: &["Корпус PW без теплоотводящей площадки."],
            ..Driver::blank(
                "drv8833-pw",
                "DRV8833 (PW)",
                DriverKind::HBridge,
                "Тот же DRV8833 в корпусе без площадки: держит втрое меньший длительный ток.",
                BRUSHED_STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(2.5, TB6612),
            voltage_max_v: Spec::ds(13.5, TB6612),
            voltage_abs_max_v: Spec::ds(15.0, TB6612),
            current_continuous_a: Spec::ds(1.0, "Toshiba, рабочий диапазон: 1.0 А при VM ≥ 4.5 В"),
            low_voltage_current: Some((4.5, 0.4)),
            current_peak_a: Spec::ds(2.0, "Toshiba: 2 А, импульс 20 мс, скважность ≤ 20 %"),
            peak_max_duration_s: Spec::ds(0.020, "Toshiba: 20 мс"),
            drop_table: &TB6612_DROP,
            r_theta_ja_k_per_w: Spec::derived(160.0, "(150 − 25) °C / 0.78 Вт, только корпус"),
            t_j_max_c: Spec::NONE,
            t_shutdown_c: Spec::ds(175.0, "Toshiba: TSD 175 °C (расчётное)"),
            max_ambient_c: Spec::ds(85.0, TB6612),
            quiescent_current_a: Spec::ds(0.0022, "Toshiba: ICC до 2.2 мА"),
            sources: &[TB6612],
            caveats: &[
                "Заголовок обещает 1.2 А, рабочий диапазон — 1.0 А: берём 1.0.",
                "При 2.5–4.5 В только 0.4 А и без ШИМ.",
            ],
            ..Driver::blank(
                "tb6612fng",
                "TB6612FNG",
                DriverKind::HBridge,
                "Двойной MOSFET-мост до 13.5 В: заметно холоднее и экономнее L298N на малых токах.",
                BRUSHED_STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::derived(4.8, "ST: VIH + 2.5 В, VIH минимум 2.3 В"),
            voltage_max_v: Spec::ds(46.0, L298),
            voltage_abs_max_v: Spec::ds(50.0, L298),
            current_continuous_a: Spec::ds(2.0, "ST: 2 А на канал, DC"),
            current_peak_a: Spec::ds(
                2.5,
                "ST: 2.5 А повторяющийся, 80 % вкл., 10 мс (3 А — разовый 100 мкс)",
            ),
            peak_max_duration_s: Spec::ds(0.010, "ST: ton = 10 мс"),
            drop_table: &L298_DROP,
            r_theta_ja_k_per_w: Spec::ds(35.0, "ST: Multiwatt15 без радиатора"),
            r_theta_jc_k_per_w: Spec::ds(3.0, "ST: Multiwatt15"),
            t_j_max_c: Spec::ds(130.0, "ST: рабочая температура кристалла до 130 °C"),
            quiescent_current_a: Spec::ds(0.106, "ST: IS до 70 мА + ISS до 36 мА"),
            sources: &[L298],
            caveats: &[
                "Падение до 4.9 В при 2 А: на 12 В мотор получает около 7 В.",
                "Тепловое сопротивление радиатора модуля не опубликовано: без with_heatsink считается корпус без радиатора.",
            ],
            ..Driver::blank(
                "l298n",
                "L298N",
                DriverKind::HBridge,
                "Старый биполярный мост: дешёвый, но теряет до 4.9 В и превращает их в тепло.",
                BRUSHED_STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(5.5, BTS),
            voltage_max_v: Spec::ds(27.5, BTS),
            voltage_abs_max_v: Spec::ds(45.0, BTS),
            current_continuous_a: Spec::ds(40.0, "Infineon: 40 А при температуре корпуса до 85 °C"),
            current_peak_a: Spec::ds(
                33.0,
                "Infineon: минимальный порог ограничения тока, нижний ключ при 150 °C",
            ),
            peak_max_duration_s: Spec::ds(0.010, "Infineon: импульс 10 мс"),
            r_on_hot_ohm: Spec::derived(0.0305, "Infineon: 12.5 + 18 мОм, максимум при 150 °C"),
            r_theta_ja_k_per_w: Spec::ds(35.0, "Infineon: 6 см² меди"),
            r_theta_jc_k_per_w: Spec::ds(1.0, BTS),
            t_j_max_c: Spec::ds(150.0, BTS),
            t_shutdown_c: Spec::ds(152.0, "Infineon: порог минимум 152 °C"),
            quiescent_current_a: Spec::ds(0.006, "Infineon: до 3 мА на полумост, в модуле два"),
            sources: &[BTS],
            caveats: &[
                "«43 А» — типовой порог ограничения, а не рабочий ток; без радиатора предел около 10 А.",
                "В модуле два кристалла; весь нагрев условно отнесён к одному, это запас.",
            ],
            ..Driver::blank(
                "bts7960",
                "BTS7960 (модуль из двух полумостов)",
                DriverKind::HBridge,
                "Мощный мост на десятки ампер для больших коллекторных моторов; без радиатора держит много меньше.",
                BRUSHED,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(8.2, DRV8825),
            voltage_max_v: Spec::ds(45.0, DRV8825),
            voltage_abs_max_v: Spec::ds(47.0, DRV8825),
            current_continuous_a: Spec::ds(2.5, "TI: 2.5 А при 24 В и 25 °C с теплоотводом"),
            current_peak_a: Spec::ds(2.5, "TI: полная шкала 2.5 А"),
            r_on_hot_ohm: Spec::derived(0.64, "TI: 0.32 + 0.32 Ом, максимум при 85 °C"),
            r_theta_ja_k_per_w: Spec::ds(31.6, "TI: корпус PWP"),
            r_theta_jc_k_per_w: Spec::ds(15.9, "TI: к верху корпуса"),
            t_j_max_c: Spec::ds(150.0, DRV8825),
            t_shutdown_c: Spec::ds(150.0, "TI: порог минимум 150 °C"),
            quiescent_current_a: Spec::ds(0.008, DRV8825),
            sources: &[DRV8825],
            caveats: &["2.5 А достижимы только с хорошим теплоотводом."],
            ..Driver::blank(
                "drv8825",
                "DRV8825",
                DriverKind::StepperChopper,
                "Драйвер шагового двигателя до 45 В с микрошагом 1/32.",
                STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(8.0, A4988),
            voltage_max_v: Spec::ds(35.0, A4988),
            voltage_abs_max_v: Spec::ds(35.0, A4988),
            current_peak_a: Spec::ds(2.0, "Allegro: ±2 А, абсолютный максимум"),
            r_on_hot_ohm: Spec::derived(0.86, "Allegro: 430 + 430 мОм, максимум"),
            r_theta_ja_k_per_w: Spec::ds(32.0, "Allegro: 4-слойная плата JEDEC"),
            t_j_max_c: Spec::ds(150.0, A4988),
            t_shutdown_c: Spec::ds(165.0, A4988),
            max_ambient_c: Spec::ds(85.0, A4988),
            quiescent_current_a: Spec::ds(0.012, "Allegro: IBB до 4 мА + IDD до 8 мА"),
            sources: &[A4988],
            caveats: &["R_θJA дано для 4-слойной платы; маленькие модули греются сильнее."],
            ..Driver::blank(
                "a4988",
                "A4988",
                DriverKind::StepperChopper,
                "Распространённый драйвер шагового двигателя до 35 В с микрошагом 1/16.",
                STEPPER,
            )
        },
        Driver {
            voltage_min_v: Spec::ds(10.0, ESCON),
            voltage_max_v: Spec::ds(50.0, ESCON),
            voltage_abs_max_v: Spec::ds(50.0, ESCON),
            current_continuous_a: Spec::ds(5.0, ESCON),
            current_peak_a: Spec::ds(15.0, ESCON),
            peak_max_duration_s: Spec::ds(20.0, "maxon: пик 15 А до 20 с"),
            output_fraction: Spec::ds(0.98, "maxon: выход 0.98·Vcc"),
            max_ambient_c: Spec::ds(45.0, "maxon: полный ток до 45 °C, выше снижение 0.125 А/°C"),
            rated_module: true,
            sources: &[ESCON],
            caveats: &["maxon отмечает модель как NRND: для новых разработок не рекомендована."],
            ..Driver::blank(
                "escon-50-5",
                "maxon ESCON 50/5",
                DriverKind::ServoController,
                "Сервоконтроллер maxon для BLDC и коллекторных моторов: 5 А длительно, 15 А пик.",
                BLDC_BRUSHED,
            )
        },
        integrated(
            "builtin-dynamixel-xl330",
            "Встроенный драйвер XL330",
            &[XL330],
            (3.7, 6.0),
            1.74,
            None,
        ),
        integrated(
            "builtin-dynamixel-xl430",
            "Встроенный драйвер XL430",
            &[XL430],
            (6.5, 12.0),
            1.4,
            None,
        ),
        integrated(
            "builtin-dynamixel-xm430",
            "Встроенный драйвер XM430",
            &[XM430],
            (10.0, 14.8),
            2.7,
            None,
        ),
        integrated(
            "builtin-feetech-sts3215",
            "Встроенный драйвер STS3215",
            &[STS],
            (4.0, 7.4),
            2.5,
            Some(0.5),
        ),
        Driver {
            sources: &["Параметры схемы производители хобби-серв не публикуют."],
            caveats: &["Ток и нагрев встроенной платы хобби-сервы не нормированы."],
            ..Driver::blank(
                "builtin-hobby-servo",
                "Встроенная плата хобби-сервы",
                DriverKind::Integrated,
                "Плата внутри сервы; отдельно не подключается, данных о ней нет.",
                BRUSHED,
            )
        },
        cubemars(
            "builtin-cubemars-ak80",
            "Встроенный драйвер AK80-64",
            &[AK80],
            (24.0, 48.0),
            7.0,
            19.0,
        ),
        cubemars(
            "builtin-cubemars-ak70",
            "Встроенный драйвер AK70-10",
            &[AK70],
            (24.0, 48.0),
            7.2,
            23.2,
        ),
        cubemars(
            "builtin-cubemars-ak60",
            "Встроенный драйвер AK60-6",
            &[AK60],
            (24.0, 24.0),
            6.5,
            22.7,
        ),
    ]
}

fn integrated(
    id: &'static str,
    name: &'static str,
    src: &'static [&'static str],
    range: (f64, f64),
    stall_a: f64,
    continuous_a: Option<f64>,
) -> Driver {
    Driver {
        voltage_min_v: Spec::ds(range.0, src[0]),
        voltage_max_v: Spec::ds(range.1, src[0]),
        voltage_abs_max_v: Spec::ds(range.1, src[0]),
        current_continuous_a: continuous_a.map_or(Spec::NONE, |i| Spec::ds(i, src[0])),
        current_peak_a: Spec::ds(stall_a, "стоповый ток привода при наибольшем напряжении"),
        sources: src,
        caveats: &["Падение на ключах входит в паспортный стоповый ток привода."],
        ..Driver::blank(
            id,
            name,
            DriverKind::Integrated,
            "Драйвер внутри умного сервопривода; отдельно не подключается.",
            BRUSHED,
        )
    }
}

fn cubemars(
    id: &'static str,
    name: &'static str,
    src: &'static [&'static str],
    range: (f64, f64),
    rated_a: f64,
    peak_a: f64,
) -> Driver {
    Driver {
        voltage_min_v: Spec::ds(range.0, src[0]),
        voltage_max_v: Spec::ds(range.1, src[0]),
        voltage_abs_max_v: Spec::ds(range.1, src[0]),
        current_continuous_a: Spec::ds(rated_a, src[0]),
        current_peak_a: Spec::ds(peak_a, src[0]),
        max_ambient_c: Spec::ds(50.0, src[0]),
        sources: src,
        caveats: &["Длительность пика и тепловые данные драйвера не опубликованы."],
        ..Driver::blank(
            id,
            name,
            DriverKind::Integrated,
            "Драйвер внутри привода CubeMars; отдельно не подключается.",
            BLDC,
        )
    }
}
