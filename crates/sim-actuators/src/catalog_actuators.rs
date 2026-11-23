//! Каталог приводов. Каждое число — с источником; чего нет в документах, то `Spec::NONE`.
//! Где данные расходятся, в каталоге стоит безопасное значение, а расхождение показано в
//! [`crate::datasheet_discrepancies`].
//!
//! Числа maxon EC 90 flat + Harmonic Drive CSG-25-50 и CubeMars AK80-64 согласованы с
//! `crates/sim-em/src/motor.rs` чтением. Два осознанных отличия:
//! * КПД волновой передачи здесь 0.70 (нижний край диапазона 70–85 % из того же файла),
//!   в `sim-em` — 0.85: для вердикта берём опасную сторону;
//! * длительный момент maxon считается через K_t·(I_N − I₀), а не через напечатанные
//!   964 мН·м, которые из K_t и I_N не следуют.

use crate::actuator::{Actuator, ActuatorKind, Backdrive, VoltagePoint};
use crate::spec::{KGF_CM, RPM, Spec};

/// Скорость холостого хода в об/мин из паспортного «секунд на 60°».
const fn s60(seconds_per_60deg: f64) -> f64 {
    10.0 / seconds_per_60deg
}

const MAXON: &str = "maxon EC 90 flat 500267: https://www.maxongroup.com/maxon/view/product/motor/ecmotor/ecflat/ecflat90/500267";
const HD: &str = "Harmonic Drive CSG-25-50-2A-GR: https://www.harmonicdrive.net/products/component-sets/cup-type/csg-2a/csg-25-50-2a-gr";
const AK80: &str =
    "CubeMars AK80-64 KV80: https://www.cubemars.com/product/ak80-64-kv80-robotic-actuator.html";
const AK70: &str = "CubeMars AK70-10 KV100: https://www.cubemars.com/goods-1031-AK70-10.html";
const AK60: &str = "CubeMars AK60-6 V1.1: https://www.cubemars.com/goods-1142-AK60-6+V11.html";
const XL330: &str = "ROBOTIS XL330-M288-T: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/";
const XL430: &str = "ROBOTIS XL430-W250-T: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/";
const XM430: &str = "ROBOTIS XM430-W350-T/R: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/";
const STS: &str = "Feetech STS3215, спецификация A/0: https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf";
const STS2: &str = "Feetech STS3215, описание 200620: https://www.mantech.co.za/datasheets/products/sts3215-200620a-1.pdf";
const SG90: &str = "листок SG90: https://dac70r.github.io/ELEC3300/Labs/SG90.pdf";
const SG90_TOWER: &str = "TowerPro SG90 Analog: https://towerpro.com.tw/product/sg90-analog/";
const SG90_LOW: &str = "копия листка TowerPro с 1.2 кгс·см при 4.8 В: https://bit.nkust.edu.tw/~csshieh/teach/111A/mc/datasheet/SG90.pdf";
const SG90_M: &str =
    "Protosupplies, замер SG90: https://protosupplies.com/product/servo-motor-micro-sg90/";
const SG90_I: &str = "форум Arduino, стоповый ток 650 ± 80 мА: https://forum.arduino.cc/t/how-much-power-supply-do-i-need-for-controlling-5-sg90-9g-microservo/627666";
const MG90: &str = "AZ-Delivery MG90S: https://cdn.shopify.com/s/files/1/1509/1638/files/MG90S_Micro_Servo_Datenblatt_AZ-Delivery_Vertriebs_GmbH.pdf";
const MG90_M: &str =
    "Protosupplies, MG90S: https://protosupplies.com/product/servo-motor-micro-mg90s/";
const MG996: &str = "листок MG996R Tower Pro: https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/kmv48_rp625/kmv48_rp625/kmv48_rp625/MG996RTowerPro.pdf";
const MG996_TOWER: &str = "TowerPro MG996R: https://towerpro.com.tw/product/mg996r/";
const DS3218: &str = "DIYmall DS3218 (18 кгс·см при 5 В): https://manuals.plus/asin/B07FMBBPD2";
const DS3218_2: &str =
    "DSSERVO DS3218, листинг: https://rcdrone.top/products/dsservo-ds3218-digital-servo";
const DS3218_DS: &str =
    "DSSERVO, паспорт DS3218: https://www.dsservo.com/d_file/DS3218%20datasheet.pdf";
const DS3218_I: &str =
    "листинг DS3218 MG, 2.6 А: https://www.zennixtek.com/products/dsservo-ds3218-mg-servo-motor";
const N20_100: &str = "Pololu #2214, 100:1 HP 6V: https://www.pololu.com/product/2214";
const N20_CAT: &str = "Pololu, таблица Micro Metal Gearmotors: https://www.pololu.com/category/60/micro-metal-gearmotors";
const N20_298: &str = "Pololu #2218: https://www.pololu.com/product/2218/specs";
const JGA: &str = "Open Impulse JGA25-371 126 об/мин: https://people.ece.ubc.ca/~eng-services/files/courses/elec391-data_sheets/MOT4-info.pdf";
const NEMA: &str = "STEPPERONLINE 17HS19-2004S1, чертёж A0842: https://static.maritex.eu/file/display/5sXxpHH1SP-JwnhJEZ0lhfX-xdYKZRi3/17HS19-2004S1_Full_Datasheet.pdf";
const NEMA_PRODUCT: &str = "STEPPERONLINE, 17HS19-2004S1: https://www.omc-stepperonline.com/nema-17-bipolar-59ncm-84oz-in-2a-42x48mm-4-wires-w-1m-cable-connector-17hs19-2004s1";
const NEMA_CURVE: &str = "STEPPERONLINE, график pull-out при 24 В, 2 А, полушаг: https://www.omc-stepperonline.com/download/17HS19-2004S1_Torque_Curve.pdf";

// Таблицы ROBOTIS e-Manual, раздел технических характеристик указанных моделей.
static XL330_POINTS: [VoltagePoint; 3] = [
    VoltagePoint {
        voltage_v: 3.7,
        stall_torque_nm: 0.42,
        no_load_rpm: 76.0,
        stall_current_a: Some(1.11),
    },
    VoltagePoint {
        voltage_v: 5.0,
        stall_torque_nm: 0.52,
        no_load_rpm: 103.0,
        stall_current_a: Some(1.47),
    },
    VoltagePoint {
        voltage_v: 6.0,
        stall_torque_nm: 0.60,
        no_load_rpm: 123.0,
        stall_current_a: Some(1.74),
    },
];
static XL430_POINTS: [VoltagePoint; 3] = [
    VoltagePoint {
        voltage_v: 9.0,
        stall_torque_nm: 1.0,
        no_load_rpm: 47.0,
        stall_current_a: Some(1.0),
    },
    VoltagePoint {
        voltage_v: 11.1,
        stall_torque_nm: 1.4,
        no_load_rpm: 57.0,
        stall_current_a: Some(1.3),
    },
    VoltagePoint {
        voltage_v: 12.0,
        stall_torque_nm: 1.5,
        no_load_rpm: 61.0,
        stall_current_a: Some(1.4),
    },
];
static XM430_POINTS: [VoltagePoint; 3] = [
    VoltagePoint {
        voltage_v: 11.1,
        stall_torque_nm: 3.8,
        no_load_rpm: 43.0,
        stall_current_a: Some(2.1),
    },
    VoltagePoint {
        voltage_v: 12.0,
        stall_torque_nm: 4.1,
        no_load_rpm: 46.0,
        stall_current_a: Some(2.3),
    },
    VoltagePoint {
        voltage_v: 14.8,
        stall_torque_nm: 4.8,
        no_load_rpm: 57.0,
        stall_current_a: Some(2.7),
    },
];
// Feetech STS, таблица 5 электрических характеристик.
static STS_POINTS: [VoltagePoint; 2] = [
    VoltagePoint {
        voltage_v: 6.0,
        stall_torque_nm: 16.5 * KGF_CM,
        no_load_rpm: s60(0.238),
        stall_current_a: Some(2.0),
    },
    VoltagePoint {
        voltage_v: 7.4,
        stall_torque_nm: 19.5 * KGF_CM,
        no_load_rpm: s60(0.192),
        stall_current_a: Some(2.5),
    },
];
// SG90_LOW, SG90_M и SG90_I: нижний момент, медленная скорость, верхний ток.
static SG90_POINTS: [VoltagePoint; 1] = [VoltagePoint {
    voltage_v: 4.8,
    stall_torque_nm: 1.2 * KGF_CM,
    no_load_rpm: s60(0.12),
    stall_current_a: Some(0.73),
}];
// AZ-Delivery и Protosupplies. Вторая точка тока и скорости — оценка
// пропорционально напряжению, для вердикта подтверждением не служит.
static MG90_POINTS: [VoltagePoint; 2] = [
    VoltagePoint {
        voltage_v: 4.8,
        stall_torque_nm: 1.8 * KGF_CM,
        no_load_rpm: s60(0.12),
        stall_current_a: Some(0.7),
    },
    VoltagePoint {
        voltage_v: 6.0,
        stall_torque_nm: 2.2 * KGF_CM,
        no_load_rpm: s60(0.096),
        stall_current_a: Some(0.875),
    },
];
// TowerPro: момент и скорость. 2.5 А — верхняя оценка по листку Cornell;
// 2.0 А — оценка при 4.8 В по пропорции напряжений.
static MG996_POINTS: [VoltagePoint; 2] = [
    VoltagePoint {
        voltage_v: 4.8,
        stall_torque_nm: 9.4 * KGF_CM,
        no_load_rpm: s60(0.19),
        stall_current_a: Some(2.0),
    },
    VoltagePoint {
        voltage_v: 6.0,
        stall_torque_nm: 11.0 * KGF_CM,
        no_load_rpm: s60(0.15),
        stall_current_a: Some(2.5),
    },
];
// DSSERVO, таблица электрических характеристик.
static DS3218_POINTS: [VoltagePoint; 2] = [
    VoltagePoint {
        voltage_v: 5.0,
        stall_torque_nm: 18.0 * KGF_CM,
        no_load_rpm: s60(0.16),
        stall_current_a: Some(1.8),
    },
    VoltagePoint {
        voltage_v: 6.8,
        stall_torque_nm: 21.5 * KGF_CM,
        no_load_rpm: s60(0.14),
        stall_current_a: Some(2.2),
    },
];
// Pololu #2214 и #2218: типовая скорость и экстраполяция стоповых значений.
static N20_100_POINTS: [VoltagePoint; 1] = [VoltagePoint {
    voltage_v: 6.0,
    stall_torque_nm: 1.7 * KGF_CM,
    no_load_rpm: 310.0,
    stall_current_a: Some(1.6),
}];
static N20_298_POINTS: [VoltagePoint; 1] = [VoltagePoint {
    voltage_v: 6.0,
    stall_torque_nm: 4.0 * KGF_CM,
    no_load_rpm: 100.0,
    stall_current_a: Some(1.6),
}];
// Open Impulse, листок конкретного JGA25-371 1:34.
static JGA_POINTS: [VoltagePoint; 1] = [VoltagePoint {
    voltage_v: 12.0,
    stall_torque_nm: 4.2 * KGF_CM,
    no_load_rpm: 126.0,
    stall_current_a: Some(1.0),
}];

/// Все приводы каталога.
pub(crate) fn build() -> Vec<Actuator> {
    vec![
        maxon_ec90_hd(),
        ak80_64(),
        ak70_10(),
        ak60_6(),
        xl330(),
        xl430(),
        xm430(),
        sts3215(),
        sg90(),
        mg90s(),
        mg996r(),
        ds3218(),
        n20_100(),
        n20_298(),
        jga25(),
        nema17(),
    ]
}

fn maxon_ec90_hd() -> Actuator {
    const ETA: f64 = 0.70;
    const KT: f64 = 0.231;
    const IN: f64 = 4.06;
    const I0: f64 = 0.278;
    const N: f64 = 50.0;
    Actuator {
        voltage_nominal_v: Spec::ds(48.0, MAXON),
        voltage_max_v: Spec::ds(
            48.0,
            "номинальное напряжение maxon 500267; выше паспорт не разрешает",
        ),
        rated_torque_nm: Spec::est(
            KT * (IN - I0) * N * ETA,
            "K_t·(I_N − I₀)·N·η = 0.231·(4.06 − 0.278)·50·0.70; ниже номинала редуктора 51 Н·м",
        ),
        peak_torque_nm: Spec::ds(127.0, "Harmonic Drive CSG-25-50: повторяющийся пик 127 Н·м"),
        no_load_speed_rpm: Spec::derived(1960.0 / N, "maxon: 1960 об/мин / 50"),
        rated_speed_rpm: Spec::derived(1670.0 / N, "maxon: номинальная скорость 1670 об/мин / 50"),
        no_load_current_a: Spec::ds(I0, MAXON),
        stall_current_a: Spec::ds(56.9, MAXON),
        rated_current_a: Spec::ds(IN, MAXON),
        peak_current_a: Spec::est(
            15.0,
            "предел тока в sim-em и пик ESCON 50/5; maxon пиковый ток двигателя не нормирует",
        ),
        resistance_ohm: Spec::ds(0.844, MAXON),
        inductance_h: Spec::ds(1.07e-3, MAXON),
        kt_motor_nm_per_a: Spec::ds(KT, MAXON),
        kv_rpm_per_v: Spec::ds(41.3, MAXON),
        gear_ratio: Spec::ds(N, HD),
        gear_efficiency: Spec::est(
            ETA,
            "нижний край 70–85 % для волновых передач (в sim-em принято 0.85)",
        ),
        rotor_inertia_kg_m2: Spec::ds(5060e-7, MAXON),
        gear_input_inertia_kg_m2: Spec::est(
            0.413e-4,
            "каталог CSF/CSG, типоразмер 25; для i = 50 не подтверждено (как в sim-em)",
        ),
        mass_kg: Spec::derived(0.985 + 0.42, "maxon 985 г + Harmonic Drive 0.42 кг"),
        r_th_winding_housing_k_per_w: Spec::ds(1.82, MAXON),
        r_th_housing_ambient_k_per_w: Spec::ds(1.78, MAXON),
        tau_winding_s: Spec::ds(54.3, MAXON),
        tau_housing_s: Spec::ds(265.0, MAXON),
        max_winding_temp_c: Spec::ds(125.0, MAXON),
        min_ambient_c: Spec::ds(-40.0, MAXON),
        max_ambient_c: Spec::ds(100.0, MAXON),
        backdrive: Backdrive::Reversible {
            torque_nm: Spec::ds(8.3, HD),
        },
        sources: &[
            MAXON,
            HD,
            "maxon EC flat, каталог 2022: https://maxonjapan.com/wp-content/uploads/2022/10/EC_flat.pdf",
        ],
        caveats: &[
            "Стоповый момент: 9.41 Н·м на текущей странице maxon, 13.1 Н·м в каталоге 2022.",
            "КПД редуктора — оценка, взят нижний край диапазона.",
            "Нужен внешний BLDC-драйвер, например ESCON 50/5.",
        ],
        ..Actuator::blank(
            "maxon-ec90-hd-csg25-50",
            "maxon EC 90 flat 260 Вт 48 В + Harmonic Drive CSG-25-50",
            ActuatorKind::GearedBldc,
            "Мощный плоский бесколлекторный мотор с волновым редуктором 50:1: сустав экзоскелета, десятки Н·м без люфта.",
        )
    }
}

fn ak80_64() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(48.0, AK80),
        voltage_min_v: Spec::ds(24.0, "CubeMars: рабочие напряжения 24/48 В"),
        voltage_max_v: Spec::ds(48.0, "CubeMars: рабочие напряжения 24/48 В"),
        rated_torque_nm: Spec::ds(48.0, AK80),
        peak_torque_nm: Spec::ds(120.0, AK80),
        no_load_speed_rpm: Spec::ds(75.0, AK80),
        rated_speed_rpm: Spec::ds(48.0, "CubeMars: 23/48 об/мин при 24/48 В"),
        rated_current_a: Spec::ds(7.0, AK80),
        peak_current_a: Spec::ds(19.0, AK80),
        resistance_ohm: Spec::ds(0.22, AK80),
        inductance_h: Spec::ds(133.5e-6, AK80),
        kt_motor_nm_per_a: Spec::ds(0.136, AK80),
        ke_motor_v_s: Spec::ds(13.7 / (1000.0 * RPM), "CubeMars: K_e = 13.7 В/1000 об/мин"),
        kv_rpm_per_v: Spec::ds(80.0, AK80),
        gear_ratio: Spec::ds(64.0, AK80),
        gear_efficiency: Spec::est(0.85, "две планетарные ступени по ~0.92, как в sim-em"),
        rotor_inertia_kg_m2: Spec::ds(564.5e-7, AK80),
        mass_kg: Spec::ds(0.85, AK80),
        min_ambient_c: Spec::ds(-20.0, AK80),
        max_ambient_c: Spec::ds(50.0, AK80),
        backdrive: Backdrive::Reversible {
            torque_nm: Spec::ds(4.7, AK80),
        },
        builtin_driver: Some("builtin-cubemars-ak80"),
        sources: &[AK80],
        caveats: &[
            "Тепловые сопротивления и постоянные времени производитель не публикует. В sim-em для них есть оценки.",
            "Скорость 75 об/мин при 48 В не следует из K_t и K_v; расчёт идёт по меньшей скорости.",
        ],
        ..Actuator::blank(
            "cubemars-ak80-64",
            "CubeMars AK80-64 KV80",
            ActuatorKind::Qdd,
            "Готовый сустав: мотор, планетарный редуктор 64:1 и драйвер в одном корпусе, 48 Н·м длительно.",
        )
    }
}

fn ak70_10() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(48.0, AK70),
        voltage_min_v: Spec::ds(24.0, "CubeMars: рабочие напряжения 24/48 В"),
        voltage_max_v: Spec::ds(48.0, "CubeMars: рабочие напряжения 24/48 В"),
        rated_torque_nm: Spec::ds(8.3, AK70),
        peak_torque_nm: Spec::ds(24.8, AK70),
        no_load_speed_rpm: Spec::ds(480.0, AK70),
        rated_speed_rpm: Spec::ds(310.0, "CubeMars: 148/310 об/мин при 24/48 В"),
        rated_current_a: Spec::ds(7.2, AK70),
        peak_current_a: Spec::ds(23.2, AK70),
        resistance_ohm: Spec::ds(0.272, AK70),
        inductance_h: Spec::ds(113e-6, AK70),
        kt_motor_nm_per_a: Spec::ds(0.123, AK70),
        ke_motor_v_s: Spec::ds(11.2 / (1000.0 * RPM), "CubeMars: K_e = 11.2 В/1000 об/мин"),
        kv_rpm_per_v: Spec::ds(100.0, AK70),
        gear_ratio: Spec::ds(10.0, AK70),
        rotor_inertia_kg_m2: Spec::ds(753.4788e-7, AK70),
        mass_kg: Spec::ds(0.621, AK70),
        min_ambient_c: Spec::ds(-20.0, AK70),
        max_ambient_c: Spec::ds(50.0, AK70),
        backdrive: Backdrive::Reversible {
            torque_nm: Spec::ds(0.48, AK70),
        },
        builtin_driver: Some("builtin-cubemars-ak70"),
        sources: &[AK70],
        caveats: &[
            "Тепловое сопротивление и предел обмотки не опубликованы.",
            "Удельный момент 47.6 Н·м/кг на странице не следует из 24.8 Н·м и 621 г.",
        ],
        ..Actuator::blank(
            "cubemars-ak70-10",
            "CubeMars AK70-10 KV100",
            ActuatorKind::Qdd,
            "Лёгкий квазипрямой привод 10:1: легко проворачивается рукой, подходит для податливых суставов.",
        )
    }
}

fn ak60_6() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(24.0, AK60),
        voltage_max_v: Spec::ds(24.0, "CubeMars: номинальное напряжение 24 В"),
        rated_torque_nm: Spec::ds(3.0, AK60),
        peak_torque_nm: Spec::ds(9.0, AK60),
        no_load_speed_rpm: Spec::ds(560.0, "CubeMars, таблица сравнения, KV140"),
        rated_speed_rpm: Spec::ds(420.0, AK60),
        rated_current_a: Spec::ds(6.5, AK60),
        peak_current_a: Spec::ds(22.7, AK60),
        resistance_ohm: Spec::ds(0.202, AK60),
        inductance_h: Spec::ds(138e-6, AK60),
        kt_motor_nm_per_a: Spec::ds(0.078, AK60),
        ke_motor_v_s: Spec::ds(7.5 / (1000.0 * RPM), "CubeMars: K_e = 7.5 В/1000 об/мин"),
        kv_rpm_per_v: Spec::ds(140.0, AK60),
        gear_ratio: Spec::ds(6.0, AK60),
        rotor_inertia_kg_m2: Spec::ds(243.5e-7, AK60),
        mass_kg: Spec::ds(
            0.368,
            "CubeMars: 368 г в таблице (в тексте ~358 г, берём большее)",
        ),
        min_ambient_c: Spec::ds(-20.0, AK60),
        max_ambient_c: Spec::ds(50.0, AK60),
        backdrive: Backdrive::Reversible {
            torque_nm: Spec::ds(0.2, AK60),
        },
        builtin_driver: Some("builtin-cubemars-ak60"),
        sources: &[AK60],
        caveats: &[
            "Страница смешивает KV80 и KV140: 320 об/мин относится к KV80 (24·80/6).",
            "Тепловое сопротивление и предел обмотки не опубликованы.",
        ],
        ..Actuator::blank(
            "cubemars-ak60-6",
            "CubeMars AK60-6 V1.1 KV140",
            ActuatorKind::Qdd,
            "Компактный быстрый привод 6:1 на 24 В для лёгких звеньев; груз без питания не держит.",
        )
    }
}

fn xl330() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(5.0, XL330),
        voltage_min_v: Spec::ds(3.7, XL330),
        voltage_max_v: Spec::ds(6.0, XL330),
        peak_torque_nm: Spec::ds(0.60, "ROBOTIS: стоповый момент при 6 В, мгновенный предел"),
        no_load_speed_rpm: Spec::ds(103.0, XL330),
        stall_current_a: Spec::ds(1.47, XL330),
        standby_current_a: Spec::ds(0.017, XL330),
        gear_ratio: Spec::ds(288.4, XL330),
        mass_kg: Spec::ds(0.018, XL330),
        min_ambient_c: Spec::ds(-5.0, XL330),
        max_ambient_c: Spec::ds(70.0, XL330),
        voltage_points: &XL330_POINTS,
        builtin_driver: Some("builtin-dynamixel-xl330"),
        sources: &[XL330],
        caveats: &["Стоповый момент — мгновенный; длительный момент ROBOTIS не публикует."],
        ..Actuator::blank(
            "dynamixel-xl330-m288",
            "Dynamixel XL330-M288-T",
            ActuatorKind::SmartServo,
            "Крошечный умный сервопривод 18 г с датчиком положения и шиной: пальцы, лёгкие механизмы.",
        )
    }
}

fn xl430() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(11.1, XL430),
        voltage_min_v: Spec::ds(6.5, XL430),
        voltage_max_v: Spec::ds(12.0, XL430),
        peak_torque_nm: Spec::ds(1.5, "ROBOTIS: стоповый момент при 12 В, мгновенный предел"),
        no_load_speed_rpm: Spec::ds(57.0, XL430),
        stall_current_a: Spec::ds(1.3, XL430),
        standby_current_a: Spec::ds(0.052, XL430),
        gear_ratio: Spec::ds(258.5, XL430),
        mass_kg: Spec::ds(0.0572, XL430),
        min_ambient_c: Spec::ds(-5.0, XL430),
        max_ambient_c: Spec::ds(72.0, XL430),
        voltage_points: &XL430_POINTS,
        builtin_driver: Some("builtin-dynamixel-xl430"),
        sources: &[XL430],
        caveats: &[
            "ROBOTIS: реальный максимум на графике N-T ниже стопового момента; числового графика нет.",
            "Заряженная батарея 3S даёт 12.6 В — выше предела 12 В.",
        ],
        ..Actuator::blank(
            "dynamixel-xl430-w250",
            "Dynamixel XL430-W250-T",
            ActuatorKind::SmartServo,
            "Умный сервопривод среднего размера на 11.1 В: манипуляторы, настольные роботы.",
        )
    }
}

fn xm430() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(12.0, XM430),
        voltage_min_v: Spec::ds(10.0, XM430),
        voltage_max_v: Spec::ds(14.8, XM430),
        peak_torque_nm: Spec::ds(
            4.8,
            "ROBOTIS: стоповый момент при 14.8 В, мгновенный предел",
        ),
        no_load_speed_rpm: Spec::ds(46.0, XM430),
        stall_current_a: Spec::ds(2.3, XM430),
        standby_current_a: Spec::ds(0.040, XM430),
        gear_ratio: Spec::ds(353.5, XM430),
        mass_kg: Spec::ds(0.082, XM430),
        min_ambient_c: Spec::ds(-5.0, XM430),
        max_ambient_c: Spec::ds(80.0, XM430),
        voltage_points: &XM430_POINTS,
        builtin_driver: Some("builtin-dynamixel-xm430"),
        sources: &[XM430],
        caveats: &[
            "ROBOTIS: реальный максимум на графике N-T ниже стопового момента; числового графика нет.",
        ],
        ..Actuator::blank(
            "dynamixel-xm430-w350",
            "Dynamixel XM430-W350-T",
            ActuatorKind::SmartServo,
            "Сильный умный сервопривод на 12 В с управлением по току: кисти и лёгкие руки роботов.",
        )
    }
}

fn sts3215() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(7.4, STS),
        voltage_min_v: Spec::ds(4.0, STS),
        voltage_max_v: Spec::ds(7.4, STS),
        rated_torque_nm: Spec::ds(
            4.0 * KGF_CM,
            "Feetech: 4 кгс·см при 6 В (при 7.4 В — 5, берём меньшее)",
        ),
        peak_torque_nm: Spec::ds(19.5 * KGF_CM, STS),
        no_load_speed_rpm: Spec::ds(s60(0.192), STS),
        no_load_current_a: Spec::ds(0.15, STS),
        stall_current_a: Spec::ds(2.5, STS),
        rated_current_a: Spec::ds(
            0.5,
            "Feetech: 500 мА при 6 В (при 7.4 В — 650 мА, берём меньшее)",
        ),
        standby_current_a: Spec::ds(0.006, STS),
        resistance_ohm: Spec::ds(2.5, STS),
        gear_ratio: Spec::ds(345.0, STS2),
        mass_kg: Spec::ds(0.055, STS),
        min_ambient_c: Spec::ds(-20.0, STS),
        max_ambient_c: Spec::ds(60.0, STS),
        voltage_points: &STS_POINTS,
        builtin_driver: Some("builtin-feetech-sts3215"),
        sources: &[STS, STS2],
        caveats: &[
            "Сопротивление 2.5 Ом не сходится со стоповым током: 7.4 В / 2.5 Ом = 2.96 А против 2.5 А.",
            "Есть варианты C044, C046 и 12 В с другими числами: это данные версии 7.4 В, 1:345.",
        ],
        ..Actuator::blank(
            "feetech-sts3215",
            "Feetech STS3215 7.4 В 19 кгс·см",
            ActuatorKind::SmartServo,
            "Недорогой шинный сервопривод с магнитным энкодером: учебные манипуляторы.",
        )
    }
}

fn sg90() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::vendor(4.8, SG90),
        voltage_min_v: Spec::vendor(4.8, SG90),
        voltage_max_v: Spec::vendor(4.8, SG90_TOWER),
        peak_torque_nm: Spec::vendor(1.2 * KGF_CM, SG90_LOW),
        no_load_speed_rpm: Spec::vendor(s60(0.12), SG90_M),
        stall_current_a: Spec::vendor(0.73, SG90_I),
        standby_current_a: Spec::vendor(0.010, SG90_M),
        mass_kg: Spec::vendor(0.009, SG90),
        min_ambient_c: Spec::vendor(0.0, SG90_TOWER),
        max_ambient_c: Spec::vendor(55.0, SG90_TOWER),
        voltage_points: &SG90_POINTS,
        builtin_driver: Some("builtin-hobby-servo"),
        sources: &[SG90_TOWER, SG90, SG90_LOW, SG90_M, SG90_I],
        caveats: &[
            "TowerPro указывает 1.8 кгс·см при 4.8 В, в другом листке 1.2 кгс·см: берём 1.2.",
            "Для напряжения берём 4.8 В с сайта TowerPro, хотя продавцы указывают до 6 В.",
            "Партии заметно различаются, шестерни пластиковые.",
            "Стоповый ток: от 360 мА (замер Protosupplies) до 650 ± 80 мА; для питания берём 730 мА.",
        ],
        ..Actuator::blank(
            "sg90",
            "SG90 (9 г)",
            ActuatorKind::HobbyServo,
            "Самая дешёвая пластиковая микросерва: флажки и лёгкие заслонки, нагрузку руки не держит.",
        )
    }
}

fn mg90s() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::vendor(4.8, MG90),
        voltage_min_v: Spec::vendor(4.8, MG90),
        voltage_max_v: Spec::vendor(6.0, MG90),
        peak_torque_nm: Spec::vendor(2.2 * KGF_CM, MG90),
        no_load_speed_rpm: Spec::vendor(s60(0.12), MG90),
        stall_current_a: Spec::vendor(0.7, MG90_M),
        standby_current_a: Spec::vendor(0.010, MG90),
        mass_kg: Spec::vendor(0.0134, MG90),
        voltage_points: &MG90_POINTS,
        builtin_driver: Some("builtin-hobby-servo"),
        sources: &[MG90, MG90_M],
        caveats: &[
            "Листок сам себе противоречит: 0.1 с/60° в таблице и 0.12 с/60° в тексте при 4.8 В; берём 0.12.",
            "Скорость при 6 В пересчитана от 0.12 с/60° пропорционально напряжению (0.096 с/60°).",
            "Стоповый ток при 6 В — пересчёт 700 мА пропорционально напряжению.",
        ],
        ..Actuator::blank(
            "mg90s",
            "MG90S (металл, 13 г)",
            ActuatorKind::HobbyServo,
            "Микросерва с металлическими шестернями: прочнее SG90, момент всё ещё около 2 кгс·см.",
        )
    }
}

fn mg996r() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::vendor(6.0, MG996),
        voltage_min_v: Spec::vendor(4.8, MG996),
        voltage_max_v: Spec::vendor(6.6, MG996_TOWER),
        peak_torque_nm: Spec::vendor(11.0 * KGF_CM, MG996),
        no_load_speed_rpm: Spec::vendor(s60(0.15), MG996_TOWER),
        no_load_current_a: Spec::vendor(0.17, MG996_TOWER),
        stall_current_a: Spec::vendor(2.5, MG996),
        standby_current_a: Spec::vendor(0.010, MG996_TOWER),
        mass_kg: Spec::vendor(0.055, MG996),
        min_ambient_c: Spec::vendor(0.0, MG996_TOWER),
        max_ambient_c: Spec::vendor(55.0, MG996_TOWER),
        voltage_points: &MG996_POINTS,
        builtin_driver: Some("builtin-hobby-servo"),
        sources: &[MG996_TOWER, MG996],
        caveats: &[
            "Стоповый ток в листингах от 1.2 до 2.5 А: для питания берём 2.5 А.",
            "Скорость берём по более медленному листингу (0.19 и 0.15 с/60°).",
            "Листок указывает 7.2 В, сайт TowerPro — 6.6 В. Выше 6 В прирост момента не засчитывается.",
        ],
        ..Actuator::blank(
            "mg996r",
            "MG996R",
            ActuatorKind::HobbyServo,
            "Дешёвая стандартная серва с металлическими шестернями: около 10 кгс·см на бумаге, большой люфт.",
        )
    }
}

fn ds3218() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::vendor(6.8, DS3218),
        voltage_min_v: Spec::ds(4.8, DS3218_DS),
        voltage_max_v: Spec::ds(6.8, DS3218_DS),
        peak_torque_nm: Spec::vendor(21.5 * KGF_CM, DS3218),
        no_load_speed_rpm: Spec::vendor(s60(0.14), DS3218),
        stall_current_a: Spec::ds(2.2, DS3218_DS),
        peak_current_a: Spec::vendor(2.6, DS3218_I),
        standby_current_a: Spec::ds(0.005, DS3218_DS),
        gear_ratio: Spec::ds(275.0, DS3218_DS),
        min_ambient_c: Spec::ds(-15.0, DS3218_DS),
        max_ambient_c: Spec::ds(70.0, DS3218_DS),
        mass_kg: Spec::vendor(0.068, "листинг DSSERVO: 68 г (другие 60 г, берём большее)"),
        voltage_points: &DS3218_POINTS,
        builtin_driver: Some("builtin-hobby-servo"),
        sources: &[DS3218_DS, DS3218, DS3218_2, DS3218_I],
        caveats: &[
            "При 6.8 В паспорт даёт 2.2 А в стопе, листинг — 2.6 А. Для сводки питания считаем больший ток.",
            "При 5 В источники дают 18 и 19 кгс·см: берём 18.",
        ],
        ..Actuator::blank(
            "ds3218",
            "DS3218 20 кг",
            ActuatorKind::HobbyServo,
            "Цифровая серва в стандартном корпусе с металлическим редуктором и управлением по ШИМ.",
        )
    }
}

fn n20_100() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(6.0, N20_100),
        voltage_max_v: Spec::ds(6.0, "Pololu: номинал 6 В"),
        peak_torque_nm: Spec::ds(1.7 * KGF_CM, "Pololu: экстраполированный стоповый момент"),
        no_load_speed_rpm: Spec::ds(310.0, N20_100),
        no_load_current_a: Spec::ds(0.10, N20_100),
        stall_current_a: Spec::ds(1.6, N20_100),
        rated_current_a: Spec::derived(0.4, "Pololu: держать ток не выше 25 % стопового"),
        gear_ratio: Spec::ds(100.37, N20_100),
        resistance_ohm: Spec::derived(
            6.0 / 1.6,
            "Pololu #2214: 6 В / 1.6 А, эквивалент по экстраполированному стоповому току",
        ),
        mass_kg: Spec::ds(0.0095, N20_100),
        voltage_points: &N20_100_POINTS,
        sources: &[N20_100, N20_CAT],
        caveats: &[
            "Стоповые значения Pololu — экстраполяция: длительный упор может повредить мотор.",
            "Безымянные N20 с маркетплейсов этим данным не соответствуют.",
        ],
        ..Actuator::blank(
            "n20-100-6v-hp",
            "N20 100:1 HP 6 В (Pololu)",
            ActuatorKind::BrushedGearmotor,
            "Маленький коллекторный мотор-редуктор размером с ноготь: колёса и лёгкие механизмы.",
        )
    }
}

fn n20_298() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::ds(6.0, N20_CAT),
        voltage_max_v: Spec::ds(6.0, "Pololu: номинал 6 В"),
        peak_torque_nm: Spec::ds(
            2.0 * KGF_CM,
            "Pololu: предел мгновенного момента редуктора 2 кгс·см (ниже стоповых 4 кгс·см)",
        ),
        no_load_speed_rpm: Spec::ds(100.0, N20_CAT),
        no_load_current_a: Spec::ds(0.10, N20_CAT),
        stall_current_a: Spec::ds(1.6, N20_CAT),
        rated_current_a: Spec::derived(0.4, "Pololu: держать ток не выше 25 % стопового"),
        gear_ratio: Spec::ds(297.92, N20_298),
        resistance_ohm: Spec::derived(
            6.0 / 1.6,
            "Pololu #2218: 6 В / 1.6 А, эквивалент по экстраполированному стоповому току",
        ),
        mass_kg: Spec::ds(0.0095, N20_298),
        voltage_points: &N20_298_POINTS,
        sources: &[N20_298, N20_CAT],
        caveats: &[
            "Мотор может развить 4 кгс·см, но редуктор допускает только 2: предел — редуктор.",
        ],
        ..Actuator::blank(
            "n20-298-6v-hp",
            "N20 298:1 HP 6 В (Pololu)",
            ActuatorKind::BrushedGearmotor,
            "Медленный вариант N20: мотор способен сломать собственный редуктор, предел — 2 кгс·см.",
        )
    }
}

fn jga25() -> Actuator {
    Actuator {
        voltage_nominal_v: Spec::vendor(12.0, JGA),
        voltage_min_v: Spec::vendor(6.0, JGA),
        voltage_max_v: Spec::vendor(
            12.0,
            "продавец пишет 6–24 В, но все данные — при 12 В; выше не разрешаем",
        ),
        peak_torque_nm: Spec::vendor(4.2 * KGF_CM, JGA),
        no_load_speed_rpm: Spec::vendor(126.0, JGA),
        no_load_current_a: Spec::vendor(0.046, JGA),
        stall_current_a: Spec::vendor(1.0, JGA),
        gear_ratio: Spec::vendor(34.0, JGA),
        mass_kg: Spec::vendor(0.099, JGA),
        voltage_points: &JGA_POINTS,
        sources: &[JGA],
        caveats: &[
            "Единого паспорта JGA25 нет: числа зависят от передаточного числа и продавца.",
            "Данные версии с энкодером JGA25-371 1:34; другие варианты в каталог не переносить.",
        ],
        ..Actuator::blank(
            "jga25-371-34-12v",
            "JGA25-371 1:34 12 В",
            ActuatorKind::BrushedGearmotor,
            "Недорогой металлический мотор-редуктор 25 мм с энкодером: тележки и простые приводы.",
        )
    }
}

fn nema17() -> Actuator {
    const R: f64 = 1.6;
    const I: f64 = 2.0;
    Actuator {
        voltage_nominal_v: Spec::derived(
            I * R,
            "I·R = 2 А · 1.6 Ом; рабочее напряжение задаёт драйвер",
        ),
        // 59 Н·см — удержание на месте. Длительный момент в движении не задан.
        peak_torque_nm: Spec::ds(0.59, NEMA),
        rated_current_a: Spec::ds(I, NEMA),
        resistance_ohm: Spec::ds(
            R,
            "чертёж A0842: 1.60 Ом ± 10 % (в руководстве 1.4 Ом, берём большее для нагрева)",
        ),
        inductance_h: Spec::ds(3.0e-3, NEMA),
        gear_ratio: Spec::ds(1.0, "прямой вал"),
        rotor_inertia_kg_m2: Spec::ds(82e-7, NEMA),
        mass_kg: Spec::ds(0.390, NEMA_PRODUCT),
        r_th_total_k_per_w: Spec::derived(
            80.0 / (2.0 * I * I * R * (1.0 + 0.00393 * 80.0)),
            "перегрев не более 80 К при 2 А в двух фазах, R при +80 К",
        ),
        max_winding_temp_c: Spec::ds(130.0, "чертёж A0842: класс изоляции B, 130 °C"),
        max_ambient_c: Spec::ds(50.0, NEMA),
        backdrive: Backdrive::Reversible {
            torque_nm: Spec::NONE,
        },
        heated_phases: 2.0,
        stepper_pole_pairs: Some(50.0),
        sources: &[NEMA, NEMA_PRODUCT, NEMA_CURVE],
        caveats: &[
            "График pull-out относится к 24 В, 2 А и полушагу. Без заданной настройки драйвера положительный вывод по скорости недоступен.",
            "Момент фиксации без питания не указан.",
            "Без питания вал держит только небольшой момент фиксации.",
        ],
        ..Actuator::blank(
            "nema17-17hs19-2004s1",
            "NEMA 17 17HS19-2004S1",
            ActuatorKind::Stepper,
            "Шаговый двигатель 42 мм: точно шагает без датчика, но при перегрузке молча теряет шаги.",
        )
    }
}
