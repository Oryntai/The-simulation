//! Дополнительные данные проекта. Пустое поле означает отсутствие исходных данных.

use std::str::FromStr;

#[derive(Clone, Copy, Debug)]
pub struct Field {
    pub key: &'static str,
    pub title: &'static str,
    pub numeric: bool,
    pub choices: &'static [(&'static str, &'static str)],
}

macro_rules! settings {
    ($name:ident { $($key:ident : $ty:ty => $title:literal $(, [$($id:literal => $label:literal),+])? ;)* }) => {
        #[derive(Clone, Debug, Default, PartialEq)]
        pub struct $name {
            $(pub $key: Option<$ty>,)*
        }

        impl $name {
            pub const FIELDS: &'static [Field] = &[
                $(Field { key: stringify!($key), title: $title, numeric: <$ty as SettingValue>::NUMERIC, choices: &[$($(( $id, $label )),+)?] },)*
            ];

            pub fn get(&self, key: &str) -> Option<String> {
                match key {
                    $(stringify!($key) => self.$key.as_ref().map(ToString::to_string),)*
                    _ => None,
                }
            }

            pub fn set(&mut self, key: &str, value: &str) -> Result<(), String> {
                match key {
                    $(stringify!($key) => self.$key = parse(value).map_err(|_| format!("«{}»: неверное значение", $title))?,)*
                    _ => return Err(format!("Неизвестное поле «{key}»")),
                }
                Ok(())
            }

            pub fn validate_fields(&self) -> Vec<String> {
                Self::FIELDS.iter().filter_map(|f| {
                    let value = self.get(f.key)?;
                    if (!f.choices.is_empty() && !f.choices.iter().any(|(id, _)| *id == value))
                        || (f.numeric && value.parse::<f64>().is_ok_and(|x| !x.is_finite()))
                    {
                        return Some(format!("«{}»: недопустимое значение", f.title));
                    }
                    let (low, high) = physical_range(f.key).filter(|_| f.numeric)?;
                    let x = value.parse::<f64>().ok()?;
                    (!(low..=high).contains(&x)).then(|| {
                        format!("«{}»: {x} вне физически возможного диапазона {low}…{high}", f.title)
                    })
                }).collect()
            }
        }
    };
}

/// Границы, за которыми значение не описывает реальную деталь. Это не запас прочности, а защита
/// от опечатки или числа, которое делает расчёт заведомо оптимистичным (идеальная теплоотдача,
/// нулевое сопротивление, бесконечное плечо). Границы намеренно широкие.
fn physical_range(key: &str) -> Option<(f64, f64)> {
    Some(match key {
        "volume_x_mm" | "volume_y_mm" | "volume_z_mm" => (10.0, 2000.0),
        "nozzle_mm" => (0.1, 2.0),
        "strength_pct" => (1.0, 99.0),
        "height_mm" => (1000.0, 2300.0),
        "mass_kg" => (25.0, 250.0),
        "age_years" => (10.0, 100.0),
        "shoulder_plane_deg" | "shoulder_elevation_deg" | "shoulder_rotation_deg" | "elbow_deg"
        | "forearm_deg" | "trunk_deg" | "lumbar_deg" | "lateral_deg" | "lumbar_rotation_deg"
        | "start_shoulder_deg" | "start_elbow_deg" => (-180.0, 180.0),
        "hold_s" => (0.0, 86_400.0),
        "loss_s" => (0.001, 60.0),
        "lifetime_h" => (0.01, 1e6),
        "cold_c" => (-60.0, 60.0),
        "arm_lever_mm" => (0.0, 1500.0),
        "accessory_mass_kg" => (0.0, 300.0),
        "bbox_x_mm" | "bbox_y_mm" | "bbox_z_mm" => (0.1, 2000.0),
        "overhang_deg" => (0.0, 90.0),
        "bridge_mm" => (0.0, 1000.0),
        "wall_mm" => (0.05, 200.0),
        "print_hole_mm" => (0.0, 500.0),
        "sn_a_mpa" => (1.0, 3000.0),
        "sn_b" => (-1.0, -0.001),
        "sn_cycles" => (100.0, 1e10),
        "thread_length_mm" => (0.5, 200.0),
        "insert_mm" => (1.0, 50.0),
        "plate_mm" => (0.1, 200.0),
        "edge_mm" => (0.1, 1000.0),
        "bolt_lever_mm" => (1.0, 2000.0),
        "acceleration_s" => (0.001, 60.0),
        "move_s" => (0.01, 86_400.0),
        "rest_s" => (0.0, 86_400.0),
        "inertia_kg_m2" => (1e-7, 1000.0),
        "assist_nm" => (0.0, 1000.0),
        "motor_part_c" => (-40.0, 250.0),
        "resistance_spread" => (0.005, 2.0),
        "group_spread_v" => (0.001, 2.0),
        "interconnect_ohm" | "contacts_ohm" => (1e-5, 10.0),
        "wire_outer_mm" => (0.2, 40.0),
        "fuse_position_m" => (0.0, 10.0),
        "bms_scd_min_a" | "bms_scd_max_a" | "bms_ocd_min_a" | "bms_ocd_max_a" => (0.01, 5000.0),
        "bms_scd_delay_s" => (1e-6, 2.0),
        "bms_ocd_min_s" | "bms_ocd_max_s" => (1e-4, 120.0),
        "bms_ovp_v" | "bms_uvp_v" => (1.0, 6.0),
        "mosfet_v" => (1.0, 1000.0),
        "mosfet_a" => (0.1, 10_000.0),
        "mosfet_s" => (1e-7, 100.0),
        "mosfet_ohm" => (1e-5, 10.0),
        "mosfet_j" => (0.0, 1000.0),
        "enclosure_area_m2" => (1e-4, 10.0),
        "enclosure_height_mm" => (1.0, 2000.0),
        "enclosure_wall_mm" => (0.1, 100.0),
        "enclosure_k" => (0.01, 500.0),
        "enclosure_h" => (0.1, 100.0),
        "enclosure_emissivity" => (0.01, 1.0),
        "enclosure_body_fraction" | "skin_fraction" => (0.0, 1.0),
        "spacer_mm" => (0.0, 100.0),
        "spacer_k" => (0.01, 500.0),
        "skin_s" => (1.0, 86_400.0),
        "charger_a" => (0.001, 500.0),
        "charger_v" => (0.5, 1000.0),
        "charge_min_c" | "charge_max_c" => (-40.0, 100.0),
        "rail_v" => (0.5, 120.0),
        "board_avg_a" | "board_peak_a" => (0.0001, 100.0),
        "board_thermal_kw" => (0.01, 1000.0),
        "board_temp_limit_c" => (20.0, 175.0),
        "firmware_s" => (1e-6, 3600.0),
        "current_a" => (1e-6, 100.0),
        "adc_v" => (0.0, 60.0),
        "pwm_min" | "pwm_max" => (0.0, 1.0),
        _ => return None,
    })
}

trait SettingValue: FromStr + Sized {
    const NUMERIC: bool = false;

    fn parse_field(value: &str) -> Result<Self, Self::Err> {
        value.trim().parse()
    }
}

impl SettingValue for bool {}
impl SettingValue for u32 {}
impl SettingValue for f64 {
    const NUMERIC: bool = true;
}
impl SettingValue for String {
    fn parse_field(value: &str) -> Result<Self, Self::Err> {
        Ok(value.to_owned())
    }
}

fn parse<T: SettingValue>(value: &str) -> Result<Option<T>, T::Err> {
    if value.trim().is_empty() {
        Ok(None)
    } else {
        T::parse_field(value).map(Some)
    }
}

settings!(DeviceSettings {
    volume_x_mm: f64 => "Ход принтера X, мм";
    volume_y_mm: f64 => "Ход принтера Y, мм";
    volume_z_mm: f64 => "Ход принтера Z, мм";
    nozzle_mm: f64 => "Сопло, мм";
    supports: bool => "Поддержки", ["true" => "есть", "false" => "нет"];
    enclosed: bool => "Закрытая камера", ["true" => "есть", "false" => "нет"];
    sex: String => "Пол", ["female" => "женский", "male" => "мужской"];
    strength_pct: f64 => "Перцентиль силы, %";
    height_mm: f64 => "Рост, мм";
    mass_kg: f64 => "Масса человека, кг";
    age_years: f64 => "Возраст, лет";
    shoulder_plane_deg: f64 => "Плоскость подъёма плеча, °";
    shoulder_elevation_deg: f64 => "Подъём плеча, °";
    shoulder_rotation_deg: f64 => "Вращение плеча, °";
    elbow_deg: f64 => "Сгибание локтя, °";
    forearm_deg: f64 => "Вращение предплечья, °";
    trunk_deg: f64 => "Наклон корпуса, °";
    lumbar_deg: f64 => "Сгибание поясницы, °";
    lateral_deg: f64 => "Боковой наклон поясницы, °";
    lumbar_rotation_deg: f64 => "Поворот поясницы, °";
    start_shoulder_deg: f64 => "Подъём плеча в начале движения, °";
    start_elbow_deg: f64 => "Сгибание локтя в начале движения, °";
    hold_s: f64 => "Непрерывное удержание в рабочей позе, с";
    loss_s: f64 => "Время снятия помощи при отказе, с";
    cycles_day: u32 => "Нагружений за день";
    lifetime_h: f64 => "Часов под нагрузкой за весь срок";
    cold_c: f64 => "Наименьшая температура воздуха, °C";
    arm_lever_mm: f64 => "Верхняя граница плеча приведённого груза от кисти, мм";
    arm_equivalence_source: String => "Источник приведения масс и моментов к модели руки";
    accessory_mass_kg: f64 => "Масса батареи, платы, корпуса и прочих деталей, кг";
    inventory_source: String => "Источник полной ведомости масс и расположения деталей";
    contact_source: String => "Проверка доступных зон по всему ходу и контакта креплений";
});

settings!(LinkSettings {
    bbox_x_mm: f64 => "Габарит в положении печати X, мм";
    bbox_y_mm: f64 => "Габарит в положении печати Y, мм";
    bbox_z_mm: f64 => "Габарит в положении печати Z, мм";
    overhang_deg: f64 => "Наибольшее нависание от вертикали, °";
    bridge_mm: f64 => "Наибольший мост, мм (0 — нет)";
    wall_mm: f64 => "Минимальная стенка, мм";
    print_hole_mm: f64 => "Минимальное отверстие, мм (0 — нет)";
    sn_a_mpa: f64 => "Кривая усталости σ = a·Nᵇ: a, МПа";
    sn_b: f64 => "Кривая усталости: показатель b";
    sn_cycles: f64 => "Кривая усталости: предел испытанных циклов";
    sn_source: String => "Протокол S–N для этого материала и режима печати";
    bolt_size: String => "Болт", ["none" => "без болтов", "M2" => "M2", "M2.5" => "M2.5", "M3" => "M3", "M4" => "M4", "M5" => "M5", "M6" => "M6", "M8" => "M8"];
    bolt_grade: String => "Класс болта", ["8.8" => "8.8", "10.9" => "10.9", "12.9" => "12.9", "A2-70" => "A2-70", "A4-80" => "A4-80"];
    bolt_count: u32 => "Число болтов";
    thread: String => "Резьба", ["through" => "сквозная с гайкой", "tapped" => "в детали", "insert" => "вставка"];
    thread_length_mm: f64 => "Глубина зацепления резьбы, мм";
    insert_mm: f64 => "Наружный диаметр вставки, мм";
    plate_mm: f64 => "Толщина детали под болтом, мм";
    edge_mm: f64 => "От центра болта до края, мм";
    bolt_lever_mm: f64 => "Минимальное плечо реакции болтовой группы, мм";
    hole_axis: String => "Ось отверстия", ["wide" => "вдоль широкой стороны сечения", "narrow" => "вдоль узкой стороны сечения"];
    hole_centered: bool => "Отверстие по центру", ["true" => "по центру", "false" => "со смещением"];
    acceleration_s: f64 => "Самый быстрый разгон до предельной скорости, с";
    move_s: f64 => "Движение под нагрузкой за цикл, с";
    rest_s: f64 => "Неподвижная часть цикла под нагрузкой, с (0 — нет)";
    inertia_kg_m2: f64 => "Верхняя граница приведённой инерции привода, кг·м²";
    inertia_source: String => "Источник верхней границы инерции";
    limits_source: String => "Протокол независимой проверки пределов скорости, момента и разгона";
    arm_action: String => "Действие привода на руку", ["elbow" => "сгибание локтя", "shoulder" => "подъём плеча"];
    assist_nm: f64 => "Подтверждённая помощь человеку, Н·м";
    motor_part_c: f64 => "Температура этой детали у мотора (замер или расчёт), °C";
    driver_id: String => "Драйвер выбранного шарнира";
    motor_gpio: u32 => "Вывод платы управления этим приводом";
    motor_regulated: bool => "Силовое питание привода", ["true" => "от стабилизатора", "false" => "от батареи"];
});

settings!(ElectricalSettings {
    resistance_spread: f64 => "Разброс DCIR ячеек, доля";
    group_spread_v: f64 => "Разница напряжений групп, В";
    interconnect_ohm: f64 => "Сопротивление перемычек, Ом";
    contacts_ohm: f64 => "Сопротивление контактов, Ом";
    wire_kind: String => "Задание медной жилы", ["mm2" => "сечение в мм²", "awg" => "калибр AWG"];
    awg: u32 => "Калибр медной жилы AWG";
    wire_outer_mm: f64 => "Наружный диаметр провода, мм";
    insulation: String => "Изоляция", ["pvc" => "ПВХ", "silicone" => "силикон", "ptfe" => "ПТФЭ"];
    cooling: String => "Укладка", ["air" => "свободный воздух", "bundled" => "жгут", "enclosed" => "корпус"];
    fuse_position_m: f64 => "От клеммы до предохранителя, м";
    bms_name: String => "Модель BMS и источник параметров";
    bms_scd_min_a: f64 => "BMS: нижний порог КЗ, А";
    bms_scd_max_a: f64 => "BMS: верхний порог КЗ, А";
    bms_scd_delay_s: f64 => "BMS: наибольшая задержка КЗ, с";
    bms_ocd_min_a: f64 => "BMS: нижний порог перегрузки, А";
    bms_ocd_max_a: f64 => "BMS: верхний порог перегрузки, А";
    bms_ocd_min_s: f64 => "BMS: наименьшая задержка перегрузки, с";
    bms_ocd_max_s: f64 => "BMS: наибольшая задержка перегрузки, с";
    bms_ovp_v: f64 => "BMS: защита от перезаряда группы, В";
    bms_uvp_v: f64 => "BMS: защита от переразряда группы, В";
    mosfet_v: f64 => "Ключ BMS: предел сток–исток, В";
    mosfet_a: f64 => "Ключ BMS: импульсный ток, А";
    mosfet_s: f64 => "Ключ BMS: допустимая длительность импульса, с";
    mosfet_ohm: f64 => "Ключ BMS: сопротивление, Ом";
    mosfet_j: f64 => "Ключ BMS: предел лавинной энергии, Дж";
    enclosure_area_m2: f64 => "Наружная площадь корпуса батареи, м²";
    enclosure_height_mm: f64 => "Высота корпуса батареи, мм";
    enclosure_wall_mm: f64 => "Толщина корпуса батареи, мм";
    enclosure_k: f64 => "Теплопроводность корпуса, Вт/(м·К)";
    enclosure_h: f64 => "Внутренняя теплоотдача, Вт/(м²·К)";
    enclosure_emissivity: f64 => "Коэффициент излучения корпуса";
    enclosure_body_fraction: f64 => "Доля корпуса, закрытая телом";
    spacer_mm: f64 => "Толщина прокладки у кожи, мм";
    spacer_k: f64 => "Теплопроводность прокладки, Вт/(м·К)";
    skin_s: f64 => "Время контакта с кожей, с";
    skin_class: String => "Поверхность у кожи", ["metal" => "металл", "plastic" => "пластик", "wood" => "дерево", "ceramic" => "керамика", "coated" => "металл с покрытием"];
    skin_fraction: f64 => "Доля поверхности тела под батареей";
    charger_a: f64 => "Наибольший ток зарядки, А";
    charger_v: f64 => "Наибольшее напряжение зарядки, В";
    charge_min_c: f64 => "Наименьшая температура заряда, °C";
    charge_max_c: f64 => "Наибольшая температура заряда, °C";
    rail_v: f64 => "Настройка выхода стабилизатора, В";
    board_regulated: bool => "Источник питания платы", ["true" => "от стабилизатора", "false" => "от батареи"];
    board_avg_a: f64 => "Верхняя граница среднего тока платы, А";
    board_peak_a: f64 => "Верхняя граница пикового тока платы, А";
    board_thermal_kw: f64 => "Тепловое сопротивление платы, К/Вт";
    board_temp_limit_c: f64 => "Допустимая температура указанной точки платы, °C";
    board_source: String => "Источник измерений тока и тепловых данных платы";
    sequencing_source: String => "Протокол уровней цифровых входов при пуске, сбросе и отключении";
    sensors_declared: bool => "Состав датчиков проверен", ["true" => "да, список полный", "false" => "нет"];
    firmware_s: f64 => "Интервал наблюдения прошивки, с";
});

settings!(SensorSettings {
    id: String => "Идентификатор датчика";
    count: u32 => "Количество";
    on_regulator: bool => "Подключение питания", ["true" => "через стабилизатор", "false" => "к батарее"];
    current_a: f64 => "Подтверждённый верхний ток одного датчика с обвязкой, А";
});

settings!(FirmwareSettings {
    boot: bool => "Условия включения платы", ["true" => "да, входы при включении", "false" => "отдельный сценарий"];
    input_pin: u32 => "Вход GPIO";
    input_high: bool => "Уровень на входе GPIO", ["true" => "высокий", "false" => "низкий"];
    adc: u32 => "Канал АЦП";
    adc_v: f64 => "Напряжение на АЦП, В";
    output_pin: u32 => "Выход GPIO или ШИМ";
    output_high: bool => "Ожидаемый уровень GPIO", ["true" => "высокий", "false" => "низкий"];
    pwm_min: f64 => "Допустимый минимум заполнения ШИМ";
    pwm_max: f64 => "Допустимый максимум заполнения ШИМ";
    reason: String => "Какое требование проверяется";
});
