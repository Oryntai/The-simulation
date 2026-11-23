//! Готовые схемы для редактора нетлистов приложения. У каждой — описание по-русски.
//!
//! Параметры моделей приборов, помеченные «оценка», подобраны под типичные характеристики
//! класса приборов (прямое падение при рабочем токе), а не взяты из даташита конкретной детали.
//! Модель 1N4148 (IS = 2.52 нА, N = 1.752, RS = 0.568 Ом) — широко распространённая в
//! библиотеках SPICE; её точность — ±несколько десятков мВ в области 1–100 мА.

/// Пример схемы.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Example {
    pub id: &'static str,
    pub title: &'static str,
    pub description: &'static str,
    pub netlist: &'static str,
}

pub const RC_LOWPASS: Example = Example {
    id: "rc_lowpass",
    title: "RC-фильтр: отклик на ступеньку",
    description: "Ступенька 5 В на входе RC-фильтра нижних частот. Постоянная времени τ = R·C = \
                  1 мс: за τ выход доходит до 63%, за 5τ — до 99.3% входа.",
    netlist: "RC-фильтр нижних частот: отклик на ступеньку
V1 in 0 PULSE(0 5 0.5m 1u 1u 10m 20m)
R1 in out 1k
C1 out 0 1u
.tran 10u 6m
.end
",
};

pub const RLC_SERIES: Example = Example {
    id: "rlc_series",
    title: "Последовательный RLC: затухающие колебания",
    description: "Ступенька 1 В на последовательный RLC-контур. Собственная частота \
                  f₀ = 1/(2π√LC) ≈ 5.03 кГц, затухание α = R/2L = 5000 1/с — колебательный \
                  режим (α < ω₀): напряжение на конденсаторе звенит и выходит на 1 В.",
    netlist: "Последовательный RLC: затухающие колебания
V1 in 0 PULSE(0 1 0.1m 1u 1u 10m 20m)
R1 in a 10
L1 a b 1m
C1 b 0 1u
.tran 1u 2m
.end
",
};

pub const DIODE_RESISTOR: Example = Example {
    id: "diode_resistor",
    title: "Диод и резистор",
    description: "Кремниевый диод 1N4148 через 1 кОм от 5 В. Рабочая точка: ток ≈ 4.35 мА, \
                  падение на диоде ≈ 0.65 В. Точное решение — через функцию Ламберта W \
                  (см. сверку circuit.diode_lambert_w).",
    netlist: "Диод и резистор
V1 in 0 DC 5
R1 in a 1k
D1 a 0 D1N4148
.model D1N4148 D(IS=2.52n N=1.752 RS=0.568)
.op
.end
",
};

pub const HALF_WAVE_RECTIFIER: Example = Example {
    id: "half_wave_rectifier",
    title: "Однополупериодный выпрямитель с конденсатором",
    description: "Синус 17 В амплитуды (12 В действующих), 50 Гц, через диод на конденсатор \
                  470 мкФ с нагрузкой 220 Ом. Конденсатор заряжается на пиках и разряжается \
                  нагрузкой: пульсации ≈ I/(f·C) ≈ 3 В. Модель диода — оценка для класса 1N4007.",
    netlist: "Однополупериодный выпрямитель со сглаживающим конденсатором
V1 ac 0 SIN(0 17 50)
D1 ac out DRECT
C1 out 0 470u
Rl out 0 220
.model DRECT D(IS=14n N=1.98 RS=0.03) ; оценка: выпрямительный диод класса 1N4007
.tran 50u 100m
.end
",
};

pub const LED_RESISTOR: Example = Example {
    id: "led_resistor",
    title: "Светодиод с резистором",
    description: "Красный светодиод от 5 В через 150 Ом: ток ≈ 20 мА, падение ≈ 2 В. \
                  Резистор считают как (Vпит − Vсд)/Iсд. Модель светодиода — оценка.",
    netlist: "Светодиод с ограничительным резистором
V1 vcc 0 DC 5
R1 vcc a 150
D1 a 0 LEDRED
.model LEDRED D(IS=1e-22 N=1.6 RS=2) ; оценка: красный LED, около 2 В при 20 мА
.op
.end
",
};

pub const MOTOR_LOWSIDE_PWM: Example = Example {
    id: "motor_lowside_pwm",
    title: "Мотор: ключ N-MOSFET в нижнем плече, ШИМ 20 кГц",
    description: "Щёточный мотор (R = 1 Ом, L = 1 мГн, противо-ЭДС Vbemf задаёт механика через \
                  EXT) включён между +12 В и стоком N-MOSFET; ШИМ 20 кГц, скважность 0.6. \
                  Обратный диод Шоттки D1 замыкает ток мотора, пока ключ закрыт, — без него \
                  индуктивность дала бы выброс напряжения на стоке. Модели ключа и диода — оценка.",
    netlist: "Мотор постоянного тока, ключ в нижнем плече, ШИМ 20 кГц
Vbat vbat 0 DC 12
Vpwm pwm 0 PULSE(0 10 0 100n 100n 29.8u 50u)
Rg pwm g 10
M1 d g 0 0 NPWR W=1 L=1
.model NPWR NMOS(VTO=2 KP=6.25) ; оценка: Rds(on) около 20 мОм при Vgs = 10 В
Rm vbat m1 1
Lm m1 m2 1m
Vbemf m2 d EXT 4
D1 d vbat DSCH
.model DSCH D(IS=5u N=1.05 RS=0.02) ; оценка: диод Шоттки, около 0.45 В при 3 А
.tran 1u 10m
.end
",
};

pub const HBRIDGE_PWM: Example = Example {
    id: "hbridge_pwm",
    title: "H-мост с ШИМ и мёртвым временем",
    description: "Мотор на H-мосту из четырёх ключей. Левое плечо (S1 сверху, S2 снизу) \
                  переключается в противофазе с паузой 1 мкс; правый нижний S4 открыт \
                  постоянно — режим «знак–модуль», вращение вперёд, скважность ≈ 0.58. \
                  В мёртвое время ток мотора идёт через диоды D1–D4 (встроенные диоды ключей).",
    netlist: "H-мост с ШИМ 20 кГц и мёртвым временем
Vbus vbus 0 DC 12
Vh gh 0 PULSE(0 1 1u 10n 10n 28.98u 50u)
Vl gl 0 PULSE(1 0 0 10n 10n 30.98u 50u)
V3 g3 0 DC 0
V4 g4 0 DC 1
S1 vbus a gh 0 SWM
S2 a 0 gl 0 SWM
S3 vbus b g3 0 SWM
S4 b 0 g4 0 SWM
.model SWM SW(RON=10m ROFF=1meg VT=0.5)
D1 a vbus DB
D2 0 a DB
D3 b vbus DB
D4 0 b DB
.model DB D(IS=1e-12 N=1.3 RS=0.01) ; оценка: встроенный диод MOSFET
Rm a m1 1
Lm m1 m2 1m
Vbemf m2 b EXT 4
.tran 0.5u 10m
.end
",
};

pub const HBRIDGE_EXT: Example = Example {
    id: "hbridge_ext",
    title: "H-мост для прошивки (ключи из кода)",
    description: "Тот же H-мост, но ключи S1–S4 переключает прошивка/хост вызовами \
                  set_switch, а противо-ЭДС Vbemf задаёт механика (set_source). Мёртвое время \
                  — забота прошивки: если открыть S1 и S2 одновременно, будет сквозной ток.",
    netlist: "H-мост: ключи управляются из кода
Vbus vbus 0 DC 12
S1 vbus a EXT RON=10m ROFF=1meg
S2 a 0 EXT RON=10m ROFF=1meg
S3 vbus b EXT RON=10m ROFF=1meg
S4 b 0 EXT RON=10m ROFF=1meg
D1 a vbus DB
D2 0 a DB
D3 b vbus DB
D4 0 b DB
.model DB D(IS=1e-12 N=1.3 RS=0.01) ; оценка: встроенный диод MOSFET
Rm a m1 1
Lm m1 m2 1m
Vbemf m2 b EXT 0
.tran 1u 10m
.end
",
};

pub const BUCK_SYNC: Example = Example {
    id: "buck_sync",
    title: "Синхронный понижающий преобразователь 24 → 5 В",
    description: "Buck 200 кГц: верхний ключ S1 открыт ≈ 1.04 мкс из 5 мкс (D ≈ 5/24), нижний \
                  S2 — в противофазе с паузами 10–20 нс, в паузы ток дросселя идёт через диод D2. \
                  Vout ≈ D·Vin = 5 В, размах тока дросселя ≈ (Vin−Vout)·D/(L·f) ≈ 0.42 А. \
                  Плавного пуска нет — на старте виден выброс LC-фильтра.",
    netlist: "Синхронный понижающий преобразователь 24 -> 5 В, 200 кГц
Vin vin 0 DC 24
Vg1 g1 0 PULSE(0 1 20n 10n 10n 1.0317u 5u)
Vg2 g2 0 PULSE(1 0 0 10n 10n 1.0717u 5u)
S1 vin sw g1 0 SWM
S2 sw 0 g2 0 SWM
.model SWM SW(RON=10m ROFF=1meg VT=0.5)
D2 0 sw DB
.model DB D(IS=1e-12 N=1.3 RS=0.01) ; оценка: встроенный диод нижнего ключа
L1 sw out 47u
C1 out 0 47u
Rl out 0 5
.tran 20n 2m
.end
",
};

pub const BATTERY_DIVIDER_ADC: Example = Example {
    id: "battery_divider_adc",
    title: "Делитель напряжения батареи для АЦП",
    description: "Батарея 3S Li-ion (до 12.6 В) через делитель 100k/27k на вход АЦП 3.3 В: \
                  12.6 В → 2.68 В. Сопротивление источника для АЦП R1‖R2 ≈ 21 кОм больше \
                  допустимого для выборки 1 мкс при 12 битах (≈ 19.6 кОм для Cвыб = 5 пФ, \
                  см. adc::max_source_resistance), поэтому у входа стоит C1 = 100 нФ: он отдаёт \
                  заряд выборки. Ключ выборки Ssmp — EXT (управляет прошивка), Csh — ёмкость \
                  выборки АЦП.",
    netlist: "Делитель напряжения батареи 3S для АЦП 3.3 В
Vbat bat 0 DC 12.6
R1 bat adc 100k
R2 adc 0 27k
C1 adc 0 100n
Ssmp adc smp EXT RON=1k ROFF=1g
Csh smp 0 5p
.tran 1u 20m
.end
",
};

pub const NTC_DIVIDER: Example = Example {
    id: "ntc_divider",
    title: "Делитель с NTC-термистором",
    description: "Термистор 10 кОм (B25/85 = 3977 K, как Vishay NTCLE100E3103) в нижнем плече \
                  делителя от 3.3 В с верхним резистором 10 кОм: при 25 °C на выходе 1.65 В. \
                  Нетлист для другой температуры даёт examples::ntc_divider(t), сопротивление — \
                  examples::ntc_resistance(t).",
    netlist: "Делитель с NTC-термистором (25 °C)
Vref vref 0 DC 3.3
Rtop vref out 10k
Rntc out 0 10k
C1 out 0 100n
.op
.end
",
};

/// Все примеры в порядке показа.
pub fn all() -> Vec<Example> {
    vec![
        RC_LOWPASS,
        RLC_SERIES,
        DIODE_RESISTOR,
        HALF_WAVE_RECTIFIER,
        LED_RESISTOR,
        MOTOR_LOWSIDE_PWM,
        HBRIDGE_PWM,
        HBRIDGE_EXT,
        BUCK_SYNC,
        BATTERY_DIVIDER_ADC,
        NTC_DIVIDER,
    ]
}

/// Пример по идентификатору.
pub fn by_id(id: &str) -> Option<Example> {
    all().into_iter().find(|e| e.id == id)
}

/// Сопротивление NTC по B-уравнению: R(T) = R25·exp(B·(1/T − 1/T25)), Ом.
/// Параметры по умолчанию — Vishay NTCLE100E3103 (R25 = 10 кОм, B25/85 = 3977 K ±0.75%),
/// даташит <https://www.vishay.com/docs/29049/ntcle100.pdf>. B-уравнение — двухточечная
/// аппроксимация: вне 25…85 °C её ошибка растёт до ~1 °C (точнее — уравнение Стейнхарта–Харта).
pub fn ntc_resistance(temp_c: f64) -> f64 {
    ntc_resistance_with(10e3, 3977.0, temp_c)
}

pub fn ntc_resistance_with(r25: f64, beta: f64, temp_c: f64) -> f64 {
    let t = temp_c + 273.15;
    r25 * (beta * (1.0 / t - 1.0 / 298.15)).exp()
}

/// Нетлист делителя с NTC при заданной температуре.
pub fn ntc_divider(temp_c: f64) -> String {
    format!(
        "Делитель с NTC-термистором ({temp_c:.1} °C)\n\
         Vref vref 0 DC 3.3\n\
         Rtop vref out 10k\n\
         Rntc out 0 {:.6e}\n\
         C1 out 0 100n\n\
         .op\n\
         .end\n",
        ntc_resistance(temp_c)
    )
}
