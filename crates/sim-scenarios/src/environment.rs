//! Сценарий «Среда: холод, жара, вода, ветер» — человек в экзоскелете на улице/в воде.
//!
//! * Человек: двухузловая модель Gagge (ASHRAE 55, `sim-env`) — ядро, кожа, пот, дрожь,
//!   потеря воды за время пребывания; в воде — эмпирика погружения (Hayward 1975, хронология
//!   «холодовой шок → отказ мышц → переохлаждение»).
//! * Погода: ветро-холодовой индекс и время до обморожения, индекс жары, шкала Бофорта.
//! * Устройство: температура рамы (воздух/вода/солнце), риск при касании кожей (ISO 13732-1/3),
//!   конденсат при входе в тепло, пределы Li-ion и электроники, охлаждение мотора ветром или
//!   водой → сколько момента привод даёт длительно при этой погоде.

use sim_core::{Pose, Shape, Vec3};
use sim_em::motor::{MotorParams, derated_current_maxon};
use sim_env::air::condensation_check;
use sim_env::contact::{Material, RiskLevel, contact_risk};
use sim_env::convection::{Medium, h_cylinder, h_radiation};
use sim_env::effects::{
    ELECTRONICS_GRADES, HandFunction, LI_ION, MATERIAL_LIMITS, SURFACES, hand_function,
    muscle_power_factor, solar_device_temperature,
};
use sim_env::immersion::immersion_timeline;
use sim_env::indices::{beaufort, heat_index_c, wind_chill};
use sim_env::strain::{CoreStatus, HydrationStatus, core_status, hydration_status};
use sim_env::two_node::{TwoNodeInputs, TwoNodeOutput, TwoNodeWarning, two_node, two_node_rk4};
use sim_mujoco::{GeomInfo, Sim};

use crate::anatomy::{Which, bone_geoms, bone_visuals, open_model};
use crate::scenario::*;

const FRAMES: [(Material, &str); 6] = [
    (Material::Aluminium6061, "алюминий 6061"),
    (Material::StainlessSteel304, "нержавеющая сталь"),
    (Material::Titanium6Al4V, "титан"),
    (Material::CarbonFibreComposite, "углепластик"),
    (Material::Pa12, "полиамид PA12 (печать)"),
    (Material::Pla, "PLA (печать)"),
];
const LINERS: [(Option<Material>, &str); 4] = [
    (None, "без подкладки (рама касается кожи)"),
    (Some(Material::NeopreneFoam), "вспененный неопрен"),
    (Some(Material::SiliconeRubber), "силикон"),
    (Some(Material::Tpu), "TPU"),
];
const MEDIA: [&str; 3] = [
    "воздух",
    "в пресной воде по грудь",
    "в морской воде по грудь",
];
/// Масса и рост «среднего человека» модели, кг и м.
const BODY_MASS: f64 = 75.0;
const BODY_HEIGHT: f64 = 1.75;
/// Корпус мотора EC 90 flat: диаметр и длина, м (габарит по каталогу maxon, округлённо).
const MOTOR_D: f64 = 0.09;
const MOTOR_L: f64 = 0.035;

pub struct Environment {
    sim: Option<Sim>,
    bones: Vec<GeomInfo>,
    device: Vec3,
    torso: Vec3,
    // погода и человек
    t_air: f64,
    rh: f64,
    wind: f64,
    sun: f64,
    t_water: f64,
    medium: usize,
    exposure_min: f64,
    clo: f64,
    met: f64,
    // устройство
    frame: usize,
    liner: usize,
    coating: usize,
    // результаты
    human: Option<TwoNodeOutput>,
    human_rk4_core: f64,
    epoch: u64,
    t: f64,
}

impl Default for Environment {
    fn default() -> Self {
        let sim = open_model(Which::FullBody).ok();
        let bones = sim.as_ref().map(bone_geoms).unwrap_or_default();
        let (device, torso) = sim
            .as_ref()
            .map(|s| {
                let body = |n: &str| s.bodies().iter().position(|(b, _)| b == n);
                let dev = body("radius_r")
                    .map(|b| s.body_com(b))
                    .unwrap_or([0.0, -0.2, 0.9]);
                let tor = body("lumbar1")
                    .map(|b| s.body_com(b))
                    .unwrap_or([0.0, 0.0, 1.1]);
                (
                    Vec3::new(dev[0] + 0.04, dev[1] - 0.03, dev[2]),
                    Vec3::new(tor[0], tor[1], tor[2] + 0.15),
                )
            })
            .unwrap_or((Vec3::new(0.0, -0.2, 0.9), Vec3::new(0.0, 0.0, 1.25)));
        let mut s = Self {
            sim,
            bones,
            device,
            torso,
            t_air: -15.0,
            rh: 70.0,
            wind: 8.0,
            sun: 0.0,
            t_water: 12.0,
            medium: 0,
            exposure_min: 120.0,
            clo: 1.5,
            met: 2.0,
            frame: 0,
            liner: 0,
            coating: 0,
            human: None,
            human_rk4_core: f64::NAN,
            epoch: 0,
            t: 0.0,
        };
        s.recompute();
        s
    }
}

impl Environment {
    fn in_water(&self) -> bool {
        self.medium > 0
    }

    /// Ветер у тела: на высоте ~1.5 м скорость ≈ 2/3 скорости на 10 м (логарифмический
    /// профиль; оценка).
    fn wind_body(&self) -> f64 {
        self.wind * 2.0 / 3.0
    }

    fn recompute(&mut self) {
        self.epoch += 1;
        if self.in_water() {
            self.human = None;
            return;
        }
        let inp = TwoNodeInputs::ashrae55(
            self.t_air,
            self.t_air,
            self.wind_body().max(0.1),
            self.rh,
            self.met,
            self.clo,
        )
        .for_person(BODY_MASS, BODY_HEIGHT)
        .with_exposure(self.exposure_min, 1.0);
        let out = two_node(&inp);
        self.human_rk4_core = two_node_rk4(&inp, 0.1).0;
        self.human = Some(out);
    }

    /// Температура рамы устройства, °C: среда, плюс солнце на воздухе.
    fn device_temp(&self) -> f64 {
        if self.in_water() {
            self.t_water
        } else if self.sun > 0.0 {
            solar_device_temperature(
                &SURFACES[self.coating].0,
                self.sun,
                self.t_air,
                self.wind_body(),
                0.1,
            )
        } else {
            self.t_air
        }
    }

    fn fluid_temp(&self) -> f64 {
        if self.in_water() {
            self.t_water
        } else {
            self.t_air
        }
    }

    fn conv_medium(&self) -> Medium {
        match self.medium {
            1 => Medium::FreshWater,
            2 => Medium::SeaWater {
                salinity_g_kg: 35.0,
            },
            _ => Medium::air(self.rh),
        }
    }

    /// Тепловое сопротивление «корпус мотора → среда», К/Вт: конвекция (ветер/вода) +
    /// излучение (на воздухе, ε = 0.9 — оценка для анодированного корпуса).
    fn motor_r_ha(&self, t_surface: f64) -> f64 {
        let area = std::f64::consts::PI * MOTOR_D * MOTOR_L
            + 2.0 * std::f64::consts::PI * (MOTOR_D / 2.0).powi(2);
        let speed = if self.in_water() {
            0.2
        } else {
            self.wind_body()
        };
        let mut h = h_cylinder(
            &self.conv_medium(),
            MOTOR_D,
            speed,
            t_surface,
            self.fluid_temp(),
        );
        if !self.in_water() {
            h += h_radiation(0.9, t_surface, self.t_air);
        }
        1.0 / (h * area)
    }

    fn motor_continuous_torque(&self) -> (f64, f64, f64) {
        let p = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
        let r = self.motor_r_ha(60.0);
        let i = p.continuous_current_a(self.fluid_temp(), 0.0, r);
        let tpa = p.torque_constant_nm_per_a * p.gear.ratio * p.gear.efficiency_forward;
        (i, i * tpa, r)
    }

    fn skin_color(&self) -> [f32; 4] {
        let t = self
            .human
            .as_ref()
            .map(|h| h.t_skin_c)
            .unwrap_or(if self.in_water() {
                self.t_water + 2.0
            } else {
                33.0
            });
        // 20 °C и ниже — синий, 33 — телесный, 37 и выше — красный.
        let (c0, c1, k) = if t < 33.0 {
            (
                [0.35, 0.55, 0.95],
                [0.93, 0.76, 0.64],
                ((t - 20.0) / 13.0).clamp(0.0, 1.0),
            )
        } else {
            (
                [0.93, 0.76, 0.64],
                [0.95, 0.3, 0.25],
                ((t - 33.0) / 4.0).clamp(0.0, 1.0),
            )
        };
        let m = |a: f32, b: f32| a + (b - a) * k as f32;
        [m(c0[0], c1[0]), m(c0[1], c1[1]), m(c0[2], c1[2]), 0.28]
    }

    fn temp_color(t: f64) -> [f32; 4] {
        if t < 0.0 {
            [0.3, 0.55, 1.0, 1.0]
        } else if t < 15.0 {
            [0.5, 0.75, 0.95, 1.0]
        } else if t < 43.0 {
            C_EXO
        } else if t < 60.0 {
            [1.0, 0.6, 0.2, 1.0]
        } else {
            [1.0, 0.2, 0.15, 1.0]
        }
    }
}

fn risk_text(l: RiskLevel) -> &'static str {
    match l {
        RiskLevel::Safe => "безопасно",
        RiskLevel::HeatPain => "горячо, больно",
        RiskLevel::ColdPain => "холодовая боль",
        RiskLevel::Numbness => "онемение",
        RiskLevel::FrostnipRisk => "риск обморожения",
        RiskLevel::BurnRisk => "риск ожога",
    }
}

fn risk_level(l: RiskLevel) -> Level {
    match l {
        RiskLevel::Safe => Level::Good,
        RiskLevel::HeatPain | RiskLevel::ColdPain => Level::Warn,
        _ => Level::Danger,
    }
}

impl Scenario for Environment {
    fn id(&self) -> &'static str {
        "environment"
    }

    fn title(&self) -> String {
        "Среда: холод, жара, вода, ветер".into()
    }

    fn summary(&self) -> String {
        "Человек в экзоскелете на морозе, в жару, на ветру и в воде: терморегуляция (модель \
         Gagge, ASHRAE 55), обморожение и перегрев, касание рамы кожей, солнце на корпусе, \
         конденсат, батарея и электроника, охлаждение мотора — сколько момента привод даёт \
         длительно в этих условиях."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        let p = |id, label: &str, value, min, max, unit, decimals| Param {
            id,
            label: label.to_string(),
            value,
            min,
            max,
            unit,
            decimals,
        };
        let mut v = vec![
            p(
                "t_air",
                "Температура воздуха",
                self.t_air,
                -40.0,
                50.0,
                "°C",
                0,
            ),
            p("rh", "Влажность", self.rh, 5.0, 100.0, "%", 0),
            p(
                "wind",
                "Ветер (на 10 м, как в прогнозе)",
                self.wind,
                0.0,
                30.0,
                "м/с",
                1,
            ),
            p(
                "sun",
                "Солнце на устройстве",
                self.sun,
                0.0,
                1000.0,
                "Вт/м²",
                0,
            ),
        ];
        if self.in_water() {
            v.push(p(
                "t_water",
                "Температура воды",
                self.t_water,
                0.0,
                35.0,
                "°C",
                0,
            ));
        }
        v.extend([
            p(
                "exposure",
                "Сколько времени",
                self.exposure_min,
                10.0,
                480.0,
                "мин",
                0,
            ),
            p(
                "clo",
                "Одежда (1 — костюм, 2.5 — зимняя куртка)",
                self.clo,
                0.1,
                4.0,
                "кло",
                1,
            ),
            p(
                "met",
                "Нагрузка (1 — покой, 4 — тяжёлая работа)",
                self.met,
                0.8,
                6.0,
                "мет",
                1,
            ),
        ]);
        v
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "t_air" => self.t_air = value,
            "rh" => self.rh = value,
            "wind" => self.wind = value,
            "sun" => self.sun = value,
            "t_water" => self.t_water = value,
            "exposure" => self.exposure_min = value,
            "clo" => self.clo = value,
            "met" => self.met = value,
            _ => return,
        }
        self.recompute();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "medium",
                label: "Где".into(),
                options: MEDIA.iter().map(|s| s.to_string()).collect(),
                selected: self.medium,
            },
            Choice {
                id: "frame",
                label: "Рама экзо".into(),
                options: FRAMES.iter().map(|f| format!("рама: {}", f.1)).collect(),
                selected: self.frame,
            },
            Choice {
                id: "liner",
                label: "Подкладка".into(),
                options: LINERS
                    .iter()
                    .map(|f| format!("подкладка: {}", f.1))
                    .collect(),
                selected: self.liner,
            },
            Choice {
                id: "coating",
                label: "Покрытие корпуса".into(),
                options: SURFACES
                    .iter()
                    .map(|s| format!("корпус: {}", s.0.name))
                    .collect(),
                selected: self.coating,
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "medium" => self.medium = index.min(MEDIA.len() - 1),
            "frame" => self.frame = index.min(FRAMES.len() - 1),
            "liner" => self.liner = index.min(LINERS.len() - 1),
            "coating" => self.coating = index.min(SURFACES.len() - 1),
            _ => return,
        }
        self.recompute();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Мороз −25 °C, ветер 10 м/с".into(),
            "Жара +40 °C, солнце, работа".into(),
            "Дождь +5 °C, ветер 12 м/с".into(),
            "Озеро 12 °C".into(),
            "Офис 22 °C".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        let (t, rh, wind, sun, medium, clo, met, water) = match index {
            0 => (-25.0, 70.0, 10.0, 0.0, 0, 2.5, 2.0, 12.0),
            1 => (40.0, 35.0, 2.0, 900.0, 0, 0.5, 3.5, 12.0),
            2 => (5.0, 98.0, 12.0, 0.0, 0, 1.0, 2.0, 12.0),
            3 => (18.0, 60.0, 3.0, 0.0, 1, 0.1, 2.0, 12.0),
            _ => (22.0, 45.0, 0.0, 0.0, 0, 1.0, 1.2, 12.0),
        };
        self.t_air = t;
        self.rh = rh;
        self.wind = wind;
        self.sun = sun;
        self.medium = medium;
        self.clo = clo;
        self.met = met;
        self.t_water = water;
        self.recompute();
    }

    fn reset(&mut self) {
        self.t = 0.0;
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let mut out = Vec::new();
        if let Some(sim) = &self.sim {
            bone_visuals(sim, &self.bones, &mut out);
        }
        let skin = self.skin_color();
        // Мягкие ткани — полупрозрачная «кожа», цвет по средней температуре кожи.
        for (c, r, l) in [
            (self.torso, 0.16, 0.42),
            (self.torso + Vec3::new(0.0, 0.0, 0.47), 0.1, 0.08),
        ] {
            out.push(VisualInstance {
                shape: Shape::Capsule {
                    radius: r,
                    length: l,
                },
                pose: Pose::z_along(c, Vec3::z()),
                color: skin,
            });
        }
        for (dy, z, len) in [
            (-0.2, 1.2, 0.24),
            (0.2, 1.2, 0.24),
            (-0.1, 0.65, 0.34),
            (0.1, 0.65, 0.34),
            (-0.1, 0.27, 0.34),
            (0.1, 0.27, 0.34),
        ] {
            out.push(VisualInstance {
                shape: Shape::Capsule {
                    radius: 0.06,
                    length: len,
                },
                pose: Pose::z_along(Vec3::new(self.torso.x, self.torso.y + dy, z), Vec3::z()),
                color: skin,
            });
        }
        // Устройство на предплечье — цвет по температуре рамы.
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.04, 0.025, 0.03),
            },
            pose: Pose::at(self.device),
            color: Self::temp_color(self.device_temp()),
        });
        if self.in_water() {
            let surface = 1.3;
            out.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.8, 0.8, surface / 2.0),
                },
                pose: Pose::at(Vec3::new(self.torso.x, self.torso.y, surface / 2.0)),
                color: if self.medium == 2 {
                    [0.1, 0.45, 0.55, 0.28]
                } else {
                    [0.25, 0.55, 0.9, 0.25]
                },
            });
        }
        if self.sun > 0.0 {
            out.push(VisualInstance {
                shape: Shape::Sphere { radius: 0.12 },
                pose: Pose::at(Vec3::new(1.2, -1.0, 2.6)),
                color: [1.0, 0.85, 0.3, 1.0],
            });
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let mut v = Vec::new();
        if self.wind > 0.5 && !self.in_water() {
            let len = 0.03 * self.wind;
            for i in -2..=2 {
                for k in 0..4 {
                    let start = Vec3::new(
                        self.torso.x + 0.9,
                        self.torso.y + 0.2 * i as f64,
                        0.4 + 0.4 * k as f64,
                    );
                    v.push(Overlay::Arrow {
                        from: start,
                        to: start - Vec3::x() * len,
                        color: [0.6, 0.85, 1.0, 0.8],
                    });
                }
            }
        }
        if self.sun > 0.0 {
            v.push(Overlay::Arrow {
                from: Vec3::new(1.1, -0.9, 2.45),
                to: self.device + Vec3::new(0.05, -0.05, 0.08),
                color: [1.0, 0.85, 0.3, 0.7],
            });
        }
        v
    }

    fn readouts(&self) -> Vec<Readout> {
        let mut r = Vec::new();
        if let Some(h) = &self.human {
            r.push(Readout::new(
                "Человек: ядро тела к концу",
                h.t_core_c,
                "°C",
                2,
            ));
            r.push(Readout::new("Человек: средняя кожа", h.t_skin_c, "°C", 1));
            r.push(Readout::new("Ощущается как (SET)", h.set_c, "°C", 1));
            r.push(Readout::new("Пот", h.sweat_g_h, "г/ч", 0));
            r.push(Readout::new("Потеря воды за время", h.water_loss_g, "г", 0));
            r.push(Readout::new(
                "Дрожь (тепло мышц)",
                h.shivering_w_m2,
                "Вт/м²",
                0,
            ));
        } else {
            let tl = immersion_timeline(self.t_water);
            r.push(Readout::new("Вода", self.t_water, "°C", 0));
            if let Some(m) = tl.time_to_core_30_min {
                r.push(Readout::new("До ядра 30 °C (Hayward 1975)", m, "мин", 0));
            }
            if let Some(c) = tl.core_cooling_c_per_min {
                r.push(Readout::new("Остывание ядра", c * 60.0, "°C/ч", 1));
            }
        }
        if !self.in_water() {
            let wc = wind_chill(self.t_air, self.wind * 3.6);
            r.push(Readout::new("Ветро-холодовой индекс", wc.index_c, "°C", 0));
            if let Some(m) = wc.frostbite_minutes {
                r.push(Readout::new("До обморожения открытой кожи", m, "мин", 0));
            }
            if self.t_air >= 27.0 {
                r.push(Readout::new(
                    "Индекс жары",
                    heat_index_c(self.t_air, self.rh),
                    "°C",
                    0,
                ));
            }
            r.push(Readout::new(
                "Ветер по Бофорту",
                beaufort(self.wind).number as f64,
                "балл",
                0,
            ));
        }
        let td = self.device_temp();
        r.push(Readout::new("Рама экзо", td, "°C", 1));
        let (i, tau, rha) = self.motor_continuous_torque();
        r.push(Readout::new(
            "Мотор: тепловое сопр. корпус→среда",
            rha,
            "К/Вт",
            2,
        ));
        r.push(Readout::new("Мотор: длительный ток", i, "А", 2));
        r.push(Readout::new(
            "Привод: длительный момент на локте",
            tau,
            "Н·м",
            1,
        ));
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let mut v = Vec::new();
        if let Some(h) = &self.human {
            v.push(Comparison {
                title: "Ядро тела: два численных способа".into(),
                sim_label: "явная схема ASHRAE (шаг 1 мин)".into(),
                sim: h.t_core_c,
                ref_label: "Рунге–Кутта 4 (шаг 0.1 мин)".into(),
                reference: self.human_rk4_core,
                unit: "°C",
                decimals: 3,
                status: if (h.t_core_c - self.human_rk4_core).abs() < 0.1 {
                    Status::Ok
                } else {
                    Status::Explained
                },
                explanation: "Одна модель, два интегратора. Расходятся, если модель вне \
                              области проверки (сильный холод — дрожь, ветер)."
                    .into(),
            });
        }
        // Мотор: наш расчёт охлаждения против паспорта maxon в тихом воздухе 25 °C.
        let p = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
        let calm = Environment {
            wind: 0.0,
            t_air: 25.0,
            medium: 0,
            sim: None,
            bones: Vec::new(),
            human: None,
            ..*self
        };
        v.push(Comparison {
            title: "Охлаждение мотора: наш расчёт против паспорта (тихий воздух)".into(),
            sim_label: "конвекция + излучение по корпусу".into(),
            sim: calm.motor_r_ha(60.0),
            ref_label: "maxon R_th2 (корпус → среда)".into(),
            reference: p.thermal.r_housing_ambient_k_per_w,
            unit: "К/Вт",
            decimals: 2,
            status: Status::Explained,
            explanation: "maxon меряет мотор, прикрученный к пластине (часть тепла уходит в \
                          крепление); у нас голый корпус — отводит хуже. Для экзо крепление \
                          через пластик ближе к нашему числу."
                .into(),
        });
        let (i, _, _) = self.motor_continuous_torque();
        let r_ds = p.thermal.r_housing_ambient_k_per_w;
        let i_model = p.continuous_current_a(self.fluid_temp(), 0.0, r_ds);
        let i_n = p.continuous_current_a(25.0, 0.0, r_ds);
        let i_formula = derated_current_maxon(i_n, p.thermal.max_winding_temp_c, self.fluid_temp());
        v.push(Comparison {
            title: "Длительный ток при этой температуре: модель против формулы maxon".into(),
            sim_label: "тепловая модель (паспортное R_th2)".into(),
            sim: i_model,
            ref_label: "I₂₅·√((T_max − T_a)/(T_max − 25))".into(),
            reference: i_formula,
            unit: "А",
            decimals: 3,
            status: Status::agreement(i_model, i_formula, 0.01),
            explanation: format!(
                "Обе — при паспортном охлаждении. С учётом ветра/воды по нашему расчёту: {i:.2} А."
            ),
        });
        v
    }

    fn notes(&self) -> Vec<Note> {
        let mut n = Vec::new();
        // Человек.
        if let Some(h) = &self.human {
            match core_status(h.t_core_c) {
                CoreStatus::SevereHypothermia | CoreStatus::ModerateHypothermia => n.push(Note::new(
                    Level::Danger,
                    format!("Ядро {:.1} °C — тяжёлое переохлаждение.", h.t_core_c),
                )),
                CoreStatus::MildHypothermia => n.push(Note::new(
                    Level::Danger,
                    format!("Ядро {:.1} °C — переохлаждение (< 35 °C) за {:.0} мин.", h.t_core_c, self.exposure_min),
                )),
                CoreStatus::AboveUnacclimatisedLimit | CoreStatus::AboveAcclimatisedLimit => {
                    n.push(Note::new(
                        Level::Warn,
                        format!("Ядро {:.1} °C — выше предела для работы (38–38.5 °C, ISO 7933/ACGIH): нужен отдых и вода.", h.t_core_c),
                    ))
                }
                CoreStatus::HeatStroke => n.push(Note::new(
                    Level::Danger,
                    format!("Ядро {:.1} °C — тепловой удар.", h.t_core_c),
                )),
                CoreStatus::Normal => n.push(Note::new(
                    Level::Good,
                    format!("Ядро тела к концу {:.0} мин — {:.1} °C, в норме.", self.exposure_min, h.t_core_c),
                )),
            }
            match hydration_status(h.water_loss_g / 1000.0, BODY_MASS) {
                HydrationStatus::Ok => {}
                HydrationStatus::Impaired => n.push(Note::new(
                    Level::Warn,
                    format!(
                        "Потеря воды {:.1} л (2–3 % массы) — работоспособность падает.",
                        h.water_loss_g / 1000.0
                    ),
                )),
                _ => n.push(Note::new(
                    Level::Danger,
                    format!(
                        "Потеря воды {:.1} л — больше 3 % массы, без питья нельзя.",
                        h.water_loss_g / 1000.0
                    ),
                )),
            }
            let out_of_range = h.warnings.iter().any(|w| {
                matches!(
                    w,
                    TwoNodeWarning::OutsideValidity { .. } | TwoNodeWarning::ColdExtrapolation
                )
            });
            if out_of_range {
                n.push(Note::new(
                    Level::Info,
                    "Модель Gagge проверена для помещений (10–40 °C, ветер до 2 м/с): здесь это \
                     экстраполяция — числа ориентировочные, у рук и лица охлаждение сильнее \
                     среднего по телу.",
                ));
            }
        } else {
            let tl = immersion_timeline(self.t_water);
            let mut s = format!("Вода {:.0} °C: ", self.t_water);
            if tl.cold_shock {
                s += &format!(
                    "холодовой шок первые ~{:.0} мин (неконтролируемый вдох), ",
                    tl.cold_shock_end_min
                );
            }
            if let Some((a, b)) = tl.swim_failure_window_min {
                s += &format!("руки отказывают через {a:.0}–{b:.0} мин, ");
            }
            if let Some(m) = tl.time_to_core_30_min {
                s += &format!("ядро 30 °C — через ~{m:.0} мин (Hayward 1975).");
            }
            n.push(Note::new(
                if self.t_water < 15.0 {
                    Level::Danger
                } else {
                    Level::Warn
                },
                s,
            ));
            n.push(Note::new(
                Level::Warn,
                "В воде экзо: Архимед снимает ~88 % веса руки (помощь «по весу» перебарщивает), \
                 Bluetooth не проходит, корпус — герметичный IP68, батарея — отдельно от воды. \
                 См. «Экзо на анатомической руке» и «Радио на теле».",
            ));
        }
        if !self.in_water() {
            let wc = wind_chill(self.t_air, self.wind * 3.6);
            if let Some(m) = wc.frostbite_minutes {
                n.push(Note::new(
                    if m < 30.0 { Level::Danger } else { Level::Warn },
                    format!(
                        "Ветро-холод {:.0} °C: открытая кожа обморозится за ~{m:.0} мин (таблица Environment Canada). Пальцам нужны перчатки — значит, кнопки экзо должны нажиматься в перчатке.",
                        wc.index_c
                    ),
                ));
            }
            if self.t_air >= 27.0 {
                let hi = heat_index_c(self.t_air, self.rh);
                n.push(Note::new(
                    if hi > 41.0 {
                        Level::Danger
                    } else {
                        Level::Warn
                    },
                    format!(
                        "Индекс жары {hi:.0} °C (NWS): риск теплового удара при работе в экзо."
                    ),
                ));
            }
            let b = beaufort(self.wind);
            if b.number >= 6 {
                n.push(Note::new(
                    Level::Warn,
                    format!(
                        "Ветер {} баллов ({}): {}",
                        b.number, b.name, b.effect_on_people
                    ),
                ));
            }
        }
        // Руки и мышцы при холоде — по литературе (температуру пальцев модель не считает).
        if self.fluid_temp() < 10.0 {
            let (f, lo, hi, _) = muscle_power_factor(32.0);
            let hand = |t: f64| match hand_function(t) {
                HandFunction::Normal => "норма",
                HandFunction::ReducedDexterity => "ловкость падает",
                HandFunction::ReducedTactile => "осязание падает",
                HandFunction::Numb => "онемение",
                HandFunction::FreezingRisk => "риск обморожения",
            };
            n.push(Note::new(
                Level::Info,
                format!(
                    "Холодные мышцы и пальцы (литература): мышца предплечья 32 °C вместо 37 — \
                     мощность ×{f:.2} ({lo:.2}–{hi:.2}); пальцы 15 °C — {}, 10 °C — {}, 7 °C — {}. \
                     Экзо, усиливающее хват на морозе, должно учитывать, что человек хуже \
                     чувствует силу сжатия.",
                    hand(15.0),
                    hand(10.0),
                    hand(7.0)
                ),
            ));
        }
        // Касание рамы и подкладки.
        let td = self.device_temp();
        let (frame, frame_name) = FRAMES[self.frame];
        match LINERS[self.liner] {
            (None, _) => {
                for (secs, what) in [
                    (1.0, "коснуться"),
                    (10.0, "подержать 10 с"),
                    (60.0, "носить (1 мин)"),
                ] {
                    let rk = contact_risk(frame, td, secs);
                    if rk.level != RiskLevel::Safe {
                        n.push(Note::new(
                            risk_level(rk.level),
                            format!(
                                "Рама ({frame_name}, {td:.0} °C) у кожи: {what} — {} (температура контакта {:.0} °C; {}). Нужна подкладка.",
                                risk_text(rk.level),
                                rk.contact_temp_c,
                                rk.threshold_source
                            ),
                        ));
                        break;
                    }
                }
            }
            (Some(m), name) => {
                let rk = contact_risk(m, td, 600.0);
                n.push(Note::new(
                    if rk.level == RiskLevel::Safe { Level::Good } else { risk_level(rk.level) },
                    format!(
                        "Подкладка «{name}» при {td:.0} °C: касание 10 мин — {} (контакт {:.0} °C). Рама сама кожи не касается.",
                        risk_text(rk.level),
                        rk.contact_temp_c
                    ),
                ));
            }
        }
        // Конденсат: устройство с мороза занесли в комнату 22 °C, 50 %.
        if !self.in_water() && td < 10.0 {
            let c = condensation_check(22.0, 50.0, td);
            if c.condenses {
                n.push(Note::new(
                    Level::Warn,
                    format!(
                        "Занесли с {td:.0} °C в комнату 22 °C/50 %: точка росы {:.0} °C — на плате выпадет {}. Включать после прогрева (~30–60 мин) или покрыть плату лаком.",
                        c.dew_point_c,
                        if c.frost { "иней, потом вода" } else { "вода" }
                    ),
                ));
            }
        }
        // Батарея и электроника.
        // Батарея в корпусе на руке: греется солнцем вместе с корпусом.
        let t_bat = if self.sun > 0.0 {
            td
        } else {
            self.fluid_temp()
        };
        if t_bat < LI_ION.charge_c.0 {
            n.push(Note::new(Level::Danger, format!("Батарея {t_bat:.0} °C: заряжать Li-ion ниже {:.0} °C нельзя (литий осаждается на аноде) — зарядку в прошивке блокировать по датчику.", LI_ION.charge_c.0)));
        }
        if t_bat < LI_ION.discharge_c.0 || t_bat > LI_ION.discharge_c.1 {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Батарея при {t_bat:.0} °C — вне диапазона разряда {:.0}…{:.0} °C ({}).{}",
                    LI_ION.discharge_c.0,
                    LI_ION.discharge_c.1,
                    LI_ION.source,
                    if self.sun > 0.0 {
                        " Батарею — в тень/на пояс, корпус — светлый."
                    } else {
                        ""
                    }
                ),
            ));
        }
        let commercial = ELECTRONICS_GRADES[0];
        if td < commercial.range_c.0 || td > commercial.range_c.1 {
            n.push(Note::new(Level::Warn, format!("Электроника: {td:.0} °C вне коммерческого диапазона {:.0}…{:.0} °C — нужны компоненты промышленного класса (−40…85 °C).", commercial.range_c.0, commercial.range_c.1)));
        }
        // Пластики на солнце.
        if self.sun > 0.0 {
            for m in MATERIAL_LIMITS
                .iter()
                .filter(|m| m.name == "PLA" || m.name == "PETG")
            {
                if td > m.max_service_c {
                    n.push(Note::new(Level::Danger, format!("Корпус на солнце {td:.0} °C: детали из {} размягчаются (предел службы ~{:.0} °C) — печатать из PA12/ASA или металла.", m.name, m.max_service_c)));
                }
            }
        }
        let (i, tau, _) = self.motor_continuous_torque();
        n.push(Note::new(
            Level::Info,
            format!(
                "Привод maxon + HD в этих условиях длительно даёт {tau:.0} Н·м ({i:.1} А): холод и \
                 ветер/вода охлаждают — момент растёт, жара — падает."
            ),
        ));
        n
    }

    fn plot(&self) -> Option<Plot> {
        let h = self.human.as_ref()?;
        Some(Plot {
            title: "Температура тела за время пребывания".into(),
            x_label: "минуты".into(),
            y_label: "°C".into(),
            series: vec![
                Series {
                    name: "ядро".into(),
                    points: h.history.iter().map(|s| [s.t_min, s.t_core_c]).collect(),
                    color: C_RED,
                    style: SeriesStyle::Solid,
                },
                Series {
                    name: "кожа (средняя)".into(),
                    points: h.history.iter().map(|s| [s.t_min, s.t_skin_c]).collect(),
                    color: C_BLUE,
                    style: SeriesStyle::Solid,
                },
            ],
            bands: vec![Band {
                label: "ядро в норме 35–38 °C".into(),
                lo: 35.0,
                hi: 38.0,
                color: [0.2, 0.85, 0.35, 0.12],
            }],
        })
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(self.torso.x, self.torso.y, 1.0)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(1.2, -2.4, 0.4)
    }

    fn mesh_data(&self, id: u32) -> Option<MeshData> {
        let sim = self.sim.as_ref()?;
        let m = sim.mesh(id as usize);
        Some(MeshData {
            positions: m.positions,
            indices: m.indices,
        })
    }
}
