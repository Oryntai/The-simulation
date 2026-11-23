//! Сценарий «Радио на теле»: устройство на предплечье (экзоскелет) говорит с телефоном по
//! Bluetooth LE или Wi-Fi. Бюджет линии собирается из измерений канала на теле
//! (IEEE 802.15.6), потерь в воде (две модели проницаемости) и паспорта ESP32-C3;
//! пакеты живут во времени: события соединения BLE, повторы, разрывы, задержки.

use sim_core::{Pose, Shape, Vec3};
use sim_mujoco::{GeomInfo, Sim};
use sim_radio::body::{around_body_db, off_body_shadowing_mean_db};
use sim_radio::channel::{Dir, PacketChannel, Protocol, Rng, ble_mean_latency_analytic};
use sim_radio::dielectric::{
    Medium, Permittivity, Tissue, interface_loss_db, seawater_conductivity_klein_swift,
    water_itu_p840, water_klein_swift,
};
use sim_radio::link::packet_error_rate;
use sim_radio::pathloss::{Surroundings, fspl_db, fspl_p525_db};
use sim_radio::phy::Phy;
use sim_radio::wearable::{F_2G4, WearableLink, max_depth_m};

use crate::anatomy::{Which, bone_geoms, bone_visuals, open_model};
use crate::scenario::*;

const SURROUNDINGS: [Surroundings; 3] = [
    Surroundings::OpenLos,
    Surroundings::IndoorLos,
    Surroundings::IndoorObstructed,
];
const BLE_INTERVALS_MS: [f64; 4] = [7.5, 15.0, 30.0, 100.0];
/// Длина пакета телеметрии/команды в эфире, бит (заголовок + 12 байт данных + CRC, оценка).
const PACKET_BITS: f64 = 30.0 * 8.0;
const STEP: f64 = 0.005;
const TELEMETRY_PERIOD: f64 = 0.02;
const COMMAND_PERIOD: f64 = 0.05;
/// Время корреляции медленных замираний (движение руки/корпуса), с. Оценка.
const FADING_TAU: f64 = 0.5;

pub struct RadioBody {
    sim: Option<Sim>,
    bones: Vec<GeomInfo>,
    load_error: Option<String>,
    device: Vec3,
    torso: Vec3,
    // параметры
    distance: f64,
    tx_dbm: f64,
    depth_cm: f64,
    water_t: f64,
    phy: usize,
    surroundings: usize,
    body_blocks: bool,
    medium: usize,
    interval: usize,
    // живое состояние
    t: f64,
    clock: FixedClock,
    rng: Rng,
    fading_db: f64,
    channel: PacketChannel,
    next_telemetry: f64,
    next_command: f64,
    history: History,
    lat_history: History,
    log: Vec<String>,
    was_connected: bool,
}

impl Default for RadioBody {
    fn default() -> Self {
        let (sim, load_error) = match open_model(Which::FullBody) {
            Ok(s) => (Some(s), None),
            Err(e) => (None, Some(e)),
        };
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
            load_error,
            device,
            torso,
            distance: 3.0,
            tx_dbm: 9.0,
            depth_cm: 0.0,
            water_t: 15.0,
            phy: 0,
            surroundings: 1,
            body_blocks: false,
            medium: 0,
            interval: 1,
            t: 0.0,
            clock: FixedClock::new(STEP),
            rng: Rng::new(1),
            fading_db: 0.0,
            channel: PacketChannel::new(Protocol::ble_default(), 1),
            next_telemetry: 0.0,
            next_command: 0.0,
            history: History::new(3, 20.0),
            lat_history: History::new(1, 20.0),
            log: Vec::new(),
            was_connected: true,
        };
        s.reset();
        s
    }
}

impl RadioBody {
    fn phy(&self) -> Phy {
        Phy::ALL[self.phy]
    }

    fn medium(&self) -> Medium {
        match self.medium {
            1 => Medium::FreshWater { t_c: self.water_t },
            2 => Medium::SeaWater { t_c: self.water_t },
            _ => Medium::Air,
        }
    }

    fn link(&self) -> WearableLink {
        WearableLink {
            phy: self.phy(),
            tx_dbm: self.tx_dbm.min(self.phy().max_tx_dbm()),
            distance_m: self.distance,
            surroundings: SURROUNDINGS[self.surroundings],
            body_blocks: self.body_blocks,
            medium: self.medium(),
            depth_m: if self.medium == 0 {
                0.0
            } else {
                self.depth_cm / 100.0
            },
            wet_detuning_db: 0.0,
        }
    }

    fn protocol(&self) -> Protocol {
        if self.phy().is_ble() {
            Protocol::Ble {
                interval_s: BLE_INTERVALS_MS[self.interval] / 1000.0,
                supervision_timeout_s: 0.5,
                adv_interval_s: 0.1,
            }
        } else {
            Protocol::Wifi
        }
    }

    fn phone_pos(&self) -> Vec3 {
        // Человек смотрит вдоль +X. Телефон перед ним или за спиной (тело закрывает).
        let dir = if self.body_blocks { -1.0 } else { 1.0 };
        Vec3::new(
            self.torso.x + dir * self.distance,
            self.device.y,
            0.9_f64.max(self.device.z),
        )
    }

    fn gauss(&mut self) -> f64 {
        let u1 = self.rng.uniform().max(1e-12);
        let u2 = self.rng.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    fn step(&mut self) {
        let budget = self.link().budget();
        let sigma = budget.sigma_db;
        // Медленные замирания: процесс Гаусса–Маркова с σ из измерений и τ = 0.5 с.
        let a = STEP / FADING_TAU;
        let n = self.gauss();
        self.fading_db += -a * self.fading_db + sigma * (2.0 * a).sqrt() * n;
        let margin = budget.margin_db() + self.fading_db;
        let per = packet_error_rate(self.phy(), margin, PACKET_BITS);
        let rssi = budget.rx_dbm() + self.fading_db;
        self.channel.set_link(per, rssi);
        self.t += STEP;
        // Часы устройства, телефона и эфира независимы: момент отправки внутри шага случаен.
        if self.t >= self.next_telemetry {
            let at = self.t - STEP * self.rng.uniform();
            self.channel.send(at, Dir::DeviceToPhone, &[0x7E; 14]);
            self.next_telemetry += TELEMETRY_PERIOD;
        }
        if self.t >= self.next_command {
            let at = self.t - STEP * self.rng.uniform();
            self.channel.send(at, Dir::PhoneToDevice, &[0xC1; 8]);
            self.next_command += COMMAND_PERIOD;
        }
        for d in self.channel.advance(self.t) {
            self.lat_history.push(self.t, &[d.latency_s * 1000.0]);
        }
        let connected = self.channel.connected();
        if connected != self.was_connected {
            self.log.push(format!(
                "{:7.2} с  {}",
                self.t,
                if connected {
                    "соединение восстановлено"
                } else {
                    "СОЕДИНЕНИЕ ПОТЕРЯНО (таймаут надзора 0.5 с)"
                }
            ));
            self.was_connected = connected;
        }
        self.history.push(
            self.t,
            &[
                rssi,
                self.phy().sensitivity_dbm(),
                if connected { 1.0 } else { 0.0 },
            ],
        );
    }
}

impl Scenario for RadioBody {
    fn id(&self) -> &'static str {
        "radio_body"
    }

    fn title(&self) -> String {
        "Радио на теле: Bluetooth и Wi-Fi".into()
    }

    fn summary(&self) -> String {
        "Устройство на предплечье (экзоскелет, ESP32-C3) говорит с телефоном. Потери у тела — \
         по измерениям IEEE 802.15.6, в воде — по двум моделям проницаемости, приёмник — по \
         даташиту. Пакеты идут во времени: события соединения, повторы, разрывы."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param {
                id: "distance",
                label: "Расстояние до телефона".into(),
                value: self.distance,
                min: 0.5,
                max: 60.0,
                unit: "м",
                decimals: 1,
            },
            Param {
                id: "tx",
                label: "Мощность передатчика".into(),
                value: self.tx_dbm,
                min: -24.0,
                max: 21.0,
                unit: "дБм",
                decimals: 0,
            },
            Param {
                id: "depth",
                label: "Вода над устройством".into(),
                value: self.depth_cm,
                min: 0.0,
                max: 50.0,
                unit: "см",
                decimals: 1,
            },
            Param {
                id: "water_t",
                label: "Температура воды".into(),
                value: self.water_t,
                min: 0.0,
                max: 35.0,
                unit: "°C",
                decimals: 0,
            },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "distance" => self.distance = value,
            "tx" => self.tx_dbm = value,
            "depth" => self.depth_cm = value,
            "water_t" => self.water_t = value,
            _ => {}
        }
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "phy",
                label: "Радио".into(),
                options: Phy::ALL.iter().map(|p| p.name().to_string()).collect(),
                selected: self.phy,
            },
            Choice {
                id: "room",
                label: "Обстановка".into(),
                options: SURROUNDINGS.iter().map(|s| s.name().to_string()).collect(),
                selected: self.surroundings,
            },
            Choice {
                id: "body",
                label: "Тело между".into(),
                options: vec![
                    "телефон перед человеком (видно)".into(),
                    "телефон за спиной (тело закрывает)".into(),
                ],
                selected: self.body_blocks as usize,
            },
            Choice {
                id: "medium",
                label: "Среда".into(),
                options: vec![
                    "сухо (воздух)".into(),
                    "рука в пресной воде".into(),
                    "рука в морской воде".into(),
                ],
                selected: self.medium,
            },
            Choice {
                id: "interval",
                label: "Интервал соединения BLE".into(),
                options: BLE_INTERVALS_MS
                    .iter()
                    .map(|m| format!("интервал BLE {m} мс"))
                    .collect(),
                selected: self.interval,
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "phy" => self.phy = index.min(Phy::ALL.len() - 1),
            "room" => self.surroundings = index.min(2),
            "body" => self.body_blocks = index == 1,
            "medium" => {
                self.medium = index.min(2);
                if self.medium > 0 && self.depth_cm == 0.0 {
                    self.depth_cm = 5.0;
                }
            }
            "interval" => self.interval = index.min(BLE_INTERVALS_MS.len() - 1),
            _ => {}
        }
        self.reset();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Телефон в 3 м, видно".into(),
            "Телефон за спиной".into(),
            "Рука в воде 5 см".into(),
            "Море, 5 см".into(),
            "Wi-Fi 54 Мбит/с, 30 м".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        let (phy, dist, blocks, medium, depth, room) = match index {
            0 => (0, 3.0, false, 0, 0.0, 1),
            1 => (0, 3.0, true, 0, 0.0, 1),
            2 => (0, 3.0, false, 1, 5.0, 1),
            3 => (0, 3.0, false, 2, 5.0, 0),
            _ => (5, 30.0, false, 0, 0.0, 2),
        };
        self.phy = phy;
        self.distance = dist;
        self.body_blocks = blocks;
        self.medium = medium;
        self.depth_cm = depth;
        self.surroundings = room;
        self.tx_dbm = 9.0;
        self.reset();
    }

    fn reset(&mut self) {
        self.t = 0.0;
        self.clock.reset();
        self.rng = Rng::new(42);
        self.fading_db = 0.0;
        self.channel = PacketChannel::new(self.protocol(), 7);
        self.next_telemetry = 0.0;
        self.next_command = 0.0;
        self.history.clear();
        self.lat_history.clear();
        self.log.clear();
        self.was_connected = true;
        self.log.push(format!(
            "   0.00 с  старт: {}, {:.0} дБм, {:.1} м",
            self.phy().name(),
            self.tx_dbm.min(self.phy().max_tx_dbm()),
            self.distance
        ));
    }

    fn advance(&mut self, dt_real: f64) {
        for _ in 0..self.clock.steps(dt_real) {
            self.step();
        }
        if self.log.len() > 200 {
            self.log.drain(..self.log.len() - 200);
        }
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let mut out = Vec::new();
        if let Some(sim) = &self.sim {
            bone_visuals(sim, &self.bones, &mut out);
        }
        // Мягкие ткани корпуса — полупрозрачная оболочка (через неё сигнал не идёт).
        out.push(VisualInstance {
            shape: Shape::Capsule {
                radius: 0.15,
                length: 0.45,
            },
            pose: Pose::z_along(self.torso, Vec3::z()),
            color: [0.93, 0.7, 0.6, 0.18],
        });
        // Устройство на предплечье и телефон.
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.03, 0.02, 0.012),
            },
            pose: Pose::at(self.device),
            color: C_EXO_CUFF,
        });
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.004, 0.036, 0.075),
            },
            pose: Pose::at(self.phone_pos()),
            color: [0.1, 0.1, 0.12, 1.0],
        });
        out.push(VisualInstance {
            shape: Shape::Cylinder {
                radius: 0.02,
                length: self.phone_pos().z,
            },
            pose: Pose::z_along(
                Vec3::new(
                    self.phone_pos().x,
                    self.phone_pos().y,
                    self.phone_pos().z / 2.0,
                ),
                Vec3::z(),
            ),
            color: C_GRAY,
        });
        if self.medium != 0 {
            let surface = self.device.z + self.depth_cm / 100.0;
            let col = if self.medium == 2 {
                [0.1, 0.45, 0.55, 0.28]
            } else {
                [0.25, 0.55, 0.9, 0.25]
            };
            out.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.6, 0.6, surface / 2.0),
                },
                pose: Pose::at(Vec3::new(self.torso.x, self.torso.y, surface / 2.0)),
                color: col,
            });
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        (self.medium as u64) * 1000
            + (self.depth_cm * 10.0) as u64 * 7
            + (self.distance * 10.0) as u64 * 131
            + self.body_blocks as u64 * 999_983
    }

    fn overlays(&self) -> Vec<Overlay> {
        let budget = self.link().budget();
        let m = budget.margin_db() + self.fading_db;
        let color = if !self.channel.connected() || m < 0.0 {
            C_RED
        } else if m < 10.0 {
            C_YELLOW
        } else {
            C_GREEN
        };
        let phone = self.phone_pos();
        let mut v = Vec::new();
        if self.body_blocks {
            // Волна огибает корпус по коже (полуокружность вокруг туловища), потом — к телефону.
            let r = 0.17;
            let c = Vec3::new(self.torso.x, self.torso.y, self.device.z);
            let mut prev = self.device;
            let n = 10;
            for i in 0..=n {
                // От правого бока (−Y) назад (−X).
                let a = -std::f64::consts::FRAC_PI_2
                    - std::f64::consts::FRAC_PI_2 * i as f64 / n as f64;
                let p = c + Vec3::new(r * a.cos(), r * a.sin(), 0.0);
                v.push(Overlay::Line {
                    from: prev,
                    to: p,
                    color,
                });
                prev = p;
            }
            v.push(Overlay::Line {
                from: prev,
                to: phone,
                color,
            });
        } else {
            v.push(Overlay::Line {
                from: self.device,
                to: phone,
                color,
            });
        }
        v.push(Overlay::WireSphere {
            center: self.device,
            radius: 0.05,
            color,
        });
        v
    }

    fn readouts(&self) -> Vec<Readout> {
        let link = self.link();
        let b = link.budget();
        let mut r = vec![
            Readout::new("Мощность передатчика", b.tx_dbm, "дБм", 0),
            Readout::new("Уровень на приёмнике (средний)", b.rx_dbm(), "дБм", 1),
            Readout::new(
                "Чувствительность (ESP32-C3)",
                self.phy().sensitivity_dbm(),
                "дБм",
                1,
            ),
            Readout::new("Запас линии", b.margin_db(), "дБ", 1),
            Readout::new(
                "Вероятность обрыва (замирания)",
                100.0 * b.outage_probability(),
                "%",
                2,
            ),
            Readout::new(
                "Потери пакетов (в среднем)",
                100.0 * b.mean_per(PACKET_BITS),
                "%",
                2,
            ),
        ];
        for t in &b.terms {
            r.push(Readout::new(&format!("  {}", t.name), t.db, "дБ", 1));
        }
        let st = &self.channel.stats;
        r.push(Readout::new("Отправлено пакетов", st.sent as f64, "", 0));
        r.push(Readout::new("Потеряно", 100.0 * st.loss_fraction(), "%", 2));
        r.push(Readout::new(
            "Средняя задержка",
            st.mean_latency_s() * 1000.0,
            "мс",
            1,
        ));
        r.push(Readout::new(
            "Наибольшая задержка",
            st.latency_max_s * 1000.0,
            "мс",
            1,
        ));
        r.push(Readout::new(
            "Разрывов соединения",
            st.disconnects as f64,
            "",
            0,
        ));
        if self.medium != 0 {
            r.push(Readout::new(
                "Предельная глубина воды",
                100.0 * max_depth_m(&link),
                "см",
                1,
            ));
        }
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let f = F_2G4;
        let d = self.distance;
        let (friis, p525) = (fspl_db(f, d), fspl_p525_db(f * 1e-6, d / 1000.0));
        let mut v = vec![Comparison {
            title: "Свободное пространство на этом расстоянии".into(),
            sim_label: "Фриис (4π·d/λ)²".into(),
            sim: friis,
            ref_label: "ITU-R P.525".into(),
            reference: p525,
            unit: "дБ",
            decimals: 2,
            // Константы 32.45 и 32.4 расходятся на 0.05 дБ, поэтому допуск абсолютный.
            status: if friis.is_finite() && p525.is_finite() && (friis - p525).abs() <= 0.1 {
                Status::Ok
            } else {
                Status::Mismatch
            },
            explanation: "Две записи одной формулы; разница — округление константы 32.4.".into(),
        }];
        let link = self.link();
        v.push(Comparison {
            title: "Трасса у тела против свободного пространства (1–4 м)".into(),
            sim_label: "Измерено IEEE 802.15.6 CM4".into(),
            sim: link.body_excess_over_free_space_db() + fspl_db(f, d.clamp(1.0, 4.0)),
            ref_label: "Свободное пространство".into(),
            reference: fspl_db(f, d.clamp(1.0, 4.0)),
            unit: "дБ",
            decimals: 1,
            status: Status::Explained,
            explanation: "Тело рядом с антенной поглощает и расстраивает её: +10…25 дБ к \
                          свободному пространству. Считать «как в воздухе» нельзя."
                .into(),
        });
        let muscle = Tissue::Muscle
            .permittivity_2g45()
            .attenuation_db_per_m(2.45e9)
            * 0.25;
        v.push(Comparison {
            title: "Почему сигнал огибает тело".into(),
            sim_label: "Сквозь 25 см мышцы".into(),
            sim: muscle,
            ref_label: "Вокруг корпуса (40 см по коже)".into(),
            reference: around_body_db(40.0).0,
            unit: "дБ",
            decimals: 0,
            status: Status::Explained,
            explanation: format!(
                "Ткани теряют ~{:.1} дБ/см (IT'IS), а ползущая волна — 2 дБ/см и упирается в \
                 отражения от комнаты. Тело за спиной в среднем добавляет {:.0} дБ.",
                muscle / 25.0,
                off_body_shadowing_mean_db()
            ),
        });
        match self.medium() {
            Medium::FreshWater { t_c } => {
                let a = water_klein_swift(t_c, 0.0, 0.0, f).attenuation_db_per_m(f) / 100.0;
                let b = water_itu_p840(t_c, f).attenuation_db_per_m(f) / 100.0;
                v.push(Comparison {
                    title: format!("Затухание в пресной воде {t_c:.0} °C"),
                    sim_label: "Klein & Swift 1977".into(),
                    sim: a,
                    ref_label: "ITU-R P.840".into(),
                    reference: b,
                    unit: "дБ/см",
                    decimals: 2,
                    status: if ((a - b) / b).abs() < 0.08 {
                        Status::Ok
                    } else {
                        Status::Mismatch
                    },
                    explanation: "Две независимые модели воды.".into(),
                });
            }
            Medium::SeaWater { t_c } => {
                let eps = Medium::SeaWater { t_c }.permittivity(f);
                v.push(Comparison {
                    title: format!("Морская вода {t_c:.0} °C: откуда потери"),
                    sim_label: "Полные потери (ε'')".into(),
                    sim: eps.conductivity(f),
                    ref_label: "Ионная проводимость".into(),
                    reference: seawater_conductivity_klein_swift(t_c, 35.0),
                    unit: "См/м",
                    decimals: 2,
                    status: Status::Explained,
                    explanation: format!(
                        "Соль даёт основную часть потерь: {:.1} дБ/см против {:.1} в пресной.",
                        eps.attenuation_db_per_m(f) / 100.0,
                        water_klein_swift(t_c, 0.0, 0.0, f).attenuation_db_per_m(f) / 100.0
                    ),
                });
            }
            Medium::Air => {}
        }
        if let Protocol::Ble { interval_s, .. } = self.protocol() {
            let b = link.budget();
            let per = b.mean_per(PACKET_BITS);
            let st = &self.channel.stats;
            let reference = ble_mean_latency_analytic(interval_s, per.min(0.95));
            let enough = st.delivered > 200 && st.disconnects == 0;
            v.push(Comparison {
                title: "Средняя задержка пакета BLE".into(),
                sim_label: "Моделирование канала".into(),
                sim: st.mean_latency_s() * 1000.0,
                ref_label: "T/2 + T·p/(1 − p)".into(),
                reference: reference * 1000.0,
                unit: "мс",
                decimals: 1,
                status: if !enough {
                    Status::Pending
                } else if (st.mean_latency_s() - reference).abs() / reference < 0.2 {
                    Status::Ok
                } else {
                    Status::Explained
                },
                explanation: "Формула — для одиночных пакетов; очередь и медленные замирания \
                              удлиняют хвост."
                    .into(),
            });
        }
        v
    }

    fn notes(&self) -> Vec<Note> {
        let link = self.link();
        let b = link.budget();
        let mut n = Vec::new();
        if let Some(e) = &self.load_error {
            n.push(Note::new(
                Level::Warn,
                format!("Модель тела не открылась: {e}"),
            ));
        }
        let m = b.margin_db();
        if m < 0.0 {
            n.push(Note::new(
                Level::Danger,
                format!("Связи нет: сигнал на {:.0} дБ ниже чувствительности.", -m),
            ));
        } else if b.outage_probability() > 0.01 {
            n.push(Note::new(
                Level::Warn,
                format!(
                    "Запас {:.0} дБ мал: при движениях тела связь будет рваться ({:.1} % времени).",
                    m,
                    100.0 * b.outage_probability()
                ),
            ));
        } else {
            n.push(Note::new(
                Level::Good,
                format!("Запас {m:.0} дБ: связь устойчива при обычных замираниях."),
            ));
        }
        if self.medium != 0 {
            let eps = self.medium().permittivity(F_2G4);
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Под водой 2.4 ГГц не работает: {:.1} дБ на каждый сантиметр + {:.1} дБ на \
                     границе вода–воздух. Связь пропадёт глубже {:.1} см. Под водой — только \
                     провод, звук или длинные волны; прошивка должна сама держать безопасный \
                     режим без связи.",
                    eps.attenuation_db_per_m(F_2G4) / 100.0,
                    interface_loss_db(eps, Permittivity::VACUUM),
                    100.0 * max_depth_m(&link)
                ),
            ));
        }
        if self.body_blocks {
            n.push(Note::new(
                Level::Warn,
                "Тело между антеннами: сигнал идёт в обход по коже и отражениями от комнаты. \
                 Ставьте антенну наружу (на внешнюю сторону манжеты), а не к телу.",
            ));
        }
        n.push(Note::new(
            Level::Info,
            "Правило конструкции: по радио — только команды режима и телеметрия. Петли тока и \
             положения мотора крутятся на плате (10 кГц / 1 кГц); при потере связи прошивка \
             плавно снимает усилие — это проверяется в сценарии «Прошивка в контуре».",
        ));
        n
    }

    fn plots(&self) -> Vec<Plot> {
        vec![
            Plot {
                title: "Уровень сигнала у телефона".into(),
                x_label: "время, с".into(),
                y_label: "дБм".into(),
                series: vec![
                    Series {
                        name: "уровень с замираниями".into(),
                        points: self.history.points(0, 15.0, 600),
                        color: C_BLUE,
                        style: SeriesStyle::Solid,
                    },
                    Series {
                        name: "чувствительность".into(),
                        points: self.history.points(1, 15.0, 50),
                        color: C_RED,
                        style: SeriesStyle::Dashed,
                    },
                ],
                bands: vec![],
            },
            Plot {
                title: "Задержка доставленных пакетов".into(),
                x_label: "время, с".into(),
                y_label: "мс".into(),
                series: vec![Series {
                    name: "задержка".into(),
                    points: self.lat_history.points(0, 15.0, 600),
                    color: C_YELLOW,
                    style: SeriesStyle::Solid,
                }],
                bands: vec![],
            },
        ]
    }

    fn console(&self) -> Vec<String> {
        self.log.clone()
    }

    fn focus(&self) -> Vec3 {
        let p = self.phone_pos();
        Vec3::new((self.torso.x + p.x) / 2.0, self.torso.y, 1.0)
    }

    fn view_offset(&self) -> Vec3 {
        let span = self.distance.clamp(1.0, 12.0);
        Vec3::new(0.3 * span, -1.6 * span - 1.0, 0.5 + 0.2 * span)
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
