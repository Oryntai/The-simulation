//! Общий интерфейс сценария: окно приложения ничего не знает о физике конкретного
//! устройства — оно рисует то, что сценарий отдаёт (геометрию, стрелки, цифры, сверки).

use nalgebra::DVector;
use sim_core::dynamics::body_poses;
use sim_core::{Model, Pose, Shape, Vec3};

#[derive(Clone, Debug)]
pub struct Param {
    pub id: &'static str,
    pub label: String,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub unit: &'static str,
    pub decimals: usize,
}

#[derive(Clone, Debug)]
pub struct Choice {
    pub id: &'static str,
    pub label: String,
    pub options: Vec<String>,
    pub selected: usize,
}

#[derive(Clone, Debug)]
pub struct Readout {
    pub label: String,
    pub value: f64,
    pub unit: &'static str,
    pub decimals: usize,
}

impl Readout {
    pub fn new(label: &str, value: f64, unit: &'static str, decimals: usize) -> Self {
        Self {
            label: label.to_string(),
            value,
            unit,
            decimals,
        }
    }
}

/// Статус сверки «симуляция ↔ ручной расчёт».
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Совпало в пределах допуска.
    Ok,
    /// Расходится, но причина известна и показана (допущение формулы).
    Explained,
    /// Расходится без объяснения — где-то ошибка, верить нельзя.
    Mismatch,
    /// Ещё нечего сравнивать (переходный процесс, нет данных).
    Pending,
    /// Это не сверка двух расчётов, а требование, и оно выполнено.
    WithinLimit,
    /// Требование нарушено: устройство в таком виде не проходит.
    OverLimit,
    /// Опасность существует, но симуляция её пока не считает.
    NotAnalyzed,
}

impl Status {
    /// «Совпало» только по самим числам: оба конечны и расходятся не больше `rel_tol`.
    pub fn agreement(sim: f64, reference: f64, rel_tol: f64) -> Status {
        if sim.is_finite()
            && reference.is_finite()
            && (sim - reference).abs() <= rel_tol * reference.abs()
        {
            Status::Ok
        } else {
            Status::Mismatch
        }
    }
}

#[derive(Clone, Debug)]
pub struct Comparison {
    pub title: String,
    pub sim_label: String,
    pub sim: f64,
    pub ref_label: String,
    pub reference: f64,
    pub unit: &'static str,
    pub decimals: usize,
    pub status: Status,
    pub explanation: String,
}

impl Comparison {
    pub fn rel_diff(&self) -> f64 {
        if self.reference.abs() > 0.0 {
            (self.sim - self.reference) / self.reference.abs()
        } else {
            self.sim - self.reference
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Info,
    Good,
    Warn,
    Danger,
}

#[derive(Clone, Debug)]
pub struct Note {
    pub level: Level,
    pub text: String,
}

impl Note {
    pub fn new(level: Level, text: impl Into<String>) -> Self {
        Self {
            level,
            text: text.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeriesStyle {
    Solid,
    Dashed,
}

#[derive(Clone, Debug)]
pub struct Series {
    pub name: String,
    pub points: Vec<[f64; 2]>,
    pub color: [f32; 4],
    pub style: SeriesStyle,
}

#[derive(Clone, Debug)]
pub struct Band {
    pub label: String,
    pub lo: f64,
    pub hi: f64,
    pub color: [f32; 4],
}

#[derive(Clone, Debug)]
pub struct Plot {
    pub title: String,
    pub x_label: String,
    pub y_label: String,
    pub series: Vec<Series>,
    pub bands: Vec<Band>,
}

/// Многострочный редактор (нетлист схемы, исходник, конфигурация).
#[derive(Clone, Debug)]
pub struct TextField {
    pub id: &'static str,
    pub label: String,
    pub text: String,
    pub rows: usize,
}

/// Кнопка действия (пересобрать прошивку, внести отказ, сбросить МК).
#[derive(Clone, Debug)]
pub struct Action {
    pub id: &'static str,
    pub label: String,
    pub hint: String,
    /// Действие «включено» (для переключателей отказов — подсветка).
    pub active: bool,
}

impl Action {
    pub fn new(id: &'static str, label: &str, hint: &str, active: bool) -> Self {
        Self {
            id,
            label: label.to_string(),
            hint: hint.to_string(),
            active,
        }
    }
}

/// Поле формы редактирования (конструктор). Окно рисует форму, не зная, что она описывает.
#[derive(Clone, Debug, PartialEq)]
pub enum FormWidget {
    /// Заголовок сворачиваемой группы: всё после него до следующего заголовка.
    Heading,
    /// Строка только для чтения.
    Info(String),
    Text(String),
    /// Редактирование, которое сразу снимает устаревший результат.
    LiveText(String),
    Number {
        value: f64,
        min: f64,
        max: f64,
        unit: &'static str,
        decimals: usize,
    },
    Choice {
        options: Vec<String>,
        selected: usize,
    },
    Toggle(bool),
    Button {
        hint: String,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct FormItem {
    pub id: String,
    pub label: String,
    pub widget: FormWidget,
}

impl FormItem {
    pub fn new(id: impl Into<String>, label: impl Into<String>, widget: FormWidget) -> Self {
        Self {
            id: id.into(),
            label: label.into(),
            widget,
        }
    }
}

/// Что пользователь изменил в форме.
#[derive(Clone, Debug, PartialEq)]
pub enum FormValue {
    Text(String),
    Number(f64),
    Choice(usize),
    Toggle(bool),
    Pressed,
}

/// Готовая к показу геометрия в мировых координатах движка (Z вверх).
#[derive(Clone, Copy, Debug)]
pub struct VisualInstance {
    pub shape: Shape,
    pub pose: Pose,
    pub color: [f32; 4],
}

/// Подсказки поверх сцены (стрелки сил, кольца моментов, «призрак» аналитики).
#[derive(Clone, Copy, Debug)]
pub enum Overlay {
    Arrow {
        from: Vec3,
        to: Vec3,
        color: [f32; 4],
    },
    Line {
        from: Vec3,
        to: Vec3,
        color: [f32; 4],
    },
    Ring {
        center: Vec3,
        normal: Vec3,
        radius: f64,
        color: [f32; 4],
    },
    WireSphere {
        center: Vec3,
        radius: f64,
        color: [f32; 4],
    },
}

pub trait Scenario: Send + Sync {
    fn id(&self) -> &'static str;
    fn title(&self) -> String;
    fn summary(&self) -> String;

    fn params(&self) -> Vec<Param>;
    fn set_param(&mut self, id: &str, value: f64);
    fn presets(&self) -> Vec<String> {
        Vec::new()
    }
    fn apply_preset(&mut self, _index: usize) {}
    fn choices(&self) -> Vec<Choice> {
        Vec::new()
    }
    fn set_choice(&mut self, _id: &str, _index: usize) {}

    fn reset(&mut self);
    /// Продвинуть симуляцию на `dt_real` секунд (внутри — фиксированные подшаги).
    fn advance(&mut self, dt_real: f64);
    fn time(&self) -> f64;
    /// Получить готовый фоновый результат независимо от паузы симуляции.
    fn poll_background(&mut self) {}

    fn visuals(&self) -> Vec<VisualInstance>;
    /// Меняется, когда меняется набор/размер геометрии (например, масса гантели).
    fn visual_epoch(&self) -> u64;
    fn overlays(&self) -> Vec<Overlay> {
        Vec::new()
    }

    fn readouts(&self) -> Vec<Readout>;
    fn comparisons(&self) -> Vec<Comparison>;
    fn notes(&self) -> Vec<Note> {
        Vec::new()
    }
    fn plot(&self) -> Option<Plot> {
        None
    }
    /// Все графики сценария (по умолчанию — один из [`Scenario::plot`]).
    fn plots(&self) -> Vec<Plot> {
        self.plot().into_iter().collect()
    }

    /// Редакторы текста (нетлист и т.п.); `set_text` вызывается по кнопке «Применить».
    fn text_fields(&self) -> Vec<TextField> {
        Vec::new()
    }
    fn set_text(&mut self, _id: &str, _text: String) {}

    /// Кнопки действий и переключатели отказов.
    fn actions(&self) -> Vec<Action> {
        Vec::new()
    }
    fn run_action(&mut self, _id: &str) {}

    /// Форма редактирования: группы полей, списки, кнопки.
    fn form(&self) -> Vec<FormItem> {
        Vec::new()
    }
    fn set_form(&mut self, _id: &str, _value: FormValue) {}

    /// Журнал: вывод UART прошивки, ошибки разбора, события.
    fn console(&self) -> Vec<String> {
        Vec::new()
    }

    /// Точка, на которую смотреть камере (координаты движка).
    fn focus(&self) -> Vec3;

    /// Откуда смотреть по умолчанию: смещение камеры от `focus` (координаты движка).
    fn view_offset(&self) -> Vec3 {
        Vec3::new(0.95, -1.9, 0.3)
    }

    /// Треугольная сетка для `Shape::Mesh { id }` (кости и т.п.).
    fn mesh_data(&self, _id: u32) -> Option<MeshData> {
        None
    }
}

/// Сетка: вершины (координаты движка, кадр фигуры) и треугольники.
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

/// Геометрия модели в мире для текущих координат `q`.
pub fn model_visuals(model: &Model, q: &DVector<f64>) -> Vec<VisualInstance> {
    let poses = body_poses(model, q);
    let mut out: Vec<VisualInstance> = model
        .world_visuals
        .iter()
        .map(|v| VisualInstance {
            shape: v.shape,
            pose: v.pose,
            color: v.color,
        })
        .collect();
    for (b, pose) in model.bodies.iter().zip(poses.iter()) {
        for v in &b.visuals {
            out.push(VisualInstance {
                shape: v.shape,
                pose: pose.compose(&v.pose),
                color: v.color,
            });
        }
    }
    out
}

/// Накопитель реального времени → целое число фиксированных шагов.
#[derive(Clone, Debug)]
pub struct FixedClock {
    pub dt: f64,
    acc: f64,
}

impl FixedClock {
    pub fn new(dt: f64) -> Self {
        Self { dt, acc: 0.0 }
    }

    pub fn steps(&mut self, dt_real: f64) -> usize {
        self.acc += dt_real.clamp(0.0, 0.1);
        let n = (self.acc / self.dt).floor();
        self.acc -= n * self.dt;
        n as usize
    }

    pub fn reset(&mut self) {
        self.acc = 0.0;
    }
}

/// Скользящая история сигнала (для графиков и усреднения).
#[derive(Clone, Debug, Default)]
pub struct History {
    pub t: Vec<f64>,
    pub y: Vec<Vec<f64>>,
    keep: f64,
}

impl History {
    pub fn new(channels: usize, keep_seconds: f64) -> Self {
        Self {
            t: Vec::new(),
            y: vec![Vec::new(); channels],
            keep: keep_seconds,
        }
    }

    pub fn push(&mut self, t: f64, values: &[f64]) {
        self.t.push(t);
        for (c, v) in self.y.iter_mut().zip(values) {
            c.push(*v);
        }
        let cutoff = t - self.keep;
        let drop = self.t.iter().take_while(|&&x| x < cutoff).count();
        if drop > 512 {
            self.t.drain(..drop);
            for c in &mut self.y {
                c.drain(..drop);
            }
        }
    }

    pub fn clear(&mut self) {
        self.t.clear();
        for c in &mut self.y {
            c.clear();
        }
    }

    /// Среднее канала за последние `window` секунд.
    pub fn mean(&self, channel: usize, window: f64) -> Option<f64> {
        let t_end = *self.t.last()?;
        let mut s = 0.0;
        let mut n = 0usize;
        for (t, y) in self.t.iter().zip(self.y[channel].iter()).rev() {
            if *t < t_end - window {
                break;
            }
            s += y;
            n += 1;
        }
        (n > 0).then(|| s / n as f64)
    }

    /// Максимум |y| за последние `window` секунд.
    pub fn max_abs(&self, channel: usize, window: f64) -> Option<f64> {
        let t_end = *self.t.last()?;
        self.t
            .iter()
            .zip(self.y[channel].iter())
            .rev()
            .take_while(|(t, _)| **t >= t_end - window)
            .map(|(_, y)| y.abs())
            .reduce(f64::max)
    }

    /// Точки для графика за последние `window` секунд (прореживание до ~`max_points`).
    pub fn points(&self, channel: usize, window: f64, max_points: usize) -> Vec<[f64; 2]> {
        let Some(&t_end) = self.t.last() else {
            return Vec::new();
        };
        let start = self
            .t
            .iter()
            .position(|t| *t >= t_end - window)
            .unwrap_or(0);
        let n = self.t.len() - start;
        let stride = (n / max_points.max(1)).max(1);
        (start..self.t.len())
            .step_by(stride)
            .map(|i| [self.t[i], self.y[channel][i]])
            .collect()
    }
}

pub const C_SKIN: [f32; 4] = [0.93, 0.76, 0.64, 1.0];
pub const C_MANNEQUIN: [f32; 4] = [0.62, 0.66, 0.72, 1.0];
pub const C_EXO: [f32; 4] = [0.16, 0.18, 0.22, 1.0];
pub const C_EXO_CUFF: [f32; 4] = [0.08, 0.55, 0.6, 1.0];
pub const C_MOTOR: [f32; 4] = [1.0, 0.55, 0.1, 1.0];
pub const C_STEEL: [f32; 4] = [0.72, 0.74, 0.78, 1.0];
pub const C_IRON: [f32; 4] = [0.12, 0.12, 0.13, 1.0];
pub const C_GREEN: [f32; 4] = [0.2, 0.85, 0.35, 1.0];
pub const C_YELLOW: [f32; 4] = [1.0, 0.8, 0.15, 1.0];
pub const C_RED: [f32; 4] = [1.0, 0.25, 0.2, 1.0];
pub const C_BLUE: [f32; 4] = [0.25, 0.6, 1.0, 1.0];
pub const C_GRAY: [f32; 4] = [0.6, 0.6, 0.6, 1.0];
