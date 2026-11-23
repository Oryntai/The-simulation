//! Настоящие события мыши и клавиатуры через интерфейс, без окна и без экрана.

use bevy::prelude::*;
use bevy_egui::egui::{self, Event, Modifiers, PointerButton, Pos2, RawInput, Rect as UiRect, pos2, vec2};
use sim_scenarios::FormWidget;

use crate::Workshop;
use crate::camera::{FlyCam, FlyInput, steer};
use crate::ui::left_panel;

struct Node {
    label: String,
    value: String,
    number: Option<f64>,
    rect: UiRect,
}

struct Ui {
    ctx: egui::Context,
    ws: Workshop,
    events: Vec<Event>,
    nodes: Vec<Node>,
    time: f64,
}

impl Ui {
    fn new() -> Self {
        let ctx = egui::Context::default();
        ctx.enable_accesskit();
        let mut ui = Self {
            ctx,
            ws: Workshop {
                scenarios: sim_scenarios::catalog(),
                active: 0,
                paused: false,
                speed: 1.0,
                drafts: Default::default(),
                form_drafts: Default::default(),
                scripted: Vec::new(),
                clock: 0.0,
            },
            events: Vec::new(),
            nodes: Vec::new(),
            time: 0.0,
        };
        ui.frames(3);
        ui
    }

    fn frame(&mut self) {
        self.time += 1.0 / 60.0;
        let input = RawInput {
            // Окно высокое, чтобы вся форма помещалась без прокрутки.
            screen_rect: Some(UiRect::from_min_size(Pos2::ZERO, vec2(700.0, 7000.0))),
            time: Some(self.time),
            predicted_dt: 1.0 / 60.0,
            events: std::mem::take(&mut self.events),
            ..Default::default()
        };
        let ws = &mut self.ws;
        let mut out = self.ctx.run_ui(input, |ui| left_panel(ui, ws));
        // Текстуры здесь некуда отправлять: окна нет.
        out.textures_delta.clear();
        if let Some(update) = out.platform_output.accesskit_update.take() {
            self.nodes = update
                .nodes
                .iter()
                .filter_map(|(_, n)| {
                    let b = n.bounds()?;
                    Some(Node {
                        label: n.label().unwrap_or_default().to_string(),
                        value: n.value().unwrap_or_default().to_string(),
                        number: n.numeric_value(),
                        rect: UiRect::from_min_max(
                            pos2(b.x0 as f32, b.y0 as f32),
                            pos2(b.x1 as f32, b.y1 as f32),
                        ),
                    })
                })
                .collect();
        }
    }

    fn frames(&mut self, n: usize) {
        for _ in 0..n {
            self.frame();
        }
    }

    fn at(&self, label: &str) -> Pos2 {
        self.nodes
            .iter()
            .find(|n| n.label == label || n.label.ends_with(label))
            .unwrap_or_else(|| {
                let all: Vec<_> = self.nodes.iter().map(|n| n.label.as_str()).collect();
                panic!("нет элемента «{label}»: {all:?}")
            })
            .rect
            .center()
    }

    fn press(&mut self, p: Pos2, pressed: bool) {
        self.events.push(Event::PointerButton {
            pos: p,
            button: PointerButton::Primary,
            pressed,
            modifiers: Modifiers::NONE,
        });
    }

    fn click_at(&mut self, p: Pos2) {
        self.events.push(Event::PointerMoved(p));
        self.frame();
        self.press(p, true);
        self.frame();
        self.press(p, false);
        self.frames(3);
    }

    fn click(&mut self, label: &str) {
        let p = self.at(label);
        self.click_at(p);
    }

    fn drag(&mut self, from: Pos2, dx: f32) {
        self.events.push(Event::PointerMoved(from));
        self.frame();
        self.press(from, true);
        self.frame();
        for step in 1..=5 {
            self.events
                .push(Event::PointerMoved(from + vec2(dx * step as f32 / 5.0, 0.0)));
            self.frame();
        }
        self.press(from + vec2(dx, 0.0), false);
        self.frames(3);
    }

    fn widget(&self, id: &str) -> FormWidget {
        self.ws
            .current()
            .form()
            .into_iter()
            .find(|f| f.id == id)
            .unwrap_or_else(|| panic!("нет поля {id}"))
            .widget
    }

    fn checked(&mut self) -> bool {
        !self.ws.current().comparisons().is_empty()
    }

    fn run_check(&mut self) {
        self.click("Проверить устройство");
        let end = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while !self.checked() {
            assert!(std::time::Instant::now() < end, "проверка не завершилась");
            std::thread::yield_now();
            self.frame();
        }
    }

    fn notes(&self) -> String {
        self.ws
            .current()
            .notes()
            .into_iter()
            .map(|n| n.text)
            .collect::<Vec<_>>()
            .join("\n")
    }
}

#[test]
fn constructor_works_through_real_input_events() {
    let mut ui = Ui::new();
    assert_eq!(ui.ws.current().id(), "constructor");

    // Кнопка проверки выносит вердикт.
    ui.run_check();
    assert!(ui.notes().contains("НЕ ДОКАЗАНО"), "{}", ui.notes());

    // Галочка меняет устройство и снимает вердикт.
    let FormWidget::Toggle(before) = ui.widget("use.long") else {
        panic!("поле носки")
    };
    ui.click("Носится долго");
    assert_eq!(ui.widget("use.long"), FormWidget::Toggle(!before));
    assert!(!ui.checked(), "вердикт пережил правку галочкой");

    // Перетаскивание числа мышью.
    ui.run_check();
    let FormWidget::Number { value: air, .. } = ui.widget("use.ambient") else {
        panic!("поле воздуха")
    };
    let field = ui
        .nodes
        .iter()
        .find(|n| n.number == Some(air) && n.value.contains("°C"))
        .expect("числовое поле воздуха")
        .rect
        .center();
    ui.drag(field, 20.0);
    let FormWidget::Number { value: moved, .. } = ui.widget("use.ambient") else {
        panic!("поле воздуха")
    };
    assert!(moved > air, "{air} → {moved}");
    assert!(!ui.checked(), "вердикт пережил правку числа");

    // Ввод с клавиатуры в текстовое поле.
    let name = match ui.widget("project.name") {
        FormWidget::Text(t) | FormWidget::LiveText(t) => t,
        other => panic!("{other:?}"),
    };
    let field = ui
        .nodes
        .iter()
        .find(|n| n.value == name)
        .expect("поле названия")
        .rect
        .center();
    ui.click_at(field);
    ui.events.push(Event::Text(" проверка".into()));
    ui.frame();
    ui.events.push(Event::Key {
        key: egui::Key::Enter,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Modifiers::NONE,
    });
    ui.frames(3);
    let typed = match ui.widget("project.name") {
        FormWidget::Text(t) | FormWidget::LiveText(t) => t,
        other => panic!("{other:?}"),
    };
    assert!(typed.contains("проверка"), "{typed}");

    // Выпадающий список.
    let FormWidget::Choice { options, selected } = ui.widget("use.mount") else {
        panic!("поле крепления")
    };
    let current = options[selected].clone();
    let target = options[(selected + 1) % options.len()].clone();
    let combo = ui
        .nodes
        .iter()
        .find(|n| n.value == current || n.label == current)
        .expect("список крепления")
        .rect
        .center();
    ui.click_at(combo);
    ui.click(&target);
    let FormWidget::Choice { selected: now, .. } = ui.widget("use.mount") else {
        panic!("поле крепления")
    };
    assert_ne!(now, selected);

    // Шаблон и переход в другой сценарий.
    ui.click("Экзоскелет колена");
    match ui.widget("project.name") {
        FormWidget::Text(t) | FormWidget::LiveText(t) => assert_eq!(t, "Экзоскелет колена"),
        other => panic!("{other:?}"),
    }
    ui.click("Комната: предметы, вес, удары");
    assert_eq!(ui.ws.current().id(), "room");
}

fn camera() -> (Transform, FlyCam) {
    (
        Transform::from_xyz(0.0, 1.0, 5.0),
        FlyCam {
            yaw: 0.0,
            pitch: 0.0,
            speed: 2.0,
        },
    )
}

fn input<'a>(
    keys: &'a ButtonInput<KeyCode>,
    buttons: &'a ButtonInput<MouseButton>,
) -> FlyInput<'a> {
    FlyInput {
        keys,
        buttons,
        motion: Vec2::ZERO,
        scroll: 0.0,
        ui_pointer: false,
        ui_keys: false,
        dt: 0.5,
        home: (Vec3::new(3.0, 2.0, 1.0), 0.4, -0.2),
        focus: Vec3::new(0.0, 1.0, 0.0),
    }
}

#[test]
fn keyboard_moves_the_camera_unless_a_field_is_focused() {
    let mut keys = ButtonInput::<KeyCode>::default();
    let buttons = ButtonInput::<MouseButton>::default();
    keys.press(KeyCode::KeyW);
    let (mut tf, mut fc) = camera();
    steer(&mut tf, &mut fc, &input(&keys, &buttons));
    assert!((tf.translation.z - 4.0).abs() < 1e-5, "{:?}", tf.translation);

    let (mut tf, mut fc) = camera();
    let typing = FlyInput {
        ui_keys: true,
        ..input(&keys, &buttons)
    };
    steer(&mut tf, &mut fc, &typing);
    assert_eq!(tf.translation, Vec3::new(0.0, 1.0, 5.0));

    let mut down = ButtonInput::<KeyCode>::default();
    down.press(KeyCode::KeyQ);
    let (mut tf, mut fc) = camera();
    tf.translation.y = 0.1;
    steer(&mut tf, &mut fc, &input(&down, &buttons));
    assert!(tf.translation.y >= 0.05, "камера ушла под пол");
}

#[test]
fn mouse_looks_and_wheel_changes_speed_only_over_the_scene() {
    let keys = ButtonInput::<KeyCode>::default();
    let mut buttons = ButtonInput::<MouseButton>::default();
    buttons.press(MouseButton::Right);
    let (mut tf, mut fc) = camera();
    let look = FlyInput {
        motion: Vec2::new(100.0, 50.0),
        ..input(&keys, &buttons)
    };
    steer(&mut tf, &mut fc, &look);
    assert!(fc.yaw < 0.0 && fc.pitch < 0.0, "{} {}", fc.yaw, fc.pitch);

    let idle = ButtonInput::<MouseButton>::default();
    let (mut tf, mut fc) = camera();
    let wheel = FlyInput {
        scroll: 1.0,
        ..input(&keys, &idle)
    };
    steer(&mut tf, &mut fc, &wheel);
    assert!(fc.speed > 2.0);

    let (mut tf, mut fc) = camera();
    let over_panel = FlyInput {
        scroll: 1.0,
        ui_pointer: true,
        ..input(&keys, &idle)
    };
    steer(&mut tf, &mut fc, &over_panel);
    assert_eq!(fc.speed, 2.0);
}

#[test]
fn reset_and_focus_keys_restore_the_view() {
    let mut keys = ButtonInput::<KeyCode>::default();
    let buttons = ButtonInput::<MouseButton>::default();
    keys.press(KeyCode::KeyR);
    let (mut tf, mut fc) = camera();
    steer(&mut tf, &mut fc, &input(&keys, &buttons));
    assert_eq!(tf.translation, Vec3::new(3.0, 2.0, 1.0));
    assert_eq!((fc.yaw, fc.pitch), (0.4, -0.2));

    let mut keys = ButtonInput::<KeyCode>::default();
    keys.press(KeyCode::KeyF);
    let (mut tf, mut fc) = camera();
    fc.yaw = 1.0;
    steer(&mut tf, &mut fc, &input(&keys, &buttons));
    let look = tf.rotation * Vec3::NEG_Z;
    let want = (Vec3::new(0.0, 1.0, 0.0) - Vec3::new(0.0, 1.0, 5.0)).normalize();
    assert!(look.distance(want) < 1e-4, "{look:?}");
}
