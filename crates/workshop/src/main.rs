//! Мастерская устройств — настольное 3D-приложение: устройство сначала проверяется
//! в симуляции, потом собирается в железе.
//!
//! Запуск: `cargo run -p workshop` (из папки `sim/`).
//! Флаги (для скриптов и проверок без рук):
//!   --scenario <id>      exo_arm | anatomy | beam | pendulum | free_fall
//!   --preset <n>         номер готового варианта сценария (0, 1, …)
//!   --choice <id>=<n>    выбрать вариант в списке сценария (например, model=1)
//!   --screenshot <файл>  отрисовать кадр в текстуру на GPU (без экрана), сохранить PNG и выйти
//!   --after <сек>        когда снимать (по умолчанию 6 с)

mod camera;
mod convert;
mod scene;
mod ui;
#[cfg(test)]
mod ui_tests;

use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::{EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass};
use sim_scenarios::Scenario;
use sim_scenarios::checks::CheckGroup;

/// Все сценарии и что сейчас открыто.
#[derive(Resource)]
pub struct Workshop {
    pub scenarios: Vec<Box<dyn Scenario>>,
    pub active: usize,
    pub paused: bool,
    pub speed: f64,
    /// Черновики редакторов текста: (сценарий, поле) → текст до «Применить».
    pub drafts: std::collections::HashMap<(usize, &'static str), String>,
    /// Черновики текстовых полей формы конструктора: id поля → текст до потери фокуса.
    pub form_drafts: std::collections::HashMap<String, String>,
    /// Кнопки сценария, которые нажать по сценарию запуска: (через сколько секунд, id).
    pub scripted: Vec<(f64, String)>,
    /// Секунды с запуска (для `scripted`).
    pub clock: f64,
}

impl Workshop {
    pub fn current(&self) -> &dyn Scenario {
        self.scenarios[self.active].as_ref()
    }

    pub fn current_mut(&mut self) -> &mut dyn Scenario {
        self.scenarios[self.active].as_mut()
    }
}

/// Эталонные проверки физики — считаются в фоне при старте.
#[derive(Resource, Clone, Default)]
pub struct ReferenceChecks(pub Arc<Mutex<Option<Vec<CheckGroup>>>>);

/// Снимок окна по флагу `--screenshot`.
#[derive(Resource, Default)]
pub struct AutoShot {
    pub path: Option<String>,
    pub after: f32,
    pub requested: bool,
    pub frames_after: u32,
}

struct Args {
    scenario: Option<String>,
    preset: Option<usize>,
    choices: Vec<(String, usize)>,
    actions: Vec<(f64, String)>,
    screenshot: Option<String>,
    after: f32,
}

fn parse_args() -> Args {
    let mut a = Args {
        scenario: None,
        preset: None,
        choices: Vec::new(),
        actions: Vec::new(),
        screenshot: None,
        after: 6.0,
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        match k.as_str() {
            "--scenario" => a.scenario = it.next(),
            "--preset" => a.preset = it.next().and_then(|v| v.parse().ok()),
            "--choice" => {
                let kv = it.next().unwrap_or_default();
                if let Some((k, v)) = kv.split_once('=')
                    && let Ok(i) = v.parse()
                {
                    a.choices.push((k.to_string(), i));
                }
            }
            "--action" => {
                // id или id@секунды: нажать кнопку сценария через столько секунд.
                let v = it.next().unwrap_or_default();
                let (id, t) = match v.split_once('@') {
                    Some((id, t)) => (id.to_string(), t.parse().unwrap_or(0.5)),
                    None => (v, 0.5),
                };
                a.actions.push((t, id));
            }
            "--screenshot" => a.screenshot = it.next(),
            "--after" => a.after = it.next().and_then(|v| v.parse().ok()).unwrap_or(6.0),
            "-h" | "--help" => {
                println!(
                    "workshop [--scenario id] [--choice id=N] [--preset N] [--action id[@сек]] [--screenshot файл.png] [--after сек]"
                );
                std::process::exit(0);
            }
            other => eprintln!("неизвестный флаг: {other}"),
        }
    }
    a
}

fn main() -> AppExit {
    let args = parse_args();

    let mut scenarios = sim_scenarios::catalog();
    let active = args
        .scenario
        .as_deref()
        .and_then(|id| scenarios.iter().position(|s| s.id() == id))
        .unwrap_or(0);
    for (id, i) in &args.choices {
        scenarios[active].set_choice(id, *i);
    }
    if let Some(p) = args.preset {
        scenarios[active].apply_preset(p);
    }

    let checks = ReferenceChecks::default();
    {
        let slot = checks.0.clone();
        std::thread::spawn(move || {
            let result = sim_scenarios::checks::all_reference_checks();
            *slot.lock().unwrap() = Some(result);
        });
    }

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.085, 0.09, 0.105)))
        .insert_resource(Workshop {
            scenarios,
            active,
            paused: false,
            speed: 1.0,
            drafts: Default::default(),
            form_drafts: Default::default(),
            scripted: args.actions.clone(),
            clock: 0.0,
        })
        .insert_resource(checks)
        .insert_resource(AutoShot {
            path: args.screenshot,
            after: args.after,
            ..default()
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Мастерская устройств — симуляция до сборки".into(),
                resolution: WindowResolution::new(1480, 920),
                focused: true,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_systems(
            Startup,
            (
                disable_auto_egui_context,
                scene::setup,
                camera::spawn_cameras,
            ),
        )
        .add_systems(
            Update,
            (
                scene::advance_simulation,
                scene::sync_visuals,
                scene::draw_overlays,
                camera::fly_camera,
                auto_screenshot,
            )
                .chain(),
        )
        .add_systems(EguiPrimaryContextPass, ui::ui_system)
        .run()
}

fn disable_auto_egui_context(mut settings: ResMut<EguiGlobalSettings>) {
    // egui рисуется отдельной камерой поверх 3D, чтобы 3D-камере можно было задать вьюпорт.
    settings.auto_create_primary_context = false;
}

fn auto_screenshot(
    mut commands: Commands,
    time: Res<Time>,
    mut shot: ResMut<AutoShot>,
    offscreen: Option<Res<camera::OffscreenTarget>>,
    mut exit: MessageWriter<AppExit>,
) {
    let Some(path) = shot.path.clone() else {
        return;
    };
    if !shot.requested {
        if time.elapsed_secs() >= shot.after {
            use bevy::render::view::screenshot::{Screenshot, save_to_disk};
            let shot_entity = match &offscreen {
                Some(off) => Screenshot::image(off.image.clone()),
                None => Screenshot::primary_window(),
            };
            commands.spawn(shot_entity).observe(save_to_disk(path));
            shot.requested = true;
        }
        return;
    }
    shot.frames_after += 1;
    let saved = std::fs::metadata(&path)
        .map(|m| m.len() > 0)
        .unwrap_or(false);
    if (saved && shot.frames_after > 10) || shot.frames_after > 600 {
        exit.write(AppExit::Success);
    }
}
