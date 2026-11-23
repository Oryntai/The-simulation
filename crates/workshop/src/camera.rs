//! Свободная камера: WASD — ходить, Q/E — вниз/вверх, правая кнопка мыши — осмотреться,
//! Shift — быстрее, колесо — скорость, F — посмотреть на устройство, R — вернуть вид.

use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, ClearColorConfig, RenderTarget};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::render::render_resource::BlendState;
use bevy_egui::{EguiContexts, PrimaryEguiContext};

use crate::{Workshop, convert};

#[derive(Component)]
pub struct FlyCam {
    pub yaw: f32,
    pub pitch: f32,
    pub speed: f32,
}

#[derive(Component)]
pub struct SceneCamera;

/// Вид по умолчанию для сценария: камера в `focus + offset`, смотрит на `focus`.
pub fn scenario_view(ws: &Workshop) -> (Vec3, f32, f32) {
    let sc = ws.current();
    default_view(convert::pos(&sc.focus()), convert::dir(&sc.view_offset()))
}

pub fn default_view(focus: Vec3, offset: Vec3) -> (Vec3, f32, f32) {
    let eye = focus + offset;
    let d = (focus - eye).normalize();
    let yaw = (-d.x).atan2(-d.z);
    let pitch = d.y.asin();
    (eye, yaw, pitch)
}

/// Съёмка без экрана: 3D и интерфейс рисуются в текстуру на GPU, а не в окно.
/// Работает и когда экран заблокирован или дисплей спит (окну macOS тогда кадров не даёт).
#[derive(Resource, Clone)]
pub struct OffscreenTarget {
    pub image: Handle<Image>,
    pub size: UVec2,
}

/// Размер кадра в режиме съёмки (как окно 1600×1000 на Retina) и масштаб интерфейса.
pub const OFFSCREEN_SIZE: UVec2 = UVec2::new(3200, 2000);
pub const OFFSCREEN_UI_ZOOM: f32 = 2.0;

pub fn spawn_cameras(
    mut commands: Commands,
    ws: Res<Workshop>,
    shot: Res<crate::AutoShot>,
    mut images: ResMut<Assets<Image>>,
) {
    let (eye, yaw, pitch) = scenario_view(&ws);

    let offscreen = shot.path.as_ref().map(|_| {
        let mut img = Image::new_target_texture(
            OFFSCREEN_SIZE.x,
            OFFSCREEN_SIZE.y,
            bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
            None,
        );
        img.texture_descriptor.usage |= bevy::render::render_resource::TextureUsages::COPY_SRC;
        OffscreenTarget {
            image: images.add(img),
            size: OFFSCREEN_SIZE,
        }
    });

    let mut scene_cam = commands.spawn((
        Camera3d::default(),
        Camera {
            order: 0,
            ..default()
        },
        Transform::from_translation(eye).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            yaw,
            pitch,
            0.0,
        )),
        FlyCam {
            yaw,
            pitch,
            speed: 2.0,
        },
        SceneCamera,
    ));
    if let Some(off) = &offscreen {
        scene_cam.insert(RenderTarget::Image(off.image.clone().into()));
    }
    // Отдельная камера только для интерфейса egui — рисует поверх 3D.
    let mut ui_cam = commands.spawn((
        PrimaryEguiContext,
        Camera2d,
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
    ));
    if let Some(off) = offscreen {
        ui_cam.insert(RenderTarget::Image(off.image.clone().into()));
        commands.insert_resource(off);
    }
}

#[allow(clippy::too_many_arguments)]
pub fn fly_camera(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    ws: Res<Workshop>,
    mut contexts: EguiContexts,
    mut cam: Query<(&mut Transform, &mut FlyCam), With<SceneCamera>>,
    mut last_scenario: Local<Option<usize>>,
) {
    let Ok((mut tf, mut fc)) = cam.single_mut() else {
        return;
    };
    // Сменили сценарий — переносим камеру к нему.
    if *last_scenario != Some(ws.active) {
        if last_scenario.is_some() {
            let (eye, yaw, pitch) = scenario_view(&ws);
            tf.translation = eye;
            fc.yaw = yaw;
            fc.pitch = pitch;
        }
        *last_scenario = Some(ws.active);
    }
    let (ui_pointer, ui_keys) = match contexts.ctx_mut() {
        Ok(ctx) => (
            ctx.egui_wants_pointer_input() || ctx.is_pointer_over_egui(),
            ctx.egui_wants_keyboard_input(),
        ),
        Err(_) => (false, false),
    };

    let input = FlyInput {
        keys: &keys,
        buttons: &buttons,
        motion: motion.delta,
        scroll: scroll.delta.y,
        ui_pointer,
        ui_keys,
        dt: time.delta_secs(),
        home: scenario_view(&ws),
        focus: convert::pos(&ws.current().focus()),
    };
    steer(&mut tf, &mut fc, &input);
}

/// Ввод одного кадра для камеры.
pub struct FlyInput<'a> {
    pub keys: &'a ButtonInput<KeyCode>,
    pub buttons: &'a ButtonInput<MouseButton>,
    pub motion: Vec2,
    pub scroll: f32,
    /// Указатель над интерфейсом: колесо и обзор не трогают сцену.
    pub ui_pointer: bool,
    /// Клавиатура занята полем ввода: WASD не двигает камеру.
    pub ui_keys: bool,
    pub dt: f32,
    /// Вид по умолчанию: положение, рыскание, тангаж.
    pub home: (Vec3, f32, f32),
    pub focus: Vec3,
}

/// Один шаг камеры.
pub fn steer(tf: &mut Transform, fc: &mut FlyCam, i: &FlyInput) {
    if !i.ui_pointer || i.buttons.pressed(MouseButton::Right) {
        if i.buttons.pressed(MouseButton::Right) {
            let sens = 0.0035;
            fc.yaw -= i.motion.x * sens;
            fc.pitch = (fc.pitch - i.motion.y * sens).clamp(-1.54, 1.54);
        }
        if !i.ui_pointer && i.scroll != 0.0 {
            fc.speed = (fc.speed * (1.0 + 0.1 * i.scroll.signum())).clamp(0.2, 20.0);
        }
    }

    if !i.ui_keys {
        let rot = Quat::from_euler(EulerRot::YXZ, fc.yaw, fc.pitch, 0.0);
        let forward = rot * Vec3::NEG_Z;
        let right = rot * Vec3::X;
        let mut m = Vec3::ZERO;
        if i.keys.pressed(KeyCode::KeyW) {
            m += forward;
        }
        if i.keys.pressed(KeyCode::KeyS) {
            m -= forward;
        }
        if i.keys.pressed(KeyCode::KeyD) {
            m += right;
        }
        if i.keys.pressed(KeyCode::KeyA) {
            m -= right;
        }
        if i.keys.pressed(KeyCode::KeyE) || i.keys.pressed(KeyCode::Space) {
            m += Vec3::Y;
        }
        if i.keys.pressed(KeyCode::KeyQ) {
            m -= Vec3::Y;
        }
        let boost = if i.keys.pressed(KeyCode::ShiftLeft) || i.keys.pressed(KeyCode::ShiftRight) {
            3.0
        } else {
            1.0
        };
        if m != Vec3::ZERO {
            tf.translation += m.normalize() * fc.speed * boost * i.dt;
        }
        let focus = i.focus;
        if i.keys.just_pressed(KeyCode::KeyR) {
            let (eye, yaw, pitch) = i.home;
            tf.translation = eye;
            fc.yaw = yaw;
            fc.pitch = pitch;
        }
        if i.keys.just_pressed(KeyCode::KeyF) {
            let d = (focus - tf.translation).normalize();
            fc.yaw = (-d.x).atan2(-d.z);
            fc.pitch = d.y.asin();
        }
    }
    tf.translation.y = tf.translation.y.max(0.05);
    tf.rotation = Quat::from_euler(EulerRot::YXZ, fc.yaw, fc.pitch, 0.0);
}
