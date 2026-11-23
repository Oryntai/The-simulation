//! 3D-сцена: пол, свет, геометрия сценария и подсказки (силы, моменты, «призраки» формул).

use bevy::light::CascadeShadowConfigBuilder;
use bevy::prelude::*;
use sim_core::Shape;
use sim_scenarios::Overlay;

use crate::{Workshop, convert};

/// Сущность, нарисованная по геометрии сценария.
#[derive(Component)]
pub struct SimVisual;

#[derive(Resource, Default)]
pub struct SpawnedVisuals {
    pub scenario: Option<usize>,
    pub epoch: u64,
    pub entities: Vec<Entity>,
    /// Сетки костей сценария по id и единичные примитивы (эллипсоид, трубка) — переиспользуем.
    mesh_cache: std::collections::HashMap<(u8, u32), Handle<Mesh>>,
    /// Материалы по цвету — сотни мышц делят один материал.
    material_cache: std::collections::HashMap<[u8; 4], Handle<StandardMaterial>>,
    /// Текущий цвет каждой сущности — чтобы менять материал, когда сценарий перекрашивает фигуру.
    colors: Vec<[u8; 4]>,
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.init_resource::<SpawnedVisuals>();
    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.85, 0.9, 1.0),
        brightness: 350.0,
        affects_lightmapped_meshes: true,
    });

    // Пол мастерской.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(40.0, 40.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.21, 0.23),
            perceptual_roughness: 0.92,
            ..default()
        })),
    ));
    // Подиум под устройством.
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(1.2, 0.02))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.27, 0.28, 0.31),
            perceptual_roughness: 0.8,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.01, 0.0),
    ));

    // Солнце с тенями.
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(3.0, 6.0, 4.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
        CascadeShadowConfigBuilder {
            maximum_distance: 14.0,
            first_cascade_far_bound: 3.5,
            ..default()
        }
        .build(),
    ));
    // Мягкий заполняющий свет с другой стороны.
    commands.spawn((
        PointLight {
            intensity: 400_000.0,
            range: 30.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_xyz(-3.0, 3.0, -2.5),
    ));
}

pub fn advance_simulation(time: Res<Time>, mut ws: ResMut<Workshop>) {
    ws.clock += time.delta_secs_f64();
    let now = ws.clock;
    let due: Vec<String> = ws
        .scripted
        .iter()
        .filter(|(t, _)| *t <= now)
        .map(|(_, id)| id.clone())
        .collect();
    if !due.is_empty() {
        ws.scripted.retain(|(t, _)| *t > now);
        for id in due {
            ws.current_mut().run_action(&id);
        }
    }
    if ws.paused {
        return;
    }
    let dt = time.delta_secs_f64() * ws.speed;
    ws.current_mut().advance(dt);
}

pub fn sync_visuals(
    mut commands: Commands,
    ws: Res<Workshop>,
    mut spawned: ResMut<SpawnedVisuals>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut transforms: Query<(&mut Transform, &mut MeshMaterial3d<StandardMaterial>), With<SimVisual>>,
) {
    let sc = ws.current();
    let visuals = sc.visuals();
    let stale = spawned.scenario != Some(ws.active)
        || spawned.epoch != sc.visual_epoch()
        || spawned.entities.len() != visuals.len();
    if stale {
        for e in spawned.entities.drain(..) {
            commands.entity(e).despawn();
        }
        spawned.colors.clear();
        // Сетки моделей принадлежат сценарию и его версии геометрии — старые выбрасываем.
        spawned.mesh_cache.retain(|(kind, _), _| *kind != 0);
        for v in &visuals {
            let mesh = match v.shape {
                Shape::Mesh { id } => spawned
                    .mesh_cache
                    .entry((0, id))
                    .or_insert_with(|| match sc.mesh_data(id) {
                        Some(md) => meshes.add(convert::mesh_from_data(&md.positions, &md.indices)),
                        None => meshes.add(convert::shape_mesh(&v.shape)),
                    })
                    .clone(),
                Shape::Ellipsoid { .. } => spawned
                    .mesh_cache
                    .entry((1, 0))
                    .or_insert_with(|| meshes.add(convert::shape_mesh(&v.shape)))
                    .clone(),
                Shape::Tube { .. } => spawned
                    .mesh_cache
                    .entry((2, 0))
                    .or_insert_with(|| meshes.add(convert::shape_mesh(&v.shape)))
                    .clone(),
                _ => meshes.add(convert::shape_mesh(&v.shape)),
            };
            let key = v.color.map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8);
            let material = spawned
                .material_cache
                .entry(key)
                .or_insert_with(|| materials.add(material_for(&v.color)))
                .clone();
            let e = commands
                .spawn((
                    Mesh3d(mesh),
                    MeshMaterial3d(material),
                    convert::shape_transform(&v.shape, &v.pose),
                    SimVisual,
                ))
                .id();
            spawned.entities.push(e);
            spawned.colors.push(key);
        }
        spawned.scenario = Some(ws.active);
        spawned.epoch = sc.visual_epoch();
        return;
    }
    let spawned = &mut *spawned;
    for (k, (e, v)) in spawned.entities.iter().zip(visuals.iter()).enumerate() {
        if let Ok((mut tf, mut mat)) = transforms.get_mut(*e) {
            *tf = convert::shape_transform(&v.shape, &v.pose);
            let key = v.color.map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8);
            if spawned.colors[k] != key {
                let handle = spawned
                    .material_cache
                    .entry(key)
                    .or_insert_with(|| materials.add(material_for(&v.color)))
                    .clone();
                *mat = MeshMaterial3d(handle);
                spawned.colors[k] = key;
            }
        }
    }
}

/// Материал по цвету: полупрозрачное (вода, поле) — со смешиванием.
fn material_for(color: &[f32; 4]) -> StandardMaterial {
    StandardMaterial {
        base_color: convert::color(color),
        perceptual_roughness: 0.55,
        metallic: 0.05,
        alpha_mode: if color[3] < 0.99 {
            AlphaMode::Blend
        } else {
            AlphaMode::Opaque
        },
        ..default()
    }
}

pub fn draw_overlays(ws: Res<Workshop>, mut gizmos: Gizmos) {
    // Сетка пола: шаг 0.5 м.
    gizmos.grid(
        Isometry3d::new(
            Vec3::new(0.0, 0.002, 0.0),
            Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        ),
        UVec2::new(24, 24),
        Vec2::splat(0.5),
        Color::srgba(0.55, 0.6, 0.7, 0.18),
    );
    // Оси мира у подножия: X — красная, Y движка (влево) — зелёная, Z движка (вверх) — синяя.
    let o = Vec3::new(-1.0, 0.003, 1.0);
    gizmos.arrow(o, o + Vec3::X * 0.3, Color::srgb(0.9, 0.25, 0.2));
    gizmos.arrow(o, o + Vec3::NEG_Z * 0.3, Color::srgb(0.3, 0.85, 0.3));
    gizmos.arrow(o, o + Vec3::Y * 0.3, Color::srgb(0.3, 0.5, 1.0));

    for ov in ws.current().overlays() {
        match ov {
            Overlay::Arrow { from, to, color } => {
                gizmos.arrow(
                    convert::pos(&from),
                    convert::pos(&to),
                    convert::color(&color),
                );
            }
            Overlay::Line { from, to, color } => {
                gizmos.line(
                    convert::pos(&from),
                    convert::pos(&to),
                    convert::color(&color),
                );
            }
            Overlay::Ring {
                center,
                normal,
                radius,
                color,
            } => {
                let n = convert::dir(&normal).normalize();
                let iso =
                    Isometry3d::new(convert::pos(&center), Quat::from_rotation_arc(Vec3::Z, n));
                gizmos
                    .circle(iso, radius as f32, convert::color(&color))
                    .resolution(48);
                gizmos
                    .circle(iso, radius as f32 * 0.92, convert::color(&color))
                    .resolution(48);
            }
            Overlay::WireSphere {
                center,
                radius,
                color,
            } => {
                gizmos
                    .sphere(
                        Isometry3d::from_translation(convert::pos(&center)),
                        radius as f32,
                        convert::color(&color),
                    )
                    .resolution(24);
            }
        }
    }
}
