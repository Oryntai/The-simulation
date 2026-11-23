//! Перевод координат: движок — Z вверх (как MuJoCo/Drake), Bevy — Y вверх.
//! (x, y, z)_движок → (x, z, −y)_Bevy — это поворот на −90° вокруг X.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use sim_core::{Pose, Shape};

pub fn pos(p: &sim_core::Vec3) -> Vec3 {
    Vec3::new(p.x as f32, p.z as f32, -p.y as f32)
}

pub fn dir(d: &sim_core::Vec3) -> Vec3 {
    pos(d)
}

fn axes() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

pub fn quat(q: &nalgebra::UnitQuaternion<f64>) -> Quat {
    Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
}

/// Transform для примитива Bevy, изображающего `shape` в позе движка `pose`.
/// У цилиндра и капсулы Bevy ось — локальная Y, у нас — локальная z.
pub fn shape_transform(shape: &Shape, pose: &Pose) -> Transform {
    let r = axes() * quat(&pose.rot);
    let rotation = match shape {
        Shape::Capsule { .. } | Shape::Cylinder { .. } | Shape::Tube { .. } => {
            r * Quat::from_rotation_x(FRAC_PI_2)
        }
        _ => r,
    };
    // Эллипсоид и трубка — единичные сетки, размер задаётся масштабом (трубка: ось — Y примитива).
    let scale = match *shape {
        Shape::Ellipsoid { radii } => Vec3::new(radii.x as f32, radii.y as f32, radii.z as f32),
        Shape::Tube { radius, length } => {
            let (r, l) = ((radius as f32).max(1e-6), (length as f32).max(1e-6));
            Vec3::new(r, l, r)
        }
        _ => Vec3::ONE,
    };
    Transform {
        translation: pos(&pose.pos),
        rotation,
        scale,
    }
}

/// Сетка для примитива. `Shape::Mesh` сюда не попадает — её строит `mesh_from_data`.
pub fn shape_mesh(shape: &Shape) -> Mesh {
    match *shape {
        Shape::Ellipsoid { .. } => Sphere::new(1.0).mesh().uv(24, 14),
        Shape::Tube { .. } => Cylinder::new(1.0, 1.0).mesh().resolution(10).into(),
        Shape::Mesh { .. } => Sphere::new(0.01).mesh().uv(6, 4),
        Shape::Sphere { radius } => Sphere::new(radius as f32).mesh().uv(32, 18),
        Shape::Capsule { radius, length } => Capsule3d::new(radius as f32, length as f32).into(),
        Shape::Cylinder { radius, length } => Cylinder::new(radius as f32, length as f32).into(),
        Shape::Box { half } => Cuboid::new(
            2.0 * half.x as f32,
            2.0 * half.y as f32,
            2.0 * half.z as f32,
        )
        .into(),
    }
}

pub fn color(c: &[f32; 4]) -> Color {
    Color::srgba(c[0], c[1], c[2], c[3])
}

/// Сетка из вершин и треугольников сценария (кости из MuJoCo-моделей), с гладкими нормалями.
pub fn mesh_from_data(positions: &[[f32; 3]], indices: &[u32]) -> Mesh {
    use bevy::asset::RenderAssetUsages;
    use bevy::mesh::{Indices, PrimitiveTopology};
    // Координаты сценария — Z вверх; переводим вершины сразу в оси примитива Bevy не нужно:
    // поза меша задаётся тем же поворотом осей, что и у остальных фигур.
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.to_vec());
    mesh.insert_indices(Indices::U32(indices.to_vec()));
    mesh.compute_smooth_normals();
    mesh
}
