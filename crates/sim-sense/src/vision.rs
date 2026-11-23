use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sim_signal::{SignalId, Time};
use sim_world::{Geometry, Shape};

use crate::optics::{Pose, checked};

/// Идентификатор соответствует номеру геометрии MuJoCo. Поля body и body_name
/// связывают несколько геометрий одного тела.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectPose {
    pub id: u32,
    pub name: String,
    pub body: u32,
    pub body_name: String,
    pub pose: Pose,
}

/// Геометрическая истина по центрам пикселей, без шума и закрытия дыр;
/// при скользящем затворе каждая строка взята в середине своей экспозиции.
#[derive(Clone, Debug, PartialEq)]
pub struct FrameTruth {
    pub signal: SignalId,
    pub captured: Time,
    pub available: Time,
    pub width: u32,
    pub height: u32,
    pub depth_m: Arc<[Option<f64>]>,
    pub object_ids: Arc<[Option<u32>]>,
    pub row_times: Arc<[Time]>,
    /// Позы камеры и геометрий в середине временного интервала всего кадра, в мировых осях.
    pub reference_at: Time,
    pub camera_pose: Pose,
    pub objects: Arc<[ObjectPose]>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Detection {
    pub object_id: u32,
    /// Оценка позы геометрии в мировой системе координат.
    pub pose: Pose,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PoseError {
    pub object_id: u32,
    pub translation_m: f64,
    pub rotation_rad: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PerceptionScore {
    pub pose_errors: Vec<PoseError>,
    pub missed: Vec<u32>,
    pub false_positives: Vec<u32>,
}

/// Сопоставление по идентификаторам. Объект из expected учитывается, если занимает
/// не менее min_visible_pixels. Повторное сообщение об объекте считается ложным.
pub fn compare_detections(
    truth: &FrameTruth,
    expected: &[u32],
    detections: &[Detection],
    min_visible_pixels: usize,
) -> Result<PerceptionScore, String> {
    if min_visible_pixels == 0 {
        return Err("порог видимости должен быть не меньше одного пикселя".into());
    }
    let pixels = (truth.width as usize)
        .checked_mul(truth.height as usize)
        .ok_or("размер карты истины вышел за диапазон")?;
    if pixels == 0
        || truth.object_ids.len() != pixels
        || truth.depth_m.len() != pixels
        || truth.row_times.len() != truth.height as usize
    {
        return Err("размеры карт истины не совпадают с размером кадра".into());
    }
    let acquisition = truth.captured..=truth.available;
    if !acquisition.contains(&truth.reference_at)
        || truth.row_times.iter().any(|t| !acquisition.contains(t))
        || !truth.row_times.is_sorted()
    {
        return Err("неверные моменты измерения в истине кадра".into());
    }
    truth.camera_pose.frame()?;
    let mut objects = BTreeMap::new();
    for object in &*truth.objects {
        if objects.insert(object.id, object.pose.frame()?).is_some() {
            return Err(format!("повторный объект {} в истине кадра", object.id));
        }
    }
    let mut targets = BTreeSet::new();
    for &id in expected {
        if !objects.contains_key(&id) || !targets.insert(id) {
            return Err(format!(
                "неизвестный или повторный объект {id} в списке оценки"
            ));
        }
    }
    let mut coverage = BTreeMap::new();
    for (&id, &depth) in truth.object_ids.iter().zip(truth.depth_m.iter()) {
        match (id, depth) {
            (Some(id), Some(depth))
                if depth.is_finite() && depth > 0.0 && objects.contains_key(&id) =>
            {
                *coverage.entry(id).or_insert(0usize) += 1;
            }
            (None, None) => {}
            _ => return Err("некорректные глубина или идентификатор в истине кадра".into()),
        }
    }
    targets.retain(|id| coverage.get(id).is_some_and(|&n| n >= min_visible_pixels));

    let mut result = PerceptionScore {
        pose_errors: Vec::new(),
        missed: Vec::new(),
        false_positives: Vec::new(),
    };
    for detection in detections {
        let observed = detection.pose.frame()?;
        let id = detection.object_id;
        if !targets.remove(&id) {
            result.false_positives.push(id);
            continue;
        }
        let actual = &objects[&id];
        let translation_m = (observed.translation.vector - actual.translation.vector).norm();
        let rotation_rad = observed.rotation.angle_to(&actual.rotation);
        checked(&[translation_m, rotation_rad], "ошибка позы")?;
        result.pose_errors.push(PoseError {
            object_id: id,
            translation_m,
            rotation_rad,
        });
    }
    result.pose_errors.sort_unstable_by_key(|e| e.object_id);
    result.false_positives.sort_unstable();
    result.missed.extend(targets);
    Ok(result)
}

/// Геометрия плоской мишени для visual тела (или visual среды).
/// Размеры и расцветка приходят из аргументов. Лицевая плоскость — XY при Z=0;
/// первая ячейка в отрицательных X и Y имеет цвет dark.
pub fn checkerboard(
    columns: u32,
    rows: u32,
    cell_m: f64,
    thickness_m: f64,
    dark: [f64; 4],
    light: [f64; 4],
) -> Result<Vec<Geometry>, String> {
    if columns == 0
        || rows == 0
        || !cell_m.is_finite()
        || !thickness_m.is_finite()
        || cell_m <= 0.0
        || thickness_m <= 0.0
        || !dark.iter().chain(&light).all(|v| (0.0..=1.0).contains(v))
    {
        return Err("неверные размеры или цвета шахматной мишени".into());
    }
    checked(
        &[cell_m * f64::from(columns), cell_m * f64::from(rows)],
        "размер шахматной мишени",
    )?;
    let mut cells = Vec::new();
    let count = (columns as usize)
        .checked_mul(rows as usize)
        .ok_or("слишком много ячеек")?;
    cells
        .try_reserve_exact(count)
        .map_err(|_| "не удалось выделить геометрию мишени")?;
    for y in 0..rows {
        for x in 0..columns {
            let mut geom = Geometry::new(Shape::Box {
                size_m: [cell_m, cell_m, thickness_m],
            });
            geom.pose.position_m = [
                (f64::from(x) + 0.5 - f64::from(columns) / 2.0) * cell_m,
                (f64::from(y) + 0.5 - f64::from(rows) / 2.0) * cell_m,
                thickness_m / 2.0,
            ];
            geom.rgba = Some(if (x ^ y) & 1 == 0 { dark } else { light });
            cells.push(geom);
        }
    }
    Ok(cells)
}
