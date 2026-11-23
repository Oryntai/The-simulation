//! Конструктор: пользователь собирает своё устройство из звеньев, шарниров, материалов,
//! приводов и электроники, сохраняет проект и получает вердикт.
//!
//! Любая правка сразу снимает прошлый вердикт: на экране не должно остаться «годен» от
//! устройства, которого уже нет.

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};

use nalgebra::DVector;
use sim_core::dynamics::body_poses;
use sim_core::{Pose, Shape, Vec3};
use sim_device::file::{from_text, to_text};
use sim_device::measure::{MeasureStatus, Measurement, Quantity};
use sim_device::mechanics::{Built, build};
use sim_device::*;

use crate::device_assess::{
    DeviceReport, Finding, Group, Outcome, Readiness, assess_device_in, open_items,
    project_directory, readiness,
};
use crate::device_catalog::{self as parts, PartOption, WorkshopCatalog};
use crate::device_templates::{TemplateCatalog, empty_device, template_directory, valid_stem};
use crate::scenario::*;

fn num(
    id: &str,
    label: &str,
    value: f64,
    min: f64,
    max: f64,
    unit: &'static str,
    decimals: usize,
) -> FormItem {
    FormItem::new(
        id,
        label,
        FormWidget::Number {
            value,
            min,
            max,
            unit,
            decimals,
        },
    )
}

fn choice(id: &str, label: &str, options: Vec<String>, selected: usize) -> FormItem {
    FormItem::new(id, label, FormWidget::Choice { options, selected })
}

fn button(id: &str, label: &str, hint: &str) -> FormItem {
    FormItem::new(
        id,
        label,
        FormWidget::Button {
            hint: hint.to_string(),
        },
    )
}

fn heading(id: &str, label: &str) -> FormItem {
    FormItem::new(id, label, FormWidget::Heading)
}

/// Список деталей с пунктом «нет» в начале: (подписи, номер выбранной).
fn part_choice(list: &[PartOption], current: &str) -> (Vec<String>, usize) {
    let mut options = vec!["— нет —".to_string()];
    options.extend(list.iter().map(|p| p.title.clone()));
    let selected = list
        .iter()
        .position(|p| p.id == current)
        .map_or(0, |i| i + 1);
    (options, selected)
}

fn part_id(list: &[PartOption], index: usize) -> String {
    if index == 0 {
        String::new()
    } else {
        list.get(index - 1).map_or(String::new(), |p| p.id.clone())
    }
}

/// Где на манекене находится крепление, м.
/// Короткий список по группам: без повторов, в группе не больше нескольких названий.
fn digest(items: &[&Finding]) -> String {
    const SHOWN: usize = 4;
    let mut lines = Vec::new();
    for g in Group::ALL {
        let mut titles: Vec<&str> = Vec::new();
        for f in items.iter().filter(|f| f.group == g) {
            if !titles.contains(&f.title.as_str()) {
                titles.push(&f.title);
            }
        }
        if titles.is_empty() {
            continue;
        }
        let more = titles.len().saturating_sub(SHOWN);
        titles.truncate(SHOWN);
        let tail = if more > 0 {
            format!(" и ещё {more}")
        } else {
            String::new()
        };
        lines.push(format!("• {g}: {}{tail}", titles.join("; ")));
    }
    lines.join("\n")
}

/// Сколько разных пунктов осталось по группам.
fn group_counts(items: &[&Finding]) -> String {
    Group::ALL
        .into_iter()
        .filter_map(|g| {
            let mut titles: Vec<&str> = items
                .iter()
                .filter(|f| f.group == g)
                .map(|f| f.title.as_str())
                .collect();
            titles.sort_unstable();
            titles.dedup();
            (!titles.is_empty()).then(|| format!("{g} {}", titles.len()))
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn mount_point(r: BodyRegion) -> Vec3 {
    match r {
        BodyRegion::None => Vec3::new(0.35, 0.0, 1.0),
        BodyRegion::Head => Vec3::new(0.0, 0.0, 1.68),
        BodyRegion::Face => Vec3::new(0.1, 0.0, 1.63),
        BodyRegion::Neck => Vec3::new(0.0, 0.0, 1.5),
        BodyRegion::Torso => Vec3::new(-0.13, 0.0, 1.3),
        BodyRegion::UpperArm => Vec3::new(0.0, -0.22, 1.42),
        BodyRegion::Forearm => Vec3::new(0.0, -0.22, 1.12),
        BodyRegion::Hand => Vec3::new(0.0, -0.22, 0.86),
        BodyRegion::Thigh => Vec3::new(0.0, -0.11, 0.92),
        BodyRegion::Shank => Vec3::new(0.0, -0.11, 0.5),
    }
}

fn mannequin() -> Vec<VisualInstance> {
    let skin = [0.72, 0.74, 0.78, 0.55];
    let capsule = |a: Vec3, b: Vec3, radius: f64| {
        let d = b - a;
        VisualInstance {
            shape: Shape::Capsule {
                radius,
                length: d.norm(),
            },
            pose: Pose::z_along((a + b) * 0.5, d),
            color: skin,
        }
    };
    let mut v = vec![
        VisualInstance {
            shape: Shape::Sphere { radius: 0.105 },
            pose: Pose::at(Vec3::new(0.0, 0.0, 1.62)),
            color: skin,
        },
        capsule(Vec3::new(0.0, 0.0, 0.98), Vec3::new(0.0, 0.0, 1.38), 0.13),
    ];
    for s in [-1.0, 1.0] {
        let (shoulder, arm_middle, wrist) = (
            Vec3::new(0.0, s * 0.22, 1.42),
            Vec3::new(0.0, s * 0.22, 1.12),
            Vec3::new(0.0, s * 0.22, 0.86),
        );
        v.push(capsule(shoulder, arm_middle, 0.045));
        v.push(capsule(arm_middle, wrist, 0.037));
        let (hip, knee, ankle) = (
            Vec3::new(0.0, s * 0.11, 0.92),
            Vec3::new(0.0, s * 0.11, 0.5),
            Vec3::new(0.0, s * 0.11, 0.08),
        );
        v.push(capsule(hip, knee, 0.075));
        v.push(capsule(knee, ankle, 0.05));
    }
    v
}

pub struct Constructor {
    device: Device,
    selected: usize,
    built: Option<Built>,
    build_errors: Vec<String>,
    report: Option<DeviceReport>,
    rx: Option<Mutex<Receiver<(u64, DeviceReport)>>>,
    requested: Option<u64>,
    worker_epoch: u64,
    crashed: bool,
    status: String,
    template_catalog: Option<TemplateCatalog>,
    template_error: Option<String>,
    files: Vec<String>,
    file_sel: usize,
    dir: PathBuf,
    measure_quantity: usize,
    measure_link: usize,
    measure_value: f64,
    measure_note: String,
    animate: bool,
    t: f64,
    epoch: u64,
    invalid_fields: BTreeMap<String, (String, String)>,
    sn_editor: Option<usize>,
}

impl Default for Constructor {
    fn default() -> Self {
        Self::with_template_dir(&template_directory())
    }
}

impl Constructor {
    /// Создаёт конструктор с каталогом шаблонов по указанному пути.
    pub fn with_template_dir(dir: &Path) -> Self {
        let (template_catalog, initial) = match TemplateCatalog::open(dir) {
            Ok(catalog) => {
                let initial = catalog
                    .default_index
                    .map_or_else(|| Ok(empty_device()), |index| catalog.load(index));
                (Some(catalog), initial)
            }
            Err(error) => (None, Err(error)),
        };
        let mut s = Self {
            device: empty_device(),
            selected: 0,
            built: None,
            build_errors: Vec::new(),
            report: None,
            rx: None,
            requested: None,
            worker_epoch: 0,
            crashed: false,
            status:
                "Выберите шаблон или соберите устройство, затем нажмите «Проверить устройство»."
                    .into(),
            template_catalog,
            template_error: None,
            files: Vec::new(),
            file_sel: 0,
            dir: project_directory(),
            measure_quantity: 0,
            measure_link: 0,
            measure_value: 0.0,
            measure_note: String::new(),
            animate: false,
            t: 0.0,
            epoch: 0,
            invalid_fields: BTreeMap::new(),
            sn_editor: None,
        };
        s.refresh_files();
        s.set_template(initial);
        s
    }

    fn set_template(&mut self, result: Result<Device, String>) {
        match result {
            Ok(device) => self.set_device(device),
            Err(error) => {
                self.set_device(empty_device());
                self.template_error = Some(error);
            }
        }
    }

    /// Устройство изменилось: прошлый вердикт больше не про него.
    fn changed(&mut self) {
        self.report = None;
        self.requested = None;
        self.crashed = false;
        self.selected = self.selected.min(self.device.links.len().saturating_sub(1));
        match build(&self.device, &WorkshopCatalog) {
            Ok((b, _)) => {
                self.built = Some(b);
                self.build_errors.clear();
            }
            Err(e) => {
                self.built = None;
                self.build_errors = e;
            }
        }
        self.epoch = self.epoch.wrapping_add(1);
    }

    fn refresh_files(&mut self) {
        let mut names: Vec<String> = std::fs::read_dir(&self.dir)
            .map(|rd| {
                rd.filter_map(Result::ok)
                    .filter_map(|e| {
                        if !e.file_type().ok()?.is_file() {
                            return None;
                        }
                        let p = e.path();
                        let stem = p.file_stem()?.to_str()?;
                        (p.extension()? == "device" && valid_stem(stem)).then(|| stem.to_owned())
                    })
                    .collect()
            })
            .unwrap_or_default();
        names.sort();
        self.files = names;
        self.file_sel = self.file_sel.min(self.files.len().saturating_sub(1));
    }

    /// Имя файла из названия устройства: только безопасные символы, без путей.
    fn file_stem(&self) -> String {
        let s: String = self
            .device
            .name
            .trim()
            .chars()
            .map(|c| {
                if c.is_alphanumeric() || c == '-' || c == '_' {
                    c
                } else {
                    '_'
                }
            })
            .collect();
        if s.is_empty() { "device".into() } else { s }
    }

    /// Сохранить проект атомарной заменой обычного файла. Имена не содержат частей пути.
    pub fn save(&mut self) -> Result<PathBuf, String> {
        if !self.invalid_fields.is_empty() {
            return Err("Исправьте ошибки в полях формы".into());
        }
        let errors = self.device.validate();
        if !errors.is_empty() {
            return Err(errors.join(". "));
        }
        std::fs::create_dir_all(&self.dir).map_err(|e| format!("Папка проектов: {e}"))?;
        self.check_directory()?;
        let stem = self.file_stem();
        let path = self.project_file(&stem)?;
        let text = to_text(&self.device);
        if from_text(&text).as_ref() != Ok(&self.device) {
            return Err("Описание проекта не восстанавливается из файла".into());
        }
        static TEMP: AtomicU64 = AtomicU64::new(0);
        let tmp = self.dir.join(format!(
            ".project-{}-{}.tmp",
            std::process::id(),
            TEMP.fetch_add(1, Ordering::Relaxed)
        ));
        let mut file = std::fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&tmp)
            .map_err(|e| format!("Не удалось открыть временный файл проекта: {e}"))?;
        let result = file
            .write_all(text.as_bytes())
            .and_then(|()| file.sync_all());
        drop(file);
        if let Err(error) = result.and_then(|()| std::fs::rename(&tmp, &path)) {
            let _ = std::fs::remove_file(&tmp);
            return Err(format!("Не удалось записать проект: {error}"));
        }
        self.refresh_files();
        self.file_sel = self.files.iter().position(|f| *f == stem).unwrap_or(0);
        Ok(path)
    }

    fn check_directory(&self) -> Result<(), String> {
        let meta = self
            .dir
            .symlink_metadata()
            .map_err(|e| format!("Папка проектов: {e}"))?;
        if !meta.is_dir() {
            return Err("Папка проектов должна быть обычным каталогом".into());
        }
        Ok(())
    }

    fn project_file(&self, stem: &str) -> Result<PathBuf, String> {
        if !valid_stem(stem) {
            return Err(
                "Имя проекта должно состоять из букв, цифр, дефисов или подчёркиваний".into(),
            );
        }
        let path = self.dir.join(format!("{stem}.device"));
        match path.symlink_metadata() {
            Ok(meta) if !meta.is_file() => Err("Проект должен быть обычным файлом".into()),
            Ok(_) => Ok(path),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(path),
            Err(e) => Err(format!("Не удалось проверить файл проекта: {e}")),
        }
    }

    /// Загрузить проект по имени файла (без расширения).
    pub fn load(&mut self, stem: &str) -> Result<(), String> {
        self.check_directory()?;
        let path = self.project_file(stem)?;
        let text = std::fs::read_to_string(&path).map_err(|e| format!("Чтение проекта: {e}"))?;
        self.set_device(from_text(&text)?);
        Ok(())
    }

    /// Папка проектов (для тестов).
    pub fn set_project_dir(&mut self, dir: PathBuf) {
        self.dir = dir;
        self.refresh_files();
        self.changed();
    }

    pub fn device(&self) -> &Device {
        &self.device
    }

    pub fn set_device(&mut self, d: Device) {
        self.device = d;
        self.template_error = None;
        self.invalid_fields.clear();
        self.sn_editor = None;
        self.selected = 0;
        self.changed();
    }

    pub fn report(&self) -> Option<&DeviceReport> {
        self.report.as_ref()
    }

    pub fn is_checking(&self) -> bool {
        self.requested.is_some()
    }

    /// Запустить проверку в фоне.
    pub fn start_check(&mut self) {
        if self.is_checking() || self.device.links.is_empty() || !self.invalid_fields.is_empty() {
            return;
        }
        self.requested = Some(self.epoch);
        self.report = None;
        self.crashed = false;
        self.start_worker_if_idle();
    }

    fn start_worker_if_idle(&mut self) {
        let Some(epoch) = self.requested.filter(|_| self.rx.is_none()) else {
            return;
        };
        let d = self.device.clone();
        let dir = self.dir.clone();
        let (tx, rx) = mpsc::channel();
        if std::thread::Builder::new()
            .name("проверка устройства".into())
            .spawn(move || {
                let _ = tx.send((epoch, assess_device_in(&d, &dir)));
            })
            .is_err()
        {
            self.requested = None;
            self.crashed = true;
            return;
        }
        self.worker_epoch = epoch;
        self.rx = Some(Mutex::new(rx));
    }

    fn poll(&mut self) {
        let polled = self
            .rx
            .as_mut()
            .map(|m| m.get_mut().expect("канал проверки").try_recv());
        match polled {
            Some(Ok((epoch, r))) => {
                if self.requested == Some(epoch) && epoch == self.epoch {
                    self.report = Some(r);
                    self.requested = None;
                }
                self.rx = None;
            }
            Some(Err(TryRecvError::Disconnected)) => {
                self.rx = None;
                // Недождавшаяся проверка текущей эпохи считается упавшей. Если
                // запрос уже новый, завершение старого потока запускает его из очереди.
                if self.requested == Some(self.worker_epoch) {
                    self.crashed = true;
                    self.requested = None;
                }
            }
            _ => {}
        }
        self.start_worker_if_idle();
    }

    fn link_titles(&self) -> Vec<String> {
        self.device
            .links
            .iter()
            .enumerate()
            .map(|(i, l)| format!("{}. {}", i + 1, l.name))
            .collect()
    }

    fn add_link(&mut self, parent: Option<usize>) {
        let parent = parent.filter(|&i| self.device.links.get(i).is_some());
        let Some(mut l) = self.device.links.get(self.selected).cloned() else {
            self.status = "Чтобы добавить звено, откройте шаблон или проект.".into();
            return;
        };
        l.name = format!("Звено {}", self.device.links.len() + 1);
        l.parent = parent;
        l.end_mass_kg = 0.0;
        l.settings.sn_source = None;
        l.settings.inertia_source = None;
        l.settings.limits_source = None;
        l.offset_mm = match parent {
            Some(p) => {
                let up = &self.device.links[p];
                let tip = up.direction.unit() * up.length_mm;
                [tip.x, tip.y, tip.z]
            }
            None => [0.0; 3],
        };
        self.device.links.push(l);
        self.selected = self.device.links.len() - 1;
        self.changed();
    }

    /// Удалить выбранное звено вместе со всем, что на нём держится.
    fn remove_link(&mut self) {
        if self.device.links.len() <= 1 {
            self.status = "Последнее звено удалить нельзя.".into();
            return;
        }
        let gone = self.device.subtree(self.selected);
        if gone.len() == self.device.links.len() {
            self.status = "Удаление убрало бы все звенья. Сначала добавьте отдельное звено.".into();
            return;
        }
        let mut map = vec![None; self.device.links.len()];
        let mut next = 0;
        for (i, slot) in map.iter_mut().enumerate() {
            if !gone.contains(&i) {
                *slot = Some(next);
                next += 1;
            }
        }
        let mut kept = Vec::with_capacity(next);
        for (i, mut l) in std::mem::take(&mut self.device.links)
            .into_iter()
            .enumerate()
        {
            if map[i].is_some() {
                l.parent = l.parent.and_then(|p| map[p]);
                kept.push(l);
            }
        }
        self.device.links = kept;
        for &i in gone.iter().rev() {
            self.shift_invalid("link_settings", i);
        }
        self.sn_editor = None;
        // Замеры по удалённым звеньям больше ни к чему не относятся.
        self.device.measurements.retain_mut(|m| match m.link {
            Some(l) => match map.get(l).copied().flatten() {
                Some(n) => {
                    m.link = Some(n);
                    true
                }
                None => false,
            },
            None => true,
        });
        self.selected = 0;
        self.changed();
    }

    fn pose_q(&self, b: &Built) -> DVector<f64> {
        let mut q = DVector::zeros(b.model.nv());
        for (i, l) in self.device.links.iter().enumerate() {
            if let Some(v) = b.dof[i] {
                let (lo, hi) = (l.joint.min_deg.to_radians(), l.joint.max_deg.to_radians());
                let s = if self.animate {
                    0.5 + 0.5 * (self.t * 0.8 + i as f64).sin()
                } else {
                    0.5
                };
                q[v] = lo + s * (hi - lo);
            }
        }
        q
    }

    fn apply_link_field(&mut self, field: &str, v: &FormValue) -> bool {
        let materials = parts::materials();
        let actuators = parts::actuators();
        let i = self.selected;
        let parent_choices = i;
        let Some(l) = self.device.links.get_mut(i) else {
            return false;
        };
        if matches!(
            field,
            "material" | "layer" | "walls" | "infill" | "across" | "print_enabled"
        ) {
            l.settings.sn_source = None;
        }
        if matches!(field, "actuator" | "axis" | "kind") {
            l.settings.inertia_source = None;
        }
        if matches!(
            field,
            "actuator" | "axis" | "kind" | "speed" | "torque" | "min" | "max"
        ) {
            l.settings.limits_source = None;
        }
        match (field, v) {
            ("name", FormValue::Text(t)) => l.name = t.clone(),
            ("parent", FormValue::Choice(k)) => {
                l.parent = if *k == 0 || *k > parent_choices {
                    None
                } else {
                    Some(*k - 1)
                };
            }
            ("direction", FormValue::Choice(k)) => l.direction = Direction::ALL[(*k).min(5)],
            ("length", FormValue::Number(x)) => l.length_mm = *x,
            ("off_x", FormValue::Number(x)) => l.offset_mm[0] = *x,
            ("off_y", FormValue::Number(x)) => l.offset_mm[1] = *x,
            ("off_z", FormValue::Number(x)) => l.offset_mm[2] = *x,
            ("section", FormValue::Choice(k)) => {
                let r = l.section.outer_radius_m() * 2000.0;
                l.section = match k {
                    0 => Section::Rect {
                        b_mm: (r * 0.5).max(3.0),
                        h_mm: r.max(5.0),
                    },
                    1 => Section::Round { d_mm: r.max(5.0) },
                    _ => Section::Tube {
                        od_mm: r.max(6.0),
                        id_mm: (r.max(6.0) - 3.0).max(1.0),
                    },
                };
            }
            ("sec_a", FormValue::Number(x)) => match &mut l.section {
                Section::Rect { b_mm, .. } => *b_mm = *x,
                Section::Round { d_mm } => *d_mm = *x,
                Section::Tube { od_mm, .. } => *od_mm = *x,
            },
            ("sec_b", FormValue::Number(x)) => match &mut l.section {
                Section::Rect { h_mm, .. } => *h_mm = *x,
                Section::Tube { id_mm, .. } => *id_mm = *x,
                Section::Round { .. } => {}
            },
            ("material", FormValue::Choice(k)) => {
                l.material = part_id(&materials, *k);
                if !parts::is_printable(&l.material) {
                    l.print = None;
                }
            }
            ("print_enabled", FormValue::Toggle(b)) => {
                l.print = b.then(|| {
                    let p = sim_materials::PrintProcess::default();
                    PrintSpec {
                        layer_mm: p.layer / sim_materials::catalog::MM,
                        walls: p.walls,
                        infill_pct: p.infill * 100.0,
                        across_layers: true,
                    }
                });
            }
            ("layer", FormValue::Number(x)) => {
                if let Some(p) = &mut l.print {
                    p.layer_mm = *x;
                }
            }
            ("walls", FormValue::Number(x)) => {
                if let Some(p) = &mut l.print {
                    p.walls = x.round() as u32;
                }
            }
            ("infill", FormValue::Number(x)) => {
                if let Some(p) = &mut l.print {
                    p.infill_pct = *x;
                }
            }
            ("across", FormValue::Toggle(b)) => {
                if let Some(p) = &mut l.print {
                    p.across_layers = *b;
                }
            }
            ("hole", FormValue::Number(x)) => l.hole_mm = *x,
            ("end_mass", FormValue::Number(x)) => l.end_mass_kg = *x,
            ("near", FormValue::Choice(k)) => l.near_body = BodyRegion::ALL[(*k).min(9)],
            ("lever", FormValue::Number(x)) => l.contact_lever_mm = *x,
            ("kind", FormValue::Choice(k)) => {
                if *k == 0 {
                    l.joint = JointSpec::fixed();
                } else {
                    l.joint.kind = JointKind::Hinge;
                }
            }
            ("axis", FormValue::Choice(k)) => l.joint.axis = Axis::ALL[(*k).min(2)],
            ("min", FormValue::Number(x)) => l.joint.min_deg = *x,
            ("max", FormValue::Number(x)) => l.joint.max_deg = *x,
            ("actuator", FormValue::Choice(k)) => l.joint.actuator = part_id(&actuators, *k),
            ("speed", FormValue::Number(x)) => l.joint.speed_limit_dps = *x,
            ("torque", FormValue::Number(x)) => l.joint.torque_limit_nm = *x,
            ("gap", FormValue::Number(x)) => l.joint.pinch_gap_mm = *x,
            ("guarded", FormValue::Toggle(b)) => l.joint.guarded = *b,
            _ => return false,
        }
        true
    }

    fn apply_electronics(&mut self, field: &str, v: &FormValue) -> bool {
        let e = &mut self.device.electronics;
        match (field, v) {
            ("board", FormValue::Choice(k)) => e.board = part_id(&parts::boards(), *k),
            ("driver", FormValue::Choice(k)) => e.driver = part_id(&parts::drivers(), *k),
            ("regulator", FormValue::Choice(k)) => e.regulator = part_id(&parts::regulators(), *k),
            ("cell", FormValue::Choice(k)) => e.cell = part_id(&parts::cells(), *k),
            ("series", FormValue::Number(x)) => e.series = x.round() as u32,
            ("parallel", FormValue::Number(x)) => e.parallel = x.round() as u32,
            ("fuse", FormValue::Number(x)) => e.fuse_a = *x,
            ("wire", FormValue::Number(x)) => e.wire_mm2 = *x,
            ("wire_m", FormValue::Number(x)) => e.wire_m = *x,
            ("bms", FormValue::Toggle(b)) => e.bms = *b,
            ("firmware", FormValue::Text(t)) => e.firmware = t.trim().to_string(),
            _ => return false,
        }
        true
    }

    fn add_measurement(&mut self) {
        let quantity = Quantity::ALL[self.measure_quantity.min(Quantity::ALL.len() - 1)];
        if !self.measure_value.is_finite() {
            self.status = "Замер должен быть числом.".into();
            return;
        }
        if quantity.per_link() && self.device.links.is_empty() {
            self.status = "Добавьте звено для замера.".into();
            return;
        }
        let link = quantity
            .per_link()
            .then(|| self.measure_link.min(self.device.links.len() - 1));
        let design = Some(crate::device_assess::measurement_stamp(
            &self.device,
            &self.dir,
        ));
        self.device.measurements.push(Measurement {
            quantity,
            link,
            value: self.measure_value,
            note: self.measure_note.trim().to_string(),
            design,
        });
        self.measure_note.clear();
        self.status =
            "Замер добавлен. Нажмите «Проверить устройство», чтобы сравнить его с расчётом.".into();
        self.changed();
    }
    fn setting_rows(
        &self,
        out: &mut Vec<FormItem>,
        prefix: &str,
        fields: impl Iterator<Item = &'static Field>,
        get: impl Fn(&str) -> Option<String>,
    ) {
        for field in fields {
            let id = format!("{prefix}.{}", field.key);
            let value = get(field.key);
            if field.choices.is_empty() {
                let text = self
                    .invalid_fields
                    .get(&id)
                    .map(|(t, _)| t.clone())
                    .or(value)
                    .unwrap_or_default();
                out.push(FormItem::new(id, field.title, FormWidget::LiveText(text)));
            } else {
                let selected = field
                    .choices
                    .iter()
                    .position(|(key, _)| value.as_deref() == Some(*key))
                    .map_or(0, |i| i + 1);
                let options = std::iter::once("— не задано —".into())
                    .chain(field.choices.iter().map(|(_, title)| (*title).to_owned()))
                    .collect();
                out.push(choice(&id, field.title, options, selected));
            }
        }
    }

    fn additional_form(&self, form: &mut Vec<FormItem>) {
        let d = &self.device;
        let extra = &d.electronics;
        form.push(heading("g.person", "Человек, поза и условия работы"));
        self.setting_rows(form, "design", DeviceSettings::FIELDS.iter().skip(6), |k| {
            d.settings.get(k)
        });
        if d.links.iter().any(|l| parts::is_printable(&l.material)) {
            form.push(heading("g.printer", "Принтер"));
            self.setting_rows(form, "design", DeviceSettings::FIELDS.iter().take(6), |k| {
                d.settings.get(k)
            });
        }
        if let Some(l) = d.links.get(self.selected) {
            let prefix = format!("link_settings.{}", self.selected);
            form.push(heading(
                "g.link_details",
                "Крепёж, ускорение и подтверждённые пределы звена",
            ));
            self.setting_rows(
                form,
                &prefix,
                LinkSettings::FIELDS
                    .iter()
                    .skip(11)
                    .filter(|f| f.key != "driver_id"),
                |k| l.settings.get(k),
            );
            if !l.joint.actuator.is_empty() {
                let (options, selected) = part_choice(
                    &parts::drivers(),
                    l.settings.driver_id.as_deref().unwrap_or_default(),
                );
                form.push(choice(
                    &format!("{prefix}.driver_id"),
                    "Драйвер шарнира (пусто — общий или встроенный)",
                    options,
                    selected,
                ));
            }
            if parts::is_printable(&l.material) {
                form.push(heading(
                    "g.print_details",
                    "Геометрия печати выбранного звена",
                ));
                self.setting_rows(form, &prefix, LinkSettings::FIELDS.iter().take(7), |k| {
                    l.settings.get(k)
                });
                if self.sn_editor == Some(self.selected)
                    || LinkSettings::FIELDS
                        .iter()
                        .skip(7)
                        .take(4)
                        .any(|f| l.settings.get(f.key).is_some())
                {
                    form.push(heading("g.sn", "Кривая усталости S–N выбранного звена"));
                    self.setting_rows(
                        form,
                        &prefix,
                        LinkSettings::FIELDS.iter().skip(7).take(4),
                        |k| l.settings.get(k),
                    );
                    form.push(button(
                        "sn.remove",
                        "Удалить кривую S–N",
                        "Усталость без замеров останется неподтверждённой",
                    ));
                } else {
                    form.push(button(
                        "sn.add",
                        "Добавить кривую S–N",
                        "Введите коэффициенты и протокол стендового испытания",
                    ));
                }
            }
        }
        form.push(heading(
            "g.battery_details",
            "Батарея, защита, заряд и тепловой контакт",
        ));
        self.setting_rows(
            form,
            "electrical",
            ElectricalSettings::FIELDS
                .iter()
                .take_while(|f| f.key != "rail_v"),
            |k| extra.settings.get(k),
        );
        form.push(heading("g.power_details", "Питание и нагрев платы"));
        self.setting_rows(
            form,
            "electrical",
            ElectricalSettings::FIELDS
                .iter()
                .skip_while(|f| f.key != "rail_v")
                .take_while(|f| f.key != "firmware_s"),
            |k| extra.settings.get(k),
        );
        form.push(heading("g.sensors", "Датчики"));
        form.push(button(
            "sensor.add",
            "Добавить датчик",
            "Выберите модель и шину питания",
        ));
        for (i, s) in extra.sensors.iter().enumerate() {
            let list = parts::sensors();
            let (options, selected) = part_choice(&list, s.id.as_deref().unwrap_or_default());
            form.push(choice(
                &format!("sensor.{i}.id"),
                &format!("Датчик {}", i + 1),
                options,
                selected,
            ));
            self.setting_rows(
                form,
                &format!("sensor.{i}"),
                SensorSettings::FIELDS.iter().skip(1),
                |k| s.get(k),
            );
            form.push(button(
                &format!("sensor.{i}.delete"),
                "Удалить датчик",
                "Удалить строку из ведомости",
            ));
        }
        form.push(heading("g.connections", "Соединения платы"));
        for (key, example) in Wiring::FORMATS {
            form.push(FormItem::new(
                format!("format.{key}"),
                "",
                FormWidget::Info((*example).into()),
            ));
        }
        form.push(button(
            "connection.add",
            "Добавить соединение",
            "В новой строке выберите формат из списка выше",
        ));
        for (i, w) in extra.connections.iter().enumerate() {
            let id = format!("connection.{i}.spec");
            let text = self
                .invalid_fields
                .get(&id)
                .map_or_else(|| w.to_line(), |(text, _)| text.clone());
            form.push(FormItem::new(
                id,
                format!("Соединение {}", i + 1),
                FormWidget::LiveText(text),
            ));
            form.push(button(
                &format!("connection.{i}.delete"),
                "Удалить соединение",
                "Удалить эту цепь из проверки",
            ));
        }
        form.push(heading("g.firmware", "Сценарии прошивки"));
        form.push(FormItem::new("firmware.hint", "", FormWidget::Info(
            "Файл ELF или Intel HEX в папке проектов. В сценарии укажите вход, сигнал и ожидаемый выход платы.".into())));
        self.setting_rows(
            form,
            "electrical",
            ElectricalSettings::FIELDS
                .iter()
                .filter(|f| f.key == "firmware_s"),
            |k| extra.settings.get(k),
        );
        form.push(button(
            "firmware.add",
            "Добавить сценарий прошивки",
            "Проверить реакцию на заданное событие",
        ));
        for (i, case) in extra.firmware_cases.iter().enumerate() {
            form.push(FormItem::new(
                format!("firmware.{i}.title"),
                "",
                FormWidget::Info(format!("Сценарий {}", i + 1)),
            ));
            self.setting_rows(
                form,
                &format!("firmware.{i}"),
                FirmwareSettings::FIELDS.iter(),
                |k| case.get(k),
            );
            form.push(button(
                &format!("firmware.{i}.delete"),
                "Удалить сценарий",
                "Удалить сценарий из проверки",
            ));
        }
    }

    fn shift_invalid(&mut self, prefix: &str, removed: usize) {
        let old = std::mem::take(&mut self.invalid_fields);
        let prefix = format!("{prefix}.");
        for (key, value) in old {
            let row = key
                .strip_prefix(&prefix)
                .and_then(|s| s.split_once('.'))
                .and_then(|(index, field)| Some((index.parse::<usize>().ok()?, field)));
            match row {
                Some((index, _)) if index == removed => (),
                Some((index, field)) if index > removed => {
                    self.invalid_fields
                        .insert(format!("{prefix}{}.{field}", index - 1), value);
                }
                _ => {
                    self.invalid_fields.insert(key, value);
                }
            }
        }
    }

    fn edit_settings(&mut self, id: &str, value: &FormValue) -> bool {
        fn edit(
            fields: &[Field],
            key: &str,
            value: &FormValue,
            set: impl FnOnce(&str, &str) -> Result<(), String>,
        ) -> Option<(String, Result<(), String>)> {
            let f = fields.iter().find(|f| f.key == key)?;
            let input = match value {
                FormValue::Text(text) => text.clone(),
                FormValue::Choice(0) => String::new(),
                FormValue::Choice(i) => f.choices.get(i - 1)?.0.to_owned(),
                _ => return None,
            };
            let result = set(key, &input);
            Some((input, result))
        }
        let Some((group, rest)) = id.split_once('.') else {
            return false;
        };
        let result = match group {
            "design" => {
                let outcome = edit(DeviceSettings::FIELDS, rest, value, |k, v| {
                    self.device.settings.set(k, v)
                });
                if DeviceSettings::FIELDS.iter().take(6).any(|f| f.key == rest) {
                    for link in &mut self.device.links {
                        link.settings.sn_source = None;
                    }
                }
                outcome
            }
            "electrical" => edit(ElectricalSettings::FIELDS, rest, value, |k, v| {
                self.device.electronics.settings.set(k, v)
            }),
            _ => {
                let Some((index, field)) = rest.split_once('.') else {
                    return false;
                };
                let Ok(index) = index.parse::<usize>() else {
                    return false;
                };
                let e = &mut self.device.electronics;
                if field == "delete" && matches!(value, FormValue::Pressed) {
                    match group {
                        "connection" if index < e.connections.len() => {
                            e.connections.remove(index);
                        }
                        "sensor" if index < e.sensors.len() => {
                            e.sensors.remove(index);
                        }
                        "firmware" if index < e.firmware_cases.len() => {
                            e.firmware_cases.remove(index);
                        }
                        _ => return false,
                    }
                    self.shift_invalid(group, index);
                    return true;
                }
                match group {
                    "link_settings" => self.device.links.get_mut(index).and_then(|l| {
                        if field == "driver_id"
                            && let FormValue::Choice(i) = value
                        {
                            let id = part_id(&parts::drivers(), *i);
                            l.settings.driver_id = (!id.is_empty()).then_some(id);
                            l.settings.limits_source = None;
                            return Some((String::new(), Ok(())));
                        }
                        if matches!(field, "acceleration_s" | "inertia_kg_m2") {
                            l.settings.limits_source = None;
                        }
                        edit(LinkSettings::FIELDS, field, value, |k, v| {
                            l.settings.set(k, v)
                        })
                    }),
                    "connection" if field == "spec" => {
                        match (e.connections.get_mut(index), value) {
                            (Some(w), FormValue::Text(text)) => Some((
                                text.clone(),
                                Wiring::from_line(text).map(|parsed| *w = parsed),
                            )),
                            _ => None,
                        }
                    }
                    "sensor" => e.sensors.get_mut(index).and_then(|s| {
                        if field == "id"
                            && let FormValue::Choice(i) = value
                        {
                            let id = part_id(&parts::sensors(), *i);
                            s.id = (!id.is_empty()).then_some(id);
                            return Some((String::new(), Ok(())));
                        }
                        edit(SensorSettings::FIELDS, field, value, |k, v| s.set(k, v))
                    }),
                    "firmware" => e.firmware_cases.get_mut(index).and_then(|s| {
                        edit(FirmwareSettings::FIELDS, field, value, |k, v| s.set(k, v))
                    }),
                    _ => None,
                }
            }
        };
        let Some((text, result)) = result else {
            return false;
        };
        match result {
            Ok(()) => {
                self.invalid_fields.remove(id);
            }
            Err(error) => {
                self.invalid_fields.insert(id.into(), (text, error));
            }
        }
        true
    }
}

impl Scenario for Constructor {
    fn id(&self) -> &'static str {
        "constructor"
    }

    fn title(&self) -> String {
        "Конструктор: своё устройство".into()
    }

    fn summary(&self) -> String {
        "Соберите устройство из звеньев, материалов, приводов и электроники, сохраните проект и проверьте. После сборки внесите замеры: если реальность опаснее расчёта, устройство не пройдёт.".into()
    }

    fn params(&self) -> Vec<Param> {
        Vec::new()
    }

    fn set_param(&mut self, _id: &str, _value: f64) {}

    fn presets(&self) -> Vec<String> {
        self.template_catalog
            .iter()
            .flat_map(|catalog| &catalog.entries)
            .map(|entry| entry.title.clone())
            .collect()
    }

    fn apply_preset(&mut self, index: usize) {
        if let Some(catalog) = &self.template_catalog {
            self.set_template(catalog.load(index));
        }
    }

    fn reset(&mut self) {
        self.t = 0.0;
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
        self.poll();
    }

    fn poll_background(&mut self) {
        self.poll();
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn form(&self) -> Vec<FormItem> {
        let d = &self.device;
        let mut f = vec![heading("g.project", "Проект")];
        f.push(FormItem::new(
            "project.name",
            "Название",
            FormWidget::LiveText(d.name.clone()),
        ));
        f.push(button(
            "project.save",
            "Сохранить проект",
            "Записать в папку sim/projects",
        ));
        if !self.files.is_empty() {
            f.push(choice(
                "project.file",
                "Файл",
                self.files.clone(),
                self.file_sel,
            ));
            f.push(button(
                "project.load",
                "Открыть проект",
                "Заменить текущее устройство выбранным файлом",
            ));
        }
        f.push(FormItem::new(
            "project.status",
            "",
            FormWidget::Info(self.status.clone()),
        ));

        f.push(heading("g.check", "Проверка"));
        f.push(button(
            "check.run",
            "▶ Проверить устройство",
            "Посчитать всё и вынести вердикт",
        ));
        f.push(FormItem::new(
            "view.animate",
            "Показать движение шарниров",
            FormWidget::Toggle(self.animate),
        ));

        f.push(heading("g.use", "Как используется"));
        let regions: Vec<String> = BodyRegion::ALL
            .iter()
            .map(|r| r.title().to_string())
            .collect();
        let at = |r: BodyRegion| BodyRegion::ALL.iter().position(|x| *x == r).unwrap_or(0);
        f.push(choice(
            "use.mount",
            "Крепится к",
            regions.clone(),
            at(d.mount),
        ));
        f.push(num(
            "use.ambient",
            "Воздух",
            d.usage.ambient_c,
            -40.0,
            70.0,
            "°C",
            0,
        ));
        f.push(num(
            "use.cycles",
            "Циклов за срок службы",
            d.usage.cycles,
            0.0,
            1e8,
            "",
            0,
        ));
        f.push(num(
            "use.hours",
            "Часов в день",
            d.usage.hours_per_day,
            0.0,
            24.0,
            "ч",
            1,
        ));
        f.push(FormItem::new(
            "use.long",
            "Носится долго",
            FormWidget::Toggle(d.usage.long_wear),
        ));

        f.push(heading("g.links", "Звенья"));
        f.push(choice(
            "links.select",
            "Выбрано",
            self.link_titles(),
            self.selected,
        ));
        f.push(button(
            "links.add_child",
            "+ Звено на выбранном",
            "Новое звено крепится к концу выбранного",
        ));
        f.push(button(
            "links.add_root",
            "+ Звено на теле",
            "Новое звено крепится прямо к телу",
        ));
        f.push(button(
            "links.remove",
            "Удалить выбранное",
            "Удаляет звено и всё, что на нём держится",
        ));

        if let Some(l) = d.links.get(self.selected) {
            f.push(heading("g.link", "Выбранное звено"));
            f.push(FormItem::new(
                "link.name",
                "Имя",
                FormWidget::LiveText(l.name.clone()),
            ));
            let mut parents = vec!["тело".to_string()];
            parents.extend(self.link_titles().into_iter().take(self.selected));
            f.push(choice(
                "link.parent",
                "Крепится к",
                parents,
                l.parent.map_or(0, |p| p + 1),
            ));
            let dirs: Vec<String> = Direction::ALL
                .iter()
                .map(|x| x.title().to_string())
                .collect();
            let dir = Direction::ALL
                .iter()
                .position(|x| *x == l.direction)
                .unwrap_or(0);
            f.push(choice("link.direction", "Направление", dirs, dir));
            f.push(num(
                "link.length",
                "Длина",
                l.length_mm,
                1.0,
                2000.0,
                "мм",
                0,
            ));
            f.push(num(
                "link.off_x",
                "Смещение X",
                l.offset_mm[0],
                -1000.0,
                1000.0,
                "мм",
                0,
            ));
            f.push(num(
                "link.off_y",
                "Смещение Y",
                l.offset_mm[1],
                -1000.0,
                1000.0,
                "мм",
                0,
            ));
            f.push(num(
                "link.off_z",
                "Смещение Z",
                l.offset_mm[2],
                -1000.0,
                1000.0,
                "мм",
                0,
            ));
            let (kind, a, b) = match l.section {
                Section::Rect { b_mm, h_mm } => (0, b_mm, Some(("Высота", h_mm))),
                Section::Round { d_mm } => (1, d_mm, None),
                Section::Tube { od_mm, id_mm } => (2, od_mm, Some(("Внутр. диаметр", id_mm))),
            };
            f.push(choice(
                "link.section",
                "Сечение",
                vec!["прямоугольник".into(), "круг".into(), "труба".into()],
                kind,
            ));
            f.push(num(
                "link.sec_a",
                if kind == 0 {
                    "Ширина"
                } else {
                    "Диаметр"
                },
                a,
                0.5,
                200.0,
                "мм",
                1,
            ));
            if let Some((label, value)) = b {
                f.push(num("link.sec_b", label, value, 0.5, 200.0, "мм", 1));
            }
            let materials = parts::materials();
            let (options, sel) = part_choice(&materials, &l.material);
            f.push(choice("link.material", "Материал", options, sel));
            if parts::is_printable(&l.material) {
                f.push(FormItem::new(
                    "link.print_enabled",
                    "Задать параметры послойной печати",
                    FormWidget::Toggle(l.print.is_some()),
                ));
            }
            if let Some(p) = l.print {
                f.push(num("link.layer", "Слой", p.layer_mm, 0.04, 0.6, "мм", 2));
                f.push(num(
                    "link.walls",
                    "Стенок",
                    p.walls as f64,
                    1.0,
                    20.0,
                    "",
                    0,
                ));
                f.push(num(
                    "link.infill",
                    "Заполнение",
                    p.infill_pct,
                    0.0,
                    100.0,
                    "%",
                    0,
                ));
                f.push(FormItem::new(
                    "link.across",
                    "Нагрузка поперёк слоёв",
                    FormWidget::Toggle(p.across_layers),
                ));
            }
            f.push(num(
                "link.hole",
                "Отверстие у основания",
                l.hole_mm,
                0.0,
                100.0,
                "мм",
                1,
            ));
            f.push(num(
                "link.end_mass",
                "Масса на конце",
                l.end_mass_kg,
                0.0,
                100.0,
                "кг",
                2,
            ));
            f.push(choice(
                "link.near",
                "Рядом с телом",
                regions,
                at(l.near_body),
            ));
            if l.near_body != BodyRegion::None {
                f.push(num(
                    "link.lever",
                    "Точка давления от шарнира",
                    l.contact_lever_mm,
                    1.0,
                    2000.0,
                    "мм",
                    0,
                ));
            }

            f.push(heading("g.joint", "Шарнир выбранного звена"));
            let hinge = l.joint.kind == JointKind::Hinge;
            f.push(choice(
                "link.kind",
                "Тип",
                vec!["жёстко".into(), "шарнир".into()],
                hinge as usize,
            ));
            if hinge {
                let axes: Vec<String> = Axis::ALL.iter().map(|x| x.title().to_string()).collect();
                let ax = Axis::ALL
                    .iter()
                    .position(|x| *x == l.joint.axis)
                    .unwrap_or(0);
                f.push(choice("link.axis", "Ось", axes, ax));
                f.push(num(
                    "link.min",
                    "Угол от",
                    l.joint.min_deg,
                    -360.0,
                    360.0,
                    "°",
                    0,
                ));
                f.push(num(
                    "link.max",
                    "Угол до",
                    l.joint.max_deg,
                    -360.0,
                    360.0,
                    "°",
                    0,
                ));
                let actuators = parts::actuators();
                let (options, sel) = part_choice(&actuators, &l.joint.actuator);
                f.push(choice("link.actuator", "Привод", options, sel));
                if !l.joint.actuator.is_empty() {
                    f.push(num(
                        "link.speed",
                        "Предел скорости",
                        l.joint.speed_limit_dps,
                        1.0,
                        1000.0,
                        "°/с",
                        0,
                    ));
                    f.push(num(
                        "link.torque",
                        "Предел момента (0 — нет)",
                        l.joint.torque_limit_nm,
                        0.0,
                        500.0,
                        "Н·м",
                        1,
                    ));
                }
                f.push(num(
                    "link.gap",
                    "Зазор при схождении",
                    l.joint.pinch_gap_mm,
                    0.0,
                    1000.0,
                    "мм",
                    0,
                ));
                f.push(FormItem::new(
                    "link.guarded",
                    "Зона закрыта кожухом",
                    FormWidget::Toggle(l.joint.guarded),
                ));
            }
        }

        f.push(heading("g.electronics", "Электроника и батарея"));
        let e = &d.electronics;
        for (id, label, list, current) in [
            ("el.board", "Плата", parts::boards(), &e.board),
            ("el.driver", "Драйвер", parts::drivers(), &e.driver),
            (
                "el.regulator",
                "Стабилизатор",
                parts::regulators(),
                &e.regulator,
            ),
            ("el.cell", "Ячейка батареи", parts::cells(), &e.cell),
        ] {
            let (options, sel) = part_choice(&list, current);
            f.push(choice(id, label, options, sel));
        }
        f.push(num(
            "el.series",
            "Последовательно",
            e.series as f64,
            0.0,
            24.0,
            "S",
            0,
        ));
        f.push(num(
            "el.parallel",
            "Параллельно",
            e.parallel as f64,
            0.0,
            12.0,
            "P",
            0,
        ));
        f.push(num(
            "el.fuse",
            "Предохранитель (0 — нет)",
            e.fuse_a,
            0.0,
            200.0,
            "А",
            1,
        ));
        f.push(num(
            "el.wire",
            "Сечение провода",
            e.wire_mm2,
            0.0,
            50.0,
            "мм²",
            2,
        ));
        f.push(num(
            "el.wire_m",
            "Длина провода",
            e.wire_m,
            0.0,
            10.0,
            "м",
            2,
        ));
        f.push(FormItem::new(
            "el.bms",
            "Есть плата защиты (BMS)",
            FormWidget::Toggle(e.bms),
        ));
        f.push(FormItem::new(
            "el.firmware",
            "Файл прошивки",
            FormWidget::LiveText(e.firmware.clone()),
        ));

        self.additional_form(&mut f);
        f.push(heading("g.measure", "Замеры на настоящем устройстве"));
        let quantities: Vec<String> = Quantity::ALL
            .iter()
            .map(|q| format!("{}, {}", q.title(), q.unit()))
            .collect();
        f.push(choice(
            "meas.quantity",
            "Что мерили",
            quantities,
            self.measure_quantity,
        ));
        let q = Quantity::ALL[self.measure_quantity.min(Quantity::ALL.len() - 1)];
        if q.per_link() {
            f.push(choice(
                "meas.link",
                "Звено",
                self.link_titles(),
                self.measure_link.min(d.links.len().saturating_sub(1)),
            ));
        }
        f.push(num(
            "meas.value",
            "Значение",
            self.measure_value,
            -1e6,
            1e6,
            q.unit(),
            2,
        ));
        f.push(FormItem::new(
            "meas.note",
            "Чем мерили",
            FormWidget::LiveText(self.measure_note.clone()),
        ));
        f.push(button(
            "meas.add",
            "Добавить замер",
            "Сохраняется в проекте",
        ));
        for (i, m) in d.measurements.iter().enumerate() {
            let at = m
                .link
                .map_or(String::new(), |l| format!(" (звено {})", l + 1));
            f.push(FormItem::new(
                format!("meas.row.{i}"),
                "",
                FormWidget::Info(format!(
                    "{}{at}: {} {}",
                    m.quantity.title(),
                    m.value,
                    m.quantity.unit()
                )),
            ));
            f.push(button(
                &format!("meas.del.{i}"),
                "Убрать замер",
                "Удалить этот замер из проекта",
            ));
        }
        f
    }

    fn set_form(&mut self, id: &str, value: FormValue) {
        if let Some(i) = id
            .strip_prefix("meas.del.")
            .and_then(|x| x.parse::<usize>().ok())
        {
            if i < self.device.measurements.len() {
                self.device.measurements.remove(i);
                self.changed();
            }
            return;
        }
        match (id, &value) {
            ("project.name", FormValue::Text(t)) => {
                self.device.name = t.trim().to_string();
                self.changed();
            }
            ("project.save", _) => {
                self.status = match self.save() {
                    Ok(p) => format!(
                        "Сохранено: {}",
                        p.file_name().and_then(|x| x.to_str()).unwrap_or("?")
                    ),
                    Err(e) => format!("Не сохранилось: {e}"),
                };
            }
            ("project.file", FormValue::Choice(k)) => self.file_sel = *k,
            ("project.load", _) => {
                let stem = self.files.get(self.file_sel).cloned().unwrap_or_default();
                self.status = match self.load(&stem) {
                    Ok(()) => format!("Открыт проект «{stem}»."),
                    Err(e) => format!("Не открылся: {e}"),
                };
            }
            ("check.run", _) => {
                self.start_check();
            }
            ("view.animate", FormValue::Toggle(b)) => self.animate = *b,
            ("use.mount", FormValue::Choice(k)) => {
                self.device.mount = BodyRegion::ALL[(*k).min(9)];
                self.changed();
            }
            ("use.ambient", FormValue::Number(x)) => {
                self.device.usage.ambient_c = *x;
                self.changed();
            }
            ("use.cycles", FormValue::Number(x)) => {
                self.device.usage.cycles = *x;
                self.changed();
            }
            ("use.hours", FormValue::Number(x)) => {
                self.device.usage.hours_per_day = *x;
                self.changed();
            }
            ("use.long", FormValue::Toggle(b)) => {
                self.device.usage.long_wear = *b;
                self.changed();
            }
            ("links.select", FormValue::Choice(k)) => {
                self.selected = (*k).min(self.device.links.len().saturating_sub(1))
            }
            ("links.add_child", _) => self.add_link(Some(self.selected)),
            ("links.add_root", _) => self.add_link(None),
            ("links.remove", _) => self.remove_link(),
            ("meas.quantity", FormValue::Choice(k)) => self.measure_quantity = *k,
            ("meas.link", FormValue::Choice(k)) => self.measure_link = *k,
            ("meas.value", FormValue::Number(x)) => self.measure_value = *x,
            ("meas.note", FormValue::Text(t)) => self.measure_note = t.clone(),
            ("meas.add", _) => self.add_measurement(),
            ("sn.add", FormValue::Pressed) => {
                self.sn_editor = Some(self.selected);
                self.changed();
            }
            ("sn.remove", FormValue::Pressed) => {
                if let Some(l) = self.device.links.get_mut(self.selected) {
                    for k in ["sn_a_mpa", "sn_b", "sn_cycles", "sn_source"] {
                        let _ = l.settings.set(k, "");
                        self.invalid_fields
                            .remove(&format!("link_settings.{}.{k}", self.selected));
                    }
                }
                self.sn_editor = None;
                self.changed();
            }
            ("connection.add", FormValue::Pressed) => {
                self.device
                    .electronics
                    .connections
                    .push(Wiring(sim_boards::Connection::Ground { shared: None }));
                self.changed();
            }
            ("sensor.add", FormValue::Pressed) => {
                self.device
                    .electronics
                    .sensors
                    .push(SensorSettings::default());
                self.changed();
            }
            ("firmware.add", FormValue::Pressed) => {
                self.device
                    .electronics
                    .firmware_cases
                    .push(FirmwareSettings::default());
                self.changed();
            }
            _ => {
                let done = if let Some(field) = id.strip_prefix("link.") {
                    self.apply_link_field(field, &value)
                } else if let Some(field) = id.strip_prefix("el.") {
                    self.apply_electronics(field, &value)
                } else {
                    self.edit_settings(id, &value)
                };
                if done {
                    self.changed();
                }
            }
        }
    }

    fn actions(&self) -> Vec<Action> {
        vec![Action::new(
            "check",
            "Проверить устройство",
            "Посчитать всё и вынести вердикт",
            self.is_checking(),
        )]
    }

    fn run_action(&mut self, id: &str) {
        if id == "check" {
            self.start_check();
        }
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let mut v = mannequin();
        let Some(b) = &self.built else {
            return v;
        };
        let origin = mount_point(self.device.mount);
        let q = self.pose_q(b);
        let frames = body_poses(&b.model, &q);
        for (i, l) in self.device.links.iter().enumerate() {
            let f = frames[i];
            let base = origin + f.pos;
            let u = f.rot * l.direction.unit();
            let len = l.length_mm * 1e-3;
            let center = base + u * (len / 2.0);
            // Цвета выбранного и обычного звена из прежней палитры `scenario.rs`.
            let color = if i == self.selected {
                [0.08, 0.55, 0.6, 1.0]
            } else {
                [0.16, 0.18, 0.22, 1.0]
            };
            let shape = match l.section {
                Section::Rect { b_mm, h_mm } => Shape::Box {
                    half: Vec3::new(b_mm * 0.5e-3, h_mm * 0.5e-3, len / 2.0),
                },
                Section::Round { d_mm } => Shape::Cylinder {
                    radius: d_mm * 0.5e-3,
                    length: len,
                },
                Section::Tube { od_mm, .. } => Shape::Cylinder {
                    radius: od_mm * 0.5e-3,
                    length: len,
                },
            };
            v.push(VisualInstance {
                shape,
                pose: Pose::z_along(center, u),
                color,
            });
            if l.joint.kind == JointKind::Hinge {
                let axis = f.rot * l.joint.axis.unit();
                let r = (l.section.outer_radius_m() * 1.3).max(0.015);
                let color = if l.joint.actuator.is_empty() {
                    C_GRAY
                } else {
                    C_MOTOR
                };
                v.push(VisualInstance {
                    shape: Shape::Cylinder {
                        radius: r,
                        length: r * 1.4,
                    },
                    pose: Pose::z_along(base, axis),
                    color,
                });
            }
            if l.end_mass_kg > 0.0 {
                let r = (3.0 * l.end_mass_kg / (4.0 * std::f64::consts::PI * 7200.0)).cbrt();
                v.push(VisualInstance {
                    shape: Shape::Sphere { radius: r },
                    pose: Pose::at(base + u * len),
                    color: C_IRON,
                });
            }
        }
        v
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn readouts(&self) -> Vec<Readout> {
        let Some(r) = &self.report else {
            return Vec::new();
        };
        let n = |o| r.findings.iter().filter(|f| f.outcome == o).count() as f64;
        let mut out = vec![
            Readout::new("Выдержано", n(Outcome::Pass), "", 0),
            Readout::new("Нарушено", n(Outcome::Fail), "", 0),
            Readout::new("Не посчитано", n(Outcome::NotAnalyzed), "", 0),
        ];
        if let Some(a) = &r.analysis {
            out.push(Readout::new("Масса устройства", a.total_mass_kg, "кг", 3));
            for j in &a.joints {
                let name = &self.device.links[j.link].name;
                out.push(Readout::new(
                    &format!("Момент: {name}"),
                    j.required_nm,
                    "Н·м",
                    2,
                ));
            }
        }
        out
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let Some(r) = &self.report else {
            return Vec::new();
        };
        let (to_fill, _) = open_items(&r.findings);
        let rank = |f: &Finding| match f.outcome {
            Outcome::Fail => 0,
            Outcome::NotAnalyzed if to_fill.iter().any(|x| std::ptr::eq(*x, f)) => 1,
            Outcome::NotAnalyzed => 2,
            Outcome::Pass => 3,
        };
        let mut sorted: Vec<_> = r.findings.iter().collect();
        sorted.sort_by_key(|f| rank(f));
        // Один привод считается при нескольких напряжениях: одинаковые строки показываем раз.
        let mut seen = std::collections::HashSet::new();
        sorted.retain(|f| {
            seen.insert((
                f.group.to_string(),
                f.title.clone(),
                f.detail.clone(),
                f.value.to_bits(),
                f.limit.to_bits(),
            ))
        });
        sorted
            .into_iter()
            .map(|f| Comparison {
                title: format!("{}: {}", f.group, f.title),
                sim_label: "у устройства".into(),
                sim: f.value,
                ref_label: "требование".into(),
                reference: f.limit,
                unit: f.unit,
                decimals: 2,
                status: match f.outcome {
                    Outcome::Pass => Status::WithinLimit,
                    Outcome::Fail => Status::OverLimit,
                    Outcome::NotAnalyzed => Status::NotAnalyzed,
                },
                explanation: f.detail.clone(),
            })
            .collect()
    }

    fn notes(&self) -> Vec<Note> {
        let mut validation: Vec<_> = self
            .template_error
            .iter()
            .chain(&self.build_errors)
            .chain(self.invalid_fields.values().map(|(_, error)| error))
            .map(|e| Note::new(Level::Danger, e.clone()))
            .collect();
        if self.report.is_none() && !validation.is_empty() && !self.is_checking() {
            return validation;
        }
        if self.crashed {
            return vec![Note::new(
                Level::Danger,
                "Проверка упала, вердикта нет. Считать устройство непроверенным.",
            )];
        }
        if self.is_checking() {
            return vec![Note::new(Level::Info, "Идёт проверка устройства…")];
        }
        let Some(r) = &self.report else {
            return vec![Note::new(
                Level::Info,
                "Вердикта нет: устройство ещё не проверено или изменилось после проверки.",
            )];
        };
        let fails: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.outcome == Outcome::Fail)
            .collect();
        let (to_fill, on_bench) = open_items(&r.findings);
        let bench_line = |lead: &str| {
            Note::new(
                Level::Info,
                format!("{lead} ({}): {}.", on_bench.len(), group_counts(&on_bench)),
            )
        };
        let mut out = match readiness(&r.findings) {
            Readiness::Rejected => vec![Note::new(
                Level::Danger,
                format!("НЕ ГОДЕН. Нарушено:\n{}", digest(&fails)),
            )],
            Readiness::NeedsWork => vec![
                Note::new(
                    Level::Warn,
                    format!(
                        "НЕ ДОКАЗАНО. Нарушений нет, но строить пока нельзя. Заполните в чертеже:\n{}",
                        digest(&to_fill)
                    ),
                ),
                bench_line("Потом закроется только на стенде"),
            ],
            Readiness::BenchOnly => vec![
                Note::new(
                    Level::Warn,
                    "МОЖНО НА СТЕНД БЕЗ ЧЕЛОВЕКА. Расчёт конструкции сошёлся. Собирайте и испытывайте с грузом вместо руки, за ограждением, с аварийной кнопкой и ограничением тока. НА СЕБЯ НЕ НАДЕВАТЬ.",
                ),
                bench_line("Перед надеванием закрыть"),
            ],
            Readiness::Passed => vec![Note::new(
                Level::Good,
                "ПРОШЁЛ СИМУЛЯЦИЮ И ЗАМЕРЫ. Все проверки и стендовые замеры этой версии сошлись.",
            )],
        };
        if !r.uart.is_empty() {
            out.push(Note::new(Level::Info, format!("Вывод UART:\n{}", r.uart)));
        }
        out.append(&mut validation);
        for m in &r.measures {
            let level = match m.status {
                MeasureStatus::RealIsWorse => Level::Danger,
                MeasureStatus::NoPrediction | MeasureStatus::Suspicious => Level::Warn,
                _ => Level::Info,
            };
            out.push(Note::new(level, m.detail.clone()));
        }
        out
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.05, 0.0, 1.05)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(1.5, -2.2, 0.45)
    }
}

#[cfg(test)]
mod template_tests {
    use super::*;
    use std::fs;

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(label: &str) -> Self {
            let dir =
                std::env::temp_dir().join(format!("sim-templates-{}-{label}", std::process::id()));
            fs::create_dir(&dir).unwrap();
            Self(dir)
        }

        fn write_device(&self, device: &Device) -> PathBuf {
            let path = self.0.join(format!("{}.device", device.template));
            fs::write(&path, to_text(device)).unwrap();
            path
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn example(id: &str, title: &str) -> Device {
        let mut device = empty_device();
        device.name = title.into();
        device.template = id.into();
        // Размеры условного стержня заданы только для теста загрузки и сборки.
        device.links.push(LinkSpec {
            name: "Стержень".into(),
            parent: None,
            length_mm: 100.0,
            near_body: BodyRegion::None,
            offset_mm: [0.0; 3],
            direction: Direction::Down,
            section: Section::Round { d_mm: 10.0 },
            material: "al6061-t6".into(),
            print: None,
            hole_mm: 0.0,
            end_mass_kg: 0.0,
            contact_lever_mm: 100.0,
            joint: JointSpec::fixed(),
            settings: LinkSettings::default(),
        });
        device
    }

    #[test]
    fn constructor_discovers_added_template_files() {
        let dir = TempDir::new("discovery");
        let block = example("blok", "Блок");
        let rod = example("sterzhen", "Стержень");
        dir.write_device(&rod);
        dir.write_device(&block);
        let index = "default = \"sterzhen\"\norder = [\"blok\", \"sterzhen\"]\n";
        fs::write(dir.0.join("index.toml"), index).unwrap();

        let c = Constructor::with_template_dir(&dir.0);
        assert_eq!(c.presets(), ["Блок", "Стержень"]);
        assert_eq!(c.device(), &rod);
        assert!(c.built.is_some());

        let carriage = example("karetka", "Каретка");
        let pendulum = example("mayatnik", "Маятник");
        dir.write_device(&pendulum);
        dir.write_device(&carriage);
        let mut c = Constructor::with_template_dir(&dir.0);
        assert_eq!(c.presets(), ["Блок", "Стержень", "Каретка", "Маятник"]);
        assert_eq!(c.device(), &rod);
        c.apply_preset(2);
        assert_eq!(c.device(), &carriage);
        assert!(c.built.is_some());
        c.apply_preset(3);
        assert_eq!(c.device(), &pendulum);
    }

    #[test]
    fn broken_or_missing_template_drops_even_a_positive_report() {
        let dir = TempDir::new("broken");
        let mut block = example("blok", "Блок");
        let path = dir.write_device(&block);
        fs::write(dir.0.join("index.toml"), "default = \"blok\"\n").unwrap();
        let mut c = Constructor::with_template_dir(&dir.0);
        assert_eq!(c.presets(), ["Блок"]);

        // Имитируем положительный отчёт и сообщение от предыдущей эпохи без запуска потока.
        let report = DeviceReport {
            findings: Group::ALL
                .into_iter()
                .map(|group| Finding {
                    group,
                    code: "test",
                    link: None,
                    title: String::new(),
                    outcome: Outcome::Pass,
                    value: 0.0,
                    limit: 0.0,
                    unit: "",
                    detail: String::new(),
                })
                .collect(),
            ..Default::default()
        };
        assert_eq!(report.readiness(), Readiness::Passed);

        for (bad_number, detail) in [(f64::NAN, "NaN"), (f64::INFINITY, "inf")] {
            c.set_device(block.clone());
            c.report = Some(report.clone());
            assert!(c.notes().iter().any(|note| note.level == Level::Good));
            let (tx, rx) = mpsc::channel();
            tx.send((c.epoch, report.clone())).unwrap();
            c.requested = Some(c.epoch);
            c.rx = Some(Mutex::new(rx));

            let mut broken = block.clone();
            broken.links[0].length_mm = bad_number;
            dir.write_device(&broken);
            c.apply_preset(0);
            c.poll_background();
            assert_failed_template(&mut c, detail);
        }

        fs::remove_file(path).unwrap();
        c.apply_preset(0);
        assert_failed_template(&mut c, "Чтение шаблона «blok»");
        let mut at_start = Constructor::with_template_dir(&dir.0);
        assert_failed_template(&mut at_start, "Чтение шаблона «blok»");

        block.name = "Исправленный блок".into();
        dir.write_device(&block);
        c.apply_preset(0);
        assert_eq!(c.device(), &block);
        assert!(c.built.is_some());
        assert!(c.notes().iter().all(|note| note.level != Level::Danger));
    }

    fn assert_failed_template(c: &mut Constructor, detail: &str) {
        assert_eq!(c.device(), &empty_device());
        assert!(c.built.is_none());
        // Тот же вход отдельно отвергает оценщик, даже в обход кнопки конструктора.
        assert_eq!(
            assess_device_in(c.device(), &c.dir).readiness(),
            Readiness::Rejected
        );
        c.start_check();
        assert!(c.report().is_none());
        assert!(!c.is_checking());
        assert!(
            c.notes()
                .iter()
                .any(|note| { note.level == Level::Danger && note.text.contains(detail) })
        );
    }

    #[test]
    fn absent_default_never_chooses_another_template() {
        let dir = TempDir::new("default");
        let block = example("blok", "Блок");
        dir.write_device(&block);
        let mut c = Constructor::with_template_dir(&dir.0);
        assert_eq!(c.presets(), ["Блок"]);
        assert_eq!(c.device(), &empty_device());
        c.apply_preset(0);
        assert_eq!(c.device(), &block);
        c.apply_preset(usize::MAX);
        assert_failed_template(&mut c, "Нет шаблона с индексом");

        fs::write(dir.0.join("index.toml"), "default = \"../sterzhen\"\n").unwrap();
        let mut c = Constructor::with_template_dir(&dir.0);
        assert!(c.presets().is_empty());
        assert_failed_template(&mut c, "Индекс шаблонов");

        let mut c = Constructor::with_template_dir(&dir.0.join("missing"));
        assert!(c.presets().is_empty());
        assert_failed_template(&mut c, "Каталог шаблонов");
    }
}
