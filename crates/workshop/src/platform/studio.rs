//! Окно управляет просмотром, вызов конвейера выполняется отдельно от отрисовки.
use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::thread::{self, JoinHandle};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, PrimaryEguiContext, egui};
use egui_plot::{Line, Plot, Points, VLine};
use sim_signal::{PixelFormat, Time};

use super::{
    Force, Input, Participant, PointSample, Replay, Report, Scene, Series, SignalKind, Texture,
    Verdict, View,
    input::{Suite, seconds},
    read_text, run_with_peak,
};

// Рабочее разрешение обзорной текстуры. При увеличении окна egui масштабирует её.
const SIZE: [u32; 2] = [480, 360];
/// Наибольшая ширина обзорного кадра в точках экрана: растеризация идёт на процессоре.
const MAX_WIDTH: u32 = 1600;

#[derive(Default)]
pub(super) struct Slot {
    pub(super) scene: Option<Scene>,
    pub(super) replay: Option<Replay>,
    report: Option<Report>,
    texture: Option<egui::TextureHandle>,
    fingerprint_matches: bool,
    source: Option<Input>,
}

pub(super) enum SlotFrame<'a> {
    Rendered(Texture),
    Recorded(&'a sim_signal::ImageFrame, Option<Force>),
}

impl Slot {
    fn sample(
        &self,
        source: &Replay,
        signal: &Series,
        at: Time,
    ) -> Option<(&Series, Option<PointSample>)> {
        let replay = self.replay.as_ref()?;
        if signal.kind == SignalKind::Truth
            && (signal.name.starts_with("qpos[") || signal.name.starts_with("qvel["))
            && replay.world_hash != source.world_hash
        {
            // Индекс координаты относится к конкретной модели движка.
            return None;
        }
        let series = replay
            .series
            .iter()
            .find(|s| s.name == signal.name && s.unit == signal.unit && s.kind == signal.kind)?;
        let sample = (at <= replay.end)
            .then(|| series.at(at))
            .flatten()
            .filter(|s| s.valid);
        Some((series, sample))
    }

    pub(super) fn frame(
        &mut self,
        at: Time,
        view: View<'_>,
        size: [u32; 2],
        force_name: &str,
    ) -> Result<Option<SlotFrame<'_>>, String> {
        let Some(scene) = &mut self.scene else {
            return Ok(None);
        };
        let replay = self.replay.as_ref();
        if replay.is_some_and(|r| at > r.end) {
            return Ok(None);
        }
        let force = replay.and_then(|r| {
            r.series
                .iter()
                .find(|s| s.name == force_name && s.unit == "N")?
                .force(at)
        });
        let pose_at = replay.map(|r| scene.seek(r, at)).transpose()?.unwrap_or(at);
        if let View::Sensor(sensor) = view {
            if let Some(frame) = replay.and_then(|r| r.image(sensor, at)) {
                return Ok(Some(SlotFrame::Recorded(frame, force)));
            }
            if replay.is_some_and(|r| !r.sensor_has_image(sensor))
                || !scene.sensors().any(|name| name == sensor)
            {
                return Ok(None);
            }
        }
        let reactions = replay
            .and_then(|r| r.wrenches.get(&pose_at))
            .map_or(&[][..], Vec::as_slice);
        let camera = scene.camera(view, size)?;
        let frame = scene.render(
            &camera,
            force.as_ref(),
            reactions,
            replay.map_or(0.0, |r| r.wrench_scale),
        )?;
        Ok(Some(SlotFrame::Rendered(frame)))
    }
}

type Completed = Result<(Report, Option<Replay>), String>;

struct Worker {
    slot: usize,
    seek_worst: bool,
    campaign_repeat: bool,
    handle: JoinHandle<Completed>,
}

struct Studio {
    slots: [Slot; 2],
    active: usize,
    world: String,
    task: String,
    case: String,
    matrix: bool,
    duration: String,
    catalog: String,
    campaign: String,
    campaign_measurements: String,
    journal: String,
    report_path: String,
    suite: Option<Suite>,
    participant: Option<Participant>,
    inspect_body: String,
    placements: BTreeMap<String, sim_world::Pose>,
    worker: Option<Worker>,
    campaign_report: Option<Report>,
    campaign_repeat: Option<Report>,
    imported_report: Option<Report>,
    campaign_report_source: String,
    campaign_report_files: Option<(PathBuf, Option<PathBuf>)>,
    assembly: String,
    message: String,
    sensor: String,
    force: String,
    time: Time,
    time_input: String,
    playing: bool,
    azimuth: f64,
    elevation: f64,
    zoom: f64,
    dirty: bool,
    /// Размер обзорного кадра в точках экрана и их плотность. Панель задаёт его
    /// по ширине колонки, без панели остаётся SIZE.
    render: [u32; 2],
    pixels_per_point: f32,
}

impl Studio {
    fn new() -> Self {
        Self {
            slots: Default::default(),
            active: 0,
            world: String::new(),
            task: String::new(),
            case: String::new(),
            matrix: false,
            // Горизонт предварительного просмотра в секундах, пользователь может изменить.
            duration: "1".into(),
            catalog: String::new(),
            campaign: String::new(),
            campaign_measurements: String::new(),
            journal: String::new(),
            report_path: String::new(),
            suite: None,
            participant: None,
            inspect_body: String::new(),
            placements: BTreeMap::new(),
            worker: None,
            campaign_report: None,
            campaign_repeat: None,
            imported_report: None,
            campaign_report_source: String::new(),
            campaign_report_files: None,
            assembly: String::new(),
            message: String::new(),
            sensor: String::new(),
            force: String::new(),
            time: Time::ZERO,
            time_input: String::new(),
            playing: false,
            azimuth: -1.0,
            elevation: 0.5,
            zoom: 1.0,
            dirty: true,
            render: SIZE,
            pixels_per_point: 1.0,
        }
    }

    fn select_entry(&mut self, index: usize) {
        let Some(entry) = self
            .suite
            .as_ref()
            .and_then(|suite| suite.entries.get(index))
        else {
            return;
        };
        self.world = entry.world.to_string_lossy().into_owned();
        self.task = entry.task.to_string_lossy().into_owned();
        self.case.clone_from(&entry.case);
        self.matrix = false;
        self.participant.clone_from(&entry.participant);
        self.inspect_body.clone_from(&entry.inspect_body);
        self.placements.clone_from(&entry.placements);
        self.assembly.clone_from(&entry.id);
        self.force = entry.force.clone().unwrap_or_default();
        self.sensor.clear();
        self.message = self.open_world().err().unwrap_or_default();
    }

    fn open_world(&mut self) -> Result<(), String> {
        let scene = Scene::open(
            Path::new(&self.world),
            self.participant.as_ref(),
            &self.placements,
        )?;
        self.slots[self.active] = Slot {
            scene: Some(scene),
            ..Default::default()
        };
        self.set_time(Time::ZERO);
        Ok(())
    }

    fn input(&self) -> Result<Input, String> {
        if self.world.trim().is_empty() {
            return Err("укажите файл мира".into());
        }
        let world = PathBuf::from(&self.world);
        let catalog = (!self.catalog.is_empty()).then(|| PathBuf::from(&self.catalog));
        if self.task.is_empty() {
            Ok(Input::World {
                world,
                duration: seconds(&self.duration)?,
                catalog,
            })
        } else {
            Ok(Input::Task {
                world,
                task: PathBuf::from(&self.task),
                case: (!self.matrix && !self.case.is_empty()).then(|| self.case.clone()),
                catalog,
                matrix: self.matrix,
            })
        }
    }

    fn spawn(&mut self, input: Input) -> Result<(), String> {
        if self.worker.is_some() {
            return Err("дождитесь завершения текущего прогона".into());
        }
        if matches!(input, Input::World { .. } | Input::Task { .. }) {
            self.open_world()?;
            self.slots[self.active].source = Some(input.clone());
        } else if let Input::Campaign {
            manifest,
            repeat,
            measurements,
        } = &input
        {
            self.campaign_repeat = None;
            if repeat.is_none() {
                self.campaign_report = None;
                self.campaign_report_source = manifest.to_string_lossy().into_owned();
                self.campaign_report_files = Some((manifest.clone(), measurements.clone()));
            }
        }
        self.spawn_worker(input, false)
    }

    fn campaign_input(&self, repeat: Option<String>) -> Input {
        // Для повтора отчёта из окна берём пути исходного запуска, даже если поля уже изменены.
        let (manifest, measurements) = self
            .campaign_report_files
            .as_ref()
            .filter(|_| repeat.is_some())
            .cloned()
            .unwrap_or_else(|| {
                let measurements = (!self.campaign_measurements.is_empty())
                    .then(|| PathBuf::from(&self.campaign_measurements));
                (PathBuf::from(&self.campaign), measurements)
            });
        Input::Campaign {
            manifest,
            repeat,
            measurements,
        }
    }

    fn spawn_worker(&mut self, input: Input, seek_worst: bool) -> Result<(), String> {
        let campaign_repeat = matches!(
            input,
            Input::Campaign {
                repeat: Some(_),
                ..
            }
        );
        let peak = match &input {
            Input::Task { world, task, case: Some(case), .. } => self.suite.as_ref()
                .and_then(|suite| suite.entries.iter().find(|e| &e.world == world && &e.task == task && &e.case == case))
                .and_then(|entry| entry.peak_signal().map(str::to_owned)),
            _ => None,
        };
        let handle = thread::Builder::new()
            .name("конвейер".into())
            .spawn(move || run_with_peak(&input, peak.as_deref())?.into_replay())
            .map_err(|e| e.to_string())?;
        self.worker = Some(Worker {
            slot: self.active,
            seek_worst,
            campaign_repeat,
            handle,
        });
        self.message = "Конвейер выполняет прогон".into();
        Ok(())
    }

    fn finish(&mut self) -> Result<(), String> {
        if !self
            .worker
            .as_ref()
            .is_some_and(|worker| worker.handle.is_finished())
        {
            return Ok(());
        }
        let worker = self.worker.take().expect("проверено завершение");
        self.apply_worker(worker)
    }

    fn apply_worker(&mut self, worker: Worker) -> Result<(), String> {
        let index = worker.slot;
        let (report, replay) = worker
            .handle
            .join()
            .map_err(|_| "авария потока конвейера")??;
        self.message.clear();
        if report.assemblies().next().is_some() {
            if worker.campaign_repeat {
                if let Some(replay) = replay {
                    if replay.campaign.is_none() {
                        return Err("нет проверенной строки повтора".into());
                    }
                    self.put_replay(index, replay, true)?;
                    self.slots[index].source = None;
                    self.slots[index].report = None;
                    self.sensor.clear();
                } else {
                    self.slots[index] = Slot::default();
                    self.set_time(Time::ZERO);
                    self.message = report.view_unavailable()
                        .ok_or("нет причины отсутствия пакета просмотра")?.into();
                }
                self.campaign_repeat = Some(report);
            } else {
                if !report.assemblies().any(|id| id == self.assembly) {
                    self.assembly = report.assemblies().next().unwrap_or_default().into();
                }
                self.campaign_report = Some(report);
            }
        } else {
            if let Some(replay) = replay {
                let strict = matches!(self.slots[index].source, Some(Input::World { .. }));
                self.put_replay(index, replay, strict)?;
            }
            self.slots[index].report = Some(report);
            if worker.seek_worst {
                self.seek_worst_time(index)?;
            }
        }
        self.dirty = true;
        Ok(())
    }

    fn jump_to_worst(&mut self) -> Result<(), String> {
        let slot = &self.slots[self.active];
        if slot.replay.as_ref().is_some_and(|r| r.recorded_scene.is_some()) {
            return self.seek_worst_time(self.active);
        }
        let target = slot
            .report
            .as_ref()
            .and_then(Report::worst_variant)
            .ok_or("в отчёте нет варианта худшего случая")?;
        let Some(Input::Task {
            world,
            task,
            case,
            catalog,
            ..
        }) = &slot.source
        else {
            return Err("сначала запустите задачу в этом слоте".into());
        };
        if case.as_deref() == Some(target) && slot.replay.is_some() {
            return self.seek_worst_time(self.active);
        }
        let input = Input::Task {
            world: world.clone(),
            task: task.clone(),
            case: Some(target.to_owned()),
            catalog: catalog.clone(),
            matrix: false,
        };
        self.spawn_worker(input.clone(), true)?;
        let slot = &mut self.slots[self.active];
        slot.source = Some(input);
        slot.replay = None;
        slot.report = None;
        slot.texture = None;
        self.playing = false;
        self.dirty = true;
        Ok(())
    }

    fn seek_worst_time(&mut self, index: usize) -> Result<(), String> {
        let slot = &self.slots[index];
        let replay = slot
            .replay
            .as_ref()
            .ok_or("журнал худшего варианта не выгружен")?;
        if let Some(binding) = &replay.campaign {
            let at = binding.playback.event.as_ref()
                .ok_or("в оценщике нет привязанного события для этой сетки")?.at_ns;
            self.set_time(Time(at));
            return Ok(());
        }
        if replay.recorded_scene.is_some() {
            let at = replay.peak.as_ref().ok_or("в журнале нет измеренного события для перехода")?.at_ns;
            self.set_time(Time(at));
            return Ok(());
        }
        let report = slot.report.as_ref().ok_or("нет отчёта прогона")?;
        let Some(Input::Task {
            case: Some(case), ..
        }) = &slot.source
        else {
            return Err("вариант журнала не определён".into());
        };
        if report.worst_variant() != Some(case.as_str()) {
            return Err("худший вариант отчёта отличается от варианта журнала".into());
        }
        let ns = report
            .worst_time(None)
            .ok_or("конвейер не записал момент худшего события")?;
        if Time(ns) > replay.end {
            return Err("момент худшего события за границей журнала".into());
        }
        self.set_time(Time(ns));
        Ok(())
    }

    fn set_time(&mut self, time: Time) {
        self.time = time;
        self.time_input = time.0.to_string();
        self.playing = false;
        self.dirty = true;
    }

    fn put_replay(&mut self, index: usize, replay: Replay, strict: bool) -> Result<(), String> {
        if let Some(report) = &self.campaign_report {
            verify_playback(report, &replay)?;
        }
        let slot = &mut self.slots[index];
        let mut exact = replay.recorded_scene.as_ref().map(Scene::from_recorded).transpose()?;
        let scene = exact.as_mut().or(slot.scene.as_mut()).ok_or("откройте сцену журнала")?;
        let matches = scene.same_world(&replay)?;
        if (strict || replay.recorded_scene.is_some()) && !matches {
            return Err("отпечаток модели журнала отличается от открытого мира".into());
        }
        scene.seek(&replay, Time::ZERO)?;
        if exact.is_some() {
            slot.scene = exact;
        }
        slot.fingerprint_matches = matches;
        let event = replay.campaign.as_ref().and_then(|binding| binding.playback.event.as_ref());
        let time = event.map_or(Time::ZERO, |event| Time(event.at_ns));
        if replay.campaign.is_some() {
            self.message = event.map_or_else(
                || "В оценщике нет привязанного события для этой сетки".into(),
                |event| format!("{} · {} нс", event.label, event.at_ns),
            );
        }
        slot.replay = Some(replay);
        self.set_time(time);
        Ok(())
    }

    fn end(&self) -> Time {
        self.slots
            .iter()
            .filter_map(|slot| slot.replay.as_ref().map(|r| r.end))
            .max()
            .unwrap_or(Time::ZERO)
    }

    fn redraw(&mut self, ctx: &egui::Context) -> Result<(), String> {
        if !self.dirty {
            return Ok(());
        }
        self.dirty = false;
        let view = if self.sensor.is_empty() {
            View::Overview {
                azimuth: self.azimuth,
                elevation: self.elevation,
                zoom: self.zoom,
            }
        } else {
            View::Sensor(&self.sensor)
        };
        for slot in &mut self.slots {
            slot.texture = None;
        }
        for slot in &mut self.slots {
            let image = match slot.frame(self.time, view, self.render, &self.force)? {
                Some(SlotFrame::Recorded(frame, force)) => display_image(frame, force.as_ref())?,
                Some(SlotFrame::Rendered(frame)) => {
                    let size = frame.size.map(|n| n as usize);
                    egui::ColorImage::from_rgb(size, frame.rgb.as_flattened())
                }
                None => continue,
            };
            slot.texture =
                Some(ctx.load_texture("кадр прогона", image, egui::TextureOptions::LINEAR));
        }
        Ok(())
    }
}

impl Drop for Studio {
    fn drop(&mut self) {
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
    }
}

pub(super) fn display_image(
    frame: &sim_signal::ImageFrame,
    force: Option<&Force>,
) -> Result<egui::ColorImage, String> {
    if frame.width == 0 || frame.height == 0 || !frame.is_consistent() {
        return Err("размер кадра не соответствует записанным пикселям".into());
    }
    let size = [frame.width as usize, frame.height as usize];
    let mut image = match frame.format {
        PixelFormat::Rgb8 => egui::ColorImage::from_rgb(size, &frame.data),
        PixelFormat::DepthMm16 => {
            let depth = || {
                frame
                    .data
                    .chunks_exact(2)
                    .map(|b| u16::from_le_bytes([b[0], b[1]]))
            };
            let far = depth().max().filter(|&v| v > 0).unwrap_or(u16::MAX);
            let pixels = depth()
                .map(|v| {
                    if v == 0 {
                        egui::Color32::TRANSPARENT
                    } else {
                        egui::Color32::from_gray(
                            (u32::from(v) * u32::from(u8::MAX) / u32::from(far)) as u8,
                        )
                    }
                })
                .collect();
            egui::ColorImage::new(size, pixels)
        }
    };
    if let Some(force) = force {
        // Полоса рядом с пикселями датчика, которые сохраняются без изменений.
        let length = force.pixels(size[0])?;
        let [r, g, b] = super::render::FORCE_COLOR;
        for _ in 0..super::render::FORCE_ROWS {
            image.pixels.extend(std::iter::repeat_n(
                egui::Color32::from_rgb(r, g, b),
                length,
            ));
            image
                .pixels
                .extend(std::iter::repeat_n(egui::Color32::BLACK, size[0] - length));
        }
        image.size[1] += super::render::FORCE_ROWS;
        image.source_size.y = image.size[1] as f32;
    }
    Ok(image)
}

pub(crate) fn start(args: Vec<OsString>) -> AppExit {
    match settings(args) {
        Ok(studio) => App::new()
            .insert_non_send(studio)
            .add_plugins(DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Мастерская · прогоны платформы".into(),
                    resolution: bevy::window::WindowResolution::new(1360, 860),
                    ..default()
                }),
                ..default()
            }))
            .add_plugins(EguiPlugin::default())
            .add_systems(Startup, |mut commands: Commands| {
                commands.spawn((Camera2d, PrimaryEguiContext));
            })
            .add_systems(EguiPrimaryContextPass, panel)
            .run(),
        Err(error) => {
            eprintln!("{error}");
            AppExit::error()
        }
    }
}

fn settings(args: Vec<OsString>) -> Result<Studio, String> {
    let mut studio = Studio::new();
    let mut args = args.into_iter();
    let mut seen = BTreeSet::new();
    let mut selected = None;
    while let Some(flag) = args.next() {
        let flag = flag.into_string().map_err(|_| "ключ должен быть в UTF-8")?;
        if !seen.insert(flag.clone()) {
            return Err(format!("повтор ключа {flag}"));
        }
        let value = args
            .next()
            .and_then(|s| s.into_string().ok())
            .filter(|s| !s.is_empty() && !s.starts_with("--"))
            .ok_or_else(|| format!("нет значения для {flag}"))?;
        match flag.as_str() {
            "--platform" => studio.world = value,
            "--suite" => studio.suite = Some(Suite::load(Path::new(&value))?),
            "--entry" => selected = Some(value),
            "--task" => studio.task = value,
            "--case" => studio.case = value,
            "--catalog" => studio.catalog = value,
            "--duration" => {
                seconds(&value)?;
                studio.duration = value;
            }
            "--journal" => studio.journal = value,
            "--report" => studio.report_path = value,
            "--campaign" => studio.campaign = value,
            _ => return Err(format!("неизвестный ключ {flag}")),
        }
    }
    if let Some(suite) = &studio.suite {
        if !studio.world.is_empty() || !studio.task.is_empty() || !studio.case.is_empty() {
            return Err("--suite требует выбрать мир через --entry".into());
        }
        if studio.campaign.is_empty() {
            studio.campaign = suite.campaign.to_string_lossy().into_owned();
        }
        let index = selected
            .as_ref()
            .map_or(Some(0), |id| suite.entries.iter().position(|e| &e.id == id))
            .ok_or("мир с таким id отсутствует в списке")?;
        studio.select_entry(index);
        if !studio.message.is_empty() {
            return Err(std::mem::take(&mut studio.message));
        }
    } else if selected.is_some() {
        return Err("--entry требует --suite".into());
    } else if !studio.world.is_empty() {
        studio.open_world()?;
    }
    if !studio.journal.is_empty() {
        open_journal(&mut studio)?;
    }
    if !studio.report_path.is_empty() {
        open_report(&mut studio)?;
    }
    Ok(studio)
}

fn open_journal(st: &mut Studio) -> Result<(), String> {
    let bytes = std::fs::read(&st.journal).map_err(|e| e.to_string())?;
    let replay = if bytes.starts_with(b"SIMVIEW") {
        Replay::from_view(sim_cli::view::RunView::read(&bytes)?)?
    } else {
        Replay::read(&bytes)?
    };
    st.message.clear();
    st.put_replay(st.active, replay, true)?;
    // У отдельно открытого журнального файла нет отчёта, связанного конвейером.
    st.slots[st.active].report = None;
    st.slots[st.active].source = None;
    Ok(())
}

#[cfg(test)]
pub(super) fn open_at_peak(replay: Replay) -> Result<(Slot, Time), String> {
    let mut studio = Studio::new();
    studio.put_replay(0, replay, true)?;
    studio.jump_to_worst()?;
    Ok((std::mem::take(&mut studio.slots[0]), studio.time))
}

fn open_report(st: &mut Studio) -> Result<(), String> {
    let report = Report::parse(read_text(Path::new(&st.report_path))?)?;
    if report.assemblies().next().is_some() && report.repeat_key().is_none() {
        if let Some(replay) = &st.slots[st.active].replay {
            verify_playback(&report, replay)?;
        }
        if !report.assemblies().any(|id| id == st.assembly) {
            st.assembly = report.assemblies().next().unwrap_or_default().into();
        }
        st.campaign_report_source.clone_from(&st.report_path);
        st.campaign_report_files = None;
        st.campaign_report = Some(report);
        st.campaign_repeat = None;
    } else {
        st.imported_report = Some(report);
    }
    Ok(())
}

fn verify_playback(report: &Report, replay: &Replay) -> Result<(), String> {
    if let Some(actual) = &replay.campaign
        && report.playback(&actual.key).is_none_or(|expected| {
            expected.input != actual.input || expected.playback != actual.playback
        })
    {
        return Err("повтор не совпадает с исходным отчётом кампании".into());
    }
    Ok(())
}

fn field(ui: &mut egui::Ui, title: &str, value: &mut String) -> bool {
    ui.label(title);
    ui.add(egui::TextEdit::singleline(value).desired_width(f32::INFINITY))
        .changed()
}

fn show_report(ui: &mut egui::Ui, report: &Report, assembly: Option<&str>) {
    let verdict = report.verdict(assembly);
    let color = match verdict {
        Verdict::Rejected => egui::Color32::LIGHT_RED,
        Verdict::NotProven | Verdict::NotAnalyzed => egui::Color32::YELLOW,
        Verdict::ProvenInSimulation => egui::Color32::LIGHT_BLUE,
    };
    ui.colored_label(color, egui::RichText::new(verdict.text()).strong());
    ui.label(format!("Статус конвейера · {}", report.status()));
    if let Some(outcome) = report.outcome() {
        ui.label(format!("Исход задачи · {outcome}"));
    }
    if let Some(readiness) = assembly.and_then(|id| report.readiness(id)) {
        ui.label(format!("Готовность по отчёту · {readiness}"));
    }
    if let Some(worst) = report.worst(assembly) {
        ui.label(format!("Худший случай · {worst}"));
    }
    let reasons: BTreeSet<_> = report.reasons(assembly).into_iter().collect();
    ui.collapsing(
        format!("Причины из отчёта ({})", reasons.len()),
        |ui| {
            for reason in reasons {
                ui.label(reason);
            }
        },
    );
}

fn panel(mut contexts: EguiContexts, mut st: NonSendMut<Studio>, time: Res<TimeSource>) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    if let Err(e) = st.finish() {
        st.message = e;
    }
    if st.worker.is_some() {
        ctx.request_repaint();
    }
    let end = st.end();
    if st.playing {
        // Часы интерфейса влияют только на положение курсора при воспроизведении.
        let elapsed = u64::try_from(time.delta().as_nanos()).unwrap_or(u64::MAX);
        st.time = Time(st.time.0.saturating_add(elapsed)).min(end);
        st.time_input = st.time.0.to_string();
        st.playing = st.time < end;
        st.dirty = true;
        if st.playing {
            ctx.request_repaint();
        }
    }
    let mut root = egui::Ui::new(
        ctx.clone(),
        "platform_root".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );
    egui::Panel::left("входы")
        .default_size(315.0)
        .show(&mut root, |ui| {
            ui.heading("Мир и задачи");
            egui::ScrollArea::vertical().show(ui, |ui| {
                let mut entry = None;
                ui.add_enabled_ui(st.worker.is_none(), |ui| {
                    if let Some(suite) = &st.suite {
                        egui::ComboBox::from_label("Список миров")
                            .selected_text(&st.assembly)
                            .show_ui(ui, |ui| {
                                for (index, e) in suite.entries.iter().enumerate() {
                                    if ui
                                        .selectable_label(
                                            st.world == e.world.to_string_lossy(),
                                            &e.label,
                                        )
                                        .clicked()
                                    {
                                        entry = Some(index);
                                    }
                                }
                            });
                    }
                    if let Some(index) = entry {
                        st.select_entry(index);
                    }
                    ui.horizontal(|ui| {
                        ui.label("Запись");
                        ui.selectable_value(&mut st.active, 0, "А");
                        ui.selectable_value(&mut st.active, 1, "Б");
                    });
                    if field(ui, "Файл мира", &mut st.world) {
                        st.participant = None;
                        st.inspect_body.clear();
                        st.placements.clear();
                    }
                    field(ui, "Каталог моделей", &mut st.catalog);
                    if ui.button("Открыть мир").clicked() {
                        st.sensor.clear();
                        st.message = st.open_world().err().unwrap_or_default();
                    }
                    field(ui, "Горизонт обзора, с", &mut st.duration);
                    field(ui, "Файл задачи", &mut st.task);
                    ui.checkbox(&mut st.matrix, "Полная матрица контура");
                    st.matrix.then(|| ui.label("Для просмотра отдельной строки запустите повтор из отчёта кампании"));
                    ui.add_enabled_ui(!st.matrix, |ui| {
                        field(ui, "Вариант задачи", &mut st.case);
                    });
                    if ui.button("Запустить").clicked()
                        && let Err(e) = st.input().and_then(|input| st.spawn(input))
                    {
                        st.message = e;
                    }
                    ui.separator();
                    field(ui, "Файл журнала", &mut st.journal);
                    if ui.button("Открыть журнал").clicked() {
                        if let Err(error) = open_journal(&mut st) {
                            st.message = error;
                        }
                    }
                    field(ui, "Файл отчёта JSON", &mut st.report_path);
                    if ui.button("Открыть отчёт").clicked() {
                        st.message = open_report(&mut st).err().unwrap_or_default();
                    }
                    field(ui, "Файл кампании", &mut st.campaign);
                    field(
                        ui,
                        "Файл заверенных замеров (необязательно)",
                        &mut st.campaign_measurements,
                    );
                    if ui.button("Запустить кампанию").clicked() {
                        let input = st.campaign_input(None);
                        if let Err(e) = st.spawn(input) {
                            st.message = e;
                        }
                    }
                });
                if !st.message.is_empty() {
                    ui.label(&st.message);
                }
                let mut replay_worst = None;
                let mut variant_worst = false;
                if let Some(report) = &st.slots[st.active].report {
                    ui.separator();
                    show_report(ui, report, None);
                }
                let slot = &st.slots[st.active];
                if let Some(replay) = slot.replay.as_ref().filter(|r| r.recorded_scene.is_some()) {
                    if let Some(binding) = &replay.campaign {
                        if binding.playback.event.is_some() {
                            variant_worst = ui.button("К событию из отчёта").clicked();
                        }
                    } else if let Some(peak) = &replay.peak {
                        ui.label(format!("{} · {} {} · {} нс", peak.signal, peak.value, peak.unit, peak.at_ns));
                        variant_worst = ui.add_enabled(st.worker.is_none(), egui::Button::new("К максимуму ряда")).clicked();
                    } else {
                        ui.label("Максимум ряда в журнале отсутствует");
                    }
                } else if slot.report.as_ref().and_then(Report::worst_variant).is_some() {
                    variant_worst = ui.add_enabled(st.worker.is_none(), egui::Button::new("К худшему варианту задачи")).clicked();
                }
                if let Some(report) = &st.campaign_report {
                    ui.separator();
                    ui.label(format!("Отчёт кампании · {}", st.campaign_report_source));
                    let mut assembly = st.assembly.clone();
                    egui::ComboBox::from_label("Сборка отчёта")
                        .selected_text(&assembly)
                        .show_ui(ui, |ui| {
                            for id in report.assemblies() {
                                ui.selectable_value(&mut assembly, id.to_owned(), id);
                            }
                        });
                    show_report(ui, report, Some(&assembly));
                    if ui
                        .add_enabled(
                            st.worker.is_none()
                                && report.worst(Some(&assembly)).is_some()
                                && (st.campaign_report_files.is_some() || !st.campaign.is_empty()),
                            egui::Button::new("Повторить худший прогон кампании"),
                        )
                        .clicked()
                    {
                        replay_worst = report.worst(Some(&assembly)).map(str::to_owned);
                    }
                    st.assembly = assembly;
                }
                if let Some(report) = &st.campaign_repeat {
                    ui.separator();
                    ui.label(format!(
                        "Отчёт повтора · {}",
                        report.repeat_key().unwrap_or_default()
                    ));
                    show_report(ui, report, Some(&st.assembly));
                }
                if let Some(report) = &st.imported_report {
                    ui.separator();
                    ui.label("Загруженный отчёт без привязки к журналу");
                    show_report(ui, report, None);
                }
                if variant_worst {
                    st.message = st.jump_to_worst().err().unwrap_or_default();
                }
                if let Some(key) = replay_worst {
                    let input = st.campaign_input(Some(key));
                    if let Err(e) = st.spawn(input) {
                        st.message = e;
                    }
                }
            });
        });
    egui::CentralPanel::default().show(&mut root, |ui| {
        let end = st.end();
        let mut time_ns = st.time.0;
        ui.horizontal(|ui| {
            if ui.add_enabled(end > Time::ZERO,egui::Button::new(if st.playing { "Пауза" } else { "Играть" })).clicked() {
                if st.time == end { st.set_time(Time::ZERO); time_ns = 0; }
                st.playing = !st.playing;
                st.dirty = true;
                ctx.request_repaint();
            }
            if ui.add(egui::Slider::new(&mut time_ns,0..=end.0).show_value(false)).changed() {
                st.set_time(Time(time_ns));
            }
            let input = ui.add(egui::TextEdit::singleline(&mut st.time_input).desired_width(160.0));
            ui.label("нс");
            if input.changed() {
                st.playing = false;
            }
            if input.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                match st.time_input.trim().parse::<u64>() {
                    Ok(ns) if Time(ns) <= end => st.set_time(Time(ns)),
                    _ => st.message = "Введите целое время в границах журналов".into(),
                }
            }
        });
        choose_views(ui,&mut st);
        // Кадр рисуется в точках экрана под ширину колонки, иначе на ретине он растянут вдвое.
        let columns = st.slots.iter().filter(|slot| slot.scene.is_some()).count().max(1) as f32;
        let ppp = ctx.pixels_per_point();
        let width = ((ui.available_width() / columns - 8.0).max(160.0) * ppp).round() as u32;
        let width = width.min(MAX_WIDTH) & !1;
        let render = [width, width * SIZE[1] / SIZE[0]];
        if render != st.render || ppp != st.pixels_per_point {
            st.render = render;
            st.pixels_per_point = ppp;
            st.dirty = true;
        }
        if let Err(e) = st.redraw(ctx) { st.message = e; }
        egui::ScrollArea::vertical().show(ui, |ui| {
            let slots: Vec<_> = st.slots.iter().enumerate().filter(|(_, slot)| slot.scene.is_some()).collect();
            if slots.is_empty() { ui.label("Откройте файл мира"); }
            ui.columns(slots.len().max(1), |columns| {
                for (ui,(index,slot)) in columns.iter_mut().zip(slots) {
                    let scene = slot.scene.as_ref().unwrap();
                    ui.heading(format!("{} · {}",["А","Б"][index],scene.name()));
                    if scene.uses_kinematics() {
                        ui.label("Положения тел и центры масс · поверхности звеньев отсутствуют");
                    }
                    if let Some(replay) = &slot.replay {
                        ui.label(format!("Зерно {} · журнал {:016x} · {:?}",replay.seed,replay.journal,replay.status));
                        if let Some(binding) = &replay.campaign {
                            ui.label(format!("Повтор {} · {:?} · {:?}", binding.key, binding.verdict, binding.readiness));
                            ui.label(format!("Исполнено {} из {}", binding.coverage.executed, binding.coverage.required));
                            let p = &binding.playback;
                            ui.label(format!("{} / {} · {} Гц", p.child.mode, p.child.path, p.physics_hz));
                            if let Some(event) = &p.event {
                                ui.label(format!("{} · {} нс", event.label, event.at_ns));
                            } else {
                                ui.label("В оценщике нет привязанного события для этой сетки");
                            }
                        }
                        if !slot.fingerprint_matches { ui.colored_label(egui::Color32::YELLOW,"Поза из журнала на проектной геометрии. Точная сцена прогона не выгружена"); }
                    }
                    if let Some(texture) = &slot.texture {
                        // Обзорный кадр нарисован в точках экрана, записанный кадр камеры — в своих пикселях.
                        let rendered = texture.size() == st.render.map(|n| n as usize);
                        let size = texture.size_vec2() / if rendered { st.pixels_per_point } else { 1.0 };
                        let width = ui.available_width().min(size.x);
                        ui.image((texture.id(),size*(width/size.x)));
                    }
                    ui.push_id(index, |ui| show_samples(ui,slot,&st.sensor,&st.force,st.time));
                }
            });
            comparison(ui,&st);
        });
    });
}

type TimeSource = bevy::time::Time<bevy::time::Real>;

fn choose_views(ui: &mut egui::Ui, st: &mut Studio) {
    let sensors: BTreeSet<_> = st
        .slots
        .iter()
        .filter_map(|s| s.scene.as_ref())
        .flat_map(Scene::sensors)
        .chain(st.slots.iter().filter_map(|s| s.replay.as_ref()).flat_map(Replay::sensor_names))
        .map(str::to_owned)
        .collect();
    let forces: BTreeSet<_> = st
        .slots
        .iter()
        .filter_map(|s| s.replay.as_ref())
        .flat_map(|r| &r.series)
        .filter(|s| s.unit == "N")
        .map(|s| s.name.clone())
        .collect();
    ui.horizontal(|ui| {
        egui::ComboBox::from_label("Вид")
            .selected_text(if st.sensor.is_empty() {
                "Обзор"
            } else {
                &st.sensor
            })
            .show_ui(ui, |ui| {
                st.dirty |= ui
                    .selectable_value(&mut st.sensor, String::new(), "Обзор")
                    .changed();
                for sensor in sensors {
                    st.dirty |= ui
                        .selectable_value(&mut st.sensor, sensor.clone(), sensor)
                        .changed();
                }
            });
        egui::ComboBox::from_label("Сила")
            .selected_text(if st.force.is_empty() {
                "Не выбрана"
            } else {
                &st.force
            })
            .show_ui(ui, |ui| {
                st.dirty |= ui
                    .selectable_value(&mut st.force, String::new(), "Скрыть")
                    .changed();
                for name in forces {
                    st.dirty |= ui
                        .selectable_value(&mut st.force, name.clone(), name)
                        .changed();
                }
            });
    });
    if st.sensor.is_empty() {
        if !st.inspect_body.is_empty() && ui.button("Объект крупно").clicked() {
            let active = st.active;
            let body = st.inspect_body.clone();
            if let Some(scene) = st.slots[active].scene.as_mut() {
                match scene.focus(&body) {
                    Ok(()) => {
                        st.zoom = 1.0;
                        st.dirty = true;
                    }
                    Err(e) => st.message = e,
                }
            }
        }
        ui.horizontal(|ui| {
            let pi = std::f64::consts::PI;
            // Диапазоны управления свободной камерой, без отношения к модели.
            st.dirty |= ui
                .add(egui::Slider::new(&mut st.azimuth, -pi..=pi).text("Азимут"))
                .changed();
            st.dirty |= ui
                .add(egui::Slider::new(&mut st.elevation, -1.5..=1.5).text("Высота"))
                .changed();
            st.dirty |= ui
                .add(
                    egui::Slider::new(&mut st.zoom, 0.25..=32.0)
                        .logarithmic(true)
                        .text("Масштаб"),
                )
                .changed();
        });
    }
}

fn show_samples(ui: &mut egui::Ui, slot: &Slot, sensor: &str, force: &str, at: Time) {
    let Some(replay) = &slot.replay else {
        ui.label("Журнал прогона отсутствует");
        return;
    };
    if at > replay.end {
        ui.label(format!("Запись закончилась на {} нс", replay.end.0));
        return;
    }
    if !sensor.is_empty() {
        if let Some(frame) = replay.image(sensor, at) {
            ui.label(format!(
                "Доставленный кадр · захват {} нс · доставка {} нс",
                frame.captured.0, frame.at.0
            ));
        } else if replay.sensor_has_image(sensor) {
            ui.label("На этой отметке доставленный кадр недоступен");
            if slot.scene.as_ref().is_some_and(|s| s.sensors().any(|name| name == sensor)) {
                ui.label("Вид из точки установки");
            }
        }
        let mut channels = replay.sensor_series(sensor).peekable();
        if channels.peek().is_none() && !replay.sensor_has_image(sensor) {
            ui.label("У датчика нет канала в журнале этого прогона");
        }
        for series in channels {
            if let Some(sample) = series.at(at).filter(|s| s.valid) {
                ui.label(format!(
                    "{} · {} {} · {} нс",
                    series.name, sample.value, series.unit, sample.at.0
                ));
            } else {
                ui.label(format!("{} · показание недоступно", series.name));
            }
            Plot::new(("датчик", &series.name))
                .height(160.0)
                .x_axis_label("Время доставки, с")
                .y_axis_label(&series.unit)
                .include_x(0.0)
                .include_x(replay.end.as_secs())
                .show(ui, |plot| {
                    plot_series(plot, series, &series.name, egui::Color32::LIGHT_BLUE);
                    plot.vline(VLine::new("Курсор", at.as_secs()));
                });
        }
    }
    if let Some(reason) = &replay.contact_gap {
        ui.label(reason);
    }
    for event in replay.attachments.iter().filter(|e| e.evaluated_ns == at.0) {
        use sim_cli::view::MountOutcome;
        let outcome = match &event.outcome {
            MountOutcome::Transition { from, to } => format!("{from} → {to}"),
            MountOutcome::Equality { active: true } => "Связь включена".into(),
            MountOutcome::Equality { active: false } => "Связь отключена".into(),
            MountOutcome::Rejected { reason } | MountOutcome::NotAnalyzed { reason } => reason.clone(),
        };
        ui.label(format!("{} · {outcome}", event.name));
    }
    let pose_at = replay
        .qpos
        .first()
        .and_then(|&i| replay.series[i].at(at))
        .map_or(at, |p| p.at);
    if let Some(reactions) = replay.wrenches.get(&pose_at) {
        ui.collapsing(
            format!("Пространственные реакции · {} нс", pose_at.0),
            |ui| {
                ui.label(format!(
                    "Масштаб векторов · максимум {:.4} Н",
                    replay.wrench_scale
                ));
                for w in reactions {
                    let norm = super::Vec3::from(w.force_n).norm();
                    ui.label(format!("{} → {} · {:.4} Н", w.pair[0], w.pair[1], norm));
                }
            },
        );
    } else if force.is_empty() && sensor.is_empty() {
        ui.label("На этой отметке пространственные реакции не записаны");
    }
    if !force.is_empty() {
        let series = replay
            .series
            .iter()
            .find(|s| s.name == force && s.unit == "N");
        if let Some((force, kind)) = series.and_then(|s| Some((s.force(at)?, s.kind))) {
            let role = if kind == SignalKind::Command {
                "Команда силы"
            } else {
                "Сила из журнала"
            };
            ui.label(format!(
                "{role} · {} · {} Н · шкала 0–{} Н",
                force.name, force.value_n, force.scale_n
            ));
        } else {
            ui.label("Сила не записана на этой отметке");
        }
    }
}

fn comparison(ui: &mut egui::Ui, st: &Studio) {
    let Some(base) = st.slots[st.active].replay.as_ref() else {
        return;
    };
    let id = egui::Id::new("сравнение");
    let mut selected = ui.data_mut(|d| d.get_temp::<(SignalKind, String, String)>(id));
    egui::ComboBox::from_label("Сигнал для сравнения")
        .selected_text(selected.as_ref().map_or("Выбрать", |(_, name, _)| name))
        .show_ui(ui, |ui| {
            for series in &base.series {
                let kind = match series.kind {
                    SignalKind::Observation => "датчик",
                    SignalKind::Truth => "состояние",
                    SignalKind::Command => "команда",
                    SignalKind::Process => "процесс",
                };
                ui.selectable_value(
                    &mut selected,
                    Some((series.kind, series.name.clone(), series.unit.clone())),
                    format!("{} · {} · {kind}", series.name, series.unit),
                );
            }
        });
    let Some((kind, name, unit)) = selected else {
        return;
    };
    let Some(series) = base
        .series
        .iter()
        .find(|s| s.kind == kind && s.name == name && s.unit == unit)
    else {
        return;
    };
    ui.data_mut(|d| d.insert_temp(id, (kind, name, unit)));
    let cursors: Vec<_> = st
        .slots
        .iter()
        .enumerate()
        .filter_map(|(index, slot)| {
            let (candidate, sample) = slot.sample(base, series, st.time)?;
            Some((index, candidate, sample))
        })
        .collect();
    ui.horizontal(|ui| {
        for (index, _, sample) in &cursors {
            let value = sample.map_or("нет отсчёта".into(), |s| {
                format!("{} {}", s.value, series.unit)
            });
            ui.label(format!("{} · {value}", ["А", "Б"][*index]));
        }
    });
    // Ограничение числа точек относится к предпросмотру. Значение курсора берётся из полного ряда.
    Plot::new("кривые")
        .height(160.0)
        .x_axis_label("Время, с")
        .y_axis_label(&series.unit)
        .show(ui, |plot| {
            for (index, candidate, _) in cursors {
                plot_series(plot, candidate, ["А", "Б"][index],
                    [egui::Color32::LIGHT_BLUE, egui::Color32::LIGHT_GREEN][index]);
            }
            plot.vline(VLine::new("Курсор", st.time.as_secs()));
        });
}

fn plot_series(plot: &mut egui_plot::PlotUi<'_>, series: &Series, label: &str, color: egui::Color32) {
    // Бюджет отрисовки. В каждом интервале сохраняются границы и оба экстремума.
    let stride = series.points.len().div_ceil(2000).max(1);
    for group in series.points.split(|sample| !sample.valid).filter(|group| !group.is_empty()) {
        let mut points: Vec<[f64; 2]> = Vec::new();
        for chunk in group.chunks(stride) {
            let min = chunk.iter().enumerate().min_by(|(_, a), (_, b)| a.value.total_cmp(&b.value)).unwrap().0;
            let max = chunk.iter().enumerate().max_by(|(_, a), (_, b)| a.value.total_cmp(&b.value)).unwrap().0;
            let mut indices = [0, min, max, chunk.len() - 1];
            indices.sort_unstable();
            for (i, index) in indices.into_iter().enumerate() {
                if i == 0 || index != indices[i - 1] {
                    let sample = chunk[index];
                    points.push([sample.at.as_secs(), sample.value]);
                }
            }
        }
        if points.len() == 1 {
            plot.points(Points::new(label, points).color(color));
        } else {
            plot.line(Line::new(label, points).color(color));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::{Temp, json, render::FORCE_COLOR, report::text};

    #[test]
    fn заверенная_прочность_показывает_все_три_решения_кампании() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().unwrap();
        let data = root.join("devices/stage8");
        let corpus = root.join("devices/stage6/corpus");
        let temp = Temp::new().unwrap();
        let manifest = temp.0.join("campaign.toml");
        let records = temp.0.join("records.toml");
        let original = read_text(&data.join("decision-campaign.toml")).unwrap();
        let installation = "installation = \"../stage6/corpus/under-travel-install.toml\"";
        assert!(original.contains(installation));
        std::fs::write(&manifest, original
            .replace(installation, "installation = \"installation.toml\"")
            .replace("../stage6/corpus/", &format!("{}/", corpus.display()))
            .replace("acceptance/bounded.toml", &data.join("acceptance/bounded.toml").to_string_lossy())
        ).unwrap();
        let original = read_text(&corpus.join("under-travel-install.toml")).unwrap();
        let original = original.replacen("version = 1", r#"version = 1
measurement_build = { design = "стержень", revision = "r1" }"#, 1);
        // Расчётная проверка остаётся в установке вместе со стендовой.
        std::fs::write(temp.0.join("installation.toml"), format!(r#"{original}
[[checks]]
class = "static_strength"
object = "каретка/стержень"
window = {{ start_ns = 0, end_ns = 4000000, max_gap_ns = 1000000 }}
scope = "Синтетический стенд. Изгиб по всему окну двух посадок."
applicability = {{ kind = "datasheet", reference = "Протокол синтетического стенда из установки хода." }}
inputs = []
verification = {{ kind = "measurement", limit = {{ kind = "range", signal = {{ name = "напряжение", unit = "Pa" }}, max = 5000000.0, source = {{ kind = "datasheet", reference = "Тот же предел 5 МПа, что у проверки сечения этой установки." }} }} }}
"#)).unwrap();

        let mut studio = Studio::new();
        studio.campaign = manifest.to_string_lossy().into_owned();
        let before = super::super::run(&studio.campaign_input(None)).unwrap();
        let raw = json::parse(&before.report.raw).unwrap();
        let rows = raw.get("assemblies").unwrap().get("bounded").unwrap()
            .get("runs").unwrap().object().unwrap();
        let detail = rows.values().next().unwrap().get("detail").unwrap();
        let build = text(detail, "build_fingerprint").unwrap();

        // 2 Н·м и сечение 40×20 мм из установки. σ = M/W, W = bh²/6.
        let stress = 2.0 / (0.04 * 0.02_f64.powi(2) / 6.0);
        let write_records = |origin: &str, revision: &str| {
            std::fs::write(temp.0.join("profile.toml"), format!(r#"
build = {{ design = "стержень", revision = "{revision}" }}
models = ["изгиб"]
[[channels]]
name = "напряжение"
unit = "Pa"
role = "observation"
sampling = {{ kind = "periodic", first_ns = 0, period_ns = 1000000 }}
"#)).unwrap();
            // Четыре шага по 1 мс и начальный отсчёт из окна установки.
            let mut capture = format!("sim-reality,1,{origin}\nbuild,стержень,{revision},4000000\ntime_ns,signal,unit,seq,value,valid\n");
            for tick in 0..=4 {
                capture.push_str(&format!("{},напряжение,Pa,{tick},750000,1\n", tick * 1_000_000));
            }
            capture.push_str("end,5\n");
            let stamp = sim_signal::fnv1a(capture.bytes());
            std::fs::write(temp.0.join("strength.csv"), capture).unwrap();
            let mut document = String::from("version = 1\n");
            for sample in [r#"kind = "nominal""#, r#"kind = "boundary", id = "смещение""#] {
                document.push_str(&format!(r#"
[[records]]
experiment = "bounded/loaded"
run = {{ station = "stand", mode = "loaded", path = "axis", faults = [], grid = "base", sample = {{ {sample} }} }}
build_fingerprint = "{build}"
signal = {{ name = "напряжение", unit = "Pa" }}
# Точный синтетический ряд протокольного теста без шума.
uncertainty = 0.0
profile = "profile.toml"
capture = "strength.csv"
[records.receipt]
build = {{ design = "стержень", revision = "{revision}" }}
log_fingerprint = "{stamp:016x}"
capture_id = "Синтетическая запись теста протокола"
calibration = "Точная шкала синтетической постановки"
scope = "Проверка интерфейса на четырёх интервалах по 1 мс"
"#));
            }
            std::fs::write(&records, document).unwrap();
        };
        studio.campaign_measurements = records.to_string_lossy().into_owned();
        for (origin, revision, reason) in [
            ("synthetic", "r1", "синтетический журнал"),
            ("recorded", "r2", "другой сборке или ревизии"),
        ] {
            write_records(origin, revision);
            match super::super::run(&studio.campaign_input(None)) {
                Err(error) => assert!(error.contains(reason), "{error}"),
                Ok(_) => panic!("ожидался отказ: {reason}"),
            }
        }

        // Имитируем пакет регистратора только во временном каталоге этого теста.
        write_records("recorded", "r1");
        studio.spawn(studio.campaign_input(None)).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        let report = studio.campaign_report.as_ref().unwrap();
        assert_eq!(report.verdict(Some("exceeded")), Verdict::Rejected);
        assert_eq!(report.verdict(Some("estimated")), Verdict::NotProven);
        assert_eq!(report.verdict(Some("bounded")), Verdict::ProvenInSimulation);
        assert_eq!(report.readiness("bounded"), Some("BenchOnly"));
        let assert_strength = |report: &Report, row_key: &str| {
            let raw = json::parse(&report.raw).unwrap();
            let row = raw.get("assemblies").unwrap().get("bounded").unwrap()
                .get("runs").unwrap().get(row_key).unwrap();
            let findings: Vec<_> = row.get("findings").unwrap().array().unwrap().iter()
                .filter(|f| text(f.get("key").unwrap(), "class") == Some("static_strength"))
                .collect();
            assert_eq!(findings.len(), 1);
            assert_eq!(text(findings[0], "outcome"), Some("pass"));
            let bounds = findings[0].get("measurement").unwrap().array().unwrap();
            assert_eq!(bounds.len(), 2);
            for bound in bounds {
                assert!((bound.f64().unwrap() - stress).abs() < stress * f64::EPSILON);
            }
        };
        for key in rows.keys() {
            assert_strength(report, key);
        }
        let key = report.worst(Some("bounded")).unwrap().to_owned();
        studio.campaign.clear();
        studio.campaign_measurements.clear();
        studio.spawn(studio.campaign_input(Some(key.clone()))).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        let repeat = studio.campaign_repeat.as_ref().unwrap();
        assert_eq!(repeat.repeat_key(), Some(key.as_str()));
        assert_eq!(repeat.verdict(Some("bounded")), Verdict::NotProven);
        assert_strength(repeat, &key);
        println!("Кампания с имитацией заверения · Rejected, NotProven, PassedSimulation/BenchOnly, прочность в двух посадках {stress:.0} Па");
    }

    #[test]
    fn новый_блок_проходит_задачу_отчёт_и_просмотр() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../devices/stage8/acceptance/campaign.toml");
        let mut studio = Studio::new();
        studio.campaign = manifest.to_string_lossy().into_owned();
        studio.spawn(studio.campaign_input(None)).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        let report = studio.campaign_report.as_ref().unwrap();
        assert_eq!(report.verdict(Some("block")), Verdict::Rejected);
        let key = report.worst(Some("block")).unwrap().to_owned();
        studio.spawn(studio.campaign_input(Some(key.clone()))).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        // x₀ = 1 мкм, затем шаги Euler 1, 2, 4 и 6 мкм. Предел 8 мкм.
        assert_eq!(studio.time, Time(4_000_000));
        let slot = &mut studio.slots[0];
        let expected_journal = studio.campaign_report.as_ref().unwrap().playback(&key).unwrap().playback.journal;
        assert_eq!(slot.replay.as_ref().unwrap().journal, expected_journal);
        let Some(SlotFrame::Rendered(frame)) = slot.frame(studio.time, View::OVERVIEW, SIZE, "").unwrap() else {
            panic!("отсутствует кадр блока");
        };
        assert!(frame.ids.iter().any(Option::is_some));
        assert!((slot.scene.as_ref().unwrap().sim.qpos()[0] - 14e-6).abs() < 1e-14);
        let mut replay = slot.replay.take().unwrap();
        replay.campaign.as_mut().unwrap().playback.journal ^= 1;
        assert!(studio.put_replay(0, replay, true).unwrap_err().contains("исходным отчётом"));
        println!("Новый блок · Rejected, 14 мкм при пределе 8 мкм, {} нс, журнал {expected_journal:016x}", studio.time.0);
    }

    #[test]
    fn две_задачи_дают_сигнал_со_шкалой_и_общим_курсором() {
        let data = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage8");
        let mut studio = Studio::new();
        studio.world = data.join("rod-world.toml").to_string_lossy().into_owned();
        studio.task = data.join("acceptance/rod-task.toml").to_string_lossy().into_owned();
        for (slot, mode) in ["длинный", "короткий"].into_iter().enumerate() {
            studio.active = slot;
            studio.case = mode.into();
            let input = studio.input().unwrap();
            studio.spawn(input).unwrap();
            let worker = studio.worker.take().unwrap();
            studio.apply_worker(worker).unwrap();
            assert!(studio.slots[slot].report.is_some());
        }
        studio.sensor = "датчик хода".into();
        let at = Time(37_000_003); // Между границами тактов и за концом второго опыта.
        studio.set_time(at);
        let replay = studio.slots[0].replay.as_ref().unwrap();
        assert!(!replay.sensor_has_image(&studio.sensor));
        let series = replay.sensor_series(&studio.sensor).find(|s| s.unit == "m").unwrap();
        let a = studio.slots[0].sample(replay, series, at).unwrap().1.unwrap();
        let b = studio.slots[1].sample(replay, series, at).unwrap().1;
        assert_eq!(a.at, Time(37_000_000));
        // Независимая сумма скоростей базовой сетки.
        let expected = 10.0 * 0.001_f64.powi(2) * 37.0 * 38.0 / 2.0;
        assert!((a.value - expected).abs() < 1e-13);
        assert!(b.is_none());
        let common = Time(30_000_000);
        let value = |slot: &Slot| slot.sample(replay, series, common).unwrap().1.unwrap().value;
        assert_eq!(value(&studio.slots[0]).to_bits(), value(&studio.slots[1]).to_bits());
        let ctx = egui::Context::default();
        let graphs = ctx.run_ui(egui::RawInput::default(), |ui| {
            show_samples(ui, &studio.slots[0], &studio.sensor, "", at);
        });
        assert!(graphs.shapes.iter().any(|shape| {
            matches!(&shape.shape, egui::Shape::Path(path) if path.points.len() >= 3)
        }), "окно не подготовило кривую сигнала");
        graphs.drop_without_applying_deltas();
        println!("Две задачи · общие 30 мс совпали, А = {:.3} мм на {} нс, у Б отсчёта нет", a.value * 1000.0, at.0);
    }

    #[test]
    fn худшие_строки_кампании_открываются_в_трёх_текстурах_без_окна() {
        let suite = Suite::load(&PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../devices/stage8/suite.json")).unwrap();
        let mut studio = Studio::new();
        studio.campaign_report = Some(super::super::run(&Input::Campaign {
            manifest: suite.campaign.clone(), repeat: None, measurements: None,
        }).unwrap().report);
        let report = studio.campaign_report.as_ref().unwrap();
        assert!(report.repeat_key().is_none());
        assert_eq!(report.assemblies().count(), suite.entries.len());
        let temp = Temp::new().unwrap();
        let ctx = egui::Context::default();
        for entry in &suite.entries {
            let expected = studio.campaign_report.as_ref().unwrap();
            let key = expected.worst(Some(&entry.id)).unwrap().to_owned();
            let expected = expected.playback(&key).unwrap().clone();
            let event = expected.playback.event.as_ref();
            let event_time = event.map_or(Time::ZERO, |event| Time(event.at_ns));
            let archive = temp.0.join(format!("{}.simview", entry.id));
            let journal = temp.0.join(format!("{}.simlog", entry.id));
            let cli = |flag: &str, path: &Path| {
                let args = [
                    "--json".into(), "--campaign".into(), suite.campaign.as_os_str().to_owned(),
                    "--repeat".into(), (&key).into(), flag.into(), path.as_os_str().to_owned(),
                ];
                let mut json = Vec::new();
                let mut err = Vec::new();
                let code = sim_cli::run(args, std::io::Cursor::new([]), &mut json, &mut err);
                let raw = String::from_utf8(json).unwrap();
                assert_eq!(code, 0, "{key} · {raw} · {}", String::from_utf8_lossy(&err));
                raw
            };
            let raw = cli("--view", &archive);
            let repeat = Report::parse(raw.clone()).unwrap();
            let binding = repeat.playback(&key).unwrap();
            assert_eq!((&binding.input, &binding.playback), (&expected.input, &expected.playback));
            let view = sim_cli::view::RunView::read(&std::fs::read(&archive).unwrap()).unwrap();
            assert_eq!(view.metadata.campaign.as_ref(), Some(binding));
            assert_eq!(raw, cli("--log", &journal));
            assert!(view.binary == std::fs::read(&journal).unwrap(), "{key} · журналы различаются");
            let log = view.journal;
            drop(view.binary);
            std::fs::remove_file(&journal).unwrap();

            // Тот же загрузчик, который вызывается параметром окна --journal.
            studio.journal = archive.to_string_lossy().into_owned();
            open_journal(&mut studio).unwrap();
            assert_eq!(studio.time, event_time);
            if event.is_none() {
                assert!(studio.message.contains("нет привязанного события"));
                let at = Time(expected.playback.end_ns / 2);
                studio.set_time(at);
                assert!(studio.jump_to_worst().unwrap_err().contains("нет привязанного события"));
                assert_eq!(studio.time, at);
            }
            assert_eq!(studio.slots[0].replay.as_ref().unwrap().campaign.as_ref(), Some(binding));
            std::fs::remove_file(&archive).unwrap();
            let (mut scalar, mut image) = (0, 0);
            let mut units = BTreeSet::new();
            let mut pixel_formats = BTreeSet::new();
            let replay = studio.slots[0].replay.as_ref().unwrap();
            for sensor in replay.sensor_names() {
                if let Some(frame) = replay.frames.get(sensor)
                    .and_then(|frames| frames.iter().find(|frame| frame.valid))
                {
                    image += 1;
                    pixel_formats.insert(match frame.format {
                        PixelFormat::Rgb8 => "RGB8",
                        PixelFormat::DepthMm16 => "глубина",
                    });
                }
                let mut valid = false;
                for series in replay.sensor_series(sensor).filter(|s| s.points.iter().any(|p| p.valid)) {
                    units.insert(series.unit.clone());
                    valid = true;
                }
                scalar += usize::from(valid);
            }
            assert!(scalar + image > 0, "{key} · нет действительных данных датчиков или процессов");
            let mut visible = Vec::new();
            for at in [Time::ZERO, Time(expected.playback.end_ns / 2), event.map_or(Time(expected.playback.end_ns), |event| Time(event.at_ns))] {
                studio.set_time(at);
                studio.redraw(&ctx).unwrap();
                let slot = &mut studio.slots[0];
                assert_eq!(slot.texture.as_ref().unwrap().size(), SIZE.map(|n| n as usize));
                let Some(SlotFrame::Rendered(frame)) = slot.frame(at, View::OVERVIEW, SIZE, "").unwrap() else {
                    panic!("{key} · нет кадра на {} нс", at.0);
                };
                let scene = slot.scene.as_ref().unwrap();
                let camera = scene.camera(View::OVERVIEW, SIZE).unwrap();
                if scene.uses_kinematics() {
                    let center = super::super::Point::from(scene.sim.body_pose(1).0);
                    let d = center - camera.eye;
                    let dot = |axis| (0..3).map(|i| camera.axes[(i, axis)] * d[i]).sum::<f64>();
                    let x = camera.lens.fx * dot(0) / dot(2) + camera.lens.cx;
                    let y = camera.lens.fy * dot(1) / dot(2) + camera.lens.cy;
                    assert!(frame.projected_kinematics.iter().flatten().any(|p| {
                        // Независимая проекция анкера, запас на округление f64.
                        (p[0] - x).abs() < 1e-10 && (p[1] - y).abs() < 1e-10
                    }), "{key} · схема расходится с позой на {} нс", at.0);
                    let count = frame.rgb.iter().filter(|&&pixel| pixel == super::super::render::KINEMATICS_COLOR).count();
                    assert!(count > 0, "{key} · пустая схема на {} нс", at.0);
                    visible.push(count);
                } else {
                    let count = frame.ids.iter().filter(|id| id.is_some()).count();
                    assert!(count > 0, "{key} · пустой кадр на {} нс", at.0);
                    visible.push(count);
                    let (index, geom, radius) = frame.ids.iter().enumerate()
                        .filter_map(|(index, geom)| {
                            let geom = (*geom).filter(|&id| id < scene.sim.ngeom())?;
                            // Идентификатор проверен по размеру массива модели MuJoCo.
                            let radius = unsafe { *scene.sim.model().geom_rbound.add(geom) };
                            (radius.is_finite() && radius > 0.0).then_some((index, geom, radius))
                        }).next().expect("в кадре нет тела с конечным охватом");
                    let width = SIZE[0] as usize;
                    let (x, y) = ((index % width) as f64 + 0.5, (index / width) as f64 + 0.5);
                    let z = frame.depth[index];
                    let pixel = camera.eye + camera.axes * super::super::Vec3::new(
                        (x - camera.lens.cx) * z / camera.lens.fx,
                        (y - camera.lens.cy) * z / camera.lens.fy,
                        z,
                    );
                    let (center, _) = scene.geometry_pose(geom).unwrap();
                    let distance = (pixel - super::super::Point::from(center)).norm();
                    // Как в сверке штатных сцен, запас на арифметику обратной проекции.
                    assert!(z.is_finite() && z > 0.0 && distance <= radius + 1e-6, "{key} · пиксель вне геометрии на {} нс", at.0);
                }
                let entries = &log.entries()[..log.entries().partition_point(|e| e.at <= at)];
                for (i, actual) in scene.sim.qpos().iter().enumerate() {
                    let name = format!("qpos[{i}]");
                    let id = log.header().truth.iter().position(|s| s.name == name).unwrap();
                    let expected = entries.iter().rev().find(|e| {
                        e.kind == sim_runtime::LogKind::Truth && e.id as usize == id
                    }).unwrap();
                    assert_eq!(actual.to_bits(), expected.value.to_bits());
                }
            }
            let cursor = event.map_or_else(|| "нет события оценщика".into(), |event| {
                assert!(event.violation);
                format!("{} нс ({})", event.at_ns, event.label)
            });
            println!("{} · {key} · {} / {} · {} Гц · {cursor} · датчики {scalar}+{image}, {units:?} {pixel_formats:?} · кадры {visible:?} · журнал {:016x} побитово",
                entry.id, expected.playback.child.mode, expected.playback.child.path,
                expected.playback.physics_hz, expected.playback.journal);
        }
    }

    #[test]
    fn moving_sensor_comparison_and_exact_cursor_render_without_window() {
        let world =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage8/rod-world.toml");
        let mut studio =
            settings(vec!["--platform".into(), world.clone().into_os_string()]).unwrap();
        let input = |ns| Input::World {
            world: world.clone(),
            duration: Time(ns),
            catalog: None,
        };
        // Кадр приходит на 24 мс. Запись Б кончается на 30 мс, запись А на 80 мс.
        studio.spawn(input(80_000_000)).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        studio.active = 1;
        studio.spawn(input(30_000_000)).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        assert!(studio.slots.iter().all(|slot| slot.fingerprint_matches));
        let ctx = egui::Context::default();
        studio.sensor = "направляющая/кадр".into();
        studio.set_time(Time(23_999_999));
        studio.redraw(&ctx).unwrap();
        assert!(
            studio
                .slots
                .iter()
                .all(|slot| slot.texture.as_ref().unwrap().size() == SIZE.map(|u| u as usize))
        );
        studio.set_time(Time(24_000_000));
        studio.redraw(&ctx).unwrap();
        assert!(
            studio
                .slots
                .iter()
                .all(|slot| slot.texture.as_ref().unwrap().size() == [96, 72])
        );

        // Нецелая отметка относительно сетки 1 мс использует предыдущую позу и доставленный кадр.
        let cursor = Time(37_000_003);
        studio.set_time(cursor);
        studio.redraw(&ctx).unwrap();
        assert!(studio.slots[1].texture.is_none());
        let first = &studio.slots[0];
        let replay = first.replay.as_ref().unwrap();
        let x = &replay.series[replay.qpos[0]];
        let x_at = first.sample(replay, x, cursor).unwrap().1.unwrap();
        let second = &studio.slots[1];
        let common = Time(30_000_000);
        let sample = |slot: &Slot, at| slot.sample(replay, x, at).unwrap().1;
        assert_eq!(
            sample(first, common).unwrap().value.to_bits(),
            sample(second, common).unwrap().value.to_bits()
        );
        assert!(sample(second, cursor).is_none());
        assert_eq!(x_at.at, Time(37_000_000));
        let displacement = 10.0 * 0.001_f64.powi(2) * 37.0 * 38.0 / 2.0;
        // Допуск округления решения Euler за 37 шагов.
        assert!((x_at.value - displacement).abs() < 1e-13);
        let scene = first.scene.as_ref().unwrap();
        let camera = scene.camera(View::Sensor(&studio.sensor), SIZE).unwrap();
        assert!((camera.eye.x - (0.125 + displacement)).abs() < 1e-13);
        assert_eq!([camera.eye.y, camera.eye.z], [0.0, 0.5]);
        let frame = replay.image(&studio.sensor, cursor).unwrap();
        assert_eq!((frame.captured, frame.at), (Time::ZERO, Time(24_000_000)));
        // 1/4 полной шкалы даёт 24 пикселя из 96 в каждой строке.
        let force = Force {
            name: "сила".into(),
            value_n: -2.0,
            scale_n: 8.0,
        };
        let displayed = display_image(frame, Some(&force)).unwrap();
        assert_eq!(displayed.size, [96, 75]);
        for (pixel, rgb) in displayed.pixels[..96 * 72]
            .iter()
            .zip(frame.data.chunks_exact(3))
        {
            assert_eq!(pixel.to_array(), [rgb[0], rgb[1], rgb[2], 255]);
        }
        let [r, g, b] = FORCE_COLOR;
        assert_eq!(
            displayed.pixels[96 * 72..]
                .iter()
                .filter(|&&p| p == egui::Color32::from_rgb(r, g, b))
                .count(),
            3 * 24
        );
        println!(
            "Стержень · x={displacement:.6} м, камера x={:.6} м, курсор {} нс, поза {} нс, кадр {} нс",
            camera.eye.x, cursor.0, x_at.at.0, frame.at.0
        );

        // Тот же индекс в мире с другим порядком координат не годится для сравнения.
        let old_hash = studio.slots[1].replay.as_ref().unwrap().world_hash;
        studio.slots[1].replay.as_mut().unwrap().world_hash ^= 1;
        let replay = studio.slots[0].replay.as_ref().unwrap();
        assert!(
            studio.slots[1]
                .sample(replay, &replay.series[replay.qpos[0]], common)
                .is_none()
        );
        studio.slots[1].replay.as_mut().unwrap().world_hash = old_hash;

        let replay = studio.slots[0].replay.as_mut().unwrap();
        let series = replay
            .series
            .iter_mut()
            .find(|s| s.name == "направляющая/кадр/image")
            .unwrap();
        series.points[0].valid = false;
        assert!(replay.image(&studio.sensor, cursor).is_none());
    }

    #[test]
    fn глубина_стержня_видна_в_текстуре_после_доставки() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let data = root.join("devices/stage8");
        let temp = Temp::new().unwrap();
        let catalog = temp.0.join("catalog");
        for dir in ["drives", "sensors", "boards"] {
            std::fs::create_dir_all(catalog.join(dir)).unwrap();
        }
        let rgb = std::fs::read_to_string(root.join("catalog/sensors/reference-rgb.toml")).unwrap();
        let mut depth = rgb.replace("reference-rgb", "reference-depth")
            .replace("color_camera", "depth_camera")
            .replace("Синтетическая RGB-камера для сверки проекции", "Дальномерная камера для сверки плоской грани")
            .replace("кодирования RGB8", "округления глубины до миллиметра");
        depth = depth.lines().filter(|line| {
            !line.starts_with("ambient =") && !line.starts_with("spatial_samples =")
        }).collect::<Vec<_>>().join("\n");
        depth.push_str("\nhole_probability = { value = 0.0, provenance = 'estimate', source = 'Дыры видны только вне поверхности плоской грани' }\n");
        std::fs::write(catalog.join("sensors/reference-depth.toml"), depth).unwrap();
        let rod = std::fs::read_to_string(data.join("rod.toml")).unwrap()
            .replace("reference-rgb", "reference-depth");
        std::fs::write(temp.0.join("rod.toml"), rod).unwrap();
        let world = temp.0.join("rod-world.toml");
        std::fs::copy(data.join("rod-world.toml"), &world).unwrap();

        // Из параметров камеры: 16 мс экспозиции и 8 мс доставки.
        let delivered = Time(24_000_000);
        let mut studio = Studio::new();
        studio.world = world.to_string_lossy().into_owned();
        studio.catalog = catalog.to_string_lossy().into_owned();
        studio.duration = super::super::format_seconds(delivered);
        let input = studio.input().unwrap();
        studio.spawn(input).unwrap();
        let worker = studio.worker.take().unwrap();
        studio.apply_worker(worker).unwrap();
        studio.sensor = "направляющая/кадр".into();
        let replay = studio.slots[0].replay.as_ref().unwrap();
        assert!(replay.image(&studio.sensor, Time(delivered.0 - 1)).is_none());
        let frame = replay.image(&studio.sensor, delivered).unwrap();
        assert_eq!(frame.format, PixelFormat::DepthMm16);
        assert_eq!(frame.captured, Time::ZERO);
        let size = [frame.width as usize, frame.height as usize];
        let depths: Vec<_> = frame.data.chunks_exact(2)
            .map(|pair| u16::from_le_bytes([pair[0], pair[1]])).collect();
        // Камера в 0,5 м от центра, полутолщина грани 0,0625/2 м.
        // Проекция вдоль оптической оси и округление до целого миллиметра.
        let face_mm = ((0.5_f64 - 0.0625 / 2.0) * 1000.0).round() as u16;
        assert_eq!(depths[size[0] * (size[1] / 2) + size[0] / 2], face_mm);
        assert!(depths.contains(&0));
        assert!(depths.iter().all(|&z| z == 0 || z == face_mm));
        let image = display_image(frame, None).unwrap();
        assert!(depths.iter().zip(&image.pixels).all(|(&z, &pixel)| {
            pixel == if z == 0 { egui::Color32::TRANSPARENT } else { egui::Color32::WHITE }
        }));
        let mut truncated = frame.clone();
        truncated.data = frame.data[..frame.data.len() - 1].into();
        assert!(display_image(&truncated, None).is_err());
        let ctx = egui::Context::default();
        studio.set_time(delivered);
        studio.redraw(&ctx).unwrap();
        assert_eq!(studio.slots[0].texture.as_ref().unwrap().size(), size);
        println!("Текстура глубины · {}×{}, {face_mm} мм, доставка {} нс", size[0], size[1], delivered.0);
    }

    #[test]
    fn worst_seek_requires_bound_variant_and_exact_time() {
        let path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage8/block-world.toml");
        let out = super::super::run(&Input::World {
            world: path.clone(),
            duration: Time(50_000_000),
            catalog: None,
        })
        .unwrap();
        let temp = Temp::new().unwrap();
        let journal_path = temp.0.join("запись.bin");
        std::fs::write(&journal_path, out.binary.as_ref().unwrap()).unwrap();
        // Здесь проверяется ручная связка старого SIMLOG с отчётом варианта.
        let replay = Replay::read(out.binary.as_deref().unwrap()).unwrap();
        let mut studio =
            settings(vec!["--platform".into(), path.clone().into_os_string()]).unwrap();
        studio.put_replay(0, replay, true).unwrap();
        let slot = &mut studio.slots[0];
        slot.source = Some(Input::Task {
            world: path.clone(),
            task: path,
            case: Some("сдвиг".into()),
            catalog: None,
            matrix: false,
        });
        let report = |variant: &str, time: &str| {
            Report::parse(format!(
                r#"{{"status":"failed","outcome":"fail","worst_variant":"{variant}"{time}}}"#
            ))
            .unwrap()
        };
        slot.report = Some(report("сдвиг", ""));
        assert!(
            studio
                .jump_to_worst()
                .unwrap_err()
                .contains("не записал момент")
        );
        assert_eq!(studio.time, Time::ZERO);
        studio.slots[0].report = Some(report("сдвиг", ",\"worst_time_ns\":50000001"));
        assert!(studio.jump_to_worst().unwrap_err().contains("за границей"));
        studio.slots[0].report = Some(report("сдвиг", ",\"worst_time_ns\":37000003"));
        studio.playing = true;
        studio.jump_to_worst().unwrap();
        assert_eq!(studio.time, Time(37_000_003));
        assert!(!studio.playing);
        assert_eq!(studio.time_input, "37000003");
        studio.slots[0].report = Some(report("другой", ",\"worst_time_ns\":37000003"));
        assert!(
            studio
                .seek_worst_time(0)
                .unwrap_err()
                .contains("отличается")
        );

        let report_path = temp.0.join("отчёт.json");
        std::fs::write(&report_path, &studio.slots[0].report.as_ref().unwrap().raw).unwrap();
        studio.journal = journal_path.to_string_lossy().into_owned();
        open_journal(&mut studio).unwrap();
        assert!(studio.slots[0].report.is_none() && studio.slots[0].source.is_none());
        studio.report_path = report_path.to_string_lossy().into_owned();
        open_report(&mut studio).unwrap();
        assert!(studio.imported_report.is_some() && studio.slots[0].report.is_none());
        assert!(studio.jump_to_worst().is_err());
    }
}
