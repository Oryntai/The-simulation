use std::borrow::Cow;
use std::collections::VecDeque;
use std::ops::Range;
use std::sync::Arc;

use nalgebra::Translation3;
use sim_runtime::{ObservationOut, Sensor, Truth};
use sim_signal::{ImageFrame, PixelFormat, Rate, Rng, SignalId, Time};

use crate::optics::{Frame, checked, pose};
use crate::pipeline::{self, Drift, SampleMode};
use crate::raster::{Grid, Pixel, Scene};
use crate::scalar::Mount;
use crate::settings::{CameraMode, CameraSettings, Settings, add_time};
use crate::vision::FrameTruth;

#[derive(Clone)]
pub(crate) struct CameraPlan {
    pub mount: Mount,
    pub camera: CameraSettings,
    pub settings: Settings,
    pub physics: Rate,
    pub scene: Arc<Scene>,
    pub grid: Arc<Grid>,
}

#[derive(Clone)]
struct WorldAt {
    time: Time,
    mount_base: Frame,
    bodies: Vec<Frame>,
}

#[derive(Clone)]
struct Timeline {
    plan: CameraPlan,
    next_frame: u64,
    next_step: u64,
    history: VecDeque<Arc<WorldAt>>,
}

impl Timeline {
    fn new(plan: CameraPlan) -> Self {
        Self {
            plan,
            next_frame: 0,
            next_step: 0,
            history: VecDeque::new(),
        }
    }

    fn push(&mut self, truth: &Truth<'_>) -> Result<(), String> {
        let now = truth.time();
        if now != self.plan.physics.tick(self.next_step) {
            return Err("камере нужны последовательные такты физики, начиная с нулевого".into());
        }
        let state = WorldAt {
            time: now,
            mount_base: self.plan.mount.base_frame(truth.engine())?,
            bodies: self.plan.scene.frames(truth.engine())?,
        };
        self.next_step = self
            .next_step
            .checked_add(1)
            .ok_or("исчерпаны такты физики")?;
        self.history.push_back(Arc::new(state));
        Ok(())
    }

    fn next_start(&self) -> Result<Time, String> {
        // Частота в каталоге целочисленная. Шкала Time и Rate выражена в наносекундах.
        const NS_PER_S: u128 = 1_000_000_000;
        let ns = u128::from(self.next_frame) * NS_PER_S / self.plan.settings.rate.as_hz() as u128;
        u64::try_from(ns)
            .map(Time)
            .map_err(|_| "исчерпана шкала кадров".into())
    }

    fn available(&self, start: Time, now: Time) -> Result<Time, String> {
        let end = self.end(start)?;
        Ok(add_time(end, self.plan.settings.latency)?.max(now))
    }

    fn end(&self, start: Time) -> Result<Time, String> {
        add_time(
            add_time(start, self.plan.camera.readout)?,
            self.plan.camera.exposure,
        )
    }

    fn due(&self, now: Time) -> Result<Option<Time>, String> {
        let start = self.next_start()?;
        Ok((self.end(start)? <= now).then_some(start))
    }

    fn finish(&mut self) -> Result<(), String> {
        self.next_frame = self
            .next_frame
            .checked_add(1)
            .ok_or("исчерпан счётчик кадров")?;
        Ok(())
    }

    fn prune(&mut self) -> Result<(), String> {
        // Для следующей экспозиции нужен последний снимок перед её началом.
        let next = self.next_start()?;
        while self.history.get(1).is_some_and(|step| step.time <= next) {
            self.history.pop_front();
        }
        Ok(())
    }

    fn at(&self, time: Time) -> Result<Cow<'_, WorldAt>, String> {
        let right = self.history.partition_point(|step| step.time < time);
        if let Some(frame) = self.history.get(right) {
            if frame.time == time {
                return Ok(Cow::Borrowed(frame));
            }
            if right > 0 {
                let left = &self.history[right - 1];
                let fraction = (time.0 - left.time.0) as f64 / (frame.time.0 - left.time.0) as f64;
                let interpolate = |a: &Frame, b: &Frame| -> Result<Frame, String> {
                    let translation = a.translation.vector.lerp(&b.translation.vector, fraction);
                    checked(translation.as_slice(), "интерполяция кадра")?;
                    Ok(Frame::from_parts(
                        Translation3::from(translation),
                        a.rotation.slerp(&b.rotation, fraction),
                    ))
                };
                return Ok(Cow::Owned(WorldAt {
                    time,
                    mount_base: interpolate(&left.mount_base, &frame.mount_base)?,
                    bodies: left
                        .bodies
                        .iter()
                        .zip(&frame.bodies)
                        .map(|(a, b)| interpolate(a, b))
                        .collect::<Result<_, _>>()?,
                }));
            }
        }
        Err("для экспозиции не хватает состояний физики".into())
    }

    fn bands(&self) -> impl Iterator<Item = (Range<usize>, Time)> {
        let height = self.plan.camera.lens.height as usize;
        let readout = self.plan.camera.readout.0;
        let count = if readout == 0 || height == 1 {
            1
        } else {
            height
        };
        (0..count).map(move |row| {
            if count == 1 {
                (0..height, Time::ZERO)
            } else {
                // readout — разница моментов старта первой и последней строки.
                let offset = u128::from(readout) * row as u128 / (height - 1) as u128;
                (row..row + 1, Time(offset as u64))
            }
        })
    }

    fn draw(
        &self,
        time: Time,
        rows: Range<usize>,
        ambient: Option<f64>,
        spatial: u32,
    ) -> Result<Vec<Pixel>, String> {
        let frame = self.at(time)?;
        let band = (self.plan.camera.near, self.plan.camera.far);
        let camera = self.plan.mount.with_base(&frame.mount_base)?;
        let geometry = self
            .plan
            .scene
            .prepare(&camera, &frame.bodies, band.0, band.1)?;
        let grid = &self.plan.grid;
        if spatial == 1 {
            return grid.draw(rows, &geometry, band, ambient, [0.5; 2]);
        }
        // Цвет интегрируется по площади пикселя равномерной сеткой середины ячеек.
        let mut pixels = grid.draw(rows.clone(), &geometry, band, None, [0.5; 2])?;
        let n = f64::from(spatial);
        for sy in 0..spatial {
            for sx in 0..spatial {
                let offset = [(f64::from(sx) + 0.5) / n, (f64::from(sy) + 0.5) / n];
                let subpixels = grid.draw(rows.clone(), &geometry, band, ambient, offset)?;
                for (out, sample) in pixels.iter_mut().zip(subpixels) {
                    for (total, value) in out.color.iter_mut().zip(sample.color) {
                        *total += value / (n * n);
                    }
                }
            }
        }
        Ok(pixels)
    }

    fn render_truth(
        &self,
        signal: SignalId,
        captured: Time,
        now: Time,
    ) -> Result<FrameTruth, String> {
        let lens = self.plan.camera.lens;
        let mut depth = Vec::new();
        let mut ids = Vec::new();
        let mut row_times = Vec::new();
        for (rows, offset) in self.bands() {
            let center = add_time(
                add_time(captured, offset)?,
                Time(self.plan.camera.exposure.0 / 2),
            )?;
            row_times.extend(std::iter::repeat_n(center, rows.len()));
            for pixel in self.draw(center, rows, None, 1)? {
                depth.push(pixel.id.map(|_| pixel.depth));
                ids.push(pixel.id);
            }
        }
        let end = self.end(captured)?;
        let reference_at = Time(captured.0 + (end.0 - captured.0) / 2);
        let frame = self.at(reference_at)?;
        Ok(FrameTruth {
            signal,
            captured,
            available: self.available(captured, now)?,
            width: lens.width,
            height: lens.height,
            depth_m: depth.into(),
            object_ids: ids.into(),
            row_times: row_times.into(),
            reference_at,
            camera_pose: pose(&self.plan.mount.with_base(&frame.mount_base)?),
            objects: self.plan.scene.objects(&frame.bodies)?.into(),
        })
    }

    fn exposed(&self, start: Time, rows: Range<usize>) -> Result<Vec<Pixel>, String> {
        let c = &self.plan.camera;
        let (ambient, spatial) = match c.mode {
            CameraMode::Color {
                ambient,
                spatial_samples,
            } => (Some(ambient), spatial_samples),
            CameraMode::Depth { .. } => (None, 1),
        };
        if c.exposure == Time::ZERO || c.samples == 1 {
            return self.draw(
                add_time(start, Time(c.exposure.0 / 2))?,
                rows,
                ambient,
                spatial,
            );
        }

        // Равномерная квадратура средних точек внутри экспозиции. Если за один
        // отсчёт глубины пиксель увидел разные поверхности, выдаём дыру.
        let width = c.lens.width as usize;
        let mut result = vec![Pixel::default(); rows.len() * width];
        let count = f64::from(c.samples);
        for k in 0..c.samples {
            let mid_num = 2 * u128::from(k) + 1;
            let mid_den = 2 * u128::from(c.samples);
            let offset = (u128::from(c.exposure.0) * mid_num / mid_den) as u64;
            let pixels = self.draw(
                add_time(start, Time(offset))?,
                rows.clone(),
                ambient,
                spatial,
            )?;
            for (out, sample) in result.iter_mut().zip(pixels) {
                if k == 0 {
                    out.id = sample.id;
                } else if out.id != sample.id {
                    out.id = None;
                }
                if out.id.is_some() {
                    out.depth += sample.depth / count;
                }
                for (total, value) in out.color.iter_mut().zip(sample.color) {
                    *total += value / count;
                }
            }
        }
        Ok(result)
    }
}

#[derive(Clone)]
pub(crate) struct CameraSensor {
    timeline: Timeline,
    signal: SignalId,
    drift: Drift,
    previous: Time,
    last: Option<(Time, Arc<[u8]>)>,
}

impl CameraSensor {
    pub fn new(plan: CameraPlan, signal: SignalId) -> Self {
        Self {
            timeline: Timeline::new(plan),
            signal,
            drift: Drift::default(),
            previous: Time::ZERO,
            last: None,
        }
    }

    fn frame(&mut self, captured: Time, now: Time, rng: &mut Rng) -> Result<ImageFrame, String> {
        let plan = &self.timeline.plan;
        let p = &plan.settings.pipe;
        let mode = pipeline::mode(
            plan.settings.fault,
            captured..=self.timeline.end(captured)?,
            plan.settings.drop_probability,
            rng,
        );
        let dt = Time(captured.0 - self.previous.0).as_secs();
        let drift = self.drift.advance(p, captured.as_secs(), dt, rng)?;
        self.previous = captured;
        let lens = plan.camera.lens;
        let format = match plan.camera.mode {
            CameraMode::Color { .. } => PixelFormat::Rgb8,
            CameraMode::Depth { .. } => PixelFormat::DepthMm16,
        };
        let available = self.timeline.available(captured, now)?;
        let payload = match mode {
            SampleMode::Invalid => None,
            SampleMode::Stuck => self.last.clone(),
            SampleMode::Fresh | SampleMode::Shifted => {
                let mut data = Vec::new();
                for (rows, offset) in self.timeline.bands() {
                    let start = add_time(captured, offset)?;
                    for pixel in self.timeline.exposed(start, rows)? {
                        let shifted = mode == SampleMode::Shifted;
                        match plan.camera.mode {
                            CameraMode::Color { .. } => {
                                for channel in pixel.color {
                                    let value =
                                        pipeline::value(channel, p, drift, shifted, None, rng)?;
                                    data.push(
                                        (value.clamp(0.0, 1.0) * f64::from(u8::MAX)).round() as u8
                                    );
                                }
                            }
                            CameraMode::Depth { hole_probability } => {
                                let mm = if pixel.id.is_some()
                                    && !pipeline::event(hole_probability, rng)
                                {
                                    let depth =
                                        pipeline::value(pixel.depth, p, drift, shifted, None, rng)?;
                                    let mm = (depth * 1000.0).round();
                                    // Формат DepthMm16 резервирует ноль; за пределом формата нет отсчёта.
                                    if (1.0..=f64::from(u16::MAX)).contains(&mm) {
                                        mm as u16
                                    } else {
                                        0
                                    }
                                } else {
                                    0
                                };
                                data.extend(mm.to_le_bytes());
                            }
                        }
                    }
                }
                Some((captured, Arc::from(data)))
            }
        };
        let valid = payload.is_some() && mode != SampleMode::Stuck;
        if valid {
            self.last = payload.clone();
        }
        let (captured, data) = payload.unwrap_or_else(|| {
            // ImageFrame требует полный размер даже для явно недействительного кадра.
            let data =
                vec![0; lens.width as usize * lens.height as usize * format.bytes_per_pixel()];
            (captured, data.into())
        });
        Ok(ImageFrame {
            signal: self.signal,
            captured,
            at: available,
            width: lens.width,
            height: lens.height,
            format,
            data,
            valid,
        })
    }
}

impl Sensor for CameraSensor {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        rng: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        self.timeline.push(truth)?;
        let now = truth.time();
        while let Some(captured) = self.timeline.due(now)? {
            out.write_frame(self.frame(captured, now, rng)?);
            self.timeline.finish()?;
        }
        self.timeline.prune()
    }
}

/// Считыватель истины для оценщика со своей историей, совместимой со снимками.
/// Вызовите advance на каждом такте физики. Сопоставлять с ImageFrame по
/// (signal, available) для попытки съёмки и (signal, captured) для годного кадра.
///
/// ```compile_fail,E0308
/// use sim_sense::FrameTruthReader;
/// use sim_signal::Observation;
/// fn получить_истину(reader: &mut FrameTruthReader, observed: &Observation) {
///     reader.advance(observed);
/// }
/// ```
#[derive(Clone)]
pub struct FrameTruthReader {
    timeline: Timeline,
    signal: SignalId,
}

impl FrameTruthReader {
    pub(crate) fn new(plan: CameraPlan, signal: SignalId) -> Self {
        Self {
            timeline: Timeline::new(plan),
            signal,
        }
    }

    /// Истина готова по окончании экспозиции. available задаёт срок доставки самого кадра.
    pub fn advance(&mut self, truth: &Truth<'_>) -> Result<Vec<FrameTruth>, String> {
        self.timeline.push(truth)?;
        let mut frames = Vec::new();
        let now = truth.time();
        while let Some(start) = self.timeline.due(now)? {
            frames.push(self.timeline.render_truth(self.signal, start, now)?);
            self.timeline.finish()?;
        }
        self.timeline.prune()?;
        Ok(frames)
    }
}
