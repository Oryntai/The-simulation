//! Адаптер openEMS 0.37 для спектров портов локальной постановки.
//! В электрической схеме применимость рядов, сходимость сетки и стенд проверяются отдельно.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::fs::{self, File};
use std::io::{self, BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use nalgebra::Complex;

use crate::SimError;

use super::loop_model::{positive, with_source};
use super::{Analysis, Sourced};

/// Декартова область с включёнными границами, метры.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Region {
    pub lower_m: [f64; 3],
    pub upper_m: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boundary {
    Pec,
    Pmc,
    Mur,
    Pml(u8),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Material {
    Pec,
    Dielectric {
        epsilon_r: f64,
        mu_r: f64,
        conductivity_s_per_m: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box(Region),
    Cylinder {
        start_m: [f64; 3],
        stop_m: [f64; 3],
        radius_m: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solid {
    pub shape: Shape,
    pub material: Material,
    pub priority: u32,
}

/// Резистивное окончание, источник продольного поля или оба на одном участке.
/// Открытый измерительный порт задаётся зондами без элемента в геометрии.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LumpedPort {
    pub region: Region,
    pub axis: Axis,
    pub priority: u32,
    pub resistance_ohm: Option<f64>,
    pub excitation_v_per_m: Option<f64>,
}

/// Интеграл E по линии или интеграл H по контуру плоскости с заданной нормалью.
/// Знак и масштаб показания задаёт вес.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeKind {
    Voltage,
    Current(Axis),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Probe<'a> {
    pub name: &'a str,
    pub kind: ProbeKind,
    pub region: Region,
    pub weight: f64,
    /// Минимальный различимый модуль спектра после учёта веса, в В·с или А·с.
    pub floor: Option<Sourced<'a, f64>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GaussianPulse {
    pub center_hz: f64,
    /// Полуширина по уровню −20 дБ согласно openEMS SetGaussExcite.
    pub cutoff_hz: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Band {
    pub range_hz: [f64; 2],
    /// Частоты должны включать обе границы и идти по возрастанию.
    pub frequencies_hz: Vec<f64>,
    /// Допустимый разрыв между соседними отсчётами из постановки опыта.
    pub max_spacing_hz: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Model<'a> {
    pub grid_m: [Vec<f64>; 3],
    /// Грани в порядке −X, +X, −Y, +Y, −Z, +Z.
    pub boundaries: [Boundary; 6],
    /// Фон openEMS — вакуум. Прочие материалы входят в геометрию явно.
    pub solids: Vec<Solid>,
    pub ports: Vec<LumpedPort>,
    pub probes: Vec<Probe<'a>>,
    pub pulse: GaussianPulse,
    pub band: Band,
    pub max_steps: u32,
    pub end_energy_ratio: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProbeSpectrum {
    pub name: String,
    /// Фактические индексы узлов, напечатанные openEMS. Токовый контур
    /// проецируется на двойственную сетку Yee.
    pub snapped_start: [usize; 3],
    pub snapped_stop: [usize; 3],
    /// Спектр импульса openEMS: 2·∫ x(t)·exp(−j·2π·f·t) dt, в В·с или А·с.
    pub values: Vec<Complex<f64>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpenEmsResult {
    /// Точные байты XML, переданные процессу: материалы, сетка, границы, порты и зонды.
    pub input_xml: String,
    pub openems_version: String,
    pub csxcad_version: String,
    pub iterations: u32,
    pub timestep_s: f64,
    pub frequencies_hz: Vec<f64>,
    pub probes: Vec<ProbeSpectrum>,
}

/// Проверить постановку и построить XML openEMS для воспроизводимого запуска.
pub fn prepare_openems(model: &Model<'_>) -> Result<Analysis<String>, SimError> {
    model.validate()?;
    let band = &model.band;
    if band.frequencies_hz.first() != Some(&band.range_hz[0])
        || band.frequencies_hz.last() != Some(&band.range_hz[1])
        || band
            .frequencies_hz
            .windows(2)
            .any(|w| w[1] - w[0] > band.max_spacing_hz)
    {
        return Ok(Analysis::NotAnalyzed(
            "частотная сетка не покрывает заявленную полосу",
        ));
    }
    if band.range_hz[0] < model.pulse.center_hz - model.pulse.cutoff_hz
        || band.range_hz[1] > model.pulse.center_hz + model.pulse.cutoff_hz
        || !model.ports.iter().any(|p| p.excitation_v_per_m.is_some())
    {
        return Ok(Analysis::NotAnalyzed(
            "полоса не обеспечена возбуждением openEMS",
        ));
    }
    if model.probes.is_empty() || model.probes.iter().any(|p| with_source(p.floor).is_none()) {
        return Ok(Analysis::NotAnalyzed(
            "требуется зонд и источник его спектрального пола",
        ));
    }

    let mut xml = String::new();
    writeln!(xml, "<openEMS>").unwrap();
    // Формат InitFDTD / SetGaussExcite. Oversampling 4 — штатное значение InitFDTD v0.37.
    writeln!(xml, "<FDTD NumberOfTimesteps=\"{}\" endCriteria=\"{:.17e}\" OverSampling=\"4\" f_max=\"{:.17e}\">",
        model.max_steps, model.end_energy_ratio, model.pulse.center_hz + model.pulse.cutoff_hz).unwrap();
    writeln!(
        xml,
        "<Excitation Type=\"0\" f0=\"{:.17e}\" fc=\"{:.17e}\"/>",
        model.pulse.center_hz, model.pulse.cutoff_hz
    )
    .unwrap();
    write!(xml, "<BoundaryCond").unwrap();
    for (face, bc) in ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"]
        .into_iter()
        .zip(model.boundaries)
    {
        let bc = match bc {
            Boundary::Pec => "PEC".into(),
            Boundary::Pmc => "PMC".into(),
            Boundary::Mur => "MUR".into(),
            Boundary::Pml(cells) => format!("PML_{cells}"),
        };
        write!(xml, " {face}=\"{bc}\"").unwrap();
    }
    writeln!(xml, "/></FDTD><ContinuousStructure CoordSystem=\"0\"><RectilinearGrid DeltaUnit=\"1\" CoordSystem=\"0\">").unwrap();
    for (axis, grid) in ["XLines", "YLines", "ZLines"]
        .into_iter()
        .zip(&model.grid_m)
    {
        writeln!(xml, "<{axis}>{}</{axis}>", xml_numbers(grid)).unwrap();
    }
    writeln!(xml, "</RectilinearGrid><Properties>").unwrap();

    for (i, solid) in model.solids.iter().enumerate() {
        let tag = match solid.material {
            Material::Pec => {
                writeln!(xml, "<Metal Name=\"solid_{i}\">").unwrap();
                "Metal"
            }
            Material::Dielectric {
                epsilon_r,
                mu_r,
                conductivity_s_per_m,
            } => {
                writeln!(xml, "<Material Name=\"solid_{i}\"><Property Epsilon=\"{epsilon_r:.17e}\" Mue=\"{mu_r:.17e}\" Kappa=\"{conductivity_s_per_m:.17e}\"/>").unwrap();
                "Material"
            }
        };
        writeln!(xml, "<Primitives>").unwrap();
        match solid.shape {
            Shape::Box(region) => xml_box(&mut xml, region, solid.priority),
            Shape::Cylinder {
                start_m,
                stop_m,
                radius_m,
            } => {
                writeln!(
                    xml,
                    "<Cylinder Priority=\"{}\" Radius=\"{radius_m:.17e}\">",
                    solid.priority
                )
                .unwrap();
                xml_points(&mut xml, start_m, stop_m);
                writeln!(xml, "</Cylinder>").unwrap();
            }
        }
        writeln!(xml, "</Primitives></{tag}>").unwrap();
    }

    // Схема свойств совпадает с AddLumpedPort.m и AddProbe.m openEMS/CSXCAD.
    for (i, port) in model.ports.iter().enumerate() {
        if let Some(r) = port.resistance_ohm {
            writeln!(
                xml,
                "<LumpedElement Name=\"load_{i}\" Direction=\"{}\" Caps=\"1\" LEtype=\"0\"
                R=\"{r:.17e}\"><Primitives>",
                port.axis as usize
            )
            .unwrap();
            xml_box(&mut xml, port.region, port.priority);
            writeln!(xml, "</Primitives></LumpedElement>").unwrap();
        }
        if let Some(e) = port.excitation_v_per_m {
            let mut vector = [0.0; 3];
            vector[port.axis as usize] = e;
            writeln!(
                xml,
                "<Excitation Name=\"drive_{i}\" Type=\"0\" Excite=\"{}\"><Primitives>",
                xml_numbers(&vector)
            )
            .unwrap();
            xml_box(&mut xml, port.region, port.priority);
            writeln!(xml, "</Primitives></Excitation>").unwrap();
        }
    }
    let frequencies = xml_numbers(&band.frequencies_hz);
    for (i, probe) in model.probes.iter().enumerate() {
        let (kind, normal) = match probe.kind {
            ProbeKind::Voltage => (0, -1),
            ProbeKind::Current(axis) => (1, axis as i32),
        };
        writeln!(
            xml,
            "<ProbeBox Name=\"probe_{i}\" Type=\"{kind}\" NormDir=\"{normal}\" Weight=\"{:.17e}\" OverSampling=\"4\">
            <FD_Samples>{frequencies}</FD_Samples><Primitives>",
            probe.weight
        )
        .unwrap();
        // Для зонда приоритет не участвует в распределении материала.
        xml_box(&mut xml, probe.region, 0);
        writeln!(xml, "</Primitives></ProbeBox>").unwrap();
    }
    writeln!(xml, "</Properties></ContinuousStructure></openEMS>").unwrap();
    Ok(Analysis::Calculated(xml))
}

fn xml_numbers(values: &[f64]) -> String {
    values
        .iter()
        .map(|x| format!("{x:.17e}"))
        .collect::<Vec<_>>()
        .join(",")
}

fn xml_points(xml: &mut String, start: [f64; 3], stop: [f64; 3]) {
    for (i, [x, y, z]) in [start, stop].into_iter().enumerate() {
        writeln!(
            xml,
            "<P{} X=\"{x:.17e}\" Y=\"{y:.17e}\" Z=\"{z:.17e}\"/>",
            i + 1
        )
        .unwrap();
    }
}

fn xml_box(xml: &mut String, region: Region, priority: u32) {
    writeln!(xml, "<Box Priority=\"{priority}\">").unwrap();
    xml_points(xml, region.lower_m, region.upper_m);
    writeln!(xml, "</Box>").unwrap();
}

impl Model<'_> {
    fn validate(&self) -> Result<(), SimError> {
        let band = &self.band;
        let [low, high] = band.range_hz;
        let GaussianPulse {
            center_hz,
            cutoff_hz,
        } = self.pulse;
        if !positive(low)
            || !positive(high)
            || high <= low
            || !positive(band.max_spacing_hz)
            || !band.max_spacing_hz.recip().is_finite()
            || !center_hz.is_finite()
            || center_hz < 0.0
            || !positive(cutoff_hz)
            || !(center_hz + cutoff_hz).is_finite()
        {
            return Err(invalid("некорректная полоса или гауссов импульс"));
        }
        if band
            .frequencies_hz
            .iter()
            .any(|&f| !positive(f) || f < low || f > high || !positive(native_rounded(f)))
            || band
                .frequencies_hz
                .windows(2)
                .any(|w| w[0] >= w[1] || native_rounded(w[0]) >= native_rounded(w[1]))
        {
            return Err(invalid(
                "частотная сетка некорректна или неоднозначна в выводе openEMS",
            ));
        }
        // Parse_XML_FDTDSetup ограничивает число шагов типом int.
        if self.max_steps == 0
            || self.max_steps > i32::MAX as u32
            || !positive(self.end_energy_ratio)
            || self.end_energy_ratio >= 1.0
        {
            return Err(invalid("некорректный предел шагов или критерий затухания"));
        }

        let mut interior = Region {
            lower_m: [0.0; 3],
            upper_m: [0.0; 3],
        };
        for (axis, grid) in self.grid_m.iter().enumerate() {
            let start = pml_cells(self.boundaries[2 * axis])?;
            let end = pml_cells(self.boundaries[2 * axis + 1])?;
            if grid.len() < start + end + 2
                || grid.iter().any(|v| !v.is_finite())
                || grid.windows(2).any(|w| !positive(w[1] - w[0]))
                || !(grid[grid.len() - 1] - grid[0]).is_finite()
            {
                return Err(invalid(
                    "некорректная декартова сетка или нет ячеек между слоями PML",
                ));
            }
            interior.lower_m[axis] = grid[start];
            interior.upper_m[axis] = grid[grid.len() - end - 1];
        }
        for solid in &self.solids {
            if solid.priority > i32::MAX as u32 {
                return Err(invalid("приоритет примитива не помещается в int openEMS"));
            }
            if let Material::Dielectric {
                epsilon_r,
                mu_r,
                conductivity_s_per_m,
            } = solid.material
                && (!positive(epsilon_r)
                    || !positive(mu_r)
                    || !conductivity_s_per_m.is_finite()
                    || conductivity_s_per_m < 0.0)
            {
                return Err(invalid("некорректные параметры материала"));
            }
            let bounds = match solid.shape {
                Shape::Box(region) => region,
                Shape::Cylinder {
                    start_m,
                    stop_m,
                    radius_m,
                } => {
                    let varying = (0..3)
                        .filter(|&i| start_m[i] != stop_m[i])
                        .collect::<Vec<_>>();
                    if !positive(radius_m)
                        || varying.len() != 1
                        || start_m.iter().chain(&stop_m).any(|x| !x.is_finite())
                    {
                        return Err(invalid(
                            "цилиндр должен иметь положительный радиус и декартову ось",
                        ));
                    }
                    let extent = |axis| if axis == varying[0] { 0.0 } else { radius_m };
                    Region {
                        lower_m: std::array::from_fn(|i| start_m[i].min(stop_m[i]) - extent(i)),
                        upper_m: std::array::from_fn(|i| start_m[i].max(stop_m[i]) + extent(i)),
                    }
                }
            };
            bounds.validate(interior)?;
        }

        for port in &self.ports {
            port.region.validate(interior)?;
            if !port.region.extends(port.axis as usize)
                || port.priority > i32::MAX as u32
                || port.resistance_ohm.is_some_and(|r| !positive(r))
                || port
                    .excitation_v_per_m
                    .is_some_and(|e| !e.is_finite() || e == 0.0)
                || (port.resistance_ohm.is_none() && port.excitation_v_per_m.is_none())
            {
                return Err(invalid("некорректная геометрия или параметры порта"));
            }
        }

        let mut names = BTreeSet::new();
        for probe in &self.probes {
            probe.region.validate(interior)?;
            let dimensions = (0..3).filter(|&i| probe.region.extends(i)).count();
            let valid_geometry = match probe.kind {
                ProbeKind::Voltage => dimensions == 1,
                ProbeKind::Current(axis) => dimensions == 2 && !probe.region.extends(axis as usize),
            };
            if !valid_geometry
                || !probe.weight.is_finite()
                || probe.weight == 0.0
                || probe.floor.is_some_and(|f| !positive(f.value))
                || probe.name.trim().is_empty()
                || !names.insert(probe.name)
            {
                return Err(invalid("некорректная область, имя, вес или пол зонда"));
            }
        }
        Ok(())
    }
}

impl Region {
    fn extends(self, axis: usize) -> bool {
        self.upper_m[axis] > self.lower_m[axis]
    }

    fn validate(self, inside: Region) -> Result<(), SimError> {
        if (0..3).any(|i| {
            let span = self.upper_m[i] - self.lower_m[i];
            !span.is_finite()
                || span < 0.0
                || self.lower_m[i] < inside.lower_m[i]
                || self.upper_m[i] > inside.upper_m[i]
        }) {
            return Err(invalid(
                "примитив задан вне внутренней области сетки или имеет неверные координаты",
            ));
        }
        Ok(())
    }
}

fn pml_cells(boundary: Boundary) -> Result<usize, SimError> {
    match boundary {
        // Диапазон толщины слоя из SetBoundaryCond.m.
        Boundary::Pml(n @ 4..=50) => Ok(n.into()),
        Boundary::Pml(_) => Err(invalid("толщина PML вне диапазона 4–50 ячеек openEMS")),
        _ => Ok(0),
    }
}

fn native_rounded(value: f64) -> f64 {
    // ProcessIntegral::Dump_FD_Data в версии 0.37 пишет defaultfloat с шестью
    // значащими цифрами. Ключи должны оставаться различимыми после печати.
    format!("{value:.5e}").parse().unwrap()
}

/// Запустить установленный openEMS отдельным процессом. При открытом опыте
/// возвращается причина без частичных спектров. Лимит относится к процессу.
pub fn run_openems(
    executable: &Path,
    model: &Model<'_>,
    timeout: Duration,
) -> Result<Analysis<OpenEmsResult>, SimError> {
    if timeout.is_zero() {
        return Err(invalid(
            "нужен положительный лимит времени процесса openEMS",
        ));
    }
    let xml = match prepare_openems(model)? {
        Analysis::Calculated(xml) => xml,
        Analysis::NotAnalyzed(reason) => return Ok(Analysis::NotAnalyzed(reason)),
    };
    let binary = match executable.canonicalize() {
        Ok(path) if path.is_file() => path,
        _ => return Ok(Analysis::NotAnalyzed("исполняемый файл openEMS недоступен")),
    };
    let temp = Temp::new().map_err(io_error)?;
    let result = run_in(&binary, temp.path.as_deref().unwrap(), xml, model, timeout);
    temp.remove().map_err(io_error)?;
    result
}

fn run_in(
    binary: &Path,
    work: &Path,
    xml: String,
    model: &Model<'_>,
    timeout: Duration,
) -> Result<Analysis<OpenEmsResult>, SimError> {
    fs::write(work.join("model.xml"), &xml).map_err(io_error)?;
    let stdout = File::create(work.join("stdout")).map_err(io_error)?;
    let stderr = File::create(work.join("stderr")).map_err(io_error)?;
    let mut command = Command::new(binary);
    command
        .args([
            "model.xml",
            "--engine=basic",
            "--numThreads=1",
            "--exact-endcriteria",
            "--dump-statistics",
        ])
        .current_dir(work)
        .env_clear()
        .env("LC_ALL", "C")
        .stdin(Stdio::null())
        .stdout(stdout)
        .stderr(stderr);
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    // Монотонный таймер ограничивает запуск и работу внешнего процесса.
    let started = Instant::now();
    let mut child = match command.spawn() {
        Ok(child) => child,
        Err(_) => {
            return Ok(Analysis::NotAnalyzed(
                "исполняемый файл openEMS не удалось запустить",
            ));
        }
    };
    let Some(status) = wait(&mut child, started, timeout).map_err(io_error)? else {
        return Ok(Analysis::NotAnalyzed(
            "процесс openEMS исчерпал лимит времени",
        ));
    };
    if !status.success() {
        return Ok(Analysis::NotAnalyzed(
            "openEMS завершился с ненулевым кодом или сигналом",
        ));
    }

    let logs = fs::read_to_string(work.join("stdout")).map_err(io_error)?;
    let stderr = fs::read_to_string(work.join("stderr")).map_err(io_error)?;
    if !stderr.trim().is_empty()
        || logs.lines().any(|line| {
            let line = line.to_ascii_lowercase();
            line.contains("warning:") || line.contains("error:")
        })
    {
        return Ok(Analysis::NotAnalyzed(
            "openEMS сообщил ошибку или предупреждение",
        ));
    }
    if !logs.lines().any(|l| l == "openEMS - enabled basic engine")
        || !logs
            .lines()
            .any(|l| l == "openEMS - fixed number of threads: 1")
        || !logs
            .lines()
            .any(|l| l.starts_with("Exact-endcriteria: evaluating the end criteria every "))
    {
        return Ok(Analysis::NotAnalyzed(
            "openEMS не подтвердил детерминированный режим",
        ));
    }
    let version = logs.lines().find_map(|line| {
        line.trim()
            .strip_prefix("| openEMS ")?
            .split_once(" -- version ")
            .map(|(_, v)| v.trim())
    });
    let csx = logs.lines().find_map(|line| {
        line.trim()
            .strip_prefix("CSXCAD -- Version: ")
            .map(str::trim)
    });
    let Some((version, csx)) = version
        .filter(|v| !v.is_empty())
        .zip(csx.filter(|v| !v.is_empty()))
    else {
        return Ok(Analysis::NotAnalyzed(
            "не определены версии openEMS и CSXCAD",
        ));
    };
    let reached = confirmed_decay(&logs, model.end_energy_ratio)?;
    let excitation_steps = logs.lines().find_map(|line| {
        line.strip_prefix("Excitation signal length is: ")?
            .split_once(" timesteps")?
            .0
            .parse::<u32>()
            .ok()
    });
    let Some((reached, excitation_steps)) = reached.zip(excitation_steps) else {
        return Ok(Analysis::NotAnalyzed(
            "openEMS не подтвердил затухание после возбуждения",
        ));
    };
    if excitation_steps == 0 || reached < excitation_steps || reached > model.max_steps {
        return Ok(Analysis::NotAnalyzed(
            "решение остановилось вне допустимого окна",
        ));
    }

    let Some(stats) = parse_stats(work)? else {
        return Ok(Analysis::NotAnalyzed("отсутствует статистика openEMS"));
    };
    let cells = model
        .grid_m
        .iter()
        .try_fold(1usize, |n, axis| n.checked_mul(axis.len()));
    if Some(stats.cells) != cells || stats.iterations != reached {
        return Err(invalid(
            "статистика openEMS расходится с сеткой или числом шагов",
        ));
    }
    // Ширине разрешения Δf соответствует окно наблюдения не короче 1/Δf.
    let min_span = model.band.max_spacing_hz.recip();
    if stats.duration_s < min_span {
        return Ok(Analysis::NotAnalyzed(
            "окно расчёта короче 1/Δf заявленной полосы",
        ));
    }
    let mut spectra = Vec::with_capacity(model.probes.len());
    for (i, probe) in model.probes.iter().enumerate() {
        let Some(spectrum) = parse_fd(work, model, probe, i, version)? else {
            return Ok(Analysis::NotAnalyzed("отсутствует спектр зонда openEMS"));
        };
        if !spectrum.full_band {
            return Ok(Analysis::NotAnalyzed(
                "в выводе зонда неполная заявленная полоса",
            ));
        }
        let floor = probe.floor.unwrap().value;
        // В FD-таблице вещественная и мнимая части округлены до шести
        // значащих цифр. Нижняя оценка учитывает до 5·10⁻⁶ ошибки каждой части.
        if spectrum
            .probe
            .values
            .iter()
            .any(|value| value.norm() * (1.0 - 5.0e-6) <= floor)
        {
            return Ok(Analysis::NotAnalyzed(
                "в полосе есть отсчёт на уровне спектрального пола или ниже",
            ));
        }
        if !check_trace(work, model, probe, i, version, &stats, &spectrum.probe)? {
            return Ok(Analysis::NotAnalyzed(
                "зонд не покрыл временное окно или частоту Найквиста",
            ));
        }
        spectra.push(spectrum.probe);
    }
    Ok(Analysis::Calculated(OpenEmsResult {
        input_xml: xml,
        openems_version: version.into(),
        csxcad_version: csx.into(),
        iterations: reached,
        timestep_s: stats.dt,
        frequencies_hz: model.band.frequencies_hz.clone(),
        probes: spectra,
    }))
}

struct Stats {
    cells: usize,
    dt: f64,
    iterations: u32,
    duration_s: f64,
}

fn confirmed_decay(logs: &str, energy_ratio: f64) -> Result<Option<u32>, SimError> {
    let mut reports = logs
        .lines()
        .filter_map(|line| line.strip_prefix("RunFDTD: end-criteria of -"));
    let Some(report) = reports.next() else {
        return Ok(None);
    };
    let malformed = || invalid("некорректная строка затухания openEMS");
    if reports.next().is_some() {
        return Err(malformed());
    }
    let (target, rest) = report
        .split_once("dB reached after ")
        .ok_or_else(malformed)?;
    let (steps, actual) = rest.split_once(" timesteps (-").ok_or_else(malformed)?;
    let steps = steps.parse().map_err(|_| malformed())?;
    let target = number(target)?;
    let actual = number(actual.strip_suffix("dB)").ok_or_else(malformed)?)?;
    // openEMS 0.37 печатает положительные модули 10·log₁₀ отношения
    // энергий с шестью значащими цифрами и знаком минус перед каждым.
    if target != native_rounded(-10.0 * energy_ratio.log10()) || actual < target {
        return Ok(None);
    }
    Ok(Some(steps))
}

fn parse_stats(work: &Path) -> Result<Option<Stats>, SimError> {
    let Some(file) = open_output(&work.join("openEMS_stats.txt"))? else {
        return Ok(None);
    };
    let mut lines = file.lines();
    let err = || invalid("некорректная статистика openEMS");
    let mut field = |key: &str| -> Result<String, SimError> {
        let line = lines.next().ok_or_else(err)?.map_err(io_error)?;
        let (value, label) = line.split_once('%').ok_or_else(err)?;
        if label.trim() != key {
            return Err(err());
        }
        Ok(value.trim().into())
    };
    let cells = field("number of cells")?.parse().map_err(|_| err())?;
    let dt = number(&field("timestep (s)")?)?;
    let iterations = field("number of iterations")?.parse().map_err(|_| err())?;
    let duration_s = number(&field("total numerical time (s)")?)?;
    let expected = dt * f64::from(iterations);
    // DumpStatistics печатает 16 значащих цифр. Учитываем округление dt, t и умножения.
    if !positive(dt)
        || !positive(duration_s)
        || !positive(expected)
        || (duration_s - expected).abs() > duration_s * (8.0 * f64::EPSILON)
    {
        return Err(err());
    }
    // Две оставшиеся строки содержат время работы и скорость машины.
    Ok(Some(Stats {
        cells,
        dt,
        iterations,
        duration_s,
    }))
}

struct ParsedSpectrum {
    probe: ProbeSpectrum,
    full_band: bool,
}

fn parse_fd(
    work: &Path,
    model: &Model<'_>,
    probe: &Probe<'_>,
    i: usize,
    version: &str,
) -> Result<Option<ParsedSpectrum>, SimError> {
    let Some(file) = open_output(&work.join(format!("probe_{i}_FD")))? else {
        return Ok(None);
    };
    let mut lines = file.lines();
    let kind = probe_kind(probe.kind);
    let [snapped_start, snapped_stop] =
        probe_header(&mut lines, model, "frequency", kind, version)?;
    columns(&mut lines, &["%", "f/Hz", kind])?;
    columns(&mut lines, &["%", "real", "imag"])?;
    // Вырожденный зонд мог остаться после привязки к сетке.
    let dimensions = (0..3)
        .filter(|&j| snapped_start[j] < snapped_stop[j])
        .count();
    let shape_matches = match probe.kind {
        ProbeKind::Voltage => {
            dimensions == 1
                && (0..3).all(|j| probe.region.extends(j) == (snapped_start[j] < snapped_stop[j]))
        }
        ProbeKind::Current(axis) => {
            dimensions == 2 && snapped_start[axis as usize] == snapped_stop[axis as usize]
        }
    };

    let mut values = Vec::new();
    let mut all_match = shape_matches && (0..3).all(|j| snapped_start[j] <= snapped_stop[j]);
    let mut prev_hz = 0.0;
    for line in lines {
        let line = line.map_err(io_error)?;
        if line.trim().is_empty() {
            continue;
        }
        let [hz, re, im] = row(&line)?;
        let value = Complex::new(re, im);
        if hz <= prev_hz || !value.norm().is_finite() {
            return Err(invalid(
                "частоты повторяются, переставлены или спектр переполнен",
            ));
        }
        prev_hz = hz;
        all_match &= model
            .band
            .frequencies_hz
            .get(values.len())
            .is_some_and(|f| native_rounded(*f) == hz);
        values.push(value);
    }
    Ok(Some(ParsedSpectrum {
        full_band: all_match && values.len() == model.band.frequencies_hz.len(),
        probe: ProbeSpectrum {
            name: probe.name.into(),
            snapped_start,
            snapped_stop,
            values,
        },
    }))
}

fn probe_header(
    lines: &mut impl Iterator<Item = io::Result<String>>,
    model: &Model<'_>,
    domain: &str,
    kind: &str,
    version: &str,
) -> Result<[[usize; 3]; 2], SimError> {
    let header = format!("% {domain}-domain {kind} integration by openEMS {version} @");
    if !lines
        .next()
        .transpose()
        .map_err(io_error)?
        .is_some_and(|line| line.starts_with(&header))
    {
        return Err(invalid("не распознан заголовок зонда openEMS"));
    }
    let mut read_indices = |key| -> Result<[usize; 3], SimError> {
        let err = || invalid("координаты зонда openEMS расходятся с сеткой");
        let line = lines.next().ok_or_else(err)?.map_err(io_error)?;
        let (coordinates, indices) = line
            .strip_prefix(key)
            .and_then(|s| s.split_once(") m -> ["))
            .ok_or_else(err)?;
        let coordinates = coordinates
            .split(',')
            .map(number)
            .collect::<Result<Vec<_>, _>>()?;
        let indices = indices
            .strip_suffix(']')
            .ok_or_else(err)?
            .split(',')
            .map(str::parse)
            .collect::<Result<Vec<usize>, _>>()
            .map_err(|_| err())?;
        let indices: [usize; 3] = indices.try_into().map_err(|_| err())?;
        if coordinates.len() != 3
            || (0..3).any(|j| {
                model.grid_m[j]
                    .get(indices[j])
                    .is_none_or(|&x| native_rounded(x) != coordinates[j])
            })
        {
            return Err(err());
        }
        Ok(indices)
    };
    Ok([
        read_indices("% start-coordinates: (")?,
        read_indices("% stop-coordinates: (")?,
    ])
}

fn columns(
    lines: &mut impl Iterator<Item = io::Result<String>>,
    expected: &[&str],
) -> Result<(), SimError> {
    let line = lines.next().transpose().map_err(io_error)?;
    if line.is_some_and(|line| line.split_whitespace().eq(expected.iter().copied())) {
        return Ok(());
    }
    Err(invalid("не распознаны столбцы зонда openEMS"))
}

fn check_trace(
    work: &Path,
    model: &Model<'_>,
    probe: &Probe<'_>,
    i: usize,
    version: &str,
    stats: &Stats,
    spectrum: &ProbeSpectrum,
) -> Result<bool, SimError> {
    let Some(file) = open_output(&work.join(format!("probe_{i}")))? else {
        return Ok(false);
    };
    let mut lines = file.lines();
    let kind = probe_kind(probe.kind);
    let placement = probe_header(&mut lines, model, "time", kind, version)?;
    if placement != [spectrum.snapped_start, spectrum.snapped_stop] {
        return Err(invalid(
            "независимые файлы зонда openEMS указывают разные координаты",
        ));
    }
    columns(&mut lines, &["%", "t/s", kind])?;

    // Ток берётся с полуцелого шага магнитного поля, напряжение — с целого.
    let first = match probe.kind {
        ProbeKind::Voltage => 0.0,
        ProbeKind::Current(_) => stats.dt / 2.0,
    };
    // ProcessIntegral печатает временные метки с 12 значащими цифрами.
    let rounded =
        |actual: f64, expected: f64| (actual - expected).abs() <= expected.abs() * 5.0e-12;
    let mut regular = true;
    let mut count = 0_u32;
    let mut interval = 0.0;
    let mut last = -1.0;
    for line in lines {
        let line = line.map_err(io_error)?;
        if line.trim().is_empty() {
            continue;
        }
        let [t, _] = row(&line)?;
        if t < 0.0 || t <= last {
            return Err(invalid(
                "временные точки зонда переставлены или повторяются",
            ));
        }
        if count == 1 {
            interval = ((t - first) / stats.dt).round() * stats.dt;
        }
        regular &= rounded(t, first + f64::from(count) * interval);
        last = t;
        count += 1;
    }
    let slack = stats.duration_s * 5.0e-12;
    Ok(regular
        && count >= 2
        && positive(interval)
        && interval < 0.5 / model.band.range_hz[1]
        && last - first + slack >= model.band.max_spacing_hz.recip()
        && last + interval + slack >= stats.duration_s
        && last <= stats.duration_s + first + slack)
}

fn probe_kind(kind: ProbeKind) -> &'static str {
    match kind {
        ProbeKind::Voltage => "voltage",
        ProbeKind::Current(_) => "current",
    }
}

fn open_output(path: &Path) -> Result<Option<BufReader<File>>, SimError> {
    match File::open(path) {
        Ok(file) => Ok(Some(BufReader::new(file))),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(io_error(e)),
    }
}

fn row<const N: usize>(line: &str) -> Result<[f64; N], SimError> {
    let mut items = line.split_whitespace();
    let mut values = [0.0; N];
    for value in &mut values {
        *value = number(
            items
                .next()
                .ok_or_else(|| invalid("неполная строка зонда openEMS"))?,
        )?;
    }
    if items.next().is_some() {
        return Err(invalid("лишний столбец зонда openEMS"));
    }
    Ok(values)
}

fn number(text: &str) -> Result<f64, SimError> {
    text.parse::<f64>()
        .ok()
        .filter(|x| x.is_finite())
        .ok_or_else(|| invalid("неконечное или некорректное число в выводе openEMS"))
}

fn wait(child: &mut Child, started: Instant, timeout: Duration) -> io::Result<Option<ExitStatus>> {
    // Каденция опроса 5 мс не входит в расчёт и ограничена остатком лимита.
    loop {
        let remaining = timeout.saturating_sub(started.elapsed());
        if remaining.is_zero() {
            terminate(child)?;
            return Ok(None);
        }
        match child.try_wait() {
            Ok(Some(status)) => return Ok(Some(status)),
            Ok(None) => std::thread::sleep(remaining.min(Duration::from_millis(5))),
            Err(error) => {
                terminate(child)?;
                return Err(error);
            }
        }
    }
}

fn terminate(child: &mut Child) -> io::Result<()> {
    #[cfg(unix)]
    {
        unsafe extern "C" {
            fn kill(pid: std::ffi::c_int, sig: std::ffi::c_int) -> std::ffi::c_int;
        }
        // SIGKILL = 9 в macOS/Linux. Отрицательный PID адресует отдельную группу.
        // PID дочернего процесса ещё не освобождён через wait.
        if unsafe { kill(-(child.id() as i32), 9) } == 0 {
            return child.wait().map(|_| ());
        }
    }
    if let Err(e) = child.kill()
        && child.try_wait()?.is_none()
    {
        return Err(e);
    }
    child.wait().map(|_| ())
}

struct Temp {
    path: Option<PathBuf>,
}

impl Temp {
    fn new() -> io::Result<Self> {
        let mut builder = fs::DirBuilder::new();
        #[cfg(unix)]
        let root = {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
            PathBuf::from("/tmp")
        };
        #[cfg(not(unix))]
        let root = std::env::temp_dir();
        // Атомарно занимаем свободный слот. Имя не зависит от часов и не входит в результат.
        for slot in 0.. {
            let path = root.join(format!("sim-openems-{}-{slot}", std::process::id()));
            match builder.create(&path) {
                Ok(()) => return Ok(Self { path: Some(path) }),
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {}
                Err(e) => return Err(e),
            }
        }
        unreachable!()
    }

    fn remove(mut self) -> io::Result<()> {
        fs::remove_dir_all(self.path.as_ref().unwrap())?;
        self.path = None;
        Ok(())
    }
}

impl Drop for Temp {
    fn drop(&mut self) {
        if let Some(path) = &self.path {
            let _ = fs::remove_dir_all(path);
        }
    }
}

fn invalid(message: &str) -> SimError {
    SimError::Invalid(format!("openEMS: {message}"))
}

fn io_error(error: io::Error) -> SimError {
    invalid(&format!("ошибка ввода-вывода: {error}"))
}
