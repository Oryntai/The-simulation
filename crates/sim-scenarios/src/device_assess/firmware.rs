//! Запуск образа в эмуляторе платы и проверка наблюдаемых входов и выходов.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::io::Read;
use std::path::Path;

use sim_boards::{BoardId, BoardSim, Connection, pin_spec};
use sim_device::{Device, FirmwareSettings};

use super::{ClosedLoopInput, DeviceReport, Group, Outcome, specified};

const MAX_IMAGE_BYTES: u64 = 16 * 1024 * 1024; // Технический предел чтения образа за одну проверку.
const MAX_WINDOW_S: f64 = 1.0; // Технический предел работы эмулятора на один образ и сценарий.
const MAX_TOTAL_EMULATION_S: f64 = 4.0; // Предельная суммарная стоимость фоновой проверки.

fn image(projects: &Path, name: &str) -> Result<Vec<u8>, (Outcome, String)> {
    let p = Path::new(name);
    if p.extension()
        .and_then(|x| x.to_str())
        .is_none_or(|x| !matches!(x.to_ascii_lowercase().as_str(), "elf" | "hex"))
    {
        return Err((Outcome::Fail, "Требуется образ ELF или Intel HEX".into()));
    }
    let dir = projects
        .canonicalize()
        .map_err(|_| (Outcome::NotAnalyzed, "Папка проектов недоступна".into()))?;
    let file = dir.join(p).canonicalize().map_err(|_| {
        (
            Outcome::NotAnalyzed,
            "Образ прошивки отсутствует или недоступен".into(),
        )
    })?;
    if !file.starts_with(dir) {
        return Err((
            Outcome::Fail,
            "Образ прошивки должен находиться в папке проектов".into(),
        ));
    }
    let metadata = file
        .metadata()
        .map_err(|_| (Outcome::NotAnalyzed, "Не удалось проверить образ".into()))?;
    if !metadata.is_file() || metadata.len() > MAX_IMAGE_BYTES {
        return Err((
            Outcome::Fail,
            "Образ должен быть обычным файлом допустимого размера".into(),
        ));
    }
    let mut bytes = Vec::new();
    let fd = std::fs::File::open(file)
        .map_err(|_| (Outcome::NotAnalyzed, "Образ не удалось открыть".into()))?;
    fd.take(MAX_IMAGE_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|_| (Outcome::NotAnalyzed, "Образ не удалось прочитать".into()))?;
    if bytes.len() as u64 > MAX_IMAGE_BYTES {
        return Err((Outcome::Fail, "Образ превышает допустимый размер".into()));
    }
    Ok(bytes)
}

/// Содержимое образа для отпечатка версии. Недоступный файл даёт пустой отпечаток образа.
pub(super) fn image_bytes(projects: &Path, name: &str) -> Vec<u8> {
    if name.trim().is_empty() {
        return Vec::new();
    }
    image(projects, name).unwrap_or_default()
}

fn append_uart(r: &mut DeviceReport, bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }
    if !r.uart.is_empty() {
        r.uart.push('\n');
    }
    for &byte in bytes {
        match byte {
            b' '..=b'~' | b'\n' | b'\r' | b'\t' => r.uart.push(char::from(byte)),
            _ => {
                let _ = write!(r.uart, "\\x{byte:02x}");
            }
        }
    }
}

fn run_base(board: &mut dyn BoardSim, data: &[u8], seconds: f64, r: &mut DeviceReport) -> bool {
    let result = board.load(data).and_then(|()| board.run_for(seconds));
    let uart = board.uart_take();
    append_uart(r, &uart);
    match result {
        Ok(()) => {
            r.finding(
                Group::Firmware,
                "firmware_execution",
                None,
                "Запуск машинного кода",
                Outcome::Pass,
                format!("Эмуляция платы выполнена в течение {seconds} с"),
            );
            true
        }
        Err(fault) => {
            r.finding(
                Group::Firmware,
                "firmware_fault",
                None,
                "Ошибка образа или исполнения",
                Outcome::Fail,
                fault.to_string(),
            );
            false
        }
    }
}

fn run_case(
    d: &Device,
    id: BoardId,
    data: &[u8],
    seconds: f64,
    case: &FirmwareSettings,
    r: &mut DeviceReport,
) -> Option<CaseResult> {
    let mut report = |outcome, detail: String| {
        r.finding(
            Group::Firmware,
            "firmware_behavior",
            None,
            case.reason
                .as_deref()
                .unwrap_or("Проверка поведения прошивки"),
            outcome,
            detail,
        )
    };
    let Some(out) = case
        .output_pin
        .filter(|_| specified(case.reason.as_deref()))
    else {
        report(
            Outcome::NotAnalyzed,
            "Укажите выход и требование для сценария".into(),
        );
        return None;
    };
    let Some(output) = pin_spec(id, out).filter(|p| p.gpio) else {
        report(
            Outcome::Fail,
            "Неверный выход GPIO для выбранной платы".into(),
        );
        return None;
    };
    let wires = &d.electronics.connections;
    if !wires.iter().any(|c| {
        matches!(c.0,
        Connection::Output { pin, .. } | Connection::Motor { pin, direct: false } if pin == out)
    }) {
        report(
            Outcome::NotAnalyzed,
            format!("Вывод {out} должен быть описан в схеме как выход"),
        );
        return None;
    }
    let digital = case.input_pin.zip(case.input_high);
    let analog = case.adc.zip(case.adc_v);
    if case.input_pin.is_some() != case.input_high.is_some()
        || case.adc.is_some() != case.adc_v.is_some()
    {
        report(
            Outcome::NotAnalyzed,
            "Дополните пару входа и подаваемого сигнала".into(),
        );
        return None;
    }
    if digital.is_some_and(|(pin, _)| !wires.iter().any(|c| matches!(c.0, Connection::DigitalInput { pin: p, .. } if p == pin)))
        || analog.is_some_and(|(channel, volts)| volts < 0.0 || !wires.iter().any(|c| matches!(c.0,
            Connection::AnalogInput { pin, volts: max, .. }
            if pin_spec(id, pin).and_then(|p| p.adc_channel).map(u32::from) == Some(channel) && volts <= max)))
    {
        report(Outcome::Fail, "Сигнал испытания не соответствует описанному входу платы".into());
        return None;
    }
    let pwm = case.pwm_min.zip(case.pwm_max);
    if case.pwm_min.is_some() != case.pwm_max.is_some()
        || (pwm.is_none() && case.output_high.is_none())
    {
        report(
            Outcome::NotAnalyzed,
            "Укажите ожидаемый уровень GPIO или обе границы ШИМ".into(),
        );
        return None;
    }
    if pwm.is_some_and(|(lo, hi)| !output.pwm || !(0.0 <= lo && lo <= hi && hi <= 1.0)) {
        report(
            Outcome::Fail,
            "Неверные границы заполнения ШИМ или выбранный вывод не поддерживает ШИМ".into(),
        );
        return None;
    }
    let mut board = sim_boards::new_board(id);
    if let Err(e) = board.load(data) {
        report(Outcome::Fail, e.to_string());
        return None;
    }
    if let Some((pin, high)) = digital {
        board.set_pin_input(pin, high);
    }
    if let Some((channel, v)) = analog {
        board.set_adc_volts(channel, v);
    }
    let result = board.run_for(seconds);
    let uart = board.uart_take();
    let observed_pin = board.pin_output(out);
    let observed_pwm = board.pwm_duty(out);
    let mut ok = false;
    if let Err(fault) = result {
        report(Outcome::Fail, fault.to_string());
    } else if case.output_high.is_some_and(|v| observed_pin != Some(v))
        || pwm
            .is_some_and(|(lo, hi)| observed_pwm.is_none_or(|d| !d.is_finite() || d < lo || d > hi))
    {
        report(
            Outcome::Fail,
            format!(
                "Выход {out}: ожидание не выполнено. GPIO {observed_pin:?}, ШИМ {observed_pwm:?}"
            ),
        );
    } else {
        report(
            Outcome::Pass,
            format!("Выход {out} соответствует требованию в конце интервала {seconds} с"),
        );
        ok = true;
    }
    append_uart(r, &uart);
    ok.then_some(CaseResult {
        output: out,
        input: digital,
        level: observed_pin,
        duty: observed_pwm,
    })
}

/// Что наблюдалось на выходе в успешном сценарии.
#[derive(Clone, Copy)]
struct CaseResult {
    output: u32,
    input: Option<(u32, bool)>,
    level: Option<bool>,
    duty: Option<f64>,
}

impl CaseResult {
    /// Выход при другом уровне входа действительно другой.
    fn differs_from(&self, other: &Self) -> bool {
        // Меньше одного шага 8-разрядного ШИМ разница неотличима от шума измерения.
        const MIN_DUTY_STEP: f64 = 1.0 / 256.0;
        match (self.duty, other.duty) {
            (Some(a), Some(b)) => (a - b).abs() >= MIN_DUTY_STEP,
            (Some(a), None) | (None, Some(a)) => a >= MIN_DUTY_STEP,
            (None, None) => self.level != other.level,
        }
    }
}

pub(super) fn evaluate(
    d: &Device,
    projects: &Path,
    r: &mut DeviceReport,
    closed_loop: Option<ClosedLoopInput<'_>>,
) {
    evaluate_board(d, projects, r, closed_loop.is_none());
    if let Some(input) = closed_loop {
        evaluate_closed_loop(d, projects, r, input);
    }
}

fn evaluate_board(d: &Device, projects: &Path, r: &mut DeviceReport, legacy: bool) {
    let e = &d.electronics;
    let Some(id) = super::super::device_catalog::board(&e.board) else {
        r.finding(
            Group::Firmware,
            "firmware_board",
            None,
            "Плата прошивки",
            Outcome::NotAnalyzed,
            "Плата неизвестна",
        );
        return;
    };
    if e.firmware.trim().is_empty() {
        r.finding(
            Group::Firmware,
            "missing_firmware",
            None,
            "Образ прошивки",
            Outcome::NotAnalyzed,
            "Файл не указан",
        );
        return;
    }
    let data = match image(projects, &e.firmware) {
        Ok(data) => data,
        Err((outcome, detail)) => {
            r.finding(
                Group::Firmware,
                "firmware_image",
                None,
                "Образ прошивки",
                outcome,
                detail,
            );
            return;
        }
    };
    // Даже при пустом интервале проверяем формат и архитектуру образа. Запуск с нулевым
    // временем не является доказательством исполнения.
    let seconds = e.settings.firmware_s;
    if seconds.is_some_and(|t| t <= 0.0) {
        r.finding(
            Group::Firmware,
            "firmware_time",
            None,
            "Интервал эмуляции",
            Outcome::Fail,
            "Интервал должен быть больше нуля",
        );
        return;
    }
    let mut board = sim_boards::new_board(id);
    let Some(seconds) = seconds.filter(|&t| t <= MAX_WINDOW_S) else {
        if let Err(fault) = board.load(&data) {
            r.finding(
                Group::Firmware,
                "firmware_fault",
                None,
                "Ошибка образа",
                Outcome::Fail,
                fault.to_string(),
            );
        } else {
            r.finding(
                Group::Firmware,
                "firmware_time",
                None,
                "Интервал эмуляции",
                Outcome::NotAnalyzed,
                format!("Укажите длительность проверки, до {MAX_WINDOW_S} с за сценарий"),
            );
        }
        return;
    };
    let boot: Vec<_> = e
        .firmware_cases
        .iter()
        .filter(|c| c.boot == Some(true))
        .collect();
    if let [case] = boot.as_slice() {
        if let Some((pin, high)) = case.input_pin.zip(case.input_high) {
            board.set_pin_input(pin, high);
        }
        if let Some((channel, volts)) = case.adc.zip(case.adc_v) {
            board.set_adc_volts(channel, volts);
        }
    } else if boot.len() > 1 {
        r.finding(
            Group::Firmware,
            "boot_case",
            None,
            "Условия включения платы",
            Outcome::NotAnalyzed,
            "Укажите ровно один сценарий условий включения платы",
        );
        return;
    }
    if !run_base(board.as_mut(), &data, seconds, r) {
        return;
    }
    if (1.0 + e.firmware_cases.len() as f64) * seconds > MAX_TOTAL_EMULATION_S {
        r.finding(Group::Firmware, "firmware_budget", None, "Сценарии прошивки", Outcome::NotAnalyzed,
            format!("Суммарный интервал эмуляции превышает лимит фоновой проверки {MAX_TOTAL_EMULATION_S} с"));
        return;
    }
    let mut responded = false;
    let mut checked = Vec::new();
    for case in &e.firmware_cases {
        if let Some(result) = run_case(d, id, &data, seconds, case, r) {
            // Низкий уровень и ШИМ от нуля выполняет и образ, который ничего не делает.
            responded |= case.output_high == Some(true) || case.pwm_min.is_some_and(|x| x > 0.0);
            checked.push(result);
        }
    }
    if !e.firmware_cases.is_empty() && !responded {
        r.finding(
            Group::Firmware,
            "firmware_response",
            None,
            "Реакция прошивки",
            Outcome::NotAnalyzed,
            "Ни один сценарий не показал активный выход: высокий уровень или ШИМ больше нуля. Пустая прошивка прошла бы такую проверку",
        );
    }
    if e.firmware_cases.is_empty() {
        r.finding(
            Group::Firmware,
            "firmware_coverage",
            None,
            "Работа прошивки",
            Outcome::NotAnalyzed,
            "Успешный запуск не проверяет ни одной заявленной реакции устройства",
        );
    }
    for c in &e.connections {
        if let Connection::Motor { pin, direct: false } = c.0 {
            let reacts = checked.iter().any(|a| {
                a.output == pin
                    && a.input.is_some_and(|(input, high)| {
                        checked.iter().any(|b| {
                            b.output == pin && b.input == Some((input, !high)) && a.differs_from(b)
                        })
                    })
            });
            if !reacts {
                r.finding(
                    Group::Firmware,
                    "motor_control",
                    None,
                    format!("Управление приводом на выводе {pin}"),
                    Outcome::NotAnalyzed,
                    "Нужны проверки выхода при обоих уровнях входа, и выход должен от него меняться",
                );
            }
            if legacy {
                r.finding(Group::Firmware, "closed_loop", None, format!("Защита привода на выводе {pin}"), Outcome::NotAnalyzed,
                    "Модель платы не связывает ШИМ с током, положением и физическими пределами. Переходные процессы и отказы в замкнутом контуре не проверены");
            }
        }
    }
}

fn evaluate_closed_loop(
    d: &Device,
    projects: &Path,
    r: &mut DeviceReport,
    input: ClosedLoopInput<'_>,
) {
    let pins: BTreeSet<_> = d
        .electronics
        .connections
        .iter()
        .filter_map(|connection| match connection.0 {
            Connection::Motor { pin, direct: false } => Some(pin),
            _ => None,
        })
        .collect();
    let links: BTreeSet<_> = d
        .links
        .iter()
        .enumerate()
        .filter_map(|(index, link)| (!link.joint.actuator.is_empty()).then_some(index))
        .collect();
    let expected = input.expected;
    let identity = expected.identity();
    let matching = input.evidence.identity() == identity
        && image(projects, &d.electronics.firmware).is_ok_and(|bytes| {
            !bytes.is_empty()
                && Some(sim_task::Fingerprint::of(&bytes)) == identity.inputs.firmware_image
                && Some(sim_device::measure::design_stamp(d, &bytes))
                    == identity.inputs.legacy_design
        });
    let mut pin_links = BTreeMap::<u32, BTreeSet<usize>>::new();
    let mut link_drives = BTreeMap::<usize, BTreeSet<&str>>::new();
    let mut links_for_drive = BTreeMap::<&str, BTreeSet<usize>>::new();
    let mut complete = input.evidence.uncovered_drives().is_empty();
    for path in &expected.topology().paths {
        let Some(binding) = &path.legacy else {
            complete = false;
            continue;
        };
        pin_links
            .entry(binding.pin)
            .or_default()
            .insert(binding.link);
        link_drives
            .entry(binding.link)
            .or_default()
            .insert(path.drive.as_str());
        links_for_drive
            .entry(path.drive.as_str())
            .or_default()
            .insert(binding.link);
        complete &= pins.contains(&binding.pin) && links.contains(&binding.link);
    }
    complete &= pin_links.keys().copied().eq(pins.iter().copied())
        && link_drives.keys().copied().eq(links.iter().copied())
        && pin_links.values().all(|links| links.len() == 1)
        && link_drives.values().all(|drives| drives.len() == 1)
        && links_for_drive.values().all(|links| links.len() == 1)
        && pin_links.len() == link_drives.len();

    for pin in pins {
        let binding = pin_links
            .get(&pin)
            .filter(|links| links.len() == 1)
            .and_then(|links| links.first().copied());
        let mut outcomes = Vec::new();
        let mut paths = Vec::new();
        if matching
            && let Some(index) = binding
            && pin_links
                .values()
                .filter(|links| links.contains(&index))
                .count()
                == 1
            && let Some(link) = d.links.get(index)
            && let Some(drives) = link_drives.get(&index).filter(|drives| drives.len() == 1)
            && let Some(drive) = drives.first()
            && links_for_drive
                .get(drive)
                .is_some_and(|links| links.len() == 1)
            && expected
                .world()
                .drives
                .iter()
                .any(|compiled| compiled.name == *drive && compiled.part == link.joint.actuator)
        {
            for path in &expected.topology().paths {
                if path
                    .legacy
                    .as_ref()
                    .is_some_and(|binding| binding.pin == pin)
                {
                    let outcome = if Some(path.program) == identity.inputs.firmware_image {
                        input.evidence.board_outcome_for(expected, &path.id)
                    } else {
                        sim_task::Outcome::NotAnalyzed
                    };
                    outcomes.push(outcome);
                    paths.push(path.id.as_str());
                }
            }
        }
        if !complete {
            outcomes.push(sim_task::Outcome::NotAnalyzed);
        }
        let (outcome, detail) = match sim_task::Outcome::aggregate(outcomes) {
            sim_task::Outcome::Pass => (Outcome::Pass, format!("Доказаны режимы и отказы путей {}", paths.join(", "))),
            sim_task::Outcome::Fail => (Outcome::Fail, format!("Наблюдалось нарушение в прогонах путей {}", paths.join(", "))),
            sim_task::Outcome::NotAnalyzed => (Outcome::NotAnalyzed,
                "Нет полного доказательства для этой версии, выводов платы и соответствующих приводов".into()),
        };
        r.finding(
            Group::Firmware,
            "closed_loop",
            binding,
            format!("Защита привода на выводе {pin}"),
            outcome,
            detail,
        );
    }
}
