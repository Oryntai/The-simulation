use std::collections::BTreeSet;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{Error, Fingerprint, Topology, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceKind {
    Datasheet,
    Paper,
    Measured,
    Derived,
    Estimate,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Source {
    pub kind: SourceKind,
    pub reference: String,
}

impl Source {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        check_name(&self.reference)
    }

    pub(crate) fn can_prove(&self) -> bool {
        self.kind != SourceKind::Estimate
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Signal {
    pub name: String,
    pub unit: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    SensorUnavailable,
    WireOpen,
    ControllerHalted,
    DriverDisabled,
    PowerCut,
    DriveDisabled,
    PacketLoss,
    PacketDelay,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fault {
    pub node: String,
    pub kind: FaultKind,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    pub path: String,
    pub mode: String,
    pub fault: Option<Fault>,
}

/// Вход задачи. Начальные условия используют ту же форму с at_ns = 0.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Stimulus {
    pub id: String,
    pub signal: Signal,
    pub at_ns: u64,
    pub value: f64,
    /// Контрольное значение того же стимула. Без него нельзя доказать
    /// зависимость команды от возбуждения в штатном опыте.
    pub comparison_value: Option<f64>,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Injection {
    pub fault: Fault,
    pub at_ns: u64,
    pub extra_delay_ns: Option<u64>,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FaultSweep {
    at_ns: u64,
    packet_delay_ns: Option<u64>,
    source: Source,
    #[serde(default)]
    invariant_limits: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mode {
    pub id: String,
    pub horizon_ns: u64,
    pub before_ns: u64,
    pub after_ns: u64,
    pub max_gap_ns: u64,
    pub source: Source,
    pub excitation: String,
    pub stimuli: Vec<Stimulus>,
    /// Парный опыт для диагностики даже при ограничениях физической модели.
    #[serde(default)]
    pub diagnostic_comparison: bool,
    #[serde(default)]
    pub faults: Vec<Injection>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Limit {
    Range {
        signal: Signal,
        min: Option<f64>,
        max: Option<f64>,
        source: Source,
    },
    Forbidden {
        signal: Signal,
        source: Source,
    },
}

impl Limit {
    pub(crate) fn signal(&self) -> &Signal {
        match self {
            Self::Range { signal, .. } | Self::Forbidden { signal, .. } => signal,
        }
    }
}

/// Время первой реакции физики относительно фактического воздействия.
/// Положительный min_change проверяется по модулю относительно последнего значения до реакции.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Response {
    pub signal: Signal,
    pub min_change: f64,
    pub within_ns: u64,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Criteria {
    pub case: Case,
    pub limits: Vec<Limit>,
    pub response: Option<Response>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    version: u32,
    control: String,
    profile: Option<String>,
    modes: Vec<Mode>,
    criteria: Vec<Criteria>,
    fault_sweep: Option<FaultSweep>,
}

/// Проверенная неизменяемая схема. Отпечаток берётся по исходным байтам файла.
#[derive(Clone, Debug)]
pub struct Task {
    document: Document,
    fingerprint: Fingerprint,
    matrix_expanded: bool,
}

impl Task {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let contents = std::fs::read_to_string(path)
            .map_err(|error| invalid(format!("{}: {error}", path.display())))?;
        Self::parse(&contents)
    }

    pub fn parse(text: &str) -> Result<Self, Error> {
        let document: Document = toml::from_str(text).map_err(|e| invalid(e.to_string()))?;
        // Версия 1 задаёт время в целых наносекундах и фактические окна вокруг воздействия.
        if document.version != 1 {
            return Err(invalid("неподдерживаемая версия задания"));
        }
        check_name(&document.control)?;
        if let Some(profile) = &document.profile {
            check_name(profile)?;
        }
        if let Some(sweep) = &document.fault_sweep {
            sweep.source.validate()?;
            if sweep.packet_delay_ns == Some(0) {
                return Err(invalid("задержка матрицы должна быть положительной"));
            }
        }
        let mut modes = BTreeSet::new();
        for mode in &document.modes {
            mode.validate()?;
            if !modes.insert(&mode.id) {
                return Err(invalid(format!("повтор режима «{}»", mode.id)));
            }
        }
        let mut cases = BTreeSet::new();
        for criteria in &document.criteria {
            criteria.validate()?;
            if !modes.contains(&criteria.case.mode) || !cases.insert(&criteria.case) {
                return Err(invalid(
                    "критерии повторены или ссылаются на неизвестный режим",
                ));
            }
        }
        Ok(Self {
            document,
            fingerprint: Fingerprint::of(text.as_bytes()),
            matrix_expanded: false,
        })
    }

    pub fn control(&self) -> &str {
        &self.document.control
    }

    pub fn profile(&self) -> Option<&str> {
        self.document.profile.as_deref()
    }

    pub fn fingerprint(&self) -> Fingerprint {
        self.fingerprint
    }

    pub fn modes(&self) -> &[Mode] {
        &self.document.modes
    }

    pub fn criteria(&self) -> &[Criteria] {
        &self.document.criteria
    }

    /// Дополняет индивидуальные расписания отказов правилом задачи.
    /// Без заданного времени или величины задержки новый опыт не создаётся.
    pub fn expand_faults(&mut self, topology: &Topology) -> Result<(), Error> {
        let Some(sweep) = &self.document.fault_sweep else {
            return Ok(());
        };
        for mode in &self.document.modes {
            mode.check_window(sweep.at_ns)
                .map_err(|error| invalid(format!("матрица, режим «{}»: {error}", mode.id)))?;
        }
        for case in topology.cases() {
            let Some(fault) = &case.fault else { continue };
            let Some(mode) = self
                .document
                .modes
                .iter_mut()
                .find(|mode| mode.id == case.mode)
            else {
                continue;
            };
            let extra_delay_ns = if fault.kind == FaultKind::PacketDelay {
                if sweep.packet_delay_ns.is_none() {
                    continue;
                }
                sweep.packet_delay_ns
            } else {
                None
            };
            if mode.injection(fault).is_none() {
                mode.faults.push(Injection {
                    fault: fault.clone(),
                    at_ns: sweep.at_ns,
                    extra_delay_ns,
                    source: sweep.source.clone(),
                });
            }
            if sweep.invariant_limits
                && !self.document.criteria.iter().any(|item| item.case == case)
                && let Some(nominal) = self.document.criteria.iter().find(|item| {
                    item.case.path == case.path
                        && item.case.mode == case.mode
                        && item.case.fault.is_none()
                })
            {
                // Абсолютные ограничения остаются обязательными под отказом.
                // Реакция на штатное возбуждение не заменяет критерий защиты.
                self.document.criteria.push(Criteria {
                    case,
                    limits: nominal.limits.clone(),
                    response: None,
                });
            }
        }
        self.matrix_expanded = true;
        Ok(())
    }

    pub(crate) fn assessment_fingerprint(&self) -> Fingerprint {
        if self.matrix_expanded {
            Fingerprint::of(
                &[b"matrix-full".as_slice(), &self.fingerprint.0.to_le_bytes()].concat(),
            )
        } else {
            self.fingerprint
        }
    }

    pub(crate) fn mode(&self, name: &str) -> Option<&Mode> {
        self.modes().iter().find(|mode| mode.id == name)
    }
}

impl Mode {
    pub(crate) fn injection(&self, fault: &Fault) -> Option<&Injection> {
        self.faults
            .iter()
            .find(|injection| &injection.fault == fault)
    }

    fn validate(&self) -> Result<(), Error> {
        check_name(&self.id)?;
        self.source.validate()?;
        if self.before_ns == 0
            || self.after_ns == 0
            || self.max_gap_ns == 0
            || self.max_gap_ns > self.before_ns.min(self.after_ns)
        {
            return Err(invalid("некорректные длительности окна или шаг наблюдения"));
        }
        let mut stimuli = BTreeSet::new();
        for input in &self.stimuli {
            check_name(&input.id)?;
            input.source.validate()?;
            input.signal.validate()?;
            finite(input.value)?;
            if let Some(value) = input.comparison_value {
                finite(value)?;
                if value == input.value {
                    return Err(invalid("контрольное значение не меняет стимул"));
                }
            }
            if !stimuli.insert(&input.id) || input.at_ns > self.horizon_ns {
                return Err(invalid("повтор стимула или время стимула за горизонтом"));
            }
        }
        let excitation = self
            .stimuli
            .iter()
            .find(|item| item.id == self.excitation)
            .ok_or_else(|| invalid("возбуждение отсутствует в списке стимулов"))?;
        self.check_window(excitation.at_ns)?;
        let mut faults = BTreeSet::new();
        for injection in &self.faults {
            check_name(&injection.fault.node)?;
            injection.source.validate()?;
            self.check_window(injection.at_ns)?;
            let delay = injection.fault.kind == FaultKind::PacketDelay;
            if !faults.insert(&injection.fault)
                || (delay && injection.extra_delay_ns.is_none_or(|ns| ns == 0))
                || (!delay && injection.extra_delay_ns.is_some())
            {
                return Err(invalid("повтор отказа или неверная величина задержки"));
            }
        }
        Ok(())
    }

    fn check_window(&self, ns: u64) -> Result<(), Error> {
        if ns < self.before_ns
            || ns
                .checked_add(self.after_ns)
                .is_none_or(|end| end > self.horizon_ns)
        {
            return Err(invalid(
                "окно до и после воздействия не помещается в горизонт",
            ));
        }
        Ok(())
    }
}

impl Criteria {
    fn validate(&self) -> Result<(), Error> {
        check_name(&self.case.path)?;
        if let Some(fault) = &self.case.fault {
            check_name(&fault.node)?;
        }
        for limit in &self.limits {
            limit.signal().validate()?;
            match limit {
                Limit::Range {
                    min, max, source, ..
                } => {
                    source.validate()?;
                    for value in min.iter().chain(max) {
                        finite(*value)?;
                    }
                    if (min.is_none() && max.is_none())
                        || min.zip(*max).is_some_and(|(lo, hi)| lo > hi)
                    {
                        return Err(invalid("границы диапазона заданы неверно"));
                    }
                }
                Limit::Forbidden { signal, source } => {
                    source.validate()?;
                    // По схеме 1 событие истины это безразмерный двоичный сигнал 0/1.
                    if !matches!(signal.unit.as_str(), "1" | "bool") {
                        return Err(invalid(
                            "запретное событие должно иметь единицу «1» или «bool»",
                        ));
                    }
                }
            }
        }
        if let Some(response) = &self.response {
            response.signal.validate()?;
            response.source.validate()?;
            finite(response.min_change)?;
            if response.min_change <= 0.0 || response.within_ns == 0 {
                return Err(invalid("порог и срок реакции должны быть положительны"));
            }
        }
        Ok(())
    }
}

impl Signal {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        check_name(&self.name)?;
        check_name(&self.unit)
    }
}

pub(crate) fn check_name(text: &str) -> Result<(), Error> {
    if text.trim().is_empty() {
        return Err(invalid("пустое имя, единица или источник"));
    }
    Ok(())
}

pub(crate) fn finite(value: f64) -> Result<(), Error> {
    if !value.is_finite() {
        return Err(invalid("неконечное значение входа или метрики"));
    }
    Ok(())
}
