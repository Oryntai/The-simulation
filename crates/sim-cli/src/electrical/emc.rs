//! Опциональные постановки ЭМС для электрического манифеста формата 1.

use std::{collections::BTreeMap, path::Path};

use serde::Deserialize;
use serde_json::{Value, json};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        Endpoint, Finding, Parameter, ResetProfile, Scope, Status,
        emi_reset::{
            EmiBand, EmiCriterion, EmiEvidence, EmiInput, EmiInterval, EmiOutcome, EmiPacketProbe,
            EmiReturn, EmiSourceKind, EmiTransfer, EmiTrial, prepare_emi_reset,
        },
        esd_event::{
            EsdCriterion, EsdDomain, EsdEvidence, EsdInput, EsdLevel, EsdMargin, EsdNetwork,
            EsdPinLimits, EsdPolarity, EsdPort, EsdProtection, EsdPulse, EsdSample, EsdTvs,
            assess_esd_event,
        },
        sim_circuit::emc::loop_model::{LumpedDomain, SeriesRl},
    },
};
use sim_runtime::Runtime;
use sim_signal::Time;
use sim_task::{
    Fingerprint, Source, SourceKind,
    verdict::{ElectricalEvidence, HazardClass},
};
use sim_world::Provenance;

use super::input::{SchematicInput, parse};

const ESD_MEASUREMENT: &str = "стендовый замер остаточных напряжения и тока на защищаемом выводе при адресованном контактном разряде";
const EMI_MEASUREMENT: &str = "синхронный стендовый замер дифференциального напряжения входа, сброса и доставки при коммутации";

/// Файл, на который ссылается `assemblies.<id>.emc` манифеста.
/// Имена в `instances` относятся к экземплярам схем в мире этой сборки.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EmcProfile {
    format: u32,
    pub(super) instances: BTreeMap<String, Checks>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Checks {
    emi_reset: Option<ResetCase>,
    esd_event: Option<EsdCase>,
}

impl Checks {
    pub(super) fn emi_trial(&self, scope: Scope) -> Result<Option<EmiTrial>, String> {
        let Some(case) = &self.emi_reset else {
            return Ok(None);
        };
        let (input, _) = case.clone().resolve(scope)?;
        match prepare_emi_reset(input).map_err(|err| err.to_string())? {
            Analysis::Analyzed { value, .. } => Ok(Some(value)),
            Analysis::NotAnalyzed { .. } => Ok(None),
        }
    }
}

impl EmcProfile {
    pub fn parse(path: &Path, bytes: &[u8]) -> Result<Self, String> {
        let profile: Self = parse(path, bytes)?;
        if profile.format != 1 || profile.instances.is_empty() {
            return Err(format!(
                "{}: нужен непустой профиль ЭМС формата 1",
                path.display()
            ));
        }
        Ok(profile)
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ResetCase {
    source: String,
    #[serde(with = "ResetSource")]
    source_kind: EmiSourceKind,
    board: String,
    input: String,
    reference: String,
    binding: Option<Source>,
    start_ns: u64,
    end_ns: u64,
    bias_v: Option<Range>,
    detector: Option<Detector>,
    transfer: Option<Transfer>,
    packet: Option<Packet>,
}

#[derive(Deserialize)]
#[serde(remote = "EmiSourceKind", rename_all = "snake_case")]
enum ResetSource {
    Motor,
    Switch,
}

impl ResetCase {
    fn resolve(self, scope: Scope) -> Result<(EmiInput, Option<EmiPacketProbe>), String> {
        let input = EmiInput {
            scope,
            source: self.source,
            source_kind: self.source_kind,
            board: self.board,
            input: endpoint(&self.input)?,
            reference: endpoint(&self.reference)?,
            binding: provenance(self.binding)?,
            start: Time(self.start_ns),
            end: Time(self.end_ns),
            bias_v: parameter(self.bias_v, "замерить дифференциальное напряжение в покое")?,
            detector: self.detector.map(Detector::resolve).transpose()?,
            transfer: self.transfer.map(Transfer::resolve).transpose()?,
        };
        let packet = self.packet.map(Packet::resolve).transpose()?;
        Ok((input, packet))
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Range {
    min: Option<f64>,
    nominal: Option<f64>,
    max: Option<f64>,
    source: Option<Source>,
}

impl Range {
    fn resolve(self) -> Result<Option<SourcedRange>, String> {
        let source = provenance(self.source)?;
        let (Some(min), Some(nominal), Some(max)) = (self.min, self.nominal, self.max) else {
            return Ok(None);
        };
        if min > nominal || nominal > max {
            return Err("нарушен порядок границ диапазона ЭМС".into());
        }
        Ok(source.map(|provenance| SourcedRange {
            min,
            nominal,
            max,
            provenance,
        }))
    }
}

fn parameter(range: Option<Range>, measurement: &str) -> Result<Parameter, String> {
    Ok(Parameter {
        range: range.map(Range::resolve).transpose()?.flatten(),
        measurement: measurement.into(),
    })
}

fn provenance(source: Option<Source>) -> Result<Option<Provenance>, String> {
    source
        .map(|source| {
            let reference = source.reference;
            if reference.trim().is_empty() || reference.chars().any(char::is_control) {
                return Err("нужно описание источника данных ЭМС".into());
            }
            Ok(match source.kind {
                SourceKind::Datasheet => Provenance::Datasheet(reference),
                SourceKind::Paper => Provenance::Paper(reference),
                SourceKind::Measured => Provenance::Measured(reference),
                SourceKind::Derived => Provenance::Derived(reference),
                SourceKind::Estimate => Provenance::Estimate(reference),
            })
        })
        .transpose()
}

fn endpoint(address: &str) -> Result<Endpoint, String> {
    let (component, pin) = address
        .split_once('/')
        .ok_or_else(|| format!("«{address}»: нужен адрес компонент/вывод"))?;
    Ok(Endpoint {
        component: component.into(),
        pin: pin.into(),
    })
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Detector {
    falling_v: Option<Range>,
    rising_v: Option<Range>,
    assert_delay_s: Option<Range>,
    release_delay_s: Option<Range>,
}

impl Detector {
    fn resolve(self) -> Result<ResetProfile, String> {
        Ok(ResetProfile {
            falling_v: parameter(self.falling_v, "замерить порог сброса")?,
            rising_v: parameter(self.rising_v, "замерить порог отпускания")?,
            assert_delay_s: parameter(self.assert_delay_s, "замерить выдержку сброса")?,
            release_delay_s: parameter(self.release_delay_s, "замерить выдержку отпускания")?,
        })
    }
}

#[derive(Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Transfer {
    Measured {
        source: Option<Source>,
        #[serde(default)]
        intervals: Vec<MeasuredInterval>,
    },
    SharedReturn {
        geometry: String,
        #[serde(with = "Domain")]
        domain: LumpedDomain,
        resistance_ohm: f64,
        inductance_h: f64,
        highest_frequency_hz: f64,
        model_source: Option<Source>,
        remainder_v: Option<Range>,
        #[serde(default)]
        intervals: Vec<ReturnInterval>,
    },
}

#[derive(Deserialize)]
#[serde(remote = "LumpedDomain", deny_unknown_fields)]
struct Domain {
    max_frequency_hz: f64,
    longest_path_m: f64,
    min_wave_speed_m_s: f64,
    max_wavelength_fraction: f64,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MeasuredInterval {
    end_ns: u64,
    delta_v: [f64; 3],
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReturnInterval {
    end_ns: u64,
    current_a: [f64; 3],
    slope_a_s: [f64; 3],
}

fn band([lower, nominal, upper]: [f64; 3]) -> EmiBand {
    EmiBand {
        lower,
        nominal,
        upper,
    }
}

impl Transfer {
    fn resolve(self) -> Result<EmiTransfer, String> {
        Ok(match self {
            Self::Measured { source, intervals } => EmiTransfer::Measured {
                intervals: intervals
                    .into_iter()
                    .map(|row| EmiInterval {
                        end: Time(row.end_ns),
                        value: band(row.delta_v),
                    })
                    .collect(),
                source: provenance(source)?,
            },
            Self::SharedReturn {
                geometry,
                domain,
                resistance_ohm,
                inductance_h,
                highest_frequency_hz,
                model_source,
                remainder_v,
                intervals,
            } => EmiTransfer::SharedReturn(EmiReturn {
                geometry,
                domain,
                rl: SeriesRl {
                    resistance_ohm,
                    inductance_h,
                },
                highest_frequency_hz,
                model_source: provenance(model_source)?,
                remainder_v: parameter(remainder_v, "замерить остаток вне эквивалента возврата")?,
                intervals: intervals
                    .into_iter()
                    .map(|row| EmiInterval {
                        end: Time(row.end_ns),
                        value: (band(row.current_a), band(row.slope_a_s)),
                    })
                    .collect(),
            }),
        })
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Packet {
    route: String,
    sent_ns: u64,
    deadline_ns: u64,
    source: Option<Source>,
}

impl Packet {
    fn resolve(self) -> Result<EmiPacketProbe, String> {
        Ok(EmiPacketProbe {
            route: self.route,
            sent_at: Time(self.sent_ns),
            deadline: Time(self.deadline_ns),
            source: provenance(self.source)?,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EsdCase {
    exposed: Option<String>,
    protected: Option<String>,
    #[serde(rename = "return")]
    return_: Option<String>,
    start_ns: Option<u64>,
    exposure: Option<Source>,
    pulse: Option<EsdPulseProfile>,
    network: Option<EsdNetworkProfile>,
    limits: Option<EsdLimitsProfile>,
}

#[derive(Deserialize)]
#[serde(remote = "EsdLevel", rename_all = "snake_case")]
enum Level {
    Kv2,
    Kv4,
    Kv6,
    Kv8,
}

#[derive(Deserialize)]
#[serde(remote = "EsdPolarity", rename_all = "snake_case")]
enum Polarity {
    Positive,
    Negative,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EsdPulseProfile {
    #[serde(default, deserialize_with = "optional_level")]
    level: Option<EsdLevel>,
    #[serde(default, deserialize_with = "optional_polarity")]
    polarity: Option<EsdPolarity>,
    #[serde(default)]
    samples_ns_a: Vec<[f64; 2]>,
    bandwidth_hz: Option<f64>,
    source: Option<Source>,
}

fn optional_level<'de, D: serde::Deserializer<'de>>(
    input: D,
) -> Result<Option<EsdLevel>, D::Error> {
    Level::deserialize(input).map(Some)
}

fn optional_polarity<'de, D: serde::Deserializer<'de>>(
    input: D,
) -> Result<Option<EsdPolarity>, D::Error> {
    Polarity::deserialize(input).map(Some)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EsdNetworkProfile {
    series_ohm: Option<f64>,
    pin_ohm: Option<f64>,
    domain: Option<EsdDomainProfile>,
    protection: Option<EsdProtectionProfile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EsdDomainProfile {
    bandwidth_hz: Option<f64>,
    length_m: Option<f64>,
    wave_speed_m_s: Option<f64>,
    max_length_wavelengths: Option<f64>,
    max_terminal_abs_v: Option<f64>,
    voltage_error_v: Option<f64>,
    current_error_a: Option<f64>,
    source: Option<Source>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum EsdProtectionProfile {
    Absent {
        source: Option<Source>,
    },
    Tvs {
        #[serde(default, deserialize_with = "optional_polarity")]
        polarity: Option<EsdPolarity>,
        knee_v: Option<f64>,
        dynamic_ohm: Option<f64>,
        max_injected_peak_a: Option<f64>,
        min_rise_ns: Option<f64>,
        max_pulse_ns: Option<f64>,
        datasheet: Option<Source>,
    },
}

impl EsdProtectionProfile {
    fn resolve<'a>(&self, source: &'a Option<Provenance>) -> Option<EsdProtection<'a>> {
        match *self {
            Self::Absent { .. } => source.as_ref().map(EsdProtection::Absent),
            Self::Tvs {
                polarity,
                knee_v,
                dynamic_ohm,
                max_injected_peak_a,
                min_rise_ns,
                max_pulse_ns,
                ..
            } => Some(EsdProtection::Tvs(EsdTvs {
                polarity: polarity?,
                knee_v: knee_v?,
                dynamic_ohm: dynamic_ohm?,
                max_injected_peak_a: max_injected_peak_a?,
                min_rise_s: min_rise_ns? * SECONDS_PER_NS,
                max_pulse_s: max_pulse_ns? * SECONDS_PER_NS,
                datasheet: source.as_ref(),
            })),
        }
    }

    fn source(&self) -> Option<Source> {
        match self {
            Self::Absent { source } => source.clone(),
            Self::Tvs { datasheet, .. } => datasheet.clone(),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EsdLimitsProfile {
    voltage_abs_v: Option<Range>,
    current_abs_a: Option<Range>,
}

impl EsdLimitsProfile {
    fn resolve(&self) -> Result<Option<EsdPinLimits>, String> {
        let voltage = self
            .voltage_abs_v
            .clone()
            .map(Range::resolve)
            .transpose()?
            .flatten();
        let current = self
            .current_abs_a
            .clone()
            .map(Range::resolve)
            .transpose()?
            .flatten();
        Ok(voltage
            .zip(current)
            .map(|(voltage_abs_v, current_abs_a)| EsdPinLimits {
                voltage_abs_v,
                current_abs_a,
            }))
    }
}

// Перевод дробных наносекунд в секунды без округления до целого такта.
const SECONDS_PER_NS: f64 = 1e-9;

impl EsdCase {
    fn evaluate(
        &self,
        profile: Fingerprint,
        scheme: &SchematicInput,
        evidence: &mut ElectricalEvidence,
    ) -> Result<Value, String> {
        let [exposed, protected, return_] = [&self.exposed, &self.protected, &self.return_]
            .map(|address| address.as_deref().map(endpoint).transpose());
        let [exposed, protected, return_] = [exposed?, protected?, return_?];
        for end in [&exposed, &protected, &return_].into_iter().flatten() {
            if !scheme.compiled.pins().any(|pin| pin == end) {
                return Err(format!("неизвестный вывод ESD «{end}»"));
            }
        }
        let (Some(exposed), Some(protected), Some(return_), Some(start_ns)) =
            (exposed, protected, return_, self.start_ns)
        else {
            let reason = "нужны адреса трёх клемм и метка начала контактного разряда";
            evidence
                .record_emc_open(HazardClass::EsdEvent, "esd_event", profile, reason)
                .map_err(|err| err.to_string())?;
            return Ok(open(reason, profile, ESD_MEASUREMENT));
        };
        let port = EsdPort {
            exposed,
            protected,
            return_,
        };
        let exposure = provenance(self.exposure.clone())?;
        let pulse_source = provenance(self.pulse.as_ref().and_then(|pulse| pulse.source.clone()))?;
        let samples: Vec<_> = self
            .pulse
            .iter()
            .flat_map(|pulse| &pulse.samples_ns_a)
            .map(|&[offset_ns, current_abs_a]| EsdSample {
                offset_s: offset_ns * SECONDS_PER_NS,
                current_abs_a,
            })
            .collect();
        let pulse = self.pulse.as_ref().and_then(|pulse| {
            Some(EsdPulse {
                level: pulse.level?,
                polarity: pulse.polarity?,
                samples: &samples,
                bandwidth_hz: pulse.bandwidth_hz?,
                source: pulse_source.as_ref(),
            })
        });
        let domain_source = provenance(
            self.network
                .as_ref()
                .and_then(|network| network.domain.as_ref())
                .and_then(|domain| domain.source.clone()),
        )?;
        let protection = self
            .network
            .as_ref()
            .and_then(|network| network.protection.as_ref());
        let protection_source = provenance(protection.and_then(EsdProtectionProfile::source))?;
        let network = self.network.as_ref().and_then(|network| {
            let domain = network.domain.as_ref()?;
            Some(EsdNetwork {
                series_ohm: network.series_ohm?,
                pin_ohm: network.pin_ohm?,
                domain: EsdDomain {
                    bandwidth_hz: domain.bandwidth_hz?,
                    length_m: domain.length_m?,
                    wave_speed_m_s: domain.wave_speed_m_s?,
                    max_length_wavelengths: domain.max_length_wavelengths?,
                    max_terminal_abs_v: domain.max_terminal_abs_v?,
                    voltage_error_v: domain.voltage_error_v?,
                    current_error_a: domain.current_error_a?,
                    source: domain_source.as_ref(),
                },
                protection: protection.and_then(|profile| profile.resolve(&protection_source)),
            })
        });
        let limits = self
            .limits
            .as_ref()
            .map(EsdLimitsProfile::resolve)
            .transpose()?
            .flatten();
        let input = EsdInput {
            port: &port,
            start: Time(start_ns),
            exposure: exposure.as_ref(),
            pulse,
            network,
            limits: limits.as_ref(),
        };
        let result = evidence
            .record_esd_event(
                &scheme.compiled,
                &scheme.modes,
                "esd_event",
                profile,
                |compiled| assess_esd_event(compiled, &input),
            )
            .map_err(|err| err.to_string())?;
        Ok(esd_report(result, profile))
    }
}

fn esd_report(result: Finding<EsdEvidence>, profile: Fingerprint) -> Value {
    let evidence = result.evidence;
    let (response, scope) = match evidence.response {
        Analysis::Analyzed { value, scope } => (
            Some(json!({
                "terminal_v": value.peak.terminal_v, "pin_v": value.peak.pin_v,
                "pin_a": value.peak.pin_a, "tvs_a": value.peak.tvs_a,
                "peak_offset_s": value.peak_offset_s,
                "electrical_length_wavelengths": value.electrical_length_wavelengths,
            })),
            Some(scope),
        ),
        Analysis::NotAnalyzed { .. } => (None, None),
    };
    let margin = |range: EsdMargin| [range.lower, range.nominal, range.upper];
    let (voltage, current) = match evidence.margins {
        Analysis::Analyzed { value, .. } => {
            (Some(margin(value.voltage_v)), Some(margin(value.current_a)))
        }
        Analysis::NotAnalyzed { .. } => (None, None),
    };
    json!({
        "status": if result.status == Status::Fail { "fail" } else { "not_analyzed" },
        "reason": result.reason, "profile": profile, "required_measurement": ESD_MEASUREMENT,
        "model_scope": scope, "start_ns": evidence.start.0,
        "exposed": evidence.port.exposed.to_string(), "protected": evidence.port.protected.to_string(),
        "return": evidence.port.return_.to_string(), "response": response,
        "voltage_margin_v": voltage, "current_margin_a": current,
        "bench_criterion": evidence.criterion.map(|criterion| match criterion {
            EsdCriterion::Exceeded => "exceeded",
            EsdCriterion::WithinBench => "within_bench",
            EsdCriterion::Overlap => "overlap",
        }),
    })
}

pub(super) fn evaluate(
    checks: &Checks,
    profile: Fingerprint,
    scheme: &SchematicInput,
    evidence: &mut ElectricalEvidence,
    pair: Option<(&Runtime, &Runtime)>,
) -> Result<BTreeMap<&'static str, Value>, String> {
    let open_finding =
        |evidence: &mut ElectricalEvidence, class: HazardClass, reason: &str, measurement| {
            evidence
                .record_emc_open(class, class.spec().id, profile, reason)
                .map_err(|err| err.to_string())?;
            Ok::<_, String>(open(reason, profile, measurement))
        };
    let esd = if let Some(case) = &checks.esd_event {
        case.evaluate(profile, scheme, evidence)?
    } else {
        open_finding(
            evidence,
            HazardClass::EsdEvent,
            "нужны адресованный контактный импульс, применимый эквивалент защиты и стендовые U/I",
            ESD_MEASUREMENT,
        )?
    };
    let emi = if let Some(case) = &checks.emi_reset {
        let (input, packet) = case.clone().resolve(scheme.compiled.depth_scope())?;
        let result = evidence
            .record_emi_reset(
                &scheme.compiled,
                &scheme.modes,
                "emi_reset",
                profile,
                input,
                pair,
                packet.as_ref(),
            )
            .map_err(|err| err.to_string())?;
        reset_report(result, profile)
    } else {
        open_finding(
            evidence,
            HazardClass::EmiReset,
            "нужны источник помехи, адресованный вход и синхронный замер напряжения и сброса",
            EMI_MEASUREMENT,
        )?
    };
    Ok(BTreeMap::from([("esd_event", esd), ("emi_reset", emi)]))
}

fn open(reason: &str, profile: Fingerprint, measurement: &str) -> Value {
    json!({
        "status": "not_analyzed", "reason": reason, "profile": profile, "required_measurement": measurement,
    })
}

fn reset_report(assessment: Analysis<EmiEvidence>, profile: Fingerprint) -> Value {
    let (evidence, scope) = match assessment {
        Analysis::NotAnalyzed { reason } => return open(&reason, profile, EMI_MEASUREMENT),
        Analysis::Analyzed { value, scope } => (value, scope),
    };
    let (status, reason, event_ns) = match evidence.outcome {
        EmiOutcome::Fail { at } => (
            "fail",
            "возмущение вызвало подтверждённый сброс адресованной платы".into(),
            Some(at.0),
        ),
        EmiOutcome::NotAnalyzed { reason } => ("not_analyzed", reason, None),
    };
    let observation = evidence.observation.map(|obs| {
        json!({
            "observed_until_ns": obs.observed_until.0,
            "complete": obs.complete,
            "reset_ns": obs.reset_at.map(|at| at.0),
            "external_asserted_ns": obs.external_asserted_at.map(|at| at.0),
            "baseline_reset_ns": obs.baseline_reset_at.map(|at| at.0),
        })
    });
    let packet = evidence.packet.map(|packet| {
        json!({
            "route": packet.route, "id": packet.id, "sent_ns": packet.sent_at.0,
            "deadline_ns": packet.deadline.0, "baseline_received_ns": packet.baseline_received_at.0,
            "disturbed_received_ns": packet.disturbed_received_at.map(|at| at.0),
        })
    });
    let voltage = evidence.margins.voltage_v;
    json!({
        "status": status, "reason": reason, "profile": profile, "event_ns": event_ns,
        "model_scope": scope, "required_measurement": EMI_MEASUREMENT,
        "source": evidence.source,
        "source_kind": match evidence.source_kind { EmiSourceKind::Motor => "motor", EmiSourceKind::Switch => "switch" },
        "board": evidence.board, "input": evidence.input.to_string(), "reference": evidence.reference.to_string(),
        "bench_criterion": evidence.criterion.map(|criterion| match criterion {
            EmiCriterion::Violated => "violated",
            EmiCriterion::WithinBench => "within_bench",
        }),
        "voltage_margin_v": [voltage.lower, voltage.nominal, voltage.upper],
        // Выдержка хранится точно, в том числе за пределами диапазона i64.
        "dwell_margin_ns": evidence.margins.dwell_ns.map(|value| value.to_string()),
        "observation": observation, "packet": packet, "packet_open": evidence.packet_open,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    use sim_process::schematic::{
        Finding as ModelFinding, Status,
        emi_reset::{EmiTrial, prepare_emi_reset},
        sim_circuit::netlist,
    };
    use sim_runtime::{
        control::{self, ControlAssets, ControlConfig, RunSpec},
        electrical::{BoardConnection, BoardCurrents, ElectricalSetup, TransientCircuit},
    };
    use sim_signal::Rate;
    use sim_task::Outcome;
    use sim_world::CompiledWorld;

    const CATALOG: &str =
        include_str!("../../../../devices/electrical/corpus/catalog/reference.toml");
    const SCHEME: &str =
        include_str!("../../../../devices/electrical/corpus/pin_overvoltage/fixed.toml");

    // Независимый расчёт блока: Vпокоя 3 В, порог 2 В, помеха ∓0,2 В
    // относительно границы −1 В. Запасы 3 + ΔV − 2 = −0,2 / +0,2 В.
    // Окно [11,22) мкс, выдержка 8 мкс. Если уровень ниже порога все
    // 11 мкс, запас выдержки −3 мкс, иначе +8 мкс.
    fn profile(delta: &str, sourced: bool, end_ns: u64) -> String {
        let source = r#"{ kind = "derived", reference = "синтетический блок" }"#;
        let limit =
            |v: f64| format!("{{ min = {v:e}, nominal = {v:e}, max = {v:e}, source = {source} }}");
        let transfer_source = sourced
            .then(|| format!("source = {source}"))
            .unwrap_or_default();
        format!(
            r#"
format = 1
[instances.block.emi_reset]
source = "sender"
source_kind = "switch"
board = "fixture"
input = "board/io"
reference = "board/g"
binding = {source}
start_ns = 11000
end_ns = 22000
bias_v = {bias}

[instances.block.emi_reset.detector]
falling_v = {falling}
rising_v = {rising}
assert_delay_s = {hold}
release_delay_s = {release}

[instances.block.emi_reset.transfer]
kind = "measured"
{transfer_source}
intervals = [{{ end_ns = {end_ns}, delta_v = [{delta}, {delta}, {delta}] }}]
"#,
            bias = limit(3.0),
            falling = limit(2.0),
            rising = limit(2.5),
            hold = limit(Time(8_000).as_secs()),
            release = limit(Time(3_000).as_secs()),
        )
    }

    fn scheme() -> (SchematicInput, ElectricalEvidence) {
        let path = Path::new("блок.toml");
        let catalog = crate::electrical::parse_catalog(path, CATALOG.as_bytes()).unwrap();
        let scheme = SchematicInput::parse(path, SCHEME.as_bytes().to_vec(), &catalog).unwrap();
        let evidence = ElectricalEvidence::evaluate(
            "block",
            &scheme.bytes,
            &scheme.compiled,
            &scheme.modes,
            Fingerprint::of("синтетический блок".as_bytes()),
        )
        .unwrap();
        assert!(
            evidence
                .findings()
                .iter()
                .all(|item| item.outcome != Outcome::Fail)
        );
        (scheme, evidence)
    }

    struct BlockAssets;

    impl ControlAssets for BlockAssets {
        fn read(&self, name: &str) -> Result<Vec<u8>, String> {
            match name {
                "образ" => {
                    Ok(include_bytes!("../../../sim-boards/testdata/vmcu_ports.elf").to_vec())
                }
                // Формат тестового образа: баннер R, watchdog выключен, ШИМ 1 кГц.
                "данные" => {
                    Ok([vec![b'R', 0, 0, 0], 1000_u32.to_le_bytes().to_vec()].concat())
                }
                _ => Err("неизвестный ресурс блока".into()),
            }
        }

        fn executable(&self, _: &str) -> Result<PathBuf, String> {
            Err("блок использует образ платы".into())
        }
    }

    fn run(trial: EmiTrial, coupled: bool, horizon: Time) -> Runtime {
        // Независимая идеальная шина блока. Дискретизация физики и шины
        // 10 мкс, такт платы 20 мкс. Окно помехи пересекает обе сетки.
        let mut runtime = Runtime::load(CompiledWorld {
            name: "блок".into(),
            seed: 0,
            physics: Rate::hz(100_000).unwrap(),
            mjcf: r#"<mujoco><option timestep="0.00001"/><worldbody/></mujoco>"#.into(),
            bodies: vec![],
            joints: vec![],
            efforts: vec![],
            drives: vec![],
            sensors: vec![],
            simplifications: vec![],
        })
        .unwrap();
        runtime
            .register_observation("постоянный вход блока", "V")
            .unwrap();
        let circuit = netlist::parse("t\nVrail p 0 3.3\nIload p 0 EXT 0\n.end\n").unwrap();
        let step = Time(10_000);
        let source = Provenance::Derived("идеальная шина стенда блока".into());
        let zero = SourcedRange {
            min: 0.0,
            nominal: 0.0,
            max: 0.0,
            provenance: source,
        };
        runtime
            .install_electrical(
                ElectricalSetup {
                    max_step: step,
                    scope: Scope::Reference,
                    sources: vec![],
                    probes: BTreeMap::new(),
                    boards: vec![BoardConnection {
                        board: "fixture".into(),
                        plus: "p".into(),
                        minus: "0".into(),
                        current_source: "iload".into(),
                        reset: trial.input().detector.clone().unwrap(),
                        pins: vec![],
                        initially_in_reset: false,
                        boot_duration_s: Parameter {
                            range: Some(zero.clone()),
                            measurement: "время загрузки блока".into(),
                        },
                        currents: BoardCurrents {
                            unpowered_a: zero.clone(),
                            reset_a: zero.clone(),
                            boot_a: zero.clone(),
                            working_a: zero,
                        },
                    }],
                },
                b"block",
                TransientCircuit::new(circuit, step, vec![]).unwrap(),
            )
            .unwrap();
        runtime.bind_emi_reset(trial, coupled).unwrap();
        let config = ControlConfig::parse(
            r#"
version = 1
[[board]]
name = "fixture"
catalog = "vmcu-c3"
image = "образ"
config = "данные"
rate_hz = 50000
source = "синтетический стенд блока"
[[board_input]]
name = "постоянный вход образа"
board = "fixture"
signal = "постоянный вход блока"
unit = "V"
kind = "adc"
port = 0
max_age_ns = 60000
scale = 1
offset = 0
fallback = 1.65
source = "половина шины 3,3 В для входа тестового образа"
"#,
        )
        .unwrap();
        control::install(
            &mut runtime,
            &config,
            &BlockAssets,
            RunSpec {
                world: b"block",
                catalog: b"stand",
                task: b"emi",
                criteria: b"reset",
                mode: "стенд блока",
                horizon,
                model_ports: &[],
            },
        )
        .unwrap();
        assert_eq!(
            runtime.run_until(horizon).unwrap(),
            sim_runtime::Status::Running
        );
        runtime
    }

    #[test]
    fn журнал_пары_публикует_нарушение_с_меткой_и_контрольным_отпечатком() {
        // В первом опыте 1,8 В удерживается от 11 мкс и достигает выдержки
        // 8 мкс в 19 мкс. Во втором 2,2 В ни разу не опускается ниже 2 В.
        let horizon = Time(60_000);
        for (delta, status, criterion, event_ns, outcome) in [
            ("-1.2", "fail", "violated", Some(19_000), Outcome::Fail),
            (
                "-0.8",
                "not_analyzed",
                "within_bench",
                None,
                Outcome::NotAnalyzed,
            ),
        ] {
            let text = profile(delta, true, 22_000);
            let parsed = EmcProfile::parse(Path::new("блок.toml"), text.as_bytes()).unwrap();
            let (scheme, mut evidence) = scheme();
            let checks = &parsed.instances["block"];
            let (input, _) = checks
                .emi_reset
                .clone()
                .unwrap()
                .resolve(Scope::Reference)
                .unwrap();
            let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(input).unwrap() else {
                panic!("должна быть полная огибающая");
            };
            let active = run(trial.clone(), true, horizon);
            let quiet = run(trial, false, horizon);
            let fingerprint = Fingerprint::of(text.as_bytes());
            let report = evaluate(
                checks,
                fingerprint,
                &scheme,
                &mut evidence,
                Some((&active, &quiet)),
            )
            .unwrap();
            let reset = &report["emi_reset"];
            assert_eq!(reset["status"], status);
            assert_eq!(reset["bench_criterion"], criterion);
            assert_eq!(reset["event_ns"].as_u64(), event_ns);
            assert_eq!(reset["observation"]["reset_ns"].as_u64(), event_ns);
            assert!(reset["observation"]["baseline_reset_ns"].is_null());
            let finding = evidence
                .findings()
                .iter()
                .find(|item| item.code == "emi_reset")
                .unwrap();
            assert_eq!(finding.outcome, outcome);
            assert_eq!(finding.event_ns, event_ns);
            let fingerprint = |runtime| control::run(runtime).unwrap().log().fingerprint();
            assert_eq!(
                finding.journal,
                Some(Fingerprint::from_fnv1a(fingerprint(&active)))
            );
            assert!(
                finding
                    .measurement_sources
                    .iter()
                    .any(|source| source.contains(&format!("{:016x}", fingerprint(&quiet))))
            );

            let (_, mut other_evidence) = self::scheme();
            assert!(
                evaluate(
                    checks,
                    Fingerprint::of(text.as_bytes()),
                    &scheme,
                    &mut other_evidence,
                    Some((&quiet, &active))
                )
                .is_err()
            );
        }
    }

    #[test]
    fn знаковый_запас_в_отчете_без_положительного_исхода() {
        for (delta, voltage, dwell_ns) in [("-1.2", -0.2, "-3000"), ("-0.8", 0.2, "8000")] {
            let text = profile(delta, true, 22_000);
            let parsed = EmcProfile::parse(Path::new("блок.toml"), text.as_bytes()).unwrap();
            let (scheme, mut evidence) = scheme();
            let fingerprint = Fingerprint::of(text.as_bytes());
            let report = evaluate(
                &parsed.instances["block"],
                fingerprint,
                &scheme,
                &mut evidence,
                None,
            )
            .unwrap();
            let reset = &report["emi_reset"];
            // Допуск только на округление разности десятичных напряжений.
            assert!((reset["voltage_margin_v"][1].as_f64().unwrap() - voltage).abs() < 1e-12);
            assert_eq!(reset["dwell_margin_ns"][1], dwell_ns);
            assert_eq!(reset["status"], "not_analyzed");
            assert!(reset["observation"].is_null());
            assert_eq!(reset["required_measurement"], EMI_MEASUREMENT);
            assert_eq!(report["esd_event"]["status"], "not_analyzed");
            assert_eq!(report["esd_event"]["required_measurement"], ESD_MEASUREMENT);
            let finding = evidence
                .findings()
                .iter()
                .find(|item| item.code == "emi_reset")
                .unwrap();
            assert_eq!(finding.outcome, Outcome::NotAnalyzed);
            assert!(!finding.missing_data.is_empty());
            assert_eq!(finding.key.window_ns, Some([11_000, 22_000]));
            assert!(
                evaluate(
                    &parsed.instances["block"],
                    fingerprint,
                    &scheme,
                    &mut evidence,
                    None
                )
                .is_err()
            );
        }
    }

    #[test]
    fn пробелы_и_ошибочные_входы_не_закрывают_проверку() {
        let path = Path::new("блок.toml");
        for text in [
            profile("-1.2", false, 22_000),
            profile("-1.2", true, 21_000),
        ] {
            let parsed = EmcProfile::parse(path, text.as_bytes()).unwrap();
            let (scheme, mut evidence) = scheme();
            let report = evaluate(
                &parsed.instances["block"],
                Fingerprint::of(text.as_bytes()),
                &scheme,
                &mut evidence,
                None,
            )
            .unwrap();
            assert_eq!(report["emi_reset"]["status"], "not_analyzed");
            assert!(report["emi_reset"].get("voltage_margin_v").is_none());
        }
        for value in ["nan", "inf", "-inf"] {
            assert!(EmcProfile::parse(path, profile(value, false, 22_000).as_bytes()).is_err());
        }
        let text = profile("-1.2", true, 22_000).replace("board/io", "board/нет");
        let parsed = EmcProfile::parse(path, text.as_bytes()).unwrap();
        let (scheme, mut evidence) = scheme();
        assert!(
            evaluate(
                &parsed.instances["block"],
                Fingerprint::of(text.as_bytes()),
                &scheme,
                &mut evidence,
                None,
            )
            .unwrap_err()
            .contains("клеммы")
        );
    }

    #[test]
    fn публикация_esd_сохраняет_нарушение_и_оставляет_запас_открытым() {
        for (status, expected) in [
            (Status::Fail, Outcome::Fail),
            (Status::Pass, Outcome::NotAnalyzed),
        ] {
            let (scheme, mut evidence) = scheme();
            // Изолированная проверка перехода между оценщиком и вердиктом.
            let result = evidence
                .record_esd_event(
                    &scheme.compiled,
                    &scheme.modes,
                    "блок",
                    Fingerprint::of("контактный импульс".as_bytes()),
                    |_| {
                        Ok(ModelFinding {
                            subject: "board/io".into(),
                            status,
                            reason: "результат стенда".into(),
                            evidence: (),
                        })
                    },
                )
                .unwrap();
            let finding = evidence.findings().last().unwrap();
            assert_eq!(finding.code, "esd_event");
            assert_eq!(finding.key.class, HazardClass::EsdEvent);
            assert_eq!(finding.outcome, expected);
            assert_eq!(finding.confirmed_hazard, expected == Outcome::Fail);
            assert_ne!(result.status, Status::Pass);
        }
    }
}
