//! Вход платы электрического манифеста. Путь `assemblies.<опыт>.pcb` ведёт
//! к TOML с `format = 1` и таблицами `instances.<экземпляр>`.
//! В каждой таблице указываются `board` и `rules` относительно её файла.
//! Правила формата 1 содержат `schematic_fingerprint` (16 шестнадцатеричных
//! цифр FNV-1a исходных байтов схемы), `bindings` (ref/pad → клемма схемы)
//! и именованные `checks`.
//! Длины в СИ, координаты в целых нанометрах; переходы и зоны нумеруются
//! с нуля в порядке файла. Дорожки адресуются UUID, для KiCad 8 также tstamp.
//! [Sections Header, Setup, Tracks, Zones](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/).

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use serde::Deserialize;
use serde_json::{Value, json};
use sim_process::{
    Analysis,
    schematic::{
        Endpoint, Scope,
        pcb_map::{CopperGraph, CopperSource, PadAddress, PadBinding, check_pcb_map},
        sim_circuit::pcb::{
            clearance, impedance,
            kicad::{self, TrackIds},
            layout::{Layout, Location, Point},
            ratings as rating, return_path as ret,
        },
    },
};
use sim_task::{
    Fingerprint, Outcome,
    verdict::{ElectricalEvidence, HazardClass},
};

use super::{SchematicInput, input::parse};
use crate::tasks::processes::Inputs;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Setup {
    format: u32,
    instances: BTreeMap<String, BoardFiles>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BoardFiles {
    board: PathBuf,
    rules: PathBuf,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Rules {
    format: u32,
    schematic_fingerprint: String,
    bindings: Vec<Binding>,
    #[serde(default)]
    checks: BTreeMap<String, Case>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Binding {
    reference: String,
    pad: String,
    component: String,
    pin: String,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
enum Source {
    Reference(String),
    Verified(String),
    Estimate(String),
}

impl Source {
    fn typed(&self) -> rating::Source<'_> {
        match self {
            Self::Reference(s) => rating::Source::Reference(s),
            Self::Verified(s) => rating::Source::Verified(s),
            Self::Estimate(s) => rating::Source::Estimate(s),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Datum {
    value: f64,
    source: Source,
}

impl Datum {
    fn typed(&self) -> rating::Datum<'_> {
        rating::Datum {
            value: self.value,
            source: self.source.typed(),
        }
    }
}

fn datum(value: &Option<Datum>) -> Option<rating::Datum<'_>> {
    value.as_ref().map(Datum::typed)
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Face {
    Front,
    Back,
}

impl Face {
    fn typed(&self) -> clearance::Face {
        match self {
            Self::Front => clearance::Face::Front,
            Self::Back => clearance::Face::Back,
        }
    }

    fn layer(&self) -> &'static str {
        match self {
            Self::Front => "F.Cu",
            Self::Back => "B.Cu",
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Conditions {
    source: Source,
    material: Option<String>,
    environment: Option<String>,
    working_v: Option<f64>,
    peak_v: Option<f64>,
    frequency_hz: Option<f64>,
    altitude_m: Option<f64>,
    distance_error_m: Option<f64>,
}

impl Conditions {
    fn typed(&self, scope: Scope) -> Option<clearance::Conditions<'_>> {
        let (scope, source) = match (&self.source, scope) {
            (Source::Reference(s), Scope::Reference) => (clearance::Scope::Reference, s),
            (Source::Verified(s), Scope::Assembly) => (clearance::Scope::Sourced, s),
            (Source::Verified(s), Scope::Reference) => (clearance::Scope::Reference, s),
            _ => return None,
        };
        Some(clearance::Conditions {
            scope,
            source,
            material: self.material.as_deref()?,
            environment: self.environment.as_deref()?,
            working_v: self.working_v?,
            peak_v: self.peak_v?,
            frequency_hz: self.frequency_hz?,
            altitude_m: self.altitude_m?,
            distance_error_m: self.distance_error_m,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Limits {
    envelope: Conditions,
    clearance_m: Option<f64>,
    creepage_m: Option<f64>,
    slot_credit: Option<SlotCredit>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum SlotCredit {
    Rectangular { minimum_width_m: f64 },
    Verified { source: String },
}

impl Limits {
    fn typed(&self, scope: Scope) -> Option<clearance::Limits<'_>> {
        Some(clearance::Limits {
            envelope: self.envelope.typed(scope)?,
            clearance_m: self.clearance_m,
            creepage_m: self.creepage_m,
            slot_credit: self.slot_credit.as_ref().map(|credit| match credit {
                SlotCredit::Rectangular { minimum_width_m } => clearance::SlotCredit::Rectangular {
                    minimum_width_m: *minimum_width_m,
                },
                SlotCredit::Verified { source } => clearance::SlotCredit::Verified { source },
            }),
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Node {
    at_nm: [i32; 2],
    layer: String,
}

fn point([x_nm, y_nm]: [i32; 2]) -> Point {
    Point { x_nm, y_nm }
}

fn position(p: Point) -> [i32; 2] {
    [p.x_nm, p.y_nm]
}

impl Node {
    fn typed(&self) -> rating::Node<'_> {
        rating::Node {
            at: point(self.at_nm),
            layer: &self.layer,
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Load {
    node: Node,
    current_a: Option<Datum>,
    max_drop_v: Option<Datum>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ViaLimit {
    via: usize,
    plating_m: Option<Datum>,
    max_current_a: Option<Datum>,
    min_ring_m: Option<Datum>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Copper {
    rho_20_ohm_m: Datum,
    alpha_per_k: Datum,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Ratings {
    net: String,
    reference: Node,
    #[serde(default)]
    loads: Vec<Load>,
    topology: Option<Source>,
    copper: Option<Copper>,
    copper_temp_c: Option<Datum>,
    ambient_c: Option<Datum>,
    max_rise_k: Option<Datum>,
    max_laminate_c: Option<Datum>,
    thermal_model: Option<Source>,
    #[serde(default)]
    vias: Vec<ViaLimit>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Plane {
    zone: usize,
    layer: String,
    tie_nm: [i32; 2],
    connection: Option<Source>,
}

impl Plane {
    fn typed(&self) -> ret::Plane<'_> {
        ret::Plane {
            zone: self.zone,
            layer: &self.layer,
            tie: point(self.tie_nm),
            connection: self.connection.as_ref().map(Source::typed),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    track: String,
    plane: Plane,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Step {
    Track { track: String, to_nm: [i32; 2] },
    Via { via: usize, to_layer: String },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Route {
    net: String,
    start: Node,
    steps: Vec<Step>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Terminal {
    reference: String,
    pad: String,
    layer: String,
}

impl Terminal {
    fn typed(&self) -> ret::Terminal<'_> {
        ret::Terminal {
            reference: &self.reference,
            pad: &self.pad,
            layer: &self.layer,
        }
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Transfer {
    StitchVia {
        via: usize,
    },
    Capacitor {
        arms: [Route; 2],
        pads: [Terminal; 2],
        coupling: Option<Source>,
    },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Plating {
    via: usize,
    thickness_m: Datum,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ProcessWindow {
    width_m: [f64; 2],
    dielectric_height_m: [f64; 2],
    trace_copper_m: [f64; 2],
    epsilon_r: [f64; 2],
    band_hz: [f64; 2],
    source: Source,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ImpedanceLimit {
    ohms: [f64; 2],
    source: Source,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Case {
    Clearance {
        face: Face,
        nets: [String; 2],
        conditions: Option<Conditions>,
        limits: Option<Limits>,
    },
    Ratings(Box<Ratings>),
    Reference(Reference),
    Transition {
        tracks: [String; 2],
        planes: [Plane; 2],
        signal_via: usize,
        transfer: Option<Transfer>,
        max_detour_m: Option<Datum>,
    },
    Decoupling {
        supply: Route,
        return_arm: Route,
        load: [Terminal; 2],
        capacitor: [Terminal; 2],
        topology: Option<Source>,
        max_perimeter_m: Option<Datum>,
    },
    GroundBounce {
        power: Route,
        signal: Route,
        topology: Option<Source>,
        resistivity_ohm_m: Option<Datum>,
        #[serde(default)]
        plating: Vec<Plating>,
        shared_inductance_h: Option<Datum>,
        current_a: Option<Datum>,
        slew_a_per_s: Option<Datum>,
        max_bounce_v: Option<Datum>,
        omitted_bounce_v: Option<Datum>,
    },
    Impedance {
        reference: Reference,
        process: Option<ProcessWindow>,
        limit: Option<ImpedanceLimit>,
        model_error_ohm: Option<Datum>,
    },
}

pub(super) struct BoardInput {
    path: PathBuf,
    profile: Fingerprint,
    layout: Layout,
    connectivity: Analysis<CopperGraph>,
    ids: TrackIds,
    rules: Rules,
}

pub(super) fn load(
    path: &Path,
    files: &mut Inputs,
    schemes: &BTreeMap<String, SchematicInput>,
) -> Result<BTreeMap<String, BoardInput>, String> {
    let bytes = files.capture(path)?;
    let setup: Setup = parse(path, bytes)?;
    let setup_stamp = Fingerprint::of(bytes);
    if setup.format != 1 || setup.instances.is_empty() {
        return Err(format!(
            "{}: нужен непустой профиль платы формата 1",
            path.display()
        ));
    }
    let root = super::report::dir(path);
    setup
        .instances
        .into_iter()
        .map(|(instance, paths)| {
            let scheme = schemes
                .get(&instance)
                .ok_or_else(|| format!("{}: нет схемы экземпляра «{instance}»", path.display()))?;
            let path = root.join(paths.board);
            if path.extension().and_then(|ext| ext.to_str()) != Some("kicad_pcb") {
                return Err(format!("{}: ожидается файл .kicad_pcb", path.display()));
            }
            let bytes = files.capture(&path)?;
            let text =
                std::str::from_utf8(bytes).map_err(|e| format!("{}: {e}", path.display()))?;
            let (board, ids) = kicad::parse_with_track_ids(text)
                .map_err(|e| format!("{}: {e}", path.display()))?;
            let board_stamp = Fingerprint::of(bytes);
            let rule_path = root.join(paths.rules);
            let bytes = files.capture(&rule_path)?;
            let rules: Rules = parse(&rule_path, bytes)?;
            if rules.format != 1
                || rules.schematic_fingerprint.len() != 16
                || u64::from_str_radix(&rules.schematic_fingerprint, 16).ok()
                    != Some(Fingerprint::of(&scheme.bytes).as_u64())
            {
                return Err(format!(
                    "{}: версия правил или отпечаток схемы не совпадает",
                    rule_path.display()
                ));
            }
            let rule_stamp = Fingerprint::of(bytes);
            let bindings = rules
                .bindings
                .iter()
                .map(|b| PadBinding {
                    address: PadAddress {
                        reference: b.reference.clone(),
                        pad: b.pad.clone(),
                    },
                    endpoint: Endpoint {
                        component: b.component.clone(),
                        pin: b.pin.clone(),
                    },
                })
                .collect::<Vec<_>>();
            let layout =
                Layout::from_kicad(&board).map_err(|e| format!("{}: {e}", path.display()))?;
            let connectivity = check_pcb_map(&board, &scheme.compiled, &bindings)
                .map_err(|e| format!("{}: {e}", path.display()))?;
            let profile = Fingerprint::of(
                &[
                    setup_stamp.as_u64().to_le_bytes(),
                    board_stamp.as_u64().to_le_bytes(),
                    rule_stamp.as_u64().to_le_bytes(),
                ]
                .concat(),
            );
            Ok((
                instance,
                BoardInput {
                    path,
                    profile,
                    layout,
                    connectivity,
                    ids,
                    rules,
                },
            ))
        })
        .collect()
}

impl BoardInput {
    pub(super) fn evaluate(
        &self,
        evidence: &mut ElectricalEvidence,
    ) -> Result<(Outcome, Value), String> {
        self.evaluate_inner(evidence)
            .map_err(|e| format!("{}: {e}", self.path.display()))
    }

    fn evaluate_inner(
        &self,
        evidence: &mut ElectricalEvidence,
    ) -> Result<(Outcome, Value), String> {
        let checks = Checks {
            layout: &self.layout,
            ids: &self.ids,
            scope: evidence.assessment().scope,
            profile: self.profile,
            copper: match &self.connectivity {
                Analysis::Analyzed { value, .. } => Some(value),
                Analysis::NotAnalyzed { .. } => None,
            },
        };
        let mut results = BTreeMap::new();
        let mut outcomes = Vec::new();
        let mut present = BTreeSet::new();
        let mut pairs = BTreeSet::new();
        let mut gaps = BTreeMap::new();
        let (connectivity, gap) = match &self.connectivity {
            Analysis::Analyzed { scope, value } => (
                json!({
                    "status": Outcome::Pass, "scope": scope, "pieces": value.pieces.len(),
                    "contacts": value.contacts.len(), "groups": value.groups.len(),
                    "terminals": value.terminals.len(),
                }),
                None,
            ),
            Analysis::NotAnalyzed { reason } => (
                json!({ "status": Outcome::NotAnalyzed, "reason": reason }),
                Some(reason.as_str()),
            ),
        };
        for (id, case) in &self.rules.checks {
            if id.trim().is_empty()
                || id.starts_with('@')
                || id.contains('/')
                || id.chars().any(char::is_control)
            {
                return Err(format!("некорректное имя проверки платы «{id}»"));
            }
            if let Case::Clearance { face, nets, .. } = case {
                let mut nets = [checks.net(&nets[0])?, checks.net(&nets[1])?];
                nets.sort();
                pairs.insert((face.layer(), nets));
            }
            let (status, mut value) = checks
                .evaluate_case(evidence, id, case, gap)
                .map_err(|e| format!("проверка «{id}»: {e}"))?;
            present.insert(value["kind"].as_str().expect("вид проверки").to_owned());
            if let Some(reason) = gap.filter(|_| status == Outcome::Pass) {
                value["status"] = json!(Outcome::NotAnalyzed);
                value["reason"] = json!(reason);
                outcomes.push(Outcome::NotAnalyzed);
            } else {
                outcomes.push(status);
            }
            results.insert(id, value);
        }
        for (kind, reason) in [
            (
                "ratings",
                "не заданы расчётные токи и технологические пределы",
            ),
            ("reference", "не заданы трассы и их опорные плоскости"),
            (
                "transition",
                "не заданы переходы между опорными плоскостями",
            ),
            ("decoupling", "не заданы контуры и пределы развязки"),
            ("impedance", "не заданы трассы с нормированным импедансом"),
        ] {
            if !present.contains(kind) {
                gaps.insert(kind, reason);
            }
        }
        let uncovered = checks.copper.map_or_else(Vec::new, |graph| {
            uncovered_pairs(&self.layout, graph, &pairs)
        });
        for (class, reason) in [
            (
                HazardClass::CreepageClearance,
                gap.or({
                    if !present.contains("creepage_clearance") {
                        Some("не заданы пары и пределы изоляции")
                    } else if !uncovered.is_empty() {
                        Some("не заданы пределы для всех пар на наружных слоях")
                    } else if checks.copper.is_some_and(has_interlayer_pairs) {
                        Some("изоляция между слоями, через торцы и сверления остаётся открытой")
                    } else {
                        None
                    }
                }),
            ),
            (
                HazardClass::GroundBounce,
                gap.or((!present.contains("ground_bounce"))
                    .then_some("не заданы маршруты, ток, фронт и предел смещения опоры")),
            ),
        ] {
            if let Some(reason) = reason {
                evidence
                    .record_pcb_open(class, "@coverage", self.profile, reason)
                    .map_err(|e| e.to_string())?;
                gaps.insert(class.spec().id, reason);
            }
        }
        if !gaps.is_empty() {
            outcomes.push(Outcome::NotAnalyzed);
        }
        let status = Outcome::aggregate(outcomes);
        Ok((
            status,
            json!({
                "status": status, "fingerprint": self.profile, "schematic": self.rules.schematic_fingerprint,
                "kicad_version": self.layout.format_version, "generator_version": self.layout.generator_version,
                "coordinate_unit": "nm", "copper_layers": self.layout.copper_layers,
                "connectivity": connectivity, "checks": results, "open": gaps,
                "clearance_uncovered": uncovered,
            }),
        ))
    }
}

fn uncovered_pairs(
    layout: &Layout,
    graph: &CopperGraph,
    provided: &BTreeSet<(&str, [u32; 2])>,
) -> Vec<Value> {
    let mut uncovered = Vec::new();
    for layer in ["F.Cu", "B.Cu"] {
        let nets = graph
            .pieces
            .iter()
            .filter(|piece| piece.layer == layer)
            .filter_map(|piece| piece.net)
            .collect::<BTreeSet<_>>();
        for (i, &a) in nets.iter().enumerate() {
            for &b in nets.iter().skip(i + 1) {
                if !provided.contains(&(layer, [a, b])) {
                    uncovered.push(
                        json!({ "layer": layer, "nets": [&layout.nets[&a], &layout.nets[&b]] }),
                    );
                }
            }
        }
    }
    uncovered
}

fn has_interlayer_pairs(graph: &CopperGraph) -> bool {
    let pieces = || graph.pieces.iter().filter_map(|p| Some((&p.layer, p.net?)));
    let Some((first_layer, first_net)) = pieces().next() else {
        return false;
    };
    pieces().any(|(layer, _)| layer != first_layer) && pieces().any(|(_, net)| net != first_net)
}

struct Checks<'a> {
    layout: &'a Layout,
    ids: &'a TrackIds,
    scope: Scope,
    profile: Fingerprint,
    copper: Option<&'a CopperGraph>,
}

impl Checks<'_> {
    fn track(&self, uuid: &str) -> Result<usize, String> {
        self.ids
            .index_of(uuid)
            .ok_or_else(|| format!("нет дорожки «{uuid}»"))
    }

    fn net(&self, name: &str) -> Result<u32, String> {
        self.layout
            .nets
            .iter()
            .find_map(|(&net, n)| (n == name).then_some(net))
            .ok_or_else(|| format!("нет цепи «{name}»"))
    }

    fn scope(&self) -> rating::Scope {
        match self.scope {
            Scope::Reference => rating::Scope::Reference,
            Scope::Assembly => rating::Scope::Assembly,
        }
    }

    fn reference<'a>(&self, reference: &'a Reference) -> Result<ret::ReferenceInput<'a>, String> {
        Ok(ret::ReferenceInput {
            scope: self.scope(),
            track: self.track(&reference.track)?,
            plane: self.plane(&reference.plane)?,
        })
    }

    fn plane<'a>(&self, input: &'a Plane) -> Result<ret::Plane<'a>, String> {
        let mut plane = input.typed();
        if let Some(Source::Reference(label) | Source::Verified(label) | Source::Estimate(label)) =
            &input.connection
            && label.trim().is_empty()
        {
            return Err("пустой источник привязки опорной плоскости".into());
        }
        // Имя цепи и точка внутри полигона сами по себе не связывают
        // заливку с клеммой схемы.
        let attached = self.copper.is_some_and(|graph| {
            self.layout
                .zones
                .get(plane.zone)
                .and_then(|zone| zone.fill(plane.layer).ok())
                .is_some_and(|regions| {
                    graph.groups.iter().any(|group| {
                        group.iter().any(|&i| {
                            let piece = &graph.pieces[i];
                            piece.layer == plane.layer
                                && matches!(piece.source, CopperSource::Fill { zone, region }
                                    if zone == plane.zone
                                        && regions[region].polygon.locate(plane.tie) == Location::Inside)
                        }) && graph.terminals.values().any(|terminal| {
                            terminal.pieces.iter().any(|i| group.contains(i))
                        })
                    })
                })
        });
        if !attached {
            plane.connection = None;
        }
        Ok(plane)
    }

    fn route<'a>(&self, route: &'a Route) -> Result<(u32, Vec<ret::Step<'a>>), String> {
        let steps = route
            .steps
            .iter()
            .map(|step| {
                Ok(match step {
                    Step::Track { track, to_nm } => ret::Step::Track {
                        index: self.track(track)?,
                        to: point(*to_nm),
                    },
                    Step::Via { via, to_layer } => ret::Step::Via {
                        index: *via,
                        to_layer,
                    },
                })
            })
            .collect::<Result<_, String>>()?;
        Ok((self.net(&route.net)?, steps))
    }
}

fn route<'a>(raw: &'a Route, resolved: &'a (u32, Vec<ret::Step<'a>>)) -> ret::Route<'a> {
    ret::Route {
        net: resolved.0,
        start: raw.start.typed(),
        steps: &resolved.1,
    }
}

fn distance(value: Option<clearance::Distance>) -> Value {
    json!(value.map(|d| [Some(d.lower_m), d.upper_m]))
}

fn open_status(reason: impl ToString) -> (Outcome, Option<String>) {
    (Outcome::NotAnalyzed, Some(reason.to_string()))
}

fn clearance_status(status: clearance::Status) -> Outcome {
    match status {
        clearance::Status::Pass => Outcome::Pass,
        clearance::Status::Fail => Outcome::Fail,
        clearance::Status::NotAnalyzed => Outcome::NotAnalyzed,
    }
}

fn ret_status(status: &ret::Status) -> (Outcome, Option<String>) {
    match status {
        ret::Status::Pass => (Outcome::Pass, None),
        ret::Status::Fail => (Outcome::Fail, None),
        ret::Status::NotAnalyzed(reason) => open_status(reason),
    }
}

fn rating_status(status: &rating::Status) -> (Outcome, Option<String>) {
    match status {
        rating::Status::Pass => (Outcome::Pass, None),
        rating::Status::Fail => (Outcome::Fail, None),
        rating::Status::NotAnalyzed(reason) => open_status(reason),
    }
}

fn rating_check(check: &rating::Check) -> Value {
    let (status, reason) = rating_status(&check.status);
    json!({ "status": status, "reason": reason, "value": check.value, "limit": check.limit })
}

fn clearance_check(check: &clearance::Check) -> Value {
    json!({
        "status": clearance_status(check.status),
        "distance_m": distance(check.distance), "minimum_m": check.minimum_m,
        "open": check.open.iter().map(ToString::to_string).collect::<Vec<_>>(),
    })
}

impl Checks<'_> {
    fn evaluate_case(
        &self,
        evidence: &mut ElectricalEvidence,
        id: &str,
        case: &Case,
        topology_gap: Option<&str>,
    ) -> Result<(Outcome, Value), String> {
        let text = |e: &dyn std::fmt::Display| e.to_string();
        let (kind, mut status, mut reason, metrics) = match case {
            Case::Clearance {
                face,
                nets,
                conditions,
                limits,
            } => {
                let conditions = conditions.as_ref().and_then(|c| c.typed(self.scope));
                let limits = limits.as_ref().and_then(|l| l.typed(self.scope));
                let report = evidence
                    .record_pcb_clearance(
                        id,
                        self.profile,
                        self.layout,
                        face.typed(),
                        [self.net(&nets[0])?, self.net(&nets[1])?],
                        conditions.as_ref(),
                        limits.as_ref(),
                        topology_gap,
                    )
                    .map_err(|e| text(&e))?;
                let finding = evidence
                    .findings()
                    .last()
                    .expect("опубликован результат зазора");
                (
                    "creepage_clearance",
                    finding.outcome,
                    Some(finding.reason.clone()),
                    json!({
                        "nets": nets, "layer": face.layer(),
                        "clearance": clearance_check(&report.clearance), "creepage": clearance_check(&report.creepage),
                        "geometric_creepage_m": distance(report.geometric_creepage),
                    }),
                )
            }
            Case::Ratings(input) => {
                let (status, reason, metrics) = self.ratings(input)?;
                ("ratings", status, reason, metrics)
            }
            Case::Reference(reference) => {
                let report = ret::check_reference(self.layout, &self.reference(reference)?)
                    .map_err(|e| text(&e))?;
                let (status, reason) = ret_status(&report.status);
                (
                    "reference",
                    status,
                    reason,
                    json!({
                        "track": reference.track, "height_m": report.height_m,
                        "center_gaps_m": report.center_gaps_m, "copper_gaps_m": report.copper_gaps_m,
                    }),
                )
            }
            Case::Transition {
                tracks,
                planes,
                signal_via,
                transfer,
                max_detour_m,
            } => {
                let arms;
                let transfer = match transfer {
                    Some(Transfer::StitchVia { via }) => Some(ret::Transfer::StitchVia(*via)),
                    Some(Transfer::Capacitor {
                        arms: raw,
                        pads,
                        coupling,
                    }) => {
                        arms = [self.route(&raw[0])?, self.route(&raw[1])?];
                        Some(ret::Transfer::Capacitor {
                            arms: [route(&raw[0], &arms[0]), route(&raw[1], &arms[1])],
                            pads: [pads[0].typed(), pads[1].typed()],
                            coupling: coupling.as_ref().map(Source::typed),
                        })
                    }
                    None => None,
                };
                let report = ret::check_transition(
                    self.layout,
                    &ret::TransitionInput {
                        scope: self.scope(),
                        tracks: [self.track(&tracks[0])?, self.track(&tracks[1])?],
                        planes: [self.plane(&planes[0])?, self.plane(&planes[1])?],
                        signal_via: *signal_via,
                        transfer,
                        max_detour_m: datum(max_detour_m),
                    },
                )
                .map_err(|e| text(&e))?;
                let (status, reason) = ret_status(&report.status);
                (
                    "transition",
                    status,
                    reason,
                    json!({ "detour_m": report.detour_m }),
                )
            }
            Case::Decoupling {
                supply,
                return_arm,
                load,
                capacitor,
                topology,
                max_perimeter_m,
            } => {
                let report = ret::check_decoupling(
                    self.layout,
                    &ret::DecouplingInput {
                        scope: self.scope(),
                        supply: route(supply, &self.route(supply)?),
                        return_arm: route(return_arm, &self.route(return_arm)?),
                        load: [load[0].typed(), load[1].typed()],
                        capacitor: [capacitor[0].typed(), capacitor[1].typed()],
                        topology: topology.as_ref().map(Source::typed),
                        max_perimeter_m: datum(max_perimeter_m),
                    },
                )
                .map_err(|e| text(&e))?;
                let (status, reason) = ret_status(&report.status);
                (
                    "decoupling",
                    status,
                    reason,
                    json!({
                        "arm_lengths_m": report.arm_lengths_m, "perimeter_m": report.perimeter_m,
                    }),
                )
            }
            Case::GroundBounce {
                power,
                signal,
                topology,
                resistivity_ohm_m,
                plating,
                shared_inductance_h,
                current_a,
                slew_a_per_s,
                max_bounce_v,
                omitted_bounce_v,
            } => {
                let plating = plating
                    .iter()
                    .map(|row| (row.via, row.thickness_m.typed()))
                    .collect::<Vec<_>>();
                let report = evidence
                    .record_pcb_ground_bounce(
                        id,
                        self.profile,
                        self.layout,
                        &ret::GroundBounceInput {
                            scope: self.scope(),
                            power: route(power, &self.route(power)?),
                            signal: route(signal, &self.route(signal)?),
                            topology: topology.as_ref().map(Source::typed),
                            resistivity_ohm_m: datum(resistivity_ohm_m),
                            plating_m: &plating,
                            shared_inductance_h: datum(shared_inductance_h),
                            current_a: datum(current_a),
                            slew_a_per_s: datum(slew_a_per_s),
                            max_bounce_v: datum(max_bounce_v),
                            omitted_bounce_v: datum(omitted_bounce_v),
                        },
                        topology_gap,
                    )
                    .map_err(|e| text(&e))?;
                let finding = evidence
                    .findings()
                    .last()
                    .expect("опубликован результат смещения опоры");
                (
                    "ground_bounce",
                    finding.outcome,
                    Some(finding.reason.clone()),
                    json!({
                        "shared_length_m": report.shared_length_m, "resistance_ohm": report.resistance_ohm,
                        "inductance_h": report.inductance_h, "bounce_v": report.bounce_v, "bounds_v": report.bounds_v,
                        "max_bounce_v": max_bounce_v.as_ref().map(|d| d.value),
                    }),
                )
            }
            Case::Impedance {
                reference,
                process,
                limit,
                model_error_ohm,
            } => {
                let report = impedance::evaluate(
                    self.layout,
                    &impedance::Mark {
                        reference: self.reference(reference)?,
                        process: process.as_ref().map(|p| impedance::ProcessWindow {
                            width_m: p.width_m,
                            dielectric_height_m: p.dielectric_height_m,
                            trace_copper_m: p.trace_copper_m,
                            epsilon_r: p.epsilon_r,
                            band_hz: p.band_hz,
                            source: p.source.typed(),
                        }),
                        limit: limit.as_ref().map(|l| impedance::Limit {
                            ohms: l.ohms,
                            source: l.source.typed(),
                        }),
                        model_error_ohm: datum(model_error_ohm),
                    },
                )
                .map_err(|e| text(&e))?;
                let (status, reason) = match &report.status {
                    impedance::Status::Pass => (Outcome::Pass, None),
                    impedance::Status::Fail => (Outcome::Fail, None),
                    impedance::Status::NotAnalyzed(reason) => open_status(reason),
                };
                (
                    "impedance",
                    status,
                    reason,
                    json!({
                        "track": reference.track, "nominal_ohm": report.nominal_ohm,
                        "process_range_ohm": report.process_range_ohm, "comparison_range_ohm": report.comparison_range_ohm,
                    }),
                )
            }
        };
        if status == Outcome::Pass && self.scope == Scope::Assembly {
            (status, reason) = open_status("нужна натурная сверка для ревизии и условий установки");
        }
        Ok((
            status,
            json!({ "kind": kind, "status": status, "reason": reason, "metrics": metrics }),
        ))
    }

    fn ratings(&self, input: &Ratings) -> Result<(Outcome, Option<String>, Value), String> {
        let loads = input
            .loads
            .iter()
            .map(|load| rating::Load {
                node: load.node.typed(),
                current_a: datum(&load.current_a),
                max_drop_v: datum(&load.max_drop_v),
            })
            .collect::<Vec<_>>();
        let vias = input
            .vias
            .iter()
            .map(|via| rating::ViaLimit {
                via: via.via,
                plating_m: datum(&via.plating_m),
                max_current_a: datum(&via.max_current_a),
                min_ring_m: datum(&via.min_ring_m),
            })
            .collect::<Vec<_>>();
        let report = rating::evaluate(
            self.layout,
            &rating::Input {
                scope: self.scope(),
                net: self.net(&input.net)?,
                reference: input.reference.typed(),
                loads: &loads,
                topology: input.topology.as_ref().map(Source::typed),
                copper: input.copper.as_ref().map(|c| rating::Copper {
                    rho_20_ohm_m: c.rho_20_ohm_m.typed(),
                    alpha_per_k: c.alpha_per_k.typed(),
                }),
                copper_temp_c: datum(&input.copper_temp_c),
                ambient_c: datum(&input.ambient_c),
                max_rise_k: datum(&input.max_rise_k),
                max_laminate_c: datum(&input.max_laminate_c),
                thermal_model: input.thermal_model.as_ref().map(Source::typed),
                vias: &vias,
            },
        )
        .map_err(|e| e.to_string())?;
        let (status, reason) = rating_status(&report.status());
        Ok((
            status,
            reason,
            json!({
                "net": input.net, "open": report.network_open.iter().map(ToString::to_string).collect::<Vec<_>>(),
                "loads": report.loads.iter().map(|load| json!({
                    "at_nm": position(load.node.at), "layer": load.node.layer,
                    "drop_v": rating_check(&load.drop),
                })).collect::<Vec<_>>(),
                "vias": report.vias.iter().map(|via| json!({
                    "via": via.via, "ring_m": rating_check(&via.ring),
                })).collect::<Vec<_>>(),
                "branches": report.branches.iter().map(|branch| {
                    let section = match branch.section {
                        rating::Section::Track(index) => json!({ "track": index }),
                        rating::Section::Via(index) => json!({ "via": index }),
                    };
                    json!({
                        "section": section,
                        "ends": branch.ends.map(|node| json!({ "at_nm": position(node.at), "layer": node.layer })),
                        "length_m": branch.length_m, "resistance_ohm": branch.resistance_ohm,
                        "open": branch.open.as_ref().map(ToString::to_string),
                        "current_a": branch.current_a, "drop_v": branch.drop_v, "power_w": branch.power_w,
                        "ipc_rise_k": branch.ipc_rise_k, "temperature_c": branch.temperature_c.as_ref().map(rating_check),
                        "current_limit_a": rating_check(&branch.current_limit),
                    })
                }).collect::<Vec<_>>(),
            }),
        ))
    }
}
