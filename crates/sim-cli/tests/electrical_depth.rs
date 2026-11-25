use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    process::{Command, Output, Stdio},
};

use serde::Deserialize;
use serde_json::Value;
use sim_runtime::{Log, LogKind};
use toml::{Table, Value as Toml};

#[path = "fixtures/depth/dual.rs"]
mod dual;
#[path = "fixtures/depth/linear.rs"]
mod linear;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read(path: &Path) -> Toml {
    let text = fs::read_to_string(path).unwrap_or_else(|err| panic!("{}: {err}", path.display()));
    toml::from_str(&text).unwrap_or_else(|err| panic!("{}: {err}", path.display()))
}

fn write(path: &Path, raw: &Toml) {
    fs::write(path, toml::to_string(raw).unwrap()).unwrap();
}

fn string(value: &Toml) -> &str {
    value.as_str().unwrap()
}

fn number(value: &Toml) -> f64 {
    let result = value
        .as_float()
        .unwrap_or_else(|| value.as_integer().unwrap() as f64);
    assert!(result.is_finite(), "{value}");
    result
}

fn ns(value: &Toml) -> u64 {
    u64::try_from(value.as_integer().unwrap()).unwrap()
}

fn seconds(ns: u64) -> String {
    sim_signal::Time(ns).as_secs().to_string()
}

fn array(value: &Toml) -> &[Toml] {
    value.as_array().unwrap()
}

fn table(value: &Toml) -> &Table {
    value.as_table().unwrap()
}

fn edit(raw: &mut Toml) -> &mut Table {
    raw.as_table_mut().unwrap()
}

struct Scratch(PathBuf);

impl Scratch {
    fn new(label: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("sim-depth-{}-{label}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        Self(dir)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).expect("удаление временных файлов электрического опыта");
    }
}

#[derive(Deserialize)]
struct Installation {
    world: PathBuf,
    profile: PathBuf,
    horizon_ns: u64,
}

#[derive(Deserialize)]
struct Acceptance {
    catalog: PathBuf,
    assemblies: BTreeMap<String, Installation>,
}

struct Case {
    world: PathBuf,
    profile: PathBuf,
    catalog: PathBuf,
    horizon: u64,
}

impl Case {
    fn load(id: &str) -> Self {
        let base = root().join("devices/electrical");
        let manifest: Acceptance = read(&base.join("acceptance.toml")).try_into().unwrap();
        let config = &manifest.assemblies[id];
        Self {
            world: fs::canonicalize(base.join(&config.world)).unwrap(),
            profile: fs::canonicalize(base.join(&config.profile)).unwrap(),
            catalog: fs::canonicalize(base.join(manifest.catalog)).unwrap(),
            horizon: config.horizon_ns,
        }
    }

    fn copy(&self, dest: &Path) -> Self {
        // Для изолированных опытов все используемые локальные файлы находятся в каталоге мира.
        fn local_data(source: &Path, dest: &Path) {
            fs::create_dir(dest).unwrap();
            for entry in fs::read_dir(source).unwrap() {
                let entry = entry.unwrap();
                let path = entry.path();
                let target = dest.join(entry.file_name());
                if entry.file_type().unwrap().is_dir() {
                    local_data(&path, &target);
                } else if path.extension().is_some_and(|ext| ext == "toml") {
                    fs::copy(path, target).unwrap();
                }
            }
        }
        let source = self.world.parent().unwrap();
        local_data(source, dest);
        Self {
            world: dest.join(self.world.strip_prefix(source).unwrap()),
            profile: dest.join(self.profile.strip_prefix(source).unwrap()),
            catalog: self.catalog.clone(),
            horizon: self.horizon,
        }
    }

    fn command(&self, end: u64) -> Command {
        let mut cli = Command::new(env!("CARGO_BIN_EXE_sim-run"));
        cli.arg(&self.world)
            .arg("--electrical")
            .arg(&self.profile)
            .arg("--catalog")
            .arg(&self.catalog)
            .args(["--duration", &seconds(end), "--json"]);
        cli
    }

    fn nominal(&self, log: &Path) -> Value {
        let out = self
            .command(self.horizon)
            .arg("--log")
            .arg(log)
            .output()
            .unwrap();
        self.completed(out)
    }

    fn completed(&self, out: Output) -> Value {
        let report = json(&out);
        assert_eq!(out.status.code(), Some(0), "{report}");
        assert_eq!(report["status"], "completed");
        assert_eq!(report["blocks_proof"], true);
        assert_eq!(report["proof_status"], "not_analyzed");
        let electrical = &report["electrical_verdict"];
        assert_eq!(electrical["simulation"], "NotProven");
        assert_eq!(electrical["live"]["complete"], true);
        assert_eq!(electrical["live"]["required_ns"], self.horizon);
        assert_eq!(electrical["live"]["observed_ns"], self.horizon);
        report
    }

    fn device(&self, owner: &str) -> PathBuf {
        let world = read(&self.world);
        let device = array(&world["devices"])
            .iter()
            .find(|dev| dev["name"].as_str() == Some(owner))
            .unwrap();
        self.world.parent().unwrap().join(string(&device["file"]))
    }

    fn scheme(&self, owner: &str) -> PathBuf {
        let device = self.device(owner);
        device
            .parent()
            .unwrap()
            .join(string(&read(&device)["schematic"]))
    }
}

fn json(output: &Output) -> Value {
    serde_json::from_slice(&output.stdout).unwrap_or_else(|err| {
        panic!(
            "{err}: {}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        )
    })
}

fn rejects_input(out: Output, expected: &str) {
    let report = json(&out);
    assert_eq!(out.status.code(), Some(2), "{report}");
    assert_eq!(report["blocks_proof"], true, "{report}");
    assert!(
        report["detail"].as_str().unwrap().contains(expected),
        "{report}"
    );
}

fn confirms_hazard(out: Output, code: &str, object: Option<&str>) {
    let report = json(&out);
    assert!(matches!(out.status.code(), Some(0 | 1)), "{report}");
    assert_eq!(report["blocks_proof"], true);
    let electrical = &report["electrical_verdict"];
    assert_eq!(electrical["simulation"], "Rejected", "{report}");
    assert!(
        electrical["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|row| row["code"] == code
                && row["outcome"] == "fail"
                && row["confirmed_hazard"] == true
                && object.is_none_or(|name| row["key"]["object"] == name)),
        "{code}: {object:?}"
    );
}

fn steps_from_stdin(mut cli: Command, ticks: u64) -> Output {
    let mut child = cli
        .args(["--mode", "step"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let input = child.stdin.take().unwrap();
    // Шаги подаются из конечного файла. Pipe может закрыться раньше, если CLI вернул ошибку.
    {
        let mut input = input;
        if let Err(err) = input.write_all(&vec![b'\n'; usize::try_from(ticks).unwrap()]) {
            assert_eq!(err.kind(), io::ErrorKind::BrokenPipe);
        }
    }
    child.wait_with_output().unwrap()
}

type Series = BTreeMap<u64, f64>;
type Traces = BTreeMap<String, Series>;

fn traces(path: &Path, owners: &BTreeSet<&str>) -> Traces {
    let log = Log::read_binary(fs::File::open(path).unwrap()).unwrap();
    let mut result = Traces::new();
    for entry in log
        .entries()
        .iter()
        .filter(|entry| entry.kind == LogKind::Truth)
    {
        let name = &log.header().truth[entry.id as usize].name;
        if name
            .split_once('/')
            .is_some_and(|(owner, _)| owners.contains(owner))
        {
            // У механических проб пропуск допустим. Проверяемые электрические ряды ниже
            // сравниваются по ожидаемой сетке либо по явно указанным отсчётам.
            if entry.valid {
                assert!(entry.value.is_finite(), "{name}: {:?}", entry.at);
                assert!(
                    result
                        .entry(name.to_string())
                        .or_default()
                        .insert(entry.at.0, entry.value)
                        .is_none(),
                    "повторный отсчёт {name} в {:?}",
                    entry.at
                );
            }
        }
    }
    result
}

fn remove_joint(scheme: &mut Toml, id: &str) {
    let connections = edit(scheme)
        .get_mut("connections")
        .unwrap()
        .as_table_mut()
        .unwrap();
    assert!(connections.remove(id).is_some(), "{id}");
    for (_, joint) in connections.iter_mut() {
        if let Some(after) = edit(joint).get_mut("after") {
            after
                .as_array_mut()
                .unwrap()
                .retain(|item| string(item) != id);
        }
    }
}

type Graph = BTreeMap<String, BTreeSet<String>>;

fn conductive(scheme: &Toml, parts: &Table) -> Graph {
    let mut graph = Graph::new();
    let mut add = |a: String, b: String| {
        graph.entry(a.clone()).or_default().insert(b.clone());
        graph.entry(b).or_default().insert(a);
    };
    for joint in table(&scheme["connections"]).values() {
        let ends = array(&joint["ends"]);
        add(string(&ends[0]).into(), string(&ends[1]).into());
    }
    for (component, spec) in table(&scheme["components"]) {
        if let Some(elements) = parts[string(&spec["part"])].get("elements") {
            for element in table(elements).values() {
                if element.get("continuity").and_then(Toml::as_bool) == Some(true) {
                    let ends = array(&element["nodes"]);
                    add(
                        format!("{component}/{}", string(&ends[0])),
                        format!("{component}/{}", string(&ends[1])),
                    );
                }
            }
        }
    }
    graph
}

fn reachable<'a>(graph: &'a Graph, start: &'a str, end: &str) -> bool {
    let mut pending = vec![start];
    let mut seen = BTreeSet::new();
    while let Some(next) = pending.pop() {
        if next == end {
            return true;
        }
        if seen.insert(next) {
            pending.extend(graph.get(next).into_iter().flatten().map(String::as_str));
        }
    }
    false
}

fn montaged_parts(catalog: &Path) -> Table {
    let mut parts = Table::new();
    for file in fs::read_dir(catalog.join("electrical")).unwrap() {
        let path = file.unwrap().path();
        if path.extension().is_some_and(|ext| ext == "toml") {
            for (id, part) in table(&read(&path)["parts"]) {
                assert!(parts.insert(id.clone(), part.clone()).is_none(), "{id}");
            }
        }
    }
    parts
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PowerCheck {
    id: String,
    owner: String,
    rail: String,
    return_from: String,
    return_joint: String,
    feed_from: String,
    feed_joint: String,
    probe: String,
    on_ns: u64,
    on_min_v: f64,
    off_ns: Option<u64>,
    off_max_v: Option<f64>,
    restored_ns: Option<u64>,
}

#[test]
fn питание_и_одиночный_обрыв_проверены_по_каждой_схеме() {
    #[derive(Deserialize)]
    struct Checks {
        checks: Vec<PowerCheck>,
    }
    let data: Checks = read(&root().join("devices/electrical/continuity.toml"))
        .try_into()
        .unwrap();
    let mut groups = BTreeMap::<_, Vec<_>>::new();
    for check in &data.checks {
        groups.entry(check.id.as_str()).or_default().push(check);
    }
    assert_eq!(
        groups.keys().copied().collect::<Vec<_>>(),
        ["01", "02", "03", "04", "05", "06", "07"]
    );
    let scratch = Scratch::new("power");
    for (id, checks) in groups {
        let case = Case::load(id);
        let parts = montaged_parts(&case.catalog);
        let catalog = sim_cli::electrical::load_catalog(&case.catalog.join("electrical"))
            .unwrap()
            .catalog;
        let log = scratch.0.join(format!("{id}.simlog"));
        let result = case.nominal(&log);
        let owners = checks
            .iter()
            .map(|c| c.owner.as_str())
            .collect::<BTreeSet<_>>();
        let electrical = &result["electrical_verdict"];
        assert_eq!(owners.len(), checks.len());
        assert_eq!(
            owners,
            electrical["assemblies"]
                .as_object()
                .unwrap()
                .keys()
                .map(String::as_str)
                .collect()
        );
        let traces = traces(&log, &owners);
        let profile = read(&case.profile);
        for check in checks {
            let context = format!("{id}/{}/{}", check.owner, check.rail);
            let path = case.scheme(&check.owner);
            let raw = read(&path);
            let ends = array(&raw["rails"][&check.rail]["ends"]);
            let (plus, minus) = (string(&ends[0]), string(&ends[1]));
            let circuit = &profile["circuits"][&check.owner];
            let declared_ends =
                if let Some(probe) = circuit.get("probes").and_then(|p| p.get(&check.probe)) {
                    assert_eq!(string(&probe["kind"]), "voltage");
                    &probe["ends"]
                } else {
                    let sensor = check.probe.strip_suffix("/питание").unwrap();
                    assert!(circuit["meters"].get(sensor).is_some());
                    let binding = array(&raw["bindings"])
                        .iter()
                        .find(|binding| {
                            string(&binding["target"]["kind"]) == "sensor_power"
                                && binding["target"]["sensor"].as_str() == Some(sensor)
                        })
                        .unwrap();
                    &binding["ends"]
                };
            assert_eq!(array(declared_ends), ends, "{context}: клеммы измерения");
            let graph = conductive(&raw, &parts);
            assert!(
                !reachable(&graph, plus, minus),
                "{context}: плюс и возврат замкнуты"
            );
            let routes = [
                (&check.return_from, minus, &check.return_joint),
                (&check.feed_from, plus, &check.feed_joint),
            ];
            for ((from, to, joint), (other_from, other_to, _)) in
                routes.into_iter().zip(routes.into_iter().rev())
            {
                assert_ne!(from, to);
                assert!(reachable(&graph, from, to), "{context}: {from} → {to}");
                let mut broken = raw.clone();
                remove_joint(&mut broken, joint);
                let changed = conductive(&broken, &parts);
                assert!(
                    !reachable(&changed, from, to),
                    "{context}: отключён {joint}"
                );
                assert!(
                    reachable(&changed, other_from, other_to),
                    "{context}: затронуто другое плечо"
                );
                let altered = sim_cli::electrical::parse_schematic(
                    &path,
                    toml::to_string(&broken).unwrap().as_bytes(),
                    &catalog,
                )
                .unwrap()
                .assembly_sheet();
                let original = &electrical["assemblies"][&check.owner]["sheet"]["operations"];
                assert_eq!(
                    altered.operations.len() + 1,
                    original.as_array().unwrap().len()
                );
                assert!(altered.operations.iter().all(|op| op.id != *joint));
                assert!(altered.inspections.iter().all(|op| op.subject != *joint));
            }
            let series = &traces[&format!("{}/{}", check.owner, check.probe)];
            let on = series[&check.on_ns];
            assert!(check.on_min_v.is_finite() && check.on_min_v > 0.0);
            assert!(on >= check.on_min_v, "{context}: Uвкл = {on} В");
            if let Some(at) = check.off_ns {
                let off = series[&at];
                let limit = check.off_max_v.unwrap();
                assert!(limit.is_finite() && limit > 0.0 && limit <= check.on_min_v);
                assert!(off.abs() < limit, "{context}: Uвыкл = {off} В");
            }
            if let Some(at) = check.restored_ns {
                assert!(
                    series[&at] >= check.on_min_v,
                    "{context}: нет восстановления питания"
                );
            }
            for rail in table(&raw["rails"]).values() {
                let ends = array(&rail["ends"]);
                assert!(
                    !reachable(&graph, string(&ends[0]), string(&ends[1])),
                    "{context}: {rail}"
                );
            }
        }
    }
}

// Допуски и граница сравнения из приёмки Э2, ARCHITECTURE §8.2.
const DC_ABS: f64 = 0.01;
const DC_REL: f64 = 0.01;
const EDGE_REL: f64 = 0.05;
const ROUNDING: f64 = 1e-8; // Округление ручного баланса KCL/KVL.

fn near(actual: f64, reference: f64, tolerance: f64) {
    assert!(
        (actual - reference).abs() <= tolerance,
        "{actual} против {reference}, допуск {tolerance}"
    );
}

fn dc_tolerance(reference: f64) -> f64 {
    DC_ABS.max(DC_REL * reference.abs())
}

fn spice(dir: &Path, elements: &str, analysis: &str, vectors: &str) -> Vec<Vec<f64>> {
    let engine = root().join("assets/third_party/ngspice/bin/ngspice");
    assert!(
        engine.is_file(),
        "NotAnalyzed: отсутствует ngspice в assets/third_party"
    );
    fs::write(dir.join("check.cir"), format!(
        "Independent reference\n{elements}\n.control\nset wr_singlescale\nset numdgt=16\n{analysis}\nwrdata values.txt {vectors}\nquit\n.endc\n.end\n"
    )).unwrap();
    let out = Command::new(engine)
        .current_dir(dir)
        .args(["-n", "-b", "check.cir"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let values = dir.join("values.txt");
    let rows: Vec<Vec<f64>> = fs::read_to_string(&values)
        .unwrap()
        .lines()
        .map(|line| {
            let row = line
                .split_whitespace()
                .map(|part| part.parse::<f64>().unwrap())
                .collect::<Vec<_>>();
            assert_eq!(row.len(), 1 + vectors.split_whitespace().count());
            assert!(row.iter().all(|value| value.is_finite()));
            row
        })
        .collect();
    fs::remove_file(values).unwrap();
    assert!(!rows.is_empty());
    assert!(rows.windows(2).all(|w| w[0][0] < w[1][0]));
    rows
}
