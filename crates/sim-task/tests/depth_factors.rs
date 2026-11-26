use std::{collections::BTreeMap, fs, path::PathBuf};

use sim_task::{
    Fault, FaultKind, Fingerprint, Source, SourceKind,
    campaign::{
        Campaign, FaultSupport, Grid, RequiredPath, ResolvedInputs, ResolvedReference, RunKey,
        WorldInputs,
        depth_factors::{DepthFactors, DepthLevels, DepthRow, Selection},
    },
};

const SOURCE: &str = r#"{ kind = "derived", reference = "синтетический стенд" }"#;

const DEPTH: &str = r#"
[depth_factors.factors.operation]
safety = true
source = { kind = "derived", reference = "таблица режимов стенда" }
levels = { kind = "mode", values = { idle = { station = "bench", mode = "idle" }, moving = { station = "bench", mode = "motion" } } }

[depth_factors.factors.route]
safety = false
source = { kind = "derived", reference = "таблица маршрутов стенда" }
levels = { kind = "path", values = { primary = "line", secondary = "reserve" } }

[depth_factors.factors.clock]
safety = false
source = { kind = "derived", reference = "сетки стенда" }
levels = { kind = "grid", values = { base = "base", fine = "fine" } }

[depth_factors.factors.cut]
safety = true
source = { kind = "derived", reference = "матрица инъекций стенда" }
levels = { kind = "faults", values = { intact = [], open = [{ node = "a", kind = "wire_open" }], late = [{ node = "c", kind = "wire_open" }] } }

[depth_factors.factors.mass]
safety = true
source = { kind = "measured", reference = "синтетические гири 1 и 2 кг" }
levels = { kind = "axis", parameter = { kind = "mass", body = "block/rod" }, unit = "kg", nominal = 2.0, values = { light = 1.0, nominal = 2.0 } }

[depth_factors.factors.wind]
safety = false
source = { kind = "derived", reference = "вход стенда 0 и 3 м/с" }
levels = { kind = "axis", parameter = { kind = "wind", axis = 0 }, unit = "m/s", nominal = 0.0, values = { still = 0.0, gust = 3.0 } }

[depth_factors.constraints.reserve_locked]
when = { operation = "idle", route = "secondary" }
reason = "в покое запасной маршрут механически заблокирован"
source = { kind = "measured", reference = "синтетическая таблица блокировки стенда" }
"#;

struct Files(PathBuf);

impl Files {
    fn with_depth(id: &str, depth: &str) -> Self {
        Self::with_schedule(id, depth, "")
    }

    fn with_schedule(id: &str, depth: &str, extra_fault: &str) -> Self {
        let path = std::env::temp_dir().join(format!("sim-task-depth-{}-{id}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let files = Self(path);
        files.write("catalog.toml", "version = 1");
        files.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 1
physics_hz = 1000
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "block"
file = "rod.toml"
"#,
        );
        files.write(
            "rod.toml",
            r#"
format = 3
name = "Стержень"
[[links]]
name = "rod"
[links.joint]
kind = "fixed"
[links.mass]
kind = "explicit"
mass_kg = 2.0
com_m = [0.0, 0.0, 0.0]
inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0]
provenance = { measured = "синтетические масса и инерция стержня" }
"#,
        );
        // Сетка 1 кГц, инъекция на 1 мс, две стороны окна [0; 2] мс.
        files.write(
            "campaign.toml",
            &format!(
                r#"version = 1
seed = 37
world = "world.toml"
catalog = "catalog.toml"

[stations.bench]
experiment = {{ kind = "scenario" }}
[stations.bench.modes.motion]
horizon_ns = 2000000
source = {SOURCE}
faults = [{{ fault = {{ node = "a", kind = "wire_open" }}, at_ns = 1000000, max_lag_ns = 0, window = "whole", source = {SOURCE} }}{extra_fault}]
criteria = [{{ id = "travel", window = "whole", limit = {{ kind = "range", signal = {{ name = "travel", unit = "m" }}, min = 0.0, max = 1.0, source = {SOURCE} }} }}]
[stations.bench.modes.motion.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
[stations.bench.modes.idle]
horizon_ns = 2000000
source = {SOURCE}
criteria = [{{ id = "travel", window = "whole", limit = {{ kind = "range", signal = {{ name = "travel", unit = "m" }}, min = 0.0, max = 1.0, source = {SOURCE} }} }}]
[stations.bench.modes.idle.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
{depth}"#
            ),
        );
        files
    }

    fn write(&self, file: &str, text: &str) {
        fs::write(self.0.join(file), text).unwrap();
    }

    fn load(&self) -> Result<Campaign, sim_task::Error> {
        Campaign::load(self.0.join("campaign.toml"))
    }
}

impl Drop for Files {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn fault(node: &str) -> Fault {
    Fault {
        node: node.into(),
        kind: FaultKind::WireOpen,
    }
}

fn inventory(campaign: &Campaign) -> ResolvedInputs {
    let source = Source {
        kind: SourceKind::Derived,
        reference: "измерения синтетического стенда".into(),
    };
    let grids = [("base", 1000), ("fine", 2000)].map(|(id, hz)| Grid {
        id: id.into(),
        hz,
        source: source.clone(),
    });
    let support = |node: &str| FaultSupport::Available {
        channel: format!("{node}/state"),
        resource: node.into(),
    };
    let paths = [("idle", "line"), ("motion", "line"), ("motion", "reserve")]
        .map(|(mode, path)| RequiredPath {
            station: "bench".into(),
            mode: mode.into(),
            path: path.into(),
            source: source.clone(),
            grids: if path == "line" {
                grids.to_vec()
            } else {
                vec![grids[0].clone()]
            },
            faults: BTreeMap::from([(fault("a"), support("a")), (fault("c"), support("c"))]),
            combinations: vec![],
            unavailable: None,
        })
        .to_vec();
    ResolvedInputs {
        world: WorldInputs::load(campaign.world_path()).unwrap(),
        references: campaign
            .references()
            .map(|r| {
                (
                    r.path.clone(),
                    ResolvedReference {
                        kind: r.kind,
                        file_fingerprint: r.file_fingerprint,
                        graph_fingerprint: Fingerprint::of(b"fixture"),
                    },
                )
            })
            .collect(),
        paths,
        targets: BTreeMap::new(),
        signals: [sim_task::Signal {
            name: "travel".into(),
            unit: "m".into(),
        }]
        .into(),
        model_parameters: BTreeMap::new(),
        initial_velocity: false,
    }
}

fn selection(entries: &[(&str, &str)]) -> Selection {
    entries.iter().map(|&(f, l)| (f.into(), l.into())).collect()
}

fn moving() -> Selection {
    selection(&[
        ("operation", "moving"),
        ("route", "primary"),
        ("clock", "base"),
        ("cut", "open"),
        ("mass", "light"),
        ("wind", "gust"),
    ])
}

#[test]
fn полный_ключ_восстанавливает_уровни_сетку_и_инъекцию() {
    let files = Files::with_depth("bind", DEPTH);
    let campaign = files.load().unwrap();
    let factors = campaign.depth_factors().unwrap();
    assert!(factors.factors()["mass"].safety);
    assert!(!factors.factors()["wind"].safety);
    assert!(matches!(
        factors.factors()["mass"].levels,
        DepthLevels::Axis { .. }
    ));
    let inputs = inventory(&campaign);
    let plan = campaign.prepare_depth(&inputs).unwrap();
    let picked = moving();
    let DepthRow::Bound(row) = plan.row(&picked).unwrap() else {
        panic!("строка не связана")
    };
    assert!(matches!(
        row.disposition,
        sim_task::campaign::Disposition::Ready
    ));
    // Ручная раскладка из данных: bench / motion / line, обрыв a, базовая сетка.
    assert_eq!(
        (
            &*row.key.station,
            &*row.key.mode,
            &*row.key.path,
            &*row.key.grid
        ),
        ("bench", "motion", "line", "base")
    );
    assert_eq!(row.key.faults, [fault("a")]);
    assert_eq!(row.faults.len(), 1);
    assert_eq!(row.faults[0].channel, "a/state");
    assert_eq!(row.faults[0].schedule.at_ns, 1_000_000);
    assert_eq!(
        row.faults[0].window.end_ns - row.faults[0].schedule.at_ns,
        1_000_000
    );
    assert_eq!(factors.selection(&row.key).unwrap(), picked);
    let roundtrip: RunKey = toml::from_str(&toml::to_string(&row.key).unwrap()).unwrap();
    assert_eq!(roundtrip, row.key);

    let materialized = plan.materialize(&picked).unwrap();
    let values: Vec<_> = materialized
        .values()
        .iter()
        .map(|v| (v.id.as_str(), v.value))
        .collect();
    assert_eq!(values, [("mass", 1.0), ("wind", 3.0)]);
    // Независимая сверка наложения: по скомпилированному телу и входу среды.
    assert_eq!(
        materialized
            .world
            .compile()
            .unwrap()
            .bodies
            .iter()
            .find(|b| b.name == "block/rod")
            .unwrap()
            .mass_kg,
        1.0
    );
    assert_eq!(
        materialized.world.world.environment.wind_m_s,
        [3.0, 0.0, 0.0]
    );
    assert_eq!(materialized.world.world.physics_hz, 1000);
    assert_eq!(materialized.mode.unwrap().horizon_ns, 2_000_000);

    let mut reversed = inputs.clone();
    reversed.paths.reverse();
    for path in &mut reversed.paths {
        path.grids.reverse();
    }
    let second = campaign.prepare_depth(&reversed).unwrap();
    let repeat = second.materialize(&picked).unwrap();
    assert_eq!(plan.fingerprint(), second.fingerprint());
    assert_eq!(materialized.fingerprint, repeat.fingerprint);
    assert_eq!(materialized.sample.seeds, repeat.sample.seeds);
    let mut fine = picked.clone();
    fine.insert("clock".into(), "fine".into());
    let fine = plan.materialize(&fine).unwrap();
    assert_eq!(fine.world.world.physics_hz, 2000);
    assert_eq!(fine.sample.seeds, materialized.sample.seeds);
    assert_ne!(fine.key, row.key);
    assert_ne!(fine.fingerprint, materialized.fingerprint);
    let mut other = picked;
    other.insert("mass".into(), "nominal".into());
    assert_ne!(factors.run_key(&other).unwrap().sample, row.key.sample);
}

#[test]
fn физический_запрет_и_пропущенная_инъекция_дают_разные_исходы() {
    let files = Files::with_depth("blocked", DEPTH);
    let campaign = files.load().unwrap();
    let inputs = inventory(&campaign);
    let plan = campaign.prepare_depth(&inputs).unwrap();
    let factors = campaign.depth_factors().unwrap();
    assert!(
        factors
            .exclusion(&selection(&[("operation", "idle")]))
            .unwrap()
            .is_none()
    );
    let forbidden = selection(&[("operation", "idle"), ("route", "secondary")]);
    assert_eq!(
        factors.exclusion(&forbidden).unwrap().unwrap().0,
        "reserve_locked"
    );
    let mut row = moving();
    row.extend(forbidden);
    assert!(matches!(plan.row(&row).unwrap(), DepthRow::Excluded { .. }));
    assert!(plan.materialize(&row).is_err());

    row = moving();
    row.insert("cut".into(), "late".into());
    let DepthRow::Bound(bound) = plan.row(&row).unwrap() else {
        panic!("потерян ключ отказа")
    };
    let sim_task::campaign::Disposition::NotAnalyzed { reasons } = bound.disposition else {
        panic!("строка без инъекции допущена")
    };
    assert!(reasons.iter().any(|r| r.contains("расписание инъекции")));
    assert_eq!(bound.key.faults, [fault("c")]);
    assert!(plan.materialize(&row).is_err());

    row = moving();
    row.insert("route".into(), "secondary".into());
    row.insert("clock".into(), "fine".into());
    assert!(matches!(
        plan.row(&row).unwrap(),
        DepthRow::NotAnalyzed { .. }
    ));
    row.remove("clock");
    assert!(plan.row(&row).is_err());
    row.insert("clock".into(), "unknown".into());
    assert!(factors.exclusion(&row).is_err());
    assert!(campaign.plan(&inputs).is_err());
}

#[test]
fn неверные_данные_отклоняются_до_построения_строк() {
    let parse = |text: &str| toml::from_str::<DepthFactors>(text);
    let raw = DEPTH.replace("depth_factors.", "");
    assert!(parse(&raw).is_ok());
    for (from, to) in [
        ("light = 1.0", "light = nan"),
        ("light = 1.0", "light = inf"),
        ("light = 1.0", "light = -inf"),
        ("light = 1.0", "light = 2.0"),
        ("light = 1.0", "light = -1.0"),
        ("nominal = 2.0", "nominal = 4.0"),
        ("safety = true", "other = true"),
        ("fine = \"fine\"", "fine = \"base\""),
        ("route = \"secondary\"", "route = \"missing\""),
        ("operation = \"idle\"", "stranger = \"idle\""),
        (
            "kind = \"measured\", reference = \"синтетическая таблица",
            "kind = \"estimate\", reference = \"синтетическая таблица",
        ),
    ] {
        assert!(raw.contains(from));
        assert!(parse(&raw.replacen(from, to, 1)).is_err(), "{from} -> {to}");
    }
    let no_depth = Files::with_depth("legacy", "");
    let old = no_depth.load().unwrap();
    assert!(old.depth_factors().is_none());
    assert!(old.plan(&inventory(&old)).is_ok());
}

#[test]
fn конечные_оси_связываются_без_полного_произведения() {
    // 64 независимые двоичные оси дали бы 2^64 строк ещё до добавления режимов и отказов.
    // Идентификаторы синтетические, опыт физики для проверки адресного ключа не нужен.
    let mut raw = DEPTH.replace("depth_factors.", "");
    let mut picked = moving();
    for index in 0..64 {
        raw.push_str(&format!(r#"
[factors.axis{index}]
safety = false
source = {{ kind = "derived", reference = "исходные двоичные уровни" }}
levels = {{ kind = "axis", parameter = {{ kind = "mass", body = "block/rod{index}" }}, unit = "kg", nominal = 1.0, values = {{ one = 1.0, two = 2.0 }} }}
"#));
        picked.insert(format!("axis{index}"), "two".into());
    }
    let factors: DepthFactors = toml::from_str(&raw).unwrap();
    assert_eq!(factors.factors().len(), 70);
    let key = factors.run_key(&picked).unwrap();
    assert_eq!(factors.selection(&key).unwrap(), picked);
}

#[test]
fn группы_отказов_собирают_только_объявленную_совместную_инъекцию() {
    let depth = format!(
        r#"{DEPTH}
[depth_factors.factors.second_cut]
safety = true
source = {{ kind = "derived", reference = "дополнительный тракт стенда" }}
levels = {{ kind = "faults", values = {{ intact = [], open = [{{ node = "b", kind = "wire_open" }}] }} }}
"#
    );
    // Второй обрыв происходит на середине первого миллисекундного такта стенда.
    let extra = format!(
        r#", {{ fault = {{ node = "b", kind = "wire_open" }}, at_ns = 500000, max_lag_ns = 0, window = "whole", source = {SOURCE} }}"#
    );
    let files = Files::with_schedule("groups", &depth, &extra);
    let campaign = files.load().unwrap();
    let mut inputs = inventory(&campaign);
    let path = inputs
        .paths
        .iter_mut()
        .find(|path| path.mode == "motion" && path.path == "line")
        .unwrap();
    path.faults.insert(
        fault("b"),
        FaultSupport::Available {
            channel: "b/state".into(),
            resource: "b".into(),
        },
    );
    let mut picked = moving();
    picked.insert("second_cut".into(), "open".into());
    let plan = campaign.prepare_depth(&inputs).unwrap();
    assert!(matches!(
        plan.row(&picked).unwrap(),
        DepthRow::NotAnalyzed { .. }
    ));
    let path = inputs
        .paths
        .iter_mut()
        .find(|path| path.mode == "motion" && path.path == "line")
        .unwrap();
    path.combinations
        .push(sim_task::campaign::FaultCombination {
            path: "line".into(),
            faults: vec![fault("b"), fault("a")],
            source: path.source.clone(),
        });
    let plan = campaign.prepare_depth(&inputs).unwrap();
    let DepthRow::Bound(row) = plan.row(&picked).unwrap() else {
        panic!("нет совместной строки")
    };
    assert!(matches!(
        row.disposition,
        sim_task::campaign::Disposition::Ready
    ));
    assert!(row.combination_source.is_some());
    assert_eq!(row.key.faults, [fault("a"), fault("b")]);
    // В ключе порядок адресов, в установке порядок времени: 0,5 мс перед 1 мс.
    assert_eq!(
        row.faults
            .iter()
            .map(|f| (f.schedule.fault.node.as_str(), f.schedule.at_ns))
            .collect::<Vec<_>>(),
        [("b", 500_000), ("a", 1_000_000)]
    );
    assert_eq!(plan.factors().selection(&row.key).unwrap(), picked);
    let mut foreign = row.key;
    foreign.faults.push(fault("c"));
    assert!(plan.factors().selection(&foreign).is_err());
}
