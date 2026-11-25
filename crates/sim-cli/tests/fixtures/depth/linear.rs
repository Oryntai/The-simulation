//! Сверка независимых резистивных участков с раздельным включением.
use super::*;

struct Circuits<'a> {
    upstream: &'a str,
    gated: &'a str,
    passive: Vec<&'a str>,
}

impl<'a> Circuits<'a> {
    fn identify(profile: &'a Toml) -> Self {
        let circuits = table(&profile["circuits"]);
        let upstream = circuits
            .iter()
            .find(|(_, c)| table(&c["loads"]).len() > 1)
            .unwrap()
            .0;
        let gated = circuits
            .iter()
            .find(|(owner, c)| {
                *owner != upstream
                    && array(&c["steps"])
                        .iter()
                        .any(|edge| edge["branch"].as_str() == Some("control/command"))
            })
            .unwrap()
            .0;
        let passive = circuits
            .keys()
            .filter(|name| *name != upstream && *name != gated)
            .map(String::as_str)
            .collect::<Vec<_>>();
        assert_eq!(passive.len(), 2);
        Self {
            upstream,
            gated,
            passive,
        }
    }

    fn attachments(&self) -> impl Iterator<Item = &str> + '_ {
        std::iter::once(self.gated).chain(self.passive.iter().copied())
    }
}

#[test]
fn резистивные_участки_сверены_с_монтажом_кирхгофом_и_ngspice() {
    let case = Case::load("06");
    let scratch = Scratch::new("linear-reference");
    let log = scratch.0.join("nominal.simlog");
    let report = case.nominal(&log);
    let profile = read(&case.profile);
    let circuits = Circuits::identify(&profile);
    let dt = ns(&profile["step_ns"]);
    let operation_file = case.world.parent().unwrap().join("operations.toml");
    let operations = read(&operation_file);
    assert_eq!(case.horizon, ns(&operations["window_ns"][1]));
    assert_eq!(case.horizon % dt, 0);

    let mut live_world = read(&case.world);
    let legacy_world = read(&case.world.with_file_name("world.toml"));
    assert_eq!(
        dt,
        sim_signal::Time::from_secs(1.0).unwrap().0 / ns(&legacy_world["physics_hz"])
    );
    let legacy_devices = array(&legacy_world["devices"]);
    let devices = edit(&mut live_world)
        .get_mut("devices")
        .unwrap()
        .as_array_mut()
        .unwrap();
    assert_eq!(devices.len(), legacy_devices.len());
    for (live, legacy) in devices.iter_mut().zip(legacy_devices) {
        let owner = string(&live["name"]);
        let path = case.device(owner);
        if table(&profile["circuits"]).contains_key(owner) {
            let mut variant = read(&path);
            let scheme = edit(&mut variant).remove("schematic").unwrap();
            let legacy_file = Path::new(string(&legacy["file"]));
            let stem = legacy_file.file_stem().unwrap().to_str().unwrap();
            assert_eq!(
                Path::new(string(&scheme)),
                Path::new("..").join(format!("{stem}-electrical.toml"))
            );
            let links = edit(&mut variant)
                .get_mut("links")
                .unwrap()
                .as_array_mut()
                .unwrap();
            for link in links {
                if let Some(mounts) = edit(link).get_mut("mounts") {
                    mounts
                        .as_array_mut()
                        .unwrap()
                        .retain(|mount| string(&mount["kind"]) != "power");
                }
            }
            assert_eq!(
                variant,
                read(&case.world.parent().unwrap().join(legacy_file)),
                "{owner}"
            );
        }
        edit(live).insert("file".into(), legacy["file"].clone());
    }
    assert_eq!(live_world, legacy_world);

    let owner_set = table(&profile["circuits"])
        .keys()
        .map(String::as_str)
        .collect::<BTreeSet<_>>();
    let electrical = &report["electrical_verdict"];
    assert_eq!(
        owner_set,
        electrical["assemblies"]
            .as_object()
            .unwrap()
            .keys()
            .map(String::as_str)
            .collect()
    );
    let commands = array(&operations["models"]["держатель"]["commands"]);
    for owner in &owner_set {
        let steps = array(&profile["circuits"][owner]["steps"]);
        let actual = steps
            .iter()
            .filter(|step| *owner == circuits.upstream || string(&step["branch"]) == "mating/emf")
            .map(|step| ns(&step["at_ns"]))
            .collect::<Vec<_>>();
        let expected = commands
            .iter()
            .filter(|command| {
                let attachment = string(&command["attachment"]);
                attachment == *owner
                    || (*owner == circuits.upstream && owner_set.contains(attachment))
            })
            .map(|command| ns(&command["at_ns"]))
            .collect::<Vec<_>>();
        assert_eq!(actual, expected, "{owner}");
    }
    let gated_steps = array(&profile["circuits"][circuits.gated]["steps"]);
    assert_eq!(
        gated_steps
            .iter()
            .filter(|edge| string(&edge["branch"]) == "control/command")
            .map(|step| ns(&step["at_ns"]))
            .collect::<Vec<_>>(),
        array(&operations["models"]["тепло"]["power_w"])[1..]
            .iter()
            .map(|step| ns(&step["at_ns"]))
            .collect::<Vec<_>>()
    );

    let parts = montaged_parts(&case.catalog);
    for owner in &owner_set {
        let sheet = &electrical["assemblies"][owner];
        assert_eq!(sheet["proof_scope"], "сборка");
        assert!(
            sheet["inapplicable"].as_object().unwrap().is_empty(),
            "{owner}"
        );
        let joints = sheet["sheet"]["operations"].as_array().unwrap().len();
        let items = electrical["findings"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| {
                f["key"]["object"]
                    .as_str()
                    .unwrap()
                    .starts_with(&format!("{owner}/"))
            })
            .collect::<Vec<_>>();
        assert_eq!(
            items.iter().filter(|f| f["outcome"] == "pass").count(),
            joints
        );
        assert!(items.iter().any(|f| f["outcome"] == "not_analyzed"));
        assert!(items.iter().all(|f| f["outcome"] != "fail"));

        let scheme = read(&case.scheme(owner));
        let graph = conductive(&scheme, &parts);
        let ground = if *owner == circuits.upstream {
            "supply/g"
        } else {
            "mating/g"
        };
        for rail in table(&scheme["rails"]).values() {
            let ports = array(&rail["ends"]);
            assert!(reachable(&graph, ground, string(&ports[1])));
        }
        if *owner == circuits.upstream {
            assert!(reachable(&graph, "supply/p", "key/input"));
            assert!(reachable(&graph, "key/output", "outlet/p"));
            assert!(!reachable(&graph, "supply/p", "outlet/p"));
            for axis in ['x', 'y', 'z'] {
                assert!(reachable(
                    &graph,
                    &format!("endstop-{axis}/state"),
                    &format!("control/limit_{axis}")
                ));
            }
        } else {
            assert!(reachable(&graph, "mating/p", "inlet/p"));
            assert_eq!(
                reachable(&graph, "mating/p", "load/p"),
                *owner != circuits.gated
            );
        }
    }

    let signals = traces(&log, &owner_set);
    compare(&scratch.0, &case, &profile, &circuits, &signals);
    let switch_on = ns(&gated_steps[1]["at_ns"]);
    let repeat = case.completed(
        case.command(case.horizon)
            .args(["--snapshot-at", &seconds(switch_on)])
            .output()
            .unwrap(),
    );
    assert_eq!(repeat["snapshot"]["time_ns"], switch_on);
    assert_eq!(repeat["log_fingerprint"], report["log_fingerprint"]);
    assert_eq!(repeat["electrical_verdict"], *electrical);
}

#[derive(Clone, Copy, PartialEq)]
enum Probe {
    Output,
    Axis,
    Gated,
    Passive,
}

fn plateau(probe: Probe, supply: f64, enabled: bool) -> (f64, &'static str, String) {
    // Ручное свёртывание схем и electrical-stimuli.toml: жила 0,2 Ом, стык 0,1 Ом.
    // Три оси по 100,8 Ом, четыре входа по 1000,8 Ом.
    // 2,1 Ом = источник 1 + жила 0,4 + защита 0,1 + две магистрали по 0,3.
    // Ключ: 0,1 Ом вкл., 1 ГОм выкл. Разрядный монитор: 1 МОм с учётом стыков.
    let key = if enabled { 0.1 } else { 1e9 };
    match probe {
        Probe::Output | Probe::Axis => {
            let common = 2.1;
            let branch = key + 1.4 + 1e6;
            let bus = supply / (1.0 + common * (3.0 / 100.8 + 4.0 / 1000.8 + 1.0 / branch));
            let deck = format!(
                "V1 src 0 {supply}\nRsrc src bus {common}\nRaxes bus 0 50.4\nRlogic bus 0 {}\nRpath bus outlet {}\nRbleed outlet 0 1000000.2\nRwire bus axis 0.8\nRaxis axis 0 100",
                1000.8 / 4.0,
                key + 1.2
            );
            if probe == Probe::Output {
                (bus * 1000000.2 / branch, "outlet", deck)
            } else {
                (bus * 100.0 / 100.8, "axis", deck)
            }
        }
        Probe::Gated => {
            // Выходное R источника и стык до развилки: 1,4 Ом.
            // Вход управления: 1000,8 Ом, проводка / отсечка нагрузки: 1,7 Ом.
            let branch = key + 101.7;
            let bus = supply / (1.0 + 1.4 * (1.0 / branch + 1.0 / 1000.8));
            (
                bus * 100.0 / branch,
                "load",
                format!(
                    "V1 src 0 {supply}\nRcommon src bus 1.4\nRlogic bus 0 1000.8\nRwire bus load {}\nRload load 0 100",
                    key + 1.7
                ),
            )
        }
        Probe::Passive => {
            // Источник 1 Ом, стыки 0,2, разъём 0,2, возврат 0,4 и подвод 0,7.
            (
                supply * 100.0 / 102.5,
                "load",
                format!("V1 src 0 {supply}\nRwire src load 2.5\nRload load 0 100"),
            )
        }
    }
}

fn compare(dir: &Path, case: &Case, profile: &Toml, circuits: &Circuits<'_>, traces: &Traces) {
    let dt = ns(&profile["step_ns"]);
    let upstream = circuits.upstream;
    let input = &profile["circuits"][upstream];
    let settings = read(
        &case
            .profile
            .parent()
            .unwrap()
            .join(string(&input["diagnostic_connections"]["parameters"])),
    );
    let scheme = read(&case.scheme(upstream));
    let part = string(&scheme["components"]["supply"]["part"]);
    let emf = number(&settings["parts"][part]["emf"]["volt"]["nominal"]);
    let mut probes = vec![
        (upstream, "фланец", Probe::Output),
        (upstream, "шина X", Probe::Axis),
    ];
    probes.extend(circuits.attachments().map(|owner| {
        (
            owner,
            "нагрузка",
            if owner == circuits.gated {
                Probe::Gated
            } else {
                Probe::Passive
            },
        )
    }));
    let mut dc_error = 0_f64;
    let mut max_edge_ns = 0;
    for (owner, signal, probe) in probes {
        let label = format!("{owner}/{signal}");
        let values = &traces[&label];
        assert!(
            values
                .keys()
                .copied()
                .eq((0..=case.horizon).step_by(dt as usize)),
            "{label}"
        );
        let steps = array(&profile["circuits"][owner]["steps"]);
        let mut supply = if owner == upstream { emf } else { 0.0 };
        let mut enabled = false;
        let mut previous = 0;
        let ends = steps
            .iter()
            .map(|step| ns(&step["at_ns"]))
            .chain(std::iter::once(case.horizon));
        for (index, end) in ends.enumerate() {
            let (manual, node, deck) = plateau(probe, supply, enabled);
            let rows = spice(dir, &deck, "op", &format!("v({node})"));
            assert_eq!(rows.len(), 1);
            let reference = rows[0][1];
            near(reference, manual, ROUNDING.max(1e-7 * manual.abs())); // Сверка печатных контрольных точек Э2.
            let mut points = values.range((
                std::ops::Bound::Excluded(previous),
                std::ops::Bound::Included(end),
            ));
            let (&first_at, &first) = points.next().expect("полка содержит ожидаемые отсчёты");
            let mut peak = first;
            for value in std::iter::once(first).chain(points.map(|(_, &v)| v)) {
                near(value, manual, ROUNDING.max(1e-7 * manual.abs()));
                near(value, reference, dc_tolerance(reference));
                dc_error = dc_error.max((value - reference).abs());
                peak = peak.max(value);
            }
            near(peak, reference, ROUNDING.max(EDGE_REL * reference.abs()));
            if index > 0 {
                max_edge_ns = max_edge_ns.max(first_at - previous);
                near(first, reference, ROUNDING.max(1e-7 * reference.abs()));
            }
            if let Some(step) = steps.get(index) {
                match string(&step["branch"]) {
                    "control/command" => enabled = number(&step["value"]) != 0.0,
                    "mating/emf" => supply = number(&step["value"]),
                    branch => panic!("неизвестная команда {branch}"),
                }
            }
            previous = end;
        }
        eprintln!(
            "{label}: пик {:.6} В",
            values.values().copied().reduce(f64::max).unwrap()
        );
    }
    let axis_voltage = &traces[&format!("{upstream}/шина X")];
    let outlet = &traces[&format!("{upstream}/фланец")];
    for (&at, &voltage) in axis_voltage {
        for axis in ['X', 'Y', 'Z'] {
            near(
                traces[&format!("{upstream}/ток {axis}")][&at] * 100.0,
                voltage,
                ROUNDING,
            );
        }
        let bus = voltage * 100.8 / 100.0;
        let total = 3.0 * bus / 100.8 + 4.0 * bus / 1000.8 + outlet[&at] / 1000000.2;
        near(
            traces[&format!("{upstream}/ток источника")][&at],
            total,
            ROUNDING / 10.0,
        );
    }
    for owner in circuits.attachments() {
        let load = &traces[&format!("{owner}/ток нагрузки")];
        let feed = &traces[&format!("{owner}/ток входа")];
        let tail = &traces[&format!("{owner}/возврат")];
        assert!(
            load.keys().eq(axis_voltage.keys())
                && feed.keys().eq(load.keys())
                && tail.keys().eq(load.keys())
        );
        for (&at, &voltage) in &traces[&format!("{owner}/нагрузка")] {
            near(load[&at] * 100.0, voltage, ROUNDING);
            near(feed[&at], -tail[&at], ROUNDING / 10.0);
        }
    }
    assert_eq!(max_edge_ns, dt);
    compare_gate(
        dir,
        &traces[&format!("{}/нагрузка", circuits.gated)],
        array(&profile["circuits"][circuits.gated]["steps"]),
        dt,
    );
    eprintln!(
        "Резистивные полки: максимальная ΔDC = {dc_error:.3e} В, фронт на сетке {max_edge_ns} нс"
    );
}

fn compare_gate(dir: &Path, truth: &Series, steps: &[Toml], dt: u64) {
    assert_eq!(
        steps
            .iter()
            .map(|s| string(&s["branch"]))
            .collect::<Vec<_>>(),
        [
            "mating/emf",
            "control/command",
            "control/command",
            "mating/emf",
        ]
    );
    let time = |ns| sim_signal::Time(ns).as_secs();
    let start = time(ns(&steps[0]["at_ns"]));
    let rise = time(ns(&steps[1]["at_ns"]));
    let fall = time(ns(&steps[2]["at_ns"]));
    // Фронт ngspice короче макрошага в тысячу раз, чтобы избежать совпадающих точек PWL.
    let dt = time(dt);
    let h = dt / 1000.0;
    let supply = number(&steps[0]["value"]);
    let level = number(&steps[1]["value"]);
    let deck = format!(
        "Vin src 0 PWL(0 0 {start} 0 {} {supply})\nVcmd cmd 0 PWL(0 0 {rise} 0 {} {level} {fall} {level} {} 0)\nRcommon src bus 1.4\nRlogic bus 0 1000.8\nS1 bus out cmd 0 gate\nRwire out load 1.7\nRload load 0 100\n.model gate SW(Ron=0.1 Roff=1e9 Vt=1 Vh=0)",
        start + h,
        rise + h,
        fall + h,
    );
    // Два шага после отсечки дают один полный отсчёт нулевой полки.
    let end = fall + 2.0 * dt;
    let reference = spice(dir, &deck, &format!("tran {dt} {end} 0 {dt}"), "v(load)");
    near(reference.last().unwrap()[0], end, h);
    let peak = reference.iter().map(|r| r[1]).reduce(f64::max).unwrap();
    let actual = truth.values().copied().reduce(f64::max).unwrap();
    near(actual, peak, EDGE_REL * peak);
    let band = dc_tolerance(peak);
    let mut max_delay = 0_f64;
    for (command, target) in [(rise, peak), (fall, 0.0)] {
        let achieved = truth
            .iter()
            .find(|(at, v)| time(**at) >= command && (**v - target).abs() <= band)
            .unwrap();
        let expected = reference
            .iter()
            .find(|r| r[0] >= command && (r[1] - target).abs() <= band)
            .unwrap();
        near(time(*achieved.0), expected[0], dt);
        max_delay = max_delay.max((time(*achieved.0) - expected[0]).abs());
    }
    eprintln!(
        "Ключевой участок: Δпика = {:.3e} В, Δфронта = {:.3e} с",
        (actual - peak).abs(),
        max_delay
    );
}

#[test]
fn резистивные_участки_отвергают_ошибки_монтажа_и_входов() {
    let scratch = Scratch::new("linear-negative");
    let case = Case::load("06").copy(&scratch.0.join("input"));
    let profile = read(&case.profile);
    let circuits = Circuits::identify(&profile);
    let run = || case.command(case.horizon).output().unwrap();

    let device = case.device(circuits.gated);
    let original = fs::read(&device).unwrap();
    let mut broken = read(&device);
    assert!(edit(&mut broken).remove("schematic").is_some());
    write(&device, &broken);
    rejects_input(
        run(),
        "профиль должен покрывать все подключённые схемы мира",
    );
    fs::write(&device, original).unwrap();

    let upstream = case.scheme(circuits.upstream);
    let original = fs::read(&upstream).unwrap();
    let mut broken = read(&upstream);
    assert_eq!(
        broken["connections"]["endstop-x-return-to"]["ends"][1].as_str(),
        Some("endstop-x/g")
    );
    broken["connections"]["endstop-x-return-to"]["ends"][1] = Toml::String("control/g".into());
    write(&upstream, &broken);
    confirms_hazard(
        run(),
        "missing_ground",
        Some(&format!(
            "{}/endstop-x/state → control/limit_x",
            circuits.upstream
        )),
    );
    fs::write(&upstream, original).unwrap();

    let owner = circuits.passive[0];
    let scheme = case.scheme(owner);
    let original = fs::read(&scheme).unwrap();
    let mut altered = read(&scheme);
    let part_file = case.catalog.join("electrical/depth-modules.toml");
    let local_file = "check-parts.toml";
    let mut parts = read(&part_file);
    let mating_part = string(&altered["components"]["mating"]["part"]);
    let pin = &mut parts["parts"][mating_part]["pins"]["p"];
    assert_eq!(pin["key"].as_str(), Some("plus"));
    pin["key"] = Toml::String("wrong".into());
    write(&case.profile.with_file_name(local_file), &parts);
    let original_profile = fs::read(&case.profile).unwrap();
    let mut incompatible_profile = profile.clone();
    assert!(
        edit(&mut incompatible_profile)
            .insert("parts".into(), local_file.into())
            .is_none()
    );
    write(&case.profile, &incompatible_profile);
    confirms_hazard(
        run(),
        "assembly_mismatch",
        Some(&format!("{owner}/dock-feed")),
    );
    fs::write(&case.profile, &original_profile).unwrap();

    let backup = altered["components"]["mating"].clone();
    assert_eq!(backup["nets"]["p"].as_str(), Some("input"));
    assert!(
        edit(&mut altered["components"])
            .insert("backup".into(), backup)
            .is_none()
    );
    let mut second = altered["connections"]["dock-feed"].clone();
    second["ends"][0] = Toml::String("backup/p".into());
    assert!(
        edit(&mut altered["connections"])
            .insert("second-source".into(), second)
            .is_none()
    );
    write(&scheme, &altered);
    rejects_input(run(), "один контакт разъёма занят дважды");
    fs::write(&scheme, original).unwrap();

    // Неконечная команда на фронте первого активного входа.
    for invalid in [f64::NAN, f64::INFINITY] {
        let mut nonfinite = profile.clone();
        let steps = nonfinite["circuits"][owner]["steps"]
            .as_array_mut()
            .unwrap();
        assert_eq!(steps[0]["branch"].as_str(), Some("mating/emf"));
        steps[0]["value"] = Toml::Float(invalid);
        write(&case.profile, &nonfinite);
        rejects_input(run(), "допустимы только конечные числа");
    }
    fs::write(&case.profile, original_profile).unwrap();
    // Сразу после включения, внутри импульса из профиля. Команда снятия ещё впереди.
    let short = ns(&profile["circuits"][circuits.gated]["steps"][1]["at_ns"])
        + 20 * ns(&profile["step_ns"]);
    rejects_input(
        case.command(short).output().unwrap(),
        "электрические события или горизонт вне сетки прогона",
    );
}
