//! Два потребителя с общей шиной и независимыми ключами.
use super::*;
use std::fmt::Write as _;

struct Cycle {
    start: u64,
    disabled: [u64; 2],
    cut: [u64; 2],
    restore: u64,
    enable: u64,
    reverse: u64,
}

impl Cycle {
    fn read(steps: &[Toml], stimuli: &[Toml]) -> Self {
        let times = |unit: &str, value: f64| {
            stimuli
                .iter()
                .filter(|s| string(&s["unit"]) == unit && number(&s["value"]) == value)
                .map(|s| ns(&s["at_ns"]))
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect::<Vec<_>>()
        };
        let [start, enable] = times("bool", 1.0).try_into().unwrap();
        let [_, disable_a, disable_b] = times("bool", 0.0).try_into().unwrap();
        let [reverse] = times("1", -1.0).try_into().unwrap();
        let edge = |index: usize| ns(&steps[index]["at_ns"]);
        assert_eq!(steps.len(), 6);
        assert_eq!(edge(4), edge(5));
        assert!(
            edge(1) < start
                && start < disable_a
                && disable_a < edge(2)
                && edge(2) < disable_b
                && disable_b < edge(3)
                && edge(3) < edge(4)
                && edge(4) < enable
                && enable < reverse
        );
        Self {
            start,
            disabled: [disable_a, disable_b],
            cut: [edge(2), edge(3)],
            restore: edge(4),
            enable,
            reverse,
        }
    }
}

fn sides(circuit: &Toml) -> [String; 2] {
    ['l', 'r'].map(|suffix| {
        let branch = format!("bridge_{suffix}/dc_load");
        let name = table(&circuit["drives"])
            .iter()
            .find(|(_, v)| string(v) == branch)
            .unwrap()
            .0;
        name.split_once('_').unwrap().1.to_owned()
    })
}

fn single_owner(profile: &Toml) -> &str {
    let circuits = table(&profile["circuits"]);
    assert_eq!(circuits.len(), 1);
    circuits.keys().next().unwrap()
}

fn driven<'a>(traces: &'a Traces, owner: &str, suffix: &str, channel: &str) -> &'a Series {
    &traces[&format!("{owner}/мотор_{suffix}/{channel}")]
}

fn probe<'a>(traces: &'a Traces, owner: &str, signal: &str) -> &'a Series {
    &traces[&format!("{owner}/{signal}")]
}

#[test]
fn два_плеча_сверены_с_монтажом_кирхгофом_rc_и_ngspice() {
    let case = Case::load("07");
    let scratch = Scratch::new("dual-reference");
    let directory = &scratch.0;
    let log = directory.join("nominal.simlog");
    let report = case.nominal(&log);
    let profile = read(&case.profile);
    let owner = single_owner(&profile);
    let circuit = &profile["circuits"][owner];
    let scheme = read(&case.scheme(owner));
    let parts = montaged_parts(&case.catalog);
    let graph = conductive(&scheme, &parts);
    for side in ['l', 'r'] {
        let routes = [
            ("source/p".into(), format!("cut_{side}/p")),
            ("source/g".into(), format!("bridge_{side}/g")),
            ("controller/g".into(), format!("bridge_{side}/g")),
            ("controller/g".into(), format!("cut_{side}/g")),
            ("controller/g".into(), format!("encoder_{side}/g")),
            (format!("cut_{side}/q"), format!("bridge_{side}/p")),
            (format!("controller/cut_{side}"), format!("cut_{side}/gate")),
        ];
        for (a, b) in routes {
            assert!(reachable(&graph, &a, &b), "{a} → {b}");
        }
        for pin in ["pwm", "dir", "enable"] {
            assert!(reachable(
                &graph,
                &format!("controller/{pin}_{side}"),
                &format!("bridge_{side}/{pin}")
            ));
        }
        for pin in ['a', 'b'] {
            assert!(reachable(
                &graph,
                &format!("bridge_{side}/{pin}"),
                &format!("motor_{side}/{pin}")
            ));
            assert!(reachable(
                &graph,
                &format!("encoder_{side}/{pin}"),
                &format!("controller/enc{pin}_{side}")
            ));
        }
    }
    assert!(reachable(&graph, "controller/g", "imu/g"));
    for endpoint in [
        "imu/vdd",
        "imu/vddio",
        "controller/p",
        "encoder_l/p",
        "encoder_r/p",
    ] {
        assert!(reachable(&graph, "logic/q", endpoint));
    }
    let rows = report["electrical_verdict"]["findings"].as_array().unwrap();
    assert!(rows.iter().any(|row| row["outcome"] == "not_analyzed"));
    assert!(rows.iter().all(|row| row["outcome"] != "fail"));

    let dir = case.profile.parent().unwrap();
    let control = read(&dir.join(string(&profile["control"])));
    let stimuli = array(&control["stimulus"]);
    let steps = array(&circuit["steps"]);
    let timing = Cycle::read(steps, stimuli);
    let dt = ns(&profile["step_ns"]);
    let labels = sides(circuit);
    let estimates = read(&dir.join(string(&circuit["diagnostic_connections"]["parameters"])));
    let signals = traces(&log, &BTreeSet::from([owner]));
    let network = Network::new(
        &scheme,
        &estimates["parts"],
        number(&circuit["diagnostic_connections"]["ohm"]),
    );
    check_power(&signals, owner, &labels, circuit, &timing, dt);
    check_network(
        directory,
        &signals,
        owner,
        &labels,
        &network,
        &timing,
        &profile,
        case.horizon,
    );

    let repeat_log = directory.join("repeat.simlog");
    // Снимок после отсечения обоих плеч, за такт команд до замыкания ключей.
    let short = timing.restore - (timing.enable - timing.restore);
    let repeat = case.completed(
        case.command(case.horizon)
            .arg("--log")
            .arg(&repeat_log)
            .args(["--snapshot-at", &seconds(short)])
            .output()
            .unwrap(),
    );
    assert_eq!(repeat["snapshot"]["time_ns"], short);
    assert_eq!(fs::read(&log).unwrap(), fs::read(repeat_log).unwrap());
    assert_eq!(
        repeat["electrical_verdict"]["live"]["journal"],
        report["electrical_verdict"]["live"]["journal"]
    );
}

struct Network {
    emf: f64,
    source: f64,
    cap: f64,
    wire: f64,
    trunk: f64,
    bleed: f64,
    drop: f64,
    logic_in: f64,
    logic: f64,
    switch: [f64; 2],
    discharge: [f64; 2],
}

impl Network {
    fn new(scheme: &Toml, estimates: &Toml, joint: f64) -> Self {
        let nominal = |component: &str, element: &str, field: &str| {
            let part = string(&scheme["components"][component]["part"]);
            number(&estimates[part][element][field]["nominal"])
        };
        let r = |component, element| nominal(component, element, "ohm");
        // По схемам плеча зеркальны. Каждая жила имеет два последовательных стыка.
        let wire = r("main_feed", "copper") + 2.0 * joint;
        let parallel = |resistors: &[f64]| 1.0 / resistors.iter().map(|r| r.recip()).sum::<f64>();
        let imu = wire + parallel(&[r("imu", "core") + wire, r("imu", "io") + wire]);
        let logic = wire
            + parallel(&[
                r("controller", "load") + 2.0 * wire,
                imu,
                r("encoder_l", "load") + 2.0 * wire,
                r("encoder_r", "load") + 2.0 * wire,
            ]);
        Self {
            emf: nominal("source", "emf", "volt"),
            source: r("source", "series"),
            cap: nominal("source", "filter", "farad"),
            wire,
            trunk: 3.0 * wire + r("fuse", "closed"),
            bleed: r("bridge_l", "bleed"),
            drop: nominal("logic", "drop", "volt"),
            logic_in: r("logic", "input") + wire,
            logic,
            switch: ["off_ohm", "on_ohm"].map(|field| nominal("cut_l", "switch", field)),
            discharge: ["on_ohm", "off_ohm"].map(|field| nominal("cut_l", "discharge", field)),
        }
    }

    fn logic_current(&self, bus: f64) -> f64 {
        // Через общую жилу текут и нагрузка логики, и входной ток покоя.
        let vin = (bus + self.wire * self.drop / self.logic)
            / (1.0 + self.wire / self.logic + self.wire / self.logic_in);
        (vin - self.drop) / self.logic + vin / self.logic_in
    }

    fn resistances(&self, on: bool) -> (f64, f64, f64) {
        let state = usize::from(on);
        (
            self.wire + self.switch[state],
            self.wire + self.discharge[state],
            2.0 * self.wire,
        )
    }

    fn bridge(&self, bus: f64, load: f64, on: bool) -> (f64, f64, f64) {
        // Uшины = Uключа + Rверх·(Iмоста + Uключа/Rразряда).
        // Uключа = Uмоста + Rниз·Iмоста. Iмоста = Iпривода + Uмоста/Rутечки.
        let (upper, shunt, lower) = self.resistances(on);
        let factor = 1.0 + upper / shunt;
        let rail = (bus - load * (lower * factor + upper))
            / ((1.0 + lower / self.bleed) * factor + upper / self.bleed);
        let local = load + rail / self.bleed;
        (rail, local, local + (rail + lower * local) / shunt)
    }
}

fn check_network(
    dir: &Path,
    traces: &Traces,
    owner: &str,
    labels: &[String; 2],
    n: &Network,
    timing: &Cycle,
    profile: &Toml,
    end: u64,
) {
    let dt = ns(&profile["step_ns"]);
    let circuit = &profile["circuits"][owner];
    let steps = array(&circuit["steps"]);
    let times = (0..=end).step_by(dt as usize).collect::<Vec<_>>();
    let local = |name: &str| probe(traces, owner, name);
    let drive = |side: usize, name| driven(traces, owner, &labels[side], name);
    let source = local("источник");
    let bus = local("общая шина");
    let current = local("ток общей подачи");
    let load = [drive(0, "ток MNA"), drive(1, "ток MNA")];
    let rail = labels
        .each_ref()
        .map(|label| local(&format!("шина {label}")));
    let scalar = [
        bus,
        current,
        source,
        local("ток источника"),
        local("ток общего возврата"),
        local("ток накопителя"),
    ];
    for series in scalar.into_iter().chain(rail).chain(load) {
        assert!(series.keys().eq(&times));
    }
    let enabled = |t, side| {
        let addr = format!("cut_{side}/gate");
        steps
            .iter()
            .rev()
            .find(|s| ns(&s["at_ns"]) < t && string(&s["branch"]) == addr)
            .is_some_and(|s| number(&s["value"]) != 0.0)
    };
    for side in 0..labels.len() {
        let next = drive(side, "next_bus_current_a");
        let volts = drive(side, "bus_voltage_v");
        assert!(next.keys().eq(&times) && volts.keys().eq(&times[1..]));
        for pair in times.windows(2) {
            assert_eq!(load[side][&pair[1]], next[&pair[0]]);
            near(volts[&pair[1]], rail[side][&pair[0]], ROUNDING);
        }
    }
    let mut kcl = 0_f64;
    let mut kvl = 0_f64;
    for t in &times {
        let v = bus[t];
        let i = local("ток источника")[t];
        kcl = kcl
            .max((i - local("ток накопителя")[t] - current[t]).abs())
            .max((current[t] + local("ток общего возврата")[t]).abs());
        kvl = kvl
            .max((source[t] - v - n.trunk * current[t]).abs())
            .max((n.emf - source[t] - n.source * i).abs());
        let mut total = n.logic_current(v);
        for (side, suffix) in ['l', 'r'].into_iter().enumerate() {
            let (u, i, up) = n.bridge(v, load[side][t], enabled(*t, suffix));
            let label = &labels[side];
            kcl = kcl
                .max((i - local(&format!("ток подачи {label}"))[t]).abs())
                .max((i - local(&format!("ток возврата {label}"))[t]).abs())
                .max((up - local(&format!("ток ключа {label}"))[t]).abs())
                .max((up - i - local(&format!("ток разряда {label}"))[t]).abs());
            kvl = kvl.max((u - rail[side][t]).abs());
            total += up;
        }
        kcl = kcl.max((total - current[t]).abs());
    }
    assert!(kcl < ROUNDING && kvl < ROUNDING, "ΔI={kcl}, ΔU={kvl}");

    let mut deck = String::new();
    let mut vectors = String::new();
    let mut compared = Vec::new();
    // Точка DC с замороженной нагрузкой берётся за один такт силового отключения до снятия EN.
    let before_disable = timing.disabled[0] - (timing.cut[0] - timing.disabled[0]);
    let common = n.trunk + n.source;
    for (index, t) in [0, before_disable].into_iter().enumerate() {
        let on = ['l', 'r'].map(|key| enabled(t, key));
        let loads = load.map(|series| series[&t]);
        let total = |v| {
            n.logic_current(v) + n.bridge(v, loads[0], on[0]).2 + n.bridge(v, loads[1], on[1]).2
        };
        // Баланс U = ЭДС − R·I(U) для аффинного I(U).
        let offset = total(0.0);
        let conductance = total(1.0) - offset;
        let voltage = (n.emf - common * offset) / (1.0 + common * conductance);
        let suffix = format!("d{index}");
        writeln!(deck,
            "V{suffix} e{suffix} 0 {}\nR{suffix} e{suffix} m{suffix} {common}\nRf{suffix} m{suffix} in{suffix} {}\nRi{suffix} in{suffix} 0 {}\nVo{suffix} in{suffix} out{suffix} {}\nRo{suffix} out{suffix} 0 {}",
            n.emf, n.wire, n.logic_in, n.drop, n.logic).unwrap();
        write!(vectors, " v(m{suffix})").unwrap();
        compared.push((voltage, bus[&t]));
        for ((side, on), (load, rail)) in ['l', 'r']
            .into_iter()
            .zip(on)
            .zip(loads.into_iter().zip(rail))
        {
            let name = format!("{suffix}{side}");
            let (upper, shunt, lower) = n.resistances(on);
            writeln!(deck,
                "Ru{name} m{suffix} q{name} {upper}\nRd{name} q{name} 0 {shunt}\nRh{name} q{name} b{name} {lower}\nRb{name} b{name} 0 {}\nI{name} b{name} 0 {load}",
                n.bleed).unwrap();
            write!(vectors, " v(b{name})").unwrap();
            compared.push((n.bridge(voltage, load, on).0, rail[&t]));
        }
    }
    let rows = spice(dir, &deck, "op", &vectors);
    assert_eq!(rows.len(), 1);
    let mut dc_error = 0_f64;
    for ((manual, actual), reference) in compared.into_iter().zip(&rows[0][1..]) {
        near(*reference, manual, dc_tolerance(manual));
        near(actual, manual, dc_tolerance(manual));
        near(actual, *reference, dc_tolerance(*reference));
        dc_error = dc_error
            .max((actual - *reference).abs())
            .max((actual - manual).abs());
    }

    let end_s = sim_signal::Time(end).as_secs();
    let dt_s = sim_signal::Time(dt).as_secs();
    // Миллионная доля макрошага нужна строго возрастающему PWL в ngspice.
    let ramp = dt_s * 1e-6;
    let mut previous = current[&0];
    let mut analytic = source[&0];
    let mut rc_error = 0_f64;
    let mut deck = format!(
        "Vsrc e 0 {}\nRs e p {}\nCsrc p 0 {}\nIload p 0 PWL(\n+ 0 {previous}\n",
        n.emf, n.source, n.cap
    );
    for &t in &times[1..] {
        let now = current[&t];
        let equilibrium = n.emf - n.source * now;
        analytic = equilibrium + (analytic - equilibrium) * (-dt_s / (n.source * n.cap)).exp();
        rc_error = rc_error.max((analytic - source[&t]).abs());
        if now != previous {
            let at = sim_signal::Time(t - dt).as_secs();
            if at > 0.0 {
                writeln!(deck, "+ {at:.17e} {previous:.17e}").unwrap();
            }
            writeln!(deck, "+ {:.17e} {now:.17e}", at + ramp).unwrap();
        }
        previous = now;
    }
    writeln!(deck, "+ {end_s:.17e} {previous:.17e}\n+ )\n.option method=trap reltol=1e-8 vntol=1e-9 abstol=1e-11").unwrap();
    let reference = spice(dir, &deck, &format!("tran {dt_s} {end_s}"), "v(p)");
    near(reference.last().unwrap()[0], end_s, ramp);
    let reference = times
        .iter()
        .map(|&t| {
            let secs = sim_signal::Time(t).as_secs();
            let index = reference.partition_point(|row| row[0] < secs);
            let v = match index {
                0 => reference[0][1],
                i if i == reference.len() => reference[i - 1][1],
                i => {
                    let [a, b] = &reference[i - 1..=i] else {
                        unreachable!()
                    };
                    a[1] + (b[1] - a[1]) * (secs - a[0]) / (b[0] - a[0])
                }
            };
            (t, v)
        })
        .collect::<Series>();
    let spice_error = source
        .iter()
        .map(|(t, v)| (v - reference[t]).abs())
        .reduce(f64::max)
        .unwrap();
    assert!(rc_error.max(spice_error) <= dc_tolerance(n.emf));
    eprintln!(
        "Два плеча: ΔI={kcl:.3e} А, ΔU={kvl:.3e} В, ΔDC={dc_error:.3e} В, ΔRC={rc_error:.3e} В, Δngspice={spice_error:.3e} В"
    );
    for start in [timing.start, timing.enable] {
        // Исходная приёмка: 20 постоянных RC захватывают передний фронт до медленной механики.
        let rc_window = sim_signal::Time::from_secs(20.0 * n.source * n.cap)
            .unwrap()
            .0;
        let end = start + rc_window / dt * dt;
        let target = reference[&end];
        let band = EDGE_REL * (reference[&start] - target).abs();
        let settle = |series: &Series| {
            let last_outside = series
                .range(start..=end)
                .filter(|(_, v)| (**v - target).abs() > band)
                .map(|(&t, _)| t)
                .next_back()
                .unwrap();
            assert!(last_outside < end);
            (last_outside + dt - start) as f64
        };
        let peak = local("ток источника")
            .range(start..=end)
            .map(|(_, &v)| v)
            .reduce(f64::max)
            .unwrap();
        let expected = reference
            .range(start..=end)
            .map(|(_, &v)| (n.emf - v) / n.source)
            .reduce(f64::max)
            .unwrap();
        let peak_error = (peak / expected - 1.0).abs();
        let settle_error = (settle(source) / settle(&reference) - 1.0).abs();
        assert!(
            peak_error.max(settle_error) <= EDGE_REL,
            "{start}: {peak_error}, {settle_error}"
        );
        eprintln!(
            "Пуск {} нс: пик {peak:.6} А, установление {} нс, относительные ошибки {peak_error:.3e} / {settle_error:.3e}",
            start,
            settle(source)
        );
    }
}

fn check_power(
    signals: &Traces,
    owner: &str,
    sides: &[String; 2],
    circuit: &Toml,
    cycle: &Cycle,
    dt: u64,
) {
    let currents = sides
        .each_ref()
        .map(|name| driven(signals, owner, name, "ток MNA"));
    let voltage = sides
        .each_ref()
        .map(|name| probe(signals, owner, &format!("шина {name}")));
    let first = cycle.start + dt;
    near(currents[0][&first], currents[1][&first], ROUNDING);
    assert!(currents[0][&first] > 0.0 && voltage[0][&first] < voltage[0][&cycle.start]);
    let bus = probe(signals, owner, "общая шина");
    let tick = cycle.cut[0] - cycle.disabled[0];
    assert!(bus[&(cycle.start + tick)] < bus[&(cycle.disabled[1] - tick)]);
    let separate = cycle.cut[0] + tick;
    assert!(voltage[0][&separate] < voltage[1][&separate] && currents[1][&separate] > 0.0);
    for (index, side) in sides.iter().enumerate() {
        let cut = cycle.cut[index];
        let current = currents[index];
        assert!(
            current
                .range(cut + dt..cycle.restore)
                .any(|(_, &v)| v < 0.0)
        );
        let key = probe(signals, owner, &format!("ток ключа {side}"));
        let discharge = probe(signals, owner, &format!("ток разряда {side}"));
        let off_peak = key
            .range(cut + dt..=cycle.restore)
            .map(|(_, v)| v.abs())
            .reduce(f64::max)
            .unwrap();
        let discharge_peak = discharge
            .range(cut + dt..=cycle.restore)
            .map(|(_, &v)| v)
            .reduce(f64::max)
            .unwrap();
        assert!(off_peak < ROUNDING && key[&(cut - dt)] > off_peak && discharge_peak > off_peak);
        let duty = driven(signals, owner, side, "duty");
        let mut idle = duty.range(cycle.disabled[1] + dt..=cycle.enable).peekable();
        assert!(idle.peek().is_some() && idle.all(|(_, &d)| d == 0.0));
        assert!(voltage[index][&(cycle.restore + dt)] > voltage[index][&cycle.restore]);
        assert_eq!(duty[&(cycle.enable + dt)], 1.0);
        assert!(current[&(cycle.enable + dt)] > 0.0);
        assert_eq!(duty[&(cycle.reverse + dt)], -1.0);
    }
    let meter = table(&circuit["meters"]);
    assert_eq!(meter.len(), 1);
    let (name, config) = meter.iter().next().unwrap();
    let min = probe(signals, owner, &format!("{name}/питание"))
        .values()
        .copied()
        .reduce(f64::min)
        .unwrap();
    assert!(min < number(&config["minimum_v"]));
    eprintln!(
        "Два плеча: Uшины min {:.6} В, пик общей подачи {:.6} А, Uдатчика min {min:.6} В",
        bus.values().copied().reduce(f64::min).unwrap(),
        probe(signals, owner, "ток общей подачи")
            .values()
            .copied()
            .reduce(f64::max)
            .unwrap()
    );
}

#[test]
fn два_плеча_отвергают_дефекты_а_возврат_питания_не_заменяет_разрешение() {
    let scratch = Scratch::new("dual-negative");
    let case = Case::load("07").copy(&scratch.0.join("input"));
    let profile = read(&case.profile);
    let owner = single_owner(&profile);
    let run = || case.command(case.horizon).output().unwrap();
    let scheme = case.scheme(owner);
    let raw = read(&scheme);
    let original = fs::read(&scheme).unwrap();
    let mut broken = raw.clone();
    remove_joint(&mut broken, "bridge_return_l_finish");
    let parts = montaged_parts(&case.catalog);
    let graph = conductive(&broken, &parts);
    assert!(!reachable(&graph, "controller/g", "bridge_l/g"));
    assert!(reachable(&graph, "controller/g", "bridge_r/g"));
    write(&scheme, &broken);
    confirms_hazard(run(), "missing_ground", None);

    let mut wrong = raw.clone();
    let joint = &mut wrong["connections"]["bridge_feed_l_finish"];
    assert_eq!(joint["kind"].as_str(), Some("terminal"));
    joint["kind"] = "solder".into();
    write(&scheme, &wrong);
    confirms_hazard(
        run(),
        "assembly_mismatch",
        Some(&format!("{owner}/bridge_feed_l_finish")),
    );

    let mut wrong = raw.clone();
    let end = &mut wrong["connections"]["bridge_feed_l_finish"]["ends"][1];
    assert_eq!(end.as_str(), Some("bridge_l/p"));
    *end = "bridge_l/g".into();
    write(&scheme, &wrong);
    rejects_input(run(), "bridge_feed_l_finish");
    fs::write(&scheme, original).unwrap();

    let device = case.device(owner);
    let original = fs::read(&device).unwrap();
    let mut absent = read(&device);
    assert!(edit(&mut absent).remove("schematic").is_some());
    write(&device, &absent);
    rejects_input(run(), "схемы мира");
    fs::write(&device, original).unwrap();

    let control = case
        .profile
        .parent()
        .unwrap()
        .join(string(&profile["control"]));
    let original = fs::read(&control).unwrap();
    let mut stimuli = read(&control);
    let stimuli_rows = stimuli["stimulus"].as_array_mut().unwrap();
    let circuit = &profile["circuits"][owner];
    let timing = Cycle::read(array(&circuit["steps"]), stimuli_rows);
    let count = stimuli_rows.len();
    stimuli_rows.retain(|row| ns(&row["at_ns"]) != timing.enable);
    assert_eq!(count - stimuli_rows.len(), sides(circuit).len());
    write(&control, &stimuli);
    let log = scratch.0.join("without-enable.simlog");
    case.nominal(&log);
    let signals = traces(&log, &BTreeSet::from([owner]));
    let short = timing.restore - (timing.enable - timing.restore);
    for side in sides(circuit) {
        let rail = probe(&signals, owner, &format!("шина {side}"));
        assert!(rail[&timing.enable] > rail[&short]);
        let dt = ns(&profile["step_ns"]);
        let duty = driven(&signals, owner, &side, "duty");
        let mut samples = duty.range(timing.enable + dt..=case.horizon).peekable();
        assert!(samples.peek().is_some() && samples.all(|(_, &v)| v == 0.0));
    }
    fs::write(&control, original).unwrap();

    rejects_input(case.command(short).output().unwrap(), "вне режима");
    let out = steps_from_stdin(case.command(case.horizon), short / ns(&profile["step_ns"]));
    let report = json(&out);
    assert_eq!(out.status.code(), Some(1));
    assert_eq!(report["status"], "interrupted");
    assert_eq!(report["blocks_proof"], true);
    let electrical = &report["electrical_verdict"];
    let live = &electrical["live"];
    assert_eq!(live["complete"], false);
    assert_eq!(live["observed_ns"], short);
    assert_eq!(live["required_ns"], case.horizon);
    assert_eq!(electrical["simulation"], "NotProven");
    assert!(
        electrical["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|f| f["code"] == "execution"
                && f["outcome"] == "not_analyzed"
                && f["journal"] == live["journal"])
    );

    let original = fs::read(&case.profile).unwrap();
    for invalid in [f64::NAN, f64::INFINITY] {
        let mut mutated = profile.clone();
        mutated["circuits"][owner]["steps"][0]["value"] = Toml::Float(invalid);
        write(&case.profile, &mutated);
        rejects_input(run(), "конечн");
    }
    fs::write(&case.profile, original).unwrap();

    // Формат 1 со старым относительным полем parts остаётся рабочим.
    let mut legacy = profile.clone();
    assert!(
        edit(&mut legacy)
            .insert("parts".into(), "electrical-parts.toml".into())
            .is_none()
    );
    write(&case.profile, &legacy);
    case.completed(run());
}

#[test]
fn монтаж_доступен_через_cli_без_профиля() {
    let scratch = Scratch::new("dual-static");
    let case = Case::load("07");
    let path = scratch.0.join("acceptance.toml");
    let mut manifest = read(&root().join("devices/electrical/acceptance.toml"));
    manifest["catalog"] = case.catalog.to_str().unwrap().into();
    let cases = edit(&mut manifest["assemblies"]);
    let mut config = cases.remove("07").unwrap();
    cases.clear();
    let table = edit(&mut config);
    table.remove("profile").unwrap();
    table.remove("horizon_ns").unwrap();
    table.insert("world".into(), case.world.to_str().unwrap().into());
    cases.insert("07".into(), config);
    write(&path, &manifest);
    let out = Command::new(env!("CARGO_BIN_EXE_sim-run"))
        .arg("--campaign")
        .arg(path)
        .arg("--json")
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let report = json(&out);
    let assembly = &report["assemblies"]["07"];
    assert_eq!(assembly["simulation"], "NotProven");
    assert!(assembly["live"].is_null());
    let published = root().join("devices/electrical/assembly-07.json");
    let published: Value = serde_json::from_slice(&fs::read(published).unwrap()).unwrap();
    let rows = assembly["assemblies"].as_object().unwrap();
    assert_eq!(rows.len(), published.as_object().unwrap().len());
    for (owner, row) in rows {
        assert_eq!(row["sheet"], published[owner]["sheet"]);
        assert_eq!(row["schematic"], published[owner]["schematic"]);
    }
}
