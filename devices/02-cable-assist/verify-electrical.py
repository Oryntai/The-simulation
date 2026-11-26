"""Приёмка данных 1.1 через sim-run, прозвонку графа и ngspice."""

import csv
import json
import math
import re
import shutil
import subprocess
import tempfile
import tomllib
from collections import Counter, defaultdict, deque
from pathlib import Path


DEVICE = Path(__file__).resolve().parent
ROOT = DEVICE.parents[1]
RUN = ROOT / "target/debug/sim-run"
SPICE = ROOT / "assets/third_party/ngspice/bin/ngspice"
SCHEME = "electrical-scheme.toml"


def read(path):
    return tomllib.loads(path.read_text())


def run(directory=DEVICE, suffix="", *options):
    result = subprocess.run(
        [RUN, directory / f"world{suffix}.toml", "--duration", DURATION,
         "--electrical", directory / f"electrical-profile{suffix}.toml",
         "--catalog", ROOT / "catalog", "--json", *options],
        cwd=ROOT, capture_output=True, check=False,
    )
    return result.returncode, json.loads(result.stdout)


def findings(report, code, outcome):
    return [
        item for item in report["electrical_verdict"]["findings"]
        if item["code"] == code and item["outcome"] == outcome
    ]


def connected(data, parts, start, end):
    graph = defaultdict(set)

    def add(a, b):
        graph[a].add(b)
        graph[b].add(a)

    for item in data["connections"].values():
        add(*item["ends"])
    for name, component in data["components"].items():
        model = parts[component["part"]]
        for branch in model.get("elements", {}).values():
            if branch.get("continuity"):
                add(*(f"{name}/{pin}" for pin in branch["nodes"]))
    queue = deque([start])
    seen = {start}
    while queue:
        at = queue.popleft()
        if at == end:
            return True
        for neighbour in sorted(graph[at] - seen):
            seen.add(neighbour)
            queue.append(neighbour)
    return False


def sheet_and_routes(report, owner, data, parts):
    verdict = report["electrical_verdict"]
    assembly = verdict["assemblies"][owner]
    assert report["status"] == "completed" and report["blocks_proof"]
    assert verdict["simulation"] == "NotProven" and verdict["live"]["complete"]
    assert assembly["proof_scope"] == "сборка"
    sheet = assembly["sheet"]
    assert {item["component"]: item["pins"] for item in sheet["parts"]} == {
        name: part["nets"] for name, part in data["components"].items()
    }

    def ends(values):
        return [f"{value['component']}/{value['pin']}" for value in values]

    seen = set()
    for operation in sheet["operations"]:
        name = operation["id"]
        original = data["connections"][name]
        assert name not in seen and operation["compatible"]
        assert ends(operation["ends"]) == original["ends"]
        assert set(operation["after"]) == set(original["after"]) <= seen
        seen.add(name)
    assert seen == data["connections"].keys()
    assert len(findings(report, "assembly_mismatch", "pass")) == len(seen)
    assert {tuple(ends(check["ends"])) for check in sheet["inspections"]
            if check["test"] == "continuity"} == {("controller/g", "bridge/g")}
    for stage, test in [("before_power", "resistance"),
                        ("current_limited_start", "current_limit")]:
        assert {tuple(ends(item["ends"])) for item in sheet["inspections"]
                if item["stage"] == stage and item["test"] == test} == {
            tuple(item["ends"]) for item in data["connections"].values()
        }
    routes = [
        ("cell/g", "boost/g"), ("bms/g", "cell/g"), ("bridge/g", "boost/g"),
        ("controller/g", "bridge/g"), ("cell/p", "bms/input"),
        ("bms/output", "boost/input"), ("boost/input", "controller/p"),
        ("boost/output", "bridge/p"),
        ("bridge/a", "motor/a"), ("bridge/b", "motor/b"),
        *((f"controller/{pin}", f"bridge/{pin}") for pin in ("pwm", "dir", "enable")),
    ]
    assert all(connected(data, parts, *pair) for pair in routes)
    assert all(not connected(data, parts, *rail["ends"]) for rail in data["rails"].values())
    return sheet


def trace(path, owner):
    signals = defaultdict(dict)
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if row["kind"] != "truth" or not row["name"].startswith(owner + "/"):
                continue
            assert row["valid"] == "true"
            name = row["name"].split("/", 1)[1]
            time = int(row["time_ns"])
            value = float(row["value"])
            assert math.isfinite(value) and time not in signals[name]
            signals[name][time] = value
    return signals


def spice_reference(folder, time_ns, supply, current, resistance, harness, capacitance, dc_load):
    # На границе CLI публикует значение слева, после этого действует новая полка.
    # Фронт SPICE занимает миллионную часть штатного электрического шага.
    edge = min(b - a for a, b in zip(time_ns, time_ns[1:])) * 1e-6

    def pwl(values):
        points = [(0, values[0])]
        previous = values[0]
        for at, value in values.items():
            if value != previous:
                points += [(at, previous), (at + edge, value)]
                previous = value
        points.append((time_ns[-1], previous))
        return " ".join(f"{float(t)*1e-9:.17g} {v:.17g}" for t, v in points)

    (folder / "bus.cir").write_text(
        "RC-шина с удержанным током привода\n"
        f"Vsrc emf 0 PWL({pwl(supply)})\n"
        f"Rsrc emf out {resistance}\n"
        f"Cout out 0 {capacitance}\n"
        f"Rh out rail {harness}\n"
        f"Iload rail 0 PWL({pwl(current)})\n"
        f"Vdc dc_emf 0 DC {supply[0]}\n"
        f"Rdc dc_emf dc_rail {resistance + harness}\n"
        f"Idc dc_rail 0 DC {dc_load}\n"
        ".option method=trap reltol=1e-8 vntol=1e-9 abstol=1e-11\n"
        ".control\nset wr_singlescale\nset numdgt=16\n"
        "op\nwrdata dc.dat v(rail) v(dc_rail)\n"
        f"tran {min(b-a for a,b in zip(time_ns, time_ns[1:])) * 1e-9} "
        f"{time_ns[-1] * 1e-9}\n"
        "wrdata tran.dat v(out) v(rail) i(Vsrc)\n"
        "quit\n.endc\n.end\n"
    )
    subprocess.run([SPICE, "-n", "-b", "bus.cir"], cwd=folder,
                   capture_output=True, check=True)
    dc = tuple(map(float, (folder / "dc.dat").read_text().split()[1:]))
    table = [tuple(map(float, row.split()))
             for row in (folder / "tran.dat").read_text().splitlines()]
    assert all(math.isfinite(v) for row in table for v in row)
    assert all(b[0] > a[0] for a, b in zip(table, table[1:]))
    # Точки точно перед фронтом записаны явно. Интерполяция нужна между ними.
    interpolated = {}
    j = 0
    for at in time_ns:
        sec = at * 1e-9
        while j + 1 < len(table) and table[j + 1][0] < sec:
            j += 1
        if j + 1 == len(table):
            assert abs(sec - table[j][0]) < edge * 1e-9
            interpolated[at] = table[j][1:]
        else:
            left, right = table[j:j+2]
            fraction = (sec - left[0]) / (right[0] - left[0])
            interpolated[at] = tuple(a + (b - a) * fraction
                                     for a, b in zip(left[1:], right[1:]))
    return dc, interpolated


def numerics(folder, signals, profile, estimates, world):
    circuit = profile["circuits"]["устройство"]
    model = estimates["parts"]["motor_boost"]
    emf = model["emf"]["volt"]["nominal"]
    resistance = model["series"]["ohm"]["nominal"]
    capacitance = model["filter"]["farad"]["nominal"]
    # В каждой из двух жил медь и два монтажных стыка.
    copper = estimates["parts"]["flexible_harness_wire"]["copper"]["ohm"]["nominal"]
    harness = 2 * (copper + 2 * circuit["diagnostic_connections"]["ohm"])
    load = signals["двигатель/ток MNA"]
    source = signals["ток источника шины"]
    output = signals["выход преобразователя"]
    rail = signals["шина"]
    next_current = signals["двигатель/next_bus_current_a"]
    dt = 10**9 // world["physics_hz"]
    assert len(load) == HORIZON // profile["step_ns"] + 1
    assert list(next_current) == list(range(0, HORIZON + dt, dt))
    assert all(load[t + dt] == next_current[t] for t in next_current if t < HORIZON)
    assert all(v == rail[t - dt] or abs(v - rail[t - dt]) < 1e-12
               for t, v in signals["двигатель/bus_voltage_v"].items())
    kcl = max(abs(source[t] - load[t] - signals["ток выходного фильтра"][t])
              for t in load)
    wire_error = max(abs(rail[t] - output[t] + load[t] * harness) for t in load)
    assert kcl < 1e-9 and wire_error < 1e-9
    for name in ("ток подающей жилы", "ток возвратной жилы"):
        assert max(abs(signals[name][t] - load[t]) for t in load) < 1e-9

    steps = {step["at_ns"]: step["value"] for step in circuit["steps"]}
    drop, restore = steps
    pwm = signals["двигатель/duty"]
    accepted = signals["двигатель/accepted_command"]
    # Во время провала команда сохраняется, драйвер переходит на выбег.
    assert pwm[drop] > 0.0
    assert accepted[drop] == accepted[restore] > 0.0
    assert pwm[restore] == 0.0 and min(load[t] for t in load if drop < t < restore) < 0.0
    assert rail[restore] < circuit["reference_drive_min_v"]["двигатель"]
    assert pwm[restore + (restore - drop)] == accepted[restore]
    reversal = next(t for t, value in accepted.items() if value < 0.0)
    assert load[reversal + dt] < 0.0 and pwm[reversal + dt] < 0.0

    assert load[drop] > 0.0
    (dc_idle, dc_loaded), spice = spice_reference(
        folder, list(rail), {0: emf, **steps}, {**next_current, HORIZON: load[HORIZON]},
        resistance, harness, capacitance, load[drop],
    )
    calculated_dc = emf - load[drop] * (resistance + harness)
    dc_error = max(abs(emf - rail[0]), abs(emf - dc_idle),
                   abs(rail[drop] - dc_loaded), abs(calculated_dc - dc_loaded))
    assert dc_error <= max(0.01, 0.01 * abs(emf))  # Допуск Э2.
    ref_output = emf
    ref_source = emf
    held = load[0]
    previous = 0
    max_rc_error = 0.0
    for t in rail:
        equilibrium = ref_source - resistance * held
        ref_output = equilibrium + (ref_output - equilibrium) * math.exp(
            -(t - previous) * 1e-9 / (resistance * capacitance))
        max_rc_error = max(max_rc_error, abs(rail[t] - ref_output + harness * held))
        ref_source = steps.get(t, ref_source)
        held = next_current.get(t, held)
        previous = t
    max_spice_error = max(abs(rail[t] - spice[t][1]) for t in rail)
    assert max(max_rc_error, max_spice_error) <= max(0.01, 0.01 * abs(emf))

    def settle(series, window, target):
        # Полоса 5% амплитуды ступени. Установление должно сохраниться до конца окна.
        outside = [i for i, t in enumerate(window) if abs(series[t] - target) > 0.05 * emf]
        assert outside and outside[-1] + 1 < len(window)
        return window[outside[-1] + 1] - window[0]

    fronts = []
    for origin, target in steps.items():
        window = [t for t in rail if origin <= t <= origin + (restore - drop)]
        actual_peak = max(abs(source[t]) for t in window)
        reference_peak = max(abs(spice[t][2]) for t in window)
        actual_settle = settle(output, window, target)
        reference_settle = settle({t: spice[t][0] for t in window}, window, target)
        peak_error = abs(actual_peak / reference_peak - 1)
        settle_error = abs(actual_settle / reference_settle - 1)
        assert max(peak_error, settle_error) <= 0.05, (origin, peak_error, settle_error)
        fronts.append((actual_peak, actual_settle))
        print(f"Фронт {origin/1e6:g} мс: пик {actual_peak:.6f} А, "
              f"установление {actual_settle/1e3:g} мкс, ошибки {peak_error:.3%}/{settle_error:.3%}")
    print(f"DC Δ={dc_error:.3g} В, RC Δ={max_rc_error:.3g} В, ngspice Δ={max_spice_error:.3g} В; "
          f"Кирхгоф Δ={kcl:.3g} А / {wire_error:.3g} В")
    print(f"Шина {min(rail.values()):.6f}…{max(rail.values()):.6f} В; "
          f"ток шины {min(load.values()):.6f}…{max(load.values()):.6f} А")
    return fronts


def copy_fixture(folder, name):
    variant = folder / name
    variant.mkdir()
    for name in ("world.toml", "device.toml", SCHEME, "electrical-profile.toml",
                 "electrical-stimuli.toml", "electrical-diagnostics.toml"):
        shutil.copyfile(DEVICE / name, variant / name)
    return variant


def negative(folder, parts):
    variant = copy_fixture(folder, "negative")
    device = variant / "device.toml"
    original_device = device.read_text()
    device.write_text(re.sub(r"^schematic = .*\n", "", original_device, flags=re.M))
    code, report = run(variant)
    assert code == 2 and "профиль должен покрывать" in report["detail"], (code, report.get("detail"))
    device.write_text(original_device)

    scheme = variant / SCHEME
    original = scheme.read_text()
    broken = re.sub(r"\[connections\.controller_return_finish\]\n.*?(?=^\[)", "",
                    original, flags=re.M | re.S)
    broken = broken.replace(', "controller_return_finish"', "")
    scheme.write_text(broken)
    assert not connected(read(scheme), parts, "controller/g", "bridge/g")
    code, report = run(variant)
    assert code == 0 and report["electrical_verdict"]["simulation"] == "Rejected", (code, report.get("detail"))
    assert len(findings(report, "missing_ground", "fail")) >= 3

    scheme.write_text(original.replace('[connections.motor_a_lead_start]\nkind = "terminal"',
                                      '[connections.motor_a_lead_start]\nkind = "solder"'))
    code, report = run(variant)
    assert code == 0 and report["electrical_verdict"]["simulation"] == "Rejected", (code, report.get("detail"))
    assert len(findings(report, "assembly_mismatch", "fail")) >= 1

    scheme.write_text(original.replace("[modes.source_disconnects]",
        '[[bindings]]\ntarget = { kind = "drive_power", drive = "двигатель" }\n'
        'ends = ["boost/output", "boost/g"]\n\n[modes.source_disconnects]'))
    code, report = run(variant)
    assert code == 2 and "повторная привязка" in report["detail"], (code, report.get("detail"))

    profile = variant / "electrical-profile.toml"
    original_profile = profile.read_text()
    scheme.write_text(original)
    profile.write_text(original_profile.replace("ohm = 0.1", "ohm = nan"))
    code, report = run(variant)
    assert code == 2 and "конеч" in report["detail"], (code, report.get("detail"))

    stopped = subprocess.run(
        [RUN, DEVICE / "world.toml", "--duration", DURATION,
         "--electrical", DEVICE / "electrical-profile.toml",
         "--mode", "step", "--json"],
        cwd=ROOT, input=b"\n", capture_output=True, check=False,
    )
    report = json.loads(stopped.stdout)
    assert stopped.returncode == 1 and report["status"] == "interrupted"
    assert not report["electrical_verdict"]["live"]["complete"]
    assert report["electrical_verdict"]["simulation"] == "NotProven"
    assert findings(report, "execution", "not_analyzed")
    print("Отрицательные случаи: снятая схема, обрыв возврата, неверная клемма, "
          "двойное питание, NaN, усечённый журнал.")


def main():
    data = read(DEVICE / SCHEME)
    parts = read(ROOT / "catalog/electrical/depth-modules.toml")["parts"]
    profile = read(DEVICE / "electrical-profile.toml")
    world = read(DEVICE / "world.toml")
    estimates = read(DEVICE / "electrical-diagnostics.toml")
    with tempfile.TemporaryDirectory(prefix="sim-next-t11-") as temporary:
        folder = Path(temporary)
        csv_path = folder / "trace.csv"
        log = folder / "continuous.bin"
        code, primary = run(DEVICE, "", "--csv", csv_path, "--log", log)
        assert code == 0, primary.get("detail") or primary.get("component_failure")
        sheet = sheet_and_routes(primary, "устройство", data, parts)
        snap_at = profile["circuits"]["устройство"]["steps"][0]["at_ns"] / 1e9
        split = folder / "snapshot.bin"
        code, replay = run(DEVICE, "", "--snapshot-at", str(snap_at), "--log", split)
        assert code == 0 and log.read_bytes() == split.read_bytes()
        assert primary["electrical_verdict"] == replay["electrical_verdict"]
        code, renamed = run(DEVICE, "-renamed")
        assert code == 0, renamed.get("detail") or renamed.get("component_failure")
        assert sheet == sheet_and_routes(renamed, "стенд", data, parts)
        assert (primary["electrical_verdict"]["assemblies"]["устройство"]["schematic"] ==
                renamed["electrical_verdict"]["assemblies"]["стенд"]["schematic"])
        for signal in ("шина", "двигатель/ток MNA", "двигатель/current_a"):
            assert (primary["channels"]["truth"][f"устройство/{signal}"] ==
                    renamed["channels"]["truth"][f"стенд/{signal}"])
        print(f"Электрический шаг {profile['step_ns']} нс")
        nominal_fronts = numerics(folder, trace(csv_path, "устройство"), profile, estimates, world)
        fine = copy_fixture(folder, "fine")
        fine_profile = fine / "electrical-profile.toml"
        fine_step = profile["step_ns"] // 2
        fine_profile.write_text(fine_profile.read_text().replace(
            f"step_ns = {profile['step_ns']}", f"step_ns = {fine_step}",
        ))
        fine_csv = fine / "trace.csv"
        code, result = run(fine, "", "--csv", fine_csv)
        assert code == 0, result.get("detail") or result.get("component_failure")
        sheet_and_routes(result, "устройство", data, parts)
        print(f"Электрический шаг {fine_step} нс")
        fine_fronts = numerics(fine, trace(fine_csv, "устройство"), read(fine_profile), estimates, world)
        # Допуск Э2 между сетками. Для времени учитываем грубый отсчёт.
        for (peak, settling), (fine_peak, fine_settling) in zip(nominal_fronts, fine_fronts):
            assert abs(peak / fine_peak - 1) <= 0.05
            assert abs(settling - fine_settling) <= max(profile["step_ns"], 0.05 * fine_settling)
        negative(folder, parts)

    totals = Counter(item["outcome"] for item in primary["electrical_verdict"]["findings"])
    missing = sum(len(item["missing_parameters"]) for item in sheet["parts"])
    missing += sum(item["limit"] is not None and item["limit"]["range"] is None
                   for item in sheet["inspections"])
    print(f"Оба владельца и снимок: NotProven, {len(sheet['parts'])} позиций, "
          f"{len(sheet['operations'])} операций, {missing} открытых параметров и пределов, "
          f"{totals['not_analyzed']} NotAnalyzed.")


if __name__ == "__main__":
    # После штатного стопа в 108 мс оставляем стартовый интервал без команды (8 мс).
    commands = read(DEVICE / "electrical-stimuli.toml")["stimulus"]
    HORIZON = commands[-1]["at_ns"] + commands[1]["at_ns"]
    DURATION = str(HORIZON / 1e9)
    main()
