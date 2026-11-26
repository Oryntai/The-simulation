"""Задание «налив»: взять литровую бутылку, взвесить в руке, налить 200 мл в кружку, вернуть бутылку.

Пишет control.toml (контроллер и его программа навыков), profile.toml, case.toml (жидкость в физике
режима) и acceptance.toml. Программа навыков — то, что пишет Claude: последовательность «подойти,
взять, взвесить, налить по весу, поставить». Исполняет её контроллер 250 раз в секунду.
"""
import json, math, os

from make_arm import (BOTTLE_PROFILE, LAYOUTS, MUG_FLOOR, MUG_H, MUG_R_IN, LIMITS, H_YAW, H_SHOULDER, L1,
                      E_ROLL, L2B, TCP, TCP_Z, G, CAM_OPTICAL)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# GRIP_TEST=1: короткое задание «хват» — взять, поднять, взвесить и держать; для отладки схвата.
GRIP_TEST = bool(os.environ.get("GRIP_TEST"))
SUFFIX = "-grip" if GRIP_TEST else ""
MODE = "хват" if GRIP_TEST else "налив"
GRAMS = 200.0
TOLERANCE_ML = 5.0
HORIZON_S = 24.0 if GRIP_TEST else 52.0
GRASP_Z = 0.11          # центр захвата над столом: середина корпуса бутылки
OFFSET = 0.008          # при подходе бутылка у подвижной губы, сжатие сдвигает её к неподвижной
MESSAGE = 262144        # предел сообщения: кадр глубины 203 520 байт и показания
FRAME_AGE = 100_000_000  # кадр не старше 100 мс от начала экспозиции и от доставки
NAMES = ["поворот", "плечо", "локоть", "ротация", "запястье", "вращение"]
DRIVES = ["поворота", "плеча", "локтя", "ротации", "запястья", "кисти"]
# Система налива (её строит контроллер по тому, что увидел): начало — ось бутылки на столе, x — подход
# поперёк линии «бутылка — кружка», y — к кружке, z — высота над столом.
CARRY = {"approach": [1.0, 0.0, 0.0], "closing": [0.0, 1.0, 0.0]}
# Взгляд на стол: камера вниз, на 48 см над столом над серединой рабочей зоны (x = 0.45 м). Оптический
# центр — на 68 мм по оси y ладони от центра захвата (при взгляде вниз это +x мира) и на 57 мм выше.
LOOK = {"p": [0.45 - CAM_OPTICAL[1], 0.0, 0.48 - (TCP_Z - CAM_OPTICAL[2])], "approach": [0.0, 0.0, -1.0],
        "closing": [0.0, 1.0, 0.0], "t": 3.0, "frames": 8}


def program():
    """Программа Claude: где стоят предметы, она не знает — сначала смотрит, потом всё от бутылки."""
    y = -OFFSET
    pre = [-0.12, y, GRASP_Z]
    grasp = [0.0, y, GRASP_Z]
    lifted = [0.0, y, GRASP_Z + 0.09]
    # Ставится на исходное место: после сжатия губ бутылка в центре захвата.
    above = [0.0, 0.0, GRASP_Z + 0.025]
    place = [0.0, 0.0, GRASP_Z + 0.0015]
    move = lambda p, t, joint=False: {"move": {"p": p, **CARRY, "t": t, "joint": joint, "of": "бутылка"}}
    high = [pre[0], pre[1], 0.30]
    # После взгляда — через «дом» (0.30, 0, 0.35): напрямую по суставам из позы «камерой вниз» угол губы
    # проходит сквозь горло бутылки (локоть 37° → 136°, запястье 113° → −66°).
    look = {"look": LOOK}
    home = {"move": {"p": [0.30, 0.0, 0.35], **CARRY, "t": 2.5, "joint": True}}
    if GRIP_TEST:
        return [{"open": {"current": -0.4, "t": 0.3}}, look, home, move(high, 3.0, True), move(pre, 1.5),
                {"tare": {"t": 0.5}}, move(grasp, 1.5), {"grip": {"current": 0.8, "t": 1.0}}, move(lifted, 1.5),
                {"weigh": {"t": 0.6}}, {"wait": {"t": 6.0}}]
    return [
        {"open": {"current": -0.4, "t": 0.3}},
        look,
        home,
        move(high, 3.0, True),
        move(pre, 1.5),
        {"tare": {"t": 0.5}},
        move(grasp, 1.5),
        {"grip": {"current": 0.8, "t": 1.0}},
        move(lifted, 1.5),
        {"weigh": {"t": 0.6}},
        {"pour": {"grams": GRAMS, "head_mm": 10.0, "lead_g": 0.5, "max_s": 25.0}},
        move(above, 3.0, True),
        move(place, 1.5),
        {"open": {"current": -0.4, "t": 0.6}},
        # Сначала в сторону неподвижной губы (−y), прочь от бутылки: её дальняя подушка заходит за
        # середину бутылки и на отходе утащила бы бутылку за собой.
        move([place[0], place[1] - OFFSET, place[2]], 0.6),
        move([pre[0], pre[1] - OFFSET, place[2]], 1.5),
        move([high[0], high[1] - OFFSET, high[2]], 1.5),
        {"move": {"p": [0.30, 0.0, 0.35], **CARRY, "t": 2.0, "joint": True}},
        {"wait": {"t": 0.5}},
    ]


def arm_model():
    """Массы звеньев цепи для упреждения просадки: в железе — по CAD и идентификации."""
    from make_arm import LINKS
    from kinematics import CHAIN, STIFFNESS, HALF_GAP
    by = {l["name"]: l for l in LINKS}
    chain = []
    for name in CHAIN:
        l = by[name]
        j = l["joint"]
        axis = None if not j or j[0] != "hinge" else ("z" if j[2] == [0.0, 0.0, 1.0] else "y")
        m, c, _ = l["mass"]
        chain.append({"pos": l["pos"], "axis": axis, "mass": m, "com": c})
    return {"chain": chain, "stiffness": STIFFNESS, "half_gap": HALF_GAP}


def config():
    return {
        "arm": arm_model(),
        "geometry": {"h0": H_YAW + H_SHOULDER, "l1": L1, "e_roll": E_ROLL, "l2b": L2B, "tcp": TCP,
                     "limits_deg": [list(LIMITS[n]) for n in NAMES]},
        "bottle": {"profile": BOTTLE_PROFILE, "height": 0.27, "radius": 0.04, "mouth_above_grasp": 0.27 - GRASP_Z,
                   "empty_kg": 0.035, "density": 998.2, "discharge": 0.6},
        # Камера — как её знает контроллер: номинал каталога и чертежа, без ошибки крепления экземпляра.
        "camera": {"width": 424, "height": 240, "fx": 223.4, "fy": 223.4, "cx": 212.0, "cy": 120.0,
                   "mount": [CAM_OPTICAL[0], CAM_OPTICAL[1], CAM_OPTICAL[2] - TCP_Z], "min_m": 0.07, "max_m": 0.8},
        # Что известно о предметах: высота среза горла бутылки и пределы высоты кружки.
        "expect": {"bottle_top": 0.27, "mug_top": [0.06, 0.15]},
        **({"dump": os.environ["DUMP"]} if os.environ.get("DUMP") else {}),
        "program": program(),
    }


def control():
    inputs = [(f"рука/энкодер-{n}/value", "rad") for n in NAMES] + [("рука/энкодер-схвата/value", "m")]
    inputs += [(f"рука/сила/{a}", "N") for a in "xyz"] + [(f"рука/момент/{a}", "N*m") for a in "xyz"]
    outputs = [(n, "rad") for n in NAMES] + [("схват", "A"), ("масса", "kg"), ("налито", "kg"), ("наклон", "rad"), ("этап", "1")]
    outputs += [(f"видит {what} {a}", "m") for what in ("бутылку", "кружку") for a in "xy"]
    camera = ("рука/камера/depth", "mm")
    args = [str(MESSAGE), "20000000", "20000000"]
    for name, unit in inputs:
        args += [f"$input:{name}", name, unit]
    args += [f"$input:{camera[0]}", camera[0], camera[1], str(FRAME_AGE), str(FRAME_AGE)]
    for name, unit in outputs:
        args += [f"$output:{name}", name, unit]
    args.append(json.dumps(config(), ensure_ascii=False, separators=(",", ":")))
    q = lambda s: json.dumps(s, ensure_ascii=False)
    lines = ["version = 1", "", "[[controller]]", 'name = "компьютер"', 'kind = "external"', "rate_hz = 250",
             'program = "../../target/release/claude-arm-2-controller"',
             "# Аргументы пишет tools/make_task.py: предел сообщения, сроки, 13 входов, камера, 15 выходов и программа навыков JSON.",
             f"args = [{', '.join(q(a) for a in args)}]",
             "input_latency_ns = 1000000", "input_jitter_ns = 200000", "output_latency_ns = 1000000",
             "output_jitter_ns = 200000", "request_hz = 1000", "request_max_age_ns = 20000000", "queue_capacity = 16",
             f"max_message_bytes = {MESSAGE}",
             'source = "Компьютер с Claude по EtherCAT/CAN через USB: задержки 1 ± 0.2 мс — оценка до замера. '
             'Кадр глубины 424 × 240 × 2 байта идёт в каждом запросе, предел сообщения 256 КиБ."', ""]
    for name, unit in inputs:
        lines += ["[[input]]", 'controller = "компьютер"', f"signal = {q(name)}", f'unit = "{unit}"', 'kind = "scalar"',
                  "max_age_ns = 20000000", 'source = "Опрос по шине 1 кГц, срок пять тактов контроллера."', ""]
    lines += ["[[input]]", 'controller = "компьютер"', f"signal = {q(camera[0])}", f'unit = "{camera[1]}"', 'kind = "depth_mm16"',
              f"max_capture_age_ns = {FRAME_AGE}", f"max_delivery_age_ns = {FRAME_AGE}",
              'source = "Камера глубины D405 по USB: 30 кадров/с, кадр не старше трёх периодов."', ""]
    for (name, unit), drive in zip(outputs[:6], DRIVES):
        lines += ["[[output]]", f'name = "{name}"', 'controller = "компьютер"', f'target = "рука/{name}"',
                  'target_kind = "drive"', f'unit = "{unit}"', f'ack = "рука/привод-{drive}/accepted_command"',
                  "max_age_ns = 20000000", 'fallback = "hold"',
                  'source = "Задание угла драйверу сустава. Без свежей команды драйвер держит последнее."', ""]
    lines += ["[[output]]", 'name = "схват"', 'controller = "компьютер"', 'target = "рука/схват"', 'target_kind = "drive"',
              'unit = "A"', 'ack = "рука/привод-схвата/accepted_command"', "max_age_ns = 20000000", 'fallback = "hold"',
              'source = "Ток схвата — сила сжатия. Без свежей команды драйвер держит последний ток."', ""]
    for name, unit in outputs[7:]:
        lines += ["[[output]]", f'name = "{name}"', 'controller = "компьютер"', f'target = "рука/{name}"',
                  'target_kind = "goal"', f'unit = "{unit}"', "max_age_ns = 20000000",
                  'source = "Что контроллер думает: весы, этап программы и где камера увидела предметы. Только для отчёта."', ""]
    return "\n".join(lines)


def readout(name, unit, formula, ref):
    return ["[[readouts]]", f'signal = {{ name = "{name}", unit = "{unit}" }}', "rate_hz = 1000",
            f'source = {{ kind = "derived", reference = "{ref}" }}', f"formula = {formula}", ""]


def profile():
    lines = ["version = 1", f'required_modes = ["{MODE}"]',
             "# Координаты и скорости MuJoCo в журнал раз в 1 мс (DEFECTS.md руки 1, Д8).", "joint_state_hz = 1000",
             "# Ряды истины приводов окнами 1 мс: первое значение, минимум и максимум (DEFECTS.md, Д12).", "truth_hz = 1000", "",
             "# Вода живёт в физике режима: процесс «вода» ведёт объёмы, струю и пролив.", "[mode_physics]",
             f'"{MODE}" = {{ case = "case{SUFFIX}.toml", witness = "бутылка/подъём", journal_hz = 1000 }}', ""]
    for i, n in enumerate(NAMES):
        lines += readout(f"угол {n}", "rad", f'{{ kind = "position", index = {i} }}', "Истинная координата сустава из MuJoCo.")
    lines += readout("губа", "m", '{ kind = "position", index = 6 }', "Истинный ход подвижной губы.")
    for body, label in (("бутылка/корпус", "бутылка"), ("кружка/корпус", "кружка")):
        for axis, a in enumerate("xyz"):
            lines += readout(f"{label} {a}", "m", f'{{ kind = "body_point", body = "{body}", local_m = [0.0, 0.0, 0.0], axis = {axis} }}',
                             "Центр дна снаружи из позы тела MuJoCo.")
    for n in NAMES:
        lines += [f'[paths."{n}"]', f'observation = {{ name = "рука/энкодер-{n}/value", unit = "rad" }}',
                  f'physical_response = [{{ name = "угол {n}", unit = "rad" }}]', "command_delta = 0.01",
                  "max_observation_age_ns = 20000000",
                  'source = { kind = "derived", reference = "0.01 рад много больше шага энкодера 0.00038 рад." }', "hops = [",
                  '  { target = "transport:компьютер/request", direction = "input", delay_ns = 1000000, jitter_ns = 200000, receiver_tick_ns = 1000000, source = { kind = "derived", reference = "control.toml, такт получателя 1 кГц" } },',
                  '  { target = "transport:компьютер/response", direction = "output", delay_ns = 1000000, jitter_ns = 200000, receiver_tick_ns = 4000000, source = { kind = "derived", reference = "control.toml, такт получателя 250 Гц" } },',
                  "]", ""]
    lines += ['[paths."схват"]', 'observation = { name = "рука/энкодер-схвата/value", unit = "m" }',
              'physical_response = [{ name = "губа", unit = "m" }]', "command_delta = 0.05", "max_observation_age_ns = 20000000",
              'source = { kind = "derived", reference = "Ток схвата 0.05 А — около 2 Н сжатия." }', "hops = [",
              '  { target = "transport:компьютер/request", direction = "input", delay_ns = 1000000, jitter_ns = 200000, receiver_tick_ns = 1000000, source = { kind = "derived", reference = "control.toml, такт получателя 1 кГц" } },',
              '  { target = "transport:компьютер/response", direction = "output", delay_ns = 1000000, jitter_ns = 200000, receiver_tick_ns = 4000000, source = { kind = "derived", reference = "control.toml, такт получателя 250 Гц" } },',
              "]", ""]
    for axis, a in enumerate("xyz"):
        lines += readout(f"горло {a}", "m", f'{{ kind = "body_point", body = "бутылка/корпус", local_m = [0.0, 0.0, 0.27], axis = {axis} }}',
                         "Центр среза горла из позы тела MuJoCo.")
    lines += readout("бутылка ось z", "m", '{ kind = "body_point", body = "бутылка/корпус", local_m = [0.0, 0.0, 1.0], axis = 2 }',
                     "Точка на оси бутылки в 1 м от дна: высота над дном — косинус наклона.")
    return "\n".join(lines)


MATERIALS = ["сталь", "алюминий", "резина", "ПЭТ", "керамика", "дерево"]
# Трение скольжения пар — справочный порядок величины, не замер этих поверхностей.
FRICTION = {("резина", "ПЭТ"): 0.9, ("резина", "керамика"): 0.9, ("резина", "дерево"): 0.9, ("ПЭТ", "дерево"): 0.4,
            ("керамика", "дерево"): 0.5, ("ПЭТ", "керамика"): 0.3, ("резина", "резина"): 1.0}


def assignments():
    """Материал каждой геометрии столкновений: имена как в скомпилированном мире."""
    from make_arm import LINKS
    out = [("environment/collision/0", "дерево")]
    for link in LINKS:
        for i, (kind, *_rest) in enumerate(link["collisions"]):
            material = "резина" if kind == "sphere" else ("сталь" if link["name"] == "стойка" else "алюминий")
            out.append((f"рука/{link['name']}/collision/{i}", material))
    out += [(f"бутылка/корпус/collision/{i}", "ПЭТ") for i in range(3)]
    out += [(f"кружка/корпус/collision/{i}", "керамика") for i in range(18)]
    return out


def calibrations():
    """Пары-кандидаты из примера `contact_candidates`; импеданс общий, оценка."""
    import subprocess
    sim = os.path.abspath(os.path.join(ROOT, "..", ".."))
    out = subprocess.run([os.path.join(sim, "target", "debug", "examples", "contact_candidates"), os.path.join(ROOT, "world.toml")],
                         capture_output=True, text=True, check=True, cwd=sim).stdout
    lines = []
    for row in out.strip().split("\n"):
        a, b = row.split("\t")
        lines += ["[[contacts.pairs]]", f'geometries = ["{a}", "{b}"]', "solimp = [0.95, 0.99, 0.001, 0.5, 2.0]",
                  'source = { estimate = "Импеданс близок к жёсткому контакту; пара не идентифицирована." }', ""]
    return lines


def materials():
    """Таблица пар материалов: прямая жёсткость контакта и трение скольжения."""
    q = lambda v, why: f'{{ value = {v}, source = {{ estimate = "{why}" }} }}'
    lines = ["# Пары материалов контакта. Пишет tools/make_task.py. Всё — оценки до замера пар.", "version = 1", ""]
    for i, a in enumerate(MATERIALS):
        for b in MATERIALS[i:]:
            mu = FRICTION.get((a, b), FRICTION.get((b, a), 0.4))
            lines += ["[[entries]]", f'materials = ["{a}", "{b}"]',
                      f"sliding = [{q(mu, 'Справочный порядок трения пары, не замер.')}, {q(mu, 'Справочный порядок трения пары, не замер.')}]", "",
                      "[entries.normal]", 'kind = "direct"',
                      f"stiffness_per_s2 = {q(160000.0, 'Контакт ω = 400 рад/с: жёстче шага 30 кГц не нужно; нужна идентификация пары.')}",
                      f"damping_per_s = {q(800.0, 'Критическое затухание ζ = 1 при ω = 400 рад/с; нужна идентификация пары.')}", ""]
    return "\n".join(lines)


def case():
    horizon = int(HORIZON_S * 1e9)
    bottle = {"body": "бутылка/корпус", "bottom_m": [0.0, 0.0, 0.001], "axis": [0.0, 0.0, 1.0], "profile_m": BOTTLE_PROFILE,
              "initial_ml": 1000.0, "rigid_initial": True}
    mug = {"body": "кружка/корпус", "bottom_m": [0.0, 0.0, MUG_FLOOR], "axis": [0.0, 0.0, 1.0],
           "profile_m": [[0.0, MUG_R_IN], [round(MUG_H - MUG_FLOOR, 6), MUG_R_IN]], "initial_ml": 0.0}
    t = lambda v: json.dumps(v)
    return "\n".join([
        f"# Физика режима «{MODE}»: вода в бутылке и кружке. Пишет tools/make_task.py.",
        f"window_ns = [0, {horizon}]",
        'source = "Полный горизонт режима «налив»."', "",
        '[models."вода"]', 'kind = "liquid"',
        f"gravity_m_s2 = [0.0, 0.0, {-G}]",
        'source = "Квазистатическая жидкость (crates/sim-cli/src/tasks/processes/models/liquid.rs): плоская поверхность по эффективной тяжести, '
        'Торричелли через срез горла, баллистическая струя, эффект чайника. Волны, брызги, капли на стенках и инерция перетекающей воды не посчитаны."',
        *([] if GRIP_TEST else [f'goal = {{ vessel = "кружка", ml = {GRAMS / 0.9982:.3f}, tolerance_ml = {TOLERANCE_ML}, at_ns = {horizon - 1_000_000_000} }}']), "",
        '[models."вода".fluid]',
        'density_kg_m3 = { value = 998.2, source = { datasheet = "Вода при 20 °C, справочная плотность." } }',
        'discharge = { value = 0.6, source = { estimate = "Коэффициент расхода острой кромки около 0.6–0.62; кромка горла ПЭТ не замерена." } }',
        'detach_m_s = { value = 0.15, source = { estimate = "Порог отрыва струи от кромки (эффект чайника) — порядок величины, не замер на этом горле." } }',
        'slosh_s = { value = 0.047, source = { estimate = "1/ω первой моды плеска в цилиндре R = 39.5 мм: ω² = 1.841·g/R." } }', "",
        '[models."вода".vessels."бутылка"]',
        *[f"{k} = {t(v)}" for k, v in bottle.items()],
        'glug_ml_s = { value = 100.0, source = { estimate = "Перевёрнутая литровая бутылка опорожняется толчками около 10 с; не замер." } }', "",
        '[models."вода".vessels."кружка"]',
        *[f"{k} = {t(v)}" for k, v in mug.items()], "",
        "# Решатель контактов: noslip убирает численную ползучесть мягкого трения MuJoCo — бутылка",
        "# под постоянным весом сползала в схвате на 5 мм/с при шестикратном запасе по трению.",
        "[contacts]", 'table = "contact-materials.toml"', "",
        "[contacts.solver]", 'method = "newton"', "elliptic = true", "impratio = 1.0", "tolerance = 1e-8", "iterations = 100",
        "line_search_tolerance = 0.01", "line_search_iterations = 50", "noslip_tolerance = 1e-6", "noslip_iterations = 20",
        "ccd_tolerance = 1e-6", "ccd_iterations = 50", "",
        "[contacts.assignments]", *[f'"{g}" = "{m}"' for g, m in assignments()], "",
        "# Калибровки всех пар, которые MuJoCo может столкнуть (пример contact_candidates): импеданс — оценка.",
        *calibrations(),
        "# Свидетель режима: бутылка поднята над столом.",
        "[[criteria]]", 'name = "бутылка/подъём"', 'unit = "m"', "series = { position = 9 }", 'statistic = "maximum"',
        'tolerance = { value = 0.001, source = { estimate = "Проектный допуск численного опыта." } }',
        'remainder = [{ value = 0.0, source = { estimate = "Остаток за пределами двух наблюдаемых разностей не оценён." } }, '
        '{ value = 0.0, source = { estimate = "Остаток проверки решателя не оценён." } }, '
        '{ value = 0.0, source = { estimate = "Разброс материалов требует стенда." } }]',
        'lower = { value = 0.05, source = { estimate = "Бутылка поднята для налива." } }', ""])


def acceptance(layout=""):
    """Приёмка на раскладку стола: программа одна, где кружка — знает только приёмка."""
    _, MUG_AT, _ = LAYOUTS[layout]
    horizon = int(HORIZON_S * 1e9)
    src = lambda ref: f'source = {{ kind = "estimate", reference = "{ref}" }}'
    common = [
        ('{ kind = "range", signal = { name = "вода/мимо", unit = "ml" }, min = 0.0, max = 0.05, '
         + src("Ни капли мимо: пролито меньше одной капли, 0.05 мл.") + " }"),
        ('{ kind = "range", signal = { name = "вода/кружка", unit = "ml" }, min = 0.0, max = 205.0, '
         + src("Кружку не переливать: не больше 200 + 5 мл.") + " }"),
        *([] if GRIP_TEST else [('{ kind = "forbidden", signal = { name = "вода/цель", unit = "1" }, '
         + src(f"К концу прогона в кружке 200 ± {TOLERANCE_ML:g} мл: процесс «вода» поднимает событие, если нет.") + " }")]),
        ('{ kind = "range", signal = { name = "вода/поток", unit = "ml/s" }, min = 0.0, max = 1000.0, '
         + src("Расход из бутылки — для отчёта; предел заведомо широкий.") + " }"),
        ('{ kind = "range", signal = { name = "горло z", unit = "m" }, min = -0.01, max = 1.0, '
         + src("Высота среза горла — для отчёта; предел заведомо широкий.") + " }"),
        ('{ kind = "range", signal = { name = "бутылка z", unit = "m" }, min = -0.002, max = 0.5, '
         + src("Бутылка не проваливается в стол и не улетает.") + " }"),
        ('{ kind = "range", signal = { name = "бутылка ось z", unit = "m" }, min = -1.0, max = 1.5, '
         + src("Наклон бутылки — для отчёта: точка оси в 1 м над дном.") + " }"),
        ('{ kind = "range", signal = { name = "бутылка x", unit = "m" }, min = 0.0, max = 0.8, '
         + src("Бутылка над столом; где она встала в конце — по последнему значению.") + " }"),
        ('{ kind = "range", signal = { name = "бутылка y", unit = "m" }, min = -0.5, max = 0.5, '
         + src("Бутылка над столом; где она встала в конце — по последнему значению.") + " }"),
        ('{ kind = "range", signal = { name = "кружка x", unit = "m" }, min = ' + f"{MUG_AT[0] - 0.003}, max = {MUG_AT[0] + 0.003}, "
         + src("Кружку не сдвинули больше чем на 3 мм.") + " }"),
        ('{ kind = "range", signal = { name = "кружка y", unit = "m" }, min = ' + f"{MUG_AT[1] - 0.003}, max = {MUG_AT[1] + 0.003}, "
         + src("Кружку не сдвинули больше чем на 3 мм.") + " }"),
        ('{ kind = "range", signal = { name = "кружка z", unit = "m" }, min = -0.002, max = 0.003, '
         + src("Кружку не подняли и не опрокинули.") + " }"),
    ]
    for drive in DRIVES[:3]:
        common.append('{ kind = "range", signal = { name = "рука/привод-' + drive + '/winding_c", unit = "degC" }, min = 0.0, max = 100.0, '
                      + src("Обмотка maxon ниже 100 °C при пределе 125 °C (каталог).") + " }")
    lines = ["version = 1", f'control = "control{SUFFIX}.toml"', f'profile = "profile{SUFFIX}.toml"', "", "[[modes]]", f'id = "{MODE}"',
             f"horizon_ns = {horizon}", "before_ns = 100000000", "after_ns = 300000000", "max_gap_ns = 2000000",
             'source = { kind = "derived", reference = "Взять литровую бутылку воды, взвесить в руке, налить 200 мл в кружку, вернуть бутылку на место." }',
             "# Толчок по плечу в конце: рука после работы держит позу.", 'excitation = "толчок"',
             f'stimuli = [{{ id = "толчок", signal = {{ name = "рука/толчок-плеча", unit = "N*m" }}, at_ns = {horizon - 400_000_000}, value = 1.0, '
             'comparison_value = 0.0, source = { kind = "estimate", reference = "Касание рукой человека." } }]', ""]
    for n in NAMES:
        lo, hi = (math.radians(v) for v in LIMITS[n])
        limits = [f'{{ kind = "range", signal = {{ name = "угол {n}", unit = "rad" }}, min = {lo}, max = {hi}, '
                  + src("Рабочий диапазон сустава по эскизу.") + " }"] + common
        lines += ["[[criteria]]", f'case = {{ path = "{n}", mode = "{MODE}" }}', f"limits = [{', '.join(limits)}]", ""]
    jaw = ['{ kind = "range", signal = { name = "губа", unit = "m" }, min = -0.0005, max = 0.0305, '
           + src("Ход губы в пределах рейки.") + " }"] + common
    lines += ["[[criteria]]", f'case = {{ path = "схват", mode = "{MODE}" }}', f"limits = [{', '.join(jaw)}]", ""]
    return "\n".join(lines)


def main():
    files = {f"control{SUFFIX}.toml": control(), f"profile{SUFFIX}.toml": profile(), f"case{SUFFIX}.toml": case(),
             **{f"acceptance{SUFFIX}{'-' + k if k else ''}.toml": acceptance(k) for k in LAYOUTS},
             "contact-materials.toml": materials()}
    for name, text in files.items():
        open(os.path.join(ROOT, name), "w").write(text)
    print(f"записано: {', '.join(files)}; горизонт {HORIZON_S} с")


if __name__ == "__main__":
    main()
