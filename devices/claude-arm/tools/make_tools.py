"""Пишет мир сменных насадок world-tools.toml и два задания смены: «щуп» и «маркер».

Для каждой насадки — control-, profile-, case- и acceptance-<насадка>.toml. Рука берёт насадку из
держателя, работает ею и ставит на место. Щуп касается двух точек стола, маркер проводит линию.
Замком сменщика командует контроллер. На столе всегда обе насадки: сменщик сам выбирает ту, что под
фланцем. Задания разделены, потому что рука считается на 240 кГц и журнал прогона приводов растёт
примерно на 0.47 ГБ за секунду модели (DEFECTS.md, Д8).
"""
import json, math, os
from kinematics import inverse, H0, A1, A2, D, LIMITS_DEG
from make_visuals import table_visuals, holder_visuals

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
JOINTS = [("поворот", "энкодер-поворота", "серва-поворота"), ("плечо", "энкодер-плеча", "серва-плеча"),
          ("локоть", "энкодер-локтя", "серва-локтя"), ("запястье", "энкодер-запястья", "серва-запястья"),
          ("вращение", "энкодер-вращения", "серва-вращения")]

# Насадки: метка файлов, файл устройства, держатель (x, y), вылет от фланца до центра наконечника,
# радиус наконечника, поджатие при касании (цель ниже поверхности стола), индекс высоты точки
# крепления в qpos (рука — 5 координат, насадки по 7).
TOOLS = {
    "щуп": dict(tag="probe", file="tool-probe.toml", hold=(0.16, -0.08), length=0.045, tip=0.003, press=0.001, z_index=7),
    "маркер": dict(tag="marker", file="tool-marker.toml", hold=(0.16, 0.08), length=0.055, tip=0.0015, press=0.001, z_index=14),
}
# Линия маркера: 60 мм за 2.5 с и выдержка 0.6 с в конце. Серва в модели следит за движущейся
# целью с запаздыванием 0.2–0.27 с (tools/measure_touch.py): за 1.2 с узел отставал от линии
# на 19–30 мм и не доходил до конца до подъёма.
STROKE_S, STROKE_HOLD_S = 2.5, 0.6
PARK = 0.075  # высота фланца и точки крепления насадки в держателе
COLLAR = 0.011  # от точки крепления до низа воротника
UP = 0.14  # высота переездов: наконечник выше вилки больше чем на 2 см
HOVER = 0.02  # высота наконечника над столом перед касанием
# Допуски замка: зазор 2 мм и угол 2° — заходные фаски хвостовика (оценка). Скорость и вращение
# такие, чтобы за срабатывание защёлки, около 50 мс (оценка), стык ушёл не дальше половины зазора
# и угла: 2 мм / 0.1 с = 20 мм/с, 0.035 рад / 0.1 с = 0.35 рад/с. Серва в мёртвой зоне подрабатывает:
# у держателя через 0.5 с после остановки фланец ходит до 7 мм/с, кисть до 0.14 рад/с.
LATCH_S = 0.05
BOUNDS = [0.002, 0.035, round(0.002 / (2 * LATCH_S), 6), round(0.035 / (2 * LATCH_S), 6)]
# Касания щупа и линия маркера, координаты стола.
TOUCHES = [(0.21, -0.02), (0.21, 0.02)]
LINE = ((0.21, -0.03), (0.21, 0.03))


def tip_flange(tool, x, y, tip_z):
    """Фланец над точкой, где центр наконечника должен быть на высоте tip_z."""
    return (x, y, tip_z + TOOLS[tool]["length"])


def plan(tool):
    """Точки (t, x, y, z фланца, замок) задания насадки. Замок точки действует с её времени."""
    out, t = [], 0.0
    x, y = TOOLS[tool]["hold"]
    r = TOOLS[tool]["tip"]
    press = TOOLS[tool]["press"]

    def go(dt, p, lock):
        nonlocal t
        t = round(t + dt, 6)
        out.append((t, *p, lock))

    def dock(lock_after):
        """Опуститься к держателю, выдержать, переключить замок, выдержать, подняться."""
        before = 1 - lock_after
        go(0.3, (x, y, UP), before)
        go(0.6, (x, y, PARK), before)
        go(0.5, (x, y, PARK), before)  # успокоение перед замком
        out[-1] = (*out[-1][:4], lock_after)  # замок переключается в конце выдержки
        go(0.3, (x, y, PARK), lock_after)
        go(0.6, (x, y, UP), lock_after)

    go(1.2, (x, y, UP), 0)  # из вертикали по суставам
    dock(1)
    if tool == "щуп":
        for px, py in TOUCHES:
            go(0.6, tip_flange(tool, px, py, HOVER), 1)
            go(0.1, tip_flange(tool, px, py, HOVER), 1)
            go(0.3, tip_flange(tool, px, py, r - press), 1)
            go(0.3, tip_flange(tool, px, py, r - press), 1)
            go(0.3, tip_flange(tool, px, py, HOVER), 1)
    else:
        (ax, ay), (bx, by) = LINE
        go(0.7, tip_flange(tool, ax, ay, HOVER), 1)
        go(0.1, tip_flange(tool, ax, ay, HOVER), 1)
        go(0.3, tip_flange(tool, ax, ay, r - press), 1)
        go(0.2, tip_flange(tool, ax, ay, r - press), 1)
        go(STROKE_S, tip_flange(tool, bx, by, r - press), 1)  # линия 60 мм
        go(STROKE_HOLD_S, tip_flange(tool, bx, by, r - press), 1)
        go(0.3, tip_flange(tool, bx, by, HOVER), 1)
    go(0.7, (x, y, UP), 1)
    dock(0)
    go(0.3, (x, y, UP), 0)
    return out


def check(points):
    """Те же проверки, что делает контроллер: каждая точка и каждый отрезок через 1 % пути."""
    for (t0, *a), (t1, *b) in zip(points, points[1:]):
        for k in range(101):
            s = k / 100
            p = [a[i] + s * (b[i] - a[i]) for i in range(3)]
            assert inverse(*p), f"{t0}–{t1} с: точка {p} вне досягаемости"


def quat_down(x, y):
    """Поза фланца над (x, y) при наклоне 180° и вращении 0: Rz(atan2(y, x))·Ry(180°)."""
    h = math.atan2(y, x) / 2
    return [0.0, -math.sin(h), math.cos(h), 0.0]


def world():
    w = ['# Рука Claude, два держателя-вилки и две насадки. Пишет tools/make_tools.py.', 'format = 3',
         'name = "Рука Claude со сменщиком насадок"', 'seed = 8',
         '# Как в world.toml: сверка привода на шаге толчка требует 240 кГц (DEFECTS.md, Д5).', 'physics_hz = 240000',
         'gravity_m_s2 = [0.0, 0.0, -9.80665]', '', '[environment]', 'density_kg_m3 = 0.0', 'viscosity_pa_s = 0.0',
         'wind_m_s = [0.0, 0.0, 0.0]', '', '[[environment.collision]]', 'shape = { kind = "box", size_m = [1.2, 0.8, 0.04] }',
         'pose = { position_m = [0.15, 0.0, -0.02] }', 'friction = [0.8, 0.005, 0.0001]', '']
    top = PARK - COLLAR
    for name, tool in TOOLS.items():
        x, y = tool["hold"]
        w.append(f'# Вилка держателя «{name}»: два бруска 30×8 мм, щель 16 мм, верх на {top * 1000:.0f} мм.')
        for side in (-1, 1):
            w += ['[[environment.collision]]', f'shape = {{ kind = "box", size_m = [0.03, 0.008, {top}] }}',
                  f'pose = {{ position_m = [{x}, {round(y + side * 0.012, 6)}, {top / 2}] }}',
                  'friction = [0.8, 0.005, 0.0001]', '']
    w += table_visuals()
    for name, tool in TOOLS.items():
        w += holder_visuals(*tool["hold"], top)
    w += ['[[devices]]', 'name = "рука"', 'file = "device.toml"', 'pose = { position_m = [0.0, 0.0, 0.0] }', '']
    for name, tool in TOOLS.items():
        x, y = tool["hold"]
        q = ", ".join(f"{v:.12f}" for v in quat_down(x, y))
        w += [f'# «{name}» стоит в держателе воротником на вилке, повёрнут как фланец над ним.', '[[devices]]',
              f'name = "{name}"', f'file = "{tool["file"]}"',
              f'pose = {{ position_m = [{x}, {y}, {PARK}], quat_wxyz = [{q}] }}', '',
              '[[attachments]]', f'name = "{name}"', 'first = "рука/кисть/фланец"', f'second = "{name}/основание/фланец"',
              'mode = "runtime"', '']
    return "\n".join(w)


def control(tool, points):
    tag = TOOLS[tool]["tag"]
    args = ["65536", "20000000", "20000000"]
    for j, e, s in JOINTS:
        args += [f"$input:рука/{e}/value", f"рука/{e}/value", "rad"]
    for j, e, s in JOINTS:
        args += [f"$output:{j}", j, "rad"]
    args += ["2", "$output:замок", "замок", "1"]
    args += [str(H0), str(A1), str(A2), str(D)]
    for lo, hi in LIMITS_DEG:
        args += [str(lo), str(hi)]
    args += [str(len(points))]
    for t, x, y, z, lock in points:
        args += [str(t), str(x), str(y), str(round(z, 6)), "180", "0", str(lock)]
    args += ["прямые"]
    c = ['version = 1', '', '[[controller]]', 'name = "компьютер"', 'kind = "external"', 'rate_hz = 100',
         '# Собирается в общую target симулятора: см. README.',
         'program = "../../target/debug/claude-arm-controller"',
         '# Аргументы пишет tools/make_tools.py: как в control.toml, плюс выход замка (2) с командой 0/1 в каждой точке',
         '# и режим «прямые»: фланец идёт между точками по отрезку.',
         'args = ' + json.dumps(args, ensure_ascii=False),
         'input_latency_ns = 2000000', 'input_jitter_ns = 500000', 'output_latency_ns = 1000000', 'output_jitter_ns = 500000',
         'request_hz = 1000', 'request_max_age_ns = 20000000', 'queue_capacity = 16', 'max_message_bytes = 65536',
         'source = "Компьютер с Claude шлёт задания углов и команду замка по шине 1 Мбит/с через USB-UART; задержки — оценка до замера."']
    for j, e, s in JOINTS:
        c += ['', '[[input]]', 'controller = "компьютер"', f'signal = "рука/{e}/value"', 'unit = "rad"', 'kind = "scalar"',
              'max_age_ns = 20000000', 'source = "Опрос энкодера сервы по шине, срок два такта по 10 мс."']
    for j, e, s in JOINTS:
        c += ['', '[[output]]', f'name = "{j}"', 'controller = "компьютер"', f'target = "рука/{j}"', 'target_kind = "drive"',
              'unit = "rad"', f'ack = "рука/{s}/accepted_command"', 'max_age_ns = 20000000',
              '# Шинная серва при пропуске команды держит последнее принятое задание.',
              'fallback = "hold"', 'source = "Задание угла сервы. Резерв — последнее ушедшее задание."']
    c += ['', f'# Команда замка сменщика насадок: 1 — закрыть, 0 — открыть. Её читает процесс «сменщик» (case-{tag}.toml).',
          '[[output]]', 'name = "замок"', 'controller = "компьютер"', 'target = "рука/замок"', 'target_kind = "goal"',
          'unit = "1"', 'max_age_ns = 20000000',
          'source = "Клапан замка на той же шине. Без свежей команды замок остаётся в прежнем положении."']
    return "\n".join(c) + "\n"


def profile(tool):
    tag = TOOLS[tool]["tag"]
    hops = ('hops = [\n  { target = "transport:компьютер/request", direction = "input", delay_ns = 2000000, jitter_ns = 500000, '
            f'receiver_tick_ns = 1000000, source = {{ kind = "derived", reference = "control-{tag}.toml, такт получателя 1 кГц" }} }},\n'
            '  { target = "transport:компьютер/response", direction = "output", delay_ns = 1000000, jitter_ns = 500000, '
            f'receiver_tick_ns = 10000000, source = {{ kind = "derived", reference = "control-{tag}.toml, такт получателя 100 Гц" }} }},\n]')
    p = ['version = 1', f'required_modes = ["{tool}"]',
         '# Координаты и скорости MuJoCo в журнал раз в 1 мс, а не на каждом из 240 000 шагов в секунду (DEFECTS.md, Д8).',
         'joint_state_hz = 1000', '',
         '# Сменщик насадок и его замок живут в физике режима: процесс читает команду замка с шины.',
         '# Журнал процессов раз в 1 мс: на 240 кГц сплошной журнал шагов не помещается в память (DEFECTS.md, Д8).',
         '[mode_physics]', f'"{tool}" = {{ case = "case-{tag}.toml", witness = "{tool}/подъём", journal_hz = 1000 }}', '']
    for i, (j, e, s) in enumerate(JOINTS):
        p += ['[[readouts]]', f'signal = {{ name = "угол {j}", unit = "rad" }}', 'rate_hz = 1000',
              'source = { kind = "derived", reference = "Истинная координата сустава из MuJoCo." }',
              f'formula = {{ kind = "position", index = {i} }}', '']
    for name, t in TOOLS.items():
        for axis, a in enumerate("xyz"):
            p += ['[[readouts]]', f'signal = {{ name = "{name} {a}", unit = "m" }}', 'rate_hz = 1000',
                  'source = { kind = "derived", reference = "Точка крепления насадки из позы тела MuJoCo." }',
                  f'formula = {{ kind = "body_point", body = "{name}/основание", local_m = [0.0, 0.0, 0.0], axis = {axis} }}', '']
        for axis, a in enumerate("xyz"):
            p += ['[[readouts]]', f'signal = {{ name = "{name} кончик {a}", unit = "m" }}', 'rate_hz = 1000',
                  'source = { kind = "derived", reference = "Центр наконечника из позы тела MuJoCo." }',
                  f'formula = {{ kind = "body_point", body = "{name}/основание", local_m = [0.0, 0.0, {t["length"]}], axis = {axis} }}', '']
    for j, e, s in JOINTS:
        p += [f'[paths."{j}"]', f'observation = {{ name = "рука/{e}/value", unit = "rad" }}',
              f'physical_response = [{{ name = "угол {j}", unit = "rad" }}]', 'command_delta = 0.01',
              'max_observation_age_ns = 20000000',
              'source = { kind = "derived", reference = "0.01 рад больше шага энкодера 0.0015 рад." }', hops, '']
    return "\n".join(p)


def case(tool, horizon):
    stations = "\n".join(
        f'"{name}" = {{ tool = {{ body = "{name}/основание", local_m = [0.0, 0.0, 0.0] }}, '
        f'parking_m = [{t["hold"][0]}, {t["hold"][1]}, {PARK}] }}' for name, t in TOOLS.items())
    remainder = ('remainder = [{ value = 0.0, source = { estimate = "Остаток за пределами двух наблюдаемых разностей не оценён." } }, '
                 '{ value = 0.0, source = { estimate = "Остаток проверки решателя не оценён." } }, '
                 '{ value = 0.0, source = { estimate = "Разброс материалов требует стенда." } }]')
    return "\n".join([
        f'# Физика режима «{tool}»: сменщик насадок с замком от контроллера. Пишет tools/make_tools.py.',
        f'window_ns = [0, {horizon}]', f'source = "Полный горизонт режима «{tool}»."', '',
        '[models."сменщик"]', 'kind = "coupler"', 'carrier = { body = "рука/кисть", local_m = [0.0, 0.0, 0.03] }',
        '# Идеальный замок: стык замыкается мгновенно, если насадка в держателе под фланцем.',
        'source = "Эскиз замка: допуски совпадения 2 мм и 2° по фаскам, скорость и вращение из срабатывания защёлки за 50 мс. Механика защёлки и податливость стыка не замерены."',
        f'bounds = {json.dumps(BOUNDS)}', 'lock = { signal = "рука/замок", max_age_ns = 20000000 }', '',
        '[models."сменщик".stations]', stations, '',
        f'# Свидетель режима: {tool} поднят из держателя.', '[[criteria]]', f'name = "{tool}/подъём"', 'unit = "m"',
        f'series = {{ position = {TOOLS[tool]["z_index"]} }}', 'statistic = "maximum"',
        'tolerance = { value = 0.001, source = { estimate = "Проектный допуск численного опыта." } }', remainder,
        f'lower = {{ value = {UP - 0.01}, source = {{ estimate = "Насадка поднята на высоту переездов." }} }}']) + "\n"


def acceptance(tool, end, horizon):
    tag = TOOLS[tool]["tag"]
    what = "коснуться двух точек стола" if tool == "щуп" else "провести линию 60 мм"
    a = ['version = 1', f'control = "control-{tag}.toml"', f'profile = "profile-{tag}.toml"', '', '[[modes]]', f'id = "{tool}"',
         f'horizon_ns = {horizon}', 'before_ns = 100000000', 'after_ns = 300000000', 'max_gap_ns = 2000000',
         f'source = {{ kind = "derived", reference = "Взять {tool} из держателя, {what}, поставить на место." }}',
         '# Лёгкий толчок по плечу после отхода: проверка, что рука держит позу.',
         'excitation = "толчок"', 'stimuli = [',
         f'  {{ id = "толчок", signal = {{ name = "рука/толчок-плеча", unit = "N*m" }}, at_ns = {int(round((end + 0.1) * 1e9))}, value = 0.1, comparison_value = 0.0, '
         'source = { kind = "estimate", reference = "Касание рукой человека." } },', ']', '']
    tool_limits = []
    for name, t in TOOLS.items():
        tool_limits += [
            f'{{ kind = "range", signal = {{ name = "{name} кончик z", unit = "m" }}, min = {t["tip"] - 0.002}, max = 0.3, '
            'source = { kind = "estimate", reference = "Наконечник не входит в стол глубже 2 мм и не улетает." } }',
            f'{{ kind = "range", signal = {{ name = "{name} x", unit = "m" }}, min = 0.0, max = 0.4, '
            'source = { kind = "estimate", reference = "Насадка остаётся над столом перед рукой." } }',
            f'{{ kind = "range", signal = {{ name = "{name} y", unit = "m" }}, min = -0.2, max = 0.2, '
            'source = { kind = "estimate", reference = "Насадка остаётся над столом перед рукой." } }',
            f'{{ kind = "range", signal = {{ name = "{name} z", unit = "m" }}, min = 0.0, max = 0.3, '
            'source = { kind = "estimate", reference = "Насадка не падает со стола и не улетает." } }']
    for (j, e, s), (lo, hi) in zip(JOINTS, LIMITS_DEG):
        lim = (f'{{ kind = "range", signal = {{ name = "угол {j}", unit = "rad" }}, min = {math.radians(lo)}, max = {math.radians(hi)}, '
               'source = { kind = "estimate", reference = "Рабочий диапазон сустава по эскизу." } }')
        a += ['[[criteria]]', f'case = {{ path = "{j}", mode = "{tool}" }}', f'limits = [{", ".join([lim] + tool_limits)}]', '']
    return "\n".join(a)


def main():
    open(os.path.join(ROOT, 'world-tools.toml'), 'w').write(world())
    for tool, t in TOOLS.items():
        points = plan(tool)
        check(points)
        end = points[-1][0]
        horizon = int(round((end + 0.5) * 1e9))
        files = {f'control-{t["tag"]}.toml': control(tool, points), f'profile-{t["tag"]}.toml': profile(tool),
                 f'case-{t["tag"]}.toml': case(tool, horizon), f'acceptance-{t["tag"]}.toml': acceptance(tool, end, horizon)}
        for name, text in files.items():
            open(os.path.join(ROOT, name), 'w').write(text)
        print(f"«{tool}»: точек {len(points)}, горизонт {horizon / 1e9} с, файлы {', '.join(files)}")


if __name__ == "__main__":
    main()
