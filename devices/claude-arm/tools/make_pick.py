"""Пишет control-pick.toml, profile-pick.toml и acceptance-pick.toml: перенос кубика схватом."""
import json, math, os
from kinematics import inverse, H0, A1, A2, D_GRIP, LIMITS_DEG

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
JOINTS = [("поворот", "энкодер-поворота", "серва-поворота"), ("плечо", "энкодер-плеча", "серва-плеча"),
          ("локоть", "энкодер-локтя", "серва-локтя"), ("запястье", "энкодер-запястья", "серва-запястья"),
          ("вращение", "энкодер-вращения", "серва-вращения")]
OPEN, CLOSE = 0.0, 0.016  # ход губы, м: касание кубика 25 мм на 9 мм, прижим к неподвижной губе на 12 мм
PICK, PLACE = (0.16, 0.06), (0.16, -0.06)
HIGH, LOW = 0.08, 0.020  # высота точки захвата: над кубиком и у его середины, кончики губ в 2 мм над столом
# (время, x, y, z, ход губы): переезды 0.8 с, выдержки 0.3–0.5 с.
PLAN = [
    (1.0, *PICK, HIGH, OPEN), (1.5, *PICK, HIGH, OPEN),
    (2.3, *PICK, LOW, OPEN), (2.6, *PICK, LOW, OPEN),
    (3.2, *PICK, LOW, CLOSE), (3.5, *PICK, LOW, CLOSE),
    (4.3, *PICK, HIGH, CLOSE), (5.3, *PLACE, HIGH, CLOSE),
    (6.1, *PLACE, LOW, CLOSE), (6.4, *PLACE, LOW, CLOSE),
    (6.9, *PLACE, LOW, OPEN), (7.2, *PLACE, LOW, OPEN),
    (8.0, *PLACE, HIGH, OPEN), (8.5, *PLACE, HIGH, OPEN),
]


def main():
    for t, x, y, z, g in PLAN:
        assert inverse(x, y, z, d=D_GRIP), f"точка {(x, y, z)} вне досягаемости"
    args = ["65536", "20000000", "20000000"]
    for j, e, s in JOINTS:
        args += [f"$input:рука/{e}/value", f"рука/{e}/value", "rad"]
    for j, e, s in JOINTS:
        args += [f"$output:{j}", j, "rad"]
    args += ["1", "$output:схват", "схват", "m"]
    args += [str(H0), str(A1), str(A2), str(D_GRIP)]
    for lo, hi in LIMITS_DEG:
        args += [str(lo), str(hi)]
    args += [str(len(PLAN))]
    for t, x, y, z, g in PLAN:
        args += [str(t), str(x), str(y), str(z), "180", "0", str(g)]
    c = ['version = 1', '', '[[controller]]', 'name = "компьютер"', 'kind = "external"', 'rate_hz = 100',
         '# Собирается в общую target симулятора: см. README.',
         'program = "../../target/debug/claude-arm-controller"',
         '# Аргументы пишет tools/make_pick.py: как в control.toml плюс выход схвата и ход губы в каждой точке.',
         'args = ' + json.dumps(args, ensure_ascii=False),
         'input_latency_ns = 2000000', 'input_jitter_ns = 500000', 'output_latency_ns = 1000000', 'output_jitter_ns = 500000',
         'request_hz = 1000', 'request_max_age_ns = 20000000', 'queue_capacity = 16', 'max_message_bytes = 65536',
         'source = "Компьютер с Claude шлёт задания углов и хода губы по шине 1 Мбит/с через USB-UART; задержки — оценка до замера."']
    for j, e, s in JOINTS:
        c += ['', '[[input]]', 'controller = "компьютер"', f'signal = "рука/{e}/value"', 'unit = "rad"', 'kind = "scalar"',
              'max_age_ns = 20000000', 'source = "Опрос энкодера сервы по шине, срок два такта по 10 мс."']
    for name, target, ack, unit in [(j, f"рука/{j}", f"рука/{s}/accepted_command", "rad") for j, e, s in JOINTS] + \
            [("схват", "рука/схват", "рука/серва-схвата/accepted_command", "m")]:
        c += ['', '[[output]]', f'name = "{name}"', 'controller = "компьютер"', f'target = "{target}"', 'target_kind = "drive"',
              f'unit = "{unit}"', f'ack = "{ack}"', 'max_age_ns = 20000000',
              '# Шинная серва при пропуске команды держит последнее принятое задание.',
              'fallback = "hold"', 'source = "Задание сервы. Резерв — последнее ушедшее задание."']
    open(os.path.join(ROOT, 'control-pick.toml'), 'w').write("\n".join(c) + "\n")
    hops = ('hops = [\n  { target = "transport:компьютер/request", direction = "input", delay_ns = 2000000, jitter_ns = 500000, '
            'receiver_tick_ns = 1000000, source = { kind = "derived", reference = "control-pick.toml, такт получателя 1 кГц" } },\n'
            '  { target = "transport:компьютер/response", direction = "output", delay_ns = 1000000, jitter_ns = 500000, '
            'receiver_tick_ns = 10000000, source = { kind = "derived", reference = "control-pick.toml, такт получателя 100 Гц" } },\n]')
    p = ['version = 1', 'required_modes = ["перенос"]', '']
    for i, (j, e, s) in enumerate(JOINTS):
        p += ['[[readouts]]', f'signal = {{ name = "угол {j}", unit = "rad" }}', 'rate_hz = 1000',
              'source = { kind = "derived", reference = "Истинная координата сустава из MuJoCo." }',
              f'formula = {{ kind = "position", index = {i} }}', '']
    p += ['[[readouts]]', 'signal = { name = "ход губы", unit = "m" }', 'rate_hz = 1000',
          'source = { kind = "derived", reference = "Истинная координата ползуна губы из MuJoCo." }',
          'formula = { kind = "position", index = 5 }', '']
    for axis, name in enumerate("xyz"):
        p += ['[[readouts]]', f'signal = {{ name = "кубик {name}", unit = "m" }}', 'rate_hz = 1000',
              'source = { kind = "derived", reference = "Центр кубика из позы тела MuJoCo." }',
              f'formula = {{ kind = "body_point", body = "кубик/кубик", local_m = [0.0, 0.0, 0.0], axis = {axis} }}', '']
    for j, e, s in JOINTS:
        p += [f'[paths."{j}"]', f'observation = {{ name = "рука/{e}/value", unit = "rad" }}',
              f'physical_response = [{{ name = "угол {j}", unit = "rad" }}]', 'command_delta = 0.01',
              'max_observation_age_ns = 20000000',
              'source = { kind = "derived", reference = "0.01 рад больше шага энкодера 0.0015 рад." }', hops, '']
    p += ['[paths."схват"]', 'observation = { name = "рука/энкодер-запястья/value", unit = "rad" }',
          'physical_response = [{ name = "ход губы", unit = "m" }]', 'command_delta = 0.001',
          'max_observation_age_ns = 20000000',
          'source = { kind = "derived", reference = "Энкодера губы нет: контроллер схват не наблюдает, путь замкнут через тот же опрос шины." }',
          hops, '']
    open(os.path.join(ROOT, 'profile-pick.toml'), 'w').write("\n".join(p))
    end = PLAN[-1][0]
    a = ['version = 1', 'control = "control-pick.toml"', 'profile = "profile-pick.toml"', '', '[[modes]]', 'id = "перенос"',
         f'horizon_ns = {int(round((end + 0.5) * 1e9))}', 'before_ns = 100000000', 'after_ns = 300000000', 'max_gap_ns = 2000000',
         'source = { kind = "derived", reference = "Подвести, опустить, сжать, поднять, перенести на 120 мм, поставить, разжать, отойти." }',
         '# Лёгкий толчок по плечу после отхода: проверка, что рука держит позу.',
         'excitation = "толчок"', 'stimuli = [',
         f'  {{ id = "толчок", signal = {{ name = "рука/толчок-плеча", unit = "N*m" }}, at_ns = {int(round((end + 0.1) * 1e9))}, value = 0.1, comparison_value = 0.0, '
         'source = { kind = "estimate", reference = "Касание рукой человека." } },', ']', '']
    for (j, e, s), (lo, hi) in list(zip(JOINTS, LIMITS_DEG)) + [(("схват", "", ""), (0, 0))]:
        lim = (f'{{ kind = "range", signal = {{ name = "угол {j}", unit = "rad" }}, min = {math.radians(lo)}, max = {math.radians(hi)}, '
               'source = { kind = "estimate", reference = "Рабочий диапазон сустава по эскизу." } }') if j != "схват" else \
              ('{ kind = "range", signal = { name = "ход губы", unit = "m" }, min = -0.0005, max = 0.0305, '
               'source = { kind = "estimate", reference = "Ход губы 0–30 мм по эскизу; упор MuJoCo мягкий, допуск 0.5 мм." } }')
        a += ['[[criteria]]', f'case = {{ path = "{j}", mode = "перенос" }}',
              f'limits = [{lim}, {{ kind = "range", signal = {{ name = "кубик z", unit = "m" }}, min = 0.0, max = 0.2, '
              'source = { kind = "estimate", reference = "Кубик не проваливается под стол и не улетает." } }, '
              '{ kind = "range", signal = { name = "кубик x", unit = "m" }, min = 0.0, max = 0.4, '
              'source = { kind = "estimate", reference = "Кубик остаётся на столе перед рукой." } }, '
              '{ kind = "range", signal = { name = "кубик y", unit = "m" }, min = -0.2, max = 0.2, '
              'source = { kind = "estimate", reference = "Кубик остаётся на столе перед рукой." } }]', '']
    open(os.path.join(ROOT, 'acceptance-pick.toml'), 'w').write("\n".join(a))
    print("записаны control-pick.toml, profile-pick.toml, acceptance-pick.toml; точек", len(PLAN))


if __name__ == "__main__":
    main()
