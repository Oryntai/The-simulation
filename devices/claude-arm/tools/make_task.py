"""Пишет control.toml, profile.toml и acceptance.toml для обхода точек с выдержкой."""
import json, math, os
from kinematics import inverse, H0, A1, A2, D, LIMITS_DEG

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
JOINTS = [("поворот", "энкодер-поворота", "серва-поворота"), ("плечо", "энкодер-плеча", "серва-плеча"),
          ("локоть", "энкодер-локтя", "серва-локтя"), ("запястье", "энкодер-запястья", "серва-запястья"),
          ("вращение", "энкодер-вращения", "серва-вращения")]
# Углы «коробки» над столом, инструмент вниз. Переезд 0.8 с, выдержка 0.5 с.
POINTS = [(0.16, 0, 0.10), (0.12, -0.08, 0.03), (0.20, -0.08, 0.03), (0.20, 0.08, 0.03), (0.12, 0.08, 0.03), (0.16, 0, 0.10)]
MOVE, DWELL, START = 0.8, 0.5, 0.2


def schedule():
    out, t = [], START
    for p in POINTS:
        t += MOVE
        out.append((round(t, 6), p))
        t += DWELL
        out.append((round(t, 6), p))
    return out


def main():
    plan = schedule()
    for _, p in plan:
        assert inverse(*p), f"точка {p} вне досягаемости"
    args = ["65536", "20000000", "20000000"]
    for j, e, s in JOINTS:
        args += [f"$input:рука/{e}/value", f"рука/{e}/value", "rad"]
    for j, e, s in JOINTS:
        args += [f"$output:{j}", j, "rad"]
    args += ["0"]  # без схвата
    args += [str(H0), str(A1), str(A2), str(D)]
    for lo, hi in LIMITS_DEG:
        args += [str(lo), str(hi)]
    args += [str(len(plan))]
    for t, (x, y, z) in plan:
        args += [str(t), str(x), str(y), str(z), "180", "0"]
    c = ['version = 1', '', '[[controller]]', 'name = "компьютер"', 'kind = "external"', 'rate_hz = 100',
         '# Собирается в общую target симулятора: см. README.',
         'program = "../../target/debug/claude-arm-controller"',
         '# Аргументы пишет tools/make_task.py: предел сообщения, сроки входа и выхода, 5 входов, 5 выходов,',
         '# геометрия h0 a1 a2 d, пределы суставов в градусах, число точек и точки (t, x, y, z, наклон, вращение).',
         'args = ' + json.dumps(args, ensure_ascii=False),
         'input_latency_ns = 2000000', 'input_jitter_ns = 500000', 'output_latency_ns = 1000000', 'output_jitter_ns = 500000',
         'request_hz = 1000', 'request_max_age_ns = 20000000', 'queue_capacity = 16', 'max_message_bytes = 65536',
         'source = "Компьютер с Claude шлёт задания углов по шине 1 Мбит/с через USB-UART; задержки — оценка до замера."']
    for j, e, s in JOINTS:
        c += ['', '[[input]]', 'controller = "компьютер"', f'signal = "рука/{e}/value"', 'unit = "rad"', 'kind = "scalar"',
              'max_age_ns = 20000000', 'source = "Опрос энкодера сервы по шине, срок два такта по 10 мс."']
    for j, e, s in JOINTS:
        c += ['', '[[output]]', f'name = "{j}"', 'controller = "компьютер"', f'target = "рука/{j}"', 'target_kind = "drive"',
              'unit = "rad"', f'ack = "рука/{s}/accepted_command"', 'max_age_ns = 20000000',
              '# Шинная серва при пропуске команды держит последнее принятое задание.',
              'fallback = "hold"', 'source = "Задание угла сервы. Резерв — последнее ушедшее задание."']
    open(os.path.join(ROOT, 'control.toml'), 'w').write("\n".join(c) + "\n")
    hops = ('hops = [\n  { target = "transport:компьютер/request", direction = "input", delay_ns = 2000000, jitter_ns = 500000, '
            'receiver_tick_ns = 1000000, source = { kind = "derived", reference = "control.toml, такт получателя 1 кГц" } },\n'
            '  { target = "transport:компьютер/response", direction = "output", delay_ns = 1000000, jitter_ns = 500000, '
            'receiver_tick_ns = 10000000, source = { kind = "derived", reference = "control.toml, такт получателя 100 Гц" } },\n]')
    p = ['version = 1', 'required_modes = ["обход"]', '']
    for i, (j, e, s) in enumerate(JOINTS):
        p += ['[[readouts]]', f'signal = {{ name = "угол {j}", unit = "rad" }}', 'rate_hz = 1000',
              'source = { kind = "derived", reference = "Истинная координата сустава из MuJoCo." }',
              f'formula = {{ kind = "position", index = {i} }}', '']
    # Кончик — центр фланца кисти, 30 мм от оси вращения, в координатах стола.
    for axis, name in enumerate("xyz"):
        p += ['[[readouts]]', f'signal = {{ name = "кончик {name}", unit = "m" }}', 'rate_hz = 1000',
              'source = { kind = "derived", reference = "Центр фланца из позы тела кисти MuJoCo." }',
              f'formula = {{ kind = "body_point", body = "рука/кисть", local_m = [0.0, 0.0, 0.03], axis = {axis} }}', '']
    for j, e, s in JOINTS:
        p += [f'[paths."{j}"]', f'observation = {{ name = "рука/{e}/value", unit = "rad" }}',
              f'physical_response = [{{ name = "угол {j}", unit = "rad" }}]', 'command_delta = 0.01',
              'max_observation_age_ns = 20000000',
              'source = { kind = "derived", reference = "0.01 рад больше шага энкодера 0.0015 рад." }', hops, '']
    open(os.path.join(ROOT, 'profile.toml'), 'w').write("\n".join(p))
    end = plan[-1][0]
    push = int(round((end + 0.2) * 1e9))
    horizon = int(round((end + 1.0) * 1e9))
    a = ['version = 1', 'control = "control.toml"', 'profile = "profile.toml"', '', '[[modes]]', 'id = "обход"',
         f'horizon_ns = {horizon}', 'before_ns = 100000000', 'after_ns = 500000000', 'max_gap_ns = 2000000',
         'source = { kind = "derived", reference = "Шесть точек: переезд 0.8 с, выдержка 0.5 с; затем толчок." }',
         '# Толчок по плечу после обхода: человек нажимает на руку.',
         'excitation = "толчок"', 'stimuli = [',
         f'  {{ id = "толчок", signal = {{ name = "рука/толчок-плеча", unit = "N*m" }}, at_ns = {push}, value = 0.3, comparison_value = 0.0, '
         'source = { kind = "estimate", reference = "Нажим около 1 Н на вылете 0.3 м: человек толкает руку." } },', ']', '',
         '# Компьютер с Claude зависает, когда рука стоит в третьем углу коробки над столом.',
         '[[modes.faults]]', 'fault = { node = "controller:компьютер", kind = "controller_halted" }',
         f'at_ns = {int(round(plan[5][0] * 1e9))}',
         'source = { kind = "derived", reference = "Конец выдержки в точке (200, −80, 30) мм." }', '']
    for fault in (None, '{ node = "controller:компьютер", kind = "controller_halted" }'):
        for (j, e, s), (lo, hi) in zip(JOINTS, LIMITS_DEG):
            case = f'{{ path = "{j}", mode = "обход"' + (f', fault = {fault}' if fault else '') + ' }'
            a += ['[[criteria]]', f'case = {case}',
                  f'limits = [{{ kind = "range", signal = {{ name = "угол {j}", unit = "rad" }}, min = {math.radians(lo)}, '
                  f'max = {math.radians(hi)}, source = {{ kind = "estimate", reference = "Рабочий диапазон сустава по эскизу." }} }}, '
                  '{ kind = "range", signal = { name = "кончик z", unit = "m" }, min = 0.0, max = 0.5, '
                  'source = { kind = "estimate", reference = "Фланец не уходит ниже столешницы z = 0." } }]', '']
    open(os.path.join(ROOT, 'acceptance.toml'), 'w').write("\n".join(a))
    print("записаны control.toml, profile.toml, acceptance.toml; точек", len(plan), "горизонт", horizon / 1e9, "с")


if __name__ == "__main__":
    main()
