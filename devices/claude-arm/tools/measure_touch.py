"""Касания насадками, вне вердикта платформы, как tools/measure.py.

Насадка прикреплена к фланцу с самого начала (крепление design), рука по командам суставов 100 Гц
подходит к точке над столом по суставам, дальше идёт по отрезкам, как контроллер в режиме «прямые».
Короткий прогон до нужного момента, из истинной позы насадки считается центр наконечника.
Замок сменщика здесь не нужен: меряется только, куда фактически приходит наконечник.

- щуп: промах по столу в двух точках касания;
- маркер: высота центра пишущего узла вдоль линии, касается ли он стола на всём пути, и отставание
  от заданной точки вдоль линии.

Запуск из папки руки: python3 tools/measure_touch.py [путь к sim-run]
"""
import json, math, os, shutil, subprocess, sys, tempfile
from kinematics import inverse
from measure import min_jerk, NAMES
from make_tools import TOOLS, TOUCHES, LINE, HOVER, STROKE_S, STROKE_HOLD_S, tip_flange

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.abspath(os.path.join(ROOT, "..", ".."))
RUN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIM, "target", "debug", "sim-run")
# Подвод 1.5 с по суставам, опускание 0.4 с, выдержка 0.4 с; линия маркера как в задании.
MOVE, DOWN, DWELL, STROKE = 1.5, 0.4, 0.4, STROKE_S


def world(tool):
    """Мир руки с насадкой на фланце: в стартовой вертикали фланец в (0, 0, 0.41), ось z вверх."""
    text = open(os.path.join(ROOT, "world.toml")).read()
    return text + "\n".join([
        "", f"# {tool} на фланце с начала прогона.", "[[devices]]", f'name = "{tool}"', f'file = "{TOOLS[tool]["file"]}"',
        "pose = { position_m = [0.0, 0.0, 0.41] }", "", "[[attachments]]", f'name = "{tool}"',
        'first = "рука/кисть/фланец"', f'second = "{tool}/основание/фланец"', 'mode = "design"', ""])


def target(points, t):
    """Как контроллер: первый переезд из вертикали по суставам, дальше по отрезкам."""
    prev_t, prev_q, prev_p = 0.0, [0.0] * 5, None
    for ti, p in points:
        if t < ti:
            s = min_jerk((t - prev_t) / (ti - prev_t))
            if prev_p is None:
                return [a + s * (b - a) for a, b in zip(prev_q, inverse(*p))]
            return inverse(*[a + s * (b - a) for a, b in zip(prev_p, p)])
        prev_t, prev_q, prev_p = ti, inverse(*p), p
    return prev_q


def commands(points, until):
    rows = ["time_s,channel,value"]
    for k in range(int(round(until / 0.01))):
        rows += [f"{k * 0.01:.2f},рука/{n},{v:.9f}" for n, v in zip(NAMES, target(points, k * 0.01))]
    return "\n".join(rows) + "\n"


def tip(tool, truth):
    """Центр наконечника из позы основания насадки: qpos[5..7] — точка крепления, qpos[8..11] — wxyz."""
    p = [truth[f"qpos[{i}]"]["value"] for i in range(5, 8)]
    w, x, y, z = (truth[f"qpos[{i}]"]["value"] for i in range(8, 12))
    axis = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
    return [p[i] + TOOLS[tool]["length"] * axis[i] for i in range(3)]


def run(tmp, tool, points, until):
    csv = os.path.join(tmp, "commands.csv")
    open(csv, "w").write(commands(points, until))
    out = subprocess.run([RUN, os.path.join(tmp, f"world-{TOOLS[tool]['tag']}.toml"), "--catalog", os.path.join(SIM, "catalog"),
                          "--duration", f"{until:.6f}", "--commands", csv, "--json"], capture_output=True, text=True)
    d = json.loads(out.stdout)
    if d["status"] != "completed":
        raise SystemExit(f"{tool}, {until:.2f} с: прогон не завершён: {d.get('component_failure') or d.get('detail')}")
    return tip(tool, d["channels"]["truth"])


def main():
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copy(os.path.join(ROOT, "device.toml"), tmp)
        for tool in TOOLS:
            shutil.copy(os.path.join(ROOT, TOOLS[tool]["file"]), tmp)
            open(os.path.join(tmp, f"world-{TOOLS[tool]['tag']}.toml"), "w").write(world(tool))
        r, press = TOOLS["щуп"]["tip"], TOOLS["щуп"]["press"]
        worst = 0.0
        for x, y in TOUCHES:
            points = [(MOVE, tip_flange("щуп", x, y, HOVER)), (MOVE + DOWN, tip_flange("щуп", x, y, r - press))]
            t = run(tmp, "щуп", points, MOVE + DOWN + DWELL)
            e = math.hypot(t[0] - x, t[1] - y) * 1000
            worst = max(worst, e)
            print(f"щуп, цель ({x * 1e3:.0f}, {y * 1e3:.0f}) мм: наконечник ({t[0] * 1e3:.2f}, {t[1] * 1e3:.2f}, {t[2] * 1e3:.2f}) мм, "
                  f"промах по столу {e:.2f} мм, центр сферы на {t[2] * 1e3:.2f} мм при радиусе {r * 1e3:.1f} мм", flush=True)
        print(f"щуп: наибольший промах {worst:.2f} мм")
        (ax, ay), (bx, by) = LINE
        r, press = TOOLS["маркер"]["tip"], TOOLS["маркер"]["press"]
        start = MOVE + DOWN + DWELL
        points = [(MOVE, tip_flange("маркер", ax, ay, HOVER)), (MOVE + DOWN, tip_flange("маркер", ax, ay, r - press)),
                  (start, tip_flange("маркер", ax, ay, r - press)), (start + STROKE, tip_flange("маркер", bx, by, r - press))]
        points.append((start + STROKE + STROKE_HOLD_S, tip_flange("маркер", bx, by, r - press)))
        for share in (0.0, 0.25, 0.5, 0.75, 1.0, None):
            if share is None:
                t = run(tmp, "маркер", points, start + STROKE + STROKE_HOLD_S)
                print(f"маркер, конец выдержки: узел ({t[0] * 1e3:.2f}, {t[1] * 1e3:.2f}, {t[2] * 1e3:.2f}) мм при цели "
                      f"({bx * 1e3:.0f}, {by * 1e3:.0f}) мм, недоход {(by - t[1]) * 1e3:.2f} мм", flush=True)
                continue
            t = run(tmp, "маркер", points, start + share * STROKE)
            goal = ay + min_jerk(share) * (by - ay)
            print(f"маркер, {share * 100:3.0f} % линии: узел ({t[0] * 1e3:.2f}, {t[1] * 1e3:.2f}) мм при цели y {goal * 1e3:.2f} мм, "
                  f"отставание {(goal - t[1]) * 1e3:.2f} мм, центр на {t[2] * 1e3:.2f} мм при радиусе {r * 1e3:.1f} мм: "
                  f"{'касается' if t[2] <= r else 'над столом на ' + format((t[2] - r) * 1e3, '.2f') + ' мм'}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
