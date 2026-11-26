"""Точность кончика в конце каждой выдержки, вне вердикта платформы (дефект Д2).

Повторяет команды контроллера по 100 Гц и для каждой точки делает короткий прогон
до конца выдержки, читает истинные углы из JSON и считает положение кончика.
Запуск из папки руки: python3 tools/measure.py [путь к sim-run]
"""
import json, math, os, subprocess, sys, tempfile
from kinematics import inverse, forward
from make_task import schedule, START

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.abspath(os.path.join(ROOT, "..", ".."))
RUN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIM, "target", "debug", "sim-run")
NAMES = ["поворот", "плечо", "локоть", "запястье", "вращение"]


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def commands(plan, until):
    """Команды по 100 Гц до момента `until` включительно: sim-run не принимает команд после конца прогона."""
    segs, prev, t0 = [], [0.0] * 5, START
    for t, p in plan:
        q = inverse(*p)
        segs.append((t0, t, prev, q))
        prev, t0 = q, t
    rows = ["time_s,channel,value"]
    for k in range(int(round(until / 0.01))):
        tt, q = k * 0.01, [0.0] * 5
        for a, b, q0, q1 in segs:
            if tt >= a:
                q = [q0[j] + min_jerk((tt - a) / (b - a)) * (q1[j] - q0[j]) for j in range(5)]
        rows += [f"{tt:.2f},рука/{n},{v:.9f}" for n, v in zip(NAMES, q)]
    return "\n".join(rows) + "\n"


def main():
    plan = schedule()
    with tempfile.TemporaryDirectory() as tmp:
        csv = os.path.join(tmp, "commands.csv")
        worst = 0.0
        for (t, p), (t_prev, _) in zip(plan[1::2], plan[0::2]):
            open(csv, "w").write(commands(plan, t))
            out = subprocess.run([RUN, os.path.join(ROOT, "world.toml"), "--catalog", os.path.join(SIM, "catalog"),
                                  "--duration", f"{t:.6f}", "--commands", csv, "--json"], capture_output=True, text=True)
            d = json.loads(out.stdout)
            if d["status"] != "completed":
                print(f"{t:.2f} с: прогон не завершён: {d.get('component_failure') or d.get('detail')}")
                return 1
            q = [d["channels"]["truth"][f"qpos[{i}]"]["value"] for i in range(5)]
            f = forward(q)
            e = math.dist(f, p) * 1000
            worst = max(worst, e)
            print(f"{t:5.2f} с  цель ({p[0]*1e3:4.0f},{p[1]*1e3:4.0f},{p[2]*1e3:4.0f}) мм  ошибка {e:5.2f} мм  "
                  f"dx/dy/dz {(f[0]-p[0])*1e3:+.2f}/{(f[1]-p[1])*1e3:+.2f}/{(f[2]-p[2])*1e3:+.2f}", flush=True)
        print(f"наибольшая ошибка {worst:.2f} мм")
    return 0


if __name__ == "__main__":
    sys.exit(main())
