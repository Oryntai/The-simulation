"""Опыт без контроллера: суставы по командам CSV идут из вертикали в позу и держат её.

Печатает промах центра захвата по истинным углам, ошибки суставов, ток и температуру обмоток.
Запуск из папки руки: python3 tools/hold_test.py [поза] [длительность хода, с] [выдержка, с]
Позы: бутылка (центр захвата у бутылки), вылет (рука горизонтально на 0.6 м), кружка.
"""
import json, math, os, subprocess, sys, tempfile

from kinematics import NAMES, forward, frame, inverse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUN = os.path.join(ROOT, "..", "..", "target", "debug", "sim-run")

POSES = {
    "бутылка": ([0.45, -0.10, 0.11], frame([1, 0, 0], [0, 1, 0])),
    "вылет": ([0.70, 0.0, 0.23], frame([1, 0, 0], [0, 1, 0])),
    "кружка": ([0.45, 0.0, 0.25], frame([1, 0, 0], [0, 1, 0])),
}


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def main():
    pose = sys.argv[1] if len(sys.argv) > 1 else "бутылка"
    move = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    p, r = POSES[pose]
    goal = inverse(p, r)
    rows = ["time_s,channel,value"]
    rate = 200
    for k in range(int((move + hold) * rate) + 1):
        t = k / rate
        s = min_jerk(t / move)
        for n, g in zip(NAMES, goal):
            rows.append(f"{t:.4f},рука/{n},{s * g:.9f}")
        rows.append(f"{t:.4f},рука/схват,-0.3")
    with tempfile.TemporaryDirectory() as tmp:
        csv = os.path.join(tmp, "c.csv")
        open(csv, "w").write("\n".join(rows) + "\n")
        out = subprocess.run([RUN, os.path.join(ROOT, "world.toml"), "--duration", f"{move + hold:.4f}",
                              "--commands", csv, "--json"], capture_output=True, text=True)
    try:
        d = json.loads(out.stdout)
    except json.JSONDecodeError:
        print(out.stdout[-2000:], out.stderr[-2000:])
        return 1
    if d["status"] != "completed":
        print("прогон:", d["status"], d.get("component_failure") or d.get("detail"))
    truth = d["channels"]["truth"]
    q = [truth[f"qpos[{i}]"]["value"] for i in range(6)]
    tcp, _ = forward(q)
    err = math.dist(tcp, p) * 1000
    print(f"поза «{pose}»: TCP ({', '.join(f'{c * 1000:.2f}' for c in tcp)}) мм при цели ({', '.join(f'{c * 1000:.0f}' for c in p)}), промах {err:.3f} мм")
    for n, g, v in zip(NAMES, goal, q):
        drive = {"поворот": "поворота", "плечо": "плеча", "локоть": "локтя", "ротация": "ротации",
                 "запястье": "запястья", "вращение": "кисти"}[n]
        cur = truth[f"рука/привод-{drive}/current_a"]["value"]
        tor = truth[f"рука/привод-{drive}/joint_torque_nm"]["value"]
        wind = truth.get(f"рука/привод-{drive}/winding_c", {}).get("value")
        print(f"  {n:9s} задание {math.degrees(g):8.3f}°, факт {math.degrees(v):8.3f}°, ошибка {math.degrees(v - g) * 3600:8.1f}″,"
              f" ток {cur:6.3f} А, момент {tor:7.3f} Н·м" + (f", обмотка {wind:.1f} °C" if wind is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
