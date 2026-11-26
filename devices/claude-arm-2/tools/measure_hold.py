"""Сила и точность с полной бутылкой, вне вердикта платформы (как measure.py первой руки).

Бутылка 1.033 кг жёстко на центре захвата (крепление design вместо сжатия губ). Суставы по
командам CSV из вертикали идут в позу и держат её. Печатает промах центра захвата по истинным
углам, ошибку каждого сустава, момент против номинала привода и нагрев обмоток maxon.

Запуск из папки руки: python3 tools/measure_hold.py [поза] [выдержка, с] [упреждение]
Позы: вылет — центр захвата в 0.62 м от оси поворота на высоте плеча; налив — поза налива над кружкой.
"""
import json, math, os, shutil, subprocess, sys, tempfile

from kinematics import NAMES, compensate, forward, frame, inverse
from make_arm import H_YAW, H_SHOULDER, L1, L2, TCP, GRASP

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUN = os.path.join(ROOT, "..", "..", "target", "debug", "sim-run")
CARRY = frame([1, 0, 0], [0, 1, 0])
RATED = {"поворот": 30.6, "плечо": 30.6, "локоть": 30.6, "ротация": 3.0, "запястье": 3.0, "вращение": 3.0}
DRIVE = {"поворот": "поворота", "плечо": "плеча", "локоть": "локтя", "ротация": "ротации", "запястье": "запястья", "вращение": "кисти"}


def pour_pose():
    """Наклон 78° к кружке, горло над кружкой: центр захвата на 0.16 м ниже-левее горла."""
    theta = math.radians(78)
    mouth = [0.45, 0.085, 0.125]
    k = [-1.0, 0.0, 0.0]
    c, s = math.cos(theta), math.sin(theta)
    rot = [[1, 0, 0], [0, c, -s], [0, s, c]] if k == [1.0, 0.0, 0.0] else [[1, 0, 0], [0, c, s], [0, -s, c]]
    r = [[sum(rot[i][m] * CARRY[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
    axis = [r[i][1] for i in range(3)]
    return [mouth[i] - 0.16 * axis[i] for i in range(3)], r


POSES = {"вылет": ([0.62, 0.0, H_YAW + H_SHOULDER], CARRY), "налив": pour_pose()}


def world():
    """Мир руки с бутылкой на центре захвата: в вертикали ладонь в осях мира, центр захвата вверху."""
    text = open(os.path.join(ROOT, "world.toml")).read()
    text = text[:text.index('[[devices]]\nname = "бутылка"')]
    top = H_YAW + H_SHOULDER + L1 + L2 + TCP
    # Ось бутылки — y ладони, подход — x бутылки: поворот бутылки обратен хвату (rpy 90°, 0, 90°).
    return text + "\n".join([
        "# Бутылка жёстко на центре захвата с начала прогона.", "[[devices]]", 'name = "бутылка"', 'file = "bottle.toml"',
        f"pose = {{ position_m = [0.0, {-GRASP}, {top}], quat_wxyz = [0.5, -0.5, -0.5, -0.5] }}", "",
        "[[attachments]]", 'name = "хват"', 'first = "рука/ладонь/захват"', 'second = "бутылка/корпус/хват"',
        'mode = "design"', ""])


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def main():
    pose = sys.argv[1] if len(sys.argv) > 1 else "вылет"
    hold = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    move = 4.0
    p, r = POSES[pose]
    goal = inverse(p, r)
    # «упреждение»: задание мотору с поправкой на скручивание передач и люфт под весом руки и бутылки.
    comp = len(sys.argv) > 3 and sys.argv[3] == "упреждение"
    command = compensate(goal, 1.0332) if comp else goal
    rows = ["time_s,channel,value"]
    for k in range(int((move + hold) * 100) + 1):
        t = k / 100
        s = min_jerk(t / move)
        rows += [f"{t:.2f},рука/{n},{s * g:.9f}" for n, g in zip(NAMES, command)]
        rows.append(f"{t:.2f},рука/схват,0.0")
    with tempfile.TemporaryDirectory() as tmp:
        for name in ("device.toml", "bottle.toml"):
            shutil.copy(os.path.join(ROOT, name), tmp)
        open(os.path.join(tmp, "world.toml"), "w").write(world())
        open(os.path.join(tmp, "c.csv"), "w").write("\n".join(rows) + "\n")
        out = subprocess.run([RUN, os.path.join(tmp, "world.toml"), "--catalog", os.path.join(ROOT, "..", "..", "catalog"),
                              "--duration", f"{move + hold:.2f}", "--commands", os.path.join(tmp, "c.csv"), "--json"],
                             capture_output=True, text=True)
    try:
        d = json.loads(out.stdout)
    except json.JSONDecodeError:
        print(out.stdout[-1500:], out.stderr[-1500:])
        return 1
    if d["status"] != "completed":
        print("прогон:", d["status"], d.get("component_failure") or d.get("detail"))
        if "qpos[0]" not in d["channels"]["truth"]:
            return 1
    truth = d["channels"]["truth"]
    q = [truth[f"qpos[{i}]"]["value"] for i in range(6)]
    tcp, _ = forward(q)
    reach = math.hypot(p[0], p[1])
    print(f"поза «{pose}»{' с упреждением просадки' if comp else ''}, бутылка 1.033 кг, выдержка {hold:g} с: центр захвата в {reach * 1000:.0f} мм от оси поворота, "
          f"промах {math.dist(tcp, p) * 1000:.3f} мм (по z {(tcp[2] - p[2]) * 1000:+.3f} мм)")
    for n, g, v in zip(NAMES, goal, q):
        d_ = f"рука/привод-{DRIVE[n]}"
        tor = truth[f"{d_}/joint_torque_nm"]["value"]
        wind = truth.get(f"{d_}/winding_c", {}).get("value")
        print(f"  {n:9s} ошибка {math.degrees(v - g) * 3600:7.1f}″, момент {tor:7.3f} Н·м = {abs(tor) / RATED[n] * 100:5.1f} % номинала"
              + (f", обмотка {wind:.1f} °C" if wind is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
