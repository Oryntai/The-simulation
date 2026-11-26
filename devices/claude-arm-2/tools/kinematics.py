"""Кинематика кронштейна: прямая задача по звеньям device.toml и обратная для сферического запястья.

Поза инструмента — центр захвата (TCP) и поворот ладони (столбцы — оси x, y, z ладони в мире).
Ось z ладони — направление подхода, ось x — направление сжатия (от неподвижной губы к подвижной).
"""
import math

from make_arm import H_YAW, H_SHOULDER, L1, L2, E_ROLL, L2B, D_FLANGE, D_PLATE, D_SENSOR, TCP_Z, TCP, LIMITS

H0 = H_YAW + H_SHOULDER
NAMES = ["поворот", "плечо", "локоть", "ротация", "запястье", "вращение"]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [list(r) for r in zip(*a)]


def rz(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def ry(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def apply(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def forward(q):
    """TCP и поворот ладони для углов q (рад)."""
    r = rz(q[0])
    p = [0.0, 0.0, H0]
    r = matmul(r, ry(q[1]))
    p = [p[i] + apply(r, [0, 0, L1])[i] for i in range(3)]
    r = matmul(r, ry(q[2]))
    p = [p[i] + apply(r, [0, 0, E_ROLL])[i] for i in range(3)]
    r = matmul(r, rz(q[3]))
    p = [p[i] + apply(r, [0, 0, L2B])[i] for i in range(3)]
    r = matmul(r, ry(q[4]))
    r = matmul(r, rz(q[5]))
    p = [p[i] + apply(r, [0, 0, TCP])[i] for i in range(3)]
    return p, r


def inverse(p, r, near=None):
    """Углы для TCP p и поворота ладони r. Локоть сверху. `near` — прошлые углы для выбора ветви запястья."""
    z = [r[i][2] for i in range(3)]
    w = [p[i] - TCP * z[i] for i in range(3)]
    q1 = math.atan2(w[1], w[0])
    rr = math.hypot(w[0], w[1])
    zz = w[2] - H0
    d2 = rr * rr + zz * zz
    c3 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c3) > 1.0:
        raise ValueError(f"центр запястья вне досягаемости: {math.sqrt(d2) * 1000:.0f} мм от плеча")
    q3 = math.acos(c3)
    q2 = math.atan2(rr, zz) - math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    r03 = matmul(rz(q1), ry(q2 + q3))
    m = matmul(transpose(r03), r)
    # m = Rz(q4)·Ry(q5)·Rz(q6)
    s5 = math.hypot(m[0][2], m[1][2])
    candidates = []
    for sign in (1, -1):
        q5 = math.atan2(sign * s5, m[2][2])
        if s5 < 1e-9:
            q4 = near[3] if near else 0.0
            q6 = math.atan2(m[1][0], m[0][0]) - q4
        else:
            q4 = math.atan2(sign * m[1][2], sign * m[0][2])
            q6 = math.atan2(sign * m[2][1], -sign * m[2][0])
        candidates.append([q1, q2, q3, q4, q5, q6])
    def cost(q):
        ok = all(math.radians(LIMITS[n][0]) <= v <= math.radians(LIMITS[n][1]) for n, v in zip(NAMES, q))
        ref = near or [0.0] * 6
        return (0 if ok else 1, sum((a - b) ** 2 for a, b in zip(q[3:], ref[3:])))
    return min(candidates, key=cost)


def frame(approach, closing):
    """Поворот ладони по оси подхода (z) и сжатия (x); y = z × x."""
    def unit(v):
        n = math.sqrt(sum(c * c for c in v))
        return [c / n for c in v]
    z = unit(approach)
    x = unit([closing[i] - sum(closing[k] * z[k] for k in range(3)) * z[i] for i in range(3)])
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[i], y[i], z[i]] for i in range(3)]


if __name__ == "__main__":
    import random
    random.seed(1)
    worst = 0.0
    for _ in range(2000):
        q = [random.uniform(math.radians(LIMITS[n][0]), math.radians(LIMITS[n][1])) for n in NAMES]
        q[2] = abs(q[2]) + 0.05
        p, r = forward(q)
        try:
            back = inverse(p, r, near=q)
        except ValueError:
            continue
        p2, r2 = forward(back)
        err = max(abs(a - b) for a, b in zip(p, p2)) + max(abs(r[i][j] - r2[i][j]) for i in range(3) for j in range(3))
        worst = max(worst, err)
    print(f"обратная задача: наибольшая невязка {worst:.2e}")
    grip = frame([1, 0, 0], [0, 1, 0])
    for name, p in (("бутылка", [0.45, -0.10, 0.11]), ("над кружкой", [0.45, 0.0, 0.25])):
        q = inverse(p, grip)
        print(name, [round(math.degrees(v), 1) for v in q])


# ---------- вес звеньев: моменты тяжести и упреждение просадки ----------
G_VEC = [0.0, 0.0, -9.80665]
# Звенья после каждого сустава: (имя звена, родительский сустав по порядку цепи).
CHAIN = ["башня", "плечо", "предплечье", "рукав", "запястье", "кисть", "датчик", "ладонь", "губа"]


def link_frames(q):
    """Положение и поворот каждого звена цепи (начало звена) и ось его сустава в мире."""
    from make_arm import LINKS
    by_name = {l["name"]: l for l in LINKS}
    frames = {}
    p, r = [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    joint_index = 0
    axes = []
    for name in CHAIN:
        l = by_name[name]
        p = [p[i] + apply(r, l["pos"])[i] for i in range(3)]
        j = l["joint"]
        if j and j[0] == "hinge":
            axis = apply(r, j[2])
            axes.append((p, axis))
            r = matmul(r, rz(q[joint_index]) if j[2] == [0.0, 0.0, 1.0] else ry(q[joint_index]))
            joint_index += 1
        frames[name] = (p, r, l["mass"])
    return frames, axes


def gravity_torques(q, payload_kg=0.0):
    """Моменты тяжести на сустав (знак — как тянет вес), груз в центре захвата."""
    frames, axes = link_frames(q)
    bodies = []
    for name, (p, r, (m, c, _)) in frames.items():
        bodies.append((m, [p[i] + apply(r, c)[i] for i in range(3)], name))
    if payload_kg:
        tcp, _ = forward(q)
        bodies.append((payload_kg, tcp, "груз"))
    order = {name: i for i, name in enumerate(CHAIN)}
    out = []
    for k, (pj, axis) in enumerate(axes):
        tau = 0.0
        for m, c, name in bodies:
            if name != "груз" and order[name] < k:
                continue
            d = [c[i] - pj[i] for i in range(3)]
            f = [m * g for g in G_VEC]
            t = [d[1] * f[2] - d[2] * f[1], d[2] * f[0] - d[0] * f[2], d[0] * f[1] - d[1] * f[0]]
            tau += sum(t[i] * axis[i] for i in range(3))
        out.append(tau)
    return out


STIFFNESS = [2.0e4, 2.0e4, 2.0e4, 600.0, 600.0, 600.0]
HALF_GAP = [0.0, 0.0, 0.0] + [math.radians(0.1) / 2] * 3


def compensate(q, payload_kg=0.0):
    """Задание мотору, при котором выход под весом приходит в q: минус скручивание и полу-люфт."""
    tau = gravity_torques(q, payload_kg)
    return [v - t / k - math.copysign(h, t) * (abs(t) > 0.05) for v, t, k, h in zip(q, tau, STIFFNESS, HALF_GAP)]
