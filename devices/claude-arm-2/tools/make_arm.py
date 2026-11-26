"""Рука Claude, эскиз 2: кронштейн на струбцине у края стола, шесть суставов, весы в кисти, схват под бутылку.

Пишет device.toml (рука), bottle.toml (литровая бутылка с водой), mug.toml (кружка) и world.toml (стол,
рука, бутылка, кружка). Кинематика: поворот Z, плечо Y, локоть Y, вращение предплечья Z, запястье Y,
вращение кисти Z — сферическое запястье, у трёх последних осей общая точка (центр запястья).
В нулевой позе рука смотрит вверх. Размеры и массы — эскиз, не чертёж; источник каждого числа рядом.

Приводы:
- поворот, плечо, локоть — maxon EC 90 flat 48 В + Harmonic Drive CSG-25-50 под позиционным контуром драйвера;
- вращение предплечья, запястье, вращение кисти — CubeMars AK60-6 в серворежиме;
- схват — CubeMars AK60-6 в режиме тока через рейку: компьютер задаёт силу сжатия, а не положение.
Между фланцем и схватом шестиосевой датчик ATI Mini40 (весы в руке).
"""
import math, os

TUNE = {k: float(os.environ.get(k, 1.0)) for k in ("KP", "KI", "GAIN", "BACKLASH", "DAMP")}

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
G = 9.80665

# ---------- геометрия ----------
H_YAW = 0.13        # выход поворота над столешницей
H_SHOULDER = 0.10   # ось плеча над выходом поворота
L1 = 0.30           # плечо: ось плеча — ось локтя
E_ROLL = 0.09       # локоть — вращение предплечья
L2B = 0.16          # вращение предплечья — центр запястья
L2 = E_ROLL + L2B   # локоть — центр запястья
D_FLANGE = 0.06     # центр запястья — выход вращения кисти
D_PLATE = 0.005     # толщина фланца до датчика
D_SENSOR = 0.0122   # высота ATI Mini40
TCP_Z = 0.085       # от основания ладони до центра захвата
TCP = D_FLANGE + D_PLATE + D_SENSOR + TCP_Z   # центр запястья — центр захвата

# Схват: шарики-подушки r 6 мм, по два по оси захвата (±15 мм) и по два вдоль бутылки (±25 мм).
PAD_R = 0.006
PAD_X = 0.0435      # центры подушек неподвижной губы и сжатой подвижной: бутылка Ø80 по центру
PAD_SPREAD = (0.015, 0.025)
OPEN = 0.020        # раскрытие подвижной губы относительно сжатого положения
JAW_T = 0.006

BOTTLE_R = 0.04
BOTTLE_H = 0.27
GRASP = 0.11       # центр захвата над дном бутылки

LIMITS = {
    "поворот": (-170, 170), "плечо": (-30, 115), "локоть": (-10, 160),
    "ротация": (-175, 175), "запястье": (-125, 125), "вращение": (-175, 175),
}

# ---------- палитра ----------
DARK = [0.13, 0.13, 0.15, 1.0]
GRAPH = [0.24, 0.25, 0.28, 1.0]
ALU = [0.78, 0.79, 0.82, 1.0]
WHITE = [0.93, 0.92, 0.89, 1.0]
PRINT = [0.85, 0.46, 0.33, 1.0]
STEEL = [0.55, 0.57, 0.6, 1.0]
RUBBER = [0.1, 0.1, 0.1, 1.0]
BLUE = [0.15, 0.35, 0.8, 1.0]
TABLE = [0.80, 0.74, 0.64, 1.0]
Y = [90.0, 0.0, 0.0]   # цилиндр осью вдоль y
X = [0.0, 90.0, 0.0]   # цилиндр осью вдоль x


def fmt(v):
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(round(v, 9))
    if isinstance(v, int):
        return str(v)
    return '"' + str(v) + '"'


def shape(kind, size):
    if kind == "box":
        return f'{{ kind = "box", size_m = {fmt(list(size))} }}'
    if kind == "sphere":
        return f'{{ kind = "sphere", radius_m = {fmt(size)} }}'
    r, h = size
    return f'{{ kind = "{kind}", radius_m = {fmt(r)}, length_m = {fmt(h)} }}'


def pose(pos, rpy=None):
    s = f"position_m = {fmt([float(p) for p in pos])}"
    if rpy:
        s += f", rpy_deg = {fmt([float(a) for a in rpy])}"
    return f"{{ {s} }}"


# ---------- массы ----------
def rot(rpy):
    """Матрица поворота rpy_deg (x, затем y, затем z) — как у вида: R = Rz·Ry·Rx."""
    r, p, y = (math.radians(a) for a in (rpy or (0, 0, 0)))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def part_inertia(kind, size, m):
    if kind == "box":
        a, b, c = size
        return [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12]
    if kind == "sphere":
        i = 0.4 * m * size * size
        return [i, i, i]
    r, h = size
    t = m * (3 * r * r + h * h) / 12
    return [t, t, 0.5 * m * r * r]


def composite(parts):
    """parts: (масса, форма, размер, позиция, rpy). Возвращает массу, центр и тензор в осях звена."""
    M = sum(p[0] for p in parts)
    c = [sum(p[0] * p[3][i] for p in parts) / M for i in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    for m, kind, size, pos, rpy in parts:
        R = rot(rpy)
        d = part_inertia(kind, size, m)
        local = [[sum(R[i][k] * d[k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        r = [pos[i] - c[i] for i in range(3)]
        rr = sum(x * x for x in r)
        for i in range(3):
            for j in range(3):
                I[i][j] += local[i][j] + m * ((rr if i == j else 0.0) - r[i] * r[j])
    return M, c, [I[0][0], I[1][1], I[2][2], I[0][1], I[0][2], I[1][2]]


EC90 = 1.405     # maxon EC 90 flat 985 г + HD CSG-25 0.42 кг (каталог)
AK60 = 0.368     # CubeMars AK60-6 (каталог)


def maxon(pos, axis_rpy):
    """Привод maxon+HD: двигатель Ø90×32 и волновая ступень Ø85×38, общая масса из каталога."""
    return [(EC90 * 0.7, "cylinder", (0.045, 0.032), pos, axis_rpy), (EC90 * 0.3, "cylinder", (0.0425, 0.038), pos, axis_rpy)]


# ---------- звенья ----------
def ax(axis, offset):
    """Точка на оси вращения (y или z) со сдвигом offset."""
    return {"y": [0.0, offset, 0.0], "z": [0.0, 0.0, offset]}[axis]


def actuator_visual(center, axis, r, length, body, face, ring=None):
    """Корпус привода: цилиндр, торец выхода и кольцо."""
    rpy = Y if axis == "y" else (X if axis == "x" else None)
    out = [("cylinder", (r, length), center, rpy, body)]
    k = {"y": 1, "z": 2, "x": 0}[axis]
    tip = list(center)
    tip[k] += length / 2 + 0.002
    out.append(("cylinder", (r * 0.62, 0.004), tip, rpy, face))
    if ring:
        mid = list(center)
        mid[k] += length * 0.18
        out.append(("cylinder", (r + 0.0012, 0.004), mid, rpy, ring))
    return out


LINKS = []


def link(name, parent, pos, joint, mass_parts, collisions, visuals, provenance, extra=""):
    LINKS.append(dict(name=name, parent=parent, pos=pos, joint=joint, mass=composite(mass_parts),
                      collisions=collisions, visuals=visuals, provenance=provenance, extra=extra))


# Стойка: струбцина на краю стола (x = 0), колонна и статор привода поворота.
link("стойка", None, [0.0, 0.0, 0.0], None,
     [(0.45, "box", (0.10, 0.08, 0.008), [0.03, 0.0, 0.004], None),
      (0.40, "box", (0.012, 0.08, 0.075), [-0.006, 0.0, -0.0295], None),
      (0.30, "box", (0.09, 0.07, 0.008), [0.035, 0.0, -0.063], None),
      (0.25, "cylinder", (0.03, 0.06), [0.0, 0.0, 0.038], None),
      *maxon([0.0, 0.0, 0.103], None)],
     [("box", (0.10, 0.08, 0.008), [0.03, 0.0, 0.004], None),
      ("cylinder", (0.045, 0.062), [0.0, 0.0, 0.099], None)],
     [("box", (0.10, 0.08, 0.008), [0.03, 0.0, 0.004], None, DARK),
      ("box", (0.012, 0.08, 0.075), [-0.006, 0.0, -0.0295], None, DARK),
      ("box", (0.09, 0.07, 0.008), [0.035, 0.0, -0.063], None, DARK),
      ("cylinder", (0.007, 0.06), [0.05, 0.0, -0.085], None, STEEL),
      ("cylinder", (0.004, 0.07), [0.05, 0.0, -0.112], X, STEEL),
      ("cylinder", (0.03, 0.06), [0.0, 0.0, 0.038], None, ALU),
      ("cylinder", (0.045, 0.032), [0.0, 0.0, 0.084], None, GRAPH),
      ("cylinder", (0.0425, 0.03), [0.0, 0.0, 0.115], None, ALU),
      ("cylinder", (0.0462, 0.004), [0.0, 0.0, 0.1], None, PRINT)],
     "сталь струбцины 1.15 кг и алюминиевая колонна 0.25 кг по эскизу; статор и волновая ступень привода поворота 1.405 кг (каталог)")

# Башня: вилка на выходе поворота, на ней привод плеча осью Y.
link("башня", "стойка", [0.0, 0.0, H_YAW], ("hinge", "поворот", [0.0, 0.0, 1.0]),
     [(0.25, "box", (0.07, 0.09, 0.02), [0.0, 0.0, 0.01], None),
      (0.15, "box", (0.06, 0.008, 0.10), [0.0, -0.052, 0.06], None),
      *maxon([0.0, 0.058, H_SHOULDER], Y)],
     [("box", (0.07, 0.09, 0.02), [0.0, 0.0, 0.01], None),
      ("cylinder", (0.045, 0.07), [0.0, 0.058, H_SHOULDER], Y)],
     [("cylinder", (0.046, 0.02), [0.0, 0.0, 0.01], None, PRINT),
      ("box", (0.06, 0.008, 0.11), [0.0, -0.052, 0.055], None, PRINT),
      *actuator_visual([0.0, 0.058, H_SHOULDER], "y", 0.045, 0.07, GRAPH, ALU, PRINT)],
     "вилка из алюминия 0.4 кг по эскизу; привод плеча 1.405 кг (каталог)")

# Плечо: труба 40 мм из углепластика, на дальнем конце привод локтя осью Y с другой стороны.
link("плечо", "башня", [0.0, 0.0, H_SHOULDER], ("hinge", "плечо", [0.0, 1.0, 0.0]),
     [(0.22, "cylinder", (0.02, 0.24), [0.0, 0.0, 0.15], None),
      (0.12, "box", (0.06, 0.03, 0.05), [0.0, 0.0, 0.02], None),
      (0.10, "box", (0.06, 0.03, 0.05), [0.0, 0.0, L1 - 0.02], None),
      *maxon([0.0, -0.058, L1], Y)],
     [("capsule", (0.022, 0.20), [0.0, 0.0, 0.15], None),
      ("cylinder", (0.045, 0.07), [0.0, -0.058, L1], Y)],
     [("box", (0.06, 0.03, 0.05), [0.0, 0.0, 0.02], None, PRINT),
      ("cylinder", (0.02, 0.24), [0.0, 0.0, 0.15], None, DARK),
      ("box", (0.06, 0.03, 0.05), [0.0, 0.0, L1 - 0.02], None, PRINT),
      *actuator_visual([0.0, -0.058, L1], "y", 0.045, 0.07, GRAPH, ALU, PRINT)],
     "труба из углепластика 0.22 кг и две алюминиевые проушины 0.22 кг по эскизу; привод локтя 1.405 кг (каталог)")

# Предплечье: от локтя до привода вращения предплечья (AK60-6 осью Z).
link("предплечье", "плечо", [0.0, 0.0, L1], ("hinge", "локоть", [0.0, 1.0, 0.0]),
     [(0.12, "box", (0.05, 0.03, 0.05), [0.0, 0.0, 0.025], None),
      (AK60, "cylinder", (0.04, 0.036), [0.0, 0.0, E_ROLL - 0.02], None)],
     [("box", (0.05, 0.03, 0.05), [0.0, 0.0, 0.025], None),
      ("cylinder", (0.04, 0.036), [0.0, 0.0, E_ROLL - 0.02], None)],
     [("box", (0.05, 0.03, 0.05), [0.0, 0.0, 0.025], None, PRINT),
      *actuator_visual([0.0, 0.0, E_ROLL - 0.02], "z", 0.04, 0.036, DARK, ALU, BLUE)],
     "алюминиевая проушина 0.12 кг по эскизу; AK60-6 0.368 кг (каталог)")

# Рукав: вращается вокруг оси предплечья, несёт привод запястья (AK60-6 осью Y).
link("рукав", "предплечье", [0.0, 0.0, E_ROLL], ("hinge", "ротация", [0.0, 0.0, 1.0]),
     [(0.10, "cylinder", (0.018, 0.10), [0.0, 0.0, 0.055], None),
      (0.06, "box", (0.05, 0.02, 0.04), [0.0, -0.035, L2B - 0.01], None),
      (AK60, "cylinder", (0.04, 0.036), [0.0, 0.035, L2B], Y)],
     [("cylinder", (0.018, 0.10), [0.0, 0.0, 0.055], None),
      ("cylinder", (0.04, 0.036), [0.0, 0.035, L2B], Y)],
     [("cylinder", (0.018, 0.10), [0.0, 0.0, 0.055], None, WHITE),
      ("box", (0.05, 0.008, 0.06), [0.0, -0.035, L2B - 0.02], None, PRINT),
      *actuator_visual([0.0, 0.035, L2B], "y", 0.04, 0.036, DARK, ALU, BLUE)],
     "трубка и вилка запястья 0.16 кг по эскизу; AK60-6 0.368 кг (каталог)")

# Запястье: несёт привод вращения кисти (AK60-6 осью Z).
link("запястье", "рукав", [0.0, 0.0, L2B], ("hinge", "запястье", [0.0, 1.0, 0.0]),
     [(0.05, "box", (0.04, 0.03, 0.02), [0.0, 0.0, 0.01], None),
      (AK60, "cylinder", (0.04, 0.036), [0.0, 0.0, D_FLANGE - 0.02], None)],
     [("cylinder", (0.04, 0.036), [0.0, 0.0, D_FLANGE - 0.02], None)],
     [("box", (0.04, 0.03, 0.03), [0.0, 0.0, 0.01], None, PRINT),
      *actuator_visual([0.0, 0.0, D_FLANGE - 0.02], "z", 0.04, 0.036, DARK, ALU, BLUE)],
     "проушина 0.05 кг по эскизу; AK60-6 0.368 кг (каталог)")

# Кисть: фланец на выходе вращения кисти.
link("кисть", "запястье", [0.0, 0.0, D_FLANGE], ("hinge", "вращение", [0.0, 0.0, 1.0]),
     [(0.03, "cylinder", (0.025, D_PLATE), [0.0, 0.0, D_PLATE / 2], None)],
     [("cylinder", (0.025, D_PLATE), [0.0, 0.0, D_PLATE / 2], None)],
     [("cylinder", (0.025, D_PLATE), [0.0, 0.0, D_PLATE / 2], None, ALU)],
     "алюминиевый фланец 30 г по эскизу",
     extra='\n[[links.mounts]]\nname = "фланец"\nkind = "mechanical"\ninterface = "эскизный фланец ISO 9409-1-31.5-4-M5"\n'
           f'pose = {pose([0.0, 0.0, D_PLATE])}\n')

# Датчик: инструментальная половина ATI Mini40. Его реакция — сила и момент на всё, что дальше.
link("датчик", "кисть", [0.0, 0.0, D_PLATE], ("fixed",),
     [(0.025, "cylinder", (0.02, D_SENSOR), [0.0, 0.0, D_SENSOR / 2], None)],
     [],
     [("cylinder", (0.02, D_SENSOR), [0.0, 0.0, D_SENSOR / 2], None, ALU),
      ("cylinder", (0.0203, 0.003), [0.0, 0.0, D_SENSOR * 0.7], None, BLUE)],
     "половина массы ATI Mini40 (50 г всего, ATI) за чувствительным элементом")

# Камера глубины на ладони: Intel RealSense D405 (42 × 42 × 23 мм, 60 г) на уголке сбоку от губ, по оси y
# ладони. Смотрит вдоль подхода. При хвате ось бутылки идёт по y ладони, но корпус бутылки начинается
# только с z = 45 мм — камера позади него. Оптический центр — 1 мм перед лицевой гранью.
CAM_BOX = (0.042, 0.042, 0.023)
CAM_CENTER = [0.0, 0.068, 0.004 + CAM_BOX[2] / 2]
CAM_OPTICAL = [0.0, 0.068, 0.004 + CAM_BOX[2] + 0.001]
# Ошибка крепления экземпляра — остаток калибровки «рука — глаз», оценка. Контроллер её не знает.
CAM_MOUNT_ERROR = {"mount_dx_m": 0.0004, "mount_dy_m": -0.0003, "mount_dz_m": 0.0005,
                   "mount_roll_rad": math.radians(0.15), "mount_pitch_rad": math.radians(-0.1), "mount_yaw_rad": math.radians(0.2)}

# Ладонь: привод схвата плашмя, рамка, неподвижная губа с четырьмя подушками, камера.
fixed_pads = [(-PAD_X, sy * PAD_SPREAD[1], TCP_Z + sz * PAD_SPREAD[0]) for sy in (-1, 1) for sz in (-1, 1)]
link("ладонь", "датчик", [0.0, 0.0, D_SENSOR], ("fixed",),
     [(AK60, "cylinder", (0.04, 0.034), [0.0, 0.0, 0.017], None),
      (0.06, "box", (0.13, 0.07, 0.005), [0.0, 0.0, 0.0375], None),
      (0.04, "box", (JAW_T, 0.07, 0.07), [-PAD_X - PAD_R - JAW_T / 2, 0.0, TCP_Z], None),
      (0.07, "box", CAM_BOX, CAM_CENTER, None)],
     [("cylinder", (0.04, 0.034), [0.0, 0.0, 0.017], None),
      ("box", (0.13, 0.07, 0.005), [0.0, 0.0, 0.0375], None),
      ("box", (JAW_T, 0.07, 0.07), [-PAD_X - PAD_R - JAW_T / 2, 0.0, TCP_Z], None),
      *[("sphere", PAD_R, list(p), None) for p in fixed_pads],
      ("box", CAM_BOX, CAM_CENTER, None)],
     [*actuator_visual([0.0, 0.0, 0.017], "z", 0.04, 0.034, DARK, ALU, BLUE),
      ("box", (0.13, 0.07, 0.005), [0.0, 0.0, 0.0375], None, ALU),
      ("box", (JAW_T, 0.07, 0.07), [-PAD_X - PAD_R - JAW_T / 2, 0.0, TCP_Z], None, GRAPH),
      *[("sphere", PAD_R, list(p), None, RUBBER) for p in fixed_pads],
      ("box", CAM_BOX, CAM_CENTER, None, DARK)],
     "привод схвата 0.368 кг (каталог), рамка 60 г и губа 40 г по эскизу; камера D405 60 г (паспорт) с уголком 10 г",
     extra='\n[[links.mounts]]\nname = "захват"\nkind = "mechanical"\ninterface = "центр захвата ладони"\n'
           f'pose = {pose([0.0, 0.0, TCP_Z])}\n')

jaw_x = PAD_X + OPEN + PAD_R + JAW_T / 2
moving_pads = [(-(PAD_R + JAW_T / 2), sy * PAD_SPREAD[1], sz * PAD_SPREAD[0]) for sy in (-1, 1) for sz in (-1, 1)]
link("губа", "ладонь", [jaw_x, 0.0, TCP_Z], ("slide", "схват", [-1.0, 0.0, 0.0]),
     [(0.04, "box", (JAW_T, 0.07, 0.07), [0.0, 0.0, 0.0], None)],
     [("box", (JAW_T, 0.07, 0.07), [0.0, 0.0, 0.0], None),
      *[("sphere", PAD_R, list(p), None) for p in moving_pads]],
     [("box", (JAW_T, 0.07, 0.07), [0.0, 0.0, 0.0], None, GRAPH),
      *[("sphere", PAD_R, list(p), None, RUBBER) for p in moving_pads]],
     "подвижная губа с кареткой 40 г по эскизу")

PAD_FRICTION = [0.9, 0.005, 0.0001]   # резина по ПЭТ, оценка


# ---------- приводы ----------
def maxon_params(limit_a, joint, load_j, damp_j, note):
    k = 0.231
    w0 = 1960 * 2 * math.pi / 60
    j_rot = (0.000506 + 4.13e-5) * 50 ** 2
    stiffness = 2.0e4
    lo, hi = LIMITS[joint]
    return [
        ("# " + note, None),
        ("# Ток защиты драйвера: момент на выходе k·N·η·I.", None),
        ("current_limit_a", limit_a),
        ("# k = K_t maxon 500267 (каталог, 231 мН·м/А): в СИ постоянная момента равна постоянной ЭДС.", None),
        ("motor_constant", k),
        ("# Обратный КПД волновой передачи принят равным прямому 0.7 (оценка каталога).", None),
        ("backward_efficiency", 0.7),
        ("# B = k·I₀/ω₀ мотора: I₀ = 0.278 А, 1960 об/мин без нагрузки (maxon 500267). Сухое трение не выделено.", None),
        ("viscous_nm_per_rad_s", k * 0.278 / w0),
        ("coulomb_nm", 0.0), ("static_friction_nm", 0.0),
        ("stribeck_rad_s", 10.0), ("friction_band_rad_s", 10.0),
        ("# Волновая передача беззазорна; её потерянный ход — податливость, она в coupling_stiffness.", None),
        ("backlash", 0.0),
        ("# Крутильная жёсткость волновой передачи типоразмера 25 — порядок по каталогу HD, не сверено.", None),
        ("coupling_stiffness", stiffness),
        (f"# c = 2ζ√(K/w), ζ = 0.3 (конструкционное затухание передачи, оценка), w = {damp_j} 1/(кг·м²) — обратная", None),
        ("# инерция оси с ротором и связанными звеньями в позе с наименьшей нагрузкой (проверка шага MuJoCo).", None),
        ("coupling_damping", 2 * 0.3 * TUNE["DAMP"] * math.sqrt(stiffness / damp_j)),
        ("# Позиционный и скоростной контур драйвера 2.5 кГц (класс EPOS4), задержка шины 0.5 мс — оценки.", None),
        ("control_hz", 2500.0), ("bus_delay_s", 0.0005),
        ("position_min", math.radians(lo)), ("position_max", math.radians(hi)),
        ("# Без мёртвой зоны: интеграл держит груз без подработки.", None),
        ("position_deadband", 0.0),
        ("position_gain", 20.0 * TUNE["GAIN"]),
        ("speed_limit", 3.0),
        ("# Скоростной ПИ: полоса около 60 рад/с при Jрот·N² + Jнагр, момент на ток k·N·η = 8.1 Н·м/А.", None),
        ("speed_kp", round(60 * TUNE["KP"] * (j_rot + load_j) / (k * 50 * 0.7), 3)),
        ("speed_ki", round(15 * 60 * TUNE["KI"] * (j_rot + load_j) / (k * 50 * 0.7), 3)),
        ("# Тепловая сеть: R и τ из каталога; α меди, среда 25 °C, потери обмотки в обмотке, передачи — в корпусе (оценки).", None),
        ("ambient_c", 25.0), ("resistance_alpha_per_k", 0.00393),
        ("winding_loss_fraction", 1.0), ("gear_loss_to_housing_fraction", 1.0),
    ]


def ak60_params(limit_a, joint, load_j, damp_j, note, position=True):
    k = 0.078
    eta = 0.9
    j_rot = 2.435e-5 * 36
    stiffness = 600.0
    rows = [
        ("# " + note, None),
        ("current_limit_a", limit_a),
        ("# k = K_t CubeMars AK60-6 KV140 (каталог, 0.078 Н·м/А).", None),
        ("motor_constant", k),
        ("# КПД планетарной ступени 6:1 — оценка: 3 Н·м при 6.5 А по паспорту дают 0.99, пик 9 Н·м при 22.7 А — 0.85.", None),
        ("gear_efficiency", eta), ("backward_efficiency", eta),
        ("# Ток холостого хода не опубликован: вязкое трение оценено малым.", None),
        ("viscous_nm_per_rad_s", 2.0e-5),
        ("coulomb_nm", 0.0), ("static_friction_nm", 0.0),
        ("stribeck_rad_s", 10.0), ("friction_band_rad_s", 10.0),
        ("gear_input_inertia_kg_m2", 0.0),
        ("# Люфт планетарной ступени 0.1° и жёсткость 600 Н·м/рад — гипотезы до замера.", None),
        ("backlash", math.radians(0.1)),
        ("coupling_stiffness", stiffness),
        (f"# c = 2ζ√(K/w), ζ = 0.3 (оценка), w = {damp_j} 1/(кг·м²) — обратная инерция оси с ротором в лёгкой позе.", None),
        ("coupling_damping", 2 * 0.3 * math.sqrt(stiffness / damp_j)),
        ("control_hz", 2500.0), ("bus_delay_s", 0.0005),
    ]
    if position:
        lo, hi = LIMITS[joint]
        kt_out = k * 6 * eta
        rows += [
            ("position_min", math.radians(lo)), ("position_max", math.radians(hi)),
            ("position_deadband", 0.0),
            ("position_gain", 30.0),
            ("speed_limit", 6.0),
            ("# Скоростной ПИ: полоса около 80 рад/с с бутылкой, момент на ток 0.42 Н·м/А.", None),
            ("speed_kp", round(80 * (j_rot + load_j) / kt_out, 3)),
            ("speed_ki", round(20 * 80 * (j_rot + load_j) / kt_out, 3)),
        ]
    return rows


TRAVEL = 2 * math.pi * 0.01   # шестерня r = 10 мм на рейке губы
DRIVES = [
    ("привод-поворота", "maxon-ec90-hd-csg25-50-position", "поворот",
     maxon_params(4.0, "поворот", 0.6, 115.0, "Поворот: груз не поднимает, ток 4 А (32 Н·м) — для разгона.")),
    ("привод-плеча", "maxon-ec90-hd-csg25-50-position", "плечо",
     maxon_params(8.0, "плечо", 0.8, 30.0, "Плечо: статически до 18 Н·м с бутылкой на полном вылете, ток 8 А (65 Н·м) с запасом на разгон.")),
    ("привод-локтя", "maxon-ec90-hd-csg25-50-position", "локоть",
     maxon_params(8.0, "локоть", 0.3, 72.5, "Локоть: статически до 7 Н·м.")),
    ("привод-ротации", "cubemars-ak60-6-position", "ротация",
     ak60_params(9.0, "ротация", 0.02, 1800.0, "Вращение предплечья: до 2.3 Н·м при согнутом запястье, ток 9 А (3.8 Н·м).")),
    ("привод-запястья", "cubemars-ak60-6-position", "запястье",
     ak60_params(9.0, "запястье", 0.04, 1400.0, "Запястье: до 2.3 Н·м с бутылкой при горизонтальной кисти, ток 9 А (3.8 Н·м).")),
    ("привод-кисти", "cubemars-ak60-6-position", "вращение",
     ak60_params(6.0, "вращение", 0.01, 2700.0, "Вращение кисти: момент от смещения бутылки и воды, до 0.5 Н·м.")),
    ("привод-схвата", "cubemars-ak60-6", "схват",
     [("# Схват в режиме тока: команда — ток, сила сжатия F = k·N·η·I / r, около 42 Н на ампер.", None),
      ("# Рейка: шестерня r = 10 мм, ход за оборот выходного вала 2π·r.", None),
      ("linear_travel_m_per_rev", TRAVEL)]
     + [r for r in ak60_params(2.0, "схват", 0.04, 1800.0, "Ток до 2 А: сжатие до 84 Н.", position=False)
        if r[0] not in ("coupling_stiffness", "coupling_damping", "backlash") and not str(r[0]).startswith("# c =")
        and not str(r[0]).startswith("# Люфт")]
     + [("# Люфт и податливость рейки, приведённые к ходу губы: 0.1° на шестерне 10 мм и 2 мм/100 Н — гипотезы.", None),
        ("backlash", math.radians(0.1) * 0.01),
        ("coupling_stiffness", 5.0e4),
        ("# c = 2ζ√(K·m), m = 0.04 кг губы плюс ротор, приведённый к ходу.", None),
        ("coupling_damping", 2 / math.sqrt(2) * math.sqrt(5.0e4 * 0.04))]),
]

ENCODER_JOINTS = [("поворот", "башня"), ("плечо", "плечо"), ("локоть", "предплечье"),
                  ("ротация", "рукав"), ("запястье", "запястье"), ("вращение", "кисть")]


def encoder(name, link_name, joint, quantum, note):
    return "\n".join([
        "[[sensors]]", f'name = "{name}"', 'part = "as5048a"', f'link = "{link_name}"', f'joint = "{joint}"', "",
        "[sensors.params]", f"# {note}",
        "rate_hz = 1000.0", "latency_s = 0.001", "encoder_mode = 0.0", "encoder_zero = 0.0",
        f"quantum = {fmt(quantum)}", "bias = 0.0", "noise_std = 0.0", "drift_per_s = 0.0", "drift_rw_std = 0.0",
        "drop_probability = 0.0", "fault = 0.0", "saturation = 0.0", ""])


def device():
    out = ['# Рука Claude, эскиз 2: кронштейн на струбцине. Пишет tools/make_arm.py, не править руками.',
           f'# Ось поворота на краю стола, выход поворота {H_YAW * 1000:.0f} мм над столешницей, ось плеча ещё {H_SHOULDER * 1000:.0f} мм выше.',
           f'# Плечо {L1 * 1000:.0f} мм, локоть — центр запястья {L2 * 1000:.0f} мм, центр запястья — центр захвата {TCP * 1000:.1f} мм.',
           'format = 3', 'name = "Рука Claude, кронштейн"', '']
    for l in LINKS:
        out += ["[[links]]", f'name = "{l["name"]}"']
        if l["parent"]:
            out.append(f'parent = "{l["parent"]}"')
        out.append(f"pose = {pose(l['pos'])}")
        out.append("")
        j = l["joint"]
        if j is None or j[0] == "fixed":
            out += ["[links.joint]", 'kind = "fixed"', ""]
        else:
            kind, name, axis = j
            out += ["[links.joint]", f'kind = "{kind}"', f'name = "{name}"', f"axis = {fmt(axis)}"]
            if kind == "hinge":
                out.append(f"limits_deg = {fmt(list(LIMITS[name]))}")
            else:
                out.append(f"limits_m = {fmt([0.0, 0.03])}")
            out += ["damping = 0.0", "friction_loss = 0.0", "armature = 0.0"]
            efforts = [f'{{ name = "{name}", dof = 0 }}']
            if name == "плечо":
                efforts.append('{ name = "толчок-плеча", dof = 0 }')
            out.append(f"efforts = [{', '.join(efforts)}]")
            out.append("")
        m, c, I = l["mass"]
        out += ["[links.mass]", 'kind = "explicit"', f"mass_kg = {fmt(m)}", f"com_m = {fmt(c)}",
                f"inertia_kg_m2 = {fmt(I)}", f'provenance = {{ estimate = "{l["provenance"]}" }}', ""]
        for kind, size, pos_, rpy, rgba in l["visuals"]:
            out += ["[[links.visual]]", f"shape = {shape(kind, size)}", f"pose = {pose(pos_, rpy)}", f"rgba = {fmt(rgba)}", ""]
        for kind, size, pos_, rpy in l["collisions"]:
            out += ["[[links.collision]]", f"shape = {shape(kind, size)}", f"pose = {pose(pos_, rpy)}"]
            if kind == "sphere":
                out.append(f"friction = {fmt(PAD_FRICTION)}")
            out.append("")
        if l["extra"]:
            out.append(l["extra"])
    for name, part, effort, params in DRIVES:
        out += ["[[drives]]", f'name = "{name}"', f'part = "{part}"', f'effort = "{effort}"', "", "[drives.params]"]
        for key, value in params:
            out.append(key if value is None else f"{key} = {fmt(value)}")
        out.append("")
    for joint, link_name in ENCODER_JOINTS:
        out.append(encoder(f"энкодер-{joint}", link_name, joint, 2 * math.pi / 16384,
                           "AS5048A на выходе сустава: 14 бит на оборот. Опрос 1 кГц, доставка через такт."))
    out.append(encoder("энкодер-схвата", "губа", "схват", TRAVEL / 16384,
                       "AS5048A на шестерне схвата: 14 бит на оборот, приведено к ходу губы."))
    for name, part, unit in (("сила", "ati-mini40-si-40-2-force", "Н"), ("момент", "ati-mini40-si-40-2-torque", "Н·м")):
        out += ["[[sensors]]", f'name = "{name}"', f'part = "{part}"', 'link = "датчик"',
                f"# Реакция между фланцем и инструментальной половиной ATI Mini40 в осях кисти, {unit}. Паспорт — в каталоге.", ""]
    out += ["[[sensors]]", 'name = "камера"', 'part = "intel-realsense-d405-depth"', 'link = "ладонь"',
            f"pose = {pose(CAM_OPTICAL)}",
            "# Оси кадра совпадают с осями ладони: X — сжатие, Y — вдоль губ, Z — подход.", "",
            "[sensors.params]",
            "# Ошибка крепления экземпляра: остаток калибровки «рука — глаз», оценка. Контроллер знает только номинал.",
            *[f"{k} = {fmt(v)}" for k, v in CAM_MOUNT_ERROR.items()], ""]
    return "\n".join(out)


# ---------- бутылка и кружка ----------
# Литровая ПЭТ-бутылка: корпус Ø80 до 195 мм, плечики до 235 мм, горло Ø27 (внутри 21.7 мм) до 270 мм.
BOTTLE_PROFILE = [[0.0, 0.0395], [0.194, 0.0395], [0.234, 0.01085], [0.269, 0.01085]]
WATER_ML = 1000.0


def frustum(r1, r2, h):
    return math.pi * h / 3 * (r1 * r1 + r1 * r2 + r2 * r2)


def bottle_water():
    """Центр тяжести и объём полной стоящей бутылки (для массы тела)."""
    p = BOTTLE_PROFILE
    vol, mom, z = 0.0, 0.0, 0.0
    target = WATER_ML * 1e-6
    # слоями по 0.1 мм
    dz = 1e-4
    while vol < target and z < p[-1][0]:
        zz = z + dz / 2
        for (z0, r0), (z1, r1) in zip(p, p[1:]):
            if z0 <= zz <= z1:
                r = r0 + (r1 - r0) * (zz - z0) / (z1 - z0)
        dv = math.pi * r * r * dz
        vol += dv
        mom += dv * zz
        z += dz
    return vol, mom / vol, z


def bottle():
    vol, zc, level = bottle_water()
    water = 998.2 * WATER_ML * 1e-6
    pet = 0.035
    m = water + pet
    # Вода — столб до уровня, ПЭТ — тонкая оболочка; инерция как у сплошного цилиндра высотой до уровня.
    com = [0.0, 0.0, (water * (zc + 0.001) + pet * 0.12) / m]
    ixx = water * (3 * 0.0395 ** 2 + level ** 2) / 12 + water * ((zc + 0.001) - com[2]) ** 2 + pet * (0.04 ** 2 / 2 + 0.27 ** 2 / 12)
    izz = water * 0.0395 ** 2 / 2 + pet * 0.04 ** 2
    shell = [0.5, 0.75, 0.95, 0.35]
    lines = ["# Литровая ПЭТ-бутылка с 1 л воды, крышка снята. Пишет tools/make_arm.py.",
             f"# Вода входит в массу тела неподвижно (уровень {level * 1000:.0f} мм, центр {zc * 1000:.1f} мм от внутреннего дна);",
             "# процесс «жидкость» прикладывает разницу с фактической водой.",
             'format = 3', 'name = "бутылка"', "", "[[links]]", 'name = "корпус"', "",
             "[links.joint]", 'kind = "free"', 'name = "корпус"', "damping = 0.0", "friction_loss = 0.0", "armature = 0.0", "",
             "[links.mass]", 'kind = "explicit"', f"mass_kg = {fmt(m)}", f"com_m = {fmt(com)}",
             f"inertia_kg_m2 = {fmt([ixx, ixx, izz, 0.0, 0.0, 0.0])}",
             f'provenance = {{ estimate = "вода 998.2 кг/м³ при 20 °C, 1 л; ПЭТ 35 г — типичная литровая бутылка, не взвешено" }}', ""]
    vis = [("cylinder", (0.04, 0.195), [0.0, 0.0, 0.0975], shell),
           ("cylinder", (0.0395, level), [0.0, 0.0, 0.001 + level / 2], [0.35, 0.6, 0.95, 0.55]),
           ("cylinder", (0.026, 0.04), [0.0, 0.0, 0.215], shell),
           ("cylinder", (0.0135, 0.035), [0.0, 0.0, 0.2525], shell),
           ("cylinder", (0.0165, 0.003), [0.0, 0.0, 0.237], [0.9, 0.9, 0.95, 0.8]),
           ("cylinder", (0.0405, 0.06), [0.0, 0.0, 0.11], [0.2, 0.45, 0.75, 0.9])]
    for kind, size, pos_, rgba in vis:
        lines += ["[[links.visual]]", f"shape = {shape(kind, size)}", f"pose = {pose(pos_)}", f"rgba = {fmt(rgba)}", ""]
    for size, pos_ in (((0.04, 0.195), [0.0, 0.0, 0.0975]), ((0.026, 0.04), [0.0, 0.0, 0.215]), ((0.0135, 0.035), [0.0, 0.0, 0.2525])):
        lines += ["[[links.collision]]", f"shape = {shape('cylinder', size)}", f"pose = {pose(pos_)}", "friction = [0.5, 0.005, 0.0001]", ""]
    # Хват: середина корпуса в осях ладони — y вдоль оси бутылки, z — направление подхода.
    lines += ["[[links.mounts]]", 'name = "хват"', 'kind = "mechanical"', 'interface = "центр захвата ладони"',
              f"pose = {pose([0.0, 0.0, GRASP], [90.0, 0.0, 90.0])}", ""]
    return "\n".join(lines)


MUG_R_IN, MUG_R_OUT, MUG_H, MUG_FLOOR = 0.037, 0.041, 0.095, 0.006


def mug():
    m = 0.32
    walls = 16
    lines = ["# Керамическая кружка: внутри Ø74 × 89 мм (383 мл), стенка 4 мм, дно 6 мм. Пишет tools/make_arm.py.",
             'format = 3', 'name = "кружка"', "", "[[links]]", 'name = "корпус"', "",
             "[links.joint]", 'kind = "free"', 'name = "корпус"', "damping = 0.0", "friction_loss = 0.0", "armature = 0.0", ""]
    parts = [(0.06, "cylinder", (MUG_R_OUT, MUG_FLOOR), [0.0, 0.0, MUG_FLOOR / 2], None),
             (0.22, "cylinder", (MUG_R_OUT, MUG_H), [0.0, 0.0, MUG_H / 2], None),
             (0.04, "box", (0.03, 0.012, 0.06), [-0.058, 0.0, 0.05], None)]
    M, c, I = composite(parts)
    # Стенка — тонкая оболочка: момент вокруг оси ≈ m·r² вместо m·r²/2.
    I[2] += 0.22 * MUG_R_OUT ** 2 / 2
    lines += ["[links.mass]", 'kind = "explicit"', f"mass_kg = {fmt(M)}", f"com_m = {fmt(c)}", f"inertia_kg_m2 = {fmt(I)}",
              'provenance = { estimate = "керамика 320 г — типичная кружка на 350 мл, не взвешено" }', ""]
    white = [0.95, 0.95, 0.93, 1.0]
    lines += ["[[links.visual]]", f"shape = {shape('cylinder', (MUG_R_OUT, MUG_FLOOR))}", f"pose = {pose([0, 0, MUG_FLOOR / 2])}", f"rgba = {fmt(white)}", ""]
    width = 2 * math.pi * (MUG_R_IN + MUG_R_OUT) / 2 / walls * 1.08
    for i in range(walls):
        a = 2 * math.pi * i / walls
        r = (MUG_R_IN + MUG_R_OUT) / 2
        p = [r * math.cos(a), r * math.sin(a), MUG_FLOOR + (MUG_H - MUG_FLOOR) / 2]
        size = (MUG_R_OUT - MUG_R_IN, width, MUG_H - MUG_FLOOR)
        rpy = [0.0, 0.0, math.degrees(a)]
        lines += ["[[links.visual]]", f"shape = {shape('box', size)}", f"pose = {pose(p, rpy)}", f"rgba = {fmt(white)}", ""]
        lines += ["[[links.collision]]", f"shape = {shape('box', size)}", f"pose = {pose(p, rpy)}", "friction = [0.6, 0.005, 0.0001]", ""]
    for size, p in (((0.006, 0.012, 0.06), [-0.068, 0.0, 0.05]), ((0.03, 0.012, 0.006), [-0.055, 0.0, 0.077]), ((0.03, 0.012, 0.006), [-0.055, 0.0, 0.023])):
        lines += ["[[links.visual]]", f"shape = {shape('box', size)}", f"pose = {pose(p)}", f"rgba = {fmt(white)}", ""]
    lines += ["[[links.collision]]", f"shape = {shape('cylinder', (MUG_R_OUT, MUG_FLOOR))}", f"pose = {pose([0, 0, MUG_FLOOR / 2])}", "friction = [0.6, 0.005, 0.0001]", ""]
    lines += ["[[links.collision]]", f"shape = {shape('box', (0.03, 0.012, 0.06))}", f"pose = {pose([-0.058, 0.0, 0.05])}", ""]
    return "\n".join(lines)


BOTTLE_AT = (0.45, -0.10)
MUG_AT = (0.45, 0.10)
# Раскладки стола: где бутылка, где кружка и куда смотрит ручка кружки (поворот вокруг z, °).
# Контроллер их не знает: находит предметы камерой. Кружка — слева от бутылки (по +y от неё).
LAYOUTS = {
    "": (BOTTLE_AT, MUG_AT, 0.0),
    "b": ((0.40, -0.06), (0.50, 0.15), 120.0),
    "c": ((0.50, -0.14), (0.41, 0.06), 250.0),
}


def world(physics_hz=30000, extra="", layout=""):
    bottle_at, mug_at, mug_yaw = LAYOUTS[layout]
    return "\n".join([
        "# Рука Claude, эскиз 2, на краю стола; литровая бутылка воды и кружка. Пишет tools/make_arm.py.",
        "format = 3", 'name = "Кронштейн, бутылка и кружка"', "seed = 11", f"physics_hz = {physics_hz}",
        f"gravity_m_s2 = [0.0, 0.0, {-G}]", "",
        "[environment]", "density_kg_m3 = 0.0", "viscosity_pa_s = 0.0", "wind_m_s = [0.0, 0.0, 0.0]", "",
        "# Столешница 0.8 × 1.0 × 0.04 м, край у x = 0, верх на z = 0.",
        "[[environment.collision]]", 'shape = { kind = "box", size_m = [0.8, 1.0, 0.04] }',
        "pose = { position_m = [0.4, 0.0, -0.02] }", "friction = [0.4, 0.005, 0.0001]", "",
        "[[environment.visual]]", 'shape = { kind = "box", size_m = [0.8, 1.0, 0.04] }',
        "pose = { position_m = [0.4, 0.0, -0.02] }", f"rgba = {fmt(TABLE)}", "",
        "[[devices]]", 'name = "рука"', 'file = "device.toml"', "pose = { position_m = [0.0, 0.0, 0.0] }", "",
        "[[devices]]", 'name = "бутылка"', 'file = "bottle.toml"', f"pose = {{ position_m = [{bottle_at[0]}, {bottle_at[1]}, 0.0] }}", "",
        "[[devices]]", 'name = "кружка"', 'file = "mug.toml"',
        f"pose = {{ position_m = [{mug_at[0]}, {mug_at[1]}, 0.0], rpy_deg = [0.0, 0.0, {mug_yaw}] }}", "",
        extra])


def main():
    files = [("device.toml", device()), ("bottle.toml", bottle()), ("mug.toml", mug())]
    files += [(f"world{'-' + k if k else ''}.toml", world(layout=k)) for k in LAYOUTS]
    for name, text in files:
        open(os.path.join(ROOT, name), "w").write(text)
    mass = sum(l["mass"][0] for l in LINKS if l["name"] != "стойка")
    print(f"записано: {', '.join(n for n, _ in files)}; подвижная масса руки {mass:.3f} кг, "
          f"центр запястья — захват {TCP * 1000:.1f} мм, бутылка {998.2e-3 + 0.035:.3f} кг")


if __name__ == "__main__":
    main()
