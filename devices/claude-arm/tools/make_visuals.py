"""Внешний вид руки, насадок и стола: только `[[links.visual]]` и `[[environment.visual]]`.

Физика не меняется: вид не сталкивается и не даёт массы (масса звеньев задана явно). Генератор
переписывает блоки вида в device.toml, device-gripper.toml, tool-probe.toml, tool-marker.toml и в
world.toml, world-gripper.toml. Мир насадок пишет tools/make_tools.py, он берёт вид стола и держателей
отсюда. Размеры сервы — Feetech STS3215, 45.2 × 24.7 × 35 мм; остальное — эскиз.
"""
import math, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MARK = "# вид: tools/make_visuals.py"

SERVO = [0.12, 0.12, 0.14, 1.0]
HORN = [0.78, 0.79, 0.82, 1.0]
PRINT = [0.85, 0.46, 0.33, 1.0]
LIGHT = [0.94, 0.92, 0.87, 1.0]
BELT = [0.08, 0.08, 0.08, 1.0]
STEEL = [0.66, 0.68, 0.72, 1.0]
DARK = [0.25, 0.26, 0.29, 1.0]
RUBY = [0.78, 0.08, 0.15, 1.0]
BLUE = [0.15, 0.35, 0.80, 1.0]
TABLE = [0.80, 0.74, 0.64, 1.0]
STAND = [0.32, 0.34, 0.38, 1.0]
Y = [90.0, 0.0, 0.0]  # цилиндр осью вдоль y


def box(size, pos, rgba, rpy=None):
    return ("box", f"size_m = {list(size)}", pos, rpy, rgba)


def cyl(r, length, pos, rgba, rpy=None):
    return ("cylinder", f"radius_m = {r}, length_m = {length}", pos, rpy, rgba)


def capsule(r, length, pos, rgba):
    return ("capsule", f"radius_m = {r}, length_m = {length}", pos, None, rgba)


def sphere(r, pos, rgba):
    return ("sphere", f"radius_m = {r}", pos, None, rgba)


def belt(y, small, big):
    """Две ветви ремня между шкивами в плоскости xz: (x, z, радиус) малого и большого."""
    out = []
    for side in (-1, 1):
        x1, z1 = small[0] + side * small[2], small[1]
        x2, z2 = big[0] + side * big[2], big[1]
        length = math.hypot(x2 - x1, z2 - z1)
        angle = math.degrees(math.atan2(x2 - x1, z2 - z1))
        out.append(box([0.0015, 0.005, round(length, 5)], [round((x1 + x2) / 2, 5), y, round((z1 + z2) / 2, 5)], BELT,
                       [0.0, round(angle, 3), 0.0]))
    return out


def plate(y, length, width, rgba, r=None):
    """Пластина звена со скруглёнными концами вдоль z от 0 до length."""
    r = r or width / 2
    return [box([width, 0.004, length], [0.0, y, length / 2], rgba),
            cyl(r, 0.004, [0.0, y, 0.0], rgba, Y), cyl(r, 0.004, [0.0, y, length], rgba, Y)]


ARM = {
    "башня": [
        cyl(0.05, 0.01, [0.0, 0.0, 0.005], DARK),
        cyl(0.042, 0.006, [0.0, 0.0, 0.013], PRINT),
        # Серва плеча стоит на поворотной площадке и тянет плечо ремнём 1:3.
        box([0.035, 0.025, 0.045], [-0.012, 0.0, 0.045], SERVO),
        cyl(0.009, 0.006, [-0.012, 0.042, 0.045], HORN, Y),
        *belt(0.042, (-0.012, 0.045, 0.009), (0.0, 0.10, 0.027)),
        *[box([0.03, 0.004, 0.095], [0.0, s * 0.032, 0.0635], PRINT) for s in (-1, 1)],
        *[cyl(0.015, 0.004, [0.0, s * 0.032, 0.10], PRINT, Y) for s in (-1, 1)],
    ],
    "плечо": [
        cyl(0.027, 0.005, [0.0, 0.042, 0.0], HORN, Y),
        *plate(0.022, 0.12, 0.024, PRINT), *plate(-0.022, 0.12, 0.024, PRINT),
        # Серва локтя у плеча тянет локоть ремнём 1:2.
        box([0.034, 0.025, 0.042], [0.0, 0.0, 0.045], SERVO),
        cyl(0.008, 0.005, [0.0, -0.032, 0.045], HORN, Y),
        *belt(-0.032, (0.0, 0.045, 0.008), (0.0, 0.12, 0.016)),
    ],
    "предплечье": [
        cyl(0.016, 0.005, [0.0, -0.032, 0.0], HORN, Y),
        cyl(0.013, 0.04, [0.0, 0.0, 0.0], LIGHT, Y),
        capsule(0.011, 0.07, [0.0, 0.0, 0.055], PRINT),
        box([0.032, 0.025, 0.04], [0.0, 0.0, 0.098], SERVO),
        cyl(0.011, 0.003, [0.0, 0.014, 0.12], HORN, Y),
    ],
    "запястье": [
        *[box([0.02, 0.003, 0.03], [0.0, s * 0.0175, 0.005], LIGHT) for s in (-1, 1)],
        *[cyl(0.01, 0.003, [0.0, s * 0.0175, 0.0], LIGHT, Y) for s in (-1, 1)],
        box([0.024, 0.032, 0.028], [0.0, 0.0, 0.024], SERVO),
    ],
    "кисть": [
        cyl(0.012, 0.003, [0.0, 0.0, 0.0015], HORN),
        cyl(0.018, 0.022, [0.0, 0.0, 0.014], PRINT),
        cyl(0.02, 0.004, [0.0, 0.0, 0.028], STEEL),
        *[cyl(0.0022, 0.0012, [sx * 0.0088, sy * 0.0088, 0.0302], DARK) for sx in (-1, 1) for sy in (-1, 1)],
    ],
    "ладонь": [
        box([0.05, 0.03, 0.02], [0.0, 0.0, 0.01], PRINT),
        box([0.024, 0.03, 0.006], [0.008, 0.0, 0.023], SERVO),
        box([0.04, 0.006, 0.003], [0.004, 0.0, 0.0215], HORN),
        box([0.006, 0.02, 0.035], [-0.0185, 0.0, 0.0375], LIGHT),
        box([0.001, 0.018, 0.03], [-0.015, 0.0, 0.04], DARK),
    ],
    "губа": [
        box([0.006, 0.02, 0.035], [0.0, 0.0, 0.0], LIGHT),
        box([0.001, 0.018, 0.03], [-0.0035, 0.0, 0.0025], DARK),
    ],
}

TOOLS = {
    "основание@tool-probe.toml": [
        cyl(0.015, 0.01, [0.0, 0.0, 0.006], HORN),
        cyl(0.0152, 0.002, [0.0, 0.0, 0.006], DARK),
        cyl(0.006, 0.008, [0.0, 0.0, 0.015], DARK),
        cyl(0.0025, 0.026, [0.0, 0.0, 0.032], STEEL),
        sphere(0.003, [0.0, 0.0, 0.045], RUBY),
    ],
    "основание@tool-marker.toml": [
        cyl(0.015, 0.01, [0.0, 0.0, 0.006], HORN),
        cyl(0.0152, 0.002, [0.0, 0.0, 0.006], DARK),
        cyl(0.005, 0.03, [0.0, 0.0, 0.026], LIGHT),
        cyl(0.0052, 0.006, [0.0, 0.0, 0.016], BLUE),
        cyl(0.0035, 0.01, [0.0, 0.0, 0.046], LIGHT),
        cyl(0.0015, 0.004, [0.0, 0.0, 0.053], BLUE),
        sphere(0.0015, [0.0, 0.0, 0.055], BLUE),
    ],
}


def block(item, header="[[links.visual]]"):
    kind, size, pos, rpy, rgba = item
    pose = f"position_m = {[round(v, 6) for v in pos]}" + (f", rpy_deg = {rpy}" if rpy else "")
    return [header, f'shape = {{ kind = "{kind}", {size} }}', f"pose = {{ {pose} }}", f"rgba = {rgba}", ""]


def table_visuals():
    """Вид столешницы под дерево. Физическая столешница — первая геометрия окружения."""
    return [MARK] + block(box([1.2, 0.8, 0.04], [0.15, 0.0, -0.02], TABLE), "[[environment.visual]]")


def holder_visuals(x, y, top):
    """Вид держателя: подставка и две ветви вилки поверх физических брусков."""
    out = [MARK]
    out += block(box([0.05, 0.05, 0.004], [x, y, 0.002], STAND), "[[environment.visual]]")
    for side in (-1, 1):
        out += block(box([0.03, 0.008, top], [x, round(y + side * 0.012, 6), top / 2], STAND), "[[environment.visual]]")
    return out


def rewrite_links(path, visuals):
    """Убирает старые блоки вида у звеньев и вставляет новые перед первой геометрией столкновений."""
    lines = open(path).read().split("\n")
    out, i, link = [], 0, None
    pending = None
    while i < len(lines):
        line = lines[i]
        if line == "[[links]]":
            link = None
        if link is None and line.startswith('name = "') and out and "[[links]]" in out[-3:]:
            link = line.split('"')[1]
        if line == MARK:
            i += 1
            continue
        if line == "[[links.visual]]":
            i += 1
            while i < len(lines) and lines[i] and not lines[i].startswith("["):
                i += 1
            if i < len(lines) and lines[i] == "":
                i += 1
            continue
        if line == "[[links.collision]]" and link in visuals and pending != link:
            pending = link
            # Комментарий над геометрией столкновений остаётся при ней.
            notes = []
            while out and out[-1].startswith("#") and out[-1] != MARK:
                notes.insert(0, out.pop())
            for item in visuals[link]:
                out += [MARK] + block(item)
            out += notes
        out.append(line)
        i += 1
    open(path, "w").write("\n".join(out))


def rewrite_world(path):
    """Добавляет вид стола в мир руки, если его ещё нет."""
    lines = open(path).read().split("\n")
    out, skip = [], False
    for line in lines:
        if line == MARK:
            skip = True
            continue
        if skip:
            if line == "":
                skip = False
            continue
        out.append(line)
    text = "\n".join(out)
    anchor = "\n[[devices]]"
    i = text.index(anchor)
    text = text[:i] + "\n" + "\n".join(table_visuals()).rstrip("\n") + "\n" + text[i:]
    open(path, "w").write(text)


def main():
    for name in ("device.toml", "device-gripper.toml"):
        rewrite_links(os.path.join(ROOT, name), ARM)
    for key, items in TOOLS.items():
        link, file = key.split("@")
        rewrite_links(os.path.join(ROOT, file), {link: items})
    for name in ("world.toml", "world-gripper.toml"):
        rewrite_world(os.path.join(ROOT, name))
    print("вид записан: device.toml, device-gripper.toml, tool-probe.toml, tool-marker.toml, world.toml, world-gripper.toml")


if __name__ == "__main__":
    main()
