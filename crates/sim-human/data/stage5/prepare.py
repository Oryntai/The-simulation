"""Воспроизводимая подготовка данных из локального MS-Human-700 (Apache-2.0).
Запуск из любого каталога: python3 crates/sim-human/data/stage5/prepare.py.
"""
import copy
import struct
from collections import defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET

OUT = Path(__file__).resolve().parent
SOURCE = OUT.parents[3] / "assets/third_party/ms_human_700"


def read(path):
    return ET.parse(SOURCE / path).getroot()


def values(element, key, default):
    return [float(value) for value in element.get(key, default).split()]


def encode(numbers):
    return " ".join(format(number, ".15g") for number in numbers)


def reflected(element):
    result = copy.deepcopy(element)
    for node in result.iter():
        # Q' = S Q S, S = diag(1,1,-1). Для оси вращения a' = det(S) S a.
        assert "euler" not in node.attrib and "fromto" not in node.attrib
        for key, signs in [("pos", [1, 1, -1]), ("quat", [1, -1, -1, 1]),
                           ("axis", [1, 1, -1] if node.get("type") == "slide" else [-1, -1, 1])]:
            if key in node.attrib:
                node.set(key, encode(v * s for v, s in zip(values(node, key, ""), signs)))
        if "name" in node.attrib:
            node.set("name", "mirror_" + node.get("name"))
        if "mesh" in node.attrib:
            node.set("mesh", "mirror_" + node.get("mesh"))
    return result


def rotate(quaternion, vector):
    w, x, y, z = quaternion
    if [x, y, z] == [0, 0, 0]:
        return vector
    cross = lambda a, b: [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    u = [x, y, z]
    uv = cross(u, vector)
    uuv = cross(u, uv)
    norm_sq = sum(v*v for v in quaternion)
    return [v + 2*(w*c+d)/norm_sq for v, c, d in zip(vector, uv, uuv)]


def transform(point, frame):
    return [v + o for v, o in zip(rotate(values(frame, "quat", "1 0 0 0"), point),
                               values(frame, "pos", "0 0 0"))]


def mesh_points(geom, mesh):
    file = SOURCE / mesh.get("file")
    scale = values(mesh, "scale", "1 1 1")
    assert "refquat" not in mesh.attrib and "refpos" not in mesh.attrib
    if file.suffix == ".obj":
        vertices = (map(float, line.split()[1:4]) for line in file.read_text().splitlines()
                    if line.startswith("v "))
    else:
        stl = file.read_bytes()
        n, = struct.unpack_from("<I", stl, 80)
        assert len(stl) == 84 + n * 50, file
        vertices = (struct.unpack_from("<3f", stl, 84 + face * 50 + 12 * (1 + vertex))
                    for face in range(n) for vertex in range(3))
    for point in vertices:
        yield transform([v*s for v, s in zip(point, scale)], geom)


def main():
    primary = read("MS-Human-700.xml")
    detailed = read("MS-Human-700-Manipulation.xml")
    root = ET.Element("mujoco", model="participant")
    ET.SubElement(root, "compiler", angle="radian", usethread="false", boundmass=primary.find("compiler").get("boundmass"))
    defaults = copy.deepcopy(primary.find("default"))
    defaults.find("geom").attrib.pop("margin")
    defaults.remove(defaults.find("default[@class='muscle']"))
    defaults.append(copy.deepcopy(detailed.find("default/default[@class='finger']")))
    root.append(defaults)
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    pelvis = copy.deepcopy(primary.find("worldbody/body"))
    world.append(pelvis)
    for el in list(pelvis):
        if el.tag in ("joint", "camera", "light", "include"):
            pelvis.remove(el)
    for side in ("r", "l"):
        pelvis.extend(read(f"Body_Primary/Body_Leg_Foot_{side}.xml"))
    pelvis.extend(el for el in read("Body_Primary/Body_Torso_Simple.xml") if el.tag != "include")
    arm_parent = next(b for b in pelvis.iter("body") if b.find("include") is not None)
    for el in list(arm_parent.findall("include")):
        arm_parent.remove(el)
    arm = read("Body_Manipulation/Body_Arm_Hand_r.xml").find("body")
    mirror = reflected(arm)
    arm_names = {tag: {n.get("name") for n in arm.iter(tag)} for tag in ("body", "geom", "joint", "site")}
    torso_sites = {s.get("name"): (b, s) for b in pelvis.iter("body") for s in b.findall("site")}
    arm_parent.extend([arm, mirror])

    meshes = {}
    for file in ("Lowerbody", "Upperbody", "Torso"):
        for mesh in read(f"Asset/Asset_{file}.xml").iter("mesh"):
            meshes[mesh.get("name")] = mesh
    for name in sorted({g.get("mesh") for g in mirror.iter("geom") if g.get("mesh")}):
        obj = copy.deepcopy(meshes[name.removeprefix("mirror_")])
        obj.set("name", name)
        scale = values(obj, "scale", "1 1 1")
        scale[2] *= -1
        obj.set("scale", encode(scale))
        meshes[name] = obj

    contact = ET.SubElement(root, "contact")
    exclusions = [*read("Contact/Contact_Lowerbody.xml").findall("contact/exclude"),
                  *read("Contact/Contact_Upperbody.xml").findall("contact/exclude")]
    contact.extend(exclusions)
    for exclusion in exclusions:
        if exclusion.get("body1") in arm_names["body"]:
            contact.append(ET.Element("exclude", {k: "mirror_" + v for k, v in exclusion.attrib.items()}))

    equality = ET.SubElement(root, "equality")
    equality.extend(read("Equality/Equality_Lowerbody.xml").find("equality"))
    for joint in read("Equality/Equality_Upperbody_r.xml").find("equality"):
        equality.append(joint)
        equality.append(ET.Element("joint", {
            k: ("mirror_" + v if k in ("name", "joint1", "joint2") else v)
            for k, v in joint.attrib.items()
        }))

    tendon = ET.SubElement(root, "tendon")
    for piece in ("Leg_r", "Leg_l", "Torso", "Arm_Hand_r"):
        tendon.extend(read(f"Tendon/Tendon_{piece}.xml").find("tendon"))
    for path in read("Tendon/Tendon_Arm_Hand_r.xml").find("tendon"):
        mirrored = reflected(path)
        for point in mirrored:
            for key in point.attrib:
                name = point.get(key)
                if key == "geom":
                    assert name in arm_names["geom"]
                elif name not in arm_names["site"]:
                    holder, site = torso_sites[name]
                    # Точки начала этих сухожилий находятся на срединной цепи.
                    assert values(holder, "pos", "0 0 0")[2] == 0
                    if holder.find(f"site[@name='mirror_{name}']") is None:
                        holder.append(reflected(site))
                point.set(key, "mirror_" + name)
        tendon.append(mirrored)

    # Сохраняем собственные левые точки исходной модели для путей туловища.
    left = read("Body_Primary/Body_Arm_l.xml")
    mirror_bodies = {b.get("name"): b for b in mirror.iter("body")}
    refs = {point.get(key) for path in tendon for point in path for key in ("site", "sidesite") if point.get(key)}
    present = {s.get("name") for s in pelvis.iter("site")}
    for holder in left.iter("body"):
        for site in holder.findall("site"):
            if site.get("name") in refs - present:
                target = mirror_bodies["mirror_" + holder.get("name").removesuffix("_l") + "_r"]
                target.append(copy.deepcopy(site))
                present.add(site.get("name"))

    # Одно объявление сетки на геометрию позволяет независимо менять её масштаб.
    mesh_files = set()
    for geom in pelvis.iter("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        if geom.get("class") == "skin":
            geom.set("rgba", "0 0 0 0")
        if geom.get("mesh"):
            mesh = copy.deepcopy(meshes[geom.get("mesh")])
            mesh.set("name", f"mesh{len(asset)}")
            mesh_files.add(mesh.get("file"))
            mesh.set("file", "@assets@/" + mesh.get("file"))
            geom.set("mesh", mesh.get("name"))
            asset.append(mesh)

    by_name = {b.get("name"): b for b in pelvis.iter("body")}
    parents = {child: parent for parent in pelvis.iter() for child in parent.findall("body")}
    metadata = []
    ports = []
    region_by_proxy = {}

    def add(body, label, side, part, geo, origin):
        # Маска принимает внешнюю активную геометрию, но не создаёт самоконтакт.
        name = f"proxy{len(metadata)}"
        region_by_proxy[name] = f"r{len(metadata):04d}"
        geo.attrib.update(name=name, mass="0", contype="0", conaffinity="1",
                          rgba="0.72 0.53 0.4 0.25", group="1")
        geo.attrib.pop("class", None)
        body.append(geo)
        metadata.append((name, label, side, part, origin, body.get("name")))
        for axis, direction in enumerate(("x", "y", "z")):
            size = values(geo, "size", "")
            if geo.get("type") == "capsule":
                size = [size[0], size[0], size[0] + size[1]]
            for sign, suffix in ((-1, "-"), (1, "+")):
                point = [0.0] * 3
                point[axis] = sign * size[axis]
                site = ET.SubElement(body, "site", name=f"mount{len(ports)}",
                                     pos=encode(transform(point, geo)), size="0.001", group="4")
                ports.append((site, name, direction + suffix))

    def skin(body, label, side, part):
        for geom in body.findall("geom[@class='skin']"):
            geo = copy.deepcopy(geom)
            geo.set("type", "capsule")
            add(body, label, side, part, geo, geom.get("name"))

    def bounds(body, selection, label, side, part):
        points = []
        for holder, geo in selection:
            mesh = asset.find(f"mesh[@name='{geo.get('mesh')}']")
            mesh = copy.deepcopy(mesh)
            mesh.set("file", mesh.get("file").removeprefix("@assets@/"))
            for point in mesh_points(geo, mesh):
                frame = holder
                while frame is not body:
                    point = transform(point, frame)
                    frame = parents[frame]
                points.append(point)
        lo = [min(v[i] for v in points) for i in range(3)]
        hi = [max(v[i] for v in points) for i in range(3)]
        add(body, label, side, part, ET.Element("geom", type="ellipsoid",
            pos=encode((a+b)/2 for a, b in zip(lo, hi)), size=encode((b-a)/2 for a, b in zip(lo, hi))),
            "Габариты сеток: " + ", ".join(geo.get("name") for _, geo in selection))

    def own(body, names=None):
        return [(body, g) for g in body.findall("geom[@mesh]") if names is None or g.get("name") in names]

    bounds(pelvis, own(pelvis), "Таз", "mid", "pelvis")
    head = by_name["head_neck"]
    bounds(head, own(head, {"jaw", "skull"}), "Голова", "mid", "head")
    bounds(head, own(head)[:-2], "Шея", "mid", "neck")
    for levels, first, label in [(range(1, 5), 4, "Верх грудной клетки"),
                                  (range(5, 9), 8, "Середина грудной клетки"),
                                  (range(9, 13), 12, "Низ грудной клетки")]:
        holders = [by_name[f"thoracic{i}"] for i in levels]
        bounds(by_name[f"thoracic{first}"], [entry for holder in holders for entry in
               (own(holder) + [entry for b in holder.findall("body") if b.get("name").startswith(("rib", "sternum")) for entry in own(b)])],
               label, "mid", "trunk")
    lumbar = by_name["lumbar5"]
    bounds(lumbar, [entry for i in range(1, 6) for entry in own(by_name[f"lumbar{i}"])],
           "Поясница", "mid", "lumbar")

    for side in ("r", "l"):
        prefix = "" if side == "r" else "mirror_"
        for body, label, part in [
            ("clavicle_r", "Ключица", "shoulder"), ("humerus_r", "Плечо", "upper"),
            ("ulna_r", "Предплечье", "lower"), ("radius_r", "Лучевая сторона предплечья", "lower"),
            ("capitate", "Основание ладони", "palm")]:
            b = by_name[prefix + body]
            bounds(b, own(b), label, side, part)
        for body, label, part in [("femur", "Бедро", "thigh"), ("tibia", "Голень", "shin"),
                                  ("calcn", "Стопа", "foot"), ("toes", "Пальцы стопы", "foot")]:
            skin(by_name[f"{body}_{side}"], label, side, part)
        for index, anatomic in enumerate(("first", "second", "third", "fourth", "fifth"), 1):
            skin(by_name[prefix + anatomic + "mc"], f"Ладонь, пястная кость {index}", side, "palm")
            phalanges = [("proximal_thumb", "Проксимальная"), ("distal_thumb", "Дистальная")] if index == 1 else [
                (f"{index}proxph", "Проксимальная"), (f"{index}midph", "Средняя"), (f"{index}distph", "Дистальная")]
            for phalanx, label in phalanges:
                skin(by_name[prefix + phalanx], f"{label} фаланга, палец {index}", side, f"digit{index}")

    # Стабильные непрозрачные локальные идентификаторы по порядку источника.
    scopes = {tag: list(pelvis.iter(tag)) for tag in ("body", "joint", "geom", "site")}
    scopes.update(spatial=list(tendon), equation=list(equality), mesh=list(asset))
    letters = dict(body="b", joint="j", geom="g", site="s", spatial="t", equation="e", mesh="m")
    names = defaultdict(dict)
    rows = []
    for scope, elements in scopes.items():
        for i, element in enumerate(elements):
            name = element.get("name")
            assert name is not None and name not in names[scope], (scope, name)
            code = f"{letters[scope]}{i:04d}"
            rows.append((scope, code, name))
            names[scope][name] = code
            element.set("name", code)
    for el in root.iter():
        if el.tag == "geom" and "mesh" in el.attrib:
            el.set("mesh", names["mesh"][el.get("mesh")])
        for attr in ("body1", "body2", "joint1", "joint2", "geom", "site", "sidesite"):
            if attr in el.attrib:
                typ = {"body1":"body", "body2":"body", "joint1":"joint", "joint2":"joint", "sidesite":"site"}.get(attr, attr)
                el.set(attr, names[typ][el.get(attr)])

    ET.indent(root)
    (OUT / "participant.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")
    def write(name, records):
        (OUT / name).write_text("".join("\t".join(fields) + "\n" for fields in records))
    write("names.tsv", rows)
    write("surfaces.tsv", [(f"r{i:04d}", names["geom"][g], names["body"][b], side, part, label, source)
                          for i, (g, label, side, part, source, b) in enumerate(metadata)])
    write("ports.tsv", [(site.get("name"), region_by_proxy[name], direction)
                       for site, name, direction in ports])
    write("resources.tsv", [(path,) for path in sorted(mesh_files)])


if __name__ == "__main__":
    main()
