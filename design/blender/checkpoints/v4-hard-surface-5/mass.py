"""Mass and balance of TinyREX against Microduck, at rest.

Sent after tinyrex_v2.py's and rig.py's globals (the rig empties give Microduck's body frames).
Microduck's masses and centres come from its MJCF (its authors' CAD); TinyREX's are Microduck's
minus the shells it replaces, plus TinyREX's own printed parts (mesh volume x density; parts
printed solid in the model count at an effective walls-plus-infill density), the extra servos
and hardware. Millimetres and grams; prints one JSON line, MASS {...}."""
import json
import xml.etree.ElementTree as ET

import bpy
from mathutils import Vector

PLA, TPU, STEEL, NDFEB, ACRYLIC = 1.24, 1.21, 7.85, 7.5, 1.19  # g/cm3
INFILL = 0.45  # a solid-modelled printed part: 3 walls + 20 % infill, as a fraction of solid
XL330_G = 18.0
SCREW_G = 0.35  # M2 x 6 button head


def volume_centroid(o, deps):
    ev = o.evaluated_get(deps)
    me = ev.to_mesh()
    mw = o.matrix_world
    V = [mw @ v.co for v in me.vertices]
    vol, c = 0.0, Vector()
    for p in me.polygons:
        a = V[p.vertices[0]]
        for i in range(1, len(p.vertices) - 1):
            b, d = V[p.vertices[i]], V[p.vertices[i + 1]]
            t = a.dot(b.cross(d)) / 6.0
            vol += t
            c += t * (a + b + d) / 4.0
    ev.to_mesh_clear()
    return abs(vol) / 1000.0, (c / vol if vol else Vector())  # cm3, mm


def microduck_masses():
    """(mass g, world centre mm) per MJCF body, at rest, from the rig's empties."""
    root = ET.parse(MJCF).getroot()
    out = []
    for b in root.iter("body"):
        i = b.find("inertial")
        if i is None:
            continue
        pos = Vector([float(v) for v in i.get("pos").split()]) * 1000.0
        out.append((float(i.get("mass")) * 1000.0, E(b.get("name")).matrix_world @ pos, b.get("name"), b.get("name")))
    return out


# Microduck's shells, eyes, face and feet - replaced by TinyREX's own parts. (Its reference scene
# places some meshes twice, a visual and a collision geom: one of each is counted.)
REPLACED = {"left_shell": PLA, "right_shell": PLA, "top_head_shell": PLA, "bottom_head_shell": PLA, "face_part": PLA,
            "jaw": PLA, "jaw_soft": PLA, "soft_mouth_top": TPU, "noenoeil": PLA, "foot_left": PLA, "foot_right": PLA,
            "sole_left": TPU, "sole_right": TPU}


MD_BODY = {"left_shell": "trunk_base", "right_shell": "trunk_base", "foot_left": "ankle_left", "sole_left": "ankle_left",
           "foot_right": "ankle_right", "sole_right": "ankle_right"}  # the rest are the head's
XL330_STALL = 0.52  # N m, XL330-M288 at 5 V


def stance_torques(items):
    """Standing on the left leg alone, at the rig's current pose: the static torque each joint of
    that leg holds - the moment, about its axis, of the weight of everything on the body's side of
    it (all of the robot but the leg below the joint)."""
    out = {}
    for body in ("yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left"):
        e = E(body)
        below = subtree(body)
        p = e.matrix_world.translation
        axis = (e.matrix_world.to_3x3() @ Vector(e["axis"])).normalized()
        f = Vector((0, 0, -9.81e-3))  # N per gram
        t = sum(((c - p) * 1e-3).cross(f * m).dot(axis) for m, c, _, b in items if b not in below)
        out[e["joint"]] = round(abs(t), 3)
    return out


def density(name, o):
    n = name
    if n.startswith(("JawShaft", "CheekPin", "ArmPin")):
        return STEEL, 1.0
    if n.startswith("TailMagnet"):
        return NDFEB, 1.0
    if n.startswith("VisorGlass"):
        return ACRYLIC, 1.0
    if n.startswith(("TailFlex", "Sole", "ToeSole")):
        return TPU, 1.0
    hollow = any(m.type == "SOLIDIFY" for m in o.modifiers)
    return PLA, (1.0 if hollow else INFILL)


def run():
    rest_all()
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    md = microduck_masses()
    m_md = sum(m for m, _, _, _ in md)
    c_md = sum((c * m for m, c, _, _ in md), Vector()) / m_md
    # minus the replaced shells (and the camera's old place: it moved to the visor)
    ref = bpy.data.scenes["Microduck (reference)"]
    seen = set()
    minus = []
    for o in ref.objects:
        if o.type != "MESH":
            continue
        n = o.name.split(".")[0][3:]
        if n in REPLACED or n in ("lens", "m12_lens_holder"):
            v, c = volume_centroid(o, deps)
            key = (n, round(c.x), round(c.y), round(c.z))
            if key in seen:
                continue
            seen.add(key)
            minus.append((v * REPLACED.get(n, PLA), c, n, MD_BODY.get(n, HEAD)))
    # plus TinyREX's parts
    plus = []
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o:
            continue
        n = o.name.split(".")[0]
        if n in ("XL330_ArmL", "XL330_ArmR", "XL330_TailYaw"):
            sub = o.name.split(".")
            if len(sub) == 1 or sub[1].isdigit():  # the case (its horn, hub, screws are included)
                plus.append((XL330_G, volume_centroid(o, deps)[1], n, o["body"]))
        elif n in ("CameraHolder", "CameraLens"):
            v, c = volume_centroid(o, deps)
            plus.append((v * PLA, c, n, o["body"]))
        elif o.get("skin") and not n.startswith("XL330"):
            v, c = volume_centroid(o, deps)
            if "Screw" in n:
                plus.append((SCREW_G, c, n, o["body"]))
            elif n.startswith(("ToFBoard", "HeadIMU")):
                plus.append((1.5, c, n, o["body"]))
            else:
                rho, k = density(n, o)
                plus.append((v * rho * k, c, n, o["body"]))
    m_tr = m_md - sum(m for m, _, _, _ in minus) + sum(m for m, _, _, _ in plus)
    c_tr = (c_md * m_md - sum((c * m for m, c, _, _ in minus), Vector()) + sum((c * m for m, c, _, _ in plus), Vector())) / m_tr
    tinyrex = md + [(-m, c, n, b) for m, c, n, b in minus] + plus

    # support polygon at rest: both feet's soles and toe soles (TinyREX), Microduck's soles
    def footprint(names, sc):
        pts = []
        for o in sc.objects:
            if o.type == "MESH" and o.name.split(".")[0] in names:
                pts += [o.matrix_world @ Vector(b) for b in o.bound_box]
        return min(p.x for p in pts), max(p.x for p in pts), min(p.y for p in pts), max(p.y for p in pts)
    fp_tr = footprint({"Sole_L", "Sole_R", "ToeSole_L0", "ToeSole_L2", "ToeSole_R0", "ToeSole_R2"}, scene)
    fp_md = footprint({"md_sole_left", "md_sole_right"}, ref)

    def margins(c, fp):
        return {"front": round(fp[1] - c.x, 1), "back": round(c.x - fp[0], 1), "side": round(min(fp[3] - c.y, c.y - fp[2]), 1)}
    top = sorted(plus, key=lambda t: -t[0])[:14]
    return {
        "microduck": {"mass_g": round(m_md, 1), "com_mm": [round(v, 1) for v in c_md], "support_x": [round(fp_md[0], 1), round(fp_md[1], 1)], "margins_mm": margins(c_md, fp_md)},
        "tinyrex": {"mass_g": round(m_tr, 1), "com_mm": [round(v, 1) for v in c_tr], "support_x": [round(fp_tr[0], 1), round(fp_tr[1], 1)], "margins_mm": margins(c_tr, fp_tr)},
        "removed_g": round(sum(m for m, _, _, _ in minus), 1), "added_g": round(sum(m for m, _, _, _ in plus), 1),
        "removed": sorted(((n, round(m, 1)) for m, _, n, _ in minus), key=lambda t: -t[1]),
        "heaviest_added": [(n, round(m, 1), [round(v) for v in c]) for m, c, n, _ in top],
        "stance_torque_Nm": {"microduck": stance_torques(md), "tinyrex": stance_torques(tinyrex)},
    }


print("MASS", json.dumps(run()))
