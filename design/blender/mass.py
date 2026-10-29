"""Mass and balance of TinyREX against Microduck: at the zero pose and standing (HOME).

Sent after tinyrex_v2.py's, rig.py's and v4_parts.py's globals (the rig empties give the body
frames). Microduck's masses and centres come from its MJCF (its authors' CAD). TinyREX's are
Microduck's minus the shells it replaces, plus TinyREX's own printed parts (mesh volume x
density; parts printed solid in the model count at an effective walls-plus-infill density), the
extra servos and hardware. Every mass is held in its body's frame, so any pose is cheap.

Prints MASS {...} (zero pose), HOME {...} (Microduck's standing pose) and TREX_HOME {...} (the
standing pose that puts TinyREX's centre of mass over its ankle axis, as Microduck's HOME does
for Microduck). Millimetres, grams, newton metres."""
import json
import math
import xml.etree.ElementTree as ET

import bpy
from mathutils import Vector

PETG, TPU, STEEL, CARBON, POLYCARB = 1.27, 1.21, 7.85, 1.55, 1.20  # g/cm3
INFILL = 0.45  # a solid-modelled printed part: walls + 20 % gyroid, as a fraction of solid (the shells carry the 2.4 mm walls)
XL330_G = 18.0
SCREW_G = 0.35  # M2 x 6 button head
XL330_STALL = 0.52  # N m, XL330-M288 at 5 V

# Microduck's shells, eyes, face and feet - replaced by TinyREX's own parts. (Its reference scene
# places some meshes twice, a visual and a collision geom: one of each is counted.)
REPLACED = {"left_shell": PETG, "right_shell": PETG, "top_head_shell": PETG, "bottom_head_shell": PETG, "face_part": PETG,
            "jaw": PETG, "jaw_soft": PETG, "soft_mouth_top": TPU, "noenoeil": PETG, "foot_left": PETG, "foot_right": PETG,
            "sole_left": TPU, "sole_right": TPU, "lens": PETG, "m12_lens_holder": PETG}
MD_BODY = {"left_shell": "trunk_base", "right_shell": "trunk_base", "foot_left": "ankle_left", "sole_left": "ankle_left",
           "foot_right": "ankle_right", "sole_right": "ankle_right"}  # the rest are the head's

# Microduck's HOME standing pose (mjlab_microduck/robot/microduck_constants.py, HOME_FRAME): the
# stance its walking and standing policies start from, its centre of mass over the ankle axis.
HOME = {"left_hip_roll": -0.0873, "right_hip_roll": 0.0873, "left_hip_pitch": -0.4579, "right_hip_pitch": 0.4579,
        "left_knee": -0.0049, "right_knee": 0.0049, "left_ankle": 0.4530, "right_ankle": -0.4530,
        "neck_pitch": 0.3491, "head_pitch": 0.3491}
NECK_HOME = 0.29  # rad: Microduck's 20 deg neck pitch is past TinyREX's 18 deg collision-free limit
SOLES = {"Sole_L", "Sole_R", "ToeSole_L0", "ToeSole_L2", "ToeSole_R0", "ToeSole_R2"}


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


def density(name, o):
    if name.startswith(("JawShaft", "CheekPin", "ArmPin", "ArmSpring", "TailSpring")):
        return STEEL, 1.0
    if name.startswith("TailSpine"):
        return CARBON, 1.0
    if name.startswith(("VisorGlass", "Eye_")):
        return POLYCARB, 1.0
    if name.startswith(("Sole", "ToeSole", "TailTip", "ArmClutch", "TailClutch", "JawClutch")):
        return TPU, 1.0
    hollow = any(m.type == "SOLIDIFY" for m in o.modifiers)
    return PETG, (1.0 if hollow else INFILL)


def pose(angles):
    rest_all()
    for body, (_, local, joint, _) in BODIES.items():
        if joint and local is not None and joint[0] in angles:
            set_joint(body, angles[joint[0]])
    bpy.context.view_layer.update()


def inventory():
    """Every mass as (grams, centre in its body's frame, name, body), at rest: Microduck's bodies,
    minus the shells TinyREX replaces, plus TinyREX's parts."""
    rest_all()
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    inv = {b: E(b).matrix_world.inverted() for b in BODIES if BODIES[b][1] is not None}
    md = []
    for b in ET.parse(MJCF).getroot().iter("body"):
        i = b.find("inertial")
        if i is not None:
            pos = Vector([float(v) for v in i.get("pos").split()]) * 1000.0
            md.append((float(i.get("mass")) * 1000.0, pos, b.get("name"), b.get("name")))
    minus, seen = [], set()
    for o in bpy.data.scenes["Microduck (reference)"].objects:
        n = o.name.split(".")[0][3:]
        if o.type != "MESH" or n not in REPLACED:
            continue
        v, c = volume_centroid(o, deps)
        key = (n, round(c.x), round(c.y), round(c.z))
        if key in seen:
            continue
        seen.add(key)
        b = MD_BODY.get(n, HEAD)
        minus.append((-v * REPLACED[n], inv[b] @ c, n, b))
    plus = []
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o:
            continue
        n, b = o.name.split(".")[0], o["body"]
        if n in ("XL330_ArmL", "XL330_ArmR", "XL330_TailYaw"):
            sub = o.name.split(".")
            if len(sub) == 1 or sub[1].isdigit():  # the case (its horn, hub, screws are included)
                plus.append((XL330_G, inv[b] @ volume_centroid(o, deps)[1], n, b))
        elif n in ("CameraHolder", "CameraLens"):
            v, c = volume_centroid(o, deps)
            plus.append((v * PETG, inv[b] @ c, n, b))
        elif o.get("skin") and not n.startswith("XL330"):
            v, c = volume_centroid(o, deps)
            if "Screw" in n:
                m = SCREW_G
            elif n.startswith(("ToFBoard", "HeadIMU")):
                m = 1.5
            else:
                rho, k = density(n, o)
                m = v * rho * k
            plus.append((m, inv[b] @ c, n, b))
    return md, minus, plus


def world(items):
    return [(m, E(b).matrix_world @ c, n, b) for m, c, n, b in items]


def com(items):
    w = world(items)
    m = sum(t[0] for t in w)
    return m, sum((c * mm for mm, c, _, _ in w), Vector()) / m


def stance_torques(items):
    """Standing on the left leg alone, at the current pose: the static torque each joint of that
    leg holds - the moment, about its axis, of the weight of everything on the body's side of it."""
    w = world(items)
    out = {}
    for body in ("yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left"):
        e = E(body)
        below = subtree(body)
        p = e.matrix_world.translation
        axis = (e.matrix_world.to_3x3() @ Vector(e["axis"])).normalized()
        f = Vector((0, 0, -9.81e-3))  # N per gram
        t = sum(((c - p) * 1e-3).cross(f * m).dot(axis) for m, c, _, b in w if b not in below)
        out[e["joint"]] = round(abs(t), 3)
    return out


def footprint():
    pts = [o.matrix_world @ Vector(b) for o in scene.objects if o.type == "MESH" and o.name.split(".")[0] in SOLES for b in o.bound_box]
    return min(p.x for p in pts), max(p.x for p in pts), min(p.y for p in pts), max(p.y for p in pts)


def report(items_md, items_tr, angles):
    pose(angles)
    fp = footprint()
    ankle = (E("ankle_left").matrix_world.translation + E("ankle_right").matrix_world.translation) / 2
    out = {}
    for k, items in (("microduck", items_md), ("tinyrex", items_tr)):
        m, c = com(items)
        out[k] = {"mass_g": round(m, 1), "com_mm": [round(v, 1) for v in c], "com_ahead_of_ankle_mm": round(c.x - ankle.x, 1),
                  "torque_Nm": stance_torques(items)}
    # (TinyREX's soles; Microduck's sit 1.2 mm further back at the heel and 9.8 mm short at the toe)
    c = com(items_tr)[1]
    out["tinyrex"]["margins_mm"] = {"front": round(fp[1] - c.x, 1), "back": round(c.x - fp[0], 1), "side": round(min(fp[3] - c.y, c.y - fp[2]), 1)}
    out["clips"] = sorted({f"{a.split('.')[0]} x {b.split('.')[0]}" for a, b in clips()}) if angles else []
    rest_all()
    bpy.context.view_layer.update()
    return out


def trex_home(items_tr):
    """Microduck's HOME, with the legs' pitch chain leaned so that TinyREX's centre of mass sits
    over its ankle axis (feet flat: ankle = -hip pitch, knee as in HOME), the neck inside its
    collision-free range."""
    def ahead(a):
        angles = dict(HOME, left_hip_pitch=-a, right_hip_pitch=a, left_ankle=a, right_ankle=-a, neck_pitch=NECK_HOME)
        pose(angles)
        ankle = (E("ankle_left").matrix_world.translation + E("ankle_right").matrix_world.translation) / 2
        return com(items_tr)[1].x - ankle.x, angles
    lo, hi = 0.0, 0.9
    for _ in range(20):
        mid = (lo + hi) / 2
        d, _ = ahead(mid)
        if (d > 0) == (ahead(lo)[0] > 0):
            lo = mid
        else:
            hi = mid
    d, angles = ahead((lo + hi) / 2)
    rest_all()
    return {k: round(v, 4) for k, v in angles.items()}


md, minus, plus = inventory()
tr = md + minus + plus
zero = report(md, tr, {})
zero["removed_g"] = round(-sum(m for m, _, _, _ in minus), 1)
zero["added_g"] = round(sum(m for m, _, _, _ in plus), 1)
zero["heaviest_added"] = [(n, round(m, 1)) for m, _, n, _ in sorted(plus, key=lambda t: -t[0])[:12]]
print("MASS", json.dumps(zero))
print("HOME", json.dumps(report(md, tr, HOME)))
th = trex_home(tr)
out = report(md, tr, th)
out["angles"] = th
print("TREX_HOME", json.dumps(out))
