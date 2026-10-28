"""Rig and checks for a TinyREX scene built by tinyrex_v2.py: Microduck's body tree as empties,
every part parented to the body it moves with, a joint driver, and clip checks at rest and over
every joint's range.

Sent after tinyrex_v2.py's helpers (same globals: SCENE, MJCF, GROUND, scene, collection, ...),
with RIG_ACTION set:
    "rig"     build the empties and parent the parts
    "clips"   report parts of different, unconnected bodies that intersect at rest
    "sweep"   drive each joint through its range and report what collides
"""
import json
import math
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

RIG_ACTION = globals().get("RIG_ACTION", "rig")


def _tf(el):
    pos = Vector([float(v) for v in el.get("pos", "0 0 0").split()]) * 1000.0
    q = el.get("quat")
    return Matrix.Translation(pos) @ (Quaternion([float(v) for v in q.split()]) if q else Quaternion()).to_matrix().to_4x4()


def mjcf_bodies():
    """Body name -> (parent name, rest local matrix, joint (name, axis, (lo, hi)) or None, geoms
    as (mesh, index in the scene builder's traversal))."""
    root = ET.parse(MJCF).getroot()
    out, order = {}, [0]

    def walk(body, parent):
        j = body.find("joint")
        joint = None
        if j is not None:
            lo, hi = (float(v) for v in j.get("range", "-3.14 3.14").split())
            joint = (j.get("name"), Vector([float(v) for v in j.get("axis", "0 0 1").split()]), (lo, hi))
        geoms = []
        for g in body.findall("geom"):
            if g.get("mesh") and "collision" not in (g.get("class") or ""):  # (as tinyrex_v2.mjcf_geoms)
                geoms.append((g.get("mesh"), order[0]))
                order[0] += 1
        local = _tf(body)
        if parent is None:
            local = Matrix.Translation((0, 0, GROUND)) @ local
        out[body.get("name")] = (parent, local, joint, geoms)
        for c in body.findall("body"):
            walk(c, body.get("name"))

    for b in root.find("worldbody").findall("body"):
        walk(b, None)
    return out


BODIES = mjcf_bodies()
SIDE = {"L": ("upper_leg_left", "leg", "ankle_left"), "R": ("upper_leg_right", "leg_2", "ankle_right")}
HEAD, TRUNK = "jaw_soft", "trunk_base"


def skin_body(name):
    """Which body a TinyREX skin part moves with: the body of the Microduck part it replaces."""
    side = "L" if "_L" in name else "R" if "_R" in name else None
    if name.startswith(("Thigh", "ThighScrew")) and side:
        return SIDE[side][0]
    if name.startswith("Shin") and side:
        return SIDE[side][1]
    if name.startswith(("Foot", "Toe", "Sole")) and side:
        return SIDE[side][2]
    if name.startswith("NeckCollar1"):
        return "neck"
    if name.startswith(("Head", "Eye", "Crest", "Grille", "Camera", "Nose", "Blush", "Cheek", "Jaw", "Mouth", "Tongue", "Fang", "Brow", "Lid", "HeadScrew")):
        return HEAD
    return TRUNK


def part_bodies():
    """Object name -> body, for the skeleton and electronics, from the builder's naming."""
    m, n_servo, n_bearing = {}, 0, 0
    ELEC = {"pcb__raspberry_pi_zero_2_w": "Compute", "elec_rpi_robot_hat_pcb": "RobotHat", "np_f970": "Battery",
            "speaker": "Speaker", "m12_lens_holder": "CameraHolder", "lens": "CameraLens"}
    flat = sorted(((idx, mesh, body) for body, (_, _, _, geoms) in BODIES.items() for mesh, idx in geoms))
    for idx, mesh, body in flat:
        if mesh == "xl330":
            m[f"XL330_{n_servo:02d}"] = body
            n_servo += 1
        elif mesh.startswith("seeed_bearing"):
            m[f"Bearing_{n_bearing:02d}"] = body
            n_bearing += 1
        elif mesh in ELEC:
            m[ELEC[mesh]] = body
        m[f"Bracket_{mesh}_{idx}"] = body
    return m


def build_rig():
    coll = collection("rig")
    empties = {}
    for name, (parent, local, joint, _) in BODIES.items():
        e = bpy.data.objects.new(f"B:{name}", None)
        e.empty_display_type = "ARROWS"
        e.empty_display_size = 12
        coll.objects.link(e)
        empties[name] = e
    for name, (parent, local, joint, _) in BODIES.items():
        e = empties[name]
        if parent:
            e.parent = empties[parent]
            e.matrix_parent_inverse = Matrix.Identity(4)
        e.matrix_basis = local
        e["rest"] = [v for row in local for v in row]
        if joint:
            e["joint"], e["axis"], e["lo"], e["hi"] = joint[0], list(joint[1]), joint[2][0], joint[2][1]
    bpy.context.view_layer.update()
    pm = part_bodies()
    n = 0
    for c in (f"{SCENE}: skeleton", f"{SCENE}: electronics", f"{SCENE}: skins"):
        if c not in bpy.data.collections:
            continue
        for o in bpy.data.collections[c].objects:
            base = o.name.split(".")[0]
            body = pm.get(base) or (skin_body(base) if c.endswith("skins") else None)
            if body is None:
                continue
            mw = o.matrix_world.copy()
            o.parent = empties[body]
            o.matrix_parent_inverse = empties[body].matrix_world.inverted()
            o.matrix_world = mw
            o["body"] = body
            n += 1
    return {"bodies": len(empties), "parented": n}


def E(body):
    """This scene's empty for a body: names repeat across scenes (Blender suffixes them)."""
    return next(o for o in scene.objects if o.type == "EMPTY" and o.name.split(".")[0] == f"B:{body}")


def set_joint(body, angle):
    e = E(body)
    rest = Matrix([e["rest"][i * 4:i * 4 + 4] for i in range(4)])
    e.matrix_basis = rest @ Matrix.Rotation(angle, 4, Vector(e["axis"]))


def rest_all():
    for name in BODIES:
        e = next((o for o in scene.objects if o.type == "EMPTY" and o.name.split(".")[0] == f"B:{name}"), None)
        if e is not None:
            e.matrix_basis = Matrix([e["rest"][i * 4:i * 4 + 4] for i in range(4)])


def _connected(a, b):
    return BODIES[a][0] == b or BODIES[b][0] == a or a == b


def _trees():
    deps = bpy.context.evaluated_depsgraph_get()
    trees = []
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o:
            continue
        ev = o.evaluated_get(deps)
        me = ev.to_mesh()
        if len(me.polygons):
            mw = o.matrix_world
            trees.append((o.name, o["body"], BVHTree.FromPolygons([mw @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons]), bool(o.get("skin"))))
        ev.to_mesh_clear()
    return trees


def clips(only=None):
    """Pairs of parts on unconnected bodies whose surfaces cross (`only`: limit to pairs where
    one part's body is in that set)."""
    bpy.context.view_layer.update()
    trees = _trees()
    hits = []
    for i in range(len(trees)):
        na, ba, ta, sa = trees[i]
        for j in range(i + 1, len(trees)):
            nb, bb, tb, sb = trees[j]
            # Same body: one rigid part. Connected bodies: their hardware meets by design (a case on
            # its horn, a bracket on its bearing); their shells must still clear each other.
            # Same body: one rigid part, but a shell must not cut through that body's hardware
            # (it encloses it or clears it). Connected bodies: their hardware meets by design (a
            # case on its horn, a bracket on its bearing); their shells must still clear each other.
            if ba == bb:
                if sa == sb:
                    continue
            elif _connected(ba, bb) and not (sa and sb):
                continue
            if only and ba not in only and bb not in only:
                continue
            if ta.overlap(tb):
                hits.append((na, nb))
    return hits


def subtree(body):
    out = {body}
    for name, (parent, *_rest) in BODIES.items():
        if parent in out:
            out |= subtree(name)
    return out


def sweep(steps=7):
    report = {}
    for name, (_, _, joint, _) in BODIES.items():
        if not joint:
            continue
        lo, hi = joint[2]
        moving = subtree(name)
        bad = {}
        for k in range(steps):
            a = lo + (hi - lo) * k / (steps - 1)
            rest_all()
            set_joint(name, a)
            for pa, pb in clips(only=moving):
                bad.setdefault(f"{pa} x {pb}", []).append(round(math.degrees(a)))
        report[joint[0]] = bad
    rest_all()
    bpy.context.view_layer.update()
    return report


def attach(o, body=None):
    """Parent a newly built part to its body (keeping where it is)."""
    body = body or skin_body(o.name.split(".")[0])
    e = E(body)
    mw = o.matrix_world.copy()
    o.parent = e
    o.matrix_parent_inverse = e.matrix_world.inverted()
    o.matrix_world = mw
    o["body"] = body


def part_check(names, joints=(), steps=7):
    """Clips at rest involving these parts, then while each of `joints` sweeps its range."""
    names = set(names)

    def mine(pairs):
        return sorted({f"{a.split('.')[0]} x {b.split('.')[0]}" for a, b in pairs if a.split(".")[0] in names or b.split(".")[0] in names})

    rest_all()
    out = {"rest": mine(clips())}
    for body in joints:
        lo, hi = BODIES[body][2][2]
        bad = {}
        for k in range(steps):
            a = lo + (hi - lo) * k / (steps - 1)
            rest_all()
            set_joint(body, a)
            for pair in mine(clips()):
                bad.setdefault(pair, []).append(round(math.degrees(a)))
        out[BODIES[body][2][0]] = bad
    rest_all()
    bpy.context.view_layer.update()
    return out


VIEWS = {"front": (1.45, 0, math.pi / 2), "side": (1.45, 0, 0), "three_q": (1.2, 0, 2.2), "back_q": (1.2, 0, -0.9), "top": (0, 0, math.pi / 2)}


def review(names, prefix, target=None, distance=None, isolate=True, views=("three_q", "front", "side", "top")):
    """Viewport renders of the given parts, alone (other parts hidden) or in context."""
    from mathutils import Euler
    names = set(names)
    objs = [o for o in scene.objects if o.type == "MESH" and "body" in o]
    hidden = [e for e in scene.objects if e.type == "EMPTY" and e.name.startswith("B:") and not e.hide_get()]
    for e in hidden:
        e.hide_set(True)
    for o in scene.objects:
        o.select_set(False)
    bpy.context.view_layer.objects.active = None
    if isolate:
        for o in objs:
            if o.name.split(".")[0] not in names and not o.hide_get():
                o.hide_set(True)
                hidden.append(o)
    shown = [o for o in objs if o.name.split(".")[0] in names]
    pts = [o.matrix_world @ Vector(c) for o in shown for c in o.bound_box] or [Vector((0, 0, 130))]
    lo = Vector([min(p[i] for p in pts) for i in range(3)])
    hi = Vector([max(p[i] for p in pts) for i in range(3)])
    centre = target or (lo + hi) / 2
    dist = distance or max((hi - lo).length * 1.6, 120)
    area = next(a for a in bpy.context.window.screen.areas if a.type == "VIEW_3D")
    region = next(r for r in area.regions if r.type == "WINDOW")
    r3d = area.spaces.active.region_3d
    files = []
    for v in views:
        r3d.view_location = centre
        r3d.view_distance = dist
        r3d.view_rotation = Euler(VIEWS[v]).to_quaternion()
        path = f"{OUT_DIR}/{prefix}-{v}.png"
        scene.render.filepath = path
        scene.render.resolution_x, scene.render.resolution_y = 900, 700
        with bpy.context.temp_override(area=area, region=region):
            bpy.ops.render.opengl(write_still=True, view_context=True)
        files.append(path)
    for o in hidden:
        o.hide_set(False)
    return files


OUT_DIR = globals().get("OUT_DIR", "/tmp")


if RIG_ACTION == "rig":
    print(json.dumps(build_rig()))
elif RIG_ACTION == "clips":
    h = clips()
    print(json.dumps({"rest_clips": len(h), "pairs": h[:60]}, indent=1))
elif RIG_ACTION == "sweep":
    print(json.dumps(sweep(), indent=1))


def supports(tol=0.4, sample=1500):
    """Nothing floats: on each body, parts that touch (surfaces within `tol` mm or crossing) form a
    graph; every shell part must reach that body's hardware through it. Returns the shell parts
    that don't, each with its gap to the nearest part on its body."""
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    parts = {}
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o or o.hide_get():
            continue
        ev = o.evaluated_get(deps)
        me = ev.to_mesh()
        if len(me.polygons):
            mw = o.matrix_world
            vs = [mw @ v.co for v in me.vertices]
            # (face centres too: two flat faces resting on each other share no vertex)
            pts = vs + [mw @ p.center for p in me.polygons]
            step = max(1, len(pts) // sample)
            parts[o.name] = (o["body"], bool(o.get("skin")), BVHTree.FromPolygons(vs, [tuple(p.vertices) for p in me.polygons]), pts[::step])
        ev.to_mesh_clear()
    names = list(parts)
    hw = {body for body, skin, _, _ in parts.values() if not skin}
    edges = {n: set() for n in names}
    nearest = {n: (1e9, "") for n in names}
    for i, a in enumerate(names):
        ba, _, ta, va = parts[a]
        for b in names[i + 1:]:
            bb, sb, tb, vb = parts[b]
            if ba != bb:
                # A body with no hardware of its own (the tail, the jaw) hangs on its parent's
                # hardware: its servo's horn, or a boss on the frame.
                sa = parts[a][1]
                hangs = (sa and not sb and ba not in hw and BODIES.get(ba, (None,))[0] == bb) or \
                        (sb and not sa and bb not in hw and BODIES.get(bb, (None,))[0] == ba)
                if not hangs:
                    continue
            if ta.overlap(tb):
                d = 0.0
            else:
                d = min(min((tb.find_nearest(p)[3] or 1e9) for p in va), min((ta.find_nearest(p)[3] or 1e9) for p in vb))
            if d <= tol:
                edges[a].add(b)
                edges[b].add(a)
            for x, y in ((a, b), (b, a)):
                if d < nearest[x][0]:
                    nearest[x] = (d, y)
    reach = set(n for n in names if not parts[n][1])
    frontier = list(reach)
    while frontier:
        n = frontier.pop()
        for m in edges[n]:
            if m not in reach:
                reach.add(m)
                frontier.append(m)
    out = {}
    for n in names:
        if parts[n][1] and n not in reach:
            d, m = nearest[n]
            out[n.split(".")[0]] = {"touches": sorted(m.split(".")[0] for m in edges[n]), "nearest": m.split(".")[0], "gap": round(d, 2)}
    return out
