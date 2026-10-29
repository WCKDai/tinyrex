"""Audit every part of TinyREX: what it is, what it's made of, whether it can be made and fixed.

Sent after tinyrex_v2.py's, rig.py's, v4_parts.py's (builder section) and mass.py's globals.
Prints AUDIT {...}: one entry per part (role, material, body, size, volume, mass, watertight,
thinnest wall where it's printed, the parts it touches) and the findings."""
import json

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def role(o):
    n = o.name.split(".")[0]
    if not o.get("skin"):
        if n.startswith("XL330"):
            return "servo"
        if n.startswith("Bearing"):
            return "bearing"
        if n.startswith("Cable"):
            return "cable"
        if n.startswith("Bracket_"):
            return "frame (Microduck layout)"
        return "electronics"
    if n.startswith("Boss_"):
        return "insert boss"
    if "Screw" in n:
        return "fastener"
    if n.startswith(("JawShaft", "CheekPin", "ArmPin")):
        return "steel pin"
    if n.startswith(("ArmSpring", "TailSpring")):
        return "wave spring"
    if n.startswith("TailSpine"):
        return "carbon rod"
    if n.startswith(("VisorGlass", "Eye_")):
        return "polycarbonate window"
    if n.startswith(("Sole", "ToeSole", "TailTip", "ArmClutch", "TailClutch", "JawClutch")):
        return "TPU"
    if n.startswith(("ToF", "HeadIMU", "StatusLED", "Cam")):
        return "electronics"
    if any(m.type == "SOLIDIFY" for m in o.modifiers):
        return "printed shell"
    return "printed part"


def evaluated(o, deps):
    ev = o.evaluated_get(deps)
    me = ev.to_mesh()
    mw = o.matrix_world
    V = [mw @ v.co for v in me.vertices]
    P = [tuple(p.vertices) for p in me.polygons]
    N = [(mw.to_3x3() @ p.normal).normalized() for p in me.polygons]
    C = [mw @ p.center for p in me.polygons]
    A = [p.area for p in me.polygons]
    bm = bmesh.new()
    bm.from_mesh(me)
    open_edges = sum(1 for e in bm.edges if not e.is_manifold)
    bm.free()
    ev.to_mesh_clear()
    return V, P, N, C, A, open_edges


def thinnest(tree, C, N, A, sample=400):
    """Wall thickness: from points on the surface, inward along the face normal, to the first
    opposing surface (one facing back the other way); neighbouring fillet facets don't count. The
    5th percentile (by face area) of what's under 12 mm."""
    idx = sorted(range(len(C)), key=lambda i: -A[i])[:sample]
    d = []
    for i in idx:
        start, travelled = C[i] - N[i] * 0.02, 0.02
        while travelled < 12.0:
            loc, nrm, _, dist = tree.ray_cast(start, -N[i], 12.0 - travelled)
            if loc is None:
                break
            travelled += dist
            if nrm.dot(N[i]) < -0.5:  # (an opposing wall)
                d.append(travelled)
                break
            start = loc - N[i] * 0.02
            travelled += 0.02
    if not d:
        return None
    d.sort()
    return round(d[max(0, len(d) // 20)], 2)


def run():
    rest_all()
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    parts, trees = {}, {}
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o or o.hide_get():
            continue
        name = o.name.split(".")[0]
        if "." in o.name and not o.name.split(".")[1].isdigit():
            continue  # (an XL330's horn, hub, screws and sockets: part of the servo)
        V, P, N, C, A, open_edges = evaluated(o, deps)
        if not P:
            continue
        tree = BVHTree.FromPolygons(V, P)
        trees[o.name] = (tree, o)
        lo = Vector([min(v[i] for v in V) for i in range(3)])
        hi = Vector([max(v[i] for v in V) for i in range(3)])
        r = role(o)
        rec = {"role": r, "body": o["body"], "material": o.active_material.name if o.active_material else None,
               "size_mm": [round(v, 1) for v in hi - lo], "watertight": open_edges == 0}
        if r in ("printed shell", "printed part", "TPU", "insert boss"):
            rec["thinnest_mm"] = thinnest(tree, C, N, A)
        parts[o.name] = rec
    # what each printed part touches (its fixings)
    names = list(parts)
    for a in names:
        if parts[a]["role"] not in ("printed shell", "printed part", "TPU", "acrylic window", "magnet", "steel pin"):
            continue
        ta, oa = trees[a]
        touch = []
        for b in names:
            if b == a:
                continue
            tb, ob = trees[b]
            if ob["body"] != oa["body"] and BODIES.get(oa["body"], (None,))[0] != ob["body"] and BODIES.get(ob["body"], (None,))[0] != oa["body"]:
                continue
            if ta.overlap(tb):
                touch.append(b.split(".")[0])
        parts[a]["touches"] = sorted(touch)
        parts[a]["screws"] = sorted(t for t in touch if "Screw" in t)
    out = {k.split(".")[0]: v for k, v in parts.items()}
    print("AUDIT", json.dumps(out))


run()
