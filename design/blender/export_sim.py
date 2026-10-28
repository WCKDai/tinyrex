"""Export TinyREX for simulation: per-body inertials, the extra bodies, foot contacts and meshes.

Sent after tinyrex_v2.py's, rig.py's, v4_parts.py's (builder section) and mass.py's globals, with
SIM_DIR set. Writes SIM_DIR/tinyrex_bodies.json and SIM_DIR/meshes/<body>.stl (TinyREX's own
parts only, in each body's frame, metres). sim/build_mjcf.py turns these and Microduck's MJCF
(Apache-2.0) into sim/tinyrex.xml.

Inertia: each part's full tensor is integrated over its mesh (as a solid of its density - shells
are modelled with their real walls, solid-modelled parts at the walls-plus-infill fraction), not
treated as a point; Microduck's bodies keep their MJCF inertia, less the shells TinyREX replaces."""
import json
import math
import os
import struct

import bpy
from mathutils import Matrix, Vector

SIM_DIR = globals().get("SIM_DIR", "/tmp/tinyrex_sim")


def second_moments(verts, polys):
    """Volume, first moment and second moment (about the origin) of a closed mesh, mm units."""
    vol, first = 0.0, Vector()
    C = Matrix(((0, 0, 0), (0, 0, 0), (0, 0, 0)))
    for p in polys:
        a = verts[p[0]]
        for i in range(1, len(p) - 1):
            b, c = verts[p[i]], verts[p[i + 1]]
            det = a.dot(b.cross(c))
            vol += det / 6.0
            first += det / 24.0 * (a + b + c)
            s = a + b + c
            for u in (a, b, c, s):
                C += Matrix(((u.x * u.x, u.x * u.y, u.x * u.z), (u.y * u.x, u.y * u.y, u.y * u.z), (u.z * u.x, u.z * u.y, u.z * u.z))) * (det / 120.0)
    return vol, first, C


def part_inertia(o, deps, to_body, rho_k):
    """(mass g, centre mm, inertia about the centre g mm^2 as a 3x3) of an object in a body frame."""
    ev = o.evaluated_get(deps)
    me = ev.to_mesh()
    M = to_body @ o.matrix_world
    V = [M @ v.co for v in me.vertices]
    P = [tuple(p.vertices) for p in me.polygons]
    ev.to_mesh_clear()
    vol, first, C = second_moments(V, P)
    if vol < 0:
        vol, first, C = -vol, -first, C * -1.0
    if vol < 1e-6:
        return 0.0, Vector(), Matrix.Identity(3) * 0
    m = vol / 1000.0 * rho_k  # g (vol mm3 -> cm3)
    c = first / vol
    Io = Matrix.Identity(3) * (C[0][0] + C[1][1] + C[2][2]) - C  # about the origin, per unit density (mm5)
    Io = Io * (rho_k / 1000.0)  # g mm2
    shift = Matrix.Identity(3) * c.dot(c) - Matrix(((c.x * c.x, c.x * c.y, c.x * c.z), (c.y * c.x, c.y * c.y, c.y * c.z), (c.z * c.x, c.z * c.y, c.z * c.z)))
    return m, c, Io - shift * m


def combine(items):
    """items: (mass, centre, inertia about own centre) -> total mass, centre, inertia about it."""
    M = sum(m for m, _, _ in items)
    c = sum((cc * m for m, cc, _ in items), Vector()) / M
    I = Matrix(((0, 0, 0), (0, 0, 0), (0, 0, 0)))
    for m, cc, Ii in items:
        d = cc - c
        I += Ii + (Matrix.Identity(3) * d.dot(d) - Matrix(((d.x * d.x, d.x * d.y, d.x * d.z), (d.y * d.x, d.y * d.y, d.y * d.z), (d.z * d.x, d.z * d.y, d.z * d.z)))) * m
    return M, c, I


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"TinyREX".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = (b - a).cross(c - a)
            n = n.normalized() if n.length else n
            f.write(struct.pack("<12fH", n.x, n.y, n.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, 0))


def run():
    rest_all()
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    os.makedirs(os.path.join(SIM_DIR, "meshes"), exist_ok=True)
    frames = {b: E(b).matrix_world.copy() for b in BODIES if BODIES[b][1] is not None}
    to_body = {b: m.inverted() for b, m in frames.items()}
    items = {b: [] for b in frames}
    # Microduck's bodies, as its MJCF has them (inertia about the centre, body axes)
    root = ET.parse(MJCF).getroot()
    for b in root.iter("body"):
        i = b.find("inertial")
        if i is None:
            continue
        xx, yy, zz, xy, xz, yz = (float(v) * 1e9 for v in i.get("fullinertia").split())  # kg m2 -> g mm2
        items[b.get("name")].append((float(i.get("mass")) * 1000.0, Vector([float(v) for v in i.get("pos").split()]) * 1000.0,
                                     Matrix(((xx, xy, xz), (xy, yy, yz), (xz, yz, zz)))))
    # minus the shells TinyREX replaces (Microduck's reference scene, aligned with ours)
    seen = set()
    for o in bpy.data.scenes["Microduck (reference)"].objects:
        n = o.name.split(".")[0][3:]
        if o.type != "MESH" or n not in REPLACED:
            continue
        b = MD_BODY.get(n, HEAD)
        m, c, I = part_inertia(o, deps, to_body[b], REPLACED[n])
        key = (n, round(c.x), round(c.y), round(c.z))
        if key in seen:
            continue
        seen.add(key)
        items[b].append((-m, c, I * -1.0))
    # plus TinyREX's own parts, and their meshes
    tris = {b: [] for b in frames}
    for o in scene.objects:
        if o.type != "MESH" or "body" not in o or o.hide_get():
            continue
        n, b = o.name.split(".")[0], o["body"]
        if b not in frames:
            continue
        ev = o.evaluated_get(deps)
        me = ev.to_mesh()
        M = to_body[b] @ o.matrix_world
        V = [M @ v.co * 0.001 for v in me.vertices]
        for p in me.polygons:
            for k in range(1, len(p.vertices) - 1):
                tris[b].append((V[p.vertices[0]], V[p.vertices[k]], V[p.vertices[k + 1]]))
        ev.to_mesh_clear()
        sub = o.name.split(".")
        if n in ("XL330_ArmL", "XL330_ArmR", "XL330_TailYaw"):
            if len(sub) == 1 or sub[1].isdigit():
                m, c, I = part_inertia(o, deps, to_body[b], 1.0)
                k = XL330_G / m if m else 0
                items[b].append((XL330_G, c, I * k))
        elif n in ("CameraHolder", "CameraLens"):
            items[b].append(part_inertia(o, deps, to_body[b], PLA))
        elif o.get("skin") and not n.startswith("XL330"):
            if "Screw" in n:
                items[b].append((SCREW_G, to_body[b] @ o.matrix_world.translation, Matrix.Identity(3) * 0))
            elif n.startswith(("ToFBoard", "HeadIMU")):
                m, c, I = part_inertia(o, deps, to_body[b], 1.0)
                items[b].append((1.5, c, I * (1.5 / m if m else 0)))
            else:
                rho, k = density(n, o)
                items[b].append(part_inertia(o, deps, to_body[b], rho * k))
    out = {"bodies": {}, "extra": {}, "feet": {}}
    for b, its in items.items():
        if not its:
            continue
        M, c, I = combine(its)
        out["bodies"][b] = {"mass": M / 1000.0, "pos": [v / 1000.0 for v in c],
                            "fullinertia": [I[0][0] * 1e-9, I[1][1] * 1e-9, I[2][2] * 1e-9, I[0][1] * 1e-9, I[0][2] * 1e-9, I[1][2] * 1e-9]}
        if tris[b]:
            write_stl(os.path.join(SIM_DIR, "meshes", f"{b}.stl"), tris[b])
            out["bodies"][b]["mesh"] = f"{b}.stl"
    for b in ("tail", "jaw", "arm_L", "arm_R"):
        parent, _, joint, _ = BODIES[b]
        e = E(b)
        local = Matrix([e["rest"][i * 4:i * 4 + 4] for i in range(4)])
        q = local.to_quaternion()
        out["extra"][b] = {"parent": parent, "pos": [v / 1000.0 for v in local.translation], "quat": [q.w, q.x, q.y, q.z],
                           "joint": joint[0], "axis": list(e["axis"]), "range": [e["lo"], e["hi"]]}
    # foot contacts: the soles and toe soles as boxes, in each ankle's frame
    for side, body in (("L", "ankle_left"), ("R", "ankle_right")):
        boxes = []
        for o in scene.objects:
            if o.type == "MESH" and o.name.split(".")[0] in (f"Sole_{side}", f"ToeSole_{side}0", f"ToeSole_{side}2"):
                pts = [to_body[body] @ o.matrix_world @ Vector(v) for v in o.bound_box]
                lo = Vector([min(p[i] for p in pts) for i in range(3)])
                hi = Vector([max(p[i] for p in pts) for i in range(3)])
                boxes.append({"pos": [v / 2000.0 for v in lo + hi], "size": [v / 2000.0 for v in hi - lo]})
        out["feet"][body] = boxes
    with open(os.path.join(SIM_DIR, "tinyrex_bodies.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("SIM", json.dumps({b: round(v["mass"] * 1000, 1) for b, v in out["bodies"].items()}))


run()
