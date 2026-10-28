"""PolyCore STEP: export Blender geometry as exact B-rep STEP assemblies through PolyCore.

Each mesh object becomes a part. Where the object's mesh data carries a construction recipe
(`pc_recipe`: a box, a cylinder, a ruled loft through planar sections - written by the scripts that
build the model), the add-on reads the object's modifier stack on top of it and hands PolyCore the
exact construction:
  Bevel     -> fillet (radius = the bevel width) of every sharp edge
  Solidify  -> hollow: cut an inset copy of the loft (inward walls)
  Boolean   -> cut / intersect / union with the cutter object's own recipe
  Subsurf   -> left as the ruled loft under it (reported)
Anything it can't express goes as a closed mesh and comes back faceted. Parts are grouped into the
bodies they move with (an object's `body` property and its `B:<body>` rig empty; otherwise one
body), so the STEP is an assembly with placements, ready to assemble in Polykrom.

The geometry work is done by the `polycore-step` command-line tool (tools/polycore-step), built on
PolyCore; set its path in the add-on's preferences.
"""
import json
import math
import os
import subprocess
import tempfile

import bpy
from bpy.props import BoolProperty, IntProperty, StringProperty
from mathutils import Matrix, Vector

UNSUPPORTED = "unsupported"


def colmajor(m):
    return [m[r][c] for c in range(4) for r in range(4)]


def unit_mm(scene):
    return scene.unit_settings.scale_length * 1000.0 if scene.unit_settings.system != "NONE" else 1000.0


# ---------------------------------------------------------------- recipe from an object
def base_recipe(me):
    """The mesh's construction: its recorded recipe, else one recognised from its shape."""
    raw = me.get("pc_recipe")
    if raw:
        return json.loads(raw)
    return infer_recipe(me)


def infer_recipe(me, tol=1e-4):
    """Recognise a box (axis-aligned, 8 vertices), a cylinder or a ring along local Z (one or two
    circles of vertices at two heights), or a ruled loft (vertices in consecutive planar rings of
    equal size, the rings in parallel planes along one axis)."""
    V = [v.co.copy() for v in me.vertices]
    nv, nf = len(V), len(me.polygons)
    if nv < 6:
        return None
    if nv == 8 and nf == 6:
        xs, ys, zs = ({round(v[i], 4) for v in V} for i in range(3))
        if len(xs) == len(ys) == len(zs) == 2:
            return {"box": {"min": [min(xs), min(ys), min(zs)], "max": [max(xs), max(ys), max(zs)]}}
    zs = sorted({round(v.z, 4) for v in V})
    radii = sorted({round(math.hypot(v.x, v.y), 3) for v in V})
    per_z = [{round(math.hypot(v.x, v.y), 3) for v in V if abs(v.z - z) < 1e-3} for z in zs] if len(zs) == 2 else []
    if len(zs) == 2 and len(radii) in (1, 2) and nv >= 12 and per_z[0] == per_z[1]:  # (a cone has a radius per end)
        z0, z1 = zs
        node = {"cylinder": {"base": [0.0, 0.0, z0], "axis": [0.0, 0.0, 1.0], "radius": radii[-1], "height": z1 - z0}}
        if len(radii) == 2:
            node = {"cut": [node, {"cylinder": {"base": [0.0, 0.0, z0 - 0.1], "axis": [0.0, 0.0, 1.0], "radius": radii[0], "height": z1 - z0 + 0.2}}]}
        return node
    for k in sorted({len(p.vertices) for p in me.polygons if len(p.vertices) >= 3}, reverse=True):
        if nv % k or nv // k < 2 or nf != k * (nv // k - 1) + 2:
            continue
        rings = [V[i * k:(i + 1) * k] for i in range(nv // k)]
        for axis in range(3):
            if all(max(abs(p[axis] - r[0][axis]) for p in r) < tol for r in rings):
                coords = [r[0][axis] for r in rings]
                if all(b - a > tol for a, b in zip(coords, coords[1:])) or all(a - b > tol for a, b in zip(coords, coords[1:])):
                    u, w = [i for i in range(3) if i != axis]
                    unit = [[1.0 if i == j else 0.0 for i in range(3)] for j in range(3)]
                    secs = []
                    for r in rings:
                        o = [0.0, 0.0, 0.0]
                        o[axis] = r[0][axis]
                        secs.append({"origin": o, "x": unit[u], "y": unit[w], "points": [[p[u], p[w]] for p in r]})
                    return {"loft": {"sections": secs}}
    return None


def inset_polygon(points, d):
    """A convex polygon moved inward by d (its edges offset, the corners re-intersected)."""
    n = len(points)
    area = sum(points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1] for i in range(n)) / 2
    s = 1.0 if area > 0 else -1.0
    lines = []
    for i in range(n):
        (ax, ay), (bx, by) = points[i], points[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        L = math.hypot(ex, ey) or 1.0
        nx, ny = -ey / L * s, ex / L * s  # inward (left of a counter-clockwise edge)
        lines.append(((ax + nx * d, ay + ny * d), (ex, ey)))
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            out.append(list(p2))
            continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        out.append([p1[0] + d1[0] * t, p1[1] + d1[1] * t])
    return out


def inset_loft(node, d):
    """The inside of a hollowed loft: every section inset by d, the end sections moved in by d."""
    if "loft" not in node:
        return None
    secs = node["loft"]["sections"]
    normal = lambda s: Vector(s["x"]).cross(Vector(s["y"]))
    out = []
    for i, s in enumerate(secs):
        o = Vector(s["origin"])
        if i in (0, len(secs) - 1):
            other = Vector(secs[1 if i == 0 else -2]["origin"])
            nrm = normal(s)
            step = d if nrm.dot(other - o) > 0 else -d
            o = o + nrm * step
        out.append({"origin": list(o), "x": s["x"], "y": s["y"], "points": inset_polygon(s["points"], d)})
    return {"loft": {"sections": out}}


def tool_size(node):
    """A rough size (bounding-box volume) of a recipe node, to order set operations."""
    pts = []

    def walk(n):
        k, v = next(iter(n.items()))
        if k == "box":
            pts.extend([v["min"], v["max"]])
        elif k == "cylinder":
            b, r, h = Vector(v["base"]), v["radius"], v["height"]
            pts.extend([list(b - Vector((r, r, r))), list(b + Vector((r, r, r)) + Vector(v["axis"]) * h)])
        elif k == "loft":
            for sec in v["sections"]:
                o, x, y = Vector(sec["origin"]), Vector(sec["x"]), Vector(sec["y"])
                pts.extend([list(o + x * p[0] + y * p[1]) for p in sec["points"]])
        elif k in ("transform", "fillet", "chamfer"):
            walk(v["body"])
        elif k in ("cut", "union", "intersect"):
            walk(v[0])
    walk(node)
    if not pts:
        return 0.0
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    return (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2])


def object_recipe(o):
    """(node in the object's own coordinates, warnings) - or (None, reason).

    Cuts and intersects after the last fillet are set operations on one solid and commute, so they
    are applied largest tool first: a part split by a half-space is split before its smaller cuts,
    which PolyCore's booleans handle far better (TinyREX's body halves fail the other way round)."""
    me = o.data
    base = base_recipe(me)
    if base is None:
        return None, "no construction recipe on its mesh"
    node, warnings = base, []
    ops = []  # (op, tool) set operations waiting to be applied, after `node`
    covered = set(filter(None, me.get("pc_recipe_covers", "").split(",")))  # (modifiers the recipe includes)

    def flush():
        nonlocal node, ops
        if all(op in ("cut", "intersect") for op, _ in ops):
            ops.sort(key=lambda t: -tool_size(t[1]))
        for op, tool in ops:
            node = {op: [node, tool]}
        ops = []

    for m in o.modifiers:
        if not m.show_viewport or m.name in covered:
            continue
        if m.type == "BEVEL":
            flush()
            ang = math.degrees(m.angle_limit) if m.limit_method == "ANGLE" else 20.0
            node = {"fillet": {"body": node, "radius": float(m.width), "edges": "all", "min_angle_deg": ang}}
        elif m.type == "SOLIDIFY":
            if m.offset > -0.99:
                return None, f"solidify '{m.name}' isn't an inward wall"
            inner = inset_loft(base, m.thickness)
            if inner is None:
                return None, f"solidify '{m.name}' on a non-loft"
            ops.append(("cut", inner))
        elif m.type == "BOOLEAN":
            if m.operand_type != "OBJECT" or m.object is None:
                return None, f"boolean '{m.name}' without a cutter object"
            tool = base_recipe(m.object.data) if m.object.type == "MESH" else None
            if tool is None:
                return None, f"boolean cutter '{m.object.name}' has no recipe"
            rel = o.matrix_world.inverted() @ m.object.matrix_world
            tool = {"transform": {"body": tool, "matrix": colmajor(rel)}}
            ops.append(({"DIFFERENCE": "cut", "INTERSECT": "intersect", "UNION": "union"}[m.operation], tool))
        elif m.type == "SUBSURF":
            warnings.append(f"subdivision '{m.name}' exported as the ruled loft under it")
        else:
            return None, f"modifier '{m.name}' ({m.type}) has no exact equivalent"
    flush()
    return node, warnings


def evaluated_mesh(o, deps, M):
    ev = o.evaluated_get(deps)
    me = ev.to_mesh()
    me.calc_loop_triangles()
    pos = [list(M @ v.co) for v in me.vertices]
    tris = [list(t.vertices) for t in me.loop_triangles]
    ev.to_mesh_clear()
    return {"positions": pos, "triangles": tris}


def body_of(o, scene):
    name = o.get("body")
    if name:
        e = next((x for x in scene.objects if x.type == "EMPTY" and x.name.split(".")[0] == f"B:{name}"), None)
        if e is not None:
            return name, e.matrix_world.copy()
    return "root", Matrix.Identity(4)


def build(context, objects):
    scene = context.scene
    deps = context.evaluated_depsgraph_get()
    k = unit_mm(scene)
    S = Matrix.Scale(k, 4)
    bodies, parts, notes = {}, [], {}
    for o in objects:
        if o.type != "MESH" or o.hide_get() or o.name.startswith("Cut_"):
            continue
        bname, frame = body_of(o, scene)
        bodies.setdefault(bname, frame)
        to_body = S @ frame.inverted() @ o.matrix_world  # (object -> body frame, in mm)
        node, extra = object_recipe(o)
        part = {"name": o.name, "body": bname, "mesh": evaluated_mesh(o, deps, to_body)}
        mat = o.active_material
        if mat is not None:
            part["color"] = list(mat.diffuse_color[:3])
        if node is not None:
            part["solid"] = {"transform": {"body": node, "matrix": colmajor(to_body)}}
            notes[o.name] = extra
        else:
            notes[o.name] = [f"faceted: {extra}"]
        parts.append(part)
    recipe = {"name": scene.name, "bodies": [{"name": b, "placement": colmajor(S @ f @ S.inverted())} for b, f in bodies.items()], "parts": parts}
    return recipe, notes


# ---------------------------------------------------------------- Blender UI
class PolycoreStepPreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    tool: StringProperty(name="polycore-step", subtype="FILE_PATH", default="",
                         description="Path to the polycore-step executable (tools/polycore-step, cargo build --release)")
    threads: IntProperty(name="Threads", default=0, min=0, description="Worker threads (0: all cores)")
    timeout: IntProperty(name="Seconds per part", default=60, min=5)

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "tool")
        col.prop(self, "threads")
        col.prop(self, "timeout")


class POLYCORE_OT_export_step(bpy.types.Operator):
    """Export the visible (or selected) mesh objects as a STEP assembly through PolyCore"""
    bl_idname = "polycore.export_step"
    bl_label = "Export STEP (PolyCore)"
    bl_options = {"REGISTER"}

    filepath: StringProperty(subtype="FILE_PATH")
    selected_only: BoolProperty(name="Selected only", default=False)
    keep_recipe: BoolProperty(name="Keep the recipe JSON", default=True,
                              description="Write the recipe next to the STEP, to re-run polycore-step by hand")

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = bpy.path.ensure_ext(bpy.path.abspath("//") + (context.scene.name or "model"), ".step")
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        prefs = context.preferences.addons[__package__].preferences
        tool = bpy.path.abspath(prefs.tool)
        if not tool or not os.path.exists(tool):
            self.report({"ERROR"}, "Set the path to polycore-step in the add-on preferences")
            return {"CANCELLED"}
        objects = context.selected_objects if self.selected_only else context.scene.objects
        recipe, notes = build(context, objects)
        out = bpy.path.ensure_ext(self.filepath, ".step")
        base = os.path.splitext(out)[0]
        recipe_path = base + ".recipe.json" if self.keep_recipe else tempfile.mktemp(suffix=".json")
        report_path = base + ".report.json"
        with open(recipe_path, "w") as f:
            json.dump(recipe, f)
        cmd = [tool, recipe_path, out, "--report", report_path, "--timeout-s", str(prefs.timeout)]
        if prefs.threads:
            cmd += ["--threads", str(prefs.threads)]
        context.window_manager.progress_begin(0, 1)
        try:
            run = subprocess.run(cmd, capture_output=True, text=True)
        finally:
            context.window_manager.progress_end()
        if not os.path.exists(report_path):
            self.report({"ERROR"}, f"polycore-step failed: {run.stderr[-400:]}")
            return {"CANCELLED"}
        report = json.load(open(report_path))
        text = bpy.data.texts.get("PolyCore report") or bpy.data.texts.new("PolyCore report")
        text.clear()
        counts = {}
        for p in report.get("parts", []):
            counts[p["status"]] = counts.get(p["status"], 0) + 1
            o = bpy.data.objects.get(p["name"])
            if o is not None:
                o["pc_status"] = p["status"]
            extra = "; ".join((p.get("warnings") or []) + notes.get(p["name"], []) + ([p["error"]] if p.get("error") else []))
            text.write(f"{p['status']:20s} {p['name']:28s} {p.get('representation', ''):12s} {extra}\n")
        summary = ", ".join(f"{v} {k}" for k, v in sorted(counts.items()))
        text.write(f"\n{summary}\nSTEP: {out}\nvalidation: {json.dumps(report.get('validation'))}\n")
        if not self.keep_recipe:
            os.remove(recipe_path)
        self.report({"INFO"} if run.returncode == 0 else {"WARNING"}, f"PolyCore STEP: {summary} (see the 'PolyCore report' text)")
        return {"FINISHED"}


class POLYCORE_PT_panel(bpy.types.Panel):
    bl_label = "PolyCore STEP"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "PolyCore"

    def draw(self, context):
        col = self.layout.column()
        op = col.operator(POLYCORE_OT_export_step.bl_idname, text="Export scene as STEP")
        op.selected_only = False
        op = col.operator(POLYCORE_OT_export_step.bl_idname, text="Export selection as STEP")
        op.selected_only = True
        o = context.active_object
        if o is not None and o.type == "MESH":
            col.separator()
            node, extra = object_recipe(o)
            col.label(text=f"{o.name}: {'exact recipe' if node is not None else 'faceted'}")
            if node is None:
                col.label(text=str(extra))
            elif "pc_status" in o:
                col.label(text=f"last export: {o['pc_status']}")


def menu_export(self, context):
    self.layout.operator(POLYCORE_OT_export_step.bl_idname, text="STEP via PolyCore (.step)")


CLASSES = (PolycoreStepPreferences, POLYCORE_OT_export_step, POLYCORE_PT_panel)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.TOPBAR_MT_file_export.append(menu_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_export)
    for c in reversed(CLASSES):
        bpy.utils.unregister_class(c)
