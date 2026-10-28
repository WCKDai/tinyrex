"""TinyREX v2 in Blender: Microduck's frame and hardware layout, TinyREX's own parts and skins.

Sent to the open Blender in phases through the MCP bridge (tools/blendexec.py), with PHASE set:
    "skeleton"     own XL330s, bearings and brackets at Microduck's joint placements
    "electronics"  own compute board, robot hat, battery, camera, speaker, ToF and IMU boards
Placements come from Microduck's MJCF (Apache-2.0, pollen-robotics/microduck_rl); no Microduck
mesh is used: every part here is modelled from datasheet / measured dimensions.
Millimetres, X forward, Y left, Z up, ground at z = 0 (Microduck's frame, lifted to the ground).
"""
import math
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Quaternion, Vector

MJCF = "/Users/dearwolf/wckdboy/reference/microduck/robot_walk.xml"
PHASE = globals().get("PHASE", "skeleton")
SCENE = globals().get("SCENE_NAME", "TinyREX v2")
GROUND = -13.0  # lifts Microduck's MJCF frame so its soles rest on z = 0 (measured against the reference)

scene = bpy.data.scenes.get(SCENE) or bpy.data.scenes.new(SCENE)
bpy.context.window.scene = scene
scene.unit_settings.scale_length = 0.001
scene.unit_settings.length_unit = "MILLIMETERS"


def collection(name):
    c = bpy.data.collections.get(f"{SCENE}: {name}")
    if c is None:
        c = bpy.data.collections.new(f"{SCENE}: {name}")
        scene.collection.children.link(c)
    for o in list(c.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    return c


def mat(name, hexrgb, rough=0.45, metal=0.0, coat=0.0):
    m = bpy.data.materials.get(f"TR_{name}") or bpy.data.materials.new(f"TR_{name}")
    m.use_nodes = True
    h = hexrgb.lstrip("#")
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    lin = [((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c]
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*lin, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if "Coat Weight" in p.inputs:
        p.inputs["Coat Weight"].default_value = coat
    m.diffuse_color = (*lin, 1)
    return m


SERVO_BLACK = mat("ServoBlack", "#151618", 0.5)
HORN = mat("Horn", "#B9BDC2", 0.3, metal=0.8)
SCREW = mat("Screw", "#2A2B2D", 0.35, metal=0.6)
JST = mat("JST", "#F2F0EA", 0.6)
BRACKET = mat("Bracket", "#3A3D42", 0.55)
BEARING = mat("Bearing", "#C9CDD2", 0.25, metal=1.0)
PCB_GREEN = mat("PCBGreen", "#1F6B3A", 0.5)
PCB_BLUE = mat("PCBBlue", "#1D3F8F", 0.5)
CHIP = mat("Chip", "#1B1B1D", 0.35)
GOLD = mat("Gold", "#D4AF37", 0.25, metal=1.0)
SILVER = mat("Silver", "#C0C4C8", 0.25, metal=1.0)
BATTERY = mat("Battery", "#2E3033", 0.5)
LABEL = mat("Label", "#C6F000", 0.5)
GLASS = mat("Glass", "#20303F", 0.05, coat=1.0)


def obj_from(name, me, coll, material, matrix=None, smooth=False):
    o = bpy.data.objects.new(name, me)
    coll.objects.link(o)
    me.materials.append(material)
    if matrix is not None:
        o.matrix_world = matrix
    if smooth:
        for poly in me.polygons:
            poly.use_smooth = True
    return o


def pc_tag(me, node):
    """Record a mesh's exact construction (PolyCore recipe node, in the mesh's own coordinates) on
    its data, for the PolyCore STEP exporter (tools/polycore-blender)."""
    import json
    me["pc_recipe"] = json.dumps(node)
    return me


def pc_loft(rings, axis):
    """A ruled-loft node through planar rings of 3D points, each in a plane across `axis`."""
    k = "xyz".index(axis)
    u, w = [i for i in range(3) if i != k]
    unit = [[1.0 if i == j else 0.0 for i in range(3)] for j in range(3)]
    secs = []
    for ring in rings:
        o = [0.0, 0.0, 0.0]
        o[k] = float(ring[0][k])
        secs.append({"origin": o, "x": unit[u], "y": unit[w], "points": [[float(p[u]), float(p[w])] for p in ring]})
    return {"loft": {"sections": secs}}


def box_mesh(name, lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    return pc_tag(me, {"box": {"min": [float(c) for c in (x0, y0, z0)], "max": [float(c) for c in (x1, y1, z1)]}})


def cyl_mesh(name, r, z0, z1, n=32, r_in=0.0):
    """A cylinder (or a ring with r_in) along local Z."""
    v, f = [], []
    for z in (z0, z1):
        for k in range(n):
            a = 2 * math.pi * k / n
            v.append((r * math.cos(a), r * math.sin(a), z))
    if r_in:
        for z in (z0, z1):
            for k in range(n):
                a = 2 * math.pi * k / n
                v.append((r_in * math.cos(a), r_in * math.sin(a), z))
        for k in range(n):
            j = (k + 1) % n
            f += [(k, j, n + j, n + k), (2 * n + j, 2 * n + k, 3 * n + k, 3 * n + j), (2 * n + k, 2 * n + j, j, k), (n + k, n + j, 3 * n + j, 3 * n + k)]
    else:
        f.append(tuple(range(n - 1, -1, -1)))
        f.append(tuple(range(n, 2 * n)))
        for k in range(n):
            j = (k + 1) % n
            f.append((k, j, n + j, n + k))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    node = {"cylinder": {"base": [0.0, 0.0, float(z0)], "axis": [0.0, 0.0, 1.0], "radius": float(r), "height": float(z1 - z0)}}
    if r_in:
        node = {"cut": [node, {"cylinder": {"base": [0.0, 0.0, float(z0) - 0.1], "axis": [0.0, 0.0, 1.0], "radius": float(r_in), "height": float(z1 - z0) + 0.2}}]}
    pc_tag(me, node)
    return me


def bevel(o, width, segments=3):
    b = o.modifiers.new("Bevel", "BEVEL")
    b.width = width
    b.segments = segments
    b.limit_method = "ANGLE"
    b.harden_normals = True
    o.data.shade_smooth()
    try:
        o.data.set_sharp_from_angle(angle=math.radians(35))
    except AttributeError:
        pass
    return o


def axis_matrix(axis):
    """Rotation taking local Z onto axis 'X', 'Y', 'Z' (or their negatives)."""
    return {
        "X": Matrix.Rotation(math.pi / 2, 4, "Y"), "-X": Matrix.Rotation(-math.pi / 2, 4, "Y"),
        "Y": Matrix.Rotation(-math.pi / 2, 4, "X"), "-Y": Matrix.Rotation(math.pi / 2, 4, "X"),
        "Z": Matrix.Identity(4), "-Z": Matrix.Rotation(math.pi, 4, "X"),
    }[axis]


def part(name, coll, material, mesh, local, world, bevel_w=0.0, smooth=False):
    o = obj_from(name, mesh, coll, material, world @ local, smooth=smooth)
    if bevel_w:
        bevel(o, bevel_w)
    return o


# ---------------------------------------------------------------- own part models (local frames)
def xl330(name, coll, world):
    """Dynamixel XL330/XC330: 26 x 20 x 34 mm case, output axis along local X at the origin,
    9.5 mm below the case top; a horn on +X, an idler hub on -X, screws, two cable sockets."""
    T = Matrix.Translation
    part(name, coll, SERVO_BLACK, box_mesh(name, (-13, -10, -24.5), (13, 10, 9.5)), Matrix.Identity(4), world, bevel_w=1.2)
    horn = part(f"{name}.horn", coll, HORN, cyl_mesh(f"{name}.horn", 9.0, 0, 1.5, 48), T((13, 0, 0)) @ axis_matrix("X"), world, smooth=True)
    part(f"{name}.hub", coll, HORN, cyl_mesh(f"{name}.hub", 5.0, 0, 1.5, 32), T((-13, 0, 0)) @ axis_matrix("-X"), world, smooth=True)
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        part(f"{name}.hs{k}", coll, SCREW, cyl_mesh(f"{name}.hs{k}", 1.1, 0, 0.6, 12), T((14.5, 5.6 * math.cos(a), 5.6 * math.sin(a))) @ axis_matrix("X"), world)
    for side in (1, -1):  # case screws on both broad faces
        for zx in ((-8, 6), (8, 6), (-8, -21), (8, -21)):
            part(f"{name}.cs{side}{zx}", coll, SCREW, cyl_mesh(f"{name}.cs", 0.9, 0, 0.4, 10), T((zx[0], side * 10, zx[1])) @ axis_matrix("Y" if side > 0 else "-Y"), world)
    for k, y in enumerate((-5.5, 5.5)):  # JST sockets on the bottom
        part(f"{name}.jst{k}", coll, JST, box_mesh(f"{name}.jst", (-4, y - 3.5, -26.5), (4, y + 3.5, -24.5)), Matrix.Identity(4), world)


def bearing(name, coll, world, d, w):
    part(name, coll, BEARING, cyl_mesh(name, d / 2, 0, w, 40, r_in=d / 2 - 3.5), Matrix.Identity(4), world, smooth=True)


PRINTED = mat("PrintedCream", "#ECE5D6", 0.5)
VISIBLE = {"leg", "yaw2roll", "hip_l", "bearing_roll", "ankle_left", "ankle_right"}


def bracket(name, coll, world, lo, size, mesh=""):
    """A bracket filling a Microduck bracket's envelope: printed cream where it shows on the legs,
    dark grey where the skins hide it."""
    hi = tuple(a + b for a, b in zip(lo, size))
    shown = mesh in VISIBLE
    part(name, coll, PRINTED if shown else BRACKET, box_mesh(name, lo, hi), Matrix.Identity(4), world, bevel_w=min(4.0 if shown else 1.5, min(size) / 2 - 0.05))


# Local bounding boxes (min corner, size) of Microduck's structural parts, measured: my brackets fill
# the same envelopes until TinyREX's skins and links replace them.
BRACKETS = {
    "leg": ((32.5, 12.0, -12.2), (8.0, 20.0, 58.0)),
    "yaw2roll": ((6.0, -13.3, -5.0), (23.0, 25.8, 20.5)),
    "upper_leg_rigidity_plate": ((42.5, -12.0, -11.0), (1.0, 45.0, 58.1)),
    "hip_l": ((8.0, -25.0, -9.5), (32.5, 34.5, 19.0)),
    "bearing_roll": ((6.0, -14.5, -24.5), (23.0, 3.0, 40.0)),
    "yaw_roll_motion": ((-24.5, -18.0, -4.5), (34.0, 35.9, 22.5)),
    "upper_leg_right": ((-70.5, -13.5, -12.2), (28.0, 47.7, 61.0)),
    "upper_leg_left": ((42.5, -13.5, -12.2), (28.0, 47.7, 61.0)),
    "trunk_base": ((-28.5, -13.0, 16.5), (57.0, 36.0, 3.0)),
    "neck": ((0.0, -32.0, 69.5), (2.0, 20.0, 11.0)),
    "neck_pitch": ((-17.5, -9.0, -37.8), (35.0, 18.0, 27.7)),
    "motor_support": ((-39.5, -26.2, 0.4), (73.5, 54.2, 18.8)),
    "ankle_left": ((30.4, 0.2, -22.5), (39.5, 36.5, 25.5)),
    "ankle_right": ((-69.9, 0.2, -22.5), (39.5, 36.5, 25.5)),
    "power_support": ((-27.2, 18.0, -26.5), (54.5, 17.0, 83.5)),
    "banana_pcb_locker": ((-27.0, 32.7, 49.0), (54.1, 3.8, 6.7)),
}
SHELLS = {"top_head_shell", "bottom_head_shell", "left_shell", "right_shell", "jaw", "jaw_soft", "soft_mouth_top",
          "face_part", "noenoeil", "foot_left", "foot_right", "sole_left", "sole_right", "rim", "tire", "roller_blade"}
ELECTRONICS = {"pcb__raspberry_pi_zero_2_w", "elec_rpi_robot_hat_pcb", "np_f970", "speaker", "m12_lens_holder", "lens"}


# ---------------------------------------------------------------- Microduck placements from the MJCF
def mjcf_geoms():
    root = ET.parse(MJCF).getroot()

    def tf(el):
        pos = Vector([float(v) for v in el.get("pos", "0 0 0").split()]) * 1000.0
        q = el.get("quat")
        return Matrix.Translation(pos) @ (Quaternion([float(v) for v in q.split()]) if q else Quaternion()).to_matrix().to_4x4()

    out = []

    def walk(body, parent):
        world = parent @ tf(body)
        for g in body.findall("geom"):
            # (a mesh placed again as a collision-only geom is the same part: skip the copy)
            if g.get("mesh") and "collision" not in (g.get("class") or ""):
                out.append((g.get("mesh"), world @ tf(g)))
        for child in body.findall("body"):
            walk(child, world)

    lift = Matrix.Translation((0, 0, GROUND))
    for b in root.find("worldbody").findall("body"):
        walk(b, lift)
    return out


GEOMS = mjcf_geoms()


def build_skeleton():
    coll = collection("skeleton")
    n = {"servo": 0, "bearing": 0, "bracket": 0}
    for i, (mesh, world) in enumerate(GEOMS):
        if mesh == "xl330":
            xl330(f"XL330_{n['servo']:02d}", coll, world)
            n["servo"] += 1
        elif mesh.startswith("seeed_bearing"):
            d, w = (22, 4) if "22x16x4" in mesh else (15, 3)
            bearing(f"Bearing_{n['bearing']:02d}", coll, world, d, w)
            n["bearing"] += 1
        elif mesh in BRACKETS:
            lo, size = BRACKETS[mesh]
            bracket(f"Bracket_{mesh}_{i}", coll, world, lo, size, mesh)
            n["bracket"] += 1
    return n


def build_electronics():
    """Own models of Microduck's electronics, in each part's local frame (measured), at its place."""
    coll = collection("electronics")
    T = Matrix.Translation
    I = Matrix.Identity(4)
    made = 0
    for i, (mesh, world) in enumerate(GEOMS):
        if mesh == "pcb__raspberry_pi_zero_2_w":
            # Compute board (Radxa Zero 3W / Pi Zero footprint): 65 x 30 x 1.6, components on +Y.
            part("Compute", coll, PCB_GREEN, box_mesh("Compute", (-32.5, -0.8, -15), (32.5, 0.8, 15)), I, world, bevel_w=0.4)
            part("Compute.soc", coll, CHIP, box_mesh("soc", (-8, 0.8, -6), (6, 2.0, 6)), I, world, bevel_w=0.3)
            part("Compute.ram", coll, CHIP, box_mesh("ram", (9, 0.8, -5), (19, 1.8, 5)), I, world, bevel_w=0.3)
            part("Compute.usbc", coll, SILVER, box_mesh("usbc", (20, 0.8, -15.5), (29, 4.0, -11.5)), I, world, bevel_w=0.6)
            part("Compute.wifi", coll, SILVER, box_mesh("wifi", (-28, 0.8, 2), (-18, 2.2, 12)), I, world, bevel_w=0.3)
            for k in range(20):
                for row in (0, 1):
                    part(f"Compute.pin{k}{row}", coll, GOLD, cyl_mesh("pin", 0.5, 0, 0.3, 8), T((-24.13 + k * 2.54, 0.8, 11.5 + row * 2.54)) @ axis_matrix("Y"), world)
            for hx in (-29, 29):
                for hz in (-11.5, 11.5):
                    part(f"Compute.hole{hx}{hz}", coll, SILVER, cyl_mesh("hole", 3.0, 0, 0.1, 20, r_in=1.35), T((hx, 0.8, hz)) @ axis_matrix("Y"), world)
            made += 1
        elif mesh == "elec_rpi_robot_hat_pcb":
            # Robot hat (servo bus, power): 65 x 30 x 0.8, parts on +Z.
            part("RobotHat", coll, PCB_BLUE, box_mesh("RobotHat", (-3.5, -26.5, 0), (61.5, 3.5, 0.8)), I, world, bevel_w=0.3)
            for k in range(4):
                part(f"RobotHat.bus{k}", coll, JST, box_mesh("bus", (2 + k * 9, -24, 0.8), (9 + k * 9, -18, 5.5)), I, world, bevel_w=0.3)
            part("RobotHat.cap", coll, SILVER, cyl_mesh("cap", 3.2, 0.8, 8.0, 24), T((48, -10, 0)), world, smooth=True)
            part("RobotHat.reg", coll, CHIP, box_mesh("reg", (38, -24, 0.8), (46, -16, 2.6)), I, world, bevel_w=0.3)
            made += 1
        elif mesh == "np_f970":
            # 2S Li-ion pack (NP-F style): 38.6 x 20.6 x 70.8, a lime label band, terminals at -Z.
            part("Battery", coll, BATTERY, box_mesh("Battery", (-19.3, -10.3, -35.4), (19.3, 10.3, 35.4)), I, world, bevel_w=2.0)
            part("Battery.label", coll, LABEL, box_mesh("label", (-19.5, -10.5, -6), (19.5, 10.5, 10)), I, world, bevel_w=0.8)
            for k, x in enumerate((-8, 0, 8)):
                part(f"Battery.term{k}", coll, GOLD, box_mesh("term", (x - 2, -3, -35.8), (x + 2, 3, -35.4)), I, world)
            made += 1
        elif mesh == "speaker":
            part("Speaker", coll, CHIP, box_mesh("Speaker", (0, -25, 0), (35, 0, 7)), I, world, bevel_w=2.0)
            for gx in range(5):
                for gy in range(3):
                    part(f"Speaker.g{gx}{gy}", coll, SILVER, cyl_mesh("g", 0.7, 7, 7.2, 10), T((8 + gx * 4.5, -18 + gy * 5.5, 0)), world)
            made += 1
        elif mesh == "m12_lens_holder":
            part("CameraHolder", coll, CHIP, box_mesh("CameraHolder", (-12, 0, -8), (12, 14.8, 8)), I, world, bevel_w=1.0)
            made += 1
        elif mesh == "lens":
            # M12 lens barrel along local Y, the glass at its front.
            part("CameraLens", coll, CHIP, cyl_mesh("CameraLens", 8.4, 7.4, 24.0, 40), axis_matrix("Y"), world, smooth=True)
            part("CameraGlass", coll, GLASS, cyl_mesh("CameraGlass", 6.0, 24.0, 26.3, 40), axis_matrix("Y"), world, smooth=True)
            made += 1
    return made


LIME = mat("Lime", "#C6F000", 0.45)
CREAM = mat("Cream", "#ECE5D6", 0.5)
INK = mat("Ink", "#161616", 0.4)
EYE = mat("EyeGloss", "#0A0A0B", 0.04, coat=1.0)
SHINE = mat("Shine", "#FFFFFF", 0.15)
LED_RED = mat("LED", "#FF2A1A", 0.3)
RUBBER = mat("Rubber", "#141414", 0.85)
FANG = mat("Fang", "#F7F4EE", 0.3)


def loft_skin(name, coll, material, sections, subsurf=2, keep=0.3, yc=0.0):
    """A smooth shell through rectangular sections (x, half width, z bottom, z top) along X, bevelled
    then subdivided so it keeps its size: a surfaced form, not a box."""
    v, f = [], []
    for x, w, z0, z1 in sections:
        v += [(x, yc - w, z0), (x, yc + w, z0), (x, yc + w, z1), (x, yc - w, z1)]
    n = len(sections)
    f.append((3, 2, 1, 0))
    for i in range(n - 1):
        a, b = 4 * i, 4 * (i + 1)
        for k in range(4):
            f.append((a + k, a + (k + 1) % 4, b + (k + 1) % 4, b + k))
    f.append((4 * (n - 1), 4 * (n - 1) + 1, 4 * (n - 1) + 2, 4 * (n - 1) + 3))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    pc_tag(me, pc_loft([v[4 * i:4 * i + 4] for i in range(n)], "x"))
    o = obj_from(name, me, coll, material, smooth=True)
    if keep:
        b = o.modifiers.new("Hold", "BEVEL")
        b.width = min(min(2 * w, z1 - z0) for _, w, z0, z1 in sections) * keep
        b.segments = 2
    sub = o.modifiers.new("Subsurf", "SUBSURF")
    sub.levels = sub.render_levels = subsurf
    return o


def ball(name, coll, material, r, at, scale=(1, 1, 1)):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=r, location=at)
    o = bpy.context.active_object
    o.name = name
    for c in o.users_collection:
        c.objects.unlink(o)
    coll.objects.link(o)
    o.scale = scale
    o.data.materials.append(material)
    o.data.shade_smooth()
    pc_tag(o.data, {"sphere": {"center": [0.0, 0.0, 0.0], "radius": float(r)}})
    return o


def pc_cone(r1, r2, depth, n):
    """A primitive cone's recipe (along local Z, centred): a loft through its two rims."""
    ring = lambda r: [[r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)] for k in range(n)]
    return {"loft": {"sections": [{"origin": [0.0, 0.0, -depth / 2], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "points": ring(r1)},
                                  {"origin": [0.0, 0.0, depth / 2], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "points": ring(r2)}]}}


def disc(name, coll, material, r, h, at, axis, n=40, bevel_w=0.0):
    o = part(name, coll, material, cyl_mesh(name, r, 0, h, n), Matrix.Translation(at) @ axis_matrix(axis), Matrix.Identity(4), smooth=True)
    if bevel_w:
        bevel(o, bevel_w)
    return o


def rounded(name, coll, material, lo, hi, w=2.0):
    return part(name, coll, material, box_mesh(name, lo, hi), Matrix.Identity(4), Matrix.Identity(4), bevel_w=min(w, min(b - a for a, b in zip(lo, hi)) / 2 - 0.05))


def screw_dot(name, coll, at, axis):
    disc(name, coll, INK, 1.7, 1.2, at, axis, n=20, bevel_w=0.5)


def build_skins():
    coll = collection("skins")
    T = Matrix.Translation
    # Head: a cute lime dino head over Microduck's head frame (x -30..92, z 205..264), the camera
    # lens (front at x 90, z 237) as its nose camera.
    loft_skin("Head", coll, LIME, [(-32, 34, 214, 250), (-14, 47, 208, 270), (28, 49, 207, 274), (60, 39, 210, 264), (82, 27, 214, 254), (96, 17, 220, 246)], subsurf=3, keep=0.32)
    for sy, side in ((1, "L"), (-1, "R")):
        eye = Vector((48, sy * 39, 248))
        toe = Matrix.Rotation(sy * math.radians(18), 4, "Z")
        for n, r, off, sc in ((f"EyeSocket_{side}", 18.0, (0, 0, 0), (1, 0.55, 1)), (f"Eye_{side}", 15.5, (1.5, sy * 4.2, 0), (1, 0.72, 1))):
            o = ball(n, coll, INK if "Socket" in n else EYE, r, eye + Vector(off), sc)
            o.rotation_euler = toe.to_euler()
        ball(f"EyeShine_{side}", coll, SHINE, 3.8, eye + Vector((9, sy * 13.2, 6.5)))
        ball(f"EyeShine2_{side}", coll, SHINE, 1.7, eye + Vector((-2, sy * 14.4, -6)))
        brow = loft_skin(f"Brow_{side}", coll, LIME, [(34, 3, 266, 271), (48, 3.5, 268, 274), (62, 3, 265, 270)], subsurf=2, keep=0.35, yc=sy * 41)
        ball(f"Blush_{side}", coll, mat("Blush", "#FF9E8C", 0.6), 6.5, Vector((70, sy * 33.5, 230)), (1.2, 0.25, 0.8)).rotation_euler = (0, 0, sy * math.radians(35))
        for k, (x, z) in enumerate(((0, 236), (74, 232))):
            screw_dot(f"HeadScrew_{side}{k}", coll, (x, sy * (45 if k == 0 else 34.5), z), "Y" if sy > 0 else "-Y")
    # Nose: a rounded black camera module round the real lens (barrel 16.9 mm, its glass at x 90),
    # the red status LED beside it.
    rounded("CameraModule", coll, INK, (88, -12, 226), (95.5, 12, 248), 4.5)
    disc("CameraWindow", coll, GLASS, 7.4, 1.2, (95.0, 0, 237.5), "X", bevel_w=0.4)
    disc("NoseLED", coll, LED_RED, 1.8, 1.6, (93.5, -17.5, 233), "X")
    # Speaker grille over the real speaker (x -18..7, z 232..239, inside the crown's back).
    for k in range(5):
        rounded(f"Grille{k}", coll, INK, (-18 + k * 5.5, -12, 268), (-15.5 + k * 5.5, 12, 271.5), 1.0)
    rounded("CrownSlot", coll, INK, (14, -9, 271.5), (34, 9, 276), 2)
    # Jaw: cream on Microduck's mouth joint (x 17..86, z 205..234), fangs in front of its lip.
    # Jaw: cream, hinged on the jaw servo's axis (XL330_09: along Y through x 26, z 225.6), its arms
    # running up inside the cheeks to the hinge; the cheek caps sit on that axis.
    HINGE = (26.1, 225.6)
    loft_skin("Jaw", coll, CREAM, [(14, 33, 190, 207), (44, 36, 186, 208), (70, 28, 189, 208), (88, 17, 195, 207)], subsurf=3, keep=0.32)
    rounded("Mouth", coll, INK, (24, -26, 206), (84, 26, 210), 1.5)
    rounded("Tongue", coll, mat("Tongue", "#FF7A8A", 0.6), (42, -12, 205.5), (74, 12, 208.2), 1.3)
    for sy, side in ((1, "L"), (-1, "R")):
        rounded(f"JawArm_{side}", coll, CREAM, (16, sy * 36 - 3, 196), (36, sy * 36 + 3, HINGE[1] + 6), 2.5)
        disc(f"CheekCap_{side}", coll, CREAM, 10, 4, (HINGE[0], sy * 44, HINGE[1]), "Y" if sy > 0 else "-Y", bevel_w=1.8)
        disc(f"CheekHorn_{side}", coll, HORN, 6, 1.4, (HINGE[0], sy * 48, HINGE[1]), "Y" if sy > 0 else "-Y")
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            screw_dot(f"CheekScrew_{side}{k}", coll, (HINGE[0] + 3.6 * math.cos(a), sy * 49.2, HINGE[1] + 3.6 * math.sin(a)), "Y" if sy > 0 else "-Y")
        disc(f"Fang_{side}", coll, FANG, 2.8, 7, (88, sy * 10, 214), "-Z", n=24, bevel_w=0.5)
        ball(f"FangTip_{side}", coll, FANG, 2.8, (88, sy * 10, 207))
    # Neck: cream collars between the exposed servos.
    for k, (z0, z1) in enumerate(((148, 160), (186, 196))):
        loft_skin(f"NeckCollar{k}", coll, CREAM, [(8, 18, z0, z1), (26, 20, z0, z1 + 2), (44, 17, z0, z1)], subsurf=2, keep=0.35)
    # Body: a rounded cream pear around the trunk (x -46..35, z 107..149) and the battery hanging
    # behind (x -47..-25, down to z 67): the hips of a dinosaur.
    loft_skin("Body", coll, CREAM, [(-70, 30, 58, 130), (-50, 40, 50, 152), (-20, 42, 56, 158), (2, 42, 92, 158), (28, 38, 102, 156), (48, 26, 112, 148)], subsurf=3, keep=0.3)
    # The battery (x -47..-25, z 67..138) swaps through a door in the back; USB-C charging and a
    # lime power switch beside it, under the tail.
    rounded("BatteryDoor", coll, mat("Seam", "#BDB6A8", 0.6), (-73.5, -20, 64), (-71.5, 20, 104), 2.0)
    rounded("ChargePort", coll, INK, (-74.5, -14, 108), (-72, -5, 112), 1.0)
    rounded("PowerSwitch", coll, LIME, (-75, 6, 107), (-71.5, 13, 113), 1.4)
    for k, (y, z) in enumerate(((-16, 68), (16, 68), (-16, 100), (16, 100))):
        screw_dot(f"DoorScrew{k}", coll, (-73.8, y, z), "-X")
    for sy, side in ((1, "L"), (-1, "R")):
        loft_skin(f"Shoulder_{side}", coll, LIME, [(4, 5, 118, 150), (24, 7, 112, 154), (44, 5, 118, 146)], subsurf=2, keep=0.3, yc=sy * 39)
        for k, x in enumerate((12, 36)):
            screw_dot(f"BodyScrew_{side}{k}", coll, (x, sy * 46, 134), "Y" if sy > 0 else "-Y")
        loft_skin(f"HipPlate_{side}", coll, LIME, [(-58, 4, 92, 140), (-34, 6, 84, 146), (-12, 4, 96, 138)], subsurf=2, keep=0.3, yc=sy * 41)
    # Tail: a new tail-yaw XL330 behind the body, then a tapered tail with black rings and a lime tip.
    # Tail: its yaw XL330 inside the hips, then a short chubby tail curling up, three rounded rings,
    # lime studs along the top like little scales, and a round lime tip.
    xl330("XL330_TailYaw", coll, T((-58, 0, 112)) @ axis_matrix("Z") @ Matrix.Rotation(math.pi / 2, 4, "Y"))
    tail = [(-66, 30, 0), (-96, 27, 3), (-124, 22, 11), (-148, 17, 24), (-166, 13, 39), (-176, 10, 54)]
    loft_skin("Tail", coll, CREAM, [(x, r * 0.95, 112 - r + dz, 112 + r * 0.9 + dz) for x, r, dz in tail], subsurf=3, keep=0.32)
    for k, (x, r, dz) in enumerate(tail[1:4], 1):
        disc(f"TailRing{k}", coll, INK, r + 2.4, 6, (x - 3, 0, 112 - r * 0.05 + dz), "X", bevel_w=2.4)
    for k, (x, r, dz) in enumerate(tail[:5]):
        ball(f"TailStud{k}", coll, LIME, max(3.2, r * 0.26), Vector((x - 14, 0, 112 + r * 0.9 + dz + 1)), (1.3, 1, 0.65))
    ball("TailTip", coll, LIME, 12, Vector((-182, 0, 112 + 62)), (1.15, 1, 1))
    # Arms: a shoulder XL330 and an elbow XL330 each side, a cream forearm and a two-finger gripper.
    for sy, side in ((1, "L"), (-1, "R")):
        xl330(f"XL330_Shoulder_{side}", coll, T((50, sy * 30, 120)) @ Matrix.Rotation(sy * math.pi / 2, 4, "Z"))
        rounded(f"UpperArm_{side}", coll, CREAM, (52, sy * 40 - 6, 92), (64, sy * 40 + 6, 118), 4)
        xl330(f"XL330_Elbow_{side}", coll, T((62, sy * 40, 90)) @ Matrix.Rotation(sy * math.pi / 2, 4, "Z") @ Matrix.Scale(0.8, 4))
        loft_skin(f"Forearm_{side}", coll, CREAM, [(66, 5.5, 82, 94), (84, 5, 84, 95), (92, 4.5, 85, 94)], subsurf=2, keep=0.35, yc=sy * 40)
        rounded(f"ForearmPlate_{side}", coll, LIME, (70, sy * 40 + (5 if sy > 0 else -8), 85), (84, sy * 40 + (8 if sy > 0 else -5), 93), 1.2)
        for t, dz in ((1, 4), (-1, -4)):
            loft_skin(f"Finger_{side}{t}", coll, CREAM, [(91, 3, 89 + dz - 3, 89 + dz + 3), (101, 2.6, 89 + dz * 1.6 - 2.6, 89 + dz * 1.6 + 2.6), (106, 1.8, 89 + dz * 1.4 - 1.8, 89 + dz * 1.4 + 1.8)], subsurf=2, keep=0.35, yc=sy * 40)
    # Legs: lime drumstick plates on the thighs (y 42..71, z 55..102), cream shins, feet with toes.
    for sy, side in ((1, "L"), (-1, "R")):
        yo = sy * 74
        loft_skin(f"ThighPlate_{side}", coll, LIME, [(-50, 3.5, 60, 96), (-20, 5.5, 50, 106), (12, 3.5, 60, 98)], subsurf=2, keep=0.35, yc=yo)
        for k, (x, z) in enumerate(((-34, 92), (-2, 66))):
            screw_dot(f"ThighScrew_{side}{k}", coll, (x, sy * 79.6, z), "Y" if sy > 0 else "-Y")
        loft_skin(f"Shin_{side}", coll, CREAM, [(-48, 8, 34, 58), (-32, 9, 32, 62), (-16, 8, 34, 58)], subsurf=2, keep=0.35, yc=sy * 50)
        yf = sy * 50
        loft_skin(f"Foot_{side}", coll, CREAM, [(-56, 22, 6, 20), (-30, 24, 6, 24), (0, 23, 6, 20)], subsurf=2, keep=0.3, yc=yf)
        rounded(f"Sole_{side}", coll, RUBBER, (-58, yf - 24, 0), (4, yf + 24, 6.5), 2.5)
        for t in (-1, 0, 1):
            yt = yf + t * 15
            loft_skin(f"Toe_{side}{t}", coll, CREAM, [(-4, 7.5, 5, 18), (14, 7, 5, 16)], subsurf=2, keep=0.35, yc=yt)
            loft_skin(f"ToeCap_{side}{t}", coll, LIME, [(12, 7.8, 4.5, 16.5), (24, 6.5, 4.5, 12), (29, 3.5, 4.5, 8)], subsurf=2, keep=0.3, yc=yt)
            rounded(f"ToeSole_{side}{t}", coll, RUBBER, (-4, yt - 7.5, 0), (28, yt + 7.5, 5), 2)
    return len(coll.objects)


def build_cute_shells():
    """v3: cute rounded shells on Microduck's frame - a bean-shaped head with a face (two big eyes,
    the camera as the nose, a smile on the jaw servo's hinge), an egg body round the battery, a
    bubble tail curling up, stubby arms and rounded drumstick legs."""
    coll = collection("skins")
    T = Matrix.Translation
    HINGE = (26.1, 225.6)  # the jaw servo's axis (XL330_09), along Y
    # Head: a rounded bean over Microduck's head frame (x -30..92, z 205..264).
    loft_skin("Head", coll, LIME, [(-34, 30, 214, 248), (-18, 45, 208, 266), (20, 49, 206, 272), (58, 45, 207, 268), (82, 34, 211, 258), (95, 22, 216, 248)], subsurf=3, keep=0.42)
    # Face: two big glossy eyes either side, a catch-light each, lime lids; the camera as the nose.
    for sy, side in ((1, "L"), (-1, "R")):
        eye = Vector((80, sy * 22.5, 250))
        ball(f"EyeRim_{side}", coll, INK, 15.0, eye, (0.55, 1, 1))
        ball(f"Eye_{side}", coll, EYE, 13.2, eye + Vector((2.6, 0, 0)), (0.6, 1, 1))
        ball(f"EyeShine_{side}", coll, SHINE, 3.9, eye + Vector((9.6, -sy * 4.2, 5.2)))
        ball(f"EyeShine2_{side}", coll, SHINE, 1.8, eye + Vector((9.4, sy * 4.6, -4.6)))
        ball(f"Blush_{side}", coll, mat("Blush", "#FF9E8C", 0.6), 5.0, Vector((88.5, sy * 25, 233)), (0.22, 1.1, 0.7)).rotation_euler = (0, 0, sy * math.radians(-30))
    rounded("CameraNose", coll, INK, (88, -10, 229), (96, 10, 245), 5.0)
    disc("CameraWindow", coll, GLASS, 6.4, 1.2, (95.6, 0, 237.5), "X", bevel_w=0.4)
    disc("NoseLED", coll, LED_RED, 1.5, 1.4, (94.6, -14.5, 232), "X")
    for k in range(4):  # speaker grille on the crown's back, over the real speaker
        rounded(f"Grille{k}", coll, INK, (-20 + k * 6.5, -11, 269), (-17 + k * 6.5, 11, 272), 1.2)
    for k, (x, z, r) in enumerate(((44, 272, 7.5), (22, 273, 8.5), (0, 270, 8.0), (-20, 262, 7.0), (-34, 246, 6.0))):  # crest bumps
        ball(f"Crest{k}", coll, LIME, r, Vector((x, 0, z)), (1.1, 0.75, 0.75))
    # Jaw: cream, on the jaw servo's hinge, a dark smile and a tongue, two little fangs.
    loft_skin("Jaw", coll, CREAM, [(12, 34, 192, 208), (42, 38, 188, 209), (68, 31, 190, 209), (86, 19, 196, 208)], subsurf=3, keep=0.4)
    rounded("Mouth", coll, INK, (30, -25, 206.5), (84, 25, 210.5), 1.8)
    rounded("Tongue", coll, mat("Tongue", "#FF7A8A", 0.6), (44, -11, 206), (72, 11, 209), 1.4)
    for sy, side in ((1, "L"), (-1, "R")):
        disc(f"CheekCap_{side}", coll, CREAM, 10, 4, (HINGE[0], sy * 46, HINGE[1]), "Y" if sy > 0 else "-Y", bevel_w=2.0)
        disc(f"CheekHorn_{side}", coll, HORN, 6, 1.4, (HINGE[0], sy * 50, HINGE[1]), "Y" if sy > 0 else "-Y")
        rounded(f"JawArm_{side}", coll, CREAM, (18, sy * 38 - 3, 196), (34, sy * 38 + 3, HINGE[1] + 4), 2.5)
        disc(f"Fang_{side}", coll, FANG, 2.6, 6, (84, sy * 9, 213), "-Z", n=24, bevel_w=0.5)
        ball(f"FangTip_{side}", coll, FANG, 2.6, (84, sy * 9, 207))
    # Neck: cream collars between the exposed servos.
    for k, (z0, z1) in enumerate(((150, 161), (186, 197))):
        loft_skin(f"NeckCollar{k}", coll, CREAM, [(8, 18, z0, z1), (26, 21, z0, z1 + 2), (44, 18, z0, z1)], subsurf=2, keep=0.4)
    # Body: a rounded egg round the trunk and the battery (x -47..-25, z 67..138), a lime backpack.
    loft_skin("Body", coll, CREAM, [(-74, 24, 66, 126), (-54, 40, 52, 152), (-20, 44, 55, 162), (4, 43, 88, 162), (30, 37, 104, 157), (50, 22, 116, 146)], subsurf=3, keep=0.49)
    loft_skin("Backpack", coll, LIME, [(-56, 26, 140, 152), (-30, 30, 150, 163), (-4, 27, 152, 162)], subsurf=2, keep=0.4)
    for k, x in enumerate((-46, -14)):
        for sy in (1, -1):
            screw_dot(f"PackScrew{k}{sy}", coll, (x, sy * 18, 161 if k else 157), "Z")
    rounded("BatteryDoor", coll, mat("Seam", "#BDB6A8", 0.6), (-75.5, -18, 70), (-73.5, 18, 104), 2.0)
    rounded("ChargePort", coll, INK, (-76, -13, 108), (-73.5, -5, 112), 1.0)
    rounded("PowerSwitch", coll, LIME, (-76.5, 5, 107), (-73, 12, 113), 1.4)
    # Tail: its yaw servo in the hips, then rounded beads curling up to a lime ball.
    xl330("XL330_TailYaw", coll, T((-60, 0, 112)) @ axis_matrix("Z") @ Matrix.Rotation(math.pi / 2, 4, "Y"))
    beads = [(-80, 112, 25), (-104, 117, 21), (-124, 126, 17), (-139, 138, 13.5), (-149, 152, 10.5)]
    for k, (x, z, r) in enumerate(beads):
        ball(f"TailBead{k}", coll, CREAM, r, Vector((x, 0, z)), (1.15, 1, 0.95))
        if k < len(beads) - 1:
            nx, nz, nr = beads[k + 1]
            o = disc(f"TailJoint{k}", coll, INK, (r + nr) / 2 * 0.78, 5, ((x + nx) / 2 - 2.5, 0, (z + nz) / 2), "X", bevel_w=1.6)
            o.rotation_euler = Euler((0, math.pi / 2 - math.atan2(nz - z, x - nx), 0))
    ball("TailTip", coll, LIME, 10, Vector((-155, 0, 166)), (1, 1, 1.05))
    # Arms: a servo in the chest each side turns a stubby cream arm with a lime mitten hand.
    for sy, side in ((1, "L"), (-1, "R")):
        loft_skin(f"Arm_{side}", coll, CREAM, [(40, 6, 116, 132), (52, 6.5, 106, 122), (60, 6, 100, 114)], subsurf=2, keep=0.45, yc=sy * 45)
        disc(f"ArmHorn_{side}", coll, HORN, 6, 1.6, (40, sy * 51.5, 125), "Y" if sy > 0 else "-Y")
        ball(f"Hand_{side}", coll, LIME, 7.5, Vector((63, sy * 45, 103)), (1.1, 0.9, 0.95))
        for t in (-1, 1):
            ball(f"Finger_{side}{t}", coll, LIME, 3.2, Vector((69, sy * 45 + t * 3.8, 99)), (1.3, 1, 1))
    # Legs: rounded lime drumsticks over the thigh brackets, cream shins, rounded feet and toes.
    for sy, side in ((1, "L"), (-1, "R")):
        # A lime drumstick enclosing the thigh bracket (x -45..16, y 42..71, z 55..102).
        loft_skin(f"Thigh_{side}", coll, LIME, [(-56, 17, 54, 104), (-34, 23, 45, 113), (0, 23, 46, 113), (24, 16, 56, 102)], subsurf=3, keep=0.4, yc=sy * 57)
        for k, (x, z) in enumerate(((-30, 96), (-2, 66))):
            screw_dot(f"ThighScrew_{side}{k}", coll, (x, sy * 75.6, z), "Y" if sy > 0 else "-Y")
        ball(f"Shin_{side}", coll, CREAM, 1.0, Vector((-30, sy * 52, 46)), (15, 13, 19))
        yf = sy * 50
        loft_skin(f"Foot_{side}", coll, CREAM, [(-58, 22, 3, 18), (-30, 25, 3, 23), (2, 24, 3, 19)], subsurf=3, keep=0.4, yc=yf)
        for t in (-1, 0, 1):
            ball(f"Toe_{side}{t}", coll, LIME, 1.0, Vector((10, yf + t * 15, 9)), (11, 7, 7))
        rounded(f"Sole_{side}", coll, RUBBER, (-60, yf - 24, 0), (16, yf + 24, 4), 2.0)
    return len(coll.objects)


def fit_report():
    """Which real parts sit inside the skins, which poke through, and where every servo is: points
    (box corners and centre, after modifiers) tested by ray parity against each skin."""
    from mathutils.bvhtree import BVHTree
    deps = bpy.context.evaluated_depsgraph_get()
    skins = []
    for o in bpy.data.collections[f"{SCENE}: skins"].objects:
        if o.type != "MESH" or o.name.startswith(("XL330", "Eye", "Fang", "Screw", "Toe", "Sole", "Nose", "Crown", "Mouth", "Cheek", "Tail Ring")):
            continue
        ev = o.evaluated_get(deps)
        me = ev.to_mesh()
        verts = [o.matrix_world @ v.co for v in me.vertices]
        tree = BVHTree.FromPolygons(verts, [tuple(p.vertices) for p in me.polygons])
        ev.to_mesh_clear()
        skins.append((o.name, tree))

    def inside(p):
        names = []
        for name, tree in skins:
            hits, origin, d = 0, p.copy(), Vector((0.31, 0.47, 0.83)).normalized()
            for _ in range(64):
                loc, _n, _i, _dist = tree.ray_cast(origin, d)
                if loc is None:
                    break
                hits += 1
                origin = loc + d * 1e-3
            if hits % 2:
                names.append(name)
        return names

    report = {}
    groups = {"Battery": ["Battery"], "Compute": ["Compute"], "Camera": ["CameraHolder", "CameraLens"], "Speaker": ["Speaker"], "RobotHat": ["RobotHat"]}
    for label, names in groups.items():
        objs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
        pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
        pts.append(sum(pts, Vector()) / len(pts))
        ins = [inside(p) for p in pts]
        report[label] = {"inside": sum(1 for i in ins if i), "of": len(pts), "in": sorted({n for i in ins for n in i})}
    servos = []
    for o in bpy.data.objects:
        if o.name.startswith("XL330") and "." not in o.name and o.users_scene and o.users_scene[0].name == SCENE:
            c = o.matrix_world.translation
            servos.append((o.name, [round(v) for v in c], len(inside(o.matrix_world @ Vector((0, 0, -7.5)))) > 0))
    report["servos"] = sorted(servos, key=lambda s: -s[1][2])
    return report


def show():
    for area in bpy.context.window.screen.areas:
        if area.type != "VIEW_3D":
            continue
        space = area.spaces.active
        space.shading.type = "MATERIAL"
        space.clip_start, space.clip_end = 1, 100000
        region = next(r for r in area.regions if r.type == "WINDOW")
        with bpy.context.temp_override(area=area, region=region):
            bpy.ops.object.select_all(action="SELECT")
            bpy.ops.view3d.view_selected()
            bpy.ops.object.select_all(action="DESELECT")


if PHASE == "skeleton":
    print("skeleton", build_skeleton())
elif PHASE == "electronics":
    print("electronics", build_electronics())
elif PHASE == "skins":
    print("skins", build_skins())
elif PHASE == "cute":
    print("cute shells", build_cute_shells())
elif PHASE == "fit":
    import json
    print(json.dumps(fit_report(), indent=1))
show()
