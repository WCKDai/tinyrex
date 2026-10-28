"""TinyREX v1 in Blender: a Microduck-sized biped as a cute robot dinosaur.

Run inside Blender (MCP execute, or headless):
    Blender --background --factory-startup --python tinyrex_build.py -- OUT_DIR

Units are millimetres (scene unit scale 0.001). X forward, Y left, Z up, ground at z = 0.
Frame of reference: Microduck (Pollen Robotics) - about 25 cm tall, hips about 12 cm up,
5-joint legs, Dynamixel X330 servos (34 x 20 x 26 mm). Look: the TinyREX reference boards -
cream shells, lime head, thigh plates, toe caps and tail tip, black servos shown at the joints.
"""
import math
import os
import sys

import bpy
from mathutils import Vector

LIVE = globals().get("LIVE", False)  # set when sent to an open Blender through the MCP bridge
OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else ""

# ---------------------------------------------------------------- scene
if LIVE:
    # Build into its own scene in the open Blender, leaving the user's scenes alone; a rebuild
    # clears only this scene.
    scene = bpy.data.scenes.get("TinyREX") or bpy.data.scenes.new("TinyREX")
    for o in list(scene.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.context.window.scene = scene
else:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 0.001
scene.unit_settings.length_unit = "MILLIMETERS"


def material(name, rgb, rough=0.45, metal=0.0, coat=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1.0)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if "Coat Weight" in p.inputs:
        p.inputs["Coat Weight"].default_value = coat
    return m


def srgb(h):
    h = h.lstrip("#")
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c)


LIME = material("Lime", srgb("#C6F000"), 0.5)
CREAM = material("Cream", srgb("#ECE5D6"), 0.55)
BLACK = material("Black", srgb("#1A1B1D"), 0.4)
FRAME = material("Frame", srgb("#2C2E31"), 0.5)
ALU = material("Aluminium", srgb("#B8BCC1"), 0.3, metal=1.0)
GLOSS = material("EyeGloss", srgb("#0B0B0C"), 0.05, coat=1.0)
WHITE = material("White", srgb("#FFFFFF"), 0.2)
RED = material("Red", srgb("#FF2A1A"), 0.3)
LENS = material("Lens", srgb("#2B3B4D"), 0.05, coat=1.0)
RUBBER = material("Rubber", srgb("#141414"), 0.8)

ROOT = bpy.data.objects.new("TinyREX", None)
scene.collection.objects.link(ROOT)


def finish(obj, mat, parent=None, smooth=True):
    obj.data.materials.append(mat)
    obj.parent = parent or ROOT
    if smooth:
        for poly in obj.data.polygons:
            poly.use_smooth = True
    return obj


def mods(obj, bevel=0.0, segments=3, subsurf=0):
    if bevel > 0:
        b = obj.modifiers.new("Bevel", "BEVEL")
        b.width = bevel
        b.segments = segments
        b.limit_method = "ANGLE"
        b.harden_normals = subsurf == 0
    if subsurf:
        s = obj.modifiers.new("Subsurf", "SUBSURF")
        s.levels = subsurf
        s.render_levels = subsurf
    if bevel > 0 and subsurf == 0:
        obj.data.shade_smooth()
        try:
            obj.data.set_sharp_from_angle(angle=math.radians(35))
        except AttributeError:
            pass
    return obj


def mesh_obj(name, verts, faces):
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) for v in verts], [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    scene.collection.objects.link(o)
    return o


def rbox(name, size, loc, mat, bevel=3.0, rot=(0, 0, 0), parent=None, segments=3):
    """A rounded box: size (x, y, z), centred at loc."""
    sx, sy, sz = (s / 2 for s in size)
    v = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz), (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    o = mesh_obj(name, v, f)
    o.location = loc
    o.rotation_euler = [math.radians(a) for a in rot]
    mods(o, bevel=min(bevel, min(size) / 2 - 0.01), segments=segments)
    return finish(o, mat, parent, smooth=False)


def blob(name, back, front, mat, subsurf=2, crease=0.0, parent=None):
    """A soft shell between two rounded rectangular ends: back/front = (x, half width, z bottom,
    z top), smoothed by subdivision, so it reads as a surfaced form, not a box."""
    (xb, wb, zb0, zb1), (xf, wf, zf0, zf1) = back, front
    v = [(xb, -wb, zb0), (xf, -wf, zf0), (xf, wf, zf0), (xb, wb, zb0), (xb, -wb, zb1), (xf, -wf, zf1), (xf, wf, zf1), (xb, wb, zb1)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    o = mesh_obj(name, v, f)
    if crease:
        o.data.edges.foreach_set("crease", [crease] * len(o.data.edges)) if hasattr(o.data.edges[0], "crease") else None
    mods(o, subsurf=subsurf)
    return finish(o, mat, parent)


def loft_blob(name, sections, mat, subsurf=2, parent=None, keep=0.28):
    """A smooth shell through several rectangular sections (x, half width, z bottom, z top) along X,
    subdivided into a rounded surface: crown, cheeks and snout read as one form."""
    v, f = [], []
    for x, w, z0, z1 in sections:
        v += [(x, -w, z0), (x, w, z0), (x, w, z1), (x, -w, z1)]
    n = len(sections)
    f.append((3, 2, 1, 0))
    for i in range(n - 1):
        a, b = 4 * i, 4 * (i + 1)
        for k in range(4):
            f.append((a + k, a + (k + 1) % 4, b + (k + 1) % 4, b + k))
    f.append((4 * (n - 1), 4 * (n - 1) + 1, 4 * (n - 1) + 2, 4 * (n - 1) + 3))
    o = mesh_obj(name, v, f)
    if keep:
        smallest = min(min(2 * w, z1 - z0) for _, w, z0, z1 in sections)
        b = o.modifiers.new("Hold", "BEVEL")
        b.width = smallest * keep
        b.segments = 2
    mods(o, subsurf=subsurf)
    return finish(o, mat, parent)


def cyl(name, r, depth, loc, axis="Y", mat=BLACK, parent=None, verts=40, bevel=0.6):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=depth, location=loc)
    o = bpy.context.active_object
    o.name = name
    o.rotation_euler = {"X": (0, math.pi / 2, 0), "Y": (math.pi / 2, 0, 0), "Z": (0, 0, 0)}[axis]
    mods(o, bevel=min(bevel, r / 2, depth / 2 - 0.01))
    return finish(o, mat, parent, smooth=False)


def sphere(name, r, loc, mat, parent=None, scale=(1, 1, 1)):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=r, location=loc)
    o = bpy.context.active_object
    o.name = name
    o.scale = scale
    return finish(o, mat, parent)


def screw(name, loc, axis, parent=None):
    return cyl(name, 1.6, 1.4, loc, axis=axis, mat=BLACK, parent=parent, verts=20, bevel=0.5)


def servo(name, centre, horn_axis="Y", horn_side=1, size=(34, 20, 26), parent=None):
    """A Dynamixel X330: a black 34 x 20 x 26 case, its aluminium horn and the horn's screws.
    `size` is (along x, along y, along z) as placed; the horn sits on the face along horn_axis."""
    o = rbox(name, size, centre, BLACK, bevel=2.0, parent=parent)
    ax = "XYZ".index(horn_axis)
    p = list(centre)
    p[ax] += horn_side * (size[ax] / 2 + 1.0)
    cyl(f"{name}_horn", 6.5, 2.0, tuple(p), axis=horn_axis, mat=ALU, parent=parent)
    for k, (du, dv) in enumerate(((3.4, 0), (-3.4, 0), (0, 3.4), (0, -3.4))):
        q = list(p)
        q[ax] += horn_side * 1.2
        others = [i for i in range(3) if i != ax]
        q[others[0]] += du
        q[others[1]] += dv
        cyl(f"{name}_hs{k}", 0.8, 0.6, tuple(q), axis=horn_axis, mat=BLACK, parent=parent, verts=12, bevel=0.2)
    return o


# ---------------------------------------------------------------- legs (Microduck frame)
HIP_Z = 122  # hip pitch axis height, about Microduck's pelvis
for s, side in ((1, "L"), (-1, "R")):
    y = s * 40
    # Hip yaw and roll servos under the body, then hip pitch with its horn out.
    servo(f"HipYaw_{side}", (4, s * 26, HIP_Z + 18), "Z", -1, (34, 20, 26))
    servo(f"HipPitch_{side}", (0, y, HIP_Z), "Y", s, (34, 26, 26))
    # Thigh: a cream link with a big lime "drumstick" plate on the outside.
    rbox(f"Thigh_{side}", (24, 14, 46), (4, y, HIP_Z - 26), CREAM, bevel=5)
    loft_blob(f"ThighPlate_{side}", [(-22, 4, HIP_Z - 44, HIP_Z + 8), (0, 6, HIP_Z - 50, HIP_Z + 12), (22, 4, HIP_Z - 40, HIP_Z + 4)], LIME, subsurf=2)
    bpy.data.objects[f"ThighPlate_{side}"].location = (2, y + s * 16, 0)
    for k, (x, z) in enumerate(((-8, HIP_Z - 8), (10, HIP_Z - 30))):
        screw(f"ThighScrew_{side}{k}", (x + 2, y + s * 22.5, z), "Y")
    # Knee and shin: black knee servo, cream shin.
    knee = (10, y, HIP_Z - 52)
    servo(f"Knee_{side}", knee, "Y", s, (30, 22, 26))
    cyl(f"KneeCap_{side}", 9, 4, (knee[0], y + s * 13, knee[2]), axis="Y", mat=CREAM)
    rbox(f"Shin_{side}", (20, 16, 40), (4, y, HIP_Z - 80), CREAM, bevel=5)
    # Ankle servo and the foot: cream block, two toes with lime caps, a black rubber sole.
    ankle_z = 34
    servo(f"Ankle_{side}", (0, y, ankle_z), "Y", s, (30, 22, 24))
    rbox(f"Foot_{side}", (62, 40, 16), (8, y, 14), CREAM, bevel=5)
    rbox(f"Sole_{side}", (64, 42, 5), (8, y, 3), RUBBER, bevel=2)
    for t in (-1, 1):
        rbox(f"Toe_{side}{t}", (22, 17, 14), (44, y + t * 10.5, 12), CREAM, bevel=5)
        loft_blob(f"ToeCap_{side}{t}", [(52, 8.5, 6, 18), (62, 7, 6, 13), (64, 4, 6, 9)], LIME, subsurf=1)
        bpy.data.objects[f"ToeCap_{side}{t}"].location = (0, y + t * 10.5, 0)

# ---------------------------------------------------------------- body
BODY_Z = 150
loft_blob("Torso", [(-50, 26, BODY_Z - 26, BODY_Z + 22), (-20, 38, BODY_Z - 34, BODY_Z + 34), (25, 37, BODY_Z - 32, BODY_Z + 36), (52, 26, BODY_Z - 18, BODY_Z + 30)], CREAM, subsurf=2)
# Dark chest gap with the electronics frame showing, as on the front board.
rbox("ChestFrame", (16, 34, 40), (52, 0, BODY_Z + 4), FRAME, bevel=3)
rbox("ChestPlate", (10, 30, 30), (58, 0, BODY_Z - 6), CREAM, bevel=4)
# Lime shoulder plates over the arm roots.
for s, side in ((1, "L"), (-1, "R")):
    loft_blob(f"Shoulder_{side}", [(20, 4, BODY_Z - 8, BODY_Z + 28), (40, 6, BODY_Z - 12, BODY_Z + 30), (56, 4, BODY_Z - 4, BODY_Z + 20)], LIME, subsurf=2)
    bpy.data.objects[f"Shoulder_{side}"].location = (0, s * 38, 0)
    for k, x in enumerate((28, 46)):
        screw(f"BodyScrew_{side}{k}", (x, s * 41.5, BODY_Z + 12), "Y")

# ---------------------------------------------------------------- arms (articulated, with grippers)
for s, side in ((1, "L"), (-1, "R")):
    servo(f"ArmShoulder_{side}", (58, s * 34, BODY_Z - 14), "Y", s, (24, 18, 22))
    rbox(f"UpperArm_{side}", (26, 12, 12), (72, s * 40, BODY_Z - 24), CREAM, bevel=4, rot=(0, 35, 0))
    servo(f"ArmElbow_{side}", (84, s * 40, BODY_Z - 34), "Y", s, (20, 16, 18))
    rbox(f"Forearm_{side}", (22, 11, 11), (96, s * 40, BODY_Z - 32), CREAM, bevel=4, rot=(0, -20, 0))
    rbox(f"ForearmPlate_{side}", (12, 3, 9), (96, s * 46.5, BODY_Z - 32), LIME, bevel=1.5, rot=(0, -20, 0))
    for t in (-1, 1):
        rbox(f"Finger_{side}{t}", (14, 6, 6), (110, s * 40 + t * 5, BODY_Z - 26 - (4 if t < 0 else 0)), CREAM, bevel=2.8, rot=(0, -50 if t > 0 else -20, 0))

# ---------------------------------------------------------------- neck and head
servo("NeckPitch", (52, 0, BODY_Z + 30), "Y", 1, (24, 30, 26))
loft_blob("NeckCollar", [(34, 20, BODY_Z + 14, BODY_Z + 40), (56, 17, BODY_Z + 30, BODY_Z + 60), (68, 14, BODY_Z + 44, BODY_Z + 70)], CREAM, subsurf=2, keep=0.3)
servo("HeadPitch", (66, 0, BODY_Z + 62), "Y", 1, (24, 30, 26))
HEAD = (58, 0, BODY_Z + 72)  # mouth line, back of the skull
hx, _, hz = HEAD
loft_blob("Skull", [
    (hx, 24, hz + 6, hz + 46),
    (hx + 14, 37, hz, hz + 64),
    (hx + 52, 36, hz, hz + 62),
    (hx + 82, 24, hz + 2, hz + 44),
    (hx + 100, 14, hz + 4, hz + 30),
], LIME, subsurf=3)
# Glossy dome eyes with a white catch-light, set a little forward so they read from the front.
for s, side in ((1, "L"), (-1, "R")):
    eye = (hx + 50, s * 28, hz + 38)
    sphere(f"Eye_{side}", 12.5, eye, GLOSS, scale=(1, 0.9, 1))
    bpy.data.objects[f"Eye_{side}"].rotation_euler = (0, 0, math.radians(s * 22))
    sphere(f"EyeRing_{side}", 14.2, eye, FRAME, scale=(1, 0.7, 1))
    bpy.data.objects[f"EyeRing_{side}"].rotation_euler = (0, 0, math.radians(s * 22))
    sphere(f"Glint_{side}", 2.8, (eye[0] + 6, s * 39.0, eye[2] + 5), WHITE)
    for k, (x, z) in enumerate(((hx + 20, hz + 20), (hx + 78, hz + 18))):
        screw(f"SkullScrew_{side}{k}", (x, s * (36 if k == 0 else 24.5), z), "Y")
# Nose: camera module, lens and the red LED; nostril dots; ToF window on the crown.
rbox("CameraModule", (5, 13, 10), (hx + 99.5, 6, hz + 17), BLACK, bevel=2.2)
cyl("CameraLens", 3.2, 1.4, (hx + 102.6, 6, hz + 17), axis="X", mat=LENS, bevel=0.4)
cyl("LEDBezel", 2.4, 2.0, (hx + 99.5, -8, hz + 17), axis="X", mat=BLACK, bevel=0.5)
cyl("LED", 1.5, 1.0, (hx + 100.8, -8, hz + 17), axis="X", mat=RED, bevel=0.3)
rbox("ToFWindow", (14, 16, 4), (hx + 26, 0, hz + 63), BLACK, bevel=1.8)
rbox("ToFGlass", (8, 10, 1), (hx + 26, 0, hz + 65.2), LENS, bevel=0.4)
# Jaw: cream, under a dark mouth, wide like a smile, on cheek drums with the jaw servo's horn.
loft_blob("Jaw", [(hx + 6, 26, hz - 20, hz - 3), (hx + 40, 30, hz - 24, hz - 3), (hx + 78, 24, hz - 20, hz - 3), (hx + 96, 13, hz - 13, hz - 3)], CREAM, subsurf=2)
rbox("Mouth", (78, 40, 4), (hx + 52, 0, hz - 1.5), BLACK, bevel=1.5)
for s, side in ((1, "L"), (-1, "R")):
    cyl(f"CheekDrum_{side}", 12, 10, (hx + 12, s * 31, hz - 6), axis="Y", mat=CREAM, bevel=2.5)
    cyl(f"JawHorn_{side}", 6.5, 2, (hx + 12, s * 36.6, hz - 6), axis="Y", mat=ALU)
    for k, (dx, dz) in enumerate(((3.4, 0), (-3.4, 0), (0, 3.4), (0, -3.4))):
        cyl(f"JawHornScrew_{side}{k}", 0.8, 0.6, (hx + 12 + dx, s * 37.8, hz - 6 + dz), axis="Y", verts=12, bevel=0.2)
    # Two rounded fangs, in front of the lower lip.
    cyl(f"Fang_{side}", 2.4, 6, (hx + 92, s * 9, hz - 1), axis="Z", mat=WHITE, bevel=0.3)
    sphere(f"FangTip_{side}", 2.4, (hx + 92, s * 9, hz - 4), WHITE)

# ---------------------------------------------------------------- tail: five segments, black rings, lime tip
TAIL = [(-40, 28, 0), (-80, 24, 4), (-120, 19, 12), (-160, 14, 22), (-196, 10, 33), (-222, 7, 42)]
loft_blob("Tail", [(x, r * 0.92, BODY_Z - r + dz, BODY_Z + r * 0.85 + dz) for x, r, dz in TAIL], CREAM, subsurf=2, keep=0.3)
for i, (x, r, dz) in enumerate(TAIL[1:5], 1):
    cyl(f"TailRing{i}", r * 0.98 + 2.0, 8, (x, 0, BODY_Z - r * 0.07 + dz), axis="X", mat=BLACK, bevel=1.5)
    for sy in (1, -1):
        screw(f"TailScrew{i}{'L' if sy > 0 else 'R'}", (x - 16, sy * r * 0.86, BODY_Z + dz + 2), "Y")
loft_blob("TailTip", [(-218, 8, BODY_Z + 42 - 8, BODY_Z + 42 + 6), (-246, 4.5, BODY_Z + 52 - 4, BODY_Z + 52 + 3), (-262, 1.5, BODY_Z + 58 - 1.5, BODY_Z + 58 + 1.5)], LIME, subsurf=2, keep=0.2)
servo("TailYaw", (-46, 0, BODY_Z), "Z", 1, (26, 24, 20))


def show_in_viewport():
    """Material preview, the robot framed, in every 3D view of the open Blender."""
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


if LIVE:
    show_in_viewport()
    print("TINYREX LIVE", len([o for o in scene.objects if o.type == "MESH"]), "meshes")
else:
    # ---------------------------------------------------------------- studio, cameras, renders
    world = bpy.data.worlds.new("Studio")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.92, 0.92, 0.93, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.55
    scene.world = world
    bpy.ops.mesh.primitive_plane_add(size=4000, location=(0, 0, 0))
    floor = bpy.context.active_object
    floor.name = "Floor"
    floor.data.materials.append(material("Floor", (0.92, 0.92, 0.92), 0.9))
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (1.0, 1.0, 1.0, 1)


    def light(name, loc, energy, size):
        d = bpy.data.lights.new(name, "AREA")
        d.energy = energy
        d.size = size
        o = bpy.data.objects.new(name, d)
        o.location = loc
        scene.collection.objects.link(o)
        o.rotation_euler = (Vector((0, 0, 140)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


    light("Key", (650, -550, 800), 2200000, 500)
    light("Fill", (-500, 650, 350), 500000, 900)
    light("Rim", (-700, -200, 650), 900000, 400)

    cam_data = bpy.data.cameras.new("Cam")
    cam = bpy.data.objects.new("Cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam_data.lens = 85
    cam_data.clip_start = 10
    cam_data.clip_end = 20000

    engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items]
    scene.render.engine = "BLENDER_EEVEE" if "BLENDER_EEVEE" in engines else engines[0]
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1000
    scene.render.film_transparent = False
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
        scene.view_settings.exposure = 0.0
    except TypeError:
        scene.view_settings.view_transform = "Standard"

    target = Vector((-10, 0, 140))
    views = {
        "hero": Vector((720, -860, 420)),
        "side": Vector((0, -1350, 120)),
        "front": Vector((1300, 0, 120)),
        "top": Vector((0, 0, 1500)),
    }
    os.makedirs(OUT, exist_ok=True)
    for name, pos in views.items():
        cam.location = target + pos if name != "top" else Vector((-20, 0, 1500))
        look = (target if name != "top" else Vector((-20, 0, 0))) - cam.location
        cam.rotation_euler = look.to_track_quat("-Z", "Y" if name != "top" else "X").to_euler()
        cam_data.lens = 85 if name == "hero" else 70
        scene.render.filepath = os.path.join(OUT, f"tinyrex-{name}.png")
        bpy.ops.render.render(write_still=True)

    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "tinyrex.blend"))
    print("TINYREX DONE", len([o for o in scene.objects if o.type == "MESH"]), "meshes")
