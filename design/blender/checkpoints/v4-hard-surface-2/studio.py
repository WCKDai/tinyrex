"""Studio renders of a TinyREX scene (EEVEE): soft key/fill/rim, a pale floor, a few views.
Sent after tinyrex_v2.py's globals with OUT_DIR and VIEWS_STUDIO set."""
import math
import os

import bpy
from mathutils import Vector

sc = bpy.data.scenes[SCENE]
bpy.context.window.scene = sc
studio = bpy.data.collections.get(f"{SCENE}: studio")
if studio is None:
    studio = bpy.data.collections.new(f"{SCENE}: studio")
    sc.collection.children.link(studio)
for o in list(studio.objects):
    bpy.data.objects.remove(o, do_unlink=True)
for c in sc.collection.children:
    if c.name.endswith(": rig"):
        c.hide_render = True
for o in sc.objects:
    if o.type == "EMPTY" or o.name.split(".")[0].startswith("Cut_"):
        o.hide_render = True


def light(name, loc, energy, size):
    d = bpy.data.lights.new(name, "AREA")
    d.energy, d.size = energy, size
    o = bpy.data.objects.new(name, d)
    studio.objects.link(o)
    o.location = loc
    o.rotation_euler = (Vector((0, 0, 120)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


light("Key", (700, -600, 900), 3.0e6, 600)
light("Fill", (-300, 800, 400), 1.0e6, 800)
light("Rim", (-800, -150, 700), 1.6e6, 500)
floor_me = bpy.data.meshes.new("Floor")
R = 4000
floor_me.from_pydata([(-R, -R, -0.2), (R, -R, -0.2), (R, R, -0.2), (-R, R, -0.2)], [], [(0, 1, 2, 3)])
floor = bpy.data.objects.new("Floor", floor_me)
fm = bpy.data.materials.get("StudioFloor") or bpy.data.materials.new("StudioFloor")
fm.use_nodes = True
fm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.93, 0.93, 0.92, 1)
fm.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.7
floor_me.materials.append(fm)
studio.objects.link(floor)
w = bpy.data.worlds.get("TinyREX Studio") or bpy.data.worlds.new("TinyREX Studio")
w.use_nodes = True
w.node_tree.nodes["Background"].inputs["Color"].default_value = (1, 1, 1, 1)
w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.4
sc.world = w
cd = bpy.data.cameras.new("StudioCam")
cd.lens = 85
cd.clip_start, cd.clip_end = 10, 30000
cam = bpy.data.objects.new("StudioCam", cd)
studio.objects.link(cam)
sc.camera = cam
for eng in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
    try:
        sc.render.engine = eng
        break
    except TypeError:
        pass
sc.render.resolution_x, sc.render.resolution_y = 1400, 1200
sc.render.film_transparent = False
sc.view_settings.view_transform = "Standard"
sc.view_settings.look = "None"
sc.view_settings.exposure = 0.0
mid = Vector((0, 0, 135))
out = []
for view, off in VIEWS_STUDIO.items():
    cam.location = mid + Vector(off)
    cam.rotation_euler = (mid - cam.location).to_track_quat("-Z", "Y").to_euler()
    sc.render.filepath = os.path.join(OUT_DIR, f"studio-{view}.png")
    bpy.ops.render.render(write_still=True)
    out.append(sc.render.filepath)
print(out)
