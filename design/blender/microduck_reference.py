"""Rebuild Pollen Robotics' Microduck in Blender from its MJCF, as a size and layout reference.

    Blender --background --factory-startup --python microduck_reference.py -- MJCF OUT_DIR

The meshes (CC BY-NC-SA 4.0, pollen-robotics/microduck_rl) are read from the MJCF's meshdir and
placed by composing every body's and geom's pos/quat. Nothing of them is copied into TinyREX:
this scene is only rendered and saved locally, next to the reference files.
Millimetres, X forward, Z up, like the TinyREX scene.
"""
import os
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Quaternion, Vector

args = sys.argv[sys.argv.index("--") + 1:]
MJCF, OUT = args[0], args[1]
root = ET.parse(MJCF).getroot()
meshdir = os.path.join(os.path.dirname(MJCF), root.find("compiler").get("meshdir", "."))
meshes = {m.get("name", os.path.splitext(m.get("file"))[0]): m.get("file") for m in root.iter("mesh")}

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.scale_length = 0.001


def transform(el):
    pos = Vector([float(v) for v in el.get("pos", "0 0 0").split()]) * 1000.0
    q = el.get("quat")
    rot = Quaternion([float(v) for v in q.split()]) if q else Quaternion()
    return Matrix.Translation(pos) @ rot.to_matrix().to_4x4()


GREY = bpy.data.materials.new("MicroduckGrey")
GREY.diffuse_color = (0.75, 0.75, 0.78, 1)
GREY.use_nodes = True
GREY.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.75, 0.75, 0.78, 1)
SERVO = bpy.data.materials.new("MicroduckServo")
SERVO.use_nodes = True
SERVO.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.03, 0.03, 0.035, 1)
cache = {}
count = 0


def place(geom, world):
    global count
    name = geom.get("mesh")
    if not name or geom.get("type", "mesh") != "mesh" or name not in meshes:
        return
    path = os.path.join(meshdir, meshes[name])
    if name not in cache:
        bpy.ops.wm.stl_import(filepath=path)
        src = bpy.context.selected_objects[0]
        cache[name] = src.data
        bpy.data.objects.remove(src)
    o = bpy.data.objects.new(f"md_{name}", cache[name])
    scene.collection.objects.link(o)
    o.matrix_world = world @ transform(geom) @ Matrix.Scale(1000.0, 4)
    o.data.materials.clear()
    o.data.materials.append(SERVO if name == "xl330" else GREY)
    count += 1


def walk(body, parent):
    world = parent @ transform(body)
    for g in body.findall("geom"):
        place(g, world)
    for child in body.findall("body"):
        walk(child, world)


for b in root.find("worldbody").findall("body"):
    walk(b, Matrix.Identity(4))
for g in root.find("worldbody").findall("geom"):
    place(g, Matrix.Identity(4))

# Stand it on the ground and report its size.
objs = [o for o in scene.objects if o.type == "MESH"]
pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
lo = Vector([min(p[i] for p in pts) for i in range(3)])
hi = Vector([max(p[i] for p in pts) for i in range(3)])
for o in objs:
    o.matrix_world = Matrix.Translation((0, 0, -lo.z)) @ o.matrix_world
print("MICRODUCK size mm", [round(hi[i] - lo[i], 1) for i in range(3)], "parts", count)

world = bpy.data.worlds.new("Studio")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (1, 1, 1, 1)
world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.6
scene.world = world
for name, loc, e in (("Key", (600, -550, 800), 2200000), ("Fill", (-500, 650, 350), 500000), ("Rim", (-700, -200, 650), 900000)):
    d = bpy.data.lights.new(name, "AREA")
    d.energy, d.size = e, 500
    o = bpy.data.objects.new(name, d)
    o.location = loc
    scene.collection.objects.link(o)
    o.rotation_euler = (Vector((0, 0, 120)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
cam_data = bpy.data.cameras.new("Cam")
cam = bpy.data.objects.new("Cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
cam_data.lens = 70
cam_data.clip_start, cam_data.clip_end = 10, 20000
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x, scene.render.resolution_y = 1200, 1000
mid = (lo + hi) / 2
mid.z = (hi.z - lo.z) / 2
os.makedirs(OUT, exist_ok=True)
for view, off in (("hero", Vector((520, -620, 300))), ("side", Vector((0, -900, 0))), ("front", Vector((900, 0, 0)))):
    cam.location = mid + off
    cam.rotation_euler = (mid - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = os.path.join(OUT, f"microduck-{view}.png")
    bpy.ops.render.render(write_still=True)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "microduck_reference.blend"))
