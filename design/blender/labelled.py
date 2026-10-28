"""Labelled renders: the assembled robot with its parts and servos named, and an exploded view.

Sent after tinyrex_v2.py's, rig.py's and studio.py's globals (studio lights and camera set up),
with OUT_DIR. Renders OUT_DIR/labelled-<view>.png and writes OUT_DIR/labelled-<view>.json: each
label's text and its anchor in the image (0..1, origin top left), for annotate.py to draw."""
import json
import math
import os

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

sc = bpy.data.scenes[SCENE]

# Printed and visible parts worth naming (base name -> label); servos are named by the joint
# they drive.
NAMES = {
    "HeadTop": "head top (lime)", "HeadBase": "head base (lime)", "HeadBack": "head back cap",
    "Visor": "brow visor: camera + ToF", "Muzzle": "muzzle panel", "Jaw": "jaw", "JawArm_L": "jaw arm (on jaw servo)",
    "EyeSocket_L": "eye socket", "Spike_H1": "dorsal spikes", "BodyL": "body, left half", "BodyR": "body, right half",
    "Backpack": "backpack", "Flank_L": "flank plate", "NeckCollar0": "neck collar", "Throat": "throat plate",
    "Nape": "nape plate", "ArmUpper_L": "upper arm", "Hand_L": "hand", "Thigh_L": "thigh shield", "FootGuardO_L": "ankle guard",
    "ToeCap_L0": "toe cap", "Sole_L": "sole (TPU)", "TailSeg0": "tail segment", "TailFlex0": "flex ring (TPU)",
    "TailTip": "tail tip", "TailCoupling": "magnetic tail coupling", "BatteryDoor": "battery door",
}
EXPLODE = {  # base name -> offset (mm)
    "HeadTop": (0, 0, 45), "Visor": (0, 0, 80), "VisorGlass": (0, 0, 80), "HeadBack": (-40, 0, 10), "Muzzle": (30, 0, -5),
    "BodyL": (0, 45, 0), "BodyR": (0, -45, 0), "Backpack": (0, 0, 45), "BatteryDoor": (0, 0, -40),
    "Flank_L": (0, 75, 0), "Flank_R": (0, -75, 0), "Thigh_L": (0, 35, 0), "Thigh_R": (0, -35, 0),
    "FootGuardO_L": (0, 22, 0), "FootGuardO_R": (0, -22, 0), "Nape": (-25, 0, 0), "Throat": (25, 0, 0),
}
EXPLODE_PREFIX = {"Spike_H": "HeadTop", "EyeSocket": "HeadTop", "Brow": "HeadTop", "Eye": "HeadTop", "Spike_B": "Backpack"}


def joint_servos():
    """Each joint's servo: the XL330 case whose output (local X, at the case's origin) lies on the
    joint's axis."""
    out = {}
    cases = [o for o in sc.objects if o.type == "MESH" and o.name.split(".")[0].startswith("XL330") and (len(o.name.split(".")) == 1 or o.name.split(".")[1].isdigit())]
    for e in sc.objects:
        if e.type != "EMPTY" or "joint" not in e:
            continue
        p = e.matrix_world.translation
        a = (e.matrix_world.to_3x3() @ Vector(e["axis"])).normalized()
        best = None
        for o in cases:
            c = o.matrix_world.translation
            d = (c - p) - a * (c - p).dot(a)  # distance from the axis line
            if best is None or d.length < best[0]:
                best = (d.length, o)
        if best and best[0] < 6:
            out[best[1].name] = e["joint"].replace("_", " ")
    return out


def anchors(names):
    cam = sc.camera
    out = []
    for o in sc.objects:
        if o.type != "MESH" or o.hide_render:
            continue
        base = o.name.split(".")[0]
        text = names.get(o.name) or names.get(base)
        if not text:
            continue
        pts = [o.matrix_world @ Vector(c) for c in o.bound_box]
        mid = sum(pts, Vector()) / 8
        co = world_to_camera_view(sc, cam, mid)
        out.append({"text": text, "x": co.x, "y": 1 - co.y, "depth": co.z})
    return out


def render(view, off, names, explode=False):
    moved = []
    if explode:
        for o in sc.objects:
            if o.type != "MESH" or "body" not in o:
                continue
            base = o.name.split(".")[0]
            host = base if base in EXPLODE else next((h for p, h in EXPLODE_PREFIX.items() if base.startswith(p)), None)
            if host is None and ("Screw" in base or base.startswith("Boss_")):
                o.hide_render = True
                moved.append((o, None))
                continue
            if host:
                d = Vector(EXPLODE[host])
                moved.append((o, o.matrix_world.copy()))
                o.matrix_world.translation += d
    mid = Vector((0, 0, 135))
    cam = sc.camera
    cam.location = mid + Vector(off)
    cam.rotation_euler = (mid - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.view_layer.update()
    path = os.path.join(OUT_DIR, f"labelled-{view}.png")
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    labels = anchors(names)
    for o, mw in moved:
        if mw is None:
            o.hide_render = False
        else:
            o.matrix_world = mw
    bpy.context.view_layer.update()
    json.dump({"image": path, "labels": labels}, open(path.replace(".png", ".json"), "w"))
    return path


names = dict(NAMES)
names.update({k: f"XL330 · {v}" for k, v in joint_servos().items()})
sc.render.resolution_x, sc.render.resolution_y = 1800, 1500
print("LABELLED", [render("hero", (640, -760, 300), names), render("back", (-650, 680, 330), names),
                   render("exploded", (720, -820, 380), NAMES, explode=True)])
