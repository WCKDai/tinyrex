"""Build TinyREX's MJCF from Microduck's (Apache-2.0, pollen-robotics/microduck_rl) and the export
from Blender (blender/export_sim.py).

    python build_mjcf.py MICRODUCK_ROBOT_WALK_XML

Keeps Microduck's kinematic tree, joint and actuator models (its XL330 "chosen_actuator" class);
replaces every body's geometry and inertia with TinyREX's own; sets the collision-free joint
limits (blender/joint_limits.json); adds the jaw, the tail and the two shoulders, each on an XL330;
gives the feet box contacts (TinyREX's soles and toes). No Microduck mesh is used or referenced.
Writes tinyrex.xml (the robot) and scene.xml (floor, light, the HOME keyframe)."""
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = sys.argv[1]
bodies = json.load(open(os.path.join(HERE, "tinyrex_bodies.json")))
limits = json.load(open(os.path.join(HERE, "..", "blender", "joint_limits.json")))

tree = ET.parse(SRC)
root = tree.getroot()
root.set("model", "tinyrex")
root.find("compiler").set("meshdir", "meshes")

# Assets: TinyREX's meshes only.
asset = root.find("asset")
for el in list(asset):
    asset.remove(el)
COLOURS = {"jaw_soft": "0.62 0.82 0.18 1", "jaw": "0.86 0.84 0.78 1", "tail": "0.86 0.84 0.78 1", "trunk_base": "0.86 0.84 0.78 1",
           "upper_leg_left": "0.62 0.82 0.18 1", "upper_leg_right": "0.62 0.82 0.18 1"}
for b, v in bodies["bodies"].items():
    if "mesh" in v:
        ET.SubElement(asset, "mesh", name=f"tr_{b}", file=v["mesh"])


def set_inertial(el, name):
    v = bodies["bodies"][name]
    i = el.find("inertial")
    if i is None:
        i = ET.SubElement(el, "inertial")
    i.attrib = {"pos": " ".join(f"{x:.6g}" for x in v["pos"]), "mass": f"{v['mass']:.6g}",
                "fullinertia": " ".join(f"{x:.6g}" for x in v["fullinertia"])}


def stl_box(path):
    """The bounds of a binary STL (metres)."""
    import struct
    with open(path, "rb") as f:
        f.read(80)
        (n,) = struct.unpack("<I", f.read(4))
        lo, hi = [1e9] * 3, [-1e9] * 3
        for _ in range(n):
            vals = struct.unpack("<12fH", f.read(50))
            for k in range(3):
                for i in range(3):
                    c = vals[3 + 3 * k + i]
                    lo[i], hi[i] = min(lo[i], c), max(hi[i], c)
    return lo, hi


def dress(el, name):
    """Replace a body's geoms with TinyREX's: its mesh (visual), a box round it that meets the floor
    (falls and getting up need the whole body to touch the ground; contype 2 / conaffinity 1: the
    floor, not each other), and, for a foot, its sole contacts."""
    for g in list(el.findall("geom")):
        el.remove(g)
    v = bodies["bodies"].get(name, {})
    if "mesh" in v:
        ET.SubElement(el, "geom", type="mesh", mesh=f"tr_{name}", attrib={"class": "visual"}, rgba=COLOURS.get(name, "0.2 0.2 0.22 1"))
        if not bodies["feet"].get(name):
            lo, hi = stl_box(os.path.join(HERE, "meshes", v["mesh"]))
            size = [max((h - l) / 2, 0.002) for l, h in zip(lo, hi)]
            ET.SubElement(el, "geom", type="box", name=f"{name}_hull", group="3", contype="2", conaffinity="1", mass="0",
                          pos=" ".join(f"{(l + h) / 2:.5g}" for l, h in zip(lo, hi)), size=" ".join(f"{x:.5g}" for x in size))
    for k, box in enumerate(bodies["feet"].get(name, [])):
        side = "left" if name.endswith("left") else "right"
        ET.SubElement(el, "geom", type="box", name=f"{side}_foot_collision" if k == 0 else f"{side}_toe_collision{k}",
                      attrib={"class": "collision"}, pos=" ".join(f"{x:.6g}" for x in box["pos"]), size=" ".join(f"{x:.6g}" for x in box["size"]))


for el in root.iter("body"):
    name = el.get("name")
    set_inertial(el, name)
    dress(el, name)
    j = el.find("joint")
    if j is not None and j.get("name") in limits["joints"]:
        lo, hi = limits["joints"][j.get("name")]["collision_free"]
        j.set("range", f"{math.radians(lo):.5f} {math.radians(hi):.5f}")

# The extra bodies, each on an XL330 with Microduck's actuator model.
by_name = {el.get("name"): el for el in root.iter("body")}
actuator = root.find("actuator")
for b, x in bodies["extra"].items():
    parent = by_name[x["parent"]]
    el = ET.Element("body", name=b, pos=" ".join(f"{v / 1.0:.6g}" for v in x["pos"]), quat=" ".join(f"{v:.6g}" for v in x["quat"]))
    lo, hi = limits["joints"][x["joint"]]["collision_free"]
    ET.SubElement(el, "joint", name=x["joint"], type="hinge", axis=" ".join(f"{v:.6g}" for v in x["axis"]),
                  range=f"{math.radians(lo):.5f} {math.radians(hi):.5f}", attrib={"class": "chosen_actuator"})
    set_inertial(el, b)
    dress(el, b)
    # (inertial first, as MJCF expects)
    el[:] = sorted(el, key=lambda c: {"inertial": 0, "joint": 1}.get(c.tag, 2))
    parent.append(el)
    ET.SubElement(actuator, "position", name=x["joint"], joint=x["joint"], attrib={"class": "chosen_actuator"})

ET.indent(tree, "  ")
robot = os.path.join(HERE, "tinyrex.xml")
tree.write(robot, xml_declaration=True, encoding="unicode")
with open(robot) as f:
    body = f.read()
with open(robot, "w") as f:
    f.write(body.replace("<mujoco", "<!-- TinyREX: generated by design/sim/build_mjcf.py from Microduck's MJCF (Apache-2.0,\n"
                         "     pollen-robotics/microduck_rl) and TinyREX's own parts. Do not edit by hand. -->\n<mujoco", 1))

# The scene, with TinyREX's HOME keyframe (blender/joint_limits.json: standing_pose_rad).
model = mujoco.MjModel.from_xml_path(robot)
home = limits["standing_pose_rad"]["tinyrex_home"]
qpos = []
for j in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
        qpos += [0, 0, 0.12, 1, 0, 0, 0]
    else:
        qpos.append(home.get(name, 0.0))
data = mujoco.MjData(model)
data.qpos[:] = qpos
mujoco.mj_forward(model, data)
# stand the soles on the floor
def lowest(g):
    corners = [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    R = data.geom_xmat[g].reshape(3, 3)
    return min((R @ (model.geom_size[g] * c) + data.geom_xpos[g])[2] for c in corners)


low = min(lowest(g) for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX)
qpos[2] -= low - 0.0002
ctrl = [home.get(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[a][0]), 0.0) for a in range(model.nu)]
scene = f"""<mujoco model="tinyrex scene">
  <include file="tinyrex.xml"/>
  <option timestep="0.002"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="150" elevation="-15"/>
  </visual>
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.85 0.86 0.88" rgb2="0.78 0.79 0.82" markrgb="0.6 0.6 0.6" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
  <keyframe>
    <key name="HOME" qpos="{' '.join(f'{v:.6g}' for v in qpos)}" ctrl="{' '.join(f'{v:.6g}' for v in ctrl)}"/>
  </keyframe>
</mujoco>
"""
with open(os.path.join(HERE, "scene.xml"), "w") as f:
    f.write(scene)
m2 = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
print("built", {"bodies": m2.nbody, "joints": m2.njnt, "actuators": m2.nu, "mass_kg": round(sum(m2.body_mass), 4)})
