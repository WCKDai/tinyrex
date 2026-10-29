"""Write design/MASS.md from mass.py's output (its MASS / HOME / TREX_HOME lines).

    python write_mass_md.py MASS_OUTPUT.txt"""
import json
import math
import os
import sys

R = {}
for line in open(sys.argv[1]):
    k, v = line.split(" ", 1)
    R[k] = json.loads(v)
z, h, t = R["MASS"], R["HOME"], R["TREX_HOME"]
th = t["angles"]


def row(j, name):
    cols = " | ".join(f"{d[r]['torque_Nm'][j]:.3f}" for d, r in ((z, "microduck"), (z, "tinyrex"), (h, "microduck"), (h, "tinyrex"), (t, "tinyrex")))
    return f"| {name} | {cols} | {100 * max(d['tinyrex']['torque_Nm'][j] for d in (z, h, t)) / 0.52:.0f} % |"


md = f"""# TinyREX v4 — mass and balance

Computed by `blender/mass.py` from the Blender scene; this file by `blender/write_mass_md.py`.

Microduck's masses and centres of mass come from its MJCF (its authors' CAD). TinyREX's start from those:
- subtract the Microduck parts it replaces (shells, face, eyes, jaw, feet, soles, camera parts it moves): {z['removed_g']} g;
- add its own parts: {z['added_g']} g.

Each added part's mass is its mesh volume × material density, placed at its centroid:
- PETG 1.27 g/cm³; parts modelled solid are counted at 45 % (walls + 20 % gyroid);
- TPU 1.21; steel 7.85; NdFeB 7.5; acrylic 1.19;
- XL330 18 g; M2 screw 0.35 g.

**Check:** the same calculation puts Microduck's own centre of mass {h['microduck']['com_ahead_of_ankle_mm']} mm from its
ankle axis in its HOME pose. Its authors tuned HOME to put it there.

## Mass

| | Microduck | TinyREX |
|---|---|---|
| Mass | {z['microduck']['mass_g']} g | {z['tinyrex']['mass_g']} g (+{100 * (z['tinyrex']['mass_g'] / z['microduck']['mass_g'] - 1):.0f} %) |
| Centre of mass, zero pose (x, y, z mm) | {z['microduck']['com_mm']} | {z['tinyrex']['com_mm']} |

## Standing

Microduck stands in HOME (`mjlab_microduck/robot/microduck_constants.py`):
- hip pitch / ankle ±26.2°;
- hip roll ±5°;
- neck and head pitch 20°.

TinyREX's tail moves its centre of mass back. In Microduck's HOME, TinyREX's centre of mass sits
{-h['tinyrex']['com_ahead_of_ankle_mm']} mm behind the ankle axis, with only {h['tinyrex']['margins_mm']['back']} mm to the heel.

**TinyREX HOME** is Microduck's HOME with these changes. It is clip-free and stored in `blender/joint_limits.json`.
- hip pitch / ankle at **{math.degrees(abs(th['left_hip_pitch'])):.2f}°**, feet flat;
- neck pitch at **{math.degrees(th['neck_pitch']):.1f}°**, inside its 18° collision-free limit.

| | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME |
|---|---|---|---|
| CoM ahead of ankle axis | {h['microduck']['com_ahead_of_ankle_mm']} mm | {h['tinyrex']['com_ahead_of_ankle_mm']} mm | {t['tinyrex']['com_ahead_of_ankle_mm']} mm |
| CoM to heel / toe edge | — | {h['tinyrex']['margins_mm']['back']} / {h['tinyrex']['margins_mm']['front']} mm | {t['tinyrex']['margins_mm']['back']} / {t['tinyrex']['margins_mm']['front']} mm |
| CoM height | {h['microduck']['com_mm'][2]} mm | {h['tinyrex']['com_mm'][2]} mm | {t['tinyrex']['com_mm'][2]} mm |

## Static torque, standing on one leg (N·m)

Each figure is the moment of everything above each joint of the stance leg, about that joint's axis.

| Joint | Microduck zero pose | TinyREX zero pose | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME | TinyREX worst / XL330 stall (0.52) |
|---|---|---|---|---|---|---|
{row('left_hip_roll', 'hip roll')}
{row('left_hip_pitch', 'hip pitch')}
{row('left_knee', 'knee')}
{row('left_ankle', 'ankle')}

In TinyREX HOME the knees carry about what Microduck's do in its HOME, and the ankles the same. Hip
roll carries the extra mass, at about 40 % of stall. The simulation (`sim/`, see `sim/README.md`)
takes this further: it shows that both robots stand and how hard a push each survives.

## Heaviest added parts

| Part | g |
|---|---|
""" + "\n".join(f"| {n} | {g} |" for n, g in z["heaviest_added"]) + "\n"
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "MASS.md"), "w").write(md)
