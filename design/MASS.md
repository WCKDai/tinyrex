# TinyREX v4 — mass and balance

Computed by `blender/mass.py` from the Blender scene; this file by `blender/write_mass_md.py`.

Microduck's masses and centres of mass come from its MJCF (its authors' CAD). TinyREX's start from those:
- subtract the Microduck parts it replaces (shells, face, eyes, jaw, feet, soles, camera parts it moves): 195.2 g;
- add its own parts: 447.8 g.

Each added part's mass is its mesh volume × material density, placed at its centroid:
- PLA 1.24 g/cm³; parts modelled solid are counted at 45 % (walls + 20 % infill);
- TPU 1.21; steel 7.85; NdFeB 7.5; acrylic 1.19;
- XL330 18 g; M2 screw 0.35 g.

**Check:** the same calculation puts Microduck's own centre of mass 0.5 mm from its
ankle axis in its HOME pose. Its authors tuned HOME to put it there.

## Mass

| | Microduck | TinyREX |
|---|---|---|
| Mass | 737.2 g | 989.8 g (+34 %) |
| Centre of mass, zero pose (x, y, z mm) | [1.7, -0.0, 133.2] | [-7.9, 0.0, 132.6] |

## Standing

Microduck stands in HOME (`mjlab_microduck/robot/microduck_constants.py`):
- hip pitch / ankle ±26.2°;
- hip roll ±5°;
- neck and head pitch 20°.

TinyREX's tail moves its centre of mass back. In Microduck's HOME, TinyREX's centre of mass sits
8.9 mm behind the ankle axis, with only 12.4 mm to the heel.

**TinyREX HOME** is Microduck's HOME with these changes. It is clip-free and stored in `blender/joint_limits.json`.
- hip pitch / ankle at **19.51°**, feet flat;
- neck pitch at **16.6°**, inside its 18° collision-free limit.

| | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME |
|---|---|---|---|
| CoM ahead of ankle axis | 0.5 mm | -8.9 mm | 0.0 mm |
| CoM to heel / toe edge | — | 12.4 / 52.6 mm | 21.3 / 43.7 mm |
| CoM height | 128.7 mm | 128.8 mm | 129.2 mm |

## Static torque, standing on one leg (N·m)

Each figure is the moment of everything above each joint of the stance leg, about that joint's axis.

| Joint | Microduck zero pose | TinyREX zero pose | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME | TinyREX worst / XL330 stall (0.52) |
|---|---|---|---|---|---|---|
| hip roll | 0.161 | 0.215 | 0.157 | 0.210 | 0.210 | 41 % |
| hip pitch | 0.011 | 0.087 | 0.015 | 0.115 | 0.106 | 22 % |
| knee | 0.241 | 0.228 | 0.126 | 0.080 | 0.124 | 44 % |
| ankle | 0.241 | 0.228 | 0.002 | 0.089 | 0.003 | 44 % |

In TinyREX HOME the knees carry about what Microduck's do in its HOME, and the ankles the same. Hip
roll carries the extra mass, at about 40 % of stall. The simulation (`sim/`, see `sim/README.md`)
takes this further: it shows that both robots stand and how hard a push each survives.

## Heaviest added parts

| Part | g |
|---|---|
| BodyR | 30.9 |
| BodyL | 30.8 |
| HeadBack | 25.4 |
| HeadTop | 23.5 |
| XL330_TailYaw | 18.0 |
| XL330_ArmL | 18.0 |
| XL330_ArmR | 18.0 |
| TailSeg0 | 17.8 |
| HeadBase | 17.6 |
| Jaw | 15.8 |
| Thigh_R | 14.2 |
| Thigh_L | 14.2 |
