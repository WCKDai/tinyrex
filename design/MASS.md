# TinyREX v4 — mass and balance

Computed by `blender/mass.py` from the Blender scene.

Microduck's masses and centres of mass come from its MJCF (its authors' CAD). TinyREX's start from those:
- subtract the Microduck parts it replaces (shells, face, eyes, jaw, feet, soles, camera parts it moves): 195.2 g;
- add its own parts: 436.9 g.

Each added part's mass is its mesh volume × material density, placed at its centroid:
- PLA 1.24 g/cm³; parts modelled solid are counted at 45 % (walls + 20 % infill);
- TPU 1.21; steel 7.85; NdFeB 7.5; acrylic 1.19;
- XL330 18 g; M2 screw 0.35 g.

**Check:** the same calculation puts Microduck's own centre of mass 0.5 mm from its
ankle axis in its HOME pose. Its authors tuned HOME to put it there.

## Mass

| | Microduck | TinyREX |
|---|---|---|
| Mass | 737.2 g | 979.0 g (+33 %) |
| Centre of mass, zero pose (x, y, z mm) | [1.7, -0.0, 133.2] | [-8.3, 0.0, 132.5] |

## Standing

Microduck stands in HOME (`mjlab_microduck/robot/microduck_constants.py`):
- hip pitch / ankle ±26.2°;
- hip roll ±5°;
- neck and head pitch 20°.

TinyREX's tail moves its centre of mass back. In Microduck's HOME, TinyREX's centre of mass sits
9.3 mm behind the ankle axis, with only 12.0 mm to the heel.

**TinyREX HOME** is Microduck's HOME with hip pitch / ankle at **17.84°**, feet flat. It
brings the centre of mass back over the ankle axis. It is clip-free and stored in `blender/joint_limits.json`.

| | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME |
|---|---|---|---|
| CoM ahead of ankle axis | 0.5 mm | -9.3 mm | -0.0 mm |
| CoM to heel / toe edge | — | 12.0 / 53.0 mm | 21.3 / 43.7 mm |
| CoM height | 128.7 mm | 128.7 mm | 129.2 mm |

## Static torque, standing on one leg (N·m)

Each figure is the moment of everything above each joint of the stance leg, about that joint's axis.

| Joint | Microduck zero pose | TinyREX zero pose | Microduck HOME | TinyREX in Microduck HOME | TinyREX HOME | TinyREX worst / XL330 stall (0.52) |
|---|---|---|---|---|---|---|
| hip roll | 0.161 | 0.212 | 0.157 | 0.208 | 0.208 | 41 % |
| hip pitch | 0.011 | 0.090 | 0.015 | 0.117 | 0.124 | 24 % |
| knee | 0.241 | 0.222 | 0.126 | 0.075 | 0.112 | 43 % |
| ankle | 0.241 | 0.222 | 0.002 | 0.092 | 0.003 | 43 % |

In TinyREX HOME, the knees and ankles carry less than Microduck in its HOME. Hip roll carries the extra
mass, at 40 % of stall. Two things remain:
- this is static only;
- Microduck's walking policy must be retrained for the new mass, centre of mass and pose.

## Heaviest added parts

| Part | g |
|---|---|
| Body | 63.8 |
| Head | 41.1 |
| HeadBack | 25.4 |
| XL330_TailYaw | 18.0 |
| XL330_ArmL | 18.0 |
| XL330_ArmR | 18.0 |
| TailSeg0 | 17.8 |
| Jaw | 15.8 |
| Thigh_R | 14.2 |
| Thigh_L | 14.2 |
| TailSeg1 | 10.8 |
| Sole_L | 10.2 |
