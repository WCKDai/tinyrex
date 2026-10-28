# TinyREX v4 — mass and balance

Computed by `blender/mass.py` from the Blender scene, at the zero pose (all joints at 0).
Microduck's masses and centres of mass come from its MJCF (its authors' CAD). TinyREX's start from
those, then:
- subtract the Microduck parts it replaces (shells, face, eyes, jaw, feet, soles: 195.2 g);
- add its own parts (408.0 g).

Each added part's mass is its mesh volume × material density, placed at its centroid:
- PLA 1.24 g/cm³; parts modelled solid are counted at 45 % (walls + 20 % infill);
- TPU 1.21; steel 7.85; NdFeB 7.5; acrylic 1.19;
- XL330 18 g; M2 screw 0.35 g.

| | Microduck | TinyREX |
|---|---|---|
| Mass | 737.2 g | 977.1 g (+33 %) |
| Centre of mass (x, y, z mm) | [1.7, -0.0, 133.2] | [-8.4, 0.0, 132.1] |
| Feet, front to back (x mm) | [-51.8, 2.2] | [-53.0, 12.0] |
| CoM to the feet's front / back edge | 0.5 / 53.5 mm | 20.4 / 44.6 mm |

The tail moves the centre of mass back, over the middle of the feet. Microduck's sits on the front
edge of its soles at this pose; TinyREX's toes also lengthen the feet forward.

## Static torque, standing on one leg

This is the moment of everything above each joint of the stance leg, about that joint's axis:

| Joint | Microduck (N·m) | TinyREX (N·m) | TinyREX / XL330 stall (0.52 N·m @ 5 V) |
|---|---|---|---|
| hip_yaw | 0.000 | 0.000 | 0 % |
| hip_roll | 0.161 | 0.212 | 41 % |
| hip_pitch | 0.011 | 0.090 | 17 % |
| knee | 0.241 | 0.221 | 42 % |
| ankle | 0.241 | 0.221 | 42 % |

TinyREX is 33 % heavier, yet its knee and ankle load is *lower* than Microduck's: the centre of mass
sits nearer their axes. The extra weight shows at the hip roll, +28 %, and the hip pitch. Every joint
stays under about 45 % of stall here. Two things still need checking: a crouched walking pose, and
retraining Microduck's walking policy for the new mass.

## Heaviest added parts

| Part | g |
|---|---|
| Body | 55.2 |
| Head | 42.9 |
| HeadBack | 27.5 |
| TailSeg0 | 18.3 |
| XL330_TailYaw | 18.0 |
| XL330_ArmL | 18.0 |
| XL330_ArmR | 18.0 |
| Jaw | 15.4 |
| Thigh_L | 10.9 |
| Thigh_R | 10.9 |
| TailSeg1 | 10.8 |
| Sole_L | 10.2 |
