# TinyREX v4 — prototype bill of materials

Generated from the Blender scene (`blender/tinyrex_v4.blend`, built by `tinyrex_v2.py` + `rig.py` + `v4_parts.py`).
Frame and joint layout follow Microduck (MJCF, Apache-2.0); every part here is TinyREX's own model.

## Actuators — 18 × Dynamixel XL330-M288-T (TTL bus)

| Qty | Joint(s) | Notes |
|---|---|---|
| 10 | legs: hip yaw, hip roll, hip pitch, knee, ankle × 2 | Microduck placement |
| 2 | neck pitch, head pitch | Microduck placement |
| 2 | head yaw, head roll | Microduck placement |
| 1 | jaw | drives the jaw through a shaft on its horn; jaw arms run outside the cheeks to the hinge |
| 1 | tail yaw | in the tail root; tail on a magnetic breakaway coupling |
| 2 | shoulders (left, right) | cases sit in pockets in the upper body, horns outside; arms screw to the horns |

Software joint limits (collision-free ranges, from a sweep of the full model): `blender/joint_limits.json`.

## Electronics

- Raspberry Pi Zero 2 W + robot hat, in the head
- M12 camera module on a CSI ribbon, centred in the brow visor on two rails (looks out through the smoked visor strip)
- VL53L5CX ToF board, behind a small window in the snout front under the status LED
- LSM6DSV16X head IMU on the head frame
- Status LED (red) on the snout front
- Speaker, in the head
- NP-F970-type battery in the body, swapped through the belly door; USB-C charge port and power switch on the tail root

## Printed and other parts

- Shells (PLA, 2.4 mm walls): head, head back cap, muzzle band, brow visor, body, backpack, battery door
- Jaw, jaw arms and cheeks, throat plate, neck collar (4 bars), body mounting bosses (4)
- Arms: upper arm, forearm, hand, 2 claws each; elbow hub and pin (fixed elbow)
- Legs: thigh armour plates, feet, two toe caps each
- Tail: arm, 3 segments, tip; 3 TPU flex rings (bonded into both neighbours); lime coupling with 2 × 6×3 mm magnets
- Soles and toe soles: TPU / rubber
- Smoked acrylic visor strip (IR-transparent for the ToF)
- Hinge shafts (2), cheek pins (2), elbow pins (2): 6 mm / 4 mm steel
- 50 visible M2 button-head screws (plus each XL330's own horn and case screws)
