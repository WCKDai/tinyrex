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
| 1 | tail yaw | in the tail root; the tail rides a slip clutch on its horn (it pushes the robot up again, so it stays on) |
| 2 | shoulders (left, right) | cases sit in pockets in the upper body, horns outside; each arm rides a slip clutch on its horn |

Software joint limits (collision-free ranges, from a sweep of the full model): `blender/joint_limits.json`.

## Electronics

- Raspberry Pi Zero 2 W + robot hat, in the head
- M12 camera module on a CSI ribbon, centred in the brow visor on two rails (looks out through the smoked visor strip)
- VL53L5CX ToF board, behind a small window in the snout front under the status LED
- LSM6DSV16X head IMU on the head frame
- Status LED (red) on the snout front
- Speaker, in the head
- NP-F970-type battery in the body, swapped through the belly door; USB-C charge port and power switch on the tail root

## Cables

- Camera: 22-way 0.5 mm-pitch CSI flex, about 60 mm. It runs from under the camera in the visor, over the
  compute board's top edge, and across its face to the CSI connector.
- Arm servos: 2 × Dynamixel 3-pin TTL leads, about 120 mm each. Each runs from the servo's bottom socket, back
  between the arm and hip-yaw servos, and over the battery holders into Microduck's bus board.
- The tail servo and all of Microduck's servos use Microduck's harness.

## Printed and other parts

Printed in PETG (it tumbles: PLA is too brittle), shells with 2.4 mm walls (6 perimeters of a 0.4 mm
nozzle), solid-modelled parts with walls and 20 % gyroid infill.

- Shells: head top and head base, head back cap, muzzle band, brow visor, body halves (left/right), backpack, battery door
- Printed with their shells instead of glued on (no glue joint to break in a fall):
  - head top: the four dorsal spikes, both brows and the eye sockets (black: a two-colour print);
  - body halves: the back spike (half in each);
  - each tail segment: its lime spike (two-colour);
  - each foot: its two lime toe caps (two-colour);
  - every M2 insert boss and standoff, printed with the panel it backs.
- Jaw, jaw arms and cheeks, throat plate, neck collar (4 bars), body mounting bosses (4)
- Arms: upper arm, forearm, hand, 2 claws each; elbow hub and pin (fixed elbow)
- Legs: thigh armour plates, feet with toe caps, ankle guards
- Tail: arm, 3 segments, black collars between them, all on a 4 mm pultruded carbon spine; soft TPU tip (the bumper)
- Slip clutches (TPU 95A friction parts, each pressed by one screw through a wave spring; they hold about
  1.2 N·m, more than any servo can push with, and slip in a fall before the servo's gears strip):
  - each arm: a friction disc in a pocket in the upper arm, on the shoulder horn;
  - the tail: a friction disc in the coupling hub, on the tail-yaw horn;
  - the jaw: a friction sleeve in the driven jaw arm's hub, round its shaft.
- Soles and toe soles: TPU
- Eye domes and visor strip: polycarbonate (it doesn't crack on impact; the visor strip is smoked, IR-transparent for the ToF)
- Hinge shafts (2), cheek pins (2), elbow pins (2): 6 mm / 4 mm steel; 3 wave springs (clutches)
- 107 M2 button-head screws (73 on the shells and plates, 34 fixing the frame brackets to the servos), plus each
  XL330's own horn screws. Each one bites into one of:
  - a servo horn;
  - an XL330 case screw hole (frame brackets, nape, throat, tail root);
  - a solid printed part;
  - a printed boss or standoff behind its panel, with an M2 × 3 mm heat-set insert.
- Bonded (cyanoacrylate): the eye domes, the soles, the visor window and the tail tip.
