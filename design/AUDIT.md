# TinyREX v4 — model audit

A part-by-part check of the Blender model:
- what each part is and what it is made of;
- whether it can be printed, fixed and assembled;
- whether everything clears everything else in motion;
- how it compares with Microduck.

The checks are scripts, re-run after every change:

| Check | Script | What it does |
|---|---|---|
| Inventory | `blender/audit.py` | Role, material, size, watertightness, thinnest wall, fixings |
| Clips | `rig.py` `clips` / `part_check` | Every joint swept through its range against every part |
| Floating parts | `rig.py` `supports` | Each part must reach its body's hardware through parts it touches |
| Screws | `v4_parts.py` `part_bosses` | Rays along each screw: thread or insert behind it, or a boss is added |
| Limits | `limits.py` bisection | Collision-free joint ranges |
| Balance | `blender/mass.py` | Mass and balance |
| Simulation | `sim/stand_test.py` | Standing in MuJoCo |

Labelled renders: `renders/labelled-{hero,back,exploded}-annotated.png`.

## Inventory

The model has **237 parts**:
- 18 Dynamixel XL330 servos, each named by the joint it drives in the labelled renders;
- 23 frame brackets and 14 bearings, in Microduck's layout;
- 13 electronics parts and 3 cables (the camera CSI flex and the two arm-servo leads);
- 9 printed shells and 71 printed parts;
- 9 TPU parts (soles, toe soles, tail flex rings);
- 6 steel pins (jaw shafts, cheek pins, elbow pins), 2 magnets and the acrylic visor window;
- 78 M2 screws (including 8 that hold the jaw-hinge horn caps) and 22 printed bosses and standoffs for M2 heat-set
  inserts.

The XL330s' own horn and case screws aren't counted; they come with the servos.

The printed parts break down as:
- 2 head shells, the head back cap, muzzle panel and brow visor;
- 2 body halves, the backpack and the battery door;
- jaw, jaw arms and cheeks;
- arms;
- thigh shields, feet, toe caps and ankle guards;
- tail arm, segments and tip;
- collar, throat and nape plates;
- flank plates, spikes, eye sockets and brows;
- insert bosses.

## Found and fixed in this audit

| Finding | Fix |
|---|---|
| The body and the head were **single closed shells around their hardware**, so they could not be assembled | The body prints as **left and right halves**, as Microduck's shells do. Each half screws to the trunk plate on its own bosses, and the backpack and battery door bridge them. The lime head shell splits along its shoulder crease into a **head base**, screwed to the head frame through standoffs, and a **head top**, screwed down onto tabs. Microduck's head splits top and bottom the same way |
| The **battery door was buried inside the body**: it stayed at 56.8 mm when the pod floor moved down to 54 mm | A real opening in the floor, sized for the battery to come out, with the door as a cover plate under it. Two screws into standoffs on the battery holders at the front; the rear edge hooks under the floor's lip |
| **No fixing** for the head shell, the muzzle panel, the foot plates and ankle guards, or the hands | Head base: screws and standoffs to the head frame. Head top: screws onto tabs. Muzzle: screws. Foot plate: screws up into the ankle cradle, before the sole is bonded on. Ankle guards: screws into the cradle's sides. Hands: screws into the forearms |
| The elbow pin was a 0.8 mm cap, not a pin | A steel pin through both arm parts |
| **Two shin brackets per leg**: Microduck's MJCF places the shin bracket twice, as a visual and as a collision-only geom, and the builder copied both | The builder and the rig skip collision-only geoms, and the duplicates are deleted |
| The camera **visor's walls ended in slivers** (0.06–0.6 mm) at its floor cut | A full-width floor cut. The visor's walls now start clean |
| TailArm and the left eye socket and brow were **inside out** (inverted normals) | Every generated mesh is oriented outward when it is made |
| Screw bosses could land on the wrong thing: a camera lens, a 1 mm gap between two servos, an eye socket | The boss check uses a ring of rays at the boss's radius. Standoffs only land on frame brackets and servo cases. The affected screws moved |

## Checks, after the fixes

- **Clips:**
  - none at rest;
  - none within the collision-free joint limits (`blender/joint_limits.json`);
  - none in either standing pose, Microduck's HOME or TinyREX's.
- **Floating parts:** none. Every part reaches its body's hardware through parts it touches.
- **Fixings:** every printed part is either screwed down or printed or bonded with its host. Bonded parts are the spikes, the eye sockets and brows, the toe caps and soles, the visor window and the tail tip.
- **Screws:** every screw bites something:
  - a servo horn (arms);
  - an XL330 case screw hole (nape, throat, tail root);
  - a solid printed part (tail, thighs);
  - a printed boss or standoff for an M2 heat-set insert (22 bosses and standoffs).
- **Walls (3+ perimeters of a 0.4 mm nozzle):**
  - every printed part is at least 1.2 mm;
  - shells are 1.6–1.9 mm, thinnest where the 2 mm wall wraps a fillet;
  - plates are 3–9 mm;
  - TPU is 3.4–8 mm;
  - the eye domes are 1.2 mm acrylic.
- **Watertight meshes:** the six boolean-cut shells (the body halves, both head shells, the head back cap and the visor) are **not watertight as meshes**. Their cut edges leave open seams. Slicers repair this, and the exact STEP export rebuilds them from their construction, so it is unaffected.

## Does it look as polished as Microduck?

Not yet. Two things separate it.

1. **The frame is placeholder geometry.** The 23 frame brackets, in Microduck's positions, are plain boxes the size of
   Microduck's: hip, shin, ankle cradle, neck, head frame, trunk plate and battery holders. In the labelled renders
   they are the cream blocks at the hips, knees and ankles and the black neck stack. Microduck's brackets are real CAD
   parts: U-cradles around the servos, filleted links with bosses at the axes, lightening holes. This is the largest
   gap. It needs TinyREX's own bracket designs, made to the same servo and bearing positions.
2. **The surfaces are panelled, not sculpted.** TinyREX's shells are ruled lofts with fillets, which reads as a toy
   robot. That is the boards' language, and the boards ask for it. Microduck's shells are smooth freeform surfaces.
   The head dome, thigh shields and tail could take a smoother loft without losing the boards' hard-surface look.

## Open items

- TinyREX's own frame brackets, replacing the placeholders. This is the next design job.
- Watertight cut shells for printing straight from the Blender meshes.
- A PolyCore report for the failures the STEP export hit; see `tools/polycore-blender/README.md`.
