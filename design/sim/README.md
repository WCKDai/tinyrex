# TinyREX in MuJoCo

`scene.xml` loads `tinyrex.xml`, TinyREX v4 as a MuJoCo model:
- 18 XL330 actuators: Microduck's 14, plus jaw, tail and two shoulders;
- TinyREX's own meshes (`meshes/`), one per body;
- per-body mass, centre of mass and full inertia integrated from every part;
- the collision-free joint limits;
- box contacts for the soles and toes;
- a `HOME` keyframe, TinyREX's standing pose.

## Rebuilding

The model is generated, not hand-written.

1. In the open Blender scene, run `design/blender/export_sim.py` after the scene scripts. It writes
   `tinyrex_bodies.json` and `meshes/`.
2. Run `python build_mjcf.py <microduck_rl>/src/mjlab_microduck/robot/microduck/robot_walk.xml`. Microduck's MJCF is
   Apache-2.0; `../../CREDITS.md` lists what is kept and what is changed.
3. Run `python stand_test.py <same robot_walk.xml>`. It needs `pip install mujoco` and the Microduck model files,
   locally, for the comparison.

## Can it stand? (`stand_test.py`)

The test runs each robot in its HOME pose on a floor, under servo position control alone (no walking policy).

### Why a stiff hold

Microduck's MJCF servo model is the soft one identified for RL training: 0.55 N·m/rad. Under
standing loads its joints sag 0.1–0.3 rad, and its walking policy compensates all the time. On fixed
targets with that model neither robot stands, Microduck included. So the test holds the pose stiffly
(10 N·m/rad), within the XL330's real force limit (0.96 N·m), for both robots alike. It asks two
things: can the hardware hold the pose, and how robust is the balance. It does not show how a policy
behaves.

| | Microduck (its own model) | TinyREX |
|---|---|---|
| Mass | 0.737 kg | 1.002 kg |
| Settles in 3 s | sags 0.7 mm, tilts 0.3° | sags 1.6 mm, tilts 0.6° |
| Peak servo effort (of 0.96 N·m) | knees 7 %, hip roll 4 % | hip roll 19 %, knees 10 % |
| Largest push survived, forward | 0.28 m/s | 0.52 m/s |
| Largest push survived, back | 0.18 m/s | 0.29 m/s |
| Largest push survived, sideways | 0.52 m/s | 0.97 m/s |

A push is a 0.1 s shove on the trunk, given as the change in the whole robot's velocity. "Survived"
means the trunk is within 35° of upright 2 s later.

TinyREX stands and survives larger pushes than Microduck in every direction. This comes from:
- its centre of mass sitting over the ankles;
- its longer feet;
- its heavier, lower tail.

### The hip-roll figure

TinyREX's hip roll effort (19 %, 0.18 N·m) is higher than its single-leg static estimate would
suggest for double support. The likely cause is that both feet are flat boxes, pressed down at
HOME's ±5° hip roll by stiff servos, so part of that load is the legs squeezing against each other.
This should be checked on hardware, and with the soft servo model once a policy is trained.

## Walking and getting up

`mjpython play.py` opens the MuJoCo viewer with the trained controllers.
- TinyREX walks, balancing with its tail.
- Knocked over, it works out how it landed, gets back up and walks on.
- Click the window, then: W walk or stand, P a sideways shove, T a big shove (it tumbles), R reset.

These are optimised controllers (CMA-ES), not neural networks. `train.py` trains them:

| Command | Trains | Scored on |
|---|---|---|
| `train.py walk` | `gait.py`'s step cycle (period, lift, stride, sway, lean, width), plus IMU feedback: the tail swings against roll; hip roll and ankle pitch correct a topple | distance over four 20 s walks with random shoves, minus drift and falls |
| `train.py bank` | records real falls (walking, shoved hard from random directions) into `fall_bank.npz` | — |
| `train.py getup` | face down and on its back: four whole-body poses and their timings, then home | ending upright and full height, from a clean drop and 8 recorded falls |
| `train.py roll` | each side landing: six poses that rock it onto its belly or back, followed by the get-up for that | standing at the end, from the same starts |

Results, with the stiff servo hold above:

| | |
|---|---|
| Walking speed | 0.2 m/s (4.0–4.2 m over three unseen 20 s runs with shoves, no falls) |
| Gait | 94 % single support, feet slip about 1.7 cm/s while planted, tilt under 14° |
| Get up in one try, from recorded falls | face down 18/27, left side 18/41, right side 11/41, on its back 0/12 |
| Walking again after 16 unseen hard shoves, with retries | 11/16 |

### Limits

- The walk is a fast shuffle: about 5.7 steps a second with roughly 4 cm strides. Real XL330s may not keep up.
- From its back, a fixed pose sequence can't cope with how variously the tail and backpack leave it. A get-up
  that reacts to how it's lying needs a learned feedback policy.

## Next

Retrain Microduck's walking policy (`microduck_rl`, mjlab) on this model. The policy needs:
- its new mass and inertia;
- the `HOME` pose;
- the new joint limits;
- the four extra joints, held at their defaults while it walks.
