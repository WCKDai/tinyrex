# Credits

TinyRex stands on the shoulders of two small walking robots. We borrowed their ideas, not their files.

## Microduck — Pollen Robotics / Hugging Face

[Microduck](https://pollen-robotics.com/microduck) is a 25 cm, roughly 800 g biped that walks on reinforcement-learning
policies. It showed that a robot this small can be serious engineering and still make people smile, and TinyRex
borrows that idea: a small, characterful biped with a big personality, built to be learned from.

- Robot software: [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) (Apache-2.0)
- Policy training (MuJoCo, PPO, sim2real): [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl)
  (Apache-2.0 code; its 3D model files are CC BY-NC-SA 4.0)

**What TinyRex uses from Microduck:**

- **Its frame layout (the v4 design, `design/blender/`).** TinyRex places its own models of the
  servos, brackets and electronics where Microduck's joints and parts sit. Those placements are read
  from Microduck's MJCF (`robot_walk.xml`, Apache-2.0) at design time. Like Microduck, TinyRex uses
  Dynamixel XL330 servos.
- **Its MJCF, for simulation.** `design/sim/tinyrex.xml` is generated from that MJCF by
  `design/sim/build_mjcf.py`, which keeps Microduck's kinematic tree and its joint and actuator
  models. The script makes these changes: every body's geometry and inertia are replaced with
  TinyRex's own; the joint ranges are TinyRex's collision-free limits; jaw, tail and shoulder joints
  are added; the feet get new contacts.
- **Its HOME standing pose** (`microduck_constants.py`, Apache-2.0), as the starting point for
  TinyRex's.

The repository contains **no Microduck mesh**. Microduck's 3D model files (CC BY-NC-SA 4.0) were
used only locally, as a size and layout reference and to measure the shells TinyRex replaces. If you
combine TinyRex with those files, they keep their CC BY-NC-SA 4.0 licence.

## Open Duck Mini — Antoine Pirrone and contributors

[Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini) (Apache-2.0) is an open miniature of Disney's BDX droid,
sponsored by Hugging Face and Pollen Robotics. Earlier TinyRex designs followed its choice of parts: Feetech STS3215
serial servos and a Raspberry Pi Zero 2 W. The v4 design moved to Microduck's XL330 layout and keeps the Pi Zero 2 W. Open Duck Mini's runtime and MuJoCo Playground environments are the obvious place
to start for learned walking.

## The tools

- [Polykrom](https://github.com/WCKDai/polykrom): the parametric CAD the agent designed TinyRex in.
- [PolyCore](https://github.com/WCKDai/polycore): the Rust B-rep kernel under Polykrom.
- [MuJoCo](https://mujoco.org) (Apache-2.0): physics simulation for the standing and walking checks.
- Claude Code: the agent that ran each design stage, using Polykrom's MCP tools as its only tools.

Disney's BDX droid, Microduck and Open Duck Mini belong to their owners. Naming them here is attribution, not
endorsement.
