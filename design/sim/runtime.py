"""TinyREX's runtime: one 50 Hz loop for the simulator and the real robot, as Microduck's sim2real
has it - read the robot's state, pick the behaviour, write joint targets. Behaviours hot-swap behind
one observation (robot.Obs: joint angles and speeds, the IMU's gravity and rotation rate):

  stand     hold the standing pose (HOME)
  walk      the trained gait with IMU feedback (train.Walker; walk_params.json)
  getup     after a fall: the trained get-up for the way it landed (getup_params.json)
  emote     an expressive move (emotes.py): yes, no, excited, roar, chomp, curious, sad, wave

A fall (trunk tilted over 50 deg for 0.8 s) switches to getup from anything; getup goes back to
stand. Commands come from the keyboard (sim viewer) or a script.

    mjpython runtime.py sim            the viewer: W walk/stand, 1-8 emotes, P shove, T big shove, R reset
    python runtime.py rehearse         headless test: every behaviour, with sensor noise and a tick of
                                       latency, as on the real bus; prints PASS / FAIL per test
    python runtime.py real --port /dev/ttyUSB0 --check     the real robot: servos answer? within limits?
    python runtime.py real --port /dev/ttyUSB0 --home      torque on, ease to the standing pose
    python runtime.py real --port /dev/ttyUSB0             run (stand; keys from stdin: w, 1-8, q)"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import emotes  # noqa: E402
from robot import DT, Obs, SimRobot, landing  # noqa: E402

EMOTES = ["yes", "no", "excited", "roar", "chomp", "curious", "sad", "wave"]


class Brain:
    """The behaviours and the switching between them. Needs the MuJoCo model for kinematics (the
    walk's inverse kinematics and the get-up plans), on the robot as in the simulator."""

    def __init__(self, joints, home):
        from gait import load
        from train import Walker, plan
        self.m, self.key = load()
        self.joints, self.home = list(joints), np.asarray(home, float)
        self.idx = {j: i for i, j in enumerate(self.joints)}
        self.walker = Walker(self.m, self.key, json.load(open(os.path.join(HERE, "walk_params.json")))["params"])
        self.getups = json.load(open(os.path.join(HERE, "getup_params.json")))
        self.plan = plan
        self.mode, self.t_mode, self.down_since = "stand", 0.0, None
        self.emote, self.steps, self.seg, self.seg_t, self.prev = None, None, 0, 0.0, None
        self.log = []

    def command(self, c, t):
        """'walk', 'stand', or an emote's name."""
        if self.mode == "getup":
            return
        if c in ("walk", "stand"):
            self._switch(c, t)
        elif c in emotes.CLIPS:
            self.emote = c
            self._switch("emote", t)

    def _switch(self, mode, t):
        self.mode, self.t_mode = mode, t
        self.log.append((round(t, 2), mode, self.emote if mode == "emote" else None))

    def step(self, obs: Obs):
        t = obs.t
        down = obs.tilt_deg() > 50
        if self.mode != "getup":
            self.down_since = (self.down_since or t) if down else None
            stuck = obs.tilt_deg() > 25  # (half over - sat on the tail or an arm - for 2 s: get up too)
            self.stuck_since = (getattr(self, "stuck_since", None) or t) if stuck else None
            if (self.down_since is not None and t - self.down_since > 0.8) or (self.stuck_since is not None and t - self.stuck_since > 2.0):
                self.stuck_since = None
                g = self.getups[landing(obs)]
                then = self.getups[g["then"]]["x"] if g.get("then") else None
                self.steps = self.plan(self.m, self.key, g["x"], then) + [(self.home.copy(), 1.0)]
                self.seg, self.seg_t, self.prev = 0, t, np.asarray(obs.q, float).copy()
                self._switch("getup", t)
                self.log[-1] = self.log[-1][:2] + (landing(obs),)
        if self.mode == "getup":
            pose, dur = self.steps[self.seg]
            f = min(1.0, (t - self.seg_t) / dur)
            out = self.prev + (pose - self.prev) * f
            if f >= 1.0:
                self.prev, self.seg, self.seg_t = pose, self.seg + 1, t
                if self.seg >= len(self.steps):
                    self.down_since = None
                    self._switch("stand", t)  # (still down? the fall check starts the next try)
            return out
        if self.mode == "walk" and t - self.t_mode > 0.3:
            roll, pitch = obs.roll_pitch()
            return self.walker.ctrl_state(t - self.t_mode - 0.3, roll, pitch, obs.gyro)
        if self.mode == "emote":
            off = emotes.offsets(self.emote, t - self.t_mode)
            if off is None:
                self._switch("stand", t)
                return self.home.copy()
            out = self.home.copy()
            for j, v in off.items():
                out[self.idx[j]] += v
            return out
        return self.home.copy()


def run_sim():
    import time

    import mujoco.viewer
    robot = SimRobot()
    brain = Brain(robot.joints, robot.home)
    keys = {ord(str(i + 1)): e for i, e in enumerate(EMOTES)}

    def on_key(k):
        if k == ord("W"):
            brain.command("stand" if brain.mode == "walk" else "walk", robot.d.time)
        elif k in keys:
            brain.command(keys[k], robot.d.time)
        elif k == ord("P"):
            robot.shove((0.0, 0.3, 0.0))
        elif k == ord("T"):
            robot.shove((0.9, 0.7, 0.0))
        elif k == ord("R"):
            robot.reset()
            brain.mode, brain.down_since = "stand", None

    print("keys: W walk/stand, " + ", ".join(f"{i + 1} {e}" for i, e in enumerate(EMOTES)) + ", P shove, T big shove, R reset")
    with mujoco.viewer.launch_passive(robot.m, robot.d, key_callback=on_key) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.95, -15, 135
        while v.is_running():
            start = time.time()
            robot.write(brain.step(robot.read()))
            robot.tick()
            v.cam.lookat[:] = robot.d.qpos[:3] + np.array([0, 0, 0.02])
            v.sync()
            time.sleep(max(0.0, DT - (time.time() - start)))


def rehearse():
    """Every behaviour through the runtime loop, with sensor noise and one tick of latency."""
    results = []

    def session(name, script, secs, noise=1.0, latency=1, seed=0):
        robot = SimRobot(noise=noise, latency=latency, seed=seed)
        brain = Brain(robot.joints, robot.home)
        x0 = robot.d.qpos[0]
        worst, fell = 0.0, False
        while robot.d.time < secs:
            for at, what in script:
                if abs(robot.d.time - at) < DT / 2:
                    if what.startswith("shove"):
                        robot.shove([float(v) for v in what.split(":")[1].split(",")])
                    else:
                        brain.command(what, robot.d.time)
            obs = robot.read()
            robot.write(brain.step(obs))
            robot.tick()
            worst = max(worst, robot.read().tilt_deg())
            fell = fell or brain.mode == "getup"
        return dict(worst_tilt=worst, fell=fell, upright=robot.read().tilt_deg() < 25, moved=robot.d.qpos[0] - x0, log=brain.log)

    for e in EMOTES:
        r = session(e, [(0.5, e)], 0.5 + emotes.CLIPS[e][-1][0] + 1.0)
        results.append((f"emote {e}", not r["fell"] and r["worst_tilt"] < 15, f"worst tilt {r['worst_tilt']:.1f} deg"))
    r = session("walk", [(0.5, "walk")], 10.5)
    results.append(("walk 10 s", not r["fell"] and r["moved"] > 0.5, f"{r['moved']:.2f} m, worst tilt {r['worst_tilt']:.1f} deg"))
    r = session("walk + emote", [(0.5, "walk"), (5.0, "stand"), (6.0, "excited"), (8.5, "walk")], 13.0)
    results.append(("walk, stop, excited, walk on", not r["fell"], f"{r['moved']:.2f} m, worst tilt {r['worst_tilt']:.1f} deg"))
    r = session("small shove", [(0.5, "walk"), (3.0, "shove:0,0.3,0")], 8.0)
    results.append(("walking, 0.3 m/s side shove", not r["fell"], f"worst tilt {r['worst_tilt']:.1f} deg"))
    ups = 0
    for seed, v in enumerate(((0.9, 0.7, 0), (-0.9, 0.5, 0), (0.2, -1.2, 0), (-0.6, -0.9, 0))):
        r = session("tumble", [(0.5, "walk"), (2.0, f"shove:{v[0]},{v[1]},{v[2]}")], 26.0, seed=seed)
        ups += r["upright"]
        results.append((f"big shove {v[:2]} -> gets up", r["upright"], f"landings tried: {[l[2] for l in r['log'] if l[1] == 'getup']}"))
    w = max(len(n) for n, _, _ in results)
    for n, ok, note in results:
        print(f"{'PASS' if ok else 'FAIL'}  {n:{w}s}  {note}")
    print(f"{sum(ok for _, ok, _ in results)}/{len(results)} passed")
    return all(ok for _, ok, _ in results)


def run_real(args):
    from robot import DynamixelRobot
    port = args[args.index("--port") + 1] if "--port" in args else "/dev/ttyUSB0"
    robot = DynamixelRobot(port)
    if "--check" in args:
        bad = {j: r for j, r in robot.ping_all().items() if r[0] is None}
        print("servos answering:", len(robot.joints) - len(bad), "of", len(robot.joints), "missing:", list(bad))
        obs = robot.read()
        for j, q in zip(robot.joints, obs.q):
            lo, hi = robot.servo[j]["limits_rad"]
            print(f"  {j:16s} {math.degrees(q):7.1f} deg  {'OK' if lo - 0.05 <= q <= hi + 0.05 else 'OUTSIDE its limits'}")
        return
    robot.torque(True)
    start = robot.read().q
    if "--home" in args:  # (ease over 3 s)
        for i in range(150):
            robot.write(start + (robot.home - start) * (i + 1) / 150)
            robot.tick()
        return
    from imu import HeadIMU, LSM6DSV16X
    head = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"]
    robot.imu = HeadIMU(LSM6DSV16X(), head)
    brain = Brain(robot.joints, robot.home)
    import select
    print("running: w walk/stand, 1-8 emotes, q quit")
    try:
        while True:
            obs = robot.read()
            robot.imu.set_joints([obs.q[robot.joints.index(j)] for j in head])
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.readline().strip()
                if c == "q":
                    break
                if c == "w":
                    brain.command("stand" if brain.mode == "walk" else "walk", obs.t)
                elif c.isdigit() and 1 <= int(c) <= len(EMOTES):
                    brain.command(EMOTES[int(c) - 1], obs.t)
            robot.write(brain.step(obs))
            robot.tick()
    finally:
        robot.torque(False)


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "sim"
    {"sim": lambda: run_sim(), "rehearse": lambda: sys.exit(0 if rehearse() else 1), "real": lambda: run_real(sys.argv)}[what]()
