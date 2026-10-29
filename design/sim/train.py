"""Train TinyREX's controllers in MuJoCo with CMA-ES (a derivative-free optimiser): a walking gait
that balances with its tail and legs, and a get-up motion for each way it can land.

    python train.py walk  [--workers N] [--iters N]    -> walk_params.json
    python train.py getup [--workers N] [--iters N]    -> getup_params.json

These are optimised controllers, not neural-network policies:
  walk   the stepping pattern of gait.py (lift, stride, sway, lean, width, rate) plus feedback from
         the trunk's tilt (the IMU): the tail swings and the hips and ankles correct against a
         topple. Scored on forward speed over 10 s walks with random shoves, minus drift and falls.
  getup  for each landing (face down, on its back, on either side), a sequence of four whole-body
         poses (legs, arms, neck, tail) and their timings, then home. Scored on ending upright and
         standing at full height.
The servos hold stiffly within the XL330's force limit, as in stand_test.py."""
import json
import math
import multiprocessing as mp
import os
import sys
import time

import cma
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gait import FREE_SPEED, Gait, load, tilt  # noqa: E402

# ---------------------------------------------------------------- walking
WALK = [  # name, low, high
    ("period", 0.35, 1.2), ("lift", 0.006, 0.03), ("stride", 0.0, 0.08), ("sway", 0.0, 0.025),
    ("lean", -0.02, 0.02), ("width", -0.008, 0.012), ("sway_phase", -0.8, 0.8), ("tail", 0.0, 0.6),
    ("k_tail_roll", -4.0, 4.0), ("k_tail_rate", -0.6, 0.6), ("k_hip_roll", -1.5, 1.5),
    ("k_ankle_pitch", -1.5, 1.5), ("k_ankle_rate", -0.3, 0.3),
]


def unpack(x, spec):
    x = np.clip(x, 0, 1)
    return {n: lo + (hi - lo) * float(v) for (n, lo, hi), v in zip(spec, x)}


def trunk_state(d):
    """Roll, pitch (rad) of the trunk and its body-frame angular rate."""
    w, x, y, z = d.qpos[3:7]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    return roll, pitch, d.qvel[3:6]


class Walker(Gait):
    """gait.Gait with feedback: tail, hip roll and ankle pitch react to the trunk's tilt."""

    def __init__(self, m, key, p):
        super().__init__(m, key, p)
        self.base = {s: self.home_pos[s].copy() for s in self.home_pos}
        for s, sy in (("left", 1), ("right", -1)):
            self.home_pos[s] = self.base[s] + np.array([p["lean"], sy * p["width"], 0.0])

    def ctrl_fb(self, t, d):
        p = self.p
        q = dict(p)
        q["sway"] = p["sway"]
        ctrl = super().ctrl(t + p["sway_phase"] * p["period"] / (2 * math.pi) * 0)  # (phase handled below)
        roll, pitch, w = trunk_state(d)
        a = self.aid
        for n in ("left_hip_roll", "right_hip_roll"):
            ctrl[a[n]] += p["k_hip_roll"] * roll
        u = p["k_ankle_pitch"] * pitch + p["k_ankle_rate"] * w[1]
        ctrl[a["left_ankle"]] += u
        ctrl[a["right_ankle"]] -= u
        phi = 2 * math.pi * t / p["period"]
        ctrl[a["tail_yaw"]] = -p["tail"] * math.sin(phi + p["sway_phase"]) + p["k_tail_roll"] * roll + p["k_tail_rate"] * w[0]
        lo, hi = self.m.jnt_range[self.m.actuator_trnid[a["tail_yaw"]][0]]
        ctrl[a["tail_yaw"]] = min(max(ctrl[a["tail_yaw"]], lo), hi)
        return ctrl


def walk_episode(m, key, p, seed, secs=10.0):
    rng = np.random.default_rng(seed)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    g = Walker(m, key, p)
    mass = m.body_subtreemass[1]
    pushes = [(float(rng.uniform(2.5, 4.5)), rng.normal(0, 0.12, 3) * [0.6, 1, 0]), (float(rng.uniform(6, 8)), rng.normal(0, 0.12, 3) * [0.6, 1, 0])]
    x0 = d.qpos[0]
    dofs = [m.jnt_dofadr[m.actuator_trnid[a][0]] for a in range(m.nu)]
    fast, n = 0, 0  # (time with a joint above 75 % of the servo's no-load speed: no headroom left)
    while d.time < secs:
        if int(round(d.time / m.opt.timestep)) % 5 == 0:
            d.ctrl[:] = g.ctrl_fb(d.time - 0.5, d) if d.time > 0.5 else g.home_ctrl
        d.xfrc_applied[1, :3] = 0
        for t0, dv in pushes:
            if t0 <= d.time < t0 + 0.1:
                d.xfrc_applied[1, :3] = dv * mass / 0.1
        mujoco.mj_step(m, d)
        n += 1
        fast += np.abs(d.qvel[dofs]).max() > 0.75 * FREE_SPEED
        if tilt(d) > 45 or d.qpos[2] < 0.06:
            return (d.qpos[0] - x0) - 1.0 - 0.1 * (secs - d.time) - 2.0 * fast / n, True
    w, x, y, z = d.qpos[3:7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (d.qpos[0] - x0) - 0.5 * abs(d.qpos[1]) - 0.05 * abs(yaw) - 2.0 * fast / n, False


def walk_score(x):
    m, key = load()
    p = unpack(x, WALK)
    r = [walk_episode(m, key, p, s, secs=20.0) for s in (11, 22, 33, 44)]
    return -float(np.mean([v for v, _ in r]))


# ---------------------------------------------------------------- getting up
POSE_JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
               "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
               "neck_pitch", "head_pitch", "tail_yaw", "l_shoulder", "r_shoulder"]
KEYS = 4
LANDINGS = {"face_down": ("y", 90), "on_back": ("y", -90), "left_side": ("x", -90), "right_side": ("x", 90)}


BANK = os.path.join(HERE, "fall_bank.npz")
_bank = {}


def landing_of(d):
    """Which way the trunk lies: the trunk axis closest to pointing at the ground."""
    R = d.xmat[1].reshape(3, 3)
    x_down, y_down = -R[2, 0], -R[2, 1]
    if abs(x_down) >= abs(y_down):
        return "face_down" if x_down > 0 else "on_back"
    return "left_side" if y_down > 0 else "right_side"


def make_bank(m, key, walk, n=240):
    """Real falls: walking, shoved hard from a random direction, lying still 0.8 s (as play.py
    sees them). Saved per landing as (qpos, qvel)."""
    out = {l: [] for l in LANDINGS}
    mass = m.body_subtreemass[1]
    for seed in range(n):
        rng = np.random.default_rng(1000 + seed)
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, key)
        g = Walker(m, key, walk)
        a, v, t_push = rng.uniform(0, 2 * math.pi), rng.uniform(0.8, 1.5), rng.uniform(1.5, 3.5)
        down_since = None
        while d.time < t_push + 6:
            down = tilt(d) > 50
            if int(round(d.time / m.opt.timestep)) % 5 == 0:
                d.ctrl[:] = g.ctrl_fb(d.time - 0.5, d) if d.time > 0.5 and not down else g.home_ctrl
            d.xfrc_applied[1, :3] = [math.cos(a) * v * mass / 0.1, math.sin(a) * v * mass / 0.1, 0] if t_push <= d.time < t_push + 0.1 else 0
            mujoco.mj_step(m, d)
            down_since = (down_since or d.time) if down else None
            if down_since and d.time - down_since > 0.8:
                out[landing_of(d)].append(np.concatenate([d.qpos, d.qvel]))
                break
    np.savez(BANK, **{l: np.array(v) for l, v in out.items() if v})
    return {l: len(v) for l, v in out.items()}


def starts(landing, k=8):
    """The clean drop plus up to k real falls for this landing."""
    if not _bank and os.path.exists(BANK):
        _bank.update(dict(np.load(BANK)))
    real = _bank.get(landing, np.zeros((0, 1)))
    return [None] + [real[i] for i in np.linspace(0, len(real) - 1, min(k, len(real))).astype(int)] if len(real) else [None]


def fallen(m, key, landing, start=None):
    """The robot dropped from standing onto a side, settled 1.5 s; or a recorded real fall."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    if start is not None:
        d.qpos[:], d.qvel[:] = start[:m.nq], start[m.nq:]
        d.ctrl[:] = m.key_ctrl[key]
        mujoco.mj_forward(m, d)
        return d
    axis, deg = LANDINGS[landing]
    a = math.radians(deg) / 2
    d.qpos[3:7] = [math.cos(a), math.sin(a) if axis == "x" else 0, math.sin(a) if axis == "y" else 0, 0]
    d.qpos[2] = 0.16
    while d.time < 1.5:
        mujoco.mj_step(m, d)
    return d


SIDES = ("left_side", "right_side")  # (these roll onto the belly, then get up as face_down)


def keyposes(m, key, x):
    """A get-up's flat parameters as (pose, duration) keyframes."""
    home = m.key_ctrl[key].copy()
    aid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]): a for a in range(m.nu)}
    x = np.clip(np.array(x), 0, 1)
    out = []
    for k in range(len(x) // (len(POSE_JOINTS) + 1)):
        seg = x[k * (len(POSE_JOINTS) + 1):(k + 1) * (len(POSE_JOINTS) + 1)]
        pose = home.copy()
        for n, v in zip(POSE_JOINTS, seg[:-1]):
            lo, hi = m.jnt_range[m.actuator_trnid[aid[n]][0]]
            pose[aid[n]] = lo + (hi - lo) * v
        out.append((pose, 0.2 + 0.8 * seg[-1]))
    return out


def plan(m, key, x, then=None):
    """The full get-up: its own keyframes, then (for a side landing) the face-down get-up, then home."""
    steps = keyposes(m, key, x)
    if then is not None:
        steps += keyposes(m, key, then)
    return steps + [(m.key_ctrl[key].copy(), 0.6)]


def getup_episode(m, key, x, landing, record=None, then=None, start=None):
    d = fallen(m, key, landing, start)
    home = m.key_ctrl[key].copy()
    steps = plan(m, key, x, then)
    poses, durs = [p for p, _ in steps], [t for _, t in steps]
    t0, start = d.time, d.ctrl.copy()
    start[:] = home
    prev = start
    for pose, dur in zip(poses, durs):
        t_seg = d.time
        while d.time < t_seg + dur:
            f = (d.time - t_seg) / dur
            d.ctrl[:] = prev + (pose - prev) * f
            mujoco.mj_step(m, d)
            if record is not None and int(round(d.time / m.opt.timestep)) % 10 == 0:
                record.append(d.qpos.copy())
        prev = pose
    up, h = [], []
    t_end = d.time + 1.5
    while d.time < t_end:
        d.ctrl[:] = home
        mujoco.mj_step(m, d)
        up.append(math.cos(math.radians(tilt(d))))
        h.append(d.qpos[2])
    z_home = m.key_qpos[key][2]
    score = 3.0 * float(np.mean(up[-250:])) + min(1.0, float(np.mean(h[-250:])) / z_home)
    return score, tilt(d) < 20 and d.qpos[2] > 0.85 * z_home


def roll_episode(m, key, x, landing, getups=None, start=None):
    """From a side landing: a few poses that roll it onto its belly or back, then (given the trained
    getups) the get-up for whichever way it ended up. Scored on standing at the end, with credit for
    how far it rocked off its side and for where the roll left it."""
    d = fallen(m, key, landing, start)
    home = m.key_ctrl[key].copy()
    prev, peak = home, 0.0
    for pose, dur in plan(m, key, x):
        t_seg = d.time
        while d.time < t_seg + dur:
            d.ctrl[:] = prev + (pose - prev) * ((d.time - t_seg) / dur)
            mujoco.mj_step(m, d)
            peak = max(peak, 1 - abs(d.xmat[1][7]))  # (how far it has rocked off its side)
        prev = pose
    t_end = d.time + 1.0
    while d.time < t_end:
        d.ctrl[:] = home
        mujoco.mj_step(m, d)
    R = d.xmat[1].reshape(3, 3)
    rolled = abs(R[2, 0]) - 0.5 * abs(R[2, 1]) + 0.5 * peak
    if getups is None:
        return rolled, abs(R[2, 0]) > 0.7
    nxt = "face_down" if -R[2, 0] > 0 else "on_back"
    prev = home
    for pose, dur in plan(m, key, getups[nxt]["x"]):
        t_seg = d.time
        while d.time < t_seg + dur:
            d.ctrl[:] = prev + (pose - prev) * ((d.time - t_seg) / dur)
            mujoco.mj_step(m, d)
        prev = pose
    up, h = [], []
    t_end = d.time + 1.5
    while d.time < t_end:
        d.ctrl[:] = home
        mujoco.mj_step(m, d)
        up.append(math.cos(math.radians(tilt(d))))
        h.append(d.qpos[2])
    z_home = m.key_qpos[key][2]
    stood = 3.0 * float(np.mean(up[-250:])) + min(1.0, float(np.mean(h[-250:])) / z_home)
    return rolled + stood, tilt(d) < 20 and d.qpos[2] > 0.85 * z_home


def roll_score(args):
    x, (landing, getups) = args
    m, key = load()
    return -float(np.mean([roll_episode(m, key, x, landing, getups, s)[0] for s in starts(landing)]))


def getup_score(args):
    x, (landing, then) = args
    m, key = load()
    return -float(np.mean([getup_episode(m, key, x, landing, then=then, start=s)[0] for s in starts(landing)]))


# ---------------------------------------------------------------- driver
def optimise(name, n, f, x0, sigma, iters, workers, extra=None):
    es = cma.CMAEvolutionStrategy(x0, sigma, {"bounds": [0, 1], "popsize": max(16, workers * 2), "maxiter": iters, "verbose": -9, "seed": 7})
    t = time.time()
    with mp.Pool(workers) as pool:
        while not es.stop():
            xs = es.ask()
            vals = pool.map(f, [x if extra is None else (x, extra) for x in xs])
            es.tell(xs, vals)
            print(f"{name} gen {es.countiter} best {-es.result.fbest:.4f} mean {-np.mean(vals):.4f} ({time.time() - t:.0f}s)", flush=True)
    return es.result.xbest, -es.result.fbest


def main():
    what = sys.argv[1]
    workers = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else max(1, mp.cpu_count() - 1)
    iters = int(sys.argv[sys.argv.index("--iters") + 1]) if "--iters" in sys.argv else 60
    if what == "walk":
        prev = os.path.join(HERE, "walk_params.json")
        if os.path.exists(prev):  # (refine the last result)
            start, sigma = json.load(open(prev))["params"], 0.2
        else:
            start, sigma = json.load(open(os.path.join(HERE, "gait_params.json"))), 0.25
            start.update({"lean": 0.0, "width": 0.0, "sway_phase": 0.0, "k_tail_roll": 0.0, "k_tail_rate": 0.0, "k_hip_roll": 0.0, "k_ankle_pitch": 0.0, "k_ankle_rate": 0.0})
        x0 = [min(1, max(0, (start[n] - lo) / (hi - lo))) for n, lo, hi in WALK]
        xb, fb = optimise("walk", len(WALK), walk_score, x0, sigma, iters, workers)
        p = unpack(xb, WALK)
        m, key = load()
        check = [walk_episode(m, key, p, s, secs=20.0) for s in (101, 202, 303)]
        json.dump({"params": p, "score": fb, "check_20s": [[round(v, 3), fell] for v, fell in check]}, open(os.path.join(HERE, "walk_params.json"), "w"), indent=1)
        print("walk done", p, check)
    elif what == "bank":
        m, key = load()
        print(make_bank(m, key, json.load(open(os.path.join(HERE, "walk_params.json")))["params"], n=iters))
    elif what == "roll":  # (the side landings: roll over, then the face-down or on-back get-up)
        path = os.path.join(HERE, "getup_params.json")
        out = json.load(open(path))
        dims = 6 * (len(POSE_JOINTS) + 1)  # (enough poses to rock back and forth into a roll)
        for landing in SIDES:
            fronts = {n: out[n] for n in ("face_down", "on_back")}
            x0 = out[landing]["x"] if landing in out and len(out[landing]["x"]) == dims else [0.5] * dims
            xb, fb = optimise(f"roll {landing}", dims, roll_score, x0, 0.25, iters, workers, extra=(landing, fronts))
            m, key = load()
            res = [roll_episode(m, key, xb, landing, fronts, st) for st in starts(landing, 40)]
            score, ok = float(np.mean([r[0] for r in res])), f"{sum(r[1] for r in res)}/{len(res)}"
            out[landing] = {"x": [float(v) for v in xb], "then": None, "roll": True, "score": score, "up": ok}
            json.dump(out, open(path, "w"), indent=1)
            print("roll", landing, score, ok, flush=True)
    elif what == "getup":
        path = os.path.join(HERE, "getup_params.json")
        old = json.load(open(path)) if os.path.exists(path) else {}
        out = dict(old)  # (keeps the side rolls)
        for landing in ("face_down", "on_back"):  # (the sides: train.py roll, afterwards)
            then = out["face_down"]["x"] if landing in SIDES else None
            keys = 2 if landing in SIDES else KEYS
            dims = keys * (len(POSE_JOINTS) + 1)
            x0 = old[landing]["x"] if landing in old and len(old[landing]["x"]) == dims else [0.5] * dims
            xb, fb = optimise(f"getup {landing}", dims, getup_score, x0, 0.2, iters, workers, extra=(landing, then))
            m, key = load()
            res = [getup_episode(m, key, xb, landing, then=then, start=st) for st in starts(landing, 40)]
            score, ok = float(np.mean([r[0] for r in res])), f"{sum(r[1] for r in res)}/{len(res)}"
            out[landing] = {"x": [float(v) for v in xb], "then": "face_down" if then is not None else None, "score": score, "up": ok}
            json.dump(out, open(os.path.join(HERE, "getup_params.json"), "w"), indent=1)
            print("getup", landing, score, ok, flush=True)


if __name__ == "__main__":
    main()
