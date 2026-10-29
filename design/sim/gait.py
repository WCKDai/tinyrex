"""A scripted walking gait for TinyREX in MuJoCo - not a learned policy (Microduck's walk is an RL
policy; retraining it for TinyREX is the real job). This one: each foot follows a step cycle
(lift, stride) in the trunk's frame while both feet sway sideways to put the trunk over the
stance foot; inverse kinematics (MuJoCo Jacobians, the trunk held) turns that into leg joint targets
for the position servos. The tail wags against the sway.

    python gait.py tune         search the gait's parameters headless (walks the furthest, stays up)
    mjpython gait.py view       the interactive viewer: W walk / stop, P push, T tumble (big shove),
                                R reset, G pick it back up
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "scene.xml")
PARAMS = os.path.join(HERE, "gait_params.json")
HOLD_KP, HOLD_KV = 10.0, 0.3  # (stiff position hold within the XL330's 0.96 N m - see stand_test.py)
LEGS = {"left": ["left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle"],
        "right": ["right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]}


def load():
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.actuator_gainprm[:, 0] = HOLD_KP
    m.actuator_biasprm[:, 1] = -HOLD_KP
    m.actuator_biasprm[:, 2] = -HOLD_KV
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "HOME")
    return m, key


class Gait:
    def __init__(self, m, key, p):
        self.m, self.p = m, p
        self.kin = mujoco.MjData(m)  # kinematics only: the trunk held at the origin
        self.kin.qpos[:] = m.key_qpos[key]
        self.kin.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_kinematics(m, self.kin)
        self.jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for leg in LEGS.values() for n in leg}
        self.aid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]): a for a in range(m.nu)}
        self.site = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s}_foot") for s in LEGS}
        self.home_pos = {s: self.kin.site_xpos[self.site[s]].copy() for s in LEGS}
        self.home_rot = {s: self.kin.site_xmat[self.site[s]].reshape(3, 3).copy() for s in LEGS}
        self.home_ctrl = m.key_ctrl[key].copy()

    def ik(self, side, target, iters=6):
        m, d = self.m, self.kin
        dofs = [m.jnt_dofadr[self.jid[n]] for n in LEGS[side]]
        qadr = [m.jnt_qposadr[self.jid[n]] for n in LEGS[side]]
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            s = self.site[side]
            err_p = target - d.site_xpos[s]
            R = d.site_xmat[s].reshape(3, 3)
            dR = self.home_rot[side] @ R.T
            err_r = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
            mujoco.mj_jacSite(m, d, jp, jr, s)
            J = np.vstack([jp[:, dofs], 0.05 * jr[:, dofs]])  # (feet flat, weighted against position)
            e = np.concatenate([err_p, 0.05 * err_r])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
            for a, v in zip(qadr, dq):
                d.qpos[a] += v
        return {n: d.qpos[a] for n, a in zip(LEGS[side], qadr)}

    def ctrl(self, t):
        p = self.p
        phi = 2 * math.pi * t / p["period"]
        ctrl = self.home_ctrl.copy()
        sway = p["sway"] * math.sin(phi)
        for side, shift in (("left", 0.0), ("right", math.pi)):
            ph = (phi + shift) % (2 * math.pi)
            if ph < math.pi:  # swing: lift and carry forward
                z = p["lift"] * math.sin(ph)
                x = -p["stride"] / 2 + p["stride"] * ph / math.pi
            else:  # stance: push back
                z = 0.0
                x = p["stride"] / 2 - p["stride"] * (ph - math.pi) / math.pi
            target = self.home_pos[side] + np.array([x, sway, z])
            for n, v in self.ik(side, target).items():
                ctrl[self.aid[n]] = v
        if "tail_yaw" in self.aid:
            ctrl[self.aid["tail_yaw"]] = -p.get("tail", 0.3) * math.sin(phi)
        return ctrl


def tilt(d):
    return math.degrees(2 * math.acos(min(1.0, abs(d.qpos[3]))))


def trial(m, key, p, secs=8.0):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    g = Gait(m, key, p)
    x0 = d.qpos[0]
    worst = 0.0
    while d.time < secs:
        if int(d.time / m.opt.timestep) % 5 == 0:  # (targets at 100 Hz)
            d.ctrl[:] = g.ctrl(max(0.0, d.time - 0.5)) if d.time > 0.5 else g.home_ctrl
        mujoco.mj_step(m, d)
        worst = max(worst, tilt(d))
        if worst > 40:
            return {"forward_mm": (d.qpos[0] - x0) * 1000, "fell": True, "worst_tilt": worst, "t": d.time}
    return {"forward_mm": (d.qpos[0] - x0) * 1000, "fell": False, "worst_tilt": worst, "drift_y_mm": d.qpos[1] * 1000}


def tune():
    m, key = load()
    best = None
    rng = np.random.default_rng(1)
    for i in range(60):
        p = {"period": float(rng.uniform(0.6, 1.4)), "lift": float(rng.uniform(0.008, 0.022)),
             "stride": float(rng.uniform(0.0, 0.03)), "sway": float(rng.uniform(0.004, 0.02)), "tail": 0.3}
        r = trial(m, key, p)
        score = r["forward_mm"] - (1000 if r["fell"] else 0) - 2 * r["worst_tilt"]
        if best is None or score > best[0]:
            best = (score, p, r)
            print(i, round(score, 1), {k: round(v, 4) for k, v in p.items()}, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
    json.dump(best[1], open(PARAMS, "w"), indent=1)
    print("best", best)


def view():
    import mujoco.viewer
    m, key = load()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    p = json.load(open(PARAMS)) if os.path.exists(PARAMS) else {"period": 1.0, "lift": 0.015, "stride": 0.015, "sway": 0.01, "tail": 0.3}
    g = Gait(m, key, p)
    state = {"walk": True, "t0": 0.5, "push": None}

    def key_cb(k):
        c = chr(k) if k < 256 else ""
        if c == "W":
            state["walk"] = not state["walk"]
            state["t0"] = d.time
        elif c == "P":
            state["push"] = (d.time, np.array([0.0, 0.25, 0.0]))   # a sideways nudge (m/s)
        elif c == "T":
            state["push"] = (d.time, np.array([0.9, 0.6, 0.0]))    # a shove that tumbles it
        elif c in ("R", "G"):
            keep = d.qpos[:2].copy() if c == "G" else None
            mujoco.mj_resetDataKeyframe(m, d, key)
            if keep is not None:
                d.qpos[:2] = keep
            state["t0"] = d.time + 0.5

    with mujoco.viewer.launch_passive(m, d, key_callback=key_cb) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.9, -15, 135
        import time
        mass = m.body_subtreemass[1]
        while v.is_running():
            start = time.time()
            for _ in range(int(0.02 / m.opt.timestep)):
                if int(d.time / m.opt.timestep) % 5 == 0:
                    d.ctrl[:] = g.ctrl(d.time - state["t0"]) if state["walk"] and d.time > state["t0"] else g.home_ctrl
                d.xfrc_applied[1, :3] = 0
                if state["push"] and state["push"][0] <= d.time < state["push"][0] + 0.1:
                    d.xfrc_applied[1, :3] = state["push"][1] * mass / 0.1
                mujoco.mj_step(m, d)
            v.cam.lookat[:] = d.qpos[:3] + np.array([0, 0, 0.02])
            v.sync()
            time.sleep(max(0.0, 0.02 - (time.time() - start)))


if __name__ == "__main__":
    {"tune": tune, "view": view}[sys.argv[1] if len(sys.argv) > 1 else "view"]()
