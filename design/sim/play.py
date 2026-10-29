"""TinyREX in the MuJoCo viewer with its trained controllers (train.py): it walks, balancing with
its tail; knocked over, it works out how it landed and gets back up, then walks on.

    mjpython play.py

Keys (click the window first): W walk / stand, P a sideways shove, T a big shove (it tumbles),
R reset. Drag with Ctrl + double-click to push it about yourself."""
import json
import math
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gait import load, tilt  # noqa: E402
from train import Walker, landing_of, plan  # noqa: E402


def getup_plan(m, key, getups, landing):
    """The trained get-up as (pose, duration) steps ending at home."""
    g = getups[landing]
    then = getups[g["then"]]["x"] if g.get("then") else None
    return plan(m, key, g["x"], then) + [(m.key_ctrl[key].copy(), 1.0)]


def main():
    m, key = load()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    walk = json.load(open(os.path.join(HERE, "walk_params.json")))["params"]
    getups = json.load(open(os.path.join(HERE, "getup_params.json"))) if os.path.exists(os.path.join(HERE, "getup_params.json")) else {}
    g = Walker(m, key, walk)
    mass = m.body_subtreemass[1]
    st = {"mode": "walk", "t0": 0.5, "push": None, "down_since": None, "plan": None, "seg": 0, "seg_t": 0.0, "prev": None}

    def key_cb(k):
        c = chr(k) if k < 256 else ""
        if c == "W":
            st["mode"] = "stand" if st["mode"] == "walk" else "walk"
            st["t0"] = d.time
        elif c == "P":
            st["push"] = (d.time, np.array([0.0, 0.3, 0.0]))
        elif c == "T":
            st["push"] = (d.time, np.array([0.9, 0.7, 0.0]))
        elif c == "R":
            mujoco.mj_resetDataKeyframe(m, d, key)
            st.update(mode="walk", t0=0.5, plan=None, down_since=None)

    with mujoco.viewer.launch_passive(m, d, key_callback=key_cb) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.95, -15, 135
        while v.is_running():
            start = time.time()
            for _ in range(int(0.02 / m.opt.timestep)):
                step = int(round(d.time / m.opt.timestep))
                down = tilt(d) > 50
                if st["mode"] in ("walk", "stand") and down:
                    st["down_since"] = st["down_since"] or d.time
                    if d.time - st["down_since"] > 0.8 and getups:
                        landing = landing_of(d)
                        if landing in getups:
                            st.update(mode="getup", plan=getup_plan(m, key, getups, landing), seg=0, seg_t=d.time, prev=d.ctrl.copy())
                elif not down:
                    st["down_since"] = None
                if st["mode"] == "getup":
                    pose, dur = st["plan"][st["seg"]]
                    f = min(1.0, (d.time - st["seg_t"]) / dur)
                    d.ctrl[:] = st["prev"] + (pose - st["prev"]) * f
                    if f >= 1.0:
                        st["prev"], st["seg"], st["seg_t"] = pose, st["seg"] + 1, d.time
                        if st["seg"] >= len(st["plan"]):
                            st.update(mode="walk", t0=d.time, plan=None, down_since=None)
                elif step % 5 == 0:
                    if st["mode"] == "walk" and d.time > st["t0"] and not down:
                        d.ctrl[:] = g.ctrl_fb(d.time - st["t0"], d)
                    else:
                        d.ctrl[:] = g.home_ctrl
                d.xfrc_applied[1, :3] = 0
                if st["push"] and st["push"][0] <= d.time < st["push"][0] + 0.1:
                    d.xfrc_applied[1, :3] = st["push"][1] * mass / 0.1
                mujoco.mj_step(m, d)
            v.cam.lookat[:] = d.qpos[:3] + np.array([0, 0, 0.02])
            v.sync()
            time.sleep(max(0.0, 0.02 - (time.time() - start)))


if __name__ == "__main__":
    main()
