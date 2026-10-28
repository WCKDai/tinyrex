"""Can it stand? TinyREX against Microduck, in MuJoCo, under the servos' position control alone
(no walking policy): each robot in its HOME pose on a floor.

Microduck's MJCF servo model is the soft one identified for RL training (kp 0.55 N m/rad): under
standing loads a joint sags 0.2-0.4 rad, and its walking policy compensates continuously - with
fixed targets neither robot stands on it (Microduck included). So this test holds the pose
stiffly (HOLD_KP) within the XL330's real force limit (0.96 N m), for both robots alike: it asks
whether the hardware can hold the pose and how robust the balance is, not how the policy behaves.

    python stand_test.py MICRODUCK_ROBOT_WALK_XML

For each robot: settle for 3 s and report the trunk's sag and tilt and each actuator's peak effort
(fraction of its force limit); then the largest push it survives - a 0.1 s shove on the trunk,
forward, back and sideways, sized as a change of the whole robot's momentum (m/s) - found by
bisection; 'survives' = trunk tilt under 35 degrees 2 s later."""
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MICRODUCK_STAND = [0, -0.08726646259971647, -0.457924, -0.004940, 0.452984, 0.3490658503988659, 0.3490658503988659, 0, 0,
                   0, 0.08726646259971647, 0.457924, 0.004940, -0.452984]  # its scene.xml STAND keyframe


def microduck():
    spec = mujoco.MjSpec.from_file(sys.argv[1])
    spec.option.timestep = 0.002
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05])
    m = spec.compile()
    d = mujoco.MjData(m)
    d.qpos[:7] = [0, 0, 0.12, 1, 0, 0, 0]
    d.qpos[7:] = MICRODUCK_STAND
    d.ctrl[:] = MICRODUCK_STAND
    mujoco.mj_forward(m, d)
    # stand it on the floor: lowest foot collision geom to z = 0
    feet = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").endswith("foot_collision")]
    low = min(lowest_point(m, d, g) for g in feet)
    d.qpos[2] -= low - 0.0002
    return m, d.qpos.copy(), d.ctrl.copy()


def tinyrex():
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
    k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "HOME")
    return m, m.key_qpos[k].copy(), m.key_ctrl[k].copy()


def lowest_point(m, d, g):
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        a, n = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
        v = m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        return v[:, 2].min()
    R = d.geom_xmat[g].reshape(3, 3)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * m.geom_size[g]
    return (corners @ R.T + d.geom_xpos[g])[:, 2].min()


def tilt(d):
    w = d.qpos[3]
    return math.degrees(2 * math.acos(min(1.0, abs(w))))  # (an upright start: the trunk's angle from it)


HOLD_KP, HOLD_KV = 10.0, 0.3


def stiffen(m):
    m.actuator_gainprm[:, 0] = HOLD_KP
    m.actuator_biasprm[:, 1] = -HOLD_KP
    m.actuator_biasprm[:, 2] = -HOLD_KV
    return m


def run(m, qpos, ctrl, push=None, t_end=3.0):
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    d.ctrl[:] = ctrl
    mujoco.mj_forward(m, d)
    z0 = d.qpos[2]
    mass = m.body_subtreemass[1]
    peak = np.zeros(m.nu)
    worst = 0.0
    while d.time < t_end:
        if push and 1.0 <= d.time < 1.1:
            d.xfrc_applied[1, :3] = np.array(push) * mass / 0.1  # (dv m/s over 0.1 s, on the trunk)
        else:
            d.xfrc_applied[1, :3] = 0
        mujoco.mj_step(m, d)
        lim = np.maximum(np.abs(m.actuator_forcerange[:, 1]), 1e-9)
        peak = np.maximum(peak, np.abs(d.actuator_force) / lim)
        worst = max(worst, tilt(d))
    return {"sag_mm": (z0 - d.qpos[2]) * 1000, "tilt_deg": tilt(d), "worst_tilt": worst, "peak": peak, "d": d}


def survives(m, qpos, ctrl, direction):
    r = run(m, qpos, ctrl, push=direction, t_end=3.0)
    return r["tilt_deg"] < 35


def max_push(m, qpos, ctrl, unit):
    lo, hi = 0.0, 1.0
    for _ in range(9):
        mid = (lo + hi) / 2
        if survives(m, qpos, ctrl, [u * mid for u in unit]):
            lo = mid
        else:
            hi = mid
    return lo


for name, (m, qpos, ctrl) in (("microduck", microduck()), ("tinyrex", tinyrex())):
    soft_load = None
    stiffen(m)
    r = run(m, qpos, ctrl)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]
    top = sorted(zip(names, r["peak"]), key=lambda t: -t[1])[:4]
    print(f"{name}: mass {m.body_subtreemass[1]:.3f} kg; settled 3 s: sag {r['sag_mm']:.1f} mm, tilt {r['tilt_deg']:.1f} deg;"
          f" peak effort (of 0.96 N m) " + ", ".join(f"{n} {p:.0%}" for n, p in top))
    torque = r["d"].actuator_force
    print("  at the soft RL servo model (0.55 N m/rad) these holding torques would sag the joints by: "
          + ", ".join(f"{n} {abs(t) / 0.55:.2f} rad" for n, t in sorted(zip(names, torque), key=lambda t: -abs(t[1]))[:4]))
    pushes = {k: max_push(m, qpos, ctrl, u) for k, u in (("forward", (1, 0, 0)), ("back", (-1, 0, 0)), ("side", (0, 1, 0)))}
    print(f"  largest push survived (m/s of the whole robot): " + ", ".join(f"{k} {v:.3f}" for k, v in pushes.items()))
