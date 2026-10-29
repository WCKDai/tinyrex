"""TinyREX's robot interface: one API for the MuJoCo model and the real robot, as Microduck's runtime
has it (one 50 Hz loop: read state, decide, write joint targets).

    robot.read()  -> Obs: joint angles and speeds (actuator order), the IMU's gravity direction and
                     rotation rate in the trunk frame, time. Only what the real robot can sense.
    robot.write(targets)   joint position targets (rad, actuator order)
    robot.tick()           wait for (real) or simulate (sim) one 20 ms control period

SimRobot runs scene.xml (optionally with sensor noise and a tick of latency, to rehearse);
DynamixelRobot drives the XL330s over the TTL bus (servos.json maps joints to IDs) and reads an
IMU (imu.py)."""
import json
import math
import os
import time
from dataclasses import dataclass

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.02  # the control period: 50 Hz


@dataclass
class Obs:
    q: np.ndarray        # joint angles, rad (actuator order)
    dq: np.ndarray       # joint speeds, rad/s
    gravity: np.ndarray  # gravity's direction in the trunk frame (upright: 0, 0, -1)
    gyro: np.ndarray     # trunk rotation rate, rad/s (trunk frame)
    t: float

    def roll_pitch(self):
        g = self.gravity
        return math.atan2(-g[1], -g[2]), math.asin(max(-1.0, min(1.0, g[0])))

    def tilt_deg(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, -self.gravity[2]))))


def landing(obs):
    """Which way the trunk lies (as train.landing_of, from the IMU alone)."""
    gx, gy = obs.gravity[0], obs.gravity[1]
    if abs(gx) >= abs(gy):
        return "face_down" if gx > 0 else "on_back"
    return "left_side" if gy > 0 else "right_side"


class SimRobot:
    """The MuJoCo model behind the robot API. noise: IMU and encoder noise; latency: ticks between a
    command and its arrival (the real bus and USB add about one)."""

    def __init__(self, noise=0.0, latency=0, seed=0):
        import mujoco
        from gait import load
        self.mj = mujoco
        self.m, self.key = load()
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.key)
        self.joints = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, self.m.actuator_trnid[a][0]) for a in range(self.m.nu)]
        jid = [self.m.actuator_trnid[a][0] for a in range(self.m.nu)]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in jid])
        self.dadr = np.array([self.m.jnt_dofadr[j] for j in jid])
        self.home = self.m.key_ctrl[self.key].copy()
        self.noise, self.queue = noise, [self.home.copy()] * latency
        self.rng = np.random.default_rng(seed)
        self.push_until, self.push_v = -1.0, np.zeros(3)

    def read(self):
        d, n = self.d, self.noise
        R = d.xmat[1].reshape(3, 3)
        g = R.T @ np.array([0.0, 0.0, -1.0])
        obs = Obs(d.qpos[self.qadr].copy(), d.qvel[self.dadr].copy(), g, d.qvel[3:6].copy(), d.time)
        if n:
            obs.q += self.rng.normal(0, 0.003 * n, obs.q.shape)    # encoder noise (an XL330 tick is 0.0015 rad)
            obs.gyro += self.rng.normal(0, 0.03 * n, 3)
            obs.gravity = obs.gravity + self.rng.normal(0, 0.01 * n, 3)
            obs.gravity /= np.linalg.norm(obs.gravity)
        return obs

    def write(self, targets):
        self.queue.append(np.asarray(targets, float).copy())
        self.d.ctrl[:] = self.queue.pop(0)

    def shove(self, v, secs=0.1):
        """A push: v (m/s) of the whole robot, over secs."""
        self.push_until, self.push_v = self.d.time + secs, np.asarray(v, float) * self.m.body_subtreemass[1] / secs

    def reset(self):
        self.mj.mj_resetDataKeyframe(self.m, self.d, self.key)

    def tick(self):
        for _ in range(int(round(DT / self.m.opt.timestep))):
            self.d.xfrc_applied[1, :3] = self.push_v if self.d.time < self.push_until else 0.0
            self.mj.mj_step(self.m, self.d)


class DynamixelRobot:
    """The real TinyREX: 18 XL330s on one TTL bus (Protocol 2.0, sync read/write), and an IMU.

    servos.json gives each joint its ID, direction and zero offset (set them once with the robot in
    its calibration pose, `runtime.py real --calibrate`). Positions are 4096 ticks per turn, 2048 at zero."""

    ADDR_TORQUE, ADDR_GOAL, ADDR_PRESENT_VEL, ADDR_PRESENT_POS = 64, 116, 128, 132
    ADDR_PROFILE_VEL, ADDR_CURRENT_LIMIT = 112, 38
    TICK = 2 * math.pi / 4096

    def __init__(self, port, baud=1_000_000, config=os.path.join(HERE, "servos.json"), imu=None):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
        self.ok = COMM_SUCCESS
        cfg = json.load(open(config))
        self.joints = cfg["order"]
        self.servo = cfg["servos"]
        self.home = np.array([cfg["home_rad"].get(j, 0.0) for j in self.joints])
        self.port, self.ph = PortHandler(port), PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud)):
            raise RuntimeError(f"can't open {port} at {baud}")
        self.write_goal = GroupSyncWrite(self.port, self.ph, self.ADDR_GOAL, 4)
        self.read_state = GroupSyncRead(self.port, self.ph, self.ADDR_PRESENT_VEL, 8)  # (velocity, then position)
        for j in self.joints:
            self.read_state.addParam(self.servo[j]["id"])
        self.imu = imu
        self.t0 = time.monotonic()
        self.next_tick = self.t0

    def ping_all(self):
        """{joint: (model number or None, error)} - which servos answer."""
        out = {}
        for j in self.joints:
            model, res, err = self.ph.ping(self.port, self.servo[j]["id"])
            out[j] = (model if res == self.ok else None, err)
        return out

    def torque(self, on):
        for j in self.joints:
            self.ph.write1ByteTxRx(self.port, self.servo[j]["id"], self.ADDR_TORQUE, 1 if on else 0)

    def _rad(self, j, ticks):
        s = self.servo[j]
        return s["sign"] * (ticks - 2048) * self.TICK - s.get("offset_rad", 0.0)

    def _ticks(self, j, rad):
        s = self.servo[j]
        return int(round(2048 + s["sign"] * (rad + s.get("offset_rad", 0.0)) / self.TICK))

    def read(self):
        if self.read_state.txRxPacket() != self.ok:
            raise RuntimeError("sync read failed")
        q, dq = [], []
        for j in self.joints:
            i = self.servo[j]["id"]
            pos = self.read_state.getData(i, self.ADDR_PRESENT_POS, 4)
            vel = self.read_state.getData(i, self.ADDR_PRESENT_VEL, 4)
            pos = pos - (1 << 32) if pos >= 1 << 31 else pos
            vel = vel - (1 << 32) if vel >= 1 << 31 else vel
            q.append(self._rad(j, pos))
            dq.append(self.servo[j]["sign"] * vel * 0.229 * 2 * math.pi / 60)  # (0.229 rpm per unit)
        g, w = self.imu.read() if self.imu else (np.array([0.0, 0.0, -1.0]), np.zeros(3))
        return Obs(np.array(q), np.array(dq), g, w, time.monotonic() - self.t0)

    def write(self, targets):
        self.write_goal.clearParam()
        for j, rad in zip(self.joints, targets):
            lo, hi = self.servo[j]["limits_rad"]
            v = self._ticks(j, min(max(rad, lo), hi))  # (never past the collision-free limits)
            self.write_goal.addParam(self.servo[j]["id"], list(int(v).to_bytes(4, "little", signed=True)))
        self.write_goal.txPacket()

    def tick(self):
        self.next_tick += DT
        time.sleep(max(0.0, self.next_tick - time.monotonic()))
