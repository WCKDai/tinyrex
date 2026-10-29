"""TinyREX's IMU on the real robot: an LSM6DSV16X on the head frame (BOM), read over I2C.

The controllers want the trunk's gravity direction and rotation rate. The IMU rides on the head, so
HeadIMU turns its readings into the trunk frame every tick through the neck and head joints'
current angles (the MuJoCo model as a kinematics library: the same one the simulator uses).

Gravity comes from a complementary filter: the gyro integrated, pulled toward the accelerometer.
Register map per the LSM6DSV16X datasheet (ST DS13510): check it against the board before first use."""
import math

import numpy as np

WHO_AM_I, WHO = 0x0F, 0x70
CTRL1, CTRL2, CTRL6, CTRL8 = 0x10, 0x11, 0x15, 0x17
OUTX_L_G, OUTX_L_A = 0x22, 0x28
GYRO_DPS_PER_LSB = 0.035      # at +-1000 dps
ACCEL_G_PER_LSB = 0.000122    # at +-4 g


class LSM6DSV16X:
    def __init__(self, bus=1, addr=0x6A):
        from smbus2 import SMBus
        self.bus, self.addr = SMBus(bus), addr
        if self.bus.read_byte_data(addr, WHO_AM_I) != WHO:
            raise RuntimeError(f"no LSM6DSV16X at 0x{addr:02x}")
        self.bus.write_byte_data(addr, CTRL1, 0x07)  # accel, high-performance mode, 240 Hz
        self.bus.write_byte_data(addr, CTRL2, 0x07)  # gyro, 240 Hz
        self.bus.write_byte_data(addr, CTRL6, 0x04)  # gyro +-1000 dps
        self.bus.write_byte_data(addr, CTRL8, 0x01)  # accel +-4 g

    def _vec(self, reg, scale):
        b = self.bus.read_i2c_block_data(self.addr, reg, 6)
        return np.array([int.from_bytes(bytes(b[i:i + 2]), "little", signed=True) for i in (0, 2, 4)]) * scale

    def read(self):
        """(accel in g, gyro in rad/s), in the chip's frame."""
        return self._vec(OUTX_L_A, ACCEL_G_PER_LSB), self._vec(OUTX_L_G, math.radians(GYRO_DPS_PER_LSB))


class HeadIMU:
    """The head IMU, reported in the trunk frame. chip_to_body: the chip's axes in the head body's
    frame (measure once: `runtime.py real --imu`)."""

    def __init__(self, chip, joints, chip_to_body=np.eye(3), alpha=0.02):
        import mujoco
        from gait import load
        self.mj, self.chip, self.alpha = mujoco, chip, alpha
        self.m, key = load()
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        self.joints = joints
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in joints]
        self.head = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "jaw_soft")
        self.C = np.asarray(chip_to_body, float)
        self.g = np.array([0.0, 0.0, -1.0])
        self.q = None

    def set_joints(self, q):
        self.q = q

    def read(self):
        acc, gyro = self.chip.read()
        acc, gyro = self.C @ acc, self.C @ gyro
        if self.q is not None:  # head frame -> trunk frame through the current neck and head angles
            self.d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
            for a, v in zip(self.qadr, self.q):
                self.d.qpos[a] = v
            self.mj.mj_kinematics(self.m, self.d)
            R = self.d.xmat[1].reshape(3, 3).T @ self.d.xmat[self.head].reshape(3, 3)
            acc, gyro = R @ acc, R @ gyro
        # complementary filter: rotate the last estimate by the gyro, pull it toward the accelerometer
        g = self.g - np.cross(gyro, self.g) * 0.02
        a = -acc / max(np.linalg.norm(acc), 1e-6)
        g = (1 - self.alpha) * g + self.alpha * a
        self.g = g / np.linalg.norm(g)
        return self.g.copy(), gyro
