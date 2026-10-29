"""A neural get-up policy for TinyREX, trained with PPO (stable-baselines3) in MuJoCo.

Unlike train.py's fixed pose sequences, the policy reacts to how it is lying, 50 times a second,
from what the robot can sense: the IMU's gravity direction and rotation rate, its joint angles and
speeds, and its last command. It outputs every servo's target.

    python rl_getup.py train [--steps N] [--envs N]   -> getup_policy.zip, getup_policy.npz
    python rl_getup.py eval                           held-out recorded falls: how many it gets up from

Episodes start from a recorded real fall (train.py bank; the even-numbered ones, the odd ones are
held out), a clean drop on any side, or a random collapse from standing. Reward: upright, then tall,
then back in the HOME pose; small penalties for jerky commands."""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gait import load, tilt  # noqa: E402
from train import BANK, LANDINGS, fallen  # noqa: E402

CTRL_DT = 0.02
EPISODE = 6.0
POLICY = os.path.join(HERE, "getup_policy")


def falls(split):
    """Recorded falls (qpos, qvel) from the bank: split 0 for training, 1 held out."""
    b = np.load(BANK)
    out = []
    for landing in b.files:
        out += [s for i, s in enumerate(b[landing]) if i % 2 == split]
    return out


class Body:
    """What the policy sees and how its actions become servo targets."""

    def __init__(self, m, key):
        self.m, self.key = m, key
        self.home = m.key_ctrl[key].copy()
        jnt = [m.actuator_trnid[a][0] for a in range(m.nu)]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jnt])
        self.dadr = np.array([m.jnt_dofadr[j] for j in jnt])
        self.lo, self.hi = m.jnt_range[jnt, 0], m.jnt_range[jnt, 1]
        self.span = 0.5 * (self.hi - self.lo)
        self.z_home = m.key_qpos[key][2]
        self.n_obs = 3 + 3 + 3 * m.nu

    def obs(self, d, last):
        R = d.xmat[1].reshape(3, 3)
        g = R.T @ np.array([0.0, 0.0, -1.0])  # (gravity in the trunk frame: the IMU)
        q = d.qpos[self.qadr] - self.home
        dq = d.qvel[self.dadr]
        return np.concatenate([g, d.qvel[3:6] * 0.25, q, dq * 0.05, last]).astype(np.float32)

    def target(self, a):
        return np.clip(self.home + np.clip(a, -1, 1) * self.span, self.lo, self.hi)

    def upright(self, d):
        return d.xmat[1][8]  # (the trunk's z axis against the world's: 1 upright)


try:  # (training only: play.py runs the exported policy with numpy alone)
    import gymnasium as gym
    _Env = gym.Env
except ImportError:
    gym, _Env = None, object


class GetupEnv(_Env):
    def __init__(self, seed=0, split=0):
        self.m, key = load()
        self.b = Body(self.m, key)
        self.key = key
        self.falls = falls(split)
        self.rng = np.random.default_rng(seed)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (self.b.n_obs,), np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (self.m.nu,), np.float32)
        self.sub = int(round(CTRL_DT / self.m.opt.timestep))
        self.drops = {l: fallen(self.m, key, l) for l in LANDINGS}

    def start(self):
        m, r = self.m, self.rng.random()
        d = mujoco.MjData(m)
        if r < 0.6 and self.falls:  # a recorded real fall
            s = self.falls[self.rng.integers(len(self.falls))]
            mujoco.mj_resetDataKeyframe(m, d, self.key)
            d.qpos[:], d.qvel[:] = s[:m.nq], s[m.nq:]
        elif r < 0.8:  # a clean drop, turned to a random heading
            src = self.drops[list(LANDINGS)[self.rng.integers(4)]]
            d.qpos[:], d.qvel[:] = src.qpos, src.qvel
            yaw = self.rng.uniform(0, 2 * math.pi)
            qy = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
            q = np.zeros(4)
            mujoco.mju_mulQuat(q, qy, d.qpos[3:7])
            d.qpos[3:7] = q
        else:  # a random collapse: shoved, servos to random targets, settled
            mujoco.mj_resetDataKeyframe(m, d, self.key)
            d.ctrl[:] = self.b.target(self.rng.uniform(-0.6, 0.6, m.nu))
            a = self.rng.uniform(0, 2 * math.pi)
            d.qvel[0:2] = [math.cos(a) * 1.5, math.sin(a) * 1.5]
            while d.time < 1.5:
                mujoco.mj_step(m, d)
            d.time = 0.0
        d.ctrl[:] = self.b.home
        mujoco.mj_forward(m, d)
        return d

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.m.geom_friction[:, 0] = self.rng.uniform(0.6, 1.2)  # (floor and feet grip, randomised)
        self.d = self.start()
        self.t = 0.0
        self.last = np.zeros(self.m.nu, np.float32)
        return self.b.obs(self.d, self.last), {}

    def step(self, a):
        a = np.clip(a, -1, 1).astype(np.float32)
        self.d.ctrl[:] = self.b.target(a)
        for _ in range(self.sub):
            mujoco.mj_step(self.m, self.d)
        self.t += CTRL_DT
        up = self.b.upright(self.d)
        h = min(1.0, self.d.qpos[2] / self.b.z_home)
        pose = math.exp(-float(np.sum((self.d.qpos[self.b.qadr] - self.b.home) ** 2)) / 0.3)
        r = up + (1.5 * h if up > 0.6 else 0.0) + (2.0 * pose if up > 0.9 and h > 0.85 else 0.0)
        r -= 0.02 * float(np.sum((a - self.last) ** 2))
        self.last = a
        bad = not np.isfinite(self.d.qpos).all()
        return self.b.obs(self.d, self.last), float(r) if not bad else -10.0, bad, self.t >= EPISODE, {}

    def stood(self):
        return tilt(self.d) < 20 and self.d.qpos[2] > 0.85 * self.b.z_home


def export(model, norm, path):
    """The policy as plain arrays, for play.py (numpy, no torch)."""
    sd = model.policy.state_dict()
    w = {k.replace(".", "_"): v.cpu().numpy() for k, v in sd.items() if k.startswith(("mlp_extractor.policy_net", "action_net"))}
    w["obs_mean"], w["obs_var"] = norm.obs_rms.mean, norm.obs_rms.var
    w["clip_obs"] = np.array(norm.clip_obs)
    np.savez(path, **w)


class NumpyPolicy:
    """The exported policy: normalise, two tanh layers, linear out (stable-baselines3's MlpPolicy)."""

    def __init__(self, path=POLICY + ".npz"):
        self.w = dict(np.load(path))
        self.layers = sorted({k.rsplit("_", 1)[0] for k in self.w if k.startswith("mlp_extractor_policy_net")})

    def __call__(self, obs):
        w = self.w
        x = np.clip((obs - w["obs_mean"]) / np.sqrt(w["obs_var"] + 1e-8), -w["clip_obs"], w["clip_obs"])
        for l in self.layers:
            x = np.tanh(w[l + "_weight"] @ x + w[l + "_bias"])
        return np.clip(w["action_net_weight"] @ x + w["action_net_bias"], -1, 1)


def evaluate(pi, n=None):
    """Held-out recorded falls: does it stand within the episode?"""
    env = GetupEnv(seed=1, split=1)
    held = env.falls if n is None else env.falls[:n]
    ok = []
    for s in held:
        env.falls = [s]
        env.rng = np.random.default_rng(0)
        o, _ = env.reset()
        env.d.qpos[:], env.d.qvel[:] = s[:env.m.nq], s[env.m.nq:]
        mujoco.mj_forward(env.m, env.d)
        done, t_up = False, None
        while not done:
            o, r, term, trunc, _ = env.step(pi(o))
            done = term or trunc
            if env.stood():
                t_up = t_up if t_up is not None else env.t
        ok.append(env.stood())
    return sum(ok), len(ok)


def main():
    what = sys.argv[1]
    if what == "train":
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize
        steps = int(float(sys.argv[sys.argv.index("--steps") + 1])) if "--steps" in sys.argv else 30_000_000
        n = int(sys.argv[sys.argv.index("--envs") + 1]) if "--envs" in sys.argv else 16
        torch.set_num_threads(4)
        venv = VecNormalize(VecMonitor(SubprocVecEnv([lambda i=i: GetupEnv(seed=i) for i in range(n)])), norm_reward=True, clip_obs=10.0)
        model = PPO("MlpPolicy", venv, n_steps=max(64, 8192 // n), batch_size=4096, n_epochs=5, learning_rate=3e-4, gamma=0.99,
                    gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, policy_kwargs=dict(net_arch=[256, 256], log_std_init=-1.0),
                    verbose=0, seed=0)
        chunk = int(float(sys.argv[sys.argv.index("--chunk") + 1])) if "--chunk" in sys.argv else 2_000_000
        done, t0 = 0, __import__("time").time()
        while done < steps:
            model.learn(chunk, reset_num_timesteps=False)
            done += chunk
            model.save(POLICY)
            venv.save(POLICY + "_norm.pkl")
            export(model, venv, POLICY + ".npz")
            ok, total = evaluate(NumpyPolicy(), n=30)
            ep = [e["r"] for e in model.ep_info_buffer] if model.ep_info_buffer else [0]
            print(f"steps {done / 1e6:.0f}M  episode reward {np.mean(ep):.0f}  held-out falls stood {ok}/{total}  ({(__import__("time").time() - t0) / 60:.0f} min)", flush=True)
    elif what == "eval":
        ok, total = evaluate(NumpyPolicy())
        print(f"held-out recorded falls: stood up from {ok}/{total}")


if __name__ == "__main__":
    main()
