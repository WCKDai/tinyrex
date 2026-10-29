"""TinyREX's expressive moves: short clips on the jaw, neck, head, tail and arms, played from the
standing pose while the legs hold it.

Each clip is keyframes (time s, {joint: offset rad from HOME}); joints not named sit at HOME. Between
keyframes the offset eases (a smoothstep), so moves start and stop softly and the servos aren't
jerked. Directions, measured on the model:
  neck_pitch +  head up and back (HOME is near its +18 deg limit, so it mostly goes down)
  head_pitch +  snout down            head_yaw +   snout left        head_roll +  tilt
  jaw +         open (0..28 deg)      tail_yaw +   tail left
  l_shoulder -  left arm forward / up      r_shoulder +  right arm forward / up"""


def _wag(joint, amp, period, n, t0=0.0, others=None):
    """n half-swings of `joint` between +amp and -amp."""
    keys = []
    for i in range(n):
        k = {joint: amp if i % 2 == 0 else -amp}
        if others:
            k.update(others(i))
        keys.append((t0 + period / 2 * (i + 0.5), k))
    return keys


CLIPS = {
    # yes: two nods
    "yes": [(0.0, {}), (0.25, {"head_pitch": 0.35}), (0.5, {"head_pitch": -0.15}), (0.75, {"head_pitch": 0.35}),
            (1.0, {"head_pitch": -0.1}), (1.3, {})],
    # no: a head shake, the head rolling a little into each turn
    "no": [(0.0, {})] + _wag("head_yaw", 0.5, 0.6, 5, others=lambda i: {"head_roll": 0.12 if i % 2 == 0 else -0.12}) + [(1.9, {})],
    # excited: the tail wags fast, the jaw chatters, the arms flap, the head bobs
    "excited": [(0.0, {})] + [
        (0.12 * (i + 1), {"tail_yaw": 0.55 if i % 2 == 0 else -0.55, "jaw": 0.32 if i % 2 == 0 else 0.0,
                          "l_shoulder": -0.6 if i % 2 == 0 else 0.0, "r_shoulder": 0.6 if i % 2 == 0 else 0.0,
                          "head_pitch": -0.12 if i % 2 == 0 else 0.05})
        for i in range(14)] + [(1.95, {})],
    # roar: snout up, jaw wide, arms up and out, a held shake of the head, then back
    "roar": [(0.0, {}), (0.45, {"head_pitch": -0.55, "neck_pitch": 0.02, "jaw": 0.47, "l_shoulder": -1.0, "r_shoulder": 1.0, "tail_yaw": 0.15}),
             (0.75, {"head_pitch": -0.55, "jaw": 0.48, "l_shoulder": -1.05, "r_shoulder": 1.05, "head_roll": 0.1, "tail_yaw": -0.15}),
             (1.05, {"head_pitch": -0.55, "jaw": 0.48, "l_shoulder": -1.0, "r_shoulder": 1.0, "head_roll": -0.1, "tail_yaw": 0.15}),
             (1.35, {"head_pitch": -0.5, "jaw": 0.45, "l_shoulder": -1.0, "r_shoulder": 1.0}), (1.9, {})],
    # chomp: snout forward and down, three quick bites
    "chomp": [(0.0, {}), (0.25, {"head_pitch": 0.3, "neck_pitch": -0.15, "jaw": 0.45})] + [
        (0.25 + 0.16 * (i + 1), {"head_pitch": 0.3, "neck_pitch": -0.15, "jaw": 0.0 if i % 2 == 0 else 0.45}) for i in range(6)] + [(1.5, {})],
    # curious: head tilted to one side, then the other, a small look around
    "curious": [(0.0, {}), (0.5, {"head_roll": 0.3, "head_yaw": 0.35, "head_pitch": 0.1}), (1.3, {"head_roll": 0.3, "head_yaw": 0.35, "head_pitch": 0.1}),
                (1.8, {"head_roll": -0.3, "head_yaw": -0.3, "head_pitch": 0.05}), (2.6, {"head_roll": -0.3, "head_yaw": -0.3}), (3.0, {})],
    # sad: head drops, arms hang, a slow small tail sway
    "sad": [(0.0, {}), (0.8, {"neck_pitch": -0.55, "head_pitch": 0.35, "tail_yaw": 0.1}), (1.6, {"neck_pitch": -0.6, "head_pitch": 0.4, "tail_yaw": -0.1}),
            (2.4, {"neck_pitch": -0.6, "head_pitch": 0.4, "tail_yaw": 0.1}), (3.2, {})],
    # wave: the right arm up, three waves, the head turned toward it
    "wave": [(0.0, {}), (0.4, {"r_shoulder": 1.15, "head_yaw": -0.25, "head_roll": -0.08})] + [
        (0.4 + 0.22 * (i + 1), {"r_shoulder": 0.85 if i % 2 == 0 else 1.2, "head_yaw": -0.25, "head_roll": -0.08, "jaw": 0.15}) for i in range(6)] + [(2.2, {})],
}


def offsets(name, t):
    """The clip's joint offsets at time t (s), or None when it has finished."""
    keys = CLIPS[name]
    if t >= keys[-1][0]:
        return None
    for (t0, a), (t1, b) in zip(keys, keys[1:]):
        if t0 <= t < t1:
            u = (t - t0) / (t1 - t0)
            u = u * u * (3 - 2 * u)  # (smoothstep: no jerk at the keys)
            return {j: a.get(j, 0.0) + (b.get(j, 0.0) - a.get(j, 0.0)) * u for j in set(a) | set(b)}
    return {}
