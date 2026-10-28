#!/usr/bin/env python3
"""TinyREX head, v3: writes a Polykrom command script (JSON) for the head alone.

    head.py OUT.json

Frame: X forward (snout at +X), Y left, Z up; the skull's underside (the mouth line) is z = 0 and
its back is x = 0. All sizes in mm.

The head follows the reference boards: a lime helmet over a dark robot frame, panel seams that
cut through the helmet so the frame shows through, glossy dome eyes, a camera module and a red LED
on a rounded nose, nostril vents, a ToF window on the crown, black button-head screws on every
panel, a cream jaw on its servo's horn with two rounded fangs, and the head-yaw servo and its
bracket under it. The helmet and the jaw are smooth lofts through rounded cross-sections; the
helmet is shelled to a printable 2.2 mm wall, open underneath for the electronics.
"""
import json
import math
import sys

LIME, CREAM, BLACK, FRAME, ALU = "#C8F000", "#EDE6D6", "#161616", "#2A2C2F", "#B9BDC2"
RED, FANG, GLASS, WHITE = "#FF2A1A", "#F7F4EE", "#26282B", "#FFFFFF"

# Helmet sections: (x, half width, bottom z, top z, corner radius).
SKULL = [
    (0, 30, 8, 50, 14),
    (5, 37, 2, 59, 17),
    (22, 40, 0, 66, 19),
    (46, 39, 0, 67, 19),
    (70, 33, 0, 57, 16),
    (94, 23, 0, 41, 12),
    (110, 15, 4, 28, 8),
]
WALL = 2.2
GAP = 4
JAW = [
    (6, 28, -21, -GAP, 8),
    (40, 33, -25, -GAP, 10),
    (74, 30, -25, -GAP, 10),
    (94, 23, -21, -GAP, 8),
    (102, 16, -16, -GAP, 5.5),
]
EYE = dict(x=60, z=38, hole_r=13.5, dome_r=12.5)
FANG_AT = [(104, 10), (104, -10)]
SEAM = 0.9  # panel gap width

cmds = []
colours = {}


def c(command, **params):
    cmds.append({"command": command, "params": params})


def paint(colour, *names):
    colours.setdefault(colour, []).extend(names)


def section_at(x, sections=SKULL):
    """The rounded section (half width, bottom, top, radius) at x, linear between stations."""
    for (x0, *a), (x1, *b) in zip(sections, sections[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return [p + (q - p) * t for p, q in zip(a, b)]
    raise ValueError(x)


def flank_y(x, z, sections=SKULL):
    """How far out a loft's side is at (x, z) (the helmet's by default)."""
    w, zb, zt, r = section_at(x, sections)
    if z > zt - r:
        d = z - (zt - r)
    elif z < zb + r:
        d = (zb + r) - z
    else:
        return w
    return w - r + math.sqrt(max(r * r - d * d, 0.0))


def top_z(x, y):
    """How high the helmet's crown is at (x, y)."""
    w, zb, zt, r = section_at(x)
    d = abs(y) - (w - r)
    return zt if d <= 0 else zt - r + math.sqrt(max(r * r - d * d, 0.0))


def loft(name, sections):
    """A smooth loft through rounded rectangles on YZ planes along X."""
    names = []
    for i, (x, w, zb, zt, r) in enumerate(sections):
        sk = f"{name}{i}"
        c("sketch.create", plane="YZ", offset=x, name=sk)
        c("sketch.rectangle.rounded", p0=[-w, zb], p1=[w, zt], radius=r)
        c("sketch.finish")
        names.append({"sketch": sk})
    c("solid.loft", sections=names, operation="new", name=name, body_name=name)


def cyl(name, base, axis, r, h, op="new", targets=None):
    p = dict(radius=r, height=h, base=list(base), axis=list(axis), operation=op)
    if op == "new":
        p["body_name"] = name
    else:
        p["targets"] = targets
    c("solid.cylinder", **p)


def box(name, lo, hi, op="new", targets=None):
    p = dict(length=hi[0] - lo[0], width=hi[1] - lo[1], height=hi[2] - lo[2], corner=list(lo))
    if op == "new":
        p["body_name"] = name
    else:
        p["operation"] = op
        p["targets"] = targets
    c("solid.box", **p)


def screw(name, at, axis):
    """A black button-head screw sitting on the surface at `at`, facing out along `axis`."""
    base = [a - 0.4 * n for a, n in zip(at, axis)]
    cyl(name, base, axis, 1.7, 1.1)
    paint(BLACK, name)


# Helmet: the outer loft, shelled to a printable wall, open underneath for the electronics.
loft("Skull", SKULL)
# Hollowed by cutting an inner loft inset by the wall (top and sides) and running out through
# the bottom, which leaves the helmet open underneath. (solid.shell refuses the corner where the
# loft's flat back cap meets its curved sides.)
inner = [(x + (WALL if i == 0 else -WALL if i == len(SKULL) - 1 else 0), w - WALL, zb - 6, zt - WALL, r - WALL) for i, (x, w, zb, zt, r) in enumerate(SKULL)]
loft("SkullInner", inner)
c("solid.combine", target="Skull", tools=["SkullInner"], operation="cut")
paint(LIME, "Skull")
# The dark frame inside, seen through the seams, the eye bezels and the vents.
loft("Frame", [(x + (1.5 if i == 0 else -1.5 if i == len(SKULL) - 1 else 0), w - 4, zb + 3, zt - 4.5, max(r - 4, 3)) for i, (x, w, zb, zt, r) in enumerate(SKULL)])
paint(FRAME, "Frame")

# Panel seams, cut through the helmet wall: along the crown, across behind the eyes, round the snout.
box(None, (6, -SEAM / 2, 22), (104, SEAM / 2, 90), op="cut", targets=["Skull"])
box(None, (40 - SEAM / 2, -50, 44), (40 + SEAM / 2, 50, 90), op="cut", targets=["Skull"])
box(None, (84 - SEAM / 2, -50, 7), (84 + SEAM / 2, 50, 90), op="cut", targets=["Skull"])

# Eyes: a hole through the helmet, a glossy black dome bulging out of it, a catch-light.
for s in (1, -1):
    side = "L" if s > 0 else "R"
    f = flank_y(EYE["x"], EYE["z"])
    cyl(None, (EYE["x"], s * (f - 8), EYE["z"]), (0, s, 0), EYE["hole_r"], 16, op="cut", targets=["Skull"])
    centre = f - 10
    c("solid.sphere", radius=EYE["dome_r"], center=[EYE["x"], s * centre, EYE["z"]], operation="new", body_name=f"Eye_{side}")
    reach = centre + math.sqrt(EYE["dome_r"] ** 2 - 2 * 4.6**2)
    c("solid.sphere", radius=2.6, center=[EYE["x"] + 4.6, s * (reach + 0.6), EYE["z"] + 4.6], operation="new", body_name=f"EyeGlint_{side}")
    paint(BLACK, f"Eye_{side}")
    paint(WHITE, f"EyeGlint_{side}")

# Nose: a rounded camera module with its lens (robot's left), the red LED in a bezel (its right).
c("sketch.create", plane="YZ", offset=108.6, name="CameraModule")
c("sketch.rectangle.rounded", p0=[0.5, 11.5], p1=[13.5, 22.5], radius=2.8)
c("sketch.finish")
c("solid.extrude", sketch="CameraModule", distance=3, operation="new", body_name="CameraModule")
cyl("CameraLens", (111.5, 7, 17), (1, 0, 0), 3.3, 0.6)
cyl("CameraGlass", (112.0, 7, 17), (1, 0, 0), 2.1, 0.4)
cyl("LEDBezel", (109.3, -8, 17), (1, 0, 0), 2.6, 1.4)
cyl("LED", (110.6, -8, 17), (1, 0, 0), 1.6, 0.6)
paint(BLACK, "CameraModule", "LEDBezel")
paint(GLASS, "CameraLens")
paint("#3A4A5A", "CameraGlass")
paint(RED, "LED")
# Nostril vents: two slots through the top of the snout.
for s in (1, -1):
    box(None, (95, s * 6.5 - 0.7, 20), (102, s * 6.5 + 0.7, 60), op="cut", targets=["Skull"])

# ToF window on the crown, behind the cross seam.
c("sketch.create", plane="XY", offset=top_z(26, 8) - 2.5, name="ToFPad")
c("sketch.rectangle.rounded", p0=[20, -8], p1=[33, 8], radius=3)
c("sketch.finish")
c("solid.extrude", sketch="ToFPad", distance=3.2, operation="new", body_name="ToFPad")
c("sketch.create", plane="XY", offset=top_z(26, 4) + 0.1, name="ToFGlass")
c("sketch.rectangle.rounded", p0=[23, -5], p1=[30, 5], radius=1.5)
c("sketch.finish")
c("solid.extrude", sketch="ToFGlass", distance=0.6, operation="new", body_name="ToFGlass")
paint(BLACK, "ToFPad")
paint("#3A4A5A", "ToFGlass")

# Screws on every panel: cheeks, crown, snout, back.
for s in (1, -1):
    side = "L" if s > 0 else "R"
    for x, z in ((20, 28), (30, 48), (92, 14), (76, 30)):
        screw(f"Screw_{side}{x}", (x, s * flank_y(x, z), z), (0, s, 0))
    for x in (12, 58):
        y = s * 22
        screw(f"CrownScrew_{side}{x}", (x, y, top_z(x, y)), (0, 0, 1))
    screw(f"BackScrew_{side}", (0, s * 14, 32), (-1, 0, 0))

# Jaw: cream loft under a dark mouth, two rounded fangs in front of its lip.
loft("Jaw", JAW)
paint(CREAM, "Jaw")
box("Mouth", (18, -18, -GAP - 0.5), (90, 18, 0.5))
paint(BLACK, "Mouth")
for i, (x, y) in enumerate(FANG_AT, 1):
    cyl(f"Fang{i}", (x, y, 1.5), (0, 0, -1), 2.4, 5.0)
    c("solid.sphere", radius=2.4, center=[x, y, -3.5], operation="new", body_name=f"FangTip{i}")
    paint(FANG, f"Fang{i}", f"FangTip{i}")

# Jaw servo: cream cheek drums with the aluminium horn and its four screws.
for s in (1, -1):
    side = "L" if s > 0 else "R"
    cyl(f"CheekPod_{side}", (14, s * 27, -6), (0, s, 0), 13, 12)
    cyl(f"JawHorn_{side}", (14, s * 39, -6), (0, s, 0), 7, 1.6)
    paint(CREAM, f"CheekPod_{side}")
    paint(ALU, f"JawHorn_{side}")
    for dx, dz in ((3.6, 0), (-3.6, 0), (0, 3.6), (0, -3.6)):
        n = f"HornScrew_{side}{dx:+.0f}{dz:+.0f}"
        cyl(n, (14 + dx, s * 40.5, -6 + dz), (0, s, 0), 0.9, 0.8)
        paint(BLACK, n)

# Neck: the head-yaw XL330 (34 x 20 x 26) under the helmet, its horn up into the head, and the
# cream bracket plates that carry it.
box("HeadYawServo", (18, -10, -52), (52, 10, -26))
cyl("HeadYawHorn", (35, 0, -26), (0, 0, 1), 7, 2)
for s in (1, -1):
    side = "L" if s > 0 else "R"
    # A rounded cream plate each side (XZ sketch: u = x, v = -z), 3 mm thick.
    c("sketch.create", plane="XZ", offset=10 if s > 0 else -13, name=f"NeckBracket_{side}")
    c("sketch.rectangle.rounded", p0=[16, 22], p1=[54, 54], radius=7)
    c("sketch.finish")
    c("solid.extrude", sketch=f"NeckBracket_{side}", distance=3, operation="new", body_name=f"NeckBracket_{side}")
    paint(CREAM, f"NeckBracket_{side}")
    for x in (24, 46):
        screw(f"BracketScrew_{side}{x}", (x, s * 13, -46), (0, s, 0))
    # Jaw panel screws.
    for x, z in ((44, -15), (72, -15)):
        screw(f"JawScrew_{side}{x}", (x, s * flank_y(x, z, JAW), z), (0, s, 0))
paint(BLACK, "HeadYawServo")
paint(ALU, "HeadYawHorn")

for colour, names in colours.items():
    c("appearance.assign", bodies=names, color=colour)

json.dump(cmds, open(sys.argv[1], "w"), indent=1)
print(len(cmds), "commands")
