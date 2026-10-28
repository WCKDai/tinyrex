"""TinyREX v4 shells, one part at a time, each kept inside the room Microduck's frame leaves it.

Sent after tinyrex_v2.py and rig.py (same globals), with PART set to the part to (re)build. Each
part is built, parented to the body it moves with, then checked: clips at rest, clips while the
joints that can reach it sweep their range, and viewport renders alone and in context.
"""
import json

PART = globals().get("PART", "body")
coll = bpy.data.collections.get(f"{SCENE}: skins") or collection("skins")


def clear(prefixes):
    for o in list(coll.objects):
        if o.name.split(".")[0].startswith(prefixes):
            bpy.data.objects.remove(o, do_unlink=True)


SOFT = 3  # fillet segments on every panel edge: soft product edges, flat faces kept


def soft_edges(o, width, segments=SOFT):
    """Round a panel's edges with a small fillet (flat faces stay flat, shaded crisp); named
    "Chamfer" so later passes recognise it."""
    b = o.modifiers.new("Chamfer", "BEVEL")
    b.width, b.segments, b.limit_method = width, segments, "ANGLE"
    b.angle_limit = math.radians(25)
    b.profile = 0.6
    b.harden_normals = True
    o.data.shade_smooth()
    try:
        o.data.set_sharp_from_angle(angle=math.radians(40))
    except AttributeError:
        pass
    return b


def facet(o, width=4.0):
    """Turn a subdivided shell into the boards' hard-surface panel: drop the smoothing, chamfer its
    edges (before its wall and openings), flat shading."""
    subs = [m for m in o.modifiers if m.type == "SUBSURF" or m.name == "Hold"]
    if not subs or any(m.name == "Chamfer" for m in o.modifiers):
        return
    for m in subs:
        o.modifiers.remove(m)
    soft_edges(o, width)
    o.modifiers.move(len(o.modifiers) - 1, 0)


def made(objs, body=None):
    for o in objs:
        if o.type == "MESH" and not o.get("keep_round"):
            facet(o)
        if o.name not in coll.objects:
            for c in o.users_collection:
                c.objects.unlink(o)
            coll.objects.link(o)
        attach(o, body)
        if not o.name.split(".")[0].startswith("XL330"):
            o["skin"] = True
    return [o.name.split(".")[0] for o in objs]


WALL = 2.0  # printed shell wall, mm (5 perimeters of a 0.4 mm nozzle)


def hollow(o, wall=WALL, even=True):
    """Make a closed surface a printed shell: a wall `wall` thick, inward (`even` off where sharp
    steps would fold an even-thickness wall into spikes)."""
    m = o.modifiers.new("Wall", "SOLIDIFY")
    m.thickness = wall
    m.offset = -1.0
    m.use_even_offset = even
    return o


def loft_trap(name, sections, material, subsurf=3, keep=0.42, yc=0.0, bevel=None, facet=0.0):
    """A smooth shell through trapezoid sections (x, half width at the bottom, half width at the top,
    z bottom, z top): narrower below where the legs swing, fuller above."""
    v, f = [], []
    for x, wb, wt, z0, z1 in sections:
        v += [(x, yc - wb, z0), (x, yc + wb, z0), (x, yc + wt, z1), (x, yc - wt, z1)]
    n = len(sections)
    f.append((3, 2, 1, 0))
    for i in range(n - 1):
        a, b = 4 * i, 4 * (i + 1)
        for k in range(4):
            f.append((a + k, a + (k + 1) % 4, b + (k + 1) % 4, b + k))
    f.append((4 * (n - 1), 4 * (n - 1) + 1, 4 * (n - 1) + 2, 4 * (n - 1) + 3))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    o = obj_from(name, me, coll, material, smooth=not facet)
    if facet:
        # The concept boards' look: flat panels with softly filleted edges, not a pebble.
        soft_edges(o, facet)
        return o
    if keep:
        b = o.modifiers.new("Hold", "BEVEL")
        b.width = bevel or min(min(2 * wb, 2 * wt, z1 - z0) for _, wb, wt, z0, z1 in sections) * keep
        b.segments = 2
    sub = o.modifiers.new("Subsurf", "SUBSURF")
    sub.levels = sub.render_levels = subsurf
    return o


def battery_enclosed():
    from mathutils.bvhtree import BVHTree
    deps = bpy.context.evaluated_depsgraph_get()
    o = next(x for x in coll.objects if x.name.split(".")[0] == "Body")
    off = [m for m in o.modifiers if m.type in ("SOLIDIFY", "BOOLEAN") and m.show_viewport]
    for m in off:
        m.show_viewport = False
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(deps)
    me = ev.to_mesh()
    tree = BVHTree.FromPolygons([o.matrix_world @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons])
    ev.to_mesh_clear()
    for m in off:
        m.show_viewport = True
    bpy.context.view_layer.update()
    b = next(x for x in scene.objects if x.name.split(".")[0] == "Battery")
    out = []
    for c in b.bound_box:
        p, hits, d = b.matrix_world @ Vector(c), 0, Vector((0.31, 0.47, 0.83)).normalized()
        q = p.copy()
        for _ in range(64):
            loc, *_r = tree.ray_cast(q, d)
            if loc is None:
                break
            hits += 1
            q = loc + d * 1e-3
        if hits % 2 == 0:
            out.append([round(v) for v in p])
    return out


HIPS = ["yaw2roll", "hip_l", "upper_leg_left", "bearing_roll", "hip_l_2", "upper_leg_right"]


BODY = [(-90, 14.5, 15, 97, 134), (-76, 17, 17.5, 95, 134.5), (-62, 22, 23, 62, 137), (-50, 27, 29, 56, 149), (-34, 31, 33, 54, 152), (-14, 31, 34, 54, 154), (-9, 31, 34, 70, 154), (-6, 33, 34, 99, 154), (17, 33, 33, 99, 151), (36, 23, 24, 110, 143)]


def part_body():
    """A rounded pebble inside Microduck's body-shell room (x -46..35, |y| <= 35, z >= 104), and a
    narrow pod below the back for the battery's lower half (x -47..-25, |y| 19, down to z 67),
    between the thighs (|y| >= 42)."""
    for o in list(scene.objects):
        if o.name.split(".")[0] in ("Cut_Belly", "Cut_TailSlot", "Cut_TailHorn", "Cut_Shoulder1", "Cut_Shoulder-1", "Cut_LegWindow1", "Cut_LegWindow-1"):
            bpy.data.objects.remove(o, do_unlink=True)
    clear(("Body", "BatteryPod", "Backpack", "PackScrew", "BodyMount", "BodyScrew", "Flank", "Spike_B", "BatteryDoor", "ChargePort", "PowerSwitch"))
    objs = [
        # One smooth body: full above (|y| <= 34), narrow below the back (|y| <= 22) round the
        # battery's lower half, so the thighs swinging in on hip roll pass it.
        # Round the trunk's hardware (measured): battery x -47..-25 |y| 20 z 68..139, its holders
        # x -31..-14 |y| 27 z 63..146, the hip yaw servos x -21..16 |y| 28 z 102..134, the trunk
        # plate |y| 29 z 106..109; the back narrows into the tail root round the tail servo
        # (x -84..-48 |y| 10 z 102..131), its back 14.5 mm from the tail's yaw axis.
        hollow(loft_trap("Body", BODY, CREAM, subsurf=3, keep=0.28), even=False),
        loft_skin("Backpack", coll, LIME, [(-46, 19, 150, 157), (-22, 23, 152, 160), (2, 20, 151, 158.5)], subsurf=2, keep=0.42),
        # The battery swaps through a door underneath; USB-C and the power switch on the tail root.
        rounded("BatteryDoor", coll, mat("Seam", "#BDB6A8", 0.6), (-46, -15, 56.8), (-27, 15, 58.6), 1.6),
        rounded("ChargePort", coll, INK, (-80, -17.6, 108), (-73, -15.4, 112), 1.0),
        rounded("PowerSwitch", coll, LIME, (-80, 15.4, 107.5), (-73, 18, 112.5), 1.3),
    ]
    for k, x in enumerate((-34, -6)):
        for sy in (1, -1):
            screw_dot(f"PackScrew{k}{'L' if sy > 0 else 'R'}", coll, (x, sy * 15, 159.1 if k else 157.8), "Z")
            objs.append(bpy.data.objects[f"PackScrew{k}{'L' if sy > 0 else 'R'}"])
    # Mounting bosses: the shell screws to the trunk plate's edges (|y| 28.5, z 106..109).
    for k, (xa, xb) in enumerate(((-16, -11), (8, 13))):
        for sy in (1, -1):
            objs.append(rounded(f"BodyMount{k}{'L' if sy > 0 else 'R'}", coll, CREAM, (xa, min(sy * 28.6, sy * 31.8), 106.2), (xb, max(sy * 28.6, sy * 31.8), 109.5), 0.6))
    for sy in (1, -1):  # the screws into the rear bosses
        for k, (x, z) in enumerate(((-13.5, 108.5),)):
            objs.append(screw(f"BodyScrew{k}{'L' if sy > 0 else 'R'}", (x, sy * (side_y(BODY, x, z) - 0.4), z), "Y" if sy > 0 else "-Y"))
    # Lime flank armour: a teardrop plate on each side, over the battery holders, two screws each.
    for sy, side in ((1, "L"), (-1, "R")):
        y0, y1 = (32.2, 35.6) if sy > 0 else (-35.6, -32.2)
        objs.append(slab(f"Flank_{side}", LIME, teardrop((-6, 133), 14.5, (-27, 124), 9.0), y0, y1, chamfer=1.6))
        for k, (x, z) in enumerate(((-4, 134), (-24, 125))):
            objs.append(screw(f"FlankScrew_{side}{k}", (x, sy * 35.2, z), "Y" if sy > 0 else "-Y"))
    # Dorsal spikes continue down the back, behind the backpack, on the tail root.
    for k, (x, h) in enumerate(((-55, 8.0),)):
        objs.append(spike(f"Spike_B{k}", x, side_top(BODY, x), h))
    names = made(objs, TRUNK)
    body = objs[0]
    # Openings: the belly, for the hip yaw servos and bearings; the back, for the tail servo.
    # Openings: the floor under the hip yaw servos and bearings; the top of the tail root for the
    # tail servo's horn.
    # (the hips' full footprint: hip yaw servos and bearings, and the hip roll brackets that swing
    # to |y| 40, x -6..29)
    belly = bpy.data.objects.new("Cut_Belly", box_mesh("Cut_Belly", (-12, -42, 80), (32, 42, 107.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(belly)
    boolean_cut(body, belly, TRUNK)
    for sy in (1, -1):  # pockets for the arm servos' cases (x 17..37, |y| 18..44, z 113.5..149.5)
        pk = bpy.data.objects.new(f"Cut_Shoulder{sy}", box_mesh(f"Cut_Shoulder{sy}", (16.4, min(sy * 15.6, sy * 38), 112.9), (37.6, max(sy * 15.6, sy * 38), 153)))
        bpy.data.collections[f"{SCENE}: rig"].objects.link(pk)
        boolean_cut(body, pk, TRUNK)
    for sy in (1, -1):  # windows where the shin brackets swing in on hip yaw and roll: the battery
        # holders are the side walls there, as on Microduck, which keeps its full hip range
        wn = bpy.data.objects.new(f"Cut_LegWindow{sy}", box_mesh(f"Cut_LegWindow{sy}", (-47, min(sy * 23.5, sy * 38), 48), (-16, max(sy * 23.5, sy * 38), 84)))
        bpy.data.collections[f"{SCENE}: rig"].objects.link(wn)
        boolean_cut(body, wn, TRUNK)
    boolean_cut(body, cutter_cylinder("Cut_TailHorn", 10.6, 0, 14, Vector((-74.5, 0, 127.5)), "Z"), TRUNK)
    bpy.context.view_layer.update()
    print("battery corners outside the body:", battery_enclosed())
    return names, HIPS


TAIL_AXIS = Vector((-74.5, 0, 116))  # the tail-yaw XL330's output axis (vertical), behind the battery


def tail_body():
    """A rig body for the tail, on the tail servo's horn, turning about Z (+-40 degrees)."""
    if "B:tail" not in bpy.data.objects or bpy.data.objects["B:tail"].name not in scene.objects:
        e = bpy.data.objects.new("B:tail", None)
        e.empty_display_size = 12
        bpy.data.collections[f"{SCENE}: rig"].objects.link(e)
    e = next(o for o in scene.objects if o.name.split(".")[0] == "B:tail")
    trunk = E(TRUNK)
    local = trunk.matrix_world.inverted() @ Matrix.Translation(TAIL_AXIS)
    e.parent = trunk
    e.matrix_parent_inverse = Matrix.Identity(4)
    e.matrix_basis = local
    e["rest"] = [v for row in local for v in row]
    e["joint"], e["axis"], e["lo"], e["hi"] = "tail_yaw", [0, 0, 1], -0.7, 0.7
    BODIES["tail"] = (TRUNK, local, ("tail_yaw", Vector((0, 0, 1)), (-0.7, 0.7)), [])
    return e


def part_tail():
    """A cream root pod round the tail servo (fixed to the body), and a cute moving tail on its horn:
    a yoke over the top, three rounded beads with black rings, a round lime tip."""
    clear(("Tail", "XL330_TailYaw", "Spike_T"))
    tail_body()
    T = Matrix.Translation
    # The servo, horn up: local X (output) onto +Z, the case reaching forward to just behind the battery.
    xl330("XL330_TailYaw", coll, T(TAIL_AXIS) @ Matrix.Rotation(-math.pi / 2, 4, "Y"))
    fixed = [o for o in coll.objects if o.name.split(".")[0].startswith("XL330_TailYaw")]
    # (The body's tail root encloses the servo; its top has the horn opening.)
    for sy in (1, -1):
        screw_dot(f"TailRootScrew{'L' if sy > 0 else 'R'}", coll, (-84, sy * 14.6, 116), "Y" if sy > 0 else "-Y")
        fixed.append(bpy.data.objects[f"TailRootScrew{'L' if sy > 0 else 'R'}"] if f"TailRootScrew{'L' if sy > 0 else 'R'}" in coll.objects else next(o for o in coll.objects if o.name.split(".")[0] == f"TailRootScrew{'L' if sy > 0 else 'R'}"))
    names = made(fixed, TRUNK)
    # The moving tail, compliant so a fall can't strip the servo:
    #  - a magnetic breakaway coupling on the horn (two 6 x 3 mm magnets, two locating pins): a hard
    #    knock pops the tail off and it snaps back on;
    #  - cream PLA segments joined by black TPU flexure rings, so it bends about 20 degrees and springs
    #    back;
    #  - a soft TPU lime tip.
    # It continues the back line, rising gently and tapering; its front stays 21.5 mm from the yaw
    # axis, clear of the root pod's corners (17.4 mm) over the whole swing.
    TPU = mat("TPU", "#1C1D1F", 0.85)
    moving = [disc("TailCoupling", coll, LIME, 9.8, 6.0, (TAIL_AXIS.x, 0, 130.6), "Z", bevel_w=0.8)]  # (up through the root's horn opening to the arm)
    for k, dy in enumerate((-5.5, 5.5)):
        moving.append(disc(f"TailMagnet{k}", coll, SILVER, 3.0, 0.6, (TAIL_AXIS.x - 3, dy, 136.6), "Z", n=20))
    moving.append(loft_skin("TailArm", coll, CREAM, [(-68, 8.5, 136.5, 142.5), (-93, 11, 136.5, 145.5), (-101, 15, 127, 147)], subsurf=2, keep=0.45))
    # (thick at the root and tapering, sweeping up toward the tip as on the boards)
    # (one taper: each segment starts the size the last one ended)
    segs = [(-99, -127, 19.0, 16.5, 128, 134), (-131, -155, 16.5, 13.5, 135, 144), (-160, -180, 13.5, 10.5, 145.5, 156)]
    for k, (x0, x1, r0, r1, z0, z1) in enumerate(segs):
        moving.append(loft_skin(f"TailSeg{k}", coll, CREAM, [(x0, r0 * 0.95, z0 - r0, z0 + r0 * 0.92), ((x0 + x1) / 2, (r0 + r1) / 2, (z0 + z1) / 2 - (r0 + r1) / 2, (z0 + z1) / 2 + (r0 + r1) / 2 * 0.92), (x1, r1 * 0.95, z1 - r1, z1 + r1 * 0.92)], subsurf=3, keep=0.2))
        for sy in (1, -1):
            moving.append(disc(f"TailScrew{k}{'L' if sy > 0 else 'R'}", coll, INK, 1.5, 1.0, ((x0 + x1) / 2, sy * ((r0 + r1) / 2 - 0.3), (z0 + z1) / 2 + 2), "Y" if sy > 0 else "-Y", n=16, bevel_w=0.4))
        moving.append(spike(f"Spike_T{k}", (x0 + x1) / 2, (z0 + z1) / 2 + (r0 + r1) / 2 * 0.92, 6.0 - k))
        nxt = segs[k + 1] if k + 1 < len(segs) else None
        # (each ring reaches 1.5 mm into both neighbours: bonded, it holds them together)
        x_next = nxt[0] if nxt else -182
        ra = (r1 if nxt is None else (r1 + nxt[2]) / 2) * 0.82
        moving.append(disc(f"TailFlex{k}", coll, TPU, ra, (x1 - x_next) + 3.0, (x1 + 1.5, 0, z1 - 0.5), "-X", bevel_w=1.2))
    moving.append(loft_skin("TailTip", coll, LIME, [(-182, 10.5, 146, 166), (-198, 8.0, 154, 171), (-214, 5.0, 164, 176), (-228, 1.8, 175, 180)], subsurf=3, keep=0.3))
    bpy.context.view_layer.update()
    names += made(moving, "tail")
    return names, ["tail", "upper_leg_left", "upper_leg_right", "hip_l", "hip_l_2"]


HINGE = Vector((26.1, 0, 225.6))  # the jaw servo's axis (XL330_09), along Y
NECK_AXIS = Vector((26.0, 0, 0))   # the head-yaw servo's axis (XL330_08), vertical


def jaw_body():
    """A rig body for the jaw on the jaw servo's axis, opening 0..29 degrees."""
    if "B:jaw" not in [o.name.split(".")[0] for o in scene.objects]:
        e = bpy.data.objects.new("B:jaw", None)
        e.empty_display_size = 10
        bpy.data.collections[f"{SCENE}: rig"].objects.link(e)
    e = next(o for o in scene.objects if o.name.split(".")[0] == "B:jaw")
    head = E(HEAD)
    local = head.matrix_world.inverted() @ Matrix.Translation(HINGE)
    axis = (head.matrix_world.inverted().to_3x3() @ Vector((0, 1, 0))).normalized()
    e.parent = head
    e.matrix_parent_inverse = Matrix.Identity(4)
    e.matrix_basis = local
    e["rest"] = [v for row in local for v in row]
    e["joint"], e["axis"], e["lo"], e["hi"] = "jaw", list(axis), 0.0, 0.5
    BODIES["jaw"] = (HEAD, local, ("jaw", axis, (0.0, 0.5)), [])
    return e


def boolean_cut(o, cutter, body=None, op="DIFFERENCE"):
    """Cut `cutter` out of `o`; the cutter rides with the part's body (or the openings would stay
    put while the part moves) and is not checked itself."""
    if body:
        attach(cutter, body)
        del cutter["body"]
    m = o.modifiers.new(f"Cut_{cutter.name}", "BOOLEAN")
    m.operation = op
    m.object = cutter
    m.solver = "EXACT"
    m.use_self = True  # the hollowed shells self-intersect where they curve tightly
    # Guard: an opening must not swallow the part. Compare with and without this cut; if the
    # result collapses, try the float solver; if that collapses too, leave the cut off and say so.
    def verts(with_cut):
        m.show_viewport = with_cut
        bpy.context.view_layer.update()
        deps = bpy.context.evaluated_depsgraph_get()
        ev = o.evaluated_get(deps)
        k = len(ev.to_mesh().vertices)
        ev.to_mesh_clear()
        return k
    before = verts(False)
    floor = 0.5 if op == "DIFFERENCE" else 0.05  # an intersect keeps a part by design
    if verts(True) < before * floor:
        m.solver = "FLOAT"
        if verts(True) < before * floor:
            m.show_viewport = m.show_render = False
            print(f"WARNING: opening {cutter.name} collapses {o.name}; left off")
            return
        print(f"note: opening {cutter.name} on {o.name} uses the float solver")
    m.show_viewport = True
    cutter.hide_set(True)
    cutter.hide_render = True
    cutter.display_type = "WIRE"


def cutter_cylinder(name, r, z0, z1, at, axis):
    me = cyl_mesh(name, r, z0, z1, 48)
    o = bpy.data.objects.new(name, me)
    bpy.data.collections[f"{SCENE}: rig"].objects.link(o)
    o.matrix_world = Matrix.Translation(at) @ axis_matrix(axis)
    return o


CHAMFER = 6.0  # mm: the shells' edge fillet
# The M12 camera (and the ToF sensor beside it) look out of a brow visor on the head's top, the
# black slot the boards show above the snout: they see forward over the snout, out of harm's way.
CAM_GLASS = Vector((89.9, 0.0, 238.1))  # Microduck's camera: the front of its glass
CAM_TO = Vector((85.0, -4.0, 261.0))    # where that point goes, in the visor
CAM_TILT = math.radians(5)              # looking slightly down, at the floor ahead
CAM_POSE = Matrix.Translation(CAM_TO) @ Matrix.Rotation(CAM_TILT, 4, "Y") @ Matrix.Translation(-CAM_GLASS)


def hemidome(name, material, r, at, depth=0.6, n=40, rings=14):
    """A hollow front half-dome facing +X: the eye's window (r across, `depth` x r deep)."""
    v, f = [], []
    for i in range(rings + 1):
        t = (math.pi / 2) * i / rings
        for k in range(n):
            a = 2 * math.pi * k / n
            v.append((math.cos(t) * r * depth, math.sin(t) * r * math.cos(a), math.sin(t) * r * math.sin(a)))
    for i in range(rings):
        for k in range(n):
            j = (k + 1) % n
            f.append((i * n + k, i * n + j, (i + 1) * n + j, (i + 1) * n + k))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    o = obj_from(name, me, coll, material, Matrix.Translation(at), smooth=True)
    sol = o.modifiers.new("Wall", "SOLIDIFY")
    sol.thickness = 1.2
    sol.offset = -1.0
    return o


def move_camera():
    """Put the M12 camera (holder, lens, glass) in the brow visor: undo whatever pose it was given
    before, apply CAM_POSE (it rides on a CSI ribbon, so it needn't sit on the compute board)."""
    rest_all()
    bpy.context.view_layer.update()
    for o in scene.objects:
        if o.type == "MESH" and not o.get("skin") and o.name.split(".")[0] in ("CameraHolder", "CameraLens", "CameraGlass"):
            if "cam_pose" in o:
                done = Matrix([o["cam_pose"][i * 4:i * 4 + 4] for i in range(4)])
            else:
                done = Matrix.Translation(Vector(o.get("cam_shift", (0, 0, 0))))
            o.matrix_world = CAM_POSE @ done.inverted() @ o.matrix_world
            o["cam_pose"] = [v for row in CAM_POSE for v in row]
            o["cam_shift"] = [0, 0, 0]


def part_head():
    """One lime head shell round Microduck's head hardware: high underneath at the front (the mouth),
    low behind, a bore for the camera, an opening round the neck for the head to turn; glossy eyes,
    the ToF window and board, the head IMU, the speaker grille, the red LED; and a cream jaw on
    its own joint on the jaw servo's axis, in front of the neck opening."""
    clear(("Head", "HeadScrew", "Spike_H", "Eye", "Brow", "Visor", "Cam", "ToF", "HeadIMU", "Grille", "LED", "StatusLED", "Camera", "Cheek", "Jaw", "Mouth", "Tongue", "Fang", "Blush", "Crest", "Chin", "Nostril", "Muzzle"))
    for o in list(scene.objects):
        if o.name.split(".")[0] in ("Cut_Camera", "Cut_Neck", "Cut_EyeL", "Cut_EyeR", "Cut_MuzzleSlab", "Cut_MuzzleCurve", "Cut_ToF", "Cut_VisorBay", "Cut_VisorFloor", "Cut_VisorSlot", "Cut_Hinge", "Cut_BackSlab", "Cut_NeckBack"):
            bpy.data.objects.remove(o, do_unlink=True)
    jaw_body()
    move_camera()
    # The shell: underside at z 204 behind (enclosing the head-roll servo and the jaw servo's case),
    # rising to z 221 in front (above the mouth, below the compute board's lower edge at 223).
    # A wedge: a tall crown behind, the sides one flat facet each from the cheek to the snout (the
    # eye sockets sit on it), a flat snout front carrying the camera window, the ToF window and the
    # status LED - the sensors look out where the boards put them.
    HEAD_SECTIONS = [(-34, 30, 24, 205, 240), (-24, 42, 37, 199, 247), (6, 48, 43, 199, 251), (40, 49, 43, 199, 257), (54, 47, 40, 209, 260.5), (60, 44, 39, 218.6, 261), (98, 26, 21, 223, 250), (105, 20, 16, 226, 249)]
    shell = loft_trap("Head", HEAD_SECTIONS, LIME, facet=CHAMFER)
    hollow(shell)
    # (wide enough for the head to roll 25 degrees over the neck bracket and servo)
    boolean_cut(shell, cutter_cylinder("Cut_Neck", 28.0, 0, 26.0, Vector((NECK_AXIS.x, 0, 186)), "Z"), HEAD)
    parts = [shell]
    # The brow visor: a raised lime ridge over the snout, open into the head below; a smoked strip
    # across its front hides the camera (right of centre) and the ToF sensor (left). The head's top
    # is opened under it for the camera module.
    hcut = bpy.data.objects.new("Cut_VisorBay", box_mesh("Cut_VisorBay", (50, -18.5, 245), (86, 18.5, 290)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(hcut)
    boolean_cut(shell, hcut, HEAD)
    # (a brow ridge: it rises from the crown, runs forward over the eyes and drops to the snout)
    VISOR = [(40, 15, 6, 252, 258), (56, 20, 18.5, 254, 273.8), (80, 20, 18.5, 252, 274.4), (92, 19.5, 15, 248, 270)]
    visor = loft_trap("Visor", VISOR, LIME, facet=4.5)
    hollow(visor, 2.0)
    vfloor = bpy.data.objects.new("Cut_VisorFloor", box_mesh("Cut_VisorFloor", (40, -18.1, 240), (95, 18.1, 256.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(vfloor)
    boolean_cut(visor, vfloor, HEAD)
    vslot = bpy.data.objects.new("Cut_VisorSlot", box_mesh("Cut_VisorSlot", (86, -16, 257.5), (96, 16, 266.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(vslot)
    boolean_cut(visor, vslot, HEAD)
    parts.append(visor)
    for sy, side in ((1, "L"), (-1, "R")):  # the visor screws down into the head on each side
        for k, x in enumerate((62, 82)):
            parts.append(screw(f"HeadScrew_V{side}{k}", (x, sy * (side_y(VISOR, x, 263) - 0.5), 263), "Y" if sy > 0 else "-Y"))
    parts.append(rounded("VisorGlass", coll, mat("Smoke", "#101114", 0.06), (88.9, -16.6, 256.9), (90.3, 16.6, 267.1), 0.5))
    tof = Vector((84.0, 12.0, 262.0))
    parts.append(rounded("ToFBoard", coll, mat("ToFPCB", "#5B2C8F", 0.5), (tof.x - 1.6, tof.y - 4.5, tof.z - 6), (tof.x, 18.4, tof.z + 6), 0.4))  # (screwed to the visor wall)
    parts.append(rounded("ToFSensor", coll, CHIP, (tof.x, tof.y - 3.2, tof.z - 1.5), (tof.x + 1.0, tof.y + 3.2, tof.z + 1.5), 0.3))
    # (and a camera mount: a black cradle from the visor's side wall to the holder)
    parts.append(rounded("CamMount", coll, INK, (60, -18.4, 254.8), (72, -16.2, 258.5), 0.5))
    # The status LED: a red dot on the snout front, as on the boards.
    parts.append(disc("StatusLED", coll, LED_RED, 1.8, 1.0, (104.6, 0, 240.0), "X"))
    # Eye sockets on the side facets: a black angular socket, a glossy black lens, a lime brow.
    A, B, C, D = (Vector(v) for v in ((60, 44, 218.6), (60, 39, 261), (98, 21, 250), (98, 26, 223)))
    for sy, side in ((1, "L"), (-1, "R")):
        m = Vector((1, sy, 1))
        a, b, c, d = (v * 1 for v in (A, B, C, D))
        a.y, b.y, c.y, d.y = sy * A.y, sy * B.y, sy * C.y, sy * D.y
        n = (c - a).cross(b - d).normalized()
        if n.y * sy < 0:
            n = -n
        u = (d - a).normalized()  # along the facet, forward
        v = n.cross(u).normalized()
        if v.z < 0:
            v = -v
        # a point on the facet at the eye: x 82, z 243
        t = (82 - 60) / 38.0
        wb, wt = 44 + (26 - 44) * t, 39 + (21 - 39) * t
        z0, z1 = 218.6 + (223 - 218.6) * t, 261 + (250 - 261) * t
        eye = Vector((82, sy * (wb + (wt - wb) * (243 - z0) / (z1 - z0)), 243))
        frame = Matrix((tuple(u) + (0,), tuple(v) + (0,), tuple(n) + (0,), (0, 0, 0, 1))).transposed()
        frame.translation = eye

        def plate(name, material, outline, h0, h1):
            k = len(outline)
            vs = [(p[0], p[1], h0) for p in outline] + [(p[0], p[1], h1) for p in outline]
            fs = [tuple(range(k))[::-1], tuple(range(k, 2 * k))] + [(i, (i + 1) % k, k + (i + 1) % k, k + i) for i in range(k)]
            me = bpy.data.meshes.new(name)
            me.from_pydata(vs, [], fs)
            o = obj_from(name, me, coll, material, frame)
            soft_edges(o, 0.7, 2)
            return o

        # (u forward along the facet, v up; sockets slant down to the front like the boards')
        parts.append(plate(f"EyeSocket_{side}", INK, [(-17, -6.5), (5, -12), (18, -5), (17, 5.5), (0, 11), (-16, 8)], -0.4, 1.2))
        parts.append(plate(f"Brow_{side}", LIME, [(-19, 9.5), (1, 13), (19, 7), (20, 11.5), (2, 18), (-19, 14)], -0.4, 2.8))
        parts.append(hemidome(f"Eye_{side}", EYE, 8.6, Vector((0, 0, 0)), depth=0.55))
        e = parts[-1]
        e.matrix_world = frame @ Matrix.Translation((2.0, -0.5, 1.0)) @ Matrix.Rotation(-math.pi / 2, 4, "Y")
        parts.append(ball(f"EyeShine_{side}", coll, SHINE, 1.6, frame @ Vector((5.0, 3.0, 5.3))))
    # The cream upper lip: a band below the snout, as on the boards (lime above, cream below),
    # following the head's facets just proud of them.
    # (a panel of the head itself, not a layer on it: the shell is cut where it goes)
    muzzle = loft_trap("Muzzle", HEAD_SECTIONS, CREAM, facet=CHAMFER)
    hollow(muzzle)
    slab = bpy.data.objects.new("Cut_MuzzleSlab", box_mesh("Cut_MuzzleSlab", (56, -70, 218.1), (120, 70, 230.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(slab)
    boolean_cut(muzzle, slab, HEAD, "INTERSECT")
    boolean_cut(shell, slab, HEAD)
    parts.append(muzzle)
    # The back of the head in cream, as on the boards: a cap over the shell behind the cheeks,
    # its front edge a panel seam.
    back = loft_trap("HeadBack", HEAD_SECTIONS, CREAM, facet=CHAMFER)
    hollow(back)
    bslab = bpy.data.objects.new("Cut_BackSlab", box_mesh("Cut_BackSlab", (-60, -70, 190), (4, 70, 300)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(bslab)
    boolean_cut(back, bslab, HEAD, "INTERSECT")
    boolean_cut(back, cutter_cylinder("Cut_NeckBack", 28.0, 0, 26.0, Vector((NECK_AXIS.x, 0, 186)), "Z"), HEAD)
    boolean_cut(shell, bslab, HEAD)
    parts.append(back)
    for sy, side in ((1, "L"), (-1, "R")):  # the back cap's screws, just behind its seam
        for k, z in enumerate((212, 236)):
            parts.append(screw(f"HeadScrew_B{side}{k}", (-2, sy * (side_y(HEAD_SECTIONS, -2, z) - 0.4), z), "Y" if sy > 0 else "-Y"))
    # Head IMU (LSM6DSV16X breakout) on the head frame. (The speaker vents through the mouth.)
    parts.append(rounded("HeadIMU", coll, mat("IMUPCB", "#1D3F8F", 0.5), (10, -26, 237.4), (28, -13, 239.0), 0.4))
    # Dorsal spikes: little lime fins down the crown, the dino's tell (they continue down the
    # back and the tail).
    for k, (x, h) in enumerate(((-22, 7.0), (-6, 8.5), (10, 9.5), (26, 9.0))):
        top = next(z1 + (z1b - z1) * (x - xa) / (xb - xa) for (xa, _, _, _, z1), (xb, _, _, _, z1b) in zip(HEAD_SECTIONS, HEAD_SECTIONS[1:]) if xa <= x <= xb)
        parts.append(spike(f"Spike_H{k}", x, top, h))
    names = made(parts, HEAD)
    # Jaw: cream, x 53..96 (in front of the neck opening and the jaw servo's case), arms up the
    # sides to cheek caps on the hinge axis; a dark mouth floor, a tongue, two rounded fangs on the
    # head above its lip.
    # (The jaw is driven from the servo by a linkage inside the head; the cheek caps and pins mark
    # the hinge axis on the head's sides.)
    # The jaw is the cream lower half of the face, flush with the head's sides, a smile under the
    # snout; behind x 56 the neck opening and the jaw servo's case leave the underside lime.
    jaw = [loft_trap("Jaw", [(61.5, 38, 42, 206, 217.4), (72, 37, 40.5, 204, 217.6), (86, 30, 33, 206, 217.6), (95, 21, 23.5, 209, 217.6), (99.5, 12, 14, 212.5, 217.2)], CREAM, facet=4.0)]
    jaw.append(loft_trap("Mouth", [(64, 33, 34, 216.2, 217.7), (88, 27, 28, 216.2, 217.8), (97, 15, 16, 216.2, 217.8)], INK, facet=0.5))
    jaw.append(rounded("Tongue", coll, mat("Tongue", "#FF7A8A", 0.6), (66, -12, 215.2), (86, 12, 217.4), 1.2))
    # The jaw hangs on its servo: cream arms run back outside the cheeks to the hinge, a shaft
    # through the head wall on each side - onto the jaw servo's horn (left) and an idler boss on the
    # head frame (right) - capped by black hubs, like the boards' exposed joints.
    boolean_cut(shell, cutter_cylinder("Cut_Hinge", 4.2, -60, 60, HINGE, "Y"), HEAD)
    for sy, side in ((1, "L"), (-1, "R")):
        ax = "Y" if sy > 0 else "-Y"
        jaw.append(loft_skin(f"JawArm_{side}", coll, CREAM, [(17, 1.5, 216, 235), (34, 1.5, 213, 231), (50, 1.5, 206, 222), (66, 1.5, 205, 217.2)], subsurf=2, keep=0.3, yc=sy * 51.0))
        jaw.append(rounded(f"JawCheek_{side}", coll, CREAM, (61, min(sy * 39.5, sy * 52.0), 205.5), (70, max(sy * 39.5, sy * 52.0), 217.2), 2.5))
        jaw.append(disc(f"JawShaft_{side}", coll, SILVER, 3.2, 13.0 if sy > 0 else 13.5, (HINGE.x, sy * (39.9 if sy > 0 else 39.4), HINGE.z), ax, n=24))
        jaw.append(disc(f"CheekCap_{side}", coll, INK, 8.5, 3.0, (HINGE.x, sy * 52.4, HINGE.z), ax, n=8, bevel_w=1.0))
        jaw.append(disc(f"CheekPin_{side}", coll, SILVER, 3.2, 1.0, (HINGE.x, sy * 55.3, HINGE.z), ax))
        for k, (x, z) in enumerate(((44, 216.5), (62, 210.5))):
            jaw.append(screw(f"JawScrew_{side}{k}", (x, sy * 52.4, z), ax))
    bpy.context.view_layer.update()
    names += made(jaw, "jaw")
    fangs = []
    for sy, side in ((1, "L"), (-1, "R")):  # just in front of the jaw's lip
        # (small rounded cones: a cute overbite, not buck teeth)
        bpy.ops.mesh.primitive_cone_add(vertices=24, radius1=3.2, radius2=0.7, depth=7.5, location=(102.6, sy * 9.0, 224.9), rotation=(math.pi, 0, 0))
        fang = bpy.context.active_object
        fang.name = f"Fang_{side}"
        for c in fang.users_collection:
            c.objects.unlink(fang)
        coll.objects.link(fang)
        fang.data.materials.append(FANG)
        fang.data.shade_smooth()
        fang.modifiers.new("Soft", "BEVEL").width = 0.4
        fangs.append(fang)
    names += made(fangs, HEAD)
    return names, ["neck", "neck_pitch", "yaw_roll_motion", HEAD, "jaw"]


def pad(name, material, cx, cz, a, b, ang, y0, y1, taper=0.72, n=8, subsurf=2):
    """A soft oval plate on a leg's outer side: an ellipse (half axes a along the tilt `ang`, b
    across) centred at (cx, cz), from y0 (against the hardware) out to y1, domed outward."""
    import bmesh
    rings = [(y0, 1.0), (y0 + 0.3 * (y1 - y0), 0.97), (y0 + 0.72 * (y1 - y0), 0.82), (y1, taper)]
    ca, sa = math.cos(ang), math.sin(ang)
    v = []
    for y, k in rings:
        for i in range(n):
            t = 2 * math.pi * (i + 0.5) / n
            u, w = a * k * math.cos(t), b * k * math.sin(t)
            v.append((cx + u * ca - w * sa, y, cz + u * sa + w * ca))
    last = len(rings) - 1
    f = [tuple(range(n))[::-1], tuple(range(last * n, (last + 1) * n))]
    for r in range(last):
        for i in range(n):
            j = (i + 1) % n
            f.append((r * n + i, r * n + j, (r + 1) * n + j, (r + 1) * n + i))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    o = obj_from(name, me, coll, material, smooth=True)
    sub = o.modifiers.new("Subsurf", "SUBSURF")
    sub.levels = sub.render_levels = subsurf
    return o


def spike(name, x, base, h, width=None, y=0.0, thick=4.4):
    """A small rounded dorsal fin, leaning back, on a surface at height `base` (centre line)."""
    w = width or h * 1.25
    o = slab(name, LIME, [(x - w * 0.6, base - 1.0), (x + w * 0.4, base - 1.0), (x + w * 0.05, base + h * 0.55), (x - w * 0.3, base + h), (x - w * 0.55, base + h * 0.75)],
             y - thick / 2, y + thick / 2, chamfer=1.4)
    return o


def hull2d(pts):
    pts = sorted(set(pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, hi = half(pts), half(pts[::-1])
    return lo[:-1] + hi[:-1]


def teardrop(c1, r1, c2, r2, n=20):
    """The outline round two circles (x, z centres): a teardrop, broad at c1."""
    pts = []
    for (cx, cz), r in ((c1, r1), (c2, r2)):
        for i in range(n):
            t = 2 * math.pi * i / n
            pts.append((round(cx + r * math.cos(t), 3), round(cz + r * math.sin(t), 3)))
    return hull2d(pts)


def shield(name, material, outline, y0, y1, dome=(1.0, 0.96, 0.84, 0.62)):
    """A domed armour plate: the outline (XZ) at y0 against the hardware, shrinking through three
    rings to a flat top at y1. Returns the object and a function giving a point on its top face."""
    cx = sum(p[0] for p in outline) / len(outline)
    cz = sum(p[1] for p in outline) / len(outline)
    ys = (y0, y0 + 0.35 * (y1 - y0), y0 + 0.72 * (y1 - y0), y1)
    n = len(outline)
    v = []
    for y, k in zip(ys, dome):
        v += [(cx + (x - cx) * k, y, cz + (z - cz) * k) for x, z in outline]
    f = [tuple(range(n)), tuple(range(3 * n, 4 * n))]
    for r in range(3):
        f += [(r * n + i, r * n + (i + 1) % n, (r + 1) * n + (i + 1) % n, (r + 1) * n + i) for i in range(n)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    o = obj_from(name, me, coll, material, smooth=True)
    sub = o.modifiers.new("Smooth", "SUBSURF")
    sub.levels = sub.render_levels = 2
    o["keep_round"] = True
    return o, (lambda fx, fz: (cx + fx, cz + fz))


LEG_JOINTS = {"L": ["yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left"], "R": ["bearing_roll", "hip_l_2", "upper_leg_right", "leg_2", "ankle_right"]}


def part_legs():
    """Chamfered lime armour plates on the thighs (the ankle bracket sweeps the shins, so they stay bare), cream
    feet on black soles with three lime toe caps; the servos stay in view at every joint.
    Measured (left; the right mirrors): thigh hardware x -45..16, y 39..71, z 55..103, the knee
    axis at (-31.8, z 67.5); the shin servo x -42..-22, y 37..65, z 16..50; the ankle bracket
    x -46..-10, y 30..70, z 9..34; Microduck's sole x -52..2, y 29..70."""
    clear(("Thigh", "Shin", "Foot", "Toe", "Sole", "Knee"))
    names = []
    tilt = math.atan2(22.0, 35.8)  # the thigh's line, hip pitch axis to knee
    for side, sy in (("L", 1), ("R", -1)):
        up, shin, ankle = SIDE[side]
        # (a domed teardrop shield, broad over the hip, narrowing down the thigh to the knee, as
        # on the arm board; three screws on its crown)
        ca, sa = math.cos(tilt), math.sin(tilt)
        hip, knee = (-14 + 13 * ca, 79 + 13 * sa), (-14 - 25 * ca, 79 - 25 * sa)
        plate, at = shield(f"Thigh_{side}", LIME, teardrop(hip, 31.0, knee, 15.0), sy * 70.6, sy * 81.0)
        thigh = [plate]
        for k, (u, w) in enumerate(((10, 9), (10, -9), (-12, 0))):
            x, z = at(u * ca - w * sa, u * sa + w * ca)
            thigh.append(screw(f"ThighScrew_{side}{k}", (x, sy * 80.6, z), "Y" if sy > 0 else "-Y"))
        names += made(thigh, up)
        yf = sy * 50
        foot = [rounded(f"Sole_{side}", coll, RUBBER, (-53, yf - 21.5, 0), (3, yf + 21.5, 3.6), 1.6),
                loft_skin(f"Foot_{side}", coll, CREAM, [(-52, 20, 3, 8.2), (-30, 21, 3, 8.4), (-4, 20.5, 3, 8.2)], subsurf=2, keep=0.4, yc=yf)]
        for t in (-1, 1):  # two chunky toes, lime caps on black soles
            yt = yf + t * 10.5
            foot.append(loft_skin(f"ToeCap_{side}{t + 1}", coll, LIME, [(-10, 9.5, 3, 15), (4, 9.0, 3, 14), (13, 6.0, 3, 8)], subsurf=2, keep=0.45, yc=yt))
            foot.append(rounded(f"ToeSole_{side}{t + 1}", coll, RUBBER, (-10, yt - 9.5, 0), (12, yt + 9.5, 3.4), 1.4))
        names += made(foot, ankle)
    return names, LEG_JOINTS["L"] + LEG_JOINTS["R"]


def oval_ring(name, material, at, a, b, r, n=40, m=12):
    """A soft oval ring (a torus with an oval centre line, half axes a along X, b along Y; tube r)."""
    v, f = [], []
    for i in range(n):
        t = 2 * math.pi * i / n
        c = Vector((a * math.cos(t), b * math.sin(t), 0))
        out = Vector((b * math.cos(t), a * math.sin(t), 0)).normalized()
        for k in range(m):
            u = 2 * math.pi * k / m
            v.append(tuple(at + c + out * (r * math.cos(u)) + Vector((0, 0, r * math.sin(u)))))
    for i in range(n):
        for k in range(m):
            f.append((i * m + k, ((i + 1) % n) * m + k, ((i + 1) % n) * m + (k + 1) % m, i * m + (k + 1) % m))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    o = obj_from(name, me, coll, material)
    b = o.modifiers.new("Chamfer", "BEVEL")
    b.width, b.segments = 1.2, 1
    return o


def part_dress():
    """A lime collar round the neck servos (on the neck: it pitches with them) and two small
    T-rex arms tucked in front of the chest (passive, on friction pins: the head does the
    expression, the arms just complete the silhouette)."""
    clear(("NeckCollar", "Throat", "CollarScrew", "ThroatScrew"))
    # (a cream frame clamped round the upper neck servo's case, x 16..36, |y| 13)
    z0, z1 = 170.5, 175.5
    # (and a cream throat plate down the front of the lower neck servo, the boards' neck armour)
    names = made([rounded("Throat", coll, CREAM, (36.5, -12, 147), (40.0, 12, 170.4), 1.6),
                  rounded("NeckCollar0", coll, CREAM, (36.1, -17.5, z0), (40.5, 17.5, z1), 1.9),
                  rounded("NeckCollar1", coll, CREAM, (11.5, -17.5, z0), (15.9, 17.5, z1), 1.9),
                  rounded("NeckCollar2", coll, CREAM, (15.9, 13.1, z0), (36.1, 17.5, z1), 1.9),
                  rounded("NeckCollar3", coll, CREAM, (15.9, -17.5, z0), (36.1, -13.1, z1), 1.9)]
                 + [screw(f"ThroatScrew{k}", (40.0, dy, z), "X") for k, (dy, z) in enumerate(((-7, 152), (7, 152), (-7, 165), (7, 165)))]
                 + [screw(f"CollarScrew{k}", (40.5, dy, 173), "X") for k, dy in enumerate((-12, 12))], "neck")
    return names, ["neck", "neck_pitch", "yaw_roll_motion"] + LEG_JOINTS["L"][:3] + LEG_JOINTS["R"][:3]


def screw(name, at, axis):
    """A visible M2 button-head screw (black), its head on the surface at `at`."""
    screw_dot(name, coll, at, axis)
    return next(o for o in coll.objects if o.name.split(".")[0] == name)


def side_top(sections, x):
    """A loft_trap's top (centre line) at x."""
    for (xa, _, _, _, za), (xb, _, _, _, zb) in zip(sections, sections[1:]):
        if xa <= x <= xb:
            return za + (zb - za) * (x - xa) / (xb - xa)
    raise ValueError(x)


def side_y(sections, x, z):
    """Half width of a loft_trap at (x, z): its side wall's |y| there (between sections)."""
    for (xa, wba, wta, z0a, z1a), (xb, wbb, wtb, z0b, z1b) in zip(sections, sections[1:]):
        if xa <= x <= xb:
            t = (x - xa) / (xb - xa)
            wb, wt = wba + (wbb - wba) * t, wta + (wtb - wta) * t
            z0, z1 = z0a + (z0b - z0a) * t, z1a + (z1b - z1a) * t
            return wb + (wt - wb) * (z - z0) / (z1 - z0)
    raise ValueError(x)


def slab(name, material, outline, y0, y1, chamfer=2.2):
    """A chamfered plate: a polygon outline in the XZ plane, from y0 to y1."""
    k = len(outline)
    vs = [(x, y0, z) for x, z in outline] + [(x, y1, z) for x, z in outline]
    fs = [tuple(range(k)), tuple(range(k, 2 * k))[::-1]] + [(i, k + i, k + (i + 1) % k, (i + 1) % k) for i in range(k)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(vs, [], fs)
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    o = obj_from(name, me, coll, material)
    soft_edges(o, chamfer)
    return o


SHOULDER = {"L": Vector((27.0, 44.0, 140.0)), "R": Vector((27.0, -44.0, 140.0))}  # each arm servo's output (horn face)


def arm_body(side):
    """A rig body for an arm on its shoulder servo's horn, pitching about Y."""
    sy = 1 if side == "L" else -1
    name = f"arm_{side}"
    if f"B:{name}" not in [o.name.split(".")[0] for o in scene.objects]:
        e = bpy.data.objects.new(f"B:{name}", None)
        e.empty_display_size = 10
        bpy.data.collections[f"{SCENE}: rig"].objects.link(e)
    e = next(o for o in scene.objects if o.name.split(".")[0] == f"B:{name}")
    trunk = E(TRUNK)
    local = trunk.matrix_world.inverted() @ Matrix.Translation(SHOULDER[side])
    axis = (trunk.matrix_world.inverted().to_3x3() @ Vector((0, sy, 0))).normalized()
    e.parent = trunk
    e.matrix_parent_inverse = Matrix.Identity(4)
    e.matrix_basis = local
    e["rest"] = [v for row in local for v in row]
    # (raised forward to reach, 75 degrees; back to tuck, 25 - then the forearm meets the thigh);
    # the right arm's axis points the other way, so its range mirrors
    lo, hi = (-1.3, 0.45) if sy > 0 else (-0.45, 1.3)
    e["joint"], e["axis"], e["lo"], e["hi"] = f"{side.lower()}_shoulder", list(axis), lo, hi
    BODIES[name] = (TRUNK, local, (f"{side.lower()}_shoulder", axis, (lo, hi)), [])
    return e


def part_arms():
    """Two small arms, each on its own XL330 at the shoulder (as on the arm concept board: reach,
    tuck, wave). The servo case sits in a pocket in the body's upper side, horn out; the arm - a
    cream upper arm, a black elbow hub on a fixed bent elbow, a cream forearm, a lime hand with
    two claws - is screwed to the horn."""
    clear(("XL330_Arm", "ArmUpper", "ArmFore", "ArmElbow", "ArmPin", "Hand", "Claw", "ArmScrew", "Shoulder"))
    names = []
    for side, sy in (("L", 1), ("R", -1)):
        arm_body(side)
        c = SHOULDER[side] - Vector((0, sy * 13.0, 0))  # the case's centre on the output axis
        world = Matrix(((0, -sy, 0, c.x), (sy, 0, 0, c.y), (0, 0, 1, c.z), (0, 0, 0, 1)))
        xl330(f"XL330_Arm{side}", coll, world)
        names += made([o for o in coll.objects if o.name.split(".")[0].startswith(f"XL330_Arm{side}")], TRUNK)
        y0 = sy * 45.6
        def ys(a, b):
            return (sy * a, sy * b) if sy > 0 else (sy * b, sy * a)
        arm = [slab(f"ArmUpper_{side}", CREAM, [(16.5, 144), (22, 151.5), (33, 151.5), (38.5, 144), (44, 113), (38, 104), (29.5, 106)], *ys(45.6, 54.6)),
               disc(f"ArmElbow_{side}", coll, INK, 6.0, 2.0, (38, sy * 54.6, 110), "Y" if sy > 0 else "-Y", n=8, bevel_w=0.6),
               disc(f"ArmPin_{side}", coll, SILVER, 2.2, 0.8, (38, sy * 56.6, 110), "Y" if sy > 0 else "-Y", n=16),
               slab(f"ArmFore_{side}", CREAM, [(33, 116), (60, 118.5), (64, 111), (60, 103.5), (35, 102)], *ys(46.4, 53.8)),
               slab(f"Hand_{side}", LIME, [(58, 121), (69, 119.5), (72.5, 111), (68.5, 103), (58, 102)], *ys(45.9, 54.3))]
        for k, a in enumerate((math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)):
            arm.append(screw(f"ArmScrew_{side}{k}", (27 + 5.6 * math.cos(a), sy * 54.6, 140 + 5.6 * math.sin(a)), "Y" if sy > 0 else "-Y"))
        for k, dy in enumerate((2.6, 7.4)):
            bpy.ops.mesh.primitive_cone_add(vertices=6, radius1=2.1, radius2=0.5, depth=8.0, location=(74.5, sy * (45.9 + dy), 108.5), rotation=(0, math.radians(115), 0))
            claw = bpy.context.active_object
            claw.name = f"Claw_{side}{k}"
            for cc in claw.users_collection:
                cc.objects.unlink(claw)
            coll.objects.link(claw)
            claw.data.materials.append(CREAM)
            arm.append(claw)
        names += made(arm, f"arm_{side}")
    joints = ["arm_L", "arm_R", "neck", "neck_pitch", "yaw_roll_motion", HEAD, "jaw"] + LEG_JOINTS["L"][:3] + LEG_JOINTS["R"][:3]
    return names, joints


def part_all():
    """No new part: check every shell against every joint (the extra tail and jaw included)."""
    names = [o.name.split(".")[0] for o in coll.objects if o.get("skin")]
    return names, [b for b, (_, _, j, _) in BODIES.items() if j]


BUILDERS = {"body": part_body, "tail": part_tail, "head": part_head, "legs": part_legs, "dress": part_dress, "arms": part_arms, "all": part_all}
for _side in ("L", "R"):
    if PART != "arms" and f"B:arm_{_side}" in [o.name.split(".")[0] for o in scene.objects]:
        arm_body(_side)
if PART != "tail" and "B:tail" in [o.name.split(".")[0] for o in scene.objects]:
    tail_body()
if PART != "head" and "B:jaw" in [o.name.split(".")[0] for o in scene.objects]:
    jaw_body()
names, joints = BUILDERS[PART]()
check = part_check(names, joints)
files = review(names, f"v4-{PART}-alone", isolate=True)
ctx = {"head": (Vector((40, 0, 225)), 260), "legs": (Vector((-20, 0, 60)), 380), "dress": (Vector((10, 0, 140)), 360), "arms": (Vector((30, 0, 130)), 300)}.get(PART, (Vector((-30, 0, 120)), 560))
files += review(names, f"v4-{PART}-context", target=ctx[0], distance=ctx[1], isolate=False, views=("three_q", "front", "side"))
print(json.dumps({"part": PART, "objects": len(names), "check": check, "renders": files}, indent=1))
