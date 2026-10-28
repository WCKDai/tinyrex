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


def facet(o, width=2.5):
    """Turn a subdivided shell into the boards' hard-surface panel: drop the smoothing, chamfer its
    edges (before its wall and openings), flat shading."""
    subs = [m for m in o.modifiers if m.type == "SUBSURF" or m.name == "Hold"]
    if not subs or any(m.name == "Chamfer" for m in o.modifiers):
        return
    for m in subs:
        o.modifiers.remove(m)
    b = o.modifiers.new("Chamfer", "BEVEL")
    b.width, b.segments, b.limit_method = width, 1, "ANGLE"
    o.modifiers.move(len(o.modifiers) - 1, 0)
    for poly in o.data.polygons:
        poly.use_smooth = False


def made(objs, body=None):
    for o in objs:
        if o.type == "MESH" and o.name.split(".")[0] != "NeckCollar":
            facet(o)
        if o.name not in coll.objects:
            for c in o.users_collection:
                c.objects.unlink(o)
            coll.objects.link(o)
        attach(o, body)
        if not o.name.split(".")[0].startswith("XL330"):
            o["skin"] = True
    return [o.name.split(".")[0] for o in objs]


WALL = 2.4  # printed shell wall, mm


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
        # The concept boards' look: flat panels with chamfered edges, not a pebble.
        b = o.modifiers.new("Chamfer", "BEVEL")
        b.width, b.segments = facet, 1
        b.limit_method = "ANGLE"
        b.harden_normals = False
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


def part_body():
    """A rounded pebble inside Microduck's body-shell room (x -46..35, |y| <= 35, z >= 104), and a
    narrow pod below the back for the battery's lower half (x -47..-25, |y| 19, down to z 67),
    between the thighs (|y| >= 42)."""
    for o in list(scene.objects):
        if o.name.split(".")[0] in ("Cut_Belly", "Cut_TailSlot", "Cut_TailHorn"):
            bpy.data.objects.remove(o, do_unlink=True)
    clear(("Body", "BatteryPod", "Backpack", "PackScrew", "BatteryDoor", "ChargePort", "PowerSwitch"))
    objs = [
        # One smooth body: full above (|y| <= 34), narrow below the back (|y| <= 22) round the
        # battery's lower half, so the thighs swinging in on hip roll pass it.
        # Round the trunk's hardware (measured): battery x -47..-25 |y| 20 z 68..139, its holders
        # x -31..-14 |y| 27 z 63..146, the hip yaw servos x -21..16 |y| 28 z 102..134, the trunk
        # plate |y| 29 z 106..109; the back narrows into the tail root round the tail servo
        # (x -84..-48 |y| 10 z 102..131), its back 14.5 mm from the tail's yaw axis.
        hollow(loft_trap("Body", [(-90, 14.5, 15, 97, 134), (-76, 17, 17.5, 95, 134.5), (-62, 22, 23, 62, 137), (-50, 27, 29, 56, 149), (-34, 31, 33, 54, 152), (-14, 31, 34, 54, 154), (-9, 31, 34, 70, 154), (-6, 33, 34, 99, 154), (17, 33, 33, 99, 151), (36, 23, 24, 110, 143)], CREAM, subsurf=3, keep=0.28), even=False),
        loft_skin("Backpack", coll, LIME, [(-46, 19, 150, 157), (-22, 23, 152, 160), (2, 20, 151, 158.5)], subsurf=2, keep=0.42),
        # The battery swaps through a door underneath; USB-C and the power switch on the tail root.
        rounded("BatteryDoor", coll, mat("Seam", "#BDB6A8", 0.6), (-46, -15, 56.8), (-27, 15, 58.6), 1.6),
        rounded("ChargePort", coll, INK, (-80, -17.6, 108), (-73, -15.4, 112), 1.0),
        rounded("PowerSwitch", coll, LIME, (-80, 15.4, 107.5), (-73, 18, 112.5), 1.3),
    ]
    for k, x in enumerate((-34, -6)):
        for sy in (1, -1):
            screw_dot(f"PackScrew{k}{'L' if sy > 0 else 'R'}", coll, (x, sy * 15, 159.6 if k else 157.8), "Z")
            objs.append(bpy.data.objects[f"PackScrew{k}{'L' if sy > 0 else 'R'}"])
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
    boolean_cut(body, cutter_cylinder("Cut_TailHorn", 10.2, 0, 8, Vector((-74.5, 0, 127.5)), "Z"), TRUNK)
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
    clear(("Tail", "XL330_TailYaw"))
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
    moving = [disc("TailCoupling", coll, LIME, 9.8, 3.0, (TAIL_AXIS.x, 0, 130.6), "Z", bevel_w=0.8)]
    for k, dy in enumerate((-5.5, 5.5)):
        moving.append(disc(f"TailMagnet{k}", coll, SILVER, 3.0, 0.6, (TAIL_AXIS.x - 3, dy, 133.6), "Z", n=20))
    moving.append(loft_skin("TailArm", coll, CREAM, [(-68, 8.5, 136.5, 142), (-88, 9, 136.5, 143), (-102, 10, 131, 143)], subsurf=2, keep=0.45))
    segs = [(-99, -127, 17.5, 15.5, 127, 130), (-131, -155, 14.5, 12.5, 131, 134), (-160, -180, 11.5, 9.5, 135, 138)]
    for k, (x0, x1, r0, r1, z0, z1) in enumerate(segs):
        moving.append(loft_skin(f"TailSeg{k}", coll, CREAM, [(x0, r0 * 0.95, z0 - r0, z0 + r0 * 0.92), ((x0 + x1) / 2, (r0 + r1) / 2, (z0 + z1) / 2 - (r0 + r1) / 2, (z0 + z1) / 2 + (r0 + r1) / 2 * 0.92), (x1, r1 * 0.95, z1 - r1, z1 + r1 * 0.92)], subsurf=3, keep=0.2))
        for sy in (1, -1):
            moving.append(disc(f"TailScrew{k}{'L' if sy > 0 else 'R'}", coll, INK, 1.5, 1.0, ((x0 + x1) / 2, sy * ((r0 + r1) / 2 - 0.3), (z0 + z1) / 2 + 2), "Y" if sy > 0 else "-Y", n=16, bevel_w=0.4))
        nxt = segs[k + 1] if k + 1 < len(segs) else None
        xa = x1 - 2.5
        ra = (r1 if nxt is None else (r1 + nxt[2]) / 2) * 0.82
        moving.append(disc(f"TailFlex{k}", coll, TPU, ra, 5, (xa, 0, z1 - 0.5), "-X", bevel_w=1.6))
    moving.append(loft_skin("TailTip", coll, LIME, [(-182, 9.5, 128.5, 147), (-200, 7.5, 134, 150), (-218, 4.5, 142, 155), (-233, 1.6, 152, 157)], subsurf=3, keep=0.3))
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


CHAMFER = 3.5  # mm: the shells' edge chamfer
CAM_SHIFT = Vector((0, 0, 0))  # the M12 camera stays on Microduck's centre line, behind the snout window
CAM_AT = Vector((105.0, 0.0, 238.1))  # its window in the snout front (the lens axis, at the face)


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
    """Put the M12 camera (holder, lens, glass) behind the right eye, once."""
    rest_all()
    bpy.context.view_layer.update()
    for o in scene.objects:
        if o.type == "MESH" and not o.get("skin") and o.name.split(".")[0] in ("CameraHolder", "CameraLens", "CameraGlass"):
            done = Vector(o.get("cam_shift", (0, 0, 0)))
            o.matrix_world = Matrix.Translation(CAM_SHIFT - done) @ o.matrix_world
            o["cam_shift"] = list(CAM_SHIFT)


def part_head():
    """One lime head shell round Microduck's head hardware: high underneath at the front (the mouth),
    low behind, a bore for the camera, an opening round the neck for the head to turn; glossy eyes,
    the ToF window and board, the head IMU, the speaker grille, the red LED; and a cream jaw on
    its own joint on the jaw servo's axis, in front of the neck opening."""
    clear(("Head", "Eye", "Brow", "Cam", "ToF", "HeadIMU", "Grille", "LED", "StatusLED", "Camera", "Cheek", "Jaw", "Mouth", "Tongue", "Fang", "Blush", "Crest", "Chin", "Nostril", "Muzzle"))
    for o in list(scene.objects):
        if o.name.split(".")[0] in ("Cut_Camera", "Cut_Neck", "Cut_EyeL", "Cut_EyeR", "Cut_MuzzleSlab", "Cut_MuzzleCurve", "Cut_ToF"):
            bpy.data.objects.remove(o, do_unlink=True)
    jaw_body()
    move_camera()
    # The shell: underside at z 204 behind (enclosing the head-roll servo and the jaw servo's case),
    # rising to z 221 in front (above the mouth, below the compute board's lower edge at 223).
    # A wedge: a tall crown behind, the sides one flat facet each from the cheek to the snout (the
    # eye sockets sit on it), a flat snout front carrying the camera window, the ToF window and the
    # status LED - the sensors look out where the boards put them.
    HEAD_SECTIONS = [(-34, 30, 24, 205, 248), (-24, 42, 34, 199, 258), (6, 48, 40, 199, 263), (40, 49, 41, 199, 263), (54, 47, 40, 209, 262), (60, 44, 39, 218.6, 261), (98, 26, 21, 223, 250), (105, 20, 16, 226, 249)]
    shell = loft_trap("Head", HEAD_SECTIONS, LIME, facet=CHAMFER)
    hollow(shell)
    # (wide enough for the head to roll 25 degrees over the neck bracket and servo)
    boolean_cut(shell, cutter_cylinder("Cut_Neck", 28.0, 0, 26.0, Vector((NECK_AXIS.x, 0, 186)), "Z"), HEAD)
    parts = [shell]
    # Snout front: the camera looks through a bore to its glass (x 90), behind a black bezel; a
    # black tube lines the bore. The ToF sensor sits in its own window left of it, the LED right.
    boolean_cut(shell, cutter_cylinder("Cut_Camera", 5.8, 0, 22, CAM_AT - Vector((18, 0, 0)), "X"), HEAD)
    parts.append(part("CamBezel", coll, INK, cyl_mesh("CamBezel", 8.0, 0, 1.6, 48, r_in=5.6), Matrix.Translation(CAM_AT - Vector((0.4, 0, 0))) @ axis_matrix("X"), Matrix.Identity(4), smooth=True))
    parts.append(part("CamTube", coll, INK, cyl_mesh("CamTube", 5.6, 0, 14.5, 32, r_in=5.0), Matrix.Translation((90.2, CAM_AT.y, CAM_AT.z)) @ axis_matrix("X"), Matrix.Identity(4), smooth=True))
    tof = Vector((CAM_AT.x, 11.5, CAM_AT.z))
    tcut = bpy.data.objects.new("Cut_ToF", box_mesh("Cut_ToF", (96, tof.y - 2.6, tof.z - 3.6), (110, tof.y + 2.6, tof.z + 3.6)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(tcut)
    boolean_cut(shell, tcut, HEAD)
    parts.append(rounded("ToFWindow", coll, mat("ToFGlass", "#141418", 0.08), (tof.x - 0.6, tof.y - 3.4, tof.z - 4.4), (tof.x + 0.7, tof.y + 3.4, tof.z + 4.4), 0.6))
    parts.append(rounded("ToFBoard", coll, mat("ToFPCB", "#5B2C8F", 0.5), (tof.x - 5.4, tof.y - 4.5, tof.z - 6), (tof.x - 3.8, tof.y + 4.5, tof.z + 6), 0.4))
    parts.append(disc("StatusLED", coll, LED_RED, 1.8, 1.4, (CAM_AT.x - 0.4, -11.5, CAM_AT.z), "X"))
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
            bv = o.modifiers.new("Chamfer", "BEVEL")
            bv.width, bv.segments = 0.6, 1
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
    muzzle = loft_trap("Muzzle", [(x, wb + 0.8, wt + 0.8, z0 - 0.5, z1 + 0.8) for x, wb, wt, z0, z1 in HEAD_SECTIONS], CREAM, facet=CHAMFER)
    hollow(muzzle, 1.6)
    slab = bpy.data.objects.new("Cut_MuzzleSlab", box_mesh("Cut_MuzzleSlab", (56, -70, 218.1), (120, 70, 230.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(slab)
    boolean_cut(muzzle, slab, HEAD, "INTERSECT")
    parts.append(muzzle)
    # Head IMU (LSM6DSV16X breakout) on the head frame. (The speaker vents through the mouth.)
    parts.append(rounded("HeadIMU", coll, mat("IMUPCB", "#1D3F8F", 0.5), (10, -26, 238), (28, -13, 239.6), 0.4))
    names = made(parts, HEAD)
    # Jaw: cream, x 53..96 (in front of the neck opening and the jaw servo's case), arms up the
    # sides to cheek caps on the hinge axis; a dark mouth floor, a tongue, two rounded fangs on the
    # head above its lip.
    # (The jaw is driven from the servo by a linkage inside the head; the cheek caps and pins mark
    # the hinge axis on the head's sides.)
    # The jaw is the cream lower half of the face, flush with the head's sides, a smile under the
    # snout; behind x 56 the neck opening and the jaw servo's case leave the underside lime.
    jaw = [loft_trap("Jaw", [(61.5, 38, 42, 206, 217.4), (72, 37, 40.5, 204, 217.6), (86, 30, 33, 206, 217.6), (95, 21, 23.5, 209, 217.6), (99.5, 12, 14, 212.5, 217.2)], CREAM, facet=2.5)]
    jaw.append(loft_trap("Mouth", [(64, 33, 34, 216.2, 217.7), (88, 27, 28, 216.2, 217.8), (97, 15, 16, 216.2, 217.8)], INK, facet=0.5))
    jaw.append(rounded("Tongue", coll, mat("Tongue", "#FF7A8A", 0.6), (66, -12, 215.2), (86, 12, 217.4), 1.2))
    for sy, side in ((1, "L"), (-1, "R")):
        caps = [disc(f"CheekCap_{side}", coll, INK, 8.5, 3.0, (HINGE.x, sy * 49.6, HINGE.z), "Y" if sy > 0 else "-Y", bevel_w=1.4),
                disc(f"CheekPin_{side}", coll, SILVER, 3.2, 1.0, (HINGE.x, sy * 52.6, HINGE.z), "Y" if sy > 0 else "-Y")]
        names += made(caps, HEAD)
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


LEG_JOINTS = {"L": ["yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left"], "R": ["bearing_roll", "hip_l_2", "upper_leg_right", "leg_2", "ankle_right"]}


def part_legs():
    """Lime drumstick plates over the thighs, cream shin guards on the knee-to-ankle servo, cream
    feet on black soles with three lime toe caps; the servos stay in view at every joint.
    Measured (left; the right mirrors): thigh hardware x -45..16, y 39..71, z 55..103, the knee
    axis at (-31.8, z 67.5); the shin servo x -42..-22, y 37..65, z 16..50; the ankle bracket
    x -46..-10, y 30..70, z 9..34; Microduck's sole x -52..2, y 29..70."""
    clear(("Thigh", "Shin", "Foot", "Toe", "Sole", "Knee"))
    names = []
    tilt = math.atan2(22.0, 35.8)  # the thigh's line, hip pitch axis to knee
    for side, sy in (("L", 1), ("R", -1)):
        up, shin, ankle = SIDE[side]
        # (a chamfered square armour plate, as on the boards, a screw in each corner)
        thigh = [pad(f"Thigh_{side}", LIME, -14, 79, 44, 33, tilt, sy * 71.8, sy * 80, taper=0.86, n=4)]
        ca, sa = math.cos(tilt), math.sin(tilt)
        for k, (u, w) in enumerate(((19, 14), (-19, 14), (-19, -14), (19, -14))):
            x, z = -14 + u * ca - w * sa, 79 + u * sa + w * ca
            screw_dot(f"ThighScrew_{side}{k}", coll, (x, sy * 79.6, z), "Y" if sy > 0 else "-Y")
            thigh.append(next(o for o in coll.objects if o.name.split(".")[0] == f"ThighScrew_{side}{k}"))
        names += made(thigh, up)
        names += made([pad(f"Shin_{side}", CREAM, -32, 31, 13, 19, 0.0, sy * 68.4, sy * 71.3, taper=0.8)], shin)
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
    clear(("NeckCollar", "Arm", "Hand", "Claw", "Shoulder"))
    names = made([oval_ring("NeckCollar", INK, Vector((26, 0, 173.2)), 19.5, 23.5, 4.6, n=8, m=4)], "neck")
    arm = []
    for side, sy in (("L", 1), ("R", -1)):
        arm.append(disc(f"Shoulder_{side}", coll, INK, 9.0, 4.0, (25, sy * 30.0, 128), "Y" if sy > 0 else "-Y", n=8, bevel_w=1.0))
        yc = sy * 37.0
        arm.append(loft_skin(f"ArmUpper_{side}", coll, CREAM, [(22, 4.8, 116, 134), (34, 4.4, 108, 124)], subsurf=2, keep=0.3, yc=yc))
        arm.append(disc(f"ArmElbow_{side}", coll, INK, 5.0, 11.0, (34, yc - 5.5, 113), "Y", bevel_w=0.8))
        arm.append(loft_skin(f"ArmFore_{side}", coll, CREAM, [(33, 4.2, 108, 118), (49, 3.8, 110.5, 119)], subsurf=2, keep=0.3, yc=yc))
        arm.append(loft_skin(f"Hand_{side}", coll, LIME, [(47, 5.0, 108, 121), (56, 4.4, 109.5, 119.5)], subsurf=2, keep=0.3, yc=yc))
        for c, dy in enumerate((-2.6, 2.6)):
            bpy.ops.mesh.primitive_cone_add(vertices=6, radius1=1.9, radius2=0.4, depth=7.0, location=(58.5, yc + dy, 110.5), rotation=(0, math.radians(150), 0))
            claw = bpy.context.active_object
            claw.name = f"Claw_{side}{c}"
            for cc in claw.users_collection:
                cc.objects.unlink(claw)
            coll.objects.link(claw)
            claw.data.materials.append(CREAM)
            arm.append(claw)
    names += made(arm, TRUNK)
    return names, ["neck", "neck_pitch", "yaw_roll_motion"] + LEG_JOINTS["L"][:3] + LEG_JOINTS["R"][:3]


def part_all():
    """No new part: check every shell against every joint (the extra tail and jaw included)."""
    names = [o.name.split(".")[0] for o in coll.objects if o.get("skin")]
    return names, [b for b, (_, _, j, _) in BODIES.items() if j]


BUILDERS = {"body": part_body, "tail": part_tail, "head": part_head, "legs": part_legs, "dress": part_dress, "all": part_all}
if PART != "tail" and "B:tail" in [o.name.split(".")[0] for o in scene.objects]:
    tail_body()
if PART != "head" and "B:jaw" in [o.name.split(".")[0] for o in scene.objects]:
    jaw_body()
names, joints = BUILDERS[PART]()
check = part_check(names, joints)
files = review(names, f"v4-{PART}-alone", isolate=True)
ctx = {"head": (Vector((40, 0, 225)), 260), "legs": (Vector((-20, 0, 60)), 380), "dress": (Vector((10, 0, 140)), 360)}.get(PART, (Vector((-30, 0, 120)), 560))
files += review(names, f"v4-{PART}-context", target=ctx[0], distance=ctx[1], isolate=False, views=("three_q", "front", "side"))
print(json.dumps({"part": PART, "objects": len(names), "check": check, "renders": files}, indent=1))
