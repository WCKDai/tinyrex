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


def orient(o):
    """Outward normals on a generated closed mesh (a loft built back to front comes out inside out)."""
    me = o.data
    vol = 0.0
    for p in me.polygons:
        a = me.vertices[p.vertices[0]].co
        for k in range(1, len(p.vertices) - 1):
            vol += a.dot(me.vertices[p.vertices[k]].co.cross(me.vertices[p.vertices[k + 1]].co))
    if vol < 0 and o.matrix_world.determinant() > 0 or vol > 0 and o.matrix_world.determinant() < 0:
        import bmesh
        bm = bmesh.new()
        bm.from_mesh(me)
        bmesh.ops.reverse_faces(bm, faces=bm.faces)
        bm.to_mesh(me)
        bm.free()


def split(o, body, axis, at, names):
    """Split a finished shell into two printed parts at a plane: `names` = (the part on the high
    side of `at` along `axis`, the part on the low side). The low part shares the mesh and every
    opening; each keeps its own half."""
    lo = [-600.0, -600.0, -600.0]
    hi = [600.0, 600.0, 600.0]
    hi["xyz".index(axis)] = at
    other = o.copy()
    for c in o.users_collection:
        c.objects.link(other)
    o.name, other.name = names
    cut = bpy.data.objects.new(f"Cut_Split{names[0]}", box_mesh(f"Cut_Split{names[0]}", tuple(lo), tuple(hi)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(cut)
    boolean_cut(o, cut, body, guard=False)
    boolean_cut(other, cut, body, "INTERSECT", guard=False)
    return other


def made(objs, body=None):
    for o in objs:
        if o.type == "MESH" and o.data.users == 1:
            orient(o)
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
    pc_tag(me, pc_loft([v[4 * i:4 * i + 4] for i in range(n)], "x"))
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
    o = next(x for x in coll.objects if x.name.split(".")[0] in ("Body", "BodyL"))
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
        if o.name.split(".")[0] in ("Cut_Belly", "Cut_TailSlot", "Cut_TailHorn", "Cut_BatteryOpening", "Cut_SplitBodyL", "Cut_Shoulder1", "Cut_Shoulder-1", "Cut_LegWindow1", "Cut_LegWindow-1"):
            bpy.data.objects.remove(o, do_unlink=True)
    clear(("Body", "BatteryPod", "Backpack", "PackScrew", "BodyMount", "BodyScrew", "Flank", "Spike_B", "DoorScrew", "BatteryDoor", "ChargePort", "PowerSwitch"))
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
        # (a cover plate under the floor, over the opening the battery swaps through, three screws)
        rounded("BatteryDoor", coll, mat("Seam", "#BDB6A8", 0.6), (-48.5, -22.5, 52.2), (-22.5, 22.5, 54.0), 1.4),
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
    for k, (x, y) in enumerate(((-23.8, 17.0), (-23.8, -17.0))):  # (its rear edge hooks under the floor's lip)
        objs.append(screw(f"DoorScrew{k}", (x, y, 52.6), "-Z"))
    names = made(objs, TRUNK)
    body = objs[0]
    door = bpy.data.objects.new("Cut_BatteryOpening", box_mesh("Cut_BatteryOpening", (-47.5, -20, 40), (-24.5, 20, 57.0)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(door)
    boolean_cut(body, door, TRUNK)
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
        wn = bpy.data.objects.new(f"Cut_LegWindow{sy}", box_mesh(f"Cut_LegWindow{sy}", (-47, min(sy * 23.5, sy * 38), 46), (-1, max(sy * 23.5, sy * 38), 84)))
        bpy.data.collections[f"{SCENE}: rig"].objects.link(wn)
        boolean_cut(body, wn, TRUNK)
    boolean_cut(body, cutter_cylinder("Cut_TailHorn", 10.6, 0, 14, Vector((-74.5, 0, 127.5)), "Z"), TRUNK)
    bpy.context.view_layer.update()
    print("battery corners outside the body:", battery_enclosed())
    # Printed as two halves, left and right, as Microduck's shells are: each screws to the trunk
    # plate on its own bosses; the backpack and the battery door bridge them.
    names.append(split(body, TRUNK, "y", 0.0, ("BodyL", "BodyR")).name.split(".")[0])
    names[names.index("Body")] = "BodyL"
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


def boolean_cut(o, cutter, body=None, op="DIFFERENCE", guard=True):
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
    floor = (0.5 if op == "DIFFERENCE" else 0.05) if guard else 0.0  # (an intersect, or a split, keeps a part by design)
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
CAM_TO = Vector((85.0, 0.0, 261.0))    # where that point goes, in the visor
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
    # (its recipe: a half ellipsoid, hollowed - the wall included)
    half = {"intersect": [{"sphere": {"center": [0.0, 0.0, 0.0], "radius": r}}, {"box": {"min": [0.0, -r - 1, -r - 1], "max": [r + 1, r + 1, r + 1]}}]}
    shell = {"cut": [half, {"sphere": {"center": [0.0, 0.0, 0.0], "radius": r - 1.2}}]}
    pc_tag(me, {"transform": {"body": shell, "matrix": [depth, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}})
    me["pc_recipe_covers"] = "Wall"
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
    clear(("Head", "HeadScrew", "MuzzleScrew", "Spike_H", "Eye", "Brow", "Visor", "Cam", "ToF", "HeadIMU", "Grille", "LED", "StatusLED", "Camera", "Cheek", "Jaw", "Mouth", "Tongue", "Fang", "Blush", "Crest", "Chin", "Nostril", "Muzzle"))
    for o in list(scene.objects):
        if o.name.split(".")[0] in ("Cut_Camera", "Cut_Neck", "Cut_EyeL", "Cut_EyeR", "Cut_MuzzleSlab", "Cut_MuzzleCurve", "Cut_ToF", "Cut_VisorBay", "Cut_VisorFloor", "Cut_VisorSlot", "Cut_Hinge", "Cut_BackSlab", "Cut_NeckBack", "Cut_SplitHeadTop"):
            bpy.data.objects.remove(o, do_unlink=True)
    jaw_body()
    move_camera()
    # The shell: underside at z 204 behind (enclosing the head-roll servo and the jaw servo's case),
    # rising to z 221 in front (above the mouth, below the compute board's lower edge at 223).
    # A wedge: a tall crown behind, the sides one flat facet each from the cheek to the snout (the
    # eye sockets sit on it), a flat snout front carrying the camera window, the ToF window and the
    # status LED - the sensors look out where the boards put them.
    # (x, |y| bottom, |y| shoulder, shoulder z, |y| top, z bottom, z top): wide at the shoulder,
    # where the jaw servo and the head frame are, domed above it, rounded off behind - a skull.
    HEAD_SECTIONS = [(-36, 16, 20, 222, 10, 210, 236), (-28, 32, 36, 228, 18, 203, 244), (-16, 40, 44, 234, 24, 199, 249),
                     (6, 44, 48, 236, 30, 199, 252), (40, 45, 48.5, 236, 31, 199, 256), (54, 44, 47, 236, 34, 209, 259.5),
                     (60, 42, 44, 236, 38, 218.6, 261), (98, 25, 26, 236, 20, 223, 250), (105, 19, 20, 238, 15, 226, 249)]
    shell = loft_hex("Head", HEAD_SECTIONS, LIME)
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
    VISOR = [(40, 14, 6, 252, 258), (56, 20, 14.5, 254, 274.5), (80, 20, 14.5, 252, 275), (93, 18.5, 11.5, 247, 269)]
    visor = loft_trap("Visor", VISOR, LIME, facet=4.5)
    hollow(visor, 2.0)
    vfloor = bpy.data.objects.new("Cut_VisorFloor", box_mesh("Cut_VisorFloor", (38, -25, 240), (97, 25, 257.0)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(vfloor)
    boolean_cut(visor, vfloor, HEAD)
    vslot = bpy.data.objects.new("Cut_VisorSlot", box_mesh("Cut_VisorSlot", (86, -11, 257.5), (97, 11, 266.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(vslot)
    boolean_cut(visor, vslot, HEAD)
    parts.append(visor)
    for sy, side in ((1, "L"), (-1, "R")):  # the visor screws down into the head on each side
        for k, x in enumerate((62, 82)):
            parts.append(screw(f"HeadScrew_V{side}{k}", (x, sy * (side_y(VISOR, x, 263) - 0.5), 263), "Y" if sy > 0 else "-Y"))
    parts.append(rounded("VisorGlass", coll, mat("Smoke", "#101114", 0.06), (89.6, -11.6, 256.9), (91.0, 11.6, 267.1), 0.5))
    # (the camera rides on two black rails from the visor's side walls)
    for sy in (1, -1):
        parts.append(rounded(f"CamMount{sy}", coll, INK, (60, min(sy * 18.3, sy * 12.2), 255.5), (72, max(sy * 18.3, sy * 12.2), 258.5), 0.5))
    # The snout front: the red status LED, and below it the ToF sensor's small window - looking
    # straight ahead at obstacle height; its board is screwed inside the snout.
    parts.append(disc("StatusLED", coll, LED_RED, 1.8, 1.0, (104.6, 0, 242.0), "X"))
    tcut = bpy.data.objects.new("Cut_ToF", box_mesh("Cut_ToF", (99, -3.4, 232.0), (110, 3.4, 236.0)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(tcut)
    boolean_cut(shell, tcut, HEAD)
    parts.append(rounded("ToFWindow", coll, mat("ToFGlass", "#141418", 0.08), (104.0, -4.2, 231.3), (105.4, 4.2, 236.7), 0.6))
    parts.append(rounded("ToFBoard", coll, mat("ToFPCB", "#5B2C8F", 0.5), (101.4, -6, 230.8), (103.0, 6, 239.5), 0.4))
    parts.append(rounded("ToFSensor", coll, CHIP, (103.0, -2.6, 232.6), (103.9, 2.6, 235.4), 0.3))
    # Eye sockets on the side facets: a black angular socket, a glossy black lens, a lime brow.
    A, B, C, D = (Vector(v) for v in ((60, 44, 236), (60, 38, 261), (98, 20, 250), (98, 26, 236)))
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
        eye = Vector((82, sy * hex_side(HEAD_SECTIONS, 82, 246), 246))
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
    muzzle = loft_hex("Muzzle", HEAD_SECTIONS, CREAM)
    hollow(muzzle)
    mslab = bpy.data.objects.new("Cut_MuzzleSlab", box_mesh("Cut_MuzzleSlab", (56, -70, 218.1), (120, 70, 230.5)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(mslab)
    boolean_cut(muzzle, mslab, HEAD, "INTERSECT")
    boolean_cut(shell, mslab, HEAD)
    parts.append(muzzle)
    # The back of the head in cream, as on the boards: a cap over the shell behind the cheeks,
    # its front edge a panel seam.
    back = loft_hex("HeadBack", HEAD_SECTIONS, CREAM)
    hollow(back)
    bslab = bpy.data.objects.new("Cut_BackSlab", box_mesh("Cut_BackSlab", (-60, -70, 190), (4, 70, 300)))
    bpy.data.collections[f"{SCENE}: rig"].objects.link(bslab)
    boolean_cut(back, bslab, HEAD, "INTERSECT")
    boolean_cut(back, cutter_cylinder("Cut_NeckBack", 28.0, 0, 26.0, Vector((NECK_AXIS.x, 0, 186)), "Z"), HEAD)
    boolean_cut(shell, bslab, HEAD)
    parts.append(back)
    for sy, side in ((1, "L"), (-1, "R")):  # the back cap's screws, just behind its seam
        for k, z in enumerate((212, 236)):
            parts.append(screw(f"HeadScrew_B{side}{k}", (-2, sy * (hex_side(HEAD_SECTIONS, -2, z) - 0.4), z), "Y" if sy > 0 else "-Y"))
    # Head IMU (LSM6DSV16X breakout) on the head frame. (The speaker vents through the mouth.)
    parts.append(rounded("HeadIMU", coll, mat("IMUPCB", "#1D3F8F", 0.5), (10, -26, 237.4), (28, -13, 239.0), 0.4))
    # Dorsal spikes: little lime fins down the crown, the dino's tell (they continue down the
    # back and the tail).
    for k, (x, h) in enumerate(((-22, 7.0), (-6, 8.5), (10, 9.5), (26, 9.0))):
        top = hex_at(HEAD_SECTIONS, x)[6]
        parts.append(spike(f"Spike_H{k}", x, top, h))
    # Fixings: the head base screws to the head frame through standoffs; the top screws down onto
    # tabs on the base at the crease; the muzzle panel screws to the base.
    for sy, side in ((1, "L"), (-1, "R")):
        ax = "Y" if sy > 0 else "-Y"
        parts.append(screw(f"HeadScrew_F{side}", (8, sy * (hex_side(HEAD_SECTIONS, 8, 228) - 0.4), 228), ax))
        for k, x in enumerate((32, 52)):  # (behind the eye socket)
            parts.append(screw(f"HeadScrew_T{side}{k}", (x, sy * (hex_side(HEAD_SECTIONS, x, 242.0) - 0.4), 242.0), ax))
        parts.append(screw(f"MuzzleScrew_{side}", (82, sy * (hex_side(HEAD_SECTIONS, 82, 226.5) - 0.4), 226.5), ax))
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
        # (shaped like a jaw bone: a round boss on the hinge, tapering forward to the jaw)
        jaw.append(slab(f"JawArm_{side}", CREAM, teardrop((HINGE.x, HINGE.z), 10.5, (63, 211.0), 6.5), *sorted((sy * 49.5, sy * 52.5)), chamfer=1.4))
        jaw.append(rounded(f"JawCheek_{side}", coll, CREAM, (61, min(sy * 39.5, sy * 52.0), 205.5), (70, max(sy * 39.5, sy * 52.0), 217.2), 2.5))
        jaw.append(disc(f"JawShaft_{side}", coll, SILVER, 3.2, 13.0 if sy > 0 else 13.5, (HINGE.x, sy * (39.9 if sy > 0 else 39.4), HINGE.z), ax, n=24))
        # (the hinge reads as a servo horn: a black horn disc, four screws, a silver hub)
        jaw.append(disc(f"CheekCap_{side}", coll, INK, 8.0, 2.4, (HINGE.x, sy * 52.4, HINGE.z), ax, n=40, bevel_w=0.6))
        jaw.append(disc(f"CheekPin_{side}", coll, SILVER, 3.0, 0.9, (HINGE.x, sy * 54.7, HINGE.z), ax))
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            jaw.append(disc(f"CheekScrew_{side}{k}", coll, SILVER, 1.0, 0.6, (HINGE.x + 5.4 * math.cos(a), sy * 54.7, HINGE.z + 5.4 * math.sin(a)), ax, n=12))
        for k, (x, z) in enumerate(((44, 216.5), (62, 210.5))):
            jaw.append(screw(f"JawScrew_{side}{k}", (x, sy * 52.4, z), ax))
    bpy.context.view_layer.update()
    names += made(jaw, "jaw")
    fangs = []
    for sy, side in ((1, "L"), (-1, "R")):  # just in front of the jaw's lip
        # (small rounded cones: a cute overbite, not buck teeth)
        bpy.ops.mesh.primitive_cone_add(vertices=24, radius1=3.2, radius2=0.7, depth=7.5, location=(102.6, sy * 9.0, 224.9), rotation=(math.pi, 0, 0))
        fang = bpy.context.active_object
        pc_tag(fang.data, pc_cone(3.2, 0.7, 7.5, 24))
        fang.name = f"Fang_{side}"
        for c in fang.users_collection:
            c.objects.unlink(fang)
        coll.objects.link(fang)
        fang.data.materials.append(FANG)
        fang.data.shade_smooth()
        fang.modifiers.new("Soft", "BEVEL").width = 0.4
        fangs.append(fang)
    names += made(fangs, HEAD)
    # The lime shell prints as two parts split at its shoulder crease (z 236), as Microduck's head
    # is top and bottom: the base goes on over the head frame, the top drops on and screws down.
    names.append(split(shell, HEAD, "z", 236.0, ("HeadTop", "HeadBase")).name.split(".")[0])
    names[names.index("Head")] = "HeadTop"
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
    pc_tag(me, pc_loft([v[r * n:(r + 1) * n] for r in range(4)], "y"))
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


GUARD_Y = (70.05, 29.95)  # the ankle cradle's outer and inner faces (|y|), measured


LEG_JOINTS = {"L": ["yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left"], "R": ["bearing_roll", "hip_l_2", "upper_leg_right", "leg_2", "ankle_right"]}


def part_legs():
    """Chamfered lime armour plates on the thighs (the ankle bracket sweeps the shins, so they stay bare), cream
    feet on black soles with three lime toe caps; the servos stay in view at every joint.
    Measured (left; the right mirrors): thigh hardware x -45..16, y 39..71, z 55..103, the knee
    axis at (-31.8, z 67.5); the shin servo x -42..-22, y 37..65, z 16..50; the ankle bracket
    x -46..-10, y 30..70, z 9..34; Microduck's sole x -52..2, y 29..70."""
    clear(("Thigh", "Shin", "Foot", "Toe", "Sole", "Knee", "GuardScrew"))  # (FootGuard* too)
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
        # Ankle guards: rounded cream plates over both faces of the ankle cradle - with the toes
        # and sole, the foot reads as a shoe.
        guard = teardrop((-30, 21), 14.0, (-15, 15), 8.5)
        foot.append(slab(f"FootGuardO_{side}", CREAM, guard, *sorted((sy * GUARD_Y[0], sy * (GUARD_Y[0] + 2.8))), chamfer=1.2))
        foot.append(slab(f"FootGuardI_{side}", CREAM, guard, *sorted((sy * GUARD_Y[1], sy * (GUARD_Y[1] - 2.4))), chamfer=1.2))
        # (the guards screw to the cradle's sides; the foot plate to its underside, from below,
        # before the sole is bonded on)
        for gname, gy, ax in (("O", GUARD_Y[0] + 2.4, "Y"), ("I", GUARD_Y[1] - 2.0, "-Y")):
            foot.append(screw(f"GuardScrew_{side}{gname}", (-27, sy * gy, 21), ax if sy > 0 else ("-Y" if ax == "Y" else "Y")))
        for k, dx in enumerate((-40, -18)):
            foot.append(screw(f"FootScrew_{side}{k}", (dx, yf, 3.4), "-Z"))
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
    clear(("NeckCollar", "Throat", "CollarScrew", "ThroatScrew", "Nape"))
    # (a cream frame clamped round the upper neck servo's case, x 16..36, |y| 13)
    z0, z1 = 170.5, 175.5
    # (and a cream throat plate down the front of the lower neck servo, the boards' neck armour)
    # (and a cream nape plate down its back, on the servo's case-screw bosses)
    names = made([rounded("Throat", coll, CREAM, (36.5, -12, 147), (40.0, 12, 170.4), 1.6),
                  rounded("Nape", coll, CREAM, (11.4, -11, 158.5), (15.5, 11, 170.4), 1.6),  # (on the case screw bosses)
                  rounded("NeckCollar0", coll, CREAM, (36.1, -17.5, z0), (40.5, 17.5, z1), 1.9),
                  rounded("NeckCollar1", coll, CREAM, (11.5, -17.5, z0), (15.9, 17.5, z1), 1.9),
                  rounded("NeckCollar2", coll, CREAM, (15.9, 13.1, z0), (36.1, 17.5, z1), 1.9),
                  rounded("NeckCollar3", coll, CREAM, (15.9, -17.5, z0), (36.1, -13.1, z1), 1.9)]
                 + [screw(f"ThroatScrew{k}", (40.0, dy, z), "X") for k, (dy, z) in enumerate(((-8, 160.4), (8, 160.4)))]  # (into the servo's own case screw holes)
                 + [screw(f"CollarScrew{k}", (40.5, dy, 173), "X") for k, dy in enumerate((-12, 12))]
                 + [screw(f"NapeScrew{k}", (11.4, dy, 160.4), "-X") for k, dy in enumerate((-8, 8))], "neck")
    return names, ["neck", "neck_pitch", "yaw_roll_motion"] + LEG_JOINTS["L"][:3] + LEG_JOINTS["R"][:3]


def screw(name, at, axis):
    """A visible M2 button-head screw (black), its head on the surface at `at`."""
    screw_dot(name, coll, at, axis)
    return next(o for o in coll.objects if o.name.split(".")[0] == name)


def loft_hex(name, sections, material, facet=CHAMFER if "CHAMFER" in globals() else 6.0):
    """A panelled shell through six-point sections (x, half width at the bottom, at the shoulder,
    shoulder height, half width at the top, z bottom, z top): wide only at the shoulder, domed
    above it."""
    v, f = [], []
    for x, wb, wm, zm, wt, z0, z1 in sections:
        v += [(x, -wb, z0), (x, wb, z0), (x, wm, zm), (x, wt, z1), (x, -wt, z1), (x, -wm, zm)]
    n = len(sections)
    f.append(tuple(range(6))[::-1])
    for i in range(n - 1):
        a, b = 6 * i, 6 * (i + 1)
        for k in range(6):
            f.append((a + k, a + (k + 1) % 6, b + (k + 1) % 6, b + k))
    f.append(tuple(range(6 * (n - 1), 6 * n)))
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    pc_tag(me, pc_loft([v[6 * i:6 * i + 6] for i in range(n)], "x"))
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    o = obj_from(name, me, coll, material)
    soft_edges(o, facet)
    return o


def hex_at(sections, x):
    for a, b in zip(sections, sections[1:]):
        if a[0] <= x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            return tuple(p + (q - p) * t for p, q in zip(a, b))
    raise ValueError(x)


def hex_side(sections, x, z):
    """|y| of a loft_hex's side at (x, z)."""
    _, wb, wm, zm, wt, z0, z1 = hex_at(sections, x)
    return wb + (wm - wb) * (z - z0) / (zm - z0) if z <= zm else wm + (wt - wm) * (z - zm) / (z1 - zm)


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
    pc_tag(me, pc_loft([vs[:k], vs[k:]], "y"))
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
    clear(("XL330_Arm", "ArmUpper", "ArmFore", "ArmElbow", "ArmPin", "Hand", "Claw", "ArmScrew", "Shoulder"))  # (HandScrew too)
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
               disc(f"ArmPin_{side}", coll, SILVER, 2.2, 11.8, (38, sy * 45.6, 110), "Y" if sy > 0 else "-Y", n=16),  # (through both arm parts)
               slab(f"ArmFore_{side}", CREAM, [(33, 116), (60, 118.5), (64, 111), (60, 103.5), (35, 102)], *ys(46.4, 53.8)),
               slab(f"Hand_{side}", LIME, [(58, 121), (69, 119.5), (72.5, 111), (68.5, 103), (58, 102)], *ys(45.9, 54.3))]
        for k, a in enumerate((math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)):
            arm.append(screw(f"ArmScrew_{side}{k}", (27 + 5.6 * math.cos(a), sy * 54.6, 140 + 5.6 * math.sin(a)), "Y" if sy > 0 else "-Y"))
        for k, dy in enumerate((2.6, 7.4)):
            bpy.ops.mesh.primitive_cone_add(vertices=6, radius1=2.1, radius2=0.5, depth=8.0, location=(74.5, sy * (45.9 + dy), 108.5), rotation=(0, math.radians(115), 0))
            claw = bpy.context.active_object
            pc_tag(claw.data, pc_cone(2.1, 0.5, 8.0, 6))
            claw.name = f"Claw_{side}{k}"
            for cc in claw.users_collection:
                cc.objects.unlink(claw)
            coll.objects.link(claw)
            claw.data.materials.append(CREAM)
            arm.append(claw)
        arm.append(screw(f"HandScrew_{side}", (59.6, sy * 54.0, 111), "Y" if sy > 0 else "-Y"))
        names += made(arm, f"arm_{side}")
    joints = ["arm_L", "arm_R", "neck", "neck_pitch", "yaw_roll_motion", HEAD, "jaw"] + LEG_JOINTS["L"][:3] + LEG_JOINTS["R"][:3]
    return names, joints


def tube(name, material, pts, r, n=10):
    """A round cable through the points (mm), radius r."""
    pts = [Vector(p) for p in pts]
    v, f = [], []
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        a = t.cross(Vector((0, 0, 1)) if abs(t.z) < 0.9 else Vector((1, 0, 0))).normalized()
        b = t.cross(a)
        v += [tuple(p + (a * math.cos(2 * math.pi * k / n) + b * math.sin(2 * math.pi * k / n)) * r) for k in range(n)]
    for i in range(len(pts) - 1):
        f += [(i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k) for k in range(n)]
    f += [tuple(range(n))[::-1], tuple(range((len(pts) - 1) * n, len(pts) * n))]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    return obj_from(name, me, coll, material, smooth=True)


def ribbon(name, material, pts, width, normals, thick=0.3):
    """A flat flex cable through the points, at each point lying against a surface facing that
    point's normal."""
    pts = [Vector(p) for p in pts]
    v, f = [], []
    for i, p in enumerate(pts):
        nrm = Vector(normals[i]).normalized()
        t = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        w = t.cross(nrm).normalized() * width / 2
        h = nrm * thick / 2
        v += [tuple(p - w - h), tuple(p + w - h), tuple(p + w + h), tuple(p - w + h)]
    for i in range(len(pts) - 1):
        f += [(i * 4 + k, i * 4 + (k + 1) % 4, (i + 1) * 4 + (k + 1) % 4, (i + 1) * 4 + k) for k in range(4)]
    f += [(3, 2, 1, 0), tuple(range((len(pts) - 1) * 4, len(pts) * 4))]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    return obj_from(name, me, coll, material, smooth=True)


def part_cables():
    """The cables that leave a part's own body or run past moving parts:
    - the camera's 22-way CSI ribbon (11.5 mm), from under the camera in the visor, over the
      compute board's top edge, down its face to the CSI connector at its short edge;
    - each arm servo's Dynamixel lead (2.5 mm), from its bottom socket back between the arm and
      hip-yaw servos, up and over the battery holders into Microduck's bus board (the banana PCB)
      at the back of the trunk.
    They are hardware (fixed to their body, clipped against every shell and moving part)."""
    clear(("Cable", "Compute.csi"))
    names = []
    csi = rounded("Compute.csi", coll, mat("Connector", "#E8E2D0", 0.5), (68.9, 26.8, 230.5), (71.4, 32.3, 246.4), 0.3)
    # (flat under the camera, inside the visor's opening; over the board's top edge; down and
    # across its face at mid-height, clear of the visor walls; into the connector)
    flex = ribbon("CableCamera", mat("Flex", "#C8862A", 0.5),
                  [(64, 2, 253.8), (69.0, 2, 253.8), (72.0, 2, 252.6), (72.7, 2, 248), (72.7, 4, 241), (72.7, 16, 238.5), (72.5, 25, 238.5), (71.7, 29.5, 238.5)], 11.5,
                  [(0, 0, 1), (0, 0, 1), (1, 0, 1), (1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 0)])
    for o in (csi, flex):
        attach(o, HEAD)
        names.append(o.name.split(".")[0] if o is flex else "Compute.csi")
    lead = mat("Cable", "#121214", 0.6)
    for side, sy in (("L", 1), ("R", -1)):
        o = tube(f"CableArm{side}", lead, [(21.5, sy * 28.8, 114.1), (14.5, sy * 28.8, 114.1), (14.5, sy * 28.8, 147.6), (-10, sy * 28.0, 148.0), (-29.0, sy * 20, 148.0), (-31.8, sy * 20, 146.5)], 1.25)
        attach(o, TRUNK)
        names.append(o.name.split(".")[0])
    return names, [b for b, (_, _, j, _) in BODIES.items() if j]


DEBUG_SCREWS = globals().get("DEBUG_SCREWS", ())
ENGAGE = 3.0      # mm of material an M2 screw needs behind the panel it holds (insert or thread)
BOSS_R = 2.6      # boss radius: an M2 heat-set insert (3.2 mm) with a 1 mm wall
BOSS_LEN = 5.0


def ray_intervals(trees, origin, d, reach=20.0):
    """(start, end, name) of every part the ray passes through, from origin along d."""
    out = []
    for name, t in trees.items():
        hits, o = [], origin.copy()
        travelled = 0.0
        while True:
            loc, _, _, dist = t.ray_cast(o, d, reach - travelled)
            if loc is None:
                break
            travelled += dist
            hits.append(travelled)
            o = loc + d * 0.01
            travelled += 0.01
        if len(hits) % 2:
            hits.append(reach)  # (still inside at the end of the reach)
        for a, b in zip(hits[0::2], hits[1::2]):
            out.append((a, b, name))
    return sorted(out)


def part_bosses():
    """Every screw needs something to bite: fire a ray inward along it, measure the material
    behind the panel it passes through (servo horns count - they are threaded); where it's under
    ENGAGE, print a boss behind the panel for an M2 heat-set insert, as deep as the room allows."""
    from mathutils.bvhtree import BVHTree
    clear(("Boss_",))
    rest_all()
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    meshes = {}
    for o in scene.objects:
        if o.type == "MESH" and "body" in o and not o.name.split(".")[0].startswith("Boss_"):
            ev = o.evaluated_get(deps)
            me = ev.to_mesh()
            mw = o.matrix_world
            meshes[o.name] = (o, BVHTree.FromPolygons([mw @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons]))
            ev.to_mesh_clear()
    report, names = {}, []
    for o in [m[0] for m in meshes.values() if m[0].get("skin") and "Screw" in m[0].name]:
        body = o["body"]
        near = {body, BODIES.get(body, (None,))[0]} | {b for b, v in BODIES.items() if v[0] == body}
        trees = {n: t for n, (oo, t) in meshes.items() if oo.get("body") in near and n != o.name and "Screw" not in n}
        axis = (o.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
        head = o.matrix_world.translation
        start = head + axis * 3.0
        iv = [(a - 3.0, b - 3.0, n) for a, b, n in ray_intervals(trees, start, -axis)]
        iv = [t for t in iv if t[1] > -0.5]
        if o.name.split(".")[0] in DEBUG_SCREWS:
            print("DBG", o.name, [round(v, 2) for v in axis], [(round(a, 1), round(b, 1), n.split(".")[0]) for a, b, n in iv][:8])
        if not iv:
            report[o.name.split(".")[0]] = "nothing behind it"
            continue
        # The panel the head sits on is the first part along the ray; the screw clamps it and
        # bites whatever lies behind (overlapping parts - a plate set into a shell - are one stack).
        plate_end, plate = iv[0][1], iv[0][2]
        # (a horn is threaded; a solid printed part over 6 mm thick takes the screw itself)
        threaded = any(".horn" in n for _, _, n in iv[1:3]) or iv[0][1] - max(iv[0][0], 0) >= 6.0
        engage = sum(min(b, plate_end + 8) - max(a, plate_end) for a, b, n in iv[1:] if a < plate_end + 8 and b > plate_end)
        if threaded or engage >= ENGAGE:
            continue
        stack_end = plate_end
        for a, b, _ in iv[1:]:
            if a <= stack_end + 0.05:
                stack_end = max(stack_end, b)
        nxt = next(((a, n) for a, _, n in iv[1:] if a > stack_end + 0.05), (stack_end + 20, None))
        free = nxt[0] - stack_end - 0.4
        # (a frame part within 15 mm behind: a standoff that reaches it, the screw going through to
        # bite the frame)
        standoff = (nxt[1] is not None and nxt[1].split(".")[0].startswith(("Bracket_", "XL330")) and "." not in nxt[1].split(".")[0]
                    and nxt[0] - stack_end <= 15.0)
        # (the boss is 2 x BOSS_R across: check the room on a ring of rays at its radius too)
        side = axis.orthogonal().normalized()
        for k in range(8):
            off = Quaternion(axis, 2 * math.pi * k / 8) @ side * (BOSS_R + 0.3)
            ring = [(a - 3.0, b - 3.0, n) for a, b, n in ray_intervals(trees, start + off, -axis)]
            ring_end = stack_end
            for a, b, n in ring:
                if a <= ring_end + 0.05 and b > -0.5:
                    ring_end = max(ring_end, b)
            first = next((a for a, _, _ in ring if a > ring_end + 0.05), stack_end + 20)
            free = min(free, first - stack_end - 0.4)
        plate_end = stack_end
        length = free + 0.3 if standoff else min(BOSS_LEN, free)
        tag = o.name.split(".")[0]
        if length < 2.0:
            report[tag] = f"no room for a boss behind {plate.split('.')[0]} ({free:.1f} mm)"
            continue
        at = head - axis * (plate_end - 0.3)
        b = part(f"Boss_{tag}", coll, meshes[plate][0].active_material or CREAM, cyl_mesh(f"Boss_{tag}", BOSS_R, 0, length + 0.3, 20),
                 Matrix.Translation(at) @ (axis * -1).to_track_quat("Z", "Y").to_matrix().to_4x4(), Matrix.Identity(4), smooth=True)
        names += made([b], body)
        report[tag] = f"{'standoff to ' + nxt[1].split('.')[0] + ',' if standoff else 'boss'} {length + 0.3:.1f} mm behind {plate.split('.')[0]}"
    print("BOSSES", json.dumps(report))
    return names, []


XL330_CASE = ((-13.0, -10.0, -24.5), (13.0, 10.0, 9.5))  # an XL330's case in its own frame (output along +X)
FRAME_GAP = 0.3   # clearance round each part a bracket holds
FRAME_ROUND = 1.5  # the brackets' edge fillet


def local_box(o):
    """An object's mesh bounds in its own coordinates."""
    vs = [v.co for v in o.data.vertices]
    return Vector([min(v[i] for v in vs) for i in range(3)]), Vector([max(v[i] for v in vs) for i in range(3)])


def world_box(o):
    pts = [o.matrix_world @ Vector(c) for c in o.bound_box]
    return Vector([min(p[i] for p in pts) for i in range(3)]), Vector([max(p[i] for p in pts) for i in range(3)])


def boxes_meet(a, b, pad=0.0):
    return all(a[0][i] - pad < b[1][i] and b[0][i] - pad < a[1][i] for i in range(3))


def frame_cutters(o):
    """The pockets a piece of hardware needs in a bracket: (mesh, world matrix) pairs - a servo's
    case with its horn and hub, a bearing, the battery, a cable's run."""
    base = o.name.split(".")[0]
    M = o.matrix_world
    g = FRAME_GAP
    T = Matrix.Translation
    if base.startswith("XL330") and (len(o.name.split(".")) == 1 or o.name.split(".")[1].isdigit()):
        (x0, y0, z0), (x1, y1, z1) = XL330_CASE
        return [(box_mesh("Cut_Frame", (x0 - g, y0 - g, z0 - 2.0 - g), (x1 + g, y1 + g, z1 + g)), M),  # (and its sockets)
                (cyl_mesh("Cut_Frame", 9.0 + g, 0, 1.5 + 2 * g, 40), M @ T((13 - g, 0, 0)) @ axis_matrix("X")),
                (cyl_mesh("Cut_Frame", 5.0 + g, 0, 1.5 + 2 * g, 32), M @ T((-13 + g, 0, 0)) @ axis_matrix("-X"))]
    if base.startswith("Bearing"):
        lo, hi = local_box(o)
        return [(cyl_mesh("Cut_Frame", (hi.x - lo.x) / 2 + g, lo.z - g, hi.z + g, 40), M)]
    if base == "Battery":
        lo, hi = local_box(o)
        return [(box_mesh("Cut_Frame", tuple(lo - Vector((g, g, g))), tuple(hi + Vector((g, g, g)))), M)]
    if base.startswith("Cable") and o.data.vertices:
        n = 10  # (tube(): rings of 10)
        V = [M @ v.co for v in o.data.vertices]
        rings = [sum(V[i * n:(i + 1) * n], Vector()) / n for i in range(len(V) // n)]
        out = []
        for a, b in zip(rings, rings[1:]):
            d = b - a
            if d.length < 0.5:
                continue
            rot = d.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
            out.append((cyl_mesh("Cut_Frame", 1.25 + 0.4, -0.6, d.length + 0.6, 16), T(a) @ rot))
        return out
    return []


def part_frame():
    """TinyREX's own frame brackets, carved from the clip-checked envelopes of Microduck's (so they
    can't add a clip): each pocketed round the servos, horns, bearings, battery and cables it holds
    (a cradle that hugs them), plates lightened with holes where nothing mounts, edges filleted,
    printed in the boards' dark frame colour. The two battery holders become left and right
    clamps."""
    for o in list(scene.objects):
        if o.name.split(".")[0].startswith("Cut_Frame"):
            bpy.data.objects.remove(o, do_unlink=True)
    hardware = [o for o in scene.objects if o.type == "MESH" and "body" in o and not o.get("skin")]
    brackets = [o for o in hardware if o.name.startswith("Bracket_")]
    held = [o for o in hardware if not o.name.startswith("Bracket_")]
    frame_mat = bpy.data.materials.get("Bracket") or BRACKET
    names = []
    for b in brackets:
        for m in [m for m in b.modifiers]:
            b.modifiers.remove(m)
        b.data.materials.clear()
        b.data.materials.append(frame_mat)
        lo, hi = local_box(b)
        size = hi - lo
        # (the placeholder's own corner round, so the bracket stays inside its clip-checked envelope)
        shown = any(k in b.name for k in ("Bracket_leg", "Bracket_yaw2roll", "Bracket_hip_l", "Bracket_bearing_roll", "Bracket_ankle"))
        bevel(b, min(4.0 if shown else FRAME_ROUND, min(size) / 2 - 0.05), segments=3)
        body = b["body"]
        near = {body, BODIES.get(body, (None,))[0]} | {k for k, v in BODIES.items() if v[0] == body}
        wb = world_box(b)
        cut_boxes = []
        k = 0
        for h in held:
            if h["body"] not in near or not boxes_meet(wb, world_box(h)):
                continue
            for me, M in frame_cutters(h):
                c = bpy.data.objects.new(f"Cut_Frame_{b.name.split('.')[0]}_{k}", me)
                bpy.data.collections[f"{SCENE}: rig"].objects.link(c)
                c.matrix_world = M
                k += 1
                bpy.context.view_layer.update()
                if not boxes_meet(wb, world_box(c)):
                    bpy.data.objects.remove(c, do_unlink=True)
                    continue
                cut_boxes.append(world_box(c))
                boolean_cut(b, c, body, guard=False)
        base = b.name.split(".")[0]
        if base.startswith("Bracket_power_support"):  # (two clamps, either side of the battery)
            left = base.endswith("_5")
            c = bpy.data.objects.new(f"Cut_Frame_{base}_half", box_mesh("Cut_Frame", (-200, -0.3 if left else 0.3, -200), (200, -200 if left else 200, 400)))
            bpy.data.collections[f"{SCENE}: rig"].objects.link(c)
            c.matrix_world = Matrix.Identity(4)
            boolean_cut(b, c, body, guard=False)
        # Lightening holes through a plate, where nothing is held behind them.
        thin = min(range(3), key=lambda i: size[i])
        u, w = [i for i in range(3) if i != thin]
        if size[thin] <= 6.0 and min(size[u], size[w]) >= 16.0:
            r = min(4.0, 0.16 * min(size[u], size[w]))
            pitch, margin = 2.7 * r, 2.5 + r
            nu = max(1, int((size[u] - 2 * margin) // pitch) + 1)
            nw = max(1, int((size[w] - 2 * margin) // pitch) + 1)
            M = b.matrix_world
            inv = M.inverted()
            local_cuts = []
            for cb in cut_boxes:
                pts = [inv @ Vector((x, y, z)) for x in (cb[0].x, cb[1].x) for y in (cb[0].y, cb[1].y) for z in (cb[0].z, cb[1].z)]
                local_cuts.append((Vector([min(p[i] for p in pts) for i in range(3)]), Vector([max(p[i] for p in pts) for i in range(3)])))
            for i in range(nu):
                for j in range(nw):
                    cu = lo[u] + (size[u] - (nu - 1) * pitch) / 2 + i * pitch
                    cw = lo[w] + (size[w] - (nw - 1) * pitch) / 2 + j * pitch
                    if any(cl[0][u] - 2 < cu + r and cu - r < cl[1][u] + 2 and cl[0][w] - 2 < cw + r and cw - r < cl[1][w] + 2 for cl in local_cuts):
                        continue
                    at = Vector((0.0, 0.0, 0.0))
                    at[u], at[w], at[thin] = cu, cw, lo[thin] - 1.0
                    axis = ["X", "Y", "Z"][thin]
                    c = bpy.data.objects.new(f"Cut_Frame_{base}_hole{i}_{j}", cyl_mesh("Cut_Frame", r, 0, size[thin] + 2.0, 32))
                    bpy.data.collections[f"{SCENE}: rig"].objects.link(c)
                    c.matrix_world = M @ Matrix.Translation(at) @ axis_matrix(axis)
                    boolean_cut(b, c, body, guard=False)
        names.append(base)
    bpy.context.view_layer.update()
    return names, [b for b, (_, _, j, _) in BODIES.items() if j]


def part_all():
    """No new part: check every shell against every joint (the extra tail and jaw included)."""
    names = [o.name.split(".")[0] for o in coll.objects if o.get("skin")]
    return names, [b for b, (_, _, j, _) in BODIES.items() if j]


BUILDERS = {"body": part_body, "tail": part_tail, "head": part_head, "legs": part_legs, "dress": part_dress, "arms": part_arms, "cables": part_cables, "bosses": part_bosses, "frame": part_frame, "all": part_all}
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
