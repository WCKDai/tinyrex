"""Split Polykrom's 3MF export of TinyREX into a print kit: one binary STL per printed part (PLA and
TPU), from PolyCore's exact solids, each checked watertight (every edge shared by exactly two
triangles, consistently).

    python print_kit.py tinyrex.3mf audit.json OUT_DIR"""
import collections
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET

src, audit_path, out = sys.argv[1:4]
roles = json.loads(open(audit_path).read().split(" ", 1)[1]) if open(audit_path).read().startswith("AUDIT") else json.load(open(audit_path))
PRINTED = {"printed shell", "printed part", "TPU", "insert boss"}
os.makedirs(out, exist_ok=True)
import zipfile
ns = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"
report = {}
with zipfile.ZipFile(src) as z:
    with z.open("3D/3dmodel.model") as f:
        for ev, el in ET.iterparse(f, events=("end",)):
            if el.tag != ns + "object":
                continue
            name = el.get("name", "")
            base = name.split(".")[0]
            role = roles.get(base, {}).get("role")
            if role in PRINTED:
                verts = [(float(v.get("x")), float(v.get("y")), float(v.get("z"))) for v in el.iter(ns + "vertex")]
                tris = [(int(t.get("v1")), int(t.get("v2")), int(t.get("v3"))) for t in el.iter(ns + "triangle")]
                # (weld vertices that coincide to 1e-4 mm: tessellated faces meet at duplicated seam vertices)
                key, remap, welded = {}, [], []
                for v in verts:
                    k = tuple(round(c * 1e4) for c in v)
                    if k not in key:
                        key[k] = len(welded)
                        welded.append(v)
                    remap.append(key[k])
                verts = welded
                tris = [(remap[a], remap[b], remap[c]) for a, b, c in tris]
                tris = [t for t in tris if len(set(t)) == 3]
                def bad_edges(tris):
                    edges = collections.Counter()
                    for a, b, c in tris:
                        for u, v in ((a, b), (b, c), (c, a)):
                            edges[(u, v)] += 1
                    return edges, sum(1 for (u, v), n in edges.items() if n != 1 or edges.get((v, u), 0) != 1)
                edges, open_ = bad_edges(tris)
                filled = 0
                if open_:
                    # (first make the triangles' winding consistent: walk across shared edges, turning
                    # any neighbour that runs the same way round; then outward by signed volume)
                    by_edge = collections.defaultdict(list)
                    for i, (a, b, c) in enumerate(tris):
                        for u, v in ((a, b), (b, c), (c, a)):
                            by_edge[frozenset((u, v))].append(i)
                    tris = [list(t) for t in tris]
                    done = [False] * len(tris)
                    for seed in range(len(tris)):
                        if done[seed]:
                            continue
                        done[seed] = True
                        stack = [seed]
                        while stack:
                            i = stack.pop()
                            a, b, c = tris[i]
                            for u, v in ((a, b), (b, c), (c, a)):
                                for j in by_edge[frozenset((u, v))]:
                                    if done[j]:
                                        continue
                                    x, y, zz = tris[j]
                                    if (u, v) in ((x, y), (y, zz), (zz, x)):
                                        tris[j] = [x, zz, y]
                                    done[j] = True
                                    stack.append(j)
                    vol = sum((verts[a][0] * (verts[b][1] * verts[c][2] - verts[b][2] * verts[c][1]) - verts[a][1] * (verts[b][0] * verts[c][2] - verts[b][2] * verts[c][0]) + verts[a][2] * (verts[b][0] * verts[c][1] - verts[b][1] * verts[c][0])) for a, b, c in tris)
                    tris = [tuple(t) if vol > 0 else (t[0], t[2], t[1]) for t in tris]
                    edges, open_ = bad_edges(tris)
                if open_:
                    # (cap the tessellation's small gaps: walk each boundary loop and fan it shut)
                    nxt = {}
                    for (u, v), n in edges.items():
                        if edges.get((v, u), 0) == 0:
                            nxt.setdefault(v, []).append(u)  # the cap runs the other way round
                    seen = set()
                    for start in list(nxt):
                        if start in seen:
                            continue
                        loop, cur = [], start
                        while cur not in seen and nxt.get(cur):
                            seen.add(cur)
                            loop.append(cur)
                            cur = nxt[cur].pop()
                        if cur == start and len(loop) >= 3:
                            tris += [(loop[0], loop[i], loop[i + 1]) for i in range(1, len(loop) - 1)]
                            filled += 1
                    edges, open_ = bad_edges(tris)
                with open(os.path.join(out, f"{base}.stl"), "wb") as g:
                    g.write(base.encode()[:80].ljust(80, b" "))
                    g.write(struct.pack("<I", len(tris)))
                    for a, b, c in tris:
                        p, q, r = verts[a], verts[b], verts[c]
                        g.write(struct.pack("<12fH", 0, 0, 0, *p, *q, *r, 0))
                report[base] = {"role": role, "triangles": len(tris), "watertight": open_ == 0, "bad_edges": open_, "holes_capped": filled}
            el.clear()
json.dump(report, open(os.path.join(out, "kit.json"), "w"), indent=1)
bad = {k: v["bad_edges"] for k, v in report.items() if not v["watertight"]}
print(f"{len(report)} printed parts written; watertight: {len(report) - len(bad)}; not: {bad}")
