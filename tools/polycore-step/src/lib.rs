//! `polycore-step`: a JSON recipe of CAD parts in, an AP242 assembly STEP file out.
//!
//! Each part is built from its exact `solid` recipe with the PolyCore kernel; when that errors,
//! panics or runs past the per-part time budget, the part falls back to its `mesh` (a mesh body,
//! converted to planar/cylindrical B-rep faces where PolyCore can). Parts run on worker threads;
//! everything that reaches the output is collected in recipe order, so the same input gives the
//! same file.

use std::collections::VecDeque;
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Loop2, Mesh, Plane, Region2, Vec2, Vec3};
use solidcraft_kernel::{self as kernel, Body, BoolOp, ExportBody, ExportProduct, KernelError, StepHeader};

mod push;
pub use push::{Rigid, push_down};

// ---------------------------------------------------------------------------------------------
// Recipe
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, Deserialize)]
pub struct Recipe {
    pub name: String,
    #[serde(default)]
    pub bodies: Vec<BodyDef>,
    #[serde(default)]
    pub parts: Vec<Part>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct BodyDef {
    pub name: String,
    /// Column-major 4x4: the body frame in the root (world) frame.
    #[serde(default = "identity16")]
    pub placement: [f64; 16],
}

#[derive(Clone, Debug, Deserialize)]
pub struct Part {
    pub name: String,
    pub body: String,
    #[serde(default)]
    pub color: Option<[f32; 3]>,
    #[serde(default)]
    pub solid: Option<Node>,
    #[serde(default)]
    pub mesh: Option<MeshDef>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct MeshDef {
    pub positions: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct Section {
    pub origin: [f64; 3],
    #[serde(alias = "x_dir")]
    pub x: [f64; 3],
    #[serde(alias = "y_dir")]
    pub y: [f64; 3],
    pub points: Vec<[f64; 2]>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct PlaneDef {
    pub origin: [f64; 3],
    #[serde(alias = "x_dir")]
    pub x: [f64; 3],
    #[serde(alias = "y_dir")]
    pub y: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
#[serde(untagged)]
pub enum EdgeSel {
    /// `"all"`: every sharp edge.
    Keyword(String),
    /// Points near the edges to blend.
    Points(Vec<[f64; 3]>),
}

impl Default for EdgeSel {
    fn default() -> Self {
        EdgeSel::Keyword("all".into())
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct Blend {
    pub body: Box<Node>,
    /// Fillet radius, or chamfer distance (`"distance"`).
    #[serde(alias = "distance")]
    pub radius: f64,
    #[serde(default)]
    pub edges: EdgeSel,
    /// With `"edges": "all"`: only edges whose faces turn at least this much (degrees).
    #[serde(default = "default_min_angle")]
    pub min_angle_deg: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Node {
    Box { min: [f64; 3], max: [f64; 3] },
    Cylinder { base: [f64; 3], axis: [f64; 3], radius: f64, height: f64 },
    Sphere { center: [f64; 3], radius: f64 },
    Loft {
        sections: Vec<Section>,
        /// One smooth surface through all sections (PolyCore's own loft) instead of ruled faces.
        #[serde(default)]
        smooth: bool,
    },
    Extrude {
        plane: PlaneDef,
        points: Vec<[f64; 2]>,
        #[serde(default)]
        holes: Vec<Vec<[f64; 2]>>,
        start: f64,
        end: f64,
    },
    Union(Vec<Node>),
    Cut(Vec<Node>),
    Intersect(Vec<Node>),
    Fillet(Blend),
    Chamfer(Blend),
    Transform { body: Box<Node>, matrix: [f64; 16] },
    Mesh(MeshDef),
}

fn identity16() -> [f64; 16] {
    [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]
}

fn default_min_angle() -> f64 {
    20.0
}

pub fn parse_recipe(text: &str) -> Result<Recipe, String> {
    serde_json::from_str(text).map_err(|e| format!("recipe: {e}"))
}

// ---------------------------------------------------------------------------------------------
// Building one exact solid
// ---------------------------------------------------------------------------------------------

struct Ctx {
    warnings: Vec<String>,
    deadline: Instant,
    /// When open-ended retries (edge-by-edge blends) stop and keep what they have, leaving time
    /// to finish the part before `deadline`.
    soft_deadline: Instant,
    /// A step fell back to a faceted boolean: the result is no longer exact.
    faceted: bool,
}

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn kerr(path: &str, e: KernelError) -> String {
    format!("{path}: {e}")
}

fn matrix(a: &[f64; 16]) -> [[f64; 4]; 4] {
    let mut m = [[0.0; 4]; 4];
    for (c, col) in m.iter_mut().enumerate() {
        for (r, x) in col.iter_mut().enumerate() {
            *x = a[c * 4 + r];
        }
    }
    m
}

/// A plane from an origin and two in-plane directions, made orthonormal (x kept, y squared to it).
fn plane(path: &str, origin: [f64; 3], x: [f64; 3], y: [f64; 3]) -> Result<Plane, String> {
    let (o, x, y) = (v3(origin), v3(x), v3(y));
    if !(o.is_finite() && x.is_finite() && y.is_finite()) {
        return Err(format!("{path}: non-finite plane"));
    }
    let x = x.normalized().ok_or_else(|| format!("{path}: plane x direction is zero"))?;
    let y = (y - x * y.dot(x)).normalized().ok_or_else(|| format!("{path}: plane y direction is zero or parallel to x"))?;
    Ok(Plane { origin: o, x, y })
}

/// A closed polygon loop: consecutive duplicates and a repeated closing point dropped.
fn polygon(path: &str, pts: &[[f64; 2]]) -> Result<Loop2, String> {
    let mut v: Vec<Vec2> = Vec::with_capacity(pts.len());
    for p in pts {
        let q = Vec2::new(p[0], p[1]);
        if !q.is_finite() {
            return Err(format!("{path}: non-finite point"));
        }
        if v.last().is_none_or(|l| l.dist(q) > 1e-9) {
            v.push(q);
        }
    }
    while v.len() > 1 && v.first().zip(v.last()).is_some_and(|(a, b)| a.dist(*b) <= 1e-9) {
        v.pop();
    }
    if v.len() < 3 {
        return Err(format!("{path}: a loop needs at least 3 distinct points"));
    }
    let lp = Loop2::polygon(&v);
    if lp.signed_area().abs() < 1e-12 {
        return Err(format!("{path}: loop has zero area"));
    }
    Ok(lp)
}

fn check_time(path: &str, ctx: &Ctx) -> Result<(), String> {
    if Instant::now() >= ctx.deadline { Err(format!("{path}: out of time")) } else { Ok(()) }
}

fn boolean(path: &str, a: &Body, b: &Body, op: BoolOp, ctx: &mut Ctx) -> Result<Body, String> {
    let first = match kernel::boolean(a, b, op) {
        Ok(Some(r)) => return Ok(r),
        Ok(None) => return Err(format!("{path}: the result is empty")),
        Err(e) => kerr(path, e),
    };
    // The exact boolean failed. Fallbacks, each giving planar faces where they facet:
    //  1. the same boolean with the tool's tessellation (`faceted_boolean`);
    //  2. the exact boolean with the tool faceted by the kernel itself;
    //  3. both operands faceted by the kernel, then a planar boolean.
    let tol = facet_tol(a.size().max(b.size()));
    let mut why = Vec::new();
    let done = |r: Body, how: &str, ctx: &mut Ctx| {
        ctx.faceted = true;
        ctx.warnings.push(format!("{first}; used {how} (tolerance {tol:.3} mm)"));
        Ok(r)
    };
    check_time(path, ctx)?;
    match b.tessellate(tol).map_err(|e| e.to_string()).and_then(|mesh| {
        kernel::guard("faceted boolean", || kernel::faceted_boolean(a, &mesh, op, tol)).map_err(|e| e.to_string())
    }) {
        Ok(Some(r)) => return done(r, "a faceted boolean", ctx),
        Ok(None) => why.push("faceted boolean: empty".to_string()),
        Err(e) => why.push(format!("faceted boolean: {e}")),
    }
    check_time(path, ctx)?;
    let bf = match tessellated_brep(b) {
        Ok((bf, _, _)) => bf,
        Err(e) => return Err(format!("{first}; {}; faceting the tool: {e}", why.join("; "))),
    };
    match kernel::boolean(a, &bf, op) {
        Ok(Some(r)) => return done(r, "the tool faceted", ctx),
        Ok(None) => why.push("faceted tool: empty".to_string()),
        Err(e) => why.push(format!("faceted tool: {e}")),
    }
    check_time(path, ctx)?;
    match tessellated_brep(a).map_err(|e| e.to_string()).and_then(|(af, _, _)| kernel::planar_boolean(&af, &bf, op).map_err(|e| e.to_string())) {
        Ok(Some(r)) => return done(r, "both operands faceted", ctx),
        Ok(None) => why.push("both faceted: empty".to_string()),
        Err(e) => why.push(format!("both faceted: {e}")),
    }
    Err(format!("{first}; {}", why.join("; ")))
}

/// Chord tolerance (mm) for faceting a body of this size.
fn facet_tol(size: f64) -> f64 {
    (size * 2e-4).clamp(0.005, 0.05)
}

fn fold(path: &str, kind: &str, nodes: &[Node], op: BoolOp, ctx: &mut Ctx) -> Result<Body, String> {
    let Some(first) = nodes.first() else { return Err(format!("{path}: {kind} needs at least one operand")) };
    if op == BoolOp::Intersect && nodes.len() < 2 {
        return Err(format!("{path}: intersect needs two operands"));
    }
    let mut acc = build_node(first, &format!("{path}/{kind}[0]"), ctx)?;
    for (i, n) in nodes.iter().enumerate().skip(1) {
        let p = format!("{path}/{kind}[{i}]");
        check_time(&p, ctx)?;
        // A box tool that crosses the body with only one face is a half-space: split the body by
        // that face's plane instead (direct and robust where the boolean against a big box can
        // fail, e.g. halving a symmetric body on its mirror plane).
        if op != BoolOp::Union
            && let Some((plane, keep_positive)) = oriented_box(n).and_then(|ob| half_space_of(&acc, &ob, op))
        {
            match kernel::half_space(&acc, &plane, keep_positive) {
                Ok((Some(r), _)) => {
                    acc = r;
                    continue;
                }
                Ok((None, _)) => return Err(format!("{p}: the result is empty")),
                Err(e) => debug(|| format!("{p}: half-space split failed ({e}); trying the boolean")),
            }
        }
        let b = build_node(n, &p, ctx)?;
        check_time(&p, ctx)?;
        acc = boolean(&p, &acc, &b, op, ctx)?;
    }
    Ok(acc)
}

/// A box: a corner, three orthonormal axes and the extents along them.
struct OrientedBox {
    corner: Vec3,
    axes: [Vec3; 3],
    ext: [f64; 3],
}

/// The box a node is: a `box`, or a rectangle extruded square to its plane (a box turned by a
/// pushed-down transform).
fn oriented_box(n: &Node) -> Option<OrientedBox> {
    match n {
        Node::Box { min, max } => {
            let (lo, hi) = (v3(*min).min(v3(*max)), v3(*min).max(v3(*max)));
            let d = hi - lo;
            Some(OrientedBox { corner: lo, axes: [Vec3::X, Vec3::Y, Vec3::Z], ext: [d.x, d.y, d.z] })
        }
        Node::Extrude { plane: pd, points, holes, start, end } if holes.is_empty() && points.len() == 4 => {
            let pl = plane("box", pd.origin, pd.x, pd.y).ok()?;
            let us: Vec<f64> = points.iter().map(|p| p[0]).collect();
            let vs: Vec<f64> = points.iter().map(|p| p[1]).collect();
            let (u0, u1) = (us.iter().copied().fold(f64::MAX, f64::min), us.iter().copied().fold(f64::MIN, f64::max));
            let (v0, v1) = (vs.iter().copied().fold(f64::MAX, f64::min), vs.iter().copied().fold(f64::MIN, f64::max));
            // Every corner of the bounding rectangle is a point of the loop.
            let corner = |u: f64, v: f64| points.iter().any(|p| (p[0] - u).abs() < 1e-9 && (p[1] - v).abs() < 1e-9);
            if !(corner(u0, v0) && corner(u1, v0) && corner(u1, v1) && corner(u0, v1)) {
                return None;
            }
            let (a, b) = (start.min(*end), start.max(*end));
            let nrm = pl.normal();
            let c = pl.origin + pl.x * u0 + pl.y * v0 + nrm * a;
            Some(OrientedBox { corner: c, axes: [pl.x, pl.y, nrm], ext: [u1 - u0, v1 - v0, b - a] })
        }
        _ => None,
    }
}

/// When `ob` covers the body except across one of its faces: that face's plane (normal along
/// the box axis) and which side of it (`true`: where the normal points) the result keeps.
fn half_space_of(body: &Body, ob: &OrientedBox, op: BoolOp) -> Option<(Plane, bool)> {
    let bb = kernel::measure_mass(body).ok()?.bbox;
    let margin = (bb.max - bb.min).len() * 1e-6 + 1e-6;
    let corners = (0..8).map(|i| {
        Vec3::new(if i & 1 == 0 { bb.min.x } else { bb.max.x }, if i & 2 == 0 { bb.min.y } else { bb.max.y }, if i & 4 == 0 { bb.min.z } else { bb.max.z })
    });
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for c in corners {
        for k in 0..3 {
            let t = (c - ob.corner).dot(ob.axes[k]);
            lo[k] = lo[k].min(t);
            hi[k] = hi[k].max(t);
        }
    }
    let covers = |k: usize| lo[k] > margin && hi[k] < ob.ext[k] - margin;
    let open: Vec<usize> = (0..3).filter(|k| !covers(*k)).collect();
    let [k] = open.as_slice() else { return None };
    let k = *k;
    // Which face crosses: the low one (box inside on the positive side) or the high one.
    let (offset, inside_positive) = if lo[k] < -margin && hi[k] > margin && hi[k] < ob.ext[k] - margin {
        (0.0, true)
    } else if hi[k] > ob.ext[k] + margin && lo[k] > margin && lo[k] < ob.ext[k] - margin {
        (ob.ext[k], false)
    } else {
        return None;
    };
    let plane = Plane::from_normal(ob.corner + ob.axes[k] * offset, ob.axes[k])?;
    let keep_positive = if op == BoolOp::Intersect { inside_positive } else { !inside_positive };
    Some((plane, keep_positive))
}

fn build_node(n: &Node, path: &str, ctx: &mut Ctx) -> Result<Body, String> {
    check_time(path, ctx)?;
    match n {
        Node::Box { min, max } => kernel::box_solid(v3(*min), v3(*max)).map_err(|e| kerr(&format!("{path}/box"), e)),
        Node::Cylinder { base, axis, radius, height } => {
            kernel::cylinder(v3(*base), v3(*axis), *radius, *height).map_err(|e| kerr(&format!("{path}/cylinder"), e))
        }
        Node::Sphere { center, radius } => kernel::sphere(v3(*center), *radius).map_err(|e| kerr(&format!("{path}/sphere"), e)),
        Node::Loft { sections, smooth } => {
            let p = format!("{path}/loft");
            let mut secs = Vec::with_capacity(sections.len());
            for (i, s) in sections.iter().enumerate() {
                let sp = format!("{p}.sections[{i}]");
                secs.push((plane(&sp, s.origin, s.x, s.y)?, polygon(&sp, &s.points)?));
            }
            if *smooth || secs.len() < 3 {
                return kernel::loft(&secs).map_err(|e| kerr(&p, e));
            }
            ruled_loft(&p, &secs, ctx)
        }
        Node::Extrude { plane: pd, points, holes, start, end } => {
            let p = format!("{path}/extrude");
            let pl = plane(&p, pd.origin, pd.x, pd.y)?;
            let outer = polygon(&format!("{p}.points"), points)?;
            let holes = holes.iter().enumerate().map(|(i, h)| polygon(&format!("{p}.holes[{i}]"), h)).collect::<Result<Vec<_>, _>>()?;
            let (a, b) = if start <= end { (*start, *end) } else { (*end, *start) };
            let mut bodies = kernel::extrude(&pl, &[Region2 { outer, holes }], a, b).map_err(|e| kerr(&p, e))?;
            match bodies.len() {
                0 => Err(format!("{p}: extrude made no body")),
                1 => bodies.pop().ok_or_else(|| format!("{p}: extrude made no body")),
                _ => {
                    let mut it = bodies.into_iter();
                    let mut acc = it.next().ok_or_else(|| format!("{p}: extrude made no body"))?;
                    for b in it {
                        acc = boolean(&p, &acc, &b, BoolOp::Union, ctx)?;
                    }
                    Ok(acc)
                }
            }
        }
        Node::Union(v) => fold(path, "union", v, BoolOp::Union, ctx),
        Node::Cut(v) => fold(path, "cut", v, BoolOp::Cut, ctx),
        Node::Intersect(v) => fold(path, "intersect", v, BoolOp::Intersect, ctx),
        Node::Fillet(b) => blend(b, true, &format!("{path}/fillet"), ctx),
        Node::Chamfer(b) => blend(b, false, &format!("{path}/chamfer"), ctx),
        Node::Transform { body, matrix: m } => {
            let p = format!("{path}/transform");
            if m.iter().any(|x| !x.is_finite()) {
                return Err(format!("{p}: non-finite matrix"));
            }
            let b = build_node(body, &p, ctx)?;
            kernel::transform_matrix(&b, matrix(m)).map_err(|e| kerr(&p, e))
        }
        Node::Mesh(m) => mesh_from(m).map_err(|e| format!("{path}/mesh: {e}")),
    }
}

/// A loft ruled between consecutive sections. PolyCore's `loft` through three or more sections
/// makes one smooth surface per side instead, so each pair of sections is lofted on its own
/// (two sections are always ruled) and the pieces are united across the shared sections.
fn ruled_loft(path: &str, secs: &[(Plane, Loop2)], ctx: &mut Ctx) -> Result<Body, String> {
    let mut acc: Option<Body> = None;
    for (i, w) in secs.windows(2).enumerate() {
        let p = format!("{path}.segment[{i}]");
        check_time(&p, ctx)?;
        let piece = kernel::loft(w).map_err(|e| kerr(&p, e))?;
        acc = Some(match acc {
            None => piece,
            Some(a) => boolean(&p, &a, &piece, BoolOp::Union, ctx)?,
        });
    }
    acc.ok_or_else(|| format!("{path}: a loft needs at least 2 sections"))
}

fn mesh_from(m: &MeshDef) -> Result<Body, String> {
    let n = m.positions.len();
    if let Some(t) = m.triangles.iter().find(|t| t.iter().any(|i| *i as usize >= n)) {
        return Err(format!("triangle {t:?} indexes past {n} positions"));
    }
    let pos: Vec<Vec3> = m.positions.iter().map(|p| v3(*p)).collect();
    kernel::mesh_body(&pos, &m.triangles).map_err(|e| e.to_string())
}

fn polyline_mid(pts: &[Vec3]) -> Option<Vec3> {
    let total: f64 = pts.windows(2).map(|w| w[0].dist(w[1])).sum();
    let mut left = total * 0.5;
    for w in pts.windows(2) {
        let l = w[0].dist(w[1]);
        if l >= left && l > 0.0 {
            return Some(w[0].lerp(w[1], left / l));
        }
        left -= l;
    }
    pts.first().copied()
}

fn tri_normal(m: &Mesh, t: &[u32; 3]) -> Option<Vec3> {
    let p = |i: u32| m.positions.get(i as usize).copied();
    let (a, b, c) = (p(t[0])?, p(t[1])?, p(t[2])?);
    (b - a).cross(c - a).normalized()
}

fn tri_dist(m: &Mesh, t: &[u32; 3], q: Vec3) -> f64 {
    let p = |i: u32| m.positions.get(i as usize).copied().unwrap_or(Vec3::ZERO);
    let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
    q.dist_to_segment(a, b).min(q.dist_to_segment(b, c)).min(q.dist_to_segment(c, a))
}

/// Points on every edge whose two faces meet at `min_angle_deg` or more (the angle between the
/// face normals beside the edge's midpoint, from the tessellation). Seams and edges with one
/// face are skipped.
pub fn sharp_edges(body: &Body, min_angle_deg: f64) -> Result<Vec<Vec3>, String> {
    let tol = (body.size() * 1e-3).clamp(1e-3, 0.1);
    let m = body.tessellate(tol).map_err(|e| e.to_string())?;
    let nf = m.tri_face.iter().map(|f| *f as usize + 1).max().unwrap_or(0);
    let mut by_face: Vec<Vec<usize>> = vec![Vec::new(); nf];
    for (i, f) in m.tri_face.iter().enumerate() {
        if let Some(v) = by_face.get_mut(*f as usize) {
            v.push(i);
        }
    }
    let normal_near = |f: u32, q: Vec3| -> Option<Vec3> {
        let tris = by_face.get(f as usize)?;
        let best = tris
            .iter()
            .filter_map(|i| m.triangles.get(*i))
            .filter(|t| tri_normal(&m, t).is_some())
            .min_by(|a, b| tri_dist(&m, a, q).total_cmp(&tri_dist(&m, b, q)))?;
        tri_normal(&m, best)
    };
    let cos_min = min_angle_deg.clamp(0.0, 180.0).to_radians().cos();
    let mut out = Vec::new();
    for (i, poly) in m.edges.iter().enumerate() {
        if m.seams.get(i).copied().unwrap_or(false) {
            continue;
        }
        let Some(faces) = m.edge_faces.get(i) else { continue };
        let [fa, fb] = faces.as_slice() else { continue };
        if fa == fb {
            continue;
        }
        let Some(mid) = polyline_mid(poly) else { continue };
        let (Some(na), Some(nb)) = (normal_near(*fa, mid), normal_near(*fb, mid)) else { continue };
        if na.dot(nb) <= cos_min + 1e-12 {
            out.push(mid);
        }
    }
    Ok(out)
}

fn blend(spec: &Blend, fillet: bool, path: &str, ctx: &mut Ctx) -> Result<Body, String> {
    let what = if fillet { "fillet" } else { "chamfer" };
    let body = build_node(&spec.body, path, ctx)?;
    if body.is_mesh() {
        ctx.warnings.push(format!("{path}: {what} skipped on a mesh body"));
        return Ok(body);
    }
    let r = spec.radius;
    if !(r.is_finite() && r > 0.0) {
        return Err(format!("{path}: {what} size must be positive"));
    }
    let edges: Vec<Vec3> = match &spec.edges {
        EdgeSel::Keyword(k) if k == "all" => sharp_edges(&body, spec.min_angle_deg).map_err(|e| format!("{path}: edges: {e}"))?,
        EdgeSel::Keyword(k) => return Err(format!("{path}: unknown edge selection {k:?} (\"all\" or a list of points)")),
        EdgeSel::Points(p) => p.iter().map(|q| v3(*q)).collect(),
    };
    if edges.is_empty() {
        ctx.warnings.push(format!("{path}: no edges to {what}"));
        return Ok(body);
    }
    check_time(path, ctx)?;
    let op = |b: &Body, edges: &[Vec3], r: f64| {
        let t = Instant::now();
        let out = if fillet { kernel::fillet(b, edges, r) } else { kernel::chamfer(b, edges, r) };
        debug(|| format!("{path}: {what} {} edge(s) at {r}: {} in {:.2}s", edges.len(), if out.is_ok() { "ok" } else { "failed" }, t.elapsed().as_secs_f64()));
        out
    };
    let t_first = Instant::now();
    let first = match op(&body, &edges, r) {
        Ok(b) => return Ok(b),
        Err(e) => e,
    };
    let first_took = t_first.elapsed();
    // One smaller size for all the edges together: the size the kernel says fits (when it
    // says, and it is at least a quarter of the request), else half. A failed many-edge blend
    // can take many seconds, so no longer ladder.
    let smaller = size_hint(&first.to_string()).filter(|h| *h >= r / 4.0 - 1e-12 && *h < r).unwrap_or(r * 0.5);
    check_time(path, ctx)?;
    // Skipped when the failed attempt took more than half the time left, to leave time for the
    // edge-by-edge pass.
    let left = ctx.soft_deadline.saturating_duration_since(Instant::now());
    if first_took * 2 <= left
        && let Ok(b) = op(&body, &edges, smaller)
    {
        ctx.warnings.push(format!("{path}: {what} {r} failed ({first}); used {smaller}"));
        return Ok(b);
    }
    // One edge at a time, keeping each that works.
    if edges.len() > 1 {
        let mut acc = body.clone();
        let (mut done, mut reduced, mut tried) = (0usize, 0usize, 0usize);
        for e in &edges {
            if Instant::now() >= ctx.soft_deadline {
                break;
            }
            tried += 1;
            if let Ok(b) = op(&acc, std::slice::from_ref(e), r) {
                acc = b;
                done += 1;
            } else if let Ok(b) = op(&acc, std::slice::from_ref(e), r * 0.5) {
                acc = b;
                done += 1;
                reduced += 1;
            }
        }
        let unfinished = if tried < edges.len() { format!("; ran out of time after trying {tried}") } else { String::new() };
        if done > 0 {
            ctx.warnings.push(format!(
                "{path}: {what} {r} on all {} edges together failed ({first}); applied edge by edge to {done} of them ({reduced} at {}){unfinished}",
                edges.len(),
                r * 0.5
            ));
            return Ok(acc);
        }
    }
    ctx.warnings.push(format!("{path}: {what} {r} on {} edge(s) failed ({first}); kept the body without it", edges.len()));
    Ok(body)
}

/// Progress lines on stderr when `POLYCORE_STEP_DEBUG` is set.
fn debug(msg: impl FnOnce() -> String) {
    if std::env::var_os("POLYCORE_STEP_DEBUG").is_some() {
        eprintln!("debug: {}", msg());
    }
}

/// The size a refusal says would fit ("... the largest radius that fits is 4 mm").
fn size_hint(msg: &str) -> Option<f64> {
    let rest = &msg[msg.find("that fits is ")? + "that fits is ".len()..];
    let num: String = rest.chars().take_while(|c| c.is_ascii_digit() || *c == '.' || *c == 'e' || *c == '-').collect();
    num.parse::<f64>().ok().filter(|v| v.is_finite() && *v > 0.0)
}

// ---------------------------------------------------------------------------------------------
// One part
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Exact,
    ExactWithWarnings,
    Faceted,
    Failed,
}

impl Status {
    pub fn as_str(self) -> &'static str {
        match self {
            Status::Exact => "exact",
            Status::ExactWithWarnings => "exact_with_warnings",
            Status::Faceted => "faceted",
            Status::Failed => "failed",
        }
    }
}

/// What a part became.
#[derive(Clone, Debug, Serialize)]
pub struct PartResult {
    pub name: String,
    pub body: String,
    pub status: Status,
    /// `brep` (exact recipe), `prismatic` (mesh converted to planar/cylindrical faces) or
    /// `faceted_brep` (a planar face per mesh triangle, coplanar ones merged); absent when failed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub representation: Option<&'static str>,
    pub warnings: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume_mm3: Option<f64>,
    pub seconds: f64,
    #[serde(skip)]
    pub solid: Option<Body>,
}

struct Outcome {
    status: Status,
    representation: Option<&'static str>,
    solid: Option<Body>,
    warnings: Vec<String>,
    error: Option<String>,
    volume: Option<f64>,
}

fn panic_text(p: &(dyn std::any::Any + Send)) -> String {
    p.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_else(|| "unknown panic".into())
}

fn volume(b: &Body) -> Result<f64, String> {
    let v = kernel::measure_mass(b).map_err(|e| format!("measure: {e}"))?.volume;
    if !v.is_finite() {
        return Err("measure: non-finite volume".into());
    }
    Ok(v)
}

/// The exact recipe, checked: a closed body with positive volume.
/// The exact recipe (rigid transforms pushed down to the primitives first), checked: a closed
/// body with positive volume. The flag says a step had to fall back to a faceted boolean.
fn exact(node: &Node, deadline: Instant, budget: Duration) -> Result<(Body, Vec<String>, f64, bool), String> {
    let soft_deadline = deadline.checked_sub(budget.mul_f64(0.2)).unwrap_or(deadline);
    let mut ctx = Ctx { warnings: Vec::new(), deadline, soft_deadline, faceted: false };
    // `POLYCORE_STEP_NO_PUSHDOWN` builds the recipe as written (for diagnosis).
    let node = if std::env::var_os("POLYCORE_STEP_NO_PUSHDOWN").is_some() { node.clone() } else { push_down(node, &Rigid::IDENTITY) };
    // A box as the last tool of the top-level cut/intersect (halving a symmetric body on its
    // mirror plane, typically) is applied first: the same solid, the later cuts run on the
    // smaller body, and a body that other cuts have left awkward is not split last. The order
    // as written is the retry.
    let last_is_box = match &node {
        Node::Cut(v) | Node::Intersect(v) => v.len() >= 2 && v.last().and_then(oriented_box).is_some(),
        _ => false,
    };
    let sunk = sink_last_tool(&node);
    let (first, second) = match (&sunk, last_is_box) {
        (Some(s), true) => (s, Some((&node, "as written"))),
        (Some(s), false) => (&node, Some((s, "with the last tool applied first"))),
        (None, _) => (&node, None),
    };
    let body = match kernel::with_deadline(deadline, || build_node(first, "solid", &mut ctx)) {
        Ok(b) => b,
        Err(e) => match second {
            Some((alt, how)) if Instant::now() < soft_deadline => {
                debug(|| format!("{e}; retrying {how}"));
                ctx.warnings.clear();
                ctx.faceted = false;
                let b = kernel::with_deadline(deadline, || build_node(alt, "solid", &mut ctx)).map_err(|e2| format!("{e}; {how}: {e2}"))?;
                ctx.warnings.push(format!("{e}; built {how} instead"));
                b
            }
            _ => return Err(e),
        },
    };
    let v = volume(&body)?;
    if !(v > 1e-9) {
        return Err(format!("solid: degenerate result (volume {v} mm3)"));
    }
    Ok((body, ctx.warnings, v, ctx.faceted))
}

/// The top-level `cut`/`intersect` with its last tool `h` moved to the innermost first operand:
/// `(Y - T) - H = (Y - H) - T`, `(Y - T) ∩ H = (Y ∩ H) - T`, `(Y ∩ Z) op H = (Y op H) ∩ Z`.
/// Stops at anything else (a fillet does not commute). `None` when nothing moves.
pub fn sink_last_tool(n: &Node) -> Option<Node> {
    fn sink(x: &Node, h: &Node, cut: bool, moved: &mut bool) -> Node {
        match x {
            Node::Cut(v) | Node::Intersect(v) if v.len() >= 2 => {
                *moved = true;
                let mut w = v.clone();
                w[0] = sink(&v[0], h, cut, moved);
                if matches!(x, Node::Cut(_)) { Node::Cut(w) } else { Node::Intersect(w) }
            }
            _ if cut => Node::Cut(vec![x.clone(), h.clone()]),
            _ => Node::Intersect(vec![x.clone(), h.clone()]),
        }
    }
    let (v, cut) = match n {
        Node::Cut(v) if v.len() >= 2 => (v, true),
        Node::Intersect(v) if v.len() == 2 => (v, false),
        _ => return None,
    };
    let (h, rest) = v.split_last()?;
    let mut moved = false;
    let inner = sink(&rest[0], h, cut, &mut moved);
    if !moved {
        return None;
    }
    if rest.len() == 1 {
        return Some(inner);
    }
    let mut w = rest.to_vec();
    w[0] = inner;
    Some(if cut { Node::Cut(w) } else { Node::Intersect(w) })
}

/// Why this body cannot be written to STEP on its own, if it cannot.
pub fn export_error(body: &Body) -> Option<String> {
    let one = [ExportBody { name: "check".into(), body, color: None }];
    kernel::guard("step export", || kernel::step_export_bodies(&one, &StepHeader::default())).err().map(|e| e.to_string())
}

/// A B-rep body as a faceted B-rep (for a body the STEP writer refuses): the body intersected
/// with a closed box mesh around it by `faceted_boolean`, which facets the body's curved faces
/// itself and keeps its planar ones. (The body's display tessellation, welded by `mesh_body`, is
/// not reliably closed.)
pub fn tessellated_brep(body: &Body) -> Result<(Body, &'static str, f64), String> {
    let bb = kernel::measure_mass(body).map_err(|e| format!("measure: {e}"))?.bbox;
    let pad = (bb.max - bb.min).len() * 0.05 + 1.0;
    let (lo, hi) = (bb.min - Vec3::new(pad, pad, pad), bb.max + Vec3::new(pad, pad, pad));
    let positions = vec![
        Vec3::new(lo.x, lo.y, lo.z),
        Vec3::new(hi.x, lo.y, lo.z),
        Vec3::new(hi.x, hi.y, lo.z),
        Vec3::new(lo.x, hi.y, lo.z),
        Vec3::new(lo.x, lo.y, hi.z),
        Vec3::new(hi.x, lo.y, hi.z),
        Vec3::new(hi.x, hi.y, hi.z),
        Vec3::new(lo.x, hi.y, hi.z),
    ];
    let triangles = vec![[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
    let frame = Mesh { positions, triangles, ..Default::default() };
    let tol = facet_tol(body.size());
    let b = kernel::guard("faceted boolean", || kernel::faceted_boolean(body, &frame, BoolOp::Intersect, tol))
        .map_err(|e| format!("faceting: {e}"))?
        .ok_or_else(|| "faceting: empty result".to_string())?;
    let v = volume(&b)?;
    Ok((b, "faceted_brep", v))
}

/// A mesh body as an exportable B-rep (STEP takes no mesh bodies): planar and cylindrical faces
/// where `convert_prismatic_mesh` certifies them, else one planar face per triangle (merged
/// where coplanar), made by intersecting the mesh with a box around it (`faceted_boolean`).
/// Returns the body, its representation and its volume.
pub fn mesh_to_brep(mb: &Body) -> Result<(Body, &'static str, f64), String> {
    let Some((pos, tris)) = mb.triangle_mesh() else {
        let v = volume(mb)?;
        return Ok((mb.clone(), "brep", v));
    };
    if !mb.is_closed_mesh() {
        return Err("mesh: not closed (every triangle side must be shared by two triangles)".into());
    }
    let vm = volume(mb)?;
    if !(vm > 0.0) {
        return Err(format!("mesh: volume {vm} mm3 (inside out or degenerate)"));
    }
    let close = |v: f64| (v - vm).abs() <= 1e-3 * vm.abs() + 1e-9;
    if let Ok(b) = kernel::guard("convert prismatic mesh", || kernel::convert_prismatic_mesh(mb, 0.01))
        && let Ok(v) = volume(&b)
        && close(v)
    {
        return Ok((b, "prismatic", v));
    }
    let (mut lo, mut hi) = (Vec3::new(f64::MAX, f64::MAX, f64::MAX), Vec3::new(f64::MIN, f64::MIN, f64::MIN));
    for p in pos {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    // The mesh as given, then welded (Blender's UV-sphere poles leave coincident vertices and
    // zero-area triangles behind), each in a frame box of two sizes (the boolean's tolerances
    // follow the overall size).
    let diag = (hi - lo).len();
    let mut last = String::new();
    for (mp, mt) in [(pos.to_vec(), tris.to_vec()), weld(pos, tris, diag * 1e-4)] {
        let mesh = Mesh { positions: mp, triangles: mt, ..Default::default() };
        for rel in [0.05, 0.37] {
            let pad = Vec3::new(1.0, 1.0, 1.0) * (diag * rel + 1.0);
            let frame = kernel::box_solid(lo - pad, hi + pad).map_err(|e| format!("mesh: frame box: {e}"))?;
            match kernel::guard("faceted boolean", || kernel::faceted_boolean(&frame, &mesh, BoolOp::Intersect, 0.01)) {
                Ok(Some(b)) => match volume(&b) {
                    Ok(v) if close(v) => return Ok((b, "faceted_brep", v)),
                    Ok(v) => last = format!("mesh to B-rep: volume {v} mm3, the mesh has {vm} mm3"),
                    Err(e) => last = format!("mesh to B-rep: {e}"),
                },
                Ok(None) => last = "mesh to B-rep: empty result".into(),
                Err(e) => last = format!("mesh to B-rep: {e}"),
            }
        }
    }
    // Last: a planar face per triangle of the welded mesh, sewn into a solid.
    let (wp, wt) = weld(pos, tris, diag * 1e-4);
    let faces: Vec<kernel::FaceSpec> = wt
        .iter()
        .map(|t| {
            let p = |i: u32| wp[i as usize];
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            kernel::FaceSpec {
                loops: vec![vec![kernel::EdgeSpec::Line { a, b }, kernel::EdgeSpec::Line { a: b, b: c }, kernel::EdgeSpec::Line { a: c, b: a }]],
                surface: kernel::SurfSpec::Plane,
                probe: None,
            }
        })
        .collect();
    match kernel::guard("sew", || kernel::sew(&faces)) {
        Ok(b) => match volume(&b) {
            Ok(v) if close(v) => return Ok((b, "faceted_brep", v)),
            Ok(v) => last = format!("{last}; sewn: volume {v} mm3, the mesh has {vm} mm3"),
            Err(e) => last = format!("{last}; sewn: {e}"),
        },
        Err(e) => last = format!("{last}; sewn: {e}"),
    }
    Err(last)
}

/// Vertices within `tol` merged (on a grid of that size), and triangles that collapse or repeat
/// dropped.
fn weld(pos: &[Vec3], tris: &[[u32; 3]], tol: f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    use std::collections::{BTreeMap, BTreeSet};
    let tol = tol.max(1e-9);
    let key = |p: Vec3| ((p.x / tol).round() as i64, (p.y / tol).round() as i64, (p.z / tol).round() as i64);
    let mut ids: BTreeMap<(i64, i64, i64), u32> = BTreeMap::new();
    let mut out_pos = Vec::new();
    let remap: Vec<u32> = pos
        .iter()
        .map(|p| {
            *ids.entry(key(*p)).or_insert_with(|| {
                out_pos.push(*p);
                (out_pos.len() - 1) as u32
            })
        })
        .collect();
    let mut seen = BTreeSet::new();
    let mut out_tris = Vec::new();
    for t in tris {
        let (Some(&a), Some(&b), Some(&c)) = (remap.get(t[0] as usize), remap.get(t[1] as usize), remap.get(t[2] as usize)) else { continue };
        if a == b || b == c || a == c {
            continue;
        }
        let mut k = [a, b, c];
        k.sort_unstable();
        if seen.insert(k) {
            out_tris.push([a, b, c]);
        }
    }
    (out_pos, out_tris)
}

/// The mesh fallback.
fn faceted(m: &MeshDef) -> Result<(Body, &'static str, f64), String> {
    let mb = mesh_from(m).map_err(|e| format!("mesh: {e}"))?;
    let (b, rep, v) = mesh_to_brep(&mb)?;
    if let Some(e) = export_error(&b) {
        return Err(format!("mesh: not exportable: {e}"));
    }
    Ok((b, rep, v))
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    /// The solid recipe, then the mesh if it fails.
    Exact,
    /// Only the mesh (the solid ran out of time).
    MeshOnly,
}

fn run_part(part: &Part, mode: Mode, deadline: Instant, budget: Duration) -> Outcome {
    let mut error = None;
    if mode == Mode::Exact
        && let Some(node) = &part.solid
    {
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let (body, mut warnings, v, faceted) = exact(node, deadline, budget)?;
            let (body, v, rep) = if body.is_mesh() {
                // A solid recipe that is (or became) a mesh: faceted.
                let (b, rep, v) = mesh_to_brep(&body).map_err(|e| format!("solid: {e}"))?;
                (b, v, rep)
            } else if faceted {
                (body, v, "faceted_brep")
            } else {
                (body, v, "brep")
            };
            // One body the STEP writer refuses must not cost the whole file: write its own
            // tessellation instead.
            match export_error(&body) {
                None => Ok((body, warnings, v, rep)),
                Some(why) => {
                    let (b, rep2, v2) = tessellated_brep(&body).map_err(|e| format!("solid: exact body not exportable: {why}; {e}"))?;
                    if let Some(e) = export_error(&b) {
                        return Err(format!("solid: exact body not exportable: {why}; its tessellation neither: {e}"));
                    }
                    warnings.push(format!("exact body not exportable: {why}; wrote its tessellation ({v:.1} -> {v2:.1} mm3)"));
                    Ok((b, warnings, v2, if rep2 == "prismatic" { "prismatic" } else { "faceted_brep" }))
                }
            }
        }))
        .unwrap_or_else(|p| Err(format!("solid: panic: {}", panic_text(&*p))));
        match r {
            Ok((body, warnings, v, rep)) => {
                let status = match (rep, warnings.is_empty()) {
                    ("brep", true) => Status::Exact,
                    ("brep", false) => Status::ExactWithWarnings,
                    _ => Status::Faceted,
                };
                return Outcome { status, representation: Some(rep), solid: Some(body), warnings, error: None, volume: Some(v) };
            }
            Err(e) => error = Some(e),
        }
    }
    mesh_outcome(part, error)
}

fn mesh_outcome(part: &Part, solid_error: Option<String>) -> Outcome {
    let failed = |error: String| Outcome { status: Status::Failed, representation: None, solid: None, warnings: vec![], error: Some(error), volume: None };
    let Some(m) = &part.mesh else {
        return failed(solid_error.unwrap_or_else(|| "the part has neither a solid nor a mesh".into()));
    };
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| faceted(m))).unwrap_or_else(|p| Err(format!("mesh: panic: {}", panic_text(&*p))));
    match r {
        Ok((b, rep, v)) => Outcome { status: Status::Faceted, representation: Some(rep), solid: Some(b), warnings: vec![], error: solid_error, volume: Some(v) },
        Err(e) => failed(match solid_error {
            Some(s) => format!("{s}; {e}"),
            None => e,
        }),
    }
}

// ---------------------------------------------------------------------------------------------
// All parts, on worker threads with a time budget each
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Options {
    pub threads: usize,
    pub only: Option<Vec<String>>,
    pub timeout: Duration,
}

impl Default for Options {
    fn default() -> Self {
        Options { threads: std::thread::available_parallelism().map(usize::from).unwrap_or(1), only: None, timeout: Duration::from_secs(60) }
    }
}

struct Running {
    idx: usize,
    mode: Mode,
    deadline: Instant,
    budget: Duration,
}

/// Build every selected part (recipe order kept). A part that overruns its budget is
/// abandoned (its thread is left to finish on its own; the result is ignored) and gets its
/// mesh, under a fresh budget.
pub fn build_parts(recipe: &Recipe, opts: &Options) -> Vec<PartResult> {
    let parts: Vec<Arc<Part>> = recipe
        .parts
        .iter()
        .filter(|p| opts.only.as_ref().is_none_or(|o| o.iter().any(|n| n == &p.name)))
        .cloned()
        .map(Arc::new)
        .collect();
    let n = parts.len();
    let threads = opts.threads.max(1);
    let mut started: Vec<Option<Instant>> = vec![None; n];
    let mut timed_out: Vec<Option<String>> = vec![None; n];
    let mut done: Vec<Option<(Outcome, f64)>> = (0..n).map(|_| None).collect();
    let mut queue: VecDeque<(usize, Mode)> = (0..n).map(|i| (i, Mode::Exact)).collect();
    let mut running: Vec<Running> = Vec::new();
    let (tx, rx) = mpsc::channel::<(usize, Mode, Outcome)>();

    loop {
        while running.len() < threads {
            let Some((idx, mode)) = queue.pop_front() else { break };
            let now = Instant::now();
            started[idx].get_or_insert(now);
            // The mesh fallback is quick, but a tight budget for the solid should not starve it.
            let budget = if mode == Mode::MeshOnly { opts.timeout.max(Duration::from_secs(30)) } else { opts.timeout };
            let deadline = now + budget;
            let (part, tx) = (parts[idx].clone(), tx.clone());
            let spawned = std::thread::Builder::new()
                .name(format!("part-{idx}"))
                .stack_size(256 << 20)
                .spawn(move || {
                    let o = run_part(&part, mode, deadline, budget);
                    let _ = tx.send((idx, mode, o));
                });
            match spawned {
                Ok(_) => running.push(Running { idx, mode, deadline, budget }),
                Err(e) => {
                    let o = Outcome {
                        status: Status::Failed,
                        representation: None,
                        solid: None,
                        warnings: vec![],
                        error: Some(format!("could not start a worker thread: {e}")),
                        volume: None,
                    };
                    done[idx] = Some((o, 0.0));
                }
            }
        }
        if running.is_empty() {
            break;
        }
        let next = running.iter().map(|r| r.deadline).min().unwrap_or_else(Instant::now);
        match rx.recv_timeout(next.saturating_duration_since(Instant::now())) {
            Ok((idx, mode, mut o)) => {
                let Some(pos) = running.iter().position(|r| r.idx == idx && r.mode == mode) else { continue };
                running.remove(pos);
                if let Some(t) = timed_out[idx].take() {
                    o.error = Some(match o.error {
                        Some(e) => format!("{t}; {e}"),
                        None => t,
                    });
                }
                let secs = started[idx].map_or(0.0, |s| s.elapsed().as_secs_f64());
                done[idx] = Some((o, secs));
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                let now = Instant::now();
                let (late, keep): (Vec<Running>, Vec<Running>) = running.into_iter().partition(|r| r.deadline <= now);
                running = keep;
                for r in late {
                    let msg = format!("timed out after {} s", r.budget.as_secs_f64());
                    match r.mode {
                        Mode::Exact if parts[r.idx].mesh.is_some() => {
                            timed_out[r.idx] = Some(format!("solid: {msg}"));
                            queue.push_back((r.idx, Mode::MeshOnly));
                        }
                        _ => {
                            let prior = timed_out[r.idx].take();
                            let what = if r.mode == Mode::Exact { "solid" } else { "mesh" };
                            let error = match prior {
                                Some(p) => format!("{p}; {what}: {msg}"),
                                None => format!("{what}: {msg}"),
                            };
                            let o = Outcome { status: Status::Failed, representation: None, solid: None, warnings: vec![], error: Some(error), volume: None };
                            let secs = started[r.idx].map_or(0.0, |s| s.elapsed().as_secs_f64());
                            done[r.idx] = Some((o, secs));
                        }
                    }
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    parts
        .iter()
        .zip(done)
        .map(|(p, d)| {
            let (o, seconds) = d.unwrap_or_else(|| {
                (
                    Outcome { status: Status::Failed, representation: None, solid: None, warnings: vec![], error: Some("not run".into()), volume: None },
                    0.0,
                )
            });
            PartResult {
                name: p.name.clone(),
                body: p.body.clone(),
                status: o.status,
                representation: o.representation,
                warnings: o.warnings,
                error: o.error,
                volume_mm3: o.volume,
                seconds,
                solid: o.solid,
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------
// STEP assembly
// ---------------------------------------------------------------------------------------------

/// A rigid version of a placement: the rotation columns made orthonormal (scale and shear
/// dropped; a mirror cannot be kept). `None` when the placement is unusable.
fn rigid(m: [[f64; 4]; 4]) -> Option<([[f64; 4]; 4], bool)> {
    let col = |c: usize| Vec3::new(m[c][0], m[c][1], m[c][2]);
    let (x, y, z) = (col(0), col(1), col(2));
    let t = Vec3::new(m[3][0], m[3][1], m[3][2]);
    if !(x.is_finite() && y.is_finite() && z.is_finite() && t.is_finite()) {
        return None;
    }
    let xn = x.normalized()?;
    let yn = (y - xn * y.dot(xn)).normalized()?;
    let zn = xn.cross(yn);
    // Blender's float32 matrices are off by a few 1e-6: only a real scale or shear is reported.
    let changed = (xn - x).len() > 1e-4 || (yn - y).len() > 1e-4 || (zn - z).len() > 1e-4 || m[0][3].abs() + m[1][3].abs() + m[2][3].abs() > 1e-9;
    let out = [[xn.x, xn.y, xn.z, 0.0], [yn.x, yn.y, yn.z, 0.0], [zn.x, zn.y, zn.z, 0.0], [t.x, t.y, t.z, 1.0]];
    Some((out, changed))
}

/// Solids a body is written as (a body of several lumps is several solids).
pub fn solid_count(b: &Body) -> usize {
    if b.is_mesh() || b.shell_count() < 2 {
        return 1;
    }
    b.lumps().map(|l| l.len().max(1)).unwrap_or(1)
}

pub struct Assembly {
    pub text: String,
    pub warnings: Vec<String>,
    /// Solids written (lumps counted separately).
    pub solids: usize,
}

/// The STEP text: root product `recipe.name`, one placed child product per recipe body holding
/// its parts (in recipe order). Parts of an unknown body go on the root.
pub fn assemble<'r>(recipe: &Recipe, results: &'r [PartResult], file_name: &str) -> Result<Assembly, String> {
    let mut warnings = Vec::new();
    let mut products: Vec<ExportProduct> = vec![ExportProduct { name: recipe.name.clone(), bodies: vec![], children: vec![] }];
    let colour = |r: &PartResult| recipe.parts.iter().find(|p| p.name == r.name).and_then(|p| p.color);
    fn export_body<'a>(r: &'a PartResult, color: Option<[f32; 3]>) -> Option<ExportBody<'a>> {
        Some(ExportBody { name: r.name.clone(), body: r.solid.as_ref()?, color })
    }
    let export = |r: &'r PartResult| export_body(r, colour(r));
    let mut solids = 0usize;
    for bd in &recipe.bodies {
        let mine: Vec<ExportBody> = results.iter().filter(|r| r.body == bd.name).filter_map(export).collect();
        if mine.is_empty() {
            continue;
        }
        let (m, changed) = rigid(matrix(&bd.placement)).ok_or_else(|| format!("body {}: unusable placement", bd.name))?;
        if changed {
            warnings.push(format!("body {}: placement had scale/shear/projection; exported its rigid part", bd.name));
        }
        solids += results.iter().filter(|r| r.body == bd.name).filter_map(|r| r.solid.as_ref()).map(solid_count).sum::<usize>();
        let idx = products.len();
        products.push(ExportProduct { name: bd.name.clone(), bodies: mine, children: vec![] });
        products[0].children.push((idx, m, bd.name.clone()));
    }
    for r in results.iter().filter(|r| !recipe.bodies.iter().any(|b| b.name == r.body)) {
        if let Some(e) = export(r) {
            warnings.push(format!("part {}: unknown body {:?}; placed on the root product", r.name, r.body));
            solids += solid_count(e.body);
            products[0].bodies.push(e);
        }
    }
    let header = StepHeader {
        file_name: file_name.to_string(),
        time_stamp: String::new(),
        author: String::new(),
        organization: String::new(),
        description: format!("polycore-step: {}", recipe.name),
    };
    let text = kernel::guard("step export", || kernel::step_export_products(&products, 0, &header)).map_err(|e| format!("STEP export: {e}"))?;
    Ok(Assembly { text, warnings, solids })
}

/// [`assemble`]; when the whole file is refused, every part whose body cannot be written on its
/// own is marked failed and left out, and the rest is written.
pub fn assemble_checked(recipe: &Recipe, results: &mut [PartResult], file_name: &str) -> Result<Assembly, String> {
    let first = match assemble(recipe, results, file_name) {
        Ok(a) => return Ok(a),
        Err(e) => e,
    };
    let mut dropped = Vec::new();
    for r in results.iter_mut() {
        if let Some(why) = r.solid.as_ref().and_then(export_error) {
            r.status = Status::Failed;
            r.solid = None;
            r.representation = None;
            r.volume_mm3 = None;
            r.error = Some(format!("not exportable: {why}"));
            dropped.push(r.name.clone());
        }
    }
    if dropped.is_empty() {
        return Err(first);
    }
    let mut a = assemble(recipe, results, file_name)?;
    a.warnings.push(format!("the STEP export failed ({first}); left out {} unexportable part(s): {}", dropped.len(), dropped.join(", ")));
    Ok(a)
}

#[derive(Clone, Debug, Serialize)]
pub struct Validation {
    pub valid: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entities: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    pub reimported: bool,
    pub solids_expected: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solids_read: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solids_open: Option<usize>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub import_warnings: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub import_error: Option<String>,
}

impl Validation {
    pub fn ok(&self) -> bool {
        self.valid && self.reimported && self.solids_read == Some(self.solids_expected)
    }
}

/// `step_validate`, then a re-import counting the solids read back.
pub fn validate(text: &str, solids_expected: usize) -> Validation {
    let (valid, entities, error) = match kernel::step_validate(text) {
        Ok(n) => (true, Some(n), None),
        Err(e) => (false, None, Some(e)),
    };
    let mut v = Validation {
        valid,
        entities,
        error,
        reimported: false,
        solids_expected,
        solids_read: None,
        solids_open: None,
        import_warnings: vec![],
        import_error: None,
    };
    match kernel::guard("step import", || kernel::step_import(text)) {
        Ok(imp) => {
            v.reimported = true;
            v.solids_read = Some(imp.bodies.len());
            v.solids_open = Some(imp.bodies.iter().filter(|b| !b.closed).count());
            v.import_warnings = imp.warnings;
        }
        Err(e) => v.import_error = Some(e.to_string()),
    }
    v
}

// ---------------------------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, Default, Serialize)]
pub struct Totals {
    pub parts: usize,
    pub exact: usize,
    pub exact_with_warnings: usize,
    pub faceted: usize,
    pub failed: usize,
    pub volume_mm3: f64,
    pub seconds: f64,
}

pub fn totals(results: &[PartResult], seconds: f64) -> Totals {
    let count = |s: Status| results.iter().filter(|r| r.status == s).count();
    Totals {
        parts: results.len(),
        exact: count(Status::Exact),
        exact_with_warnings: count(Status::ExactWithWarnings),
        faceted: count(Status::Faceted),
        failed: count(Status::Failed),
        volume_mm3: results.iter().filter_map(|r| r.volume_mm3).sum(),
        seconds,
    }
}

#[derive(Debug, Serialize)]
pub struct Report<'a> {
    pub name: &'a str,
    pub output: &'a str,
    pub parts: &'a [PartResult],
    pub totals: Totals,
    pub assembly_warnings: &'a [String],
    /// Why no STEP was written (e.g. every part failed).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub export_error: Option<&'a str>,
    pub validation: Option<&'a Validation>,
}

/// One line per part for stderr.
pub fn summary_line(r: &PartResult) -> String {
    let mut s = format!("{:<20} {} [{}]", r.status.as_str(), r.name, r.body);
    if let Some(rep) = r.representation {
        s += &format!(" {rep}");
    }
    if let Some(v) = r.volume_mm3 {
        s += &format!(" {v:.1} mm3");
    }
    s += &format!(" {:.2}s", r.seconds);
    if !r.warnings.is_empty() {
        s += &format!(" warnings: {}", r.warnings.join(" | "));
    }
    if let Some(e) = &r.error {
        s += &format!(" error: {e}");
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrices_are_column_major() {
        let m = matrix(&[1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 7., 8., 9., 1.]);
        assert_eq!(m[3], [7., 8., 9., 1.]);
    }

    #[test]
    fn rigid_drops_scale_and_flags_it() {
        let (m, changed) = rigid(matrix(&[2., 0., 0., 0., 0., 2., 0., 0., 0., 0., 2., 0., 1., 2., 3., 1.])).expect("usable");
        assert!(changed);
        assert_eq!(m[0], [1., 0., 0., 0.]);
        assert_eq!(m[3], [1., 2., 3., 1.]);
        let (_, changed) = rigid(matrix(&identity16())).expect("usable");
        assert!(!changed);
    }

    #[test]
    fn polygon_drops_repeated_points() {
        let lp = polygon("t", &[[0., 0.], [1., 0.], [1., 0.], [1., 1.], [0., 0.]]).expect("loop");
        assert_eq!(lp.segs.len(), 3);
        assert!(polygon("t", &[[0., 0.], [1., 0.], [2., 0.]]).is_err());
    }

    #[test]
    fn size_hint_reads_the_refusal() {
        assert_eq!(size_hint("the blend is larger than the neighbouring faces; the largest radius that fits is 4 mm"), Some(4.0));
        assert_eq!(size_hint("the largest radius that fits is 0.35 mm"), Some(0.35));
        assert_eq!(size_hint("no hint"), None);
    }

    #[test]
    fn recipe_nodes_parse() {
        let r = parse_recipe(
            r#"{"name":"x","parts":[{"name":"p","body":"b","solid":{"fillet":{"body":{"box":{"min":[0,0,0],"max":[1,1,1]}},"radius":0.1}}},
                {"name":"q","body":"b","solid":{"chamfer":{"body":{"mesh":{"positions":[],"triangles":[]}},"distance":0.1,"edges":[[0,0,1]]}}}]}"#,
        )
        .expect("parses");
        let Some(Node::Fillet(f)) = &r.parts[0].solid else { panic!("fillet") };
        assert!(matches!(&f.edges, EdgeSel::Keyword(k) if k == "all"));
        assert_eq!(f.min_angle_deg, 20.0);
        assert!(matches!(&r.parts[1].solid, Some(Node::Chamfer(c)) if c.radius == 0.1));
        assert!(parse_recipe(r#"{"name":"x","parts":[{"name":"p","body":"b","solid":{"cone":{}}}]}"#).is_err());
    }
}

#[cfg(test)]
mod push_tests {
    use super::*;

    fn bx(lo: [f64; 3], hi: [f64; 3]) -> Node {
        Node::Box { min: lo, max: hi }
    }

    #[test]
    fn sinking_moves_the_last_tool_inside() {
        let (y, t, h) = (bx([0.; 3], [1.; 3]), bx([0.; 3], [0.5; 3]), bx([-9.; 3], [9., 0., 9.]));
        let n = Node::Cut(vec![Node::Cut(vec![y, t]), h]);
        let Some(Node::Cut(v)) = sink_last_tool(&n) else { panic!("sunk") };
        assert_eq!(v.len(), 2);
        let Node::Cut(inner) = &v[0] else { panic!("inner cut") };
        assert!(matches!(inner[1], Node::Box { max: [9., 0., 9.], .. }));
        // Nothing to sink into.
        assert!(sink_last_tool(&Node::Cut(vec![bx([0.; 3], [1.; 3]), bx([0.; 3], [2.; 3])])).is_none());
    }

    #[test]
    fn pushed_box_is_an_oriented_box() {
        let m = Rigid::from_matrix(&[0., 1., 0., 0., -1., 0., 0., 0., 0., 0., 1., 0., 5., 0., 0., 1.]).expect("rigid");
        let n = push_down(&bx([0., 0., 0.], [2., 3., 4.]), &m);
        let ob = oriented_box(&n).expect("box");
        assert_eq!(ob.ext, [2., 3., 4.]);
        assert!(Rigid::from_matrix(&[2., 0., 0., 0., 0., 2., 0., 0., 0., 0., 2., 0., 0., 0., 0., 1.]).is_none());
    }
}
