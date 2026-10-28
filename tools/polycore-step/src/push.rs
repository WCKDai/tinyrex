//! Rigid transforms pushed down to the leaves of a recipe, so that every solid is built where it
//! ends up and booleans and blends run in the final frame. PolyCore's `transform_matrix` of a
//! finished body can leave surfaces its STEP writer refuses ("Transform does not keep the surface
//! one of revolution": a filleted cylinder or box turned by a rotation); a cylinder built on the
//! turned axis has no such problem.
//!
//! A matrix that is not a rotation (or a mirror) plus a translation (scale, shear) stays a
//! `transform` node over its pushed-down subtree.

use crate::{Blend, EdgeSel, MeshDef, Node, PlaneDef, Section};

/// A rigid motion: `p -> r * p + t`, `r` given by its columns (orthonormal; a mirror when `det < 0`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rigid {
    pub r: [[f64; 3]; 3],
    pub t: [f64; 3],
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = dot(a, a).sqrt();
    (l > 1e-12 && l.is_finite()).then(|| scale(a, 1.0 / l))
}

impl Rigid {
    pub const IDENTITY: Rigid = Rigid { r: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], t: [0., 0., 0.] };

    /// The rigid motion of a column-major 4x4, squared up (Blender's float32 matrices are a little
    /// off); `None` when it scales, shears or projects by more than `1e-4`.
    pub fn from_matrix(m: &[f64; 16]) -> Option<Rigid> {
        if m.iter().any(|x| !x.is_finite()) || m[3].abs() + m[7].abs() + m[11].abs() > 1e-9 || (m[15] - 1.0).abs() > 1e-9 {
            return None;
        }
        let c = |i: usize| [m[i * 4], m[i * 4 + 1], m[i * 4 + 2]];
        let (x, y, z) = (c(0), c(1), c(2));
        const TOL: f64 = 1e-4;
        let unitish = |a: [f64; 3]| (dot(a, a).sqrt() - 1.0).abs() < TOL;
        if !(unitish(x) && unitish(y) && unitish(z) && dot(x, y).abs() < TOL && dot(x, z).abs() < TOL && dot(y, z).abs() < TOL) {
            return None;
        }
        let xn = unit(x)?;
        let yn = unit(sub(y, scale(xn, dot(y, xn))))?;
        let zr = cross(xn, yn);
        // Keep a mirror a mirror.
        let zn = if dot(zr, z) < 0.0 { scale(zr, -1.0) } else { zr };
        Some(Rigid { r: [xn, yn, zn], t: [m[12], m[13], m[14]] })
    }

    pub fn det(&self) -> f64 {
        dot(cross(self.r[0], self.r[1]), self.r[2])
    }
    pub fn mirrors(&self) -> bool {
        self.det() < 0.0
    }
    pub fn is_identity(&self) -> bool {
        *self == Rigid::IDENTITY
    }
    /// No rotation, only a translation.
    fn translation_only(&self) -> bool {
        self.r == Rigid::IDENTITY.r
    }
    pub fn vec(&self, v: [f64; 3]) -> [f64; 3] {
        let [a, b, c] = self.r;
        [a[0] * v[0] + b[0] * v[1] + c[0] * v[2], a[1] * v[0] + b[1] * v[1] + c[1] * v[2], a[2] * v[0] + b[2] * v[1] + c[2] * v[2]]
    }
    pub fn point(&self, p: [f64; 3]) -> [f64; 3] {
        let v = self.vec(p);
        [v[0] + self.t[0], v[1] + self.t[1], v[2] + self.t[2]]
    }
    /// `self` after `inner` (`inner` applied first).
    pub fn then_inner(&self, inner: &Rigid) -> Rigid {
        Rigid { r: [self.vec(inner.r[0]), self.vec(inner.r[1]), self.vec(inner.r[2])], t: self.point(inner.t) }
    }
    pub fn to_matrix(&self) -> [f64; 16] {
        let [a, b, c] = self.r;
        [a[0], a[1], a[2], 0., b[0], b[1], b[2], 0., c[0], c[1], c[2], 0., self.t[0], self.t[1], self.t[2], 1.]
    }
}

/// `m` (column-major 4x4) after `outer`, as a column-major 4x4.
fn compose(outer: &Rigid, m: &[f64; 16]) -> [f64; 16] {
    let o = outer.to_matrix();
    let mut out = [0.0; 16];
    for c in 0..4 {
        for r in 0..4 {
            out[c * 4 + r] = (0..4).map(|k| o[k * 4 + r] * m[c * 4 + k]).sum();
        }
    }
    out
}

/// `n` with every rigid transform moved down to the primitives, then `m` applied to the whole.
pub fn push_down(n: &Node, m: &Rigid) -> Node {
    let id = m.is_identity();
    match n {
        Node::Transform { body, matrix } => match Rigid::from_matrix(matrix) {
            Some(inner) => push_down(body, &m.then_inner(&inner)),
            // Not rigid: kept here, over its pushed-down subtree, with `m` folded in.
            None => Node::Transform { body: Box::new(push_down(body, &Rigid::IDENTITY)), matrix: if id { *matrix } else { compose(m, matrix) } },
        },
        Node::Box { min, max } if id => Node::Box { min: *min, max: *max },
        Node::Box { min, max } if m.translation_only() => Node::Box { min: m.point(*min), max: m.point(*max) },
        Node::Box { min, max } => {
            let (lo, hi) = ([min[0].min(max[0]), min[1].min(max[1]), min[2].min(max[2])], [min[0].max(max[0]), min[1].max(max[1]), min[2].max(max[2])]);
            let d = sub(hi, lo);
            // The box as a rectangle on its turned bottom face, extruded along its turned z
            // (against the plane normal when the motion mirrors).
            let (start, end) = if m.mirrors() { (-d[2], 0.0) } else { (0.0, d[2]) };
            Node::Extrude {
                plane: PlaneDef { origin: m.point(lo), x: m.vec([1., 0., 0.]), y: m.vec([0., 1., 0.]) },
                points: vec![[0., 0.], [d[0], 0.], [d[0], d[1]], [0., d[1]]],
                holes: vec![],
                start,
                end,
            }
        }
        Node::Cylinder { base, axis, radius, height } => Node::Cylinder { base: m.point(*base), axis: m.vec(*axis), radius: *radius, height: *height },
        Node::Sphere { center, radius } => Node::Sphere { center: m.point(*center), radius: *radius },
        Node::Loft { sections, smooth } => Node::Loft {
            sections: sections.iter().map(|s| Section { origin: m.point(s.origin), x: m.vec(s.x), y: m.vec(s.y), points: s.points.clone() }).collect(),
            smooth: *smooth,
        },
        Node::Extrude { plane, points, holes, start, end } => {
            // A mirror turns the plane's normal (x × y) the other way round: the same solid
            // spans the negated distances.
            let (s, e) = if m.mirrors() { (-*end, -*start) } else { (*start, *end) };
            Node::Extrude {
                plane: PlaneDef { origin: m.point(plane.origin), x: m.vec(plane.x), y: m.vec(plane.y) },
                points: points.clone(),
                holes: holes.clone(),
                start: s,
                end: e,
            }
        }
        Node::Union(v) => Node::Union(v.iter().map(|c| push_down(c, m)).collect()),
        Node::Cut(v) => Node::Cut(v.iter().map(|c| push_down(c, m)).collect()),
        Node::Intersect(v) => Node::Intersect(v.iter().map(|c| push_down(c, m)).collect()),
        Node::Fillet(b) => Node::Fillet(push_blend(b, m)),
        Node::Chamfer(b) => Node::Chamfer(push_blend(b, m)),
        Node::Mesh(md) => Node::Mesh(push_mesh(md, m)),
    }
}

fn push_blend(b: &Blend, m: &Rigid) -> Blend {
    Blend {
        body: Box::new(push_down(&b.body, m)),
        radius: b.radius,
        edges: match &b.edges {
            EdgeSel::Points(p) => EdgeSel::Points(p.iter().map(|q| m.point(*q)).collect()),
            k => k.clone(),
        },
        min_angle_deg: b.min_angle_deg,
    }
}

fn push_mesh(md: &MeshDef, m: &Rigid) -> MeshDef {
    MeshDef {
        positions: md.positions.iter().map(|p| m.point(*p)).collect(),
        // A mirror turns the winding inside out; swap two corners to keep it outward.
        triangles: if m.mirrors() { md.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() } else { md.triangles.clone() },
    }
}
