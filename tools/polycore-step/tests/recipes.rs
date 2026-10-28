//! End-to-end recipes: build, export, validate and read back. Expected volumes are analytic.

use std::f64::consts::PI;
use std::time::Duration;

use polycore_step::{Options, PartResult, Recipe, Status, assemble, build_parts, parse_recipe, validate};
use serde_json::{Value, json};

const IDENTITY: [f64; 16] = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

fn opts() -> Options {
    Options { threads: 4, only: None, timeout: Duration::from_secs(60) }
}

fn recipe(parts: Value) -> Recipe {
    let r = json!({ "name": "Test", "bodies": [ { "name": "b", "placement": IDENTITY } ], "parts": parts });
    parse_recipe(&r.to_string()).expect("recipe parses")
}

fn part(name: &str, solid: Option<Value>, mesh: Option<Value>) -> Value {
    let mut p = json!({ "name": name, "body": "b", "color": [0.5, 0.5, 0.5] });
    if let Some(s) = solid {
        p["solid"] = s;
    }
    if let Some(m) = mesh {
        p["mesh"] = m;
    }
    p
}

fn rect_section(z: f64, w: f64, h: f64) -> Value {
    json!({ "origin": [0, 0, z], "x": [1, 0, 0], "y": [0, 1, 0],
            "points": [[-w / 2.0, -h / 2.0], [w / 2.0, -h / 2.0], [w / 2.0, h / 2.0], [-w / 2.0, h / 2.0]] })
}

/// Ruled frustum between two centred rectangles (prismatoid formula).
fn frustum(h: f64, (w0, d0): (f64, f64), (w1, d1): (f64, f64)) -> f64 {
    let mid = ((w0 + w1) / 2.0) * ((d0 + d1) / 2.0);
    h / 6.0 * (w0 * d0 + 4.0 * mid + w1 * d1)
}

fn cube_mesh(lo: [f64; 3], hi: [f64; 3]) -> Value {
    let [x0, y0, z0] = lo;
    let [x1, y1, z1] = hi;
    json!({
        "positions": [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],
        "triangles": [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    })
}

fn tet_mesh(s: f64) -> Value {
    json!({ "positions": [[0,0,0],[s,0,0],[0,s,0],[0,0,s]], "triangles": [[0,2,1],[0,1,3],[1,2,3],[0,3,2]] })
}

fn close(r: &PartResult, expected: f64, rel: f64) {
    let v = r.volume_mm3.unwrap_or_else(|| panic!("{}: no volume ({:?})", r.name, r.error));
    assert!((v - expected).abs() <= rel * expected, "{}: volume {v}, expected {expected}", r.name);
}

/// Export, validate, re-import; the solid count read back must be `solids`.
fn export_ok(recipe: &Recipe, results: &[PartResult], solids: usize) -> String {
    let asm = assemble(recipe, results, "test.step").expect("export");
    assert_eq!(asm.solids, solids, "solids written");
    let v = validate(&asm.text, asm.solids);
    assert!(v.valid, "step_validate: {:?}", v.error);
    assert!(v.reimported, "re-import: {:?}", v.import_error);
    assert_eq!(v.solids_read, Some(solids), "solids read back");
    assert_eq!(v.solids_open, Some(0), "every solid read back closed");
    asm.text
}

#[test]
fn box_with_cylinder_cut_and_filleted_edges() {
    let solid = json!({ "fillet": {
        "body": { "cut": [ { "box": { "min": [-30, -20, 0], "max": [30, 20, 40] } },
                           { "cylinder": { "base": [0, -25, 20], "axis": [0, 1, 0], "radius": 8, "height": 50 } } ] },
        "radius": 2, "edges": [[0, -20, 40], [0, 20, 40]] } });
    let r = recipe(json!([part("Body", Some(solid), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Exact, "{:?} {:?}", res[0].error, res[0].warnings);
    assert_eq!(res[0].representation, Some("brep"));
    // Box, less the bore through it, less two quarter-round edges of radius 2 and length 60.
    let expected = 60.0 * 40.0 * 40.0 - PI * 64.0 * 40.0 - 2.0 * (4.0 - PI) * 60.0;
    close(&res[0], expected, 1e-4);
    export_ok(&r, &res, 1);
}

#[test]
fn fillet_all_edges_of_a_cylinder_skips_the_seam() {
    let solid = json!({ "fillet": { "body": { "cylinder": { "base": [0,0,0], "axis": [0,0,1], "radius": 10, "height": 20 } },
                                   "radius": 2, "edges": "all" } });
    let r = recipe(json!([part("Puck", Some(solid), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Exact, "{:?} {:?}", res[0].error, res[0].warnings);
    // Pappus: each rounded rim removes the spandrel (1 - pi/4) r^2 swept at its centroid radius.
    let rho: f64 = 2.0;
    let spandrel = (1.0 - PI / 4.0) * rho * rho;
    let centroid = 10.0 - rho * (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    let expected = PI * 100.0 * 20.0 - 2.0 * 2.0 * PI * centroid * spandrel;
    close(&res[0], expected, 1e-4);
}

#[test]
fn too_large_fillet_is_reduced_with_a_warning() {
    let solid = json!({ "fillet": { "body": { "box": { "min": [0,0,0], "max": [20,10,8] } }, "radius": 6, "edges": "all" } });
    let r = recipe(json!([part("Brick", Some(solid), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::ExactWithWarnings, "{:?}", res[0].error);
    assert!(res[0].warnings.iter().any(|w| w.contains("used 4")), "{:?}", res[0].warnings);
    export_ok(&r, &res, 1);
}

#[test]
fn three_section_loft_is_ruled() {
    let solid = json!({ "loft": { "sections": [rect_section(0.0, 20.0, 10.0), rect_section(10.0, 30.0, 20.0), rect_section(30.0, 10.0, 10.0)] } });
    let r = recipe(json!([part("Neck", Some(solid), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Exact, "{:?}", res[0].error);
    let expected = frustum(10.0, (20.0, 10.0), (30.0, 20.0)) + frustum(20.0, (30.0, 20.0), (10.0, 10.0));
    close(&res[0], expected, 1e-6);
    export_ok(&r, &res, 1);
}

#[test]
fn smooth_loft_is_available_and_differs() {
    let solid = json!({ "loft": { "smooth": true,
        "sections": [rect_section(0.0, 20.0, 10.0), rect_section(10.0, 30.0, 20.0), rect_section(30.0, 10.0, 10.0)] } });
    let r = recipe(json!([part("Neck", Some(solid), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Exact, "{:?}", res[0].error);
    let ruled = frustum(10.0, (20.0, 10.0), (30.0, 20.0)) + frustum(20.0, (30.0, 20.0), (10.0, 10.0));
    assert!(res[0].volume_mm3.unwrap_or(0.0) > ruled * 1.05, "smooth loft bulges past the ruled one");
}

#[test]
fn hollow_from_cut_of_inner_loft() {
    let outer = json!({ "loft": { "sections": [rect_section(0.0, 40.0, 40.0), rect_section(20.0, 30.0, 30.0)] } });
    let inner = json!({ "loft": { "sections": [rect_section(2.0, 20.0, 20.0), rect_section(18.0, 14.0, 14.0)] } });
    let r = recipe(json!([part("Skull", Some(json!({ "cut": [outer, inner] })), None)]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Exact, "{:?}", res[0].error);
    let expected = frustum(20.0, (40.0, 40.0), (30.0, 30.0)) - frustum(16.0, (20.0, 20.0), (14.0, 14.0));
    close(&res[0], expected, 1e-6);
    // A closed void is one solid with an inner shell, not a second solid.
    export_ok(&r, &res, 1);
}

#[test]
fn mesh_only_parts() {
    let r = recipe(json!([part("Cube", None, Some(cube_mesh([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]))), part("Tet", None, Some(tet_mesh(6.0)))]));
    let res = build_parts(&r, &opts());
    for p in &res {
        assert_eq!(p.status, Status::Faceted, "{}: {:?}", p.name, p.error);
        assert!(p.error.is_none());
    }
    // The cube converts to planar B-rep faces; the tetrahedron is not prismatic.
    assert_eq!(res[0].representation, Some("prismatic"));
    assert_eq!(res[1].representation, Some("faceted_brep"));
    close(&res[0], 1000.0, 1e-9);
    close(&res[1], 36.0, 1e-9);
    export_ok(&r, &res, 2);
}

#[test]
fn impossible_solids_fall_back_to_their_mesh() {
    let coplanar = json!({ "loft": { "sections": [rect_section(0.0, 10.0, 10.0), rect_section(0.0, 5.0, 5.0)] } });
    let emptied = json!({ "cut": [ { "box": { "min": [0,0,0], "max": [5,5,5] } }, { "box": { "min": [-1,-1,-1], "max": [6,6,6] } } ] });
    let bad_box = json!({ "box": { "min": [0,0,0], "max": [0,5,5] } });
    let r = recipe(json!([
        part("Coplanar", Some(coplanar), Some(cube_mesh([0.0, 0.0, 0.0], [10.0, 10.0, 1.0]))),
        part("Emptied", Some(emptied), Some(cube_mesh([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]))),
        part("Flat", Some(bad_box), Some(tet_mesh(3.0))),
    ]));
    let res = build_parts(&r, &opts());
    for p in &res {
        assert_eq!(p.status, Status::Faceted, "{}: {:?}", p.name, p.error);
        assert!(p.error.is_some(), "{}: the solid's error is reported", p.name);
    }
    assert!(res[0].error.as_deref().unwrap_or("").contains("same plane"), "{:?}", res[0].error);
    assert!(res[1].error.as_deref().unwrap_or("").contains("empty"), "{:?}", res[1].error);
    close(&res[0], 100.0, 1e-9);
    export_ok(&r, &res, 3);
}

#[test]
fn failed_part_is_left_out_and_the_rest_exports() {
    let r = recipe(json!([
        part("Nothing", Some(json!({ "sphere": { "center": [0,0,0], "radius": -1 } })), None),
        part("Open", None, Some(json!({ "positions": [[0,0,0],[1,0,0],[0,1,0]], "triangles": [[0,1,2]] }))),
        part("Ball", Some(json!({ "sphere": { "center": [0,0,0], "radius": 5 } })), None),
    ]));
    let res = build_parts(&r, &opts());
    assert_eq!(res[0].status, Status::Failed);
    assert_eq!(res[1].status, Status::Failed);
    assert!(res[1].error.as_deref().unwrap_or("").contains("not closed"), "{:?}", res[1].error);
    assert_eq!(res[2].status, Status::Exact);
    close(&res[2], 4.0 / 3.0 * PI * 125.0, 1e-4);
    export_ok(&r, &res, 1);
}

#[test]
fn over_budget_solid_falls_back_to_mesh() {
    // A many-edge fillet cannot finish in a millisecond.
    let solid = json!({ "fillet": { "body": { "union": [ { "sphere": { "center": [0,0,0], "radius": 10 } },
                                                         { "box": { "min": [-5,-5,0], "max": [5,5,20] } } ] },
                                   "radius": 1, "edges": "all" } });
    let r = recipe(json!([part("Slow", Some(solid), Some(cube_mesh([0.0, 0.0, 0.0], [2.0, 3.0, 4.0])))]));
    let res = build_parts(&r, &Options { timeout: Duration::from_millis(1), ..opts() });
    assert_eq!(res[0].status, Status::Faceted, "{:?}", res[0].error);
    let e = res[0].error.as_deref().unwrap_or("");
    assert!(e.contains("time"), "{e}");
    close(&res[0], 24.0, 1e-9);
}

#[test]
fn all_node_kinds_and_the_assembly_tree() {
    let placement_head = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 5., 40., 60., 1.];
    // 90 degrees about Z, then moved.
    let placement_leg = [0., 1., 0., 0., -1., 0., 0., 0., 0., 0., 1., 0., -25., 0., -10., 1.];
    let r = json!({
        "name": "Rex",
        "bodies": [ { "name": "trunk", "placement": IDENTITY }, { "name": "head", "placement": placement_head }, { "name": "leg", "placement": placement_leg } ],
        "parts": [
            { "name": "Lid", "body": "trunk", "color": [0.2, 0.2, 0.2],
              "solid": { "chamfer": { "body": { "box": { "min": [0,0,0], "max": [10,10,4] } }, "distance": 1, "edges": [[5,0,4]] } } },
            { "name": "Eye", "body": "head", "color": [0.0, 0.0, 0.0],
              "solid": { "intersect": [ { "sphere": { "center": [0,0,0], "radius": 10 } }, { "box": { "min": [0,0,0], "max": [20,20,20] } } ] } },
            { "name": "Thigh", "body": "leg", "color": [0.9, 0.9, 0.8],
              "solid": { "extrude": { "plane": { "origin": [0,0,0], "x": [1,0,0], "y": [0,0,1] },
                                      "points": [[-8,-30],[8,-30],[10,0],[-10,0]], "holes": [[[-3,-6],[3,-6],[3,-3],[-3,-3]]],
                                      "start": -5, "end": 5 } } },
            { "name": "Servo", "body": "leg",
              "solid": { "transform": { "body": { "union": [ { "box": { "min": [0,0,0], "max": [4,4,4] } }, { "box": { "min": [4,0,0], "max": [8,4,4] } } ] },
                                        "matrix": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,-40,1] } } },
            { "name": "Toe", "body": "leg", "mesh": tet_mesh(3.0) }
        ]
    });
    let r = parse_recipe(&r.to_string()).expect("recipe");
    let res = build_parts(&r, &opts());
    let by = |n: &str| res.iter().find(|p| p.name == n).expect("part");
    assert_eq!(by("Lid").status, Status::Exact, "{:?} {:?}", by("Lid").error, by("Lid").warnings);
    close(by("Lid"), 400.0 - 0.5 * 10.0, 1e-6);
    close(by("Eye"), 4.0 / 3.0 * PI * 1000.0 / 8.0, 1e-4);
    close(by("Thigh"), ((16.0 + 20.0) / 2.0 * 30.0 - 18.0) * 10.0, 1e-9);
    close(by("Servo"), 128.0, 1e-9);
    assert_eq!(by("Toe").status, Status::Faceted);

    let text = export_ok(&r, &res, 5);
    let imp = solidcraft_kernel::step_import(&text).expect("import");
    assert_eq!(imp.tree.len(), 1);
    let root = &imp.tree[0];
    assert_eq!(root.name, "Rex");
    let names: Vec<&str> = root.children.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["trunk", "head", "leg"]);
    let head = &root.children[1];
    assert_eq!([head.transform[3][0], head.transform[3][1], head.transform[3][2]], [5.0, 40.0, 60.0]);
    let leg = &root.children[2];
    assert!((leg.transform[0][1] - 1.0).abs() < 1e-9 && (leg.transform[1][0] + 1.0).abs() < 1e-9, "{:?}", leg.transform);
    assert_eq!(leg.bodies.len(), 3);
    let eye = imp.bodies.iter().find(|b| b.name == "Eye").expect("Eye read back");
    assert_eq!(eye.color, Some([0.0, 0.0, 0.0]));
    // The same input gives the same file.
    let again = assemble(&r, &build_parts(&r, &Options { threads: 1, ..opts() }), "test.step").expect("export");
    assert_eq!(again.text, text);
}

#[test]
fn only_selects_parts() {
    let r = recipe(json!([
        part("A", Some(json!({ "box": { "min": [0,0,0], "max": [1,1,1] } })), None),
        part("B", Some(json!({ "box": { "min": [0,0,0], "max": [2,2,2] } })), None),
        part("C", Some(json!({ "box": { "min": [0,0,0], "max": [3,3,3] } })), None),
    ]));
    let res = build_parts(&r, &Options { only: Some(vec!["C".into(), "A".into()]), ..opts() });
    let names: Vec<&str> = res.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["A", "C"]);
    export_ok(&r, &res, 2);
}

#[test]
fn sample_recipe_builds() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/sample.json")).expect("sample");
    let r = parse_recipe(&text).expect("sample parses");
    let res = build_parts(&r, &opts());
    assert!(res.iter().all(|p| p.status != Status::Failed), "{:?}", res.iter().map(|p| (&p.name, &p.error)).collect::<Vec<_>>());
    let solids = res.len();
    export_ok(&r, &res, solids);
}

/// Column-major rotation by `deg` about the axis `(x, y, z)` (unit), then a move.
fn rotation(axis: [f64; 3], deg: f64, t: [f64; 3]) -> Vec<f64> {
    let (s, c) = deg.to_radians().sin_cos();
    let [x, y, z] = axis;
    let k = 1.0 - c;
    vec![
        k * x * x + c, k * x * y + s * z, k * x * z - s * y, 0.0,
        k * x * y - s * z, k * y * y + c, k * y * z + s * x, 0.0,
        k * x * z + s * y, k * y * z - s * x, k * z * z + c, 0.0,
        t[0], t[1], t[2], 1.0,
    ]
}

#[test]
fn transforms_are_pushed_down_and_stay_exact() {
    // A filleted cylinder turned by a rotation: PolyCore's transform of the finished body
    // leaves a surface its STEP writer refuses; built in place it is exact.
    let puck = json!({ "fillet": { "body": { "cylinder": { "base": [0,0,0], "axis": [0,0,1], "radius": 10, "height": 20 } },
                                  "radius": 2, "edges": "all" } });
    let m = rotation([0.6, 0.0, 0.8], 37.0, [5.0, -3.0, 12.0]);
    // A mirrored box with an explicit fillet edge (y = 0 mirror), and a mirrored extrude.
    let mirror = [1., 0., 0., 0., 0., -1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];
    let brick = json!({ "fillet": { "body": { "box": { "min": [0,2,0], "max": [20,12,8] } }, "radius": 1, "edges": [[10, 12, 8]] } });
    let wedge = json!({ "extrude": { "plane": { "origin": [0,0,0], "x": [1,0,0], "y": [0,1,0] }, "points": [[0,0],[10,0],[0,5]], "start": 1, "end": 4 } });
    let r = recipe(json!([
        part("Puck", Some(json!({ "transform": { "body": puck, "matrix": m } })), None),
        part("Brick", Some(json!({ "transform": { "body": brick, "matrix": mirror } })), None),
        part("Wedge", Some(json!({ "transform": { "body": { "transform": { "body": wedge, "matrix": mirror } }, "matrix": m } })), None),
    ]));
    let res = build_parts(&r, &opts());
    for p in &res {
        assert_eq!(p.status, Status::Exact, "{}: {:?} {:?}", p.name, p.error, p.warnings);
    }
    let rho: f64 = 2.0;
    let centroid = 10.0 - rho * (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    close(&res[0], PI * 100.0 * 20.0 - 4.0 * PI * centroid * (1.0 - PI / 4.0) * rho * rho, 1e-4);
    close(&res[1], 1600.0 - (4.0 - PI) / 4.0 * 20.0, 1e-4);
    close(&res[2], 25.0 * 3.0, 1e-9);
    // Built where it ends up: the mirrored brick lies at y < 0.
    let bb = solidcraft_kernel::measure_mass(res[1].solid.as_ref().expect("brick")).expect("measure").bbox;
    assert!(bb.max.y <= -2.0 + 1e-6 && bb.min.y >= -12.0 - 1e-6, "{bb:?}");
    export_ok(&r, &res, 3);
}

#[test]
fn half_space_box_tool_splits_a_symmetric_body() {
    // A loft symmetric about y = 0 halved by a big box (the way the Blender side mirrors).
    let sec = |x: f64, w: f64| json!({ "origin": [x, 0, 0], "x": [0, 1, 0], "y": [0, 0, 1],
                                     "points": [[-w, 0], [w, 0], [w * 0.8, 30], [-w * 0.8, 30]] });
    let body = json!({ "loft": { "sections": [sec(0.0, 10.0), sec(20.0, 15.0), sec(40.0, 8.0)] } });
    let half = json!({ "transform": { "body": { "box": { "min": [-600,-600,-600], "max": [600,0,600] } },
                                      "matrix": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1] } });
    let r = recipe(json!([
        part("Whole", Some(body.clone()), None),
        part("Left", Some(json!({ "cut": [ { "cut": [body.clone(), { "box": { "min": [15,-3,10], "max": [25,3,20] } }] }, half.clone()] })), None),
        part("Right", Some(json!({ "intersect": [ { "cut": [body, { "box": { "min": [15,-3,10], "max": [25,3,20] } }] }, half] })), None),
    ]));
    let res = build_parts(&r, &opts());
    for p in &res {
        assert_eq!(p.status, Status::Exact, "{}: {:?} {:?}", p.name, p.error, p.warnings);
    }
    let whole = res[0].volume_mm3.expect("whole");
    close(&res[1], (whole - 600.0) / 2.0, 1e-4);
    close(&res[2], (whole - 600.0) / 2.0, 1e-4);
    export_ok(&r, &res, 3);
}

#[test]
fn unexportable_body_is_written_as_its_tessellation() {
    use solidcraft_geom::Vec3;
    // Made the way the tool used to (TinyREX part `Compute.csi`): a box with every edge
    // filleted, then PolyCore's transform by a quarter turn about y.
    let bx = solidcraft_kernel::box_solid(Vec3::new(68.9, 26.8, 230.5), Vec3::new(71.4, 32.3, 246.4)).expect("box");
    let edges: Vec<Vec3> = polycore_step::sharp_edges(&bx, 30.0).expect("edges");
    let f = solidcraft_kernel::fillet(&bx, &edges, 0.3).expect("fillet");
    let m = [
        5.205467346058867e-07, 1.5099563199783006e-07, -0.9999967813491821, 0.0, -7.860037490661775e-14, 0.9999988079071045,
        1.5099531935902633e-07, 0.0, 0.9999982118606567, 1.4834876804696692e-15, 5.205477009440074e-07, 0.0, -222.61427307128906,
        2.253768343507545e-06, 8.09980583190918, 1.0,
    ];
    let mut cm = [[0.0; 4]; 4];
    for (i, x) in m.iter().enumerate() {
        cm[i / 4][i % 4] = *x;
    }
    let turned = solidcraft_kernel::transform_matrix(&f, cm).expect("transform");
    let why = polycore_step::export_error(&turned).expect("PolyCore refuses this body; if it no longer does, this fallback is moot");
    assert!(why.contains("STEP export"), "{why}");
    let (b, rep, v) = polycore_step::tessellated_brep(&turned).expect("faceted");
    assert_eq!(rep, "faceted_brep");
    assert!(polycore_step::export_error(&b).is_none());
    let exact = solidcraft_kernel::measure_mass(&f).expect("measure").volume;
    assert!((v - exact).abs() < 2e-3 * exact, "{v} vs {exact}");

    // Through a recipe: a transform that is not rigid (scaled 0.1%) stays a PolyCore transform
    // of the finished body, which the STEP writer refuses; the part is written faceted.
    let puck = json!({ "fillet": { "body": { "box": { "min": [68.9, 26.8, 230.5], "max": [71.4, 32.3, 246.4] } },
                                  "radius": 0.3, "edges": "all", "min_angle_deg": 30 } });
    let mut m2: Vec<f64> = m.to_vec();
    for x in m2.iter_mut().take(12) {
        *x *= 1.001;
    }
    let r = recipe(json!([part("Puck", Some(json!({ "transform": { "body": puck, "matrix": m2 } })), None)]));
    let res = build_parts(&r, &opts());
    let p = &res[0];
    assert_eq!(p.status, Status::Faceted, "{:?}", p.error);
    assert_eq!(p.representation, Some("faceted_brep"));
    assert!(p.warnings.iter().any(|w| w.contains("not exportable")), "{:?}", p.warnings);
    export_ok(&r, &res, 1);
}
