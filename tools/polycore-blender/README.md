# PolyCore STEP for Blender

A Blender add-on that exports the scene, or the selection, as a STEP assembly built by
[PolyCore](https://github.com/WCKDai/polycore), the kernel under Polykrom.

## Installing

1. **Build the tool.** In `tools/polycore-step`, run `cargo build --release`. This needs the PolyCore checkout next to
   this repository (`../polycore`); the binary is `tools/polycore-step/target/release/polycore-step`.
2. **Package the add-on.** Run `zip -r polycore_step.zip polycore_step` in this directory.
3. **Install it.** In Blender, open Edit › Preferences › Get Extensions › ⌄ › Install from Disk and pick the zip.
4. **Point it at the tool.** In the add-on's preferences, set the path to `polycore-step`.

## Using it

There are two ways to export:
- **File menu:** File › Export › STEP via PolyCore.
- **Sidebar:** in the 3D view, open the sidebar (N) › PolyCore › Export scene / Export selection.

Each export writes three files:
- `NAME.step`: the assembly.
- `NAME.recipe.json`: what was sent to PolyCore. You can re-run `polycore-step` on it by hand.
- `NAME.report.json`: per-part results. The same results go to Blender's text editor as **PolyCore report**, and
  each object gets a `pc_status` property.

## What comes out exact

A part is exported as exact B-rep when its mesh data carries a construction recipe (`pc_recipe`). The scripts that
build the model write these recipes for:
- boxes and cylinders;
- ruled lofts through planar sections, such as the shells, panels and plates.

The add-on reads the object's modifiers on top of the recipe:

| Modifier | Exported as |
|---|---|
| Bevel | Fillet of every sharp edge, at the bevel width |
| Solidify (inward) | A hollow: an inset copy of the loft, cut out |
| Boolean | Cut, intersect or union with the cutter object's own recipe |
| Subdivision | Not reproduced: the part exports as the ruled loft under it, and the report says so |

Anything else is exported as its closed, evaluated mesh, and comes back as a faceted solid. This includes spheres
scaled into ellipsoids, cones, cables and domes. A part whose exact build fails in PolyCore, or runs out of time,
falls back the same way. The report gives each part's status: `exact`, `exact_with_warnings` (for example a fillet
reduced to fit), `faceted` or `failed`.

## Assembly

Parts are grouped by the body they move with. An object's `body` property and the matching `B:<body>` empty give the
body and its frame; objects without one go in a single `root` body. In the STEP, each body is a product placed in the
assembly, holding its parts in its own frame.

Polykrom (`docs/step-import.md`) imports the product tree as nested components, one per body,
placed where the file puts them. They're rigid by default; to joint them, use **Unlock Contents**
on the import's component (`component.unlock`).

PolyCore is still improving. As its fillets, booleans and sweeps get more capable, more parts come out exact without
any change here.
