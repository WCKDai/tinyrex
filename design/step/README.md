# TinyREX as STEP

`tinyrex.step` is the whole robot as a STEP AP242 assembly, built by PolyCore:
- 634 solids;
- one product per moving body (19 bodies), placed in the root.

The file is regenerated, not committed, because it is 43 MB. Two ways to make it:
- in Blender, with the PolyCore STEP add-on (`tools/polycore-blender`): File › Export › STEP via PolyCore;
- from the saved recipe:
  `tools/polycore-step/target/release/polycore-step tinyrex.recipe.json tinyrex.step --report tinyrex.report.json --timeout-s 120`.

`tinyrex.report.json` is the last export's report, which is committed.

## Last export

| | |
|---|---|
| Exact B-rep | 604 parts |
| Exact, with warnings | 24 parts. Fillets PolyCore couldn't put on every edge of the bigger lofts at once; it applied them edge by edge or at a smaller radius |
| Faceted | 6 parts: the 3 cables (mesh only); the 2 eye domes, which are scaled spheres PolyCore's STEP writer can't express yet; the left battery holder, where PolyCore's exact boolean gave an implausible volume |
| Failed | 0 |
| Validation | Valid: all 634 solids re-import, none open |
| Time | 112 s on 14 threads |

In Polykrom, insert or open the file. Each body becomes a component, rigid by default; use Unlock Contents to joint
them.
