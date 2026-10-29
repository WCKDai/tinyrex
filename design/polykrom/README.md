# TinyREX in Polykrom

Polykrom opens the STEP assembly (`../step/tinyrex.step`) in about 3 s:
- 668 bodies, 1.4 M triangles;
- one component per moving body, rigid until you use Unlock Contents.

The design, its 3MF and the print kit are generated, not committed. They are large and rebuild in seconds.

```sh
PK=~/wckdboy/solidcraft-wt/target/release            # a Polykrom build (polykrom, polykrom-cli)
$PK/polykrom-cli bench-open ../step/tinyrex.step --save tinyrex.polykrom       # STEP -> Polykrom design
$PK/polykrom-cli snapshot --in tinyrex.polykrom --out polykrom-iso.png --view iso
$PK/polykrom tinyrex.polykrom                                                   # open it in the app
$PK/polykrom-cli run --in tinyrex.polykrom --out tinyrex.3mf                    # every part, named
python ../blender/print_kit.py tinyrex.3mf <audit.json> print                  # the print kit
```

## Print kit

`print/` has one binary STL for each of the 111 printed parts, PLA and TPU. Each is checked watertight: every
edge is shared by exactly two consistently wound triangles. `print/kit.json` records the check.

108 of the parts come from PolyCore's exact solids, through Polykrom's 3MF:
- coincident seam vertices are welded;
- triangle winding is made consistent;
- the odd small gap is capped.

The tongue and the two jaw arms come from their Blender meshes, which are closed. PolyCore's tessellation of their
fillets is non-manifold.

This replaces printing from the Blender meshes, where six boolean-cut shells (the body halves, both head shells,
the head back cap and the visor) have open seams.
