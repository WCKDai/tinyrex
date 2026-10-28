"""Draw labelled.py's callouts on its renders: labels down both margins, a leader to each part.

    python annotate.py OUT_DIR/labelled-<view>.json [...]   (needs Pillow)"""
import json
import sys

from PIL import Image, ImageDraw, ImageFont


def font(size):
    for f in ("/System/Library/Fonts/SFNS.ttf", "/System/Library/Fonts/Helvetica.ttc", "/Library/Fonts/Arial.ttf"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


for spec in sys.argv[1:]:
    data = json.load(open(spec))
    img = Image.open(data["image"]).convert("RGB")
    W, H = img.size
    pad = 520
    canvas = Image.new("RGB", (W + 2 * pad, H), (246, 246, 244))
    canvas.paste(img, (pad, 0))
    d = ImageDraw.Draw(canvas)
    f = font(26)
    labels = [l for l in data["labels"] if 0 < l["x"] < 1 and 0 < l["y"] < 1]
    # (one entry per text: the nearest instance)
    seen = {}
    for l in sorted(labels, key=lambda l: l["depth"]):
        seen.setdefault(l["text"], l)
    labels = list(seen.values())
    left = sorted([l for l in labels if l["x"] < 0.5], key=lambda l: l["y"])
    right = sorted([l for l in labels if l["x"] >= 0.5], key=lambda l: l["y"])
    for side, group in (("L", left), ("R", right)):
        n = len(group)
        for i, l in enumerate(group):
            ty = int(H * (i + 0.5) / max(n, 1))
            ax, ay = pad + l["x"] * W, l["y"] * H
            if side == "L":
                tx = 20
                d.text((tx, ty - 15), l["text"], fill=(30, 30, 30), font=f)
                end = tx + d.textlength(l["text"], font=f) + 8
            else:
                tw = d.textlength(l["text"], font=f)
                tx = W + 2 * pad - 20 - tw
                d.text((tx, ty - 15), l["text"], fill=(30, 30, 30), font=f)
                end = tx - 8
            d.line([(end, ty), (pad if side == "L" else pad + W, ty), (ax, ay)], fill=(110, 110, 110), width=2)
            d.ellipse([ax - 5, ay - 5, ax + 5, ay + 5], fill=(220, 60, 40))
    out = data["image"].replace(".png", "-annotated.png")
    canvas.save(out)
    print(out)
