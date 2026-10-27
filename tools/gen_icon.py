#!/usr/bin/env python3
"""Generate the LVGL 8 launcher icon for MkLinkApp.

What the shell's icons actually look like
-----------------------------------------
Measured from the shipped assets (decoded with the correct 3 B/px interleaved
layout, see below):

    * 112x112 canvas
    * full-bleed rounded square plate, corner radius ~31 px  -> ~90% opaque
    * brand-coloured plate (setting #60A4F8, calculator blue, drawpanel grey)
    * white glyph on top (#FFFFFF)

So the icon is a plate + white glyph, **not** a transparent line drawing. Our
glyph here is the device itself: USB-C plug + probe body with an LED + two
wireless arcs.

Pixel layout (this is the part that is easy to get wrong)
--------------------------------------------------------
With LV_COLOR_DEPTH == 16, LVGL 8 stores LV_IMG_CF_TRUE_COLOR_ALPHA as 3 bytes
per pixel, **interleaved**:

    [RGB565 low][RGB565 high][alpha]   repeated

Not "colour plane then alpha plane" - writing it that way renders as noise even
though data_size is right. lv_img_buf.c proves it:

    uint32_t px = w * y * px_size + x * px_size;   // px_size == 3
    return buf_u8[px + px_size - 1];               // alpha = 3rd byte

Requirements:
    pip install --break-system-packages pillow

Usage:
    python3 tools/gen_icon.py components/apps/mklink/assets/img_app_mklink.c
    python3 tools/gen_icon.py --style line  ...        # transparent line-art variant
    python3 tools/gen_icon.py --inspect <icon.c>       # ASCII preview / layout check
"""

import os
import re
import sys

try:
    from PIL import Image, ImageDraw
except ImportError:  # pragma: no cover - environment hint
    sys.exit("Pillow missing: pip install --break-system-packages pillow")

# ---------------------------------------------------------------- design tokens
SIZE = 112              # must match the other launcher icons
SS = 4                  # supersampling factor for the glyph
BPP = 3                 # RGB565 + A8, interleaved (LV_COLOR_DEPTH == 16)
PLATE_RADIUS = 31       # measured from the shipped icons
GLYPH_SCALE = 0.66      # glyph bbox as a fraction of the canvas

PLATE_FROM = (0x22, 0xD3, 0xEE)    # cyan
PLATE_TO = (0x3B, 0x82, 0xF6)      # blue
WHITE = (0xFF, 0xFF, 0xFF, 255)

STROKE = 7              # glyph outline width, final pixels

# glyph geometry on the 112x112 canvas (before scaling/centering)
PLUG_BOX = (8, 42, 42, 58)          # USB-C plug outline
PLUG_RADIUS = 6
PLUG_PINS = ((16, 46, 20, 54), (25, 46, 29, 54))   # two contact lines
BODY_BOX = (38, 20, 86, 92)         # probe body outline
BODY_RADIUS = 13
LED_CENTER = (62, 40)
LED_RADIUS = 5
ARC_CENTER = (90, 32)               # waves to the right of the body
ARC_RADII = (12, 22)
ARC_START, ARC_END = -70, 55


def _s(c):
    """Scale a coordinate to the supersampled canvas."""
    if isinstance(c, (tuple, list)):
        return tuple(int(round(v * SS)) for v in c)
    return int(round(c * SS))


def render_glyph(color=WHITE):
    """Draw the probe glyph on a transparent canvas, then normalise its size."""
    layer = Image.new("RGBA", (SIZE * SS, SIZE * SS), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    stroke = STROKE * SS

    draw.rounded_rectangle(_s(BODY_BOX), radius=BODY_RADIUS * SS, outline=color, width=stroke)

    cx, cy = LED_CENTER
    draw.ellipse([_s(cx - LED_RADIUS), _s(cy - LED_RADIUS),
                  _s(cx + LED_RADIUS), _s(cy + LED_RADIUS)], fill=color)

    draw.rounded_rectangle(_s(PLUG_BOX), radius=PLUG_RADIUS * SS, outline=color, width=stroke)
    for pin in PLUG_PINS:
        draw.rectangle(_s(pin), fill=color)

    import math

    for radius in ARC_RADII:
        draw.arc([_s(ARC_CENTER[0] - radius), _s(ARC_CENTER[1] - radius),
                  _s(ARC_CENTER[0] + radius), _s(ARC_CENTER[1] + radius)],
                 start=ARC_START, end=ARC_END, fill=color, width=stroke)
        for angle in (ARC_START, ARC_END):
            rad = math.radians(angle)
            ex = ARC_CENTER[0] + radius * math.cos(rad)
            ey = ARC_CENTER[1] + radius * math.sin(rad)
            cap = STROKE / 2.0
            draw.ellipse([_s(ex - cap), _s(ey - cap), _s(ex + cap), _s(ey + cap)], fill=color)

    layer = layer.resize((SIZE, SIZE), Image.LANCZOS)

    # normalise: crop to the glyph bbox, scale to GLYPH_SCALE, centre it
    bbox = layer.getbbox()
    if bbox is None:
        sys.exit("glyph rendering produced an empty image")
    glyph = layer.crop(bbox)
    span = int(round(SIZE * GLYPH_SCALE))
    scale = span / max(glyph.width, glyph.height)
    glyph = glyph.resize((max(1, int(glyph.width * scale)),
                          max(1, int(glyph.height * scale))), Image.LANCZOS)

    out = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))
    out.paste(glyph, ((SIZE - glyph.width) // 2, (SIZE - glyph.height) // 2), glyph)
    return out


def render_plate():
    """Full-bleed rounded square with a diagonal cyan->blue gradient."""
    grad = Image.new("RGB", (SIZE, SIZE))
    denom = 2.0 * (SIZE - 1)
    for y in range(SIZE):
        for x in range(SIZE):
            t = (x + y) / denom
            grad.putpixel((x, y), tuple(int(round(PLATE_FROM[i] + (PLATE_TO[i] - PLATE_FROM[i]) * t))
                                        for i in range(3)))

    mask = Image.new("L", (SIZE, SIZE), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, SIZE - 1, SIZE - 1], radius=PLATE_RADIUS, fill=255)

    plate = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))
    plate.paste(grad, (0, 0), mask)
    return plate


def render(style="plate"):
    if style == "line":
        return render_glyph()
    plate = render_plate()
    plate.alpha_composite(render_glyph())
    return plate


def render_from_image(path):
    """Use an external PNG (e.g. the official Mklink-AI-Probe icon)."""
    img = Image.open(path).convert("RGBA")

    # centre-crop to a square so the aspect ratio is never distorted
    side = min(img.width, img.height)
    img = img.crop(((img.width - side) // 2, (img.height - side) // 2,
                    (img.width + side) // 2, (img.height + side) // 2))

    return img.resize((SIZE, SIZE), Image.LANCZOS)


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def rgb565_to_rgb(v):
    return (((v >> 11) & 0x1F) << 3, ((v >> 5) & 0x3F) << 2, (v & 0x1F) << 3)


def encode_lvgl(img):
    """Interleaved [RGB565 LE][alpha] per pixel - see the module docstring."""
    out = bytearray()
    for y in range(SIZE):
        for x in range(SIZE):
            r, g, b, a = img.getpixel((x, y))
            c = rgb565(r, g, b)
            out += bytes((c & 0xFF, (c >> 8) & 0xFF, a))
    return bytes(out)


def decode_pixels(data):
    for i in range(len(data) // BPP):
        c = data[i * BPP] | (data[i * BPP + 1] << 8)
        r, g, b = rgb565_to_rgb(c)
        yield r, g, b, data[i * BPP + 2]


def verify(data, style="plate"):
    """Catch layout/geometry regressions before they reach the panel."""
    problems = []

    if len(data) != SIZE * SIZE * BPP:
        problems.append("data_size %d != %d (w*h*%d)" % (len(data), SIZE * SIZE * BPP, BPP))

    pixels = list(decode_pixels(data))
    if len(pixels) < SIZE * SIZE:
        problems.append("decoded %d pixels, expected %d" % (len(pixels), SIZE * SIZE))
    else:
        corners = [pixels[0][3], pixels[SIZE - 1][3],
                   pixels[(SIZE - 1) * SIZE][3], pixels[SIZE * SIZE - 1][3]]
        if any(c != 0 for c in corners):
            problems.append("corners are not transparent: %s" % corners)

        ratio = 100.0 * sum(1 for p in pixels if p[3] > 180) / (SIZE * SIZE)
        if style == "plate":
            low, high = 85.0, 96.0
        elif style == "line":
            low, high = 10.0, 45.0
        else:  # image: anything visible is fine, only guard against garbage
            low, high = 5.0, 100.0
        if not low <= ratio <= high:
            problems.append("opaque ratio %.1f%% outside %.0f%%..%.0f%%" % (ratio, low, high))

        if style == "plate":
            white = sum(1 for p in pixels if p[3] > 200 and min(p[0], p[1], p[2]) > 200)
            if white < SIZE * SIZE * 0.02:
                problems.append("plate style needs a visible white glyph (%d white px)" % white)
            # the plate must stay in the cyan/blue family
            px = pixels[(SIZE // 2) * SIZE + SIZE // 2]
            if not (px[2] > 0x80 and px[2] >= px[0]):
                problems.append("plate centre colour #%02X%02X%02X is not cyan/blue" % px[:3])

    if problems:
        sys.exit("icon self-check FAILED:\n  - " + "\n  - ".join(problems))


def write_c(path, data):
    lines = [
        "/* Generated by tools/gen_icon.py - do not edit manually.",
        " * LVGL 8 LV_IMG_CF_TRUE_COLOR_ALPHA @ LV_COLOR_DEPTH=16:",
        " * each pixel is [RGB565 low][RGB565 high][alpha] (interleaved, 3 B/px). */",
        '#include "lvgl.h"',
        "",
        "#ifndef LV_ATTRIBUTE_MEM_ALIGN",
        "#define LV_ATTRIBUTE_MEM_ALIGN",
        "#endif",
        "",
        "static const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST uint8_t img_app_mklink_map[] = {",
    ]
    for i in range(0, len(data), 16):
        lines.append("    " + ", ".join("0x%02x" % b for b in data[i:i + 16]) + ",")
    lines += [
        "};",
        "",
        "const lv_img_dsc_t img_app_mklink = {",
        "    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,",
        "    .header.always_zero = 0,",
        "    .header.reserved = 0,",
        "    .header.w = %d," % SIZE,
        "    .header.h = %d," % SIZE,
        "    .data_size = %d," % len(data),
        "    .data = img_app_mklink_map,",
        "};",
        "",
    ]
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def _active_16bit_bytes(body):
    """Extract the 16-bit RGB565A8 branch from an LVGL image-converter .c file.

    Converter output carries one array per colour depth behind `#if
    LV_COLOR_DEPTH == ...`; summing them all yields nonsense. This project runs
    LV_COLOR_DEPTH=16 with LV_COLOR_16_SWAP=0.
    """
    keep = []
    stack = []
    for line in body.splitlines():
        stripped = line.strip()
        if stripped.startswith("#if"):
            stack.append(("LV_COLOR_DEPTH == 16" in stripped) and ("!= 0" not in stripped))
            continue
        if stripped.startswith("#elif") or stripped.startswith("#else"):
            if stack:
                stack[-1] = False
            continue
        if stripped.startswith("#endif"):
            if stack:
                stack.pop()
            continue
        if stack and not all(stack):
            continue
        keep.append(line)
    return bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", "\n".join(keep)))


def inspect(path, step=4):
    """Decode an existing icon .c and print an ASCII preview (layout regression check)."""
    src = open(path, encoding="utf-8", errors="ignore").read()
    w = int(re.search(r"\.header\.w = (\d+)", src).group(1))
    h = int(re.search(r"\.header\.h = (\d+)", src).group(1))
    body = re.search(r"=\s*\{(.*?)\};", src, re.S).group(1)
    data = _active_16bit_bytes(body)

    expected = w * h * BPP
    print("%s: %dx%d, 16-bit section=%d bytes (interleaved %d B/px -> expected %d)"
          % (os.path.basename(path), w, h, len(data), BPP, expected))
    if len(data) < expected:
        print("  ! data too small for an interleaved TRUE_COLOR_ALPHA image")
        return 1

    for y in range(0, h, step * 2):
        row = ""
        for x in range(0, w, step):
            a = data[(y * w + x) * BPP + 2]
            row += " " if a < 40 else ("." if a < 110 else ("+" if a < 200 else "#"))
        print("  " + row)
    return 0


def main():
    args = sys.argv[1:]
    if args and args[0] == "--inspect":
        if len(args) != 2:
            print(__doc__)
            return 1
        return inspect(args[1])

    style = "plate"
    if args and args[0] == "--style":
        if len(args) < 3 or args[1] not in ("plate", "line"):
            print(__doc__)
            return 1
        style = args[1]
        args = args[2:]

    source = None
    if args and args[0] == "--from":
        if len(args) < 3:
            print(__doc__)
            return 1
        source = args[1]
        args = args[2:]
        style = "image"

    if len(args) != 1:
        print(__doc__)
        return 1

    out_c = args[0]
    img = render_from_image(source) if source is not None else render(style)
    data = encode_lvgl(img)
    verify(data, style)
    write_c(out_c, data)

    preview_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools", "preview")
    os.makedirs(preview_dir, exist_ok=True)
    preview = os.path.join(preview_dir, "img_app_mklink_112.png")
    img.save(preview)

    pixels = list(decode_pixels(data))
    opaque = sum(1 for p in pixels if p[3] > 200)
    print("wrote %s (%d bytes, %dx%d, %d B/px, style=%s, opaque %.0f%%)" %
          (out_c, len(data), SIZE, SIZE, BPP, style, 100.0 * opaque / (SIZE * SIZE)))
    print("preview %s" % preview)
    return 0


if __name__ == "__main__":
    sys.exit(main())
