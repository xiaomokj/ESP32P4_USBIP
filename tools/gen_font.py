#!/usr/bin/env python3
"""Generate the Chinese font used by MkLinkApp.

Why this exists
---------------
LVGL 8 ships `lv_font_simsun_16_cjk`, but that font only contains ~1400 CJK
glyphs (the most common ones). Characters like 连 / 探 / 针 / 配 / 网 are missing,
so labels rendered with it look like mojibake / empty boxes.

This script builds a subset font from Noto Sans CJK SC instead:

  * every non-ASCII character that appears in the MKLink sources, plus
  * GB2312 level-1 (3755 common simplified characters) so scanned SSIDs and
    future log strings still render.

Requirements (host side only, not needed to build the firmware):
    pip install --break-system-packages fonttools
    npm install lv_font_conv            # or a global install

Usage:
    python3 tools/gen_font.py           # regenerate the C font
    python3 tools/gen_font.py --check   # only report missing glyphs of sources

The generated file components/apps/mklink/assets/mklink_font_cjk_16.c exports
`mklink_font_cjk_16` (LVGL 8, 16 px, 4 bpp, no compression).
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_C = os.path.join(ROOT, "components/apps/mklink/assets/mklink_font_cjk_16.c")
NOTO_TTC = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
NOTO_OTF = "/tmp/NotoSansCJKsc-Regular.otf"
SOURCE_GLOBS = [
    "components/apps/mklink/*.cpp",
    "components/apps/mklink/*.hpp",
    "components/mklink_core/src/*.c",
    "components/mklink_core/include/*.h",
]


def collect_chars():
    chars = set()
    for pattern in SOURCE_GLOBS:
        for path in glob.glob(os.path.join(ROOT, pattern)):
            with open(path, encoding="utf-8") as fh:
                for ch in fh.read():
                    if ord(ch) > 0x7F:
                        chars.add(ch)
    return chars


def gb2312_symbols():
    """GB2312 rows 0xA1..0xA9: punctuation, full width forms, box drawing, etc.

    These are the characters that show up as empty boxes when only the level-1
    ideograph block is generated (e.g. the full width colon in a hint label).
    """
    chars = set()
    for hi in range(0xA1, 0xAA):
        for lo in range(0xA1, 0xFF):
            try:
                ch = bytes((hi, lo)).decode("gb2312")
            except UnicodeDecodeError:
                continue
            if ch.strip():
                chars.add(ch)
    return chars


def gb2312_level1():
    chars = set()
    for hi in range(0xB0, 0xD8):
        for lo in range(0xA1, 0xFF):
            try:
                ch = bytes((hi, lo)).decode("gb2312")
            except UnicodeDecodeError:
                continue
            if ord(ch) > 0x7F:
                chars.add(ch)
    return chars


def ensure_otf():
    if os.path.exists(NOTO_OTF):
        return NOTO_OTF
    try:
        from fontTools.ttLib import TTCollection
    except ImportError:
        sys.exit("fonttools missing: pip install --break-system-packages fonttools")
    collection = TTCollection(NOTO_TTC)
    for font in collection.fonts:
        family = font["name"].getDebugName(1) or ""
        if "SC" in family or "Simplified" in family:
            font.save(NOTO_OTF)
            print("extracted %r -> %s" % (family, NOTO_OTF))
            return NOTO_OTF
    sys.exit("no Simplified Chinese face found in %s" % NOTO_TTC)


def find_lv_font_conv():
    # A Windows npm install can shadow the Linux one via PATH (/mnt/c/...), and
    # that shim is not runnable from WSL - so probe each candidate for real.
    candidates = [
        os.environ.get("LV_FONT_CONV"),
        os.path.join(ROOT, "tools/node_modules/.bin/lv_font_conv"),
        "/tmp/lvfc/node_modules/.bin/lv_font_conv",
        shutil.which("lv_font_conv"),
    ]
    for candidate in candidates:
        if not candidate or not os.path.exists(candidate):
            continue
        probe = subprocess.run([candidate, "--version"], capture_output=True, text=True)
        if probe.returncode == 0:
            return candidate
    sys.exit("lv_font_conv not found (npm install lv_font_conv)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="only list source characters")
    args = parser.parse_args()

    source_chars = collect_chars()
    print("characters used by sources: %d" % len(source_chars))
    if args.check:
        print("".join(sorted(source_chars)))
        return 0

    symbols = sorted(source_chars | gb2312_symbols() | gb2312_level1())
    symbols = "".join(ch for ch in symbols if ch not in "\n\r\t")
    print("total glyphs to render: %d" % len(symbols))

    cmd = [
        find_lv_font_conv(),
        "--font", ensure_otf(),
        "--size", "16",
        "--bpp", "4",
        "--format", "lvgl",
        "--no-compress",
        "--lv-include", "lvgl.h",
        "--lv-font-name", "mklink_font_cjk_16",
        "-r", "0x20-0x7E",
        "--symbols", symbols,
        "-o", OUT_C,
    ]
    subprocess.run(cmd, check=True)
    print("wrote %s (%d bytes)" % (OUT_C, os.path.getsize(OUT_C)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
