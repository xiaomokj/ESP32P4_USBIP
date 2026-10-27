#!/usr/bin/env python3
"""Fetch the official Mklink-AI-Probe launcher icon.

The upstream Tauri GUI ships its icon set in
`Mklink-AI-Probe/gui/src-tauri/icons`. We only need one raster size (128x128) and
downscale it to the 112x112 LVGL image ourselves, so the result stays crisp and
keeps the same encoder as every other asset in this project.

Usage:
    python3 tools/fetch_icon.py            # download if not cached
    python3 tools/fetch_icon.py --force    # re-download
"""

import os
import sys
import urllib.request

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSET_DIR = os.path.join(PROJECT_DIR, "tools", "assets")
DEST = os.path.join(ASSET_DIR, "icon_128.png")

URLS = (
    "https://raw.githubusercontent.com/Aladdin-Wang/Mklink-AI-Probe/master/"
    "Mklink-AI-Probe/gui/src-tauri/icons/128x128.png",
    # fork upstream in case the author renames the default branch
    "https://raw.githubusercontent.com/Aladdin-Wang/Mklink-AI-Probe/main/"
    "Mklink-AI-Probe/gui/src-tauri/icons/128x128.png",
)


def main(argv):
    force = "--force" in argv

    if os.path.exists(DEST) and not force:
        print(DEST)
        return 0

    os.makedirs(ASSET_DIR, exist_ok=True)
    last_error = None
    for url in URLS:
        try:
            with urllib.request.urlopen(url, timeout=20) as resp:
                data = resp.read()
            if len(data) < 256:
                raise ValueError("suspiciously small response (%d bytes)" % len(data))
            with open(DEST, "wb") as fh:
                fh.write(data)
            print(DEST)
            return 0
        except Exception as exc:  # noqa: BLE001 - report the last failure
            last_error = exc

    sys.exit("failed to download icon: %s" % last_error)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
