#!/usr/bin/env python3
"""Put the ESP32-C6 co-processor into ROM download mode.

Why esptool cannot do it alone on this board
--------------------------------------------
The C6 USB port exposes a CH340 whose **DTR is wired to the C6 BOOT pin (IO9)**
and nothing is wired to C6 EN/RTS. esptool's reset sequence needs both DTR and
RTS, so `idf.py flash` fails with "No serial data received".

The C6 EN line is driven by the ESP32-P4 on GPIO54 (esp_hosted's slave reset),
so the working sequence is:

    1. hold BOOT low  -> CH340 DTR asserted
    2. reset the C6   -> hard reset the P4, whose GPIO54 pulses the C6 EN pin
    3. release DTR    -> C6 already latched download mode at reset

After this script finishes, flash with `--before no_reset`, e.g.
`python -m esptool --chip esp32c6 -p /dev/ttyUSB0 --before no_reset write_flash ...`
(see tools/c6_flash.sh).

Usage:
    python3 tools/c6_download.py                    # DTR low = BOOT low
    python3 tools/c6_download.py --invert-dtr       # if the board inverts DTR
"""

import argparse
import subprocess
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--c6-port", default="/dev/ttyUSB0", help="CH340 port of the C6")
    ap.add_argument("--p4-port", default="/dev/ttyACM0", help="UART bridge port of the P4")
    ap.add_argument("--invert-dtr", action="store_true",
                    help="assert DTR high instead of low (board with inverting stage)")
    ap.add_argument("--skip-p4-reset", action="store_true",
                    help="do not reset the P4 (use when you power-cycle the board manually)")
    args = ap.parse_args()

    ser = serial.Serial(args.c6_port, 115200, timeout=0.1)
    boot_level = bool(args.invert_dtr)
    ser.dtr = boot_level
    time.sleep(0.2)
    print("DTR set to %s (C6 IO9 held in boot mode)" % boot_level)

    if not args.skip_p4_reset:
        print("resetting ESP32-P4 to pulse the C6 EN line via GPIO54 ...")
        subprocess.run(
            [sys.executable, "-m", "esptool", "--chip", "esp32p4",
             "-p", args.p4_port, "--before", "default_reset", "--after", "hard_reset", "run"],
            check=False,
        )
        time.sleep(2.0)

    ser.dtr = not boot_level
    ser.close()
    print("done: C6 should be sitting in ROM download mode")
    return 0


if __name__ == "__main__":
    sys.exit(main())
