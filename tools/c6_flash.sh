#!/usr/bin/env bash
#
# Flash the ESP32-C6 esp_hosted slave firmware.
#
# The firmware is built from the esp_hosted component's own `slave` project
# (version matched with the host side, otherwise WiFi dies with
# "Unrecoverable host sdio state"). Build it with tools/c6_build.sh first.
#
# This board only wires the CH340 DTR line to the C6 BOOT pin (no EN/RTS), so
# esptool cannot reset the chip by itself: put the C6 into download mode first
# (tools/c6_download.py, or hold IO9 low and reset the board), then run this.
#
# Usage:
#   tools/c6_flash.sh [PORT]        # default /dev/ttyUSB0
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-/dev/ttyUSB0}"

. "${PROJECT_DIR}/tools/_idf_env.sh"

SLAVE_DIR="${SLAVE_DIR:-${PROJECT_DIR}/build/c6/slave}"
if [ ! -f "${SLAVE_DIR}/build/network_adapter.bin" ]; then
    echo "C6 slave firmware not built yet, run tools/c6_build.sh" >&2
    exit 1
fi

python -m esptool --chip esp32c6 -p "${PORT}" -b 460800 \
    --before no_reset --after hard_reset \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB \
    0x0 "${SLAVE_DIR}/build/bootloader/bootloader.bin" \
    0x8000 "${SLAVE_DIR}/build/partition_table/partition-table.bin" \
    0xd000 "${SLAVE_DIR}/build/ota_data_initial.bin" \
    0x10000 "${SLAVE_DIR}/build/network_adapter.bin"
