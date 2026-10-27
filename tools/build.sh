#!/usr/bin/env bash
#
# Build / flash helper for MKLink Remote.
#
# The EIM installation pins a *local* component storage mirror that only carries
# a couple of esp_hosted versions, so it has to be unset before the component
# manager runs. Keep both steps in one place to avoid surprises.
#
# Usage:
#   tools/build.sh                 # build
#   tools/build.sh flash [PORT]    # build + flash (default /dev/ttyACM0)
#   tools/build.sh monitor [PORT]  # reset + monitor
#   tools/build.sh fullflash [PORT]# erase + full flash (after partition changes)
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_ACTIVATE="${IDF_ACTIVATE:-/root/.espressif/tools/activate_idf_v5.5.5.sh}"
PORT="${PORT:-/dev/ttyACM0}"
ACTION="${1:-build}"

# shellcheck source=/dev/null
set +eu
source "${IDF_ACTIVATE}" >/dev/null 2>&1 || true
set -eu
unset IDF_COMPONENT_LOCAL_STORAGE_URL

cd "${PROJECT_DIR}"

idf() { python "${IDF_PATH}/tools/idf.py" "$@"; }

case "${ACTION}" in
    build)
        idf build
        ;;
    flash)
        PORT="${2:-${PORT}}"
        idf -p "${PORT}" -b 2000000 build flash
        ;;
    fullflash)
        PORT="${2:-${PORT}}"
        idf -p "${PORT}" -b 2000000 erase-flash
        idf -p "${PORT}" -b 2000000 build flash
        ;;
    monitor)
        PORT="${2:-${PORT}}"
        printf 'reset\n' | idf -p "${PORT}" -b 921600 monitor
        ;;
    *)
        echo "unknown action: ${ACTION}" >&2
        exit 1
        ;;
esac
