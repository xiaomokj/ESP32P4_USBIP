#!/usr/bin/env bash
#
# Build the ESP32-C6 esp_hosted slave firmware from the esp_hosted component.
#
# The component ships the slave project; it must be built with the *same*
# esp_hosted version as the host side (2.12.13 here). A version mismatch shows up
# at runtime as "Version mismatch: Host [x] > Co-proc [y]" followed by
# "Unrecoverable host sdio state" as soon as WiFi carries real traffic.
#
# Output: build/c6/slave/build/network_adapter.bin  (flash with tools/c6_flash.sh)
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_ACTIVATE="${IDF_ACTIVATE:-/root/.espressif/tools/activate_idf_v5.5.5.sh}"
HOSTED_DIR="${PROJECT_DIR}/managed_components/espressif__esp_hosted"
SW_DIR="${PROJECT_DIR}/build/c6"

# shellcheck source=/dev/null
set +eu
source "${IDF_ACTIVATE}" >/dev/null 2>&1 || true
set -eu
unset IDF_COMPONENT_LOCAL_STORAGE_URL

[ -d "${HOSTED_DIR}/slave" ] || { echo "esp_hosted component missing: ${HOSTED_DIR}" >&2; exit 1; }

mkdir -p "${SW_DIR}"
rm -rf "${SW_DIR}/slave" "${SW_DIR}/common"
cp -r "${HOSTED_DIR}/slave" "${SW_DIR}/slave"
cp -r "${HOSTED_DIR}/common" "${SW_DIR}/common"

cd "${SW_DIR}/slave"
idf() { python "${IDF_PATH}/tools/idf.py" "$@"; }
idf set-target esp32c6
idf build

echo
echo "firmware: ${SW_DIR}/slave/build/network_adapter.bin"
echo "next: python3 tools/c6_download.py && tools/c6_flash.sh"
