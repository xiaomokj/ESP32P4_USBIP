# shellcheck shell=bash
#
# Shared bootstrap for the helper scripts.
#
# The EIM-generated activation script bails out with `exit 1` unless it is
# sourced from a shell whose $0 is bash/sh/..., which is never the case when a
# script sources it directly. So we re-exec through `bash -c` (where $0 is
# explicitly set to "bash"), source the activation script there, and hand
# control back to the real script with the environment ready.
#
# It also drops IDF_COMPONENT_LOCAL_STORAGE_URL: EIM points that at a local
# mirror that does not carry the esp_hosted version this project needs.
#
# Usage (top of a script):
#   . "$(dirname "${BASH_SOURCE[0]}")/_idf_env.sh"

IDF_ACTIVATE="${IDF_ACTIVATE:-/root/.espressif/tools/activate_idf_v5.5.5.sh}"

if [ -z "${MK_IDF_ENV_READY:-}" ]; then
    export MK_IDF_ENV_READY=1
    # $0 is "bash" inside the wrapper, $1 is the activation script, and the rest
    # is this script plus its original arguments.
    exec bash -c 'set +e; . "$1" >/dev/null 2>&1; unset IDF_COMPONENT_LOCAL_STORAGE_URL; exec "${@:2}"' \
        bash "${IDF_ACTIVATE}" "${BASH_SOURCE[1]}" "$@"
fi

unset IDF_COMPONENT_LOCAL_STORAGE_URL
