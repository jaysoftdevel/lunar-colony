#!/usr/bin/env bash
set -euo pipefail

GAME_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GAME_BUILD_DIR="$GAME_ROOT/build/linux"
GAME_BIN="$GAME_BUILD_DIR/bin/game"

normalize_config() {
    case "${1:-Release}" in
        Release|release) echo Release ;;
        RelWithDebInfo|relwithdebinfo|reldeb) echo RelWithDebInfo ;;
        MinSizeRel|minsizerel|minsize) echo MinSizeRel ;;
        *) echo "Unsupported configuration: $1" >&2; return 2 ;;
    esac
}

configure_variant() {
    local engine_flags=()
    if [[ -n "${JSD_GAME_ENGINE_DIR:-}" ]]; then engine_flags=("-DJSD_GAME_ENGINE_DIR=$JSD_GAME_ENGINE_DIR"); fi
    cmake -S "$GAME_ROOT" -B "$GAME_BUILD_DIR/$1" "${engine_flags[@]}" \
        -DCMAKE_BUILD_TYPE="$1" -DGAME_BIN_DIR="$GAME_BUILD_DIR/bin" \
        -DGAME_BUILD_NAVIGATION_TESTS="$2" -DGAME_DEBUG_FEATURES="$3"
}

cpu_jobs() {
    if [[ -n "${GAME_BUILD_JOBS:-}" ]]; then
        echo "$GAME_BUILD_JOBS"
        return
    fi
    local n
    n="$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || echo 2)"
    [[ "$n" =~ ^[0-9]+$ ]] || n=2
    (( n < 1 )) && n=1
    # Bound concurrent C++ compiler memory; GAME_BUILD_JOBS explicitly overrides it.
    local mem_kb mem_jobs
    mem_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
    if [[ "$mem_kb" =~ ^[0-9]+$ ]] && ((mem_kb>0)); then
        mem_jobs=$((mem_kb/1000000)); ((mem_jobs<1)) && mem_jobs=1
        ((n>mem_jobs)) && n=$mem_jobs
    fi
    echo "$n"
}

require_tool() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Required tool '$1' was not found in PATH." >&2
        echo "Run ./install-deps.sh or install the dependency manually." >&2
        exit 127
    }
}
