#!/usr/bin/env bash
set -euo pipefail

GAME_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GAME_BUILD_DIR="$GAME_ROOT/build/linux"
GAME_BIN="$GAME_BUILD_DIR/bin/game"
GAME_DEBUG_BIN="$GAME_BUILD_DIR/bin/game-debug"

normalize_config() {
    case "${1:-Release}" in
        Debug|debug) echo Debug ;;
        Release|release) echo Release ;;
        RelWithDebInfo|relwithdebinfo|reldeb) echo RelWithDebInfo ;;
        MinSizeRel|minsizerel|minsize) echo MinSizeRel ;;
        *) echo "Unsupported configuration: $1" >&2; return 2 ;;
    esac
}

build_variants() {
    if [[ "$1" == Debug ]]; then
        printf '%s\n' Debug Release
    else
        printf '%s\n' "$1" Debug
    fi
}

configure_variant() {
    cmake -S "$GAME_ROOT" -B "$GAME_BUILD_DIR/$1" \
        -DCMAKE_BUILD_TYPE="$1" -DGAME_BIN_DIR="$GAME_BUILD_DIR/bin" \
        -DGAME_BUILD_NAVIGATION_TESTS="$2"
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
    echo "$n"
}

require_tool() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Required tool '$1' was not found in PATH." >&2
        echo "Run ./install-deps.sh or install the dependency manually." >&2
        exit 127
    }
}
