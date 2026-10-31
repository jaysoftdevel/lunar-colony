#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

CONFIG="Release"
TESTS="${GAME_BUILD_TESTS:-OFF}"
CLEAN=0
for arg in "$@"; do
    case "$arg" in
        Debug|debug|Release|release|RelWithDebInfo|relwithdebinfo|reldeb|MinSizeRel|minsizerel|minsize) CONFIG="$(normalize_config "$arg")" ;;
        --tests) TESTS=ON ;;
        --clean) CLEAN=1 ;;
        *) echo "Usage: ./build.sh [Debug|Release|RelWithDebInfo|MinSizeRel] [--tests] [--clean]" >&2; exit 2 ;;
    esac
done

require_tool cmake
if (( CLEAN )); then rm -rf "$GAME_BUILD_DIR"; fi

cmake -S "$GAME_ROOT" -B "$GAME_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$CONFIG" \
    -DGAME_BUILD_NAVIGATION_TESTS="$TESTS"
cmake --build "$GAME_BUILD_DIR" --parallel "$(cpu_jobs)"

echo
echo "Linux build complete: $GAME_BIN"
echo "Backend: X11 + OpenGL"
echo "Run: ./run.sh"
