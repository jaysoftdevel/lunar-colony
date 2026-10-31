#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

CONFIG="Release"
TESTS="${GAME_BUILD_TESTS:-OFF}"
for arg in "$@"; do
    case "$arg" in
        Debug|debug|Release|release|RelWithDebInfo|relwithdebinfo|reldeb|MinSizeRel|minsizerel|minsize) CONFIG="$(normalize_config "$arg")" ;;
        --tests) TESTS=ON ;;
        *) echo "Usage: ./configure.sh [Debug|Release|RelWithDebInfo|MinSizeRel] [--tests]" >&2; exit 2 ;;
    esac
done

require_tool cmake
mkdir -p "$GAME_BUILD_DIR"
cmake -S "$GAME_ROOT" -B "$GAME_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$CONFIG" \
    -DGAME_BUILD_NAVIGATION_TESTS="$TESTS"

echo "Configured Linux/X11/OpenGL: $GAME_BUILD_DIR ($CONFIG, tests=$TESTS)"
