#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
for variant in Release Debug RelWithDebInfo MinSizeRel; do
    if [[ -f "$GAME_BUILD_DIR/$variant/CMakeCache.txt" ]]; then
        require_tool cmake
        cmake --build "$GAME_BUILD_DIR/$variant" --config "$variant" --target clean
    fi
done
rm -f -- "$GAME_BIN" "$GAME_DEBUG_BIN"
