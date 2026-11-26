#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
for variant in Release RelWithDebInfo MinSizeRel; do
    if [[ -f "$GAME_BUILD_DIR/$variant/CMakeCache.txt" ]]; then
        require_tool cmake
        cmake --build "$GAME_BUILD_DIR/$variant" --config "$variant" --target clean
    fi
done
rm -f -- "$GAME_BIN" "$GAME_BUILD_DIR/bin/game-debug" "$GAME_BUILD_DIR/bin/debug/0-54-0-debug-start.gmsave" "$GAME_BUILD_DIR/bin/lib"/libSDL2-2.0d.so*
