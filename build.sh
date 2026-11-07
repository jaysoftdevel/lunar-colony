#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

CONFIG=Release
TESTS="${GAME_BUILD_TESTS:-OFF}"
build_flags=()
for arg in "$@"; do
    case "$arg" in
        --tests) TESTS=ON ;;
        --clean) build_flags=(--clean-first) ;;
        *) CONFIG="$(normalize_config "$arg")" ;;
    esac
done

require_tool cmake
mapfile -t variants < <(build_variants "$CONFIG")
for variant in "${variants[@]}"; do
    configure_variant "$variant" "$TESTS"
    cmake --build "$GAME_BUILD_DIR/$variant" --config "$variant" --parallel "$(cpu_jobs)" "${build_flags[@]}"
done

if [[ ! -x "$GAME_BIN" || ! -x "$GAME_DEBUG_BIN" ]]; then
    echo "Both game binaries are required in $GAME_BUILD_DIR/bin." >&2
    exit 1
fi
printf '\nRelease: %s\nDebug:   %s\n' "$GAME_BIN" "$GAME_DEBUG_BIN"
