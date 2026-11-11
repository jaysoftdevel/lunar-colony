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
build_start_ms="$(date +%s%3N)"
mapfile -t variants < <(build_variants "$CONFIG")
variant_times_ms=()
for variant in "${variants[@]}"; do
    configure_variant "$variant" "$TESTS"
    variant_start_ms="$(date +%s%3N)"
    cmake --build "$GAME_BUILD_DIR/$variant" --config "$variant" --parallel "$(cpu_jobs)" "${build_flags[@]}"
    variant_times_ms+=("$(( $(date +%s%3N) - variant_start_ms ))")
done

if [[ ! -x "$GAME_BIN" || ! -x "$GAME_DEBUG_BIN" ]]; then
    echo "Both game binaries are required in $GAME_BUILD_DIR/bin." >&2
    exit 1
fi
printf '\nRelease: %s\nDebug:   %s\n' "$GAME_BIN" "$GAME_DEBUG_BIN"
cmake "-DRUNTIME_DIR=$GAME_BUILD_DIR/bin" \
    "-DFIRST_CONFIG=${variants[0]}" "-DFIRST_MS=${variant_times_ms[0]}" \
    "-DSECOND_CONFIG=${variants[1]}" "-DSECOND_MS=${variant_times_ms[1]}" \
    "-DTOTAL_MS=$(( $(date +%s%3N) - build_start_ms ))" \
    -P "$GAME_ROOT/scripts/build-summary.cmake"
