#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"

require_tool cmake
build_start_ms="$(date +%s%3N)"
configure_variant "$CONFIG" "$TESTS" "$DEBUG_FEATURES"
variant_start_ms="$(date +%s%3N)"
cmake --build "$GAME_BUILD_DIR/$CONFIG" --config "$CONFIG" --parallel "$(cpu_jobs)" "${build_flags[@]}"
variant_time_ms="$(( $(date +%s%3N) - variant_start_ms ))"
expected="game $(tr -d '\r\n' < "$GAME_ROOT/VERSION")"
expected_mode=Release
if [[ "$DEBUG_FEATURES" == ON ]]; then expected_mode=Debug; fi
if [[ ! -x "$GAME_BIN" || "$("$GAME_BIN" --version)" != "$expected" || "$("$GAME_BIN" --build-mode)" != "$expected_mode" ]]; then
    echo 'Missing game executable or incorrect version/debug feature mode. Rebuild required.' >&2
    exit 1
fi
rm -f -- "$GAME_BUILD_DIR/bin/game-debug" "$GAME_BUILD_DIR/bin/lib"/libSDL2-2.0d.so*
printf '\nGame: %s (debug features: %s)\n' "$GAME_BIN" "$DEBUG_FEATURES"
cmake "-DRUNTIME_DIR=$GAME_BUILD_DIR/bin" \
    "-DFIRST_CONFIG=$CONFIG" "-DFIRST_MS=$variant_time_ms" \
    "-DDEBUG_FEATURES=$DEBUG_FEATURES" \
    "-DTOTAL_MS=$(( $(date +%s%3N) - build_start_ms ))" \
    -P "$GAME_ROOT/scripts/build-summary.cmake"
