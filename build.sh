#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/common.sh"

source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"

require_tool cmake
build_start_ms="$(date +%s%3N)"
configure_variant
variant_start_ms="$(date +%s%3N)"
cmake --build "$GAME_VARIANT_DIR" --config "$CONFIG" --parallel "$(cpu_jobs)" "${build_flags[@]}"
variant_time_ms="$(( $(date +%s%3N) - variant_start_ms ))"
expected="game $(tr -d '\r\n' < "$GAME_ROOT/VERSION")"
expected_mode=Release
if [[ "$DEBUG_FEATURES" == ON ]]; then expected_mode=Debug; fi
variant_bin="$GAME_VARIANT_DIR/bin/game"
if [[ ! -x "$variant_bin" || "$("$variant_bin" --version)" != "$expected" || "$("$variant_bin" --build-mode)" != "$expected_mode" ]]; then
    echo 'Missing game executable or incorrect version/debug feature mode. Rebuild required.' >&2
    exit 1
fi
mkdir -p "$GAME_BUILD_DIR/bin"
for runtime_folder in lib licenses debug; do
    if [[ -d "$GAME_VARIANT_DIR/bin/$runtime_folder" ]]; then
        cp -a -- "$GAME_VARIANT_DIR/bin/$runtime_folder" "$GAME_BUILD_DIR/bin/"
    fi
done
if [[ "$DEBUG_FEATURES" == OFF ]]; then
    rm -f -- "$GAME_BUILD_DIR/bin/debug/0-54-0-debug-start.gmsave"
fi
# Publish the selected executable after staging its runtime files.
staged_bin="$(mktemp "$GAME_BUILD_DIR/bin/.game-XXXXXX")"
trap 'rm -f -- "$staged_bin"' EXIT
cp -p -- "$variant_bin" "$staged_bin"
mv -f -- "$staged_bin" "$GAME_BIN"
rm -f -- "$GAME_BUILD_DIR/bin/game-debug" "$GAME_BUILD_DIR/bin/lib"/libSDL2-2.0d.so*
printf '\nGame: %s (debug features: %s)\n' "$GAME_BIN" "$DEBUG_FEATURES"
cmake "-DRUNTIME_DIR=$GAME_BUILD_DIR/bin" \
    "-DFIRST_CONFIG=$CONFIG" "-DFIRST_MS=$variant_time_ms" \
    "-DDEBUG_FEATURES=$DEBUG_FEATURES" \
    "-DTOTAL_MS=$(( $(date +%s%3N) - build_start_ms ))" \
    -P "$GAME_ROOT/scripts/build-summary.cmake"
