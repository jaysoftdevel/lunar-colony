#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${1:-Release}"

# Savegames live beside the executable. Preserve them outside the clean build
# tree and restore them even if configuration or compilation fails.
SAVE_BACKUP=""
restore_saves(){
    local build_status=$?
    if [[ -n "$SAVE_BACKUP" ]]; then
        mkdir -p "$ROOT/build/bin"
        if cp -a "$SAVE_BACKUP/savegames" "$ROOT/build/bin/"; then
            rm -rf "$SAVE_BACKUP"
        else
            echo "Could not restore savegames; preserved copy: $SAVE_BACKUP/savegames" >&2
            return 1
        fi
    fi
    return "$build_status"
}
if [[ -d "$ROOT/build/bin/savegames" ]]; then
    SAVE_BACKUP="$(mktemp -d "$ROOT/.savegames-preserve.XXXXXX")"
    cp -a "$ROOT/build/bin/savegames" "$SAVE_BACKUP/savegames"
    trap restore_saves EXIT
fi

# Always build from a clean local build tree. No binary is copied into the source workspace.
rm -rf "$ROOT/build"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# Default build parallelism uses every logical CPU visible to the process, the
# same policy as the runtime worker pool. GAME_BUILD_JOBS may explicitly cap it.
if [[ -n "${GAME_BUILD_JOBS:-}" ]]; then
    JOBS="$GAME_BUILD_JOBS"
else
    CPUS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
    JOBS="$CPUS"
    (( JOBS < 1 )) && JOBS=1
fi
cmake --build "$ROOT/build" --parallel "$JOBS"

echo
GAME_BINARY="$ROOT/build/bin/game"
BINARY_BYTES="$(stat -c %s "$GAME_BINARY")"
BINARY_KB="$(( (BINARY_BYTES + 1023) / 1024 ))"

echo "Local build complete: $GAME_BINARY"
echo "Game binary size: ${BINARY_KB} KB (${BINARY_BYTES} bytes)"
echo "Run with: $GAME_BINARY"
