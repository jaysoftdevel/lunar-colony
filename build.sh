#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${1:-Release}"

# Always build from a clean local build tree. No binary is copied into the source workspace.
rm -rf "$ROOT/build"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# The project is intentionally split into many small translation units. Limiting
# default parallelism avoids excessive compiler memory use on typical developer
# machines while GAME_BUILD_JOBS can still override it explicitly.
if [[ -n "${GAME_BUILD_JOBS:-}" ]]; then
    JOBS="$GAME_BUILD_JOBS"
else
    CPUS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
    (( CPUS > 4 )) && JOBS=4 || JOBS="$CPUS"
fi
cmake --build "$ROOT/build" --parallel "$JOBS"

echo
GAME_BINARY="$ROOT/build/bin/game"
BINARY_BYTES="$(stat -c %s "$GAME_BINARY")"
BINARY_KB="$(( (BINARY_BYTES + 1023) / 1024 ))"

echo "Local build complete: $GAME_BINARY"
echo "Game binary size: ${BINARY_KB} KB (${BINARY_BYTES} bytes)"
echo "Run with: $GAME_BINARY"
