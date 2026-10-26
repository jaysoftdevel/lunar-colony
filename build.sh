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
echo "Local build complete: $ROOT/build/bin/game"
echo "Run with: $ROOT/build/bin/game"
