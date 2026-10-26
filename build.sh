#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${1:-Release}"

# Always build from a clean local build tree. No binary is copied into the source workspace.
rm -rf "$ROOT/build"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$ROOT/build" --parallel

echo
echo "Local build complete: $ROOT/build/bin/game"
echo "Run with: $ROOT/build/bin/game"
