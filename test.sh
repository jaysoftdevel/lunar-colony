#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
require_tool ctest
CONFIG="$(normalize_config "${1:-Release}")"
"$GAME_ROOT/build.sh" "$CONFIG" --tests
mapfile -t variants < <(build_variants "$CONFIG")
for variant in "${variants[@]}"; do
    ctest --test-dir "$GAME_BUILD_DIR/$variant" -C "$variant" --output-on-failure
done
