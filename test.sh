#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
require_tool ctest
source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"
"$GAME_ROOT/build.sh" "$@" --tests
mapfile -t variants < <(build_variants "$CONFIG")
for variant in "${variants[@]}"; do
    ctest --test-dir "$GAME_BUILD_DIR/$variant" -C "$variant" --output-on-failure
done
