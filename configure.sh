#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"

require_tool cmake
mapfile -t variants < <(build_variants "$CONFIG")
for variant in "${variants[@]}"; do
    configure_variant "$variant" "$TESTS"
done
