#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

CONFIG=Release
TESTS="${GAME_BUILD_TESTS:-OFF}"
for arg in "$@"; do
    case "$arg" in
        --tests) TESTS=ON ;;
        *) CONFIG="$(normalize_config "$arg")" ;;
    esac
done

require_tool cmake
mapfile -t variants < <(build_variants "$CONFIG")
for variant in "${variants[@]}"; do
    configure_variant "$variant" "$TESTS"
done
