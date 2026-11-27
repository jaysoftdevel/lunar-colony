#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/common.sh"
require_tool ctest
source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"
"$GAME_ROOT/build.sh" "$@" --tests
ctest --test-dir "$GAME_VARIANT_DIR" -C "$CONFIG" --output-on-failure
