#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
require_tool ctest
source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"
"$GAME_ROOT/build.sh" "$@" --tests
ctest --test-dir "$GAME_BUILD_DIR/$CONFIG" -C "$CONFIG" --output-on-failure
