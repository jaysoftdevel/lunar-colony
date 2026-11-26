#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"

source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"

require_tool cmake
configure_variant "$CONFIG" "$TESTS" "$DEBUG_FEATURES"
