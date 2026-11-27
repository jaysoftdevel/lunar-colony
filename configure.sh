#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/common.sh"

source "$GAME_ROOT/scripts/build-arguments.sh"
parse_build_arguments "$@"

require_tool cmake
configure_variant
