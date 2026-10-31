#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
CONFIG="$(normalize_config "${1:-Release}")"
"$GAME_ROOT/build.sh" "$CONFIG" --tests
ctest --test-dir "$GAME_BUILD_DIR" --output-on-failure
