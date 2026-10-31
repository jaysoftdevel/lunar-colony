#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
if [[ ! -x "$GAME_BIN" ]]; then
    echo "No Linux executable found. Building Release first..."
    "$GAME_ROOT/build.sh" Release
fi
exec "$GAME_BIN" "$@"
