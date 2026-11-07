#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
bin="$GAME_BIN"
case "${1:-}" in
    Debug|debug|--debug) bin="$GAME_DEBUG_BIN"; shift ;;
    Release|release) shift ;;
esac
if [[ ! -x "$GAME_BIN" || ! -x "$GAME_DEBUG_BIN" ]]; then
    "$GAME_ROOT/build.sh"
fi
exec "$bin" "$@"
