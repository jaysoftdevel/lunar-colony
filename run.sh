#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/common.sh"
bin="$GAME_BIN"
case "${1:-}" in
    Debug|debug|--debug|Release|release) echo "Select debug features at build time: ./build.sh PATH_TO_ENGINE --debug" >&2; exit 2 ;;
esac
if [[ ! -x "$GAME_BIN" ]]; then
    if [[ -z "${JSD_GAME_ENGINE_DIR:-}" ]]; then
        echo 'Build the game first with ./build.sh PATH_TO_ENGINE.' >&2
        exit 1
    fi
    "$GAME_ROOT/build.sh" "$JSD_GAME_ENGINE_DIR"
fi
expected="game $(tr -d '\r\n' < "$GAME_ROOT/VERSION")"
actual="$("$bin" --version)"
if [[ "$actual" != "$expected" ]]; then
    echo "The installed binary is '$actual'; expected '$expected'." >&2
    echo 'Rebuild with ./build.sh PATH_TO_ENGINE before launching.' >&2
    exit 1
fi
exec "$bin" "$@"
