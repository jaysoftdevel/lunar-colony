#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
bin="$GAME_BIN"
case "${1:-}" in
    Debug|debug|--debug|Release|release) echo "Select debug features at build time: ./build.sh [--debug]; then ./run.sh" >&2; exit 2 ;;
esac
if [[ ! -x "$GAME_BIN" ]]; then
    "$GAME_ROOT/build.sh"
fi
expected="game $(tr -d '\r\n' < "$GAME_ROOT/VERSION")"
actual="$("$bin" --version)"
if [[ "$actual" != "$expected" ]]; then
    echo "The installed binary is '$actual'; expected '$expected'." >&2
    echo 'Rebuild with ./build.sh /absolute/path/to/3d-engine before launching.' >&2
    exit 1
fi
exec "$bin" "$@"
