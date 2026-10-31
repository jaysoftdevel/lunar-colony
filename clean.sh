#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/common.sh"
rm -rf "$GAME_BUILD_DIR"
echo "Removed Linux build tree: $GAME_BUILD_DIR"
