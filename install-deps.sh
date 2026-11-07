#!/usr/bin/env bash
set -euo pipefail
if ! command -v apt-get >/dev/null 2>&1; then
    cat <<'MSG'
Automatic dependency installation currently supports Debian/Ubuntu systems.
Install these packages using your distribution's package manager:
  git, cmake, a C++20 compiler, make or ninja, libx11-dev, libgl1-mesa-dev, libopenal1
MSG
    exit 1
fi
SUDO=""
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then SUDO=sudo; fi
$SUDO apt-get update
$SUDO apt-get install -y git build-essential cmake ninja-build libx11-dev libgl1-mesa-dev mesa-common-dev libopenal1
cat <<'MSG'
Linux development dependencies installed.
Build both binaries with: ./build.sh
MSG
