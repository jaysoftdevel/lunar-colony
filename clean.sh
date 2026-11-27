#!/usr/bin/env bash
# Remove the entire generated game build tree, not just the current target.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD="$ROOT/build"
if [[ $# -ne 0 ]]; then
    if [[ $# -eq 1 && ( "$1" == --help || "$1" == -h ) ]]; then
        echo 'Usage: ./clean.sh (removes all Linux and Windows game build configurations)'
        exit 0
    fi
    echo 'Usage: ./clean.sh' >&2
    exit 2
fi
if [[ ! -f "$ROOT/VERSION" || ! -f "$ROOT/game/CMakeLists.txt" || ! -f "$ROOT/cmake/ExternalEngine.cmake" ]]; then
    echo 'Refusing cleanup: not a Lunar Colony game source workspace.' >&2
    exit 1
fi
if [[ -L "$BUILD" || ( -e "$BUILD" && ! -d "$BUILD" ) ]]; then
    echo "Refusing cleanup: build is a symbolic link or is not a directory: $BUILD" >&2
    exit 1
fi
if [[ ! -d "$BUILD" ]]; then
    echo 'Already clean: no game build directory.'
    exit 0
fi
# Old versions stored player saves under bin/savegames. Protect those and any
# other unexpected save/config files. Only identical staged debug fixtures
# and known generated regression outputs in test-saves are build artifacts.
list="$(mktemp)"
trap 'rm -f -- "$list"' EXIT
find -P "$BUILD" ! -type d \( -iname '*.gmsave' -o -iname 'settings.cfg' -o -iname 'graphics.cfg' \) -print0 > "$list"
blocked=0
while IFS= read -r -d '' path; do
    rel="${path#"$BUILD"/}"
    case "$rel" in
        linux/bin/debug/*|windows/bin/debug/*|linux/*/bin/debug/*|windows/*/bin/debug/*)
            name="${path##*/}"
            case "$name" in
                0-54-0-debug-start.gmsave|0-40-32-debug-start.gmsave)
                    if [[ ! -L "$path" ]] && cmp -s -- "$path" "$ROOT/game/debug-templates/$name"; then
                        continue
                    fi ;;
            esac ;;
        linux/*/game/test-saves/*|windows/game/test-saves/*) continue ;;
    esac
    printf 'Preserved possible user data: %s\n' "$path" >&2
    blocked=1
done < "$list"
if (( blocked )); then
    echo 'No build files removed. Move the listed files outside build/ (e.g. saves to savegames/) and rerun clean.sh.' >&2
    exit 1
fi
# GNU rm does not traverse symbolic links and --one-file-system prevents
# traversal into mounted filesystems nested within the generated build tree.
rm -rf --one-file-system -- "$BUILD"
if [[ -e "$BUILD" || -L "$BUILD" ]]; then
    echo "Cleanup incomplete: $BUILD still exists." >&2
    exit 1
fi
printf 'Cleaned all game build artifacts: %s\n' "$BUILD"
echo 'Source files, root savegames, settings outside build/, and the external engine remain unchanged.'
echo 'Next build: ./build.sh /path/to/3d-engine [--debug]'
