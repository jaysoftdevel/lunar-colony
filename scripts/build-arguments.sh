#!/usr/bin/env bash
# Shared argument handling for build.sh, configure.sh and test.sh.
build_usage() {
    echo "Usage: $0 PATH_TO_ENGINE [Release|RelWithDebInfo|MinSizeRel] [--debug] [--tests] [--clean]"
}

parse_build_arguments() {
    if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then build_usage; exit 0; fi
    if (($# == 0)); then build_usage >&2; return 2; fi
    local engine="$1" argument
    shift
    if [[ ! -f "$GAME_ROOT/VERSION" || ! -f "$GAME_ROOT/game/CMakeLists.txt" || ! -f "$GAME_ROOT/cmake/ExternalEngine.cmake" ]]; then
        echo "Invalid Lunar Colony source directory: $GAME_ROOT" >&2
        return 2
    fi
    if [[ ! -f "$engine/JSD_PROJECT.json" || ! -f "$engine/include/jsd/engine.hpp" ]]; then
        echo "Invalid JSD engine source directory: $engine" >&2
        return 2
    fi
    JSD_GAME_ENGINE_DIR="$(CDPATH= cd -- "$engine" && pwd -P)"

    CONFIG=Release
    DEBUG_FEATURES=OFF
    TESTS="${GAME_BUILD_TESTS:-OFF}"
    build_flags=()
    while (($#)); do
        argument="$1"
        case "$argument" in
            --tests) TESTS=ON ;;
            --debug) DEBUG_FEATURES=ON ;;
            Debug|debug) echo "Use --debug to include debug features in the normal build." >&2; return 2 ;;
            --clean) build_flags=(--clean-first) ;;
            Release|release|RelWithDebInfo|relwithdebinfo|reldeb|MinSizeRel|minsizerel|minsize)
                CONFIG="$(normalize_config "$argument")" ;;
            -h|--help) build_usage; exit 0 ;;
            *) echo "Unknown build argument: $argument" >&2; return 2 ;;
        esac
        shift
    done
    # CMake and bundled dependency caches contain absolute paths. Give each
    # game/engine pair a stable build tree and stage the selected binary at bin/.
    require_tool sha256sum
    local path_hash
    path_hash="$(printf '%s\0%s\0' "$GAME_ROOT" "$JSD_GAME_ENGINE_DIR" | sha256sum)"
    GAME_VARIANT_DIR="$GAME_BUILD_DIR/engines/${path_hash%% *}/$CONFIG"
}
