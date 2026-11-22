#!/usr/bin/env bash
# Shared argument handling for build.sh and configure.sh.
parse_build_arguments() {
    CONFIG=Release
    TESTS="${GAME_BUILD_TESTS:-OFF}"
    build_flags=()
    local selected_engine="" argument
    while (($#)); do
        argument="$1"
        case "$argument" in
            --tests) TESTS=ON ;;
            --clean) build_flags=(--clean-first) ;;
            --engine)
                shift
                if (($#==0)); then echo '--engine requires an engine source directory.' >&2; return 2; fi
                [[ -z "$selected_engine" ]] || { echo 'Supply one engine directory.' >&2; return 2; }
                selected_engine="$1" ;;
            --engine=*)
                [[ -z "$selected_engine" ]] || { echo 'Supply one engine directory.' >&2; return 2; }
                selected_engine="${argument#--engine=}"
                [[ -n "$selected_engine" ]] || { echo '--engine requires a directory.' >&2; return 2; } ;;
            Release|release|Debug|debug|RelWithDebInfo|relwithdebinfo|reldeb|MinSizeRel|minsizerel|minsize)
                CONFIG="$(normalize_config "$argument")" ;;
            -h|--help)
                echo "Usage: $0 /path/to/3d-engine [Release|Debug] [--tests] [--clean]"
                echo 'The engine can also be supplied with --engine PATH or JSD_GAME_ENGINE_DIR.'
                echo 'Later builds reuse the engine path cached by CMake.'
                exit 0 ;;
            -*) echo "Unknown option: $argument" >&2; return 2 ;;
            *)
                [[ -z "$selected_engine" ]] || { echo 'Supply one engine directory.' >&2; return 2; }
                selected_engine="$argument" ;;
        esac
        shift
    done
    JSD_GAME_ENGINE_DIR="${selected_engine:-${JSD_GAME_ENGINE_DIR:-}}"
    if [[ -n "$JSD_GAME_ENGINE_DIR" ]]; then
        if [[ ! -f "$JSD_GAME_ENGINE_DIR/JSD_PROJECT.json" || ! -f "$JSD_GAME_ENGINE_DIR/include/jsd/engine.hpp" ]]; then
            echo "Invalid JSD engine source directory: $JSD_GAME_ENGINE_DIR" >&2
            return 2
        fi
        JSD_GAME_ENGINE_DIR="$(cd -- "$JSD_GAME_ENGINE_DIR" && pwd -P)"
    fi
}
