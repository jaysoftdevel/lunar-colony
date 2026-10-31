#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CMD="${1:-build}"
shift || true
case "$CMD" in
    configure) exec "$ROOT/configure.sh" "$@" ;;
    build)     exec "$ROOT/build.sh" "$@" ;;
    rebuild)   exec "$ROOT/build.sh" "$@" --clean ;;
    run)       exec "$ROOT/run.sh" "$@" ;;
    test)      exec "$ROOT/test.sh" "$@" ;;
    clean)     exec "$ROOT/clean.sh" ;;
    deps)      exec "$ROOT/install-deps.sh" "$@" ;;
    *)
        echo "Usage: ./dev.sh {configure|build|rebuild|run|test|clean|deps} [arguments]" >&2
        exit 2
        ;;
esac
