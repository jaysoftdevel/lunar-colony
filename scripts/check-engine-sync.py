#!/usr/bin/env python3
"""Check game/engine source identities and current native-Windows integration."""
import argparse
import json
from pathlib import Path
import re
import sys

GAME = '0.93.8'
ENGINE = '0.5.25'

def check(game: Path, engine: Path):
    game, engine = game.resolve(), engine.resolve()
    if not (game/'VERSION').is_file() or not (engine/'JSD_PROJECT.json').is_file():
        raise RuntimeError('Supply the game source and external JSD source roots.')
    version = (game/'VERSION').read_text().strip()
    marker = json.loads((engine/'JSD_PROJECT.json').read_text())
    if version != GAME or marker.get('project_id') != 'jsd.engine' or marker.get('version') != ENGINE:
        raise RuntimeError(f'Expected game {GAME} / JSD {ENGINE}; found {version} / {marker.get("version")}')
    if game == engine or game in engine.parents or engine in game.parents:
        raise RuntimeError('Game and engine must be separate source roots.')
    engine_header = (engine/'include/jsd/engine.hpp').read_text()
    frontier_header = (engine/'include/frontier/renderer.hpp').read_text()
    runtime = (engine/'src/runtime/runtime.cpp').read_text()
    for label, content in [('JSD C++ API', engine_header), ('Frontier C++ API', frontier_header), ('runtime', runtime)]:
        if ENGINE not in content: raise RuntimeError(f'{label} is out of sync with JSD {ENGINE}.')
    for name in ('build.sh', 'install_deps.sh'):
        launcher = (engine/name).read_text()
        if f'--expected {ENGINE}' not in launcher:
            raise RuntimeError(f'Engine {name} is out of sync with JSD {ENGINE}.')
    engine_windows = (engine/'tools/build_windows.py').read_text()
    guards = re.findall(r"source\(root,\s*'jsd.engine',\s*'([^']+)'\)", engine_windows)
    if not guards or any(expected != ENGINE for expected in guards):
        raise RuntimeError(f'Engine Windows build driver is out of sync with JSD {ENGINE}.')
    external = (game/'cmake/ExternalEngine.cmake').read_text()
    windows_driver = (game/'scripts/windows-build.py').read_text()
    game_targets = (game/'game/CMakeLists.txt').read_text()
    clusters = (game/'game/cmake/GameClusters.cmake').read_text()
    if ENGINE not in external or f"ENGINE_VERSION = '{ENGINE}'" not in windows_driver:
        raise RuntimeError('Windows driver / CMake use different JSD engine versions.')
    if 'add_custom_command(TARGET ${target} POST_BUILD' not in game_targets:
        raise RuntimeError('Windows game/test runtime staging is not configuration-local.')
    if 'add_dependencies(${target} SDL2)' not in game_targets:
        raise RuntimeError('SDL runtime staging dependency is missing.')
    if '/bigobj' not in clusters or '/utf-8' not in clusters:
        raise RuntimeError('MSVC cluster build flags are missing.')
    print(f'Game {version} / JSD {ENGINE}: source interfaces and Windows build integration aligned.')

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--game', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--engine', type=Path, required=True)
    args = parser.parse_args(argv)
    check(args.game, args.engine)

if __name__ == '__main__':
    try: main()
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f'Sync check failed: {exc}', file=sys.stderr); sys.exit(1)
