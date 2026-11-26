#!/usr/bin/env python3
"""Build the colony and its external engine with one native Windows toolchain."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
ENGINE_VERSION = '0.5.22'
CONFIGURATIONS = ('Release', 'RelWithDebInfo', 'MinSizeRel')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def cache_values(path):
    result = {}
    if path.is_file():
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
            match = re.match(r'([^/#][^:=]*):[^=]*=(.*)', line)
            if match:
                result[match[1]] = match[2]
    return result


def engine_source(root, selected, cache):
    chosen = selected or os.environ.get('JSD_GAME_ENGINE_DIR') or cache.get('JSD_GAME_ENGINE_DIR')
    require(chosen, 'Supply the external engine: build.bat --engine PATH')
    engine = Path(chosen).resolve()
    require(engine != root and not engine.is_relative_to(root) and not root.is_relative_to(engine),
            'Game and engine must be separate, non-nested source directories.')
    marker = engine/'JSD_PROJECT.json'
    require(marker.is_file(), f'Engine source marker is missing: {marker}')
    meta = json.loads(marker.read_text(encoding='utf-8'))
    require(meta.get('project_id') == 'jsd.engine' and meta.get('kind') == 'source' and
            meta.get('version') == ENGINE_VERSION and meta.get('cmake_project') == 'JSDGameEngine',
            f'Expected JSD engine source {ENGINE_VERSION}; selected {meta.get("project_id")} {meta.get("version")}.')
    for name in ('include/frontier/renderer.hpp', 'src/renderer.cpp', 'tools/build_windows.py'):
        require((engine/name).is_file(), f'Incomplete engine source: {name}')
    previous = cache.get('GAME_CONFIGURED_ENGINE_DIR') or cache.get('JSD_GAME_ENGINE_DIR')
    require(not previous or Path(previous).resolve() == engine,
            'This build directory belongs to another engine. Remove build/windows before switching engine roots.')
    return engine


def select_toolchain(engine, cmake, generator):
    # Reuse the external engine's installed-toolchain detection and CMake checks.
    sys.path.insert(0, str(engine/'tools'))
    try:
        spec = importlib.util.spec_from_file_location('colony_jsd_windows_toolchain', engine/'tools/build_windows.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.select_toolchain(cmake, generator)
    finally:
        sys.path.pop(0)


def command(args, root):
    print('Running: '+subprocess.list2cmdline([str(arg) for arg in args]), flush=True)
    subprocess.run([str(arg) for arg in args], cwd=root, check=True)


def run(args, root=ROOT):
    root = root.resolve()
    start = time.perf_counter()
    require(args.jobs >= 0, "Build jobs must be a positive integer (0 selects automatic concurrency).")
    require((root/'VERSION').is_file() and (root/'game/include/game/game_app.hpp').is_file(),
            'Select the Lunar Colony source root.')
    version = (root/'VERSION').read_text(encoding='utf-8').strip()
    require(re.fullmatch(r'\d+\.\d+\.\d+', version), 'Invalid game source VERSION.')
    build = root/'build/windows'
    cache = cache_values(build/'CMakeCache.txt')
    require(not cache.get('CMAKE_HOME_DIRECTORY') or Path(cache['CMAKE_HOME_DIRECTORY']).resolve() == root,
            'Build cache belongs to another game source root. Remove build/windows first.')
    require(not cache.get('CMAKE_PROJECT_NAME') or cache['CMAKE_PROJECT_NAME'] == 'game_workspace',
            'Build cache belongs to another project. Remove build/windows first.')
    cached_version = cache.get('CMAKE_PROJECT_VERSION')
    if cached_version and re.fullmatch(r'\d+\.\d+\.\d+', cached_version):
        require(tuple(map(int, cached_version.split('.'))) <= tuple(map(int, version.split('.'))),
                'Build cache is newer than the source version. Remove build/windows before downgrading.')
    engine = engine_source(root, args.engine, cache)
    require(not cache.get('CMAKE_GENERATOR_PLATFORM') or cache['CMAKE_GENERATOR_PLATFORM'] == 'x64',
            'Build cache belongs to another architecture. Remove build/windows for an x64 build.')
    cmake, generator, instance = select_toolchain(engine, args.cmake, args.generator or cache.get('CMAKE_GENERATOR'))
    require(not cache.get('CMAKE_GENERATOR') or cache['CMAKE_GENERATOR'] == generator,
            'Build cache uses another generator. Remove build/windows before changing toolchains.')
    previous_instance = cache.get('CMAKE_GENERATOR_INSTANCE', '').split(',')[0]
    require(not previous_instance or not instance or Path(previous_instance).resolve() == Path(instance).resolve(),
            'Build cache uses another Visual Studio installation. Remove build/windows first.')
    tests = args.tests or args.action == 'test' or args.render_tests
    flags = [cmake, '-S', root, '-B', build, '-G', generator, '-A', 'x64',
             '-DJSD_GAME_ENGINE_DIR='+str(engine), '-DJSD_GUARD_PYTHON='+sys.executable, '-DGAME_BUILD_NAVIGATION_TESTS='+('ON' if tests else 'OFF'),
             '-DGAME_BUILD_RENDER_TESTS='+('ON' if args.render_tests else 'OFF'),
             '-DGAME_DEBUG_FEATURES='+('ON' if args.debug else 'OFF')]
    if instance:
        flags.append('-DCMAKE_GENERATOR_INSTANCE='+instance)
    command(flags, root)
    if args.action == 'configure':
        return
    jobs = args.jobs or os.cpu_count() or 4
    require(jobs > 0, 'Build jobs must be a positive integer.')
    variant_start = time.perf_counter()
    flags = [cmake, '--build', build, '--config', args.configuration, '--parallel', str(jobs)]
    if args.clean:
        flags.append('--clean-first')
    command(flags, root)
    duration = round((time.perf_counter()-variant_start)*1000)
    runtime_dir = build/'bin'
    executable = runtime_dir/'game.exe'
    require(executable.is_file(), f'Missing game binary: {executable}')
    require((runtime_dir/'SDL2.dll').is_file(), 'Missing SDL2 runtime DLL beside game.exe. Rebuild the game and JSD engine together.')
    for option, expected in (('--version', 'game '+version), ('--build-mode', 'Debug' if args.debug else 'Release')):
        result = subprocess.run([str(executable), option], cwd=root, check=True,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        require(result.stdout.strip() == expected, f'Binary {option} does not match requested build: {executable}')
    # Remove obsolete packaged outputs only after a successful verified build.
    for name in ('game-debug.exe', 'game-debug.pdb', 'SDL2d.dll'):
        (runtime_dir/name).unlink(missing_ok=True)
    command([cmake, '-DRUNTIME_DIR='+str(runtime_dir),
             '-DFIRST_CONFIG='+args.configuration, '-DFIRST_MS='+str(duration),
             '-DDEBUG_FEATURES='+('ON' if args.debug else 'OFF'),
             '-DTOTAL_MS='+str(round((time.perf_counter()-start)*1000)),
             '-P', root/'scripts/build-summary.cmake'], root)
    if args.action == 'test':
        ctest = Path(cmake).with_name('ctest.exe')
        require(ctest.is_file(), f'CTest is missing beside CMake: {ctest}')
        command([ctest, '--test-dir', build, '-C', args.configuration, '--output-on-failure'], root)
    print(f'Lunar Colony {version} / JSD {ENGINE_VERSION}: game.exe ready in {runtime_dir} (debug features: {args.debug})')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'configure', 'test'))
    parser.add_argument('inputs', nargs='*', help='Configuration and/or positional engine source path')
    parser.add_argument('--engine')
    parser.add_argument('--debug', action='store_true', help='Include debug gameplay features in the single game executable')
    parser.add_argument('--tests', action='store_true', default=os.environ.get('GAME_BUILD_TESTS', '').upper() in ('ON', '1', 'TRUE'))
    parser.add_argument('--render-tests', action='store_true')
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('--cmake')
    parser.add_argument('--generator', choices=('Visual Studio 17 2022', 'Visual Studio 18 2026'))
    parser.add_argument('--jobs', type=int, default=int(os.environ.get('GAME_BUILD_JOBS') or 0))
    args = parser.parse_intermixed_args(argv)
    args.configuration = 'Release'
    configured = False
    for value in args.inputs:
        if value.lower() == "debug":
            parser.error("Use --debug to include debug features in the normal build.")
        config = next((c for c in CONFIGURATIONS if c.lower() == value.lower()), None)
        if config:
            if configured:
                parser.error('Specify one build configuration.')
            args.configuration = config
            configured = True
        elif (Path(value)/'JSD_PROJECT.json').is_file():
            if args.engine:
                parser.error('Specify the engine source path once.')
            args.engine = value
        else:
            parser.error('Unknown configuration or engine source path: '+value)
    return args


if __name__ == '__main__':
    try:
        require(sys.version_info >= (3, 10), 'Python 3.10 or newer is required.')
        require(os.name == 'nt', 'Use build.sh on Linux. This driver selects a native Windows x64 toolchain.')
        run(parse_args())
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Windows build stopped: {error}', file=sys.stderr)
        sys.exit(1)
