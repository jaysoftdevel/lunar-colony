#!/usr/bin/env python3
"""Portable checks of the single-binary build contract."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='colony-build-contract-') as temp:
    folder = Path(temp)
    game = folder / 'game workspace'
    engine = folder / '3d engine'
    fake_bin = folder / 'tools'
    (game / 'scripts').mkdir(parents=True)
    (game / 'game').mkdir()
    (game / 'cmake').mkdir()
    (game / 'game/CMakeLists.txt').touch()
    (game / 'cmake/ExternalEngine.cmake').touch()
    (engine / 'include/jsd').mkdir(parents=True)
    fake_bin.mkdir()
    for item in ['VERSION', 'build.sh', 'configure.sh', 'run.sh', 'test.sh', 'scripts/common.sh', 'scripts/build-arguments.sh']:
        shutil.copy2(root / item, game / item)
    (engine / 'include/jsd/engine.hpp').write_text('// test source root')
    identity = {'project_id': 'jsd.engine', 'kind': 'source', 'version': '0.5.25'}
    (engine / 'JSD_PROJECT.json').write_text(json.dumps(identity))
    mock = fake_bin / 'cmake'
    mock.write_text(r'''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
args=sys.argv[1:]
with open(os.environ['COLONY_BUILD_LOG'], 'a') as f: f.write(json.dumps(args)+'\n')
if '-S' in args:
    path=Path(args[args.index('-B')+1]); path.mkdir(parents=True,exist_ok=True)
    (path/'source').write_text(args[args.index('-S')+1])
    (path/'mode').write_text('Debug' if '-DGAME_DEBUG_FEATURES=ON' in args else 'Release')
if '--build' in args:
    path=Path(args[args.index('--build')+1]); output=path/'bin'
    output.mkdir(parents=True, exist_ok=True)
    mode=(path/'mode').read_text(); version=(Path((path/'source').read_text())/'VERSION').read_text().strip()
    p=output/'game'; p.write_text(f'#!/bin/sh\ncase "$1" in --version) echo "game {version}";; --build-mode) echo {mode};; *) echo launched;; esac\n'); p.chmod(0o755)
''')
    mock.chmod(0o755)
    ctest = fake_bin/'ctest'; ctest.write_text('#!/bin/sh\nexit 0\n'); ctest.chmod(0o755)
    log = folder / 'calls.jsonl'
    env = dict(os.environ, PATH=str(fake_bin)+os.pathsep+os.environ['PATH'], COLONY_BUILD_LOG=str(log), GAME_BUILD_JOBS='2')
    env['JSD_GAME_ENGINE_DIR'] = str(folder / 'stale engine')
    output=game/'build/linux/bin'; output.mkdir(parents=True)
    (output/'game-debug').touch()
    def call(script, *args, cwd=game, environment=env):
        return subprocess.run(['bash', str(game/script), *args], env=environment, cwd=cwd,
                              capture_output=True, text=True)
    engine_copy = folder / 'moved engine'
    shutil.copytree(engine, engine_copy)
    builds = {}
    for selected, options, mode in [(engine, ['--debug','--tests'], 'ON'),
                                    (engine_copy, [], 'OFF'),
                                    (engine, ['Release','--debug'], 'ON')]:
        log.write_text('')
        result = call('build.sh', os.path.relpath(selected, game), *options)
        assert result.returncode == 0, result.stderr
        calls = [json.loads(line) for line in log.read_text().splitlines()]
        configured = [c for c in calls if '-S' in c]
        assert len(configured) == 1 and sum('--build' in c for c in calls) == 1, calls
        args = configured[0]
        build_dir = args[args.index('-B')+1]
        assert args[args.index('-S')+1] == str(game)
        assert '-DJSD_GAME_ENGINE_DIR='+str(selected) in args
        assert '-DGAME_DEBUG_FEATURES='+mode in args
        assert '-DCMAKE_BUILD_TYPE=Release' in args
        assert builds.setdefault(selected, build_dir) == build_dir
        assert not (output/'game-debug').exists()
        assert call('run.sh','--build-mode').stdout.strip() == ('Debug' if mode=='ON' else 'Release')
    assert builds[engine] != builds[engine_copy]
    assert call('run.sh','--debug').returncode == 2
    for args in [[], [str(game), str(engine)], [str(engine), str(engine)],
                 [str(folder/'missing')], [str(engine),'--unsupported']]:
        log.write_text(''); result = call('build.sh', *args)
        assert result.returncode != 0 and not log.read_text(), args
    log.write_text('')
    assert call('configure.sh', engine_copy.name, '--debug', cwd=folder).returncode == 0
    configured = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(configured) == 1 and '-DJSD_GAME_ENGINE_DIR='+str(engine_copy) in configured[0]
    assert configured[0][configured[0].index('-S')+1] == str(game)
    assert call('test.sh', str(engine)).returncode == 0
    (output/'game').unlink()
    assert call('run.sh', '--version', environment=dict(env, JSD_GAME_ENGINE_DIR=str(engine))).returncode == 0
    # Reject another project/version before configuring bundled dependencies.
    for identity in [{'project_id':'other.game','kind':'source','version':'0.5.25'}, {'project_id':'jsd.engine','kind':'source','version':'0.5.10'}]:
        (engine/'JSD_PROJECT.json').write_text(json.dumps(identity))
        result=subprocess.run(['cmake','-DJSD_GAME_ENGINE_DIR='+str(engine),'-P',str(root/'cmake/ExternalEngine.cmake')],capture_output=True,text=True)
        assert result.returncode!=0 and 'requires JSD engine source 0.5.25' in result.stderr
print('PASS single executable, debug ON/OFF switching, launch, legacy cleanup, engine relocation and identity guards')
