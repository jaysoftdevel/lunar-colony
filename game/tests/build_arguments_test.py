#!/usr/bin/env python3
"""Check the external-source build contract without compiling a second engine."""
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
    (engine / 'include/jsd').mkdir(parents=True)
    fake_bin.mkdir()
    for item in ['build.sh', 'configure.sh', 'scripts/common.sh', 'scripts/build-arguments.sh']:
        shutil.copy2(root / item, game / item)
    (engine / 'include/jsd/engine.hpp').write_text('// test source root')
    identity = {'project_id': 'jsd.engine', 'kind': 'source', 'version': '0.5.12'}
    (engine / 'JSD_PROJECT.json').write_text(json.dumps(identity))
    mock = fake_bin / 'cmake'
    mock.write_text('''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
with open(os.environ['COLONY_BUILD_LOG'], 'a') as f: f.write(json.dumps(sys.argv[1:])+'\\n')
if '--build' in sys.argv:
    output=Path(sys.argv[sys.argv.index('--build')+1]).parent/'bin'
    output.mkdir(parents=True, exist_ok=True)
    for name in ['game', 'game-debug']:
        p=output/name; p.write_text('#!/bin/sh\\nexit 0\\n'); p.chmod(0o755)
''')
    mock.chmod(0o755)
    log = folder / 'calls.jsonl'
    env = dict(os.environ, PATH=str(fake_bin)+os.pathsep+os.environ['PATH'], COLONY_BUILD_LOG=str(log), GAME_BUILD_JOBS='2')
    env.pop('JSD_GAME_ENGINE_DIR', None)
    for args in [[str(engine), 'Debug', '--tests'], ['--engine', str(engine)], ['Release', '--engine='+str(engine)]]:
        log.write_text('')
        result = subprocess.run(['bash', str(game/'build.sh'), *args], env=env, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        calls = [json.loads(line) for line in log.read_text().splitlines()]
        configured = [call for call in calls if '-S' in call]
        assert len(configured) == 2
        assert all('-DJSD_GAME_ENGINE_DIR='+str(engine) in call for call in configured)
    for args in [['--engine'], ['--engine='], ['--unsupported'], [str(folder/'missing')], [str(engine), str(engine)]]:
        log.write_text('')
        result = subprocess.run(['bash', str(game/'build.sh'), *args], env=env, capture_output=True, text=True)
        assert result.returncode != 0 and not log.read_text(), args
    # Reject another project/version before configuring bundled dependencies.
    for identity in [{'project_id':'other.game','kind':'source','version':'0.5.12'}, {'project_id':'jsd.engine','kind':'source','version':'0.5.10'}]:
        (engine/'JSD_PROJECT.json').write_text(json.dumps(identity))
        result = subprocess.run(['cmake', '-DJSD_GAME_ENGINE_DIR='+str(engine), '-P', str(root/'cmake/ExternalEngine.cmake')], capture_output=True, text=True)
        assert result.returncode != 0 and 'requires JSD engine source 0.5.12' in result.stderr
print('PASS external engine arguments, spaces, both configurations and identity guards')
