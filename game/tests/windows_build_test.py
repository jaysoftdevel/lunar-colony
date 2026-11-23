"""Portable tests of Windows toolchain selection, argument paths and build guards."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('colony_windows', ROOT/'scripts/windows-build.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)

class WindowsBuild(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='windows build with spaces ')
        self.parent = Path(self.temp.name)
        self.root, self.engine = self.parent/'lunar colony', self.parent/'3d engine'
        for name, text in [('VERSION', '0.72.1'), ('game/include/game/game_app.hpp', '')]:
            p = self.root/name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
        for name in ('include/frontier/renderer.hpp', 'src/renderer.cpp', 'tools/build_windows.py'):
            p = self.engine/name; p.parent.mkdir(parents=True, exist_ok=True); p.touch()
        self.identity = self.engine/'JSD_PROJECT.json'
        self.identity.write_text(json.dumps(dict(project_id='jsd.engine', kind='source', version=driver.ENGINE_VERSION, cmake_project='JSDGameEngine')))
        self.cmake = self.parent/'Visual Studio tools/cmake.exe'; self.cmake.parent.mkdir(); self.cmake.touch()
        self.cmake.with_name('ctest.exe').touch()
        self.calls = []
        self.environment = patch.dict(os.environ, {'JSD_GAME_ENGINE_DIR':'', 'GAME_BUILD_JOBS':'', 'GAME_BUILD_TESTS':''})
        self.environment.start()
    def tearDown(self):
        self.environment.stop(); self.temp.cleanup()
    def args(self, *extra):
        return driver.parse_args(['build', '--engine', str(self.engine), *extra])
    def invoke(self, args):
        build = self.root/'build/windows/bin'; build.mkdir(parents=True, exist_ok=True)
        for name in ('game.exe', 'game-debug.exe', 'SDL2.dll'): (build/name).touch()
        with patch.object(driver, 'select_toolchain', return_value=(str(self.cmake), 'Visual Studio 18 2026', str(self.parent/'Visual Studio'))), \
             patch.object(driver, 'command', side_effect=lambda args, root: self.calls.append([str(x) for x in args])), \
             patch.object(driver.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'game 0.72.1\n', '')):
            driver.run(args, self.root)
    def write_cache(self, text):
        p = self.root/'build/windows/CMakeCache.txt'; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
    def test_paths_configuration_and_both_builds(self):
        self.invoke(self.args('Debug', '--jobs', '3'))
        self.assertIn('-DJSD_GAME_ENGINE_DIR='+str(self.engine), self.calls[0])
        self.assertIn('-DCMAKE_GENERATOR_INSTANCE='+str(self.parent/'Visual Studio'), self.calls[0])
        self.assertEqual(self.calls[1][-4:], ['--config', 'Debug', '--parallel', '3'])
        self.assertEqual(self.calls[2][-4:], ['--config', 'Release', '--parallel', '3'])
        self.assertIn('-P', self.calls[3])
    def test_configure_only(self):
        args = self.args(); args.action = 'configure'; self.invoke(args)
        self.assertEqual(len(self.calls), 1)
    def test_ctest_both_variants_and_render_flag(self):
        args = self.args('--render-tests'); args.action = 'test'; self.invoke(args)
        self.assertIn('-DGAME_BUILD_RENDER_TESTS=ON', self.calls[0])
        self.assertEqual([call[-2] for call in self.calls[-2:]], ['Release', 'Debug'])
    def test_positional_engine_and_interspersed_options(self):
        args = driver.parse_args(['build', str(self.engine), '--jobs', '2', 'debug', '--tests'])
        self.assertEqual((args.engine, args.configuration, args.jobs, args.tests), (str(self.engine), 'Debug', 2, True))
    def test_cached_engine_used(self):
        self.write_cache('JSD_GAME_ENGINE_DIR:PATH='+str(self.engine)+'\n')
        args = driver.parse_args(['configure']); self.invoke(args)
        self.assertIn('-DJSD_GAME_ENGINE_DIR='+str(self.engine), self.calls[0])
    def test_foreign_cache_rejected_before_command(self):
        self.write_cache('CMAKE_HOME_DIRECTORY:INTERNAL='+str(self.parent/'other game')+'\n')
        with self.assertRaisesRegex(RuntimeError, 'another game'): self.invoke(self.args())
        self.assertEqual(self.calls, [])
    def test_newer_source_cache_refused(self):
        self.write_cache('CMAKE_PROJECT_VERSION:STATIC=0.99.0\n')
        with self.assertRaisesRegex(RuntimeError, 'newer'): self.invoke(self.args())
        self.assertEqual(self.calls, [])
    def test_engine_switch_refused(self):
        self.write_cache('GAME_CONFIGURED_ENGINE_DIR:INTERNAL='+str(self.parent/'other engine')+'\n')
        with self.assertRaisesRegex(RuntimeError, 'another engine'): self.invoke(self.args())
        self.assertEqual(self.calls, [])
    def test_wrong_engine_version_or_identity(self):
        for key, value in [('version', '0.5.11'), ('project_id', 'other.game')]:
            meta = json.loads(self.identity.read_text()); original = meta.copy(); meta[key] = value; self.identity.write_text(json.dumps(meta))
            with self.assertRaisesRegex(RuntimeError, 'Expected JSD'): self.invoke(self.args())
            self.identity.write_text(json.dumps(original))
        self.assertEqual(self.calls, [])
    def test_negative_jobs_refused_before_configure(self):
        with self.assertRaisesRegex(RuntimeError, 'positive integer'): self.invoke(self.args('--jobs', '-1'))
        self.assertEqual(self.calls, [])
    def test_missing_sdl_runtime_rejected(self):
        original = self.invoke
        def invoke_without_dll(args):
            root = self.root
            binary_dir = root/'build/windows/bin'
            binary_dir.mkdir(parents=True, exist_ok=True)
            for name in ('game.exe', 'game-debug.exe'): (binary_dir/name).touch()
            with patch.object(driver, 'select_toolchain', return_value=(str(self.cmake), 'Visual Studio 18 2026', str(self.parent/'Visual Studio'))), \
                 patch.object(driver, 'command', return_value=None):
                driver.run(args, root)
        with self.assertRaisesRegex(RuntimeError, 'Missing SDL2 runtime DLL'):
            invoke_without_dll(self.args())
    def test_native_test_launchers_and_dll_staging(self):
        for name in ('build.bat', 'configure.bat', 'test.bat'):
            text = (ROOT/name).read_text()
            self.assertIn('windows-build.py', text); self.assertIn('%*', text)
        text = (ROOT/'game/CMakeLists.txt').read_text()
        self.assertIn('$<TARGET_FILE_DIR:${target}>', text)
        self.assertIn('$<TARGET_FILE:SDL2>', text)
        self.assertIn('add_custom_command(TARGET ${target} POST_BUILD', text)
        self.assertIn('add_dependencies(${target} SDL2)', text)
        self.assertNotIn('stage_colony_windows_runtime', text)
        self.assertNotIn('0.54.3', (ROOT/'scripts/validate-windows-latest.bat').read_text())

if __name__ == '__main__': unittest.main()
