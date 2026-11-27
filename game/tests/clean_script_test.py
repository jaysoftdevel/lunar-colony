#!/usr/bin/env python3
"""Exercise clean.sh against isolated synthetic workspace trees (Linux)."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[2] / 'clean.sh'

class CleanTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='lunar clean ')
        self.addCleanup(self.temp.cleanup)
        self.parent = pathlib.Path(self.temp.name)
        self.root = self.parent / 'game workspace'
        self.root.mkdir()
        shutil.copy2(SCRIPT, self.root / 'clean.sh')
        for path in ('VERSION', 'game/CMakeLists.txt', 'cmake/ExternalEngine.cmake'):
            self.write(path, 'marker')
        self.write('savegames/colony.gmsave', 'player save')
        self.write('game/debug-templates/0-54-0-debug-start.gmsave', 'template')
        self.write('settings.cfg', 'settings')
        self.engine = self.parent / '3d-engine'
        self.engine.mkdir()
        (self.engine / 'library.a').write_text('engine')

    def write(self, rel, contents='generated'):
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(contents)
        return p

    def clean(self, *args):
        return subprocess.run(['bash', str(self.root / 'clean.sh'), *args],
                              cwd=self.parent, capture_output=True, text=True)

    def test_all_variants(self):
        for variant in ('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel'):
            self.write(f'build/linux/{variant}/CMakeCache.txt')
            self.write(f'build/linux/{variant}/external-jsd/lib/libjsd.a')
            self.write(f'build/windows/{variant}/game.pdb')
        for path in ('build/linux/bin/game', 'build/linux/bin/game-debug',
                     'build/windows/bin/game.exe', 'build/windows/bin/game-debug.exe',
                     'build/windows/bin/SDL2.dll', 'build/linux/.runtime-stage.lock',
                     'build/linux/Release/game/test-saves/startup/temporary.gmsave'):
            self.write(path)
        self.write('build/linux/bin/debug/0-54-0-debug-start.gmsave', 'template')
        (self.root/'build'/'linked-engine').symlink_to(self.engine, target_is_directory=True)
        result = self.clean()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.root/'build').exists())
        self.assertEqual((self.root/'savegames/colony.gmsave').read_text(), 'player save')
        self.assertEqual((self.root/'settings.cfg').read_text(), 'settings')
        self.assertTrue((self.root/'game/debug-templates/0-54-0-debug-start.gmsave').is_file())
        self.assertTrue((self.engine/'library.a').is_file())
        self.assertEqual(self.clean().returncode, 0)

    def test_legacy_save_blocks_cleanup(self):
        save = self.write('build/windows/bin/savegames/old colony.gmsave', 'old save')
        artifact = self.write('build/linux/Release/game.o')
        self.assertNotEqual(self.clean().returncode, 0)
        self.assertEqual(save.read_text(), 'old save')
        self.assertTrue(artifact.exists())

    def test_modified_template_blocks_cleanup(self):
        self.write('build/linux/bin/debug/0-54-0-debug-start.gmsave', 'changed')
        self.assertNotEqual(self.clean().returncode, 0)

    def test_config_blocks_cleanup(self):
        self.write('build/linux/bin/settings.cfg', 'custom')
        self.assertNotEqual(self.clean().returncode, 0)

    def test_symlink_root_is_refused(self):
        (self.root/'build').symlink_to(self.engine, target_is_directory=True)
        self.assertNotEqual(self.clean().returncode, 0)
        self.assertTrue((self.engine/'library.a').is_file())

    def test_wrong_workspace_and_arguments(self):
        artifact = self.write('build/linux/Release/game.o')
        self.assertNotEqual(self.clean('--unknown').returncode, 0)
        (self.root/'VERSION').unlink()
        self.assertNotEqual(self.clean().returncode, 0)
        self.assertTrue(artifact.exists())

if __name__ == '__main__':
    unittest.main(verbosity=2)
