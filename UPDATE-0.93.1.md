# Version 0.93.1 — complete build cleanup

Run `./clean.sh` from any working directory by specifying its path. Stop the game and all builds first. It removes the entire workspace-local `build/` tree, including Linux and Windows configurations, legacy Debug, staged executables and libraries, CMake caches, object files, generated test binaries and game-local external engine build artifacts. No CMake invocation is needed. Repeated calls are safe when the build tree is absent.

The script validates the workspace, refuses a symlinked build root and protects unexpected `.gmsave`, `settings.cfg` and `graphics.cfg` files inside `build/` before deleting anything. Move any listed player files to a safe location and rerun. Identical staged debug templates and generated regression files in known `test-saves` locations are disposable. Root savegames, source assets, debug template sources, external engine source/builds and shared runtime shader caches are not removed. Custom out-of-tree build directories and `clean.bat` are outside the scope of this change.

The CMake cache is removed, including its remembered engine path. Build again with `./build.sh ../3d-engine` or `./build.sh ../3d-engine --debug`.

No gameplay, engine version or save schema changes.
