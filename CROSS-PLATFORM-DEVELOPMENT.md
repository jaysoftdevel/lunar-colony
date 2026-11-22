# Shared Linux and Windows development — 0.63.0

Both platforms use the same game sources and external JSD 0.5.14 engine services.

| Action | Linux | Windows 11 |
| --- | --- | --- |
| Build Release and Debug | ./build.sh /path/to/3d-engine | build.bat --engine PATH |
| Configure | ./configure.sh /path/to/3d-engine | configure.bat --engine PATH |
| Run Release | ./run.sh | run.bat |
| Run Debug | ./run.sh Debug | run.bat Debug |
| Run tests | ./test.sh /path/to/3d-engine | Set JSD_GAME_ENGINE_DIR, then test.bat |
| Clean | ./clean.sh | clean.bat |

The engine source path is cached after configuration. CMake validates identity,
version and engine-root changes. The game source root has no engine/ directory.
JSD/Jolt link statically; SDL is staged app-local with licenses. Audio uses JSD.
Build dependencies and details are in ENGINE-INTEGRATION.md and DEPENDENCIES.md.
The launchers reject binaries whose versions do not match the source VERSION.
