# Lunar Base 0.92.0 — unified game build

Baseline: 0.91.0. JSD engine: 0.5.22, unchanged. Save schema: 57, unchanged.

- Linux and Windows build only one game executable: `build/linux/bin/game` or `build/windows/bin/game.exe`.
- Default build is optimized Release without debug gameplay tools. `--debug` passes `GAME_DEBUG_FEATURES=ON` to CMake and compiles the existing debug gameplay features and debug-frame source into the same target. Omitting the flag explicitly resets the option to OFF, even in an existing CMake cache. Release optimization and normal SDL runtime remain in effect. RelWithDebInfo and MinSizeRel remain optional configurations; separate Debug compiler configuration is retired.
- `run.sh` / `run.bat` always launch the current game; `--build-mode` reports `Debug` or `Release` (feature policy, not optimization level). Legacy launch selectors are rejected with instructions. `--version` remains unchanged. Rebuild without `--debug` to disable the tools; debug-only saves remain protected.
- `configure`, `test`, CMake presets, Windows PowerShell (`-Debug`) and build summaries follow the same single-target model. Debug-only regression registration and template staging follow the feature option. Old `game-debug` binaries and the obsolete Windows debug SDL DLL are removed after successful builds; normal mode removes the known staged debug template, never the source fixture or player saves. Old build caches are not recursively deleted.

## Usage

Linux: `./build.sh ../3d-engine --debug` then `./run.sh`. Normal build: `./build.sh ../3d-engine`.
Windows: `build.bat --engine ..\3d-engine --debug` then `run.bat`. Normal build: omit `--debug`.

For CMake users, configure Release with `-DGAME_DEBUG_FEATURES=ON` or `OFF`. For tests, pass `--tests` or use `test.sh` / `test.bat` (add `--debug` for debug-only cases).
