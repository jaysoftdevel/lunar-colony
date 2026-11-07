# Cross-platform development

This repository is one game implementation with two native platform backends:

| Host | Window/input | Renderer | Build directory |
|---|---|---|---|
| Linux | X11 | OpenGL | `build/linux` |
| Windows 11 | Win32 | Direct3D 12 | `build/windows` |

Gameplay, simulation, construction, navigation/RVO2, save/load, UI logic and tests are shared source. Only the low-level platform and renderer implementation is selected by CMake.

## Mirrored commands

The shell and batch commands build the two variants from the same sources. Release includes the campaign and playable stages. Debug also enables the developer frame, layout switch, robot orders, template starts, and Stage 3 scenarios. Both executables share their platform's `bin/savegames` folder. Stage 3 test saves open in Debug.

| Action | Linux | Windows |
|---|---|---|
| Install/check dependencies | `./dev.sh deps` | `dev.bat deps` |
| Build both variants | `./dev.sh build` | `dev.bat build` |
| Clean rebuild of both variants | `./dev.sh rebuild` | `dev.bat rebuild` |
| Run Release | `./dev.sh run` | `dev.bat run` |
| Run Debug | `./dev.sh run Debug` | `dev.bat run Debug` |
| Build and run regressions | `./dev.sh test` | `dev.bat test` |
| Clean generated build outputs | `./dev.sh clean` | `dev.bat clean` |

The direct paired scripts (`build.*`, `run.*`, `test.*`, `clean.*`, `configure.*`) can be used instead of `dev.*`.

## Parallel development model

Linux builds in `build/linux/Release` and `build/linux/Debug`. Windows uses the corresponding configurations in `build/windows`. Each platform writes `game` and `game-debug` into its `bin` directory, with an `.exe` extension on Windows.

| Runtime file | Linux | Windows |
|---|---|---|
| Executables | `build/linux/bin` | `build/windows/bin` |
| OpenAL Soft | `bin/lib/libopenal.so.1` when installed | `bin/OpenAL32.dll` from `install-deps.bat` |
| Scenario template for Debug | `bin/debug/0-40-32-debug-start.gmsave` | `bin/debug/0-40-32-debug-start.gmsave` |
| Optional Agility runtime | — | `bin/D3D12` |
| Player saves | `bin/savegames` | `bin/savegames` |

The game can run from its platform's `bin` directory without the source tree. Keep its runtime subdirectories with the executables. The host still provides its graphics driver, window system, and OS libraries. Graphics preferences use the user's existing configuration directory.

The static engine and RVO2 archives are collected in the CMake build tree's `lib` directory. CMake builds the game clusters as object targets and links them into each executable. The archives and objects are inputs to the build.

Shared code belongs in `game/`, `engine/src/ui.cpp`, `engine/src/task_pool.cpp`, and common engine headers. Platform-specific behavior should remain in:

- `engine/src/platform_x11.cpp`
- `engine/src/renderer_gl.cpp`
- `engine/src/platform_win32.cpp`
- `engine/src/renderer_d3d12.cpp`

When changing shared gameplay behavior, run `test.sh`/`test.bat` on both hosts before baselining the change. When changing renderer or input behavior, validate the corresponding native backend plus at least the shared regression suite on the other platform.

## Build options

`GAME_BUILD_TESTS=ON` or `--tests` enables tests on either platform. The developer regression suite runs in Debug and the build policy is tested in both configurations. `GAME_BUILD_JOBS=N` caps compile parallelism.

## Save compatibility

Both platforms retain the same `.gmsave` format and executable-relative `savegames` layout. Do not introduce native path separators into serialized save data; normalize runtime paths only at the platform/file-system boundary.
