# Cross-platform development

This repository is one game implementation with two native platform backends:

| Host | Window/input | Renderer | Build directory |
|---|---|---|---|
| Linux | X11 | OpenGL | `build/linux` |
| Windows 11 | Win32 | Direct3D 12 | `build/windows` |

Gameplay, simulation, construction, navigation/RVO2, save/load, UI logic and tests are shared source. Only the low-level platform and renderer implementation is selected by CMake.

## Mirrored commands

The shell and batch interfaces intentionally use the same verbs and configuration names.

| Action | Linux | Windows |
|---|---|---|
| Install/check dependencies | `./dev.sh deps` | `dev.bat deps` |
| Incremental Release build | `./dev.sh build Release` | `dev.bat build Release` |
| Incremental Debug build | `./dev.sh build Debug` | `dev.bat build Debug` |
| Clean rebuild | `./dev.sh rebuild Release` | `dev.bat rebuild Release` |
| Run | `./dev.sh run` | `dev.bat run` |
| Build + regressions | `./dev.sh test Release` | `dev.bat test Release` |
| Remove platform build tree | `./dev.sh clean` | `dev.bat clean` |

The direct paired scripts (`build.*`, `run.*`, `test.*`, `clean.*`, `configure.*`) can be used instead of `dev.*`.

## Parallel development model

The two platforms use different build directories. A checkout can therefore be shared or synchronized between Linux and Windows without conflicting CMake caches. Do not commit `build/`.

Shared code belongs in `game/`, `engine/src/ui.cpp`, `engine/src/task_pool.cpp`, and common engine headers. Platform-specific behavior should remain in:

- `engine/src/platform_x11.cpp`
- `engine/src/renderer_gl.cpp`
- `engine/src/platform_win32.cpp`
- `engine/src/renderer_d3d12.cpp`

When changing shared gameplay behavior, run `test.sh`/`test.bat` on both hosts before baselining the change. When changing renderer or input behavior, validate the corresponding native backend plus at least the shared regression suite on the other platform.

## Build options

`GAME_BUILD_TESTS=ON ./build.sh Release` can enable Linux tests through the environment. `--tests` works on both direct build scripts. `GAME_BUILD_JOBS=N` caps Linux compile parallelism; Windows delegates parallelism to CMake/MSBuild.

## Save compatibility

Both platforms retain the same `.gmsave` format and executable-relative `savegames` layout. Do not introduce native path separators into serialized save data; normalize runtime paths only at the platform/file-system boundary.
