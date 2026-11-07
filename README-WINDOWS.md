# Windows 11 / Direct3D 12 build

This workspace contains the shared Linux and Windows sources for JSD Lunar Base. The Windows build uses Win32 and Direct3D 12 with the shared simulation, save format, UI, and audio mixer.

## What the Windows build uses

- Native Win32 window/event loop.
- Per-monitor DPI awareness v2.
- Win32 Raw Input for relative mouse movement in POV mode.
- Win32 display-mode enumeration and fullscreen mode switching.
- Direct3D 12 rendering.
- DXGI 1.6 high-performance adapter selection and flip-model triple buffering.
- Automatic probing of the highest GPU feature level, up to `D3D_FEATURE_LEVEL_12_2` (DirectX 12 Ultimate-class hardware).
- Hardware depth buffer and alpha blending.
- GPU geometry-shader expansion for thick mesh/wireframe lines so the blue/yellow/green construction visualization remains close to the Linux version.
- The existing persistent C++ task pool, with Windows logical-processor counting across processor groups.

The game intentionally refuses the WARP software rasterizer, matching the Linux build's existing requirement for a hardware GPU renderer.

## Required Windows build dependencies

1. **Windows 11 x64**.
2. **Visual Studio 2022 or newer** with the **Desktop development with C++** workload (MSVC v143 or newer).
3. **Windows 11 SDK 10.0.26100 or newer**. A newer stable SDK is fine. The current Windows SDK also supplies the Direct3D 12, DXGI and D3DCompiler headers/import libraries used by this project.
4. **CMake 3.20 or newer** for the project; **3.25+ is recommended** for the included presets file. Visual Studio can install CMake as an optional component, or you can install it separately.
5. A current **DirectX 12-capable GPU driver** from NVIDIA, AMD or Intel.

No third-party package manager is required. RVO2 is already included under `third_party/rvo2/`. DirectX 12 is part of Windows 11; the project links the Windows SDK/system libraries `d3d12`, `dxgi`, `d3dcompiler`, `user32`, `gdi32` and `shell32`.

### Optional development tools

- PIX for Windows, for GPU captures and timing.
- Visual Studio Graphics Diagnostics.
- A recent DirectX 12 Agility SDK can be integrated later if the renderer needs API features newer than the installed Windows runtime. It is not required for this game's current rendering path.

## Build from cmd.exe / Developer Command Prompt

The primary Windows development interface mirrors the Linux shell scripts:

```bat
build.bat
run.bat
run.bat Debug
test.bat
```

Each build writes both executables to `build/windows/bin`.

| Variant | Executable |
|---|---|
| Release | `game.exe` |
| Debug | `game-debug.exe` |

The Debug executable can read the optional construction template from the source tree or its `debug/` directory.

`dev.bat` provides the common command wrapper (`build`, `rebuild`, `run`, `test`, `clean`, `deps`). `build-windows.bat` and `build-windows.ps1` remain compatibility wrappers for the earlier Windows-only workflow.

## Manual CMake build

```powershell
cmake -S . -B build/windows -G "Visual Studio 17 2022" -A x64 -DGAME_BUILD_NAVIGATION_TESTS=ON
cmake --build build/windows --config Release --parallel
cmake --build build/windows --config Debug --parallel
ctest --test-dir build/windows -C Release --output-on-failure
ctest --test-dir build/windows -C Debug --output-on-failure
```

If you use a newer Visual Studio generator, replace the generator name accordingly.

## Runtime controls

The same performance environment variables remain available:

```text
GAME_CPU_LANES=<n>          Cap total simulation/planner CPU lanes.
GAME_ROUTE_SEARCH_LANES=<n> Cap concurrent heavy route-search jobs.
GAME_VSYNC=0|1              0/unset = uncapped present, 1 = synchronized present.
GAME_FRAME_LIMIT_FPS=<n>    Optional explicit frame limiter.
```

When VSync is off and the driver/display supports DXGI tearing, the Direct3D backend uses `DXGI_PRESENT_ALLOW_TEARING` for uncapped borderless presentation.

## Save-game compatibility

The Windows build uses the same binary `.gmsave` serializer/schema as the Linux game. Saves are placed in `savegames/` beside `game.exe`, matching the existing installed-game-relative behavior. Compatible saves can be copied between Linux and Windows. The Release executable opens campaign and normal colony saves. Stage 3 test saves require Debug.

## Platform differences / items that are not literally portable

The complete gameplay feature set is retained, but these Linux implementation details cannot exist unchanged on Windows:

- **X11 window management** is replaced by Win32 window/message APIs.
- **XRandR monitor mode switching** is replaced by `EnumDisplaySettingsExW` / `ChangeDisplaySettingsExW`.
- **GLX/OpenGL rendering** is replaced by Direct3D 12 and DXGI. The result is intended to be visually equivalent, but pixel-level rasterization/anti-aliasing can differ between OpenGL and Direct3D drivers.
- **`/proc/self/exe`** executable discovery is replaced by `GetModuleFileNameW`.
- **Linux `sched_getaffinity` CPU visibility** is replaced by Windows processor-group enumeration plus the C++ worker pool.
- **GLX swap-interval extensions** are replaced by DXGI Present/VSync/tearing control.

Windows loads the same sound mixer through the Win64 OpenAL Soft runtime. `install-deps.bat` installs that runtime beside both executables.
MSVC builds link the C++ runtime statically for the game and its compiled libraries. The Debug build stages its scenario template under `build/windows/bin/debug`.

## Direct3D 12 graphics

The lunar stages use an FP16 scene buffer for material lighting and emissive fixtures. The game applies bloom, exposure, tone mapping to SDR, and optional FXAA before drawing the interface.

**Settings → Graphics** provides the Enhanced and Classic views, material and terrain detail, lighting, and image controls. Terrain VRS is an optional performance setting for compatible GPUs. Enhanced barriers are selected automatically when the runtime and driver support them. Older adapters use legacy barriers and can fall back to the Classic view if the FP16 pipeline is unavailable.

Both build variants default to the Direct3D 12 runtime supplied by Windows.
To use [DirectX 12 Agility SDK 1.620.1](https://www.nuget.org/packages/Microsoft.Direct3D.D3D12/1.620.1),
set `GAME_D3D12_AGILITY_DIR` to its extracted `build/native/bin/x64` directory
before running `build.bat`. Both executables then use SDK version 620, with
the runtime copied to the `D3D12` directory beside them.
