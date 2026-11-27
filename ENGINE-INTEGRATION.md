# Lunar Colony 0.93.7 — external JSD engine

The game requires the supplied **JSD 3d-engine source 0.5.24** in a separate
directory. There is no in-game `engine/`
directory, legacy engine target, alternate graphics backend, OpenAL backend,
private ray-tracing pipeline, trained filter, or custom worker implementation.

## Build

From the updated game source root:

```bash
./build.sh /absolute/path/to/3d-engine
./run.sh
./build.sh /absolute/path/to/3d-engine --debug # optional
./run.sh
```

The script uses the game checkout it belongs to. Pass the engine checkout as
the first argument on every build. Quote paths with spaces. Relative engine
paths are resolved from the directory where you invoke the script. Each engine
checkout gets its own incremental CMake directory under `build/linux/engines/`.

The selected runtime is staged in the game's `build/linux/bin/` directory.
Add `--debug` to enable debug gameplay features. Set `GAME_BUILD_JOBS=4` to
limit build concurrency.

On Windows 11, with the Visual Studio 2022/2026 C++ workload, a Windows SDK, Python 3.10+
and CMake 3.21+:

```bat
build.bat --engine "C:\3D\3d-engine"
run.bat
build.bat --debug
run.bat
```

Or `./build-windows.ps1 -EnginePath 'C:\3D\3d-engine' -Configuration Release`.
Both platforms use JSD's SDL2/desktop OpenGL 3.3 implementation. The former
Direct3D/OpenAL dependencies and installation scripts are removed.

## Ownership

| Service | Implementation |
| --- | --- |
| World rendering | External `rf::gfx::Renderer`, linked as `JSD::GameEngineStatic` |
| Windows, GL context and presentation | External `rf::Platform` and its SDL2 dependency |
| Vector/matrix storage and operations | External `rf::V3` and `rf::Mat4` |
| Work scheduling and nested jobs | External `rf::Jobs`, used by the game's `GameJobs` binding |
| PCM playback, spatial mixing and audio device | External `jsd::AudioMixer` and `jsd::AudioDevice` |
| Gameplay, saves, navigation and construction | The colony game |
| Mesh/material submission, HUD font, input mapping, preference migration and cue selection | Game bindings in `game/src/runtime/` and `game/src/ui/` |

The small `colony_runtime` target binds the colony's existing scene/UI calls to
JSD. It does not contain a second engine or a fallback world-rendering pipeline.
The game-owned HUD retains its SDF font and scaling. Old graphics preference
fields are read for saved-settings compatibility, while the UI exposes supported
JSD controls. Game geometry and collision rules remain game-owned.

## Rendering and first person

This version includes all integration work from 0.55.0: JSD rendering for the
Earth campus, orbit and first-person interiors/exteriors; PBR materials, local
fixtures, cascaded sunlight shadows, AO, GI, reflections, bloom, temporal AA,
exposure and tone mapping. The original flat build plane, crater landscape and
boulders are preserved as game scenery. Exhaust and dust use JSD particle sprites.
The flat ground receives building shadows without casting a coplanar shadow.

Settings → FIRST PERSON FOV adjusts the vertical field of view from 50° to 110°
in 2° steps (default 68°), applies immediately and persists between launches.
The first-person near plane remains 0.035 m. Orbit
projection/picking remains aligned at vertical angles. First-person shadows use
a 48 m range. Depth of field and motion blur stay disabled. UI renders after
world effects. Stable material/spatial mesh batches avoid uploading unchanged
geometry, and unused GPU batches expire.

## Audio and workers

Existing synthesized sounds/music now feed JSD PCM clips and its native mixer
and device. Game cue bindings convert pitch and enclosed-space filtering to PCM
clip variants. Moving loops are rebased at their current PCM phase when position,
gain or clip changes, using the engine's public play/stop API. This engine does
not expose the old backend's source-priority or fade controls; excess sounds use
JSD's 128-voice limit and omitted loops stop at the end of the frame.

`GAME_CPU_LANES` limits the game's JSD jobs including the calling lane;
`JSD_WORKERS` supplies the engine worker preference. The custom CPU-pinning code
has been removed. On Linux, inherited affinity/`taskset` still limits available
engine CPUs. `./run.sh --cpu-info` reports the selected JSD workers.

## Runtime and tests

JSD/Jolt link statically. Source-built SDL libraries and license notices are
staged beside the game (under `bin/lib/` on Linux), using the selected optimized configuration. The debug gameplay option does not change SDL configuration. No OpenAL or Direct3D runtime is required. Saves are unchanged.

```bash
./test.sh /path/to/3d-engine
./test.sh /path/to/3d-engine --debug # includes debug-only regressions
# Optional actual GL/EGL rendering tests:
cmake -S . -B build/linux/Release -DJSD_GAME_ENGINE_DIR=/path/to/3d-engine -DGAME_BUILD_RENDER_TESTS=ON
cmake --build build/linux/Release --parallel 4
ctest --test-dir build/linux/Release -R jsd_ --output-on-failure
```

Normal desktop builds use the engine's default `JSD_HEADLESS_ONLY=OFF` and need
OpenGL/X11 development packages (`./install-deps.sh`). Set `JSD_HEADLESS_ONLY=ON`
only for offscreen EGL validation without desktop development headers.
