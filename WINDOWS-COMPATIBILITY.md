# Windows compatibility — game 0.61.1 / engine 0.5.12

The game and external engine share the Windows x64 toolchain, SDL2, Jolt and
OpenGL renderer. The new procedural textures, station interiors and saved FOV
control remain in the shared code path. No in-game engine is restored.

## Changes

- Build/configure/test entry points use the engine's CMake/Visual Studio discovery
  rather than a hard-coded generator. Paths are passed as argument arrays.
- Both configurations build, then their binary versions are checked. Existing
  source/engine/architecture/generator/Visual Studio cache mismatches are refused.
- SDL2.dll or SDL2d.dll is staged beside each executable, including tests in
  separate output folders. An always-run staging dependency also repairs stale
  or missing DLLs on incremental builds. Public engine targets propagate NOMINMAX,
  WIN32_LEAN_AND_MEAN and SDL_MAIN_HANDLED to consumers.
- SDL's UTF-8 executable location is converted without an ANSI-codepage roundtrip.
  Preferences use Windows wide environment paths. Saves retain native filesystem
  paths internally and UTF-8 paths in the save list; replacements use MoveFileExW.
- Workspace save discovery checks the external-engine CMake binding instead of
  the deleted engine/ directory. Earlier bin/savegames folders still migrate.
- Mesh/clip cache keys use typed filesystem paths, sizes and timestamps. This
  avoids ambiguous integer formatting on standard libraries with wide file clocks.

## Performed validation

| Check | Result |
| --- | --- |
| Windows x64 Release game, engine archive and engine DLL | Cross-build passed |
| Windows x64 Debug game, engine archive and engine DLL | Cross-build passed |
| Windows navigation/UI/GL regression executables | Compiled and linked in both configurations |
| PE imports and SDL DLL hashes beside all test executables | Correct configuration and identical staged DLL verified |
| Linux Release regressions, including new filesystem and build-driver tests | 19/19 passed |
| External engine portable tooling regressions | 46/46 passed |

Windows cross-builds used Clang 22.1.8/MinGW through Zig 0.17.1, with bundled
SDL/Jolt source and the actual _WIN32 implementation. The output was inspected
as PE32+ x86-64. Linux runtime checks used GCC 12 and Mesa offscreen EGL. The new
path test covers root discovery, Unicode preference/save paths, overwrite,
save listing and loading. Native GL tests retain texture/interior/FOV coverage.

A native MSVC build and execution on Windows 11 were not available. Cross-build
success establishes source/link compatibility with the tested Windows toolchain;
it does not establish native MSVC, vendor-driver, input, audio or frame-rate
behavior. To complete those checks on Windows:

```bat
test.bat --engine "C:\3D\3d-engine" --render-tests
```

This builds and runs both configurations using the detected native Visual Studio
toolchain. See README-WINDOWS.md for requirements and command variants.
