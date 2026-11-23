# Lunar Colony 0.69.1 / JSD 0.5.19 — Windows synchronization

Requires game source **0.69.0** and separate JSD engine source **0.5.18**.
Applies to **both Windows 11 x64 and Linux** using identical gameplay/rendering
C++ code. Keeps engine outside game and preserves current save schema.

- Aligns JSD engine project identity, C++ API version constants and runtime
  version with game CMake + Windows build-driver version guard.
- Stages the source-built SDL2 DLL using per-executable POST_BUILD rules so
  Visual Studio uses the correct DLL for each configuration. Includes game,
  Debug game, tests and the late-created sky regression target.
- Gives MSVC the same modern C++ settings as the engine (/bigobj, /utf-8,
  /Zc:__cplusplus) for large game compilation clusters.
- Fails Windows verification if the application DLL is not beside its EXE.
- Adds a cross-platform source/Windows synchronization validator.
- Retains all gameplay and engine materials introduced through v0.69.0:
  gate frames, reserved station clearance, tracking photovoltaic material,
  solar-filter glazing, detailed displays and FPS controls.

Windows build (in a Developer Command Prompt with VS C++ workload):

```bat
apply.bat "C:\3D\lunar-colony" "C:\3D\3d-engine"
cd /d C:\3D\lunar-colony
py -3 scripts\check-engine-sync.py --engine "C:\3D\3d-engine"
build.bat --engine "C:\3D\3d-engine"
test.bat --engine "C:\3D\3d-engine"
run.bat
```

Linux: `./build.sh /absolute/path/to/3d-engine` after applying.

## Validation and limitations

The update is guarded by exact per-file hashes and two-root project versions.
Test the Microsoft Visual Studio x64 Release/Debug builds locally: no native
Windows compiler is available in the packaging environment. The source-level
synchronizer and mocked Windows driver tests are portable, but are not a
substitute for compiling, linking or running the game on Windows.
