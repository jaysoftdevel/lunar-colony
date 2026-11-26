# Windows 11 x64 — Lunar Colony 0.92.0

Use the Visual Studio Desktop development with C++ workload and a Windows SDK,
Python 3.10+, and CMake 3.21+ (standalone or the Visual Studio CMake component).
The build driver selects a compatible installed Visual Studio 2022/2026 and
CMake pair. SDL2 and Jolt compile from the external JSD **0.5.22 source**.
Both platforms use the same JSD desktop OpenGL 3.3 renderer, including the new
materials, interiors and FOV control. A working vendor OpenGL driver is required.

```bat
build.bat --engine "C:\3D projects\3d-engine"
run.bat
build.bat --debug
run.bat
```

The positional engine path is also accepted. A later build reuses the cached
engine path. Only game.exe is built in build/windows/bin. The optional --debug flag includes debug gameplay features in the same optimized executable. Rebuild without it to disable them. The source-built SDL2.dll is staged beside the game and tests. The driver checks the game version and compiled feature mode before reporting success. Rebuild other engine SDK consumers
when updating the engine.

The game and engine share the selected x64 toolchain and DLL C runtime settings.
Wrong project/version, source root, engine root, generator, Visual Studio
installation or architecture in an existing cache is rejected. Remove only
build/windows when intentionally changing the toolchain or engine location.
Build commands preserve arguments containing spaces and Unicode characters.
Preferences, game-root save discovery, save/load and save replacement support
Windows Unicode paths; saves from the former bin/savegames locations migrate
without overwriting another save.

```bat
configure.bat --engine "C:\3D projects\3d-engine"
build.bat --debug --engine "C:\3D projects\3d-engine" --tests --jobs 6
test.bat --engine "C:\3D projects\3d-engine"
rem Optional native GL rendering checks (requires a working OpenGL driver):
test.bat --engine "C:\3D projects\3d-engine" --render-tests
```

For PowerShell:

```powershell
./build-windows.ps1 -EnginePath 'C:\3D projects\3d-engine' -Configuration Release -Debug -Tests
```

The supplied update contains source changes only. See WINDOWS-COMPATIBILITY.md
for performed checks and the distinction between cross-build and native runtime
validation. No Direct3D or OpenAL build dependency is used.
