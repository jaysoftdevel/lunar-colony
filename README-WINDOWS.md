# Windows 11 x64 — Lunar Colony 0.66.0

Use the Visual Studio Desktop development with C++ workload and a Windows SDK,
Python 3.10+, and CMake 3.21+ (standalone or the Visual Studio CMake component).
The build driver selects a compatible installed Visual Studio 2022/2026 and
CMake pair. SDL2 and Jolt compile from the external JSD **0.5.16 source**.
Both platforms use the same JSD desktop OpenGL 3.3 renderer, including the new
materials, interiors and FOV control. A working vendor OpenGL driver is required.

```bat
build.bat --engine "C:\3D projects\3d-engine"
run.bat
run.bat Debug
```

The positional engine path is also accepted. A later build reuses the cached
engine path. Both game.exe and game-debug.exe are built in build/windows/bin.
The matching source-built SDL DLL is copied beside each game and test executable;
Debug and Release use their own SDL runtime names. Incremental builds refresh
these DLLs even when the executable does not need relinking. The driver checks both
binary versions before reporting success. Rebuild other engine SDK consumers
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
build.bat Debug --engine "C:\3D projects\3d-engine" --tests --jobs 6
test.bat --engine "C:\3D projects\3d-engine"
rem Optional native GL rendering checks (requires a working OpenGL driver):
test.bat --engine "C:\3D projects\3d-engine" --render-tests
```

For PowerShell:

```powershell
./build-windows.ps1 -EnginePath 'C:\3D projects\3d-engine' -Configuration Release -Tests
```

The supplied update contains source changes only. See WINDOWS-COMPATIBILITY.md
for performed checks and the distinction between cross-build and native runtime
validation. No Direct3D or OpenAL build dependency is used.
