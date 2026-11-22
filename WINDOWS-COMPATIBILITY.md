# Windows compatibility — game 0.63.0 / engine 0.5.14

The same gameplay, sky shader, geometry and UI code is used on Linux and Windows.
Windows x64 Release/Debug game, static engine, shared engine and regression
executables cross-build. Correct SDL2/SDL2d DLL staging and PE imports were
inspected. Engine/toolchain discovery, cache/source/version guards, Unicode
filesystem handling and both-configuration builds from 0.61.1 are retained.

Linux runtime regression results: 24 Release and 10 targeted Debug checks pass;
46 engine portable tooling tests pass. A native MSVC build and Windows 11
execution/graphics-driver check were not available. To run native validation:

```bat
test.bat --engine "C:\3D\3d-engine" --render-tests
```

See README-WINDOWS.md for requirements and UPDATE-0.63.0.md for hangar service bays,
camera-aware drilling audio, safe ore outlets and the power-off symbol.
