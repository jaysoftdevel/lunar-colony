# Windows compatibility — game 0.66.0 / engine 0.5.16

The same gameplay, corridor pressure geometry, sky shader and UI code is used on Linux and Windows.
Windows x64 Release/Debug game, static engine, shared engine and regression
executables cross-build. Correct SDL2/SDL2d DLL staging and PE imports were
inspected. Engine/toolchain discovery, cache/source/version guards, Unicode
filesystem handling and both-configuration builds from 0.61.1 are retained.

Linux runtime regression results: 27 Release and 15 targeted Debug checks pass;
46 engine portable tooling tests pass. A native MSVC build and Windows 11
execution/graphics-driver check were not available. To run native validation:

```bat
test.bat --engine "C:\3D\3d-engine" --render-tests
```

See README-WINDOWS.md for requirements and UPDATE-0.66.0.md for sealed corridor joins,
conformal building/storage collars and stable sealed atmosphere.
