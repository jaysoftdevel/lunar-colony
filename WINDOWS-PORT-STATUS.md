# Windows port status — game 0.42.0

## Preserved game systems

The Windows target compiles the same game source clusters as Linux. No gameplay subsystem was intentionally removed:

- Main menu, new-game wizard, settings, credits, save/load.
- Playfield sizes and camera controls.
- POV/first-person traversal and collision rules.
- Habitat/resource structure catalog and all structure sizes.
- Corridors, walkways and utility/supply channels.
- Construction staging, costs, visual progress and delivered resource boxes.
- BR-02 builder and CR-01 carrier robots.
- Shared safety-contract coordinator, RVO2/local avoidance, route planning and multithreaded path work.
- Resource economy and storage inventory.
- Storage domes with fixed loading gate plus up to two buildable extra gates.
- Hermetic sliding pressure doors on pressure connections.
- Outdoor four-bay robot workshop, ramp/gate clearance, parking and idle return behavior.
- Game speed controls including Pause.
- Save schema 16 and older-save loading support inherited unchanged from the 0.41.8 gameplay baseline.

## Windows-specific implementation files

```text
engine/src/platform_win32.cpp   Win32 window, input, DPI, display modes, fullscreen
engine/src/renderer_d3d12.cpp   Direct3D 12 / DXGI renderer
README-WINDOWS.md               dependency and build instructions
build.bat / dev.bat             Primary Windows build/development helpers
build-windows.ps1/.bat           Compatibility wrappers
CMakePresets.json                Linux + Windows configure/build/test presets
```

Linux remains buildable through `platform_x11.cpp` and `renderer_gl.cpp`; both targets now live in one shared source tree with independent `build/linux` and `build/windows` caches.

## Validation performed for this archive

- CMake configure and complete Linux compilation succeeded after introducing the cross-platform backend selection.
- `game --version` reports `0.42.0`.
- `navigation_geometry` passed.
- `navigation_pressure_gates` passed.
- The storage-extension stress regression was started but exceeded the execution window available in the build environment; it is not marked as passed here.
- A Windows binary was not produced in this Linux environment because the Microsoft Windows SDK and Direct3D runtime are not installed here. The archive is therefore source-complete for Windows, but the Windows backend should be built and smoke-tested on an actual Windows 11 development machine before distribution.
