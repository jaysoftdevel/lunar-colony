# Validation status

Validated in the available Linux build environment:

- CMake detects `linux` / `opengl` backend correctly.
- Full source tree compiles successfully with navigation tests enabled.
- `navigation_geometry` passes.
- `navigation_pressure_gates` passes.
- `CMakePresets.json` is accepted by CMake and exposes Linux presets on a Linux host.
- All shell helper scripts pass `bash -n` syntax checks.
- Every primary development operation has a paired `.sh` and `.bat` entry point.

Windows compilation cannot be executed in this Linux environment because MSVC, the Windows SDK and a Direct3D runtime are not present. The Windows backend remains the same Win32/Direct3D 12 implementation from the preceding port, while the build paths and command wrappers were unified around `build/windows`.
