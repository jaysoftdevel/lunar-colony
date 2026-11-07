# Build dependencies

## Shared
- CMake 3.20+
- Git for applying update packages
- C++20 compiler
- RVO2: bundled in `third_party/rvo2`; no separate install needed

## Linux
- GCC or Clang with C++20 support
- X11 development headers (`libx11-dev` on Debian/Ubuntu)
- OpenGL/Mesa development files (`libgl1-mesa-dev`, `mesa-common-dev` on Debian/Ubuntu)
- POSIX threads / `dl` (provided by the system toolchain)
- Make or Ninja

`./install-deps.sh` installs the Debian/Ubuntu package set.

## Windows 11 x64
- Visual Studio 2022 with **Desktop development with C++**
- MSVC v143 x64/x86 build tools
- Windows 11 SDK (10.0.26100 or newer recommended)
- CMake 3.20+
- Direct3D 12, DXGI, D3DCompiler, User32, GDI32 and Shell32 libraries from the Windows SDK

`install-deps.bat` can install CMake through `winget`; the Visual Studio C++ workload/SDK is best installed via Visual Studio Installer.


The advanced Linux graphics path uses desktop OpenGL 4.3 compute shaders.
The deferred raster fallback uses desktop OpenGL 3.3. OpenAL Soft supplies
audio on both platforms. The Windows dependency script installs the Win64
runtime in `build/windows/bin`. The game can run silently when an audio
device is unavailable.
