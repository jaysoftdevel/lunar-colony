# Build dependencies — 0.66.0

CMake 3.21+, Python 3.10+, a C++20 compiler, and the external JSD 3d-engine 0.5.16
source tree. That engine supplies source-built SDL2/Jolt and its runtime services.
RVO2 remains bundled with the game. No downloads occur during CMake configuration.

Linux: GCC/Clang, Make/Ninja, X11 and OpenGL development headers. Use
`./install-deps.sh` on Debian/Ubuntu. Audio uses the engine's SDL mixer/device.

Windows 11 x64: Visual Studio 2022's Desktop development with C++ workload, MSVC
v143, Windows SDK, and a desktop OpenGL 3.3 GPU driver. SDL DLLs are staged beside
the game by the build. OpenAL, Direct3D 12 and the Agility SDK are not dependencies.

Build: `./build.sh /path/to/3d-engine` or `build.bat --engine PATH`.
