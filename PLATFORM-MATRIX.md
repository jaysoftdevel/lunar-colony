# Platform matrix

| Capability | Linux | Windows 11 | Shared? |
|---|---|---|---|
| Gameplay/simulation | C++20 | C++20 | Yes |
| Save format | `.gmsave` schema 16 | `.gmsave` schema 16 | Yes |
| Robot/pathfinding/RVO2 | Native | Native | Yes |
| Construction/world/UI logic | Native | Native | Yes |
| Task pool / multithreading | `std::thread`/system threads | `std::thread`/system threads | Yes |
| Window/input | X11 | Win32 + Raw Input | Platform layer |
| Renderer | OpenGL/GLX | Direct3D 12/DXGI | Platform layer |
| Executable | `build/linux/bin/game` | `build/windows/bin/game.exe` | No |
| Build helper | `.sh` | `.bat` | Mirrored interface |

## Compatibility rule

Shared gameplay and serialized data must not depend on X11, Win32, OpenGL or Direct3D types. Native handles and renderer objects stay inside the engine platform backend. File paths written into game state must use platform-neutral logical names rather than native absolute paths.
