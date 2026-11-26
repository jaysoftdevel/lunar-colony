# Platform matrix — 0.92.0

| Capability | Linux | Windows 11 |
| --- | --- | --- |
| Gameplay, saves, collision and logistics | Shared C++20 | Shared C++20 |
| Engine | External JSD 0.5.22 | External JSD 0.5.22 |
| World rendering | JSD desktop OpenGL 3.3 | JSD desktop OpenGL 3.3 |
| Window/input | JSD Platform / SDL2 | JSD Platform / SDL2 |
| Audio | JSD mixer/device / SDL2 | JSD mixer/device / SDL2 |
| Workers | JSD Jobs | JSD Jobs |
| Game (with or without debug features) | build/linux/bin/game | build/windows/bin/game.exe |

Use the same source contents on both platforms. See ENGINE-INTEGRATION.md for
build commands and current validation limits.
