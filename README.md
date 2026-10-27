# Game workspace

Native C++20 prototype with a strict split between the reusable engine (`engine/`) and game implementation (`game/`).

## Local Linux build

Required tools/libraries: a C++20 compiler, CMake, X11 development headers and a working OpenGL/GLX runtime.

Debian/Ubuntu example:

```bash
sudo apt install build-essential cmake libx11-dev libgl1-mesa-dev
./build.sh
./build/bin/game
```

`build.sh` always deletes the previous `build/` directory first and produces the executable only under `build/bin/`. No prebuilt executable is stored in the project workspace.

## Current controls

- Mouse wheel: zoom relative to the current view center.
- Middle mouse drag left/right: rotate around the current view center.
- Middle mouse drag up/down: tilt from 0 to 90 degrees.
- Right mouse drag: pan relative to camera orientation and zoom.
- Left click: menus and build-object selection.
- Escape: return to the main menu.

The game starts fullscreen. The bottom build palette is data-driven through `BuildCatalog`, and the top status bar receives its prototype values from `ColonyStatus` so both can be replaced by later simulation systems without changing the renderer.

## Structure modules
Crew-accessible habitats and corridors share a common structural access contract. Every habitat keeps exterior and interior geometry in separate modules, while its module registration declares the authoritative POV deck height and access behavior. Corridor shell and walking-deck generation follow the same outer/inner split.
