# Architecture

## Engine layer
`engine/` owns Linux/X11 windowing, raw input, GPU renderer abstraction, OpenGL backend, math primitives, and immediate UI drawing. The game only sees the public interfaces in `engine/include/engine`.

The `Renderer` interface is intentionally backend-agnostic. A Vulkan renderer can be added later without changing the game-state, world, or camera APIs.

## Game layer
`game/` owns all game-specific behavior.

- `GameApp`: high-level screen/state routing.
- `world/Playfield`: generated playfield mesh data only.
- `systems/CameraController`: zoom, 0-90 degree tilt, and zoom-relative panning.
- `features/`: reserved for future gameplay feature modules such as placement, selection, construction, economy, agents, pathfinding, and simulation.

## Extension direction
Keep new reusable rendering/platform facilities in `engine/`. Keep game rules and simulation in `game/`. Avoid exposing X11/OpenGL objects to game code.
