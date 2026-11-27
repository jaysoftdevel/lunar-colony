# Lunar Colony 0.66.0 architecture

The standalone C++20 game has one external engine dependency: JSD 3d-engine
0.5.24. Select it with `./build.sh /path/to/3d-engine`.

- `game/src/runtime/`: game-specific scene submission, SDL input/event mapping,
  audio cue conversion, JSD worker access and graphics preference migration.
- `game/include/game/runtime/`: game presentation contracts, native JSD math
  aliases, and game-owned HUD/input/cue types.
- `game/src/ui/`: HUD widgets and game interface logic.
- `game/src/systems/`, `game/src/robots/`, `game/src/world/`: colony simulation,
  resource logistics, navigation, construction, world meshes and collision rules.
- `third_party/rvo2/`: colony navigation dependency.
- `cmake/ExternalEngine.cmake`: required engine identity/version/path guards.
- `game/cmake/Runtime.cmake`: links game bindings to `JSD::GameEngineStatic`.

The original in-game engine and all its platform/rendering/audio/worker backends
are removed. Rendering, platform, math, scheduling and audio services come from
the external engine. See [ENGINE-INTEGRATION.md](ENGINE-INTEGRATION.md).
