# v0.40.33 Implementation Clusters

This file is the context map for future changes. Start from the smallest build target(s) below and load additional clusters only when an interface crosses the requested change. The target names are real CMake OBJECT targets in `game/cmake/GameClusters.cmake`.

| Cluster target | Main responsibility | Primary files / directories | Load together with |
|---|---|---|---|
| `game_cluster_pathfinding` | Static-geometry advisory route search plus shared motion-contract safety coordinator | `game/include/game/robots/path_planner.hpp`, `game/src/robots/path_planner.cpp`, `game/include/game/robots/safety_coordinator.hpp`, `game/src/robots/safety_coordinator.cpp` | `game_cluster_robots` for task intents/world callbacks |
| `game_cluster_robots` | BR-02/CR-01 task policy, logistics, construction work sequencing, safety-world callbacks and fleet intent submission | `game/include/game/systems/builder_robot.hpp`, `game/src/systems/builder_robot.cpp` | pathfinding/safety for authoritative locomotion; construction for resource/step semantics; rendering only for visuals |
| `game_cluster_construction` | placement/catalog, construction costs/stages, resource ownership and build commands | `game/src/features/`, construction/resource system files | world cluster for mesh semantics; robots only for autonomous delivery/building |
| `game_cluster_rendering` | world/robot visual output | `game/src/playfield/world_renderer.cpp`, `builder_robot_renderer.cpp` | UI only for overlays; robots only if robot state changes |
| `game_cluster_ui` | HUD, build bar, status/debug/selection panels | `game/src/ui/` | app/interaction if input behavior changes |
| `game_cluster_interaction` | playfield input, selection, camera and first-person control | playfield view + gameplay/world-selection/camera/FP systems | UI for controls; world for collision/access rules |
| `game_cluster_simulation` | colony status and save/load persistence | `colony_status.cpp`, `save_game.cpp` | affected state-owning cluster only |
| `game_cluster_world_core` | playfield, common mesh data, construction reveal sequence | core files under `game/src/world/` | habitat/resource/network cluster as needed |
| `game_cluster_habitats` | habitat outer/inner implementations | `game/src/world/habitats/` | world core; construction when placement/build rules change |
| `game_cluster_resources` | outdoor/resource structure implementations | `game/src/world/resources/` | world core; construction when placement/build rules change |
| `game_cluster_networks` | corridors, supply channels, walkways and routing geometry | corridor/network sources | world core; construction when build stages/costs change |
| `game_cluster_app` | top-level game state and screens | `game_app.cpp`, `game/src/screens/` | whichever cluster owns the requested feature |
| `game_engine` | X11 platform, GPU renderer, immediate UI primitives | `engine/` | only when the request is engine/platform/render-backend related |

## Robot/pathfinding/safety boundary

`game_cluster_pathfinding` contains two deliberately separate layers. `path_planner.*` is a static-geometry coarse route finder: it receives floor/point/segment callbacks and may cache a route toward the task goal, but that route is never a movement authorization. `safety_coordinator.*` is the authoritative fleet-motion layer.

High-level robot code submits one destination/work intent per rover. The coordinator then:

1. regenerates short dynamically feasible proposals on a 0.50 s horizon every 0.05 s;
2. builds a 10 ms `MotionContract` for each candidate, including conservative braking branches and terminal stopping holds;
3. checks continuous robot/environment and robot/robot separation in absolute time;
4. reserves required passage cells and arbitrates all robots together using passage ownership, wait age and deterministic IDs;
5. atomically commits the common simulation step only after the full fleet sweep is valid; otherwise the old certified brake branch remains authoritative;
6. resolves persistent blocking with waiting, wait-for-cycle detection, certified pull-out/retreat goals and static-route replanning.

No job routine may directly change a robot pose, insert an avoidance steering impulse, teleport a stuck rover, or post-correct overlaps. Scene changes that can become robot obstacles must be checked against active contracts before commit.

## Future-request context examples

- “robots get stuck / path is bad” -> load `game_cluster_pathfinding` + `game_cluster_robots` only.
- “carrier/builder resource workflow” -> load `game_cluster_robots` + `game_cluster_construction`.
- “change building appearance” -> load `game_cluster_rendering` + the relevant habitat/resource/network mesh cluster.
- “corridor placement/routing” -> load `game_cluster_networks` + `game_cluster_construction`.
- “POV movement inside structures” -> load `game_cluster_interaction` + relevant world cluster.
- “HUD/menu/settings” -> load `game_cluster_ui` + `game_cluster_app`.
- “save a new field” -> load the owning cluster + `game_cluster_simulation`.

## v0.40.33 debug-start template

- `game/src/screens/new_game.cpp`: debug toggle, 64x64 template lock, paused startup.
- `game/src/systems/save_game.cpp`: construction-only debug template parser/importer.
- `game/debug-templates/0-40-32-debug-start.gmsave`: bundled source construction layout.
- `game/CMakeLists.txt`: copies the template beside the built executable under `bin/debug/`.

## v0.40.43 GPU / CPU runtime cluster

For GPU backend, GLX/OpenGL hardware validation, or renderer diagnostics, start with `engine/src/renderer_gl.cpp`, `engine/src/platform_x11.cpp`, and `engine/include/engine/engine.hpp` (`game_engine`). For CPU worker-count policy or task dispatch, use `engine/src/task_pool.cpp`. Rover multi-core use is limited to proposal/route preparation in `game/src/robots/safety_coordinator.cpp` (`game_cluster_pathfinding`); do not parallelize reservation arbitration or world mutation without preserving the shared safety epoch.
