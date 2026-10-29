# v0.40.0 Implementation Clusters

This file is the context map for future changes. Start from the smallest build target(s) below and load additional clusters only when an interface crosses the requested change. The target names are real CMake OBJECT targets in `game/cmake/GameClusters.cmake`.

| Cluster target | Main responsibility | Primary files / directories | Load together with |
|---|---|---|---|
| `game_cluster_pathfinding` | Geometry-only robot path search (corridor A*/Theta*-style simplification) | `game/include/game/robots/path_planner.hpp`, `game/src/robots/path_planner.cpp` | `game_cluster_robots` only when changing fleet traffic/execution |
| `game_cluster_robots` | BR-02/CR-01 autonomy, logistics, traffic prediction, collision-safe route execution, construction work sequencing | `game/include/game/systems/builder_robot.hpp`, `game/src/systems/builder_robot.cpp` | pathfinding for route search; construction for resource/step semantics; rendering only for visuals |
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

## Robot/pathfinding boundary

`game_cluster_pathfinding` has no construction or fleet-policy knowledge. It receives four callbacks from `game_cluster_robots`: floor height, point occupancy, segment validity and traffic cost. This is intentional. Path search can be optimized/replaced independently while robot logistics and construction task state remain untouched. Broad/full searches may pre-sample independent grid occupancy/traffic across up to half of the available logical CPUs; narrow routine searches remain single-threaded to minimize scheduling overhead.

The robot integration uses two search levels:

1. **Normal route search** — bounded corridor widths (4.5 / 9 / 18 m), stationary fleet hulls included as temporary obstacles, low deterministic CPU cost.
2. **Full route resolution** — if a robot cannot resolve or make route progress for 2.0 s, local route state is discarded and a full-width world-geometry search is requested. Temporary rover positions are removed from hard search occupancy during this recovery search; actual hull collisions remain enforced by the continuous traffic/collision execution layer.

Successful physical progress resets the 2 s route-resolution watchdog. Merely publishing another local route does not reset it, preventing endless local replan loops.

## Future-request context examples

- “robots get stuck / path is bad” -> load `game_cluster_pathfinding` + `game_cluster_robots` only.
- “carrier/builder resource workflow” -> load `game_cluster_robots` + `game_cluster_construction`.
- “change building appearance” -> load `game_cluster_rendering` + the relevant habitat/resource/network mesh cluster.
- “corridor placement/routing” -> load `game_cluster_networks` + `game_cluster_construction`.
- “POV movement inside structures” -> load `game_cluster_interaction` + relevant world cluster.
- “HUD/menu/settings” -> load `game_cluster_ui` + `game_cluster_app`.
- “save a new field” -> load the owning cluster + `game_cluster_simulation`.
