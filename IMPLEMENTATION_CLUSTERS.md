# v0.42.0 Implementation Clusters

This file is the context map for future changes. Start from the smallest build target(s) below and load additional clusters only when an interface crosses the requested change. The target names are real CMake OBJECT targets in `game/cmake/GameClusters.cmake`.

| Cluster target | Main responsibility | Primary files / directories | Load together with |
|---|---|---|---|
| `game_cluster_pathfinding` | Static-geometry advisory route search plus shared motion-contract safety coordinator | `game/include/game/robots/path_planner.hpp`, `game/src/robots/path_planner.cpp`, `game/src/robots/incremental_path_planner.cpp`, `game/include/game/robots/safety_coordinator.hpp`, `game/src/robots/safety_coordinator.cpp` | `game_cluster_robots` for task intents/world callbacks |
| `game_cluster_robots` | BR-02/CR-01 task policy, logistics, construction work sequencing, safety-world callbacks and fleet intent submission | `game/include/game/systems/builder_robot.hpp`, `game/src/systems/builder_robot.cpp` | pathfinding/safety for authoritative locomotion; construction for resource/step semantics; rendering only for visuals |
| `game_cluster_construction` | placement/catalog, construction costs/stages, resource ownership and build commands | `game/src/features/`, construction/resource system files | world cluster for mesh semantics; robots only for autonomous delivery/building |
| `game_cluster_rendering` | world/robot visual output | `game/src/playfield/world_renderer.cpp`, `builder_robot_renderer.cpp` | UI only for overlays; robots only if robot state changes |
| `game_cluster_ui` | HUD, build bar, status/debug/selection panels | `game/src/ui/` | app/interaction if input behavior changes |
| `game_cluster_interaction` | playfield input, selection, camera and first-person control | playfield view + gameplay/world-selection/camera/FP systems | UI for controls; world for collision/access rules |
| `game_cluster_simulation` | colony status, utility/life-support simulation, progression and save/load persistence | `colony_status.cpp`, `supply_simulation.cpp`, `game_progress.cpp`, `save_game.cpp` | affected state-owning cluster only |
| `game_cluster_world_core` | playfield, common mesh data, construction reveal sequence | core files under `game/src/world/` | habitat/resource/network cluster as needed |
| `game_cluster_habitats` | habitat outer/inner implementations | `game/src/world/habitats/` | world core; construction when placement/build rules change |
| `game_cluster_resources` | outdoor/resource structure implementations | `game/src/world/resources/` | world core; construction when placement/build rules change |
| `game_cluster_networks` | corridors, supply channels, walkways and routing geometry | corridor/network sources | world core; construction when build stages/costs change |
| `game_cluster_app` | top-level game state and screens | `game_app.cpp`, `game/src/screens/` | whichever cluster owns the requested feature |
| `game_engine` | X11 platform, GPU renderer, immediate UI primitives | `engine/` | only when the request is engine/platform/render-backend related |

## Robot/pathfinding/safety boundary

`game_cluster_pathfinding` contains deliberately separate global-routing and motion-authority layers. `path_planner.cpp` retains A*/dogleg fallback routing, while `incremental_path_planner.cpp` provides the persistent D* Lite static-world planner with soft clearance costs. Both only suggest waypoints. `safety_coordinator.*` is the authoritative fleet-motion layer.

High-level robot code submits one destination/work intent per rover. The coordinator then:

1. regenerates short dynamically feasible proposals on a 1.00 s horizon every 0.10 s;
2. builds a 10 ms `MotionContract` for each candidate, including conservative braking branches and terminal stopping holds;
3. checks continuous robot/environment and robot/robot separation in absolute time;
4. uses cell ownership as an ordering hint and arbitrates exact motion/braking/hold separation using task priority, wait age and deterministic IDs;
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

For GPU backend, GLX/OpenGL hardware validation, or renderer diagnostics, start with `engine/src/renderer_gl.cpp`, `engine/src/platform_x11.cpp`, and `engine/include/engine/engine.hpp` (`game_engine`). For CPU worker-count policy or task dispatch, use `engine/src/task_pool.cpp`. Since v0.40.50 the runtime pool uses all logical CPUs visible to the process. Rover multi-core work includes world-revision route validation, D* repair/full route preparation and local proposal preview in `game/src/robots/safety_coordinator.cpp` (`game_cluster_pathfinding`). Heavy route jobs are dynamically allocated first, with up to one heavy search per CPU lane. Do not parallelize reservation arbitration, pose commit or world/resource mutation without preserving the shared safety epoch.

## v0.41.1 shared access and crowd advice

The pathfinding cluster links vendored `third_party/rvo2` for velocity advice; it retains sole contract authority over actual movement. `game/world/mesh_portals.hpp` supplies exact doorway and circulation cuts. Habitat size normalization, storage port placement, corridor collars and the POV floor resolver share model-declared thresholds. The optional `navigation_regression` target links the same game objects and runs without an X11 window.

## v0.41.2 host-throughput runtime defaults

- `engine/src/task_pool.cpp` keeps host-wide CPU lane usage (`GAME_CPU_LANES` remains the diagnostic cap).
- `engine/src/renderer_gl.cpp` now explicitly requests swap interval 0 by default so GPU present is uncapped; `GAME_VSYNC=1` enables synchronized present.
- `engine/src/platform_x11.cpp` removes the unconditional per-frame 1 ms sleep and runs uncapped by default; `GAME_FRAME_LIMIT_FPS=<n>` provides an explicit cap.

## v0.41.3 head-on right-side evasion

- `game/src/robots/safety_coordinator.cpp` adds explicit head-on closing detection and applies a per-rover right-turn evasion bias from each robot's local frame.
- This behavior is local steering guidance only; the existing shared 10 ms motion-contract safety arbitration remains authoritative.

## v0.41.4 planner peak-load smoothing

- `game/src/robots/safety_coordinator.cpp` adds deterministic per-rover planner phase offsets so large fleets do not synchronize route/proposal refreshes in one frame.
- Heavy global route-search concurrency now uses a bounded default lane count to avoid all-core bursts; `GAME_ROUTE_SEARCH_LANES` provides an explicit override.
- Planner intake per safety tick is budgeted, prioritizing hard/blocked requests first and deferring lower-priority refreshes to following ticks while certified contracts remain authoritative.

## v0.41.5 close-range carrier blueprint drop-offs

- `game/src/systems/builder_robot.cpp` now allows CR-01 delivery aprons to be selected closer to incomplete habitat/resource blueprint targets.
- Own-target conservative footprint clearance is relaxed only for that incomplete target during apron selection, preserving strict clearance for completed structures and all non-target geometry.

## v0.41.6 storage gates and pressure seals

- `game/world/storage_loading_gate.hpp` is the shared fixed hatch/service/apron geometry contract for logistics, corridor routing, placement, rendering and crew collision. `PlacedResource::storage_loading_face` is persisted by save schema 14.
- `game/world/pressure_door.hpp` defines seal dimensions, animation and swept body/leaf collision. `game/src/systems/pressure_doors.cpp` (interaction cluster) owns transient openings, cargo shutter animation, legacy gate assignment and Air Lock interlocking.
- `world_renderer.cpp` draws mesh pressure leaves, seals and cargo transfer modules in normal and POV views. Gate access is independent of the crew pressure network.

## v0.41.7 outdoor workshop

- `game/world/robot_workshop.hpp` owns the four bay poses, exact wall/dock/door/curb collision, ramp/deck heights and approach reservation. `world/resources/robot_workshop.cpp` builds the corresponding native mesh in the resource cluster.
- `systems/robot_workshop.cpp` updates the automatic gate and caches placement-to-fleet reachability. `builder_robot.cpp` owns home assignment, inbound admission/queueing, idle-return requests and docked condition recovery; safety remains the only pose authority.
- Placement, corridor routing and walkway routing protect the same rotated approach rectangle. Rendering shows its outline while placing the workshop. Save schema 15 accepts the appended ResourceKind while retaining old schemas.
- `safety_coordinator.cpp` excludes stationary terminal holds from advisory planner intake, keeping budget slots available for returning robots without changing hold validation or fleet separation.

## v0.41.8 storage extensions

- `systems/storage_extensions.cpp` (interaction cluster) validates and creates at most two paid child gates per storage dome, synchronizes stable parent links and derives all gate faces/readiness. `world/resources/storage_gate.cpp` supplies the native chamber mesh in the resource cluster.
- `storage_loading_gate.hpp` shares face-specific geometry and all-face reservations across placement, routing, logistics, rendering and crew collision. `builder_robot.cpp` reuses existing resource delivery/build stages with a completed-parent dependency, exterior work bays, per-gate transfer claims and queues.
- `selection_panel.cpp` exposes ADD GATE for blueprint/completed domes, cost and upgrade progress. `save_game.cpp` schema 16 persists parent placement sequences and validates links. `world_selection.cpp` cascades dome deletion and repairs robot resource targets.
- `world_renderer.cpp` emits blue/yellow/green meshes for gates and pressure doors, clips obsolete crew-port detail furniture at cargo faces and animates independent shutters. `navigation_storage_extensions` tests the complete feature with the real game objects.
## v0.43.15 hydrogen / stellar-energy ownership

- `game_cluster_simulation`: `game/include/game/systems/solar_tracking.hpp`, `supply_simulation.cpp`, `colony_status.*` and `save_game.cpp` own the star pose, H2 mass/energy accounting, component-local dispatch and schema-20 persistence.
- `game_cluster_resources`: `world/resources/hydrogen_generator.cpp`, `resource_mesh.cpp` and `resource_builder.hpp` own the outdoor fuel-cell mesh and resource refinements.
- `game_cluster_rendering`: `world_renderer.cpp` transforms only the completed Solar Panel collector head toward `ColonyStatus::sun_direction()`; it intentionally does not render the star.
- `game_cluster_ui`: build icon/selection details and `supply_warnings.hpp` expose H2 production, buffer, fuel-cell output/consumption and missing-H2 state without changing the status-bar layout.
- `game_cluster_construction`: build catalog and construction costs register the new resource kind. Hydrogen Generator is independent of the Stage 1-3 progress tree unless a later stage explicitly adds it.

## v0.43.16 precision-passage navigation

- `game/src/systems/builder_robot.cpp`: adaptive route-tier selection. Preferred 1.00 m structure halo -> physical-only direct probe -> wide/coarse 0.30 m -> fine/local physical-only -> wide/fine 0.80 m physical-only final fallback.
- `game/tests/navigation_regression.cpp`: `clearance-save <path>` regression mode validates that the supplied blocked-builder save publishes a route and escapes without a safety fault or replan thrash.
