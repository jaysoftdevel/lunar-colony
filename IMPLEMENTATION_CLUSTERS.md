v0.51.3 HUD and wireframe ownership: `ui/status_bar.cpp` owns both status rows, complete resource labels/values and exact tooltips. Shared `ui/layout.hpp::STATUS_BAR_H` reserves the additional row for panels, hit testing and HUD elements. `playfield/lunar_lander_renderer.cpp` builds and submits only primitive edges; `playfield/trader_renderer.cpp` selects edge-only ship/crane/plume rendering while preserving separate export package rendering. UI regressions cover complete inventory, gas exclusions, tooltip hitboxes, resolutions and zooms. Ship regressions assert nonempty wire topology and no filled-triangle submissions across flight and deployment phases.

v0.51.2 deployment layout ownership: `systems/lander_stash.cpp` owns mouse-site validation, stash geometry/slots, atomic package placement and legacy conversion, salvage accounting and stash UI. `lunar_lander.cpp` owns explicit instant cargo handoff and continuing ramp/fleet deployment. Ship renderers own concept-based meshes. `builder_robot.cpp` uses stash aisles and spatially indexed exact package obstacles; fleet arbitration remains unchanged. `save_game.cpp` appends schema-34 placement/salvage fields. `lander_regression.cpp` covers mouse picking, package mass/count, real construction, one-shot salvage, waiting/boxed/salvaged saves, schema-33 conversion and the full Free Play reserve.

v0.51.1 trader ownership: `systems/trader.hpp/.cpp` own visit/manifest state, prices, exact physical stock reservation, separate oxygen export buffers, rover jobs and pad controls. `playfield/trader_renderer.cpp` owns native ship approach/loading/departure geometry and export packages. `builder_robot.cpp` submits trade intents through the unchanged safety coordinator after normal construction dispatch; occupied trade jobs retain their own custody/watchdog state. `supply_simulation.cpp` bottles real surplus O2 with cabin-air priority and electrolysis mass balance. `save_game.cpp` appends schema-33 state with cargo-custody validation. `trader_regression.cpp` replays powered call, live gas production, real hauling from ground/storage, mid-haul save/load, departure/payment-once and UI scaling.

v0.51.0 lunar deployment ownership: `systems/lunar_lander.cpp` / `systems/lunar_lander.hpp` own terminal guidance samples, resumable phase sequencing, exclusive ramp traversal, manifest accounting, landing-yard reservation and status UI. `playfield/lunar_lander_renderer.cpp` owns the native 3D lander, articulated ramp, landing gear, engine plume, ballistic dust and crane. Existing robot rendering is reused for the ramp rover; normal fleet locomotion remains under the safety coordinator. `save_game.cpp` appends schema-32 state. `lander_regression.cpp` exercises the full handoff, all rover counts, exact mass balance, mid-sequence save/resume, UI and actual depot logistics.

v0.50.16: Hydrogen tank mesh (`resources/hydrogen_tank.cpp`) and catalog/build-icon/cost/label integration; `supply_simulation.cpp` component-local tank filling and landing-pad consumption; `selection_panel.cpp` fueling controls; `save_game.cpp` schema-31 landing-pad allocation save/load; `navigation_regression.cpp` isolated and connected mass-balance tests.

v0.50.15: 20 additional one-time Stage-1 contracts, EUR 94.35M prelaunch commercial base funding, conservative full research+vehicle+minimum cargo budget, schema-30 12-to-32 contract migration.
### v0.50.13 Stage-1 UI interaction ownership

- `game/src/systems/planning_game.cpp`: renders modernized Stage-1 navigation/contract/technology/mission panels and dispatches double-click on selectable cards to existing guarded game actions.
- `game/include/game/game_app.hpp`: owns transient double-click tracking; no savegame data changes.
- `game/tests/ui_regression.cpp`: verifies direct item actions and no duplicate cash charges.

0.50.8: Stage-1 technology graph implementation remains inside the
`planning_game.cpp` simulation cluster to access its existing immutable tech
definitions, but is isolated in `GameApp::render_planning_technology_tree()`.
Only view-local drawing/navigation changed; no engineering cluster boundary or
build dependencies changed.

## v0.50.7 rendering stabilization

- `engine/src/renderer_gl.cpp`: 1024px GPU material generation and streaming,
  world-aligned texture sampling, mipmaps, per-face normal lighting, diagnostic
  software OpenGL opt-in.
- `game/src/systems/camera_controller.cpp` and header: dedicated Stage-1
  zoom-aware near/far projection only; Stage-2 projection unchanged.
- `game/src/systems/planning_game.cpp`: 3D coastal landscape, shaded facades,
  building details, shoreline, roads and sunset-free daylight atmosphere.
- `game/src/screens/new_game.cpp`: lower (47-degree) oblique camera on Stage-1
  start. No mission logic or save schema changes.

## v0.50.6 source ownership

- `game/src/screens/new_game.cpp`, `main_menu.cpp`: Career/Free Play screen flow and responsive background/menu graphics.
- `game/include/game/game_app.hpp`, `game/src/systems/save_game.cpp`: active game mode, fixed Stage-1 extent and schema-29 persistence/migration.
- `game/src/systems/planning_game.cpp`: procedural launch-center environment, campus facade detailing, ground shadows and site material use.
- `engine/include/engine/engine.hpp`, `engine/src/renderer_gl.cpp`: optional material geometry API and Linux GLSL fine surface patterns. D3D12 uses the API's compatible fallback; do not change its main draw path unnecessarily.
- `game/tests/ui_regression.cpp`: Career and Free Play UI routes, fixed Stage-1 dimensions, 3D site camera and schema-29 save checks.

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

- `game/src/screens/new_game.cpp`: debug toggle, 128x128 template lock, paused startup.
- `game/src/systems/save_game.cpp`: construction-only debug template parser/importer.
- `game/debug-templates/0-40-32-debug-start.gmsave`: bundled debug-template payload, now replaced with the supplied 0.43.28 save layout (filename retained for runtime compatibility).
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


## v0.43.17 generated entrances and robot batteries

- `game_cluster_world`: `habitat_mesh.cpp` strips legacy authored entrance metadata before size variants are produced; `corridor/entrances.cpp` resolves only generated hull doors for habitats. Air-lock connection/cycle code derives end zones and chamber partitions from hull dimensions instead of entrance arrays.
- `game_cluster_navigation`: `builder_robot.hpp/.cpp` owns persistent role-sized battery state and integrates distance-based traction plus auxiliary/work loads from authoritative committed movement. Low-charge idle/unloaded robots release work and return to an operational workshop.
- `game_cluster_supply`: `supply_simulation.cpp` adds docked workshop chargers as real network loads and transfers energy to rover packs using the component power fraction.
- `game_cluster_persistence`: save schema 21 appends rover battery capacity/charge and migrates older saves to full role packs.
- `game_cluster_ui`: robot selection exposes kWh, percentage, draw/charging state and low/empty warnings.
- `navigation_regression battery-bootstrap` constructs the complete Stage-1/Stage-2 bootstrap with one constructor and one carrier, forbids charging, and requires both roles to retain at least 10% charge. Geometry/Air Lock tests require empty authored entrance arrays and connection-created generated hatches.


## v0.43.18 perimeter lane / debug render mode

- `game/world/geometry_standards.hpp` owns the canonical 2.25 m `kPlayfieldRobotPerimeterLane` placement reserve.
- `features/habitat_placement.cpp`, `features/resource_placement.cpp`, `systems/storage_extensions.cpp` and `systems/gate_areas.cpp` keep structure bodies and reserved service/access geometry inside the lane boundary.
- `playfield/playfield_view.cpp` renders the inner lane guide; navigation itself still uses the full playfield and therefore may use the reserved band as a bypass.
- `GameApp::WorldLayoutMode` plus `ui/debug_frame.cpp` expose the future MESH/TEXTURED render dispatch. Current textured mode intentionally falls back to the same procedural mesh renderer because no texture asset/material pipeline exists yet.

## v0.43.19 continuous corridor attachments

- `world/corridor/entrances.cpp` resolves generated habitat entrances from the cursor's exact polar ray and replaces the cardinal face/slot fallback with nearest-free angular search. Storage Dome corridor attachment no longer snaps to cardinal crew faces; real cargo-gate sectors and existing pressure entrances still reserve their space.
- `world/corridor/factory.cpp` tries a two-point straight centerline first for close structure-to-structure connections when the direct swept route is valid. Existing obstacle routing remains the fallback.
- `navigation_regression geometry` sweeps 360 one-degree habitat attachment positions, checks continuous hull movement, verifies on-demand entrances remain traversable, and requires a close unobstructed structure pair to produce exactly one straight corridor segment.

## v0.43.20 corridor start-preview sizing
- `game/src/features/corridor_builder.cpp`: short structure-attached pre-click sample (1.35 m) and compact fallback marker (1.10 m).
- Actual corridor routing and committed geometry are unchanged after the source point is selected.

## v0.43.21 corridor-safe structure envelopes
- `game/world/geometry_standards.hpp`: canonical `kStructureConnectionReserve`, `kStructurePlacementGap`, and `structure_ground_extent()` derive placement space from corridor docking geometry.
- `features/habitat_placement.cpp` / `features/resource_placement.cpp`: structure-to-structure placement and map-edge checks use the expanded ground reservation; workshop/service checks respect the same reserved structure area.
- `playfield/world_renderer.cpp`: placement previews and step-0 construction blueprints draw the expanded ground reservation instead of only the physical mesh footprint.
- `features/corridor_builder.cpp`: structure-hover start preview is a 0.56 m compact endpoint hint; no temporary corridor route is generated before the source click.
- `navigation_regression geometry`: reproduces the supplied 0.43.20 Air Lock/Oxygen Generator spacing and requires the nearest new legal placement to accept a straight two-point corridor.

## v0.43.22 Bio Dome pressure end caps
- `world/habitats/bio_dome.cpp`: closes both X-axis ends of the greenhouse half-cylinder with glazed semicircular pressure surfaces, retaining the existing foundation below the arch spring. Generated corridor doorway profiles can therefore conform to the complete end face.
- `navigation_regression geometry`: verifies both Bio Dome long-axis ends for every habitat size at 0, 37 and 90 degree rotations and requires a full generated wall profile plus corridor preview.
- The perimeter placement validation fixture now derives its safe test position from `structure_ground_extent()` so it matches the v0.43.21 corridor-reserved ground rectangle.

## v0.43.23 new-game playfield sizing

- `game_app.hpp` owns the new-game playfield constants: 32 m minimum, 1024 m maximum, 32 m step, 128 m default/debug-template size.
- `screens/new_game.cpp` consumes those constants for the selector and debug-template lock; `systems/save_game.cpp` retains legacy 16 m compatibility while accepting the new 1024 m maximum.

## v0.43.24 selection-panel scrolling

- `systems/world_selection.cpp`: owns selection-panel content height, viewport-height capping, scroll range, per-selection scroll identity and wheel-scroll state.
- `systems/gameplay_controller.cpp`: routes wheel input to the info pane before camera/build-size controls whenever the pointer is over the pane.
- `ui/selection_panel.cpp`: offsets only contained rows/buttons by the scroll position, suppresses rows outside the viewport, and renders a proportional scrollbar without enlarging the panel.
- `ui_regression`: forces a warning-heavy Storage Dome to overflow at 200% UI scale and verifies the bottom rows become reachable, the scrollbar is visible, scrolled text stays inside the panel, and the camera distance does not change.

## v0.44.0 Bio Dome hydroponics
- `game/include/game/world/hydroponics.hpp`: crop catalog, crop profiles, bay runtime state, size-independent bay storage and balanced default assignments.
- `game/include/game/world/habitat_mesh.hpp`: Bio Dome bay-count scaling and persistent per-habitat hydroponics state.
- `game/src/systems/supply_simulation.cpp`: hydroponics electrical/water load, breathable-air gating, crop growth and Food harvest integration.
- `game/src/ui/selection_panel.cpp` + `game/src/systems/world_selection.cpp`: interactive per-bay crop table, progress bars and scroll-aware Bio Dome status sizing.
- `game/src/systems/save_game.cpp`: schema 23 crop/progress/harvest persistence and schema <=22 balanced migration.
- `navigation_regression hydroponics` and `ui_regression`: resource dependencies, harvest, save round-trip, bay counts, selectors, progress bars and overflow scrolling.


## v0.44.1 POV interior collision
- `game/src/systems/first_person_mode.cpp`: triangle-based interior-body collision for habitat interior/details meshes, Storage Dome racks/cargo, and Robot Workshop service equipment; portal clearance aligned with dynamic render carving.
- `game/tests/navigation_regression.cpp`: all habitat types must retain a clear interior point while exposing at least one solid interior obstacle; Storage Dome entrance aisles remain walkable.

## v0.44.2 Bio Dome commissioning / photoperiod UI
- `game/include/game/world/hydroponics.hpp`: authoritative 06:00-22:00 light / 22:00-06:00 dark greenhouse schedule and phase helpers; bays default to Empty.
- `game/src/features/habitat_placement.cpp`: removes automatic starter-crop assignment from newly placed Bio Domes.
- `game/src/systems/save_game.cpp`: schema <=22 hydroponics migration now initializes empty bays; schema 23 continues to preserve explicit crop/progress/harvest state.
- `game/src/systems/supply_simulation.cpp`: grow-light/water demand and growth are light-phase gated while base Bio Dome climate power remains continuous; breathable-air and power gating remain authoritative.
- `game/src/ui/selection_panel.cpp` + `game/src/systems/world_selection.cpp`: post-construction-only two-column bay table, per-bay dropdown selection/removal, progress bars, photoperiod/system state and scroll sizing.
- `game/src/playfield/world_renderer.cpp`: completed Bio Domes use dimmed greenhouse colors during the dark phase and the bright grow-light palette during the light phase.
- `navigation_regression hydroponics` and `ui_regression`: empty defaults, photoperiod boundaries, dark/no-air/no-power growth blocking, dropdown crop removal, construction gating and two-column layout.

## v0.44.3 corridor rejection diagnostics
- `game/src/features/corridor_builder.cpp`: maps corridor endpoint/routing/gate-clearance rejection states to player-facing reasons and renders the cursor tooltip after the 3D placement pass.
- `game/src/playfield/playfield_view.cpp`: resets per-frame corridor diagnostic state before preview generation and renders the tooltip last in the HUD layer.
- `game/include/game/game_app.hpp`: transient corridor tooltip title/detail state and render hook.
- `game/tests/ui_regression.cpp`: verifies rejection tooltip text remains inside HUD bounds near a viewport corner.

## v0.50.0 top-level game stages
- `game/include/game/game_app.hpp`: introduces the independent `GamePhase` model and tracks New Game selection vs active-session phase.
- `game/src/screens/new_game.cpp`: owns the three-stage selector, defaults to Stage 2, preserves Stage-2 setup, initializes Stage 1 on the Earth-side company campus and Stage 3 from the completed debug-base scenario.
- `game/src/game_app.cpp` + `game/src/playfield/playfield_view.cpp`: route Stage 2 and Stage 3 to the lunar playfield while Stage 1 uses its independent company/planning renderer.
- `game/src/systems/gameplay_controller.cpp`: Stage 1 runs its planning economy plus Company Site camera controls; Stage 2/3 own lunar-world simulation. Stage 1 never advances robot/construction/POV systems.
- `game/src/systems/save_game.cpp`: schema 24 persists the top-level game stage and maps all older saves to Stage 2.
- `game/tests/ui_regression.cpp`: covers selector presence/default, empty Stage 1 creation, Stage 1/3 placeholder rendering and high-zoom menu bounds.


## v0.50.1 Stage-1 planning cluster

- `game/include/game/systems/planning_game.hpp` owns the persistent Stage-1 planning state: funds, planning time/speed, view, technology unlock flags and contract progress/history.
- `game/src/systems/planning_game.cpp` owns the Stage-1 contract catalog, 16-node technology graph, funding/progression rules and the Contracts/Tech Tree rendering.
- `game/src/systems/gameplay_controller.cpp` dispatches Stage 1 to the planning simulation and keeps Stage-2 world simulation isolated.
- `game/src/systems/save_game.cpp` schema 25 persists the planning state.
- `game/src/screens/new_game.cpp` resets the Stage-1 company state when a new Planning session is created.


## v0.50.2 Stage-1 tree / Stage-3 built-base cluster
- `game/src/systems/planning_game.cpp`: lays the 16 technologies out by dependency depth and lane, draws directional prerequisite branches, and rotates the hierarchy for compact/high-zoom layouts.
- `game/src/screens/new_game.cpp`: Stage 3 automatically selects the fixed 128 x 128 debug-base layout and imports it as a completed starting base.
- `game/src/systems/save_game.cpp`: the debug-template importer accepts a completed-construction mode; Stage 2 still imports blueprints while Stage 3 marks every construction entity complete. Debug-template path lookup also supports test/source-tree execution.
- `game/src/game_app.cpp` and `game/src/systems/gameplay_controller.cpp`: Stage 3 now uses the normal playfield render/update path instead of the placeholder shell.
- `game/tests/ui_regression.cpp`: verifies the connected Stage-1 tree, Stage-3 debug-base import, completion state across all construction domains, removal of the placeholder view, and Stage-3 save/load persistence.


## v0.50.3 Stage-1 mission planning cluster
- `game/include/game/systems/planning_game.hpp`: persistent rocket/lander selection, lunar cargo bundle and mission-payment state.
- `game/src/systems/planning_game.cpp` (`game_cluster_simulation`): vehicle catalog, payload/cost rules, recipe-derived minimum mission cargo and Mission Planning UI.
- `game/src/systems/save_game.cpp` (`game_cluster_simulation`): schema-26 persistence/migration for the mission plan.
- `game/tests/ui_regression.cpp`: mission UI scaling, funding gates and persistence regression.


## v0.50.4 Stage-1 company-site cluster
- `game/src/systems/planning_game.cpp` (`game_cluster_simulation`): procedural launch-campus geometry, world-space facility labels, footprint picking and facility-to-planning-view routing.
- `game/src/systems/gameplay_controller.cpp`: enables zoom/orbit/pan/WASD camera movement while the Site planning view is active.
- `game/src/screens/new_game.cpp`: applies the default Stage-1 campus overview camera.
- `game/include/game/systems/planning_game.hpp`: adds `PlanningView::Site` as value 3 to preserve prior enum values.
- `game/src/systems/save_game.cpp`: save schema 27 validates/persists the Site view.
- `game/tests/ui_regression.cpp`: covers default Company Site startup, physical Program Office click routing to Contracts and 25 site resolution/zoom combinations.


## v0.50.5 Stage-1 facility workflow cluster
- `game/include/game/systems/planning_game.hpp`: persistent `MissionPreparationStep` / `MissionFacility` state.
- `game/src/systems/planning_game.cpp` (`game_cluster_simulation`): ordered mission gates, facility ownership checks, facility-specific preparation UI, campus next-facility highlighting, design/manifest freeze points, program funding, build/integration/readiness/launch progression.
- `game/src/systems/save_game.cpp` (`game_cluster_simulation`): schema-28 workflow persistence and schema-26/27 migration.
- `game/tests/ui_regression.cpp`: full Research-to-Launch gate chain, wrong-facility rejection, under-minimum cargo rejection, schema-28 persistence and 25 mission-workflow resolution/UI-scale cases.

## v0.50.9 — Stage-1 customer-project and research-evidence integration

Contract catalog IDs, advance/final payment amounts and duration data remain stable for historical saves. Existing `PlanningContractProgress` persists the progress and completion history; a project at or above duration is now *ready for explicit delivery*, not automatically paid. Research evidence is computed deterministically from `completed_count > 0`, mapped to a linked technology, with a maximum 35% discount from distinct customer projects; no new serialized fields are required. Final lunar customer delivery is a special event settled by the ordered Launch gate, not the planning timer. The Contracts view is now a selected customer-brief workflow rather than 12 compact list rows, and the Site view indicates the immediate useful action before the eventual mission gate. Schema 29 is retained; older Stage-1 contract runs may have ready-for-submission reports. Stage 2 and Stage 3 are unaffected.


### 0.50.10 Stage-1 campaign balance and coaching

- Adds a persistent context-sensitive **NEXT STEP** navigation shortcut and direct facility action on the 3D campus; never purchases or submits work without a separate click.
- Commercial projects: faster completion for non-tutorial jobs; use **ADVANCE TO NEXT REPORT** to advance exactly to the nearest active project delivery and pause.
- Technical evidence: first completed customer project provides a 20% discount to its target technology (distinct evidence capped at 35%). Previously completed projects still apply without a save schema change.
- One-time first-customer completion premium (+20% final payment for projects 2–11), plus transparent investor reputation awards when unique completed projects reach 3, 6, and 9 (EUR 750K, 1.5M, and 3M). Repeat projects pay their normal price and remain available to prevent deadlocks.
- Contract list ranks work ready for submission, active jobs and first-time relevant opportunities before repeat work; research shows a funding shortcut when unaffordable.
- Stage-2 and Stage-3 mechanics are unchanged. Save schema 29 is preserved; old contract IDs and progress are retained.
