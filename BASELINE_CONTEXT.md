# game v0.41.8 baseline context

- Standalone C++20 Linux/X11 game, separate `engine/` and `game/` trees.
- Root `VERSION` is the release source. Build with `./build.sh`; `./run.sh` runs `build/bin/game`.
- All visible CPU lanes remain available through the persistent task pool. `GAME_CPU_LANES` is an optional diagnostic cap. Per-rover planning runs concurrently; arbitration, pose commit and resource/world mutation remain deterministic on the game thread.
- Render presents with swap interval 0 by default (VSync off) so the GPU can run at full host throughput; `GAME_VSYNC=1` re-enables synchronized present.
- The runtime loop is uncapped by default (no forced 1 ms idle sleep). Optional `GAME_FRAME_LIMIT_FPS=<n>` applies an explicit limiter when profiling or thermally constrained.

## Robot movement

- `safety_coordinator.*` is the only pose authority. Job routines submit goals and work holds.
- Global routing uses bounded persistent D* Lite with clearance costs and epoch-tagged parked-robot footprints, plus fleet-aware A*/dogleg recovery routes. Moving actors remain local traffic. The returned clearance callbacks capture margin values by value.
- Local rollouts and execution follow a moving geometric corridor with projection, visibility-limited lookahead, arrival braking and acceleration/turn limits. A preview follows turns over the full 1 s horizon instead of holding one heading for that second.
- In true head-on closing encounters, both active rovers apply an explicit local right-side evasion bias from their own frame before contract arbitration, producing stable symmetric passes.
- Planner load is phase-staggered per rover, heavy route-search concurrency is bounded by default, and per-tick planner-job intake is budgeted to smooth frame spikes under multi-rover replans. `GAME_ROUTE_SEARCH_LANES` may override heavy search lanes.
- CR-01 carriers delivering to incomplete habitat/resource blueprints may approach the target apron more closely so resource drop-offs visually align with staged blueprint packages.
- Local proposals refresh at 10 Hz; authoritative motion/braking/terminal contracts remain at 100 Hz. Path progress, rather than distance to a corner alone, scores candidates and detects circling.
- Vendored RVO2 (Apache-2.0) provides reciprocal velocity advice. See `third_party/rvo2/UPSTREAM.md` and `LICENSE`. It never commits poses or bypasses contracts. No online dependency fetch occurs at build time.
- Physical hull discs plus numeric margin are mandatory. Additional personal space is a steering preference, so a physically passable narrow gap remains usable. Coarse cell tokens are scheduling hints, not another spatial exclusion grid.
- Completed static geometry uses exact swept tests against inflated rotated footprints, corridor capsules and resource discs. Construction geometry still uses scene-admission and next-stage evacuation gates. Delivered resource boxes are admitted atomically against current hulls and all active motion/braking/terminal reservations before inventory changes.
- No speculative reverse fan or positional nudges. A blocked rover waits, pivots safely, obtains a stable detour, or retreats to a passing pocket. A parked idle robot may receive a certified yield task when it blocks a real destination or seals the only route; it holds the new bay until another job needs it. Active work holds remain protected. Untasked robots do not wander.
- The existing FIFO shipping/material accounting and staged construction policy remain in `builder_robot.cpp`. Service and queue positions use the same disc clearance as locomotion.
- Exterior rover queries use the planar playfield height except for the shared workshop ramp/deck height function. They do not run indoor crew/pressure-volume queries on every planner edge or safety sample.

## Structures and crew access

- Each habitat module declares its real `walkable_floor_y`; storage meshes declare theirs as well. Entrance anchors, frames, stairs, corridor collars and crew floor queries use that exact threshold.
- Human-scale frames and exterior access steps are rebuilt after horizontal size scaling. Steps end at the threshold and are oriented in the entrance's own basis. Air-lock access renders at actual construction completion.
- New structure corridors snap to authored entrances. Additional branches reuse the existing sealed trunk. Legacy generated hatches remain installed physical entrances; loaded collars and floors are synchronized with current models.
- Dock sleeves contain a continuous ramp and an interior threshold deck. Storage ports sit on the outer pressure wall. Interior circulation aisles retain crew head/shoulder clearance across size tiers.
- `mesh_portals.hpp` exactly clips triangles and lines around a doorway. The renderer caches local clipped layers, preserving wall material around the aperture and cutting an approach through tall foundations. Connected door openings are visible in both normal and POV views.
- Only the connected Air Lock permits exterior access; open corridor ends remain sealed. Outdoor machinery remains solid, while habitats, storage and the pressure corridor network are crew traversable.

## Verification and packaging

- Optional headless regressions: configure with `-DGAME_BUILD_NAVIGATION_TESTS=ON`, then use CTest. They cover swept fleet separation, smooth open travel, obstacle corners, changed scenes, occupied service points, opposing traffic, single-lane yielding, rotations/size tiers, storage ports, interior headroom and construction logistics.
- Update archive: `game-update-vX.Y.Z.zip` containing one `game-update-vX.Y.Z/` directory with only `apply.sh` and `update.patch`. Apply does not build. Clean builds preserve existing executable-adjacent savegames, including on build failure.
- v0.40.47 remains the historical logistics/build baseline. For current locomotion and access behavior, this document supersedes older version notes in `ARCHITECTURE.md`.

## v0.41.6 storage handling and pressure doors

- Storage includes one fixed robot loading gate with a sealed transfer buffer and supports two paid additional gates as of v0.41.8. Both BR-02 pickup and CR-01 deposit/pickup use completed gates and per-gate off-axis queues. Crew cannot use them as surface entrances.
- New gates occupy local +X. Existing saves select a free docking face once (a spare diagonal if every cardinal face is already used); schema 14 persists that choice and still reads schemas 1-13.
- The loading apron rejects crew-corridor attachments and crossing routes, including corridor-link shortcuts. Structure placement also keeps existing aprons accessible.
- Every completed corridor/structure interface has solid paired sliding leaves with perimeter and centre seals. One transient opening state drives animation and swept POV collision; occupancy prevents closing on a person and Air Lock doors interlock. Doors reset sealed on load.
- Added `navigation_pressure_gates` headless regression for seals, reservations, gate inventory, concurrent deposits and persistence; geometry regressions now wait for doors to open and exclude reserved cargo ports.

## v0.41.7 outdoor robot workshop

- Robot Workshop is a fixed-size outdoor structure in the Structures palette, independent of the habitat/corridor chain. Its procedural barrel-roof model has four independent charging/cargo/service docks, a wide automatic sliding gate and a continuous 0.4 m deck reached by a 5 m boarding ramp.
- Each completed workshop assigns four unique home bays. Idle constructors/carriers with no active task or cargo return through a staged approach; inbound ramp admission is serialized, actual work exits take priority, and new tasks immediately preempt parking. Extra fleet members require additional workshops.
- Parked robots hold their own bays without repeated route invalidation. Service restores mechanical condition at 1 percentage point per simulation second; charging hardware is provided for a future energy model.
- Workshop walls, dock cabinets, gate plane and ramp curbs use exact swept disc/box tests in the existing motion authority. Rover/POV floors follow the shared ramp and deck geometry. No pose nudges or teleportation are added.
- A visible, rotated approach reservation covers the boarding ramp and waiting/turning area. Placement checks bounds, structures, ground piles, corridors and walkways; a cached static path search rejects an enclosed approach with no route to the fleet. Later structure/corridor/walkway placement also respects the reservation.
- Save schema 15 adds the new persisted structure kind; schemas 1-14 remain readable. Parking assignments and gate animation are transient and reconstructed from saved robot poses. A gate occupied on load restores open before collision certification.
- Stationary certified holds no longer consume planner intake slots. This prevents parked high-priority actors from starving the last low-priority returning robot in larger fleets; all existing contract arbitration and collision checks remain active.
- `navigation_workshops` covers four robots, rotated buildings, eight robots/two workshops, real task dispatch and return, save/load, construction completion, ramp clearance and static reachability.

## v0.41.8 storage extensions and mesh rendering

- The storage selection panel plans at most two additional loading gates on blueprint or completed domes. Each is a child `StorageGate` construction target with its own material plan, site buffer, construction stages and stable parent placement sequence. Builders wait for the parent to complete; carriers may pre-stage materials. Each gate currently consumes 78 kg of construction materials. Inventory is charged by actual construction, not by clicking the option.
- All planned gate faces reserve their loading aprons immediately. Placement/routing excludes those aprons and occupied crew corridor faces. Completed gates have separate service claims and queues, permitting parallel transfers against one shared inventory.
- Gate builders work outside the parent hull. Gate mesh chambers, animated leaves and pressure doors follow blue/yellow/green construction rendering without opaque structure surfaces. Unused crew-port furniture is clipped only at cargo faces; hull and rear transfer seals remain intact.
- Schema 16 persists child-parent links, validates the two-gate/unique-face limit and restores unfinished upgrades. Deleting a dome cascades to its gates and remaps active construction targets. Schemas 1-15 remain readable.
- `navigation_storage_extensions` covers real selection-button clicks, both parent build states, exact material accounting, dependency order, all size/rotation gate meshes, sealed rear bulkheads, reserved faces, persistence, parallel transfers and cascading removal.
