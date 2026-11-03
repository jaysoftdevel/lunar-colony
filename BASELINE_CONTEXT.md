# game v0.42.0 baseline context

- Standalone C++20 cross-platform game with separate `engine/` and `game/` trees. Linux uses X11/OpenGL; Windows uses Win32/Direct3D 12.
- Root `VERSION` is the release source. Linux builds to `build/linux/bin/game`; Windows builds to `build/windows/bin/game.exe`. The paired `build.*`, `run.*`, `test.*`, and `dev.*` scripts expose matching workflows.
- All visible CPU lanes remain available through the persistent task pool. `GAME_CPU_LANES` is an optional diagnostic cap. Per-rover planning runs concurrently; arbitration, pose commit and resource/world mutation remain deterministic on the game thread.
- Render presents with swap interval 0 by default (VSync off) so the GPU can run at full host throughput; `GAME_VSYNC=1` re-enables synchronized present.
- The runtime loop is uncapped by default (no forced 1 ms idle sleep). Optional `GAME_FRAME_LIMIT_FPS=<n>` applies an explicit limiter when profiling or thermally constrained.

## Robot movement

- `safety_coordinator.*` is the only pose authority. Job routines submit goals and work holds.
- Global routing uses bounded persistent D* Lite with clearance costs and epoch-tagged parked-robot footprints, plus fleet-aware A*/dogleg recovery routes. Moving actors remain local traffic. The returned clearance callbacks capture margin values by value.
- Local rollouts and execution follow a moving geometric corridor with projection, visibility-limited lookahead, arrival braking and acceleration/turn limits. A preview follows turns over the full 1 s horizon instead of holding one heading for that second.
- In true head-on closing encounters, both active rovers apply an explicit local right-side evasion bias from their own frame before contract arbitration, producing stable symmetric passes.
- Planner load is phase-staggered per rover, heavy route-search concurrency is bounded by default (at most two lanes, independent of host core count), and per-tick planner-job intake is budgeted to smooth frame spikes under multi-rover replans. `GAME_ROUTE_SEARCH_LANES` may override heavy search lanes.
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
- Habitat templates no longer contain predefined crew entrances or exterior stair sets. A completed corridor connection creates its pressure hatch at the selected hull point on demand; additional branches reuse the existing sealed trunk.
- Generated hatch collars, pressure-door thresholds, docking ramps and crew floor queries use each habitat's real `walkable_floor_y`. The Air Lock creates its exterior access on the opposite end only after its first real base connection defines the inside side.
- Dock sleeves contain a continuous ramp and an interior threshold deck. Storage ports sit on the outer pressure wall. Interior circulation aisles retain crew head/shoulder clearance across size tiers.
- `mesh_portals.hpp` exactly clips triangles and lines around a doorway. The renderer caches local clipped layers, preserving wall material around the aperture and cutting an approach through tall foundations. Connected door openings are visible in both normal and POV views.
- Only the connected Air Lock permits exterior access; open corridor ends remain sealed. Outdoor machinery remains solid, while habitats, storage and the pressure corridor network are crew traversable.

## Verification and packaging

- Optional headless regressions: configure with `-DGAME_BUILD_NAVIGATION_TESTS=ON`, then use CTest. They cover swept fleet separation, smooth open travel, obstacle corners, changed scenes, occupied service points, opposing traffic, single-lane yielding, rotations/size tiers, storage ports, interior headroom and construction logistics.
- Update archive: `game-update-vX.Y.Z.zip` containing one `game-update-vX.Y.Z/` directory with `update.patch`, `apply.sh`, `apply.bat` and `update-summary.txt`. Installers print the full changed-file summary and do not build. Clean builds preserve existing executable-adjacent savegames, including on build failure.
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
- Parked robots hold their own bays without repeated route invalidation. Service restores mechanical condition at 1 percentage point per simulation second; powered bays now recharge persistent rover traction batteries and the charging load is part of the connected utility component.
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
## v0.43.15 hydrogen energy and tracked solar generation

- `ResourceKind::HydrogenGenerator` is an outdoor utility structure. Oxygen Generators retain electrolysis hydrogen in a local size-scaled buffer; only a completed Supply Channel component can make that inventory available to a fuel-cell generator. Hydrogen is never a global resource pool.
- Fuel-cell dispatch is load-following. Solar generation is used first, hydrogen generation covers remaining component demand and available battery charge input, then batteries discharge for any residual deficit. A fuel cell does not burn H2 when no load or battery can accept its output.
- The hydrogen energy model uses 33.3 kWh/kg LHV and 55% net electrical conversion, yielding 18.315 kWh/kg. Standard Hydrogen Generator peak output is 3.0 kW; SMALL scales to 50%. Oxygen Generator H2 production is 0.125 kg per kg O2 with a 4.5 kg standard local buffer.
- `solar_tracking.hpp` owns the nonvisualized stellar pose used by both simulation and rendering. The default reference is equinox/30 degrees north on the existing 24-hour game clock. `SolarPose` exposes direction, future render position, azimuth, elevation, daylight state and available irradiance.
- Completed Solar Panel collector heads rotate toward the simulated star on two axes; the pedestal/inverter remains fixed. No visible sun/star object is rendered in v0.43.15.
- Save schema 20 appends persistent hydrogen inventory to every `SupplyState`; schemas 1-19 remain readable with H2 initialized to zero. Progression stages and the v0.40.47 logistics/collision baseline are not changed by this update.

## v0.43.16 narrow-passage route fallback

- Rover structure clearance has two distinct layers: the hard circumscribed chassis envelope is never reduced, while the preferred extra structure halo is discretionary. Normal travel keeps the 1.00 m comfort halo; recovery may reduce only that extra halo to physical-only clearance.
- A wide 0.80 m precision grid with physical-only clearance runs only after the normal direct/dogleg, wide/coarse and fine/local attempts all fail. This keeps ordinary construction/service routing on the established fast tiers.
- This specifically fixes layouts where a rover must pass through a narrow but valid gap before making a wide detour. Do not replace this with a globally smaller robot radius; fleet and structure collision certification remains based on the full visible rover hull.

## v0.43.17 generated entrances and rover battery energy

- All habitat size variants expose zero predefined crew entrances and zero predefined entrance-access stair meshes. Corridor placement creates a generated pressure hatch at the player-selected pressure-hull point; the Air Lock accepts only its selected longitudinal base side and synthesizes the opposite exterior hatch from that real connection.
- Existing saved authored corridor doors migrate in place to generated physical hatches so old colonies keep their established openings without reintroducing template entrances.
- BR-02 and CR-01 now carry persistent traction batteries. Distance travelled consumes traction energy; onboard electronics and active work consume time-based electrical energy. Empty packs stop rover motion/work.
- Completed powered Robot Workshops recharge docked rovers at 8 kW input with 92% pack efficiency, and charger demand participates in the normal component-local power dispatch/brownout model. At or below 10% charge, an idle/unloaded rover returns to an operational workshop and remains until at least 95% charge.
- Starter packs are 48 kWh for BR-02 and 76 kWh for CR-01. The uninterrupted Stage-1/Stage-2 calibration run reaches Stage 2 without any recharge and finishes with 15.7% constructor / 13.9% carrier charge remaining, exceeding the requested 10% reserve. Save schema 21 persists battery capacity/charge; older saves load with full role-appropriate packs.


## v0.43.18 playfield perimeter circulation

New structures may no longer consume the final 2.25 m along any playfield edge. Habitat/resource footprints, Robot Workshop approach clearance, Storage Dome cargo aprons and generated corridor gate pads all respect the same reserved band, leaving the largest current rover enough hard-clearance space to route around edge-side colonies. Existing loaded structures are not moved; only new placement/access creation is constrained. The inner edge is rendered as a subtle guide.

The debug frame now owns a `WorldLayoutMode` selector (`MESH` / `TEXTURED`). No texture/material asset path exists yet, so both modes currently render through the procedural mesh backend; future texture work can dispatch on the stored mode without changing gameplay/UI contracts.

## v0.43.19 corridor attachment / short-run routing

Corridor structure attachments are cursor-angle driven over the complete available hull. The exact pressure-hull hit follows the cursor continuously through 360 degrees; the old four-face/seven-slot fallback has been removed. If the exact angular sector is unavailable because of an existing connection/gate or a tiny mesh seam, the resolver searches outward from the requested angle in 1 degree increments and uses the nearest valid free sector. Storage Dome crew-corridor anchors likewise keep their exact selected radial angle instead of snapping near cardinal faces; cargo-gate sectors remain exclusive.

For close unobstructed structure-to-structure links, the main corridor centerline is now a single straight segment. Generated vestibules still handle each hull's local pressure-door orientation, but the tube between the two outer docking endpoints is not allowed to form a tangent-driven S-curve. A bend is introduced only when the direct short segment is physically blocked.

## v0.43.20 shorter corridor start preview
- The initial corridor blueprint shown while hovering a structure is intentionally compact: the sample run is 1.35 m instead of 2.75 m.
- The fallback visual start marker is 1.10 m instead of 1.55 m.
- This only changes the pre-click blueprint visualization. It does not impose a minimum/maximum length on committed corridors.

## v0.43.21 minimal corridor start / connection-safe structure spacing
- Structure-hover corridor placement shows only a 0.56 m orientation/docking hint before the first click; it no longer creates a temporary routed corridor sample.
- `kStructureConnectionReserve` is derived from the complete generated corridor approach (`1.70 m` generated entrance + `1.75 m` docking transition = `3.45 m` per side).
- New habitat/resource placement uses that expanded ground reservation against other structures and the playfield perimeter. The visible blueprint ground rectangle matches the reserved area.
- The supplied `0-43-20.gmsave` spacing case (Small Air Lock at `(-20,-4)`, Small Oxygen Generator at `(-11,-4)`) remains loadable but is intentionally too close under the new placement rule; the nearest legal replacement spacing produces a straight two-point corridor.

## v0.43.22 Bio Dome end-cap corridor connectivity
- The Bio Dome greenhouse barrel now has real semicircular pressure end caps on both long-axis ends. Previously the half-cylinder roof was open at those ends above the low foundation, so a generated corridor hatch could find the lower wall but could not project its full doorway profile onto a pressure surface.
- Front/back corridor attachment now uses the same generated 360-degree pressure-hatch path as every other free Bio Dome hull sector; no special authored entrance or snap point is introduced.
- Existing saves need no migration; the mesh change affects rendering/attachment geometry only.

## v0.43.23 doubled new-game playfields

- New Game defaults to 128 x 128. Selectable sizes run from 32 x 32 through 1024 x 1024 in 32 m increments, preserving the same number/order of size choices at twice the previous linear scale.
- DEBUG START TEMPLATE now uses 128 x 128. Legacy saves may still contain the historical 16..512 m sizes; load validation now also accepts the new 1024 m maximum.

## v0.43.24 scrollable upper-left info pane

- The selection/details pane is capped to the usable area between the status bar and build catalog instead of allowing overflowing telemetry to draw outside its frame.
- Overflowing content is vertically scrollable with the mouse wheel and displays a proportional scrollbar. Non-overflowing selections keep the existing static layout.
- Wheel input while the pointer is inside the selection pane is UI-owned: it neither zooms the playfield camera nor cycles placement sizes.
- Scroll position resets when the selected world object changes and is clamped when content/viewport height changes.
