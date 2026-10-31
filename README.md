# Game workspace

**Baseline release: 0.30.0** — rendering, pressure-network topology, crew scale, movement and buildable geometry are frozen here as the gameplay-development baseline.

Native C++20 prototype with a strict split between the reusable engine (`engine/`) and game implementation (`game/`).

## Local Linux build

Required tools/libraries: a C++20 compiler, CMake, X11 development headers and a working OpenGL/GLX runtime.

Debian/Ubuntu example:

```bash
sudo apt install build-essential cmake libx11-dev libgl1-mesa-dev
./build.sh
./build/bin/game
```

`build.sh` always deletes the previous `build/` directory first and produces the executable only under `build/bin/`. At completion it prints the final game binary size in KB and bytes. No prebuilt executable is stored in the project workspace.

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

## Baseline physical/visual contract
- The canonical astronaut is 80% of the earlier prototype scale; eye height, body radius and clearance tests use the same shared constants.
- Habitats and resource structures receive shared maintainability hardware plus their type-specific equipment.
- Corridors use continuous pressure shells, reinforced ribs, internal decks, service trunks, double dock seals and structural sealed-end bulkheads.
- Exterior walkways are non-pressurised and can meet the colony pressure system only at a connected Air Lock.
- Supply channels include structural supports, electrical/data routing and water-service hardware.
- World construction geometry is mesh-only in the current development phase: blue target mesh, yellow installed mesh and substantially bolder green mesh at completion. No structure textures or filled surfaces are emitted yet.

### 0.31.2
BR-02 builders use chassis-sized collision avoidance around structures, resource packages and fleet mates. Construction work is visibly animated through pick-up, put-down, material application, welding and build-up; stages advance only after the resource-backed animation sequence completes.

## 0.31.3 construction presentation
Incomplete habitats, resource facilities, corridors, supply channels and walkways retain a faint full final-mesh outline while the built portions are revealed. BR-02 builders use acceleration/turn-limited steering with stand-off arrival, move around the target during assembly, and use staged manipulator sequences for pickup, placement, material application, welding and build-up.
## 0.31.4 carrier logistics and simulation speed
Construction blueprints no longer reserve their complete footprint as robot collision. BR-02/CR-01 motion is blocked only by geometry that has actually been erected at the current construction stage, allowing robots to cross future/unbuilt volume while naturally routing around walls, machinery and hull segments as they appear. New games include one CR-01 Carrier Bot in addition to the selected 1-6 BR-02 constructors. CR-01 prioritizes material delivery to waiting construction sites and automatically consolidates loose resource pallets into a completed Storage Dome, which becomes the preferred supply depot. Constructors remain responsible for building and self-haul only when carrier support cannot satisfy a site. The bottom bar provides NORMAL 1X, QUICK 2X and FAST 4X simulation speed controls; POV/camera response remains real-time.


## v0.31.5 construction safety
Construction steps use a pre-commit fleet evacuation gate: blueprint space remains traversable until its mesh exists, but a new stage cannot appear around a robot. Before walls/body/network geometry is revealed, affected robots are moved to a safe exterior position and the step commits only after the fleet is clear.

## v0.31.6 fleet routing

New Game independently selects 1-6 BR-02 builders and 1-6 CR-01 carriers. Carrier bots specialize in hauling resources from ground piles or the Storage Dome depot to active construction sites; builders remain construction-first but can self-haul when a site is starved. Robot trips are planned before motion using collision-aware shortest-path search, line-of-sight smoothing and fleet route reservations, so conflicting routes wait/replan instead of crossing.

## v0.31.7 cooperative robot crews
Multiple BR-02 builders can now share one construction target and work around it in parallel while one coordinator owns the authoritative site buffer and stage commit. Multiple CR-01 carriers can feed the same site when the remaining material mass requires more than one payload, without duplicating already-reserved deliveries. Builders still self-haul if the shared site remains starved. CR-01 transit speed is 10% above BR-02 (2.42 m/s vs 2.20 m/s).

## v0.31.9 synchronized carrier logistics and robot inspection
CR-01 carriers now derive every trip from the shared construction-site material ledger: required mass minus delivered mass minus fleet-reserved/in-transit mass. Each trip reserves one concrete material and kilogram amount before movement, preventing duplicate hauling while allowing several carriers to support the same site in parallel. Ground resource stacks expose collision-free service positions and the starter depot uses robot-accessible service spacing, so a valid reservation always maps to a reachable pickup. BR-02 builders continue to self-haul only the unreserved deficit when carrier logistics cannot satisfy a site.

Every BR-02/CR-01 is individually selectable in the world. The robot inspector shows role, live state/action, condition, movement/route state, construction target, cargo by material and kilograms, exact logistics reservation, and the target site's delivered / reserved-in-transit / still-unassigned material ledger.


## v0.31.9 construction tracking
Incomplete habitats, resource structures, corridors, supply channels and walkways keep a projected status frame visible until completion. Each frame shows build progress and remaining material demand after delivered and in-transit resources are accounted for.

## v0.31.10 construction logistics and movement stability
Placement rotation is keyboard-only through N/M; the mouse wheel is reserved for scalable-object size selection or normal camera zoom. The starter material row is positioned relative to the marked stash on every supported playfield size, including migration of legacy misplaced starter piles. Construction frames now separate still-needed resources from material already delivered or reserved/in transit and retain the progress bar until completion. Robot routing uses short obstruction hysteresis plus a smoothed visual chassis pose to avoid edge/replan flicker. Carrier reservations act as bounded leases, and BR-02 builders promptly self-haul any shortage that is not being actively supplied, preventing construction from deadlocking when carriers are unavailable or blocked.

## v0.31.11 independent fleet routing and construction staging

- Delivered construction resources remain visibly staged as boxes at the construction site until consumed by a build step.
- BR-02/CR-01 routes are independently calculated. Other robot routes are soft planning costs rather than blocking reservations.
- Short-horizon reciprocal steering keeps robots physically separated without fleet-wide waiting chains and uses a smoothed avoidance bias to prevent left/right flicker.
- Construction status frames explicitly use the global UI scale transform.

## v0.31.12 construction blueprint rendering
Incomplete habitats, resource structures, corridors, supply channels and walkways are rendered as blueprint-only wireframes with no filled/material surfaces. Unbuilt geometry remains visible in a subdued blue while erected portions use brighter, substantially thicker blue lines. Normal filled materials are applied only after construction reaches 100%.

## v0.31.13 robot-runtime stabilization
Robot simulation is sub-stepped independently from game-speed rendering and the visible chassis follows the authoritative position through a critically damped render spring. Pickup, delivery and construction use stable per-task service/work stations instead of recomputing moving goals each frame. Path following accepts intermediate waypoints with a safe pass radius and slows nearly to a turn-in-place for sharp heading changes, eliminating waypoint orbiting.

Multi-robot operation is fully local: each unit owns its route, resource reservation and recovery watchdog. Stalled units rotate to another persistent service slot without resetting other robots. Small structures use footprint-aware constructor crew limits so carriers retain access, while larger structures can still use larger cooperative teams. Construction safety checks only geometry introduced by the pending build stage; evacuation has exclusive movement control and uses an exterior safety margin, preventing robots from being enclosed as new mesh appears. Delivered site-buffer resources remain visible as staged boxes until consumed.


## v0.31.17 continuous multi-robot traffic
Robot travel no longer executes fixed Cooperative-A* time slots. Each task builds an any-angle, line-of-sight-reduced route before departure, rounds legal corners, predicts the complete fleet trajectory and prefers a separate traffic lane when conflicts are detected. During execution a continuous speed governor negotiates right-of-way without changing the planned steering line; persistent yielding triggers a whole-route replan instead of reverse/steer/spin recovery. Dynamic priority age prevents one robot from being starved indefinitely. Authoritative and rendered robot poses are now identical, and robot simulation uses 90 Hz bounded substeps so NORMAL/QUICK/FAST modes do not expose stop/go solver quantization.

Habitats, resource structures, corridors, supply channels and walkways remain mesh-only at every build state. Blue denotes unbuilt target geometry, yellow denotes installed construction and completed geometry is rendered as a heavier green mesh; filled structure surfaces/textures are intentionally disabled.

## v0.31.18 balanced construction-fleet scheduling
Structure placement no longer triggers full-playfield route searches. Robot paths use a bounded corridor-local A* search that widens only for real detours and caches occupancy checks, keeping planning cost proportional to the useful route instead of map area. Constructor and carrier route planning use separate per-slice budgets so builders cannot monopolize the planner while logistics waits.

The starter depot is a deterministic traffic layout with hull-safe parking, one front pickup lane per material and a clear entry/exit spine. Same-material pickup is serialized only at the source lane; different materials and construction routes continue in parallel. Idle carriers return to dedicated standby bays instead of blocking pallet exits. Construction logistics are carrier-first; builders remain at their worksite unless the site coordinator sees a genuine carrier-service timeout. Pickup/delivery/work stations stay fixed for the current operation so destinations cannot drift while a robot is travelling.

The release QA places an Air Lock, adds a Factory while 6 BR-02 builders and 6 CR-01 carriers are already active, and checks fleet collisions, carrier delivery liveness, low-speed spin, simultaneous movement and placement/update timing. Placement scaling is also checked from 16 m through 512 m playfields.


## v0.31.19 concurrent logistics lanes and close-range construction
BR-02 construction stations are calculated from the actual rotated structure perimeter. Constructors face the work and stop with the front of the chassis roughly 0.20 m outside the modeled shell instead of using the old footprint-diagonal stand-off ring.

CR-01 starter-depot traffic uses a per-material admission queue with an outer staging spine, one-way lane entry, front pickup bay, and dedicated outbound clearance point. Multiple trips for the same material may be reserved concurrently, while only the front rover enters the pallet throat; queued rovers remain far enough away for the loaded rover's full hull to leave without intersection. Storage Dome pickup stations are also hull-reserved before travel.

Short-horizon fleet prediction now integrates each rover's real current velocity with the same acceleration limit used by movement. A yielding rover is predicted at its actual held velocity rather than being assumed to jump to cruise speed, reducing false future conflicts and stop/go traffic. Construction-stage commit leadership is claimed by a builder actually working at the site, so a delayed original site owner cannot freeze helpers that have already completed the stage work.

## v0.31.20 construction handoff and four-side builder work

Construction logistics now use one explicit material drop area per construction target. Carriers unload there, clear the apron before releasing it, and builders pick the current stage material from that pile before assembly. If the delivered and already-reserved material does not cover the active stage, one builder may self-haul only the uncovered amount from the stash/storage and rejoins the same drop queue. Each construction step is worked from four perimeter sectors with outside-corner transitions, and the next construction segment is reserved as future geometry before robots plan through it.

## v0.40.0 modular baseline

The source tree is now built through explicit implementation/context clusters rather than one monolithic game target. Start future maintenance from `IMPLEMENTATION_CLUSTERS.md`; it maps common requests to the minimum CMake target and source set to inspect. `BASELINE_CONTEXT.md` is the compact handoff for starting a new chat from this release.

Robot geometry path search is isolated in `game_cluster_pathfinding`. Normal movement uses bounded corridor searches; if a rover cannot resolve or make useful route progress for 2.0 seconds, its local route state is discarded and the next attempt becomes a full-width world-geometry route search. Temporary rover positions are not hard A* obstacles during that recovery search, while the continuous traffic layer and hull checks still prevent physical robot intersections.


## v0.40.33 debug blueprint start

The New Game screen includes a `DEBUG START TEMPLATE` toggle. Enabling it locks the playfield to 64x64, imports the bundled construction layout from the supplied 0.40.32 debug save as fresh blueprints, keeps the selected new-game robot counts and starter resource stash, and opens the playfield paused.

## v0.40.32 local fleet awareness and active-only traffic motion

Every rover now refreshes a local nearby-robot map each safety tick. Active 0.5-second motion proposals are pre-checked against nearby robots' certified motion/braking envelopes before contract arbitration, and coarse route replans include current robot positions as temporary obstacles so an active rover can choose a route around parked traffic. The shared time-indexed contract board remains the final collision authority. Idle and working robots never receive collision-avoidance pull-out commands; only robots that currently have an active navigation request may retreat to resolve an active-active deadlock.

## v0.40.31 contract-gameplay integration fixes

The first contract-certified release exposed overly tight starter parking and inconsistent numerical envelope inflation. Safety radii are now identical across initialization, contract arbitration, terminal holds and the final shared-epoch sweep, so a state accepted as initially safe cannot immediately fail only because a later phase counted the same margin again. Partial safety ticks build contracts for their real lease duration, and certified braking can continue across adjacent branch pieces instead of faulting at a 10 ms boundary.

Robot gameplay now plans for the new authority instead of assuming free steering. Local proposals can pivot in place before translating, target speed is capped by remaining braking distance, deadlock retreat favors a lateral pull-out and retreat commands receive arbitration priority without pre-empting occupied passage reservations. New games use wider starter bays; an untouched v0.40.30 starter fleet is migrated to those bays on load. High-level pickup, delivery and build actions still wait for the rover to physically stop before work time advances.

## v0.40.30 contract-certified robot motion

The former robot traffic resolver, rolling collision-avoidance sampler, direct-drive escape logic and post-move overlap repair are no longer movement authorities. High-level construction/logistics code only submits goals. `game_cluster_robots` collects those requests and one shared safety coordinator commits all robot motion at a common simulation epoch.

Each rover keeps a conservative time-indexed motion contract containing its short command lease, every authorized braking branch, terminal stopping holds and reserved passage cells. Local 0.50 s proposals are regenerated every 0.05 s, but a proposal is only advisory until continuous swept-disc checks certify it against static geometry and the complete active contracts of the other robots. Rejected movement follows the previously certified braking branch; it never invents an unchecked emergency steering command. Idle and stopped robots remain reserved obstacles.

Blocked robots first wait. Persistent wait-for cycles are resolved by certified retreat goals to real free pull-out points; an idle blocker can receive the same yield request. If no safe retreat or detour exists, the robot remains blocked rather than being teleported or nudged through another object. Construction-stage mesh growth and autonomous resource-pallet spawning are gated against active motion/braking/terminal reservations so new geometry cannot materialize inside a certified stopping region.

## v0.40.34 compact rover safety corridor and traversable untouched blueprints

BR-02 and CR-01 now share the same 4.84 m/s default travel pace, including constructor travel between construction work positions. Robot navigation no longer adds the former broad comfort halo: the safety corridor targets roughly 1.5x rover chassis width while retaining the full circumscribed physical hull and numerical collision margin as the hard lower bound.

Blueprints with zero construction progress are non-solid for rover navigation. Constructors may claim a supplied blueprint while traffic crosses it; only when a constructor is actually ready to start the first physical build action does the pre-build safety gate evacuate the footprint. Once visible construction progress begins, the erected geometry becomes a normal rover obstacle and future construction stages remain guarded by the scene-admission safety checks.

## v0.40.35 builder construction motion and adaptive structure clearance

BR-02 no longer cancels its own navigation request while travelling to a construction work bay. Constructors assigned to an untouched blueprint may cross that blueprint on the way to the exterior work bay, while other robots are cleared before geometry appears. The pass-through state ends as soon as visible construction starts: traffic reserves the planned footprint immediately, while hard collision geometry advances only on safety-approved committed construction stages. Rover route planning now prefers about 1.0 m of extra clearance around completed habitat/resource structures and progressively relaxes that preference only when the enlarged halo would close the available passage; physical hull collision remains mandatory at every fallback level. The comfort halo is deliberately not part of hard contract certification, so a rover already standing in a legitimate service bay is never faulted merely because a nearby structure completes.

## v0.40.36 robot personal safety zones

Every rover now owns an additional 0.30 m robot-to-robot personal safety zone outside its hard physical safety envelope. Local 0.50 s previews, coarse fleet-aware route search and shared motion-contract arbitration all respect the combined zones of both robots, so traffic yields before reaching the near-contact state instead of repeatedly accepting and braking tiny movements.

Existing saves that already contain physically clear robots inside the new personal-zone spacing are handled as an escape case rather than being faulted or frozen. An active rover may only maintain or increase separation until it leaves the zone; idle and working robots remain stationary. This specifically prevents the accept/brake flicker reproduced by `0-40-35(1).gmsave`, whose two rovers are only about 2.07 m center-to-center.

## v0.40.37 stationary-rover detour routing

An active rover that remains blocked by a stationary fleet member no longer invalidates and retries the same approach indefinitely. After the normal short wait, the safety coordinator identifies a stationary blocker, searches deterministic left/right bypass waypoints outside both rovers' personal safety zones, verifies that the waypoint is reachable through the existing fleet-aware coarse planner, and temporarily routes the active rover around the blocker. The parked/working rover remains fixed. Once the bypass waypoint is reached, the temporary detour is cleared and the original construction/logistics goal resumes automatically. The same recovery applies to every rover role and also detects stationary blockers that caused a coarse `NO ROUTE` result before a robot contract could name them directly.

## v0.40.38 independent rover tasks and builder-first arbitration

The fleet safety coordinator now keeps BR-02 construction and CR-01 logistics independent during traffic arbitration. Occupied certified passages remain non-preemptible, but when fresh builder/carrier movement requests compete for the same free space, the constructor is considered first. In an active-active deadlock, the lower-priority carrier yields before the builder; logistics reservations and construction ownership are retained while either rover waits, detours or replans.

Short certified motion/braking segments now receive an interior static-clearance sample instead of an endpoint-only test. This fixes the `0-40-37.gmsave` failure where a roughly five-centimetre carrier braking segment was accepted because its far endpoint was clear even though the partial substep crossed conservative obstacle clearance. The final guard can also contain a recoverable integration rejection to the offending rover at its last valid position instead of marking unrelated fleet members as `SAFETY FAULT`; that rover then replans while the other robots continue their own tasks.

## v0.40.39 robot safety stability and performance

BR-02 and CR-01 now use a 6.00 m/s default transit speed. The safety coordinator keeps stationary IDLE/WORK holds persistent instead of rebuilding and following braking tubes every 10 ms, and failed full coarse-route searches are rate-limited rather than repeating up to four 18k-cell A* attempts every 50 ms while the blocking geometry is unchanged. Certified straight-line braking trajectories use larger time pieces with analytic chord-error inflation, substantially reducing pairwise tube work without reducing the conservative occupied envelope.

Constructor work holds no longer fault merely because the conservative envelope of the BR-02's own partially revealed target grows around a prevalidated exterior service position. Initial-geometry validation is only an initialization check; stopped stale-contract states are locally re-established as stationary holds when their current physical position is still clear, while real moving contract violations remain faults.

## v0.40.40 lunar construction economy and aluminum

Construction recipes are rebalanced for the lunar-base economy. Every newly generated habitat, resource structure, corridor, supply channel and walkway recipe now uses exactly 50% of the previous total construction mass (nearest kilogram). The reduced structural budget is split between iron and the new Aluminum (`Al`) resource: lightweight pressure shells, habitat frames, solar arrays and antenna structures are aluminum-heavy, while drilling machinery, industrial plants and landing pads remain more iron-heavy. Copper remains the power/data conductor, silicon the electronics/control feedstock and plastic the pressure-seal/insulation/interior polymer feedstock.

Aluminum is fully integrated into starter inventory, pallets, storage, hauling, recovery, construction UI, savegames and the resource-production economy. A powered ore-processing chain produces aluminum alongside iron, copper and silicon. Save schema 11 persists the eighth resource while remaining backward-compatible with older seven-resource saves; older colonies receive one recalculated starter aluminum pallet so future v0.40.40 construction remains possible. Existing in-progress legacy construction plans are not rewritten mid-build.

BR-02 payload capacity is doubled from 1200 kg to 2400 kg and CR-01 from 1800 kg to 3600 kg. Loaded fleets are normalized to those role capacities.
## v0.40.41 gameplay-aware robot traffic and eight-resource depot routing

Robot routing now distinguishes stable traffic obstacles from genuinely moving fleet traffic. Idle, working, blocked and waiting rovers remain static coarse-route obstacles; a higher-priority BR-02 that is only creeping around its construction work bay is also treated as a stable obstacle by a lower-priority CR-01. Actively moving rovers are no longer baked into coarse A* as permanent walls and remain governed by the 0.50 s prediction and shared safety-contract arbitration instead. A stopped active rover with no accepted motion collapses to one persistent stationary hold rather than regenerating zero-distance braking contracts every safety tick.

Stationary-rover detours are now latched until completion or invalidation. Fleet-aware A* bypass waypoints are followed directly instead of being line-of-sight-shortcut through the stationary rover's personal safety zone, eliminating the low-speed forward/brake flicker reproduced by `0-40-40.gmsave`. Lower-priority carriers can therefore pass a working or near-stationary builder without moving the builder or losing their logistics job.

v0.40.40 expanded the starter depot from seven to eight resources without enlarging its boundary inset. The outer Iron/Aluminum pallet lanes could therefore be physically pickup-valid but full-hull route-invalid on a 64x64 field. New games place the eight-pallet row one metre farther from the south boundary, and schema-11 saves that still match the exact v0.40.40 starter-row signature are migrated one metre north. Persisted carrier pickup stations are invalidated so they are recalculated against the migrated row; player-moved and production-created resource piles are not relocated.

## v0.40.42 full-catalog starter manifest and lightweight rover routing

New games now ship a starter manifest sized from one largest available instance of every habitat/resource structure kind rather than mixing STANDARD habitats with SMALL outdoor structures. Scalable habitat families are budgeted at BIG, non-storage outdoor structures at STANDARD and Storage Dome at HUGE. The manifest also includes one additional full-size Solar Panel and Ore Driller, 240 m of corridor stock, 280 m of supply-channel stock, 160 m of walkway stock and a 30% construction-material contingency. The resulting starter reserve is approximately 168 t of construction materials plus 22 t water, 6 t food and 1.2 t medical supplies, so the complete current build catalog can be exercised from a fresh colony before local extraction is mandatory.

Robot route generation is now staged for predictable frame time. A direct certified line is tried first, followed by bounded one/two-bend doglegs around simple obstacles; only genuinely obstructed routes fall back to a coarse A* search. Clearance probing performs cheap geometry-only passes at 1.00/0.65/0.30/0.00 m structure margins and at most two bounded grid searches (0.30 m and physical-hull-only), using a 1.05 m route grid and 4,200-node expansion cap instead of repeated 18,000-node searches. Normal 0.50 s local motion uses a compact steering set and 0.10 s preview samples; the wide steering/reverse recovery fan is generated only after a rover is actually blocked. The 10 ms shared motion-contract authority and hard collision envelopes remain unchanged.

## v0.40.43 hardware GPU enforcement and bounded multi-core planning

The Linux renderer now requires a direct hardware-backed GLX/OpenGL context. Startup queries `GL_VENDOR`, `GL_RENDERER`, `GL_VERSION` and `glXIsDirect()` after the context is made current. Known Mesa software rasterizers such as llvmpipe/softpipe/swrast are rejected instead of silently running the complete 3D renderer on the CPU. The debug frame reports the active GPU renderer so hardware acceleration is visible in-game.

CPU-heavy rover route/proposal preparation now uses a persistent engine task pool sized to about 50% of the logical CPUs visible to the process. Per-rover short-horizon route/proposal preparation is parallel; shared reservation arbitration, safety-contract acceptance, pose commits and world/construction mutations remain serialized and deterministic. `GAME_CPU_LANES` may explicitly cap the lane count for diagnostics. The default build parallelism follows the same half-CPU policy.

## v0.40.45 save-profiled construction/logistics performance
The `0-40-43.gmsave` regression exposed two independent frame-time costs. Robot navigation was discarding a valid coarse route after small steering deviations and relaunching A* during ordinary travel, while the OpenGL bitmap font submitted one GPU buffer update/draw per lit 5x7 font pixel. v0.40.45 keeps validated coarse routes until the goal/world changes or blocking recovery explicitly requests a new route, uses a wide low-resolution fallback before a fine narrow A* search for long structure/corridor barriers, reuses the dynamic OpenGL VBO, and batches each text label into one triangle submission.

BR-02 construction is now next-stage-fed instead of waiting for the complete remaining project inventory. CR-01 logistics is blueprint-owned and stages the next construction stage for habitats/resource structures, corridors, supply channels and walkways even when construction dependencies are not yet ready. Site choice favors the least-supplied next stage so one corridor cannot monopolize the carrier while other placed blueprints remain empty.

### v0.40.46 FIFO construction logistics
Carrier robots service one global construction queue in blueprint placement order. The fleet stages the complete remaining bill of materials at the oldest unfinished blueprint before moving to the next. This applies consistently to structures, corridors, supply channels, and walkways; build dependencies still gate BR-02 construction, not CR-01 delivery. Save schema 13 persists the cross-kind placement sequence for new saves.

### v0.40.47 - Builder idle parking / site release

- BR-02 constructors with no executable construction stage release their target, work station, route and action state instead of remaining beside the blueprint.
- Waiting/no-task builders return to their dedicated parking bay beside the starter resource stash.
- Parking-return navigation has lower traffic priority than CR-01 logistics, so a waiting builder cannot block material delivery.
- Builders immediately leave the parking return and take a construction target as soon as a dependency-ready, staged construction stage becomes available.
- Parking bays avoid unfinished blueprint envelopes when possible; if the original spawn bay was built over, a deterministic nearby stash-side fallback is selected.
