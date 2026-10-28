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
