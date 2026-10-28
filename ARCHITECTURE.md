# Architecture

## Engine layer
`engine/` owns Linux/X11 windowing, raw input, GPU rendering, math primitives and immediate UI drawing. Game code uses only the public interfaces in `engine/include/engine`.

## Game layer
`game/` owns gameplay, simulation, world geometry and interaction. `GameApp` routes the high-level screen/state flow while focused systems implement placement, construction, networks, POV traversal and rendering state.

## Structural module hierarchy
All crew-accessible pressure structures share `StructureModule` and its `StructureAccessProfile`. The profile is the authoritative source for walkable deck height, crew accessibility, pressure behavior and exterior-gateway permission.

- `StructureModule`
  - `habitats::HabitatModule`
    - one registered instance per `HabitatKind`
    - each habitat has a separate `<name>.cpp` outer module and `<name>_inner.cpp` inner module
    - the outer module owns pressure hull/foundation/glazing/exterior details/entrances
    - the inner module owns the usable deck plus furniture/equipment
  - `corridor_detail::CorridorModule`
    - outer implementation builds the pressure shell and structural ribs
    - inner implementation builds the continuous walkable corridor deck

Habitat types do not duplicate common access/navigation behavior. Their registry entry supplies the concrete outer/inner builders and access profile, then `HabitatModule::instantiate()` creates the complete structure mesh.

## POV floor and access contract
`HabitatMesh::walkable_floor_y` is copied directly from the structure module and is never inferred from arbitrary triangles. The same value is used by POV floor resolution and corridor docking stairs, so the visual basement/deck and character elevation stay synchronized.

POV structure containment uses the projected union of structural shell, glazing and foundation meshes. This is important for mixed-envelope structures such as the Bio Dome. Structural borders remain solid except at legal connected doorway portals. Corridors use the deck height declared by `CorridorModule`.

## Extension direction
Add a new habitat by creating its outer and inner builder files and registering one `HabitatModule` entry. Common access, floor, connection and POV behavior should remain in the shared structural system rather than being reimplemented per habitat.

## 0.30.0 baseline invariants
The 0.30.0 baseline treats structural pressure boundaries as topology, not decoration. Corridors terminate in solid pressure bulkheads unless they are connected, docking sleeves carry redundant visual seal/flange geometry, and no unpressurised walkway may create a corridor penetration. The only exterior crew transition into the pressurised colony network is a connected `AirLock`.

All crew dimensions come from `geometry_standards.hpp`. `kAstronautScale = 0.80` is the canonical character scale for POV and future astronaut instances, so movement collision, eye height and clearance remain consistent.

Rendering keeps construction state explicit through mesh color/weight rather than materials. In the current development phase habitats, corridors, resource structures, supply channels and walkways never emit filled structure surfaces: unbuilt target geometry is blue, installed geometry is yellow, and completion switches the full engineering mesh to substantially heavier green lines. Shared detail passes still provide load anchors, service trunks, diagnostics, panel seams, supports and maintainability hardware without duplicating those rules in every concrete structure module.

## Autonomous construction logistics (0.31.1)
Construction placement now creates unfunded blueprints. `BuilderRobotState` remains the construction-stage gate, now as a 1-6 unit fleet: each BR-02 selects an unclaimed incomplete target, fetches the exact next-stage resource bundle from physical ground/storage inventory, transports it to the site, and advances one stage only after delivery and on-site work. Habitats/resource structures retain ten stages, corridors five stages, and supply channels/walkways three stages.

New games let the player ship 1-6 BR-02 units (default 1); the selected fleet spawns in marked bays beside a calculated corner starter depot. The reserve covers one standard instance of every current habitat and resource structure (with redundant solar/ore extraction), plus practical corridor, supply-channel and walkway allowances and consumable contingency. Mature facility chains return Fe/Cu/Si, polymer, water, food and medical supplies to physical collectable output piles; stable industrial production is gated by completed power/storage and the corresponding processing facilities.

## BR-02 physical navigation and work animation (0.31.2)
BR-02 movement now validates a 0.48 m chassis radius rather than reusing only the narrower astronaut center path. Candidate motion is checked against the pressure-shell access rules at multiple envelope offsets, incomplete habitat/resource footprints, physical ground resource packages, deterministic Storage Dome box poses, and other robots. Local steering samples a broad fan of alternate headings so pallets and structure corners are routed around rather than intersected.

Construction logistics are visually staged without weakening resource gating. A fetch transaction runs a pick-up animation before cargo is removed from its physical source; site delivery runs put-down before cargo enters the site buffer; a funded construction step then runs material application, welding and build-up before the step is committed and its resource bundle is consumed. Animation fields are transient, so loading an existing schema-3 save safely restarts only the visible phase while preserving cargo/site-buffer accounting.

## Builder motion and construction visualization (0.31.3)
BR-02 navigation uses scored local steering, turn-rate/acceleration limits and stand-off interaction radii to avoid side-to-side oscillation at construction footprints. During the welding/build-up phase a robot follows a collision-safe work perimeter around the current target while its independent arm turret remains focused on the work. Construction rendering keeps the final mesh as a faint reference envelope at every incomplete stage and overlays the currently assembled geometry at higher intensity.
## Constructor/carrier logistics and partial-mesh collision (0.31.4)
`BuilderRobotState::role` separates BR-02 Constructor and CR-01 Carrier behavior while sharing the same collision/navigation framework. Constructors exclusively claim construction targets and own each target's delivered-resource buffer. CR-01 does not construct: it services those buffers with a larger/faster logistics chassis, preferring completed Storage Dome inventory over loose ground stock. When no construction delivery is pending, CR-01 sweeps loose resource packages into available Storage Dome capacity; this makes an operational dome the colony's authoritative physical depot while preserving global resource accounting. Constructors wait for logistics support first and fall back to self-haul after a short unsatisfied interval.

Incomplete structures no longer expose their final placement footprint as a solid vehicle obstacle. Navigation derives collision from the currently revealed construction triangles for habitat foundations/shells/interiors/details/glazing, resource structures, corridors, supply channels and walkways. Unbuilt blueprint volume is traversable; each erected wall/hull/machine segment becomes an immediate obstacle. Completed structures continue to use the pressure/door/stair movement rules. Simulation time has a persisted three-state multiplier (`Normal=1x`, `Quick=2x`, `Fast=4x`) applied to colony status, production, robot logistics and construction while input/camera movement stays unscaled.


## v0.31.5 construction safety
Construction steps use a pre-commit fleet evacuation gate: blueprint space remains traversable until its mesh exists, but a new stage cannot appear around a robot. Before walls/body/network geometry is revealed, affected robots are moved to a safe exterior position and the step commits only after the fleet is clear.

## v0.31.6 planned fleet traffic

`BuilderRobotState` keeps transient planned-route state. `update_builder_robot()` plans each trip before travel with grid A* over current walkable geometry, smooths the resulting polyline, then reserves the remaining route as a fleet traffic lane. Subsequent robots must plan around that reservation or wait; narrow corridors therefore operate as one-at-a-time lanes. Route state is transient and is safely recalculated after save/load. New-game fleet composition independently supports 1-6 constructors and 1-6 carriers.

## v0.31.7 cooperative construction teams
Construction targets support small cooperative BR-02 crews (up to all six shipped builders on habitats/resource structures and two on narrow routed network items). The first assigned constructor is the site coordinator: its site buffer is authoritative and only it may commit a construction stage. Helper builders share that buffer, can self-haul into it, occupy offset work positions and run synchronized build animations. Carrier assignment is per site rather than per builder; several CR-01s can serve one site when its missing mass exceeds already reserved carrier capacity. Inbound cargo is counted once before any additional trip is dispatched. CR-01 role speed is defined as 1.10x the BR-02 transit speed and normalized on save load.

## 0.31.9 synchronized material-demand ledger
Construction logistics use one authoritative ledger per target and construction step. The owner constructor's `site_buffer` is delivered stock; every robot's `logistics_reservation` is explicit in-transit stock; `construction_site_remaining()` subtracts both before another robot can claim material. CR-01 dispatch therefore reserves an exact material/amount before path planning and other robots immediately see the reduced unassigned demand. BR-02 fallback hauling uses the same ledger and can only claim still-unreserved demand.

Physical pickup routing is separated from logical reservation. Ground resource stacks are approached through collision-free service points around the package envelope, and the initial depot is arranged as accessible aggregate stacks rather than a dense pallet block. Carrier trips use one material type per reservation, allowing multiple carriers to parallelize different deficits without forcing one vehicle through several pallet lanes before it can make a delivery.

Robot selection is a first-class `SelectionKind::Robot`. The selection panel reads the same runtime state used by the AI and exposes role, condition, mode/action, route state, cargo, reservation and construction-site ledger values. Save schema 5 persists robot condition and logistics reservations while retaining backward load compatibility.


### Construction status overlays (v0.31.9)
The playfield UI projects a compact persistent status frame above every incomplete construction target. The frame is derived from the same ConstructionResourcePlan and shared robot site ledger used by BR-02/CR-01 logistics, so remaining demand stays synchronized with delivered and reserved resources.

## 0.31.10 logistics liveness and presentation
The construction ledger distinguishes active transport from stale assignments. A logistics reservation contributes to `construction_site_reserved()` only while its robot is actively fetching/delivering and making viable progress; an empty transporter blocked for several seconds releases its reservation, so site demand becomes claimable by another CR-01 or a BR-02 fallback hauler. Builders remain construction-first but claim unassigned deficits after a short hand-off interval, guaranteeing logistics liveness even if the carrier fleet is busy or unable to reach a source.

Starter inventory placement is stash-relative rather than world-absolute. The seven material stacks form a compact service row beside `starter_stash_center_`, which keeps the physical reserve inside and aligned with the rendered depot on every supported field size. Load migration recognizes the older absolute-Z starter-row signature and relocates only matching starter piles.

Robot physics continues to use authoritative navigation positions, while rendering follows a transient smoothed pose. Route conflicts use a short braking/hysteresis interval before replanning, which prevents one-frame left/right route oscillation without weakening collision or reservation guarantees. Construction overlays read the same shared ledger and expose `NEEDED` (uncovered remaining mass), `SHIPPED` (delivered plus active in-transit mass), and construction progress.

## 0.31.11 fleet independence and staged materials

Fleet path planning is static-world authoritative per robot. Other planned routes contribute a soft separation cost only; collision safety is enforced at motion time with short-horizon reciprocal steering. This prevents one inactive route from owning a corridor and deadlocking unrelated robots. Delivered site-buffer material is rendered as non-interactive staged boxes until consumed. Construction status frames render in global logical UI coordinates using the configured UI scale.

## 0.31.12 blueprint-only construction rendering
The incomplete-object render path never emits filled surface triangles. Full target geometry remains visible as a subdued engineering-blue wireframe and the construction sequence reveals already erected geometry using a brighter/heavier blue stroke. This policy is shared by habitats, resource structures, corridors, supply channels and walkways; completed assets retain the normal material rendering path.

## 0.31.13 independent robot runtime and liveness
`update_builder_robot()` separates authoritative simulation movement from presentation. Robot simulation runs in bounded substeps, while each `BuilderRobotState` owns a transient critically damped visual position/velocity so QUICK/FAST simulation never exposes large solver steps directly in rendering. Routes remain per-robot and are not globally reserved.

Every pickup/delivery/build operation acquires a stable interaction station. Ground-pile stations are distributed around the package envelope per robot, construction service positions are distributed around the target perimeter, and a persistent service-slot offset lets only the stalled unit select a new approach after watchdog recovery. A* may terminate on a nearby coarse grid node only when a final clear segment to the exact stand-off point exists; intermediate waypoints use a safe pass radius and sharp turns reduce forward velocity to prevent waypoint orbiting.

Construction-stage safety tests only mesh triangles newly revealed by the pending stage rather than all already-visible geometry. Fleet members intersecting that future mesh are evacuated under exclusive movement control to generously separated exterior points before commit. A final validated relocation fallback guarantees that no robot can be sealed into newly appearing geometry even under pathological dynamic traffic.

Physical site staging remains authoritative: PUT-DOWN adds kilograms to the shared site buffer, the renderer shows those kilograms as resource boxes beside the construction site, and boxes disappear only as construction consumes the corresponding buffer mass. Robot selection continues to expose the individual role, task/mode/action, condition, cargo, route state, reservation and site ledger.


## 0.31.17 continuous traffic planner
`BuilderRobotState` now stores a dense route plus cumulative arc-length metadata instead of a list of fixed-duration reservation slots. `update_builder_robot()` first computes a static-world path with Theta*-style line-of-sight relaxation, performs an additional visibility pass, rounds only collision-valid corners, and densifies the result for continuous following. Several traffic-weighted alternatives are evaluated before departure; full candidate trajectories are sampled against the current fleet itinerary so independently assigned robots prefer simultaneous non-conflicting lanes.

Runtime coordination is velocity-based rather than steering-based. A PIBT-inspired dynamic priority age plus a deterministic tie-break decides right-of-way, while a short-horizon reciprocal speed governor chooses the highest safe continuous speed along the already-planned route. The local layer may slow or stop, but it never changes left/right steering, backs away, or inserts orbit behavior. A prolonged yield invalidates the whole route and requests a new traffic-weighted lane. Hard SAT chassis checks remain the final invariant for both translation and rotation.

The former critically damped render follower was removed because the authoritative route itself is continuous; rendering now uses the exact collision-validated pose. Robot simulation runs in 90 Hz bounded substeps with a 64-step frame guard, preventing FAST-mode frames from dropping unprocessed robot time. Completed and incomplete construction geometry is mesh-only; there is no structure surface/material rendering in this version.
