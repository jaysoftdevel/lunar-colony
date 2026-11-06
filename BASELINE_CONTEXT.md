v0.51.14: Structure power switches persist in schema 41. Switching off equipment removes its electrical demand or supply while connected conduits continue carrying services. The HUD separates power delivered and requested from the net battery rate. Conveyor blueprints show the first section at the driller port before placing the full run.

v0.51.13: A driller without a completed conveyor exposes its outdoor raw ore crates to carrier bots. Processing Facilities and Factories are preferred when ready. Storage Domes with a completed loading gate accept raw ore as a fallback. Raw ore occupies storage space but stays separate from construction resources. Schema 40 retains ore on a carrier and in a dome.

v0.51.12: Ore drillers survey location-dependent iron, copper, silicon and aluminum concentrations and extract separate raw ore boxes. One directed conveyor per driller carries a box at a time to a Processing Facility or Factory. Delivered crates become refined resource pallets when the receiver has power and breathable air. Belts do not provide utility services. Save schema 39 persists ore reserves and cargo in flight.

v0.51.11: Stage 3 Free Play opens a separate Debug Mode menu. Template starts the existing prebuilt base with its rover fleet and normal construction for new builds. Sandbox retains the empty 128 × 128 instant-build start. Test Astronauts opens a reserved empty field. Save schema 38 stores the Stage 3 mode. Previous saves use the existing sandbox behavior.

v0.51.10: Stage 3 is an empty 128 × 128 debug sandbox. It is selectable in Free Play when Debug Frame is enabled. No template, fleet or starter cargo is loaded. All placement tools create completed instances with no material cost. Explicit saves remain supported. Pending construction in an older Stage 3 save completes on load while preserving invested materials. Stage 2 retains its existing optional blueprint template.

v0.51.9: The two build tabs group related tiles under compact labels and separators. The status bar shows stored material mass as a percentage of the combined completed Storage Dome capacity. Each completed dome above 85% has an individual overhead capacity icon. Status bar and inspector rates sample at 0.5 real-second intervals. Inventory and simulation updates use their normal cadence.

v0.51.8: A completed antenna is required to call a trader from a powered pad. Structure demolition removes its attached routes, including dependent corridor branches and service runs on removed corridors. During structure placement a clear footprint near a free route end snaps and creates a normal connection blueprint. The optional debug template is loaded at runtime and is no longer copied by the build. Save schema 37 remains readable without conversion.

v0.51.7: Stage 1 starts new mission cargo empty and budgets selected structures from the Stage-2 construction recipes. Auto-add raises each material to the planned requirement and preserves extra cargo. Stage 2 adds exterior oxygen storage on the supply network, trader cargo remaining by resource, manual camera controls during lander approach, and parking bays beside the ramp. Schema 37 saves structure counts and oxygen stock and preserves the apron for in-progress or already parked older deployments.

v0.51.6: Persistent rover geometry blockages outline the responsible structure in red. Physical pickup and unload outcomes share an operation retry counter with the progress watchdog. Each rejected unload completes a fresh attempt. On failure five, the stopped rover is placed on clear ground after checking the current structures, terrain bounds, other hulls and active motion reservations. Cargo, site reservations and job state remain authoritative. A rover holds if no safe destination is available. Operation retries are transient like the existing navigation watchdog.

v0.51.5: Stash stacks are placed in paired columns with 0.60 m across a pair and 0.64 m between rows. Wider access aisles separate pairs. Stage-2 rovers merge onto the ramp at two-second intervals and follow a clear cross lane into assigned parking bays. Park deep bays first so no rover needs to pass through an occupied bay. Schema 36 persists the common deployment clock. Older saves preserve parked rovers and queue those still aboard, with automatic stash consolidation where space allows.

v0.51.4: The lander has a beveled garage portal, octagonal chassis, tapered command cabin, exposed pods, six landing struts and a side crane, all drawn with mesh edges. Stage 2 now places a 2.5 m stash marker and arranges cargo around it in columns of up to eight individual boxes with rover aisles. Stack masses are authoritative for logistics. Schema 35 saves stack membership and builder salvage progress. Salvage routes an idle builder to a service point and requires 18 seconds of stopped cutting work. Ship modules detach as cutting advances, then the completed job deposits the same 5,400 kg yield. The source ship remains a navigation obstacle until its recovered stock is safely deposited. Legacy saves remain readable.

v0.51.3: Stage-2 status bar uses two rows (92 logical px): colony day/cash, stored power/rate, volume-weighted cabin air, water tanks/rate and crew above; all eight material/cargo resources plus bottled O2 and stored H2 below. Every resource is named, shows kg, and has an exact-quantity tooltip. O2 is the separate export buffer of completed generators, not cabin air. H2 sums completed tanks and oxygen generators, excluding vehicle fuel allocations and blueprints. Both ships submit line/edge topology only, without filled faces or textures, in every flight/deployment phase; trader source crates retain their normal independent resource rendering. Existing logistics, ship timelines, save schema 34 and economy remain unchanged.

v0.51.2: Both native ship meshes follow the approved industrial concepts: wide six-leg robot garage/ramp/roof cargo and side crane for the lander; elongated merchant with cockpit, open freight bay, aft insulated gas pods and engines. Before descent the player clicks a touchdown site and a clear, nearby stash zone (right click returns to touchdown selection). UNLOAD atomically drops the exact remaining manifest into separate physical, selectable material packages in that zone; only the ramp/robots animate. Package capacities remain the existing material-specific 80-200 kg, not one box per kilogram. After the entire selected fleet is deployed, DECONSTRUCT / SALVAGE (or Delete on the lander) removes the hull and adds Fe 2400, Al 1800, Cu 600, Si 300 and Plastic 300 kg once, in physical boxes. The selected stash reserves enough package slots for original cargo plus salvage; stock and normal robot construction remain authoritative. Empty zone clicks select the stash, while box clicks retain individual box selection. Schema 34 persists placement and salvage; older lander saves pause for a clear stash location and repack existing aggregate pallets without adding inventory. Existing trader hauling/payment and mission planning remain intact. Resource obstacles use a spatial index with unchanged exact clearance predicates to support large physical depots.

v0.51.1: Stage-2 powered, completed Landing Pads can call a trader. One visit remains latched through powered approach, editable manifest, robot loading, receiving-bay clearance and departure. Materials/supplies, separate bottled oxygen and actual tank/generator hydrogen can be sold (20 t per visit). Confirming a sale atomically reserves actual stock into physical source lots, normal logistics cannot spend it again, and existing fleet safety arbitration moves export robots. Cash is the existing planning_.funds_eur account, credited once after departure and shown in the Stage-2 HUD. Schema 33 persists lots, rover custody, ship progress, quote and oxygen export buffers; schemas 1-32 remain readable. Calling the first trader enables surplus oxygen bottling (500 kg per standard generator, size scaled), using actual water/power electrolysis only after connected cabin air is full. Never sell cabin oxygen or already allocated vehicle fuel.

v0.51.0: Stage-2 lunar deployment begins aboard a cargo lander. A 79 s powered terminal descent and 4 s engine-safe dwell precede explicit UNLOAD. A two-section ramp opens, the exact selected fleet disembarks sequentially, then a 2.5 t-rated crane transfers the exact selected manifest into physical depot pallets beside the lander. Inventory is credited only on crane release. No corner stash or surface robots appear during approach. Existing established colonies and Stage-3/debug starts retain their saved layouts. Schema 32 resumes every phase and persists Stage-1 builder/carrier counts (1-6 each), which now contribute to payload mass and mission procurement cost. Preserve the ordinary robot/safety/construction behavior after deployment.

v0.50.17: LH2 tanks now physically sized 2t/8t with spherical standard vessel and fully scaled small tier. Lunar surface-return vehicle demands 4.2t rocket H2 and 1.15t lander H2, plus displayed LOX requirements 25.2t/6.9t; Stage-2 oxidizer and launch still not simulated. Schema31 backward compatible.

v0.50.16: Stages 2/3 gain a buildable H2 storage tank (400/800 kg), connected electrolysis hydrogen routing, landing-pad rocket/lander fueling allocations (80/30 kg), selection-panel fuel controls, and schema-31 persistence. Stage 1 mission/contract progression unchanged.

v0.50.15: 20 additional one-time Stage-1 contracts, EUR 94.35M prelaunch commercial base funding, conservative full research+vehicle+minimum cargo budget, schema-30 12-to-32 contract migration.
## 0.50.13 baseline — Stage-1 interaction consistency

Stage 1 uses the same mental model for contract cards and research nodes: single click selects/inspects and double-click invokes the primary guarded action. The double-click detector is transient in `GameApp` (group/item, 410 ms and a 22-logical-pixel radius). It does not change the save format. Contract completion remains single-use; research prerequisite/funding checks and one-time purchase checks are unchanged. Mission cards navigate to the current facility, while confirmation/funding/launch remain explicit buttons. The selected navigation tab and primary-action area are visually prominent. Stage 2 and Stage 3 remain the previous baseline.

## 0.50.12 baseline — one-time customer jobs and resumable planning

All Stage-1 customer contracts are one-time, not repeatable. An earlier repeatable save entry with completed_count>0 is treated as Completed even if its previous status was Available or Active. Contract cards use green=completed, gold=ready, blue=active, cyan=available and gray=locked. The 16-node tech tree and one-time lunar mission/Stage-2 cargo handoff remain unchanged. Selecting Mission Prep resumes the previously chosen facility; an Overview button and a Continue at Required Facility shortcut are available. Rocket/lander/cargo/funding/mission step are kept in `planning_` and serialized by save schema 29. Do not reset `mission_facility` on tab navigation or after leaving the campus.

## 0.50.11 baseline — one-way career lunar launch

Stage-1 lunar mission is a **single-use** flight: launch at Launch Complex after final confirmation switches to the ordinary Stage-2 robot-preparation world, spawning one builder and one carrier and importing the selected paid manifest into both the central resource inventory and eight starter depot piles. Preserve `planning_` after deployment so research/contracts, paid cost, cargo and completed launch survive schema-29 saves. Previously completed Stage-1 saves migrate to deployed Stage 2 on load. Do not repeat a lunar mission or launch payout. Commercial customer contracts are now one-time only (since 0.50.12); direct Free Play Stage 2 must keep its existing starter stock. Stage 3 unchanged.

0.50.8 update note: Stage-1 Tech Tree is now a navigable dependency map;
research catalogue and Stage-1 economics unchanged, Stage-2 robot/colony
baseline remains untouched, Stage-3 full-base start unchanged. Save schema 29.

# game v0.50.7 baseline context

- New Game has two independent modes. **Career / Campaign** starts at Stage 1 Planning (fixed 128 × 128 space-company site); **Free Play** allows choosing Stages 1/2/3. Existing saves are treated as Free Play unless schema 29 stores a mode. Career mode automatically transitions into Stage 2 on its unique lunar launch, shipping the paid manifest into Stage-2 resource counters and ground pallets.
- Stage 1 campus remains physically clickable, with the existing research, contracts and facility mission workflow untouched. Its Earth-site playfield size is fixed, but camera controls include wheel zoom, MMB orbit, RMB pan and WASD camera movement.
- Stage-1 visual rendering now uses an optional OpenGL procedural, per-pixel high-frequency world-space material shader (grass / sand / asphalt / concrete / water / roofing), sunlit façade faces and geometric ground shadows. The cross-platform base renderer falls back to plain 3D triangles on Windows/D3D12. No imported photograph textures and no true realtime shadow map are claimed.
- Main menu and New Game use a layered, scaled, illustrated space/launch visual theme, validated by the standard UI matrix.
- Stage 2 and Stage 3 construction, resources, and navigation are unchanged. Stage 2 alone retains adjustable playfield dimensions; Stage 3 remains the completed debug start.
- Save schema 29 adds one mode byte after Stage-1 mission state. Older game save schemas remain readable; existing debug templates are unchanged.

- Standalone C++20 cross-platform game with separate `engine/` and `game/` trees. Linux uses X11/OpenGL; Windows uses Win32/Direct3D 12.
- Root `VERSION` is the release source. Linux builds to `build/linux/bin/game`; Windows builds to `build/windows/bin/game.exe`. The paired `build.*`, `run.*`, `test.*`, and `dev.*` scripts expose matching workflows.
- All visible CPU lanes remain available through the persistent task pool. `GAME_CPU_LANES` is an optional diagnostic cap. Per-rover planning runs concurrently; arbitration, pose commit and resource/world mutation remain deterministic on the game thread.
- Render presents with swap interval 0 by default (VSync off) so the GPU can run at full host throughput; `GAME_VSYNC=1` re-enables synchronized present.
- The runtime loop is uncapped by default (no forced 1 ms idle sleep). Optional `GAME_FRAME_LIMIT_FPS=<n>` applies an explicit limiter when profiling or thermally constrained.


## v0.50.5 top-level game content

- The product has three independent top-level game phases: Planning, Preparing Base with Robots, and Full Manned Moon Base.
- Stage 1 is an independent company/mission-planning simulation with contract funding, a shared cash balance, planning-time progression, a 16-node prerequisite technology tree and a freely viewable 3D company launch campus. The technology UI is a directional eight-level tree that rotates vertically on compact/high-zoom layouts. It must never silently run Stage-2 robot/world simulation.
- New Stage-1 games start on the Company Site. Campus facilities route into the existing Contracts, Tech Tree or Plan Mission views; they do not create duplicate planning state.
- Stage 2 remains the complete pre-0.50 robot-preparation game. Stage 3 now starts from the Stage-2 128 x 128 debug layout with all habitats, resource structures, corridors, utilities and walkways already construction-complete, then runs on the normal playfield surface.
- Stage-1 research culminates in Stage-2 Module Competence and Robotic Lunar Base Mission Architecture; upon a one-time paid launch, a fresh Stage-2 simulation is created with the exact shipped payload (without merging the underlying simulation engines).

- Stage 1 now includes a third `PLAN MISSION` surface. The player chooses a launch vehicle and lunar cargo lander, adjusts each Stage-2 material/consumable quantity, and pays a combined vehicle + cargo procurement/integration cost from company funds.
- The default mission manifest is derived from the real Stage-2 construction-cost model and contains the minimum practical robotic-base package (power/water chain, pressurized core, six-person quarters, landing pad, robot workshop, corridors/utilities/walkways, contingency and survival consumables). Funding is blocked when any minimum is missing or the selected vehicle pair cannot land the mass.
- Mission commitment also requires Stage-2 Module Competence and Robotic Lunar Base Mission Architecture. A paid mission is frozen and survives save/load.
- New Game still defaults to Stage 2. Save schema 28 persists the Stage-1 planning view, ordered facility mission-preparation gate, selected facility, rocket/lander mission selection, lunar cargo manifest and mission payment; schema 26/27 saves migrate into the workflow and schema 1-23 saves still load as Stage 2.

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

## v0.50.5 Stage-1 mission workflow
- Mission preparation is strictly ordered: Research -> Design -> Test -> Refine -> Plan -> Confirm/Fund -> Build -> Integrate Cargo -> Final Confirm -> Launch.
- Facility ownership: Research & Engineering (Research/Design/Refine), Propulsion Test (Test), Mission Control (Plan), Program & Contracts (Confirm/Fund), Vehicle Assembly (Build), Cargo Integration (Integrate), Communications (Final Confirm), Launch Complex (Launch).
- The Company Site highlights the next required facility. Facility actions reject attempts from the wrong facility or out-of-order gate.
- Program authorization deducts the mission cost once; vehicle build, cargo integration, readiness review and launch remain separate required gates after payment.

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

## v0.50.9 — Stage-1 customer-project and research-evidence integration

Contract catalog IDs, advance/final payment amounts and duration data remain stable for historical saves. Existing `PlanningContractProgress` persists the progress and completion history; a project at or above duration is now *ready for explicit delivery*, not automatically paid. Research evidence is computed deterministically from `completed_count > 0`, mapped to a linked technology, with a maximum 35% discount from distinct customer projects; no new serialized fields are required. Final lunar customer delivery is a special event settled by the ordered Launch gate, not the planning timer. The Contracts view is now a selected customer-brief workflow rather than 12 compact list rows, and the Site view indicates the immediate useful action before the eventual mission gate. Schema 29 is retained; older Stage-1 contract runs may have ready-for-submission reports. Stage 2 and Stage 3 are unaffected.


### 0.50.10 Stage-1 campaign balance and coaching

- Adds a persistent context-sensitive **NEXT STEP** navigation shortcut and direct facility action on the 3D campus; never purchases or submits work without a separate click.
- Commercial projects: faster completion for non-tutorial jobs; use **ADVANCE TO NEXT REPORT** to advance exactly to the nearest active project delivery and pause.
- Technical evidence: first completed customer project provides a 20% discount to its target technology (distinct evidence capped at 35%). Previously completed projects still apply without a save schema change.
- One-time first-customer completion premium (+20% final payment for projects 2–11), plus transparent investor reputation awards when unique completed projects reach 3, 6, and 9 (EUR 750K, 1.5M, and 3M). Repeat projects pay their normal price and remain available to prevent deadlocks.
- Contract list ranks work ready for submission, active jobs and first-time relevant opportunities before repeat work; research shows a funding shortcut when unaffordable.
- Stage-2 and Stage-3 mechanics are unchanged. Save schema 29 is preserved; old contract IDs and progress are retained.
