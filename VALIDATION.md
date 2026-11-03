# v0.43.15 validation (2026-11-01)

- Optimized Linux build completes for the game executable, UI regression executable and navigation regression executable after the new shared solar/status state is compiled through all dependent clusters.
- New `colony_hydrogen_solar` regression verifies east-to-west 3D stellar motion, ~60-degree southern noon elevation, below-horizon night, the stable nonvisualized future-render position, tracked irradiance, physical collector-head rotation, component-local H2 isolation and fuel-cell energy balance.
- Oxygen regression verifies electrolysis H2 production at 1:8 H2:O2 mass ratio and retained generator buffer. Supply save/load regression verifies persistent hydrogen inventory under schema 20. Existing schemas remain accepted by the conditional supply reader.
- UI regression passes 475 resolution/zoom/screen cases plus 44 structure-warning cases. Oxygen-generator H2 output/buffer and Hydrogen Generator output/fuel telemetry are covered; the compact `NO CONNECTED H2` warning keeps panels clear of the build bar at maximum UI scaling.
- CTest groups 1-21 pass together in 31.22 s. The two long construction groups (`navigation_construction`, `navigation_late_corridor`) also pass in a separate CTest invocation (10.57 s), covering all 23 registered tests.
- Existing progression stages, robot logistics, collision/safety, air-lock behavior, utility topology and construction ordering are not intentionally changed.

# v0.43.14 validation (2026-11-01)

- GCC/C++20 build passes with `GAME_BUILD_NAVIGATION_TESTS=ON`. The resource simulation reuses the existing connected-component topology and does not change robot pathfinding, collision arbitration or build sequencing.
- Colony progression regression builds a completed Stage 1 resource network, extends the same physical utility component through an Air Lock/Oxygen Generator for Stage 2, then verifies a Canteen, Mid Crew Quarters (6-person capacity), Landing Pad and >=95% breathability unlock READY FOR HUMAN CREW. Fresh colonies report 0/0 crew.
- Oxygen regression verifies a nearly/full pressurized base still incurs bounded seal/hatch leakage and therefore nominal oxygen-generator/water use. Existing supply-network isolation tests continue to confirm disconnected components cannot exchange power or water. Priority/FIFO construction regressions remain unchanged and pass.
- Progress-tree UI regression covers all five UI zoom levels across five resolutions. The checklist stays between the top status strip and bottom build bar at the maximum 200% scale, is absent when the debug frame is enabled, and its reserved rectangle prevents construction-status frames from overlapping it.
- The status HUD preserves signed battery/tank net-rate readouts and now adds current POWER LOAD / WATER USE; demand remains a requested-load quantity even when generation/storage cannot fully serve it.
- Full CTest suite passes: 22/22 groups in 32.06 s, including UI scaling, progression, oxygen, isolated supply components, construction priority/queue flow, navigation, gates, workshops and construction.
- Shell installer validation applies the package to a clean v0.43.13 tree outside Git through a path containing spaces, reproduces all 21 update-owned files byte-for-byte, preserves an unrelated local `.gitignore` edit, reports zero changed files on repeat application, rejects a wrong source version without writes, and rejects a conflicting patched hunk atomically before VERSION changes. `apply.bat` uses the same `git apply --check`/apply sequence; it is included for Windows but was not executed in this Linux validation environment. Neither installer builds.

# v0.43.10 validation (2026-11-01)

- GCC/C++20 UI build passes using the headless Application harness. Gameplay, supply simulation and render backends are unchanged; native OpenGL/Direct3D display execution remains unverified.
- 75 new status-bar cases cover five zoom levels, five resolutions (720p to 4K), and empty/mixed/full readings. A standard and small battery at half charge show 45.0/90.0 kWh and a 50% bar. Over-capacity values clamp to 100%; unfinished batteries are excluded.
- Indoor-air fixture combines four completed interiors, including a disconnected room and a Storage Dome, into a volume-weighted 25.0% bar. Unfinished interiors, exterior Robot Workshops and large corridor volumes do not affect the combined indoor-structure value. Empty colonies show AIR -- and zero fill; full interiors show 100%.
- Existing 75 priority control cases, 450 screen/zoom/resolution cases, and 42 warning-panel cases pass. Inspected software rasterizations of the 50%-charge / 25%-air status bar at HD and downscaled 4K; proportional positions and bars agree. These are layout checks, not native driver output.
- Installer checks successful v0.43.9 application and full changed-file summary, repeat application with zero changes, paths with spaces, local .gitignore preservation and rejected-version atomicity. Additional source-conflict and missing-manifest checks require zero modifications and an explicit zero-file result. The package contains update.patch, apply.sh, apply.bat and update-summary.txt; neither script builds.

# v0.43.9 validation (2026-11-01)

- Built gameplay and UI regression executables with GCC/C++20 using the headless Application harness. No platform or graphics backend changes. Native GPU/interactive display execution remains unverified.
- Reproduced the supplied 0-43-7.gmsave stall on v0.43.8: CR-01 remains at (-26.146,-30.675) with 21 kg and NO SAFE ROUTE for 180 simulated seconds. Direct callback diagnostics confirm the exact delivery point (25.431,-11.521), its zero-length sweep and the starting position are physically clear; the old D* initializer cannot attach that destination to its coarse grid.
- Final replay runs 900 simulation seconds without a safety fault. The first saved load reaches its site after approximately 27.45 seconds; repeated outbound/return trips finish the original supply-channel task, and the colony continues later jobs. A carrier legitimately back at the original pickup position after further trips is accepted; the replay separately tracks actual travel, cargo arrival and target completion.
- The long replay records occasional simulation wall-time spikes (about 107 ms maximum during this run); these include work outside global graph propagation and are not a native rendering/FPS result. The new 4 ms propagation deadline is an advisory search slice, not a guarantee that an entire frame stays below that time. The earlier congested-traffic limitation is not claimed universally solved.
- Seventeen gameplay groups pass: global routes, debug moves, construction priority, mixed supply network, supply, oxygen, fault recovery, interactions, gates, extensions, utilities, geometry, fleet traffic, workshops, construction, late corridor and fault-injected construction. The standard six-robot construction case completes all 7 targets after 611.878 simulated seconds, matching its previous completion sequence.
- Focused global-route checks cover a narrow valid destination missed by the old sampling phase, departure from that bay, a barrier requiring travel outside the old local window, pending-state reuse near the map edge, and a changed obstacle. Every returned segment must pass the independent geometric clearance callback.
- UI recorder passes 75 priority/control cases, 450 existing resolution/zoom/screen cases and 42 structure warning cases. Existing debug move/cancel and priority controls remain functional.
- Installer checks v0.43.8 application, repeat application, paths with spaces, local .gitignore preservation and conflict atomicity. The ZIP contains one game-update-v0.43.9 directory with update.patch, apply.sh and apply.bat. Neither installer builds.

# v0.43.8 validation (2026-11-01)

- GCC/C++20 gameplay and UI builds pass using the headless Application harness. Native GPU/OpenGL/Direct3D display execution remains unavailable; this change does not modify the platform/render backends.
- Debug-order regression moves a constructor and carrier for 40 simulation seconds around completed machinery without safety faults or robot overlap. Both arrive and hold their locations for three more seconds. Retargeting starts them moving again, then real builder/carrier assignments cancel both manual orders on dispatch. Non-debug, busy and off-map orders are rejected; disabling debug releases a custom order.
- Priority regression confirms newest promotion first, empty-carrier queue retargeting, restoration of normal ordering, promoted builder selection over a nearer target, stable IDs after index deletion, rejection of completed objects and schema-18 save/load preservation. Active material transactions retain existing safety/accounting behavior.
- Continued the supplied schema-17 0-43-6.gmsave for 20 simulation seconds with no safety fault. Sixteen gameplay groups pass, including debug orders, priority, geometry, gates, storage/workshops, fault recovery, loaded-carrier recovery, fleet safety, construction, oxygen and supply tests.
- UI recorder passes 75 priority control/marker cases across five target domains, five zoom settings, and 720p/1080p/4K. Clicks promote and restore normal order; every promotion has a PRIO marker; controls remain above the build catalog. Debug inspector tests cover move-picker activation, a ground click without losing robot selection, Escape cancellation, task text and explicit order cancellation.
- Existing 450 resolution/zoom/screen and 42 warning-panel cases pass. Inspected software rasterizations of the recorded construction-priority header and debug robot inspector at 200% zoom; these are layout checks, not native GPU captures.
- Installer checks clean v0.43.7 application, repeat application, paths with spaces, local .gitignore preservation and conflict atomicity. The single-directory ZIP contains update.patch, apply.sh and apply.bat; neither installer builds the game.
- The previously documented congested-traffic stress limitation remains outside this update. Priority dispatch does not interrupt an active builder job or discard a loaded carrier's physical cargo.

# v0.43.7 validation (2026-11-01)

- Loaded the supplied 0-43-6.gmsave and continued its autonomous construction/navigation for 20 simulation seconds with no safety fault. The colony has 3 habitats, 8 resources, 3 corridors, 5 supply channels and 2 robots. The carrier still has depot-housekeeping movement although no construction is pending.
- Same-container GCC 13/C++20/O1 headless profile, 120 iterations per stage with four CPU lanes: before optimization, construction/robot update averaged 12.257 ms and render preparation 3.665 ms; afterward approximately 2.452 and 1.119 ms. Supply (~0.1 ms) is included in construction time. This is CPU-side timing, excluding native GPU/driver/present cost, and is not a measured player FPS claim.
- Scene draw recorder reports identical total vertex count (472386) and reduces 3D submissions from 1827 to 162 by batching stored packages. Grid caching reduced resources-only render preparation from 2.947 to approximately 0.343 ms.
- GCC syntax-checks the changed OpenGL translation unit with minimal temporary X11 declarations because native headers/libraries/display are unavailable. Ordered stream tests check adjacent command merging, preserved vertex order, non-overlapping ranges and reset across passes. Native OpenGL driver behavior and Windows/Direct3D display execution remain unverified.
- Fourteen gameplay groups pass: fleet, geometry, pressure gates, extensions, workshops, interaction/jump, utilities, supply, oxygen, new mixed supply network, three-second recovery, construction, late corridor and fault-injected construction. The existing congested-traffic stress limitation remains outside this optimization.
- New mixed-network regression reaches every indoor habitat kind and the storage dome from vacuum, using alternating completed corridors/pipelines and exterior machinery service manifolds. All reach more than 95% air from a common 94% fill state. Blueprints remain isolated and join on completion. Existing electrolysis conservation, delivery delay, depletion, tank and battery limits/save-roundtrip cases pass. Solar test checks doubled small-panel output and the standard-panel battery charge limit.
- UI recorder passes 450 resolution/zoom/screen cases and 42 structure warning cases. Actual native visual rendering is not exercised by this recorder.
- Shell installer checks clean v0.43.6 application, repeat application, paths with spaces, local .gitignore preservation and conflict atomicity. Package contains update.patch, apply.sh and apply.bat; neither installer builds.

# v0.43.6 validation (2026-11-01)

- GCC 13/C++20 gameplay and UI builds using the temporary headless Application harness. Native Linux platform syntax checking is blocked by missing X11/Xatom.h; Windows SDK/MSVC and native driver/display execution are unavailable.
- Interaction regression checks 480 gate angle/tier combinations for identical rigid vertices/dimensions, fixed chamber depth, pickup clearance, full shutter reservation and a circular dome envelope containing all rendered dome/foundation/details vertices. Eight autonomous cargo arrivals cover four dome sizes at two free angles with a rotated owner; overlapping gate approaches are rejected.
- Jump checks cover the approximately 1 m lunar arc, landing, held-key behavior, rejected mid-air re-jump, low-ceiling contact, the rendered corridor roof/deck and collision with solid outdoor machinery while airborne.
- Gate/extension regressions pass: pressure seals, rotated gate logistics, reserved approaches, shared two-carrier deposits, storage withdrawals during construction, optional 0-3 tasks, blocked placement, save round trips and legacy migration. Supply regression checks that unused pads are hidden and selected cargo pads use unfilled non-orange outlines.
- UI recorder passes 450 screen/zoom/resolution cases plus 42 structure-kind/warning-state cases. Every habitat/resource type displays all supplied warning bits, no stale warning text remains when flags clear, wrapped text stays in its frame, and details stay above the build bar at maximum zoom. Existing HD/4K proportions, aspect ratios, hitboxes, paging and font/stroke checks pass.
- Inspected software rasterizations of storage and workshop warning panels, generator details and POV jump help. These verify recorded layout rather than native GPU output.
- Oxygen/supply, safety-fault recovery, loaded-carrier recovery, fleet safety, workshop parking, original and late-corridor construction, utility topology and interior geometry regressions pass. The earlier congested-traffic stress limitation remains outside this change.
- Installer application, repeat application, paths with spaces, local .gitignore preservation and conflict atomicity are checked against v0.43.5. Apply scripts never build the game.

# v0.43.5 validation (2026-11-01)

- GCC 13/C++20 gameplay and UI build using the temporary headless Application harness. Native OpenGL/Direct3D compilation, driver rendering and interactive display execution remain unavailable here.
- Dedicated oxygen regression checks actual 9 kg/day standard output, 1.125 L/kg water debit, water/oxygen conservation, delayed downstream filling, connected room/storage delivery, disconnected networks, signed mass/percentage rates, dry/unpowered/full/near-empty inputs and zero-time refresh without mass changes.
- Supply regression covers battery/tank dispatch, existing air/warning/gate behavior and save round trips; loaded interiors expose their saved breathable percentage immediately, including while paused.
- UI recorder checks sixteen contexts at five UI-size settings and five 16:9 resolutions: 400 cases. New checks cover generator intake/output/status, breathable percentage and positive/negative change rates. Existing aspect-ratio, text bounds, geometry, hitbox, paging and shared font/stroke checks pass.
- Inspected software rasterizations of generator, falling-air room and corridor details at 200% UI scale. They verify recorded UI layout, not native render-backend output.
- Fault recovery, loaded-carrier fault recovery, fleet safety, gates, extensions, workshops, original/late construction, utilities and geometry regressions pass. The previously documented congested-traffic stress limitation remains outside this oxygen telemetry update.
- Installer application, repeat application, paths containing spaces, local .gitignore preservation and conflict atomicity are checked against v0.43.4. Apply scripts never build the game.
- The existing estimated cabin volume, assumed inert buffer gas and conservative oxygen-transfer approximation remain unchanged; these readings are not a full gas-composition/CO2/humidity simulation.

# v0.43.4 validation (2026-11-01)

- GCC 13/C++20 gameplay/UI build with the temporary headless Application harness. Native OpenGL/Direct3D compilation and GPU/display execution remain unavailable here; platform/render integration requires an on-host check.
- UI recorder checks thirteen menu/HUD contexts, five UI-size settings (100-200%), and five 16:9 viewports (1280x720, 1600x900, 1920x1080, 2560x1440, 3840x2160): 325 cases. Logical rectangles, text extents, icon geometry and strokes match across resolutions, normalized screen proportions agree, and panels/text remain within the viewport.
- Additional checks cover 5:4 and ultrawide displays, button/selection hitboxes, next-frame zoom changes, fixed-size catalog paging, wheel consumption, fitted button captions, font metrics/punctuation and logical stroke thickness.
- Software rasterizations of recorded UI draw calls were inspected for main/new-game/settings screens, construction/resource/robot panels, and an HD/4K comparison. These verify layout and shared glyph geometry, not native-driver output or 3D scene rendering.
- Existing fault recovery, loaded-carrier recovery, fleet safety, cargo gates, storage extensions, workshops, construction, late corridors, supply, topology and geometry regressions are run with this update.
- Installer application, idempotence, target paths with spaces and conflict atomicity are checked against v0.43.3. Apply scripts do not build the game.
- The existing congested-traffic stress limitation documented for v0.43.3 remains outside this UI change.

# v0.43.3 validation (2026-11-01)

- GCC 13/C++20 gameplay build using the existing temporary headless Application harness; production platform/render backends were not compiled or run in this environment.
- New fault tests cover one robot, twelve simultaneous faults, repeated unsafe-origin faults, an inactive working hold, bounded incremental search continuation, and invalid motion bounds corrected later. They check the three-second deadline, discarded path/search state, latest destination, stationary fault holds, absence of pose jumps and swept fleet collisions, and certified progress after recovery.
- Loaded-carrier late-corridor scenario: fault injected during delivery, cargo preserved throughout the wait, recovery attempted within three seconds, and all seven construction targets completed.
- Standard fleet, gate logistics, storage extensions, workshops, original construction, late-corridor construction, supply, topology and geometry regressions also run for this update.
- Additional stress limitation: injecting an early fault into the more congested full construction layout recovered successfully, but subsequent non-fault starter-lane traffic stalled (1/7 targets after 1,800 simulation seconds). This patch addresses SafetyMode::Fault recovery; generic congested-traffic livelocks remain unresolved.
- Timing is simulation time: pause freezes the recovery clock; faster game speeds shorten real waiting time. Physically unsafe origins remain stopped while a fresh search is retried, with no teleport or disabled collision checks.
- Patch application, repeat application, target paths containing spaces and conflict atomicity are checked against the supplied v0.43.2 source.

# v0.43.2 validation (2026-11-01)

- Compiled every gameplay translation unit, RVO2, UI and task pool using GCC 13, C++20 and a temporary headless Application stub. Production platform and renderer sources were unchanged.
- Passed colony supply regression: connected battery/tank dispatch, night discharge, isolated tank, capacity limits, electrolysis mass balance, downstream air delay, warnings, gate pad clearance/highlighting, continuous cargo angle, and schema-17 save/load round trips.
- Passed utility topology, pressure/cargo gates including two-carrier shared deposits, optional storage extensions and legacy migration, 120 habitat rotation/size geometry cases, 16 storage approaches, workshop parking/task dispatch, and seven fleet safety scenarios.
- Passed original construction template and late-corridor template: all seven construction tasks completed in each case with no safety fault.
- Update packaging is checked on a clean 0.43.1 tree, an already updated tree, a target path with spaces, a locally customized .gitignore, and a conflicting VERSION file. Apply scripts never invoke a build.
- Full CMake/graphical Linux build and Windows compilation were unavailable in this container (CMake, X11 development headers, MSVC/Windows SDK and display runtime were absent). Screen appearance and interactive mouse operation require an in-game check on the host.
- Cabin volume is an explicit 65% mesh-envelope estimate; inert buffer gas is assumed available. Air transfer is a conservative oxygen/pressure network approximation, not CFD or separate CO2/humidity simulation. Equipment and production rates are game design assumptions.

# Validation status

Validated in the available Linux build environment:

- CMake detects `linux` / `opengl` backend correctly.
- Full source tree compiles successfully with navigation tests enabled.
- `navigation_geometry` passes.
- `navigation_pressure_gates` passes.
- `CMakePresets.json` is accepted by CMake and exposes Linux presets on a Linux host.
- All shell helper scripts pass `bash -n` syntax checks.
- Every primary development operation has a paired `.sh` and `.bat` entry point.

Windows compilation cannot be executed in this Linux environment because MSVC, the Windows SDK and a Direct3D runtime are not present. The Windows backend remains the same Win32/Direct3D 12 implementation from the preceding port, while the build paths and command wrappers were unified around `build/windows`.

## v0.43.16 narrow-passage regression

- Reproduction fixture: user save `0-43-14.gmsave`, builder initially at approximately `(8.23, 0.54)` and workshop return goal approximately `(-23.0, -2.5)`.
- v0.43.15 result: repeated `NO SAFE ROUTE`; the 1.00 m comfort start was blocked, 0.00 m physical-only start/goal were legal, wide/coarse and fine/narrow route tiers both failed.
- Diagnostic proof: a wide sub-metre physical-only search found a full-hull-safe route, confirming that the robot was not physically trapped.
- v0.43.16 regression command: `navigation_regression clearance-save <path>`; requires a route publication, >8 m escape travel within 30 simulated seconds, no SafetyMode::Fault, and bounded replan count.
- Optimized regression result: all 23 registered CTest groups passed across split runs. The relevant navigation subset (`fault_recovery`, `fleet`, `workshops`, `global_routes`, `queue_flow`, `construction`, `late_corridor`) passed together in 18.35 s; remaining colony/UI/navigation groups also passed in their split runs.
- Baseline construction regression retained established behavior after narrowing the precision search to last-resort only: 7/7 targets completed, with no safety fault.


## v0.43.17 generated-entrance and robot-energy regression

- Release build: `cmake --build build/linux --target navigation_regression game -j2`; full `ctest --output-on-failure` passes 24/24 registered groups.
- `navigation_geometry` covers every habitat kind/size/rotation with empty `mesh.entrances`/`mesh.entrance_access`, requires generated on-demand doors, and physically traverses the generated docking ramp/door in both directions.
- Air Lock regressions cover rotated Small/Standard variants with no authored hatches, generated longitudinal connection selection, opposite-side rejection, four-seal three-chamber cycling and synthesized exterior access.
- `colony_robot_battery_bootstrap` runs the intended uninterrupted Stage-1 -> Stage-2 construction sequence with one BR-02 and one CR-01 and explicitly rejects any workshop charging. Measured use is 40.4401/48.0 kWh for BR-02 (15.7498% left) and 65.4184/76.0 kWh for CR-01 (13.9231% left), satisfying the >=10% reserve.
- `navigation_workshops` additionally verifies that a low-charge rover parked in a powered workshop actually gains stored battery energy and reports charging.


## v0.43.18 perimeter lane / debug layout validation

- `navigation_workshops` now verifies that a structure which geometrically fits inside the map is rejected when its footprint consumes the 2.25 m rover perimeter lane, while the same structure remains valid immediately inside the reserved boundary.
- Workshop approach and Storage Dome cargo apron placement share the same boundary rule; generated corridor access pads are also rejected when they enter the lane.
- `ui_resolution_scaling` exercises the debug-frame MESH -> TEXTURED selector and checks the enlarged debug panel remains inside the UI bounds.
- Release validation: 24/24 registered CTest groups pass after the v0.43.18 changes.

## v0.43.19 corridor angle / short-run validation

- `navigation_geometry` sweeps a generated Crew Quarters corridor anchor through all 360 integer degrees and requires the pressure-hull contact point to remain on the selected polar ray with no large adjacent-position jump.
- The same regression creates a close Crew Quarters/Canteen pair and requires an unobstructed short corridor to contain exactly two path points (one straight segment), while preserving generated entrance traversal and floor continuity.
- Existing Storage Dome, Air Lock, pressure-door, construction, robot-navigation, UI, supply, hydrogen/solar and Stage-2 battery regressions remain enabled.
- Release validation: 24/24 optimized registered CTest groups pass after the v0.43.19 changes.

## v0.43.20 corridor start-preview validation
- Build/test validation confirms the shortened pre-click blueprint does not change committed corridor routing or geometry.
- Existing corridor geometry, 360-degree attachment, straight short-run, construction and navigation regressions remain applicable.

## v0.43.21 corridor start / placement-envelope validation
- Loaded the supplied `0-43-20.gmsave` and reproduced its two close habitats: Small Air Lock at `(-20,-4)` and Small Oxygen Generator at `(-11,-4)`, with only about 1.21 m between the old footprint edges.
- `navigation_geometry` now rejects that spacing for new placement, finds the nearest legal integer-grid position using the corridor-derived ground reservation, and requires the resulting connection to be one straight two-point corridor with a non-degenerate gap.
- Existing save loading remains unchanged; the supplied save continued for 20 simulated seconds without a safety fault.
- Optimized validation: tests 1-22 passed in the broad run; the two long tests (`navigation_construction`, `navigation_late_corridor`) passed separately, for 24/24 registered groups overall.

## v0.43.22 Bio Dome end-cap validation
- Reproduced the supplied `0-43-21.gmsave`; its Bio Dome uses the greenhouse barrel geometry whose two long-axis ends previously lacked upper pressure surfaces.
- `navigation_geometry` now tests both long-axis Bio Dome ends for all four size tiers at 0, 37 and 90 degree rotation, requiring a valid on-demand hatch, conformal doorway wall profile and corridor preview.
- A clean optimized rebuild passes 24/24 registered CTest groups.
- The Robot Workshop perimeter fixture was updated to compute its boundary probe from the corridor-reserved `structure_ground_extent()` introduced in v0.43.21; this is a test-fixture correction only.

## v0.43.23 playfield-size validation

- UI regression asserts the new 128 x 128 default, 32..1024 selectable bounds, 32 m selector step and 128 x 128 debug-template size, then directly rebuilds the maximum 1024 x 1024 playfield.
- Legacy internal/saved playfield sizes remain supported; save validation accepts the new 1024 m upper bound without changing the save schema.
- Full optimized registered regression suite remains the release gate.

## v0.43.24 selection-panel overflow validation

- UI regression creates an overflowing upper-left Storage Dome info panel at 1280x720 / 200% UI scale and requires its content height to exceed the capped viewport.
- Mouse-wheel input inside the panel must advance the panel scroll offset while leaving camera zoom unchanged; repeated scrolling must reach the exact bottom limit.
- The previously hidden bottom action row must become visible after scrolling, a scrollbar must be rendered, and every scrolled text row must stay within the panel viewport.
- Full optimized registered regression suite: 24/24 CTest groups pass.

## v0.44.0 Bio Dome hydroponics validation
- `navigation_regression hydroponics` verifies crop-specific growth, water and power dependency, automatic Food harvest, physical Food output, size-tier bay counts and schema-23 save/load persistence.
- `navigation_regression supply` verifies the existing power/water network behavior remains valid with hydroponics loads.
- `navigation_regression progression` verifies colony progression and crew-readiness behavior remains intact.
- `ui_regression` verifies the Bio Dome bay table, crop selection/reset behavior, per-bay growth bars and scrolling at enlarged UI scale, together with the existing resolution/zoom UI suite.
- A schema-22 v0.43.36 save loads successfully under the schema-23 reader, exercising legacy hydroponics migration without corrupting the existing world layout.


## v0.44.1 POV interior collision validation
- Optimized Linux Release build completes with navigation tests enabled.
- `navigation_regression geometry` passes: generated habitat entrances, corridor ramps, Storage Dome entrance aisles and all existing geometry cases remain traversable where intended; every habitat kind also has verified solid interior equipment and remaining walkable floor space.
- `navigation_pressure_gates`, `colony_airlock_cycle`, `colony_airlock_connections`, `navigation_storage_extensions` and `ui_resolution_scaling` pass.
- `navigation_workshops` remains a very long-running pre-existing regression and exceeded the 300 s harness window during this validation; no robot/navigation workshop code is changed by v0.44.1.

## v0.44.2 Bio Dome commissioning / photoperiod validation
- Optimized Linux Release build completes with navigation/UI tests enabled.
- `colony_hydroponics` passes: new bays are empty, the 06:00-22:00 light phase grows crops, 22:00-06:00 dark phase stops growth and crop light/water load, missing breathable air stops growth, missing electrical power stops growth, harvest/output and schema-23 persistence remain valid.
- `ui_resolution_scaling` passes: the completed Bio Dome renders a two-column bay table with dropdown selectors and growth bars, EMPTY / REMOVE CROP clears a crop and resets growth, unfinished Bio Domes do not expose crop controls, and photoperiod state is shown.
- `colony_oxygen`, `colony_progression`, `colony_supply_network` and `colony_supply` pass with the revised Bio Dome load behavior.

## v0.44.3 corridor placement tooltip validation
- Optimized Linux Release build completes with navigation/UI tests enabled.
- `ui_resolution_scaling` verifies that a corridor rejection tooltip renders its title and explanatory body inside the HUD bounds even near the lower-right corner at 1280x720.
- Existing corridor geometry, Air Lock connection and Storage Dome docking regressions remain enabled to ensure the diagnostic UI does not alter corridor routing/placement behavior.
