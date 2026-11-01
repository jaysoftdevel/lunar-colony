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
