# JSD Lunar Base v0.88.1

JSD Lunar Base is a 3D lunar colony and mission simulation built on the JSD C++ game engine. Start with a space company, fund and design a lunar delivery mission, then build a surface base with a fleet of robots.

The game renders through the external **JSD 3d-engine 0.5.22**, including first-person interiors and exteriors. Build with `./build.sh /path/to/3d-engine`; the engine stays in its own folder. See [ENGINE-INTEGRATION.md](ENGINE-INTEGRATION.md) for Linux/Windows commands, requirements, rendering details and tests.

Version 0.88.1 refines corridor connector structure with rounded transition frames, basement-fitted supports and cleaner sleeve-to-foundation joins. See [UPDATE-0.88.1.md](UPDATE-0.88.1.md).

Version 0.88.0 adds a clock-driven Earth, continuous corridor connector floors and steps, storage-gate closure, a more visible crosshair and rectangular crew habitats. See [UPDATE-0.88.0.md](UPDATE-0.88.0.md).

Version 0.87.0 adds supported corridor connector walkways in no-hull mode, restores small processing-facility refinery placement and adds plastic processing to the interior-unit options. See [UPDATE-0.87.0.md](UPDATE-0.87.0.md).

Version 0.86.0 adds confirmed save deletion, keyboard recovery, battery-backed cabin lighting and builder-driven deconstruction. See [UPDATE-0.86.0.md](UPDATE-0.86.0.md).

Version 0.85.1 adds native-engine validation and exact-save carrier/corridor plus FPV light regressions. See [UPDATE-0.85.1.md](UPDATE-0.85.1.md).

Version 0.85.0 adds carrier charging-route protection, a suit light (L), clearer status information and docking-deck repair. See [UPDATE-0.85.0.md](UPDATE-0.85.0.md).

Version 0.84.0 retains the selected airlock's full hull, fits end-door collars
and stairs before any connection, grounds elevated corridor supports, keeps
door controls available on battery-fed grids, and adds configurable autosave
with a ten-minute default. See [UPDATE-0.84.0.md](UPDATE-0.84.0.md).

Version 0.80.0 restored the original oxygen-dome sizes and adds Very Small,
Small and Medium generator units inside the dome. See [UPDATE-0.80.0.md](UPDATE-0.80.0.md).

Version 0.79.0 replaced barrel habitats with dome pressure volumes, adds Very
Small/Small/Medium oxygen domes, fixes generator placement, aligns perimeter
machines to the local hull and provides a clear structural cutaway view.
See [UPDATE-0.79.0.md](UPDATE-0.79.0.md).

Version 0.78.0 consolidated interior floors and added explicit E-key hatches.

Version 0.77.0 added critical first-person vital alerts and distinct need icons.

Version 0.76.0 removed duplicate airlock transfer barriers, added the save-name
editor/sidebar, and explicit recovery for deceased FPV crew.

Version 0.75.0 added fullscreen startup and game-wide settings separate from
progress-only schema52 saves. These behaviors remain.

Version 0.74.0 added detailed corridor decks, airlock hardware and FPV clearance.

Version 0.73.0 added structural ribs, bulkheads and supported UV glazing.

Version 0.72.1 repairs engine build-launcher version checks and aligns the game
with JSD 0.5.22. Gameplay and saves retain the 0.72.0 behavior.

Version 0.72.0 adds direct selected-astronaut FPV control, saved character view
positions, live per-astronaut vitals and solid classified resource packages.
See [UPDATE-0.72.0.md](UPDATE-0.72.0.md).

Version 0.71.0 adds pressure-driven airflow, custom canteen furnishings, larger
placeable oxygen generators, night occupancy lighting and viewport construction
audio. See [UPDATE-0.71.0.md](UPDATE-0.71.0.md).

Version 0.70.0 doubles the day length, adds detailed corridor deck materials,
powered doors with an FPV manual crank (E), and empty configurable habitat
interiors with saved processing units. Selecting a habitat centers an isometric
cutaway view. See [UPDATE-0.70.0.md](UPDATE-0.70.0.md).

Version 0.69.1 retains the 0.69.0 gameplay features, and synchronizes the native Windows build.
See [UPDATE-0.69.1.md](UPDATE-0.69.1.md).

Version 0.69.0 marked disconnected gates red and usable corridor connections green,
reserves internal doorway approaches by removing obstructing station assemblies,
and keeps photovoltaic surfaces attached to rigidly moving collector heads.
See [UPDATE-0.69.0.md](UPDATE-0.69.0.md).

Version 0.68.0 adds solar-filter upper glazing, revised pressure-door hardware,
readable live structure displays with attached housings, and Alt-held FPV cursor
release. See [UPDATE-0.68.0.md](UPDATE-0.68.0.md).

Version 0.67.1 restores the star sky and global lunar surface in Classic
and minimum graphics settings. See [UPDATE-0.67.1.md](UPDATE-0.67.1.md).

Version 0.67.0 adds cyan 12% faded hulls, first-person building with a free
cursor (Tab), persistent bottom controls, brighter foundation guides and new
boot footsteps. Fine aisle routing frees the carrier in the 0.65 simple save.
See [UPDATE-0.67.0.md](UPDATE-0.67.0.md).

Version 0.66.0 opens fullscreen with a loading splash before graphics and game
assets initialize. Compiled engine shaders are cached for later starts.
See [UPDATE-0.66.0.md](UPDATE-0.66.0.md).

Version 0.65.0 adds retained GPU guide rendering, bounded edge scrolling and
robot destination coordination with persistent precision route recovery.
See [UPDATE-0.65.0.md](UPDATE-0.65.0.md).

Version 0.64.0 rebuilds corridor pressure boundaries, seals bends/junctions and
building collars, and preserves atmosphere in intact unoccupied volumes.
See [UPDATE-0.64.0.md](UPDATE-0.64.0.md) for details.

Version 0.63.0 added the glass hangar roof, four service bays, driller acoustics,
blueprint-safe ore stacks and the red power-off symbol; these features remain.

The FPS option, native playfield crosshair, sky sphere, sprint/jump and gate-based ore
hauling from 0.62.0 are retained.

The Windows build fixes from 0.61.1 are retained.
Windows uses the same external engine source and rendering features as Linux.
See [WINDOWS-COMPATIBILITY.md](WINDOWS-COMPATIBILITY.md) for validation.

Version 0.61.0 added seeded non-tiling terrain and surface materials, articulated robot armor,
and fitted station interiors with deck plating, pressure liners, service runs,
lockers, instrument screens, practical lighting and fabric bedding. Adjust
**Settings → FIRST PERSON FOV** from 50° to 110° (vertical, default 68°).

## What you can do

| Stage | Gameplay |
| --- | --- |
| 1. Space company | Earn funding from contracts, unlock technology, choose a rocket and lander, and qualify the mission through the company facilities. Select the structures you plan to build and use their construction recipes to budget the payload. Add supplies and connection materials separately. |
| 2. Robotic base | Choose the landing site, control the view during descent, unload robots and cargo, and build the base. Rovers haul physical resources, build in stages, share routes and recover from blockages. |
| 3. Debug scenarios | Template Mode restores the prebuilt base. Sandbox Mode gives you an empty field with instant construction. Test Astronauts lets you exercise the crew controls. These scenarios are available in Debug. |

Build exterior power, extraction and storage facilities and link them with supply channels. Track solar output, batteries, water, oxygen and hydrogen. Use oxygen tanks as a reserve for connected rooms or sell stored oxygen to a trader called from a powered landing pad after building an antenna. Attach new structures to free corridor and supply-channel ends wherever their footprints fit. View the base from orbit or explore structures and airlocks in first person.

Survey deposits with the Ore Driller blueprint. At full power, a standard driller extracts 60 kg per in-game day and a small driller extracts 30 kg. Carriers collect crates at the driller and bring them to a powered, pressurized processor or a Storage Dome with a ready loading gate. A storage-to-pad conveyor can load reserved trader cargo.

The **Hull** selector starts on **Full Hull**. Select **None** to hide exterior shells and glazing while keeping floors, equipment, and doors visible. Both views use the same collision, access, and simulation rules. Full Graphics uses basic terrain and structure textures on the Classic preset.

Career follows the mission stages. Free Play lets you start at a selected stage with its own setup.

Both builds store `.gmsave` files in `savegames/` at the game workspace root. The first launch moves saves from the former Linux and Windows `bin/savegames/` directories and gives conflicting filenames a numeric suffix. Older saves open normally. Saving again writes the compact format.

Press **V** to enter/exit control of the selected or last astronaut. The first
unassigned character spawns beside the lander. Use **Alt** or **Tab** for the
mouse cursor, and the lower-right panel for individual vitals and rest.

Press **1–3** to select a simulation speed and **Space** to pause or resume in orbit view. A pulsing frame marks paused simulation. In first person, **Shift** sprints and **Space** jumps (**J** is also accepted). In the Debug build, **Settings → Debug Layout** switches between the configured renderer and the meshed view. **Settings → Music** controls the atmospheric theme independently of sound effects.

## Build and run

The game uses the external JSD 3d-engine 0.5.20 for rendering, platform, math,
workers and audio. Its former in-game engine directory is removed. Game-specific
bindings live under `game/src/runtime/` alongside the game code.

```bash
./build.sh /absolute/path/to/3d-engine
./run.sh
./run.sh Debug
```

Both Release and Debug are built. On Windows use `build.bat --engine PATH`.
See [ENGINE-INTEGRATION.md](ENGINE-INTEGRATION.md) for exact requirements,
first-person rendering, runtime staging, audio/worker behavior and tests.

See [UPDATE_HISTORY.log](UPDATE_HISTORY.log) for release notes and [README-HISTORY.md](README-HISTORY.md) for the archived README and earlier feature history.
