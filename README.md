# JSD Lunar Base v0.69.0

JSD Lunar Base is a 3D lunar colony and mission simulation built on the JSD C++ game engine. Start with a space company, fund and design a lunar delivery mission, then build a surface base with a fleet of robots.

The game renders through the external **JSD 3d-engine 0.5.18**, including first-person interiors and exteriors. Build with `./build.sh /path/to/3d-engine`; the engine stays in its own folder. See [ENGINE-INTEGRATION.md](ENGINE-INTEGRATION.md) for Linux/Windows commands, requirements, rendering details and tests.

Version 0.69.0 marks disconnected gates red and usable corridor connections green,
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

Press **1–3** to select a simulation speed and **Space** to pause or resume in orbit view. A pulsing frame marks paused simulation. In first person, **Shift** sprints and **Space** jumps (**J** is also accepted). In the Debug build, **Settings → Debug Layout** switches between the configured renderer and the meshed view. **Settings → Music** controls the atmospheric theme independently of sound effects.

## Build and run

The game uses the external JSD 3d-engine 0.5.18 for rendering, platform, math,
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
