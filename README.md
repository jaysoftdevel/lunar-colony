# JSD Lunar Base v0.54.2

JSD Lunar Base is a 3D lunar colony and mission simulation built on the JSD C++ game engine. Start with a space company, fund and design a lunar delivery mission, then build a surface base with a fleet of robots.

The game includes a Direct3D 12 lunar renderer on Windows and an OpenGL
compute renderer with a deferred fallback on Linux. Use **Settings → Graphics** to adjust
the active renderer and its effects. **Graphics → Benchmark** measures the
current renderer and recommends a profile for your frame-rate target. See [RENDERING.md](RENDERING.md) for
the feature breakdown and hardware requirements.

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

Press **1–3** to select a simulation speed and **Space** to pause or resume. A pulsing frame marks paused simulation. In first person, **J** jumps. In the Debug build, **Settings → Debug Layout** switches between the configured renderer and the meshed view. **Settings → Music** controls the atmospheric theme independently of sound effects.

## Build and run

The workspace separates the engine in `engine/` from the simulation and game in `game/`. The Linux and Windows build scripts each produce Release and Debug executables and stage their app-local runtime files under the platform's `build` directory. The default launcher starts Release. Pass `Debug` to start the developer build. The [cross-platform guide](CROSS-PLATFORM-DEVELOPMENT.md) lists the paired commands. The [dependency list](DEPENDENCIES.md), [architecture guide](ARCHITECTURE.md), and [Windows build guide](README-WINDOWS.md) cover the toolchains and source layout.

## CPU scheduling

At startup the C++ task pool sizes itself to the available logical CPUs. The application thread can join a batch alongside the persistent workers. Linux reads the launcher's permitted CPU mask and gives each worker its own CPU. Windows 11 places workers with CPU Sets. The application thread keeps its original scheduling policy.

Terrain generation, OpenGL mesh preparation, model generation, navigation, utility networks and music synthesis use the pool. The Stage 1 campus builds its geometry once and reuses it across frames. Simulation state and graphics commands are committed on the application thread.

Run `./run.sh --cpu-info` or `run.bat --cpu-info` to see the detected CPUs, configured lanes and successful worker placements. `GAME_CPU_LANES=<n>` caps the pool, including the application lane. Set `GAME_CPU_PIN=0` to let the OS place workers while keeping the detected lane count. A Linux launch under `taskset` uses the subset supplied to the game.

See [UPDATE_HISTORY.log](UPDATE_HISTORY.log) for release notes and [README-HISTORY.md](README-HISTORY.md) for the archived README and earlier feature history.
