# JSD Lunar Base v0.52.6

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

Build exterior power, extraction and storage facilities and link them with supply channels. Track solar output, batteries, water, oxygen and hydrogen. Use oxygen tanks as a reserve for connected rooms or sell stored oxygen to a trader called from a powered landing pad after building an antenna. Attach new structures to free corridor and supply-channel ends wherever their footprints fit. View the base from orbit or explore structures and airlocks in first person. Survey local ore deposits with the Ore Driller blueprint. Conveyor belts move ore crates to powered, pressurized processing facilities or factories. Without a built belt, carriers pick up the outdoor stacks and bring them to a working processor or a Storage Dome with a ready loading gate.

Career follows the mission stages. Free Play lets you start at a selected stage with its own setup. Save and resume a game locally.

Press **1–3** to select a simulation speed and **Space** to pause or resume. A pulsing frame marks paused simulation. In first person, **J** jumps. In the Debug build, **Settings → Debug Layout** switches between the configured renderer and the meshed view. **Settings → Music** controls the atmospheric theme independently of sound effects.

## Build and run

The workspace separates the engine in `engine/` from the simulation and game in `game/`. The Linux and Windows build scripts each produce Release and Debug executables and stage their app-local runtime files under the platform's `build` directory. The default launcher starts Release. Pass `Debug` to start the developer build. The [cross-platform guide](CROSS-PLATFORM-DEVELOPMENT.md) lists the paired commands. The [dependency list](DEPENDENCIES.md), [architecture guide](ARCHITECTURE.md), and [Windows build guide](README-WINDOWS.md) cover the toolchains and source layout.

See [UPDATE_HISTORY.log](UPDATE_HISTORY.log) for release notes and [README-HISTORY.md](README-HISTORY.md) for the archived README and earlier feature history.
