# JSD Lunar Base

JSD Lunar Base is a 3D lunar colony and mission simulation built on the JSD C++ game engine. Start with a space company, fund and design a lunar delivery mission, then build a surface base with a fleet of robots.

## What you can do

| Stage | Gameplay |
| --- | --- |
| 1. Space company | Earn funding from contracts, unlock technology, choose a rocket and lander, and qualify the mission through the company facilities. Select the structures you plan to build and use their construction recipes to budget the payload. Add supplies and connection materials separately. |
| 2. Robotic base | Choose the landing site, control the view during descent, unload robots and cargo, and build the base. Rovers haul physical resources, build in stages, share routes and recover from blockages. |
| 3. Debug modes | Enable Debug Frame to enter the Free Play mode selector. Template Mode restores the prebuilt lunar base. Sandbox Mode opens an empty 128 × 128 field with instant construction. Test Astronauts opens an empty test field. |

Build exterior power, extraction and storage facilities and link them with supply channels. Track solar output, batteries, water, oxygen and hydrogen. Use oxygen tanks as a reserve for connected rooms or sell stored oxygen to a trader called from a powered landing pad after building an antenna. Attach new structures to free corridor and supply-channel ends wherever their footprints fit. View the base from orbit or explore structures and airlocks in first person. Survey local ore deposits with the Ore Driller blueprint. Conveyor belts move ore crates to powered, pressurized processing facilities or factories. Without a built belt, carriers pick up the outdoor stacks and bring them to a working processor or a Storage Dome with a ready loading gate.

Career follows the mission stages. Free Play lets you start at a selected stage with its own setup. Save and resume a game locally.

## Build and run

The workspace separates the engine in `engine/` from the simulation and game in `game/`. On Linux use `./install-deps.sh`, `./build.sh`, and `./run.sh`. Run `./test.sh` for regression tests. See [DEPENDENCIES.md](DEPENDENCIES.md) for requirements, [ARCHITECTURE.md](ARCHITECTURE.md) for the code layout, and [README-WINDOWS.md](README-WINDOWS.md) for Windows tooling.

See [UPDATE_HISTORY.log](UPDATE_HISTORY.log) for release notes and [README-HISTORY.md](README-HISTORY.md) for the archived README and earlier feature history.
