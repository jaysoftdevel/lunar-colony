# Implementation ownership — 0.61.1

Explicit CMake object clusters live in game/cmake/GameClusters.cmake. Consult the
smallest relevant group before changing shared game logic.

| Area | Source |
| --- | --- |
| External engine validation | cmake/ExternalEngine.cmake |
| Game-to-JSD bindings | game/cmake/Runtime.cmake and game/src/runtime/ |
| Menus, preferences, benchmark | game/src/screens/ and game/src/config/ |
| Orbit/first-person camera and input | game/src/systems/camera_controller.cpp, first_person_controller.cpp, first_person_mode.cpp |
| World/robot/lander/trader submission | game/src/playfield/ |
| HUD/widgets/layout | game/src/ui/ and game/include/game/ui/ |
| Navigation/safety/route planning | game/src/robots/ |
| Construction/resources/colony systems | game/src/systems/ and game/src/features/ |
| World models/connections | game/src/world/ |
| Persistence | game/src/systems/save_game.cpp |

There is no internal engine target or fallback renderer. Engine implementations
belong to the selected external source tree. Use ENGINE-INTEGRATION.md for its
public services and the game's binding responsibilities.
