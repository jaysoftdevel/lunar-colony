# Lunar Colony 0.72.1 / JSD engine 0.5.21

Paired source correction for game 0.72.0 and external engine 0.5.20.

The engine metadata, CMake and public/runtime version were advanced to 0.5.20,
but build.sh, install_deps.sh and the native Windows build driver still expected
0.5.14. This stopped the standalone build before configuration. All three entry
points now require 0.5.21, matching engine metadata, CMake, SDK headers and runtime.
The game CMake guard and Windows driver also require 0.5.21. Version, project,
cache-ownership and install-prefix guards remain enabled.

The game sync check now verifies the engine's Linux build/dependency entry
points and both Windows-driver preflights. New engine regression tests execute
the shipped launchers against actual release metadata, test configure/build/
install dispatch with fake tools, and reject wrong versions/projects before any
external build or dependency command. Dependency-installer tests use fake tools;
they do not install system packages.

The graphics settings label now reports the current engine version rather than
an old 0.5.14 literal. Gameplay, renderer behavior, public API version (1) and
save schema (51) retain their 0.72.0 behavior. Existing saves remain supported.

Apply the package to both source roots, then rebuild. A standalone engine build
uses ./build.sh from the engine directory. The game builds its external engine
from source with ./build.sh /absolute/path/to/3d-engine from the game directory.
See the package README and VALIDATION.md for commands and verification results.
