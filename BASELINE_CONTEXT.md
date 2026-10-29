# game v0.40.0 baseline context

Use this file as the compact starting context in a new chat.

- Standalone C++20 Linux/X11 game with separate `engine/` and `game/` trees.
- Version source: root `VERSION`; this baseline is `0.40.0`.
- Build: `./build.sh`; run: `./run.sh`.
- Build implementation is split into CMake context targets; see `IMPLEMENTATION_CLUSTERS.md` and `game/cmake/GameClusters.cmake`.
- Robot route geometry search is isolated in `game_cluster_pathfinding` (`game/robots/path_planner.*`).
- Robot task/logistics/traffic execution remains in `game_cluster_robots` (`builder_robot.*`).
- BR-02 builders and CR-01 carriers use preplanned routes, hard hull collision exclusion and continuous speed/traffic prediction.
- If a route cannot be resolved or makes no useful progress for 2.0 simulated seconds, the robot discards the local route and requests a full-width world-geometry replan. Normal local replans do not reset this watchdog.
- Construction logistics use one drop area per site. Carriers place delivered resources there; builders physically collect the current stage bundle before assembly. Builders can self-haul uncovered demand.
- Builders work construction stages around all sides. Future construction geometry is reserved before the next segment is revealed.
- Construction rendering is mesh-only: blue blueprint, yellow installed/in-progress geometry, bold green completed geometry. No structure textures/surfaces yet.
- Update packaging convention: `game-update-vX.Y.Z.zip` -> one top-level `game-update-vX.Y.Z/` containing only `apply.sh` + `update.patch`; `apply.sh` does not build.

For a new request, identify the smallest cluster(s) from `IMPLEMENTATION_CLUSTERS.md` first instead of loading the whole workspace.
