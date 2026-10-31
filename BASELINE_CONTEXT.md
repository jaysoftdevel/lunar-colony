# game v0.40.48 baseline context

Use this file as the compact starting context in a new chat.

- Standalone C++20 Linux/X11 game with separate `engine/` and `game/` trees.
- Version source: root `VERSION`; current release is `0.40.48`.
- v0.40.48 pathfinding: coarse A* has bounded longitudinal escape space so routes may backtrack around structures; world revisions retain still-clear cached routes; full fleet replans are capped per rendered safety tick and failed searches use staggered backoff. v0.40.47 logistics/build/collision behavior remains the baseline.

- v0.40.33 adds a New Game debug-start toggle that imports the bundled 64x64 construction blueprint layout from `0-40-32-debug-start.gmsave`, keeps the newly configured robot fleet/starter stash, resets all imported construction progress/material buffers to blueprint state, and starts paused.
- Build: `./build.sh`; run: `./run.sh`.
- Build implementation is split into CMake context targets; see `IMPLEMENTATION_CLUSTERS.md` and `game/cmake/GameClusters.cmake`.
- Robot task/logistics policy remains in `game_cluster_robots` (`builder_robot.*`), but it no longer changes robot poses directly.
- v0.40.32 adds explicit local fleet awareness to every rover. Active navigation proposals predict nearby robots over the 0.5 s local horizon and coarse route replans treat current rover positions as temporary obstacles. Idle/working robots are never moved merely to clear traffic; only robots with active navigation requests may be assigned collision-recovery retreats.
- v0.40.31 fixes the first gameplay integration of the contract coordinator: all safety-envelope checks use the same inflated radius, short tail ticks get contracts with the actual lease duration, certified brake branches can be followed across piece boundaries, and local proposals include safe pivot-in-place plus braking-distance speed caps.
- Starter rover bays are widened for the conservative safety discs. Saves whose entire fleet is still in the untouched v0.40.30 starter apron are migrated to the wider parking layout on load. Retreats prefer lateral pull-outs and receive arbitration priority only between actively navigating rovers; idle and working rovers remain stationary traffic obstacles.
- `game/robots/safety_coordinator.*` is the single fleet-movement authority: 10 ms safety ticks, 0.50 s local proposal horizon, 0.05 s replan interval, time-indexed motion/braking contracts, terminal holds, passage reservations and shared atomic arbitration.
- `game_cluster_pathfinding` is static-geometry advisory routing only. Its coarse A*/Theta*-style route can suggest waypoints but never authorizes movement or handles robot-vs-robot collision avoidance.
- Rejected motion follows the robot's previously certified braking branch. Idle/stopped robots retain terminal reservations. Existing unsafe overlap is a safety fault, not repaired by teleport/nudge.
- Persistent blocking first waits, then uses wait-for/deadlock analysis and certified retreat/pull-out behavior. An idle blocker can be asked to yield. No safe escape means `BLOCKED`.
- Construction-stage geometry is committed only after current robot positions and active normal/braking/terminal contracts are clear. Autonomous resource-pallet creation is likewise deferred away from active reservations.
- Construction logistics use one drop area per site. Carriers place delivered resources there; builders physically collect the current stage bundle before assembly.
- Robot default movement speed multiplier is 200%.
- Construction rendering is mesh-only: blue blueprint, yellow installed/in-progress geometry, bold green completed geometry. Unselected construction blueprints show only their progress bar; selected blueprints expose details.
- Structures/blueprints carry persistent type/size-aware instance names.
- Update packaging convention: `game-update-vX.Y.Z.zip` -> one top-level `game-update-vX.Y.Z/` containing only `apply.sh` + `update.patch`; `apply.sh` does not build.

For robot-motion changes, load `game_cluster_robots` + `game_cluster_pathfinding`; treat the safety coordinator contract as authoritative and do not reintroduce direct robot movement or ad-hoc collision recovery.

v0.40.43: hardware OpenGL is mandatory at runtime (`glXIsDirect` plus renderer-string software fallback rejection). The debug frame shows the GPU renderer. Robot path/proposal preparation uses `eng::TaskPool` with about 50% of visible logical CPUs; authoritative safety arbitration and world mutation remain serial. Engine files: `engine/src/renderer_gl.cpp`, `engine/src/task_pool.cpp`, `engine/include/engine/engine.hpp`.
