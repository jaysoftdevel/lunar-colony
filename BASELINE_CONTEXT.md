# game v0.41.0 baseline context

- Standalone C++20 Linux/X11 game, separate `engine/` and `game/` trees.
- Root `VERSION` is the release source. Build with `./build.sh`; `./run.sh` runs `build/bin/game`.
- All visible CPU lanes remain available through the persistent task pool. `GAME_CPU_LANES` is an optional diagnostic cap. Per-rover planning runs concurrently; arbitration, pose commit and resource/world mutation remain deterministic on the game thread.

## Robot movement

- `safety_coordinator.*` is the only pose authority. Job routines submit goals and work holds.
- Global routing uses bounded persistent D* Lite with clearance costs and epoch-tagged parked-robot footprints, plus fleet-aware A*/dogleg recovery routes. Moving actors remain local traffic. The returned clearance callbacks capture margin values by value.
- Local rollouts and execution follow a moving geometric corridor with projection, visibility-limited lookahead, arrival braking and acceleration/turn limits. A preview follows turns over the full 1 s horizon instead of holding one heading for that second.
- Local proposals refresh at 10 Hz; authoritative motion/braking/terminal contracts remain at 100 Hz. Path progress, rather than distance to a corner alone, scores candidates and detects circling.
- Vendored RVO2 (Apache-2.0) provides reciprocal velocity advice. See `third_party/rvo2/UPSTREAM.md` and `LICENSE`. It never commits poses or bypasses contracts. No online dependency fetch occurs at build time.
- Physical hull discs plus numeric margin are mandatory. Additional personal space is a steering preference, so a physically passable narrow gap remains usable. Coarse cell tokens are scheduling hints, not another spatial exclusion grid.
- Completed static geometry uses exact swept tests against inflated rotated footprints, corridor capsules and resource discs. Construction geometry still uses scene-admission and next-stage evacuation gates. Delivered resource boxes are admitted atomically against current hulls and all active motion/braking/terminal reservations before inventory changes.
- No speculative reverse fan or positional nudges. A blocked rover waits, pivots safely, obtains a stable detour, or retreats to a passing pocket. A parked idle robot may receive a certified yield task when it blocks a real destination or seals the only route; it holds the new bay until another job needs it. Active work holds remain protected. Untasked robots do not wander.
- The existing FIFO shipping/material accounting and staged construction policy remain in `builder_robot.cpp`. Service and queue positions use the same disc clearance as locomotion.
- Exterior rover queries use the planar playfield height. They do not run indoor crew/pressure-volume queries on every planner edge or safety sample.

## Structures and crew access

- Each habitat module declares its real `walkable_floor_y`; storage meshes declare theirs as well. Entrance anchors, frames, stairs, corridor collars and crew floor queries use that exact threshold.
- Human-scale frames and exterior access steps are rebuilt after horizontal size scaling. Steps end at the threshold and are oriented in the entrance's own basis. Air-lock access renders at actual construction completion.
- New structure corridors snap to authored entrances. Additional branches reuse the existing sealed trunk. Legacy generated hatches remain installed physical entrances; loaded collars and floors are synchronized with current models.
- Dock sleeves contain a continuous ramp and an interior threshold deck. Storage ports sit on the outer pressure wall. Interior circulation aisles retain crew head/shoulder clearance across size tiers.
- `mesh_portals.hpp` exactly clips triangles and lines around a doorway. The renderer caches local clipped layers, preserving wall material around the aperture and cutting an approach through tall foundations. Connected door openings are visible in both normal and POV views.
- Only the connected Air Lock permits exterior access; open corridor ends remain sealed. Outdoor machinery remains solid, while habitats, storage and the pressure corridor network are crew traversable.

## Verification and packaging

- Optional headless regressions: configure with `-DGAME_BUILD_NAVIGATION_TESTS=ON`, then use CTest. They cover swept fleet separation, smooth open travel, obstacle corners, changed scenes, occupied service points, opposing traffic, single-lane yielding, rotations/size tiers, storage ports, interior headroom and construction logistics.
- Update archive: `game-update-vX.Y.Z.zip` containing one `game-update-vX.Y.Z/` directory with only `apply.sh` and `update.patch`. Apply does not build. Clean builds preserve existing executable-adjacent savegames, including on build failure.
- v0.40.47 remains the historical logistics/build baseline. For current locomotion and access behavior, this document supersedes older version notes in `ARCHITECTURE.md`.
