# Lunar Colony 0.65.0 / JSD 0.5.15

Apply the paired source update to game 0.64.0 and engine 0.5.14, then rebuild
Release and Debug. Save schema 48 is unchanged; existing saves load directly.

## Rendering and input

Engineering wireframes, blueprints and route guides use native GPU-instanced
screen-width segments. They retain scene-depth occlusion and are composited
without PBR shading, shadow passes or temporal pointer smearing. Exact-content
wireframe caches remove shared edges. Unchanged segment buffers stay resident,
including while the camera moves. Large segment preparation uses the engine
worker pool; all OpenGL calls remain on the context thread.

Triangle batches retain their previous CPU buffers and compare bytes rather
than hashing every vertex byte each frame. The playfield crosshair is now a
native desktop cursor, so pointer motion does not wait for world rendering.

Move the pointer into the outer band of the visible world to scroll. Movement
scales with zoom and frame time and stops at the visible ground/map boundary.
An axis locks when its whole playfield fits. Panels, menus, first-person mode,
window focus loss and pointer drags suppress edge scrolling. WASD and camera
orbit/drag remain available.

## Robot recovery

Idle dispatch preserves a constructor's live parking route instead of clearing
its incremental search every frame. Stalled navigation checks destination apron
ownership and requests a certified pull-out from an idle peer, even beyond local
collision-sensor range. Such a yield request survives until the peer clears its
pull-out destination. Working robots retain their protected work holds.

A coarse grid that cannot connect a real pose or service bay can escalate to a
finer grid aligned along the trip. Search state persists across frames under
bounded expansion/time slices. A pending route computation no longer counts as
an intentional traffic queue in the operation watchdog. Resetting advisory paths
preserves physical cargo and the current braking/hold contract; there is no
teleportation or collision bypass.

Optional render diagnostics: set GAME_RENDER_PROFILE=1 before starting the
binary. It prints submission/upload timing, native segment counts and uploaded
bytes. Zero line bytes indicates resident geometry was reused. JSD_WORKERS and
GAME_CPU_LANES remain optional worker controls; defaults use available CPUs.
