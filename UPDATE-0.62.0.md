# Lunar Colony 0.62.0

Requires the accompanying external JSD engine source 0.5.13. Apply the game and
engine update together, then rebuild with `./build.sh /path/to/3d-engine` or
`build.bat --engine PATH`. Linux and Windows continue to share the same sources.

## Driller collection

Completed drillers retain raw ore crates until an eligible carrier can collect
them. A completed Storage Dome with free capacity and a completed loading gate
now takes priority over processing. Dispatch checks a route to the pickup and
the gate's exterior service point. Blocked gates are skipped in favor of another
reachable gate/dome. Empty carriers receive this work before optional workshop
returns or a new construction trip; current cargo, construction and trader tasks
finish normally. Charging reserves and navigation safety still apply.

A parked carrier requests its workshop door before route certification, and
active ore trips keep the door available for departure. A newly ready gate
invalidates the short retry delay. Failed probes retry at a bounded rate instead
of running a large search every frame. If a gate disappears or storage fills
while a crate is carried, the robot redirects or returns that physical crate to
the driller. It does not teleport cargo or credit raw ore as refined material.
Conveyors still do not take new driller output.

## Gameplay options and first person

Settings → Gameplay → FPS COUNTER is available in Release and Debug. It is off
by default, applies immediately and persists. The lower-left overlay measures
wall-clock frames, independently of the simulation's timestep clamp and speed.
The crosshair uses a yellow foreground and dark outline for contrast against
bright panels and the dark lunar surface.

First person: WASD walks, Shift sprints (4.2 m/s versus 2.0 m/s walking), and
Space jumps. J remains a jump alias. Movement retains acceleration, swept hull
checks, ceiling collision and lunar gravity. Held Space does not auto-repeat a
jump. Space continues to pause/unpause in orbit view; it does not pause first
person. Number keys still select simulation speed.

## Sky and workshop

The native engine draws a camera-centered infinite sky sphere from direction
rays. Stars are seeded in world directions, so camera rotation/FOV/aspect changes
reproject the catalog, while translation introduces no parallax. Fine stars are
filtered to reduce shimmer. The Moon uses vacuum black, a solar disc and stars;
the Earth campus retains atmospheric daylight/clouds, which suppress stars.
The sky writes background motion/depth and is occluded by normal scene geometry.

The robot workshop/hangar now has opaque curved gable walls at both ends. The
front retains the moving gate opening. Shutters use weathered metal panels,
raised stiffeners, edge rails and lower hazard markings instead of green solid
panels. Texture coordinates move with each shutter. Hull hiding still only
affects rendering; door access and collision are unchanged.

Saves retain their format. New FPS preference and transient route-probe fields
do not require a saved-game migration. See VALIDATION.md for performed checks.
