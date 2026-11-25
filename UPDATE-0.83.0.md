# Lunar Colony 0.83.0 / JSD 0.5.22

The cumulative Linux/Windows update now includes an executable apply.sh.
Its source mode and ZIP Unix permissions are explicitly 0755. Source roots,
version pairs, affected local edits and paired rollback remain guarded.

Compiler cleanup fixes deprecated UTF-8 filesystem construction, ambiguous
indentation/precedence, unused declarations, optional stockyard initialization
and missing quaternion initializers. Warnings remain enabled. The engine
point release supplies a shared C++20 UTF-8 path helper, preserving Windows
Unicode paths. Game 0.83.0 requires external JSD engine 0.5.22.

Rovers use the same real models aboard the lander and outside. After the ramp opens, a short visible deployment phase lets wheel assemblies spread and steer out of their transport configuration,
sensor masts hinge up and manipulators unfold. This movement follows the saved opening
clock and finishes before departure. The panel identifies UNFOLDING ROVERS
during that phase. Cargo-bay work lights illuminate the mechanisms. The transition from the opening ramp
to the driving queue is continuous; gray placeholder blocks are removed.
On current landing layouts, parked robots enter normal work independently
while the rest unload. Outer parking bays fill first, giving early workers a clear exit. The
remaining rovers' ground routes stay reserved in both navigation and collision
until their parking handoff completes. Older parking-layout saves retain their compatible queue.

Crew dispatch now shares live site demand and material reservations. Carriers
scan all actionable jobs, skipping a reserved or unavailable queue head instead
of idling behind it. Priority determines claim order rather than blocking the
rest of the crew. Free constructors can help collect and stage construction
stock, using the same physical source/drop leases as carriers. A peer arriving
never cancels an active constructor pickup.

An empty robot with an exhausted route can release its task for a peer and try
another job; a short personal cooldown prevents repeatedly reclaiming the same
failed route. Blocked loaded carriers can redirect their existing cargo to a
matching useful job without changing stock. Pickup, put-down, charging and
stage-commit transactions remain protected. Existing safety/path-reset logic
still controls every physical move; task exchange does not teleport robots.

The crew stocks a project's remaining bill before its structure grows, so a
new wall cannot sever the next delivery route. Other robots can stage/build
other projects meanwhile. A resumed constructor's useful surplus cargo is
assigned to later stages; true excess returns to physical stock rather than
looping an empty-reservation put-down. Idle carriers may recover loose stock
while other construction continues.

Save schema54 remains unchanged. Unfolding uses the existing saved phase clock;
crew cooldowns are transient and re-evaluated after load. Progress, astronaut
stats, existing interiors and settings behavior remain intact.

See package VALIDATION.md for executed checks and native Windows limits.
