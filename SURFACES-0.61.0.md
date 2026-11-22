# Surfaces, interiors and FOV — 0.61.0

Requires external JSD engine source 0.5.11. Apply both parts of this update,
then build as before: `./build.sh /path/to/3d-engine` (Windows:
`build.bat --engine PATH`). The engine remains outside the game source tree.

## Material detail

The engine evaluates seeded, domain-warped surface fields directly in its PBR
shaders. Regolith combines broad mineral deposits, grains and fine relief;
painted hulls and robot armor add panel seams, fasteners, scuffs and chipped
coating. Interior liners, grip decks, rubber and fabric have their own surface
response. Neighboring structures and robots get distinct deterministic seeds.
There are no repeating terrain texture tiles or downloaded texture dependencies.
Architectural panels and manufactured weave deliberately retain their physical
regularity; their tone, wear and dirt vary across the surface.

Object-oriented, metre-scaled surface coordinates keep texture detail attached to
structures and articulated robot parts. Surface variation is stable between
frames. Powered instrument screens animate their readouts using the render
clock; unpowered screens become dark glass. This visual animation does not
modify production or resource simulation.

Material Detail controls fine relief cost. Basic settings retain broad textured
variation. Screen-derivative filtering suppresses fine patterns at distance.
Debug mesh mode still intentionally displays mesh lines.

## Station interiors

All ten habitat types and their size variants receive fitted inner pressure
liners, grip deck plates, ceiling service runs and hangers, access panels,
wall displays, LED strips and deck wayfinding. Greenhouses retain transparent
canopies and use low service skirts. Furniture includes framed consoles,
vented lockers, padded chairs, supported benches, tank collars and valves.
Crew bunks have fabric mattress/pillow surfaces and safety rails.

These finish layers do not add navigation obstacles. Existing pressure envelopes,
walkable floors, hatches, furniture clearances and construction rules remain
responsible for collision and crew access. Liners render inside completed
structures in first-person Full Hull view. The None hull mode still allows
an unobstructed overview of furnishings.

## First-person field of view

Open Settings and use the arrows beside FIRST PERSON FOV. The range is 50–110°
vertical, in 2° steps; the default is 68°. It applies immediately and is saved
in the normal preferences file as `first_person_fov`. Loaded values are clamped.
Orbit camera zoom is independent. Changing FOV changes projection and picking,
while retaining the player's physical position and direction.
