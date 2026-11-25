# Lunar Colony 0.79.0 — Dome habitats and usable equipment bays

Requires external JSD engine 0.5.21. The same C++20/SDL2 sources and build
drivers serve Linux and Windows 11. This is a cumulative source update.

## Oxygen dome and the supplied colony

The supplied 0.77 save contains an empty Small oxygen dome whose initial
generator fit had already failed. The old full-width aisle reservation,
equipment overlap bounds and low service ring left no legal generator pose.
The replacement uses a central circulation core, oriented equipment footprints
and overhead services above the 2.7-metre generator. The regression installs a
full-size generator in that exact colony, with the normal construction cost.
It does not reset the colony or change the original save.

New oxygen construction offers **Very Small, Small and Medium**. Mouse wheel
cycles these three sizes. All three accommodate the full-size oxygen plant
and retain access space. Medium uses the existing Standard tier identity;
old Mid/Big domes remain loadable. Previously removed generators are not
silently restored. Select the dome and use its internal-unit build options
to add a generator to an existing empty room.

## Habitat envelopes and interiors

All ten habitat types now use doubly curved ellipsoidal roofs on short
equipment-height pressure walls, with radial ribs, circumferential rings,
raised decks and supporting foundations. Footprint reservations, docking
elevations and building positions are retained. Existing corridor endpoints
are reprojected onto the new skin, and their pressure collars are rebuilt.
New hatch anchors sample above the deck so they attach to the pressure wall,
rather than the wider foundation. This prevents a vacuum gap that could stop
an autonomous astronaut at the airlock-to-corridor join.
Hatch orientation uses the continuous hull normal, and the pressurized docking
volume includes the recessed curved-wall throat on either side of its center.
The workshop also has a doubly curved roof while retaining its four service
bays, vehicle clearances and ramp alignment. Storage keeps its dome envelope.

Habitat layouts use a clear central circulation zone and perimeter equipment
bays. Utilities and occupancy-controlled lighting run overhead. Crew bunks
retain human dimensions across size tiers. Operating equipment remains
placeable rather than being baked back into the room mesh. Airlocks retain
their three chambers, four E-operated seals and pressure interlock.

Processing and service machines stay upright on the deck. Their default yaw
follows the local hull tangent, with the controls facing inward, including
when the host structure is rotated. N/M still provides a deliberate rotation
offset. Wall-facing units receive short rear service supports when close to
the pressure wall. Placement checks the actual rotated footprint, headroom,
other units, circulation and reserved door approaches.

Earlier saves realign installed units to the new deck and bay orientation,
moving a unit to another legal bay if needed while retaining its inventory,
batch state and production counters. If no alternative bay exists, the saved
unit is retained rather than discarded. Save schema53 appends the Very Small
tier without changing existing serialized tier numbers. Saves from earlier
schemas continue to load; settings remain game-wide, outside progress saves.

## Clear cutaway

The former faded selector is now **HIDE HULLS**. It removes the outer skin,
glazing, ribs, rings, hull edges and pressure partitions from the view. Habitat,
storage/workshop and corridor cutaways retain decks, machines, furnishings and
internal services. Exterior equipment remains visible. The view change does
not remove collisions, atmospheric boundaries or door requirements.

Dome containment and roof-clearance queries use their analytic ellipse rather
than repeatedly scanning the detailed skin triangles. Geometry is still built
and cached with the room models.

## Design references and validation

Layout decisions are informed by circulation, functional zoning and accessible
systems discussed in NASA's [Internal Layout of a Lunar Surface Habitat](https://ntrs.nasa.gov/citations/20220013669)
and ESA's [Moon Village habitat assessment](https://www.esa.int/es/articles/esa-engineers-assess-moon-village-habitat).
These are game representations, not engineering-certified pressure vessels or
radiation-shield designs. Transparent UV-filter skins retain the game's
existing visual convention.

See the package VALIDATION.md for executed checks and Windows test limits.
