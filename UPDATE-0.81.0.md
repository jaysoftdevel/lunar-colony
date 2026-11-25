# Lunar Colony 0.81.0 — Readable cutaways and suit oxygen refill

Requires external JSD engine 0.5.21. Shared C++20/SDL2 code and the same guarded
update launchers support Linux and Windows. Save schema54 is unchanged.

## No-hull view

HIDE HULLS now retains an opaque lower perimeter, approximately 0.72 m above
the interior deck, along with all floors, machines, furniture and internal
displays. Habitat walls use distinct functional colors, with floor stencils
such as OXYGEN, BIO, CREW, FACTORY and LAB. Storage/workshop enclosures retain
their lower boundaries and identifying floor labels too. Corridors retain a
low side wall that follows their deck elevation and slope.

Roof-mounted circular service runs and their lamp rings are separate from
internal equipment. They disappear with the upper hull rather than hovering
over the exposed room. Full-hull view keeps the original overhead assemblies.
Internal units, their geometry and their screens are not cropped by height.
The mesh/debug view also retains the internal material layers.

This is a rendering change. Walls, seals, interior collisions and atmospheric
connections retain their physical behavior while hidden.

## Suit refill inside oxygenated cabins

Living astronauts, including the directly controlled FPV astronaut, now refill
their finite suit tank from oxygenated room air. The refill rate is proportional
to the current room oxygen percentage, capped at the nominal full-air rate.
At full air the nominal empty-to-full fill time is about 40 real seconds;
100%, 50% and 25% air provide 100%, 50% and 25% of that fill rate.

Breathing continues during filling, so a suited astronaut in very poor air
may still use oxygen faster than the refill pump supplies it. Empty cabins
and vacuum cannot refill the suit. Tanks stop at their existing 0.28 kg
capacity. Every transferred kilogram is removed from cabin air or connected
oxygen storage; normal life support must replace it. Cabin-air filling does
not require the building to be powered. Powered connections can continue
drawing from the existing network oxygen tanks. The lander cache retains its
existing finite-stock refill behavior.

Airlock cycling chambers remain excluded from cabin intake, preserving the
existing pressure/isolation rule. Suit oxygen already persists per astronaut,
so no new save field, progress reset or settings migration is needed.

Generator Very Small / Small / Medium choices from 0.80 remain per-machine
choices inside the original oxygen-dome building sizes.

See package VALIDATION.md for executed checks and Windows validation limits.
