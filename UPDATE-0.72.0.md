# Lunar Colony 0.72.0

Requires game 0.71.0 and external JSD engine 0.5.20. Engine sources are unchanged.
Apply this source update to both Linux and Windows installations, then rebuild.

## Astronaut control

V immediately enters FPV. Selecting an astronaut first takes control of that
character; otherwise the last controlled astronaut is resumed. An unassigned
first character starts beside the lander's service/cache position. In a world
without a lander, the origin is the initial anchor. Click-to-place FPV is removed.

WASD movement moves the actual astronaut. Leaving FPV keeps that astronaut at the
exit position. View direction is remembered separately for each astronaut.
Previously controlled characters hold position unless given a colony-view move
order. Other crew can continue their existing tasks. NEXT cycles available crew.

V returns to the character's current/last position. A blocked return uses the
nearest clear point on a 0.25 m search grid, accounting for playfield bounds,
hulls, equipment, doors, packages, crew and rover occupancy. The search preserves
an available deck/support height. The first unassigned spawn stays outdoors.
Selecting an astronaut already indoors takes control there.

The original four named role profiles are shared by colony and FPV control.
Normal games create the first profile when FPV is first entered; Debug Stage 3
retains its existing four-astronaut roster. There is no independent immortal FPV
explorer. Taking control clears that astronaut's pending route and duty/bed claim.

## Individual vitals

The lower-right panel shows the controlled/selected astronaut's health, oxygen,
hunger, thirst and sleep need. Its background has 25% transparency (75% opacity).
It remains above the bottom configuration/build bar. Alt or Tab releases the
mouse for panel interaction and FPV building.

Vitals use existing finite suit oxygen, cabin air, canteen meals/water and the
lander's emergency reserves. Abstract colony residents exclude represented
astronauts to avoid charging their meals/air/water twice. Oxygen in the panel is
suit reserve outdoors or actual cabin breathability indoors. Hunger, thirst and
sleep need rise toward 100%; health and oxygen fall toward 0%.

REST HERE is available in breathable interior air or beside the emergency cache;
WAKE UP or WASD ends rest. This uses the existing sleep recovery rates. Deceased
astronauts cannot move or be entered. Vitals continue changing while simulation
runs, including outside FPV; pause stops their simulation.

## Physical packages

Ground, stacked, stored, driller and belt packages have opaque faces. Each of the
six faces carries a high-contrast material icon: iron beam, copper coil, silicon
chip, plastic/recycling, water drop, food leaf, medical cross or aluminum sheets.
Small distant icons are omitted below their readable screen size to reduce
per-frame geometry cost; nearby icons are available on every face.

Package rendering and FPV collision share stack poses and dimensions. A package
blocks the astronaut's swept body; its top supports the player after a jump and
its underside limits headroom. FPV retains the existing grab/drop interactions.

## Saves and compatibility

Schema 51 adds per-character pose, view direction, vitals, presence and finite
emergency reserves to both Release and Debug saves. Loading opens colony view;
press V to resume the last character. Existing older saves remain supported and
start with fresh FPV memory. The compressed save envelope remains schema 48.
New schema 51 saves require 0.72.0 or newer; retain an older save for rollback.

The historical 0.69.1 Windows-sync package also supports Linux; it is a separate
0.69.0 -> 0.69.1 update and is not a prerequisite installer for this package.
Read VALIDATION.md in the update package for checks and existing Debug issues.
