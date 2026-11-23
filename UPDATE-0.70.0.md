# Lunar Colony 0.70.0 / JSD 0.5.20

Apply to **game 0.69.1 / external engine 0.5.19**, then rebuild both game
configurations. The same source update supports Linux and Windows and retains
the Windows integration changes from 0.69.1.

## Longer day

The full day/night cycle is now **480 seconds at 1x speed**, twice the previous
240 seconds. The current hour and day are retained when loading a save.
Production, crop growth, life support, battery charging, electrical loads and
suit range use the same longer clock. Rates displayed per game day/hour retain
their meaning. Rover motion, walking, construction animations and door travel
retain their physical speeds.

## Corridor floors

Corridor decks use a separate engineered floor material, with inset tile faces,
gasket joints, beveled lips, fasteners, non-slip diamond tread, individual tile
finish and restrained wear. Metallic response and roughness vary across seams,
grip and exposed edges. Fine patterns fade with screen derivatives at distance.
Floor coordinates follow the corridor basis; rib and wall fittings retain their
own material. JSD appends CorridorDeck as recipe 17, preserving earlier recipe
numbers. Changed shader programs compile once and enter the existing cache.

## Powered and manual doors

Unused docking candidates and capped corridor ends no longer show red borders.
Actual doors show **red when their motor supply is unavailable**. A powered
completed connection retains its green border. Cargo gates use their owner's
power supply and the gate's power switch; workshops use their own supply.

Unpowered doors do not respond to proximity. Their brakes hold the current
position until manually operated or powered again. In FPV, approach an unpowered
door and look toward its center from within 3.2 metres. Press **E** to manually
open/close it, or hold **Alt** / toggle **Tab** and click the manual button in
the FPV information panel. Manual operation is slower than the motor and has
no electrical cost. Obstruction protection remains active during manual travel.
Airlock manual controls still require the other hatches to be sealed first;
a rejected request is explained in the activity log. Power restoration returns
the door to automatic operation. Manual latch positions are transient on load.

## Empty, configurable habitat interiors

Habitat templates no longer contain their preinstalled operating machinery,
including their old process tanks and crane assemblies. Oxygen generators keep
their electrolysis equipment. Hulls, structural supports, floors, lighting,
mounted building controls, seating, lockers and crew bunks remain.

Exterior resource structures ignore the habitat Hull view selector: switching
Full Hull/None no longer fades their equipment or enclosures. Designed glass
and solar-filter surfaces retain their normal materials.

Selecting a habitat centers it with a 35.264-degree isometric view and a yaw
45 degrees from the building's local axes. The selected enclosure fades so
its floor and installed units can be seen. This does not move the structure
or alter collision/pressure. FPV selection does not move the walking camera.

Scroll the selected structure's panel to **INTERNAL UNITS**. Choose a compatible
unit, move its preview over the floor, and click to install it. **N/M** rotates
15 degrees, **Shift** repeats placement, and **RMB/Esc** cancels. Positions snap
to a 0.25-metre grid in the host's local coordinates. Installation consumes the
listed materials immediately as a prefabricated unit.

Units must fit under the pressure hull and cannot overlap furniture, machinery,
the central walking aisle or any planned/completed doorway approach. Connected
doorways remove overlapping units from rendering/collision and suspend their
operation; their saved state remains available if that connection is removed.
The per-unit panel supports switching units off and removing/scrapping them.
Scrapping an active batch discards its consumed inputs; the button says so.

| Unit | Compatible structures | Nominal load | Batch |
| --- | --- | --- | --- |
| Metal fabricator | Factory, Manufactory | 3 kW | 5 kg raw ore to 3 kg matching refined stock, 0.25 day |
| Plastic fabricator | Factory, Manufactory | 2 kW | 1 kg silicon + 2 kg water to 2 kg plastic, 0.5 day |
| Ore refinery | Processing Facility | 4 kW | 5 kg raw ore to 3 kg matching refined stock, 0.20 day |
| Medical synthesizer | Laboratory, Sick Bay | 1.5 kW | 1 kg food + 1 kg water + 1 kg plastic to 1 kg medical, 1 day |
| Hydroponic rack | Bio Dome | Crop dependent | One selectable crop bay using the existing seed/growth system |
| Service terminal | All configurable habitats | 0.1 kW | Live unit/room service status; no material production |

Rooms also have a 0.3 kW service load. Oxygen-generator loads retain their existing
definition. Material-processing units require enabled power and at least 95% cabin
breathability. Hydroponics retains reduced growth at 80-95% air and full speed
at 95%; service terminals can monitor an unpressurized room.
Feedstock is consumed once when a batch starts; partial progress pauses when
services fail. Output is emitted as existing physical resource pallets, and a
blocked output waits without charging inputs again. Unit screens show real
state, recipe, power load and completed batch count.

Ore carriers choose processing destinations only when a compatible unit is
installed. They deposit raw ore in the room's saved input buffers. Delivery no
longer refines ore by itself, and empty buildings no longer generate automatic
plastic/medical production. Existing cargo already travelling by belt can finish
delivery into the raw buffer.

Existing Bio Dome crop state is retained, with growth and loads suspended until
an enabled rack is installed for the corresponding bay. Missing, disabled and
doorway-reserved racks are identified in the crop selector. Crew bunk capacity
and the oxygen generator continue to work without installing replacement units.

## Saves and update

Schema 49 stores unit type, position, rotation, enabled state, active batch,
progress, grow-bay assignment, completed batch count and raw input buffers.
Earlier saves, including compact schema 48, remain readable. Older game binaries
cannot read schema 49 saves. The updater backs up affected source files, checks
versions and hashes, rolls back failed writes, and does not change save files or
preferences. See the package's VALIDATION.md for actual checks and platform limits.
