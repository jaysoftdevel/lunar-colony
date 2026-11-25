# Lunar Colony 0.74.0 — corridors, pressure doors and FPV clearance

Requires external JSD engine **0.5.21**, unchanged from 0.73.0. Linux and Windows
use the same C++20 sources and existing SDL/OpenGL renderer.

- Corridor floors now have three lanes of solid chamfered removable panels,
  recessed joins, captive screw heads/slots, six raised grip strips per cassette
  and edge support girders. Panels follow route distance through bends; shared
  junction chambers use clipped versions of the same floor geometry.
- Hoops, underfloor ties, bearing pads and junction posts share consistent ground
  and deck references. Floor/metal draw batches are prepared when a corridor is
  rebuilt, avoiding per-frame triangle repartitioning.
- Airlock leaves have locking dogs, mechanical handwheels, separate equalization
  valve hardware, grab handles and dual fixed gaskets. Live door panels show seal,
  pressure and interlock state. Existing pressure cycling, power requirements and
  FPV E manual operation remain active. This is game geometry, not a certified
  pressure-vessel design or a detailed latch-mechanics solver.
- Airlock floor-standing lockers, benches, tanks and console were removed. The
  transfer aisle retains pressure partitions, lights, vents and wall handholds;
  controls are attached to the hatches instead of unrelated wall screens.
- Oxygen generators now have solid volume collision, including a saved body
  wholly inside a tank/cabinet. FPV movement and nearest-safe spawn share that
  collision. Jumping onto low installed equipment now lands on its top instead
  of falling into the solid body. Service metal/furniture participates in crew-body
  clearance.
  Small-dome screens were moved clear of generator placement; legacy low lamps
  were removed and overhead services raised above the 2.70 m equipment envelope.
- Structure clicks open their inspector even when an astronaut remains selected
  after FPV. Shift-click explicitly orders that astronaut to walk inside a
  structure; ordinary ground clicks still issue the usual walk command.

Existing save schema 51 and compact wrapper 48 are unchanged. A saved embedded
FPV position is moved to the nearest clear floor when entering FPV. This update
contains source only: apply it to the game and separate engine roots, then build.

Design reference: NASA OCHMO-TB-028, Vehicle Hatches (2023), for recognizable
seals, latching, pressure indication and manual equalization controls:
https://www.nasa.gov/wp-content/uploads/2023/03/vehicle-hatches-technical-brief-ochmo.pdf
