# Lunar Colony 0.82.0 — Fitted airlocks and clear transfer paths

Requires external JSD engine 0.5.21. Shared sources support Linux and Windows.
Save schema54 remains unchanged.

Airlocks now have a fixed factory layout: two axial end hatches, two internal
pressure doors and three chambers. All four doors exist when the airlock is
finished, before a corridor is built. Pumps, recovery/filter cartridges,
valves, vents, handholds, lights and wall-mounted control displays are fitted
with construction. The central transfer aisle stays clear in both Small and
Standard sizes. Optional floor terminals are no longer offered in airlocks;
legacy terminals remain saved but inactive and hidden, avoiding obstructions.

Corridors snap to a fixed end port rather than cutting a new off-axis hatch.
One longitudinal end connects to the pressure network; the opposite end has
its exterior hatch and access stair. An exterior walkway can reach that end
without needing to create a door first. Door locations and fitted equipment
remain fixed when connections are added or removed.

Pressure doors remain physical. The existing E-key request, powered pressure
interlock and held-E manual operation are retained. Closed seals and external
hulls are not removed. Open seals permit passage through the full central
route, with no duplicate transfer caps. Airlock doors remain visible in the
cutaway, making the remaining pressure boundaries explicit.

Corridor docking-frame lower ties and pressure flanges are below the walking
surface, removing raised transverse sills while retaining structural support.
Zero-depth fixed-port throats no longer emit coincident duplicate frames.

On load, older airlock corridor endpoints are aligned to the fixed axial
ports, and walkway endpoints to the exterior stair foot. Construction progress,
resources and astronaut stats are retained. No supplied save is overwritten.
The finite oxygen-scaled suit refill and low-wall cutaways from 0.81 remain.

See package README.md and VALIDATION.md for application commands and checks.
