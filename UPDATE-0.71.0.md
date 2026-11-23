# Lunar Colony 0.71.0 — Interior services

Requires game **0.70.0** and separate JSD engine source **0.5.20**. The engine
stays at 0.5.20. Rebuild using the normal Linux or Windows game build script.

## Furnish interiors

Select a completed interior structure and scroll to INTERNAL UNITS. Choose a
unit, click its deck, N/M rotate 15 degrees, Shift repeats, RMB/Esc cancels.
Placement also works in FPV. Costs, floor/roof clearance, existing furniture,
central aisles and connected/planned door approaches are checked.

Canteens offer a food processor, drinking well, dining table and two-seat
bench. Fixed dining groups are cleared to make room for custom furnishings.

| Unit | Width × height × depth, m | Load | Function |
| --- | --- | --- | --- |
| Food processor | 1.6 × 1.7 × 1.0 | 1 kW | 2 kg stored food + 1 L stored water → 2 kg prepared meals per 0.1 game-day; 20 kg reserve |
| Drinking well | 0.8 × 1.4 × 0.7 | 0.08 kW | Connected water tank/driller → 10 L drinking reserve, up to 2 L/game-hour |
| Dining table | 1.6 × 0.78 × 0.9 | None | Four-place dining surface |
| Seating bench | 1.5 × 0.85 × 0.55 | None | Two-place passive furniture |
| Oxygen generator | 3.0 × 2.7 × 1.4 | 6 kW | Up to 18 kg O₂/day from 20.25 L water; produces 2.25 kg H₂/day |

Ingredients are charged once when a food batch starts; power loss pauses it.
Meals stay in the processor instead of being counted again as raw inventory.
The well transfers existing connected water; it never creates water. Astronauts
inside the canteen draw from finite meal/water reserves before emergency stock.
The normal colony economy also consumes prepared meals before raw food stock.
Stored drinking water can be tapped without pump electricity. Tables and
benches are passive furnishings, without a seated-animation AI.
Removal scraps a unit and discards its stored ingredients/output.

Oxygen domes include one fitted generator on completion. Older saves receive
the same equipment once. It can be switched off, removed and replaced through
the catalog; removing it never makes it respawn. Extra generators have real
costs and loads. A fully reserved room cannot auto-fit equipment; its placement
catalog remains available. The pressure roof and smallest footprint have more
clearance for the larger equipment. Original fixed processing equipment is
removed. Electrolysis can start in vacuum and keeps the existing hydrogen
storage/refueling and oxygen reserve/export systems.

## Pressure-driven airflow

Rooms, sealed service manifolds and ducts form an isothermal finite-volume
pressure network. Pressure follows the ideal-gas inventory/volume relation.
Compressible orifice flow includes the sonic/choked-flow limit; route length
affects duct resistance. Engineering assumptions: 80 mm service trunk,
discharge coefficient 0.65, Darcy friction factor 0.025 and local loss 1.5.

A backward-Euler conductance solve conserves air inventory, supports reverse
flow and prevents negative masses/pressure overshoot. Small service manifolds
do not impose artificial per-frame transfer caps. Physical time follows the
480-second game day: one game second corresponds to 180 physical seconds.
Disconnected/unfinished supply components cannot exchange gas. Downstream
pressure may change in the first step without immediately becoming full.

Cabin volume is estimated from footprint × deck-to-roof height × 0.65.
Small external service manifolds use 0.02 m³; duct/corridor volumes follow route
length and nominal cross-section. These are approximate envelope volumes.

The atmosphere remains the saved oxygen-equivalent inventory for a fixed
30 mol% O₂ / 70 mol% implicit inert buffer, at 293 K and 70 kPa target.
This reduced model does not separately track nitrogen, CO₂, diffusion,
temperature gradients, buoyancy or spatial turbulence. It is not full CFD.
Service-terminal screens also show cabin pressure in kPa.

Equation references:
- NASA Glenn: https://www.grc.nasa.gov/www/BGH/mflchk.html
- NASA mass-flow equations: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/mass-flow-rate-equations/
- NIST oxygen molecular mass: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7782447&Mask=1805

## Night lighting, construction sound and storage doors

All completed powered interiors have automatic night lighting. Empty rooms
remain at 18% brightness. An astronaut, FPV avatar or rover entering the room
raises lighting to full. Exterior/below-deck traffic does not count. Room lights
stop at sunrise; power-off/brownout below 50% extinguishes them. Storage domes
and workshops follow the same controller. Light emission, local light spill
and electrical demand use that state. Local light spill needs Local Lights
enabled. Equipment displays, approach lights and crop lamps have separate
controls; crop lamps retain their grow schedule.

Visible rover and astronaut construction uses positional welding/building
sound with eye-distance attenuation. Moving outside the viewport fades the
tool to silence; pausing stops construction loops. The synthetic cues follow
the game's receiver/suit audio convention in the lunar vacuum.

Storage domes no longer show four unused entrance panels. Real corridor
doors/collars and cargo gates retain their powered/manual operation.

## Save compatibility

Schema 50 saves finite meal/water inventories and the one-time generator
installation latch. Older schemas 1–49 remain readable. Older game versions
cannot open new schema-50 saves. The update installer leaves saves/preferences
alone. See VALIDATION.md in the package for test results and known limitations.

