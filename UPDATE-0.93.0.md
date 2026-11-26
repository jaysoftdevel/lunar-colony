# Version 0.93.0 — structure hover and oxygen efficiency

Baseline: 0.92.0. Engine: JSD 0.5.22, unchanged.

- The orbit-view ground reticle changes to the default native arrow when the cursor hits the same selectable ground footprint used by existing habitat, resource, corridor, walkway, utility and lander selection. It returns on empty terrain. Placement tools retain their ground reticle; HUD and FPV behavior remain unchanged. No extra mesh raycasting is performed.
- Oxygen generator nominal output doubles from 4.5/9/18 to 9/18/36 kg per in-game day for Very Small/Small/Medium units, including installed units. Electrical demand remains 1.5/3/6 kW. Water use at full output increases proportionally to 10.125/20.25/40.5 L/day, preserving electrolysis mass balance. Hydrogen byproduct remains proportional. Existing power, water and oxygen storage/pressure limits still apply. UI recipe labels reflect the new capacities.
- Save schema and single-binary normal/--debug build system unchanged.
