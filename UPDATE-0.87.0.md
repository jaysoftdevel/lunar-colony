# Version 0.87.0 — corridor connector structure and processing interiors

## Gameplay and building

- Add structural deck/support geometry to corridor connection sleeves at structure interfaces.
- Keep connector walkway/floor geometry visible in **Hull = None** mode so no-hull cutaways still show a usable ground/deck path.
- Refine corridor entrance floor/support details to look less flat and more adaptive around the connection ramps.
- Restore valid **Ore Refinery** placement inside the **Small Processing Facility** by relaxing the oversized reserved center-core exclusion for that dedicated machine.
- Add **Plastic Fabricator** availability to the **Processing Facility** interior-unit options, giving the player a plastic-processing machine from the processing-room panel.

## Regression coverage

- Extend `ui_regression` to require legal refinery and plastic-processing bays in the small processing facility.
- Extend no-hull corridor coverage with a synthetic connector case that requires preserved lower deck/support geometry near both connected structures.

## Notes

- Save format unchanged.
- External engine version unchanged: **JSD 3d-engine 0.5.22**.
