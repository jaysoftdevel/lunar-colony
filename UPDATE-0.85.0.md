# Lunar Base 0.85.0

Requires the unchanged external JSD 3d-engine source version 0.5.22. Shared Linux/Windows game sources. Save schema unchanged (54).

## Carrier diagnosis and recovery

The supplied save is paused, at day 20, with both carriers idle, empty, and without an assigned construction target. Their batteries are about 20% and 21%. There is no workshop, commissioned storage loading gate, or installed ore refinery. Ore logistics cannot invent an accessible destination, so an idle carrier does not necessarily have a navigation fault. Build a storage loading gate or install an appropriate refinery, then resume simulation. Bootstrap charging is now requested before the ore-dispatch cutoff for otherwise idle carriers. Idle cleanup no longer cancels an active charging, hauling, trading or safety navigation request, and traffic-yield recovery recognizes charging trips. No teleportation or fabricated work is introduced.

## FPV and displays

- L toggles a visor-mounted point light for the currently controlled living astronaut while the FPV cursor is captured. The state is per astronaut for the current session. Light rendering requires the lunar graphics path and compatible local lighting support.
- The existing finite suit tank (0.28 kg, nominal eight in-game hours) remains independently tracked, refilled and saved. The HUD always shows its remaining capacity separately from cabin air.
- Interior display lettering uses cached triangulated outlines generated from the bundled OFL font atlas, rather than pixel block glyphs. Generation is offline; no new runtime font dependency.
- Game label scaling increases by 20% (1.40 to 1.68). The astronaut and rover panels prioritize vitals, work, cargo, power and relevant blockers. Detailed navigation/coordinate and material bookkeeping is retained in debug mode. Essential controls remain available.
- Hull-off filtering uses the local corridor deck elevation instead of absolute ground-level height so elevated supports remain visible. Structure foundations are retained.

## Corridors

The supplied airlock-to-oxygen-dome path starts at y=0 and jumps to y=0.60 m in approximately 0.12 m. Interface synchronization had reset the endpoint datum. Short habitat-to-habitat routes now use a common continuous base aligned to the lower door threshold, including during save loading. Docking floors follow the actual wall, outer hatch and corridor ring, and FPV floor sampling follows the same spans. Small terminal leads align with the door normal. The original save is unchanged and is included as a regression fixture. Load it in the updated game and re-save after checking the repaired connection.

## Validation limits

The external JSD source tree was not supplied; a genuine Linux/Windows build and an interactive saved-game replay could not be performed. Portable Linux build argument checks and 12 Windows build-driver tests passed (these use mocks, not native game builds). The font generator validated all printable ASCII outline meshes. A C++ regression for the damaged short docking path was added but requires the external engine headers/build to run. See VALIDATION.md.
