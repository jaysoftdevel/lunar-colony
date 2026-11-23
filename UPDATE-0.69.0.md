# Lunar Colony 0.69.0 / JSD 0.5.18

Apply to game 0.68.0 and separate JSD engine source 0.5.17, then rebuild.
The package updates the shared Linux and Windows sources.

## Gate frames

Interior pressure gates have a red border when disconnected and a green border
when a completed corridor route reaches another completed structure. The check
follows branches and automatic pressure crossings. Planned connections and
isolated capped corridor stubs stay red. A green frame describes the connection;
the normal pressure/interlock system still controls when its leaves open.

Unused warehouse docking frames are red. Connected warehouse crew gates are
green; cargo gates turn green when commissioned. A completed workshop's exterior
entrance is green. Markings appear on both sides and remain depth-tested.
Connectivity is cached until corridor paths or completion state change.

## Clear interior approaches

Each planned or completed corridor endpoint reserves a local, person-width
approach extending 2.65 metres into its host structure. Any operating assembly
that overlaps that space is removed as a whole: work surfaces, supports,
equipment, screens and their collision triangles. Warehouse service cabinets
use the same rule. Hulls are cut only by the existing completed-door logic.

The reserved space remains unavailable to station assemblies. Reservations are
derived from the saved corridor endpoints, so existing saves gain the clearance
when loaded. Cancelling/removing the connection releases its reservation and
restores the original stations. Repositioning or rotating a structure recomputes
the reservation in its local coordinates.

Reserved bunks are excluded from bed capacity and assignments. Reserved grow
bays cannot be planted and have their growth, light and water demand suspended;
their crop state is retained for restoration. The inspector labels them
GATE APPROACH RESERVED, and structure displays use the available capacity.

## Solar collectors

Collector plates, support struts, crossbars and moving harnesses now use one
rigid tracker transform. The mast, bearing and ground services remain fixed.
Explicit moving-part ranges replace the old per-vertex height heuristic and
floor clamp, which could separate or deform pieces during rotation.

Photovoltaic cell seams and conductors are mapped in the collector's own moving
basis. Translating, rotating and tracking therefore keep the surface pattern
attached to the plates. Standard and Small variants use their actual authored
geometry basis. Only the front collector faces use the cell material; sides,
backs and supports retain their structural materials.

JSD 0.5.18 adds PhotovoltaicCells as material recipe 16. Existing ordinals are
preserved. Derivative filtering softens fine cell lines at distance. The shader
program cache automatically invalidates changed shaders on the next launch.

## Validation and compatibility

All 37 Linux Release CTest checks and all 35 engine guard tests passed.
Coverage includes disconnected/planned/connected gates, automatic crossings,
changed topology, complete station removal, FPV clearance, reserved placement,
grow suspension, cabinet cache changes, and 9,216 solar mapping samples across
both panel sizes. All 32 native game scenes rendered, including pixel checks
for visible red and green borders and captures at two collector angles.

The source update is version- and hash-checked and preserves saves/preferences.
The save schema is unchanged. Native Windows compilation and a Debug play
session were not run here; platform build-driver checks pass. Native rendering
was verified on the available software OpenGL driver, not benchmarked on a GPU.
