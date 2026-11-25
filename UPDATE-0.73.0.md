# Lunar Colony 0.73.0 — structural hulls and corridors

Uses JSD engine 0.5.21. The cumulative source package supports the documented
matching game/engine pairs from 0.69.0 onward and installs game 0.73.0 / JSD
0.5.21. Existing 0.72.1/0.5.21 installations retain their engine source.
Windows and Linux share the same geometry and native build interfaces.

## Hull construction

Every habitat and resource structure now has additional solid structural mesh.
The upper pressure/glazing surfaces retain their solar-filter transparency;
primary and secondary structural members stay opaque.

| Shape / structure | Added construction |
| --- | --- |
| Cylindrical habitats | Closed transverse I-section ribs, longitudinal stringers, supported end-bulkhead grids and foundation cradles |
| Domes | Meridional ribs, closed circumferential tension rings, crown hub, grounded stanchions and bolted foot plates |
| Factory/manufactory halls | Portal frames, roof purlins, wall girts, knee braces and open internal transfer frames |
| Biodome | Solid arch ribs, longitudinal glazing mullions, supported end frames and deck ties below the walking surface |
| Storage domes | Shape-matched ribs and rings for every storage size, with docking apertures still cut only at real connections |
| Robot workshop | More roof arches and grounded side columns, preserving the full rover aisle and gate envelope |
| Outside equipment | Real panel-edge/girt framing, connected chassis skids and equipment-specific supports |
| Tanks | Vessel clamp rings and saddle supports; the cryogenic sphere retains its spherical pressure shell |
| Antenna / landing pad | Grounded mast braces; radial girders below the landing surface |
| Solar collector | Perimeter rails and underside cross-battens in the moving collector range; foundation braces stay fixed |

Primary sections have visibly separate flanges and webs, with nominal web and
flange thicknesses of several millimetres. Connections have doubler/foot plates
and hex bolt heads. Frames are rebuilt after size scaling so larger habitats get
more bays rather than larger unsupported spans. Supported glazing grids use a
nominal maximum bay of 1.5 m. This spacing and section sizing are game design
choices, not values certified for an actual pressure window or habitat.

## Corridors and junctions

The previous corridor ribs were line outlines. They are now solid I-section
hoops, with an end frame and intermediate bays no longer than 1.35 m. Seven
continuous stringers follow the swept path. Under-deck ties and supports stay
below the walking surface. Framing stays inside the existing pressure-envelope
width and clear of the central crew/head envelope.

Docking apertures have larger outboard jamb/header doublers and bolted corner
plates. Junctions have roof perimeter girders, floor ties and corner columns.
Existing crossing cuts remove frame fragments from shared junction volumes;
new headers transfer around the openings. Factory/manufactory transfer bays
remain open rather than becoming new internal walls.

## Geometry, physics and performance scope

This is a structural geometry and collision-clearance upgrade, not an FEA,
material-strength, buckling, fatigue or structural-failure solver. The layout
uses continuous load paths, curved pressure shells, closed rings, supported
bulkheads and triangulated equipment supports. It does not claim that the
existing fictional transparent habitats are certified real-world designs.
The sealed gas model, construction costs and save schema 51 are unchanged.

Members are generated once into model/corridor meshes, then use the existing
cached rendering/material pipeline. No extra per-frame structural solver or
shader is introduced. The regression suite limits generated geometry per model,
checks finite non-degenerate faces, bay spacing, crew/transfer clearances,
opaque framing with transparent UV skin and solar tracking ranges. Actual
performance on the user's GPU is not claimed from offscreen test rendering.

## Design references

- [Composite Structure Enabling an Efficient Lunar Surface Habitat, 2025](https://ntrs.nasa.gov/api/citations/20250009434/downloads/Comp_Struct_Enabling_Eff_Lunar_Surf_Hab_Ppr_R2.pdf): pressure-shell structure, longerons and reinforced axial window/bulkhead interfaces informed the hull framing layout.
- [NASA space-station module and berthing interface architecture](https://ntrs.nasa.gov/citations/19840014544): flanged, circumferentially clamped sealed connections informed the reinforced docking collars.

The specific mesh layouts and member dimensions are adaptations for this game.
Read the update package's VALIDATION.md for native/portable checks and the
previously documented Debug navigation and crew-autonomy limitations.
