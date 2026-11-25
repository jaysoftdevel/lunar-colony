# Lunar Base 0.88.1 — fitted corridor connectors

Baseline 0.88.0. Engine JSD 0.5.22 unchanged. Save schema 55 unchanged.

- Replace independent rectangular decorative connector frames with continuous rounded load-bearing bands following the same profiles as the pressure sleeve. The outer hatch profile now has a 0.28 m crown, with height increased to clear hatch hardware. Functional sliding doors retain their rectangular clear openings.
- Fit intermediate ribs at no more than 0.82 m bay spacing. Join them with longitudinal stringers trimmed to the flange faces. Remove duplicate freestanding hatch frames, threshold blocks, terminal rings and the former duplicate support sets.
- Locate footing pads at the lunar build plane. Cut the column tops to the actual underside of the oval bands rather than extending into the hull or deck. Correct the outward triangle winding and omit coincident skin-facing triangles. Keep the existing main corridor profile and room shapes.
- Preserve the stair and deck surfaces in both hull modes. Omit zero-height stair riser triangles. Match terminal profile orientation for opposite-end fittings; use one shared rounded door profile for pressure geometry, attachment sampling and airlock fittings.
- Existing saved connections regenerate their meshes on load. No inventory or save schema change. Back up important saves before updating.

Validation: incremental native Linux Debug build and link against JSD 0.5.22; eight targeted CTest tests passed. Includes new raised/rotated/reversed connector checks for crown contact, outward flange, ground-bearing pairs, nondegenerate geometry and clear walking lanes. Existing tests cover pressure collars, sealed joins, no-hull floors and stairs, airlock FPV, load frames and saved-carrier recovery. The test build reused an older CMake version-definition cache; the supplied VERSION file and ordinary build launcher select 0.88.1. The precise colony from the screenshot was not supplied; no interactive visual or full mesh-intersection audit, Windows native build, or Linux Release build was performed.
