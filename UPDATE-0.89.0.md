# Lunar Base 0.89.0

Baseline: 0.88.1. External JSD engine: 0.5.22, unchanged.

- Orbit/building crosshair is rendered in the 3D ground plane, with a segmented circular reticle, cardinal arms, high-contrast outline, green strokes and amber center. It scales with camera distance. The reference image URL was not retrievable; this is an independently chosen fallback shape. The FPV aiming crosshair remains camera-aligned.
- During landing-site selection N/M rotate the lander by 90 degrees in opposite directions. Its ground preview shows the hull, extended ramp, ramp direction arrow and rover apron. Lander model, thruster positions, approach view, rover deployment paths and headings, collision reservation and salvage points share the chosen orientation.
- FPV astronauts can walk and sprint up and down the deployed ramp and enter the lower cargo bay after rover unloading. The bay remains unpressurized; suit oxygen is required. The upper command capsule is not accessible. The ramp and collision floor use the same endpoints and slope. Salvage is blocked while a crew member occupies the accessible ramp/bay.
- EXIT FPV clicks are handled in the input update before world placement actions. Release the captured cursor with Alt or Tab, then click EXIT FPV; V remains a shortcut.
- The oxygen, canteen and bio domes return to oval ground profiles and curved crowns. Other habitat types retain rectangular bases with curved dome-style roofs. Structural ribs, interior fit, roof clearance, doorway normals and pressure collision follow the new profile. Storage domes remain round. Prior connector changes are retained.
- Save schema 56 stores the lander quarter-turn orientation; older saves default to zero. Back up valuable saves before loading the revised habitat geometry. Existing fittings are reprojected by the existing interface synchronization logic. A full migrated-colony replay was not run.

## Validation
See VALIDATION.md. Standalone C++ ramp and geometry checks passed; full native build, linked UI regression suite, desktop rendering, physical keyboard/mouse input, and Windows execution are unverified.
