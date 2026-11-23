# Lunar Colony 0.68.0 / JSD 0.5.17

Apply to game 0.67.1 and separate JSD engine source 0.5.16, then rebuild.
The same updated sources support Linux and Windows.

## Hulls and doors

Upper enclosure panels above the opaque waist now use transparent, blue-grey
solar-filter glazing. Habitat, corridor, warehouse, workshop and facility
skins retain opaque foundation decks, lower plating, structural ribs, frames
and machinery. Active solar panels, antenna masts and landing decks remain
functional opaque equipment. Existing glass uses the same protective coating.
Full Hull shows this complete structural design; FADED retains the cyan 12%
cutaway cue. Debug Meshed remains a wire view.

The film has a simulated UV-cut tint, neutral-density sun screening and a fine
ceramic frit pattern. Screening strengthens with sun exposure/orientation;
the pattern fades at a distance to avoid shimmer. This is a visual coating
approximation, not a new UV-radiation, thermal or crop-growth model.

Large cylindrical/domed pressure skins have smoother silhouettes. Remaining
opaque panels use metre-scale engineered joints, fasteners, restrained finish
variation, coated-metal roughness and exposed metal at seams. Capsule, dome and
industrial silhouettes retain their authored shapes.

Crew pressure doors, storage cargo shutters and workshop gates share beveled
faces, matching plated finishes, metal stiffeners/fasteners, rubber edge seals,
handles and warning markings. Door travel, apertures, physical envelopes and
pressure-control rules remain in place. Door leaves stay opaque.

## Mounted, meaningful displays

Wall terminals have back cases, bezels, mounting plates and standoffs that follow
the real skin at each bracket. Their positions use the actual pressure envelope,
including small variants and offset industrial bodies. Airlock consoles retain
their supported console mounts. Workshop displays have raised stems above the
cabinet and rover body, with visible status lamps.

Screens render physical, depth-tested glyphs in their local mounting basis.
They move/rotate with the structure and show measured state with explicit units:

- Airlock: chamber pressure, power availability and current cycle state.
- Oxygen: oxygen production in kg per game day, water use in litres per game day
  and cabin breathability percentage.
- Hydroponics: planted bay count, average crop growth and cabin air.
- Quarters: actual bed capacity, cabin air and power availability.
- Canteen/medical: available colony supplies, cabin air and relevant services.
- Processing/factory/manufactory/lab: actual power, air and water service values.
- Workshop: bay number, empty/repairing/charging/ready state, rover battery and
  hull condition. Empty bays show standby, not invented rover readings.

Powered text is emissive; unpowered terminals lose emission and show a passive
POWER OFF indication. There are no randomized telemetry bars. Glyph geometry
is cached and distance/back-face culled where the physical screen is too small
to read. Frames and screen substrates remain part of the model at those distances.

## FPV cursor

Hold either Alt key to release the pointer for the HUD/catalog or world placement.
Release Alt to return to mouse look. If Tab already released the cursor, releasing
Alt preserves that preference. Cursor transitions suppress mouse-look jumps and
stray placement clicks. Existing Tab toggle, placement, wheel/N-M controls and
V-to-exit remain available.

## Engine and compatibility

JSD adds PressureDoor and SolarShield material recipes and accepts them in the
runtime material validation. The minimum renderer now uses per-pixel material
roughness/metalness for shading. Shader-cache keys include changed sources and
the engine version, so new programs compile once and are reused on later starts.
Transparent upper/lower splits are retained, with exact source comparison to
handle corridor geometry rebuilt in an existing allocation.

Save format, inventories, robot jobs, logical collision envelopes and pressure
simulation are preserved. The star sky, global lunar surface, fullscreen splash,
shader cache and saved-rover recovery from previous updates remain available.
