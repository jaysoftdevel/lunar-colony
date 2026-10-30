#pragma once

namespace game::geometry {

// World units are metres.  Every crew-accessible dimension in the procedural
// geometry is derived from this 2 m reference human instead of maintaining
// independent, implicit scales per feature.
// Baseline astronaut scale. The old 2.00 m proxy made POV/navigation read too
// large against the structures. Future crew/astronaut implementations use this
// same canonical 80% scale unless a specific character archetype overrides it.
inline constexpr float kAstronautScale = 0.80f;
inline constexpr float kHumanReferenceHeight = 2.00f;
inline constexpr float kHumanHeight = kHumanReferenceHeight*kAstronautScale;
inline constexpr float kHumanEyeHeight = 1.82f*kAstronautScale;
inline constexpr float kHumanBodyRadius = 0.34f*kAstronautScale;
inline constexpr float kHeadClearance = 0.28f*kAstronautScale;
inline constexpr float kShoulderClearance = 0.26f*kAstronautScale;

// Common structure interface.  Existing structures keep their realistic
// building footprints, while all generated entrances/connectors share these
// dimensions.
// Crew-accessible structures now share the corridor deck elevation.  Older
// meshes used elevated hatch/floor assumptions (0.7-0.8 m) and then hid the
// mismatch with stairs in the docking sleeve.  Keep one continuous walking
// plane instead so a connected interior is physically passable.
inline constexpr float kHabitatFloorY = 0.20f;
inline constexpr float kStorageFloorY = 0.20f;
// Logical door anchor.  The pressure-surface solver may sample the hull at a
// higher Y, but gameplay/door frames use this common threshold reference.
inline constexpr float kEntranceSliceY = 0.50f;
inline constexpr float kDoorClearHeight = 2.45f;
inline constexpr float kDoorClearWidth = 1.40f;
inline constexpr float kDoorFrameHeight = 2.68f;
inline constexpr float kDoorFrameHalfWidth = 1.12f;
inline constexpr float kEntranceMinSpacing = 3.10f;
inline constexpr float kDoorTriggerRadius = 2.25f;

// Pressurised corridor.  The internal envelope gives a 2 m crew member useful
// head/shoulder clearance while retaining the rounded pressure-shell profile.
inline constexpr float kCorridorHalfWidth = 1.45f;
inline constexpr float kCorridorFloorY = 0.10f;
inline constexpr float kCorridorWallY = 0.85f;
inline constexpr float kCorridorRoofRadius = 1.65f; // vertical rise of the elliptical roof
inline constexpr float kCorridorDeckY = 0.20f;
inline constexpr float kCorridorDeckHalfWidth = 0.95f;
inline constexpr float kCorridorDockRun = 1.75f;
inline constexpr float kDockSleeveHalfWidth = 1.25f;
inline constexpr float kDockSleeveHeight = 2.72f;
inline constexpr float kGeneratedEntranceDepth = 1.70f;
inline constexpr float kGeneratedEntranceHalfWidth = 1.16f;
inline constexpr float kGeneratedEntranceHeight = 2.68f;
inline constexpr float kRouteClearance = 0.55f;

// Exposed corridor endpoints are hard-sealed pressure bulkheads.  Exterior
// crew access to the pressurised structural network is intentionally provided
// only by the dedicated Air Lock habitat.

// Exterior crew walkway: wide enough for two suited crew members to pass, but
// deliberately open and railing-free per the game design.
inline constexpr float kWalkwayHalfWidth = 0.95f;
inline constexpr float kWalkwayDeckY = 0.22f;
inline constexpr float kWalkwayStructureClearance = 0.65f;
inline constexpr float kWalkwayAccessHeight = 2.45f;
inline constexpr float kWalkwayBridgeRise = 0.95f;
inline constexpr float kWalkwayBridgeHalfSpan = 3.40f;

// Flush service carrier. The complete run stays visibly above the terrain,
// but its vertical profile is deliberately lower than a rover wheel radius.
// Rover navigation treats the channel as traversable, so robots drive across
// it instead of stopping/replanning around utility infrastructure.
inline constexpr float kSupplyChannelY = 0.035f;
inline constexpr float kSupplyTrayWidth = 0.70f;
inline constexpr float kSupplyTrayHeight = 0.055f;
inline constexpr float kSupplyWaterRadius = 0.040f;
inline constexpr float kSupplyJunctionWidth = 0.78f;

static_assert(kCorridorWallY + kCorridorRoofRadius - kCorridorDeckY >=
              kHumanHeight + kHeadClearance,
              "corridor must clear the 2 m crew reference");
static_assert(kCorridorHalfWidth - kCorridorDeckHalfWidth >= 0.50f,
              "corridor must retain service space beside the crew deck");

} // namespace game::geometry
