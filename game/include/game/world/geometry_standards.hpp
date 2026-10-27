#pragma once

namespace game::geometry {

// World units are metres.  Every crew-accessible dimension in the procedural
// geometry is derived from this 2 m reference human instead of maintaining
// independent, implicit scales per feature.
inline constexpr float kHumanHeight = 2.00f;
inline constexpr float kHumanEyeHeight = 1.82f;
inline constexpr float kHumanBodyRadius = 0.34f;
inline constexpr float kHeadClearance = 0.28f;
inline constexpr float kShoulderClearance = 0.26f;

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

// Every pressurised corridor endpoint exposed directly to the exterior is
// terminated by a two-door air-lock chamber.  The chamber extends beyond the
// routed corridor centerline; connecting another corridor to that endpoint
// consumes the air-lock and restores a seamless pressure-tube splice.
inline constexpr float kCorridorAirlockLength = 3.60f;
inline constexpr float kCorridorAirlockHalfWidth = 1.34f;
inline constexpr float kCorridorAirlockDoorSpacing = 3.60f;

// Exterior crew walkway: wide enough for two suited crew members to pass, but
// deliberately open and railing-free per the game design.
inline constexpr float kWalkwayHalfWidth = 0.95f;
inline constexpr float kWalkwayDeckY = 0.22f;
inline constexpr float kWalkwayStructureClearance = 0.65f;
inline constexpr float kWalkwayAccessHeight = 2.45f;
inline constexpr float kWalkwayBridgeRise = 0.95f;
inline constexpr float kWalkwayBridgeHalfSpan = 3.40f;

// Low service carrier.  These values keep the tray/pipe visibly serviceable
// while clearly smaller than crew-access infrastructure.
inline constexpr float kSupplyChannelY = 0.30f;
inline constexpr float kSupplyTrayWidth = 0.62f;
inline constexpr float kSupplyTrayHeight = 0.18f;
inline constexpr float kSupplyWaterRadius = 0.18f;
inline constexpr float kSupplyJunctionWidth = 0.78f;

static_assert(kCorridorWallY + kCorridorRoofRadius - kCorridorDeckY >=
              kHumanHeight + kHeadClearance,
              "corridor must clear the 2 m crew reference");
static_assert(kCorridorHalfWidth - kCorridorDeckHalfWidth >= 0.50f,
              "corridor must retain service space beside the crew deck");

} // namespace game::geometry
