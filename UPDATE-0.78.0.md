# Lunar Colony 0.78.0 — Interior finishes and explicit hatch controls

Requires external JSD engine 0.5.21. Shared C++20/SDL2 sources support Linux and
Windows 11. Save schema52, earlier save loading, game-wide preferences and
fullscreen startup are unchanged.

## Interior surfaces and common hatch design

Habitat floors now have one consolidated deck finish instead of overlapping
copies of authored plates and deck skins. A planar sweep unions the plates,
including partially overlapping areas, and keeps station/furniture ranges valid.
The work runs when models are created, not every frame. All 40 habitat type/size
variants retain their usable floor and existing placed units.

Personnel, airlock, storage and workshop leaves share a common beveled pressure
panel design: inspection windows, captive perimeter fasteners, a vertical grip,
reinforcing rails and restrained hazard markings. Airlocks retain their extra
pressure-locking hardware. The window openings are cut into the panels and
sealed with glazing. Moving leaves meet without coplanar skin overlap; hidden
seals and the existing collision envelope retain closure. Jamb/header joints
and double-gasket loops no longer share overlapping exposed surfaces.

Lighter deck finishes, restrained material relief and matte interior/door
surfaces reduce harsh highlights and fine-detail shimmer. The FPV near plane
is now 8 cm, improving depth precision while retaining close inspection and
the existing distant lunar view. Structural ribs, upper solar-filter glazing,
custom furnishings and operational units remain available.

## E-key operation

Approaching a door in FPV no longer commands it to open. Look at a hatch within
3.2 metres to see the E-key prompt. Another closed hatch in front of a target
prevents reaching through it.

- Powered: press E to open, cancel an opening request, or close. Airlocks close
  the preceding seal and equalize pressure before opening the requested seal.
- Unpowered: hold E to operate the manual crank. A full opening or closing takes
  approximately three seconds. An amber ring around the pressed E key follows
  the actual movement. Release E to stop at the current partial opening; resume
  while still facing the same hatch. Release after completion before holding
  again for the opposite action.
- Moving out of reach, looking away, leaving FPV or pausing cancels the hold.
  A partially opened unpowered leaf does not keep moving by itself.
- An unpowered airlock cannot open a second hatch until the other hatch is
  sealed. The prompt explains the interlock. Powered airlocks retain their
  pressure matching and one-open-seal rule.

Powered opening requests last eight seconds, followed by the existing short
hold/closing delay. Occupied openings stay clear. Autonomous crew and working
rovers still request their own route/service access; idle player proximity is
not an opening request. Exterior workshop/cargo gates use the same E workflow.

Door requests, hold input and animations are transient and are not added to
save files. The colony, saved crew state and placed interior units are retained.
See package VALIDATION.md for test results and platform limits.
