# Lunar Colony 0.67.0

Update from game 0.66.0 with JSD engine source 0.5.16. The engine remains
0.5.16; the fullscreen splash, persistent shader cache, GPU guides and bounded
edge scrolling from the preceding updates remain available on Linux and Windows.

## Faded hulls and build guides

The former NONE hull button is now FADED. Completed habitat, corridor, storage
and workshop shells/glazing retain a cyan tint at **12% opacity**, with cyan
edge guides. The generated storage loading chamber follows the same rule.
Foundation decks, equipment and doors remain readable. Collision, atmosphere,
construction and all other gameplay still use the physical structure.
Debug Meshed mode uses cyan wire guides rather than filled faded surfaces.
Uninstalled construction parts remain blue engineering drafts.

In build mode, existing structures and the candidate show stronger boundaries:
bright cyan for blueprints, amber for completed foundations, dark backing and
raised corner posts. Invalid candidate reservations are red. These are the
same reserved ground footprints used by placement validation.

## First-person construction

The bottom catalog, hull controls and simulation-speed controls remain visible.
Press **Tab** to release the cursor, choose a building from the bottom bar, and
place it using the ground under the pointer. Press Tab again for mouse look
and center-crosshair placement. Walk with WASD to adjust the viewpoint.
The preview uses the first-person camera/FOV, validates the usual clearance,
rover escape and placement rules, and rejects clicks on the visible HUD.
Aim down at ground within 90 m; upward/sky rays cannot place anything.

- Mouse wheel: structure size, where supported; N/M: rotate by 15 degrees.
- LMB: place; Shift+LMB: retain the tool for repeat placement.
- RMB or Esc with a tool: cancel the tool. V exits FPV.
- Structure, corridor, supply-channel and walkway previews share FPV picking.

FPV boot sounds now combine heel impact, rolling sole and toe/scuff contacts,
with four variants for dust and deck surfaces and much less ringing on metal.
Cadence follows actual movement: about 116 contacts/min walking and 159/min
sprinting at normal steady speeds. Standing still, pushing against a wall,
teleport-size discontinuities and airborne movement do not emit walking steps.
Landing retains its separate impact cue. Colony speed does not speed up footsteps.

## Saved carrier recovery

The attached 0-65-0_simple save put a loaded carrier in the long, narrow cargo
stack aisle north of the lander. Its real pose was legal, but both the coarse
lattice and trip-aligned refinement could disconnect that aisle from the rest
of the navigation graph. Repeating a path reset recreated the same failure.

A final, persistent fine aisle graph anchors a grid row at the exact rover pose,
allowing a route to reverse out of the aisle before heading to the delivery
station. Search propagation retains the existing expansion/time slices. The
usual swept hull collision checks, fleet communication, yielding and certified
motion contracts still own movement. No rover is relocated or passed through
cargo. Save format and resource balances are unchanged.
