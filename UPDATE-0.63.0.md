# Lunar Colony 0.63.0

Apply to game 0.62.0 and the separate JSD engine source 0.5.13. The package updates
both to game 0.63.0 / engine 0.5.14. Build with the engine path argument as before.

- Hangar/workshop: transparent pressure-glass barrel roof with transverse arches,
  longitudinal mullions, grounded columns and footplates. Hull None hides the glass
  and cladding; frames and equipment remain grounded. Glass retains ceiling collision.
- Each of four parking bays has service rails, tool drawers, diagnostics, pneumatic
  equipment, hoses, suspended task lights and a repair tool. Powered parked robots needing repair deploy
  the tool; the amber indicator and tool motion follow actual repair progress.
  Charging, ready, idle and offline states have distinct indicators. Existing automatic
  repair/charging behavior is retained; blueprint workshops do not service robots.
- Drilling: gearbox torque, rock fracture and rubble chatter; actual camera-eye
  distance controls presence in orbit and first person. Viewport edges fade smoothly
  to quieter offscreen sound. Live engine voice updates preserve audio playback
  while panning/zooming and smooth gain changes without restarting the sound.
- Raw-ore stacks avoid planned and completed supply-channel/conveyor routes,
  structures, corridors and walkways. Carrier approach and pickup animations use the
  same reserved outlet. New channel plans invalidate stale pickup goals. If no safe
  outlet remains, stock stays buffered; ore is neither deleted nor put on conveyors.
- Switched-off structures now show a red power symbol instead of the blue info icon.
  Enabled structures retain their existing supply warnings and storage indicators.

All previous materials, interiors, FOV/FPS options, sprint/jump, sky and ore hauling
features are retained. Saves keep their current format. Linux/Windows use identical
sources; rebuild Release and Debug after updating. See VALIDATION.md for results.
