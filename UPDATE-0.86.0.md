# Lunar Base 0.86.0

Baseline: 0.85.1; engine JSD 0.5.22 unchanged.

- Save and load managers now have a per-file DELETE / CONFIRM control. Confirmed deletion permanently removes only the selected save file. Release builds preserve the existing restriction on debug-only saves.
- Main-row and keypad 1–3 select game speed, with keyboard-state and text-event fallbacks; DEL queues the selected completed world structure in normal view and FPV cursor mode. Text editing retains its normal Delete semantics.
- Nighttime cabin lighting is prioritized on the connected electrical grid and consumes stored battery energy. Existing occupancy dimming remains: empty rooms use 18% intensity and occupied rooms use full available lighting. Disabled, disconnected or exhausted supplies do not provide light.
- Completed buildings, resource structures, corridors, utilities and walkways are dismantled by a constructor. The constructor travels to a certified exterior service point and performs the existing lander cutting action for 18 simulation seconds. The structure's rendered parts progressively disappear, then its collision geometry, links and inventory are removed using the existing cleanup/salvage logic. Incomplete construction plans still use immediate cancellation. Attached dependent links are cleaned up with the parent, not independently animated. A job waits when a protected footprint is occupied or an active trade is using the selected pad. A free charged builder is required.
- Save schema 55 adds a stable-placement-ID demolition queue with progress. Older saves load without jobs; newly saved schema55 files require this or a later game version.

Validation: Linux headless Debug build and link against the supplied JSD engine 0.5.22; targeted battery-doors, interior-services, save-editor, settings-boundary, removal-controls and builder deconstruction tests passed. A builder traveled 17.7641 m and completed cutting and removal. Actual desktop key-event delivery, visual appearance, Windows native build and Release build were not tested. No engine changes needed.
