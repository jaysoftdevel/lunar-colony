# Lunar Colony 0.84.0 / JSD 0.5.22

Full-hull mode now keeps the selected airlock's pressure skin and structural
frames visible. Explicit no-hull mode still exposes the lower boundaries and
interior. Dome selection for placing other habitats' internal equipment retains
its existing behavior.

Fixed end hatches have metal skin-to-seat adapters joining the curved pressure
hull to each flat door frame. Internal pressure partitions and their seals stay
in their original positions. Both free airlock ends have stairs during the
structure's construction, independent of camera mode or a corridor connection.
A completed docking sleeve replaces only its own stair, while the other free
end retains access. FPV walking uses the actual tread tops.

Corridor bearing pads now rest on the lunar build plane rather than on the
possibly elevated route centerline. Telescopic legs and side braces join those
pads to the real deck frame, including sloped and bent routes. Existing saved
corridors rebuild this geometry when loaded; paths and progress are retained.

Door power checks now accept a positive electrical allocation from the local
grid. A battery can run doors at night and during partial grid allocation;
charging or full industrial power allocation is not required. Disabled,
disconnected and depleted systems still require the held-E manual control.
Explicit E activation and pressure interlocks remain unchanged.

Autosave defaults to ten minutes of active real play time. Settings > Gameplay
provides interval controls (Off, or up to sixty minutes). This is a game-wide
preference, independent of each save. Game speed does not accelerate the timer;
pause and menus suspend it. New games, successful loads and interval changes
restart the timer. The three oldest-first automatic slots live in
savegames/autosaves and appear in the ordinary load list. Manual saves stay in
their own folder and are never chosen as automatic overwrite targets. Saving
uses the existing temporary-file/atomic-replacement path on Linux and Windows.
A brief on-screen notice and activity-log entry report success or failure;
failed attempts retry after one minute of active play.

Save schema54 and external JSD 0.5.22 are unchanged. This is a cumulative,
version-guarded Linux/Windows source update; apply.sh remains executable (0755).
See package VALIDATION.md for executed checks and platform limitations.
