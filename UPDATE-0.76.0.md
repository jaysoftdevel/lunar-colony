# Lunar Colony 0.76.0

Requires external JSD engine 0.5.21. Shared C++20/SDL2 code and source build
scripts support Linux and Windows 11. This update keeps fullscreen startup,
loading splash, game-wide preferences, and progress-only schema52 saves.

## Airlock passage

Removed duplicate hull caps and inset liner faces where the entrance/exit
cabins meet the pressure chamber. Clearances are cut after size scaling, for
both small and standard airlocks and arbitrary rotations. Structural frames,
real moving pressure doors, closed-door collision and pressure interlocks remain.
The extra surfaces were visual obstacles distinct from the actual door seals.

## Supplied save: FPV recovery

The attached latest.gmsave (game 0.74.0, schema51) has a deceased FPV astronaut:
health is 0%. At the stored location, collision clearance and movement directions
are clear. The health check prevented control, but the failed FPV entry gave no
explanation. The original save is not edited.

Press V after loading to show the selected astronaut's status. It now explicitly
says DECEASED / CANNOT MOVE. Choose living crew, or click NEW EXPLORER to dispatch
a fresh astronaut at the nearest safe lander-side spawn. This explicit action
uses an unused crew slot, preserves the deceased astronaut and colony, and needs
a lander depot. It is disabled when all four crew slots are occupied or in the
crew test scenario. It does not resurrect or relocate the casualty.

Without an explicit crew selection, FPV prefers another living astronaut if
one exists. Death during FPV exits control and displays the status panel.
The build-order panel moves beside selected crew, leaving the recovery controls
accessible. Other world selections are cleared when the deceased panel opens.

## Save dialog

The name field has a blinking caret, mouse placement and drag selection,
left/right and Home/End navigation, Shift selection, Ctrl+word navigation,
Backspace/Delete, Insert overwrite mode, Ctrl+A/C/X/V, Ctrl+Z/Y and Ctrl+Shift+Z.
Long names scroll horizontally. The existing 64-byte name limit is enforced
without cutting UTF-8 characters. The built-in font displays unsupported
characters as one placeholder each; clipboard and saved text retain the UTF-8.
This is a plain-text name editor; filenames do not carry rich-text markup.

A scrollable sidebar shows existing saves and their game versions. Click a save
to use its name, or Tab to the list and use Up/Down and Enter. Enter from the list
chooses the name without writing. Matching names show an explicit OVERWRITE
button. Release still protects Debug-only saves. Save success/error feedback and
the refreshed sidebar stay visible.

## Validation

See the package VALIDATION.md and validation-logs. Regression coverage includes
the supplied save, collision and visible airlock clearances, real door seals,
text editing and responsive layouts. Source update guards reject unsupported or
modified installations before writing. Native Windows compilation was unavailable;
the portable Windows build driver and shared sources are included and checked.
