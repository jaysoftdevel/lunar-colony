# Lunar Colony 0.77.0 — First-person vital alerts

Requires external JSD engine 0.5.21. Linux and Windows share the same C++20/SDL2
rendering and build sources. Save schema52 and global settings remain unchanged;
the warnings are derived live from the controlled astronaut's current state.

In first-person mode, any reserve strictly below 20% triggers a flashing red
border around all four screen edges. A separate visible icon and bold red
caption identifies every active problem:

| Reserve | Caption / symbol | Trigger in the existing simulation |
| --- | --- | --- |
| Health | LOW HEALTH / medical cross | Health below 20% |
| Oxygen | LOW OXYGEN / lungs | Suit oxygen below 20% of capacity when suited; cabin breathability below 20% when unsuited |
| Food | LOW FOOD / cutlery | Hunger above 80% |
| Water | LOW WATER / droplet | Thirst above 80% |
| Rest | LOW REST / bed | Less than 20% remaining before the existing exhaustion limit |

Hunger, thirst and sleep need increase as the astronaut deteriorates. They are
converted to remaining reserves for alert decisions, so a healthy astronaut
with low hunger/thirst does not trigger an emergency. Exactly 20% does not
trigger; recovery to 20% or higher removes that problem's warning immediately.
An unsuited astronaut in vacuum has no breathable air and triggers LOW OXYGEN.
A healthy cabin does not warn about an unused empty suit.

All simultaneous problems are shown in a compact lower-left grid above the
bottom configuration/build bar, beside the crew panel and clear of the reticle.
Each icon has a distinct silhouette. Red captions remain visible continuously;
only the border pulses smoothly once per second. The animation uses UI time
and continues while simulation time is paused. It is drawn above the pause
frame and other HUD elements. The warning clears when all reserves recover or
FPV is exited, and follows the astronaut under direct control when crew changes.
Menus and the top-down view do not display these FPV alerts.

The overlay uses the existing retained UI renderer, procedural icons and font
atlas, with no new textures, shaders, per-frame asset loading or platform APIs.

See package VALIDATION.md for regression coverage and platform limits.
