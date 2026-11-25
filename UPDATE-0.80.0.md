# Lunar Colony 0.80.0 — Generator sizes, not dome sizes

Corrects the oxygen sizing interpretation in 0.79.0. Requires external JSD
engine 0.5.21. Linux and Windows use the same sources and update package.

## Oxygen generator choices

Select an oxygen dome, then find OXYGEN GENERATOR in its internal-unit panel.
Choose **VERY SMALL**, **SMALL** or **MEDIUM**, then use PLACE and click a legal
deck position. The preview, footprint, solid collision, construction cost,
screen attachment and saved unit all use the selected machine size. Normal
hull alignment and N/M rotation offsets still apply.

| Generator | Width × height × depth | Oxygen / day | Water / day | Power |
| --- | --- | --- | --- | --- |
| Very Small | 1.65 × 1.49 × 0.77 m | 4.5 kg | 5.0625 L | 1.5 kW |
| Small | 2.25 × 2.03 × 1.05 m | 9 kg | 10.125 L | 3 kW |
| Medium | 3.00 × 2.70 × 1.40 m | 18 kg | 20.25 L | 6 kW |

Water consumption and hydrogen coproduct retain the same electrolysis mass
balance. Output and electrical demand scale with the selected capacity.
Construction costs scale to match, rounding fractional material quantities
upward. Disabled or doorway-reserved units do not produce oxygen.

## Dome correction and saves

The oxygen dome again offers its original Small, Standard, Mid and Big
building sizes. Small returns to its former 0.76 horizontal scale, and
Standard is no longer relabeled Medium. The added Very Small building choice
is removed. A Very Small dome from a 0.79 save maps to the restored Small
building tier. The general domed habitat design, structural cutaway and
other 0.79 improvements remain.

All three generator sizes fit the restored Small dome. Its equipment bay
search retains a clear two-metre circulation core and reserved gate
approaches. Installed machines are realigned where a legal bay is available;
their inventory and production state are retained. The original supplied
save is not overwritten.

New saves use schema54 to store size per installed unit. Older saves load
their generators as Medium, retaining the original full-size dimensions and
18 kg/day rating. Other interior machines are unchanged. Existing generators
are not silently resized or downgraded. Game-wide preferences remain outside
progress saves. New schema54 saves require 0.80.0 or later.

See the package README for application/build commands and VALIDATION.md for
executed checks and platform limits. Native Windows compilation was not
available; portable Windows build-driver and version/launcher checks are
included.
