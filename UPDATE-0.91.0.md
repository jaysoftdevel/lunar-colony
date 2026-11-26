# Lunar Base 0.91.0

Baseline 0.90.0; JSD 0.5.22 unchanged.

- All astronaut roles have 0.56 kg oxygen tanks, up from 0.28 kg. Nominal vacuum endurance is 16 in-game hours instead of 8. Existing saves retain actual oxygen mass, not the old percentage; refill uses finite available oxygen. AI resupply cutoff remains the same fraction of capacity.
- All eleven installable interior-unit types automatically face the interior with their backs parallel to the local hull tangent. Manual N/M angle overrides are removed for interior units only. Freestanding furniture and racks do not acquire mounting brackets. The existing fixed structural equipment remains as authored. Older saved units are rotated and, where required, moved to nearby legal bays. Units with no valid aligned location are retained unchanged rather than discarded or forced into obstructions.
- Standard/small solar peak generation is 20/10 kW instead of 16/8 kW. Tracking, darkness, electrical dispatch and inspector share the rating; battery capacities, battery charge limits and solar local buffer capacities remain unchanged.
- Schema 57 retains the schema 56 binary field layout but signals the larger supported oxygen amount and the interior alignment migration. New saves are not readable in earlier versions.

Validation: seven affected game/test translation units passed C++20 syntax checks. A compiled standalone check passed for four crew roles, eleven unit alignment flags, 360 hull normal samples, oxygen endurance, solar ratings, day/night irradiance and unchanged storage capacities. Added full-game regression assertions for actual solar generation at both sizes and night shutdown, plus tank size and alignment coverage. Full native application build, linked gameplay regressions, save migration replay and Windows runtime testing were not performed. Existing synthetic legacy save tests were not rerun.
