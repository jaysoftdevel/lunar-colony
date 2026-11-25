# Lunar Base 0.85.1 — validated follow-up

Compatible with game 0.84.0 or 0.85.0 and unchanged JSD engine source 0.5.22. This is a validation/coverage revision of the 0.85.0 gameplay changes, not an engine update. The two accepted game baselines are independently hash-checked. User saves and engine files are not replaced.

## Native validation (0.85.0 gameplay sources before version-only bump)

- CMake/Ninja Debug, GNU C++20, bundled SDL2 and Jolt from the supplied JSD 0.5.22 source: all 608 build steps completed successfully. Headless mode used because this container lacks normal desktop development headers.
- 15/15 targeted CTest tests passed, including real offscreen OpenGL renderer integration, pressure shell, suit oxygen, FPV controls, hull cutaway and interior surfaces. The added L-key regression passed as part of game_astronaut_fpv_continuity.
- Exact original schema-54 `latest.gmsave` loaded successfully. Added regression asserts a continuous first corridor deck and endpoint alignment, then runs the real robot controller until both saved carriers reach the starter charging bays and finish charging. Both completed; carrier 1 15.3302 -> 60.8034 kWh, carrier 2 16.1184 -> 60.7915 kWh (80% of 76 kWh, rounding/idle draw). Maximum displacement from original positions 63.3122 m and 52.5323 m, no final safety blocker. This is a simulation test, not an interactive play session.
- Actual save rendered for 12 frames using the real JSD renderer, Mesa llvmpipe OpenGL 4.5, offscreen. No runtime rendering error. No physical-GPU visual signoff.
- Broader CTest run was not completed. One runtime-path test failed in an external build folder; it passed when the same test executable was placed under the game workspace (normal layout), including Unicode save roundtrip.
- No Windows native build or Linux Release build performed. Windows build-driver mock tests passed. New revision changes only tests, docs and version metadata relative to 0.85.0; no functional code changes.

## Showcase folder

The engine's `projects/sundog/sundog.jsdproject` and `sundog.assets/` contain the editable SUNDOG scene/resources. Keep or archive them as a pair. Native `run-showcase.sh` without project arguments runs the C++ showcase and does not require the exported project; `run-project.sh`, the editor and project-player regression tests do. Regeneration from native sources cannot be assumed to preserve editor-only changes.
