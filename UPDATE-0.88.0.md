# Lunar Base 0.88.0

Baseline: 0.87.0. External JSD engine 0.5.22 unchanged. Save schema remains 55.

- Visible procedural Earth with a stylized 24-game-hour orbit, solar shading and clouds. Its sky position follows the colony clock, not real lunar ephemerides.
- Level main corridors between buildings, aligned branch heights, smoothed terminal leads, continuous cutaway docking floors and supports. Steep adapters use steps with nominal rises of at most 0.12 m; collision and tread elevations share the same calculation.
- Storage dome loading-gate roof and side collar close the gap to its curved hull.
- Larger outlined green crosshair with amber center in FPV; larger native colored cursor.
- Rectangular pressure rooms for all ten crew habitat types, including airlock, with flat roofs, revised structural frames, doors, equipment placement and saved connection realignment. The outdoor storage dome remains round.

## Validation
Linux Debug native build against JSD 0.5.22 (headless configuration). 17 distinct targeted regressions passed, including software OpenGL sky rendering, pressure shells, corridor floor coverage in both hull modes, airlock access, saved-carrier recovery, room placement and interior services. Earth regression counted 707 and 576 blue pixels when aimed at the globe at hours 6 and 18, and 24 when it moved out of view. Windows and hardware GPU rendering were not tested. The current screenshots were used as references; their exact savegame was not supplied. Older save fixture tested; back up valuable saves before opening with new room geometry.
