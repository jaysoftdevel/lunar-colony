# RVO2

Vendored unchanged C++ sources from https://github.com/snape/RVO2
at commit 6b4eb00a93a9f3abf723235ff006d6aad3e2f212 (Apache-2.0).
Copyright 2008 University of North Carolina at Chapel Hill.
The full license and upstream per-file notices accompany the sources.

Used for reciprocal velocity advice in the local rover planner. RVO2 never
commits a pose: acceleration-limited swept motion and braking reservations
remain authoritative in the game safety coordinator.

Path-following design also references Detour's moving path corridor:
https://github.com/recastnavigation/recastnavigation/blob/main/DetourCrowd/Source/DetourPathCorridor.cpp
No Detour source is copied or required by this build.
