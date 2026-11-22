# Current baseline — Lunar Colony 0.61.1

Game source: game/. External engine: JSD source 0.5.12, selected with
`./build.sh /path/to/3d-engine` or `build.bat --engine PATH`.
This update applies to game 0.61.0 plus external engine 0.5.11.
Both source roots are updated together; no engine is copied into the game.

Materials, station interiors and FOV from 0.61.0 are retained. Windows fixes
cover toolchain selection, runtime DLL staging, filesystem paths and asset-cache
portability. See WINDOWS-COMPATIBILITY.md and README-WINDOWS.md.
