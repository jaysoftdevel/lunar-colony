# Lunar Colony 0.75.0 — fullscreen startup and game-wide settings

External JSD remains **0.5.21**. Linux and Windows use the same runtime policy.

- Fullscreen is the game window default and is verified before the loading
  splash/assets. Preferences are applied while loading; the first ready frame
  uses the resulting display dimensions. Exclusive fullscreen after a resolution
  change is recognized correctly, alongside borderless desktop fullscreen.
- Every launch starts fullscreen. The settings-menu windowed toggle remains
  available for the current session; loading a save preserves that current mode.
- Game-wide preferences now include graphics, UI scale, hull visibility, sound
  volume/mute, music, FPS counter, FPV field of view, debug layout/frame and chosen
  display resolution. Changes persist immediately from their controls. New Game
  and Load/Continue do not overwrite them with a colony's preferences.
- New save schema **52** removes display, fullscreen, UI scale, hull visibility
  and build-tool settings from the file. It stores colony/mission progress,
  simulation state, entities, inventories, orders and the world/astronaut view
  positions needed to resume. Simulation speed/pause and campaign/debug mode
  remain world session state. Build-tool selections are cancelled when loading.
- Schemas 1–51 still load through the existing compatibility readers. Their old
  settings fields are consumed and discarded, without applying them or writing
  them into the settings file. Compact representation threshold remains 48.

The game-wide settings file is per user, shared by all saves and both binaries:

| Platform | File |
| --- | --- |
| Linux | $XDG_CONFIG_HOME/lunar-frontier/settings.cfg, or ~/.config/lunar-frontier/settings.cfg |
| Windows | %APPDATA%\Lunar Frontier\settings.cfg |

If settings.cfg does not yet exist, the previous graphics.cfg is read as a
fallback. The next settings change writes settings.cfg; graphics.cfg is left
intact. GAME_SETTINGS_CONFIG selects an explicit file. GAME_GRAPHICS_CONFIG is
retained as a compatible older override when the newer variable is not set.
Unsupported saved display modes fall back to the current desktop mode.

Apply to the game and separate engine source folders, then rebuild. The update
contains shared C++20 source and Linux/Windows launchers, not compiled binaries.
