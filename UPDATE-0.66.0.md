# Lunar Colony 0.66.0 / JSD 0.5.16

Apply to game 0.65.0 and separate engine source 0.5.15, then rebuild Release and
Debug. Saves, preferences, graphics quality and save schema 48 remain compatible.

The desktop window is created in fullscreen immediately. A small independent
loading splash appears before the main renderer, engine shaders or colony game
assets are created. It shows graphics preparation, cached/compiled shader counts,
asset preparation and first-frame readiness. The first completed game frame
replaces it. Startup polls window events between shader jobs; Escape or closing
the loading window cancels cleanly.

On first use the engine compiles its shader programs and, when the driver supports
reusable program binaries, writes a local cache. Later launches restore those
binaries. Cache identity includes the complete shader sources and stages, engine
version, GPU vendor/device, driver/OpenGL/GLSL versions and supported binary
formats. Driver/engine/source changes rebuild only the appropriate entries.
Corrupt, truncated, rejected, unavailable or unwritable cache data falls back to
normal source compilation, without preventing play.

Windows default: `%LOCALAPPDATA%\JSD\shader-cache`.
Linux default: `$XDG_CACHE_HOME/jsd/shader-cache`, otherwise
`$HOME/.cache/jsd/shader-cache`. These are generated local files and are not shipped
inside the source update. `JSD_SHADER_CACHE_DIR` overrides the cache location;
`JSD_SHADER_CACHE=0` disables this additional cache. A driver namespace is bounded
to 256 entries / 256 MiB and an individual binary to 64 MiB.

The cache reduces repeated shader compilation/link work during loading; it does
not cache texture pixels, terrain geometry or saves and does not reduce the
per-pixel execution cost of shaders. The procedural material/texture shaders
benefit from compiled-program reuse without changing their appearance. Small
bootstrap/UI shaders remain fast source-compiled so the splash can appear before
the main shader system exists. No artificial loading delay is added.

The implementation is shared by Linux and Windows. See VALIDATION.md for the
measured cold/warm behavior and the tested startup/corruption/fallback paths.
