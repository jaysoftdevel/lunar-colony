# Rendering in v0.52.6

## Windows

The Direct3D 12 lunar pass renders physically based materials, procedural
regolith and crater detail, and powered fixtures into an FP16 scene buffer.
A reduced-resolution bloom pass and an SDR tone-mapping pass apply exposure
and optional FXAA before the game draws its HUD. The buildable floor stays flat.

The Output tab offers Enhanced and Classic views, lunar render scale, FXAA
mode and quality, and terrain variable-rate shading. VRS applies 2×2 shading
beyond the playfield on supported adapters and defaults to off. World controls
material detail, terrain detail, and decals. Lighting controls PBR and fixture
count. Image controls bloom resolution and intensity, exposure, the tone curve,
and vignette. The HUD draws at display resolution.

The Windows settings file is `%APPDATA%/Lunar Frontier/graphics.cfg`.
Direct3D code and Windows-only settings are compiled into the Windows targets.
The build can stage the stable Agility SDK 1.620.1 runtime for both executables.

## Linux

The Linux renderer selects the compute path on desktop OpenGL 4.3 or newer.
OpenGL 3.3 uses deferred raster lighting, screen-space effects, and the same
world materials and UI. The enhanced lunar renderer has an older mesh-view
fallback if its required render targets or shaders are unavailable.

The Earth campus uses a scattering sky, distance haze, and an animated
ray-marched cloud layer. Lunar stages use an airless sky and localized exhaust
and regolith dust. The render surface around the colony has procedural relief.
The buildable playfield uses a flat placement plane with shaded crater relief.

## Techniques

| Technique | Implementation |
| --- | --- |
| PBR | GGX material lighting, physically attenuated fixtures, mipmapped relief and roughness, material layering, HDR emission, and filmic tonemapping |
| Deferred and visibility-buffer shading | The compute path first records opaque triangle IDs. Material resolve fetches the triangle data and reconstructs the G-buffer from depth. The compatibility path writes the G-buffer directly. |
| Virtualized geometry | The terrain surface uses frustum-culled clusters. Boulders have coarse resident meshes and fine geometry streamed through a bounded page pool. |
| Real-time ray tracing | Compute shaders trace nearby scene triangles for reflections, sun and fixture shadows, contact AO, and one-bounce diffuse lighting. Raster shadows cover the far field and snapshot warmup. |
| Software ray tracing | A worker-built triangle BVH supplies scene intersections. Lighting uses stochastic hemisphere samples and actual off-screen geometry. |
| ReSTIR | The direct-light pass reuses candidates across time and nearby pixels with target-density correction, visibility testing, and bounded reservoir weights. It evaluates up to 64 fixtures. |
| Volumetrics | The Earth campus has volumetric clouds. The lander and trader render depth-clipped plume volumes. A low lander adds an expanding regolith dust volume. |
| Shadow maps | Three stabilized sun cascades and a near shadow page cache. Moving caster changes invalidate the affected pages. |
| Screen-space effects | Contact SSAO, SSR, and single-bounce screen-space diffuse lighting on the raster path. Skin uses geometry-aware, per-channel screen-space diffusion. |
| Temporal AA and upscaling | Jittered temporal reconstruction at the history resolution, disocclusion rejection, FXAA edge cleanup, and an optional learned luminance residual at reduced render scales. |
| GPU-driven rendering | Compute frustum culling and detail selection write indirect terrain draw commands. |
| Hair, cloth, and skin | Anisotropic highlights and strand shading on the coverall hair mesh, fabric relief and sheen, animated coverall wrinkles, and skin diffusion. |
| Decals and virtual texturing | Worn landing-pad paint overlays, dust on low cladding, and paged regolith color, slope, and roughness. |
| Order-independent transparency | Weighted color accumulation and optical-depth resolve for glass and translucent cutaways. |
| Neural radiance caching and denoising | A small online neural model learns a diffuse-lighting prior from ray samples. An offline-trained confidence model guides geometry-aware temporal filtering. |

Ray tracing uses ordinary GPU compute instructions. Dedicated Vulkan or DXR
ray-tracing pipelines, native mesh-shader stages, and the DLSS, FSR, and XeSS
SDKs require separate backend integrations. The resident detail geometry
system currently covers the lunar terrain and boulder field.

## Runtime controls

Open **Settings → Graphics** from the title or pause menu. Settings take
effect when rendering resumes and are saved for future sessions. **Reset
Defaults** restores the release defaults. Controls explain dependencies and
are dimmed when the selected renderer or the GPU cannot run them.

Scalable effects have their own Off, Low, Medium, and High control. Resolution
and image adjustments keep their separate percentage or exposure controls.
Rendering techniques such as PBR, visibility buffers, and transparency retain
their mode or enable controls. Existing on/off preferences are read as High
or Off.

| Linux tab | Controls |
| --- | --- |
| Output | Auto, Deferred, or Classic renderer, 3D render scale, AA mode and spatial sample quality, learned upscaling, and visibility buffer |
| World | Terrain relief, GPU culling, streamed geometry, virtual textures, material detail, decals, hair, cloth, skin, and transparency |
| Lighting | PBR, fixture count, sun shadow mode and filtering, ReSTIR, and separate screen-space AO, GI, and reflection workloads |
| Ray Tracing | Software tracing, individual shadow, reflection, AO, and GI reach, compute lighting resolution, denoising, and the neural cache |
| Image | Bloom resolution and intensity, exposure, tone mapping, and vignette |
| Atmosphere | Earth sky, clouds, distance haze, exhaust, and lunar dust |

**Settings → Debug Layout** switches between the configured renderer and a
mesh view that bypasses the lunar effects and campus material textures.
The in-game debug frame uses the same switch.

## Hardware benchmark

**Graphics → Benchmark** shows the adapter and driver used by the active
renderer. Choose a 30, 60, 90, or 120 FPS target and run the suite. The benchmark
warms up each supported profile, then times a fixed lunar orbit view and a
surface view. Linux also tests the Earth campus when enhanced rendering is
available. The results show per-scene render throughput and the worst 90th
percentile frame time. Each timed frame waits for rendering completion before
presentation. The test excludes simulation and uses the current display size.

The game recommends the highest tested quality within the target frame budget,
or the fastest tested profile when the target is out of reach. **Apply
Recommendation** saves it. Escape cancels a run and a display-size change
requires new measurements. The benchmark runs its own scene and leaves the
current colony and saved settings available while you review the results.

## Preferences

The Linux preferences file is `$XDG_CONFIG_HOME/lunar-frontier/graphics.cfg`
or `~/.config/lunar-frontier/graphics.cfg` when XDG_CONFIG_HOME is unset.
It stores the graphics, debug layout, and music preferences.
`GAME_GRAPHICS_CONFIG` can select another path. The previous graphics
environment variables provide initial values for unset preferences.
`GAME_AUDIO=0` still starts the game muted.

The BVH refreshes asynchronously. It covers a region around the camera focus,
with shorter traces for the individual lighting effects. A newly loaded world
can use raster lighting while the first snapshot builds. The renderer limits
near-shadow page updates and uses cascades for missing pages. Neural models
use conservative weights and their training scope is specific to this project.
Python is needed only to regenerate the embedded offline models in `scripts/`.

## References

- OpenGL shader storage buffers. https://wikis.khronos.org/opengl/Shader_Storage_Buffer_Object
- Virtualized geometry reference. https://dev.epicgames.com/documentation/en-us/unreal-engine/nanite-virtualized-geometry-in-unreal-engine
- ReSTIR direct-light resampling. https://research.nvidia.com/index.php/publication/2020-07_spatiotemporal-reservoir-resampling-real-time-ray-tracing-dynamic-direct
- Spatiotemporal filtering reference. https://research.nvidia.com/labs/rtr/publication/schied2017spatiotemporal/
- Vulkan hardware ray tracing. https://docs.vulkan.org/guide/latest/extensions/ray_tracing.html
- Vulkan mesh shaders. https://docs.vulkan.org/refpages/latest/refpages/source/VK_EXT_mesh_shader.html
- Vendor upscalers. https://developer.nvidia.com/rtx/dlss, https://gpuopen.com/fidelityfx-super-resolution-3/, https://github.com/intel/xess
