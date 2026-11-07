#pragma once
#include <array>
#include <cstdint>

namespace eng {
enum class GraphicsPath { Auto, Deferred, Classic };
enum class GraphicsAA { Temporal, Fxaa, Off };
enum class GraphicsShadows { Paged, Cascades, Off };
enum class GraphicsTransparency { Weighted, Sorted };

enum class GraphicsFeature {
    VisibilityBuffer, GpuTerrain, StreamedGeometry, TerrainRelief, VirtualTextures,
    MaterialDetail, Decals, Hair, Cloth, Skin, LocalLights, RayTracing, RayShadows,
    RayReflections, RayOcclusion, RayIndirect, ReservoirLights, ScreenOcclusion,
    ScreenIndirect, ScreenReflections, RayDenoise, NeuralRadiance, LearnedDenoise,
    LearnedUpscale, Filmic, Vignette, Clouds, Haze, EarthAtmosphere, VolumetricPlumes,
    VolumetricDust, Pbr, Count
};

inline constexpr auto graphics_feature_keys=std::to_array<const char*>({
    "visibility_buffer","gpu_terrain","streamed_geometry","terrain_relief","virtual_textures",
    "material_detail","decals","hair","cloth","skin","local_lights","ray_tracing","ray_shadows",
    "ray_reflections","ray_occlusion","ray_indirect","reservoir_lights","screen_occlusion",
    "screen_indirect","screen_reflections","ray_denoise","neural_radiance","learned_denoise",
    "learned_upscale","filmic","vignette","clouds","haze","earth_atmosphere","volumetric_plumes",
    "volumetric_dust","pbr"
});
static_assert(graphics_feature_keys.size()==static_cast<unsigned>(GraphicsFeature::Count));
static_assert(graphics_feature_keys.size()<=32);

constexpr std::uint32_t graphics_bit(GraphicsFeature feature){
    return std::uint32_t{1}<<static_cast<unsigned>(feature);
}
inline constexpr auto all_graphics_features=static_cast<std::uint32_t>(
    (std::uint64_t{1}<<graphics_feature_keys.size())-1);

struct GraphicsSettings {
    GraphicsPath path{GraphicsPath::Auto};
    GraphicsAA aa{GraphicsAA::Temporal};
    GraphicsShadows shadows{GraphicsShadows::Paged};
    GraphicsTransparency transparency{GraphicsTransparency::Weighted};
    int render_scale{100};
    int ray_scale{50};
    int bloom{100};
    int exposure_tenths{};
    std::uint32_t features{all_graphics_features};
#ifdef _WIN32
    bool terrain_vrs{};
#endif

    constexpr bool enabled(GraphicsFeature feature) const { return (features&graphics_bit(feature))!=0; }
    void set(GraphicsFeature feature,bool enabled){
        if(enabled) features|=graphics_bit(feature);
        else features&=~graphics_bit(feature);
    }
    bool operator==(const GraphicsSettings&) const = default;
    void normalize();
};

struct GraphicsCapabilities {
    bool enhanced{};
    bool visibility_buffer{};
    bool gpu_terrain{};
    bool pages{};
    bool ray_lighting{};
    bool neural_radiance{};
#ifdef _WIN32
    bool enhanced_barriers{};
    bool variable_rate_shading{};
#endif
};

GraphicsSettings graphics_settings_from_environment();
} // namespace eng
