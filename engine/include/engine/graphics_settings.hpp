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

inline constexpr std::array scalable_graphics_features{
    GraphicsFeature::TerrainRelief,GraphicsFeature::StreamedGeometry,GraphicsFeature::VirtualTextures,
    GraphicsFeature::MaterialDetail,GraphicsFeature::Decals,GraphicsFeature::Hair,GraphicsFeature::Cloth,
    GraphicsFeature::Skin,GraphicsFeature::LocalLights,GraphicsFeature::RayShadows,GraphicsFeature::RayReflections,
    GraphicsFeature::RayOcclusion,GraphicsFeature::RayIndirect,GraphicsFeature::ReservoirLights,
    GraphicsFeature::ScreenOcclusion,GraphicsFeature::ScreenIndirect,GraphicsFeature::ScreenReflections,
    GraphicsFeature::RayDenoise,GraphicsFeature::Vignette,GraphicsFeature::Clouds,GraphicsFeature::Haze,
    GraphicsFeature::VolumetricPlumes,GraphicsFeature::VolumetricDust
};
inline constexpr auto scalable_graphics_mask=[]{
    std::uint32_t mask{};
    for(auto feature:scalable_graphics_features) mask|=graphics_bit(feature);
    return mask;
}();
constexpr bool scalable_graphics_feature(GraphicsFeature feature){
    return feature<GraphicsFeature::Count && (scalable_graphics_mask&graphics_bit(feature))!=0;
}

template<class T> constexpr T quality_budget(int quality,T low,T medium,T high){
    return quality<=1?low:quality==2?medium:high;
}

struct GraphicsSettings {
    GraphicsPath path{GraphicsPath::Auto};
    GraphicsAA aa{GraphicsAA::Temporal};
    GraphicsShadows shadows{GraphicsShadows::Paged};
    GraphicsTransparency transparency{GraphicsTransparency::Weighted};
    int render_scale{100};
    int ray_scale{50};
    int bloom{100};
    int exposure_tenths{};
    int aa_quality{3};
    int shadow_quality{3};
    int bloom_quality{3};
    int surface_resolution{3}; // 128, 256 or 512 pixels at the base mip
    int texture_filter{1}; // bilinear mip, trilinear, then 2x through 16x anisotropy
    int shadow_resolution{3};
    int ground_clutter{3};
    int clutter_range{100};
    int particle_density{3};
    bool vertical_sync{};
    std::uint32_t features{all_graphics_features};
    std::uint32_t low_quality{},medium_quality{};
#ifdef _WIN32
    bool terrain_vrs{};
#endif

    constexpr bool enabled(GraphicsFeature feature) const { return (features&graphics_bit(feature))!=0; }
    void set(GraphicsFeature feature,bool enabled){
        if(enabled) features|=graphics_bit(feature);
        else features&=~graphics_bit(feature);
    }
    constexpr int quality(GraphicsFeature feature) const {
        const auto bit=graphics_bit(feature);
        return !(features&bit)?0:(low_quality&bit)?1:(medium_quality&bit)?2:3;
    }
    void set_quality(GraphicsFeature feature,int level){
        const auto bit=graphics_bit(feature);
        set(feature,level>0);
        low_quality=(low_quality&~bit)|(level==1?bit:0);
        medium_quality=(medium_quality&~bit)|(level==2?bit:0);
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
    bool vertical_sync{};
    int texture_anisotropy{1};
};

constexpr int surface_resolution_pixels(int level){ return quality_budget(level,128,256,512); }
constexpr int cascade_resolution_pixels(int level){ return quality_budget(level,768,1024,1536); }
constexpr int texture_anisotropy(int filter){ return filter<2?1:1<<(filter-1); }

GraphicsSettings graphics_settings_from_environment();
} // namespace eng
