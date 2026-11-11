#include "engine/graphics_settings.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>

namespace eng {
void GraphicsSettings::normalize(){
    path=static_cast<GraphicsPath>(std::clamp(int(path),0,2));
    aa=static_cast<GraphicsAA>(std::clamp(int(aa),0,2));
    shadows=static_cast<GraphicsShadows>(std::clamp(int(shadows),0,2));
    transparency=static_cast<GraphicsTransparency>(std::clamp(int(transparency),0,1));
    render_scale=std::clamp(render_scale,50,100);
    ray_scale=std::clamp(ray_scale,25,75);
    bloom=std::clamp(bloom,0,150);
    exposure_tenths=std::clamp(exposure_tenths,-20,20);
    aa_quality=std::clamp(aa_quality,1,3);
    shadow_quality=std::clamp(shadow_quality,1,3);
    bloom_quality=std::clamp(bloom_quality,1,3);
    surface_resolution=std::clamp(surface_resolution,1,3);
    texture_filter=std::clamp(texture_filter,0,5);
    shadow_resolution=std::clamp(shadow_resolution,1,3);
    ground_clutter=std::clamp(ground_clutter,0,3);
    clutter_range=std::clamp(clutter_range,25,100);
    particle_density=std::clamp(particle_density,0,3);
    features&=all_graphics_features;
    low_quality&=scalable_graphics_mask;
    medium_quality&=scalable_graphics_mask&~low_quality;
}

GraphicsSettings balanced_graphics_settings(){
    using F=GraphicsFeature;
    GraphicsSettings settings;
    settings.render_scale=85;
    settings.ray_scale=40;
    settings.bloom=75;
    settings.aa_quality=settings.shadow_quality=settings.bloom_quality=2;
    settings.surface_resolution=settings.shadow_resolution=settings.ground_clutter=settings.particle_density=2;
    settings.clutter_range=75;
    settings.shadows=GraphicsShadows::Cascades;
    for(const auto feature:scalable_graphics_features) settings.set_quality(feature,2);
    settings.set_quality(F::ScreenOcclusion,1);
    for(const auto feature:{F::VisibilityBuffer,F::GpuTerrain,F::StreamedGeometry,F::VirtualTextures,
                            F::RayTracing,F::RayShadows,F::RayReflections,F::RayOcclusion,F::RayIndirect,
                            F::ReservoirLights,F::RayDenoise,F::NeuralRadiance,F::LearnedDenoise,
                            F::LearnedUpscale,F::ScreenIndirect,F::ScreenReflections})
        settings.set(feature,false);
    return settings;
}

GraphicsSettings graphics_settings_from_environment(){
    const auto is=[](const char* key,std::string_view value){
        const char* setting=std::getenv(key);
        return setting && setting==value;
    };
    auto settings=balanced_graphics_settings();
    if(const char* value=std::getenv("GAME_VSYNC")){
        std::string sync(value);
        std::transform(sync.begin(),sync.end(),sync.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
        settings.vertical_sync=sync=="1" || sync=="true" || sync=="on";
    }
    if(is("GAME_LUNAR_EXPERIMENT","0")) settings.path=GraphicsPath::Classic;
    else if(is("GAME_RENDER_PATH","deferred")) settings.path=GraphicsPath::Deferred;
    if(is("GAME_LUNAR_TAA","0")) settings.aa=GraphicsAA::Fxaa;
    if(is("GAME_NEURAL_LIGHTING","0")){
        settings.set(GraphicsFeature::NeuralRadiance,false);
        settings.set(GraphicsFeature::LearnedDenoise,false);
    }
    if(is("GAME_NEURAL_UPSCALE","0")) settings.set(GraphicsFeature::LearnedUpscale,false);
    if(const char* value=std::getenv("GAME_RENDER_SCALE")){
        char* end{};
        const float scale=std::strtof(value,&end);
        if(end!=value && !*end && std::isfinite(scale))
            settings.render_scale=int(std::lround(std::clamp(scale,0.5f,1.0f)*100));
    }
    return settings;
}
} // namespace eng
