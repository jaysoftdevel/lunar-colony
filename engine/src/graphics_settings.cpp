#include "engine/graphics_settings.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
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
    features&=all_graphics_features;
}

GraphicsSettings graphics_settings_from_environment(){
    const auto is=[](const char* key,std::string_view value){
        const char* setting=std::getenv(key);
        return setting && setting==value;
    };
    GraphicsSettings settings;
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
