#include "game/config/graphics_benchmark.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
using F=GraphicsFeature;

GraphicsSettings graphics_profile_settings(BenchmarkProfile profile,const GraphicsSettings& current,const GraphicsCapabilities& caps){
    if(profile==BenchmarkProfile::Classic){
        auto settings=current;
        settings.path=GraphicsPath::Classic;
        return settings;
    }

    const bool balanced=profile==BenchmarkProfile::Balanced;
    GraphicsSettings settings=balanced?balanced_graphics_settings():GraphicsSettings{};
    settings.exposure_tenths=current.exposure_tenths;
    settings.vertical_sync=current.vertical_sync;
    if(balanced) return settings;
    const int quality=profile==BenchmarkProfile::High?3:profile==BenchmarkProfile::Balanced?2:1;
    for(const auto feature:scalable_graphics_features) settings.set_quality(feature,quality);
    settings.render_scale=quality_budget(quality,65,85,100);
    settings.ray_scale=quality_budget(quality,25,40,50);
    settings.aa_quality=settings.shadow_quality=settings.bloom_quality=quality;
    settings.bloom=quality_budget(quality,50,75,100);
    settings.particle_density=quality;
#ifndef _WIN32
    settings.surface_resolution=settings.shadow_resolution=settings.ground_clutter=quality;
    settings.clutter_range=quality_budget(quality,50,75,100);
    settings.texture_filter=quality==3 && caps.texture_anisotropy>=4?3:quality==1?0:1;
#endif
    if(quality==1) settings.aa=GraphicsAA::Fxaa;

#ifdef _WIN32
    settings.terrain_vrs=quality==1 && caps.variable_rate_shading;
#else
    if(quality==1) settings.path=GraphicsPath::Deferred;
    const bool compute=settings.path==GraphicsPath::Auto;
    settings.shadows=compute && caps.pages?GraphicsShadows::Paged:GraphicsShadows::Cascades;
    settings.set(F::VisibilityBuffer,compute && caps.visibility_buffer);
    settings.set(F::GpuTerrain,settings.enabled(F::VisibilityBuffer) && caps.gpu_terrain);
    settings.set(F::StreamedGeometry,settings.enabled(F::GpuTerrain));
    settings.set(F::VirtualTextures,compute && caps.pages);
    const bool rays=compute && caps.ray_lighting;
    for(const auto feature:{F::RayTracing,F::RayShadows,F::RayOcclusion,F::ReservoirLights,F::RayDenoise})
        settings.set(feature,rays);
    settings.set(F::RayReflections,rays && quality==3);
    settings.set(F::RayIndirect,rays && quality==3);
    settings.set(F::NeuralRadiance,rays && caps.neural_radiance && quality==3);
    settings.set(F::LearnedDenoise,rays && quality==3);
    settings.set(F::LearnedUpscale,quality>1);
    if(quality==1){
        settings.transparency=GraphicsTransparency::Sorted;
        settings.set(F::ScreenIndirect,false);
        settings.set(F::ScreenReflections,false);
    }
#endif
    return settings;
}

double BenchmarkTrial::worst_p90() const {
    double worst{};
    for(const auto& scene:scenes) worst=std::max(worst,scene.p90_ms);
    return worst;
}

void GraphicsBenchmark::start(const eng::GraphicsSettings& current,const eng::GraphicsCapabilities& caps,
                              int width,int height,int target){
    *this={};
    width_=width;
    height_=height;
    set_target_fps(target);
#ifndef _WIN32
    scene_count_=caps.enhanced?3:2;
#endif
    if(caps.enhanced){
        for(const auto profile:{BenchmarkProfile::High,BenchmarkProfile::Balanced,BenchmarkProfile::Performance})
            trials_[trial_count_++]={profile,graphics_profile_settings(profile,current,caps)};
    }
    trials_[trial_count_++]={BenchmarkProfile::Classic,graphics_profile_settings(BenchmarkProfile::Classic,current,caps)};
    status_=BenchmarkStatus::Running;
}

void GraphicsBenchmark::set_target_fps(int target){
    target_fps_=std::clamp(target,30,120);
}

void GraphicsBenchmark::record(double milliseconds,bool rendered_profile){
    if(!running()) return;
    if(!rendered_profile){
        trials_[trial_].available=false;
        scene_=scene_count_-1;
        advance();
        return;
    }
    if(!std::isfinite(milliseconds) || milliseconds<=0.0){
        if(++invalid_frames_>=3) stop(BenchmarkStatus::Failed);
        return;
    }
    invalid_frames_=0;
    if(warm_frames_<48 && (warm_frames_<4 || warm_ms_<300.0)){
        ++warm_frames_;
        warm_ms_+=milliseconds;
        return;
    }

    samples_[sample_count_++]=milliseconds;
    sample_ms_+=milliseconds;
    if(sample_count_<samples_.size() && (sample_count_<8 || sample_ms_<450.0) && sample_ms_<2500.0) return;
    std::sort(samples_.begin(),samples_.begin()+sample_count_);
    const auto p90=(sample_count_*9+9)/10-1;
    trials_[trial_].scenes[scene_]={sample_ms_/sample_count_,samples_[p90]};
    advance();
}

void GraphicsBenchmark::advance(){
    if(++scene_>=scene_count_){
        scene_=0;
        if(++trial_>=trial_count_) status_=BenchmarkStatus::Complete;
    }
    warm_frames_=invalid_frames_=0;
    warm_ms_=sample_ms_=0.0;
    sample_count_=0;
}

void GraphicsBenchmark::stop(BenchmarkStatus reason){
    if(running()) status_=reason;
}

const BenchmarkTrial* GraphicsBenchmark::recommendation() const {
    if(status_!=BenchmarkStatus::Complete) return nullptr;
    const BenchmarkTrial* fastest=nullptr;
    for(const auto& trial:trials()){
        if(!trial.available || trial.worst_p90()<=0.0) continue;
        if(trial.worst_p90()<=1000.0/target_fps_) return &trial;
        if(!fastest || trial.worst_p90()<fastest->worst_p90()) fastest=&trial;
    }
    return fastest;
}

float GraphicsBenchmark::progress() const {
    if(status_==BenchmarkStatus::Complete) return 1.0f;
    if(!running()) return 0.0f;
    const float pass=sample_count_?0.35f+0.65f*float(sample_count_)/samples_.size():
        0.35f*std::min(1.0f,std::max(float(warm_ms_/300.0),float(warm_frames_)/48.0f));
    return (float(trial_*scene_count_+scene_)+pass)/float(trial_count_*scene_count_);
}

} // namespace game
