#pragma once
#include "game/runtime/presentation.hpp"
#include <frontier/renderer.hpp>
#include <stdexcept>

namespace game::runtime::jsd_bridge {
inline rf::V3 vector(Vec3 p){return {p.x,p.y,p.z};}
inline rf::Color color(Color c){return {c.r,c.g,c.b,c.a};}

struct View {
    rf::gfx::Camera camera;
    // Correct the engine's fixed world-up basis at the orbit camera's poles.
    rf::Mat4 correction=rf::Mat4::identity();
};
inline View decode_view(const Mat4& vp){
    const auto& m=vp.m;
    const Vec3 row0{m[0],m[4],m[8]}, row1{m[1],m[5],m[9]}, row3{m[3],m[7],m[11]};
    const float fx=length(row0),fy=length(row1);
    if(fx<1e-6f || fy<1e-6f || length(row3)<.5f)
        throw std::invalid_argument("JSD bridge expects a perspective view-projection matrix");
    const auto right=row0*(1/fx),up=row1*(1/fy),back=normalize(row3);
    const auto eye=right*(-m[12]/fx)+up*(-m[13]/fy)+back*(-m[15]);
    const Vec3 row2{m[2],m[6],m[10]};
    const float a=-dot(row2,back),b=m[14]+a*m[15];
    View result;
    result.camera.position=vector(eye);
    result.camera.direction=vector(back);
    result.camera.fovDegrees=2*std::atan(1/fy)*180/rf::pi;
    result.camera.nearPlane=std::max(.001f,b/(a-1));
    rf::Mat4 original=rf::Mat4::identity();
    for(int c=0;c<3;++c){
        original.m[c*4]=m[c*4]/fx;
        original.m[c*4+1]=m[c*4+1]/fy;
        original.m[c*4+2]=-m[c*4+3];
    }
    original.m[12]=m[12]/fx;original.m[13]=m[13]/fy;original.m[14]=-m[15];
    const auto canonical=rf::lookAt(result.camera.position,result.camera.position+result.camera.direction);
    // Suppress subpixel numerical noise in the usual world-up case.
    float difference=0;
    for(int i=0;i<16;++i)difference=std::max(difference,std::abs(original.m[i]-canonical.m[i]));
    if(difference>.0005f)result.correction=rf::inverse(canonical)*original;
    return result;
}
inline rf::gfx::GraphicsSettings settings(GraphicsSettings s){
    using namespace rf::gfx;
    s.normalize();
    auto out=rf::gfx::GraphicsSettings::balanced();
    const auto quality=[](int q){return q<=0?Quality::Off:q==1?Quality::Low:q==2?Quality::Medium:Quality::High;};
    out.antiAliasing=s.aa==GraphicsAA::Off?AntiAliasing::Off:s.aa==GraphicsAA::Fxaa?AntiAliasing::FXAA:AntiAliasing::TAA;
    out.temporalWeight=.80f;
    out.renderScale=s.render_scale/100.f;
    out.sharpening=.16f;
    out.shadows=s.shadows==GraphicsShadows::Off?Quality::Off:quality(s.shadow_quality);
    out.ambientOcclusion=quality(s.quality(GraphicsFeature::ScreenOcclusion));
    out.globalIllumination=s.enabled(GraphicsFeature::ScreenIndirect)?GlobalIllumination::ScreenSpace:GlobalIllumination::AmbientProbe;
    out.illuminationQuality=Quality::Medium;
    out.reflections=quality(s.quality(GraphicsFeature::ScreenReflections));
    out.bloom=s.bloom?quality(s.bloom_quality):Quality::Off;
    out.bloomStrength=.07f*s.bloom/100.f;
    out.exposureCompensation=s.exposure_tenths/10.f;
    out.autoExposure=false;
    out.toneMapper=s.enabled(GraphicsFeature::Filmic)?ToneMapper::ACES:ToneMapper::Neutral;
    out.vignette=s.enabled(GraphicsFeature::Vignette)?.08f:0;
    out.textureQuality=s.surface_resolution<=1?TextureQuality::Low:s.surface_resolution==2?TextureQuality::Medium:TextureQuality::High;
    out.anisotropy=texture_anisotropy(s.texture_filter);
    out.depthOfField=Quality::Off;out.motionBlur=Quality::Off;
    out.volumetricFog=Quality::Off;out.fogDensity=0;
    out.terrainTessellation=Quality::Off;
    out.viewDistance=5000;
    out.gpuProfiling=true;
    if(s.path==GraphicsPath::Classic){
        out.globalIllumination=GlobalIllumination::AmbientProbe;
        out.ambientOcclusion=Quality::Off;out.reflections=Quality::Off;out.shadows=Quality::Off;
    }
    return out;
}
std::unique_ptr<SceneRenderer> create_renderer(rf::Platform& platform, std::function<void(const rf::gfx::ShaderBuildProgress&)> progress = {});
}
