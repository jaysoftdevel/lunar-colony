#include "game/game_app.hpp"
#include "engine/ui_metrics.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace game {
using namespace eng;
namespace {
using F=GraphicsFeature;

enum class Control {
    Feature, Path, Scale, RayScale, AA, Shadows, Transparency, Bloom, Exposure,
#ifdef _WIN32
    TerrainVrs,
#endif
};
enum class Requirement { Enhanced, Visibility, Terrain, Streamed, Pages, Compute, Ray, Denoise, Neural,
                         Reservoir, Temporal, Atmosphere };
struct Row {
    const char* label;
    const char* description;
    Control control;
    F feature{F::Count};
    Requirement requires_mode{Requirement::Enhanced};
};
constexpr Row toggle(const char* label,const char* description,F feature,
                     Requirement requirement=Requirement::Enhanced){
    return {label,description,Control::Feature,feature,requirement};
}

#ifdef _WIN32
constexpr std::array output{
    Row{"RENDERER","Enhanced Direct3D 12 lunar lighting or classic mesh graphics.",Control::Path},
    Row{"ANTI-ALIASING","FXAA smooths the lunar scene before the interface is drawn.",Control::AA},
    Row{"TERRAIN VRS","Use 2x2 shading on terrain outside the build area.",Control::TerrainVrs}
};
constexpr std::array world{
    toggle("TERRAIN RELIEF","Adds crater detail and hills beyond the playfield.",F::TerrainRelief),
    toggle("MATERIAL DETAIL","Adds regolith grain, solar cells, dust and surface finish.",F::MaterialDetail)
};
constexpr std::array lights{
    toggle("PHYSICALLY BASED LIGHTING","Uses material roughness and metalness in the lighting model.",F::Pbr),
    toggle("LOCAL LIGHTS","Powered fixtures illuminate nearby surfaces.",F::LocalLights)
};
#else
constexpr std::array output{
    Row{"RENDERER","Auto uses available compute features. Deferred uses raster effects.",Control::Path},
    Row{"3D RENDER SCALE","Internal resolution. Interface stays at display resolution.",Control::Scale},
    Row{"ANTI-ALIASING","Temporal reconstruction, spatial FXAA, or off.",Control::AA},
    toggle("LEARNED UPSCALING","Learned detail recovery when temporal AA enlarges a low-resolution image.",F::LearnedUpscale,Requirement::Temporal),
    toggle("VISIBILITY BUFFER","Reconstructs materials from triangle IDs on the compute path.",F::VisibilityBuffer,Requirement::Visibility)
};
constexpr std::array world{
    toggle("TERRAIN RELIEF","Draws the surrounding hills and boulders.",F::TerrainRelief),
    toggle("GPU TERRAIN CULLING","Uses GPU culling and indirect draws for terrain clusters.",F::GpuTerrain,Requirement::Terrain),
    toggle("STREAMED TERRAIN DETAIL","Streams fine boulder meshes around the camera.",F::StreamedGeometry,Requirement::Streamed),
    toggle("VIRTUAL TEXTURES","Streams near-field lunar ground material pages.",F::VirtualTextures,Requirement::Pages),
    toggle("MATERIAL TEXTURES","Adds material grain, relief, and panel detail.",F::MaterialDetail),
    toggle("DECALS AND DUST LAYERS","Shows landing-pad markings and dust buildup on surfaces.",F::Decals),
    toggle("HAIR SHADING","Adds directional highlights and strand detail.",F::Hair),
    toggle("CLOTH DETAIL","Adds fabric sheen, weave, and coverall wrinkles.",F::Cloth),
    toggle("SKIN SHADING","Adds skin lighting and screen-space diffusion.",F::Skin),
    Row{"TRANSPARENCY","Weighted order-independent blending or sorted mesh blending.",Control::Transparency}
};
constexpr std::array lights{
    toggle("PHYSICALLY BASED LIGHTING","Uses material roughness and metalness in the lighting model.",F::Pbr),
    toggle("LOCAL LIGHTS","Illuminates surfaces with nearby fixtures.",F::LocalLights),
    Row{"SUN SHADOW MAPS","Near shadow pages with cascades, cascades alone, or off.",Control::Shadows},
    toggle("RESTIR DIRECT LIGHTING","Reuses fixture samples when many lights are visible.",F::ReservoirLights,Requirement::Reservoir),
    toggle("SCREEN-SPACE AO","Adds contact shadows from visible geometry.",F::ScreenOcclusion),
    toggle("SCREEN-SPACE GI","Adds diffuse bounce when ray-traced GI is inactive.",F::ScreenIndirect),
    toggle("SCREEN-SPACE REFLECTIONS","Adds reflections when ray-traced reflections are inactive.",F::ScreenReflections)
};
constexpr std::array rays{
    toggle("SOFTWARE RAY TRACING","Traces scene triangles using OpenGL compute.",F::RayTracing,Requirement::Compute),
    toggle("RAY-TRACED SHADOWS","Traces sunlight and fixture visibility.",F::RayShadows,Requirement::Ray),
    toggle("RAY-TRACED REFLECTIONS","Reflects geometry even when it is off screen.",F::RayReflections,Requirement::Ray),
    toggle("RAY-TRACED AO","Traces near-field contact occlusion.",F::RayOcclusion,Requirement::Ray),
    toggle("RAY-TRACED GI","Traces one diffuse bounce through the scene.",F::RayIndirect,Requirement::Ray),
    Row{"COMPUTE LIGHTING SCALE","Ray and reservoir buffer resolution relative to the 3D image.",Control::RayScale,F::Count,Requirement::Compute},
    toggle("RAY DENOISING","Filters compute lighting across space and time.",F::RayDenoise,Requirement::Compute),
    toggle("LEARNED DENOISING","Uses learned confidence weights in the ray filter.",F::LearnedDenoise,Requirement::Denoise),
    toggle("NEURAL RADIANCE CACHE","Learns a local diffuse lighting estimate from ray samples.",F::NeuralRadiance,Requirement::Neural)
};
constexpr std::array atmosphere{
    toggle("EARTH ATMOSPHERE","Draws the scattering sky above the Earth campus.",F::EarthAtmosphere),
    toggle("VOLUMETRIC CLOUDS","Adds ray-marched clouds to the Earth sky.",F::Clouds,Requirement::Atmosphere),
    toggle("DISTANCE HAZE","Adds atmospheric haze to distant Earth campus surfaces.",F::Haze),
    toggle("VOLUMETRIC EXHAUST","Renders depth-clipped rocket and trader engine plumes.",F::VolumetricPlumes),
    toggle("VOLUMETRIC LUNAR DUST","Draws a dust cloud below a low lander.",F::VolumetricDust)
};
#endif
constexpr std::array image{
    Row{"BLOOM INTENSITY","Glow from bright surfaces and lights. Set to 0% to disable.",Control::Bloom},
    Row{"EXPOSURE","Adjusts world brightness in stops.",Control::Exposure},
    toggle("FILMIC TONE MAPPING","Uses a filmic highlight roll-off. Off uses a simpler tone curve.",F::Filmic),
    toggle("VIGNETTE","Adds gentle shading at the edges of the 3D image.",F::Vignette)
};
struct Tab { const char* name; const Row* rows; int count; };
template<std::size_t N> constexpr Tab tab(const char* name,const std::array<Row,N>& rows){
    return {name,rows.data(),int(N)};
}
#ifdef _WIN32
constexpr std::array tabs{tab("OUTPUT",output),tab("WORLD",world),tab("LIGHTING",lights),tab("IMAGE",image)};
#else
constexpr std::array tabs{tab("OUTPUT",output),tab("WORLD",world),tab("LIGHTING",lights),
                          tab("RAY TRACING",rays),tab("IMAGE",image),tab("ATMOSPHERE",atmosphere)};
static_assert([]{
    std::uint32_t mask{};
    for(const auto& t:tabs)
        for(int i=0;i<t.count;++i){
            if(t.rows[i].control!=Control::Feature) continue;
            const auto bit=graphics_bit(t.rows[i].feature);
            if(mask&bit) return false;
            mask|=bit;
        }
    return mask==all_graphics_features;
}());

constexpr std::array path_names{"AUTO","DEFERRED","CLASSIC"};
constexpr std::array aa_names{"TEMPORAL","FXAA","OFF"};
#endif
constexpr std::array shadow_names{"PAGED + CSM","CASCADES","OFF"};
constexpr std::array transparency_names{"WEIGHTED","SORTED"};

const char* restriction(const Row& row,const GraphicsSettings& s,const GraphicsCapabilities& caps){
#ifdef _WIN32
    if(!caps.enhanced) return "ENHANCED DIRECT3D 12 RENDERER UNAVAILABLE";
    if(row.control==Control::Path) return nullptr;
    if(s.path==GraphicsPath::Classic) return "SWITCH TO ENHANCED TO ADJUST";
    if(row.control==Control::TerrainVrs && !caps.variable_rate_shading) return "GPU TERRAIN VRS UNAVAILABLE";
#else
    if(row.control==Control::Path) return caps.enhanced?nullptr:"ENHANCED OPENGL RENDERER UNAVAILABLE";
    if(!caps.enhanced) return "ENHANCED OPENGL RENDERER UNAVAILABLE";
    if(s.path==GraphicsPath::Classic) return "SWITCH FROM CLASSIC TO ADJUST";
    const bool auto_path=s.path==GraphicsPath::Auto;
    switch(row.requires_mode){
        case Requirement::Visibility:
            if(!caps.visibility_buffer) return "VISIBILITY BUFFER UNAVAILABLE";
            if(!auto_path) return "SELECT AUTO RENDERER";
            break;
        case Requirement::Terrain:
        case Requirement::Streamed:
            if(!caps.gpu_terrain) return "GPU TERRAIN UNAVAILABLE";
            if(!auto_path || !s.enabled(F::VisibilityBuffer)) return "REQUIRES AUTO AND VISIBILITY BUFFER";
            if(!s.enabled(F::TerrainRelief)) return "ENABLE TERRAIN RELIEF";
            if(row.requires_mode==Requirement::Streamed && !s.enabled(F::GpuTerrain)) return "ENABLE GPU TERRAIN CULLING";
            break;
        case Requirement::Pages:
            if(!caps.pages) return "MATERIAL PAGE CACHE UNAVAILABLE";
            if(!auto_path) return "SELECT AUTO RENDERER";
            break;
        case Requirement::Ray:
        case Requirement::Compute:
        case Requirement::Denoise:
        case Requirement::Neural:
        case Requirement::Reservoir:
            if(!caps.ray_lighting) return "COMPUTE LIGHTING UNAVAILABLE";
            if(!auto_path) return "SELECT AUTO RENDERER";
            if(row.requires_mode==Requirement::Ray && !s.enabled(F::RayTracing)) return "ENABLE SOFTWARE RAY TRACING";
            if(row.requires_mode==Requirement::Reservoir && !s.enabled(F::LocalLights)) return "ENABLE LOCAL LIGHTS";
            if(row.requires_mode==Requirement::Neural && !caps.neural_radiance) return "NEURAL CACHE UNAVAILABLE";
            if(row.requires_mode==Requirement::Neural && (!s.enabled(F::RayTracing) || !s.enabled(F::RayIndirect)))
                return "ENABLE RAY-TRACED GI";
            if((row.requires_mode==Requirement::Neural || row.requires_mode==Requirement::Denoise) && !s.enabled(F::RayDenoise))
                return "ENABLE RAY DENOISING";
            break;
        case Requirement::Temporal:
            if(s.aa!=GraphicsAA::Temporal) return "ENABLE TEMPORAL AA";
            break;
        case Requirement::Atmosphere:
            if(!s.enabled(F::EarthAtmosphere)) return "ENABLE EARTH ATMOSPHERE";
            break;
        case Requirement::Enhanced:
            break;
    }
#endif
    return nullptr;
}
} // namespace

void GameApp::render_graphics_settings(Renderer& r){
    ui::title(r,{32,24},"GRAPHICS",theme_);
    const float width=std::min(950.0f,ui_width(r)-40.0f);
    const float height=std::min(570.0f,ui_height(r)-82.0f);
    const Rect panel{(ui_width(r)-width)*0.5f,65.0f,width,height};
    ui::panel(r,panel,theme_);
    const auto caps=r.graphics_capabilities();
#ifdef _WIN32
    const char* mode=mesh_layout_active()?"DEBUG MESH ACTIVE / SETTINGS APPLY TO FULL GRAPHICS":
        !caps.enhanced?"DIRECT3D 12 CLASSIC / ENHANCED GRAPHICS UNAVAILABLE":
        graphics_.path==GraphicsPath::Classic?"DIRECT3D 12 CLASSIC":"DIRECT3D 12 ENHANCED LUNAR GRAPHICS";
    const std::string status=std::string(mode)+(caps.enhanced_barriers?" / ENHANCED BARRIERS":" / LEGACY BARRIERS");
#else
    const char* status=mesh_layout_active()?"DEBUG MESH ACTIVE  /  SETTINGS APPLY TO FULL GRAPHICS":
        !caps.enhanced?"STANDARD BACKEND  /  ENHANCED GRAPHICS UNAVAILABLE":
        graphics_.path==GraphicsPath::Classic?"CLASSIC VIEW":
        graphics_.path==GraphicsPath::Auto && (caps.ray_lighting || caps.visibility_buffer || caps.pages)?"AUTO  /  OPENGL COMPUTE AVAILABLE":
        "DEFERRED RASTER";
#endif
    const float x=panel.x+14.0f;
    const float inner=panel.w-28.0f;
    ui::label_fitted(r,{x,panel.y+13.0f,inner,15.0f},status,1.05f,theme_,true);

    constexpr float gap=4.0f;
    const float tab_width=(inner-gap*(tabs.size()-1))/tabs.size();
    for(int i=0;i<int(tabs.size());++i){
        const Rect box{x+(tab_width+gap)*i,panel.y+36.0f,tab_width,30.0f};
        if(ui::button(r,ui_input_,box,tabs[i].name,theme_) && i!=graphics_tab_){
            graphics_tab_=i;
            graphics_page_=0;
        }
        if(i==graphics_tab_) r.draw_rect({box.x,box.y+box.h-3,box.w,3},theme_.accent);
    }

    const auto& page=tabs[graphics_tab_];
    constexpr float row_height=43.0f;
    const float list_top=panel.y+74.0f;
    const float bottom=panel.y+panel.h;
    const float footer=bottom-48.0f;
    const int rows_per_page=std::max(1,int((footer-list_top-8.0f)/row_height));
    const int page_count=(page.count+rows_per_page-1)/rows_per_page;
    graphics_page_=std::clamp(graphics_page_,0,page_count-1);
    if(Rect{x,list_top,inner,footer-list_top}.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y))
       && ui_input_.wheel!=0.0f)
        graphics_page_=std::clamp(graphics_page_+(ui_input_.wheel<0?1:-1),0,page_count-1);

    auto next=graphics_;
    const int first=graphics_page_*rows_per_page;
    for(int i=first;i<std::min(first+rows_per_page,page.count);++i){
        const Row& row=page.rows[i];
        const float y=list_top+float(i-first)*row_height;
        const char* blocked=restriction(row,graphics_,caps);
        const float control_width=std::min(172.0f,inner*0.36f);
        const Rect control{x+inner-control_width,y+6.0f,control_width,30.0f};
        const float label_width=inner-control_width-12.0f;
        const bool paged=row.control==Control::Shadows && graphics_.shadows==GraphicsShadows::Paged;
        const char* description=paged && (!caps.pages || graphics_.path!=GraphicsPath::Auto)
            ?"USING CASCADES. SHADOW PAGES REQUIRE AUTO AND PAGE CACHE.":row.description;
        r.draw_rect({x,y,inner,1.0f},theme_.border);
        ui::label_fitted(r,{x+4,y+6,label_width,13},row.label,1.24f,theme_,blocked);
        ui::label_fitted(r,{x+4,y+24,label_width,12},blocked?blocked:description,1.08f,theme_,true);

        const auto cycle=[&](auto& value,const auto& names){
            if(ui::button(r,ui_input_,control,names[int(value)],theme_,!blocked))
                value=static_cast<std::decay_t<decltype(value)>>((int(value)+1)%names.size());
        };
        const auto step=[&](int& value,int lower,int upper,int delta,const std::string& label){
            constexpr float arrow=30.0f;
            const Rect minus{control.x,control.y,arrow,control.h};
            const Rect plus{control.x+control.w-arrow,control.y,arrow,control.h};
            if(ui::button(r,ui_input_,minus,"-",theme_,!blocked && value>lower))
                value=std::max(lower,value-delta);
            if(ui::button(r,ui_input_,plus,"+",theme_,!blocked && value<upper))
                value=std::min(upper,value+delta);
            const float space=control.w-2.0f*arrow-4.0f;
            const float scale=ui::fit_text(label,ui::font_scale(0.84f,theme_),{space,15});
            const auto extent=ui::text_extent(label,scale);
            r.draw_text({control.x+(control.w-extent.x)*0.5f,control.y+(control.h-extent.y)*0.5f},
                        label,scale,blocked?theme_.muted:theme_.text);
        };
        switch(row.control){
            case Control::Feature:{
                const bool on=graphics_.enabled(row.feature);
                if(ui::button(r,ui_input_,control,on?"ON":"OFF",theme_,!blocked)) next.set(row.feature,!on);
                break;
            }
            case Control::Path:
#ifdef _WIN32
                if(ui::button(r,ui_input_,control,next.path==GraphicsPath::Classic?"CLASSIC":"ENHANCED",theme_,!blocked))
                    next.path=next.path==GraphicsPath::Classic?GraphicsPath::Auto:GraphicsPath::Classic;
#else
                cycle(next.path,path_names);
#endif
                break;
            case Control::AA:
#ifdef _WIN32
                if(ui::button(r,ui_input_,control,next.aa==GraphicsAA::Off?"OFF":"FXAA",theme_,!blocked))
                    next.aa=next.aa==GraphicsAA::Off?GraphicsAA::Fxaa:GraphicsAA::Off;
#else
                cycle(next.aa,aa_names);
#endif
                break;
#ifdef _WIN32
            case Control::TerrainVrs:
                if(ui::button(r,ui_input_,control,next.terrain_vrs?"ON":"OFF",theme_,!blocked))
                    next.terrain_vrs=!next.terrain_vrs;
                break;
#endif
            case Control::Shadows: cycle(next.shadows,shadow_names); break;
            case Control::Transparency: cycle(next.transparency,transparency_names); break;
            case Control::Scale: step(next.render_scale,50,100,5,std::to_string(next.render_scale)+"%"); break;
            case Control::RayScale: step(next.ray_scale,25,75,5,std::to_string(next.ray_scale)+"%"); break;
            case Control::Bloom: step(next.bloom,0,150,10,std::to_string(next.bloom)+"%"); break;
            case Control::Exposure:{
                const int stops=std::abs(next.exposure_tenths);
                const std::string label=(next.exposure_tenths<0?"-":"")+std::to_string(stops/10)+
                    "."+std::to_string(stops%10)+" EV";
                step(next.exposure_tenths,-20,20,1,label);
                break;
            }
        }
    }

    const float middle=x+inner*0.5f;
    if(ui::button(r,ui_input_,{x,footer,82,31},"BACK",theme_)) screen_=Screen::Settings;
    if(page_count>1){
        if(ui::button(r,ui_input_,{middle-66,footer,31,31},"<",theme_,graphics_page_>0)) --graphics_page_;
        if(ui::button(r,ui_input_,{middle+35,footer,31,31},">",theme_,graphics_page_+1<page_count)) ++graphics_page_;
        ui::label_fitted(r,{middle-26,footer+9,54,13},
            std::to_string(graphics_page_+1)+"/"+std::to_string(page_count),0.84f,theme_,true);
    }
    const bool reset=ui::button(r,ui_input_,{x+inner-116,footer,116,31},"RESET DEFAULTS",theme_,caps.enhanced);
    if(reset) next={};
    if(next!=graphics_ || reset){
        graphics_=next;
        persist_preferences();
    }
    if(preferences_save_failed_)
        ui::label_fitted(r,{x,bottom-13,inner,11},"APPLIED FOR THIS SESSION. PREFERENCES COULD NOT BE SAVED.",0.78f,theme_);
}
} // namespace game
