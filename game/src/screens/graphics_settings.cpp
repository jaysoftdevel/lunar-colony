#include "game/game_app.hpp"
#include "engine/ui_metrics.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace game {
using namespace eng;
namespace {
using F=GraphicsFeature;

enum class Control {
    Feature, DisplayMode, Resolution, ApplyResolution, Vsync, Path, Scale, RayScale, AA, AAQuality,
    Shadows, ShadowQuality, ShadowResolution, Transparency, Bloom, BloomQuality, Exposure,
    SurfaceResolution, TextureFilter, GroundClutter, ClutterRange, Particles,
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

constexpr std::array display{
    Row{"DISPLAY MODE","Choose between fullscreen and a window.",Control::DisplayMode},
    Row{"DISPLAY RESOLUTION","Choose a size reported by the current display.",Control::Resolution},
    Row{"APPLY DISPLAY RESOLUTION","Resize the game to the selected display size.",Control::ApplyResolution},
    Row{"VERTICAL SYNC","Sync presentation to the display refresh to reduce tearing.",Control::Vsync},
    Row{"3D RENDER SCALE","Set internal 3D resolution independently of the interface.",Control::Scale},
    Row{"ANTI-ALIASING","Choose the method used to smooth edges in the 3D image.",Control::AA},
#ifdef _WIN32
    Row{"ANTI-ALIASING QUALITY","Set the amount of spatial edge smoothing.",Control::AAQuality},
#else
    Row{"ANTI-ALIASING QUALITY","Set the spatial filter. High adds edge cleanup after temporal AA.",Control::AAQuality},
#endif
#ifndef _WIN32
    toggle("LEARNED UPSCALING","Reconstructs detail at reduced scale with temporal AA.",F::LearnedUpscale,Requirement::Temporal),
#endif
};
constexpr std::array textures{
#ifndef _WIN32
    Row{"SURFACE TEXTURE RESOLUTION","Finest sampled mip of lunar terrain and structure maps.",Control::SurfaceResolution},
    Row{"TEXTURE FILTERING","Choose mip filtering or GPU-supported anisotropy for angled surfaces.",Control::TextureFilter},
    toggle("VIRTUAL TEXTURES","Adjusts near-field ground texture detail and page uploads.",F::VirtualTextures,Requirement::Pages),
    toggle("HAIR SHADING","Adds directional highlights and strand detail.",F::Hair),
    toggle("CLOTH DETAIL","Scales fabric texture detail.",F::Cloth),
    toggle("SKIN SHADING","Adjusts the sample count for skin diffusion.",F::Skin),
#endif
    toggle("MATERIAL DETAIL","Scales lunar textures above the basic material detail.",F::MaterialDetail),
    toggle("PROJECTED SURFACE DETAIL","Controls landing-pad markings, wear and dust layers.",F::Decals)
};
constexpr std::array environment{
    toggle("TERRAIN RELIEF","Scales the detail of cratered terrain around the build area.",F::TerrainRelief),
#ifndef _WIN32
    Row{"GROUND CLUTTER","Sets the density of decorative boulders outside the build area.",Control::GroundClutter},
    Row{"GROUND CLUTTER RANGE","Controls how far the boulder field extends beyond the build area.",Control::ClutterRange},
    toggle("STREAMED TERRAIN DETAIL","Sets the number of fine boulder pages resident near the camera.",F::StreamedGeometry,Requirement::Streamed),
    toggle("EARTH ATMOSPHERE","Draws the scattering sky above the Earth campus.",F::EarthAtmosphere),
    toggle("VOLUMETRIC CLOUDS","Adjusts ray-march steps in the Earth cloud layer.",F::Clouds,Requirement::Atmosphere),
    toggle("DISTANCE HAZE","Adjusts the density of Earth campus haze.",F::Haze),
#endif
};
constexpr std::array lights{
    toggle("PHYSICALLY BASED LIGHTING","Uses material roughness and metalness in the lighting model.",F::Pbr),
    toggle("LOCAL LIGHTS","Scales the number of powered fixtures that illuminate surfaces.",F::LocalLights),
#ifndef _WIN32
    Row{"SUN SHADOW MAPS","Near shadow pages with cascades, cascades alone, or off.",Control::Shadows},
    Row{"SHADOW FILTER QUALITY","Scales filter samples and near-shadow page updates.",Control::ShadowQuality},
    Row{"CASCADE SHADOW RESOLUTION","Sets the pixel size of each directional shadow map.",Control::ShadowResolution},
    toggle("RESTIR DIRECT LIGHTING","Adjusts fixture candidates and nearby samples.",F::ReservoirLights,Requirement::Reservoir),
    toggle("SCREEN-SPACE AO","Adjusts the number of contact occlusion samples.",F::ScreenOcclusion),
    toggle("SCREEN-SPACE GI","Adjusts diffuse bounce samples when ray GI is inactive.",F::ScreenIndirect),
    toggle("SCREEN-SPACE REFLECTIONS","Adjusts steps and refinement for screen reflections.",F::ScreenReflections),
#endif
};
constexpr std::array effects{
    Row{"PARTICLE DENSITY","Controls airborne lander dust and rover welding sparks.",Control::Particles},
    Row{"BLOOM INTENSITY","Glow from bright surfaces and lights. Set to 0% to disable.",Control::Bloom},
    Row{"BLOOM QUALITY","Scales the resolution of the glow buffer.",Control::BloomQuality},
    Row{"EXPOSURE","Adjusts world brightness in stops.",Control::Exposure},
    toggle("FILMIC TONE MAPPING","Smooths the transition into bright highlights.",F::Filmic),
    toggle("VIGNETTE","Adjusts shading at the edges of the 3D image.",F::Vignette),
#ifndef _WIN32
    toggle("VOLUMETRIC EXHAUST","Adjusts integration steps for engine plumes.",F::VolumetricPlumes),
    toggle("VOLUMETRIC LUNAR DUST","Adjusts integration steps for the lander dust cloud.",F::VolumetricDust),
    Row{"TRANSPARENCY","Weighted order-independent blending or sorted mesh blending.",Control::Transparency},
#endif
};
constexpr std::array advanced{
#ifdef _WIN32
    Row{"RENDERER","Enhanced Direct3D 12 lunar lighting or Classic with basic textures.",Control::Path},
    Row{"TERRAIN VRS","Use 2x2 shading on terrain outside the build area.",Control::TerrainVrs},
#else
    Row{"RENDERER","Auto uses available compute features. Classic uses basic textures.",Control::Path},
    toggle("VISIBILITY BUFFER","Reconstructs materials from triangle IDs on the compute path.",F::VisibilityBuffer,Requirement::Visibility),
    toggle("GPU TERRAIN CULLING","Uses GPU culling and indirect draws for terrain clusters.",F::GpuTerrain,Requirement::Terrain),
#endif
};
#ifndef _WIN32
constexpr std::array rays{
    toggle("SOFTWARE RAY TRACING","Traces scene triangles using OpenGL compute.",F::RayTracing,Requirement::Compute),
    toggle("RAY-TRACED SHADOWS","Adjusts sunlight ray reach and enables fixture visibility.",F::RayShadows,Requirement::Ray),
    toggle("RAY-TRACED REFLECTIONS","Scales the maximum reflection ray distance.",F::RayReflections,Requirement::Ray),
    toggle("RAY-TRACED AO","Scales the contact occlusion radius.",F::RayOcclusion,Requirement::Ray),
    toggle("RAY-TRACED GI","Scales the diffuse bounce ray distance.",F::RayIndirect,Requirement::Ray),
    Row{"COMPUTE LIGHTING SCALE","Ray and reservoir buffer resolution relative to the 3D image.",Control::RayScale,F::Count,Requirement::Compute},
    toggle("RAY DENOISING","Scales spatial filter samples for compute lighting.",F::RayDenoise,Requirement::Compute),
    toggle("LEARNED DENOISING","Uses learned confidence weights in the ray filter.",F::LearnedDenoise,Requirement::Denoise),
    toggle("NEURAL RADIANCE CACHE","Learns a local diffuse lighting estimate from ray samples.",F::NeuralRadiance,Requirement::Neural)
};
#endif
struct Tab { const char* name; const Row* rows; int count; };
template<std::size_t N> constexpr Tab tab(const char* name,const std::array<Row,N>& rows){
    return {name,rows.data(),int(N)};
}
constexpr std::array tabs{
    tab("DISPLAY",display),tab("TEXTURES",textures),tab("ENVIRONMENT",environment),tab("LIGHTING",lights),
    tab("EFFECTS",effects),tab("ADVANCED",advanced),
#ifndef _WIN32
    tab("RAY TRACING",rays),
#endif
};
#ifndef _WIN32
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
constexpr std::array quality_names{"OFF","LOW","MEDIUM","HIGH"};
constexpr std::array material_quality_names{"BASIC","LOW","MEDIUM","HIGH"};
constexpr std::array detail_names{"LOW","MEDIUM","HIGH"};
constexpr std::array display_names{"WINDOWED","FULLSCREEN"};
constexpr std::array enabled_names{"OFF","ON"};
constexpr std::array texture_names{"BILINEAR","TRILINEAR","2X ANISOTROPIC","4X ANISOTROPIC","8X ANISOTROPIC","16X ANISOTROPIC"};
constexpr std::array surface_names{"128 PX","256 PX","512 PX"};
constexpr std::array cascade_names{"768 PX","1024 PX","1536 PX"};
constexpr std::array profiles{BenchmarkProfile::Performance,BenchmarkProfile::Balanced,BenchmarkProfile::High};
constexpr std::array profile_labels{"PERFORMANCE","BALANCED","HIGH QUALITY"};

struct Popup {
    Rect rect;
    std::vector<std::string> labels;
    int selected{};
};
constexpr float popup_row=26.f;

const char* restriction(const Row& row,const GraphicsSettings& s,const GraphicsCapabilities& caps){
    switch(row.control){
        case Control::DisplayMode:
        case Control::Resolution:
        case Control::ApplyResolution:
        case Control::Particles: return nullptr;
        case Control::Vsync: return caps.vertical_sync?nullptr:"SWAP INTERVAL CONTROL UNAVAILABLE";
        default: break;
    }
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
    if(row.control==Control::AAQuality && s.aa==GraphicsAA::Off) return "ENABLE ANTI-ALIASING";
    if((row.control==Control::ShadowQuality || row.control==Control::ShadowResolution) &&
       s.shadows==GraphicsShadows::Off) return "ENABLE SUN SHADOW MAPS";
    if((row.control==Control::GroundClutter || row.control==Control::ClutterRange) &&
       !s.enabled(F::TerrainRelief)) return "ENABLE TERRAIN RELIEF";
    if(row.control==Control::ClutterRange && !s.ground_clutter) return "ENABLE GROUND CLUTTER";
    if(row.control==Control::BloomQuality && s.bloom==0) return "INCREASE BLOOM INTENSITY";
    return nullptr;
}
} // namespace

void GameApp::render_graphics_settings(Renderer& r){
    ensure_display_modes();
    const float width=std::min(1400.f,ui_width(r)-40.f);
    const float height=std::min(690.f,ui_height(r)-82.f);
    const Rect panel{(ui_width(r)-width)*.5f,std::max(65.f,(ui_height(r)-height)*.5f),width,height};
    ui::title(r,{panel.x,panel.y-42.f},"GRAPHICS",theme_);
    ui::panel(r,panel,theme_);
    r.draw_rect({panel.x,panel.y,panel.w,2.f},theme_.accent);
    const auto caps=r.graphics_capabilities();
#ifdef _WIN32
    const char* status=mesh_layout_active()?"DEBUG MESH ACTIVE / SETTINGS APPLY TO FULL GRAPHICS":
        !caps.enhanced || graphics_.path==GraphicsPath::Classic?"DIRECT3D 12 CLASSIC / BASIC TEXTURES":
        "DIRECT3D 12 ENHANCED LUNAR GRAPHICS";
#else
    const char* status=mesh_layout_active()?"DEBUG MESH ACTIVE / SETTINGS APPLY TO FULL GRAPHICS":
        !caps.enhanced || graphics_.path==GraphicsPath::Classic?"OPENGL CLASSIC / BASIC TEXTURES":
        graphics_.path==GraphicsPath::Auto && (caps.ray_lighting || caps.visibility_buffer || caps.pages)?
            "OPENGL AUTO / COMPUTE AVAILABLE":"OPENGL DEFERRED RASTER";
#endif
    const float bottom=panel.y+panel.h;
    const float footer=bottom-45.f;
    constexpr int benchmark_tab=static_cast<int>(tabs.size());
    const float nav_x=panel.x+14.f,nav_width=std::clamp(panel.w*.17f,126.f,182.f);
    const float nav_height=std::min(37.f,(footer-panel.y-52.f)/(benchmark_tab+1));
    const float x=nav_x+nav_width+22.f,inner=panel.x+panel.w-14.f-x;
    const int open_at_start=graphics_dropdown_;
    auto control_input=ui_input_;
    if(open_at_start>=0)control_input.left_pressed=false;

    ui::label_fitted(r,{nav_x+6,panel.y+18,nav_width-12,18},"CATEGORIES",1.04f,theme_,true);
    r.draw_line({x-11,panel.y+12},{x-11,bottom-12},theme_.border,1.f);
    for(int i=0;i<=benchmark_tab;++i){
        const Rect box{nav_x,panel.y+43.f+nav_height*i,nav_width,nav_height-4.f};
        if(ui::button(r,control_input,box,i==benchmark_tab?"BENCHMARK":tabs[i].name,theme_) && i!=graphics_tab_){
            graphics_tab_=i;
            graphics_page_=0;
            graphics_dropdown_=-1;
        }
        if(i==graphics_tab_)r.draw_rect({box.x,box.y+2,3.f,box.h-4},theme_.accent);
    }

    ui::label_fitted(r,{x,panel.y+12.f,inner,17.f},status,1.f,theme_,true);
    if(graphics_tab_==benchmark_tab){
        render_benchmark_settings(r,{x-14.f,panel.y,inner+28.f,panel.h},caps);
        return;
    }

    auto next=graphics_;
    int active_profile=-1;
    std::array<GraphicsSettings,profiles.size()> preset_settings;
    for(std::size_t i=0;i<profiles.size();++i){
        preset_settings[i]=graphics_profile_settings(profiles[i],graphics_,caps);
        if(graphics_==preset_settings[i])active_profile=static_cast<int>(i);
    }
    GraphicsSettings defaults;
    defaults.vertical_sync=graphics_.vertical_sync;
    const char* preset=active_profile>=0?profile_labels[active_profile]:graphics_==defaults?"DEFAULT":"CUSTOM";
    const std::string preset_label=std::string("GRAPHICS QUALITY  /  ")+preset;
    ui::label_fitted(r,{x,panel.y+38.f,inner,16.f},preset_label,1.22f,theme_);
    const float preset_width=(inner-12.f)/3.f;
    for(int i=0;i<static_cast<int>(profiles.size());++i){
        const Rect box{x+i*(preset_width+6),panel.y+59.f,preset_width,31.f};
        if(ui::button(r,control_input,box,profile_labels[i],theme_,caps.enhanced))next=preset_settings[i];
        if(active_profile==i)r.draw_rect({box.x+2,box.y+box.h-3,box.w-4,2.f},theme_.accent);
    }

    const auto& page=tabs[graphics_tab_];
    const float list_top=panel.y+132.f;
    ui::label_fitted(r,{x,panel.y+104.f,inner,18.f},std::string(page.name)+" SETTINGS",1.25f,theme_);
    r.draw_line({x,panel.y+125.f},{x+inner,panel.y+125.f},theme_.border,1.f);
    constexpr float row_height=54.f,column_gap=18.f;
    const int columns=page.count>5 && inner>=620.f?2:1;
    const float column_width=(inner-column_gap*(columns-1))/columns;
    const int rows_available=std::max(1,static_cast<int>((footer-list_top-6.f)/row_height));
    const int rows_per_column=std::min(rows_available,(page.count+columns-1)/columns);
    const int per_page=rows_per_column*columns;
    const int page_count=(page.count+per_page-1)/per_page;
    graphics_page_=std::clamp(graphics_page_,0,page_count-1);
    if(open_at_start<0 && Rect{x,list_top,inner,footer-list_top}.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y))
       && ui_input_.wheel!=0.f)
        graphics_page_=std::clamp(graphics_page_+(ui_input_.wheel<0?1:-1),0,page_count-1);

    if(columns==2){
        const float divider=x+column_width+column_gap*.5f;
        r.draw_line({divider,list_top},{divider,list_top+rows_per_column*row_height},theme_.border,1.f);
    }
    std::optional<Popup> popup;
    const int first=graphics_page_*per_page;
    for(int i=first;i<std::min(first+per_page,page.count);++i){
        const Row& row=page.rows[i];
        const int offset=i-first;
        const float left=x+(offset/rows_per_column)*(column_width+column_gap);
        const float y=list_top+(offset%rows_per_column)*row_height;
        const float control_width=std::min(162.f,column_width*.40f);
        const Rect control{left+column_width-control_width,y+5.f,control_width,31.f};
        const float label_width=column_width-control_width-10.f;
        const char* blocked=restriction(row,graphics_,caps);
        const bool paged=row.control==Control::Shadows && graphics_.shadows==GraphicsShadows::Paged;
        const char* description=paged && (!caps.pages || graphics_.path!=GraphicsPath::Auto)
            ?"USING CASCADES. SHADOW PAGES REQUIRE AUTO AND PAGE CACHE.":row.description;
        r.draw_rect({left,y+row_height-1,column_width,1.f},theme_.border);
        ui::label_fitted(r,{left+2,y+12,label_width,17},row.label,1.24f,theme_,blocked);
        ui::label_fitted(r,{left+2,y+37,column_width-4,15},blocked?blocked:description,1.03f,theme_,true);

        const auto choose=[&](int selected,const auto& labels,auto set_value){
            if(ui::button(r,control_input,control,std::string(labels[selected])+"  V",theme_,!blocked))
                graphics_dropdown_=i;
            if(graphics_dropdown_!=i || blocked)return;
            const float list_height=popup_row*labels.size();
            const float below=control.y+control.h+2.f;
            const float top=below+list_height<=footer?below:std::max(panel.y+3.f,control.y-list_height-2.f);
            const Rect bounds{control.x,top,control.w,list_height};
            if(open_at_start==i && ui_input_.left_pressed){
                if(bounds.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y))){
                    const int option=std::clamp(static_cast<int>((ui_input_.mouse_y-top)/popup_row),
                                                0,static_cast<int>(labels.size())-1);
                    set_value(option);
                }
                graphics_dropdown_=-1;
            }else{
                popup=Popup{bounds,{},selected};
                for(const auto& label:labels)popup->labels.emplace_back(label);
            }
        };
        const auto cycle=[&](auto& value,const auto& names){
            choose(static_cast<int>(value),names,[&](int index){
                value=static_cast<std::decay_t<decltype(value)>>(index);
            });
        };
        const auto quality=[&](int& level){
            choose(level-1,detail_names,[&](int index){level=index+1;});
        };
        const auto step=[&](int& value,int lower,int upper,int delta,const std::string& label){
            constexpr float arrow=28.f;
            if(ui::button(r,control_input,{control.x,control.y,arrow,control.h},"-",theme_,!blocked && value>lower))
                value=std::max(lower,value-delta);
            if(ui::button(r,control_input,{control.x+control.w-arrow,control.y,arrow,control.h},"+",theme_,!blocked && value<upper))
                value=std::min(upper,value+delta);
            ui::label_fitted(r,{control.x+arrow+2,control.y+8,control.w-2*arrow-4,15},label,1.04f,theme_,blocked);
        };
        switch(row.control){
            case Control::Feature:{
                if(scalable_graphics_feature(row.feature))
                    choose(graphics_.quality(row.feature),row.feature==F::MaterialDetail?material_quality_names:quality_names,
                           [&](int index){next.set_quality(row.feature,index);});
                else
                    choose(graphics_.enabled(row.feature),enabled_names,[&](int index){next.set(row.feature,index!=0);});
                break;
            }
            case Control::DisplayMode:
                choose(app_.fullscreen(),display_names,[&](int index){app_.set_fullscreen(index!=0);});
                break;
            case Control::Resolution:{
                const auto& mode=display_modes_[display_mode_index_];
                step(display_mode_index_,0,static_cast<int>(display_modes_.size())-1,1,
                     std::to_string(mode.width)+" X "+std::to_string(mode.height));
                break;
            }
            case Control::ApplyResolution:
                if(ui::button(r,control_input,control,"APPLY SIZE",theme_)){
                    const auto& mode=display_modes_[display_mode_index_];
                    graphics_display_failed_=!app_.set_display_mode(mode.width,mode.height);
                }
                break;
            case Control::Vsync:
                cycle(next.vertical_sync,enabled_names);
                break;
            case Control::Path:
#ifdef _WIN32
                choose(next.path==GraphicsPath::Classic,std::array{"ENHANCED","CLASSIC"},
                       [&](int index){next.path=index==0?GraphicsPath::Auto:GraphicsPath::Classic;});
#else
                cycle(next.path,path_names);
#endif
                break;
            case Control::AA:
#ifdef _WIN32
                choose(next.aa==GraphicsAA::Off,std::array{"FXAA","OFF"},
                       [&](int index){next.aa=index==0?GraphicsAA::Fxaa:GraphicsAA::Off;});
#else
                cycle(next.aa,aa_names);
#endif
                break;
#ifdef _WIN32
            case Control::TerrainVrs: cycle(next.terrain_vrs,enabled_names); break;
#endif
            case Control::Shadows: cycle(next.shadows,shadow_names); break;
            case Control::AAQuality: quality(next.aa_quality); break;
            case Control::ShadowQuality: quality(next.shadow_quality); break;
            case Control::ShadowResolution:
                choose(next.shadow_resolution-1,cascade_names,[&](int index){next.shadow_resolution=index+1;});
                break;
            case Control::BloomQuality: quality(next.bloom_quality); break;
            case Control::SurfaceResolution:
                choose(next.surface_resolution-1,surface_names,[&](int index){next.surface_resolution=index+1;});
                break;
            case Control::TextureFilter:{
                int count=2;
                while(count<static_cast<int>(texture_names.size()) && texture_anisotropy(count)<=caps.texture_anisotropy)++count;
                choose(std::min(next.texture_filter,count-1),std::span(texture_names).first(count),
                       [&](int index){next.texture_filter=index;});
                break;
            }
            case Control::GroundClutter: cycle(next.ground_clutter,quality_names); break;
            case Control::Particles: cycle(next.particle_density,quality_names); break;
            case Control::Transparency: cycle(next.transparency,transparency_names); break;
            case Control::Scale: step(next.render_scale,50,100,5,std::to_string(next.render_scale)+"%"); break;
            case Control::RayScale: step(next.ray_scale,25,75,5,std::to_string(next.ray_scale)+"%"); break;
            case Control::ClutterRange: step(next.clutter_range,25,100,5,std::to_string(next.clutter_range)+"%"); break;
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

    if(ui::button(r,control_input,{nav_x,footer,nav_width,30},"BACK",theme_))screen_=Screen::Settings;
    const float middle=x+inner*.5f;
    if(page_count>1){
        if(ui::button(r,control_input,{middle-59,footer,30,30},"<",theme_,graphics_page_>0))--graphics_page_;
        if(ui::button(r,control_input,{middle+29,footer,30,30},">",theme_,graphics_page_+1<page_count))++graphics_page_;
        ui::label_fitted(r,{middle-21,footer+8,42,14},
            std::to_string(graphics_page_+1)+"/"+std::to_string(page_count),.95f,theme_,true);
    }else{
        ui::label_fitted(r,{x,footer+8,inner-150,14},"GRAPHICS CHANGES APPLY IMMEDIATELY",.95f,theme_,true);
    }
    const bool reset=ui::button(r,control_input,{x+inner-136,footer,136,30},"RESET DEFAULTS",theme_);
    if(reset)next={};
    if(next!=graphics_ || reset){
        graphics_=next;
        persist_preferences();
    }
    if(preferences_save_failed_ || (graphics_tab_==0 && graphics_display_failed_))
        ui::label_fitted(r,{x,bottom-13,inner,11},
            graphics_tab_==0 && graphics_display_failed_?"THE DISPLAY REJECTED THIS RESOLUTION.":
            "APPLIED FOR THIS SESSION. PREFERENCES COULD NOT BE SAVED.",.84f,theme_);
    if(popup){
        const auto& [bounds,labels,selected]=*popup;
        r.draw_rect(bounds,theme_.panel2);
        for(int i=0;i<static_cast<int>(labels.size());++i){
            const Rect item{bounds.x,bounds.y+i*popup_row,bounds.w,popup_row};
            const bool hovered=item.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y));
            r.draw_rect(item,hovered?Color{.16f,.30f,.36f,1}:i==selected?Color{.10f,.24f,.30f,1}:theme_.panel2);
            if(i==selected)r.draw_rect({item.x,item.y,2.f,item.h},theme_.accent);
            ui::label_fitted(r,{item.x+8,item.y+5,item.w-16,item.h-10},labels[i],1.04f,theme_);
        }
    }else if(open_at_start>=0){
        graphics_dropdown_=-1;
    }
}
} // namespace game
