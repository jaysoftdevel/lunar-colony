#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/resource_builder.hpp"
#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>

namespace game {
namespace {
using namespace eng;

constexpr std::array profile_names{"HIGH","BALANCED","PERFORMANCE","CLASSIC"};
constexpr std::array scene_names{"LUNAR ORBIT","LUNAR SURFACE","EARTH CAMPUS"};
constexpr std::array target_rates{30,60,90,120};

struct TestColony {
    enum Part { Pad, Paint, Alloy, Solar, Glass, Lamps, Marks, PartCount };
    std::array<std::vector<Vec3>,PartCount> parts;
    std::vector<Vec3> floor;
    std::array<LunarLight,64> fixtures{};
    std::size_t light_count{};

    TestColony(){
        resources::Builder shapes;
        shapes.box(floor,{0,-0.12f,0},{180,0.20f,180},false);
        for(int x=-3;x<=3;++x) for(int z=-2;z<=2;++z){
            const Vec3 p{14.0f*x,0,15.0f*z};
            shapes.box(parts[Pad],p+Vec3{0,0.14f,0},{10,0.28f,11},false);
            if((x+z)%2==0){
                shapes.dome(parts[Paint],p+Vec3{0,0.28f,0},3.4f,7,24,false,0.9f);
                shapes.dome(parts[Glass],p+Vec3{0,0.58f,0},2.75f,5,20,false,1.22f);
                shapes.box(parts[Alloy],p+Vec3{0,0.9f,3.5f},{2.4f,1.65f,1.9f},false);
            }else{
                shapes.box(parts[Paint],p+Vec3{0,1.42f,0},{7.2f,2.3f,6.4f},false);
                shapes.box(parts[Glass],p+Vec3{0,1.64f,3.25f},{5.8f,0.90f,0.12f},false);
                shapes.oriented_box(parts[Solar],p+Vec3{0,3.2f,0},{1,0,0},{0,1,0.35f},{0,-0.35f,1},
                                    {6.8f,0.12f,4.8f},false);
            }
            for(float side:{-1.0f,1.0f}){
                const auto tube=p+Vec3{4.1f*side,0.6f,0};
                shapes.cylinder_z(parts[Alloy],tube,7.8f,0.18f,10,false);
                shapes.box(parts[Lamps],p+Vec3{side*2.5f,2.25f,3.7f},{0.62f,0.18f,0.18f},false);
            }
            const Vec3 light=p+Vec3{0,2.5f,4.5f};
            fixtures[light_count++]={light,{0.35f,0.80f,1.0f,1},14,52};
            if((x+z)%2==0) fixtures[light_count++]={p+Vec3{0,3.8f,0},{1.0f,0.64f,0.33f,1},12,38};
        }
        const Vec3 landing{0,0,-47};
        shapes.cylinder_y(parts[Pad],landing+Vec3{0,0.2f,0},0.4f,10.5f,44,false);
        for(float side:{-1.0f,1.0f}){
            shapes.box(parts[Marks],landing+Vec3{side*2.2f,0.41f,0},{0.55f,0.02f,5.5f},false);
            shapes.cylinder_y(parts[Alloy],landing+Vec3{side*4.3f,0.95f,1.4f},1.1f,0.62f,12,false);
        }
        shapes.box(parts[Marks],landing+Vec3{0,0.41f,0},{4.5f,0.02f,0.55f},false);
        shapes.cylinder_y(parts[Paint],landing+Vec3{0,2.4f,0},3.9f,2.1f,28,false);
        shapes.dome(parts[Glass],landing+Vec3{0,4.35f,0},2.1f,7,28,false);
        std::sort(fixtures.begin(),fixtures.begin()+light_count,[](const auto& a,const auto& b){
            return dot(a.position,a.position)<dot(b.position,b.position);
        });
    }

    bool draw(Renderer& r,BenchmarkScene test,bool enhanced) const {
        const Vec3 eye=test==BenchmarkScene::Orbit?Vec3{75,79,94}:
            test==BenchmarkScene::Surface?Vec3{12,4,29}:Vec3{32,12,59};
        const Vec3 look=test==BenchmarkScene::Orbit?Vec3{0,1,-5}:
            test==BenchmarkScene::Surface?Vec3{-2,2,-32}:Vec3{0,14,-75};
        const auto view=multiply(perspective(radians(56.0f),float(r.width())/std::max(1,r.height()),0.2f,2100.0f),
                                 look_at(eye,look,{0,1,0}));
        bool active=false;
        if(test==BenchmarkScene::Campus){
            active=r.draw_campus_atmosphere(view,eye,0.0f);
            r.set_3d(view);
            r.draw_material_triangles_3d(floor,0,{0.37f,0.47f,0.30f,1});
        }else{
            r.set_3d(view);
            if(enhanced){
                const Vec3 sun=test==BenchmarkScene::Orbit?normalize(Vec3{-0.4f,0.60f,-0.5f}):
                    normalize(Vec3{-0.62f,0.15f,-0.58f});
                LunarScene scene{view,eye,look,sun,test==BenchmarkScene::Orbit?0.92f:0.42f,64.0f};
                scene.lights=fixtures;
                scene.light_count=light_count;
                active=r.begin_lunar_scene(scene);
            }
            if(!active) r.draw_triangles_3d(floor,{0.20f,0.21f,0.23f,1});
        }

        constexpr std::array materials{LunarMaterial::Pad,LunarMaterial::Paint,LunarMaterial::Alloy,
            LunarMaterial::Solar,LunarMaterial::Glass,LunarMaterial::Emissive,LunarMaterial::Marking};
        constexpr std::array colors{Color{0.46f,0.48f,0.51f,1},Color{0.83f,0.85f,0.80f,1},
            Color{0.67f,0.58f,0.43f,1},Color{0.18f,0.35f,0.60f,1},Color{0.27f,0.55f,0.68f,0.63f},
            Color{0.52f,0.87f,1.0f,1},Color{0.92f,0.81f,0.44f,1}};
        // Glass follows the opaque groups on the forward path.
        for(const auto part:{Pad,Paint,Alloy,Solar,Lamps,Marks,Glass}){
            if(test==BenchmarkScene::Campus) r.draw_material_triangles_3d(parts[part],part==Pad?3:5,colors[part]);
            else r.draw_lunar_surface(parts[part],materials[part],colors[part]);
        }
        if(test==BenchmarkScene::Surface && active){
            r.draw_lunar_volume({{0,5.8f,-47},{0,1,0},10.0f,2.8f,{0.40f,0.67f,1.0f,1},0.30f,0.75f,true});
            r.draw_lunar_volume({{0,0.50f,-44},{0,0,1},8.0f,2.1f,{0.52f,0.48f,0.38f,1},0.23f,0.0f,false});
        }
        return !enhanced || active;
    }
};

const char* profile_name(BenchmarkProfile profile){ return profile_names[static_cast<unsigned>(profile)]; }
std::string number(double value,int precision){
    std::ostringstream out;
    out<<std::fixed<<std::setprecision(precision)<<value;
    return out.str();
}
}

void GameApp::render_benchmark_settings(Renderer& r,Rect panel,const GraphicsCapabilities& caps){
    const float x=panel.x+14.0f,inner=panel.w-28.0f,top=panel.y+77.0f;
    const auto line=[&](float y,const std::string& text,bool muted=true){
        ui::label_fitted(r,{x,y,inner,15},text,1.03f,theme_,muted);
    };
    line(top,r.gpu_renderer()+" / "+r.gpu_version(),false);
    line(top+22,"FIXED SCENES MEASURE COMPLETED CPU AND GPU RENDER TIME");
    const bool supported=r.supports_graphics_benchmark();
    const auto target=target_rates[benchmark_target_index_];
    if(ui::button(r,ui_input_,{x,top+45,133,32},std::to_string(target)+" FPS TARGET",theme_)){
        benchmark_target_index_=(benchmark_target_index_+1)%target_rates.size();
        graphics_benchmark_.set_target_fps(target_rates[benchmark_target_index_]);
    }
    if(ui::button(r,ui_input_,{x+140,top+45,132,32},"RUN BENCHMARK",theme_,supported))
        graphics_benchmark_.start(graphics_,caps,r.width(),r.height(),target);

    const float footer=panel.y+panel.h-47.0f;
    if(ui::button(r,ui_input_,{x,footer,82,31},"BACK",theme_)) screen_=Screen::Settings;
    const auto status=graphics_benchmark_.status();
    if(!supported) line(top+92,"FRAME TIMING UNAVAILABLE ON THIS RENDERER");
    else if(status==BenchmarkStatus::Idle) line(top+92,"RUN THE SUITE TO RECOMMEND A PROFILE FOR THIS DISPLAY SIZE");
    else if(status==BenchmarkStatus::Cancelled) line(top+92,"BENCHMARK CANCELLED");
    else if(status==BenchmarkStatus::Resized) line(top+92,"DISPLAY SIZE CHANGED. RUN THE SUITE AGAIN.");
    else if(status==BenchmarkStatus::Failed) line(top+92,"RENDERER FRAME TIMING FAILED");
    if(status!=BenchmarkStatus::Complete) return;

    const auto* best=graphics_benchmark_.recommendation();
    if(!best){ line(top+92,"NO VALID BENCHMARK RESULTS"); return; }
    const bool same_size=graphics_benchmark_.same_size(r.width(),r.height());
    const auto measured_target=graphics_benchmark_.target_fps();
    const bool met_target=best->worst_p90()<=1000.0/measured_target;
    const std::string result=std::string(profile_name(best->profile))+
        (met_target?" RECOMMENDED FOR ":" FASTEST TESTED / TARGET ")+std::to_string(measured_target)+" FPS";
    line(top+90,same_size?result:"DISPLAY SIZE CHANGED. RUN THE SUITE AGAIN.",false);
    const auto scene_count=graphics_benchmark_.scene_count();
    const float table_y=top+112.0f;
    const float row_height=std::clamp((footer-table_y-6.0f)/5.0f,12.0f,29.0f);
    const float column=inner/float(scene_count+2);
    const auto cell=[&](int i,float y,const std::string& text,bool muted){
        ui::label_fitted(r,{x+column*i+3,y+2,column-6,row_height-3},text,0.94f,theme_,muted);
    };
    cell(0,table_y,"PROFILE",true);
    for(int i=0;i<scene_count;++i) cell(i+1,table_y,std::string(scene_names[i])+" FPS",true);
    cell(scene_count+1,table_y,"WORST P90 MS",true);
    int row=1;
    for(const auto& trial:graphics_benchmark_.trials()){
        const float y=table_y+row_height*row++;
        if(&trial==best) r.draw_rect({x,y,inner,row_height},{0.075f,0.15f,0.18f,1});
        cell(0,y,profile_name(trial.profile),false);
        for(int i=0;i<scene_count;++i){
            const double ms=trial.scenes[i].mean_ms;
            cell(i+1,y,trial.available && ms>0.0?number(1000.0/ms,0):"-",!trial.available);
        }
        cell(scene_count+1,y,trial.available?number(trial.worst_p90(),1):"UNAVAILABLE",!trial.available);
    }
    if(ui::button(r,ui_input_,{x+inner-180,footer,180,31},"APPLY RECOMMENDATION",theme_,same_size)){
        graphics_=best->settings;
        persist_preferences();
    }
    if(preferences_save_failed_)
        ui::label_fitted(r,{x,panel.y+panel.h-13,inner,11},"APPLIED FOR THIS SESSION. PREFERENCES COULD NOT BE SAVED.",0.78f,theme_);
}

bool GameApp::render_benchmark_scene(Renderer& r){
    static const TestColony colony;
    const auto& trial=graphics_benchmark_.current();
    const auto scene=graphics_benchmark_.scene();
    const bool rendered=colony.draw(r,scene,trial.settings.path!=GraphicsPath::Classic);
    r.set_2d(ui_scale_);
    const float width=std::min(480.0f,ui_width(r)-32.0f);
    const Rect panel{16,16,width,81};
    ui::panel(r,panel,theme_);
    const std::string title=std::string("BENCHMARK / ")+profile_name(trial.profile)+" / "+scene_names[static_cast<int>(scene)];
    ui::label_fitted(r,{panel.x+10,panel.y+8,width-20,15},title,1.05f,theme_);
    r.draw_rect({panel.x+10,panel.y+30,width-20,5},theme_.border);
    r.draw_rect({panel.x+10,panel.y+30,(width-20)*graphics_benchmark_.progress(),5},theme_.accent);
    if(ui::button(r,ui_input_,{panel.x+10,panel.y+44,114,27},"CANCEL / ESC",theme_))
        graphics_benchmark_.stop(BenchmarkStatus::Cancelled);
    return rendered;
}

} // namespace game
