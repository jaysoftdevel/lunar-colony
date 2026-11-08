#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;

void GameApp::render_debug_frame(Renderer& r){
    const Rect frame=debug_frame_rect();

    r.draw_rect(frame,kHudFill);
    r.draw_line({frame.x,frame.y},{frame.x+frame.w,frame.y},theme_.accent,2.0f);
    r.draw_line({frame.x+frame.w,frame.y},{frame.x+frame.w,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x+frame.w,frame.y+frame.h},{frame.x,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x,frame.y+frame.h},{frame.x,frame.y},theme_.border,1.0f);


    const auto game_label=[&](Renderer& renderer,Vec2 pos,const std::string& text,float scale,const ui::Theme& theme,bool muted=false){
        layout::game_label_fitted(renderer,{pos.x,pos.y,std::max(0.0f,frame.x+frame.w-pos.x-12.0f),14.0f},text,scale,theme,muted);
    };
    std::ostringstream fps,playfield,camera_a,camera_b,cpu;
    fps<<"FPS "<<(int)std::round(fps_);
    playfield<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    camera_a<<"TILT "<<(int)camera_.elevation_degrees()<<"  ROT "<<(int)camera_.azimuth_degrees();
    camera_b<<"ZOOM "<<(int)camera_.distance()<<"  CENTER "
            <<std::fixed<<std::setprecision(1)<<camera_.target().x<<" / "<<camera_.target().z;
    cpu<<"CPU LANES "<<eng::cpu_parallel_lanes()<<"  WORKERS "
       <<eng::TaskPool::shared().background_workers()<<" + MAIN";
    std::string gpu=r.gpu_renderer();
    if(gpu.size()>42) gpu=gpu.substr(0,39)+"...";

    const char* title=active_game_phase_!=GamePhase::Debug?"DEBUG":
        stage3_mode_==Stage3Mode::Template?"DEBUG / TEMPLATE":"DEBUG / SANDBOX";
    game_label(r,{frame.x+12,frame.y+8},title,0.66f,theme_,true);
    game_label(r,{frame.x+12,frame.y+29},"VERSION " + std::string(version::current),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+50},fps.str(),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+71},playfield.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+94},camera_a.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+117},camera_b.str(),0.58f,theme_);
    game_label(r,{frame.x+12,frame.y+140},cpu.str(),0.56f,theme_);
    game_label(r,{frame.x+12,frame.y+163},"GPU HW " + gpu,0.54f,theme_);

    game_label(r,{frame.x+12,frame.y+188},"WORLD LAYOUT",0.56f,theme_,true);
    const char* layout_label=mesh_layout_active()?"MESHED":"FULL GRAPHICS";
    if(ui::button(r,ui_input_,{frame.x+222,frame.y+181,126,32},layout_label,theme_))
        toggle_world_layout();
}

} // namespace game
