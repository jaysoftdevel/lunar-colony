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
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    constexpr float frame_w=292.0f;
    constexpr float frame_h=154.0f;
    const Rect frame={logical_w-frame_w-PLAYFIELD_MARGIN,logical_h-BUILD_BAR_H-frame_h-PLAYFIELD_MARGIN,frame_w,frame_h};

    r.draw_rect(frame,{0.025f,0.035f,0.045f,0.92f});
    r.draw_line({frame.x,frame.y},{frame.x+frame.w,frame.y},theme_.accent,2.0f);
    r.draw_line({frame.x+frame.w,frame.y},{frame.x+frame.w,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x+frame.w,frame.y+frame.h},{frame.x,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x,frame.y+frame.h},{frame.x,frame.y},theme_.border,1.0f);

    std::ostringstream fps,playfield,camera_a,camera_b;
    fps<<"FPS "<<(int)std::round(fps_);
    playfield<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    camera_a<<"TILT "<<(int)camera_.elevation_degrees()<<"  ROT "<<(int)camera_.azimuth_degrees();
    camera_b<<"ZOOM "<<(int)camera_.distance()<<"  CENTER "
            <<std::fixed<<std::setprecision(1)<<camera_.target().x<<" / "<<camera_.target().z;

    game_label(r,{frame.x+12,frame.y+8},"DEBUG",0.66f,theme_,true);
    game_label(r,{frame.x+12,frame.y+29},"VERSION " + std::string(version::current),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+50},fps.str(),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+71},playfield.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+94},camera_a.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+117},camera_b.str(),0.58f,theme_);
}

} // namespace game
