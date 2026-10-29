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

void GameApp::render_view_selector(Renderer& r,float y,float logical_w){
    constexpr float margin=10.0f;
    constexpr float gap=5.0f;
    constexpr float default_w=74.0f;
    constexpr float full_w=58.0f;
    constexpr float button_h=31.0f;
    constexpr float speed_group_w=176.0f;
    constexpr float between_groups=12.0f;
    const float view_group_w=default_w+full_w+gap;
    const float total_w=speed_group_w+between_groups+view_group_w;
    const float x=logical_w-total_w-margin;
    const float by=y+35.0f;

    // Pause plus three deterministic simulation rates. Camera/POV input remains real-time;
    // colony time, robots, production and construction consume scaled sim time.
    game_label(r,{x,y+6.0f},"GAME SPEED",0.48f,theme_,true);
    const Rect minus{x,by,31.0f,button_h};
    const Rect plus{x+145.0f,by,31.0f,button_h};
    if(ui::button(r,ui_input_,minus,"-",theme_)){
        switch(game_speed_){
            case GameSpeed::Fast: game_speed_=GameSpeed::Quick; break;
            case GameSpeed::Quick: game_speed_=GameSpeed::Normal; break;
            case GameSpeed::Normal: game_speed_=GameSpeed::Pause; break;
            case GameSpeed::Pause: break;
        }
    }
    if(ui::button(r,ui_input_,plus,"+",theme_)){
        switch(game_speed_){
            case GameSpeed::Pause: game_speed_=GameSpeed::Normal; break;
            case GameSpeed::Normal: game_speed_=GameSpeed::Quick; break;
            case GameSpeed::Quick: game_speed_=GameSpeed::Fast; break;
            case GameSpeed::Fast: break;
        }
    }
    const char* speed_name="NORMAL";
    const char* speed_mult="1X";
    if(game_speed_==GameSpeed::Pause){ speed_name="PAUSE"; speed_mult="0X"; }
    if(game_speed_==GameSpeed::Quick){ speed_name="QUICK"; speed_mult="2X"; }
    if(game_speed_==GameSpeed::Fast){ speed_name="FAST"; speed_mult="4X"; }
    const Rect speed_box{x+36.0f,by,104.0f,button_h};
    r.draw_rect(speed_box,{0.055f,0.085f,0.10f,1.0f});
    r.draw_line({speed_box.x,speed_box.y},{speed_box.x+speed_box.w,speed_box.y},theme_.accent,1.2f);
    r.draw_line({speed_box.x+speed_box.w,speed_box.y},{speed_box.x+speed_box.w,speed_box.y+speed_box.h},theme_.border,1.0f);
    r.draw_line({speed_box.x+speed_box.w,speed_box.y+speed_box.h},{speed_box.x,speed_box.y+speed_box.h},theme_.border,1.0f);
    r.draw_line({speed_box.x,speed_box.y+speed_box.h},{speed_box.x,speed_box.y},theme_.border,1.0f);
    std::string speed_label=std::string(speed_name)+"  "+speed_mult;
    const float sw=text_width(speed_label,0.42f*GAME_FONT_BOOST,theme_);
    game_label(r,{speed_box.x+(speed_box.w-sw)*0.5f,speed_box.y+7.0f},speed_label,0.42f,theme_);

    const float view_x=x+speed_group_w+between_groups;
    game_label(r,{view_x,y+6.0f},"VIEW",0.48f,theme_,true);
    struct ViewButton { HabitatViewMode mode; const char* label; float width; };
    const ViewButton buttons[]={
        {HabitatViewMode::Default,"DEFAULT",default_w},
        {HabitatViewMode::Full,"FULL",full_w}
    };

    float bx=view_x;
    for(const auto& b:buttons){
        const Rect rect{bx,by,b.width,button_h};
        const bool active=habitat_view_mode_==b.mode;
        const bool hover=rect.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        r.draw_rect(rect,active?Color{0.10f,0.23f,0.27f,1.0f}:(hover?Color{0.11f,0.14f,0.17f,1.0f}:Color{0.06f,0.075f,0.09f,1.0f}));
        const Color border=active?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        r.draw_line({rect.x,rect.y},{rect.x+rect.w,rect.y},border,active?2.0f:1.0f);
        r.draw_line({rect.x+rect.w,rect.y},{rect.x+rect.w,rect.y+rect.h},border,active?2.0f:1.0f);
        r.draw_line({rect.x+rect.w,rect.y+rect.h},{rect.x,rect.y+rect.h},border,active?2.0f:1.0f);
        r.draw_line({rect.x,rect.y+rect.h},{rect.x,rect.y},border,active?2.0f:1.0f);
        const float label_scale=0.43f;
        const float label_w=text_width(b.label,label_scale*GAME_FONT_BOOST,theme_);
        game_label(r,{rect.x+(rect.w-label_w)*0.5f,rect.y+7.0f},b.label,label_scale,theme_,!active);
        if(hover && ui_input_.left_pressed) habitat_view_mode_=b.mode;
        bx+=b.width+gap;
    }
}

} // namespace game
