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
    const float group_w=default_w+full_w+gap;
    const float x=logical_w-group_w-margin;

    game_label(r,{x,y+6.0f},"VIEW",0.48f,theme_,true);
    const float by=y+35.0f;

    struct ViewButton { HabitatViewMode mode; const char* label; float width; };
    const ViewButton buttons[]={
        {HabitatViewMode::Default,"DEFAULT",default_w},
        {HabitatViewMode::Full,"FULL",full_w}
    };

    float bx=x;
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
