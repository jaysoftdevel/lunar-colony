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

void GameApp::render_build_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    // Reserve the right side for the two-state view selector. Build objects
    // stay horizontally arranged in the remaining area.
    constexpr float view_area_w=170.0f;
    const float build_area_w=std::max(330.0f,logical_w-view_area_w-18.0f);
    const auto& items=build_catalog_.items();
    const float total=items.empty()?0.0f:items.size()*BUILD_ICON_SIZE+(items.size()-1)*BUILD_ICON_GAP;
    const float start_x=std::max(12.0f,(build_area_w-total)*0.5f);
    const float icon_y=y+(BUILD_BAR_H-BUILD_ICON_SIZE)*0.5f;

    int hovered=-1;
    for(size_t i=0;i<items.size();++i){
        const Rect box={start_x+(BUILD_ICON_SIZE+BUILD_ICON_GAP)*(float)i,icon_y,BUILD_ICON_SIZE,BUILD_ICON_SIZE};
        const bool hover=box.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        const bool selected=(int)i==build_catalog_.selected_index();
        if(hover) hovered=(int)i;
        const Color fill=hover?Color{0.12f,0.16f,0.19f,1.0f}:Color{0.065f,0.08f,0.10f,1.0f};
        r.draw_rect(box,fill);
        const Color border=selected?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        const float bw=selected?3.0f:1.5f;
        r.draw_line({box.x,box.y},{box.x+box.w,box.y},border,bw);
        r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},border,bw);
        r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},border,bw);
        r.draw_line({box.x,box.y+box.h},{box.x,box.y},border,bw);
        render_build_icon(r,items[i].icon,{box.x+8,box.y+8,box.w-16,box.h-16},selected?theme_.accent:theme_.text);
        if(hover && ui_input_.left_pressed){
            const int before=build_catalog_.selected_index();
            build_catalog_.select((int)i);
            if(build_catalog_.selected_index()!=before){
                placement_rotation_degrees_=0.0f;
                corridor_build_start_.reset();
            }
            clear_world_selection();
            corridor_link_source_=-1;
        }
    }

    render_view_selector(r,y,logical_w);

    if(hovered>=0){
        const auto& item=items[(size_t)hovered];
        const std::string line1=item.name;
        const std::string line2="TYPE " + item.type;
        const float tw=std::max(168.0f,std::max(text_width(line1,0.78f*GAME_FONT_BOOST,theme_),text_width(line2,0.60f*GAME_FONT_BOOST,theme_))+22.0f);
        const float tx=std::clamp((float)ui_input_.mouse_x-tw*0.5f,8.0f,logical_w-tw-8.0f);
        const Rect tip={tx,y-60.0f,tw,52.0f};
        ui::panel(r,tip,theme_);
        game_label(r,{tip.x+10,tip.y+5},line1,0.78f,theme_);
        game_label(r,{tip.x+10,tip.y+25},line2,0.60f,theme_,true);
    }
}

} // namespace game
