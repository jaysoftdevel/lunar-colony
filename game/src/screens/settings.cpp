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

void GameApp::ensure_display_modes(){
    if(!display_modes_.empty()) return;

    const DisplayMode current=app_.current_display_mode();
    const auto supported=app_.display_modes();

    for(const auto& common:COMMON_RESOLUTIONS){
        const auto it=std::find_if(supported.begin(),supported.end(),[&](const DisplayMode& mode){
            return mode.width==common.width && mode.height==common.height;
        });
        if(it!=supported.end()) display_modes_.push_back(*it);
    }

    if(current.width>0 && current.height>0){
        const bool already_listed=std::any_of(display_modes_.begin(),display_modes_.end(),[&](const DisplayMode& mode){
            return mode.width==current.width && mode.height==current.height;
        });
        if(!already_listed) display_modes_.push_back(current);
    }

    if(display_modes_.empty()) display_modes_.push_back(current.width>0?current:DisplayMode{1600,900,0});

    display_mode_index_=0;
    for(size_t i=0;i<display_modes_.size();++i){
        if(display_modes_[i].width==current.width && display_modes_[i].height==current.height){
            display_mode_index_=(int)i;
            break;
        }
    }
}

void GameApp::render_settings(Renderer& r){
    ensure_display_modes();
    ui::title(r,{32,24},"SETTINGS",theme_);

    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float w=std::min(820.0f,logical_w-2.0f*SCREEN_MARGIN);
    const float h=std::min(300.0f,std::max(286.0f,logical_h-92.0f));
    Rect p={(logical_w-w)*0.5f,70,w,h};
    ui::panel(r,p,theme_);

    const float inner_x=p.x+PANEL_PADDING;
    const float inner_y=p.y+18.0f;
    const float inner_w=p.w-2.0f*PANEL_PADDING;
    const float column_gap=SECTION_GAP+4.0f;
    const float column_w=(inner_w-column_gap)*0.5f;
    const float left_x=inner_x;
    const float right_x=inner_x+column_w+column_gap;
    const float separator_x=inner_x+column_w+column_gap*0.5f;

    ui::label(r,{left_x,inner_y},"DISPLAY",1.18f,theme_);
    ui::label(r,{left_x,inner_y+39},"FULLSCREEN",0.92f,theme_);
    ui::label(r,{left_x+column_w-26,inner_y+39},"ON",0.92f,theme_);

    ui::label(r,{left_x,inner_y+78},"UI SCALE",0.92f,theme_);
    const float small_button=38.0f;
    const float control_right=left_x+column_w;
    if(ui::button(r,ui_input_,{control_right-150,inner_y+66,small_button,34},"<",theme_) && ui_scale_index_>0){
        --ui_scale_index_;
        ui_scale_=UI_SCALES[ui_scale_index_];
    }
    std::ostringstream scale;
    scale<<(int)std::lround(UI_SCALES[ui_scale_index_]*100.0f)<<"%";
    ui::label(r,{control_right-99,inner_y+78},scale.str(),0.94f,theme_);
    if(ui::button(r,ui_input_,{control_right-small_button,inner_y+66,small_button,34},">",theme_) && ui_scale_index_<UI_SCALE_COUNT-1){
        ++ui_scale_index_;
        ui_scale_=UI_SCALES[ui_scale_index_];
    }

    ui::label(r,{left_x,inner_y+119},"RESOLUTION",0.92f,theme_);
    bool resolution_changed=false;
    if(ui::button(r,ui_input_,{control_right-190,inner_y+107,small_button,34},"<",theme_) && display_mode_index_>0){
        --display_mode_index_;
        resolution_changed=true;
    }
    if(ui::button(r,ui_input_,{control_right-small_button,inner_y+107,small_button,34},">",theme_) && display_mode_index_<(int)display_modes_.size()-1){
        ++display_mode_index_;
        resolution_changed=true;
    }
    const auto& mode=display_modes_[(size_t)std::max(0,display_mode_index_)];
    std::ostringstream resolution;
    resolution<<mode.width<<" X "<<mode.height;
    ui::label(r,{control_right-140,inner_y+119},resolution.str(),0.86f,theme_);

    if(resolution_changed) app_.set_display_mode(mode.width,mode.height);
    if(ui::button(r,ui_input_,{control_right-112,inner_y+151,112,34},"APPLY",theme_)){
        app_.set_display_mode(mode.width,mode.height);
    }
    ui::label(r,{left_x,inner_y+158},"COMMON SUPPORTED MODES",0.72f,theme_,true);

    r.draw_line({separator_x,p.y+16},{separator_x,p.y+p.h-58},theme_.border,1.0f);

    ui::label(r,{right_x,inner_y},"GAMEPLAY",1.18f,theme_);
    ui::label(r,{right_x,inner_y+39},"DEBUG FRAME",0.92f,theme_);
    const std::string debug_label=debug_frame_enabled_?"ON":"OFF";
    if(ui::button(r,ui_input_,{right_x+column_w-82,inner_y+27,82,34},debug_label,theme_)){
        debug_frame_enabled_=!debug_frame_enabled_;
    }
    ui::label(r,{right_x,inner_y+82},"UI SCALE APPLIES TO ALL HUD",0.76f,theme_,true);
    ui::label(r,{right_x,inner_y+109},"CROSSHAIR ONLY OVER PLAYFIELD",0.76f,theme_,true);

    constexpr float back_w=130.0f;
    constexpr float back_h=38.0f;
    if(ui::button(r,ui_input_,{p.x+PANEL_PADDING,p.y+p.h-PANEL_PADDING-back_h,back_w,back_h},"BACK",theme_)){
        screen_=Screen::MainMenu;
    }
}

} // namespace game
