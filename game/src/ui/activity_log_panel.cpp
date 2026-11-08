#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
using namespace layout;

namespace {
constexpr float kLogHeader=28.f,kLogFooter=42.f,kLogRow=24.f;
}

bool GameApp::activity_log_visible() const {
    return screen_==Screen::Playfield && first_person_mode_==FirstPersonMode::Off &&
           (active_game_phase_!=GamePhase::Planning || planning_.view==PlanningView::Site);
}

Rect GameApp::activity_log_panel_rect() const {
    const float w=viewport_width_/ui_scale_,h=viewport_height_/ui_scale_;
    const float bottom=h-BUILD_BAR_H-PLAYFIELD_MARGIN;
    const float available=std::max(0.f,bottom-STATUS_BAR_H-PLAYFIELD_MARGIN);
    const int content_rows=static_cast<int>(std::clamp<std::size_t>(activity_log_.entries().size(),1,8));
    const int rows=activity_log_expanded_?std::clamp((int)((available*.55f-kLogHeader-kLogFooter)/kLogRow),0,content_rows):0;
    if(rows==0)return {};
    const float height=kLogHeader+kLogFooter+rows*kLogRow;
    return {PLAYFIELD_MARGIN,bottom-height,std::min(480.f,w*.48f),height};
}

bool GameApp::keep_clear_of_activity_log(Rect& panel) const {
    if(!activity_log_visible())return true;
    const auto dock=activity_log_panel_rect();
    constexpr float gap=6.f;
    const float right=dock.x+dock.w+gap;
    if(dock.h<=0.f || panel.x>=right || panel.x+panel.w<=dock.x-gap ||
       panel.y>=dock.y+dock.h+gap || panel.y+panel.h<=dock.y-gap)return true;
    const float above=dock.y-panel.h-gap;
    if(above>=STATUS_BAR_H+gap){
        panel.y=above;
        return true;
    }
    if(right+panel.w>viewport_width_/ui_scale_-gap)return false;
    panel.x=right;
    return true;
}

bool GameApp::mouse_over_activity_log(const InputState& input) const {
    if(!activity_log_visible())return false;
    const auto x=(float)std::lround(input.mouse_x/ui_scale_),y=(float)std::lround(input.mouse_y/ui_scale_);
    const auto dock=activity_log_panel_rect();
    return activity_log_button_rect().contains(x,y) || (dock.h>0.f && dock.contains(x,y));
}

bool GameApp::debug_frame_visible() const {
    return build::debug_features && screen_==Screen::Playfield && active_game_phase_!=GamePhase::Planning &&
           debug_mode_active() && !astronaut_test_active() && first_person_mode_!=FirstPersonMode::Active;
}

Rect GameApp::debug_frame_rect() const {
    const float w=viewport_width_/ui_scale_;
    const float bottom=build_queue_panel_rect().y-8.f;
    return {w-360.f-PLAYFIELD_MARGIN,bottom-DEBUG_FRAME_H,360.f,DEBUG_FRAME_H};
}

bool GameApp::mouse_over_playfield_overlay(const InputState& input) const {
    const float x=(float)std::lround(input.mouse_x/ui_scale_),y=(float)std::lround(input.mouse_y/ui_scale_);
    return mouse_over_build_queue(input) || mouse_over_activity_log(input) ||
           (debug_frame_visible() && debug_frame_rect().contains(x,y)) ||
           (progress_tree_visible() && first_person_mode_==FirstPersonMode::Off && progress_tree_panel_rect().contains(x,y));
}

void GameApp::render_activity_log(Renderer& r){
    const Rect panel=activity_log_panel_rect();
    const int rows=std::max(0,(int)((panel.h-kLogHeader-kLogFooter)/kLogRow));
    const auto& entries=activity_log_.entries();
    const bool expanded=activity_log_expanded_ && rows>0;
    if(ui::button(r,ui_input_,activity_log_button_rect(),expanded?"LOG -":"LOG +",theme_)){
        activity_log_expanded_=!activity_log_expanded_;
        selection_marquee_active_=false;
        selection_marquee_dragged_=false;
        return;
    }
    if(!expanded)return;
    hud_panel(r,panel,theme_);
    game_label_fitted(r,{panel.x+11,panel.y+6,panel.w-22,17},
                      "ACTIVITY LOG  ["+std::to_string(entries.size())+"]",.52f,theme_);

    const auto capacity=(std::size_t)rows;
    const auto count=entries.size();
    const auto max_offset=count>capacity?count-capacity:0;
    if(activity_log_offset_>0 && count>activity_log_seen_count_)
        activity_log_offset_+=count-activity_log_seen_count_;
    activity_log_seen_count_=count;
    activity_log_offset_=std::min(activity_log_offset_,max_offset);
    const auto older=[&](std::size_t step){activity_log_offset_=std::min(activity_log_offset_+step,max_offset);};
    const auto newer=[&](std::size_t step){activity_log_offset_-=std::min(activity_log_offset_,step);};
    if(mouse_over_activity_log(input_)){
        if(ui_input_.wheel>0.f)older(1);
        else if(ui_input_.wheel<0.f)newer(1);
    }

    const auto end=count-activity_log_offset_;
    const auto start=end>capacity?end-capacity:0;
    for(std::size_t i=start;i<end;++i){
        const auto& entry=entries[i];
        const float y=panel.y+kLogHeader+kLogRow*(float)(i-start);
        const bool warning=entry.event==LogEvent::Warning || entry.event==LogEvent::Incident;
        if(warning)r.draw_rect({panel.x+5,y+1,panel.w-10,kLogRow-2},{.21f,.13f,.09f,.40f});
        game_label_fitted(r,{panel.x+11,y+5,panel.w-22,14},log_timestamp(entry.seconds,false)+"  "+
            log_event_name(entry.event)+"  "+entry.message,.44f,theme_,entry.event==LogEvent::Stopped);
    }
    if(entries.empty())
        game_label_fitted(r,{panel.x+11,panel.y+36,panel.w-22,14},"NO ACTIVITY RECORDED",.44f,theme_,true);

    const float footer=panel.y+panel.h-kLogFooter;
    const float button_w=(panel.w-26.f)/3.f;
    if(ui::button(r,ui_input_,{panel.x+6,footer,button_w,23},"OLDER",theme_,activity_log_offset_<max_offset))older(capacity);
    if(ui::button(r,ui_input_,{panel.x+13+button_w,footer,button_w,23},"NEWER",theme_,activity_log_offset_>0))newer(capacity);
    if(ui::button(r,ui_input_,{panel.x+20+2*button_w,footer,button_w,23},"EXPORT LOG",theme_,!entries.empty()))
        export_activity_log();
    const std::string status=activity_log_export_status_.empty()?
        std::to_string(count?start+1:0)+"-"+std::to_string(end)+" / "+std::to_string(count):
        activity_log_export_status_;
    game_label_fitted(r,{panel.x+8,footer+25,panel.w-16,12},status,.41f,theme_,true);
}
} // namespace game
