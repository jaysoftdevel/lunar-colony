#include "game/game_app.hpp"
#include "game/systems/game_progress.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>

namespace game {
using namespace eng;
using namespace layout;
namespace {
constexpr float header_height=26.f;
constexpr float compact_threshold=680.f;
}

bool GameApp::progress_tree_visible() const {
    return screen_==Screen::Playfield && active_game_phase_!=GamePhase::Planning && !debug_mode_active();
}

Rect GameApp::progress_tree_panel_rect() const {
    const float width=viewport_width_/ui_scale_,height=viewport_height_/ui_scale_;
    const float w=std::min(386.f,width*.40f);
    const float h=!progress_tree_expanded_?54.f:height<compact_threshold?192.f:272.f;
    return {width-w-PLAYFIELD_MARGIN,STATUS_BAR_H+34.f,w,h};
}

void GameApp::render_progress_tree(Renderer& r){
    if(!progress_tree_visible())return;
    const auto progress=game_progress_snapshot();
    const Rect panel=progress_tree_panel_rect();
    hud_panel(r,panel,theme_);
    auto header_theme=theme_;
    header_theme.panel2={0,0,0,0};
    const bool can_toggle=first_person_mode_==FirstPersonMode::Off;
    if(ui::button(r,ui_input_,{panel.x,panel.y,panel.w,header_height},
                  progress_tree_expanded_?"-  COLONY PROGRESS":"+  COLONY PROGRESS",header_theme,can_toggle)){
        progress_tree_expanded_=!progress_tree_expanded_;
        selection_marquee_active_=selection_marquee_dragged_=false;
        return;
    }

    if(!progress_tree_expanded_){
        std::string target="READY FOR HUMAN CREW";
        for(const auto& stage:progress.stages){
            if(stage.complete)continue;
            const auto next=std::find_if(stage.items.begin(),stage.items.end(),[](const auto& item){return !item.complete;});
            if(next!=stage.items.end())target="STAGE "+std::to_string(stage.number)+" / "+next->label;
            break;
        }
        game_label_fitted(r,{panel.x+9,panel.y+30,panel.w-18,18},target,.48f,theme_);
        return;
    }

    const bool compact=panel.h<220.f;
    const float left=panel.x+9.f,top=panel.y+35.f;
    const float column=compact?(panel.w-26.f)*.5f:panel.w-18.f;
    float x=left,y=top;
    for(const auto& stage:progress.stages){
        if(compact && stage.number==2){ x=left+column+8.f; y=top; }
        const bool active=!progress.ready_for_human_crew && stage.number==progress.active_stage;
        const std::string title="STAGE "+std::to_string(stage.number)+"  "+stage.title+(stage.complete?" [DONE]":"");
        game_label_fitted(r,{x,y,column,14},title,.46f,theme_,!active&&!stage.complete);
        y+=15.f;
        for(const auto& item:stage.items){
            const auto color=item.complete?Color{.27f,.92f,.58f,1}:theme_.muted;
            r.draw_rect({x+1,y+3,5,5},color);
            game_label_fitted(r,{x+11,y,column-11,12},item.label,.37f,theme_,!item.complete);
            y+=13.f;
        }
        y+=3.f;
    }
    const Rect milestone{left,panel.y+panel.h-24.f,panel.w-18.f,18.f};
    if(progress.ready_for_human_crew)r.draw_rect(milestone,{.08f,.28f,.20f,kHudOpacity});
    game_label_fitted(r,{milestone.x+4,milestone.y+2,milestone.w-8,14},
                      progress.ready_for_human_crew?"READY FOR HUMAN CREW":"MILESTONE: READY FOR HUMAN CREW",
                      .43f,theme_,!progress.ready_for_human_crew);
}
} // namespace game
