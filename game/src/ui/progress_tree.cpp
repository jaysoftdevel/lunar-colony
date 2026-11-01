#include "game/game_app.hpp"
#include "game/systems/game_progress.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;

Rect GameApp::progress_tree_panel_rect() const {
    const float logical_w=(float)viewport_width_/std::max(0.01f,ui_scale_);
    constexpr float width=430.0f;
    constexpr float height=350.0f;
    return {std::max(PLAYFIELD_MARGIN,logical_w-width-PLAYFIELD_MARGIN),STATUS_BAR_H+40.0f,width,height};
}

void GameApp::render_progress_tree(Renderer& r){
    if(debug_frame_enabled_)return;
    const auto progress=game_progress_snapshot();
    const Rect p=progress_tree_panel_rect();
    r.draw_rect(p,{0.025f,0.035f,0.045f,0.93f});
    r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2.0f);
    game_label(r,{p.x+14,p.y+9},"COLONY PROGRESS",0.65f,theme_);
    game_label(r,{p.x+14,p.y+28},progress.ready_for_human_crew?"MILESTONE COMPLETE":"CURRENT STAGE  "+std::to_string(progress.active_stage),0.40f,theme_,true);

    float y=p.y+48.0f;
    for(const auto& stage:progress.stages){
        const bool active=!progress.ready_for_human_crew && stage.number==progress.active_stage;
        std::ostringstream title;
        title<<"STAGE "<<stage.number<<"  "<<stage.title;
        if(stage.complete)title<<"  [DONE]";
        game_label_fitted(r,{p.x+14,y,p.w-28,16},title.str(),active?0.49f:0.45f,theme_,!active&&!stage.complete);
        y+=17.0f;
        for(const auto& item:stage.items){
            const Color mark=item.complete?Color{0.27f,0.92f,0.58f,1.0f}:Color{0.55f,0.64f,0.69f,1.0f};
            const float bx=p.x+17.0f,by=y+1.0f;
            r.draw_line({bx,by},{bx+9,by},mark,1.4f);r.draw_line({bx+9,by},{bx+9,by+9},mark,1.4f);
            r.draw_line({bx+9,by+9},{bx,by+9},mark,1.4f);r.draw_line({bx,by+9},{bx,by},mark,1.4f);
            if(item.complete){r.draw_line({bx+2,by+4.5f},{bx+4.2f,by+7},mark,1.7f);r.draw_line({bx+4.2f,by+7},{bx+7.8f,by+2},mark,1.7f);}
            game_label_fitted(r,{p.x+33,y-2,p.w-49,14},item.label,0.37f,theme_,!item.complete);
            y+=15.0f;
        }
        y+=4.0f;
    }

    const Rect milestone{p.x+14,p.y+p.h-39,p.w-28,27};
    const Color milestone_color=progress.ready_for_human_crew?Color{0.08f,0.28f,0.20f,0.96f}:Color{0.06f,0.09f,0.11f,0.96f};
    r.draw_rect(milestone,milestone_color);
    game_label_fitted(r,{milestone.x+9,milestone.y+5,milestone.w-18,17},progress.ready_for_human_crew?"READY FOR HUMAN CREW":"MILESTONE: READY FOR HUMAN CREW",0.45f,theme_,!progress.ready_for_human_crew);
}

} // namespace game
