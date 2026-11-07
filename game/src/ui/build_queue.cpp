#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/world_labels.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
using namespace layout;

namespace {
constexpr float kHeaderHeight=30.f;
constexpr float kFooterHeight=30.f;
constexpr float kRowHeight=38.f;
constexpr float kPadding=8.f;
constexpr int kMaxRows=8;

Rect queue_header(Rect panel){return {panel.x,panel.y,panel.w,kHeaderHeight};}

int visible_rows(Rect panel){
    return std::max(0,static_cast<int>((panel.h-kHeaderHeight-kFooterHeight-2*kPadding)/kRowHeight));
}

bool arrow(Renderer& r,const InputState& input,Rect rect,int direction,const ui::Theme& theme,bool enabled){
    const bool clicked=ui::button(r,input,rect,"",theme,enabled);
    const float x=rect.x+rect.w*.5f,y=rect.y+rect.h*.5f;
    const Color ink=enabled?theme.text:theme.muted;
    r.draw_line({x-4,y-2.f*direction},{x,y+2.f*direction},ink,1.7f);
    r.draw_line({x,y+2.f*direction},{x+4,y-2.f*direction},ink,1.7f);
    return clicked;
}
}

Rect GameApp::build_queue_panel_rect() const {
    const float width=viewport_width_/ui_scale_,height=viewport_height_/ui_scale_;
    const float w=std::min(420.f,width*.40f);
    const float y=STATUS_BAR_H+6.f;
    const float available=height-BUILD_BAR_H-PLAYFIELD_MARGIN-y;
    const float padding=kHeaderHeight+kFooterHeight+2*kPadding;
    const int rows=std::clamp(static_cast<int>((available-padding)/kRowHeight),0,kMaxRows);
    const float h=build_queue_expanded_ && rows>0?padding+rows*kRowHeight:kHeaderHeight;
    return {width-w-PLAYFIELD_MARGIN,y,w,h};
}

bool GameApp::mouse_over_build_queue(const InputState& input) const {
    if(screen_!=Screen::Playfield || active_game_phase_==GamePhase::Planning || first_person_mode_!=FirstPersonMode::Off)
        return false;
    return build_queue_panel_rect().contains(static_cast<float>(std::lround(input.mouse_x/ui_scale_)),
                                             static_cast<float>(std::lround(input.mouse_y/ui_scale_)));
}

void GameApp::render_build_queue(Renderer& r){
    using namespace world_labels;
    const Rect panel=build_queue_panel_rect();
    const auto queue=construction_queue();
    const auto count=static_cast<int>(queue.size());
    const std::string title=std::string(build_queue_expanded_?"-  BUILD ORDER  [":"+  BUILD ORDER  [")+std::to_string(count)+"]";
    if(build_queue_expanded_ && visible_rows(panel)>0) ui::panel(r,panel,theme_);
    if(ui::button(r,ui_input_,queue_header(panel),title,theme_)){
        build_queue_expanded_=!build_queue_expanded_;
        selection_marquee_active_=false;
        selection_marquee_dragged_=false;
        return;
    }
    const int capacity=visible_rows(panel);
    if(!build_queue_expanded_ || capacity<1) return;

    const int max_scroll=std::max(0,count-capacity);
    if(mouse_over_build_queue(input_) && ui_input_.wheel!=0.f)
        build_queue_scroll_+=ui_input_.wheel<0.f?1:-1;
    build_queue_scroll_=std::clamp(build_queue_scroll_,0,max_scroll);
    if(queue.empty()){
        game_label_fitted(r,{panel.x+12,panel.y+44,panel.w-24,20},"NO PENDING CONSTRUCTION",.54f,theme_,true);
        return;
    }

    const auto name=[&](const BuildQueueItem& job)->std::string{
        const auto i=job.target.index;
        switch(job.target.kind){
            case ConstructionTargetKind::Habitat: return habitat_instance_name(placed_habitats_[i]);
            case ConstructionTargetKind::Resource: return resource_instance_name(placed_resources_[i]);
            case ConstructionTargetKind::Corridor: return "CORRIDOR #"+std::to_string(job.sequence);
            case ConstructionTargetKind::Utility:
                return std::string(utility_kind_name(utility_network_.connections()[i].kind))+" #"+std::to_string(job.sequence);
            case ConstructionTargetKind::Walkway: return "WALKWAY #"+std::to_string(job.sequence);
            default: return "BLUEPRINT";
        }
    };
    const int first=build_queue_scroll_;
    const int last=std::min(count,first+capacity);
    const float left=panel.x+kPadding;
    const float right=panel.x+panel.w-kPadding;
    const float badge_x=right-142.f;
    for(int row=first;row<last;++row){
        const auto& job=queue[static_cast<std::size_t>(row)];
        const float y=panel.y+kHeaderHeight+kPadding+(row-first)*kRowHeight;
        r.draw_rect({left,y,right-left,kRowHeight-3},row==0?Color{.085f,.17f,.20f,1}:theme_.panel2);
        game_label_fitted(r,{left+6,y+5,30,15},std::to_string(row+1),.53f,theme_,true);
        game_label_fitted(r,{left+37,y+4,badge_x-left-42,15},name(job),.53f,theme_);

        const auto step=construction_target_step(job.target);
        const auto steps=construction_target_total_steps(job.target);
        const bool ready=construction_target_dependencies_ready(job.target);
        const bool working=std::any_of(builder_robots_.begin(),builder_robots_.end(),[&](const auto& rover){
            return rover.target.same_as(job.target) && rover.mode!=BuilderRobotMode::Idle;
        }) || (astronaut_test_active() && std::any_of(test_astronauts_.begin(),test_astronauts_.end(),[&](const auto& actor){
            return job.sequence && actor.construction_job==job.sequence;
        }));
        const std::string state=!ready?"WAITING FOR PREREQUISITES":
            !construction_queue_allows(job)?"WAITING FOR EARLIER PRIORITY":
            working?"ASSIGNED   "+std::to_string(step)+"/"+std::to_string(steps):
            std::to_string(step)+"/"+std::to_string(steps)+" STAGES";
        game_label_fitted(r,{left+37,y+21,badge_x-left-42,11},state,.43f,theme_,true);

        const Rect badge{badge_x,y+8,74,19};
        const auto rank=construction_priority_rank(job.target);
        r.draw_rect(badge,job.manual_priority?Color{.32f,.22f,.085f,1}:
            job.automatic_priority?Color{.10f,.26f,.28f,1}:Color{.07f,.10f,.12f,1});
        const std::string priority=job.manual_priority?"PRIO "+std::to_string(rank+1):
            job.automatic_priority?"AUTO":"NORMAL";
        game_label_fitted(r,{badge.x+5,badge.y+4,badge.w-10,12},priority,.49f,theme_,!job.prioritized());

        const bool up=job.sequence && row>0 && queue[static_cast<std::size_t>(row-1)].sequence;
        const bool down=job.sequence && row+1<count && queue[static_cast<std::size_t>(row+1)].sequence;
        const auto move=[&](int direction){
            if(!move_construction_order(job.sequence,direction)) return;
            build_queue_scroll_=std::clamp(build_queue_scroll_,row+direction-capacity+1,row+direction);
            build_queue_scroll_=std::clamp(build_queue_scroll_,0,max_scroll);
        };
        if(arrow(r,ui_input_,{right-62,y+4,28,27},-1,theme_,up)) move(-1);
        if(arrow(r,ui_input_,{right-30,y+4,28,27},1,theme_,down)) move(1);
    }

    const float footer=panel.y+panel.h-kFooterHeight;
    game_label_fitted(r,{left+6,footer+7,panel.w-166,16},
        std::to_string(first+1)+"-"+std::to_string(last)+" / "+std::to_string(count),.47f,theme_,true);
    if(max_scroll>0){
        if(ui::button(r,ui_input_,{right-142,footer,66,24},"PREV",theme_,build_queue_scroll_>0))
            build_queue_scroll_=std::max(0,build_queue_scroll_-capacity);
        if(ui::button(r,ui_input_,{right-70,footer,66,24},"NEXT",theme_,build_queue_scroll_<max_scroll))
            build_queue_scroll_=std::min(max_scroll,build_queue_scroll_+capacity);
    }
}
} // namespace game
