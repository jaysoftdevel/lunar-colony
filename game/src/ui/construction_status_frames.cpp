#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/world_labels.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_labels;

namespace {
struct ConstructionFrame {
    ConstructionTargetRef target{};
    Vec3 anchor{};
    std::string title{};
    ResourceBundle needed{};
    ResourceBundle shipped{};
    float progress{0.0f};
    std::uint8_t step{0};
    std::uint8_t total_steps{1};
    float screen_x{0.0f};
    float screen_y{0.0f};
    Rect rect{};
    bool selected{false};
};

bool project_world_to_logical(const Mat4& m,Vec3 p,int viewport_width,int viewport_height,float ui_scale,float& sx,float& sy){
    const float cx=m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12];
    const float cy=m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13];
    const float cw=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];
    if(cw<=0.001f) return false;
    const float nx=cx/cw,ny=cy/cw;
    sx=(nx*0.5f+0.5f)*(float)viewport_width/std::max(0.01f,ui_scale);
    sy=(1.0f-(ny*0.5f+0.5f))*(float)viewport_height/std::max(0.01f,ui_scale);
    return nx>=-1.08f && nx<=1.08f && ny>=-1.08f && ny<=1.08f;
}

Vec3 path_midpoint(const std::vector<Vec3>& path,float lift){
    if(path.empty()) return {0,lift,0};
    if(path.size()==1) return path.front()+Vec3{0,lift,0};
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i) total+=length(path[i+1]-path[i]);
    if(total<=0.001f) return path.front()+Vec3{0,lift,0};
    float remaining=total*0.5f;
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 delta=path[i+1]-path[i];
        const float segment=length(delta);
        if(segment<=0.001f) continue;
        if(remaining<=segment){
            const float t=std::clamp(remaining/segment,0.0f,1.0f);
            return path[i]+delta*t+Vec3{0,lift,0};
        }
        remaining-=segment;
    }
    return path.back()+Vec3{0,lift,0};
}

ResourceBundle shipped_resources(const ConstructionResourcePlan& plan,std::uint8_t step,
                                 const ResourceBundle* delivered,const ResourceBundle& reserved){
    const ResourceBundle remaining=plan.remaining(step);
    ResourceBundle shipped{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        const std::int64_t covered=(delivered?delivered->amount(material):0)+reserved.amount(material);
        shipped.amount(material)=std::min(remaining.amount(material),std::max<std::int64_t>(0,covered));
    }
    return shipped;
}

ResourceBundle remaining_need(const ConstructionResourcePlan& plan,std::uint8_t step,
                              const ResourceBundle* delivered,const ResourceBundle& reserved){
    ResourceBundle need=plan.remaining(step);
    const ResourceBundle shipped=shipped_resources(plan,step,delivered,reserved);
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        need.amount(material)=std::max<std::int64_t>(0,need.amount(material)-shipped.amount(material));
    }
    return need;
}

std::string short_mass(std::int64_t kg){
    std::ostringstream out;
    if(kg>=10000){ out<<std::fixed<<std::setprecision(0)<<(double)kg/1000.0<<"t"; }
    else if(kg>=1000){ out<<std::fixed<<std::setprecision(1)<<(double)kg/1000.0<<"t"; }
    else out<<kg<<"kg";
    return out.str();
}

std::vector<std::string> resource_lines(const char* label,const ResourceBundle& bundle,const char* empty_text){
    std::vector<std::string> tokens;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        const std::int64_t amount=bundle.amount(material);
        if(amount<=0) continue;
        tokens.push_back(std::string(material_resource_symbol(material))+" "+short_mass(amount));
    }
    if(tokens.empty()) return {std::string(label)+"  "+empty_text};
    std::vector<std::string> lines;
    for(std::size_t i=0;i<tokens.size();i+=2){
        std::string line=i==0?std::string(label)+"  ":std::string(std::char_traits<char>::length(label)+2,' ');
        line+=tokens[i];
        if(i+1<tokens.size()) line+="   "+tokens[i+1];
        lines.push_back(std::move(line));
    }
    return lines;
}

bool overlaps(const Rect& a,const Rect& b,float pad=4.0f){
    return a.x < b.x+b.w+pad && a.x+a.w+pad > b.x &&
           a.y < b.y+b.h+pad && a.y+a.h+pad > b.y;
}
}

void GameApp::render_construction_status_frames(Renderer& r){
    // Construction frames explicitly use the same global 2D transform as the
    // rest of the HUD. World projection is converted back to logical UI units,
    // so panel geometry, progress bars, text and spacing all follow UI SCALE.
    r.set_2d(ui_scale_);
    construction_priority_button_rect_={};construction_priority_button_target_={};
    std::vector<ConstructionFrame> frames;
    frames.reserve(placed_habitats_.size()+placed_resources_.size()+
                   corridor_network_.connections().size()+utility_network_.connections().size()+
                   walkway_network_.connections().size());

    auto target_selected=[&](const ConstructionTargetRef& target){
        switch(target.kind){
            case ConstructionTargetKind::Habitat:
                return selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)target.index;
            case ConstructionTargetKind::Resource:
                return selection_kind_==SelectionKind::Resource && selected_resource_==(int)target.index;
            case ConstructionTargetKind::Corridor:
                return selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)target.index;
            case ConstructionTargetKind::Utility:
                return selection_kind_==SelectionKind::Utility && selected_utility_==(int)target.index;
            case ConstructionTargetKind::Walkway:
                return selection_kind_==SelectionKind::Walkway && selected_walkway_==(int)target.index;
            case ConstructionTargetKind::None:
                break;
        }
        return false;
    };

    auto add_frame=[&](ConstructionTargetRef target,Vec3 anchor,std::string title,float progress,
                       const ConstructionResourcePlan& plan,std::uint8_t step){
        if(progress>=1.0f) return;
        ConstructionFrame f;
        f.target=target;
        f.anchor=anchor;
        f.title=std::move(title);
        f.progress=std::clamp(progress,0.0f,1.0f);
        f.step=step;
        f.total_steps=construction_target_total_steps(target);
        f.selected=target_selected(target);
        // Keep the passive blueprint HUD intentionally cheap and quiet. The
        // material ledger is only needed for the blueprint the player actually
        // selected; every other unfinished blueprint renders a progress bar only.
        if(f.selected){
            const ResourceBundle* delivered=construction_site_buffer(target);
            const ResourceBundle reserved=construction_site_reserved(target);
            f.shipped=shipped_resources(plan,step,delivered,reserved);
            f.needed=remaining_need(plan,step,delivered,reserved);
        }
        frames.push_back(std::move(f));
    };

    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        if(h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        add_frame({ConstructionTargetKind::Habitat,i},h.position+Vec3{0,mesh.height+0.85f,0},
                  habitat_instance_name(h),h.construction.progress(),h.resource_plan,h.construction.step);
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(item.construction.complete()) continue;
        const auto& mesh=resource_models_.mesh(item);
        add_frame({ConstructionTargetKind::Resource,i},item.position+Vec3{0,mesh.height+0.85f,0},
                  resource_instance_name(item),item.construction.progress(),item.resource_plan,item.construction.step);
    }
    const auto& corridors=corridor_network_.connections();
    for(std::size_t i=0;i<corridors.size();++i){
        const auto& c=corridors[i];
        if(c.construction.complete()) continue;
        add_frame({ConstructionTargetKind::Corridor,i},path_midpoint(c.path,3.15f),"CORRIDOR #"+std::to_string(i+1),
                  c.construction.progress(),c.resource_plan,c.construction.step);
    }
    const auto& utilities=utility_network_.connections();
    for(std::size_t i=0;i<utilities.size();++i){
        const auto& c=utilities[i];
        if(c.construction.complete()) continue;
        add_frame({ConstructionTargetKind::Utility,i},path_midpoint(c.path,1.65f),"SUPPLY CHANNEL #"+std::to_string(i+1),
                  c.construction.progress(),c.resource_plan,c.construction.step);
    }
    const auto& walkways=walkway_network_.connections();
    for(std::size_t i=0;i<walkways.size();++i){
        const auto& w=walkways[i];
        if(w.construction.complete()) continue;
        add_frame({ConstructionTargetKind::Walkway,i},path_midpoint(w.path,1.80f),"WALKWAY #"+std::to_string(i+1),
                  w.construction.progress(),w.resource_plan,w.construction.step);
    }

    if(frames.empty()) return;
    const float aspect=(float)r.width()/std::max(1,r.height());
    const Mat4 vp=first_person_mode_==FirstPersonMode::Active?first_person_.view_projection(aspect):camera_.view_projection(aspect);
    std::vector<ConstructionFrame> visible;
    visible.reserve(frames.size());
    for(auto& frame:frames){
        if(project_world_to_logical(vp,frame.anchor,r.width(),r.height(),ui_scale_,frame.screen_x,frame.screen_y)) visible.push_back(std::move(frame));
    }
    std::sort(visible.begin(),visible.end(),[](const ConstructionFrame& a,const ConstructionFrame& b){ return a.screen_y<b.screen_y; });

    const float logical_w=ui_width(r),logical_h=ui_height(r);
    const float bottom_guard=first_person_mode_==FirstPersonMode::Active?8.0f:BUILD_BAR_H+8.0f;
    std::vector<Rect> occupied;
    occupied.reserve(visible.size()+3);
    if(first_person_mode_!=FirstPersonMode::Active){
        if(selection_kind_!=SelectionKind::None)occupied.push_back(selection_kind_==SelectionKind::GroundResource && selected_ground_resources_.size()<=1?ground_resource_panel_rect():selection_panel_rect());
        else occupied.push_back({PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,560,130});
        if(debug_frame_enabled_)occupied.push_back({logical_w-360-PLAYFIELD_MARGIN,logical_h-BUILD_BAR_H-198-PLAYFIELD_MARGIN,360,198});
    }
    // Normal gameplay reserves the right-edge progression checklist so floating
    // construction frames never cover objectives or the crew-readiness milestone.
    if(!debug_frame_enabled_)occupied.push_back(progress_tree_panel_rect());
    for(auto& frame:visible){
        const auto needed_lines=frame.selected?resource_lines("NEEDED",frame.needed,"NONE"):std::vector<std::string>{};
        const auto shipped_lines=frame.selected?resource_lines("SHIPPED",frame.shipped,"NONE"):std::vector<std::string>{};
        const float width=frame.selected?255.0f:126.0f;
        const float height=frame.selected?82.0f+18.0f*(float)(needed_lines.size()+shipped_lines.size()):9.0f;
        float x=frame.screen_x-width*0.5f;
        float y=frame.screen_y-height-13.0f;
        x=std::clamp(x,6.0f,std::max(6.0f,logical_w-width-6.0f));
        y=std::clamp(y,STATUS_BAR_H+6.0f,std::max(STATUS_BAR_H+6.0f,logical_h-bottom_guard-height));
        Rect candidate{x,y,width,height};
        if(frame.selected && !occupied.empty() && overlaps(candidate,occupied.front())){
            const float right=occupied.front().x+occupied.front().w+8;
            if(right+width<=logical_w-6)candidate.x=right;
        }

        // Dense construction sites can project several objects onto almost the
        // same screen location. Keep every progress indicator visible, while
        // selected detailed frames reserve their larger footprint.
        for(int attempt=0;attempt<12;++attempt){
            bool hit=false;
            for(const Rect& other:occupied) if(overlaps(candidate,other)){ hit=true; break; }
            if(!hit) break;
            const float direction=(attempt%2==0)?1.0f:-1.0f;
            const float distance=(float)(attempt/2+1)*(height+5.0f);
            candidate.y=std::clamp(y+direction*distance,STATUS_BAR_H+6.0f,
                                   std::max(STATUS_BAR_H+6.0f,logical_h-bottom_guard-height));
        }
        frame.rect=candidate;
        occupied.push_back(candidate);

        const Color track{0.09f,0.13f,0.15f,0.96f};
        const Color bar{1.00f,0.68f,0.12f,0.98f};
        if(!frame.selected){
            // Unselected blueprints intentionally expose no name, stage,
            // percentage or material details: only construction progress.
            r.draw_rect(candidate,track);
            if(frame.progress>0.0f)
                r.draw_rect({candidate.x,candidate.y,candidate.w*frame.progress,candidate.h},bar);
            continue;
        }

        const Color border{0.31f,0.79f,0.93f,0.95f};
        const Color fill{0.020f,0.032f,0.041f,0.93f};
        r.draw_rect(candidate,fill);
        r.draw_line({candidate.x,candidate.y},{candidate.x+candidate.w,candidate.y},border,1.5f);

        std::ostringstream title;
        title<<frame.title<<"   STEP "<<(int)frame.step<<"/"<<(int)frame.total_steps
             <<"   "<<(int)std::lround(frame.progress*100.0f)<<"%";
        const auto rank=construction_priority_rank(frame.target);
        if(rank!=kNoRobotIndex)title<<"   PRIO "<<rank+1;
        else if(construction_ready_connection(frame.target))title<<"   AUTO LINK";
        game_label_fitted(r,{candidate.x+8.0f,candidate.y+6.0f,candidate.w-16.0f,13.0f},title.str(),0.43f,theme_);

        const Rect progress_track{candidate.x+8.0f,candidate.y+23.0f,candidate.w-16.0f,7.0f};
        r.draw_rect(progress_track,track);
        if(frame.progress>0.0f) r.draw_rect({progress_track.x,progress_track.y,progress_track.w*frame.progress,progress_track.h},bar);

        float text_y=candidate.y+36.0f;
        for(const std::string& line:needed_lines){
            game_label_fitted(r,{candidate.x+8.0f,text_y,candidate.w-16.0f,14.0f},line,0.36f,theme_,true);
            text_y+=18.0f;
        }
        for(const std::string& line:shipped_lines){
            game_label_fitted(r,{candidate.x+8.0f,text_y,candidate.w-16.0f,14.0f},line,0.36f,theme_,true);
            text_y+=18.0f;
        }
        construction_priority_button_rect_={candidate.x+8,candidate.y+candidate.h-30,candidate.w-16,22};
        construction_priority_button_target_=frame.target;
        const bool queued=construction_priority_rank(frame.target)!=kNoRobotIndex;
        if(ui::button(r,ui_input_,construction_priority_button_rect_,queued?"REMOVE PRIORITY":"PRIORITIZE BUILD",theme_)){
            toggle_construction_priority(frame.target);selection_marquee_active_=false;selection_marquee_dragged_=false;
        }

    }
}

void GameApp::render_supply_indicators(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    const Mat4 vp=first_person_mode_==FirstPersonMode::Active?first_person_.view_projection(aspect):camera_.view_projection(aspect);
    const float pulse=0.48f+0.52f*(0.5f+0.5f*std::sin(supply_warning_time_*2.0943951f));
    const auto draw=[&](Vec3 position,float height,const SupplyState& supply,bool inside){
        float x=0,y=0;if(!project_world_to_logical(vp,position+Vec3{0,height+1.0f,0},r.width(),r.height(),ui_scale_,x,y))return;
        if(supply.warnings){
            const Color color{1.0f,0.68f,0.10f,pulse};
            r.draw_line({x,y-16},{x-10,y+3},color,2.5f);r.draw_line({x-10,y+3},{x+10,y+3},color,2.5f);r.draw_line({x+10,y+3},{x,y-16},color,2.5f);
            r.draw_line({x,y-10},{x,y-4},color,2.5f);r.draw_line({x,y-1},{x,y},color,2.5f);
        }
        if(inside){
            const std::string label="AIR "+std::to_string((int)std::lround(supply.breathability()))+"%";
            r.draw_rect({x-30,y+7,65,15},{0.02f,0.03f,0.04f,0.90f});
            game_label(r,{x-26,y+8},label,0.40f,theme_);
        }
    };
    const auto priority=[&](ConstructionTargetRef target,Vec3 position,float height){
        if(target.same_as(selected_construction_target()))return; // selected frame already labels its priority
        const auto rank=construction_priority_rank(target);const bool automatic=construction_ready_connection(target);
        if((rank==kNoRobotIndex && !automatic) || construction_target_complete(target))return;
        float x=0,y=0;if(!project_world_to_logical(vp,position+Vec3{0,height+2.0f,0},r.width(),r.height(),ui_scale_,x,y))return;
        const Color c{0.28f,0.88f,1.0f,1};
        r.draw_rect({x+20,y-23,82,24},{0.02f,0.06f,0.09f,0.95f});
        r.draw_line({x+24,y-10},{x+29,y-16},c,2);r.draw_line({x+29,y-16},{x+34,y-10},c,2);
        game_label(r,{x+39,y-17},rank==kNoRobotIndex?"AUTO LINK":"PRIO "+std::to_string(rank+1),0.42f,theme_);
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i)priority({ConstructionTargetKind::Habitat,i},placed_habitats_[i].position,habitat_models_.mesh(placed_habitats_[i]).height);
    for(std::size_t i=0;i<placed_resources_.size();++i)priority({ConstructionTargetKind::Resource,i},placed_resources_[i].position,resource_models_.mesh(placed_resources_[i]).height);
    const auto links=[&](const auto& connections,ConstructionTargetKind kind){for(std::size_t i=0;i<connections.size();++i){const auto& path=connections[i].path;if(!path.empty())priority({kind,i},path[path.size()/2],3);}};
    links(corridor_network_.connections(),ConstructionTargetKind::Corridor);links(utility_network_.connections(),ConstructionTargetKind::Utility);links(walkway_network_.connections(),ConstructionTargetKind::Walkway);
    for(const auto& h:placed_habitats_)if(h.construction.complete())draw(h.position,habitat_models_.mesh(h).height,h.supply,true);
    for(const auto& q:placed_resources_)if(q.construction.complete())draw(q.position,resource_models_.mesh(q).height,q.supply,q.kind==ResourceKind::StorageDome);
}

} // namespace game
