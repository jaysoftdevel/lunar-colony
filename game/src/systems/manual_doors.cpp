#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/world/storage_loading_gate.hpp"
namespace game {
namespace {constexpr float command_seconds=8.f,crank_seconds=3.f;}
std::optional<std::pair<bool,std::size_t>> GameApp::manual_door_target() const {
    using namespace runtime;
    if(first_person_mode_!=FirstPersonMode::Active)return {};
    const auto eye=first_person_.eye_position(),view=first_person_.look_direction();
    float best=3.2f;std::optional<std::pair<bool,std::size_t>> target;
    const auto consider=[&](Vec3 center,bool resource,std::size_t index){
        const auto delta=center-eye;const float distance=length(delta);
        if(distance>=best||distance<=.05f||dot(normalize(delta),view)<=.72f)return;
        for(std::size_t j=0;j<pressure_doors_.size();++j){
            if(!resource&&j==index)continue;
            const auto& other=pressure_doors_[j];
            if(eye.y<other.door.position.y||eye.y>other.door.position.y+geometry::kDoorClearHeight)continue;
            if(pressure_door::blocks(other.door,other.open_fraction,eye,center,.01f))return;
        }
        best=distance;target={{resource,index}};
    };
    for(std::size_t i=0;i<pressure_doors_.size();++i)
        consider(pressure_doors_[i].door.wall_position+Vec3{0,geometry::kDoorClearHeight*.5f,0},false,i);
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& item=placed_resources_[i];if(!item.construction.complete())continue;
        if(item.kind==ResourceKind::RobotWorkshop)
            consider(workshop::world(item,{0,workshop::kDeck+1.4f,-workshop::kHalfZ}),true,i);
        if(item.kind==ResourceKind::StorageGate&&item.storage_parent_index<placed_resources_.size()){
            const auto& owner=placed_resources_[item.storage_parent_index];
            consider(storage_loading::mouth(owner,item.storage_loading_face)+Vec3{0,1.4f,0},true,i);
        }
    }
    return target;
}
bool GameApp::door_target_powered(std::pair<bool,std::size_t> target) const {
    const auto [resource,index]=target;if(!resource)return pressure_doors_[index].powered;
    const auto& item=placed_resources_[index];
    const auto& owner=item.kind==ResourceKind::StorageGate?placed_resources_[item.storage_parent_index]:item;
    return item.power_enabled&&door_power_available(owner.supply,owner.power_enabled);
}
float GameApp::door_target_fraction(std::pair<bool,std::size_t> target) const {
    const auto [resource,index]=target;if(!resource)return pressure_doors_[index].open_fraction;
    const auto& item=placed_resources_[index];return item.kind==ResourceKind::RobotWorkshop?item.workshop_gate_open:item.storage_extension_open;
}
std::string GameApp::manual_door_label() const {
    const auto target=manual_door_target();if(!target)return {};
    const auto [resource,index]=*target;const bool powered=door_target_powered(*target);
    if(!powered&&door_hold_.target==target&&door_hold_.blocked)return "INTERLOCK / CLOSE OTHER HATCH";
    const float command=resource?placed_resources_[index].player_gate_request_seconds:pressure_doors_[index].player_request_seconds;
    bool opening=command<=0&&door_target_fraction(*target)<.5f;
    if(!powered&&door_hold_.target==target&&!door_hold_.complete)opening=door_hold_.opening;
    return std::string(powered?"PRESS E / ":"HOLD E / MANUAL ")+(opening?"OPEN HATCH":"CLOSE HATCH");
}
bool GameApp::manual_door_interaction(bool activate,std::optional<bool> goal){
    const auto target=manual_door_target();if(!target)return false;if(!activate)return true;
    const auto [resource,index]=*target;const bool powered=door_target_powered(*target);
    const float command=resource?placed_resources_[index].player_gate_request_seconds:pressure_doors_[index].player_request_seconds;
    const bool opening=goal.value_or(command<=0&&door_target_fraction(*target)<.5f);
    if(resource){auto& item=placed_resources_[index];
        if(powered)item.player_gate_request_seconds=opening?command_seconds:0;
        else {item.manual_gate_open=opening;item.manual_gate_override=true;}return true;}
    auto& state=pressure_doors_[index];
    for(std::size_t i=0;i<placed_habitats_.size();++i)if(placed_habitats_[i].kind==HabitatKind::AirLock){
        const auto doors=habitat_doors(i);bool owns=false;for(const auto& d:doors)owns|=runtime::length(d.wall_position-state.door.wall_position)<.01f;if(!owns)continue;
        for(auto& other:pressure_doors_)for(const auto& d:doors)
            if(&other!=&state&&runtime::length(d.wall_position-other.door.wall_position)<.01f){
                if(opening&&!powered&&(!pressure_door::sealed(other.open_fraction)||other.manual_open))return false;
                if(powered)other.player_request_seconds=0;
            }
    }
    if(powered)state.player_request_seconds=opening?command_seconds:0;
    else {state.manual_override=true;state.manual_open=opening;}
    state.hold_seconds=0;return true;
}
void GameApp::update_door_interaction(float dt,const runtime::InputState& input){
    using namespace runtime;
    const auto target=manual_door_target();
    if(!target||simulation_paused()){door_hold_={};return;}
    const auto [resource,index]=*target;
    const Vec3 anchor=resource?placed_resources_[index].position:pressure_doors_[index].door.wall_position;
    if(door_target_powered(*target)){
        door_hold_={};if(input.key_e_pressed)manual_door_interaction(true);return;
    }
    const bool held=input.key_e_down||input.key_e_pressed;
    if(door_hold_.target!=target||length(door_hold_.anchor-anchor)>.01f){
        door_hold_={};door_hold_.target=target;door_hold_.anchor=anchor;door_hold_.opening=door_target_fraction(*target)<.5f;
    }
    if(!held){door_hold_.down=false;return;}
    if(door_hold_.complete){if(door_hold_.down)return;door_hold_.opening=!door_hold_.opening;door_hold_.complete=false;}
    door_hold_.down=true;
    // Keep the crank's physical partial opening on release. Retargeting or
    // leaving reach cancels the input; no motor runs for an unpowered door.
    const float before=door_target_fraction(*target);const bool opening=door_hold_.opening;
    door_hold_.blocked=!manual_door_interaction(true,opening);
    if(door_hold_.blocked)return; // Enforces the mechanical airlock interlock.
    const float after=std::clamp(before+(opening?1.f:-1.f)*std::clamp(dt,0.f,.10f)/crank_seconds,0.f,1.f);
    if(!resource){
        auto& state=pressure_doors_[index];state.manual_override=false;state.manual_open=false;
        const auto d=first_person_.feet_position()-state.door.wall_position;const auto side=Vec3{-state.door.normal.z,0,state.door.normal.x};
        bool straddled=std::abs(dot(d,state.door.normal))<first_person_.body_radius()+.1f&&std::abs(dot(d,side))<geometry::kDoorClearWidth*.5f+first_person_.body_radius();
        for(std::size_t j=0;j<test_astronauts_.size();++j)if(astronaut_present(j)){
            const auto delta=test_astronauts_[j].position-state.door.wall_position;
            straddled|=std::abs(dot(delta,state.door.normal))<geometry::kHumanBodyRadius+.1f&&std::abs(dot(delta,side))<geometry::kDoorClearWidth*.5f+geometry::kHumanBodyRadius;
        }
        if(!opening&&straddled)return;
        state.open_fraction=after;state.hold_seconds=0;
    }else{
        auto& item=placed_resources_[index];item.manual_gate_override=false;item.manual_gate_open=false;
        const auto point=first_person_.feet_position();
        bool straddled=false;
        if(item.kind==ResourceKind::RobotWorkshop){const auto p=workshop::local(item,point);straddled=std::abs(p.z+workshop::kHalfZ)<1.2f&&std::abs(p.x)<3.f;}
        else {const auto& owner=placed_resources_[item.storage_parent_index];const auto d=point-storage_loading::mouth(owner,item.storage_loading_face);const auto n=storage_loading::normal(owner,item.storage_loading_face);straddled=std::abs(dot(d,n))<.7f&&length(d-n*dot(d,n))<storage_loading::kHalfWidth+.4f;}
        for(const auto& robot:builder_robots_){
            if(item.kind==ResourceKind::RobotWorkshop){const auto p=workshop::local(item,robot.position);straddled|=std::abs(p.z+workshop::kHalfZ)<2.f&&std::abs(p.x)<workshop::kGateHalf+1.f;}
            else {const auto& owner=placed_resources_[item.storage_parent_index];straddled|=length(robot.position-storage_loading::mouth(owner,item.storage_loading_face))<2.2f;}
        }
        if(!opening&&straddled)return;
        if(item.kind==ResourceKind::RobotWorkshop)item.workshop_gate_open=after;else item.storage_extension_open=after;
    }
    door_hold_.progress=opening?after:1-after;door_hold_.complete=door_hold_.progress>=.9999f;
}
void GameApp::render_door_interaction(runtime::SceneRenderer& r) const {
    using namespace runtime;using namespace layout;
    const auto target=manual_door_target();if(!target)return;
    const bool powered=door_target_powered(*target);const float w=ui_width(r),h=ui_height(r);
    const float x=w*.5f,y=h*.5f-68;
    const Color ink=powered?Color{.52f,.96f,.88f,1}:Color{1.f,.73f,.29f,1};
    const bool pressed=door_hold_.target==target&&door_hold_.down;
    const Rect key{x-21,y-21,42,42};r.draw_rect(key,pressed?Color{.19f,.11f,.035f,.98f}:Color{.02f,.035f,.045f,.93f});
    r.draw_text({x-7.2f,y-8.4f},"E",2.4f,ink);
    constexpr int segments=48;const float radius=29;
    const float progress=door_hold_.target==target?door_hold_.progress:0;
    for(int i=0;i<segments;++i){
        const float a=-1.5707963f+6.2831853f*i/segments,b=-1.5707963f+6.2831853f*(i+1)/segments;
        const Color color=!powered&&float(i)/segments<progress?ink:Color{.20f,.27f,.29f,.95f};
        r.draw_line({x+std::cos(a)*radius,y+std::sin(a)*radius},{x+std::cos(b)*radius,y+std::sin(b)*radius},color,3.f);
    }
    const std::string label=manual_door_label();const float safe_width=std::min({470.f,w-48.f,2.f*(astronaut_status_rect().x-x-10)});
    const float font=ui::fit_text(label,1.8f,{safe_width-16,24});
    const auto extent=ui::text_extent(label,font);const Rect caption{x-extent.x*.5f-8,y+37,extent.x+16,26};
    r.draw_rect(caption,{.02f,.035f,.045f,.93f});r.draw_text({caption.x+8,caption.y+6},label,font,ink);
}
}
