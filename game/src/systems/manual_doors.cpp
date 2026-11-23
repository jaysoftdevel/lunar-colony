#include "game/game_app.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/world/storage_loading_gate.hpp"
namespace game {
std::optional<std::pair<bool,std::size_t>> GameApp::manual_door_target() const {
    using namespace runtime;
    if(first_person_mode_!=FirstPersonMode::Active)return {};
    const auto eye=first_person_.eye_position(),view=first_person_.look_direction();
    float best=3.2f;std::optional<std::pair<bool,std::size_t>> target;
    const auto consider=[&](Vec3 center,bool resource,std::size_t index){
        const auto delta=center-eye;const float distance=length(delta);
        if(distance<best&&distance>.05f&&dot(normalize(delta),view)>.72f){best=distance;target={{resource,index}};}
    };
    for(std::size_t i=0;i<pressure_doors_.size();++i)if(!pressure_doors_[i].powered)
        consider(pressure_doors_[i].door.wall_position+Vec3{0,geometry::kDoorClearHeight*.5f,0},false,i);
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& item=placed_resources_[i];if(!item.construction.complete())continue;
        if(item.kind==ResourceKind::RobotWorkshop&&!door_power_available(item.supply,item.power_enabled))
            consider(workshop::world(item,{0,workshop::kDeck+1.4f,-workshop::kHalfZ}),true,i);
        if(item.kind==ResourceKind::StorageGate&&item.storage_parent_index<placed_resources_.size()){
            const auto& owner=placed_resources_[item.storage_parent_index];
            if(!door_power_available(owner.supply,owner.power_enabled)||!item.power_enabled)
                consider(storage_loading::mouth(owner,item.storage_loading_face)+Vec3{0,1.4f,0},true,i);
        }
    }
    return target;
}
std::string GameApp::manual_door_label() const {
    const auto target=manual_door_target();if(!target)return {};
    const auto [resource,index]=*target;
    const bool open=resource?(placed_resources_[index].manual_gate_override?placed_resources_[index].manual_gate_open:
        (placed_resources_[index].kind==ResourceKind::RobotWorkshop?placed_resources_[index].workshop_gate_open:placed_resources_[index].storage_extension_open)>.5f):
        (pressure_doors_[index].manual_override?pressure_doors_[index].manual_open:pressure_doors_[index].open_fraction>.5f);
    return std::string("NO POWER / E: MANUAL ")+(open?"CLOSE":"OPEN");
}
bool GameApp::manual_door_interaction(bool activate){
    const auto target=manual_door_target();if(!target)return false;
    if(!activate)return true;
    const auto [resource,index]=*target;
    if(resource){auto& item=placed_resources_[index];const float fraction=item.kind==ResourceKind::RobotWorkshop?item.workshop_gate_open:item.storage_extension_open;
        item.manual_gate_open=item.manual_gate_override?!item.manual_gate_open:fraction<.5f;item.manual_gate_override=true;return true;}
    auto& state=pressure_doors_[index];
    const bool opening=state.manual_override?!state.manual_open:state.open_fraction<.5f;
    if(opening){
        // A manual crank bypasses the motor, never the airlock interlock.
        for(std::size_t i=0;i<placed_habitats_.size();++i)if(placed_habitats_[i].kind==HabitatKind::AirLock){
            const auto doors=habitat_doors(i);bool owns=false;
            for(const auto& d:doors)owns|=runtime::length(d.wall_position-state.door.wall_position)<.01f;
            if(!owns)continue;
            for(const auto& other:pressure_doors_)for(const auto& d:doors)
                if(&other!=&state&&runtime::length(d.wall_position-other.door.wall_position)<.01f&&
                   (!pressure_door::sealed(other.open_fraction)||other.manual_open)){
                    activity_log_.record(ui_time_seconds_,LogEvent::Incident,"Manual airlock: close the other hatch first");return false;
                }
        }
    }
    state.manual_override=true;state.manual_open=opening;state.hold_seconds=0;return true;
}
}
