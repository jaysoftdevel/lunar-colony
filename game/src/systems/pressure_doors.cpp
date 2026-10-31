#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/util/world_math.hpp"

namespace game {
void GameApp::assign_storage_loading_faces(){
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageDome || item.storage_loading_face>=0) continue;
        // Keep every installed legacy corridor. Once selected the gate face is
        // persisted, so building/removing other connections cannot move it.
        item.storage_loading_face=4;
        for(int face:{1,0,2,3}){
            const auto n=world_math::rotate_y(storage_loading::local_normal(face),item.rotation_degrees);
            bool used=false;
            for(const auto& c:corridor_network_.connections()) for(const auto* ep:{&c.endpoint_a,&c.endpoint_b})
                if(ep->kind==CorridorEndpointKind::Resource && ep->index==i && eng::dot(ep->door.normal,n)>0.70f) used=true;
            if(!used){item.storage_loading_face=(std::int8_t)face;break;}
        }
    }
}

void GameApp::update_pressure_doors(float dt){
    update_workshop_gates(dt);
    for(auto& state:pressure_doors_) state.seen=false;
    auto register_door=[&](const CorridorDoor& door,bool permitted=true){
        auto it=std::find_if(pressure_doors_.begin(),pressure_doors_.end(),[&](const auto& state){
            return eng::length(state.door.wall_position-door.wall_position)<0.01f && eng::dot(state.door.normal,door.normal)>0.99f;
        });
        if(it==pressure_doors_.end()){
            pressure_doors_.push_back({door});it=pressure_doors_.end()-1;
        }
        if(it->seen) return;
        it->seen=true;it->door=door;
        bool requested=false,occupied=false;
        if(first_person_mode_==FirstPersonMode::Active){
            const auto feet=first_person_.feet_position();
            auto d=feet-door.wall_position;d.y=0;
            const float vertical=feet.y-door.position.y;
            // Robots on the lunar surface do not trigger crew pressure doors.
            if(vertical>-1.0f && vertical<geometry::kDoorClearHeight){
                requested=eng::length(d)<geometry::kDoorTriggerRadius;
                const eng::Vec3 side{-door.normal.z,0,door.normal.x};
                occupied=std::abs(eng::dot(d,door.normal))<first_person_.body_radius()+0.35f &&
                    std::abs(eng::dot(d,side))<geometry::kDoorClearWidth*0.5f+first_person_.body_radius();
            }
        }
        pressure_door::advance(*it,dt,requested && permitted,occupied);
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        if(!placed_habitats_[i].construction.complete()) continue;
        const auto doors=habitat_doors(i);
        for(const auto& d:doors){
            bool permitted=true;
            if(placed_habitats_[i].kind==HabitatKind::AirLock){
                // The surface hatch and pressure-network hatch must never
                // open together, including during the close animation/hold.
                for(const auto& other:doors){
                    if(eng::length(other.wall_position-d.wall_position)<0.01f) continue;
                    if(!pressure_door::sealed(first_person_door_open_fraction(other.wall_position))) permitted=false;
                }
            }
            register_door(d,permitted);
        }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i)
        if(placed_resources_[i].construction.complete()) for(const auto& d:storage_dome_doors(i)) register_door(d);
    std::erase_if(pressure_doors_,[](const auto& state){return !state.seen;});

    storage_gate_open_.resize(placed_resources_.size(),0);
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];bool operating=false;
        if(item.kind==ResourceKind::StorageDome && item.construction.complete())
            for(const auto& robot:builder_robots_)
                if(storage_loading::at_service(item,robot.position) &&
                   (robot.action==BuilderRobotAction::Pickup || robot.action==BuilderRobotAction::PutDown)) operating=true;
        // Outer cargo shutters expose a sealed transfer buffer, never the
        // warehouse atmosphere or an exterior crew doorway.
        storage_gate_open_[i]=std::clamp(storage_gate_open_[i]+(operating?3.2f:-2.0f)*std::clamp(dt,0.0f,0.10f),0.0f,1.0f);
    }
}
}
