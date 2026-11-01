#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/util/world_math.hpp"

namespace game {
int GameApp::airlock_inside_sign(std::size_t index) const {
    if(index>=placed_habitats_.size())return 1;
    const auto& habitat=placed_habitats_[index];
    for(const auto& c:corridor_network_.connections())if(c.construction.complete())
        for(const auto* e:{&c.endpoint_a,&c.endpoint_b})if(e->kind==CorridorEndpointKind::Habitat && e->index==index)
            return world_math::rotate_y(e->door.wall_position-habitat.position,-habitat.rotation_degrees).x>=0?1:-1;
    return 1;
}

void GameApp::update_pressure_doors(float dt){
    update_workshop_gates(dt);
    for(auto& state:pressure_doors_) state.seen=false;
    airlock_cycles_.resize(placed_habitats_.size());
    const auto ensure_door=[&](const CorridorDoor& door){
        auto it=std::find_if(pressure_doors_.begin(),pressure_doors_.end(),[&](const auto& state){
            return eng::length(state.door.wall_position-door.wall_position)<0.01f && eng::dot(state.door.normal,door.normal)>0.99f;
        });
        if(it==pressure_doors_.end()){pressure_doors_.push_back({door});it=pressure_doors_.end()-1;}
        it->seen=true;it->door=door;return (std::size_t)(it-pressure_doors_.begin());
    };
    const auto register_door=[&](const CorridorDoor& door){
        const auto index=ensure_door(door);bool requested=false,occupied=false;
        if(first_person_mode_==FirstPersonMode::Active){
            const auto feet=first_person_.feet_position();auto d=feet-door.wall_position;d.y=0;
            const float vertical=feet.y-door.position.y;
            if(vertical>-1.0f && vertical<geometry::kDoorClearHeight){
                requested=eng::length(d)<geometry::kDoorTriggerRadius;
                const eng::Vec3 side{-door.normal.z,0,door.normal.x};
                occupied=std::abs(eng::dot(d,door.normal))<first_person_.body_radius()+0.35f &&
                    std::abs(eng::dot(d,side))<geometry::kDoorClearWidth*0.5f+first_person_.body_radius();
            }
        }
        pressure_door::advance(pressure_doors_[index],dt,requested,occupied);
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& habitat=placed_habitats_[i];
        if(!habitat.construction.complete())continue;
        const auto doors=habitat_doors(i);
        if(habitat.kind!=HabitatKind::AirLock){for(const auto& d:doors)register_door(d);continue;}
        auto& cycle=airlock_cycles_[i];
        if(cycle.identity!=habitat.placement_sequence){cycle={};cycle.identity=habitat.placement_sequence;}
        std::vector<std::size_t> indices;for(const auto& d:doors)indices.push_back(ensure_door(d));
        std::vector<airlock::DoorSample> samples;
        for(auto index:indices){auto& state=pressure_doors_[index];samples.push_back({&state,
            world_math::rotate_y(state.door.wall_position-habitat.position,-habitat.rotation_degrees).x,airlock::internal(state.door)});}
        const auto local=world_math::rotate_y(first_person_.feet_position()-habitat.position,-habitat.rotation_degrees);
        const auto& mesh=habitat_models_.mesh(habitat);
        const bool present=first_person_mode_==FirstPersonMode::Active &&
            std::abs(local.x)<std::abs(mesh.entrances.front().local_position.x)+airlock::kApproach+0.5f &&
            std::abs(local.z)<geometry::kDoorClearWidth*0.5f+first_person_.body_radius()+0.2f &&
            local.y>=mesh.walkable_floor_y-0.2f && local.y<mesh.walkable_floor_y+geometry::kDoorClearHeight;
        auto forward=first_person_.look_direction();forward.y=0;forward=eng::normalize(forward);
        const eng::Vec3 right{-forward.z,0,forward.x};eng::Vec3 wish{};
        if(input_.key_w_down)wish+=forward;
        if(input_.key_s_down)wish+=forward*-1.f;
        if(input_.key_d_down)wish+=right;
        if(input_.key_a_down)wish+=right*-1.f;
        const float wish_x=world_math::rotate_y(wish,-habitat.rotation_degrees).x;
        airlock::advance(cycle,samples,local,wish_x,present,airlock_inside_sign(i),first_person_.body_radius(),dt);
    }
    for(std::size_t i=0;i<placed_resources_.size();++i)
        if(placed_resources_[i].construction.complete()) for(const auto& d:storage_dome_doors(i)) register_door(d);
    std::erase_if(pressure_doors_,[](const auto& state){return !state.seen;});

    // Every cargo gate is now an independent StorageGate construction object.
    // Animate only completed child gates; a dome with zero gates has no hidden
    // or implicit loading hatch.
    storage_gate_open_.clear();
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageGate || !item.construction.complete() ||
           item.storage_parent_index>=placed_resources_.size()) continue;
        const auto& owner=placed_resources_[item.storage_parent_index];bool using_gate=false;
        if(owner.construction.complete()) for(const auto& robot:builder_robots_)
            if(storage_loading::at_service_face(owner,robot.position,item.storage_loading_face) &&
               (robot.action==BuilderRobotAction::Pickup || robot.action==BuilderRobotAction::PutDown)) using_gate=true;
        auto& opening=placed_resources_[i].storage_extension_open;
        opening=std::clamp(opening+(using_gate?3.2f:-2.0f)*std::clamp(dt,0.0f,0.10f),0.0f,1.0f);
    }
}
}
