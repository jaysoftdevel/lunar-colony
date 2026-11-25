#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/util/world_math.hpp"
#include <limits>

namespace game {
int GameApp::airlock_inside_sign(std::size_t index) const {
    if(index>=placed_habitats_.size())return 1;
    const auto& habitat=placed_habitats_[index];
    for(const auto& c:corridor_network_.connections())if(c.construction.complete())
        for(const auto* e:{&c.endpoint_a,&c.endpoint_b})if(e->kind==CorridorEndpointKind::Habitat && e->index==index)
            return world_math::rotate_y(e->door.wall_position-habitat.position,-habitat.rotation_degrees).x>=0?1:-1;
    return 1;
}

bool GameApp::airlock_powered(std::size_t index) const {
    const auto& lock=placed_habitats_[index];
    return door_power_available(lock.supply,lock.power_enabled);
}

bool GameApp::door_power_available(const SupplyState& supply,bool enabled) const { return enabled && supply.power_fraction>1e-6; }

void GameApp::update_pressure_doors(float dt){
    const auto next_direction=[](const AstronautTestActor& actor){
        return !astronaut_resting(actor) && actor.destination && actor.route_step<actor.route.size()
             ? game::runtime::normalize(actor.route[actor.route_step]-actor.position):game::runtime::Vec3{};
    };
    update_workshop_gates(dt);
    for(auto& state:pressure_doors_) state.seen=false;
    airlock_cycles_.resize(placed_habitats_.size());
    const auto ensure_door=[&](const CorridorDoor& door){
        auto it=std::find_if(pressure_doors_.begin(),pressure_doors_.end(),[&](const auto& state){
            return game::runtime::length(state.door.wall_position-door.wall_position)<0.01f && game::runtime::dot(state.door.normal,door.normal)>0.99f;
        });
        if(it==pressure_doors_.end()){pressure_doors_.push_back({door});it=pressure_doors_.end()-1;}
        it->seen=true;it->door=door;return (std::size_t)(it-pressure_doors_.begin());
    };
    const auto register_door=[&](const CorridorDoor& door,bool powered){
        const auto index=ensure_door(door);bool requested=false,occupied=false;
        if(first_person_mode_==FirstPersonMode::Active){
            const auto feet=first_person_.feet_position();auto d=feet-door.wall_position;d.y=0;
            const float vertical=feet.y-door.position.y;
            if(vertical>-1.0f && vertical<geometry::kDoorClearHeight){
                const game::runtime::Vec3 side{-door.normal.z,0,door.normal.x};
                occupied=!pressure_door::sealed(pressure_doors_[index].open_fraction) && std::abs(game::runtime::dot(d,door.normal))<first_person_.body_radius()+0.35f &&
                    std::abs(game::runtime::dot(d,side))<geometry::kDoorClearWidth*0.5f+first_person_.body_radius();
            }
        }
        if(astronauts_active()){
            const game::runtime::Vec3 side{-door.normal.z,0,door.normal.x};
            for(const auto& actor:test_astronauts_){
                if(!astronaut_present(std::size_t(&actor-test_astronauts_.data())))continue;
                if(first_person_mode_==FirstPersonMode::Active&&int(&actor-test_astronauts_.data())==controlled_astronaut_)continue;
                auto delta=actor.position-door.wall_position;
                delta.y=0.f;
                const auto direction=next_direction(actor);
                requested|=game::runtime::length(delta)<geometry::kDoorTriggerRadius &&
                           game::runtime::dot(direction,delta)<-.05f;
                occupied|=!pressure_door::sealed(pressure_doors_[index].open_fraction) &&
                          std::abs(game::runtime::dot(delta,door.normal))<geometry::kHumanBodyRadius+.10f &&
                          std::abs(game::runtime::dot(delta,side))<geometry::kDoorClearWidth*.5f;
            }
        }
        auto& state=pressure_doors_[index];
        if(!powered)state.player_request_seconds=0;
        state.player_request_seconds=std::max(0.f,state.player_request_seconds-std::clamp(dt,0.f,.1f));state.powered=powered;
        if(powered){state.manual_override=false;state.manual_open=false;pressure_door::advance(state,dt,requested||state.player_request_seconds>0,occupied);}
        else if(state.manual_override){state.hold_seconds=0;pressure_door::advance(state,dt*.28f,state.manual_open,occupied && state.open_fraction>.001f);}
        else state.hold_seconds=0;
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& habitat=placed_habitats_[i];
        if(!habitat.construction.complete())continue;
        const auto doors=habitat_doors(i);
        if(habitat.kind!=HabitatKind::AirLock){for(const auto& d:doors)register_door(d,door_power_available(habitat.supply,habitat.power_enabled));continue;}
        auto& cycle=airlock_cycles_[i];
        if(cycle.identity!=habitat.placement_sequence){cycle={};cycle.identity=habitat.placement_sequence;}
        std::vector<std::size_t> indices;for(const auto& d:doors)indices.push_back(ensure_door(d));
        std::vector<airlock::DoorSample> samples;
        for(auto index:indices){auto& state=pressure_doors_[index];samples.push_back({&state,
            world_math::rotate_y(state.door.wall_position-habitat.position,-habitat.rotation_degrees).x,airlock::internal(state.door)});}
        auto local=world_math::rotate_y(first_person_.feet_position()-habitat.position,-habitat.rotation_degrees);
        const auto& mesh=habitat_models_.mesh(habitat);
        bool present=first_person_mode_==FirstPersonMode::Active &&
            std::abs(local.x)<mesh.footprint_x*0.5f+airlock::kApproach+0.5f &&
            std::abs(local.z)<geometry::kDoorClearWidth*0.5f+first_person_.body_radius()+0.2f &&
            local.y>=mesh.walkable_floor_y-0.2f && local.y<mesh.walkable_floor_y+geometry::kDoorClearHeight;
        const bool player_present=present;
        auto forward=first_person_.look_direction();forward.y=0;forward=game::runtime::normalize(forward);
        const game::runtime::Vec3 right{-forward.z,0,forward.x};game::runtime::Vec3 wish{};
        if(input_.key_w_down)wish+=forward;
        if(input_.key_s_down)wish+=forward*-1.f;
        if(input_.key_d_down)wish+=right;
        if(input_.key_a_down)wish+=right*-1.f;
        float wish_x=world_math::rotate_y(wish,-habitat.rotation_degrees).x;
        const float radius=first_person_.body_radius();
        if(!present && astronauts_active()){
            float best=std::numeric_limits<float>::infinity();
            for(const auto& actor:test_astronauts_){
                if(!astronaut_present(std::size_t(&actor-test_astronauts_.data())))continue;
                if(first_person_mode_==FirstPersonMode::Active&&int(&actor-test_astronauts_.data())==controlled_astronaut_)continue;
                const auto feet=world_math::rotate_y(actor.position-habitat.position,-habitat.rotation_degrees);
                const auto direction=world_math::rotate_y(next_direction(actor),-habitat.rotation_degrees);
                if(std::abs(feet.z)>geometry::kDoorClearWidth*.5f+radius+.2f)continue;
                for(const auto& sample:samples){
                    const float delta=sample.x-feet.x;
                    const bool occupying=std::abs(delta)<radius+.12f && !pressure_door::sealed(sample.state->open_fraction);
                    const bool approaching=std::abs(delta)<airlock::kApproach && delta*direction.x>0.f;
                    const float score=occupying?-1.f:std::abs(delta)+
                        (std::abs(feet.x)<mesh.footprint_x*.5f?0.f:2.f);
                    if((occupying || approaching) && score<best){
                        best=score;
                        local=feet;
                        wish_x=direction.x;
                        present=true;
                    }
                }
            }
        }
        for(auto index:indices)pressure_doors_[index].powered=airlock_powered(i);
        if(airlock_powered(i)){
            int command=player_present?-1:-2;
            for(std::size_t j=0;j<indices.size();++j){auto& state=pressure_doors_[indices[j]];
                state.manual_override=false;state.manual_open=false;
                state.player_request_seconds=std::max(0.f,state.player_request_seconds-std::clamp(dt,0.f,.1f));
                if(state.player_request_seconds>0)command=int(j);
            }
            airlock::advance(cycle,samples,local,wish_x,present,airlock_inside_sign(i),radius,dt,command);
        }else{
            cycle.requested=-1;
            cycle.equalizing=false;
            cycle.sealing=false;
            for(const auto& sample:samples){
                auto& state=*sample.state;
                state.player_request_seconds=0;
                const bool straddled=present && std::abs(local.x-sample.x)<radius+.07f;
                state.hold_seconds=0.f;
                if(state.manual_override)pressure_door::advance(state,dt*.28f,state.manual_open,straddled && state.open_fraction>.001f);
                cycle.sealing|=!pressure_door::sealed(state.open_fraction);
            }
        }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i)
        if(placed_resources_[i].construction.complete()) for(const auto& d:storage_dome_doors(i)) register_door(d,door_power_available(placed_resources_[i].supply,placed_resources_[i].power_enabled));
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
        if(opening>.001f){
            const auto mouth=storage_loading::mouth(owner,item.storage_loading_face),normal=storage_loading::normal(owner,item.storage_loading_face);
            const game::runtime::Vec3 side{-normal.z,0,normal.x};
            const auto straddles=[&](game::runtime::Vec3 p,float radius){const auto d=p-mouth;return std::abs(game::runtime::dot(d,normal))<radius+.15f&&std::abs(game::runtime::dot(d,side))<storage_loading::kHalfWidth+radius;};
            if(first_person_mode_==FirstPersonMode::Active)using_gate|=straddles(first_person_.feet_position(),first_person_.body_radius());
            for(const auto& robot:builder_robots_)using_gate|=straddles(robot.position,1.1f);
        }
        const bool powered=door_power_available(owner.supply,owner.power_enabled)&&item.power_enabled;
        auto& gate=placed_resources_[i];gate.player_gate_request_seconds=powered?std::max(0.f,gate.player_gate_request_seconds-std::clamp(dt,0.f,.1f)):0;
        if(powered){gate.manual_gate_override=false;opening=std::clamp(opening+(using_gate||gate.player_gate_request_seconds>0?3.2f:-2.0f)*std::clamp(dt,0.0f,0.10f),0.0f,1.0f);}
        else if(item.manual_gate_override)opening=std::clamp(opening+(item.manual_gate_open?.65f:-.50f)*std::clamp(dt,0.0f,.10f),0.0f,1.0f);
    }
}
}
