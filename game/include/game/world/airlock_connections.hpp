#pragma once
#include "game/world/walkway_network.hpp"
#include "game/util/world_math.hpp"

namespace game::airlock_connections {

// The two authored hatches are at opposite ends of the local X axis. Reserve
// the opposite end even while a connection is only a construction blueprint.
inline int side(const PlacedHabitat& habitat,eng::Vec3 point){
    const float x=world_math::rotate_y(point-habitat.position,-habitat.rotation_degrees).x;
    return x>0.01f?1:x<-0.01f?-1:0;
}

inline bool allows(std::size_t index,eng::Vec3 point,const std::vector<PlacedHabitat>& habitats,
                   const std::vector<CorridorConnection>& corridors={},
                   const std::vector<UtilityConnection>& utilities={},
                   const std::vector<WalkwayConnection>& walkways={}){
    if(index>=habitats.size())return false;
    const auto& habitat=habitats[index];
    if(habitat.kind!=HabitatKind::AirLock)return true;
    const int requested=side(habitat,point);
    if(!requested)return false;
    for(const auto& c:corridors)for(const auto* e:{&c.endpoint_a,&c.endpoint_b})
        if(e->kind==CorridorEndpointKind::Habitat && e->index==index && side(habitat,e->door.wall_position)!=requested)return false;
    for(const auto& c:utilities)for(const auto* e:{&c.endpoint_a,&c.endpoint_b})
        if(e->kind==UtilityEndpointKind::Habitat && e->index==index && side(habitat,e->position)!=requested)return false;
    for(const auto& c:walkways)for(const auto* e:{&c.endpoint_a,&c.endpoint_b})
        if(e->kind==WalkwayEndpointKind::Habitat && e->index==index && side(habitat,e->door.wall_position)!=requested)return false;
    return true;
}

} // namespace game::airlock_connections
