#pragma once
#include "game/world/walkway_network.hpp"
#include "game/util/world_math.hpp"
#include <cmath>

namespace game::airlock_connections {

// Air Locks have no authored hatches any more.  Their first connection may
// create an entrance on either longitudinal end, but never on a side wall; the
// chosen end is then reserved for all base-side utility/corridor connections.
inline int side(const PlacedHabitat& habitat,eng::Vec3 point){
    const auto local=world_math::rotate_y(point-habitat.position,-habitat.rotation_degrees);
    const float scale=habitat.size==HabitatSizeTier::Small?0.76f:1.0f;
    const float end_zone=5.0f*scale*0.68f;
    if(std::abs(local.x)<end_zone) return 0;
    return local.x>0.0f?1:-1;
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
