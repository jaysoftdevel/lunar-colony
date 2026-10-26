#pragma once
#include "game/world/corridor_network.hpp"
#include "game/world/resource_mesh.hpp"
#include <sstream>
#include <string>

namespace game::world_labels {
inline const char* habitat_name(HabitatKind kind){
    switch(kind){
        case HabitatKind::AirLock: return "AIR LOCK";
        case HabitatKind::OxygenGeneratorDome: return "OXYGEN GENERATOR DOME";
        case HabitatKind::CantineDome: return "CANTINE DOME";
        case HabitatKind::BioDome: return "BIO DOME";
        case HabitatKind::CrewQuarters: return "CREW QUARTERS";
        default: return "HABITAT";
    }
}
inline const char* resource_name(ResourceKind kind){
    switch(kind){
        case ResourceKind::SolarPanel: return "SOLAR PANEL";
        case ResourceKind::WaterDriller: return "WATER DRILLER";
        case ResourceKind::PowerStorage: return "POWER STORAGE";
        case ResourceKind::WaterStorage: return "WATER STORAGE";
        default: return "RESOURCE STRUCTURE";
    }
}
inline std::string corridor_endpoint_name(const CorridorEndpoint& endpoint){
    std::ostringstream out;
    switch(endpoint.kind){
        case CorridorEndpointKind::Habitat: out<<"HABITAT "<<(endpoint.index+1); break;
        case CorridorEndpointKind::Corridor: out<<"CORRIDOR "<<(endpoint.index+1)<<" INTERSECTION"; break;
        case CorridorEndpointKind::OpenDoor: out<<"SEALED END"; break;
    }
    return out.str();
}
} // namespace game::world_labels
