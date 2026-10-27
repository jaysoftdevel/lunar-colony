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
        case HabitatKind::ProcessingFacility: return "PROCESSING FACILITY";
        case HabitatKind::Factory: return "FACTORY";
        case HabitatKind::Manufactory: return "MANUFACTORY";
        case HabitatKind::SickBay: return "SICK BAY";
        case HabitatKind::Laboratory: return "LABORATORY";
        default: return "HABITAT";
    }
}
inline const char* resource_name(ResourceKind kind){
    switch(kind){
        case ResourceKind::SolarPanel: return "SOLAR PANEL";
        case ResourceKind::WaterDriller: return "WATER DRILLER";
        case ResourceKind::PowerStorage: return "POWER STORAGE";
        case ResourceKind::WaterStorage: return "WATER STORAGE";
        case ResourceKind::OreDriller: return "ORE DRILLER";
        case ResourceKind::WaterPurifier: return "WATER PURIFIER";
        case ResourceKind::Antenna: return "COMMUNICATION ANTENNA";
        case ResourceKind::LandingPad: return "LANDING PAD";
        case ResourceKind::StorageDome: return "STORAGE DOME";
        default: return "RESOURCE STRUCTURE";
    }
}
inline std::string corridor_endpoint_name(const CorridorEndpoint& endpoint){
    std::ostringstream out;
    switch(endpoint.kind){
        case CorridorEndpointKind::Habitat: out<<"HABITAT "<<(endpoint.index+1); break;
        case CorridorEndpointKind::Resource: out<<"STORAGE DOME "<<(endpoint.index+1); break;
        case CorridorEndpointKind::Corridor: out<<"CORRIDOR "<<(endpoint.index+1)<<" INTERSECTION"; break;
        case CorridorEndpointKind::OpenDoor: out<<"SEALED END"; break;
    }
    return out.str();
}
} // namespace game::world_labels
