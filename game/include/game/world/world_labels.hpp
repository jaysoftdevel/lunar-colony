#pragma once
#include "game/world/corridor_network.hpp"
#include "game/world/resource_mesh.hpp"
#include <algorithm>
#include <cstdint>
#include <iomanip>
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

inline std::string habitat_instance_name(const PlacedHabitat& placed){
    std::ostringstream out;
    out<<habitat_name(placed.kind)<<" / "<<habitat_size_name(placed.size)<<" / #"
       <<std::setfill('0')<<std::setw(3)<<std::max<std::uint32_t>(1,placed.instance_serial);
    return out.str();
}

inline const char* resource_instance_size_name(const PlacedResource& placed){
    return placed.kind==ResourceKind::StorageDome?storage_dome_size_name(placed.storage_size):resource_size_name(placed.size);
}

inline std::string resource_instance_name(const PlacedResource& placed){
    std::ostringstream out;
    out<<resource_name(placed.kind)<<" / "<<resource_instance_size_name(placed)<<" / #"
       <<std::setfill('0')<<std::setw(3)<<std::max<std::uint32_t>(1,placed.instance_serial);
    return out.str();
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
