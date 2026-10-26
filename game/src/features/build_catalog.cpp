#include "game/features/build_catalog.hpp"

namespace game {
BuildCatalog::BuildCatalog(){
    items_={
        {"air_lock","Air Lock","Habitat / Access",BuildIcon::AirLock,BuildCategory::Habitats,BuildObjectKind::Habitat,HabitatKind::AirLock,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"oxygen_generator_dome","Oxygen Generator Dome","Life Support",BuildIcon::OxygenDome,BuildCategory::Habitats,BuildObjectKind::Habitat,HabitatKind::OxygenGeneratorDome,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"cantine_dome","Cantine Dome","Crew / Food",BuildIcon::CantineDome,BuildCategory::Habitats,BuildObjectKind::Habitat,HabitatKind::CantineDome,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"bio_dome","Bio Dome","Food / Biology",BuildIcon::BioDome,BuildCategory::Habitats,BuildObjectKind::Habitat,HabitatKind::BioDome,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"crew_quarters","Crew Quarters","Crew / Housing",BuildIcon::CrewQuarters,BuildCategory::Habitats,BuildObjectKind::Habitat,HabitatKind::CrewQuarters,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"corridor","Corridor","Infrastructure / Pressure Link",BuildIcon::Corridor,BuildCategory::Habitats,BuildObjectKind::Corridor,HabitatKind::AirLock,ResourceKind::SolarPanel,UtilityKind::PowerCable},

        {"solar_panel","Solar Panel","Resource / Power Generation",BuildIcon::SolarPanel,BuildCategory::Resources,BuildObjectKind::Resource,HabitatKind::AirLock,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"water_driller","Water Driller","Resource / Extraction",BuildIcon::WaterDriller,BuildCategory::Resources,BuildObjectKind::Resource,HabitatKind::AirLock,ResourceKind::WaterDriller,UtilityKind::PowerCable},
        {"power_storage","Power Storage","Resource / Energy Storage",BuildIcon::PowerStorage,BuildCategory::Resources,BuildObjectKind::Resource,HabitatKind::AirLock,ResourceKind::PowerStorage,UtilityKind::PowerCable},
        {"water_storage","Water Storage","Resource / Water Storage",BuildIcon::WaterStorage,BuildCategory::Resources,BuildObjectKind::Resource,HabitatKind::AirLock,ResourceKind::WaterStorage,UtilityKind::PowerCable},
        {"power_cable","Power Cable","Utility / Electrical Channel",BuildIcon::PowerCable,BuildCategory::Resources,BuildObjectKind::Utility,HabitatKind::AirLock,ResourceKind::SolarPanel,UtilityKind::PowerCable},
        {"water_pipe","Water Pipe","Utility / Water Channel",BuildIcon::WaterPipe,BuildCategory::Resources,BuildObjectKind::Utility,HabitatKind::AirLock,ResourceKind::SolarPanel,UtilityKind::WaterPipe}
    };
}

const BuildObjectDefinition* BuildCatalog::selected() const {
    if(selected_index_<0 || selected_index_>=(int)items_.size()) return nullptr;
    return &items_[(size_t)selected_index_];
}

void BuildCatalog::select(int index){
    if(index<0 || index>=(int)items_.size()) return;
    if(items_[(size_t)index].category!=category_) return;
    selected_index_=(selected_index_==index)?-1:index;
}
}
