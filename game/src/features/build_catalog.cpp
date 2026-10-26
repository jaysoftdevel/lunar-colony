#include "game/features/build_catalog.hpp"

namespace game {
BuildCatalog::BuildCatalog(){
    items_={
        {"air_lock","Air Lock","Habitat / Access",BuildIcon::AirLock,BuildObjectKind::Habitat,HabitatKind::AirLock},
        {"oxygen_generator_dome","Oxygen Generator Dome","Life Support",BuildIcon::OxygenDome,BuildObjectKind::Habitat,HabitatKind::OxygenGeneratorDome},
        {"cantine_dome","Cantine Dome","Crew / Food",BuildIcon::CantineDome,BuildObjectKind::Habitat,HabitatKind::CantineDome},
        {"bio_dome","Bio Dome","Food / Biology",BuildIcon::BioDome,BuildObjectKind::Habitat,HabitatKind::BioDome},
        {"crew_quarters","Crew Quarters","Crew / Housing",BuildIcon::CrewQuarters,BuildObjectKind::Habitat,HabitatKind::CrewQuarters},
        {"corridor","Corridor","Infrastructure / Pressure Link",BuildIcon::Corridor,BuildObjectKind::Corridor,HabitatKind::AirLock}
    };
}

const BuildObjectDefinition* BuildCatalog::selected() const {
    if(selected_index_<0 || selected_index_>=(int)items_.size()) return nullptr;
    return &items_[(size_t)selected_index_];
}

void BuildCatalog::select(int index){
    if(index<0 || index>=(int)items_.size()) return;
    selected_index_=(selected_index_==index)?-1:index;
}
}
