#include "game/features/build_catalog.hpp"

namespace game {
namespace {
BuildObjectDefinition habitat(const char* id,const char* name,const char* type,BuildIcon icon,HabitatKind kind,HabitatSizeTier size=HabitatSizeTier::Standard){
    return {id,name,type,icon,BuildCategory::Habitats,BuildObjectKind::Habitat,kind,size,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition corridor(){
    return {"corridor","Corridor","Infrastructure / Pressure Link",BuildIcon::Corridor,BuildCategory::Habitats,BuildObjectKind::Corridor,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition resource(const char* id,const char* name,const char* type,BuildIcon icon,BuildCategory category,ResourceKind kind){
    return {id,name,type,icon,category,BuildObjectKind::Resource,HabitatKind::AirLock,HabitatSizeTier::Standard,kind,UtilityKind::SupplyChannel};
}
BuildObjectDefinition utility(){
    return {"supply_channel","Supply Channel","Utility / Power + Water",BuildIcon::SupplyChannel,BuildCategory::Resources,BuildObjectKind::Utility,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition walkway(){
    return {"walkway","Walkway","Infrastructure / Access + Supply",BuildIcon::Walkway,BuildCategory::Facilities,BuildObjectKind::Walkway,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
}

BuildCatalog::BuildCatalog(){
    items_={
        habitat("air_lock","Air Lock","Habitat / Access",BuildIcon::AirLock,HabitatKind::AirLock),

        habitat("oxygen_generator_dome","Oxygen Generator Dome","Life Support / Scalable",BuildIcon::OxygenDome,HabitatKind::OxygenGeneratorDome),
        habitat("cantine_dome","Cantine Dome","Crew / Food / Scalable",BuildIcon::CantineDome,HabitatKind::CantineDome),
        habitat("bio_dome","Bio Dome","Food / Biology / Scalable",BuildIcon::BioDome,HabitatKind::BioDome),
        habitat("crew_quarters","Crew Quarters","Crew / Housing / Scalable",BuildIcon::CrewQuarters,HabitatKind::CrewQuarters),

        habitat("processing_facility","Processing Facility","Industry / Basic Ingots",BuildIcon::ProcessingFacility,HabitatKind::ProcessingFacility),
        habitat("factory","Factory","Industry / Advanced Items",BuildIcon::Factory,HabitatKind::Factory),
        habitat("manufactory","Manufactory","Industry / Complex Fabrication",BuildIcon::Manufactory,HabitatKind::Manufactory),
        habitat("sick_bay","Sick Bay","Crew / Medical",BuildIcon::SickBay,HabitatKind::SickBay),
        habitat("laboratory","Laboratory","Research / Advanced Technology",BuildIcon::Laboratory,HabitatKind::Laboratory),
        corridor(),

        resource("solar_panel","Solar Panel","Resource / Power Generation",BuildIcon::SolarPanel,BuildCategory::Resources,ResourceKind::SolarPanel),
        resource("water_driller","Water Driller","Resource / Extraction",BuildIcon::WaterDriller,BuildCategory::Resources,ResourceKind::WaterDriller),
        resource("power_storage","Power Storage","Resource / Energy Storage",BuildIcon::PowerStorage,BuildCategory::Resources,ResourceKind::PowerStorage),
        resource("water_storage","Water Storage","Resource / Water Storage",BuildIcon::WaterStorage,BuildCategory::Resources,ResourceKind::WaterStorage),
        resource("ore_driller","Ore Driller","Resource / Ore Extraction",BuildIcon::OreDriller,BuildCategory::Resources,ResourceKind::OreDriller),
        utility(),

        resource("water_purifier","Water Purifier","Facility / Water Recycling",BuildIcon::WaterPurifier,BuildCategory::Facilities,ResourceKind::WaterPurifier),
        resource("antenna","Antenna","Facility / Communications",BuildIcon::Antenna,BuildCategory::Facilities,ResourceKind::Antenna),
        resource("landing_pad","Landing Pad","Facility / Transport",BuildIcon::LandingPad,BuildCategory::Facilities,ResourceKind::LandingPad),
        walkway()
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
