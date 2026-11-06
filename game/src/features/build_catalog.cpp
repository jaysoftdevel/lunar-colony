#include "game/features/build_catalog.hpp"

namespace game {
namespace {
BuildObjectDefinition habitat(const char* id,const char* name,const char* type,const char* group,BuildIcon icon,HabitatKind kind,HabitatSizeTier size=HabitatSizeTier::Standard){
    return {id,name,type,group,icon,BuildCategory::Habitats,BuildObjectKind::Habitat,kind,size,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition corridor(){
    return {"corridor","Corridor","Infrastructure / Pressure Link","ACCESS",BuildIcon::Corridor,BuildCategory::Habitats,BuildObjectKind::Corridor,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition resource(const char* id,const char* name,const char* type,const char* group,BuildIcon icon,BuildCategory category,ResourceKind kind){
    return {id,name,type,group,icon,category,BuildObjectKind::Resource,HabitatKind::AirLock,HabitatSizeTier::Standard,kind,UtilityKind::SupplyChannel};
}
BuildObjectDefinition utility(){
    return {"supply_channel","Supply Channel","Utility / Power + Water","SUPPLY",BuildIcon::SupplyChannel,BuildCategory::Structures,BuildObjectKind::Utility,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
BuildObjectDefinition walkway(){
    return {"walkway","Walkway","Infrastructure / Access + Supply","SUPPLY",BuildIcon::Walkway,BuildCategory::Structures,BuildObjectKind::Walkway,HabitatKind::AirLock,HabitatSizeTier::Standard,ResourceKind::SolarPanel,UtilityKind::SupplyChannel};
}
}

BuildCatalog::BuildCatalog(){
    items_={
        habitat("air_lock","Air Lock","Habitat / Access","ACCESS",BuildIcon::AirLock,HabitatKind::AirLock),
        corridor(),

        habitat("oxygen_generator_dome","Oxygen Generator Dome","Life Support / Scalable","AIR",BuildIcon::OxygenDome,HabitatKind::OxygenGeneratorDome),

        habitat("crew_quarters","Crew Quarters","Crew / Housing / Scalable","LIVING",BuildIcon::CrewQuarters,HabitatKind::CrewQuarters),
        habitat("cantine_dome","Cantine Dome","Crew / Food / Scalable","LIVING",BuildIcon::CantineDome,HabitatKind::CantineDome),
        habitat("bio_dome","Bio Dome","Food / Biology / Scalable","LIVING",BuildIcon::BioDome,HabitatKind::BioDome),

        habitat("sick_bay","Sick Bay","Crew / Medical","MEDICAL",BuildIcon::SickBay,HabitatKind::SickBay),

        habitat("processing_facility","Processing Facility","Industry / Basic Ingots","INDUSTRY",BuildIcon::ProcessingFacility,HabitatKind::ProcessingFacility),
        habitat("factory","Factory","Industry / Advanced Items","INDUSTRY",BuildIcon::Factory,HabitatKind::Factory),
        habitat("manufactory","Manufactory","Industry / Complex Fabrication","INDUSTRY",BuildIcon::Manufactory,HabitatKind::Manufactory),
        habitat("laboratory","Laboratory","Research / Advanced Technology","INDUSTRY",BuildIcon::Laboratory,HabitatKind::Laboratory),

        resource("storage_dome","Storage Dome","Habitat / Material Storage / Scalable","STORAGE",BuildIcon::StorageDome,BuildCategory::Habitats,ResourceKind::StorageDome),

        resource("solar_panel","Solar Panel","Resource / Power Generation","POWER",BuildIcon::SolarPanel,BuildCategory::Structures,ResourceKind::SolarPanel),
        resource("power_storage","Power Storage","Resource / Energy Storage","POWER",BuildIcon::PowerStorage,BuildCategory::Structures,ResourceKind::PowerStorage),

        resource("water_driller","Water Driller","Resource / Extraction","WATER",BuildIcon::WaterDriller,BuildCategory::Structures,ResourceKind::WaterDriller),
        resource("water_purifier","Water Purifier","Facility / Water Recycling","WATER",BuildIcon::WaterPurifier,BuildCategory::Structures,ResourceKind::WaterPurifier),
        resource("water_storage","Water Storage","Resource / Water Storage","WATER",BuildIcon::WaterStorage,BuildCategory::Structures,ResourceKind::WaterStorage),

        resource("hydrogen_tank","Hydrogen Storage Tank","Resource / H2 Storage / Rocket + Lander Fuel","HYDROGEN",BuildIcon::HydrogenTank,BuildCategory::Structures,ResourceKind::HydrogenTank),
        resource("hydrogen_generator","Hydrogen Generator","Resource / H2 Fuel Cell / Power Generation","HYDROGEN",BuildIcon::HydrogenGenerator,BuildCategory::Structures,ResourceKind::HydrogenGenerator),

        resource("oxygen_tank","Oxygen Storage Tank","Outdoor / O2 Reserve / Trade","AIR",BuildIcon::OxygenTank,BuildCategory::Structures,ResourceKind::OxygenTank),

        resource("ore_driller","Ore Driller","Resource / Ore Extraction","MINING",BuildIcon::OreDriller,BuildCategory::Structures,ResourceKind::OreDriller),
        {"conveyor_belt","Conveyor Belt","Mining / Ore Driller to Processing Facility or Factory","MINING",
         BuildIcon::ConveyorBelt,BuildCategory::Structures,BuildObjectKind::Utility,HabitatKind::AirLock,
         HabitatSizeTier::Standard,ResourceKind::OreDriller,UtilityKind::ConveyorBelt},

        utility(),
        walkway(),
        resource("robot_workshop","Robot Workshop","Outdoor / Four Service Bays / Supply Required","SUPPLY",BuildIcon::RobotWorkshop,BuildCategory::Structures,ResourceKind::RobotWorkshop),

        resource("antenna","Antenna","Facility / Communications","TRADE",BuildIcon::Antenna,BuildCategory::Structures,ResourceKind::Antenna),
        resource("landing_pad","Landing Pad","Facility / Transport","TRADE",BuildIcon::LandingPad,BuildCategory::Structures,ResourceKind::LandingPad)
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
