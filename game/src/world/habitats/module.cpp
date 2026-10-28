#include "game/world/habitats/habitat_module.hpp"
#include <stdexcept>

namespace game::habitats {

HabitatMesh HabitatModule::instantiate() const {
    Builder b;
    if(outer_) outer_(b);
    if(inner_) inner_(b);
    b.future_detail_pass(access_.walkable_floor_y);
    b.mesh.walkable_floor_y=access_.walkable_floor_y;
    b.mesh.crew_accessible=access_.crew_accessible;
    return std::move(b.mesh);
}

namespace {
constexpr StructureAccessProfile access(float floor,bool exterior=false){
    return {floor,true,true,exterior};
}

// Explicit deck heights replace the old triangle-area inference. These are the
// actual top surfaces rendered by the corresponding inner/foundation modules.
const HabitatModule kAirLock{HabitatKind::AirLock,"air_lock",access(0.80f,true),build_airlock_outer,build_airlock_inner};
const HabitatModule kOxygen{HabitatKind::OxygenGeneratorDome,"oxygen_generator_dome",access(1.19f),build_oxygen_dome_outer,build_oxygen_dome_inner};
const HabitatModule kCantine{HabitatKind::CantineDome,"cantine_dome",access(1.20f),build_cantine_dome_outer,build_cantine_dome_inner};
const HabitatModule kBio{HabitatKind::BioDome,"bio_dome",access(1.245f),build_bio_dome_outer,build_bio_dome_inner};
const HabitatModule kCrew{HabitatKind::CrewQuarters,"crew_quarters",access(1.03f),build_crew_quarters_outer,build_crew_quarters_inner};
const HabitatModule kProcessing{HabitatKind::ProcessingFacility,"processing_facility",access(1.14f),build_processing_facility_outer,build_processing_facility_inner};
const HabitatModule kFactory{HabitatKind::Factory,"factory",access(1.14f),build_factory_outer,build_factory_inner};
const HabitatModule kManufactory{HabitatKind::Manufactory,"manufactory",access(1.26f),build_manufactory_outer,build_manufactory_inner};
const HabitatModule kSickBay{HabitatKind::SickBay,"sick_bay",access(1.24f),build_sick_bay_outer,build_sick_bay_inner};
const HabitatModule kLaboratory{HabitatKind::Laboratory,"laboratory",access(1.34f),build_laboratory_outer,build_laboratory_inner};

const std::array<const HabitatModule*,(std::size_t)HabitatKind::Count> kModules{{
    &kAirLock,&kOxygen,&kCantine,&kBio,&kCrew,&kProcessing,&kFactory,&kManufactory,&kSickBay,&kLaboratory
}};
}

const HabitatModule& module(HabitatKind kind){
    const auto i=(std::size_t)kind;
    if(i>=kModules.size()) throw std::out_of_range("invalid habitat kind");
    return *kModules[i];
}

const std::array<const HabitatModule*,(std::size_t)HabitatKind::Count>& modules(){ return kModules; }

} // namespace game::habitats
