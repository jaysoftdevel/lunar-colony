#include "game/world/habitat_mesh.hpp"
#include "game/world/habitats/habitat_generators.hpp"

namespace game {

HabitatModelLibrary::HabitatModelLibrary(){
    meshes_[(size_t)HabitatKind::AirLock]=habitats::make_airlock();
    meshes_[(size_t)HabitatKind::OxygenGeneratorDome]=habitats::make_oxygen_dome();
    meshes_[(size_t)HabitatKind::CantineDome]=habitats::make_cantine_dome();
    meshes_[(size_t)HabitatKind::BioDome]=habitats::make_bio_dome();
    meshes_[(size_t)HabitatKind::CrewQuarters]=habitats::make_crew_quarters();
}

const HabitatMesh& HabitatModelLibrary::mesh(HabitatKind kind) const {
    return meshes_[(size_t)kind];
}

} // namespace game
