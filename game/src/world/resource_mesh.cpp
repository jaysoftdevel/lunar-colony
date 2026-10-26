#include "game/world/resource_mesh.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game {
ResourceModelLibrary::ResourceModelLibrary(){
    meshes_[(size_t)ResourceKind::SolarPanel]=resources::make_solar_panel();
    meshes_[(size_t)ResourceKind::WaterDriller]=resources::make_water_driller();
    meshes_[(size_t)ResourceKind::PowerStorage]=resources::make_power_storage();
    meshes_[(size_t)ResourceKind::WaterStorage]=resources::make_water_storage();
}

const ResourceMesh& ResourceModelLibrary::mesh(ResourceKind kind) const {
    return meshes_[(size_t)kind];
}
}
