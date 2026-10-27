#include "game/world/resource_mesh.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game {
ResourceModelLibrary::ResourceModelLibrary(){
    meshes_[(size_t)ResourceKind::SolarPanel]=resources::make_solar_panel();
    meshes_[(size_t)ResourceKind::WaterDriller]=resources::make_water_driller();
    meshes_[(size_t)ResourceKind::PowerStorage]=resources::make_power_storage();
    meshes_[(size_t)ResourceKind::WaterStorage]=resources::make_water_storage();
    meshes_[(size_t)ResourceKind::OreDriller]=resources::make_ore_driller();
    meshes_[(size_t)ResourceKind::WaterPurifier]=resources::make_water_purifier();
    meshes_[(size_t)ResourceKind::Antenna]=resources::make_antenna();
    meshes_[(size_t)ResourceKind::LandingPad]=resources::make_landing_pad();
    storage_domes_[(size_t)StorageDomeSize::Small]=resources::make_storage_dome(StorageDomeSize::Small);
    storage_domes_[(size_t)StorageDomeSize::Mid]=resources::make_storage_dome(StorageDomeSize::Mid);
    storage_domes_[(size_t)StorageDomeSize::Big]=resources::make_storage_dome(StorageDomeSize::Big);
    storage_domes_[(size_t)StorageDomeSize::Huge]=resources::make_storage_dome(StorageDomeSize::Huge);
    meshes_[(size_t)ResourceKind::StorageDome]=storage_domes_[(size_t)StorageDomeSize::Small];
}

const ResourceMesh& ResourceModelLibrary::mesh(ResourceKind kind) const {
    return meshes_[(size_t)kind];
}

const ResourceMesh& ResourceModelLibrary::mesh(ResourceKind kind,StorageDomeSize storage_size) const {
    if(kind==ResourceKind::StorageDome) return storage_domes_[(size_t)storage_size];
    return meshes_[(size_t)kind];
}
}
