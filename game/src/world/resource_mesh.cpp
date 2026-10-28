#include "game/world/resource_mesh.hpp"
#include "game/world/resources/resource_generators.hpp"
#include "game/world/resource_builder.hpp"

namespace game {
ResourceModelLibrary::ResourceModelLibrary(){
    meshes_[(size_t)ResourceKind::SolarPanel]=resources::refine_future_resource(resources::make_solar_panel(),ResourceKind::SolarPanel);
    meshes_[(size_t)ResourceKind::WaterDriller]=resources::refine_future_resource(resources::make_water_driller(),ResourceKind::WaterDriller);
    meshes_[(size_t)ResourceKind::PowerStorage]=resources::refine_future_resource(resources::make_power_storage(),ResourceKind::PowerStorage);
    meshes_[(size_t)ResourceKind::WaterStorage]=resources::refine_future_resource(resources::make_water_storage(),ResourceKind::WaterStorage);
    meshes_[(size_t)ResourceKind::OreDriller]=resources::refine_future_resource(resources::make_ore_driller(),ResourceKind::OreDriller);
    meshes_[(size_t)ResourceKind::WaterPurifier]=resources::refine_future_resource(resources::make_water_purifier(),ResourceKind::WaterPurifier);
    meshes_[(size_t)ResourceKind::Antenna]=resources::refine_future_resource(resources::make_antenna(),ResourceKind::Antenna);
    meshes_[(size_t)ResourceKind::LandingPad]=resources::refine_future_resource(resources::make_landing_pad(),ResourceKind::LandingPad);
    storage_domes_[(size_t)StorageDomeSize::Small]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Small),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Mid]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Mid),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Big]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Big),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Huge]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Huge),ResourceKind::StorageDome);
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
