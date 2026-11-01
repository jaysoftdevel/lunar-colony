#include "game/world/resource_mesh.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <map>
#include <tuple>
#include "game/world/resources/resource_generators.hpp"
#include "game/world/resource_builder.hpp"

namespace game {
namespace {
ResourceMesh scale_xz(const ResourceMesh& source,float scale){
    ResourceMesh out=source;
    auto scale_vertices=[scale](std::vector<eng::Vec3>& vertices){
        for(auto& v:vertices){ v.x*=scale; v.z*=scale; }
    };
    scale_vertices(out.body);
    scale_vertices(out.foundation);
    scale_vertices(out.details);
    scale_vertices(out.edges);
    out.footprint_x*=scale;
    out.footprint_z*=scale;
    // Intentionally leave out.height and all Y coordinates untouched.
    return out;
}
}

ResourceModelLibrary::ResourceModelLibrary(){
    meshes_[(size_t)ResourceKind::SolarPanel]=resources::refine_future_resource(resources::make_solar_panel(),ResourceKind::SolarPanel);
    meshes_[(size_t)ResourceKind::WaterDriller]=resources::refine_future_resource(resources::make_water_driller(),ResourceKind::WaterDriller);
    meshes_[(size_t)ResourceKind::PowerStorage]=resources::refine_future_resource(resources::make_power_storage(),ResourceKind::PowerStorage);
    meshes_[(size_t)ResourceKind::WaterStorage]=resources::refine_future_resource(resources::make_water_storage(),ResourceKind::WaterStorage);
    meshes_[(size_t)ResourceKind::OreDriller]=resources::refine_future_resource(resources::make_ore_driller(),ResourceKind::OreDriller);
    meshes_[(size_t)ResourceKind::WaterPurifier]=resources::refine_future_resource(resources::make_water_purifier(),ResourceKind::WaterPurifier);
    meshes_[(size_t)ResourceKind::Antenna]=resources::refine_future_resource(resources::make_antenna(),ResourceKind::Antenna);
    meshes_[(size_t)ResourceKind::LandingPad]=resources::refine_future_resource(resources::make_landing_pad(),ResourceKind::LandingPad);
    meshes_[(size_t)ResourceKind::RobotWorkshop]=resources::make_robot_workshop();
    storage_domes_[(size_t)StorageDomeSize::Small]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Small),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Mid]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Mid),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Big]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Big),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Huge]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Huge),ResourceKind::StorageDome);
    for(std::size_t i=0;i<(std::size_t)StorageDomeSize::Count;++i) storage_gates_[i]=resources::make_storage_gate((StorageDomeSize)i);
    meshes_[(size_t)ResourceKind::StorageGate]=storage_gates_[0];
    meshes_[(size_t)ResourceKind::StorageDome]=storage_domes_[(size_t)StorageDomeSize::Small];
    for(std::size_t i=0;i<(std::size_t)ResourceKind::Count;++i){
        const auto kind=(ResourceKind)i;
        small_meshes_[i]=(kind==ResourceKind::StorageDome || kind==ResourceKind::RobotWorkshop || kind==ResourceKind::StorageGate)?meshes_[i]:scale_xz(meshes_[i],0.76f);
    }
}

const ResourceMesh& ResourceModelLibrary::mesh(ResourceKind kind,ResourceSizeTier size) const {
    if(kind==ResourceKind::StorageDome) return meshes_[(size_t)kind];
    return size==ResourceSizeTier::Small?small_meshes_[(size_t)kind]:meshes_[(size_t)kind];
}

const ResourceMesh& ResourceModelLibrary::mesh(ResourceKind kind,StorageDomeSize storage_size) const {
    if(kind==ResourceKind::StorageDome) return storage_domes_[(size_t)storage_size];
    if(kind==ResourceKind::StorageGate) return storage_gates_[(size_t)storage_size];
    return meshes_[(size_t)kind];
}
const ResourceMesh& ResourceModelLibrary::mesh(const PlacedResource& placed) const {
    if(placed.kind==ResourceKind::StorageDome)return mesh(placed.kind,placed.storage_size);
    if(placed.kind!=ResourceKind::StorageGate)return mesh(placed.kind,placed.size);
    using Key=std::tuple<int,int,float>;
    // Per-thread derived geometry avoids writes shared by navigation workers.
    static thread_local std::map<Key,ResourceMesh> cache;
    const Key key{(int)placed.storage_size,placed.storage_loading_face,placed.storage_gate_angle_degrees};
    const auto found=cache.find(key);if(found!=cache.end())return found->second;
    ResourceMesh model=mesh(placed.kind,placed.storage_size);
    PlacedResource owner;owner.kind=ResourceKind::StorageDome;owner.storage_size=placed.storage_size;
    owner.storage_loading_face=placed.storage_loading_face;owner.storage_gate_angle_degrees=placed.storage_gate_angle_degrees;
    const float desired=eng::length(storage_loading::mouth(owner)-storage_loading::wall(owner));
    const float ratio=desired/std::max(0.01f,model.footprint_x-0.20f);
    for(auto* layer:{&model.body,&model.foundation,&model.details,&model.edges})for(auto& v:*layer)v.x*=ratio;
    model.footprint_x*=ratio;
    return cache.emplace(key,std::move(model)).first->second;
}

}
