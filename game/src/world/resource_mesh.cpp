#include "game/world/resource_mesh.hpp"
#include "game/world/storage_loading_gate.hpp"
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
ResourceMesh make_oxygen_storage(){
    resources::Builder b;
    b.mesh.footprint_x=8.6f;
    b.mesh.footprint_z=7.0f;
    b.mesh.height=4.6f;
    b.box(b.mesh.foundation,{0,.19f,0},{8.3f,.38f,6.7f});
    for(float x:{-1.65f,1.65f}){
        b.cylinder_y(b.mesh.body,{x,2.03f,0},3.2f,1.25f,28);
        b.dome(b.mesh.body,{x,3.63f,0},1.25f,7,28,false,.62f);
        b.cylinder_y(b.mesh.details,{x,4.45f,0},.32f,.18f,12);
        for(float y:{.72f,2.85f}) b.cylinder_y(b.mesh.details,{x,y,0},.12f,1.32f,28);
        b.cylinder_z(b.mesh.details,{x,1.05f,-1.78f},1.4f,.12f,12);
    }
    b.cylinder_x(b.mesh.details,{0,1.05f,-2.4f},4.05f,.13f,12);
    b.box(b.mesh.body,{3.27f,.92f,0},{1.3f,1.08f,2.5f});
    b.cylinder_z(b.mesh.details,{3.26f,1.32f,-1.29f},.16f,.27f,16);
    return std::move(b.mesh);
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
    meshes_[(size_t)ResourceKind::HydrogenGenerator]=resources::refine_future_resource(resources::make_hydrogen_generator(),ResourceKind::HydrogenGenerator);
    meshes_[(size_t)ResourceKind::HydrogenTank]=resources::refine_future_resource(resources::make_hydrogen_tank(),ResourceKind::HydrogenTank);
    meshes_[(size_t)ResourceKind::OxygenTank]=make_oxygen_storage();
    storage_domes_[(size_t)StorageDomeSize::Small]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Small),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Mid]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Mid),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Big]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Big),ResourceKind::StorageDome);
    storage_domes_[(size_t)StorageDomeSize::Huge]=resources::refine_future_resource(resources::make_storage_dome(StorageDomeSize::Huge),ResourceKind::StorageDome);
    for(std::size_t i=0;i<(std::size_t)StorageDomeSize::Count;++i) storage_gates_[i]=resources::make_storage_gate((StorageDomeSize)i);
    meshes_[(size_t)ResourceKind::StorageGate]=storage_gates_[0];
    meshes_[(size_t)ResourceKind::StorageDome]=storage_domes_[(size_t)StorageDomeSize::Small];
    for(std::size_t i=0;i<(std::size_t)ResourceKind::Count;++i){
        const auto kind=(ResourceKind)i;
        if(kind==ResourceKind::HydrogenTank||kind==ResourceKind::OxygenTank){
            // Preserve physical vessel volume: 0.63^3 ~= 0.25, matching the
            // 2 t SMALL versus 8 t STANDARD rated cryogenic LH2 capacities.
            small_meshes_[i]=meshes_[i];
            const float s=(float)kHydrogenTankSmallLinearScale;
            auto scale=[s](std::vector<eng::Vec3>& vs){for(auto& v:vs)v=v*s;};
            auto& m=small_meshes_[i];
            scale(m.body);scale(m.foundation);scale(m.details);scale(m.edges);
            m.footprint_x*=s;m.footprint_z*=s;m.height*=s;
        }else{
            small_meshes_[i]=(kind==ResourceKind::StorageDome || kind==ResourceKind::RobotWorkshop || kind==ResourceKind::StorageGate)?meshes_[i]:scale_xz(meshes_[i],0.76f);
        }
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
    return mesh(placed.kind,placed.storage_size);
}

}
