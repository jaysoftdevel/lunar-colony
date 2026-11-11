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
    struct Generator { ResourceKind kind;ResourceMesh (*make)();bool refine=true; };
    constexpr Generator generators[]{
        {ResourceKind::SolarPanel,resources::make_solar_panel},
        {ResourceKind::WaterDriller,resources::make_water_driller},
        {ResourceKind::PowerStorage,resources::make_power_storage},
        {ResourceKind::WaterStorage,resources::make_water_storage},
        {ResourceKind::OreDriller,resources::make_ore_driller},
        {ResourceKind::WaterPurifier,resources::make_water_purifier},
        {ResourceKind::Antenna,resources::make_antenna},
        {ResourceKind::LandingPad,resources::make_landing_pad},
        {ResourceKind::RobotWorkshop,resources::make_robot_workshop,false},
        {ResourceKind::HydrogenGenerator,resources::make_hydrogen_generator},
        {ResourceKind::HydrogenTank,resources::make_hydrogen_tank},
        {ResourceKind::OxygenTank,make_oxygen_storage,false}
    };
    constexpr auto base_count=std::size(generators);
    auto& pool=eng::TaskPool::shared();
    pool.parallel_for(base_count+storage_domes_.size(),[&](std::size_t i){
        if(i<base_count){
            const auto& generator=generators[i];
            auto mesh=generator.make();
            if(generator.refine) mesh=resources::refine_future_resource(std::move(mesh),generator.kind);
            meshes_[static_cast<std::size_t>(generator.kind)]=std::move(mesh);
        }else{
            const auto index=i-base_count;
            const auto size=static_cast<StorageDomeSize>(index);
            storage_domes_[index]=resources::refine_future_resource(resources::make_storage_dome(size),ResourceKind::StorageDome);
            storage_gates_[index]=resources::make_storage_gate(size);
        }
    });
    meshes_[(size_t)ResourceKind::StorageGate]=storage_gates_[0];
    meshes_[(size_t)ResourceKind::StorageDome]=storage_domes_[(size_t)StorageDomeSize::Small];
    pool.parallel_for(small_meshes_.size(),[&](std::size_t i){
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
    });
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
