#include "game/world/resource_builder.hpp"
#include "game/world/storage_loading_gate.hpp"

namespace game::resources {
ResourceMesh make_storage_gate(StorageDomeSize size){
    Builder b;
    PlacedResource owner;owner.kind=ResourceKind::StorageDome;owner.storage_size=size;owner.storage_loading_face=1;
    const float depth=length(storage_loading::mouth(owner)-storage_loading::wall(owner));
    const float w=storage_loading::kHalfWidth,h=storage_loading::kHeight;
    b.mesh.footprint_x=depth+0.20f;b.mesh.footprint_z=2*w+0.20f;b.mesh.height=h+0.28f;
    b.box(b.mesh.foundation,{0,0.14f,0},{depth+0.20f,0.18f,2*w+0.18f});
    b.box(b.mesh.body,{0,h+0.18f,0},{depth+0.20f,0.16f,2*w+0.18f});
    for(float sign:{-1.0f,1.0f})
        b.box(b.mesh.body,{0,h*0.5f+0.15f,sign*(w+0.05f)},{depth+0.20f,h,0.12f});
    // Inner bulkhead keeps the warehouse sealed while the outer gate operates.
    b.box(b.mesh.body,{-depth*0.5f,h*0.5f+0.15f,0},{0.12f,h,2*w});
    for(int i=0;i<6;++i){
        const float x=-depth*0.5f+depth*(float)(i+1)/7;
        b.cylinder_z(b.mesh.details,{x,0.25f,0},2*w-0.25f,0.04f,6);
    }
    return std::move(b.mesh);
}
}
