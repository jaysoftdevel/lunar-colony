#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/systems/construction_costs.hpp"
#include <unordered_map>

namespace game {
void GameApp::synchronize_storage_gates(){
    std::unordered_map<std::uint64_t,std::size_t> owners;
    for(std::size_t i=0;i<placed_resources_.size();++i){auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageDome) continue;
        item.storage_extra_faces={-1,-1};item.storage_extra_ready={false,false};
        item.storage_extra_indices={kNoRobotIndex,kNoRobotIndex};
        if(item.placement_sequence) owners[item.placement_sequence]=i;
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){auto& extension=placed_resources_[i];
        if(extension.kind!=ResourceKind::StorageGate) continue;
        const auto found=owners.find(extension.storage_parent_sequence);
        extension.storage_parent_index=found==owners.end()?kNoRobotIndex:found->second;
        if(found==owners.end()) continue;
        auto& owner=placed_resources_[found->second];
        for(std::size_t slot=0;slot<2;++slot) if(owner.storage_extra_faces[slot]<0){
            owner.storage_extra_faces[slot]=extension.storage_loading_face;
            owner.storage_extra_ready[slot]=owner.construction.complete() && extension.construction.complete();
            owner.storage_extra_indices[slot]=i;break;
        }
    }
}

int GameApp::available_storage_gate_face(std::size_t index,int requested) const {
    if(index>=placed_resources_.size()) return -1;
    const auto& owner=placed_resources_[index];
    if(owner.kind!=ResourceKind::StorageDome) return -1;
    int count=0;for(int f:owner.storage_extra_faces) count+=f>=0;
    if(count>=2) return -1;
    const int primary=storage_loading::face(owner)<0?1:storage_loading::face(owner);
    const int order[]{(primary+2)%4,(primary+1)%4,(primary+3)%4,primary%4};
    for(int face:order){
        if(requested>=0 && requested!=face) continue;
        bool used=face==primary;
        for(int f:owner.storage_extra_faces) if(face==f) used=true;
        const auto normal=storage_loading::normal(owner,face);
        for(const auto& c:corridor_network_.connections()) for(const auto* ep:{&c.endpoint_a,&c.endpoint_b})
            if(ep->kind==CorridorEndpointKind::Resource && ep->index==index && eng::dot(ep->door.normal,normal)>0.70f) used=true;
        if(used) continue;
        const auto bay=storage_loading::apron(owner,face);bool clear=true;
        const float half=(float)playfield_.size()*0.5f;
        for(float x:{-bay.half_x,bay.half_x}) for(float z:{-bay.half_z,bay.half_z})
            if(std::abs(bay.center.x+bay.axis_x.x*x+bay.axis_z.x*z)>half-0.10f ||
               std::abs(bay.center.y+bay.axis_x.y*x+bay.axis_z.y*z)>half-0.10f) clear=false;
        for(const auto& h:placed_habitats_){const auto& m=habitat_models_.mesh(h);
            if(world_math::overlaps(bay,world_math::footprint(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z),0.45f)) clear=false;}
        for(std::size_t j=0;j<placed_resources_.size();++j){
            const auto& r=placed_resources_[j];
            if(j==index || (r.kind==ResourceKind::StorageGate && r.storage_parent_sequence==owner.placement_sequence)) continue;
            const auto& m=resource_models_.mesh(r);
            if(world_math::overlaps(bay,world_math::footprint(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z),0.45f)) clear=false;
            if(r.kind==ResourceKind::RobotWorkshop && world_math::overlaps(bay,workshop::approach(r),0.45f)) clear=false;
        }
        for(const auto& c:corridor_network_.connections()) for(std::size_t j=1;j<c.path.size();++j)
            if(storage_loading::segment_reserved_face(owner,c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+geometry::kRouteClearance,face)) clear=false;
        for(const auto& w:walkway_network_.connections()) for(std::size_t j=1;j<w.path.size();++j)
            if(storage_loading::segment_reserved_face(owner,w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.25f,face)) clear=false;
        for(const auto& pile:ground_resource_piles_)
            if(storage_loading::reserved_face(owner,pile.position,pile.radius+0.4f,face)) clear=false;
        if(clear) return face;
    }
    return -1;
}

bool GameApp::add_storage_gate(std::size_t index,int requested){
    assign_storage_loading_faces();synchronize_storage_gates();
    const int face=available_storage_gate_face(index,requested);
    if(face<0) return false;
    auto& owner=placed_resources_[index];
    if(!owner.placement_sequence) owner.placement_sequence=next_blueprint_placement_sequence_++;
    PlacedResource extension;extension.kind=ResourceKind::StorageGate;extension.size=ResourceSizeTier::Standard;
    extension.storage_size=owner.storage_size;extension.storage_loading_face=(std::int8_t)face;
    extension.storage_parent_sequence=owner.placement_sequence;
    extension.placement_sequence=next_blueprint_placement_sequence_++;
    const auto normal=storage_loading::normal(owner,face);
    extension.rotation_degrees=std::atan2(-normal.z,normal.x)*180.0f/3.14159265358979323846f;
    extension.position=(storage_loading::wall(owner,face)+storage_loading::mouth(owner,face))*0.5f;
    for(const auto& r:placed_resources_) if(r.kind==ResourceKind::StorageGate)
        extension.instance_serial=std::max(extension.instance_serial,r.instance_serial);
    ++extension.instance_serial;
    extension.resource_plan=construction_costs::structure(extension.kind,resource_models_.mesh(extension));
    placed_resources_.push_back(std::move(extension));synchronize_storage_gates();
    return true;
}
}
