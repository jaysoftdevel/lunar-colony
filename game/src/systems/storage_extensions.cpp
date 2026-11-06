#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/gate_area.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/systems/construction_costs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace game {
namespace {
float gate_rotation(const PlacedResource& owner,int face){
    const auto normal=storage_loading::normal(owner,face);
    return std::atan2(-normal.z,normal.x)*180.0f/3.14159265358979323846f;
}
eng::Vec3 gate_position(const PlacedResource& owner,int face){
    return (storage_loading::wall(owner,face)+storage_loading::mouth(owner,face))*0.5f;
}
}

// Compatibility migration for pre-0.43.1 saves: those domes stored one cargo
// gate directly on the dome while only the additional two gates were child
// construction objects. Convert that persisted primary face into a normal
// StorageGate child. New domes start with storage_loading_face == -1 and are
// therefore untouched by this function.
void GameApp::assign_storage_loading_faces(){
    std::uint32_t gate_serial=0;
    std::uint64_t next_sequence=next_blueprint_placement_sequence_;
    for(const auto& item:placed_resources_){
        next_sequence=std::max(next_sequence,item.placement_sequence+1);
        if(item.kind==ResourceKind::StorageGate) gate_serial=std::max(gate_serial,item.instance_serial);
    }

    struct Pending {std::size_t owner;int face;};
    std::vector<Pending> migrations;
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& owner=placed_resources_[i];
        if(owner.kind!=ResourceKind::StorageDome || owner.storage_loading_face<0) continue;
        if(!owner.placement_sequence) owner.placement_sequence=next_sequence++;
        bool represented=false;
        for(const auto& child:placed_resources_){
            if(child.kind==ResourceKind::StorageGate &&
               child.storage_parent_sequence==owner.placement_sequence &&
               child.storage_loading_face==owner.storage_loading_face){
                represented=true;break;
            }
        }
        if(!represented) migrations.push_back({i,owner.storage_loading_face});
    }

    for(const auto migration:migrations){
        if(migration.owner>=placed_resources_.size()) continue;
        const auto& owner=placed_resources_[migration.owner];
        PlacedResource extension;
        extension.kind=ResourceKind::StorageGate;
        extension.size=ResourceSizeTier::Standard;
        extension.storage_size=owner.storage_size;
        extension.storage_loading_face=(std::int8_t)migration.face;
        extension.storage_parent_sequence=owner.placement_sequence;
        extension.placement_sequence=next_sequence++;
        extension.instance_serial=++gate_serial;
        extension.rotation_degrees=gate_rotation(owner,migration.face);
        extension.position=gate_position(owner,migration.face);
        extension.resource_plan=construction_costs::structure(extension.kind,resource_models_.mesh(extension));
        // Preserve the old integrated gate's construction state. Imported
        // blueprint templates have already been reset to step 0 at this point,
        // while normal saves keep their previous completion/progress.
        extension.construction=owner.construction;
        placed_resources_.push_back(std::move(extension));
    }
    next_blueprint_placement_sequence_=std::max(next_blueprint_placement_sequence_,next_sequence);
}

void GameApp::synchronize_storage_gates(){
    std::unordered_map<std::uint64_t,std::size_t> owners;
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageDome) continue;
        item.storage_loading_face=-1;
        item.storage_loading_ready=false;
        item.storage_loading_index=kNoRobotIndex;
        item.storage_extra_faces={-1,-1};
        item.storage_extra_ready={false,false};
        item.storage_extra_indices={kNoRobotIndex,kNoRobotIndex};
        if(item.placement_sequence) owners[item.placement_sequence]=i;
    }

    std::unordered_map<std::uint64_t,int> slot_counts;
    for(std::size_t i=0;i<placed_resources_.size();++i){
        auto& extension=placed_resources_[i];
        if(extension.kind!=ResourceKind::StorageGate) continue;
        const auto found=owners.find(extension.storage_parent_sequence);
        extension.storage_parent_index=found==owners.end()?kNoRobotIndex:found->second;
        if(found==owners.end()) continue;
        auto& owner=placed_resources_[found->second];
        int& slot=slot_counts[extension.storage_parent_sequence];
        if(slot>=3) continue;
        const bool ready=owner.construction.complete() && extension.construction.complete();
        if(slot==0){
            owner.storage_loading_face=extension.storage_loading_face;
            owner.storage_gate_angle_degrees=extension.storage_gate_angle_degrees;
            owner.storage_loading_ready=ready;
            owner.storage_loading_index=i;
        }else{
            const std::size_t extra=(std::size_t)(slot-1);
            owner.storage_extra_faces[extra]=extension.storage_loading_face;
            owner.storage_extra_angles[extra]=extension.storage_gate_angle_degrees;
            owner.storage_extra_ready[extra]=ready;
            owner.storage_extra_indices[extra]=i;
        }
        extension.position=gate_position(owner,extension.storage_loading_face);
        extension.rotation_degrees=gate_rotation(owner,extension.storage_loading_face);
        ++slot;
    }
}

bool GameApp::storage_gate_face_clear(const PlacedResource& owner,int face,std::optional<std::size_t> owner_index) const {
    if(owner.kind!=ResourceKind::StorageDome || face<0 || face>10) return false;
    for(int f:storage_loading::faces(owner))if(f>=0 && f!=face &&
        world_math::distance_xz(storage_loading::mouth(owner,f),storage_loading::mouth(owner,face))<4.0f)return false;
    const auto bay=storage_loading::apron(owner,face);
    for(int f:storage_loading::faces(owner))if(f>=0 && f!=face && world_math::overlaps(bay,storage_loading::apron(owner,f),storage_loading::kClearance))return false;
    const gate_area::Area pad{storage_loading::service(owner,face),storage_loading::normal(owner,face),1.0f,0.75f};
    for(std::size_t i=0;i<placed_habitats_.size();++i)for(const auto& a:gate_area::habitat(placed_habitats_[i],habitat_models_.mesh(placed_habitats_[i]),i,corridor_network_.connections()))if(world_math::overlaps(pad.bounds(),a.bounds(),0.2f))return false;
    for(std::size_t i=0;i<placed_resources_.size();++i)for(const auto& a:gate_area::storage(placed_resources_[i],resource_models_.mesh(placed_resources_[i]),i,corridor_network_.connections())){
        if(owner_index && i==*owner_index && world_math::distance_xz(a.center,pad.center)<0.25f)continue;
        if(world_math::overlaps(pad.bounds(),a.bounds(),0.2f))return false;
    }
    const float half=(float)playfield_.size()*0.5f;
    constexpr float clearance=0.45f;

    const float build_limit=half-geometry::kPlayfieldRobotPerimeterLane;
    for(float x:{-bay.half_x,bay.half_x}) for(float z:{-bay.half_z,bay.half_z})
        if(std::abs(bay.center.x+bay.axis_x.x*x+bay.axis_z.x*z)>build_limit ||
           std::abs(bay.center.y+bay.axis_x.y*x+bay.axis_z.y*z)>build_limit) return false;

    for(const auto& h:placed_habitats_){
        const auto& m=habitat_models_.mesh(h);
        if(world_math::overlaps(bay,world_math::footprint(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z),clearance)) return false;
    }
    for(std::size_t j=0;j<placed_resources_.size();++j){
        const auto& r=placed_resources_[j];
        if(owner_index && j==*owner_index) continue;
        if(owner.placement_sequence && r.kind==ResourceKind::StorageGate && r.storage_parent_sequence==owner.placement_sequence) continue;
        const auto& m=resource_models_.mesh(r);
        if(world_math::overlaps(bay,world_math::footprint(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z),clearance)) return false;
        if(r.kind==ResourceKind::RobotWorkshop && world_math::overlaps(bay,workshop::approach(r),clearance)) return false;
    }
    for(const auto& c:corridor_network_.connections()) for(std::size_t j=1;j<c.path.size();++j)
        if(storage_loading::segment_reserved_face(owner,c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+geometry::kRouteClearance,face)) return false;
    for(const auto& u:utility_network_.connections()) for(std::size_t j=1;j<u.path.size();++j)
        if(storage_loading::segment_reserved_face(owner,u.path[j-1],u.path[j],geometry::kSupplyTrayWidth*0.5f+0.25f,face)) return false;
    for(const auto& w:walkway_network_.connections()) for(std::size_t j=1;j<w.path.size();++j)
        if(storage_loading::segment_reserved_face(owner,w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.25f,face)) return false;
    for(const auto& pile:ground_resource_piles_)
        if(storage_loading::reserved_face(owner,pile.position,pile.radius+0.4f,face)) return false;
    return true;
}

int GameApp::available_storage_gate_face(std::size_t index,int requested) const {
    if(index>=placed_resources_.size()) return -1;
    const auto& owner=placed_resources_[index];
    if(owner.kind!=ResourceKind::StorageDome || storage_loading::gate_count(owner)>=3) return -1;

    for(int face:storage_loading::kCargoFaces){
        if(requested>=0 && requested!=face) continue;
        bool used=false;
        for(int existing:storage_loading::faces(owner)) if(existing==face){used=true;break;}
        if(used || !storage_gate_face_clear(owner,face,index)) continue;
        return face;
    }
    return -1;
}

bool GameApp::add_storage_gate(std::size_t index,int requested){
    assign_storage_loading_faces();
    synchronize_storage_gates();
    const int face=available_storage_gate_face(index,requested);
    if(face<0) return false;
    auto& owner=placed_resources_[index];
    if(!owner.placement_sequence) owner.placement_sequence=next_blueprint_placement_sequence_++;

    PlacedResource extension;
    extension.kind=ResourceKind::StorageGate;
    extension.size=ResourceSizeTier::Standard;
    extension.storage_size=owner.storage_size;
    extension.storage_loading_face=(std::int8_t)face;
    extension.storage_parent_sequence=owner.placement_sequence;
    extension.rotation_degrees=gate_rotation(owner,face);
    extension.position=gate_position(owner,face);
    for(const auto& r:placed_resources_) if(r.kind==ResourceKind::StorageGate)
        extension.instance_serial=std::max(extension.instance_serial,r.instance_serial);
    ++extension.instance_serial;
    extension.resource_plan=construction_costs::structure(extension.kind,resource_models_.mesh(extension));
    if(!can_start_construction(extension.resource_plan)) return false;
    finalize_placement(extension);
    placed_resources_.push_back(std::move(extension));
    synchronize_storage_gates();
    return true;
}
bool GameApp::add_storage_gate_at(std::size_t index,float local_angle){
    synchronize_storage_gates();if(index>=placed_resources_.size() || !std::isfinite(local_angle))return false;
    auto owner=placed_resources_[index];if(owner.kind!=ResourceKind::StorageDome || storage_loading::gate_count(owner)>=3)return false;
    int face=8;for(;face<=10;++face){bool used=false;for(int f:storage_loading::faces(owner))if(f==face)used=true;if(!used)break;}
    if(face>10)return false;
    storage_loading::set_angle(owner,face,local_angle);
    if(!storage_gate_face_clear(owner,face,index))return false;
    if(!owner.placement_sequence){placed_resources_[index].placement_sequence=next_blueprint_placement_sequence_++;owner.placement_sequence=placed_resources_[index].placement_sequence;}
    PlacedResource gate;gate.kind=ResourceKind::StorageGate;gate.storage_size=owner.storage_size;
    gate.storage_loading_face=(std::int8_t)face;gate.storage_gate_angle_degrees=local_angle;
    gate.storage_parent_sequence=owner.placement_sequence;
    gate.position=gate_position(owner,face);gate.rotation_degrees=gate_rotation(owner,face);
    for(const auto& r:placed_resources_)if(r.kind==ResourceKind::StorageGate)gate.instance_serial=std::max(gate.instance_serial,r.instance_serial);
    ++gate.instance_serial;gate.resource_plan=construction_costs::structure(gate.kind,resource_models_.mesh(gate));
    finalize_placement(gate);
    placed_resources_.push_back(std::move(gate));synchronize_storage_gates();return true;
}

}
