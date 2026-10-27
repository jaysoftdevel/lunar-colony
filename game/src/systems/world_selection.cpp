#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;

namespace {
float rotated_rect_distance_xz(Vec3 hit,Vec3 center,float rotation,float sx,float sz){
    const Vec3 local=rotate_y(hit-center,-rotation);
    const float dx=std::max(std::abs(local.x)-sx*0.5f,0.0f);
    const float dz=std::max(std::abs(local.z)-sz*0.5f,0.0f);
    return std::sqrt(dx*dx+dz*dz);
}

Vec3 closest_point_segment_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<1.0e-6f) return {a.x,0.0f,a.z};
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return {a.x+dx*t,0.0f,a.z+dz*t};
}

struct NetworkSnap {
    bool found{false};
    std::size_t index{0};
    Vec3 point{};
    float distance{std::numeric_limits<float>::max()};
};

template<class ConnectionList>
NetworkSnap nearest_network_snap(Vec3 hit,const ConnectionList& links,float radius){
    NetworkSnap best;
    best.distance=radius;
    for(std::size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(std::size_t p=0;p+1<path.size();++p){
            const Vec3 q=closest_point_segment_xz(hit,path[p],path[p+1]);
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best.distance){ best={true,i,q,d}; }
        }
    }
    return best;
}
}

Rect GameApp::selection_panel_rect() const {
    return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,360.0f,218.0f};
}

Rect GameApp::ground_resource_panel_rect() const {
    if(selected_ground_resource_<0 || (std::size_t)selected_ground_resource_>=ground_resource_piles_.size()) return {};
    const auto& pile=ground_resource_piles_[(std::size_t)selected_ground_resource_];
    const float aspect=(float)viewport_width_/std::max(1,viewport_height_);
    const Mat4 m=camera_.view_projection(aspect);
    const Vec3 p=pile.position+Vec3{0.0f,0.85f,0.0f};
    const float cx=m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12];
    const float cy=m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13];
    const float cw=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];
    if(cw<=0.001f) return selection_panel_rect();
    const float nx=cx/cw, ny=cy/cw;
    const float logical_w=(float)viewport_width_/ui_scale_;
    const float logical_h=(float)viewport_height_/ui_scale_;
    float x=(nx*0.5f+0.5f)*logical_w+18.0f;
    float y=(1.0f-(ny*0.5f+0.5f))*logical_h-54.0f;
    constexpr float w=228.0f,h=126.0f;
    if(x+w>logical_w-8.0f) x=(nx*0.5f+0.5f)*logical_w-w-18.0f;
    x=std::clamp(x,8.0f,logical_w-w-8.0f);
    y=std::clamp(y,STATUS_BAR_H+8.0f,logical_h-BUILD_BAR_H-h-8.0f);
    return {x,y,w,h};
}

bool GameApp::mouse_over_selection_panel(const InputState& input) const {
    if(selection_kind_==SelectionKind::None) return false;
    const Rect p=selection_kind_==SelectionKind::GroundResource?ground_resource_panel_rect():selection_panel_rect();
    return p.contains((float)input.mouse_x/ui_scale_,(float)input.mouse_y/ui_scale_);
}

void GameApp::clear_world_selection(){
    selection_kind_=SelectionKind::None;
    selected_habitat_=-1;
    selected_corridor_=-1;
    selected_resource_=-1;
    selected_ground_resource_=-1;
    selected_utility_=-1;
    selected_walkway_=-1;
}

int GameApp::pick_habitat(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h);
        const float a=radians(h.rotation_degrees);
        const float c=std::cos(a),si=std::sin(a);
        const float dx=hit.x-h.position.x,dz=hit.z-h.position.z;
        const float lx=dx*c-dz*si;
        const float lz=dx*si+dz*c;
        if(std::abs(lx)<=mesh.footprint_x*0.5f && std::abs(lz)<=mesh.footprint_z*0.5f){
            const float d=dx*dx+dz*dz;
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_resource(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        const auto& mesh=resource_models_.mesh(item);
        const float a=radians(item.rotation_degrees);
        const float c=std::cos(a),si=std::sin(a);
        const float dx=hit.x-item.position.x,dz=hit.z-item.position.z;
        const float lx=dx*c-dz*si;
        const float lz=dx*si+dz*c;
        if(std::abs(lx)<=mesh.footprint_x*0.5f && std::abs(lz)<=mesh.footprint_z*0.5f){
            const float d=dx*dx+dz*dz;
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_ground_resource(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(std::size_t i=0;i<ground_resource_piles_.size();++i){
        const auto& pile=ground_resource_piles_[i];
        const float dx=hit.x-pile.position.x,dz=hit.z-pile.position.z;
        const float d=std::sqrt(dx*dx+dz*dz);
        if(d<=pile.radius+radius && d<best_distance){ best_distance=d; best=(int)i; }
    }
    return best;
}

int GameApp::pick_walkway(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& links=walkway_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_utility(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& links=utility_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

std::optional<UtilityBuildAnchor> GameApp::utility_anchor_at(Vec3 hit,UtilityKind kind) const {
    // Direct footprint hit always wins. Keep the real cursor hit in the anchor:
    // the structure still snaps as a whole, while the hit position is used to
    // fine-adjust the exact utility attachment point on its mesh surface.
    const int resource=pick_resource(hit);
    if(resource>=0){
        UtilityBuildAnchor a{UtilityEndpointKind::Resource,(size_t)resource,hit};
        if(utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) return a;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        UtilityBuildAnchor a{UtilityEndpointKind::Habitat,(size_t)habitat,hit};
        if(utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) return a;
    }

    // Supply channels are deliberately easy to attach: when the pointer is near
    // a resource or habitat, snap to the closest compatible structure even if
    // it is just outside the footprint. The original cursor hit is retained for
    // exact mesh-surface port placement.
    if(kind==UtilityKind::SupplyChannel){
        constexpr float SNAP_MARGIN=2.25f;
        float best=SNAP_MARGIN;
        std::optional<UtilityBuildAnchor> snapped;
        for(std::size_t i=0;i<placed_resources_.size();++i){
            UtilityBuildAnchor a{UtilityEndpointKind::Resource,i,hit};
            if(!utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) continue;
            const auto& mesh=resource_models_.mesh(placed_resources_[i]);
            const float d=rotated_rect_distance_xz(hit,placed_resources_[i].position,placed_resources_[i].rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(d<best){ best=d; snapped=a; }
        }
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            UtilityBuildAnchor a{UtilityEndpointKind::Habitat,i,hit};
            if(!utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) continue;
            const auto& mesh=habitat_models_.mesh(placed_habitats_[i]);
            const float d=rotated_rect_distance_xz(hit,placed_habitats_[i].position,placed_habitats_[i].rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(d<best){ best=d; snapped=a; }
        }
        if(snapped) return snapped;
    }

    // Existing supply runs are valid manifold starts/ends. Snap the open anchor
    // to the actual tray centerline so a new branch creates a true junction.
    const auto utility_snap=nearest_network_snap(hit,utility_network_.connections(),1.35f);
    if(utility_snap.found) return UtilityBuildAnchor{UtilityEndpointKind::Open,0,utility_snap.point};

    // Free endpoints are supported as deliberate capped service stubs. This
    // also guarantees that the pre-click one-section sample is always present.
    return UtilityBuildAnchor{UtilityEndpointKind::Open,0,hit};
}

std::optional<WalkwayBuildAnchor> GameApp::walkway_anchor_at(Vec3 hit,bool source_only) const {
    // Structures win when the pointer is actually on them. Habitat resolution
    // then snaps to a built-in entrance or generates a valid new entrance.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,(size_t)habitat,hit};
    const int resource=pick_resource(hit);
    if(resource>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Resource,(size_t)resource,hit};

    // Existing traversable connections are magnetic intersection targets. On
    // the destination click prefer a candidate that the real walkway solver can
    // intersect. If none is legal, keep the closest attempted snap so the full
    // blueprint is shown in red instead of silently falling back to open ground.
    const auto walkway_snap=nearest_network_snap(hit,walkway_network_.connections(),1.65f);
    const auto corridor_snap=nearest_network_snap(hit,corridor_network_.connections(),1.75f);
    if(source_only || !walkway_build_start_){
        if(walkway_snap.found && (!corridor_snap.found || walkway_snap.distance<=corridor_snap.distance))
            return WalkwayBuildAnchor{WalkwayEndpointKind::Walkway,walkway_snap.index,walkway_snap.point};
        if(corridor_snap.found)
            return WalkwayBuildAnchor{WalkwayEndpointKind::Corridor,corridor_snap.index,corridor_snap.point};
    }else{
        std::optional<WalkwayBuildAnchor> valid;
        float valid_distance=std::numeric_limits<float>::max();
        std::optional<WalkwayBuildAnchor> attempted;
        float attempted_distance=std::numeric_limits<float>::max();
        const auto consider=[&](WalkwayEndpointKind kind,const NetworkSnap& snap){
            if(!snap.found) return;
            WalkwayBuildAnchor candidate{kind,snap.index,snap.point};
            if(snap.distance<attempted_distance){ attempted=candidate; attempted_distance=snap.distance; }
            const auto preview=walkway_network_.preview_connection(*walkway_build_start_,candidate,corridor_network_,utility_network_,
                                                                   placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            if(preview && snap.distance<valid_distance){ valid=candidate; valid_distance=snap.distance; }
        };
        consider(WalkwayEndpointKind::Walkway,walkway_snap);
        consider(WalkwayEndpointKind::Corridor,corridor_snap);
        if(valid) return valid;
        if(attempted) return attempted;
    }

    // Near a structure, keep snapping even when the pointer sits just outside
    // its footprint. The exact hit is retained so the entrance resolver can
    // choose the nearest built-in hatch or generate a new one at that side.
    constexpr float SNAP_MARGIN=2.25f;
    float best=SNAP_MARGIN;
    std::optional<WalkwayBuildAnchor> snapped;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i]; const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,i,hit}; }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& r=placed_resources_[i]; const auto& mesh=resource_models_.mesh(r);
        const float d=rotated_rect_distance_xz(hit,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Resource,i,hit}; }
    }
    if(snapped) return snapped;

    return WalkwayBuildAnchor{WalkwayEndpointKind::Open,0,hit};
}

int GameApp::pick_corridor(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& path=corridors[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

CorridorBuildAnchor GameApp::corridor_anchor_at(Vec3 hit) const {
    // Storage domes expose four fixed pressure docking faces. They participate
    // in corridor snapping like habitats, but remain resource/storage objects.
    const int resource=pick_resource(hit);
    if(resource>=0 && placed_resources_[(std::size_t)resource].kind==ResourceKind::StorageDome)
        return {CorridorEndpointKind::Resource,(std::size_t)resource,hit};
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)habitat,hit};

    constexpr float STRUCTURE_SNAP_MARGIN=2.10f;
    float best=STRUCTURE_SNAP_MARGIN;
    std::optional<CorridorBuildAnchor> snapped;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=CorridorBuildAnchor{CorridorEndpointKind::Habitat,i,hit}; }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        const float d=rotated_rect_distance_xz(hit,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=CorridorBuildAnchor{CorridorEndpointKind::Resource,i,hit}; }
    }
    if(snapped) return *snapped;

    const int corridor=pick_corridor(hit,1.65f);
    if(corridor>=0) return corridor_network_.corridor_anchor((size_t)corridor,hit);
    return {CorridorEndpointKind::OpenDoor,0,{std::round(hit.x),0.0f,std::round(hit.z)}};
}

void GameApp::select_world_at(int screen_x,int screen_y){
    Vec3 hit{};
    if(!camera_.intersect_playfield(screen_x,screen_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int ground_resource=pick_ground_resource(hit);
    if(ground_resource>=0){
        selection_kind_=SelectionKind::GroundResource;
        selected_ground_resource_=ground_resource;
        selected_habitat_=-1; selected_corridor_=-1; selected_resource_=-1; selected_utility_=-1; selected_walkway_=-1;
        return;
    }
    const int utility=pick_utility(hit);
    if(utility>=0){
        selection_kind_=SelectionKind::Utility;
        selected_utility_=utility;
        selected_corridor_=-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1;
        return;
    }
    const int walkway=pick_walkway(hit);
    if(walkway>=0){
        selection_kind_=SelectionKind::Walkway;
        selected_walkway_=walkway;
        selected_utility_=-1;
        selected_corridor_=-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_ground_resource_=-1;
        return;
    }
    const int corridor=pick_corridor(hit);
    if(corridor>=0){
        selection_kind_=SelectionKind::Corridor;
        selected_corridor_=corridor;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1;
        return;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        selection_kind_=SelectionKind::Habitat;
        selected_habitat_=habitat;
        selected_corridor_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1;
        return;
    }
    const int resource=pick_resource(hit);
    if(resource>=0){
        selection_kind_=SelectionKind::Resource;
        selected_resource_=resource;
        selected_habitat_=-1;
        selected_corridor_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1;
        return;
    }
    clear_world_selection();
}



void GameApp::drop_structure_resources(Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used){
    const std::array<MaterialResource,4> materials{MaterialResource::Iron,MaterialResource::Copper,MaterialResource::Silicon,MaterialResource::Plastic};
    const float ox=std::clamp(footprint_x*0.22f,0.42f,1.65f);
    const float oz=std::clamp(footprint_z*0.22f,0.42f,1.65f);
    const std::array<Vec3,4> local{{{-ox,0.0f,-oz},{ox,0.0f,-oz},{-ox,0.0f,oz},{ox,0.0f,oz}}};
    for(std::size_t i=0;i<materials.size();++i){
        const std::int64_t amount=used.amount(materials[i]);
        if(amount<=0) continue;
        GroundResourcePile pile;
        pile.resource=materials[i];
        pile.amount=amount;
        pile.position=position+rotate_y(local[i],rotation_degrees);
        pile.position.y=0.0f;
        pile.radius=std::clamp(0.30f+0.085f*std::log10((float)amount+1.0f),0.34f,0.72f);
        switch(materials[i]){
            case MaterialResource::Iron: pile.condition=0.94f; break;
            case MaterialResource::Copper: pile.condition=0.97f; break;
            case MaterialResource::Silicon: pile.condition=0.90f; break;
            case MaterialResource::Plastic: pile.condition=0.84f; break;
            case MaterialResource::Count: pile.condition=1.0f; break;
        }
        ground_resource_piles_.push_back(pile);
    }
}

bool GameApp::move_ground_resource_to_storage(std::size_t pile_index){
    if(pile_index>=ground_resource_piles_.size()) return false;
    const auto pile=ground_resource_piles_[pile_index];
    for(auto& structure:placed_resources_){
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        const auto used=structure.stored_resources.iron+structure.stored_resources.copper+structure.stored_resources.silicon+structure.stored_resources.plastic;
        if(used+pile.amount>storage_dome_capacity_kg(structure.storage_size)) continue;
        structure.stored_resources.amount(pile.resource)+=pile.amount;
        ResourceBundle added{};
        added.amount(pile.resource)=pile.amount;
        resources_.add(added);
        ground_resource_piles_.erase(ground_resource_piles_.begin()+(std::ptrdiff_t)pile_index);
        return true;
    }
    return false;
}

void GameApp::delete_selected_world_object(){
    switch(selection_kind_){
        case SelectionKind::Habitat:
            if(selected_habitat_>=0 && (std::size_t)selected_habitat_<placed_habitats_.size()){
                const std::size_t index=(std::size_t)selected_habitat_;
                const auto& removed=placed_habitats_[index];
                const auto& mesh=habitat_models_.mesh(removed);
                drop_structure_resources(removed.position,removed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,
                                         removed.resource_plan.consumed(removed.construction.step));
                corridor_network_.remove_habitat(index);
                utility_network_.remove_habitat(index);
                walkway_network_.remove_habitat(index);
                placed_habitats_.erase(placed_habitats_.begin()+(std::ptrdiff_t)index);
            }
            break;
        case SelectionKind::Resource:
            if(selected_resource_>=0 && (std::size_t)selected_resource_<placed_resources_.size()){
                const std::size_t index=(std::size_t)selected_resource_;
                const auto& removed=placed_resources_[index];
                const auto& mesh=resource_models_.mesh(removed);
                if(removed.kind==ResourceKind::StorageDome && !removed.stored_resources.empty()){
                    // Stored inventory returns to physical ground boxes before the dome disappears.
                    drop_structure_resources(removed.position,removed.rotation_degrees,mesh.footprint_x*0.72f,mesh.footprint_z*0.72f,removed.stored_resources);
                    ResourceBundle subtract{-removed.stored_resources.iron,-removed.stored_resources.copper,-removed.stored_resources.silicon,-removed.stored_resources.plastic};
                    resources_.add(subtract);
                }
                drop_structure_resources(removed.position,removed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,
                                         removed.resource_plan.consumed(removed.construction.step));
                corridor_network_.remove_resource(index);
                utility_network_.remove_resource(index);
                walkway_network_.remove_resource(index);
                placed_resources_.erase(placed_resources_.begin()+(std::ptrdiff_t)index);
            }
            break;
        case SelectionKind::GroundResource:
            if(selected_ground_resource_>=0 && (std::size_t)selected_ground_resource_<ground_resource_piles_.size()){
                if(!debug_frame_enabled_ || !move_ground_resource_to_storage((std::size_t)selected_ground_resource_)) return;
            }else return;
            break;
        case SelectionKind::Corridor:
            if(selected_corridor_>=0 && (std::size_t)selected_corridor_<corridor_network_.connections().size()){
                const std::size_t index=(std::size_t)selected_corridor_;
                walkway_network_.remove_corridor(index);
                corridor_network_.remove_corridor(index);
            }
            break;
        case SelectionKind::Utility:
            if(selected_utility_>=0 && (std::size_t)selected_utility_<utility_network_.connections().size()){
                const std::size_t index=(std::size_t)selected_utility_;
                walkway_network_.remove_utility(index);
                utility_network_.remove_connection(index);
            }
            break;
        case SelectionKind::Walkway:
            if(selected_walkway_>=0 && (std::size_t)selected_walkway_<walkway_network_.connections().size())
                walkway_network_.remove_connection((std::size_t)selected_walkway_);
            break;
        case SelectionKind::None:
            return;
    }

    // Any operation that stores world indices must be cancelled after a delete;
    // deletion can compact vectors and therefore invalidate those indices.
    corridor_link_source_=-1;
    corridor_build_start_.reset();
    utility_build_start_.reset();
    walkway_build_start_.reset();
    clear_world_selection();
}

} // namespace game
