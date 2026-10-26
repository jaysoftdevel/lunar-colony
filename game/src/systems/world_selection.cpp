#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
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
}

Rect GameApp::selection_panel_rect() const {
    return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,360.0f,218.0f};
}

bool GameApp::mouse_over_selection_panel(const InputState& input) const {
    if(selection_kind_==SelectionKind::None) return false;
    const Rect p=selection_panel_rect();
    return p.contains((float)input.mouse_x/ui_scale_,(float)input.mouse_y/ui_scale_);
}

void GameApp::clear_world_selection(){
    selection_kind_=SelectionKind::None;
    selected_habitat_=-1;
    selected_corridor_=-1;
    selected_resource_=-1;
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
        const auto& mesh=resource_models_.mesh(item.kind);
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
            const auto& mesh=resource_models_.mesh(placed_resources_[i].kind);
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
    return std::nullopt;
}

std::optional<WalkwayBuildAnchor> GameApp::walkway_anchor_at(Vec3 hit,bool source_only) const {
    (void)source_only;
    // Walkways may connect any two placed structures. Habitat endpoints always
    // resolve to a real/generated habitat entrance. External structures resolve
    // to an open access gate at their perimeter (the Landing Pad uses its fixed
    // approach ramp).  This keeps every walkway crew-accessible while remaining
    // explicitly non-pressurized.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,(size_t)habitat,hit};
    const int resource=pick_resource(hit);
    if(resource>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Resource,(size_t)resource,hit};

    constexpr float SNAP_MARGIN=2.25f;
    float best=SNAP_MARGIN;
    std::optional<WalkwayBuildAnchor> snapped;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i]; const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,i,hit}; }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& r=placed_resources_[i]; const auto& mesh=resource_models_.mesh(r.kind);
        const float d=rotated_rect_distance_xz(hit,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Resource,i,hit}; }
    }
    return snapped;
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
    // Match the supply-channel interaction: a habitat is selected as a whole,
    // but the original pointer hit is retained so the exact pressure-door
    // location can slide over the real hull surface before the click.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)habitat,hit};

    constexpr float STRUCTURE_SNAP_MARGIN=1.85f;
    float best=STRUCTURE_SNAP_MARGIN;
    int snapped_habitat=-1;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped_habitat=(int)i; }
    }
    if(snapped_habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)snapped_habitat,hit};

    const int corridor=pick_corridor(hit,1.65f);
    if(corridor>=0) return corridor_network_.corridor_anchor((size_t)corridor,hit);
    return {CorridorEndpointKind::OpenDoor,0,{std::round(hit.x),0.0f,std::round(hit.z)}};
}

void GameApp::select_world_at(int screen_x,int screen_y){
    Vec3 hit{};
    if(!camera_.intersect_playfield(screen_x,screen_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int utility=pick_utility(hit);
    if(utility>=0){
        selection_kind_=SelectionKind::Utility;
        selected_utility_=utility;
        selected_corridor_=-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_walkway_=-1;
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
        return;
    }
    clear_world_selection();
}


void GameApp::delete_selected_world_object(){
    switch(selection_kind_){
        case SelectionKind::Habitat:
            if(selected_habitat_>=0 && (std::size_t)selected_habitat_<placed_habitats_.size()){
                const std::size_t index=(std::size_t)selected_habitat_;
                corridor_network_.remove_habitat(index);
                utility_network_.remove_habitat(index);
                walkway_network_.remove_habitat(index);
                placed_habitats_.erase(placed_habitats_.begin()+(std::ptrdiff_t)index);
            }
            break;
        case SelectionKind::Resource:
            if(selected_resource_>=0 && (std::size_t)selected_resource_<placed_resources_.size()){
                const std::size_t index=(std::size_t)selected_resource_;
                utility_network_.remove_resource(index);
                walkway_network_.remove_resource(index);
                placed_resources_.erase(placed_resources_.begin()+(std::ptrdiff_t)index);
            }
            break;
        case SelectionKind::Corridor:
            if(selected_corridor_>=0 && (std::size_t)selected_corridor_<corridor_network_.connections().size())
                corridor_network_.remove_corridor((std::size_t)selected_corridor_);
            break;
        case SelectionKind::Utility:
            if(selected_utility_>=0 && (std::size_t)selected_utility_<utility_network_.connections().size())
                utility_network_.remove_connection((std::size_t)selected_utility_);
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
