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
}

int GameApp::pick_habitat(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h.kind);
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
    return std::nullopt;
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
    // Habitats take priority when the pointer is actually over their footprint;
    // otherwise nearby corridor centerlines become branch/intersection targets.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)habitat,hit};
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
        return;
    }
    const int corridor=pick_corridor(hit);
    if(corridor>=0){
        selection_kind_=SelectionKind::Corridor;
        selected_corridor_=corridor;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        return;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        selection_kind_=SelectionKind::Habitat;
        selected_habitat_=habitat;
        selected_corridor_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        return;
    }
    const int resource=pick_resource(hit);
    if(resource>=0){
        selection_kind_=SelectionKind::Resource;
        selected_resource_=resource;
        selected_habitat_=-1;
        selected_corridor_=-1;
        selected_utility_=-1;
        return;
    }
    clear_world_selection();
}

} // namespace game
