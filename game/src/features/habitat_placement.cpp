#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/world_labels.hpp"
#include "game/world/geometry_standards.hpp"
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

bool GameApp::placement_valid(HabitatKind kind,HabitatSizeTier size,Vec3 position,float rotation_degrees) const {
    const HabitatMesh& candidate=habitat_models_.mesh(kind,size);
    const float half_field=(float)playfield_.size()*0.5f;
    const float a=radians(rotation_degrees);
    const float hx=std::abs(std::cos(a))*candidate.footprint_x*0.5f+std::abs(std::sin(a))*candidate.footprint_z*0.5f;
    const float hz=std::abs(std::sin(a))*candidate.footprint_x*0.5f+std::abs(std::cos(a))*candidate.footprint_z*0.5f;
    if(position.x-hx<-half_field || position.x+hx>half_field || position.z-hz<-half_field || position.z+hz>half_field) return false;

    const auto candidate_box=footprint(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z);
    constexpr float clearance=0.45f;
    for(const auto& placed:placed_habitats_){
        const HabitatMesh& other=habitat_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& placed:placed_resources_){
        const ResourceMesh& other=resource_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& pile:ground_resource_piles_){
        const float diameter=std::max(0.20f,pile.radius*2.0f);
        const auto pile_box=footprint(pile.position,0.0f,diameter,diameter);
        if(overlaps(candidate_box,pile_box,0.12f)) return false;
    }
    // Structures cannot be placed over an already-built/planned pressure
    // corridor.  Use the footprint circumradius so rotated long buildings are
    // conservative rather than leaving a corner cutting through the shell.
    const float corridor_clearance=0.5f*std::sqrt(candidate.footprint_x*candidate.footprint_x+
                                                   candidate.footprint_z*candidate.footprint_z)+
                                   geometry::kCorridorHalfWidth+geometry::kRouteClearance;
    for(const auto& corridor:corridor_network_.connections()){
        for(std::size_t i=0;i+1<corridor.path.size();++i)
            if(point_segment_distance_xz(position,corridor.path[i],corridor.path[i+1])<corridor_clearance) return false;
    }
    return true;
}

void GameApp::render_habitat_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Habitat) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    Vec3 snapped{std::round(hit.x),0.0f,std::round(hit.z)};
    const HabitatSizeTier size=placement_habitat_size_;
    const auto plan=construction_costs::habitat(selected->habitat_kind,size,habitat_models_.mesh(selected->habitat_kind,size));
    const bool valid=placement_valid(selected->habitat_kind,size,snapped,placement_rotation_degrees_) && can_start_construction(plan);

    render_habitat_mesh(r,habitat_models_.mesh(selected->habitat_kind,size),snapped,placement_rotation_degrees_,true,valid);

    if(valid && input_.left_pressed){
        PlacedHabitat placed;
        placed.kind=selected->habitat_kind;
        std::uint32_t serial=1;
        for(const auto& existing:placed_habitats_){
            if(existing.kind==placed.kind && existing.size==size) serial=std::max(serial,existing.instance_serial+1);
        }
        placed.instance_serial=serial;
        placed.position=snapped;
        placed.rotation_degrees=placement_rotation_degrees_;
        placed.size=size;
        placed.resource_plan=plan;
        placed_habitats_.push_back(std::move(placed));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_habitat_size_=HabitatSizeTier::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
