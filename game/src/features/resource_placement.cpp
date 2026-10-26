#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include <cmath>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;

bool GameApp::resource_placement_valid(ResourceKind kind,Vec3 position,float rotation_degrees) const {
    const ResourceMesh& candidate=resource_models_.mesh(kind);
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
        const ResourceMesh& other=resource_models_.mesh(placed.kind);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    return true;
}

void GameApp::render_resource_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Resource) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const Vec3 snapped{std::round(hit.x),0.0f,std::round(hit.z)};
    const bool valid=resource_placement_valid(selected->resource_kind,snapped,placement_rotation_degrees_);
    render_resource_mesh(r,resource_models_.mesh(selected->resource_kind),snapped,placement_rotation_degrees_,true,valid);

    if(valid && input_.left_pressed){
        placed_resources_.push_back({selected->resource_kind,snapped,placement_rotation_degrees_});
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        corridor_build_start_.reset();
    }
}

} // namespace game
