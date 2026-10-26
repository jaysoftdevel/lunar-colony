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

bool GameApp::placement_valid(HabitatKind kind,Vec3 position,float rotation_degrees) const {
    const HabitatMesh& candidate=habitat_models_.mesh(kind);
    const float half_field=(float)playfield_.size()*0.5f;
    const float a=radians(rotation_degrees);
    const float hx=std::abs(std::cos(a))*candidate.footprint_x*0.5f+std::abs(std::sin(a))*candidate.footprint_z*0.5f;
    const float hz=std::abs(std::sin(a))*candidate.footprint_x*0.5f+std::abs(std::cos(a))*candidate.footprint_z*0.5f;
    if(position.x-hx<-half_field || position.x+hx>half_field || position.z-hz<-half_field || position.z+hz>half_field) return false;

    const auto candidate_box=footprint(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z);
    constexpr float clearance=0.45f;
    for(const auto& placed:placed_habitats_){
        const HabitatMesh& other=habitat_models_.mesh(placed.kind);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
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
    const bool valid=placement_valid(selected->habitat_kind,snapped,placement_rotation_degrees_);

    // Preview the exact corridor plan that will be committed on placement.
    // The temporary habitat list includes the candidate with its current
    // rotation, so door choice, generated side entrances and all route bends
    // update live while the player moves/rotates the object.
    if(!placed_habitats_.empty()){
        auto preview_habitats=placed_habitats_;
        preview_habitats.push_back({selected->habitat_kind,snapped,placement_rotation_degrees_});
        if(auto preview=corridor_network_.preview_connection(preview_habitats.size()-1,preview_habitats,habitat_models_)){
            const Color shell_color=valid?Color{0.12f,0.72f,0.55f,0.28f}:Color{0.82f,0.20f,0.18f,0.24f};
            const Color detail_color=valid?Color{0.24f,0.88f,0.68f,0.38f}:Color{0.94f,0.30f,0.25f,0.34f};
            const Color edge_color=valid?Color{0.36f,1.00f,0.76f,0.92f}:Color{1.00f,0.40f,0.32f,0.90f};
            r.draw_triangles_3d(preview->mesh.shell,shell_color);
            r.draw_triangles_3d(preview->mesh.details,detail_color);
            r.draw_lines_3d(preview->mesh.edges,edge_color,1.7f);
        }
    }

    render_habitat_mesh(r,habitat_models_.mesh(selected->habitat_kind),snapped,placement_rotation_degrees_,true,valid);

    if(valid && input_.left_pressed){
        placed_habitats_.push_back({selected->habitat_kind,snapped,placement_rotation_degrees_});
        corridor_network_.connect_new_habitat(placed_habitats_.size()-1,placed_habitats_,habitat_models_);
        // A successful placement is a completed operation. Deselect the build
        // object so a second structure is never placed accidentally.
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        corridor_build_start_.reset();
    }
}

} // namespace game
