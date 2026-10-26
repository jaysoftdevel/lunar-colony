#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/util/world_math.hpp"
#include <optional>

namespace game {
using namespace eng;
using namespace layout;

namespace {
void marker(Renderer& r,Vec3 p,Color c,float size=0.62f){
    p.y=0.10f;
    r.draw_lines_3d({
        p+Vec3{-size,0,0},p+Vec3{size,0,0},
        p+Vec3{0,0,-size},p+Vec3{0,0,size}
    },c,2.8f);
}

void endpoint_marker(Renderer& r,const UtilityEndpoint& ep,Color c){
    const Vec3 p=ep.position;
    const Vec3 n=ep.normal;
    const Vec3 side{-n.z,0,n.x};
    r.draw_lines_3d({
        p-side*0.46f,p+side*0.46f,
        p+Vec3{0,-0.18f,0},p+Vec3{0,0.42f,0},
        p,p+n*0.58f
    },c,3.0f);
}
}

void GameApp::render_utility_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Utility) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const auto hover=utility_anchor_at(hit,selected->utility_kind);

    if(!utility_build_start_){
        if(hover) marker(r,hover->hit,theme_.accent);
        if(input_.left_pressed && hover){
            utility_build_start_=*hover;
            clear_world_selection();
            corridor_link_source_=-1;
        }
        return;
    }

    marker(r,utility_build_start_->hit,theme_.accent,0.72f);
    std::optional<UtilityConnection> preview;
    if(hover){
        preview=utility_network_.preview_connection(selected->utility_kind,*utility_build_start_,*hover,
                                                     placed_habitats_,habitat_models_,placed_resources_,resource_models_);
    }

    if(preview){
        const Color shell=selected->utility_kind==UtilityKind::PowerCable?Color{1.00f,0.78f,0.26f,0.95f}:Color{0.30f,0.74f,1.00f,0.95f};
        const Color detail=selected->utility_kind==UtilityKind::PowerCable?Color{0.94f,0.62f,0.18f,0.90f}:Color{0.22f,0.62f,0.90f,0.90f};
        mesh_wireframe::draw_layer(r,preview->mesh.shell,shell,1.45f);
        mesh_wireframe::draw_layer(r,preview->mesh.details,detail,1.20f);
        r.draw_lines_3d(preview->mesh.edges,shell,1.9f);
        endpoint_marker(r,preview->endpoint_a,theme_.accent);
        endpoint_marker(r,preview->endpoint_b,shell);
    }else if(hover){
        marker(r,hover->hit,{1.0f,0.30f,0.22f,1.0f},0.72f);
    }

    if(!input_.left_pressed) return;
    if(!hover) return;
    if(preview && utility_network_.build_connection(selected->utility_kind,*utility_build_start_,*hover,
                                                     placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
        selected_utility_=(int)utility_network_.connections().size()-1;
        selected_habitat_=-1;
        selected_corridor_=-1;
        selected_resource_=-1;
        selection_kind_=SelectionKind::Utility;
        utility_build_start_.reset();
        build_catalog_.clear_selection();
    }
}

} // namespace game
