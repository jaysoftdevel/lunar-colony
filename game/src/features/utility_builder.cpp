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

    // Supply channels snap to whole structures while preserving the cursor hit
    // for precise port placement. Highlight both snapped structures in blueprint
    // mode before commit.
    const auto highlight_anchor=[&](const UtilityBuildAnchor& anchor){
        if(anchor.kind==UtilityEndpointKind::Habitat && anchor.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[anchor.index];
            render_habitat_mesh(r,habitat_models_.mesh(h),h.position,h.rotation_degrees,true,true,false);
        }else if(anchor.kind==UtilityEndpointKind::Resource && anchor.index<placed_resources_.size()){
            const auto& item=placed_resources_[anchor.index];
            render_resource_mesh(r,resource_models_.mesh(item.kind),item.position,item.rotation_degrees,true,true,false);
        }
    };

    if(!utility_build_start_){
        if(hover){
            highlight_anchor(*hover);
            // Resolve and draw the real combined service port before the first
            // click. As the pointer moves over the snapped structure this
            // blueprint floats over its actual mesh and shows exactly where the
            // source will attach.
            if(auto endpoint=utility_network_.preview_anchor(selected->utility_kind,*hover,hover->hit+Vec3{1,0,0},
                                                             placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
                const auto port=utility_network_.preview_endpoint_mesh(*endpoint);
                mesh_wireframe::draw_layer(r,port.shell,{0.68f,0.86f,0.98f,0.92f},1.4f);
                mesh_wireframe::draw_layer(r,port.details,{0.93f,0.72f,0.28f,0.92f},1.2f);
                r.draw_lines_3d(port.edges,{0.68f,0.86f,0.98f,0.96f},1.8f);
                endpoint_marker(r,*endpoint,theme_.accent);
            }else marker(r,hover->hit,theme_.accent);
        }
        if(input_.left_pressed && hover){
            utility_build_start_=*hover;
            clear_world_selection();
            corridor_link_source_=-1;
        }
        return;
    }

    highlight_anchor(*utility_build_start_);
    marker(r,utility_build_start_->hit,theme_.accent,0.72f);
    if(hover && !(hover->kind==utility_build_start_->kind && hover->index==utility_build_start_->index)) highlight_anchor(*hover);
    std::optional<UtilityConnection> preview;
    if(hover){
        preview=utility_network_.preview_connection(selected->utility_kind,*utility_build_start_,*hover,
                                                     placed_habitats_,habitat_models_,placed_resources_,resource_models_);
    }

    if(preview){
        const Color shell{0.68f,0.86f,0.98f,0.95f};
        const Color detail{0.93f,0.72f,0.28f,0.90f};
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
