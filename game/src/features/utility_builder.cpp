#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/util/world_math.hpp"
#include <optional>
#include <array>
#include <limits>

namespace game {
using namespace eng;
using namespace layout;

namespace {
ConstructionResourcePlan supply_plan(const UtilityConnection& c){
    return construction_costs::supply_channel(c.path,c.junctions.size());
}

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

void draw_supply_start_sample(Renderer& r,Vec3 origin,Vec3 tangent,bool valid){
    tangent.y=0.0f;
    if(length(tangent)<0.001f) tangent={1,0,0};
    tangent=normalize(tangent);
    const Vec3 side{-tangent.z,0,tangent.x};
    constexpr float run=1.85f;
    constexpr float half=0.30f;
    constexpr float y=0.28f;
    const Vec3 a=origin+Vec3{0,y,0};
    const Vec3 b=origin+tangent*run+Vec3{0,y,0};
    const Vec3 s=side*half;
    const Color fill=valid?Color{0.20f,0.64f,1.00f,0.18f}:Color{1.0f,0.22f,0.18f,0.16f};
    const Color edge=valid?Color{0.34f,0.82f,1.00f,0.96f}:Color{1.0f,0.30f,0.22f,0.92f};
    r.draw_triangles_3d({a-s,b-s,b+s,a-s,b+s,a+s},fill);
    r.draw_lines_3d({a-s,b-s,b-s,b+s,b+s,a+s,a+s,a-s},edge,2.0f);
    r.draw_lines_3d({a+side*0.15f+Vec3{0,0.08f,0},b+side*0.15f+Vec3{0,0.08f,0},
                     a-side*0.15f+Vec3{0,-0.08f,0},b-side*0.15f+Vec3{0,-0.08f,0}},edge,1.5f);
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
            render_resource_mesh(r,resource_models_.mesh(item),item.position,item.rotation_degrees,true,true,false);
        }
    };

    if(!utility_build_start_){
        std::optional<UtilityEndpoint> source_endpoint;
        std::optional<UtilityConnection> sample;
        if(hover){
            source_endpoint=utility_network_.preview_anchor(selected->utility_kind,*hover,hover->hit+Vec3{1,0,0},
                                                            placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            const Vec3 base=source_endpoint?source_endpoint->position:hover->hit;
            constexpr std::array<Vec3,4> dirs{{Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}}};
            float best=std::numeric_limits<float>::max();
            for(const Vec3 dir:dirs){
                UtilityBuildAnchor end{UtilityEndpointKind::Open,0,base+dir*2.35f};
                auto candidate=utility_network_.preview_connection(selected->utility_kind,*hover,end,
                                                                    placed_habitats_,habitat_models_,placed_resources_,resource_models_);
                if(!candidate) continue;
                float n=0.0f;
                for(std::size_t i=0;i+1<candidate->path.size();++i) n+=length(candidate->path[i+1]-candidate->path[i]);
                if(n<best){ best=n; sample=std::move(candidate); }
            }
        }
        const bool sample_valid=sample.has_value() && can_start_construction(supply_plan(*sample));
        if(sample){
            const Color shell{0.34f,0.82f,1.00f,0.74f};
            const Color detail{0.42f,0.88f,1.00f,0.68f};
            mesh_wireframe::draw_layer(r,sample->mesh.shell,shell,1.35f);
            mesh_wireframe::draw_layer(r,sample->mesh.details,detail,1.15f);
            r.draw_lines_3d(sample->mesh.edges,shell,1.8f);
        }else{
            draw_supply_start_sample(r,source_endpoint?source_endpoint->position:hit,
                                     source_endpoint?source_endpoint->normal:Vec3{1,0,0},false);
        }
        if(hover){
            highlight_anchor(*hover);
            // Structure endpoints show their real service port. Open/network
            // junction points stay as a centerline marker so no fake cap is
            // drawn over an existing channel intersection.
            if(source_endpoint && source_endpoint->kind!=UtilityEndpointKind::Open){
                const auto port=utility_network_.preview_endpoint_mesh(*source_endpoint);
                const Color port_color=sample_valid?Color{0.68f,0.86f,0.98f,0.92f}:Color{1.0f,0.28f,0.22f,0.90f};
                mesh_wireframe::draw_layer(r,port.shell,port_color,1.4f);
                mesh_wireframe::draw_layer(r,port.details,port_color,1.2f);
                r.draw_lines_3d(port.edges,port_color,1.8f);
                endpoint_marker(r,*source_endpoint,port_color);
            }else if(source_endpoint){
                marker(r,source_endpoint->position,sample_valid?theme_.accent:Color{1.0f,0.30f,0.22f,1.0f});
            }
        }
        if(input_.left_pressed && hover && sample_valid){
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

    const bool affordable=preview && can_start_construction(supply_plan(*preview));
    if(preview){
        const Color shell=affordable?Color{0.68f,0.86f,0.98f,0.95f}:Color{1.00f,0.24f,0.18f,0.86f};
        const Color detail=affordable?Color{0.93f,0.72f,0.28f,0.90f}:Color{1.00f,0.34f,0.24f,0.82f};
        mesh_wireframe::draw_layer(r,preview->mesh.shell,shell,1.45f);
        mesh_wireframe::draw_layer(r,preview->mesh.details,detail,1.20f);
        r.draw_lines_3d(preview->mesh.edges,shell,1.9f);
        endpoint_marker(r,preview->endpoint_a,theme_.accent);
        endpoint_marker(r,preview->endpoint_b,shell);
    }else if(hover){
        const Vec3 a=utility_build_start_->hit+Vec3{0,0.30f,0};
        const Vec3 b=hover->hit+Vec3{0,0.30f,0};
        r.draw_lines_3d({a,b},{1.0f,0.28f,0.22f,0.90f},2.2f);
        marker(r,hover->hit,{1.0f,0.30f,0.22f,1.0f},0.72f);
    }

    if(!input_.left_pressed) return;
    if(!hover) return;
    if(preview && affordable && utility_network_.build_connection(selected->utility_kind,*utility_build_start_,*hover,
                                                     placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
        utility_network_.mutable_connections().back().placement_sequence=next_blueprint_placement_sequence_++;
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
