#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/mesh_wireframe.hpp"
#include <optional>
#include <array>
#include <limits>

namespace game {
using namespace eng;
using namespace layout;

namespace {
void marker(Renderer& r,Vec3 p,Color c,float size=0.68f){
    p.y=0.12f;
    r.draw_lines_3d({p+Vec3{-size,0,0},p+Vec3{size,0,0},p+Vec3{0,0,-size},p+Vec3{0,0,size}},c,2.8f);
}

bool same_anchor(const WalkwayBuildAnchor& a,const WalkwayBuildAnchor& b){
    if(a.kind==WalkwayEndpointKind::Open || b.kind==WalkwayEndpointKind::Open){
        const float dx=a.hit.x-b.hit.x,dz=a.hit.z-b.hit.z;
        return a.kind==b.kind && dx*dx+dz*dz<0.20f*0.20f;
    }
    return a.kind==b.kind && a.index==b.index;
}

void draw_start_section(Renderer& r,Vec3 p,Vec3 tangent,bool valid){
    // A real one-section strip follows the pointer before the first click. This
    // is directional (unlike the old square tile), so intersections and branch
    // orientation are readable before the source is committed.
    tangent.y=0.0f;
    if(length(tangent)<0.001f) tangent={1,0,0};
    tangent=normalize(tangent);
    const Vec3 side{-tangent.z,0,tangent.x};
    constexpr float run=2.55f;
    constexpr float half=0.78f;
    constexpr float y=0.205f;
    const Vec3 a=p+Vec3{0,y,0};
    const Vec3 b=p+tangent*run+Vec3{0,y,0};
    const Vec3 s=side*half;
    const Color fill=valid?Color{0.18f,0.60f,1.00f,0.22f}:Color{1.00f,0.18f,0.14f,0.18f};
    const Color edge=valid?Color{0.34f,0.82f,1.00f,0.98f}:Color{1.00f,0.28f,0.22f,0.98f};
    r.draw_triangles_3d({a-s,b-s,b+s,a-s,b+s,a+s},fill);
    r.draw_lines_3d({a-s,b-s,b-s,b+s,b+s,a+s,a+s,a-s},edge,2.2f);
}

float preview_length(const WalkwayConnection& c){
    float total=0.0f;
    for(std::size_t i=0;i+1<c.path.size();++i){
        const Vec3 d=c.path[i+1]-c.path[i];
        total+=std::sqrt(d.x*d.x+d.z*d.z);
    }
    return total;
}

void draw_blueprint(Renderer& r,const WalkwayConnection& preview,const eng::ui::Theme& theme,bool valid){
    const Color deck=valid?Color{0.22f,0.66f,1.00f,0.88f}:Color{1.00f,0.18f,0.14f,0.82f};
    const Color supply=valid?Color{0.30f,0.84f,1.00f,0.82f}:Color{1.00f,0.32f,0.22f,0.86f};
    const Color edge=valid?theme.accent:Color{1.00f,0.12f,0.10f,1.00f};
    mesh_wireframe::draw_layer(r,preview.mesh.deck,deck,1.45f);
    mesh_wireframe::draw_layer(r,preview.mesh.details,supply,1.25f);
    r.draw_lines_3d(preview.mesh.edges,edge,2.10f);
}
}

void GameApp::render_walkway_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Walkway) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const auto hover=walkway_anchor_at(hit,!walkway_build_start_.has_value());
    if(!hover) return;

    const auto highlight=[&](const WalkwayBuildAnchor& a){
        if(a.kind==WalkwayEndpointKind::Habitat && a.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[a.index];
            render_habitat_mesh(r,habitat_models_.mesh(h),h.position,h.rotation_degrees,true,true,false);
        }else if(a.kind==WalkwayEndpointKind::Resource && a.index<placed_resources_.size()){
            const auto& item=placed_resources_[a.index];
            render_resource_mesh(r,resource_models_.mesh(item),item.position,item.rotation_degrees,true,true,false);
        }
    };

    // Before the first click, render one real walkway section using the same
    // solver as final placement. Existing walkways/corridors therefore show a
    // blue branch only when an intersection is actually possible; otherwise
    // the source section remains red and cannot be committed.
    if(!walkway_build_start_){
        highlight(*hover);
        std::optional<WalkwayConnection> sample;
        constexpr std::array<Vec3,8> dirs{{
            Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1},
            Vec3{0.7071f,0,0.7071f},Vec3{-0.7071f,0,0.7071f},
            Vec3{0.7071f,0,-0.7071f},Vec3{-0.7071f,0,-0.7071f}
        }};
        float best=std::numeric_limits<float>::max();
        for(const Vec3 dir:dirs){
            WalkwayBuildAnchor end{WalkwayEndpointKind::Open,0,hover->hit+dir*3.10f};
            auto candidate=walkway_network_.preview_connection(*hover,end,corridor_network_,utility_network_,
                                                               placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            if(!candidate) continue;
            const float n=preview_length(*candidate);
            if(n<best){ best=n; sample=std::move(candidate); }
        }
        const bool sample_valid=sample.has_value() && can_start_construction(construction_costs::walkway(preview_length(*sample)));
        if(sample){
            draw_blueprint(r,*sample,theme_,true);
            marker(r,sample->endpoint_a.position,theme_.accent,0.55f);
        }else{
            draw_start_section(r,hover->hit,Vec3{1,0,0},false);
            marker(r,hover->hit,{1.0f,0.28f,0.22f,1.0f},0.55f);
        }

        if(input_.left_pressed && sample_valid){
            clear_world_selection();
            corridor_link_source_=-1;
            walkway_build_start_=*hover;
        }
        return;
    }

    highlight(*walkway_build_start_);
    highlight(*hover);
    marker(r,walkway_build_start_->hit,theme_.accent,0.70f);

    std::optional<WalkwayConnection> preview;
    if(!same_anchor(*hover,*walkway_build_start_)){
        preview=walkway_network_.preview_connection(*walkway_build_start_,*hover,corridor_network_,utility_network_,
                                                    placed_habitats_,habitat_models_,placed_resources_,resource_models_);
    }

    const bool affordable=preview && can_start_construction(construction_costs::walkway(preview_length(*preview)));
    if(preview){
        // Valid geometry is still shown red when the colony cannot reserve the
        // complete staged material plan for this walkway.
        draw_blueprint(r,*preview,theme_,affordable);
        marker(r,preview->endpoint_a.position,theme_.accent,0.70f);
        marker(r,preview->endpoint_b.position,{0.34f,0.82f,1.0f,1.0f},0.70f);
    }else{
        // Invalid structure/corridor entrance, blocked route, or conflicting snap:
        // keep drawing the attempted full walkway, but in red. A click cannot
        // commit this fallback preview.
        const auto invalid=walkway_network_.preview_invalid_connection(*walkway_build_start_,*hover,corridor_network_,utility_network_,
                                                                        placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        if(invalid){
            draw_blueprint(r,*invalid,theme_,false);
            marker(r,invalid->endpoint_a.position,{1.0f,0.24f,0.18f,1.0f},0.70f);
            marker(r,invalid->endpoint_b.position,{1.0f,0.24f,0.18f,1.0f},0.70f);
        }else{
            marker(r,hover->hit,{1.0f,0.30f,0.22f,1.0f},0.70f);
        }
    }

    if(!input_.left_pressed || !preview || !affordable) return;
    if(walkway_network_.build_connection(*walkway_build_start_,*hover,corridor_network_,utility_network_,
                                         placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
        selected_walkway_=(int)walkway_network_.connections().size()-1;
        selected_habitat_=selected_corridor_=selected_resource_=selected_utility_=-1;
        selection_kind_=SelectionKind::Walkway;
        walkway_build_start_.reset();
        build_catalog_.clear_selection();
    }
}

} // namespace game
