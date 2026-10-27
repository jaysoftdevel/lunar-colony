#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;

namespace {
Vec3 flat(Vec3 v){ v.y=0.0f; return v; }
Vec3 flat_dir(Vec3 v){
    v=flat(v);
    const float l=length(v);
    return l>0.0001f?v*(1.0f/l):Vec3{1,0,0};
}
Vec3 side_of(Vec3 normal){ normal=flat_dir(normal); return {-normal.z,0,normal.x}; }

void line(std::vector<Vec3>& out,Vec3 a,Vec3 b){ out.push_back(a); out.push_back(b); }

float route_length(const CorridorConnection& c){
    float total=0.0f;
    for(std::size_t i=0;i+1<c.path.size();++i) total+=distance_xz(c.path[i],c.path[i+1]);
    return total;
}

std::vector<Vec3> doorway_outline(Vec3 center,Vec3 normal,float half_width,float height){
    const Vec3 side=side_of(normal)*half_width;
    const Vec3 base{center.x,0.06f,center.z};
    const Vec3 top{0,height,0};
    const Vec3 p0=base-side,p1=base+side,p2=p1+top,p3=p0+top;
    std::vector<Vec3> out;
    line(out,p0,p1); line(out,p1,p2); line(out,p2,p3); line(out,p3,p0);
    return out;
}

void append_d_profile(std::vector<Vec3>& out,Vec3 center,Vec3 tangent,float scale=1.0f){
    const Vec3 side=side_of(tangent);
    constexpr float half_w=1.16f;
    constexpr float floor_y=0.10f;
    constexpr float side_y=0.66f;
    constexpr float roof=1.12f;
    std::vector<Vec3> pts;
    pts.push_back(center-side*(half_w*scale)+Vec3{0,floor_y,0});
    pts.push_back(center+side*(half_w*scale)+Vec3{0,floor_y,0});
    constexpr int arc=10;
    for(int i=0;i<=arc;++i){
        const float a=(float)i/(float)arc*3.14159265358979323846f;
        pts.push_back(center+side*(std::cos(a)*roof*scale)+Vec3{0,side_y+std::sin(a)*roof*scale,0});
    }
    for(std::size_t i=0;i<pts.size();++i) line(out,pts[i],pts[(i+1)%pts.size()]);
}

void render_endpoint_highlight(Renderer& r,const CorridorEndpoint& ep,Color color,float width){
    std::vector<Vec3> lines;
    if(ep.kind==CorridorEndpointKind::Habitat || ep.kind==CorridorEndpointKind::Resource){
        const Vec3 n=flat_dir(ep.door.normal);
        const auto wall=doorway_outline(ep.door.wall_position,n,1.34f,2.92f);
        const auto outer=doorway_outline(ep.door.position,n,1.34f,2.92f);
        lines.insert(lines.end(),wall.begin(),wall.end());
        lines.insert(lines.end(),outer.begin(),outer.end());

        const Vec3 s=side_of(n)*1.34f;
        const Vec3 wb{ep.door.wall_position.x,0.06f,ep.door.wall_position.z};
        const Vec3 ob{ep.door.position.x,0.06f,ep.door.position.z};
        line(lines,wb-s,ob-s); line(lines,wb+s,ob+s);
        line(lines,wb-s+Vec3{0,2.92f,0},ob-s+Vec3{0,2.92f,0});
        line(lines,wb+s+Vec3{0,2.92f,0},ob+s+Vec3{0,2.92f,0});

        // Highlight the corridor-side pressure collar as well. For a generated
        // entrance the two bright frames make the complete new vestibule clear
        // before the corridor is committed.
        append_d_profile(lines,ep.position,n,1.10f);
        line(lines,ep.door.position+Vec3{0,1.46f,0},ep.position+Vec3{0,1.46f,0});
    }else if(ep.kind==CorridorEndpointKind::Corridor){
        append_d_profile(lines,ep.position,ep.tangent,1.18f);
        const Vec3 s=side_of(ep.tangent)*0.56f;
        line(lines,ep.position-s+Vec3{0,0.05f,0},ep.position+s+Vec3{0,0.05f,0});
        line(lines,ep.position+Vec3{0,0.05f,-0.56f},ep.position+Vec3{0,0.05f,0.56f});
    }else{
        lines=doorway_outline(ep.position,ep.tangent,0.72f,1.82f);
        const Vec3 p=ep.position+Vec3{0,0.05f,0};
        line(lines,p+Vec3{-0.40f,0,0},p+Vec3{0.40f,0,0});
        line(lines,p+Vec3{0,0,-0.40f},p+Vec3{0,0,0.40f});
    }
    r.draw_lines_3d(lines,color,width);
}

Vec3 source_hint(const CorridorBuildAnchor& anchor,Vec3 hit,const std::vector<PlacedHabitat>& habitats,const std::vector<PlacedResource>& resources){
    Vec3 c{};
    bool structure=false;
    if(anchor.kind==CorridorEndpointKind::Habitat && anchor.index<habitats.size()){ c=habitats[anchor.index].position; structure=true; }
    if(anchor.kind==CorridorEndpointKind::Resource && anchor.index<resources.size()){ c=resources[anchor.index].position; structure=true; }
    if(!structure) return hit+Vec3{4,0,0};
    Vec3 d=flat_dir(hit-c);
    if(length(flat(hit-c))<0.20f) d={1,0,0};
    return c+d*10.0f;
}

void draw_corridor_start_sample(Renderer& r,Vec3 origin,Vec3 tangent,bool valid){
    tangent=flat_dir(tangent);
    const Vec3 side=side_of(tangent);
    constexpr float length_m=2.35f;
    constexpr float half_w=1.16f;
    constexpr float y=0.12f;
    const Vec3 a=origin+Vec3{0,y,0};
    const Vec3 b=origin+tangent*length_m+Vec3{0,y,0};
    const Vec3 s=side*half_w;
    const Color fill=valid?Color{0.20f,0.64f,1.00f,0.18f}:Color{1.00f,0.20f,0.16f,0.16f};
    const Color edge_color=valid?Color{0.34f,0.82f,1.00f,0.96f}:Color{1.00f,0.28f,0.22f,0.96f};
    r.draw_triangles_3d({a-s,b-s,b+s,a-s,b+s,a+s},fill);
    std::vector<Vec3> lines;
    append_d_profile(lines,origin,tangent,1.0f);
    append_d_profile(lines,origin+tangent*length_m,tangent,1.0f);
    line(lines,a-s,b-s); line(lines,a+s,b+s);
    line(lines,a+Vec3{0,1.72f,0},b+Vec3{0,1.72f,0});
    r.draw_lines_3d(lines,edge_color,2.0f);
}
} // namespace

void GameApp::render_corridor_link_preview(Renderer& r){
    if(corridor_link_source_<0 || build_catalog_.selected()!=nullptr) return;
    const float logical_y=(float)input_.mouse_y/ui_scale_;
    if(logical_y<STATUS_BAR_H || logical_y>=ui_height(r)-BUILD_BAR_H) return;
    if(mouse_over_selection_panel(input_)) return;
    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int target=pick_corridor(hit,1.70f);
    if(target<0 || target==corridor_link_source_) return;
    auto preview=corridor_network_.preview_corridor_link((size_t)corridor_link_source_,(size_t)target,placed_habitats_,habitat_models_);
    if(!preview) return;
    const bool affordable=can_start_construction(construction_costs::corridor(route_length(*preview)));
    const Color shell=affordable?Color{0.36f,1.00f,0.76f,0.92f}:Color{1.00f,0.24f,0.18f,0.82f};
    const Color detail=affordable?Color{0.24f,0.88f,0.68f,0.84f}:Color{1.00f,0.34f,0.24f,0.78f};
    mesh_wireframe::draw_layer(r,preview->mesh.shell,shell,1.35f);
    mesh_wireframe::draw_layer(r,preview->mesh.details,detail,1.15f);
    r.draw_lines_3d(preview->mesh.edges,shell,1.9f);
    render_endpoint_highlight(r,preview->endpoint_a,affordable?theme_.accent:Color{1.00f,0.28f,0.22f,1.0f},2.8f);
    render_endpoint_highlight(r,preview->endpoint_b,affordable?Color{0.42f,1.00f,0.66f,1.0f}:Color{1.00f,0.28f,0.22f,1.0f},2.8f);

    // Commit only after the full candidate connection has been submitted as a
    // blueprint in this frame. This keeps manual links consistent with corridor
    // build mode, utility channels and automatic habitat corridor previews.
    if(input_.left_pressed && affordable &&
       corridor_network_.connect_corridors((std::size_t)corridor_link_source_,(std::size_t)target,placed_habitats_,habitat_models_)){
        selected_corridor_=(int)corridor_network_.connections().size()-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        selection_kind_=SelectionKind::Corridor;
        corridor_link_source_=-1;
    }
}

void GameApp::render_corridor_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Corridor) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const CorridorBuildAnchor hover=corridor_anchor_at(hit);

    // Like supply channels, a corridor snaps to the complete habitat while the
    // pointer chooses the exact door position on its pressure hull. Highlight
    // the owning structure so source and destination are unambiguous.
    const auto highlight_habitat_anchor=[&](const CorridorBuildAnchor& anchor){
        if(anchor.kind!=CorridorEndpointKind::Habitat || anchor.index>=placed_habitats_.size()) return;
        const auto& h=placed_habitats_[anchor.index];
        render_habitat_mesh(r,habitat_models_.mesh(h),h.position,h.rotation_degrees,true,true,false);
    };
    const auto highlight_resource_anchor=[&](const CorridorBuildAnchor& anchor){
        if(anchor.kind!=CorridorEndpointKind::Resource || anchor.index>=placed_resources_.size()) return;
        const auto& item=placed_resources_[anchor.index];
        if(item.kind!=ResourceKind::StorageDome) return;
        render_resource_mesh(r,resource_models_.mesh(item),item.position,item.rotation_degrees,true,true,false);
    };

    // Before the first click, highlight the exact structural element that would
    // become the corridor source: habitat entrance/vestibule, corridor branch
    // sleeve, or a free sealed end.
    if(!corridor_build_start_){
        highlight_habitat_anchor(hover);
        highlight_resource_anchor(hover);
        auto candidate=corridor_network_.preview_anchor(hover,source_hint(hover,hit,placed_habitats_,placed_resources_),placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        std::optional<CorridorConnection> sample;
        if(candidate){
            const Vec3 dir=flat_dir(candidate->tangent);
            const CorridorBuildAnchor sample_end{CorridorEndpointKind::OpenDoor,0,candidate->position+dir*2.75f};
            sample=corridor_network_.preview_built_corridor(hover,sample_end,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        }
        const bool sample_valid=sample.has_value() && can_start_construction(construction_costs::corridor(route_length(*sample)));
        if(sample){
            mesh_wireframe::draw_layer(r,sample->mesh.shell,{0.30f,0.72f,1.00f,0.34f},1.35f);
            mesh_wireframe::draw_layer(r,sample->mesh.details,{0.24f,0.88f,0.68f,0.30f},1.10f);
            r.draw_lines_3d(sample->mesh.edges,{0.34f,0.82f,1.00f,0.96f},1.9f);
        }else{
            draw_corridor_start_sample(r,candidate?candidate->position:hit,candidate?candidate->tangent:Vec3{1,0,0},false);
        }
        if(candidate){
            const Color c=sample_valid?(candidate->kind==CorridorEndpointKind::OpenDoor?Color{0.96f,0.67f,0.22f,1.0f}:theme_.accent)
                                      :Color{1.00f,0.28f,0.22f,1.0f};
            // Always render the actual endpoint hardware before the first click.
            // Generated entrances therefore float over the hull while built-in
            // doors show their exact pressure adapter. Reused entrances resolve
            // to their shared corridor manifold and show that junction instead.
            if(candidate->kind==CorridorEndpointKind::Habitat || candidate->kind==CorridorEndpointKind::Resource){
                const auto entrance_mesh=corridor_network_.preview_endpoint_mesh(*candidate);
                const Color shell=sample_valid?Color{0.34f,1.00f,0.72f,0.88f}:Color{1.00f,0.24f,0.18f,0.80f};
                const Color details=sample_valid?Color{0.24f,0.88f,0.68f,0.82f}:Color{1.00f,0.34f,0.24f,0.78f};
                mesh_wireframe::draw_layer(r,entrance_mesh.shell,shell,1.35f);
                mesh_wireframe::draw_layer(r,entrance_mesh.details,details,1.15f);
                r.draw_lines_3d(entrance_mesh.edges,c,1.8f);
            }
            render_endpoint_highlight(r,*candidate,c,3.0f);
        }
        if(input_.left_pressed && sample_valid){
            corridor_build_start_=hover;
            clear_world_selection();
            corridor_link_source_=-1;
        }
        return;
    }

    std::optional<CorridorConnection> preview;
    std::optional<CorridorEndpoint> source_endpoint;
    std::optional<CorridorEndpoint> destination_endpoint;
    if(corridor_build_start_){
        highlight_habitat_anchor(*corridor_build_start_);
        highlight_resource_anchor(*corridor_build_start_);
        if(!(hover.kind==corridor_build_start_->kind && hover.index==corridor_build_start_->index)){
            highlight_habitat_anchor(hover);
            highlight_resource_anchor(hover);
        }
        preview=corridor_network_.preview_built_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        if(preview){
            source_endpoint=preview->endpoint_a;
            destination_endpoint=preview->endpoint_b;
            const bool affordable=can_start_construction(construction_costs::corridor(route_length(*preview)));
            const Color shell=affordable?Color{0.36f,1.00f,0.76f,0.92f}:Color{1.00f,0.24f,0.18f,0.82f};
            const Color detail=affordable?Color{0.24f,0.88f,0.68f,0.88f}:Color{1.00f,0.34f,0.24f,0.78f};
            mesh_wireframe::draw_layer(r,preview->mesh.shell,shell,1.35f);
            mesh_wireframe::draw_layer(r,preview->mesh.details,detail,1.15f);
            r.draw_lines_3d(preview->mesh.edges,shell,1.9f);
        }else{
            // Even if the route is blocked, resolve both endpoint structures so
            // the player can see which existing/generated entrance is being
            // considered at the source and destination.
            source_endpoint=corridor_network_.preview_anchor(*corridor_build_start_,hover.position,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            const Vec3 other=source_endpoint?source_endpoint->position:corridor_build_start_->position;
            destination_endpoint=corridor_network_.preview_anchor(hover,other,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            const Vec3 a=source_endpoint?source_endpoint->position:corridor_build_start_->position;
            r.draw_lines_3d({a+Vec3{0,0.08f,0},hit+Vec3{0,0.08f,0}},{1.0f,0.28f,0.22f,0.9f},2.0f);
        }

        if(source_endpoint) render_endpoint_highlight(r,*source_endpoint,theme_.accent,3.2f);
        if(destination_endpoint){
            const Color dc=preview?Color{0.42f,1.00f,0.66f,1.0f}:Color{1.00f,0.36f,0.28f,1.0f};
            render_endpoint_highlight(r,*destination_endpoint,dc,3.2f);
        }
    }

    if(!input_.left_pressed) return;

    const bool affordable=preview && can_start_construction(construction_costs::corridor(route_length(*preview)));
    if(preview && affordable && corridor_network_.build_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
        selected_corridor_=(int)corridor_network_.connections().size()-1;
        selected_habitat_=-1;
        selection_kind_=SelectionKind::Corridor;
        corridor_build_start_.reset();
        build_catalog_.clear_selection();
    }
}

} // namespace game
