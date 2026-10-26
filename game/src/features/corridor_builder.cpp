#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
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
    if(ep.kind==CorridorEndpointKind::Habitat){
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

Vec3 source_hint(const CorridorBuildAnchor& anchor,Vec3 hit,const std::vector<PlacedHabitat>& habitats){
    if(anchor.kind!=CorridorEndpointKind::Habitat || anchor.index>=habitats.size()) return hit+Vec3{4,0,0};
    const Vec3 c=habitats[anchor.index].position;
    Vec3 d=flat_dir(hit-c);
    if(length(flat(hit-c))<0.20f) d={1,0,0};
    return c+d*10.0f;
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
    r.draw_triangles_3d(preview->mesh.shell,{0.12f,0.72f,0.55f,0.30f});
    r.draw_triangles_3d(preview->mesh.details,{0.24f,0.88f,0.68f,0.46f});
    r.draw_lines_3d(preview->mesh.edges,{0.36f,1.00f,0.76f,0.96f},1.9f);
    render_endpoint_highlight(r,preview->endpoint_a,theme_.accent,2.8f);
    render_endpoint_highlight(r,preview->endpoint_b,{0.42f,1.00f,0.66f,1.0f},2.8f);
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

    // Before the first click, highlight the exact structural element that would
    // become the corridor source: habitat entrance/vestibule, corridor branch
    // sleeve, or a free sealed end.
    if(!corridor_build_start_){
        if(auto candidate=corridor_network_.preview_anchor(hover,source_hint(hover,hit,placed_habitats_),placed_habitats_,habitat_models_)){
            const Color c=candidate->kind==CorridorEndpointKind::OpenDoor?Color{0.96f,0.67f,0.22f,1.0f}:theme_.accent;
            render_endpoint_highlight(r,*candidate,c,3.0f);
        }
    }

    std::optional<CorridorConnection> preview;
    std::optional<CorridorEndpoint> source_endpoint;
    std::optional<CorridorEndpoint> destination_endpoint;
    if(corridor_build_start_){
        preview=corridor_network_.preview_built_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_);
        if(preview){
            source_endpoint=preview->endpoint_a;
            destination_endpoint=preview->endpoint_b;
            r.draw_triangles_3d(preview->mesh.shell,{0.12f,0.72f,0.55f,0.30f});
            r.draw_triangles_3d(preview->mesh.details,{0.24f,0.88f,0.68f,0.52f});
            r.draw_lines_3d(preview->mesh.edges,{0.36f,1.00f,0.76f,0.96f},1.9f);
        }else{
            // Even if the route is blocked, resolve both endpoint structures so
            // the player can see which existing/generated entrance is being
            // considered at the source and destination.
            source_endpoint=corridor_network_.preview_anchor(*corridor_build_start_,hover.position,placed_habitats_,habitat_models_);
            const Vec3 other=source_endpoint?source_endpoint->position:corridor_build_start_->position;
            destination_endpoint=corridor_network_.preview_anchor(hover,other,placed_habitats_,habitat_models_);
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
    if(!corridor_build_start_){
        corridor_build_start_=hover;
        clear_world_selection();
        corridor_link_source_=-1;
        return;
    }

    if(preview && corridor_network_.build_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_)){
        selected_corridor_=(int)corridor_network_.connections().size()-1;
        selected_habitat_=-1;
        selection_kind_=SelectionKind::Corridor;
        corridor_build_start_.reset();
        build_catalog_.clear_selection();
    }
}

} // namespace game
