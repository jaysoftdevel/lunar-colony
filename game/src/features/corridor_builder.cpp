#include "game/world/airlock_connections.hpp"
#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
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

bool same_structure_anchor(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b){
    if(a.kind!=b.kind || a.index!=b.index) return false;
    return a.kind==CorridorEndpointKind::Habitat || a.kind==CorridorEndpointKind::Resource;
}

void line(std::vector<Vec3>& out,Vec3 a,Vec3 b){ out.push_back(a); out.push_back(b); }

std::size_t corridor_junction_count(const CorridorConnection& c){
    std::size_t count=0;
    if(c.endpoint_a.kind==CorridorEndpointKind::Corridor) ++count;
    if(c.endpoint_b.kind==CorridorEndpointKind::Corridor) ++count;
    return count;
}

ConstructionResourcePlan corridor_plan(const CorridorConnection& c){
    return construction_costs::corridor(c.path,corridor_junction_count(c));
}

std::vector<Vec3> doorway_outline(Vec3 center,Vec3 normal,float half_width,float height){
    const Vec3 side=side_of(normal)*half_width;
    const Vec3 base=center+Vec3{0,0.015f,0};
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


Vec3 nearest_blueprint_center(Vec3 point,const std::vector<Vec3>& path){
    if(path.empty()) return point;
    if(path.size()==1) return path.front();
    Vec3 best=path.front();
    float best_d=1.0e30f;
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 a=path[i],b=path[i+1];
        const float dx=b.x-a.x,dz=b.z-a.z;
        const float denom=dx*dx+dz*dz;
        float t=0.0f;
        if(denom>1.0e-7f) t=std::clamp(((point.x-a.x)*dx+(point.z-a.z)*dz)/denom,0.0f,1.0f);
        const Vec3 q=a+(b-a)*t;
        const float qx=point.x-q.x,qz=point.z-q.z;
        const float d=qx*qx+qz*qz;
        if(d<best_d){ best_d=d; best=q; }
    }
    return best;
}

std::vector<Vec3> scaled_blueprint_vertices(const std::vector<Vec3>& vertices,const std::vector<Vec3>& path,float scale){
    std::vector<Vec3> out;
    out.reserve(vertices.size());
    for(const Vec3& v:vertices){
        const Vec3 center=nearest_blueprint_center(v,path);
        out.push_back(center+(v-center)*scale);
    }
    return out;
}

void draw_compact_corridor_blueprint(Renderer& r,const CorridorConnection& preview,bool affordable){
    // The blueprint is deliberately smaller than the final pressure tube. It
    // keeps dense connection nodes readable and leaves the exact endpoint
    // highlights visible while still following the final routed centerline.
    constexpr float BLUEPRINT_SCALE=0.62f;
    const Color shell=affordable?Color{0.36f,1.00f,0.76f,0.84f}:Color{1.00f,0.24f,0.18f,0.78f};
    const Color detail=affordable?Color{0.24f,0.88f,0.68f,0.76f}:Color{1.00f,0.34f,0.24f,0.72f};
    const auto shell_vertices=scaled_blueprint_vertices(preview.mesh.shell,preview.path,BLUEPRINT_SCALE);
    const auto detail_vertices=scaled_blueprint_vertices(preview.mesh.details,preview.path,BLUEPRINT_SCALE);
    const auto edge_vertices=scaled_blueprint_vertices(preview.mesh.edges,preview.path,BLUEPRINT_SCALE);
    mesh_wireframe::draw_layer(r,shell_vertices,shell,1.20f);
    mesh_wireframe::draw_layer(r,detail_vertices,detail,1.00f);
    r.draw_lines_3d(edge_vertices,shell,1.65f);
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
        const Vec3 wb=ep.door.wall_position+Vec3{0,0.015f,0};
        const Vec3 ob=ep.door.position+Vec3{0,0.015f,0};
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

constexpr float initial_corridor_preview_run(){
    // Before the source is selected the player only needs an orientation hint,
    // not a fake section of already-sized corridor. Half a door-frame half
    // width is enough to show the pressure direction without implying length.
    return geometry::kDoorFrameHalfWidth*0.5f;
}

CorridorEndpoint compact_structure_start_endpoint(CorridorEndpoint endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Habitat && endpoint.kind!=CorridorEndpointKind::Resource) return endpoint;
    const Vec3 n=flat_dir(endpoint.door.normal);
    const float run=initial_corridor_preview_run();
    endpoint.door.position=endpoint.door.wall_position+n*(run*0.5f);
    endpoint.position=endpoint.door.wall_position+n*run;
    endpoint.position.y=0.0f;
    endpoint.tangent=n;
    return endpoint;
}

void draw_corridor_start_sample(Renderer& r,Vec3 origin,Vec3 tangent,bool valid){
    tangent=flat_dir(tangent);
    const Vec3 side=side_of(tangent);
    const float length_m=initial_corridor_preview_run();
    constexpr float half_w=0.76f;
    constexpr float y=0.12f;
    const Vec3 a=origin+Vec3{0,y,0};
    const Vec3 b=origin+tangent*length_m+Vec3{0,y,0};
    const Vec3 s=side*half_w;
    const Color fill=valid?Color{0.20f,0.64f,1.00f,0.18f}:Color{1.00f,0.20f,0.16f,0.16f};
    const Color edge_color=valid?Color{0.34f,0.82f,1.00f,0.96f}:Color{1.00f,0.28f,0.22f,0.96f};
    r.draw_triangles_3d({a-s,b-s,b+s,a-s,b+s,a+s},fill);
    std::vector<Vec3> lines;
    append_d_profile(lines,origin,tangent,0.66f);
    append_d_profile(lines,origin+tangent*length_m,tangent,0.66f);
    line(lines,a-s,b-s); line(lines,a+s,b+s);
    line(lines,a+Vec3{0,1.12f,0},b+Vec3{0,1.12f,0});
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
    const int target=pick_corridor(hit,geometry::kCorridorHalfWidth+0.28f);
    if(target<0 || target==corridor_link_source_) return;
    auto preview=corridor_network_.preview_corridor_link((size_t)corridor_link_source_,(size_t)target,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
    if(!preview){
        corridor_placement_failure_title_="CORRIDOR LINK CANNOT BE PLACED";
        corridor_placement_failure_detail_="No collision-free connector can be routed between the selected corridor sections.";
        return;
    }
    const bool gate_clear=corridor_gate_layout_clear(*preview);
    const bool affordable=(can_start_construction(corridor_plan(*preview)) && gate_clear);
    if(!gate_clear){
        corridor_placement_failure_title_="GATE / SERVICE CLEARANCE BLOCKED";
        corridor_placement_failure_detail_="The connector or its junction area overlaps a crew member, structure, route or reserved service space.";
    }else if(!can_start_construction(corridor_plan(*preview))){
        corridor_placement_failure_title_="CORRIDOR CANNOT BE FUNDED";
        corridor_placement_failure_detail_="Required construction resources are not available for this corridor.";
    }
    draw_compact_corridor_blueprint(r,*preview,affordable);
    render_endpoint_highlight(r,preview->endpoint_a,affordable?theme_.accent:Color{1.00f,0.28f,0.22f,1.0f},2.8f);
    render_endpoint_highlight(r,preview->endpoint_b,affordable?Color{0.42f,1.00f,0.66f,1.0f}:Color{1.00f,0.28f,0.22f,1.0f},2.8f);

    // Commit only after the full candidate connection has been submitted as a
    // blueprint in this frame. This keeps manual links consistent with corridor
    // build mode, utility channels and automatic habitat corridor previews.
    const auto previous_corridors=corridor_network_.connections();
    if(input_.left_pressed && affordable &&
       corridor_network_.connect_corridors((std::size_t)corridor_link_source_,(std::size_t)target,placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
        for(const auto& c:corridor_network_.connections())if(!gate_route_clear(c.path,geometry::kCorridorHalfWidth+0.2f)){corridor_network_.restore_connections(previous_corridors);return;}
        finalize_placement(corridor_network_.mutable_connections().back());
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

    const auto airlock_allowed=[&](const CorridorBuildAnchor& anchor){
        return anchor.kind!=CorridorEndpointKind::Habitat || airlock_connections::allows(anchor.index,anchor.position,
            placed_habitats_,corridor_network_.connections(),utility_network_.connections(),walkway_network_.connections());
    };
    const auto fail=[&](std::string title,std::string detail){
        corridor_placement_failure_title_=std::move(title);
        corridor_placement_failure_detail_=std::move(detail);
    };
    const auto invalid_anchor=[&](const CorridorBuildAnchor& anchor,bool source){
        if(anchor.kind==CorridorEndpointKind::Habitat){
            if(anchor.index<placed_habitats_.size() && placed_habitats_[anchor.index].kind==HabitatKind::AirLock && !airlock_allowed(anchor)){
                fail("AIR LOCK PASSENGER SIDE RESERVED","A corridor can use only the Air Lock passenger hatch. Supply channels remain attachable on every side.");
            }else{
                fail(source?"INVALID CORRIDOR START":"INVALID CORRIDOR END","No free pressure entrance can be created at the selected position on this structure.");
            }
        }else if(anchor.kind==CorridorEndpointKind::Resource){
            fail(source?"INVALID STORAGE DOME START":"INVALID STORAGE DOME END","This Storage Dome face is reserved or unavailable for a pressure corridor. Select another free docking face.");
        }else if(anchor.kind==CorridorEndpointKind::Corridor){
            fail(source?"INVALID CORRIDOR START":"INVALID CORRIDOR END","The selected point cannot host a corridor junction. Move farther from the existing junction/end and try again.");
        }else{
            fail(source?"INVALID CORRIDOR START":"INVALID CORRIDOR END","The selected endpoint cannot host a pressure corridor.");
        }
    };

    // Before the first click, highlight the exact structural element that would
    // become the corridor source: habitat entrance/vestibule, corridor branch
    // sleeve, or a free sealed end.
    if(!corridor_build_start_){
        highlight_habitat_anchor(hover);
        highlight_resource_anchor(hover);
        auto candidate=corridor_network_.preview_anchor(hover,source_hint(hover,hit,placed_habitats_,placed_resources_),placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        if(!airlock_allowed(hover))candidate.reset();
        // The first click selects only a source node. Do not preview a real
        // corridor run yet: that created the impression that every structure
        // connection started with a long mandatory segment. Structure anchors
        // show a compact 0.56 m docking hint; the full vestibule/corridor appears
        // only after the source has been selected and a destination exists.
        const bool start_valid=candidate.has_value();
        if(!candidate) invalid_anchor(hover,true);
        if(candidate){
            const Color c=candidate->kind==CorridorEndpointKind::OpenDoor?Color{0.96f,0.67f,0.22f,1.0f}:theme_.accent;
            if(candidate->kind==CorridorEndpointKind::Habitat || candidate->kind==CorridorEndpointKind::Resource){
                const CorridorEndpoint compact=compact_structure_start_endpoint(*candidate);
                const auto entrance_mesh=corridor_network_.preview_endpoint_mesh(compact);
                mesh_wireframe::draw_layer(r,entrance_mesh.shell,{0.34f,1.00f,0.72f,0.88f},1.35f);
                mesh_wireframe::draw_layer(r,entrance_mesh.details,{0.24f,0.88f,0.68f,0.82f},1.15f);
                r.draw_lines_3d(entrance_mesh.edges,c,1.8f);
                render_endpoint_highlight(r,compact,c,3.0f);
            }else if(candidate->kind==CorridorEndpointKind::Corridor){
                const auto junction_mesh=corridor_network_.preview_endpoint_mesh(*candidate);
                mesh_wireframe::draw_layer(r,junction_mesh.shell,{0.34f,1.00f,0.72f,0.88f},1.35f);
                mesh_wireframe::draw_layer(r,junction_mesh.details,{0.24f,0.88f,0.68f,0.82f},1.15f);
                r.draw_lines_3d(junction_mesh.edges,c,1.8f);
                render_endpoint_highlight(r,*candidate,c,3.0f);
            }else{
                draw_corridor_start_sample(r,candidate->position,candidate->tangent,true);
                render_endpoint_highlight(r,*candidate,c,3.0f);
            }
        }else{
            draw_corridor_start_sample(r,hit,Vec3{1,0,0},false);
        }
        if(input_.left_pressed && start_valid){
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
        const bool source_airlock_allowed=airlock_allowed(*corridor_build_start_);
        const bool destination_airlock_allowed=airlock_allowed(hover);
        preview=corridor_network_.preview_built_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        if(!source_airlock_allowed || !destination_airlock_allowed)preview.reset();
        if(preview){
            source_endpoint=preview->endpoint_a;
            destination_endpoint=preview->endpoint_b;
            const bool gate_clear=corridor_gate_layout_clear(*preview);
            const bool affordable=(can_start_construction(corridor_plan(*preview)) && gate_clear);
            if(!gate_clear){
                fail("GATE / SERVICE CLEARANCE BLOCKED","The corridor or entrance service area overlaps a structure, existing corridor, walkway, supply channel, or protected perimeter space.");
            }else if(!can_start_construction(corridor_plan(*preview))){
                fail("CORRIDOR CANNOT BE FUNDED","Required construction resources are not available for this corridor.");
            }
            draw_compact_corridor_blueprint(r,*preview,affordable);
        }else{
            // Even if the route is blocked, resolve both endpoint structures so
            // the player can see which existing/generated entrance is being
            // considered at the source and destination.
            source_endpoint=corridor_network_.preview_anchor(*corridor_build_start_,hover.position,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            const Vec3 other=source_endpoint?source_endpoint->position:corridor_build_start_->position;
            destination_endpoint=corridor_network_.preview_anchor(hover,other,placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            if(!source_airlock_allowed)source_endpoint.reset();
            if(!destination_airlock_allowed)destination_endpoint.reset();

            if(!source_airlock_allowed) invalid_anchor(*corridor_build_start_,true);
            else if(!destination_airlock_allowed) invalid_anchor(hover,false);
            else if(same_structure_anchor(*corridor_build_start_,hover)){
                fail("SAME STRUCTURE","A corridor must connect two different structures, corridor nodes, or an open endpoint.");
            }else if(corridor_build_start_->kind==CorridorEndpointKind::Corridor && hover.kind==CorridorEndpointKind::Corridor &&
                    corridor_build_start_->index==hover.index &&
                    distance_xz(corridor_network_.nearest_point_on_corridor(corridor_build_start_->index,corridor_build_start_->position),
                                corridor_network_.nearest_point_on_corridor(hover.index,hover.position))<0.75f){
                fail("CORRIDOR POINTS TOO CLOSE","Move the destination farther along the existing corridor before creating a branch or bypass.");
            }else if(!source_endpoint) invalid_anchor(*corridor_build_start_,true);
            else if(!destination_endpoint) invalid_anchor(hover,false);
            else{
                fail("NO CLEAR CORRIDOR ROUTE","All tested routes are blocked by structure or corridor clearance. Move an endpoint or create more free space.");
            }
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

    const bool affordable=preview && (can_start_construction(corridor_plan(*preview)) && corridor_gate_layout_clear(*preview));
    const auto previous_corridors=corridor_network_.connections();
    if(preview && affordable){
        if(corridor_network_.build_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_,placed_resources_,resource_models_)){
            for(const auto& c:corridor_network_.connections())if(!gate_route_clear(c.path,geometry::kCorridorHalfWidth+0.2f)){
                corridor_network_.restore_connections(previous_corridors);
                fail("CORRIDOR MERGE BLOCKED","The merged corridor network would enter a reserved gate or service clearance area.");
                return;
            }
            if(corridor_network_.connections().size()>previous_corridors.size())
                finalize_placement(corridor_network_.mutable_connections().back());
            selected_corridor_=(int)corridor_network_.connections().size()-1;
            selected_habitat_=-1;
            selection_kind_=SelectionKind::Corridor;
            corridor_build_start_.reset();
            build_catalog_.clear_selection();
            corridor_placement_failure_title_.clear();
            corridor_placement_failure_detail_.clear();
        }else{
            fail("CORRIDOR NETWORK MERGE REJECTED","The route preview is clear, but joining it to the existing pressure network would create an invalid overlap or reserved-area conflict.");
        }
    }
}

void GameApp::render_corridor_placement_tooltip(Renderer& r){
    if(corridor_placement_failure_title_.empty()) return;
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    if(logical_w<80.0f || logical_h<80.0f) return;

    const float width=std::min(440.0f,std::max(250.0f,logical_w-16.0f));
    const auto lines=ui::wrap_text(corridor_placement_failure_detail_,width-20.0f,
                                   ui::font_scale(0.43f*GAME_FONT_BOOST,theme_));
    const float height=38.0f+14.0f*(float)std::max<std::size_t>(1,lines.size());
    const float mx=(float)input_.mouse_x/ui_scale_;
    const float my=(float)input_.mouse_y/ui_scale_;
    float x=mx+18.0f;
    if(x+width>logical_w-8.0f) x=mx-width-18.0f;
    x=std::clamp(x,8.0f,std::max(8.0f,logical_w-width-8.0f));
    const float min_y=STATUS_BAR_H+6.0f;
    const float max_y=std::max(min_y,logical_h-BUILD_BAR_H-height-6.0f);
    float y=my+18.0f;
    if(y>max_y) y=my-height-18.0f;
    y=std::clamp(y,min_y,max_y);

    const Rect tip{x,y,width,height};
    ui::panel(r,tip,theme_);
    r.draw_line({tip.x,tip.y},{tip.x+tip.w,tip.y},{1.0f,0.30f,0.22f,1.0f},2.0f);
    game_label_fitted(r,{tip.x+10.0f,tip.y+7.0f,tip.w-20.0f,17.0f},corridor_placement_failure_title_,0.58f,theme_);
    float ty=tip.y+27.0f;
    if(lines.empty()) game_label_fitted(r,{tip.x+10.0f,ty,tip.w-20.0f,13.0f},corridor_placement_failure_detail_,0.43f,theme_,true);
    else for(const auto& line_text:lines){
        game_label_fitted(r,{tip.x+10.0f,ty,tip.w-20.0f,13.0f},line_text,0.43f,theme_,true);
        ty+=14.0f;
    }
}

} // namespace game
