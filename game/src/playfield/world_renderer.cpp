#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/construction_sequence.hpp"
#include "game/world/geometry_standards.hpp"
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

namespace {
std::vector<Vec3> construction_path_lines(const std::vector<Vec3>& path){
    std::vector<Vec3> lines;
    if(path.size()<2) return lines;
    lines.reserve((path.size()-1)*2);
    for(std::size_t i=0;i+1<path.size();++i){
        lines.push_back(path[i]);
        lines.push_back(path[i+1]);
    }
    return lines;
}

bool entrance_access_hits_corridor(const HabitatEntranceAccess& access,Vec3 structure_position,float rotation_degrees,
                                  const CorridorNetwork& network){
    // Authored exterior stairs/ramps are only surface-access geometry.  If a
    // pressurised corridor occupies that volume, hide the access set instead
    // of rendering stairs through the pressure tube.
    const float clearance=geometry::kCorridorHalfWidth+0.18f;
    auto vertex_hits=[&](Vec3 local){
        const Vec3 world=world_math::rotate_y(local,rotation_degrees)+structure_position;
        for(const auto& corridor:network.connections()){
            if(corridor.path.size()<2) continue;
            for(std::size_t i=0;i+1<corridor.path.size();++i){
                if(world_math::point_segment_distance_xz(world,corridor.path[i],corridor.path[i+1])<clearance) return true;
            }
        }
        return false;
    };
    for(const Vec3& v:access.details) if(vertex_hits(v)) return true;
    for(const Vec3& v:access.edges) if(vertex_hits(v)) return true;
    return false;
}

Color resource_box_color(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return {0.48f,0.52f,0.56f,1.0f};
        case MaterialResource::Copper: return {0.74f,0.42f,0.20f,1.0f};
        case MaterialResource::Silicon: return {0.28f,0.58f,0.76f,1.0f};
        case MaterialResource::Plastic: return {0.72f,0.70f,0.58f,1.0f};
        case MaterialResource::Water: return {0.22f,0.58f,0.86f,1.0f};
        case MaterialResource::Food: return {0.48f,0.72f,0.28f,1.0f};
        case MaterialResource::Medical: return {0.88f,0.42f,0.48f,1.0f};
        case MaterialResource::Count: break;
    }
    return {0.6f,0.6f,0.6f,1.0f};
}

void draw_resource_box(Renderer& r,Vec3 base,float half_width,float height,MaterialResource resource,bool selected=false){
    const Vec3 c=base+Vec3{0,height*0.5f+0.025f,0};
    const float x0=c.x-half_width,x1=c.x+half_width,z0=c.z-half_width,z1=c.z+half_width,y0=c.y-height*0.5f,y1=c.y+height*0.5f;
    const std::vector<Vec3> tris{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z1},{x0,y0,z0},{x1,y0,z1},{x0,y0,z1},
        {x0,y1,z0},{x1,y1,z1},{x1,y1,z0},{x0,y1,z0},{x0,y1,z1},{x1,y1,z1},
        {x0,y0,z0},{x0,y1,z0},{x1,y1,z0},{x0,y0,z0},{x1,y1,z0},{x1,y0,z0},
        {x1,y0,z1},{x1,y1,z1},{x0,y1,z1},{x1,y0,z1},{x0,y1,z1},{x0,y0,z1},
        {x0,y0,z1},{x0,y1,z1},{x0,y1,z0},{x0,y0,z1},{x0,y1,z0},{x0,y0,z0},
        {x1,y0,z0},{x1,y1,z0},{x1,y1,z1},{x1,y0,z0},{x1,y1,z1},{x1,y0,z1}
    };
    const std::vector<Vec3> edges{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z0},{x1,y0,z1},{x1,y0,z1},{x0,y0,z1},{x0,y0,z1},{x0,y0,z0},
        {x0,y1,z0},{x1,y1,z0},{x1,y1,z0},{x1,y1,z1},{x1,y1,z1},{x0,y1,z1},{x0,y1,z1},{x0,y1,z0},
        {x0,y0,z0},{x0,y1,z0},{x1,y0,z0},{x1,y1,z0},{x1,y0,z1},{x1,y1,z1},{x0,y0,z1},{x0,y1,z1}
    };
    const Color base_color=resource_box_color(resource);
    r.draw_triangles_3d(tris,{base_color.r,base_color.g,base_color.b,0.90f});
    r.draw_lines_3d(edges,selected?Color{0.34f,1.00f,0.72f,1.0f}:Color{0.10f,0.12f,0.13f,1.0f},selected?2.8f:1.25f);
}


bool point_in_door_aperture(Vec3 p,const CorridorDoor& door){
    Vec3 n=door.normal; n.y=0.0f; n=normalize(n);
    if(length(n)<0.001f) n={1,0,0};
    const Vec3 side{-n.z,0,n.x};
    const float base_y=std::max(0.03f,door.wall_position.y-1.18f);
    const Vec3 rel=p-door.wall_position;
    const float plane=std::abs(dot(rel,n));
    const float lateral=std::abs(dot(rel,side));
    return plane<0.72f && lateral<1.12f && p.y>base_y-0.08f && p.y<base_y+2.68f;
}

std::vector<Vec3> carve_door_apertures(const std::vector<Vec3>& triangles,const std::vector<CorridorDoor>& doors){
    if(doors.empty() || triangles.size()<3) return triangles;
    std::vector<Vec3> out;
    out.reserve(triangles.size());
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const Vec3 a=triangles[i],b=triangles[i+1],c=triangles[i+2];
        const Vec3 centroid=(a+b+c)*(1.0f/3.0f);
        bool remove=false;
        for(const auto& door:doors){
            if(point_in_door_aperture(a,door) || point_in_door_aperture(b,door) ||
               point_in_door_aperture(c,door) || point_in_door_aperture(centroid,door)){
                remove=true; break;
            }
        }
        if(!remove){ out.push_back(a); out.push_back(b); out.push_back(c); }
    }
    return out;
}

void draw_sliding_door(Renderer& r,const CorridorDoor& door,float open_fraction){
    Vec3 n=door.normal; n.y=0.0f; n=normalize(n);
    if(length(n)<0.001f) n={1,0,0};
    const Vec3 side{-n.z,0,n.x};
    const float base_y=std::max(0.04f,door.wall_position.y-1.18f);
    const float panel_h=2.48f;
    const float panel_half=0.52f;
    const float slide=0.84f*std::clamp(open_fraction,0.0f,1.0f);
    const Vec3 base{door.wall_position.x,base_y+panel_h*0.5f,door.wall_position.z};
    const Color fill{0.30f,0.39f,0.43f,0.96f};
    const Color edge_color{0.08f,0.13f,0.15f,1.0f};
    for(int sign:{-1,1}){
        const float center_side=(float)sign*(panel_half+slide);
        const Vec3 center=base+side*center_side+n*0.035f;
        const Vec3 s=side*panel_half;
        const Vec3 u{0,panel_h*0.5f,0};
        const Vec3 p0=center-s-u,p1=center+s-u,p2=center+s+u,p3=center-s+u;
        r.draw_triangles_3d({p0,p1,p2,p0,p2,p3},fill);
        r.draw_lines_3d({p0,p1,p1,p2,p2,p3,p3,p0},edge_color,1.25f);
    }
}
}

void GameApp::render_habitat_mesh(Renderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,std::uint8_t construction_step,const std::vector<CorridorDoor>* fpv_doors) const {
    if(preview){
        const auto shell=transformed_vertices(mesh.shell,position,rotation_degrees);
        const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
        const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
        const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
        const auto glazing=transformed_vertices(mesh.glazing,position,rotation_degrees);
        const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
        const Color line=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        const Color faint=valid?Color{0.18f,0.78f,0.56f,0.78f}:Color{0.92f,0.25f,0.20f,0.78f};
        mesh_wireframe::draw_layer(r,foundation,faint,1.25f);
        mesh_wireframe::draw_layer(r,shell,line,1.35f);
        mesh_wireframe::draw_layer(r,interior,faint,1.05f);
        mesh_wireframe::draw_layer(r,details,faint,1.15f);
        mesh_wireframe::draw_layer(r,glazing,{line.r,line.g,line.b,0.62f},1.0f);
        r.draw_lines_3d(edges,line,1.8f);
        for(const auto& access:mesh.entrance_access){
            const auto access_details=transformed_vertices(access.details,position,rotation_degrees);
            const auto access_edges=transformed_vertices(access.edges,position,rotation_degrees);
            mesh_wireframe::draw_layer(r,access_details,faint,1.15f);
            r.draw_lines_3d(access_edges,line,1.6f);
        }
        return;
    }

    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::habitat_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto shell=transformed_vertices(construction_sequence::reveal_triangles(mesh.shell,visibility.shell),position,rotation_degrees);
        const auto interior=transformed_vertices(construction_sequence::reveal_triangles(mesh.interior,visibility.interior),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto glazing=transformed_vertices(construction_sequence::reveal_triangles(mesh.glazing,visibility.glazing),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(mesh.edges,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.40f:1.0f;
        const Color build_main{0.78f,0.66f,0.30f,1.0f};
        const Color build_dim{0.58f,0.53f,0.32f,0.92f};
        mesh_wireframe::draw_layer(r,foundation,build_dim,1.05f*boost);
        mesh_wireframe::draw_layer(r,shell,build_main,1.15f*boost);
        mesh_wireframe::draw_layer(r,interior,{0.66f,0.60f,0.36f,0.94f},1.00f*boost);
        mesh_wireframe::draw_layer(r,details,{0.72f,0.61f,0.31f,0.96f},1.00f*boost);
        mesh_wireframe::draw_layer(r,glazing,{0.70f,0.68f,0.44f,0.78f},0.95f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,selected?theme_.accent:build_main,selected?2.4f:1.25f);
        if(construction_step==0){
            const float hx=mesh.footprint_x*0.5f, hz=mesh.footprint_z*0.5f;
            std::vector<Vec3> marker{{-hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,hz},{hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,-hz}};
            r.draw_lines_3d(transformed_vertices(marker,position,rotation_degrees),build_main,selected?2.5f:1.6f);
        }
        return;
    }

    auto shell=transformed_vertices(mesh.shell,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
    auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    auto glazing=transformed_vertices(mesh.glazing,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
    if(fpv_doors){
        shell=carve_door_apertures(shell,*fpv_doors);
        details=carve_door_apertures(details,*fpv_doors);
        glazing=carve_door_apertures(glazing,*fpv_doors);
    }
    const Color shell_color{0.56f,0.66f,0.70f,1.0f};
    const Color foundation_color{0.40f,0.47f,0.50f,1.0f};
    const Color interior_color{0.58f,0.64f,0.64f,0.92f};
    const Color detail_color{0.43f,0.53f,0.57f,0.96f};
    const Color glass_color{0.34f,0.58f,0.63f,0.72f};
    const Color edge_color=selected?theme_.accent:Color{0.10f,0.16f,0.18f,1.0f};
    const float selected_boost=selected?1.45f:1.0f;

    mesh_wireframe::draw_layer(r,foundation,foundation_color,1.05f*selected_boost);
    mesh_wireframe::draw_layer(r,interior,interior_color,0.95f*selected_boost);
    mesh_wireframe::draw_layer(r,details,detail_color,1.00f*selected_boost);
    mesh_wireframe::draw_layer(r,glazing,glass_color,0.90f*selected_boost);
    if(habitat_view_mode_==HabitatViewMode::Default){
        mesh_wireframe::draw_layer(r,shell,shell_color,1.10f*selected_boost);
    }else{
        mesh_wireframe::draw_layer(r,shell,{shell_color.r,shell_color.g,shell_color.b,0.14f},0.80f);
    }
    r.draw_lines_3d(edges,edge_color,selected?2.4f:1.15f);
}

void GameApp::render_resource_mesh(Renderer& r,const ResourceMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,std::uint8_t construction_step,const std::vector<CorridorDoor>* fpv_doors) const {
    if(preview){
        const auto body=transformed_vertices(mesh.body,position,rotation_degrees);
        const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
        const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
        const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
        const Color line=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        const Color faint=valid?Color{0.18f,0.78f,0.56f,0.78f}:Color{0.92f,0.25f,0.20f,0.78f};
        mesh_wireframe::draw_layer(r,foundation,faint,1.20f);
        mesh_wireframe::draw_layer(r,body,line,1.35f);
        mesh_wireframe::draw_layer(r,details,faint,1.10f);
        r.draw_lines_3d(edges,line,1.8f);
        return;
    }

    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::resource_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto body=transformed_vertices(construction_sequence::reveal_triangles(mesh.body,visibility.body),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(mesh.edges,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.40f:1.0f;
        const Color build_main{0.78f,0.66f,0.30f,1.0f};
        mesh_wireframe::draw_layer(r,foundation,{0.58f,0.53f,0.32f,0.92f},1.05f*boost);
        mesh_wireframe::draw_layer(r,body,build_main,1.15f*boost);
        mesh_wireframe::draw_layer(r,details,{0.72f,0.61f,0.31f,0.96f},1.00f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,selected?theme_.accent:build_main,selected?2.4f:1.25f);
        if(construction_step==0){
            const float hx=mesh.footprint_x*0.5f, hz=mesh.footprint_z*0.5f;
            std::vector<Vec3> marker{{-hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,hz},{hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,-hz}};
            r.draw_lines_3d(transformed_vertices(marker,position,rotation_degrees),build_main,selected?2.5f:1.6f);
        }
        return;
    }

    auto body=transformed_vertices(mesh.body,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
    if(fpv_doors){
        body=carve_door_apertures(body,*fpv_doors);
        details=carve_door_apertures(details,*fpv_doors);
    }
    const float boost=selected?1.45f:1.0f;
    mesh_wireframe::draw_layer(r,foundation,{0.42f,0.48f,0.50f,1.0f},1.05f*boost);
    mesh_wireframe::draw_layer(r,body,{0.57f,0.67f,0.69f,1.0f},1.10f*boost);
    mesh_wireframe::draw_layer(r,details,{0.44f,0.54f,0.56f,0.96f},1.00f*boost);
    r.draw_lines_3d(edges,selected?theme_.accent:Color{0.10f,0.16f,0.18f,1.0f},selected?2.4f:1.15f);
}


void GameApp::render_ground_resources(Renderer& r){
    for(std::size_t pile_index=0;pile_index<ground_resource_piles_.size();++pile_index){
        const auto& pile=ground_resource_piles_[pile_index];
        const float half=resource_package_half_width(pile.resource,pile.amount);
        const float h=resource_package_height(pile.resource,pile.amount);
        const bool selected=selection_kind_==SelectionKind::GroundResource &&
            (selected_ground_resource_==(int)pile_index ||
             std::find(selected_ground_resources_.begin(),selected_ground_resources_.end(),(int)pile_index)!=selected_ground_resources_.end());
        draw_resource_box(r,pile.position,half,h,pile.resource,selected);
    }
}

void GameApp::render_resources(Renderer& r){
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& instance=placed_resources_[i];
        const bool selected=selection_kind_==SelectionKind::Resource && selected_resource_==(int)i;
        const auto& mesh=resource_models_.mesh(instance);
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(first_person_mode_==FirstPersonMode::Active && instance.kind==ResourceKind::StorageDome && instance.construction.complete()){
            fpv_doors=storage_dome_doors(i);
            fpv_ptr=&fpv_doors;
        }
        render_resource_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.step,fpv_ptr);
        if(fpv_ptr) for(const auto& door:fpv_doors) draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position));

        if(instance.kind!=ResourceKind::StorageDome || !instance.construction.complete() || instance.stored_boxes.empty()) continue;

        // Every stored resource remains a physical human-portable package. The
        // same deterministic pallet pose is used by FPV picking, so the box a
        // player sees is exactly the box that can be grabbed.
        for(std::size_t bi=0;bi<instance.stored_boxes.size();++bi){
            Vec3 base{}; float half=0.0f,box_h=0.0f;
            stored_resource_box_pose(instance,mesh,bi,base,half,box_h);
            draw_resource_box(r,base,half,box_h,instance.stored_boxes[bi].resource,false);
        }
    }
}

void GameApp::render_utilities(Renderer& r){
    const auto& links=utility_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& link=links[i];
        const bool selected=selection_kind_==SelectionKind::Utility && selected_utility_==(int)i;
        const float boost=selected?1.55f:1.0f;

        if(!link.construction.complete()){
            const float fraction=link.construction.progress();
            const Color build_main{0.80f,0.66f,0.27f,1.0f};
            const Color build_detail{0.66f,0.55f,0.30f,0.96f};
            if(link.construction.step==0){
                r.draw_lines_3d(construction_path_lines(link.path),build_main,selected?2.5f:1.5f);
            }else{
                const auto shell=construction_sequence::reveal_triangles(link.mesh.shell,fraction);
                const auto details=construction_sequence::reveal_triangles(link.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(link.mesh.edges,fraction);
                mesh_wireframe::draw_layer(r,shell,build_main,1.10f*boost);
                mesh_wireframe::draw_layer(r,details,build_detail,1.00f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,selected?theme_.accent:build_main,selected?2.5f:1.20f);
            }
            continue;
        }

        const Color shell{0.48f,0.68f,0.80f,1.0f};
        const Color details{0.70f,0.56f,0.24f,0.96f};
        mesh_wireframe::draw_layer(r,link.mesh.shell,shell,1.05f*boost);
        mesh_wireframe::draw_layer(r,link.mesh.details,details,1.00f*boost);
        r.draw_lines_3d(link.mesh.edges,selected?theme_.accent:Color{0.10f,0.14f,0.15f,1.0f},selected?2.5f:1.15f);
    }
}


void GameApp::render_walkways(Renderer& r){
    const auto& links=walkway_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& w=links[i];
        const bool selected=selection_kind_==SelectionKind::Walkway && selected_walkway_==(int)i;
        const float boost=selected?1.55f:1.0f;
        if(!w.construction.complete()){
            const float fraction=w.construction.progress();
            const Color build_main{0.80f,0.66f,0.27f,1.0f};
            const Color build_detail{0.66f,0.58f,0.34f,0.96f};
            if(w.construction.step==0){
                r.draw_lines_3d(construction_path_lines(w.path),build_main,selected?2.6f:1.6f);
            }else{
                const auto deck=construction_sequence::reveal_triangles(w.mesh.deck,fraction);
                const auto details=construction_sequence::reveal_triangles(w.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(w.mesh.edges,fraction);
                mesh_wireframe::draw_layer(r,deck,build_main,1.10f*boost);
                mesh_wireframe::draw_layer(r,details,build_detail,1.00f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,selected?theme_.accent:build_main,selected?2.5f:1.20f);
            }
            continue;
        }
        mesh_wireframe::draw_layer(r,w.mesh.deck,{0.54f,0.64f,0.66f,1.0f},1.10f*boost);
        mesh_wireframe::draw_layer(r,w.mesh.details,{0.72f,0.58f,0.25f,0.96f},1.00f*boost);
        r.draw_lines_3d(w.mesh.edges,selected?theme_.accent:Color{0.10f,0.15f,0.16f,1.0f},selected?2.5f:1.15f);
    }
}

void GameApp::render_corridors(Renderer& r){
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& connection=corridors[i];
        const bool selected=selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)i;
        const bool link_source=corridor_link_source_==(int)i;
        const Color edge_color=(selected||link_source)?theme_.accent:Color{0.09f,0.15f,0.17f,1.0f};
        const float boost=(selected||link_source)?1.55f:1.0f;

        if(!connection.construction.complete()){
            const float fraction=connection.construction.progress();
            const Color build_main{0.80f,0.66f,0.27f,1.0f};
            const Color build_detail{0.66f,0.58f,0.34f,0.96f};
            if(connection.construction.step==0){
                r.draw_lines_3d(construction_path_lines(connection.path),build_main,(selected||link_source)?2.6f:1.6f);
            }else{
                const auto shell=construction_sequence::reveal_triangles(connection.mesh.shell,fraction);
                const auto details=construction_sequence::reveal_triangles(connection.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(connection.mesh.edges,fraction);
                mesh_wireframe::draw_layer(r,details,build_detail,1.00f*boost);
                mesh_wireframe::draw_layer(r,shell,build_main,1.10f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,(selected||link_source)?theme_.accent:build_main,(selected||link_source)?2.5f:1.20f);
            }
            continue;
        }

        const Color shell_color{0.50f,0.62f,0.66f,1.0f};
        const Color detail_color{0.42f,0.54f,0.58f,0.95f};
        auto corridor_details=connection.mesh.details;
        std::vector<CorridorDoor> open_end_doors;
        if(first_person_mode_==FirstPersonMode::Active){
            for(const CorridorEndpoint* ep:{&connection.endpoint_a,&connection.endpoint_b}){
                if(ep->kind!=CorridorEndpointKind::OpenDoor || ep->suppress_open_door) continue;
                Vec3 n=ep->tangent; n.y=0.0f; n=normalize(n);
                if(length(n)<0.001f) n={1,0,0};
                CorridorDoor inner;
                inner.position=ep->position;
                inner.wall_position=ep->position+Vec3{0,1.18f,0};
                inner.normal=n;
                CorridorDoor outer=inner;
                outer.position=ep->position-n*geometry::kCorridorAirlockLength;
                outer.wall_position=outer.position+Vec3{0,1.18f,0};
                open_end_doors.push_back(inner);
                open_end_doors.push_back(outer);
            }
            corridor_details=carve_door_apertures(corridor_details,open_end_doors);
        }
        mesh_wireframe::draw_layer(r,corridor_details,detail_color,1.00f*boost);
        if(habitat_view_mode_==HabitatViewMode::Default || first_person_mode_==FirstPersonMode::Active){
            mesh_wireframe::draw_layer(r,connection.mesh.shell,shell_color,1.10f*boost);
        }else{
            mesh_wireframe::draw_layer(r,connection.mesh.shell,{shell_color.r,shell_color.g,shell_color.b,0.14f},0.80f);
        }
        r.draw_lines_3d(connection.mesh.edges,edge_color,(selected||link_source)?2.5f:1.15f);
        if(first_person_mode_==FirstPersonMode::Active){
            // Open-end doors are stored as inner/outer pairs.  Interlock them:
            // if the proximity volumes overlap, only the nearer panel may open.
            const Vec3 feet=first_person_.feet_position();
            for(std::size_t di=0;di<open_end_doors.size();di+=2){
                const CorridorDoor& inner=open_end_doors[di];
                const CorridorDoor& outer=open_end_doors[di+1];
                float fi=first_person_door_open_fraction(inner.wall_position);
                float fo=first_person_door_open_fraction(outer.wall_position);
                if(fi>0.0f && fo>0.0f){
                    const float dix=feet.x-inner.wall_position.x,diz=feet.z-inner.wall_position.z;
                    const float dox=feet.x-outer.wall_position.x,doz=feet.z-outer.wall_position.z;
                    if(dix*dix+diz*diz<=dox*dox+doz*doz) fo=0.0f; else fi=0.0f;
                }
                draw_sliding_door(r,inner,fi);
                draw_sliding_door(r,outer,fo);
            }
        }
    }
}


void GameApp::render_habitats(Renderer& r){
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(instance);
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(first_person_mode_==FirstPersonMode::Active && instance.construction.complete()){
            fpv_doors=habitat_doors(i);
            fpv_ptr=&fpv_doors;
        }
        render_habitat_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.step,fpv_ptr);
        if(fpv_ptr) for(const auto& door:fpv_doors) draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position));

        // Entrance-specific exterior access stays visible only while that hatch
        // opens directly to the lunar surface. Once a corridor docks to the
        // same built-in entrance, remove the stairs so they cannot intersect
        // the airtight corridor sleeve/walkway.
        if(instance.construction.step>=9) for(size_t entrance_index=0;entrance_index<mesh.entrance_access.size();++entrance_index){
            if(corridor_network_.habitat_entrance_connected(i,entrance_index)) continue;
            const auto& access=mesh.entrance_access[entrance_index];
            if(entrance_access_hits_corridor(access,instance.position,instance.rotation_degrees,corridor_network_)) continue;
            const float access_fraction=instance.construction.complete()?1.0f:0.5f;
            const auto details=transformed_vertices(construction_sequence::reveal_triangles(access.details,access_fraction),instance.position,instance.rotation_degrees);
            const auto edges=transformed_vertices(construction_sequence::reveal_lines(access.edges,instance.construction.complete()?1.0f:0.0f),instance.position,instance.rotation_degrees);
            const Color access_color=instance.construction.complete()?Color{0.43f,0.53f,0.57f,0.96f}:Color{0.72f,0.61f,0.31f,0.96f};
            mesh_wireframe::draw_layer(r,details,access_color,1.0f);
            if(!edges.empty()){
                const Color ec=selected?theme_.accent:Color{0.08f,0.11f,0.13f,1.0f};
                r.draw_lines_3d(edges,ec,selected?2.4f:1.15f);
            }
        }
    }
}

} // namespace game
