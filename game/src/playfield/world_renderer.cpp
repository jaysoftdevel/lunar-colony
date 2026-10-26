#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/construction_sequence.hpp"
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
}

void GameApp::render_habitat_mesh(Renderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,std::uint8_t construction_step) const {
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

    const auto shell=transformed_vertices(mesh.shell,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
    const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    const auto glazing=transformed_vertices(mesh.glazing,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
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

void GameApp::render_resource_mesh(Renderer& r,const ResourceMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,std::uint8_t construction_step) const {
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

    const auto body=transformed_vertices(mesh.body,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);
    const float boost=selected?1.45f:1.0f;
    mesh_wireframe::draw_layer(r,foundation,{0.42f,0.48f,0.50f,1.0f},1.05f*boost);
    mesh_wireframe::draw_layer(r,body,{0.57f,0.67f,0.69f,1.0f},1.10f*boost);
    mesh_wireframe::draw_layer(r,details,{0.44f,0.54f,0.56f,0.96f},1.00f*boost);
    r.draw_lines_3d(edges,selected?theme_.accent:Color{0.10f,0.16f,0.18f,1.0f},selected?2.4f:1.15f);
}

void GameApp::render_resources(Renderer& r){
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& instance=placed_resources_[i];
        const bool selected=selection_kind_==SelectionKind::Resource && selected_resource_==(int)i;
        render_resource_mesh(r,resource_models_.mesh(instance.kind),instance.position,instance.rotation_degrees,false,true,selected,instance.construction.step);
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
        mesh_wireframe::draw_layer(r,connection.mesh.details,detail_color,1.00f*boost);
        if(habitat_view_mode_==HabitatViewMode::Default){
            mesh_wireframe::draw_layer(r,connection.mesh.shell,shell_color,1.10f*boost);
        }else{
            mesh_wireframe::draw_layer(r,connection.mesh.shell,{shell_color.r,shell_color.g,shell_color.b,0.14f},0.80f);
        }
        r.draw_lines_3d(connection.mesh.edges,edge_color,(selected||link_source)?2.5f:1.15f);
    }
}


void GameApp::render_habitats(Renderer& r){
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(instance);
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        render_habitat_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.step);

        // Entrance-specific exterior access stays visible only while that hatch
        // opens directly to the lunar surface. Once a corridor docks to the
        // same built-in entrance, remove the stairs so they cannot intersect
        // the airtight corridor sleeve/walkway.
        if(instance.construction.step>=9) for(size_t entrance_index=0;entrance_index<mesh.entrance_access.size();++entrance_index){
            if(corridor_network_.habitat_entrance_connected(i,entrance_index)) continue;
            const auto& access=mesh.entrance_access[entrance_index];
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
