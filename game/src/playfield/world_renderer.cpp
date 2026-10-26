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

void GameApp::render_habitat_mesh(Renderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected) const {
    const auto shell=transformed_vertices(mesh.shell,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
    const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    const auto glazing=transformed_vertices(mesh.glazing,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);

    if(preview){
        const Color pc=valid?Color{0.18f,0.78f,0.56f,0.30f}:Color{0.92f,0.25f,0.20f,0.30f};
        const Color pe=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        r.draw_triangles_3d(foundation,{pc.r,pc.g,pc.b,0.42f});
        r.draw_triangles_3d(shell,pc);
        r.draw_triangles_3d(interior,{pc.r,pc.g,pc.b,0.34f});
        r.draw_triangles_3d(details,{pc.r,pc.g,pc.b,0.40f});
        r.draw_triangles_3d(glazing,{pc.r,pc.g,pc.b,0.16f});
        r.draw_lines_3d(edges,pe,1.8f);
        return;
    }

    const Color shell_opaque{0.46f,0.51f,0.54f,1.0f};
    const Color foundation_color{0.31f,0.35f,0.37f,1.0f};
    const Color interior_color{0.57f,0.60f,0.60f,1.0f};
    const Color detail_color{0.34f,0.39f,0.42f,1.0f};
    const Color glass_color{0.16f,0.26f,0.29f,0.42f};
    const Color edge_color=selected?theme_.accent:Color{0.08f,0.11f,0.13f,1.0f};
    const float edge_width=selected?2.4f:1.15f;

    r.draw_triangles_3d(foundation,foundation_color);

    if(habitat_view_mode_==HabitatViewMode::Default){
        r.draw_triangles_3d(shell,shell_opaque);
        r.draw_triangles_3d(interior,interior_color);
        r.draw_triangles_3d(details,detail_color);
        r.draw_triangles_3d(glazing,glass_color);
        r.draw_lines_3d(edges,edge_color,edge_width);
        return;
    }

    r.draw_triangles_3d(interior,interior_color);
    r.draw_triangles_3d(details,detail_color);
    r.draw_triangles_3d(glazing,glass_color);
    r.draw_lines_3d(edges,{edge_color.r,edge_color.g,edge_color.b,selected?1.0f:0.52f},selected?2.4f:1.0f);
    r.draw_triangles_3d(shell,{0.46f,0.51f,0.54f,0.10f});
}

void GameApp::render_corridors(Renderer& r){
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& connection=corridors[i];
        const bool selected=selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)i;
        const bool link_source=corridor_link_source_==(int)i;
        const Color shell_color{0.37f,0.43f,0.46f,1.0f};
        const Color detail_color{0.28f,0.33f,0.36f,1.0f};
        const Color edge_color=(selected||link_source)?theme_.accent:Color{0.07f,0.10f,0.12f,1.0f};
        const float edge_width=(selected||link_source)?2.5f:1.15f;

        if(habitat_view_mode_==HabitatViewMode::Default){
            r.draw_triangles_3d(connection.mesh.shell,shell_color);
            r.draw_triangles_3d(connection.mesh.details,detail_color);
            r.draw_lines_3d(connection.mesh.edges,edge_color,edge_width);
            continue;
        }

        // FULL inspection view: only the corridor pressure hull fades. The
        // internal walkway, ribs, docking hardware, junctions and doors stay
        // visible, matching the habitat hull-only transparency rule.
        r.draw_triangles_3d(connection.mesh.details,detail_color);
        r.draw_lines_3d(connection.mesh.edges,{edge_color.r,edge_color.g,edge_color.b,(selected||link_source)?1.0f:0.52f},edge_width);
        r.draw_triangles_3d(connection.mesh.shell,{shell_color.r,shell_color.g,shell_color.b,0.10f});
    }
}

void GameApp::render_habitats(Renderer& r){
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        render_habitat_mesh(r,habitat_models_.mesh(instance.kind),instance.position,instance.rotation_degrees,false,true,selected);
    }
}

} // namespace game
