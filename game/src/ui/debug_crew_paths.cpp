#include "game/game_app.hpp"
#include <vector>

namespace game {
using namespace eng;

namespace {
void append_polyline(std::vector<Vec3>& lines,const std::vector<Vec3>& path,float lift){
    if(path.size()<2) return;
    for(std::size_t i=0;i+1<path.size();++i){
        Vec3 a=path[i];
        Vec3 b=path[i+1];
        a.y+=lift;
        b.y+=lift;
        lines.push_back(a);
        lines.push_back(b);
    }
}

void append_link(std::vector<Vec3>& lines,Vec3 a,Vec3 b,float lift){
    a.y+=lift; b.y+=lift;
    lines.push_back(a); lines.push_back(b);
}
}

void GameApp::render_debug_crew_paths(Renderer& r){
    if(!debug_frame_enabled_ || !debug_crew_paths_enabled_) return;

    std::vector<Vec3> lines;
    // Pressure corridors: show the crew centerline and extend the debug path
    // through the docking adapter to the actual habitat door plane.
    for(const auto& c:corridor_network_.connections()){
        append_polyline(lines,c.path,0.40f);
        if(!c.path.empty() && c.endpoint_a.kind==CorridorEndpointKind::Habitat)
            append_link(lines,c.endpoint_a.door.wall_position,c.path.front(),0.40f);
        if(!c.path.empty() && c.endpoint_b.kind==CorridorEndpointKind::Habitat)
            append_link(lines,c.path.back(),c.endpoint_b.door.wall_position,0.40f);
    }

    // Exterior walkways are also crew-accessible.  They intentionally carry no
    // atmosphere, but are part of the same accessibility graph.
    for(const auto& w:walkway_network_.connections()){
        append_polyline(lines,w.path,0.34f);
        if(!w.path.empty() && w.endpoint_a.kind==WalkwayEndpointKind::Habitat)
            append_link(lines,w.endpoint_a.door.wall_position,w.path.front(),0.34f);
        if(!w.path.empty() && w.endpoint_b.kind==WalkwayEndpointKind::Habitat)
            append_link(lines,w.path.back(),w.endpoint_b.door.wall_position,0.34f);
    }

    if(!lines.empty()) r.draw_lines_3d(lines,{0.12f,0.50f,1.00f,1.0f},3.0f);
}

} // namespace game
