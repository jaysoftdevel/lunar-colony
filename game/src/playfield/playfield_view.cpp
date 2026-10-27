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

void GameApp::render_playfield(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    r.set_3d(camera_.view_projection(aspect));
    r.draw_triangles_3d(playfield_.floor_triangles(),{0.055f,0.075f,0.088f,1});
    r.draw_lines_3d(playfield_.grid_lines(),{0.22f,0.43f,0.47f,1},1.0f);
    render_ground_resources(r);
    render_corridors(r);
    render_utilities(r);
    render_walkways(r);
    render_habitats(r);
    render_resources(r);
    render_corridor_link_preview(r);
    render_corridor_placement(r);
    render_habitat_placement(r);
    render_resource_placement(r);
    render_utility_placement(r);
    render_walkway_placement(r);
    render_playfield_crosshair(r);

    r.set_2d(ui_scale_);
    render_status_bar(r);
    render_build_bar(r);
    if(debug_frame_enabled_) render_debug_frame(r);
    if(selection_kind_!=SelectionKind::None) render_selection_panel(r);

    Rect info={PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,430,56};
    r.draw_rect(info,{0.025f,0.035f,0.045f,0.88f});
    std::ostringstream a;
    a<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    game_label(r,{info.x+12,info.y+8},a.str(),0.78f,theme_);
    const BuildObjectDefinition* active_build=build_catalog_.selected();
    const char* controls=corridor_link_source_>=0?"LMB TARGET CORRIDOR / ESC CANCEL":
        (active_build && active_build->kind==BuildObjectKind::Corridor?
            (corridor_build_start_?"LMB SET CORRIDOR END / ESC CANCEL":"LMB SET CORRIDOR START / ESC CANCEL"):
        (active_build && active_build->kind==BuildObjectKind::Utility?
            (utility_build_start_?"MOVE TARGET TO ADJUST PORT / LMB CONNECT / ESC CANCEL":"LMB SELECT SUPPLY SOURCE / ESC CANCEL"):
        (active_build && active_build->kind==BuildObjectKind::Walkway?
            (walkway_build_start_?"MOVE TO ADJUST WALKWAY BLUEPRINT / LMB BUILD / ESC CANCEL":"LMB SET WALKWAY START / ESC CANCEL"):
        (active_build && active_build->kind==BuildObjectKind::Habitat && habitat_has_size_tiers(active_build->habitat_kind)?
            "LMB PLACE / WHEEL SIZE / N-M ROTATE / ESC CANCEL":
        (active_build?"LMB PLACE / WHEEL OR N/M ROTATE / ESC CANCEL":"LMB SELECT / WHEEL ZOOM / MMB ORBIT / RMB PAN / ESC MENU")))));
    game_label(r,{info.x+12,info.y+31},controls,0.52f,theme_,true);
}

void GameApp::render_playfield_crosshair(Renderer& r){
    // The top status strip and lower build strip are UI, not part of the
    // playfield pane. Show the generic OS pointer whenever the mouse is there.
    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px){
        app_.set_cursor_visible(true);
        return;
    }

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,r.width(),r.height(),playfield_.size(),hit)){
        app_.set_cursor_visible(true);
        return;
    }

    app_.set_cursor_visible(false);
    const float arm=std::clamp(camera_.distance()*0.030f,1.35f,5.0f);
    const float gap=arm*0.24f;
    const float y=0.055f;
    std::vector<Vec3> lines={
        {hit.x-arm,y,hit.z},{hit.x-gap,y,hit.z},
        {hit.x+gap,y,hit.z},{hit.x+arm,y,hit.z},
        {hit.x,y,hit.z-arm},{hit.x,y,hit.z-gap},
        {hit.x,y,hit.z+gap},{hit.x,y,hit.z+arm}
    };
    r.draw_lines_3d(lines,theme_.accent,2.8f);
}

} // namespace game
