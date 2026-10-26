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

void GameApp::render_selection_panel(Renderer& r){
    const Rect p=selection_panel_rect();
    ui::panel(r,p,theme_);
    r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2.0f);
    const float x=p.x+14.0f;

    if(selection_kind_==SelectionKind::Habitat && selected_habitat_>=0 && (size_t)selected_habitat_<placed_habitats_.size()){
        const auto& h=placed_habitats_[(size_t)selected_habitat_];
        std::ostringstream pos,rot,conn,utilities;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<h.position.x<<" / "<<h.position.z;
        rot<<"ROTATION "<<(int)std::round(h.rotation_degrees)<<" DEG";
        conn<<"CORRIDORS "<<corridor_network_.habitat_connection_count((size_t)selected_habitat_);
        utilities<<"UTILITIES "<<utility_network_.endpoint_connection_count(UtilityEndpointKind::Habitat,(size_t)selected_habitat_);
        game_label(r,{x,p.y+10},"HABITAT #"+std::to_string(selected_habitat_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},habitat_name(h.kind),0.76f,theme_);
        game_label(r,{x,p.y+60},"STATUS  OPERATIONAL / SEALED",0.58f,theme_);
        game_label(r,{x,p.y+84},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+132},conn.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+154},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+188},"LMB SELECT OTHER OBJECT",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Resource && selected_resource_>=0 && (size_t)selected_resource_<placed_resources_.size()){
        const auto& item=placed_resources_[(size_t)selected_resource_];
        std::ostringstream pos,rot,utilities;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<item.position.x<<" / "<<item.position.z;
        rot<<"ROTATION "<<(int)std::round(item.rotation_degrees)<<" DEG";
        utilities<<"UTILITIES "<<utility_network_.endpoint_connection_count(UtilityEndpointKind::Resource,(size_t)selected_resource_);
        game_label(r,{x,p.y+10},"RESOURCE #"+std::to_string(selected_resource_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},resource_name(item.kind),0.76f,theme_);
        game_label(r,{x,p.y+60},"STATUS  PLACED / AVAILABLE",0.58f,theme_);
        game_label(r,{x,p.y+84},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+132},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+170},"LMB SELECT OTHER OBJECT",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Utility && selected_utility_>=0 && (size_t)selected_utility_<utility_network_.connections().size()){
        const auto& u=utility_network_.connections()[(size_t)selected_utility_];
        std::ostringstream length,endpoints;
        length<<"LENGTH "<<std::fixed<<std::setprecision(1)<<utility_network_.connection_length((size_t)selected_utility_)<<" M";
        endpoints<<utility_endpoint_name(u.endpoint_a.kind)<<" "<<(u.endpoint_a.index+1)<<" <-> "
                 <<utility_endpoint_name(u.endpoint_b.kind)<<" "<<(u.endpoint_b.index+1);
        game_label(r,{x,p.y+10},"UTILITY #"+std::to_string(selected_utility_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},utility_kind_name(u.kind),0.76f,theme_);
        game_label(r,{x,p.y+60},"STATUS  CONNECTED / ONLINE",0.58f,theme_);
        game_label(r,{x,p.y+84},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+170},"LMB SELECT OTHER OBJECT",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Corridor && selected_corridor_>=0 && (size_t)selected_corridor_<corridor_network_.connections().size()){
        const auto& c=corridor_network_.connections()[(size_t)selected_corridor_];
        std::ostringstream length,segments,endpoints;
        length<<"LENGTH "<<std::fixed<<std::setprecision(1)<<corridor_network_.connection_length((size_t)selected_corridor_)<<" M";
        segments<<"CURVE SAMPLES "<<c.path.size();
        endpoints<<corridor_endpoint_name(c.endpoint_a)<<" <-> "<<corridor_endpoint_name(c.endpoint_b);
        const char* kind_label=c.kind==CorridorConnectionKind::Automatic?"AUTOMATIC LINK":
                               (c.kind==CorridorConnectionKind::NetworkLink?"NETWORK LINK":"BUILT SECTION");
        game_label(r,{x,p.y+10},"CORRIDOR #"+std::to_string(selected_corridor_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},kind_label,0.74f,theme_);
        game_label(r,{x,p.y+59},"STATUS  PRESSURIZED / WALKABLE",0.56f,theme_);
        game_label(r,{x,p.y+82},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+105},segments.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+128},endpoints.str(),0.50f,theme_,true);

        const bool linking=corridor_link_source_==selected_corridor_;
        const std::string label=linking?"CANCEL CONNECTION":"CONNECT TO CORRIDOR";
        const bool enabled=linking || corridor_network_.connections().size()>1;
        if(ui::button(r,ui_input_,{x,p.y+p.h-47.0f,220.0f,34.0f},label,theme_,enabled)){
            if(linking) corridor_link_source_=-1;
            else corridor_link_source_=selected_corridor_;
        }
        if(linking) game_label(r,{x+230.0f,p.y+p.h-38.0f},"SELECT TARGET",0.47f,theme_,true);
    }
}

} // namespace game
