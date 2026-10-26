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
        utilities<<"SUPPLY LINKS "<<(utility_network_.endpoint_connection_count(UtilityEndpointKind::Habitat,(size_t)selected_habitat_)+walkway_network_.endpoint_connection_count(WalkwayEndpointKind::Habitat,(size_t)selected_habitat_));
        game_label(r,{x,p.y+10},"HABITAT #"+std::to_string(selected_habitat_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},std::string(habitat_name(h.kind))+" / "+habitat_size_name(h.size),0.72f,theme_);
        if(h.construction.complete()){
            game_label(r,{x,p.y+60},"STATUS  OPERATIONAL / SEALED",0.58f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)h.construction.step<<" / "<<(int)kConstructionSteps<<"  "<<(int)std::lround(h.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+60},build.str(),0.58f,theme_);
        }
        game_label(r,{x,p.y+84},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+132},conn.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+154},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+188},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Resource && selected_resource_>=0 && (size_t)selected_resource_<placed_resources_.size()){
        const auto& item=placed_resources_[(size_t)selected_resource_];
        std::ostringstream pos,rot,utilities;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<item.position.x<<" / "<<item.position.z;
        rot<<"ROTATION "<<(int)std::round(item.rotation_degrees)<<" DEG";
        utilities<<"SUPPLY LINKS "<<(utility_network_.endpoint_connection_count(UtilityEndpointKind::Resource,(size_t)selected_resource_)+walkway_network_.endpoint_connection_count(WalkwayEndpointKind::Resource,(size_t)selected_resource_));
        game_label(r,{x,p.y+10},"RESOURCE #"+std::to_string(selected_resource_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},resource_name(item.kind),0.76f,theme_);
        if(item.construction.complete()){
            game_label(r,{x,p.y+60},"STATUS  PLACED / AVAILABLE",0.58f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)item.construction.step<<" / "<<(int)kConstructionSteps<<"  "<<(int)std::lround(item.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+60},build.str(),0.58f,theme_);
        }
        game_label(r,{x,p.y+84},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+132},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+170},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Utility && selected_utility_>=0 && (size_t)selected_utility_<utility_network_.connections().size()){
        const auto& u=utility_network_.connections()[(size_t)selected_utility_];
        std::ostringstream length,endpoints;
        length<<"LENGTH "<<std::fixed<<std::setprecision(1)<<utility_network_.connection_length((size_t)selected_utility_)<<" M";
        endpoints<<utility_endpoint_name(u.endpoint_a.kind);
        if(u.endpoint_a.kind!=UtilityEndpointKind::Open) endpoints<<" "<<(u.endpoint_a.index+1);
        endpoints<<" <-> "<<utility_endpoint_name(u.endpoint_b.kind);
        if(u.endpoint_b.kind!=UtilityEndpointKind::Open) endpoints<<" "<<(u.endpoint_b.index+1);
        game_label(r,{x,p.y+10},"UTILITY #"+std::to_string(selected_utility_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},utility_kind_name(u.kind),0.76f,theme_);
        const bool open_end=u.endpoint_a.kind==UtilityEndpointKind::Open || u.endpoint_b.kind==UtilityEndpointKind::Open;
        if(u.construction.complete()){
            game_label(r,{x,p.y+60},open_end?"STATUS  DISCONNECTED / CAPPED":"STATUS  CONNECTED / ONLINE",0.58f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)u.construction.step<<" / "<<(int)kSupplyConstructionSteps
                 <<"  "<<(int)std::lround(u.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+60},build.str(),0.58f,theme_);
        }
        game_label(r,{x,p.y+84},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+170},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Walkway && selected_walkway_>=0 && (size_t)selected_walkway_<walkway_network_.connections().size()){
        const auto& w=walkway_network_.connections()[(size_t)selected_walkway_];
        std::ostringstream length,endpoints;
        length<<"LENGTH "<<std::fixed<<std::setprecision(1)<<walkway_network_.connection_length((size_t)selected_walkway_)<<" M";
        endpoints<<walkway_endpoint_name(w.endpoint_a.kind);
        if(w.endpoint_a.kind!=WalkwayEndpointKind::Open) endpoints<<" "<<(w.endpoint_a.index+1);
        endpoints<<" <-> "<<walkway_endpoint_name(w.endpoint_b.kind);
        if(w.endpoint_b.kind!=WalkwayEndpointKind::Open) endpoints<<" "<<(w.endpoint_b.index+1);
        game_label(r,{x,p.y+10},"WALKWAY #"+std::to_string(selected_walkway_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},"EXTERNAL ACCESS WALKWAY",0.72f,theme_);
        game_label(r,{x,p.y+60},"STATUS  ACCESS + POWER + WATER",0.56f,theme_);
        game_label(r,{x,p.y+84},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+170},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
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
        if(c.construction.complete()){
            game_label(r,{x,p.y+59},"STATUS  PRESSURIZED / WALKABLE",0.56f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)c.construction.step<<" / "<<(int)kCorridorConstructionSteps
                 <<"  "<<(int)std::lround(c.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+59},build.str(),0.56f,theme_);
        }
        game_label(r,{x,p.y+82},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+105},segments.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+128},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+150},"DEL REMOVE",0.48f,theme_,true);

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
