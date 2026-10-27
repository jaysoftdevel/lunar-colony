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

namespace {
std::string next_material_cost(const ConstructionResourcePlan& plan,std::uint8_t completed_steps){
    const ResourceBundle& c=plan.next_step_cost(completed_steps);
    std::ostringstream out;
    out<<"NEXT  FE "<<c.iron<<"  CU "<<c.copper<<"  SI "<<c.silicon<<"  P "<<c.plastic;
    return out.str();
}

std::string total_material_cost(const ConstructionResourcePlan& plan){
    const ResourceBundle c=plan.total();
    std::ostringstream out;
    out<<"TOTAL  FE "<<c.iron<<"  CU "<<c.copper<<"  SI "<<c.silicon<<"  P "<<c.plastic;
    return out.str();
}

std::string used_material_cost(const ConstructionResourcePlan& plan,std::uint8_t completed_steps){
    const ResourceBundle c=plan.consumed(completed_steps);
    std::ostringstream out;
    out<<"USED / FREED";
    bool any=false;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        const std::int64_t amount=c.amount(material);
        if(amount<=0) continue;
        out<<"  "<<material_resource_symbol(material)<<" "<<amount;
        any=true;
    }
    if(!any) out<<"  NONE YET";
    return out.str();
}
}

void GameApp::render_selection_panel(Renderer& r){
    if(selection_kind_==SelectionKind::GroundResource){
        if(selected_ground_resources_.size()>1){
            const Rect p=selection_panel_rect();
            ui::panel(r,p,theme_);
            r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2.0f);
            const float x=p.x+14.0f;
            ResourceBundle totals{};
            float condition_sum=0.0f;
            std::size_t valid_count=0;
            for(int index:selected_ground_resources_){
                if(index<0 || (std::size_t)index>=ground_resource_piles_.size()) continue;
                const auto& box=ground_resource_piles_[(std::size_t)index];
                totals.amount(box.resource)+=box.amount;
                condition_sum+=box.condition;
                ++valid_count;
            }
            if(valid_count==0) return;
            game_label(r,{x,p.y+10},"RESOURCE BOX SELECTION",0.62f,theme_,true);
            game_label(r,{x,p.y+34},std::to_string(valid_count)+" BOXES SELECTED",0.70f,theme_);
            std::ostringstream mass;
            mass<<"TOTAL MASS  "<<totals.total_amount()<<" KG";
            game_label(r,{x,p.y+58},mass.str(),0.52f,theme_);
            std::ostringstream cond;
            cond<<"AVERAGE CONDITION  "<<(int)std::lround((condition_sum/(float)valid_count)*100.0f)<<"%";
            game_label(r,{x,p.y+78},cond.str(),0.47f,theme_,true);
            float y=p.y+104.0f;
            for(std::size_t i=0;i<kMaterialResourceCount;++i){
                const auto material=static_cast<MaterialResource>(i);
                const auto amount=totals.amount(material);
                if(amount<=0) continue;
                std::ostringstream line;
                line<<material_resource_name(material)<<"  "<<amount<<" KG";
                game_label(r,{x,y},line.str(),0.48f,theme_);
                y+=19.0f;
            }
            game_label(r,{x,p.y+p.h-28.0f},debug_frame_enabled_?"DEL  MOVE SELECTED BOXES TO STORAGE":"DEBUG REQUIRED FOR DEL",0.43f,theme_,true);
            return;
        }
        if(selected_ground_resource_<0 || (std::size_t)selected_ground_resource_>=ground_resource_piles_.size()) return;
        const auto& box=ground_resource_piles_[(std::size_t)selected_ground_resource_];
        const Rect p=ground_resource_panel_rect();
        ui::panel(r,p,theme_);
        r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2.0f);
        const float x=p.x+11.0f;
        const int condition=(int)std::lround(std::clamp(box.condition,0.0f,1.0f)*100.0f);
        const char* grade=condition>=95?"EXCELLENT":(condition>=85?"GOOD":(condition>=70?"WORN":"DAMAGED"));
        std::ostringstream amount,cond;
        amount<<"AMOUNT  "<<box.amount<<" KG";
        cond<<"CONDITION  "<<condition<<"% / "<<grade;
        game_label(r,{x,p.y+9},"RESOURCE BOX",0.56f,theme_,true);
        game_label(r,{x,p.y+31},material_resource_name(box.resource),0.72f,theme_);
        game_label(r,{x,p.y+55},amount.str(),0.52f,theme_);
        game_label(r,{x,p.y+76},cond.str(),0.50f,theme_,true);
        bool can_store=false;
        for(const auto& structure:placed_resources_){
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            const std::int64_t used=stored_resource_box_totals(structure).total_amount();
            if(used+box.amount<=storage_dome_capacity_kg(structure.storage_size)){ can_store=true; break; }
        }
        const char* action=!debug_frame_enabled_?"DEBUG REQUIRED FOR DEL":(can_store?"DEL  MOVE TO STORAGE DOME":"NO AVAILABLE STORAGE DOME");
        game_label(r,{x,p.y+97},action,0.44f,theme_,true);
        return;
    }

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
            game_label(r,{x,p.y+73},next_material_cost(h.resource_plan,h.construction.step),0.43f,theme_,true);
        }
        game_label(r,{x,p.y+88},used_material_cost(h.resource_plan,h.construction.step),0.44f,theme_,true);
        game_label(r,{x,p.y+112},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+136},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+160},conn.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+184},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+224},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Resource && selected_resource_>=0 && (size_t)selected_resource_<placed_resources_.size()){
        const auto& item=placed_resources_[(size_t)selected_resource_];
        std::ostringstream pos,rot,utilities;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<item.position.x<<" / "<<item.position.z;
        rot<<"ROTATION "<<(int)std::round(item.rotation_degrees)<<" DEG";
        utilities<<"SUPPLY LINKS "<<(utility_network_.endpoint_connection_count(UtilityEndpointKind::Resource,(size_t)selected_resource_)+walkway_network_.endpoint_connection_count(WalkwayEndpointKind::Resource,(size_t)selected_resource_));
        game_label(r,{x,p.y+10},"RESOURCE #"+std::to_string(selected_resource_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},item.kind==ResourceKind::StorageDome?std::string(resource_name(item.kind))+" / "+storage_dome_size_name(item.storage_size):resource_name(item.kind),0.76f,theme_);
        if(item.construction.complete()){
            game_label(r,{x,p.y+60},"STATUS  PLACED / AVAILABLE",0.58f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)item.construction.step<<" / "<<(int)kConstructionSteps<<"  "<<(int)std::lround(item.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+60},build.str(),0.58f,theme_);
            game_label(r,{x,p.y+73},next_material_cost(item.resource_plan,item.construction.step),0.43f,theme_,true);
        }
        if(item.kind==ResourceKind::StorageDome){
            const ResourceBundle stored_resources=stored_resource_box_totals(item);
            const std::int64_t stored=stored_resources.total_amount();
            std::ostringstream capacity,corridors,boxes;
            capacity<<"STORAGE (BOX SUM)  "<<stored<<" / "<<storage_dome_capacity_kg(item.storage_size)<<" KG";
            boxes<<"PHYSICAL BOXES  "<<item.stored_boxes.size();
            corridors<<"CORRIDORS "<<corridor_network_.resource_connection_count((size_t)selected_resource_);
            game_label(r,{x,p.y+84},used_material_cost(item.resource_plan,item.construction.step),0.44f,theme_,true);
            game_label(r,{x,p.y+108},capacity.str(),0.56f,theme_,true);
            game_label(r,{x,p.y+130},boxes.str(),0.50f,theme_,true);
            game_label(r,{x,p.y+154},"STORED RESOURCES",0.52f,theme_);

            for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                const auto material=static_cast<MaterialResource>(ri);
                const int col=ri<4?0:1;
                const int row=col==0?(int)ri:(int)ri-4;
                const float ix=x+(col==0?0.0f:235.0f);
                const float iy=p.y+177.0f+(float)row*22.0f;
                std::ostringstream line;
                line<<material_resource_name(material)<<"  "<<stored_resources.amount(material)<<" KG";
                game_label(r,{ix,iy},line.str(),0.48f,theme_,true);
            }

            game_label(r,{x,p.y+270},pos.str(),0.50f,theme_,true);
            game_label(r,{x,p.y+292},corridors.str()+" / "+utilities.str(),0.47f,theme_,true);
            game_label(r,{x,p.y+322},"DEL REMOVE / LMB SELECT OTHER",0.46f,theme_,true);
            return;
        }
        game_label(r,{x,p.y+88},used_material_cost(item.resource_plan,item.construction.step),0.44f,theme_,true);
        game_label(r,{x,p.y+112},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+136},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+160},utilities.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+224},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
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
            game_label(r,{x,p.y+73},next_material_cost(u.resource_plan,u.construction.step),0.43f,theme_,true);
        }
        game_label(r,{x,p.y+88},used_material_cost(u.resource_plan,u.construction.step),0.43f,theme_,true);
        game_label(r,{x,p.y+110},total_material_cost(u.resource_plan),0.43f,theme_,true);
        game_label(r,{x,p.y+134},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+158},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+224},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
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
        if(w.construction.complete()){
            game_label(r,{x,p.y+60},"STATUS  ACCESS + POWER + WATER",0.56f,theme_);
        }else{
            std::ostringstream build;
            build<<"CONSTRUCTION  STEP "<<(int)w.construction.step<<" / "<<(int)kWalkwayConstructionSteps
                 <<"  "<<(int)std::lround(w.construction.progress()*100.0f)<<"%";
            game_label(r,{x,p.y+60},build.str(),0.56f,theme_);
            game_label(r,{x,p.y+73},next_material_cost(w.resource_plan,w.construction.step),0.43f,theme_,true);
        }
        game_label(r,{x,p.y+88},used_material_cost(w.resource_plan,w.construction.step),0.43f,theme_,true);
        game_label(r,{x,p.y+110},total_material_cost(w.resource_plan),0.43f,theme_,true);
        game_label(r,{x,p.y+134},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+158},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+224},"DEL REMOVE / LMB SELECT OTHER",0.50f,theme_,true);
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
            game_label(r,{x,p.y+72},next_material_cost(c.resource_plan,c.construction.step),0.43f,theme_,true);
        }
        game_label(r,{x,p.y+87},used_material_cost(c.resource_plan,c.construction.step),0.43f,theme_,true);
        game_label(r,{x,p.y+109},total_material_cost(c.resource_plan),0.43f,theme_,true);
        game_label(r,{x,p.y+133},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+156},segments.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+179},endpoints.str(),0.50f,theme_,true);
        game_label(r,{x,p.y+204},"DEL REMOVE",0.48f,theme_,true);

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
