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

std::string compact_bundle(const ResourceBundle& bundle,const char* empty_text="NONE"){
    std::ostringstream out;
    bool any=false;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        const auto amount=bundle.amount(material);
        if(amount<=0) continue;
        if(any) out<<"  ";
        out<<material_resource_symbol(material)<<" "<<amount;
        any=true;
    }
    if(!any) out<<empty_text;
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

    if(selection_kind_==SelectionKind::Robot && selected_robot_>=0 && (std::size_t)selected_robot_<builder_robots_.size()){
        const std::size_t robot_index=(std::size_t)selected_robot_;
        const auto& robot=builder_robots_[robot_index];

        auto robot_identity=[&](std::size_t index){
            if(index>=builder_robots_.size()) return std::string("UNKNOWN ROBOT");
            const auto role=builder_robots_[index].role;
            std::size_t number=0;
            for(std::size_t i=0;i<=index;++i) if(builder_robots_[i].role==role) ++number;
            std::ostringstream out;
            out<<robot_role_name(role)<<" #"<<number;
            return out.str();
        };
        auto target_identity=[&](const ConstructionTargetRef& ref){
            switch(ref.kind){
                case ConstructionTargetKind::Habitat:
                    if(ref.index<placed_habitats_.size()) return habitat_instance_name(placed_habitats_[ref.index]);
                    break;
                case ConstructionTargetKind::Resource:
                    if(ref.index<placed_resources_.size()) return resource_instance_name(placed_resources_[ref.index]);
                    break;
                case ConstructionTargetKind::Corridor:
                    if(ref.index<corridor_network_.connections().size()) return std::string("CORRIDOR #")+std::to_string(ref.index+1);
                    break;
                case ConstructionTargetKind::Utility:
                    if(ref.index<utility_network_.connections().size()) return std::string("SUPPLY CHANNEL #")+std::to_string(ref.index+1);
                    break;
                case ConstructionTargetKind::Walkway:
                    if(ref.index<walkway_network_.connections().size()) return std::string("WALKWAY #")+std::to_string(ref.index+1);
                    break;
                case ConstructionTargetKind::None: break;
            }
            return std::string("UNKNOWN TARGET");
        };
        auto target_progress=[&](const ConstructionTargetRef& ref){
            switch(ref.kind){
                case ConstructionTargetKind::Habitat:
                    return ref.index<placed_habitats_.size()?placed_habitats_[ref.index].construction.progress():0.0f;
                case ConstructionTargetKind::Resource:
                    return ref.index<placed_resources_.size()?placed_resources_[ref.index].construction.progress():0.0f;
                case ConstructionTargetKind::Corridor:
                    return ref.index<corridor_network_.connections().size()?corridor_network_.connections()[ref.index].construction.progress():0.0f;
                case ConstructionTargetKind::Utility:
                    return ref.index<utility_network_.connections().size()?utility_network_.connections()[ref.index].construction.progress():0.0f;
                case ConstructionTargetKind::Walkway:
                    return ref.index<walkway_network_.connections().size()?walkway_network_.connections()[ref.index].construction.progress():0.0f;
                case ConstructionTargetKind::None: break;
            }
            return 0.0f;
        };

        const std::string title=robot_identity(robot_index);
        const std::string target_name=robot.target.valid()?target_identity(robot.target):std::string{};
        std::ostringstream state,task,operation,navigation,motion,target,build,cargo,assigned,team,recovery;

        state<<"STATE  "<<builder_robot_mode_name(robot.mode);
        if(robot.action!=BuilderRobotAction::None) state<<" / "<<builder_robot_action_name(robot.action);

        if(robot.action!=BuilderRobotAction::None){
            operation<<"OPERATION  "<<builder_robot_action_name(robot.action);
            if(robot.action_amount>0){
                operation<<" "<<robot.action_amount<<" KG "<<material_resource_symbol(robot.action_material);
            }
            if(robot.action_duration>0.001f){
                const int action_pct=(int)std::lround(std::clamp(robot.action_elapsed/robot.action_duration,0.0f,1.0f)*100.0f);
                operation<<" / "<<action_pct<<"%";
            }
        }else operation<<"OPERATION  NONE";

        switch(robot.mode){
            case BuilderRobotMode::Idle:
                if(robot.post_delivery_hold || robot.drop_exit_pending) task<<"TASK  CLEARING CONSTRUCTION DROP ZONE";
                else task<<"TASK  WAITING FOR ASSIGNMENT";
                break;
            case BuilderRobotMode::AwaitingMaterials:
                task<<"TASK  WAITING FOR MATERIALS";
                if(robot.target.valid()) task<<" AT "<<target_name;
                break;
            case BuilderRobotMode::Fetching:
                if(robot.action==BuilderRobotAction::Pickup) task<<"TASK  LOADING ASSIGNED MATERIAL";
                else if(robot.interaction_position_valid) task<<"TASK  TRAVELLING TO MATERIAL PICKUP";
                else task<<"TASK  LOCATING MATERIAL SOURCE";
                break;
            case BuilderRobotMode::Delivering:
                if(robot.action==BuilderRobotAction::PutDown) task<<"TASK  UNLOADING AT CONSTRUCTION SITE";
                else if(robot.post_delivery_hold || robot.drop_exit_pending) task<<"TASK  CLEARING CONSTRUCTION DROP ZONE";
                else task<<"TASK  DELIVERING "<<robot.cargo.total_amount()<<" KG";
                if(robot.target.valid()) task<<" TO "<<target_name;
                break;
            case BuilderRobotMode::Building:
                if(robot.stage_commit_pending) task<<"TASK  CLEARING AREA FOR NEXT BUILD STAGE";
                else if(!robot.stage_materials_picked) task<<"TASK  COLLECTING DELIVERED SITE MATERIALS";
                else task<<"TASK  ASSEMBLING "<<(robot.target.valid()?target_name:std::string("CONSTRUCTION"));
                break;
            case BuilderRobotMode::DepotTransfer:
                if(robot.cargo.total_amount()>0) task<<"TASK  RETURNING RECOVERED MATERIAL TO STORAGE";
                else task<<"TASK  RECOVERING / CONSOLIDATING LOOSE RESOURCES";
                break;
        }

        const float remaining_m=robot.route_active?std::max(0.0f,robot.route_total_length_m-robot.route_progress_m):0.0f;
        if(robot.traffic_escape_active){
            navigation<<"NAVIGATION  COLLISION FALLBACK / PULL-ASIDE";
        }else if(robot.route_waiting_for_clearance){
            navigation<<"NAVIGATION  YIELDING";
            if(robot.traffic_blocker_index<builder_robots_.size()) navigation<<" TO "<<robot_identity(robot.traffic_blocker_index);
            if(robot.route_wait_elapsed>0.01f) navigation<<" / "<<std::fixed<<std::setprecision(1)<<robot.route_wait_elapsed<<" S";
        }else if(robot.traffic_detour_active){
            navigation<<"NAVIGATION  TRAFFIC DETOUR";
            if(robot.traffic_detour_avoid_index<builder_robots_.size()) navigation<<" AROUND "<<robot_identity(robot.traffic_detour_avoid_index);
        }else if(robot.route_force_full_replan){
            navigation<<"NAVIGATION  LOCAL FALLBACK REQUEST";
        }else if(robot.route_active){
            navigation<<"NAVIGATION  0.5S ROLLING / "<<std::fixed<<std::setprecision(1)<<remaining_m<<" M HORIZON";
        }else if(robot.action!=BuilderRobotAction::None){
            navigation<<"NAVIGATION  STOPPED AT WORK POSITION";
        }else{
            navigation<<"NAVIGATION  STATIONARY";
        }

        motion<<"MOTION  "<<std::fixed<<std::setprecision(2)<<robot.current_speed_mps<<" / "<<robot.move_speed_mps<<" M/S";
        if(robot.route_active && robot.route_speed_scale<0.995f) motion<<" / GOV "<<(int)std::lround(std::clamp(robot.route_speed_scale,0.0f,1.0f)*100.0f)<<"%";

        if(robot.target.valid()){
            target<<"TARGET  "<<target_name;
            const auto step=construction_target_step(robot.target);
            const auto total=construction_target_total_steps(robot.target);
            const float fraction=target_progress(robot.target);
            build<<"BUILD  STEP "<<(int)step<<"/"<<(int)total<<" / "<<(int)std::lround(fraction*100.0f)<<"%";
            if(construction_target_complete(robot.target)) build<<" / COMPLETE";
        }else if(robot.mode==BuilderRobotMode::DepotTransfer){
            target<<"TARGET  MAIN DEPOT / RESOURCE FIELD";
            build<<"BUILD  NO CONSTRUCTION TARGET";
        }else{
            target<<"TARGET  NONE";
            build<<"BUILD  NO CONSTRUCTION TARGET";
        }

        cargo<<"CARGO  "<<robot.cargo.total_amount()<<" / "<<robot.payload_capacity_kg<<" KG  ["<<compact_bundle(robot.cargo)<<"]";
        assigned<<"ASSIGNED  "<<robot.logistics_reservation.total_amount()<<" KG  ["<<compact_bundle(robot.logistics_reservation)<<"]";

        if(robot.support_robot_index<builder_robots_.size() && robot.support_robot_index!=robot_index)
            team<<"TEAM  COORDINATED BY "<<robot_identity(robot.support_robot_index);
        else if(robot.role==RobotRole::Constructor && robot.target.valid())
            team<<"TEAM  SITE COORDINATOR";
        else team<<"TEAM  INDEPENDENT";

        recovery<<"RECOVERY  BLOCKED "<<std::fixed<<std::setprecision(1)<<robot.blocked_elapsed<<" S"
                <<" / ATTEMPTS "<<(int)robot.recovery_attempts
                <<" / LOCAL RECOVERIES "<<robot.route_full_replan_count;

        const int condition_pct=(int)std::lround(std::clamp(robot.condition,0.0f,1.0f)*100.0f);
        const char* condition_grade=condition_pct>=90?"NOMINAL":(condition_pct>=70?"SERVICE DUE":"DEGRADED");
        std::ostringstream condition;
        condition<<"CONDITION  "<<condition_pct<<"% / "<<condition_grade;


        game_label(r,{x,p.y+10},title,0.66f,theme_,true);
        game_label(r,{x,p.y+34},state.str(),0.55f,theme_);
        game_label(r,{x,p.y+56},task.str(),0.46f,theme_);
        game_label(r,{x,p.y+78},operation.str(),0.45f,theme_,true);
        game_label(r,{x,p.y+100},navigation.str(),0.43f,theme_,true);
        game_label(r,{x,p.y+122},motion.str(),0.44f,theme_,true);
        game_label(r,{x,p.y+144},target.str(),0.44f,theme_);
        game_label(r,{x,p.y+166},build.str(),0.43f,theme_,true);
        game_label(r,{x,p.y+188},cargo.str(),0.44f,theme_);
        game_label(r,{x,p.y+210},assigned.str(),0.43f,theme_,true);
        game_label(r,{x,p.y+232},team.str(),0.42f,theme_,true);

        if(robot.target.valid() && !construction_target_complete(robot.target)){
            const ResourceBundle delivered=construction_site_buffer(robot.target)?*construction_site_buffer(robot.target):ResourceBundle{};
            const ResourceBundle reserved=construction_site_reserved(robot.target);
            const ResourceBundle left=construction_site_remaining(robot.target,true);
            std::ostringstream site;
            site<<"SITE  DELIVERED "<<delivered.total_amount()<<" KG / RESERVED "<<reserved.total_amount()<<" KG";
            game_label(r,{x,p.y+254},site.str(),0.42f,theme_);
            game_label(r,{x,p.y+276},"SITE LEFT  "+compact_bundle(left),0.42f,theme_,true);
        }else{
            game_label(r,{x,p.y+254},"SITE  NO ACTIVE CONSTRUCTION LEDGER",0.42f,theme_,true);
        }

        game_label(r,{x,p.y+300},recovery.str(),0.40f,theme_,true);
        game_label(r,{x,p.y+322},condition.str(),0.42f,theme_,true);
        game_label(r,{x,p.y+p.h-27.0f},"LMB SELECT OTHER / ROBOTS ARE NOT DELETABLE",0.42f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Habitat && selected_habitat_>=0 && (size_t)selected_habitat_<placed_habitats_.size()){
        const auto& h=placed_habitats_[(size_t)selected_habitat_];
        std::ostringstream pos,rot,conn,utilities;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<h.position.x<<" / "<<h.position.z;
        rot<<"ROTATION "<<(int)std::round(h.rotation_degrees)<<" DEG";
        conn<<"CORRIDORS "<<corridor_network_.habitat_connection_count((size_t)selected_habitat_);
        utilities<<"SUPPLY LINKS "<<(utility_network_.endpoint_connection_count(UtilityEndpointKind::Habitat,(size_t)selected_habitat_)+walkway_network_.endpoint_connection_count(WalkwayEndpointKind::Habitat,(size_t)selected_habitat_));
        game_label(r,{x,p.y+10},"STRUCTURE IDENTITY",0.50f,theme_,true);
        game_label(r,{x,p.y+34},habitat_instance_name(h),0.66f,theme_);
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
        game_label(r,{x,p.y+10},"STRUCTURE IDENTITY",0.50f,theme_,true);
        game_label(r,{x,p.y+34},resource_instance_name(item),0.66f,theme_);
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
