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

void GameApp::render_empty_game_phase(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    r.set_3d(camera_.view_projection(aspect));

    // Empty field reserved for future astronaut tests.
    r.draw_triangles_3d(playfield_.floor_triangles(),{0.055f,0.075f,0.088f,1});
    r.draw_lines_3d(playfield_.grid_lines(),{0.22f,0.43f,0.47f,1},1.0f);
    const float lane_half=(float)playfield_.size()*0.5f-geometry::kPlayfieldRobotPerimeterLane;
    const std::vector<Vec3> perimeter_lane{
        {-lane_half,0.018f,-lane_half},{ lane_half,0.018f,-lane_half},
        { lane_half,0.018f,-lane_half},{ lane_half,0.018f, lane_half},
        { lane_half,0.018f, lane_half},{-lane_half,0.018f, lane_half},
        {-lane_half,0.018f, lane_half},{-lane_half,0.018f,-lane_half}};
    r.draw_lines_3d(perimeter_lane,{0.72f,0.52f,0.18f,0.72f},1.35f);

    app_.set_cursor_visible(true);
    r.set_2d(ui_scale_);
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    r.draw_rect({0,0,logical_w,STATUS_BAR_H},{0.020f,0.030f,0.040f,0.96f});
    r.draw_rect({0,logical_h-BUILD_BAR_H,logical_w,BUILD_BAR_H},{0.020f,0.030f,0.040f,0.96f});

    game_label(r,{18.0f,12.0f},"STAGE 3 - TEST ASTRONAUTS",0.62f,theme_);
    game_label(r,{logical_w-172.0f,13.0f},"VERSION "+std::string(version::current),0.45f,theme_,true);
    game_label(r,{18.0f,logical_h-BUILD_BAR_H+17.0f},"ESC  GAME MENU",0.47f,theme_,true);

    const float panel_w=std::min(640.0f,logical_w-28.0f);
    const float panel_h=146.0f;
    Rect panel{(logical_w-panel_w)*0.5f,(logical_h-panel_h)*0.5f,panel_w,panel_h};
    ui::panel(r,panel,theme_);
    const std::string title="ASTRONAUT TEST FIELD";
    const std::string detail="RESERVED FOR FUTURE ASTRONAUT TESTS";
    const float tw=game_text_width(title,0.82f,theme_);
    game_label(r,{panel.x+(panel.w-tw)*0.5f,panel.y+34.0f},title,0.82f,theme_);
    game_label_fitted(r,{panel.x+34.0f,panel.y+79.0f,panel.w-68.0f,22.0f},detail,0.52f,theme_,true);

}

void GameApp::render_playfield(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    if(first_person_mode_==FirstPersonMode::Active) r.set_3d(first_person_.view_projection(aspect));
    else r.set_3d(lander_.in_flight()?camera_.campus_view_projection(aspect):camera_.view_projection(aspect));

    if(lander_.in_flight())r.draw_triangles_3d({{-900,-0.035f,-900},{900,-0.035f,-900},{900,-0.035f,900},
        {-900,-0.035f,-900},{900,-0.035f,900},{-900,-0.035f,900}},{0.055f,0.075f,0.088f,1});
    r.draw_triangles_3d(playfield_.floor_triangles(),{0.055f,0.075f,0.088f,1});
    r.draw_lines_3d(playfield_.grid_lines(),{0.22f,0.43f,0.47f,1},1.0f);
    // Mark the inner edge of the permanent rover perimeter lane. This is a
    // placement reserve only; robots remain free to use the entire band.
    const float lane_half=(float)playfield_.size()*0.5f-geometry::kPlayfieldRobotPerimeterLane;
    const std::vector<Vec3> perimeter_lane{
        {-lane_half,0.018f,-lane_half},{ lane_half,0.018f,-lane_half},
        { lane_half,0.018f,-lane_half},{ lane_half,0.018f, lane_half},
        { lane_half,0.018f, lane_half},{-lane_half,0.018f, lane_half},
        {-lane_half,0.018f, lane_half},{-lane_half,0.018f,-lane_half}};
    r.draw_lines_3d(perimeter_lane,{0.72f,0.52f,0.18f,0.72f},1.35f);
    if(first_person_mode_!=FirstPersonMode::Active)render_ore_deposits(r);
    render_ground_resources(r);
    render_corridors(r);
    render_utilities(r);
    render_walkways(r);
    render_habitats(r);
    render_resources(r);
    render_lunar_lander(r);
    render_robot_blockers(r);
    render_stash(r);
    render_trader(r);
    render_builder_robot(r);
    render_gate_areas(r);

    if(first_person_mode_==FirstPersonMode::Active){
        r.set_2d(ui_scale_);
        render_construction_status_frames(r);
    render_supply_indicators(r);
        render_status_bar(r);
        render_progress_tree(r);
        render_first_person_hud(r);
        return;
    }

    corridor_placement_failure_title_.clear();
    corridor_placement_failure_detail_.clear();
    render_corridor_link_preview(r);
    render_corridor_placement(r);
    render_habitat_placement(r);
    render_resource_placement(r);
    render_utility_placement(r);
    render_walkway_placement(r);
    render_playfield_crosshair(r);

    r.set_2d(ui_scale_);
    render_construction_status_frames(r);
    render_supply_indicators(r);
    render_status_bar(r);
    render_progress_tree(r);
    render_build_bar(r);
    render_ore_survey(r);
    render_selection_marquee(r);
    if(debug_mode_active()) render_debug_frame(r);
    if(selection_kind_!=SelectionKind::None) render_selection_panel(r);
    render_corridor_placement_tooltip(r);

    if(selection_kind_==SelectionKind::None){
    const BuildObjectDefinition* active_build=build_catalog_.selected();
    const char* controls=storage_gate_placement_active_?"STORAGE GATE / MOVE AROUND DOME / LMB PLACE EXTENSION / ESC CANCEL":
        corridor_link_source_>=0?"LMB TARGET CORRIDOR / ESC CANCEL":
        (active_build && active_build->kind==BuildObjectKind::Corridor?
            (corridor_build_start_?"LMB SET CORRIDOR END / ESC CANCEL":"LMB SET CORRIDOR START / ESC CANCEL"):
        (active_build && active_build->kind==BuildObjectKind::Utility?
            (active_build->utility_kind==UtilityKind::ConveyorBelt?
                (utility_build_start_?"LMB SELECT PROCESSING FACILITY OR FACTORY / ESC CANCEL":"LMB SELECT ORE DRILLER / ESC CANCEL"):
                (utility_build_start_?"MOVE TARGET TO ADJUST PORT / LMB CONNECT / ESC CANCEL":"LMB SELECT SUPPLY SOURCE / ESC CANCEL")):
        (active_build && active_build->kind==BuildObjectKind::Walkway?
            (walkway_build_start_?"MOVE TO ADJUST WALKWAY BLUEPRINT / LMB BUILD / ESC CANCEL":"LMB SET WALKWAY START / ESC CANCEL"):
        (active_build && (active_build->kind==BuildObjectKind::Habitat || active_build->kind==BuildObjectKind::Resource)?
            "LMB PLACE / WHEEL SIZE / N-M ROTATE / ESC CANCEL":
        (active_build?"LMB PLACE / N-M ROTATE / ESC CANCEL":"LMB SELECT / DRAG SELECT RESOURCE BOXES / WHEEL ZOOM / MMB ORBIT / RMB PAN / V FPV / ESC MENU")))));
    const auto control_lines=ui::wrap_text(controls,406,ui::font_scale(0.52f*GAME_FONT_BOOST,theme_));
    Rect info={PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,430,34.0f+13.0f*(float)control_lines.size()};
    r.draw_rect(info,{0.025f,0.035f,0.045f,0.88f});
    game_label(r,{info.x+12,info.y+8},"PLAYFIELD "+std::to_string(playfield_.size())+" X "+std::to_string(playfield_.size()),0.78f,theme_);
    for(std::size_t i=0;i<control_lines.size();++i)game_label(r,{info.x+12,info.y+28+13*(float)i},control_lines[i],0.52f,theme_,true);

    Rect robot_info={info.x,info.y+info.h+6.0f,560,40};
    r.draw_rect(robot_info,{0.025f,0.035f,0.045f,0.88f});
    std::size_t working=0,constructors=0,carriers=0;
    std::int64_t fleet_cargo=0;
    std::int64_t fleet_capacity=0;
    for(const auto& robot:builder_robots_){
        if(robot.role==RobotRole::Carrier) ++carriers; else ++constructors;
        if(robot.mode!=BuilderRobotMode::Idle) ++working;
        fleet_cargo+=robot.cargo.total_amount();
        fleet_capacity+=robot.payload_capacity_kg;
    }
    std::ostringstream robot_line;
    robot_line<<"ROBOTS  BR-02 "<<constructors<<"  CR-01 "<<carriers<<"  ACTIVE "<<working
              <<"  CARGO "<<fleet_cargo<<"/"<<fleet_capacity<<" kg";
    for(const auto& robot:builder_robots_){
        if(robot.action!=BuilderRobotAction::None){
            robot_line<<"  |  "<<robot_role_name(robot.role)<<" "<<builder_robot_action_name(robot.action);
            break;
        }
    }
    game_label_fitted(r,{robot_info.x+12,robot_info.y+11,robot_info.w-24,16},robot_line.str(),0.47f,theme_);
    }
}

void GameApp::render_selection_marquee(Renderer& r){
    if(!selection_marquee_active_ || !selection_marquee_dragged_ || build_catalog_.selected()!=nullptr || corridor_link_source_>=0) return;
    const float x0=(float)selection_marquee_start_x_/ui_scale_;
    const float y0=(float)selection_marquee_start_y_/ui_scale_;
    const float x1=(float)input_.mouse_x/ui_scale_;
    const float y1=(float)input_.mouse_y/ui_scale_;
    Rect box{std::min(x0,x1),std::min(y0,y1),std::abs(x1-x0),std::abs(y1-y0)};
    const float top=STATUS_BAR_H;
    const float bottom=ui_height(r)-BUILD_BAR_H;
    if(box.y<top){ box.h-=top-box.y; box.y=top; }
    if(box.y+box.h>bottom) box.h=std::max(0.0f,bottom-box.y);
    if(box.w<=0.0f || box.h<=0.0f) return;
    r.draw_rect(box,{0.18f,0.58f,0.82f,0.12f});
    const Color c{0.32f,0.82f,1.00f,0.95f};
    r.draw_line({box.x,box.y},{box.x+box.w,box.y},c,1.6f);
    r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},c,1.6f);
    r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},c,1.6f);
    r.draw_line({box.x,box.y+box.h},{box.x,box.y},c,1.6f);
}

void GameApp::render_first_person_hud(Renderer& r){
    if(first_person_mode_==FirstPersonMode::Active){
        const float cx=ui_width(r)*0.5f,cy=ui_height(r)*0.5f;
        const Color cross{0.82f,0.94f,0.96f,0.90f};
        r.draw_line({cx-8.0f,cy},{cx-2.0f,cy},cross,1.5f);
        r.draw_line({cx+2.0f,cy},{cx+8.0f,cy},cross,1.5f);
        r.draw_line({cx,cy-8.0f},{cx,cy-2.0f},cross,1.5f);
        r.draw_line({cx,cy+2.0f},{cx,cy+8.0f},cross,1.5f);

        Rect help{PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,580.0f,54.0f};
        r.draw_rect(help,{0.025f,0.035f,0.045f,0.86f});
        game_label(r,{help.x+12.0f,help.y+8.0f},"FIRST PERSON TEST VIEW",0.64f,theme_);
        game_label(r,{help.x+12.0f,help.y+30.0f},"WASD WALK / SHIFT FAST / SPACE JUMP / LMB GRAB-DROP / MOUSE LOOK / V OR ESC EXIT",0.43f,theme_,true);

        const std::string environment=first_person_environment_label();
        Rect env{help.x,help.y+help.h+8.0f,580.0f,34.0f};
        const bool exterior=environment.find("EXTERIOR")!=std::string::npos;
        r.draw_rect(env,exterior?Color{0.20f,0.055f,0.045f,0.92f}:Color{0.035f,0.095f,0.085f,0.92f});
        game_label_fitted(r,{env.x+10,env.y+8,env.w-20,20},environment,0.48f,theme_);
        const auto feet=first_person_.feet_position();
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            const auto& h=placed_habitats_[i];if(h.kind!=HabitatKind::AirLock || !h.construction.complete())continue;
            const auto& mesh=habitat_models_.mesh(h);const auto local=rotate_y(feet-h.position,-h.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+0.2f || std::abs(local.z)>mesh.footprint_z*0.5f)continue;
            const airlock::Cycle idle{};const auto& cycle=i<airlock_cycles_.size()?airlock_cycles_[i]:idle;
            Rect status{env.x,env.y+env.h,env.w,44};r.draw_rect(status,{0.035f,0.075f,0.085f,0.92f});
            std::ostringstream text;text<<"CENTER "<<std::fixed<<std::setprecision(0)<<cycle.pressure*100<<"% OF INSIDE / "<<airlock::phase(cycle);
            game_label_fitted(r,{status.x+10,status.y+6,status.w-20,16},text.str(),0.46f,theme_);
            game_label_fitted(r,{status.x+10,status.y+25,status.w-20,13},"APPROACH NEXT DOOR / WAIT FOR PRESSURE / WALK EITHER DIRECTION",0.39f,theme_,true);
            env.h+=status.h;break;
        }

        if(first_person_carried_resource_){
            const auto& carried=*first_person_carried_resource_;
            std::ostringstream line;
            line<<"CARRYING  "<<material_resource_name(carried.resource)<<"  "<<carried.amount<<" kg";
            Rect carry{PLAYFIELD_MARGIN,env.y+env.h+8.0f,300.0f,44.0f};
            r.draw_rect(carry,{0.035f,0.055f,0.065f,0.92f});
            game_label(r,{carry.x+10.0f,carry.y+8.0f},line.str(),0.54f,theme_);
            game_label(r,{carry.x+10.0f,carry.y+26.0f},"LMB DROP  |  DROP INSIDE DOME TO STORE",0.39f,theme_,true);
        }

        MaterialResource hover_resource=MaterialResource::Iron;
        std::int64_t hover_amount=0;
        float hover_condition=1.0f;
        const char* hover_source=nullptr;
        if(first_person_hover_ground_resource_>=0 && (std::size_t)first_person_hover_ground_resource_<ground_resource_piles_.size()){
            const auto& box=ground_resource_piles_[(std::size_t)first_person_hover_ground_resource_];
            hover_resource=box.resource;
            hover_amount=box.amount;
            hover_condition=box.condition;
            hover_source="GROUND PACKAGE";
        }else if(first_person_hover_storage_resource_>=0 && first_person_hover_storage_box_>=0 &&
                 (std::size_t)first_person_hover_storage_resource_<placed_resources_.size()){
            const auto& dome=placed_resources_[(std::size_t)first_person_hover_storage_resource_];
            if((std::size_t)first_person_hover_storage_box_<dome.stored_boxes.size()){
                const auto& box=dome.stored_boxes[(std::size_t)first_person_hover_storage_box_];
                hover_resource=box.resource;
                hover_amount=box.amount;
                hover_condition=box.condition;
                hover_source="STORAGE PACKAGE";
            }
        }
        if(hover_source){
            const float panel_w=245.0f,panel_h=86.0f;
            Rect hover{std::min(cx+18.0f,ui_width(r)-panel_w-12.0f),
                       std::min(cy+18.0f,ui_height(r)-panel_h-12.0f),panel_w,panel_h};
            r.draw_rect(hover,{0.025f,0.040f,0.050f,0.95f});
            std::ostringstream amount;
            amount<<hover_amount<<" / "<<resource_package_capacity_kg(hover_resource)<<" kg CARRY";
            std::ostringstream condition;
            condition<<"CONDITION  "<<(int)std::lround(std::clamp(hover_condition,0.0f,1.0f)*100.0f)<<"%";
            game_label(r,{hover.x+10.0f,hover.y+8.0f},material_resource_name(hover_resource),0.58f,theme_);
            game_label(r,{hover.x+10.0f,hover.y+29.0f},amount.str(),0.46f,theme_);
            game_label(r,{hover.x+10.0f,hover.y+47.0f},condition.str(),0.43f,theme_,true);
            game_label(r,{hover.x+10.0f,hover.y+65.0f},std::string(hover_source)+"  |  LMB GRAB",0.39f,theme_,true);
        }
        return;
    }
}

void GameApp::render_playfield_crosshair(Renderer& r){
    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px || mouse_over_selection_panel(input_)){
        app_.set_cursor_visible(true);
        return;
    }

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,r.width(),r.height(),playfield_.size(),hit)){
        app_.set_cursor_visible(true);
        return;
    }

    app_.set_cursor_visible(false);
    const float arm=std::clamp(camera_.distance()*0.030f,1.35f,5.0f)*UI_SCALES[ui_scale_index_]/UI_SCALES[2];
    const float gap=arm*0.24f;
    const float y=0.055f;
    std::vector<Vec3> lines={
        {hit.x-arm,y,hit.z},{hit.x-gap,y,hit.z},
        {hit.x+gap,y,hit.z},{hit.x+arm,y,hit.z},
        {hit.x,y,hit.z-arm},{hit.x,y,hit.z-gap},
        {hit.x,y,hit.z+gap},{hit.x,y,hit.z+arm}
    };
    r.draw_lines_3d(lines,theme_.accent,2.8f*ui_scale_/UI_SCALES[2]);
}

} // namespace game
