#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <array>
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
// Shared menu backdrop: layered silhouettes rather than a static plain-color
// screen. Coordinates are logical UI units and scale with the viewport.
void draw_new_game_backdrop(Renderer& r,float w,float h){
    r.draw_rect({0,0,w,h},{0.012f,0.025f,0.049f,1.0f});
    for(int band=0;band<18;++band){
        const float y=h*(float)band/18.0f;
        r.draw_rect({0,y,w,std::min(h/18.0f+1.0f,h-y)},{0.012f+0.0017f*band,0.025f+0.0023f*band,0.049f+0.0021f*band,1.0f});
    }
    for(int i=0;i<76;++i){
        const unsigned k=(unsigned)(i*104729+8191);
        const float x=(float)((k*137u+29u)%1009u)/1009.0f*w;
        const float y=(float)((k*73u+61u)%991u)/991.0f*h*0.76f;
        const float a=0.18f+0.3f*(float)(i%4)/4.0f;
        r.draw_rect({x,y,i%7==0?2.0f:1.0f,i%7==0?2.0f:1.0f},{0.58f,0.78f,0.86f,a});
    }
    const float hill=h*0.83f;
    r.draw_rect({0,hill,w,h-hill},{0.022f,0.045f,0.059f,1.0f});
    for(int i=0;i<28;++i){
        float x=w*(float)i/27.0f;
        float ht=9.0f+16.0f*std::abs(std::sin(i*1.73f));
        r.draw_rect({x,hill-ht,std::min(w/27.0f+1,w-x),ht+4},{0.027f,0.060f,0.075f,1});
    }
    const float rocket_x=w-std::min(w*0.11f,125.0f);
    if(rocket_x>w*0.70f){
        r.draw_rect({rocket_x-5,h*0.38f,10,h*0.33f},{0.21f,0.35f,0.43f,0.42f});
        r.draw_rect({rocket_x-2,h*0.29f,4,h*0.10f},{0.37f,0.60f,0.68f,0.40f});
        r.draw_line({rocket_x-2,h*0.29f},{rocket_x,h*0.23f},{0.36f,0.60f,0.69f,0.55f},2);
        r.draw_line({rocket_x,h*0.23f},{rocket_x+2,h*0.29f},{0.36f,0.60f,0.69f,0.55f},2);
        r.draw_line({rocket_x-5,h*0.66f},{rocket_x-16,h*0.70f},{0.22f,0.42f,0.48f,0.5f},2);
        r.draw_line({rocket_x+5,h*0.66f},{rocket_x+16,h*0.70f},{0.22f,0.42f,0.48f,0.5f},2);
    }
}
}

void GameApp::render_new_game(Renderer& r){
    const float screen_w=ui_width(r), screen_h=ui_height(r);
    draw_new_game_backdrop(r,screen_w,screen_h);
    const bool compact=screen_h<680.0f;
    const bool very_compact=screen_h<475.0f;
    const float panel_w=std::min(920.0f,screen_w-24.0f);
    const float panel_y=very_compact?38.0f:compact?48.0f:65.0f;
    const float panel_h=std::max(205.0f,std::min(very_compact?400.0f:compact?575.0f:685.0f,screen_h-panel_y-18.0f));
    Rect p{(screen_w-panel_w)*0.5f,panel_y,panel_w,panel_h};
    r.draw_rect({p.x-2,p.y-2,p.w+4,p.h+4},{0.16f,0.29f,0.33f,0.92f});
    r.draw_rect(p,{0.023f,0.039f,0.056f,0.96f});
    r.draw_rect({p.x, p.y, p.w,very_compact?30.0f:43.0f},{0.044f,0.088f,0.113f,1});
    r.draw_rect({p.x,p.y+ (very_compact?29.0f:42.0f),p.w,2.0f},{0.36f,0.68f,0.72f,1});
    const float inset=very_compact?11.0f:22.0f;
    const float left=p.x+inset,right=p.x+p.w-inset,content_w=p.w-2*inset;
    ui::label(r,{left,p.y+(very_compact?8.0f:11.0f)},very_compact?"NEW GAME":"NEW GAME / MISSION DIRECTOR",very_compact?0.64f:0.82f,theme_);
    ui::label(r,{screen_w-std::min(180.0f,screen_w*0.25f),very_compact?8.0f:18.0f},"LUNAR FRONTIER  /  "+std::string(version::current),0.37f,theme_,true);
    const float row1=p.y+(very_compact?40.0f:58.0f);
    ui::label(r,{left,row1},"CHOOSE GAME MODE",very_compact?0.48f:0.68f,theme_);
    const float gap=10.0f;
    const float mode_w=(content_w-gap)*0.5f;
    const float mode_y=row1+(very_compact?15.0f:24.0f);
    const float mode_h=very_compact?29.0f:compact?37.0f:49.0f;
    auto mode_button=[&](NewGameMode mode,float x,const char* caption){
        std::string label=(new_game_mode_selection_==mode?"[X] ":"[ ] ");label+=caption;
        if(ui::button(r,ui_input_,{x,mode_y,mode_w,mode_h},label,theme_)){
            new_game_mode_selection_=mode;
            new_game_status_message_.clear();
        }
    };
    mode_button(NewGameMode::Career,left,"CAREER / CAMPAIGN");
    mode_button(NewGameMode::FreePlay,left+mode_w+gap,"FREE PLAY");
    const bool career=new_game_mode_selection_==NewGameMode::Career;
    if(career) new_game_phase_selection_=GamePhase::Planning;
    else if(!build::debug_features && new_game_phase_selection_==GamePhase::Debug)
        new_game_phase_selection_=GamePhase::RobotPreparation;
    const float expl_y=mode_y+mode_h+(very_compact?4.0f:11.0f);
    ui::label_fitted(r,{left,expl_y,content_w,14.0f},
        career?"BEGIN IN STAGE 1. FUND, RESEARCH, PREPARE AND LAUNCH YOUR LUNAR PROGRAM.":
               build::debug_features?"PLAY A STAGE INDEPENDENTLY. STAGE 3 REQUIRES THE DEBUG FRAME SETTING.":
                                     "PLAY THE PLANNING OR ROBOT PREPARATION STAGE INDEPENDENTLY.",
        very_compact?0.34f:0.49f,theme_,true);
    const float stage_label_y=expl_y+(very_compact?15.0f:25.0f);
    ui::label(r,{left,stage_label_y},career?"CAMPAIGN START":"SELECT FREE-PLAY STAGE",very_compact?0.43f:0.63f,theme_);
    const float stage_y=stage_label_y+(very_compact?14.0f:23.0f);
    constexpr int stage_count=build::debug_features?3:2;
    const float stage_w=(content_w-(stage_count-1)*gap)/stage_count;
    const float stage_h=very_compact?27.0f:compact?35.0f:45.0f;
    const std::array<const char*,3> stage_names{{"1  PLANNING","2  ROBOT PREPARATION",
                                                 build::debug_features?"3  DEBUG MODES":""}};
    for(int index=0;index<stage_count;++index){
        const auto phase=(GamePhase)(index+1);
        const bool selected=new_game_phase_selection_==phase;
        const std::string label=(selected?"> ":"")+std::string(stage_names[(std::size_t)index])+(selected?" <":"");
        const float x=left+(stage_w+gap)*(float)index;
        if(ui::button(r,ui_input_,{x,stage_y,stage_w,stage_h},label,theme_,!career && (phase!=GamePhase::Debug || debug_frame_enabled_))){
            new_game_phase_selection_=phase;
            new_game_status_message_.clear();
            if(phase!=GamePhase::RobotPreparation) debug_start_template_enabled_=false;
            if(phase==GamePhase::Planning) playfield_size_=kCompanySitePlayfieldSize;
        }
    }
    const bool robot_stage=new_game_phase_selection_==GamePhase::RobotPreparation;
    const bool debug_stage=build::debug_features && new_game_phase_selection_==GamePhase::Debug;
    const bool planning_stage=new_game_phase_selection_==GamePhase::Planning;
    const float details_y=stage_y+stage_h+(very_compact?9.0f:compact?17.0f:27.0f);
    const float actions_h=very_compact?28.0f:compact?34.0f:43.0f;
    const float actions_y=p.y+p.h-actions_h-(very_compact?9.0f:16.0f);
    const float available_h=actions_y-details_y-(very_compact?7.0f:16.0f);
    r.draw_rect({left,details_y,content_w,std::max(1.0f,available_h)}, {0.031f,0.054f,0.070f,0.92f});
    const float detail_left=left+(very_compact?9.0f:17.0f);
    const float detail_w=content_w-2*(very_compact?9.0f:17.0f);
    const float info_scale=very_compact?0.41f:compact?0.53f:0.70f;
    ui::label_fitted(r,{detail_left,details_y+8,detail_w,20},
        planning_stage?"STAGE 1  /  SPACE COMPANY LAUNCH CENTER":robot_stage?"STAGE 2  /  ROBOTIC LUNAR DEPLOYMENT":
        build::debug_features?"STAGE 3  /  DEBUG MODES":"",info_scale,theme_);
    const float sub_y=details_y+(very_compact?27.0f:39.0f);
    if(planning_stage){
        ui::label_fitted(r,{detail_left,sub_y,detail_w,16},"FIXED COASTAL CAMPUS  /  RESEARCH  /  CONTRACTS  /  LAUNCH MISSIONS",very_compact?0.31f:0.48f,theme_,true);
        if(!very_compact && available_h>100)ui::label_fitted(r,{detail_left,sub_y+30,detail_w,16},"3D SITE CAMERA: MOUSE WHEEL ZOOM  |  MIDDLE MOUSE ORBIT  |  RIGHT MOUSE PAN",0.44f,theme_,true);
        if(!compact && available_h>140)ui::label_fitted(r,{detail_left,sub_y+72,detail_w,16},"PROGRESS THROUGH MISSION FACILITIES, TECH RESEARCH, CONTRACT FUNDING AND VEHICLE ASSEMBLY",0.43f,theme_,true);
    }else if(debug_stage){
        ui::label_fitted(r,{detail_left,sub_y,detail_w,16},"CHOOSE TEMPLATE MODE, SANDBOX MODE OR TEST ASTRONAUTS",very_compact?0.32f:0.52f,theme_,true);
        if(!very_compact && available_h>100)ui::label_fitted(r,{detail_left,sub_y+30,detail_w,16},"CONTINUE TO THE STAGE 3 MODE SELECTOR",0.45f,theme_,true);
    }else{
        // Stage 2 is the only phase with a configurable playfield size.
        const float robot_line=very_compact?23.0f:compact?29.0f:40.0f;
        const float row_y=sub_y;
        const bool fixed_size=build::debug_features && debug_start_template_enabled_;
        ui::label_fitted(r,{detail_left,row_y,detail_w*0.30f,16},"PLAYFIELD",very_compact?0.32f:0.48f,theme_);
        const float size_x=detail_left+detail_w*0.32f;
        if(ui::button(r,ui_input_,{size_x,row_y-3,28,robot_line},"<",theme_,!fixed_size)) playfield_size_=std::max(kNewGamePlayfieldMin,playfield_size_-kNewGamePlayfieldStep);
        ui::label_fitted(r,{size_x+33,row_y+2,std::max(40.0f,detail_w*0.18f),18},std::to_string(playfield_size_)+" X "+std::to_string(playfield_size_),very_compact?0.38f:0.52f,theme_);
        if(ui::button(r,ui_input_,{size_x+std::max(103.0f,detail_w*0.20f),row_y-3,28,robot_line},">",theme_,!fixed_size)) playfield_size_=std::min(kNewGamePlayfieldMax,playfield_size_+kNewGamePlayfieldStep);
        if(available_h>65){
            const float count_y=row_y+robot_line+(very_compact?4.0f:12.0f);
            const float col_w=(detail_w-10.0f)*0.5f;
            auto count=[&](float x,int& n,const char* caption){
                ui::label_fitted(r,{x,count_y,col_w,15},caption,very_compact?0.28f:0.45f,theme_);
                const float button_y=count_y+(very_compact?12.0f:18.0f);
                if(ui::button(r,ui_input_,{x,button_y,27,very_compact?22.0f:28.0f},"<",theme_)) n=std::max(1,n-1);
                ui::label(r,{x+36,button_y+6},std::to_string(n),very_compact?0.48f:0.65f,theme_);
                if(ui::button(r,ui_input_,{x+65,button_y,27,very_compact?22.0f:28.0f},">",theme_))n=std::min(6,n+1);
            };
            count(detail_left,starting_robot_count_,"BUILDER BOTS");
            count(detail_left+col_w+10,starting_carrier_count_,"CARRIER BOTS");
        }
        if(build::debug_features && available_h>150){
            const float debug_y=details_y+available_h-45.0f;
            ui::label_fitted(r,{detail_left,debug_y,detail_w-95,16},"DEBUG START TEMPLATE",0.49f,theme_);
            if(ui::button(r,ui_input_,{detail_left+detail_w-74,debug_y-6,70,29},debug_start_template_enabled_?"ON":"OFF",theme_)){
                debug_start_template_enabled_=!debug_start_template_enabled_;
                if(debug_start_template_enabled_)playfield_size_=kDebugTemplatePlayfieldSize;
            }
        }
    }
    if(!new_game_status_message_.empty() && available_h>100) ui::label_fitted(r,{detail_left,actions_y-20,detail_w,16},new_game_status_message_,0.42f,theme_,true);
    const float action_w=very_compact?100.0f:compact?124.0f:170.0f;
    if(ui::button(r,ui_input_,{left,actions_y,action_w,actions_h},"BACK",theme_)){
        screen_=Screen::MainMenu;
        new_game_status_message_.clear();
    }
    const char* action=career?"START CAREER":debug_stage?"CONTINUE":"CREATE";
    if(ui::button(r,ui_input_,{right-action_w,actions_y,action_w,actions_h},action,theme_,
        !debug_stage || debug_frame_enabled_)){
        if(debug_stage){
            new_game_status_message_.clear();
            screen_=Screen::Stage3Modes;
        }else start_new_game();
    }
}

void GameApp::render_stage3_modes(Renderer& r){
    if constexpr(!build::debug_features){
        screen_=Screen::NewGame;
        return;
    }
    const float w=ui_width(r),h=ui_height(r);
    draw_new_game_backdrop(r,w,h);
    const float panel_w=std::min(660.0f,w-24.0f);
    const float panel_h=std::min(430.0f,h-24.0f);
    const Rect panel{(w-panel_w)*.5f,(h-panel_h)*.5f,panel_w,panel_h};
    ui::panel(r,panel,theme_);
    const float left=panel.x+18,content_w=panel.w-36;
    ui::label_fitted(r,{left,panel.y+14,content_w,25},"STAGE 3 / SELECT DEBUG MODE",.81f,theme_);
    const float row_h=(panel_h-115)/3;
    const float button_h=std::min(40.0f,row_h-19);
    const auto option=[&](int row,const char* title,const char* description,bool enabled){
        const float y=panel.y+49+row_h*(float)row;
        const bool chosen=ui::button(r,ui_input_,{left,y,content_w,button_h},title,theme_,enabled);
        ui::label_fitted(r,{left+3,y+button_h+3,content_w-6,14},description,.48f,theme_,true);
        return chosen;
    };
    if(option(0,"TEMPLATE MODE","PREBUILT BASE AND ROVERS / NORMAL CONSTRUCTION",debug_frame_enabled_)){
        start_new_game(Stage3Mode::Template);
        if(screen_==Screen::Playfield)return;
    }
    if(option(1,"SANDBOX MODE","EMPTY 128 X 128 FIELD / INSTANT BUILD",debug_frame_enabled_)){
        start_new_game(Stage3Mode::Sandbox);
        return;
    }
    if(option(2,"TEST ASTRONAUTS","FOUR CREW ROLES / QUEUED CONSTRUCTION / POWERED AIR LOCKS",debug_frame_enabled_)){
        start_new_game(Stage3Mode::AstronautTest);
        return;
    }
    ui::label_fitted(r,{left,panel.y+panel.h-60,content_w,15},new_game_status_message_,.49f,theme_);
    if(ui::button(r,ui_input_,{left,panel.y+panel.h-39,114,29},"BACK",theme_)){
        new_game_status_message_.clear();
        screen_=Screen::NewGame;
    }
}

void GameApp::start_new_game(Stage3Mode stage3_mode){
    if(!build::debug_features && new_game_phase_selection_==GamePhase::Debug) return;
    const bool robot_stage=new_game_phase_selection_==GamePhase::RobotPreparation;
    const bool debug_stage=build::debug_features && new_game_phase_selection_==GamePhase::Debug;
    const bool start_template=build::debug_features && robot_stage && debug_start_template_enabled_;
    if(robot_stage && !start_template && playfield_size_<64){
        new_game_status_message_="CHOOSE AT LEAST 64 X 64 FOR THE LANDER AND CARGO STACKS";
        return;
    }
    const bool completed_template=debug_stage && stage3_mode==Stage3Mode::Template;
    const bool use_template=completed_template || start_template;
    if(use_template){
        std::string error;
        if(!load_debug_blueprint_template_from_file(debug_blueprint_template_path(),error,completed_template)){
            new_game_status_message_="DEBUG TEMPLATE ERROR: "+error;
            return;
        }
        playfield_size_=kDebugTemplatePlayfieldSize;
    }else if(debug_stage){
        playfield_size_=kNewGamePlayfieldDefault;
    }else if(new_game_phase_selection_==GamePhase::Planning){
        playfield_size_=kCompanySitePlayfieldSize;
    }
    playfield_.rebuild(playfield_size_);
    camera_.reset(playfield_size_);
    first_person_mode_=FirstPersonMode::Off;
    app_.set_relative_mouse_mode(false);
    status_.reset();
    status_rate_display_={};
    battery_net_kw_=water_net_litres_per_day_=0;
    supply_power_delivered_kw_=supply_power_load_kw_=supply_water_load_litres_per_day_=0;
    resources_.reset();
    ground_resource_piles_.clear();
    builder_robots_.clear();
    production_fraction_.fill(0.0);
    life_support_fraction_seconds_=0.0;
    reset_planning_game();
    lander_={};trader_={};trade_message_.clear();
    colony_seeds_=use_template || new_game_phase_selection_==GamePhase::Debug?kStarterCropSeeds:CropSeeds{};
    if(new_game_phase_selection_==GamePhase::Planning){
        // Stage 1 opens on the company campus at a useful oblique overview.
        // The site remains freely orbitable/pannable/zoomable afterwards.
        camera_.restore({134.0f,47.0f,38.0f,{0.0f,0.0f,4.0f}});
    }
    starter_stash_center_={};
    construction_priority_.clear();
    construction_order_.clear();
    build_queue_expanded_=false;progress_tree_expanded_=false;build_queue_scroll_=0;
    debug_move_pick_robot_=-1;
    first_person_carried_resource_.reset();
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    pressure_doors_.clear();airlock_cycles_.clear();storage_gate_open_.clear();
    corridor_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
    utility_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);

    if(!use_template){
        placed_habitats_.clear();
        placed_resources_.clear();
        corridor_network_.clear();
        utility_network_.clear();
        walkway_network_.clear();
        next_blueprint_placement_sequence_=1;
    }

    if(use_template){
        reset_builder_robot_and_starter_stash();
    }else if(robot_stage){
        begin_lunar_landing(starter_resource_reserve(),starting_robot_count_,starting_carrier_count_);
    }else{
        debug_start_template_enabled_=false;
    }

    build_catalog_.clear_selection();
    placement_rotation_degrees_=0.0f;
    placement_habitat_size_=HabitatSizeTier::Small;
    placement_resource_size_=ResourceSizeTier::Small;
    placement_storage_size_=StorageDomeSize::Small;
    storage_gate_placement_active_=false;
    storage_gate_placement_owner_=kNoRobotIndex;
    habitat_view_mode_=HabitatViewMode::Default;
    game_speed_=start_template?GameSpeed::Pause:GameSpeed::Normal;
    last_running_speed_=1;
    clear_world_selection();
    if(lander_.active())selection_kind_=SelectionKind::Lander;
    selection_marquee_active_=false;
    selection_marquee_dragged_=false;
    corridor_link_source_=-1;
    corridor_build_start_.reset();
    utility_build_start_.reset();
    walkway_build_start_.reset();
    walkway_build_target_.reset();
    active_game_phase_=new_game_phase_selection_;
    stage3_mode_=debug_stage?stage3_mode:Stage3Mode::Sandbox;
    if(astronaut_test_active())reset_astronaut_test();
    active_game_mode_=new_game_mode_selection_;
    new_game_status_message_.clear();
    game_session_active_=true;
    screen_=Screen::Playfield;
    reset_activity_log("New game");
    update_activity_log();
}

} // namespace game
