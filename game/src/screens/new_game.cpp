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

void GameApp::render_new_game(Renderer& r){
    ui::title(r,{32,26},"NEW GAME",theme_);
    const float w=std::min(560.0f,ui_width(r)-2.0f*SCREEN_MARGIN);
    Rect p={(ui_width(r)-w)*0.5f,66,w,430};
    ui::panel(r,p,theme_);

    const float left=p.x+PANEL_PADDING;
    const float right=p.x+p.w-PANEL_PADDING;
    ui::label(r,{left,p.y+20},"PLAYFIELD SIZE",1.15f,theme_);

    std::ostringstream ss;
    ss<<playfield_size_<<" X "<<playfield_size_;
    const std::string size_text=ss.str();
    const float size_scale=1.65f;
    const float size_w=text_width(size_text,size_scale,theme_);
    ui::label(r,{p.x+(p.w-size_w)*0.5f,p.y+68},size_text,size_scale,theme_);
    if(ui::button(r,ui_input_,{left,p.y+60,58,42},"<",theme_)) playfield_size_=std::max(16,playfield_size_-16);
    if(ui::button(r,ui_input_,{right-58,p.y+60,58,42},">",theme_)) playfield_size_=std::min(512,playfield_size_+16);

    auto count_selector=[&](float y,const char* title,int& value,const char* suffix){
        ui::label(r,{left,y},title,1.00f,theme_);
        const std::string text=std::to_string(value)+(value==1?" ROBOT":" ROBOTS");
        const float scale=1.28f;
        const float tw=text_width(text,scale,theme_);
        ui::label(r,{p.x+(p.w-tw)*0.5f,y+40},text,scale,theme_);
        if(ui::button(r,ui_input_,{left,y+29,58,42},"<",theme_)) value=std::max(1,value-1);
        if(ui::button(r,ui_input_,{right-58,y+29,58,42},">",theme_)) value=std::min(6,value+1);
        ui::label(r,{left,y+80},suffix,0.64f,theme_,true);
    };

    count_selector(p.y+122,"BR-02 BUILDER BOTS SHIPPED",starting_robot_count_,"PRIMARY: CONSTRUCTION   FALLBACK: RESOURCE HAULING");
    count_selector(p.y+230,"CR-01 CARRIER BOTS SHIPPED",starting_carrier_count_,"PRIMARY: RESOURCE PILE / DEPOT -> CONSTRUCTION SITE");

    ui::label(r,{left,p.y+340},"ROBOT ROUTES ARE PRE-PLANNED AND RESERVED BEFORE TRAVEL",0.61f,theme_,true);

    constexpr float action_w=160.0f;
    constexpr float action_h=42.0f;
    const float action_y=p.y+p.h-PANEL_PADDING-action_h;
    if(ui::button(r,ui_input_,{left,action_y,action_w,action_h},"BACK",theme_)) screen_=Screen::MainMenu;
    if(ui::button(r,ui_input_,{right-action_w,action_y,action_w,action_h},"CREATE",theme_)){
        playfield_.rebuild(playfield_size_);
        camera_.reset(playfield_size_);
        first_person_mode_=FirstPersonMode::Off;
        app_.set_relative_mouse_mode(false);
        status_.reset();
        resources_.reset();
        placed_habitats_.clear();
        placed_resources_.clear();
        ground_resource_piles_.clear();
        first_person_carried_resource_.reset();
        first_person_hover_ground_resource_=-1;
        first_person_hover_storage_resource_=-1;
        first_person_hover_storage_box_=-1;
        corridor_network_.clear();
        corridor_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
        utility_network_.clear();
        utility_network_.set_playfield_half_extent((float)playfield_.size()*0.5f);
        walkway_network_.clear();
        reset_builder_robot_and_starter_stash();
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_habitat_size_=HabitatSizeTier::Small;
        placement_resource_size_=ResourceSizeTier::Small;
        placement_storage_size_=StorageDomeSize::Small;
        habitat_view_mode_=HabitatViewMode::Default;
        game_speed_=GameSpeed::Normal;
        clear_world_selection();
        selection_marquee_active_=false;
        selection_marquee_dragged_=false;
        corridor_link_source_=-1;
        corridor_build_start_.reset();
        utility_build_start_.reset();
        walkway_build_start_.reset();
        game_session_active_=true;
        screen_=Screen::Playfield;
    }
}

} // namespace game
