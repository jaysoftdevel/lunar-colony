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
    Rect p={(ui_width(r)-w)*0.5f,82,w,250};
    ui::panel(r,p,theme_);

    const float left=p.x+PANEL_PADDING;
    const float right=p.x+p.w-PANEL_PADDING;
    ui::label(r,{left,p.y+22},"PLAYFIELD SIZE",1.15f,theme_);

    std::ostringstream ss;
    ss<<playfield_size_<<" X "<<playfield_size_;
    const std::string size_text=ss.str();
    const float size_scale=1.65f;
    const float size_w=text_width(size_text,size_scale,theme_);
    ui::label(r,{p.x+(p.w-size_w)*0.5f,p.y+78},size_text,size_scale,theme_);

    if(ui::button(r,ui_input_,{left,p.y+70,58,42},"<",theme_)) playfield_size_=std::max(16,playfield_size_-16);
    if(ui::button(r,ui_input_,{right-58,p.y+70,58,42},">",theme_)) playfield_size_=std::min(512,playfield_size_+16);

    ui::label(r,{left,p.y+139},"MESH ONLY - NO TEXTURES",0.82f,theme_,true);

    constexpr float action_w=160.0f;
    constexpr float action_h=42.0f;
    const float action_y=p.y+p.h-PANEL_PADDING-action_h;
    if(ui::button(r,ui_input_,{left,action_y,action_w,action_h},"BACK",theme_)) screen_=Screen::MainMenu;
    if(ui::button(r,ui_input_,{right-action_w,action_y,action_w,action_h},"CREATE",theme_)){
        playfield_.rebuild(playfield_size_);
        camera_.reset(playfield_size_);
        status_.reset();
        placed_habitats_.clear();
        corridor_network_.clear();
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        habitat_view_mode_=HabitatViewMode::Default;
        clear_world_selection();
        corridor_link_source_=-1;
        game_session_active_=true;
        screen_=Screen::Playfield;
    }
}

} // namespace game
