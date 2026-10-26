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

void GameApp::render_menu(Renderer& r){
    ui::title(r,{32,26},game_session_active_?"GAME MENU":"GAME",theme_);
    ui::label(r,{34,67},game_session_active_?"CURRENT SESSION PAUSED":"C++ PLAYFIELD PROTOTYPE",1.0f,theme_,true);

    constexpr float button_w=300.0f;
    constexpr float button_h=42.0f;
    float y=112.0f;
    if(game_session_active_){
        if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"RESUME GAME",theme_)) screen_=Screen::Playfield;
        y+=button_h+CONTROL_GAP;
    }
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"START NEW GAME",theme_)) screen_=Screen::NewGame;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"SETTINGS",theme_)){ ensure_display_modes(); screen_=Screen::Settings; }
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"CREDITS",theme_)) screen_=Screen::Credits;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"EXIT",theme_)) app_.request_quit();

}

} // namespace game
