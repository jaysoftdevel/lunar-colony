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
        if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"SAVE GAME",theme_)){ save_status_message_.clear(); screen_=Screen::SaveGame; }
        y+=button_h+CONTROL_GAP;
    }
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"START NEW GAME",theme_)) screen_=Screen::NewGame;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"LOAD GAME",theme_)){ save_status_message_.clear(); refresh_save_games(); screen_=Screen::LoadGame; }
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"SETTINGS",theme_)){ ensure_display_modes(); screen_=Screen::Settings; }
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"CREDITS",theme_)) screen_=Screen::Credits;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"EXIT",theme_)) app_.request_quit();
    if(!save_status_message_.empty()) ui::label(r,{34.0f,ui_height(r)-34.0f},save_status_message_,0.72f,theme_,true);

}

} // namespace game
