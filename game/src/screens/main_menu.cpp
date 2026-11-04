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
    const float w=ui_width(r),h=ui_height(r);
    const bool tiny=h<475.0f,compact=h<720.0f;
    r.draw_rect({0,0,w,h},{0.016f,0.031f,0.049f,1});
    for(int i=0;i<20;++i){
        const float y=h*(float)i/20.0f;
        r.draw_rect({0,y,w,std::min(h/20.0f+1.0f,h-y)},{0.016f+0.001f*i,0.031f+0.0017f*i,0.049f+0.0020f*i,1});
    }
    for(int i=0;i<70;++i){
        const unsigned k=(unsigned)(i*15485863+641);
        const float x=(float)((k*7u+13u)%1997)/1997.0f*w;
        const float y=(float)((k*43u+23u)%1597)/1597.0f*h*0.78f;
        r.draw_rect({x,y,i%11==0?2.0f:1.0f,i%11==0?2.0f:1.0f},{0.77f,0.81f,0.88f,0.18f+0.08f*(i%4)});
    }
    const float horizon=h*0.84f;
    r.draw_rect({0,horizon,w,h-horizon},{0.034f,0.069f,0.081f,1});
    for(int i=0;i<42;++i){
        const float x=w*i/42.0f;
        const float y=horizon-4.0f-std::abs(std::sin(i*1.317f))*23.0f;
        r.draw_rect({x,y,std::min(w/42.0f+1.0f,w-x),h-y},{0.046f,0.087f,0.093f,1});
    }
    const float bw=std::min(385.0f,w-30.0f);
    const float button_h=tiny?27.0f:compact?34.0f:45.0f;
    const float gap=tiny?5.0f:compact?8.0f:12.0f;
    const int button_count=game_session_active_?7:5;
    const float top=tiny?58.0f:compact?98.0f:158.0f;
    const float bottom=button_count*button_h+(button_count-1)*gap;
    const float ph=std::min(h-top-12.0f,bottom+(tiny?20.0f:38.0f));
    const Rect panel{(w-bw)*0.5f-12,top-12,bw+24,ph+24};
    r.draw_rect({panel.x-2,panel.y-2,panel.w+4,panel.h+4},{0.13f,0.29f,0.33f,0.65f});
    r.draw_rect(panel,{0.016f,0.035f,0.051f,0.92f});
    r.draw_rect({panel.x,panel.y,3,panel.h},{0.33f,0.63f,0.65f,1});
    const float header_y=tiny?8.0f:compact?13.0f:24.0f;
    ui::label(r,{(w-std::min(510.0f,w-18.0f))*0.5f,header_y},tiny?"LUNAR FRONTIER":"L U N A R   F R O N T I E R",tiny?0.75f:compact?1.08f:1.6f,theme_);
    if(!tiny) ui::label_fitted(r,{(w-540.0f)*0.5f,header_y+(compact?24.0f:43.0f),540,20},"MISSION DIRECTOR  /  DESIGN THE FUTURE OF LUNAR EXPLORATION",compact?0.42f:0.63f,theme_,true);
    ui::label_fitted(r,{w-150.0f,8.0f,139.0f,18.0f},"V "+std::string(version::current),0.36f,theme_,true);
    float y=top;
    auto menu_button=[&](const char* text,auto action){
        if(ui::button(r,ui_input_,{(w-bw)*0.5f,y,bw,button_h},text,theme_))action();
        y+=button_h+gap;
    };
    if(game_session_active_){
        menu_button("RESUME SESSION",[&]{screen_=Screen::Playfield;});
        menu_button("SAVE GAME",[&]{save_status_message_.clear();screen_=Screen::SaveGame;});
    }
    menu_button("NEW GAME  /  CAMPAIGN OR FREE PLAY",[&]{
        new_game_mode_selection_=NewGameMode::Career;
        new_game_phase_selection_=GamePhase::Planning;
        new_game_status_message_.clear();
        screen_=Screen::NewGame;
    });
    menu_button("LOAD GAME",[&]{save_status_message_.clear();refresh_save_games();screen_=Screen::LoadGame;});
    menu_button("SETTINGS",[&]{ensure_display_modes();screen_=Screen::Settings;});
    menu_button("CREDITS",[&]{screen_=Screen::Credits;});
    menu_button("EXIT",[&]{app_.request_quit();});
    if(!save_status_message_.empty()) ui::label_fitted(r,{16,h-24,w-32,18},save_status_message_,0.5f,theme_,true);
    if(game_session_active_ && h>500.0f)ui::label_fitted(r,{(w-480.0f)*0.5f,top+bottom+12,480,18},
                          std::string("ACTIVE: ")+game_phase_name(active_game_phase_),0.47f,theme_,true);
}


} // namespace game
