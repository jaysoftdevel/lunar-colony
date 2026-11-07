#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
using namespace layout;

void GameApp::render_menu(Renderer& r){
    const float w=ui_width(r),h=ui_height(r);
    const auto recent=std::find_if(save_games_.begin(),save_games_.end(),[](const SaveGameEntry& entry){
        return build::debug_features || !entry.debug_only;
    });
    const bool can_continue=!game_session_active_ && recent!=save_games_.end();

    for(int i=0;i<12;++i){
        const float y=h*i/12.0f;
        r.draw_rect({0,y,w,std::min(h/12.0f+1.0f,h-y)},
                    {0.016f+0.0015f*i,0.026f+0.0020f*i,0.044f+0.0024f*i,1});
    }
    for(unsigned i=0;i<52;++i){
        const unsigned seed=i*15485863u+641u;
        const float x=float((seed*7u+13u)%1997)/1997.0f*(w-2.0f);
        const float y=float((seed*43u+23u)%1597)/1597.0f*h*0.81f;
        const float size=i%11==0?2.0f:1.0f;
        r.draw_rect({x,y,size,size},{0.77f,0.81f,0.88f,0.18f+0.08f*(i%4)});
    }
    const float horizon=h*0.84f;
    for(int i=0;i<24;++i){
        const float x=w*i/24.0f;
        const float y=horizon-4.0f-std::abs(std::sin(i*1.317f))*23.0f;
        r.draw_rect({x,y,std::min(w/24.0f+1.0f,w-x),h-y},{0.049f,0.071f,0.082f,1});
    }

    const bool split=w>=750.0f;
    const bool compact=h<720.0f;
    const float button_h=compact?36.0f:44.0f,gap=compact?7.0f:10.0f,padding=18.0f;
    const int button_count=5+(game_session_active_?2:can_continue?1:0);
    const float panel_h=button_count*button_h+(button_count-1)*gap+2*padding;
    const float panel_w=std::min(split?870.0f:408.0f,w-2*SCREEN_MARGIN);
    const Rect panel{(w-panel_w)*0.5f,std::max(103.0f,(h-panel_h)*0.5f),panel_w,panel_h};
    ui::label_fitted(r,{panel.x,panel.y-94,panel.w,20},"MISSION CONTROL",1.04f,theme_,true);
    ui::label_fitted(r,{panel.x,panel.y-64,panel.w,48},"LUNAR FRONTIER",4.16f,theme_);
    ui::panel(r,panel,theme_);
    r.draw_rect({panel.x,panel.y,panel.w,2},theme_.accent);

    const float x=panel.x+padding;
    float y=panel.y+padding;
    const float actions_w=split?398.0f:panel.w-2*padding;
    auto controls=theme_;
    controls.font_scale=2.0f;
    auto primary=controls;
    primary.panel2={0.10f,0.28f,0.33f,1};
    primary.border={0.24f,0.57f,0.63f,1};
    const auto menu_button=[&](const char* label,bool highlighted=false){
        const bool pressed=ui::button(r,ui_input_,{x,y,actions_w,button_h},label,highlighted?primary:controls);
        y+=button_h+gap;
        return pressed;
    };
    if(game_session_active_){
        if(menu_button("RESUME SESSION",true) || ui_input_.enter_pressed){
            screen_=Screen::Playfield;
            return;
        }
        if(menu_button("SAVE GAME")){
            save_status_message_.clear();
            screen_=Screen::SaveGame;
        }
    }else if(can_continue){
        if((menu_button("CONTINUE",true) || ui_input_.enter_pressed) && resume_save(*recent)) return;
    }
    if(menu_button("NEW GAME",!game_session_active_ && !can_continue)){
        new_game_mode_selection_=NewGameMode::Career;
        new_game_phase_selection_=GamePhase::Planning;
        new_game_status_message_.clear();
        screen_=Screen::NewGame;
    }
    if(menu_button("LOAD GAME")){
        save_status_message_.clear();
        refresh_save_games();
        load_save_scroll_=0;
        screen_=Screen::LoadGame;
        return;
    }
    if(menu_button("SETTINGS")){ensure_display_modes();screen_=Screen::Settings;}
    if(menu_button("CREDITS")) screen_=Screen::Credits;
    if(menu_button("EXIT")) app_.request_quit();

    if(split){
        const Rect context{x+actions_w+padding,panel.y+padding,panel.w-actions_w-3*padding,panel.h-2*padding};
        r.draw_rect(context,{0.032f,0.055f,0.074f,1});
        const float left=context.x+padding,inner=context.w-2*padding;
        const float top=context.y+padding;
        const auto line=[&](float dy,const std::string& value,float scale,bool muted=false){
            ui::label_fitted(r,{left,top+dy,inner,23.0f},value,scale,theme_,muted);
        };
        if(game_session_active_){
            line(0,"ACTIVE SESSION",1.0f,true);
            line(32,game_phase_short_name(active_game_phase_),1.55f);
            line(65,save_game_name_.empty()?"UNSAVED COLONY":save_game_name_,1.0f,true);
        }else if(can_continue){
            line(0,"MOST RECENT SAVE",1.0f,true);
            line(32,recent->name,1.55f);
            line(65,"SAVED WITH V"+recent->version,1.0f,true);
        }else{
            line(0,"NEW MISSION",1.0f,true);
            line(32,"CAMPAIGN",1.55f);
            line(62,"PLAN AND LAUNCH A LUNAR EXPEDITION",1.0f,true);
            line(105,"FREE PLAY",1.55f);
            line(135,"START AT A PLAYABLE STAGE",1.0f,true);
        }
    }
    const std::string release=std::string(build::debug_features?"DEBUG  /  V":"RELEASE  /  V")+std::string(version::current);
    ui::label_fitted(r,{w-167.0f,h-31.0f,151.0f,18.0f},release,0.67f,theme_,true);
    if(!save_status_message_.empty())
        ui::label_fitted(r,{16,h-31,w-198,18},save_status_message_,0.73f,theme_,true);
}
} // namespace game
