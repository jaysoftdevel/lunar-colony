#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;

namespace {
std::string display_name_field(const std::string& value){
    std::string shown=value;
    if(shown.empty()) shown="TYPE SAVEGAME NAME";
    else shown.push_back('_');
    return shown;
}
}

void GameApp::render_save_game(Renderer& r){
    ui::title(r,{32,26},"SAVE GAME",theme_);
    const float logical_w=ui_width(r);
    const float w=std::min(650.0f,logical_w-2.0f*SCREEN_MARGIN);
    Rect p={(logical_w-w)*0.5f,88.0f,w,250.0f};
    ui::panel(r,p,theme_);

    for(char ch:ui_input_.text_input){
        if(save_game_name_.size()>=64) break;
        if(ch>=32 && ch!=127) save_game_name_.push_back(ch);
    }
    if(ui_input_.backspace_pressed && !save_game_name_.empty()) save_game_name_.pop_back();

    const float left=p.x+PANEL_PADDING;
    const float right=p.x+p.w-PANEL_PADDING;
    ui::label(r,{left,p.y+22},"SAVEGAME NAME",1.05f,theme_);
    Rect field{left,p.y+58,p.w-2.0f*PANEL_PADDING,48.0f};
    r.draw_rect(field,theme_.panel2);
    r.draw_line({field.x,field.y},{field.x+field.w,field.y},theme_.accent,1.5f);
    r.draw_line({field.x+field.w,field.y},{field.x+field.w,field.y+field.h},theme_.border,1.0f);
    r.draw_line({field.x+field.w,field.y+field.h},{field.x,field.y+field.h},theme_.border,1.0f);
    r.draw_line({field.x,field.y+field.h},{field.x,field.y},theme_.border,1.0f);
    ui::label(r,{field.x+12.0f,field.y+15.0f},display_name_field(save_game_name_),0.93f,theme_,save_game_name_.empty());

    ui::label(r,{left,p.y+120},"Typing replaces no files except a save with the same name.",0.72f,theme_,true);
    if(!save_status_message_.empty()) ui::label(r,{left,p.y+146},save_status_message_,0.78f,theme_);

    constexpr float bw=150.0f,bh=40.0f;
    if(ui::button(r,ui_input_,{left,p.y+p.h-PANEL_PADDING-bh,bw,bh},"BACK",theme_)){
        save_status_message_.clear();
        screen_=Screen::MainMenu;
        return;
    }
    const bool can_save=!save_game_name_.empty() && game_session_active_;
    const Rect save_button{right-bw,p.y+p.h-PANEL_PADDING-bh,bw,bh};
    const bool activate=ui::button(r,ui_input_,save_button,"SAVE",theme_,can_save) || (can_save && ui_input_.enter_pressed);
    if(activate){
        std::string error;
        if(save_game_to_name(save_game_name_,error)){
            save_status_message_="Saved: "+save_game_name_;
            screen_=Screen::MainMenu;
        }else save_status_message_=error;
    }
}

void GameApp::render_load_game(Renderer& r){
    ui::title(r,{32,26},"LOAD GAME",theme_);
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float w=std::min(720.0f,logical_w-2.0f*SCREEN_MARGIN);
    const float h=std::min(430.0f,std::max(300.0f,logical_h-110.0f));
    Rect p={(logical_w-w)*0.5f,76.0f,w,h};
    ui::panel(r,p,theme_);

    const float left=p.x+PANEL_PADDING;
    const float right=p.x+p.w-PANEL_PADDING;
    ui::label(r,{left,p.y+18},"SAVED COLONIES",1.05f,theme_);
    if(!save_status_message_.empty()) ui::label(r,{left,p.y+45},save_status_message_,0.70f,theme_,true);

    constexpr float row_h=44.0f;
    constexpr float row_gap=7.0f;
    const float list_top=p.y+68.0f;
    const float action_h=38.0f;
    const float action_y=p.y+p.h-PANEL_PADDING-action_h;
    const int visible=std::max(1,(int)((action_y-list_top-12.0f)/(row_h+row_gap)));
    const int max_scroll=std::max(0,(int)save_games_.size()-visible);
    load_save_scroll_=std::clamp(load_save_scroll_,0,max_scroll);
    if(ui_input_.wheel>0.0f) load_save_scroll_=std::max(0,load_save_scroll_-1);
    if(ui_input_.wheel<0.0f) load_save_scroll_=std::min(max_scroll,load_save_scroll_+1);

    if(save_games_.empty()){
        ui::label(r,{left,list_top+18.0f},"NO SAVE GAMES FOUND",0.92f,theme_,true);
        ui::label(r,{left,list_top+48.0f},save_game_directory(),0.60f,theme_,true);
    }else{
        for(int row=0;row<visible;++row){
            const int index=load_save_scroll_+row;
            if(index>=(int)save_games_.size()) break;
            const auto& entry=save_games_[(std::size_t)index];
            Rect button{left,list_top+(row_h+row_gap)*row,right-left,row_h};
            std::string label=entry.name;
            if(!entry.version.empty()) label+="  ["+entry.version+"]";
            if(ui::button(r,ui_input_,button,label,theme_)){
                std::string error;
                if(load_game_from_file(entry.path,error)){
                    save_status_message_.clear();
                    screen_=Screen::Playfield;
                    return;
                }
                save_status_message_=error;
            }
        }
    }

    constexpr float bw=132.0f;
    if(ui::button(r,ui_input_,{left,action_y,bw,action_h},"BACK",theme_)){
        save_status_message_.clear();
        screen_=Screen::MainMenu;
        return;
    }
    if(!save_games_.empty()){
        if(ui::button(r,ui_input_,{right-2.0f*bw-10.0f,action_y,bw,action_h},"PREV",theme_,load_save_scroll_>0))
            load_save_scroll_=std::max(0,load_save_scroll_-visible);
        if(ui::button(r,ui_input_,{right-bw,action_y,bw,action_h},"NEXT",theme_,load_save_scroll_<max_scroll))
            load_save_scroll_=std::min(max_scroll,load_save_scroll_+visible);
    }
}

} // namespace game
