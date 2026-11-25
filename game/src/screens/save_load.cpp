#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace game::runtime;
using namespace layout;

void GameApp::render_save_game(SceneRenderer& r){
    ui::title(r,{32,26},"SAVE GAME",theme_);
    const float width=std::min(1060.f,ui_width(r)-2*SCREEN_MARGIN),height=std::min(420.f,ui_height(r)-96.f);
    const Rect panel{(ui_width(r)-width)*.5f,70,width,height};ui::panel(r,panel,theme_);
    constexpr float padding=16,gap=18;const float inner=width-2*padding-gap,left_width=inner*.55f;
    const float left=panel.x+padding,right=left+left_width+gap,list_width=inner-left_width;
    const Rect field{left,panel.y+48,left_width,44};const float action_y=panel.y+height-padding-38;
    const Rect list{right,panel.y+46,list_width,std::max(30.f,action_y-panel.y-58)};
    constexpr float row_h=44;const int visible=std::max(1,int(list.h/row_h));const int max_scroll=std::max(0,int(save_games_.size())-visible);
    save_list_scroll_=std::clamp(save_list_scroll_,0,max_scroll);
    save_name_editor_.sync(save_game_name_);
    if(ui_input_.key_tab_pressed){save_name_focus_=!save_name_focus_;save_name_drag_=false;}
    if(ui_input_.wheel&&list.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y)))
        save_list_scroll_=std::clamp(save_list_scroll_+(ui_input_.wheel>0?-1:1),0,max_scroll);
    ui::label(r,{left,panel.y+18},"SAVEGAME NAME",.95f,theme_);
    ui::label_fitted(r,{right,panel.y+18,list_width,18},"EXISTING SAVES / "+std::to_string(save_games_.size()),.88f,theme_);
    const auto choose=[&](int index){if(index<0||index>=int(save_games_.size()))return;pending_save_delete_.clear();save_list_selected_=index;save_game_name_=save_games_[index].name;save_name_editor_.reset(save_game_name_,true);save_name_scroll_=0;save_caret_epoch_=ui_time_seconds_;};
    const bool choose_enter=!save_name_focus_&&ui_input_.enter_pressed&&save_list_selected_>=0;
    if(!save_name_focus_){
        if(ui_input_.edit_up)choose(std::max(0,save_list_selected_-1));
        if(ui_input_.edit_down)choose(std::min(int(save_games_.size())-1,save_list_selected_+1));
        if(save_list_selected_>=0){if(save_list_selected_<save_list_scroll_)save_list_scroll_=save_list_selected_;if(save_list_selected_>=save_list_scroll_+visible)save_list_scroll_=save_list_selected_-visible;}
        if(ui_input_.enter_pressed&&save_list_selected_>=0){choose(save_list_selected_);save_name_focus_=true;}
    }
    for(int row=0;row<visible;++row){const int index=save_list_scroll_+row;if(index>=int(save_games_.size()))break;
        const auto& entry=save_games_[index];Rect bounds{right,list.y+row*row_h,list_width,row_h-4};
        const bool enabled=build::debug_features||!entry.debug_only;
        const bool selected=index==save_list_selected_;
        if(ui::button(r,ui_input_,{bounds.x,bounds.y,bounds.w-92,bounds.h},"",theme_,enabled)){choose(index);save_name_focus_=false;save_name_drag_=false;}
        if(selected)r.draw_line({bounds.x,bounds.y},{bounds.x+bounds.w,bounds.y},theme_.accent,2);
        std::string name=text_edit::display(entry.name);const auto columns=std::size_t(std::max(4.f,(list_width-112)/7.f));if(name.size()>columns)name=name.substr(0,columns-3)+"...";
        ui::label_fitted(r,{bounds.x+9,bounds.y+6,bounds.w-110,15},name,.75f,theme_,!enabled);
        ui::label_fitted(r,{bounds.x+9,bounds.y+24,bounds.w-110,11},entry.version+(entry.debug_only?" / DEBUG":""),.58f,theme_,true);
        const bool confirm=!pending_save_delete_.empty()&&pending_save_delete_==entry.path;
        if(ui::button(r,ui_input_,{bounds.x+bounds.w-88,bounds.y,88,bounds.h},confirm?"CONFIRM":"DELETE",theme_,enabled)){
            if(confirm){const auto path=entry.path;delete_save_entry(path,save_status_message_);pending_save_delete_.clear();return;}
            pending_save_delete_=entry.path;save_status_message_="Click CONFIRM to delete this save.";
        }
    }
    if(save_games_.empty())ui::label_fitted(r,{right,list.y+12,list_width,20},"NO EXISTING SAVES",.8f,theme_,true);
    const float font=std::max(1.4f,theme_.font_scale*1.2f),advance=6*font;
    const int columns=std::max(1,int((field.w-24)/advance));
    const auto locate=[&](int mouse){return text_edit::offset(save_game_name_,std::size_t(std::max(0,save_name_scroll_+int(std::lround((mouse-field.x-12)/advance)))));};
    if(ui_input_.left_pressed&&field.contains(float(ui_input_.mouse_x),float(ui_input_.mouse_y))){
        save_name_focus_=true;save_name_drag_=true;save_name_editor_.move(locate(ui_input_.mouse_x),ui_input_.key_shift_down);save_caret_epoch_=ui_time_seconds_;
    }
    if(save_name_drag_&&ui_input_.left_down){
        if(ui_input_.mouse_x<field.x+8)save_name_scroll_=std::max(0,save_name_scroll_-1);
        if(ui_input_.mouse_x>field.x+field.w-8)++save_name_scroll_;
        save_name_editor_.move(locate(ui_input_.mouse_x),true);save_caret_epoch_=ui_time_seconds_;
    }
    if(!ui_input_.left_down&&!ui_input_.left_pressed)save_name_drag_=false;
    if(save_name_focus_){
        const auto before=save_game_name_;const auto old_caret=save_name_editor_.caret;
        if(const auto copied=save_name_editor_.update(save_game_name_,ui_input_,ui_input_.edit_paste?app_.clipboard_text():std::string{}))app_.set_clipboard_text(*copied);
        if(before!=save_game_name_||old_caret!=save_name_editor_.caret)save_caret_epoch_=ui_time_seconds_;
    }
    const int caret=int(text_edit::count(save_game_name_,save_name_editor_.caret));
    save_name_scroll_=std::clamp(save_name_scroll_,std::max(0,caret-columns+1),std::max(0,caret));
    const auto shown=text_edit::display(save_game_name_);const auto first=std::min(std::size_t(save_name_scroll_),shown.size());
    r.draw_rect(field,theme_.panel2);const auto border=save_name_focus_?theme_.accent:theme_.border;
    r.draw_line({field.x,field.y},{field.x+field.w,field.y},border,1.5f);r.draw_line({field.x,field.y+field.h},{field.x+field.w,field.y+field.h},border,1.5f);
    const auto lo=int(text_edit::count(save_game_name_,save_name_editor_.low())),hi=int(text_edit::count(save_game_name_,save_name_editor_.high()));
    const int start=std::clamp(lo-save_name_scroll_,0,columns),end=std::clamp(hi-save_name_scroll_,0,columns);
    if(end>start)r.draw_rect({field.x+12+start*advance,field.y+11,(end-start)*advance,22},{.12f,.42f,.49f,.9f});
    if(shown.empty())ui::label_fitted(r,{field.x+12,field.y+15,field.w-24,18},"TYPE SAVEGAME NAME",.85f,theme_,true);
    else r.draw_text({field.x+12,field.y+14},shown.substr(first,std::size_t(columns)),font,theme_.text);
    if(save_name_focus_&&std::fmod(ui_time_seconds_-save_caret_epoch_,1.0)<.60){const float x=field.x+12+(caret-save_name_scroll_)*advance;
        r.draw_rect({x,field.y+10,save_name_editor_.overwrite?advance:1.5f,23},{.56f,.97f,.91f,save_name_editor_.overwrite?.3f:1.f});}
    ui::label_fitted(r,{left,panel.y+106,left_width,16},"ARROWS / HOME / END / SHIFT SELECT",.68f,theme_,true);
    ui::label_fitted(r,{left,panel.y+126,left_width,16},"CTRL+A/C/X/V / UNDO Z / REDO Y",.68f,theme_,true);
    std::string candidate=save_game_name_;const auto begin=candidate.find_first_not_of(" \t\r\n");
    candidate=begin==std::string::npos?std::string{}:candidate.substr(begin,candidate.find_last_not_of(" \t\r\n")-begin+1);
    const auto existing=std::find_if(save_games_.begin(),save_games_.end(),[&](const auto& entry){return entry.name==candidate;});
    const bool overwrite=existing!=save_games_.end();
    ui::label_fitted(r,{left,panel.y+150,left_width,18},overwrite?"THIS UPDATES THE EXISTING SAVE":"A NEW SAVE WILL BE CREATED",.70f,theme_);
    if(!save_status_message_.empty())ui::label_fitted(r,{left,panel.y+175,left_width,18},save_status_message_,.75f,theme_);
    const float button_width=std::min(145.f,(left_width-12)*.5f);
    if(ui::button(r,ui_input_,{left,action_y,button_width,38},"BACK",theme_)){pending_save_delete_.clear();save_status_message_.clear();screen_=Screen::MainMenu;return;}
    const bool can_save=game_session_active_&&!candidate.empty()&&(!overwrite||build::debug_features||!existing->debug_only);
    if(ui::button(r,ui_input_,{left+left_width-button_width,action_y,button_width,38},overwrite?"OVERWRITE":"SAVE",theme_,can_save)||(can_save&&save_name_focus_&&ui_input_.enter_pressed&&!ui_input_.key_tab_pressed&&!choose_enter)){
        std::string error;if(save_game_to_name(candidate,error)){save_game_name_=candidate;save_status_message_="Saved: "+save_game_name_;save_name_editor_.reset(save_game_name_);save_list_scroll_=0;const auto saved=std::find_if(save_games_.begin(),save_games_.end(),[&](const auto& entry){return entry.name==save_game_name_;});save_list_selected_=saved==save_games_.end()?-1:int(saved-save_games_.begin());}
        else save_status_message_=error;
    }
    if(ui::button(r,ui_input_,{right,action_y,36,38},"<",theme_,save_list_scroll_>0))save_list_scroll_=std::max(0,save_list_scroll_-visible);
    if(ui::button(r,ui_input_,{right+list_width-36,action_y,36,38},">",theme_,save_list_scroll_<max_scroll))save_list_scroll_=std::min(max_scroll,save_list_scroll_+visible);
    ui::label_fitted(r,{right+43,action_y+14,list_width-86,14},std::to_string(save_games_.empty()?0:save_list_scroll_+1)+"-"+std::to_string(std::min(int(save_games_.size()),save_list_scroll_+visible))+" / "+std::to_string(save_games_.size()),.65f,theme_,true);
}

bool GameApp::resume_save(const SaveGameEntry& entry){
    std::string error;
    if(!load_game_from_file(entry.path,error)){
        save_status_message_=std::move(error);
        return false;
    }
    pending_save_delete_.clear();save_status_message_.clear();
    screen_=Screen::Playfield;
    return true;
}

void GameApp::render_load_game(SceneRenderer& r){
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
    ui::label_fitted(r,{left,p.y+45,right-left,18},
        save_status_message_.empty()?"MOST RECENT FIRST":save_status_message_,0.70f,theme_,true);

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
        const auto directory=save_game_directory().u8string();
        ui::label_fitted(r,{left,list_top+48.0f,right-left,18},std::string(directory.begin(),directory.end()),0.60f,theme_,true);
    }else{
        for(int row=0;row<visible;++row){
            const int index=load_save_scroll_+row;
            if(index>=(int)save_games_.size()) break;
            const auto& entry=save_games_[(std::size_t)index];
            Rect button{left,list_top+(row_h+row_gap)*row,right-left,row_h};
            std::string label=entry.name;
            if(!entry.version.empty()) label+="  ["+entry.version+"]";
            if(entry.debug_only) label+="  [DEBUG]";
            const bool enabled=build::debug_features || !entry.debug_only;
            if(ui::button(r,ui_input_,{button.x,button.y,button.w-108,button.h},label,theme_,enabled) && resume_save(entry)) return;
            const bool confirm=!pending_save_delete_.empty()&&pending_save_delete_==entry.path;
            if(ui::button(r,ui_input_,{button.x+button.w-102,button.y,102,button.h},confirm?"CONFIRM":"DELETE",theme_,enabled)){
                if(confirm){const auto path=entry.path;delete_save_entry(path,save_status_message_);pending_save_delete_.clear();return;}
                pending_save_delete_=entry.path;save_status_message_="Click CONFIRM to delete this save.";
            }
        }
    }

    constexpr float bw=132.0f;
    if(ui::button(r,ui_input_,{left,action_y,bw,action_h},"BACK",theme_)){
        pending_save_delete_.clear();save_status_message_.clear();
        screen_=Screen::MainMenu;
        return;
    }
    if(!save_games_.empty()){
        const int end=std::min(static_cast<int>(save_games_.size()),load_save_scroll_+visible);
        ui::label_fitted(r,{left+bw+14.0f,action_y+13.0f,right-left-3.0f*bw-34.0f,16.0f},
            std::to_string(load_save_scroll_+1)+"-"+std::to_string(end)+" / "+std::to_string(save_games_.size()),
            0.68f,theme_,true);
        if(ui::button(r,ui_input_,{right-2.0f*bw-10.0f,action_y,bw,action_h},"PREV",theme_,load_save_scroll_>0))
            load_save_scroll_=std::max(0,load_save_scroll_-visible);
        if(ui::button(r,ui_input_,{right-bw,action_y,bw,action_h},"NEXT",theme_,load_save_scroll_<max_scroll))
            load_save_scroll_=std::min(max_scroll,load_save_scroll_+visible);
    }
}

} // namespace game
