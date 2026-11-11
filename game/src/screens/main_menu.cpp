#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
using namespace layout;

namespace {
void draw_title_backdrop(Renderer& r,float w,float h){
    r.draw_gradient_rect({0,0,w,h},{.012f,.023f,.043f,1},{.066f,.111f,.141f,1});
    for(unsigned i=0;i<76;++i){
        const unsigned seed=i*15485863u+641u;
        const float x=float((seed*7u+13u)%1997)/1997.0f*(w-2.0f);
        const float y=float((seed*43u+23u)%1597)/1597.0f*h*.83f;
        const float size=i%13==0?2.0f:1.0f;
        r.draw_rect({x,y,size,size},{.77f,.86f,.95f,.19f+.11f*(i%4)});
    }

    const float radius=std::min(h*.33f,w*.22f);
    const Vec2 center{w*.76f,h*.45f};
    constexpr int segments=96;
    std::vector<Vec3> shape;
    shape.reserve(segments*6);
    const auto disc_point=[&](int i){
        const float angle=6.28318530718f*i/segments;
        return Vec3{center.x+radius*std::cos(angle),center.y+radius*std::sin(angle),0};
    };
    for(int i=0;i<segments;++i)
        shape.insert(shape.end(),{Vec3{center.x,center.y,0},disc_point(i),disc_point(i+1)});
    r.draw_triangles_3d(shape,{.044f,.070f,.091f,.98f});
    const auto crescent=[&](int i,float t){
        const float angle=3.14159265359f*i/segments-1.57079632679f;
        const float latitude=std::sin(angle);
        const float half=radius*std::max(0.f,std::cos(angle));
        return Vec3{center.x+half*std::lerp(.43f+.30f*latitude,1.f,t),center.y+radius*latitude,0};
    };
    constexpr std::array<Color,5> shades{{
        {.15f,.22f,.27f,1},{.20f,.28f,.32f,1},{.26f,.34f,.39f,1},
        {.31f,.40f,.44f,1},{.43f,.52f,.56f,1}
    }};
    constexpr float edges[]{0,.28f,.53f,.75f,.985f,1};
    for(std::size_t band=0;band<shades.size();++band){
        shape.clear();
        for(int i=0;i<segments;++i){
            const Vec3 a=crescent(i,edges[band]),b=crescent(i,edges[band+1]);
            const Vec3 c=crescent(i+1,edges[band+1]),d=crescent(i+1,edges[band]);
            shape.insert(shape.end(),{a,b,c,a,c,d});
        }
        r.draw_triangles_3d(shape,shades[band]);
    }

    const auto ridge=[&](float baseline,float height,Color color){
        constexpr int segments=40;
        const auto elevation=[&](float u){
            return baseline-height*(.53f+.24f*std::sin(u*19.0f)+.16f*std::sin(u*33.0f+2.0f));
        };
        std::vector<Vec3> triangles;
        triangles.reserve(segments*6);
        for(int i=0;i<segments;++i){
            const float a=float(i)/segments,b=float(i+1)/segments;
            const Vec3 first{a*w,elevation(a),0},last{b*w,elevation(b),0};
            const Vec3 bottom_left{a*w,h,0},bottom_right{b*w,h,0};
            triangles.insert(triangles.end(),{first,last,bottom_right,first,bottom_right,bottom_left});
        }
        r.draw_triangles_3d(triangles,color);
    };
    ridge(h*.94f,h*.12f,{.052f,.078f,.100f,1});
    ridge(h*1.01f,h*.15f,{.028f,.047f,.062f,1});
}
}

void GameApp::render_menu(Renderer& r){
    const float w=ui_width(r),h=ui_height(r);
    const auto recent=std::find_if(save_games_.begin(),save_games_.end(),[](const SaveGameEntry& entry){
        return build::debug_features || !entry.debug_only;
    });
    const bool can_continue=!game_session_active_ && recent!=save_games_.end();

    draw_title_backdrop(r,w,h);

    const bool split=w>=750.0f;
    const bool compact=h<720.0f;
    const float button_h=compact?36.0f:44.0f,gap=compact?7.0f:10.0f,padding=18.0f;
    const int button_count=5+(game_session_active_?2:can_continue?1:0);
    const float panel_h=button_count*button_h+(button_count-1)*gap+2*padding;
    const float panel_w=std::min(split?870.0f:408.0f,w-2*SCREEN_MARGIN);
    const float panel_x=split?std::max(SCREEN_MARGIN,std::min(w*.11f,w-panel_w-SCREEN_MARGIN)):(w-panel_w)*.5f;
    const Rect panel{panel_x,std::max(103.0f,(h-panel_h)*0.5f),panel_w,panel_h};
    ui::label_fitted(r,{panel.x,panel.y-79,panel.w,62},"LUNAR FRONTIER",5.40f,theme_);
    ui::panel(r,panel,theme_);
    r.draw_rect({panel.x,panel.y,panel.w,2},theme_.accent);

    const float x=panel.x+padding;
    float y=panel.y+padding;
    const float actions_w=split?398.0f:panel.w-2*padding;
    auto controls=theme_;
    controls.font_scale=2.0f;
    auto primary=controls;
    primary.panel2={0.10f,0.27f,0.28f,1};
    primary.accent_hover={0.16f,0.40f,0.40f,1};
    primary.border=theme_.accent;
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
        r.draw_gradient_rect(context,{.047f,.078f,.099f,1},{.033f,.053f,.072f,1});
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
            line(0,"CAMPAIGN",1.55f);
            line(32,"PLAN AND LAUNCH A LUNAR EXPEDITION",1.0f,true);
            line(76,"FREE PLAY",1.55f);
            line(108,"START AT A PLAYABLE STAGE",1.0f,true);
        }
    }
    const std::string release=std::string(build::debug_features?"DEBUG  /  V":"RELEASE  /  V")+std::string(version::current);
    ui::label_fitted(r,{w-167.0f,h-31.0f,151.0f,18.0f},release,0.67f,theme_,true);
    if(!save_status_message_.empty())
        ui::label_fitted(r,{16,h-31,w-198,18},save_status_message_,0.73f,theme_,true);
}
} // namespace game
