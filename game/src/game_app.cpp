#include "game/game_app.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace game {
using namespace eng;

namespace {
constexpr float UI_SCALES[]={1.0f,1.25f,1.5f,1.75f,2.0f};
constexpr int UI_SCALE_COUNT=(int)(sizeof(UI_SCALES)/sizeof(UI_SCALES[0]));

// Shared layout metrics keep menus and the in-game HUD visually consistent.
// Values are logical UI pixels and are scaled by Renderer::set_2d(ui_scale_).
constexpr float SCREEN_MARGIN=22.0f;
constexpr float PANEL_PADDING=22.0f;
constexpr float CONTROL_GAP=10.0f;
constexpr float SECTION_GAP=18.0f;
constexpr float STATUS_BAR_H=46.0f;
constexpr float BUILD_BAR_H=80.0f;
constexpr float BUILD_ICON_SIZE=50.0f;
constexpr float BUILD_ICON_GAP=9.0f;
constexpr float PLAYFIELD_MARGIN=12.0f;
// In-game HUD/readout text is intentionally larger than menu text.
// This is independent from both the global base font size and user UI scaling.
constexpr float GAME_FONT_BOOST=1.40f;

struct CommonResolution { int width; int height; };
constexpr CommonResolution COMMON_RESOLUTIONS[]={
    {1024,576},
    {1024,768},
    {1280,720},
    {1280,800},
    {1280,1024},
    {1366,768},
    {1440,900},
    {1600,900},
    {1680,1050},
    {1920,1080},
    {1920,1200},
    {2560,1080},
    {2560,1440},
    {2560,1600},
    {3440,1440},
    {3840,2160}
};

float text_width(const std::string& text,float scale,const ui::Theme& theme){
    return (float)text.size()*6.0f*scale*theme.font_scale;
}

void game_label(Renderer& r,Vec2 pos,const std::string& text,float scale,const ui::Theme& theme,bool muted=false){
    ui::label(r,pos,text,scale*GAME_FONT_BOOST,theme,muted);
}
}

Rect GameApp::centered_button(Renderer& r,float y,float w,float h) const {
    return {(ui_width(r)-w)*0.5f,y,w,h};
}

void GameApp::update(float dt,const InputState& in){
    input_=in;
    if(dt>0.000001f){
        const float instant=1.0f/dt;
        fps_=fps_<=0.0f?instant:(fps_*0.90f+instant*0.10f);
    }

    if(screen_==Screen::Playfield){
        if(in.escape_pressed){ screen_=Screen::MainMenu; return; }
        camera_.update(in,playfield_.size(),viewport_height_);
        status_.update(dt);
    } else if(in.escape_pressed && screen_!=Screen::MainMenu) {
        screen_=Screen::MainMenu;
    }
}

void GameApp::render(Renderer& r){
    viewport_height_=r.height();
    ui_input_=input_;
    ui_input_.mouse_x=(int)std::lround((float)input_.mouse_x/ui_scale_);
    ui_input_.mouse_y=(int)std::lround((float)input_.mouse_y/ui_scale_);
    ui_input_.mouse_dx=(int)std::lround((float)input_.mouse_dx/ui_scale_);
    ui_input_.mouse_dy=(int)std::lround((float)input_.mouse_dy/ui_scale_);

    r.begin_frame({0.018f,0.024f,0.032f,1});
    if(screen_==Screen::Playfield){
        render_playfield(r);
    } else {
        // Menus always use the normal desktop pointer. The crosshair is reserved
        // for the actual 3D playfield surface only.
        app_.set_cursor_visible(true);
        r.set_2d(ui_scale_);
        switch(screen_){
            case Screen::MainMenu: render_menu(r); break;
            case Screen::NewGame: render_new_game(r); break;
            case Screen::Settings: render_settings(r); break;
            case Screen::Credits: render_credits(r); break;
            case Screen::Playfield: break;
        }
    }
    r.end_frame();
}

void GameApp::render_playfield_crosshair(Renderer& r){
    // The top status strip and lower build strip are UI, not part of the
    // playfield pane. Show the generic OS pointer whenever the mouse is there.
    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px){
        app_.set_cursor_visible(true);
        return;
    }

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,r.width(),r.height(),playfield_.size(),hit)){
        app_.set_cursor_visible(true);
        return;
    }

    app_.set_cursor_visible(false);
    const float arm=std::clamp(camera_.distance()*0.030f,1.35f,5.0f);
    const float gap=arm*0.24f;
    const float y=0.055f;
    std::vector<Vec3> lines={
        {hit.x-arm,y,hit.z},{hit.x-gap,y,hit.z},
        {hit.x+gap,y,hit.z},{hit.x+arm,y,hit.z},
        {hit.x,y,hit.z-arm},{hit.x,y,hit.z-gap},
        {hit.x,y,hit.z+gap},{hit.x,y,hit.z+arm}
    };
    r.draw_lines_3d(lines,theme_.accent,2.8f);
}

void GameApp::render_menu(Renderer& r){
    ui::title(r,{32,26},"GAME",theme_);
    ui::label(r,{34,67},"C++ PLAYFIELD PROTOTYPE",1.0f,theme_,true);

    constexpr float button_w=300.0f;
    constexpr float button_h=42.0f;
    float y=112.0f;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"START NEW GAME",theme_)) screen_=Screen::NewGame;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"SETTINGS",theme_)){ ensure_display_modes(); screen_=Screen::Settings; }
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"CREDITS",theme_)) screen_=Screen::Credits;
    y+=button_h+CONTROL_GAP;
    if(ui::button(r,ui_input_,centered_button(r,y,button_w,button_h),"EXIT",theme_)) app_.request_quit();

}

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
        screen_=Screen::Playfield;
    }
}

void GameApp::ensure_display_modes(){
    if(!display_modes_.empty()) return;

    const DisplayMode current=app_.current_display_mode();
    const auto supported=app_.display_modes();

    for(const auto& common:COMMON_RESOLUTIONS){
        const auto it=std::find_if(supported.begin(),supported.end(),[&](const DisplayMode& mode){
            return mode.width==common.width && mode.height==common.height;
        });
        if(it!=supported.end()) display_modes_.push_back(*it);
    }

    if(current.width>0 && current.height>0){
        const bool already_listed=std::any_of(display_modes_.begin(),display_modes_.end(),[&](const DisplayMode& mode){
            return mode.width==current.width && mode.height==current.height;
        });
        if(!already_listed) display_modes_.push_back(current);
    }

    if(display_modes_.empty()) display_modes_.push_back(current.width>0?current:DisplayMode{1600,900,0});

    display_mode_index_=0;
    for(size_t i=0;i<display_modes_.size();++i){
        if(display_modes_[i].width==current.width && display_modes_[i].height==current.height){
            display_mode_index_=(int)i;
            break;
        }
    }
}

void GameApp::render_settings(Renderer& r){
    ensure_display_modes();
    ui::title(r,{32,24},"SETTINGS",theme_);

    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float w=std::min(820.0f,logical_w-2.0f*SCREEN_MARGIN);
    const float h=std::min(260.0f,std::max(250.0f,logical_h-92.0f));
    Rect p={(logical_w-w)*0.5f,70,w,h};
    ui::panel(r,p,theme_);

    const float inner_x=p.x+PANEL_PADDING;
    const float inner_y=p.y+18.0f;
    const float inner_w=p.w-2.0f*PANEL_PADDING;
    const float column_gap=SECTION_GAP+4.0f;
    const float column_w=(inner_w-column_gap)*0.5f;
    const float left_x=inner_x;
    const float right_x=inner_x+column_w+column_gap;
    const float separator_x=inner_x+column_w+column_gap*0.5f;

    ui::label(r,{left_x,inner_y},"DISPLAY",1.18f,theme_);
    ui::label(r,{left_x,inner_y+39},"FULLSCREEN",0.92f,theme_);
    ui::label(r,{left_x+column_w-26,inner_y+39},"ON",0.92f,theme_);

    ui::label(r,{left_x,inner_y+78},"UI SCALE",0.92f,theme_);
    const float small_button=38.0f;
    const float control_right=left_x+column_w;
    if(ui::button(r,ui_input_,{control_right-150,inner_y+66,small_button,34},"<",theme_) && ui_scale_index_>0){
        --ui_scale_index_;
        ui_scale_=UI_SCALES[ui_scale_index_];
    }
    std::ostringstream scale;
    scale<<(int)std::lround(UI_SCALES[ui_scale_index_]*100.0f)<<"%";
    ui::label(r,{control_right-99,inner_y+78},scale.str(),0.94f,theme_);
    if(ui::button(r,ui_input_,{control_right-small_button,inner_y+66,small_button,34},">",theme_) && ui_scale_index_<UI_SCALE_COUNT-1){
        ++ui_scale_index_;
        ui_scale_=UI_SCALES[ui_scale_index_];
    }

    ui::label(r,{left_x,inner_y+119},"RESOLUTION",0.92f,theme_);
    bool resolution_changed=false;
    if(ui::button(r,ui_input_,{control_right-190,inner_y+107,small_button,34},"<",theme_) && display_mode_index_>0){
        --display_mode_index_;
        resolution_changed=true;
    }
    if(ui::button(r,ui_input_,{control_right-small_button,inner_y+107,small_button,34},">",theme_) && display_mode_index_<(int)display_modes_.size()-1){
        ++display_mode_index_;
        resolution_changed=true;
    }
    const auto& mode=display_modes_[(size_t)std::max(0,display_mode_index_)];
    std::ostringstream resolution;
    resolution<<mode.width<<" X "<<mode.height;
    ui::label(r,{control_right-140,inner_y+119},resolution.str(),0.86f,theme_);

    if(resolution_changed) app_.set_display_mode(mode.width,mode.height);
    if(ui::button(r,ui_input_,{control_right-112,inner_y+151,112,34},"APPLY",theme_)){
        app_.set_display_mode(mode.width,mode.height);
    }
    ui::label(r,{left_x,inner_y+158},"COMMON SUPPORTED MODES",0.72f,theme_,true);

    r.draw_line({separator_x,p.y+16},{separator_x,p.y+p.h-58},theme_.border,1.0f);

    ui::label(r,{right_x,inner_y},"GAMEPLAY",1.18f,theme_);
    ui::label(r,{right_x,inner_y+39},"DEBUG FRAME",0.92f,theme_);
    const std::string debug_label=debug_frame_enabled_?"ON":"OFF";
    if(ui::button(r,ui_input_,{right_x+column_w-82,inner_y+27,82,34},debug_label,theme_)){
        debug_frame_enabled_=!debug_frame_enabled_;
    }
    ui::label(r,{right_x,inner_y+78},"SHOW VERSION + FPS IN GAME",0.76f,theme_,true);
    ui::label(r,{right_x,inner_y+105},"UI SCALE APPLIES TO ALL HUD",0.76f,theme_,true);
    ui::label(r,{right_x,inner_y+132},"CROSSHAIR ONLY OVER PLAYFIELD",0.76f,theme_,true);

    constexpr float back_w=130.0f;
    constexpr float back_h=38.0f;
    if(ui::button(r,ui_input_,{p.x+PANEL_PADDING,p.y+p.h-PANEL_PADDING-back_h,back_w,back_h},"BACK",theme_)){
        screen_=Screen::MainMenu;
    }
}

void GameApp::render_credits(Renderer& r){
    ui::title(r,{32,26},"CREDITS",theme_);
    const float w=std::min(610.0f,ui_width(r)-2.0f*SCREEN_MARGIN);
    Rect p={(ui_width(r)-w)*0.5f,100,w,220};
    ui::panel(r,p,theme_);
    const float x=p.x+PANEL_PADDING;
    ui::label(r,{x,p.y+28},"GAME PROTOTYPE",1.28f,theme_);
    ui::label(r,{x,p.y+76},"NATIVE C++20 / X11 / OPENGL",0.92f,theme_,true);
    ui::label(r,{x,p.y+108},"ENGINE AND GAME IMPLEMENTATION ARE SEPARATE",0.78f,theme_,true);
    if(ui::button(r,ui_input_,{x,p.y+p.h-PANEL_PADDING-40,150,40},"BACK",theme_)) screen_=Screen::MainMenu;
}

void GameApp::render_playfield(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    r.set_3d(camera_.view_projection(aspect));
    r.draw_triangles_3d(playfield_.floor_triangles(),{0.055f,0.075f,0.088f,1});
    r.draw_lines_3d(playfield_.grid_lines(),{0.22f,0.43f,0.47f,1},1.0f);
    render_playfield_crosshair(r);

    r.set_2d(ui_scale_);
    render_status_bar(r);
    render_build_bar(r);
    if(debug_frame_enabled_) render_debug_frame(r);

    Rect info={PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,430,56};
    r.draw_rect(info,{0.025f,0.035f,0.045f,0.88f});
    std::ostringstream a;
    a<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    game_label(r,{info.x+12,info.y+8},a.str(),0.78f,theme_);
    game_label(r,{info.x+12,info.y+31},"WHEEL ZOOM / MMB ROTATE+TILT / RMB PAN",0.56f,theme_,true);
}

void GameApp::render_status_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float cell=logical_w/5.0f;
    r.draw_rect({0,0,logical_w,STATUS_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,STATUS_BAR_H-1},{logical_w,STATUS_BAR_H-1},theme_.border,1.0f);

    const int total_minutes=(int)std::floor(status_.hour()*60.0f);
    const int hour=(total_minutes/60)%24;
    const int minute=total_minutes%60;
    std::ostringstream day,power,water,resources,crew;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<status_.power_current()<<"/"<<status_.power_capacity();
    water<<"WATER "<<status_.water_current()<<"/"<<status_.water_capacity();
    resources<<"RES "<<status_.resources();
    crew<<"CREW "<<status_.crew_current()<<"/"<<status_.crew_capacity();

    const std::string values[5]={day.str(),power.str(),water.str(),resources.str(),crew.str()};
    for(int i=0;i<5;++i){
        const float x=cell*i;
        if(i>0) r.draw_line({x,8},{x,STATUS_BAR_H-8},theme_.border,1.0f);
        game_label(r,{x+9,12},values[i],0.72f,theme_,false);
    }
}

void GameApp::render_debug_frame(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    constexpr float frame_w=292.0f;
    constexpr float frame_h=154.0f;
    const Rect frame={logical_w-frame_w-PLAYFIELD_MARGIN,logical_h-BUILD_BAR_H-frame_h-PLAYFIELD_MARGIN,frame_w,frame_h};

    r.draw_rect(frame,{0.025f,0.035f,0.045f,0.92f});
    r.draw_line({frame.x,frame.y},{frame.x+frame.w,frame.y},theme_.accent,2.0f);
    r.draw_line({frame.x+frame.w,frame.y},{frame.x+frame.w,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x+frame.w,frame.y+frame.h},{frame.x,frame.y+frame.h},theme_.border,1.0f);
    r.draw_line({frame.x,frame.y+frame.h},{frame.x,frame.y},theme_.border,1.0f);

    std::ostringstream fps,playfield,camera_a,camera_b;
    fps<<"FPS "<<(int)std::round(fps_);
    playfield<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    camera_a<<"TILT "<<(int)camera_.elevation_degrees()<<"  ROT "<<(int)camera_.azimuth_degrees();
    camera_b<<"ZOOM "<<(int)camera_.distance()<<"  CENTER "
            <<std::fixed<<std::setprecision(1)<<camera_.target().x<<" / "<<camera_.target().z;

    game_label(r,{frame.x+12,frame.y+8},"DEBUG",0.66f,theme_,true);
    game_label(r,{frame.x+12,frame.y+29},"VERSION " + std::string(version::current),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+50},fps.str(),0.68f,theme_);
    game_label(r,{frame.x+12,frame.y+71},playfield.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+94},camera_a.str(),0.62f,theme_);
    game_label(r,{frame.x+12,frame.y+117},camera_b.str(),0.58f,theme_);
}

void GameApp::render_build_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    const auto& items=build_catalog_.items();
    const float total=items.empty()?0.0f:items.size()*BUILD_ICON_SIZE+(items.size()-1)*BUILD_ICON_GAP;
    const float start_x=(logical_w-total)*0.5f;
    const float icon_y=y+(BUILD_BAR_H-BUILD_ICON_SIZE)*0.5f;

    int hovered=-1;
    for(size_t i=0;i<items.size();++i){
        const Rect box={start_x+(BUILD_ICON_SIZE+BUILD_ICON_GAP)*(float)i,icon_y,BUILD_ICON_SIZE,BUILD_ICON_SIZE};
        const bool hover=box.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        const bool selected=(int)i==build_catalog_.selected_index();
        if(hover) hovered=(int)i;
        const Color fill=hover?Color{0.12f,0.16f,0.19f,1.0f}:Color{0.065f,0.08f,0.10f,1.0f};
        r.draw_rect(box,fill);
        const Color border=selected?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        const float bw=selected?3.0f:1.5f;
        r.draw_line({box.x,box.y},{box.x+box.w,box.y},border,bw);
        r.draw_line({box.x+box.w,box.y},{box.x+box.w,box.y+box.h},border,bw);
        r.draw_line({box.x+box.w,box.y+box.h},{box.x,box.y+box.h},border,bw);
        r.draw_line({box.x,box.y+box.h},{box.x,box.y},border,bw);
        render_build_icon(r,items[i].icon,{box.x+8,box.y+8,box.w-16,box.h-16},selected?theme_.accent:theme_.text);
        if(hover && ui_input_.left_pressed) build_catalog_.select((int)i);
    }

    if(hovered>=0){
        const auto& item=items[(size_t)hovered];
        const std::string line1=item.name;
        const std::string line2="TYPE " + item.type;
        const float tw=std::max(168.0f,std::max(text_width(line1,0.78f*GAME_FONT_BOOST,theme_),text_width(line2,0.60f*GAME_FONT_BOOST,theme_))+22.0f);
        const float tx=std::clamp((float)ui_input_.mouse_x-tw*0.5f,8.0f,logical_w-tw-8.0f);
        const Rect tip={tx,y-60.0f,tw,52.0f};
        ui::panel(r,tip,theme_);
        game_label(r,{tip.x+10,tip.y+5},line1,0.78f,theme_);
        game_label(r,{tip.x+10,tip.y+25},line2,0.60f,theme_,true);
    }
}

void GameApp::render_build_icon(Renderer& r,BuildIcon icon,Rect q,Color c) const {
    const float x=q.x,y=q.y,w=q.w,h=q.h;
    const float cx=x+w*0.5f;
    const float cy=y+h*0.5f;
    switch(icon){
        case BuildIcon::Habitat:
            r.draw_rect({x+8,y+18,w-16,h-22},{c.r,c.g,c.b,0.12f});
            r.draw_line({x+6,y+20},{cx,y+5},c,2.0f);
            r.draw_line({cx,y+5},{x+w-6,y+20},c,2.0f);
            r.draw_line({x+8,y+18},{x+8,y+h-4},c,2.0f);
            r.draw_line({x+w-8,y+18},{x+w-8,y+h-4},c,2.0f);
            r.draw_line({x+8,y+h-4},{x+w-8,y+h-4},c,2.0f);
            r.draw_rect({cx-5,y+h-18,10,14},c);
            break;
        case BuildIcon::Solar:
            r.draw_rect({x+5,y+8,w-10,h*0.52f}, {c.r,c.g,c.b,0.14f});
            r.draw_line({x+5,y+8},{x+w-5,y+8},c,2.0f);
            r.draw_line({x+5,y+8+h*0.52f},{x+w-5,y+8+h*0.52f},c,2.0f);
            r.draw_line({x+5,y+8},{x+5,y+8+h*0.52f},c,2.0f);
            r.draw_line({x+w-5,y+8},{x+w-5,y+8+h*0.52f},c,2.0f);
            r.draw_line({cx,y+8},{cx,y+8+h*0.52f},c,1.0f);
            r.draw_line({x+5,y+8+h*0.26f},{x+w-5,y+8+h*0.26f},c,1.0f);
            r.draw_line({cx,cy+3},{cx,y+h-5},c,2.0f);
            r.draw_line({cx-12,y+h-5},{cx+12,y+h-5},c,2.0f);
            break;
        case BuildIcon::Water:
            r.draw_rect({x+11,y+5,w-22,h-10},{c.r,c.g,c.b,0.12f});
            r.draw_line({x+11,y+5},{x+w-11,y+5},c,2.0f);
            r.draw_line({x+11,y+h-5},{x+w-11,y+h-5},c,2.0f);
            r.draw_line({x+11,y+5},{x+11,y+h-5},c,2.0f);
            r.draw_line({x+w-11,y+5},{x+w-11,y+h-5},c,2.0f);
            r.draw_line({x+11,cy},{x+w-11,cy},c,1.5f);
            r.draw_line({cx,y+13},{cx,y+h-13},c,1.5f);
            break;
        case BuildIcon::Storage:
            r.draw_rect({x+7,y+8,w-14,h-16},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+7,y+8},{x+w-7,y+8},c,2.0f);
            r.draw_line({x+7,y+h-8},{x+w-7,y+h-8},c,2.0f);
            r.draw_line({x+7,y+8},{x+7,y+h-8},c,2.0f);
            r.draw_line({x+w-7,y+8},{x+w-7,y+h-8},c,2.0f);
            r.draw_line({x+7,y+8},{x+w-7,y+h-8},c,1.5f);
            r.draw_line({x+w-7,y+8},{x+7,y+h-8},c,1.5f);
            break;
        case BuildIcon::Workshop:
            r.draw_line({x+9,y+h-8},{x+w-8,y+9},c,5.0f);
            r.draw_line({x+7,y+10},{x+21,y+10},c,5.0f);
            r.draw_line({x+9,y+8},{x+9,y+22},c,5.0f);
            r.draw_line({x+w-19,y+h-8},{x+w-7,y+h-8},c,4.0f);
            break;
        case BuildIcon::Farm:
            r.draw_line({x+5,y+h-7},{x+w-5,y+h-7},c,2.0f);
            for(int i=0;i<3;++i){
                const float px=x+13.0f+i*14.0f;
                r.draw_line({px,y+h-8},{px,y+12},c,2.0f);
                r.draw_line({px,y+23},{px-8,y+17},c,2.0f);
                r.draw_line({px,y+30},{px+8,y+23},c,2.0f);
            }
            break;
    }
}
}
