#include "game/game_app.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace game {
using namespace eng;

Rect GameApp::centered_button(Renderer& r,float y,float w,float h) const {
    return {(r.width()-w)*0.5f,y,w,h};
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
    r.begin_frame({0.018f,0.024f,0.032f,1});
    r.set_2d();
    switch(screen_){
        case Screen::MainMenu: render_menu(r); break;
        case Screen::NewGame: render_new_game(r); break;
        case Screen::Settings: render_settings(r); break;
        case Screen::Credits: render_credits(r); break;
        case Screen::Playfield: render_playfield(r); break;
    }

    // Application-drawn crosshair replaces the system cursor in every screen.
    r.set_2d();
    const float x=(float)input_.mouse_x,y=(float)input_.mouse_y;
    const Color cc=theme_.accent;
    r.draw_line({x-10,y},{x+10,y},cc,1.5f);
    r.draw_line({x,y-10},{x,y+10},cc,1.5f);
    r.draw_rect({x-1.5f,y-1.5f,3,3},cc);
    r.end_frame();
}

void GameApp::render_menu(Renderer& r){
    ui::title(r,{70,70},"GAME",theme_);
    ui::label(r,{72,112},"C++ PLAYFIELD PROTOTYPE",1.5f,theme_,true);
    float y=260;
    if(ui::button(r,input_,centered_button(r,y),"START NEW GAME",theme_)) screen_=Screen::NewGame;
    y+=72;
    if(ui::button(r,input_,centered_button(r,y),"SETTINGS",theme_)) screen_=Screen::Settings;
    y+=72;
    if(ui::button(r,input_,centered_button(r,y),"CREDITS",theme_)) screen_=Screen::Credits;
    y+=72;
    if(ui::button(r,input_,centered_button(r,y),"EXIT",theme_)) app_.request_quit();
    ui::label(r,{70,(float)r.height()-55},"MOUSE CROSSHAIR   ESC BACK",1.25f,theme_,true);
}

void GameApp::render_new_game(Renderer& r){
    ui::title(r,{70,70},"NEW GAME",theme_);
    Rect p={(r.width()-620)*0.5f,205,620,350};
    ui::panel(r,p,theme_);
    ui::label(r,{p.x+42,p.y+45},"PLAYFIELD SIZE",1.7f,theme_);
    std::ostringstream ss;
    ss<<playfield_size_<<" X "<<playfield_size_;
    ui::label(r,{p.x+215,p.y+130},ss.str(),2.4f,theme_);
    if(ui::button(r,input_,{p.x+65,p.y+118,90,56},"<",theme_)) playfield_size_=std::max(16,playfield_size_-16);
    if(ui::button(r,input_,{p.x+p.w-155,p.y+118,90,56},">",theme_)) playfield_size_=std::min(512,playfield_size_+16);
    ui::label(r,{p.x+42,p.y+220},"MESH ONLY - NO TEXTURES",1.3f,theme_,true);
    if(ui::button(r,input_,{p.x+42,p.y+270,245,52},"BACK",theme_)) screen_=Screen::MainMenu;
    if(ui::button(r,input_,{p.x+p.w-287,p.y+270,245,52},"CREATE",theme_)){
        playfield_.rebuild(playfield_size_);
        camera_.reset(playfield_size_);
        status_.reset();
        screen_=Screen::Playfield;
    }
}

void GameApp::render_settings(Renderer& r){
    ui::title(r,{70,70},"SETTINGS",theme_);
    Rect p={(r.width()-620)*0.5f,205,620,340};
    ui::panel(r,p,theme_);
    ui::label(r,{p.x+42,p.y+48},"DISPLAY",1.7f,theme_);
    ui::label(r,{p.x+42,p.y+105},"FULLSCREEN",1.4f,theme_);
    ui::label(r,{p.x+p.w-165,p.y+105},"ON",1.4f,theme_);
    ui::label(r,{p.x+42,p.y+160},"FULLSCREEN IS THE GAME DISPLAY MODE",1.15f,theme_,true);
    ui::label(r,{p.x+42,p.y+205},"RENDERER OPENGL GPU",1.3f,theme_,true);
    ui::label(r,{p.x+42,p.y+235},"BACKEND REMAINS ISOLATED FOR MIGRATION",1.05f,theme_,true);
    if(ui::button(r,input_,{p.x+42,p.y+270,200,48},"BACK",theme_)) screen_=Screen::MainMenu;
}

void GameApp::render_credits(Renderer& r){
    ui::title(r,{70,70},"CREDITS",theme_);
    Rect p={(r.width()-700)*0.5f,215,700,330};
    ui::panel(r,p,theme_);
    ui::label(r,{p.x+45,p.y+55},"GAME PROTOTYPE",1.8f,theme_);
    ui::label(r,{p.x+45,p.y+115},"NATIVE C++20 / X11 / OPENGL",1.4f,theme_,true);
    ui::label(r,{p.x+45,p.y+155},"ENGINE AND GAME IMPLEMENTATION ARE SEPARATE",1.2f,theme_,true);
    if(ui::button(r,input_,{p.x+45,p.y+245,200,48},"BACK",theme_)) screen_=Screen::MainMenu;
}

void GameApp::render_playfield(Renderer& r){
    const float aspect=(float)r.width()/std::max(1,r.height());
    r.set_3d(camera_.view_projection(aspect));
    r.draw_triangles_3d(playfield_.floor_triangles(),{0.055f,0.075f,0.088f,1});
    r.draw_lines_3d(playfield_.grid_lines(),{0.22f,0.43f,0.47f,1},1.0f);
    r.set_2d();

    render_status_bar(r);
    render_build_bar(r);

    Rect info={22,70,495,92};
    r.draw_rect(info,{0.025f,0.035f,0.045f,0.88f});
    std::ostringstream a,b;
    a<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    b<<"TILT "<<(int)camera_.elevation_degrees()<<"   ROT "<<(int)camera_.azimuth_degrees()<<"   ZOOM "<<(int)camera_.distance();
    ui::label(r,{40,84},a.str(),1.22f,theme_);
    ui::label(r,{40,111},b.str(),1.05f,theme_,true);
    ui::label(r,{40,137},"WHEEL ZOOM / MMB ROTATE TILT / RMB PAN",0.92f,theme_,true);
}

void GameApp::render_status_bar(Renderer& r){
    const float h=54.0f;
    const float fps_w=118.0f;
    const float usable=std::max(500.0f,(float)r.width()-fps_w);
    const float cell=usable/5.0f;
    r.draw_rect({0,0,(float)r.width(),h},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,h-1},{(float)r.width(),h-1},theme_.border,1.0f);

    const int total_minutes=(int)std::floor(status_.hour()*60.0f);
    const int hour=(total_minutes/60)%24;
    const int minute=total_minutes%60;
    std::ostringstream day,power,water,resources,crew,fps;
    day<<"DAY "<<status_.day()<<"  "<<std::setfill('0')<<std::setw(2)<<hour<<":"<<std::setw(2)<<minute<<"  "<<(status_.is_daylight()?"LIGHT":"NIGHT");
    power<<"POWER "<<status_.power_current()<<" / "<<status_.power_capacity();
    water<<"WATER "<<status_.water_current()<<" / "<<status_.water_capacity();
    resources<<"RESOURCES "<<status_.resources();
    crew<<"CREW "<<status_.crew_current()<<" / "<<status_.crew_capacity();
    fps<<"FPS "<<(int)std::round(fps_);

    const std::string values[5]={day.str(),power.str(),water.str(),resources.str(),crew.str()};
    for(int i=0;i<5;++i){
        const float x=cell*i;
        if(i>0) r.draw_line({x,9},{x,h-9},theme_.border,1.0f);
        ui::label(r,{x+18,19},values[i],1.02f,theme_,false);
    }
    r.draw_line({usable,9},{usable,h-9},theme_.border,1.0f);
    ui::label(r,{usable+17,19},fps.str(),1.08f,theme_,false);
}

void GameApp::render_build_bar(Renderer& r){
    const float bar_h=116.0f;
    const float y=(float)r.height()-bar_h;
    r.draw_rect({0,y,(float)r.width(),bar_h},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{(float)r.width(),y},theme_.border,1.0f);

    const auto& items=build_catalog_.items();
    const float icon_size=72.0f;
    const float gap=14.0f;
    const float total=items.empty()?0.0f:items.size()*icon_size+(items.size()-1)*gap;
    const float start_x=((float)r.width()-total)*0.5f;
    const float icon_y=y+22.0f;

    int hovered=-1;
    for(size_t i=0;i<items.size();++i){
        const Rect box={start_x+(icon_size+gap)*(float)i,icon_y,icon_size,icon_size};
        const bool hover=box.contains((float)input_.mouse_x,(float)input_.mouse_y);
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
        render_build_icon(r,items[i].icon,{box.x+12,box.y+12,box.w-24,box.h-24},selected?theme_.accent:theme_.text);
        if(hover && input_.left_pressed) build_catalog_.select((int)i);
    }

    if(hovered>=0){
        const auto& item=items[(size_t)hovered];
        const std::string line1=item.name;
        const std::string line2="TYPE " + item.type;
        const size_t max_chars=std::max(line1.size(),line2.size());
        const float tw=std::max(150.0f,(float)max_chars*7.1f+30.0f);
        const float tx=std::clamp((float)input_.mouse_x-tw*0.5f,12.0f,(float)r.width()-tw-12.0f);
        const Rect tip={tx,y-66.0f,tw,54.0f};
        ui::panel(r,tip,theme_);
        ui::label(r,{tip.x+13,tip.y+10},line1,1.12f,theme_);
        ui::label(r,{tip.x+13,tip.y+31},line2,0.92f,theme_,true);
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
