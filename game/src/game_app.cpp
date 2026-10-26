#include "game/game_app.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
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

Vec3 rotate_y(Vec3 v,float degrees){
    const float a=radians(degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {v.x*c+v.z*s,v.y,-v.x*s+v.z*c};
}

float wrap_degrees(float degrees){
    degrees=std::fmod(degrees,360.0f);
    if(degrees<0.0f) degrees+=360.0f;
    return degrees;
}

struct OrientedFootprint {
    Vec2 center{};
    Vec2 axis_x{1,0};
    Vec2 axis_z{0,1};
    float half_x{};
    float half_z{};
};

OrientedFootprint footprint(Vec3 position,float rotation_degrees,float size_x,float size_z){
    const float a=radians(rotation_degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {{position.x,position.z},{c,-s},{s,c},size_x*0.5f,size_z*0.5f};
}

float projected_radius(const OrientedFootprint& box,Vec2 axis){
    return box.half_x*std::abs(box.axis_x.x*axis.x+box.axis_x.y*axis.y)
         + box.half_z*std::abs(box.axis_z.x*axis.x+box.axis_z.y*axis.y);
}

bool overlaps(const OrientedFootprint& a,const OrientedFootprint& b,float clearance){
    const Vec2 axes[4]={a.axis_x,a.axis_z,b.axis_x,b.axis_z};
    const Vec2 delta{b.center.x-a.center.x,b.center.y-a.center.y};
    for(const Vec2 axis:axes){
        const float separation=std::abs(delta.x*axis.x+delta.y*axis.y);
        if(separation>=projected_radius(a,axis)+projected_radius(b,axis)+clearance) return false;
    }
    return true;
}

float distance_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

float point_segment_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return distance_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return distance_xz(p,{a.x+dx*t,0.0f,a.z+dz*t});
}


const char* habitat_name(HabitatKind kind){
    switch(kind){
        case HabitatKind::AirLock: return "AIR LOCK";
        case HabitatKind::OxygenGeneratorDome: return "OXYGEN GENERATOR DOME";
        case HabitatKind::CantineDome: return "CANTINE DOME";
        case HabitatKind::BioDome: return "BIO DOME";
        case HabitatKind::CrewQuarters: return "CREW QUARTERS";
        default: return "HABITAT";
    }
}

std::string corridor_endpoint_name(const CorridorEndpoint& endpoint){
    std::ostringstream out;
    switch(endpoint.kind){
        case CorridorEndpointKind::Habitat: out<<"HABITAT "<<(endpoint.index+1); break;
        case CorridorEndpointKind::Corridor: out<<"CORRIDOR "<<(endpoint.index+1)<<" INTERSECTION"; break;
        case CorridorEndpointKind::OpenDoor: out<<"SEALED END"; break;
    }
    return out.str();
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
        if(in.escape_pressed){
            // ESC cancels the most specific active gameplay operation first.
            if(build_catalog_.selected()!=nullptr){
                build_catalog_.clear_selection();
                placement_rotation_degrees_=0.0f;
                corridor_build_start_.reset();
                return;
            }
            if(corridor_link_source_>=0){
                corridor_link_source_=-1;
                return;
            }
            screen_=Screen::MainMenu;
            return;
        }

        InputState camera_input=in;
        if(const BuildObjectDefinition* selected=build_catalog_.selected(); selected && selected->kind==BuildObjectKind::Habitat){
            constexpr float step=15.0f;
            if(in.wheel>0.0f) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_+step);
            if(in.wheel<0.0f) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_-step);
            if(in.key_n_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_-step);
            if(in.key_m_pressed) placement_rotation_degrees_=wrap_degrees(placement_rotation_degrees_+step);
            camera_input.wheel=0.0f;
        }
        camera_.update(camera_input,playfield_.size(),viewport_height_);
        status_.update(dt);

        // World selection and manual corridor linking use the playfield ray.
        // HUD clicks are excluded so buttons never select geometry behind them.
        const float logical_y=(float)in.mouse_y/ui_scale_;
        const bool over_world=logical_y>=STATUS_BAR_H && logical_y<((float)viewport_height_/ui_scale_-BUILD_BAR_H);
        if(in.left_pressed && build_catalog_.selected()==nullptr && over_world && !mouse_over_selection_panel(in)){
            Vec3 hit{};
            if(camera_.intersect_playfield(in.mouse_x,in.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)){
                if(corridor_link_source_>=0){
                    const int target=pick_corridor(hit,1.70f);
                    if(target>=0 && target!=corridor_link_source_){
                        if(corridor_network_.connect_corridors((std::size_t)corridor_link_source_,(std::size_t)target,placed_habitats_,habitat_models_)){
                            selected_corridor_=(int)corridor_network_.connections().size()-1;
                            selected_habitat_=-1;
                            selection_kind_=SelectionKind::Corridor;
                            corridor_link_source_=-1;
                        }
                    }
                }else{
                    select_world_at(in.mouse_x,in.mouse_y);
                }
            }
        }
    } else if(in.escape_pressed && screen_!=Screen::MainMenu) {
        screen_=Screen::MainMenu;
    }
}

void GameApp::render(Renderer& r){
    viewport_width_=r.width();
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
    render_corridors(r);
    render_habitats(r);
    render_corridor_link_preview(r);
    render_corridor_placement(r);
    render_habitat_placement(r);
    render_playfield_crosshair(r);

    r.set_2d(ui_scale_);
    render_status_bar(r);
    render_build_bar(r);
    if(debug_frame_enabled_) render_debug_frame(r);
    if(selection_kind_!=SelectionKind::None) render_selection_panel(r);

    Rect info={PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN,430,56};
    r.draw_rect(info,{0.025f,0.035f,0.045f,0.88f});
    std::ostringstream a;
    a<<"PLAYFIELD "<<playfield_.size()<<" X "<<playfield_.size();
    game_label(r,{info.x+12,info.y+8},a.str(),0.78f,theme_);
    const BuildObjectDefinition* active_build=build_catalog_.selected();
    const char* controls=corridor_link_source_>=0?"LMB TARGET CORRIDOR / ESC CANCEL":
        (active_build && active_build->kind==BuildObjectKind::Corridor?
            (corridor_build_start_?"LMB SET CORRIDOR END / ESC CANCEL":"LMB SET CORRIDOR START / ESC CANCEL"):
        (active_build?"LMB PLACE / WHEEL OR N/M ROTATE / ESC CANCEL":"LMB SELECT / WHEEL ZOOM / MMB ORBIT / RMB PAN / ESC MENU"));
    game_label(r,{info.x+12,info.y+31},controls,0.52f,theme_,true);
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


namespace {
std::vector<Vec3> transformed_vertices(const std::vector<Vec3>& src,Vec3 p,float rotation_degrees){
    std::vector<Vec3> out;
    out.reserve(src.size());
    for(const Vec3& v:src) out.push_back(rotate_y(v,rotation_degrees)+p);
    return out;
}

}

void GameApp::render_habitat_mesh(Renderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected) const {
    const auto shell=transformed_vertices(mesh.shell,position,rotation_degrees);
    const auto foundation=transformed_vertices(mesh.foundation,position,rotation_degrees);
    const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
    const auto details=transformed_vertices(mesh.details,position,rotation_degrees);
    const auto glazing=transformed_vertices(mesh.glazing,position,rotation_degrees);
    const auto edges=transformed_vertices(mesh.edges,position,rotation_degrees);

    if(preview){
        const Color pc=valid?Color{0.18f,0.78f,0.56f,0.30f}:Color{0.92f,0.25f,0.20f,0.30f};
        const Color pe=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        r.draw_triangles_3d(foundation,{pc.r,pc.g,pc.b,0.42f});
        r.draw_triangles_3d(shell,pc);
        r.draw_triangles_3d(interior,{pc.r,pc.g,pc.b,0.34f});
        r.draw_triangles_3d(details,{pc.r,pc.g,pc.b,0.40f});
        r.draw_triangles_3d(glazing,{pc.r,pc.g,pc.b,0.16f});
        r.draw_lines_3d(edges,pe,1.8f);
        return;
    }

    const Color shell_opaque{0.46f,0.51f,0.54f,1.0f};
    const Color foundation_color{0.31f,0.35f,0.37f,1.0f};
    const Color interior_color{0.57f,0.60f,0.60f,1.0f};
    const Color detail_color{0.34f,0.39f,0.42f,1.0f};
    const Color glass_color{0.16f,0.26f,0.29f,0.42f};
    const Color edge_color=selected?theme_.accent:Color{0.08f,0.11f,0.13f,1.0f};
    const float edge_width=selected?2.4f:1.15f;

    r.draw_triangles_3d(foundation,foundation_color);

    if(habitat_view_mode_==HabitatViewMode::Default){
        r.draw_triangles_3d(shell,shell_opaque);
        r.draw_triangles_3d(interior,interior_color);
        r.draw_triangles_3d(details,detail_color);
        r.draw_triangles_3d(glazing,glass_color);
        r.draw_lines_3d(edges,edge_color,edge_width);
        return;
    }

    r.draw_triangles_3d(interior,interior_color);
    r.draw_triangles_3d(details,detail_color);
    r.draw_triangles_3d(glazing,glass_color);
    r.draw_lines_3d(edges,{edge_color.r,edge_color.g,edge_color.b,selected?1.0f:0.52f},selected?2.4f:1.0f);
    r.draw_triangles_3d(shell,{0.46f,0.51f,0.54f,0.10f});
}

void GameApp::render_corridors(Renderer& r){
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& connection=corridors[i];
        const bool selected=selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)i;
        const bool link_source=corridor_link_source_==(int)i;
        const Color shell_color{0.37f,0.43f,0.46f,1.0f};
        const Color detail_color{0.28f,0.33f,0.36f,1.0f};
        const Color edge_color=(selected||link_source)?theme_.accent:Color{0.07f,0.10f,0.12f,1.0f};
        const float edge_width=(selected||link_source)?2.5f:1.15f;

        if(habitat_view_mode_==HabitatViewMode::Default){
            r.draw_triangles_3d(connection.mesh.shell,shell_color);
            r.draw_triangles_3d(connection.mesh.details,detail_color);
            r.draw_lines_3d(connection.mesh.edges,edge_color,edge_width);
            continue;
        }

        // FULL inspection view: only the corridor pressure hull fades. The
        // internal walkway, ribs, docking hardware, junctions and doors stay
        // visible, matching the habitat hull-only transparency rule.
        r.draw_triangles_3d(connection.mesh.details,detail_color);
        r.draw_lines_3d(connection.mesh.edges,{edge_color.r,edge_color.g,edge_color.b,(selected||link_source)?1.0f:0.52f},edge_width);
        r.draw_triangles_3d(connection.mesh.shell,{shell_color.r,shell_color.g,shell_color.b,0.10f});
    }
}

void GameApp::render_habitats(Renderer& r){
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        render_habitat_mesh(r,habitat_models_.mesh(instance.kind),instance.position,instance.rotation_degrees,false,true,selected);
    }
}


bool GameApp::placement_valid(HabitatKind kind,Vec3 position,float rotation_degrees) const {
    const HabitatMesh& candidate=habitat_models_.mesh(kind);
    const float half_field=(float)playfield_.size()*0.5f;
    const float a=radians(rotation_degrees);
    const float hx=std::abs(std::cos(a))*candidate.footprint_x*0.5f+std::abs(std::sin(a))*candidate.footprint_z*0.5f;
    const float hz=std::abs(std::sin(a))*candidate.footprint_x*0.5f+std::abs(std::cos(a))*candidate.footprint_z*0.5f;
    if(position.x-hx<-half_field || position.x+hx>half_field || position.z-hz<-half_field || position.z+hz>half_field) return false;

    const auto candidate_box=footprint(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z);
    constexpr float clearance=0.45f;
    for(const auto& placed:placed_habitats_){
        const HabitatMesh& other=habitat_models_.mesh(placed.kind);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    return true;
}

Rect GameApp::selection_panel_rect() const {
    return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,360.0f,218.0f};
}

bool GameApp::mouse_over_selection_panel(const InputState& input) const {
    if(selection_kind_==SelectionKind::None) return false;
    const Rect p=selection_panel_rect();
    return p.contains((float)input.mouse_x/ui_scale_,(float)input.mouse_y/ui_scale_);
}

void GameApp::clear_world_selection(){
    selection_kind_=SelectionKind::None;
    selected_habitat_=-1;
    selected_corridor_=-1;
}

int GameApp::pick_habitat(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h.kind);
        const float a=radians(h.rotation_degrees);
        const float c=std::cos(a),si=std::sin(a);
        const float dx=hit.x-h.position.x,dz=hit.z-h.position.z;
        const float lx=dx*c-dz*si;
        const float lz=dx*si+dz*c;
        if(std::abs(lx)<=mesh.footprint_x*0.5f && std::abs(lz)<=mesh.footprint_z*0.5f){
            const float d=dx*dx+dz*dz;
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_corridor(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& path=corridors[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

CorridorBuildAnchor GameApp::corridor_anchor_at(Vec3 hit) const {
    // Habitats take priority when the pointer is actually over their footprint;
    // otherwise nearby corridor centerlines become branch/intersection targets.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)habitat,hit};
    const int corridor=pick_corridor(hit,1.65f);
    if(corridor>=0) return corridor_network_.corridor_anchor((size_t)corridor,hit);
    return {CorridorEndpointKind::OpenDoor,0,{std::round(hit.x),0.0f,std::round(hit.z)}};
}

void GameApp::select_world_at(int screen_x,int screen_y){
    Vec3 hit{};
    if(!camera_.intersect_playfield(screen_x,screen_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int corridor=pick_corridor(hit);
    if(corridor>=0){
        selection_kind_=SelectionKind::Corridor;
        selected_corridor_=corridor;
        selected_habitat_=-1;
        return;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        selection_kind_=SelectionKind::Habitat;
        selected_habitat_=habitat;
        selected_corridor_=-1;
        return;
    }
    clear_world_selection();
}

void GameApp::render_corridor_link_preview(Renderer& r){
    if(corridor_link_source_<0 || build_catalog_.selected()!=nullptr) return;
    const float logical_y=(float)input_.mouse_y/ui_scale_;
    if(logical_y<STATUS_BAR_H || logical_y>=ui_height(r)-BUILD_BAR_H) return;
    if(mouse_over_selection_panel(input_)) return;
    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int target=pick_corridor(hit,1.70f);
    if(target<0 || target==corridor_link_source_) return;
    auto preview=corridor_network_.preview_corridor_link((size_t)corridor_link_source_,(size_t)target,placed_habitats_,habitat_models_);
    if(!preview) return;
    r.draw_triangles_3d(preview->mesh.shell,{0.12f,0.72f,0.55f,0.30f});
    r.draw_triangles_3d(preview->mesh.details,{0.24f,0.88f,0.68f,0.46f});
    r.draw_lines_3d(preview->mesh.edges,{0.36f,1.00f,0.76f,0.96f},1.9f);
}

void GameApp::render_selection_panel(Renderer& r){
    const Rect p=selection_panel_rect();
    ui::panel(r,p,theme_);
    r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2.0f);
    const float x=p.x+14.0f;

    if(selection_kind_==SelectionKind::Habitat && selected_habitat_>=0 && (size_t)selected_habitat_<placed_habitats_.size()){
        const auto& h=placed_habitats_[(size_t)selected_habitat_];
        std::ostringstream pos,rot,conn;
        pos<<"POSITION "<<std::fixed<<std::setprecision(1)<<h.position.x<<" / "<<h.position.z;
        rot<<"ROTATION "<<(int)std::round(h.rotation_degrees)<<" DEG";
        conn<<"CORRIDORS "<<corridor_network_.habitat_connection_count((size_t)selected_habitat_);
        game_label(r,{x,p.y+10},"HABITAT #"+std::to_string(selected_habitat_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},habitat_name(h.kind),0.76f,theme_);
        game_label(r,{x,p.y+60},"STATUS  OPERATIONAL / SEALED",0.58f,theme_);
        game_label(r,{x,p.y+84},pos.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+108},rot.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+132},conn.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+170},"LMB SELECT OTHER OBJECT",0.50f,theme_,true);
        return;
    }

    if(selection_kind_==SelectionKind::Corridor && selected_corridor_>=0 && (size_t)selected_corridor_<corridor_network_.connections().size()){
        const auto& c=corridor_network_.connections()[(size_t)selected_corridor_];
        std::ostringstream length,segments,endpoints;
        length<<"LENGTH "<<std::fixed<<std::setprecision(1)<<corridor_network_.connection_length((size_t)selected_corridor_)<<" M";
        segments<<"CURVE SAMPLES "<<c.path.size();
        endpoints<<corridor_endpoint_name(c.endpoint_a)<<" <-> "<<corridor_endpoint_name(c.endpoint_b);
        const char* kind_label=c.kind==CorridorConnectionKind::Automatic?"AUTOMATIC LINK":
                               (c.kind==CorridorConnectionKind::NetworkLink?"NETWORK LINK":"BUILT SECTION");
        game_label(r,{x,p.y+10},"CORRIDOR #"+std::to_string(selected_corridor_+1),0.64f,theme_,true);
        game_label(r,{x,p.y+34},kind_label,0.74f,theme_);
        game_label(r,{x,p.y+59},"STATUS  PRESSURIZED / WALKABLE",0.56f,theme_);
        game_label(r,{x,p.y+82},length.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+105},segments.str(),0.58f,theme_,true);
        game_label(r,{x,p.y+128},endpoints.str(),0.50f,theme_,true);

        const bool linking=corridor_link_source_==selected_corridor_;
        const std::string label=linking?"CANCEL CONNECTION":"CONNECT TO CORRIDOR";
        const bool enabled=linking || corridor_network_.connections().size()>1;
        if(ui::button(r,ui_input_,{x,p.y+p.h-47.0f,220.0f,34.0f},label,theme_,enabled)){
            if(linking) corridor_link_source_=-1;
            else corridor_link_source_=selected_corridor_;
        }
        if(linking) game_label(r,{x+230.0f,p.y+p.h-38.0f},"SELECT TARGET",0.47f,theme_,true);
    }
}

void GameApp::render_corridor_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Corridor) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const CorridorBuildAnchor hover=corridor_anchor_at(hit);

    // Small endpoint marker. Habitat/corridor anchors are cyan; a free endpoint
    // is amber to communicate that it will be closed by a pressure door.
    const Color marker=hover.kind==CorridorEndpointKind::OpenDoor?Color{0.92f,0.62f,0.18f,1.0f}:theme_.accent;
    const Vec3 m=hover.kind==CorridorEndpointKind::Corridor?hover.position:
                 (hover.kind==CorridorEndpointKind::OpenDoor?hover.position:hit);
    r.draw_lines_3d({m+Vec3{-0.45f,0.05f,0},m+Vec3{0.45f,0.05f,0},
                     m+Vec3{0,0.05f,-0.45f},m+Vec3{0,0.05f,0.45f}},marker,2.2f);

    std::optional<CorridorConnection> preview;
    if(corridor_build_start_){
        preview=corridor_network_.preview_built_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_);
        if(preview){
            r.draw_triangles_3d(preview->mesh.shell,{0.12f,0.72f,0.55f,0.30f});
            r.draw_triangles_3d(preview->mesh.details,{0.24f,0.88f,0.68f,0.52f});
            r.draw_lines_3d(preview->mesh.edges,{0.36f,1.00f,0.76f,0.96f},1.9f);
        }else{
            const Vec3 a=corridor_build_start_->position;
            r.draw_lines_3d({a+Vec3{0,0.08f,0},hit+Vec3{0,0.08f,0}},{1.0f,0.28f,0.22f,0.9f},2.0f);
        }
    }

    if(!input_.left_pressed) return;
    if(!corridor_build_start_){
        corridor_build_start_=hover;
        clear_world_selection();
        corridor_link_source_=-1;
        return;
    }

    if(preview && corridor_network_.build_corridor(*corridor_build_start_,hover,placed_habitats_,habitat_models_)){
        selected_corridor_=(int)corridor_network_.connections().size()-1;
        selected_habitat_=-1;
        selection_kind_=SelectionKind::Corridor;
        corridor_build_start_.reset();
        build_catalog_.clear_selection();
    }
}

void GameApp::render_habitat_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Habitat) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    Vec3 snapped{std::round(hit.x),0.0f,std::round(hit.z)};
    const bool valid=placement_valid(selected->habitat_kind,snapped,placement_rotation_degrees_);

    // Preview the exact corridor plan that will be committed on placement.
    // The temporary habitat list includes the candidate with its current
    // rotation, so door choice, generated side entrances and all route bends
    // update live while the player moves/rotates the object.
    if(!placed_habitats_.empty()){
        auto preview_habitats=placed_habitats_;
        preview_habitats.push_back({selected->habitat_kind,snapped,placement_rotation_degrees_});
        if(auto preview=corridor_network_.preview_connection(preview_habitats.size()-1,preview_habitats,habitat_models_)){
            const Color shell_color=valid?Color{0.12f,0.72f,0.55f,0.28f}:Color{0.82f,0.20f,0.18f,0.24f};
            const Color detail_color=valid?Color{0.24f,0.88f,0.68f,0.38f}:Color{0.94f,0.30f,0.25f,0.34f};
            const Color edge_color=valid?Color{0.36f,1.00f,0.76f,0.92f}:Color{1.00f,0.40f,0.32f,0.90f};
            r.draw_triangles_3d(preview->mesh.shell,shell_color);
            r.draw_triangles_3d(preview->mesh.details,detail_color);
            r.draw_lines_3d(preview->mesh.edges,edge_color,1.7f);
        }
    }

    render_habitat_mesh(r,habitat_models_.mesh(selected->habitat_kind),snapped,placement_rotation_degrees_,true,valid);

    if(valid && input_.left_pressed){
        placed_habitats_.push_back({selected->habitat_kind,snapped,placement_rotation_degrees_});
        corridor_network_.connect_new_habitat(placed_habitats_.size()-1,placed_habitats_,habitat_models_);
        // A successful placement is a completed operation. Deselect the build
        // object so a second structure is never placed accidentally.
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        corridor_build_start_.reset();
    }
}

void GameApp::render_build_bar(Renderer& r){
    const float logical_w=ui_width(r);
    const float logical_h=ui_height(r);
    const float y=logical_h-BUILD_BAR_H;
    r.draw_rect({0,y,logical_w,BUILD_BAR_H},{0.025f,0.035f,0.045f,0.97f});
    r.draw_line({0,y},{logical_w,y},theme_.border,1.0f);

    // Reserve the right side for the two-state view selector. Build objects
    // stay horizontally arranged in the remaining area.
    constexpr float view_area_w=170.0f;
    const float build_area_w=std::max(330.0f,logical_w-view_area_w-18.0f);
    const auto& items=build_catalog_.items();
    const float total=items.empty()?0.0f:items.size()*BUILD_ICON_SIZE+(items.size()-1)*BUILD_ICON_GAP;
    const float start_x=std::max(12.0f,(build_area_w-total)*0.5f);
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
        if(hover && ui_input_.left_pressed){
            const int before=build_catalog_.selected_index();
            build_catalog_.select((int)i);
            if(build_catalog_.selected_index()!=before){
                placement_rotation_degrees_=0.0f;
                corridor_build_start_.reset();
            }
            clear_world_selection();
            corridor_link_source_=-1;
        }
    }

    render_view_selector(r,y,logical_w);

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

void GameApp::render_view_selector(Renderer& r,float y,float logical_w){
    constexpr float margin=10.0f;
    constexpr float gap=5.0f;
    constexpr float default_w=74.0f;
    constexpr float full_w=58.0f;
    constexpr float button_h=31.0f;
    const float group_w=default_w+full_w+gap;
    const float x=logical_w-group_w-margin;

    game_label(r,{x,y+6.0f},"VIEW",0.48f,theme_,true);
    const float by=y+35.0f;

    struct ViewButton { HabitatViewMode mode; const char* label; float width; };
    const ViewButton buttons[]={
        {HabitatViewMode::Default,"DEFAULT",default_w},
        {HabitatViewMode::Full,"FULL",full_w}
    };

    float bx=x;
    for(const auto& b:buttons){
        const Rect rect{bx,by,b.width,button_h};
        const bool active=habitat_view_mode_==b.mode;
        const bool hover=rect.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
        r.draw_rect(rect,active?Color{0.10f,0.23f,0.27f,1.0f}:(hover?Color{0.11f,0.14f,0.17f,1.0f}:Color{0.06f,0.075f,0.09f,1.0f}));
        const Color border=active?theme_.accent:(hover?theme_.accent_hover:theme_.border);
        r.draw_line({rect.x,rect.y},{rect.x+rect.w,rect.y},border,active?2.0f:1.0f);
        r.draw_line({rect.x+rect.w,rect.y},{rect.x+rect.w,rect.y+rect.h},border,active?2.0f:1.0f);
        r.draw_line({rect.x+rect.w,rect.y+rect.h},{rect.x,rect.y+rect.h},border,active?2.0f:1.0f);
        r.draw_line({rect.x,rect.y+rect.h},{rect.x,rect.y},border,active?2.0f:1.0f);
        const float label_scale=0.43f;
        const float label_w=text_width(b.label,label_scale*GAME_FONT_BOOST,theme_);
        game_label(r,{rect.x+(rect.w-label_w)*0.5f,rect.y+7.0f},b.label,label_scale,theme_,!active);
        if(hover && ui_input_.left_pressed) habitat_view_mode_=b.mode;
        bx+=b.width+gap;
    }
}

void GameApp::render_build_icon(Renderer& r,BuildIcon icon,Rect q,Color c) const {
    const float x=q.x,y=q.y,w=q.w,h=q.h;
    const float cx=x+w*0.5f;
    const float cy=y+h*0.5f;
    switch(icon){
        case BuildIcon::AirLock:
            r.draw_rect({x+8,y+12,w-16,h-20},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+8,y+12},{x+w-8,y+12},c,2.0f);
            r.draw_line({x+8,y+h-8},{x+w-8,y+h-8},c,2.0f);
            r.draw_line({x+8,y+12},{x+8,y+h-8},c,2.0f);
            r.draw_line({x+w-8,y+12},{x+w-8,y+h-8},c,2.0f);
            r.draw_rect({cx-7,cy-7,14,14},{c.r,c.g,c.b,0.16f});
            r.draw_line({cx-8,cy},{cx+8,cy},c,2.0f);
            r.draw_line({cx,cy-8},{cx,cy+8},c,2.0f);
            break;
        case BuildIcon::OxygenDome:
            r.draw_line({x+5,y+h-9},{x+w-5,y+h-9},c,2.0f);
            r.draw_line({x+7,y+h-10},{x+11,y+17},c,2.0f);
            r.draw_line({x+11,y+17},{cx,y+6},c,2.0f);
            r.draw_line({cx,y+6},{x+w-11,y+17},c,2.0f);
            r.draw_line({x+w-11,y+17},{x+w-7,y+h-10},c,2.0f);
            r.draw_rect({x+5,y+h-22,8,12},{c.r,c.g,c.b,0.18f});
            r.draw_rect({x+15,y+h-25,6,15},{c.r,c.g,c.b,0.18f});
            r.draw_line({cx,y+6},{cx,y+h-9},c,1.0f);
            break;
        case BuildIcon::CantineDome:
            r.draw_line({x+5,y+h-8},{x+w-5,y+h-8},c,2.0f);
            r.draw_line({x+6,y+h-9},{x+12,y+16},c,2.0f);
            r.draw_line({x+12,y+16},{cx,y+5},c,2.0f);
            r.draw_line({cx,y+5},{x+w-12,y+16},c,2.0f);
            r.draw_line({x+w-12,y+16},{x+w-6,y+h-9},c,2.0f);
            r.draw_line({x+13,y+18},{x+w-13,y+18},c,1.0f);
            r.draw_line({x+9,y+27},{x+w-9,y+27},c,1.0f);
            r.draw_line({cx-9,cy+7},{cx+9,cy+7},c,2.0f);
            break;
        case BuildIcon::BioDome:
            r.draw_line({x+4,y+h-8},{x+w-4,y+h-8},c,2.0f);
            r.draw_line({x+6,y+h-9},{x+10,y+15},c,2.0f);
            r.draw_line({x+10,y+15},{cx,y+6},c,2.0f);
            r.draw_line({cx,y+6},{x+w-10,y+15},c,2.0f);
            r.draw_line({x+w-10,y+15},{x+w-6,y+h-9},c,2.0f);
            for(int i=0;i<3;++i){
                const float px=x+14.0f+i*10.0f;
                r.draw_line({px,y+h-9},{px,y+20},c,1.5f);
                r.draw_line({px,y+26},{px-5,y+22},c,1.5f);
                r.draw_line({px,y+30},{px+5,y+26},c,1.5f);
            }
            break;
        case BuildIcon::CrewQuarters:
            r.draw_rect({x+5,y+14,w-10,h-23},{c.r,c.g,c.b,0.10f});
            r.draw_line({x+5,y+14},{x+w-5,y+14},c,2.0f);
            r.draw_line({x+5,y+h-9},{x+w-5,y+h-9},c,2.0f);
            r.draw_line({x+5,y+14},{x+5,y+h-9},c,2.0f);
            r.draw_line({x+w-5,y+14},{x+w-5,y+h-9},c,2.0f);
            for(int i=0;i<3;++i){
                const float px=x+12.0f+i*11.0f;
                r.draw_rect({px,cy-4,7,7},{c.r,c.g,c.b,0.20f});
            }
            r.draw_line({x+13,y+12},{x+13,y+h-7},c,1.0f);
            r.draw_line({x+w-13,y+12},{x+w-13,y+h-7},c,1.0f);
            break;
        case BuildIcon::Corridor:
            r.draw_line({x+6,cy+8},{x+14,cy+8},c,2.0f);
            r.draw_line({x+14,cy+8},{x+20,cy-5},c,2.0f);
            r.draw_line({x+20,cy-5},{x+w-7,cy-5},c,2.0f);
            r.draw_line({x+6,cy+14},{x+14,cy+14},c,2.0f);
            r.draw_line({x+14,cy+14},{x+22,cy+1},c,2.0f);
            r.draw_line({x+22,cy+1},{x+w-7,cy+1},c,2.0f);
            r.draw_line({x+6,cy+8},{x+6,cy+14},c,2.0f);
            r.draw_line({x+w-7,cy-5},{x+w-7,cy+1},c,2.0f);
            break;
    }
}
}
