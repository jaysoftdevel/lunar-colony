#include "game/game_app.hpp"
#include "game/config/preferences.hpp"
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


const char* GameApp::game_phase_name(GamePhase phase){
    switch(phase){
        case GamePhase::Planning: return "STAGE 1 - PLANNING";
        case GamePhase::RobotPreparation: return "STAGE 2 - PREPARING BASE WITH ROBOTS";
        case GamePhase::Debug: return "STAGE 3 - DEBUG";
    }
    return "UNKNOWN GAME STAGE";
}

const char* GameApp::game_phase_short_name(GamePhase phase){
    switch(phase){
        case GamePhase::Planning: return "PLANNING";
        case GamePhase::RobotPreparation: return "ROBOT BASE PREPARATION";
        case GamePhase::Debug: return "DEBUG";
    }
    return "UNKNOWN";
}

void GameApp::on_start(){
    const auto preferences=load_preferences();
    graphics_=preferences.graphics;
    world_layout_mode_=preferences.mesh_layout?WorldLayoutMode::Mesh:WorldLayoutMode::FullGraphics;
    soundscape_.set_music_enabled(preferences.music_enabled);
}

void GameApp::persist_preferences(){
    preferences_save_failed_=!save_preferences({graphics_,world_layout_mode_==WorldLayoutMode::Mesh,soundscape_.music_enabled()});
}

void GameApp::toggle_world_layout(){
    if constexpr(!build::debug_features) return;
    world_layout_mode_=mesh_layout_active()?WorldLayoutMode::FullGraphics:WorldLayoutMode::Mesh;
    persist_preferences();
}

void GameApp::on_resize(int width,int height){
    viewport_width_=std::max(1,width);viewport_height_=std::max(1,height);
    ui_scale_=resolution_scale(viewport_width_,viewport_height_,UI_SCALES[std::clamp(ui_scale_index_,0,UI_SCALE_COUNT-1)]);
}

Rect GameApp::centered_button(Renderer& r,float y,float w,float h) const {
    return {(ui_width(r)-w)*0.5f,y,w,h};
}

void GameApp::render(Renderer& r){
    on_resize(r.width(),r.height());
    ui_input_=input_;
    ui_input_.mouse_x=(int)std::lround((float)input_.mouse_x/ui_scale_);
    ui_input_.mouse_y=(int)std::lround((float)input_.mouse_y/ui_scale_);
    ui_input_.mouse_dx=(int)std::lround((float)input_.mouse_dx/ui_scale_);
    ui_input_.mouse_dy=(int)std::lround((float)input_.mouse_dy/ui_scale_);

    auto render_options=graphics_;
    if(mesh_layout_active()) render_options.path=GraphicsPath::Classic;
    r.set_graphics_settings(render_options);
    r.begin_frame({0.018f,0.024f,0.032f,1});
    if(screen_==Screen::Playfield){
        if(active_game_phase_==GamePhase::Planning) render_planning_game(r);
        else render_playfield(r);
        if(simulation_paused()) render_pause_frame(r);
    } else {
        // Menus always use the normal desktop pointer. The crosshair is reserved
        // for the actual 3D playfield surface only.
        app_.set_cursor_visible(true);
        r.set_2d(ui_scale_);
        switch(screen_){
            case Screen::MainMenu: render_menu(r); break;
            case Screen::NewGame: render_new_game(r); break;
            case Screen::Stage3Modes: render_stage3_modes(r); break;
            case Screen::SaveGame: render_save_game(r); break;
            case Screen::LoadGame: render_load_game(r); break;
            case Screen::Settings: render_settings(r); break;
            case Screen::GraphicsSettings: render_graphics_settings(r); break;
            case Screen::Credits: render_credits(r); break;
            case Screen::Playfield: break;
        }
    }
    r.end_frame();
}

void GameApp::render_pause_frame(Renderer& r) const {
    r.set_2d(ui_scale_);
    const float width=ui_width(r),height=ui_height(r);
    constexpr float thickness=3.5f;
    constexpr double period=4.0;
    const float pulse=static_cast<float>(std::sin(ui_time_seconds_*6.283185307179586/period));
    const Color frame{0.96f,0.73f,0.32f,0.60f+0.11f*pulse};
    r.draw_rect({0,0,width,thickness},frame);
    r.draw_rect({0,height-thickness,width,thickness},frame);
    r.draw_rect({0,thickness,thickness,height-2*thickness},frame);
    r.draw_rect({width-thickness,thickness,thickness,height-2*thickness},frame);
}

} // namespace game
