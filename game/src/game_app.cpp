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
using namespace game::runtime;
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
    autosave_minutes_=preferences.autosave_minutes;
    show_fps_=preferences.show_fps;
    first_person_.set_fov(float(preferences.first_person_fov));
    world_layout_mode_=preferences.mesh_layout?WorldLayoutMode::Mesh:WorldLayoutMode::FullGraphics;
    soundscape_.set_music_enabled(preferences.music_enabled);
    soundscape_.set_volume(preferences.sound_volume_percent/100.f);
    ui_scale_index_=preferences.ui_scale_index;
    habitat_view_mode_=preferences.faded_hulls?HabitatViewMode::None:HabitatViewMode::FullHull;
    debug_frame_enabled_=preferences.debug_frame_enabled;
    preferred_display_mode_={preferences.display_width,preferences.display_height,0};
    if(preferences.display_width>0&&preferences.display_height>0)
        app_.set_display_mode(preferences.display_width,preferences.display_height);
    // Windowed is available during this session; every launch begins fullscreen.
    app_.set_fullscreen(true);
    on_resize(viewport_width_,viewport_height_);
    refresh_save_games();
}

void GameApp::persist_preferences(){
    Preferences preferences;
    preferences.graphics=graphics_;
    preferences.autosave_minutes=autosave_minutes_;
    preferences.mesh_layout=world_layout_mode_==WorldLayoutMode::Mesh;
    preferences.music_enabled=soundscape_.music_enabled();
    preferences.first_person_fov=int(std::lround(first_person_.fov()));
    preferences.show_fps=show_fps_;
    preferences.ui_scale_index=ui_scale_index_;
    preferences.faded_hulls=habitat_view_mode_==HabitatViewMode::None;
    preferences.sound_volume_percent=int(std::lround(soundscape_.volume()*100));
    preferences.debug_frame_enabled=debug_frame_enabled_;
    preferences.display_width=preferred_display_mode_.width;
    preferences.display_height=preferred_display_mode_.height;
    preferences_save_failed_=!save_preferences(preferences);
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

Rect GameApp::centered_button(SceneRenderer& r,float y,float w,float h) const {
    return {(ui_width(r)-w)*0.5f,y,w,h};
}

void GameApp::render(SceneRenderer& r){
    on_resize(r.width(),r.height());
    ui_input_=input_;
    ui_input_.mouse_x=(int)std::lround((float)input_.mouse_x/ui_scale_);
    ui_input_.mouse_y=(int)std::lround((float)input_.mouse_y/ui_scale_);
    ui_input_.mouse_dx=(int)std::lround((float)input_.mouse_dx/ui_scale_);
    ui_input_.mouse_dy=(int)std::lround((float)input_.mouse_dy/ui_scale_);

    if(graphics_benchmark_.running() && !graphics_benchmark_.same_size(r.width(),r.height()))
        graphics_benchmark_.stop(BenchmarkStatus::Resized);
    const bool benchmarking=graphics_benchmark_.running();
    r.set_graphics_benchmark(benchmarking);
    if(benchmarking){
        r.set_graphics_settings(graphics_benchmark_.current().settings);
        r.begin_frame({0.018f,0.024f,0.032f,1});
        const bool rendered=render_benchmark_scene(r);
        r.end_frame();
        graphics_benchmark_.record(r.benchmark_frame_ms(),rendered);
        return;
    }

    auto render_options=graphics_;
    if(mesh_layout_active()) render_options.path=GraphicsPath::Classic;
    r.set_graphics_settings(render_options);
    r.begin_frame({0.018f,0.024f,0.032f,1});
    if(screen_==Screen::Playfield){
        if(active_game_phase_==GamePhase::Planning){
            const auto planning_input=ui_input_;
            if(mouse_over_activity_log(input_)){ui_input_.left_pressed=false;ui_input_.wheel=0.f;}
            render_planning_game(r);
            ui_input_=planning_input;
            if(active_game_phase_==GamePhase::Planning && activity_log_visible())render_activity_log(r);
        }else{
            const auto first_placement=next_blueprint_placement_sequence_;
            render_playfield(r);
            if(game_session_active_){
                const bool instant=build::debug_features && active_game_phase_==GamePhase::Debug && stage3_mode_==Stage3Mode::Sandbox;
                for(auto id=first_placement;id<next_blueprint_placement_sequence_;++id)
                    activity_log_.record(ui_time_seconds_,LogEvent::Note,"Blueprint #"+std::to_string(id)+
                        (instant?" placed and completed in Sandbox":" placed"));
            }
        }
        if(simulation_paused()) render_pause_frame(r);
    } else {
        // Menus always use the normal desktop pointer. The crosshair is reserved
        // for the actual 3D playfield surface only.
        app_.set_world_cursor(false);
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
    if(show_fps_ && screen_==Screen::Playfield){
        r.set_2d(ui_scale_);
        float y=ui_height(r)-BUILD_BAR_H-29.f;
        if(first_person_mode_!=FirstPersonMode::Active && activity_log_visible()){
            const auto dock=activity_log_panel_rect();if(dock.h>0)y=std::min(y,dock.y-29.f);
        }
        r.draw_rect({10,y,124,25},{.015f,.025f,.035f,.88f});
        r.draw_text({17,y+6},"FPS "+std::to_string(int(std::lround(std::clamp(fps_,0.f,9999.f)))),2.0f,{1.f,.95f,.40f,1});
    }
    if(screen_==Screen::Playfield && ui_time_seconds_<autosave_notice_until_){
        r.set_2d(ui_scale_);
        const Rect toast{std::max(8.f,(ui_width(r)-340.f)*.5f),STATUS_BAR_H+12.f,340.f,30.f};
        r.draw_rect(toast,{.025f,.055f,.07f,.94f});
        ui::label_fitted(r,{toast.x+10,toast.y+9,toast.w-20,16},autosave_notice_,.82f,theme_);
    }
    if constexpr(build::debug_features){
        if(screen_==Screen::Playfield && !auto_run_.message.empty() &&
           (auto_run_.phase!=AutoRunPhase::Finished || ui_time_seconds_<auto_run_.notice_until)){
            r.set_2d(ui_scale_);
            const float width=std::min(400.f,ui_width(r)-16.f);
            const float y=STATUS_BAR_H+(ui_time_seconds_<autosave_notice_until_?48.f:12.f);
            const Rect banner{(ui_width(r)-width)*.5f,y,width,28.f};
            r.draw_rect(banner,{.025f,.055f,.07f,.94f});
            ui::label_fitted(r,{banner.x+9,banner.y+6,banner.w-18,16},auto_run_.message,.62f,theme_);
        }
    }
    render_first_person_alerts(r);
    r.end_frame();
}

void GameApp::render_pause_frame(SceneRenderer& r) const {
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
