#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/world/playfield.hpp"
#include <vector>

namespace game {
class GameApp final : public eng::GameDelegate {
public:
    explicit GameApp(eng::Application& app):app_(app){}
    void update(float dt,const eng::InputState& input) override;
    void render(eng::Renderer& r) override;
private:
    enum class Screen { MainMenu, NewGame, Settings, Credits, Playfield };
    eng::Application& app_;
    Screen screen_=Screen::MainMenu;
    eng::InputState input_{};
    eng::InputState ui_input_{};
    eng::ui::Theme theme_{};
    int playfield_size_=64;
    int viewport_height_=900;
    float fps_=0.0f;
    float ui_scale_=1.5f;
    bool debug_frame_enabled_=true;
    int ui_scale_index_=2;
    std::vector<eng::DisplayMode> display_modes_{};
    int display_mode_index_=-1;
    Playfield playfield_{};
    CameraController camera_{};
    ColonyStatus status_{};
    BuildCatalog build_catalog_{};

    void render_menu(eng::Renderer& r);
    void render_new_game(eng::Renderer& r);
    void render_settings(eng::Renderer& r);
    void render_credits(eng::Renderer& r);
    void render_playfield(eng::Renderer& r);
    void render_status_bar(eng::Renderer& r);
    void render_build_bar(eng::Renderer& r);
    void render_debug_frame(eng::Renderer& r);
    void render_build_icon(eng::Renderer& r,BuildIcon icon,eng::Rect rect,eng::Color color) const;
    void render_playfield_crosshair(eng::Renderer& r);
    void ensure_display_modes();
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
