#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/world/playfield.hpp"

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
    eng::ui::Theme theme_{};
    int playfield_size_=64;
    int viewport_height_=900;
    float fps_=0.0f;
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
    void render_build_icon(eng::Renderer& r,BuildIcon icon,eng::Rect rect,eng::Color color) const;
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
