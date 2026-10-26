#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/world/playfield.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/corridor_network.hpp"
#include <optional>
#include <vector>

namespace game {
class GameApp final : public eng::GameDelegate {
public:
    explicit GameApp(eng::Application& app):app_(app){}
    void update(float dt,const eng::InputState& input) override;
    void render(eng::Renderer& r) override;
private:
    enum class Screen { MainMenu, NewGame, Settings, Credits, Playfield };
    enum class HabitatViewMode { Default, Full };
    enum class SelectionKind { None, Habitat, Corridor };
    eng::Application& app_;
    Screen screen_=Screen::MainMenu;
    eng::InputState input_{};
    eng::InputState ui_input_{};
    eng::ui::Theme theme_{};
    int playfield_size_=64;
    int viewport_width_=1600;
    int viewport_height_=900;
    float fps_=0.0f;
    float ui_scale_=1.5f;
    bool debug_frame_enabled_=true;
    bool game_session_active_=false;
    int ui_scale_index_=2;
    std::vector<eng::DisplayMode> display_modes_{};
    int display_mode_index_=-1;
    Playfield playfield_{};
    CameraController camera_{};
    ColonyStatus status_{};
    BuildCatalog build_catalog_{};
    HabitatModelLibrary habitat_models_{};
    std::vector<PlacedHabitat> placed_habitats_{};
    CorridorNetwork corridor_network_{};
    float placement_rotation_degrees_=0.0f;
    HabitatViewMode habitat_view_mode_=HabitatViewMode::Default;
    SelectionKind selection_kind_=SelectionKind::None;
    int selected_habitat_=-1;
    int selected_corridor_=-1;
    int corridor_link_source_=-1;
    std::optional<CorridorBuildAnchor> corridor_build_start_{};

    void render_menu(eng::Renderer& r);
    void render_new_game(eng::Renderer& r);
    void render_settings(eng::Renderer& r);
    void render_credits(eng::Renderer& r);
    void render_playfield(eng::Renderer& r);
    void render_status_bar(eng::Renderer& r);
    void render_build_bar(eng::Renderer& r);
    void render_view_selector(eng::Renderer& r,float y,float logical_w);
    void render_debug_frame(eng::Renderer& r);
    void render_build_icon(eng::Renderer& r,BuildIcon icon,eng::Rect rect,eng::Color color) const;
    void render_playfield_crosshair(eng::Renderer& r);
    void render_corridors(eng::Renderer& r);
    void render_habitats(eng::Renderer& r);
    void render_selection_panel(eng::Renderer& r);
    void render_corridor_link_preview(eng::Renderer& r);
    void render_habitat_placement(eng::Renderer& r);
    void render_corridor_placement(eng::Renderer& r);
    void render_habitat_mesh(eng::Renderer& r,const HabitatMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false) const;
    bool placement_valid(HabitatKind kind,eng::Vec3 position,float rotation_degrees) const;
    void select_world_at(int screen_x,int screen_y);
    int pick_habitat(eng::Vec3 ground_hit) const;
    int pick_corridor(eng::Vec3 ground_hit,float radius=1.45f) const;
    CorridorBuildAnchor corridor_anchor_at(eng::Vec3 ground_hit) const;
    eng::Rect selection_panel_rect() const;
    bool mouse_over_selection_panel(const eng::InputState& input) const;
    void clear_world_selection();
    void ensure_display_modes();
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
