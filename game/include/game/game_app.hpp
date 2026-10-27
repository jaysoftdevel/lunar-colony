#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/systems/resource_management.hpp"
#include "game/world/playfield.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/utility_network.hpp"
#include "game/world/walkway_network.hpp"
#include <optional>
#include <cstdint>
#include <vector>

namespace game {

struct GroundResourcePile {
    MaterialResource resource{MaterialResource::Iron};
    std::int64_t amount{0};
    eng::Vec3 position{};
    float radius{0.45f};
    float condition{1.0f};
};

class GameApp final : public eng::GameDelegate {
public:
    explicit GameApp(eng::Application& app):app_(app){}
    void update(float dt,const eng::InputState& input) override;
    void render(eng::Renderer& r) override;
private:
    enum class Screen { MainMenu, NewGame, Settings, Credits, Playfield };
    enum class HabitatViewMode { Default, Full };
    enum class SelectionKind { None, Habitat, Corridor, Resource, GroundResource, Utility, Walkway };
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
    ResourceManagement resources_{};
    BuildCatalog build_catalog_{};
    HabitatModelLibrary habitat_models_{};
    ResourceModelLibrary resource_models_{};
    std::vector<PlacedHabitat> placed_habitats_{};
    std::vector<PlacedResource> placed_resources_{};
    std::vector<GroundResourcePile> ground_resource_piles_{};
    CorridorNetwork corridor_network_{};
    UtilityNetwork utility_network_{};
    WalkwayNetwork walkway_network_{};
    float placement_rotation_degrees_=0.0f;
    HabitatSizeTier placement_habitat_size_{HabitatSizeTier::Standard};
    StorageDomeSize placement_storage_size_{StorageDomeSize::Small};
    HabitatViewMode habitat_view_mode_=HabitatViewMode::Default;
    SelectionKind selection_kind_=SelectionKind::None;
    int selected_habitat_=-1;
    int selected_corridor_=-1;
    int selected_resource_=-1;
    int selected_ground_resource_=-1;
    int selected_utility_=-1;
    int selected_walkway_=-1;
    int corridor_link_source_=-1;
    std::optional<CorridorBuildAnchor> corridor_build_start_{};
    std::optional<UtilityBuildAnchor> utility_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_target_{};

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
    void render_resources(eng::Renderer& r);
    void render_ground_resources(eng::Renderer& r);
    void render_utilities(eng::Renderer& r);
    void render_walkways(eng::Renderer& r);
    void render_selection_panel(eng::Renderer& r);
    void render_corridor_link_preview(eng::Renderer& r);
    void render_habitat_placement(eng::Renderer& r);
    void render_resource_placement(eng::Renderer& r);
    void render_utility_placement(eng::Renderer& r);
    void render_walkway_placement(eng::Renderer& r);
    void render_corridor_placement(eng::Renderer& r);
    void render_habitat_mesh(eng::Renderer& r,const HabitatMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,std::uint8_t construction_step=10) const;
    void render_resource_mesh(eng::Renderer& r,const ResourceMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,std::uint8_t construction_step=10) const;
    bool placement_valid(HabitatKind kind,HabitatSizeTier size,eng::Vec3 position,float rotation_degrees) const;
    bool resource_placement_valid(ResourceKind kind,StorageDomeSize storage_size,eng::Vec3 position,float rotation_degrees) const;
    eng::Vec3 snap_storage_dome(eng::Vec3 hit,StorageDomeSize size,float& rotation_degrees) const;
    void select_world_at(int screen_x,int screen_y);
    int pick_habitat(eng::Vec3 ground_hit) const;
    int pick_corridor(eng::Vec3 ground_hit,float radius=1.45f) const;
    int pick_resource(eng::Vec3 ground_hit) const;
    int pick_ground_resource(eng::Vec3 ground_hit,float radius=0.45f) const;
    int pick_utility(eng::Vec3 ground_hit,float radius=0.70f) const;
    int pick_walkway(eng::Vec3 ground_hit,float radius=0.95f) const;
    std::optional<UtilityBuildAnchor> utility_anchor_at(eng::Vec3 ground_hit,UtilityKind kind) const;
    std::optional<WalkwayBuildAnchor> walkway_anchor_at(eng::Vec3 ground_hit,bool source_only) const;
    CorridorBuildAnchor corridor_anchor_at(eng::Vec3 ground_hit) const;
    eng::Rect selection_panel_rect() const;
    eng::Rect ground_resource_panel_rect() const;
    bool mouse_over_selection_panel(const eng::InputState& input) const;
    void clear_world_selection();
    void delete_selected_world_object();
    void update_construction(float dt);
    ResourceBundle committed_construction_resources() const;
    bool can_start_construction(const ConstructionResourcePlan& plan) const;
    void drop_structure_resources(eng::Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used);
    void drop_path_resources(const std::vector<eng::Vec3>& path,float lateral_width,const ResourceBundle& used);
    ResourceBundle store_resource_bundle(const ResourceBundle& resources,float condition=1.0f);
    void recover_removed_resources(eng::Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used);
    bool move_ground_resource_to_storage(std::size_t pile_index);
    void consume_storage_inventory(const ResourceBundle& cost);
    void ensure_display_modes();
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
