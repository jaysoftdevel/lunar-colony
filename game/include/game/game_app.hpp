#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/first_person_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/systems/resource_management.hpp"
#include "game/systems/builder_robot.hpp"
#include "game/world/playfield.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/utility_network.hpp"
#include "game/world/walkway_network.hpp"
#include <optional>
#include <array>
#include <cstdint>
#include <vector>
#include <string>

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
    enum class Screen { MainMenu, NewGame, SaveGame, LoadGame, Settings, Credits, Playfield };
    enum class HabitatViewMode { Default, Full };
    enum class GameSpeed { Normal = 0, Quick, Fast, Pause };
    enum class FirstPersonMode { Off, DropPending, Active };
    enum class SelectionKind { None, Habitat, Corridor, Resource, GroundResource, Utility, Walkway, Robot };

    struct SaveGameEntry {
        std::string name{};
        std::string path{};
        std::string version{};
    };
    eng::Application& app_;
    Screen screen_=Screen::MainMenu;
    eng::InputState input_{};
    eng::InputState ui_input_{};
    eng::ui::Theme theme_{};
    int playfield_size_=64;
    int starting_robot_count_=1;
    int starting_carrier_count_=1;
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
    FirstPersonController first_person_{};
    FirstPersonMode first_person_mode_{FirstPersonMode::Off};
    ColonyStatus status_{};
    ResourceManagement resources_{};
    std::vector<BuilderRobotState> builder_robots_{};
    // Synchronous pathfinding is bounded once per rendered update. All 90 Hz
    // catch-up substeps share this budget so a slow frame cannot recursively
    // create more expensive path searches.
    int robot_constructor_route_plans_budget_{0};
    int robot_carrier_route_plans_budget_{0};
    int robot_full_route_plans_budget_{0};
    eng::Vec3 starter_stash_center_{};
    std::array<double,kMaterialResourceCount> production_fraction_{};
    double life_support_fraction_seconds_{0.0};
    BuildCatalog build_catalog_{};
    HabitatModelLibrary habitat_models_{};
    ResourceModelLibrary resource_models_{};
    std::vector<PlacedHabitat> placed_habitats_{};
    std::vector<PlacedResource> placed_resources_{};
    std::vector<GroundResourcePile> ground_resource_piles_{};
    std::optional<GroundResourcePile> first_person_carried_resource_{};
    int first_person_hover_ground_resource_=-1;
    int first_person_hover_storage_resource_=-1;
    int first_person_hover_storage_box_=-1;
    CorridorNetwork corridor_network_{};
    UtilityNetwork utility_network_{};
    WalkwayNetwork walkway_network_{};
    float placement_rotation_degrees_=0.0f;
    HabitatSizeTier placement_habitat_size_{HabitatSizeTier::Standard};
    StorageDomeSize placement_storage_size_{StorageDomeSize::Small};
    HabitatViewMode habitat_view_mode_=HabitatViewMode::Default;
    GameSpeed game_speed_=GameSpeed::Normal;
    SelectionKind selection_kind_=SelectionKind::None;
    int selected_habitat_=-1;
    int selected_corridor_=-1;
    int selected_resource_=-1;
    int selected_ground_resource_=-1;
    std::vector<int> selected_ground_resources_{};
    bool selection_marquee_active_{false};
    bool selection_marquee_dragged_{false};
    int selection_marquee_start_x_{0};
    int selection_marquee_start_y_{0};
    int selected_utility_=-1;
    int selected_walkway_=-1;
    int selected_robot_=-1;
    int corridor_link_source_=-1;
    std::optional<CorridorBuildAnchor> corridor_build_start_{};
    std::optional<UtilityBuildAnchor> utility_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_target_{};
    std::string save_game_name_{};
    std::string save_status_message_{};
    std::vector<SaveGameEntry> save_games_{};
    int load_save_scroll_{0};

    void render_menu(eng::Renderer& r);
    void render_new_game(eng::Renderer& r);
    void render_save_game(eng::Renderer& r);
    void render_load_game(eng::Renderer& r);
    void render_settings(eng::Renderer& r);
    void render_credits(eng::Renderer& r);
    void render_playfield(eng::Renderer& r);
    void render_status_bar(eng::Renderer& r);
    void render_build_bar(eng::Renderer& r);
    void render_view_selector(eng::Renderer& r,float y,float logical_w);
    void render_debug_frame(eng::Renderer& r);
    void render_build_icon(eng::Renderer& r,BuildIcon icon,eng::Rect rect,eng::Color color) const;
    void render_playfield_crosshair(eng::Renderer& r);
    void render_first_person_hud(eng::Renderer& r);
    void render_corridors(eng::Renderer& r);
    void render_habitats(eng::Renderer& r);
    void render_resources(eng::Renderer& r);
    void render_ground_resources(eng::Renderer& r);
    void render_builder_robot(eng::Renderer& r);
    void render_utilities(eng::Renderer& r);
    void render_walkways(eng::Renderer& r);
    void render_selection_panel(eng::Renderer& r);
    void render_selection_marquee(eng::Renderer& r);
    void render_construction_status_frames(eng::Renderer& r);
    void render_corridor_link_preview(eng::Renderer& r);
    void render_habitat_placement(eng::Renderer& r);
    void render_resource_placement(eng::Renderer& r);
    void render_utility_placement(eng::Renderer& r);
    void render_walkway_placement(eng::Renderer& r);
    void render_corridor_placement(eng::Renderer& r);
    void render_habitat_mesh(eng::Renderer& r,const HabitatMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,float construction_step=(float)kConstructionSteps,const std::vector<CorridorDoor>* fpv_doors=nullptr) const;
    void render_resource_mesh(eng::Renderer& r,const ResourceMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,float construction_step=(float)kConstructionSteps,const std::vector<CorridorDoor>* fpv_doors=nullptr) const;
    bool placement_valid(HabitatKind kind,HabitatSizeTier size,eng::Vec3 position,float rotation_degrees) const;
    bool resource_placement_valid(ResourceKind kind,StorageDomeSize storage_size,eng::Vec3 position,float rotation_degrees) const;
    eng::Vec3 snap_storage_dome(eng::Vec3 hit,StorageDomeSize size,float& rotation_degrees) const;
    void select_world_at(int screen_x,int screen_y);
    int pick_habitat(eng::Vec3 ground_hit) const;
    int pick_corridor(eng::Vec3 ground_hit,float radius=geometry::kCorridorHalfWidth+0.18f) const;
    int pick_resource(eng::Vec3 ground_hit) const;
    int pick_ground_resource(eng::Vec3 ground_hit,float radius=0.45f) const;
    int pick_utility(eng::Vec3 ground_hit,float radius=0.70f) const;
    int pick_walkway(eng::Vec3 ground_hit,float radius=geometry::kWalkwayHalfWidth+0.16f) const;
    int pick_robot(eng::Vec3 ground_hit,float radius=0.82f) const;
    std::optional<UtilityBuildAnchor> utility_anchor_at(eng::Vec3 ground_hit,UtilityKind kind) const;
    std::optional<WalkwayBuildAnchor> walkway_anchor_at(eng::Vec3 ground_hit,bool source_only) const;
    CorridorBuildAnchor corridor_anchor_at(eng::Vec3 ground_hit) const;
    eng::Rect selection_panel_rect() const;
    eng::Rect ground_resource_panel_rect() const;
    bool mouse_over_selection_panel(const eng::InputState& input) const;
    void clear_world_selection();
    void finish_selection_marquee();
    void delete_selected_world_object();
    void update_construction(float dt);
    void reset_builder_robot_and_starter_stash();
    void update_builder_robot(float dt);
    float game_speed_multiplier() const;
    void update_resource_economy(float dt);
    ResourceBundle starter_resource_reserve() const;
    eng::Vec3 construction_target_position(const ConstructionTargetRef& target) const;
    eng::Vec3 construction_drop_position(const ConstructionTargetRef& target) const;
    eng::Vec3 construction_drop_package_position(const ConstructionTargetRef& target,int visual_slot) const;
    bool construction_target_complete(const ConstructionTargetRef& target) const;
    bool construction_target_dependencies_ready(const ConstructionTargetRef& target) const;
    const ConstructionResourcePlan* construction_target_plan(const ConstructionTargetRef& target) const;
    std::uint8_t construction_target_step(const ConstructionTargetRef& target) const;
    std::uint8_t construction_target_total_steps(const ConstructionTargetRef& target) const;
    bool advance_construction_target(const ConstructionTargetRef& target,float dt);
    std::size_t construction_site_owner_index(const ConstructionTargetRef& target) const;
    ResourceBundle* construction_site_buffer(const ConstructionTargetRef& target);
    const ResourceBundle* construction_site_buffer(const ConstructionTargetRef& target) const;
    ResourceBundle construction_site_reserved(const ConstructionTargetRef& target,std::size_t exclude_robot=kNoRobotIndex) const;
    ResourceBundle construction_site_remaining(const ConstructionTargetRef& target,bool include_reservations=true,std::size_t exclude_robot=kNoRobotIndex) const;
    bool robot_load_needed_resources(BuilderRobotState& robot,const ResourceBundle& need);
    void spawn_produced_resource(MaterialResource resource,std::int64_t amount,eng::Vec3 source_position);
    void enter_first_person_drop_mode();
    void exit_first_person_mode();
    void update_first_person(float dt,const eng::InputState& input);
    void update_first_person_resource_hover();
    void first_person_grab_or_drop_resource();
    void drop_first_person_carried_resource();
    bool store_first_person_carried_resource();
    void stored_resource_box_pose(const PlacedResource& structure,const ResourceMesh& mesh,std::size_t box_index,eng::Vec3& base,float& half_width,float& height) const;
    bool first_person_spawn_valid(eng::Vec3 p) const;
    bool first_person_move_allowed(eng::Vec3 from,eng::Vec3 to) const;
    float first_person_floor_height(eng::Vec3 p) const;
    std::string first_person_environment_label() const;
    float first_person_door_open_fraction(eng::Vec3 door_position) const;
    std::vector<CorridorDoor> habitat_doors(std::size_t habitat_index) const;
    std::vector<CorridorDoor> storage_dome_doors(std::size_t resource_index) const;
    bool habitat_has_complete_corridor_connection(std::size_t habitat_index) const;
    bool habitat_exterior_access_enabled(std::size_t habitat_index,std::size_t entrance_index) const;
    ResourceBundle committed_construction_resources() const;
    bool can_start_construction(const ConstructionResourcePlan& plan) const;
    void drop_structure_resources(eng::Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used);
    void drop_path_resources(const std::vector<eng::Vec3>& path,float lateral_width,const ResourceBundle& used);
    ResourceBundle store_resource_bundle(const ResourceBundle& resources,float condition=1.0f);
    void recover_removed_resources(eng::Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used);
    bool move_ground_resource_to_storage(std::size_t pile_index);
    void consume_storage_inventory(const ResourceBundle& cost);
    void ensure_display_modes();
    std::string save_game_directory() const;
    void refresh_save_games();
    bool save_game_to_name(const std::string& name,std::string& error);
    bool load_game_from_file(const std::string& path,std::string& error);
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
