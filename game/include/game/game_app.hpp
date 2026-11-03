#pragma once
#include "engine/engine.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/first_person_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/systems/game_progress.hpp"
#include "game/systems/resource_management.hpp"
#include "game/systems/builder_robot.hpp"
#include "game/world/playfield.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/pressure_door.hpp"
#include "game/world/airlock_cycle.hpp"
#include "game/world/utility_network.hpp"
#include "game/world/walkway_network.hpp"
#include <optional>
#include <array>
#include <cstdint>
#include <memory>
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
    void on_resize(int width,int height) override;
    void update(float dt,const eng::InputState& input) override;
    void render(eng::Renderer& r) override;
private:
    friend struct NavigationRegression;
    friend struct UiRegression;
    enum class Screen { MainMenu, NewGame, SaveGame, LoadGame, Settings, Credits, Playfield };
    enum class HabitatViewMode { Default, Full };
    enum class WorldLayoutMode { Mesh, Textured };
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
    static constexpr int kNewGamePlayfieldMin=32;
    static constexpr int kNewGamePlayfieldMax=1024;
    static constexpr int kNewGamePlayfieldStep=32;
    static constexpr int kNewGamePlayfieldDefault=128;
    static constexpr int kDebugTemplatePlayfieldSize=128;
    int playfield_size_=kNewGamePlayfieldDefault;
    int starting_robot_count_=1;
    int starting_carrier_count_=1;
    int viewport_width_=1600;
    int viewport_height_=900;
    float fps_=0.0f;
    float supply_warning_time_=0.0f;
    float ui_scale_=1.0f;
    bool debug_frame_enabled_=true;
    WorldLayoutMode world_layout_mode_{WorldLayoutMode::Mesh};
    bool game_session_active_=false;
    bool debug_start_template_enabled_=false;
    std::string new_game_status_message_{};
    int ui_scale_index_=0;
    std::vector<eng::DisplayMode> display_modes_{};
    int display_mode_index_=-1;
    Playfield playfield_{};
    CameraController camera_{};
    FirstPersonController first_person_{};
    FirstPersonMode first_person_mode_{FirstPersonMode::Off};
    ColonyStatus status_{};
    // Derived live telemetry; never serialized as stored resources.
    double battery_net_kw_{0},water_net_litres_per_day_{0};
    double supply_power_load_kw_{0},supply_water_load_litres_per_day_{0};
    ResourceManagement resources_{};
    std::vector<BuilderRobotState> builder_robots_{};
    // Monotonic simulation time used by the shared robot safety coordinator.
    // It is transient and restarts when a game/session is loaded.
    float robot_safety_time_seconds_{0.0f};
    // Rover-blocking wall triangles of unfinished construction targets, keyed by
    // target and construction stage so they are filtered once per stage instead
    // of once per collision query. Purely derived data; rebuilt on demand.
    struct NavWallCache;
    std::shared_ptr<NavWallCache> nav_wall_cache_{};
    eng::Vec3 starter_stash_center_{};
    std::array<double,kMaterialResourceCount> production_fraction_{};
    double life_support_fraction_seconds_{0.0};
    BuildCatalog build_catalog_{};
    int build_catalog_scroll_=0;
    HabitatModelLibrary habitat_models_{};
    ResourceModelLibrary resource_models_{};
    std::vector<PlacedHabitat> placed_habitats_{};
    std::vector<PlacedResource> placed_resources_{};
    std::vector<GroundResourcePile> ground_resource_piles_{};
    // Monotonic placement order shared by structures and network blueprints.
    std::uint64_t next_blueprint_placement_sequence_{1};
    std::vector<std::uint64_t> construction_priority_{}; // FIFO priorities, stable placement IDs
    eng::Rect construction_priority_button_rect_{};
    ConstructionTargetRef construction_priority_button_target_{};
    int debug_move_pick_robot_{-1};
    std::optional<GroundResourcePile> first_person_carried_resource_{};
    int first_person_hover_ground_resource_=-1;
    int first_person_hover_storage_resource_=-1;
    int first_person_hover_storage_box_=-1;
    CorridorNetwork corridor_network_{};
    std::vector<PressureDoorState> pressure_doors_{};
    std::vector<airlock::Cycle> airlock_cycles_{}; // transient, seals reset safely on load
    std::vector<float> storage_gate_open_{};
    UtilityNetwork utility_network_{};
    UtilityServiceSnapshot last_supply_services_{}; // derived live topology for progression/UI
    WalkwayNetwork walkway_network_{};
    float placement_rotation_degrees_=0.0f;
    HabitatSizeTier placement_habitat_size_{HabitatSizeTier::Small};
    ResourceSizeTier placement_resource_size_{ResourceSizeTier::Small};
    StorageDomeSize placement_storage_size_{StorageDomeSize::Small};
    // Optional Storage Dome gates are placed as independent structure
    // extensions after the dome blueprint exists. This mode stores only the
    // owning dome; the hovered cargo sector is derived continuously from the
    // cursor and validated before a gate blueprint is created.
    bool storage_gate_placement_active_{false};
    std::size_t storage_gate_placement_owner_{std::numeric_limits<std::size_t>::max()};
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
    // The upper-left selection/info pane is a fixed viewport.  Rich telemetry
    // can be taller than that viewport at high UI scales, so retain a
    // per-selection scroll position instead of allowing details to spill over
    // the build bar or disappear below the pane.
    float selection_panel_scroll_{0.0f};
    bool robot_enclosure_legacy_scan_pending_{true};
    std::uint64_t selection_panel_scroll_key_{0};
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
    void render_progress_tree(eng::Renderer& r);
    void render_build_bar(eng::Renderer& r);
    void render_view_selector(eng::Renderer& r,float y,float logical_w);
    void render_debug_frame(eng::Renderer& r);
    void render_build_icon(eng::Renderer& r,BuildIcon icon,eng::Rect rect,eng::Color color) const;
    void render_playfield_crosshair(eng::Renderer& r);
    void render_first_person_hud(eng::Renderer& r);
    void render_corridors(eng::Renderer& r);
    void render_habitats(eng::Renderer& r);
    void render_resources(eng::Renderer& r);
    void render_storage_loading_gate(eng::Renderer& r,const PlacedResource& item,float opening,bool preview=false) const;
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
    bool resource_placement_valid(ResourceKind kind,ResourceSizeTier size,StorageDomeSize storage_size,eng::Vec3 position,float rotation_degrees) const;
    bool placement_preserves_robot_escape(eng::Vec3 position,float rotation_degrees,float footprint_x,float footprint_z) const;
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
    float selection_panel_content_height() const;
    float selection_panel_max_scroll() const;
    std::uint64_t current_selection_panel_scroll_key() const;
    void sync_selection_panel_scroll();
    void update_selection_panel_scroll(const eng::InputState& input);
    eng::Rect ground_resource_panel_rect() const;
    bool mouse_over_selection_panel(const eng::InputState& input) const;
    void clear_world_selection();
    void finish_selection_marquee();
    void delete_selected_world_object();
    void update_construction(float dt);
    ConstructionTargetRef selected_construction_target() const;
    std::uint64_t construction_target_sequence(const ConstructionTargetRef& target) const;
    std::size_t construction_priority_rank(const ConstructionTargetRef& target) const;
    void toggle_construction_priority(const ConstructionTargetRef& target);
    bool construction_ready_connection(const ConstructionTargetRef& target) const;
    std::size_t construction_dispatch_rank(const ConstructionTargetRef& target) const;
    void prune_construction_priorities();
    bool debug_robot_move_available(std::size_t index) const;
    bool issue_debug_robot_move(std::size_t index,eng::Vec3 destination);

    void reset_builder_robot_and_starter_stash();
    void update_builder_robot(float dt);
    float game_speed_multiplier() const;
    void update_resource_economy(float dt);
    GameProgressSnapshot game_progress_snapshot() const;
    int crew_quarters_capacity() const;
    eng::Rect progress_tree_panel_rect() const;
    UtilityServiceSnapshot update_supply_simulation(float dt);
    void render_supply_indicators(eng::Renderer& r);
    void render_gate_areas(eng::Renderer& r) const;
    bool gate_route_clear(const std::vector<eng::Vec3>& path,float margin) const;
    bool corridor_gate_layout_clear(const CorridorConnection& connection) const;
    bool gate_layout_clear(const PlacedHabitat* habitat,const PlacedResource* resource) const;
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
    float first_person_ceiling_height(eng::Vec3 p) const;
    std::string first_person_environment_label() const;
    float first_person_door_open_fraction(eng::Vec3 door_position) const;
    void update_pressure_doors(float dt);
    int airlock_inside_sign(std::size_t habitat_index) const;
    void update_workshop_gates(float dt);
    bool workshop_accessible(const PlacedResource& candidate) const;
    bool workshop_supplied(std::size_t resource_index) const;
    bool workshop_operational(std::size_t resource_index) const;
    void render_workshop_gate(eng::Renderer& r,const PlacedResource& item,bool preview=false,bool valid=true) const;
    void assign_storage_loading_faces();
    void synchronize_storage_gates();
    bool add_storage_gate_at(std::size_t owner,float local_angle);
    bool storage_gate_face_clear(const PlacedResource& owner,int face,std::optional<std::size_t> owner_index=std::nullopt) const;
    int available_storage_gate_face(std::size_t owner,int requested_face=-1) const;
    bool add_storage_gate(std::size_t owner,int requested_face=-1);
    void render_storage_gate_shutters(eng::Renderer& r,const PlacedResource& owner,int face,float opening,float progress,bool preview=false) const;
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
    bool load_debug_blueprint_template_from_file(const std::string& path,std::string& error);
    std::string debug_blueprint_template_path() const;
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
