#include <chrono>
#pragma once
#include "engine/engine.hpp"
#include "game/build_mode.hpp"
#include "game/config/graphics_benchmark.hpp"
#include "game/features/build_catalog.hpp"
#include "game/systems/camera_controller.hpp"
#include "game/systems/astronaut_test.hpp"
#include "game/systems/first_person_controller.hpp"
#include "game/systems/colony_status.hpp"
#include "game/systems/game_progress.hpp"
#include "game/systems/planning_game.hpp"
#include "game/systems/lunar_lander.hpp"
#include "game/systems/trader.hpp"
#include "game/systems/soundscape.hpp"
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
#include <filesystem>
#include <memory>
#include <vector>
#include <string>

namespace game {

enum class GroundPileKind : std::uint8_t { Loose, Stack, Stash };
inline constexpr int kGroundStackLayers = 8;

struct GroundResourcePile {
    MaterialResource resource{MaterialResource::Iron};
    std::int64_t amount{0};
    eng::Vec3 position{};
    float radius{0.45f};
    float condition{1.0f};
    GroundPileKind kind{GroundPileKind::Loose};

    std::int64_t top_package_amount() const {
        const auto capacity=resource_package_capacity_kg(resource);
        if(kind==GroundPileKind::Loose) return std::min(amount,capacity);
        const auto remainder=amount%capacity;
        return remainder>0?remainder:std::min(amount,capacity);
    }
    float height() const {
        if(kind==GroundPileKind::Loose) return resource_package_height(resource,amount);
        const auto top=top_package_amount();
        const auto capacity=resource_package_capacity_kg(resource);
        return (float)((amount-top)/capacity)*(resource_package_height(resource,capacity)+.02f)
            +resource_package_height(resource,top);
    }
};

class GameApp final : public eng::GameDelegate {
public:
    explicit GameApp(eng::Application& app):app_(app){}
    void on_start() override;
    void on_resize(int width,int height) override;
    void update(float dt,const eng::InputState& input) override;
    void render(eng::Renderer& r) override;
private:
    friend struct NavigationRegression;
    friend struct UiRegression;
    friend struct LanderRegression;
    friend struct TraderRegression;
    friend struct ShipLayoutRegression;
    enum class Screen { MainMenu, NewGame, Stage3Modes, SaveGame, LoadGame, Settings, GraphicsSettings, Credits, Playfield };
    // Top-level game content. This is intentionally separate from the colony
    // progression stages used inside RobotPreparation. Each phase owns a
    // distinct gameplay surface so future Planning and sandbox mechanics
    // can evolve without coupling them to the robot-preparation simulation.
    enum class GamePhase : std::uint8_t { Planning = 1, RobotPreparation = 2, Debug = 3 };
    // Career begins in Stage 1; Free Play permits an independent stage start.
    // The selected mode is saved independently from the top-level game phase.
    enum class NewGameMode : std::uint8_t { Career = 0, FreePlay = 1 };
    enum class Stage3Mode : std::uint8_t { Sandbox, Template, AstronautTest };
    enum class HabitatViewMode { Default, Full };
    enum class WorldLayoutMode { FullGraphics, Mesh };
    enum class GameSpeed { Normal = 0, Quick, Fast, Pause };
    enum class FirstPersonMode { Off, DropPending, Active };
    enum class SelectionKind { None, Habitat, Corridor, Resource, GroundResource, Utility, Walkway, Robot, Lander, Stash };

    struct SaveGameEntry {
        std::string name{};
        std::string path{};
        std::string version{};
        std::filesystem::file_time_type modified{};
        bool debug_only{};
    };
    void update_simulation(float dt,const eng::InputState& input);
    void update_soundscape(float dt,const eng::InputState& input);
    eng::Application& app_;
    sound::Soundscape soundscape_;
    Screen screen_=Screen::MainMenu;
    eng::InputState input_{};
    eng::InputState ui_input_{};
    eng::ui::Theme theme_{};
    eng::GraphicsSettings graphics_;
    int graphics_tab_{},graphics_page_{};
    GraphicsBenchmark graphics_benchmark_;
    std::size_t benchmark_target_index_{1};
    bool preferences_save_failed_{};
    static constexpr int kNewGamePlayfieldMin=32;
    static constexpr int kNewGamePlayfieldMax=1024;
    static constexpr int kNewGamePlayfieldStep=32;
    static constexpr int kNewGamePlayfieldDefault=128;
    static constexpr int kDebugTemplatePlayfieldSize=128;
    static constexpr int kCompanySitePlayfieldSize=128;
    int playfield_size_=kNewGamePlayfieldDefault;
    int starting_robot_count_=1;
    int starting_carrier_count_=1;
    int viewport_width_=1600;
    int viewport_height_=900;
    float fps_=0.0f;
    double ui_time_seconds_=0.0;
    static constexpr double kStatusRateRefreshSeconds=0.5;
    float ui_scale_=1.0f;
    bool debug_frame_enabled_=build::debug_features;
    WorldLayoutMode world_layout_mode_{WorldLayoutMode::FullGraphics};
    bool game_session_active_=false;
    GamePhase active_game_phase_{GamePhase::RobotPreparation};
    bool debug_mode_active() const {
        return build::debug_features && (debug_frame_enabled_ || active_game_phase_==GamePhase::Debug);
    }
    GamePhase new_game_phase_selection_{GamePhase::RobotPreparation};
    NewGameMode new_game_mode_selection_{NewGameMode::Career};
    NewGameMode active_game_mode_{NewGameMode::FreePlay};
    Stage3Mode stage3_mode_{Stage3Mode::Sandbox};
    bool astronaut_test_active() const {
        return build::debug_features && active_game_phase_==GamePhase::Debug && stage3_mode_==Stage3Mode::AstronautTest;
    }
    std::array<AstronautTestActor,kAstronautRoles.size()> test_astronauts_{};
    int selected_astronaut_=-1;
    AstronautCache astronaut_cache_{};
    float astronaut_schedule_seconds_{};
    PlanningGameState planning_{};
    LunarLanderState lander_{};
    TraderState trader_{};
    std::string trade_message_{};
    std::string planning_status_message_{};
    int planning_contract_page_{0};
    int planning_contract_selected_{0}; // transient contract brief selection, not part of save schema
    int planning_contract_research_page_{0}; // scroll through full contract prerequisite chain
    // Transient, independent 2D navigation for the Stage-1 technology map.
    // No save migration is necessary: research state remains in planning_.
    float planning_tech_zoom_{1.02f};
    float planning_tech_pan_x_{0.0f};
    float planning_tech_pan_y_{0.0f};
    int planning_tech_selected_{0};
    int planning_structure_page_{-1}; // -1 is the payload editor
    // A single click selects; a second click on the same card within the
    // double-click window performs its primary action. Transient UI state only.
    int planning_last_click_group_{-1};
    int planning_last_click_item_{-1};
    int planning_last_click_x_{0}, planning_last_click_y_{0};
    std::chrono::steady_clock::time_point planning_last_click_time_{};
    bool planning_double_clicked(int group,int item);
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
    double supply_power_delivered_kw_{0},supply_power_load_kw_{0},supply_water_load_litres_per_day_{0};
    // Presentation samples only. Quantities and simulation rates remain live.
    struct StatusRateDisplay {
        double next_sample{0};
        bool paused{false};
        double battery_kw{0}, power_load_kw{0}, power_delivered_kw{0}, water_net{0}, water_load{0};
    } status_rate_display_{};
    struct SelectionRateDisplay {
        double next_sample{0};
        bool paused{false};
        std::uint64_t key{0};
        SupplyState supply{};
        double solar_kw{0}, rover_speed_mps{0}, rover_draw_kw{0};
    } selection_rate_display_{};
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
    UtilityServiceSnapshot live_supply_services_{};
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
    std::uint8_t last_running_speed_{1};
    SelectionKind selection_kind_=SelectionKind::None;
    int selected_habitat_=-1;
    int selected_corridor_=-1;
    int selected_resource_=-1;
    int selected_ground_resource_=-1;
    std::vector<int> selected_ground_resources_{};
    bool right_click_pending_{};
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
    // Bio Dome crop selector popup. It is transient UI state and is reset when
    // the selected structure changes or the selection is cleared.
    int hydroponics_dropdown_habitat_{-1};
    int hydroponics_dropdown_bay_{-1};
    bool robot_enclosure_legacy_scan_pending_{true};
    std::uint64_t selection_panel_scroll_key_{0};
    int corridor_link_source_=-1;
    std::optional<CorridorBuildAnchor> corridor_build_start_{};
    std::string corridor_placement_failure_title_{};
    std::string corridor_placement_failure_detail_{};
    std::optional<UtilityBuildAnchor> utility_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_start_{};
    std::optional<WalkwayBuildAnchor> walkway_build_target_{};
    std::string save_game_name_{};
    std::string save_status_message_{};
    std::vector<SaveGameEntry> save_games_{};
    int load_save_scroll_{0};

    void render_menu(eng::Renderer& r);
    void render_new_game(eng::Renderer& r);
    void render_stage3_modes(eng::Renderer& r);
    void start_new_game(Stage3Mode stage3_mode=Stage3Mode::Sandbox);
    void render_save_game(eng::Renderer& r);
    void render_load_game(eng::Renderer& r);
    void render_settings(eng::Renderer& r);
    void persist_preferences();
    bool mesh_layout_active() const { return build::debug_features && world_layout_mode_==WorldLayoutMode::Mesh; }
    void toggle_world_layout();
    void update_speed_shortcuts(const eng::InputState& input);
    bool simulation_paused() const;
    void render_pause_frame(eng::Renderer& r) const;
    void render_graphics_settings(eng::Renderer& r);
    void render_benchmark_settings(eng::Renderer& r,eng::Rect panel,const eng::GraphicsCapabilities& caps);
    bool render_benchmark_scene(eng::Renderer& r);
    void render_credits(eng::Renderer& r);
    void render_playfield(eng::Renderer& r);
    bool handle_astronaut_click(const eng::InputState& input);
    void reset_astronaut_test();
    void update_astronaut_test(float dt);
    void update_astronaut_suits();
    void assign_astronaut_duties(float dt);
    AstronautRestPose astronaut_rest_pose(const AstronautTestActor& actor) const;
    bool consume_colony_stock(MaterialResource material,std::int64_t amount);
    const SupplyState* astronaut_air_at(eng::Vec3 p,bool& airlock) const;
    bool astronaut_placement_clear(eng::Vec3 center,float rotation,float width,float depth) const;
    void render_test_astronauts(eng::Renderer& r) const;
    void render_astronaut_status(eng::Renderer& r,const eng::Mat4& view_projection) const;
    void render_planning_game(eng::Renderer& r);
    void render_planning_technology_tree(eng::Renderer& r,eng::Rect panel,bool compact);
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
    void render_corridor_placement_tooltip(eng::Renderer& r);
    void render_habitat_mesh(eng::Renderer& r,const HabitatMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,float construction_step=(float)kConstructionSteps,const std::vector<CorridorDoor>* fpv_doors=nullptr,bool greenhouse_dark=false,HabitatKind kind=HabitatKind::AirLock) const;
    void render_resource_mesh(eng::Renderer& r,const ResourceMesh& mesh,eng::Vec3 position,float rotation_degrees,bool preview,bool valid=true,bool selected=false,float construction_step=(float)kConstructionSteps,const std::vector<CorridorDoor>* fpv_doors=nullptr,ResourceKind kind=ResourceKind::Count) const;
    struct StructureAttachment {
        eng::Vec3 position;
        std::optional<CorridorConnection> corridor;
        std::optional<UtilityConnection> utility;
    };
    std::optional<StructureAttachment> preview_structure_attachment(
        eng::Vec3 hit,const PlacedHabitat* habitat,const PlacedResource* resource) const;
    void render_structure_attachment(eng::Renderer& r,const StructureAttachment& attachment) const;
    void commit_structure_attachment(StructureAttachment attachment);
    bool structure_routes_clear(eng::Vec3 position,float rotation,float width,float depth) const;
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
    void remove_structure_routes(bool habitat,std::size_t index);
    void remove_corridor_routes(const std::vector<std::size_t>& roots);
    void remove_utility_route(std::size_t index);
    void remap_robot_target_after_removal(ConstructionTargetKind kind,std::size_t index);
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

    void reset_builder_robot_and_starter_stash(const ResourceBundle* shipped_cargo=nullptr);
    void deploy_lunar_base_from_mission();
    void begin_lunar_landing(const ResourceBundle& manifest,int builders,int carriers);
    void update_lunar_lander(float dt);
    bool unload_lunar_lander();
    bool confirm_lander_site(eng::Vec3 p);
    bool confirm_stash_site(eng::Vec3 p);
    bool lander_site_valid(eng::Vec3 p) const;
    bool stash_site_valid(eng::Vec3 p) const;
    bool prepare_default_landing();
    bool salvage_lander();
    bool complete_lander_salvage();
    bool stash_ground_clear(eng::Vec3 position,float width) const;
    std::optional<std::vector<GroundResourcePile>> plan_stash_boxes(const ResourceBundle& cargo,eng::Vec3 marker) const;
    void migrate_boxed_lander_stash(bool legacy_grid);
    bool drop_lander_cargo();
    bool add_stash_boxes(const ResourceBundle& cargo,bool credit=true);
    bool repack_lander_stash();
    void render_stash(eng::Renderer& r);
    void render_stash_panel(eng::Renderer& r);
    std::string landing_message_{};
    void render_lunar_lander(eng::Renderer& r);
    std::size_t trader_pad_index() const;
    const UtilityConnection* trader_conveyor(std::size_t dome) const;
    bool trader_powered() const;
    bool trade_antenna_enabled() const;
    bool call_trader(std::size_t pad);
    bool start_trader_sale();
    void update_trader(float dt);
    TradeManifest trader_available() const;
    double& trader_oxygen_stock(std::uint64_t sequence);
    double trader_oxygen_stock_amount(std::uint64_t sequence) const;
    bool trader_robot_busy(std::size_t robot) const;
    bool trader_placement_clear(eng::Vec3 position,float rotation,float width,float depth) const;
    bool trader_position_clear(eng::Vec3 position,float radius,std::size_t ignored_resource=kNoRobotIndex,std::size_t ignored_habitat=kNoRobotIndex) const;
    void record_robot_operation_result(BuilderRobotState& robot,bool success,std::uint64_t task=0);
    void render_robot_blockers(eng::Renderer& r);
    void update_ore_hauling(float dt,const std::function<bool(std::size_t,eng::Vec3)>& clear,
        const std::function<void(std::size_t,eng::Vec3)>& move,const std::function<void(std::size_t)>& stop);
    void update_trade_hauling(float dt,const std::function<bool(std::size_t,eng::Vec3)>& clear,
        const std::function<void(std::size_t,eng::Vec3)>& move,const std::function<void(std::size_t)>& stop);
    void render_trader(eng::Renderer& r);
    void render_pad_trade(eng::Renderer& r,std::size_t pad,eng::Rect panel,float scroll,float warning_extra);

    void render_lunar_lander_panel(eng::Renderer& r);
    bool lander_placement_clear(eng::Vec3 position,float rotation,float width,float depth) const;
    BuilderRobotState lander_robot(int ordinal) const;
    void update_builder_robot(float dt);
    float game_speed_multiplier() const;
    static const char* game_phase_name(GamePhase phase);
    static const char* game_phase_short_name(GamePhase phase);
    void reset_planning_game();
    void update_planning_game(float dt);
    bool accept_planning_contract(std::size_t index);
    bool unlock_planning_technology(std::size_t index);
    bool planning_contract_available(std::size_t index) const;
    bool deliver_planning_contract(std::size_t index);
    bool advance_planning_to_next_report();
    bool planning_technology_available(std::size_t index) const;
    ResourceBundle planning_reference_mission_cargo() const;
    ResourceBundle planning_structure_requirements() const;
    bool planning_cargo_covers_structures() const;
    void planning_add_required_cargo();
    std::int64_t planning_mission_payload_kg() const;
    std::int64_t planning_mission_payload_capacity_kg() const;
    std::int64_t planning_mission_cost_eur() const;
    bool planning_mission_requirements_met() const;
    bool purchase_planning_mission();
    bool complete_planning_mission_step();
    void update_resource_economy(float dt);
    void update_ore_logistics(float dt);
    bool ore_survey_location(eng::Vec3& point) const;
    void render_ore_deposits(eng::Renderer& r) const;
    void render_ore_survey(eng::Renderer& r) const;
    bool conveyor_route_clear(const std::vector<eng::Vec3>& path) const;
    GameProgressSnapshot game_progress_snapshot() const;
    int crew_quarters_capacity() const;
    eng::Rect progress_tree_panel_rect() const;
    const UtilityServiceSnapshot& update_supply_simulation(float dt);
    double refuel_landing_pad(std::size_t pad_index,bool rocket);
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
    bool spawn_produced_resource(MaterialResource resource,std::int64_t amount,eng::Vec3 source_position,bool defer_if_blocked=true);
    void enter_first_person_drop_mode();
    void exit_first_person_mode();
    void update_first_person(float dt,const eng::InputState& input);
    void update_first_person_resource_hover();
    void first_person_grab_or_drop_resource();
    void drop_first_person_carried_resource();
    bool store_first_person_carried_resource();
    void stored_resource_box_pose(const PlacedResource& structure,const ResourceMesh& mesh,std::size_t box_index,eng::Vec3& base,float& half_width,float& height) const;
    bool first_person_spawn_valid(eng::Vec3 p) const;
    enum class CrewTraversal { Player, Astronaut, Plan };
    bool first_person_move_allowed(eng::Vec3 from,eng::Vec3 to,CrewTraversal traversal=CrewTraversal::Player) const;
    bool airlock_powered(std::size_t index) const;
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
    template<class Item>
    void finalize_placement(Item& item){
        item.placement_sequence=next_blueprint_placement_sequence_++;
        if(build::debug_features && active_game_phase_==GamePhase::Debug && stage3_mode_!=Stage3Mode::Template){
            item.construction.finish();
            item.resource_plan.steps={};
        }
    }
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
    bool resume_save(const SaveGameEntry& entry);
    bool save_game_to_name(const std::string& name,std::string& error);
    bool load_game_from_file(const std::string& path,std::string& error);
    bool load_debug_blueprint_template_from_file(const std::string& path,std::string& error,bool completed=false);
    std::string debug_blueprint_template_path() const;
    float ui_width(const eng::Renderer& r) const { return (float)r.width()/ui_scale_; }
    float ui_height(const eng::Renderer& r) const { return (float)r.height()/ui_scale_; }
    eng::Rect centered_button(eng::Renderer& r,float y,float w=360,float h=52) const;
};
}
