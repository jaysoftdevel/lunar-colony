#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/world/geometry_standards.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using runtime::Vec3;
namespace {
constexpr Vec3 landing_site{-24.f,0.f,-13.f};
constexpr Vec3 depot_site{-11.f,0.f,-13.f};
constexpr auto compact_habitat=HabitatSizeTier::Small;
constexpr auto small_equipment=ResourceSizeTier::Small;
constexpr auto full_equipment=ResourceSizeTier::Standard;
constexpr std::size_t solar=0,battery=1,driller=2,tank=3;
constexpr std::size_t oxygen_index=0,airlock_index=1;
}

void GameApp::start_auto_run(){
    if(new_game_mode_selection_!=NewGameMode::FreePlay ||
       new_game_phase_selection_!=GamePhase::RobotPreparation)return;
    debug_start_template_enabled_=false;
    // Reserve room for the starter layout, deployment apron and depot.
    playfield_size_=std::max(playfield_size_,96);
    start_new_game();
    game_speed_=GameSpeed::Fast;
    last_running_speed_=3;
    if(confirm_lander_site(landing_site)){
        set_auto_run_phase(AutoRunPhase::Landing,"AUTO RUN / LANDER APPROACH");
    }else{
        set_auto_run_phase(AutoRunPhase::Finished,"AUTO RUN / LANDING SITE BLOCKED");
    }
}

void GameApp::set_auto_run_phase(AutoRunPhase phase,const char* message){
    auto_run_.phase=phase;
    auto_run_.message=message;
    auto_run_.notice_until=ui_time_seconds_+12.0;
    activity_log_.record(ui_time_seconds_,LogEvent::Note,message);
}

bool GameApp::queue_auto_run_base(){
    // Auto Run starts from a fresh field. Preserve any manual placements made
    // during the deployment and leave their construction queue intact.
    if(!placed_resources_.empty()||!placed_habitats_.empty()||
       !corridor_network_.connections().empty()||!utility_network_.connections().empty()||
       !walkway_network_.connections().empty())return false;
    const auto first=next_blueprint_placement_sequence_;
    const auto place_equipment=[&](ResourceKind kind,ResourceSizeTier size,Vec3 at){
        PlacedResource item;
        item.kind=kind;
        item.size=size;
        item.position=at;
        if(!resource_placement_valid(kind,size,item.storage_size,at,0.f))return false;
        item.resource_plan=construction_costs::structure(kind,resource_models_.mesh(item));
        finalize_placement(item);
        placed_resources_.push_back(std::move(item));
        return true;
    };
    const auto place_room=[&](HabitatKind kind,Vec3 at){
        if(!placement_valid(kind,compact_habitat,at,0.f))return false;
        PlacedHabitat room;
        room.kind=kind;
        room.size=compact_habitat;
        room.position=at;
        room.resource_plan=construction_costs::habitat(kind,room.size,habitat_models_.mesh(room));
        finalize_placement(room);
        placed_habitats_.push_back(std::move(room));
        return true;
    };
    const auto equipment_port=[&](std::size_t index){
        return UtilityBuildAnchor{UtilityEndpointKind::Resource,index,placed_resources_[index].position};
    };
    const auto supply=[&](UtilityBuildAnchor a,UtilityBuildAnchor b){
        auto route=utility_network_.preview_connection(UtilityKind::SupplyChannel,a,b,
            placed_habitats_,habitat_models_,placed_resources_,resource_models_);
        if(!route||!gate_route_clear(route->path,geometry::kSupplyTrayWidth*.5f+.2f))return false;
        route->resource_plan=construction_costs::supply_channel(route->path,route->junctions.size());
        finalize_placement(*route);
        utility_network_.append_connection(std::move(*route));
        return true;
    };

    // Sites sit across the depot aisle. A straight, close pressure connection
    // minimizes corridor construction and the volume the oxygen plant must fill.
    const Vec3 generator_site{10.f,0.f,17.f};
    const auto& generator_mesh=habitat_models_.mesh(HabitatKind::OxygenGeneratorDome,compact_habitat);
    const auto& airlock_mesh=habitat_models_.mesh(HabitatKind::AirLock,compact_habitat);
    const float separation=std::ceil(.5f*(geometry::structure_ground_extent(generator_mesh.footprint_x)+
        geometry::structure_ground_extent(airlock_mesh.footprint_x))+geometry::kStructurePlacementGap);
    const Vec3 airlock_site=generator_site+Vec3{separation,0.f,0.f};
    const auto corridor=[&]{
        const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,airlock_index,generator_site};
        const CorridorBuildAnchor b{CorridorEndpointKind::Habitat,oxygen_index,airlock_site};
        auto route=corridor_network_.preview_built_corridor(a,b,placed_habitats_,habitat_models_,
            placed_resources_,resource_models_);
        if(!route||!corridor_gate_layout_clear(*route))return false;
        route->resource_plan=construction_costs::corridor(route->path);
        finalize_placement(*route);
        corridor_network_.append_corridor(std::move(*route));
        return true;
    };

    // The driller can feed the oxygen plant from its buffer, so tank work
    // can follow the critical path. The generator starts filling its dome
    // while the airlock and short corridor are under construction.
    if(!place_equipment(ResourceKind::SolarPanel,full_equipment,{10.f,0.f,-17.f}) ||
       !place_equipment(ResourceKind::PowerStorage,full_equipment,{10.f,0.f,0.f}) ||
       !supply(equipment_port(solar),equipment_port(battery)) ||
       !place_equipment(ResourceKind::WaterDriller,small_equipment,{28.f,0.f,0.f}) ||
       !supply(equipment_port(driller),equipment_port(battery)) ||
       !place_room(HabitatKind::OxygenGeneratorDome,generator_site) ||
       !supply({UtilityEndpointKind::Habitat,oxygen_index,generator_site},equipment_port(battery)) ||
       !place_room(HabitatKind::AirLock,airlock_site) || !corridor() ||
       !place_equipment(ResourceKind::WaterStorage,small_equipment,{28.f,0.f,-17.f}) ||
       !supply(equipment_port(tank),equipment_port(driller))){
        placed_resources_.clear();
        placed_habitats_.clear();
        corridor_network_.clear();
        utility_network_.clear();
        next_blueprint_placement_sequence_=first;
        return false;
    }
    // Give the water rig the first delivery. Its longest material haul can
    // overlap solar and battery work, and its feed can commission as soon as
    // power is available. Preserve the rest of the dependency order.
    auto queue=construction_queue();
    const auto drill_job=std::find_if(queue.begin(),queue.end(),[&](const auto& job){
        return job.sequence==placed_resources_[driller].placement_sequence;
    });
    if(drill_job!=queue.end())std::rotate(queue.begin(),drill_job,drill_job+1);
    set_construction_order(queue);
    auto_run_.first_blueprint=first;
    auto_run_.end_blueprint=next_blueprint_placement_sequence_;
    return true;
}

void GameApp::update_auto_run(){
    if(active_game_mode_!=NewGameMode::FreePlay||active_game_phase_!=GamePhase::RobotPreparation)return;
    switch(auto_run_.phase){
        case AutoRunPhase::Landing:
            if(lander_.phase!=LanderPhase::Landed)return;
            if((lander_.awaiting_stash && !confirm_stash_site(depot_site))||!unload_lunar_lander()){
                set_auto_run_phase(AutoRunPhase::Finished,"AUTO RUN / CARGO DEPOT BLOCKED");
                return;
            }
            set_auto_run_phase(AutoRunPhase::Unloading,"AUTO RUN / DEPLOYING ROVERS");
            return;
        case AutoRunPhase::Unloading:
            if(lander_.phase!=LanderPhase::Ready)return;
            if(!queue_auto_run_base()){
                set_auto_run_phase(AutoRunPhase::Finished,"AUTO RUN / STARTER LAYOUT BLOCKED");
                return;
            }
            set_auto_run_phase(AutoRunPhase::Building,"AUTO RUN / BUILDING STARTER BASE");
            return;
        case AutoRunPhase::Building:
        case AutoRunPhase::Pressurizing: {
            const auto part_of_run=[&](const auto& item){
                return item.placement_sequence>=auto_run_.first_blueprint&&
                       item.placement_sequence<auto_run_.end_blueprint;
            };
            std::uint64_t remaining=auto_run_.end_blueprint-auto_run_.first_blueprint;
            bool pending=false,breathable=true;
            const auto inspect=[&](const auto& items,bool pressure){
                for(const auto& item:items)if(part_of_run(item)){
                    --remaining;
                    pending|=!item.construction.complete();
                    breathable&=!pressure||item.supply.breathability()>=kCrewBreathableAir;
                }
            };
            inspect(placed_resources_,false);
            inspect(placed_habitats_,true);
            inspect(corridor_network_.connections(),true);
            inspect(utility_network_.connections(),false);
            if(remaining){
                set_auto_run_phase(AutoRunPhase::Finished,"AUTO RUN / BLUEPRINT REMOVED");
            }else if(!pending){
                if(breathable){
                    set_auto_run_phase(AutoRunPhase::Finished,"AUTO RUN / BASE PRESSURIZED");
                    if(game_speed_==GameSpeed::Fast)game_speed_=GameSpeed::Normal;
                }else if(auto_run_.phase==AutoRunPhase::Building){
                    set_auto_run_phase(AutoRunPhase::Pressurizing,"AUTO RUN / PRESSURIZING BASE");
                }
            }
            return;
        }
        case AutoRunPhase::Inactive:
        case AutoRunPhase::Finished:
            return;
    }
}

} // namespace game
