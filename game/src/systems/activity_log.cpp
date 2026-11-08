#include "game/game_app.hpp"
#include "game/ui/supply_warnings.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace game {

const char* log_event_name(LogEvent event){
    switch(event){
        case LogEvent::Started: return "START";
        case LogEvent::Stopped: return "STOP";
        case LogEvent::Warning: return "WARNING";
        case LogEvent::Resolved: return "RESOLVED";
        case LogEvent::Incident: return "INCIDENT";
        case LogEvent::Note: return "EVENT";
    }
    return "EVENT";
}

std::string log_timestamp(double seconds,bool precise){
    const auto ms=static_cast<std::uint64_t>(std::max(0.0,seconds)*1000);
    const auto whole=ms/1000;
    std::ostringstream text;
    text<<std::setfill('0')<<std::setw(2)<<whole/3600<<':'<<std::setw(2)<<(whole/60)%60
        <<':'<<std::setw(2)<<whole%60;
    if(precise)text<<'.'<<std::setw(3)<<ms%1000;
    return text.str();
}

void ActivityLog::reset(double now,std::string message){
    *this=ActivityLog{};
    origin_=now;
    record(now,LogEvent::Note,std::move(message));
}

void ActivityLog::begin_sample(double now){
    elapsed_=std::max(elapsed_,now-origin_);
    ++sample_;
}

void ActivityLog::stop(const Active& active){
    entries_.push_back({elapsed_,active.warning?LogEvent::Resolved:LogEvent::Stopped,active.message});
}

void ActivityLog::end_sample(){
    for(auto it=active_.begin();it!=active_.end();){
        if(it->second.sample==sample_){++it;continue;}
        stop(it->second);
        it=active_.erase(it);
    }
}

void ActivityLog::record(double now,LogEvent event,std::string message){
    elapsed_=std::max(elapsed_,now-origin_);
    entries_.push_back({elapsed_,event,std::move(message)});
}

void GameApp::reset_activity_log(std::string message){
    activity_log_.reset(ui_time_seconds_,std::move(message));
    activity_log_offset_=0;
    activity_log_seen_count_=0;
    activity_log_export_status_.clear();
}

void GameApp::update_activity_log(){
    if(!game_session_active_ || screen_!=Screen::Playfield)return;
    using C=ActivityLog::Channel;
    using namespace world_labels;
    auto& log=activity_log_;
    log.begin_sample(ui_time_seconds_);
    const auto observe=[&](C channel,std::uint64_t id,std::uint64_t state,bool warning,const auto& describe,unsigned slot=0){
        log.observe({channel,id,slot},state,warning,describe);
    };

    const auto phase=static_cast<unsigned>(active_game_phase_);
    observe(C::Session,0,phase,false,[&]{return std::string(game_phase_name(active_game_phase_));});
    if(simulation_paused())observe(C::Session,1,0,false,[]{return "Simulation paused";});
    if(active_game_phase_==GamePhase::Planning){
        log_planning_activities();
        log.end_sample();
        return;
    }

    if(lander_.active()){
        const unsigned state=lander_.awaiting_site?100u:lander_.awaiting_stash?101u:static_cast<unsigned>(lander_.phase);
        observe(C::Lander,0,state,false,[&]{return std::string("Lander / ")+(lander_.awaiting_site?"SELECTING LANDING SITE":
            lander_.awaiting_stash?"SELECTING CARGO DEPOT":lunar_lander_phase_name(lander_));});
        if(lander_.salvage_requested)observe(C::LanderSalvage,0,0,false,[]{return "Lander salvage";});
    }
    if(trader_.active()){
        observe(C::Trader,trader_.pad_sequence,static_cast<unsigned>(trader_.phase),false,[&]{
            return std::string("Trader / ")+trade_phase_name(trader_.phase);});
        if(trader_.oxygen_packaging)observe(C::TraderPacking,0,0,false,[]{return "Packaging oxygen for export";});
        for(const auto& haul:trader_.hauls){
            observe(C::TradeHaul,haul.robot+1,haul.phase,false,[&]{
                const char* phase_name=haul.phase==0?"collecting export cargo":haul.phase==1?"delivering export cargo":"clearing loading bay";
                return "Rover "+std::to_string(haul.robot+1)+" / "+phase_name;
            });
        }
    }

    for(std::size_t i=0;i<builder_robots_.size();++i){
        const auto& robot=builder_robots_[i];
        const auto id=i+1;
        const auto target=construction_target_sequence(robot.target);
        const auto label=[&]{return std::string(robot_role_name(robot.role))+" #"+std::to_string(id);};
        const auto task_label=[&](const char* task){
            return label()+" / "+task+(robot.target.valid()?" / "+construction_target_name(robot.target):"");
        };
        if(robot.mode!=BuilderRobotMode::Idle){
            observe(C::RoverTask,id,target*16+static_cast<unsigned>(robot.mode),false,[&]{
                return task_label(builder_robot_mode_name(robot.mode));});
        }
        if(robot.action!=BuilderRobotAction::None){
            observe(C::RoverAction,id,target*16+static_cast<unsigned>(robot.action),false,[&]{
                return task_label(builder_robot_action_name(robot.action));});
        }
        if(robot.route_active || robot.workshop_returning || robot.debug_move_active || robot.ore_haul.active()){
            const unsigned travel=robot.workshop_returning?1:robot.debug_move_active?2:robot.ore_haul.active()?
                (robot.ore_haul.loaded?4:3):0;
            constexpr const char* names[]{"travelling","returning to workshop","manual movement","collecting ore","hauling ore"};
            observe(C::RoverTravel,id,travel,false,[&]{return label()+" / "+names[travel];});
        }
        if(robot.battery_charging || robot.workshop_parked)
            observe(C::RoverCharge,id,robot.battery_charging,false,[&]{return label()+
                (robot.battery_charging?" / charging":" / parked at workshop");});
        const auto mode=robot.safety.mode;
        const auto reason=robot.safety.block_reason;
        if(mode==robots::SafetyMode::Fault || mode==robots::SafetyMode::Blocked || robot.safety.route_failed){
            observe(C::RoverProblem,id,static_cast<unsigned>(mode)*32+static_cast<unsigned>(reason),true,[&]{
                return label()+" / "+(robot.safety.route_failed && reason==robots::BlockReason::None?"NO SAFE ROUTE":
                    robots::safety_mode_name(mode))+(reason==robots::BlockReason::None?"":std::string(" / ")+robots::block_reason_name(reason));});
        }
        if(robot.route_waiting_for_clearance)
            observe(C::RoverTraffic,id,0,true,[&]{return label()+" / waiting for route clearance";});
        if(robot.operation_retry.relocation_pending())
            observe(C::RoverProblem,id,0,true,[&]{return label()+" / relocating a blocked operation";},1);
        if(robot.battery_charge_kwh<robot.battery_capacity_kwh*.1f)
            observe(C::RoverBattery,id,robot.battery_charge_kwh<=0.f,true,[&]{return label()+
                (robot.battery_charge_kwh<=0.f?" / battery depleted":" / battery below 10%");});
    }

    if(astronaut_test_active())for(std::size_t i=0;i<test_astronauts_.size();++i){
        const auto& actor=test_astronauts_[i];
        const auto label=[&]{return std::string(kAstronautRoles[i].name);};
        constexpr const char* activities[]{"idle","manual movement","seeking shelter","profession duty","heading to bed",
            "sleeping","exhausted","resupplying suit air","construction duty"};
        if(actor.health>0.f && actor.activity!=AstronautActivity::Idle){
            const auto task=static_cast<unsigned>(actor.activity);
            observe(C::CrewTask,i,actor.construction_job*32+task*2+actor.destination.has_value(),false,[&]{
                return label()+" / "+activities[task]+(actor.construction_job?" / blueprint #"+std::to_string(actor.construction_job):"");});
        }
        if(actor.health>0.f){
            observe(C::CrewSuit,i,actor.suited,false,[&]{return label()+(actor.suited?" / wearing EVA suit":" / wearing habitat coveralls");});
            if(actor.waiting_for_door)observe(C::CrewDoor,i,0,true,[&]{return label()+" / waiting at pressure door";});
            if(actor.suited && actor.suit_oxygen_kg<kSuitOxygenCapacityKg*.1)
                observe(C::CrewAir,i,actor.suit_oxygen_kg<=0.0,true,[&]{return label()+
                    (actor.suit_oxygen_kg<=0.0?" / suit air depleted":" / suit air below 10%");});
            if(actor.hunger>=.8f)observe(C::CrewNeeds,i,0,true,[&]{return label()+" / high hunger";},0);
            if(actor.thirst>=.8f)observe(C::CrewNeeds,i,0,true,[&]{return label()+" / high thirst";},1);
            if(actor.activity==AstronautActivity::Knockout)observe(C::CrewNeeds,i,0,true,[&]{return label()+" / exhaustion knockout";},2);
            if(actor.destination && !actor.waiting_for_door && actor.route.empty() && actor.retry_seconds>0.f)
                observe(C::CrewDoor,i,0,true,[&]{return label()+" / no available walking route";},1);
        }
        if(actor.health<1.f)observe(C::CrewHealth,i,actor.health<=0.f,true,[&]{return label()+
            (actor.health<=0.f?" / deceased":" / health below full");});
    }

    const auto observe_builds=[&](const auto& items,ConstructionTargetKind kind,C channel){
        for(std::size_t i=0;i<items.size();++i){
            const auto& item=items[i];
            const auto id=item.placement_sequence;
            if(!id)continue;
            const ConstructionTargetRef target{kind,i};
            if(item.construction.complete()){
                if(log.active({channel,id,0}))log.record(ui_time_seconds_,LogEvent::Note,
                    construction_target_name(target)+" / construction complete");
                continue;
            }
            const auto name=[&]{return construction_target_name(target);};
            observe(channel,id,0,false,[&]{return name()+" / blueprint queued";});
            if(!construction_target_dependencies_ready(target)){
                observe(channel,id,0,true,[&]{return name()+" / waiting for prerequisites";},1);
                continue;
            }
            const auto step=item.construction.step;
            const auto& cost=item.resource_plan.next_step_cost(step);
            const bool missing=std::any_of(kConstructionMaterialResources.begin(),kConstructionMaterialResources.end(),[&](auto m){
                return cost.amount(m)>item.site_buffer.amount(m);
            });
            if(missing)observe(channel,id,step,true,[&]{return name()+" / waiting for stage "+
                std::to_string(step+1)+" materials";},2);
            if(!missing && item.construction.step_elapsed>0.f)
                observe(channel,id,step,false,[&]{return name()+" / construction stage "+std::to_string(step+1);},3);
        }
    };
    observe_builds(placed_habitats_,ConstructionTargetKind::Habitat,C::HabitatBuild);
    observe_builds(placed_resources_,ConstructionTargetKind::Resource,C::ResourceBuild);
    observe_builds(corridor_network_.connections(),ConstructionTargetKind::Corridor,C::CorridorBuild);
    observe_builds(utility_network_.connections(),ConstructionTargetKind::Utility,C::UtilityBuild);
    observe_builds(walkway_network_.connections(),ConstructionTargetKind::Walkway,C::WalkwayBuild);

    const auto observe_supply=[&](const auto& item,C channel,const auto& name,ResourceKind kind){
        if(!item.construction.complete() || !item.placement_sequence)return;
        if(!item.power_enabled){
            observe(channel,item.placement_sequence,0,false,[&]{return name()+" / switched off";},0);
            return;
        }
        for(const auto& reason:supply_warning_reasons(kind)){
            if(item.supply.warnings&reason.flag)
                observe(channel,item.placement_sequence,0,true,[&]{return name()+" / "+reason.text;},reason.flag);
        }
    };
    for(const auto& h:placed_habitats_){
        const auto name=[&]{return habitat_instance_name(h);};
        observe_supply(h,C::HabitatSupply,name,ResourceKind::Count);
        if(h.supply.oxygen_produced_kg_per_day>0.)
            observe(C::Production,h.placement_sequence,0,false,[&]{return name()+" / producing oxygen";});
        if(h.supply.hydrogen_vented_kg_per_day>0.)
            observe(C::Production,h.placement_sequence,0,true,[&]{return name()+" / venting surplus hydrogen";},1);
    }
    for(const auto& item:placed_resources_){
        const auto name=[&]{return resource_instance_name(item);};
        observe_supply(item,C::ResourceSupply,name,item.kind);
        if(item.supply.hydrogen_power_kw>0.)
            observe(C::Production,item.placement_sequence,0,false,[&]{return name()+" / generating fuel-cell power";});
    }
    const auto& channels=utility_network_.connections();
    for(std::size_t i=0;i<channels.size();++i){
        const auto& belt=channels[i];
        if(belt.kind!=UtilityKind::ConveyorBelt || !belt.construction.complete() || belt.ore_cargo==ore::no_cargo)continue;
        const auto name=[&]{return construction_target_name({ConstructionTargetKind::Utility,i});};
        observe(C::Belt,belt.placement_sequence,belt.ore_cargo,false,[&]{return name()+" / carrying ore";});
        if(belt.ore_distance_m>=utility_network_.connection_length(i))
            observe(C::Belt,belt.placement_sequence,0,true,[&]{return name()+" / receiving outlet blocked";},1);
    }
    for(const auto& cycle:airlock_cycles_){
        if(cycle.sealing || cycle.equalizing){
            const unsigned state=cycle.sealing?0u:cycle.target>=.5f?1u:2u;
            constexpr const char* states[]{"sealing","equalizing to cabin","equalizing to exterior"};
            observe(C::Airlock,cycle.identity,state,false,[&]{return "Air lock #"+std::to_string(cycle.identity)+" / "+states[state];});
        }
    }
    log.end_sample();
}

void GameApp::export_activity_log(){
    namespace fs=std::filesystem;
    const fs::path dir(save_game_directory());
    std::error_code error;
    fs::create_directories(dir,error);
    const auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto path=dir/("activity-"+std::to_string(stamp)+".txt");
    auto temp=path;
    temp+=".tmp";
    bool written=false;
    if(!error){
        std::ofstream out(temp,std::ios::binary);
        if(out){
            out<<"Lunar Frontier v"<<version::current<<" activity log\n"
               <<"Time is elapsed session time (HH:MM:SS.mmm). START or WARNING without a closing entry is still active.\n\n";
            for(const auto& entry:activity_log_.entries())
                out<<log_timestamp(entry.seconds,true)<<"  "<<log_event_name(entry.event)<<"  "<<entry.message<<'\n';
            out.close();
            if(out){
                fs::rename(temp,path,error);
                written=!error;
            }
        }
    }
    if(!written){
        fs::remove(temp,error);
        activity_log_export_status_="EXPORT FAILED / SAVEGAME FOLDER";
        return;
    }
    activity_log_export_status_="EXPORTED "+path.filename().string();
}

} // namespace game
