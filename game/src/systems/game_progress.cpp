#include "game/game_app.hpp"
#include "game/systems/game_progress.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <iterator>

namespace game {
namespace {
struct RequirementDefinition {
    ProgressRequirementKind kind;
    const char* label;
};
struct StageDefinition {
    int number;
    const char* title;
    const RequirementDefinition* requirements;
    std::size_t count;
};

constexpr RequirementDefinition kStage1[] = {
    {ProgressRequirementKind::SolarCollectorBuilt,"Build a solar collector"},
    {ProgressRequirementKind::BatteryBuilt,"Build a battery"},
    {ProgressRequirementKind::WaterDrillerBuilt,"Build a water driller"},
    {ProgressRequirementKind::WaterStorageBuilt,"Build a water storage"},
    {ProgressRequirementKind::CoreResourceNetworkConnected,"Connect all Stage 1 resource structures"}
};
constexpr RequirementDefinition kStage2[] = {
    {ProgressRequirementKind::AirLockBuilt,"Build an air-lock (first inside structure)"},
    {ProgressRequirementKind::OxygenGeneratorBuilt,"Build an oxygen generator"},
    {ProgressRequirementKind::ResourceToHabitatConnected,"Connect the resource network to the inside base"}
};
constexpr RequirementDefinition kStage3[] = {
    {ProgressRequirementKind::CanteenBuilt,"Build a canteen"},
    {ProgressRequirementKind::DormCapacitySix,"Build dorms for at least 6 people"},
    {ProgressRequirementKind::LandingPadBuilt,"Build a landing pad"},
    {ProgressRequirementKind::PressurizedInteriors,"Fill all completed interiors with breathable air"}
};

// The progression model is deliberately table-driven. Further stages only need
// another RequirementDefinition array and one StageDefinition entry; runtime
// evaluation stays centralized below.
constexpr StageDefinition kStages[] = {
    {1,"Resource independence",kStage1,std::size(kStage1)},
    {2,"Pressurized base",kStage2,std::size(kStage2)},
    {3,"Crew preparation",kStage3,std::size(kStage3)}
};

int quarters_capacity(HabitatSizeTier size){
    switch(size){
        case HabitatSizeTier::Small:return 2;
        case HabitatSizeTier::Standard:return 4;
        case HabitatSizeTier::Mid:return 6;
        case HabitatSizeTier::Big:return 8;
        default:return 0;
    }
}
}

GameProgressSnapshot GameApp::game_progress_snapshot() const {
    GameProgressSnapshot out;
    const auto complete_resource=[&](ResourceKind kind){
        return std::any_of(placed_resources_.begin(),placed_resources_.end(),[&](const PlacedResource& r){
            return r.kind==kind && r.construction.complete();
        });
    };
    const auto complete_habitat=[&](HabitatKind kind){
        return std::any_of(placed_habitats_.begin(),placed_habitats_.end(),[&](const PlacedHabitat& h){
            return h.kind==kind && h.construction.complete();
        });
    };

    const UtilityServiceSnapshot* services=&last_supply_services_;
    UtilityServiceSnapshot refreshed;
    if(services->resources.size()!=placed_resources_.size() || services->habitats.size()!=placed_habitats_.size()){
        refreshed=utility_network_.evaluate_services(corridor_network_,placed_habitats_,placed_resources_);
        services=&refreshed;
    }

    const auto component_with_resources=[&](){
        struct Flags {bool solar=false,battery=false,driller=false,tank=false;};
        std::vector<std::pair<std::size_t,Flags>> components;
        for(std::size_t i=0;i<placed_resources_.size() && i<services->resources.size();++i){
            const auto& r=placed_resources_[i];
            const auto& s=services->resources[i];
            if(!r.construction.complete() || !s.connected || s.component==kNoUtilityComponent)continue;
            auto it=std::find_if(components.begin(),components.end(),[&](const auto& v){return v.first==s.component;});
            if(it==components.end()){components.push_back({s.component,{}});it=std::prev(components.end());}
            switch(r.kind){
                case ResourceKind::SolarPanel:it->second.solar=true;break;
                case ResourceKind::PowerStorage:it->second.battery=true;break;
                case ResourceKind::WaterDriller:it->second.driller=true;break;
                case ResourceKind::WaterStorage:it->second.tank=true;break;
                default:break;
            }
        }
        for(const auto& [component,f]:components)if(f.solar&&f.battery&&f.driller&&f.tank)return component;
        return kNoUtilityComponent;
    };
    const std::size_t core_component=component_with_resources();

    const auto resource_to_habitat_connected=[&](){
        for(std::size_t hi=0;hi<placed_habitats_.size() && hi<services->habitats.size();++hi){
            if(!placed_habitats_[hi].construction.complete())continue;
            const auto& hs=services->habitats[hi];
            if(!hs.connected || hs.component==kNoUtilityComponent)continue;
            for(std::size_t ri=0;ri<placed_resources_.size() && ri<services->resources.size();++ri){
                if(!placed_resources_[ri].construction.complete())continue;
                const auto& rs=services->resources[ri];
                if(rs.connected && rs.component==hs.component)return true;
            }
        }
        return false;
    };

    const auto dorm_capacity=[&](){
        int total=0;
        for(const auto& h:placed_habitats_)if(h.kind==HabitatKind::CrewQuarters && h.construction.complete())total+=quarters_capacity(h.size);
        return total;
    };
    const auto all_interiors_pressurized=[&](){
        bool any=false;
        for(const auto& h:placed_habitats_){
            if(!h.construction.complete())continue;
            any=true;
            if(h.supply.breathability()<95.0)return false;
        }
        for(const auto& r:placed_resources_){
            if(r.kind!=ResourceKind::StorageDome || !r.construction.complete())continue;
            any=true;
            if(r.supply.breathability()<95.0)return false;
        }
        return any;
    };

    const auto requirement_complete=[&](ProgressRequirementKind kind){
        switch(kind){
            case ProgressRequirementKind::SolarCollectorBuilt:return complete_resource(ResourceKind::SolarPanel);
            case ProgressRequirementKind::BatteryBuilt:return complete_resource(ResourceKind::PowerStorage);
            case ProgressRequirementKind::WaterDrillerBuilt:return complete_resource(ResourceKind::WaterDriller);
            case ProgressRequirementKind::WaterStorageBuilt:return complete_resource(ResourceKind::WaterStorage);
            case ProgressRequirementKind::CoreResourceNetworkConnected:return core_component!=kNoUtilityComponent;
            case ProgressRequirementKind::AirLockBuilt:return complete_habitat(HabitatKind::AirLock);
            case ProgressRequirementKind::OxygenGeneratorBuilt:return complete_habitat(HabitatKind::OxygenGeneratorDome);
            case ProgressRequirementKind::ResourceToHabitatConnected:return resource_to_habitat_connected();
            case ProgressRequirementKind::CanteenBuilt:return complete_habitat(HabitatKind::CantineDome);
            case ProgressRequirementKind::DormCapacitySix:return dorm_capacity()>=6;
            case ProgressRequirementKind::LandingPadBuilt:return complete_resource(ResourceKind::LandingPad);
            case ProgressRequirementKind::PressurizedInteriors:return all_interiors_pressurized();
        }
        return false;
    };

    out.stages.reserve(std::size(kStages));
    bool prior_complete=true;
    for(const auto& definition:kStages){
        ProgressStageState stage;
        stage.number=definition.number;
        stage.title=definition.title;
        stage.complete=true;
        stage.items.reserve(definition.count);
        for(std::size_t i=0;i<definition.count;++i){
            const auto& req=definition.requirements[i];
            const bool complete=requirement_complete(req.kind);
            stage.items.push_back({req.kind,req.label,complete});
            stage.complete=stage.complete&&complete;
        }
        if(prior_complete && !stage.complete)out.active_stage=stage.number;
        prior_complete=prior_complete&&stage.complete;
        out.stages.push_back(std::move(stage));
    }
    out.ready_for_human_crew=prior_complete;
    if(out.ready_for_human_crew)out.active_stage=(int)std::size(kStages);
    return out;
}

int GameApp::crew_quarters_capacity() const {
    int capacity=0;
    for(const auto& h:placed_habitats_)if(h.kind==HabitatKind::CrewQuarters && h.construction.complete())capacity+=quarters_capacity(h.size);
    return capacity;
}

} // namespace game
