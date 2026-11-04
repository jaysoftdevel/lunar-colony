#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include "game/systems/resource_management.hpp"

namespace game {

constexpr std::size_t kPlanningTechnologyCount=16;
constexpr std::size_t kPlanningContractCount=32;
constexpr std::size_t kPlanningMaxActiveContracts=3;

// Site is appended as value 3 so schema-26 saves keep their existing view
// values (Contracts=0, Technology=1, Mission=2) unchanged.
enum class PlanningView : std::uint8_t { Contracts=0, Technology=1, Mission=2, Site=3 };
enum class PlanningContractStatus : std::uint8_t { Available=0, Active=1, Completed=2 };

// Stage-1 mission preparation is an ordered facility workflow.  A mission can
// only advance when the task owned by the current facility is complete.
enum class MissionPreparationStep : std::uint8_t {
    Research=0, Design=1, Test=2, Refine=3, Plan=4, Confirm=5, Build=6,
    Integrate=7, FinalConfirm=8, Launch=9, Complete=10
};

enum class MissionFacility : std::uint8_t {
    Overview=0, ProgramOffice=1, ResearchEngineering=2, VehicleAssembly=3,
    CargoIntegration=4, MissionControl=5, Communications=6, PropulsionTest=7,
    LaunchComplex=8
};

struct PlanningContractProgress {
    PlanningContractStatus status{PlanningContractStatus::Available};
    double progress_days{0.0};
    std::uint16_t completed_count{0};
};

struct PlanningGameState {
    std::int64_t funds_eur{1'500'000};
    double elapsed_days{0.0};
    std::uint8_t time_speed{1}; // 0 pause, 1 normal, 2 fast, 3 very fast
    PlanningView view{PlanningView::Site};
    std::array<std::uint8_t,kPlanningTechnologyCount> technologies{};
    std::array<PlanningContractProgress,kPlanningContractCount> contracts{};
    // Mission-planning state. Rocket and lander selections are indexes into the
    // Stage-1 planning catalog; lunar cargo is stored in kilograms using the
    // same material vocabulary as Stage 2 so the manifest can later become a
    // direct hand-off to the robotic-base stage.
    std::uint8_t mission_rocket{1};
    std::uint8_t mission_lander{1};
    ResourceBundle mission_cargo{};
    bool mission_paid{false};
    std::int64_t mission_paid_cost_eur{0};
    MissionPreparationStep mission_step{MissionPreparationStep::Research};
    MissionFacility mission_facility{MissionFacility::Overview};
};

inline PlanningGameState default_planning_game_state(){ return {}; }

} // namespace game
