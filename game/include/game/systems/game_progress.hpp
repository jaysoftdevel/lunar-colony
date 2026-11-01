#pragma once
#include <string>
#include <vector>

namespace game {

enum class ProgressRequirementKind {
    SolarCollectorBuilt,
    BatteryBuilt,
    WaterDrillerBuilt,
    WaterStorageBuilt,
    CoreResourceNetworkConnected,
    AirLockBuilt,
    OxygenGeneratorBuilt,
    ResourceToHabitatConnected,
    CanteenBuilt,
    DormCapacitySix,
    LandingPadBuilt,
    PressurizedInteriors
};

struct ProgressChecklistItem {
    ProgressRequirementKind requirement{ProgressRequirementKind::SolarCollectorBuilt};
    const char* label{""};
    bool complete{false};
};

struct ProgressStageState {
    int number{0};
    const char* title{""};
    std::vector<ProgressChecklistItem> items{};
    bool complete{false};
};

struct GameProgressSnapshot {
    std::vector<ProgressStageState> stages{};
    int active_stage{1};
    bool ready_for_human_crew{false};
};

} // namespace game
