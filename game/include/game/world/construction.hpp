#pragma once
#include <algorithm>
#include <cstdint>

namespace game {

inline constexpr std::uint8_t kConstructionSteps = 10;
inline constexpr float kConstructionStepSeconds = 1.0f;


inline constexpr std::uint8_t kCorridorConstructionSteps = 5;
inline constexpr std::uint8_t kSupplyConstructionSteps = 3;
inline constexpr std::uint8_t kWalkwayConstructionSteps = 3;

struct StagedConstructionState {
    std::uint8_t step{0};
    std::uint8_t total_steps{1};
    float step_elapsed{0.0f};

    explicit StagedConstructionState(std::uint8_t steps=1)
        : total_steps(std::max<std::uint8_t>(1,steps)) {}

    bool complete() const { return step >= total_steps; }
    float progress() const {
        return static_cast<float>(step) / static_cast<float>(std::max<std::uint8_t>(1,total_steps));
    }

    bool update(float dt){
        return update_gated(dt,[](std::uint8_t){ return true; });
    }

    template<class StepGate>
    bool update_gated(float dt,StepGate&& gate){
        if(complete() || dt <= 0.0f) return false;
        step_elapsed += dt;
        bool changed=false;
        while(step_elapsed >= kConstructionStepSeconds && step < total_steps){
            if(!gate(static_cast<std::uint8_t>(step+1))){
                // Keep one full interval queued so construction resumes as soon
                // as the required material bundle becomes available.
                step_elapsed=kConstructionStepSeconds;
                break;
            }
            step_elapsed -= kConstructionStepSeconds;
            ++step;
            changed=true;
        }
        if(complete()) step_elapsed=0.0f;
        return changed;
    }
};

struct ConstructionState {
    std::uint8_t step{0};
    float step_elapsed{0.0f};

    bool complete() const { return step >= kConstructionSteps; }
    float progress() const { return static_cast<float>(step) / static_cast<float>(kConstructionSteps); }

    bool update(float dt){
        return update_gated(dt,[](std::uint8_t){ return true; });
    }

    template<class StepGate>
    bool update_gated(float dt,StepGate&& gate){
        if(complete() || dt <= 0.0f) return false;
        step_elapsed += dt;
        bool changed=false;
        while(step_elapsed >= kConstructionStepSeconds && step < kConstructionSteps){
            if(!gate(static_cast<std::uint8_t>(step+1))){
                step_elapsed=kConstructionStepSeconds;
                break;
            }
            step_elapsed -= kConstructionStepSeconds;
            ++step;
            changed=true;
        }
        if(complete()) step_elapsed=0.0f;
        return changed;
    }
};

} // namespace game
