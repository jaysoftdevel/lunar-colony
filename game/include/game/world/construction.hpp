#pragma once
#include <algorithm>
#include <cstdint>

namespace game {

inline constexpr std::uint8_t kConstructionSteps = 6;
inline constexpr float kConstructionStepSeconds = 1.0f;


inline constexpr std::uint8_t kCorridorConstructionSteps = 3;
inline constexpr std::uint8_t kSupplyConstructionSteps = 2;
inline constexpr std::uint8_t kWalkwayConstructionSteps = 2;

struct StagedConstructionState {
    std::uint8_t step{0};
    std::uint8_t total_steps{1};
    float step_elapsed{0.0f};

    explicit StagedConstructionState(std::uint8_t steps=1)
        : total_steps(std::max<std::uint8_t>(1,steps)) {}

    bool complete() const { return step >= total_steps; }
    float progress() const {
        const float total=static_cast<float>(std::max<std::uint8_t>(1,total_steps));
        if(complete()) return 1.0f;
        const float within=std::clamp(step_elapsed/std::max(0.001f,kConstructionStepSeconds),0.0f,0.999f);
        return std::clamp((static_cast<float>(step)+within)/total,0.0f,1.0f);
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
    float progress() const {
        if(complete()) return 1.0f;
        const float within=std::clamp(step_elapsed/std::max(0.001f,kConstructionStepSeconds),0.0f,0.999f);
        return std::clamp((static_cast<float>(step)+within)/static_cast<float>(kConstructionSteps),0.0f,1.0f);
    }

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
