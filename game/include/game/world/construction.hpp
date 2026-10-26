#pragma once
#include <algorithm>
#include <cstdint>

namespace game {

inline constexpr std::uint8_t kConstructionSteps = 10;
inline constexpr float kConstructionStepSeconds = 1.0f;


inline constexpr std::uint8_t kCorridorConstructionSteps = 5;
inline constexpr std::uint8_t kSupplyConstructionSteps = 3;

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
        if(complete() || dt <= 0.0f) return false;
        step_elapsed += dt;
        bool changed=false;
        while(step_elapsed >= kConstructionStepSeconds && step < total_steps){
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
        if(complete() || dt <= 0.0f) return false;
        step_elapsed += dt;
        bool changed=false;
        while(step_elapsed >= kConstructionStepSeconds && step < kConstructionSteps){
            step_elapsed -= kConstructionStepSeconds;
            ++step;
            changed=true;
        }
        if(complete()) step_elapsed=0.0f;
        return changed;
    }
};

} // namespace game
