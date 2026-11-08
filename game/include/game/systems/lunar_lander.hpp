#pragma once
#include "engine/math.hpp"
#include "game/systems/resource_management.hpp"
#include "game/world/hydroponics.hpp"
#include <cstdint>

namespace game {
enum class LanderPhase : std::uint8_t {
    Inactive, Approach, Settling, Landed, OpeningRamp, DeployingRobots, UnloadingCargo, Ready
};
struct LunarLanderState {
    LanderPhase phase{LanderPhase::Inactive};
    float elapsed{0};
    eng::Vec3 site{};
    ResourceBundle manifest{}, remaining{};
    CropSeeds seeds_aboard{};
    std::uint8_t builders{1}, carriers{1};
    eng::Vec3 stash_center{};
    bool awaiting_site{false},awaiting_stash{false},salvaged{false},boxed{false};
    bool legacy_parking{false};
    bool salvage_requested{false};
    int salvage_worker{-1};
    std::uint8_t salvage_port{0};
    float salvage_seconds{0};
    bool salvage_uses_robot(std::size_t index) const {
        return salvage_requested && salvage_worker>=0 && (std::size_t)salvage_worker==index;
    }
    bool has_depot()const{return phase!=LanderPhase::Inactive;}
    bool active() const { return has_depot()&&!salvaged; }
    bool in_flight() const { return phase==LanderPhase::Approach&&!awaiting_site; }
    bool deploying() const { return active() && phase!=LanderPhase::Ready; }
};
struct LanderFlightSample {
    eng::Vec3 offset{}, velocity{}, acceleration{};
    float roll_degrees{0}, thrust_acceleration{0}, gear{0};
};
inline constexpr float kLanderDescentSeconds=79.0f;
inline constexpr float kLanderSettleSeconds=4.0f;
// Preserve authored flight keyframes and the phase clock recorded in saves.
inline constexpr float kLanderApproachPlaybackRate=2.5f;
inline constexpr float kLanderRampSeconds=6.0f;
inline constexpr std::int64_t kMissionBuilderMassKg=850, kMissionCarrierMassKg=1000;
inline constexpr float kStashMarkerRadius = 1.25f;
inline constexpr float kLanderSalvageSeconds = 18.0f;
inline constexpr std::size_t kLanderSalvagePortCount = 3;
eng::Vec3 lunar_lander_salvage_station(const LunarLanderState& lander,std::size_t port);
eng::Vec3 lunar_lander_salvage_focus(const LunarLanderState& lander,std::size_t port);
LanderFlightSample lunar_lander_flight(float seconds);
const char* lunar_lander_phase_name(const LunarLanderState& lander);
eng::Vec3 lunar_lander_stash_position(const LunarLanderState& lander,MaterialResource material);
eng::Vec3 lunar_lander_robot_bay(const LunarLanderState& lander,int ordinal);
struct LanderRoverPose {
    eng::Vec3 position{};
    float heading{0};
    float pitch{0};
    bool parked{false};
};
eng::Vec3 lunar_lander_robot_berth(int queue_index);
float lunar_lander_deployment_seconds(const LunarLanderState& lander,int first);
LanderRoverPose lunar_lander_robot_pose(const LunarLanderState& lander,int ordinal,int first);
bool lunar_lander_deployment_clear(const LunarLanderState& lander,eng::Vec3 position,float rotation,
                                  float width,float depth,float margin);
bool lunar_lander_segment_blocked(const LunarLanderState& lander,eng::Vec3 a,eng::Vec3 b,float radius);
} // namespace game
