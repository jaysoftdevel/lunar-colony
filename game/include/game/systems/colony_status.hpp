#pragma once
#include "game/systems/solar_tracking.hpp"

namespace game {
struct ColonyStatusState {
    int day{1};
    float hour{8.0f};
    int power_current{72};
    int power_capacity{100};
    int water_current{840};
    int water_capacity{1000};
    int crew_current{0};
    int crew_capacity{0};
};

class ColonyStatus {
public:
    void reset();
    void update(float dt);
    int day() const { return day_; }
    float hour() const { return hour_; }
    solar_tracking::SolarPose sun_pose() const { return solar_tracking::pose(hour_); }
    eng::Vec3 sun_direction() const { return sun_pose().direction; }
    bool is_daylight() const { return sun_pose().above_horizon; }
    int power_current() const { return power_current_; }
    int power_capacity() const { return power_capacity_; }
    int water_current() const { return water_current_; }
    int water_capacity() const { return water_capacity_; }
    int crew_current() const { return crew_current_; }
    int crew_capacity() const { return crew_capacity_; }
    ColonyStatusState state() const;
    void restore(const ColonyStatusState& state);
private:
    int day_=1;
    float hour_=8.0f;
    float day_length_seconds_=240.0f;
    int power_current_=72;
    int power_capacity_=100;
    int water_current_=840;
    int water_capacity_=1000;
    int crew_current_=0;
    int crew_capacity_=0;
};
}
