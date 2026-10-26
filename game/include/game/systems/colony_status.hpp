#pragma once

namespace game {
class ColonyStatus {
public:
    void reset();
    void update(float dt);
    int day() const { return day_; }
    float hour() const { return hour_; }
    bool is_daylight() const { return hour_>=6.0f && hour_<18.0f; }
    int power_current() const { return power_current_; }
    int power_capacity() const { return power_capacity_; }
    int water_current() const { return water_current_; }
    int water_capacity() const { return water_capacity_; }
    int resources() const { return resources_; }
    int crew_current() const { return crew_current_; }
    int crew_capacity() const { return crew_capacity_; }
private:
    int day_=1;
    float hour_=8.0f;
    float day_length_seconds_=240.0f;
    int power_current_=72;
    int power_capacity_=100;
    int water_current_=840;
    int water_capacity_=1000;
    int resources_=1250;
    int crew_current_=6;
    int crew_capacity_=8;
};
}
