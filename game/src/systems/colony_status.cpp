#include "game/systems/colony_status.hpp"

namespace game {
void ColonyStatus::reset(){
    day_=1;
    hour_=8.0f;
    power_current_=72;
    power_capacity_=100;
    water_current_=840;
    water_capacity_=1000;
    crew_current_=6;
    crew_capacity_=8;
}

ColonyStatusState ColonyStatus::state() const {
    return {day_,hour_,power_current_,power_capacity_,water_current_,water_capacity_,crew_current_,crew_capacity_};
}

void ColonyStatus::restore(const ColonyStatusState& state){
    day_=state.day;
    hour_=state.hour;
    power_current_=state.power_current;
    power_capacity_=state.power_capacity;
    water_current_=state.water_current;
    water_capacity_=state.water_capacity;
    crew_current_=state.crew_current;
    crew_capacity_=state.crew_capacity;
}

void ColonyStatus::update(float dt){
    if(day_length_seconds_<=0.0f) return;
    hour_ += dt*(24.0f/day_length_seconds_);
    while(hour_>=24.0f){
        hour_-=24.0f;
        ++day_;
    }
}
}
