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

void ColonyStatus::update(float dt){
    if(day_length_seconds_<=0.0f) return;
    hour_ += dt*(24.0f/day_length_seconds_);
    while(hour_>=24.0f){
        hour_-=24.0f;
        ++day_;
    }
}
}
