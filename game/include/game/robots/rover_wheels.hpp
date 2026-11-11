#pragma once
#include "engine/math.hpp"
#include <array>
#include <cmath>

namespace game::robots {

inline constexpr float kRoverWheelRadiusM=0.22f;
inline constexpr std::array<eng::Vec3,6> kRoverWheelCenters{{
    {-.49f,kRoverWheelRadiusM,-.48f},{.49f,kRoverWheelRadiusM,-.48f},
    {-.49f,kRoverWheelRadiusM,0},{.49f,kRoverWheelRadiusM,0},
    {-.49f,kRoverWheelRadiusM,.48f},{.49f,kRoverWheelRadiusM,.48f}
}};

struct WheelPose {
    float steer_degrees{};
    float roll_radians{};
};
using RoverWheels=std::array<WheelPose,kRoverWheelCenters.size()>;

inline void roll_rover_wheels(RoverWheels& wheels,float distance,float yaw_radians){
    constexpr float pi=3.14159265358979323846f;
    for(std::size_t i=0;i<wheels.size();++i){
        const auto center=kRoverWheelCenters[i];
        const float lateral=yaw_radians*center.z;
        const float longitudinal=distance-yaw_radians*center.x;
        float travel=std::hypot(lateral,longitudinal);
        if(travel<1e-7f)continue;
        float steer=std::atan2(lateral,longitudinal);
        // Either end of the tread can lead. Keep the steering knuckle within
        // a half turn, reversing the spin on the inside of a tight pivot.
        if(std::abs(steer)>pi*.5f){
            steer-=std::copysign(pi,steer);
            travel=-travel;
        }
        wheels[i].steer_degrees=steer*180.0f/pi;
        wheels[i].roll_radians=std::remainder(wheels[i].roll_radians+travel/kRoverWheelRadiusM,2*pi);
    }
}

} // namespace game::robots
