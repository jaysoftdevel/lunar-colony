#include "game/systems/first_person_controller.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;
namespace {
float wrap_degrees(float value){
    while(value>=360.0f) value-=360.0f;
    while(value<0.0f) value+=360.0f;
    return value;
}
}

void FirstPersonController::spawn(Vec3 feet_position,float yaw_degrees){
    feet_position_=feet_position;
    yaw_degrees_=wrap_degrees(yaw_degrees);
    pitch_degrees_=0.0f;
    planar_velocity_={};
}

void FirstPersonController::update_look(const InputState& input){
    constexpr float sensitivity=0.105f;
    yaw_degrees_=wrap_degrees(yaw_degrees_+(float)input.mouse_dx*sensitivity);
    pitch_degrees_=std::clamp(pitch_degrees_-(float)input.mouse_dy*sensitivity,-82.0f,82.0f);
}

Vec3 FirstPersonController::desired_displacement(const InputState& input,float dt) {
    const float step=std::clamp(dt,0.0f,0.05f);
    const float yaw=radians(yaw_degrees_);
    const Vec3 forward{std::sin(yaw),0.0f,-std::cos(yaw)};
    const Vec3 right{std::cos(yaw),0.0f,std::sin(yaw)};
    Vec3 wish{};
    if(input.key_w_down) wish+=forward;
    if(input.key_s_down) wish+=forward*-1.0f;
    if(input.key_d_down) wish+=right;
    if(input.key_a_down) wish+=right*-1.0f;

    // A suited astronaut has mass and does not jump instantly between zero and
    // sprint speed. Smooth acceleration/deceleration also makes tight air-lock
    // and stair movement substantially more controllable in POV.
    Vec3 target{};
    if(length(wish)>=0.001f){
        wish=normalize(wish);
        const float speed=input.key_shift_down?3.40f:2.00f;
        target=wish*speed;
    }
    const Vec3 delta=target-planar_velocity_;
    const float change=length(delta);
    const float acceleration=length(target)>0.001f?5.5f:7.0f;
    const float max_change=acceleration*step;
    if(change<=max_change || change<0.0001f) planar_velocity_=target;
    else planar_velocity_+=delta*(max_change/change);
    return planar_velocity_*step;
}

Vec3 FirstPersonController::look_direction() const {
    const float yaw=radians(yaw_degrees_);
    const float pitch=radians(pitch_degrees_);
    return normalize(Vec3{
        std::sin(yaw)*std::cos(pitch),
        std::sin(pitch),
        -std::cos(yaw)*std::cos(pitch)
    });
}

Mat4 FirstPersonController::view_projection(float aspect) const {
    const Vec3 eye=eye_position();
    const Vec3 forward=look_direction();
    return multiply(perspective(radians(68.0f),aspect,0.035f,5000.0f),
                    look_at(eye,eye+forward,{0.0f,1.0f,0.0f}));
}

} // namespace game
