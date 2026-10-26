#include "game/systems/camera_controller.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;

namespace {
float wrap_degrees(float value) {
    while(value >= 360.0f) value -= 360.0f;
    while(value < 0.0f) value += 360.0f;
    return value;
}
}

void CameraController::reset(int playfield_size){
    target_={0,0,0};
    distance_=std::max(28.0f,playfield_size*1.35f);
    elevation_deg_=55.0f;
    azimuth_deg_=0.0f;
}

void CameraController::update(const InputState& in,int playfield_size,int viewport_height){
    if(in.wheel!=0){
        const float factor=std::pow(0.88f,in.wheel);
        distance_=std::clamp(distance_*factor,8.0f,(float)playfield_size*5.0f);
    }

    // Middle mouse rotates the camera around the current view center.
    // Horizontal movement rotates around the playfield, vertical movement tilts 0..90 degrees.
    if(in.middle_down){
        if(in.mouse_dy!=0)
            elevation_deg_=std::clamp(elevation_deg_-(float)in.mouse_dy*0.32f,0.0f,90.0f);
        if(in.mouse_dx!=0)
            azimuth_deg_=wrap_degrees(azimuth_deg_-(float)in.mouse_dx*0.35f);
    }

    // Right mouse pans in the camera's ground-plane basis. Scaling by camera distance keeps
    // drag speed stable relative to zoom and the current view center.
    if(in.right_down && (in.mouse_dx||in.mouse_dy)){
        const float yaw=radians(azimuth_deg_);
        const Vec3 right{std::cos(yaw),0.0f,-std::sin(yaw)};
        const Vec3 ground_fwd{-std::sin(yaw),0.0f,-std::cos(yaw)};
        const float h=(float)std::max(1,viewport_height);
        const float units_per_px=2.0f*distance_*std::tan(radians(55.0f)*0.5f)/h;
        target_ += right*(-in.mouse_dx*units_per_px) + ground_fwd*(in.mouse_dy*units_per_px);
        const float half=(float)playfield_size*0.75f;
        target_.x=std::clamp(target_.x,-half,half);
        target_.z=std::clamp(target_.z,-half,half);
    }
}

Mat4 CameraController::view_projection(float aspect) const {
    const float elev=radians(elevation_deg_);
    const float yaw=radians(azimuth_deg_);
    const float horizontal=std::cos(elev)*distance_;
    Vec3 eye=target_+Vec3{
        std::sin(yaw)*horizontal,
        std::sin(elev)*distance_,
        std::cos(yaw)*horizontal
    };
    if(elevation_deg_<0.5f) eye.y=0.05f;

    const Vec3 forward=normalize(target_-eye);
    const Vec3 right{std::cos(yaw),0.0f,-std::sin(yaw)};
    Vec3 camera_up=normalize(cross(right,forward));
    if(length(camera_up)<0.0001f) camera_up={0.0f,0.0f,-1.0f};

    return multiply(
        perspective(radians(55.0f),aspect,0.05f,5000.0f),
        look_at(eye,target_,camera_up)
    );
}
}
