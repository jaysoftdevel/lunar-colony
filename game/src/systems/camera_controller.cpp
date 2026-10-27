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


void CameraController::restore(const CameraState& state){
    distance_=std::max(0.1f,state.distance);
    elevation_deg_=std::clamp(state.elevation_degrees,0.0f,90.0f);
    azimuth_deg_=wrap_degrees(state.azimuth_degrees);
    target_=state.target;
}

void CameraController::update(const InputState& in,int playfield_size,int viewport_height){
    if(in.wheel!=0){
        const float factor=std::pow(0.88f,in.wheel);
        distance_=std::clamp(distance_*factor,8.0f,(float)playfield_size*5.0f);
    }

    if(in.middle_down){
        if(in.mouse_dy!=0)
            elevation_deg_=std::clamp(elevation_deg_+(float)in.mouse_dy*0.32f,0.0f,90.0f);
        if(in.mouse_dx!=0)
            azimuth_deg_=wrap_degrees(azimuth_deg_-(float)in.mouse_dx*0.35f);
    }

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

void CameraController::basis(Vec3& eye,Vec3& forward,Vec3& right,Vec3& up) const {
    const float elev=radians(elevation_deg_);
    const float yaw=radians(azimuth_deg_);
    const float horizontal=std::cos(elev)*distance_;
    eye=target_+Vec3{
        std::sin(yaw)*horizontal,
        std::sin(elev)*distance_,
        std::cos(yaw)*horizontal
    };
    if(elevation_deg_<0.5f) eye.y=0.05f;
    forward=normalize(target_-eye);
    right={std::cos(yaw),0.0f,-std::sin(yaw)};
    up=normalize(cross(right,forward));
    if(length(up)<0.0001f) up={0.0f,0.0f,-1.0f};
}

Vec3 CameraController::eye_position() const {
    Vec3 eye,forward,right,up;
    basis(eye,forward,right,up);
    return eye;
}

Mat4 CameraController::view_projection(float aspect) const {
    Vec3 eye,forward,right,up;
    basis(eye,forward,right,up);
    return multiply(
        perspective(radians(55.0f),aspect,0.05f,5000.0f),
        look_at(eye,target_,up)
    );
}

bool CameraController::intersect_playfield(int screen_x,int screen_y,int viewport_width,int viewport_height,int playfield_size,Vec3& hit) const {
    if(viewport_width<=0 || viewport_height<=0) return false;
    Vec3 eye,forward,right,up;
    basis(eye,forward,right,up);
    const float aspect=(float)viewport_width/(float)viewport_height;
    const float tan_half=std::tan(radians(55.0f)*0.5f);
    const float ndc_x=2.0f*(float)screen_x/(float)viewport_width-1.0f;
    const float ndc_y=1.0f-2.0f*(float)screen_y/(float)viewport_height;
    const Vec3 dir=normalize(forward + right*(ndc_x*aspect*tan_half) + up*(ndc_y*tan_half));
    if(std::abs(dir.y)<0.00001f) return false;
    const float t=-eye.y/dir.y;
    if(t<0.0f) return false;
    hit=eye+dir*t;
    const float half=(float)playfield_size*0.5f;
    return hit.x>=-half && hit.x<=half && hit.z>=-half && hit.z<=half;
}
}
