#include "game/systems/camera_controller.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace game::runtime;

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

void CameraController::update(const InputState& in,int playfield_size,int viewport_height,float dt){
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

    const float yaw=radians(azimuth_deg_);
    const Vec3 right{std::cos(yaw),0.0f,-std::sin(yaw)};
    const Vec3 ground_fwd{-std::sin(yaw),0.0f,-std::cos(yaw)};

    if(in.right_down && (in.mouse_dx||in.mouse_dy)){
        const float h=(float)std::max(1,viewport_height);
        const float units_per_px=2.0f*distance_*std::tan(radians(55.0f)*0.5f)/h;
        target_ += right*(-in.mouse_dx*units_per_px) + ground_fwd*(in.mouse_dy*units_per_px);
    }

    // Keyboard camera translation mirrors familiar RTS/free-camera controls.
    // W/S move along the camera's ground-forward axis while A/D strafe left
    // and right. Movement is time based (not frame based) and scales gently
    // with zoom so it remains useful both close to and far above the base.
    Vec3 keyboard_move{};
    if(in.key_w_down) keyboard_move+=ground_fwd;
    if(in.key_s_down) keyboard_move+=ground_fwd*-1.0f;
    if(in.key_d_down) keyboard_move+=right;
    if(in.key_a_down) keyboard_move+=right*-1.0f;
    if(length(keyboard_move)>0.001f){
        keyboard_move=normalize(keyboard_move);
        const float speed=std::clamp(distance_*0.70f,12.0f,90.0f)*(in.key_shift_down?1.75f:1.0f);
        target_+=keyboard_move*(speed*std::clamp(dt,0.0f,0.05f));
    }

    if(in.right_down || length(keyboard_move)>0.001f){
        const float half=(float)playfield_size*0.75f;
        target_.x=std::clamp(target_.x,-half,half);
        target_.z=std::clamp(target_.z,-half,half);
    }
}

void CameraController::scroll_at_edge(const InputState& in,int size,int width,int height,Rect world,float dt){
    if(!in.window_focused || !in.pointer_inside_window || in.left_down || in.middle_down || in.right_down ||
       width<=0 || height<=0 || world.w<=0 || world.h<=0 || !world.contains(float(in.mouse_x),float(in.mouse_y)))return;
    const float band=std::clamp(float(height)*.025f,14.f,32.f);
    const auto axis=[&](float value,float lo,float hi){
        return value<lo+band?-(lo+band-value)/band:value>hi-band?(value-hi+band)/band:0.f;
    };
    Vec2 edge{axis(float(in.mouse_x),world.x,world.x+world.w),axis(float(in.mouse_y),world.y,world.y+world.h)};
    if(std::abs(edge.x)+std::abs(edge.y)<.001f)return;
    const float norm=std::hypot(edge.x,edge.y);if(norm>1){edge.x/=norm;edge.y/=norm;}
    Vec3 eye,forward,right,up;basis(eye,forward,right,up);
    const float yaw=radians(azimuth_deg_);
    const Vec3 ground_fwd{-std::sin(yaw),0,-std::cos(yaw)};
    Vec3 move=(right*edge.x-ground_fwd*edge.y)*
        (std::clamp(distance_*.70f,12.f,90.f)*std::clamp(dt,0.f,.05f));
    // Bound the visible ground footprint, not just the orbit target. When the
    // whole map fits on screen an axis has no scrolling range. Invalid horizon
    // rays fall back to the finite playfield-centred range at low elevations.
    Vec3 low{1.e20f,0,1.e20f},high{-1.e20f,0,-1.e20f};bool valid=true;
    const float tangent=std::tan(radians(55.f)*.5f),aspect=float(width)/height;
    for(float x:{world.x,world.x+world.w})for(float y:{world.y,world.y+world.h}){
        const Vec3 direction=forward+right*((2*x/width-1)*aspect*tangent)+up*((1-2*y/height)*tangent);
        if(direction.y>=-.00001f){valid=false;continue;}
        const auto p=eye+direction*(-eye.y/direction.y)-target_;
        low.x=std::min(low.x,p.x);low.z=std::min(low.z,p.z);high.x=std::max(high.x,p.x);high.z=std::max(high.z,p.z);
    }
    const float half=std::max(0.f,float(size)*.5f-.002f);
    const auto bound=[&](float current,float delta,float a,float b){
        float lo=valid?-half-a:-half,hi=valid?half-b:half;
        if(lo>hi)lo=hi=(lo+hi)*.5f;
        // An older save may already be beyond an edge. Allow a smooth return;
        // never jump the camera to the newly bounded interval.
        if(current<lo)return delta>0?std::min(delta,hi-current):0.f;
        if(current>hi)return delta<0?std::max(delta,lo-current):0.f;
        return std::clamp(current+delta,lo,hi)-current;
    };
    target_.x+=bound(target_.x,move.x,low.x,high.x);
    target_.z+=bound(target_.z,move.z,low.z,high.z);
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
    if(elevation_deg_<0.5f) eye.y=target_.y+0.05f;
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

Mat4 CameraController::campus_view_projection(float aspect) const {
    Vec3 eye,forward,right,up;
    basis(eye,forward,right,up);
    // The old 0.05..5000 frustum lost most of its depth precision at campus
    // orbit distances (~100-600 m), causing millimetre-thin markings to fight
    // with the roads beneath them. Keep the far plane just beyond the coast.
    const float near_plane=std::clamp(distance_*0.055f,0.35f,32.0f);
    const float far_plane=std::max(near_plane+80.0f,distance_+420.0f);
    return multiply(perspective(radians(55.0f),aspect,near_plane,far_plane),
                    look_at(eye,target_,up));
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
