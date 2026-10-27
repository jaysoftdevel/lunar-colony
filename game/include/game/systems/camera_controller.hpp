#pragma once
#include "engine/engine.hpp"

namespace game {
struct CameraState {
    float distance{95.0f};
    float elevation_degrees{55.0f};
    float azimuth_degrees{0.0f};
    eng::Vec3 target{};
};

class CameraController {
public:
    void reset(int playfield_size);
    void update(const eng::InputState& input,int playfield_size,int viewport_height);
    eng::Mat4 view_projection(float aspect) const;
    eng::Vec3 eye_position() const;
    bool intersect_playfield(int screen_x,int screen_y,int viewport_width,int viewport_height,int playfield_size,eng::Vec3& hit) const;
    float distance() const { return distance_; }
    float elevation_degrees() const { return elevation_deg_; }
    float azimuth_degrees() const { return azimuth_deg_; }
    eng::Vec3 target() const { return target_; }
    CameraState state() const { return {distance_,elevation_deg_,azimuth_deg_,target_}; }
    void restore(const CameraState& state);
private:
    float distance_=95.0f;
    float elevation_deg_=55.0f;
    float azimuth_deg_=0.0f;
    eng::Vec3 target_{0,0,0};
    void basis(eng::Vec3& eye,eng::Vec3& forward,eng::Vec3& right,eng::Vec3& up) const;
};
}
