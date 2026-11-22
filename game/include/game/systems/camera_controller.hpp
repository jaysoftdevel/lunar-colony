#pragma once
#include "game/runtime/presentation.hpp"

namespace game {
struct CameraState {
    float distance{95.0f};
    float elevation_degrees{55.0f};
    float azimuth_degrees{0.0f};
    game::runtime::Vec3 target{};
};

class CameraController {
public:
    void reset(int playfield_size);
    void update(const game::runtime::InputState& input,int playfield_size,int viewport_height,float dt);
    game::runtime::Mat4 view_projection(float aspect) const;
    // Stage-1 coastal campus uses a close, distance-dependent depth range so
    // thin road/pad/marking layers remain stable during orbit and zoom.
    game::runtime::Mat4 campus_view_projection(float aspect) const;
    game::runtime::Vec3 eye_position() const;
    bool intersect_playfield(int screen_x,int screen_y,int viewport_width,int viewport_height,int playfield_size,game::runtime::Vec3& hit) const;
    float distance() const { return distance_; }
    float elevation_degrees() const { return elevation_deg_; }
    float azimuth_degrees() const { return azimuth_deg_; }
    game::runtime::Vec3 target() const { return target_; }
    CameraState state() const { return {distance_,elevation_deg_,azimuth_deg_,target_}; }
    void restore(const CameraState& state);
private:
    float distance_=95.0f;
    float elevation_deg_=55.0f;
    float azimuth_deg_=0.0f;
    game::runtime::Vec3 target_{0,0,0};
    void basis(game::runtime::Vec3& eye,game::runtime::Vec3& forward,game::runtime::Vec3& right,game::runtime::Vec3& up) const;
};
}
