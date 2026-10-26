#pragma once
#include "engine/engine.hpp"

namespace game {
class CameraController {
public:
    void reset(int playfield_size);
    void update(const eng::InputState& input,int playfield_size,int viewport_height);
    eng::Mat4 view_projection(float aspect) const;
    float distance() const { return distance_; }
    float elevation_degrees() const { return elevation_deg_; }
    float azimuth_degrees() const { return azimuth_deg_; }
    eng::Vec3 target() const { return target_; }
private:
    float distance_=95.0f;
    float elevation_deg_=55.0f;
    float azimuth_deg_=0.0f;
    eng::Vec3 target_{0,0,0};
};
}
