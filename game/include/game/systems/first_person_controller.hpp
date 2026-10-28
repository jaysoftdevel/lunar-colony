#pragma once
#include "engine/engine.hpp"
#include "game/world/geometry_standards.hpp"

namespace game {

class FirstPersonController {
public:
    void spawn(eng::Vec3 feet_position,float yaw_degrees=0.0f);
    void update_look(const eng::InputState& input);
    eng::Vec3 desired_displacement(const eng::InputState& input,float dt);
    eng::Mat4 view_projection(float aspect) const;
    eng::Vec3 eye_position() const { return feet_position_+eng::Vec3{0.0f,eye_height_,0.0f}; }
    eng::Vec3 look_direction() const;
    eng::Vec3 feet_position() const { return feet_position_; }
    void set_feet_position(eng::Vec3 p){ feet_position_=p; }
    float yaw_degrees() const { return yaw_degrees_; }
    float pitch_degrees() const { return pitch_degrees_; }
    float body_radius() const { return body_radius_; }
    float body_height() const { return body_height_; }
private:
    eng::Vec3 feet_position_{};
    float yaw_degrees_{0.0f};
    float pitch_degrees_{0.0f};
    // The FPV body is the canonical astronaut scale reference for all future
    // crew implementations. World units remain metres.
    eng::Vec3 planar_velocity_{};
    float eye_height_{geometry::kHumanEyeHeight};
    float body_height_{geometry::kHumanHeight};
    float body_radius_{geometry::kHumanBodyRadius};
};

} // namespace game
