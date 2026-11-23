#pragma once
#include "game/runtime/presentation.hpp"
#include "game/world/geometry_standards.hpp"

namespace game {

class FirstPersonController {
public:
    void spawn(game::runtime::Vec3 feet_position,float yaw_degrees=0.0f);
    void restore_view(float yaw,float pitch);
    void update_look(const game::runtime::InputState& input);
    game::runtime::Vec3 desired_displacement(const game::runtime::InputState& input,float dt);
    void advance_vertical(bool jump_pressed,float dt,float floor_y,float ceiling_y);
    void set_fov(float degrees){fov_degrees_=std::isfinite(degrees)?std::clamp(degrees,50.f,110.f):68.f;}
    float fov() const {return fov_degrees_;}
    void stop_motion(){planar_velocity_={};}
    bool grounded() const {return grounded_;}
    game::runtime::Mat4 view_projection(float aspect) const;
    game::runtime::Vec3 eye_position() const { return feet_position_+game::runtime::Vec3{0.0f,eye_height_,0.0f}; }
    game::runtime::Vec3 look_direction() const;
    game::runtime::Vec3 feet_position() const { return feet_position_; }
    void set_feet_position(game::runtime::Vec3 p){ feet_position_=p; }
    float yaw_degrees() const { return yaw_degrees_; }
    float pitch_degrees() const { return pitch_degrees_; }
    float body_radius() const { return body_radius_; }
    float body_height() const { return body_height_; }
private:
    game::runtime::Vec3 feet_position_{};
    float fov_degrees_{68.f};
    float yaw_degrees_{0.0f};
    float pitch_degrees_{0.0f};
    // The FPV body is the canonical astronaut scale reference for all future
    // crew implementations. World units remain metres.
    game::runtime::Vec3 planar_velocity_{};
    float vertical_velocity_{0};
    bool grounded_{true};
    float eye_height_{geometry::kHumanEyeHeight};
    float body_height_{geometry::kHumanHeight};
    float body_radius_{geometry::kHumanBodyRadius};
};

} // namespace game
