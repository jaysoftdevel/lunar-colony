#pragma once
#include "engine/engine.hpp"
#include <cstdint>
#include <vector>

namespace game::construction_sequence {

struct HabitatVisibility {
    float foundation{0.0f};
    float shell{0.0f};
    float interior{0.0f};
    float details{0.0f};
    float glazing{0.0f};
    float edges{0.0f};
    float access{0.0f};
};

struct ResourceVisibility {
    float foundation{0.0f};
    float body{0.0f};
    float details{0.0f};
    float edges{0.0f};
};

HabitatVisibility habitat_visibility(std::uint8_t step);
ResourceVisibility resource_visibility(std::uint8_t step);
std::vector<eng::Vec3> reveal_triangles(const std::vector<eng::Vec3>& triangles,float fraction);
std::vector<eng::Vec3> reveal_lines(const std::vector<eng::Vec3>& lines,float fraction);

} // namespace game::construction_sequence
