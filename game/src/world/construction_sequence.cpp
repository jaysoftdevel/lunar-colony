#include "game/world/construction_sequence.hpp"
#include "game/world/construction.hpp"
#include <algorithm>
#include <cmath>

namespace game::construction_sequence {
namespace {
float clamp01(float v){ return std::clamp(v,0.0f,1.0f); }
float phase(std::uint8_t step,int first,int last){
    if(step < first) return 0.0f;
    if(step >= last) return 1.0f;
    const float span=static_cast<float>(last-first+1);
    return clamp01((static_cast<float>(step-first)+1.0f)/span);
}
}

HabitatVisibility habitat_visibility(std::uint8_t step){
    if(step>=kConstructionSteps) return {1,1,1,1,1,1,1};
    HabitatVisibility v{};
    // Discrete assembly plan: ground works -> pressure shell -> internal modules
    // -> glazing/services -> access/finish. Every placed habitat uses its own
    // mesh order, so the visible modules are specific to that model.
    v.foundation = phase(step,1,2);
    v.shell      = phase(step,2,5);
    v.interior   = phase(step,5,7);
    v.glazing    = phase(step,7,9);
    v.details    = phase(step,7,9);
    v.access     = phase(step,9,10);
    v.edges      = step>=10 ? 1.0f : 0.0f;
    return v;
}

ResourceVisibility resource_visibility(std::uint8_t step){
    if(step>=kConstructionSteps) return {1,1,1,1};
    ResourceVisibility v{};
    // External facilities assemble base-first, then the main machine/body,
    // then service hardware and final structural edges.
    v.foundation = phase(step,1,2);
    v.body       = phase(step,2,7);
    v.details    = phase(step,7,9);
    v.edges      = step>=10 ? 1.0f : 0.0f;
    return v;
}

std::vector<eng::Vec3> reveal_triangles(const std::vector<eng::Vec3>& triangles,float fraction){
    if(triangles.empty() || fraction<=0.0f) return {};
    if(fraction>=1.0f) return triangles;
    const std::size_t tri_count=triangles.size()/3;
    const std::size_t visible=std::min(tri_count,std::max<std::size_t>(1,(std::size_t)std::ceil((double)tri_count*fraction)));
    return {triangles.begin(),triangles.begin()+static_cast<std::ptrdiff_t>(visible*3)};
}

std::vector<eng::Vec3> reveal_lines(const std::vector<eng::Vec3>& lines,float fraction){
    if(lines.empty() || fraction<=0.0f) return {};
    if(fraction>=1.0f) return lines;
    const std::size_t segment_count=lines.size()/2;
    const std::size_t visible=std::min(segment_count,std::max<std::size_t>(1,(std::size_t)std::ceil((double)segment_count*fraction)));
    return {lines.begin(),lines.begin()+static_cast<std::ptrdiff_t>(visible*2)};
}

} // namespace game::construction_sequence
