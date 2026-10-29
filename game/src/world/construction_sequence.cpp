#include "game/world/construction_sequence.hpp"
#include "game/world/construction.hpp"
#include <algorithm>
#include <cmath>

namespace game::construction_sequence {
namespace {
float clamp01(float v){ return std::clamp(v,0.0f,1.0f); }
float continuous_phase(float step,float first,float last){
    if(step<=first) return 0.0f;
    if(step>=last) return 1.0f;
    return clamp01((step-first)/std::max(0.001f,last-first));
}
}

HabitatVisibility habitat_visibility(float step){
    if(step>=kConstructionSteps) return {1,1,1,1,1,1,1};
    // Keep the original ten-stage visual choreography while allowing the
    // gameplay stage count to be shortened. The normalized value means a
    // six-stage build reveals the same mesh groups at the same relative
    // completion percentages instead of dropping late-stage details.
    step=step*(10.0f/(float)std::max<std::uint8_t>(1,kConstructionSteps));
    HabitatVisibility v{};
    // Discrete assembly plan: ground works -> pressure shell -> internal modules
    // -> glazing/services -> access/finish. Every placed habitat uses its own
    // mesh order, so the visible modules are specific to that model.
    // Construction mesh must become visible during the very first work
    // iteration. Use continuous ramps rather than waiting for the first whole
    // legacy stage boundary; reveal_triangles() guarantees at least one authored
    // triangle as soon as the fraction becomes non-zero.
    v.foundation = step>0.0f?std::max(0.12f,continuous_phase(step,0.00f,1.60f)):0.0f;
    v.shell      = continuous_phase(step,0.70f,5.20f);
    v.interior   = continuous_phase(step,4.50f,7.30f);
    v.glazing    = continuous_phase(step,6.80f,9.30f);
    v.details    = continuous_phase(step,6.50f,9.20f);
    v.access     = continuous_phase(step,8.40f,10.0f);
    v.edges      = continuous_phase(step,8.80f,10.0f);
    return v;
}

ResourceVisibility resource_visibility(float step){
    if(step>=kConstructionSteps) return {1,1,1,1};
    step=step*(10.0f/(float)std::max<std::uint8_t>(1,kConstructionSteps));
    ResourceVisibility v{};
    // External facilities assemble base-first, then the main machine/body,
    // then service hardware and final structural edges.
    v.foundation = step>0.0f?std::max(0.12f,continuous_phase(step,0.00f,1.60f)):0.0f;
    v.body       = continuous_phase(step,0.75f,7.20f);
    v.details    = continuous_phase(step,6.30f,9.30f);
    v.edges      = continuous_phase(step,8.70f,10.0f);
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
