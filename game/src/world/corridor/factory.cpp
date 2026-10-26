#include "game/world/corridor/detail.hpp"

namespace game::corridor_detail {
std::optional<CorridorConnection> make_generic(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                               CorridorConnectionKind kind,
                                               const std::vector<PlacedHabitat>& habitats,
                                               const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing){
    Vec3 a_hint=a_anchor.position,b_hint=b_anchor.position;
    if(a_anchor.kind==CorridorEndpointKind::Habitat && a_anchor.index<habitats.size()) a_hint=habitats[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Habitat && b_anchor.index<habitats.size()) b_hint=habitats[b_anchor.index].position;
    auto a=resolve_anchor(a_anchor,b_hint,habitats,library,existing);
    auto b=resolve_anchor(b_anchor,a_hint,habitats,library,existing);
    if(!a || !b) return std::nullopt;

    // Re-resolve habitats once using the actual opposite endpoint so generated
    // door placement tracks corridor/corridor/open targets precisely.
    if(a_anchor.kind==CorridorEndpointKind::Habitat) a=resolve_anchor(a_anchor,b->position,habitats,library,existing);
    if(b_anchor.kind==CorridorEndpointKind::Habitat) b=resolve_anchor(b_anchor,a->position,habitats,library,existing);
    if(!a || !b) return std::nullopt;

    // Habitat-to-habitat runs shorter than eight raster cells stay elevated
    // and connect directly. The corridor walkway is aligned close to the door
    // thresholds instead of dropping to the lunar surface and climbing again.
    if(a->kind==CorridorEndpointKind::Habitat && b->kind==CorridorEndpointKind::Habitat &&
       distance_xz(a->position,b->position)<DIRECT_CORRIDOR_MAX_RASTERS){
        const float threshold_a=std::max(WALKWAY_Y,a->door.wall_position.y-0.30f);
        const float threshold_b=std::max(WALKWAY_Y,b->door.wall_position.y-0.30f);
        const float base=std::max(DIRECT_CORRIDOR_MIN_BASE_Y,std::min(threshold_a,threshold_b)-WALKWAY_Y);
        a->position.y=base;
        b->position.y=base;
    }

    auto path=smooth_route(*a,*b,habitats,library);
    if(path.size()<2) return std::nullopt;

    // A corridor extended from an existing open end must share the exact end
    // ring plane. Pin the first/last sampled segment to the host terminal
    // tangent; otherwise curve sampling can rotate the D-profile a few degrees
    // and leave a wedge-shaped pressure gap.
    if(path.size()>=3){
        const float terminal=std::clamp(distance_xz(path.front(),path.back())*0.08f,0.05f,0.18f);
        if(a->seamless_splice) path[1]=a->position+a->tangent*terminal;
        if(b->seamless_splice) path[path.size()-2]=b->position+b->tangent*terminal;
    }

    CorridorConnection out;
    out.kind=kind;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,existing);
    return out;
}
} // namespace game::corridor_detail
