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

    auto path=smooth_route(*a,*b,habitats,library);
    if(path.size()<2) return std::nullopt;

    CorridorConnection out;
    out.kind=kind;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,existing);
    return out;
}
} // namespace game::corridor_detail
