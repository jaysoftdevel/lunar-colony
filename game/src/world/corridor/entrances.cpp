#include "game/world/corridor/detail.hpp"
#include <array>

namespace game::corridor_detail {
float local_tangent_extent(const HabitatMesh& mesh,Vec3 local_normal){
    return std::abs(local_normal.x)>0.5f?mesh.footprint_z*0.5f:mesh.footprint_x*0.5f;
}

void consider_wall_vertices(const std::vector<Vec3>& vertices,Vec3 n,Vec3 t,float tangent,float band,
                            float& best_n,float& best_diff,bool& found){
    for(const Vec3& v:vertices){
        if(v.y<0.30f || v.y>3.35f) continue;
        const float tv=dot(flatten(v),t);
        const float diff=std::abs(tv-tangent);
        if(diff>band) continue;
        const float nv=dot(flatten(v),n);
        if(!found || nv>best_n+0.02f || (std::abs(nv-best_n)<=0.02f && diff<best_diff)){
            best_n=nv; best_diff=diff; found=true;
        }
    }
}

// Locate the actual pressure-envelope surface instead of relying on a
// rectangular footprint approximation. Shell and glazing are both considered
// because the bio/cantine domes carry part of their outer envelope in the
// glazing layer. The generated vestibule therefore starts inside the real mesh
// silhouette for every current habitat shape.
Vec3 local_hull_wall_point(const HabitatMesh& mesh,Vec3 local_normal,float tangent){
    local_normal=flat_normal(local_normal);
    const Vec3 local_tangent=perp_xz(local_normal);
    const float extent=std::max(0.5f,local_tangent_extent(mesh,local_normal));
    const float band=std::max(0.55f,extent*0.18f);
    float best_n=-std::numeric_limits<float>::max();
    float best_diff=std::numeric_limits<float>::max();
    bool found=false;
    consider_wall_vertices(mesh.shell,local_normal,local_tangent,tangent,band,best_n,best_diff,found);
    consider_wall_vertices(mesh.glazing,local_normal,local_tangent,tangent,band,best_n,best_diff,found);
    if(!found){
        // Conservative fallback: use the footprint support plane. The docking
        // sleeve still penetrates it, so the connection cannot stop short.
        best_n=std::abs(local_normal.x)>0.5f?mesh.footprint_x*0.5f:mesh.footprint_z*0.5f;
    }
    return local_normal*best_n+local_tangent*tangent+Vec3{0,1.18f,0};
}

CorridorDoor transformed_builtin_door(const PlacedHabitat& habitat,const HabitatMesh& mesh,std::size_t entrance_index){
    HabitatEntrance local{};
    if(entrance_index<mesh.entrances.size()) local=mesh.entrances[entrance_index];
    CorridorDoor d;
    d.position=habitat.position+rotate_y(local.local_position,habitat.rotation_degrees);
    d.wall_position=d.position;
    d.normal=flat_normal(rotate_y(local.local_normal,habitat.rotation_degrees));
    d.generated=false;
    d.entrance_index=(int)entrance_index;
    return d;
}

bool door_occupied(std::size_t habitat_index,const CorridorDoor& candidate,const std::vector<CorridorConnection>& existing){
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.position,ep->door.position)<1.20f) return true;
        }
    }
    return false;
}

bool entrance_slot_clear(std::size_t habitat_index,const CorridorDoor& candidate,
                         const PlacedHabitat& habitat,const HabitatMesh& mesh,
                         const std::vector<CorridorConnection>& existing){
    for(std::size_t i=0;i<mesh.entrances.size();++i){
        const CorridorDoor builtin=transformed_builtin_door(habitat,mesh,i);
        if(distance_xz(candidate.wall_position,builtin.wall_position)<ENTRANCE_MIN_SPACING) return false;
    }
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.wall_position,ep->door.wall_position)<ENTRANCE_MIN_SPACING) return false;
            if(distance_xz(candidate.position,ep->door.position)<ENTRANCE_MIN_SPACING) return false;
        }
    }
    return true;
}

std::optional<CorridorDoor> generated_face_door(std::size_t habitat_index,
                                                const PlacedHabitat& habitat,
                                                const HabitatMesh& mesh,
                                                Vec3 target,
                                                const std::vector<CorridorConnection>& existing){
    const Vec3 target_dir=flat_normal(target-habitat.position);
    const std::array<Vec3,4> normals={Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}};
    const std::array<float,7> slots={0.0f,0.30f,-0.30f,0.50f,-0.50f,0.68f,-0.68f};
    bool found=false;
    float best=std::numeric_limits<float>::max();
    CorridorDoor result{};
    for(const Vec3 ln:normals){
        const Vec3 wn=flat_normal(rotate_y(ln,habitat.rotation_degrees));
        const float facing_penalty=(1.0f-dot(wn,target_dir))*4.0f;
        const float tangent_extent=local_tangent_extent(mesh,ln);
        for(float factor:slots){
            CorridorDoor d;
            d.wall_position=habitat.position+rotate_y(local_hull_wall_point(mesh,ln,factor*tangent_extent),habitat.rotation_degrees);
            d.normal=wn;
            d.position=d.wall_position+d.normal*GENERATED_ENTRANCE_DEPTH;
            d.generated=true;
            d.entrance_index=-1;
            if(!entrance_slot_clear(habitat_index,d,habitat,mesh,existing)) continue;
            const float score=distance_xz(d.position,target)+facing_penalty+std::abs(factor)*0.4f;
            if(score<best){ best=score; result=d; found=true; }
        }
    }
    if(!found) return std::nullopt;
    return result;
}

CorridorEndpoint resolve_open_or_corridor(const CorridorBuildAnchor& anchor,Vec3 other,
                                          const std::vector<CorridorConnection>& existing){
    CorridorEndpoint out;
    out.kind=anchor.kind;
    out.index=anchor.index;
    out.position=anchor.position;
    out.position.y=0;
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<existing.size()){
        // Snap to the host centerline exactly. This prevents tiny visual gaps
        // when a branch/link was selected a few pixels away from the line.
        out.position=nearest_path_point(existing[anchor.index].path,anchor.position);
        out.host_tangent=nearest_path_tangent(existing[anchor.index].path,out.position);
    }
    out.tangent=flat_normal(other-out.position);
    if(length(out.tangent)<0.001f) out.tangent=length(out.host_tangent)>0.001f?out.host_tangent:Vec3{1,0,0};
    return out;
}

std::optional<CorridorEndpoint> resolve_habitat(const CorridorBuildAnchor& anchor,Vec3 other,
                                                const std::vector<PlacedHabitat>& habitats,
                                                const HabitatModelLibrary& library,
                                                const std::vector<CorridorConnection>& existing){
    if(anchor.index>=habitats.size()) return std::nullopt;
    const auto& habitat=habitats[anchor.index];
    const auto& mesh=library.mesh(habitat.kind);

    // Prefer a real built-in entrance that faces the target and is still free.
    // This is especially important for the air lock, which has a front entry
    // and a rear exit. A corridor approaching from behind must use the rear
    // hatch instead of inventing a side vestibule.
    bool found_builtin=false;
    float best_score=std::numeric_limits<float>::max();
    CorridorDoor d{};
    for(std::size_t i=0;i<mesh.entrances.size();++i){
        CorridorDoor candidate=transformed_builtin_door(habitat,mesh,i);
        const Vec3 toward=flat_normal(other-candidate.position);
        const float facing_dot=length(toward)>0.001f?dot(candidate.normal,toward):-1.0f;
        if(facing_dot<=0.45f || door_occupied(anchor.index,candidate,existing)) continue;
        const float score=distance_xz(candidate.position,other)+(1.0f-facing_dot)*1.5f;
        if(score<best_score){ best_score=score; d=candidate; found_builtin=true; }
    }

    if(!found_builtin){
        auto generated=generated_face_door(anchor.index,habitat,mesh,other,existing);
        if(!generated) return std::nullopt;
        d=*generated;
    }

    CorridorEndpoint out;
    out.kind=CorridorEndpointKind::Habitat;
    out.index=anchor.index;
    out.door=d;
    out.tangent=d.normal;
    out.position=d.position+d.normal*DOCK_RUN;
    out.position.y=0;
    return out;
}

std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,
                                               const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing){
    if(anchor.kind==CorridorEndpointKind::Habitat) return resolve_habitat(anchor,other,habitats,library,existing);
    return resolve_open_or_corridor(anchor,other,existing);
}
} // namespace game::corridor_detail
