#include "game/world/corridor/detail.hpp"
#include <array>

namespace game::corridor_detail {
struct AttachmentExtents {
    float half_x{};
    float half_z{};
    bool rounded_xz{false};
};

AttachmentExtents attachment_extents(HabitatKind kind){
    switch(kind){
        case HabitatKind::AirLock: return {3.62f,2.16f,false};
        case HabitatKind::OxygenGeneratorDome: return {4.38f,4.38f,true};
        case HabitatKind::CantineDome: return {4.55f,4.55f,true};
        case HabitatKind::BioDome: return {5.26f,3.31f,false};
        case HabitatKind::CrewQuarters: return {5.42f,2.36f,false};
        default: return {2.0f,2.0f,false};
    }
}

CorridorDoor transformed_default_door(const PlacedHabitat& habitat,const HabitatMesh& mesh){
    HabitatEntrance local{};
    if(!mesh.entrances.empty()) local=mesh.entrances.front();
    CorridorDoor d;
    d.position=habitat.position+rotate_y(local.local_position,habitat.rotation_degrees);
    d.wall_position=d.position;
    d.normal=flat_normal(rotate_y(local.local_normal,habitat.rotation_degrees));
    d.generated=false;
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
    const CorridorDoor main=transformed_default_door(habitat,mesh);
    if(distance_xz(candidate.wall_position,main.wall_position)<ENTRANCE_MIN_SPACING) return false;
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.wall_position,ep->door.wall_position)<ENTRANCE_MIN_SPACING) return false;
            if(distance_xz(candidate.position,ep->door.position)<ENTRANCE_MIN_SPACING) return false;
        }
    }
    return true;
}

Vec3 local_wall_point(const AttachmentExtents& e,Vec3 local_normal,float tangent){
    if(std::abs(local_normal.x)>0.5f){
        const float z=std::clamp(tangent,-e.half_z*0.55f,e.half_z*0.55f);
        float x=e.half_x;
        if(e.rounded_xz){
            const float q=std::clamp(z/e.half_z,-0.80f,0.80f);
            x=e.half_x*std::sqrt(std::max(0.18f,1.0f-q*q));
        }
        return {local_normal.x>0.0f?x:-x,1.18f,z};
    }
    const float x=std::clamp(tangent,-e.half_x*0.55f,e.half_x*0.55f);
    float z=e.half_z;
    if(e.rounded_xz){
        const float q=std::clamp(x/e.half_x,-0.80f,0.80f);
        z=e.half_z*std::sqrt(std::max(0.18f,1.0f-q*q));
    }
    return {x,1.18f,local_normal.z>0.0f?z:-z};
}

std::optional<CorridorDoor> generated_face_door(std::size_t habitat_index,
                                                const PlacedHabitat& habitat,
                                                const HabitatMesh& mesh,
                                                Vec3 target,
                                                const std::vector<CorridorConnection>& existing){
    const AttachmentExtents ext=attachment_extents(habitat.kind);
    const Vec3 target_dir=flat_normal(target-habitat.position);
    const std::array<Vec3,4> normals={Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}};
    const std::array<float,7> slots={0.0f,0.30f,-0.30f,0.50f,-0.50f,0.68f,-0.68f};
    bool found=false;
    float best=std::numeric_limits<float>::max();
    CorridorDoor result{};
    for(const Vec3 ln:normals){
        const Vec3 wn=flat_normal(rotate_y(ln,habitat.rotation_degrees));
        const float facing_penalty=(1.0f-dot(wn,target_dir))*4.0f;
        const float tangent_extent=std::abs(ln.x)>0.5f?ext.half_z:ext.half_x;
        for(float factor:slots){
            CorridorDoor d;
            d.wall_position=habitat.position+rotate_y(local_wall_point(ext,ln,factor*tangent_extent),habitat.rotation_degrees);
            d.normal=wn;
            d.position=d.wall_position+d.normal*GENERATED_ENTRANCE_DEPTH;
            d.generated=true;
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
    out.tangent=flat_normal(other-out.position);
    if(length(out.tangent)<0.001f) out.tangent={1,0,0};
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<existing.size()){
        // Keep tangent aimed toward the new route; the junction hardware itself
        // wraps the existing corridor at the exact selected midpoint.
        out.position.y=0;
    }
    return out;
}

std::optional<CorridorEndpoint> resolve_habitat(const CorridorBuildAnchor& anchor,Vec3 other,
                                                const std::vector<PlacedHabitat>& habitats,
                                                const HabitatModelLibrary& library,
                                                const std::vector<CorridorConnection>& existing){
    if(anchor.index>=habitats.size()) return std::nullopt;
    const auto& habitat=habitats[anchor.index];
    const auto& mesh=library.mesh(habitat.kind);
    CorridorDoor d=transformed_default_door(habitat,mesh);
    const Vec3 toward=flat_normal(other-d.position);
    const bool facing=length(toward)>0.001f && dot(d.normal,toward)>0.45f;
    const bool available=!door_occupied(anchor.index,d,existing);
    if(!facing || !available){
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
