#pragma once
#include "game/world/pressure_door.hpp"
#include "game/util/world_math.hpp"

namespace game::airlock {
inline constexpr float kPartitionRatio=1.6f/5.25f;
inline constexpr float kPressureRate=0.4f; // fraction of colony pressure per real second
inline constexpr float kApproach=1.15f;
struct Cycle {
    std::uint64_t identity{0};
    float pressure{1},target{1},previous_x{0};
    int direction{0},requested{-1};
    bool tracking{false},equalizing{false},sealing{false};
};
struct DoorSample {PressureDoorState* state{};float x{};bool internal{false};};
inline bool internal(const CorridorDoor& door){return door.entrance_index==-2 || door.entrance_index==-3;}
inline float partition_x(const HabitatMesh& mesh){
    // The three-chamber geometry is structural, not tied to a predefined hatch.
    // Standard Air Lock: 10 m long -> partitions at +/-1.6 m. Small scales in X/Z.
    return mesh.footprint_x*0.16f;
}
inline std::array<CorridorDoor,2> partitions(const PlacedHabitat& habitat,const HabitatMesh& mesh){
    std::array<CorridorDoor,2> result{};
    for(int i=0;i<2;++i){
        const float sign=i?1.f:-1.f;auto& d=result[i];
        d.position=d.wall_position=habitat.position+world_math::rotate_y({sign*partition_x(mesh),mesh.walkable_floor_y,0},habitat.rotation_degrees);
        d.normal=world_math::rotate_y({sign,0,0},habitat.rotation_degrees);d.entrance_index=-2-i;
    }
    return result;
}
// The finished airlock owns two fixed axial end hatches, plus its two
// internal partitions. All consumers use this same port definition.
inline CorridorDoor end_door(const PlacedHabitat& h,const HabitatMesh& m,int sign){
    CorridorDoor d;const float s=sign<0?-1.f:1.f;
    d.wall_position=d.position=h.position+world_math::rotate_y({s*m.dome_radii.x,m.walkable_floor_y,0},h.rotation_degrees);
    d.normal=world_math::rotate_y({s,0,0},h.rotation_degrees);
    d.generated=true;d.entrance_index=sign<0?-5:-6;
    const auto point=[&](float lateral,float y){
        const float x=s*m.dome_radii.x*(m.dome_radii.y==0?1.f:std::sqrt(std::max(0.f,1-lateral*lateral/(m.dome_radii.z*m.dome_radii.z))));
        return h.position+world_math::rotate_y({x,m.walkable_floor_y+y,lateral},h.rotation_degrees);
    };
    constexpr float half=geometry::kGeneratedEntranceHalfWidth;
    d.wall_profile.push_back(point(-s*half,-.15f));d.wall_profile.push_back(point(s*half,-.15f));
    for(int i=0;i<=10;++i){const float a=3.14159265358979323846f*i/10.f;
        d.wall_profile.push_back(point(s*std::cos(a)*half,geometry::kGeneratedEntranceHeight-geometry::kCorridorDeckY-geometry::kGeneratedEntranceCrownRise+std::sin(a)*geometry::kGeneratedEntranceCrownRise));
    }
    return d;
}
// Skin-to-seat adapter for a fixed end hatch. The skin is curved in plan;
// rendering a flat jamb at its tangent plane alone leaves an open visual gap.
inline std::vector<game::runtime::Vec3> fitted_collar(const CorridorDoor& door){
    using game::runtime::Vec3;
    std::vector<Vec3> out;if(door.wall_profile.size()<3)return out;
    const auto n=game::runtime::normalize(door.normal);const Vec3 side{-n.z,0,n.x};
    const auto inner=[&](Vec3 p){
        const auto delta=p-door.wall_position;
        const float u=game::runtime::dot(delta,side)/geometry::kGeneratedEntranceHalfWidth;
        const float v=(delta.y+.15f)/(geometry::kGeneratedEntranceHeight-.05f);
        return door.wall_position+side*(u*(geometry::kDoorClearWidth*.5f+.055f))+
            Vec3{0,std::clamp(v,0.f,1.f)*(geometry::kDoorClearHeight+.035f)-.015f,0}+n*.015f;
    };
    for(std::size_t i=0;i<door.wall_profile.size();++i){
        const auto a=door.wall_profile[i]-n*.045f,b=door.wall_profile[(i+1)%door.wall_profile.size()]-n*.045f;
        const auto c=inner(door.wall_profile[(i+1)%door.wall_profile.size()]),d=inner(door.wall_profile[i]);
        out.insert(out.end(),{a,b,c,a,c,d,c,b,a,d,c,a});
    }return out;
}
inline const char* phase(const Cycle& cycle){
    return cycle.sealing?"SEALING DOORS":cycle.equalizing?(cycle.target>0.5f?"EQUALIZING TO INSIDE":"EQUALIZING TO OUTSIDE"):
        cycle.requested>=0?"READY / PASS THROUGH":"SEALED / STANDBY";
}
inline const char* chamber(float local_x,float partition,int inside_sign){
    return std::abs(local_x)<partition?"CENTER PRESSURE CHAMBER":local_x*inside_sign>0?"INTERIOR ACCESS CABIN":"EXTERIOR ISOLATION CABIN";
}

// One controller owns all four seals. Request the next door in the direction
// of travel; doors just passed close even when still within approach range.
// This also permits retreat/backtracking without a one-way traversal script.
inline void advance(Cycle& cycle,std::vector<DoorSample>& doors,game::runtime::Vec3 feet,float wish_x,bool present,
                    int inside_sign,float radius,float dt,int explicit_request=-2){
    dt=std::clamp(dt,0.f,0.1f);
    cycle.requested=-1;cycle.equalizing=false;cycle.sealing=false;
    if(present){
        const float movement=cycle.tracking?feet.x-cycle.previous_x:0;
        if(std::abs(wish_x)>0.05f)cycle.direction=wish_x>0?1:-1;
        else if(std::abs(movement)>0.001f)cycle.direction=movement>0?1:-1;
        else if(!cycle.direction)cycle.direction=feet.x>0?-1:1;
        cycle.previous_x=feet.x;cycle.tracking=true;
    }else {cycle.tracking=false;cycle.direction=0;}
    std::vector<bool> occupied(doors.size(),false),sealed(doors.size(),true);
    float best=kApproach;int occupied_door=-1;
    for(std::size_t i=0;i<doors.size();++i){
        sealed[i]=pressure_door::sealed(doors[i].state->open_fraction);
        const float delta=doors[i].x-feet.x;
        occupied[i]=present && std::abs(delta)<radius+pressure_door::kThickness*0.5f+0.06f;
        if(occupied[i])occupied_door=(int)i;
        if(explicit_request==-2 && present && delta*cycle.direction>=-0.02f && std::abs(delta)<best){best=std::abs(delta);cycle.requested=(int)i;}
    }
    if(explicit_request>=-1)cycle.requested=explicit_request<int(doors.size())?explicit_request:-1;
    if(occupied_door>=0&&!sealed[std::size_t(occupied_door)])cycle.requested=occupied_door;
    bool all_sealed=std::all_of(sealed.begin(),sealed.end(),[](bool value){return value;});
    if(cycle.requested>=0){
        const auto& wanted=doors[(std::size_t)cycle.requested];
        if(wanted.internal){
            cycle.target=wanted.x*inside_sign>0?1.f:0.f;
            if(std::abs(cycle.pressure-cycle.target)>0.001f){
                if(all_sealed)cycle.pressure+=std::clamp(cycle.target-cycle.pressure,-kPressureRate*dt,kPressureRate*dt);
                cycle.equalizing=std::abs(cycle.pressure-cycle.target)>0.001f;
            }
        }
    }
    for(std::size_t i=0;i<doors.size();++i){
        bool others_sealed=true;for(std::size_t j=0;j<doors.size();++j)if(i!=j && !sealed[j])others_sealed=false;
        auto& state=*doors[i].state;
        const bool ready=!doors[i].internal || std::abs(cycle.pressure-(doors[i].x*inside_sign>0?1.f:0.f))<=0.001f;
        const bool requested=cycle.requested==(int)i && others_sealed && ready;
        // A body in a moving aperture holds that seal open. A closed seal
        // cannot bypass the interlock merely because a saved/debug pose is in it.
        const bool hold_body=occupied[i] && !sealed[i];
        if(!requested && !hold_body)state.hold_seconds=0;
        pressure_door::advance(state,dt,requested,hold_body);
        if(cycle.requested!=(int)i && !pressure_door::sealed(state.open_fraction))cycle.sealing=true;
    }
    if(cycle.requested>=0 && !all_sealed){
        for(std::size_t i=0;i<doors.size();++i)if((int)i!=cycle.requested && !sealed[i])cycle.sealing=true;
    }
}

// The entire partition is solid outside its human-sized aperture. This test
// prevents walking round a leaf, shoulder clipping, and jumping over the seal.
inline bool bulkhead_blocks(const CorridorDoor& door,float fraction,game::runtime::Vec3 from,game::runtime::Vec3 to,float radius,float height,float horizontal_scale=1.f){
    const auto n=door.normal;const game::runtime::Vec3 side{-n.z,0,n.x};
    const auto a=from-door.wall_position,b=to-door.wall_position;
    const float da=game::runtime::dot(a,n),db=game::runtime::dot(b,n),slab=radius+0.07f;
    if(std::min(da,db)>slab || std::max(da,db)<-slab)return false;
    float t0=0,t1=1;const float dv=db-da;
    if(std::abs(dv)>0.00001f){t0=std::max(0.f,std::min((-slab-da)/dv,(slab-da)/dv));t1=std::min(1.f,std::max((-slab-da)/dv,(slab-da)/dv));}
    const auto p=a+(b-a)*t0,q=a+(b-a)*t1;
    // Finite elliptical pressure disk, matching the horizontally scaled mesh.
    // An internal bulkhead must never become an invisible wall across the map.
    const float u=game::runtime::dot(p,side),v=game::runtime::dot(q,side);
    const float near_side=u*v<=0?0:std::max(0.f,std::min(std::abs(u),std::abs(v))-radius);
    constexpr float section=.94280904f; // sqrt(1 - (1.6 / 4.8)^2)
    const float z=near_side/(2.5f*section*horizontal_scale);
    const float bottom=std::min(p.y,q.y),top=std::max(p.y,q.y)+height;
    if(z>1.f || top<-.1f)return false;
    const float roof=2.9f+1.25f*section*std::sqrt(std::max(0.f,1-z*z));
    if(bottom>roof)return false;
    return std::max(std::abs(game::runtime::dot(p,side)),std::abs(game::runtime::dot(q,side)))+radius+0.015f>pressure_door::clear_half_width(fraction) ||
        std::min(p.y,q.y)<-0.1f || std::max(p.y,q.y)+height>geometry::kDoorClearHeight;
}
} // namespace game::airlock
