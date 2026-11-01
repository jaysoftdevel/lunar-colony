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
inline float partition_x(const HabitatMesh& mesh){return std::abs(mesh.entrances.front().local_position.x)*kPartitionRatio;}
inline std::array<CorridorDoor,2> partitions(const PlacedHabitat& habitat,const HabitatMesh& mesh){
    std::array<CorridorDoor,2> result{};
    for(int i=0;i<2;++i){
        const float sign=i?1.f:-1.f;auto& d=result[i];
        d.position=d.wall_position=habitat.position+world_math::rotate_y({sign*partition_x(mesh),mesh.walkable_floor_y,0},habitat.rotation_degrees);
        d.normal=world_math::rotate_y({sign,0,0},habitat.rotation_degrees);d.entrance_index=-2-i;
    }
    return result;
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
inline void advance(Cycle& cycle,std::vector<DoorSample>& doors,eng::Vec3 feet,float wish_x,bool present,
                    int inside_sign,float radius,float dt){
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
        if(present && delta*cycle.direction>=-0.02f && std::abs(delta)<best){best=std::abs(delta);cycle.requested=(int)i;}
    }
    if(occupied_door>=0)cycle.requested=occupied_door;
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
inline bool bulkhead_blocks(const CorridorDoor& door,float fraction,eng::Vec3 from,eng::Vec3 to,float radius,float height,float horizontal_scale=1.f){
    const auto n=door.normal;const eng::Vec3 side{-n.z,0,n.x};
    const auto a=from-door.wall_position,b=to-door.wall_position;
    const float da=eng::dot(a,n),db=eng::dot(b,n),slab=radius+0.07f;
    if(std::min(da,db)>slab || std::max(da,db)<-slab)return false;
    float t0=0,t1=1;const float dv=db-da;
    if(std::abs(dv)>0.00001f){t0=std::max(0.f,std::min((-slab-da)/dv,(slab-da)/dv));t1=std::min(1.f,std::max((-slab-da)/dv,(slab-da)/dv));}
    const auto p=a+(b-a)*t0,q=a+(b-a)*t1;
    // Finite elliptical pressure disk, matching the horizontally scaled mesh.
    // An internal bulkhead must never become an invisible wall across the map.
    const float u=eng::dot(p,side),v=eng::dot(q,side);
    const float near_side=u*v<=0?0:std::max(0.f,std::min(std::abs(u),std::abs(v))-radius);
    const float near_height=std::max({0.f,std::min(p.y,q.y)-1.75f,1.75f-(std::max(p.y,q.y)+height)});
    const float z=near_side/(2.12f*horizontal_scale),y=near_height/2.12f;
    if(z*z+y*y>1.f)return false;
    return std::max(std::abs(eng::dot(p,side)),std::abs(eng::dot(q,side)))+radius+0.015f>pressure_door::clear_half_width(fraction) ||
        std::min(p.y,q.y)<-0.1f || std::max(p.y,q.y)+height>geometry::kDoorClearHeight;
}
} // namespace game::airlock
