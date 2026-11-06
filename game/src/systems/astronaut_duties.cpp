#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace game {
using namespace eng;
using namespace world_math;

namespace {
bool ready(const SupplyState* air){
    return air && air->power_fraction>=.99 && air->breathability()>=kCrewBreathableAir;
}

bool role_facility(std::size_t role,HabitatKind kind){
    switch(role){
        case 0:return kind==HabitatKind::CantineDome;
        case 1:return kind==HabitatKind::OxygenGeneratorDome || kind==HabitatKind::Factory ||
                      kind==HabitatKind::ProcessingFacility || kind==HabitatKind::Manufactory;
        case 2:return kind==HabitatKind::BioDome;
        case 3:return kind==HabitatKind::SickBay;
        default:return false;
    }
}

void order(AstronautTestActor& actor,AstronautActivity activity,std::optional<Vec3> destination={}){
    actor.activity=activity;
    actor.destination=destination;
    actor.entry_step=2;
    actor.route.clear();
    actor.route_step=0;
    actor.retry_seconds=0.f;
    actor.order_timeout=0.f;
    if(activity!=AstronautActivity::ToBed && activity!=AstronautActivity::Sleeping)actor.bed_owner=0;
}
}

AstronautRestPose GameApp::astronaut_rest_pose(const AstronautTestActor& actor) const {
    if(actor.activity==AstronautActivity::Sleeping){
        for(const auto& h:placed_habitats_){
            if(h.placement_sequence!=actor.bed_owner || h.kind!=HabitatKind::CrewQuarters || !h.construction.complete())continue;
            const auto& bunks=habitat_models_.mesh(h).crew_bunks;
            if(actor.bed_slot<bunks.size())
                return {h.position+rotate_y(bunks[actor.bed_slot].feet,h.rotation_degrees),h.rotation_degrees+90.f};
        }
    }
    const auto feet=astronaut_resting(actor)?
        actor.position-rotate_y({0,0,.65f},actor.heading_degrees):actor.position;
    return {feet,actor.heading_degrees};
}

void GameApp::assign_astronaut_duties(float dt){
    if(dt<=0.f)return;
    astronaut_schedule_seconds_-=dt;
    for(auto& actor:test_astronauts_){
        actor.decision_delay=std::max(0.f,actor.decision_delay-dt);
        if(actor.destination && actor.activity!=AstronautActivity::Manual){
            if(actor.speed>.05f && !actor.waiting_for_door)actor.order_timeout=0.f;
            else actor.order_timeout+=dt;
        }
        if(actor.health<=0.f && actor.activity!=AstronautActivity::Knockout)order(actor,AstronautActivity::Knockout);
    }
    if(astronaut_schedule_seconds_>0.f)return;
    astronaut_schedule_seconds_=.65f;

    // Join the passenger corridors only. Supply pipes do not grant access.
    const auto& corridors=corridor_network_.connections();
    const std::size_t habitat_count=placed_habitats_.size(),resource_count=placed_resources_.size();
    const std::size_t corridor_start=habitat_count+resource_count;
    const std::size_t count=corridor_start+corridors.size();
    std::vector<std::size_t> groups(count);
    std::iota(groups.begin(),groups.end(),0);
    const auto group=[&](std::size_t n){while(groups[n]!=n)n=groups[n];return n;};
    for(std::size_t i=0;i<corridors.size();++i){
        if(!corridors[i].construction.complete())continue;
        for(const auto* e:{&corridors[i].endpoint_a,&corridors[i].endpoint_b}){
            std::size_t n=count;
            if(e->kind==CorridorEndpointKind::Habitat && e->index<habitat_count)n=e->index;
            if(e->kind==CorridorEndpointKind::Resource && e->index<resource_count)n=habitat_count+e->index;
            if(e->kind==CorridorEndpointKind::Corridor && e->index<corridors.size() &&
               corridors[e->index].construction.complete())n=corridor_start+e->index;
            if(n<count)groups[group(n)]=group(corridor_start+i);
        }
    }
    std::vector<bool> accessible(count);
    for(std::size_t i=0;i<habitat_count;++i)
        if(placed_habitats_[i].kind==HabitatKind::AirLock && placed_habitats_[i].construction.complete() &&
           airlock_powered(i) && habitat_has_complete_corridor_connection(i))accessible[group(i)]=true;
    const auto node=[&](const SupplyState* air){
        for(std::size_t i=0;i<habitat_count;++i)if(air==&placed_habitats_[i].supply)return i;
        for(std::size_t i=0;i<resource_count;++i)if(air==&placed_resources_[i].supply)return habitat_count+i;
        for(std::size_t i=0;i<corridors.size();++i)if(air==&corridors[i].supply)return corridor_start+i;
        return count;
    };

    bool entry_in_progress=false;
    for(const auto& crew:test_astronauts_){
        bool in_lock=false;
        const auto* air=astronaut_air_at(crew.position,in_lock);
        entry_in_progress|=crew.destination && (!air || in_lock) &&
            (crew.activity==AstronautActivity::Duty || crew.activity==AstronautActivity::Shelter ||
             crew.activity==AstronautActivity::ToBed) && crew.order_timeout<16.f;
    }
    for(std::size_t i=0;i<test_astronauts_.size();++i){
        auto& actor=test_astronauts_[i];
        if(actor.health<=0.f)continue;
        bool in_lock=false;
        const auto* air=astronaut_air_at(actor.position,in_lock);
        const auto here=node(air);
        const auto can_reach=[&](std::size_t n){
            return here<count?group(here)==group(n):accessible[group(n)];
        };
        const auto enter_via_airlock=[&](std::size_t destination_node){
            if(air)return;
            float best=std::numeric_limits<float>::infinity();
            for(std::size_t h=0;h<habitat_count;++h){
                const auto& habitat=placed_habitats_[h];
                if(habitat.kind!=HabitatKind::AirLock || group(h)!=group(destination_node) ||
                   !habitat.construction.complete() || !airlock_powered(h))continue;
                for(const auto& door:habitat_doors(h)){
                    if(door.entrance_index!=-4)continue;
                    const Vec3 outside=door.wall_position+door.normal*1.75f;
                    const float d=distance_xz(actor.position,outside);
                    if(d>=best)continue;
                    for(const auto& c:corridors){
                        if(!c.construction.complete())continue;
                        for(const auto* e:{&c.endpoint_a,&c.endpoint_b}){
                            if(e->kind!=CorridorEndpointKind::Habitat || e->index!=h)continue;
                            if(!first_person_move_allowed(outside,outside,CrewTraversal::Plan))continue;
                            Vec3 inside=e->position;
                            const auto doors=destination_node<habitat_count?habitat_doors(destination_node):
                                storage_dome_doors(destination_node-habitat_count);
                            float dock_distance=std::numeric_limits<float>::infinity();
                            for(const auto& dock:doors){
                                Vec3 point=dock.wall_position-dock.normal*.85f;
                                point.y=first_person_floor_height(point);
                                const float distance=distance_xz(point,outside);
                                if(distance>=dock_distance || !first_person_move_allowed(point,point,CrewTraversal::Plan))continue;
                                inside=point;
                                dock_distance=distance;
                            }
                            if(!first_person_move_allowed(inside,inside,CrewTraversal::Plan))continue;
                            actor.entry_waypoints={outside,inside};
                            actor.entry_step=0;
                            best=d;
                        }
                    }
                }
            }
        };
        const auto spot_free=[&](Vec3 p){
            for(const auto& other:test_astronauts_){
                if(&other==&actor)continue;
                if(other.activity!=AstronautActivity::Sleeping && distance_xz(p,other.position)<kIndoorAstronautSeparation)return false;
                if(other.destination && distance_xz(p,*other.destination)<kIndoorAstronautSeparation)return false;
            }
            return first_person_move_allowed(p,p,CrewTraversal::Plan);
        };
        const auto owns_bed=[&]{
            return std::any_of(placed_habitats_.begin(),placed_habitats_.end(),[&](const auto& h){
                return h.placement_sequence==actor.bed_owner && h.kind==HabitatKind::CrewQuarters &&
                       h.construction.complete() && actor.bed_slot<crew_quarters_bed_count(h.size);
            });
        };
        if(actor.activity==AstronautActivity::Sleeping){
            if(owns_bed() && ready(air) && (actor.fatigue>.12f || !spot_free(actor.position)))continue;
            order(actor,AstronautActivity::Idle);
        }
        if(actor.activity==AstronautActivity::Knockout){
            if(actor.fatigue>.38f)continue;
            order(actor,AstronautActivity::Idle);
        }
        if(actor.activity==AstronautActivity::ToBed && !actor.destination && owns_bed() && ready(air)){
            order(actor,AstronautActivity::Sleeping);
            actor.speed=0.f;
            continue;
        }
        if(actor.activity==AstronautActivity::ToBed && !owns_bed())order(actor,AstronautActivity::Idle);
        // Turn back while the remaining air still covers the return trip.
        const float cache_distance=std::hypot(actor.position.x,actor.position.z)-kCrewCacheRadius;
        const float suit_range=(float)(240.0*actor.suit_oxygen_kg/kCrewOxygenKgPerDay)*astronaut_walk_speed(true);
        bool target_lock=false;
        const bool shelter_in_range=actor.destination && ready(astronaut_air_at(*actor.destination,target_lock)) &&
                                    distance_xz(actor.position,*actor.destination)+5.f<suit_range;
        if(actor.fatigue<kCrewExhausted && !air && cache_distance>0.f && suit_range<cache_distance+8.f &&
           !shelter_in_range && astronaut_cache_.oxygen_kg>0){
            if(actor.activity!=AstronautActivity::Resupply){
                constexpr std::array<Vec3,4> spots{{{-1.65f,0,.20f},{1.65f,0,-.20f},{-3.75f,0,-1.45f},{3.75f,0,-1.45f}}};
                order(actor,AstronautActivity::Resupply,spots[i]);
            }
            continue;
        }
        if(actor.destination && actor.activity==AstronautActivity::Manual && actor.fatigue<kCrewExhausted)continue;
        if(actor.destination && actor.activity==AstronautActivity::Resupply &&
           actor.suit_oxygen_kg<.22 && astronaut_cache_.oxygen_kg>0)continue;
        if(actor.destination && actor.order_timeout<16.f){
            bool target_lock=false;
            if(ready(astronaut_air_at(*actor.destination,target_lock)) &&
               (actor.fatigue<kCrewExhausted || actor.activity==AstronautActivity::ToBed))continue;
        }
        std::optional<Vec3> failed;
        if(actor.destination && actor.order_timeout>=16.f)failed=actor.destination;
        if(actor.destination)order(actor,AstronautActivity::Idle);
        const bool queued=!air && entry_in_progress;
        if(actor.fatigue<kCrewExhausted && (actor.decision_delay>0.f || queued))continue;

        if(actor.fatigue>=kCrewTired){
            float best=std::numeric_limits<float>::infinity();
            std::optional<Vec3> target;
            std::uint64_t owner=0;
            std::size_t bed_habitat=0;
            std::uint8_t slot=0;
            for(std::size_t h=0;h<habitat_count;++h){
                const auto& room=placed_habitats_[h];
                if(room.kind!=HabitatKind::CrewQuarters || !room.placement_sequence || !room.construction.complete() ||
                   !ready(&room.supply) || !can_reach(h))continue;
                const auto& mesh=habitat_models_.mesh(room);
                for(int bed=0;bed<crew_quarters_bed_count(room.size) && bed<(int)mesh.crew_bunks.size();++bed){
                    const bool reserved=std::any_of(test_astronauts_.begin(),test_astronauts_.end(),[&](const auto& other){
                        return &other!=&actor && other.bed_owner==room.placement_sequence && other.bed_slot==bed;
                    });
                    if(reserved)continue;
                    Vec3 p=room.position+rotate_y(mesh.crew_bunks[bed].access,room.rotation_degrees);
                    p.y=first_person_floor_height(p);
                    const float d=distance_xz(actor.position,p);
                    if(d>=best || (failed && distance_xz(*failed,p)<.15f) || !spot_free(p))continue;
                    best=d;target=p;owner=room.placement_sequence;bed_habitat=h;slot=(std::uint8_t)bed;
                }
            }
            if(target){
                // A free bed remains available while another crew member
                // is cycling the exterior lock.
                if(queued)continue;
                order(actor,AstronautActivity::ToBed,target);
                actor.bed_owner=owner;actor.bed_slot=slot;
                enter_via_airlock(bed_habitat);
                entry_in_progress|=!air;
                continue;
            }
        }

        if(actor.fatigue>=kCrewExhausted){
            order(actor,AstronautActivity::Knockout);
            actor.speed=0.f;
            continue;
        }
        const bool at_duty=ready(air) && here<count &&
            (here<habitat_count?role_facility(i,placed_habitats_[here].kind):
             i==0 && here<corridor_start && placed_resources_[here-habitat_count].kind==ResourceKind::StorageDome);
        if(at_duty){
            actor.activity=AstronautActivity::Duty;
            continue;
        }

        float best=std::numeric_limits<float>::infinity();
        std::optional<Vec3> target;
        bool duty=false;
        std::size_t target_node=0;
        const auto consider=[&](std::size_t n,const auto& structure,float width,float depth,bool preferred){
            if(!structure.construction.complete() || !ready(&structure.supply) || !can_reach(n))return;
            if(!preferred && ready(air) && !in_lock)return;
            const float penalty=preferred?0.f:200.f;
            const auto doors=n<habitat_count?habitat_doors(n):storage_dome_doors(n-habitat_count);
            for(float x:{-.30f,.30f,0.f,-.15f,.15f})for(float z:{-.52f,.52f,-.38f,.38f,-depth*.22f,depth*.22f,0.f}){
                Vec3 p=structure.position+rotate_y({width*x,0.f,z},structure.rotation_degrees);
                // Idle crew stand off the travel line and clear of door leaves.
                if(std::any_of(doors.begin(),doors.end(),[&](const auto& door){
                    return distance_xz(p,door.wall_position)<1.55f;
                }))continue;
                const float cost=penalty+distance_xz(actor.position,p)+std::abs(x)*12.f+(z==0.f?6.f:0.f);
                if(cost>=best || (failed && distance_xz(*failed,p)<.3f))continue;
                bool target_lock=false;
                if(astronaut_air_at(p,target_lock)!=&structure.supply)continue;
                p.y=first_person_floor_height(p);
                if(!spot_free(p))continue;
                best=cost;target=p;duty=preferred;target_node=n;
            }
        };
        for(std::size_t h=0;h<habitat_count;++h){
            const auto& room=placed_habitats_[h];
            if(room.kind==HabitatKind::AirLock)continue;
            const auto& mesh=habitat_models_.mesh(room);
            consider(h,room,mesh.footprint_x,mesh.footprint_z,role_facility(i,room.kind));
        }
        for(std::size_t r=0;r<resource_count;++r){
            const auto& room=placed_resources_[r];
            if(room.kind!=ResourceKind::StorageDome)continue;
            const auto& mesh=resource_models_.mesh(room);
            consider(habitat_count+r,room,mesh.footprint_x,mesh.footprint_z,i==0);
        }
        if(target){
            order(actor,duty?AstronautActivity::Duty:AstronautActivity::Shelter,target);
            enter_via_airlock(target_node);
            entry_in_progress|=!air;
        }else actor.activity=ready(air)?AstronautActivity::Shelter:AstronautActivity::Idle;
    }
}

} // namespace game
