#include "game/game_app.hpp"
#include "game/robots/path_planner.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;

namespace {
constexpr float kArrivalDistance=.025f;
constexpr float kWalkAcceleration=2.6f;
constexpr float kWalkBraking=3.2f;

template<class Pose>
int astronaut_at(const std::array<AstronautTestActor,kAstronautRoles.size()>& crew,const CameraController& camera,
                 int x,int y,int width,int height,const Pose& rest_pose){
    if(width<=0 || height<=0)return -1;
    const auto m=camera.view_projection((float)width/height).m;
    const auto project=[&](Vec3 p,float& px,float& py,float& depth){
        depth=m[3]*p.x+m[7]*p.y+m[11]*p.z+m[15];
        if(depth<=.05f)return false;
        px=(.5f+.5f*(m[0]*p.x+m[4]*p.y+m[8]*p.z+m[12])/depth)*width;
        py=(.5f-.5f*(m[1]*p.x+m[5]*p.y+m[9]*p.z+m[13])/depth)*height;
        return true;
    };
    int picked=-1;
    float closest=std::numeric_limits<float>::infinity();
    for(std::size_t i=0;i<crew.size();++i){
        const auto& actor=crew[i];
        const auto pose=rest_pose(actor);
        const bool lying=astronaut_resting(actor);
        const Vec3 p=pose.feet+Vec3{0,lying?.26f:0.f,0};
        const Vec3 extent=lying?rotate_y({0,0,kAstronautMeshHeight},pose.heading_degrees):
                                 Vec3{0,kAstronautMeshHeight,0};
        float fx,fy,fd,hx,hy,hd;
        if(!project(p,fx,fy,fd) || !project(p+extent,hx,hy,hd))continue;
        const float pixels_per_metre=.5f*height/(std::tan(radians(27.5f))*std::min(fd,hd));
        const float radius=std::max(8.0f,.33f*pixels_per_metre);
        const float distance=point_segment_distance_xz({(float)x,0,(float)y},{fx,0,fy},{hx,0,hy});
        if(distance<=radius && fd<closest){
            closest=fd;
            picked=(int)i;
        }
    }
    return picked;
}

template<class ClearPoint,class ClearSegment,class MoveClear,class Floor>
void walk_to(AstronautTestActor& actor,float half_extent,float dt,bool indoors,
             const ClearPoint& clear_point,const ClearSegment& clear_segment,
             const MoveClear& move_clear,const Floor& floor){
    actor.waiting_for_door=false;
    if(!actor.destination){
        actor.speed=std::max(0.f,actor.speed-kWalkBraking*dt);
        return;
    }
    actor.retry_seconds=std::max(0.f,actor.retry_seconds-dt);
    const bool entering=actor.entry_step<actor.entry_waypoints.size();
    Vec3 destination=entering?actor.entry_waypoints[actor.entry_step]:*actor.destination;
    destination.y=floor(destination);
    if(distance_xz(actor.position,destination)<=kArrivalDistance && move_clear(actor.position,destination)){
        actor.position=destination;
        actor.route.clear();
        actor.route_step=0;
        if(entering){
            ++actor.entry_step;
            actor.retry_seconds=0.f;
            return;
        }
        actor.destination.reset();
        if(actor.activity==AstronautActivity::Manual){
            actor.activity=AstronautActivity::Idle;
            actor.decision_delay=3.f;
        }
        return;
    }
    if(actor.route_step>=actor.route.size() && actor.retry_seconds<=0.f){
        actor.retry_seconds=.65f;
        actor.route_step=0;
        robots::PathPlanRequest request;
        request.start=actor.position;
        request.goal=destination;
        request.world_half_extent=half_extent;
        request.corridor_half_width=18.0f;
        request.cell_size=indoors?.32f:.48f;
        request.max_expansions=1750;
        robots::PathPlannerCallbacks callbacks;
        callbacks.floor_height=floor;
        callbacks.point_clear=clear_point;
        callbacks.segment_clear=clear_segment;
        robots::plan_corridor_path(request,callbacks,actor.route);
    }
    if(actor.route_step>=actor.route.size()){
        actor.speed=std::max(0.f,actor.speed-kWalkBraking*dt);
        return;
    }

    Vec3 delta=actor.route[actor.route_step]-actor.position;
    delta.y=0.f;
    const float remaining=length(delta);
    if(remaining<kArrivalDistance){
        ++actor.route_step;
        return;
    }
    const float target_heading=std::atan2(delta.x,delta.z)*57.2957795f;
    const float turn=std::remainder(target_heading-actor.heading_degrees,360.f);
    const float eased_turn=turn*-std::expm1(-14.f*dt);
    actor.heading_degrees=std::fmod(actor.heading_degrees+std::clamp(eased_turn,-240.f*dt,240.f*dt)+360.f,360.f);
    const float error=std::abs(std::remainder(target_heading-actor.heading_degrees,360.f));
    const float facing=std::clamp(1.f-error/50.f,0.f,1.f);
    const float walk_speed=astronaut_walk_speed(actor.suited);
    float junction_speed=0.f;
    if(actor.route_step+1<actor.route.size()){
        Vec3 outgoing=actor.route[actor.route_step+1]-actor.route[actor.route_step];
        outgoing.y=0.f;
        const float span=length(outgoing);
        if(span>kArrivalDistance){
            // Keep momentum through gentle bends, with time to steer at
            // sharper corners. The final point of each air-lock leg still stops.
            const float alignment=dot(delta,outgoing)/(remaining*span);
            const float corner=std::clamp((alignment-.45f)/.52f,0.f,1.f);
            junction_speed=std::min(walk_speed*corner,std::sqrt(2.f*kWalkBraking*span));
        }
    }
    const bool last_point=actor.route_step+1==actor.route.size();
    const float braking_distance=last_point?remaining-kArrivalDistance*.75f:remaining;
    const float speed_limit=std::sqrt(junction_speed*junction_speed+2.f*kWalkBraking*braking_distance);
    const float desired=std::min(walk_speed,speed_limit)*facing;
    actor.speed+=std::clamp(desired-actor.speed,-kWalkBraking*dt,kWalkAcceleration*dt);
    const float traveled=std::min(remaining,actor.speed*dt)*std::min(1.f,2.f*facing);
    if(traveled<=0.f)return;
    Vec3 next=actor.position+delta*(traveled/remaining);
    next.y=floor(next);
    if(move_clear(actor.position,next)){
        actor.position=next;
        actor.stride_phase=std::fmod(actor.stride_phase+traveled/astronaut_stride(actor.suited),1.f);
        if(remaining-traveled<=kArrivalDistance && actor.route_step+1<actor.route.size())
            ++actor.route_step;
    }else{
        actor.speed=std::max(0.f,actor.speed-kWalkBraking*dt);
        actor.waiting_for_door=clear_segment(actor.position,next);
        if(!actor.waiting_for_door){
            actor.route.clear();
            actor.route_step=0;
            actor.retry_seconds=.25f;
        }
    }
}
}

void GameApp::reset_astronaut_test(){
    test_astronauts_={};
    astronaut_cache_={};
    astronaut_schedule_seconds_=0.f;
    test_astronauts_[0].position={-1.65f,0.f,.20f};
    test_astronauts_[1].position={1.65f,0.f,-.20f};
    test_astronauts_[2].position={-3.75f,0.f,-1.45f};
    test_astronauts_[3].position={3.75f,0.f,-1.45f};
    selected_astronaut_=-1;
    camera_.restore({10.0f,28.0f,12.0f,{0.f,.70f,0.f}});
}

bool GameApp::handle_astronaut_click(const InputState& input){
    if(!input.left_pressed || input.middle_down || input.right_down)return false;
    const int picked=astronaut_at(test_astronauts_,camera_,input.mouse_x,input.mouse_y,
                                  viewport_width_,viewport_height_,[&](const auto& actor){return astronaut_rest_pose(actor);});
    if(picked>=0){
        selected_astronaut_=picked;
        clear_world_selection();
        return true;
    }
    if(selected_astronaut_<0)return false;
    Vec3 hit{};
    if(!camera_.intersect_playfield(input.mouse_x,input.mouse_y,viewport_width_,viewport_height_,
                                    playfield_.size(),hit))return false;
    const float half=playfield_.size()*.5f-kAstronautFieldMargin;
    hit.x=std::clamp(hit.x,-half,half);
    hit.z=std::clamp(hit.z,-half,half);
    hit.y=first_person_floor_height(hit);
    auto& actor=test_astronauts_[selected_astronaut_];
    if(astronaut_resting(actor))return true;
    actor.activity=AstronautActivity::Manual;
    actor.construction_job=0;
    actor.bed_owner=0;
    actor.entry_step=2;
    actor.destination=hit;
    actor.route.clear();
    actor.route_step=0;
    actor.retry_seconds=0.f;
    return true;
}

void GameApp::update_astronaut_suits(){
    for(auto& actor:test_astronauts_){
        bool airlock=false;
        const auto* air=astronaut_air_at(actor.position,airlock);
        // Removing the suit requires full indicated pressure. Once removed,
        // allow ordinary breathing draw without cycling suits on every tick.
        const bool suited=airlock || !air ||
                          air->breathability()<(actor.suited?kCrewFullAir:kCrewBreathableAir);
        if(suited!=actor.suited){
            actor.suited=suited;
            actor.speed=std::min(actor.speed,astronaut_walk_speed(suited));
        }
    }
}

bool GameApp::astronaut_placement_clear(Vec3 center,float rotation,float width,float depth) const {
    if(!astronaut_test_active())return true;
    const auto hull=footprint(center,rotation,width,depth);
    for(const auto& actor:test_astronauts_)
        if(overlaps(hull,footprint(actor.position,0.f,.90f,.90f),0.f))return false;
    return true;
}

void GameApp::update_astronaut_test(float dt){
    const float half=playfield_.size()*.5f-kAstronautFieldMargin;
    const bool has_obstacles=!placed_habitats_.empty() || !placed_resources_.empty() ||
                             !corridor_network_.connections().empty();
    const auto floor=[&](Vec3 p){return has_obstacles?first_person_floor_height(p):0.f;};
    update_astronaut_suits();
    assign_astronaut_duties(dt);

    float time=std::clamp(dt,0.f,.24f);
    while(time>0.f){
        const float step=std::min(time,.02f);
        std::array<bool,kAstronautRoles.size()> inside{};
        if(has_obstacles)for(std::size_t i=0;i<inside.size();++i){
            bool lock=false;
            inside[i]=astronaut_air_at(test_astronauts_[i].position,lock) && !lock;
        }
        for(std::size_t i=0;i<test_astronauts_.size();++i){
            auto& actor=test_astronauts_[i];
            if(astronaut_resting(actor)){actor.tool_carry_blend=0.f;continue;}
            const float carry=actor.activity==AstronautActivity::Construction && actor.construction_job && actor.destination?1.f:0.f;
            actor.tool_carry_blend+=std::clamp(carry-actor.tool_carry_blend,-3.f*step,3.f*step);
            if(!actor.destination){actor.speed=std::max(0.f,actor.speed-kWalkBraking*step);continue;}
            actor.passing_cooldown=std::max(0.f,actor.passing_cooldown-step);
            const bool indoors=inside[i];
            const auto crew_clear=[&](Vec3 a,Vec3 b,bool planning=false){
                if(std::abs(b.x)>half || std::abs(b.z)>half)return false;
                for(std::size_t j=0;j<test_astronauts_.size();++j){
                    const auto& other=test_astronauts_[j];
                    if(j==i || other.activity==AstronautActivity::Sleeping)continue;
                    // Distant moving crew are handled by the local passing and
                    // swept-step checks. A following actor can plan the same aisle.
                    if(planning && other.destination && distance_xz(other.position,actor.position)>2.5f)continue;
                    const bool shared_interior=indoors && inside[j];
                    const float spacing=shared_interior?kIndoorAstronautSeparation:kAstronautSeparation;
                    const float initial=distance_xz(other.position,a);
                    const float final=distance_xz(other.position,b);
                    if(initial<spacing && final>initial && dot(a-other.position,b-a)>=0.f)continue;
                    if(final<spacing || point_segment_distance_xz(other.position,a,b)<spacing)return false;
                }
                return true;
            };
            const auto clear_point=[&](Vec3 p){
                return crew_clear(p,p,true) && (!has_obstacles || first_person_move_allowed(p,p,CrewTraversal::Plan));
            };
            const auto clear_segment=[&](Vec3 a,Vec3 b){
                if(!crew_clear(a,b,true))return false;
                if(!has_obstacles)return true;
                // The shared collision routine is tuned for short steps.
                const int slices=std::max(1,(int)std::ceil(distance_xz(a,b)/.65f));
                const Vec3 origin=a;
                for(int i=1;i<=slices;++i){
                    Vec3 next=origin+(b-origin)*((float)i/slices);
                    next.y=floor(next);
                    if(!first_person_move_allowed(a,next,CrewTraversal::Plan))return false;
                    a=next;
                }
                return true;
            };
            const auto move_clear=[&](Vec3 a,Vec3 b){
                return crew_clear(a,b) && (!has_obstacles || first_person_move_allowed(a,b,CrewTraversal::Astronaut));
            };
            if(indoors && actor.destination && actor.passing_cooldown<=0.f && actor.route_step<actor.route.size()){
                Vec3 direction=actor.route[actor.route_step]-actor.position;
                direction.y=0.f;
                direction=normalize(direction);
                const Vec3 right{direction.z,0.f,-direction.x};
                for(const auto& other:test_astronauts_){
                    if(&other==&actor || !other.destination || astronaut_resting(other))continue;
                    const Vec3 gap=other.position-actor.position;
                    const Vec3 opposing=*other.destination-other.position;
                    if(length(gap)>2.5f || dot(gap,direction)<.55f ||
                       std::abs(dot(gap,right))>.48f || dot(normalize(opposing),direction)>-.65f)continue;
                    for(float offset:{.58f,.38f}){
                        Vec3 lane=actor.position+right*offset+direction*.30f;
                        lane.y=floor(lane);
                        if(!clear_segment(actor.position,lane))continue;
                        actor.route.insert(actor.route.begin()+actor.route_step,lane);
                        actor.passing_cooldown=2.5f;
                        break;
                    }
                    break;
                }
            }
            walk_to(actor,half,step,indoors,clear_point,clear_segment,move_clear,floor);
        }
        time-=step;
    }
    advance_astronaut_construction(std::clamp(dt,0.f,.24f));
    update_astronaut_suits();
}

} // namespace game
