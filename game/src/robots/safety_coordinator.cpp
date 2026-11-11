#include "game/robots/safety_coordinator.hpp"
#include "RVOSimulator.h"
#include "Vector2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace game::robots {
namespace {
using eng::Vec3;
constexpr float PI=3.14159265358979323846f;

float dist_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x;
    const float dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}
float dot_xz(Vec3 a,Vec3 b){ return a.x*b.x+a.z*b.z; }
float wrap_degrees(float a){
    while(a>180.0f) a-=360.0f;
    while(a<-180.0f) a+=360.0f;
    return a;
}
float approach(float value,float target,float max_delta){
    if(value<target) return std::min(target,value+max_delta);
    return std::max(target,value-max_delta);
}
float heading_between(Vec3 from,Vec3 to,float fallback){
    const float dx=to.x-from.x;
    const float dz=to.z-from.z;
    if(dx*dx+dz*dz<1.0e-8f) return fallback;
    return std::atan2(dx,dz)*180.0f/PI;
}
Vec3 forward_from_heading(float degrees){
    const float a=degrees*PI/180.0f;
    return {std::sin(a),0.0f,std::cos(a)};
}
float planner_phase_offset_seconds(std::size_t robot_id){
    // Spread proposal refreshes across time so many robots do not replan in
    // the same frame. This lowers frame-time spikes without changing contract
    // safety because already-certified motion remains authoritative at 100 Hz.
    return 0.016f*(float)(robot_id%5u);
}

std::size_t configured_route_search_lanes(std::size_t available_lanes){
    const std::size_t clamped_available=std::max<std::size_t>(1,available_lanes);
    static const std::size_t env_override=[](){
        if(const char* env=std::getenv("GAME_ROUTE_SEARCH_LANES")){
            char* end=nullptr;
            const long parsed=std::strtol(env,&end,10);
            if(end!=env && parsed>0) return (std::size_t)parsed;
        }
        return (std::size_t)0;
    }();
    if(env_override>0)
        return std::max<std::size_t>(1,std::min(clamped_available,env_override));

    // Default to a bounded subset of lanes for heavy global route searches.
    // This keeps aggregate CPU utilization high while avoiding all-core
    // microbursts that can starve rendering and produce visible stutter.
    // The cap is deliberately small and independent of the host size: when
    // three or more rovers re-plan against each other's current positions in
    // the same call, the navigation_construction fleet livelocks (rovers
    // endlessly re-route around one another and never finish), while one or
    // two concurrent searches complete. Gameplay must not depend on how many
    // cores the machine has.
    return std::max<std::size_t>(1,std::min<std::size_t>(2,clamped_available/2));
}

std::size_t planner_job_budget_per_tick(std::size_t robot_count,std::size_t lanes){
    if(robot_count<=2) return robot_count;
    const std::size_t bounded_lanes=std::max<std::size_t>(1,lanes);
    // Refresh planning for a bounded working set each tick. Deferred robots
    // keep certified motion/holds and are serviced in the next ticks.
    const std::size_t budget=std::max<std::size_t>(4,bounded_lanes+2);
    return std::min(robot_count,budget);
}

float cross_xz(Vec3 a,Vec3 b){ return a.x*b.z-a.z*b.x; }

float head_on_right_bias_degrees(const SafetyAgent& self,const std::vector<SafetyAgent>& agents){
    const auto& rt=*self.runtime;
    if((!rt.goal_valid && !rt.temporary_goal_valid) || std::abs(self.speed_mps)<0.03f)
        return 0.0f;

    const Vec3 self_forward=forward_from_heading(self.heading_degrees);
    float best_bias=0.0f;
    for(const auto id:rt.nearby_robot_ids){
        const auto& other=agents[id];
        const auto& ort=*other.runtime;
        if((!ort.goal_valid && !ort.temporary_goal_valid) || std::abs(other.speed_mps)<0.03f) continue;

        Vec3 delta=other.position-self.position;
        delta.y=0.0f;
        const float d2=dot_xz(delta,delta);
        if(d2<0.0001f) continue;
        const float d=std::sqrt(d2);
        if(d>7.5f) continue;

        const Vec3 other_forward=forward_from_heading(other.heading_degrees);
        // Require roughly opposing headings and both robots in each other's front arc.
        if(dot_xz(self_forward,other_forward)>-0.45f) continue;
        if(dot_xz(self_forward,delta)<=0.0f) continue;
        if(dot_xz(other_forward,self.position-other.position)<=0.0f) continue;

        // Require a near centerline approach, otherwise keep normal corridor follow.
        const Vec3 dir=delta*(1.0f/d);
        const float lateral=std::abs(cross_xz(self_forward,dir));
        if(lateral>0.52f) continue;

        // Predict whether they are actually closing.
        const Vec3 self_vel=self_forward*std::abs(self.speed_mps);
        const Vec3 other_vel=other_forward*std::abs(other.speed_mps);
        const Vec3 rel_vel=other_vel-self_vel;
        const float closing=-dot_xz(rel_vel,dir);
        if(closing<0.10f) continue;

        // Stronger right bias when close and centered.
        const float proximity=1.0f-std::clamp((d-1.4f)/4.8f,0.0f,1.0f);
        const float centerline=1.0f-std::clamp(lateral/0.52f,0.0f,1.0f);
        const float bias=std::clamp(9.0f+17.0f*(0.65f*proximity+0.35f*centerline),9.0f,26.0f);
        best_bias=std::max(best_bias,bias);
    }
    return best_bias;
}

std::int64_t passage_key(Vec3 p){
    constexpr float cell=1.35f;
    const auto x=(std::int32_t)std::floor(p.x/cell);
    const auto z=(std::int32_t)std::floor(p.z/cell);
    return ((std::int64_t)x<<32) ^ (std::uint32_t)z;
}

Vec3 interpolate(const TubePiece& p,float t){
    if(p.t1<=p.t0+1.0e-7f) return p.end;
    const float alpha=std::clamp((t-p.t0)/(p.t1-p.t0),0.0f,1.0f);
    return p.start+(p.end-p.start)*alpha;
}

bool contract_valid_bounds(const SafetyAgent& a){
    return a.runtime && std::isfinite(a.position.x) && std::isfinite(a.position.z) &&
           std::isfinite(a.heading_degrees) && std::isfinite(a.speed_mps) &&
           a.radius_m>0.0f && a.max_speed_mps>0.0f && a.acceleration_mps2>0.0f &&
           a.braking_mps2>0.0f && a.turn_rate_degrees_per_second>0.0f;
}

MotionContract stationary_contract(const SafetyAgent& a,float now){
    MotionContract c{};
    c.valid=true;
    c.robot_id=a.id;
    c.valid_start=now;
    c.command_expiry=now+kSafetyDtSeconds;
    c.permitted_command={0.0f,a.heading_degrees,false};
    c.occupancy_tube.push_back({now,now+kSafetyDtSeconds,a.position,a.position,a.radius_m+kSafetyNumericMarginM});
    c.terminal_holds.push_back({now,a.position,a.radius_m+kSafetyNumericMarginM});
    c.passage_keys.push_back(passage_key(a.position));
    return c;
}

struct RolloutState {
    Vec3 position{};
    float heading{0.0f};
    float speed{0.0f};
};

float arc_chord_scale(float yaw_radians){
    const float half=yaw_radians*.5f;
    // All authorized steps are <= 10 ms; use the stable small-angle sinc.
    return 1.0f-half*half/6.0f+half*half*half*half/120.0f;
}

float motion_tube_radius(const SafetyAgent& agent,Vec3 end,float dt){
    const float radius=agent.radius_m+kSafetyNumericMarginM;
    if(dist_xz(agent.position,end)<1e-6f) return radius;
    // Bound the curve and speed-ramp error of the time-linear swept chord.
    const float acceleration=kRoverLateralAccelerationMps2+
        std::max(agent.acceleration_mps2,agent.braking_mps2);
    return radius+acceleration*dt*dt*.125f;
}

// A moving path corridor, rather than a heading held for the entire preview.
// Projection makes progress independent of whether the chassis hits a tiny
// waypoint disc exactly. Visibility checks preserve the obstacle-clear route.
struct SteeringSample { Vec3 target{}; float remaining{0.0f}; };
SteeringSample corridor_steering(const SafetyAgent& a,const RolloutState& s,
                                 const SafetyWorldCallbacks& world){
    const auto& rt=*a.runtime;
    const auto& path=rt.cached_route;
    if(path.empty()) return {rt.temporary_goal_valid?rt.temporary_goal:rt.goal,0.0f};
    std::size_t i=std::min(rt.route_cursor,path.size()-1);
    Vec3 start=i?path[i-1]:rt.route_origin;
    while(i+1<path.size()){
        const Vec3 edge=path[i]-start;
        const float len2=dot_xz(edge,edge);
        const bool passed=len2>0.001f && dot_xz(s.position-path[i],edge)>=0.0f;
        if(dist_xz(s.position,path[i])>0.16f && !passed) break;
        if(world.segment_clear && !world.segment_clear(a.id,s.position,path[i+1])) break;
        if(!rt.nearby_robot_ids.empty() && world.stationary_segment_clear && !world.stationary_segment_clear(a.id,s.position,path[i+1])) break;
        start=path[i++];
    }
    const Vec3 edge=path[i]-start;
    const float len2=dot_xz(edge,edge);
    const float t=len2>0.001f?std::clamp(dot_xz(s.position-start,edge)/len2,0.0f,1.0f):1.0f;
    Vec3 projection=start+edge*t;
    float ahead=std::clamp(0.55f+std::abs(s.speed)*0.22f,0.55f,2.0f);
    Vec3 target=path[i];
    float segment=dist_xz(projection,target);
    if(segment>ahead) target=projection+(target-projection)*(ahead/segment);
    else {
        ahead-=segment;
        for(std::size_t j=i+1;j<path.size() && ahead>0.0f;++j){
            const float length=dist_xz(path[j-1],path[j]);
            const Vec3 candidate=length>ahead?path[j-1]+(path[j]-path[j-1])*(ahead/length):path[j];
            if(world.segment_clear && !world.segment_clear(a.id,s.position,candidate)) break;
            if(!rt.nearby_robot_ids.empty() && world.stationary_segment_clear && !world.stationary_segment_clear(a.id,s.position,candidate)) break;
            target=candidate;
            ahead-=length;
        }
    }
    float remaining=dist_xz(s.position,path[i]);
    for(std::size_t j=i+1;j<path.size();++j) remaining+=dist_xz(path[j-1],path[j]);
    return {target,remaining};
}

RolloutState step_control(const SafetyAgent& agent,RolloutState state,const MotionControl& control,float dt,const SafetyWorldCallbacks& world){
    float heading=control.target_heading_degrees;
    float requested_speed=control.target_speed_mps;
    if(control.follow_route && agent.runtime && !agent.runtime->cached_route.empty()){
        const auto steer=corridor_steering(agent,state,world);
        heading=heading_between(state.position,steer.target,state.heading)+control.heading_offset_degrees;
        const float error=std::abs(wrap_degrees(heading-state.heading));
        const float room=std::max(0.0f,steer.remaining-agent.runtime->stop_radius*0.8f);
        const float stop_cap=std::sqrt(2.0f*agent.braking_mps2*room)*0.8f;
        const float alignment=error>72.0f?0.0f:std::max(0.08f,std::pow(std::cos(error*PI/180.0f),2.0f));
        requested_speed=std::min(requested_speed*alignment,stop_cap);
        if(steer.remaining<=agent.runtime->stop_radius) requested_speed=0.0f;
    }
    const float error=wrap_degrees(heading-state.heading);
    const float commanded_yaw=std::clamp(error/.14f,-agent.turn_rate_degrees_per_second,
                                        agent.turn_rate_degrees_per_second)*PI/180.0f;
    const float corner_speed=kRoverLateralAccelerationMps2/std::max(std::abs(commanded_yaw),.001f);
    const float speed_limit=std::min(agent.max_speed_mps,corner_speed);
    float target=std::clamp(requested_speed,-speed_limit,speed_limit);
    if(state.speed*target<0.0f) target=0.0f;
    const float rate=std::abs(target)<std::abs(state.speed)?agent.braking_mps2:agent.acceleration_mps2;
    const float next_speed=approach(state.speed,target,rate*dt);
    const float ramp_time=std::min(dt,std::abs(next_speed-state.speed)/rate);
    const float distance=.5f*(state.speed+next_speed)*ramp_time+next_speed*(dt-ramp_time);
    const float peak_speed=std::max(std::abs(state.speed),std::abs(next_speed));
    const float grip_yaw=kRoverLateralAccelerationMps2/std::max(.04f,peak_speed);
    const float yaw=std::clamp(commanded_yaw,-grip_yaw,grip_yaw)*dt;
    const float turn_degrees=yaw*180.0f/PI;
    // Integrate forward travel along the turn, with no sideways velocity.
    state.position+=forward_from_heading(state.heading+.5f*turn_degrees)*(distance*arc_chord_scale(yaw));
    state.heading=wrap_degrees(state.heading+turn_degrees);
    state.speed=next_speed;
    if(world.floor_height) state.position.y=world.floor_height(state.position);
    return state;
}

constexpr float kProposalPreviewDtSeconds=0.05f;

bool preview_static_clear(const SafetyAgent& agent,const MotionControl& control,const SafetyWorldCallbacks& world,std::vector<Vec3>& preview,float& final_heading,float& final_speed){
    preview.clear();
    RolloutState s{agent.position,agent.heading_degrees,agent.speed_mps};
    Vec3 previous=s.position;
    const int steps=(int)std::ceil(kPlannerHorizonSeconds/kProposalPreviewDtSeconds);
    for(int i=0;i<steps;++i){
        // Match the authoritative integrator instead of taking one coarse turn.
        for(int sub=0;sub<5;++sub) s=step_control(agent,s,control,kSafetyDtSeconds,world);
        if(world.point_clear && !world.point_clear(agent.id,s.position)) return false;
        if(world.segment_clear && !world.segment_clear(agent.id,previous,s.position)) return false;
        preview.push_back(s.position);
        previous=s.position;
    }
    final_heading=s.heading;
    final_speed=s.speed;
    return true;
}

bool point_clear_of_predicted_robot(const SafetyAgent& self,Vec3 point,float sample_time,
                                    const SafetyAgent& other,float& min_clearance){
    const float self_radius=self.radius_m+kSafetyNumericMarginM;
    const float other_hard_radius=other.radius_m+kSafetyNumericMarginM;
    const float current_clearance=dist_xz(self.position,other.position)-(self_radius+other_hard_radius);
    const bool escaping_existing_zone=current_clearance<kNeighborComfortClearanceM;

    auto clearance_allowed=[&](float clearance){
        min_clearance=std::min(min_clearance,clearance-kNeighborComfortClearanceM);
        // Comfort is a local-planning cost. Enforcing it as another hard hull
        // sealed physically usable gaps beside the starter parking apron.
        return clearance>=0.0f;
    };
    (void)escaping_existing_zone;

    bool represented=false;
    const auto& contract=other.runtime->active_contract;
    if(contract.valid){
        for(const auto& piece:contract.occupancy_tube){
            if(sample_time+1.0e-5f<piece.t0 || sample_time-1.0e-5f>piece.t1) continue;
            represented=true;
            const Vec3 center=interpolate(piece,std::clamp(sample_time,piece.t0,piece.t1));
            const float clearance=dist_xz(point,center)-(self_radius+piece.radius);
            if(!clearance_allowed(clearance)) return false;
        }
        for(const auto& hold:contract.terminal_holds){
            if(sample_time+1.0e-5f<hold.start_time) continue;
            represented=true;
            const float clearance=dist_xz(point,hold.position)-(self_radius+hold.radius);
            if(!clearance_allowed(clearance)) return false;
        }
    }
    if(!represented){
        const float clearance=dist_xz(point,other.position)-(self_radius+other_hard_radius);
        if(!clearance_allowed(clearance)) return false;
    }
    return true;
}

bool preview_fleet_clear(const SafetyAgent& self,const std::vector<Vec3>& preview,
                         const std::vector<SafetyAgent>& agents,float now,float& min_clearance){
    min_clearance=std::numeric_limits<float>::infinity();
    const auto& neighbors=self.runtime->nearby_robot_ids;
    if(neighbors.empty()) return true;
    for(std::size_t sample=0;sample<preview.size();++sample){
        const float t=now+static_cast<float>(sample+1)*kProposalPreviewDtSeconds;
        for(const auto id:neighbors)
            if(!point_clear_of_predicted_robot(self,preview[sample],t,agents[id],min_clearance)) return false;
    }
    return true;
}

Vec3 current_target(const SafetyRuntime& rt){ return rt.temporary_goal_valid?rt.temporary_goal:rt.goal; }

void invalidate_route(SafetyRuntime& rt){
    rt.cached_route.clear();
    rt.route_cursor=0;
    rt.proposals.clear();
    rt.route_replan_required=true;
    rt.route_failed=false;
    rt.route_retry_elapsed=0.0f;
    rt.route_failure_count=0;
    // invalidate_route() is used for a changed logical goal/temporary target.
    // A world-only revision intentionally does NOT call this function so the
    // persistent D* Lite g/rhs state survives and can be repaired in place.
    reset_incremental_path_state(rt.incremental_path);
}

void finish_traffic_maneuver(SafetyRuntime& rt){
    rt.temporary_goal_valid=false;
    if(rt.goal_valid) rt.yielding_for_traffic=false;
    rt.stationary_detour_active=false;
    rt.traffic_detour_partner=kNoSafetyRobot;
    rt.motion_intent=rt.goal_valid?MotionIntent::Navigate:MotionIntent::Wait;
    ++rt.goal_revision;
    invalidate_route(rt);
}

void update_nearby_robot_awareness(std::vector<SafetyAgent>& agents,std::vector<std::size_t>& counts){
    std::fill(counts.begin(),counts.end(),0);
    for(auto& a:agents){
        auto& rt=*a.runtime;
        rt.nearest_robot_id=kNoSafetyRobot;
        rt.nearest_robot_distance_m=std::numeric_limits<float>::infinity();
        rt.predicted_robot_clearance_m=std::numeric_limits<float>::infinity();
    }
    const auto add_neighbor=[&](std::size_t id,std::size_t other,float distance,float clearance){
        auto& rt=*agents[id].runtime;
        const auto slot=counts[id]++;
        auto& neighbors=rt.nearby_robot_ids;
        if(slot==neighbors.size()){
            neighbors.push_back(other);
            rt.avoidance_elapsed=kPlannerReplanSeconds;
            rt.proposals.clear();
        }else if(neighbors[slot]!=other){
            neighbors[slot]=other;
            rt.avoidance_elapsed=kPlannerReplanSeconds;
            rt.proposals.clear();
        }
        if(distance<rt.nearest_robot_distance_m){
            rt.nearest_robot_distance_m=distance;
            rt.nearest_robot_id=other;
        }
        rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,clearance);
    };
    constexpr float overlap_distance_sq=kRobotCollisionDetectionDistanceM*kRobotCollisionDetectionDistanceM;
    for(std::size_t i=0;i<agents.size();++i){
        for(std::size_t j=i+1;j<agents.size();++j){
            const Vec3 delta=agents[i].position-agents[j].position;
            const float distance_sq=dot_xz(delta,delta);
            if(!(distance_sq<overlap_distance_sq)) continue;
            const float distance=std::sqrt(distance_sq);
            const float clearance=distance-(agents[i].radius_m+agents[j].radius_m+
                                            2.0f*kSafetyNumericMarginM+kNeighborComfortClearanceM);
            add_neighbor(i,j,distance,clearance);
            add_neighbor(j,i,distance,clearance);
        }
    }
    for(auto& a:agents){
        auto& rt=*a.runtime;
        if(rt.nearby_robot_ids.size()!=counts[a.id]){
            rt.nearby_robot_ids.resize(counts[a.id]);
            rt.avoidance_elapsed=kPlannerReplanSeconds;
            rt.proposals.clear();
        }
        if(rt.temporary_goal_valid && rt.traffic_detour_partner!=kNoSafetyRobot &&
           !std::binary_search(rt.nearby_robot_ids.begin(),rt.nearby_robot_ids.end(),rt.traffic_detour_partner))
            finish_traffic_maneuver(rt);
        if(rt.nearby_robot_ids.empty()){
            rt.avoidance_velocity={};
            rt.avoidance_velocity_valid=false;
            rt.avoidance_elapsed=0.0f;
        }
    }
}

float route_corner_angle_degrees(const SafetyAgent& a,const SafetyRuntime& rt,std::size_t index){
    if(index>=rt.cached_route.size() || index+1>=rt.cached_route.size()) return 0.0f;
    const Vec3 before=index>0?rt.cached_route[index-1]:a.position;
    const Vec3 corner=rt.cached_route[index];
    const Vec3 after=rt.cached_route[index+1];
    Vec3 incoming=corner-before; incoming.y=0.0f;
    Vec3 outgoing=after-corner; outgoing.y=0.0f;
    const float in_len=std::sqrt(dot_xz(incoming,incoming));
    const float out_len=std::sqrt(dot_xz(outgoing,outgoing));
    if(in_len<0.05f || out_len<0.05f) return 0.0f;
    const float cosine=std::clamp(dot_xz(incoming,outgoing)/(in_len*out_len),-1.0f,1.0f);
    return std::acos(cosine)*180.0f/PI;
}

bool route_has_sharp_corner_at(const SafetyAgent& a,const SafetyRuntime& rt,std::size_t index){
    return route_corner_angle_degrees(a,rt,index)>=22.0f;
}

void advance_route_cursor(const SafetyAgent& a,SafetyRuntime& rt,const SafetyWorldCallbacks& world){
    while(rt.route_cursor<rt.cached_route.size()){
        // Do not switch to the post-corner waypoint while the chassis is still
        // approaching the corner. At 6 m/s the old 0.42 m acceptance radius
        // could start the turn early enough for the swept hull to touch the
        // structure even though both route segments were individually clear.
        const float acceptance=route_has_sharp_corner_at(a,rt,rt.route_cursor)?0.20f:0.42f;
        const Vec3 start=rt.route_cursor?rt.cached_route[rt.route_cursor-1]:rt.route_origin;
        const Vec3 edge=rt.cached_route[rt.route_cursor]-start;
        const float len2=dot_xz(edge,edge);
        bool passed=len2>0.001f && dot_xz(a.position-rt.cached_route[rt.route_cursor],edge)>=0.0f;
        if(passed && rt.route_cursor+1<rt.cached_route.size()){
            const auto next=rt.cached_route[rt.route_cursor+1];
            passed=(!world.segment_clear || world.segment_clear(a.id,a.position,next)) &&
                (rt.nearby_robot_ids.empty() || !world.stationary_segment_clear || world.stationary_segment_clear(a.id,a.position,next));
        }
        if(dist_xz(a.position,rt.cached_route[rt.route_cursor])>=acceptance && !passed) break;
        ++rt.route_cursor;
    }
}

Vec3 route_waypoint(const SafetyAgent& a,SafetyRuntime& rt,const SafetyWorldCallbacks& world){
    advance_route_cursor(a,rt,world);
    if(rt.route_cursor>=rt.cached_route.size()) return current_target(rt);

    // A stationary-rover bypass is a fleet-aware A* result. The generic
    // visibility shortcut below checks only static world geometry, so using it
    // here can jump straight across the very rover safety zone the route was
    // created to avoid. Follow the detour/retreat route one waypoint at a time.
    if(rt.stationary_detour_active || rt.motion_intent==MotionIntent::Retreat)
        return rt.cached_route[rt.route_cursor];

    std::size_t best=rt.route_cursor;
    const std::size_t far=std::min(rt.cached_route.size()-1,rt.route_cursor+5);
    for(std::size_t i=rt.route_cursor;i<=far;++i){
        // Visibility shortcuts are useful on straight terrain, but may not jump
        // over a real route corner. The individual straight chord can be hard-
        // collision-clear while still forcing the rover to begin turning too
        // early and clip the corner with its swept chassis.
        if(i>rt.route_cursor && route_has_sharp_corner_at(a,rt,i-1)) break;
        if(world.segment_clear && !world.segment_clear(a.id,a.position,rt.cached_route[i])) break;
        if(!rt.nearby_robot_ids.empty() && world.stationary_segment_clear && !world.stationary_segment_clear(a.id,a.position,rt.cached_route[i])) break;
        best=i;
    }
    return rt.cached_route[best];
}

float route_retry_delay(const SafetyAgent& a,const SafetyRuntime& rt){
    // Repeated failures in an unchanged scene are almost always the same
    // impossible search. Back off exponentially and add a deterministic rover
    // offset so a blocked fleet does not wake up and launch A* in lock-step.
    const unsigned failures=std::min<unsigned>(rt.route_failure_count,4u);
    const float exponential=0.45f*(float)(1u<<failures);
    const float stagger=0.09f*(float)(a.id%5u);
    return std::min(2.40f,exponential)+stagger;
}

bool remaining_route_is_static_clear(const SafetyAgent& a,SafetyRuntime& rt,const SafetyWorldCallbacks& world){
    if(rt.cached_route.empty() || !world.segment_clear) return false;
    advance_route_cursor(a,rt,world);
    if(rt.route_cursor>=rt.cached_route.size())
        return world.segment_clear(a.id,a.position,current_target(rt));

    // A world revision no longer invalidates every rover route blindly. Check
    // the small simplified polyline that remains for this rover. If the new
    // structure/corridor is unrelated, the route remains valid and no A* work
    // is required. Dynamic rover-vs-rover safety is intentionally excluded from
    // this test and remains owned by proposal prediction/reservation contracts.
    Vec3 previous=a.position;
    for(std::size_t i=rt.route_cursor;i<rt.cached_route.size();++i){
        if(!world.segment_clear(a.id,previous,rt.cached_route[i])) return false;
        previous=rt.cached_route[i];
    }
    const Vec3 target=current_target(rt);
    if(dist_xz(previous,target)>0.04f && !world.segment_clear(a.id,previous,target)) return false;
    return true;
}

void refresh_route_world_revision(SafetyAgent& a,std::uint64_t world_epoch,const SafetyWorldCallbacks& world){
    auto& rt=*a.runtime;
    if(!rt.goal_valid && !rt.temporary_goal_valid) return;
    if(rt.route_goal_revision!=rt.goal_revision) return; // goal change already requires a full route
    if(rt.route_world_epoch==world_epoch) return;

    if(!rt.route_replan_required && !rt.cached_route.empty() && remaining_route_is_static_clear(a,rt,world)){
        rt.route_world_epoch=world_epoch;
        return;
    }

    // Only the rover whose actual remaining route intersects the changed world
    // loses its cached route. This prevents one construction stage elsewhere in
    // the colony from synchronously replanning the entire fleet.
    rt.cached_route.clear();
    rt.route_cursor=0;
    rt.proposals.clear();
    rt.route_replan_required=true;
    rt.route_failed=false;
    rt.route_retry_elapsed=0.0f;
    rt.route_failure_count=0;
}

bool route_search_due(const SafetyAgent& a,std::uint64_t world_epoch){
    const auto& rt=*a.runtime;
    if(!rt.goal_valid && !rt.temporary_goal_valid) return false;
    const bool revision_changed=rt.route_goal_revision!=rt.goal_revision || rt.route_world_epoch!=world_epoch;
    const bool stale=rt.route_replan_required || rt.cached_route.empty() || revision_changed;
    if(!stale) return false;
    if(rt.route_failed && !rt.route_replan_required && !revision_changed &&
       rt.route_retry_elapsed<route_retry_delay(a,rt)) return false;
    return true;
}

void ensure_route(SafetyAgent& a,std::uint64_t world_epoch,const SafetyWorldCallbacks& world,bool allow_route_search){
    auto& rt=*a.runtime;
    if(!rt.goal_valid && !rt.temporary_goal_valid) return;
    const Vec3 target=current_target(rt);
    const bool world_or_goal_changed=rt.route_goal_revision!=rt.goal_revision || rt.route_world_epoch!=world_epoch;
    if(rt.route_failed && !rt.route_replan_required && !world_or_goal_changed &&
       rt.route_retry_elapsed<route_retry_delay(a,rt)) return;
    const bool stale=rt.route_replan_required || rt.cached_route.empty() || world_or_goal_changed;
    if(!stale) return;
    if(!allow_route_search) return;

    std::vector<Vec3> route;
    const float radius=std::max(0.05f,rt.stop_radius);
    bool ok=false;
    bool incremental_pending=false;
    // Normal work/logistics travel uses the persistent static-world D* Lite
    // planner. Temporary retreat/stationary-detour targets intentionally use
    // the fleet-aware coarse planner because another rover is the obstacle that
    // created those targets in the first place.
    if(!rt.temporary_goal_valid && world.incremental_route){
        const auto result=world.incremental_route(a.id,a.position,target,radius,world_epoch,
                                                  rt.incremental_path,route);
        ok=result==IncrementalPathResult::Success;
        incremental_pending=result==IncrementalPathResult::Pending;
    }
    if(incremental_pending){
        // The search consumed its bounded expansion slice. Keep the rover in a
        // certified hold and continue the SAME D* state on a later frame rather
        // than paying for an immediate full A* fallback and recreating a spike.
        rt.route_replan_required=true;
        rt.route_failed=false;
        rt.proposals.clear();
        return;
    }
    if(!ok && world.coarse_route)
        ok=world.coarse_route(a.id,a.position,target,radius,route);
    if(!ok && world.segment_clear && world.segment_clear(a.id,a.position,target)){
        route={target};
        ok=true;
    }
    if(ok && !rt.nearby_robot_ids.empty() && world.stationary_segment_clear && world.coarse_route){
        Vec3 previous=a.position;
        bool fleet_clear=true;
        for(const Vec3 p:route){
            if(!world.stationary_segment_clear(a.id,previous,p)){fleet_clear=false;break;}
            previous=p;
        }
        if(!fleet_clear){
            std::vector<Vec3> detour;
            if(world.coarse_route(a.id,a.position,target,radius,detour)) route=std::move(detour);
        }
    }
    rt.cached_route=ok?std::move(route):std::vector<Vec3>{};
    rt.route_origin=a.position;
    rt.best_route_remaining=std::numeric_limits<float>::infinity();
    rt.route_progress_elapsed=0.0f;
    rt.route_cursor=0;
    rt.route_world_epoch=world_epoch;
    rt.route_goal_revision=rt.goal_revision;
    rt.route_replan_required=false;
    rt.route_failed=!ok;
    rt.route_retry_elapsed=0.0f;
    if(ok) rt.route_failure_count=0;
    else rt.route_failure_count=(std::uint8_t)std::min<unsigned>(255u,(unsigned)rt.route_failure_count+1u);
    ++rt.local_replans;
    if(!ok){
        rt.block_reason=BlockReason::NoRoute;
        rt.proposals.clear();
    }
}

void enter_fault(SafetyAgent& a,BlockReason reason,std::uint8_t phase,float now){
    auto& rt=*a.runtime;
    // A persistent overlap or bounds failure is detected again every tick. It
    // must never postpone the first attempt or the following three-second retry.
    if(rt.mode!=SafetyMode::Fault || rt.fault_recovery_deadline<0.0f)
        rt.fault_recovery_deadline=now+kFaultRecoveryMaxWaitSeconds;
    rt.mode=SafetyMode::Fault;rt.block_reason=reason;rt.fault_phase=phase;
    a.speed_mps=0.0f;
}

void recover_faulted_agents(float dt,float now,std::uint64_t world_epoch,
                            std::vector<SafetyAgent>& agents,
                            const std::vector<NavigationRequest>& requests,
                            const SafetyWorldCallbacks& world){
    std::vector<std::size_t> due;
    for(auto& a:agents){
        auto& rt=*a.runtime;
        if(rt.mode!=SafetyMode::Fault){rt.fault_recovery_deadline=-1.0f;continue;}
        a.speed_mps=0.0f;
        if(rt.fault_recovery_deadline<0.0f)rt.fault_recovery_deadline=now+kFaultRecoveryMaxWaitSeconds;
        // Start in the frame containing the deadline, never one frame later.
        if(now+dt+1.0e-5f<rt.fault_recovery_deadline)continue;
        rt.fault_recovery_deadline=now+dt+kFaultRecoveryMaxWaitSeconds;
        ++rt.fault_recovery_attempts;
        const auto& req=requests[a.id];
        rt.goal_valid=req.active;rt.goal=req.goal;
        rt.stop_radius=std::max(0.02f,req.stop_radius);
        rt.speed_scale=std::clamp(req.speed_scale,0.10f,1.0f);
        ++rt.goal_revision;
        rt.temporary_goal_valid=false;rt.stationary_detour_active=false;
        rt.traffic_detour_partner=kNoSafetyRobot;rt.yielding_for_traffic=false;
        rt.blocker_robot_id=kNoSafetyRobot;
        invalidate_route(rt); // discards the entire D* search, coarse path and proposals
        rt.route_world_epoch=0;rt.route_goal_revision=0;rt.route_origin=a.position;
        rt.avoidance_velocity={};rt.avoidance_velocity_valid=false;rt.avoidance_elapsed=0;
        rt.steering_side=0;rt.recovery_cooldown=0;rt.blocked_elapsed=0;rt.wait_age=0;
        rt.best_route_remaining=std::numeric_limits<float>::infinity();rt.route_progress_elapsed=0;
        rt.last_progress_position=a.position;rt.last_progress_time=0;
        rt.motion_intent=req.active?MotionIntent::Navigate:MotionIntent::Wait;
        if(contract_valid_bounds(a))due.push_back(a.id);
    }
    // Due fault recovery is not queued behind ordinary work/traffic replans.
    // Each search is still expansion-bounded and all read the same frozen world.
    eng::TaskPool::shared().parallel_for(due.size(),[&](std::size_t job){
        ensure_route(agents[due[job]],world_epoch,world,true);
    });
    for(const auto i:due){
        auto& a=agents[i];auto& rt=*a.runtime;
        bool safe=world.point_clear(i,a.position) && world.segment_clear(i,a.position,a.position);
        for(const auto& other:agents)if(other.id!=i &&
            dist_xz(a.position,other.position)<=a.radius_m+other.radius_m+2*kSafetyNumericMarginM)safe=false;
        if(!safe)continue; // a computed route never authorizes escape through an overlap
        rt.initialized=true;rt.active_contract=stationary_contract(a,now);
        rt.fault_phase=0;rt.fault_recovery_deadline=-1;
        rt.planner_elapsed=kPlannerReplanSeconds;
        rt.mode=rt.goal_valid?(rt.route_failed?SafetyMode::Blocked:SafetyMode::Waiting):
            (requests[i].work_target?SafetyMode::Working:SafetyMode::Idle);
        rt.block_reason=rt.route_failed?BlockReason::NoRoute:BlockReason::None;
    }
}

void update_reciprocal_velocities(std::vector<SafetyAgent>& agents,const SafetyWorldCallbacks& world,float dt){
    bool due=false;
    for(auto& a:agents){
        if(a.runtime->nearby_robot_ids.empty()) continue;
        a.runtime->avoidance_elapsed+=dt;
        due=due || a.runtime->avoidance_elapsed>=kPlannerReplanSeconds;
    }
    if(!due) return;
    RVO::RVOSimulator simulator;
    simulator.setTimeStep(kPlannerReplanSeconds);
    for(const auto& a:agents){
        const auto& rt=*a.runtime;
        if(rt.nearby_robot_ids.empty()) continue;
        const bool moving=rt.goal_valid || rt.temporary_goal_valid;
        const Vec3 actual=forward_from_heading(a.heading_degrees)*a.speed_mps;
        const auto index=simulator.addAgent(RVO::Vector2(a.position.x,a.position.z),
            kRobotCollisionDetectionDistanceM,rt.nearby_robot_ids.size(),
            2.0f,2.0f,a.radius_m+kSafetyNumericMarginM+kRobotPersonalSafetyZoneM,
            moving?a.max_speed_mps:0.0f,RVO::Vector2(actual.x,actual.z));
        Vec3 preferred{};
        if(moving){
            const Vec3 target=rt.cached_route.empty()?current_target(rt):
                corridor_steering(a,{a.position,a.heading_degrees,a.speed_mps},world).target;
            const float heading=heading_between(a.position,target,a.heading_degrees);
            // Head-on encounter rule: each rover evades to ITS own right.
            // Outside that case, keep a small stable right-bias for symmetry.
            const float right_bias=head_on_right_bias_degrees(a,agents);
            const float bias=right_bias>0.0f?right_bias:5.0f;
            const float room=std::max(0.0f,dist_xz(a.position,current_target(rt))-rt.stop_radius);
            const float speed=std::min(a.max_speed_mps*rt.speed_scale,std::sqrt(2.0f*a.braking_mps2*room)*0.8f);
            preferred=forward_from_heading(heading+bias)*speed;
        }
        simulator.setAgentPrefVelocity(index,RVO::Vector2(preferred.x,preferred.z));
    }
    simulator.doStep();
    std::size_t simulator_index=0;
    for(auto& a:agents){
        auto& rt=*a.runtime;
        if(rt.nearby_robot_ids.empty()) continue;
        const auto velocity=simulator.getAgentVelocity(simulator_index++);
        rt.avoidance_velocity={velocity.x(),0.0f,velocity.y()};
        rt.avoidance_velocity_valid=std::any_of(rt.nearby_robot_ids.begin(),rt.nearby_robot_ids.end(),
            [&](std::size_t id){
                const auto& neighbor=*agents[id].runtime;
                return neighbor.goal_valid || neighbor.temporary_goal_valid;
            });
        rt.avoidance_elapsed=0.0f;
    }
}

void generate_proposals(SafetyAgent& a,std::uint64_t world_epoch,const SafetyWorldCallbacks& world,const std::vector<SafetyAgent>& agents,float now,bool allow_route_search){
    auto& rt=*a.runtime;
    rt.proposals.clear();
    if(rt.motion_intent==MotionIntent::Wait || (!rt.goal_valid && !rt.temporary_goal_valid)){
        rt.proposals.push_back({{0.0f,a.heading_degrees,false},{},0.0f});
        return;
    }

    ensure_route(a,world_epoch,world,allow_route_search);
    if((rt.route_replan_required && !allow_route_search) || rt.cached_route.empty()){
        // Replan work is deliberately budgeted across rendered frames. A rover
        // awaiting its turn remains stopped under its existing terminal hold;
        // safety is never traded for pathfinding throughput.
        rt.proposals.push_back({{0.0f,a.heading_degrees,false},{},10000.0f});
        return;
    }

    const Vec3 waypoint=route_waypoint(a,rt,world);
    const float remaining_before=corridor_steering(a,{a.position,a.heading_degrees,a.speed_mps},world).remaining;
    auto path_progress=[&](Vec3 end,float heading,float speed){
        return remaining_before-corridor_steering(a,{end,heading,speed},world).remaining;
    };
    const float desired=heading_between(a.position,waypoint,a.heading_degrees);
    const float cruise=std::max(0.08f,a.max_speed_mps*std::clamp(rt.speed_scale,0.15f,1.0f));
    const float target_distance=dist_xz(a.position,current_target(rt));
    const float braking_room=std::max(0.0f,target_distance-std::max(0.03f,rt.stop_radius));
    const float stopping_speed_cap=std::sqrt(std::max(0.0f,2.0f*a.braking_mps2*braking_room))*0.90f;
    float planned_cruise=std::min(cruise,stopping_speed_cap);

    // Brake before a mandatory structure-corner waypoint instead of arriving at
    // 6 m/s and discovering that every turning preview intersects the wall.
    // The exact move remains contract-certified at 100 Hz; this only shapes the
    // advisory target speed so the rover reaches a turn in a controllable state.
    if(rt.route_cursor<rt.cached_route.size() &&
       route_has_sharp_corner_at(a,rt,rt.route_cursor)){
        const float corner_distance=dist_xz(a.position,rt.cached_route[rt.route_cursor]);
        const float turn_angle=route_corner_angle_degrees(a,rt,rt.route_cursor);
        const float room=std::max(0.0f,corner_distance-0.18f);
        const float brake_cap=std::sqrt(std::max(0.0f,2.0f*a.braking_mps2*room))*0.72f;
        const float turn_cap=std::clamp(2.20f-(turn_angle-22.0f)*0.012f,0.55f,2.20f);
        planned_cruise=std::min(planned_cruise,std::max(0.20f,std::min(brake_cap,turn_cap)));
    }

    // A rover may pivot in place. This is essential when conservative safety
    // discs are close: translating while turning can consume the remaining
    // clearance even though rotating the disc in place is completely safe.
    // The coordinator certifies these orientation-only commands exactly like
    // every other motion.
    const float heading_error=wrap_degrees(desired-a.heading_degrees);
    if(std::abs(heading_error)>2.0f){
        MotionControl pivot{};
        pivot.target_speed_mps=0.0f;
        pivot.target_heading_degrees=desired;
        std::vector<Vec3> preview;
        float end_heading=0.0f,end_speed=0.0f;
        if(preview_static_clear(a,pivot,world,preview,end_heading,end_speed)){
            float fleet_clearance=std::numeric_limits<float>::infinity();
            if(preview_fleet_clear(a,preview,agents,now,fleet_clearance))
                rt.proposals.push_back({pivot,std::move(preview),5.0f+std::abs(heading_error)*0.002f});
            rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
        }
    }
    constexpr std::array<float,1> angle_offsets{{0.0f}};
    constexpr std::array<float,3> speed_scales{{1.0f,0.58f,0.34f}};
    // Normal travel should be cheap: follow the coarse route with a small set
    // of local corrections. Only a rover that is actually blocked/retreating
    // expands to the full steering fan. The final contract arbitration remains
    // unchanged, so reducing proposal count lowers CPU cost without weakening
    // the collision authority.
    const bool recovery_search=rt.stationary_detour_active || rt.motion_intent==MotionIntent::Retreat ||
        rt.blocked_elapsed>=0.30f || rt.mode==SafetyMode::Waiting || rt.mode==SafetyMode::Blocked;
    // The route follower owns static turns. Fixed angular fans around its
    // tangent create circular orbits beside a corner; moving traffic gets its
    // explicit reciprocal-velocity candidate instead.
    const std::size_t angle_count=1u;
    const std::size_t speed_count=recovery_search?speed_scales.size():2u;

    if(rt.avoidance_velocity_valid){
        const float speed=std::sqrt(dot_xz(rt.avoidance_velocity,rt.avoidance_velocity));
        if(speed>0.06f){
            MotionControl control{};
            control.target_speed_mps=std::min(planned_cruise,speed);
            control.target_heading_degrees=heading_between({},rt.avoidance_velocity,desired);
            control.heading_offset_degrees=std::clamp(wrap_degrees(control.target_heading_degrees-desired),-75.0f,75.0f);
            control.follow_route=true;
            std::vector<Vec3> preview;
            float end_heading=0.0f,end_speed=0.0f,clearance=0.0f;
            if(preview_static_clear(a,control,world,preview,end_heading,end_speed) &&
               preview_fleet_clear(a,preview,agents,now,clearance)){
                const float progress=path_progress(preview.back(),end_heading,end_speed);
                // Score actual path progress. A fixed ORCA bonus made a safe
                // sideways orbit cheaper than reaching a nearby work station.
                if(progress>0.015f){
                    const float deviation=std::abs(control.heading_offset_degrees)*0.008f;
                    rt.proposals.push_back({control,std::move(preview),-progress+deviation});
                }
            }
        }
    }

    for(std::size_t si=0;si<speed_count;++si){
        const float speed_scale=speed_scales[si];
        for(std::size_t ai=0;ai<angle_count;++ai){
            const float offset=angle_offsets[ai];
            MotionControl control{};
            control.target_speed_mps=planned_cruise*speed_scale;
            control.target_heading_degrees=desired+offset;
            control.follow_route=true;
            control.heading_offset_degrees=offset;
            std::vector<Vec3> preview;
            float end_heading=0.0f,end_speed=0.0f;
            if(!preview_static_clear(a,control,world,preview,end_heading,end_speed)) continue;
            float fleet_clearance=std::numeric_limits<float>::infinity();
            if(!preview_fleet_clear(a,preview,agents,now,fleet_clearance)){
                rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
                continue;
            }
            rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
            const Vec3 end=preview.empty()?a.position:preview.back();
            const float progress=path_progress(end,end_heading,end_speed);
            const float heading_cost=std::abs(wrap_degrees(control.target_heading_degrees-desired))*0.018f;
            const float smoothness=std::abs(wrap_degrees(control.target_heading_degrees-a.heading_degrees))*0.008f;
            const float slow_cost=(1.0f-speed_scale)*0.65f;
            const float switch_cost=rt.steering_side*offset<0.0f?0.45f:0.0f;
            const float comfort_cost=std::isfinite(fleet_clearance)?std::max(0.0f,-fleet_clearance)*0.4f:0.0f;
            const float cost=-progress+heading_cost+smoothness+slow_cost+switch_cost+comfort_cost;
            rt.proposals.push_back({control,std::move(preview),cost});
        }
    }

    // Waiting and braking are valid proposals, but are deliberately more
    // expensive than any safe progress-producing command.
    rt.proposals.push_back({{0.0f,a.heading_degrees,false},{},2.5f});

    // A disc-bounded rover can pivot safely in place. Recovery uses a latched
    // route/pull-out destination; speculative reverse fans caused repeated
    // forward/backward pulses at blocked service stations.

    bool has_translation=false;
    for(const auto& proposal:rt.proposals)
        has_translation=has_translation || proposal.control.target_speed_mps>0.001f;
    if(!has_translation){
        const Vec3 delta=waypoint-a.position;
        const float len2=dot_xz(delta,delta);
        float nearest=std::numeric_limits<float>::infinity();
        for(const auto id:rt.nearby_robot_ids){
            const auto& other=agents[id];
            const float t=len2>0.001f?std::clamp(dot_xz(other.position-a.position,delta)/len2,0.0f,1.0f):0.0f;
            const float required=a.radius_m+other.radius_m+0.25f;
            if(dist_xz(a.position+delta*t,other.position)>required) continue;
            const float d=dist_xz(a.position,other.position);
            if(d<nearest){nearest=d;rt.blocker_robot_id=other.id;rt.block_reason=BlockReason::RobotReservation;}
        }
    }

    std::stable_sort(rt.proposals.begin(),rt.proposals.end(),[](const MotionProposal& x,const MotionProposal& y){
        return x.cost<y.cost;
    });
}

MotionContract build_contract(const SafetyAgent& a,const MotionControl& control,float now,float lease,const SafetyWorldCallbacks& world){
    MotionContract c{};
    if(!contract_valid_bounds(a)) return c;
    c.robot_id=a.id;
    c.valid_start=now;
    lease=std::clamp(lease,1.0e-5f,kSafetyDtSeconds);
    c.command_expiry=now+lease;
    c.permitted_command=control;
    const float radius=a.radius_m+kSafetyNumericMarginM;
    // Bound the continuum of possible brake-trigger instants inside the 10 ms
    // command lease. The two explicit endpoint branches below are inflated by
    // the maximum distance the robot can travel while the trigger time varies.
    const float trigger_time_margin=
        std::abs(a.speed_mps)*lease +
        0.5f*a.acceleration_mps2*lease*lease;
    const float brake_radius=radius+trigger_time_margin;

    RolloutState start{a.position,a.heading_degrees,a.speed_mps};
    RolloutState normal=step_control(a,start,control,lease,world);
    c.occupancy_tube.push_back({now,now+lease,start.position,normal.position,motion_tube_radius(a,normal.position,lease)});

    // Conservative timeout/emergency branch: allow the command to run for the
    // complete lease, then brake at the guaranteed minimum deceleration without
    // changing heading. The branch beginning immediately is also included. The
    // union conservatively covers every trigger instant within this 10 ms lease.
    auto append_brake=[&](RolloutState s,float branch_start){
        const float braking=std::max(0.05f,a.braking_mps2);
        float t=branch_start;
        float speed=s.speed;
        Vec3 p=s.position;
        const float sign=speed<0.0f?-1.0f:1.0f;
        const Vec3 dir=forward_from_heading(s.heading)*sign;
        // A braking path is spatially straight; only speed varies quadratically
        // with time. Five-centisecond pieces plus the analytic chord-error
        // inflation below conservatively enclose that time-position curve while
        // reducing tube-pair comparisons by about 5x versus 10 ms pieces.
        constexpr float brake_dt=0.05f;
        const float temporal_chord_error=braking*brake_dt*brake_dt*0.125f;
        const float piece_radius=brake_radius+temporal_chord_error;
        int guard=0;
        while(std::abs(speed)>0.001f && guard++<160){
            const float step_dt=std::min(brake_dt,std::abs(speed)/braking);
            const float next_speed=approach(speed,0.0f,braking*step_dt);
            const float avg=0.5f*(std::abs(speed)+std::abs(next_speed));
            Vec3 q=p+dir*(avg*step_dt);
            if(world.floor_height) q.y=world.floor_height(q);
            TubePiece piece{t,t+step_dt,p,q,piece_radius};
            c.brake_branches.push_back(piece);
            c.occupancy_tube.push_back(piece);
            p=q;
            speed=next_speed;
            t+=step_dt;
        }
        c.terminal_holds.push_back({t,p,brake_radius});
    };
    append_brake(start,now);
    c.committed_brake_first=c.brake_branches.size();
    append_brake(normal,now+lease);

    // Reserve all spatial cells touched by normal motion, braking branches and
    // terminal holds. This is a conservative passage-token approximation whose
    // exit pocket is the terminal hold itself.
    std::unordered_set<std::int64_t> keys;
    auto add_piece_keys=[&](const TubePiece& p){
        const float length=dist_xz(p.start,p.end);
        const int samples=std::max(1,(int)std::ceil(length/0.45f));
        for(int i=0;i<=samples;++i){
            const Vec3 q=p.start+(p.end-p.start)*((float)i/(float)samples);
            keys.insert(passage_key(q));
        }
    };
    for(const auto& p:c.occupancy_tube) add_piece_keys(p);
    for(const auto& h:c.terminal_holds) keys.insert(passage_key(h.position));
    c.passage_keys.assign(keys.begin(),keys.end());
    c.valid=true;
    return c;
}

bool contract_static_clear(const MotionContract& c,const SafetyWorldCallbacks& world){
    if(!c.valid) return false;
    
    for(const auto& p:c.occupancy_tube){
        if(world.point_clear && (!world.point_clear(c.robot_id,p.start) || !world.point_clear(c.robot_id,p.end))) return false;
        if(world.segment_clear && !world.segment_clear(c.robot_id,p.start,p.end)) return false;
    }
    for(const auto& hold:c.terminal_holds)
        if(world.point_clear && !world.point_clear(c.robot_id,hold.position)) return false;
    return true;
}

struct ReservationBoard {
    std::vector<MotionContract> contracts{};
    std::unordered_map<std::int64_t,std::size_t> passage_owner{};
};

// Pairwise personal-zone certification is implemented below the public tube
// helpers; forward-declare it here for contract arbitration.
bool personal_zone_contract_clear(const MotionContract& candidate,const MotionContract& other);

void rebuild_passages(ReservationBoard& board){
    board.passage_owner.clear();
    for(const auto& c:board.contracts){
        if(!c.valid) continue;
        for(auto key:c.passage_keys) board.passage_owner.emplace(key,c.robot_id);
    }
}

void replace_contract(ReservationBoard& board,const MotionContract& c){
    if(c.robot_id>=board.contracts.size()) return;
    board.contracts[c.robot_id]=c;
    rebuild_passages(board);
}

[[maybe_unused]] bool passages_available(const MotionContract& c,const ReservationBoard& board,std::size_t& blocker){
    for(auto key:c.passage_keys){
        const auto it=board.passage_owner.find(key);
        if(it!=board.passage_owner.end() && it->second!=c.robot_id){
            blocker=it->second;
            return false;
        }
    }
    return true;
}

// Planar bounds of every swept disc and terminal hold of a contract. Every
// robot-vs-robot rejection below requires two discs to touch, so contracts
// whose inflated bounds are disjoint cannot conflict and skip the O(P^2) tests.
struct ContractBounds {
    float min_x{std::numeric_limits<float>::infinity()},max_x{-std::numeric_limits<float>::infinity()};
    float min_z{std::numeric_limits<float>::infinity()},max_z{-std::numeric_limits<float>::infinity()};
};

ContractBounds contract_bounds(const MotionContract& c){
    ContractBounds b{};
    auto add=[&](Vec3 p,float r){
        b.min_x=std::min(b.min_x,p.x-r);b.max_x=std::max(b.max_x,p.x+r);
        b.min_z=std::min(b.min_z,p.z-r);b.max_z=std::max(b.max_z,p.z+r);
    };
    for(const auto& piece:c.occupancy_tube){ add(piece.start,piece.radius); add(piece.end,piece.radius); }
    for(const auto& hold:c.terminal_holds) add(hold.position,hold.radius);
    return b;
}

bool contract_bounds_disjoint(const ContractBounds& a,const ContractBounds& b){
    constexpr float slack=1.0e-3f;
    return a.max_x+slack<b.min_x || b.max_x+slack<a.min_x ||
           a.max_z+slack<b.min_z || b.max_z+slack<a.min_z;
}

bool contract_clear(const MotionContract& c,const SafetyWorldCallbacks& world,const ReservationBoard& board,std::size_t& blocker,BlockReason& reason){
    blocker=kNoSafetyRobot;
    reason=BlockReason::None;
    if(!contract_static_clear(c,world)){
        reason=BlockReason::StaticGeometry;
        return false;
    }
    // Cell tokens are scheduling hints only. Two disjoint swept hulls may share
    // a coarse cell; the continuous motion/braking/hold tests below decide.
    const ContractBounds own_bounds=contract_bounds(c);
    for(const auto& other:board.contracts){
        if(!other.valid || other.robot_id==c.robot_id) continue;
        if(contract_bounds_disjoint(own_bounds,contract_bounds(other))) continue;
        if(tubes_overlap(c.occupancy_tube,other.occupancy_tube)){
            blocker=other.robot_id;
            reason=BlockReason::RobotReservation;
            return false;
        }
        if(!personal_zone_contract_clear(c,other)){
            blocker=other.robot_id;
            reason=BlockReason::RobotReservation;
            return false;
        }
        // Terminal holds are indefinite reservations. A moving piece must not
        // enter a hold after that hold starts, and a new hold must not be placed
        // into any still-authorized motion/braking tube of another robot.
        auto piece_hits_hold=[](const TubePiece& piece,const TerminalHold& hold){
            if(piece.t1+1.0e-7f<hold.start_time) return false;
            const float t0=std::max(piece.t0,hold.start_time);
            if(piece.t1+1.0e-7f<t0) return false;
            TubePiece stationary{t0,piece.t1,hold.position,hold.position,hold.radius};
            return moving_discs_overlap(piece,stationary,t0,piece.t1);
        };
        for(const auto& piece:c.occupancy_tube){
            for(const auto& hold:other.terminal_holds){
                if(piece_hits_hold(piece,hold)){
                    blocker=other.robot_id; reason=BlockReason::RobotReservation; return false;
                }
            }
        }
        for(const auto& hold:c.terminal_holds){
            for(const auto& piece:other.occupancy_tube){
                if(piece.t1+1.0e-7f<hold.start_time) continue;
                const float t0=std::max(piece.t0,hold.start_time);
                TubePiece stationary{t0,piece.t1,hold.position,hold.position,hold.radius};
                if(piece.t1+1.0e-7f>=t0 && moving_discs_overlap(piece,stationary,t0,piece.t1)){
                    blocker=other.robot_id; reason=BlockReason::RobotReservation; return false;
                }
            }
        }
        for(const auto& ah:c.terminal_holds){
            for(const auto& bh:other.terminal_holds){
                if(dist_xz(ah.position,bh.position)<=ah.radius+bh.radius){
                    blocker=other.robot_id; reason=BlockReason::RobotReservation; return false;
                }
            }
        }
    }
    return true;
}

bool reached_target(const SafetyAgent& a,const SafetyRuntime& rt){
    if(!rt.goal_valid && !rt.temporary_goal_valid) return true;
    return dist_xz(a.position,current_target(rt))<=std::max(0.03f,rt.stop_radius);
}

bool state_inside_active_contract(const SafetyAgent& a,float now){
    const auto& c=a.runtime->active_contract;
    if(!c.valid) return false;
    for(const auto& piece:c.occupancy_tube){
        if(now+1.0e-5f<piece.t0 || now-1.0e-5f>piece.t1) continue;
        const Vec3 center=interpolate(piece,std::clamp(now,piece.t0,piece.t1));
        if(dist_xz(a.position,center)<=piece.radius) return true;
    }
    for(const auto& hold:c.terminal_holds){
        if(now+1.0e-5f>=hold.start_time && dist_xz(a.position,hold.position)<=hold.radius) return true;
    }
    return false;
}

bool follow_certified_brake_branch(const SafetyAgent& a,float now,float tick,const SafetyWorldCallbacks& world,
                                   Vec3& out_position,float& out_speed){
    const auto& contract=a.runtime->active_contract;
    if(!contract.valid) return false;

    // Roll straight along the measured heading. A reservation is an envelope,
    // never a position to interpolate or snap to.
    const float braking_time=std::min(tick,std::abs(a.speed_mps)/a.braking_mps2);
    const float next_speed=approach(a.speed_mps,0.0f,a.braking_mps2*braking_time);
    const float distance=.5f*(a.speed_mps+next_speed)*braking_time;
    Vec3 end=a.position+forward_from_heading(a.heading_degrees)*distance;
    if(world.floor_height) end.y=world.floor_height(end);

    const float chassis=a.radius_m+kSafetyNumericMarginM;
    const auto covered=[&](Vec3 position,float time){
        // Use only the branch that starts at the pose committed by the last
        // command. The immediate-brake branch is for collision arbitration.
        for(std::size_t i=contract.committed_brake_first;i<contract.brake_branches.size();++i){
            const auto& piece=contract.brake_branches[i];
            if(time<piece.t0-1e-5f || time>piece.t1+1e-5f) continue;
            const auto expected=interpolate(piece,std::clamp(time,piece.t0,piece.t1));
            if(dist_xz(position,expected)<=std::max(0.0f,piece.radius-chassis)+.002f) return true;
        }
        if(contract.terminal_holds.empty()) return false;
        const auto& hold=contract.terminal_holds.back();
        return time+1e-5f>=hold.start_time &&
            dist_xz(position,hold.position)<=std::max(0.0f,hold.radius-chassis)+.002f;
    };
    if(!covered(a.position,now)||!covered(end,now+tick)) return false;
    out_position=end;
    out_speed=next_speed;
    return true;
}

Vec3 find_pullout(const SafetyAgent& a,const SafetyWorldCallbacks& world,const std::vector<SafetyAgent>& agents,std::size_t blocker){
    Vec3 away{};
    if(blocker<agents.size()){
        away=a.position-agents[blocker].position;
        away.y=0.0f;
    }
    float len=std::sqrt(away.x*away.x+away.z*away.z);
    if(len<0.05f){ const Vec3 f=forward_from_heading(a.heading_degrees); away={-f.x,0.0f,-f.z}; }
    else away=away*(1.0f/len);
    const float base=std::atan2(away.z,away.x);
    constexpr std::array<float,11> offsets{{1.57f,-1.57f,1.25f,-1.25f,0.95f,-0.95f,0.62f,-0.62f,0.32f,-0.32f,0.0f}};
    constexpr std::array<float,8> radii{{1.8f,2.4f,3.2f,4.2f,6.0f,9.0f,12.0f,18.0f}};
    // Candidates whose straight chord is blocked need a coarse A* search to be
    // accepted. Those searches are independent, so they are batched and run
    // across the worker pool, but results are consumed strictly in candidate
    // order: the chosen pull-out is identical to the serial scan.
    std::vector<Vec3> pending;
    const std::size_t batch=std::max<std::size_t>(1,std::min<std::size_t>(8,eng::cpu_parallel_lanes()));
    auto resolve_pending=[&](Vec3& chosen){
        if(pending.empty()) return false;
        std::vector<std::uint8_t> reachable(pending.size(),0);
        eng::TaskPool::shared().parallel_for(pending.size(),[&](std::size_t k){
            std::vector<Vec3> route;
            reachable[k]=world.coarse_route(a.id,a.position,pending[k],0.12f,route)?1:0;
        });
        for(std::size_t k=0;k<pending.size();++k){
            if(reachable[k]){ chosen=pending[k]; pending.clear(); return true; }
        }
        pending.clear();
        return false;
    };
    Vec3 chosen{};
    for(float r:radii){
        for(float off:offsets){
            const float ang=base+off;
            Vec3 p=a.position+Vec3{std::cos(ang),0.0f,std::sin(ang)}*r;
            if(world.floor_height) p.y=world.floor_height(p);
            if(world.point_clear && !world.point_clear(a.id,p)) continue;
            if(blocker<agents.size() && world.point_clear){
                // A pull-out must have room to pass. Retreating one metre
                // deeper into the same single-lane throat only repeats the
                // head-on deadlock at a different coordinate.
                const Vec3 side{-away.z,0,away.x};
                const float passing=a.radius_m+agents[blocker].radius_m+0.25f;
                if(!world.point_clear(a.id,p+side*passing) && !world.point_clear(a.id,p-side*passing)) continue;
            }
            bool fleet_clear=true;
            for(const auto id:a.runtime->nearby_robot_ids){
                const auto& other=agents[id];
                if(dist_xz(p,other.position)<=a.radius_m+other.radius_m+0.20f){ fleet_clear=false; break; }
            }
            if(!fleet_clear) continue;
            if(world.segment_clear && !world.segment_clear(a.id,a.position,p)){
                if(!world.coarse_route) continue;
                pending.push_back(p);
                if(pending.size()>=batch && resolve_pending(chosen)) return chosen;
                continue;
            }
            if(resolve_pending(chosen)) return chosen;
            return p;
        }
    }
    if(resolve_pending(chosen)) return chosen;
    return a.position;
}

bool stationary_for_detour(std::size_t observer_index,std::size_t blocker_index,
                           const std::vector<SafetyAgent>& agents,
                           const std::vector<NavigationRequest>& requests){
    if(observer_index>=agents.size() || blocker_index>=agents.size() || observer_index==blocker_index) return false;
    const auto& neighbors=agents[observer_index].runtime->nearby_robot_ids;
    if(!std::binary_search(neighbors.begin(),neighbors.end(),blocker_index)) return false;
    const auto& blocker=agents[blocker_index];
    const auto& rt=*blocker.runtime;
    if(std::abs(blocker.speed_mps)>0.30f) return false;

    const bool navigation_active=blocker_index<requests.size() && requests[blocker_index].active;
    if(!navigation_active || rt.motion_intent==MotionIntent::Wait ||
       rt.mode==SafetyMode::Idle || rt.mode==SafetyMode::Working ||
       rt.mode==SafetyMode::Blocked || rt.mode==SafetyMode::Waiting)
        return true;

    // A higher-priority rover that is only creeping around its work bay is a
    // stable traffic obstacle from the lower-priority rover's perspective.
    // Treating it as generic moving traffic made CR-01 repeatedly search the
    // same impossible path through a BR-02 work bay. The lower-priority rover
    // now computes one deterministic bypass while the builder keeps its task.
    const auto observer_priority=observer_index<requests.size()?requests[observer_index].task_priority:0u;
    const auto blocker_priority=blocker_index<requests.size()?requests[blocker_index].task_priority:0u;
    if(blocker_priority>observer_priority && std::abs(blocker.speed_mps)<=0.30f) return true;

    return false;
}

float route_length(Vec3 start,const std::vector<Vec3>& route){
    float total=0.0f;
    Vec3 previous=start;
    for(const Vec3 p:route){ total+=dist_xz(previous,p); previous=p; }
    return total;
}

bool point_respects_fleet_zone(const SafetyAgent& self,Vec3 p,const std::vector<SafetyAgent>& agents){
    for(const auto id:self.runtime->nearby_robot_ids){
        const auto& other=agents[id];
        const float required=self.radius_m+other.radius_m+2.0f*kSafetyNumericMarginM+
                             kNeighborComfortClearanceM;
        if(dist_xz(p,other.position)<required) return false;
    }
    return true;
}

bool find_stationary_detour(const SafetyAgent& a,std::size_t blocker_index,
                            const std::vector<SafetyAgent>& agents,
                            const SafetyWorldCallbacks& world,Vec3 goal,
                            float stop_radius,Vec3& out_waypoint){
    if(blocker_index>=agents.size() || !world.coarse_route) return false;
    const auto& blocker=agents[blocker_index];

    Vec3 travel=goal-a.position; travel.y=0.0f;
    float travel_len=std::sqrt(dot_xz(travel,travel));
    if(travel_len<0.05f){
        travel=blocker.position-a.position; travel.y=0.0f;
        travel_len=std::sqrt(dot_xz(travel,travel));
    }
    if(travel_len<0.05f) travel=forward_from_heading(a.heading_degrees);
    else travel=travel*(1.0f/travel_len);
    const Vec3 lateral{-travel.z,0.0f,travel.x};

    const float zone_radius=a.radius_m+blocker.radius_m+2.0f*kSafetyNumericMarginM+
                            kNeighborComfortClearanceM;
    // Sample both sides and several clearances.  The small forward bias places
    // the waypoint slightly beyond the blocker relative to the task direction,
    // preventing a route that merely stops abreast of the stationary rover and
    // immediately blocks again on the next replan.
    constexpr std::array<float,4> extra{{0.35f,0.75f,1.25f,1.85f}};
    constexpr std::array<float,3> forward_bias{{0.35f,0.85f,1.45f}};

    bool found=false;
    float best_score=std::numeric_limits<float>::infinity();
    Vec3 best{};
    // Deterministic side preference avoids left/right oscillation if two sides
    // have nearly identical costs, while still allowing the other side when it
    // is materially shorter or the preferred side is unavailable.
    const float preferred_sign=(a.id%2u)==0u?1.0f:-1.0f;
    struct Candidate { Vec3 position{}; float side_penalty{0.0f}; };
    std::vector<Candidate> candidates;
    candidates.reserve(extra.size()*forward_bias.size()*2u);
    for(float side_sign: {preferred_sign,-preferred_sign}){
        for(float add:extra){
            for(float ahead:forward_bias){
                Vec3 candidate=blocker.position+lateral*(side_sign*(zone_radius+add))+travel*ahead;
                if(world.floor_height) candidate.y=world.floor_height(candidate);
                if(world.point_clear && !world.point_clear(a.id,candidate)) continue;
                if(!point_respects_fleet_zone(a,candidate,agents)) continue;
                candidates.push_back({candidate,side_sign==preferred_sign?0.0f:0.12f});
            }
        }
    }
    // Each surviving waypoint needs its own coarse A* route. They are
    // independent, so evaluate them across the worker pool and then select the
    // winner in the original candidate order (first minimum wins), which keeps
    // the result identical to the serial search.
    std::vector<float> scores(candidates.size(),std::numeric_limits<float>::infinity());
    std::vector<std::uint8_t> reachable(candidates.size(),0);
    eng::TaskPool::shared().parallel_for(candidates.size(),[&](std::size_t k){
        std::vector<Vec3> route;
        if(!world.coarse_route(a.id,a.position,candidates[k].position,std::max(0.10f,stop_radius),route) || route.empty())
            return;
        // Prefer short reachable detours that still make progress
        // toward the original task goal.  A tiny penalty for the
        // non-preferred side keeps the decision stable across replans.
        scores[k]=route_length(a.position,route)+0.42f*dist_xz(candidates[k].position,goal)+candidates[k].side_penalty;
        reachable[k]=1;
    });
    for(std::size_t k=0;k<candidates.size();++k){
        if(reachable[k] && scores[k]<best_score){
            best_score=scores[k];
            best=candidates[k].position;
            found=true;
        }
    }
    if(found) out_waypoint=best;
    return found;
}

std::size_t nearest_stationary_blocker_on_goal_line(const SafetyAgent& a,
                                                     const SafetyRuntime& rt,
                                                     const std::vector<SafetyAgent>& agents,
                                                     const std::vector<NavigationRequest>& requests){
    if(!rt.goal_valid) return kNoSafetyRobot;
    Vec3 ab=rt.goal-a.position; ab.y=0.0f;
    const float len2=dot_xz(ab,ab);
    if(len2<0.04f) return kNoSafetyRobot;
    std::size_t best=kNoSafetyRobot;
    float best_t=std::numeric_limits<float>::infinity();
    for(const auto j:rt.nearby_robot_ids){
        if(!stationary_for_detour(a.id,j,agents,requests)) continue;
        Vec3 ap=agents[j].position-a.position; ap.y=0.0f;
        const float t=std::clamp(dot_xz(ap,ab)/len2,0.0f,1.0f);
        if(t<=0.01f) continue;
        const Vec3 closest=a.position+ab*t;
        const float required=a.radius_m+agents[j].radius_m+2.0f*kSafetyNumericMarginM+
                             kNeighborComfortClearanceM+0.55f;
        if(dist_xz(closest,agents[j].position)>required) continue;
        if(t<best_t){ best_t=t; best=j; }
    }
    return best;
}

void update_blocking_recovery(std::vector<SafetyAgent>& agents,const std::vector<NavigationRequest>& requests,const SafetyWorldCallbacks& world,float dt){
    auto request_idle_yield=[&](std::size_t blocked,std::size_t blocker){
        if(blocker>=agents.size() || blocker>=requests.size() || requests[blocker].active || requests[blocker].work_target) return false;
        auto& a=agents[blocker];auto& idle=*a.runtime;
        if(idle.mode==SafetyMode::Fault || idle.temporary_goal_valid) return false;
        const Vec3 goal=agents[blocked].runtime->goal;
        const float angle=std::atan2(a.position.z-goal.z,a.position.x-goal.x);
        for(float radius:{3.0f,4.2f,5.4f}) for(float offset:{0.0f,0.6f,-0.6f,1.2f,-1.2f,1.57f,-1.57f}){
            Vec3 p=a.position+Vec3{std::cos(angle+offset),0,std::sin(angle+offset)}*radius;
            if(world.floor_height) p.y=world.floor_height(p);
            if(dist_xz(p,goal)<a.radius_m+agents[blocked].radius_m+0.5f) continue;
            if(!world.point_clear(a.id,p) || !world.segment_clear(a.id,a.position,p)) continue;
            if(!point_respects_fleet_zone(a,p,agents)) continue;
            idle.temporary_goal=p;idle.temporary_goal_valid=true;idle.yielding_for_traffic=true;
            idle.motion_intent=MotionIntent::Retreat;idle.stationary_detour_active=false;
            idle.traffic_detour_partner=blocked;
            ++idle.goal_revision;++idle.retreat_count;invalidate_route(idle);
            idle.mode=SafetyMode::Retreating;idle.blocked_elapsed=0.0f;
            return true;
        }
        return false;
    };
    // First update progress and blocked timers.
    for(auto& a:agents){
        auto& rt=*a.runtime;
        if(rt.mode==SafetyMode::Fault)continue;
        if((rt.goal_valid || rt.temporary_goal_valid) && !rt.cached_route.empty()){
            const float remaining=corridor_steering(a,{a.position,a.heading_degrees,a.speed_mps},world).remaining;
            if(remaining+0.06f<rt.best_route_remaining){rt.best_route_remaining=remaining;rt.route_progress_elapsed=0.0f;}
            else rt.route_progress_elapsed+=dt;
            if(rt.route_progress_elapsed>2.0f){
                // Moving in circles is not route progress. Reconnect from the
                // measured pose, rather than resetting the stuck timer merely
                // because the rover moved a few centimetres in any direction.
                rt.cached_route.clear();rt.route_cursor=0;rt.proposals.clear();
                rt.route_replan_required=true;rt.route_progress_elapsed=0.0f;
                rt.best_route_remaining=std::numeric_limits<float>::infinity();
                rt.steering_side=0.0f;
            }
        }
        const float moved=dist_xz(a.position,rt.last_progress_position);
        if(moved>0.08f){
            rt.last_progress_position=a.position;
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
            rt.last_progress_time=0.0f;
            if(rt.block_reason!=BlockReason::NoRoute) rt.block_reason=BlockReason::None;
            continue;
        }
        if(rt.goal_valid || rt.temporary_goal_valid){
            rt.blocked_elapsed+=dt;
            if(rt.mode==SafetyMode::Waiting || rt.mode==SafetyMode::Braking) rt.wait_age+=dt;
        }else{
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
        }
        rt.recovery_cooldown=std::max(0.0f,rt.recovery_cooldown-dt);
    }

    // Detect simple wait-for cycles. This covers the common nose-to-nose and
    // multi-robot ring deadlocks without allowing a recovery to bypass safety.
    std::vector<int> state(agents.size(),0);
    std::vector<std::size_t> stack;
    std::vector<std::vector<std::size_t>> cycles;
    std::function<void(std::size_t)> dfs=[&](std::size_t u){
        state[u]=1;
        stack.push_back(u);
        const auto& rt=*agents[u].runtime;
        const std::size_t v=rt.blocker_robot_id;
        if(rt.mode!=SafetyMode::Fault && rt.blocked_elapsed>=kBlockingWaitTimeoutSeconds &&
           std::binary_search(rt.nearby_robot_ids.begin(),rt.nearby_robot_ids.end(),v) && agents[v].runtime->mode!=SafetyMode::Fault){
            if(state[v]==0) dfs(v);
            else if(state[v]==1){
                auto it=std::find(stack.begin(),stack.end(),v);
                if(it!=stack.end()) cycles.emplace_back(it,stack.end());
            }
        }
        stack.pop_back();
        state[u]=2;
    };
    for(std::size_t i=0;i<agents.size();++i) if(state[i]==0) dfs(i);

    std::unordered_set<std::size_t> retreat_assigned;
    for(const auto& cycle:cycles){
        if(cycle.empty()) continue;
        // Only a robot that currently has a navigation request may move for
        // traffic recovery. Idle/working robots remain fixed obstacles; active
        // robots must route, wait or resolve the deadlock amongst themselves.
        std::vector<std::size_t> movable;
        for(std::size_t i:cycle)
            if((i<requests.size() && requests[i].active) || agents[i].runtime->temporary_goal_valid)
                movable.push_back(i);
        if(movable.empty()) continue;
        // Lowest gameplay priority yields first. BR-02 therefore keeps the
        // right-of-way against CR-01 when both are active and neither already
        // owns the passage. Within one role, younger waiters yield before older
        // ones so aging still guarantees fairness.
        std::size_t chosen=movable.front();
        for(std::size_t i:movable){
            const auto& a=*agents[i].runtime;
            const auto& b=*agents[chosen].runtime;
            const auto ap=i<requests.size()?requests[i].task_priority:0u;
            const auto bp=chosen<requests.size()?requests[chosen].task_priority:0u;
            if(ap<bp ||
               (ap==bp && (a.wait_age+1e-4f<b.wait_age ||
                (std::abs(a.wait_age-b.wait_age)<1e-4f && agents[i].id>agents[chosen].id))))
                chosen=i;
        }
        auto& a=agents[chosen];
        auto& rt=*a.runtime;
        if(rt.mode==SafetyMode::Fault)continue;
        const Vec3 pullout=find_pullout(a,world,agents,rt.blocker_robot_id);
        if(dist_xz(pullout,a.position)>0.20f){
            rt.temporary_goal=pullout;
            rt.temporary_goal_valid=true;
            rt.stationary_detour_active=false;
            rt.traffic_detour_partner=rt.blocker_robot_id;
            rt.motion_intent=MotionIntent::Retreat;
            ++rt.goal_revision;
            ++rt.retreat_count;
            invalidate_route(rt);
            rt.mode=SafetyMode::Retreating;
            rt.blocked_elapsed=0.0f;
            retreat_assigned.insert(chosen);
        }else{
            rt.mode=SafetyMode::Blocked;
            rt.block_reason=BlockReason::NoRoute;
        }
    }

    // Persistent one-way blockage without a cycle requests a fresh coarse
    // detour. It does not directly move the robot and does not revoke anyone's
    // reservation.
    for(std::size_t i=0;i<agents.size();++i){
        auto& rt=*agents[i].runtime;
        if(rt.mode==SafetyMode::Fault)continue;
        if(retreat_assigned.count(i)) continue;
        if(rt.blocked_elapsed<kBlockingWaitTimeoutSeconds || rt.recovery_cooldown>0.0f) continue;
        rt.recovery_cooldown=1.25f;
        if(i<requests.size() && requests[i].active){
            if(rt.block_reason==BlockReason::NoRoute || rt.cached_route.empty()){
                // A parked row can seal the only coarse-grid entrance to a
                // wider route even when none of its members lies on the direct
                // goal chord. Give the closest idle blocker a real yield task;
                // moving and working actors keep their normal reservations.
                std::vector<std::size_t> idle;
                for(const auto j:rt.nearby_robot_ids){
                    if(requests[j].active || requests[j].work_target || agents[j].runtime->temporary_goal_valid || agents[j].runtime->yielding_for_traffic) continue;
                    idle.push_back(j);
                }
                std::sort(idle.begin(),idle.end(),[&](auto a,auto b){return dist_xz(agents[i].position,agents[a].position)<dist_xz(agents[i].position,agents[b].position);});
                bool yielded=false;
                for(auto j:idle) if(request_idle_yield(i,j)){yielded=true;break;}
                if(yielded){rt.route_replan_required=true;rt.proposals.clear();rt.blocked_elapsed=0.0f;continue;}
            }
            if(rt.stationary_detour_active && rt.temporary_goal_valid){
                // Keep one deterministic bypass stable across short pauses.
                // Re-run A* to that SAME waypoint if needed, but do not flip
                // sides or move the temporary target every wait timeout.
                rt.last_progress_time+=rt.blocked_elapsed;
                if(rt.last_progress_time>=2.5f){
                    finish_traffic_maneuver(rt);
                    rt.last_progress_time=0.0f;
                }else {rt.route_replan_required=true;rt.proposals.clear();}
                rt.blocked_elapsed=0.0f;
                continue;
            }
            std::size_t blocker=rt.blocker_robot_id;
            if(blocker>=agents.size() || !stationary_for_detour(i,blocker,agents,requests))
                blocker=nearest_stationary_blocker_on_goal_line(agents[i],rt,agents,requests);

            if(blocker<agents.size() && stationary_for_detour(i,blocker,agents,requests)){
                if(requests[blocker].active && std::abs(agents[blocker].speed_mps)<0.08f &&
                   agents[blocker].runtime->motion_intent!=MotionIntent::Retreat){
                    const auto own_priority=requests[i].task_priority,other_priority=requests[blocker].task_priority;
                    const std::size_t yielding=own_priority<other_priority?i:
                        (own_priority>other_priority?blocker:std::max(i,blocker));
                    auto& give_way=agents[yielding];auto& yield=*give_way.runtime;
                    if(yield.mode!=SafetyMode::Fault && yield.motion_intent!=MotionIntent::Retreat){
                        const Vec3 pocket=find_pullout(give_way,world,agents,yielding==i?blocker:i);
                        if(dist_xz(pocket,give_way.position)>0.5f){
                            yield.temporary_goal=pocket;yield.temporary_goal_valid=true;
                            yield.stationary_detour_active=false;yield.motion_intent=MotionIntent::Retreat;
                            yield.traffic_detour_partner=yielding==i?blocker:i;
                            ++yield.goal_revision;++yield.retreat_count;invalidate_route(yield);
                            yield.mode=SafetyMode::Retreating;yield.blocked_elapsed=0.0f;
                            retreat_assigned.insert(yielding);
                            continue;
                        }
                    }
                }
                const float endpoint_clearance=agents[i].radius_m+agents[blocker].radius_m+0.30f;
                if(dist_xz(rt.goal,agents[blocker].position)<endpoint_clearance && request_idle_yield(i,blocker)){
                    rt.route_replan_required=true;rt.proposals.clear();rt.blocked_elapsed=0.0f;
                    continue;
                }
                Vec3 detour{};
                if(find_stationary_detour(agents[i],blocker,agents,world,rt.goal,
                                          std::max(0.10f,rt.stop_radius),detour)){
                    rt.temporary_goal=detour;
                    rt.temporary_goal_valid=true;
                    rt.stationary_detour_active=true;
                    rt.traffic_detour_partner=agents[blocker].id;
                    rt.motion_intent=MotionIntent::Navigate;
                    ++rt.goal_revision;
                    ++rt.stationary_detour_count;
                    invalidate_route(rt);
                    rt.mode=SafetyMode::Moving;
                    rt.block_reason=BlockReason::None;
                    rt.blocker_robot_id=kNoSafetyRobot;
                    rt.blocked_elapsed=0.0f;
                    rt.wait_age=0.0f;
                    continue;
                }
                if(request_idle_yield(i,blocker)){
                    rt.route_replan_required=true;rt.proposals.clear();rt.blocked_elapsed=0.0f;
                    continue;
                }
            }

            // If no explicit stationary-obstacle bypass can be found, force a
            // fresh ordinary route search while preserving the stationary
            // blocker's reservation.  The rover remains safely waiting rather
            // than asking the parked robot to move.
            rt.route_replan_required=true;
            rt.proposals.clear();
            rt.blocked_elapsed=0.0f;
        }else if(rt.block_reason==BlockReason::NoRoute){
            rt.mode=SafetyMode::Blocked;
        }
    }

    for(auto& a:agents){
        auto& rt=*a.runtime;
        if(rt.mode==SafetyMode::Fault)continue;
        if(rt.temporary_goal_valid && dist_xz(a.position,rt.temporary_goal)<=std::max(0.08f,rt.stop_radius) && std::abs(a.speed_mps)<0.04f){
            finish_traffic_maneuver(rt);
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
        }
    }
}

} // namespace

bool moving_discs_overlap(const TubePiece& a,const TubePiece& b,float common_t0,float common_t1){
    if(common_t1<common_t0) return false;
    const float duration=std::max(0.0f,common_t1-common_t0);
    const Vec3 a0=interpolate(a,common_t0);
    const Vec3 b0=interpolate(b,common_t0);
    Vec3 va{},vb{};
    if(a.t1>a.t0+1.0e-7f) va=(a.end-a.start)*(1.0f/(a.t1-a.t0));
    if(b.t1>b.t0+1.0e-7f) vb=(b.end-b.start)*(1.0f/(b.t1-b.t0));
    Vec3 q=a0-b0;
    Vec3 w=va-vb;
    const float ww=dot_xz(w,w);
    float closest=0.0f;
    if(ww>1.0e-12f) closest=std::clamp(-dot_xz(q,w)/ww,0.0f,duration);
    const Vec3 separation=q+w*closest;
    // Tube/hold radii are already enlarged by kSafetyNumericMarginM when the
    // contract is built. Do not add it again here: initialization, contract
    // arbitration and the final shared-epoch guard must use one identical
    // safety envelope.
    const float radius=a.radius+b.radius;
    return dot_xz(separation,separation)<=radius*radius;
}

bool tubes_overlap(const std::vector<TubePiece>& a,const std::vector<TubePiece>& b){
    for(const auto& pa:a){
        for(const auto& pb:b){
            const float t0=std::max(pa.t0,pb.t0);
            const float t1=std::min(pa.t1,pb.t1);
            if(t1+1.0e-7f<t0) continue;
            if(moving_discs_overlap(pa,pb,t0,t1)) return true;
        }
    }
    return false;
}

namespace {

float moving_discs_clearance(const TubePiece& a,const TubePiece& b,float common_t0,float common_t1){
    if(common_t1<common_t0) return std::numeric_limits<float>::infinity();
    const float duration=std::max(0.0f,common_t1-common_t0);
    const Vec3 a0=interpolate(a,common_t0);
    const Vec3 b0=interpolate(b,common_t0);
    Vec3 va{},vb{};
    if(a.t1>a.t0+1.0e-7f) va=(a.end-a.start)*(1.0f/(a.t1-a.t0));
    if(b.t1>b.t0+1.0e-7f) vb=(b.end-b.start)*(1.0f/(b.t1-b.t0));
    const Vec3 q=a0-b0;
    const Vec3 w=va-vb;
    const float ww=dot_xz(w,w);
    float closest=0.0f;
    if(ww>1.0e-12f) closest=std::clamp(-dot_xz(q,w)/ww,0.0f,duration);
    const Vec3 separation=q+w*closest;
    return std::sqrt(std::max(0.0f,dot_xz(separation,separation)))-(a.radius+b.radius);
}

bool contract_center_at(const MotionContract& c,float t,Vec3& position,float& radius){
    for(const auto& piece:c.occupancy_tube){
        if(t+1.0e-5f<piece.t0 || t-1.0e-5f>piece.t1) continue;
        position=interpolate(piece,std::clamp(t,piece.t0,piece.t1));
        radius=piece.radius;
        return true;
    }
    for(const auto& hold:c.terminal_holds){
        if(t+1.0e-5f<hold.start_time) continue;
        position=hold.position;
        radius=hold.radius;
        return true;
    }
    return false;
}

float contracts_minimum_hard_clearance(const MotionContract& a,const MotionContract& b){
    float minimum=std::numeric_limits<float>::infinity();
    for(const auto& pa:a.occupancy_tube){
        for(const auto& pb:b.occupancy_tube){
            const float t0=std::max(pa.t0,pb.t0);
            const float t1=std::min(pa.t1,pb.t1);
            if(t1+1.0e-7f<t0) continue;
            minimum=std::min(minimum,moving_discs_clearance(pa,pb,t0,t1));
        }
        for(const auto& hold:b.terminal_holds){
            if(pa.t1+1.0e-7f<hold.start_time) continue;
            const float t0=std::max(pa.t0,hold.start_time);
            TubePiece stationary{t0,pa.t1,hold.position,hold.position,hold.radius};
            minimum=std::min(minimum,moving_discs_clearance(pa,stationary,t0,pa.t1));
        }
    }
    for(const auto& hold:a.terminal_holds){
        for(const auto& pb:b.occupancy_tube){
            if(pb.t1+1.0e-7f<hold.start_time) continue;
            const float t0=std::max(pb.t0,hold.start_time);
            TubePiece stationary{t0,pb.t1,hold.position,hold.position,hold.radius};
            minimum=std::min(minimum,moving_discs_clearance(stationary,pb,t0,pb.t1));
        }
        for(const auto& other_hold:b.terminal_holds){
            minimum=std::min(minimum,dist_xz(hold.position,other_hold.position)-
                                      (hold.radius+other_hold.radius));
        }
    }
    return minimum;
}

bool personal_zone_contract_clear(const MotionContract& candidate,const MotionContract& other){
    Vec3 candidate_now{},other_now{};
    float candidate_radius=0.0f,other_radius=0.0f;
    if(!contract_center_at(candidate,candidate.valid_start,candidate_now,candidate_radius) ||
       !contract_center_at(other,candidate.valid_start,other_now,other_radius))
        return true;

    const float current_clearance=dist_xz(candidate_now,other_now)-
                                  (candidate_radius+other_radius);
    const float minimum_clearance=contracts_minimum_hard_clearance(candidate,other);
    (void)current_clearance;

    // Existing saves may begin inside the personal zone. A new contract may
    // continue only if every normal/braking/terminal branch preserves or opens
    // that separation. This creates a one-way escape from the zone instead of
    // the accept/brake/accept oscillation seen in 0-40-35(1).gmsave.
    return minimum_clearance>=0.0f;
}

} // namespace

SafetyTickStats safety_tick(float dt,
                            float absolute_time,
                            std::uint64_t world_epoch,
                            std::vector<SafetyAgent>& agents,
                            const std::vector<NavigationRequest>& requests,
                            const SafetyWorldCallbacks& world){
    SafetyTickStats stats{};
    if(agents.empty() || dt<=0.0f) return stats;
    if(requests.size()!=agents.size() || !world.point_clear || !world.segment_clear){
        stats.invariant_fault=true;
        return stats;
    }
    for(std::size_t i=0;i<agents.size();++i){
        if(!agents[i].runtime || agents[i].id!=i){stats.invariant_fault=true;return stats;}
    }

    std::vector<std::size_t> neighbor_counts(agents.size());
    update_nearby_robot_awareness(agents,neighbor_counts);

    // Runs even when initial-geometry checks below must stop the shared epoch.
    recover_faulted_agents(dt,absolute_time,world_epoch,agents,requests,world);

    // Initialize only from a mutually clear stopped/held state. Existing overlap
    // is an assumption failure, not something the safety layer hides by nudging.
    for(auto& a:agents){
        if(!contract_valid_bounds(a)){
            a.runtime->initialized=true;
            enter_fault(a,BlockReason::InvalidMotionBounds,0,absolute_time);
            stats.invariant_fault=true;
            continue;
        }
        if(!a.runtime->initialized){
            // Initial geometry is checked exactly once. Runtime construction
            // changes are admitted through the scene/contract gates below;
            // re-labelling a conservative work-envelope change as UNSAFE
            // INITIAL GEOMETRY every rendered frame caused false BR-02 faults.
            if(!world.point_clear(a.id,a.position)){
                a.runtime->initialized=true;
                enter_fault(a,BlockReason::UnsafeInitialGeometry,0,absolute_time);
                stats.invariant_fault=true;
                continue;
            }
            a.runtime->last_progress_position=a.position;
            a.runtime->active_contract=stationary_contract(a,absolute_time);
            a.runtime->initialized=true;
            a.speed_mps=0.0f;
        }
    }
    for(std::size_t i=0;i<agents.size();++i){
        for(std::size_t j=i+1;j<agents.size();++j){
            if(dist_xz(agents[i].position,agents[j].position)<=agents[i].radius_m+agents[j].radius_m+2.0f*kSafetyNumericMarginM){
                enter_fault(agents[i],BlockReason::UnsafeInitialGeometry,0,absolute_time);
                enter_fault(agents[j],BlockReason::UnsafeInitialGeometry,0,absolute_time);
                stats.invariant_fault=true;
            }
        }
    }
    if(stats.invariant_fault){
        for(auto& a:agents) a.speed_mps=0.0f;
        return stats;
    }

    // Update requested goals without disturbing a temporary retreat goal.
    for(std::size_t i=0;i<agents.size();++i){
        auto& a=agents[i];
        auto& rt=*a.runtime;
        const auto& req=requests[i];
        const bool changed=req.active!=rt.goal_valid ||
            (req.active && (dist_xz(req.goal,rt.goal)>0.04f || std::abs(req.stop_radius-rt.stop_radius)>0.02f || std::abs(req.speed_scale-rt.speed_scale)>0.03f));
        if(changed){
            rt.goal_valid=req.active;
            rt.goal=req.goal;
            rt.stop_radius=std::max(0.02f,req.stop_radius);
            rt.speed_scale=std::clamp(req.speed_scale,0.10f,1.0f);
            ++rt.goal_revision;
            rt.temporary_goal_valid=false;
            rt.yielding_for_traffic=false;
            rt.stationary_detour_active=false;
            rt.traffic_detour_partner=kNoSafetyRobot;
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
            invalidate_route(rt);
        }
        rt.motion_intent=rt.temporary_goal_valid?MotionIntent::Retreat:(rt.goal_valid?MotionIntent::Navigate:MotionIntent::Wait);
        if(rt.mode!=SafetyMode::Fault && req.work_target && !req.active && std::abs(a.speed_mps)<0.03f) rt.mode=SafetyMode::Working;
    }

    // Never authorize an unchecked huge step. Process at 10 ms safety cadence;
    // if the caller supplies more than one simulated second, the excess is a
    // stationary hold rather than a long unchecked movement.
    float remaining=std::min(dt,1.0f);
    float now=absolute_time;
    // Route searches are independent per rover and are the main CPU-heavy part
    // of dense-base simulation. Concurrent heavy searches are bounded by
    // configured_route_search_lanes() (at most two by default, independent of
    // host size, so fleet behaviour does not depend on core count); each
    // individual D* search is expansion-bounded/resumable, so no rover becomes
    // an unbounded frame-time task. Cheap proposal work still uses every lane.
    const std::size_t visible_lanes=std::max<std::size_t>(1,eng::cpu_parallel_lanes());
    const std::size_t route_search_lane_limit=std::min<std::size_t>(
        agents.size(),configured_route_search_lanes(visible_lanes));
    std::size_t remaining_route_search_budget=route_search_lane_limit;
    while(remaining>1.0e-6f){
        const float tick=std::min(kSafetyDtSeconds,remaining);

        // The measured simulation state must remain inside the previously
        // certified contract. A violation is an assumption failure: stop the
        // shared epoch instead of attempting a positional repair.
        bool prior_contract_violation=false;
        for(auto& a:agents){
            if(a.runtime->mode==SafetyMode::Fault) continue;
            if(state_inside_active_contract(a,now)) continue;

            // A stopped rover can recover locally from a stale/expired contract
            // as long as its current occupied position is still physically clear.
            // This is especially important at construction-stage boundaries,
            // where a BR-02 work hold remains safe while the conservative
            // blueprint envelope changes around its prevalidated service bay.
            if(std::abs(a.speed_mps)<0.03f && world.point_clear(a.id,a.position)){
                a.speed_mps=0.0f;
                a.runtime->active_contract=stationary_contract(a,now);
                a.runtime->block_reason=BlockReason::ContractViolation;
                a.runtime->route_replan_required=true;
                a.runtime->proposals.clear();
                continue;
            }

            enter_fault(a,BlockReason::ContractViolation,1,now);
            prior_contract_violation=true;
        }
        if(prior_contract_violation){
            stats.invariant_fault=true;
            for(auto& a:agents) if(a.runtime->mode==SafetyMode::Fault) a.speed_mps=0.0f;
            break;
        }

        // Goal/clock bookkeeping remains serialized because it is tiny and it
        // establishes a stable snapshot for the parallel read-mostly planner
        // phase below.
        for(std::size_t ai=0;ai<agents.size();++ai){
            auto& a=agents[ai];
            auto& rt=*a.runtime;
            rt.planner_elapsed+=tick;
            rt.route_retry_elapsed+=tick;
            if(rt.mode==SafetyMode::Fault) continue;
            if(reached_target(a,rt) && std::abs(a.speed_mps)<0.035f){
                if(rt.temporary_goal_valid){
                    finish_traffic_maneuver(rt);
                }else if(rt.goal_valid){
                    rt.motion_intent=MotionIntent::Wait;
                    rt.goal_valid=false;
                    ++rt.goal_revision;
                    invalidate_route(rt);
                }
            }
        }

        // World-revision validation can be surprisingly expensive in a large
        // colony because every remaining route segment is tested against the
        // current structure/resource geometry. Each rover owns its route state,
        // and the world callbacks are read-only in this phase, so validate all
        // affected routes concurrently instead of serially on the main thread.
        std::vector<std::size_t> revision_jobs;
        revision_jobs.reserve(agents.size());
        for(std::size_t ai=0;ai<agents.size();++ai){
            const auto& rt=*agents[ai].runtime;
            if(rt.mode==SafetyMode::Fault) continue;
            if(!rt.goal_valid && !rt.temporary_goal_valid) continue;
            if(rt.route_goal_revision!=rt.goal_revision) continue;
            if(rt.route_world_epoch==world_epoch) continue;
            revision_jobs.push_back(ai);
        }
        eng::TaskPool::shared().parallel_for(revision_jobs.size(),[&](std::size_t job){
            refresh_route_world_revision(agents[revision_jobs[job]],world_epoch,world);
        });

        update_reciprocal_velocities(agents,world,tick);

        std::vector<std::size_t> planner_jobs;
        std::vector<std::size_t> full_route_candidates;
        planner_jobs.reserve(agents.size());
        full_route_candidates.reserve(agents.size());
        for(std::size_t ai=0;ai<agents.size();++ai){
            auto& rt=*agents[ai].runtime;
            if(rt.mode==SafetyMode::Fault) continue;
            // A stationary terminal hold needs no route or local rollout.
            // Idle actors otherwise retain route_replan_required after arrival
            // and can consume every budget slot ahead of low-priority returns.
            // Their certified hold is still checked/arbitrated below.
            if(!rt.goal_valid && !rt.temporary_goal_valid && std::abs(agents[ai].speed_mps)<0.003f){
                rt.route_replan_required=false;
                rt.planner_elapsed=0.0f;
                continue;
            }
            const float interval=kPlannerReplanSeconds+planner_phase_offset_seconds(ai);
            if(rt.planner_elapsed>=interval || rt.proposals.empty() || rt.route_replan_required){
                planner_jobs.push_back(ai);
            }
        }

        // Limit planning work per tick to smooth worst-case frame spikes.
        std::stable_sort(planner_jobs.begin(),planner_jobs.end(),[&](std::size_t lhs,std::size_t rhs){
            const auto& a=*agents[lhs].runtime;
            const auto& b=*agents[rhs].runtime;
            const bool a_hard=a.route_replan_required || a.proposals.empty();
            const bool b_hard=b.route_replan_required || b.proposals.empty();
            if(a_hard!=b_hard) return a_hard>b_hard;
            const auto lp=lhs<requests.size()?requests[lhs].task_priority:0u;
            const auto rp=rhs<requests.size()?requests[rhs].task_priority:0u;
            if(lp!=rp) return lp>rp;
            const float lw=a.blocked_elapsed+a.wait_age;
            const float rw=b.blocked_elapsed+b.wait_age;
            if(std::abs(lw-rw)>1.0e-4f) return lw>rw;
            return agents[lhs].id<agents[rhs].id;
        });
        const std::size_t planner_budget=planner_job_budget_per_tick(agents.size(),visible_lanes);
        if(planner_jobs.size()>planner_budget) planner_jobs.resize(planner_budget);

        for(const std::size_t ai:planner_jobs)
            if(route_search_due(agents[ai],world_epoch)) full_route_candidates.push_back(ai);

        // Grant the limited full-search slots to the most important/most blocked
        // navigation requests first. All other planner jobs may still refresh
        // their cheap local steering proposals in parallel.
        std::stable_sort(full_route_candidates.begin(),full_route_candidates.end(),[&](std::size_t lhs,std::size_t rhs){
            const auto lp=lhs<requests.size()?requests[lhs].task_priority:0u;
            const auto rp=rhs<requests.size()?requests[rhs].task_priority:0u;
            if(lp!=rp) return lp>rp;
            const float lw=agents[lhs].runtime->blocked_elapsed+agents[lhs].runtime->wait_age;
            const float rw=agents[rhs].runtime->blocked_elapsed+agents[rhs].runtime->wait_age;
            if(std::abs(lw-rw)>1.0e-4f) return lw>rw;
            return agents[lhs].id<agents[rhs].id;
        });
        std::vector<std::uint8_t> route_search_granted(agents.size(),0);
        const std::size_t grant_count=std::min(remaining_route_search_budget,full_route_candidates.size());
        for(std::size_t i=0;i<grant_count;++i) route_search_granted[full_route_candidates[i]]=1;
        remaining_route_search_budget-=grant_count;

        // Put the expensive route-repair jobs at the front of the dynamic work
        // queue. This lets all helper threads claim heavy work immediately while
        // the main thread and any spare workers consume cheap local-proposal
        // refreshes. Completion order does not affect arbitration, which remains
        // deterministic and serialized below.
        std::stable_sort(planner_jobs.begin(),planner_jobs.end(),[&](std::size_t lhs,std::size_t rhs){
            const bool lh=route_search_granted[lhs]!=0;
            const bool rh=route_search_granted[rhs]!=0;
            if(lh!=rh) return lh>rh;
            const auto lp=lhs<requests.size()?requests[lhs].task_priority:0u;
            const auto rp=rhs<requests.size()?requests[rhs].task_priority:0u;
            if(lp!=rp) return lp>rp;
            return agents[lhs].id<agents[rhs].id;
        });

        // Local proposal preview and persistent D* repair are independent per
        // rover. Dynamic scheduling across the host-wide pool allocates all
        // visible CPU lanes to the current fleet workload. Reservation/contract
        // arbitration and world mutation remain on the game thread.
        auto run_planner_job=[&](std::size_t job){
            const std::size_t ai=planner_jobs[job];
            agents[ai].runtime->planner_elapsed=0.0f;
            generate_proposals(agents[ai],world_epoch,world,agents,now,route_search_granted[ai]!=0);
        };
        // Cheap local-steering refreshes take well under a millisecond each;
        // waking helper threads for a couple of them costs more than it saves
        // and makes the frame wait on the slowest wake-up. Only fan out when a
        // heavy route search is granted or the batch is large.
        bool heavy_batch=planner_jobs.size()>3;
        for(const std::size_t ai:planner_jobs) heavy_batch=heavy_batch || route_search_granted[ai]!=0;
        if(heavy_batch) eng::TaskPool::shared().parallel_for(planner_jobs.size(),run_planner_job);
        else for(std::size_t job=0;job<planner_jobs.size();++job) run_planner_job(job);

        ReservationBoard board{};
        board.contracts.resize(agents.size());
        for(const auto& a:agents) board.contracts[a.id]=a.runtime->active_contract.valid?a.runtime->active_contract:stationary_contract(a,now);
        rebuild_passages(board);

        std::vector<std::size_t> order(agents.size());
        for(std::size_t i=0;i<order.size();++i) order[i]=i;
        std::stable_sort(order.begin(),order.end(),[&](std::size_t lhs,std::size_t rhs){
            const auto& a=*agents[lhs].runtime;
            const auto& b=*agents[rhs].runtime;
            const bool a_owns=!a.active_contract.passage_keys.empty();
            const bool b_owns=!b.active_contract.passage_keys.empty();
            if(a_owns!=b_owns) return a_owns>b_owns;
            const bool a_retreat=agents[lhs].runtime->motion_intent==MotionIntent::Retreat;
            const bool b_retreat=agents[rhs].runtime->motion_intent==MotionIntent::Retreat;
            if(a_retreat!=b_retreat) return a_retreat>b_retreat;
            const auto a_priority=lhs<requests.size()?requests[lhs].task_priority:0u;
            const auto b_priority=rhs<requests.size()?requests[rhs].task_priority:0u;
            if(a_priority!=b_priority) return a_priority>b_priority;
            if(std::abs(a.wait_age-b.wait_age)>1.0e-4f) return a.wait_age>b.wait_age;
            return agents[lhs].id<agents[rhs].id;
        });

        struct CommandResult { Vec3 position{}; float heading{0.0f}; float speed{0.0f}; bool accepted{false}; bool brake{false}; };
        std::vector<CommandResult> commands(agents.size());
        for(std::size_t i=0;i<agents.size();++i) commands[i]={agents[i].position,agents[i].heading_degrees,agents[i].speed_mps,false,false};

        for(std::size_t oi:order){
            auto& a=agents[oi];
            auto& rt=*a.runtime;
            const std::size_t planning_blocker=rt.blocker_robot_id;
            rt.blocker_robot_id=kNoSafetyRobot;
            if(rt.mode==SafetyMode::Fault) continue;

            // A stopped WAIT/WORK rover already owns an indefinite terminal hold.
            // Do not rebuild/follow a braking contract every 10 ms while it is
            // stationary. This was pure CPU work (thousands of fallbacks per
            // minute) and caused the FPS drop as construction geometry grew.
            if(rt.motion_intent==MotionIntent::Wait && std::abs(a.speed_mps)<0.003f){
                bool hold_matches=false;
                if(rt.active_contract.valid){
                    for(const auto& hold:rt.active_contract.terminal_holds){
                        if(now+1.0e-5f>=hold.start_time &&
                           dist_xz(a.position,hold.position)<=hold.radius){
                            hold_matches=true;
                            break;
                        }
                    }
                }
                if(!hold_matches) rt.active_contract=stationary_contract(a,now);
                replace_contract(board,rt.active_contract);
                commands[a.id]={a.position,a.heading_degrees,0.0f,false,false};
                rt.mode=requests[a.id].work_target?SafetyMode::Working:SafetyMode::Idle;
                rt.block_reason=BlockReason::None;
                continue;
            }

            MotionContract chosen{};
            BlockReason last_reason=rt.block_reason;
            std::size_t last_blocker=planning_blocker;

            if(rt.motion_intent!=MotionIntent::Wait){
                for(const auto& proposal:rt.proposals){
                    const bool zero_speed=std::abs(proposal.control.target_speed_mps)<0.001f;
                    const bool no_turn=std::abs(wrap_degrees(proposal.control.target_heading_degrees-a.heading_degrees))<0.5f;
                    if(zero_speed && no_turn) continue;
                    MotionContract candidate=build_contract(a,proposal.control,now,tick,world);
                    // Temporarily remove the robot's old reservation while
                    // testing its atomic replacement.
                    MotionContract old=board.contracts[a.id];
                    board.contracts[a.id]=MotionContract{};
                    rebuild_passages(board);
                    std::size_t blocker=kNoSafetyRobot;
                    BlockReason reason=BlockReason::None;
                    const bool clear=contract_clear(candidate,world,board,blocker,reason);
                    board.contracts[a.id]=old;
                    rebuild_passages(board);
                    if(!clear){
                        ++rt.rejected_contracts;
                        ++stats.rejected_contracts;
                        last_reason=reason;
                        last_blocker=blocker;
                        continue;
                    }
                    chosen=std::move(candidate);
                    replace_contract(board,chosen);
                    break;
                }
            }

            if(chosen.valid){
                RolloutState s{a.position,a.heading_degrees,a.speed_mps};
                s=step_control(a,s,chosen.permitted_command,tick,world);
                commands[a.id]={s.position,s.heading,s.speed,true,false};
                rt.active_contract=chosen;
                if(std::abs(chosen.permitted_command.heading_offset_degrees)>5.0f)
                    rt.steering_side=chosen.permitted_command.heading_offset_degrees>0.0f?1.0f:-1.0f;
                rt.block_reason=BlockReason::None;
                rt.blocker_robot_id=kNoSafetyRobot;
                rt.mode=rt.motion_intent==MotionIntent::Retreat?SafetyMode::Retreating:SafetyMode::Moving;
                ++rt.accepted_contracts;
                ++stats.accepted_contracts;
            }else{
                const BlockReason effective_reason=last_reason==BlockReason::None?rt.block_reason:last_reason;
                // If a navigating rover is already stopped, a rejected/newly
                // unavailable route needs no brake rollout at all. Collapse it
                // to one persistent stationary contract and let the planner or
                // detour recovery wake it when a usable route exists. This
                // removes the thousands of zero-distance brake fallbacks seen
                // in the 0-40-40 save while preserving the occupied-space hold.
                if(std::abs(a.speed_mps)<0.003f){
                    rt.active_contract=stationary_contract(a,now);
                    replace_contract(board,rt.active_contract);
                    commands[a.id]={a.position,a.heading_degrees,0.0f,false,false};
                    rt.block_reason=effective_reason;
                    rt.blocker_robot_id=last_blocker;
                    rt.mode=effective_reason==BlockReason::NoRoute?SafetyMode::Blocked:SafetyMode::Waiting;
                }else{
                    // A rover that is still moving follows only the braking
                    // branch already certified by its active contract.
                    Vec3 p=a.position;
                    float next_speed=a.speed_mps;
                    const bool certified=follow_certified_brake_branch(a,now,tick,world,p,next_speed);
                    commands[a.id]={p,a.heading_degrees,next_speed,false,true};
                    if(!certified){
                        enter_fault(a,BlockReason::ContractViolation,2,now);
                        commands[a.id]={a.position,a.heading_degrees,0.0f,false,false};
                        stats.invariant_fault=true;
                    }else{
                        rt.block_reason=effective_reason;
                        rt.blocker_robot_id=last_blocker;
                        if(rt.motion_intent==MotionIntent::Wait && std::abs(next_speed)<0.03f)
                            rt.mode=requests[a.id].work_target?SafetyMode::Working:SafetyMode::Idle;
                        else if(std::abs(next_speed)>0.03f)
                            rt.mode=SafetyMode::Braking;
                        else
                            rt.mode=rt.block_reason==BlockReason::NoRoute?SafetyMode::Blocked:SafetyMode::Waiting;
                        ++rt.brake_fallbacks;
                        ++stats.brake_fallbacks;
                    }
                }
            }
        }

        // Final shared-epoch guard. A static-integration failure is contained
        // to the offending rover whenever its CURRENT position is still valid;
        // unrelated robots keep their independently certified commands. Pair
        // overlap implicates both members of that pair. A world-wide invariant
        // fault is reserved for the case where even the current occupied state
        // is invalid and there is no local safe hold to commit.
        std::vector<TubePiece> actual(agents.size());
        std::vector<bool> local_fault(agents.size(),false);
        for(std::size_t i=0;i<agents.size();++i){
            actual[i]={now,now+tick,agents[i].position,commands[i].position,
                       motion_tube_radius(agents[i],commands[i].position,tick)};
            if(!world.segment_clear(i,agents[i].position,commands[i].position)) local_fault[i]=true;
        }
        for(std::size_t i=0;i<agents.size();++i){
            if(local_fault[i]) continue;
            for(std::size_t j=i+1;j<agents.size();++j){
                if(local_fault[j]) continue;
                if(moving_discs_overlap(actual[i],actual[j],now,now+tick)){
                    local_fault[i]=true;
                    local_fault[j]=true;
                }
            }
        }
        // Replacing one rejected command by a hold changes the swept occupancy
        // seen by the rest of the fleet. Propagate that hold through dependent
        // commands until the complete simultaneous commit is clear again.
        for(std::size_t pass=0;pass<agents.size();++pass){
            bool changed=false;
            for(std::size_t i=0;i<agents.size();++i){
                if(local_fault[i]){
                    actual[i].end=agents[i].position;
                    actual[i].radius=agents[i].radius_m+kSafetyNumericMarginM;
                }
            }
            for(std::size_t i=0;i<agents.size();++i) for(std::size_t j=i+1;j<agents.size();++j){
                if(local_fault[i] && local_fault[j]) continue;
                if(moving_discs_overlap(actual[i],actual[j],now,now+tick)){
                    if(!local_fault[i]){local_fault[i]=true;changed=true;}
                    if(!local_fault[j]){local_fault[j]=true;changed=true;}
                }
            }
            if(!changed) break;
        }
        bool unrecoverable=false;
        for(std::size_t i=0;i<agents.size();++i){
            if(!local_fault[i]) continue;
            if(!world.point_clear(i,agents[i].position)){
                unrecoverable=true;
                continue;
            }
            // The offending proposed substep has not been committed yet. Hold
            // this rover at its last valid position and reset only its local
            // navigation contract; the task-level watchdog can then replan it.
            commands[i]={agents[i].position,agents[i].heading_degrees,0.0f,false,false};
            agents[i].runtime->active_contract=stationary_contract(agents[i],now);
            if(agents[i].runtime->mode==SafetyMode::Fault)continue;
            agents[i].runtime->mode=SafetyMode::Blocked;
            agents[i].runtime->block_reason=BlockReason::ContractViolation;
            agents[i].runtime->route_replan_required=true;
            agents[i].runtime->proposals.clear();
            agents[i].runtime->blocked_elapsed=std::max(agents[i].runtime->blocked_elapsed,kBlockingWaitTimeoutSeconds);
        }
        if(unrecoverable){
            stats.invariant_fault=true;
            for(std::size_t i=0;i<agents.size();++i){
                if(!local_fault[i]) continue;
                enter_fault(agents[i],BlockReason::ContractViolation,3,now);
            }
            break;
        }

        for(std::size_t i=0;i<agents.size();++i){
            if(auto* wheels=agents[i].wheels){
                const float turn=wrap_degrees(commands[i].heading-agents[i].heading_degrees);
                const float yaw=turn*PI/180.0f;
                const Vec3 tangent=forward_from_heading(agents[i].heading_degrees+.5f*turn);
                const float distance=dot_xz(commands[i].position-agents[i].position,tangent)/arc_chord_scale(yaw);
                roll_rover_wheels(*wheels,distance,yaw);
            }
            agents[i].position=commands[i].position;
            agents[i].heading_degrees=commands[i].heading;
            agents[i].speed_mps=commands[i].speed;
            // Keep the old contract while following its certified brake branch;
            // its terminal hold remains reserved. Once stopped inside that hold,
            // collapse it to an equivalent stationary reservation.
            if(!commands[i].accepted && std::abs(agents[i].speed_mps)<0.003f){
                const auto& old=agents[i].runtime->active_contract;
                bool in_terminal=false;
                for(const auto& hold:old.terminal_holds){
                    if(now+tick+1.0e-5f>=hold.start_time &&
                       dist_xz(agents[i].position,hold.position)<=hold.radius){
                        in_terminal=true; break;
                    }
                }
                if(in_terminal) agents[i].runtime->active_contract=stationary_contract(agents[i],now+tick);
            }
        }

        update_nearby_robot_awareness(agents,neighbor_counts);
        update_blocking_recovery(agents,requests,world,tick);
        now+=tick;
        remaining-=tick;
    }

    return stats;
}

void restart_rover_route(SafetyRuntime& runtime){
    runtime.temporary_goal_valid=false;
    runtime.stationary_detour_active=false;
    runtime.traffic_detour_partner=kNoSafetyRobot;
    runtime.yielding_for_traffic=false;
    runtime.motion_intent=runtime.goal_valid?MotionIntent::Navigate:MotionIntent::Wait;
    runtime.planner_elapsed=kPlannerReplanSeconds;
    runtime.avoidance_elapsed=kPlannerReplanSeconds;
    runtime.steering_side=0.0f;
    runtime.blocked_elapsed=0.0f;
    runtime.route_progress_elapsed=0.0f;
    runtime.best_route_remaining=std::numeric_limits<float>::infinity();
    // Keep the current braking/hold contract until the navigator can
    // replace it with a certified command on the freshly calculated route.
    invalidate_route(runtime);
}

bool contract_intersects_region(const MotionContract& contract,
                                const std::function<bool(Vec3,float)>& circle_intersects_region){
    if(!contract.valid || !circle_intersects_region) return false;
    for(const auto& piece:contract.occupancy_tube){
        const float length=dist_xz(piece.start,piece.end);
        const int samples=std::max(1,(int)std::ceil(length/0.20f));
        for(int i=0;i<=samples;++i){
            const Vec3 p=piece.start+(piece.end-piece.start)*((float)i/(float)samples);
            if(circle_intersects_region(p,piece.radius)) return true;
        }
    }
    for(const auto& hold:contract.terminal_holds)
        if(circle_intersects_region(hold.position,hold.radius)) return true;
    return false;
}

} // namespace game::robots
