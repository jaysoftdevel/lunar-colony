#include "game/robots/safety_coordinator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
float turn_towards(float current,float target,float max_delta){
    return wrap_degrees(current+std::clamp(wrap_degrees(target-current),-max_delta,max_delta));
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

RolloutState step_control(const SafetyAgent& agent,RolloutState state,const MotionControl& control,float dt,const SafetyWorldCallbacks& world){
    const float turn=std::max(0.0f,agent.turn_rate_degrees_per_second)*dt;
    state.heading=turn_towards(state.heading,control.target_heading_degrees,turn);
    const float target=std::clamp(control.target_speed_mps,-agent.max_speed_mps,agent.max_speed_mps);
    const float rate=(std::abs(target)<std::abs(state.speed) || state.speed*target<0.0f)?agent.braking_mps2:agent.acceleration_mps2;
    state.speed=approach(state.speed,target,std::max(0.0f,rate)*dt);
    const Vec3 forward=forward_from_heading(state.heading);
    state.position=state.position+forward*(state.speed*dt);
    if(world.floor_height) state.position.y=world.floor_height(state.position);
    return state;
}

constexpr float kProposalPreviewDtSeconds=0.10f;

bool preview_static_clear(const SafetyAgent& agent,const MotionControl& control,const SafetyWorldCallbacks& world,std::vector<Vec3>& preview,float& final_heading,float& final_speed){
    preview.clear();
    RolloutState s{agent.position,agent.heading_degrees,agent.speed_mps};
    Vec3 previous=s.position;
    const int steps=(int)std::ceil(kPlannerHorizonSeconds/kProposalPreviewDtSeconds);
    for(int i=0;i<steps;++i){
        s=step_control(agent,s,control,kProposalPreviewDtSeconds,world);
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
        if(clearance>=kNeighborComfortClearanceM) return true;
        // A loaded/legacy save can already violate the newly introduced
        // personal zone without violating the hard hull. Do not trap or flicker
        // the active rover in place: while inside the zone, permit only motion
        // that maintains or increases separation from the other rover.
        return escaping_existing_zone &&
               clearance+kPersonalZoneEscapeEpsilonM>=current_clearance;
    };

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
    for(std::size_t sample=0;sample<preview.size();++sample){
        const float t=now+(float)(sample+1)*kProposalPreviewDtSeconds;
        for(const auto& other:agents){
            if(other.id==self.id) continue;
            const float current_distance=dist_xz(self.position,other.position);
            const float dynamic_awareness=kNeighborAwarenessRadiusM+
                (std::abs(self.speed_mps)+std::abs(other.speed_mps))*kPlannerHorizonSeconds;
            if(current_distance>dynamic_awareness) continue;
            if(!point_clear_of_predicted_robot(self,preview[sample],t,other,min_clearance)) return false;
        }
    }
    return true;
}

void update_nearby_robot_awareness(std::vector<SafetyAgent>& agents){
    for(auto& a:agents){
        auto& rt=*a.runtime;
        rt.nearby_robot_count=0;
        rt.nearest_robot_id=kNoSafetyRobot;
        rt.nearest_robot_distance_m=std::numeric_limits<float>::infinity();
        rt.predicted_robot_clearance_m=std::numeric_limits<float>::infinity();
    }
    for(std::size_t i=0;i<agents.size();++i){
        for(std::size_t j=i+1;j<agents.size();++j){
            const float d=dist_xz(agents[i].position,agents[j].position);
            const float awareness=kNeighborAwarenessRadiusM+
                (std::abs(agents[i].speed_mps)+std::abs(agents[j].speed_mps))*kPlannerHorizonSeconds;
            if(d>awareness) continue;
            auto& a=*agents[i].runtime;
            auto& b=*agents[j].runtime;
            ++a.nearby_robot_count;
            ++b.nearby_robot_count;
            if(d<a.nearest_robot_distance_m){ a.nearest_robot_distance_m=d; a.nearest_robot_id=agents[j].id; }
            if(d<b.nearest_robot_distance_m){ b.nearest_robot_distance_m=d; b.nearest_robot_id=agents[i].id; }
            // Telemetry reports clearance to the personal-zone boundary, not
            // merely clearance between the hard hull discs. Negative means the
            // pair is already inside the requested comfort zone and must separate.
            const float clearance=d-(agents[i].radius_m+agents[j].radius_m+
                                      2.0f*kSafetyNumericMarginM+kNeighborComfortClearanceM);
            a.predicted_robot_clearance_m=std::min(a.predicted_robot_clearance_m,clearance);
            b.predicted_robot_clearance_m=std::min(b.predicted_robot_clearance_m,clearance);
        }
    }
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
}

void advance_route_cursor(const SafetyAgent& a,SafetyRuntime& rt){
    while(rt.route_cursor<rt.cached_route.size() && dist_xz(a.position,rt.cached_route[rt.route_cursor])<0.42f)
        ++rt.route_cursor;
}

Vec3 route_waypoint(const SafetyAgent& a,SafetyRuntime& rt,const SafetyWorldCallbacks& world){
    advance_route_cursor(a,rt);
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
        if(world.segment_clear && !world.segment_clear(a.id,a.position,rt.cached_route[i])) break;
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
    advance_route_cursor(a,rt);
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
    if(world.coarse_route) ok=world.coarse_route(a.id,a.position,target,radius,route);
    if(!ok && world.segment_clear && world.segment_clear(a.id,a.position,target)){
        route={target};
        ok=true;
    }
    rt.cached_route=ok?std::move(route):std::vector<Vec3>{};
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
    const float desired=heading_between(a.position,waypoint,a.heading_degrees);
    const float cruise=std::max(0.08f,a.max_speed_mps*std::clamp(rt.speed_scale,0.15f,1.0f));
    const float target_distance=dist_xz(a.position,current_target(rt));
    const float braking_room=std::max(0.0f,target_distance-std::max(0.03f,rt.stop_radius));
    const float stopping_speed_cap=std::sqrt(std::max(0.0f,2.0f*a.braking_mps2*braking_room))*0.90f;
    const float planned_cruise=std::min(cruise,stopping_speed_cap);

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
                rt.proposals.push_back({pivot,std::move(preview),0.20f+std::abs(heading_error)*0.002f});
            rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
        }
    }
    constexpr std::array<float,9> angle_offsets{{0.0f,-14.0f,14.0f,-34.0f,34.0f,-52.0f,52.0f,-72.0f,72.0f}};
    constexpr std::array<float,3> speed_scales{{1.0f,0.58f,0.34f}};
    // Normal travel should be cheap: follow the coarse route with a small set
    // of local corrections. Only a rover that is actually blocked/retreating
    // expands to the full steering fan. The final contract arbitration remains
    // unchanged, so reducing proposal count lowers CPU cost without weakening
    // the collision authority.
    const bool recovery_search=rt.stationary_detour_active || rt.motion_intent==MotionIntent::Retreat ||
        rt.blocked_elapsed>=0.30f || rt.mode==SafetyMode::Waiting || rt.mode==SafetyMode::Blocked;
    const std::size_t angle_count=recovery_search?angle_offsets.size():5u;
    const std::size_t speed_count=recovery_search?speed_scales.size():2u;

    for(std::size_t si=0;si<speed_count;++si){
        const float speed_scale=speed_scales[si];
        for(std::size_t ai=0;ai<angle_count;++ai){
            const float offset=angle_offsets[ai];
            MotionControl control{};
            control.target_speed_mps=planned_cruise*speed_scale;
            control.target_heading_degrees=desired+offset;
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
            const float progress=dist_xz(a.position,waypoint)-dist_xz(end,waypoint);
            const float heading_cost=std::abs(wrap_degrees(control.target_heading_degrees-desired))*0.018f;
            const float smoothness=std::abs(wrap_degrees(control.target_heading_degrees-a.heading_degrees))*0.008f;
            const float slow_cost=(1.0f-speed_scale)*0.65f;
            const float cost=-progress+heading_cost+smoothness+slow_cost;
            rt.proposals.push_back({control,std::move(preview),cost});
        }
    }

    // Waiting and braking are valid proposals, but are deliberately more
    // expensive than any safe progress-producing command.
    rt.proposals.push_back({{0.0f,a.heading_degrees,false},{},2.5f});

    // Reverse is only generated after forward progress is genuinely blocked.
    // Normal travel no longer pays to preview three recovery branches every
    // planner tick.
    if(recovery_search){
        for(float off: {0.0f,-30.0f,30.0f}){
            MotionControl reverse{};
            reverse.target_speed_mps=-std::min(std::max(0.12f,planned_cruise)*0.32f,0.85f);
            reverse.target_heading_degrees=a.heading_degrees+off;
            reverse.reverse=true;
            std::vector<Vec3> preview;
            float end_heading=0.0f,end_speed=0.0f;
            if(!preview_static_clear(a,reverse,world,preview,end_heading,end_speed)) continue;
            float fleet_clearance=std::numeric_limits<float>::infinity();
            if(!preview_fleet_clear(a,preview,agents,now,fleet_clearance)){
                rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
                continue;
            }
            rt.predicted_robot_clearance_m=std::min(rt.predicted_robot_clearance_m,fleet_clearance);
            rt.proposals.push_back({reverse,std::move(preview),4.5f+std::abs(off)*0.01f});
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
    c.occupancy_tube.push_back({now,now+lease,start.position,normal.position,radius});

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

bool passages_available(const MotionContract& c,const ReservationBoard& board,std::size_t& blocker){
    for(auto key:c.passage_keys){
        const auto it=board.passage_owner.find(key);
        if(it!=board.passage_owner.end() && it->second!=c.robot_id){
            blocker=it->second;
            return false;
        }
    }
    return true;
}

bool contract_clear(const MotionContract& c,const SafetyWorldCallbacks& world,const ReservationBoard& board,std::size_t& blocker,BlockReason& reason){
    blocker=kNoSafetyRobot;
    reason=BlockReason::None;
    if(!contract_static_clear(c,world)){
        reason=BlockReason::StaticGeometry;
        return false;
    }
    if(!passages_available(c,board,blocker)){
        reason=BlockReason::PassageReservation;
        return false;
    }
    for(const auto& other:board.contracts){
        if(!other.valid || other.robot_id==c.robot_id) continue;
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

bool follow_certified_brake_branch(const SafetyAgent& a,float now,float tick,Vec3& out_position,float& out_speed){
    const auto& c=a.runtime->active_contract;
    if(!c.valid) return false;

    const float target_time=now+tick;
    std::size_t best_index=std::numeric_limits<std::size_t>::max();
    float best_distance=std::numeric_limits<float>::max();
    for(std::size_t i=0;i<c.brake_branches.size();++i){
        const auto& piece=c.brake_branches[i];
        if(piece.t0>now+1.0e-5f || piece.t1+1.0e-5f<now) continue;
        const Vec3 at_now=interpolate(piece,std::clamp(now,piece.t0,piece.t1));
        const float d=dist_xz(a.position,at_now);
        if(d<best_distance){ best_distance=d; best_index=i; }
    }

    if(best_index!=std::numeric_limits<std::size_t>::max() &&
       best_distance<=c.brake_branches[best_index].radius){
        // Follow the already-certified branch across as many adjacent 10 ms
        // pieces as this simulation substep requires. The old implementation
        // required one single piece to span the whole substep, which fails at
        // a piece boundary and incorrectly turns an ordinary brake into a
        // CONTRACT VIOLATION.
        std::size_t i=best_index;
        Vec3 last=c.brake_branches[i].end;
        float end_time=c.brake_branches[i].t1;
        if(target_time<=end_time+1.0e-5f){
            out_position=interpolate(c.brake_branches[i],std::min(target_time,end_time));
            out_speed=approach(a.speed_mps,0.0f,a.braking_mps2*tick);
            return true;
        }
        while(i+1<c.brake_branches.size()){
            const auto& current=c.brake_branches[i];
            const auto& next=c.brake_branches[i+1];
            const bool contiguous_time=std::abs(next.t0-current.t1)<=2.0e-5f;
            const bool contiguous_space=dist_xz(next.start,current.end)<=std::max(current.radius,next.radius);
            if(!contiguous_time || !contiguous_space) break;
            ++i;
            last=next.end;
            end_time=next.t1;
            if(target_time<=end_time+1.0e-5f){
                out_position=interpolate(next,std::min(target_time,end_time));
                out_speed=approach(a.speed_mps,0.0f,a.braking_mps2*tick);
                return true;
            }
        }
        for(const auto& hold:c.terminal_holds){
            if(target_time+1.0e-5f>=hold.start_time &&
               dist_xz(last,hold.position)<=hold.radius){
                out_position=hold.position;
                out_speed=0.0f;
                return true;
            }
        }
    }

    if(std::abs(a.speed_mps)<0.003f){
        for(const auto& hold:c.terminal_holds){
            if(now+1.0e-5f>=hold.start_time && dist_xz(a.position,hold.position)<=hold.radius){
                out_position=a.position;
                out_speed=0.0f;
                return true;
            }
        }
    }
    return false;
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
    constexpr std::array<float,5> radii{{1.3f,1.8f,2.4f,3.2f,4.2f}};
    for(float r:radii){
        for(float off:offsets){
            const float ang=base+off;
            Vec3 p=a.position+Vec3{std::cos(ang),0.0f,std::sin(ang)}*r;
            if(world.floor_height) p.y=world.floor_height(p);
            if(world.point_clear && !world.point_clear(a.id,p)) continue;
            if(world.segment_clear && !world.segment_clear(a.id,a.position,p)) continue;
            bool fleet_clear=true;
            for(const auto& other:agents){
                if(other.id==a.id) continue;
                if(dist_xz(p,other.position)<=a.radius_m+other.radius_m+0.20f){ fleet_clear=false; break; }
            }
            if(fleet_clear) return p;
        }
    }
    return a.position;
}

bool stationary_for_detour(std::size_t observer_index,std::size_t blocker_index,
                           const std::vector<SafetyAgent>& agents,
                           const std::vector<NavigationRequest>& requests){
    if(observer_index>=agents.size() || blocker_index>=agents.size() || observer_index==blocker_index) return false;
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

bool point_respects_fleet_zone(const SafetyAgent& self,Vec3 p,
                               const std::vector<SafetyAgent>& agents,
                               std::size_t ignored_blocker=kNoSafetyRobot){
    for(const auto& other:agents){
        if(other.id==self.id) continue;
        const float required=self.radius_m+other.radius_m+2.0f*kSafetyNumericMarginM+
                             kNeighborComfortClearanceM;
        const float d=dist_xz(p,other.position);
        if(d>=required) continue;
        if(other.id==ignored_blocker){
            // The selected waypoint itself must be outside the stationary
            // blocker's complete personal zone.  Do not use the legacy escape
            // exception here: a detour waypoint is meant to establish clean
            // separation, not merely maintain the current close spacing.
            return false;
        }
        return false;
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
    for(float side_sign: {preferred_sign,-preferred_sign}){
        for(float add:extra){
            for(float ahead:forward_bias){
                Vec3 candidate=blocker.position+lateral*(side_sign*(zone_radius+add))+travel*ahead;
                if(world.floor_height) candidate.y=world.floor_height(candidate);
                if(world.point_clear && !world.point_clear(a.id,candidate)) continue;
                if(!point_respects_fleet_zone(a,candidate,agents,blocker.id)) continue;

                std::vector<Vec3> route;
                if(!world.coarse_route(a.id,a.position,candidate,std::max(0.10f,stop_radius),route) || route.empty())
                    continue;

                // Prefer short reachable detours that still make progress
                // toward the original task goal.  A tiny penalty for the
                // non-preferred side keeps the decision stable across replans.
                const float score=route_length(a.position,route)+0.42f*dist_xz(candidate,goal)+
                                  (side_sign==preferred_sign?0.0f:0.12f);
                if(score<best_score){
                    best_score=score;
                    best=candidate;
                    found=true;
                }
            }
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
    for(std::size_t j=0;j<agents.size();++j){
        if(agents[j].id==a.id || !stationary_for_detour(a.id,j,agents,requests)) continue;
        Vec3 ap=agents[j].position-a.position; ap.y=0.0f;
        const float t=std::clamp(dot_xz(ap,ab)/len2,0.0f,1.0f);
        if(t<=0.01f || t>=0.995f) continue;
        const Vec3 closest=a.position+ab*t;
        const float required=a.radius_m+agents[j].radius_m+2.0f*kSafetyNumericMarginM+
                             kNeighborComfortClearanceM+0.55f;
        if(dist_xz(closest,agents[j].position)>required) continue;
        if(t<best_t){ best_t=t; best=j; }
    }
    return best;
}

void update_blocking_recovery(std::vector<SafetyAgent>& agents,const std::vector<NavigationRequest>& requests,const SafetyWorldCallbacks& world,float dt){
    // First update progress and blocked timers.
    for(auto& a:agents){
        auto& rt=*a.runtime;
        const float moved=dist_xz(a.position,rt.last_progress_position);
        if(moved>0.08f){
            rt.last_progress_position=a.position;
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
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
        if(rt.blocked_elapsed>=kBlockingWaitTimeoutSeconds && v<agents.size()){
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
        const Vec3 pullout=find_pullout(a,world,agents,rt.blocker_robot_id);
        if(dist_xz(pullout,a.position)>0.20f){
            rt.temporary_goal=pullout;
            rt.temporary_goal_valid=true;
            rt.stationary_detour_active=false;
            rt.stationary_detour_blocker=kNoSafetyRobot;
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
        if(retreat_assigned.count(i)) continue;
        if(rt.blocked_elapsed<kBlockingWaitTimeoutSeconds) continue;
        if(i<requests.size() && requests[i].active){
            if(rt.stationary_detour_active && rt.temporary_goal_valid){
                // Keep one deterministic bypass stable across short pauses.
                // Re-run A* to that SAME waypoint if needed, but do not flip
                // sides or move the temporary target every wait timeout.
                rt.route_replan_required=true;
                rt.proposals.clear();
                rt.blocked_elapsed=0.0f;
                continue;
            }
            std::size_t blocker=rt.blocker_robot_id;
            if(blocker>=agents.size() || !stationary_for_detour(i,blocker,agents,requests))
                blocker=nearest_stationary_blocker_on_goal_line(agents[i],rt,agents,requests);

            if(blocker<agents.size() && stationary_for_detour(i,blocker,agents,requests)){
                Vec3 detour{};
                if(find_stationary_detour(agents[i],blocker,agents,world,rt.goal,
                                          std::max(0.10f,rt.stop_radius),detour)){
                    rt.temporary_goal=detour;
                    rt.temporary_goal_valid=true;
                    rt.stationary_detour_active=true;
                    rt.stationary_detour_blocker=agents[blocker].id;
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
        if(rt.temporary_goal_valid && dist_xz(a.position,rt.temporary_goal)<=std::max(0.08f,rt.stop_radius) && std::abs(a.speed_mps)<0.04f){
            rt.temporary_goal_valid=false;
            rt.stationary_detour_active=false;
            rt.stationary_detour_blocker=kNoSafetyRobot;
            rt.motion_intent=rt.goal_valid?MotionIntent::Navigate:MotionIntent::Wait;
            ++rt.goal_revision;
            invalidate_route(rt);
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
    if(current_clearance>=kNeighborComfortClearanceM)
        return minimum_clearance+kPersonalZoneEscapeEpsilonM>=kNeighborComfortClearanceM;

    // Existing saves may begin inside the personal zone. A new contract may
    // continue only if every normal/braking/terminal branch preserves or opens
    // that separation. This creates a one-way escape from the zone instead of
    // the accept/brake/accept oscillation seen in 0-40-35(1).gmsave.
    return minimum_clearance+kPersonalZoneEscapeEpsilonM>=current_clearance;
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

    // Initialize only from a mutually clear stopped/held state. Existing overlap
    // is an assumption failure, not something the safety layer hides by nudging.
    for(auto& a:agents){
        if(!contract_valid_bounds(a)){
            a.runtime->initialized=true;
            a.runtime->mode=SafetyMode::Fault;
            a.runtime->block_reason=BlockReason::InvalidMotionBounds;
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
                a.runtime->mode=SafetyMode::Fault;
                a.runtime->block_reason=BlockReason::UnsafeInitialGeometry;
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
                agents[i].runtime->mode=SafetyMode::Fault;
                agents[j].runtime->mode=SafetyMode::Fault;
                agents[i].runtime->block_reason=BlockReason::UnsafeInitialGeometry;
                agents[j].runtime->block_reason=BlockReason::UnsafeInitialGeometry;
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
            rt.stationary_detour_active=false;
            rt.stationary_detour_blocker=kNoSafetyRobot;
            rt.blocked_elapsed=0.0f;
            rt.wait_age=0.0f;
            invalidate_route(rt);
        }
        rt.motion_intent=rt.temporary_goal_valid?MotionIntent::Retreat:(rt.goal_valid?MotionIntent::Navigate:MotionIntent::Wait);
        if(req.work_target && !req.active && std::abs(a.speed_mps)<0.03f) rt.mode=SafetyMode::Working;
    }

    // Refresh local fleet awareness for every rover, including idle ones. The
    // planner uses this as a short-horizon dynamic obstacle map; final movement
    // still requires a certified shared reservation contract.
    update_nearby_robot_awareness(agents);

    // Never authorize an unchecked huge step. Process at 10 ms safety cadence;
    // if the caller supplies more than one simulated second, the excess is a
    // stationary hold rather than a long unchecked movement.
    float remaining=std::min(dt,1.0f);
    float now=absolute_time;
    // Full coarse-route searches are the only planner operation that can become
    // expensive in a dense base. Budget them per rendered safety_tick() call,
    // not per 10 ms substep, so six blocked rovers cannot all launch A* before
    // the frame returns to input/camera/render processing.
    std::size_t remaining_route_search_budget=std::min<std::size_t>(2,std::max<std::size_t>(1,eng::cpu_parallel_lanes()));
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

            a.runtime->mode=SafetyMode::Fault;
            a.runtime->block_reason=BlockReason::ContractViolation;
            prior_contract_violation=true;
        }
        if(prior_contract_violation){
            stats.invariant_fault=true;
            for(auto& a:agents) if(a.runtime->mode==SafetyMode::Fault) a.speed_mps=0.0f;
            break;
        }

        std::vector<std::size_t> planner_jobs;
        std::vector<std::size_t> full_route_candidates;
        planner_jobs.reserve(agents.size());
        full_route_candidates.reserve(agents.size());
        for(std::size_t ai=0;ai<agents.size();++ai){
            auto& a=agents[ai];
            auto& rt=*a.runtime;
            rt.planner_elapsed+=tick;
            rt.route_retry_elapsed+=tick;
            if(rt.mode==SafetyMode::Fault) continue;
            if(reached_target(a,rt) && std::abs(a.speed_mps)<0.035f){
                if(rt.temporary_goal_valid){
                    rt.temporary_goal_valid=false;
                    rt.stationary_detour_active=false;
                    rt.stationary_detour_blocker=kNoSafetyRobot;
                    rt.motion_intent=rt.goal_valid?MotionIntent::Navigate:MotionIntent::Wait;
                    ++rt.goal_revision;
                    invalidate_route(rt);
                }else if(rt.goal_valid){
                    rt.motion_intent=MotionIntent::Wait;
                    rt.goal_valid=false;
                    ++rt.goal_revision;
                    invalidate_route(rt);
                }
            }

            // A construction-stage/world revision first performs a cheap check
            // of this rover's existing simplified route. Unaffected routes are
            // retained and simply stamped with the new world epoch.
            refresh_route_world_revision(a,world_epoch,world);

            if(rt.planner_elapsed>=kPlannerReplanSeconds || rt.proposals.empty() || rt.route_replan_required){
                rt.planner_elapsed=0.0f;
                planner_jobs.push_back(ai);
                if(route_search_due(a,world_epoch)) full_route_candidates.push_back(ai);
            }
        }

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

        // Local proposal preview is read-mostly and independent per rover. The
        // expensive full-route work is now bounded above; the shared CPU pool
        // can therefore improve throughput without turning a mass replan into a
        // long frame-time barrier. Arbitration/world mutation remain serialized.
        eng::TaskPool::shared().parallel_for(planner_jobs.size(),[&](std::size_t job){
            const std::size_t ai=planner_jobs[job];
            generate_proposals(agents[ai],world_epoch,world,agents,now,route_search_granted[ai]!=0);
        });

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
            BlockReason last_reason=BlockReason::None;
            std::size_t last_blocker=kNoSafetyRobot;

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
                    const bool certified=follow_certified_brake_branch(a,now,tick,p,next_speed);
                    commands[a.id]={p,a.heading_degrees,next_speed,false,true};
                    if(!certified){
                        rt.mode=SafetyMode::Fault;
                        rt.block_reason=BlockReason::ContractViolation;
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
            actual[i]={now,now+tick,agents[i].position,commands[i].position,agents[i].radius_m+kSafetyNumericMarginM};
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
                agents[i].speed_mps=0.0f;
                agents[i].runtime->mode=SafetyMode::Fault;
                agents[i].runtime->block_reason=BlockReason::ContractViolation;
            }
            break;
        }

        for(std::size_t i=0;i<agents.size();++i){
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

        update_blocking_recovery(agents,requests,world,tick);
        now+=tick;
        remaining-=tick;
    }

    return stats;
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
