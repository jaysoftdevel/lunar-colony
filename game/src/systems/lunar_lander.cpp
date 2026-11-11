#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace game {
namespace {
struct Axis {float p,v,a;};
Axis hermite(float p0,float p1,float v0,float v1,float time,float duration){
    const float u=std::clamp(time/duration,0.0f,1.0f),u2=u*u,u3=u2*u;
    return {(2*u3-3*u2+1)*p0+(u3-2*u2+u)*duration*v0+(-2*u3+3*u2)*p1+(u3-u2)*duration*v1,
        ((6*u2-6*u)*p0+(3*u2-4*u+1)*duration*v0+(-6*u2+6*u)*p1+(3*u2-2*u)*duration*v1)/duration,
        ((12*u-6)*p0+(6*u-4)*duration*v0+(-12*u+6)*p1+(6*u-2)*duration*v1)/(duration*duration)};
}
bool slab(eng::Vec3 a,eng::Vec3 b,float x0,float x1,float z0,float z1){
    float lo=0,hi=1;
    auto axis=[&](float p,float d,float mn,float mx){
        if(std::abs(d)<1e-7f)return p>=mn && p<=mx;
        float t0=(mn-p)/d,t1=(mx-p)/d;if(t0>t1)std::swap(t0,t1);
        lo=std::max(lo,t0);hi=std::min(hi,t1);return lo<=hi;
    };
    return axis(a.x,b.x-a.x,x0,x1)&&axis(a.z,b.z-a.z,z0,z1);
}

}

LanderFlightSample lunar_lander_flight(float seconds){
    // Terminal powered descent, not the full orbit-to-surface transfer. C1
    // altitude/velocity joins, lunar gravity and thrust-vector pitch-over.
    // Reference: NASA SPLICE, https://www.nasa.gov/safe-and-precise-landing-integrated-capabilities-evolution-splice/
    const float t=std::clamp(seconds,0.0f,kLanderDescentSeconds);
    Axis y{},x{};
    if(t<40)y=hermite(220,35,-9,-1.5f,t,40);
    else if(t<56)y=hermite(35,6,-1.5f,-0.25f,t-40,16);
    else if(t<58)y=hermite(6,5.75f,-0.25f,0,t-56,2);
    else if(t<64)y={5.75f,0,0};
    else if(t<79)y=hermite(5.75f,0,0,-0.40f,t-64,15);
    if(t<56)x=hermite(-90,0,4,0,t,56);
    LanderFlightSample s;
    s.offset={x.p,std::max(0.0f,y.p),0};s.velocity={x.v,y.v,0};s.acceleration={x.a,y.a,0};
    if(t<79){
        s.thrust_acceleration=std::hypot(x.a,1.62f+y.a);
        s.roll_degrees=-std::atan2(x.a,1.62f+y.a)*180.0f/3.14159265f;
    }
    s.gear=std::clamp((t-24.0f)/12.0f,0.0f,1.0f);
    return s;
}
const char* lunar_lander_phase_name(const LunarLanderState& l){
    switch(l.phase){
        case LanderPhase::Approach:return l.elapsed<40?"POWERED BRAKING":l.elapsed<56?"FINAL APPROACH":l.elapsed<64?"HOVER / SITE CHECK":"VERTICAL DESCENT";
        case LanderPhase::Settling:return "TOUCHDOWN / ENGINE SAFE";
        case LanderPhase::Landed:return "LANDED / AWAITING UNLOAD";
        case LanderPhase::OpeningRamp:return "DEPLOYING BOARDING RAMP";
        case LanderPhase::DeployingRobots:return "DEPLOYING ROBOTS VIA RAMP";
        case LanderPhase::UnloadingCargo:return "PLACING CARGO BOXES";
        case LanderPhase::Ready:return "DEPLOYMENT COMPLETE";
        default:return "NO LANDER";
    }
}
eng::Vec3 lunar_lander_stash_position(const LunarLanderState& l,MaterialResource material){
    const int i=(int)material;
    return l.site+eng::Vec3{-9.0f-3.8f*(float)(i/4),0,-3.6f+2.4f*(float)(i%4)};
}
eng::Vec3 lunar_lander_robot_bay(const LunarLanderState& l,int ordinal){
    if(l.legacy_parking)
        return l.site+eng::Vec3{12.6f-3.0f*(float)(ordinal%3),0,-5.4f+3.1f*(float)(ordinal/3)};
    constexpr float sides[]={4.0f,-4.0f,6.65f,-6.65f};
    return l.site+eng::Vec3{sides[ordinal%4],0,6.7f+2.15f*(float)(ordinal/4)};
}
namespace {
constexpr float kRoverUnloadSpeed=1.65f;
constexpr float kRoverRampHeadway=2.0f;
constexpr float kApronLaneZ=13.4f;
constexpr float kFrontBerthZ=2.4f;
constexpr float kRampHingeZ=3.6f;

struct RoverUnloadRoute {
    std::array<eng::Vec3,7> points;
    std::array<float,7> distance{};
    float ramp_entry_time;

    RoverUnloadRoute(const LunarLanderState& lander,int ordinal,int first)
        : ramp_entry_time(kRoverRampHeadway*(1+ordinal-first)){
        const auto berth=lunar_lander_robot_berth(ordinal-first);
        const auto bay=lunar_lander_robot_bay(lander,ordinal)-lander.site;
        const float lane=lander.legacy_parking?12.0f:kApronLaneZ;
        points={{berth,{berth.x,2.4f,kFrontBerthZ},{0,2.4f,kRampHingeZ},{0,0,10.6f},
                 {0,0,lane},{bay.x,0,lane},bay}};
        for(std::size_t i=1;i<points.size();++i)
            distance[i]=distance[i-1]+world_math::distance_xz(points[i-1],points[i]);
    }

    float finish() const {
        return ramp_entry_time+(distance.back()-distance[2])/kRoverUnloadSpeed;
    }

    eng::Vec3 sample(float along) const {
        for(std::size_t i=1;i<points.size();++i){
            if(along<distance[i]){
                const float fraction=(along-distance[i-1])/(distance[i]-distance[i-1]);
                return points[i-1]+(points[i]-points[i-1])*fraction;
            }
        }
        return points.back();
    }
};
}

eng::Vec3 lunar_lander_robot_berth(int queue_index){
    constexpr float columns[]={0,1.48f,-1.48f};
    return {columns[queue_index%3],2.4f,kFrontBerthZ-1.92f*(float)(queue_index/3)};
}

float lunar_lander_deployment_seconds(const LunarLanderState& lander,int first){
    float finish=0;
    for(int i=first;i<lander.builders+lander.carriers;++i)
        finish=std::max(finish,RoverUnloadRoute(lander,i,first).finish());
    return finish;
}

LanderRoverPose lunar_lander_robot_pose(const LunarLanderState& lander,int ordinal,int first){
    const RoverUnloadRoute route(lander,ordinal,first);
    const float along=route.distance[2]+(lander.elapsed-route.ramp_entry_time)*kRoverUnloadSpeed;
    if(along<=0) return {lander.site+route.points.front(),0,0,false};
    if(along>=route.distance.back()) return {lander.site+route.points.back(),180,0,true,route.distance.back()};
    // Blend the heading through bends while staying on the reserved lane.
    const auto tangent=route.sample(along+.20f)-route.sample(std::max(0.0f,along-.20f));
    const float heading=std::atan2(tangent.x,tangent.z)*180.0f/3.14159265f;
    const float pitch=std::atan2(-tangent.y,std::hypot(tangent.x,tangent.z))*180.0f/3.14159265f;
    return {lander.site+route.sample(along),heading,pitch,false,along};
}

bool lunar_lander_deployment_clear(const LunarLanderState& lander,eng::Vec3 position,float rotation,
                                  float width,float depth,float margin){
    using namespace world_math;
    const auto area=footprint(position,rotation,width,depth);
    if(!lander.legacy_parking)
        return !overlaps(area,footprint(lander.site+Vec3{0,0,2.6f},0,12.2f,17.4f),margin)
            && !overlaps(area,footprint(lander.site+Vec3{0,0,10.2f},0,15.8f,9.4f),margin);
    // Keep the original apron for saved landers that were already unloaded.
    return !overlaps(area,footprint(lander.site+Vec3{0,0,2.6f},0,12.2f,17.4f),margin)
        && !overlaps(area,footprint(lander.site+Vec3{9.6f,0,3.15f},0,8.8f,19.7f),margin)
        && !overlaps(area,footprint(lander.site+Vec3{6.5f,0,11.85f},0,15.0f,2.3f),margin);
}

bool lunar_lander_segment_blocked(const LunarLanderState& l,eng::Vec3 a,eng::Vec3 b,float radius){
    if(!l.active())return false;
    a=a-l.site;b=b-l.site;
    return slab(a,b,-5.35f-radius,5.35f+radius,-5.35f-radius,5.35f+radius)||
        slab(a,b,-2.12f-radius,2.12f+radius,3.6f-radius,10.7f+radius);
}

eng::Vec3 lunar_lander_salvage_station(const LunarLanderState& lander,std::size_t port){
    constexpr eng::Vec3 stations[]={{6.5f,0,0},{-6.5f,0,0},{0,0,-6.5f}};
    return lander.site+stations[port];
}

eng::Vec3 lunar_lander_salvage_focus(const LunarLanderState& lander,std::size_t port){
    const auto offset=lunar_lander_salvage_station(lander,port)-lander.site;
    return lander.site+offset*.83f+eng::Vec3{0,.7f,0};
}

bool GameApp::lander_placement_clear(eng::Vec3 p,float rotation,float width,float depth) const {
    if(lander_.has_depot()&&lander_.boxed){
        const auto area=world_math::footprint(p,rotation,width,depth);
        const float marker_width=kStashMarkerRadius*2;
        if(world_math::overlaps(area,world_math::footprint(lander_.stash_center,0,marker_width,marker_width),.4f)) return false;
        for(const auto& pile:ground_resource_piles_){
            if(pile.kind!=GroundPileKind::Stash) continue;
            const float diameter=pile.radius*2;
            if(world_math::overlaps(area,world_math::footprint(pile.position,0,diameter,diameter),1.1f)) return false;
        }
    }
    if(!lander_.active())return true;
    using namespace world_math;
    const auto candidate=footprint(p,rotation,width,depth);
    if(overlaps(candidate,footprint(lander_.site+eng::Vec3{0,0,2.6f},0,12.2f,17.4f),0.5f))return false;
    if(lander_.deploying()&&!lunar_lander_deployment_clear(lander_,p,rotation,width,depth,.5f)) return false;
    return true;
}

BuilderRobotState GameApp::lander_robot(int ordinal) const {
    BuilderRobotState r;
    r.role=ordinal<(int)lander_.builders?RobotRole::Constructor:RobotRole::Carrier;
    r.position=lunar_lander_robot_bay(lander_,ordinal);r.heading_degrees=180;
    r.visual_position=r.position;r.visual_heading_degrees=r.heading_degrees;r.visual_initialized=true;
    const bool carrier=r.role==RobotRole::Carrier;
    r.move_speed_mps=carrier?kCarrierRobotTransitSpeedMps:kBuilderRobotTransitSpeedMps;
    r.payload_capacity_kg=carrier?3600:2400;
    r.battery_capacity_kwh=carrier?kCarrierRobotBatteryCapacityKwh:kBuilderRobotBatteryCapacityKwh;
    r.battery_charge_kwh=r.battery_capacity_kwh;
    return r;
}
void GameApp::begin_lunar_landing(const ResourceBundle& manifest,int builders,int carriers,CropSeeds seeds){
    lander_={};lander_.phase=LanderPhase::Approach;lander_.awaiting_site=true;
    lander_.manifest=manifest;lander_.remaining=manifest;
    lander_.seeds_aboard=seeds;
    colony_seeds_={};
    lander_.builders=(std::uint8_t)std::clamp(builders,1,6);lander_.carriers=(std::uint8_t)std::clamp(carriers,1,6);
    starting_robot_count_=lander_.builders;starting_carrier_count_=lander_.carriers;
    resources_.reset();ground_resource_piles_.clear();builder_robots_.clear();
    starter_stash_center_={};
    robot_safety_time_seconds_=0;nav_wall_cache_.reset();
    selection_kind_=SelectionKind::Lander;
    camera_.reset(playfield_.size());
    landing_message_="CLICK A CLEAR LANDING SITE";
}
bool GameApp::unload_lunar_lander(){
    if(lander_.phase!=LanderPhase::Landed)return false;
    if(!drop_lander_cargo())return false;
    lander_.phase=LanderPhase::OpeningRamp;lander_.elapsed=0;
    return true;
}
void GameApp::update_lunar_lander(float dt){
    if(!lander_.active() || lander_.phase==LanderPhase::Ready||lander_.awaiting_site||lander_.awaiting_stash)return;
    if(lander_.phase==LanderPhase::UnloadingCargo){if(drop_lander_cargo()){lander_.phase=LanderPhase::Ready;lander_.elapsed=0;}return;}
    float remaining=std::max(0.0f,dt);
    // Consume phase remainders so low FPS / 4x / save-resume never lose or
    // duplicate a robot. Cargo is credited atomically by explicit UNLOAD.
    for(int guard=0;guard<2048;++guard){
        auto phase=lander_.phase;
        float duration=0;
        switch(phase){
            case LanderPhase::Approach:duration=kLanderDescentSeconds;break;
            case LanderPhase::Settling:duration=kLanderSettleSeconds;break;
            case LanderPhase::OpeningRamp:duration=kLanderRampSeconds;break;
            case LanderPhase::DeployingRobots:duration=lunar_lander_deployment_seconds(lander_,(int)builder_robots_.size());break;
            default:return;
        }
        const float playback=phase==LanderPhase::Approach || phase==LanderPhase::Settling?
            kLanderApproachPlaybackRate:1.f;
        const float step=std::min(remaining*playback,std::max(0.0f,duration-lander_.elapsed));
        lander_.elapsed+=step;
        remaining=std::max(0.f,remaining-step/playback);
        if(phase==LanderPhase::Approach){
            const auto before=lunar_lander_flight(lander_.elapsed-step);
            const auto after=lunar_lander_flight(lander_.elapsed);
            const auto focus=[](eng::Vec3 offset){return eng::Vec3{offset.x*.5f,offset.y*.45f,0};};
            const auto distance=[](eng::Vec3 offset){return 46.0f+offset.y*1.25f;};
            if(lander_.elapsed==step&&step>0){
                camera_.restore({distance(after.offset),36,38,lander_.site+focus(after.offset)});
            }else if(step>0){
                auto view=camera_.state();
                view.target+=focus(after.offset)-focus(before.offset);
                view.distance=std::max(8.0f,view.distance*distance(after.offset)/distance(before.offset));
                camera_.restore(view);
            }
        }
        if(lander_.elapsed+1e-5f<duration)return;
        lander_.elapsed=0;
        if(phase==LanderPhase::Approach){lander_.phase=LanderPhase::Settling;selection_kind_=SelectionKind::Lander;}
        else if(phase==LanderPhase::Settling){
            lander_.phase=LanderPhase::Landed;
            lander_.awaiting_stash=!lander_.boxed;
        }
        else if(phase==LanderPhase::OpeningRamp)lander_.phase=LanderPhase::DeployingRobots;
        else if(phase==LanderPhase::DeployingRobots){
            const int total=lander_.builders+lander_.carriers;
            const int first=static_cast<int>(builder_robots_.size());
            for(int i=first;i<total;++i){
                auto rover=lander_robot(i);
                robots::roll_rover_wheels(rover.wheels,RoverUnloadRoute(lander_,i,first).distance.back(),0);
                builder_robots_.push_back(std::move(rover));
            }
            lander_.phase=LanderPhase::Ready;
        }
        if(lander_.phase==LanderPhase::Ready)return;
        if(remaining<=0)return;
    }
}

void GameApp::render_lunar_lander_panel(eng::Renderer& r){
    using namespace eng;using namespace layout;
    sync_selection_panel_scroll();const auto p=selection_panel_rect();layout::hud_panel(r,p,theme_);
    r.draw_line({p.x,p.y},{p.x+p.w,p.y},theme_.accent,2);
    const auto label=[&](float y,const std::string& text,float scale=0.47f){
        y+=p.y-selection_panel_scroll_;
        if(y>=p.y+4 && y+14<=p.y+p.h-4)game_label_fitted(r,{p.x+14,y,p.w-28,14},text,scale,theme_);
    };
    label(12,"LUNAR CARGO LANDER",0.66f);
    label(40,lander_.awaiting_site?"CLICK LANDING SITE":lander_.awaiting_stash?"CLICK STASH MARKER":lunar_lander_phase_name(lander_),0.48f);
    const auto s=lunar_lander_flight(lander_.in_flight()?lander_.elapsed:kLanderDescentSeconds);
    std::ostringstream telemetry;telemetry<<std::fixed<<std::setprecision(1)<<"ALT "<<s.offset.y<<" M   VERTICAL "<<s.velocity.y<<" M/S";
    label(66,telemetry.str());
    std::ostringstream lateral;lateral<<std::fixed<<std::setprecision(1)<<"LATERAL "<<std::abs(s.velocity.x)<<" M/S  |  "<<(lander_.in_flight()?"ENGINE FIRING":"ENGINE OFF");label(87,lateral.str());
    label(110,"BUILDERS "+std::to_string(lander_.builders)+"  /  CARRIERS "+std::to_string(lander_.carriers));
    const int total=lander_.builders+lander_.carriers;
    int parked=(int)builder_robots_.size();
    if(lander_.phase==LanderPhase::DeployingRobots){
        for(int i=parked;i<total;++i)
            if(lunar_lander_robot_pose(lander_,i,(int)builder_robots_.size()).parked) ++parked;
    }
    label(131,"PARKED "+std::to_string(parked)+" / "+std::to_string(total)+" ROVERS");
    label(152,"CARGO ABOARD "+std::to_string(lander_.remaining.total_amount())+" / "+std::to_string(lander_.manifest.total_amount())+" KG");
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto m=(MaterialResource)i;label(170+(float)i*14,std::string(material_resource_name(m))+"  "+std::to_string(lander_.remaining.amount(m))+" KG ABOARD",0.41f);
    }
    label(285,"SEED PACKETS ABOARD "+std::to_string(seed_packet_total(lander_.seeds_aboard)),.43f);
    Rect button{p.x+14,p.y+307-selection_panel_scroll_,p.w-28,28};
    if(button.y>=p.y+4 && button.y+button.h<=p.y+p.h-4){
        const char* caption=lander_.phase<LanderPhase::Landed?"WAIT FOR LANDING":
            lander_.awaiting_stash?"PLACE STASH MARKER":lander_.phase==LanderPhase::Landed?"UNLOAD":
            lander_.phase==LanderPhase::Ready?"UNLOADED":"UNLOADING...";
        if(ui::button(r,ui_input_,button,caption,theme_,lander_.phase==LanderPhase::Landed && lander_.boxed))
            unload_lunar_lander();
    }
    std::string status=lander_.phase<LanderPhase::Landed?"CHOOSE THE STASH AFTER TOUCHDOWN":
        lander_.awaiting_stash?"CLICK CLEAR GROUND BESIDE THE LANDER":"CARGO STACKS AROUND THE STASH MARKER";
    if(!landing_message_.empty()) status=landing_message_;
    if(lander_.salvage_requested&&lander_.salvage_worker>=0){
        const int percent=(int)(100*lander_.salvage_seconds/kLanderSalvageSeconds);
        status=percent>0?"BUILDER DISMANTLING LANDER / "+std::to_string(percent)+"%":"BUILDER HEADING TO LANDER";
    }
    label(341,status,0.38f);
    eng::Rect stash_button{p.x+14,p.y+362-selection_panel_scroll_,p.w-28,28};
    if(stash_button.y>=p.y+4&&stash_button.y+28<=p.y+p.h-4&&eng::ui::button(r,ui_input_,stash_button,"SELECT STASH",theme_,lander_.boxed && lander_.phase>=LanderPhase::Landed))selection_kind_=SelectionKind::Stash;
    eng::Rect dismantle{p.x+14,p.y+396-selection_panel_scroll_,p.w-28,29};
    const bool ready_to_store=lander_.salvage_seconds>=kLanderSalvageSeconds;
    const char* salvage_caption=ready_to_store?"STORE RECOVERED PARTS":
        lander_.salvage_requested?"SALVAGE IN PROGRESS":"DECONSTRUCT / SALVAGE";
    const bool can_salvage=lander_.phase==LanderPhase::Ready&&(!lander_.salvage_requested||ready_to_store);
    if(dismantle.y>=p.y+4&&dismantle.y+29<=p.y+p.h-4&&
       eng::ui::button(r,ui_input_,dismantle,salvage_caption,theme_,can_salvage)) salvage_lander();
    const float maximum=selection_panel_max_scroll();
    if(maximum>0){const float h=std::max(20.0f,(p.h-12)*p.h/470.0f);r.draw_rect({p.x+p.w-6,p.y+6+(p.h-12-h)*selection_panel_scroll_/maximum,3,h},theme_.accent);}
}
} // namespace game
