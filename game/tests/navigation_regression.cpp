#include "game/world/airlock_connections.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "engine/geometry_stream.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/robots/safety_coordinator.hpp"
#include "game/util/world_math.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

using eng::Vec3;
using namespace game::robots;
namespace {
void require(bool ok,const std::string& what){ if(!ok) throw std::runtime_error(what); }
float distance(Vec3 a,Vec3 b){ a.y=b.y=0; return eng::length(a-b); }
struct Box { float x0,x1,z0,z1; };
bool segment_hits_mesh(Vec3 from,Vec3 to,const std::vector<Vec3>& mesh){
    const Vec3 direction=to-from;
    for(std::size_t i=0;i+2<mesh.size();i+=3){
        const Vec3 a=mesh[i],e1=mesh[i+1]-a,e2=mesh[i+2]-a;
        const Vec3 p=eng::cross(direction,e2);const float det=eng::dot(e1,p);
        if(std::abs(det)<1.0e-7f) continue;
        const Vec3 q0=from-a;const float u=eng::dot(q0,p)/det;
        if(u<0 || u>1) continue;
        const Vec3 q=eng::cross(q0,e1);const float v=eng::dot(direction,q)/det;
        if(v<0 || u+v>1) continue;
        const float t=eng::dot(e2,q)/det;
        if(t>0.0001f && t<0.9999f) return true;
    }
    return false;
}

struct GeometryRecorder final:eng::Renderer {
    std::vector<Vec3> triangles;
    bool initialize(void*,std::uintptr_t,int,int) override{return true;}
    void resize(int,int) override{} void begin_frame(eng::Color) override{} void end_frame() override{}
    void set_3d(const eng::Mat4&) override{} void set_2d(float) override{}
    std::size_t wire_vertices{0},filled_vertices{0};
    bool blue{false},yellow{false},green{false};
    void draw_lines_3d(const std::vector<Vec3>& vertices,eng::Color color,float) override{
        blue=blue || (color.b>0.8f && color.r<0.3f);
        yellow=yellow || (color.r>0.9f && color.g>0.45f && color.b<0.4f);
        green=green || (color.g>0.8f && color.r<0.4f && color.b>0.2f);
        wire_vertices+=vertices.size();
        for(std::size_t i=0;i+5<vertices.size();i+=6)
            if(eng::length(vertices[i+1]-vertices[i+2])<0.00001f && eng::length(vertices[i+3]-vertices[i+4])<0.00001f && eng::length(vertices[i+5]-vertices[i])<0.00001f)
                triangles.insert(triangles.end(),{vertices[i],vertices[i+1],vertices[i+3]});
    }
    void draw_triangles_3d(const std::vector<Vec3>& vertices,eng::Color) override{filled_vertices+=vertices.size();triangles.insert(triangles.end(),vertices.begin(),vertices.end());}
    void draw_rect(eng::Rect,eng::Color) override{} void draw_line(eng::Vec2,eng::Vec2,eng::Color,float) override{}
    void draw_text(eng::Vec2,const std::string&,float,eng::Color) override{}
    int width() const override{return 1600;} int height() const override{return 900;}
    bool hardware_accelerated() const override{return false;}
    std::string gpu_vendor() const override{return {};}
    std::string gpu_renderer() const override{return {};}
    std::string gpu_version() const override{return {};}
};
void fleet_case(const std::string& name,std::vector<Vec3> starts,std::vector<Vec3> goals,
                std::vector<Box> obstacles,float timeout=75.0f,bool dynamic_obstacle=false,bool parked_goal=false){
    std::vector<SafetyRuntime> runtime(starts.size());
    std::vector<SafetyAgent> agents(starts.size());
    std::vector<NavigationRequest> requests(starts.size());
    SafetyWorldCallbacks world;
    world.floor_height=[](Vec3){return 0.0f;};
    world.point_clear=[&](std::size_t id,Vec3 p){
        const float r=agents[id].radius_m+0.04f;
        if(std::abs(p.x)>40-r || std::abs(p.z)>40-r) return false;
        for(const auto& b:obstacles)
            if(p.x>b.x0-r && p.x<b.x1+r && p.z>b.z0-r && p.z<b.z1+r) return false;
        return true;
    };
    world.segment_clear=[&](std::size_t id,Vec3 a,Vec3 b){
        const int n=std::max(1,(int)std::ceil(distance(a,b)/0.04f));
        for(int i=0;i<=n;++i) if(!world.point_clear(id,a+(b-a)*((float)i/n))) return false;
        return true;
    };
    auto callbacks=[&](std::size_t id,bool fleet){
        PathPlannerCallbacks cb;
        cb.floor_height=world.floor_height;
        cb.point_clear=[&,id,fleet](Vec3 p){
            if(!world.point_clear(id,p)) return false;
            if(fleet) for(const auto& other:agents){
                if(other.id==id) continue;
                const float r=agents[id].radius_m+other.radius_m+0.70f;
                if(distance(p,other.position)<r && distance(p,other.position)+0.001f<distance(agents[id].position,other.position)) return false;
            }
            return true;
        };
        cb.segment_clear=[&,id,fleet](Vec3 a,Vec3 b){
            if(!world.segment_clear(id,a,b)) return false;
            if(fleet) for(const auto& other:agents){
                if(other.id==id) continue;
                const Vec3 d=b-a;
                const float l=d.x*d.x+d.z*d.z;
                const float t=l>0.0001f?std::clamp(eng::dot(other.position-a,d)/l,0.0f,1.0f):0.0f;
                const float closest=distance(a+d*t,other.position);
                if(closest<agents[id].radius_m+other.radius_m+0.70f && closest+0.001f<distance(agents[id].position,other.position)) return false;
            }
            return true;
        };
        cb.traversal_cost=[&,id](Vec3 p){
            for(const auto& b:obstacles){
                const float r=agents[id].radius_m+1.0f;
                if(p.x>b.x0-r && p.x<b.x1+r && p.z>b.z0-r && p.z<b.z1+r) return 5.0f;
            }
            return 0.0f;
        };
        return cb;
    };
    world.coarse_route=[&](std::size_t id,Vec3 a,Vec3 b,float stop,std::vector<Vec3>& out){
        PathPlanRequest req; req.start=a;req.goal=b;req.stop_radius=stop;
        req.cell_size=0.65f;req.corridor_half_width=25;req.max_expansions=10000;
        return plan_corridor_path(req,callbacks(id,true),out);
    };
    world.incremental_route=[&](std::size_t id,Vec3 a,Vec3 b,float stop,std::uint64_t epoch,IncrementalPathPlannerState& state,std::vector<Vec3>& out){
        PathPlanRequest req;req.start=a;req.goal=b;req.stop_radius=stop;
        req.cell_size=0.75f;req.corridor_half_width=25;req.max_expansions=720;
        return plan_incremental_path(req,epoch,callbacks(id,false),state,out);
    };
    world.stationary_segment_clear=[&](std::size_t id,Vec3 a,Vec3 b){
        for(const auto& other:agents){
            if(other.id==id || requests[other.id].active) continue;
            const Vec3 d=b-a;const float len2=eng::dot(d,d);
            const float t=len2>0.001f?std::clamp(eng::dot(other.position-a,d)/len2,0.0f,1.0f):0.0f;
            if(distance(a+d*t,other.position)<agents[id].radius_m+other.radius_m+0.06f) return false;
        }
        return true;
    };
    for(std::size_t i=0;i<agents.size();++i){
        agents[i]={i,starts[i],0,0,0.99f,6.0f,4.8f,6.4f,250.0f,&runtime[i]};
        requests[i]={true,goals[i],0.18f,1.0f,false,1};
    }
    if(parked_goal) requests.back().active=false;
    const auto begin=std::chrono::steady_clock::now();
    int faults=0;float t=0;std::uint64_t epoch=1;
    for(;t<timeout;t+=0.01f){
        if(dynamic_obstacle && epoch==1 && t>1.4f){obstacles.push_back({-2,3,-3,3});++epoch;}
        const auto previous=agents;
        const auto stats=safety_tick(0.01f,t,epoch,agents,requests,world);
        faults+=stats.invariant_fault;
        bool done=true;
        for(std::size_t i=0;i<agents.size();++i){
            require(world.segment_clear(i,previous[i].position,agents[i].position),name+": static collision");
            require(distance(previous[i].position,agents[i].position)<=agents[i].max_speed_mps*0.01f+0.002f,name+": pose jump");
            require(std::abs(agents[i].speed_mps-previous[i].speed_mps)<=agents[i].braking_mps2*0.01f+0.002f,name+": speed pulse");
            require(runtime[i].mode!=SafetyMode::Fault,name+": safety fault "+block_reason_name(runtime[i].block_reason));
            if(name=="open straight" && t>1.8f && t<3.5f)
                require(agents[i].speed_mps>5.9f,"open travel contains stop/start pulses");
            if(distance(agents[i].position,goals[i])<0.22f && std::abs(agents[i].speed_mps)<0.06f) requests[i].active=false;
            done=done && !requests[i].active;
            for(std::size_t j=i+1;j<agents.size();++j){
                TubePiece a{0,0.01f,previous[i].position,agents[i].position,agents[i].radius_m};
                TubePiece b{0,0.01f,previous[j].position,agents[j].position,agents[j].radius_m};
                require(!moving_discs_overlap(a,b,0,0.01f),name+": fleet collision");
            }
        }
        if(done) break;
    }
    for(std::size_t i=0;i<agents.size();++i){
        if(requests[i].active) std::cerr<<name<<" robot "<<i<<" at "<<agents[i].position.x<<","<<agents[i].position.z<<" distance "<<distance(agents[i].position,goals[i])<<" mode "<<safety_mode_name(runtime[i].mode)<<" reason "<<block_reason_name(runtime[i].block_reason)<<" replans "<<runtime[i].local_replans<<"\n";

    }
    for(const auto& request:requests) require(!request.active,name+": did not finish");
    require(faults==0,name+": invariant fault");
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<name<<": completed in "<<t<<" s, "<<ms<<" ms CPU/wall\n";
}
void fault_recovery_case(const std::string& name,std::size_t count,bool unsafe_origin=false,bool idle=false,bool pending=false){
    std::vector<SafetyRuntime> runtime(count);
    std::vector<SafetyAgent> agents(count);
    std::vector<NavigationRequest> requests(count);
    std::vector<unsigned> calls(count,0),fresh(count,0);
    std::vector<Vec3> planned_start(count),planned_goal(count);
    bool blocked=unsafe_origin;
    SafetyWorldCallbacks world;
    world.floor_height=[](Vec3){return 0.0f;};
    world.point_clear=[&](std::size_t,Vec3){return !blocked;};
    world.segment_clear=[&](std::size_t,Vec3,Vec3){return !blocked;};
    world.incremental_route=[&](std::size_t id,Vec3 start,Vec3 goal,float,std::uint64_t,IncrementalPathPlannerState& state,std::vector<Vec3>& out){
        ++calls[id];planned_start[id]=start;planned_goal[id]=goal;
        if(!state.initialized && state.g.empty() && state.rhs.empty() && state.open_heap.empty())++fresh[id];
        state.initialized=true;state.g={7};state.rhs={7};
        if(blocked)return IncrementalPathResult::NoPath;
        if(pending && calls[id]<6)return IncrementalPathResult::Pending;
        out={goal};return IncrementalPathResult::Success;
    };
    for(std::size_t i=0;i<count;++i){
        agents[i]={i,{-8,0,(float)i*8},90,0,0.99f,6,4.8f,6.4f,250,&runtime[i]};
        requests[i]={!idle,{8,0,(float)i*8},0.18f,1,idle,1};
        auto& rt=runtime[i];rt.mode=SafetyMode::Fault;rt.initialized=true;rt.fault_phase=1;
        rt.block_reason=BlockReason::ContractViolation;rt.goal_valid=!idle;rt.goal=requests[i].goal;
        rt.stop_radius=requests[i].stop_radius;rt.cached_route={{-40,0,-40}};
        rt.incremental_path.initialized=true;rt.incremental_path.g={999};rt.incremental_path.rhs={999};
        rt.temporary_goal_valid=true;rt.temporary_goal={-40,0,-40};rt.stationary_detour_active=true;
        rt.blocked_elapsed=100;rt.wait_age=100;rt.blocker_robot_id=i;
    }
    const auto starts=agents;
    unsigned first_attempt_frame=0;const int frames=unsafe_origin?1000:600;
    for(int frame=0;frame<frames;++frame){
        const float now=frame*0.01f;
        if(unsafe_origin && frame==610)blocked=false;
        if(frame==150 && !idle)for(auto& req:requests)req.goal.x=10; // use the current scheduled destination
        const auto previous=agents;
        safety_tick(0.01f,now,1,agents,requests,world);
        for(std::size_t i=0;i<count;++i){
            const auto& rt=runtime[i];
            if(rt.fault_recovery_attempts && first_attempt_frame==0)first_attempt_frame=(unsigned)frame;
            if(frame<298){require(rt.mode==SafetyMode::Fault,name+": fault was cleared before its scheduled retry");require(calls[i]==0,name+": expensive search repeated during fault wait");}
            if(rt.mode==SafetyMode::Fault){
                require(distance(agents[i].position,starts[i].position)<1e-6f,name+": fault hold moved");
                require(agents[i].speed_mps==0,name+": fault retained motion");
            }
            if(blocked){require(rt.mode==SafetyMode::Fault,name+": unsafe origin was admitted");require(distance(agents[i].position,starts[i].position)<1e-6f,name+": positional repair bypassed collision checks");}
            else require(world.segment_clear(i,previous[i].position,agents[i].position),name+": static collision");
            require(distance(previous[i].position,agents[i].position)<=agents[i].max_speed_mps*0.01f+0.002f,name+": pose jump");
            if(calls[i])require(distance(planned_goal[i],requests[i].goal)<0.001f,name+": recovery used stale/temporary destination");
            for(std::size_t j=i+1;j<count;++j){
                TubePiece a{0,0.01f,previous[i].position,agents[i].position,agents[i].radius_m};
                TubePiece b{0,0.01f,previous[j].position,agents[j].position,agents[j].radius_m};
                require(!moving_discs_overlap(a,b,0,0.01f),name+": recovery fleet collision");
            }
        }
    }
    require(first_attempt_frame>0 && first_attempt_frame<=300,name+": recovery waited longer than 3 seconds");
    for(std::size_t i=0;i<count;++i){
        const auto& rt=runtime[i];require(rt.mode!=SafetyMode::Fault,name+": never recovered");
        require(!rt.temporary_goal_valid && !rt.stationary_detour_active,name+": stale detour survived recovery");
        if(idle){require(calls[i]==0,name+": invented an idle destination");require(distance(agents[i].position,starts[i].position)<1e-6f,name+": idle robot moved");}
        else {
            require(calls[i]>0 && fresh[i]>0,name+": search state was reused instead of starting from scratch");
            require(distance(planned_start[i],starts[i].position)<0.001f,name+": search started from stale route origin");
            require(distance(agents[i].position,starts[i].position)>0.5f,name+": no certified progress after recovery");
        }
        if(unsafe_origin)require(rt.fault_recovery_attempts==3 && fresh[i]==3,name+": persistent geometry reset/starved retry deadline");
        if(pending)require(calls[i]>=6 && fresh[i]==1,name+": bounded search was discarded instead of continued");
    }
    std::cout<<name<<": fresh recovery within 3 seconds, safe hold and certified continuation passed\n";
}

void fault_bounds_case(){
    SafetyRuntime rt;std::vector<SafetyAgent> agents{{0,{-4,0,0},90,0,0.99f,6,0,6.4f,250,&rt}};
    std::vector<NavigationRequest> requests{{true,{4,0,0},0.18f,1,false,1}};
    SafetyWorldCallbacks world;world.point_clear=[](std::size_t,Vec3){return true;};
    world.segment_clear=[](std::size_t,Vec3,Vec3){return true;};unsigned calls=0;
    world.coarse_route=[&](std::size_t,Vec3,Vec3 goal,float,std::vector<Vec3>& out){++calls;out={goal};return true;};
    safety_tick(0.01f,0,1,agents,requests,world);
    require(rt.mode==SafetyMode::Fault && rt.block_reason==BlockReason::InvalidMotionBounds,"invalid bounds not faulted");
    for(int frame=1;frame<310;++frame)safety_tick(0.01f,frame*0.01f,1,agents,requests,world);
    require(rt.mode==SafetyMode::Fault && calls==0 && rt.fault_recovery_attempts==1,"invalid bounds recovery admitted unsafe motion or postponed retry");
    agents[0].acceleration_mps2=4.8f;
    for(int frame=310;frame<650;++frame)safety_tick(0.01f,frame*0.01f,1,agents,requests,world);
    require(calls>0 && rt.mode!=SafetyMode::Fault && agents[0].position.x>-3.9f,"fixed bounds did not recover on next three-second retry");
    std::cout<<"Invalid motion bounds remain stopped, then recover when corrected\n";
}

}

namespace game {
struct Recorder:eng::Renderer {
    int w=1920,h=1080;float scale=1;bool hud=false;std::size_t vertices=0,calls=0;
    bool initialize(void*,std::uintptr_t,int,int)override{return true;}
    void resize(int width,int height)override{w=width;h=height;}
    void begin_frame(eng::Color)override{vertices=0;calls=0;hud=false;}void end_frame()override{}
    void set_3d(const eng::Mat4&)override{hud=false;}void set_2d(float s)override{scale=s;hud=true;}
    void draw_lines_3d(const std::vector<eng::Vec3>& v,eng::Color,float)override{vertices+=v.size();++calls;}
    void draw_triangles_3d(const std::vector<eng::Vec3>& v,eng::Color)override{vertices+=v.size();++calls;}
    void draw_rect(eng::Rect,eng::Color)override{} void draw_line(eng::Vec2,eng::Vec2,eng::Color,float)override{}
    void draw_text(eng::Vec2,const std::string&,float,eng::Color)override{}
    int width()const override{return w;}int height()const override{return h;}bool hardware_accelerated()const override{return false;}
    std::string gpu_vendor()const override{return "TEST";}std::string gpu_renderer()const override{return "SCALING RECORDER";}std::string gpu_version()const override{return "TEST";}
};
struct NavigationRegression {
    static void profile(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;require(app.load_game_from_file(path,error),error);
        Recorder r;app.screen_=GameApp::Screen::Playfield;
        std::cout<<"SAVE h="<<app.placed_habitats_.size()<<" r="<<app.placed_resources_.size()<<" c="<<app.corridor_network_.connections().size()<<" u="<<app.utility_network_.connections().size()<<" robots="<<app.builder_robots_.size()<<" lanes="<<eng::cpu_parallel_lanes()<<"\n";
        auto bench=[&](const char* name,auto fn,int n){auto start=std::chrono::steady_clock::now();for(int i=0;i<n;++i)fn();std::cout<<name<<" ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/n<<std::endl;};
        bench("supply",[&]{app.update_supply_simulation(1.f/60);},120);
        bench("doors",[&]{app.update_pressure_doors(1.f/60);},120);
        bench("construction",[&]{app.update_construction(1.f/60);},120);
        for(const auto& bot:app.builder_robots_)std::cout<<"ROBOT mode="<<(int)bot.mode<<" action="<<(int)bot.action<<" target="<<(int)bot.target.kind<<" route="<<bot.route_active<<" speed="<<bot.current_speed_mps<<" replans="<<bot.safety.local_replans<<"\n";
        bench("render",[&]{r.begin_frame({});app.render(r);},120);std::cout<<"vertices="<<r.vertices<<" calls="<<r.calls<<"\n";
        bench("render corridors",[&]{app.render_corridors(r);},120);
        bench("render habitats",[&]{app.render_habitats(r);},120);
        bench("render resources",[&]{app.render_resources(r);},120);
        for(const auto& h:app.placed_habitats_)std::cout<<"air h kind="<<(int)h.kind<<" air="<<h.supply.breathability()<<" power="<<h.supply.power_fraction<<" component water="<<h.supply.network_water_to_fill_litres<<"\n";
    }

    static void operation_relocation(){
        eng::Application engine;
        GameApp stuck(engine);
        stuck.playfield_.rebuild(64);
        stuck.robot_enclosure_legacy_scan_pending_=false;
        PlacedResource blocker;
        blocker.kind=ResourceKind::PowerStorage;
        blocker.construction.step=kConstructionSteps;
        blocker.placement_sequence=41;
        stuck.placed_resources_.push_back(blocker);
        blocker.position={22,0,22};
        blocker.placement_sequence=42;
        stuck.placed_resources_.push_back(blocker);
        BuilderRobotState rover;
        rover.role=RobotRole::Carrier;
        rover.cargo.iron=200;
        rover.battery_charge_kwh=kCarrierRobotBatteryCapacityKwh;
        stuck.builder_robots_.push_back(rover);
        auto& trapped=stuck.builder_robots_[0];
        for(int frame=0;frame<16*90&&trapped.operation_retry.failures<4;++frame)
            stuck.update_builder_robot(1.f/90);
        require(trapped.operation_retry.failures==4&&distance(trapped.position,{})<.01f,
                "rover relocated before four blocked retries or failed to count them");
        require(trapped.blocking_structure==41&&trapped.blocker_display_seconds>0,
                "blocked rover did not identify the actual battery footprint");
        struct BlockerCapture:Recorder {
            bool red=false,unrelated=false;
            void draw_lines_3d(const std::vector<eng::Vec3>& vertices,eng::Color color,float) override {
                red|=!vertices.empty()&&color.r>.95f&&color.g<.15f&&color.b<.15f;
                for(const auto point:vertices) unrelated|=point.x>15||point.z>15;
            }
        } capture;
        stuck.render_robot_blockers(capture);
        require(capture.red&&!capture.unrelated,"wrong structure received the red outline");
        for(int frame=0;frame<5*90&&distance(trapped.position,{})<.01f;++frame)
            stuck.update_builder_robot(1.f/90);
        require(distance(trapped.position,{})>=2&&trapped.operation_retry.failures==0&&trapped.cargo.iron==200,
                "fifth blocked attempt did not extract the rover with its cargo intact");

        GameApp delivery(engine);
        delivery.playfield_.rebuild(64);
        PlacedResource job;
        job.kind=ResourceKind::SolarPanel;
        job.placement_sequence=77;
        job.resource_plan.total_steps=kConstructionSteps;
        for(auto& cost:job.resource_plan.steps) cost.iron=200;
        delivery.placed_resources_.push_back(job);
        const ConstructionTargetRef target{ConstructionTargetKind::Resource,0};
        rover.mode=BuilderRobotMode::Delivering;
        rover.target=target;
        rover.cargo.iron=200;
        rover.logistics_reservation.iron=200;
        rover.cargo_committed_to_build=true;
        rover.starter_exit_completed=true;
        rover.position=delivery.construction_drop_position(target);
        delivery.builder_robots_.push_back(rover);
        BuilderRobotState obstruction;
        obstruction.battery_charge_kwh=0;
        obstruction.position=delivery.construction_drop_package_position(target,0);
        delivery.builder_robots_.push_back(obstruction);
        auto& carrier=delivery.builder_robots_[0];
        const Vec3 dock=carrier.position;
        // Keep the obstruction in the crate's landing volume. Normally
        // delivery asks an idle rover there to clear the drop automatically.
        const auto tick_blocked_drop=[&]{
            delivery.builder_robots_[1]=obstruction;
            delivery.update_builder_robot(1.f/90);
        };
        for(int frame=0;frame<30*90&&carrier.operation_retry.failures<4;++frame)
            tick_blocked_drop();
        require(carrier.operation_retry.failures==4&&carrier.operation_retry.action==BuilderRobotAction::PutDown,
                "failed drop fixture: count="+std::to_string(carrier.operation_retry.failures)+
                " operation="+builder_robot_action_name(carrier.operation_retry.action)+
                " cargo="+std::to_string(carrier.cargo.iron)+
                " deposited="+std::to_string(delivery.placed_resources_[0].site_buffer.iron)+
                " movement="+std::to_string(distance(carrier.position,dock))+
                " obstruction="+std::to_string(distance(delivery.builder_robots_[1].position,obstruction.position)));
        require(distance(carrier.position,dock)<.3f&&carrier.cargo.iron==200,
                "four failed drops moved the rover or changed cargo");
        for(int frame=0;frame<10*90&&distance(carrier.position,dock)<.3f;++frame)
            tick_blocked_drop();
        require(distance(carrier.position,dock)>=2&&carrier.operation_retry.failures==0&&
                carrier.cargo.iron==200&&carrier.logistics_reservation.iron==200&&
                delivery.placed_resources_[0].site_buffer.empty(),
                "fifth failed drop did not preserve cargo and reservation on relocation");
        std::cout<<"Operation recovery: red blocker outline, four retries, fifth-failure extraction and exact cargo custody passed\n";
    }

    static void global_routes(){
        std::vector<Box> boxes;PathPlannerCallbacks cb;cb.floor_height=[](Vec3){return 0.f;};
        cb.point_clear=[&](Vec3 p){if(std::abs(p.x)>40 || std::abs(p.z)>40)return false;for(const auto& b:boxes)if(p.x>=b.x0 && p.x<=b.x1 && p.z>=b.z0 && p.z<=b.z1)return false;return true;};
        cb.segment_clear=[&](Vec3 a,Vec3 b){
            if(!cb.point_clear(a) || !cb.point_clear(b))return false;
            for(const auto& box:boxes){float lo=0,hi=1;bool hit=true;for(int axis=0;axis<2;++axis){const float start=axis?a.z:a.x,delta=axis?b.z-a.z:b.x-a.x,min=axis?box.z0:box.x0,max=axis?box.z1:box.x1;
                if(std::abs(delta)<1e-8f){if(start<min || start>max){hit=false;break;}}
                else {float x=(min-start)/delta,y=(max-start)/delta;if(x>y)std::swap(x,y);lo=std::max(lo,x);hi=std::min(hi,y);if(lo>hi){hit=false;break;}}
            }if(hit)return false;}return true;
        };cb.neighbor_clear=cb.segment_clear;
        const auto solve=[&](PathPlanRequest request,std::uint64_t epoch,IncrementalPathPlannerState& state){std::vector<Vec3> route;auto status=IncrementalPathResult::Pending;
            for(int i=0;i<200 && status==IncrementalPathResult::Pending;++i)status=plan_incremental_path(request,epoch,cb,state,route);
            require(status==IncrementalPathResult::Success && !route.empty(),"global planner rejected reachable exact goal");Vec3 previous=request.start;for(const auto p:route){require(cb.segment_clear(previous,p),"global detour crosses an obstacle");previous=p;}require(distance(previous,request.goal)<0.001f,"route misses exact destination");return route;};
        PathPlanRequest request;request.start={-16,0,-10};request.goal={0,0,0.2f};request.cell_size=1.28f;request.corridor_half_width=10;request.max_expansions=90;request.world_half_extent=40;
        boxes={{-7,7,-7,-0.15f},{-7,7,0.45f,7}};IncrementalPathPlannerState state;solve(request,1,state);
        require(state.initialization_count>=1 && state.expanded_node_count>0,"narrow approach failed to enter bounded search");
        request.start={0,0,0.2f};request.goal={-16,0,-10.5f};request.world_half_extent=40;state={};solve(request,2,state);
        boxes={{-6,-4,-28,28}};request.start={-10,0,0};request.goal={0,0,0};request.world_half_extent=40;state={};auto route=solve(request,2,state);float excursion=0;for(auto p:route)excursion=std::max(excursion,std::abs(p.z));require(excursion>28,"route did not escape the old local search window");
        request.start={-39,0,0};state={};solve(request,3,state);require(state.initialization_count<=2,"map-edge start discarded pending searches");
        // Changing static obstacles repairs the persistent graph, without
        // admitting any segment across the newly blocked side.
        boxes.push_back({-40,-3,-40,-25});solve(request,4,state);
        std::cout<<"Exact-goal narrow approach, full-map detour, map-edge pending reuse and changed-obstacle repair passed\n";
    }
    static void saved_route(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;require(app.load_game_from_file(path,error),error);
        require(app.builder_robots_.size()>1,"saved route fixture needs carrier");
        const auto original=app.builder_robots_[1].target;const auto start=app.builder_robots_[1].position;
        const auto initial_step=app.construction_target_step(original);const auto buffer=app.construction_site_buffer(original);const auto initial_buffer=buffer?buffer->total_amount():0;
        bool delivered=false;float first_delivery=-1,max_travel=0;double worst=0;auto began=std::chrono::steady_clock::now();
        for(int frame=0;frame<=60*900;++frame){
            if(frame%1800==0){std::cout<<"t="<<frame/60<<"\n";for(std::size_t i=0;i<app.builder_robots_.size();++i){const auto& r=app.builder_robots_[i];std::cout<<i<<" p="<<r.position.x<<","<<r.position.z<<" v="<<r.current_speed_mps<<" mode="<<(int)r.mode<<" task="<<(int)r.target.kind<<":"<<r.target.index<<" cargo="<<r.cargo.total_amount()<<" station="<<r.interaction_position_valid<<":"<<r.interaction_position.x<<","<<r.interaction_position.z<<" goal="<<r.safety.goal.x<<","<<r.safety.goal.z<<" state="<<safety_mode_name(r.safety.mode)<<" reason="<<block_reason_name(r.safety.block_reason)<<" cached="<<r.safety.cached_route.size()<<" cursor="<<r.safety.route_cursor<<" wait="<<r.safety.wait_age<<" temp="<<r.safety.temporary_goal_valid<<":"<<r.safety.temporary_goal.x<<","<<r.safety.temporary_goal.z<<" D*="<<r.safety.incremental_path.expanded_node_count<<" init="<<r.safety.incremental_path.initialization_count<<"\n";}std::cout<<std::flush;}
            auto tick_start=std::chrono::steady_clock::now();app.update_pressure_doors(1.f/60);app.update_construction(1.f/60);
            worst=std::max(worst,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick_start).count());
            for(const auto& r:app.builder_robots_)require(r.safety.mode!=SafetyMode::Fault,"saved route entered safety fault");
            max_travel=std::max(max_travel,distance(start,app.builder_robots_[1].position));
            const auto now=app.construction_site_buffer(original);const bool progress=(now && now->total_amount()>initial_buffer) || app.construction_target_step(original)>initial_step;if(!delivered && progress)first_delivery=(float)frame/60;delivered=delivered || progress;
        }
        require(delivered,"saved carrier did not deliver its original cargo within 900 s");require(app.construction_target_complete(original),"original saved delivery task never completed after continued trips");require(max_travel>10,"saved carrier never left the blocked location");
        std::cout<<"Saved carrier delivery resumed; original target progressed, first delivery s="<<first_delivery<<" final step="<<(int)app.construction_target_step(original)<<" total ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()<<" worst tick ms="<<worst<<"\n";
    }
    static void clearance_save(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        require(!app.builder_robots_.empty(),"clearance fixture needs a builder");
        const Vec3 start=app.builder_robots_[0].position;
        float max_travel=0.0f;
        bool route_published=false;
        std::uint32_t max_replans=0;
        for(int frame=0;frame<60*30;++frame){
            app.update_pressure_doors(1.0f/60.0f);
            app.update_construction(1.0f/60.0f);
            const auto& robot=app.builder_robots_[0];
            require(robot.safety.mode!=SafetyMode::Fault,
                    std::string("clearance save entered safety fault: ")+block_reason_name(robot.safety.block_reason));
            max_travel=std::max(max_travel,distance(start,robot.position));
            route_published=route_published || !robot.safety.cached_route.empty() || robot.route_active;
            max_replans=std::max(max_replans,robot.safety.local_replans);
            if(max_travel>8.0f) break;
        }
        require(route_published,"builder never published a route from the narrow pocket");
        require(max_travel>8.0f,"builder remained blocked in the narrow pocket");
        require(max_replans<12,"precision fallback thrashed instead of escaping");
        std::cout<<"Saved narrow-passage builder escaped "<<max_travel
                 <<" m with full hard hull clearance; replans="<<max_replans<<"\n";
    }

    static void stuck_carrier_save(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        std::size_t carrier_index=app.builder_robots_.size();
        for(std::size_t i=0;i<app.builder_robots_.size();++i)
            if(app.builder_robots_[i].role==RobotRole::Carrier){carrier_index=i;break;}
        require(carrier_index<app.builder_robots_.size(),"stuck-carrier fixture needs a carrier");
        const Vec3 start=app.builder_robots_[carrier_index].position;
        std::uint32_t max_initializations=0,max_expansions=0;
        double total_ms=0.0,worst_ms=0.0;
        bool moved=false,parked=false;
        int frames=0;
        for(int frame=0;frame<60*120;++frame){
            ++frames;
            const auto began=std::chrono::steady_clock::now();
            app.update_pressure_doors(1.0f/60.0f);
            app.update_construction(1.0f/60.0f);
            const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
            total_ms+=elapsed;worst_ms=std::max(worst_ms,elapsed);
            const auto& carrier=app.builder_robots_[carrier_index];
            require(carrier.safety.mode!=SafetyMode::Fault,"stuck carrier entered safety fault");
            moved=moved || distance(start,carrier.position)>8.0f;
            parked=parked || carrier.workshop_parked;
            max_initializations=std::max(max_initializations,carrier.safety.incremental_path.initialization_count);
            max_expansions=std::max(max_expansions,carrier.safety.incremental_path.expanded_node_count);
            if(parked) break;
        }
        const auto& carrier=app.builder_robots_[carrier_index];
        require(moved,"carrier remained stuck at its saved position");
        require(parked,"carrier did not complete the workshop return");
        require(max_initializations<12,"stuck recovery repeatedly restarted the incremental planner");
        require(max_expansions<100000,"stuck recovery expanded an excessive number of route nodes");
        std::cout<<"Saved carrier recovered from "<<start.x<<","<<start.z
                 <<" to "<<carrier.position.x<<","<<carrier.position.z
                 <<"; planner init="<<max_initializations<<" nodes="<<max_expansions
                 <<" avg tick ms="<<(total_ms/std::max(1,frames))
                 <<" worst tick ms="<<worst_ms<<"\n";
    }

    static void pending_enclosure_save(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        require(!app.builder_robots_.empty(),"pending-enclosure fixture needs a builder");
        require(app.corridor_network_.connections().size()>5,"pending-enclosure fixture needs corridor 5");
        const Vec3 start=app.builder_robots_[0].position;
        const auto initial_step=app.corridor_network_.connections()[5].construction.step;
        require(initial_step<kCorridorConstructionSteps,"pending-enclosure corridor already complete");
        bool committed=false;
        Vec3 at_commit=start;
        for(int frame=0;frame<20*90;++frame){
            app.update_pressure_doors(0.05f);app.update_construction(0.05f);
            for(const auto& robot:app.builder_robots_)
                require(robot.safety.mode!=SafetyMode::Fault,"pending enclosure replay entered safety fault");
            if(app.corridor_network_.connections()[5].construction.step>initial_step){
                committed=true;at_commit=app.builder_robots_[0].position;break;
            }
        }
        require(committed,"pending enclosure corridor never committed after evacuation");
        require(distance(start,at_commit)>4.5f,"builder did not leave the future enclosure before corridor commit");
        require(at_commit.z>17.0f,"builder remained on the enclosed side when corridor committed");
        std::cout<<"Pending enclosure evacuated builder from "<<start.x<<","<<start.z
                 <<" to "<<at_commit.x<<","<<at_commit.z<<" before corridor commit\n";
    }
    static void enclosure_save(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        require(!app.builder_robots_.empty(),"enclosure fixture needs a robot");
        const Vec3 before=app.builder_robots_[0].position;
        app.update_pressure_doors(1.0f/60.0f);
        app.update_construction(1.0f/60.0f);
        const Vec3 after=app.builder_robots_[0].position;
        require(distance(before,after)>1.0f,"legacy enclosed robot was not emergency-extracted");
        for(int frame=0;frame<60*20;++frame){
            app.update_pressure_doors(1.0f/60.0f);app.update_construction(1.0f/60.0f);
            for(const auto& robot:app.builder_robots_)require(robot.safety.mode!=SafetyMode::Fault,"enclosure recovery entered safety fault");
        }
        require(distance(before,app.builder_robots_[0].position)>1.0f,"recovered robot returned to sealed pocket");
        std::cout<<"Legacy enclosed robot extracted from "<<before.x<<","<<before.z<<" to "<<after.x<<","<<after.z<<" and remained safe\n";
    }

    static void airlock_supply_save(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        std::size_t checked=0;
        for(std::size_t i=0;i<app.placed_habitats_.size();++i){
            const auto& h=app.placed_habitats_[i];
            if(h.kind!=HabitatKind::AirLock) continue;
            const auto& mesh=app.habitat_models_.mesh(h);
            for(const Vec3 local_cursor:std::array<Vec3,4>{
                    Vec3{ mesh.footprint_x*0.49f,0,0},Vec3{-mesh.footprint_x*0.49f,0,0},
                    Vec3{0,0, mesh.footprint_z*0.49f},Vec3{0,0,-mesh.footprint_z*0.49f}}){
                const Vec3 cursor=h.position+world_math::rotate_y(local_cursor,h.rotation_degrees);
                const auto anchor=app.utility_anchor_at(cursor,UtilityKind::SupplyChannel);
                require(anchor && anchor->kind==UtilityEndpointKind::Habitat && anchor->index==i,"save Air Lock side not selectable");
                const Vec3 away=cursor+world_math::rotate_y(eng::normalize(local_cursor)*14.0f,h.rotation_degrees);
                const auto port=app.utility_network_.preview_anchor(UtilityKind::SupplyChannel,*anchor,away,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
                require(port.has_value(),"save Air Lock side did not resolve to supply port");
                const Vec3 lp=world_math::rotate_y(port->position-h.position,-h.rotation_degrees);
                if(std::abs(local_cursor.x)>std::abs(local_cursor.z)) require(lp.x*local_cursor.x>0,"save Air Lock X-side remapped");
                else require(lp.z*local_cursor.z>0,"save Air Lock lateral side remapped");
                ++checked;
            }
        }
        require(checked>0,"save contains no Air Lock side to validate");
        std::cout<<"save Air Lock player-facing supply snapping passed on "<<checked<<" sides\n";
    }

    static void inspect(const std::string& path){
        eng::Application engine;GameApp app(engine);std::string error;
        require(app.load_game_from_file(path,error),error);
        for(std::size_t i=0;i<app.placed_habitats_.size();++i){const auto& h=app.placed_habitats_[i];const auto& m=app.habitat_models_.mesh(h);
            std::cout<<"habitat "<<i<<" "<<h.position.x<<","<<h.position.z<<" size="<<m.footprint_x<<","<<m.footprint_z<<" rot="<<h.rotation_degrees<<" step="<<(int)h.construction.step<<"\n";
        }
        for(std::size_t i=0;i<app.placed_resources_.size();++i){const auto& h=app.placed_resources_[i];const auto& m=app.resource_models_.mesh(h);
            std::cout<<"resource "<<i<<" kind="<<(int)h.kind<<" "<<h.position.x<<","<<h.position.z<<" size="<<m.footprint_x<<","<<m.footprint_z<<" rot="<<h.rotation_degrees<<" step="<<(int)h.construction.step<<"\n";
        }
        for(std::size_t i=0;i<app.builder_robots_.size();++i){const auto& r=app.builder_robots_[i];
            std::cout<<"robot "<<i<<" "<<r.position.x<<","<<r.position.z<<" station="<<r.interaction_position.x<<","<<r.interaction_position.z<<":"<<r.interaction_position_valid<<" step="<<(int)app.construction_target_step(r.target)<<"\n";
        }
        for(std::size_t i=0;i<app.corridor_network_.connections().size();++i){const auto& c=app.corridor_network_.connections()[i];
            std::cout<<"corridor "<<i<<" step="<<(int)c.construction.step<<" endpoints="<<(int)c.endpoint_a.kind<<":"<<c.endpoint_a.index<<" "<<(int)c.endpoint_b.kind<<":"<<c.endpoint_b.index<<"\n";
            for(auto p:c.path) std::cout<<p.x<<","<<p.z<<" ";std::cout<<"\n";
        }
        for(int frame=0;frame<1200;++frame){
            app.update_construction(1.0f/60);
            for(std::size_t i=0;i<app.builder_robots_.size();++i){auto& r=app.builder_robots_[i];if(r.safety.mode==SafetyMode::Fault) throw std::runtime_error("saved scene robot "+std::to_string(i)+" fault "+block_reason_name(r.safety.block_reason));}
        }
        std::cout<<"saved scene continued 20 s\n";
    }
    static void construction(const std::string& path,bool late_corridor=false,bool inject_fault=false){
        eng::Application engine;GameApp app(engine);
        app.playfield_.rebuild(GameApp::kDebugTemplatePlayfieldSize);app.starting_robot_count_=3;app.starting_carrier_count_=3;
        app.reset_builder_robot_and_starter_stash();
        std::string error;
        require(app.load_debug_blueprint_template_from_file(path,error),"load debug template: "+error);
        if(late_corridor){
            for(auto& h:app.placed_habitats_) h.construction.step=kConstructionSteps;
            for(auto& r:app.placed_resources_) r.construction.step=kConstructionSteps;
            auto& connections=app.corridor_network_.mutable_connections();connections[0].construction.step=kCorridorConstructionSteps;
            const ConstructionTargetRef target{ConstructionTargetKind::Corridor,1};
            const auto need=connections[1].resource_plan.remaining(0);
            auto& carrier=app.builder_robots_[3];carrier.position={-15.8f,0,-17.2f};carrier.heading_degrees=-97;
            carrier.mode=BuilderRobotMode::Delivering;carrier.target=target;carrier.cargo=need;carrier.logistics_reservation=need;
            carrier.cargo_committed_to_build=true;carrier.starter_exit_completed=true;
            app.builder_robots_[4].position={-15.1f,0,-6.9f};
            app.builder_robots_[5].position={-26.15f,0,-30.67f};
        }
        std::cout<<"construction layout: "<<app.placed_habitats_.size()<<" habitats, "<<app.placed_resources_.size()<<" resources, "<<app.corridor_network_.connections().size()<<" corridors\n";
        auto completed=[&](){
            std::size_t n=0;
            for(const auto& h:app.placed_habitats_) n+=h.construction.complete();
            for(const auto& r:app.placed_resources_) n+=r.construction.complete();
            for(const auto& c:app.corridor_network_.connections()) n+=c.construction.complete();
            return n;
        };
        const auto expected=app.placed_habitats_.size()+app.placed_resources_.size()+app.corridor_network_.connections().size();
        auto start=std::chrono::steady_clock::now();
        double worst=0;float t=0;
        std::size_t injected=game::kNoRobotIndex;float injected_at=0;std::int64_t held_cargo=0;
        for(int frame=0;t<1800.0f;++frame,t+=1.0f/60){
            const auto begin=std::chrono::steady_clock::now();
            app.update_construction(1.0f/60);
            if(inject_fault && injected==game::kNoRobotIndex && t>(late_corridor?0.5f:5.0f)){
                for(std::size_t i=3;i<app.builder_robots_.size();++i){auto& r=app.builder_robots_[i];
                    if(r.mode!=BuilderRobotMode::Delivering || r.cargo.empty() || !r.safety.goal_valid || distance(r.position,r.safety.goal)<3)continue;
                    injected=i;injected_at=t;held_cargo=r.cargo.total_amount();
                    r.safety.mode=SafetyMode::Fault;r.safety.block_reason=BlockReason::ContractViolation;r.safety.fault_phase=1;r.safety.fault_recovery_deadline=-1;
                    r.current_speed_mps=0;break;
                }
            }
            if(inject_fault && injected!=game::kNoRobotIndex){const auto& r=app.builder_robots_[injected];
                if(r.safety.mode==SafetyMode::Fault){require(t-injected_at<=3.04f,"carrier fault exceeded recovery deadline");require(r.cargo.total_amount()==held_cargo,"fault recovery lost scheduled cargo");}
                if(t-injected_at>3.04f)require(r.safety.fault_recovery_attempts>0,"carrier did not attempt fresh fault recovery");
            }
            if(frame%(late_corridor?600:7200)==0){
                std::cout<<"t="<<t<<" complete="<<completed()<<"\n";
                for(std::size_t i=3;i<app.builder_robots_.size();++i){const auto& r=app.builder_robots_[i];
                    std::cout<<i<<" p="<<r.position.x<<","<<r.position.z<<" heading="<<r.heading_degrees<<" v="<<r.current_speed_mps<<" mode="<<(int)r.mode<<" cargo="<<r.cargo.total_amount()<<" route="<<r.safety.route_cursor<<"/"<<r.safety.cached_route.size()<<" temp="<<r.safety.temporary_goal_valid<<":"<<r.safety.temporary_goal.x<<","<<r.safety.temporary_goal.z<<" goal="<<r.safety.goal.x<<","<<r.safety.goal.z<<" stop="<<r.safety.stop_radius<<" task="<<(int)r.target.kind<<":"<<r.target.index<<" exit="<<r.starter_exit_pending<<" rejected="<<r.safety.rejected_contracts<<" reason="<<block_reason_name(r.safety.block_reason)<<" replans="<<r.safety.local_replans<<" revision="<<r.safety.goal_revision<<" blocked="<<r.safety.blocked_elapsed<<" proposals="<<r.safety.proposals.size()<<"\n";

                }
                if(late_corridor) for(auto p:app.builder_robots_[3].safety.cached_route) std::cout<<" route "<<p.x<<","<<p.z<<"\n";
                std::cout.flush();
            }
            worst=std::max(worst,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
            for(std::size_t i=0;i<app.builder_robots_.size();++i){
                const auto& r=app.builder_robots_[i];
                const bool expected_fault=inject_fault && i==injected && t-injected_at<=3.04f;
                if(r.safety.mode==SafetyMode::Fault && !expected_fault){
                    std::string save_error;app.save_game_to_name("navigation-regression-fault",save_error);
                    for(std::size_t k=0;k<app.builder_robots_.size();++k){const auto& bot=app.builder_robots_[k];
                        std::cerr<<"fault state "<<k<<" "<<bot.position.x<<","<<bot.position.z<<" heading="<<bot.heading_degrees<<" speed="<<bot.current_speed_mps<<" phase="<<(int)bot.safety.fault_phase<<" mode="<<(int)bot.mode<<" target="<<(int)bot.target.kind<<":"<<bot.target.index<<" at "<<t<<"\n";
                    }
                }
                require(expected_fault || r.safety.mode!=SafetyMode::Fault,"construction robot "+std::to_string(i)+": "+block_reason_name(r.safety.block_reason)+" phase "+std::to_string(r.safety.fault_phase));
                for(std::size_t j=i+1;j<app.builder_robots_.size();++j)
                    require(distance(r.position,app.builder_robots_[j].position)>1.88f,"construction fleet overlap");
            }
            if(completed()==expected) break;
            if(late_corridor && t>240) break;
        }
        for(std::size_t i=0;i<app.builder_robots_.size();++i){
            const auto& r=app.builder_robots_[i];
            std::cout<<"robot "<<i<<" "<<r.position.x<<","<<r.position.z<<" "<<safety_mode_name(r.safety.mode)<<" "<<block_reason_name(r.safety.block_reason)<<" goal "<<r.safety.goal.x<<","<<r.safety.goal.z<<" task "<<(int)r.action<<"\n";
        }
        std::cout<<"construction "<<completed()<<"/"<<expected<<" after "<<t<<" sim s; "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<" wall s; worst frame "<<worst<<" ms\n";
        if(completed()!=expected){std::string save_error;app.save_game_to_name("navigation-regression-stalled",save_error);}
        require(completed()==expected,"construction tasks did not finish");
        if(inject_fault)require(injected!=game::kNoRobotIndex && app.builder_robots_[injected].safety.fault_recovery_attempts>0,"fault was not injected/recovered during construction");
    }
    static void storage_extensions(){
        eng::Application engine;

        // A newly placed dome is only the dome. It has no implicit gate and
        // selecting PLACE GATE enters extension-placement mode without creating
        // or charging a construction task yet.
        GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();
        PlacedResource parent;parent.kind=ResourceKind::StorageDome;parent.storage_size=StorageDomeSize::Small;
        parent.rotation_degrees=37;parent.placement_sequence=1;parent.construction.step=0;
        parent.resource_plan=construction_costs::structure(parent.kind,app.resource_models_.mesh(parent));
        app.placed_resources_={parent};app.next_blueprint_placement_sequence_=2;
        app.synchronize_storage_gates();
        require(storage_loading::gate_count(app.placed_resources_[0])==0,"new dome unexpectedly has an implicit storage gate");
        for(int face:storage_loading::faces(app.placed_resources_[0])) require(face<0,"new dome exposes an implicit cargo face");

        app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;
        const auto panel=app.selection_panel_rect();GeometryRecorder ui;
        app.ui_input_.mouse_x=(int)(panel.x+panel.w-94);app.ui_input_.mouse_y=(int)(panel.y+139);app.ui_input_.left_pressed=true;
        app.render_selection_panel(ui);
        require(app.storage_gate_placement_active_ && app.storage_gate_placement_owner_==0,"PLACE GATE did not enter extension placement mode");
        require(app.placed_resources_.size()==1,"PLACE GATE created a task before a world position was accepted");
        app.storage_gate_placement_active_=false;app.storage_gate_placement_owner_=kNoRobotIndex;app.ui_input_={};

        // Gates can be attached to a dome blueprint, one after another, and
        // each one is a fully independent construction task. Exactly three are
        // allowed; a fourth request is rejected even if another cargo sector is free.
        for(int face:{4,5,6}) require(app.add_storage_gate(0,face),"storage gate extension was rejected");
        require(app.placed_resources_.size()==4,"three gate tasks were not created");
        require(storage_loading::gate_count(app.placed_resources_[0])==3,"dome does not expose all three planned gates");
        require(!app.add_storage_gate(0,7),"a fourth storage gate was accepted");
        std::uint64_t previous_sequence=app.placed_resources_[0].placement_sequence;
        for(std::size_t i=1;i<4;++i){
            const auto& gate=app.placed_resources_[i];
            require(gate.kind==ResourceKind::StorageGate,"gate extension is not a StorageGate task");
            require(gate.storage_parent_sequence==app.placed_resources_[0].placement_sequence,"gate lost its dome parent");
            require(gate.resource_plan.total().total_amount()>0,"gate task has no construction material cost");
            require(gate.construction.step==0,"new gate task was pre-built");
            require(gate.placement_sequence>previous_sequence,"gate tasks are not ordered after their parent/previous gate");
            previous_sequence=gate.placement_sequence;
            require(storage_loading::reserved(app.placed_resources_[0],storage_loading::service(app.placed_resources_[0],gate.storage_loading_face)),
                    "gate blueprint did not reserve its loading apron");
        }

        // A completed dome uses the same optional extension path; completion of
        // the owner never auto-creates a gate.
        GameApp built(engine);built.playfield_.rebuild(128);
        parent.construction.step=kConstructionSteps;parent.storage_loading_face=-1;parent.placement_sequence=1;
        built.placed_resources_={parent};built.next_blueprint_placement_sequence_=2;built.synchronize_storage_gates();
        require(storage_loading::gate_count(built.placed_resources_[0])==0,"completed dome auto-created a gate");
        require(built.add_storage_gate(0,4) && built.placed_resources_[1].construction.step==0,"completed dome could not add a new gate blueprint");

        // Gate placement respects the usable build area. Structure occupation
        // and a supply-channel crossing each invalidate only the affected face.
        GameApp blocked(engine);blocked.playfield_.rebuild(128);
        PlacedResource blocked_parent=parent;blocked_parent.construction.step=kConstructionSteps;blocked_parent.storage_loading_face=-1;
        PlacedResource obstacle;obstacle.kind=ResourceKind::SolarPanel;obstacle.position=storage_loading::service(blocked_parent,4);
        obstacle.construction.step=kConstructionSteps;
        blocked.placed_resources_={blocked_parent,obstacle};blocked.next_blueprint_placement_sequence_=3;blocked.synchronize_storage_gates();
        require(blocked.available_storage_gate_face(0,4)<0,"gate was allowed through an occupied build area");
        require(blocked.available_storage_gate_face(0,6)==6,"an unrelated free gate sector was rejected");
        GameApp piped(engine);piped.playfield_.rebuild(128);piped.placed_resources_={blocked_parent};piped.next_blueprint_placement_sequence_=2;
        UtilityConnection pipe;pipe.kind=UtilityKind::SupplyChannel;
        const Vec3 n=storage_loading::normal(blocked_parent,4);
        pipe.path={storage_loading::wall(blocked_parent,4)-n*0.4f,storage_loading::service(blocked_parent,4)+n*1.0f};
        piped.utility_network_.mutable_connections().push_back(pipe);
        require(piped.available_storage_gate_face(0,4)<0,"gate was allowed through an existing supply channel");

        // Planned gates reserve geometry immediately, but only completed gate
        // tasks become robot-accessible loading interfaces.
        app.placed_resources_[0].construction.step=kConstructionSteps;
        app.placed_resources_[1].construction.step=0;
        app.placed_resources_[2].construction.step=kConstructionSteps;
        app.placed_resources_[3].construction.step=kConstructionSteps;
        app.synchronize_storage_gates();
        int ready=0;for(int face:storage_loading::faces(app.placed_resources_[0],true)) ready+=face>=0;
        require(ready==2,"unfinished gate became operational");
        app.placed_resources_[1].construction.step=kConstructionSteps;app.synchronize_storage_gates();
        ready=0;for(int face:storage_loading::faces(app.placed_resources_[0],true)) ready+=face>=0;
        require(ready==3,"completed gate task did not become operational");
        GeometryRecorder rendered;app.render_resources(rendered);
        require(rendered.green && rendered.filled_vertices==0,"completed storage gate extension mesh is missing");
        app.placed_resources_[1].construction.step=0;app.synchronize_storage_gates();
        GeometryRecorder blueprint;app.render_resources(blueprint);
        require(blueprint.blue && blueprint.wire_vertices>0 && blueprint.filled_vertices==0,"gate blueprint is not rendered as a construction mesh");
        app.placed_resources_[1].construction.step=kConstructionSteps;app.synchronize_storage_gates();

        // Save/load keeps all three child tasks and the derived gate slots.
        std::string error;require(app.save_game_to_name("storage-three-gates",error),error);
        const auto saved=std::filesystem::path(app.save_game_directory())/"storage-three-gates.gmsave";
        GameApp restored(engine);require(restored.load_game_from_file(saved.string(),error),error);
        require(restored.placed_resources_.size()==4,"storage gate tasks were lost on save/load");
        require(storage_loading::gate_count(restored.placed_resources_[0])==3,"derived three-gate state was not restored");
        std::filesystem::remove(saved);

        // Pre-0.43.1 saves stored the first gate directly on the dome. The
        // compatibility pass converts it into a child task instead of dropping it.
        GameApp legacy(engine);legacy.playfield_.rebuild(128);
        PlacedResource old=parent;old.storage_loading_face=4;old.placement_sequence=1;old.construction.step=kConstructionSteps;
        legacy.placed_resources_={old};legacy.next_blueprint_placement_sequence_=2;
        legacy.assign_storage_loading_faces();legacy.synchronize_storage_gates();
        require(legacy.placed_resources_.size()==2 && legacy.placed_resources_[1].kind==ResourceKind::StorageGate,
                "legacy integrated gate was not migrated to an extension task");
        require(legacy.placed_resources_[1].construction.complete() && storage_loading::gate_count(legacy.placed_resources_[0])==1,
                "legacy gate migration changed completion/availability");

        // Removing the dome removes all of its independent gate tasks as well.
        restored.selection_kind_=GameApp::SelectionKind::Resource;restored.selected_resource_=0;
        restored.delete_selected_world_object();
        require(restored.placed_resources_.empty(),"removing a storage dome orphaned gate extension tasks");

        std::cout<<"Optional 0-3 storage gates, independent build tasks, placement clearance, persistence and legacy migration passed\n";
    }
    static void workshops(){
        eng::Application engine;
        auto tick=[](GameApp& app){app.update_pressure_doors(1.0f/60);app.update_construction(1.0f/60);};
        auto connect_supply=[](GameApp& app,std::size_t resource_index){
            UtilityConnection link;link.kind=UtilityKind::SupplyChannel;
            link.endpoint_a.kind=UtilityEndpointKind::Resource;link.endpoint_a.index=resource_index;
            link.endpoint_b.kind=UtilityEndpointKind::Open;
            link.construction.step=link.construction.total_steps;
            app.utility_network_.mutable_connections().push_back(std::move(link));
        };
        auto power_network=[](GameApp& app,const std::vector<std::size_t>& consumers){
            const std::size_t solar_a=app.placed_resources_.size();
            PlacedResource a;a.kind=ResourceKind::SolarPanel;a.position={42,0,42};a.construction.step=kConstructionSteps;
            const std::size_t solar_b=solar_a+1;
            PlacedResource b=a;b.position={49,0,42};
            const std::size_t battery=solar_a+2;
            PlacedResource p;p.kind=ResourceKind::PowerStorage;p.position={56,0,42};p.construction.step=kConstructionSteps;
            app.placed_resources_.push_back(a);app.placed_resources_.push_back(b);app.placed_resources_.push_back(p);
            auto join=[&](std::size_t x,std::size_t y){
                UtilityConnection link;link.kind=UtilityKind::SupplyChannel;
                link.endpoint_a.kind=UtilityEndpointKind::Resource;link.endpoint_a.index=x;
                link.endpoint_b.kind=UtilityEndpointKind::Resource;link.endpoint_b.index=y;
                link.construction.step=link.construction.total_steps;
                app.utility_network_.mutable_connections().push_back(std::move(link));
            };
            join(solar_a,solar_b);join(solar_b,battery);
            for(std::size_t consumer:consumers) join(consumer,solar_a);
        };
        auto parked=[](const GameApp& app){return std::all_of(app.builder_robots_.begin(),app.builder_robots_.end(),[](const auto& r){return r.workshop_parked;});};
        auto verify=[](const GameApp& app){
            for(std::size_t i=0;i<app.builder_robots_.size();++i){const auto& r=app.builder_robots_[i];
                require(r.safety.mode!=SafetyMode::Fault,"workshop rover safety fault "+std::to_string(i));
                for(const auto& w:app.placed_resources_) if(w.kind==ResourceKind::RobotWorkshop && w.construction.complete())
                    require(workshop::clear(w,r.position,r.position,0.98f),"workshop wall/gate collision");
                for(std::size_t j=i+1;j<app.builder_robots_.size();++j)
                    require(distance(r.position,app.builder_robots_[j].position)>1.88f,"workshop fleet collision");
            }
        };
        auto wait_parking=[&](GameApp& app,const std::string& label){
            int frame=0;
            for(;frame<12000 && !parked(app);++frame){
                tick(app);verify(app);
                if(frame%1200==0){
                    std::cout<<label<<" t="<<(float)frame/60<<" gate="<<app.placed_resources_[0].workshop_gate_open<<"\n";
                    for(const auto& r:app.builder_robots_) std::cout<<" p="<<r.position.x<<","<<r.position.z<<" bay="<<r.workshop_bay<<" parked="<<r.workshop_parked<<" speed="<<r.current_speed_mps<<" reason="<<block_reason_name(r.safety.block_reason)<<" goal="<<r.safety.goal.x<<","<<r.safety.goal.z<<"\n";
                    std::cout.flush();
                }
            }
            if(!parked(app)) for(std::size_t i=0;i<app.builder_robots_.size();++i){const auto& r=app.builder_robots_[i];
                std::cerr<<label<<" robot "<<i<<" p="<<r.position.x<<","<<r.position.z<<" mode="<<(int)r.mode<<" home="<<r.workshop_index<<":"<<r.workshop_bay<<" reason="<<block_reason_name(r.safety.block_reason)<<" goal="<<r.safety.goal.x<<","<<r.safety.goal.z<<"\n";}
            require(parked(app),label+": not all robots parked");
            for(int i=0;i<180;++i){tick(app);verify(app);}
            for(const auto& w:app.placed_resources_) if(w.kind==ResourceKind::RobotWorkshop) require(w.workshop_gate_open==0,"workshop gate did not close after parking");
            for(const auto& r:app.builder_robots_){
                require(r.workshop_index<app.placed_resources_.size(),"missing parking assignment");
                const auto& w=app.placed_resources_[r.workshop_index];
                require(workshop::inside(w,r.position) && std::abs(r.position.y-workshop::kDeck)<0.02f,"robot did not park on the internal deck");
            }
            std::cout<<label<<": parked after "<<(float)frame/60<<" s\n";
        };
        for(float rotation:{0.0f,37.0f,90.0f}){
            GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=2;app.starting_carrier_count_=2;
            app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_.clear();
            PlacedResource w;w.kind=ResourceKind::RobotWorkshop;w.rotation_degrees=rotation;
            w.construction.step=kConstructionSteps;app.placed_resources_={w};
            connect_supply(app,0);
            power_network(app,{0});
            for(std::size_t i=0;i<4;++i){auto& r=app.builder_robots_[i];
                r.position=workshop::world(w,{-7.5f+5.0f*(float)i,0,-20.0f});r.heading_degrees=rotation;
                r.mode=BuilderRobotMode::Idle;r.target={};r.condition=0.50f;}
            wait_parking(app,"four-bay rotation "+std::to_string(rotation));
            for(const auto& r:app.builder_robots_) require(r.condition>0.52f,"docked maintenance did not restore condition");
            if(rotation==0.0f){
                auto& probe=app.builder_robots_[0];probe.battery_charge_kwh=probe.battery_capacity_kwh*0.05f;
                auto clock=app.status_.state();clock.hour=12.0f;app.status_.restore(clock);
                const float before=probe.battery_charge_kwh;
                app.update_supply_simulation(10.0f);
                require(probe.battery_charge_kwh>before && probe.battery_charging,"powered workshop did not recharge a parked low-battery rover");
                require(app.placed_resources_[0].supply.power_demand_kw>0.6,"robot charging was not accounted as workshop electrical load");
                probe.battery_charge_kwh=probe.battery_capacity_kwh;probe.battery_charging=false;
            }
            const auto poses=app.builder_robots_;
            for(int i=0;i<240;++i) tick(app);
            for(std::size_t i=0;i<4;++i) require(distance(poses[i].position,app.builder_robots_[i].position)<0.02f,"parked rover wanders");
            // A new real job must preempt parking and open the gate for exit.
            PlacedResource target;target.kind=ResourceKind::SolarPanel;target.position={27,0,15};target.placement_sequence=1;
            target.resource_plan.total_steps=kConstructionSteps;
            for(auto& step:target.resource_plan.steps) step.amount(MaterialResource::Iron)=5;
            const std::size_t target_index=app.placed_resources_.size();
            app.placed_resources_.push_back(target);
            GroundResourcePile pile;pile.position={22,0,-16};pile.amount=30;pile.resource=MaterialResource::Iron;pile.radius=0.4f;
            app.ground_resource_piles_={pile};ResourceBundle inventory;inventory.amount(MaterialResource::Iron)=30;app.resources_.restore(inventory);
            bool exited=false;
            for(int frame=0;frame<30000 && !app.placed_resources_[target_index].construction.complete();++frame){
                tick(app);verify(app);
                for(const auto& r:app.builder_robots_) if(r.target.valid() && !workshop::inside(app.placed_resources_[0],r.position)) exited=true;
            }
            require(exited && app.placed_resources_[target_index].construction.complete(),"parked fleet did not dispatch and complete new work");
            wait_parking(app,"return after task "+std::to_string(rotation));
            std::string error;require(app.save_game_to_name("workshop-regression",error),error);
            const auto saved=std::filesystem::path(app.save_game_directory())/"workshop-regression.gmsave";
            GameApp loaded(engine);require(loaded.load_game_from_file(saved.string(),error),error);
            wait_parking(loaded,"loaded workshop");std::filesystem::remove(saved);
        }
        GameApp placement(engine);placement.playfield_.rebuild(128);
        PlacedResource w;w.kind=ResourceKind::RobotWorkshop;
        const auto& compact_mesh=placement.resource_models_.mesh(ResourceKind::RobotWorkshop,ResourceSizeTier::Standard);
        require(compact_mesh.footprint_x<=11.01f && compact_mesh.footprint_z<=10.01f,"workshop footprint was not reduced");
        require(placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,0},0),"open workshop placement rejected");
        require(!placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,-51},0),"ramp clearance crossed map boundary");
        // A structure that still fits geometrically inside the map must be
        // rejected if it consumes the permanent perimeter rover lane.
        const float half_field=(float)placement.playfield_.size()*0.5f;
        const auto& solar_mesh=placement.resource_models_.mesh(ResourceKind::SolarPanel,ResourceSizeTier::Small);
        const float solar_ground_x=geometry::structure_ground_extent(solar_mesh.footprint_x);
        const float near_border_x=half_field-solar_ground_x*0.5f-0.10f;
        require(!placement.resource_placement_valid(ResourceKind::SolarPanel,ResourceSizeTier::Small,StorageDomeSize::Small,{near_border_x,0,0},0),"structure consumed perimeter rover lane");
        const float safe_border_x=half_field-geometry::kPlayfieldRobotPerimeterLane-solar_ground_x*0.5f-0.20f;
        require(placement.resource_placement_valid(ResourceKind::SolarPanel,ResourceSizeTier::Small,StorageDomeSize::Small,{safe_border_x,0,0},0),"structure rejected inside perimeter rover lane boundary");
        PlacedResource obstacle;obstacle.kind=ResourceKind::PowerStorage;obstacle.position={0,0,-14};placement.placed_resources_={obstacle};
        require(!placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,0},0),"workshop ramp can be placed against an obstacle");
        w.construction.step=kConstructionSteps;placement.placed_resources_={w};
        require(!placement.resource_placement_valid(ResourceKind::SolarPanel,ResourceSizeTier::Small,StorageDomeSize::Small,{0,0,-14},0),"new structure blocked reserved workshop ramp");
        require(!placement.placement_valid(HabitatKind::AirLock,HabitatSizeTier::Small,{0,0,-14},0),"new habitat blocked reserved workshop ramp");
        GeometryRecorder geometry;
        placement.render_workshop_gate(geometry,w);
        const auto front=workshop::world(w,{0,1.3f,-workshop::kHalfZ-0.5f}),back=workshop::world(w,{0,1.3f,-workshop::kHalfZ+0.5f});
        require(segment_hits_mesh(front,back,geometry.triangles),"closed workshop gate has no panel");
        w.workshop_gate_open=1;geometry.triangles.clear();placement.render_workshop_gate(geometry,w);
        require(!segment_hits_mesh(front,back,geometry.triangles),"open workshop gate still obstructs the centre");
        float last=0;
        for(int i=0;i<=100;++i){const float z=workshop::kRampEnd+(-workshop::kHalfZ-workshop::kRampEnd)*(float)i/100;
            const float y=workshop::floor(w,workshop::world(w,{0,0,z}));
            require(y>=last-0.001f && y-last<0.02f,"ramp floor is discontinuous");last=y;}
        w.workshop_gate_open=0;
        placement.placed_resources_.clear();placement.starting_robot_count_=1;placement.starting_carrier_count_=1;
        placement.reset_builder_robot_and_starter_stash();placement.ground_resource_piles_.clear();
        placement.builder_robots_[0].position={45,0,40};placement.builder_robots_[1].position={48,0,40};
        CorridorConnection fence;fence.path={{-20,0,-25},{20,0,-25},{20,0,20},{-20,0,20},{-20,0,-25}};
        placement.corridor_network_.restore_connections({fence});
        require(!placement.workshop_accessible(w),"enclosed unreachable workshop approach accepted");
        // More than four robots need more than one four-bay structure.
        GameApp fleet(engine);fleet.playfield_.rebuild(128);fleet.starting_robot_count_=4;fleet.starting_carrier_count_=4;
        fleet.reset_builder_robot_and_starter_stash();fleet.ground_resource_piles_.clear();
        w.position={-18,0,0};PlacedResource other=w;other.position={18,0,0};other.rotation_degrees=180;
        fleet.placed_resources_={w,other};
        connect_supply(fleet,0);connect_supply(fleet,1);power_network(fleet,{0,1});
        for(std::size_t i=0;i<8;++i){auto& r=fleet.builder_robots_[i];const auto& home=fleet.placed_resources_[i/4];
            r.position=workshop::world(home,{-7.5f+5*(float)(i%4),0,-20});r.mode=BuilderRobotMode::Idle;r.target={};}
        wait_parking(fleet,"eight robots / two workshops");
        // Exercise the actual construction sequence, not only completed assets.
        GameApp construction(engine);construction.playfield_.rebuild(128);
        construction.starting_robot_count_=2;construction.starting_carrier_count_=2;
        construction.reset_builder_robot_and_starter_stash();
        PlacedResource blueprint;blueprint.kind=ResourceKind::RobotWorkshop;blueprint.placement_sequence=1;
        blueprint.resource_plan.total_steps=kConstructionSteps;
        for(auto& step:blueprint.resource_plan.steps) step.amount(MaterialResource::Iron)=10;
        construction.placed_resources_={blueprint};
        connect_supply(construction,0);power_network(construction,{0});
        for(int frame=0;frame<36000 && !construction.placed_resources_[0].construction.complete();++frame){
            tick(construction);
            for(const auto& r:construction.builder_robots_){
                require(r.safety.mode!=SafetyMode::Fault,"workshop construction safety fault");
                if(!construction.placed_resources_[0].construction.complete()) require(r.workshop_index==kNoRobotIndex,"robot parked in an unfinished workshop");
            }
        }
        require(construction.placed_resources_[0].construction.complete(),"workshop construction did not finish");
        wait_parking(construction,"newly constructed workshop");
        // A completed but unconnected workshop is intentionally offline.  It
        // must not attract idle robots or provide maintenance until a completed
        // supply channel terminates at the structure.
        GameApp offline(engine);offline.playfield_.rebuild(128);offline.starting_robot_count_=1;offline.starting_carrier_count_=1;
        offline.reset_builder_robot_and_starter_stash();offline.ground_resource_piles_.clear();
        PlacedResource unpowered;unpowered.kind=ResourceKind::RobotWorkshop;unpowered.construction.step=kConstructionSteps;
        offline.placed_resources_={unpowered};
        for(int i=0;i<240;++i) tick(offline);
        require(!offline.workshop_supplied(0) && !offline.workshop_operational(0),"unconnected workshop reported online");
        for(const auto& r:offline.builder_robots_) require(!r.workshop_parked && r.workshop_index==kNoRobotIndex,"robot used an unpowered workshop");
        connect_supply(offline,0);
        require(offline.workshop_supplied(0) && !offline.workshop_operational(0),"unpowered pipe network incorrectly enabled workshop");
        power_network(offline,{0});
        require(offline.workshop_operational(0),"powered supply network did not enable workshop");
        wait_parking(offline,"supply-and-power-enabled workshop");
        std::cout<<"Workshop supply gating, compact parking, task dispatch, ramp clearance, reachability, save/load and multi-workshop allocation passed\n";
    }
    static void ore_carrier(){
        eng::Application engine;
        for(const bool use_dome:{false,true}){
            GameApp app(engine);
            app.playfield_.rebuild(128);
            app.active_game_phase_=GameApp::GamePhase::Debug;
            app.game_session_active_=true;
            app.utility_network_.set_playfield_half_extent(64);
            PlacedResource drill;
            drill.kind=ResourceKind::OreDriller;drill.position={-20,0,-8};drill.construction.finish();
            drill.placement_sequence=1;
            drill.mined_ore_kg[0]=ore::box_kg;
            app.placed_resources_.push_back(drill);
            app.next_blueprint_placement_sequence_=3;
            if(use_dome){
                PlacedResource dome;
                dome.kind=ResourceKind::StorageDome;dome.position={18,0,-9};dome.construction.finish();
                dome.placement_sequence=2;
                app.placed_resources_.push_back(dome);
                require(app.add_storage_gate_at(1,0),"could not build storage ore loading gate");
                require(storage_loading::has_ready_gate(app.placed_resources_[1]),"ore storage gate was not ready");
            }else{
                PlacedHabitat plant;
                plant.kind=HabitatKind::ProcessingFacility;plant.position={20,0,-10};plant.construction.finish();
                plant.placement_sequence=2;
                plant.supply.volume_m3=100;plant.supply.oxygen_kg=100*kOxygenKgPerM3;
                plant.supply.power_fraction=1;
                app.placed_habitats_.push_back(plant);
            }
            BuilderRobotState carrier;
            carrier.role=RobotRole::Carrier;
            carrier.position={-20,0,10};
            carrier.starter_exit_completed=true;
            carrier.battery_capacity_kwh=kCarrierRobotBatteryCapacityKwh;
            carrier.battery_charge_kwh=carrier.battery_capacity_kwh;
            carrier.move_speed_mps=kCarrierRobotTransitSpeedMps;
            app.builder_robots_.push_back(carrier);

            if(!use_dome){
                const UtilityBuildAnchor source{UtilityEndpointKind::Resource,0,drill.position};
                const UtilityBuildAnchor intake{UtilityEndpointKind::Habitat,0,app.placed_habitats_[0].position};
                require(app.utility_network_.build_connection(UtilityKind::ConveyorBelt,source,intake,
                    app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_),
                    "could not create the conveyor exclusion fixture");
                app.utility_network_.mutable_connections().front().construction.finish();
                app.update_builder_robot(1.f/90.f);
                require(!app.builder_robots_[0].ore_haul.active(),"carrier took over a driller with a built conveyor");
                app.utility_network_.remove_connection(0);
            }

            bool picked_up=false;
            for(int step=0;step<6000;++step){
                app.update_builder_robot(1.f/90.f);
                const auto& rover=app.builder_robots_[0];
                if(rover.ore_haul.loaded && !picked_up){
                    picked_up=true;
                    require(app.resources_.amounts().empty() && app.placed_resources_[0].mined_ore_kg[0]==0,
                            "pickup duplicated ore or credited raw material");
                    std::string error;
                    require(app.save_game_to_name("ore-carrier-regression",error),error);
                    const auto path=std::filesystem::path(app.save_game_directory())/"ore-carrier-regression.gmsave";
                    GameApp resumed(engine);
                    require(resumed.load_game_from_file(path.string(),error),error);
                    std::filesystem::remove(path);
                    require(resumed.builder_robots_[0].ore_haul.loaded &&
                            resumed.builder_robots_[0].ore_haul.kind==0 &&
                            resumed.placed_resources_[0].mined_ore_kg[0]==0,"save lost loaded carrier ore");
                }
                if(picked_up && !rover.ore_haul.active())break;
            }
            require(picked_up && !app.builder_robots_[0].ore_haul.active(),
                    use_dome?"carrier did not deliver to storage":"carrier did not deliver to processing");
            if(use_dome){
                require(app.placed_resources_[1].stored_ore_boxes[0]==1 && app.resources_.amounts().empty() &&
                        storage_dome_used_kg(app.placed_resources_[1])==ore::box_kg,"storage ore capacity or accounting mismatch");
                std::string error;
                require(app.save_game_to_name("ore-carrier-regression",error),error);
                const auto path=std::filesystem::path(app.save_game_directory())/"ore-carrier-regression.gmsave";
                GameApp resumed(engine);
                require(resumed.load_game_from_file(path.string(),error),error);
                std::filesystem::remove(path);
                require(resumed.placed_resources_[1].stored_ore_boxes[0]==1 && resumed.resources_.amounts().empty(),
                        "stored raw ore did not survive save/load");
            }else{
                require(app.resources_.amount(MaterialResource::Iron)==ore::refined_box_kg,
                        "carrier did not credit one refined crate");
            }
        }
        std::cout<<"Ore carrier physical pickup, processing delivery, storage delivery, and save/load passed\n";
    }

    static void conveyor_ore(){
        eng::Application engine;
        GameApp app(engine);
        app.playfield_.rebuild(128);
        app.active_game_phase_=GameApp::GamePhase::Debug;
        app.game_session_active_=true;
        auto& network=app.utility_network_;
        network.set_playfield_half_extent(64);
        PlacedResource drill;
        drill.kind=ResourceKind::OreDriller;drill.position={-20,0,0};drill.construction.finish();
        drill.supply.power_fraction=1;
        PlacedHabitat plant;plant.kind=HabitatKind::ProcessingFacility;plant.position={18,0,0};plant.construction.finish();
        plant.supply.volume_m3=100;plant.supply.oxygen_kg=100*kOxygenKgPerM3;
        app.placed_resources_={drill};app.placed_habitats_={plant};
        const auto ore_mix=ore::deposit(drill.position);
        const auto another_mix=ore::deposit({22,0,18});
        double sum=0;
        for(const auto share:ore_mix){require(share>0&&share<1,"invalid ore concentration");sum+=share;}
        require(std::abs(sum-1)<1e-9&&ore_mix!=another_mix,"ore survey does not vary by location");
        const UtilityBuildAnchor source{UtilityEndpointKind::Resource,0,drill.position};
        const UtilityBuildAnchor intake{UtilityEndpointKind::Habitat,0,plant.position};
        const auto build=[&](UtilityKind kind,const auto& a,const auto& b){
            return network.build_connection(kind,a,b,app.placed_habitats_,app.habitat_models_,
                app.placed_resources_,app.resource_models_);
        };
        require(!build(UtilityKind::ConveyorBelt,intake,source),"belt accepted a reversed route");
        app.placed_habitats_[0].kind=HabitatKind::CrewQuarters;
        require(!build(UtilityKind::ConveyorBelt,source,intake),"belt accepted an unrelated habitat");
        app.placed_habitats_[0].kind=HabitatKind::Factory;
        require(build(UtilityKind::ConveyorBelt,source,intake),"driller could not connect to factory");
        require(!build(UtilityKind::ConveyorBelt,source,intake),"driller accepted a duplicate conveyor");
        require(app.conveyor_route_clear(network.connections()[0].path) &&
                !network.connections()[0].resource_plan.total().empty() && !network.connections()[0].mesh.edges.empty(),
                "conveyor route lacks clearance, mesh or construction costs");
        network.mutable_connections()[0].construction.finish();
        const auto services=network.evaluate_services(app.corridor_network_,app.placed_habitats_,app.placed_resources_);
        require(!services.resources[0].connected && !services.habitats[0].connected,"conveyor carried utilities");

        require(build(UtilityKind::SupplyChannel,source,intake) && network.connections().size()==2,
                "conveyor prevented an independent power channel between its endpoints");
        auto& belt=network.mutable_connections().front();
        app.update_ore_logistics(240);
        require(app.resources_.amounts().empty() && belt.ore_cargo<ore::no_cargo &&
                belt.ore_distance_m>0,"raw ore was credited or no cargo was sent");
        const auto cargo=belt.ore_cargo;
        const auto distance=belt.ore_distance_m;
        const auto stock=app.placed_resources_[0].mined_ore_kg;
        require(std::count_if(stock.begin(),stock.end(),[](double kg){return ore::boxes(kg)>0;})>=2,
                "powered driller did not produce distinct ore boxes");

        std::string error;
        require(app.save_game_to_name("conveyor-regression",error),error);
        const auto save=std::filesystem::path(app.save_game_directory())/"conveyor-regression.gmsave";
        GameApp restored(engine);
        require(restored.load_game_from_file(save.string(),error),error);
        std::filesystem::remove(save);
        const auto& restored_belt=restored.utility_network_.connections().front();
        require(restored_belt.ore_cargo==cargo && restored_belt.ore_distance_m==distance &&
                restored.placed_resources_[0].mined_ore_kg==stock,"ore custody was lost on save/load");

        app.placed_habitats_[0].supply.power_fraction=1;
        app.update_ore_logistics(1);
        require(app.resources_.amount(ore::materials[cargo])==ore::refined_box_kg &&
                belt.ore_cargo==ore::no_cargo && app.ground_resource_piles_.size()==1,
                "powered receiver did not process the arriving crate exactly once");
        app.update_ore_logistics(1);
        require(belt.ore_cargo<ore::no_cargo && belt.ore_distance_m<network.connection_length(0),
                "second ore crate did not travel on the belt");
        const auto next=belt.ore_cargo;
        const double before=app.placed_resources_[0].mined_ore_kg[next];
        app.remove_utility_route(0);
        require(network.connections().size()==1 && network.connections()[0].kind==UtilityKind::SupplyChannel &&
                app.placed_resources_[0].mined_ore_kg[next]==before+ore::box_kg,
                "removing a conveyor lost its in-flight crate");
        std::cout<<"Ore deposits, directed conveyors, utility isolation, crate delivery, deletion and save/load passed\n";
    }

    static void structure_routes(){
        eng::Application engine;
        GameApp app(engine);
        app.playfield_.rebuild(128);
        auto& corridors=app.corridor_network_;
        auto& utilities=app.utility_network_;
        auto& habitats=app.placed_habitats_;
        auto& resources=app.placed_resources_;
        corridors.set_playfield_half_extent(64);
        utilities.set_playfield_half_extent(64);

        const auto open_corridor=[](Vec3 position){return CorridorBuildAnchor{CorridorEndpointKind::OpenDoor,0,position};};
        const auto open_pipe=[](Vec3 position){return UtilityBuildAnchor{UtilityEndpointKind::Open,0,position};};
        require(corridors.build_corridor(open_corridor({-20,0,-20}),open_corridor({0,0,-20}),
            habitats,app.habitat_models_,resources,app.resource_models_),"standalone corridor rejected");
        require(utilities.build_connection(UtilityKind::SupplyChannel,open_pipe({-25,0,20}),open_pipe({-5,0,20}),
            habitats,app.habitat_models_,resources,app.resource_models_),"standalone pipe rejected");
        require(utilities.build_connection(UtilityKind::SupplyChannel,open_pipe({5,0,35}),open_pipe({25,0,35}),
            habitats,app.habitat_models_,resources,app.resource_models_),"second standalone pipe rejected");
        require(utilities.connections().size()==2,"unrelated open pipes treated as duplicates");

        PlacedHabitat habitat;habitat.kind=HabitatKind::OxygenGeneratorDome;habitat.size=HabitatSizeTier::Small;
        habitat.rotation_degrees=30;
        const auto& hm=app.habitat_models_.mesh(habitat);
        const auto& cm=corridors.connections().front();
        const auto base=world_math::footprint({},habitat.rotation_degrees,hm.footprint_x,hm.footprint_z);
        const auto target=cm.path.back()+Vec3{world_math::projected_radius(base,{1,0})+6.05f,0,0};
        require(!app.placement_valid(habitat.kind,habitat.size,{-8,0,-20},habitat.rotation_degrees),
                "habitat overlapped a corridor");
        app.ground_resource_piles_.push_back({MaterialResource::Iron,100,target,.6f,1});
        require(!app.preview_structure_attachment(target,&habitat,nullptr),"corridor snap ignored a blocked base");
        app.ground_resource_piles_.clear();
        auto corridor_snap=app.preview_structure_attachment(target,&habitat,nullptr);
        require(corridor_snap&&corridor_snap->corridor,"rotated habitat did not attach to free corridor end");
        habitat.position=corridor_snap->position;habitat.placement_sequence=app.next_blueprint_placement_sequence_++;
        habitats.push_back(habitat);
        app.commit_structure_attachment(std::move(*corridor_snap));
        require(corridors.connections().size()==2 && corridors.connections()[0].endpoint_b.suppress_open_door &&
            corridors.connections()[1].endpoint_b.kind==CorridorEndpointKind::Habitat,"corridor terminal did not connect");

        PlacedResource antenna;antenna.kind=ResourceKind::Antenna;antenna.size=ResourceSizeTier::Small;
        const auto& am=app.resource_models_.mesh(antenna);
        require(!app.resource_placement_valid(antenna.kind,antenna.size,antenna.storage_size,{-15,0,20},0),
                "outdoor structure overlapped an existing pipe");
        auto pipe_snap=app.preview_structure_attachment({-5+am.footprint_x*.5f+1.5f,0,20},nullptr,&antenna);
        require(pipe_snap&&pipe_snap->utility,"outdoor structure did not attach to free pipe end");
        antenna.position=pipe_snap->position;antenna.placement_sequence=app.next_blueprint_placement_sequence_++;
        resources.push_back(antenna);
        app.commit_structure_attachment(std::move(*pipe_snap));
        require(utilities.connections().size()==3 && utilities.connections()[2].endpoint_b.kind==UtilityEndpointKind::Resource &&
                !utilities.connections()[0].junctions.empty(),"pipe terminal did not merge");

        require(utilities.build_connection(UtilityKind::SupplyChannel,
            {UtilityEndpointKind::Corridor,1,{3,0,-18.5f},{0,0,1}},open_pipe({3,0,-9}),
            habitats,app.habitat_models_,resources,app.resource_models_),"corridor service run rejected");
        utilities.mutable_connections().back().site_buffer.iron=27;
        app.builder_robots_.resize(2);
        app.builder_robots_[0].target={ConstructionTargetKind::Utility,3};
        app.builder_robots_[0].logistics_reservation.iron=5;
        app.builder_robots_[1].target={ConstructionTargetKind::Utility,2};
        app.selection_kind_=GameApp::SelectionKind::Habitat;
        app.selected_habitat_=0;
        app.delete_selected_world_object();
        require(habitats.empty() && corridors.connections().size()==1 &&
                !corridors.connections()[0].endpoint_b.suppress_open_door,"demolition left the corridor attached or host unsealed");
        require(utilities.connections().size()==3 && !app.builder_robots_[0].target.valid() &&
                app.builder_robots_[0].logistics_reservation.empty() &&
                app.builder_robots_[1].target.index==2,"demolition kept a pipe on a deleted corridor or broke a surviving assignment");
        std::int64_t recovered=0;
        for(const auto& pile:app.ground_resource_piles_)if(pile.resource==MaterialResource::Iron)recovered+=pile.amount;
        require(recovered==27,"removed pipe material was lost or duplicated");

        app.selection_kind_=GameApp::SelectionKind::Resource;
        app.selected_resource_=0;
        app.delete_selected_world_object();
        require(resources.empty() && utilities.connections().size()==2 &&
                utilities.connections()[0].junctions.empty() &&
                !app.builder_robots_[1].target.valid(),"outdoor demolition left a pipe attached or removed an unrelated run");
        std::cout<<"Structure routes: rotated snap, footprint rejection, corridor cap and pipe junctions, demolition, material recovery and robot remapping passed\n";
    }

    static void interaction(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);
        // Every angle and every dome tier uses an identical rigid local mesh.
        const auto& reference=app.resource_models_.mesh(ResourceKind::StorageGate,StorageDomeSize::Small);
        for(int size=0;size<4;++size)for(int angle=0;angle<360;angle+=3){
            PlacedResource dome;dome.kind=ResourceKind::StorageDome;dome.storage_size=(StorageDomeSize)size;dome.rotation_degrees=37;
            storage_loading::set_angle(dome,8,(float)angle);
            PlacedResource gate;gate.kind=ResourceKind::StorageGate;gate.storage_size=dome.storage_size;gate.storage_loading_face=8;gate.storage_gate_angle_degrees=(float)angle;
            const auto& mesh=app.resource_models_.mesh(gate);
            require(mesh.body.size()==reference.body.size() && mesh.footprint_x==reference.footprint_x && mesh.footprint_z==reference.footprint_z,"gate mesh changes with angle/tier");
            for(std::size_t i=0;i<mesh.body.size();++i)require(distance(mesh.body[i],reference.body[i])<1e-6,"cargo module squeezed or stretched");
            require(std::abs(distance(storage_loading::mouth(dome,8),storage_loading::wall(dome,8))-storage_loading::kDepth)<1e-5,"gate depth changes around dome");
            const auto n=storage_loading::normal(dome,8),side=Vec3{-n.z,0,n.x};const auto service=storage_loading::service(dome,8);
            require(storage_loading::reserved_face(dome,service+n*1.20f,0,8) && storage_loading::reserved_face(dome,storage_loading::mouth(dome,8)+side*2.48f,0,8),"minimum robot/shutter clearance missing");
            require(distance(service,dome.position)>storage_loading::body_radius(dome)+1.2f,"compact gate pickup is inside rover dome envelope");
            const auto& body=app.resource_models_.mesh(dome);
            for(const auto* layer:{&body.body,&body.foundation,&body.details})for(const auto& v:*layer)require(std::hypot(v.x,v.z)<=storage_loading::body_radius(dome)+0.001f,"round rover envelope clips dome geometry");
        }
        for(int size=0;size<4;++size)for(float angle:{17.0f,53.25f}){
            GameApp route(engine);route.playfield_.rebuild(128);route.starting_robot_count_=1;route.starting_carrier_count_=1;route.reset_builder_robot_and_starter_stash();route.ground_resource_piles_.clear();
            PlacedResource dome;dome.kind=ResourceKind::StorageDome;dome.storage_size=(StorageDomeSize)size;dome.rotation_degrees=37;dome.placement_sequence=1;dome.construction.step=kConstructionSteps;
            route.placed_resources_={dome};route.next_blueprint_placement_sequence_=2;
            require(route.add_storage_gate_at(0,angle),"free-angle compact gate fixture rejected");route.placed_resources_[1].construction.step=kConstructionSteps;route.synchronize_storage_gates();
            const auto& owner=route.placed_resources_[0];const auto n=storage_loading::normal(owner,8);const Vec3 side{-n.z,0,n.x};const auto service=storage_loading::service(owner,8);
            route.builder_robots_[0].position={-40,0,-40};route.builder_robots_[0].starter_exit_completed=true;
            auto& carrier=route.builder_robots_[1];carrier.position=service+n*8+side*4;carrier.mode=BuilderRobotMode::DepotTransfer;carrier.cargo.iron=10;carrier.starter_exit_completed=true;
            for(int frame=0;frame<7200 && route.placed_resources_[0].stored_resources.iron<10;++frame){
                route.update_construction(1.0f/60);require(carrier.safety.mode!=SafetyMode::Fault,"compact gate route entered safety fault");
            }
            require(route.placed_resources_[0].stored_resources.iron==10,"carrier could not reach compact gate at size "+std::to_string(size)+" angle "+std::to_string(angle));
            if(size==0 && angle>50)require(!route.add_storage_gate_at(0,100.0f),"overlapping cargo approaches were accepted");
        }
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-20,0,-20});
        eng::InputState jump;jump.key_space_pressed=true;jump.key_space_down=true;app.update_first_person(1.0f/120,jump);
        require(app.first_person_.feet_position().y>0 && !app.first_person_.grounded(),"Space did not launch POV jump");
        jump.key_space_pressed=false;float peak=0;
        for(int i=0;i<360;++i){app.update_first_person(1.0f/120,jump);peak=std::max(peak,app.first_person_.feet_position().y);}
        require(peak>0.97f && peak<1.02f && app.first_person_.grounded() && std::abs(app.first_person_.feet_position().y)<1e-5,"lunar jump arc/landing or held-key behavior is wrong");
        // A second press in mid-air cannot launch another jump.
        FirstPersonController single,double_press;single.spawn({0,0,0});double_press.spawn({0,0,0});
        for(int i=0;i<360;++i){single.advance_vertical(i==0,1.0f/120,0,100);double_press.advance_vertical(i==0 || i==60,1.0f/120,0,100);require(distance(single.feet_position(),double_press.feet_position())<1e-6,"airborne jump reset vertical speed");}
        // Controller and actual rendered corridor roof both stop the head.
        FirstPersonController low;low.spawn({0,0,0});
        for(int i=0;i<360;++i){low.advance_vertical(i==0,1.0f/120,0,low.body_height()+0.30f);require(low.feet_position().y<=0.281f,"jump passed ceiling");}
        require(low.grounded(),"ceiling contact prevented landing");
        app.corridor_network_.clear();
        CorridorBuildAnchor a{CorridorEndpointKind::OpenDoor,0,{-8,0,0}},b{CorridorEndpointKind::OpenDoor,0,{8,0,0}};
        require(app.corridor_network_.build_corridor(a,b,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"jump corridor fixture rejected");
        auto links=app.corridor_network_.connections();links[0].construction.step=links[0].construction.total_steps;app.corridor_network_.restore_connections(links);
        app.first_person_.spawn({0,app.first_person_floor_height({0,0,0}),0});const float ceiling=app.first_person_ceiling_height(app.first_person_.feet_position());
        require(ceiling<3.0f && ceiling>2.0f,"rendered corridor roof not found");
        jump.key_space_pressed=true;app.update_first_person(1.0f/120,jump);jump.key_space_pressed=false;
        for(int i=0;i<360;++i){app.update_first_person(1.0f/120,jump);require(app.first_person_.eye_position().y<ceiling && app.first_person_.feet_position().y+app.first_person_.body_height()+0.019f<=ceiling,"POV head crossed corridor roof");}
        require(app.first_person_.grounded(),"POV did not land on corridor deck");
        app.corridor_network_.clear();PlacedResource solid;solid.kind=ResourceKind::WaterStorage;solid.construction.step=kConstructionSteps;app.placed_resources_={solid};
        const float edge=app.resource_models_.mesh(solid).footprint_z*0.5f;
        app.first_person_.spawn({0,0,edge+1.0f});jump.key_w_down=true;jump.key_space_pressed=true;
        app.update_first_person(1.0f/120,jump);jump.key_space_pressed=false;
        for(int i=0;i<360;++i){app.update_first_person(1.0f/120,jump);require(app.first_person_.feet_position().z>=edge+app.first_person_.body_radius()-0.03f,"jump bypassed a solid structure wall");}
        std::cout<<"480 rigid gate angles/tiers, 8 autonomous gate arrivals, compact pickup/shutter clearance, lunar jump/landing, no airborne re-jump and ceiling collision passed\n";
    }

    static void oxygen(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(192);
        app.status_.reset();auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        const auto resource=[](ResourceKind kind,Vec3 p){PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;};
        app.placed_resources_={resource(ResourceKind::SolarPanel,{-40,0,-20}),resource(ResourceKind::WaterStorage,{-20,0,-20}),resource(ResourceKind::WaterStorage,{60,0,60}),resource(ResourceKind::StorageDome,{0,0,20})};
        app.placed_resources_[1].supply.water_litres=20;app.placed_resources_[2].supply.water_litres=100;
        const auto habitat=[](HabitatKind kind,Vec3 p){PlacedHabitat h;h.kind=kind;h.position=p;h.construction.step=kConstructionSteps;return h;};
        app.placed_habitats_={habitat(HabitatKind::OxygenGeneratorDome,{-20,0,0}),habitat(HabitatKind::CrewQuarters,{-20,0,20}),habitat(HabitatKind::Factory,{50,0,0})};
        UtilityConnection power;power.endpoint_a.kind=power.endpoint_b.kind=UtilityEndpointKind::Resource;power.endpoint_a.index=0;power.endpoint_b.index=1;power.path={{-40,0,-20},{-20,0,-20}};power.construction.step=power.construction.total_steps;
        UtilityConnection feed;feed.endpoint_a.kind=UtilityEndpointKind::Resource;feed.endpoint_a.index=1;feed.endpoint_b.kind=UtilityEndpointKind::Habitat;feed.endpoint_b.index=0;feed.path={{-20,0,-20},{-20,0,0}};feed.construction.step=feed.construction.total_steps;
        app.utility_network_.restore_connections({power,feed});
        CorridorConnection hall;hall.endpoint_a.kind=hall.endpoint_b.kind=CorridorEndpointKind::Habitat;hall.endpoint_a.index=0;hall.endpoint_b.index=1;hall.path={{-20,0,0},{-20,0,20}};hall.construction.step=hall.construction.total_steps;
        CorridorConnection storage;storage.endpoint_a.kind=CorridorEndpointKind::Habitat;storage.endpoint_a.index=1;storage.endpoint_b.kind=CorridorEndpointKind::Resource;storage.endpoint_b.index=3;storage.path={{-20,0,20},{0,0,20}};storage.construction.step=storage.construction.total_steps;
        app.corridor_network_.restore_connections({hall,storage});
        const auto each=[&](auto fn){for(auto& h:app.placed_habitats_)fn(h.supply);for(auto& r:app.placed_resources_)fn(r.supply);for(auto& u:app.utility_network_.mutable_connections())fn(u.supply);for(auto& c:app.corridor_network_.mutable_connections())fn(c.supply);};
        const auto mass=[&](){double total=0;each([&](const SupplyState& s){total+=s.oxygen_kg;});return total;};
        app.update_supply_simulation(0);
        require(app.placed_habitats_[0].supply.volume_m3>0 && mass()==0,"zero-time air metadata refresh advanced the simulation");
        app.update_supply_simulation(0.1f);
        const auto& gen=app.placed_habitats_[0].supply;
        require(gen.oxygen_produced_kg_per_day>17.99 && gen.oxygen_produced_kg_per_day<=18.00001,"generator does not report doubled oxygen output");
        require(std::abs(gen.water_processed_litres_per_day-gen.oxygen_produced_kg_per_day*kWaterPerOxygen)<1e-9,"water processing and oxygen rate disagree");
        require(std::abs(gen.hydrogen_produced_kg_per_day-gen.oxygen_produced_kg_per_day*kHydrogenPerOxygen)<1e-9,"electrolysis hydrogen/oxygen mass ratio is not 1:8");
        require(gen.hydrogen_kg>0 && gen.hydrogen_kg<=kOxygenGeneratorHydrogenBufferKg,"oxygen generator did not retain hydrogen by-product in its local buffer");
        require(app.placed_habitats_[1].supply.oxygen_kg==0 && app.placed_resources_[3].supply.oxygen_kg==0,"generator instantaneously filled downstream interiors");
        app.update_supply_simulation(60);
        require(std::abs((20-app.placed_resources_[1].supply.water_litres)-mass()*kWaterPerOxygen)<1e-7,"water-to-oxygen network mass balance failed");
        require(app.placed_habitats_[1].supply.oxygen_kg>0 && app.placed_resources_[3].supply.oxygen_kg>0,"connected indoor room or storage did not receive oxygen");
        require(app.placed_habitats_[2].supply.oxygen_kg==0 && app.placed_resources_[2].supply.water_litres==100,"disconnected supply teleported into the network");
        const auto& room=app.placed_habitats_[1].supply;
        require(room.air_change_percentage_points_per_day>0 && room.oxygen_net_kg_per_day>0,"filling room has no measured air-change rate");
        require(std::abs(room.air_change_percentage_points_per_day-100*room.oxygen_net_kg_per_day/(room.volume_m3*kOxygenKgPerM3))<1e-9,"air percentage rate is inconsistent with volume/mass");
        app.placed_resources_[1].supply.water_litres=0;const double before_dry=mass();app.update_supply_simulation(1);
        require(gen.oxygen_produced_kg_per_day==0 && gen.water_processed_litres_per_day==0 && (gen.warnings&NoWater),"dry generator produced oxygen or reported a false running rate");
        require(std::abs(mass()-before_dry)<1e-8,"dry network created/lost oxygen without crew consumption");
        app.placed_resources_[1].supply.water_litres=100;app.placed_resources_[0].supply.energy_kwh=0;clock.hour=0;app.status_.restore(clock);const double before_unpowered=app.placed_resources_[1].supply.water_litres;app.update_supply_simulation(1);
        require(gen.oxygen_produced_kg_per_day==0 && gen.water_processed_litres_per_day==0 && (gen.warnings&NoPower) && app.placed_resources_[1].supply.water_litres==before_unpowered,"unpowered generator processed water");
        clock.hour=12;app.status_.restore(clock);each([](SupplyState& s){s.oxygen_kg=s.volume_m3*kOxygenKgPerM3;});app.update_supply_simulation(1);
        require(gen.oxygen_produced_kg_per_day>0 && gen.oxygen_produced_kg_per_day<18 && gen.water_processed_litres_per_day>0 && app.placed_resources_[1].supply.water_litres<100,"nominal full-pressure leakage was not replenished by life support");
        clock.hour=0;clock.crew_current=6;app.status_.restore(clock);app.update_supply_simulation(1);
        require(app.placed_habitats_[2].supply.air_change_percentage_points_per_day<0,"depleting isolated interior did not report falling air");
        clock.hour=12;clock.crew_current=0;app.status_.restore(clock);each([](SupplyState& s){s.oxygen_kg=0;});app.placed_resources_[1].supply.water_litres=0.01;app.update_supply_simulation(10);
        require(std::abs(mass()*kWaterPerOxygen-0.01)<1e-9 && app.placed_resources_[1].supply.water_litres>=0 && gen.oxygen_produced_kg_per_day<18,"nearly empty tank was overdrawn or reported nominal output");
        const double saved_mass=mass();app.update_supply_simulation(0);
        require(mass()==saved_mass && gen.oxygen_produced_kg_per_day==0 && gen.oxygen_net_kg_per_day==0,"zero-time refresh consumed water or retained a stale rate");
        std::cout<<"Water electrolysis, measured output/intake and air-change rates, room/storage distribution, depletion, isolation and input/capacity limits passed\n";
    }

    static void hydrogen_refueling(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(192);app.status_.reset();
        app.builder_robots_.resize(1);app.builder_robots_[0].position={60,0,60};
        auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        auto resource=[](ResourceKind kind,Vec3 p){PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;};
        PlacedHabitat electrolyzer;electrolyzer.kind=HabitatKind::OxygenGeneratorDome;electrolyzer.position={-25,0,0};electrolyzer.construction.step=kConstructionSteps;
        electrolyzer.supply.hydrogen_kg=4.0;app.placed_habitats_={electrolyzer};
        app.placed_resources_={resource(ResourceKind::HydrogenTank,{-10,0,0}),resource(ResourceKind::LandingPad,{5,0,0}),
                               resource(ResourceKind::SolarPanel,{24,0,0}),resource(ResourceKind::PowerStorage,{38,0,0})};
        auto link=[](UtilityEndpointKind a,std::size_t ai,UtilityEndpointKind b,std::size_t bi,Vec3 pa,Vec3 pb){
            UtilityConnection c;c.endpoint_a.kind=a;c.endpoint_a.index=ai;c.endpoint_b.kind=b;c.endpoint_b.index=bi;
            c.path={pa,pb};c.construction.step=c.construction.total_steps;return c;
        };
        const auto tank_line=link(UtilityEndpointKind::Habitat,0,UtilityEndpointKind::Resource,0,{-25,0,0},{-10,0,0});
        const auto pad_line=link(UtilityEndpointKind::Resource,0,UtilityEndpointKind::Resource,1,{-10,0,0},{5,0,0});
        const auto solar_line=link(UtilityEndpointKind::Resource,1,UtilityEndpointKind::Resource,2,{5,0,0},{24,0,0});
        const auto battery_line=link(UtilityEndpointKind::Resource,2,UtilityEndpointKind::Resource,3,{24,0,0},{38,0,0});
        app.utility_network_.restore_connections({tank_line,pad_line,solar_line,battery_line});
        require(app.build_catalog_.items().size()>0,"empty build catalog");
        bool has_tank=false;for(const auto& item:app.build_catalog_.items())if(item.resource_kind==ResourceKind::HydrogenTank)has_tank=true;
        require(has_tank,"hydrogen tank is not a buildable structure");
        const auto& mesh=app.resource_models_.mesh(ResourceKind::HydrogenTank);
        require(!mesh.body.empty() && !mesh.details.empty() && mesh.footprint_x>12.0f && mesh.height>8.0f,"large cryogenic tank size/mesh incorrect");
        const auto& small_mesh=app.resource_models_.mesh(ResourceKind::HydrogenTank,ResourceSizeTier::Small);
        require(small_mesh.height<mesh.height*0.7f && small_mesh.footprint_x<mesh.footprint_x*0.7f,"small cryogenic tank was not scaled volumetrically");
        const double v=4.0/3.0*3.141592653589793*kHydrogenTankStandardVesselRadiusM*kHydrogenTankStandardVesselRadiusM*kHydrogenTankStandardVesselRadiusM;
        require(hydrogen_tank_capacity_kg(ResourceSizeTier::Standard)<=v*kLiquidHydrogenDensityKgPerM3*kHydrogenTankUsableVolumeFraction,"standard hydrogen nameplate exceeds physical storage volume");
        require(hydrogen_tank_capacity_kg(ResourceSizeTier::Small)<=v*std::pow(kHydrogenTankSmallLinearScale,3.0)*kLiquidHydrogenDensityKgPerM3*kHydrogenTankUsableVolumeFraction,"small hydrogen nameplate exceeds physical storage volume");
        require(landing_pad_rocket_fuel_capacity_kg()==4200.0 && landing_pad_lander_fuel_capacity_kg()==1150.0,"lunar return fuel model mismatch");
        require(landing_pad_rocket_oxidizer_requirement_kg()==25200.0 && landing_pad_lander_oxidizer_requirement_kg()==6900.0,"hydrolox oxidizer ratio mismatch");
        app.update_supply_simulation(0.2f);
        require(app.placed_resources_[0].supply.hydrogen_kg>3.9,"electrolysis H2 did not transfer to connected storage");
        require(app.placed_habitats_[0].supply.hydrogen_kg<0.1,"oxygen generator kept H2 despite connected tank");
        const double stored_before=app.placed_resources_[0].supply.hydrogen_kg;
        const double first=app.refuel_landing_pad(1,true);
        require(first>3.9 && first<=stored_before+1e-6,"rocket did not consume actual tank hydrogen");
        require(std::abs((stored_before-app.placed_resources_[0].supply.hydrogen_kg)-first)<1e-7,"rocket fuel transfer lost mass");
        require(app.refuel_landing_pad(1,true)<1e-8,"rocket refueled again without hydrogen");
        app.placed_resources_[0].supply.hydrogen_kg=hydrogen_tank_capacity_kg(ResourceSizeTier::Standard);
        require(app.refuel_landing_pad(1,true)>0.0,"rocket could not finish filling from storage");
        require(app.refuel_landing_pad(1,false)>0.0,"lander could not fill from storage");
        require(std::abs(app.placed_resources_[1].rocket_fuel_kg-landing_pad_rocket_fuel_capacity_kg())<1e-8,"rocket capacity exceeded or not filled");
        require(std::abs(app.placed_resources_[1].lander_fuel_kg-landing_pad_lander_fuel_capacity_kg())<1e-8,"lander capacity exceeded or not filled");
        const double left=app.placed_resources_[0].supply.hydrogen_kg;
        require(left>=0.0 && left<hydrogen_tank_capacity_kg(ResourceSizeTier::Standard)-5300.0,"fueling did not subtract tonne-scale propellant");
        require(app.refuel_landing_pad(1,true)==0 && app.refuel_landing_pad(1,false)==0,"double fueling after full tanks");
        require(std::abs(app.placed_resources_[0].supply.hydrogen_kg-left)<1e-8,"full vehicles consumed hydrogen");
        std::string error;require(app.save_game_to_name("hydrogen-tank-refuel",error),error);
        const auto saved=std::filesystem::path(app.save_game_directory())/"hydrogen-tank-refuel.gmsave";
        GameApp loaded(engine);require(loaded.load_game_from_file(saved.string(),error),error);
        require(loaded.placed_resources_[0].kind==ResourceKind::HydrogenTank,"hydrogen tank kind lost on load");
        require(std::abs(loaded.placed_resources_[0].supply.hydrogen_kg-left)<1e-8,"tank inventory changed after save/load");
        require(std::abs(loaded.placed_resources_[1].rocket_fuel_kg-landing_pad_rocket_fuel_capacity_kg())<1e-8 &&
                std::abs(loaded.placed_resources_[1].lander_fuel_kg-landing_pad_lander_fuel_capacity_kg())<1e-8,"vehicle fuel allocations lost on load");
        std::filesystem::remove(saved);
        // A disconnected tank must not magically refuel even when the pad has power.
        app.placed_resources_[1].rocket_fuel_kg=0;
        app.utility_network_.restore_connections({solar_line,battery_line});
        app.update_supply_simulation(0.0f);
        require(app.refuel_landing_pad(1,true)==0,"disconnected hydrogen tank supplied the landing pad");
        require(app.refuel_landing_pad(999,true)==0,"invalid pad index accepted");
        std::cout<<"Hydrogen tank, connected transfers, rocket/lander refueling, isolation and save/load passed\n";
    }

    static void hydrogen_solar(){
        constexpr float eps=0.015f;
        const auto morning=solar_tracking::pose(6.25f),noon=solar_tracking::pose(12.0f),evening=solar_tracking::pose(17.75f),night=solar_tracking::pose(20.0f);
        require(morning.above_horizon && morning.direction.x>0.95f && morning.elevation_degrees>0,"star does not rise in the east above the horizon");
        require(noon.above_horizon && noon.direction.z<-0.45f && noon.elevation_degrees>59.0f && noon.elevation_degrees<61.0f,"noon star is not on the southern 60-degree reference arc");
        require(std::abs(eng::length(noon.position)-solar_tracking::kStarReferenceDistanceMetres)<0.05f,"nonvisualized star has no stable future render position");
        require(evening.above_horizon && evening.direction.x<-0.95f,"star does not cross toward the western horizon");
        require(!night.above_horizon && night.irradiance_fraction==0,"night star still produces photovoltaic irradiance");
        require(noon.irradiance_fraction>0.99f && morning.irradiance_fraction>0.99f,"two-axis tracking failed to retain near-normal incidence away from the horizon ramp");

        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(160);app.status_.reset();
        auto clock=app.status_.state();clock.hour=0;clock.crew_current=0;app.status_.restore(clock);
        auto resource=[](ResourceKind kind,Vec3 p){PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;};
        auto habitat=[](HabitatKind kind,Vec3 p){PlacedHabitat h;h.kind=kind;h.position=p;h.construction.step=kConstructionSteps;return h;};
        app.placed_habitats_={habitat(HabitatKind::OxygenGeneratorDome,{-10,0,0})};
        app.placed_habitats_[0].supply.hydrogen_kg=1.0;
        app.placed_resources_={resource(ResourceKind::HydrogenGenerator,{10,0,0}),resource(ResourceKind::PowerStorage,{25,0,0})};
        UtilityConnection h2;h2.endpoint_a.kind=UtilityEndpointKind::Habitat;h2.endpoint_a.index=0;h2.endpoint_b.kind=UtilityEndpointKind::Resource;h2.endpoint_b.index=0;h2.path={{-10,0,0},{10,0,0}};h2.construction.step=h2.construction.total_steps;
        UtilityConnection battery;battery.endpoint_a.kind=battery.endpoint_b.kind=UtilityEndpointKind::Resource;battery.endpoint_a.index=0;battery.endpoint_b.index=1;battery.path={{10,0,0},{25,0,0}};battery.construction.step=battery.construction.total_steps;
        app.utility_network_.restore_connections({h2,battery});
        const double h2_before=app.placed_habitats_[0].supply.hydrogen_kg;
        app.update_supply_simulation(10.0f); // one simulated hour
        const auto& cell=app.placed_resources_[0].supply;
        require(cell.hydrogen_power_kw>2.99 && cell.hydrogen_power_kw<=kHydrogenGeneratorPeakKw+eps,"hydrogen generator did not dispatch at its rated useful-output limit");
        require(app.placed_resources_[1].supply.energy_kwh>0,"hydrogen generator surplus did not charge its connected battery");
        require(app.placed_habitats_[0].supply.hydrogen_kg<h2_before,"fuel cell produced electricity without consuming connected hydrogen");
        require(std::abs(cell.hydrogen_consumed_kg_per_day*kHydrogenElectricalKwhPerKg/24.0-cell.hydrogen_power_kw)<0.01,"fuel-cell hydrogen-to-electricity energy balance is inconsistent");

        // A disconnected generator must neither teleport hydrogen nor create power.
        const double stored=app.placed_resources_[1].supply.energy_kwh,remaining=app.placed_habitats_[0].supply.hydrogen_kg;
        app.utility_network_.restore_connections({battery});app.update_supply_simulation(10.0f);
        require((app.placed_resources_[0].supply.warnings&NoHydrogen)!=0,"disconnected hydrogen generator has no missing-fuel warning");
        require(app.placed_habitats_[0].supply.hydrogen_kg==remaining,"disconnected fuel cell teleported hydrogen from the oxygen generator");
        require(app.placed_resources_[1].supply.energy_kwh<=stored+1e-9,"disconnected hydrogen generator created stored energy");

        // Completed solar collectors physically follow the star, but a clock/target change
        // must not teleport the collector. The visual pose advances only through the
        // tracker slew update and converges smoothly toward the new orientation.
        GameApp solar(engine);solar.playfield_.rebuild(160);solar.placed_resources_={resource(ResourceKind::SolarPanel,{0,0,0})};
        auto solar_clock=solar.status_.state();solar_clock.hour=8;solar.status_.restore(solar_clock);solar.update_supply_simulation(2.0f);GeometryRecorder east_view;solar.render_resources(east_view);
        solar_clock.hour=12;solar.status_.restore(solar_clock);GeometryRecorder unchanged_view;solar.render_resources(unchanged_view);
        require(east_view.triangles.size()==unchanged_view.triangles.size() && !east_view.triangles.empty(),"tracked solar renderer emitted inconsistent geometry");
        float instant_shift=0;for(std::size_t i=0;i<east_view.triangles.size();++i)instant_shift=std::max(instant_shift,eng::length(east_view.triangles[i]-unchanged_view.triangles[i]));
        require(instant_shift<0.001f,"solar collector jumped when the target sun position changed");
        solar.update_supply_simulation(0.1f);GeometryRecorder transition_view;solar.render_resources(transition_view);
        float transition_shift=0;for(std::size_t i=0;i<east_view.triangles.size();++i)transition_shift=std::max(transition_shift,eng::length(east_view.triangles[i]-transition_view.triangles[i]));
        require(transition_shift>0.01f && transition_shift<0.75f,"solar collector did not begin a smooth slew toward the new sun position");
        solar.update_supply_simulation(2.0f);GeometryRecorder noon_view;solar.render_resources(noon_view);
        float max_shift=0;for(std::size_t i=0;i<east_view.triangles.size();++i)max_shift=std::max(max_shift,eng::length(east_view.triangles[i]-noon_view.triangles[i]));
        require(max_shift>0.25f,"solar collector head did not follow the star across the playfield");
        std::cout<<"Hydrogen electrolysis/fuel-cell energy cycle, network isolation, realistic star arc and two-axis solar tracking passed\n";
    }

    static void battery_bootstrap(){
        eng::Application engine;

        // A colony without an operational workshop must never become a permanent
        // energy deadlock. Reproduce the failure mode from 0-43-28(1).gmsave:
        // one rover is already empty and the other is below the 10% reserve.
        // Both must reach the starter charger, recover, and never enter a safety
        // fault merely because their traction battery was depleted.
        {
            GameApp recovery(engine);recovery.playfield_.rebuild(128);
            recovery.starting_robot_count_=1;recovery.starting_carrier_count_=1;
            recovery.reset_builder_robot_and_starter_stash();
            recovery.builder_robots_[0].position={-20,0,-15};
            recovery.builder_robots_[1].position={10,0,8};
            recovery.builder_robots_[0].battery_charge_kwh=0.0f;
            recovery.builder_robots_[1].battery_charge_kwh=recovery.builder_robots_[1].battery_capacity_kwh*0.055f;
            bool builder_recovered=false,carrier_recovered=false;
            for(int frame=0;frame<60*150 && !(builder_recovered&&carrier_recovered);++frame){
                recovery.update_construction(1.0f/60.0f);
                for(const auto& r:recovery.builder_robots_)
                    require(r.safety.mode!=SafetyMode::Fault,"battery depletion incorrectly became a safety fault");
                builder_recovered=!recovery.builder_robots_[0].starter_charge_active && recovery.builder_robots_[0].battery_charge_kwh>=recovery.builder_robots_[0].battery_capacity_kwh*0.75f;
                carrier_recovered=!recovery.builder_robots_[1].starter_charge_active && recovery.builder_robots_[1].battery_charge_kwh>=recovery.builder_robots_[1].battery_capacity_kwh*0.75f;
            }
            require(builder_recovered&&carrier_recovered,"starter bootstrap charger failed to recover depleted fleet");
        }

        GameApp app(engine);app.playfield_.rebuild(128);
        app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();
        app.placed_resources_.clear();app.placed_habitats_.clear();app.utility_network_.clear();
        app.utility_network_.set_playfield_half_extent(64);
        std::uint64_t sequence=1;float elapsed=0.0f;
        const auto assert_batteries=[&](){
            for(const auto& robot:app.builder_robots_){
                require(robot.safety.mode!=SafetyMode::Fault,"Stage-2 battery fixture entered safety fault");
                require(robot.battery_charge_kwh>0.0f,"robot exhausted battery before Stage 2");
                require(!robot.battery_charging,"Stage-2 bootstrap unexpectedly recharged a robot");
            }
        };
        const auto run_until=[&](auto&& done,const char* label){
            const float deadline=elapsed+900.0f;
            while(!done() && elapsed<deadline){app.update_construction(1.0f/60.0f);elapsed+=1.0f/60.0f;assert_batteries();}
            if(!done()){
                std::cerr<<"Battery bootstrap timeout "<<label<<" at "<<elapsed<<" s\n";
                for(std::size_t i=0;i<app.builder_robots_.size();++i){const auto&r=app.builder_robots_[i];std::cerr<<" bot"<<i<<" role="<<(int)r.role<<" mode="<<(int)r.mode<<" action="<<(int)r.action<<" target="<<(int)r.target.kind<<":"<<r.target.index<<" charge="<<r.battery_charge_kwh<<" pos="<<r.position.x<<","<<r.position.z<<" cargo="<<r.cargo.total_amount()<<" speed="<<r.current_speed_mps<<" actionElapsed="<<r.action_elapsed<<"/"<<r.action_duration<<"\n";}
                for(std::size_t i=0;i<app.utility_network_.connections().size();++i){const auto&u=app.utility_network_.connections()[i];std::cerr<<" util"<<i<<" step="<<(int)u.construction.step<<"/"<<(int)u.construction.total_steps<<" buffer="<<u.site_buffer.total_amount()<<" plan="<<u.resource_plan.total().total_amount()<<" path="<<u.path.size()<<"\n";}
                for(std::size_t i=0;i<app.placed_resources_.size();++i){const auto&r=app.placed_resources_[i];std::cerr<<" res"<<i<<" step="<<(int)r.construction.step<<"/"<<(int)kConstructionSteps<<" buffer="<<r.site_buffer.total_amount()<<"\n";}
                for(std::size_t i=0;i<app.placed_habitats_.size();++i){const auto&h=app.placed_habitats_[i];std::cerr<<" hab"<<i<<" step="<<(int)h.construction.step<<"/"<<(int)kConstructionSteps<<" elapsed="<<h.construction.step_elapsed<<" buffer="<<h.site_buffer.total_amount()<<"\n";}
            }
            require(done(),std::string("Stage-2 bootstrap target timed out: ")+label);
        };
        const auto add_resource=[&](ResourceKind kind,Vec3 position,const char* label){
            PlacedResource r;r.kind=kind;r.position=position;r.placement_sequence=sequence++;
            r.resource_plan=construction_costs::structure(kind,app.resource_models_.mesh(r));
            app.placed_resources_.push_back(std::move(r));const auto index=app.placed_resources_.size()-1;
            run_until([&]{return app.placed_resources_[index].construction.complete();},label);return index;
        };
        const auto add_habitat=[&](HabitatKind kind,Vec3 position,const char* label){
            PlacedHabitat h;h.kind=kind;h.size=HabitatSizeTier::Small;h.position=position;h.placement_sequence=sequence++;
            h.resource_plan=construction_costs::habitat(kind,h.size,app.habitat_models_.mesh(h));
            app.placed_habitats_.push_back(std::move(h));const auto index=app.placed_habitats_.size()-1;
            run_until([&]{return app.placed_habitats_[index].construction.complete();},label);return index;
        };
        const auto connect=[&](UtilityEndpointKind ka,std::size_t ia,UtilityEndpointKind kb,std::size_t ib,const char* label,bool wait=true){
            const Vec3 pa=ka==UtilityEndpointKind::Resource?app.placed_resources_[ia].position:app.placed_habitats_[ia].position;
            const Vec3 pb=kb==UtilityEndpointKind::Resource?app.placed_resources_[ib].position:app.placed_habitats_[ib].position;
            UtilityBuildAnchor a{ka,ia,pa},b{kb,ib,pb};
            require(app.utility_network_.build_connection(UtilityKind::SupplyChannel,a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_),std::string("bootstrap supply connection rejected: ")+label);
            const auto index=app.utility_network_.connections().size()-1;app.utility_network_.mutable_connections()[index].placement_sequence=sequence++;
            if(wait) run_until([&]{return app.utility_network_.connections()[index].construction.complete();},label);return index;
        };

        // Intended uninterrupted early-game build order. The layout is compact
        // enough for a starter colony but still requires real logistics travel.
        const auto solar=add_resource(ResourceKind::SolarPanel,{-30,0,-30},"solar collector");
        const auto battery=add_resource(ResourceKind::PowerStorage,{-10,0,-30},"battery");
        connect(UtilityEndpointKind::Resource,solar,UtilityEndpointKind::Resource,battery,"solar-battery supply link");
        const auto driller=add_resource(ResourceKind::WaterDriller,{10,0,-30},"water driller");
        connect(UtilityEndpointKind::Resource,battery,UtilityEndpointKind::Resource,driller,"battery-driller supply link");
        const auto tank=add_resource(ResourceKind::WaterStorage,{30,0,-30},"water storage");
        connect(UtilityEndpointKind::Resource,driller,UtilityEndpointKind::Resource,tank,"driller-tank supply link");
        app.update_supply_simulation(0.0f);
        auto p1=app.game_progress_snapshot();
        require(p1.stages.size()>=2 && p1.stages[0].complete,"uninterrupted bootstrap did not complete Stage 1");

        const auto airlock=add_habitat(HabitatKind::AirLock,{-10,0,5},"air lock");
        (void)airlock;
        PlacedHabitat oxygen_blueprint;oxygen_blueprint.kind=HabitatKind::OxygenGeneratorDome;oxygen_blueprint.size=HabitatSizeTier::Small;oxygen_blueprint.position={25,0,8};oxygen_blueprint.placement_sequence=sequence++;
        oxygen_blueprint.resource_plan=construction_costs::habitat(oxygen_blueprint.kind,oxygen_blueprint.size,app.habitat_models_.mesh(oxygen_blueprint));
        app.placed_habitats_.push_back(std::move(oxygen_blueprint));const auto oxygen=app.placed_habitats_.size()-1;
        // Wait until CR-01 has physically staged the complete oxygen-generator
        // bill of materials. Then place the final service connection: this
        // gives the otherwise-idle carrier real work away from BR-02's build
        // bay without pre-placing a utility blueprint through that bay.
        bool carrier_served_oxygen=false;const float staging_deadline=elapsed+420.0f;
        while(elapsed<staging_deadline){
            app.update_construction(1.0f/60.0f);elapsed+=1.0f/60.0f;assert_batteries();
            bool carrier_busy=false;
            for(const auto& r:app.builder_robots_) if(r.role==RobotRole::Carrier){
                if(r.target.same_as({ConstructionTargetKind::Habitat,oxygen})) carrier_served_oxygen=true;
                if(r.target.valid() || !r.cargo.empty() || r.action!=BuilderRobotAction::None) carrier_busy=true;
            }
            if(carrier_served_oxygen && !carrier_busy) break;
        }
        require(carrier_served_oxygen,"carrier never supplied oxygen-generator blueprint");
        const auto final_link=connect(UtilityEndpointKind::Resource,tank,UtilityEndpointKind::Habitat,oxygen,"resource-to-oxygen supply link",false);
        run_until([&]{return app.placed_habitats_[oxygen].construction.complete();},"oxygen generator");
        run_until([&]{return app.utility_network_.connections()[final_link].construction.complete();},"resource-to-oxygen supply link completion");
        app.update_supply_simulation(0.0f);
        const auto progress=app.game_progress_snapshot();
        require(progress.stages.size()>=2 && progress.stages[0].complete && progress.stages[1].complete,"completed uninterrupted bootstrap did not reach Stage 2");

        float builder_fraction=1.0f,carrier_fraction=1.0f;float builder_used=0,carrier_used=0;
        for(const auto& robot:app.builder_robots_){
            const float fraction=robot.battery_charge_kwh/std::max(0.001f,robot.battery_capacity_kwh);
            const float used=robot.battery_capacity_kwh-robot.battery_charge_kwh;
            if(robot.role==RobotRole::Constructor){builder_fraction=std::min(builder_fraction,fraction);builder_used=std::max(builder_used,used);}
            else {carrier_fraction=std::min(carrier_fraction,fraction);carrier_used=std::max(carrier_used,used);}
        }
        std::cout<<"Stage-2 bootstrap battery calibration: constructor used "<<builder_used<<" kWh ("<<builder_fraction*100.0f<<"% left), carrier used "<<carrier_used<<" kWh ("<<carrier_fraction*100.0f<<"% left) after "<<elapsed<<" s; no recharge\n";
        require(builder_fraction>=kRobotBatteryReserveFraction,"constructor Stage-2 reserve below 10%");
        require(carrier_fraction>=kRobotBatteryReserveFraction,"carrier Stage-2 reserve below 10%");
    }

    static void progression(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);app.debug_frame_enabled_=false;app.status_.reset();
        require(app.status_.crew_current()==0 && app.status_.crew_capacity()==0,"fresh colony already contains human crew");
        auto resource=[](ResourceKind kind,Vec3 p){PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;};
        auto habitat=[](HabitatKind kind,HabitatSizeTier size,Vec3 p){PlacedHabitat h;h.kind=kind;h.size=size;h.position=p;h.construction.step=kConstructionSteps;return h;};
        app.placed_resources_={resource(ResourceKind::SolarPanel,{-30,0,-20}),resource(ResourceKind::PowerStorage,{-15,0,-20}),resource(ResourceKind::WaterDriller,{0,0,-20}),resource(ResourceKind::WaterStorage,{15,0,-20})};
        const auto pipe_rr=[&](std::size_t a,std::size_t b){UtilityConnection u;u.endpoint_a.kind=u.endpoint_b.kind=UtilityEndpointKind::Resource;u.endpoint_a.index=a;u.endpoint_b.index=b;u.path={app.placed_resources_[a].position,app.placed_resources_[b].position};u.construction.step=u.construction.total_steps;app.utility_network_.mutable_connections().push_back(u);};
        pipe_rr(0,1);pipe_rr(1,2);pipe_rr(2,3);app.update_supply_simulation(0);
        auto p1=app.game_progress_snapshot();require(p1.stages.size()==3 && p1.stages[0].complete && p1.active_stage==2 && !p1.ready_for_human_crew,"Stage 1 resource/network progression did not complete");
        app.placed_habitats_={habitat(HabitatKind::AirLock,HabitatSizeTier::Small,{15,0,0}),habitat(HabitatKind::OxygenGeneratorDome,HabitatSizeTier::Small,{15,0,15})};
        UtilityConnection into;into.endpoint_a.kind=UtilityEndpointKind::Resource;into.endpoint_a.index=3;into.endpoint_b.kind=UtilityEndpointKind::Habitat;into.endpoint_b.index=0;into.path={{15,0,-20},{15,0,0}};into.construction.step=into.construction.total_steps;app.utility_network_.mutable_connections().push_back(into);
        UtilityConnection life;life.endpoint_a.kind=life.endpoint_b.kind=UtilityEndpointKind::Habitat;life.endpoint_a.index=0;life.endpoint_b.index=1;life.path={{15,0,0},{15,0,15}};life.construction.step=life.construction.total_steps;app.utility_network_.mutable_connections().push_back(life);app.update_supply_simulation(0);
        auto p2=app.game_progress_snapshot();require(p2.stages[1].complete && p2.active_stage==3,"Stage 2 air-lock/oxygen/network progression did not complete");
        app.placed_habitats_.push_back(habitat(HabitatKind::CantineDome,HabitatSizeTier::Small,{0,0,15}));
        app.placed_habitats_.push_back(habitat(HabitatKind::CrewQuarters,HabitatSizeTier::Mid,{-15,0,15}));
        app.placed_resources_.push_back(resource(ResourceKind::LandingPad,{-30,0,15}));
        UtilityConnection food;food.endpoint_a.kind=food.endpoint_b.kind=UtilityEndpointKind::Habitat;food.endpoint_a.index=1;food.endpoint_b.index=2;food.path={{15,0,15},{0,0,15}};food.construction.step=food.construction.total_steps;app.utility_network_.mutable_connections().push_back(food);
        UtilityConnection dorm;dorm.endpoint_a.kind=dorm.endpoint_b.kind=UtilityEndpointKind::Habitat;dorm.endpoint_a.index=2;dorm.endpoint_b.index=3;dorm.path={{0,0,15},{-15,0,15}};dorm.construction.step=dorm.construction.total_steps;app.utility_network_.mutable_connections().push_back(dorm);
        app.update_supply_simulation(0);require(app.status_.crew_capacity()==6,"crew capacity is not derived from completed dorm size");
        auto p3=app.game_progress_snapshot();require(!p3.ready_for_human_crew,"vacuum interiors prematurely completed crew-readiness milestone");
        for(auto& h:app.placed_habitats_)h.supply.oxygen_kg=h.supply.volume_m3*kOxygenKgPerM3;
        auto ready=app.game_progress_snapshot();require(ready.stages[2].complete && ready.ready_for_human_crew,"pressurized Stage 3 did not trigger Ready for human crew");
        std::cout<<"Three-stage data-driven colony progression, physical utility components, six-person dorm capacity and crew-readiness milestone passed\n";
    }

    static void debug_orders(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_.clear();
        app.builder_robots_[0].position={-20,0,-12};app.builder_robots_[1].position={-20,0,12};
        app.debug_frame_enabled_=false;require(!app.issue_debug_robot_move(0,{20,0,-12}),"manual robot order admitted outside debug mode");app.debug_frame_enabled_=true;
        require(!app.issue_debug_robot_move(0,{1000,0,0}),"off-map manual robot order admitted");
        PlacedResource obstacle;obstacle.kind=ResourceKind::PowerStorage;obstacle.construction.step=kConstructionSteps;obstacle.position={0,0,-12};obstacle.placement_sequence=1;app.placed_resources_={obstacle};
        require(app.issue_debug_robot_move(0,{20,0,-12}) && app.issue_debug_robot_move(1,{20,0,12}),"idle robots refused debug orders");
        for(int frame=0;frame<2400;++frame){app.update_construction(1.f/60);for(const auto& r:app.builder_robots_)require(r.safety.mode!=SafetyMode::Fault,"manual move bypassed safety or faulted");require(distance(app.builder_robots_[0].position,app.builder_robots_[1].position)>2.0f,"manual robots collided");}
        for(std::size_t i=0;i<2;++i){const auto& r=app.builder_robots_[i];require(r.debug_move_active && distance(r.position,r.debug_move_destination)<0.30f,"debug robot failed to reach/hold destination "+std::to_string(i)+" at "+std::to_string(r.position.x)+","+std::to_string(r.position.z));}
        std::array<Vec3,2> held{{app.builder_robots_[0].position,app.builder_robots_[1].position}};
        for(int frame=0;frame<180;++frame)app.update_construction(1.f/60);
        for(std::size_t i=0;i<2;++i)require(distance(held[i],app.builder_robots_[i].position)<0.05f,"custom placement did not hold");
        require(app.issue_debug_robot_move(0,{40,0,-12}) && app.issue_debug_robot_move(1,{40,0,12}),"idle robots could not retarget their custom orders");
        for(int frame=0;frame<20;++frame)app.update_construction(1.f/60);
        require(app.builder_robots_[0].current_speed_mps>0.1f && app.builder_robots_[1].current_speed_mps>0.1f,"manual preemption fixture never started moving");
        // Give the builder a fully staged real job, and the carrier an unstaged
        // job. Both must abandon the custom hold on the same dispatch slice.
        PlacedResource job;job.kind=ResourceKind::SolarPanel;job.position={0,0,32};job.placement_sequence=2;job.resource_plan.total_steps=kConstructionSteps;for(auto& cost:job.resource_plan.steps)cost.iron=10;job.site_buffer.iron=60;app.placed_resources_.push_back(job);
        job.position={24,0,-32};job.placement_sequence=3;job.site_buffer={};app.placed_resources_.push_back(job);app.ground_resource_piles_={{MaterialResource::Iron,1000,{-12,0,32},0.4f,1}};
        app.update_construction(1.f/60);
        for(std::size_t i=0;i<2;++i)require(!app.builder_robots_[i].debug_move_active && app.builder_robots_[i].target.valid(),"real task did not preempt manual order "+std::to_string(i));
        require(!app.issue_debug_robot_move(0,{30,0,0}),"working robot accepted debug move");
        GameApp disabled(engine);disabled.playfield_.rebuild(128);disabled.builder_robots_.resize(1);disabled.builder_robots_[0].position={-30,0,-30};disabled.debug_frame_enabled_=true;require(disabled.issue_debug_robot_move(0,{-20,0,-30}),"debug fixture order rejected");disabled.debug_frame_enabled_=false;disabled.update_construction(1.f/60);require(!disabled.builder_robots_[0].debug_move_active,"disabled debug mode retained order");
        std::cout<<"Two debug robots route around obstacles, hold destination, reject busy/off-map orders and yield to real builder/carrier work\n";
    }
    static void build_priority(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_={{MaterialResource::Iron,1000,{-30,0,30},0.4f,1}};
        for(int i=0;i<3;++i){PlacedResource job;job.kind=ResourceKind::SolarPanel;job.position={(float)(i-1)*24,0,-20};job.placement_sequence=i+1;job.resource_plan.total_steps=kConstructionSteps;for(auto& step:job.resource_plan.steps)step.iron=10;app.placed_resources_.push_back(job);}
        app.builder_robots_[0].position={24,0,20};app.builder_robots_[1].position={-24,0,20};
        app.toggle_construction_priority({ConstructionTargetKind::Resource,2});app.toggle_construction_priority({ConstructionTargetKind::Resource,1});
        require(app.construction_priority_rank({ConstructionTargetKind::Resource,2})==0 && app.construction_priority_rank({ConstructionTargetKind::Resource,1})==1,"first queued priority did not stay first");
        app.update_construction(1.f/60);require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,2}),"carrier FIFO ignored priority");
        app.toggle_construction_priority({ConstructionTargetKind::Resource,2});app.update_construction(1.f/60);require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,1}),"empty carrier failed to follow restored queue order");
        // Builder dispatch has its own distance/cooperation scoring; priority
        // must dominate it once the real material/dependency gates are met.
        GameApp builder(engine);builder.playfield_.rebuild(128);builder.starting_robot_count_=1;builder.starting_carrier_count_=0;builder.reset_builder_robot_and_starter_stash();builder.ground_resource_piles_.clear();builder.placed_resources_=app.placed_resources_;for(auto& job:builder.placed_resources_)job.site_buffer.iron=60;
        builder.builder_robots_[0].position={-24,0,-5};builder.toggle_construction_priority({ConstructionTargetKind::Resource,2});builder.update_construction(1.f/60);require(builder.builder_robots_[0].target.same_as({ConstructionTargetKind::Resource,2}),"builder ignored boosted distant target");
        // Stable identity survives deletion/reindex and save round trips.
        app.next_blueprint_placement_sequence_=4;std::string error;require(app.save_game_to_name("priority-regression",error),error);app.refresh_save_games();std::string path;for(const auto& entry:app.save_games_)if(entry.name=="priority-regression")path=entry.path;
        GameApp restored(engine);require(restored.load_game_from_file(path,error),error);require(restored.construction_priority_==app.construction_priority_,"priority lost on save/load");restored.placed_resources_.erase(restored.placed_resources_.begin());require(restored.construction_priority_rank({ConstructionTargetKind::Resource,0})==0,"priority followed vector index instead of identity");
        restored.placed_resources_[0].construction.step=kConstructionSteps;const auto priorities=restored.construction_priority_;restored.toggle_construction_priority({ConstructionTargetKind::Resource,0});require(restored.construction_priority_==priorities,"completed structure accepted promotion");

        std::cout<<"Builder/carrier priority dispatch, restore-normal ordering, stable IDs, schema-19 roundtrip passed\n";
    }
    static void airlock_cycle(){
        eng::Application engine;int cases=0;
        for(auto size:{HabitatSizeTier::Small,HabitatSizeTier::Standard})for(float rotation:{0.f,37.f,90.f,180.f})for(int inside:{-1,1}){
            GameApp app(engine);app.playfield_.rebuild(128);app.corridor_network_.set_playfield_half_extent(64);
            PlacedHabitat air;air.kind=HabitatKind::AirLock;air.size=size;air.rotation_degrees=rotation;air.construction.step=kConstructionSteps;air.placement_sequence=1;app.placed_habitats_={air};
            const auto& mesh=app.habitat_models_.mesh(air);require(mesh.entrances.empty() && mesh.entrance_access.empty(),"air lock retained authored entrance");const float end=mesh.footprint_x*0.5f;
            const auto world=[&](float x,float z=0.f){return world_math::rotate_y(Vec3{x,mesh.walkable_floor_y,z},rotation);};
            const CorridorBuildAnchor start{CorridorEndpointKind::Habitat,0,world(inside*end)},finish{CorridorEndpointKind::OpenDoor,0,world(inside*(end+14))};
            require(app.corridor_network_.build_corridor(start,finish,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"airlock cycle corridor rejected");
            app.corridor_network_.mutable_connections()[0].construction.step=kCorridorConstructionSteps;
            require(app.airlock_inside_sign(0)==inside,"air-lock inside/outside orientation reversed");
            require(app.habitat_doors(0).size()==4,"three chambers need four seals");
            app.first_person_mode_=GameApp::FirstPersonMode::Active;
            auto pose=world(-inside*(end+1.8f));pose.y=app.first_person_floor_height(pose);app.first_person_.spawn(pose,0);
            float min_pressure=1,max_pressure=0;int equalizing=0;
            const auto travel=[&](float destination,int direction){
                const auto forward=world_math::rotate_y(Vec3{(float)direction,0,0},rotation);
                const float yaw=std::atan2(forward.x,-forward.z)*180.f/3.14159265358979323846f;
                auto current=app.first_person_.feet_position();app.first_person_.spawn(current,yaw);app.input_={};app.input_.key_w_down=true;
                for(int frame=0;frame<2400;++frame){
                    current=app.first_person_.feet_position();const float x=world_math::rotate_y(current,-rotation).x;
                    if((destination-x)*direction<0.045f)return;
                    const auto old_pressure=app.airlock_cycles_.empty()?1:app.airlock_cycles_[0].pressure;
                    bool previously_sealed=true;for(const auto& door:app.pressure_doors_)previously_sealed&=pressure_door::sealed(door.open_fraction);
                    app.update_pressure_doors(0.05f);
                    const auto& cycle=app.airlock_cycles_[0];min_pressure=std::min(min_pressure,cycle.pressure);max_pressure=std::max(max_pressure,cycle.pressure);equalizing+=cycle.equalizing;
                    if(std::abs(cycle.pressure-old_pressure)>1e-6f)require(previously_sealed,"pressure changed before all doors sealed");
                    int open=0;for(const auto& door:app.pressure_doors_){
                        if(door.open_fraction>0.0001f){++open;if(airlock::internal(door.door)){
                            const float dx=world_math::rotate_y(door.door.wall_position,-rotation).x;
                            require(std::abs(cycle.pressure-(dx*inside>0?1.f:0.f))<0.002f,"internal door opened across a pressure difference");
                        }}
                    }
                    require(open<=1,"air lock opened more than one seal");
                    auto next=world(x+direction*0.04f);next.y=app.first_person_floor_height(next);
                    if(app.first_person_move_allowed(current,next))app.first_person_.set_feet_position(next);
                }
                const auto x=world_math::rotate_y(app.first_person_.feet_position(),-rotation).x;
                throw std::runtime_error("airlock traversal stuck at "+std::to_string(x)+" destination "+std::to_string(destination)+" / "+airlock::phase(app.airlock_cycles_[0]));
            };
            travel(inside*(end+5),inside); // outside -> isolation -> pressure -> inside
            travel(-inside*(end+1.8f),-inside); // exact reverse, including center decompression
            // Reverse midway through the center chamber, then make a full pass.
            travel(0,inside);travel(-inside*(end+1.8f),-inside);travel(inside*(end+5),inside);
            require(min_pressure<0.001f && max_pressure>0.999f && equalizing>20,"pressure cycle was skipped or instantaneous");
            const auto bulkheads=airlock::partitions(air,mesh);
            for(const auto& d:bulkheads){
                const Vec3 n=d.normal,side{-n.z,0,n.x};const auto base=d.wall_position;
                require(airlock::bulkhead_blocks(d,0,base-n,base+n,geometry::kHumanBodyRadius,geometry::kHumanHeight),"closed seal traversable");
                require(!airlock::bulkhead_blocks(d,1,base-n,base+n,geometry::kHumanBodyRadius,geometry::kHumanHeight),"open central aisle blocked");
                require(airlock::bulkhead_blocks(d,1,base-n+side,base+n+side,geometry::kHumanBodyRadius,geometry::kHumanHeight),"avatar walked around partition");
                require(!airlock::bulkhead_blocks(d,0,base-n+side*12,base+n+side*12,geometry::kHumanBodyRadius,geometry::kHumanHeight),"partition became a map-wide invisible wall");
                require(airlock::bulkhead_blocks(d,1,base-n+Vec3{0,1.1f,0},base+n+Vec3{0,1.1f,0},geometry::kHumanBodyRadius,geometry::kHumanHeight),"avatar jumped over partition aperture");
            }
            // Trace the actual submitted triangles, independently of collision:
            // closed leaves seal the aperture, open leaves clear it, and the
            // pressure bulkhead remains solid beside the human-sized opening.
            for(const auto& d:bulkheads){
                for(auto& state:app.pressure_doors_)state.open_fraction=0;
                const auto point=d.wall_position+Vec3{0,1.1f,0};GeometryRecorder draw;app.render_habitats(draw);
                require(segment_hits_mesh(point-d.normal*0.35f,point+d.normal*0.35f,draw.triangles),"rendered internal door does not seal");
                for(auto& state:app.pressure_doors_)if(distance(state.door.wall_position,d.wall_position)<0.01f)state.open_fraction=1;
                draw.triangles.clear();app.render_habitats(draw);
                require(!segment_hits_mesh(point-d.normal*0.35f,point+d.normal*0.35f,draw.triangles),"rendered internal door does not retract");
                const Vec3 side{-d.normal.z,0,d.normal.x};
                require(segment_hits_mesh(point+side*1.1f-d.normal*0.35f,point+side*1.1f+d.normal*0.35f,draw.triangles),"rendered partition missing beside aperture");
            }
            const float partition=airlock::partition_x(mesh);auto far_a=world(partition-1,12),far_b=world(partition+1,12);far_a.y=far_b.y=0;
            require(app.first_person_move_allowed(far_a,far_b),"internal partition blocks distant exterior walking");
            ++cases;
        }
        std::cout<<cases<<" rotated small/standard three-chamber air locks: physical traversal both ways, center retreat, noninstant pressure matching, four-door interlock and bulkhead collision passed\n";
    }

    static void airlock_connections(){
        eng::Application engine;int cases=0;
        for(auto size:{HabitatSizeTier::Small,HabitatSizeTier::Standard})for(float rotation:{0.f,37.f,90.f,180.f})for(int end:{-1,1}){
            GameApp app(engine);app.playfield_.rebuild(128);app.corridor_network_.set_playfield_half_extent(64);app.utility_network_.set_playfield_half_extent(64);
            PlacedHabitat air;air.kind=HabitatKind::AirLock;air.size=size;air.position={3,0,4};air.rotation_degrees=rotation;air.construction.step=kConstructionSteps;air.placement_sequence=1;app.placed_habitats_={air};
            const auto& mesh=app.habitat_models_.mesh(air);require(mesh.entrances.empty() && mesh.entrance_access.empty(),"air lock retained authored entrance");
            const float half=mesh.footprint_x*0.5f;
            const Vec3 local_hatch{(float)end*half,mesh.walkable_floor_y,0},local_normal{(float)end,0,0};
            const Vec3 local_outer{-(float)end*half,mesh.walkable_floor_y,0},local_outer_normal{-(float)end,0,0};
            const Vec3 hatch=air.position+world_math::rotate_y(local_hatch,rotation),normal=world_math::rotate_y(local_normal,rotation);
            const Vec3 outer=air.position+world_math::rotate_y(local_outer,rotation),outer_normal=world_math::rotate_y(local_outer_normal,rotation);
            const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,hatch},b{CorridorEndpointKind::OpenDoor,0,hatch+normal*14},blocked{CorridorEndpointKind::Habitat,0,outer},other{CorridorEndpointKind::OpenDoor,0,outer+outer_normal*14};
            require(app.corridor_network_.build_corridor(a,b,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"first air-lock hatch rejected");
            auto& root=app.corridor_network_.mutable_connections()[0];root.placement_sequence=2;
            require(root.endpoint_a.door.generated && root.endpoint_a.door.entrance_index<0,"air lock did not create its hatch from the connection");
            require(!app.corridor_network_.preview_anchor(blocked,other.position,app.placed_habitats_,app.habitat_models_),"opposite hatch preview allowed while first corridor was a blueprint");
            require(!app.corridor_network_.build_corridor(blocked,other,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"second air-lock end built");
            require(!app.corridor_network_.preview_built_corridor(other,blocked,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"blocked hatch accepted as destination");
            auto reused=app.corridor_network_.preview_anchor(a,b.position,app.placed_habitats_,app.habitat_models_);require(reused && reused->kind==CorridorEndpointKind::Corridor,"used hatch lost shared-trunk branching");
            require(!airlock_connections::allows(0,outer,app.placed_habitats_,app.corridor_network_.connections()),"passenger-side guard accepted opposite exterior side");
            app.corridor_network_.mutable_connections()[0].construction.step=kCorridorConstructionSteps;
            require(!app.habitat_exterior_access_enabled(0,0),"air lock exposed legacy authored surface access");
            const auto air_doors=app.habitat_doors(0);require(air_doors.size()==4,"air lock no longer has four generated/structural pressure doors");
            bool mirrored=false;for(const auto& d:air_doors)mirrored=mirrored || eng::dot(d.normal,outer_normal)>0.95f;
            require(mirrored,"inside connection did not create the opposite exterior air-lock seal");
            WalkwayBuildAnchor w{WalkwayEndpointKind::Habitat,0,outer},wo{WalkwayEndpointKind::Open,0,outer+outer_normal*14};
            require(!app.walkway_network_.preview_connection(w,wo,app.corridor_network_,app.utility_network_,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"walkway occupied the reserved exterior hatch");
            app.corridor_network_.remove_corridor(0);
            require(app.corridor_network_.preview_anchor(blocked,other.position,app.placed_habitats_,app.habitat_models_).has_value(),"removing final connection did not release side");
            UtilityBuildAnchor u{UtilityEndpointKind::Habitat,0,hatch},uo{UtilityEndpointKind::Open,0,hatch+normal*14},ub{UtilityEndpointKind::Habitat,0,outer},ubend{UtilityEndpointKind::Open,0,outer+outer_normal*14};
            require(app.utility_network_.build_connection(UtilityKind::SupplyChannel,u,uo,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"first air-lock utility side rejected");
            require(app.utility_network_.preview_anchor(UtilityKind::SupplyChannel,ub,ubend.hit,app.placed_habitats_,app.habitat_models_,{},app.resource_models_).has_value(),"opposite utility port preview rejected");
            require(app.utility_network_.build_connection(UtilityKind::SupplyChannel,ubend,ub,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"opposite utility port rejected");
            // Both lateral hull sides must also accept independent supply ports.
            for(int lateral:{-1,1}){
                const Vec3 local_side{0,mesh.walkable_floor_y,(float)lateral*mesh.footprint_z*0.5f};
                const Vec3 local_side_normal{0,0,(float)lateral};
                const Vec3 side_point=air.position+world_math::rotate_y(local_side,rotation);
                const Vec3 side_normal=world_math::rotate_y(local_side_normal,rotation);
                UtilityBuildAnchor us{UtilityEndpointKind::Habitat,0,side_point},use{UtilityEndpointKind::Open,0,side_point+side_normal*14};
                require(app.utility_network_.preview_anchor(UtilityKind::SupplyChannel,us,use.hit,app.placed_habitats_,app.habitat_models_,{},app.resource_models_).has_value(),"lateral air-lock utility port preview rejected");
                require(app.utility_network_.build_connection(UtilityKind::SupplyChannel,us,use,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"lateral air-lock utility port rejected");
            }
            // Player-facing cursor snapping must preserve all four Air Lock sides.
            // Earlier code rewrote every cursor hit to +/- local X, so direct
            // network tests passed while lateral ports were impossible in-game.
            for(const Vec3 local_cursor:std::array<Vec3,4>{
                    Vec3{ mesh.footprint_x*0.49f,0,0},Vec3{-mesh.footprint_x*0.49f,0,0},
                    Vec3{0,0, mesh.footprint_z*0.49f},Vec3{0,0,-mesh.footprint_z*0.49f}}){
                const Vec3 cursor=air.position+world_math::rotate_y(local_cursor,rotation);
                const auto selected=app.utility_anchor_at(cursor,UtilityKind::SupplyChannel);
                require(selected && selected->kind==UtilityEndpointKind::Habitat && selected->index==0,"Air Lock side was not selectable through player supply-channel snapping");
                const Vec3 away=cursor+world_math::rotate_y(eng::normalize(local_cursor)*14.0f,rotation);
                const auto port=app.utility_network_.preview_anchor(UtilityKind::SupplyChannel,*selected,away,app.placed_habitats_,app.habitat_models_,{},app.resource_models_);
                require(port.has_value(),"Air Lock cursor-selected side did not resolve to a physical supply port");
                const Vec3 lp=world_math::rotate_y(port->position-air.position,-rotation);
                if(std::abs(local_cursor.x)>std::abs(local_cursor.z)) require(lp.x*local_cursor.x>0,"Air Lock X-side cursor jumped to another side");
                else require(lp.z*local_cursor.z>0,"Air Lock lateral cursor jumped to an end face");
            }
            // Utility placement must not reserve or change the passenger corridor side.
            require(airlock_connections::allows(0,outer,app.placed_habitats_,{},app.utility_network_.connections()),"utility port incorrectly reserved an air-lock passenger side");
            app.utility_network_.mutable_connections().clear();
            // Cursor positions on the curved cylinder must reuse a real hatch.
            CorridorBuildAnchor side=a;side.position=air.position+world_math::rotate_y({(float)end*0.8f,0,4},rotation);
            auto snapped=app.corridor_network_.preview_anchor(side,b.position,app.placed_habitats_,app.habitat_models_);require(!snapped,"air lock allowed an arbitrary side entrance");
            ++cases;
        }
        std::cout<<cases<<" air-lock side/rotation/size cases: passenger-side reservations, all-side utility ports, shared-trunk branches, removal and exterior access passed\n";
    }

    static void hydroponics(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();
        auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        PlacedResource solar;solar.kind=ResourceKind::SolarPanel;solar.position={-20,0,0};solar.construction.step=kConstructionSteps;
        PlacedResource battery;battery.kind=ResourceKind::PowerStorage;battery.position={-10,0,0};battery.construction.step=kConstructionSteps;battery.supply.energy_kwh=power_storage_capacity_kwh(ResourceSizeTier::Standard);
        PlacedResource tank;tank.kind=ResourceKind::WaterStorage;tank.position={0,0,0};tank.construction.step=kConstructionSteps;tank.supply.water_litres=2000;
        app.placed_resources_={solar,battery,tank};
        PlacedHabitat bio;bio.kind=HabitatKind::BioDome;bio.size=HabitatSizeTier::Standard;bio.position={15,0,0};bio.construction.step=kConstructionSteps;
        for(const auto& fresh:bio.hydroponics) require(fresh.plant==HydroponicPlant::Empty && fresh.growth_fraction==0.0,"new Bio Dome bay did not start empty");
        bio.hydroponics[0].plant=HydroponicPlant::Potato;
        app.placed_habitats_={bio};
        auto connect=[&](UtilityEndpointKind ka,std::size_t a,UtilityEndpointKind kb,std::size_t b,Vec3 pa,Vec3 pb){
            UtilityConnection u;u.endpoint_a.kind=ka;u.endpoint_a.index=a;u.endpoint_b.kind=kb;u.endpoint_b.index=b;u.path={pa,pb};u.construction.step=u.construction.total_steps;app.utility_network_.mutable_connections().push_back(u);
        };
        connect(UtilityEndpointKind::Resource,0,UtilityEndpointKind::Resource,1,solar.position,battery.position);
        connect(UtilityEndpointKind::Resource,1,UtilityEndpointKind::Resource,2,battery.position,tank.position);
        connect(UtilityEndpointKind::Resource,2,UtilityEndpointKind::Habitat,0,tank.position,bio.position);
        app.update_supply_simulation(0.0f);
        app.placed_habitats_[0].supply.oxygen_kg=app.placed_habitats_[0].supply.volume_m3*kOxygenKgPerM3;
        const double water_before=app.placed_resources_[2].supply.water_litres;
        app.update_supply_simulation(24.0f);
        auto& bay=app.placed_habitats_[0].hydroponics[0];
        require(bay.growth_fraction>0.02 && bay.growth_fraction<0.06,"potato bay did not advance at the expected crop-specific rate during the light phase");
        require(app.placed_resources_[2].supply.water_litres<water_before,"hydroponics growth did not consume connected water");
        require(app.placed_habitats_[0].supply.power_demand_kw>1.5,"hydroponics grow lights were not added during the light phase");
        require(app.placed_habitats_[0].supply.water_demand_litres_per_day>=3.19,"hydroponics water demand telemetry missing during the light phase");

        const double light_growth=bay.growth_fraction;
        clock.hour=23;app.status_.restore(clock);app.update_supply_simulation(2.0f);
        require(std::abs(bay.growth_fraction-light_growth)<1e-9,"crop grew during the greenhouse dark phase");
        require(std::abs(app.placed_habitats_[0].supply.power_demand_kw-1.5)<1e-6,"grow-light demand remained active during the dark phase");
        require(app.placed_habitats_[0].supply.water_demand_litres_per_day<1e-9,"crop water demand remained active during the dark phase");

        clock.hour=12;app.status_.restore(clock);app.placed_habitats_[0].supply.oxygen_kg=0.0;
        const double no_air_growth=bay.growth_fraction;app.update_supply_simulation(2.0f);
        require(std::abs(bay.growth_fraction-no_air_growth)<1e-9,"crop grew without breathable air");
        app.placed_habitats_[0].supply.oxygen_kg=app.placed_habitats_[0].supply.volume_m3*kOxygenKgPerM3;

        bay.growth_fraction=0.999;
        const auto food_before=app.resources_.amount(MaterialResource::Food);
        app.update_supply_simulation(1.0f);
        require(bay.harvests==1 && bay.growth_fraction<0.01,"mature crop did not auto-harvest and restart growth");
        require(app.resources_.amount(MaterialResource::Food)==food_before+hydroponic_profile(HydroponicPlant::Potato).harvest_food_kg,"harvest did not enter Food inventory");
        bool food_pile=false;for(const auto& pile:app.ground_resource_piles_)if(pile.resource==MaterialResource::Food && pile.amount==hydroponic_profile(HydroponicPlant::Potato).harvest_food_kg)food_pile=true;
        require(food_pile,"hydroponics harvest did not create a physical Food package");
        const double growth_before_power_loss=bay.growth_fraction;
        app.placed_resources_[0].construction.step=0;app.placed_resources_[1].supply.energy_kwh=0;
        app.update_supply_simulation(2.0f);
        require(std::abs(bay.growth_fraction-growth_before_power_loss)<1e-9,"crop grew without electrical power");
        app.placed_resources_[0].construction.step=kConstructionSteps;clock.hour=12;app.status_.restore(clock);
        bay.plant=HydroponicPlant::Soybean;bay.growth_fraction=0.42;bay.harvests=7;
        std::string error;require(app.save_game_to_name("hydroponics-regression",error),"hydroponics save failed: "+error);app.refresh_save_games();
        std::string path;for(const auto& e:app.save_games_)if(e.name=="hydroponics-regression")path=e.path;require(!path.empty(),"hydroponics save was not listed");
        GameApp loaded(engine);require(loaded.load_game_from_file(path,error),"hydroponics load failed: "+error);
        const auto& restored=loaded.placed_habitats_[0].hydroponics[0];
        require(restored.plant==HydroponicPlant::Soybean && std::abs(restored.growth_fraction-0.42)<1e-9 && restored.harvests==7,"hydroponics bay state changed across save/load");
        std::filesystem::remove(path);
        require(hydroponics_bay_count(HabitatSizeTier::Small)==2 && hydroponics_bay_count(HabitatSizeTier::Standard)==4 && hydroponics_bay_count(HabitatSizeTier::Mid)==6 && hydroponics_bay_count(HabitatSizeTier::Big)==8,"Bio Dome bay count does not scale with dome size");
        require(hydroponics_lights_on(6.0f) && hydroponics_lights_on(21.99f) && !hydroponics_lights_on(22.0f) && !hydroponics_lights_on(5.99f),"Bio Dome 16/8 photoperiod boundaries are wrong");
        std::cout<<"Bio Dome empty defaults, 16/8 light cycle, air/power/water dependency, harvest and schema-23 persistence passed\n";
    }

    static void queue_and_flow(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=1;app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_={{MaterialResource::Iron,1000,{-30,0,30},0.4f,1}};
        for(int i=0;i<3;++i){PlacedResource r;r.kind=ResourceKind::SolarPanel;r.position={(float)(i-1)*20,0,-20};r.placement_sequence=i+1;r.resource_plan.total_steps=kConstructionSteps;for(auto& cost:r.resource_plan.steps)cost.iron=10;app.placed_resources_.push_back(r);}
        app.builder_robots_[0].position={-20,0,20};app.builder_robots_[1].position={20,0,20};
        app.toggle_construction_priority({ConstructionTargetKind::Resource,1});app.toggle_construction_priority({ConstructionTargetKind::Resource,2});
        app.placed_resources_[1].site_buffer.iron=60;app.update_construction(1.f/60);
        require(app.builder_robots_[0].target.same_as({ConstructionTargetKind::Resource,1}),"builder skipped FIFO head");
        require(app.construction_dispatch_rank({ConstructionTargetKind::Resource,2})==3 && !app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,2}),"fully supplied but unfinished head released follower early");
        app.placed_resources_[1].construction.step=kConstructionSteps;app.update_construction(1.f/60);require(app.construction_priority_.size()==1 && app.construction_priority_.front()==3,"finished FIFO head did not leave queue");require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,2}),"next FIFO item did not activate");
        app.toggle_construction_priority({ConstructionTargetKind::Resource,2});require(app.construction_priority_.empty(),"remove priority did not remove queue item");
        for(int i=0;i<2;++i){PlacedResource r;r.kind=ResourceKind::PowerStorage;r.position={(float)(i*40-20),0,0};r.construction.step=kConstructionSteps;r.placement_sequence=i+4;app.placed_resources_.push_back(r);}
        UtilityConnection pipe;pipe.endpoint_a.kind=pipe.endpoint_b.kind=UtilityEndpointKind::Resource;pipe.endpoint_a.index=3;pipe.endpoint_b.index=4;pipe.path={{-20,0,0},{20,0,0}};pipe.placement_sequence=6;pipe.resource_plan.total_steps=pipe.construction.total_steps;for(auto& cost:pipe.resource_plan.steps)cost.iron=10;app.utility_network_.mutable_connections().push_back(pipe);
        app.toggle_construction_priority({ConstructionTargetKind::Resource,0});app.update_construction(1.f/60);require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Utility,0}),"ready pipe did not precede manually prioritized structure");
        require(app.construction_ready_connection({ConstructionTargetKind::Utility,0}),"ready pipe not recognized");app.placed_resources_[4].construction.step=0;require(!app.construction_ready_connection({ConstructionTargetKind::Utility,0}),"pipe to unbuilt endpoint auto-prioritized");app.placed_resources_[4].construction.step=kConstructionSteps;
        CorridorConnection corridor;corridor.endpoint_a.kind=corridor.endpoint_b.kind=CorridorEndpointKind::Resource;corridor.endpoint_a.index=3;corridor.endpoint_b.index=4;corridor.placement_sequence=7;corridor.path={{-20,0,0},{20,0,0}};app.corridor_network_.mutable_connections().push_back(corridor);require(app.construction_ready_connection({ConstructionTargetKind::Corridor,0}),"ready corridor not prioritized");
        // A queued dependency must not deadlock the first item.
        app.construction_priority_={6,4};app.placed_resources_[3].construction.step=0;require(app.construction_dispatch_rank({ConstructionTargetKind::Resource,3})==1,"queued prerequisite of FIFO head was blocked");
        // Current-schema roundtrip must retain FIFO insertion order. Schema-18
        // reversal remains handled by the loader; do not fake an old schema by
        // changing only the file header now that schema 20 has a larger SupplyState.
        app.construction_priority_={3,1};std::string error;require(app.save_game_to_name("queue-current-regression",error),error);app.refresh_save_games();std::string path;for(const auto& entry:app.save_games_)if(entry.name=="queue-current-regression")path=entry.path;
        GameApp loaded(engine);require(loaded.load_game_from_file(path,error),error);require(loaded.construction_priority_==std::vector<std::uint64_t>({3,1}),"FIFO priority order changed across current save/load");
        GameApp flow(engine);auto clock=flow.status_.state();clock.hour=12;clock.crew_current=0;flow.status_.restore(clock);
        for(auto kind:{ResourceKind::SolarPanel,ResourceKind::PowerStorage,ResourceKind::WaterDriller,ResourceKind::WaterStorage}){PlacedResource r;r.kind=kind;r.construction.step=kConstructionSteps;r.position={(float)flow.placed_resources_.size()*10,0,0};flow.placed_resources_.push_back(r);}
        for(std::size_t i=1;i<4;++i){UtilityConnection u;u.endpoint_a.kind=u.endpoint_b.kind=UtilityEndpointKind::Resource;u.endpoint_a.index=i-1;u.endpoint_b.index=i;u.path={flow.placed_resources_[i-1].position,flow.placed_resources_[i].position};u.construction.step=u.construction.total_steps;flow.utility_network_.mutable_connections().push_back(u);}
        flow.update_supply_simulation(10);require(std::abs(flow.battery_net_kw_-9.5)<1e-6 && std::abs(flow.supply_power_load_kw_-4)<1e-6,"measured battery rate or electrical load wrong");require(std::abs(flow.water_net_litres_per_day_-500)<1e-6,"measured tank filling rate wrong");
        clock.hour=0;flow.status_.restore(clock);
        const double solar_buffer=flow.placed_resources_[0].supply.energy_kwh;
        flow.update_supply_simulation(1);
        require(std::abs(flow.battery_net_kw_)<1e-6 && std::abs(flow.supply_power_delivered_kw_-4)<1e-6 &&
            std::abs(flow.placed_resources_[0].supply.energy_kwh-solar_buffer+.4)<1e-6,
            "solar buffer should supply nighttime load ahead of the battery");
        flow.placed_resources_[0].supply.energy_kwh=0;
        flow.update_supply_simulation(1);
        require(std::abs(flow.battery_net_kw_+4)<1e-6,"night discharge rate wrong");
        for(int i=0;i<2;++i){PlacedHabitat h;h.kind=i?HabitatKind::CrewQuarters:HabitatKind::OxygenGeneratorDome;h.position={(float)i*20,0,20};h.construction.step=kConstructionSteps;flow.placed_habitats_.push_back(h);UtilityConnection u;u.endpoint_a.kind=UtilityEndpointKind::Resource;u.endpoint_a.index=3;u.endpoint_b.kind=UtilityEndpointKind::Habitat;u.endpoint_b.index=i;u.path={{30,0,0},h.position};u.construction.step=u.construction.total_steps;flow.utility_network_.mutable_connections().push_back(u);}
        clock.hour=12;clock.crew_current=6;flow.status_.restore(clock);flow.placed_resources_[2].construction.step=0;flow.placed_resources_[3].supply.water_litres=100;flow.update_supply_simulation(1);
        require(std::abs(flow.water_net_litres_per_day_+44.25)<1e-6 && std::abs(flow.supply_water_load_litres_per_day_-44.25)<1e-6,"tank consumption/doubled electrolysis rate wrong");
        const auto water=flow.placed_resources_[3].supply.water_litres;flow.update_supply_simulation(0);require(flow.battery_net_kw_==0 && flow.water_net_litres_per_day_==0 && flow.placed_resources_[3].supply.water_litres==water,"paused telemetry consumed resources or kept stale rate");
        std::cout<<"FIFO until completion/removal, ready corridor/pipe precedence, prerequisite escape, save roundtrip and measured charge/fill/drain/load rates passed\n";
    }
    static void power_switches(){
        eng::Application engine;
        GameApp app(engine);
        app.active_game_phase_=GameApp::GamePhase::Debug;
        app.playfield_.rebuild(128);
        app.utility_network_.set_playfield_half_extent(64);
        auto clock=app.status_.state();
        clock.hour=12;
        clock.crew_current=0;
        app.status_.restore(clock);

        const auto add_resource=[&](ResourceKind kind,Vec3 position){
            PlacedResource item;
            item.kind=kind;
            item.position=position;
            item.construction.step=kConstructionSteps;
            item.placement_sequence=app.placed_resources_.size()+1;
            app.placed_resources_.push_back(item);
        };
        add_resource(ResourceKind::SolarPanel,{-25,0,0});
        add_resource(ResourceKind::PowerStorage,{-25,0,-22});
        add_resource(ResourceKind::OreDriller,{-5,0,0});
        for(auto kind:{HabitatKind::ProcessingFacility,HabitatKind::Factory}){
            PlacedHabitat item;
            item.kind=kind;
            item.position={18,0,(float)app.placed_habitats_.size()*22};
            item.construction.step=kConstructionSteps;
            item.placement_sequence=app.placed_habitats_.size()+4;
            app.placed_habitats_.push_back(item);
        }

        const auto connect=[&](UtilityBuildAnchor a,UtilityBuildAnchor b){
            require(app.utility_network_.build_connection(UtilityKind::SupplyChannel,a,b,
                app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_),
                "power-switch fixture link rejected");
            auto& link=app.utility_network_.mutable_connections().back();
            link.construction.step=link.construction.total_steps;
            link.placement_sequence=6+app.utility_network_.connections().size();
        };
        const auto resource=[&](std::size_t i){return UtilityBuildAnchor{UtilityEndpointKind::Resource,i,app.placed_resources_[i].position};};
        const auto habitat=[&](std::size_t i){return UtilityBuildAnchor{UtilityEndpointKind::Habitat,i,app.placed_habitats_[i].position};};
        connect(resource(0),resource(1));
        connect(resource(0),resource(2));
        connect(resource(2),habitat(0));
        connect(habitat(0),habitat(1));

        auto& solar=app.placed_resources_[0];
        auto& battery=app.placed_resources_[1];
        auto& drill=app.placed_resources_[2];
        auto& processor=app.placed_habitats_[0];
        const auto near=[](double a,double b){return std::abs(a-b)<1e-6;};
        app.update_supply_simulation(1);
        require(near(app.supply_power_delivered_kw_,16) && near(app.supply_power_load_kw_,18) &&
            near(drill.supply.power_fraction,16.0/18.0) && near(app.battery_net_kw_,0),
            "brownout telemetry hid available power or charged without surplus");

        processor.power_enabled=false;
        auto services=app.update_supply_simulation(1);
        require(near(app.supply_power_load_kw_,12) && near(app.supply_power_delivered_kw_,12) &&
            near(app.battery_net_kw_,3.8) && processor.supply.power_fraction==0 &&
            !services.habitats[0].power && services.habitats[1].power,
            "load shedding failed or powered-off habitat cut power to downstream factory");

        solar.power_enabled=false;
        battery.power_enabled=false;
        const double stored=battery.supply.energy_kwh;
        app.update_supply_simulation(1);
        require(near(app.supply_power_delivered_kw_,0) && near(app.battery_net_kw_,0) &&
            near(battery.supply.energy_kwh,stored) && drill.supply.power_fraction==0,
            "disabled generation or storage supplied electricity");

        std::string error;
        const auto save=std::filesystem::path(app.save_game_directory())/"power-switch-regression.gmsave";
        require(app.save_game_to_name("power-switch-regression",error),error);
        GameApp loaded(engine);
        const bool restored=loaded.load_game_from_file(save.string(),error);
        std::filesystem::remove(save);
        require(restored,error);
        require(!loaded.placed_resources_[0].power_enabled && !loaded.placed_resources_[1].power_enabled &&
            loaded.placed_resources_[2].power_enabled && !loaded.placed_habitats_[0].power_enabled &&
            loaded.placed_habitats_[1].power_enabled && near(loaded.placed_resources_[1].supply.energy_kwh,stored),
            "power switches or retained battery inventory changed after save/load");

        battery.power_enabled=true;
        app.update_supply_simulation(.1f);
        require(near(app.supply_power_delivered_kw_,10) && near(app.battery_net_kw_,-10) && drill.supply.power_fraction>0,
            "battery failed to discharge after switching back on");
        std::cout<<"Brownout telemetry, load shedding, passive network links, and power state save/load passed\n";
    }
    static void supply_network(){
        // Pass uploads must retain draw order, style and nonoverlapping ranges.
        eng::GeometryStream stream;float a[]{1,2,3,4,5,6},b[]{7,8,9,10,11,12};
        stream.append(a,6,1,{1,0,0,1},2);stream.append(b,6,1,{1,0,0,1},2);
        require(stream.commands.size()==1 && stream.commands[0].count==4,"adjacent compatible draws did not merge");
        stream.append(a,6,4,{0,1,0,0.5f},1);
        require(stream.commands.size()==2 && stream.commands[1].first==4 && stream.commands[1].count==2 && stream.vertices[6]==7,"stream lost ordering or overwrote an earlier draw");
        stream.clear();require(stream.commands.empty() && stream.vertices.empty(),"projection boundary retained stale geometry");
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(192);
        auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        for(int k=0;k<(int)HabitatKind::Count;++k){PlacedHabitat h;h.kind=(HabitatKind)k;h.size=HabitatSizeTier::Small;h.position={(float)k*12,0,20};h.construction.step=kConstructionSteps;app.placed_habitats_.push_back(h);}
        for(auto kind:{ResourceKind::SolarPanel,ResourceKind::PowerStorage,ResourceKind::WaterStorage,ResourceKind::StorageDome}){PlacedResource r;r.kind=kind;r.position={(float)app.placed_resources_.size()*12,0,-20};r.construction.step=kConstructionSteps;app.placed_resources_.push_back(r);}
        auto pipe=[&](UtilityEndpointKind ka,std::size_t a,UtilityEndpointKind kb,std::size_t b,Vec3 pa,Vec3 pb){UtilityConnection u;u.endpoint_a.kind=ka;u.endpoint_a.index=a;u.endpoint_b.kind=kb;u.endpoint_b.index=b;u.path={pa,pb};u.construction.step=u.construction.total_steps;app.utility_network_.mutable_connections().push_back(u);};
        for(std::size_t i=1;i<4;++i)pipe(UtilityEndpointKind::Resource,i-1,UtilityEndpointKind::Resource,i,app.placed_resources_[i-1].position,app.placed_resources_[i].position);
        // Oxygen reaches the warehouse only through battery/tank service ports.
        pipe(UtilityEndpointKind::Habitat,1,UtilityEndpointKind::Resource,0,app.placed_habitats_[1].position,app.placed_resources_[0].position);
        for(std::size_t i=1;i<app.placed_habitats_.size();++i){
            if(i%2){CorridorConnection c;c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::Habitat;c.endpoint_a.index=i-1;c.endpoint_b.index=i;c.path={app.placed_habitats_[i-1].position,app.placed_habitats_[i].position};c.construction.step=c.construction.total_steps;app.corridor_network_.mutable_connections().push_back(c);}
            else pipe(UtilityEndpointKind::Habitat,i-1,UtilityEndpointKind::Habitat,i,app.placed_habitats_[i-1].position,app.placed_habitats_[i].position);
        }
        app.placed_resources_[2].supply.water_litres=2000;
        app.update_supply_simulation(240);
        for(const auto& h:app.placed_habitats_)require(h.supply.oxygen_kg>0,"oxygen from vacuum did not reach indoor type "+std::to_string((int)h.kind));
        require(app.placed_resources_[3].supply.oxygen_kg>0,"external manifold blocks oxygen from vacuum");
        for(auto& h:app.placed_habitats_)h.supply.oxygen_kg=h.supply.volume_m3*kOxygenKgPerM3*0.94;
        for(auto& r:app.placed_resources_)r.supply.oxygen_kg=r.supply.volume_m3*kOxygenKgPerM3*0.94;
        for(auto& u:app.utility_network_.mutable_connections())u.supply.oxygen_kg=u.supply.volume_m3*kOxygenKgPerM3*0.94;
        for(auto& c:app.corridor_network_.mutable_connections())c.supply.oxygen_kg=c.supply.volume_m3*kOxygenKgPerM3*0.94;
        for(int i=0;i<100;++i){app.placed_resources_[2].supply.water_litres=2000;app.update_supply_simulation(240);}
        for(const auto& h:app.placed_habitats_)require(h.supply.breathability()>95,"indoor kind "+std::to_string((int)h.kind)+" cannot fill through mixed network, air="+std::to_string(h.supply.breathability()));
        require(app.placed_resources_[3].supply.breathability()>95,"oxygen cannot transit outdoor service manifold to warehouse");
        // An unfinished endpoint must remain isolated, then join immediately on completion.
        app.placed_habitats_.back().construction.step=0;app.update_supply_simulation(1);require(app.placed_habitats_.back().supply.oxygen_kg==0,"blueprint acquired air");
        app.placed_habitats_.back().construction.step=kConstructionSteps;app.update_supply_simulation(100);require(app.placed_habitats_.back().supply.oxygen_kg>0,"completed room did not join the air network");
        GameApp solar(engine);clock.hour=12;solar.status_.restore(clock);PlacedResource panel;panel.kind=ResourceKind::SolarPanel;panel.construction.step=kConstructionSteps;PlacedResource battery=panel;battery.kind=ResourceKind::PowerStorage;solar.placed_resources_={panel,battery};
        UtilityConnection wire;wire.endpoint_a.kind=wire.endpoint_b.kind=UtilityEndpointKind::Resource;wire.endpoint_a.index=0;wire.endpoint_b.index=1;wire.path={{0,0,0},{5,0,0}};wire.construction.step=wire.construction.total_steps;solar.utility_network_.mutable_connections().push_back(wire);
        solar.update_supply_simulation(1);require(std::abs(solar.placed_resources_[1].supply.energy_kwh-0.95)<1e-6,"standard solar failed increased output/charge cap");
        solar.placed_resources_[0].size=ResourceSizeTier::Small;solar.placed_resources_[1].supply.energy_kwh=0;solar.update_supply_simulation(1);require(std::abs(solar.placed_resources_[1].supply.energy_kwh-0.76)<1e-6,"small solar failed doubled output");
        std::cout<<"All indoor types fill via mixed pipes/corridors and external manifolds; blueprint completion, solar doubling and ordered geometry streaming passed\n";
    }
    static void supply(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(192);app.builder_robots_.resize(1);app.builder_robots_[0].position={-80,0,80};
        auto resource=[](ResourceKind kind,Vec3 p){PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;};
        app.status_.reset();auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        require(std::abs(power_storage_capacity_kwh(ResourceSizeTier::Standard)-135.0)<1e-9 && std::abs(power_storage_capacity_kwh(ResourceSizeTier::Small)-67.5)<1e-9,"power storage capacity increase missing");

        // Generator-local buffers must preserve otherwise-curtailed output and
        // remain useful without replacing dedicated long-term storage.
        GameApp buffered(engine);buffered.playfield_.rebuild(96);buffered.status_.reset();auto buffered_clock=buffered.status_.state();buffered_clock.hour=12;buffered.status_.restore(buffered_clock);
        buffered.placed_resources_={resource(ResourceKind::SolarPanel,{0,0,0}),resource(ResourceKind::WaterDriller,{10,0,0})};
        UtilityConnection local;local.endpoint_a.kind=local.endpoint_b.kind=UtilityEndpointKind::Resource;local.endpoint_a.index=0;local.endpoint_b.index=1;local.path={{0,0,0},{10,0,0}};local.construction.step=local.construction.total_steps;buffered.utility_network_.mutable_connections().push_back(local);
        buffered.update_supply_simulation(10);
        require(std::abs(buffered.placed_resources_[0].supply.energy_kwh-solar_panel_buffer_capacity_kwh(ResourceSizeTier::Standard))<1e-6,"solar local buffer did not retain curtailed production");
        require(buffered.placed_resources_[1].supply.water_litres>0 && buffered.placed_resources_[1].supply.water_litres<=water_driller_buffer_capacity_litres(ResourceSizeTier::Standard)+1e-6,"water driller local buffer missing or overflowed");
        const double solar_buffer_before=buffered.placed_resources_[0].supply.energy_kwh;
        const double water_buffer_before=buffered.placed_resources_[1].supply.water_litres;
        buffered_clock.hour=0;buffered.status_.restore(buffered_clock);buffered.update_supply_simulation(1);
        require(buffered.placed_resources_[0].supply.energy_kwh<solar_buffer_before,"solar local buffer did not bridge night load");
        require(buffered.placed_resources_[1].supply.water_litres>water_buffer_before,"buffered solar did not keep the water driller operating at night");
        PlacedResource tank=resource(ResourceKind::WaterStorage,{20,0,0});buffered.placed_resources_.push_back(tank);
        UtilityConnection unload=local;unload.endpoint_a.index=1;unload.endpoint_b.index=2;unload.path={{10,0,0},{20,0,0}};buffered.utility_network_.mutable_connections().push_back(unload);
        const double buffered_water=buffered.placed_resources_[1].supply.water_litres;buffered.update_supply_simulation(0.1f);
        require(buffered.placed_resources_[2].supply.water_litres>=buffered_water-1e-6 && buffered.placed_resources_[1].supply.water_litres<1e-6,"water driller buffer did not unload into newly connected tank");

        app.placed_resources_={resource(ResourceKind::SolarPanel,{0,0,-30}),resource(ResourceKind::PowerStorage,{10,0,-30}),resource(ResourceKind::WaterDriller,{20,0,-30}),resource(ResourceKind::WaterStorage,{30,0,-30}),resource(ResourceKind::WaterStorage,{70,0,60})};
        const auto rr=[&](std::size_t a,std::size_t b){UtilityConnection u;u.endpoint_a.kind=u.endpoint_b.kind=UtilityEndpointKind::Resource;u.endpoint_a.index=a;u.endpoint_b.index=b;u.path={app.placed_resources_[a].position,app.placed_resources_[b].position};u.construction.step=u.construction.total_steps;app.utility_network_.mutable_connections().push_back(u);};
        rr(0,1);rr(1,2);rr(2,3);
        app.update_supply_simulation(10);
        require(app.placed_resources_[1].supply.energy_kwh>0,"solar did not charge connected battery");
        require(app.placed_resources_[3].supply.water_litres>0,"driller did not fill connected tank");
        require(app.placed_resources_[4].supply.water_litres==0,"water teleported into disconnected tank");
        require(app.ground_resource_piles_.empty(),"water appeared as a ground pallet");
        const double charged=app.placed_resources_[1].supply.energy_kwh;
        const double local_solar=app.placed_resources_[0].supply.energy_kwh;
        clock.hour=0;app.status_.restore(clock);app.update_supply_simulation(1);
        require(app.placed_resources_[0].supply.energy_kwh<local_solar,"night load did not use local solar buffer first");
        require(app.placed_resources_[1].supply.energy_kwh<=charged+1e-9,"night load unexpectedly increased battery");
        app.placed_resources_[0].supply.energy_kwh=0;app.placed_resources_[1].supply.energy_kwh=0;app.update_supply_simulation(1);
        require(app.placed_resources_[2].supply.warnings&NoPower,"unpowered outdoor consumer has no warning");
        clock.hour=12;app.status_.restore(clock);
        for(auto& q:app.placed_resources_)if(q.kind==ResourceKind::WaterStorage)q.supply.water_litres=2000;
        app.update_supply_simulation(2);require(app.placed_resources_[3].supply.water_litres<=2000,"tank overflow");
        app.placed_resources_[4].supply.water_litres=0;
        PlacedHabitat oxygen;oxygen.kind=HabitatKind::OxygenGeneratorDome;oxygen.position={30,0,0};oxygen.construction.step=kConstructionSteps;
        PlacedHabitat crew;crew.kind=HabitatKind::CrewQuarters;crew.position={30,0,25};crew.construction.step=kConstructionSteps;
        app.placed_habitats_={oxygen,crew};
        UtilityConnection feed;feed.endpoint_a.kind=UtilityEndpointKind::Resource;feed.endpoint_a.index=3;feed.endpoint_b.kind=UtilityEndpointKind::Habitat;feed.endpoint_b.index=0;feed.path={{30,0,-30},{30,0,0}};feed.construction.step=feed.construction.total_steps;app.utility_network_.mutable_connections().push_back(feed);
        CorridorConnection c;c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::Habitat;c.endpoint_a.index=0;c.endpoint_b.index=1;c.path={{30,0,0},{30,0,25}};c.construction.step=c.construction.total_steps;app.corridor_network_.restore_connections({c});
        const double water_before=app.placed_resources_[3].supply.water_litres;
        app.placed_resources_[2].construction.step=0; // isolate electrolysis mass balance
        app.update_supply_simulation(0.1f);
        require(app.placed_habitats_[0].supply.oxygen_kg>0,"electrolyzer did not produce oxygen");
        require(app.placed_habitats_[1].supply.oxygen_kg==0,"air teleported across corridor in first substep");
        require(app.placed_habitats_[0].supply.breathability()>app.placed_habitats_[1].supply.breathability(),"all rooms filled simultaneously");
        app.update_supply_simulation(60);
        double mass=0;for(const auto& h:app.placed_habitats_)mass+=h.supply.oxygen_kg;for(const auto& q:app.placed_resources_)mass+=q.supply.oxygen_kg;for(const auto& u:app.utility_network_.connections())mass+=u.supply.oxygen_kg;for(const auto& q:app.corridor_network_.connections())mass+=q.supply.oxygen_kg;
        require(std::abs((water_before-app.placed_resources_[3].supply.water_litres)-mass*kWaterPerOxygen)<1e-7,"air flow/electrolysis lost mass");
        require(app.placed_habitats_[1].supply.oxygen_kg>0,"air never reached downstream room");
        require(app.placed_habitats_[1].supply.warnings&LowAir,"low-air indoor consumer has no warning");
        for(const auto& h:app.placed_habitats_)require(h.supply.breathability()>=0 && h.supply.breathability()<=100,"air percentage outside bounds");
        const double saved_air=app.placed_habitats_[1].supply.oxygen_kg,saved_hydrogen=app.placed_habitats_[0].supply.hydrogen_kg,saved_energy=app.placed_resources_[1].supply.energy_kwh,saved_water=app.placed_resources_[3].supply.water_litres;
        std::string error;require(app.save_game_to_name("supply-regression",error),"save failed: "+error);app.refresh_save_games();
        std::string path;for(const auto& e:app.save_games_)if(e.name=="supply-regression")path=e.path;
        require(!path.empty(),"saved supply state not listed");GameApp restored(engine);require(restored.load_game_from_file(path,error),"load failed: "+error);
        require(restored.placed_habitats_[1].supply.breathability()>0,"loaded interior did not display saved air without an unpause tick");
        require(restored.placed_habitats_[1].supply.oxygen_kg==saved_air && restored.placed_habitats_[0].supply.hydrogen_kg==saved_hydrogen && restored.placed_resources_[1].supply.energy_kwh==saved_energy && restored.placed_resources_[3].supply.water_litres==saved_water,"saved inventories changed on load");
        GameApp gate(engine);gate.playfield_.rebuild(192);gate.builder_robots_.resize(1);gate.builder_robots_[0].position={-80,0,80};PlacedResource dome=resource(ResourceKind::StorageDome,{0,0,0});dome.placement_sequence=1;gate.placed_resources_={dome};gate.next_blueprint_placement_sequence_=2;
        require(gate.add_storage_gate_at(0,53.25f),"free-angle gate rejected");
        const auto& owner=gate.placed_resources_[0];const auto n=storage_loading::normal(owner);
        require(std::abs(std::atan2(n.x,n.z)*180.0/3.141592653589793-53.25)<1e-4,"continuous gate angle quantized");
        const auto service=storage_loading::service(owner);require(distance(service,storage_loading::mouth(owner))<1.6f,"cargo placing area remains too far from gate");
        UtilityConnection blocked;blocked.path={service-Vec3{4,0,0},service+Vec3{4,0,0}};gate.utility_network_.mutable_connections().push_back(blocked);
        require(!gate.gate_route_clear(blocked.path,0.35f),"pipe crosses reserved placing pad");
        require(!gate.resource_placement_valid(ResourceKind::Antenna,ResourceSizeTier::Small,StorageDomeSize::Small,service,0),"structure overlaps gate pad");
        GeometryRecorder hidden;gate.render_gate_areas(hidden);require(hidden.filled_vertices==0 && hidden.wire_vertices==0,"unused placement markers remain visible");
        gate.selection_kind_=GameApp::SelectionKind::Resource;gate.selected_resource_=0;
        GeometryRecorder recorder;gate.render_gate_areas(recorder);require(recorder.filled_vertices==0 && recorder.wire_vertices>0 && !recorder.yellow,"selected cargo pad outline missing or orange placements remain");
        require(gate.save_game_to_name("supply-gate-regression",error),"gate save failed");gate.refresh_save_games();for(const auto& e:gate.save_games_)if(e.name=="supply-gate-regression")path=e.path;
        require(restored.load_game_from_file(path,error),"free-angle gate load failed: "+error);
        require(std::abs(restored.placed_resources_[0].storage_gate_angle_degrees-53.25f)<1e-6,"gate angle changed on load");
        std::cout<<"Generator buffers, increased battery capacity, connected storage, day/night dispatch, tank limits, air delay/conservation, warnings, gate pads and save roundtrip passed\n";
    }

    static void utilities(){
        eng::Application engine;
        GameApp app(engine);app.playfield_.rebuild(192);
        auto complete_resource=[](ResourceKind kind,Vec3 p){
            PlacedResource r;r.kind=kind;r.position=p;r.construction.step=kConstructionSteps;return r;
        };
        auto complete_habitat=[](HabitatKind kind,Vec3 p){
            PlacedHabitat h;h.kind=kind;h.position=p;h.construction.step=kConstructionSteps;return h;
        };
        app.placed_resources_={
            complete_resource(ResourceKind::SolarPanel,{-30,0,-20}),
            complete_resource(ResourceKind::SolarPanel,{-22,0,-20}),
            complete_resource(ResourceKind::PowerStorage,{-14,0,-20}),
            complete_resource(ResourceKind::WaterDriller,{-6,0,-20}),
            complete_resource(ResourceKind::WaterPurifier,{2,0,-20}),
            complete_resource(ResourceKind::SolarPanel,{30,0,-20}),
            complete_resource(ResourceKind::PowerStorage,{38,0,-20})
        };
        app.placed_habitats_={
            complete_habitat(HabitatKind::ProcessingFacility,{-10,0,10}),
            complete_habitat(HabitatKind::BioDome,{0,0,10}),
            complete_habitat(HabitatKind::Factory,{12,0,10})
        };
        auto utility_rr=[&](std::size_t a,std::size_t b){
            UtilityConnection c;c.endpoint_a.kind=UtilityEndpointKind::Resource;c.endpoint_a.index=a;
            c.endpoint_b.kind=UtilityEndpointKind::Resource;c.endpoint_b.index=b;
            c.construction.step=c.construction.total_steps;
            c.path={app.placed_resources_[a].position,app.placed_resources_[b].position};
            app.utility_network_.mutable_connections().push_back(std::move(c));
        };
        utility_rr(0,1);utility_rr(1,2);utility_rr(2,3);utility_rr(3,4);
        // Deliberately separate component: connected physically, but it has only
        // one solar source and therefore must not acquire stable power from the
        // other network by global accounting.
        utility_rr(5,6);

        CorridorConnection pipe_corridor;
        pipe_corridor.endpoint_a.kind=CorridorEndpointKind::Habitat;pipe_corridor.endpoint_a.index=0;
        pipe_corridor.endpoint_b.kind=CorridorEndpointKind::Habitat;pipe_corridor.endpoint_b.index=1;
        pipe_corridor.construction.step=pipe_corridor.construction.total_steps;
        pipe_corridor.path={{-10,0,10},{0,0,10}};
        CorridorConnection unfinished;
        unfinished.endpoint_a.kind=CorridorEndpointKind::Habitat;unfinished.endpoint_a.index=1;
        unfinished.endpoint_b.kind=CorridorEndpointKind::Habitat;unfinished.endpoint_b.index=2;
        unfinished.construction.step=0;unfinished.path={{0,0,10},{12,0,10}};
        app.corridor_network_.restore_connections({pipe_corridor,unfinished});

        UtilityConnection feed;
        feed.endpoint_a.kind=UtilityEndpointKind::Resource;feed.endpoint_a.index=2;
        feed.endpoint_b.kind=UtilityEndpointKind::Corridor;feed.endpoint_b.index=0;
        feed.construction.step=feed.construction.total_steps;
        feed.path={{-14,0,-20},{-8,0,2}};
        app.utility_network_.mutable_connections().push_back(feed);

        auto services=app.utility_network_.evaluate_services(app.corridor_network_,app.placed_habitats_,app.placed_resources_);
        require(services.resources[0].power && services.resources[4].power,"powered supply component did not distribute power");
        require(services.resources[3].water && services.resources[4].water,"water source/purifier component did not distribute water");
        require(services.habitats[0].power && services.habitats[0].water && services.habitats[1].power && services.habitats[1].water,
                "completed corridor did not carry the embedded pipe network");
        require(!services.habitats[2].connected && !services.habitats[2].power && !services.habitats[2].water,
                "unfinished corridor transported utilities");
        require(services.resources[5].connected && services.resources[5].power,
                "standalone solar/battery component failed potential power topology");
        require(services.habitats[0].component==services.resources[0].component &&
                services.habitats[0].component!=services.resources[5].component,
                "utility component identity is not topology-local");

        app.corridor_network_.mutable_connections()[1].construction.step=kCorridorConstructionSteps;
        services=app.utility_network_.evaluate_services(app.corridor_network_,app.placed_habitats_,app.placed_resources_);
        require(services.habitats[2].power && services.habitats[2].water && services.habitats[2].component==services.habitats[0].component,
                "completed corridor extension did not join the utility component");

        // Storage dome corridor and supply connections are free-angle. Cargo
        // apertures remain reserved, but there is no cardinal/90-degree snap on
        // the rest of the circular hull.
        GameApp storage(engine);storage.playfield_.rebuild(192);
        PlacedResource dome;dome.kind=ResourceKind::StorageDome;dome.storage_size=StorageDomeSize::Mid;
        dome.rotation_degrees=31.0f;dome.construction.step=kConstructionSteps;dome.placement_sequence=1;
        storage.placed_resources_={dome};storage.next_blueprint_placement_sequence_=2;
        require(storage.add_storage_gate(0,4),"explicit cargo gate extension could not be added");
        storage.placed_resources_[1].construction.step=kConstructionSteps;
        storage.synchronize_storage_gates();
        dome=storage.placed_resources_[0];
        constexpr float PI_F=3.14159265358979323846f;
        const float angle=23.0f*PI_F/180.0f;
        Vec3 local{std::sin(angle),0.0f,-std::cos(angle)};
        const Vec3 dir=world_math::rotate_y(local,dome.rotation_degrees);
        const float wall_r=storage_dome_diameter_m(dome.storage_size)*(5.35f/13.0f);
        const Vec3 hit=dome.position+dir*wall_r;
        CorridorBuildAnchor corridor_anchor{CorridorEndpointKind::Resource,0,hit};
        auto corridor_ep=storage.corridor_network_.preview_anchor(corridor_anchor,hit+dir*18.0f,{},storage.habitat_models_,storage.placed_resources_,storage.resource_models_);
        require(corridor_ep.has_value(),"free-angle storage corridor anchor rejected");
        require(eng::dot(corridor_ep->door.normal,dir)>0.995f,"storage corridor anchor snapped away from cursor-selected hull angle");
        require(std::abs(corridor_ep->door.normal.x)>0.12f && std::abs(corridor_ep->door.normal.z)>0.12f,
                "storage corridor anchor quantized to a cardinal axis");

        UtilityBuildAnchor utility_anchor{UtilityEndpointKind::Resource,0,hit};
        auto utility_ep=storage.utility_network_.preview_anchor(UtilityKind::SupplyChannel,utility_anchor,hit+dir*18.0f,
            {},storage.habitat_models_,storage.placed_resources_,storage.resource_models_);
        require(utility_ep.has_value(),"free-angle storage supply anchor rejected");
        require(eng::dot(utility_ep->normal,dir)>0.995f,"storage supply anchor snapped away from cursor-selected hull angle");
        UtilityBuildAnchor open{UtilityEndpointKind::Open,0,hit+dir*18.0f+Vec3{5,0,4}};
        auto route=storage.utility_network_.preview_connection(UtilityKind::SupplyChannel,utility_anchor,open,
            {},storage.habitat_models_,storage.placed_resources_,storage.resource_models_);
        require(route.has_value() && route->path.size()>=3,"free-angle storage supply route was not generated");
        Vec3 first{};bool found_first=false;
        for(std::size_t i=1;i<route->path.size();++i){
            Vec3 step=route->path[i]-route->path[0];step.y=0;
            if(eng::length(step)>0.15f){first=eng::normalize(step);found_first=true;break;}
        }
        require(found_first && eng::dot(first,dir)>0.70f,"storage supply route did not leave the dome smoothly along its selected tangent");
        UtilityBuildAnchor cargo{UtilityEndpointKind::Resource,0,storage_loading::wall(dome)};
        require(!storage.utility_network_.preview_anchor(UtilityKind::SupplyChannel,cargo,storage_loading::service(dome),
            {},storage.habitat_models_,storage.placed_resources_,storage.resource_models_),"supply channel occupied a storage cargo gate sector");

        std::cout<<"Pipe components, corridor-carried utilities, independent power/water availability and free-angle storage attachments passed\n";
    }

    static void gates(){
        eng::Application engine;GameApp app(engine);app.playfield_.rebuild(256);
        for(float rotation:{0.0f,37.0f,90.0f,135.0f,270.0f}){
            CorridorDoor d;d.normal=world_math::rotate_y({1,0,0},rotation);
            d.position=d.wall_position={0,0.5f,0};
            const Vec3 side{-d.normal.z,0,d.normal.x};
            PressureDoorState state{d};
            require(pressure_door::sealed(state.open_fraction),"new door not sealed");
            require(pressure_door::blocks(d,0,d.normal*-2,d.normal*2,0.272f),"closed door permits tunnelling");
            for(int i=0;i<30;++i) pressure_door::advance(state,0.02f,true,false);
            require(state.open_fraction==1 && !pressure_door::blocks(d,1,d.normal*-2,d.normal*2,0.272f),"open centre blocked");
            require(pressure_door::blocks(d,0.25f,d.normal*-1,d.normal,0.272f),"partly open leaves not solid");
            require(pressure_door::blocks(d,1,d.normal*-1+side*0.60f,d.normal+side*0.60f,0.272f),"open jamb ignored");
            for(int i=0;i<100;++i) pressure_door::advance(state,0.02f,false,true);
            require(state.open_fraction==1,"door closed onto an occupant");
            for(int i=0;i<100;++i) pressure_door::advance(state,0.02f,false,false);
            require(pressure_door::sealed(state.open_fraction),"door failed to reseal");
            for(int size=0;size<4;++size){
                PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.storage_size=(StorageDomeSize)size;
                storage.rotation_degrees=rotation;storage.placement_sequence=1;storage.construction.step=kConstructionSteps;
                storage.stored_boxes={{MaterialResource::Iron,100,1}};sync_stored_resource_totals(storage);
                app.placed_resources_={storage};app.placed_habitats_.clear();app.corridor_network_.clear();app.utility_network_.clear();app.walkway_network_.clear();
                app.next_blueprint_placement_sequence_=2;
                // A gate-less dome keeps its normal hull available for smooth
                // corridor attachment. Optional cargo extensions reserve only
                // geometry that is explicitly placed afterwards.
                for(int face:{0,1,2,3}){
                    const auto direction=world_math::rotate_y(storage_loading::local_normal(face),rotation);
                    const auto wall=storage.position+direction*(storage_dome_diameter_m(storage.storage_size)*(5.35f/13.0f));
                    const CorridorBuildAnchor crew{CorridorEndpointKind::Resource,0,wall};
                    const CorridorBuildAnchor far{CorridorEndpointKind::OpenDoor,0,wall+direction*14};
                    app.corridor_network_.clear();
                    require(app.corridor_network_.build_corridor(crew,far,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"gate-less crew port build rejected");
                }
                app.corridor_network_.clear();
                require(app.add_storage_gate(0,4),"cargo gate task rejected");
                app.placed_resources_[1].construction.step=kConstructionSteps;app.synchronize_storage_gates();
                const auto& owner=app.placed_resources_[0];
                const int gate_face=app.placed_resources_[1].storage_loading_face;
                const auto service=storage_loading::service(owner,gate_face);const auto n=storage_loading::normal(owner,gate_face);
                const Vec3 gate_side{-n.z,0,n.x};
                require(storage_loading::reserved(owner,service),"gate service outside reservation");
                require(storage_loading::segment_reserved(owner,service-gate_side*10,service+gate_side*10,geometry::kCorridorHalfWidth),"crossing misses loading reservation");
                CorridorBuildAnchor cargo{CorridorEndpointKind::Resource,0,storage_loading::wall(owner,gate_face)};
                CorridorBuildAnchor end{CorridorEndpointKind::OpenDoor,0,service+n*12};
                require(!app.corridor_network_.preview_anchor(cargo,end.position,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"cargo face preview allowed");
                require(!app.corridor_network_.build_corridor(cargo,end,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"cargo face build allowed");
                app.corridor_network_.clear();
                CorridorBuildAnchor cross_a{CorridorEndpointKind::OpenDoor,0,service-gate_side*12},cross_b{CorridorEndpointKind::OpenDoor,0,service+gate_side*12};
                auto route=app.corridor_network_.preview_built_corridor(cross_a,cross_b,{},app.habitat_models_,app.placed_resources_,app.resource_models_);
                if(route) for(std::size_t j=1;j<route->path.size();++j)
                    require(!storage_loading::segment_reserved(owner,route->path[j-1],route->path[j],geometry::kCorridorHalfWidth),"bypass clips cargo bay");
                BuilderRobotState bot;bot.role=RobotRole::Carrier;
                ResourceBundle need;need.amount(MaterialResource::Iron)=20;
                bot.position=owner.position-n*(storage_dome_diameter_m(owner.storage_size)*0.5f+1.55f);
                require(!app.robot_load_needed_resources(bot,need),"pickup through wrong storage wall");
                bot.position=service;
                require(app.robot_load_needed_resources(bot,need),"cargo gate pickup failed");
                require(bot.cargo.amount(MaterialResource::Iron)==20 && app.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==80,"pickup changed inventory incorrectly");
            }
        }
        // Trace the actual triangles submitted by the world renderer. Closed
        // leaves cover the full opening; fully retracted leaves clear it.
        app.placed_resources_.clear();app.pressure_doors_.clear();
        PlacedHabitat air;air.kind=HabitatKind::AirLock;air.construction.step=kConstructionSteps;
        app.placed_habitats_={air};const auto& mesh=app.habitat_models_.mesh(air);require(mesh.entrances.empty(),"air lock retained authored hatch");
        app.corridor_network_.set_playfield_half_extent(64);
        const float air_end=mesh.footprint_x*0.5f;
        CorridorBuildAnchor hatch_anchor{CorridorEndpointKind::Habitat,0,{air_end,mesh.walkable_floor_y,0}},open_anchor{CorridorEndpointKind::OpenDoor,0,{air_end+14,0,0}};
        require(app.corridor_network_.build_corridor(hatch_anchor,open_anchor,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"generated Air Lock hatch rejected");
        app.corridor_network_.mutable_connections()[0].construction.step=kCorridorConstructionSteps;
        const auto hatch=app.corridor_network_.connections()[0].endpoint_a.door;
        app.first_person_mode_=GameApp::FirstPersonMode::Off;
        app.update_pressure_doors(0.1f);
        const auto doors=app.habitat_doors(0);require(doors.size()==4,"Air Lock needs four pressure doors across three chambers");
        GeometryRecorder draw;app.render_habitats(draw);
        const auto n=hatch.normal;const Vec3 side{-n.z,0,n.x};
        for(float offset:{-0.62f,0.0f,0.62f}) for(float height:{0.12f,1.0f,2.34f}){
            const auto point=hatch.wall_position+side*offset+Vec3{0,height,0};
            require(segment_hits_mesh(point-n*0.30f,point+n*0.30f,draw.triangles),"rendered door has an unsealed gap");
        }
        app.first_person_mode_=GameApp::FirstPersonMode::Active;
        app.first_person_.spawn(hatch.wall_position+n,0);
        for(int i=0;i<8;++i) app.update_pressure_doors(0.10f);
        draw.triangles.clear();app.render_habitats(draw);
        const auto point=hatch.wall_position+Vec3{0,1.0f,0};
        require(!segment_hits_mesh(point-n*0.30f,point+n*0.30f,draw.triangles),"retracted rendered leaves still block doorway");
        const auto outer=doors[1];app.first_person_.spawn(outer.wall_position+outer.normal,std::atan2(-outer.normal.x,outer.normal.z)*180.f/3.14159265358979323846f);app.input_={};app.input_.key_w_down=true;
        for(int i=0;i<80;++i){
            app.update_pressure_doors(0.025f);
            const float inner_open=app.first_person_door_open_fraction(hatch.wall_position);
            const float outer_open=app.first_person_door_open_fraction(outer.wall_position);
            require(inner_open==0 || outer_open==0,"Air Lock opened both seals together");
        }
        require(app.first_person_door_open_fraction(outer.wall_position)==1,"Air Lock did not release next door after sealing");
        app.first_person_mode_=GameApp::FirstPersonMode::Off;
        app.placed_habitats_.clear();app.corridor_network_.clear();
        // Cargo openings expose only a buffer: its inner pressure wall remains
        // solid with the outer shutters fully retracted.
        PlacedResource lock;lock.kind=ResourceKind::StorageDome;lock.placement_sequence=1;lock.construction.step=kConstructionSteps;
        app.placed_resources_={lock};app.next_blueprint_placement_sequence_=2;require(app.add_storage_gate(0,4),"render cargo gate task rejected");
        app.placed_resources_[1].construction.step=kConstructionSteps;app.synchronize_storage_gates();app.placed_resources_[1].storage_extension_open=1;
        draw.triangles.clear();app.render_resources(draw);
        const int lock_face=app.placed_resources_[1].storage_loading_face;
        const auto lock_n=storage_loading::normal(app.placed_resources_[0],lock_face);
        const auto mouth=storage_loading::mouth(app.placed_resources_[0],lock_face)+Vec3{0,0.8f,0};
        const auto back=storage_loading::wall(app.placed_resources_[0],lock_face)+Vec3{0,0.8f,0};
        require(!segment_hits_mesh(mouth-lock_n*0.25f,mouth+lock_n*0.25f,draw.triangles),"open cargo shutter obstructed");
        require(segment_hits_mesh(back-lock_n*0.2f,back+lock_n*0.2f,draw.triangles),"cargo gate exposes warehouse atmosphere");

        // Two real CR-01s must deposit through the same loading bay without
        // colliding, parking at another wall, or losing/duplicating material.
        app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=2;
        app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_.clear();
        PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.placement_sequence=1;storage.construction.step=kConstructionSteps;
        app.placed_resources_={storage};app.placed_habitats_.clear();app.corridor_network_.clear();app.utility_network_.clear();app.walkway_network_.clear();
        app.next_blueprint_placement_sequence_=2;require(app.add_storage_gate(0,4),"fleet cargo gate task rejected");
        app.placed_resources_[1].construction.step=kConstructionSteps;app.synchronize_storage_gates();
        const auto gate=storage_loading::service(app.placed_resources_[0],app.placed_resources_[1].storage_loading_face);
        app.builder_robots_[0].position={-25,0,-20};
        for(std::size_t i=1;i<3;++i){auto& bot=app.builder_robots_[i];
            bot.position=gate+Vec3{3.0f+(float)i*2,0,(float)(i-1)*4};bot.mode=BuilderRobotMode::DepotTransfer;
            bot.cargo.amount(MaterialResource::Iron)=30;bot.starter_exit_completed=true;}
        for(int frame=0;frame<7200 && app.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)<60;++frame){
            app.update_construction(1.0f/60);
            for(const auto& bot:app.builder_robots_) require(bot.safety.mode!=SafetyMode::Fault,"cargo fleet safety fault");
            require(distance(app.builder_robots_[1].position,app.builder_robots_[2].position)>1.88f,"cargo fleet collision");
        }
        require(app.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==60,"shared gate deposits did not finish");
        std::string error;require(app.save_game_to_name("gate-regression",error),error);
        const auto saved=std::filesystem::path(app.save_game_directory())/"gate-regression.gmsave";
        GameApp restored(engine);require(restored.load_game_from_file(saved.string(),error),error);
        require(restored.placed_resources_.size()==2 && storage_loading::gate_count(restored.placed_resources_[0])==1 &&
                restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==60,"gate save round trip failed");
        std::filesystem::remove(saved);
        PlacedResource panel;panel.kind=ResourceKind::SolarPanel;panel.position={20,0,10};panel.placement_sequence=restored.next_blueprint_placement_sequence_++;
        panel.resource_plan.total_steps=kConstructionSteps;
        for(auto& step:panel.resource_plan.steps) step.amount(MaterialResource::Iron)=5;
        restored.placed_resources_.push_back(panel);
        ResourceBundle physical;physical.amount(MaterialResource::Iron)=60;restored.resources_.restore(physical);
        const std::size_t panel_index=restored.placed_resources_.size()-1;
        for(int frame=0;frame<18000 && !restored.placed_resources_[panel_index].construction.complete();++frame){
            const auto before=restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron);
            restored.update_construction(1.0f/60);
            if(restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)<before){
                bool at_gate=false;
                for(const auto& bot:restored.builder_robots_) at_gate=at_gate || storage_loading::at_service(restored.placed_resources_[0],bot.position);
                require(at_gate,"autonomous pickup bypassed the cargo gate");
            }
            for(const auto& bot:restored.builder_robots_) require(bot.safety.mode!=SafetyMode::Fault,"storage construction safety fault");
        }
        require(restored.placed_resources_[panel_index].construction.complete(),"construction could not withdraw through cargo gate");
        require(restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==30,"construction cargo ledger mismatch");
        std::cout<<"Pressure seals, swept door collision, 20 rotated loading gates, reserved routing, shared deposits/withdrawals, construction and save round trip passed\n";
    }
    static void geometry(){
        eng::Application engine;
        GameApp app(engine);app.playfield_.rebuild(256);
        const auto check_walk=[&](Vec3 a,Vec3 b,const std::string& label){
            const int n=std::max(1,(int)std::ceil(distance(a,b)/0.04f));
            a.y=app.first_person_floor_height(a);
            for(int j=1;j<=n;++j){
                Vec3 p=a+(b-a)*((float)j/n);p.y=app.first_person_floor_height(p);
                Vec3 q=a+(b-a)*((float)(j-1)/n);q.y=app.first_person_floor_height(q);
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                const auto direction=p-q;const float yaw=std::atan2(direction.x,-direction.z)*180.f/3.14159265358979323846f;
                app.first_person_.spawn({q.x,app.first_person_floor_height(q),q.z},yaw);app.input_={};app.input_.key_w_down=true;
                for(int tick=0;tick<5;++tick)app.update_pressure_doors(0.10f);
                for(int tick=0;tick<80 && !app.first_person_move_allowed(q,p);++tick)app.update_pressure_doors(0.10f);
                require(app.first_person_move_allowed(q,p),label+": blocked at "+std::to_string(p.x)+","+std::to_string(p.z));
                require(std::abs(p.y-app.first_person_floor_height(q))<0.21f,label+": floor discontinuity at "+std::to_string(p.x)+","+std::to_string(p.z)+" y="+std::to_string(p.y)+" previous="+std::to_string(app.first_person_floor_height(q)));
            }
        };
        const std::vector<Vec3> wall{{0,0,-5},{0,4,5},{0,0,5},{0,0,-5},{0,4,-5},{0,4,5}};
        const auto carved=mesh_portals::subtract(wall,{{0,0,0},{1,0,0},0.5f,0.7f,0.5f,2.95f});
        float area=0;for(std::size_t i=0;i+2<carved.size();i+=3) area+=eng::length(eng::cross(carved[i+1]-carved[i],carved[i+2]-carved[i]))*0.5f;
        require(std::abs(area-(40.0f-1.4f*2.45f))<0.001f,"exact portal aperture area");
        for(int kind=1;kind<(int)HabitatKind::Count;++kind) for(int tier=0;tier<(int)HabitatSizeTier::Count;++tier) for(float rotation:{180.0f,135.0f,90.0f}){
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat a;a.kind=HabitatKind::AirLock;a.position={-22,0,0};a.construction.step=kConstructionSteps;
            PlacedHabitat b;b.kind=(HabitatKind)kind;b.size=(HabitatSizeTier)tier;b.position={22,0,0};b.rotation_degrees=rotation;b.construction.step=kConstructionSteps;
            app.placed_habitats_={a,b};
            const auto& mesh=app.habitat_models_.mesh(b);
            require(mesh.entrances.empty() && mesh.entrance_access.empty(),"habitat retained a predefined entrance");
            const auto toward_hull=[&](const PlacedHabitat& h,Vec3 toward){const auto& hm=app.habitat_models_.mesh(h);Vec3 d=toward-h.position;d.y=0;if(eng::length(d)<0.01f)d={1,0,0};d=eng::normalize(d);return h.position+d*(std::max(hm.footprint_x,hm.footprint_z)*0.55f);};
            const CorridorBuildAnchor ca{CorridorEndpointKind::Habitat,0,toward_hull(a,b.position)};
            const CorridorBuildAnchor cb{CorridorEndpointKind::Habitat,1,toward_hull(b,a.position)};
            auto c=app.corridor_network_.preview_built_corridor(ca,cb,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(c.has_value(),"corridor unavailable kind "+std::to_string(kind));
            require(c->endpoint_a.door.generated && c->endpoint_b.door.generated && c->endpoint_a.door.entrance_index<0 && c->endpoint_b.door.entrance_index<0,"corridor did not create on-demand entrances");
            c->construction.step=kCorridorConstructionSteps;app.corridor_network_.restore_connections({*c});
            const std::string label="habitat "+std::to_string(kind)+" tier "+std::to_string(tier);
            for(const auto* ep:{&c->endpoint_a,&c->endpoint_b}){
                const auto& h=app.placed_habitats_[ep->index];
                const auto& hm=app.habitat_models_.mesh(h);
                Vec3 inside=ep->door.wall_position-ep->door.normal*0.45f;
                inside.y=h.position.y+hm.walkable_floor_y;
                check_walk(ep->position,ep->door.position,label+" ramp");
                check_walk(ep->door.position,inside,label+" generated entrance");
                check_walk(inside,ep->door.position,label+" generated return");
            }
            for(std::size_t j=1;j<c->path.size();++j) check_walk(c->path[j-1],c->path[j],label+" corridor");
        }
        // Corridor attachment is a continuous polar choice around a structure.
        // Moving the cursor by one degree must move/rotate the generated hatch
        // smoothly instead of snapping between authored cardinal faces.
        {
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat h;h.kind=HabitatKind::CrewQuarters;h.size=HabitatSizeTier::Standard;
            h.position={0,0,0};h.rotation_degrees=23.0f;h.construction.step=kConstructionSteps;
            app.placed_habitats_={h};
            const auto& hm=app.habitat_models_.mesh(h);
            std::vector<Vec3> walls;
            walls.reserve(360);
            for(int degree=0;degree<360;++degree){
                const float a=(float)degree*3.14159265358979323846f/180.0f;
                const Vec3 dir{std::cos(a),0,std::sin(a)};
                const CorridorBuildAnchor anchor{CorridorEndpointKind::Habitat,0,h.position+dir*(std::max(hm.footprint_x,hm.footprint_z)+4.0f)};
                const auto ep=app.corridor_network_.preview_anchor(anchor,h.position+dir*100.0f,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
                require(ep.has_value(),"360-degree corridor attachment missing at "+std::to_string(degree));
                const Vec3 selected=eng::normalize(Vec3{ep->door.wall_position.x-h.position.x,0,ep->door.wall_position.z-h.position.z});
                require(eng::dot(selected,dir)>0.9995f,"corridor attachment position snapped away from cursor angle at "+std::to_string(degree));
                walls.push_back(ep->door.wall_position);
            }
            for(std::size_t i=0;i<walls.size();++i){
                const std::size_t next=(i+1)%walls.size();
                require(world_math::distance_xz(walls[i],walls[next])<0.85f,"corridor attachment position jumps around hull near degree "+std::to_string(i));
            }
        }

        // Bio Dome barrel ends are real pressure surfaces. The old half-cylinder
        // mesh had no X end caps, so a generated hatch could hit the low
        // foundation but its upper doorway profile had nothing to attach to.
        // Verify both long-axis ends for every size and with rotation applied.
        {
            for(int tier=0;tier<(int)HabitatSizeTier::Count;++tier) for(float rotation:{0.0f,37.0f,90.0f}){
                app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
                PlacedHabitat bio;bio.kind=HabitatKind::BioDome;bio.size=(HabitatSizeTier)tier;
                bio.position={0,0,0};bio.rotation_degrees=rotation;bio.construction.step=kConstructionSteps;
                app.placed_habitats_={bio};
                const auto& bm=app.habitat_models_.mesh(bio);
                for(float sign:{-1.0f,1.0f}){
                    const Vec3 direction=world_math::rotate_y({sign,0,0},rotation);
                    const CorridorBuildAnchor source{CorridorEndpointKind::Habitat,0,bio.position+direction*(bm.footprint_x+4.0f)};
                    const CorridorBuildAnchor open{CorridorEndpointKind::OpenDoor,0,bio.position+direction*(bm.footprint_x+18.0f)};
                    const auto endpoint=app.corridor_network_.preview_anchor(source,open.position,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
                    require(endpoint.has_value(),"Bio Dome front/back corridor anchor missing tier "+std::to_string(tier)+" rot "+std::to_string((int)rotation));
                    require(endpoint->door.wall_profile.size()>=10,"Bio Dome end hatch lacks conformal pressure profile");
                    require(eng::dot(endpoint->door.normal,direction)>0.94f,"Bio Dome end hatch normal points away from selected end");
                    auto connection=app.corridor_network_.preview_built_corridor(source,open,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
                    require(connection.has_value(),"Bio Dome front/back corridor build preview rejected");
                }
            }
        }

        // Reproduce the 0-43-20.gmsave spacing issue: an Air Lock at
        // (-20,-4) and a SMALL Oxygen Generator at (-11,-4) had only ~1.21 m
        // between their old ground footprints, so the opposing docking runs
        // overlapped before a straight corridor could exist. The new ground
        // reservation must reject that legacy-new-placement distance, and the
        // nearest legal integer-grid placement must connect with one straight
        // corridor segment.
        {
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat left;left.kind=HabitatKind::AirLock;left.size=HabitatSizeTier::Small;
            left.position={-20,0,-4};left.construction.step=kConstructionSteps;
            PlacedHabitat right;right.kind=HabitatKind::OxygenGeneratorDome;right.size=HabitatSizeTier::Small;
            right.rotation_degrees=0;right.construction.step=kConstructionSteps;
            app.placed_habitats_={left};
            require(!app.placement_valid(right.kind,right.size,{-11,0,-4},0.0f),
                    "0-43-20 close habitat spacing still accepted");
            bool found=false;float nearest_x=-11.0f;
            for(int x=-11;x<=16;++x){
                if(app.placement_valid(right.kind,right.size,{(float)x,0,-4},0.0f)){nearest_x=(float)x;found=true;break;}
            }
            require(found,"corridor-reserved ground rectangle left no nearby legal habitat placement");
            if(nearest_x>-11.0f) require(!app.placement_valid(right.kind,right.size,{nearest_x-1.0f,0,-4},0.0f),
                                         "nearest legal corridor-reserved spacing was not minimal");
            right.position={nearest_x,0,-4};app.placed_habitats_={left,right};
            const auto& lm=app.habitat_models_.mesh(left);const auto& rm=app.habitat_models_.mesh(right);
            const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,left.position+Vec3{lm.footprint_x,0,0}};
            const CorridorBuildAnchor b{CorridorEndpointKind::Habitat,1,right.position-Vec3{rm.footprint_x,0,0}};
            const auto c=app.corridor_network_.preview_built_corridor(a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(c.has_value(),"nearest legal structure spacing could not create corridor");
            require(c->path.size()==2,"nearest legal structure spacing did not create a straight corridor");
            require(distance(c->path.front(),c->path.back())>=0.30f,"ground reservation left a degenerate corridor gap");
        }

        // A close, unobstructed structure pair gets one straight corridor
        // centerline. Endpoint vestibules may meet the hull at different local
        // angles, but the pressure tube between them must not form an S-curve.
        {
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat left;left.kind=HabitatKind::CrewQuarters;left.construction.step=kConstructionSteps;
            PlacedHabitat right=left;right.kind=HabitatKind::CantineDome;right.rotation_degrees=31.0f;
            const auto& lm=app.habitat_models_.mesh(left);const auto& rm=app.habitat_models_.mesh(right);
            const float center_distance=lm.footprint_x*0.5f+rm.footprint_x*0.5f+10.0f;
            left.position={-center_distance*0.5f,0,0};right.position={center_distance*0.5f,0,0};
            app.placed_habitats_={left,right};
            const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,left.position+Vec3{lm.footprint_x,0,0}};
            const CorridorBuildAnchor b{CorridorEndpointKind::Habitat,1,right.position-Vec3{rm.footprint_x,0,0}};
            const auto c=app.corridor_network_.preview_built_corridor(a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(c.has_value(),"close structure corridor unavailable");
            require(c->path.size()==2,"close structure corridor was bent/twisted instead of straight");
            const Vec3 direct=eng::normalize(c->path.back()-c->path.front());
            require(std::abs(direct.y)<0.01f,"close straight corridor changed elevation across the short gap");
        }

        for(int size=0;size<4;++size) for(int port=0;port<4;++port){
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat air;air.kind=HabitatKind::AirLock;air.position={-22,0,0};air.construction.step=kConstructionSteps;app.placed_habitats_={air};
            PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.storage_size=(StorageDomeSize)size;storage.position={22,0,0};storage.rotation_degrees=37;storage.construction.step=kConstructionSteps;app.placed_resources_={storage};
            const auto& mesh=app.resource_models_.mesh(storage);
            const float angle=(float)port*3.14159265f*0.5f;
            const Vec3 normal=world_math::rotate_y({std::sin(angle),0,-std::cos(angle)},storage.rotation_degrees);
            const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,{-16,0,0}},b{CorridorEndpointKind::Resource,0,storage.position+normal*mesh.footprint_x};
            auto c=app.corridor_network_.preview_built_corridor(a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(c.has_value(),"gate-less storage dome rejected a normal hull corridor");c->construction.step=kCorridorConstructionSteps;app.corridor_network_.restore_connections({*c});
            const std::string storage_label=std::to_string(size)+":"+std::to_string(port);
            check_walk(c->endpoint_b.position,c->endpoint_b.door.position,"storage ramp "+storage_label);
            Vec3 aisle=c->endpoint_b.door.wall_position-c->endpoint_b.door.normal*1.10f;
            aisle.y=storage.position.y+mesh.walkable_floor_y;
            check_walk(c->endpoint_b.door.position,aisle,"storage entry aisle "+storage_label);
            check_walk(aisle,c->endpoint_b.door.position,"storage exit aisle "+storage_label);
        }

        // POV collision must use the visible interior geometry instead of only
        // the pressure shell. Every habitat type must contain at least one
        // blocking furniture/equipment point while still retaining a clear
        // walkable interior point. This catches regressions where tables, beds,
        // hydroponics racks, tanks or machinery become ghost geometry again.
        for(int kind=0;kind<(int)HabitatKind::Count;++kind){
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat h;h.kind=(HabitatKind)kind;h.size=HabitatSizeTier::Standard;h.construction.step=kConstructionSteps;
            app.placed_habitats_={h};const auto& hm=app.habitat_models_.mesh(h);
            bool blocked=false,clear=false;
            const auto probe_obstacles=[&](const std::vector<Vec3>& triangles){
                for(std::size_t i=0;i+2<triangles.size() && !blocked;i+=3){
                    const Vec3 a=triangles[i],b=triangles[i+1],c=triangles[i+2];
                    const float min_y=std::min({a.y,b.y,c.y}),max_y=std::max({a.y,b.y,c.y});
                    if(max_y<=hm.walkable_floor_y+0.20f || min_y>=hm.walkable_floor_y+1.65f) continue;
                    Vec3 local=(a+b+c)*(1.0f/3.0f);local.y=hm.walkable_floor_y;
                    if(std::abs(local.x)>hm.footprint_x*0.5f-0.75f || std::abs(local.z)>hm.footprint_z*0.5f-0.75f) continue;
                    Vec3 p=h.position+world_math::rotate_y(local,h.rotation_degrees);
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn(p,0);
                    if(app.first_person_environment_label()=="EXTERIOR / VACUUM") continue;
                    blocked=!app.first_person_move_allowed(p,p);
                }
            };
            probe_obstacles(hm.interior);probe_obstacles(hm.details);
            require(blocked,"POV can walk through interior geometry for habitat kind "+std::to_string(kind));
            const float hx=std::max(0.5f,hm.footprint_x*0.5f-0.85f),hz=std::max(0.5f,hm.footprint_z*0.5f-0.85f);
            for(float z=-hz;z<=hz && !clear;z+=0.40f)for(float x=-hx;x<=hx && !clear;x+=0.40f){
                Vec3 local{x,hm.walkable_floor_y,z};Vec3 p=h.position+world_math::rotate_y(local,h.rotation_degrees);
                app.first_person_.spawn(p,0);
                if(app.first_person_environment_label()=="EXTERIOR / VACUUM") continue;
                clear=app.first_person_move_allowed(p,p);
            }
            require(clear,"interior collision consumed every walkable point for habitat kind "+std::to_string(kind));
        }

        // Accessible resource interiors follow the same rule. Storage racks and
        // physical cargo packages are solid; the warehouse still keeps an aisle.
        {
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.storage_size=StorageDomeSize::Mid;storage.construction.step=kConstructionSteps;
            app.placed_resources_={storage};const auto& rm=app.resource_models_.mesh(storage);
            bool blocked=false,clear=false;
            for(std::size_t i=0;i+2<rm.details.size() && !blocked;i+=3){
                const Vec3 a=rm.details[i],b=rm.details[i+1],c=rm.details[i+2];
                const float min_y=std::min({a.y,b.y,c.y}),max_y=std::max({a.y,b.y,c.y});
                if(max_y<=rm.walkable_floor_y+0.20f || min_y>=rm.walkable_floor_y+1.65f) continue;
                Vec3 local=(a+b+c)*(1.0f/3.0f);local.y=rm.walkable_floor_y;
                if(std::hypot(local.x,local.z)>storage_dome_diameter_m(storage.storage_size)*0.35f) continue;
                const Vec3 p=storage.position+world_math::rotate_y(local,storage.rotation_degrees);
                blocked=!app.first_person_move_allowed(p,p);
            }
            require(blocked,"Storage Dome racks are not physical POV obstacles");
            const float r=storage_dome_diameter_m(storage.storage_size)*0.26f;
            for(float z=-r;z<=r && !clear;z+=0.35f)for(float x=-r;x<=r && !clear;x+=0.35f){
                if(x*x+z*z>r*r) continue;
                const Vec3 p=storage.position+Vec3{x,rm.walkable_floor_y,z};
                clear=app.first_person_move_allowed(p,p);
            }
            require(clear,"Storage Dome interior collision removed every aisle");
        }
        std::cout<<"120 habitat rotation/size cases, 16 gate-less storage connection approaches, exact apertures, solid interior equipment and walkable aisles passed\n";
    }
};
}

int main(int argc,char** argv){
    try{
        if(argc>1 && std::string(argv[1])=="global-routes") game::NavigationRegression::global_routes();
        else if(argc>2 && std::string(argv[1])=="saved-route") game::NavigationRegression::saved_route(argv[2]);
        else if(argc>2 && std::string(argv[1])=="profile") game::NavigationRegression::profile(argv[2]);
        else if(argc>1 && std::string(argv[1])=="operation-relocation") game::NavigationRegression::operation_relocation();
        else if(argc>1 && std::string(argv[1])=="fault-recovery"){
            fault_recovery_case("single fault",1);
            fault_recovery_case("simultaneous fleet faults",12);
            fault_recovery_case("persistent geometry fault",1,true);
            fault_recovery_case("working hold fault",1,false,true);
            fault_recovery_case("bounded recovery search",1,false,false,true);
            fault_bounds_case();
        }
        else if(argc>1 && std::string(argv[1])=="ore-carrier") game::NavigationRegression::ore_carrier();
        else if(argc>1 && std::string(argv[1])=="conveyor-ore") game::NavigationRegression::conveyor_ore();
        else if(argc>1 && std::string(argv[1])=="structure-routes") game::NavigationRegression::structure_routes();
        else if(argc>1 && std::string(argv[1])=="interaction") game::NavigationRegression::interaction();
        else if(argc>1 && std::string(argv[1])=="oxygen") game::NavigationRegression::oxygen();
        else if(argc>1 && std::string(argv[1])=="progression") game::NavigationRegression::progression();
        else if(argc>1 && std::string(argv[1])=="battery-bootstrap") game::NavigationRegression::battery_bootstrap();
        else if(argc>1 && std::string(argv[1])=="hydrogen-solar") game::NavigationRegression::hydrogen_solar();
        else if(argc>1 && std::string(argv[1])=="hydrogen-refueling") game::NavigationRegression::hydrogen_refueling();
        else if(argc>1 && std::string(argv[1])=="debug-orders") game::NavigationRegression::debug_orders();
        else if(argc>1 && std::string(argv[1])=="priority") game::NavigationRegression::build_priority();
        else if(argc>1 && std::string(argv[1])=="airlock-cycle") game::NavigationRegression::airlock_cycle();
        else if(argc>1 && std::string(argv[1])=="airlock") game::NavigationRegression::airlock_connections();
        else if(argc>1 && std::string(argv[1])=="hydroponics") game::NavigationRegression::hydroponics();
        else if(argc>1 && std::string(argv[1])=="power-switches") game::NavigationRegression::power_switches();
        else if(argc>1 && std::string(argv[1])=="queue-flow") game::NavigationRegression::queue_and_flow();
        else if(argc>1 && std::string(argv[1])=="network") game::NavigationRegression::supply_network();
        else if(argc>1 && std::string(argv[1])=="supply") game::NavigationRegression::supply();
        else if(argc>1 && std::string(argv[1])=="extensions") game::NavigationRegression::storage_extensions();
        else if(argc>1 && std::string(argv[1])=="workshops") game::NavigationRegression::workshops();
        else if(argc>1 && std::string(argv[1])=="utilities") game::NavigationRegression::utilities();
        else if(argc>1 && std::string(argv[1])=="gates") game::NavigationRegression::gates();
        else if(argc>1 && std::string(argv[1])=="geometry") game::NavigationRegression::geometry();
        else if(argc>2 && std::string(argv[1])=="fault-construction") game::NavigationRegression::construction(argv[2],true,true);
        else if(argc>2 && std::string(argv[1])=="construction") game::NavigationRegression::construction(argv[2]);
        else if(argc>2 && std::string(argv[1])=="clearance-save") game::NavigationRegression::clearance_save(argv[2]);
        else if(argc>2 && std::string(argv[1])=="stuck-carrier-save") game::NavigationRegression::stuck_carrier_save(argv[2]);
        else if(argc>2 && std::string(argv[1])=="airlock-supply-save") game::NavigationRegression::airlock_supply_save(argv[2]);
        else if(argc>2 && std::string(argv[1])=="enclosure-save") game::NavigationRegression::enclosure_save(argv[2]);
        else if(argc>2 && std::string(argv[1])=="pending-enclosure-save") game::NavigationRegression::pending_enclosure_save(argv[2]);
        else if(argc>2 && std::string(argv[1])=="inspect") game::NavigationRegression::inspect(argv[2]);
        else if(argc>2 && std::string(argv[1])=="late") game::NavigationRegression::construction(argv[2],true);
        else {
            fleet_case("open straight",{{-14,0,0}},{{14,0,0}},{});
            fleet_case("structure corner",{{-12,0,-5}},{{12,0,5}},{{-5,5,-3,3}});
            fleet_case("appearing obstacle",{{-14,0,0}},{{14,0,0}},{},75,true);
            fleet_case("occupied service point",{{-10,0,0},{0,0,0}},{{0,0,0},{0,0,0}},{},75,false,true);
            fleet_case("opposing rovers",{{-14,0,0},{14,0,0}},{{14,0,0},{-14,0,0}},{});
            fleet_case("single-lane yielding",{{-16,0,0},{16,0,0}},{{16,0,0},{-16,0,0}},
                {{-8,8,-40,-1.5f},{-8,8,1.5f,40}},100.0f);
            fleet_case("six-way crossing",{{-16,0,-3},{16,0,3},{-3,0,-16},{3,0,16},{-16,0,10},{16,0,-10}},
                {{16,0,-3},{-16,0,3},{-3,0,16},{3,0,-16},{16,0,10},{-16,0,-10}},{});
        }
    }catch(const std::exception& e){ std::cerr<<"FAIL: "<<e.what()<<"\n";return 1; }
}
