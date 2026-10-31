#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "game/robots/safety_coordinator.hpp"
#include "game/util/world_math.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <filesystem>
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
    bool initialize(void*,unsigned long,int,int) override{return true;}
    void resize(int,int) override{} void begin_frame(eng::Color) override{} void end_frame() override{}
    void set_3d(const eng::Mat4&) override{} void set_2d(float) override{}
    void draw_lines_3d(const std::vector<Vec3>&,eng::Color,float) override{}
    void draw_triangles_3d(const std::vector<Vec3>& vertices,eng::Color) override{triangles.insert(triangles.end(),vertices.begin(),vertices.end());}
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
}

namespace game {
struct NavigationRegression {
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
    static void construction(const std::string& path,bool late_corridor=false){
        eng::Application engine;GameApp app(engine);
        app.playfield_.rebuild(64);app.starting_robot_count_=3;app.starting_carrier_count_=3;
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
        for(int frame=0;t<1800.0f;++frame,t+=1.0f/60){
            const auto begin=std::chrono::steady_clock::now();
            app.update_construction(1.0f/60);
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
                if(r.safety.mode==SafetyMode::Fault){
                    std::string save_error;app.save_game_to_name("navigation-regression-fault",save_error);
                    for(std::size_t k=0;k<app.builder_robots_.size();++k){const auto& bot=app.builder_robots_[k];
                        std::cerr<<"fault state "<<k<<" "<<bot.position.x<<","<<bot.position.z<<" heading="<<bot.heading_degrees<<" speed="<<bot.current_speed_mps<<" phase="<<(int)bot.safety.fault_phase<<" mode="<<(int)bot.mode<<" target="<<(int)bot.target.kind<<":"<<bot.target.index<<" at "<<t<<"\n";
                    }
                }
                require(r.safety.mode!=SafetyMode::Fault,"construction robot "+std::to_string(i)+": "+block_reason_name(r.safety.block_reason)+" phase "+std::to_string(r.safety.fault_phase));
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
    }
    static void workshops(){
        eng::Application engine;
        auto tick=[](GameApp& app){app.update_pressure_doors(1.0f/60);app.update_construction(1.0f/60);};
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
            for(std::size_t i=0;i<4;++i){auto& r=app.builder_robots_[i];
                r.position=workshop::world(w,{-7.5f+5.0f*(float)i,0,-20.0f});r.heading_degrees=rotation;
                r.mode=BuilderRobotMode::Idle;r.target={};r.condition=0.50f;}
            wait_parking(app,"four-bay rotation "+std::to_string(rotation));
            for(const auto& r:app.builder_robots_) require(r.condition>0.52f,"docked maintenance did not restore condition");
            const auto poses=app.builder_robots_;
            for(int i=0;i<240;++i) tick(app);
            for(std::size_t i=0;i<4;++i) require(distance(poses[i].position,app.builder_robots_[i].position)<0.02f,"parked rover wanders");
            // A new real job must preempt parking and open the gate for exit.
            PlacedResource target;target.kind=ResourceKind::SolarPanel;target.position={27,0,15};target.placement_sequence=1;
            target.resource_plan.total_steps=kConstructionSteps;
            for(auto& step:target.resource_plan.steps) step.amount(MaterialResource::Iron)=5;
            app.placed_resources_.push_back(target);
            GroundResourcePile pile;pile.position={22,0,-16};pile.amount=30;pile.resource=MaterialResource::Iron;pile.radius=0.4f;
            app.ground_resource_piles_={pile};ResourceBundle inventory;inventory.amount(MaterialResource::Iron)=30;app.resources_.restore(inventory);
            bool exited=false;
            for(int frame=0;frame<30000 && !app.placed_resources_[1].construction.complete();++frame){
                tick(app);verify(app);
                for(const auto& r:app.builder_robots_) if(r.target.valid() && !workshop::inside(app.placed_resources_[0],r.position)) exited=true;
            }
            require(exited && app.placed_resources_[1].construction.complete(),"parked fleet did not dispatch and complete new work");
            wait_parking(app,"return after task "+std::to_string(rotation));
            std::string error;require(app.save_game_to_name("workshop-regression",error),error);
            const auto saved=std::filesystem::path(app.save_game_directory())/"workshop-regression.gmsave";
            GameApp loaded(engine);require(loaded.load_game_from_file(saved.string(),error),error);
            wait_parking(loaded,"loaded workshop");std::filesystem::remove(saved);
        }
        GameApp placement(engine);placement.playfield_.rebuild(128);
        PlacedResource w;w.kind=ResourceKind::RobotWorkshop;
        require(placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,0},0),"open workshop placement rejected");
        require(!placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,-51},0),"ramp clearance crossed map boundary");
        PlacedResource obstacle;obstacle.kind=ResourceKind::PowerStorage;obstacle.position={0,0,-14};placement.placed_resources_={obstacle};
        require(!placement.resource_placement_valid(w.kind,ResourceSizeTier::Standard,StorageDomeSize::Small,{0,0,0},0),"workshop ramp can be placed against an obstacle");
        w.construction.step=kConstructionSteps;placement.placed_resources_={w};
        require(!placement.resource_placement_valid(ResourceKind::SolarPanel,ResourceSizeTier::Small,StorageDomeSize::Small,{0,0,-14},0),"new structure blocked reserved workshop ramp");
        require(!placement.placement_valid(HabitatKind::AirLock,HabitatSizeTier::Small,{0,0,-14},0),"new habitat blocked reserved workshop ramp");
        GeometryRecorder geometry;
        placement.render_workshop_gate(geometry,w);
        const auto front=workshop::world(w,{0,1.3f,-7.5f}),back=workshop::world(w,{0,1.3f,-6.5f});
        require(segment_hits_mesh(front,back,geometry.triangles),"closed workshop gate has no panel");
        w.workshop_gate_open=1;geometry.triangles.clear();placement.render_workshop_gate(geometry,w);
        require(!segment_hits_mesh(front,back,geometry.triangles),"open workshop gate still obstructs the centre");
        float last=0;
        for(int i=0;i<=100;++i){const float z=-12.0f+5.0f*(float)i/100;
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
        for(int frame=0;frame<36000 && !construction.placed_resources_[0].construction.complete();++frame){
            tick(construction);
            for(const auto& r:construction.builder_robots_){
                require(r.safety.mode!=SafetyMode::Fault,"workshop construction safety fault");
                if(!construction.placed_resources_[0].construction.complete()) require(r.workshop_index==kNoRobotIndex,"robot parked in an unfinished workshop");
            }
        }
        require(construction.placed_resources_[0].construction.complete(),"workshop construction did not finish");
        wait_parking(construction,"newly constructed workshop");
        std::cout<<"Workshop parking, task dispatch, ramp clearance, reachability, save/load and multi-workshop allocation passed\n";
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
                storage.rotation_degrees=rotation;storage.storage_loading_face=1;storage.construction.step=kConstructionSteps;
                storage.stored_boxes={{MaterialResource::Iron,100,1}};sync_stored_resource_totals(storage);
                app.placed_resources_={storage};app.placed_habitats_.clear();app.corridor_network_.clear();
                const auto service=storage_loading::service(storage);const auto n=storage_loading::normal(storage);
                require(storage_loading::reserved(storage,service),"gate service outside reservation");
                require(storage_loading::segment_reserved(storage,service-side*10,service+side*10,geometry::kCorridorHalfWidth),"crossing misses loading reservation");
                CorridorBuildAnchor cargo{CorridorEndpointKind::Resource,0,storage_loading::wall(storage)};
                CorridorBuildAnchor end{CorridorEndpointKind::OpenDoor,0,service+n*12};
                require(!app.corridor_network_.preview_anchor(cargo,end.position,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"cargo face preview allowed");
                require(!app.corridor_network_.build_corridor(cargo,end,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"cargo face build allowed");
                for(int face:{0,2,3}){
                    const auto direction=world_math::rotate_y(storage_loading::local_normal(face),rotation);
                    const auto wall=storage.position+direction*(storage_dome_diameter_m(storage.storage_size)*(5.35f/13.0f));
                    const CorridorBuildAnchor crew{CorridorEndpointKind::Resource,0,wall};
                    const CorridorBuildAnchor far{CorridorEndpointKind::OpenDoor,0,wall+direction*14};
                    app.corridor_network_.clear();
                    require(app.corridor_network_.build_corridor(crew,far,{},app.habitat_models_,app.placed_resources_,app.resource_models_),"crew port build rejected");
                }
                app.corridor_network_.clear();
                CorridorBuildAnchor cross_a{CorridorEndpointKind::OpenDoor,0,service-side*12},cross_b{CorridorEndpointKind::OpenDoor,0,service+side*12};
                auto route=app.corridor_network_.preview_built_corridor(cross_a,cross_b,{},app.habitat_models_,app.placed_resources_,app.resource_models_);
                if(route) for(std::size_t j=1;j<route->path.size();++j)
                    require(!storage_loading::segment_reserved(storage,route->path[j-1],route->path[j],geometry::kCorridorHalfWidth),"bypass clips cargo bay");
                BuilderRobotState bot;bot.role=RobotRole::Carrier;
                ResourceBundle need;need.amount(MaterialResource::Iron)=20;
                bot.position=storage.position-n*(storage_dome_diameter_m(storage.storage_size)*0.5f+1.55f);
                require(!app.robot_load_needed_resources(bot,need),"pickup through wrong storage wall");
                bot.position=service;
                require(app.robot_load_needed_resources(bot,need),"cargo gate pickup failed");
                require(bot.cargo.amount(MaterialResource::Iron)==20 && app.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==80,"pickup changed inventory incorrectly");
                // An installed legacy +X corridor moves the new gate to a free
                // face once, and further synchronization never moves it again.
                app.placed_resources_[0].storage_loading_face=-1;
                CorridorConnection legacy;legacy.endpoint_a.kind=CorridorEndpointKind::Resource;legacy.endpoint_a.index=0;legacy.endpoint_a.door.normal=n;
                app.corridor_network_.restore_connections({legacy});app.assign_storage_loading_faces();
                const int chosen=app.placed_resources_[0].storage_loading_face;
                require(chosen!=1,"legacy corridor overwritten by cargo hatch");
                app.corridor_network_.clear();app.assign_storage_loading_faces();
                require(app.placed_resources_[0].storage_loading_face==chosen,"cargo hatch changed face");
            }
        }
        // Trace the actual triangles submitted by the world renderer. Closed
        // leaves cover the full opening; fully retracted leaves clear it.
        app.placed_resources_.clear();app.pressure_doors_.clear();
        PlacedHabitat air;air.kind=HabitatKind::AirLock;air.construction.step=kConstructionSteps;
        app.placed_habitats_={air};const auto& mesh=app.habitat_models_.mesh(air);
        CorridorConnection c;c.construction.step=kCorridorConstructionSteps;
        c.endpoint_a.kind=CorridorEndpointKind::Habitat;c.endpoint_a.index=0;
        auto& hatch=c.endpoint_a.door;hatch.entrance_index=0;hatch.normal=mesh.entrances[0].local_normal;
        hatch.position=hatch.wall_position=mesh.entrances[0].local_position;
        app.corridor_network_.restore_connections({c});app.first_person_mode_=GameApp::FirstPersonMode::Off;
        app.update_pressure_doors(0.1f);
        const auto doors=app.habitat_doors(0);require(doors.size()==2,"Air Lock needs two pressure doors");
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
        const auto outer=doors[1];app.first_person_.spawn(outer.wall_position+outer.normal,0);
        for(int i=0;i<80;++i){
            app.update_pressure_doors(0.025f);
            const float inner_open=app.first_person_door_open_fraction(hatch.wall_position);
            const float outer_open=app.first_person_door_open_fraction(outer.wall_position);
            require(inner_open==0 || outer_open==0,"Air Lock opened both seals together");
        }
        require(app.first_person_door_open_fraction(outer.wall_position)==1,"Air Lock did not release next door after sealing");
        app.first_person_mode_=GameApp::FirstPersonMode::Off;
        // Cargo openings expose only a buffer: its inner pressure wall remains
        // solid with the outer shutters fully retracted.
        PlacedResource lock;lock.kind=ResourceKind::StorageDome;lock.storage_loading_face=1;lock.construction.step=kConstructionSteps;
        app.placed_resources_={lock};app.storage_gate_open_={1};draw.triangles.clear();app.render_resources(draw);
        const auto mouth=storage_loading::mouth(lock)+Vec3{0,0.8f,0};
        const auto back=storage_loading::wall(lock)+Vec3{0,0.8f,0};
        require(!segment_hits_mesh(mouth-Vec3{0.25f,0,0},mouth+Vec3{0.25f,0,0},draw.triangles),"open cargo shutter obstructed");
        require(segment_hits_mesh(back-Vec3{0.2f,0,0},back+Vec3{0.2f,0,0},draw.triangles),"cargo gate exposes warehouse atmosphere");

        // Two real CR-01s must deposit through the same loading bay without
        // colliding, parking at another wall, or losing/duplicating material.
        app.playfield_.rebuild(128);app.starting_robot_count_=1;app.starting_carrier_count_=2;
        app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_.clear();
        PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.storage_loading_face=1;storage.construction.step=kConstructionSteps;
        app.placed_resources_={storage};app.placed_habitats_.clear();app.corridor_network_.clear();
        const auto gate=storage_loading::service(storage);
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
        app.placed_resources_[0].storage_loading_face=3;
        std::string error;require(app.save_game_to_name("gate-regression",error),error);
        const auto saved=std::filesystem::path(app.save_game_directory())/"gate-regression.gmsave";
        GameApp restored(engine);require(restored.load_game_from_file(saved.string(),error),error);
        require(restored.placed_resources_[0].storage_loading_face==3 && restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)==60,"gate save round trip failed");
        std::filesystem::remove(saved);
        PlacedResource panel;panel.kind=ResourceKind::SolarPanel;panel.position={20,0,10};panel.placement_sequence=1;
        panel.resource_plan.total_steps=kConstructionSteps;
        for(auto& step:panel.resource_plan.steps) step.amount(MaterialResource::Iron)=5;
        restored.placed_resources_.push_back(panel);
        ResourceBundle physical;physical.amount(MaterialResource::Iron)=60;restored.resources_.restore(physical);
        for(int frame=0;frame<18000 && !restored.placed_resources_[1].construction.complete();++frame){
            const auto before=restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron);
            restored.update_construction(1.0f/60);
            if(restored.placed_resources_[0].stored_resources.amount(MaterialResource::Iron)<before){
                bool at_gate=false;
                for(const auto& bot:restored.builder_robots_) at_gate=at_gate || storage_loading::at_service(restored.placed_resources_[0],bot.position);
                require(at_gate,"autonomous pickup bypassed the cargo gate");
            }
            for(const auto& bot:restored.builder_robots_) require(bot.safety.mode!=SafetyMode::Fault,"storage construction safety fault");
        }
        require(restored.placed_resources_[1].construction.complete(),"construction could not withdraw through cargo gate");
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
                const Vec3 q=a+(b-a)*((float)(j-1)/n);
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                app.first_person_.spawn({q.x,app.first_person_floor_height(q),q.z},0);
                for(int tick=0;tick<5;++tick) app.update_pressure_doors(0.10f);
                require(app.first_person_move_allowed(q,p),label+": blocked at "+std::to_string(p.x)+","+std::to_string(p.z));
                require(std::abs(p.y-app.first_person_floor_height(q))<0.21f,label+": floor discontinuity at "+std::to_string(p.x)+","+std::to_string(p.z)+" y="+std::to_string(p.y)+" previous="+std::to_string(app.first_person_floor_height(q)));
            }
        };
        const std::vector<Vec3> wall{{0,0,-5},{0,4,5},{0,0,5},{0,0,-5},{0,4,-5},{0,4,5}};
        const auto carved=mesh_portals::subtract(wall,{{0,0,0},{1,0,0},0.5f,0.7f,0.5f,2.95f});
        float area=0;for(std::size_t i=0;i+2<carved.size();i+=3) area+=eng::length(eng::cross(carved[i+1]-carved[i],carved[i+2]-carved[i]))*0.5f;
        require(std::abs(area-(40.0f-1.4f*2.45f))<0.001f,"exact portal aperture area");
        for(int kind=0;kind<(int)HabitatKind::Count;++kind) for(int tier=0;tier<(int)HabitatSizeTier::Count;++tier) for(float rotation:{180.0f,135.0f,90.0f}){
            app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
            PlacedHabitat a;a.kind=HabitatKind::AirLock;a.position={-22,0,0};a.construction.step=kConstructionSteps;
            PlacedHabitat b;b.kind=(HabitatKind)kind;b.size=(HabitatSizeTier)tier;b.position={22,0,0};b.rotation_degrees=rotation;b.construction.step=kConstructionSteps;
            app.placed_habitats_={a,b};
            const auto& mesh=app.habitat_models_.mesh(b);
            for(const auto& e:mesh.entrances) require(std::abs(e.local_position.y-mesh.walkable_floor_y)<0.001f,"hatch floor");
            const CorridorBuildAnchor ca{CorridorEndpointKind::Habitat,0,{-16,0,4}};
            const CorridorBuildAnchor cb{CorridorEndpointKind::Habitat,1,{16,0,4}};
            auto c=app.corridor_network_.preview_built_corridor(ca,cb,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(c.has_value(),"corridor unavailable kind "+std::to_string(kind));
            require(!c->endpoint_a.door.generated && !c->endpoint_b.door.generated,"corridor not on authored entrance");
            c->construction.step=kCorridorConstructionSteps;app.corridor_network_.restore_connections({*c});
            const std::string label="habitat "+std::to_string(kind)+" tier "+std::to_string(tier);
            for(const auto& door:mesh.entrances){
                const Vec3 side{-door.local_normal.z,0,door.local_normal.x};
                for(float height:{0.15f,0.8f,geometry::kHumanHeight}) for(float shoulder:{-0.28f,0.0f,0.28f}){
                    Vec3 from=door.local_position+side*shoulder+Vec3{0,height,0};
                    Vec3 to=side*shoulder+Vec3{0,mesh.walkable_floor_y+height,0};
                    require(!segment_hits_mesh(from,to,mesh.interior),label+": interior aisle obstructed");
                }
            }
            for(const auto* ep:{&c->endpoint_a,&c->endpoint_b}){
                const auto& h=app.placed_habitats_[ep->index];
                check_walk(ep->position,ep->door.position,label+" ramp");
                check_walk(ep->door.position,h.position,label+" inside");
                check_walk(h.position,ep->door.position,label+" return");
            }
            for(std::size_t j=1;j<c->path.size();++j) check_walk(c->path[j-1],c->path[j],label+" corridor");
            const auto& am=app.habitat_models_.mesh(a);
            const auto& exit=am.entrances[1];
            const Vec3 hatch=a.position+exit.local_position;
            check_walk(hatch+exit.local_normal*2.5f,hatch,label+" exterior stair");
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
            if(port==1){require(!c.has_value(),"cargo face accepted a corridor");continue;}
            require(c.has_value(),"storage corridor");c->construction.step=kCorridorConstructionSteps;app.corridor_network_.restore_connections({*c});
            check_walk(c->endpoint_b.position,c->endpoint_b.door.position,"storage ramp "+std::to_string(size)+":"+std::to_string(port));
            check_walk(c->endpoint_b.door.position,storage.position,"storage interior");
            check_walk(storage.position,c->endpoint_b.position,"storage return");
        }
        std::cout<<"120 habitat rotation/size cases, 12 crew storage ports + 4 reserved cargo ports, exact apertures and interior headroom passed\n";
    }
};
}

int main(int argc,char** argv){
    try{
        if(argc>1 && std::string(argv[1])=="workshops") game::NavigationRegression::workshops();
        else if(argc>1 && std::string(argv[1])=="gates") game::NavigationRegression::gates();
        else if(argc>1 && std::string(argv[1])=="geometry") game::NavigationRegression::geometry();
        else if(argc>2 && std::string(argv[1])=="construction") game::NavigationRegression::construction(argv[2]);
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
