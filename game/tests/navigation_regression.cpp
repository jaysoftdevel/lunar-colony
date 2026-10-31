#include "game/game_app.hpp"
#include "game/robots/safety_coordinator.hpp"
#include "game/util/world_math.hpp"
#include "game/world/mesh_portals.hpp"
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
    static void geometry(){
        eng::Application engine;
        GameApp app(engine);app.playfield_.rebuild(256);
        const auto check_walk=[&](Vec3 a,Vec3 b,const std::string& label){
            const int n=std::max(1,(int)std::ceil(distance(a,b)/0.04f));
            a.y=app.first_person_floor_height(a);
            for(int j=1;j<=n;++j){
                Vec3 p=a+(b-a)*((float)j/n);p.y=app.first_person_floor_height(p);
                const Vec3 q=a+(b-a)*((float)(j-1)/n);
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
            require(c.has_value(),"storage corridor");c->construction.step=kCorridorConstructionSteps;app.corridor_network_.restore_connections({*c});
            check_walk(c->endpoint_b.position,c->endpoint_b.door.position,"storage ramp "+std::to_string(size)+":"+std::to_string(port));
            check_walk(c->endpoint_b.door.position,storage.position,"storage interior");
            check_walk(storage.position,c->endpoint_b.position,"storage return");
        }
        std::cout<<"120 habitat rotation/size cases, 16 storage ports, exact apertures and interior headroom passed\n";
    }
};
}

int main(int argc,char** argv){
    try{
        if(argc>1 && std::string(argv[1])=="geometry") game::NavigationRegression::geometry();
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
