#include "game/game_app.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/robots/path_planner.hpp"

namespace game {
bool GameApp::workshop_supplied(std::size_t resource_index) const {
    if(resource_index>=placed_resources_.size() || placed_resources_[resource_index].kind!=ResourceKind::RobotWorkshop) return false;
    for(const auto& link:utility_network_.connections()){
        if(link.kind!=UtilityKind::SupplyChannel || !link.construction.complete()) continue;
        const auto attached=[&](const UtilityEndpoint& endpoint){
            return endpoint.kind==UtilityEndpointKind::Resource && endpoint.index==resource_index;
        };
        if(attached(link.endpoint_a) || attached(link.endpoint_b)) return true;
    }
    return false;
}

bool GameApp::workshop_operational(std::size_t resource_index) const {
    return resource_index<placed_resources_.size() && placed_resources_[resource_index].kind==ResourceKind::RobotWorkshop &&
           placed_resources_[resource_index].construction.complete() && workshop_supplied(resource_index);
}

void GameApp::update_workshop_gates(float dt){
    dt=std::clamp(dt,0.0f,0.10f);
    for(std::size_t index=0;index<placed_resources_.size();++index){
        auto& item=placed_resources_[index];
        if(item.kind!=ResourceKind::RobotWorkshop || !item.construction.complete()) continue;
        bool requested=false,occupied=false;
        for(const auto& robot:builder_robots_){
            const auto p=workshop::local(item,robot.position);
            const bool threshold=std::abs(p.z+workshop::kHalfZ)<2.60f && std::abs(p.x)<workshop::kGateHalf+1.50f;
            occupied=occupied || threshold;
            if(robot.workshop_index==index && robot.workshop_returning) requested=true;
            if(workshop::inside(item,robot.position) &&
               (!robot.workshop_parked || robot.target.valid() || robot.action!=BuilderRobotAction::None)) requested=true;
            if(workshop::approach_hits(item,robot.position,robot.position,1.5f) &&
               (robot.route_active || std::abs(robot.current_speed_mps)>0.02f)) requested=true;
        }
        if(first_person_mode_==FirstPersonMode::Active){
            const auto p=workshop::local(item,first_person_.feet_position());
            requested=requested || (std::abs(p.z+workshop::kHalfZ)<4.0f && std::abs(p.x)<4.0f);
            occupied=occupied || (std::abs(p.z+workshop::kHalfZ)<1.2f && std::abs(p.x)<3.0f);
        }
        // A loaded save may contain a rover straddling the threshold. Restore
        // an open gate before it becomes authoritative collision geometry.
        if(!item.workshop_gate_initialized){
            if(occupied) item.workshop_gate_open=1;
            item.workshop_gate_initialized=true;
        }
        if(requested || occupied) item.workshop_gate_hold=0.85f;
        else item.workshop_gate_hold=std::max(0.0f,item.workshop_gate_hold-dt);
        const bool open=requested || occupied || item.workshop_gate_hold>0;
        item.workshop_gate_open=std::clamp(item.workshop_gate_open+(open?2.80f:-1.40f)*dt,0.0f,1.0f);
    }
}

bool GameApp::workshop_accessible(const PlacedResource& candidate) const {
    if(builder_robots_.empty()) return true;
    // Cache the static reachability query while the pointer remains on the
    // same snapped footprint. Never re-run a map search every render frame.
    std::uint64_t key=1469598103934665603ULL;
    auto mix=[&](std::uint64_t v){key^=v+0x9e3779b97f4a7c15ULL+(key<<6)+(key>>2);};
    auto pose=[&](eng::Vec3 p,float rotation){
        mix((std::int64_t)std::llround(p.x*100));mix((std::int64_t)std::llround(p.z*100));mix((std::int64_t)std::llround(rotation*100));
    };
    pose(candidate.position,candidate.rotation_degrees);mix(playfield_.size());
    for(const auto& h:placed_habitats_){pose(h.position,h.rotation_degrees);mix((int)h.kind);mix((int)h.size);}
    for(const auto& r:placed_resources_){pose(r.position,r.rotation_degrees);mix((int)r.kind);mix((int)r.size);mix((int)r.storage_size);mix(r.construction.step);}
    for(const auto& c:corridor_network_.connections()) for(auto p:c.path) pose(p,0);
    for(const auto& p:ground_resource_piles_){pose(p.position,0);mix((int)std::llround(p.radius*100));}
    mix(builder_robots_.size());
    for(const auto& robot:builder_robots_){
        pose({std::floor(robot.position.x/8)*8,0,std::floor(robot.position.z/8)*8},0);
    }
    struct Cache {const GameApp* owner;std::uint64_t key;bool reachable;};
    static std::vector<Cache> cache;
    for(const auto& entry:cache) if(entry.owner==this && entry.key==key) return entry.reachable;
    constexpr float radius=1.12f;
    const float half=(float)playfield_.size()*0.5f-radius;
    auto point_clear=[&](eng::Vec3 p){
        if(std::abs(p.x)>half || std::abs(p.z)>half) return false;
        auto box_blocks=[&](const auto& item,const auto& mesh){
            const auto q=world_math::rotate_y(p-item.position,-item.rotation_degrees);
            return std::abs(q.x)<mesh.footprint_x*0.5f+radius && std::abs(q.z)<mesh.footprint_z*0.5f+radius;
        };
        if(box_blocks(candidate,resource_models_.mesh(candidate))) return false;
        for(const auto& h:placed_habitats_) if(box_blocks(h,habitat_models_.mesh(h))) return false;
        for(const auto& r:placed_resources_){
            if(r.kind==ResourceKind::RobotWorkshop && r.construction.complete()){
                auto open=r;open.workshop_gate_open=1;
                if(!workshop::clear(open,p,p,radius)) return false;
            }else if(box_blocks(r,resource_models_.mesh(r))) return false;
        }
        for(const auto& c:corridor_network_.connections()) for(std::size_t j=1;j<c.path.size();++j)
            if(world_math::point_segment_distance_xz(p,c.path[j-1],c.path[j])<geometry::kCorridorHalfWidth+radius) return false;
        for(const auto& pile:ground_resource_piles_)
            if(world_math::distance_xz(p,pile.position)<pile.radius+radius) return false;
        return true;
    };
    const auto start=workshop::world(candidate,{0,0,workshop::kApproachFar+1.0f});
    eng::Vec3 goal{};float best=std::numeric_limits<float>::max();
    for(const auto& robot:builder_robots_){
        if(!point_clear(robot.position)) continue;
        const float d=world_math::distance_xz(start,robot.position);
        if(d<best){best=d;goal=robot.position;}
    }
    bool reachable=false;
    if(best<std::numeric_limits<float>::max()){
        robots::PathPlannerCallbacks callbacks;
        callbacks.floor_height=[](eng::Vec3){return 0.0f;};callbacks.point_clear=point_clear;
        callbacks.segment_clear=[&](eng::Vec3 a,eng::Vec3 b){
            const int n=std::max(1,(int)std::ceil(world_math::distance_xz(a,b)/0.30f));
            for(int j=0;j<=n;++j) if(!point_clear(a+(b-a)*((float)j/n))) return false;
            return true;
        };
        robots::PathPlanRequest request;request.start=start;request.goal=goal;request.stop_radius=0.50f;
        request.cell_size=1.0f;request.corridor_half_width=half*2;request.max_expansions=8192;
        std::vector<eng::Vec3> route;reachable=robots::plan_corridor_path(request,callbacks,route);
    }
    if(cache.size()>64) cache.clear();
    cache.push_back({this,key,reachable});
    return reachable;
}
}
