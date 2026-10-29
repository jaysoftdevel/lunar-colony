#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/robots/path_planner.hpp"
#include "game/util/world_math.hpp"
#include "game/world/construction_sequence.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

namespace game {
namespace {
using eng::Vec3;

constexpr float PI=3.14159265358979323846f;

float dist_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

void add_bundle(ResourceBundle& dst,const ResourceBundle& src){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto r=static_cast<MaterialResource>(i);
        dst.amount(r)+=src.amount(r);
    }
}
void subtract_bundle(ResourceBundle& dst,const ResourceBundle& src){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto r=static_cast<MaterialResource>(i);
        dst.amount(r)=std::max<std::int64_t>(0,dst.amount(r)-src.amount(r));
    }
}
ResourceBundle positive_difference(const ResourceBundle& a,const ResourceBundle& b){
    ResourceBundle out{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto r=static_cast<MaterialResource>(i);
        out.amount(r)=std::max<std::int64_t>(0,a.amount(r)-b.amount(r));
    }
    return out;
}
bool contains_bundle(const ResourceBundle& have,const ResourceBundle& need){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto r=static_cast<MaterialResource>(i);
        if(have.amount(r)<need.amount(r)) return false;
    }
    return true;
}
std::int64_t free_capacity(const BuilderRobotState& robot){
    return std::max<std::int64_t>(0,robot.payload_capacity_kg-robot.cargo.total_amount());
}

constexpr float kRobotObstacleMargin=0.10f;
constexpr float kRobotHullClearanceM=0.10f;
constexpr float kPickupSeconds=1.45f;
constexpr float kPutDownSeconds=1.35f;
constexpr float kApplySeconds=1.50f;
constexpr float kConstructionWorkSpeedMultiplier=1.50f;
constexpr float kWeldSeconds=2.20f/kConstructionWorkSpeedMultiplier;
constexpr float kBuildUpSeconds=1.80f/kConstructionWorkSpeedMultiplier;
// A stage no longer forces a full four-side lap. Two opposite work positions
// provide visible perimeter work while substantially reducing non-productive
// driving; the selected pair rotates between construction stages.
constexpr int kBuildWorkSectorsPerStage=2;
constexpr float kRobotAccelerationMps2=2.4f*kRobotMovementSpeedMultiplier;
constexpr float kRobotBrakingMps2=3.2f*kRobotMovementSpeedMultiplier;
constexpr float kRobotTurnRateDegreesPerSecond=125.0f*kRobotMovementSpeedMultiplier;
constexpr float kBuildTravelSpeedScale=0.34f;
constexpr float kPickupWorkZoneM=0.30f;
constexpr float kDeliveryWorkZoneM=0.42f;
constexpr float kBuildWorkZoneM=0.34f;
constexpr float kOperationWatchdogSeconds=3.25f;
constexpr float kOperationHardRecoverySeconds=6.50f;

float wrap_radians(float a){
    while(a>PI) a-=2.0f*PI;
    while(a<-PI) a+=2.0f*PI;
    return a;
}
float approach(float value,float target,float max_delta){
    if(value<target) return std::min(target,value+max_delta);
    return std::max(target,value-max_delta);
}

void start_action(BuilderRobotState& robot,BuilderRobotAction action,float duration,
                  MaterialResource material=MaterialResource::Iron,std::int64_t amount=0,Vec3 focus={}){
    robot.action=action;
    robot.action_elapsed=0.0f;
    robot.action_duration=std::max(0.01f,duration);
    robot.action_material=material;
    robot.action_amount=std::max<std::int64_t>(0,amount);
    robot.action_focus_position=focus;
}

bool tick_action(BuilderRobotState& robot,float dt){
    if(robot.action==BuilderRobotAction::None) return true;
    robot.action_elapsed+=std::max(0.0f,dt);
    return robot.action_elapsed>=robot.action_duration;
}

void clear_action(BuilderRobotState& robot){
    robot.action=BuilderRobotAction::None;
    robot.action_elapsed=0.0f;
    robot.action_duration=0.0f;
    robot.action_amount=0;
    robot.action_focus_position={};
}

Vec3 point_along_path(const std::vector<Vec3>& path,float fraction){
    if(path.empty()) return {};
    if(path.size()==1) return path.front();
    fraction=std::clamp(fraction,0.0f,1.0f);
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i) total+=dist_xz(path[i],path[i+1]);
    if(total<0.001f) return path.front();
    float seek=total*fraction;
    for(std::size_t i=0;i+1<path.size();++i){
        const float seg=dist_xz(path[i],path[i+1]);
        if(seek<=seg || i+2==path.size()){
            const float t=seg>0.001f?std::clamp(seek/seg,0.0f,1.0f):0.0f;
            return path[i]+(path[i+1]-path[i])*t;
        }
        seek-=seg;
    }
    return path.back();
}

struct SourceChoice {
    bool valid{false};
    bool ground{true};
    std::size_t index{0};
    std::size_t box_index{0};
    MaterialResource material{MaterialResource::Iron};
    Vec3 position{};
    float distance{std::numeric_limits<float>::max()};
};

struct RobotFootprint {
    float half_width{0.62f};
    float half_length{0.72f};
};

RobotFootprint robot_footprint(RobotRole role){
    // Match the rendered wheel/chassis envelope rather than the smaller static
    // navigation radius. Fleet-vs-fleet collision is therefore based on the
    // actual visible hull, so two robots cannot visually interpenetrate even
    // while turning beside each other.
    return role==RobotRole::Carrier?RobotFootprint{0.62f,0.77f}:RobotFootprint{0.62f,0.72f};
}

float robot_resource_clearance_radius(RobotRole role){
    // Resource avoidance must account for the furthest visible hull corner.
    // The legacy robot_body_radius() is intentionally smaller for structure
    // navigation and allowed a wheel/chassis corner to pass over a crate even
    // when the rover centre stayed outside the crate radius.
    const RobotFootprint footprint=robot_footprint(role);
    return std::sqrt(footprint.half_width*footprint.half_width+
                     footprint.half_length*footprint.half_length);
}

bool robot_hulls_overlap(Vec3 a,float a_heading_degrees,RobotRole a_role,
                         Vec3 b,float b_heading_degrees,RobotRole b_role,float clearance){
    const RobotFootprint af=robot_footprint(a_role);
    const RobotFootprint bf=robot_footprint(b_role);
    const float ah=a_heading_degrees*PI/180.0f;
    const float bh=b_heading_degrees*PI/180.0f;
    const Vec3 ar{std::cos(ah),0.0f,-std::sin(ah)};
    const Vec3 afwd{std::sin(ah),0.0f,std::cos(ah)};
    const Vec3 br{std::cos(bh),0.0f,-std::sin(bh)};
    const Vec3 bfwd{std::sin(bh),0.0f,std::cos(bh)};
    const Vec3 d=b-a;
    const auto dot_xz=[](Vec3 u,Vec3 v){ return u.x*v.x+u.z*v.z; };
    const float extra=std::max(0.0f,clearance)*0.5f;
    const float aw=af.half_width+extra, al=af.half_length+extra;
    const float bw=bf.half_width+extra, bl=bf.half_length+extra;
    const std::array<Vec3,4> axes{{ar,afwd,br,bfwd}};
    for(const Vec3 axis:axes){
        const float center=std::abs(dot_xz(d,axis));
        const float ra=aw*std::abs(dot_xz(ar,axis))+al*std::abs(dot_xz(afwd,axis));
        const float rb=bw*std::abs(dot_xz(br,axis))+bl*std::abs(dot_xz(bfwd,axis));
        if(center>=ra+rb) return false;
    }
    return true;
}

float size_multiplier(HabitatSizeTier tier){
    switch(tier){
        case HabitatSizeTier::Standard: return 1.0f;
        case HabitatSizeTier::Mid: return 1.7f;
        case HabitatSizeTier::Big: return 2.6f;
        default: return 1.0f;
    }
}

float segment_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<1.0e-8f) return dist_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return dist_xz(p,{a.x+dx*t,0.0f,a.z+dz*t});
}

bool point_in_triangle_xz(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    // Use the signed 2-D area directly instead of the dot-product Gram
    // determinant. Perfectly vertical construction triangles collapse to a
    // line in X/Z; in single precision the old determinant could round to a
    // tiny *positive* value (for example 1.49e-8) and the barycentric solve
    // then classified points many metres away as inside the wall.  Those
    // degenerate projections are line obstacles and are already handled by the
    // segment-distance checks in the construction collision range test.
    const double abx=(double)b.x-(double)a.x;
    const double abz=(double)b.z-(double)a.z;
    const double acx=(double)c.x-(double)a.x;
    const double acz=(double)c.z-(double)a.z;
    const double area=abx*acz-abz*acx;
    const double scale=std::max({abx*abx+abz*abz,acx*acx+acz*acz,1.0});
    if(std::abs(area)<=1.0e-7*scale) return false;

    const double apx=(double)p.x-(double)a.x;
    const double apz=(double)p.z-(double)a.z;
    const double bcx=(double)c.x-(double)b.x;
    const double bcz=(double)c.z-(double)b.z;
    const double bpx=(double)p.x-(double)b.x;
    const double bpz=(double)p.z-(double)b.z;
    const double cax=(double)a.x-(double)c.x;
    const double caz=(double)a.z-(double)c.z;
    const double cpx=(double)p.x-(double)c.x;
    const double cpz=(double)p.z-(double)c.z;
    const double e0=abx*apz-abz*apx;
    const double e1=bcx*bpz-bcz*bpx;
    const double e2=cax*cpz-caz*cpx;
    constexpr double eps=1.0e-8;
    if(area>0.0) return e0>=-eps && e1>=-eps && e2>=-eps;
    return e0<=eps && e1<=eps && e2<=eps;
}

std::size_t revealed_triangle_vertex_count(const std::vector<Vec3>& triangles,float fraction){
    if(triangles.empty() || fraction<=0.0f) return 0;
    if(fraction>=1.0f) return (triangles.size()/3)*3;
    const std::size_t tri_count=triangles.size()/3;
    const std::size_t visible=std::min(tri_count,std::max<std::size_t>(1,(std::size_t)std::ceil((double)tri_count*fraction)));
    return visible*3;
}

template<class Transform>
bool construction_triangles_block_circle_range(const std::vector<Vec3>& tris,std::size_t first,std::size_t last,
                                               Vec3 candidate,float radius,Transform&& transform){
    first=(first/3)*3;
    last=std::min((last/3)*3,(tris.size()/3)*3);
    if(first>=last) return false;
    for(std::size_t i=first;i+2<last;i+=3){
        const Vec3 a=transform(tris[i]),b=transform(tris[i+1]),c=transform(tris[i+2]);
        const Vec3 ab=b-a,ac=c-a;
        const Vec3 n{ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x};
        const float nlen=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
        if(nlen<1.0e-6f) continue;
        // Horizontal decks/foundations are traversable surfaces. Only erected
        // walls, hulls, machinery and other rising geometry block a robot.
        if(std::abs(n.y)/nlen>0.78f) continue;
        const float ymin=std::min({a.y,b.y,c.y});
        const float ymax=std::max({a.y,b.y,c.y});
        if(ymax<0.30f || ymax-ymin<0.08f) continue;
        if(point_in_triangle_xz(candidate,a,b,c) ||
           segment_distance_xz(candidate,a,b)<radius ||
           segment_distance_xz(candidate,b,c)<radius ||
           segment_distance_xz(candidate,c,a)<radius) return true;
    }
    return false;
}

bool transformed_triangles_block_circle_range(const std::vector<Vec3>& tris,std::size_t first,std::size_t last,
                                              Vec3 position,float rotation_degrees,Vec3 candidate,float radius){
    return construction_triangles_block_circle_range(tris,first,last,candidate,radius,[&](Vec3 v){
        return world_math::rotate_y(v,rotation_degrees)+position;
    });
}

bool polyline_near_xz(Vec3 candidate,const std::vector<Vec3>& path,float clearance){
    if(path.empty()) return false;
    if(path.size()==1) return dist_xz(candidate,path.front())<=clearance;
    for(std::size_t i=0;i+1<path.size();++i)
        if(world_math::point_segment_distance_xz(candidate,path[i],path[i+1])<=clearance) return true;
    return false;
}

} // namespace

ResourceBundle GameApp::starter_resource_reserve() const {
    ResourceBundle total{};
    // One of every current habitat gives the player enough stock to establish
    // the complete functional chain rather than only a minimal survival pod.
    for(std::size_t i=0;i<(std::size_t)HabitatKind::Count;++i){
        const auto kind=(HabitatKind)i;
        add_bundle(total,construction_costs::habitat(kind,HabitatSizeTier::Standard,
                   habitat_models_.mesh(kind,HabitatSizeTier::Standard)).total());
    }
    // One of every external resource structure, plus duplicate generation and
    // extraction hardware so a sustainable base is not a single-point system.
    for(std::size_t i=0;i<(std::size_t)ResourceKind::Count;++i){
        const auto kind=(ResourceKind)i;
        const StorageDomeSize size=kind==ResourceKind::StorageDome?StorageDomeSize::Small:StorageDomeSize::Small;
        add_bundle(total,construction_costs::structure(kind,resource_models_.mesh(kind,size)).total());
        if(kind==ResourceKind::SolarPanel || kind==ResourceKind::OreDriller)
            add_bundle(total,construction_costs::structure(kind,resource_models_.mesh(kind,size)).total());
    }

    // Network reserve for a practical first base. Routed costs add terminal
    // hardware and bends at placement time; the contingency below covers that.
    add_bundle(total,construction_costs::corridor(180.0f).total());
    add_bundle(total,construction_costs::supply_channel(220.0f).total());
    add_bundle(total,construction_costs::walkway(120.0f).total());

    // 25% construction contingency for routing, junctions and player layout.
    for(std::size_t i=0;i<4;++i){
        const auto r=(MaterialResource)i;
        total.amount(r)=(total.amount(r)*125+99)/100;
    }

    // Consumable bootstrap reserve. Once Bio Dome + water chain + medical
    // chain are online the economy replaces these continuously.
    total.water=18000;
    total.food=4800;
    total.medical=900;
    return total;
}

void GameApp::reset_builder_robot_and_starter_stash(){
    production_fraction_.fill(0.0);
    life_support_fraction_seconds_=0.0;

    const float half=(float)playfield_.size()*0.5f;
    starter_stash_center_={-half+6.25f,0.0f,-half+7.50f};

    const int constructor_count=std::clamp(starting_robot_count_,1,6);
    const int carrier_count=std::clamp(starting_carrier_count_,1,6);
    builder_robots_.assign((std::size_t)(constructor_count+carrier_count),BuilderRobotState{});
    for(int i=0;i<constructor_count;++i){
        // Dedicated launch lanes: constructors face east toward the open field
        // and have enough bay spacing to leave without first rotating through a
        // neighbour. Tight 1.55 m parking in v0.31.17 was physically legal only
        // while perfectly aligned and created a departure deadlock as soon as a
        // rover tried to turn.
        const int column=i%3;
        const int row=i/3;
        auto& robot=builder_robots_[(std::size_t)i];
        robot.role=RobotRole::Constructor;
        robot.position={starter_stash_center_.x+3.20f+(float)column*2.20f,
                        0.0f,
                        starter_stash_center_.z-1.20f+(float)row*2.40f};
        robot.heading_degrees=90.0f;
        robot.visual_position=robot.position;
        robot.visual_heading_degrees=robot.heading_degrees;
        robot.visual_initialized=true;
        robot.move_speed_mps=kBuilderRobotTransitSpeedMps;
        robot.payload_capacity_kg=1200;
    }
    for(int i=0;i<carrier_count;++i){
        // CR-01 standby bays sit well east of the pallet traffic spine and
        // initially face the depot. Keeping parked carriers out of the merge
        // lanes is essential: an idle chassis must never become a permanent
        // obstacle for a loaded robot leaving a pickup lane.
        const int column=i%3;
        const int row=i/3;
        auto& carrier=builder_robots_[(std::size_t)constructor_count+(std::size_t)i];
        carrier.role=RobotRole::Carrier;
        carrier.position={starter_stash_center_.x+7.40f+(float)column*2.70f,
                          0.0f,
                          starter_stash_center_.z+9.00f+(float)row*2.80f};
        carrier.heading_degrees=-90.0f;
        carrier.visual_position=carrier.position;
        carrier.visual_heading_degrees=carrier.heading_degrees;
        carrier.visual_initialized=true;
        carrier.move_speed_mps=kCarrierRobotTransitSpeedMps;
        carrier.payload_capacity_kg=1800;
    }

    const ResourceBundle reserve=starter_resource_reserve();
    resources_.restore(reserve);
    ground_resource_piles_.clear();

    // Seven front-accessible aggregate pallet stacks, one per material. The
    // depot is inset far enough from the map corner to provide a full traffic
    // spine, and pallet lanes are separated by 2.05 m. That spacing is large
    // enough for a CR-01 to enter/leave one lane while another rover is stopped
    // in the neighbouring lane without their swept hulls blocking each other.
    constexpr float pallet_spacing=2.05f;
    const float center_index=((float)kMaterialResourceCount-1.0f)*0.5f;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=(MaterialResource)i;
        GroundResourcePile pile;
        pile.resource=material;
        pile.amount=reserve.amount(material);
        pile.position={starter_stash_center_.x-2.15f,0.0f,
                       starter_stash_center_.z+((float)i-center_index)*pallet_spacing};
        pile.radius=0.36f;
        pile.condition=1.0f;
        ground_resource_piles_.push_back(pile);
    }
}

const ConstructionResourcePlan* GameApp::construction_target_plan(const ConstructionTargetRef& target) const {
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            if(target.index<placed_habitats_.size()) return &placed_habitats_[target.index].resource_plan;
            break;
        case ConstructionTargetKind::Resource:
            if(target.index<placed_resources_.size()) return &placed_resources_[target.index].resource_plan;
            break;
        case ConstructionTargetKind::Corridor:
            if(target.index<corridor_network_.connections().size()) return &corridor_network_.connections()[target.index].resource_plan;
            break;
        case ConstructionTargetKind::Utility:
            if(target.index<utility_network_.connections().size()) return &utility_network_.connections()[target.index].resource_plan;
            break;
        case ConstructionTargetKind::Walkway:
            if(target.index<walkway_network_.connections().size()) return &walkway_network_.connections()[target.index].resource_plan;
            break;
        case ConstructionTargetKind::None: break;
    }
    return nullptr;
}

std::uint8_t GameApp::construction_target_step(const ConstructionTargetRef& target) const {
    switch(target.kind){
        case ConstructionTargetKind::Habitat: return target.index<placed_habitats_.size()?placed_habitats_[target.index].construction.step:0;
        case ConstructionTargetKind::Resource: return target.index<placed_resources_.size()?placed_resources_[target.index].construction.step:0;
        case ConstructionTargetKind::Corridor: return target.index<corridor_network_.connections().size()?corridor_network_.connections()[target.index].construction.step:0;
        case ConstructionTargetKind::Utility: return target.index<utility_network_.connections().size()?utility_network_.connections()[target.index].construction.step:0;
        case ConstructionTargetKind::Walkway: return target.index<walkway_network_.connections().size()?walkway_network_.connections()[target.index].construction.step:0;
        case ConstructionTargetKind::None: return 0;
    }
    return 0;
}

std::uint8_t GameApp::construction_target_total_steps(const ConstructionTargetRef& target) const {
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
        case ConstructionTargetKind::Resource: return kConstructionSteps;
        case ConstructionTargetKind::Corridor: return target.index<corridor_network_.connections().size()?corridor_network_.connections()[target.index].construction.total_steps:1;
        case ConstructionTargetKind::Utility: return target.index<utility_network_.connections().size()?utility_network_.connections()[target.index].construction.total_steps:1;
        case ConstructionTargetKind::Walkway: return target.index<walkway_network_.connections().size()?walkway_network_.connections()[target.index].construction.total_steps:1;
        case ConstructionTargetKind::None: return 1;
    }
    return 1;
}

bool GameApp::construction_target_complete(const ConstructionTargetRef& target) const {
    if(!target.valid()) return true;
    return construction_target_step(target)>=construction_target_total_steps(target);
}

bool GameApp::construction_target_dependencies_ready(const ConstructionTargetRef& target) const {
    if(!target.valid()) return false;
    if(target.kind!=ConstructionTargetKind::Corridor) return true;
    if(target.index>=corridor_network_.connections().size()) return false;

    const auto& connection=corridor_network_.connections()[target.index];
    auto endpoint_ready=[&](const CorridorEndpoint& endpoint){
        switch(endpoint.kind){
            case CorridorEndpointKind::Habitat:
                return endpoint.index<placed_habitats_.size() && placed_habitats_[endpoint.index].construction.complete();
            case CorridorEndpointKind::Resource:
                return endpoint.index<placed_resources_.size() && placed_resources_[endpoint.index].construction.complete();
            case CorridorEndpointKind::Corridor:
                return endpoint.index<corridor_network_.connections().size() &&
                       endpoint.index!=target.index && corridor_network_.connections()[endpoint.index].construction.complete();
            case CorridorEndpointKind::OpenDoor:
                return true;
        }
        return false;
    };
    return endpoint_ready(connection.endpoint_a) && endpoint_ready(connection.endpoint_b);
}

Vec3 GameApp::construction_target_position(const ConstructionTargetRef& target) const {
    const std::uint8_t step=construction_target_step(target);
    const std::uint8_t steps=construction_target_total_steps(target);
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            if(target.index<placed_habitats_.size()){
                const auto& h=placed_habitats_[target.index];
                const auto& m=habitat_models_.mesh(h);
                const Vec3 local{m.footprint_x*0.5f+1.0f,0.0f,0.0f};
                return h.position+world_math::rotate_y(local,h.rotation_degrees);
            }
            break;
        case ConstructionTargetKind::Resource:
            if(target.index<placed_resources_.size()){
                const auto& item=placed_resources_[target.index];
                const auto& m=resource_models_.mesh(item);
                const Vec3 local{m.footprint_x*0.5f+1.0f,0.0f,0.0f};
                return item.position+world_math::rotate_y(local,item.rotation_degrees);
            }
            break;
        case ConstructionTargetKind::Corridor:
            if(target.index<corridor_network_.connections().size()){
                const auto& c=corridor_network_.connections()[target.index];
                return point_along_path(c.path,((float)step+0.5f)/(float)std::max<std::uint8_t>(1,steps));
            }
            break;
        case ConstructionTargetKind::Utility:
            if(target.index<utility_network_.connections().size()){
                const auto& c=utility_network_.connections()[target.index];
                return point_along_path(c.path,((float)step+0.5f)/(float)std::max<std::uint8_t>(1,steps));
            }
            break;
        case ConstructionTargetKind::Walkway:
            if(target.index<walkway_network_.connections().size()){
                const auto& c=walkway_network_.connections()[target.index];
                return point_along_path(c.path,((float)step+0.5f)/(float)std::max<std::uint8_t>(1,steps));
            }
            break;
        case ConstructionTargetKind::None: break;
    }
    return {};
}

Vec3 GameApp::construction_drop_position(const ConstructionTargetRef& target) const {
    // One deterministic material handoff area per construction target. Keep
    // enough chassis clearance from the future wall envelope that a CR-01 can
    // unload and turn away while a BR-02 can approach the same pile from the
    // outside. All carriers therefore converge on one visible pile without
    // occupying the structure footprint itself.
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            if(target.index<placed_habitats_.size()){
                const auto& h=placed_habitats_[target.index];
                const auto& m=habitat_models_.mesh(h);
                // Keep logistics off the four cardinal wall-work lanes. The
                // single pile lives on the outside corner that faces the depot,
                // giving carriers a dedicated apron while builders can circulate
                // around every wall without occupying the unload throat.
                const float hx=m.footprint_x*0.5f+1.95f;
                const float hz=m.footprint_z*0.5f+1.95f;
                Vec3 best{}; float best_d=std::numeric_limits<float>::max();
                for(float sx: {-1.0f,1.0f}) for(float sz: {-1.0f,1.0f}){
                    Vec3 candidate=h.position+world_math::rotate_y({sx*hx,0.0f,sz*hz},h.rotation_degrees);
                    const float d=dist_xz(candidate,starter_stash_center_);
                    if(d<best_d){ best_d=d; best=candidate; }
                }
                best.y=first_person_floor_height(best); return best;
            }
            break;
        case ConstructionTargetKind::Resource:
            if(target.index<placed_resources_.size()){
                const auto& item=placed_resources_[target.index];
                const auto& m=resource_models_.mesh(item);
                const float hx=m.footprint_x*0.5f+1.95f;
                const float hz=m.footprint_z*0.5f+1.95f;
                Vec3 best{}; float best_d=std::numeric_limits<float>::max();
                for(float sx: {-1.0f,1.0f}) for(float sz: {-1.0f,1.0f}){
                    Vec3 candidate=item.position+world_math::rotate_y({sx*hx,0.0f,sz*hz},item.rotation_degrees);
                    const float d=dist_xz(candidate,starter_stash_center_);
                    if(d<best_d){ best_d=d; best=candidate; }
                }
                best.y=first_person_floor_height(best); return best;
            }
            break;
        case ConstructionTargetKind::Corridor:
        case ConstructionTargetKind::Utility:
        case ConstructionTargetKind::Walkway: {
            Vec3 p=construction_target_position(target)+Vec3{1.65f,0.0f,1.65f};
            p.y=first_person_floor_height(p); return p;
        }
        case ConstructionTargetKind::None: break;
    }
    return construction_target_position(target);
}

Vec3 GameApp::construction_drop_package_position(const ConstructionTargetRef& target,int visual_slot) const {
    // The rover handoff remains outside the future wall envelope, but delivered
    // material is stored INSIDE the blueprint. This keeps the visible site tidy
    // without asking a carrier to drive into geometry that may become solid on
    // the next construction step.
    const int slot=std::max(0,visual_slot)%18;
    const int column=slot%6;
    const int row=(slot/6)%3;

    auto structure_grid=[&](Vec3 center,float rotation,float footprint_x,float footprint_z){
        // Six compact columns by three rows. Spacing is clamped for unusually
        // small footprints and leaves a generous wall inset on every structure.
        const float sx=std::min(0.54f,std::max(0.30f,(footprint_x-1.40f)/5.0f));
        const float sz=std::min(0.58f,std::max(0.30f,(footprint_z-1.40f)/2.0f));
        const Vec3 local{((float)column-2.5f)*sx,0.0f,((float)row-1.0f)*sz};
        Vec3 p=center+world_math::rotate_y(local,rotation);
        p.y=first_person_floor_height(p);
        return p;
    };

    if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
        const auto& h=placed_habitats_[target.index];
        const auto& mesh=habitat_models_.mesh(h);
        return structure_grid(h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
    }
    if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
        const auto& item=placed_resources_[target.index];
        const auto& mesh=resource_models_.mesh(item);
        return structure_grid(item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
    }

    // Linear projects have no enclosed footprint, so stage their boxes directly
    // on the active blueprint centreline instead of beside it. Keep the rows
    // narrow enough to remain within the corridor/walkway/channel build strip.
    const Vec3 center=construction_target_position(target);
    Vec3 tangent{1.0f,0.0f,0.0f};
    auto tangent_from_path=[&](const std::vector<Vec3>& path){
        if(path.size()<2) return;
        const std::uint8_t step=construction_target_step(target);
        const std::uint8_t steps=std::max<std::uint8_t>(1,construction_target_total_steps(target));
        const float f=((float)step+0.5f)/(float)steps;
        const Vec3 a=point_along_path(path,std::max(0.0f,f-0.03f));
        const Vec3 b=point_along_path(path,std::min(1.0f,f+0.03f));
        Vec3 d=b-a; d.y=0.0f;
        const float len=std::sqrt(d.x*d.x+d.z*d.z);
        if(len>0.05f) tangent=d*(1.0f/len);
    };
    switch(target.kind){
        case ConstructionTargetKind::Corridor:
            if(target.index<corridor_network_.connections().size()) tangent_from_path(corridor_network_.connections()[target.index].path);
            break;
        case ConstructionTargetKind::Utility:
            if(target.index<utility_network_.connections().size()) tangent_from_path(utility_network_.connections()[target.index].path);
            break;
        case ConstructionTargetKind::Walkway:
            if(target.index<walkway_network_.connections().size()) tangent_from_path(walkway_network_.connections()[target.index].path);
            break;
        default: break;
    }
    const Vec3 lateral{-tangent.z,0.0f,tangent.x};
    Vec3 p=center+tangent*(((float)column-2.5f)*0.34f)+lateral*(((float)row-1.0f)*0.28f);
    p.y=first_person_floor_height(p);
    return p;
}

bool GameApp::advance_construction_target(const ConstructionTargetRef& target,float dt){
    auto advance=[&](auto& state){
        if(state.complete()) return false;
        state.step_elapsed+=std::max(0.0f,dt);
        if(state.step_elapsed<kConstructionStepSeconds) return false;
        state.step_elapsed=0.0f;
        ++state.step;
        return true;
    };
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            if(target.index<placed_habitats_.size()) return advance(placed_habitats_[target.index].construction);
            break;
        case ConstructionTargetKind::Resource:
            if(target.index<placed_resources_.size()) return advance(placed_resources_[target.index].construction);
            break;
        case ConstructionTargetKind::Corridor:
            if(target.index<corridor_network_.mutable_connections().size()) return advance(corridor_network_.mutable_connections()[target.index].construction);
            break;
        case ConstructionTargetKind::Utility:
            if(target.index<utility_network_.mutable_connections().size()) return advance(utility_network_.mutable_connections()[target.index].construction);
            break;
        case ConstructionTargetKind::Walkway:
            if(target.index<walkway_network_.mutable_connections().size()) return advance(walkway_network_.mutable_connections()[target.index].construction);
            break;
        case ConstructionTargetKind::None: break;
    }
    return false;
}

bool GameApp::robot_load_needed_resources(BuilderRobotState& robot,const ResourceBundle& need){
    const std::int64_t capacity=free_capacity(robot);
    if(capacity<=0) return false;

    SourceChoice best_storage{};
    SourceChoice best_ground{};
    // Once a Storage Dome exists it is the preferred depot. Ground piles remain
    // a fallback so construction never deadlocks while CR-01 is still sweeping
    // newly produced or starter resources into storage.
    for(std::size_t ri=0;ri<placed_resources_.size();++ri){
        const auto& structure=placed_resources_[ri];
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        for(std::size_t bi=0;bi<structure.stored_boxes.size();++bi){
            const auto& box=structure.stored_boxes[bi];
            if(box.amount<=0 || need.amount(box.resource)<=0) continue;
            const auto& mesh=resource_models_.mesh(structure);
            const Vec3 service=structure.position+world_math::rotate_y({mesh.footprint_x*0.5f+0.75f,0.0f,0.0f},structure.rotation_degrees);
            const float d=dist_xz(robot.position,service);
            // Storage boxes are accessed through the dome service face. Multiple
            // robots may occupy neighboring service lanes, so use a tolerant
            // manipulator reach instead of requiring one exact center point.
            if(d<=1.45f && d<best_storage.distance) best_storage={true,false,ri,bi,box.resource,service,d};
        }
    }
    for(std::size_t i=0;i<ground_resource_piles_.size();++i){
        const auto& pile=ground_resource_piles_[i];
        if(pile.amount<=0 || need.amount(pile.resource)<=0) continue;
        const float d=dist_xz(robot.position,pile.position);
        const float package_radius=std::max(pile.radius,resource_package_half_width(pile.resource,pile.amount));
        const float reach=robot_resource_clearance_radius(robot.role)+package_radius+kRobotObstacleMargin+0.42f;
        if(d<=reach && d<best_ground.distance) best_ground={true,true,i,0,pile.resource,pile.position,d};
    }
    SourceChoice best=best_storage.valid?best_storage:best_ground;
    if(!best.valid) return false;

    const std::int64_t wanted=std::min<std::int64_t>(capacity,need.amount(best.material));
    if(wanted<=0) return false;
    std::int64_t taken=0;
    if(best.ground){
        if(best.index>=ground_resource_piles_.size()) return false;
        auto& pile=ground_resource_piles_[best.index];
        taken=std::min<std::int64_t>(wanted,pile.amount);
        pile.amount-=taken;
        if(pile.amount<=0) ground_resource_piles_.erase(ground_resource_piles_.begin()+(std::ptrdiff_t)best.index);
        else{
            const bool starter_pallet=std::abs(pile.position.x-(starter_stash_center_.x-2.15f))<0.35f &&
                                      std::abs(pile.position.z-starter_stash_center_.z)<6.8f;
            // Starter resources are aggregate pallet stacks with fixed service
            // geometry. Shrinking the radius after the first withdrawal makes
            // already-planned stable pickup stations fall outside the later
            // rover's reach even though the pallet itself never moved.
            pile.radius=starter_pallet?0.36f:resource_package_half_width(pile.resource,pile.amount);
        }
    }else{
        if(best.index>=placed_resources_.size()) return false;
        auto& structure=placed_resources_[best.index];
        if(best.box_index>=structure.stored_boxes.size()) return false;
        auto& box=structure.stored_boxes[best.box_index];
        taken=std::min<std::int64_t>(wanted,box.amount);
        box.amount-=taken;
        if(box.amount<=0) structure.stored_boxes.erase(structure.stored_boxes.begin()+(std::ptrdiff_t)best.box_index);
        sync_stored_resource_totals(structure);
    }
    if(taken<=0) return false;
    robot.cargo.amount(best.material)+=taken;
    robot.cargo_committed_to_build=true;
    ResourceBundle ledger{}; ledger.amount(best.material)=taken;
    resources_.consume(ledger,false);
    return true;
}


std::size_t GameApp::construction_site_owner_index(const ConstructionTargetRef& target) const {
    if(!target.valid()) return kNoRobotIndex;
    std::size_t fallback=kNoRobotIndex;
    for(std::size_t i=0;i<builder_robots_.size();++i){
        const auto& robot=builder_robots_[i];
        if(robot.role!=RobotRole::Constructor || !robot.target.valid() || !robot.target.same_as(target) ||
           construction_target_complete(target)) continue;
        if(fallback==kNoRobotIndex) fallback=i;
        if(robot.support_robot_index==i) return i;
        if(robot.support_robot_index<builder_robots_.size()){
            const auto& owner=builder_robots_[robot.support_robot_index];
            if(owner.role==RobotRole::Constructor && owner.target.valid() && owner.target.same_as(target))
                return robot.support_robot_index;
        }
    }
    return fallback;
}

ResourceBundle* GameApp::construction_site_buffer(const ConstructionTargetRef& target) {
    if(!target.valid()) return nullptr;
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            return target.index<placed_habitats_.size()?&placed_habitats_[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Resource:
            return target.index<placed_resources_.size()?&placed_resources_[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Corridor:
            return target.index<corridor_network_.mutable_connections().size()?&corridor_network_.mutable_connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Utility:
            return target.index<utility_network_.mutable_connections().size()?&utility_network_.mutable_connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Walkway:
            return target.index<walkway_network_.mutable_connections().size()?&walkway_network_.mutable_connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::None: break;
    }
    return nullptr;
}

const ResourceBundle* GameApp::construction_site_buffer(const ConstructionTargetRef& target) const {
    if(!target.valid()) return nullptr;
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            return target.index<placed_habitats_.size()?&placed_habitats_[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Resource:
            return target.index<placed_resources_.size()?&placed_resources_[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Corridor:
            return target.index<corridor_network_.connections().size()?&corridor_network_.connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Utility:
            return target.index<utility_network_.connections().size()?&utility_network_.connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::Walkway:
            return target.index<walkway_network_.connections().size()?&walkway_network_.connections()[target.index].site_buffer:nullptr;
        case ConstructionTargetKind::None: break;
    }
    return nullptr;
}

ResourceBundle GameApp::construction_site_reserved(const ConstructionTargetRef& target,std::size_t exclude_robot) const {
    ResourceBundle reserved{};
    if(!target.valid()) return reserved;
    for(std::size_t i=0;i<builder_robots_.size();++i){
        if(i==exclude_robot) continue;
        const auto& robot=builder_robots_[i];
        if(!robot.cargo_committed_to_build || !robot.target.valid() || !robot.target.same_as(target)) continue;
        // Only active transport work owns a reservation. Stale/idle assignments
        // must never block the shared demand ledger forever; after a sustained
        // route blockage an empty robot's reservation is treated as released so
        // another CR-01 or the BR-02 fallback hauler can claim the shortage.
        if(robot.mode!=BuilderRobotMode::Fetching && robot.mode!=BuilderRobotMode::Delivering) continue;
        // A reservation is only useful while its rover can actually progress.
        // After the same 2 s route-resolution window used by navigation, a
        // carrier with no executable route no longer suppresses BR-02 fallback
        // hauling. deliver_reserved_cargo() still clamps any later arrival to
        // the site's real outstanding need, so this cannot overfund a build.
        const bool unresolved_carrier=robot.role==RobotRole::Carrier &&
            !robot.route_active && robot.action==BuilderRobotAction::None &&
            robot.current_speed_mps<0.05f && robot.route_resolution_elapsed>=2.0f;
        if(unresolved_carrier) continue;
        if(robot.role==RobotRole::Carrier && robot.cargo.empty() && robot.blocked_elapsed>11.5f &&
           !robot.route_active && robot.action==BuilderRobotAction::None) continue;
        add_bundle(reserved,robot.logistics_reservation);
    }
    return reserved;
}

ResourceBundle GameApp::construction_site_remaining(const ConstructionTargetRef& target,bool include_reservations,std::size_t exclude_robot) const {
    ResourceBundle none{};
    if(!target.valid()) return none;
    const auto* plan=construction_target_plan(target);
    if(!plan) return none;
    ResourceBundle accounted{};
    if(const auto* staged=construction_site_buffer(target)) accounted=*staged;
    if(include_reservations) add_bundle(accounted,construction_site_reserved(target,exclude_robot));
    return positive_difference(plan->next_step_cost(construction_target_step(target)),accounted);
}

void GameApp::update_builder_robot(float dt){
    if(dt<=0.0f || builder_robots_.empty()) return;

    // A completed depot-exit latch belongs to one physical load only. Once the
    // cargo is gone, the next pickup starts a fresh outbound transaction.
    for(auto& robot:builder_robots_){
        if(robot.cargo.empty()){
            robot.starter_exit_pending=false;
            robot.starter_exit_completed=false;
        }
    }

    // Corridors are dependent construction: a docking sleeve may only progress
    // after the structure/corridor it attaches to is physically complete. Old
    // saves can contain a corridor that was already assigned before this rule
    // existed, so suspend those assignments here and put any staged material
    // back onto the ground instead of trapping the only constructor forever.
    auto spill_bundle_to_ground=[&](const ResourceBundle& bundle,Vec3 base,const ConstructionTargetRef* target){
        ResourceBundle returned{};
        int visual_slot=0;
        for(std::size_t mi=0;mi<kMaterialResourceCount;++mi){
            const auto material=(MaterialResource)mi;
            std::int64_t remaining=std::max<std::int64_t>(0,bundle.amount(material));
            const std::int64_t capacity=std::max<std::int64_t>(1,resource_package_capacity_kg(material));
            while(remaining>0){
                const std::int64_t amount=std::min(remaining,capacity);
                GroundResourcePile pile{};
                pile.resource=material;
                pile.amount=amount;
                if(target && target->valid()) pile.position=construction_drop_package_position(*target,visual_slot);
                else{
                    const float a=(float)(visual_slot%8)*(2.0f*PI/8.0f);
                    const float ring=0.65f+0.42f*(float)(visual_slot/8);
                    pile.position=base+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
                    pile.position.y=first_person_floor_height(pile.position);
                }
                pile.radius=resource_package_half_width(material,amount);
                pile.condition=1.0f;
                ground_resource_piles_.push_back(pile);
                returned.amount(material)+=amount;
                remaining-=amount;
                ++visual_slot;
            }
        }
        if(!returned.empty()) resources_.add(returned);
    };

    std::vector<ConstructionTargetRef> blocked_targets;
    for(const auto& robot:builder_robots_){
        if(!robot.target.valid() || construction_target_dependencies_ready(robot.target)) continue;
        bool duplicate=false;
        for(const auto& target:blocked_targets) if(target.same_as(robot.target)){ duplicate=true; break; }
        if(!duplicate) blocked_targets.push_back(robot.target);
    }
    for(const auto& blocked:blocked_targets){
        if(auto* staged_buffer=construction_site_buffer(blocked); staged_buffer && !staged_buffer->empty()){
            const ResourceBundle staged=*staged_buffer;
            spill_bundle_to_ground(staged,construction_drop_position(blocked),&blocked);
            *staged_buffer={};
        }
        for(auto& robot:builder_robots_){
            if(!robot.target.valid() || !robot.target.same_as(blocked)) continue;
            if(robot.role==RobotRole::Constructor && !robot.cargo.empty()){
                spill_bundle_to_ground(robot.cargo,robot.position,nullptr);
                robot.cargo={};
            }
            robot.target={};
            robot.support_robot_index=kNoRobotIndex;
            robot.logistics_reservation={};
            // Loaded CR-01 cargo is still construction stock even though its
            // blocked target is released. Keep that accounting bit set so a
            // later depot return restores the global physical-stock ledger.
            robot.cargo_committed_to_build=robot.role==RobotRole::Carrier && !robot.cargo.empty();
            robot.site_buffer={};
            robot.work_elapsed=0.0f;
            robot.idle_elapsed=0.0f;
            robot.stage_commit_pending=false;
            robot.stage_commit_safe_position={};
            robot.stage_materials_picked=false;
            robot.build_orbit_phase=0.0f;
            robot.build_side_corner_reached=false;
            robot.interaction_position_valid=false;
            robot.pickup_lane_claimed=false;
            robot.drop_exit_pending=false;
            robot.post_delivery_hold=false;
            robot.planned_route.clear();
            robot.planned_route_distances.clear();
            robot.route_cursor=0;
            robot.route_active=false;
            robot.route_waiting_for_clearance=false;
            robot.route_wait_elapsed=0.0f;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            robot.traffic_escape_active=false;
            robot.current_speed_mps=0.0f;
            robot.blocked_elapsed=0.0f;
            clear_action(robot);
            robot.mode=(robot.role==RobotRole::Carrier && !robot.cargo.empty())?
                BuilderRobotMode::DepotTransfer:BuilderRobotMode::Idle;
        }
    }

    // Several BR-02s may cooperate on the same target.  The lowest-index
    // assigned constructor is the site coordinator and owns the authoritative
    // material buffer / stage commit.  Other constructors share that buffer,
    // haul into it when necessary and perform visible parallel build work.
    auto constructor_team_count=[&](const ConstructionTargetRef& target){
        std::size_t count=0;
        if(!target.valid()) return count;
        for(const auto& robot:builder_robots_){
            if(robot.role==RobotRole::Constructor && robot.target.valid() && robot.target.same_as(target) &&
               !construction_target_complete(target)) ++count;
        }
        return count;
    };
    auto constructor_team_capacity=[&](const ConstructionTargetRef& target){
        // Multiple constructors are useful, but physically small structures
        // must not attract the entire fleet. Size-aware crew limits leave
        // enough perimeter/service positions for carriers to approach as well.
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& m=habitat_models_.mesh(placed_habitats_[target.index]);
            const float largest=std::max(m.footprint_x,m.footprint_z);
            return largest<7.0f?(std::size_t)2:(largest<13.0f?(std::size_t)4:(std::size_t)6);
        }
        if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& m=resource_models_.mesh(placed_resources_[target.index]);
            const float largest=std::max(m.footprint_x,m.footprint_z);
            return largest<7.0f?(std::size_t)2:(largest<13.0f?(std::size_t)4:(std::size_t)6);
        }
        switch(target.kind){
            case ConstructionTargetKind::Corridor:
            case ConstructionTargetKind::Utility:
            case ConstructionTargetKind::Walkway: return (std::size_t)2;
            case ConstructionTargetKind::Habitat:
            case ConstructionTargetKind::Resource:
            case ConstructionTargetKind::None: break;
        }
        return (std::size_t)1;
    };
    auto site_owner_index=[&](const ConstructionTargetRef& target){
        return construction_site_owner_index(target);
    };
    auto site_buffer_for=[&](const ConstructionTargetRef& target)->ResourceBundle*{
        return construction_site_buffer(target);
    };


    auto partial_construction_blocks=[&](Vec3 candidate,float radius){
        for(const auto& h:placed_habitats_){
            if(h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            const auto v=construction_sequence::habitat_visibility(h.construction.progress()*(float)kConstructionSteps);
            const bool visible=v.foundation>0.0f || v.shell>0.0f || v.interior>0.0f ||
                               v.details>0.0f || v.glazing>0.0f || v.access>0.0f;
            if(!visible) continue;
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+radius+0.8f || std::abs(local.z)>mesh.footprint_z*0.5f+radius+0.8f) continue;
            // Once any authored structure mesh has appeared, its footprint is
            // no longer a shortcut for rover traffic. This deliberately blocks
            // the whole installed footprint rather than relying on thin-triangle
            // hits that can leave diagonal gaps between revealed mesh pieces.
            if(std::abs(local.x)<mesh.footprint_x*0.5f+radius+0.10f &&
               std::abs(local.z)<mesh.footprint_z*0.5f+radius+0.10f) return true;
            if(transformed_triangles_block_circle_range(mesh.foundation,0,revealed_triangle_vertex_count(mesh.foundation,v.foundation),h.position,h.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.shell,0,revealed_triangle_vertex_count(mesh.shell,v.shell),h.position,h.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.interior,0,revealed_triangle_vertex_count(mesh.interior,v.interior),h.position,h.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.details,0,revealed_triangle_vertex_count(mesh.details,v.details),h.position,h.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.glazing,0,revealed_triangle_vertex_count(mesh.glazing,v.glazing),h.position,h.rotation_degrees,candidate,radius)) return true;
        }
        for(const auto& item:placed_resources_){
            if(item.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(item);
            const auto v=construction_sequence::resource_visibility(item.construction.progress()*(float)kConstructionSteps);
            const bool visible=v.foundation>0.0f || v.body>0.0f || v.details>0.0f;
            if(!visible) continue;
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+radius+0.8f || std::abs(local.z)>mesh.footprint_z*0.5f+radius+0.8f) continue;
            if(std::abs(local.x)<mesh.footprint_x*0.5f+radius+0.10f &&
               std::abs(local.z)<mesh.footprint_z*0.5f+radius+0.10f) return true;
            if(transformed_triangles_block_circle_range(mesh.foundation,0,revealed_triangle_vertex_count(mesh.foundation,v.foundation),item.position,item.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.body,0,revealed_triangle_vertex_count(mesh.body,v.body),item.position,item.rotation_degrees,candidate,radius) ||
               transformed_triangles_block_circle_range(mesh.details,0,revealed_triangle_vertex_count(mesh.details,v.details),item.position,item.rotation_degrees,candidate,radius)) return true;
        }
        for(const auto& c:corridor_network_.connections()){
            if(c.construction.complete() || c.construction.progress()<=0.0001f) continue;
            if(!polyline_near_xz(candidate,c.path,radius+geometry::kCorridorHalfWidth+1.0f)) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle_range(c.mesh.shell,0,revealed_triangle_vertex_count(c.mesh.shell,f),candidate,radius,[](Vec3 v){return v;}) ||
               construction_triangles_block_circle_range(c.mesh.details,0,revealed_triangle_vertex_count(c.mesh.details,f),candidate,radius,[](Vec3 v){return v;})) return true;
        }
        for(const auto& c:utility_network_.connections()){
            if(c.construction.complete() || c.construction.progress()<=0.0001f) continue;
            if(!polyline_near_xz(candidate,c.path,radius+1.75f)) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle_range(c.mesh.shell,0,revealed_triangle_vertex_count(c.mesh.shell,f),candidate,radius,[](Vec3 v){return v;}) ||
               construction_triangles_block_circle_range(c.mesh.details,0,revealed_triangle_vertex_count(c.mesh.details,f),candidate,radius,[](Vec3 v){return v;})) return true;
        }
        for(const auto& c:walkway_network_.connections()){
            if(c.construction.complete() || c.construction.progress()<=0.0001f) continue;
            if(!polyline_near_xz(candidate,c.path,radius+geometry::kWalkwayHalfWidth+0.9f)) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle_range(c.mesh.details,0,revealed_triangle_vertex_count(c.mesh.details,f),candidate,radius,[](Vec3 v){return v;})) return true;
        }
        return false;
    };

    auto target_blocks_at_step=[&](const ConstructionTargetRef& target,std::uint8_t step,Vec3 candidate,float radius){
        // Safety must only consider geometry that will APPEAR in the pending
        // step. Testing the complete next-step mesh also includes floors,
        // stairs and hull pieces that are already present; a robot legitimately
        // standing beside those parts would then be classified as unsafe
        // forever and the construction coordinator could never commit.
        const std::uint8_t current=construction_target_step(target);
        auto new_local_blocks=[&](const std::vector<Vec3>& src,float from_fraction,float to_fraction,Vec3 position,float rotation){
            const std::size_t first=revealed_triangle_vertex_count(src,from_fraction);
            const std::size_t last=revealed_triangle_vertex_count(src,to_fraction);
            return last>first && transformed_triangles_block_circle_range(src,first,last,position,rotation,candidate,radius);
        };
        auto new_world_blocks=[&](const std::vector<Vec3>& src,float from_fraction,float to_fraction){
            const std::size_t first=revealed_triangle_vertex_count(src,from_fraction);
            const std::size_t last=revealed_triangle_vertex_count(src,to_fraction);
            return last>first && construction_triangles_block_circle_range(src,first,last,candidate,radius,[](Vec3 v){ return v; });
        };
        switch(target.kind){
            case ConstructionTargetKind::Habitat:
                if(target.index<placed_habitats_.size()){
                    const auto& h=placed_habitats_[target.index];
                    const auto& mesh=habitat_models_.mesh(h);
                    const auto a=construction_sequence::habitat_visibility(current);
                    const auto b=construction_sequence::habitat_visibility(step);
                    return new_local_blocks(mesh.foundation,a.foundation,b.foundation,h.position,h.rotation_degrees) ||
                           new_local_blocks(mesh.shell,a.shell,b.shell,h.position,h.rotation_degrees) ||
                           new_local_blocks(mesh.interior,a.interior,b.interior,h.position,h.rotation_degrees) ||
                           new_local_blocks(mesh.details,a.details,b.details,h.position,h.rotation_degrees) ||
                           new_local_blocks(mesh.glazing,a.glazing,b.glazing,h.position,h.rotation_degrees);
                }
                break;
            case ConstructionTargetKind::Resource:
                if(target.index<placed_resources_.size()){
                    const auto& item=placed_resources_[target.index];
                    const auto& mesh=resource_models_.mesh(item);
                    const auto a=construction_sequence::resource_visibility(current);
                    const auto b=construction_sequence::resource_visibility(step);
                    return new_local_blocks(mesh.foundation,a.foundation,b.foundation,item.position,item.rotation_degrees) ||
                           new_local_blocks(mesh.body,a.body,b.body,item.position,item.rotation_degrees) ||
                           new_local_blocks(mesh.details,a.details,b.details,item.position,item.rotation_degrees);
                }
                break;
            case ConstructionTargetKind::Corridor:
                if(target.index<corridor_network_.connections().size()){
                    const auto& c=corridor_network_.connections()[target.index];
                    if(!polyline_near_xz(candidate,c.path,radius+geometry::kCorridorHalfWidth+1.0f)) return false;
                    const float a=(float)current/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    const float b=(float)step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    return new_world_blocks(c.mesh.shell,a,b) || new_world_blocks(c.mesh.details,a,b);
                }
                break;
            case ConstructionTargetKind::Utility:
                if(target.index<utility_network_.connections().size()){
                    const auto& c=utility_network_.connections()[target.index];
                    if(!polyline_near_xz(candidate,c.path,radius+1.75f)) return false;
                    const float a=(float)current/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    const float b=(float)step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    return new_world_blocks(c.mesh.shell,a,b) || new_world_blocks(c.mesh.details,a,b);
                }
                break;
            case ConstructionTargetKind::Walkway:
                if(target.index<walkway_network_.connections().size()){
                    const auto& c=walkway_network_.connections()[target.index];
                    if(!polyline_near_xz(candidate,c.path,radius+geometry::kWalkwayHalfWidth+0.9f)) return false;
                    const float a=(float)current/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    const float b=(float)step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    return new_world_blocks(c.mesh.details,a,b);
                }
                break;
            case ConstructionTargetKind::None: break;
        }
        return false;
    };

    auto target_future_envelope_contains=[&](const ConstructionTargetRef& target,std::uint8_t next_step,Vec3 candidate,float radius){
        // A wall-producing habitat/resource step may not be committed while a
        // robot remains inside the final footprint. The footprint is NOT a
        // movement obstacle during ordinary construction; it is only a short
        // pre-commit evacuation envelope that guarantees the new mesh can
        // never close around a robot.
        if(next_step<2) return false;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& mesh=habitat_models_.mesh(h);
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            const float margin=radius+0.30f;
            return std::abs(local.x)<mesh.footprint_x*0.5f+margin && std::abs(local.z)<mesh.footprint_z*0.5f+margin;
        }
        if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& mesh=resource_models_.mesh(item);
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            const float margin=radius+0.30f;
            return std::abs(local.x)<mesh.footprint_x*0.5f+margin && std::abs(local.z)<mesh.footprint_z*0.5f+margin;
        }
        return false;
    };

    struct CachedResourceObstacle { Vec3 position{}; float radius{0.0f}; };
    std::vector<CachedResourceObstacle> cached_resource_obstacles;
    cached_resource_obstacles.reserve(ground_resource_piles_.size()+64);
    for(const auto& pile:ground_resource_piles_){
        const float package_radius=std::max(pile.radius,resource_package_half_width(pile.resource,pile.amount));
        cached_resource_obstacles.push_back({pile.position,package_radius});
    }
    // Material package poses are deterministic for the duration of one fleet
    // simulation slice. Build them once here instead of reconstructing up to 36
    // boxes inside every A*/segment-clearance callback. That old O(nodes*boxes)
    // path was the dominant CPU cost in slowdown.gmsave once the site buffer
    // filled with several tonnes of delivered material.
    auto cache_site_buffer_obstacles=[&](const ConstructionTargetRef& target,const ResourceBundle& buffer){
        if(buffer.empty()) return;
        ResourceBundle visible_buffer=buffer;
        const std::size_t owner_index=construction_site_owner_index(target);
        if(owner_index<builder_robots_.size() && builder_robots_[owner_index].stage_materials_picked){
            if(const auto* plan=construction_target_plan(target)){
                const ResourceBundle picked=plan->next_step_cost(construction_target_step(target));
                for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                    const auto material=(MaterialResource)ri;
                    visible_buffer.amount(material)=std::max<std::int64_t>(0,visible_buffer.amount(material)-picked.amount(material));
                }
            }
        }
        int visual_slot=0;
        constexpr int max_visual_packages=36;
        for(std::size_t mi=0;mi<kMaterialResourceCount && visual_slot<max_visual_packages;++mi){
            const auto material=(MaterialResource)mi;
            std::int64_t remaining=visible_buffer.amount(material);
            const std::int64_t capacity=std::max<std::int64_t>(1,resource_package_capacity_kg(material));
            while(remaining>0 && visual_slot<max_visual_packages){
                const std::int64_t package_amount=std::min(remaining,capacity);
                const float half=resource_package_half_width(material,package_amount);
                Vec3 package_position=construction_drop_package_position(target,visual_slot);
                package_position.y=first_person_floor_height(package_position);
                cached_resource_obstacles.push_back({package_position,half*1.41421356f});
                remaining-=package_amount;
                ++visual_slot;
            }
        }
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i)
        cache_site_buffer_obstacles({ConstructionTargetKind::Habitat,i},placed_habitats_[i].site_buffer);
    for(std::size_t i=0;i<placed_resources_.size();++i)
        cache_site_buffer_obstacles({ConstructionTargetKind::Resource,i},placed_resources_[i].site_buffer);
    for(std::size_t i=0;i<corridor_network_.connections().size();++i)
        cache_site_buffer_obstacles({ConstructionTargetKind::Corridor,i},corridor_network_.connections()[i].site_buffer);
    for(std::size_t i=0;i<utility_network_.connections().size();++i)
        cache_site_buffer_obstacles({ConstructionTargetKind::Utility,i},utility_network_.connections()[i].site_buffer);
    for(std::size_t i=0;i<walkway_network_.connections().size();++i)
        cache_site_buffer_obstacles({ConstructionTargetKind::Walkway,i},walkway_network_.connections()[i].site_buffer);
    for(std::size_t structure_index=0;structure_index<placed_resources_.size();++structure_index){
        const auto& structure=placed_resources_[structure_index];
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        const auto& mesh=resource_models_.mesh(structure);
        for(std::size_t bi=0;bi<structure.stored_boxes.size();++bi){
            Vec3 base{}; float half=0.0f,height=0.0f;
            stored_resource_box_pose(structure,mesh,bi,base,half,height);
            (void)height;
            cached_resource_obstacles.push_back({base,half*1.41421356f});
        }
    }

    auto robot_position_clear=[&](std::size_t robot_index,Vec3 candidate){
        const auto& robot=builder_robots_[robot_index];
        const float body_radius=robot_body_radius(robot.role);
        const float radius=body_radius+kRobotObstacleMargin;

        // Erected geometry is solid, and the volume that will appear in the
        // NEXT construction step is reserved before any rover starts a route.
        // This turns construction into a predictable moving obstacle instead of
        // evacuating robots only at the instant a wall/segment appears.
        if(partial_construction_blocks(candidate,radius)) return false;
        auto future_reserved=[&](const ConstructionTargetRef& target){
            if(!target.valid() || construction_target_complete(target)) return false;
            // A robot servicing this exact construction target may enter its
            // dedicated material/work apron. The next segment is not committed
            // until the fleet-safe stage gate runs, so treating that future
            // geometry as a hard pathfinding obstacle can make the legitimate
            // drop position unreachable and deadlock logistics.
            if(robot.target.valid() && robot.target.same_as(target) &&
               (robot.mode==BuilderRobotMode::Delivering ||
                robot.mode==BuilderRobotMode::Fetching ||
                robot.mode==BuilderRobotMode::AwaitingMaterials ||
                robot.mode==BuilderRobotMode::Building))
                return false;
            const auto current=construction_target_step(target);
            // A untouched blueprint is not yet an imminent geometry change. In
            // v0.40.5 every step-0 corridor/structure was treated as if its
            // first segment could appear immediately, so each pathfinding cell
            // rescanned thousands of mesh triangles for projects no robot was
            // even building. Reserve step-0 geometry only once a constructor
            // has actually claimed that target. In-progress targets remain
            // reserved exactly as before.
            if(current==0){
                bool actively_claimed=false;
                for(const auto& fleet_robot:builder_robots_){
                    if(fleet_robot.role==RobotRole::Constructor && fleet_robot.target.valid() && fleet_robot.target.same_as(target)){
                        actively_claimed=true;
                        break;
                    }
                }
                if(!actively_claimed) return false;
            }
            const auto total=construction_target_total_steps(target);
            const auto next=(std::uint8_t)std::min<int>((int)current+1,(int)total);
            if(next<=current) return false;
            // Through-traffic reserves the full robot-sized future segment. The
            // BR-02 assigned to this exact construction may approach its wall
            // face with a small centre-point clearance; the two-phase stage
            // commit still performs the strict full-hull evacuation check before
            // the geometry is revealed, so a new segment can never spawn through
            // the builder.
            if(robot.role==RobotRole::Constructor && robot.target.valid() && robot.target.same_as(target))
                return false;
            return target_blocks_at_step(target,next,candidate,radius);
        };
        for(std::size_t i=0;i<placed_habitats_.size();++i)
            if(future_reserved({ConstructionTargetKind::Habitat,i})) return false;
        for(std::size_t i=0;i<placed_resources_.size();++i)
            if(future_reserved({ConstructionTargetKind::Resource,i})) return false;
        for(std::size_t i=0;i<corridor_network_.connections().size();++i)
            if(future_reserved({ConstructionTargetKind::Corridor,i})) return false;
        for(std::size_t i=0;i<utility_network_.connections().size();++i)
            if(future_reserved({ConstructionTargetKind::Utility,i})) return false;
        for(std::size_t i=0;i<walkway_network_.connections().size();++i)
            if(future_reserved({ConstructionTargetKind::Walkway,i})) return false;

        const float resource_clearance=robot_resource_clearance_radius(robot.role);
        for(const auto& obstacle:cached_resource_obstacles)
            if(dist_xz(candidate,obstacle.position)<resource_clearance+obstacle.radius+kRobotObstacleMargin) return false;
        return true;
    };

    auto robot_structure_move_allowed=[&](const BuilderRobotState& robot,Vec3 from,Vec3 to){
        if(!first_person_move_allowed(from,to)) return false;
        const float fp_radius=first_person_.body_radius();
        const float extra=std::max(0.0f,robot_body_radius(robot.role)-fp_radius);
        if(extra<=0.01f) return true;
        constexpr std::array<Vec3,4> axes{{Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}}};
        for(const Vec3 axis:axes){
            const Vec3 off=axis*extra;
            if(!first_person_move_allowed(from+off,to+off)) return false;
        }
        return true;
    };

    auto fleet_hull_clear=[&](std::size_t robot_index,Vec3 candidate,float candidate_heading_degrees){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(robot_hulls_overlap(candidate,candidate_heading_degrees,robot.role,
                                   other.position,other.heading_degrees,other.role,kRobotHullClearanceM)) return false;
        }
        return true;
    };

    auto fleet_hull_hard_clear=[&](std::size_t robot_index,Vec3 candidate,float candidate_heading_degrees){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(robot_hulls_overlap(candidate,candidate_heading_degrees,robot.role,
                                   other.position,other.heading_degrees,other.role,0.0f)) return false;
        }
        return true;
    };

    auto fleet_position_clear=[&](std::size_t robot_index,Vec3 candidate){
        const auto& robot=builder_robots_[robot_index];
        float heading=robot.heading_degrees;
        const float dx=candidate.x-robot.position.x,dz=candidate.z-robot.position.z;
        if(dx*dx+dz*dz>0.0025f) heading=std::atan2(dx,dz)*180.0f/PI;
        return fleet_hull_clear(robot_index,candidate,heading);
    };

    // v0.31.17 continuous fleet navigator.  The previous implementation used
    // 0.48 s time slots, which made even a valid route visibly pulse at every
    // slot boundary.  Planning is now split into three layers:
    //   1) Theta*-style any-angle spatial planning around static geometry,
    //   2) full-route traffic prediction against the other robots,
    //   3) continuous arc-length following with a reciprocal speed governor.
    // Fleet conflicts therefore change the chosen lane/speed, never the steering
    // direction after departure.  This is intentionally closer to the traffic
    // schedule + continuous local avoidance used by real multi-robot systems.
    constexpr float kRouteGridCellM=0.72f;
    constexpr float kRoutePointSpacingM=0.30f;
    constexpr float kTrafficReplanYieldSeconds=1.40f;
    constexpr float kTrafficHardHorizonSeconds=0.24f;
    constexpr float kFullRouteResolutionSeconds=2.00f;

    auto heading_between=[](Vec3 from,Vec3 to,float fallback){
        const float dx=to.x-from.x,dz=to.z-from.z;
        if(dx*dx+dz*dz<1.0e-8f) return fallback;
        return std::atan2(dx,dz)*180.0f/PI;
    };

    auto turn_towards_heading=[](float current_degrees,float target_degrees,float max_delta_degrees){
        float current=current_degrees*PI/180.0f;
        const float target=target_degrees*PI/180.0f;
        const float delta=wrap_radians(target-current);
        const float max_delta=std::max(0.0f,max_delta_degrees)*PI/180.0f;
        current=wrap_radians(current+std::clamp(delta,-max_delta,max_delta));
        return current*180.0f/PI;
    };

    auto clear_route=[](BuilderRobotState& robot){
        robot.planned_route.clear();
        robot.planned_route_distances.clear();
        robot.route_cursor=0;
        robot.route_active=false;
        robot.route_waiting_for_clearance=false;
        robot.route_wait_elapsed=0.0f;
        robot.route_stop_radius=0.0f;
        robot.route_speed_scale=1.0f;
        robot.route_progress_m=0.0f;
        robot.route_total_length_m=0.0f;
        robot.route_cruise_speed_mps=0.0f;
        robot.route_segment_elapsed=0.0f;
        robot.route_segment_origin={};
        robot.route_segment_origin_valid=false;
    };

    // Static-world segment validation. Moving fleet mates are not baked into the
    // world geometry: they are handled by the traffic predictor below. Robots
    // which are actually stationary remain temporary obstacles so a fresh route
    // does not deliberately run through a parked chassis/work station.
    auto route_segment_clear=[&](std::size_t robot_index,Vec3 from,Vec3 to){
        const auto& robot=builder_robots_[robot_index];
        const float length=dist_xz(from,to);
        const int samples=std::max(1,(int)std::ceil(length/0.22f));
        Vec3 previous=from;
        for(int sample=1;sample<=samples;++sample){
            const float t=(float)sample/(float)samples;
            Vec3 p=from+(to-from)*t;
            p.y=first_person_floor_height(p);
            if(std::abs(p.y-previous.y)>0.38f) return false;
            if(!robot_position_clear(robot_index,p)) return false;
            if(!robot_structure_move_allowed(robot,previous,p)) return false;
            previous=p;
        }
        return true;
    };

    auto route_segment_plannable_clear=[&](std::size_t robot_index,Vec3 from,Vec3 to){
        if(!route_segment_clear(robot_index,from,to)) return false;
        const auto& robot=builder_robots_[robot_index];
        const float length=dist_xz(from,to);
        const int samples=std::max(1,(int)std::ceil(length/0.24f));
        const float heading=heading_between(from,to,robot.heading_degrees);
        for(int sample=1;sample<=samples;++sample){
            const float t=(float)sample/(float)samples;
            Vec3 p=from+(to-from)*t;
            p.y=first_person_floor_height(p);
            for(std::size_t i=0;i<builder_robots_.size();++i){
                if(i==robot_index) continue;
                const auto& other=builder_robots_[i];
                // A published route does not make a chassis disappear. Until a
                // rover is actually flowing along that route, later planners
                // must route around its current hull; otherwise several robots
                // can all publish paths through the same parking bay and then
                // deadlock when the hard collision guard refuses execution.
                if(other.route_active && other.current_speed_mps>0.12f && !other.route_waiting_for_clearance) continue;
                if(robot_hulls_overlap(p,heading,robot.role,other.position,other.heading_degrees,
                                       other.role,kRobotHullClearanceM+0.12f)) return false;
            }
        }
        return true;
    };

    // Spatial traffic cost used by the any-angle planner. This does not reserve
    // cells; it simply makes already used fleet lanes expensive so independently
    // assigned robots naturally choose parallel lanes rather than queueing on top
    // of each other. Exact collision exclusion is handled by the predictors and
    // hard hull guard.
    auto route_lane_penalty=[&](std::size_t robot_index,Vec3 candidate){
        const auto& robot=builder_robots_[robot_index];
        float penalty=0.0f;
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            const RobotFootprint rf=robot_footprint(robot.role);
            const RobotFootprint of=robot_footprint(other.role);
            const float desired=std::sqrt(rf.half_width*rf.half_width+rf.half_length*rf.half_length)+
                                std::sqrt(of.half_width*of.half_width+of.half_length*of.half_length)+0.42f;
            const float current=dist_xz(candidate,other.position);
            if(current<desired*1.30f) penalty+=(desired*1.30f-current)*3.1f;
            if(!other.route_active || other.route_cursor>=other.planned_route.size()) continue;
            // Planning must remain cheap enough that a new assignment never
            // freezes the visible fleet. Sample a bounded number of segments
            // from the other dense route instead of scanning every 30 cm point
            // for every A* expansion.
            const std::size_t remaining_points=other.planned_route.size()-other.route_cursor;
            const std::size_t stride=std::max<std::size_t>(1,remaining_points/12);
            Vec3 a=other.position;
            for(std::size_t k=other.route_cursor;k<other.planned_route.size();k+=stride){
                const Vec3 b=other.planned_route[k];
                const float d=segment_distance_xz(candidate,a,b);
                if(d<desired) penalty+=(desired-d)*1.15f;
                a=b;
            }
            const Vec3 last=other.planned_route.back();
            const float tail_d=segment_distance_xz(candidate,a,last);
            if(tail_d<desired) penalty+=(desired-tail_d)*1.15f;
        }
        return penalty;
    };

    auto route_position_at_distance=[&](const BuilderRobotState& robot,float distance){
        const Vec3 origin=robot.route_segment_origin_valid?robot.route_segment_origin:robot.position;
        if(robot.planned_route.empty() || robot.planned_route_distances.size()!=robot.planned_route.size()) return origin;
        distance=std::clamp(distance,0.0f,robot.route_total_length_m);
        Vec3 previous=origin;
        float previous_distance=0.0f;
        for(std::size_t i=0;i<robot.planned_route.size();++i){
            const float endpoint_distance=robot.planned_route_distances[i];
            if(distance<=endpoint_distance+0.0001f){
                const float segment=endpoint_distance-previous_distance;
                const float alpha=segment>0.0001f?std::clamp((distance-previous_distance)/segment,0.0f,1.0f):1.0f;
                Vec3 p=previous+(robot.planned_route[i]-previous)*alpha;
                p.y=first_person_floor_height(p);
                return p;
            }
            previous=robot.planned_route[i];
            previous_distance=endpoint_distance;
        }
        return robot.planned_route.back();
    };

    auto route_heading_at_distance=[&](const BuilderRobotState& robot,float distance,float fallback){
        if(robot.planned_route.empty()) return fallback;
        const Vec3 a=route_position_at_distance(robot,std::max(0.0f,distance-0.08f));
        const Vec3 b=route_position_at_distance(robot,std::min(robot.route_total_length_m,distance+0.38f));
        return heading_between(a,b,fallback);
    };

    struct PredictedRobotPose { Vec3 position{}; float heading_degrees{0.0f}; };
    auto predict_robot_pose=[&](std::size_t other_index,float seconds_from_now){
        PredictedRobotPose out{};
        if(other_index>=builder_robots_.size()) return out;
        const auto& other=builder_robots_[other_index];
        out.position=other.position;
        out.heading_degrees=other.heading_degrees;
        if(!other.route_active || other.planned_route.empty() || seconds_from_now<=0.0f) return out;

        // Predict from the rover's *actual* current speed with the same
        // acceleration bound used by execution. The old predictor jumped a
        // nearly-stationary rover directly to 72% cruise speed, so neighbouring
        // carriers repeatedly predicted collisions that could not physically
        // happen and all crawled together. A rover that is already yielding is
        // predicted to hold/continue its current speed for this short horizon.
        const float cruise=std::max(0.05f,other.route_cruise_speed_mps);
        const float start_speed=std::clamp(other.current_speed_mps,0.0f,cruise);
        float advance=0.0f;
        if(other.route_waiting_for_clearance){
            advance=start_speed*seconds_from_now;
        }else{
            const float accel=kRobotAccelerationMps2;
            const float t_to_cruise=std::max(0.0f,(cruise-start_speed)/accel);
            if(seconds_from_now<=t_to_cruise){
                advance=start_speed*seconds_from_now+0.5f*accel*seconds_from_now*seconds_from_now;
            }else{
                advance=start_speed*t_to_cruise+0.5f*accel*t_to_cruise*t_to_cruise+
                        cruise*(seconds_from_now-t_to_cruise);
            }
        }
        const float future_distance=std::min(other.route_total_length_m,other.route_progress_m+advance);
        out.position=route_position_at_distance(other,future_distance);
        out.heading_degrees=route_heading_at_distance(other,future_distance,other.heading_degrees);
        return out;
    };

    auto dynamic_route_edge_clear=[&](std::size_t robot_index,Vec3 from,Vec3 to,float t0,float t1){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        const float length=dist_xz(from,to);
        const float motion_heading=heading_between(from,to,robot.heading_degrees);
        // Sample both hull motion and predicted fleet motion through the edge.
        // The edge lengths are normally <= one search cell, so a handful of
        // samples is enough while remaining far cheaper than resolving a
        // collision after departure.
        const int samples=std::clamp((int)std::ceil(length/0.24f)+1,2,6);
        for(int q=0;q<=samples;++q){
            const float alpha=(float)q/(float)samples;
            Vec3 p=from+(to-from)*alpha;
            p.y=first_person_floor_height(p);
            const float when=t0+(t1-t0)*alpha;
            for(std::size_t i=0;i<builder_robots_.size();++i){
                if(i==robot_index) continue;
                const auto predicted=predict_robot_pose(i,when);
                if(robot_hulls_overlap(p,motion_heading,robot.role,
                                       predicted.position,predicted.heading_degrees,builder_robots_[i].role,
                                       kRobotHullClearanceM+0.10f)) return false;
            }
        }
        return true;
    };

    auto make_continuous_path=[&](std::size_t robot_index,const std::vector<Vec3>& geometric){
        std::vector<Vec3> nodes;
        nodes.reserve(geometric.size()+1);
        nodes.push_back(builder_robots_[robot_index].position);
        for(Vec3 p:geometric){ p.y=first_person_floor_height(p); if(dist_xz(nodes.back(),p)>0.025f) nodes.push_back(p); }
        if(nodes.size()<2) return std::vector<Vec3>{};

        std::vector<Vec3> rounded;
        rounded.reserve(nodes.size()*4);
        rounded.push_back(nodes.front());
        for(std::size_t i=1;i+1<nodes.size();++i){
            const Vec3 a=nodes[i-1], b=nodes[i], c=nodes[i+1];
            const float ab=dist_xz(a,b), bc=dist_xz(b,c);
            if(ab<0.24f || bc<0.24f){ rounded.push_back(b); continue; }
            const float cut=std::min(0.55f,std::min(ab,bc)*0.28f);
            const Vec3 entry=b+(a-b)*(cut/ab);
            const Vec3 exit=b+(c-b)*(cut/bc);
            std::array<Vec3,5> curve{};
            curve[0]=entry;
            for(int q=1;q<=4;++q){
                const float t=(float)q/4.0f;
                const float u=1.0f-t;
                Vec3 p=entry*(u*u)+b*(2.0f*u*t)+exit*(t*t);
                p.y=first_person_floor_height(p);
                curve[(std::size_t)q]=p;
            }
            bool legal=route_segment_plannable_clear(robot_index,rounded.back(),curve[0]);
            for(std::size_t q=1;q<curve.size() && legal;++q)
                legal=route_segment_plannable_clear(robot_index,curve[q-1],curve[q]);
            if(legal){
                if(dist_xz(rounded.back(),curve[0])>0.02f) rounded.push_back(curve[0]);
                for(std::size_t q=1;q<curve.size();++q)
                    if(dist_xz(rounded.back(),curve[q])>0.02f) rounded.push_back(curve[q]);
            }else rounded.push_back(b);
        }
        rounded.push_back(nodes.back());

        std::vector<Vec3> dense;
        Vec3 from=rounded.front();
        for(std::size_t i=1;i<rounded.size();++i){
            const Vec3 endpoint=rounded[i];
            const float d=dist_xz(from,endpoint);
            const int pieces=std::max(1,(int)std::ceil(d/kRoutePointSpacingM));
            for(int piece=1;piece<=pieces;++piece){
                Vec3 p=from+(endpoint-from)*((float)piece/(float)pieces);
                p.y=first_person_floor_height(p);
                if(dense.empty() || dist_xz(dense.back(),p)>0.01f) dense.push_back(p);
            }
            from=endpoint;
        }
        return dense;
    };

    auto route_distances=[&](Vec3 origin,const std::vector<Vec3>& points){
        std::vector<float> distances;
        distances.reserve(points.size());
        float total=0.0f;
        Vec3 previous=origin;
        for(const Vec3 p:points){ total+=dist_xz(previous,p); distances.push_back(total); previous=p; }
        return distances;
    };

    auto commit_route=[&](std::size_t robot_index,const std::vector<Vec3>& points,float speed_scale){
        auto& robot=builder_robots_[robot_index];
        if(points.empty()) return false;
        const auto distances=route_distances(robot.position,points);
        if(distances.empty() || distances.back()<0.025f) return false;
        robot.planned_route=points;
        robot.planned_route_distances=distances;
        robot.route_cursor=0;
        robot.route_progress_m=0.0f;
        robot.route_total_length_m=distances.back();
        robot.route_speed_scale=std::clamp(speed_scale,0.10f,1.0f);
        robot.route_cruise_speed_mps=std::max(0.10f,robot.move_speed_mps*robot.route_speed_scale);
        robot.route_segment_origin=robot.position;
        robot.route_segment_origin_valid=true;
        robot.route_segment_elapsed=0.0f;
        robot.route_active=true;
        robot.route_waiting_for_clearance=false;
        robot.route_wait_elapsed=0.0f;
        return true;
    };

    // Path searches are bounded per role rather than by one global token. The
    // single-token implementation could be consumed forever by an early-index
    // constructor, starving every CR-01 later in the array even though the CPU
    // budget was otherwise idle. Separate quotas keep construction and
    // logistics progressing in parallel while still bounding placement cost.
    // Two construction crews and three logistics rovers may publish/replan in
    // the same 90 Hz slice. The searches are corridor-bounded, so this remains
    // a small deterministic CPU budget while preventing low-index robots on one
    // site from starving a second simultaneously active construction.
    int& constructor_route_plans_remaining=robot_constructor_route_plans_budget_;
    int& carrier_route_plans_remaining=robot_carrier_route_plans_budget_;

    // Spatial path search lives in the robot-pathfinding implementation target.
    // This integration layer supplies game-specific occupancy, structure and
    // fleet-traffic callbacks without exposing construction/logistics state to
    // the generic A*/Theta*-style planner.
    auto build_spatial_route=[&](std::size_t robot_index,Vec3 direct_goal,float stop_radius,float corridor_half_width,
                                 float traffic_weight,int max_expansions,float grid_cell,float nominal_speed_mps,
                                 bool include_stationary_fleet,std::vector<Vec3>& geometric){
        auto& robot=builder_robots_[robot_index];
        const float half=(float)playfield_.size()*0.5f-0.52f;
        const float nominal_heading=heading_between(robot.position,direct_goal,robot.heading_degrees);
        robots::PathPlannerCallbacks callbacks;
        callbacks.floor_height=[&](Vec3 p){ return first_person_floor_height(p); };
        callbacks.point_clear=[&](Vec3 p){
            if(std::abs(p.x)>half || std::abs(p.z)>half) return false;
            if(!robot_position_clear(robot_index,p) || !robot_structure_move_allowed(robot,p,p)) return false;
            // Stationary/working robots are temporary hard obstacles for route
            // publication. Moving robots stay in the traffic-cost/prediction
            // layer so the search does not generate map-wide detours around a
            // rover which will have moved by the time this robot arrives.
            if(include_stationary_fleet){
                for(std::size_t i=0;i<builder_robots_.size();++i){
                    if(i==robot_index) continue;
                    const auto& other=builder_robots_[i];
                    if(other.route_active && other.current_speed_mps>0.12f && !other.route_waiting_for_clearance) continue;
                    if(robot_hulls_overlap(p,nominal_heading,robot.role,
                                           other.position,other.heading_degrees,other.role,
                                           kRobotHullClearanceM+0.12f)) return false;
                }
            }
            return true;
        };
        callbacks.segment_clear=[&](Vec3 a,Vec3 b){
            return include_stationary_fleet?route_segment_plannable_clear(robot_index,a,b)
                                          :route_segment_clear(robot_index,a,b);
        };
        callbacks.neighbor_clear=[&](Vec3 a,Vec3 b){
            // A search-grid edge is shorter than one metre and all navigation
            // obstacles are already inflated by the rover hull. Checking its
            // midpoint avoids repeating the strict segment sampler 3-5 times for
            // every A* neighbour. Final path simplification still uses the strict
            // segment callback, and execution revalidates every actual movement.
            Vec3 mid=(a+b)*0.5f;
            mid.y=first_person_floor_height(mid);
            if(std::abs(mid.y-a.y)>0.38f || std::abs(b.y-mid.y)>0.38f) return false;
            if(!robot_position_clear(robot_index,mid) ||
               !robot_structure_move_allowed(robot,a,mid) ||
               !robot_structure_move_allowed(robot,mid,b)) return false;
            if(include_stationary_fleet){
                const float heading=heading_between(a,b,robot.heading_degrees);
                for(std::size_t i=0;i<builder_robots_.size();++i){
                    if(i==robot_index) continue;
                    const auto& other=builder_robots_[i];
                    if(other.route_active && other.current_speed_mps>0.12f && !other.route_waiting_for_clearance) continue;
                    if(robot_hulls_overlap(mid,heading,robot.role,other.position,other.heading_degrees,other.role,
                                           kRobotHullClearanceM+0.12f)) return false;
                }
            }
            return true;
        };
        callbacks.traffic_cost=[&](Vec3 p){ return route_lane_penalty(robot_index,p); };
        callbacks.dynamic_segment_clear=[&](Vec3 a,Vec3 b,float t0,float t1){
            return dynamic_route_edge_clear(robot_index,a,b,t0,t1);
        };

        robots::PathPlanRequest request;
        request.start=robot.position;
        request.goal=direct_goal;
        request.stop_radius=stop_radius;
        request.corridor_half_width=corridor_half_width;
        request.cell_size=grid_cell;
        request.max_expansions=max_expansions;
        request.traffic_weight=traffic_weight;
        request.alternating_side_preference=((robot_index&1u)==0u)?-1.0f:1.0f;
        request.visibility_skip_limit=18;
        request.nominal_speed_mps=std::max(0.10f,nominal_speed_mps);
        // Use roughly half of the host's logical CPUs for the expensive,
        // embarrassingly-parallel occupancy sampling inside broad path searches.
        // The A* ordering itself remains deterministic on the calling thread.
        const unsigned logical_cpus=std::max(1u,std::thread::hardware_concurrency());
        const unsigned half_system=std::max(1u,logical_cpus/2u);
        // Threading is reserved for broad recovery searches. Small 5/10 m
        // corridor replans stay serial so thread start/join overhead cannot
        // become a new source of micro-stutter.
        request.worker_count=corridor_half_width>=17.5f?half_system:1u;
        return robots::plan_corridor_path(request,callbacks,geometric);
    };

    auto plan_route=[&](std::size_t robot_index,Vec3 destination,float stop_radius,float speed_scale){
        auto& robot=builder_robots_[robot_index];
        clear_route(robot);
        destination.y=first_person_floor_height(destination);
        robot.route_destination=destination;
        robot.route_stop_radius=std::max(0.0f,stop_radius);
        robot.route_speed_scale=std::clamp(speed_scale,0.10f,1.0f);

        const float d=dist_xz(robot.position,destination);
        if(d<=stop_radius+0.06f) return true;
        Vec3 direct_goal=destination;
        if(d>0.001f && stop_radius>0.0f){
            const float travel=std::max(0.0f,d-stop_radius*0.94f);
            direct_goal=robot.position+(destination-robot.position)*(travel/d);
            direct_goal.y=first_person_floor_height(direct_goal);
        }

        const bool full_resolution=robot.route_force_full_replan;
        const float nominal_speed=std::max(0.10f,robot.move_speed_mps*std::clamp(speed_scale,0.10f,1.0f));
        std::vector<Vec3> geometric;
        // Even a full-resolution recovery must respect a rover that is
        // physically stationary in front of us.  The previous recovery used
        // only static-world clearance here, so a head-on pair could repeatedly
        // republish the same straight route through each other's hull.
        const float direct_eta=dist_xz(robot.position,direct_goal)/nominal_speed;
        const bool direct_clear=route_segment_plannable_clear(robot_index,robot.position,direct_goal) &&
            dynamic_route_edge_clear(robot_index,robot.position,direct_goal,0.0f,direct_eta);
        if(direct_clear){
            geometric.push_back(direct_goal);
        }else if(full_resolution){
            // Two seconds without a resolvable/progressing route promotes the
            // next attempt to a full-resolution world search. Stationary/held
            // fleet hulls remain hard obstacles during that search so recovery
            // cannot simply republish a straight path through another rover.
            // A physically nose-to-nose start is resolved separately by the
            // deterministic traffic pull-aside maneuver below.
            const float full_width=std::max(18.0f,(float)playfield_.size()*0.72f);
            if(!build_spatial_route(robot_index,direct_goal,stop_radius,full_width,
                                    0.22f,16000,kRouteGridCellM*1.65f,nominal_speed,true,geometric)){
                // Keep the escalation request, but never retry the same expensive
                // full-map search on the next 90 Hz slice.
                robot.route_replan_cooldown=0.45f;
                return false;
            }
        }else{
            // Normal replans remain corridor-local and cheap. The full-map
            // fallback is deliberately reserved for the 2 s resolution
            // watchdog so routine traffic never causes a frame spike.
            // One spatial search per planning opportunity. A failure advances to
            // a wider corridor on a later tick rather than executing three full
            // searches back-to-back and freezing the visible frame.
            constexpr std::array<float,3> widths{{5.0f,10.0f,18.0f}};
            constexpr std::array<int,3> expansion_caps{{6500,10000,15000}};
            const std::size_t tier=std::min<std::size_t>(robot.route_search_tier,widths.size()-1);
            const float traffic_weight=0.58f+(float)tier*0.18f;
            if(!build_spatial_route(robot_index,direct_goal,stop_radius,widths[tier],
                                    traffic_weight,expansion_caps[tier],kRouteGridCellM,nominal_speed,true,geometric)){
                robot.route_search_tier=(std::uint8_t)std::min<std::size_t>(widths.size()-1,tier+1);
                robot.route_replan_cooldown=0.10f+0.06f*(float)tier;
                return false;
            }
        }

        auto dense=make_continuous_path(robot_index,geometric);
        if(dense.empty()){ robot.route_replan_cooldown=full_resolution?0.45f:0.16f; return false; }

        // Rounding/densification changes the exact spatial trajectory, so run a
        // final space-time preflight on the route that will actually be driven.
        // This closes the gap where an A* path was collision-safe but a smoothed
        // corner cut back through another rover's predicted hull.
        {
            Vec3 previous=robot.position;
            float travelled=0.0f;
            bool dynamic_safe=true;
            for(const Vec3& point:dense){
                const float edge=dist_xz(previous,point);
                const float t0=travelled/nominal_speed;
                const float t1=(travelled+edge)/nominal_speed;
                if(!dynamic_route_edge_clear(robot_index,previous,point,t0,t1)){ dynamic_safe=false; break; }
                travelled+=edge;
                previous=point;
            }
            if(!dynamic_safe){
                robot.route_replan_cooldown=full_resolution?0.28f:0.08f;
                if(!full_resolution) robot.route_search_tier=(std::uint8_t)std::min<int>(2,(int)robot.route_search_tier+1);
                return false;
            }
        }

        // The route is published immediately. Full-fleet trajectory prediction
        // remains in the continuous speed governor. A successful full-resolution
        // recovery returns the rover to the normal cheap planner immediately.
        const bool committed=commit_route(robot_index,dense,speed_scale);
        if(committed) robot.route_search_tier=0;
        if(committed && full_resolution){
            ++robot.route_full_replan_count;
            robot.route_force_full_replan=false;
            robot.route_resolution_elapsed=0.0f;
        }
        return committed;
    };

    auto carrier_standby_position=[&](std::size_t robot_index){
        std::size_t ordinal=0;
        for(std::size_t i=0;i<robot_index && i<builder_robots_.size();++i)
            if(builder_robots_[i].role==RobotRole::Carrier) ++ordinal;
        const float half=(float)playfield_.size()*0.5f;
        const float center_x=std::clamp(starter_stash_center_.x+15.0f,-half+4.1f,half-4.1f);
        const float center_z=std::clamp(starter_stash_center_.z+8.0f,-half+4.1f,half-4.1f);
        const int column=(int)(ordinal%3u)-1;
        const int row=(int)(ordinal/3u);
        Vec3 p{center_x+(float)column*2.60f,0.0f,center_z+(float)row*2.60f};
        p.x=std::clamp(p.x,-half+1.25f,half-1.25f);
        p.z=std::clamp(p.z,-half+1.25f,half-1.25f);
        p.y=first_person_floor_height(p);
        return p;
    };

    auto robot_has_right_of_way=[&](std::size_t a,std::size_t b){
        if(a>=builder_robots_.size() || b>=builder_robots_.size()) return false;
        const auto& ra=builder_robots_[a];
        const auto& rb=builder_robots_[b];
        // Idle carriers are parking traffic, never an immovable obstacle,
        // even while they already have a route toward their standby bay.
        if(rb.mode==BuilderRobotMode::Idle && ra.mode!=BuilderRobotMode::Idle)
            return true;
        if(rb.post_delivery_hold && ra.mode!=BuilderRobotMode::Idle)
            return true;
        if(!rb.route_active && rb.action!=BuilderRobotAction::None)
            return false;
        const float age_delta=ra.route_priority_age-rb.route_priority_age;
        if(std::abs(age_delta)>0.08f) return age_delta>0.0f;
        // Stable tie-break prevents reciprocal hesitation/spinning. Dynamic age
        // provides PIBT-style anti-starvation across successive tasks.
        return a<b;
    };

    auto try_activate_traffic_escape=[&](std::size_t robot_index){
        if(robot_index>=builder_robots_.size()) return false;
        auto& robot=builder_robots_[robot_index];

        // Find the closest rover which is physically close enough to make the
        // current chassis unable to start a detour. Only the lower-priority
        // participant pulls aside; the winner holds its task route. This stable
        // asymmetry prevents both rovers from choosing mirrored escape motions.
        std::size_t blocker_index=kNoRobotIndex;
        float blocker_distance=3.4f;
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const float d=dist_xz(robot.position,builder_robots_[i].position);
            if(d>=blocker_distance) continue;
            blocker_distance=d;
            blocker_index=i;
        }
        if(blocker_index==kNoRobotIndex) return false;

        // An idle carrier that is itself blocked while returning to standby
        // must clear *toward standby*, never use the generic head-on escape
        // which can send it backwards into a construction drop lane. The
        // clearance path may temporarily enter another rover's soft traffic
        // envelope, but every sampled pose must remain physically hull-clear.
        if(robot.role==RobotRole::Carrier && robot.mode==BuilderRobotMode::Idle &&
           robot.action==BuilderRobotAction::None){
            const Vec3 standby=carrier_standby_position(robot_index);
            Vec3 dir=standby-robot.position; dir.y=0.0f;
            const float dl=std::sqrt(dir.x*dir.x+dir.z*dir.z);
            if(dl>0.25f){
                dir=dir*(1.0f/dl);
                Vec3 candidate=robot.position+dir*std::min(3.20f,dl);
                candidate.y=first_person_floor_height(candidate);
                bool legal=robot_position_clear(robot_index,candidate) &&
                    robot_structure_move_allowed(robot,robot.position,candidate) &&
                    route_segment_clear(robot_index,robot.position,candidate);
                if(legal){
                    const float heading=heading_between(robot.position,candidate,robot.heading_degrees);
                    const int samples=std::max(2,(int)std::ceil(dist_xz(robot.position,candidate)/0.16f));
                    const float start_blocker_distance=dist_xz(robot.position,builder_robots_[blocker_index].position);
                    for(int q=1;q<=samples && legal;++q){
                        Vec3 pnt=robot.position+(candidate-robot.position)*((float)q/(float)samples);
                        pnt.y=first_person_floor_height(pnt);
                        if(dist_xz(pnt,builder_robots_[blocker_index].position)+0.02f<start_blocker_distance){ legal=false; break; }
                        for(std::size_t i=0;i<builder_robots_.size();++i){
                            if(i==robot_index) continue;
                            const auto& other=builder_robots_[i];
                            if(robot_hulls_overlap(pnt,heading,robot.role,other.position,other.heading_degrees,other.role,0.0f)){
                                legal=false; break;
                            }
                        }
                    }
                }
                if(legal){
                    robot.traffic_escape_position=candidate;
                    robot.traffic_escape_active=true;
                    clear_route(robot);
                    robot.current_speed_mps=0.0f;
                    robot.route_replan_cooldown=0.0f;
                    robot.route_resolution_elapsed=0.0f;
                    robot.route_force_full_replan=false;
                    robot.blocked_elapsed=0.0f;
                    return true;
                }
            }
        }

        // If useful work is blocked by an idle CR-01, move the idle rover out
        // of the lane instead of sending the working robot into a repeated
        // pull-aside loop. This is the exact no-building.gmsave failure mode.
        auto& blocker_mutable=builder_robots_[blocker_index];
        if(robot.mode!=BuilderRobotMode::Idle && blocker_mutable.role==RobotRole::Carrier &&
           blocker_mutable.mode==BuilderRobotMode::Idle && blocker_mutable.action==BuilderRobotAction::None){
            blocker_mutable.post_delivery_hold=false;
            const Vec3 standby=carrier_standby_position(blocker_index);
            const float bh=blocker_mutable.heading_degrees*PI/180.0f;
            const Vec3 forward{std::sin(bh),0.0f,std::cos(bh)};
            const Vec3 backward=forward*-1.0f;
            Vec3 toward_standby=standby-blocker_mutable.position; toward_standby.y=0.0f;
            const float standby_len=std::sqrt(toward_standby.x*toward_standby.x+toward_standby.z*toward_standby.z);
            if(standby_len>0.05f) toward_standby=toward_standby*(1.0f/standby_len);
            else toward_standby=backward;
            const Vec3 left{-forward.z,0.0f,forward.x};
            const Vec3 right=left*-1.0f;
            // Prefer a pure longitudinal pull-out. It requires no lateral
            // chassis translation and therefore cannot look like sliding.
            const std::array<Vec3,5> escape_dirs{{toward_standby,backward,forward,left,right}};
            const float current_blocker_distance=dist_xz(blocker_mutable.position,robot.position);
            for(const Vec3& dir:escape_dirs){
                Vec3 candidate=blocker_mutable.position+dir*2.80f;
                candidate.y=first_person_floor_height(candidate);
                if(dist_xz(candidate,robot.position)<current_blocker_distance+0.20f) continue;
                if(robot.interaction_position_valid && dist_xz(candidate,robot.interaction_position)<2.20f) continue;
                if(robot.route_active && dist_xz(candidate,robot.route_destination)<2.20f) continue;
                const float travel_heading=heading_between(blocker_mutable.position,candidate,blocker_mutable.heading_degrees);
                if(!robot_position_clear(blocker_index,candidate)) continue;
                if(!robot_structure_move_allowed(blocker_mutable,blocker_mutable.position,candidate)) continue;
                if(!fleet_hull_clear(blocker_index,candidate,travel_heading)) continue;
                blocker_mutable.traffic_escape_position=candidate;
                blocker_mutable.traffic_escape_active=true;
                clear_route(blocker_mutable);
                blocker_mutable.current_speed_mps=0.0f;
                blocker_mutable.route_replan_cooldown=0.0f;
                blocker_mutable.route_resolution_elapsed=0.0f;
                blocker_mutable.route_force_full_replan=false;
                blocker_mutable.blocked_elapsed=0.0f;
                break;
            }
            return false;
        }
        if(robot_has_right_of_way(robot_index,blocker_index)) return false;

        const auto& blocker=builder_robots_[blocker_index];
        Vec3 away=robot.position-blocker.position;
        away.y=0.0f;
        float len=std::sqrt(away.x*away.x+away.z*away.z);
        if(len<0.05f){
            const float h=robot.heading_degrees*PI/180.0f;
            away={-std::sin(h),0.0f,-std::cos(h)};
            len=1.0f;
        }
        away=away*(1.0f/len);
        const float rh=robot.heading_degrees*PI/180.0f;
        const Vec3 chassis_forward{std::sin(rh),0.0f,std::cos(rh)};
        const Vec3 chassis_backward=chassis_forward*-1.0f;
        const Vec3 left{-away.z,0.0f,away.x};
        const Vec3 right{away.z,0.0f,-away.x};
        // First try forward/backward clearance along the chassis axis. Only if
        // neither longitudinal exit is legal do we ask the rover to rotate for
        // a lateral/diagonal escape.
        const std::array<Vec3,8> directions{{chassis_backward,chassis_forward,away,left,right,(away+left)*0.70710678f,(away+right)*0.70710678f,away}};
        const std::array<float,8> distances{{2.45f,2.45f,2.35f,2.10f,2.10f,2.55f,2.55f,3.20f}};
        const float half=(float)playfield_.size()*0.5f-0.60f;

        auto escape_segment_clear=[&](Vec3 candidate){
            const float d=dist_xz(robot.position,candidate);
            if(d<0.25f || !route_segment_clear(robot_index,robot.position,candidate)) return false;
            const int samples=std::max(2,(int)std::ceil(d/0.18f));
            const float heading=heading_between(robot.position,candidate,robot.heading_degrees);
            for(int q=1;q<=samples;++q){
                Vec3 pnt=robot.position+(candidate-robot.position)*((float)q/(float)samples);
                pnt.y=first_person_floor_height(pnt);
                if(!fleet_hull_clear(robot_index,pnt,heading)) return false;
            }
            return true;
        };

        for(std::size_t c=0;c<directions.size();++c){
            Vec3 dir=directions[c];
            const float dl=std::sqrt(dir.x*dir.x+dir.z*dir.z);
            if(dl<0.05f) continue;
            dir=dir*(1.0f/dl);
            Vec3 candidate=robot.position+dir*distances[c];
            candidate.x=std::clamp(candidate.x,-half,half);
            candidate.z=std::clamp(candidate.z,-half,half);
            candidate.y=first_person_floor_height(candidate);
            if(!robot_position_clear(robot_index,candidate)) continue;
            if(!robot_structure_move_allowed(robot,robot.position,candidate)) continue;
            if(!fleet_hull_clear(robot_index,candidate,heading_between(robot.position,candidate,robot.heading_degrees))) continue;
            if(!escape_segment_clear(candidate)) continue;

            robot.traffic_escape_position=candidate;
            robot.traffic_escape_active=true;
            clear_route(robot);
            robot.current_speed_mps=0.0f;
            robot.route_replan_cooldown=0.0f;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            robot.blocked_elapsed=0.0f;
            return true;
        }
        return false;
    };

    auto traffic_speed_limit=[&](std::size_t robot_index,float requested_speed){
        const auto& robot=builder_robots_[robot_index];
        if(!robot.route_active || requested_speed<=0.01f) return 0.0f;
        constexpr std::array<float,7> fractions{{1.00f,0.86f,0.70f,0.54f,0.38f,0.22f,0.0f}};
        constexpr std::array<float,6> horizons{{0.16f,0.28f,0.46f,0.70f,0.96f,1.24f}};
        for(float fraction:fractions){
            const float speed=requested_speed*fraction;
            bool safe=true;
            for(float horizon:horizons){
                const float own_distance=std::min(robot.route_total_length_m,robot.route_progress_m+speed*horizon);
                const Vec3 own=route_position_at_distance(robot,own_distance);
                const float own_heading=route_heading_at_distance(robot,own_distance,robot.heading_degrees);
                for(std::size_t i=0;i<builder_robots_.size();++i){
                    if(i==robot_index) continue;
                    const auto other=predict_robot_pose(i,horizon);
                    if(!robot_hulls_overlap(own,own_heading,robot.role,other.position,other.heading_degrees,
                                            builder_robots_[i].role,kRobotHullClearanceM+0.13f)) continue;
                    // The right-of-way rover may trust the other moving rover to
                    // yield in the medium horizon, but never ignores an imminent
                    // collision or a parked chassis.
                    if(robot_has_right_of_way(robot_index,i) && horizon>kTrafficHardHorizonSeconds) continue;
                    safe=false; break;
                }
                if(!safe) break;
            }
            if(safe) return speed;
        }
        return 0.0f;
    };

    auto move_towards=[&](std::size_t robot_index,Vec3 destination,float stop_radius,float speed_scale){
        auto& robot=builder_robots_[robot_index];
        destination.y=first_person_floor_height(destination);

        // A traffic escape is a short, prevalidated pull-aside maneuver layered
        // underneath the task destination. Translation is always aligned with
        // the chassis: the rover first turns to the escape bearing, or reverses
        // straight backwards when the safe point lies behind it. This removes
        // the old sideways "sliding" escape while keeping deterministic recovery
        // for nose-to-nose conflicts.
        if(robot.traffic_escape_active){
            const float escape_distance=dist_xz(robot.position,robot.traffic_escape_position);
            if(escape_distance<=0.16f){
                robot.traffic_escape_active=false;
                clear_route(robot);
                robot.current_speed_mps=0.0f;
                robot.blocked_elapsed=0.0f;
                robot.route_resolution_elapsed=0.0f;
                robot.route_force_full_replan=false;
                robot.route_replan_cooldown=0.0f;
                return false;
            }

            const float motion_heading=heading_between(robot.position,robot.traffic_escape_position,robot.heading_degrees);
            const float raw_delta=wrap_radians((motion_heading-robot.heading_degrees)*PI/180.0f)*180.0f/PI;
            const bool reverse=std::abs(raw_delta)>135.0f;
            const float desired_heading=reverse?world_math::wrap_degrees(motion_heading+180.0f):motion_heading;
            const float heading_error=std::abs(wrap_radians((desired_heading-robot.heading_degrees)*PI/180.0f)*180.0f/PI);

            if(heading_error>5.0f){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                const float turned=turn_towards_heading(robot.heading_degrees,desired_heading,150.0f*kRobotMovementSpeedMultiplier*dt);
                if(fleet_hull_clear(robot_index,robot.position,turned) ||
                   fleet_hull_hard_clear(robot_index,robot.position,turned)){
                    // During a deliberate clearance maneuver the normal safety
                    // envelope may overlap even though the physical hulls do
                    // not. Tight in-place rotation is allowed only while the
                    // real chassis geometry remains non-intersecting.
                    robot.heading_degrees=turned;
                    robot.blocked_elapsed=0.0f;
                    robot.route_resolution_elapsed=0.0f;
                }else if(heading_error<=12.0f){
                    // A nearby hull can prevent the final few degrees of an
                    // otherwise safe turn. Do not slide diagonally toward the
                    // old target: project the escape target onto the rover's
                    // current longitudinal axis, then execute it straight.
                    const float h=robot.heading_degrees*PI/180.0f;
                    Vec3 axis{std::sin(h),0.0f,std::cos(h)};
                    if(reverse) axis=axis*-1.0f;
                    robot.traffic_escape_position=robot.position+axis*escape_distance;
                    robot.traffic_escape_position.y=first_person_floor_height(robot.traffic_escape_position);
                    robot.blocked_elapsed=0.0f;
                    robot.route_resolution_elapsed=0.0f;
                }else{
                    robot.blocked_elapsed+=dt;
                    robot.route_resolution_elapsed+=dt;
                }
                if(robot.route_resolution_elapsed>=kFullRouteResolutionSeconds){
                    robot.traffic_escape_active=false;
                    robot.route_force_full_replan=true;
                    robot.route_replan_cooldown=0.0f;
                }
                return false;
            }

            const float escape_speed=std::min(0.92f,std::max(0.48f,robot.move_speed_mps*0.42f));
            robot.current_speed_mps=approach(robot.current_speed_mps,escape_speed,kRobotAccelerationMps2*dt);
            const float travel=std::min(escape_distance,robot.current_speed_mps*dt);
            Vec3 candidate=robot.position+(robot.traffic_escape_position-robot.position)*(travel/escape_distance);
            candidate.y=first_person_floor_height(candidate);
            auto escape_fleet_clear=[&](){
                for(std::size_t i=0;i<builder_robots_.size();++i){
                    if(i==robot_index) continue;
                    const auto& other=builder_robots_[i];
                    if(!robot_hulls_overlap(candidate,robot.heading_degrees,robot.role,
                                            other.position,other.heading_degrees,other.role,kRobotHullClearanceM))
                        continue;
                    const bool already_close=robot_hulls_overlap(robot.position,robot.heading_degrees,robot.role,
                                                                 other.position,other.heading_degrees,other.role,kRobotHullClearanceM);
                    if(!already_close) return false;
                    const bool currently_intersecting=robot_hulls_overlap(robot.position,robot.heading_degrees,robot.role,
                                                                          other.position,other.heading_degrees,other.role,0.0f);
                    const bool candidate_intersecting=robot_hulls_overlap(candidate,robot.heading_degrees,robot.role,
                                                                          other.position,other.heading_degrees,other.role,0.0f);
                    if(candidate_intersecting && !currently_intersecting) return false;
                    if(dist_xz(candidate,other.position)<=dist_xz(robot.position,other.position)+0.0005f) return false;
                }
                return true;
            };
            const bool escape_legal=robot_position_clear(robot_index,candidate) &&
                robot_structure_move_allowed(robot,robot.position,candidate) && escape_fleet_clear();
            if(escape_legal){
                robot.position=candidate;
                robot.blocked_elapsed=0.0f;
                robot.route_resolution_elapsed=0.0f;
                robot.route_force_full_replan=false;
                clear_route(robot);
                return false;
            }

            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            robot.blocked_elapsed+=dt;
            robot.route_resolution_elapsed+=dt;
            if(robot.route_resolution_elapsed>=kFullRouteResolutionSeconds){
                robot.traffic_escape_active=false;
                robot.route_force_full_replan=true;
                robot.route_replan_cooldown=0.0f;
            }
            return false;
        }
        speed_scale=std::clamp(speed_scale,0.10f,1.0f);

        auto escalate_resolution=[&](){
            if(robot.route_resolution_elapsed<kFullRouteResolutionSeconds) return false;
            // Escalation is an edge, not a level. Once a full search is already
            // pending, do not clear its failure backoff on every simulation tick.
            // v0.40.6 could otherwise execute the same failed A* search at 90 Hz.
            if(robot.route_force_full_replan) return false;
            if(!robot.traffic_escape_active && try_activate_traffic_escape(robot_index)) return true;
            robot.route_force_full_replan=true;
            clear_route(robot);
            robot.route_replan_cooldown=0.0f;
            return true;
        };
        robot.route_replan_cooldown=std::max(0.0f,robot.route_replan_cooldown-dt);
        const float d=dist_xz(robot.position,destination);
        if(d<=stop_radius+0.025f){
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            robot.blocked_elapsed=0.0f;
            robot.route_priority_age=0.0f;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            clear_route(robot);
            return true;
        }

        const float goal_shift=dist_xz(destination,robot.route_destination);
        // clear_route() intentionally drops route-local stop/speed data.  Do
        // not interpret those cleared fields as a task/destination change on
        // the next tick, otherwise the 2 s full-resolution escalation flag is
        // cancelled before plan_route() can observe it.
        const bool route_contract_changed=robot.route_active &&
            (std::abs(stop_radius-robot.route_stop_radius)>0.12f ||
             std::abs(speed_scale-robot.route_speed_scale)>0.32f);
        const bool destination_changed=goal_shift>std::max(0.50f,stop_radius*0.50f) || route_contract_changed;
        if(destination_changed){
            robot.route_search_tier=0;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            if(robot.route_active) clear_route(robot);
        }

        if(!robot.route_active){
            // Full-resolution retries need backoff as well. Ignoring cooldown
            // while route_force_full_replan was set caused the v0.40.6 CPU spike.
            if(robot.route_replan_cooldown>0.0f){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.blocked_elapsed+=dt;
                robot.route_resolution_elapsed+=dt;
                escalate_resolution();
                return false;
            }
            int& role_route_plans_remaining=(robot.role==RobotRole::Carrier)?carrier_route_plans_remaining:constructor_route_plans_remaining;
            if(robot.route_force_full_replan && robot_full_route_plans_budget_<=0){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.route_resolution_elapsed+=dt;
                return false;
            }
            if(role_route_plans_remaining<=0){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.route_resolution_elapsed+=dt;
                escalate_resolution();
                return false;
            }
            --role_route_plans_remaining;
            if(robot.route_force_full_replan) --robot_full_route_plans_budget_;
            if(!plan_route(robot_index,destination,stop_radius,speed_scale)){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.blocked_elapsed+=dt;
                robot.route_resolution_elapsed+=dt;
                escalate_resolution();
                return false;
            }
            if(!robot.route_active) return dist_xz(robot.position,destination)<=stop_radius+0.035f;
        }

        robot.route_priority_age=std::min(120.0f,robot.route_priority_age+dt);
        const float remaining=std::max(0.0f,robot.route_total_length_m-robot.route_progress_m);
        if(remaining<=0.018f){
            if(!robot.planned_route.empty()) robot.position=robot.planned_route.back();
            robot.current_speed_mps=0.0f;
            robot.blocked_elapsed=0.0f;
            robot.route_priority_age=0.0f;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            clear_route(robot);
            return dist_xz(robot.position,destination)<=stop_radius+0.055f;
        }

        // Smooth acceleration/braking around one continuous arc-length clock.
        // The braking envelope prevents the old endpoint snap even when the last
        // route segment is shorter than a normal simulation step.
        const float braking_speed=std::sqrt(std::max(0.0f,2.0f*kRobotBrakingMps2*remaining));
        const float requested=std::min(robot.route_cruise_speed_mps,braking_speed);
        const float traffic_limit=traffic_speed_limit(robot_index,requested);
        const bool yielding=traffic_limit<requested*0.34f && remaining>0.22f;
        robot.route_waiting_for_clearance=yielding;
        if(yielding) robot.route_wait_elapsed+=dt; else robot.route_wait_elapsed=0.0f;

        const float accel=(traffic_limit<robot.current_speed_mps)?kRobotBrakingMps2:kRobotAccelerationMps2;
        robot.current_speed_mps=approach(robot.current_speed_mps,traffic_limit,accel*dt);
        float advance=std::min(remaining,std::max(0.0f,robot.current_speed_mps*dt));

        if(advance<0.0005f){
            if(requested>0.15f) robot.blocked_elapsed+=dt;
            robot.route_resolution_elapsed+=dt;
            // Cheap traffic replans are allowed before the hard timeout, but
            // they do not reset route_resolution_elapsed. At 2 s all local
            // assumptions are discarded and the next search is full-resolution.
            if(escalate_resolution()){
                return false;
            }else if(robot.route_wait_elapsed>kTrafficReplanYieldSeconds && robot.route_replan_cooldown<=0.0f){
                clear_route(robot);
                robot.route_replan_cooldown=0.01f;
                robot.route_wait_elapsed=0.0f;
            }
            return false;
        }

        const float next_progress=std::min(robot.route_total_length_m,robot.route_progress_m+advance);
        Vec3 candidate=route_position_at_distance(robot,next_progress);
        // Translation must follow the chassis longitudinal axis. Turn toward
        // the *actual next displacement* before changing position; using only a
        // distant look-ahead heading allowed the old implementation to drift
        // sideways through bends while its body caught up visually.
        const float motion_heading=heading_between(robot.position,candidate,robot.heading_degrees);
        float candidate_heading=turn_towards_heading(robot.heading_degrees,motion_heading,
                                                      std::max(kRobotTurnRateDegreesPerSecond,210.0f)*dt);
        const float translation_heading_error=std::abs(wrap_radians((motion_heading-candidate_heading)*PI/180.0f)*180.0f/PI);
        // Turning itself is hull-checked. If the rover is not yet aligned, turn
        // in place and brake to zero instead of translating laterally.
        if(!fleet_hull_clear(robot_index,robot.position,candidate_heading) &&
           !fleet_hull_hard_clear(robot_index,robot.position,candidate_heading)){
            candidate_heading=robot.heading_degrees;
        }
        if(translation_heading_error>6.0f ||
           std::abs(wrap_radians((motion_heading-candidate_heading)*PI/180.0f)*180.0f/PI)>6.0f){
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            if(fleet_hull_clear(robot_index,robot.position,candidate_heading) ||
               fleet_hull_hard_clear(robot_index,robot.position,candidate_heading))
                robot.heading_degrees=candidate_heading;
            robot.route_waiting_for_clearance=false;
            robot.blocked_elapsed=0.0f;
            return false;
        }

        const Vec3 midpoint=route_position_at_distance(robot,robot.route_progress_m+advance*0.5f);
        const bool legal=robot_position_clear(robot_index,midpoint) && robot_position_clear(robot_index,candidate) &&
                         robot_structure_move_allowed(robot,robot.position,midpoint) &&
                         robot_structure_move_allowed(robot,midpoint,candidate) &&
                         fleet_hull_clear(robot_index,midpoint,candidate_heading) &&
                         fleet_hull_clear(robot_index,candidate,candidate_heading);
        if(!legal){
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            robot.blocked_elapsed+=dt;
            robot.route_waiting_for_clearance=true;
            robot.route_wait_elapsed+=dt;
            robot.route_resolution_elapsed+=dt;
            if(escalate_resolution()){
                return false;
            }else if(robot.blocked_elapsed>0.70f && robot.route_replan_cooldown<=0.0f){
                clear_route(robot);
                robot.route_replan_cooldown=0.01f;
            }
            return false;
        }
        robot.position=candidate;
        robot.heading_degrees=candidate_heading;
        robot.route_progress_m=std::min(robot.route_total_length_m,robot.route_progress_m+advance);
        if(advance>0.004f){
            robot.route_search_tier=0;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
        }
        while(robot.route_cursor<robot.planned_route_distances.size() &&
              robot.planned_route_distances[robot.route_cursor]<=robot.route_progress_m+0.025f)
            ++robot.route_cursor;
        robot.blocked_elapsed=0.0f;

        if(robot.route_progress_m>=robot.route_total_length_m-0.015f){
            if(!robot.planned_route.empty()) robot.position=robot.planned_route.back();
            robot.current_speed_mps=0.0f;
            robot.route_priority_age=0.0f;
            robot.route_resolution_elapsed=0.0f;
            robot.route_force_full_replan=false;
            clear_route(robot);
            return dist_xz(robot.position,destination)<=stop_radius+0.060f;
        }
        return false;
    };

    auto robot_safe_for_next_stage=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::uint8_t next_step){
        const auto& robot=builder_robots_[robot_index];
        const float radius=robot_body_radius(robot.role)+kRobotObstacleMargin;
        if(target_future_envelope_contains(target,next_step,robot.position,radius)) return false;
        if(target_blocks_at_step(target,next_step,robot.position,radius)) return false;
        return true;
    };

    auto safe_stage_commit_destination=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::uint8_t next_step){
        const auto& robot=builder_robots_[robot_index];
        const float body_radius=robot_body_radius(robot.role);
        Vec3 center=construction_target_position(target);
        float base_radius=1.35f;
        float rotation_degrees=0.0f;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& mesh=habitat_models_.mesh(h);
            center=h.position; rotation_degrees=h.rotation_degrees;
            base_radius=0.5f*std::sqrt(mesh.footprint_x*mesh.footprint_x+mesh.footprint_z*mesh.footprint_z)+body_radius+2.75f;
        }else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& mesh=resource_models_.mesh(item);
            center=item.position; rotation_degrees=item.rotation_degrees;
            base_radius=0.5f*std::sqrt(mesh.footprint_x*mesh.footprint_x+mesh.footprint_z*mesh.footprint_z)+body_radius+2.75f;
        }

        Vec3 best=robot.position;
        float best_score=std::numeric_limits<float>::max();
        bool found=false;
        // Search several complete rings so even a robot deep inside a large
        // blueprint always gets an exterior destination before mesh reveal.
        for(int ring=0;ring<4;++ring){
            const float radius=base_radius+(float)ring*0.85f;
            for(int sample=0;sample<32;++sample){
                const float a=rotation_degrees*PI/180.0f+(2.0f*PI*(float)sample/32.0f);
                Vec3 candidate=center+Vec3{std::sin(a)*radius,0.0f,std::cos(a)*radius};
                candidate.y=first_person_floor_height(candidate);
                if(!robot_position_clear(robot_index,candidate)) continue;
                if(!fleet_position_clear(robot_index,candidate)) continue;
                if(target_future_envelope_contains(target,next_step,candidate,body_radius+kRobotObstacleMargin)) continue;
                if(target_blocks_at_step(target,next_step,candidate,body_radius+kRobotObstacleMargin)) continue;
                // Candidate itself must be legal with respect to completed
                // structures/pressure volumes. The actual escape route is
                // calculated by the cooperative planner before movement starts.
                if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
                const float turn=std::abs(wrap_radians(a-robot.heading_degrees*PI/180.0f));
                const float score=dist_xz(robot.position,candidate)+turn*0.25f;
                if(score<best_score){ best_score=score; best=candidate; found=true; }
            }
            if(found) break;
        }
        if(!found){
            // Deterministic last-resort point directly away from the target.
            Vec3 away=robot.position-center;
            const float len=std::sqrt(away.x*away.x+away.z*away.z);
            if(len<0.01f) away={1.0f,0.0f,0.0f};
            else away=away*(1.0f/len);
            best=center+away*(base_radius+2.5f);
            best.y=first_person_floor_height(best);
        }
        return best;
    };

    auto fleet_safe_for_next_stage=[&](const ConstructionTargetRef& target,std::uint8_t next_step){
        for(std::size_t i=0;i<builder_robots_.size();++i)
            if(!robot_safe_for_next_stage(i,target,next_step)) return false;
        return true;
    };

    // If a constructor finished its visual build-up on the previous frame,
    // evacuate every robot from the next step's future mesh before normal AI
    // resumes. This prevents an idle carrier (or another constructor passing
    // through the blueprint) from being sealed inside by somebody else's step.
    std::vector<bool> safety_repositioning(builder_robots_.size(),false);
    for(std::size_t ci=0;ci<builder_robots_.size();++ci){
        auto& constructor=builder_robots_[ci];
        if(constructor.role!=RobotRole::Constructor || !constructor.stage_commit_pending || !constructor.target.valid()) continue;
        const auto next_step=(std::uint8_t)std::min<int>((int)construction_target_step(constructor.target)+1,(int)construction_target_total_steps(constructor.target));
        for(std::size_t ri=0;ri<builder_robots_.size();++ri){
            if(robot_safe_for_next_stage(ri,constructor.target,next_step)) continue;
            Vec3 destination=safe_stage_commit_destination(ri,constructor.target,next_step);
            auto& evac=builder_robots_[ri];
            evac.stage_commit_safe_position=destination;
            // Evacuation owns the robot completely; work animations must not
            // keep the watchdog artificially "alive" while the chassis is
            // failing to leave a future wall volume.
            clear_action(evac);
            // Safety destinations are already body-clear points; do not stop a full
            // 20 cm short of them, because that can still leave the robot
            // intersecting the next stage's clearance envelope.
            const bool reached=move_towards(ri,destination,0.025f,0.72f);
            if(!reached && evac.blocked_elapsed>2.0f){
                // Try a different exterior sector before considering the final
                // anti-entrapment fallback. This remains local to one robot.
                evac.service_slot_offset=(std::uint8_t)((evac.service_slot_offset+7u)%24u);
                clear_route(evac);
                evac.route_replan_cooldown=0.0f;
                destination=safe_stage_commit_destination(ri,constructor.target,next_step);
                evac.stage_commit_safe_position=destination;
            }
            if(!reached && evac.blocked_elapsed>5.0f &&
               robot_position_clear(ri,destination) && fleet_position_clear(ri,destination) &&
               !target_future_envelope_contains(constructor.target,next_step,destination,robot_body_radius(evac.role)+kRobotObstacleMargin) &&
               !target_blocks_at_step(constructor.target,next_step,destination,robot_body_radius(evac.role)+kRobotObstacleMargin)){
                // Absolute invariant: construction is never allowed to seal a
                // robot into new geometry. If a pathological dynamic traffic
                // arrangement prevents evacuation for several seconds, place
                // the robot at the already validated exterior safety point and
                // resume normal independent routing there. This is intentionally
                // a last-resort safety relocation, not ordinary navigation.
                evac.position=destination;
                evac.visual_position=destination;
                evac.visual_velocity={};
                evac.current_speed_mps=0.0f;
                evac.blocked_elapsed=0.0f;
                clear_route(evac);
            }
            safety_repositioning[ri]=true;
        }
    }

    // Pick a collision-free interaction point around a ground pallet rather
    // than routing to the pallet centre. The latter is collision geometry and,
    // with several adjacent resource stacks, could leave a carrier parked in a
    // pocket where the next required material had no legal departure route.
    auto ground_service_point=[&](std::size_t robot_index,std::size_t pile_index){
        struct ServicePoint { bool valid{false}; Vec3 position{}; float distance{std::numeric_limits<float>::max()}; } result;
        if(robot_index>=builder_robots_.size() || pile_index>=ground_resource_piles_.size()) return result;
        const auto& robot=builder_robots_[robot_index];
        const auto& pile=ground_resource_piles_[pile_index];
        const float package_radius=std::max(pile.radius,resource_package_half_width(pile.resource,pile.amount));
        const float service_radius=robot_resource_clearance_radius(robot.role)+package_radius+kRobotObstacleMargin+0.12f;

        // The starter reserve is a north/south pallet row with an intentionally
        // open service aisle on its east side. Never send a carrier between two
        // adjacent pallets or behind the row near the playfield boundary: those
        // positions can be pickup-legal yet have no hull-safe departure path.
        const bool starter_pallet=std::abs(pile.position.x-(starter_stash_center_.x-2.15f))<0.35f &&
                                  std::abs(pile.position.z-starter_stash_center_.z)<6.8f;
        if(starter_pallet){
            Vec3 candidate=pile.position+Vec3{service_radius,0.0f,0.0f};
            candidate.y=first_person_floor_height(candidate);
            if(robot_position_clear(robot_index,candidate) &&
               robot_structure_move_allowed(robot,candidate,candidate)){
                // Multiple carriers may reserve the same material bay. The
                // per-lane queue below controls admission; requiring the bay to
                // be fleet-empty here would prevent followers from planning
                // their trip until the first rover had completely finished.
                result={true,candidate,dist_xz(robot.position,candidate)};
                return result;
            }
            // Never offset the pickup bay along Z. The starter stacks are only
            // 1.45 m apart, so a diagonal fallback can be pickup-legal while
            // placing the chassis in the throat of the neighbouring lane. If
            // the exact front bay is occupied, keep the assignment queued and
            // retry it instead of inventing an unreachable interaction point.
            return result;
        }

        // Select the closest statically legal side once, then the robot stores
        // that station for the lifetime of the pickup operation. This gives a
        // reachable approach from either side of a depot without making the
        // interaction target drift while the robot is driving toward it.
        const float preferred=std::atan2(robot.position.x-pile.position.x,robot.position.z-pile.position.z);
        // Preserve the nearest-side preference but deliberately fan multiple
        // robots around a stack. A recovery offset is persistent until the
        // task/target changes, so a blocked pickup side is not selected again
        // immediately after the watchdog fires.
        int recovery_delta=0;
        if(robot.service_slot_offset!=0){
            constexpr std::array<int,6> recovery{{2,-2,3,-3,4,-4}};
            recovery_delta=recovery[((std::size_t)robot.service_slot_offset/5u)%recovery.size()];
        }
        // Start from the physically nearest side of the pallet. The old
        // robot_index*5 phase could deliberately dispatch an east-side carrier
        // behind the pallet row or between neighbouring stacks, creating a
        // legal pickup point with no practical departure corridor. Fleet hull
        // checks below naturally fan simultaneous users into the next free slot.
        const int preferred_slot=(int)std::lround(preferred/(2.0f*PI)*24.0f)+recovery_delta;
        for(int sample=0;sample<24;++sample){
            const int delta=(sample==0)?0:((sample+1)/2)*(sample%2?1:-1);
            const int slot=(preferred_slot+delta+24*8)%24;
            const float a=(2.0f*PI*(float)slot)/24.0f;
            Vec3 candidate=pile.position+Vec3{std::sin(a)*service_radius,0.0f,std::cos(a)*service_radius};
            candidate.y=first_person_floor_height(candidate);
            if(!robot_position_clear(robot_index,candidate)) continue;
            if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
            if(!fleet_position_clear(robot_index,candidate)) continue;
            const float d=dist_xz(robot.position,candidate);
            result={true,candidate,d};
            break;
        }
        return result;
    };

    auto interaction_station_reserved=[&](std::size_t robot_index,const ConstructionTargetRef& target,Vec3 candidate,float candidate_heading){
        if(robot_index>=builder_robots_.size()) return true;
        const auto& robot=builder_robots_[robot_index];
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(!other.interaction_position_valid || !other.target.valid() || !other.target.same_as(target)) continue;
            float other_heading=other.heading_degrees;
            const Vec3 focus=other.interaction_focus_position;
            if(dist_xz(other.interaction_position,focus)>0.15f)
                other_heading=heading_between(other.interaction_position,focus,other_heading);
            if(robot_hulls_overlap(candidate,candidate_heading,robot.role,
                                   other.interaction_position,other_heading,other.role,kRobotHullClearanceM+0.28f)) return true;
        }
        return false;
    };

    auto construction_service_point=[&](std::size_t robot_index,const ConstructionTargetRef& target){
        Vec3 center=construction_target_position(target);
        if(robot_index>=builder_robots_.size()) return center;
        const auto& robot=builder_robots_[robot_index];
        float footprint_x=0.0f,footprint_z=0.0f;
        float rotation=0.0f;
        bool rectangular=false;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& m=habitat_models_.mesh(h);
            center=h.position; rotation=h.rotation_degrees;
            footprint_x=m.footprint_x; footprint_z=m.footprint_z; rectangular=true;
        }else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& m=resource_models_.mesh(item);
            center=item.position; rotation=item.rotation_degrees;
            footprint_x=m.footprint_x; footprint_z=m.footprint_z; rectangular=true;
        }

        const RobotFootprint hull=robot_footprint(robot.role);
        const float work_margin=robot.role==RobotRole::Constructor?0.20f:0.50f;
        const int slots=32;
        const int start=(int)((robot_index*7u+(std::size_t)target.kind*5u+target.index*3u+
                               (std::size_t)robot.service_slot_offset)%slots);
        Vec3 fallback=center;
        float fallback_distance=std::numeric_limits<float>::max();
        for(int n=0;n<slots;++n){
            const int signed_step=(n==0)?0:((n+1)/2)*(n%2?1:-1);
            const int slot=(start+signed_step+slots*8)%slots;
            const float a=rotation*PI/180.0f+(2.0f*PI*(float)slot)/(float)slots;
            float ring=1.10f+hull.half_length+work_margin;
            if(rectangular){
                const float local_a=a-rotation*PI/180.0f;
                const float dx=std::abs(std::sin(local_a));
                const float dz=std::abs(std::cos(local_a));
                const float hx=std::max(0.25f,footprint_x*0.5f);
                const float hz=std::max(0.25f,footprint_z*0.5f);
                const float tx=dx>0.0001f?hx/dx:std::numeric_limits<float>::max();
                const float tz=dz>0.0001f?hz/dz:std::numeric_limits<float>::max();
                ring=std::min(tx,tz)+hull.half_length+work_margin;
            }
            Vec3 candidate=center+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
            candidate.y=first_person_floor_height(candidate);
            const float face_heading=heading_between(candidate,center,robot.heading_degrees);
            if(!robot_position_clear(robot_index,candidate)) continue;
            if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
            if(!fleet_hull_clear(robot_index,candidate,face_heading)) continue;
            if(interaction_station_reserved(robot_index,target,candidate,face_heading)) continue;
            const float d=dist_xz(robot.position,candidate);
            if(d<fallback_distance){ fallback=candidate; fallback_distance=d; }
            if(n>=8 || d<4.0f) return candidate;
        }
        return fallback;
    };

    constexpr float kConstructionBayArrivalM=0.28f;

    // Work points are deterministic perimeter sectors. A build stage now uses
    // only two opposite sectors and rotates that pair between stages, avoiding
    // a full lap around the structure for every single construction increment.
    auto construction_work_point=[&](std::size_t robot_index,const ConstructionTargetRef& target,int phase){
        if(robot_index>=builder_robots_.size()) return construction_target_position(target);
        const auto& robot=builder_robots_[robot_index];
        Vec3 center=construction_target_position(target);
        float footprint_x=1.6f,footprint_z=1.6f,rotation=0.0f;
        bool rectangular=false;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& mesh=habitat_models_.mesh(h);
            center=h.position; footprint_x=mesh.footprint_x; footprint_z=mesh.footprint_z; rotation=h.rotation_degrees; rectangular=true;
        }else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& mesh=resource_models_.mesh(item);
            center=item.position; footprint_x=mesh.footprint_x; footprint_z=mesh.footprint_z; rotation=item.rotation_degrees; rectangular=true;
        }
        std::size_t ordinal=0,total_workers=0;
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(builder_robots_[i].role!=RobotRole::Constructor || !builder_robots_[i].target.valid() ||
               !builder_robots_[i].target.same_as(target)) continue;
            if(i<robot_index) ++ordinal;
            ++total_workers;
        }
        const int sector=((phase%4)+4+(int)(ordinal%4u))%4;
        const float local_angle=(float)sector*0.5f*PI;
        const float a=rotation*PI/180.0f+local_angle;
        const RobotFootprint hull=robot_footprint(robot.role);
        float ring=1.35f+hull.half_length;
        if(rectangular){
            const float dx=std::abs(std::sin(local_angle));
            const float dz=std::abs(std::cos(local_angle));
            const float hx=std::max(0.25f,footprint_x*0.5f);
            const float hz=std::max(0.25f,footprint_z*0.5f);
            const float tx=dx>0.0001f?hx/dx:std::numeric_limits<float>::max();
            const float tz=dz>0.0001f?hz/dz:std::numeric_limits<float>::max();
            ring=std::min(tx,tz)+hull.half_length+0.22f;
        }
        Vec3 outward{std::sin(a),0.0f,std::cos(a)};
        Vec3 tangent{-outward.z,0.0f,outward.x};
        float along=0.0f;
        if(total_workers>1){
            const float centered=(float)ordinal-((float)total_workers-1.0f)*0.5f;
            along=centered*1.35f;
            if(rectangular){
                const bool east_west=(sector%2)==1;
                const float side_length=east_west?footprint_z:footprint_x;
                const float limit=std::max(0.0f,side_length*0.5f-0.85f);
                along=std::clamp(along,-limit,limit);
            }
        }
        Vec3 p=center+outward*ring+tangent*along;
        p.y=first_person_floor_height(p);
        return p;
    };


    auto construction_wait_point=[&](std::size_t robot_index,const ConstructionTargetRef& target){
        // Waiting constructors must not occupy the one shared delivery apron.
        // Choose the perimeter work sector farthest from the material drop so a
        // CR-01 always has a physically solvable approach while materials are
        // still missing.
        const Vec3 drop=construction_drop_position(target);
        Vec3 best=construction_work_point(robot_index,target,0);
        float best_d=dist_xz(best,drop);
        for(int phase=1;phase<4;++phase){
            const Vec3 candidate=construction_work_point(robot_index,target,phase);
            const float d=dist_xz(candidate,drop);
            if(d>best_d){ best=candidate; best_d=d; }
        }
        return best;
    };

    auto construction_drop_staging_point=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::size_t rank){
        const Vec3 drop=construction_drop_position(target);
        Vec3 center=drop;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()) center=placed_habitats_[target.index].position;
        else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()) center=placed_resources_[target.index].position;
        Vec3 outward=drop-center;
        float len=std::sqrt(outward.x*outward.x+outward.z*outward.z);
        if(len<0.1f){ outward={1.0f,0.0f,0.0f}; len=1.0f; }
        outward=outward*(1.0f/len);
        Vec3 tangent{-outward.z,0.0f,outward.x};
        const float side=((robot_index&1u)?1.0f:-1.0f)*(0.55f+0.35f*(float)(rank%2u));
        Vec3 p=drop+outward*(1.75f+1.65f*(float)rank)+tangent*side;
        p.y=first_person_floor_height(p);
        return p;
    };

    auto construction_drop_exit_point=[&](std::size_t robot_index,const ConstructionTargetRef& target){
        const Vec3 drop=construction_drop_position(target);
        Vec3 center=construction_target_position(target);
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()) center=placed_habitats_[target.index].position;
        else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()) center=placed_resources_[target.index].position;
        Vec3 outward=drop-center;
        float len=std::sqrt(outward.x*outward.x+outward.z*outward.z);
        if(len<0.1f){ outward={1.0f,0.0f,0.0f}; len=1.0f; }
        outward=outward*(1.0f/len);
        Vec3 tangent{-outward.z,0.0f,outward.x};
        std::size_t ordinal=0;
        for(std::size_t i=0;i<robot_index && i<builder_robots_.size();++i)
            if(builder_robots_[i].role==RobotRole::Carrier) ++ordinal;
        // Incoming traffic queues on the outward radial lane. Departures peel
        // laterally away from that lane, then remain outside the footprint.
        // Separate lateral bays let several finished CR-01s hold without ever
        // occupying the next rover's approach corridor.
        const float lateral=2.80f+1.55f*(float)(ordinal%4u);
        Vec3 p=drop+outward*2.35f+tangent*lateral;
        p.y=first_person_floor_height(p);
        return p;
    };

    auto drop_queue_rank=[&](std::size_t robot_index,const ConstructionTargetRef& target){
        std::size_t rank=0;
        if(robot_index>=builder_robots_.size()) return rank;
        const Vec3 drop=construction_drop_position(target);
        const auto& robot=builder_robots_[robot_index];
        const float own=dist_xz(robot.position,drop);
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(!other.target.valid() || !other.target.same_as(target)) continue;
            // The unload throat remains owned by the previous CR-01 until its
            // hull has physically cleared the apron. An arriving rover must not
            // enter behind it and pin the outgoing chassis against the pile.
            if(other.role==RobotRole::Carrier && other.drop_exit_pending){ ++rank; continue; }
            if(other.mode!=BuilderRobotMode::Delivering || other.cargo.empty()) continue;
            const float d=dist_xz(other.position,drop);
            // A rover already in the unloading throat always keeps ownership;
            // otherwise queue by actual arrival distance with index tie-break.
            if(d<1.05f || d+0.18f<own || (std::abs(d-own)<=0.18f && i<robot_index)) ++rank;
        }
        return rank;
    };

    auto storage_service=[&](std::size_t index,std::size_t robot_index){
        if(index>=placed_resources_.size()) return Vec3{};
        const auto& structure=placed_resources_[index];
        const auto& mesh=resource_models_.mesh(structure);
        if(robot_index>=builder_robots_.size())
            return structure.position+world_math::rotate_y({mesh.footprint_x*0.5f+1.15f,0.0f,0.0f},structure.rotation_degrees);
        const auto& robot=builder_robots_[robot_index];
        const float ring=0.5f*std::sqrt(mesh.footprint_x*mesh.footprint_x+mesh.footprint_z*mesh.footprint_z)+
                         robot_body_radius(robot.role)+0.92f;
        const int slots=24;
        const int start=(int)((robot_index*7u+robot.service_slot_offset)%slots);
        for(int n=0;n<slots;++n){
            // Prefer the structure's service side first, then spread around the
            // perimeter. 2*pi slots provide >1 m spacing even on small domes.
            const int signed_step=(n==0)?0:((n+1)/2)*(n%2?1:-1);
            const int slot=(start+signed_step+slots*8)%slots;
            const float a=structure.rotation_degrees*PI/180.0f+(2.0f*PI*(float)slot)/(float)slots;
            Vec3 candidate=structure.position+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
            candidate.y=first_person_floor_height(candidate);
            const float candidate_heading=heading_between(candidate,structure.position,robot.heading_degrees);
            if(!robot_position_clear(robot_index,candidate)) continue;
            if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
            if(!fleet_hull_clear(robot_index,candidate,candidate_heading)) continue;
            bool reserved=false;
            for(std::size_t oi=0;oi<builder_robots_.size();++oi){
                if(oi==robot_index) continue;
                const auto& other=builder_robots_[oi];
                if(!other.interaction_position_valid || dist_xz(other.interaction_focus_position,structure.position)>0.35f) continue;
                const float other_heading=heading_between(other.interaction_position,structure.position,other.heading_degrees);
                if(robot_hulls_overlap(candidate,candidate_heading,robot.role,
                                       other.interaction_position,other_heading,other.role,
                                       kRobotHullClearanceM+0.24f)){
                    reserved=true;
                    break;
                }
            }
            if(reserved) continue;
            return candidate;
        }
        Vec3 fallback=structure.position+world_math::rotate_y({mesh.footprint_x*0.5f+1.35f,0.0f,0.0f},structure.rotation_degrees);
        fallback.y=first_person_floor_height(fallback);
        return fallback;
    };

    auto choose_source=[&](std::size_t robot_index,const ResourceBundle& need){
        SourceChoice storage{};
        SourceChoice ground{};
        if(robot_index>=builder_robots_.size()) return ground;
        const auto& robot=builder_robots_[robot_index];
        for(std::size_t ri=0;ri<placed_resources_.size();++ri){
            const auto& structure=placed_resources_[ri];
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            for(std::size_t bi=0;bi<structure.stored_boxes.size();++bi){
                const auto& box=structure.stored_boxes[bi];
                if(box.amount<=0 || need.amount(box.resource)<=0) continue;
                const Vec3 service=storage_service(ri,robot_index);
                const float d=dist_xz(robot.position,service);
                if(d<storage.distance) storage={true,false,ri,bi,box.resource,service,d};
            }
        }
        for(std::size_t i=0;i<ground_resource_piles_.size();++i){
            const auto& pile=ground_resource_piles_[i];
            if(pile.amount<=0 || need.amount(pile.resource)<=0) continue;
            const auto service=ground_service_point(robot_index,i);
            if(!service.valid) continue;
            if(service.distance<ground.distance) ground={true,true,i,0,pile.resource,service.position,service.distance};
        }
        return storage.valid?storage:ground;
    };

    auto clear_interaction_station=[](BuilderRobotState& robot){
        robot.interaction_position_valid=false;
        robot.interaction_position={};
        robot.interaction_focus_position={};
        robot.pickup_lane_claimed=false;
        robot.pickup_queue_age=0.0f;
    };

    auto acquire_source_station=[&](std::size_t robot_index,const ResourceBundle& need){
        if(robot_index>=builder_robots_.size()) return false;
        auto& robot=builder_robots_[robot_index];
        if(robot.interaction_position_valid && need.amount(robot.interaction_material)>0) return true;
        const SourceChoice best=choose_source(robot_index,need);
        if(!best.valid){ clear_interaction_station(robot); return false; }
        robot.interaction_position=best.position;
        robot.interaction_material=best.material;
        if(best.ground && best.index<ground_resource_piles_.size())
            robot.interaction_focus_position=ground_resource_piles_[best.index].position;
        else if(!best.ground && best.index<placed_resources_.size())
            robot.interaction_focus_position=placed_resources_[best.index].position;
        else robot.interaction_focus_position=best.position;
        robot.interaction_position_valid=true;
        return true;
    };

    auto starter_pallet_focus=[&](Vec3 focus){
        return std::abs(focus.x-(starter_stash_center_.x-2.15f))<0.40f &&
               std::abs(focus.z-starter_stash_center_.z)<6.8f;
    };

    // The starter depot has a deliberately open east-side traffic spine. A
    // robot always enters a material lane from that spine, drives straight to
    // the pickup bay, and exits straight back to the spine before joining its
    // construction route. This removes diagonal cuts through adjacent pallet
    // lanes and gives every pickup a deterministic, hull-safe egress.
    auto starter_depot_entry=[&](Vec3 pallet){
        Vec3 gate{starter_stash_center_.x+1.85f,0.0f,pallet.z};
        gate.y=first_person_floor_height(gate);
        return gate;
    };
    auto starter_depot_outer_gate=[&](Vec3 pallet){
        Vec3 gate{starter_stash_center_.x+3.70f,0.0f,pallet.z};
        gate.y=first_person_floor_height(gate);
        return gate;
    };
    auto starter_lane_inside=[&](Vec3 pallet,Vec3 position){
        const Vec3 inner=starter_depot_entry(pallet);
        return std::abs(position.z-pallet.z)<=0.78f &&
               position.x<=inner.x+0.35f && position.x>=pallet.x-0.65f;
    };
    auto starter_lane_queue_rank=[&](std::size_t robot_index,MaterialResource material){
        std::size_t rank=0;
        if(robot_index>=builder_robots_.size()) return rank;
        const auto& robot=builder_robots_[robot_index];
        if(!robot.interaction_position_valid) return rank;
        const bool own_inside=starter_lane_inside(robot.interaction_focus_position,robot.position);
        const float own_age=robot.pickup_queue_age;

        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(other.mode!=BuilderRobotMode::Fetching || !other.interaction_position_valid ||
               other.interaction_material!=material || !starter_pallet_focus(other.interaction_focus_position)) continue;
            const bool other_inside=starter_lane_inside(other.interaction_focus_position,other.position);
            // Physical occupancy is authoritative. A rover already past the
            // inner gate cannot lose the lane because a watchdog reset a flag
            // or an older ticket reaches the outer gate later.
            if(own_inside){
                if(other_inside && other.position.x<robot.position.x-0.18f) ++rank;
                continue;
            }
            if(other_inside){ ++rank; continue; }
            if(robot.pickup_lane_claimed) continue;
            if(other.pickup_lane_claimed){ ++rank; continue; }
            if(other.pickup_queue_age>own_age+0.02f){
                ++rank;
            }else if(std::abs(other.pickup_queue_age-own_age)<=0.02f){
                // When BR-02 fallback hauling and CR-01 reach the same pallet
                // in the same dispatch window, logistics owns the lane. The old
                // index-only tie-break let a distant builder freeze a nearby
                // carrier while both waited on the same material reservation.
                if(other.role!=robot.role){
                    if(other.role==RobotRole::Carrier) ++rank;
                }else if(i<robot_index){
                    ++rank;
                }
            }
        }
        return rank;
    };
    auto starter_depot_admission_rank=[&](std::size_t robot_index){
        if(robot_index>=builder_robots_.size()) return std::size_t{0};
        const auto& robot=builder_robots_[robot_index];
        if(!robot.interaction_position_valid || !starter_pallet_focus(robot.interaction_focus_position)) return std::size_t{0};
        std::size_t rank=0;
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            if(other.mode!=BuilderRobotMode::Fetching || !other.interaction_position_valid ||
               !starter_pallet_focus(other.interaction_focus_position)) continue;
            const bool other_inside=starter_lane_inside(other.interaction_focus_position,other.position);
            if(other_inside || other.pickup_lane_claimed){ ++rank; continue; }
            if(other.pickup_queue_age>robot.pickup_queue_age+0.02f){ ++rank; continue; }
            if(std::abs(other.pickup_queue_age-robot.pickup_queue_age)<=0.02f){
                // Carriers get the same-tick dispatch tie over BR-02 fallback
                // hauling; builders only fill genuinely uncovered logistics.
                if(other.role!=robot.role){ if(other.role==RobotRole::Carrier) ++rank; }
                else if(i<robot_index) ++rank;
            }
        }
        return rank;
    };

    auto starter_lane_occupied=[&](std::size_t robot_index,MaterialResource material,Vec3 pallet){
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==robot_index) continue;
            const auto& other=builder_robots_[i];
            // Fetching-vs-fetching ownership is fully resolved by
            // starter_lane_queue_rank(), including physical depth. This helper
            // only protects an outbound loaded rover that still occupies the
            // same throat; incoming traffic must wait until it has cleared.
            if(other.mode!=BuilderRobotMode::Delivering || other.cargo.amount(material)<=0) continue;
            if(starter_lane_inside(pallet,other.position)) return true;
        }
        return false;
    };

    auto starter_depot_exit_needed=[&](BuilderRobotState& robot){
        if(robot.cargo.empty()){
            robot.starter_exit_pending=false;
            robot.starter_exit_completed=false;
            return false;
        }
        if(robot.starter_exit_completed) return false;
        const bool in_depot_band=std::abs(robot.position.z-starter_stash_center_.z)<6.8f;
        if(!robot.starter_exit_pending){
            // Loaded legacy saves may already be just outside the starter
            // throat. Treat that as an already-completed egress rather than
            // pulling them back into a new exit transaction.
            if(!in_depot_band || robot.position.x>=starter_stash_center_.x+3.55f){
                robot.starter_exit_completed=true;
                return false;
            }
            robot.starter_exit_pending=true;
        }
        if(robot.starter_exit_pending && robot.position.x>=starter_stash_center_.x+4.30f){
            robot.starter_exit_pending=false;
            robot.starter_exit_completed=true;
            return false;
        }
        return robot.starter_exit_pending;
    };
    auto starter_depot_exit=[&](const BuilderRobotState& robot){
        Vec3 gate{starter_stash_center_.x+4.65f,0.0f,robot.position.z};
        gate.y=first_person_floor_height(gate);
        return gate;
    };

    auto physical_available=[&](MaterialResource material){
        std::int64_t amount=0;
        for(const auto& pile:ground_resource_piles_)
            if(pile.resource==material && pile.amount>0) amount+=pile.amount;
        for(const auto& structure:placed_resources_){
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            for(const auto& box:structure.stored_boxes)
                if(box.resource==material && box.amount>0) amount+=box.amount;
        }
        return amount;
    };

    auto source_reserved_not_loaded=[&](MaterialResource material,std::size_t exclude_robot){
        std::int64_t amount=0;
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(i==exclude_robot) continue;
            const auto& other=builder_robots_[i];
            if(!other.cargo_committed_to_build ||
               (other.mode!=BuilderRobotMode::Fetching && other.mode!=BuilderRobotMode::Delivering)) continue;
            // Match the site-demand ledger exactly. A BR-02 self-haul has no
            // time-based lease expiry, and a CR-01 reservation remains physical
            // stock ownership until the carrier reaches the explicit 12 s
            // cancellation condition. Using a shorter source-only timeout lets
            // a second rover reserve kilograms that the first one still owns.
            if(other.role==RobotRole::Carrier && other.cargo.empty() &&
               other.blocked_elapsed>11.5f && !other.route_active &&
               other.action==BuilderRobotAction::None) continue;
            const std::int64_t assigned=other.logistics_reservation.amount(material);
            const std::int64_t loaded=other.cargo.amount(material);
            amount+=std::max<std::int64_t>(0,assigned-loaded);
        }
        return amount;
    };

    auto trip_source_available=[&](MaterialResource material,std::size_t exclude_robot){
        // Reservations may queue behind one another for the same material. The
        // physical stock is still accounted exactly once; lane admission is a
        // separate movement concern handled at the depot staging gates.
        const std::int64_t reserved=source_reserved_not_loaded(material,exclude_robot);
        return std::max<std::int64_t>(0,physical_available(material)-reserved);
    };

    auto logistics_remaining=[&](const ConstructionTargetRef& target,bool all_remaining_steps,
                                  bool include_reservations,std::size_t exclude_robot){
        ResourceBundle none{};
        if(!target.valid()) return none;
        const auto* plan=construction_target_plan(target);
        if(!plan) return none;
        const auto completed=construction_target_step(target);
        ResourceBundle required=all_remaining_steps?plan->remaining(completed):plan->next_step_cost(completed);
        ResourceBundle accounted{};
        if(const auto* staged=construction_site_buffer(target)) accounted=*staged;
        if(include_reservations) add_bundle(accounted,construction_site_reserved(target,exclude_robot));
        return positive_difference(required,accounted);
    };

    auto reserve_trip=[&](std::size_t robot_index,const ConstructionTargetRef& target,bool all_remaining_steps){
        ResourceBundle assignment{};
        if(robot_index>=builder_robots_.size() || !target.valid()) return assignment;
        auto& robot=builder_robots_[robot_index];

        // CR-01 is the primary shipper. If it becomes available while a BR-02
        // has only RESERVED a fallback load (nothing picked up yet), transfer
        // ownership back to the carrier before calculating demand. This avoids
        // two robots reserving different chunks of the same pallet and then
        // deadlocking the single-lane starter-depot queue.
        if(robot.role==RobotRole::Carrier){
            for(std::size_t bi=0;bi<builder_robots_.size();++bi){
                if(bi==robot_index) continue;
                auto& builder=builder_robots_[bi];
                if(builder.role!=RobotRole::Constructor || !builder.target.valid() || !builder.target.same_as(target)) continue;
                if(builder.mode!=BuilderRobotMode::Fetching || !builder.cargo.empty() || builder.logistics_reservation.empty()) continue;
                builder.logistics_reservation={};
                builder.cargo_committed_to_build=false;
                builder.mode=BuilderRobotMode::AwaitingMaterials;
                builder.idle_elapsed=0.0f;
                builder.pickup_lane_claimed=false;
                builder.pickup_queue_age=0.0f;
                clear_action(builder);
                clear_interaction_station(builder);
                clear_route(builder);
            }
        }

        const ResourceBundle need=logistics_remaining(target,all_remaining_steps,true,robot_index);
        std::int64_t capacity=std::max<std::int64_t>(0,robot.payload_capacity_kg-robot.cargo.total_amount());
        for(std::size_t i=0;i<kMaterialResourceCount && capacity>0;++i){
            const auto material=(MaterialResource)i;
            const std::int64_t wanted=need.amount(material);
            if(wanted<=0) continue;
            const std::int64_t available=trip_source_available(material,robot_index);
            const std::int64_t take=std::min({wanted,available,capacity});
            if(take<=0) continue;
            assignment.amount(material)=take;
            capacity-=take;
            // Keep each physical trip homogeneous. Besides matching the CR-01
            // pallet deck, this avoids a robot having to weave through several
            // neighbouring depot stacks before it can deliver anything. Other
            // fleet members immediately see the reservation and can claim a
            // different missing material in parallel.
            break;
        }
        robot.logistics_reservation=assignment;
        robot.cargo_committed_to_build=!assignment.empty() || !robot.cargo.empty();
        robot.interaction_position_valid=false;
        return assignment;
    };

    auto reservation_pickup_remaining=[&](const BuilderRobotState& robot){
        return positive_difference(robot.logistics_reservation,robot.cargo);
    };

    auto clear_logistics_assignment=[&](BuilderRobotState& robot){
        robot.logistics_reservation={};
        robot.interaction_position_valid=false;
        if(robot.cargo.empty()) robot.cargo_committed_to_build=false;
    };

    auto deliver_reserved_cargo=[&](BuilderRobotState& robot,const ConstructionTargetRef& target,bool all_remaining_steps){
        ResourceBundle delivered{};
        ResourceBundle raw_need=logistics_remaining(target,all_remaining_steps,false,kNoRobotIndex);
        for(std::size_t i=0;i<kMaterialResourceCount;++i){
            const auto material=(MaterialResource)i;
            const std::int64_t amount=std::min({robot.cargo.amount(material),
                                                robot.logistics_reservation.amount(material),
                                                raw_need.amount(material)});
            if(amount<=0) continue;
            delivered.amount(material)=amount;
            robot.cargo.amount(material)-=amount;
            robot.logistics_reservation.amount(material)-=amount;
            raw_need.amount(material)-=amount;
        }
        if(auto* shared=site_buffer_for(target)) add_bundle(*shared,delivered);
        if(robot.logistics_reservation.empty() && robot.cargo.empty()) robot.cargo_committed_to_build=false;
        return delivered;
    };

    auto reserve_loaded_cargo_for_site=[&](BuilderRobotState& robot,const ConstructionTargetRef& target,bool all_remaining_steps){
        ResourceBundle assignment{};
        ResourceBundle need=logistics_remaining(target,all_remaining_steps,true,kNoRobotIndex);
        for(std::size_t i=0;i<kMaterialResourceCount;++i){
            const auto material=(MaterialResource)i;
            const std::int64_t amount=std::min(robot.cargo.amount(material),need.amount(material));
            if(amount>0) assignment.amount(material)=amount;
        }
        if(assignment.empty()) return false;
        robot.logistics_reservation=assignment;
        robot.cargo_committed_to_build=true;
        return true;
    };

    auto find_storage_with_space=[&](std::int64_t amount,Vec3 from){
        std::size_t best=kNoRobotIndex;
        float best_d=std::numeric_limits<float>::max();
        for(std::size_t i=0;i<placed_resources_.size();++i){
            auto& structure=placed_resources_[i];
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            sync_stored_resource_totals(structure);
            const std::int64_t free=storage_dome_capacity_kg(structure.storage_size)-structure.stored_resources.total_amount();
            if(free<=0 || (amount>0 && free<std::min<std::int64_t>(amount,1))) continue;
            const float d=dist_xz(from,storage_service(i,kNoRobotIndex));
            if(d<best_d){ best_d=d; best=i; }
        }
        return best;
    };

    auto store_carrier_cargo=[&](BuilderRobotState& carrier,std::size_t dome_index){
        if(dome_index>=placed_resources_.size() || carrier.cargo.empty()) return false;
        auto& dome=placed_resources_[dome_index];
        if(dome.kind!=ResourceKind::StorageDome || !dome.construction.complete()) return false;
        sync_stored_resource_totals(dome);
        std::int64_t free=storage_dome_capacity_kg(dome.storage_size)-dome.stored_resources.total_amount();
        if(free<=0) return false;
        ResourceBundle returned{};
        for(std::size_t i=0;i<kMaterialResourceCount && free>0;++i){
            const auto r=(MaterialResource)i;
            std::int64_t remaining=carrier.cargo.amount(r);
            while(remaining>0 && free>0){
                const std::int64_t amount=std::min({remaining,free,resource_package_capacity_kg(r)});
                dome.stored_boxes.push_back({r,amount,1.0f});
                carrier.cargo.amount(r)-=amount;
                remaining-=amount;
                free-=amount;
                returned.amount(r)+=amount;
            }
        }
        sync_stored_resource_totals(dome);
        if(carrier.cargo_committed_to_build && !returned.empty()) resources_.add(returned);
        if(carrier.cargo.empty()) carrier.cargo_committed_to_build=false;
        return !returned.empty();
    };

    auto load_free_ground=[&](BuilderRobotState& carrier,MaterialResource material,Vec3 focus){
        std::size_t best=kNoRobotIndex;
        float best_d=0.65f;
        for(std::size_t i=0;i<ground_resource_piles_.size();++i){
            const auto& pile=ground_resource_piles_[i];
            if(pile.resource!=material || pile.amount<=0) continue;
            const float d=dist_xz(pile.position,focus);
            if(d<best_d){ best_d=d; best=i; }
        }
        if(best==kNoRobotIndex) return false;
        auto& pile=ground_resource_piles_[best];
        const auto take=std::min<std::int64_t>(free_capacity(carrier),pile.amount);
        if(take<=0) return false;
        carrier.cargo.amount(material)+=take;
        carrier.cargo_committed_to_build=false;
        pile.amount-=take;
        if(pile.amount<=0) ground_resource_piles_.erase(ground_resource_piles_.begin()+(std::ptrdiff_t)best);
        else pile.radius=resource_package_half_width(pile.resource,pile.amount);
        return true;
    };


    // Constructors own build targets and build-site buffers. They wait for a
    // carrier first, but retain the ability to self-haul if no carrier can
    // satisfy the request so construction can never deadlock.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.role!=RobotRole::Constructor) continue;
        // Safety evacuation has exclusive movement control, including for the
        // coordinating constructor itself. Previously a pending coordinator
        // could be moved by the pre-pass and then immediately braked by its
        // normal Building branch in the same tick, freezing the stage forever.
        if(safety_repositioning[robot_index]) continue;

        auto choose_target=[&](){
            ConstructionTargetRef best{};
            float best_score=std::numeric_limits<float>::max();
            auto consider=[&](ConstructionTargetKind kind,std::size_t index){
                ConstructionTargetRef candidate{kind,index};
                if(construction_target_complete(candidate) || !construction_target_dependencies_ready(candidate)) return;
                const std::size_t workers=constructor_team_count(candidate);
                const std::size_t capacity=constructor_team_capacity(candidate);
                if(workers>=capacity) return;
                const float distance=dist_xz(robot.position,construction_target_position(candidate));
                // Prefer joining a nearby active build crew before opening a new
                // site, but distribute excess BR-02s once a sensible crew size
                // has been reached. This gives visible cooperative construction
                // without sending the entire fleet to one tiny walkway segment.
                const float cooperation_bonus=workers>0?2.4f:0.0f;
                const float load_penalty=(float)workers*1.15f;
                // Always seed an untouched construction site before adding a
                // second helper to an already staffed site. This guarantees that
                // simultaneous blueprints actually progress in parallel instead
                // of the fleet dog-piling the nearest structure first.
                const float staffed_site_penalty=workers>0?10000.0f:0.0f;
                const float score=distance+load_penalty-cooperation_bonus+staffed_site_penalty;
                if(score<best_score){ best_score=score; best=candidate; }
            };
            for(std::size_t i=0;i<placed_habitats_.size();++i) consider(ConstructionTargetKind::Habitat,i);
            for(std::size_t i=0;i<placed_resources_.size();++i) consider(ConstructionTargetKind::Resource,i);
            for(std::size_t i=0;i<corridor_network_.connections().size();++i) consider(ConstructionTargetKind::Corridor,i);
            for(std::size_t i=0;i<utility_network_.connections().size();++i) consider(ConstructionTargetKind::Utility,i);
            for(std::size_t i=0;i<walkway_network_.connections().size();++i) consider(ConstructionTargetKind::Walkway,i);
            return best;
        };

        if(!robot.target.valid() || construction_target_complete(robot.target)){
            robot.target=choose_target();
            robot.service_slot_offset=0;
            if(robot.target.valid()){
                const std::size_t coordinator=site_owner_index(robot.target);
                robot.support_robot_index=coordinator<builder_robots_.size()?coordinator:robot_index;
                robot.site_buffer={};
            }else{
                robot.support_robot_index=kNoRobotIndex;
                robot.site_buffer={};
            }
            robot.cargo={};
            robot.logistics_reservation={};
            robot.cargo_committed_to_build=false;
            robot.work_elapsed=0.0f;
            robot.idle_elapsed=0.0f;
            robot.stage_commit_pending=false;
            robot.stage_commit_safe_position={};
            robot.stage_materials_picked=false;
            robot.build_orbit_phase=0.0f;
            robot.build_side_corner_reached=false;
            clear_action(robot);
            clear_route(robot);
            robot.mode=robot.target.valid()?BuilderRobotMode::AwaitingMaterials:BuilderRobotMode::Idle;
        }
        if(!robot.target.valid()){
            robot.idle_elapsed+=dt;
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            continue;
        }

        const auto* plan=construction_target_plan(robot.target);
        if(!plan){ robot.target={}; robot.support_robot_index=kNoRobotIndex; robot.logistics_reservation={}; robot.stage_commit_pending=false; robot.stage_commit_safe_position={}; robot.mode=BuilderRobotMode::Idle; continue; }
        const auto step=construction_target_step(robot.target);
        const ResourceBundle step_cost=plan->next_step_cost(step);
        ResourceBundle* shared_site_buffer=site_buffer_for(robot.target);
        if(!shared_site_buffer){ robot.target={}; robot.mode=BuilderRobotMode::Idle; continue; }

        auto publish_stage_progress=[&](float fraction){
            const float elapsed=std::clamp(fraction,0.0f,0.999f)*kConstructionStepSeconds;
            auto publish=[&](auto& state){
                if(!state.complete()) state.step_elapsed=std::max(state.step_elapsed,elapsed);
            };
            switch(robot.target.kind){
                case ConstructionTargetKind::Habitat:
                    if(robot.target.index<placed_habitats_.size()) publish(placed_habitats_[robot.target.index].construction);
                    break;
                case ConstructionTargetKind::Resource:
                    if(robot.target.index<placed_resources_.size()) publish(placed_resources_[robot.target.index].construction);
                    break;
                case ConstructionTargetKind::Corridor:
                    if(robot.target.index<corridor_network_.mutable_connections().size()) publish(corridor_network_.mutable_connections()[robot.target.index].construction);
                    break;
                case ConstructionTargetKind::Utility:
                    if(robot.target.index<utility_network_.mutable_connections().size()) publish(utility_network_.mutable_connections()[robot.target.index].construction);
                    break;
                case ConstructionTargetKind::Walkway:
                    if(robot.target.index<walkway_network_.mutable_connections().size()) publish(walkway_network_.mutable_connections()[robot.target.index].construction);
                    break;
                case ConstructionTargetKind::None: break;
            }
        };

        const std::size_t owner_index=site_owner_index(robot.target);
        if(owner_index>=builder_robots_.size()){ robot.target={}; robot.mode=BuilderRobotMode::Idle; continue; }
        auto& site_owner=builder_robots_[owner_index];
        const std::size_t team_count=std::max<std::size_t>(1,constructor_team_count(robot.target));
        const float team_work_factor=std::min(2.35f,1.0f+0.45f*(float)(team_count-1));
        Vec3 build_center=construction_target_position(robot.target);
        switch(robot.target.kind){
            case ConstructionTargetKind::Habitat: if(robot.target.index<placed_habitats_.size()) build_center=placed_habitats_[robot.target.index].position; break;
            case ConstructionTargetKind::Resource: if(robot.target.index<placed_resources_.size()) build_center=placed_resources_[robot.target.index].position; break;
            default: break;
        }

        const Vec3 drop_zone=construction_drop_position(robot.target);

        // Delivered material belongs to the blueprint itself. BR-02 no longer
        // performs a second trip to the external carrier handoff for every
        // construction stage: once the next stage bundle is present in the
        // blueprint buffer, the constructor can proceed directly to a safe work
        // position. This removes a repeated route around growing geometry that
        // could become unreachable after walls/foundations appeared.
        if(robot.mode==BuilderRobotMode::Idle) robot.mode=BuilderRobotMode::AwaitingMaterials;

        if(robot.mode==BuilderRobotMode::AwaitingMaterials){
            if(contains_bundle(*shared_site_buffer,step_cost)){
                clear_logistics_assignment(robot);
                robot.idle_elapsed=0.0f;
                site_owner.stage_materials_picked=true;
                clear_action(robot);
                clear_interaction_station(robot);
                robot.mode=BuilderRobotMode::Building;
                robot.work_elapsed=0.0f;
            }else{
                robot.idle_elapsed+=dt;

                // A builder waiting for the rest of a stage may never park on
                // the shared drop zone. Move it back to its first perimeter
                // work sector so CR-01s can keep unloading without a stationary
                // BR-02 becoming a permanent throat obstacle.
                const Vec3 wait_position=construction_wait_point(robot_index,robot.target);
                if(dist_xz(robot.position,drop_zone)<1.55f || dist_xz(robot.position,wait_position)>kConstructionBayArrivalM){
                    clear_action(robot);
                    move_towards(robot_index,wait_position,0.16f,0.72f);
                }else{
                    clear_route(robot);
                    robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                }

                // Self-haul covers ONLY the part of the current stage that is
                // neither already at the drop zone nor reserved by a carrier.
                // This makes the BR-02 a real fallback collector even while
                // carriers are active, without duplicating a shipment. One
                // constructor per site may claim this fallback trip at a time.
                const ResourceBundle unassigned=construction_site_remaining(robot.target,true);
                bool other_builder_hauling=false;
                std::size_t fallback_builder=kNoRobotIndex;
                for(std::size_t bi=0;bi<builder_robots_.size();++bi){
                    const auto& b=builder_robots_[bi];
                    if(b.role!=RobotRole::Constructor || !b.target.valid() || !b.target.same_as(robot.target)) continue;
                    if(fallback_builder==kNoRobotIndex) fallback_builder=bi;
                    if(bi!=robot_index && (b.mode==BuilderRobotMode::Fetching || b.mode==BuilderRobotMode::Delivering))
                        other_builder_hauling=true;
                }
                bool carrier_available_for_site=false;
                std::size_t available_carriers=0;
                for(const auto& fleet_robot:builder_robots_){
                    if(fleet_robot.role!=RobotRole::Carrier) continue;
                    ++available_carriers;
                    const bool already_servicing=fleet_robot.target.valid() && fleet_robot.target.same_as(robot.target) &&
                        (fleet_robot.mode==BuilderRobotMode::Fetching || fleet_robot.mode==BuilderRobotMode::Delivering);
                    const bool ready_to_dispatch=fleet_robot.mode==BuilderRobotMode::Idle && fleet_robot.cargo.empty();
                    if(already_servicing || ready_to_dispatch) carrier_available_for_site=true;
                }
                const bool allow_self_haul=!other_builder_hauling && fallback_builder==robot_index && !carrier_available_for_site;
                // BR-02 remains an immediate fallback when every CR-01 is busy,
                // but an idle/active carrier owns normal shipping. This keeps the
                // constructor at the build site instead of racing logistics to a
                // shared pallet for a tiny remainder of the same material.
                const float fallback_delay=available_carriers>0?0.12f:0.0f;
                if(allow_self_haul && !unassigned.empty() && robot.idle_elapsed>=fallback_delay){
                    if(!reserve_trip(robot_index,robot.target,false).empty()){
                        robot.mode=BuilderRobotMode::Fetching;
                        robot.idle_elapsed=0.0f;
                        clear_interaction_station(robot);
                    }
                }
            }
        }

        if(robot.mode==BuilderRobotMode::Fetching){
            // Save-compatible deadlock repair: older/runtime states may already
            // contain a BR-02 fallback reservation and a CR-01 reservation for
            // the same site. If the builder has not physically picked anything
            // up yet, release its reservation immediately and let logistics own
            // the shipment. This directly repairs 0-40-10.gmsave on load.
            if(robot.cargo.empty() && !robot.logistics_reservation.empty()){
                bool carrier_servicing_site=false;
                for(const auto& fleet_robot:builder_robots_){
                    if(fleet_robot.role!=RobotRole::Carrier || !fleet_robot.target.valid() || !fleet_robot.target.same_as(robot.target)) continue;
                    if((fleet_robot.mode==BuilderRobotMode::Fetching || fleet_robot.mode==BuilderRobotMode::Delivering) &&
                       (!fleet_robot.logistics_reservation.empty() || !fleet_robot.cargo.empty())){
                        carrier_servicing_site=true;
                        break;
                    }
                }
                if(carrier_servicing_site){
                    clear_logistics_assignment(robot);
                    clear_action(robot);
                    clear_interaction_station(robot);
                    clear_route(robot);
                    robot.pickup_lane_claimed=false;
                    robot.pickup_queue_age=0.0f;
                    robot.mode=BuilderRobotMode::AwaitingMaterials;
                    robot.idle_elapsed=0.0f;
                    continue;
                }
            }
            if(robot.logistics_reservation.empty()){
                if(reserve_trip(robot_index,robot.target,false).empty()){
                    robot.mode=BuilderRobotMode::AwaitingMaterials;
                    robot.idle_elapsed=0.0f;
                    continue;
                }
            }
            ResourceBundle self_need=reservation_pickup_remaining(robot);
            if(self_need.empty() || free_capacity(robot)<=0){
                clear_action(robot);
                robot.mode=BuilderRobotMode::Delivering;
            }else{
                if(!acquire_source_station(robot_index,self_need)){
                    clear_action(robot);
                    clear_interaction_station(robot);
                    // Release any kilograms that could not actually be sourced.
                    for(std::size_t i=0;i<kMaterialResourceCount;++i){
                        const auto material=(MaterialResource)i;
                        robot.logistics_reservation.amount(material)=std::min(robot.logistics_reservation.amount(material),robot.cargo.amount(material));
                    }
                    if(!robot.cargo.empty()) robot.mode=BuilderRobotMode::Delivering;
                    else{ clear_logistics_assignment(robot); robot.mode=BuilderRobotMode::AwaitingMaterials; }
                }else{
                    bool source_motion=false;
                    if(starter_pallet_focus(robot.interaction_focus_position)){
                        robot.pickup_queue_age=std::min(300.0f,robot.pickup_queue_age+dt);
                        const bool inside_lane=starter_lane_inside(robot.interaction_focus_position,robot.position);
                        const bool depot_admitted=inside_lane || robot.pickup_lane_claimed || starter_depot_admission_rank(robot_index)<3u;
                        if(!depot_admitted){
                            clear_action(robot); source_motion=true; clear_route(robot);
                            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                            robot.blocked_elapsed=0.0f; robot.watchdog_elapsed=0.0f;
                        }
                        const std::size_t rank=depot_admitted?starter_lane_queue_rank(robot_index,robot.interaction_material):1u;
                        const bool occupied=depot_admitted && starter_lane_occupied(robot_index,robot.interaction_material,robot.interaction_focus_position);
                        Vec3 queue_gate=starter_depot_outer_gate(robot.interaction_focus_position);
                        const std::size_t hold_rank=std::max<std::size_t>(rank,occupied?1u:0u);
                        queue_gate.x+=(float)hold_rank*3.25f;
                        const Vec3 outer_gate=starter_depot_outer_gate(robot.interaction_focus_position);
                        if(hold_rank==0 && (dist_xz(robot.position,outer_gate)<0.42f ||
                           starter_lane_inside(robot.interaction_focus_position,robot.position)))
                            robot.pickup_lane_claimed=true;
                        if(hold_rank>0){
                            clear_action(robot);
                            source_motion=true;
                            // Same-material followers remain in their existing
                            // hull-safe position until the current pallet user has
                            // loaded AND cleared the outbound lane. Moving them to
                            // a staging point while the leader is still inside the
                            // throat can block its only exit and deadlock the queue.
                            clear_route(robot);
                            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                            robot.blocked_elapsed=0.0f;
                            robot.watchdog_elapsed=0.0f;
                            robot.route_waiting_for_clearance=false;
                            robot.route_wait_elapsed=0.0f;
                        }else{
                            const Vec3 gate=starter_depot_entry(robot.interaction_focus_position);
                            // One-way depot phase progression. East of the
                            // inner gate we align on the outer spine and then
                            // enter the lane. Once the rover's centre has
                            // crossed the inner gate, never command it back to
                            // that gate: it can only continue to the pallet.
                            if(robot.position.x>gate.x+0.18f){
                                clear_action(robot); source_motion=true;
                                if(std::abs(robot.position.z-robot.interaction_focus_position.z)>0.24f ||
                                   robot.position.x>outer_gate.x+0.24f)
                                    move_towards(robot_index,outer_gate,0.16f,1.0f);
                                else
                                    move_towards(robot_index,gate,0.10f,0.92f);
                            }else if(dist_xz(robot.position,robot.interaction_position)>kPickupWorkZoneM){
                                clear_action(robot); source_motion=true;
                                move_towards(robot_index,robot.interaction_position,kPickupWorkZoneM*0.55f,0.92f);
                            }
                        }
                    }else if(dist_xz(robot.position,robot.interaction_position)>kPickupWorkZoneM){
                        clear_action(robot); source_motion=true;
                        move_towards(robot_index,robot.interaction_position,kPickupWorkZoneM*0.55f,0.92f);
                    }
                    if(!source_motion){
                        clear_route(robot);
                        robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                        if(robot.action!=BuilderRobotAction::Pickup){
                            const auto amount=std::min<std::int64_t>(free_capacity(robot),self_need.amount(robot.interaction_material));
                            start_action(robot,BuilderRobotAction::Pickup,kPickupSeconds,robot.interaction_material,amount,robot.interaction_focus_position);
                        }else if(tick_action(robot,dt)){
                            const bool loaded=robot_load_needed_resources(robot,self_need);
                            clear_action(robot);
                            clear_interaction_station(robot);
                            self_need=reservation_pickup_remaining(robot);
                            if(!loaded && robot.cargo.empty()){ clear_logistics_assignment(robot); robot.mode=BuilderRobotMode::AwaitingMaterials; }
                            else if(self_need.empty() || free_capacity(robot)<=0) robot.mode=BuilderRobotMode::Delivering;
                        }
                    }
                }
            }
        }

        if(robot.mode==BuilderRobotMode::Delivering){
            // BR-02 fallback hauling joins the SAME site drop queue as CR-01s.
            // A self-hauling builder may therefore never race a carrier into the
            // one unloading apron; it stages outside until it owns the bay.
            const std::size_t rank=drop_queue_rank(robot_index,robot.target);
            const bool may_enter_drop=rank==0;
            const Vec3 delivery_goal=may_enter_drop?drop_zone:
                construction_drop_staging_point(robot_index,robot.target,std::max<std::size_t>(1,rank));
            robot.interaction_focus_position=drop_zone;
            robot.interaction_position=delivery_goal;
            robot.interaction_position_valid=true;
            if(starter_depot_exit_needed(robot)){
                clear_action(robot);
                const Vec3 gate=starter_depot_exit(robot);
                move_towards(robot_index,gate,0.20f,0.90f);
            }else if(!may_enter_drop){
                clear_action(robot);
                if(dist_xz(robot.position,delivery_goal)>0.24f)
                    move_towards(robot_index,delivery_goal,0.16f,0.80f);
                else{ clear_route(robot); robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt); }
            }else if(dist_xz(robot.position,drop_zone)>kDeliveryWorkZoneM){
                clear_action(robot);
                move_towards(robot_index,drop_zone,kDeliveryWorkZoneM*0.58f,0.88f);
            }else if(!robot.cargo.empty()){
                clear_route(robot);
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                if(robot.action!=BuilderRobotAction::PutDown){
                    MaterialResource handled=MaterialResource::Iron; std::int64_t handled_amount=0;
                    for(std::size_t i=0;i<kMaterialResourceCount;++i){ const auto r=(MaterialResource)i; if(robot.cargo.amount(r)>0){ handled=r; handled_amount=robot.cargo.amount(r); break; } }
                    start_action(robot,BuilderRobotAction::PutDown,kPutDownSeconds,handled,handled_amount,drop_zone);
                }else if(tick_action(robot,dt)){
                    deliver_reserved_cargo(robot,robot.target,false);
                    clear_action(robot);
                    clear_interaction_station(robot);
                    if(!robot.cargo.empty()) clear_logistics_assignment(robot);
                }
            }else if(!robot.logistics_reservation.empty()){
                robot.mode=BuilderRobotMode::Fetching;
            }else{
                clear_action(robot);
                clear_interaction_station(robot);
                robot.mode=BuilderRobotMode::AwaitingMaterials;
                robot.idle_elapsed=0.0f;
            }
        }

        if(robot.mode==BuilderRobotMode::Building){
            // A stage can only be worked after its complete material bundle is staged
            // inside the blueprint buffer.
            if(!contains_bundle(*shared_site_buffer,step_cost) || !site_owner.stage_materials_picked){
                clear_action(robot);
                clear_interaction_station(robot);
                robot.mode=BuilderRobotMode::AwaitingMaterials;
                continue;
            }
            robot.work_elapsed+=dt;

            if(robot.stage_commit_pending){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                const auto next_step=(std::uint8_t)std::min<int>((int)construction_target_step(robot.target)+1,(int)construction_target_total_steps(robot.target));
                if(!fleet_safe_for_next_stage(robot.target,next_step)) continue;
                if(advance_construction_target(robot.target,kConstructionStepSeconds)){
                    subtract_bundle(*shared_site_buffer,step_cost);
                    site_owner.stage_materials_picked=false;
                    // Every builder starts the next stage's work sequence from phase zero.
                    for(auto& teammate:builder_robots_){
                        if(teammate.role==RobotRole::Constructor && teammate.target.valid() && teammate.target.same_as(robot.target)){
                            teammate.build_orbit_phase=0.0f;
                            teammate.build_side_corner_reached=false;
                        }
                    }
                    robot.work_elapsed=0.0f;
                }
                robot.stage_commit_pending=false;
                robot.stage_commit_safe_position={};
                clear_action(robot);
                clear_interaction_station(robot);
                if(construction_target_complete(robot.target)){
                    robot.target={}; robot.support_robot_index=kNoRobotIndex; robot.site_buffer={}; robot.logistics_reservation={}; robot.cargo_committed_to_build=false; robot.stage_materials_picked=false; robot.mode=BuilderRobotMode::Idle;
                }else{
                    robot.mode=BuilderRobotMode::AwaitingMaterials;
                    robot.idle_elapsed=0.0f;
                }
                continue;
            }

            const Vec3 center=build_center;
            const int phase=std::clamp((int)std::floor(robot.build_orbit_phase),0,kBuildWorkSectorsPerStage-1);
            const int stage_side=(int)construction_target_step(robot.target)&3;
            const int work_sector=(stage_side+phase*2)&3;
            const Vec3 work_position=construction_work_point(robot_index,robot.target,work_sector);
            robot.interaction_focus_position=center;
            robot.interaction_position_valid=true;

            // The route planner itself finds the shortest collision-safe path
            // between the two required work positions. Do not force a corner
            // waypoint/full perimeter lap: that was realistic-looking but spent
            // more time driving than actually constructing.
            if(dist_xz(robot.interaction_position,work_position)>0.18f){
                robot.interaction_position=work_position;
                clear_route(robot);
            }
            if(dist_xz(robot.position,work_position)>kConstructionBayArrivalM){
                clear_action(robot);
                move_towards(robot_index,work_position,0.16f,kBuildTravelSpeedScale);
                continue;
            }
            clear_route(robot);
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);

            // Two opposite work positions per stage retain visible perimeter
            // construction while cutting the non-productive driving roughly in
            // half. Action time remains distributed across the required points.
            const float sector_factor=(float)kBuildWorkSectorsPerStage*team_work_factor;
            if(robot.action==BuilderRobotAction::None)
                start_action(robot,BuilderRobotAction::Welding,kWeldSeconds/sector_factor,MaterialResource::Iron,0,center);

            // Every physical welding/build-up operation contributes directly to
            // the structure's visible partial-step progress. With multiple BR-02s
            // the shared construction state keeps the furthest completed operation,
            // so progress is monotonic and never jumps backwards when a helper is
            // one sector behind.
            const float side_base=(float)phase;
            if(robot.action==BuilderRobotAction::Welding){
                const float action_fraction=robot.action_duration>0.0f?
                    std::clamp(robot.action_elapsed/robot.action_duration,0.0f,1.0f):0.0f;
                publish_stage_progress((side_base+0.5f*action_fraction)/(float)kBuildWorkSectorsPerStage);
            }else if(robot.action==BuilderRobotAction::BuildUp){
                const float action_fraction=robot.action_duration>0.0f?
                    std::clamp(robot.action_elapsed/robot.action_duration,0.0f,1.0f):0.0f;
                publish_stage_progress((side_base+0.5f+0.5f*action_fraction)/(float)kBuildWorkSectorsPerStage);
            }

            if(robot.action==BuilderRobotAction::Welding && tick_action(robot,dt)){
                publish_stage_progress((side_base+0.5f)/(float)kBuildWorkSectorsPerStage);
                start_action(robot,BuilderRobotAction::BuildUp,kBuildUpSeconds/sector_factor,MaterialResource::Iron,0,center);
            }else if(robot.action==BuilderRobotAction::BuildUp && tick_action(robot,dt)){
                publish_stage_progress(((float)phase+1.0f)/(float)kBuildWorkSectorsPerStage);
                clear_action(robot);
                clear_interaction_station(robot);
                robot.build_orbit_phase+=1.0f;
                robot.build_side_corner_reached=false;
                if(robot.build_orbit_phase<(float)kBuildWorkSectorsPerStage) continue;

                bool commit_claimed=false;
                for(std::size_t bi=0;bi<builder_robots_.size();++bi){
                    if(bi==robot_index) continue;
                    const auto& other=builder_robots_[bi];
                    if(other.role==RobotRole::Constructor && other.target.valid() &&
                       other.target.same_as(robot.target) && other.stage_commit_pending){
                        commit_claimed=true;
                        break;
                    }
                }
                if(!commit_claimed) robot.stage_commit_pending=true;
                else { robot.build_orbit_phase=0.0f; robot.build_side_corner_reached=false; }
            }
        }
    }

    // CR-01 logistics robots are demand-driven. Every trip owns an explicit
    // reservation in kilograms. Site demand is therefore always:
    // required - delivered - fleet reservations. This is the authoritative
    // synchronization point shared with fallback BR-02 hauling.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& carrier=builder_robots_[robot_index];
        if(carrier.role!=RobotRole::Carrier) continue;
        if(safety_repositioning[robot_index]) continue;

        // A reservation is a lease, not a permanent lock. If an empty CR-01
        // cannot make route progress for several seconds, release the trip so
        // another carrier or the site's builder can immediately take over.
        const bool protected_starter_pickup=carrier.mode==BuilderRobotMode::Fetching &&
            carrier.interaction_position_valid && starter_pallet_focus(carrier.interaction_focus_position) &&
            (carrier.pickup_lane_claimed || starter_lane_inside(carrier.interaction_focus_position,carrier.position) ||
             starter_lane_queue_rank(robot_index,carrier.interaction_material)>0 ||
             starter_depot_admission_rank(robot_index)>=3u);
        if(carrier.mode==BuilderRobotMode::Fetching && carrier.cargo.empty() &&
           !carrier.logistics_reservation.empty() && carrier.blocked_elapsed>12.0f &&
           !carrier.route_active && carrier.action==BuilderRobotAction::None && !protected_starter_pickup){
            carrier.logistics_reservation={};
            carrier.cargo_committed_to_build=false;
            carrier.target={};
            carrier.support_robot_index=kNoRobotIndex;
            carrier.mode=BuilderRobotMode::Idle;
            carrier.blocked_elapsed=0.0f;
            clear_action(carrier);
            clear_route(carrier);
        }

        auto valid_support=[&](){
            if(!carrier.target.valid() || construction_target_complete(carrier.target) ||
               !construction_target_dependencies_ready(carrier.target)) return false;
            // Logistics is blueprint-owned, not constructor-owned. A CR-01 may
            // therefore service an unfinished placed blueprint before any BR-02
            // has selected it. support_robot_index is only advisory when a
            // constructor coordinator exists.
            carrier.support_robot_index=construction_site_owner_index(carrier.target);
            return construction_site_buffer(carrier.target)!=nullptr;
        };

        auto site_serviceable=[&](const ResourceBundle& need,bool loaded_only){
            for(std::size_t i=0;i<kMaterialResourceCount;++i){
                const auto material=(MaterialResource)i;
                if(need.amount(material)<=0) continue;
                if(loaded_only){
                    if(carrier.cargo.amount(material)>0) return true;
                }else{
                    const std::int64_t available=trip_source_available(material,robot_index);
                    if(available>0) return true;
                }
            }
            return false;
        };

        auto choose_support=[&](bool loaded_only){
            ConstructionTargetRef best{};
            float best_score=std::numeric_limits<float>::max();

            auto consider_target=[&](const ConstructionTargetRef& target,float class_bias){
                if(!target.valid() || construction_target_complete(target) ||
                   !construction_target_dependencies_ready(target)) return;
                const ResourceBundle need=logistics_remaining(target,true,true,robot_index);
                if(need.empty() || !site_serviceable(need,loaded_only)) return;

                float score=dist_xz(carrier.position,construction_target_position(target))+class_bias;
                if(!loaded_only){
                    const SourceChoice source=choose_source(robot_index,need);
                    if(!source.valid) return;
                    score+=source.distance*0.65f;
                }
                std::size_t assigned_carriers=0;
                for(std::size_t ci=0;ci<builder_robots_.size();++ci){
                    if(ci==robot_index) continue;
                    const auto& other=builder_robots_[ci];
                    if(other.role==RobotRole::Carrier && other.target.valid() && other.target.same_as(target) &&
                       (other.mode==BuilderRobotMode::Fetching || other.mode==BuilderRobotMode::Delivering)) ++assigned_carriers;
                }
                // Seed every unfinished blueprint before assigning another CR-01
                // to one that is already being supplied.
                score+=(float)assigned_carriers*1000.0f;
                const std::size_t owner=construction_site_owner_index(target);
                if(owner<builder_robots_.size()) score-=std::min(6.0f,builder_robots_[owner].idle_elapsed*0.16f);
                if(score<best_score){ best_score=score; best=target; }
            };

            // Placed structures are primary logistics destinations. Linear
            // blueprints are serviced as well once their dependency gate allows
            // construction (e.g. a corridor waits for its endpoint structures).
            for(std::size_t i=0;i<placed_habitats_.size();++i)
                consider_target({ConstructionTargetKind::Habitat,i},0.0f);
            for(std::size_t i=0;i<placed_resources_.size();++i)
                consider_target({ConstructionTargetKind::Resource,i},0.0f);
            for(std::size_t i=0;i<corridor_network_.connections().size();++i)
                consider_target({ConstructionTargetKind::Corridor,i},25.0f);
            for(std::size_t i=0;i<utility_network_.connections().size();++i)
                consider_target({ConstructionTargetKind::Utility,i},25.0f);
            for(std::size_t i=0;i<walkway_network_.connections().size();++i)
                consider_target({ConstructionTargetKind::Walkway,i},25.0f);
            return best;
        };

        auto assign_empty_carrier=[&](){
            for(int attempt=0;attempt<2;++attempt){
                const ConstructionTargetRef target=choose_support(false);
                if(!target.valid()) return false;
                carrier.target=target;
                carrier.service_slot_offset=0;
                carrier.support_robot_index=construction_site_owner_index(target);
                carrier.logistics_reservation={};
                if(!reserve_trip(robot_index,target,true).empty()){
                    carrier.drop_exit_pending=false;
                    carrier.post_delivery_hold=false;
                    carrier.mode=BuilderRobotMode::Fetching;
                    carrier.cargo_committed_to_build=true;
                    clear_interaction_station(carrier);
                    clear_action(carrier);
                    clear_route(carrier);
                    return true;
                }
                carrier.target={};
                carrier.support_robot_index=kNoRobotIndex;
            }
            return false;
        };

        auto reassign_loaded_carrier=[&](){
            const ConstructionTargetRef target=choose_support(true);
            if(!target.valid()) return false;
            carrier.target=target;
            carrier.service_slot_offset=0;
            carrier.support_robot_index=construction_site_owner_index(target);
            carrier.logistics_reservation={};
            if(!reserve_loaded_cargo_for_site(carrier,carrier.target,true)){
                carrier.target={}; carrier.support_robot_index=kNoRobotIndex;
                return false;
            }
            carrier.drop_exit_pending=false;
            carrier.post_delivery_hold=false;
            carrier.mode=BuilderRobotMode::Delivering;
            clear_interaction_station(carrier);
            clear_action(carrier);
            clear_route(carrier);
            return true;
        };

        // Reservations are target-specific. If a site disappears or completes,
        // immediately release its uncollected kilograms so the rest of the fleet
        // sees them as available on this same simulation tick.
        if(!valid_support() && carrier.cargo_committed_to_build){
            carrier.logistics_reservation={};
            carrier.cargo_committed_to_build=!carrier.cargo.empty();
            carrier.target={};
            carrier.support_robot_index=kNoRobotIndex;
        }

        // Loaded construction material is never allowed to become invisible
        // inventory. Reassign it to another site that still needs that material;
        // only if none exists does it return to the Storage Dome.
        if(!carrier.cargo.empty() && (!valid_support() || carrier.logistics_reservation.empty())){
            if(!reassign_loaded_carrier()){
                carrier.logistics_reservation={};
                carrier.cargo_committed_to_build=false;
                carrier.target={};
                carrier.support_robot_index=kNoRobotIndex;
                carrier.mode=BuilderRobotMode::DepotTransfer;
            }
        }

        // Empty carriers continuously scan construction demand before doing any
        // depot housekeeping. This is what makes CR-01 visibly react as soon as
        // a blueprint creates a material deficit.
        if(carrier.cargo.empty() && carrier.logistics_reservation.empty() &&
           (carrier.mode==BuilderRobotMode::Idle || carrier.mode==BuilderRobotMode::DepotTransfer || !valid_support())){
            if(!assign_empty_carrier()){
                carrier.target={};
                carrier.support_robot_index=kNoRobotIndex;
                carrier.mode=BuilderRobotMode::DepotTransfer;
                carrier.cargo_committed_to_build=false;
                clear_route(carrier);
            }
        }

        if(carrier.mode==BuilderRobotMode::Fetching && valid_support()){
            ResourceBundle outstanding=reservation_pickup_remaining(carrier);
            if(outstanding.empty() || free_capacity(carrier)<=0){
                clear_action(carrier);
                carrier.mode=BuilderRobotMode::Delivering;
            }else{
                if(!acquire_source_station(robot_index,outstanding)){
                    clear_action(carrier);
                    clear_interaction_station(carrier);
                    // The physical source changed after dispatch. Shrink the
                    // reservation to what is actually on-board, releasing the
                    // rest immediately to the shared site ledger.
                    for(std::size_t i=0;i<kMaterialResourceCount;++i){
                        const auto material=(MaterialResource)i;
                        carrier.logistics_reservation.amount(material)=std::min(carrier.logistics_reservation.amount(material),carrier.cargo.amount(material));
                    }
                    if(carrier.cargo.empty()){
                        clear_logistics_assignment(carrier);
                        carrier.target={}; carrier.support_robot_index=kNoRobotIndex;
                        carrier.mode=BuilderRobotMode::Idle;
                    }else carrier.mode=BuilderRobotMode::Delivering;
                }else{
                    bool source_motion=false;
                    if(starter_pallet_focus(carrier.interaction_focus_position)){
                        carrier.pickup_queue_age=std::min(300.0f,carrier.pickup_queue_age+dt);
                        const bool inside_lane=starter_lane_inside(carrier.interaction_focus_position,carrier.position);
                        const bool depot_admitted=inside_lane || carrier.pickup_lane_claimed || starter_depot_admission_rank(robot_index)<3u;
                        if(!depot_admitted){
                            clear_action(carrier); source_motion=true; clear_route(carrier);
                            carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                            carrier.blocked_elapsed=0.0f; carrier.watchdog_elapsed=0.0f;
                        }
                        const std::size_t rank=depot_admitted?starter_lane_queue_rank(robot_index,carrier.interaction_material):1u;
                        const bool occupied=depot_admitted && starter_lane_occupied(robot_index,carrier.interaction_material,carrier.interaction_focus_position);
                        Vec3 queue_gate=starter_depot_outer_gate(carrier.interaction_focus_position);
                        const std::size_t hold_rank=std::max<std::size_t>(rank,occupied?1u:0u);
                        queue_gate.x+=(float)hold_rank*3.25f;
                        const Vec3 outer_gate=starter_depot_outer_gate(carrier.interaction_focus_position);
                        if(hold_rank==0 && (dist_xz(carrier.position,outer_gate)<0.42f ||
                           starter_lane_inside(carrier.interaction_focus_position,carrier.position)))
                            carrier.pickup_lane_claimed=true;
                        if(hold_rank>0){
                            clear_action(carrier);
                            source_motion=true;
                            // Same-material followers remain in their existing
                            // hull-safe position until the current pallet user has
                            // loaded AND cleared the outbound lane. Moving them to
                            // a staging point while the leader is still inside the
                            // throat can block its only exit and deadlock the queue.
                            clear_route(carrier);
                            carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                            carrier.blocked_elapsed=0.0f;
                            carrier.watchdog_elapsed=0.0f;
                            carrier.route_waiting_for_clearance=false;
                            carrier.route_wait_elapsed=0.0f;
                        }else{
                            const Vec3 gate=starter_depot_entry(carrier.interaction_focus_position);
                            if(carrier.position.x>gate.x+0.18f){
                                clear_action(carrier); source_motion=true;
                                if(std::abs(carrier.position.z-carrier.interaction_focus_position.z)>0.24f ||
                                   carrier.position.x>outer_gate.x+0.24f)
                                    move_towards(robot_index,outer_gate,0.16f,1.0f);
                                else
                                    move_towards(robot_index,gate,0.10f,0.94f);
                            }else if(dist_xz(carrier.position,carrier.interaction_position)>kPickupWorkZoneM){
                                clear_action(carrier); source_motion=true;
                                move_towards(robot_index,carrier.interaction_position,kPickupWorkZoneM*0.55f,0.95f);
                            }
                        }
                    }else if(dist_xz(carrier.position,carrier.interaction_position)>kPickupWorkZoneM){
                        clear_action(carrier); source_motion=true;
                        move_towards(robot_index,carrier.interaction_position,kPickupWorkZoneM*0.55f,0.95f);
                    }
                    if(!source_motion){
                        clear_route(carrier);
                        carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                        if(carrier.action!=BuilderRobotAction::Pickup){
                            const auto amount=std::min<std::int64_t>(free_capacity(carrier),outstanding.amount(carrier.interaction_material));
                            start_action(carrier,BuilderRobotAction::Pickup,kPickupSeconds*0.88f,carrier.interaction_material,amount,carrier.interaction_focus_position);
                        }else if(tick_action(carrier,dt)){
                            const bool loaded=robot_load_needed_resources(carrier,outstanding);
                            clear_action(carrier);
                            clear_interaction_station(carrier);
                            outstanding=reservation_pickup_remaining(carrier);
                            if(!loaded && carrier.cargo.empty()){ clear_logistics_assignment(carrier); carrier.target={}; carrier.support_robot_index=kNoRobotIndex; carrier.mode=BuilderRobotMode::Idle; }
                            else if(outstanding.empty() || free_capacity(carrier)<=0) carrier.mode=BuilderRobotMode::Delivering;
                        }
                    }
                }
            }
        }

        if(carrier.mode==BuilderRobotMode::Delivering){
            if(carrier.drop_exit_pending && carrier.target.valid()){
                // Unloading is a two-phase transaction: material transfer, then
                // physical departure from the shared drop apron. Keep the target
                // reference until the chassis is clear so nobody can reserve the
                // same bay underneath an "idle" rover.
                const Vec3 drop=construction_drop_position(carrier.target);
                const Vec3 departure=construction_drop_exit_point(robot_index,carrier.target);
                clear_action(carrier);
                carrier.interaction_focus_position=drop;
                carrier.interaction_position=departure;
                carrier.interaction_position_valid=true;
                if(dist_xz(carrier.position,departure)>0.28f){
                    move_towards(robot_index,departure,0.18f,0.88f);
                }else{
                    clear_route(carrier);
                    carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                    carrier.drop_exit_pending=false;
                    carrier.post_delivery_hold=true;
                    const ConstructionTargetRef serviced_target=carrier.target;
                    clear_interaction_station(carrier);
                    carrier.blocked_elapsed=0.0f;
                    carrier.watchdog_elapsed=0.0f;
                    carrier.recovery_attempts=0;

                    // Keep a CR-01 on the same active blueprint until the complete
                    // remaining bill of materials is either staged at the site or
                    // reserved by the rest of the fleet. This removes the idle
                    // gap between stage deliveries while preserving global site
                    // balancing once this target is fully covered.
                    carrier.logistics_reservation={};
                    carrier.cargo_committed_to_build=!carrier.cargo.empty();
                    if(carrier.cargo.empty() && serviced_target.valid() &&
                       !construction_target_complete(serviced_target) &&
                       !reserve_trip(robot_index,serviced_target,true).empty()){
                        carrier.target=serviced_target;
                        carrier.support_robot_index=construction_site_owner_index(serviced_target);
                        carrier.mode=BuilderRobotMode::Fetching;
                        carrier.cargo_committed_to_build=true;
                        carrier.post_delivery_hold=false;
                    }else{
                        carrier.target={};
                        carrier.support_robot_index=kNoRobotIndex;
                        carrier.mode=carrier.cargo.empty()?BuilderRobotMode::Idle:BuilderRobotMode::DepotTransfer;
                    }
                }
            }else if(!valid_support()){
                carrier.logistics_reservation={};
                carrier.cargo_committed_to_build=false;
                carrier.target={}; carrier.support_robot_index=kNoRobotIndex;
                carrier.mode=carrier.cargo.empty()?BuilderRobotMode::Idle:BuilderRobotMode::DepotTransfer;
            }else{
                const Vec3 drop=construction_drop_position(carrier.target);
                const std::size_t rank=drop_queue_rank(robot_index,carrier.target);
                bool builder_using_drop=false;
                for(std::size_t bi=0;bi<builder_robots_.size();++bi){
                    if(bi==robot_index) continue;
                    const auto& b=builder_robots_[bi];
                    if(b.role!=RobotRole::Constructor || !b.target.valid() || !b.target.same_as(carrier.target)) continue;
                    if(dist_xz(b.position,drop)<1.10f &&
                       (b.action==BuilderRobotAction::Pickup || b.mode==BuilderRobotMode::AwaitingMaterials)){
                        builder_using_drop=true;
                        break;
                    }
                }
                const bool may_enter_drop=(rank==0 && !builder_using_drop);
                carrier.interaction_focus_position=drop;
                carrier.interaction_position=may_enter_drop?drop:construction_drop_staging_point(robot_index,carrier.target,std::max<std::size_t>(1,rank));
                carrier.interaction_position_valid=true;
                const Vec3 goal=carrier.interaction_position;

                if(starter_depot_exit_needed(carrier)){
                    clear_action(carrier);
                    const Vec3 gate=starter_depot_exit(carrier);
                    move_towards(robot_index,gate,0.20f,0.95f);
                }else if(!may_enter_drop){
                    clear_action(carrier);
                    if(dist_xz(carrier.position,goal)>0.24f)
                        move_towards(robot_index,goal,0.16f,0.82f);
                    else { clear_route(carrier); carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt); }
                }else if(dist_xz(carrier.position,drop)>kDeliveryWorkZoneM){
                    clear_action(carrier);
                    move_towards(robot_index,drop,kDeliveryWorkZoneM*0.55f,0.92f);
                }else if(!carrier.cargo.empty()){
                    clear_route(carrier);
                    carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                    if(carrier.action!=BuilderRobotAction::PutDown){
                        MaterialResource handled=MaterialResource::Iron; std::int64_t amount=0;
                        for(std::size_t i=0;i<kMaterialResourceCount;++i){
                            const auto material=(MaterialResource)i;
                            if(carrier.cargo.amount(material)>0){ handled=material; amount=carrier.cargo.amount(material); break; }
                        }
                        start_action(carrier,BuilderRobotAction::PutDown,kPutDownSeconds*0.88f,handled,amount,drop);
                    }else if(tick_action(carrier,dt)){
                        deliver_reserved_cargo(carrier,carrier.target,true);
                        clear_action(carrier);
                        if(carrier.logistics_reservation.empty()){
                            // Do not become idle on the shared drop point. The
                            // following ticks drive this rover out of the apron
                            // before making the bay available to another trip.
                            carrier.drop_exit_pending=true;
                            carrier.blocked_elapsed=0.0f;
                            carrier.watchdog_elapsed=0.0f;
                            carrier.recovery_attempts=0;
                            clear_route(carrier);
                        }else{
                            clear_interaction_station(carrier);
                            carrier.mode=BuilderRobotMode::Fetching;
                        }
                    }
                }else if(!carrier.logistics_reservation.empty()){
                    clear_interaction_station(carrier);
                    carrier.mode=BuilderRobotMode::Fetching;
                }else{
                    carrier.cargo_committed_to_build=false;
                    carrier.target={}; carrier.support_robot_index=kNoRobotIndex;
                    carrier.mode=BuilderRobotMode::Idle;
                }
            }
        }

        if(carrier.mode==BuilderRobotMode::DepotTransfer){
            const std::size_t dome=find_storage_with_space(carrier.cargo.total_amount(),carrier.position);
            if(dome==kNoRobotIndex){
                carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                carrier.mode=BuilderRobotMode::Idle;
                continue;
            }
            if(carrier.cargo.empty()){
                SourceChoice pile{};
                for(std::size_t i=0;i<ground_resource_piles_.size();++i){
                    const auto& p=ground_resource_piles_[i];
                    if(p.amount<=0) continue;
                    const auto service=ground_service_point(robot_index,i);
                    if(!service.valid) continue;
                    if(service.distance<pile.distance) pile={true,true,i,0,p.resource,service.position,service.distance};
                }
                if(!pile.valid){ carrier.mode=BuilderRobotMode::Idle; continue; }
                if(pile.distance>kPickupWorkZoneM){
                    clear_action(carrier);
                    move_towards(robot_index,pile.position,kPickupWorkZoneM*0.55f,1.0f);
                }else{
                    carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                    if(carrier.action!=BuilderRobotAction::Pickup){
                        const auto amount=std::min<std::int64_t>(free_capacity(carrier),ground_resource_piles_[pile.index].amount);
                        start_action(carrier,BuilderRobotAction::Pickup,kPickupSeconds*0.88f,pile.material,amount,ground_resource_piles_[pile.index].position);
                    }else if(tick_action(carrier,dt)){
                        load_free_ground(carrier,carrier.action_material,carrier.action_focus_position);
                        clear_action(carrier);
                    }
                }
            }else{
                const Vec3 service=storage_service(dome,robot_index);
                if(dist_xz(carrier.position,service)>1.02f){
                    clear_action(carrier);
                    move_towards(robot_index,service,0.94f,1.0f);
                }else{
                    carrier.current_speed_mps=approach(carrier.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                    if(carrier.action!=BuilderRobotAction::PutDown){
                        MaterialResource handled=MaterialResource::Iron; std::int64_t amount=0;
                        for(std::size_t i=0;i<kMaterialResourceCount;++i){
                            const auto material=(MaterialResource)i;
                            if(carrier.cargo.amount(material)>0){ handled=material; amount=carrier.cargo.amount(material); break; }
                        }
                        start_action(carrier,BuilderRobotAction::PutDown,kPutDownSeconds*0.88f,handled,amount,service);
                    }else if(tick_action(carrier,dt)){
                        store_carrier_cargo(carrier,dome);
                        clear_action(carrier);
                        if(carrier.cargo.empty()) carrier.mode=BuilderRobotMode::Idle;
                    }
                }
            }
        }
    }

    // Idle CR-01s park in deterministic standby bays instead of freezing at
    // the location where their previous delivery happened. Parking at an
    // arbitrary construction approach can permanently block the BR-02's single
    // material-drop lane (the no-building.gmsave regression). Standby bays are
    // outside the starter pallet throat and separated by more than one hull.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.role!=RobotRole::Carrier || robot.mode!=BuilderRobotMode::Idle ||
           !robot.cargo.empty() || robot.target.valid()) continue;
        const Vec3 standby=carrier_standby_position(robot_index);
        robot.post_delivery_hold=false;
        if(dist_xz(robot.position,standby)>0.24f){
            clear_action(robot);
            clear_interaction_station(robot);
            move_towards(robot_index,standby,0.16f,0.74f);
        }else{
            clear_route(robot);
            clear_interaction_station(robot);
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            robot.post_delivery_hold=true;
        }
    }

    // Per-robot operation watchdog. A fleet member is never allowed to hold a
    // construction/logistics operation indefinitely just because its route or
    // interaction state stopped making progress. Recovery is strictly local:
    // the affected robot clears/replans its own work, while shared kilograms are
    // released back to the site ledger when appropriate.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.route_waiting_for_clearance){
            // Coordinated route waiting is intentional, not a deadlock. Keep
            // the operation watchdog disarmed until the reservation is clear.
            robot.watchdog_position=robot.position;
            robot.watchdog_mode=robot.mode;
            robot.watchdog_action=robot.action;
            robot.watchdog_elapsed=0.0f;
            continue;
        }
        const float moved=dist_xz(robot.position,robot.watchdog_position);
        const bool state_changed=robot.mode!=robot.watchdog_mode || robot.action!=robot.watchdog_action;
        const bool timed_action=robot.action!=BuilderRobotAction::None;
        if(state_changed || moved>0.10f || timed_action){
            robot.watchdog_position=robot.position;
            robot.watchdog_mode=robot.mode;
            robot.watchdog_action=robot.action;
            robot.watchdog_elapsed=0.0f;
            if(moved>0.18f || state_changed) robot.recovery_attempts=0;
            continue;
        }

        const bool awaiting_travel=robot.mode==BuilderRobotMode::AwaitingMaterials && robot.target.valid() &&
                                   dist_xz(robot.position,construction_service_point(robot_index,robot.target))>kBuildWorkZoneM;
        const bool should_progress=awaiting_travel || robot.mode==BuilderRobotMode::Fetching ||
                                   robot.mode==BuilderRobotMode::Delivering ||
                                   robot.mode==BuilderRobotMode::Building ||
                                   robot.mode==BuilderRobotMode::DepotTransfer;
        if(!should_progress){ robot.watchdog_elapsed=0.0f; continue; }

        // Waiting behind an older rover for the same starter-depot pallet is an
        // intentional queue state, not an operation failure. The generic stuck
        // watchdog must not clear the reservation/station here, otherwise queue
        // tickets are repeatedly lost and two robots can reacquire the same trip.
        const bool intentional_pickup_queue=robot.mode==BuilderRobotMode::Fetching &&
            robot.interaction_position_valid && starter_pallet_focus(robot.interaction_focus_position) &&
            (starter_depot_admission_rank(robot_index)>=3u ||
             starter_lane_queue_rank(robot_index,robot.interaction_material)>0);
        if(intentional_pickup_queue){
            robot.watchdog_elapsed=0.0f;
            robot.watchdog_position=robot.position;
            robot.watchdog_mode=robot.mode;
            robot.watchdog_action=robot.action;
            continue;
        }
        robot.watchdog_elapsed+=dt;
        if(robot.watchdog_elapsed<kOperationWatchdogSeconds) continue;

        const bool starter_pickup_transaction=robot.mode==BuilderRobotMode::Fetching &&
            robot.interaction_position_valid && starter_pallet_focus(robot.interaction_focus_position);
        if(starter_pickup_transaction){
            // A rover physically inside/queued for the pallet keeps ownership,
            // but the front rover must never hold a construction reservation
            // forever when even the mandatory full-resolution route recovery
            // cannot move it.  The intentional queue case already continued
            // above, so reaching this branch means this robot is the active
            // pickup transaction rather than a follower waiting its turn.
            if(robot.route_force_full_replan && robot.route_resolution_elapsed>=kOperationWatchdogSeconds){
                robot.logistics_reservation={};
                robot.cargo_committed_to_build=!robot.cargo.empty();
                robot.pickup_lane_claimed=false;
                clear_interaction_station(robot);
                clear_route(robot);
                robot.route_replan_cooldown=0.0f;
                robot.route_resolution_elapsed=0.0f;
                robot.route_force_full_replan=false;
                robot.blocked_elapsed=0.0f;
                clear_action(robot);
                if(robot.cargo.empty()){
                    robot.target={};
                    robot.support_robot_index=kNoRobotIndex;
                    robot.mode=robot.role==RobotRole::Carrier?BuilderRobotMode::Idle:BuilderRobotMode::AwaitingMaterials;
                    if(robot.role==RobotRole::Constructor) robot.idle_elapsed=kOperationWatchdogSeconds;
                }else{
                    robot.mode=BuilderRobotMode::Delivering;
                }
                continue;
            }
            // First recovery remains route-only. Preserve the pallet ticket and
            // exact reservation while the full-resolution planner gets its fair
            // chance to recover the transaction.
            robot.watchdog_elapsed=0.0f;
            robot.watchdog_position=robot.position;
            clear_route(robot);
            robot.route_replan_cooldown=0.0f;
            robot.blocked_elapsed=0.0f;
            clear_action(robot);
            continue;
        }

        ++robot.recovery_attempts;
        // Move this robot to a different pickup/work/delivery station on its
        // next attempt. This is local to the stuck unit and remains stable even
        // after it starts moving again. Prime-ish increments distribute a fleet
        // around the 24-slot rings without locking them into opposite pairs.
        robot.service_slot_offset=(std::uint8_t)((robot.service_slot_offset+5u)%24u);
        clear_interaction_station(robot);
        robot.watchdog_elapsed=0.0f;
        robot.watchdog_position=robot.position;
        clear_route(robot);
        robot.route_replan_cooldown=0.0f;
        robot.blocked_elapsed=0.0f;
        robot.avoidance_offset_degrees=0.0f;
        clear_action(robot);

        if(robot.mode==BuilderRobotMode::Fetching && robot.cargo.empty()){
            // An uncollected reservation must not freeze the site demand.
            robot.logistics_reservation={};
            robot.cargo_committed_to_build=false;
            if(robot.role==RobotRole::Carrier){
                robot.target={};
                robot.support_robot_index=kNoRobotIndex;
                robot.mode=BuilderRobotMode::Idle;
            }else{
                robot.mode=BuilderRobotMode::AwaitingMaterials;
                robot.idle_elapsed=kOperationWatchdogSeconds; // permits immediate self-haul retry
            }
        }else if(robot.mode==BuilderRobotMode::Delivering){
            // Keep loaded cargo authoritative, but force a fresh independent
            // approach route to the target on the next tick. A CR-01 that has
            // already unloaded is still performing its protected apron-exit
            // phase: watchdog recovery may replan that exit, never cancel it.
            if(robot.cargo.empty() && !(robot.role==RobotRole::Carrier && robot.drop_exit_pending))
                robot.mode=robot.role==RobotRole::Carrier?BuilderRobotMode::Idle:BuilderRobotMode::AwaitingMaterials;
        }else if(robot.mode==BuilderRobotMode::Building){
            // The service-slot offset above selects a different fixed work
            // station. Do not add any orbit/turn impulse: recovery is always a
            // fresh preplanned route to a deterministic station.
            robot.current_speed_mps=0.0f;
        }else if(robot.mode==BuilderRobotMode::DepotTransfer && robot.cargo.empty()){
            robot.mode=BuilderRobotMode::Idle;
        }

        // After repeated failures, release only uncollected logistics so another
        // robot can take over. Never discard physical cargo already on-board.
        if(robot.recovery_attempts>=2 && robot.cargo.empty() && robot.cargo_committed_to_build &&
           !(robot.role==RobotRole::Carrier && robot.drop_exit_pending)){
            robot.logistics_reservation={};
            robot.cargo_committed_to_build=false;
            if(robot.role==RobotRole::Carrier){ robot.target={}; robot.support_robot_index=kNoRobotIndex; robot.mode=BuilderRobotMode::Idle; }
        }
    }

    // Absolute fleet invariant: no authoritative robot hulls may overlap. The
    // normal movement solver prevents this before committing a step; this small
    // post-pass only repairs exceptional states such as legacy saves, safety
    // relocations or two robots that were already intersecting at frame start.
    for(int pass=0;pass<4;++pass){
        bool any_overlap=false;
        for(std::size_t a=0;a<builder_robots_.size();++a){
            for(std::size_t b=a+1;b<builder_robots_.size();++b){
                auto& ra=builder_robots_[a]; auto& rb=builder_robots_[b];
                if(!robot_hulls_overlap(ra.position,ra.heading_degrees,ra.role,rb.position,rb.heading_degrees,rb.role,kRobotHullClearanceM)) continue;
                any_overlap=true;
                // Emergency overlap repair may only move a chassis along
                // its own longitudinal axis. The previous center-to-center nudge
                // visibly slid robots sideways even though their wheels were
                // pointed elsewhere.
                auto longitudinal_nudge=[&](std::size_t index,BuilderRobotState& rover){
                    const float h=rover.heading_degrees*PI/180.0f;
                    const Vec3 axis{std::sin(h),0.0f,std::cos(h)};
                    constexpr float nudge=0.07f;
                    for(float sign: {1.0f,-1.0f}){
                        Vec3 candidate=rover.position+axis*(nudge*sign);
                        candidate.y=first_person_floor_height(candidate);
                        if(robot_position_clear(index,candidate) &&
                           robot_structure_move_allowed(rover,rover.position,candidate) &&
                           fleet_hull_clear(index,candidate,rover.heading_degrees)){
                            rover.position=candidate;
                            rover.current_speed_mps=0.0f;
                            clear_route(rover);
                            return true;
                        }
                    }
                    return false;
                };
                if(longitudinal_nudge(b,rb)) continue;
                longitudinal_nudge(a,ra);
            }
        }
        if(!any_overlap) break;
    }

    // The navigator now advances the authoritative pose continuously, so a
    // second damped render spring would only add lag and can create visible
    // catch-up pulses. Render exactly the collision-validated pose. This also
    // guarantees that the picture on screen can never disagree with the fleet
    // separation solver about where a chassis actually is.
    for(auto& robot:builder_robots_){
        robot.visual_position=robot.position;
        robot.visual_heading_degrees=robot.heading_degrees;
        robot.visual_velocity={};
        robot.visual_initialized=true;
    }

}

void GameApp::spawn_produced_resource(MaterialResource resource,std::int64_t amount,Vec3 source_position){
    if(amount<=0) return;
    ResourceBundle add{}; add.amount(resource)=amount; resources_.add(add);

    // Aggregate output pallets near a deterministic facility-side logistics
    // point to avoid unbounded object counts while keeping production physical.
    const float slot=(float)((std::size_t)resource%4);
    const Vec3 output_position=source_position+Vec3{0.8f+slot*0.28f,0.0f,0.9f};
    for(auto& pile:ground_resource_piles_){
        if(pile.resource==resource && dist_xz(pile.position,output_position)<0.45f){
            pile.amount+=amount; return;
        }
    }
    GroundResourcePile pile;
    pile.resource=resource;
    pile.amount=amount;
    pile.position=output_position;
    pile.radius=0.50f;
    ground_resource_piles_.push_back(pile);
}

void GameApp::update_resource_economy(float dt){
    if(dt<=0.0f) return;
    // One in-game day is 240 real seconds. Production figures below are kg per
    // in-game day and represent a mature automated lunar pilot plant rather
    // than present-day terrestrial equipment.
    constexpr double day_seconds=240.0;
    std::array<double,kMaterialResourceCount> rate_per_day{};
    std::array<Vec3,kMaterialResourceCount> source{};
    std::array<bool,kMaterialResourceCount> have_source{};

    int solar=0,power_storage=0,ore_drillers=0,water_drillers=0,water_purifiers=0;
    Vec3 ore_source{},water_source{};
    for(const auto& item:placed_resources_){
        if(!item.construction.complete()) continue;
        switch(item.kind){
            case ResourceKind::SolarPanel: ++solar; break;
            case ResourceKind::PowerStorage: ++power_storage; break;
            case ResourceKind::OreDriller: ++ore_drillers; ore_source=item.position; break;
            case ResourceKind::WaterDriller: ++water_drillers; water_source=item.position; break;
            case ResourceKind::WaterPurifier: ++water_purifiers; water_source=item.position; break;
            default: break;
        }
    }
    const bool stable_power=solar>=2 && power_storage>=1;
    bool processing=false,factory=false,manufactory=false,lab=false,sick=false;
    Vec3 industrial{},food_source{},medical_source{};
    float bio_capacity=0.0f;
    for(const auto& h:placed_habitats_){
        if(!h.construction.complete()) continue;
        switch(h.kind){
            case HabitatKind::ProcessingFacility: processing=true; industrial=h.position; break;
            case HabitatKind::Factory: factory=true; industrial=h.position; break;
            case HabitatKind::Manufactory: manufactory=true; industrial=h.position; break;
            case HabitatKind::Laboratory: lab=true; medical_source=h.position; break;
            case HabitatKind::SickBay: sick=true; medical_source=h.position; break;
            case HabitatKind::BioDome: bio_capacity+=size_multiplier(h.size); food_source=h.position; break;
            default: break;
        }
    }

    if(stable_power && processing && ore_drillers>0){
        rate_per_day[(size_t)MaterialResource::Iron]=11000.0*ore_drillers;
        rate_per_day[(size_t)MaterialResource::Copper]=1200.0*ore_drillers;
        rate_per_day[(size_t)MaterialResource::Silicon]=2100.0*ore_drillers;
        source[(size_t)MaterialResource::Iron]=source[(size_t)MaterialResource::Copper]=source[(size_t)MaterialResource::Silicon]=industrial;
        have_source[(size_t)MaterialResource::Iron]=have_source[(size_t)MaterialResource::Copper]=have_source[(size_t)MaterialResource::Silicon]=true;
    }
    if(stable_power && processing && factory && manufactory){
        rate_per_day[(size_t)MaterialResource::Plastic]=1800.0;
        source[(size_t)MaterialResource::Plastic]=industrial; have_source[(size_t)MaterialResource::Plastic]=true;
    }
    if(stable_power && water_drillers>0 && water_purifiers>0){
        rate_per_day[(size_t)MaterialResource::Water]=5200.0*std::min(water_drillers,water_purifiers);
        source[(size_t)MaterialResource::Water]=water_source; have_source[(size_t)MaterialResource::Water]=true;
    }
    if(stable_power && water_purifiers>0 && bio_capacity>0.0f){
        rate_per_day[(size_t)MaterialResource::Food]=420.0*(double)bio_capacity;
        source[(size_t)MaterialResource::Food]=food_source; have_source[(size_t)MaterialResource::Food]=true;
    }
    if(stable_power && lab && sick && factory){
        rate_per_day[(size_t)MaterialResource::Medical]=55.0;
        source[(size_t)MaterialResource::Medical]=medical_source; have_source[(size_t)MaterialResource::Medical]=true;
    }

    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        if(rate_per_day[i]<=0.0 || !have_source[i]) continue;
        production_fraction_[i]+=rate_per_day[i]*(double)dt/day_seconds;
        const auto whole=(std::int64_t)std::floor(production_fraction_[i]);
        if(whole>0){
            production_fraction_[i]-=(double)whole;
            spawn_produced_resource((MaterialResource)i,whole,source[i]);
        }
    }

    // Crew consumables (kg/day). Water is net make-up after high recovery;
    // food is edible mass; medical is a low continuous reserve draw.
    life_support_fraction_seconds_+=dt;
    if(life_support_fraction_seconds_>=1.0){
        const double seconds=life_support_fraction_seconds_;
        life_support_fraction_seconds_=0.0;
        const int crew=std::max(0,status_.crew_current());
        ResourceBundle use{};
        use.water=(std::int64_t)std::floor((double)crew*4.0*seconds/day_seconds);
        use.food=(std::int64_t)std::floor((double)crew*0.85*seconds/day_seconds);
        use.medical=(std::int64_t)std::floor((double)crew*0.03*seconds/day_seconds);
        // The per-second quantities are often fractional; accumulate them in
        // the same precision buckets used by production.
        const std::array<std::pair<MaterialResource,double>,3> demand{{
            {MaterialResource::Water,(double)crew*4.0*seconds/day_seconds},
            {MaterialResource::Food,(double)crew*0.85*seconds/day_seconds},
            {MaterialResource::Medical,(double)crew*0.03*seconds/day_seconds}}};
        for(const auto& [material,value]:demand){
            const std::size_t idx=(std::size_t)material;
            production_fraction_[idx]-=value;
            if(production_fraction_[idx]<=-1.0){
                const auto consume=(std::int64_t)std::floor(-production_fraction_[idx]);
                production_fraction_[idx]+=(double)consume;
                ResourceBundle cost{}; cost.amount(material)=consume;
                if(resources_.consume(cost,false)){
                    std::int64_t remaining=consume;
                    for(auto it=ground_resource_piles_.begin();it!=ground_resource_piles_.end() && remaining>0;){
                        if(it->resource!=material){ ++it; continue; }
                        const auto take=std::min<std::int64_t>(it->amount,remaining);
                        it->amount-=take; remaining-=take;
                        if(it->amount<=0) it=ground_resource_piles_.erase(it); else ++it;
                    }
                    if(remaining>0){ ResourceBundle stored{}; stored.amount(material)=remaining; consume_storage_inventory(stored); }
                }
            }
        }
        (void)use;
    }
}

} // namespace game
