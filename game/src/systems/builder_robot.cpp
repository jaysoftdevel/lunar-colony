#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/util/world_math.hpp"
#include "game/world/construction_sequence.hpp"
#include "game/robots/path_planner.hpp"
#include "game/robots/safety_coordinator.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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
constexpr float kRobotHullClearanceM=0.02f;
constexpr float kPickupSeconds=1.45f;
constexpr float kPutDownSeconds=1.35f;
constexpr float kApplySeconds=1.50f;
constexpr float kConstructionWorkSpeedMultiplier=1.50f;
constexpr float kWeldSeconds=2.20f/kConstructionWorkSpeedMultiplier;
constexpr float kBuildUpSeconds=1.80f/kConstructionWorkSpeedMultiplier;
// A stage no longer forces a full perimeter lap. One work position is used
// per stage and rotates between stages, keeping visible perimeter activity
// without making the constructor cross the growing structure mid-stage.
constexpr int kBuildWorkSectorsPerStage=1;
constexpr float kRobotAccelerationMps2=2.4f*kRobotMovementSpeedMultiplier;
constexpr float kRobotBrakingMps2=3.2f*kRobotMovementSpeedMultiplier;
constexpr float kRobotTurnRateDegreesPerSecond=125.0f*kRobotMovementSpeedMultiplier;
constexpr float kBuildTravelSpeedScale=1.00f;
constexpr float kPickupWorkZoneM=0.30f;
constexpr float kDeliveryWorkZoneM=0.42f;
constexpr float kBuildWorkZoneM=0.34f;
constexpr float kOperationWatchdogSeconds=3.25f;
constexpr float kOperationHardRecoverySeconds=6.50f;
constexpr float kStarterPalletSpacingM=2.05f;
constexpr float kStarterPalletHalfSpanM=((float)kMaterialResourceCount-1.0f)*0.5f*kStarterPalletSpacingM+0.45f;
constexpr float kStarterStashInsetZM=8.50f;

float wrap_radians(float a){
    while(a>PI) a-=2.0f*PI;
    while(a<-PI) a+=2.0f*PI;
    return a;
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
    // Work animations/actions may be selected while the rover is finishing its
    // certified stop, but physical work time advances only after the safety
    // authority has actually brought the chassis to rest.
    if(std::abs(robot.current_speed_mps)>0.045f) return false;
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

float robot_safety_corridor_radius(RobotRole role){
    // Gameplay traffic uses a compact corridor roughly 1.5x the visible rover
    // chassis width. Never shrink below the circumscribed physical hull: the
    // longer CR-01 must still be collision-safe while rotating. This removes
    // the old extra 0.10 m navigation halo without weakening the hard hull.
    const RobotFootprint footprint=robot_footprint(role);
    const float one_and_half_chassis_width_radius=footprint.half_width*1.5f;
    return std::max(robot_resource_clearance_radius(role),one_and_half_chassis_width_radius);
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

    // Ship enough construction stock to place one example of every currently
    // available structure kind at its largest gameplay size. This is more
    // useful than the old mix of STANDARD habitats and SMALL outdoor modules:
    // a fresh colony can now explore the complete build catalog without being
    // blocked simply because the initial manifest omitted the large variant's
    // structural material mix.
    for(std::size_t i=0;i<(std::size_t)HabitatKind::Count;++i){
        const auto kind=(HabitatKind)i;
        const HabitatSizeTier size=habitat_has_extended_size_tiers(kind)?
            HabitatSizeTier::Big:HabitatSizeTier::Standard;
        add_bundle(total,construction_costs::habitat(kind,size,habitat_models_.mesh(kind,size)).total());
    }

    for(std::size_t i=0;i<(std::size_t)ResourceKind::Count;++i){
        const auto kind=(ResourceKind)i;
        if(kind==ResourceKind::StorageDome){
            add_bundle(total,construction_costs::structure(
                kind,resource_models_.mesh(kind,StorageDomeSize::Huge)).total());
        }else{
            add_bundle(total,construction_costs::structure(
                kind,resource_models_.mesh(kind,ResourceSizeTier::Standard)).total());
        }
    }

    // A viable lunar starter base needs redundant power and extraction before
    // local production is dependable. Include one additional full-size solar
    // panel and ore driller rather than forcing the player to consume the only
    // example of those systems during initial experimentation.
    add_bundle(total,construction_costs::structure(
        ResourceKind::SolarPanel,resource_models_.mesh(ResourceKind::SolarPanel,ResourceSizeTier::Standard)).total());
    add_bundle(total,construction_costs::structure(
        ResourceKind::OreDriller,resource_models_.mesh(ResourceKind::OreDriller,ResourceSizeTier::Standard)).total());

    // Connection reserve sized for a complete first-base layout rather than a
    // minimal cluster. Routed costs still account for bends/junctions.
    add_bundle(total,construction_costs::corridor(240.0f).total());
    add_bundle(total,construction_costs::supply_channel(280.0f).total());
    add_bundle(total,construction_costs::walkway(160.0f).total());

    // 30% construction contingency covers non-straight routing, extra junctions
    // and a few placement experiments while preserving the per-material lunar
    // bill of materials.
    for(const auto r:kConstructionMaterialResources)
        total.amount(r)=(total.amount(r)*130+99)/100;

    // Consumable bootstrap reserve. Once Bio Dome + water chain + medical
    // production are online the economy replaces these continuously.
    total.water=22000;
    total.food=6000;
    total.medical=1200;
    return total;
}

void GameApp::reset_builder_robot_and_starter_stash(){
    robot_safety_time_seconds_=0.0f;
    production_fraction_.fill(0.0);
    life_support_fraction_seconds_=0.0;

    const float half=(float)playfield_.size()*0.5f;
    starter_stash_center_={-half+6.25f,0.0f,-half+kStarterStashInsetZM};

    const int constructor_count=std::clamp(starting_robot_count_,1,6);
    const int carrier_count=std::clamp(starting_carrier_count_,1,6);
    builder_robots_.assign((std::size_t)(constructor_count+carrier_count),BuilderRobotState{});

    // All shipped rovers start together in one marked parking apron directly
    // beside the starter resource stash, but outside the east-side pallet
    // traffic spine. They remain exactly in these bays until they receive an
    // actual task (or an explicit safety evacuation). On normal playfields a
    // 3 x 4 apron sits just north-east of the stash; the minimum 16 m field uses
    // a compact 2 x 6 east-side apron so no bay is clamped on top of another.
    auto spawn_position=[&](int ordinal){
        Vec3 p{};
        if(half>=12.0f){
            const Vec3 spawn_center=starter_stash_center_+Vec3{4.60f,0.0f,11.60f};
            const int column=ordinal%3;
            const int row=ordinal/3;
            // Contract-based navigation uses a rotation-independent safety
            // disc around the complete chassis. Leave enough clearance for a
            // rover to pivot in its bay without immediately reserving a
            // neighbour's braking envelope.
            p={spawn_center.x+((float)column-1.0f)*3.00f,
               0.0f,
               spawn_center.z+((float)row-1.5f)*3.00f};
        }else{
            const Vec3 spawn_center=starter_stash_center_+Vec3{7.00f,0.0f,0.0f};
            const int column=ordinal%2;
            const int row=ordinal/2;
            p={spawn_center.x+((float)column-0.5f)*2.40f,
               0.0f,
               spawn_center.z+((float)row-2.5f)*2.40f};
        }
        p.x=std::clamp(p.x,-half+1.05f,half-1.05f);
        p.z=std::clamp(p.z,-half+1.05f,half-1.05f);
        return p;
    };

    int spawn_ordinal=0;
    for(int i=0;i<constructor_count;++i,++spawn_ordinal){
        auto& robot=builder_robots_[(std::size_t)i];
        robot.role=RobotRole::Constructor;
        robot.position=spawn_position(spawn_ordinal);
        robot.heading_degrees=0.0f;
        robot.visual_position=robot.position;
        robot.visual_heading_degrees=robot.heading_degrees;
        robot.visual_initialized=true;
        robot.move_speed_mps=kBuilderRobotTransitSpeedMps;
        robot.payload_capacity_kg=2400;
    }
    for(int i=0;i<carrier_count;++i,++spawn_ordinal){
        auto& carrier=builder_robots_[(std::size_t)constructor_count+(std::size_t)i];
        carrier.role=RobotRole::Carrier;
        carrier.position=spawn_position(spawn_ordinal);
        carrier.heading_degrees=0.0f;
        carrier.visual_position=carrier.position;
        carrier.visual_heading_degrees=carrier.heading_degrees;
        carrier.visual_initialized=true;
        carrier.move_speed_mps=kCarrierRobotTransitSpeedMps;
        carrier.payload_capacity_kg=3600;
    }

    const ResourceBundle reserve=starter_resource_reserve();
    resources_.restore(reserve);
    ground_resource_piles_.clear();

    // Eight front-accessible aggregate pallet stacks, one per material. The
    // depot is inset far enough from the map corner to provide a full traffic
    // spine, and pallet lanes are separated by 2.05 m. That spacing is large
    // enough for a CR-01 to enter/leave one lane while another rover is stopped
    // in the neighbouring lane without their swept hulls blocking each other.
    const float pallet_spacing=kStarterPalletSpacingM;
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

    if(target.kind==ConstructionTargetKind::Corridor){
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

    if(target.kind==ConstructionTargetKind::Utility){
        if(target.index>=utility_network_.connections().size()) return false;
        const auto& connection=utility_network_.connections()[target.index];
        auto endpoint_ready=[&](const UtilityEndpoint& endpoint){
            switch(endpoint.kind){
                case UtilityEndpointKind::Habitat:
                    return endpoint.index<placed_habitats_.size() && placed_habitats_[endpoint.index].construction.complete();
                case UtilityEndpointKind::Resource:
                    return endpoint.index<placed_resources_.size() && placed_resources_[endpoint.index].construction.complete();
                case UtilityEndpointKind::Open:
                    return true;
            }
            return false;
        };
        return endpoint_ready(connection.endpoint_a) && endpoint_ready(connection.endpoint_b);
    }

    if(target.kind==ConstructionTargetKind::Walkway){
        if(target.index>=walkway_network_.connections().size()) return false;
        const auto& connection=walkway_network_.connections()[target.index];
        auto endpoint_ready=[&](const WalkwayEndpoint& endpoint){
            switch(endpoint.kind){
                case WalkwayEndpointKind::Habitat:
                    return endpoint.index<placed_habitats_.size() && placed_habitats_[endpoint.index].construction.complete();
                case WalkwayEndpointKind::Resource:
                    return endpoint.index<placed_resources_.size() && placed_resources_[endpoint.index].construction.complete();
                case WalkwayEndpointKind::Corridor:
                    return endpoint.index<corridor_network_.connections().size() && corridor_network_.connections()[endpoint.index].construction.complete();
                case WalkwayEndpointKind::Walkway:
                    return endpoint.index<walkway_network_.connections().size() &&
                           endpoint.index!=target.index && walkway_network_.connections()[endpoint.index].construction.complete();
                case WalkwayEndpointKind::Open:
                    return true;
            }
            return false;
        };
        return endpoint_ready(connection.endpoint_a) && endpoint_ready(connection.endpoint_b);
    }

    return true;
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
    // Delivery aprons are physical rover destinations. They must therefore be
    // reachable by the COMPLETE chassis, not merely by the rover centre. Two
    // regressions exposed the old fixed-point assumptions:
    //   * a Storage Dome apron at z=-31.45 was inside the map for the centre but
    //     outside it for the CR-01 hull;
    //   * a corridor apron beside that completed dome was inside the dome's
    //     full-hull exclusion envelope and triggered multi-second impossible A*.
    const float largest_hull=std::max(robot_resource_clearance_radius(RobotRole::Constructor),
                                      robot_resource_clearance_radius(RobotRole::Carrier));
    const float hull_margin=largest_hull+kRobotObstacleMargin+0.12f;
    const float service_half=std::max(1.0f,(float)playfield_.size()*0.5f-hull_margin);

    auto inside_service_bounds=[&](Vec3 p){
        return std::abs(p.x)<=service_half && std::abs(p.z)<=service_half;
    };
    auto clear_of_structure_footprints=[&](Vec3 p){
        const float clearance=largest_hull+kRobotObstacleMargin+0.20f;
        for(const auto& h:placed_habitats_){
            if(!h.construction.complete() && h.construction.progress()<=0.0001f) continue;
            const auto& mesh=habitat_models_.mesh(h);
            const Vec3 local=world_math::rotate_y(p-h.position,-h.rotation_degrees);
            if(std::abs(local.x)<mesh.footprint_x*0.5f+clearance &&
               std::abs(local.z)<mesh.footprint_z*0.5f+clearance) return false;
        }
        for(const auto& item:placed_resources_){
            if(!item.construction.complete() && item.construction.progress()<=0.0001f) continue;
            const auto& mesh=resource_models_.mesh(item);
            const Vec3 local=world_math::rotate_y(p-item.position,-item.rotation_degrees);
            if(std::abs(local.x)<mesh.footprint_x*0.5f+clearance &&
               std::abs(local.z)<mesh.footprint_z*0.5f+clearance) return false;
        }
        return true;
    };
    auto clear_of_existing_corridors=[&](Vec3 p){
        const float clearance=geometry::kCorridorHalfWidth+largest_hull+kRobotObstacleMargin+0.18f;
        for(std::size_t ci=0;ci<corridor_network_.connections().size();++ci){
            if(target.kind==ConstructionTargetKind::Corridor && target.index==ci) continue;
            const auto& c=corridor_network_.connections()[ci];
            if(!c.construction.complete() || c.path.size()<2) continue;
            for(std::size_t i=0;i+1<c.path.size();++i)
                if(world_math::point_segment_distance_xz(p,c.path[i],c.path[i+1])<clearance) return false;
        }
        return true;
    };
    auto clear_of_ground_packages=[&](Vec3 p){
        for(const auto& pile:ground_resource_piles_){
            if(pile.amount<=0) continue;
            if(dist_xz(p,pile.position)<largest_hull+pile.radius+kRobotObstacleMargin+0.16f) return false;
        }
        return true;
    };
    auto clear_of_staged_site_packages=[&](Vec3 p){
        auto buffer_clear=[&](const ConstructionTargetRef& staged_target,const ResourceBundle& buffer){
            if(buffer.empty()) return true;
            int visual_slot=0;
            constexpr int max_visual_packages=36;
            for(std::size_t mi=0;mi<kMaterialResourceCount && visual_slot<max_visual_packages;++mi){
                const auto material=(MaterialResource)mi;
                std::int64_t remaining=std::max<std::int64_t>(0,buffer.amount(material));
                const std::int64_t capacity=std::max<std::int64_t>(1,resource_package_capacity_kg(material));
                while(remaining>0 && visual_slot<max_visual_packages){
                    const std::int64_t package_amount=std::min(remaining,capacity);
                    const float package_radius=resource_package_half_width(material,package_amount)*1.41421356f;
                    const Vec3 package_position=construction_drop_package_position(staged_target,visual_slot);
                    if(dist_xz(p,package_position)<largest_hull+package_radius+kRobotObstacleMargin+0.16f) return false;
                    remaining-=package_amount;
                    ++visual_slot;
                }
            }
            return true;
        };
        for(std::size_t i=0;i<placed_habitats_.size();++i)
            if(!buffer_clear({ConstructionTargetKind::Habitat,i},placed_habitats_[i].site_buffer)) return false;
        for(std::size_t i=0;i<placed_resources_.size();++i)
            if(!buffer_clear({ConstructionTargetKind::Resource,i},placed_resources_[i].site_buffer)) return false;
        for(std::size_t i=0;i<corridor_network_.connections().size();++i)
            if(!buffer_clear({ConstructionTargetKind::Corridor,i},corridor_network_.connections()[i].site_buffer)) return false;
        for(std::size_t i=0;i<utility_network_.connections().size();++i)
            if(!buffer_clear({ConstructionTargetKind::Utility,i},utility_network_.connections()[i].site_buffer)) return false;
        for(std::size_t i=0;i<walkway_network_.connections().size();++i)
            if(!buffer_clear({ConstructionTargetKind::Walkway,i},walkway_network_.connections()[i].site_buffer)) return false;
        return true;
    };
    auto service_point_clear=[&](Vec3 p){
        return inside_service_bounds(p) && clear_of_structure_footprints(p) &&
               clear_of_existing_corridors(p) && clear_of_ground_packages(p) &&
               clear_of_staged_site_packages(p);
    };

    auto choose_structure_apron=[&](Vec3 center,float rotation,float footprint_x,float footprint_z){
        const float apron=std::max(1.55f,largest_hull+kRobotObstacleMargin+0.42f);
        const float hx=footprint_x*0.5f+apron;
        const float hz=footprint_z*0.5f+apron;
        // Include side-centre bays as well as corners. A structure near one map
        // edge may have no reachable depot-facing corner even though its inward
        // side has a wide, perfectly valid service apron.
        const std::array<Vec3,16> local_candidates{{
            {-hx,0.0f,-hz},{ hx,0.0f,-hz},{-hx,0.0f, hz},{ hx,0.0f, hz},
            {-hx,0.0f,0.0f},{ hx,0.0f,0.0f},{0.0f,0.0f,-hz},{0.0f,0.0f, hz},
            {-hx,0.0f,-hz*0.45f},{-hx,0.0f,hz*0.45f},
            { hx,0.0f,-hz*0.45f},{ hx,0.0f,hz*0.45f},
            {-hx*0.45f,0.0f,-hz},{hx*0.45f,0.0f,-hz},
            {-hx*0.45f,0.0f, hz},{hx*0.45f,0.0f, hz}
        }};
        Vec3 best{};
        float best_score=std::numeric_limits<float>::max();
        bool found=false;
        for(const Vec3 local:local_candidates){
            Vec3 candidate=center+world_math::rotate_y(local,rotation);
            if(!service_point_clear(candidate)) continue;
            candidate.y=first_person_floor_height(candidate);
            const float depot_distance=dist_xz(candidate,starter_stash_center_);
            const bool side=(std::abs(local.x)<0.01f || std::abs(local.z)<0.01f);
            const float score=depot_distance-(side?0.18f:0.0f);
            if(score<best_score){ best_score=score; best=candidate; found=true; }
        }
        if(found) return best;

        // Last-resort inward cardinal point. This should only be needed for a
        // very large structure with little perimeter room; keep the full chassis
        // inside the world even when no preferred candidate is available.
        Vec3 toward_center{-center.x,0.0f,-center.z};
        if(std::abs(toward_center.x)>=std::abs(toward_center.z))
            toward_center={toward_center.x>=0.0f?1.0f:-1.0f,0.0f,0.0f};
        else
            toward_center={0.0f,0.0f,toward_center.z>=0.0f?1.0f:-1.0f};
        const Vec3 local_dir=world_math::rotate_y(toward_center,-rotation);
        Vec3 local{};
        if(std::abs(local_dir.x)>=std::abs(local_dir.z)) local={local_dir.x>=0.0f?hx:-hx,0.0f,0.0f};
        else local={0.0f,0.0f,local_dir.z>=0.0f?hz:-hz};
        Vec3 fallback=center+world_math::rotate_y(local,rotation);
        fallback.x=std::clamp(fallback.x,-service_half,service_half);
        fallback.z=std::clamp(fallback.z,-service_half,service_half);
        fallback.y=first_person_floor_height(fallback);
        return fallback;
    };

    auto choose_linear_apron=[&](){
        const Vec3 center=construction_target_position(target);
        Vec3 best{};
        float best_score=std::numeric_limits<float>::max();
        bool found=false;

        // Linear projects are normally serviced from a perpendicular apron,
        // not from an arbitrary ring point. A ring candidate can accidentally
        // sit in the narrow throat between a corridor and its completed
        // endpoint structure; it is point-clear yet has no chassis-wide route
        // from the depot. Staying on the lateral side keeps CR-01 outside the
        // future tube while the visible packages remain inside the blueprint.
        const std::vector<Vec3>* path=nullptr;
        float project_half_width=0.80f;
        if(target.kind==ConstructionTargetKind::Corridor && target.index<corridor_network_.connections().size()){
            path=&corridor_network_.connections()[target.index].path;
            project_half_width=geometry::kCorridorHalfWidth;
        }else if(target.kind==ConstructionTargetKind::Utility && target.index<utility_network_.connections().size()){
            path=&utility_network_.connections()[target.index].path;
            project_half_width=geometry::kSupplyTrayWidth*0.5f+0.35f;
        }else if(target.kind==ConstructionTargetKind::Walkway && target.index<walkway_network_.connections().size()){
            path=&walkway_network_.connections()[target.index].path;
            project_half_width=geometry::kWalkwayHalfWidth;
        }
        if(path && path->size()>=2){
            Vec3 tangent{1.0f,0.0f,0.0f};
            float nearest=std::numeric_limits<float>::max();
            for(std::size_t i=0;i+1<path->size();++i){
                const Vec3 a=(*path)[i], b=(*path)[i+1];
                const Vec3 ab{b.x-a.x,0.0f,b.z-a.z};
                const float len2=ab.x*ab.x+ab.z*ab.z;
                if(len2<1e-6f) continue;
                const float t=std::clamp(((center.x-a.x)*ab.x+(center.z-a.z)*ab.z)/len2,0.0f,1.0f);
                const Vec3 q=a+ab*t;
                const float d=dist_xz(center,q);
                if(d<nearest){
                    nearest=d;
                    const float len=std::sqrt(len2);
                    tangent=ab*(1.0f/len);
                }
            }
            const Vec3 normal{-tangent.z,0.0f,tangent.x};
            const float base=project_half_width+largest_hull+kRobotObstacleMargin+0.72f;
            constexpr std::array<float,4> extra{{0.0f,0.8f,1.7f,2.8f}};
            constexpr std::array<float,5> along{{0.0f,1.15f,-1.15f,2.25f,-2.25f}};
            for(float add:extra){
                const float lateral=base+add;
                for(float side:{-1.0f,1.0f}){
                    for(float along_offset:along){
                        Vec3 candidate=center+normal*(side*lateral)+tangent*along_offset;
                        if(!service_point_clear(candidate)) continue;
                        candidate.y=first_person_floor_height(candidate);
                        // Strongly prefer the depot-facing side; small along
                        // offsets only avoid staged packages/local geometry.
                        const float score=dist_xz(candidate,starter_stash_center_)+add*0.35f+std::abs(along_offset)*0.08f;
                        if(score<best_score){ best_score=score; best=candidate; found=true; }
                    }
                }
                if(found) return best;
            }
        }

        // Fallback for pathological local geometry: broad exterior ring search.
        constexpr std::array<float,7> radii{{3.2f,4.2f,5.4f,6.8f,8.2f,9.6f,11.2f}};
        constexpr int directions=24;
        for(float radius:radii){
            for(int i=0;i<directions;++i){
                const float a=(2.0f*PI*(float)i)/(float)directions;
                Vec3 candidate=center+Vec3{std::cos(a)*radius,0.0f,std::sin(a)*radius};
                if(!service_point_clear(candidate)) continue;
                candidate.y=first_person_floor_height(candidate);
                const float score=dist_xz(candidate,starter_stash_center_)+radius*0.12f;
                if(score<best_score){ best_score=score; best=candidate; found=true; }
            }
            if(found) break;
        }
        if(found) return best;

        Vec3 fallback=center;
        fallback.x=std::clamp(fallback.x,-service_half,service_half);
        fallback.z=std::clamp(fallback.z,-service_half,service_half);
        fallback.y=first_person_floor_height(fallback);
        return fallback;
    };

    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            if(target.index<placed_habitats_.size()){
                const auto& h=placed_habitats_[target.index];
                const auto& m=habitat_models_.mesh(h);
                return choose_structure_apron(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z);
            }
            break;
        case ConstructionTargetKind::Resource:
            if(target.index<placed_resources_.size()){
                const auto& item=placed_resources_[target.index];
                const auto& m=resource_models_.mesh(item);
                return choose_structure_apron(item.position,item.rotation_degrees,m.footprint_x,m.footprint_z);
            }
            break;
        case ConstructionTargetKind::Corridor:
        case ConstructionTargetKind::Utility:
        case ConstructionTargetKind::Walkway:
            return choose_linear_apron();
        case ConstructionTargetKind::None: break;
    }
    Vec3 p=construction_target_position(target);
    p.x=std::clamp(p.x,-service_half,service_half);
    p.z=std::clamp(p.z,-service_half,service_half);
    p.y=first_person_floor_height(p);
    return p;
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
                                      std::abs(pile.position.z-starter_stash_center_.z)<kStarterPalletHalfSpanM;
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
    // after the structure/corridor it attaches to is physically complete. Any
    // ground object created by recovery is admitted through the same reservation
    // invariant as robot motion: it may not materialize inside an active normal
    // motion, braking branch, or terminal hold.
    auto scene_circle_clear_of_robot_contracts=[&](Vec3 center,float radius){
        for(const auto& robot:builder_robots_){
            const float robot_radius=robot_safety_corridor_radius(robot.role);
            if(robot.safety.active_contract.valid){
                if(robots::contract_intersects_region(robot.safety.active_contract,[&](Vec3 p,float reserved_radius){
                    return dist_xz(center,p)<=radius+reserved_radius+kRobotObstacleMargin;
                })) return false;
            }else if(dist_xz(center,robot.position)<=radius+robot_radius+kRobotObstacleMargin){
                return false;
            }
        }
        return true;
    };

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
                Vec3 preferred{};
                if(target && target->valid()) preferred=construction_drop_package_position(*target,visual_slot);
                else{
                    const float a=(float)(visual_slot%8)*(2.0f*PI/8.0f);
                    const float ring=0.85f+0.48f*(float)(visual_slot/8);
                    preferred=base+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
                    preferred.y=first_person_floor_height(preferred);
                }
                pile.radius=resource_package_half_width(material,amount);
                const float safety_radius=pile.radius*1.41421356f+kRobotObstacleMargin;
                auto placement_clear=[&](Vec3 p){
                    if(!scene_circle_clear_of_robot_contracts(p,safety_radius)) return false;
                    for(const auto& existing:ground_resource_piles_)
                        if(dist_xz(p,existing.position)<=safety_radius+existing.radius*1.41421356f+kRobotObstacleMargin) return false;
                    return true;
                };
                bool found=placement_clear(preferred);
                Vec3 chosen=preferred;
                if(!found){
                    constexpr std::array<float,4> rings{{0.9f,1.5f,2.2f,3.1f}};
                    for(float ring:rings){
                        for(int sample=0;sample<16 && !found;++sample){
                            const float a=(2.0f*PI*(float)sample)/16.0f;
                            Vec3 candidate=preferred+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
                            candidate.y=first_person_floor_height(candidate);
                            if(placement_clear(candidate)){ chosen=candidate; found=true; }
                        }
                        if(found) break;
                    }
                }
                // Defer this package rather than spawning it into reserved space.
                if(!found) break;
                pile.position=chosen;
                pile.condition=1.0f;
                ground_resource_piles_.push_back(pile);
                returned.amount(material)+=amount;
                remaining-=amount;
                ++visual_slot;
            }
        }
        if(!returned.empty()) resources_.add(returned);
        return returned;
    };

    // Construction dependencies gate BR-02 work only. Logistics is allowed to
    // pre-stage material at every placed blueprint (including corridors,
    // supply channels and walkways whose endpoints are not build-ready yet).
    //
    // Older code treated a dependency-blocked target as globally invalid: it
    // spilled the site buffer and cleared every assigned robot. Since v0.40.44
    // CR-01 can intentionally service those targets, that cleanup restarted a
    // carrier's PICK-UP transaction every frame and its action timer never
    // advanced. Keep staged stock and carrier reservations intact; only release
    // constructors until the dependency becomes ready.
    for(auto& robot:builder_robots_){
        if(robot.role!=RobotRole::Constructor || !robot.target.valid() ||
           construction_target_dependencies_ready(robot.target)) continue;
        robot.target={};
        robot.support_robot_index=kNoRobotIndex;
        robot.logistics_reservation={};
        robot.cargo_committed_to_build=false;
        robot.site_buffer={};
        robot.work_elapsed=0.0f;
        robot.idle_elapsed=0.0f;
        robot.stage_commit_pending=false;
        robot.stage_commit_safe_position={};
        robot.stage_commit_safe_position_valid=false;
        robot.stage_commit_alternate_attempted=false;
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
        robot.blocked_elapsed=0.0f;
        clear_action(robot);
        robot.mode=BuilderRobotMode::Idle;
    }

    // Construction is blueprint-owned. At most one BR-02 is assigned to a
    // given target at a time; other constructors remain available for separate
    // blueprints and dependency-ready network projects.
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
        // One constructor owns one blueprint at a time. Cooperative multi-BR-02
        // work on the same small structure saved only a few seconds but created
        // avoidable depot departures, perimeter conflicts and duplicated route
        // recovery. Multiple builders still work fully in parallel by taking
        // different placed blueprints; an extra builder waits until a new
        // structure/network project becomes dependency-ready.
        return target.valid()?(std::size_t)1:(std::size_t)0;
    };
    auto site_owner_index=[&](const ConstructionTargetRef& target){
        return construction_site_owner_index(target);
    };
    auto site_buffer_for=[&](const ConstructionTargetRef& target)->ResourceBundle*{
        return construction_site_buffer(target);
    };
    auto construction_stage_supplied=[&](const ConstructionTargetRef& target){
        if(!target.valid() || construction_target_complete(target)) return false;
        const auto* plan=construction_target_plan(target);
        const auto* staged=construction_site_buffer(target);
        if(!plan || !staged) return false;
        // Construction is now stage-fed by CR-01. A BR-02 may start as soon as
        // the NEXT stage bundle is physically present at the blueprint; it no
        // longer waits for the complete remaining project to be delivered.
        // This keeps builders productive while carriers independently seed all
        // other placed blueprints in the background.
        return contains_bundle(*staged,plan->next_step_cost(construction_target_step(target)));
    };
    auto construction_site_logistics_clear=[&](const ConstructionTargetRef& target){
        if(!target.valid()) return false;
        // The final CR-01 owns the apron until it has unloaded and physically
        // exited. Starting a corridor while that carrier is still beside it can
        // erect a permanent obstacle behind the rover and strand it on the
        // wrong side of the new tube. BR-02 therefore waits for the transaction
        // AND the departure to finish, not merely for the site-buffer count.
        for(const auto& fleet_robot:builder_robots_){
            if(fleet_robot.role!=RobotRole::Carrier || !fleet_robot.target.valid() ||
               !fleet_robot.target.same_as(target)) continue;
            if(fleet_robot.mode==BuilderRobotMode::Fetching ||
               fleet_robot.mode==BuilderRobotMode::Delivering ||
               fleet_robot.drop_exit_pending || fleet_robot.cargo_committed_to_build)
                return false;
        }
        return true;
    };


    auto partial_construction_blocks=[&](Vec3 candidate,float radius,const ConstructionTargetRef* ignore_target=nullptr){
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            if(ignore_target && ignore_target->kind==ConstructionTargetKind::Habitat && ignore_target->index==i) continue;
            const auto& h=placed_habitats_[i];
            if(h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            // Only COMMITTED construction stages are hard collision geometry.
            // step_elapsed is a visual build-up animation; making that partial
            // render state authoritative here can invalidate a rover contract
            // which was certified just before welding advanced a few pixels.
            // The full blueprint envelope is already traffic-reserved as soon
            // as construction starts, and every committed stage passes the
            // fleet/contract scene-admission gate below.
            const auto v=construction_sequence::habitat_visibility((float)h.construction.step);
            const bool visible=v.foundation>0.0f || v.shell>0.0f || v.interior>0.0f ||
                               v.details>0.0f || v.glazing>0.0f || v.access>0.0f;
            if(!visible) continue;
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            // Once visible, the current structure footprint is a hard rover
            // obstacle. During a one-way escape from the exact target being
            // built, callers may ignore only this conservative footprint; the
            // real mesh/structure collision test still applies to every move.
            if(std::abs(local.x)<mesh.footprint_x*0.5f+radius &&
               std::abs(local.z)<mesh.footprint_z*0.5f+radius) return true;
        }
        for(std::size_t i=0;i<placed_resources_.size();++i){
            if(ignore_target && ignore_target->kind==ConstructionTargetKind::Resource && ignore_target->index==i) continue;
            const auto& item=placed_resources_[i];
            if(item.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(item);
            const auto v=construction_sequence::resource_visibility((float)item.construction.step);
            const bool visible=v.foundation>0.0f || v.body>0.0f || v.details>0.0f;
            if(!visible) continue;
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            if(std::abs(local.x)<mesh.footprint_x*0.5f+radius &&
               std::abs(local.z)<mesh.footprint_z*0.5f+radius) return true;
        }
        for(std::size_t i=0;i<corridor_network_.connections().size();++i){
            if(ignore_target && ignore_target->kind==ConstructionTargetKind::Corridor && ignore_target->index==i) continue;
            const auto& c=corridor_network_.connections()[i];
            if(c.construction.complete() || c.construction.step==0) continue;
            if(!polyline_near_xz(candidate,c.path,radius+geometry::kCorridorHalfWidth+1.0f)) continue;
            const float f=(float)c.construction.step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
            if(construction_triangles_block_circle_range(c.mesh.shell,0,revealed_triangle_vertex_count(c.mesh.shell,f),candidate,radius,[](Vec3 v){return v;}) ||
               construction_triangles_block_circle_range(c.mesh.details,0,revealed_triangle_vertex_count(c.mesh.details,f),candidate,radius,[](Vec3 v){return v;})) return true;
        }
        // Supply channels are buried service runs. Their construction mesh is
        // deliberately below the rover wheel/chassis plane, so neither partial
        // nor completed channel geometry is a rover obstacle. Resource boxes at
        // an active construction site remain normal hard obstacles until used.
        for(std::size_t i=0;i<walkway_network_.connections().size();++i){
            if(ignore_target && ignore_target->kind==ConstructionTargetKind::Walkway && ignore_target->index==i) continue;
            const auto& c=walkway_network_.connections()[i];
            if(c.construction.complete() || c.construction.step==0) continue;
            if(!polyline_near_xz(candidate,c.path,radius+geometry::kWalkwayHalfWidth+0.9f)) continue;
            const float f=(float)c.construction.step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
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
                    // Cheap broad phase: the expensive pending-step triangle
                    // scan is only relevant close to this structure. Without
                    // this guard every A* cell on a long logistics route walked
                    // thousands of habitat vertices, producing visible 200+ ms
                    // main-thread stalls once construction was active. Keep a
                    // generous halo for authored stairs/access/details which may
                    // extend slightly beyond the nominal footprint.
                    const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
                    const float halo=radius+1.35f;
                    if(std::abs(local.x)>mesh.footprint_x*0.5f+halo ||
                       std::abs(local.z)>mesh.footprint_z*0.5f+halo) return false;
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
                    // Same broad phase as habitats: only inspect newly revealed
                    // triangles when the rover candidate is actually near the
                    // structure footprint.
                    const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
                    const float halo=radius+1.35f;
                    if(std::abs(local.x)>mesh.footprint_x*0.5f+halo ||
                       std::abs(local.z)>mesh.footprint_z*0.5f+halo) return false;
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
                // Buried supply-channel stages never create above-ground rover
                // collision geometry. The pre-build blueprint evacuation still
                // runs before construction starts, but stage commits no longer
                // force rovers to route around an underground service run.
                return false;
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

    // Full planned blueprint occupancy, independent of the current construction
    // stage. This is the pre-build exclusion volume: no rover body may remain
    // inside it when a constructor is allowed to start the first build action.
    auto target_blueprint_envelope_contains=[&](const ConstructionTargetRef& target,Vec3 candidate,float radius){
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& mesh=habitat_models_.mesh(h);
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            const float margin=radius+0.30f;
            return std::abs(local.x)<mesh.footprint_x*0.5f+margin &&
                   std::abs(local.z)<mesh.footprint_z*0.5f+margin;
        }
        if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& mesh=resource_models_.mesh(item);
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            const float margin=radius+0.30f;
            return std::abs(local.x)<mesh.footprint_x*0.5f+margin &&
                   std::abs(local.z)<mesh.footprint_z*0.5f+margin;
        }
        if(target.kind==ConstructionTargetKind::Corridor && target.index<corridor_network_.connections().size()){
            const auto& c=corridor_network_.connections()[target.index];
            return polyline_near_xz(candidate,c.path,geometry::kCorridorHalfWidth+radius+0.30f);
        }
        if(target.kind==ConstructionTargetKind::Utility && target.index<utility_network_.connections().size()){
            const auto& c=utility_network_.connections()[target.index];
            const float half_width=geometry::kSupplyTrayWidth*0.5f+0.35f;
            return polyline_near_xz(candidate,c.path,half_width+radius+0.30f);
        }
        if(target.kind==ConstructionTargetKind::Walkway && target.index<walkway_network_.connections().size()){
            const auto& c=walkway_network_.connections()[target.index];
            return polyline_near_xz(candidate,c.path,geometry::kWalkwayHalfWidth+radius+0.30f);
        }
        return false;
    };

    auto construction_blueprint_fleet_clear=[&](const ConstructionTargetRef& target){
        if(!target.valid()) return false;
        for(const auto& fleet_robot:builder_robots_){
            // Constructors assigned to this exact untouched blueprint are
            // allowed to cross its planned footprint on the way to their
            // exterior work bay.  The visible stage is still protected by the
            // fleet-safe stage-commit gate, so excluding them here removes the
            // pre-build escape/work-target tug-of-war without allowing geometry
            // to grow around a rover.
            if(fleet_robot.role==RobotRole::Constructor && fleet_robot.target.valid() &&
               fleet_robot.target.same_as(target)) continue;
            const float radius=robot_resource_clearance_radius(fleet_robot.role)+kRobotObstacleMargin;
            if(target_blueprint_envelope_contains(target,fleet_robot.position,radius)) return false;
            // Starting visible construction is a scene mutation.  Do not start
            // while an already-certified motion/braking contract can still
            // enter the future blueprint volume: current-position-only checks
            // are insufficient for a moving carrier.  Once this gate passes,
            // subsequent planners see the started blueprint reservation and
            // cannot issue a new contract through it.
            const auto& contract=fleet_robot.safety.active_contract;
            if(contract.valid && robots::contract_intersects_region(contract,[&](Vec3 p,float contract_radius){
                return target_blueprint_envelope_contains(target,p,contract_radius+kRobotObstacleMargin);
            })) return false;
        }
        return true;
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

        // Linear projects are hollow/open enough that triangle-only safety can
        // classify a rover in the middle of the future tube as "clear": its
        // centre does not touch a wall triangle, but once the shell appears it
        // has no legal route through that wall to get back out. Reserve the
        // complete planned strip before every stage commit. This is deliberately
        // conservative; construction work bays are already outside these strips.
        if(target.kind==ConstructionTargetKind::Corridor && target.index<corridor_network_.connections().size()){
            const auto& c=corridor_network_.connections()[target.index];
            return polyline_near_xz(candidate,c.path,geometry::kCorridorHalfWidth+radius+0.30f);
        }
        if(target.kind==ConstructionTargetKind::Utility){
            // The completed/next-stage channel is underground and cannot trap a
            // rover body, so no future-stage evacuation envelope is required.
            return false;
        }
        if(target.kind==ConstructionTargetKind::Walkway && target.index<walkway_network_.connections().size()){
            const auto& c=walkway_network_.connections()[target.index];
            return polyline_near_xz(candidate,c.path,geometry::kWalkwayHalfWidth+radius+0.30f);
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

    auto construction_target_started=[&](const ConstructionTargetRef& target){
        switch(target.kind){
            case ConstructionTargetKind::Habitat:
                return target.index<placed_habitats_.size() && placed_habitats_[target.index].construction.progress()>0.0001f;
            case ConstructionTargetKind::Resource:
                return target.index<placed_resources_.size() && placed_resources_[target.index].construction.progress()>0.0001f;
            case ConstructionTargetKind::Corridor:
                return target.index<corridor_network_.connections().size() && corridor_network_.connections()[target.index].construction.progress()>0.0001f;
            case ConstructionTargetKind::Utility:
                return target.index<utility_network_.connections().size() && utility_network_.connections()[target.index].construction.progress()>0.0001f;
            case ConstructionTargetKind::Walkway:
                return target.index<walkway_network_.connections().size() && walkway_network_.connections()[target.index].construction.progress()>0.0001f;
            case ConstructionTargetKind::None: break;
        }
        return false;
    };

    auto own_target_partial_penetration=[&](const BuilderRobotState& robot,Vec3 candidate){
        if(!robot.target.valid()) return 0.0f;
        const float radius=robot_safety_corridor_radius(robot.role);
        auto rectangular_penetration=[&](Vec3 center,float rotation,float footprint_x,float footprint_z){
            const Vec3 local=world_math::rotate_y(candidate-center,-rotation);
            const float dx=footprint_x*0.5f+radius-std::abs(local.x);
            const float dz=footprint_z*0.5f+radius-std::abs(local.z);
            return (dx>0.0f && dz>0.0f)?std::min(dx,dz):0.0f;
        };
        if(robot.target.kind==ConstructionTargetKind::Habitat && robot.target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[robot.target.index];
            if(h.construction.complete() || h.construction.progress()<=0.0001f) return 0.0f;
            const auto& mesh=habitat_models_.mesh(h);
            return rectangular_penetration(h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        }
        if(robot.target.kind==ConstructionTargetKind::Resource && robot.target.index<placed_resources_.size()){
            const auto& item=placed_resources_[robot.target.index];
            if(item.construction.complete() || item.construction.progress()<=0.0001f) return 0.0f;
            const auto& mesh=resource_models_.mesh(item);
            return rectangular_penetration(item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        }
        return 0.0f;
    };

    auto robot_position_clear_impl=[&](std::size_t robot_index,Vec3 candidate,const ConstructionTargetRef* ignore_partial_target){
        const auto& robot=builder_robots_[robot_index];
        // Structure/path occupancy must use the complete rendered chassis
        // envelope, not the legacy centre-body radius. The BR-02 is about
        // 1.24 m wide x 1.44 m long (CR-01 x 1.54 m), so a centre point can be
        // outside a wall while a wheel/chassis corner already intersects it.
        // A circumscribed hull radius is conservative for path planning and
        // guarantees every heading remains structure-clear.
        const float structure_radius=robot_safety_corridor_radius(robot.role);

        // A completed habitat/resource remains a solid rover footprint. Earlier
        // revisions removed the conservative footprint as soon as construction
        // reached 100% and relied only on authored triangle collision. Hollow
        // domes then looked like free A* space, so a BR-02/CR-01 could legally
        // plan through the building interior and become trapped at the shell on
        // the far side. Rovers never use indoor structure navigation, therefore
        // preserving the full footprint is both cheaper and physically correct.
        for(const auto& h:placed_habitats_){
            if(!h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            if(std::abs(local.x)<mesh.footprint_x*0.5f+structure_radius &&
               std::abs(local.z)<mesh.footprint_z*0.5f+structure_radius) return false;
        }
        for(const auto& item:placed_resources_){
            if(!item.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(item);
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            if(std::abs(local.x)<mesh.footprint_x*0.5f+structure_radius &&
               std::abs(local.z)<mesh.footprint_z*0.5f+structure_radius) return false;
        }
        // Airtight completed corridors are permanent outdoor rover obstacles.
        // Keeping their conservative centreline envelope in the cheap nav mask
        // lets A* avoid them without invoking triangle-level FPV collision for
        // every candidate grid cell. The final published route is still checked
        // by the strict mesh-aware segment validator.
        for(const auto& corridor:corridor_network_.connections()){
            if(!corridor.construction.complete() || corridor.path.size()<2) continue;
            if(polyline_near_xz(candidate,corridor.path,geometry::kCorridorHalfWidth+structure_radius+0.18f)) return false;
        }

        // Erected geometry is solid, and the volume that will appear in the
        // NEXT construction step is reserved before any rover starts a route.
        // This turns construction into a predictable moving obstacle instead of
        // evacuating robots only at the instant a wall/segment appears.
        if(partial_construction_blocks(candidate,structure_radius,ignore_partial_target)) return false;
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
            // Only a truly untouched blueprint is traversable. The instant
            // welding/build-up publishes visible progress, ordinary traffic is
            // excluded from the planned footprint even though the first full
            // stage has not committed yet. This prevents another rover from
            // entering geometry that is actively growing.
            if(current==0 && !construction_target_started(target)) return false;
            if(current==0 && construction_target_started(target))
                return target_blueprint_envelope_contains(target,candidate,structure_radius);
            const auto total=construction_target_total_steps(target);
            const auto next=(std::uint8_t)std::min<int>((int)current+1,(int)total);
            if(next<=current) return false;
            if(robot.role==RobotRole::Constructor && robot.target.valid() && robot.target.same_as(target))
                return false;
            return target_blocks_at_step(target,next,candidate,structure_radius);
        };
        for(std::size_t i=0;i<placed_habitats_.size();++i)
            if(future_reserved({ConstructionTargetKind::Habitat,i})) return false;
        for(std::size_t i=0;i<placed_resources_.size();++i)
            if(future_reserved({ConstructionTargetKind::Resource,i})) return false;
        for(std::size_t i=0;i<corridor_network_.connections().size();++i)
            if(future_reserved({ConstructionTargetKind::Corridor,i})) return false;
        // Buried supply channels never reserve rover path space after the
        // initial blueprint-clear gate; robots may cross them during/after build.
        for(std::size_t i=0;i<walkway_network_.connections().size();++i)
            if(future_reserved({ConstructionTargetKind::Walkway,i})) return false;

        const float resource_clearance=robot_resource_clearance_radius(robot.role);
        for(const auto& obstacle:cached_resource_obstacles)
            if(dist_xz(candidate,obstacle.position)<resource_clearance+obstacle.radius+kRobotObstacleMargin) return false;
        return true;
    };

    auto robot_position_clear=[&](std::size_t robot_index,Vec3 candidate){
        return robot_position_clear_impl(robot_index,candidate,nullptr);
    };

    // A newly revealed construction fraction can expand the conservative rover
    // footprint around a BR-02 that was legal a frame earlier. Never let that
    // conservative envelope imprison the constructor: it may move only in a
    // direction that strictly reduces its penetration into its own target's
    // footprint. All other structures, material boxes, fleet hulls and the real
    // authored mesh collision remain active, so this cannot become a shortcut
    // through a built structure.
    auto robot_position_clear_while_exiting_own_target=[&](std::size_t robot_index,Vec3 from,Vec3 candidate){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        const float from_penetration=own_target_partial_penetration(robot,from);
        if(from_penetration<=0.0005f) return robot_position_clear(robot_index,candidate);
        const float candidate_penetration=own_target_partial_penetration(robot,candidate);
        if(candidate_penetration>0.0005f && candidate_penetration>=from_penetration-0.0005f) return false;
        return robot_position_clear_impl(robot_index,candidate,&robot.target);
    };

    auto robot_structure_move_allowed_with_margin=[&](const BuilderRobotState& robot,Vec3 from,Vec3 to,float extra_structure_margin){
        // Rovers are exterior-only actors. Do not run the astronaut/FPV
        // pressure-volume and doorway solver for every robot segment: once
        // corridors exist that solver walks every pressure segment repeatedly
        // and can turn one A* publication into a 100+ ms main-thread stall.
        // Rover navigation already uses conservative, full-chassis inflated
        // footprints, so sample those same simple volumes along the segment.
        const float radius=robot_safety_corridor_radius(robot.role);
        const float structure_radius=radius+std::max(0.0f,extra_structure_margin);
        const float half=(float)playfield_.size()*0.5f-radius;
        const float length=dist_xz(from,to);
        const int samples=std::max(1,(int)std::ceil(length/0.22f));
        Vec3 previous=from;
        previous.y=first_person_floor_height(previous);
        for(int sample=1;sample<=samples;++sample){
            const float t=(float)sample/(float)samples;
            Vec3 p=from+(to-from)*t;
            p.y=first_person_floor_height(p);
            if(p.x<-half || p.x>half || p.z<-half || p.z>half) return false;
            if(std::abs(p.y-previous.y)>0.38f) return false;

            for(const auto& h:placed_habitats_){
                if(!h.construction.complete()) continue;
                const auto& mesh=habitat_models_.mesh(h);
                const Vec3 local=world_math::rotate_y(p-h.position,-h.rotation_degrees);
                if(std::abs(local.x)<mesh.footprint_x*0.5f+structure_radius &&
                   std::abs(local.z)<mesh.footprint_z*0.5f+structure_radius) return false;
            }
            for(const auto& item:placed_resources_){
                if(!item.construction.complete()) continue;
                const auto& mesh=resource_models_.mesh(item);
                const Vec3 local=world_math::rotate_y(p-item.position,-item.rotation_degrees);
                if(std::abs(local.x)<mesh.footprint_x*0.5f+structure_radius &&
                   std::abs(local.z)<mesh.footprint_z*0.5f+structure_radius) return false;
            }
            for(const auto& corridor:corridor_network_.connections()){
                if(!corridor.construction.complete() || corridor.path.size()<2) continue;
                if(polyline_near_xz(p,corridor.path,geometry::kCorridorHalfWidth+radius+0.18f)) return false;
            }
            for(const auto& obstacle:cached_resource_obstacles)
                if(dist_xz(p,obstacle.position)<robot_resource_clearance_radius(robot.role)+obstacle.radius+kRobotObstacleMargin)
                    return false;
            previous=p;
        }
        return true;
    };

    auto robot_structure_move_allowed=[&](const BuilderRobotState& robot,Vec3 from,Vec3 to){
        return robot_structure_move_allowed_with_margin(robot,from,to,0.0f);
    };

    auto completed_structure_margin_penetration=[&](const BuilderRobotState& robot,Vec3 candidate,float extra_margin){
        if(extra_margin<=0.0001f) return 0.0f;
        const float radius=robot_safety_corridor_radius(robot.role)+extra_margin;
        float penetration=0.0f;
        auto rect_penetration=[&](Vec3 center,float rotation,float footprint_x,float footprint_z){
            const Vec3 local=world_math::rotate_y(candidate-center,-rotation);
            const float dx=footprint_x*0.5f+radius-std::abs(local.x);
            const float dz=footprint_z*0.5f+radius-std::abs(local.z);
            if(dx>0.0f && dz>0.0f) penetration=std::max(penetration,std::min(dx,dz));
        };
        for(const auto& h:placed_habitats_){
            if(!h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            rect_penetration(h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        }
        for(const auto& item:placed_resources_){
            if(!item.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(item);
            rect_penetration(item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        }
        return penetration;
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

    auto fleet_position_clear=[&](std::size_t robot_index,Vec3 candidate){
        const auto& robot=builder_robots_[robot_index];
        float heading=robot.heading_degrees;
        const float dx=candidate.x-robot.position.x,dz=candidate.z-robot.position.z;
        if(dx*dx+dz*dz>0.0025f) heading=std::atan2(dx,dz)*180.0f/PI;
        return fleet_hull_clear(robot_index,candidate,heading);
    };

    // v0.40.30: high-level robot work code may request a destination, but it
    // never advances an authoritative pose. All requests are collected first and
    // committed together later by robots::safety_tick(). This replaces the former
    // rolling-path sampler, traffic speed governor, collision fallback steering,
    // breadcrumb reverse logic and overlap-nudge recovery.
    std::vector<robots::NavigationRequest> navigation_requests(builder_robots_.size());

    auto heading_between=[](Vec3 from,Vec3 to,float fallback){
        const float dx=to.x-from.x,dz=to.z-from.z;
        if(dx*dx+dz*dz<1.0e-8f) return fallback;
        return std::atan2(dx,dz)*180.0f/PI;
    };

    auto clear_route=[&](BuilderRobotState& robot){
        robot.safety.cached_route.clear();
        robot.safety.route_cursor=0;
        robot.safety.proposals.clear();
        robot.safety.route_replan_required=true;
        robot.planned_route.clear();
        robot.planned_route_distances.clear();
        robot.route_cursor=0;
        robot.route_active=false;
        robot.route_waiting_for_clearance=false;
        robot.route_wait_elapsed=0.0f;
        robot.route_progress_m=0.0f;
        robot.route_total_length_m=0.0f;
        robot.route_cruise_speed_mps=0.0f;
        robot.route_structure_clearance_margin_m=1.0f;
        robot.route_segment_elapsed=0.0f;
        robot.route_segment_origin={};
        robot.route_segment_origin_valid=false;
    };

    auto safety_segment_clear_with_margin=[&](std::size_t robot_index,Vec3 from,Vec3 to,float extra_structure_margin){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        const float length=dist_xz(from,to);
        // Even a very short certified motion must be checked at an interior
        // point. With one endpoint-only sample, a ~5 cm brake piece could enter
        // and leave a conservative obstacle boundary between its endpoints;
        // the contract would be accepted and the final partial-step guard would
        // then fault it. Two samples keep long-route cost unchanged while
        // closing that short-segment hole.
        const int samples=length>0.0001f?std::max(2,(int)std::ceil(length/0.18f)):1;
        Vec3 previous=from;
        previous.y=first_person_floor_height(previous);
        float previous_margin_penetration=completed_structure_margin_penetration(robot,previous,extra_structure_margin);
        for(int sample=1;sample<=samples;++sample){
            const float t=(float)sample/(float)samples;
            Vec3 p=from+(to-from)*t;
            p.y=first_person_floor_height(p);
            if(std::abs(p.y-previous.y)>0.38f) return false;
            if(!robot_position_clear_while_exiting_own_target(robot_index,previous,p)) return false;
            if(!robot_structure_move_allowed_with_margin(robot,previous,p,extra_structure_margin)){
                // If the rover already starts inside the preferred 1 m halo,
                // allow it to leave monotonically. The hard hull collision test
                // above still applies, so this is not permission to clip a wall.
                const float next_penetration=completed_structure_margin_penetration(robot,p,extra_structure_margin);
                if(previous_margin_penetration<=0.0005f ||
                   next_penetration>=previous_margin_penetration-0.0005f) return false;
                previous_margin_penetration=next_penetration;
            }else{
                previous_margin_penetration=0.0f;
            }
            previous=p;
        }
        return true;
    };

    auto safety_segment_clear=[&](std::size_t robot_index,Vec3 from,Vec3 to){
        if(robot_index>=builder_robots_.size()) return false;
        // The ~1 m structure buffer is a ROUTING preference, not hard collision
        // geometry.  The coarse planner selects the largest margin that keeps a
        // passage usable; the contract authority certifies against the actual
        // rover hull/structure geometry only.  Otherwise a builder legitimately
        // parked in its exterior work bay can become a safety fault the instant
        // the structure reaches 100% and its preferred halo appears around it.
        return safety_segment_clear_with_margin(robot_index,from,to,0.0f);
    };

    auto route_other_is_static_obstacle=[&](std::size_t robot_index,std::size_t other_index){
        if(robot_index>=builder_robots_.size() || other_index>=builder_robots_.size() || robot_index==other_index) return false;
        const auto& other=builder_robots_[other_index];
        const bool other_active=other_index<navigation_requests.size() && navigation_requests[other_index].active;
        if(!other_active) return true;
        if(std::abs(other.current_speed_mps)>0.30f) return false;
        if(other.safety.motion_intent==robots::MotionIntent::Wait ||
           other.safety.mode==robots::SafetyMode::Idle ||
           other.safety.mode==robots::SafetyMode::Working ||
           other.safety.mode==robots::SafetyMode::Blocked ||
           other.safety.mode==robots::SafetyMode::Waiting)
            return true;
        const auto own_priority=robot_index<navigation_requests.size()?navigation_requests[robot_index].task_priority:0u;
        const auto other_priority=other_index<navigation_requests.size()?navigation_requests[other_index].task_priority:0u;
        return other_priority>own_priority;
    };

    auto route_fleet_point_clear=[&](std::size_t robot_index,Vec3 candidate){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        const float own_hard_radius=robot_safety_corridor_radius(robot.role)+
            robots::kSafetyNumericMarginM;
        for(std::size_t other_index=0;other_index<builder_robots_.size();++other_index){
            if(other_index==robot_index || !route_other_is_static_obstacle(robot_index,other_index)) continue;
            const auto& other=builder_robots_[other_index];
            const float other_hard_radius=robot_safety_corridor_radius(other.role)+
                robots::kSafetyNumericMarginM;
            const float required=own_hard_radius+other_hard_radius+
                robots::kNeighborComfortClearanceM;
            const float candidate_distance=dist_xz(candidate,other.position);
            if(candidate_distance>=required) continue;

            // A save can begin with two physically clear rovers already inside
            // the personal zones. The active rover may leave that state, but
            // every sampled route point must maintain or increase separation.
            const float current_distance=dist_xz(robot.position,other.position);
            if(current_distance<required &&
               candidate_distance+robots::kPersonalZoneEscapeEpsilonM>=current_distance)
                continue;
            return false;
        }
        return true;
    };

    auto route_fleet_segment_clear=[&](std::size_t robot_index,Vec3 from,Vec3 to){
        const float length=dist_xz(from,to);
        const int samples=std::max(1,(int)std::ceil(length/0.55f));
        // The route origin is the rover's current occupied point and is allowed
        // even when another safety disc is close. Every forward sample must
        // open/maintain clearance from the local fleet.
        for(int sample=1;sample<=samples;++sample){
            const float t=(float)sample/(float)samples;
            if(!route_fleet_point_clear(robot_index,from+(to-from)*t)) return false;
        }
        return true;
    };

    auto coarse_safety_route=[&](std::size_t robot_index,Vec3 start,Vec3 goal,float stop_radius,std::vector<Vec3>& route){
        if(robot_index>=builder_robots_.size()) return false;
        auto& robot=builder_robots_[robot_index];
        robots::PathPlanRequest request{};
        request.start=start;
        request.goal=goal;
        request.stop_radius=stop_radius;
        // Keep coarse routing cheap. Most trips should resolve as a direct line
        // or a one/two-bend bypass; only genuinely obstructed layouts pay for a
        // bounded coarse-grid search. The 10 ms contract authority still owns
        // the actual collision guarantee.
        request.cell_size=1.05f;
        // Keep the common A* corridor narrow. The previous 0.40.44 draft made
        // every failed route search up to 24 m wide, which fixed one Storage
        // Dome detour but reintroduced large main-frame spikes when two builders
        // were busy. Wide clearance is cheap for the direct/dogleg probes; only
        // a final low-resolution fallback is allowed to search the wider area.
        const float map_half=std::max(8.0f,(float)playfield_.size()*0.5f-2.0f);
        const float cheap_half_width=std::clamp(dist_xz(start,goal)*0.42f+4.0f,5.0f,std::min(20.0f,map_half));
        const float normal_half_width=std::clamp(dist_xz(start,goal)*0.30f+2.8f,4.5f,std::min(12.0f,map_half));
        const float wide_half_width=std::clamp(dist_xz(start,goal)*0.42f+4.0f,8.0f,std::min(24.0f,map_half));
        request.corridor_half_width=cheap_half_width;
        request.max_expansions=1200;
        request.visibility_skip_limit=8;

        auto try_margin=[&](float margin,bool allow_grid){
            robots::PathPlannerCallbacks callbacks{};
            callbacks.floor_height=[&](Vec3 p){ return first_person_floor_height(p); };
            callbacks.point_clear=[&](Vec3 p){
                return robot_position_clear(robot_index,p) &&
                       robot_structure_move_allowed_with_margin(robot,p,p,margin) &&
                       route_fleet_point_clear(robot_index,p);
            };
            callbacks.segment_clear=[&](Vec3 a,Vec3 b){
                return safety_segment_clear_with_margin(robot_index,a,b,margin) &&
                       route_fleet_segment_clear(robot_index,a,b);
            };
            // A* neighbors are only one grid cell apart. Re-running the full
            // swept segment sampler (structures + fleet + margin) for every
            // one of up to eight neighbors was the dominant cost of blocked
            // route searches. Endpoint + midpoint occupancy is sufficient for
            // this advisory coarse grid; every simplified output segment is
            // still validated with segment_clear(), and the 10 ms contract
            // authority remains the final collision gate.
            callbacks.neighbor_clear=[&](Vec3 a,Vec3 b){
                Vec3 mid=(a+b)*0.5f;
                mid.y=first_person_floor_height(mid);
                return robot_position_clear(robot_index,mid) &&
                       robot_structure_move_allowed_with_margin(robot,mid,mid,margin) &&
                       route_fleet_point_clear(robot_index,mid) &&
                       robot_position_clear(robot_index,b) &&
                       robot_structure_move_allowed_with_margin(robot,b,b,margin) &&
                       route_fleet_point_clear(robot_index,b);
            };
            request.allow_grid_fallback=allow_grid;
            route.clear();
            if(!robots::plan_corridor_path(request,callbacks,route)) return false;
            robot.route_structure_clearance_margin_m=margin;
            return true;
        };

        // First probe all comfort margins using only direct/dogleg geometry.
        // This keeps two or more working builders from launching several large
        // A* searches for what is normally a simple path around one structure.
        constexpr std::array<float,2> cheap_margins{{1.00f,0.00f}};
        for(float margin:cheap_margins) if(try_margin(margin,false)) return true;

        // Complex layouts first get ONE low-resolution wide search. Long
        // structure/corridor barriers are common in a developed base and were
        // the source of the visible 100+ ms frame spikes: the old planner first
        // exhausted a fine narrow grid that could not possibly reach around the
        // barrier, then performed the wide search anyway. A coarse wide search
        // resolves those cases quickly while retaining a modest 0.30 m comfort
        // buffer.
        request.corridor_half_width=wide_half_width;
        request.cell_size=1.85f;
        request.max_expansions=700;
        request.visibility_skip_limit=5;
        if(try_margin(0.30f,true)) return true;

        // Narrow passages that the coarse grid cannot represent get one final
        // fine physical-clearance search. This is a fallback, not the common
        // path, so two busy builders do not repeatedly pay for it.
        request.corridor_half_width=normal_half_width;
        request.cell_size=1.05f;
        request.max_expansions=1200;
        request.visibility_skip_limit=8;
        if(try_margin(0.00f,true)) return true;

        robot.route_structure_clearance_margin_m=0.0f;
        route.clear();
        return false;
    };

    auto move_towards=[&](std::size_t robot_index,Vec3 destination,float stop_radius,float speed_scale){
        if(robot_index>=builder_robots_.size()) return false;
        auto& robot=builder_robots_[robot_index];
        destination.y=first_person_floor_height(destination);
        const float stop=std::max(0.02f,stop_radius);
        robot.route_destination=destination;
        robot.route_stop_radius=stop;
        robot.route_speed_scale=std::clamp(speed_scale,0.10f,1.0f);
        robot.route_cruise_speed_mps=robot.move_speed_mps*robot.route_speed_scale;
        navigation_requests[robot_index].active=true;
        navigation_requests[robot_index].goal=destination;
        navigation_requests[robot_index].stop_radius=stop;
        navigation_requests[robot_index].speed_scale=robot.route_speed_scale;
        navigation_requests[robot_index].work_target=false;
        return dist_xz(robot.position,destination)<=stop+0.025f && std::abs(robot.current_speed_mps)<0.04f;
    };

    auto constructor_parking_position=[&](std::size_t robot_index){
        if(robot_index>=builder_robots_.size()) return starter_stash_center_;

        // BR-02 home bays are the same parking apron used for a fresh colony.
        // This apron sits beside the starter stash but outside the pallet lanes,
        // so returning builders do not compete with CR-01 pickup traffic.
        std::size_t constructor_ordinal=0;
        for(std::size_t i=0;i<robot_index;++i)
            if(builder_robots_[i].role==RobotRole::Constructor) ++constructor_ordinal;

        const float half=(float)playfield_.size()*0.5f;
        auto nominal_for=[&](std::size_t ordinal){
            Vec3 p{};
            if(half>=12.0f){
                const Vec3 center=starter_stash_center_+Vec3{4.60f,0.0f,11.60f};
                const int column=(int)(ordinal%3u);
                const int row=(int)(ordinal/3u);
                p={center.x+((float)column-1.0f)*3.00f,0.0f,
                   center.z+((float)row-1.5f)*3.00f};
            }else{
                const Vec3 center=starter_stash_center_+Vec3{7.00f,0.0f,0.0f};
                const int column=(int)(ordinal%2u);
                const int row=(int)(ordinal/2u);
                p={center.x+((float)column-0.5f)*2.40f,0.0f,
                   center.z+((float)row-2.5f)*2.40f};
            }
            const float edge=robot_resource_clearance_radius(RobotRole::Constructor)+kRobotObstacleMargin+0.12f;
            p.x=std::clamp(p.x,-half+edge,half-edge);
            p.z=std::clamp(p.z,-half+edge,half-edge);
            p.y=first_person_floor_height(p);
            return p;
        };

        auto overlaps_any_blueprint=[&](Vec3 candidate){
            const float radius=robot_resource_clearance_radius(RobotRole::Constructor)+0.35f;
            for(std::size_t i=0;i<placed_habitats_.size();++i)
                if(!placed_habitats_[i].construction.complete() &&
                   target_blueprint_envelope_contains({ConstructionTargetKind::Habitat,i},candidate,radius)) return true;
            for(std::size_t i=0;i<placed_resources_.size();++i)
                if(!placed_resources_[i].construction.complete() &&
                   target_blueprint_envelope_contains({ConstructionTargetKind::Resource,i},candidate,radius)) return true;
            for(std::size_t i=0;i<corridor_network_.connections().size();++i)
                if(!corridor_network_.connections()[i].construction.complete() &&
                   target_blueprint_envelope_contains({ConstructionTargetKind::Corridor,i},candidate,radius)) return true;
            for(std::size_t i=0;i<utility_network_.connections().size();++i)
                if(!utility_network_.connections()[i].construction.complete() &&
                   target_blueprint_envelope_contains({ConstructionTargetKind::Utility,i},candidate,radius)) return true;
            for(std::size_t i=0;i<walkway_network_.connections().size();++i)
                if(!walkway_network_.connections()[i].construction.complete() &&
                   target_blueprint_envelope_contains({ConstructionTargetKind::Walkway,i},candidate,radius)) return true;
            return false;
        };

        const Vec3 nominal=nominal_for(constructor_ordinal);
        auto candidate_clear=[&](Vec3 candidate){
            return robot_position_clear(robot_index,candidate) &&
                   fleet_position_clear(robot_index,candidate) &&
                   !overlaps_any_blueprint(candidate);
        };
        if(candidate_clear(nominal)) return nominal;

        // A player may later build over an original parking bay. Search a small
        // bounded set around the apron instead of parking on/inside a blueprint.
        // Deterministic offsets keep multiple builders from oscillating between
        // candidate bays.
        constexpr std::array<Vec3,12> offsets{{
            {3.0f,0.0f,0.0f},{-3.0f,0.0f,0.0f},{0.0f,0.0f,3.0f},{0.0f,0.0f,-3.0f},
            {3.0f,0.0f,3.0f},{-3.0f,0.0f,3.0f},{3.0f,0.0f,-3.0f},{-3.0f,0.0f,-3.0f},
            {6.0f,0.0f,0.0f},{0.0f,0.0f,6.0f},{-6.0f,0.0f,0.0f},{0.0f,0.0f,-6.0f}
        }};
        for(const Vec3 offset:offsets){
            Vec3 candidate=nominal+offset;
            const float edge=robot_resource_clearance_radius(RobotRole::Constructor)+kRobotObstacleMargin+0.12f;
            candidate.x=std::clamp(candidate.x,-half+edge,half-edge);
            candidate.z=std::clamp(candidate.z,-half+edge,half-edge);
            candidate.y=first_person_floor_height(candidate);
            if(candidate_clear(candidate)) return candidate;
        }
        return nominal;
    };

    auto move_construction_service_direct=[&](std::size_t robot_index,Vec3 destination,float stop_radius,float speed_scale){
        // There is intentionally no direct-drive escape path anymore. Safety,
        // retreat, docking and construction evacuation all submit the same kind
        // of request and are certified by the same coordinator.
        return move_towards(robot_index,destination,stop_radius,speed_scale);
    };

    auto robot_safe_for_next_stage=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::uint8_t next_step){
        const auto& robot=builder_robots_[robot_index];
        const float radius=robot_resource_clearance_radius(robot.role)+kRobotObstacleMargin;
        if(target_future_envelope_contains(target,next_step,robot.position,radius)) return false;
        if(target_blocks_at_step(target,next_step,robot.position,radius)) return false;
        return true;
    };

    auto safe_stage_commit_destination=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::uint8_t next_step){
        const auto& robot=builder_robots_[robot_index];
        const float body_radius=robot_resource_clearance_radius(robot.role);

        // A carrier being cleared from a linear construction must stay on the
        // logistics/depot side before the new corridor/channel/walkway segment
        // becomes solid. Choosing only the nearest ring point can move CR-01
        // into the pocket between a corridor and its endpoint structures; once
        // the segment is committed there may be no legal full-body route back
        // to the starter pallets. Prefer short outward steps toward the depot.
        if(robot.role==RobotRole::Carrier &&
           (target.kind==ConstructionTargetKind::Corridor ||
            target.kind==ConstructionTargetKind::Utility ||
            target.kind==ConstructionTargetKind::Walkway)){
            Vec3 depot_dir=starter_stash_center_-robot.position; depot_dir.y=0.0f;
            float depot_len=std::sqrt(depot_dir.x*depot_dir.x+depot_dir.z*depot_dir.z);
            if(depot_len>0.05f){
                depot_dir=depot_dir*(1.0f/depot_len);
                const float base=std::atan2(depot_dir.z,depot_dir.x);
                constexpr std::array<float,9> offs{{0.0f,0.28f,-0.28f,0.58f,-0.58f,0.92f,-0.92f,1.28f,-1.28f}};
                constexpr std::array<float,5> radii{{1.45f,2.15f,3.0f,4.0f,5.2f}};
                for(float radius:radii){
                    for(float off:offs){
                        const float a=base+off;
                        Vec3 candidate=robot.position+Vec3{std::cos(a),0.0f,std::sin(a)}*radius;
                        candidate.y=first_person_floor_height(candidate);
                        if(!robot_position_clear(robot_index,candidate) || !fleet_position_clear(robot_index,candidate)) continue;
                        if(target_future_envelope_contains(target,next_step,candidate,body_radius+kRobotObstacleMargin)) continue;
                        if(target_blocks_at_step(target,next_step,candidate,body_radius+kRobotObstacleMargin)) continue;
                        if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
                        return candidate;
                    }
                }
            }
        }

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
        for(std::size_t i=0;i<builder_robots_.size();++i){
            if(!robot_safe_for_next_stage(i,target,next_step)) return false;
            const auto& contract=builder_robots_[i].safety.active_contract;
            if(contract.valid && robots::contract_intersects_region(contract,[&](Vec3 p,float radius){
                return target_future_envelope_contains(target,next_step,p,radius) ||
                       target_blocks_at_step(target,next_step,p,radius);
            })) return false;
        }
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
            if(robot_safe_for_next_stage(ri,constructor.target,next_step)){
                builder_robots_[ri].stage_commit_safe_position={};
                builder_robots_[ri].stage_commit_safe_position_valid=false;
                builder_robots_[ri].stage_commit_alternate_attempted=false;
                builder_robots_[ri].stage_commit_evacuate_elapsed=0.0f;
                continue;
            }
            auto& evac=builder_robots_[ri];
            evac.stage_commit_evacuate_elapsed+=dt;
            const float body_radius=robot_resource_clearance_radius(evac.role);
            auto evacuation_destination_still_valid=[&](Vec3 candidate){
                if(!evac.stage_commit_safe_position_valid) return false;
                // Keep the chosen point stable when another rover merely passes
                // through it. Dynamic fleet conflicts are handled by the route
                // governor; changing the goal here would recreate route thrash.
                if(!robot_position_clear(ri,candidate)) return false;
                if(target_future_envelope_contains(constructor.target,next_step,candidate,body_radius+kRobotObstacleMargin)) return false;
                if(target_blocks_at_step(constructor.target,next_step,candidate,body_radius+kRobotObstacleMargin)) return false;
                return robot_structure_move_allowed(evac,candidate,candidate);
            };
            if(!evacuation_destination_still_valid(evac.stage_commit_safe_position)){
                evac.stage_commit_safe_position=safe_stage_commit_destination(ri,constructor.target,next_step);
                evac.stage_commit_safe_position_valid=true;
                evac.stage_commit_alternate_attempted=false;
                clear_route(evac);
                evac.route_replan_cooldown=0.0f;
            }
            Vec3 destination=evac.stage_commit_safe_position;
            // Evacuation owns the robot completely; work animations must not
            // keep the watchdog artificially "alive" while the chassis is
            // failing to leave a future wall volume.
            clear_action(evac);
            // Safety destinations are already body-clear points; do not stop a full
            // 20 cm short of them, because that can still leave the robot
            // intersecting the next stage's clearance envelope.
            // Stage evacuation is always a short local safety maneuver. Using
            // the global A* planner here caused 100-250 ms main-thread spikes
            // during corridor mesh commits and made camera/mouse input stutter.
            // The local mover checks the full chassis on every step; if a direct
            // maneuver is blocked, the alternate/5 s safety recovery below takes
            // over without repeatedly launching a world search.
            const bool reached=move_construction_service_direct(ri,destination,0.025f,0.72f);
            if(!reached && evac.stage_commit_evacuate_elapsed>2.0f && !evac.stage_commit_alternate_attempted){
                // After one genuine blocked interval choose exactly one alternate
                // exterior sector. Do not reset blocked_elapsed here: if the
                // alternate is also unsolvable, the 5 s hard safety relocation
                // below must remain reachable instead of restarting forever.
                evac.service_slot_offset=(std::uint8_t)((evac.service_slot_offset+7u)%24u);
                clear_route(evac);
                evac.route_replan_cooldown=0.0f;
                evac.stage_commit_safe_position=safe_stage_commit_destination(ri,constructor.target,next_step);
                evac.stage_commit_safe_position_valid=true;
                evac.stage_commit_alternate_attempted=true;
                destination=evac.stage_commit_safe_position;
            }
            // No hard relocation fallback: persistent blocking is resolved only
            // through certified wait/retreat/detour behavior. Construction remains
            // deferred until the future geometry is clear.
            safety_repositioning[ri]=true;
        }
    }

    // Before the FIRST construction action of any blueprint, physically clear
    // every rover out of the complete planned footprint/strip. This is stronger
    // than the per-stage safety gate above: the blueprint must be empty before
    // BR-02 is even allowed to acquire/start the target. It prevents carriers
    // or idle constructors from becoming an obstacle at corridor build start.
    auto prebuild_escape_destination=[&](std::size_t robot_index,const ConstructionTargetRef& target){
        const auto& robot=builder_robots_[robot_index];
        const float body=robot_resource_clearance_radius(robot.role)+kRobotObstacleMargin;
        Vec3 candidate=robot.position;
        bool have_candidate=false;

        auto validate=[&](Vec3 p){
            p.y=first_person_floor_height(p);
            return !target_blueprint_envelope_contains(target,p,body) &&
                   robot_position_clear(robot_index,p) &&
                   robot_structure_move_allowed(robot,p,p);
        };

        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index];
            const auto& mesh=habitat_models_.mesh(h);
            Vec3 local=world_math::rotate_y(robot.position-h.position,-h.rotation_degrees);
            const float hx=mesh.footprint_x*0.5f+body+0.55f;
            const float hz=mesh.footprint_z*0.5f+body+0.55f;
            const float dx=hx-std::abs(local.x);
            const float dz=hz-std::abs(local.z);
            if(dx<=dz) local.x=(local.x>=0.0f?hx:-hx);
            else local.z=(local.z>=0.0f?hz:-hz);
            candidate=h.position+world_math::rotate_y(local,h.rotation_degrees);
            have_candidate=validate(candidate);
        }else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index];
            const auto& mesh=resource_models_.mesh(item);
            Vec3 local=world_math::rotate_y(robot.position-item.position,-item.rotation_degrees);
            const float hx=mesh.footprint_x*0.5f+body+0.55f;
            const float hz=mesh.footprint_z*0.5f+body+0.55f;
            const float dx=hx-std::abs(local.x);
            const float dz=hz-std::abs(local.z);
            if(dx<=dz) local.x=(local.x>=0.0f?hx:-hx);
            else local.z=(local.z>=0.0f?hz:-hz);
            candidate=item.position+world_math::rotate_y(local,item.rotation_degrees);
            have_candidate=validate(candidate);
        }else{
            const std::vector<Vec3>* path=nullptr;
            float half_width=0.8f;
            if(target.kind==ConstructionTargetKind::Corridor && target.index<corridor_network_.connections().size()){
                path=&corridor_network_.connections()[target.index].path;
                half_width=geometry::kCorridorHalfWidth;
            }else if(target.kind==ConstructionTargetKind::Utility && target.index<utility_network_.connections().size()){
                path=&utility_network_.connections()[target.index].path;
                half_width=geometry::kSupplyTrayWidth*0.5f+0.35f;
            }else if(target.kind==ConstructionTargetKind::Walkway && target.index<walkway_network_.connections().size()){
                path=&walkway_network_.connections()[target.index].path;
                half_width=geometry::kWalkwayHalfWidth;
            }
            if(path && path->size()>=2){
                Vec3 nearest=(*path)[0];
                Vec3 tangent=(*path)[1]-(*path)[0];
                float best=std::numeric_limits<float>::max();
                for(std::size_t pi=0;pi+1<path->size();++pi){
                    const Vec3 a=(*path)[pi], b=(*path)[pi+1];
                    const Vec3 ab{b.x-a.x,0.0f,b.z-a.z};
                    const float len2=ab.x*ab.x+ab.z*ab.z;
                    float t=0.0f;
                    if(len2>1e-6f) t=std::clamp(((robot.position.x-a.x)*ab.x+(robot.position.z-a.z)*ab.z)/len2,0.0f,1.0f);
                    Vec3 q=a+ab*t;
                    const float d=dist_xz(robot.position,q);
                    if(d<best){ best=d; nearest=q; tangent=ab; }
                }
                Vec3 away=robot.position-nearest; away.y=0.0f;
                float len=std::sqrt(away.x*away.x+away.z*away.z);
                if(len<0.05f){
                    const float tlen=std::sqrt(tangent.x*tangent.x+tangent.z*tangent.z);
                    if(tlen>0.01f) away={-tangent.z/tlen,0.0f,tangent.x/tlen};
                    else away={1.0f,0.0f,0.0f};
                }else away=away*(1.0f/len);
                const float radius=half_width+body+0.62f;
                constexpr std::array<float,7> fan{{0.0f,0.30f,-0.30f,0.62f,-0.62f,0.95f,-0.95f}};
                const float base=std::atan2(away.z,away.x);
                for(float off:fan){
                    const float a=base+off;
                    Vec3 p=nearest+Vec3{std::cos(a),0.0f,std::sin(a)}*radius;
                    if(validate(p)){ candidate=p; have_candidate=true; break; }
                }
            }
        }

        if(!have_candidate){
            candidate=safe_stage_commit_destination(robot_index,target,construction_target_total_steps(target));
            candidate.y=first_person_floor_height(candidate);
        }
        return candidate;
    };

    // Clear fully supplied, dependency-ready, not-yet-started blueprints. Network
    // connections are visited first so a corridor that is ready to build cannot
    // remain indefinitely blocked by an idle rover while builders select later
    // structures. safety_repositioning gives this short move exclusive control.
    auto evacuate_prebuild_target=[&](const ConstructionTargetRef& target){
        if(!target.valid() || construction_target_complete(target) ||
           construction_target_step(target)!=0 || !construction_target_dependencies_ready(target) ||
           !construction_stage_supplied(target) || !construction_site_logistics_clear(target)) return;

        // Do not reserve/evacuate every supplied blueprint just because it is
        // ready. Until a BR-02 has actively claimed this exact target, the
        // blueprint remains fully traversable gameplay space.
        bool claimed_for_immediate_build=false;
        for(const auto& fleet_robot:builder_robots_){
            if(fleet_robot.role!=RobotRole::Constructor || !fleet_robot.target.valid() ||
               !fleet_robot.target.same_as(target)) continue;
            if(fleet_robot.mode==BuilderRobotMode::AwaitingMaterials ||
               fleet_robot.mode==BuilderRobotMode::Building){
                claimed_for_immediate_build=true;
                break;
            }
        }
        if(!claimed_for_immediate_build) return;
        for(std::size_t ri=0;ri<builder_robots_.size();++ri){
            if(safety_repositioning[ri]) continue;
            auto& evac=builder_robots_[ri];
            // The BR-02 assigned to this blueprint owns an exterior work bay
            // and may cross the still-empty footprint to reach it.  Do not send
            // that same rover to an unrelated evacuation point every frame.
            // Other robots are still cleared before the first stage can appear.
            if(evac.role==RobotRole::Constructor && evac.target.valid() && evac.target.same_as(target)) continue;
            const float body=robot_resource_clearance_radius(evac.role)+kRobotObstacleMargin;
            if(!target_blueprint_envelope_contains(target,evac.position,body)){
                continue;
            }
            const Vec3 destination=prebuild_escape_destination(ri,target);
            clear_action(evac);
            (void)move_construction_service_direct(ri,destination,0.04f,0.70f);
            evac.stage_commit_evacuate_elapsed+=dt;
            // No pose teleport is permitted. If evacuation cannot be certified,
            // this blueprint simply remains deferred and the safety coordinator
            // continues normal wait/retreat recovery.
            safety_repositioning[ri]=true;
        }
    };
    for(std::size_t i=0;i<corridor_network_.connections().size();++i)
        evacuate_prebuild_target({ConstructionTargetKind::Corridor,i});
    for(std::size_t i=0;i<utility_network_.connections().size();++i)
        evacuate_prebuild_target({ConstructionTargetKind::Utility,i});
    for(std::size_t i=0;i<walkway_network_.connections().size();++i)
        evacuate_prebuild_target({ConstructionTargetKind::Walkway,i});
    for(std::size_t i=0;i<placed_habitats_.size();++i)
        evacuate_prebuild_target({ConstructionTargetKind::Habitat,i});
    for(std::size_t i=0;i<placed_resources_.size();++i)
        evacuate_prebuild_target({ConstructionTargetKind::Resource,i});

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
                                  std::abs(pile.position.z-starter_stash_center_.z)<kStarterPalletHalfSpanM;
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

    // Work points are deterministic perimeter sectors. Each build stage uses one
    // side only and rotates to the next side on the following stage. This keeps
    // visible construction activity around the structure without forcing a
    // constructor to traverse an obstacle corner in the middle of every stage.
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

        // Linear projects are worked from the side the constructor is already
        // on. Their staged resource packages intentionally sit on the blueprint
        // centreline, so choosing a fixed north/east/south/west sector can put
        // the work bay on the opposite side and ask A* to drive through the
        // material pile or the partially erected segment. Stay on the nearest
        // lateral side and offset multiple builders along the path instead.
        if(!rectangular && (target.kind==ConstructionTargetKind::Corridor ||
                            target.kind==ConstructionTargetKind::Utility ||
                            target.kind==ConstructionTargetKind::Walkway)){
            const std::vector<Vec3>* path=nullptr;
            if(target.kind==ConstructionTargetKind::Corridor && target.index<corridor_network_.connections().size())
                path=&corridor_network_.connections()[target.index].path;
            else if(target.kind==ConstructionTargetKind::Utility && target.index<utility_network_.connections().size())
                path=&utility_network_.connections()[target.index].path;
            else if(target.kind==ConstructionTargetKind::Walkway && target.index<walkway_network_.connections().size())
                path=&walkway_network_.connections()[target.index].path;
            if(path && path->size()>=2){
                const float f=((float)construction_target_step(target)+0.5f)/(float)std::max<std::uint8_t>(1,construction_target_total_steps(target));
                const Vec3 a=point_along_path(*path,std::max(0.0f,f-0.035f));
                const Vec3 b=point_along_path(*path,std::min(1.0f,f+0.035f));
                Vec3 tangent=b-a; tangent.y=0.0f;
                float tl=std::sqrt(tangent.x*tangent.x+tangent.z*tangent.z);
                if(tl>0.05f){
                    tangent=tangent*(1.0f/tl);
                    const Vec3 work_center=point_along_path(*path,f);
                    Vec3 lateral{-tangent.z,0.0f,tangent.x};
                    if((robot.position.x-work_center.x)*lateral.x+(robot.position.z-work_center.z)*lateral.z<0.0f)
                        lateral=lateral*-1.0f;
                    float project_half_width=0.80f;
                    if(target.kind==ConstructionTargetKind::Corridor) project_half_width=geometry::kCorridorHalfWidth;
                    else if(target.kind==ConstructionTargetKind::Utility) project_half_width=geometry::kSupplyTrayWidth*0.5f+0.35f;
                    else if(target.kind==ConstructionTargetKind::Walkway) project_half_width=geometry::kWalkwayHalfWidth;
                    // End each work cycle outside the *full chassis* future
                    // envelope, including the arrival tolerance. Search both
                    // sides of the local segment instead of blindly choosing the
                    // constructor's current side: in 0-40-15 corridor #2 that
                    // preferred side was inside the completed 13 m Storage Dome,
                    // so BR-02 started multi-second A* searches toward a work bay
                    // its body could never occupy.
                    const float base_ring=project_half_width+robot_resource_clearance_radius(robot.role)+
                                          kRobotObstacleMargin+0.70f;
                    float worker_along=0.0f;
                    if(total_workers>1){
                        const float centered=(float)ordinal-((float)total_workers-1.0f)*0.5f;
                        worker_along=centered*1.45f;
                    }
                    constexpr std::array<float,4> ring_extra{{0.0f,0.75f,1.50f,2.35f}};
                    constexpr std::array<float,5> along_extra{{0.0f,0.90f,-0.90f,1.80f,-1.80f}};
                    Vec3 best{};
                    float best_score=std::numeric_limits<float>::max();
                    bool found=false;
                    // A linear work bay must leave enough room for the complete
                    // rover bodies, not merely pass the oriented-rectangle test
                    // at one exact heading. In 0-40-20.gmsave the selected bay
                    // was only 1.98 m from an idle CR-01: geometrically legal at
                    // the sampled pose, but too tight for the dynamic route
                    // preflight, so BR-02 could never start its approach.
                    auto conservative_fleet_bay_clear=[&](Vec3 candidate){
                        const float own=robot_resource_clearance_radius(robot.role);
                        for(std::size_t fi=0;fi<builder_robots_.size();++fi){
                            if(fi==robot_index) continue;
                            const auto& other=builder_robots_[fi];
                            const float required=own+robot_resource_clearance_radius(other.role)+
                                                 kRobotHullClearanceM+0.38f;
                            if(dist_xz(candidate,other.position)<required) return false;
                        }
                        return true;
                    };
                    // Watchdog recovery increments service_slot_offset by five.
                    // Use that persistent offset to alternate the preferred side
                    // and along-path slots; otherwise every recovery recomputes
                    // the identical blocked corridor bay forever.
                    const bool flip_side=((robot.service_slot_offset/5u)&1u)!=0u;
                    const std::size_t along_rotation=(std::size_t)(robot.service_slot_offset%along_extra.size());
                    for(int side_order=0;side_order<2;++side_order){
                        const int side_index=flip_side?(1-side_order):side_order;
                        const Vec3 side=(side_index==0)?lateral:lateral*-1.0f;
                        for(float re:ring_extra){
                            const float ring=base_ring+re;
                            for(std::size_t ai=0;ai<along_extra.size();++ai){
                                const float ae=along_extra[(ai+along_rotation)%along_extra.size()];
                                Vec3 candidate=work_center+side*ring+tangent*(worker_along+ae);
                                candidate.y=first_person_floor_height(candidate);
                                const float face_heading=heading_between(candidate,work_center,robot.heading_degrees);
                                if(!robot_position_clear(robot_index,candidate)) continue;
                                if(!robot_structure_move_allowed(robot,candidate,candidate)) continue;
                                if(!fleet_hull_clear(robot_index,candidate,face_heading)) continue;
                                if(!conservative_fleet_bay_clear(candidate)) continue;
                                if(interaction_station_reserved(robot_index,target,candidate,face_heading)) continue;
                                const float score=dist_xz(robot.position,candidate)+
                                                  (side_order==0?0.0f:0.35f)+re*0.12f+std::abs(ae)*0.05f;
                                if(score<best_score){ best_score=score; best=candidate; found=true; }
                            }
                            if(found && side_order==0) break;
                        }
                    }
                    if(found) return best;
                    // construction_service_point performs a wider 32-slot body
                    // clearance search and is a safe last resort. Never return an
                    // unchecked linear work coordinate.
                    return construction_service_point(robot_index,target);
                }
            }
        }

        int sector=((phase%4)+4+(int)(ordinal%4u))%4;
        bool connector_sector_selected=false;
        // Build a structure from the side of an already-planned unfinished
        // corridor for the whole sequence. Waiting until the final stage is too
        // late: by then the almost-complete shell can already separate the rover
        // from that connector side. This does not dispatch BR-02 to the corridor;
        // it only keeps the current structure's work bay on the useful exterior
        // side. The corridor target itself remains ineligible until CR-01 has
        // physically delivered its complete bill of materials.
        if(rectangular){
            Vec3 connector{};
            bool found_connector=false;
            for(const auto& c:corridor_network_.connections()){
                if(c.construction.complete()) continue;
                for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                    const bool matches_habitat=target.kind==ConstructionTargetKind::Habitat &&
                        ep->kind==CorridorEndpointKind::Habitat && ep->index==target.index;
                    const bool matches_resource=target.kind==ConstructionTargetKind::Resource &&
                        ep->kind==CorridorEndpointKind::Resource && ep->index==target.index;
                    if(!matches_habitat && !matches_resource) continue;
                    connector=ep->position;
                    found_connector=true;
                    break;
                }
                if(found_connector) break;
            }
            if(found_connector){
                const Vec3 local=world_math::rotate_y(connector-center,-rotation);
                if(std::abs(local.x)>=std::abs(local.z)) sector=local.x>=0.0f?1:3;
                else sector=local.z>=0.0f?0:2;
                connector_sector_selected=true;
                // A connector-facing bay is preferred, not mandatory. If that
                // side is unreachable, the watchdog's persistent service-slot
                // offset rotates to another structure side on the next attempt
                // instead of selecting the identical NO ROUTE bay forever.
                if(robot.service_slot_offset>0){
                    sector=(sector+(int)(robot.service_slot_offset%4u))%4;
                    connector_sector_selected=false;
                }
            }
        }
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
            // Keep the normal construction bay outside the *next-stage*
            // evacuation envelope even when the rover stops within the arrival
            // tolerance. The old 0.22 m margin left only ~6 cm of theoretical
            // clearance on the Air Lock, so every stage unnecessarily entered
            // safety evacuation and could replan itself into a stall.
            ring=std::min(tx,tz)+hull.half_length+0.88f;
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
        }else if(rectangular && connector_sector_selected){
            // Keep the constructor on the connector side, but never place its
            // work bay on top of the CR-01 material apron at the side centre.
            // Shift toward the nearer corner while retaining full wall access.
            const bool east_west=(sector%2)==1;
            const float side_length=east_west?footprint_z:footprint_x;
            const float limit=std::max(0.0f,side_length*0.5f-1.15f);
            const float offset=std::min(2.55f,limit);
            const float side_dot=(robot.position.x-center.x)*tangent.x+(robot.position.z-center.z)*tangent.z;
            along=(side_dot>=0.0f?1.0f:-1.0f)*offset;
        }
        Vec3 p=center+outward*ring+tangent*along;
        p.y=first_person_floor_height(p);
        return p;
    };


    auto construction_service_candidate_clear=[&](std::size_t robot_index,Vec3 from,Vec3 candidate){
        if(robot_index>=builder_robots_.size()) return false;
        auto& robot=builder_robots_[robot_index];
        candidate.y=first_person_floor_height(candidate);
        // Validate the complete chassis at the candidate against terrain/map
        // bounds, erected structures, partial construction, resource piles and
        // the rest of the fleet. Requiring a direct body-safe first leg keeps a
        // CR-01 from launching an expensive full-map A* search toward a service
        // bay that its hull can never physically enter.
        if(!robot_position_clear(robot_index,candidate)) return false;
        if(!robot_structure_move_allowed(robot,candidate,candidate)) return false;
        if(!robot_structure_move_allowed(robot,from,candidate)) return false;
        const float heading=heading_between(from,candidate,robot.heading_degrees);
        if(!fleet_hull_clear(robot_index,candidate,heading)) return false;
        return true;
    };

    auto construction_drop_staging_point=[&](std::size_t robot_index,const ConstructionTargetRef& target,std::size_t rank){
        const Vec3 drop=construction_drop_position(target);
        Vec3 center=construction_target_position(target);
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()) center=placed_habitats_[target.index].position;
        else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()) center=placed_resources_[target.index].position;
        Vec3 outward=drop-center;
        float len=std::sqrt(outward.x*outward.x+outward.z*outward.z);
        if(len<0.1f){ outward={1.0f,0.0f,0.0f}; len=1.0f; }
        outward=outward*(1.0f/len);
        const float base_angle=std::atan2(outward.z,outward.x);
        const float base_distance=dist_xz(drop,center);
        const float desired_radius=1.55f+1.30f*(float)std::max<std::size_t>(1,rank);
        constexpr std::array<float,9> kAngleOffsets{{0.0f,0.38f,-0.38f,0.78f,-0.78f,1.18f,-1.18f,1.57f,-1.57f}};
        constexpr std::array<float,4> kRadiusExtra{{0.0f,0.65f,1.35f,2.15f}};
        float best_score=std::numeric_limits<float>::max();
        Vec3 best{};
        bool found=false;
        for(float extra:kRadiusExtra){
            const float radius=desired_radius+extra;
            for(float off:kAngleOffsets){
                const float angle=base_angle+off;
                Vec3 candidate=drop+Vec3{std::cos(angle),0.0f,std::sin(angle)}*radius;
                candidate.y=first_person_floor_height(candidate);
                // Queue outside the serviced construction rather than between
                // the drop apron and the blueprint/structure body.
                if(dist_xz(candidate,center)<base_distance+0.40f) continue;
                if(!construction_service_candidate_clear(robot_index,builder_robots_[robot_index].position,candidate)) continue;
                const float score=std::abs(off)*0.55f+extra*0.20f+dist_xz(candidate,drop)*0.01f;
                if(score<best_score){ best_score=score; best=candidate; found=true; }
            }
            if(found) break;
        }
        // If every queue bay is temporarily occupied, holding the current
        // position is safer than pathfinding to an impossible fixed point. The
        // queue is recomputed once fleet geometry changes.
        if(!found) return builder_robots_[robot_index].position;
        return best;
    };

    auto construction_drop_exit_point=[&](std::size_t robot_index,const ConstructionTargetRef& target,const Vec3& serviced_drop){
        // Use the apron where this load was actually unloaded.  The preferred
        // construction_drop_position() may move after site_buffer changes (the
        // newly staged boxes become obstacles), but the carrier must escape from
        // its real current apron rather than chasing that newly selected point.
        const Vec3 drop=serviced_drop;
        Vec3 center=construction_target_position(target);
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()) center=placed_habitats_[target.index].position;
        else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()) center=placed_resources_[target.index].position;
        Vec3 outward=drop-center;
        float len=std::sqrt(outward.x*outward.x+outward.z*outward.z);
        if(len<0.1f){ outward={1.0f,0.0f,0.0f}; len=1.0f; }
        outward=outward*(1.0f/len);
        // Prefer leaving toward the starter depot/source side. Following only
        // the apron-outward normal could put CR-01 on the far side of a corridor
        // just before BR-02 completed it, permanently separating the carrier
        // from the remaining starter pallets.
        Vec3 depot_dir=starter_stash_center_-drop;
        depot_dir.y=0.0f;
        float depot_len=std::sqrt(depot_dir.x*depot_dir.x+depot_dir.z*depot_dir.z);
        if(depot_len>0.10f) depot_dir=depot_dir*(1.0f/depot_len); else depot_dir=outward;
        const float base_angle=std::atan2(depot_dir.z,depot_dir.x);
        const float base_distance=dist_xz(drop,center);
        constexpr std::array<float,17> kAngleOffsets{{0.0f,0.24f,-0.24f,0.48f,-0.48f,0.74f,-0.74f,1.02f,-1.02f,1.30f,-1.30f,1.57f,-1.57f,1.88f,-1.88f,2.20f,-2.20f}};
        constexpr std::array<float,5> kRadii{{1.65f,2.15f,2.75f,3.45f,4.25f}};
        float best_score=std::numeric_limits<float>::max();
        Vec3 best{};
        bool found=false;
        const Vec3 from=builder_robots_[robot_index].position;
        for(float radius:kRadii){
            for(float off:kAngleOffsets){
                const float angle=base_angle+off;
                Vec3 candidate=drop+Vec3{std::cos(angle),0.0f,std::sin(angle)}*radius;
                candidate.y=first_person_floor_height(candidate);
                // A departure bay must actually increase clearance from the
                // serviced blueprint. This keeps the outgoing chassis from
                // being steered back through a just-completed dome/corridor.
                if(dist_xz(candidate,center)<base_distance+0.45f) continue;
                if(!construction_service_candidate_clear(robot_index,from,candidate)) continue;
                const float score=std::abs(off)*0.45f+radius*0.04f;
                if(score<best_score){ best_score=score; best=candidate; found=true; }
            }
            if(found) break;
        }
        if(!found){
            // Never start an A* search to an unverified fallback. Staying on the
            // already valid drop apron lets the transaction finish and avoids a
            // game-thread freeze; the builder cannot be assigned until the
            // complete site inventory is delivered, so there is no constructor
            // competing for this apron at this point.
            return from;
        }
        return best;
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
                         robot_resource_clearance_radius(robot.role)+0.92f;
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
               std::abs(focus.z-starter_stash_center_.z)<kStarterPalletHalfSpanM;
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
    auto starter_depot_safe_outer_gate=[&](std::size_t robot_index,Vec3 pallet){
        const Vec3 inner=starter_depot_entry(pallet);
        Vec3 outer=starter_depot_outer_gate(pallet);
        if(robot_index>=builder_robots_.size()) return inner;
        const auto& rover=builder_robots_[robot_index];
        // A player may build a large structure beside the starter depot after
        // the colony begins. The old fixed outer spine then becomes a legal
        // coordinate for the map, but an impossible coordinate for the rover's
        // full chassis. In 0-40-21.gmsave a completed solar panel occupies that
        // exact spine and CR-01 can never fetch the final 5 kg of silicon.
        // Fall back to the still-clear inner lane mouth; A* can route around the
        // structure and enter the pallet lane from there.
        if(robot_position_clear(robot_index,outer) &&
           robot_structure_move_allowed(rover,outer,outer)) return outer;
        return inner;
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
        const float dx=std::abs(robot.position.x-starter_stash_center_.x);
        const float dz=std::abs(robot.position.z-starter_stash_center_.z);
        const bool outside_depot=dx>=4.30f || dz>=7.10f;
        if(!robot.starter_exit_pending){
            // A loaded rover which is already clear of the pallet field must
            // continue toward its construction target instead of being pulled
            // back through the depot.
            if(outside_depot){
                robot.starter_exit_completed=true;
                return false;
            }
            robot.starter_exit_pending=true;
        }
        if(robot.starter_exit_pending && outside_depot){
            robot.starter_exit_pending=false;
            robot.starter_exit_completed=true;
            return false;
        }
        return robot.starter_exit_pending;
    };
    auto starter_depot_exit=[&](std::size_t robot_index){
        if(robot_index>=builder_robots_.size()) return starter_stash_center_;
        const auto& robot=builder_robots_[robot_index];
        const float half=(float)playfield_.size()*0.5f-
            (robot_resource_clearance_radius(robot.role)+kRobotObstacleMargin+0.12f);
        // The starter depot used to force every loaded rover through one east
        // gate. A completed structure can permanently block that gate (the
        // 0-40-15 Storage Dome regression), leaving CR-01 trapped beside the
        // pallet rows. Choose the first complete-hull-safe direct egress instead.
        std::array<Vec3,4> candidates{{
            {starter_stash_center_.x+4.65f,0.0f,robot.position.z},
            {robot.position.x,0.0f,starter_stash_center_.z+7.35f},
            {robot.position.x,0.0f,starter_stash_center_.z-7.35f},
            {starter_stash_center_.x-4.65f,0.0f,robot.position.z}
        }};
        for(auto& candidate:candidates){
            candidate.x=std::clamp(candidate.x,-half,half);
            candidate.z=std::clamp(candidate.z,-half,half);
            candidate.y=first_person_floor_height(candidate);
            if(!robot_position_clear(robot_index,candidate)) continue;
            if(!robot_structure_move_allowed(robot,robot.position,candidate)) continue;
            if(!safety_segment_clear(robot_index,robot.position,candidate)) continue;
            return candidate;
        }
        // Keep the rover stationary rather than launching A* toward a known
        // invalid fixed gate. The next simulation slice retries after fleet
        // geometry changes; full route recovery remains available for its
        // actual construction destination once it is outside the depot.
        return robot.position;
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


    // v0.40.30 deliberately performs no pose-repair teleport or ad-hoc escape
    // movement. If a loaded robot already intersects authoritative geometry, the
    // safety coordinator treats that as an unsafe initial state and holds/faults
    // it. New construction stages are prevented from creating this state by the
    // scene-change contract gate above.
    std::vector<bool> completed_structure_relocated(builder_robots_.size(),false);
    std::vector<bool> body_escape_repositioning(builder_robots_.size(),false);

    // Constructors are construction-only. CR-01 owns all shipping. A BR-02 is
    // dispatched once the NEXT construction stage is physically staged at the
    // blueprint, then releases the site again if the following stage is not yet
    // supplied. This keeps construction moving without making builders haul or
    // wait for the entire project inventory to arrive first.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.role!=RobotRole::Constructor) continue;
        // Safety evacuation has exclusive movement control, including for the
        // coordinating constructor itself. Previously a pending coordinator
        // could be moved by the pre-pass and then immediately braked by its
        // normal Building branch in the same tick, freezing the stage forever.
        if(safety_repositioning[robot_index] || body_escape_repositioning[robot_index] ||
           completed_structure_relocated[robot_index]) continue;

        // Role separation from v0.40.16 onward: a constructor never transports
        // construction stock. Legacy saves can still contain an in-flight BR-02
        // logistics state; put that cargo back into the world and release the
        // reservation so a CR-01 can finish the site cleanly.
        if(robot.mode==BuilderRobotMode::Fetching || robot.mode==BuilderRobotMode::Delivering ||
           robot.mode==BuilderRobotMode::DepotTransfer){
            if(!robot.cargo.empty()){
                const ResourceBundle spilled=spill_bundle_to_ground(robot.cargo,robot.position,nullptr);
                subtract_bundle(robot.cargo,spilled);
            }
            robot.logistics_reservation={};
            robot.cargo_committed_to_build=false;
            robot.target={};
            robot.support_robot_index=kNoRobotIndex;
            robot.site_buffer={};
            robot.stage_materials_picked=false;
            robot.interaction_position_valid=false;
            clear_action(robot);
            clear_route(robot);
            robot.mode=BuilderRobotMode::Idle;
        }

        // Existing saves may already have a BR-02 assigned to a blueprint that
        // is only partly supplied. Keep the robot exactly where it is and drop
        // the assignment; it becomes eligible again only after the carrier has
        // physically delivered the whole remaining plan. A builder already in
        // the middle of a stage may finish that stage before this gate applies.
        if(robot.target.valid() && !construction_target_complete(robot.target) &&
           robot.mode!=BuilderRobotMode::Building && !construction_stage_supplied(robot.target)){
            robot.target={};
            robot.support_robot_index=kNoRobotIndex;
            robot.logistics_reservation={};
            robot.cargo_committed_to_build=false;
            robot.stage_materials_picked=false;
            robot.interaction_position_valid=false;
            clear_action(robot);
            clear_route(robot);
            robot.idle_elapsed=0.0f;
            robot.mode=BuilderRobotMode::Idle;
        }

        auto choose_target=[&](){
            ConstructionTargetRef best{};
            float best_score=std::numeric_limits<float>::max();
            auto consider=[&](ConstructionTargetKind kind,std::size_t index){
                ConstructionTargetRef candidate{kind,index};
                if(construction_target_complete(candidate) || !construction_target_dependencies_ready(candidate)) return;
                if(!construction_stage_supplied(candidate) || !construction_site_logistics_clear(candidate)) return;
                // Untouched blueprints remain traversable. BR-02 may claim a
                // supplied target while another rover is crossing it; the
                // pre-build gate clears the footprint only when work is actually
                // about to begin.
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
                // Once endpoint structures are complete, finish their network
                // connection before sending BR-02 to unrelated later structures.
                // This makes placed corridors an actual construction commitment
                // rather than an indefinitely deferred low-priority blueprint.
                float network_priority=0.0f;
                if(kind==ConstructionTargetKind::Corridor) network_priority=-600.0f;
                else if(kind==ConstructionTargetKind::Utility || kind==ConstructionTargetKind::Walkway) network_priority=-350.0f;
                const float score=distance+load_penalty-cooperation_bonus+staffed_site_penalty+network_priority;
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
            robot.stage_commit_safe_position_valid=false;
            robot.stage_commit_alternate_attempted=false;
            robot.stage_materials_picked=false;
            robot.build_orbit_phase=0.0f;
            robot.build_side_corner_reached=false;
            clear_action(robot);
            clear_route(robot);
            robot.mode=robot.target.valid()?BuilderRobotMode::AwaitingMaterials:BuilderRobotMode::Idle;
        }
        if(!robot.target.valid()){
            robot.idle_elapsed+=dt;
            clear_action(robot);
            if(robot.interaction_position_valid) clear_interaction_station(robot);

            // A BR-02 with no executable construction stage must never keep a
            // structure work bay occupied while CR-01 is trying to stage the
            // missing resources. Return it to its own parking bay beside the
            // starter stash. The same home behavior applies when the entire
            // construction queue is complete. As soon as a supplied build task
            // becomes available on a later tick, choose_target() preempts this
            // cosmetic return trip immediately.
            const Vec3 home=constructor_parking_position(robot_index);
            if(dist_xz(robot.position,home)>0.30f || std::abs(robot.current_speed_mps)>0.04f){
                move_towards(robot_index,home,0.22f,0.90f);
            }else{
                clear_route(robot);
            }
            continue;
        }

        const auto* plan=construction_target_plan(robot.target);
        if(!plan){ robot.target={}; robot.support_robot_index=kNoRobotIndex; robot.logistics_reservation={}; robot.stage_commit_pending=false; robot.stage_commit_safe_position={}; robot.stage_commit_safe_position_valid=false; robot.stage_commit_alternate_attempted=false; robot.mode=BuilderRobotMode::Idle; continue; }
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
            // This state is a stage handoff. The next stage bundle must already
            // be present before the constructor starts work. If it is not yet
            // staged, release the site and let CR-01 continue shipping; BR-02
            // never self-hauls construction stock.
            if(!construction_stage_supplied(robot.target)){
                robot.target={};
                robot.support_robot_index=kNoRobotIndex;
                robot.logistics_reservation={};
                robot.cargo_committed_to_build=false;
                robot.stage_materials_picked=false;
                robot.idle_elapsed=0.0f;
                clear_action(robot);
                clear_interaction_station(robot);
                clear_route(robot);
                robot.mode=BuilderRobotMode::Idle;
                continue;
            }
            if(contains_bundle(*shared_site_buffer,step_cost)){
                // Re-check immediately before the first work action. A rover may
                // have crossed the empty blueprint after target assignment but
                // before BR-02 reached its bay. Keep waiting; the pre-build
                // evacuation pass above will move that blocker out next tick.
                if(construction_target_step(robot.target)==0 &&
                   !construction_blueprint_fleet_clear(robot.target)){
                    continue;
                }
                clear_logistics_assignment(robot);
                robot.idle_elapsed=0.0f;
                site_owner.stage_materials_picked=true;
                clear_action(robot);
                clear_interaction_station(robot);
                robot.mode=BuilderRobotMode::Building;
                robot.work_elapsed=0.0f;
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
                            robot.blocked_elapsed=0.0f; robot.watchdog_elapsed=0.0f;
                        }
                        const std::size_t rank=depot_admitted?starter_lane_queue_rank(robot_index,robot.interaction_material):1u;
                        const bool occupied=depot_admitted && starter_lane_occupied(robot_index,robot.interaction_material,robot.interaction_focus_position);
                        Vec3 queue_gate=starter_depot_outer_gate(robot.interaction_focus_position);
                        const std::size_t hold_rank=std::max<std::size_t>(rank,occupied?1u:0u);
                        queue_gate.x+=(float)hold_rank*3.25f;
                        const Vec3 outer_gate=starter_depot_safe_outer_gate(robot_index,robot.interaction_focus_position);
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
                const Vec3 gate=starter_depot_exit(robot_index);
                move_towards(robot_index,gate,0.20f,0.90f);
            }else if(!may_enter_drop){
                clear_action(robot);
                if(dist_xz(robot.position,delivery_goal)>0.24f)
                    move_towards(robot_index,delivery_goal,0.16f,0.80f);
                else{ clear_route(robot); }
            }else if(dist_xz(robot.position,drop_zone)>kDeliveryWorkZoneM){
                clear_action(robot);
                move_towards(robot_index,drop_zone,kDeliveryWorkZoneM*0.58f,0.88f);
            }else if(!robot.cargo.empty()){
                clear_route(robot);
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
            // Do not cancel the BR-02 navigation request merely because it is
            // already moving.  v0.40.34 did that at the top of BUILDING, so
            // every frame alternated between one acceleration pulse and a
            // certified brake.  The rover therefore crawled at ~0.1 m/s and
            // could never reach its work bay.  Travel is allowed here; actual
            // welding/build-up time is still gated by tick_action() once the
            // rover has reached the bay and the safety coordinator has stopped it.
            robot.work_elapsed+=dt;

            if(robot.stage_commit_pending){
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
                robot.stage_commit_safe_position_valid=false;
                robot.stage_commit_alternate_attempted=false;
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

            // Do not rotate a BR-02 to a different structure side merely because
            // the logical build stage advanced. The growing footprint can turn
            // that cosmetic side change into a mandatory corner traversal, and
            // the exact v0.40.14 save demonstrates how the planner/chassis
            // boundary mismatch can then repeat forever. Choose the nearest
            // exterior side when a stage begins and latch that work point for
            // the duration of the stage. After the stage commit the next stage
            // re-evaluates from the rover's current position, which naturally
            // keeps it on the same safe side unless another side is genuinely
            // closer. Multiple constructors remain distributed because
            // construction_work_point() already offsets by constructor ordinal.
            int work_sector=phase;
            if(robot.target.kind==ConstructionTargetKind::Habitat ||
               robot.target.kind==ConstructionTargetKind::Resource){
                const Vec3 selector=robot.interaction_position_valid?robot.interaction_position:robot.position;
                float best_clear_distance=std::numeric_limits<float>::max();
                float best_any_distance=std::numeric_limits<float>::max();
                int best_clear_sector=-1;
                int best_any_sector=work_sector;
                for(int sector=0;sector<4;++sector){
                    const Vec3 candidate=construction_work_point(robot_index,robot.target,sector);
                    const float candidate_distance=dist_xz(selector,candidate);
                    if(candidate_distance<best_any_distance){
                        best_any_distance=candidate_distance;
                        best_any_sector=sector;
                    }
                    const float face_heading=heading_between(candidate,center,robot.heading_degrees);
                    const bool bay_clear=robot_position_clear(robot_index,candidate) &&
                        robot_structure_move_allowed(robot,candidate,candidate) &&
                        fleet_hull_clear(robot_index,candidate,face_heading);
                    if(bay_clear && candidate_distance<best_clear_distance){
                        best_clear_distance=candidate_distance;
                        best_clear_sector=sector;
                    }
                }
                work_sector=best_clear_sector>=0?best_clear_sector:best_any_sector;
            }
            Vec3 work_position=construction_work_point(robot_index,robot.target,work_sector);
            // One physical work bay is latched for the complete stage. Linear
            // projects choose their nearest lateral side from the rover pose;
            // recomputing that every frame lets the sign flip after crossing the
            // centreline and makes BR-02 chase alternating sides forever. The
            // interaction station is cleared at every stage commit, so the next
            // stage still gets a fresh geometry-aware bay.
            if(robot.interaction_position_valid){
                work_position=robot.interaction_position;
            }else{
                robot.interaction_position=work_position;
                clear_route(robot);
            }
            robot.interaction_focus_position=center;
            robot.interaction_position_valid=true;

            // The coarse geometry route only supplies a waypoint hint. The shared
            // safety coordinator certifies each short motion contract; job logic
            // does not authorize or collision-check movement.
            if(dist_xz(robot.position,work_position)>kConstructionBayArrivalM){
                clear_action(robot);
                move_towards(robot_index,work_position,0.16f,kBuildTravelSpeedScale);
                continue;
            }
            clear_route(robot);

            // Save/load can restore a constructor already in BUILDING mode while
            // the blueprint is still at exactly 0%.  Such a state bypasses the
            // AwaitingMaterials -> Building admission check above.  Re-run the
            // scene-mutation gate at the first physical welding operation so no
            // visible progress is published while another rover's ACTIVE
            // motion/braking contract still crosses the future structure.
            if(construction_target_step(robot.target)==0 &&
               !construction_target_started(robot.target) &&
               !construction_blueprint_fleet_clear(robot.target)){
                clear_action(robot);
                continue;
            }

            // One work position per stage keeps construction visibly distributed around
            // the footprint while eliminating the opposite-side traversal that
            // repeatedly stranded BR-02s on partial-mesh corners.
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
    auto any_unfinished_blueprint=[&](){
        for(const auto& h:placed_habitats_) if(!h.construction.complete()) return true;
        for(const auto& r:placed_resources_) if(!r.construction.complete()) return true;
        for(const auto& c:corridor_network_.connections()) if(!c.construction.complete()) return true;
        for(const auto& u:utility_network_.connections()) if(!u.construction.complete()) return true;
        for(const auto& w:walkway_network_.connections()) if(!w.construction.complete()) return true;
        return false;
    };

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
            if(!carrier.target.valid() || construction_target_complete(carrier.target)) return false;
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

        // CR-01 logistics is strict FIFO by blueprint placement order. The
        // fleet fully stages the complete remaining bill of materials for the
        // oldest blueprint before any carrier is allowed to advance to a newer
        // one. Construction dependencies do not alter this logistics order.
        auto placement_sequence=[&](const ConstructionTargetRef& target)->std::uint64_t{
            switch(target.kind){
                case ConstructionTargetKind::Habitat:
                    return target.index<placed_habitats_.size()?placed_habitats_[target.index].placement_sequence:0;
                case ConstructionTargetKind::Resource:
                    return target.index<placed_resources_.size()?placed_resources_[target.index].placement_sequence:0;
                case ConstructionTargetKind::Corridor:
                    return target.index<corridor_network_.connections().size()?corridor_network_.connections()[target.index].placement_sequence:0;
                case ConstructionTargetKind::Utility:
                    return target.index<utility_network_.connections().size()?utility_network_.connections()[target.index].placement_sequence:0;
                case ConstructionTargetKind::Walkway:
                    return target.index<walkway_network_.connections().size()?walkway_network_.connections()[target.index].placement_sequence:0;
                default: return 0;
            }
        };

        auto fifo_front_target=[&](){
            ConstructionTargetRef best{};
            std::uint64_t best_sequence=std::numeric_limits<std::uint64_t>::max();
            std::uint64_t fallback_order=0;
            std::uint64_t best_fallback=std::numeric_limits<std::uint64_t>::max();

            auto consider=[&](const ConstructionTargetRef& target){
                if(!target.valid() || construction_target_complete(target)) return;
                // Advance the queue only after the complete REMAINING plan is
                // physically staged at this blueprint. Fleet reservations do
                // not count as supplied until the pallet actually arrives.
                if(logistics_remaining(target,true,false,kNoRobotIndex).empty()) return;
                const std::uint64_t sequence=placement_sequence(target);
                const std::uint64_t fallback=++fallback_order;
                if(sequence>0){
                    if(sequence<best_sequence){ best_sequence=sequence; best_fallback=fallback; best=target; }
                }else if(best_sequence==std::numeric_limits<std::uint64_t>::max() && fallback<best_fallback){
                    // Defensive support for a transient/legacy object without a
                    // sequence. Save migration normally guarantees non-zero IDs.
                    best_fallback=fallback;
                    best=target;
                }
            };

            for(std::size_t i=0;i<placed_habitats_.size();++i) consider({ConstructionTargetKind::Habitat,i});
            for(std::size_t i=0;i<placed_resources_.size();++i) consider({ConstructionTargetKind::Resource,i});
            for(std::size_t i=0;i<corridor_network_.connections().size();++i) consider({ConstructionTargetKind::Corridor,i});
            for(std::size_t i=0;i<utility_network_.connections().size();++i) consider({ConstructionTargetKind::Utility,i});
            for(std::size_t i=0;i<walkway_network_.connections().size();++i) consider({ConstructionTargetKind::Walkway,i});
            return best;
        };

        auto choose_support=[&](bool loaded_only){
            const ConstructionTargetRef target=fifo_front_target();
            if(!target.valid()) return ConstructionTargetRef{};

            // Other carriers may already have reserved the rest of this FIFO
            // target. In that case this rover waits; it must not jump ahead to
            // a newer blueprint before the first one is physically full.
            const ResourceBundle need=logistics_remaining(target,true,true,robot_index);
            if(need.empty() || !site_serviceable(need,loaded_only)) return ConstructionTargetRef{};
            if(!loaded_only){
                const SourceChoice source=choose_source(robot_index,need);
                if(!source.valid) return ConstructionTargetRef{};
            }
            return target;
        };

        // Enforce the FIFO queue immediately for an empty carrier, including
        // carriers restored from a pre-schema-13 save with an older target. An
        // already loaded rover finishes/reassigns its physical cargo normally,
        // but an empty PICK-UP transaction may be cancelled without material
        // loss and restarted at the true queue front.
        const ConstructionTargetRef queue_front=fifo_front_target();
        if(carrier.cargo.empty() && carrier.target.valid() && queue_front.valid() &&
           !carrier.target.same_as(queue_front)){
            clear_logistics_assignment(carrier);
            carrier.target={};
            carrier.support_robot_index=kNoRobotIndex;
            carrier.mode=BuilderRobotMode::Idle;
            carrier.drop_exit_pending=false;
            carrier.post_delivery_hold=false;
            clear_action(carrier);
            clear_interaction_station(carrier);
            clear_route(carrier);
        }

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
                // Do not start cosmetic/recovery pile housekeeping while any
                // placed construction is still active. During a fully stocked
                // corridor build there is intentionally no logistics demand;
                // CR-01 should park instead of launching unrelated A* searches
                // on the main simulation thread. Ground-pile recovery resumes
                // once the current construction queue is complete.
                carrier.mode=any_unfinished_blueprint()?BuilderRobotMode::Idle:BuilderRobotMode::DepotTransfer;
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
                            carrier.blocked_elapsed=0.0f; carrier.watchdog_elapsed=0.0f;
                        }
                        const std::size_t rank=depot_admitted?starter_lane_queue_rank(robot_index,carrier.interaction_material):1u;
                        const bool occupied=depot_admitted && starter_lane_occupied(robot_index,carrier.interaction_material,carrier.interaction_focus_position);
                        Vec3 queue_gate=starter_depot_outer_gate(carrier.interaction_focus_position);
                        const std::size_t hold_rank=std::max<std::size_t>(rank,occupied?1u:0u);
                        queue_gate.x+=(float)hold_rank*3.25f;
                        const Vec3 outer_gate=starter_depot_safe_outer_gate(robot_index,carrier.interaction_focus_position);
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
                // interaction_focus_position is the exact apron used by the
                // just-completed unload.  Keep that point stable while exiting;
                // construction_drop_position() can legitimately relocate as the
                // site buffer gains new visible packages.
                const Vec3 drop=carrier.interaction_position_valid
                                  ?carrier.interaction_focus_position
                                  :construction_drop_position(carrier.target);
                Vec3 departure=carrier.interaction_position;
                const bool latched_departure=carrier.interaction_position_valid &&
                                             dist_xz(carrier.interaction_position,drop)>0.55f;
                if(!latched_departure) departure=construction_drop_exit_point(robot_index,carrier.target,drop);
                clear_action(carrier);
                carrier.interaction_focus_position=drop;
                carrier.interaction_position=departure;
                carrier.interaction_position_valid=true;
                if(dist_xz(carrier.position,departure)>0.28f){
                    move_construction_service_direct(robot_index,departure,0.18f,0.88f);
                }else{
                    clear_route(carrier);
                    carrier.drop_exit_pending=false;
                    carrier.post_delivery_hold=true;
                    clear_interaction_station(carrier);
                    carrier.blocked_elapsed=0.0f;
                    carrier.watchdog_elapsed=0.0f;
                    carrier.recovery_attempts=0;

                    // Re-evaluate the FIFO queue after every completed trip.
                    // If this blueprint still lacks any material from its entire
                    // remaining plan, choose_support() selects it again. Only a
                    // physically fully staged blueprint lets CR-01 advance to
                    // the next object in placement order.
                    carrier.logistics_reservation={};
                    carrier.cargo_committed_to_build=!carrier.cargo.empty();
                    carrier.target={};
                    carrier.support_robot_index=kNoRobotIndex;
                    if(carrier.cargo.empty() && assign_empty_carrier()){
                        carrier.post_delivery_hold=false;
                    }else{
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
                    const Vec3 gate=starter_depot_exit(robot_index);
                    move_towards(robot_index,gate,0.20f,0.95f);
                }else if(!may_enter_drop){
                    clear_action(carrier);
                    if(dist_xz(carrier.position,goal)>0.24f)
                        move_towards(robot_index,goal,0.16f,0.82f);
                    else { clear_route(carrier); }
                }else if(dist_xz(carrier.position,drop)>kDeliveryWorkZoneM){
                    clear_action(carrier);
                    move_towards(robot_index,drop,kDeliveryWorkZoneM*0.55f,0.92f);
                }else if(!carrier.cargo.empty()){
                    clear_route(carrier);
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

    // Idle CR-01s do not spend simulation time pathfinding toward a cosmetic
    // global parking bay. Every completed delivery already has a protected
    // drop-exit phase which moves the chassis clear of the shared apron. Once
    // that exit is complete the carrier can remain where it is until new work
    // arrives. Idle carriers never move merely to clear traffic. Active rovers
    // treat them as stationary fleet obstacles and route around them or wait.
    // This also avoids repeated full-map A* searches for an otherwise idle robot
    // (the 0.40.13 Bio Dome FPS stall).
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.role!=RobotRole::Carrier || robot.mode!=BuilderRobotMode::Idle ||
           !robot.cargo.empty() || robot.target.valid()) continue;
        clear_action(robot);
        clear_interaction_station(robot);
        clear_route(robot);
        robot.route_resolution_elapsed=0.0f;
        robot.route_force_full_replan=false;
        robot.route_replan_cooldown=0.0f;
        robot.blocked_elapsed=0.0f;
        robot.post_delivery_hold=true;
    }

    // Shared safety-authority phase. All job/evacuation logic above has only
    // submitted intents. Build one authoritative fleet snapshot, arbitrate all
    // contracts against the same reservation board and commit the verified
    // motions simultaneously.
    auto mix_epoch=[](std::uint64_t& h,std::uint64_t v){
        h^=v+0x9e3779b97f4a7c15ULL+(h<<6)+(h>>2);
    };
    std::uint64_t world_epoch=1469598103934665603ULL;
    mix_epoch(world_epoch,placed_habitats_.size());
    mix_epoch(world_epoch,placed_resources_.size());
    mix_epoch(world_epoch,corridor_network_.connections().size());
    mix_epoch(world_epoch,utility_network_.connections().size());
    mix_epoch(world_epoch,walkway_network_.connections().size());
    mix_epoch(world_epoch,ground_resource_piles_.size());
    for(const auto& h:placed_habitats_) mix_epoch(world_epoch,(std::uint64_t)h.construction.step);
    for(const auto& r:placed_resources_) mix_epoch(world_epoch,(std::uint64_t)r.construction.step);
    for(const auto& c:corridor_network_.connections()) mix_epoch(world_epoch,((std::uint64_t)c.construction.step<<8)|(std::uint64_t)c.construction.total_steps);
    for(const auto& c:utility_network_.connections()) mix_epoch(world_epoch,((std::uint64_t)c.construction.step<<8)|(std::uint64_t)c.construction.total_steps);
    for(const auto& c:walkway_network_.connections()) mix_epoch(world_epoch,((std::uint64_t)c.construction.step<<8)|(std::uint64_t)c.construction.total_steps);
    for(const auto& pile:ground_resource_piles_){
        mix_epoch(world_epoch,(std::uint64_t)std::llround((double)pile.position.x*4.0));
        mix_epoch(world_epoch,(std::uint64_t)std::llround((double)pile.position.z*4.0));
        mix_epoch(world_epoch,(std::uint64_t)std::max<std::int64_t>(0,pile.amount));
    }

    std::vector<robots::SafetyAgent> safety_agents;
    safety_agents.reserve(builder_robots_.size());
    for(std::size_t i=0;i<builder_robots_.size();++i){
        auto& robot=builder_robots_[i];
        navigation_requests[i].work_target=!navigation_requests[i].active &&
            (robot.action!=BuilderRobotAction::None || robot.mode==BuilderRobotMode::Building ||
             robot.mode==BuilderRobotMode::Fetching || robot.mode==BuilderRobotMode::Delivering ||
             robot.mode==BuilderRobotMode::DepotTransfer);
        // Constructors own the higher traffic priority. This never lets BR-02
        // preempt an already occupied certified passage, but when two fresh
        // movements compete for the same free space the builder is considered
        // first and the carrier yields/replans independently.
        // Construction has the normal high traffic priority, but an idle BR-02
        // returning to its stash-side parking bay is non-work traffic. Give that
        // return trip the lowest priority so CR-01 deliveries always win shared
        // passage arbitration while the builder is waiting for materials.
        if(robot.role==RobotRole::Constructor && robot.mode==BuilderRobotMode::Idle &&
           !robot.target.valid() && navigation_requests[i].active)
            navigation_requests[i].task_priority=0u;
        else
            navigation_requests[i].task_priority=robot.role==RobotRole::Constructor?2u:1u;
        robots::SafetyAgent agent{};
        agent.id=i;
        agent.position=robot.position;
        agent.heading_degrees=robot.heading_degrees;
        agent.speed_mps=robot.current_speed_mps;
        agent.radius_m=robot_safety_corridor_radius(robot.role);
        agent.max_speed_mps=robot.move_speed_mps;
        agent.acceleration_mps2=kRobotAccelerationMps2;
        agent.braking_mps2=kRobotBrakingMps2;
        agent.turn_rate_degrees_per_second=kRobotTurnRateDegreesPerSecond;
        agent.runtime=&robot.safety;
        safety_agents.push_back(agent);
    }

    robots::SafetyWorldCallbacks safety_world{};
    safety_world.floor_height=[&](Vec3 p){ return first_person_floor_height(p); };
    safety_world.point_clear=[&](std::size_t robot_index,Vec3 p){
        if(robot_index>=builder_robots_.size()) return false;
        const auto& robot=builder_robots_[robot_index];
        // During welding/build-up the BR-02 may sit at a prevalidated exterior
        // service bay that slightly overlaps the conservative envelope of its
        // OWN partially revealed target. That envelope is a routing aid, not
        // physical geometry. Ignore only that own partial target while the bot
        // is stopped at its latched work station; all other structures, boxes,
        // terrain and completed geometry remain hard blockers.
        const bool own_active_work_hold=robot.role==RobotRole::Constructor &&
            robot.mode==BuilderRobotMode::Building && robot.target.valid() &&
            robot.interaction_position_valid && !navigation_requests[robot_index].active &&
            dist_xz(robot.position,robot.interaction_position)<=0.55f &&
            dist_xz(p,robot.interaction_position)<=0.65f;
        if(own_active_work_hold)
            return robot_position_clear_impl(robot_index,p,&robot.target) &&
                   robot_structure_move_allowed(robot,p,p);
        return robot_position_clear(robot_index,p);
    };
    safety_world.segment_clear=[&](std::size_t robot_index,Vec3 a,Vec3 b){
        return safety_segment_clear(robot_index,a,b);
    };
    safety_world.coarse_route=[&](std::size_t robot_index,Vec3 a,Vec3 b,float stop_radius,std::vector<Vec3>& route){
        return coarse_safety_route(robot_index,a,b,stop_radius,route);
    };

    const auto safety_stats=robots::safety_tick(dt,robot_safety_time_seconds_,world_epoch,
                                                safety_agents,navigation_requests,safety_world);
    (void)safety_stats;
    robot_safety_time_seconds_+=std::max(0.0f,dt);

    for(std::size_t i=0;i<builder_robots_.size();++i){
        auto& robot=builder_robots_[i];
        const auto& agent=safety_agents[i];
        const auto& safety=robot.safety;
        robot.position=agent.position;
        robot.heading_degrees=agent.heading_degrees;
        robot.current_speed_mps=agent.speed_mps;
        robot.blocked_elapsed=safety.blocked_elapsed;
        robot.route_wait_elapsed=safety.wait_age;
        robot.route_resolution_elapsed=safety.blocked_elapsed;
        robot.route_full_replan_count=(std::uint16_t)std::min<std::uint32_t>(65535u,safety.local_replans);
        robot.traffic_blocker_index=safety.blocker_robot_id<builder_robots_.size()?safety.blocker_robot_id:kNoRobotIndex;
        robot.traffic_escape_active=safety.motion_intent==robots::MotionIntent::Retreat;
        robot.traffic_detour_active=safety.stationary_detour_active;
        robot.traffic_detour_avoid_index=safety.stationary_detour_blocker<builder_robots_.size()?safety.stationary_detour_blocker:kNoRobotIndex;
        robot.route_force_full_replan=false;
        robot.route_search_tier=0;
        robot.route_replan_cooldown=0.0f;
        robot.route_active=navigation_requests[i].active && (safety.goal_valid || safety.temporary_goal_valid) &&
            safety.mode!=robots::SafetyMode::Blocked && safety.mode!=robots::SafetyMode::Fault;
        robot.route_waiting_for_clearance=navigation_requests[i].active &&
            (safety.mode==robots::SafetyMode::Waiting || safety.mode==robots::SafetyMode::Braking);
        robot.route_destination=navigation_requests[i].active?navigation_requests[i].goal:robot.position;
        robot.route_stop_radius=navigation_requests[i].active?navigation_requests[i].stop_radius:0.0f;
        robot.route_speed_scale=navigation_requests[i].active?navigation_requests[i].speed_scale:1.0f;
        robot.route_cruise_speed_mps=robot.move_speed_mps*robot.route_speed_scale;

        // Debug route visualization shows only the current short proposal, not
        // the cached full coarse route. The latter is advisory and never a
        // movement authorization.
        robot.planned_route.clear();
        if(!safety.proposals.empty()) robot.planned_route=safety.proposals.front().preview;
        robot.planned_route_distances.clear();
        float distance=0.0f;
        Vec3 previous=robot.position;
        for(const Vec3 p:robot.planned_route){
            distance+=dist_xz(previous,p);
            robot.planned_route_distances.push_back(distance);
            previous=p;
        }
        robot.route_cursor=0;
        robot.route_progress_m=0.0f;
        robot.route_total_length_m=distance;
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
            if((robot.safety.mode==robots::SafetyMode::Blocked || robot.safety.block_reason==robots::BlockReason::NoRoute) &&
               robot.safety.blocked_elapsed>=kOperationWatchdogSeconds){
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
            // exact reservation while the safety coordinator invalidates the
            // advisory coarse route and retries certified local proposals.
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
            // station. The safety coordinator owns braking; recovery only
            // invalidates the advisory route and selects a new service slot.
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

    // The safety coordinator is the only fleet-overlap authority. An overlap
    // detected at initialization or by the final continuous sweep becomes a
    // safety fault; no post-step nudge is allowed to hide or repair it.

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

    // Aggregate into an existing pallet without changing its collision geometry.
    const float slot=(float)((std::size_t)resource%4);
    Vec3 output_position=source_position+Vec3{0.8f+slot*0.28f,0.0f,0.9f};
    output_position.y=first_person_floor_height(output_position);
    for(auto& pile:ground_resource_piles_){
        if(pile.resource==resource && dist_xz(pile.position,output_position)<0.45f){
            pile.amount+=amount;
            ResourceBundle add{}; add.amount(resource)=amount; resources_.add(add);
            return;
        }
    }

    // A new physical pallet is a scene mutation. Do not materialize it inside
    // any robot's normal motion, certified braking branch, or terminal hold.
    // Search deterministic nearby output pockets; if all are reserved, defer the
    // production quantity back into its fractional accumulator for a later tick.
    auto reservation_clear=[&](Vec3 p,float radius){
        for(const auto& robot:builder_robots_){
            const float robot_radius=robot_safety_corridor_radius(robot.role);
            if(robot.safety.active_contract.valid){
                if(robots::contract_intersects_region(robot.safety.active_contract,[&](Vec3 q,float reserved_radius){
                    return dist_xz(p,q)<=radius+reserved_radius+kRobotObstacleMargin;
                })) return false;
            }else if(dist_xz(p,robot.position)<=radius+robot_radius+kRobotObstacleMargin){
                return false;
            }
        }
        return true;
    };

    constexpr float pile_radius=0.50f;
    bool found=reservation_clear(output_position,pile_radius);
    Vec3 chosen=output_position;
    if(!found){
        constexpr std::array<float,4> rings{{1.0f,1.8f,2.8f,4.0f}};
        for(float ring:rings){
            for(int sample=0;sample<16 && !found;++sample){
                const float a=(2.0f*PI*(float)sample)/16.0f;
                Vec3 candidate=output_position+Vec3{std::sin(a)*ring,0.0f,std::cos(a)*ring};
                candidate.y=first_person_floor_height(candidate);
                if(reservation_clear(candidate,pile_radius)){ chosen=candidate; found=true; }
            }
            if(found) break;
        }
    }
    if(!found){
        production_fraction_[(std::size_t)resource]+=(double)amount;
        return;
    }

    GroundResourcePile pile;
    pile.resource=resource;
    pile.amount=amount;
    pile.position=chosen;
    pile.radius=pile_radius;
    ground_resource_piles_.push_back(pile);
    ResourceBundle add{}; add.amount(resource)=amount; resources_.add(add);
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
        // Aluminum is a major lunar structural feedstock in this game balance:
        // ore extraction + processing separates it from regolith alongside Fe/Si.
        rate_per_day[(size_t)MaterialResource::Aluminum]=5200.0*ore_drillers;
        source[(size_t)MaterialResource::Iron]=source[(size_t)MaterialResource::Copper]=source[(size_t)MaterialResource::Silicon]=source[(size_t)MaterialResource::Aluminum]=industrial;
        have_source[(size_t)MaterialResource::Iron]=have_source[(size_t)MaterialResource::Copper]=have_source[(size_t)MaterialResource::Silicon]=have_source[(size_t)MaterialResource::Aluminum]=true;
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
