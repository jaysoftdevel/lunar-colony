#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/util/world_math.hpp"
#include "game/world/construction_sequence.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
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
constexpr float kWeldSeconds=2.20f;
constexpr float kBuildUpSeconds=1.80f;
constexpr float kRobotAccelerationMps2=2.4f;
constexpr float kRobotBrakingMps2=3.2f;
constexpr float kRobotTurnRateDegreesPerSecond=125.0f;
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
    const float v0x=c.x-a.x,v0z=c.z-a.z;
    const float v1x=b.x-a.x,v1z=b.z-a.z;
    const float v2x=p.x-a.x,v2z=p.z-a.z;
    const float dot00=v0x*v0x+v0z*v0z;
    const float dot01=v0x*v1x+v0z*v1z;
    const float dot02=v0x*v2x+v0z*v2z;
    const float dot11=v1x*v1x+v1z*v1z;
    const float dot12=v1x*v2x+v1z*v2z;
    const float den=dot00*dot11-dot01*dot01;
    if(std::abs(den)<1.0e-8f) return false;
    const float inv=1.0f/den;
    const float u=(dot11*dot02-dot01*dot12)*inv;
    const float v=(dot00*dot12-dot01*dot02)*inv;
    return u>=0.0f && v>=0.0f && u+v<=1.0f;
}

bool construction_triangles_block_circle(const std::vector<Vec3>& tris,Vec3 candidate,float radius){
    for(std::size_t i=0;i+2<tris.size();i+=3){
        const Vec3 a=tris[i],b=tris[i+1],c=tris[i+2];
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

std::vector<Vec3> transformed_revealed(const std::vector<Vec3>& src,float fraction,Vec3 position,float rotation_degrees){
    const auto local=construction_sequence::reveal_triangles(src,fraction);
    std::vector<Vec3> out; out.reserve(local.size());
    for(const Vec3& v:local) out.push_back(world_math::rotate_y(v,rotation_degrees)+position);
    return out;
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
        const float reach=robot_body_radius(robot.role)+package_radius+kRobotObstacleMargin+0.42f;
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
        else pile.radius=resource_package_half_width(pile.resource,pile.amount);
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

const ResourceBundle* GameApp::construction_site_buffer(const ConstructionTargetRef& target) const {
    const std::size_t owner=construction_site_owner_index(target);
    return owner<builder_robots_.size()?&builder_robots_[owner].site_buffer:nullptr;
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
        const std::size_t owner=construction_site_owner_index(target);
        return owner<builder_robots_.size()?&builder_robots_[owner].site_buffer:nullptr;
    };


    auto partial_construction_blocks=[&](Vec3 candidate,float radius){
        for(const auto& h:placed_habitats_){
            if(h.construction.complete() || h.construction.step==0) continue;
            const auto& mesh=habitat_models_.mesh(h);
            const Vec3 local=world_math::rotate_y(candidate-h.position,-h.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+radius+0.8f || std::abs(local.z)>mesh.footprint_z*0.5f+radius+0.8f) continue;
            const auto v=construction_sequence::habitat_visibility(h.construction.step);
            if(construction_triangles_block_circle(transformed_revealed(mesh.foundation,v.foundation,h.position,h.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.shell,v.shell,h.position,h.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.interior,v.interior,h.position,h.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.details,v.details,h.position,h.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.glazing,v.glazing,h.position,h.rotation_degrees),candidate,radius)) return true;
        }
        for(const auto& item:placed_resources_){
            if(item.construction.complete() || item.construction.step==0) continue;
            const auto& mesh=resource_models_.mesh(item);
            const Vec3 local=world_math::rotate_y(candidate-item.position,-item.rotation_degrees);
            if(std::abs(local.x)>mesh.footprint_x*0.5f+radius+0.8f || std::abs(local.z)>mesh.footprint_z*0.5f+radius+0.8f) continue;
            const auto v=construction_sequence::resource_visibility(item.construction.step);
            if(construction_triangles_block_circle(transformed_revealed(mesh.foundation,v.foundation,item.position,item.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.body,v.body,item.position,item.rotation_degrees),candidate,radius) ||
               construction_triangles_block_circle(transformed_revealed(mesh.details,v.details,item.position,item.rotation_degrees),candidate,radius)) return true;
        }
        for(const auto& c:corridor_network_.connections()){
            if(c.construction.complete() || c.construction.step==0) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle(construction_sequence::reveal_triangles(c.mesh.shell,f),candidate,radius) ||
               construction_triangles_block_circle(construction_sequence::reveal_triangles(c.mesh.details,f),candidate,radius)) return true;
        }
        for(const auto& c:utility_network_.connections()){
            if(c.construction.complete() || c.construction.step==0) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle(construction_sequence::reveal_triangles(c.mesh.shell,f),candidate,radius) ||
               construction_triangles_block_circle(construction_sequence::reveal_triangles(c.mesh.details,f),candidate,radius)) return true;
        }
        for(const auto& c:walkway_network_.connections()){
            if(c.construction.complete() || c.construction.step==0) continue;
            const float f=c.construction.progress();
            if(construction_triangles_block_circle(construction_sequence::reveal_triangles(c.mesh.details,f),candidate,radius)) return true;
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
            auto before=transformed_revealed(src,from_fraction,position,rotation);
            auto after=transformed_revealed(src,to_fraction,position,rotation);
            if(after.size()<=before.size()) return false;
            const std::size_t first=(before.size()/3)*3;
            std::vector<Vec3> delta(after.begin()+(std::ptrdiff_t)first,after.end());
            return construction_triangles_block_circle(delta,candidate,radius);
        };
        auto new_world_blocks=[&](const std::vector<Vec3>& src,float from_fraction,float to_fraction){
            auto before=construction_sequence::reveal_triangles(src,from_fraction);
            auto after=construction_sequence::reveal_triangles(src,to_fraction);
            if(after.size()<=before.size()) return false;
            const std::size_t first=(before.size()/3)*3;
            std::vector<Vec3> delta(after.begin()+(std::ptrdiff_t)first,after.end());
            return construction_triangles_block_circle(delta,candidate,radius);
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
                    const float a=(float)current/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    const float b=(float)step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    return new_world_blocks(c.mesh.shell,a,b) || new_world_blocks(c.mesh.details,a,b);
                }
                break;
            case ConstructionTargetKind::Utility:
                if(target.index<utility_network_.connections().size()){
                    const auto& c=utility_network_.connections()[target.index];
                    const float a=(float)current/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    const float b=(float)step/(float)std::max<std::uint8_t>(1,c.construction.total_steps);
                    return new_world_blocks(c.mesh.shell,a,b) || new_world_blocks(c.mesh.details,a,b);
                }
                break;
            case ConstructionTargetKind::Walkway:
                if(target.index<walkway_network_.connections().size()){
                    const auto& c=walkway_network_.connections()[target.index];
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
            const auto current=construction_target_step(target);
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

        for(const auto& pile:ground_resource_piles_){
            const float package_radius=std::max(pile.radius,resource_package_half_width(pile.resource,pile.amount));
            if(dist_xz(candidate,pile.position)<body_radius+package_radius+kRobotObstacleMargin) return false;
        }
        for(const auto& structure:placed_resources_){
            if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(structure);
            for(std::size_t bi=0;bi<structure.stored_boxes.size();++bi){
                Vec3 base{}; float half=0.0f,height=0.0f;
                stored_resource_box_pose(structure,mesh,bi,base,half,height);
                (void)height;
                if(dist_xz(candidate,base)<body_radius+half+kRobotObstacleMargin) return false;
            }
        }
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

    auto heading_between=[](Vec3 from,Vec3 to,float fallback){
        const float dx=to.x-from.x,dz=to.z-from.z;
        if(dx*dx+dz*dz<0.0004f) return fallback;
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
    int constructor_route_plans_remaining=2;
    int carrier_route_plans_remaining=3;

    // Corridor-local A*: rasterize only a band around the desired start->goal
    // direction instead of the entire playfield. The old v0.31.17 planner
    // allocated/searched an NxN grid for the whole map *four times per robot*;
    // even a 64x64 map could therefore spend seconds in one placement frame and
    // larger maps scaled quadratically. This planner scales with route length
    // and detour width, caches expensive world-occupancy tests once per cell,
    // and widens the corridor only when a genuine obstacle requires it.
    auto build_spatial_route=[&](std::size_t robot_index,Vec3 direct_goal,float stop_radius,float corridor_half_width,
                                 float traffic_weight,std::vector<Vec3>& geometric){
        geometric.clear();
        auto& robot=builder_robots_[robot_index];
        const Vec3 start_position=robot.position;
        Vec3 delta=direct_goal-start_position; delta.y=0.0f;
        const float direct_length=std::sqrt(delta.x*delta.x+delta.z*delta.z);
        if(direct_length<0.05f){ geometric.push_back(direct_goal); return true; }

        const Vec3 forward{delta.x/direct_length,0.0f,delta.z/direct_length};
        const Vec3 lateral{-forward.z,0.0f,forward.x};
        const float cell=kRouteGridCellM;
        const float along_margin=1.45f;
        const float side_extent=std::max(2.2f,corridor_half_width);
        const float u_min=-along_margin;
        const float u_max=direct_length+along_margin;
        const int nu=std::max(3,(int)std::ceil((u_max-u_min)/cell)+1);
        const int nv=std::max(5,(int)std::ceil((side_extent*2.0f)/cell)+1);
        const int total=nu*nv;
        const float half=(float)playfield_.size()*0.5f-0.52f;
        auto index=[&](int u,int v){ return v*nu+u; };
        auto point=[&](int u,int v){
            const float along=u_min+(float)u*cell;
            const float side=-side_extent+(float)v*cell;
            Vec3 p=start_position+forward*along+lateral*side;
            p.y=first_person_floor_height(p);
            return p;
        };
        const int su=std::clamp((int)std::lround((0.0f-u_min)/cell),0,nu-1);
        const int sv=std::clamp((int)std::lround(side_extent/cell),0,nv-1);
        const int start_index=index(su,sv);
        const float nominal_heading=heading_between(start_position,direct_goal,robot.heading_degrees);

        // -1 unknown, 0 blocked, 1 traversable. Occupancy checks include the
        // physical robot radius and partial construction geometry, so A* does
        // not repeatedly rebuild/retest revealed meshes for every neighbour.
        std::vector<std::int8_t> walkable((std::size_t)total,-1);
        std::vector<float> traffic_cache((std::size_t)total,-1.0f);
        auto cell_clear=[&](int u,int v){
            const int idx=index(u,v);
            if(idx==start_index) return true;
            auto& cached=walkable[(std::size_t)idx];
            if(cached>=0) return cached!=0;
            const Vec3 p=point(u,v);
            bool ok=std::abs(p.x)<=half && std::abs(p.z)<=half;
            if(ok) ok=robot_position_clear(robot_index,p) && robot_structure_move_allowed(robot,p,p);
            if(ok){
                // Stationary/working robots are real temporary obstacles. Now
                // that parking and pickup staging are outside the depot throat,
                // there is no reason for A* to publish a path through a chassis
                // that may remain parked for seconds. Moving robots stay in the
                // lane-cost/prediction layer so parallel traffic remains fluid.
                for(std::size_t i=0;i<builder_robots_.size();++i){
                    if(i==robot_index) continue;
                    const auto& other=builder_robots_[i];
                    if(other.route_active && other.current_speed_mps>0.12f && !other.route_waiting_for_clearance) continue;
                    if(robot_hulls_overlap(p,nominal_heading,robot.role,
                                           other.position,other.heading_degrees,other.role,
                                           kRobotHullClearanceM+0.12f)){ ok=false; break; }
                }
            }
            cached=ok?1:0;
            return ok;
        };
        auto traffic_cost=[&](int u,int v){
            const int idx=index(u,v);
            float& cached=traffic_cache[(std::size_t)idx];
            if(cached>=0.0f) return cached;
            const Vec3 p=point(u,v);
            cached=route_lane_penalty(robot_index,p);
            // Small alternating side preference breaks symmetric head-on route
            // choices without forcing either robot to weave after departure.
            const float signed_side=-side_extent+(float)v*cell;
            const float preferred=((robot_index&1u)==0u)?-1.0f:1.0f;
            cached+=std::max(0.0f,signed_side*preferred)*0.035f;
            return cached;
        };

        std::vector<float> g((std::size_t)total,std::numeric_limits<float>::infinity());
        std::vector<int> parent((std::size_t)total,-1);
        std::vector<std::uint8_t> closed((std::size_t)total,0);
        struct OpenNode { float f; int idx; bool operator<(const OpenNode& o) const { return f>o.f; } };
        std::priority_queue<OpenNode> open;
        g[(std::size_t)start_index]=0.0f;
        open.push({direct_length,start_index});
        constexpr std::array<int,8> du{{1,-1,0,0,1,1,-1,-1}};
        constexpr std::array<int,8> dv{{0,0,1,-1,1,-1,1,-1}};
        int goal_index=-1;
        int expansions=0;
        const int max_expansions=std::min(total,22000);
        const float goal_tolerance=std::max(stop_radius+0.16f,cell*0.82f);

        while(!open.empty() && expansions<max_expansions){
            const int cur=open.top().idx; open.pop();
            if(closed[(std::size_t)cur]) continue;
            closed[(std::size_t)cur]=1; ++expansions;
            const int cu=cur%nu, cv=cur/nu;
            const Vec3 cp=(cur==start_index)?start_position:point(cu,cv);
            if(dist_xz(cp,direct_goal)<=goal_tolerance && route_segment_plannable_clear(robot_index,cp,direct_goal)){
                goal_index=cur; break;
            }

            for(std::size_t n=0;n<du.size();++n){
                const int u=cu+du[n], v=cv+dv[n];
                if(u<0 || v<0 || u>=nu || v>=nv) continue;
                const int ni=index(u,v);
                if(closed[(std::size_t)ni] || !cell_clear(u,v)) continue;
                // Diagonal motion may not cut between two blocked chassis/walls.
                if(du[n]!=0 && dv[n]!=0 && (!cell_clear(cu+du[n],cv) || !cell_clear(cu,cv+dv[n]))) continue;
                const Vec3 np=point(u,v);
                if(std::abs(np.y-cp.y)>0.38f) continue;
                if(!robot_structure_move_allowed(robot,cp,np)) continue;
                const float step=dist_xz(cp,np);
                const float ng=g[(std::size_t)cur]+step+traffic_cost(u,v)*traffic_weight;
                if(ng>=g[(std::size_t)ni]) continue;
                g[(std::size_t)ni]=ng;
                parent[(std::size_t)ni]=cur;
                open.push({ng+dist_xz(np,direct_goal),ni});
            }
        }
        if(goal_index<0) return false;

        std::vector<Vec3> raw;
        raw.reserve((std::size_t)std::max(8,(int)(direct_length/cell)+8));
        for(int cur=goal_index,guard=0;cur!=start_index && cur>=0 && guard++<total;cur=parent[(std::size_t)cur]){
            raw.push_back(point(cur%nu,cur/nu));
            if(parent[(std::size_t)cur]<0) break;
        }
        std::reverse(raw.begin(),raw.end());
        if(raw.empty()) return false;
        if(dist_xz(raw.back(),direct_goal)>0.04f){
            if(!route_segment_plannable_clear(robot_index,raw.back(),direct_goal)) return false;
            raw.push_back(direct_goal);
        }

        // Bounded visibility smoothing. Limiting each look-ahead to roughly
        // 12 m avoids the old O(N^2) long-route simplifier on 256/512 maps.
        Vec3 anchor=start_position;
        std::size_t i=0;
        constexpr std::size_t kMaxVisibilitySkip=18;
        while(i<raw.size()){
            const std::size_t farthest=std::min(raw.size()-1,i+kMaxVisibilitySkip);
            std::size_t best=i;
            for(std::size_t candidate=farthest+1;candidate>i;--candidate){
                const std::size_t j=candidate-1;
                if(route_segment_plannable_clear(robot_index,anchor,raw[j])){ best=j; break; }
            }
            geometric.push_back(raw[best]);
            anchor=raw[best];
            i=best+1;
        }
        return !geometric.empty();
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

        std::vector<Vec3> geometric;
        if(route_segment_plannable_clear(robot_index,robot.position,direct_goal)){
            geometric.push_back(direct_goal);
        }else{
            // Start narrow and widen only if the world actually requires a
            // detour. Traffic is already represented in the cost field, so a
            // single successful search replaces the four full-map searches of
            // v0.31.17.
            const std::array<float,3> widths{{4.5f,9.0f,18.0f}};
            bool found=false;
            for(std::size_t attempt=0;attempt<widths.size();++attempt){
                const float traffic_weight=0.58f+(float)attempt*0.18f;
                if(build_spatial_route(robot_index,direct_goal,stop_radius,widths[attempt],traffic_weight,geometric)){
                    found=true; break;
                }
            }
            if(!found){ robot.route_replan_cooldown=0.12f; return false; }
        }

        auto dense=make_continuous_path(robot_index,geometric);
        if(dense.empty()){ robot.route_replan_cooldown=0.12f; return false; }

        // The route is published immediately. Full-fleet trajectory prediction
        // remains in the continuous speed governor, which is both cheaper and
        // more accurate once all robots are moving. This prevents one newly
        // placed structure from synchronously simulating every robot against
        // every candidate path before the frame can continue.
        return commit_route(robot_index,dense,speed_scale);
    };

    auto robot_has_right_of_way=[&](std::size_t a,std::size_t b){
        if(a>=builder_robots_.size() || b>=builder_robots_.size()) return false;
        const auto& ra=builder_robots_[a];
        const auto& rb=builder_robots_[b];
        if(!rb.route_active){
            // A truly parked rover or one performing a timed arm/weld action is
            // immovable. A route-less robot that is nevertheless in an active
            // Fetch/Deliver/Build travel task is only temporarily waiting and
            // participates in priority inheritance, otherwise two tasks can
            // wait forever for each other to become "moving" first.
            if(rb.mode==BuilderRobotMode::Idle || rb.post_delivery_hold || rb.action!=BuilderRobotAction::None)
                return false;
        }
        const float age_delta=ra.route_priority_age-rb.route_priority_age;
        if(std::abs(age_delta)>0.08f) return age_delta>0.0f;
        // Stable tie-break prevents reciprocal hesitation/spinning. Dynamic age
        // provides PIBT-style anti-starvation across successive tasks.
        return a<b;
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
        speed_scale=std::clamp(speed_scale,0.10f,1.0f);
        robot.route_replan_cooldown=std::max(0.0f,robot.route_replan_cooldown-dt);
        const float d=dist_xz(robot.position,destination);
        if(d<=stop_radius+0.025f){
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            robot.blocked_elapsed=0.0f;
            robot.route_priority_age=0.0f;
            clear_route(robot);
            return true;
        }

        const float goal_shift=dist_xz(destination,robot.route_destination);
        if(robot.route_active && (goal_shift>std::max(0.50f,stop_radius*0.50f) ||
           std::abs(stop_radius-robot.route_stop_radius)>0.12f ||
           std::abs(speed_scale-robot.route_speed_scale)>0.32f)) clear_route(robot);

        if(!robot.route_active){
            if(robot.route_replan_cooldown>0.0f){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.blocked_elapsed+=dt;
                return false;
            }
            int& role_route_plans_remaining=(robot.role==RobotRole::Carrier)?carrier_route_plans_remaining:constructor_route_plans_remaining;
            if(role_route_plans_remaining<=0){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                return false;
            }
            --role_route_plans_remaining;
            if(!plan_route(robot_index,destination,stop_radius,speed_scale)){
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.blocked_elapsed+=dt;
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
            // Persistent yielding triggers a new traffic-weighted lane search,
            // not a retreat or spin. Keep the current heading while stationary.
            if(robot.route_wait_elapsed>kTrafficReplanYieldSeconds && robot.route_replan_cooldown<=0.0f){
                clear_route(robot);
                robot.route_replan_cooldown=0.01f;
                robot.route_wait_elapsed=0.0f;
            }
            return false;
        }

        const float next_progress=std::min(robot.route_total_length_m,robot.route_progress_m+advance);
        Vec3 candidate=route_position_at_distance(robot,next_progress);
        const float lookahead=std::min(robot.route_total_length_m,next_progress+std::max(0.42f,robot.current_speed_mps*0.30f));
        const float target_heading=route_heading_at_distance(robot,lookahead,robot.heading_degrees);
        float candidate_heading=turn_towards_heading(robot.heading_degrees,target_heading,
                                                      std::max(kRobotTurnRateDegreesPerSecond,210.0f)*dt);
        // Turning itself is hull-checked. If the nose cannot rotate safely yet,
        // hold the old orientation and meter speed instead of oscillating left/right.
        if(!fleet_hull_clear(robot_index,robot.position,candidate_heading)){
            candidate_heading=robot.heading_degrees;
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            advance=std::min(advance,robot.current_speed_mps*dt);
            if(advance<0.0005f){ robot.route_waiting_for_clearance=true; robot.route_wait_elapsed+=dt; return false; }
            candidate=route_position_at_distance(robot,std::min(robot.route_total_length_m,robot.route_progress_m+advance));
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
            if(robot.blocked_elapsed>0.70f && robot.route_replan_cooldown<=0.0f){
                clear_route(robot);
                robot.route_replan_cooldown=0.01f;
            }
            return false;
        }

        robot.position=candidate;
        robot.heading_degrees=candidate_heading;
        robot.route_progress_m=std::min(robot.route_total_length_m,robot.route_progress_m+advance);
        while(robot.route_cursor<robot.planned_route_distances.size() &&
              robot.planned_route_distances[robot.route_cursor]<=robot.route_progress_m+0.025f)
            ++robot.route_cursor;
        robot.blocked_elapsed=0.0f;

        if(robot.route_progress_m>=robot.route_total_length_m-0.015f){
            if(!robot.planned_route.empty()) robot.position=robot.planned_route.back();
            robot.current_speed_mps=0.0f;
            robot.route_priority_age=0.0f;
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
        const float service_radius=robot_body_radius(robot.role)+package_radius+kRobotObstacleMargin+0.12f;

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

    // Four deterministic work sectors are visited for every construction step.
    // Each BR-02 starts at a different sector, then progresses clockwise around
    // the structure. This is deliberate task motion, not reactive orbiting, so
    // it remains route-planned and cannot degrade into spinning.
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


    auto construction_work_corner_point=[&](std::size_t robot_index,const ConstructionTargetRef& target,int phase){
        if(robot_index>=builder_robots_.size() || phase<=0)
            return construction_work_point(robot_index,target,std::max(0,phase));
        const auto& robot=builder_robots_[robot_index];
        Vec3 center=construction_target_position(target);
        float footprint_x=1.6f,footprint_z=1.6f,rotation=0.0f;
        if(target.kind==ConstructionTargetKind::Habitat && target.index<placed_habitats_.size()){
            const auto& h=placed_habitats_[target.index]; const auto& mesh=habitat_models_.mesh(h);
            center=h.position; footprint_x=mesh.footprint_x; footprint_z=mesh.footprint_z; rotation=h.rotation_degrees;
        }else if(target.kind==ConstructionTargetKind::Resource && target.index<placed_resources_.size()){
            const auto& item=placed_resources_[target.index]; const auto& mesh=resource_models_.mesh(item);
            center=item.position; footprint_x=mesh.footprint_x; footprint_z=mesh.footprint_z; rotation=item.rotation_degrees;
        }else return construction_work_point(robot_index,target,phase);
        std::size_t ordinal=0;
        for(std::size_t i=0;i<robot_index;++i)
            if(builder_robots_[i].role==RobotRole::Constructor && builder_robots_[i].target.valid() &&
               builder_robots_[i].target.same_as(target)) ++ordinal;
        const int previous=((phase-1)+(int)(ordinal%4u)+16)%4;
        const int current=(phase+(int)(ordinal%4u)+16)%4;
        auto side_x=[](int sector){ return sector==1?1.0f:(sector==3?-1.0f:0.0f); };
        auto side_z=[](int sector){ return sector==0?1.0f:(sector==2?-1.0f:0.0f); };
        float sx=side_x(previous)+side_x(current), sz=side_z(previous)+side_z(current);
        if(std::abs(sx)<0.5f) sx=side_x(current)!=0.0f?side_x(current):side_x(previous);
        if(std::abs(sz)<0.5f) sz=side_z(current)!=0.0f?side_z(current):side_z(previous);
        sx=sx>=0.0f?1.0f:-1.0f; sz=sz>=0.0f?1.0f:-1.0f;
        const RobotFootprint hull=robot_footprint(robot.role);
        const float clearance=hull.half_length+0.52f;
        Vec3 local{sx*(footprint_x*0.5f+clearance),0.0f,sz*(footprint_z*0.5f+clearance)};
        Vec3 p=center+world_math::rotate_y(local,rotation);
        p.y=first_person_floor_height(p);
        return p;
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
            if(other.pickup_queue_age>own_age+0.02f ||
               (std::abs(other.pickup_queue_age-own_age)<=0.02f && i<robot_index)) ++rank;
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

    auto starter_depot_exit_needed=[&](const BuilderRobotState& robot){
        return !robot.cargo.empty() &&
               robot.position.x<starter_stash_center_.x+3.75f &&
               std::abs(robot.position.z-starter_stash_center_.z)<6.8f;
    };
    auto starter_depot_exit=[&](const BuilderRobotState& robot){
        Vec3 gate{starter_stash_center_.x+4.30f,0.0f,robot.position.z};
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
                if(construction_target_complete(candidate)) return;
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

        // Materials are not teleported from the carrier ledger into the build.
        // Every stage has an explicit handoff: CR-01s unload into one shared
        // drop zone, then exactly one BR-02 visits that pile and picks the
        // current step's bundle before any builder may start welding.
        auto material_handler_index=[&](){
            std::size_t handler=kNoRobotIndex;
            for(std::size_t bi=0;bi<builder_robots_.size();++bi){
                const auto& b=builder_robots_[bi];
                if(b.role!=RobotRole::Constructor || !b.target.valid() || !b.target.same_as(robot.target)) continue;
                if(handler==kNoRobotIndex || bi<handler) handler=bi;
            }
            return handler;
        };

        auto begin_stage_material_handoff=[&](){
            if(site_owner.stage_materials_picked) return true;
            const std::size_t handler=material_handler_index();
            if(handler!=robot_index) return false;
            robot.interaction_position=drop_zone;
            robot.interaction_focus_position=drop_zone;
            robot.interaction_position_valid=true;
            if(dist_xz(robot.position,drop_zone)>kPickupWorkZoneM){
                clear_action(robot);
                move_towards(robot_index,drop_zone,kPickupWorkZoneM*0.52f,0.80f);
                return false;
            }
            clear_route(robot);
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            if(robot.action!=BuilderRobotAction::Pickup){
                start_action(robot,BuilderRobotAction::Pickup,kPickupSeconds,MaterialResource::Iron,step_cost.total_amount(),drop_zone);
                return false;
            }
            if(!tick_action(robot,dt)) return false;
            clear_action(robot);
            clear_interaction_station(robot);
            site_owner.stage_materials_picked=true;
            // Start each stage at sector zero. Per-builder ordinal offsets keep
            // a multi-BR crew separated while every rover still visits all four
            // sides during the stage.
            for(auto& teammate:builder_robots_){
                if(teammate.role==RobotRole::Constructor && teammate.target.valid() && teammate.target.same_as(robot.target)){
                    teammate.build_orbit_phase=0.0f;
                    teammate.build_side_corner_reached=false;
                }
            }
            return true;
        };

        if(robot.mode==BuilderRobotMode::Idle) robot.mode=BuilderRobotMode::AwaitingMaterials;

        if(robot.mode==BuilderRobotMode::AwaitingMaterials){
            if(contains_bundle(*shared_site_buffer,step_cost)){
                clear_logistics_assignment(robot);
                robot.idle_elapsed=0.0f;
                if(site_owner.stage_materials_picked || begin_stage_material_handoff()){
                    clear_interaction_station(robot);
                    robot.mode=BuilderRobotMode::Building;
                    robot.work_elapsed=0.0f;
                }else if(material_handler_index()!=robot_index){
                    // Helpers stage at their first work side while the handler
                    // collects the delivered boxes, but cannot build early.
                    const Vec3 ready=construction_work_point(robot_index,robot.target,0);
                    if(dist_xz(robot.position,ready)>kConstructionBayArrivalM)
                        move_towards(robot_index,ready,0.16f,kBuildTravelSpeedScale);
                    else { clear_route(robot); robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt); }
                }
            }else{
                robot.idle_elapsed+=dt;

                // A builder waiting for the rest of a stage may never park on
                // the shared drop zone. Move it back to its first perimeter
                // work sector so CR-01s can keep unloading without a stationary
                // BR-02 becoming a permanent throat obstacle.
                const Vec3 wait_position=construction_work_point(robot_index,robot.target,0);
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
                const bool allow_self_haul=!other_builder_hauling && fallback_builder==robot_index;
                std::size_t available_carriers=0;
                for(const auto& fleet_robot:builder_robots_)
                    if(fleet_robot.role==RobotRole::Carrier) ++available_carriers;
                // Carriers get a short dispatch window. If their reservations
                // still do not cover the stage shortage, the BR-02 collects the
                // uncovered remainder itself. With no CR-01 fleet it reacts almost
                // immediately.
                const float fallback_delay=available_carriers>0?6.0f:0.45f;
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
                        const std::size_t rank=starter_lane_queue_rank(robot_index,robot.interaction_material);
                        const bool occupied=starter_lane_occupied(robot_index,robot.interaction_material,robot.interaction_focus_position);
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
            // A stage can only be worked after a builder has physically picked
            // its resources from the construction drop zone.
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
                    // Every builder starts the next stage's four-side sequence
                    // from phase zero after the next material handoff.
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
            const int phase=std::clamp((int)std::floor(robot.build_orbit_phase),0,3);
            const Vec3 work_position=construction_work_point(robot_index,robot.target,phase);
            robot.interaction_focus_position=center;
            robot.interaction_position_valid=true;

            // Move between wall faces via an explicit outside corner waypoint.
            // This is intentional construction motion: face -> corner -> face.
            if(phase>0 && !robot.build_side_corner_reached){
                const Vec3 corner=construction_work_corner_point(robot_index,robot.target,phase);
                if(dist_xz(robot.interaction_position,corner)>0.18f){
                    robot.interaction_position=corner;
                    clear_route(robot);
                }
                clear_action(robot);
                if(dist_xz(robot.position,corner)>0.26f){
                    move_towards(robot_index,corner,0.14f,kBuildTravelSpeedScale);
                    continue;
                }
                clear_route(robot);
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.build_side_corner_reached=true;
            }

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

            // Each stage is assembled from all four sides. Action durations are
            // divided across the sectors so overall stage time remains close to
            // the existing tuning while the visible rover performs realistic
            // perimeter work between weld/build-up passes.
            const float sector_factor=4.0f*team_work_factor;
            if(robot.action==BuilderRobotAction::None)
                start_action(robot,BuilderRobotAction::Welding,kWeldSeconds/sector_factor,MaterialResource::Iron,0,center);
            if(robot.action==BuilderRobotAction::Welding && tick_action(robot,dt)){
                start_action(robot,BuilderRobotAction::BuildUp,kBuildUpSeconds/sector_factor,MaterialResource::Iron,0,center);
            }else if(robot.action==BuilderRobotAction::BuildUp && tick_action(robot,dt)){
                clear_action(robot);
                clear_interaction_station(robot);
                robot.build_orbit_phase+=1.0f;
                robot.build_side_corner_reached=false;
                if(robot.build_orbit_phase<4.0f) continue;

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
        if(carrier.mode==BuilderRobotMode::Fetching && carrier.cargo.empty() &&
           !carrier.logistics_reservation.empty() && carrier.blocked_elapsed>12.0f &&
           !carrier.route_active && carrier.action==BuilderRobotAction::None){
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
            const std::size_t owner=construction_site_owner_index(carrier.target);
            if(owner>=builder_robots_.size()) return false;
            carrier.support_robot_index=owner;
            return true;
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
            std::size_t best=kNoRobotIndex;
            float best_score=std::numeric_limits<float>::max();
            for(std::size_t i=0;i<builder_robots_.size();++i){
                const auto& b=builder_robots_[i];
                if(b.role!=RobotRole::Constructor || !b.target.valid() || construction_target_complete(b.target)) continue;
                const std::size_t owner=construction_site_owner_index(b.target);
                if(owner!=i) continue; // one ledger evaluation per cooperative site
                // CR-01s feed only the currently executable construction step.
                // This keeps simultaneous sites fair and prevents speculative
                // future-stage loads from congesting the depot and approach lanes.
                const ResourceBundle need=logistics_remaining(b.target,false,true,robot_index);
                if(need.empty() || !site_serviceable(need,loaded_only)) continue;

                float score=dist_xz(carrier.position,construction_target_position(b.target));
                if(!loaded_only){
                    const SourceChoice source=choose_source(robot_index,need);
                    if(!source.valid) continue;
                    score+=source.distance*0.65f;
                }
                std::size_t assigned_carriers=0;
                for(std::size_t ci=0;ci<builder_robots_.size();++ci){
                    if(ci==robot_index) continue;
                    const auto& other=builder_robots_[ci];
                    if(other.role==RobotRole::Carrier && other.target.valid() && other.target.same_as(b.target) &&
                       (other.mode==BuilderRobotMode::Fetching || other.mode==BuilderRobotMode::Delivering)) ++assigned_carriers;
                }
                                // Seed every serviceable active construction with logistics before
                // assigning a second CR-01 to an already-served site. Distance
                // only breaks ties inside the same staffing level. This prevents
                // two nearby carriers from both feeding one blueprint while a
                // second simultaneous structure is forced into BR-02 self-haul.
                score+=(float)assigned_carriers*1000.0f;
                score-=std::min(6.0f,b.idle_elapsed*0.16f);
                if(score<best_score){ best_score=score; best=owner; }
            }
            return best;
        };

        auto assign_empty_carrier=[&](){
            for(int attempt=0;attempt<2;++attempt){
                const std::size_t support=choose_support(false);
                if(support==kNoRobotIndex) return false;
                const ConstructionTargetRef target=builder_robots_[support].target;
                carrier.target=target;
                carrier.service_slot_offset=0;
                carrier.support_robot_index=support;
                carrier.logistics_reservation={};
                if(!reserve_trip(robot_index,target,false).empty()){
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
            const std::size_t support=choose_support(true);
            if(support==kNoRobotIndex) return false;
            carrier.target=builder_robots_[support].target;
            carrier.service_slot_offset=0;
            carrier.support_robot_index=support;
            carrier.logistics_reservation={};
            if(!reserve_loaded_cargo_for_site(carrier,carrier.target,false)){
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
                        const std::size_t rank=starter_lane_queue_rank(robot_index,carrier.interaction_material);
                        const bool occupied=starter_lane_occupied(robot_index,carrier.interaction_material,carrier.interaction_focus_position);
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
                    carrier.cargo_committed_to_build=false;
                    carrier.target={};
                    carrier.support_robot_index=kNoRobotIndex;
                    clear_interaction_station(carrier);
                    carrier.blocked_elapsed=0.0f;
                    carrier.watchdog_elapsed=0.0f;
                    carrier.recovery_attempts=0;
                    carrier.mode=carrier.cargo.empty()?BuilderRobotMode::Idle:BuilderRobotMode::DepotTransfer;
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

    // Idle CR-01s never perform cosmetic return-to-depot travel. A carrier that
    // loses/finishes a task simply holds where it has safely cleared traffic.
    // If that happens inside the starter-depot operating band, perform exactly
    // one eastward pull-out first, then keep that hull parked until real work
    // is assigned. This removes nonproductive routes that used to cross active
    // pallet approaches and block working carriers.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        auto& robot=builder_robots_[robot_index];
        if(robot.role!=RobotRole::Carrier || robot.mode!=BuilderRobotMode::Idle ||
           !robot.cargo.empty() || robot.target.valid()) continue;
        if(robot.post_delivery_hold){
            robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
            clear_route(robot);
            continue;
        }
        const bool in_depot_traffic=std::abs(robot.position.z-starter_stash_center_.z)<7.2f &&
                                    robot.position.x<starter_stash_center_.x+12.0f;
        if(in_depot_traffic){
            Vec3 pullout{starter_stash_center_.x+12.5f,0.0f,robot.position.z};
            pullout.y=first_person_floor_height(pullout);
            if(dist_xz(robot.position,pullout)>0.24f){
                clear_action(robot);
                move_towards(robot_index,pullout,0.16f,0.78f);
            }else{
                clear_route(robot);
                clear_interaction_station(robot);
                robot.current_speed_mps=approach(robot.current_speed_mps,0.0f,kRobotBrakingMps2*dt);
                robot.post_delivery_hold=true;
            }
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
            starter_lane_queue_rank(robot_index,robot.interaction_material)>0;
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
            // Preserve the pallet ticket/station/reservation. Recovery here is
            // route-only; revoking the transaction can hand logical ownership
            // to a rover behind one that is already physically in the throat.
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
                Vec3 away=rb.position-ra.position;
                float len=std::sqrt(away.x*away.x+away.z*away.z);
                if(len<0.001f){ away={(b&1u)?1.0f:-1.0f,0.0f,(b&2u)?1.0f:-1.0f}; len=std::sqrt(2.0f); }
                away=away*(1.0f/len);
                const float nudge=0.07f;
                Vec3 cb=rb.position+away*nudge; cb.y=first_person_floor_height(cb);
                if(robot_position_clear(b,cb) && robot_structure_move_allowed(rb,rb.position,cb) && fleet_hull_clear(b,cb,rb.heading_degrees)){
                    rb.position=cb; rb.current_speed_mps=0.0f; clear_route(rb);
                    continue;
                }
                Vec3 ca=ra.position-away*nudge; ca.y=first_person_floor_height(ca);
                if(robot_position_clear(a,ca) && robot_structure_move_allowed(ra,ra.position,ca) && fleet_hull_clear(a,ca,ra.heading_degrees)){
                    ra.position=ca; ra.current_speed_mps=0.0f; clear_route(ra);
                }
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
