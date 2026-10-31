#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;

namespace {
float rotated_rect_distance_xz(Vec3 hit,Vec3 center,float rotation,float sx,float sz){
    const Vec3 local=rotate_y(hit-center,-rotation);
    const float dx=std::max(std::abs(local.x)-sx*0.5f,0.0f);
    const float dz=std::max(std::abs(local.z)-sz*0.5f,0.0f);
    return std::sqrt(dx*dx+dz*dz);
}

Vec3 closest_point_segment_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<1.0e-6f) return {a.x,0.0f,a.z};
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return {a.x+dx*t,0.0f,a.z+dz*t};
}

struct NetworkSnap {
    bool found{false};
    std::size_t index{0};
    Vec3 point{};
    float distance{std::numeric_limits<float>::max()};
};

template<class ConnectionList>
NetworkSnap nearest_network_snap(Vec3 hit,const ConnectionList& links,float radius){
    NetworkSnap best;
    best.distance=radius;
    for(std::size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(std::size_t p=0;p+1<path.size();++p){
            const Vec3 q=closest_point_segment_xz(hit,path[p],path[p+1]);
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best.distance){ best={true,i,q,d}; }
        }
    }
    return best;
}

ResourceBundle bundle_difference(const ResourceBundle& a,const ResourceBundle& b){
    ResourceBundle result{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        result.amount(material)=std::max<std::int64_t>(0,a.amount(material)-b.amount(material));
    }
    return result;
}

ResourceBundle bundle_sum(const ResourceBundle& a,const ResourceBundle& b){
    ResourceBundle result{};
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        result.amount(material)=a.amount(material)+b.amount(material);
    }
    return result;
}

float recovered_resource_condition(MaterialResource material){
    switch(material){
        case MaterialResource::Iron: return 0.94f;
        case MaterialResource::Copper: return 0.97f;
        case MaterialResource::Silicon: return 0.90f;
        case MaterialResource::Plastic: return 0.84f;
        case MaterialResource::Water: return 0.99f;
        case MaterialResource::Food: return 0.93f;
        case MaterialResource::Medical: return 0.98f;
        case MaterialResource::Aluminum: return 0.96f;
        case MaterialResource::Count: break;
    }
    return 1.0f;
}

bool project_world_to_screen(const Mat4& m,Vec3 p,int viewport_width,int viewport_height,float& sx,float& sy){
    const float cx=m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12];
    const float cy=m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13];
    const float cw=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];
    if(cw<=0.001f) return false;
    const float nx=cx/cw, ny=cy/cw;
    sx=(nx*0.5f+0.5f)*(float)viewport_width;
    sy=(1.0f-(ny*0.5f+0.5f))*(float)viewport_height;
    return sx>=0.0f && sx<=(float)viewport_width && sy>=0.0f && sy<=(float)viewport_height;
}

Vec3 point_along_path(const std::vector<Vec3>& path,float fraction,Vec3* tangent_out=nullptr){
    if(path.empty()) return {};
    if(path.size()==1){ if(tangent_out) *tangent_out={1.0f,0.0f,0.0f}; return path.front(); }
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i){
        const float dx=path[i+1].x-path[i].x,dz=path[i+1].z-path[i].z;
        total+=std::sqrt(dx*dx+dz*dz);
    }
    if(total<1.0e-5f){ if(tangent_out) *tangent_out={1.0f,0.0f,0.0f}; return path.front(); }
    float target=std::clamp(fraction,0.0f,1.0f)*total;
    float walked=0.0f;
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 a=path[i],b=path[i+1];
        const float dx=b.x-a.x,dz=b.z-a.z;
        const float len=std::sqrt(dx*dx+dz*dz);
        if(len<1.0e-5f) continue;
        if(walked+len>=target || i+2==path.size()){
            const float t=std::clamp((target-walked)/len,0.0f,1.0f);
            if(tangent_out) *tangent_out={dx/len,0.0f,dz/len};
            return {a.x+dx*t,0.0f,a.z+dz*t};
        }
        walked+=len;
    }
    if(tangent_out) *tangent_out={1.0f,0.0f,0.0f};
    return path.back();
}
}

Rect GameApp::selection_panel_rect() const {
    if(selection_kind_==SelectionKind::Robot)
        return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,485.0f,390.0f};
    if(selection_kind_==SelectionKind::Resource && selected_resource_>=0 && (std::size_t)selected_resource_<placed_resources_.size() &&
       placed_resources_[(std::size_t)selected_resource_].kind==ResourceKind::StorageDome){
        return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,485.0f,360.0f};
    }
    return {PLAYFIELD_MARGIN,STATUS_BAR_H+PLAYFIELD_MARGIN+68.0f,410.0f,280.0f};
}

Rect GameApp::ground_resource_panel_rect() const {
    if(selected_ground_resources_.size()>1) return selection_panel_rect();
    if(selected_ground_resource_<0 || (std::size_t)selected_ground_resource_>=ground_resource_piles_.size()) return {};
    const auto& pile=ground_resource_piles_[(std::size_t)selected_ground_resource_];
    const float aspect=(float)viewport_width_/std::max(1,viewport_height_);
    const Mat4 m=camera_.view_projection(aspect);
    const Vec3 p=pile.position+Vec3{0.0f,resource_package_height(pile.resource,pile.amount)+0.18f,0.0f};
    const float cx=m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12];
    const float cy=m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13];
    const float cw=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];
    if(cw<=0.001f) return selection_panel_rect();
    const float nx=cx/cw, ny=cy/cw;
    const float logical_w=(float)viewport_width_/ui_scale_;
    const float logical_h=(float)viewport_height_/ui_scale_;
    float x=(nx*0.5f+0.5f)*logical_w+18.0f;
    float y=(1.0f-(ny*0.5f+0.5f))*logical_h-54.0f;
    constexpr float w=228.0f,h=126.0f;
    if(x+w>logical_w-8.0f) x=(nx*0.5f+0.5f)*logical_w-w-18.0f;
    x=std::clamp(x,8.0f,logical_w-w-8.0f);
    y=std::clamp(y,STATUS_BAR_H+8.0f,logical_h-BUILD_BAR_H-h-8.0f);
    return {x,y,w,h};
}

bool GameApp::mouse_over_selection_panel(const InputState& input) const {
    if(selection_kind_==SelectionKind::None) return false;
    const Rect p=selection_kind_==SelectionKind::GroundResource?ground_resource_panel_rect():selection_panel_rect();
    return p.contains((float)input.mouse_x/ui_scale_,(float)input.mouse_y/ui_scale_);
}

void GameApp::clear_world_selection(){
    selection_kind_=SelectionKind::None;
    selected_habitat_=-1;
    selected_corridor_=-1;
    selected_resource_=-1;
    selected_ground_resource_=-1;
    selected_ground_resources_.clear();
    selected_utility_=-1;
    selected_walkway_=-1;
    selected_robot_=-1;
}

int GameApp::pick_robot(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(std::size_t i=0;i<builder_robots_.size();++i){
        const auto& robot=builder_robots_[i];
        const float dx=hit.x-robot.position.x;
        const float dz=hit.z-robot.position.z;
        const float d=std::sqrt(dx*dx+dz*dz);
        const float pick_radius=std::max(radius,robot_body_radius(robot.role)+0.28f);
        if(d<=pick_radius && d<best_distance){ best_distance=d; best=(int)i; }
    }
    return best;
}

int GameApp::pick_habitat(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h);
        const float a=radians(h.rotation_degrees);
        const float c=std::cos(a),si=std::sin(a);
        const float dx=hit.x-h.position.x,dz=hit.z-h.position.z;
        const float lx=dx*c-dz*si;
        const float lz=dx*si+dz*c;
        if(std::abs(lx)<=mesh.footprint_x*0.5f && std::abs(lz)<=mesh.footprint_z*0.5f){
            const float d=dx*dx+dz*dz;
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_resource(Vec3 hit) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        const auto& mesh=resource_models_.mesh(item);
        const float a=radians(item.rotation_degrees);
        const float c=std::cos(a),si=std::sin(a);
        const float dx=hit.x-item.position.x,dz=hit.z-item.position.z;
        const float lx=dx*c-dz*si;
        const float lz=dx*si+dz*c;
        if(std::abs(lx)<=mesh.footprint_x*0.5f && std::abs(lz)<=mesh.footprint_z*0.5f){
            const float d=dx*dx+dz*dz;
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_ground_resource(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=std::numeric_limits<float>::max();
    for(std::size_t i=0;i<ground_resource_piles_.size();++i){
        const auto& pile=ground_resource_piles_[i];
        const float dx=hit.x-pile.position.x,dz=hit.z-pile.position.z;
        const float d=std::sqrt(dx*dx+dz*dz);
        if(d<=pile.radius+radius && d<best_distance){ best_distance=d; best=(int)i; }
    }
    return best;
}

int GameApp::pick_walkway(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& links=walkway_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

int GameApp::pick_utility(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& links=utility_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& path=links[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

std::optional<UtilityBuildAnchor> GameApp::utility_anchor_at(Vec3 hit,UtilityKind kind) const {
    // Direct footprint hit always wins. Keep the real cursor hit in the anchor:
    // the structure still snaps as a whole, while the hit position is used to
    // fine-adjust the exact utility attachment point on its mesh surface.
    const int resource=pick_resource(hit);
    if(resource>=0){
        UtilityBuildAnchor a{UtilityEndpointKind::Resource,(size_t)resource,hit};
        if(utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) return a;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        UtilityBuildAnchor a{UtilityEndpointKind::Habitat,(size_t)habitat,hit};
        if(utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) return a;
    }

    // Supply channels are deliberately easy to attach: when the pointer is near
    // a resource or habitat, snap to the closest compatible structure even if
    // it is just outside the footprint. The original cursor hit is retained for
    // exact mesh-surface port placement.
    if(kind==UtilityKind::SupplyChannel){
        constexpr float SNAP_MARGIN=2.25f;
        float best=SNAP_MARGIN;
        std::optional<UtilityBuildAnchor> snapped;
        for(std::size_t i=0;i<placed_resources_.size();++i){
            UtilityBuildAnchor a{UtilityEndpointKind::Resource,i,hit};
            if(!utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) continue;
            const auto& mesh=resource_models_.mesh(placed_resources_[i]);
            const float d=rotated_rect_distance_xz(hit,placed_resources_[i].position,placed_resources_[i].rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(d<best){ best=d; snapped=a; }
        }
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            UtilityBuildAnchor a{UtilityEndpointKind::Habitat,i,hit};
            if(!utility_network_.anchor_compatible(kind,a,placed_habitats_,placed_resources_)) continue;
            const auto& mesh=habitat_models_.mesh(placed_habitats_[i]);
            const float d=rotated_rect_distance_xz(hit,placed_habitats_[i].position,placed_habitats_[i].rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(d<best){ best=d; snapped=a; }
        }
        if(snapped) return snapped;
    }

    // Existing supply runs are valid manifold starts/ends. Snap the open anchor
    // to the actual tray centerline so a new branch creates a true junction.
    const auto utility_snap=nearest_network_snap(hit,utility_network_.connections(),1.35f);
    if(utility_snap.found) return UtilityBuildAnchor{UtilityEndpointKind::Open,0,utility_snap.point};

    // Free endpoints are supported as deliberate capped service stubs. This
    // also guarantees that the pre-click one-section sample is always present.
    return UtilityBuildAnchor{UtilityEndpointKind::Open,0,hit};
}

std::optional<WalkwayBuildAnchor> GameApp::walkway_anchor_at(Vec3 hit,bool source_only) const {
    // Structures win when the pointer is actually on them. Habitat resolution
    // then snaps to a built-in entrance or generates a valid new entrance.
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,(size_t)habitat,hit};
    const int resource=pick_resource(hit);
    if(resource>=0) return WalkwayBuildAnchor{WalkwayEndpointKind::Resource,(size_t)resource,hit};

    // Existing traversable connections are magnetic intersection targets. On
    // the destination click prefer a candidate that the real walkway solver can
    // intersect. If none is legal, keep the closest attempted snap so the full
    // blueprint is shown in red instead of silently falling back to open ground.
    const auto walkway_snap=nearest_network_snap(hit,walkway_network_.connections(),1.65f);
    const auto corridor_snap=nearest_network_snap(hit,corridor_network_.connections(),1.75f);
    if(source_only || !walkway_build_start_){
        if(walkway_snap.found && (!corridor_snap.found || walkway_snap.distance<=corridor_snap.distance))
            return WalkwayBuildAnchor{WalkwayEndpointKind::Walkway,walkway_snap.index,walkway_snap.point};
        if(corridor_snap.found)
            return WalkwayBuildAnchor{WalkwayEndpointKind::Corridor,corridor_snap.index,corridor_snap.point};
    }else{
        std::optional<WalkwayBuildAnchor> valid;
        float valid_distance=std::numeric_limits<float>::max();
        std::optional<WalkwayBuildAnchor> attempted;
        float attempted_distance=std::numeric_limits<float>::max();
        const auto consider=[&](WalkwayEndpointKind kind,const NetworkSnap& snap){
            if(!snap.found) return;
            WalkwayBuildAnchor candidate{kind,snap.index,snap.point};
            if(snap.distance<attempted_distance){ attempted=candidate; attempted_distance=snap.distance; }
            const auto preview=walkway_network_.preview_connection(*walkway_build_start_,candidate,corridor_network_,utility_network_,
                                                                   placed_habitats_,habitat_models_,placed_resources_,resource_models_);
            if(preview && snap.distance<valid_distance){ valid=candidate; valid_distance=snap.distance; }
        };
        consider(WalkwayEndpointKind::Walkway,walkway_snap);
        consider(WalkwayEndpointKind::Corridor,corridor_snap);
        if(valid) return valid;
        if(attempted) return attempted;
    }

    // Near a structure, keep snapping even when the pointer sits just outside
    // its footprint. The exact hit is retained so the entrance resolver can
    // choose the nearest built-in hatch or generate a new one at that side.
    constexpr float SNAP_MARGIN=2.25f;
    float best=SNAP_MARGIN;
    std::optional<WalkwayBuildAnchor> snapped;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i]; const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Habitat,i,hit}; }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& r=placed_resources_[i]; const auto& mesh=resource_models_.mesh(r);
        const float d=rotated_rect_distance_xz(hit,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=WalkwayBuildAnchor{WalkwayEndpointKind::Resource,i,hit}; }
    }
    if(snapped) return snapped;

    return WalkwayBuildAnchor{WalkwayEndpointKind::Open,0,hit};
}

int GameApp::pick_corridor(Vec3 hit,float radius) const {
    int best=-1;
    float best_distance=radius;
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& path=corridors[i].path;
        for(size_t p=0;p+1<path.size();++p){
            const float d=point_segment_distance_xz(hit,path[p],path[p+1]);
            if(d<best_distance){ best_distance=d; best=(int)i; }
        }
    }
    return best;
}

CorridorBuildAnchor GameApp::corridor_anchor_at(Vec3 hit) const {
    // Corridor construction is node-first: every visible point of every built
    // corridor is a legal source/destination candidate.  Test the pressure
    // network before broad structure picking; otherwise a docking sleeve or a
    // corridor running very close to a habitat footprint can be stolen by the
    // owning structure and the requested mid-run intersection becomes
    // impossible to select.  Unoccupied structure entrances are still picked
    // normally because there is no corridor under the pointer yet.
    const int corridor=pick_corridor(hit,geometry::kCorridorHalfWidth+0.82f);
    if(corridor>=0) return corridor_network_.corridor_anchor((size_t)corridor,hit);

    // Storage domes expose four fixed pressure docking faces. They participate
    // in corridor snapping like habitats, but remain resource/storage objects.
    const int resource=pick_resource(hit);
    if(resource>=0 && placed_resources_[(std::size_t)resource].kind==ResourceKind::StorageDome)
        return {CorridorEndpointKind::Resource,(std::size_t)resource,hit};
    const int habitat=pick_habitat(hit);
    if(habitat>=0) return {CorridorEndpointKind::Habitat,(size_t)habitat,hit};

    constexpr float STRUCTURE_SNAP_MARGIN=2.10f;
    float best=STRUCTURE_SNAP_MARGIN;
    std::optional<CorridorBuildAnchor> snapped;
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(h);
        const float d=rotated_rect_distance_xz(hit,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=CorridorBuildAnchor{CorridorEndpointKind::Habitat,i,hit}; }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        const float d=rotated_rect_distance_xz(hit,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
        if(d<best){ best=d; snapped=CorridorBuildAnchor{CorridorEndpointKind::Resource,i,hit}; }
    }
    if(snapped) return *snapped;

    return {CorridorEndpointKind::OpenDoor,0,{std::round(hit.x),0.0f,std::round(hit.z)}};
}

void GameApp::select_world_at(int screen_x,int screen_y){
    Vec3 hit{};
    if(!camera_.intersect_playfield(screen_x,screen_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const int robot=pick_robot(hit);
    if(robot>=0){
        selection_kind_=SelectionKind::Robot;
        selected_robot_=robot;
        selected_habitat_=-1; selected_corridor_=-1; selected_resource_=-1; selected_utility_=-1; selected_walkway_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    const int ground_resource=pick_ground_resource(hit);
    if(ground_resource>=0){
        selection_kind_=SelectionKind::GroundResource;
        selected_ground_resource_=ground_resource;
        selected_ground_resources_.assign(1,ground_resource);
        selected_habitat_=-1; selected_corridor_=-1; selected_resource_=-1; selected_utility_=-1; selected_walkway_=-1;
        return;
    }
    const int utility=pick_utility(hit);
    if(utility>=0){
        selection_kind_=SelectionKind::Utility;
        selected_utility_=utility;
        selected_corridor_=-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    const int walkway=pick_walkway(hit);
    if(walkway>=0){
        selection_kind_=SelectionKind::Walkway;
        selected_walkway_=walkway;
        selected_utility_=-1;
        selected_corridor_=-1;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    const int corridor=pick_corridor(hit);
    if(corridor>=0){
        selection_kind_=SelectionKind::Corridor;
        selected_corridor_=corridor;
        selected_habitat_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    const int habitat=pick_habitat(hit);
    if(habitat>=0){
        selection_kind_=SelectionKind::Habitat;
        selected_habitat_=habitat;
        selected_corridor_=-1;
        selected_resource_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    const int resource=pick_resource(hit);
    if(resource>=0){
        selection_kind_=SelectionKind::Resource;
        selected_resource_=resource;
        selected_habitat_=-1;
        selected_corridor_=-1;
        selected_utility_=-1;
        selected_walkway_=-1;
        selected_ground_resource_=-1; selected_ground_resources_.clear();
        return;
    }
    clear_world_selection();
}


void GameApp::finish_selection_marquee(){
    const int dx=input_.mouse_x-selection_marquee_start_x_;
    const int dy=input_.mouse_y-selection_marquee_start_y_;
    const float drag_px=std::sqrt((float)(dx*dx+dy*dy));
    selection_marquee_dragged_=selection_marquee_dragged_ || drag_px>=6.0f*ui_scale_;

    if(!selection_marquee_dragged_){
        select_world_at(selection_marquee_start_x_,selection_marquee_start_y_);
        selection_marquee_active_=false;
        selection_marquee_dragged_=false;
        return;
    }

    const float min_x=(float)std::min(selection_marquee_start_x_,input_.mouse_x);
    const float max_x=(float)std::max(selection_marquee_start_x_,input_.mouse_x);
    const float min_y=(float)std::min(selection_marquee_start_y_,input_.mouse_y);
    const float max_y=(float)std::max(selection_marquee_start_y_,input_.mouse_y);
    const float aspect=(float)viewport_width_/std::max(1,viewport_height_);
    const Mat4 vp=camera_.view_projection(aspect);

    selected_ground_resources_.clear();
    for(std::size_t i=0;i<ground_resource_piles_.size();++i){
        const auto& pile=ground_resource_piles_[i];
        const Vec3 center=pile.position+Vec3{0.0f,resource_package_height(pile.resource,pile.amount)*0.5f,0.0f};
        float sx=0.0f,sy=0.0f;
        if(!project_world_to_screen(vp,center,viewport_width_,viewport_height_,sx,sy)) continue;
        if(sx>=min_x && sx<=max_x && sy>=min_y && sy<=max_y) selected_ground_resources_.push_back((int)i);
    }

    if(selected_ground_resources_.empty()){
        clear_world_selection();
    }else{
        selection_kind_=SelectionKind::GroundResource;
        selected_ground_resource_=selected_ground_resources_.front();
        selected_habitat_=-1; selected_corridor_=-1; selected_resource_=-1; selected_utility_=-1; selected_walkway_=-1;
    }
    selection_marquee_active_=false;
    selection_marquee_dragged_=false;
}



void GameApp::drop_structure_resources(Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used){
    std::size_t total_boxes=0;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        const std::int64_t amount=std::max<std::int64_t>(0,used.amount(material));
        const std::int64_t cap=resource_package_capacity_kg(material);
        if(amount>0) total_boxes+=(std::size_t)((amount+cap-1)/cap);
    }
    if(total_boxes==0) return;

    constexpr int target_stack_layers=5;
    const std::size_t locations=(total_boxes+(std::size_t)target_stack_layers-1)/(std::size_t)target_stack_layers;
    const int grid=std::max(1,(int)std::ceil(std::sqrt((double)locations)));
    const float spacing=std::max(0.54f,std::min(0.88f,std::max(footprint_x,footprint_z)/(float)std::max(2,grid)));
    std::vector<float> column_heights(std::max<std::size_t>(1,locations),0.0f);
    std::size_t box_index=0;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto material=static_cast<MaterialResource>(i);
        std::int64_t remaining=std::max<std::int64_t>(0,used.amount(material));
        const std::int64_t cap=resource_package_capacity_kg(material);
        while(remaining>0){
            const std::int64_t amount=std::min(remaining,cap);
            // Fill all ground cells before starting another layer. Each column
            // tracks its actual package heights so the next box rests on top.
            const std::size_t cell=box_index%column_heights.size();
            const int gx=(int)(cell%(std::size_t)grid);
            const int gz=(int)(cell/(std::size_t)grid);
            const Vec3 local{((float)gx-(float)(grid-1)*0.5f)*spacing,
                             column_heights[cell],
                             ((float)gz-(float)(grid-1)*0.5f)*spacing};
            GroundResourcePile pile;
            pile.resource=material;
            pile.amount=amount;
            pile.position=position+rotate_y(local,rotation_degrees);
            pile.condition=recovered_resource_condition(material);
            pile.radius=resource_package_half_width(material,amount);
            ground_resource_piles_.push_back(pile);
            column_heights[cell]+=resource_package_height(material,amount)+0.02f;
            remaining-=amount;
            ++box_index;
        }
    }
}

void GameApp::drop_path_resources(const std::vector<Vec3>& path,float lateral_width,const ResourceBundle& used){
    if(path.empty() || used.empty()) return;

    std::size_t total_boxes=0;
    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=static_cast<MaterialResource>(ri);
        const std::int64_t amount=std::max<std::int64_t>(0,used.amount(material));
        const std::int64_t cap=resource_package_capacity_kg(material);
        if(amount>0) total_boxes+=(std::size_t)((amount+cap-1)/cap);
    }
    if(total_boxes==0) return;

    constexpr int target_stack_layers=4;
    const std::size_t route_slots=(total_boxes+(std::size_t)target_stack_layers-1)/(std::size_t)target_stack_layers;
    std::vector<float> column_heights(std::max<std::size_t>(1,route_slots),0.0f);
    std::size_t global_box=0;
    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=static_cast<MaterialResource>(ri);
        std::int64_t remaining=std::max<std::int64_t>(0,used.amount(material));
        const std::int64_t cap=resource_package_capacity_kg(material);
        while(remaining>0){
            const std::int64_t amount=std::min(remaining,cap);
            // Distribute the first layer along the route before stacking. This
            // keeps recovered boxes as visible piles instead of one tall queue.
            const std::size_t slot=global_box%column_heights.size();
            const float fraction=((float)slot+0.5f)/(float)std::max<std::size_t>(1,route_slots);
            Vec3 tangent{};
            Vec3 position=point_along_path(path,fraction,&tangent);
            const Vec3 side{-tangent.z,0.0f,tangent.x};
            const int lane=(int)(slot%3)-1;
            position=position+side*((float)lane*std::min(0.50f,std::max(0.30f,lateral_width*0.24f)));
            position.y+=column_heights[slot];

            GroundResourcePile pile;
            pile.resource=material;
            pile.amount=amount;
            pile.condition=recovered_resource_condition(material);
            pile.position=position;
            pile.radius=resource_package_half_width(material,amount);
            ground_resource_piles_.push_back(pile);

            column_heights[slot]+=resource_package_height(material,amount)+0.02f;
            remaining-=amount;
            ++global_box;
        }
    }
}

ResourceBundle GameApp::store_resource_bundle(const ResourceBundle& recovered,float condition){
    ResourceBundle stored{};

    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=static_cast<MaterialResource>(ri);
        std::int64_t remaining=std::max<std::int64_t>(0,recovered.amount(material));
        while(remaining>0){
            PlacedResource* destination=nullptr;
            std::int64_t free_capacity=0;
            for(auto& structure:placed_resources_){
                if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
                sync_stored_resource_totals(structure);
                const std::int64_t free=storage_dome_capacity_kg(structure.storage_size)-structure.stored_resources.total_amount();
                if(free<=0) continue;
                destination=&structure;
                free_capacity=free;
                break;
            }
            if(destination==nullptr) break;

            const std::int64_t amount=std::min({remaining,free_capacity,resource_package_capacity_kg(material)});
            destination->stored_boxes.push_back({material,amount,std::clamp(condition,0.0f,1.0f)});
            sync_stored_resource_totals(*destination);
            stored.amount(material)+=amount;
            remaining-=amount;
        }
    }

    if(!stored.empty()) resources_.add(stored);
    return stored;
}

void GameApp::recover_removed_resources(Vec3 position,float rotation_degrees,float footprint_x,float footprint_z,const ResourceBundle& used){
    if(used.empty()) return;

    // Debug mode represents instant logistics: recovered construction material is
    // boxed and routed directly into completed Storage Domes. In normal gameplay
    // it remains on the ground and must be handled by the logistics layer later.
    if(debug_frame_enabled_){
        const ResourceBundle stored=store_resource_bundle(used,0.90f);
        const ResourceBundle remainder=bundle_difference(used,stored);
        if(!remainder.empty()) drop_structure_resources(position,rotation_degrees,footprint_x,footprint_z,remainder);
        return;
    }
    drop_structure_resources(position,rotation_degrees,footprint_x,footprint_z,used);
}

bool GameApp::move_ground_resource_to_storage(std::size_t pile_index){
    if(pile_index>=ground_resource_piles_.size()) return false;
    const auto pile=ground_resource_piles_[pile_index];
    for(auto& structure:placed_resources_){
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        sync_stored_resource_totals(structure);
        const std::int64_t used=structure.stored_resources.total_amount();
        if(used+pile.amount>storage_dome_capacity_kg(structure.storage_size)) continue;

        // Preserve the selected amount exactly, but normalize legacy/oversized
        // piles into human-portable packages before they enter the dome.
        std::int64_t remaining=pile.amount;
        const std::int64_t cap=resource_package_capacity_kg(pile.resource);
        while(remaining>0){
            const std::int64_t amount=std::min(remaining,cap);
            structure.stored_boxes.push_back({pile.resource,amount,pile.condition});
            remaining-=amount;
        }
        sync_stored_resource_totals(structure);
        ResourceBundle added{};
        added.amount(pile.resource)=pile.amount;
        resources_.add(added);
        ground_resource_piles_.erase(ground_resource_piles_.begin()+(std::ptrdiff_t)pile_index);
        return true;
    }
    return false;
}

void GameApp::delete_selected_world_object(){
    switch(selection_kind_){
        case SelectionKind::Habitat:
            if(selected_habitat_>=0 && (std::size_t)selected_habitat_<placed_habitats_.size()){
                const std::size_t index=(std::size_t)selected_habitat_;
                const auto removed=placed_habitats_[index];
                const auto mesh=habitat_models_.mesh(removed);
                const ResourceBundle used=bundle_sum(removed.resource_plan.consumed(removed.construction.step),removed.site_buffer);
                corridor_network_.remove_habitat(index);
                utility_network_.remove_habitat(index);
                walkway_network_.remove_habitat(index);
                placed_habitats_.erase(placed_habitats_.begin()+(std::ptrdiff_t)index);
                recover_removed_resources(removed.position,removed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,used);
            }
            break;
        case SelectionKind::Resource:
            if(selected_resource_>=0 && (std::size_t)selected_resource_<placed_resources_.size()){
                const std::size_t index=(std::size_t)selected_resource_;
                auto removed=placed_resources_[index];
                sync_stored_resource_totals(removed);
                const auto mesh=resource_models_.mesh(removed);
                const ResourceBundle construction_used=bundle_sum(removed.resource_plan.consumed(removed.construction.step),removed.site_buffer);

                if(removed.kind==ResourceKind::StorageDome && !removed.stored_resources.empty()){
                    // Removing a dome releases its stored boxes unchanged. The shared
                    // available inventory is reduced by the exact amount represented
                    // by those physical boxes.
                    if(!removed.stored_boxes.empty()){
                        const std::size_t box_count=removed.stored_boxes.size();
                        for(std::size_t bi=0;bi<box_count;++bi){
                            GroundResourcePile pile;
                            pile.resource=removed.stored_boxes[bi].resource;
                            pile.amount=removed.stored_boxes[bi].amount;
                            pile.condition=removed.stored_boxes[bi].condition;
                            float half=0.0f,height=0.0f;
                            stored_resource_box_pose(removed,mesh,bi,pile.position,half,height);
                            pile.radius=half;
                            ground_resource_piles_.push_back(pile);
                        }
                    }
                    ResourceBundle subtract{};
                    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                        const auto material=static_cast<MaterialResource>(ri);
                        subtract.amount(material)=-removed.stored_resources.amount(material);
                    }
                    resources_.add(subtract);
                }

                corridor_network_.remove_resource(index);
                utility_network_.remove_resource(index);
                walkway_network_.remove_resource(index);
                placed_resources_.erase(placed_resources_.begin()+(std::ptrdiff_t)index);
                recover_removed_resources(removed.position,removed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,construction_used);
            }
            break;
        case SelectionKind::GroundResource:
            if(!debug_frame_enabled_) return;
            if(selected_ground_resources_.size()>1){
                auto indices=selected_ground_resources_;
                std::sort(indices.begin(),indices.end(),std::greater<int>());
                bool moved_any=false;
                for(int index:indices){
                    if(index<0 || (std::size_t)index>=ground_resource_piles_.size()) continue;
                    moved_any=move_ground_resource_to_storage((std::size_t)index) || moved_any;
                }
                if(!moved_any) return;
            }else if(selected_ground_resource_>=0 && (std::size_t)selected_ground_resource_<ground_resource_piles_.size()){
                // This direct box-to-dome transfer deliberately remains a debug-only
                // convenience. The exact box amount and condition are preserved.
                if(!move_ground_resource_to_storage((std::size_t)selected_ground_resource_)) return;
            }else return;
            break;
        case SelectionKind::Corridor:
            if(selected_corridor_>=0 && (std::size_t)selected_corridor_<corridor_network_.connections().size()){
                const std::size_t index=(std::size_t)selected_corridor_;
                const auto removed=corridor_network_.connections()[index];
                const ResourceBundle used=bundle_sum(removed.resource_plan.consumed(removed.construction.step),removed.site_buffer);
                walkway_network_.remove_corridor(index);
                corridor_network_.remove_corridor(index);
                // Routed construction is recovered where it physically existed.
                // Even in debug mode these boxes remain on the deleted corridor
                // footprint until explicitly moved to Storage Dome inventory.
                drop_path_resources(removed.path,2.6f,used);
            }
            break;
        case SelectionKind::Utility:
            if(selected_utility_>=0 && (std::size_t)selected_utility_<utility_network_.connections().size()){
                const std::size_t index=(std::size_t)selected_utility_;
                const auto removed=utility_network_.connections()[index];
                const ResourceBundle used=bundle_sum(removed.resource_plan.consumed(removed.construction.step),removed.site_buffer);
                walkway_network_.remove_utility(index);
                utility_network_.remove_connection(index);
                drop_path_resources(removed.path,1.4f,used);
            }
            break;
        case SelectionKind::Walkway:
            if(selected_walkway_>=0 && (std::size_t)selected_walkway_<walkway_network_.connections().size()){
                const std::size_t index=(std::size_t)selected_walkway_;
                const auto removed=walkway_network_.connections()[index];
                const ResourceBundle used=bundle_sum(removed.resource_plan.consumed(removed.construction.step),removed.site_buffer);
                walkway_network_.remove_connection(index);
                drop_path_resources(removed.path,1.8f,used);
            }
            break;
        case SelectionKind::Robot:
            // Robots are colony assets, not deletable world construction.
            return;
        case SelectionKind::None:
            return;
    }

    // Any operation that stores world indices must be cancelled after a delete;
    // deletion can compact vectors and therefore invalidate those indices.
    corridor_link_source_=-1;
    corridor_build_start_.reset();
    utility_build_start_.reset();
    walkway_build_start_.reset();
    clear_world_selection();
}

} // namespace game
