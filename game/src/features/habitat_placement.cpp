#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/gate_area.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/world_labels.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;


bool GameApp::placement_preserves_robot_escape(Vec3 position,float rotation_degrees,float footprint_x,float footprint_z) const {
    if(builder_robots_.empty()) return true;

    // Placement-time enclosure prevention. Simulate the candidate as a fully
    // solid structure before accepting the blueprint. Only currently completed
    // world geometry participates here; future blueprints are protected again
    // by the construction-stage topology guard before they become solid.
    constexpr float kPlacementRobotRadius=1.10f; // carrier hull + hard obstacle margin
    constexpr float cell=1.0f;
    const float half=std::max(2.0f,(float)playfield_.size()*0.5f-kPlacementRobotRadius-0.18f);
    const int side=std::max(5,(int)std::floor((2.0f*half)/cell)+1);
    const std::size_t count=(std::size_t)side*(std::size_t)side;
    std::vector<unsigned char> clear(count,0),exterior(count,0);
    auto index=[&](int x,int z){return (std::size_t)z*(std::size_t)side+(std::size_t)x;};
    auto point=[&](int x,int z){
        const float fx=-half+(2.0f*half)*(float)x/(float)(side-1);
        const float fz=-half+(2.0f*half)*(float)z/(float)(side-1);
        return Vec3{fx,0.0f,fz};
    };
    auto inside_rotated=[&](Vec3 p,Vec3 center,float rotation,float sx,float sz,float inflate){
        const Vec3 local=rotate_y(p-center,-rotation);
        return std::abs(local.x)<=sx*0.5f+inflate && std::abs(local.z)<=sz*0.5f+inflate;
    };
    auto blocked=[&](Vec3 p){
        if(inside_rotated(p,position,rotation_degrees,footprint_x,footprint_z,kPlacementRobotRadius)) return true;
        for(const auto& h:placed_habitats_){
            if(!h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            if(inside_rotated(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,kPlacementRobotRadius)) return true;
        }
        for(const auto& r:placed_resources_){
            if(!r.construction.complete()) continue;
            if(r.kind==ResourceKind::RobotWorkshop){
                // Workshops intentionally contain parked robots. Treat only the
                // shell as an obstacle for exterior robots; robots currently in
                // a workshop are exempted below.
                const auto& mesh=resource_models_.mesh(r);
                if(inside_rotated(p,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z,kPlacementRobotRadius)) return true;
                continue;
            }
            const auto& mesh=resource_models_.mesh(r);
            if(inside_rotated(p,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z,kPlacementRobotRadius)) return true;
        }
        const float corridor_clearance=geometry::kCorridorHalfWidth+kPlacementRobotRadius+0.18f;
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete()) continue;
            for(std::size_t i=1;i<c.path.size();++i)
                if(point_segment_distance_xz(p,c.path[i-1],c.path[i])<=corridor_clearance) return true;
        }
        const float walkway_clearance=geometry::kWalkwayHalfWidth+kPlacementRobotRadius+0.12f;
        for(const auto& w:walkway_network_.connections()){
            if(!w.construction.complete()) continue;
            for(std::size_t i=1;i<w.path.size();++i)
                if(point_segment_distance_xz(p,w.path[i-1],w.path[i])<=walkway_clearance) return true;
        }
        return false;
    };
    for(int z=0;z<side;++z) for(int x=0;x<side;++x) clear[index(x,z)]=(unsigned char)!blocked(point(x,z));

    std::vector<std::size_t> queue;queue.reserve(count);
    auto push=[&](int x,int z){const auto i=index(x,z);if(!clear[i]||exterior[i])return;exterior[i]=1;queue.push_back(i);};
    for(int i=0;i<side;++i){push(i,0);push(i,side-1);push(0,i);push(side-1,i);}
    constexpr int dx[4]{1,-1,0,0},dz[4]{0,0,1,-1};
    std::size_t q=0;
    auto edge_clear=[&](Vec3 a,Vec3 b){
        constexpr int samples=5;
        for(int i=0;i<=samples;++i){
            const float t=(float)i/(float)samples;
            if(blocked(a+(b-a)*t)) return false;
        }
        return true;
    };
    while(q<queue.size()){
        const auto i=queue[q++]; const int x=(int)(i%(std::size_t)side),z=(int)(i/(std::size_t)side);
        for(int d=0;d<4;++d){
            const int nx=x+dx[d],nz=z+dz[d]; if(nx<0||nz<0||nx>=side||nz>=side) continue;
            const auto ni=index(nx,nz); if(!clear[ni]||exterior[ni]) continue;
            if(edge_clear(point(x,z),point(nx,nz))){exterior[ni]=1;queue.push_back(ni);}
        }
    }

    auto robot_inside_completed_workshop=[&](const BuilderRobotState& robot){
        for(const auto& r:placed_resources_){
            if(r.kind!=ResourceKind::RobotWorkshop || !r.construction.complete()) continue;
            const auto& mesh=resource_models_.mesh(r);
            if(inside_rotated(robot.position,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z,0.0f)) return true;
        }
        return false;
    };
    for(const auto& robot:builder_robots_){
        if(robot_inside_completed_workshop(robot)) continue;
        if(inside_rotated(robot.position,position,rotation_degrees,footprint_x,footprint_z,kPlacementRobotRadius)) return false;
        int sx=(int)std::lround((robot.position.x+half)*(float)(side-1)/(2.0f*half));
        int sz=(int)std::lround((robot.position.z+half)*(float)(side-1)/(2.0f*half));
        sx=std::clamp(sx,0,side-1); sz=std::clamp(sz,0,side-1);
        auto start=index(sx,sz);
        if(!clear[start]){
            float best=std::numeric_limits<float>::max(); std::size_t best_i=count;
            for(int rz=std::max(0,sz-2);rz<=std::min(side-1,sz+2);++rz) for(int rx=std::max(0,sx-2);rx<=std::min(side-1,sx+2);++rx){
                const auto ri=index(rx,rz); if(!clear[ri]) continue;
                const float d=distance_xz(robot.position,point(rx,rz)); if(d<best){best=d;best_i=ri;}
            }
            if(best_i==count) return false;
            start=best_i;
        }
        if(!exterior[start]) return false;
    }
    return true;
}

bool GameApp::placement_valid(HabitatKind kind,HabitatSizeTier size,Vec3 position,float rotation_degrees) const {
    const HabitatMesh& candidate=habitat_models_.mesh(kind,size);
    const float half_field=(float)playfield_.size()*0.5f;
    const float ground_x=geometry::structure_ground_extent(candidate.footprint_x);
    const float ground_z=geometry::structure_ground_extent(candidate.footprint_z);
    const float a=radians(rotation_degrees);
    const float hx=std::abs(std::cos(a))*ground_x*0.5f+std::abs(std::sin(a))*ground_z*0.5f;
    const float hz=std::abs(std::sin(a))*ground_x*0.5f+std::abs(std::cos(a))*ground_z*0.5f;
    const float build_limit=half_field-geometry::kPlayfieldRobotPerimeterLane;
    if(position.x-hx<-build_limit || position.x+hx>build_limit ||
       position.z-hz<-build_limit || position.z+hz>build_limit) return false;

    const auto candidate_box=footprint(position,rotation_degrees,ground_x,ground_z);
    constexpr float clearance=geometry::kStructurePlacementGap;
    for(const auto& placed:placed_habitats_){
        const HabitatMesh& other=habitat_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,
                                       geometry::structure_ground_extent(other.footprint_x),
                                       geometry::structure_ground_extent(other.footprint_z));
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& placed:placed_resources_){
        if(placed.kind==ResourceKind::RobotWorkshop && overlaps(candidate_box,workshop::approach(placed),clearance)) return false;
        if(placed.kind==ResourceKind::StorageDome) for(int face:storage_loading::faces(placed))
            if(face>=0 && overlaps(candidate_box,storage_loading::apron(placed,face),clearance)) return false;
        const ResourceMesh& other=resource_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,
                                       geometry::structure_ground_extent(other.footprint_x),
                                       geometry::structure_ground_extent(other.footprint_z));
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& pile:ground_resource_piles_){
        const float diameter=std::max(0.20f,pile.radius*2.0f);
        const auto pile_box=footprint(pile.position,0.0f,diameter,diameter);
        if(overlaps(candidate_box,pile_box,0.12f)) return false;
    }
    // Structures cannot be placed over an already-built/planned pressure
    // corridor.  Use the footprint circumradius so rotated long buildings are
    // conservative rather than leaving a corner cutting through the shell.
    const float corridor_clearance=0.5f*std::sqrt(candidate.footprint_x*candidate.footprint_x+
                                                   candidate.footprint_z*candidate.footprint_z)+
                                   geometry::kCorridorHalfWidth+geometry::kRouteClearance;
    for(const auto& corridor:corridor_network_.connections()){
        for(std::size_t i=0;i+1<corridor.path.size();++i)
            if(point_segment_distance_xz(position,corridor.path[i],corridor.path[i+1])<corridor_clearance) return false;
    }
    PlacedHabitat candidate_item;candidate_item.kind=kind;candidate_item.size=size;candidate_item.position=position;candidate_item.rotation_degrees=rotation_degrees;
    if(!gate_layout_clear(&candidate_item,nullptr))return false;
    if(!placement_preserves_robot_escape(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z)) return false;
    return true;
}

void GameApp::render_habitat_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Habitat) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    Vec3 snapped{std::round(hit.x),0.0f,std::round(hit.z)};
    const HabitatSizeTier size=placement_habitat_size_;
    const auto plan=construction_costs::habitat(selected->habitat_kind,size,habitat_models_.mesh(selected->habitat_kind,size));
    const bool valid=placement_valid(selected->habitat_kind,size,snapped,placement_rotation_degrees_) && can_start_construction(plan);

    render_habitat_mesh(r,habitat_models_.mesh(selected->habitat_kind,size),snapped,placement_rotation_degrees_,true,valid);

    if(valid && input_.left_pressed){
        PlacedHabitat placed;
        placed.kind=selected->habitat_kind;
        std::uint32_t serial=1;
        for(const auto& existing:placed_habitats_){
            if(existing.kind==placed.kind && existing.size==size) serial=std::max(serial,existing.instance_serial+1);
        }
        placed.instance_serial=serial;
        placed.placement_sequence=next_blueprint_placement_sequence_++;
        placed.position=snapped;
        placed.rotation_degrees=placement_rotation_degrees_;
        placed.size=size;
        placed.resource_plan=plan;
        placed_habitats_.push_back(std::move(placed));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_habitat_size_=HabitatSizeTier::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
