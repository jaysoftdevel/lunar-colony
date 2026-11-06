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
        if(lunar_lander_segment_blocked(lander_,p,p,kPlacementRobotRadius))return true;
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

bool GameApp::structure_routes_clear(Vec3 position,float rotation,float width,float depth) const {
    const auto base=footprint(position,rotation,width,depth);
    const auto clear=[&](const auto& connections,float half_width,float gap){
        for(const auto& run:connections)for(std::size_t i=1;i<run.path.size();++i){
            const auto a=run.path[i-1],b=run.path[i],d=b-a;
            const float run_length=std::hypot(d.x,d.z);
            if(run_length<0.0001f)continue;
            const float angle=std::atan2(d.x,d.z)*180.0f/3.14159265359f;
            if(overlaps(base,footprint((a+b)*.5f,angle,2*half_width,run_length),gap))return false;
        }
        return true;
    };
    return clear(corridor_network_.connections(),geometry::kCorridorHalfWidth,geometry::kRouteClearance) &&
        clear(utility_network_.connections(),geometry::kSupplyJunctionWidth*.5f,.10f) &&
        clear(walkway_network_.connections(),geometry::kWalkwayHalfWidth,.10f);
}

std::optional<GameApp::StructureAttachment> GameApp::preview_structure_attachment(
    Vec3 hit,const PlacedHabitat* habitat,const PlacedResource* resource) const {
    const auto base=habitat ? [&]{
        const auto& mesh=habitat_models_.mesh(*habitat);
        return footprint({},habitat->rotation_degrees,mesh.footprint_x,mesh.footprint_z);
    }() : [&]{
        const auto& mesh=resource_models_.mesh(*resource);
        return footprint({},resource->rotation_degrees,mesh.footprint_x,mesh.footprint_z);
    }();

    struct FreeEnd {Vec3 center,position;float distance;std::size_t index;bool corridor;};
    std::vector<FreeEnd> ends;
    const auto consider=[&](const auto& run,std::size_t index,bool first,bool corridor){
        if(run.path.size()<2)return;
        const auto end=first?run.path.front():run.path.back();
        auto outward=end-(first?run.path[1]:run.path[run.path.size()-2]);
        outward.y=0;
        if(length(outward)<.001f)return;
        outward=normalize(outward);
        const float reach=projected_radius(base,{outward.x,outward.z});
        auto center=end+outward*(reach+(corridor?geometry::kStructureConnectionReserve+2.6f:1.5f));
        center.y=0;
        const float distance=distance_xz(hit,center);
        if(distance>2.75f)return;
        if(!corridor){
            // Supply branches use Open endpoints too. A point shared with another
            // run is a junction, not a free terminal.
            const auto& utilities=utility_network_.connections();
            for(std::size_t i=0;i<utilities.size();++i){
                if(i==index)continue;
                const auto& path=utilities[i].path;
                for(std::size_t j=1;j<path.size();++j)
                    if(point_segment_distance_xz(end,path[j-1],path[j])<.15f)return;
            }
        }
        ends.push_back({center,end,distance,index,corridor});
    };
    if(habitat || resource->kind==ResourceKind::StorageDome){
        const auto& corridors=corridor_network_.connections();
        for(std::size_t i=0;i<corridors.size();++i){
            const auto& run=corridors[i];
            if(run.endpoint_a.kind==CorridorEndpointKind::OpenDoor&&!run.endpoint_a.suppress_open_door)consider(run,i,true,true);
            if(run.endpoint_b.kind==CorridorEndpointKind::OpenDoor&&!run.endpoint_b.suppress_open_door)consider(run,i,false,true);
        }
    }
    const auto& utilities=utility_network_.connections();
    for(std::size_t i=0;i<utilities.size();++i){
        const auto& run=utilities[i];
        if(run.kind!=UtilityKind::SupplyChannel)continue;
        if(run.endpoint_a.kind==UtilityEndpointKind::Open)consider(run,i,true,false);
        if(run.endpoint_b.kind==UtilityEndpointKind::Open)consider(run,i,false,false);
    }
    if(ends.empty())return std::nullopt;
    std::stable_sort(ends.begin(),ends.end(),[](const auto& a,const auto& b){return a.distance<b.distance;});

    auto habitats=placed_habitats_;
    auto resources=placed_resources_;
    if(habitat)habitats.push_back(*habitat);
    else resources.push_back(*resource);
    for(const auto& end:ends){
        if(habitat){
            habitats.back().position=end.center;
            if(!placement_valid(habitat->kind,habitat->size,end.center,habitat->rotation_degrees))continue;
        }else{
            resources.back().position=end.center;
            if(!resource_placement_valid(resource->kind,resource->size,resource->storage_size,end.center,resource->rotation_degrees))continue;
        }
        const auto index=habitat?habitats.size()-1:resources.size()-1;
        StructureAttachment result{end.center,{},{}};
        if(end.corridor){
            const CorridorBuildAnchor from{CorridorEndpointKind::Corridor,end.index,end.position};
            const CorridorBuildAnchor to{habitat?CorridorEndpointKind::Habitat:CorridorEndpointKind::Resource,index,end.position};
            auto route=corridor_network_.preview_built_corridor(from,to,habitats,habitat_models_,resources,resource_models_);
            if(!route || !corridor_gate_layout_clear(*route))continue;
            route->resource_plan=construction_costs::corridor(route->path,1);
            result.corridor=std::move(route);
        }else{
            const UtilityBuildAnchor from{UtilityEndpointKind::Open,0,end.position};
            const UtilityBuildAnchor to{habitat?UtilityEndpointKind::Habitat:UtilityEndpointKind::Resource,index,end.center};
            auto route=utility_network_.preview_connection(UtilityKind::SupplyChannel,from,to,habitats,habitat_models_,resources,resource_models_);
            if(!route || !gate_route_clear(route->path,geometry::kSupplyTrayWidth*.5f+.2f))continue;
            route->resource_plan=construction_costs::supply_channel(route->path,route->junctions.size());
            result.utility=std::move(route);
        }
        return result;
    }
    return std::nullopt;
}

void GameApp::render_structure_attachment(Renderer& r,const StructureAttachment& attachment) const {
    const auto& edges=attachment.corridor?attachment.corridor->mesh.edges:attachment.utility->mesh.edges;
    r.draw_lines_3d(edges,theme_.accent,2.0f);
}

void GameApp::commit_structure_attachment(StructureAttachment attachment){
    if(attachment.corridor){
        finalize_placement(*attachment.corridor);
        corridor_network_.append_corridor(std::move(*attachment.corridor));
    }else{
        finalize_placement(*attachment.utility);
        utility_network_.append_connection(std::move(*attachment.utility));
    }
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

    if(!lander_placement_clear(position,rotation_degrees,ground_x,ground_z) || !trader_placement_clear(position,rotation_degrees,ground_x,ground_z))return false;
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
    if(!structure_routes_clear(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z))return false;
    PlacedHabitat candidate_item;candidate_item.kind=kind;candidate_item.size=size;candidate_item.position=position;candidate_item.rotation_degrees=rotation_degrees;
    if(!gate_layout_clear(&candidate_item,nullptr))return false;
    if(!astronaut_placement_clear(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z))return false;
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
    PlacedHabitat candidate;
    candidate.kind=selected->habitat_kind;
    candidate.size=size;
    candidate.rotation_degrees=placement_rotation_degrees_;
    auto attachment=preview_structure_attachment(hit,&candidate,nullptr);
    if(attachment)snapped=attachment->position;
    const auto plan=construction_costs::habitat(selected->habitat_kind,size,habitat_models_.mesh(selected->habitat_kind,size));
    const bool valid=(attachment || placement_valid(selected->habitat_kind,size,snapped,placement_rotation_degrees_)) &&
        can_start_construction(plan);

    render_habitat_mesh(r,habitat_models_.mesh(selected->habitat_kind,size),snapped,placement_rotation_degrees_,true,valid);

    if(valid && attachment)render_structure_attachment(r,*attachment);

    if(valid && input_.left_pressed){
        PlacedHabitat placed;
        placed.kind=selected->habitat_kind;
        std::uint32_t serial=1;
        for(const auto& existing:placed_habitats_){
            if(existing.kind==placed.kind && existing.size==size) serial=std::max(serial,existing.instance_serial+1);
        }
        placed.instance_serial=serial;
        placed.position=snapped;
        placed.rotation_degrees=placement_rotation_degrees_;
        placed.size=size;
        placed.resource_plan=plan;
        finalize_placement(placed);
        placed_habitats_.push_back(std::move(placed));
        if(attachment)commit_structure_attachment(std::move(*attachment));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_habitat_size_=HabitatSizeTier::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
