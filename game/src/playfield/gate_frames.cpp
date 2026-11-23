#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/world/corridor/detail.hpp"
#include <bit>
#include <numeric>
namespace game {
void GameApp::render_gate_connection_frames(game::runtime::SceneRenderer& renderer) const {
    using namespace game::runtime;using namespace world_math;
    const auto& runs=corridor_network_.connections();
    std::vector<std::uint64_t> signature{runs.size()};
    for(const auto& run:runs){
        signature.insert(signature.end(),{std::uint64_t(run.construction.complete()),std::uint64_t(run.endpoint_a.kind),run.endpoint_a.index,std::uint64_t(run.endpoint_b.kind),run.endpoint_b.index,run.path.size()});
        for(const auto p:run.path)signature.insert(signature.end(),{std::bit_cast<std::uint32_t>(p.x),std::bit_cast<std::uint32_t>(p.y),std::bit_cast<std::uint32_t>(p.z)});
    }
    auto& parent=gate_topology_roots_;
    const auto root=[&](std::size_t i){while(parent[i]!=i)i=parent[i];return i;};
    if(signature!=gate_topology_signature_){
        gate_topology_signature_=std::move(signature);parent.resize(runs.size());std::iota(parent.begin(),parent.end(),0);
        for(std::size_t i=0;i<runs.size();++i)if(runs[i].construction.complete())for(const auto* endpoint:{&runs[i].endpoint_a,&runs[i].endpoint_b})
            if(endpoint->kind==CorridorEndpointKind::Corridor&&endpoint->index<runs.size()&&runs[endpoint->index].construction.complete())parent[root(i)]=root(endpoint->index);
        // Automatic X junctions are real pressure connections even without an
        // explicit host endpoint. Recompute this graph only on topology edits.
        for(std::size_t i=0;i<runs.size();++i)for(std::size_t j=i+1;j<runs.size();++j){
            if(!runs[i].construction.complete()||!runs[j].construction.complete()||root(i)==root(j))continue;
            bool joined=false;
            for(std::size_t a=1;a<runs[i].path.size()&&!joined;++a)for(std::size_t b=1;b<runs[j].path.size();++b){Vec3 hit;
                if(corridor_detail::mergeable_corridor_crossing(runs[i].path[a-1],runs[i].path[a],runs[j].path[b-1],runs[j].path[b],hit)){joined=true;break;}
            }
            if(joined)parent[root(i)]=root(j);
        }
    }
    std::vector<int> destinations(runs.size());
    for(std::size_t i=0;i<runs.size();++i)if(runs[i].construction.complete())for(const auto* e:{&runs[i].endpoint_a,&runs[i].endpoint_b}){
        const bool built=e->kind==CorridorEndpointKind::Habitat?e->index<placed_habitats_.size()&&placed_habitats_[e->index].construction.complete():
            e->kind==CorridorEndpointKind::Resource&&e->index<placed_resources_.size()&&placed_resources_[e->index].kind==ResourceKind::StorageDome&&placed_resources_[e->index].construction.complete();
        if(built)++destinations[root(i)];
    }
    const auto connected=[&](CorridorEndpointKind kind,std::size_t index,const CorridorDoor& door){
        bool any=false;
        for(std::size_t i=0;i<runs.size();++i)if(runs[i].construction.complete()&&destinations[root(i)]>=2)
            for(const auto* e:{&runs[i].endpoint_a,&runs[i].endpoint_b})if(e->kind==kind&&e->index==index){
                any=true;if(distance_xz(e->door.wall_position,door.wall_position)<.40f)return true;
            }
        return kind==CorridorEndpointKind::Habitat&&index<placed_habitats_.size()&&placed_habitats_[index].kind==HabitatKind::AirLock&&door.entrance_index<=-2&&any;
    };
    const auto frame=[&](Vec3 base,Vec3 normal,float width,float height,bool accessible,float depth=.16f){
        normal.y=0;normal=normalize(normal);if(length(normal)<.1f)return;
        const Vec3 side{-normal.z,0,normal.x};std::vector<Vec3> lines;
        for(float face:{-1.f,1.f}){
            const auto p=base+normal*(face*depth);const auto a=p-side*(width*.5f+.055f),b=p+side*(width*.5f+.055f),c=b+Vec3{0,height+.055f,0},d=a+Vec3{0,height+.055f,0};
            lines.insert(lines.end(),{a,b,b,c,c,d,d,a});
        }
        renderer.draw_lines_3d(lines,{.025f,.035f,.04f,1},6.f);
        renderer.draw_lines_3d(lines,accessible?Color{.18f,.95f,.34f,1}:Color{1.f,.16f,.12f,1},3.2f);
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i)if(placed_habitats_[i].construction.complete())
        for(const auto& door:habitat_doors(i))frame(door.wall_position,door.normal,geometry::kDoorClearWidth,geometry::kDoorClearHeight,connected(CorridorEndpointKind::Habitat,i,door));
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& item=placed_resources_[i];if(!item.construction.complete())continue;
        if(item.kind==ResourceKind::StorageDome){
            const auto doors=storage_dome_doors(i);for(const auto& door:doors)frame(door.wall_position,door.normal,geometry::kDoorClearWidth,geometry::kDoorClearHeight,connected(CorridorEndpointKind::Resource,i,door),.34f);
            const auto& mesh=resource_models_.mesh(item);const float wall=storage_dome_diameter_m(item.storage_size)*(5.35f/13.f);
            for(int face=0;face<4;++face){const auto normal=rotate_y(storage_loading::local_normal(face),item.rotation_degrees);
                const auto base=item.position+normal*wall+Vec3{0,mesh.walkable_floor_y,0};
                const bool occupied=std::any_of(doors.begin(),doors.end(),[&](const auto& door){return distance_xz(door.wall_position,base)<1.1f&&dot(normal,door.normal)>.8f;});
                if(!occupied)frame(base,normal,geometry::kDoorClearWidth,geometry::kDoorClearHeight,false,.34f);
            }
            for(int face:storage_loading::faces(item))if(face>=0){const auto base=storage_loading::mouth(item,face)+Vec3{0,.15f,0};
                bool ready=face==item.storage_loading_face?item.storage_loading_ready:false;
                for(std::size_t slot=0;slot<item.storage_extra_faces.size();++slot)if(face==item.storage_extra_faces[slot])ready=item.storage_extra_ready[slot];
                frame(base,storage_loading::normal(item,face),2*storage_loading::kHalfWidth,storage_loading::kHeight,ready);
            }
        }else if(item.kind==ResourceKind::RobotWorkshop){
            frame(workshop::world(item,{0,workshop::kDeck,-workshop::kHalfZ}),rotate_y({0,0,-1},item.rotation_degrees),2*workshop::kGateHalf,2.9f,true);
        }
    }
    for(const auto& corridor:runs)if(corridor.construction.complete())for(const auto* e:{&corridor.endpoint_a,&corridor.endpoint_b})
        if(e->kind==CorridorEndpointKind::OpenDoor&&!e->suppress_open_door)frame({e->position.x,e->position.y+geometry::kCorridorDeckY,e->position.z},e->tangent,geometry::kDoorClearWidth,geometry::kDoorClearHeight,false);
}
}
