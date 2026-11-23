#include "game/world/habitat_mesh.hpp"
#include "game/world/habitats/habitat_module.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/habitat_builder.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/habitats/screen_mounts.hpp"
#include <cmath>

namespace game {
namespace {



void append_tri(std::vector<game::runtime::Vec3>& dst,game::runtime::Vec3 a,game::runtime::Vec3 b,game::runtime::Vec3 c){
    dst.push_back(a); dst.push_back(b); dst.push_back(c);
}

void append_oriented_box(HabitatMesh& mesh,game::runtime::Vec3 center,game::runtime::Vec3 forward,float half_depth,float half_width,float half_height){
    forward.y=0.0f;
    forward=game::runtime::normalize(forward);
    if(game::runtime::length(forward)<0.001f) forward={1,0,0};
    const game::runtime::Vec3 side{-forward.z,0,forward.x};
    const game::runtime::Vec3 f=forward*half_depth,s=side*half_width,u{0,half_height,0};
    game::runtime::Vec3 v[8]={center-f-s-u,center+f-s-u,center+f-s+u,center-f-s+u,
                    center-f+s-u,center+f+s-u,center+f+s+u,center-f+s+u};
    const int faces[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
    for(const auto& q:faces){ append_tri(mesh.details,v[q[0]],v[q[1]],v[q[2]]); append_tri(mesh.details,v[q[0]],v[q[2]],v[q[3]]); }
    const int edges[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    for(const auto& e:edges){ mesh.edges.push_back(v[e[0]]); mesh.edges.push_back(v[e[1]]); }
}

void standardize_crew_interfaces(HabitatMesh& mesh,bool exterior_access){
    // Keep each building's authored macro shape, but normalize every crew hatch
    // to the same human-scale interface. This also makes future structures
    // automatically compatible with corridor/walkway geometry.
    const float bar=0.12f;
    const float half_clear=geometry::kDoorClearWidth*0.5f;
    const float base_y=mesh.walkable_floor_y;
    mesh.entrance_access.clear();
    for(auto& entrance:mesh.entrances){
        entrance.local_position.y=base_y;
        game::runtime::Vec3 n=entrance.local_normal; n.y=0.0f; n=game::runtime::normalize(n);
        if(game::runtime::length(n)<0.001f) n={1,0,0};
        const game::runtime::Vec3 side{-n.z,0,n.x};
        const game::runtime::Vec3 base{entrance.local_position.x,base_y,entrance.local_position.z};
        const game::runtime::Vec3 center{0,base_y,0};
        const float run=game::runtime::length(base-center);
        const game::runtime::Vec3 inward=run>0.01f?game::runtime::normalize(center-base):n*-1.0f;
        const mesh_portals::Aperture aisle{(base+center)*0.5f,inward,run*0.5f+0.02f,
            geometry::kDoorClearWidth*0.5f,base_y+0.025f,base_y+geometry::kDoorClearHeight};
        mesh.interior=mesh_portals::subtract(mesh.interior,aisle);
        mesh.details=mesh_portals::subtract(mesh.details,aisle);
        mesh.edges=mesh_portals::subtract_lines(mesh.edges,aisle);
        HabitatMesh deck;
        append_oriented_box(deck,(base+center)*0.5f-game::runtime::Vec3{0,0.045f,0},inward,
                            run*0.5f,geometry::kDoorClearWidth*0.5f,0.045f);
        mesh.interior.insert(mesh.interior.end(),deck.details.begin(),deck.details.end());
        append_oriented_box(mesh,base-side*(half_clear+bar)+game::runtime::Vec3{0,geometry::kDoorClearHeight*0.5f,0},n,0.10f,bar,geometry::kDoorClearHeight*0.5f);
        append_oriented_box(mesh,base+side*(half_clear+bar)+game::runtime::Vec3{0,geometry::kDoorClearHeight*0.5f,0},n,0.10f,bar,geometry::kDoorClearHeight*0.5f);
        append_oriented_box(mesh,base+game::runtime::Vec3{0,geometry::kDoorClearHeight+bar,0},n,0.10f,half_clear+bar*2.0f,bar);

        // Rebuild access AFTER size scaling, in the door's local basis. Step
        // width, rise and handrail offsets therefore remain human-sized and
        // the top landing ends exactly at the actual interior deck.
        habitats::Builder b;
        const int steps=std::max(1,(int)std::ceil(base_y/0.16f));
        const float tread=0.32f,rise=base_y/(float)steps;
        for(int i=0;i<steps;++i){
            const float top=base_y-rise*(float)i;
            const game::runtime::Vec3 center=base+n*(tread*((float)i+0.5f));
            HabitatMesh part;
            append_oriented_box(part,{center.x,top-0.05f,center.z},n,tread*0.5f,0.88f,0.05f);
            b.mesh.details.insert(b.mesh.details.end(),part.details.begin(),part.details.end());
            b.mesh.edges.insert(b.mesh.edges.end(),part.edges.begin(),part.edges.end());
        }
        for(float sign:{-1.0f,1.0f}){
            const auto near=base+side*(sign*0.98f)+game::runtime::Vec3{0,0.90f,0};
            const auto far=base+n*(tread*(float)steps)+side*(sign*0.98f)+game::runtime::Vec3{0,0.90f-base_y,0};
            b.cylinder_between(b.mesh.details,near,far,0.045f,6,true,true);
        }
        mesh.entrance_access.push_back({std::move(b.mesh.details),std::move(b.mesh.edges)});
    }
    if(exterior_access){
        // The navigation/placement footprint must include visible access
        // geometry, not end at the pressure hull while steps project beyond it.
        for(const auto& access:mesh.entrance_access) for(auto p:access.details){
            mesh.footprint_x=std::max(mesh.footprint_x,2.0f*std::abs(p.x));
            mesh.footprint_z=std::max(mesh.footprint_z,2.0f*std::abs(p.z));
        }
    }
}

bool scalable(HabitatKind kind){
    return kind==HabitatKind::OxygenGeneratorDome ||
           kind==HabitatKind::CantineDome ||
           kind==HabitatKind::BioDome ||
           kind==HabitatKind::CrewQuarters;
}

HabitatMesh scale_xz(const HabitatMesh& source,float scale){
    HabitatMesh out=source;
    const auto scale_vertices=[scale](std::vector<game::runtime::Vec3>& vertices){
        for(auto& v:vertices){ v.x*=scale; v.z*=scale; }
    };
    scale_vertices(out.shell);
    scale_vertices(out.foundation);
    scale_vertices(out.interior);
    scale_vertices(out.details);
    scale_vertices(out.glazing);
    scale_vertices(out.edges);
    for(auto* layer:{&out.lining,&out.deck,&out.service_metal,&out.fabric,&out.displays,&out.lamps,&out.markings})scale_vertices(*layer);
    for(auto& station:out.stations){station.low.x*=scale;station.low.z*=scale;station.high.x*=scale;station.high.z*=scale;}
    for(auto& screen:out.screens){screen.center.x*=scale;screen.center.z*=scale;screen.width*=scale;}
    for(auto& bunk:out.crew_bunks){
        bunk.access.x*=scale;
        bunk.access.z*=scale;
        bunk.feet.x*=scale;
        bunk.feet.z*=scale;
    }
    for(auto& entrance:out.entrances){
        entrance.local_position.x*=scale;
        entrance.local_position.z*=scale;
        // Horizontal non-uniform scaling changes diagonal normals. Transform
        // and renormalize them so corridor docking remains perpendicular.
        entrance.local_normal=game::runtime::normalize({entrance.local_normal.x/scale,entrance.local_normal.y,entrance.local_normal.z/scale});
    }
    for(auto& access:out.entrance_access){
        scale_vertices(access.details);
        scale_vertices(access.edges);
    }
    out.footprint_x*=scale;
    out.footprint_z*=scale;
    return out;
}

HabitatMesh base_mesh(HabitatKind kind){
    return habitats::module(kind).instantiate();
}

} // namespace

HabitatModelLibrary::HabitatModelLibrary(){
    game::runtime::GameJobs::shared().parallel_for(meshes_.size(),[&](std::size_t i){
        const HabitatKind kind=(HabitatKind)i;
        HabitatMesh standard=base_mesh(kind);
        // Crew entrances are no longer authored into a structure template.
        // A pressure hatch is cut into the completed hull only when a real
        // corridor/walkway connection is created.  Clearing the legacy author
        // metadata here also removes the fixed exterior stair sets before size
        // variants are generated.
        standard.entrances.clear();
        standard.entrance_access.clear();
        meshes_[i][(std::size_t)HabitatSizeTier::Standard]=standard;
        // SMALL is a horizontal-only variant for every habitat. Keep the full
        // authored height, floor Y and door clear height so colonists can walk
        // inside exactly as they can in STANDARD.
        meshes_[i][(std::size_t)HabitatSizeTier::Small]=scale_xz(standard,i==std::size_t(HabitatKind::OxygenGeneratorDome)?.90f:.76f);
        if(scalable(kind)){
            // Size tiers enlarge usable floor area while keeping the original
            // vertical dimensions/door height. This preserves the common
            // corridor pressure interface across all three levels.
            meshes_[i][(std::size_t)HabitatSizeTier::Mid]=scale_xz(standard,1.35f);
            meshes_[i][(std::size_t)HabitatSizeTier::Big]=scale_xz(standard,1.70f);
        }else{
            meshes_[i][(std::size_t)HabitatSizeTier::Mid]=standard;
            meshes_[i][(std::size_t)HabitatSizeTier::Big]=standard;
        }
        for(std::size_t tier=0;tier<meshes_[i].size();++tier){
            auto& variant=meshes_[i][tier];
            if(kind==HabitatKind::CrewQuarters){
                variant.crew_bunks.resize(crew_quarters_bed_count((HabitatSizeTier)tier));
                const float scale=variant.footprint_x/standard.footprint_x;
                habitats::Builder furniture;
                std::size_t bed_slot=0;
                for(const auto& bed:variant.crew_bunks){
                    interior_layout::StationScope station(variant,StationKind::Bed,bed_slot++);
                    const float x=bed.feet.x+.78f*scale;
                    const float z=bed.feet.z;
                    furniture.box(variant.interior,{x,1.28f,z},{1.94f*scale,.44f,.68f*scale},false);
                    furniture.box(variant.interior,{x,1.52f,z},{1.91f*scale,.12f,.66f*scale},false);
                    furniture.box(variant.interior,{x+.72f*scale,1.60f,z},{.34f*scale,.07f,.54f*scale},false);
                    furniture.box(variant.fabric,{x,1.589f,z},{1.87f*scale,.018f,.63f*scale},false);
                    furniture.box(variant.fabric,{x+.72f*scale,1.642f,z},{.35f*scale,.019f,.55f*scale},false);
                    for(float sign:{-1.f,1.f})furniture.box(variant.service_metal,{x,1.39f,z+sign*.34f*scale},{1.95f*scale,.045f,.035f},false);
                }
            }
            standardize_crew_interfaces(variant,kind==HabitatKind::AirLock);
            habitats::finish_screen_mounts(variant);
        }
    });
}

const HabitatMesh& HabitatModelLibrary::mesh(HabitatKind kind,HabitatSizeTier size) const {
    return meshes_[(size_t)kind][(size_t)size];
}

} // namespace game
