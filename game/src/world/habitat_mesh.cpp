#include "game/world/habitat_mesh.hpp"
#include "game/world/habitats/habitat_module.hpp"
#include "game/world/geometry_standards.hpp"
#include <cmath>

namespace game {
namespace {



void append_tri(std::vector<eng::Vec3>& dst,eng::Vec3 a,eng::Vec3 b,eng::Vec3 c){
    dst.push_back(a); dst.push_back(b); dst.push_back(c);
}

void append_oriented_box(HabitatMesh& mesh,eng::Vec3 center,eng::Vec3 forward,float half_depth,float half_width,float half_height){
    forward.y=0.0f;
    forward=eng::normalize(forward);
    if(eng::length(forward)<0.001f) forward={1,0,0};
    const eng::Vec3 side{-forward.z,0,forward.x};
    const eng::Vec3 f=forward*half_depth,s=side*half_width,u{0,half_height,0};
    eng::Vec3 v[8]={center-f-s-u,center+f-s-u,center+f-s+u,center-f-s+u,
                    center-f+s-u,center+f+s-u,center+f+s+u,center-f+s+u};
    const int faces[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
    for(const auto& q:faces){ append_tri(mesh.details,v[q[0]],v[q[1]],v[q[2]]); append_tri(mesh.details,v[q[0]],v[q[2]],v[q[3]]); }
    const int edges[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    for(const auto& e:edges){ mesh.edges.push_back(v[e[0]]); mesh.edges.push_back(v[e[1]]); }
}

void standardize_crew_interfaces(HabitatMesh& mesh){
    // Keep each building's authored macro shape, but normalize every crew hatch
    // to the same human-scale interface. This also makes future structures
    // automatically compatible with corridor/walkway geometry.
    const float bar=0.12f;
    const float half_clear=geometry::kDoorClearWidth*0.5f;
    const float base_y=geometry::kHabitatFloorY;
    for(auto& entrance:mesh.entrances){
        entrance.local_position.y=geometry::kEntranceSliceY;
        eng::Vec3 n=entrance.local_normal; n.y=0.0f; n=eng::normalize(n);
        if(eng::length(n)<0.001f) n={1,0,0};
        const eng::Vec3 side{-n.z,0,n.x};
        const eng::Vec3 base{entrance.local_position.x,base_y,entrance.local_position.z};
        append_oriented_box(mesh,base-side*(half_clear+bar)+eng::Vec3{0,geometry::kDoorClearHeight*0.5f,0},n,0.10f,bar,geometry::kDoorClearHeight*0.5f);
        append_oriented_box(mesh,base+side*(half_clear+bar)+eng::Vec3{0,geometry::kDoorClearHeight*0.5f,0},n,0.10f,bar,geometry::kDoorClearHeight*0.5f);
        append_oriented_box(mesh,base+eng::Vec3{0,geometry::kDoorClearHeight+bar,0},n,0.10f,half_clear+bar*2.0f,bar);
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
    const auto scale_vertices=[scale](std::vector<eng::Vec3>& vertices){
        for(auto& v:vertices){ v.x*=scale; v.z*=scale; }
    };
    scale_vertices(out.shell);
    scale_vertices(out.foundation);
    scale_vertices(out.interior);
    scale_vertices(out.details);
    scale_vertices(out.glazing);
    scale_vertices(out.edges);
    for(auto& entrance:out.entrances){
        entrance.local_position.x*=scale;
        entrance.local_position.z*=scale;
        // Horizontal non-uniform scaling changes diagonal normals. Transform
        // and renormalize them so corridor docking remains perpendicular.
        entrance.local_normal=eng::normalize({entrance.local_normal.x/scale,entrance.local_normal.y,entrance.local_normal.z/scale});
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
    for(std::size_t i=0;i<(std::size_t)HabitatKind::Count;++i){
        const HabitatKind kind=(HabitatKind)i;
        HabitatMesh standard=base_mesh(kind);
        standardize_crew_interfaces(standard);
        meshes_[i][(std::size_t)HabitatSizeTier::Standard]=standard;
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
    }
}

const HabitatMesh& HabitatModelLibrary::mesh(HabitatKind kind,HabitatSizeTier size) const {
    return meshes_[(size_t)kind][(size_t)size];
}

} // namespace game
