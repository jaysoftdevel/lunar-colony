#include "game/world/habitat_mesh.hpp"
#include "game/world/habitats/habitat_generators.hpp"

namespace game {
namespace {

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
    switch(kind){
        case HabitatKind::AirLock: return habitats::make_airlock();
        case HabitatKind::OxygenGeneratorDome: return habitats::make_oxygen_dome();
        case HabitatKind::CantineDome: return habitats::make_cantine_dome();
        case HabitatKind::BioDome: return habitats::make_bio_dome();
        case HabitatKind::CrewQuarters: return habitats::make_crew_quarters();
        case HabitatKind::ProcessingFacility: return habitats::make_processing_facility();
        case HabitatKind::Factory: return habitats::make_factory();
        case HabitatKind::Manufactory: return habitats::make_manufactory();
        case HabitatKind::SickBay: return habitats::make_sick_bay();
        case HabitatKind::Laboratory: return habitats::make_laboratory();
        default: return habitats::make_airlock();
    }
}

} // namespace

HabitatModelLibrary::HabitatModelLibrary(){
    for(std::size_t i=0;i<(std::size_t)HabitatKind::Count;++i){
        const HabitatKind kind=(HabitatKind)i;
        const HabitatMesh standard=base_mesh(kind);
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
