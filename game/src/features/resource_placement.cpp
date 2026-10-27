#include "game/game_app.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;

namespace {
Vec3 closest_point_segment(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<1.0e-6f) return a;
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return a+(b-a)*t;
}
float degrees_for_minus_z(Vec3 direction){
    direction.y=0.0f;
    if(length(direction)<0.001f) return 0.0f;
    direction=normalize(direction);
    return wrap_degrees(std::atan2(-direction.x,-direction.z)*180.0f/3.14159265358979323846f);
}
}

bool GameApp::resource_placement_valid(ResourceKind kind,StorageDomeSize storage_size,Vec3 position,float rotation_degrees) const {
    const ResourceMesh& candidate=resource_models_.mesh(kind,storage_size);
    const float half_field=(float)playfield_.size()*0.5f;
    const float a=radians(rotation_degrees);
    const float hx=std::abs(std::cos(a))*candidate.footprint_x*0.5f+std::abs(std::sin(a))*candidate.footprint_z*0.5f;
    const float hz=std::abs(std::sin(a))*candidate.footprint_x*0.5f+std::abs(std::cos(a))*candidate.footprint_z*0.5f;
    if(position.x-hx<-half_field || position.x+hx>half_field || position.z-hz<-half_field || position.z+hz>half_field) return false;

    const auto candidate_box=footprint(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z);
    constexpr float clearance=0.45f;
    for(const auto& placed:placed_habitats_){
        const HabitatMesh& other=habitat_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& placed:placed_resources_){
        const ResourceMesh& other=resource_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& pile:ground_resource_piles_){
        const float diameter=std::max(0.20f,pile.radius*2.0f);
        const auto pile_box=footprint(pile.position,0.0f,diameter,diameter);
        if(overlaps(candidate_box,pile_box,0.12f)) return false;
    }
    // Storage domes may snap beside networks, but their pressure/storage shell
    // itself must never cover an existing corridor or supply tray.
    if(kind==ResourceKind::StorageDome){
        const float radius=std::max(candidate.footprint_x,candidate.footprint_z)*0.5f;
        for(const auto& c:corridor_network_.connections()) for(std::size_t i=0;i+1<c.path.size();++i)
            if(point_segment_distance_xz(position,c.path[i],c.path[i+1])<radius+1.15f) return false;
        for(const auto& u:utility_network_.connections()) for(std::size_t i=0;i+1<u.path.size();++i)
            if(point_segment_distance_xz(position,u.path[i],u.path[i+1])<radius+0.45f) return false;
    }
    return true;
}

Vec3 GameApp::snap_storage_dome(Vec3 hit,StorageDomeSize size,float& rotation_degrees) const {
    const auto& mesh=resource_models_.mesh(ResourceKind::StorageDome,size);
    const float dome_radius=mesh.footprint_x*0.5f;
    float best=2.6f;
    Vec3 best_center{std::round(hit.x),0.0f,std::round(hit.z)};
    float best_rotation=rotation_degrees;

    auto consider=[&](const std::vector<Vec3>& path,float network_half_width){
        for(std::size_t i=0;i+1<path.size();++i){
            Vec3 q=closest_point_segment(hit,path[i],path[i+1]);
            Vec3 tangent=path[i+1]-path[i]; tangent.y=0.0f;
            if(length(tangent)<0.001f) continue;
            tangent=normalize(tangent);
            const Vec3 side{-tangent.z,0.0f,tangent.x};
            const float offset=dome_radius+network_half_width+1.85f;
            for(float sign:{-1.0f,1.0f}){
                Vec3 center=q+side*(offset*sign); center.y=0.0f;
                const float d=distance_xz(center,hit);
                if(d<best){
                    best=d;
                    best_center=center;
                    best_rotation=degrees_for_minus_z(q-center);
                }
            }
        }
    };
    for(const auto& c:corridor_network_.connections()) consider(c.path,1.15f);
    for(const auto& u:utility_network_.connections()) consider(u.path,0.45f);
    if(best<2.6f){ rotation_degrees=best_rotation; return best_center; }
    return best_center;
}

void GameApp::render_resource_placement(Renderer& r){
    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Resource) return;

    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;
    const StorageDomeSize storage_size=selected->resource_kind==ResourceKind::StorageDome?placement_storage_size_:StorageDomeSize::Small;
    float rotation=placement_rotation_degrees_;
    const Vec3 snapped=selected->resource_kind==ResourceKind::StorageDome?snap_storage_dome(hit,storage_size,rotation):Vec3{std::round(hit.x),0.0f,std::round(hit.z)};
    const auto& mesh=resource_models_.mesh(selected->resource_kind,storage_size);
    const auto plan=construction_costs::structure(selected->resource_kind,mesh);
    const bool valid=resource_placement_valid(selected->resource_kind,storage_size,snapped,rotation) && can_start_construction(plan);
    render_resource_mesh(r,mesh,snapped,rotation,true,valid);

    if(valid && input_.left_pressed){
        PlacedResource placed;
        placed.kind=selected->resource_kind;
        placed.position=snapped;
        placed.rotation_degrees=rotation;
        placed.storage_size=storage_size;
        placed.resource_plan=plan;
        placed_resources_.push_back(std::move(placed));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_storage_size_=StorageDomeSize::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
