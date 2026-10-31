#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/geometry_standards.hpp"
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

bool GameApp::resource_placement_valid(ResourceKind kind,ResourceSizeTier size,StorageDomeSize storage_size,Vec3 position,float rotation_degrees) const {
    const ResourceMesh& candidate=kind==ResourceKind::StorageDome?resource_models_.mesh(kind,storage_size):resource_models_.mesh(kind,size);
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
        if(placed.kind==ResourceKind::RobotWorkshop && overlaps(candidate_box,workshop::approach(placed),clearance)) return false;
        if(placed.kind==ResourceKind::StorageDome) for(int face:storage_loading::faces(placed))
            if(face>=0 && overlaps(candidate_box,storage_loading::apron(placed,face),clearance)) return false;
        const ResourceMesh& other=resource_models_.mesh(placed);
        const auto other_box=footprint(placed.position,placed.rotation_degrees,other.footprint_x,other.footprint_z);
        if(overlaps(candidate_box,other_box,clearance)) return false;
    }
    for(const auto& pile:ground_resource_piles_){
        const float diameter=std::max(0.20f,pile.radius*2.0f);
        const auto pile_box=footprint(pile.position,0.0f,diameter,diameter);
        if(overlaps(candidate_box,pile_box,0.12f)) return false;
    }
    // No structure may be placed through an existing pressure corridor.
    const float corridor_radius=0.5f*std::sqrt(candidate.footprint_x*candidate.footprint_x+
                                                candidate.footprint_z*candidate.footprint_z)+
                                geometry::kCorridorHalfWidth+geometry::kRouteClearance;
    for(const auto& c:corridor_network_.connections()) for(std::size_t i=0;i+1<c.path.size();++i)
        if(point_segment_distance_xz(position,c.path[i],c.path[i+1])<corridor_radius) return false;

    if(kind==ResourceKind::RobotWorkshop){
        PlacedResource item;item.kind=kind;item.size=ResourceSizeTier::Standard;
        item.position=position;item.rotation_degrees=rotation_degrees;
        const auto apron=workshop::approach(item);
        for(float x:{-apron.half_x,apron.half_x}) for(float z:{-apron.half_z,apron.half_z}){
            const Vec2 q{apron.center.x+apron.axis_x.x*x+apron.axis_z.x*z,apron.center.y+apron.axis_x.y*x+apron.axis_z.y*z};
            if(std::abs(q.x)>half_field-0.20f || std::abs(q.y)>half_field-0.20f) return false;
        }
        for(const auto& h:placed_habitats_){const auto& m=habitat_models_.mesh(h);
            if(overlaps(apron,footprint(h.position,h.rotation_degrees,m.footprint_x,m.footprint_z),clearance)) return false;}
        for(const auto& r:placed_resources_){const auto& m=resource_models_.mesh(r);
            if(overlaps(apron,footprint(r.position,r.rotation_degrees,m.footprint_x,m.footprint_z),clearance)) return false;
            if(r.kind==ResourceKind::StorageDome && overlaps(apron,storage_loading::apron(r),clearance)) return false;}
        for(const auto& pile:ground_resource_piles_)
            if(overlaps(apron,footprint(pile.position,0,2*pile.radius,2*pile.radius),1.12f)) return false;
        for(const auto& c:corridor_network_.connections()) for(std::size_t j=1;j<c.path.size();++j)
            if(workshop::approach_hits(item,c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+geometry::kRouteClearance)) return false;
        for(const auto& w:walkway_network_.connections()) for(std::size_t j=1;j<w.path.size();++j)
            if(workshop::approach_hits(item,w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.40f)) return false;
        if(!workshop_accessible(item)) return false;
    }

    // Storage domes also keep their pressure/storage shell clear of service trays.
    if(kind==ResourceKind::StorageDome){
        const float radius=std::max(candidate.footprint_x,candidate.footprint_z)*0.5f;
        for(const auto& u:utility_network_.connections()) for(std::size_t i=0;i+1<u.path.size();++i)
            if(point_segment_distance_xz(position,u.path[i],u.path[i+1])<radius+0.45f) return false;

        // Step 1 is still only the dome shell, but it must leave at least one
        // legal inter-door sector for step 2.  This prevents a player from
        // freezing a footprint that can never receive its mandatory cargo gate.
        PlacedResource pending;pending.kind=ResourceKind::StorageDome;pending.storage_size=storage_size;
        pending.position=position;pending.rotation_degrees=rotation_degrees;
        bool has_gate_sector=false;
        for(int face:storage_loading::kCargoFaces){
            pending.storage_loading_face=(std::int8_t)face;
            if(storage_gate_face_clear(pending,face)){has_gate_sector=true;break;}
        }
        if(!has_gate_sector) return false;
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

    // Storage placement step 2: the dome footprint is already frozen.  Move
    // the pointer around the shell to select the nearest free inter-door cargo
    // sector.  No storage gate exists before this second click is accepted.
    if(selected->resource_kind==ResourceKind::StorageDome && storage_gate_placement_active_){
        PlacedResource pending;
        pending.kind=ResourceKind::StorageDome;
        pending.position=storage_gate_pending_position_;
        pending.rotation_degrees=storage_gate_pending_rotation_degrees_;
        pending.storage_size=storage_gate_pending_size_;
        pending.storage_loading_face=-1;

        Vec3 local=rotate_y(hit-pending.position,-pending.rotation_degrees); local.y=0.0f;
        if(length(local)<0.001f) local={1,0,1};
        local=normalize(local);
        int selected_face=storage_loading::kCargoFaces[0];
        float best_dot=-2.0f;
        for(int face:storage_loading::kCargoFaces){
            const float score=dot(local,storage_loading::local_normal(face));
            if(score>best_dot){best_dot=score;selected_face=face;}
        }
        pending.storage_loading_face=(std::int8_t)selected_face;

        const auto& mesh=resource_models_.mesh(ResourceKind::StorageDome,pending.storage_size);
        const auto plan=construction_costs::structure(ResourceKind::StorageDome,mesh);
        const bool gate_clear=storage_gate_face_clear(pending,selected_face);
        const bool valid=gate_clear && can_start_construction(plan);
        render_resource_mesh(r,mesh,pending.position,pending.rotation_degrees,true,true);
        render_storage_loading_gate(r,pending,0,true);

        const auto bay=storage_loading::apron(pending,selected_face);
        auto corner=[&](float x,float z){
            return Vec3{bay.center.x+bay.axis_x.x*x+bay.axis_z.x*z,0.055f,
                        bay.center.y+bay.axis_x.y*x+bay.axis_z.y*z};
        };
        const Vec3 a=corner(-bay.half_x,-bay.half_z),b=corner(bay.half_x,-bay.half_z);
        const Vec3 c=corner(bay.half_x,bay.half_z),d=corner(-bay.half_x,bay.half_z);
        r.draw_lines_3d({a,b,b,c,c,d,d,a},valid?Color{0.22f,1.0f,0.48f,1}:Color{1.0f,0.22f,0.12f,1},2.8f);

        if(valid && input_.left_pressed){
            std::uint32_t serial=1;
            for(const auto& existing:placed_resources_)
                if(existing.kind==ResourceKind::StorageDome && existing.storage_size==pending.storage_size)
                    serial=std::max(serial,existing.instance_serial+1);
            pending.instance_serial=serial;
            pending.placement_sequence=next_blueprint_placement_sequence_++;
            pending.size=ResourceSizeTier::Standard;
            pending.resource_plan=plan;
            placed_resources_.push_back(std::move(pending));
            storage_gate_placement_active_=false;
            build_catalog_.clear_selection();
            placement_rotation_degrees_=0.0f;
            placement_resource_size_=ResourceSizeTier::Small;
            placement_storage_size_=StorageDomeSize::Small;
            corridor_build_start_.reset();
        }
        return;
    }

    const StorageDomeSize storage_size=selected->resource_kind==ResourceKind::StorageDome?placement_storage_size_:StorageDomeSize::Small;
    const ResourceSizeTier resource_size=(selected->resource_kind==ResourceKind::StorageDome || selected->resource_kind==ResourceKind::RobotWorkshop)?ResourceSizeTier::Standard:placement_resource_size_;
    float rotation=placement_rotation_degrees_;
    const Vec3 snapped=selected->resource_kind==ResourceKind::StorageDome?snap_storage_dome(hit,storage_size,rotation):Vec3{std::round(hit.x),0.0f,std::round(hit.z)};
    const auto& mesh=selected->resource_kind==ResourceKind::StorageDome?
        resource_models_.mesh(selected->resource_kind,storage_size):resource_models_.mesh(selected->resource_kind,resource_size);
    const auto plan=construction_costs::structure(selected->resource_kind,mesh);
    const bool valid=resource_placement_valid(selected->resource_kind,resource_size,storage_size,snapped,rotation) && can_start_construction(plan);
    render_resource_mesh(r,mesh,snapped,rotation,true,valid);
    if(selected->resource_kind==ResourceKind::RobotWorkshop){
        PlacedResource item;item.kind=ResourceKind::RobotWorkshop;item.position=snapped;item.rotation_degrees=rotation;
        render_workshop_gate(r,item,true,valid);
    }

    if(valid && input_.left_pressed){
        if(selected->resource_kind==ResourceKind::StorageDome){
            storage_gate_pending_position_=snapped;
            storage_gate_pending_rotation_degrees_=rotation;
            storage_gate_pending_size_=storage_size;
            storage_gate_placement_active_=true;
            return;
        }

        PlacedResource placed;
        placed.kind=selected->resource_kind;
        std::uint32_t serial=1;
        for(const auto& existing:placed_resources_){
            if(existing.kind!=placed.kind) continue;
            if(existing.size==resource_size) serial=std::max(serial,existing.instance_serial+1);
        }
        placed.instance_serial=serial;
        placed.placement_sequence=next_blueprint_placement_sequence_++;
        placed.position=snapped;
        placed.rotation_degrees=rotation;
        placed.storage_size=storage_size;
        placed.size=resource_size;
        placed.resource_plan=plan;
        placed_resources_.push_back(std::move(placed));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_resource_size_=ResourceSizeTier::Small;
        placement_storage_size_=StorageDomeSize::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
