#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/gate_area.hpp"
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

    if(kind==ResourceKind::RobotWorkshop){
        PlacedResource item;item.kind=kind;item.size=ResourceSizeTier::Standard;
        item.position=position;item.rotation_degrees=rotation_degrees;
        const auto apron=workshop::approach(item);
        for(float x:{-apron.half_x,apron.half_x}) for(float z:{-apron.half_z,apron.half_z}){
            const Vec2 q{apron.center.x+apron.axis_x.x*x+apron.axis_z.x*z,apron.center.y+apron.axis_x.y*x+apron.axis_z.y*z};
            if(std::abs(q.x)>build_limit || std::abs(q.y)>build_limit) return false;
        }
        for(const auto& h:placed_habitats_){const auto& m=habitat_models_.mesh(h);
            if(overlaps(apron,footprint(h.position,h.rotation_degrees,
                                         geometry::structure_ground_extent(m.footprint_x),
                                         geometry::structure_ground_extent(m.footprint_z)),clearance)) return false;}
        for(const auto& r:placed_resources_){const auto& m=resource_models_.mesh(r);
            if(overlaps(apron,footprint(r.position,r.rotation_degrees,
                                         geometry::structure_ground_extent(m.footprint_x),
                                         geometry::structure_ground_extent(m.footprint_z)),clearance)) return false;
            if(r.kind==ResourceKind::StorageDome) for(int face:storage_loading::faces(r))
                if(face>=0 && overlaps(apron,storage_loading::apron(r,face),clearance)) return false;}
        for(const auto& pile:ground_resource_piles_)
            if(overlaps(apron,footprint(pile.position,0,2*pile.radius,2*pile.radius),1.12f)) return false;
        for(const auto& c:corridor_network_.connections()) for(std::size_t j=1;j<c.path.size();++j)
            if(workshop::approach_hits(item,c.path[j-1],c.path[j],geometry::kCorridorHalfWidth+geometry::kRouteClearance)) return false;
        for(const auto& w:walkway_network_.connections()) for(std::size_t j=1;j<w.path.size();++j)
            if(workshop::approach_hits(item,w.path[j-1],w.path[j],geometry::kWalkwayHalfWidth+0.40f)) return false;
        if(!workshop_accessible(item)) return false;
    }

    PlacedResource candidate_item;candidate_item.kind=kind;candidate_item.size=size;candidate_item.storage_size=storage_size;candidate_item.position=position;candidate_item.rotation_degrees=rotation_degrees;
    if(!gate_layout_clear(nullptr,&candidate_item))return false;
    if(!placement_preserves_robot_escape(position,rotation_degrees,candidate.footprint_x,candidate.footprint_z)) return false;
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
    const float top_px=STATUS_BAR_H*ui_scale_;
    const float bottom_px=(ui_height(r)-BUILD_BAR_H)*ui_scale_;

    // Optional Storage Dome gate extension placement. This is deliberately
    // independent of the normal build catalog: the dome already exists as a
    // blueprint or completed structure, and each accepted gate becomes its own
    // StorageGate construction task.
    if(storage_gate_placement_active_){
        if(storage_gate_placement_owner_>=placed_resources_.size() ||
           placed_resources_[storage_gate_placement_owner_].kind!=ResourceKind::StorageDome){
            storage_gate_placement_active_=false;
            storage_gate_placement_owner_=kNoRobotIndex;
            return;
        }
        if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;
        Vec3 hit{};
        if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;

        auto owner=placed_resources_[storage_gate_placement_owner_];
        Vec3 local=rotate_y(hit-owner.position,-owner.rotation_degrees);local.y=0.0f;
        if(length(local)<0.001f) local={1,0,1};
        local=normalize(local);
        // Continuous cursor placement, with a small magnetic region around the
        // predefined cargo sectors. Crew-door sectors remain reserved.
        float angle=std::atan2(local.x,local.z)*180.0f/3.14159265358979323846f;
        for(int face:storage_loading::kCargoFaces){const auto n=storage_loading::local_normal(face);if(dot(local,n)>0.9962f){angle=std::atan2(n.x,n.z)*180.0f/3.14159265358979323846f;break;}}
        int selected_face=8;for(;selected_face<=10;++selected_face){bool used=false;for(int f:storage_loading::faces(owner))if(f==selected_face)used=true;if(!used)break;}
        if(selected_face>10)return;
        storage_loading::set_angle(owner,selected_face,angle);

        const auto gate_plan=construction_costs::structure(ResourceKind::StorageGate,
            resource_models_.mesh(ResourceKind::StorageGate,owner.storage_size));
        const bool valid=storage_loading::gate_count(placed_resources_[storage_gate_placement_owner_])<3 && storage_gate_face_clear(owner,selected_face,storage_gate_placement_owner_) &&
                         can_start_construction(gate_plan);
        const Vec3 n=storage_loading::normal(owner,selected_face);
        const Vec3 gate_position=(storage_loading::wall(owner,selected_face)+storage_loading::mouth(owner,selected_face))*0.5f;
        const float gate_rotation=std::atan2(-n.z,n.x)*180.0f/3.14159265358979323846f;
        const auto& gate_mesh=resource_models_.mesh(ResourceKind::StorageGate,owner.storage_size);
        render_resource_mesh(r,gate_mesh,gate_position,gate_rotation,true,valid);
        render_storage_gate_shutters(r,owner,selected_face,0.0f,0.0f,true);
        gate_area::draw(r,{storage_loading::service(owner,selected_face),n,1.0f,0.75f},valid);

        const auto bay=storage_loading::apron(owner,selected_face);
        auto corner=[&](float x,float z){
            return Vec3{bay.center.x+bay.axis_x.x*x+bay.axis_z.x*z,0.055f,
                        bay.center.y+bay.axis_x.y*x+bay.axis_z.y*z};
        };
        const Vec3 a=corner(-bay.half_x,-bay.half_z),b=corner(bay.half_x,-bay.half_z);
        const Vec3 c=corner(bay.half_x,bay.half_z),d=corner(-bay.half_x,bay.half_z);
        r.draw_lines_3d({a,b,b,c,c,d,d,a},valid?Color{0.22f,1.0f,0.48f,1}:Color{1.0f,0.22f,0.12f,1},2.8f);

        if(valid && input_.left_pressed){
            if(add_storage_gate_at(storage_gate_placement_owner_,angle)){
                storage_gate_placement_active_=false;
                storage_gate_placement_owner_=kNoRobotIndex;
            }
        }
        return;
    }

    const BuildObjectDefinition* selected=build_catalog_.selected();
    if(!selected || selected->kind!=BuildObjectKind::Resource) return;
    if((float)input_.mouse_y<top_px || (float)input_.mouse_y>=bottom_px) return;

    Vec3 hit{};
    if(!camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit)) return;

    const StorageDomeSize storage_size=selected->resource_kind==ResourceKind::StorageDome?placement_storage_size_:StorageDomeSize::Small;
    const ResourceSizeTier resource_size=(selected->resource_kind==ResourceKind::StorageDome || selected->resource_kind==ResourceKind::RobotWorkshop)?ResourceSizeTier::Standard:placement_resource_size_;
    float rotation=placement_rotation_degrees_;
    Vec3 snapped=selected->resource_kind==ResourceKind::StorageDome?snap_storage_dome(hit,storage_size,rotation):Vec3{std::round(hit.x),0.0f,std::round(hit.z)};
    PlacedResource candidate;
    candidate.kind=selected->resource_kind;
    candidate.size=resource_size;
    candidate.storage_size=storage_size;
    candidate.rotation_degrees=placement_rotation_degrees_;
    auto attachment=preview_structure_attachment(hit,nullptr,&candidate);
    if(attachment){snapped=attachment->position;rotation=candidate.rotation_degrees;}
    const auto& mesh=selected->resource_kind==ResourceKind::StorageDome?
        resource_models_.mesh(selected->resource_kind,storage_size):resource_models_.mesh(selected->resource_kind,resource_size);
    const auto plan=construction_costs::structure(selected->resource_kind,mesh);
    const bool valid=(attachment || resource_placement_valid(selected->resource_kind,resource_size,storage_size,snapped,rotation)) &&
        can_start_construction(plan);
    render_resource_mesh(r,mesh,snapped,rotation,true,valid);

    if(selected->resource_kind==ResourceKind::RobotWorkshop){
        PlacedResource item;item.kind=ResourceKind::RobotWorkshop;item.position=snapped;item.rotation_degrees=rotation;
        render_workshop_gate(r,item,true,valid);
    }

    if(valid && attachment)render_structure_attachment(r,*attachment);

    if(valid && input_.left_pressed){
        PlacedResource placed;
        placed.kind=selected->resource_kind;
        std::uint32_t serial=1;
        for(const auto& existing:placed_resources_){
            if(existing.kind!=placed.kind) continue;
            const bool same_size=placed.kind==ResourceKind::StorageDome?
                existing.storage_size==storage_size:existing.size==resource_size;
            if(same_size) serial=std::max(serial,existing.instance_serial+1);
        }
        placed.instance_serial=serial;
        placed.position=snapped;
        placed.rotation_degrees=rotation;
        placed.storage_size=storage_size;
        placed.size=resource_size;
        placed.resource_plan=plan;
        finalize_placement(placed);
        placed_resources_.push_back(std::move(placed));
        if(attachment)commit_structure_attachment(std::move(*attachment));
        build_catalog_.clear_selection();
        placement_rotation_degrees_=0.0f;
        placement_resource_size_=ResourceSizeTier::Small;
        placement_storage_size_=StorageDomeSize::Small;
        corridor_build_start_.reset();
    }
}

} // namespace game
