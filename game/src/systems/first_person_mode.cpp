#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include "game/world/corridor/detail.hpp"
#include "game/world/corridor/corridor_module.hpp"
#include "game/world/geometry_standards.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;
namespace {

float dist_xz_local(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

Vec3 closest_point_segment_xz(Vec3 p,Vec3 a,Vec3 b,float* out_t=nullptr){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    float t=0.0f;
    if(den>0.00001f) t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    if(out_t) *out_t=t;
    return {a.x+dx*t,a.y+(b.y-a.y)*t,a.z+dz*t};
}

bool inside_rounded_footprint(Vec3 p,Vec3 center,float rotation,float width,float depth,float inset,float round_fraction){
    const Vec3 local=rotate_y(p-center,-rotation);
    const float hx=std::max(0.08f,width*0.5f-inset);
    const float hz=std::max(0.08f,depth*0.5f-inset);
    if(std::abs(local.x)>hx || std::abs(local.z)>hz) return false;
    const float radius=std::clamp(std::min(hx,hz)*round_fraction,0.08f,std::min(hx,hz));
    const float core_x=std::max(0.0f,hx-radius);
    const float core_z=std::max(0.0f,hz-radius);
    const float qx=std::max(0.0f,std::abs(local.x)-core_x);
    const float qz=std::max(0.0f,std::abs(local.z)-core_z);
    return qx*qx+qz*qz<=radius*radius;
}

bool inside_elliptical_footprint(Vec3 p,Vec3 center,float rotation,float width,float depth,float inset){
    const Vec3 local=rotate_y(p-center,-rotation);
    const float hx=std::max(0.08f,width*0.5f-inset);
    const float hz=std::max(0.08f,depth*0.5f-inset);
    const float nx=local.x/hx,nz=local.z/hz;
    return nx*nx+nz*nz<=1.0f;
}

bool near_rotated_footprint(Vec3 p,Vec3 center,float rotation,float width,float depth,float margin){
    const Vec3 local=rotate_y(p-center,-rotation);
    return std::abs(local.x)<=width*0.5f+margin &&
           std::abs(local.z)<=depth*0.5f+margin;
}

bool projected_mesh_contains_body(const std::vector<Vec3>& triangles,Vec3 local,float radius);
bool projected_habitat_contains_body(const HabitatMesh& mesh,Vec3 local,float radius);

bool habitat_uses_elliptical_hull(HabitatKind kind){
    return kind==HabitatKind::OxygenGeneratorDome ||
           kind==HabitatKind::CantineDome ||
           kind==HabitatKind::BioDome;
}

bool inside_habitat_hull(Vec3 p,const PlacedHabitat& h,const HabitatMesh& mesh,float inset){
    const Vec3 local=rotate_y(p-h.position,-h.rotation_degrees);
    // The structural envelope may be split over shell, glazing and foundation
    // meshes (Bio/Cantine domes are the important case). Treat their projected
    // union as one pressure boundary so the complete rendered structure remains
    // walkable while its mesh is still an impassable wall outside door portals.
    if(!mesh.shell.empty() || !mesh.glazing.empty() || !mesh.foundation.empty())
        return projected_habitat_contains_body(mesh,local,std::max(0.0f,inset));
    if(habitat_uses_elliptical_hull(h.kind))
        return inside_elliptical_footprint(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,inset);
    const float round=h.kind==HabitatKind::CrewQuarters?0.95f:0.24f;
    return inside_rounded_footprint(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,inset,round);
}

bool inside_storage_hull(Vec3 p,const PlacedResource& item,const ResourceMesh& mesh,float inset){
    const Vec3 local=rotate_y(p-item.position,-item.rotation_degrees);
    if(!mesh.body.empty()) return projected_mesh_contains_body(mesh.body,local,std::max(0.0f,inset));
    return inside_elliptical_footprint(p,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,inset);
}

bool inside_solid_resource_hull(Vec3 p,const PlacedResource& item,const ResourceMesh& mesh,float inset){
    return inside_rounded_footprint(p,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,inset,0.20f);
}

bool movement_crosses_portal(Vec3 from,Vec3 to,Vec3 center,Vec3 normal,float half_width,float body_radius){
    normal.y=0.0f;
    normal=normalize(normal);
    if(length(normal)<0.001f) return false;
    const Vec3 side{-normal.z,0.0f,normal.x};
    const Vec3 af=from-center;
    const Vec3 bf=to-center;
    const float da=dot(Vec3{af.x,0.0f,af.z},normal);
    const float db=dot(Vec3{bf.x,0.0f,bf.z},normal);
    const float den=da-db;
    float t=0.5f;
    if(std::abs(den)>0.0001f) t=da/den;
    else if(std::min(std::abs(da),std::abs(db))>0.12f) return false;
    if(t<-0.05f || t>1.05f) return false;
    t=std::clamp(t,0.0f,1.0f);
    const Vec3 hit=from+(to-from)*t;
    const Vec3 rel=hit-center;
    const float lateral=std::abs(dot(Vec3{rel.x,0.0f,rel.z},side));
    const float usable=std::max(0.16f,half_width-body_radius*0.55f);
    return lateral<=usable;
}

bool movement_crosses_doorway(Vec3 from,Vec3 to,const CorridorDoor& door,float body_radius){
    return movement_crosses_portal(from,to,door.wall_position,door.normal,
                                   geometry::kDoorClearWidth*0.5f,body_radius);
}

bool movement_crosses_any_door(Vec3 from,Vec3 to,const std::vector<CorridorDoor>& doors,float body_radius){
    for(const auto& d:doors) if(movement_crosses_doorway(from,to,d,body_radius)) return true;
    return false;
}

bool point_in_portal_clearance(Vec3 p,Vec3 center,Vec3 normal,float half_width,float body_radius){
    normal.y=0.0f;
    normal=normalize(normal);
    if(length(normal)<0.001f) return false;
    const Vec3 side{-normal.z,0.0f,normal.x};
    const Vec3 rel=p-center;
    const float longitudinal=std::abs(dot(Vec3{rel.x,0.0f,rel.z},normal));
    const float lateral=std::abs(dot(Vec3{rel.x,0.0f,rel.z},side));
    const float usable=std::max(0.16f,half_width-body_radius*0.45f);
    return longitudinal<=body_radius+0.62f && lateral<=usable;
}

bool point_in_doorway_clearance(Vec3 p,const CorridorDoor& door,float body_radius){
    return point_in_portal_clearance(p,door.wall_position,door.normal,
                                     geometry::kDoorClearWidth*0.5f,body_radius);
}

bool point_in_any_doorway_clearance(Vec3 p,const std::vector<CorridorDoor>& doors,float body_radius){
    for(const auto& door:doors) if(point_in_doorway_clearance(p,door,body_radius)) return true;
    return false;
}

bool movement_uses_any_doorway(Vec3 from,Vec3 to,const std::vector<CorridorDoor>& doors,float body_radius){
    if(movement_crosses_any_door(from,to,doors,body_radius)) return true;
    // Authored hatch anchors are not always numerically coplanar with the
    // projected outer shell (the visible frame/vestibule can sit a few
    // decimetres outside it). If the structure occupancy changes while the
    // body is already inside that narrow doorway clearance, it is still a
    // legitimate hatch crossing rather than a wall escape.
    return point_in_any_doorway_clearance(from,doors,body_radius) ||
           point_in_any_doorway_clearance(to,doors,body_radius);
}

// Sample the whole FPV body footprint against a projected pressure/solid
// volume.  Center-only tests let the avatar penetrate a wall by almost one
// body radius while approaching it from outside; sampling two body rings makes
// the same mesh boundary authoritative from both sides.
template<class Contains>
bool body_overlaps_region(Vec3 p,float radius,const Contains& contains){
    if(contains(p)) return true;
    if(radius<=0.015f) return false;
    constexpr int samples=24;
    for(float scale:{0.50f,1.0f}){
        for(int i=0;i<samples;++i){
            const float a=2.0f*3.14159265358979323846f*(float)i/(float)samples;
            Vec3 q=p;
            q.x+=std::cos(a)*radius*scale;
            q.z+=std::sin(a)*radius*scale;
            if(contains(q)) return true;
        }
    }
    return false;
}


float triangle_area_xz(Vec3 a,Vec3 b,Vec3 c){
    return std::abs((b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x))*0.5f;
}

bool triangle_height_at_xz(Vec3 p,Vec3 a,Vec3 b,Vec3 c,float& y){
    const float den=(b.z-c.z)*(a.x-c.x)+(c.x-b.x)*(a.z-c.z);
    if(std::abs(den)<1.0e-6f) return false;
    const float u=((b.z-c.z)*(p.x-c.x)+(c.x-b.x)*(p.z-c.z))/den;
    const float v=((c.z-a.z)*(p.x-c.x)+(a.x-c.x)*(p.z-c.z))/den;
    const float w=1.0f-u-v;
    constexpr float eps=0.025f;
    if(u<-eps || v<-eps || w<-eps) return false;
    y=a.y*u+b.y*v+c.y*w;
    return true;
}

bool point_in_projected_triangle(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    const float den=(b.z-c.z)*(a.x-c.x)+(c.x-b.x)*(a.z-c.z);
    if(std::abs(den)<1.0e-6f) return false;
    const float u=((b.z-c.z)*(p.x-c.x)+(c.x-b.x)*(p.z-c.z))/den;
    const float v=((c.z-a.z)*(p.x-c.x)+(a.x-c.x)*(p.z-c.z))/den;
    const float w=1.0f-u-v;
    constexpr float eps=0.012f;
    return u>=-eps && v>=-eps && w>=-eps;
}

bool projected_mesh_contains_point(const std::vector<Vec3>& triangles,Vec3 local){
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        if(point_in_projected_triangle(local,triangles[i],triangles[i+1],triangles[i+2])) return true;
    }
    return false;
}

bool projected_mesh_contains_body(const std::vector<Vec3>& triangles,Vec3 local,float radius){
    if(!projected_mesh_contains_point(triangles,local)) return false;
    if(radius<=0.015f) return true;
    constexpr int samples=12;
    for(int i=0;i<samples;++i){
        const float a=2.0f*3.14159265358979323846f*(float)i/(float)samples;
        Vec3 q=local;
        q.x+=std::cos(a)*radius;
        q.z+=std::sin(a)*radius;
        if(!projected_mesh_contains_point(triangles,q)) return false;
    }
    return true;
}

bool projected_habitat_contains_point(const HabitatMesh& mesh,Vec3 local){
    return projected_mesh_contains_point(mesh.shell,local) ||
           projected_mesh_contains_point(mesh.glazing,local) ||
           projected_mesh_contains_point(mesh.foundation,local);
}

bool projected_habitat_contains_body(const HabitatMesh& mesh,Vec3 local,float radius){
    if(!projected_habitat_contains_point(mesh,local)) return false;
    if(radius<=0.015f) return true;
    constexpr int samples=12;
    for(int i=0;i<samples;++i){
        const float a=2.0f*3.14159265358979323846f*(float)i/(float)samples;
        Vec3 q=local;
        q.x+=std::cos(a)*radius;
        q.z+=std::sin(a)*radius;
        if(!projected_habitat_contains_point(mesh,q)) return false;
    }
    return true;
}

std::optional<float> walkable_surface_height_at(const std::vector<Vec3>& triangles,Vec3 local_point,
                                                float min_y,float max_y){
    std::optional<float> best;
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const Vec3 a=triangles[i],b=triangles[i+1],c=triangles[i+2];
        const float spread=std::max({a.y,b.y,c.y})-std::min({a.y,b.y,c.y});
        if(spread>0.055f || triangle_area_xz(a,b,c)<0.015f) continue;
        float y=0.0f;
        if(!triangle_height_at_xz(local_point,a,b,c,y) || y<min_y || y>max_y) continue;
        if(!best || y>*best) best=y;
    }
    return best;
}

float dominant_floor_height(const std::vector<Vec3>& a,const std::vector<Vec3>& b,float fallback){
    struct Level { float y{}; float area{}; };
    std::vector<Level> levels;
    const auto collect=[&](const std::vector<Vec3>& triangles){
        for(std::size_t i=0;i+2<triangles.size();i+=3){
            const Vec3 p0=triangles[i],p1=triangles[i+1],p2=triangles[i+2];
            const float spread=std::max({p0.y,p1.y,p2.y})-std::min({p0.y,p1.y,p2.y});
            if(spread>0.035f) continue;
            const float y=(p0.y+p1.y+p2.y)/3.0f;
            if(y<0.075f || y>1.65f) continue;
            const float area=triangle_area_xz(p0,p1,p2);
            if(area<0.10f) continue;
            const float bucket=std::round(y*50.0f)/50.0f;
            auto it=std::find_if(levels.begin(),levels.end(),[&](const Level& l){ return std::abs(l.y-bucket)<0.005f; });
            if(it==levels.end()) levels.push_back({bucket,area}); else it->area+=area;
        }
    };
    collect(a); collect(b);
    if(levels.empty()) return fallback;
    const float best_area=std::max_element(levels.begin(),levels.end(),[](const Level& x,const Level& y){ return x.area<y.area; })->area;
    // Top faces and bottom faces of a floor slab have the same footprint.  When
    // their areas are effectively tied, use the higher level so FPV stands on
    // the visible deck rather than inside the slab.
    float selected=fallback;
    bool found=false;
    for(const auto& l:levels){
        if(l.area<best_area*0.92f) continue;
        if(!found || l.y>selected){ selected=l.y; found=true; }
    }
    return found?selected:fallback;
}

float habitat_basement_height(const HabitatMesh& mesh){
    // Habitat modules declare the real internal deck explicitly. Rendered
    // furniture/foundation triangles are no longer used to guess POV height.
    return mesh.walkable_floor_y;
}

float storage_basement_height(const ResourceMesh& mesh){
    static const std::vector<Vec3> none;
    return dominant_floor_height(mesh.foundation,none,geometry::kStorageFloorY);
}

float ray_hit_distance(Vec3 ray_origin,Vec3 ray_dir,Vec3 center,float radius,float max_distance){
    const Vec3 to=center-ray_origin;
    const float t=dot(to,ray_dir);
    if(t<0.10f || t>max_distance) return std::numeric_limits<float>::max();
    const Vec3 closest=ray_origin+ray_dir*t;
    if(length(center-closest)>radius) return std::numeric_limits<float>::max();
    return t;
}

std::vector<Vec3> storage_floor_cells(StorageDomeSize size){
    float max_half=0.0f;
    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=static_cast<MaterialResource>(ri);
        max_half=std::max(max_half,resource_package_half_width(material,resource_package_capacity_kg(material)));
    }
    constexpr float pitch=0.52f;
    const float safe_radius=std::max(0.40f,storage_dome_storage_radius_m(size)-max_half-0.04f);
    const int extent=std::max(0,(int)std::floor(safe_radius/pitch));
    std::vector<Vec3> cells;
    cells.reserve((std::size_t)(extent*2+1)*(std::size_t)(extent*2+1));
    for(int iz=-extent;iz<=extent;++iz){
        for(int ix=-extent;ix<=extent;++ix){
            const float x=(float)ix*pitch;
            const float z=(float)iz*pitch;
            if(x*x+z*z>safe_radius*safe_radius) continue;
            // Preserve a real handling aisle from the south docking door.
            if(z<-0.25f && std::abs(x)<0.65f) continue;
            cells.push_back({x,0.0f,z});
        }
    }
    std::sort(cells.begin(),cells.end(),[](const Vec3& a,const Vec3& b){
        // Fill the floor as rows before starting a new vertical layer.
        if(std::abs(a.z-b.z)>0.001f) return a.z>b.z;
        return a.x<b.x;
    });
    if(cells.empty()) cells.push_back({0.0f,0.0f,0.0f});
    return cells;
}

} // namespace

void GameApp::enter_first_person_drop_mode(){
    build_catalog_.clear_selection();
    corridor_link_source_=-1;
    corridor_build_start_.reset();
    utility_build_start_.reset();
    walkway_build_start_.reset();
    walkway_build_target_.reset();
    clear_world_selection();
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    first_person_mode_=FirstPersonMode::DropPending;
    app_.set_relative_mouse_mode(false);
    app_.set_cursor_visible(false);
}

void GameApp::exit_first_person_mode(){
    if(first_person_carried_resource_) drop_first_person_carried_resource();
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    first_person_mode_=FirstPersonMode::Off;
    app_.set_relative_mouse_mode(false);
    app_.set_cursor_visible(true);
}

bool GameApp::habitat_has_complete_corridor_connection(std::size_t habitat_index) const {
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b})
            if(ep->kind==CorridorEndpointKind::Habitat && ep->index==habitat_index) return true;
    }
    return false;
}

bool GameApp::habitat_exterior_access_enabled(std::size_t habitat_index,std::size_t entrance_index) const {
    if(habitat_index>=placed_habitats_.size()) return false;
    const auto& h=placed_habitats_[habitat_index];
    if(!h.construction.complete() || h.kind!=HabitatKind::AirLock) return false;

    // The dedicated Air Lock is the only structure permitted to expose a hatch
    // to the exterior, and only after it actually belongs to a completed
    // pressure/corridor network.  A hatch reserved by any corridor (including
    // one still under construction) is never simultaneously an exterior exit.
    if(!habitat_has_complete_corridor_connection(habitat_index)) return false;
    return !corridor_network_.habitat_entrance_connected(habitat_index,entrance_index);
}

std::vector<CorridorDoor> GameApp::habitat_doors(std::size_t habitat_index) const {
    std::vector<CorridorDoor> doors;
    if(habitat_index>=placed_habitats_.size()) return doors;
    const auto& h=placed_habitats_[habitat_index];
    const auto& mesh=habitat_models_.mesh(h);

    auto add_unique=[&](const CorridorDoor& door){
        for(const auto& d:doors)
            if(dist_xz_local(d.wall_position,door.wall_position)<0.35f) return;
        doors.push_back(door);
    };

    // Ordinary structures expose no standalone exits.  Only a completed
    // pressure connection carves/opens a hatch into their shell.
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Habitat || ep->index!=habitat_index) continue;
            add_unique(ep->door);
        }
    }

    // A connected Air Lock is the sole legal exterior gateway.  Its unused
    // authored hatch(es) remain surface-facing access points; all other habitat
    // types keep unused authored hatches sealed as part of the pressure hull.
    if(h.kind==HabitatKind::AirLock && habitat_has_complete_corridor_connection(habitat_index)){
        for(std::size_t i=0;i<mesh.entrances.size();++i){
            if(!habitat_exterior_access_enabled(habitat_index,i)) continue;
            const auto& e=mesh.entrances[i];
            CorridorDoor d;
            d.wall_position=h.position+rotate_y(e.local_position,h.rotation_degrees);
            d.wall_position.y=h.position.y+geometry::kEntranceSliceY;
            d.position=d.wall_position;
            d.position.y=h.position.y+mesh.walkable_floor_y;
            d.normal=normalize(rotate_y(e.local_normal,h.rotation_degrees));
            d.entrance_index=(int)i;
            add_unique(d);
        }
    }
    return doors;
}

std::vector<CorridorDoor> GameApp::storage_dome_doors(std::size_t resource_index) const {
    std::vector<CorridorDoor> doors;
    if(resource_index>=placed_resources_.size()) return doors;
    const auto& item=placed_resources_[resource_index];
    if(item.kind!=ResourceKind::StorageDome) return doors;

    // Storage domes no longer expose their four candidate service faces as
    // exits.  A face becomes a real doorway only when a completed corridor is
    // pressure-docked to it.
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Resource || ep->index!=resource_index) continue;
            bool duplicate=false;
            for(const auto& d:doors)
                if(dist_xz_local(d.wall_position,ep->door.wall_position)<0.35f){ duplicate=true; break; }
            if(!duplicate) doors.push_back(ep->door);
        }
    }
    return doors;
}

float GameApp::first_person_door_open_fraction(Vec3 door_position) const {
    if(first_person_mode_!=FirstPersonMode::Active) return 0.0f;
    const float d=dist_xz_local(first_person_.feet_position(),door_position);
    return std::clamp((geometry::kDoorTriggerRadius+0.35f-d)/1.25f,0.0f,1.0f);
}

bool GameApp::first_person_spawn_valid(Vec3 p) const {
    const float radius=first_person_.body_radius();
    const float half=(float)playfield_.size()*0.5f-radius;
    if(p.x<-half || p.x>half || p.z<-half || p.z>half) return false;

    // Solid, non-enterable structures are collision volumes from the outside.
    // Test the whole body footprint rather than only the avatar center.
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind==ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(!near_rotated_footprint(p,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius)) continue;
        if(body_overlaps_region(p,radius,[&](Vec3 q){
            return inside_solid_resource_hull(q,item,mesh,0.0f);
        })) return false;
    }

    // Enterable pressure structures may contain the spawn point, but the body
    // must either fit completely inside the hull or straddle it exactly at a
    // physical doorway. The same rule blocks exterior spawn points that clip
    // through the shell.
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        if(!near_rotated_footprint(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+1.0f)) continue;
        const auto doors=habitat_doors(i);
        const bool center_inside=inside_habitat_hull(p,h,mesh,0.0f);
        const bool doorway=point_in_any_doorway_clearance(p,doors,radius);
        if(center_inside){
            // POV placement is exterior-only.  Entering the pressure network
            // must happen physically through a connected Air Lock afterwards.
            return false;
        }else if(body_overlaps_region(p,radius,[&](Vec3 q){
            return inside_habitat_hull(q,h,mesh,0.0f);
        }) && !doorway){
            return false;
        }
    }

    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(!near_rotated_footprint(p,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+0.8f)) continue;
        const auto doors=storage_dome_doors(i);
        const bool center_inside=inside_storage_hull(p,item,mesh,0.0f);
        const bool doorway=point_in_any_doorway_clearance(p,doors,radius);
        if(center_inside){
            return false;
        }else if(body_overlaps_region(p,radius,[&](Vec3 q){
            return inside_storage_hull(q,item,mesh,0.0f);
        }) && !doorway){
            return false;
        }
    }

    // Do not allow POV to be dropped directly into the pressure connection
    // system.  This closes the last bypass around the Air Lock: corridors and
    // docking sleeves can only be reached by walking through the Air Lock
    // structure from the exterior.
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete() || c.path.size()<2) continue;
        for(std::size_t j=0;j+1<c.path.size();++j){
            float t=0.0f;
            const Vec3 q=closest_point_segment_xz(p,c.path[j],c.path[j+1],&t);
            if(t>=0.0f && t<=1.0f && dist_xz_local(p,q)<=corridor_detail::CORRIDOR_HALF_WIDTH+radius)
                return false;
        }
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Habitat && ep->kind!=CorridorEndpointKind::Resource) continue;
            float t=0.0f;
            const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.wall_position,&t);
            if(t>=0.0f && t<=1.0f && dist_xz_local(p,q)<=geometry::kDockSleeveHalfWidth+radius)
                return false;
        }
    }
    return true;
}

float GameApp::first_person_floor_height(Vec3 p) const {
    // A click inside a completed structure belongs to that structure first.
    // The old implementation tested corridors first and used one hard-coded
    // 0.20 m floor for every habitat, which could spawn the character below the
    // visible interior deck whenever a corridor passed close to the building.
    for(const auto& h:placed_habitats_){
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        if(inside_habitat_hull(p,h,mesh,0.02f)){
            // Every crew-accessible habitat owns one authoritative internal
            // deck level. This prevents benches, crop racks or foundation caps
            // from being mistaken for the floor and keeps all size tiers equal
            // vertically.
            return h.position.y+habitat_basement_height(mesh);
        }
    }
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(inside_storage_hull(p,item,mesh,0.02f))
            return item.position.y+storage_basement_height(mesh);
    }

    float best_corridor=std::numeric_limits<float>::max();
    float corridor_floor=0.0f;
    const CorridorConnection* floor_connection=nullptr;
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete() || c.path.size()<2) continue;
        for(std::size_t i=0;i+1<c.path.size();++i){
            float t=0.0f;
            const Vec3 q=closest_point_segment_xz(p,c.path[i],c.path[i+1],&t);
            const float d=dist_xz_local(p,q);
            const float corridor_center_limit=std::max(0.20f,corridor_detail::CORRIDOR_HALF_WIDTH-first_person_.body_radius());
            if(d<best_corridor && d<=corridor_center_limit){
                best_corridor=d;
                corridor_floor=q.y+corridor_detail::corridor_module().access_profile().walkable_floor_y;
                floor_connection=&c;
            }
        }

        // Structure docking sleeves are part of the same walkable connection,
        // not a gap between the corridor and the building.  Include them in the
        // floor resolver so FPV can traverse every completed docking adapter.
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.wall_position);
                const float d=dist_xz_local(p,q);
                const float sleeve_limit=std::max(0.20f,geometry::kDockSleeveHalfWidth-first_person_.body_radius());
                if(d<best_corridor && d<=sleeve_limit){
                    best_corridor=d;
                    corridor_floor=ep->position.y+corridor_detail::corridor_module().access_profile().walkable_floor_y;
                    floor_connection=&c;
                }
                continue;
            }
        }
    }
    if(best_corridor<std::numeric_limits<float>::max()){
        // Generated docking steps (when present) live in corridor detail mesh.
        // Sample their horizontal tread faces so vertical movement follows the
        // physical geometry instead of snapping from one flat floor to another.
        if(floor_connection){
            if(auto y=walkable_surface_height_at(floor_connection->mesh.details,p,
                                                 corridor_floor-0.08f,corridor_floor+1.20f))
                return *y;
        }
        return corridor_floor;
    }

    // Authored exterior stairs remain walkable whenever their entrance is not
    // consumed by a corridor.  The returned height is the actual tread top.
    for(std::size_t hi=0;hi<placed_habitats_.size();++hi){
        const auto& h=placed_habitats_[hi];
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        const Vec3 local=rotate_y(p-h.position,-h.rotation_degrees);
        for(std::size_t ei=0;ei<mesh.entrance_access.size();++ei){
            if(!habitat_exterior_access_enabled(hi,ei)) continue;
            if(auto y=walkable_surface_height_at(mesh.entrance_access[ei].details,local,0.05f,1.65f))
                return h.position.y+*y;
        }
    }
    return 0.0f;
}

bool GameApp::first_person_move_allowed(Vec3 from,Vec3 to) const {
    const float radius=first_person_.body_radius();
    const float half=(float)playfield_.size()*0.5f-radius;
    if(to.x<-half || to.x>half || to.z<-half || to.z>half) return false;

    struct CorridorOccupancy {
        bool inside{false};       // full FPV body fits inside a pressure volume
        bool overlaps{false};     // FPV body touches/intersects the pressure shell
    };

    auto corridor_occupancy=[&](Vec3 p){
        CorridorOccupancy state;
        const auto consider=[&](Vec3 a,Vec3 b,float half_width,bool hard_start=false,bool hard_end=false){
            const float dx=b.x-a.x,dz=b.z-a.z;
            const float den=dx*dx+dz*dz;
            if(den>0.00001f && (hard_start || hard_end)){
                const float segment_t=((p.x-a.x)*dx+(p.z-a.z)*dz)/den;
                // Door/bulkhead planes are real pressure boundaries.  Rounded
                // line-segment caps must not extend the navigable volume through
                // a structure shell or beyond a sealed corridor dead end.
                if(hard_start && segment_t<0.0f) return;
                if(hard_end && segment_t>1.0f) return;
            }
            const Vec3 q=closest_point_segment_xz(p,a,b);
            const float d=dist_xz_local(p,q);
            const float inner=std::max(0.12f,half_width-radius);
            const float outer=half_width+radius;
            if(d<=inner) state.inside=true;
            if(d<=outer) state.overlaps=true;
        };
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete() || c.path.size()<2) continue;
            for(std::size_t j=0;j+1<c.path.size();++j){
                const bool sealed_start=(j==0 && c.endpoint_a.kind==CorridorEndpointKind::OpenDoor && !c.endpoint_a.suppress_open_door);
                const bool sealed_end=(j+2==c.path.size() && c.endpoint_b.kind==CorridorEndpointKind::OpenDoor && !c.endpoint_b.suppress_open_door);
                consider(c.path[j],c.path[j+1],corridor_detail::CORRIDOR_HALF_WIDTH,sealed_start,sealed_end);
            }

            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                    // The complete docking pressure volume extends all the way
                    // to the structure hull.  For generated entrances
                    // door.position is the outer vestibule door while
                    // wall_position is the real pressure-shell contact plane.
                    // Using only door.position created a non-walkable gap.
                    consider(ep->position,ep->door.wall_position,geometry::kDockSleeveHalfWidth,true,true);
                    continue;
                }
            }
        }
        return state;
    };

    auto point_in_corridor_portal=[&](Vec3 p){
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete()) continue;
            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                    if(point_in_doorway_clearance(p,ep->door,radius)) return true;
                    continue;
                }
            }
        }
        return false;
    };

    auto crosses_corridor_portal=[&](Vec3 a,Vec3 b){
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete()) continue;
            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                    if(movement_crosses_doorway(a,b,ep->door,radius)) return true;
                    continue;
                }
            }
        }
        return false;
    };

    // Validate short samples along every requested movement.  This prevents a
    // fast diagonal/Shift step from tunnelling through a thin shell and makes
    // the exact same door rule apply when entering and when leaving.
    const float move_distance=dist_xz_local(from,to);
    const int samples=std::clamp((int)std::ceil(move_distance/0.06f),1,12);
    Vec3 previous=from;
    CorridorOccupancy previous_corridor=corridor_occupancy(previous);

    for(int sample=1;sample<=samples;++sample){
        const float t=(float)sample/(float)samples;
        const Vec3 current=from+(to-from)*t;
        if(current.x<-half || current.x>half || current.z<-half || current.z>half) return false;

        // Non-enterable machinery/resources are solid from every direction.
        // Body-ring sampling prevents the camera/body from clipping them while
        // the avatar center is still technically outside the footprint.
        for(const auto& item:placed_resources_){
            if(!item.construction.complete() || item.kind==ResourceKind::StorageDome) continue;
            const auto& mesh=resource_models_.mesh(item);
            if(!near_rotated_footprint(current,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius)) continue;
            if(body_overlaps_region(current,radius,[&](Vec3 q){
                return inside_solid_resource_hull(q,item,mesh,0.0f);
            })) return false;
        }

        // Habitat pressure shells are authoritative in both directions.  Inside
        // the structure the whole body must remain within the projected shell;
        // outside, no body sample may penetrate it.  Only actual door apertures
        // are exempt while the avatar is physically straddling the hull.
        for(std::size_t i=0;i<placed_habitats_.size();++i){
            const auto& h=placed_habitats_[i];
            if(!h.construction.complete()) continue;
            const auto& mesh=habitat_models_.mesh(h);
            const bool near_previous=near_rotated_footprint(previous,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+1.0f);
            const bool near_current=near_rotated_footprint(current,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+1.0f);
            if(!near_previous && !near_current) continue;
            const auto doors=habitat_doors(i);
            const bool was_inside=inside_habitat_hull(previous,h,mesh,0.0f);
            const bool is_inside=inside_habitat_hull(current,h,mesh,0.0f);
            const bool doorway=point_in_any_doorway_clearance(current,doors,radius);

            if(is_inside){
                if(!inside_habitat_hull(current,h,mesh,radius) && !doorway) return false;
            }else if(body_overlaps_region(current,radius,[&](Vec3 q){
                return inside_habitat_hull(q,h,mesh,0.0f);
            }) && !doorway){
                return false;
            }

            if(was_inside!=is_inside && !movement_uses_any_doorway(previous,current,doors,radius))
                return false;
        }

        // Storage Domes use the same bidirectional pressure-shell rule and the
        // four explicitly modelled docking faces as their only legal crossings.
        for(std::size_t i=0;i<placed_resources_.size();++i){
            const auto& item=placed_resources_[i];
            if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
            const auto& mesh=resource_models_.mesh(item);
            const bool near_previous=near_rotated_footprint(previous,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+0.8f);
            const bool near_current=near_rotated_footprint(current,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+0.8f);
            if(!near_previous && !near_current) continue;
            const auto doors=storage_dome_doors(i);
            const bool was_inside=inside_storage_hull(previous,item,mesh,0.0f);
            const bool is_inside=inside_storage_hull(current,item,mesh,0.0f);
            const bool doorway=point_in_any_doorway_clearance(current,doors,radius);

            if(is_inside){
                if(!inside_storage_hull(current,item,mesh,radius) && !doorway) return false;
            }else if(body_overlaps_region(current,radius,[&](Vec3 q){
                return inside_storage_hull(q,item,mesh,0.0f);
            }) && !doorway){
                return false;
            }

            if(was_inside!=is_inside && !movement_uses_any_doorway(previous,current,doors,radius))
                return false;
        }

        // Corridors, docking vestibules and air-locks form one connected
        // pressure-volume union.  That union deliberately permits
        // corridor-to-corridor movement at real merged junctions.  Its shell is
        // still solid from the exterior, and transitions to exterior/structures
        // are only legal at endpoint doors.
        const CorridorOccupancy current_corridor=corridor_occupancy(current);
        if(!current_corridor.inside && current_corridor.overlaps && !point_in_corridor_portal(current))
            return false;
        if(previous_corridor.inside!=current_corridor.inside &&
           !crosses_corridor_portal(previous,current))
            return false;

        previous=current;
        previous_corridor=current_corridor;
    }
    return true;
}

std::string GameApp::first_person_environment_label() const {
    if(first_person_mode_!=FirstPersonMode::Active) return "";
    const Vec3 p=first_person_.feet_position();

    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        if(inside_habitat_hull(p,h,mesh,0.02f)) return "INTERIOR / PRESSURIZED";
    }
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(inside_storage_hull(p,item,mesh,0.02f)) return "STORAGE INTERIOR / PRESSURIZED";
    }
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete() || c.path.size()<2) continue;
        for(std::size_t j=0;j+1<c.path.size();++j){
            const Vec3 q=closest_point_segment_xz(p,c.path[j],c.path[j+1]);
            if(dist_xz_local(p,q)<=std::max(0.20f,corridor_detail::CORRIDOR_HALF_WIDTH-first_person_.body_radius()))
                return "CORRIDOR / PRESSURIZED";
        }
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.wall_position);
                if(dist_xz_local(p,q)<=std::max(0.20f,geometry::kDockSleeveHalfWidth-first_person_.body_radius()))
                    return "DOCKING / PRESSURIZED";
                continue;
            }
        }
    }
    return "EXTERIOR / VACUUM";
}

void GameApp::stored_resource_box_pose(const PlacedResource& structure,const ResourceMesh& mesh,std::size_t box_index,Vec3& base,float& half_width,float& height) const {
    if(box_index>=structure.stored_boxes.size()){
        base=structure.position;
        half_width=0.20f;
        height=0.28f;
        return;
    }
    const auto& box=structure.stored_boxes[box_index];
    half_width=resource_package_half_width(box.resource,box.amount);
    height=resource_package_height(box.resource,box.amount);

    // Pallet packing is layer-major: every floor cell is occupied once before
    // a second layer starts. Within a column the exact previous box heights are
    // accumulated, so boxes physically sit on one another instead of sharing a
    // fixed/overlapping Y offset.
    const auto cells=storage_floor_cells(structure.storage_size);
    const std::size_t cell_count=std::max<std::size_t>(1,cells.size());
    const std::size_t cell=box_index%cell_count;
    const float storage_floor=storage_basement_height(mesh);
    float y=storage_floor;
    constexpr float stack_gap=0.02f;
    for(std::size_t previous=cell;previous<box_index;previous+=cell_count){
        const auto& below=structure.stored_boxes[previous];
        y+=resource_package_height(below.resource,below.amount)+stack_gap;
    }

    // The grid itself is constrained to a conservative cylinder inside the
    // curved dome hull. Capacity is derived from the same usable volume, so a
    // valid inventory never needs to place a package outside the pressure skin.
    const float max_top=storage_floor+storage_dome_storage_height_m(structure.storage_size);
    if(y+height>max_top) y=std::max(storage_floor,max_top-height);
    Vec3 local=cells[cell];
    local.y=y;
    base=structure.position+rotate_y(local,structure.rotation_degrees);
}

void GameApp::update_first_person_resource_hover(){
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    if(first_person_mode_!=FirstPersonMode::Active || first_person_carried_resource_) return;

    const Vec3 origin=first_person_.eye_position();
    const Vec3 dir=first_person_.look_direction();
    constexpr float max_reach=2.65f;
    float best=std::numeric_limits<float>::max();

    for(std::size_t i=0;i<ground_resource_piles_.size();++i){
        const auto& pile=ground_resource_piles_[i];
        const float half=resource_package_half_width(pile.resource,pile.amount);
        const float height=resource_package_height(pile.resource,pile.amount);
        const Vec3 center=pile.position+Vec3{0.0f,height*0.5f+0.025f,0.0f};
        const float t=ray_hit_distance(origin,dir,center,std::max(half*1.35f,height*0.58f),max_reach);
        if(t<best){ best=t; first_person_hover_ground_resource_=(int)i; }
    }

    const Vec3 feet=first_person_.feet_position();
    for(std::size_t si=0;si<placed_resources_.size();++si){
        const auto& structure=placed_resources_[si];
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete() || structure.stored_boxes.empty()) continue;
        const auto& mesh=resource_models_.mesh(structure);
        // Stored packages can only be handled while the character is actually
        // inside that dome; this prevents grabbing through the pressure hull.
        if(!inside_storage_hull(feet,structure,mesh,0.05f)) continue;
        for(std::size_t bi=0;bi<structure.stored_boxes.size();++bi){
            Vec3 base{}; float half=0.0f,height=0.0f;
            stored_resource_box_pose(structure,mesh,bi,base,half,height);
            const Vec3 center=base+Vec3{0.0f,height*0.5f+0.025f,0.0f};
            const float t=ray_hit_distance(origin,dir,center,std::max(half*1.35f,height*0.58f),max_reach);
            if(t<best){
                best=t;
                first_person_hover_ground_resource_=-1;
                first_person_hover_storage_resource_=(int)si;
                first_person_hover_storage_box_=(int)bi;
            }
        }
    }
}

bool GameApp::store_first_person_carried_resource(){
    if(!first_person_carried_resource_) return false;
    const Vec3 feet=first_person_.feet_position();
    for(auto& structure:placed_resources_){
        if(structure.kind!=ResourceKind::StorageDome || !structure.construction.complete()) continue;
        const auto& mesh=resource_models_.mesh(structure);
        if(!inside_storage_hull(feet,structure,mesh,first_person_.body_radius()*0.25f)) continue;
        sync_stored_resource_totals(structure);
        const std::int64_t free=storage_dome_capacity_kg(structure.storage_size)-structure.stored_resources.total_amount();
        if(free<first_person_carried_resource_->amount) continue;
        structure.stored_boxes.push_back({first_person_carried_resource_->resource,
                                          first_person_carried_resource_->amount,
                                          first_person_carried_resource_->condition});
        sync_stored_resource_totals(structure);
        ResourceBundle add{};
        add.amount(first_person_carried_resource_->resource)=first_person_carried_resource_->amount;
        resources_.add(add);
        first_person_carried_resource_.reset();
        return true;
    }
    return false;
}

void GameApp::drop_first_person_carried_resource(){
    if(!first_person_carried_resource_) return;
    if(store_first_person_carried_resource()) return;

    GroundResourcePile pile=*first_person_carried_resource_;
    Vec3 forward=first_person_.look_direction();
    forward.y=0.0f;
    if(length(forward)>0.001f) forward=normalize(forward); else forward={0.0f,0.0f,-1.0f};
    const Vec3 feet=first_person_.feet_position();
    Vec3 target=feet+forward*0.78f;
    if(!first_person_move_allowed(feet,target)) target=feet;
    target.y=first_person_floor_height(target);
    pile.position=target;
    pile.radius=resource_package_half_width(pile.resource,pile.amount);
    ground_resource_piles_.push_back(pile);
    first_person_carried_resource_.reset();
}

void GameApp::first_person_grab_or_drop_resource(){
    if(first_person_mode_!=FirstPersonMode::Active) return;
    if(first_person_carried_resource_){
        drop_first_person_carried_resource();
        update_first_person_resource_hover();
        return;
    }

    if(first_person_hover_ground_resource_>=0 && (std::size_t)first_person_hover_ground_resource_<ground_resource_piles_.size()){
        const std::size_t index=(std::size_t)first_person_hover_ground_resource_;
        auto& source=ground_resource_piles_[index];
        const std::int64_t take=std::min(source.amount,resource_package_capacity_kg(source.resource));
        GroundResourcePile carried=source;
        carried.amount=take;
        carried.radius=resource_package_half_width(carried.resource,carried.amount);
        first_person_carried_resource_=carried;
        source.amount-=take;
        if(source.amount<=0) ground_resource_piles_.erase(ground_resource_piles_.begin()+(std::ptrdiff_t)index);
        else source.radius=resource_package_half_width(source.resource,source.amount);
        update_first_person_resource_hover();
        return;
    }

    if(first_person_hover_storage_resource_>=0 && first_person_hover_storage_box_>=0 &&
       (std::size_t)first_person_hover_storage_resource_<placed_resources_.size()){
        auto& structure=placed_resources_[(std::size_t)first_person_hover_storage_resource_];
        if((std::size_t)first_person_hover_storage_box_>=structure.stored_boxes.size()) return;
        auto& source=structure.stored_boxes[(std::size_t)first_person_hover_storage_box_];
        const std::int64_t take=std::min(source.amount,resource_package_capacity_kg(source.resource));
        GroundResourcePile carried;
        carried.resource=source.resource;
        carried.amount=take;
        carried.condition=source.condition;
        carried.radius=resource_package_half_width(carried.resource,carried.amount);
        first_person_carried_resource_=carried;
        source.amount-=take;
        ResourceBundle subtract{};
        subtract.amount(carried.resource)=-take;
        resources_.add(subtract);
        if(source.amount<=0) structure.stored_boxes.erase(structure.stored_boxes.begin()+(std::ptrdiff_t)first_person_hover_storage_box_);
        sync_stored_resource_totals(structure);
        update_first_person_resource_hover();
    }
}

void GameApp::update_first_person(float dt,const InputState& input){
    if(first_person_mode_!=FirstPersonMode::Active) return;
    first_person_.update_look(input);
    const Vec3 displacement=first_person_.desired_displacement(input,dt);
    Vec3 current=first_person_.feet_position();
    if(length(displacement)>0.0001f){
        // Try the actual movement vector first. This is essential for rotated
        // doors: splitting a diagonal doorway crossing into world-X/world-Z
        // moves can make each artificial half-step hit the frame even though
        // the requested path goes cleanly through the opening. Preserve the
        // old axis attempts only as wall-sliding fallbacks.
        const Vec3 direct=current+displacement;
        if(first_person_move_allowed(current,direct)){
            current=direct;
        }else{
            Vec3 x_try{current.x+displacement.x,current.y,current.z};
            if(first_person_move_allowed(current,x_try)) current=x_try;
            Vec3 z_try{current.x,current.y,current.z+displacement.z};
            if(first_person_move_allowed(current,z_try)) current=z_try;
        }
    }
    const float target_floor=first_person_floor_height(current);
    const float max_step=std::max(0.08f,dt*2.8f);
    current.y += std::clamp(target_floor-current.y,-max_step,max_step);
    first_person_.set_feet_position(current);
    update_first_person_resource_hover();
    if(input.left_pressed) first_person_grab_or_drop_resource();
}

} // namespace game
