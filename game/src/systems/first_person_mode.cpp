#include "game/world/robot_workshop.hpp"
#include "game/game_app.hpp"
#include "game/world/interior_unit_mesh.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/world/corridor/detail.hpp"
#include "game/world/corridor/corridor_module.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/storage_loading_gate.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace game {
using namespace game::runtime;
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
    if(mesh.dome_radii.x>0){
        const float rx=std::max(.01f,mesh.dome_radii.x-inset),rz=std::max(.01f,mesh.dome_radii.z-inset);
        if(mesh.dome_radii.y==0)return std::abs(local.x)<=rx&&std::abs(local.z)<=rz;
        return local.x*local.x/(rx*rx)+local.z*local.z/(rz*rz)<=1.f;
    }
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
    const float usable=std::max(0.0f,half_width-body_radius);
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
    const float usable=std::max(0.0f,half_width-body_radius);
    // Rendering carves generated pressure openings 0.90 m into the source
    // structure mesh. Keep collision clearance at least that deep plus the
    // astronaut radius, otherwise uncarved source furniture/details can form
    // an invisible lip immediately behind an otherwise open hatch.
    return longitudinal<=body_radius+0.92f && lateral<=usable;
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

float point_segment_distance_xz_local(Vec3 p,Vec3 a,Vec3 b){
    return dist_xz_local(p,closest_point_segment_xz(p,a,b));
}

float projected_triangle_distance_xz(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    if(point_in_projected_triangle(p,a,b,c)) return 0.0f;
    return std::min({point_segment_distance_xz_local(p,a,b),
                     point_segment_distance_xz_local(p,b,c),
                     point_segment_distance_xz_local(p,c,a)});
}

// Limit a convex triangle/polygon to the part above or below a horizontal plane.
std::size_t clip_body_plane(const std::array<Vec3,6>& in,std::size_t count,
                            std::array<Vec3,6>& out,float y,bool above){
    if(count==0)return 0;
    std::size_t written=0;
    Vec3 previous=in[count-1];
    for(std::size_t i=0;i<count;++i){
        const Vec3 current=in[i];
        const bool keep_previous=above?previous.y>=y:previous.y<=y;
        const bool keep_current=above?current.y>=y:current.y<=y;
        if(keep_previous!=keep_current)
            out[written++]=previous+(current-previous)*((y-previous.y)/(current.y-previous.y));
        if(keep_current)out[written++]=current;
        previous=current;
    }
    return written;
}

// Interior furniture/equipment is rendered as ordinary triangle meshes, but the
// old POV collision code only treated the outer pressure shell as solid. Test
// the astronaut's body against geometry at its actual height. Clipping avoids
// projecting the below-floor part of a sloped hull rib into a clear aisle.
// Edge distance handles vertical faces that project to a line.
bool mesh_blocks_first_person_body(const std::vector<Vec3>& triangles,Vec3 local_feet,
                                   float radius,float body_height,float floor_y){
    const float body_bottom=std::max(local_feet.y+0.045f,floor_y+0.045f);
    const float body_top=local_feet.y+body_height-0.035f;
    if(body_top<=body_bottom) return false;
    const float reach=radius+0.012f;
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const Vec3 a=triangles[i],b=triangles[i+1],c=triangles[i+2];
        const float tri_min_y=std::min({a.y,b.y,c.y});
        const float tri_max_y=std::max({a.y,b.y,c.y});
        if(tri_max_y<=body_bottom || tri_min_y>=body_top) continue;
        if(std::min({a.x,b.x,c.x})>local_feet.x+reach || std::max({a.x,b.x,c.x})<local_feet.x-reach ||
           std::min({a.z,b.z,c.z})>local_feet.z+reach || std::max({a.z,b.z,c.z})<local_feet.z-reach)continue;
        if(projected_triangle_distance_xz(local_feet,a,b,c)>reach)continue;
        if(tri_min_y>=body_bottom && tri_max_y<=body_top)return true;
        std::array<Vec3,6> polygon{a,b,c},clipped{};
        const auto above_floor=clip_body_plane(polygon,3,clipped,body_bottom,true);
        const auto count=clip_body_plane(clipped,above_floor,polygon,body_top,false);
        for(std::size_t j=1;j+1<count;++j){
            if(projected_triangle_distance_xz(local_feet,polygon[0],polygon[j],polygon[j+1])<=reach)return true;
        }
    }
    return false;
}

bool habitat_interior_blocks_first_person(const HabitatMesh& mesh,Vec3 local_feet,float radius,float body_height){
    return mesh_blocks_first_person_body(mesh.overhead_services,local_feet,radius,body_height,mesh.walkable_floor_y) ||
           mesh_blocks_first_person_body(mesh.interior,local_feet,radius,body_height,mesh.walkable_floor_y) ||
           mesh_blocks_first_person_body(mesh.details,local_feet,radius,body_height,mesh.walkable_floor_y) ||
           mesh_blocks_first_person_body(mesh.service_metal,local_feet,radius,body_height,mesh.walkable_floor_y) ||
           mesh_blocks_first_person_body(mesh.fabric,local_feet,radius,body_height,mesh.walkable_floor_y);
}

bool resource_interior_blocks_first_person(const ResourceMesh& mesh,Vec3 local_feet,float radius,float body_height){
    return mesh_blocks_first_person_body(mesh.details,local_feet,radius,body_height,mesh.walkable_floor_y);
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

float habitat_basement_height(const HabitatMesh& mesh){
    // Habitat modules declare the real internal deck explicitly. Rendered
    // furniture/foundation triangles are no longer used to guess POV height.
    return mesh.walkable_floor_y;
}

float storage_basement_height(const ResourceMesh& mesh){
    return mesh.walkable_floor_y;
}

float ray_hit_distance(Vec3 ray_origin,Vec3 ray_dir,Vec3 center,float radius,float max_distance){
    const Vec3 to=center-ray_origin;
    const float t=dot(to,ray_dir);
    if(t<0.10f || t>max_distance) return std::numeric_limits<float>::max();
    const Vec3 closest=ray_origin+ray_dir*t;
    if(length(center-closest)>radius) return std::numeric_limits<float>::max();
    return t;
}

std::vector<Vec3> make_storage_floor_cells(StorageDomeSize size){
    float max_half=0.0f;
    for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
        const auto material=static_cast<MaterialResource>(ri);
        max_half=std::max(max_half,resource_package_half_width(material,resource_package_capacity_kg(material)));
    }
    constexpr float pitch=0.52f;
    const float safe_radius=storage_dome_storage_radius_m(size)-max_half-0.04f;
    // A square crate has its widest projection at 45 degrees.
    const float aisle_half=geometry::kDoorClearWidth*0.5f+std::sqrt(2.f)*max_half;
    const int extent=std::max(0,(int)std::floor(safe_radius/pitch));
    std::vector<Vec3> cells;
    cells.reserve((std::size_t)(extent*2+1)*(std::size_t)(extent*2+1));
    for(int iz=-extent;iz<=extent;++iz){
        for(int ix=-extent;ix<=extent;++ix){
            const float x=(float)ix*pitch;
            const float z=(float)iz*pitch;
            if(x*x+z*z>safe_radius*safe_radius) continue;
            if(std::abs(x)<aisle_half || std::abs(z)<aisle_half) continue;
            cells.push_back({x,0.0f,z});
        }
    }
    std::sort(cells.begin(),cells.end(),[](const Vec3& a,const Vec3& b){
        // Fill the floor as rows before starting a new vertical layer.
        if(std::abs(a.z-b.z)>0.001f) return a.z>b.z;
        return a.x<b.x;
    });
    return cells;
}

const std::vector<Vec3>& storage_floor_cells(StorageDomeSize size){
    // Packing grids depend only on the dome tier. Sorting them for every box
    // made render and navigation cost grow with inventory size every frame.
    static const std::array<std::vector<Vec3>,4> grids{{
        make_storage_floor_cells((StorageDomeSize)0),make_storage_floor_cells((StorageDomeSize)1),
        make_storage_floor_cells((StorageDomeSize)2),make_storage_floor_cells((StorageDomeSize)3)}};
    return grids.at((std::size_t)size);
}

} // namespace

bool GameApp::habitat_has_complete_corridor_connection(std::size_t habitat_index) const {
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b})
            if(ep->kind==CorridorEndpointKind::Habitat && ep->index==habitat_index) return true;
    }
    return false;
}

bool GameApp::habitat_exterior_access_enabled(std::size_t habitat_index,std::size_t entrance_index) const {
    (void)habitat_index; (void)entrance_index;
    // There are no authored structure entrances/stairs after v0.43.17. Any
    // exterior access geometry is derived from a real connection-created door.
    return false;
}

bool GameApp::airlock_access_enabled(std::size_t index,const CorridorDoor& door) const {
    if(index>=placed_habitats_.size() || placed_habitats_[index].kind!=HabitatKind::AirLock || airlock::internal(door))return false;
    for(const auto& c:corridor_network_.connections())if(c.construction.complete())
        for(const auto* end:{&c.endpoint_a,&c.endpoint_b})
            if(end->kind==CorridorEndpointKind::Habitat && end->index==index &&
               length(end->door.wall_position-door.wall_position)<.35f)return false;
    return true;
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

    if(!h.construction.complete())return doors;
    if(h.kind==HabitatKind::AirLock){
        const int inside=airlock_inside_sign(habitat_index);
        for(int sign:{-1,1}){
            auto d=airlock::end_door(h,mesh,sign);
            if(sign!=inside)d.entrance_index=-4;
            add_unique(d);
        }
        for(const auto& d:airlock::partitions(h,mesh))add_unique(d);
    }else{
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete())continue;
            for(const auto* ep:{&c.endpoint_a,&c.endpoint_b})
                if(ep->kind==CorridorEndpointKind::Habitat && ep->index==habitat_index)add_unique(ep->door);
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
    for(const auto& state:pressure_doors_)
        if(length(state.door.wall_position-door_position)<0.01f) return state.open_fraction;
    return 0.0f;
}

bool GameApp::first_person_spawn_valid(Vec3 p) const {
    if(resource_packages_block(p))return false;
    if(lunar_lander_segment_blocked(lander_,p,p,0.35f))return false;
    const float radius=first_person_.body_radius();
    const float half=(float)playfield_.size()*0.5f-radius;
    if(p.x<-half || p.x>half || p.z<-half || p.z>half) return false;

    // Solid, non-enterable structures are collision volumes from the outside.
    // Test the whole body footprint rather than only the avatar center.
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind==ResourceKind::StorageDome) continue;
        if(item.kind==ResourceKind::RobotWorkshop){if(!workshop::clear(item,p,p,radius)) return false;continue;}
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
        if(storage_loading::body_hits_buffer(item,p,radius)) return false;
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
    const float package_top=resource_package_support(p);if(package_top>0)return package_top;
    // A jump onto installed equipment must land on its solid top instead of
    // dropping through it into a volume from which no horizontal step is legal.
    float unit_top=-std::numeric_limits<float>::max();
    for(std::size_t i=0;i<placed_habitats_.size();++i){const auto& host=placed_habitats_[i];if(!host.construction.complete())continue;
        const auto local=rotate_y(p-host.position,-host.rotation_degrees);
        for(const auto& unit:host.units){
            if(interior_unit_reserved(i,unit))continue;
            const auto size=interior_unit_profile(unit).size;const auto q=rotate_y(local-unit.position,-unit.rotation_degrees);
            const float top=host.position.y+unit.position.y+size.y;
            const float dx=std::max(std::abs(q.x)-size.x*.5f,0.f),dz=std::max(std::abs(q.z)-size.z*.5f,0.f);
            if(p.y>=top-.025f&&dx*dx+dz*dz<first_person_.body_radius()*first_person_.body_radius())unit_top=std::max(unit_top,top);
        }
    }
    if(unit_top>-std::numeric_limits<float>::max())return unit_top;
    for(const auto& item:placed_resources_) if(item.kind==ResourceKind::RobotWorkshop && item.construction.complete()){
        const float y=workshop::floor(item,p);if(y>0) return y;
    }
    // A connection-created entrance owns its complete docking ramp all the way
    // from the corridor deck to the newly cut hull threshold. Resolve that
    // ramp before projected habitat foundations, otherwise a low apron can make
    // the floor jump to the interior deck before the avatar reaches the hatch.
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Corridor&&ep->surface_matched_junction){
                float t=0;const auto q=closest_point_segment_xz(p,ep->position,ep->host_position,&t);
                if(dist_xz_local(p,q)<=geometry::kCorridorDeckHalfWidth-first_person_.body_radius())return q.y+geometry::kCorridorDeckY;
            }
            if(ep->kind!=CorridorEndpointKind::Habitat && ep->kind!=CorridorEndpointKind::Resource) continue;
            Vec3 outer=ep->position;outer.y+=geometry::kCorridorDeckY;
            Vec3 wall=ep->door.wall_position;wall.y=ep->door.position.y;
            const float sleeve_limit=std::max(0.20f,geometry::kDockSleeveHalfWidth-first_person_.body_radius());
            const auto sample=[&](Vec3 a,Vec3 b)->std::optional<float>{
                const float dx=b.x-a.x,dz=b.z-a.z,den=dx*dx+dz*dz;
                if(den<.00001f)return {};
                const float t=((p.x-a.x)*dx+(p.z-a.z)*dz)/den;
                if(t<0.f||t>1.f)return {};
                const Vec3 q=a+(b-a)*t;
                if(dist_xz_local(p,q)>sleeve_limit)return {};
                return corridor_detail::docking_floor_y(a,b,t);
            };
            if(auto y=sample(outer,ep->door.position))return *y;
            if(auto y=sample(ep->door.position,wall))return *y;
        }
    }
    // The completed airlock owns its exterior hatch and stair before a
    // corridor exists. The connected end determines inside versus outside.
    for(std::size_t hi=0;hi<placed_habitats_.size();++hi){
        const auto& h=placed_habitats_[hi];
        if(h.kind!=HabitatKind::AirLock || !h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        for(const auto& door:habitat_doors(hi)){
            if(!airlock_access_enabled(hi,door)) continue;
            Vec3 n=door.normal;n.y=0;n=normalize(n);if(length(n)<0.001f) continue;
            const Vec3 side{-n.z,0,n.x};const Vec3 delta=p-door.wall_position;
            const float along=dot(Vec3{delta.x,0,delta.z},n);
            const float lateral=std::abs(dot(Vec3{delta.x,0,delta.z},side));
            const float deck=h.position.y+habitat_basement_height(mesh);
            const int steps=std::max(1,(int)std::ceil(deck/0.16f));
            const float run=0.32f*(float)steps;
            if(along>=0.0f && along<=run && lateral<=0.98f){
                const int tread=std::min(steps-1,int(along/.32f));
                return deck-deck*float(tread)/float(steps);
            }
        }
    }

    // A click inside a completed structure belongs to that structure first.
    // The old implementation tested corridors first and used one hard-coded
    // 0.20 m floor for every habitat, which could spawn the character below the
    // visible interior deck whenever a corridor passed close to the building.
    for(const auto& h:placed_habitats_){
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        if(near_rotated_footprint(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,.5f) &&
           inside_habitat_hull(p,h,mesh,0.02f)){
            // Every crew-accessible habitat owns one authoritative internal
            // deck level. This prevents benches, crop racks or foundation caps
            // from being mistaken for the floor and keeps all size tiers equal
            // vertically.
            return h.position.y+habitat_basement_height(mesh);
        }
        // The authored air-lock hatch sits slightly beyond the pressure hull.
        // Its visible deck landing bridges that gap to the exterior stairs;
        // keep feet at deck height on both sides of the hatch plane.
        if(h.kind==HabitatKind::AirLock){
            const auto local=rotate_y(p-h.position,-h.rotation_degrees);
            for(const auto& entrance:mesh.entrances){
                const auto delta=local-entrance.local_position;
                const auto normal=entrance.local_normal;const Vec3 side{-normal.z,0,normal.x};
                const float along=dot(delta,normal);
                if(along>=-0.9f && along<=0.001f && std::abs(dot(delta,side))<=geometry::kDoorClearWidth*0.5f)
                    return h.position.y+habitat_basement_height(mesh);
            }
        }
    }
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(near_rotated_footprint(p,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,.5f) &&
           inside_storage_hull(p,item,mesh,0.02f))
            return item.position.y+storage_basement_height(mesh);
    }

    float best_corridor=std::numeric_limits<float>::max();
    float corridor_floor=0.0f;

    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete() || c.path.size()<2) continue;
        for(std::size_t i=0;i+1<c.path.size();++i){
            float t=0.0f;
            const Vec3 q=closest_point_segment_xz(p,c.path[i],c.path[i+1],&t);
            const float d=dist_xz_local(p,q);
            const float corridor_center_limit=std::max(0.20f,corridor_detail::CORRIDOR_HALF_WIDTH-first_person_.body_radius());
            const auto tangent=normalize(Vec3{c.path[i+1].x-c.path[i].x,0,c.path[i+1].z-c.path[i].z});
            const bool past_start=i==0&&dot(p-c.path[i],tangent)<-.001f;
            const bool past_end=i+2==c.path.size()&&dot(p-c.path[i+1],tangent)>.001f;
            if(!past_start&&!past_end&&d<best_corridor && d<=corridor_center_limit){
                best_corridor=d;
                corridor_floor=q.y+corridor_detail::corridor_module().access_profile().walkable_floor_y;

            }
        }

        // Structure docking sleeves are part of the same walkable connection,
        // not a gap between the corridor and the building.  Include them in the
        // floor resolver so FPV can traverse every completed docking adapter.
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                const float sleeve_limit=std::max(0.20f,geometry::kDockSleeveHalfWidth-first_person_.body_radius());
                Vec3 outer=ep->position;outer.y+=geometry::kCorridorDeckY;
                Vec3 wall=ep->door.wall_position;wall.y=ep->door.position.y;
                const auto sample=[&](Vec3 a,Vec3 b){
                    if(dist_xz_local(a,b)<.001f)return;
                    float t=0;const auto q=closest_point_segment_xz(p,a,b,&t);
                    const float d=dist_xz_local(p,q);
                    if(d<best_corridor&&d<=sleeve_limit){best_corridor=d;corridor_floor=corridor_detail::docking_floor_y(a,b,t);}
                };
                sample(outer,ep->door.position);sample(ep->door.position,wall);
                continue;
            }
        }
    }
    if(best_corridor<std::numeric_limits<float>::max()){
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

float GameApp::first_person_ceiling_height(Vec3 p) const {
    float ceiling=std::numeric_limits<float>::max();
    nearby_resource_packages(p,first_person_.body_radius(),[&](const packages::Box& box){
        if(std::abs(p.x-box.base.x)<box.half+first_person_.body_radius()&&std::abs(p.z-box.base.z)<box.half+first_person_.body_radius()&&box.bottom()>=p.y+first_person_.body_height()-.02f)ceiling=std::min(ceiling,box.bottom());
        return true;
    });
    const float head=first_person_floor_height(p)+first_person_.body_height();
    const auto scan=[&](const std::vector<Vec3>& triangles,Vec3 origin,float rotation){
        const Vec3 local=rotate_y(p-origin,-rotation);
        // Cover the body footprint under curved/sloping pressure roofs.
        for(int sample=0;sample<9;++sample){
            Vec3 probe=local;
            if(sample){const float angle=2*3.14159265358979323846f*(float)(sample-1)/8;probe.x+=std::cos(angle)*first_person_.body_radius();probe.z+=std::sin(angle)*first_person_.body_radius();}
            for(std::size_t i=0;i+2<triangles.size();i+=3){
                float y=0;
                if(triangle_height_at_xz(probe,triangles[i],triangles[i+1],triangles[i+2],y) && origin.y+y>=head-0.025f)
                    ceiling=std::min(ceiling,origin.y+y);
            }
        }
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i){const auto& h=placed_habitats_[i];if(!h.construction.complete())continue;
        const auto& m=habitat_models_.mesh(h);
        if(!near_rotated_footprint(p,h.position,h.rotation_degrees,m.footprint_x,m.footprint_z,first_person_.body_radius()+0.5f))continue;
        if(m.dome_radii.x>0){
            const auto local=rotate_y(p-h.position,-h.rotation_degrees);
            for(int sample=0;sample<9;++sample){auto q=local;if(sample){const float a=6.2831853f*(sample-1)/8;q.x+=std::cos(a)*first_person_.body_radius();q.z+=std::sin(a)*first_person_.body_radius();}
                const float rho=q.x*q.x/(m.dome_radii.x*m.dome_radii.x)+q.z*q.z/(m.dome_radii.z*m.dome_radii.z);
                if(m.dome_radii.y==0){if(std::abs(q.x)<=m.dome_radii.x&&std::abs(q.z)<=m.dome_radii.z)ceiling=std::min(ceiling,h.position.y+m.dome_spring_y-.025f);}
                else if(rho<=1)ceiling=std::min(ceiling,h.position.y+m.dome_spring_y+m.dome_radii.y*std::sqrt(1-rho)-.025f);
            }
        }else {scan(m.shell,h.position,h.rotation_degrees);scan(m.glazing,h.position,h.rotation_degrees);}
        for(const auto& door:habitat_doors(i))if(point_in_doorway_clearance(p,door,first_person_.body_radius()))ceiling=std::min(ceiling,door.position.y+geometry::kDoorClearHeight);
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){const auto& item=placed_resources_[i];if(!item.construction.complete())continue;
        if(item.kind!=ResourceKind::StorageDome && item.kind!=ResourceKind::RobotWorkshop)continue;
        const auto& m=resource_models_.mesh(item);
        if(!near_rotated_footprint(p,item.position,item.rotation_degrees,m.footprint_x,m.footprint_z,first_person_.body_radius()+0.5f))continue;
        scan(m.body,item.position,item.rotation_degrees);scan(m.glazing,item.position,item.rotation_degrees);
        if(item.kind==ResourceKind::StorageDome)for(const auto& door:storage_dome_doors(i))if(point_in_doorway_clearance(p,door,first_person_.body_radius()))ceiling=std::min(ceiling,door.position.y+geometry::kDoorClearHeight);
    }
    for(const auto& c:corridor_network_.connections())if(c.construction.complete()){
        bool near=false;for(std::size_t j=1;j<c.path.size();++j)if(point_segment_distance_xz(p,c.path[j-1],c.path[j])<=geometry::kCorridorHalfWidth+first_person_.body_radius()+0.5f){near=true;break;}
        if(near)scan(c.mesh.shell,{},0);
        for(const auto* ep:{&c.endpoint_a,&c.endpoint_b})if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
            if(point_segment_distance_xz(p,ep->position,ep->door.wall_position)<=geometry::kDockSleeveHalfWidth+first_person_.body_radius())
                ceiling=std::min(ceiling,std::min(ep->position.y+geometry::kCorridorDeckY,ep->door.position.y)+geometry::kDockSleeveHeight);
        }
    }
    return ceiling;
}

const SupplyState* GameApp::astronaut_air_at(Vec3 p,bool& airlock) const {
    airlock=false;
    for(const auto& h:placed_habitats_){
        if(!h.construction.complete())continue;
        const auto& mesh=habitat_models_.mesh(h);
        if(near_rotated_footprint(p,h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z,.5f) &&
           inside_habitat_hull(p,h,mesh,0.f)){
            airlock=h.kind==HabitatKind::AirLock;
            return &h.supply;
        }
    }
    for(const auto& r:placed_resources_){
        if(r.kind!=ResourceKind::StorageDome || !r.construction.complete())continue;
        const auto& mesh=resource_models_.mesh(r);
        if(near_rotated_footprint(p,r.position,r.rotation_degrees,mesh.footprint_x,mesh.footprint_z,.5f) &&
           inside_storage_hull(p,r,mesh,0.f))return &r.supply;
    }
    const auto in_run=[&](Vec3 a,Vec3 b,float half){
        const auto d=b-a;
        const float length_sq=d.x*d.x+d.z*d.z;
        if(length_sq<1e-5f)return false;
        const float t=((p.x-a.x)*d.x+(p.z-a.z)*d.z)/length_sq;
        return t>=0.f && t<=1.f && dist_xz_local(p,a+d*t)<=half;
    };
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete())continue;
        for(std::size_t i=1;i<c.path.size();++i)
            if(in_run(c.path[i-1],c.path[i],geometry::kCorridorHalfWidth))return &c.supply;
        for(const auto* e:{&c.endpoint_a,&c.endpoint_b})
            if(e->kind==CorridorEndpointKind::Habitat || e->kind==CorridorEndpointKind::Resource){
                // A collar on a curved wall starts at its sampled pressure
                // profile, which recedes behind the central hatch plane.
                // Include that throat so off-center crew do not encounter a
                // fictitious strip of vacuum inside a sealed connection.
                float recess=0;
                for(const auto q:e->door.wall_profile)recess=std::min(recess,dot(q-e->door.wall_position,e->door.normal));
                const auto throat=e->door.wall_position+e->door.normal*(recess-.025f);
                if(in_run(e->position,throat,geometry::kDockSleeveHalfWidth))return &c.supply;
            }
    }
    return nullptr;
}

bool GameApp::first_person_move_allowed(Vec3 from,Vec3 to,CrewTraversal traversal) const {
    if(lunar_lander_segment_blocked(lander_,from,to,0.35f))return false;
    const float radius=first_person_.body_radius();
    const float half=(float)playfield_.size()*0.5f-radius;
    if(to.x<-half || to.x>half || to.z<-half || to.z>half) return false;

    const bool planning=traversal==CrewTraversal::Plan;
    const bool crew=traversal!=CrewTraversal::Player;
    const auto opening=[&](const CorridorDoor& d){return planning?1.f:first_person_door_open_fraction(d.wall_position);};

    // Closed/partly open leaves are physical seals. Use the same opening and
    // travel as rendering, so an avatar cannot step through a closed outline.
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        if(!placed_habitats_[i].construction.complete()) continue;
        for(const auto& d:habitat_doors(i)){
            float fraction=opening(d);
            if(airlock::internal(d)?airlock::bulkhead_blocks(d,fraction,from,to,radius,first_person_.body_height(),habitat_models_.mesh(placed_habitats_[i]).footprint_x/10.0f):
               pressure_door::blocks(d,fraction,from,to,radius))return false;
        }
    }
    for(std::size_t i=0;i<placed_resources_.size();++i){
        if(!placed_resources_[i].construction.complete()) continue;
        for(const auto& d:storage_dome_doors(i))
            if(pressure_door::blocks(d,opening(d),from,to,radius)) return false;
    }

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

    // A generated hatch may sit behind a low foundation/apron whose projected
    // footprint extends farther out than the pressure wall. The completed
    // docking sleeve is itself a certified walkable pressure volume, so treat
    // its full width as entrance clearance instead of requiring the avatar to
    // reach the narrow wall-plane portal first.
    auto point_in_habitat_dock=[&](Vec3 p,std::size_t habitat_index){
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete()) continue;
            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind!=CorridorEndpointKind::Habitat || ep->index!=habitat_index) continue;
                float t=0.0f;const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.wall_position,&t);
                if(t>=0.0f && t<=1.0f && dist_xz_local(p,q)<=std::max(0.05f,geometry::kDockSleeveHalfWidth-radius)) return true;
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
    bool previous_airlock=false;
    bool previous_inside=crew && astronaut_air_at(previous,previous_airlock);

    for(int sample=1;sample<=samples;++sample){
        const float t=(float)sample/(float)samples;
        const Vec3 current=from+(to-from)*t;
        if(resource_packages_block(current))return false;
        if(!crew&&controlled_astronaut_>=0){
            for(std::size_t i=0;i<test_astronauts_.size();++i)if(int(i)!=controlled_astronaut_&&astronaut_present(i)){
                const auto p=test_astronauts_[i].position;
                if(std::abs(current.y-p.y)<first_person_.body_height()&&dist_xz_local(current,p)<kIndoorAstronautSeparation-.001f)return false;
            }
        }
        if(current.x<-half || current.x>half || current.z<-half || current.z>half) return false;
        if(crew){
            bool current_airlock=false;
            const bool current_inside=astronaut_air_at(current,current_airlock)!=nullptr;
            // Docking sleeves and ordinary pressure doors may join two inside
            // volumes. Only an airlock may join a pressure volume to vacuum.
            if(previous_inside!=current_inside && !previous_airlock && !current_airlock)return false;
            previous_inside=current_inside;
            previous_airlock=current_airlock;
        }

        // Non-enterable machinery/resources are solid from every direction.
        // Body-ring sampling prevents the camera/body from clipping them while
        // the avatar center is still technically outside the footprint.
        for(const auto& item:placed_resources_){
            if(!item.construction.complete() || item.kind==ResourceKind::StorageDome) continue;
            if(item.kind==ResourceKind::RobotWorkshop){
                if(!workshop::clear(item,previous,current,radius)) return false;
                const auto& mesh=resource_models_.mesh(item);
                if(near_rotated_footprint(current,item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z,radius+0.25f)){
                    const Vec3 local=rotate_y(current-item.position,-item.rotation_degrees);
                    if(resource_interior_blocks_first_person(mesh,local,radius,first_person_.body_height())) return false;
                }
                continue;
            }
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
            const bool doorway=point_in_any_doorway_clearance(current,doors,radius) || point_in_habitat_dock(current,i);

            if(is_inside){
                if(!inside_habitat_hull(current,h,mesh,radius) && !doorway) return false;
                if(!doorway){
                    const Vec3 local=rotate_y(current-h.position,-h.rotation_degrees);
                    // Mesh surfaces alone miss a saved body wholly inside a
                    // closed tank/cabinet. Installed units own solid local
                    // volumes, shared by player movement and safe spawn search.
                    for(const auto& unit:h.units){
                        if(interior_unit_reserved(i,unit))continue;
                        const auto size=interior_unit_profile(unit).size;
                        const auto p=rotate_y(local-unit.position,-unit.rotation_degrees);
                        if(p.y+first_person_.body_height()<=.025f||p.y>=size.y-.025f)continue;
                        const float dx=std::max(std::abs(p.x)-size.x*.5f,0.f),dz=std::max(std::abs(p.z)-size.z*.5f,0.f);
                        if(dx*dx+dz*dz<(radius+.012f)*(radius+.012f))return false;
                    }
                    if(habitat_interior_blocks_first_person(habitat_runtime_mesh(i),local,radius,first_person_.body_height())) return false;
                }
            }else if(body_overlaps_region(current,radius,[&](Vec3 q){
                return inside_habitat_hull(q,h,mesh,0.0f);
            }) && !doorway){
                return false;
            }

            if(was_inside!=is_inside && !movement_uses_any_doorway(previous,current,doors,radius) &&
               !point_in_habitat_dock(previous,i) && !point_in_habitat_dock(current,i))
                return false;
        }

        // Storage Domes use the same bidirectional pressure-shell rule and the
        // four explicitly modelled docking faces as their only legal crossings.
        for(std::size_t i=0;i<placed_resources_.size();++i){
            const auto& item=placed_resources_[i];
            if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
            if(storage_loading::body_hits_buffer(item,current,radius)) return false;
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
                if(!doorway){
                    const Vec3 local=rotate_y(current-item.position,-item.rotation_degrees);
                    if(resource_interior_blocks_first_person(resource_runtime_mesh(i),local,radius,first_person_.body_height())) return false;

                }
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
        if(inside_habitat_hull(p,h,mesh,0.02f)){
            if(h.kind==HabitatKind::AirLock){const auto local=rotate_y(p-h.position,-h.rotation_degrees);return airlock::chamber(local.x,airlock::partition_x(mesh),airlock_inside_sign(i));}
            return "INTERIOR / PRESSURIZED";
        }
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
    const bool raw_ore=box_index>=structure.stored_boxes.size();
    if(raw_ore){
        half_width=.20f;
        height=.31f;
    }else{
        const auto& box=structure.stored_boxes[box_index];
        half_width=resource_package_half_width(box.resource,box.amount);
        height=resource_package_height(box.resource,box.amount);
    }

    // Pallet packing is layer-major: every floor cell is occupied once before
    // a second layer starts. Within a column the exact previous box heights are
    // accumulated, so boxes physically sit on one another instead of sharing a
    // fixed/overlapping Y offset.
    const auto& cells=storage_floor_cells(structure.storage_size);
    const std::size_t cell_count=cells.size();
    const std::size_t cell=box_index%cell_count;
    const float storage_floor=storage_basement_height(mesh);
    float y=storage_floor;
    constexpr float stack_gap=0.02f;
    for(std::size_t previous=cell;previous<box_index;previous+=cell_count){
        if(previous>=structure.stored_boxes.size())y+=.31f+stack_gap;
        else{
            const auto& below=structure.stored_boxes[previous];
            y+=resource_package_height(below.resource,below.amount)+stack_gap;
        }
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
        const float height=resource_package_height(pile.resource,pile.top_package_amount());
        const Vec3 center=pile.position+Vec3{0.0f,pile.height()-height*.5f+0.025f,0.0f};
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
        const std::int64_t free=storage_dome_capacity_kg(structure.storage_size)-storage_dome_used_kg(structure);
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
    const auto capacity=resource_package_capacity_kg(pile.resource);
    for(auto& stack:ground_resource_piles_){
        if(stack.resource!=pile.resource||std::abs(stack.condition-pile.condition)>.01f||
           std::abs(stack.position.y-target.y)>.05f||distance_xz(stack.position,target)>.65f||
           stack.amount%capacity!=0||stack.amount>=capacity*kGroundStackLayers) continue;
        stack.amount+=pile.amount;
        if(stack.kind==GroundPileKind::Loose) stack.kind=GroundPileKind::Stack;
        stack.radius=resource_package_half_width(stack.resource,stack.amount);
        first_person_carried_resource_.reset();
        return;
    }
    pile.kind=GroundPileKind::Loose;
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
        const std::int64_t take=source.top_package_amount();
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

bool GameApp::first_person_pointer_over_hud(const InputState& input) const {
    if(!first_person_cursor_free())return false;
    const float x=float(input.mouse_x)/ui_scale_,y=float(input.mouse_y)/ui_scale_;
    float help_height=88.f;
    const auto feet=first_person_.feet_position();
    for(const auto& h:placed_habitats_){
        if(h.kind!=HabitatKind::AirLock || !h.construction.complete())continue;
        const auto local=rotate_y(feet-h.position,-h.rotation_degrees);const auto& mesh=habitat_models_.mesh(h);
        if(std::abs(local.x)<=mesh.footprint_x*.5f+.2f && std::abs(local.z)<=mesh.footprint_z*.5f){help_height+=44.f;break;}
    }
    if(!manual_door_label().empty())help_height+=36.f;
    if(first_person_carried_resource_)help_height+=52.f;
    const float help_width=controlled_astronaut_>=0?std::min(528.f,astronaut_status_rect().x-2*layout::PLAYFIELD_MARGIN):528.f;
    const Rect help{layout::PLAYFIELD_MARGIN,layout::STATUS_BAR_H+layout::PLAYFIELD_MARGIN,help_width,help_height};
    return x<0 || x>=float(viewport_width_)/ui_scale_ || y<layout::STATUS_BAR_H ||
        y>=float(viewport_height_)/ui_scale_-layout::BUILD_BAR_H || help.contains(x,y) ||
        astronaut_status_hover(input)||mouse_over_playfield_overlay(input);
}
bool GameApp::pick_build_ground(Vec3& hit) const {
    if(first_person_mode_!=FirstPersonMode::Active)
        return camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,viewport_width_,viewport_height_,playfield_.size(),hit);
    if(first_person_pointer_over_hud(input_))return false;
    const auto eye=first_person_.eye_position(),forward=first_person_.look_direction();
    const auto right=normalize(cross(forward,Vec3{0,1,0})),up=normalize(cross(right,forward));
    const float tangent=std::tan(radians(first_person_.fov())*.5f);
    const float x=first_person_cursor_free()?2.f*input_.mouse_x/std::max(1,viewport_width_)-1.f:0.f;
    const float y=first_person_cursor_free()?1.f-2.f*input_.mouse_y/std::max(1,viewport_height_):0.f;
    const auto ray=normalize(forward+right*(x*float(viewport_width_)/std::max(1,viewport_height_)*tangent)+up*(y*tangent));
    if(ray.y>=-.0001f)return false;
    const float distance=-eye.y/ray.y;if(distance<=0||distance>90.f)return false;
    hit=eye+ray*distance;hit.y=0;
    const float half=playfield_.size()*.5f;return std::abs(hit.x)<=half&&std::abs(hit.z)<=half;
}
void GameApp::cancel_placement_tool(){
    interior_unit_tool_.reset();interior_unit_host_=kNoRobotIndex;
    build_catalog_.clear_selection();storage_gate_placement_active_=false;storage_gate_placement_owner_=kNoRobotIndex;
    corridor_link_source_=-1;corridor_build_start_.reset();utility_build_start_.reset();walkway_build_start_.reset();walkway_build_target_.reset();
    placement_tip_title_.clear();placement_tip_detail_.clear();placement_tip_cost_={};
}

float GameApp::interior_light_level(std::size_t index,bool resource,bool require_power) const {
    if(status_.sun_pose().above_horizon)return 0;
    const auto delivered=[&](const SupplyState& supply){return require_power?float(std::clamp(supply.lighting_fraction>=0?supply.lighting_fraction:supply.power_fraction,0.0,1.0)):1.f;};
    const auto occupied=[&](const auto& inside){
        if(first_person_mode_==FirstPersonMode::Active&&inside(first_person_.feet_position()))return true;
        for(const auto& actor:test_astronauts_)if(astronaut_present(std::size_t(&actor-test_astronauts_.data()))&&actor.health>0&&inside(actor.position))return true;
        for(const auto& robot:builder_robots_)if(inside(robot.position))return true;
        return false;
    };
    if(resource){
        if(index>=placed_resources_.size())return 0;
        const auto& h=placed_resources_[index];
        if((h.kind!=ResourceKind::StorageDome&&h.kind!=ResourceKind::RobotWorkshop)||!h.construction.complete()||!h.power_enabled)return 0;
        const auto& mesh=resource_models_.mesh(h);
        const auto inside=[&](Vec3 p){const auto local=rotate_y(p-h.position,-h.rotation_degrees);
            if(local.y<mesh.walkable_floor_y-.30f||local.y>mesh.height||std::abs(local.x)>mesh.footprint_x*.5f||std::abs(local.z)>mesh.footprint_z*.5f)return false;
            if(h.kind==ResourceKind::StorageDome)return inside_storage_hull(p,h,mesh,.05f);
            return std::abs(local.x)<mesh.footprint_x*.46f&&std::abs(local.z)<mesh.footprint_z*.46f;
        };
        return (occupied(inside)?1.f:.18f)*delivered(h.supply);
    }
    if(index>=placed_habitats_.size())return 0;
    const auto& h=placed_habitats_[index];
    if(!h.construction.complete()||!h.power_enabled)return 0;
    const auto& mesh=habitat_models_.mesh(h);
    const auto inside=[&](Vec3 p){const auto local=rotate_y(p-h.position,-h.rotation_degrees);
        return std::abs(local.x)<mesh.footprint_x*.5f&&std::abs(local.z)<mesh.footprint_z*.5f&&local.y>=mesh.walkable_floor_y-.30f&&local.y<mesh.height&&inside_habitat_hull(p,h,mesh,.05f);};
    return (occupied(inside)?1.f:.18f)*delivered(h.supply);
}

bool GameApp::interior_unit_position_clear(std::size_t index,const InteriorUnit& unit,std::string& error) const {
    if(index>=placed_habitats_.size()||int(unit.kind)>=int(InteriorUnitKind::Count)||int(unit.size)>=int(InteriorUnitSize::Count)||(unit.kind!=InteriorUnitKind::OxygenGenerator&&unit.size!=InteriorUnitSize::Medium)){error="INVALID HOST OR UNIT";return false;}
    const auto& host=placed_habitats_[index];const auto& source=habitat_models_.mesh(host);
    if(!host.construction.complete()||!interior_unit_allowed(host.kind,unit.kind)){error="UNIT IS NOT COMPATIBLE WITH THIS ROOM";return false;}
    if(host.units.size()>=64){error="ROOM UNIT LIMIT REACHED";return false;}
    if(!std::isfinite(unit.position.x)||!std::isfinite(unit.position.y)||!std::isfinite(unit.position.z)||!std::isfinite(unit.rotation_degrees)||std::abs(unit.position.y-source.walkable_floor_y)>.01f){error="PLACE ON THE INTERNAL DECK";return false;}
    const auto [low,high]=interior_unit_bounds(unit);
    if(host.kind==HabitatKind::AirLock){
        if(low.z<geometry::kDoorClearWidth*.5f+.20f&&high.z>-geometry::kDoorClearWidth*.5f-.20f){error="CENTER WALKING AISLE RESERVED";return false;}
    }else{
        const auto relative=rotate_y(unit.position*-1.f,-unit.rotation_degrees);const auto dimensions=interior_unit_profile(unit).size;
        const float dx=std::max(0.f,std::abs(relative.x)-dimensions.x*.5f-.18f),dz=std::max(0.f,std::abs(relative.z)-dimensions.z*.5f-.18f);
        float core=host.kind==HabitatKind::OxygenGeneratorDome&&(host.size==HabitatSizeTier::Small||host.size==HabitatSizeTier::VerySmall)?1.f:1.25f;
        if(host.kind==HabitatKind::ProcessingFacility&&unit.kind==InteriorUnitKind::OreRefinery){
            // The compact processing-facility tiers rely on perimeter work bays.
            // Keep the center passage clear, but do not reserve an oversized core
            // that prevents the dedicated refinery from fitting at all.
            core=(host.size==HabitatSizeTier::Small||host.size==HabitatSizeTier::VerySmall)?0.72f:0.95f;
        }
        if(dx*dx+dz*dz<core*core){error="CENTRAL CIRCULATION CORE RESERVED";return false;}
    }
    if(interior_unit_reserved(index,unit)){error="GATE APPROACH RESERVED";return false;}
    for(const auto& other:host.units){if(interior_unit_reserved(index,other))continue;
        if(interior_units_overlap(unit,other)){error="OVERLAPS AN INSTALLED UNIT";return false;}
    }
    const auto& mesh=habitat_runtime_mesh(index);const auto size=interior_unit_profile(unit).size;
    const float half_x=size.x*.5f+.18f,half_z=size.z*.5f+.18f;
    const int nx=std::max(1,int(std::ceil(2*half_x/.25f))),nz=std::max(1,int(std::ceil(2*half_z/.25f)));
    for(int ix=0;ix<=nx;++ix)for(int iz=0;iz<=nz;++iz){
        const auto point=unit.position+rotate_y({std::lerp(-half_x,half_x,float(ix)/nx),0,std::lerp(-half_z,half_z,float(iz)/nz)},unit.rotation_degrees);
        const float x=point.x,z=point.z;
        const auto world=host.position+rotate_y({x,source.walkable_floor_y,z},host.rotation_degrees);
        // Equipment already has a clearance margin; do not add avatar head radius.
        float roof=std::numeric_limits<float>::max();
        if(source.dome_radii.x>0){const auto r=source.dome_radii;const float rho=x*x/(r.x*r.x)+z*z/(r.z*r.z);roof=source.dome_spring_y+r.y*std::sqrt(std::max(0.f,1-rho))-.025f;}
        else for(const auto* layer:{&source.shell,&source.glazing})for(std::size_t t=0;t+2<layer->size();t+=3){
            float y=0;if(triangle_height_at_xz({x,0,z},(*layer)[t],(*layer)[t+1],(*layer)[t+2],y)&&y>source.walkable_floor_y+.10f)roof=std::min(roof,y);
        }
        if(!inside_habitat_hull(world,host,source,.15f)||roof<source.walkable_floor_y+size.y+.10f){error="UNIT MUST FIT INSIDE THE PRESSURE HULL";return false;}
        if(habitat_interior_blocks_first_person(mesh,{x,source.walkable_floor_y,z},.015f,size.y)){error="OVERLAPS FURNITURE OR EQUIPMENT";return false;}
    }
    if(first_person_mode_==FirstPersonMode::Active){const auto p=rotate_y(first_person_.feet_position()-host.position,-host.rotation_degrees);
        if(p.x>low.x-first_person_.body_radius()&&p.x<high.x+first_person_.body_radius()&&p.z>low.z-first_person_.body_radius()&&p.z<high.z+first_person_.body_radius()){error="MOVE AWAY FROM THE UNIT POSITION";return false;}
    }
    error.clear();return true;
}

void GameApp::update_first_person(float dt,const InputState& input){
    corridor_network_.refresh_pressure_shells();
    if(first_person_mode_!=FirstPersonMode::Active) return;
    if(controlled_astronaut_>=0&&test_astronauts_[controlled_astronaut_].health<=0){
        test_astronauts_[controlled_astronaut_].activity=AstronautActivity::Knockout;
        exit_first_person_mode();
        activity_log_.record(ui_time_seconds_,LogEvent::Incident,"Astronaut deceased / FPV ended / Select living crew or NEW EXPLORER");
        return;
    }
    if(!first_person_cursor_free() || (!first_person_alt_cursor_&&input.right_down&&!first_person_pointer_over_hud(input)))first_person_.update_look(input);
    if(controlled_astronaut_>=0){auto& actor=test_astronauts_[controlled_astronaut_];
        if(actor.activity==AstronautActivity::Sleeping&&(input.key_w_down||input.key_a_down||input.key_s_down||input.key_d_down))actor.activity=AstronautActivity::Manual;
    }
    const bool resting=controlled_astronaut_>=0&&(test_astronauts_[controlled_astronaut_].activity==AstronautActivity::Sleeping||test_astronauts_[controlled_astronaut_].health<=0);
    const Vec3 displacement=resting?Vec3{}:first_person_.desired_displacement(input,dt);
    Vec3 current=first_person_.feet_position();
    if(length(displacement)>0.0001f){
        // Try the actual movement vector first. This is essential for rotated
        // doors: splitting a diagonal doorway crossing into world-X/world-Z
        // moves can make each artificial half-step hit the frame even though
        // the requested path goes cleanly through the opening. Preserve the
        // old axis attempts only as wall-sliding fallbacks.
        const Vec3 direct=current+displacement;
        const auto move_allowed=[&](Vec3 from,Vec3 to){return (first_person_.grounded() || to.y+first_person_.body_height()+0.02f<=first_person_ceiling_height(to)) && first_person_move_allowed(from,to);};
        if(move_allowed(current,direct)){
            current=direct;
        }else{
            Vec3 x_try{current.x+displacement.x,current.y,current.z};
            if(move_allowed(current,x_try)) current=x_try;
            Vec3 z_try{current.x,current.y,current.z+displacement.z};
            if(move_allowed(current,z_try)) current=z_try;
        }
    }
    first_person_.set_feet_position(current);
    const bool jump=!resting&&(input.key_space_pressed || input.key_j_pressed);
    const float ceiling=jump || !first_person_.grounded()?first_person_ceiling_height(current):std::numeric_limits<float>::max();
    first_person_.advance_vertical(jump,dt,first_person_floor_height(current),ceiling);
    sync_controlled_astronaut(dt);
    update_first_person_resource_hover();
    update_door_interaction(dt,input);
    if(input.left_pressed&&!first_person_pointer_over_hud(input)&&!build_catalog_.selected()&&!storage_gate_placement_active_&&corridor_link_source_<0)first_person_grab_or_drop_resource();
}

} // namespace game
