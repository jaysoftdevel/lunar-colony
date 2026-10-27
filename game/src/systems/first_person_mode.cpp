#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include "game/world/corridor/detail.hpp"
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

bool projected_mesh_contains_body(const std::vector<Vec3>& triangles,Vec3 local,float radius);

bool habitat_uses_elliptical_hull(HabitatKind kind){
    return kind==HabitatKind::OxygenGeneratorDome ||
           kind==HabitatKind::CantineDome ||
           kind==HabitatKind::BioDome;
}

bool inside_habitat_hull(Vec3 p,const PlacedHabitat& h,const HabitatMesh& mesh,float inset){
    const Vec3 local=rotate_y(p-h.position,-h.rotation_degrees);
    // Use the actual projected pressure-shell triangles as the primary FPV
    // boundary.  This keeps rounded/irregular structures on their rendered
    // mesh instead of permitting movement in an invisible footprint box.
    if(!mesh.shell.empty()) return projected_mesh_contains_body(mesh.shell,local,std::max(0.0f,inset));
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

bool point_in_doorway_clearance(Vec3 p,const CorridorDoor& door,float body_radius){
    Vec3 n=door.normal; n.y=0.0f; n=normalize(n);
    if(length(n)<0.001f) return false;
    const Vec3 side{-n.z,0.0f,n.x};
    const Vec3 rel=p-door.wall_position;
    const float longitudinal=std::abs(dot(Vec3{rel.x,0.0f,rel.z},n));
    const float lateral=std::abs(dot(Vec3{rel.x,0.0f,rel.z},side));
    const float usable=std::max(0.16f,geometry::kDoorClearWidth*0.5f-body_radius*0.45f);
    return longitudinal<=body_radius+0.26f && lateral<=usable;
}

bool point_in_any_doorway_clearance(Vec3 p,const std::vector<CorridorDoor>& doors,float body_radius){
    for(const auto& door:doors) if(point_in_doorway_clearance(p,door,body_radius)) return true;
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
    return dominant_floor_height(mesh.interior,mesh.foundation,geometry::kHabitatFloorY);
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

std::vector<CorridorDoor> GameApp::habitat_doors(std::size_t habitat_index) const {
    std::vector<CorridorDoor> doors;
    if(habitat_index>=placed_habitats_.size()) return doors;
    const auto& h=placed_habitats_[habitat_index];
    const auto& mesh=habitat_models_.mesh(h);
    for(std::size_t i=0;i<mesh.entrances.size();++i){
        const auto& e=mesh.entrances[i];
        CorridorDoor d;
        d.position=h.position+rotate_y(e.local_position,h.rotation_degrees);
        d.wall_position=d.position;
        d.normal=normalize(rotate_y(e.local_normal,h.rotation_degrees));
        d.entrance_index=(int)i;
        doors.push_back(d);
    }
    for(const auto& c:corridor_network_.connections()){
        if(!c.construction.complete()) continue;
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Habitat || ep->index!=habitat_index) continue;
            bool duplicate=false;
            for(const auto& d:doors) if(dist_xz_local(d.wall_position,ep->door.wall_position)<0.35f){ duplicate=true; break; }
            if(!duplicate) doors.push_back(ep->door);
        }
    }
    return doors;
}

std::vector<CorridorDoor> GameApp::storage_dome_doors(std::size_t resource_index) const {
    std::vector<CorridorDoor> doors;
    if(resource_index>=placed_resources_.size()) return doors;
    const auto& item=placed_resources_[resource_index];
    if(item.kind!=ResourceKind::StorageDome) return doors;
    const auto& mesh=resource_models_.mesh(item);
    const float wall_radius=std::min(mesh.footprint_x,mesh.footprint_z)*0.40f;
    const Vec3 local_normals[4]={{0,0,-1},{1,0,0},{0,0,1},{-1,0,0}};
    for(Vec3 local:local_normals){
        CorridorDoor d;
        d.normal=normalize(rotate_y(local,item.rotation_degrees));
        d.wall_position=item.position+d.normal*wall_radius+Vec3{0,geometry::kEntranceSliceY,0};
        d.position=d.wall_position;
        doors.push_back(d);
    }
    return doors;
}

float GameApp::first_person_door_open_fraction(Vec3 door_position) const {
    if(first_person_mode_!=FirstPersonMode::Active) return 0.0f;
    const float d=dist_xz_local(first_person_.feet_position(),door_position);
    return std::clamp((geometry::kDoorTriggerRadius+0.35f-d)/1.25f,0.0f,1.0f);
}

bool GameApp::first_person_spawn_valid(Vec3 p) const {
    const float half=(float)playfield_.size()*0.5f-0.35f;
    if(p.x<-half || p.x>half || p.z<-half || p.z>half) return false;
    for(const auto& item:placed_resources_){
        const auto& mesh=resource_models_.mesh(item);
        if(item.construction.complete() && item.kind==ResourceKind::StorageDome){
            if(inside_storage_hull(p,item,mesh,0.0f) &&
               !inside_storage_hull(p,item,mesh,first_person_.body_radius())) return false;
            continue;
        }
        if(inside_solid_resource_hull(p,item,mesh,first_person_.body_radius()*0.40f)) return false;
    }
    for(const auto& h:placed_habitats_){
        const auto& mesh=habitat_models_.mesh(h);
        if(h.construction.complete()){
            if(inside_habitat_hull(p,h,mesh,0.0f) &&
               !inside_habitat_hull(p,h,mesh,first_person_.body_radius())) return false;
            continue;
        }
        if(inside_habitat_hull(p,h,mesh,first_person_.body_radius()*0.40f)) return false;
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
            const float basement=habitat_basement_height(mesh);
            const Vec3 local=rotate_y(p-h.position,-h.rotation_degrees);
            // Follow the rendered basement slab where it exists, but deliberately
            // ignore furniture/platform tops far above the base walking level.
            if(auto y=walkable_surface_height_at(mesh.interior,local,basement-0.10f,basement+0.28f))
                return h.position.y+*y;
            return h.position.y+basement;
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
                corridor_floor=q.y+corridor_detail::WALKWAY_Y;
                floor_connection=&c;
            }
        }

        // Structure docking sleeves are part of the same walkable connection,
        // not a gap between the corridor and the building.  Include them in the
        // floor resolver so FPV can traverse every completed docking adapter.
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.position);
                const float d=dist_xz_local(p,q);
                const float sleeve_limit=std::max(0.20f,geometry::kDockSleeveHalfWidth-first_person_.body_radius());
                if(d<best_corridor && d<=sleeve_limit){
                    best_corridor=d;
                    corridor_floor=ep->position.y+corridor_detail::WALKWAY_Y;
                    floor_connection=&c;
                }
                continue;
            }
            // Exterior corridor endpoints carry a real two-door air-lock beyond
            // the route centerline. It uses the corridor deck elevation too.
            if(ep->kind==CorridorEndpointKind::OpenDoor && !ep->suppress_open_door){
                Vec3 n=ep->tangent; n.y=0.0f; n=normalize(n);
                if(length(n)<0.001f) continue;
                const Vec3 outer=ep->position-n*geometry::kCorridorAirlockLength;
                const Vec3 q=closest_point_segment_xz(p,ep->position,outer);
                const float d=dist_xz_local(p,q);
                const float airlock_limit=std::max(0.20f,geometry::kCorridorAirlockHalfWidth-first_person_.body_radius());
                if(d<best_corridor && d<=airlock_limit){
                    best_corridor=d;
                    corridor_floor=ep->position.y+corridor_detail::WALKWAY_Y;
                    floor_connection=&c;
                }
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
            if(corridor_network_.habitat_entrance_connected(hi,ei)) continue;
            if(auto y=walkable_surface_height_at(mesh.entrance_access[ei].details,local,0.05f,1.65f))
                return h.position.y+*y;
        }
    }
    return 0.0f;
}

bool GameApp::first_person_move_allowed(Vec3 from,Vec3 to) const {
    const float half=(float)playfield_.size()*0.5f-first_person_.body_radius();
    if(to.x<-half || to.x>half || to.z<-half || to.z>half) return false;
    const Vec3 mid=(from+to)*0.5f;

    // Non-enterable machinery remains solid to the full FPV body envelope.
    for(const auto& item:placed_resources_){
        if(!item.construction.complete() || item.kind==ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        if(inside_solid_resource_hull(to,item,mesh,first_person_.body_radius()) ||
           inside_solid_resource_hull(mid,item,mesh,first_person_.body_radius())) return false;
    }

    // Habitats are bounded by their actual rounded/domed hulls. A transition
    // between exterior and interior is legal only at a physical entrance.
    for(std::size_t i=0;i<placed_habitats_.size();++i){
        const auto& h=placed_habitats_[i];
        if(!h.construction.complete()) continue;
        const auto& mesh=habitat_models_.mesh(h);
        const bool a_center=inside_habitat_hull(from,h,mesh,0.0f);
        const bool b_center=inside_habitat_hull(to,h,mesh,0.0f);
        const bool m_center=inside_habitat_hull(mid,h,mesh,0.0f);
        const bool b_body=inside_habitat_hull(to,h,mesh,first_person_.body_radius());
        const auto doors=habitat_doors(i);
        const bool b_in_doorway=point_in_any_doorway_clearance(to,doors,first_person_.body_radius());
        if(a_center && b_center && !b_body && !b_in_doorway) return false;
        if(a_center!=b_center || (!a_center && !b_center && m_center)){
            if(!movement_crosses_any_door(from,to,doors,first_person_.body_radius())) return false;
            // The body may temporarily straddle the pressure shell while the
            // centre passes through the actual door aperture. Once clear of
            // the frame, full mesh-level body containment is required again.
            if(b_center && !b_body && !b_in_doorway) return false;
        }
    }

    // Storage Domes use their circular pressure hull rather than the square
    // construction footprint. The body cannot escape the dome wall except via
    // one of the four docking doors.
    for(std::size_t i=0;i<placed_resources_.size();++i){
        const auto& item=placed_resources_[i];
        if(!item.construction.complete() || item.kind!=ResourceKind::StorageDome) continue;
        const auto& mesh=resource_models_.mesh(item);
        const bool a_center=inside_storage_hull(from,item,mesh,0.0f);
        const bool b_center=inside_storage_hull(to,item,mesh,0.0f);
        const bool m_center=inside_storage_hull(mid,item,mesh,0.0f);
        const bool b_body=inside_storage_hull(to,item,mesh,first_person_.body_radius());
        const auto doors=storage_dome_doors(i);
        const bool b_in_doorway=point_in_any_doorway_clearance(to,doors,first_person_.body_radius());
        if(a_center && b_center && !b_body && !b_in_doorway) return false;
        if(a_center!=b_center || (!a_center && !b_center && m_center)){
            if(!movement_crosses_any_door(from,to,doors,first_person_.body_radius())) return false;
            if(b_center && !b_body && !b_in_doorway) return false;
        }
    }

    auto corridor_state=[&](Vec3 p,float radius){
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete() || c.path.size()<2) continue;
            for(std::size_t j=0;j+1<c.path.size();++j){
                const Vec3 q=closest_point_segment_xz(p,c.path[j],c.path[j+1]);
                if(dist_xz_local(p,q)<=radius) return true;
            }
            // Treat the docking sleeve as part of the walkable corridor volume.
            // Without this, the FPV left the corridor collision domain before
            // reaching the structure door and could step sideways through the
            // invisible side of the vestibule.
            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                    const Vec3 q=closest_point_segment_xz(p,ep->position,ep->door.position);
                    const float sleeve_radius=std::max(0.18f,geometry::kDockSleeveHalfWidth-first_person_.body_radius());
                    if(dist_xz_local(p,q)<=sleeve_radius) return true;
                    continue;
                }
                if(ep->kind==CorridorEndpointKind::OpenDoor && !ep->suppress_open_door){
                    Vec3 n=ep->tangent; n.y=0.0f; n=normalize(n);
                    if(length(n)<0.001f) continue;
                    const Vec3 outer=ep->position-n*geometry::kCorridorAirlockLength;
                    const Vec3 q=closest_point_segment_xz(p,ep->position,outer);
                    const float airlock_radius=std::max(0.18f,geometry::kCorridorAirlockHalfWidth-first_person_.body_radius());
                    if(dist_xz_local(p,q)<=airlock_radius) return true;
                }
            }
        }
        return false;
    };
    const float corridor_center_limit=std::max(0.20f,corridor_detail::CORRIDOR_HALF_WIDTH-first_person_.body_radius());
    const bool in_corridor_from=corridor_state(from,corridor_center_limit);
    const bool in_corridor_to=corridor_state(to,corridor_center_limit);
    const bool in_corridor_mid=corridor_state(mid,corridor_center_limit);
    if(in_corridor_from!=in_corridor_to || (!in_corridor_from && !in_corridor_to && in_corridor_mid)){
        bool transition=false;
        for(const auto& c:corridor_network_.connections()){
            if(!c.construction.complete()) continue;
            for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
                if(ep->kind==CorridorEndpointKind::OpenDoor && !ep->suppress_open_door){
                    Vec3 n=ep->tangent; n.y=0.0f; n=normalize(n);
                    if(length(n)>0.001f){
                        // Exterior transitions happen at the OUTER air-lock
                        // door.  The corridor endpoint itself is the inner
                        // pressure door and remains inside the corridor domain.
                        const Vec3 outer=ep->position-n*geometry::kCorridorAirlockLength;
                        transition=movement_crosses_portal(from,to,outer,n,
                                                           geometry::kDoorClearWidth*0.5f,first_person_.body_radius());
                    }
                }else if(ep->kind==CorridorEndpointKind::Habitat || ep->kind==CorridorEndpointKind::Resource){
                    transition=movement_crosses_doorway(from,to,ep->door,first_person_.body_radius());
                }
                if(transition) break;
            }
            if(transition) break;
        }
        if(!transition) return false;
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
            if(ep->kind!=CorridorEndpointKind::OpenDoor || ep->suppress_open_door) continue;
            Vec3 n=ep->tangent; n.y=0.0f; n=normalize(n);
            if(length(n)<0.001f) continue;
            const Vec3 outer=ep->position-n*geometry::kCorridorAirlockLength;
            const Vec3 q=closest_point_segment_xz(p,ep->position,outer);
            if(dist_xz_local(p,q)<=std::max(0.20f,geometry::kCorridorAirlockHalfWidth-first_person_.body_radius()))
                return "AIRLOCK / PRESSURE BUFFER";
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
        Vec3 x_try{current.x+displacement.x,current.y,current.z};
        if(first_person_move_allowed(current,x_try)) current=x_try;
        Vec3 z_try{current.x,current.y,current.z+displacement.z};
        if(first_person_move_allowed(current,z_try)) current=z_try;
    }
    const float target_floor=first_person_floor_height(current);
    const float max_step=std::max(0.08f,dt*2.8f);
    current.y += std::clamp(target_floor-current.y,-max_step,max_step);
    first_person_.set_feet_position(current);
    update_first_person_resource_hover();
    if(input.left_pressed) first_person_grab_or_drop_resource();
}

} // namespace game
