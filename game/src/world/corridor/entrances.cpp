#include "game/world/corridor/detail.hpp"
#include <array>

namespace game::corridor_detail {
float local_tangent_extent(const HabitatMesh& mesh,Vec3 local_normal){
    return std::abs(local_normal.x)>0.5f?mesh.footprint_z*0.5f:mesh.footprint_x*0.5f;
}

void consider_surface_candidate(Vec3 p,Vec3 n,Vec3 t,float tangent,float band,
                                float& best_n,float& best_diff,bool& found){
    const float tv=dot(flatten(p),t);
    const float diff=std::abs(tv-tangent);
    if(diff>band) return;
    const float nv=dot(flatten(p),n);
    if(!found || nv>best_n+0.01f || (std::abs(nv-best_n)<=0.01f && diff<best_diff)){
        best_n=nv; best_diff=diff; found=true;
    }
}

void consider_triangle_slice(const std::vector<Vec3>& vertices,Vec3 n,Vec3 t,float tangent,float band,float slice_y,
                             float& best_n,float& best_diff,bool& found){
    constexpr float EPS=0.0001f;
    for(std::size_t i=0;i+2<vertices.size();i+=3){
        const Vec3 tri_v[3]={vertices[i],vertices[i+1],vertices[i+2]};
        std::vector<Vec3> hits;
        hits.reserve(6);
        for(int e=0;e<3;++e){
            const Vec3 a=tri_v[e],b=tri_v[(e+1)%3];
            const float da=a.y-slice_y,db=b.y-slice_y;
            if(std::abs(da)<=EPS && std::abs(db)<=EPS){
                hits.push_back(a); hits.push_back(b);
                continue;
            }
            if((da>EPS && db>EPS) || (da<-EPS && db<-EPS)) continue;
            const float den=b.y-a.y;
            if(std::abs(den)<=EPS) continue;
            const float u=std::clamp((slice_y-a.y)/den,0.0f,1.0f);
            hits.push_back(a+(b-a)*u);
        }
        for(const Vec3& h:hits) consider_surface_candidate(h,n,t,tangent,band,best_n,best_diff,found);
        // If the requested tangent coordinate crosses the triangle's horizontal
        // slice, evaluate that exact point. This is the important part for
        // curved/domed shells: it places the generated vestibule on the real
        // triangle surface rather than on the nearest mesh vertex envelope.
        for(std::size_t a=0;a<hits.size();++a){
            for(std::size_t b=a+1;b<hits.size();++b){
                const float ta=dot(flatten(hits[a]),t),tb=dot(flatten(hits[b]),t);
                const float lo=std::min(ta,tb),hi=std::max(ta,tb);
                if(tangent<lo-EPS || tangent>hi+EPS || std::abs(tb-ta)<=EPS) continue;
                const float u=std::clamp((tangent-ta)/(tb-ta),0.0f,1.0f);
                consider_surface_candidate(hits[a]+(hits[b]-hits[a])*u,n,t,tangent,band,best_n,best_diff,found);
            }
        }
    }
}

// Locate the actual pressure-envelope surface at the door threshold height.
// Using triangle/plane intersections instead of vertex maxima keeps secondary
// entrances flush to curved domes and to the more complex industrial/medical/
// research hulls introduced in v0.14.0. Shell and glazing both count as the
// pressure envelope; exterior detail geometry intentionally does not.
std::optional<Vec3> local_hull_wall_point(const HabitatMesh& mesh,Vec3 local_normal,float tangent,float slice_y){
    local_normal=flat_normal(local_normal);
    const Vec3 local_tangent=perp_xz(local_normal);
    const float extent=std::max(0.5f,local_tangent_extent(mesh,local_normal));
    const float band=std::max(0.28f,extent*0.10f);
    float best_n=-std::numeric_limits<float>::max();
    float best_diff=std::numeric_limits<float>::max();
    bool found=false;
    consider_triangle_slice(mesh.shell,local_normal,local_tangent,tangent,band,slice_y,best_n,best_diff,found);
    consider_triangle_slice(mesh.glazing,local_normal,local_tangent,tangent,band,slice_y,best_n,best_diff,found);
    consider_triangle_slice(mesh.foundation,local_normal,local_tangent,tangent,band,slice_y,best_n,best_diff,found);
    if(!found) return std::nullopt;
    return local_normal*best_n+local_tangent*tangent+Vec3{0,slice_y,0};
}

std::optional<Vec3> local_hull_wall_point(const HabitatMesh& mesh,Vec3 local_normal,float tangent){
    return local_hull_wall_point(mesh,local_normal,tangent,geometry::kEntranceSliceY);
}

std::vector<Vec2> generated_door_profile(){
    std::vector<Vec2> out;
    out.reserve(13);
    constexpr float floor_y=0.05f;
    const float side_y=std::min(0.72f,GENERATED_ENTRANCE_HEIGHT*0.30f);
    out.push_back({-GENERATED_ENTRANCE_HALF_WIDTH,floor_y});
    out.push_back({ GENERATED_ENTRANCE_HALF_WIDTH,floor_y});
    constexpr int ARC=10;
    for(int i=0;i<=ARC;++i){
        const float a=(float)i/(float)ARC*PI;
        out.push_back({std::cos(a)*GENERATED_ENTRANCE_HALF_WIDTH,
                       side_y+std::sin(a)*(GENERATED_ENTRANCE_HEIGHT-side_y)});
    }
    return out;
}

void nearest_pressure_hit(const std::vector<Vec3>& triangles,Vec3 origin,Vec3 direction,
                          float& best_t,Vec3& best_normal,bool& found);

bool attach_surface_profile(CorridorDoor& door,const PlacedHabitat& habitat,const HabitatMesh& mesh){
    const Vec3 local_normal=flat_normal(rotate_y(door.normal,-habitat.rotation_degrees));
    const Vec3 local_tangent=perp_xz(local_normal);
    const Vec3 local_center=rotate_y(door.wall_position-habitat.position,-habitat.rotation_degrees);
    const float center_t=dot(flatten(local_center),local_tangent);
    const float radius=std::max(mesh.footprint_x,mesh.footprint_z)*0.80f+5.0f;
    const auto profile=generated_door_profile();
    std::vector<Vec3> world;
    world.reserve(profile.size());
    for(const Vec2 q:profile){
        const Vec3 origin=local_normal*radius+local_tangent*(center_t+q.x)+Vec3{0,q.y,0};
        const Vec3 direction=local_normal*-1.0f;
        float best_t=std::numeric_limits<float>::max();
        Vec3 surface_normal{};
        bool found=false;
        nearest_pressure_hit(mesh.shell,origin,direction,best_t,surface_normal,found);
        nearest_pressure_hit(mesh.glazing,origin,direction,best_t,surface_normal,found);
        nearest_pressure_hit(mesh.foundation,origin,direction,best_t,surface_normal,found);
        if(found){
            world.push_back(habitat.position+rotate_y(origin+direction*best_t,habitat.rotation_degrees));
            continue;
        }
        // Faceted/open sections can miss a ray exactly along a triangle seam;
        // fall back to the horizontal slice solver at the same profile vertex.
        auto local=local_hull_wall_point(mesh,local_normal,center_t+q.x,q.y);
        if(!local) return false;
        world.push_back(habitat.position+rotate_y(*local,habitat.rotation_degrees));
    }
    door.wall_profile=std::move(world);
    return true;
}

CorridorDoor transformed_builtin_door(const PlacedHabitat& habitat,const HabitatMesh& mesh,std::size_t entrance_index){
    HabitatEntrance local{};
    if(entrance_index<mesh.entrances.size()) local=mesh.entrances[entrance_index];
    CorridorDoor d;
    d.position=habitat.position+rotate_y(local.local_position,habitat.rotation_degrees);
    // Corridor and structure decks share one threshold level.  Keep the door
    // anchor on that level even when the authored hull mesh contains an old
    // elevated hatch marker.
    d.position.y=geometry::kEntranceSliceY;
    d.wall_position=d.position;
    d.normal=flat_normal(rotate_y(local.local_normal,habitat.rotation_degrees));
    d.generated=false;
    d.entrance_index=(int)entrance_index;
    return d;
}


bool ray_triangle(Vec3 origin,Vec3 direction,Vec3 a,Vec3 b,Vec3 c,float& t,Vec3& normal){
    constexpr float EPS=1.0e-6f;
    const Vec3 e1=b-a,e2=c-a;
    const Vec3 p=cross(direction,e2);
    const float det=dot(e1,p);
    if(std::abs(det)<EPS) return false;
    const float inv=1.0f/det;
    const Vec3 q0=origin-a;
    const float u=dot(q0,p)*inv;
    if(u<-EPS || u>1.0f+EPS) return false;
    const Vec3 q=cross(q0,e1);
    const float v=dot(direction,q)*inv;
    if(v<-EPS || u+v>1.0f+EPS) return false;
    t=dot(e2,q)*inv;
    if(t<=EPS) return false;
    normal=normalize(cross(e1,e2));
    return true;
}

void nearest_pressure_hit(const std::vector<Vec3>& triangles,Vec3 origin,Vec3 direction,
                          float& best_t,Vec3& best_normal,bool& found){
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        float t=0.0f; Vec3 n{};
        if(ray_triangle(origin,direction,triangles[i],triangles[i+1],triangles[i+2],t,n) && t<best_t){
            best_t=t; best_normal=n; found=true;
        }
    }
}

bool entrance_slot_clear(std::size_t habitat_index,const CorridorDoor& candidate,
                         const PlacedHabitat& habitat,const HabitatMesh& mesh,
                         const std::vector<CorridorConnection>& existing);

// Corridor endpoints use the same interaction model as supply ports: the whole
// habitat snaps as a target, but the cursor chooses the exact pressure-hull
// location.  The ray is evaluated at door threshold height against real shell
// and glazing triangles, so the generated entrance physically begins on the
// rendered hull instead of on the rectangular footprint.
std::optional<CorridorDoor> generated_cursor_door(std::size_t habitat_index,
                                                   const PlacedHabitat& habitat,
                                                   const HabitatMesh& mesh,
                                                   Vec3 cursor_world,
                                                   const std::vector<CorridorConnection>& existing){
    Vec3 outward=cursor_world-habitat.position;
    outward.y=0.0f;
    if(length(outward)<0.05f) outward={1,0,0};
    outward=normalize(outward);
    const Vec3 local_outward=flat_normal(rotate_y(outward,-habitat.rotation_degrees));
    const float radius=std::max(mesh.footprint_x,mesh.footprint_z)*0.75f+4.0f;
    constexpr float DOOR_SLICE_Y=geometry::kEntranceSliceY;
    const Vec3 origin=local_outward*radius+Vec3{0,DOOR_SLICE_Y,0};
    const Vec3 direction=local_outward*-1.0f;
    float best_t=std::numeric_limits<float>::max();
    Vec3 local_normal{};
    bool found=false;
    nearest_pressure_hit(mesh.shell,origin,direction,best_t,local_normal,found);
    nearest_pressure_hit(mesh.glazing,origin,direction,best_t,local_normal,found);
    nearest_pressure_hit(mesh.foundation,origin,direction,best_t,local_normal,found);
    if(!found) return std::nullopt;

    const Vec3 local_hit=origin+direction*best_t;
    // Keep the surface normal pointing away from the habitat center. Curved and
    // faceted structures therefore get a door collar aligned to the real face.
    local_normal=flat_normal(local_normal);
    if(length(local_normal)<0.01f) local_normal=local_outward;
    if(dot(local_normal,local_outward)<0.0f) local_normal=local_normal*-1.0f;

    CorridorDoor d;
    d.wall_position=habitat.position+rotate_y(local_hit,habitat.rotation_degrees);
    d.wall_position.y=geometry::kEntranceSliceY;
    d.normal=flat_normal(rotate_y(local_normal,habitat.rotation_degrees));
    d.position=d.wall_position+d.normal*GENERATED_ENTRANCE_DEPTH;
    d.generated=true;
    d.entrance_index=-1;
    if(!attach_surface_profile(d,habitat,mesh)) return std::nullopt;
    if(!entrance_slot_clear(habitat_index,d,habitat,mesh,existing)) return std::nullopt;
    return d;
}

bool door_occupied(std::size_t habitat_index,const CorridorDoor& candidate,const std::vector<CorridorConnection>& existing){
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.position,ep->door.position)<1.20f) return true;
        }
    }
    return false;
}

bool entrance_slot_clear(std::size_t habitat_index,const CorridorDoor& candidate,
                         const PlacedHabitat& habitat,const HabitatMesh& mesh,
                         const std::vector<CorridorConnection>& existing){
    for(std::size_t i=0;i<mesh.entrances.size();++i){
        const CorridorDoor builtin=transformed_builtin_door(habitat,mesh,i);
        if(distance_xz(candidate.wall_position,builtin.wall_position)<ENTRANCE_MIN_SPACING) return false;
    }
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.wall_position,ep->door.wall_position)<ENTRANCE_MIN_SPACING) return false;
            if(distance_xz(candidate.position,ep->door.position)<ENTRANCE_MIN_SPACING) return false;
        }
    }
    return true;
}

std::optional<CorridorDoor> generated_face_door(std::size_t habitat_index,
                                                const PlacedHabitat& habitat,
                                                const HabitatMesh& mesh,
                                                Vec3 target,
                                                const std::vector<CorridorConnection>& existing){
    const Vec3 target_dir=flat_normal(target-habitat.position);
    const std::array<Vec3,4> normals={Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}};
    const std::array<float,7> slots={0.0f,0.30f,-0.30f,0.50f,-0.50f,0.68f,-0.68f};
    bool found=false;
    float best=std::numeric_limits<float>::max();
    CorridorDoor result{};
    for(const Vec3 ln:normals){
        const Vec3 wn=flat_normal(rotate_y(ln,habitat.rotation_degrees));
        const float facing_penalty=(1.0f-dot(wn,target_dir))*4.0f;
        const float tangent_extent=local_tangent_extent(mesh,ln);
        for(float factor:slots){
            auto local_wall=local_hull_wall_point(mesh,ln,factor*tangent_extent);
            if(!local_wall) continue; // never invent a door on a footprint plane with no pressure hull
            CorridorDoor d;
            d.wall_position=habitat.position+rotate_y(*local_wall,habitat.rotation_degrees);
            d.wall_position.y=geometry::kEntranceSliceY;
            d.normal=wn;
            d.position=d.wall_position+d.normal*GENERATED_ENTRANCE_DEPTH;
            d.generated=true;
            d.entrance_index=-1;
            if(!attach_surface_profile(d,habitat,mesh)) continue;
            if(!entrance_slot_clear(habitat_index,d,habitat,mesh,existing)) continue;
            const float score=distance_xz(d.position,target)+facing_penalty+std::abs(factor)*0.4f;
            if(score<best){ best=score; result=d; found=true; }
        }
    }
    if(!found) return std::nullopt;
    return result;
}

CorridorEndpoint resolve_open_or_corridor(const CorridorBuildAnchor& anchor,Vec3 other,
                                          const std::vector<CorridorConnection>& existing){
    CorridorEndpoint out;
    out.kind=anchor.kind;
    out.index=anchor.index;
    out.position=anchor.position;
    out.position.y=0;
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<existing.size()){
        const auto& host=existing[anchor.index];
        // If the cursor targets a still-open corridor end, treat this as a true
        // pressure-tube extension. Snap to the host's terminal center/ring and
        // preserve its exact outward tangent. The new corridor therefore starts
        // with the same D-shaped cross-section plane.
        constexpr float END_SNAP_RADIUS=0.42f;
        if(host.path.size()>=2 && host.endpoint_a.kind==CorridorEndpointKind::OpenDoor && !host.endpoint_a.suppress_open_door &&
           distance_xz(anchor.position,host.path.front())<=END_SNAP_RADIUS){
            out.position=host.path.front();
            out.host_position=out.position;
            out.host_tangent=flat_normal(host.path.front()-host.path[1]);
            out.tangent=out.host_tangent;
            out.seamless_splice=true;
            return out;
        }
        if(host.path.size()>=2 && host.endpoint_b.kind==CorridorEndpointKind::OpenDoor && !host.endpoint_b.suppress_open_door &&
           distance_xz(anchor.position,host.path.back())<=END_SNAP_RADIUS){
            out.position=host.path.back();
            out.host_position=out.position;
            out.host_tangent=flat_normal(host.path.back()-host.path[host.path.size()-2]);
            out.tangent=out.host_tangent;
            out.seamless_splice=true;
            return out;
        }

        // Every non-terminal point on a corridor is an exact selectable intersection.
        // Only a very small click zone at an exposed end is treated as an end
        // extension; everywhere else the branch uses the nearest centerline
        // point so intersections can be created even close to corridor ends.
        // Mid-corridor branches no longer terminate on the host centerline.
        // Resolve the exact centerline point first, then leave the host
        // perpendicular to its local tangent. The normal corridor route ends a
        // short distance outside the host and a dedicated adapter is projected
        // back onto the host's real D-profile surface. This prevents the branch
        // tube from penetrating halfway through the host corridor.
        out.host_position=nearest_path_point(host.path,anchor.position);
        out.host_tangent=nearest_path_tangent(host.path,out.host_position);
        Vec3 side=perp_xz(out.host_tangent);
        if(length(side)<0.001f) side={0,0,1};

        // Preserve the side the player actually pointed at.  The corridor
        // anchor itself is snapped to the host centerline for a clean junction,
        // but corridor_anchor() also stores the unsnapped mouse hit.  Use that
        // lateral offset first so moving the cursor across the corridor flips
        // the intersection to the opposite side.  Programmatic anchors and a
        // pointer exactly on the centerline retain the old destination-based
        // fallback, which keeps automatic routing deterministic.
        float side_choice=0.0f;
        if(anchor.has_cursor_position)
            side_choice=dot(flatten(anchor.cursor_position-out.host_position),side);
        if(std::abs(side_choice)<0.02f)
            side_choice=dot(flatten(other-out.host_position),side);
        if(side_choice<0.0f) side=side*-1.0f;
        out.tangent=flat_normal(side);

        // Keep the branch ring outside the host pressure shell, but adapt the
        // short surface sleeve to the actual available gap.  A fixed 0.42 m
        // sleeve used to overshoot entrances that sit very close to an existing
        // corridor and made otherwise valid connections self-intersect.  The
        // host aperture still uses the exact centerline point, so this remains a
        // true merged junction rather than overlapping pressure tubes.
        const float center_gap=distance_xz(other,out.host_position);
        const float free_gap=std::max(0.0f,center_gap-CORRIDOR_HALF_WIDTH);
        const float surface_adapter_run=std::clamp(free_gap*0.45f,0.08f,0.42f);
        out.position=out.host_position+out.tangent*(CORRIDOR_HALF_WIDTH+surface_adapter_run);
        out.position.y=out.host_position.y;
        out.surface_matched_junction=true;
        return out;
    }
    out.tangent=flat_normal(other-out.position);
    if(length(out.tangent)<0.001f) out.tangent=Vec3{1,0,0};
    return out;
}

struct ExistingDoorUse {
    std::size_t corridor_index{};
    bool endpoint_a{true};
    CorridorDoor door{};
};

std::optional<ExistingDoorUse> existing_door_use_near(CorridorEndpointKind owner_kind,std::size_t owner_index,Vec3 point,
                                                       const std::vector<CorridorConnection>& existing,
                                                       float radius){
    std::optional<ExistingDoorUse> best;
    float best_distance=radius;
    for(std::size_t i=0;i<existing.size();++i){
        for(int side=0;side<2;++side){
            const CorridorEndpoint& ep=side==0?existing[i].endpoint_a:existing[i].endpoint_b;
            const bool owns=(owner_kind==CorridorEndpointKind::Habitat && endpoint_uses_habitat(ep,owner_index)) ||
                            (owner_kind==CorridorEndpointKind::Resource && endpoint_uses_resource(ep,owner_index));
            if(!owns) continue;
            const float d=distance_xz(point,ep.door.wall_position);
            if(d<best_distance){
                best_distance=d;
                best=ExistingDoorUse{i,side==0,ep.door};
            }
        }
    }
    return best;
}

Vec3 point_along_path_from_door(const CorridorConnection& c,bool endpoint_a,float wanted){
    if(c.path.empty()) return endpoint_a?c.endpoint_a.position:c.endpoint_b.position;
    if(c.path.size()==1) return c.path.front();
    float remaining=std::max(0.0f,wanted);
    if(endpoint_a){
        for(std::size_t i=0;i+1<c.path.size();++i){
            const float len=distance_xz(c.path[i],c.path[i+1]);
            if(len>=remaining && len>0.0001f){
                const float t=remaining/len;
                return c.path[i]+(c.path[i+1]-c.path[i])*t;
            }
            remaining-=len;
        }
        return c.path.back();
    }
    for(std::size_t i=c.path.size()-1;i>0;--i){
        const float len=distance_xz(c.path[i],c.path[i-1]);
        if(len>=remaining && len>0.0001f){
            const float t=remaining/len;
            return c.path[i]+(c.path[i-1]-c.path[i])*t;
        }
        remaining-=len;
    }
    return c.path.front();
}

std::optional<CorridorEndpoint> branch_from_existing_door(const ExistingDoorUse& use,Vec3 other,
                                                          const std::vector<CorridorConnection>& existing){
    if(use.corridor_index>=existing.size()) return std::nullopt;
    // A used entrance owns one sealed trunk.  Additional corridors always
    // reuse that trunk instead of creating another coincident vestibule.  There
    // is intentionally no fixed branch-count limit: the committed network fan
    // optimizer extends/rebalances the trunk as more destinations are added.
    const auto& root=existing[use.corridor_index];
    const Vec3 root_end=(use.endpoint_a?root.path.front():root.path.back());
    int attached_branches=0;
    constexpr float REUSE_FAN_ZONE=48.0f;
    for(std::size_t i=0;i<existing.size();++i){
        if(i==use.corridor_index || existing[i].kind!=CorridorConnectionKind::BuiltSection) continue;
        for(const CorridorEndpoint* ep:{&existing[i].endpoint_a,&existing[i].endpoint_b}){
            if(ep->kind!=CorridorEndpointKind::Corridor || ep->index!=use.corridor_index) continue;
            if(distance_xz(ep->host_position,root_end)<=REUSE_FAN_ZONE) ++attached_branches;
        }
    }

    // Preview the new branch progressively farther down the common trunk.  The
    // final optimizer will place left/right branches independently, but this
    // avoids stacking every pre-build preview on the same junction point.
    const float branch_run=1.75f+1.25f*(float)attached_branches;
    const Vec3 branch_point=point_along_path_from_door(root,use.endpoint_a,branch_run);
    CorridorBuildAnchor host{CorridorEndpointKind::Corridor,use.corridor_index,branch_point};
    return resolve_open_or_corridor(host,other,existing);
}

std::optional<CorridorEndpoint> resolve_habitat(const CorridorBuildAnchor& anchor,Vec3 other,
                                                const std::vector<PlacedHabitat>& habitats,
                                                const HabitatModelLibrary& library,
                                                const std::vector<CorridorConnection>& existing){
    if(anchor.index>=habitats.size()) return std::nullopt;
    const auto& habitat=habitats[anchor.index];
    const auto& mesh=library.mesh(habitat);

    // An already-used entrance remains a valid build source. Additional
    // corridors merge into the existing sealed trunk just outside that door
    // instead of creating another overlapping entrance/vestibule.
    if(auto used=existing_door_use_near(CorridorEndpointKind::Habitat,anchor.index,anchor.position,existing,1.15f))
        return branch_from_existing_door(*used,other,existing);

    // Reuse a built-in hatch only when the cursor explicitly targets it. This
    // keeps established entrances convenient while making all other hull points
    // freely adjustable like supply-channel ports.
    constexpr float BUILTIN_SNAP_RADIUS=ENTRANCE_MIN_SPACING;
    bool found_builtin=false;
    float best_cursor_distance=BUILTIN_SNAP_RADIUS;
    CorridorDoor d{};
    for(std::size_t i=0;i<mesh.entrances.size();++i){
        CorridorDoor candidate=transformed_builtin_door(habitat,mesh,i);
        const float cursor_distance=distance_xz(anchor.position,candidate.wall_position);
        if(cursor_distance<best_cursor_distance){
            best_cursor_distance=cursor_distance;
            d=candidate;
            found_builtin=true;
        }
    }

    if(found_builtin){
        if(auto used=existing_door_use_near(CorridorEndpointKind::Habitat,anchor.index,d.wall_position,existing,0.55f))
            return branch_from_existing_door(*used,other,existing);
    }

    if(!found_builtin){
        auto generated=generated_cursor_door(anchor.index,habitat,mesh,anchor.position,existing);
        // Domed/faceted habitats can present a cursor ray through a seam or
        // through non-pressure exterior geometry. Fall back to the nearest real
        // pressure-hull face instead of dropping the blueprint entrance. This
        // is especially important for the Bio Dome size variants.
        if(!generated) generated=generated_face_door(anchor.index,habitat,mesh,anchor.position,existing);
        if(!generated){
            // A crowded/faceted hull must never make a habitat unreachable. If
            // no new generated opening fits, reuse an existing pressure entrance
            // first. For a previously unconnected habitat, fall back to its
            // nearest authored entrance even when the cursor was not directly on
            // the hatch. This preserves "any structure can be connected" while
            // still preferring the exact cursor-selected hull point whenever it
            // is physically possible.
            if(auto used=existing_door_use_near(CorridorEndpointKind::Habitat,anchor.index,anchor.position,existing,
                                                std::numeric_limits<float>::max()))
                return branch_from_existing_door(*used,other,existing);
            if(!mesh.entrances.empty()){
                float best_dist=std::numeric_limits<float>::max();
                CorridorDoor fallback{};
                for(std::size_t i=0;i<mesh.entrances.size();++i){
                    CorridorDoor candidate=transformed_builtin_door(habitat,mesh,i);
                    const float dist=distance_xz(candidate.wall_position,anchor.position);
                    if(dist<best_dist){ best_dist=dist; fallback=candidate; }
                }
                d=fallback;
                found_builtin=true;
            }else{
                return std::nullopt;
            }
        }else{
            d=*generated;
        }
    }

    // The route tangent must follow the actual door normal. The opposite point
    // only influences routing after the airtight structure-to-corridor sleeve.
    CorridorEndpoint out;
    out.kind=CorridorEndpointKind::Habitat;
    out.index=anchor.index;
    out.door=d;
    out.tangent=d.normal;

    // Fit the structure docking sleeve to the available gap when the selected
    // destination is an already-existing corridor.  A fixed 1.75 m sleeve can
    // otherwise extend through a corridor that runs directly past the entrance.
    // Stopping just before the target pressure shell lets the corridor junction
    // adapter own the final seam, so even very close doors merge without any
    // coincident tube geometry.
    float dock_run=DOCK_RUN;
    const Vec3 rel=flatten(other-d.position);
    const float forward=dot(rel,flat_normal(d.normal));
    const float lateral=std::abs(dot(rel,perp_xz(d.normal)));
    if(forward>0.0f && lateral<CORRIDOR_HALF_WIDTH*1.65f && forward<DOCK_RUN+CORRIDOR_HALF_WIDTH*2.2f)
        dock_run=std::clamp(forward-CORRIDOR_HALF_WIDTH-0.10f,0.08f,DOCK_RUN);
    out.position=d.position+d.normal*dock_run;
    out.position.y=0;
    return out;
}

std::optional<CorridorEndpoint> resolve_storage_dome(const CorridorBuildAnchor& anchor,Vec3 other,
                                                     const std::vector<PlacedResource>& resources,
                                                     const ResourceModelLibrary& library,
                                                     const std::vector<CorridorConnection>& existing){
    if(anchor.index>=resources.size()) return std::nullopt;
    const auto& placed=resources[anchor.index];
    if(placed.kind!=ResourceKind::StorageDome) return std::nullopt;
    const auto& mesh=library.mesh(placed);
    const float wall_radius=std::min(mesh.footprint_x,mesh.footprint_z)*0.40f;
    const Vec3 local_normals[4]={{0,0,-1},{1,0,0},{0,0,1},{-1,0,0}};
    float best=std::numeric_limits<float>::max();
    Vec3 normal{0,0,-1},wall{};
    for(Vec3 ln:local_normals){
        const Vec3 n=flat_normal(rotate_y(ln,placed.rotation_degrees));
        const Vec3 w=placed.position+n*wall_radius;
        const float d=distance_xz(anchor.position,w);
        if(d<best){ best=d; normal=n; wall=w; }
    }
    // If the pointer is near the center, choose the port facing the target.
    if(distance_xz(anchor.position,placed.position)<0.40f){
        Vec3 desired=flat_normal(other-placed.position);
        float best_dot=-2.0f;
        for(Vec3 ln:local_normals){
            const Vec3 n=flat_normal(rotate_y(ln,placed.rotation_degrees));
            const float score=dot(n,desired);
            if(score>best_dot){ best_dot=score; normal=n; wall=placed.position+n*wall_radius; }
        }
    }

    // Storage-dome service doors are reusable pressure entrances too.  If the
    // selected face already owns a corridor, branch from its common trunk
    // rather than laying a second corridor through the same docking sleeve.
    if(auto used=existing_door_use_near(CorridorEndpointKind::Resource,anchor.index,wall,existing,0.85f))
        return branch_from_existing_door(*used,other,existing);

    CorridorEndpoint out;
    out.kind=CorridorEndpointKind::Resource;
    out.index=anchor.index;
    out.door.generated=false;
    out.door.entrance_index=-1;
    out.door.normal=normal;
    out.door.wall_position=wall+Vec3{0,geometry::kEntranceSliceY,0};
    out.door.position=wall+Vec3{0,geometry::kEntranceSliceY,0};
    out.tangent=normal;
    float dock_run=DOCK_RUN;
    const Vec3 rel=flatten(other-wall);
    const float forward=dot(rel,normal);
    const float lateral=std::abs(dot(rel,perp_xz(normal)));
    if(forward>0.0f && lateral<CORRIDOR_HALF_WIDTH*1.65f && forward<DOCK_RUN+CORRIDOR_HALF_WIDTH*2.2f)
        dock_run=std::clamp(forward-CORRIDOR_HALF_WIDTH-0.10f,0.08f,DOCK_RUN);
    out.position=wall+normal*dock_run;
    out.position.y=0.0f;
    return out;
}

std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,
                                               const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing){
    if(anchor.kind==CorridorEndpointKind::Habitat) return resolve_habitat(anchor,other,habitats,library,existing);
    if(anchor.kind==CorridorEndpointKind::Resource) return std::nullopt;
    return resolve_open_or_corridor(anchor,other,existing);
}

std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                               const std::vector<CorridorConnection>& existing){
    if(anchor.kind==CorridorEndpointKind::Resource) return resolve_storage_dome(anchor,other,resources,resource_library,existing);
    return resolve_anchor(anchor,other,habitats,library,existing);
}
} // namespace game::corridor_detail
