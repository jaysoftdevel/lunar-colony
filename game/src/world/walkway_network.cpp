#include "game/world/walkway_network.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;
namespace {
constexpr float PI=3.14159265358979323846f;
constexpr float HALF_WIDTH=0.78f;
constexpr float DECK_Y=0.20f;
constexpr float CLEARANCE=0.55f;
constexpr float CORRIDOR_HALF_WIDTH=1.08f;
constexpr float CORRIDOR_OCCUPANCY=HALF_WIDTH+CORRIDOR_HALF_WIDTH+0.18f;
constexpr float CORRIDOR_CROSS_MIN_SIN=0.42f;
constexpr float CORRIDOR_CROSS_KEEP_CLEAR=2.55f;
constexpr float SUPPLY_OCCUPANCY=HALF_WIDTH+0.34f+0.18f;
constexpr float SUPPLY_CROSS_MIN_SIN=0.24f;
constexpr float BRIDGE_RISE=0.92f;
constexpr float BRIDGE_HALF_SPAN=3.00f;

Vec3 flat(Vec3 v){ v.y=0.0f; return v; }
Vec3 flat_normal(Vec3 v){ v=flat(v); const float l=length(v); return l>0.0001f?v*(1.0f/l):Vec3{1,0,0}; }
Vec3 side_of(Vec3 v){ v=flat_normal(v); return {-v.z,0,v.x}; }
float dist_xz(Vec3 a,Vec3 b){ const float dx=a.x-b.x,dz=a.z-b.z; return std::sqrt(dx*dx+dz*dz); }

struct Builder {
    WalkwayMesh mesh;
    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a); mesh.edges.push_back(b); }
    void cylinder(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int seg=8){
        const Vec3 axis=b-a; const float len=length(axis); if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len); const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        const Vec3 u=normalize(cross(n,ref)),v=normalize(cross(n,u));
        seg=std::max(6,seg);
        for(int i=0;i<seg;++i){
            const float a0=2.0f*PI*(float)i/(float)seg,a1=2.0f*PI*(float)(i+1)/(float)seg;
            const Vec3 r0=u*std::cos(a0)*radius+v*std::sin(a0)*radius;
            const Vec3 r1=u*std::cos(a1)*radius+v*std::sin(a1)*radius;
            tri(dst,a+r0,b+r0,b+r1); tri(dst,a+r0,b+r1,a+r1);
            line(a+r0,a+r1); line(b+r0,b+r1);
        }
    }
};

Vec3 closest_on_rotated_rect(Vec3 hit,const PlacedResource& item,const ResourceMesh& mesh){
    const Vec3 local=rotate_y(hit-item.position,-item.rotation_degrees);
    const float hx=mesh.footprint_x*0.5f,hz=mesh.footprint_z*0.5f;
    Vec3 q{std::clamp(local.x,-hx,hx),0,std::clamp(local.z,-hz,hz)};
    const float dx=hx-std::abs(q.x),dz=hz-std::abs(q.z);
    if(dx<dz) q.x=(q.x<0?-hx:hx); else q.z=(q.z<0?-hz:hz);
    return item.position+rotate_y(q,item.rotation_degrees);
}

Vec3 resource_normal(Vec3 point,const PlacedResource& item,const ResourceMesh& mesh){
    const Vec3 local=rotate_y(point-item.position,-item.rotation_degrees);
    const float hx=mesh.footprint_x*0.5f,hz=mesh.footprint_z*0.5f;
    Vec3 n{};
    if(std::abs(std::abs(local.x)-hx)<std::abs(std::abs(local.z)-hz)) n={local.x<0?-1.0f:1.0f,0,0};
    else n={0,0,local.z<0?-1.0f:1.0f};
    return flat_normal(rotate_y(n,item.rotation_degrees));
}

bool door_clear(const CorridorDoor& door,const std::vector<WalkwayConnection>& existing){
    for(const auto& c:existing){
        for(const WalkwayEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=WalkwayEndpointKind::Habitat) continue;
            if(dist_xz(ep->door.position,door.position)<2.35f) return false;
        }
    }
    return true;
}

bool connection_slot_clear(WalkwayEndpointKind kind,std::size_t index,Vec3 point,
                           const std::vector<WalkwayConnection>& existing,float spacing=2.10f){
    for(const auto& c:existing){
        for(const WalkwayEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=kind || ep->index!=index) continue;
            if(dist_xz(ep->position,point)<spacing) return false;
        }
    }
    return true;
}

std::optional<WalkwayEndpoint> from_corridor_endpoint(const CorridorEndpoint& ep,
                                                      const std::vector<WalkwayConnection>& existing){
    if(ep.kind!=CorridorEndpointKind::Corridor) return std::nullopt;
    if(!connection_slot_clear(WalkwayEndpointKind::Corridor,ep.index,ep.position,existing)) return std::nullopt;

    WalkwayEndpoint out;
    out.kind=WalkwayEndpointKind::Corridor;
    out.index=ep.index;
    out.position=ep.position;
    out.position.y=0.0f;
    out.normal=flat_normal(ep.tangent);

    // A still-open corridor end is already an entrance. A mid-corridor snap
    // needs a new exterior access opening. Store the surface-to-adapter span in
    // the door fields so the walkway mesh can draw that generated entrance.
    if(ep.surface_matched_junction){
        constexpr float CORRIDOR_HALF_WIDTH=1.08f;
        out.generated_access=true;
        out.door.generated=true;
        out.door.normal=out.normal;
        out.door.wall_position=ep.host_position+out.normal*CORRIDOR_HALF_WIDTH;
        out.door.wall_position.y=0.0f;
        out.door.position=out.position;
        out.door.position.y=0.0f;
    }
    return out;
}

std::optional<WalkwayEndpoint> resolve_habitat_endpoint(const WalkwayBuildAnchor& a,Vec3 other,
                                                        const CorridorNetwork& corridors,
                                                        const std::vector<PlacedHabitat>& habitats,
                                                        const HabitatModelLibrary& library,
                                                        const std::vector<WalkwayConnection>& existing){
    if(a.index>=habitats.size()) return std::nullopt;
    CorridorBuildAnchor ca{CorridorEndpointKind::Habitat,a.index,a.hit};
    auto ep=corridors.preview_anchor(ca,other,habitats,library);
    if(!ep) return std::nullopt;

    if(ep->kind==CorridorEndpointKind::Corridor)
        return from_corridor_endpoint(*ep,existing);
    if(ep->kind!=CorridorEndpointKind::Habitat || !door_clear(ep->door,existing)) return std::nullopt;

    WalkwayEndpoint out;
    out.kind=WalkwayEndpointKind::Habitat; out.index=a.index; out.door=ep->door;
    out.position=ep->door.position; out.position.y=0.0f; out.normal=flat_normal(ep->door.normal);
    return out;
}

std::optional<WalkwayEndpoint> resolve_corridor_endpoint(const WalkwayBuildAnchor& a,Vec3 other,
                                                         const CorridorNetwork& corridors,
                                                         const std::vector<PlacedHabitat>& habitats,
                                                         const HabitatModelLibrary& library,
                                                         const std::vector<WalkwayConnection>& existing){
    if(a.index>=corridors.connections().size()) return std::nullopt;
    const CorridorBuildAnchor ca=corridors.corridor_anchor(a.index,a.hit);
    auto ep=corridors.preview_anchor(ca,other,habitats,library);
    if(!ep) return std::nullopt;
    return from_corridor_endpoint(*ep,existing);
}

WalkwayEndpoint resolve_open_endpoint(const WalkwayBuildAnchor& a,Vec3 other){
    WalkwayEndpoint out;
    out.kind=WalkwayEndpointKind::Open;
    out.index=0;
    out.position=a.hit;
    out.position.y=0.0f;
    out.normal=flat_normal(other-out.position);
    return out;
}

std::optional<WalkwayEndpoint> resolve_resource_endpoint(const WalkwayBuildAnchor& a,
                                                         const std::vector<PlacedResource>& resources,
                                                         const ResourceModelLibrary& library){
    if(a.index>=resources.size()) return std::nullopt;
    const auto& item=resources[a.index]; const auto& mesh=library.mesh(item.kind);
    WalkwayEndpoint out; out.kind=WalkwayEndpointKind::Resource; out.index=a.index;
    if(item.kind==ResourceKind::LandingPad){
        // The landing pad has one deliberate crew approach: the outer end of
        // its built-in approach ramp.  End the walkway flush with that deck so
        // it never grows a fake entrance, vestibule, or stair block.
        const Vec3 local{0.0f,0.50f,-10.15f};
        out.position=item.position+rotate_y(local,item.rotation_degrees);
        out.normal=flat_normal(rotate_y(Vec3{0,0,-1},item.rotation_degrees));
        out.landing_pad=true;
        return out;
    }
    out.position=closest_on_rotated_rect(a.hit,item,mesh);
    out.normal=resource_normal(out.position,item,mesh);
    out.generated_access=true;
    return out;
}

float point_segment_distance(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z,den=dx*dx+dz*dz;
    if(den<0.0001f) return dist_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return dist_xz(p,{a.x+dx*t,0,a.z+dz*t});
}

struct PathFrame {
    Vec3 position{};
    Vec3 tangent{1.0f,0.0f,0.0f};
};

void insert_shared_path_vertex(WalkwayConnection& host,Vec3 point){
    // Mirror corridor branching: commit the exact branch point into the host
    // centerline before adding the new connection.  All participating walkway
    // strips then share one geometric vertex instead of merely meeting near it.
    if(host.path.size()<2) return;
    for(const Vec3& p:host.path){
        if(dist_xz(p,point)<0.015f && std::abs(p.y-point.y)<0.05f) return;
    }

    float best=std::numeric_limits<float>::max();
    std::size_t best_i=0;
    Vec3 snapped=point;
    for(std::size_t i=0;i+1<host.path.size();++i){
        const Vec3 a=host.path[i],b=host.path[i+1];
        const float dx=b.x-a.x,dz=b.z-a.z;
        const float den=dx*dx+dz*dz;
        const float t=den>0.0001f?std::clamp(((point.x-a.x)*dx+(point.z-a.z)*dz)/den,0.0f,1.0f):0.0f;
        const Vec3 q{a.x+dx*t,0.0f,a.z+dz*t};
        const float d=dist_xz(q,point);
        if(d<best){ best=d; best_i=i; snapped=q; }
    }
    host.path.insert(host.path.begin()+(std::ptrdiff_t)best_i+1,snapped);
}

PathFrame closest_path_frame(const std::vector<Vec3>& path,Vec3 p){
    if(path.size()<2) return {p,{1,0,0}};
    PathFrame best{path.front(),flat_normal(path[1]-path[0])};
    float best_d=std::numeric_limits<float>::max();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 a=path[i],b=path[i+1];
        const float dx=b.x-a.x,dz=b.z-a.z,den=dx*dx+dz*dz;
        const float t=den>0.0001f?std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f):0.0f;
        const Vec3 q{a.x+dx*t,0.0f,a.z+dz*t};
        const float d=dist_xz(p,q);
        if(d>=best_d) continue;
        best_d=d;
        best.position=q;
        Vec3 tangent=flat_normal(b-a);
        // Blend adjacent centerline segments at sampled vertices so branches
        // inherit the already-smoothed host direction instead of a faceted
        // single-segment heading.
        if(t<0.18f && i>0) tangent=flat_normal(flat_normal(path[i]-path[i-1])+tangent);
        if(t>0.82f && i+2<path.size()) tangent=flat_normal(tangent+flat_normal(path[i+2]-path[i+1]));
        best.tangent=tangent;
    }
    return best;
}

std::optional<WalkwayEndpoint> resolve_walkway_endpoint(const WalkwayBuildAnchor& a,Vec3 other,
                                                        const std::vector<WalkwayConnection>& existing){
    if(a.index>=existing.size()) return std::nullopt;
    const auto& host=existing[a.index];
    if(host.path.size()<2) return std::nullopt;

    WalkwayEndpoint out;
    out.kind=WalkwayEndpointKind::Walkway;
    out.index=a.index;

    // Match corridor branching semantics.  Near an existing walkway end, snap
    // to the terminal centerline and continue along its exact outward tangent.
    // This produces a true seamless extension instead of a branch that starts
    // beside the end and then bends back onto it.
    constexpr float END_SNAP_RADIUS=1.35f;
    if(dist_xz(a.hit,host.path.front())<=END_SNAP_RADIUS){
        out.position=host.path.front();
        out.position.y=0.0f;
        out.normal=flat_normal(host.path.front()-host.path[1]);
        return out;
    }
    if(dist_xz(a.hit,host.path.back())<=END_SNAP_RADIUS){
        out.position=host.path.back();
        out.position.y=0.0f;
        out.normal=flat_normal(host.path.back()-host.path[host.path.size()-2]);
        return out;
    }

    // Mid-walkway branches use the same model as a mid-corridor branch: first
    // resolve the exact host centerline point, then leave perpendicular to the
    // local host tangent on the side of the requested destination.  The route
    // solver is still free to bend after this first section, so the result is a
    // clean T/Y transition without the old overlapping tangent-on-tangent run.
    const PathFrame frame=closest_path_frame(host.path,a.hit);
    out.position=frame.position;
    out.position.y=0.0f;
    Vec3 side=side_of(frame.tangent);
    const Vec3 toward=flat_normal(other-out.position);
    if(dot(side,toward)<0.0f) side=side*-1.0f;
    out.normal=flat_normal(side);
    return out;
}

bool segment_clear(Vec3 a,Vec3 b,const WalkwayEndpoint& ea,const WalkwayEndpoint& eb,
                   const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                   const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    for(std::size_t i=0;i<habitats.size();++i){
        if((ea.kind==WalkwayEndpointKind::Habitat && ea.index==i)||(eb.kind==WalkwayEndpointKind::Habitat && eb.index==i)) continue;
        const auto& m=hlib.mesh(habitats[i]);
        const float r=0.46f*std::sqrt(m.footprint_x*m.footprint_x+m.footprint_z*m.footprint_z)+CLEARANCE;
        if(point_segment_distance(habitats[i].position,a,b)<r) return false;
    }
    for(std::size_t i=0;i<resources.size();++i){
        if((ea.kind==WalkwayEndpointKind::Resource && ea.index==i)||(eb.kind==WalkwayEndpointKind::Resource && eb.index==i)) continue;
        const auto& m=rlib.mesh(resources[i].kind);
        const float r=0.46f*std::sqrt(m.footprint_x*m.footprint_x+m.footprint_z*m.footprint_z)+CLEARANCE;
        if(point_segment_distance(resources[i].position,a,b)<r) return false;
    }
    return true;
}

bool path_clear(const std::vector<Vec3>& path,const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    if(path.size()<2) return false;
    for(std::size_t i=0;i+1<path.size();++i)
        if(!segment_clear(path[i],path[i+1],a,b,habitats,hlib,resources,rlib)) return false;
    return true;
}

float path_len(const std::vector<Vec3>& path){ float n=0; for(std::size_t i=0;i+1<path.size();++i) n+=dist_xz(path[i],path[i+1]); return n; }

std::vector<Vec3> smooth_candidate(const WalkwayEndpoint& a,const WalkwayEndpoint& b,float lateral){
    // Use the same ordered cubic-route model as corridors instead of repeatedly
    // subdividing a control polygon.  A fixed sample order means blueprint deck
    // sections keep the same local frame while the mouse moves; they no longer
    // swap/flip when a nearby control point crosses the centerline.
    const float d=std::max(1.0f,dist_xz(a.position,b.position));
    const Vec3 chord=flat_normal(b.position-a.position);
    const Vec3 side=side_of(chord);
    const Vec3 outward_a=flat_normal(a.normal);
    const Vec3 outward_b=flat_normal(b.normal);
    const float lead=std::clamp(d*0.28f,0.80f,4.20f);

    const Vec3 p0=a.position;
    const Vec3 p1=a.position+outward_a*lead;
    const Vec3 p2=b.position+outward_b*lead;
    const Vec3 p3=b.position;

    constexpr int SAMPLES=16;
    std::vector<Vec3> path;
    path.reserve(SAMPLES+1);
    for(int i=0;i<=SAMPLES;++i){
        const float t=(float)i/(float)SAMPLES;
        const float u=1.0f-t;
        Vec3 p=p0*(u*u*u)+p1*(3.0f*u*u*t)+p2*(3.0f*u*t*t)+p3*(t*t*t);
        // A squared sine produces obstacle-avoidance bowing without changing
        // either endpoint tangent, so the first/last walkway section remains
        // aligned exactly like a corridor splice.
        const float wave=std::sin(PI*t);
        p=p+side*(lateral*wave*wave);
        p.y=a.position.y+(b.position.y-a.position.y)*t;
        path.push_back(p);
    }

    path.front()=a.position;
    path.back()=b.position;
    if(path.size()>=4){
        const float terminal=std::clamp(d*0.05f,0.05f,0.18f);
        path[1]=a.position+outward_a*terminal;
        path[path.size()-2]=b.position+outward_b*terminal;
    }
    return path;
}

struct RouteResult {
    std::vector<Vec3> path{};
    std::vector<WalkwayCorridorCrossing> corridor_crossings{};
    std::vector<WalkwaySupplyBridge> supply_bridges{};
};

RouteResult route(const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                  const CorridorNetwork& corridors,const UtilityNetwork& utilities,
                  const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                  const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib,
                  const std::vector<WalkwayConnection>& existing);


struct SegmentClosest {
    Vec3 a{};
    Vec3 b{};
    float distance{0.0f};
};

SegmentClosest closest_segments_xz(Vec3 p0,Vec3 p1,Vec3 q0,Vec3 q1){
    const float ux=p1.x-p0.x,uz=p1.z-p0.z;
    const float vx=q1.x-q0.x,vz=q1.z-q0.z;
    const float wx=p0.x-q0.x,wz=p0.z-q0.z;
    const float aa=ux*ux+uz*uz,bb=ux*vx+uz*vz,cc=vx*vx+vz*vz;
    const float dd=ux*wx+uz*wz,ee=vx*wx+vz*wz;
    const float denom=aa*cc-bb*bb;
    float sc=0.0f,tc=0.0f;
    if(aa<1.0e-6f && cc<1.0e-6f){
        return {p0,q0,dist_xz(p0,q0)};
    }else if(aa<1.0e-6f){
        tc=std::clamp(ee/cc,0.0f,1.0f);
    }else if(cc<1.0e-6f){
        sc=std::clamp(-dd/aa,0.0f,1.0f);
    }else{
        if(std::abs(denom)>1.0e-6f) sc=std::clamp((bb*ee-cc*dd)/denom,0.0f,1.0f);
        tc=(bb*sc+ee)/cc;
        if(tc<0.0f){ tc=0.0f; sc=std::clamp(-dd/aa,0.0f,1.0f); }
        else if(tc>1.0f){ tc=1.0f; sc=std::clamp((bb-dd)/aa,0.0f,1.0f); }
    }
    Vec3 pa{p0.x+ux*sc,0.0f,p0.z+uz*sc};
    Vec3 pb{q0.x+vx*tc,0.0f,q0.z+vz*tc};
    return {pa,pb,dist_xz(pa,pb)};
}

bool segment_intersection_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& out,float* out_t=nullptr){
    const float rx=b.x-a.x,rz=b.z-a.z;
    const float sx=d.x-c.x,sz=d.z-c.z;
    const float denom=rx*sz-rz*sx;
    if(std::abs(denom)<1.0e-5f) return false;
    const float qx=c.x-a.x,qz=c.z-a.z;
    const float t=(qx*sz-qz*sx)/denom;
    const float u=(qx*rz-qz*rx)/denom;
    if(t<-1.0e-4f || t>1.0001f || u<-1.0e-4f || u>1.0001f) return false;
    out={a.x+rx*t,0.0f,a.z+rz*t};
    if(out_t) *out_t=std::clamp(t,0.0f,1.0f);
    return true;
}

float path_distance_at(const std::vector<Vec3>& path,std::size_t segment,float t){
    float s=0.0f;
    for(std::size_t i=0;i<segment && i+1<path.size();++i) s+=dist_xz(path[i],path[i+1]);
    if(segment+1<path.size()) s+=dist_xz(path[segment],path[segment+1])*std::clamp(t,0.0f,1.0f);
    return s;
}

bool close_to_corridor_attachment(const WalkwayEndpoint& endpoint,std::size_t corridor_index,Vec3 point){
    return endpoint.kind==WalkwayEndpointKind::Corridor && endpoint.index==corridor_index &&
           dist_xz(endpoint.position,point)<3.25f;
}

bool corridor_access_spacing_clear(std::size_t corridor_index,Vec3 entrance_a,Vec3 entrance_b,
                                   const std::vector<WalkwayConnection>& existing){
    constexpr float spacing=2.35f;
    for(const auto& connection:existing){
        for(const WalkwayEndpoint* endpoint:{&connection.endpoint_a,&connection.endpoint_b}){
            if(endpoint->kind!=WalkwayEndpointKind::Corridor || endpoint->index!=corridor_index) continue;
            if(dist_xz(endpoint->position,entrance_a)<spacing || dist_xz(endpoint->position,entrance_b)<spacing) return false;
        }
        for(const auto& crossing:connection.corridor_crossings){
            if(crossing.corridor_index!=corridor_index) continue;
            if(dist_xz(crossing.entrance_a,entrance_a)<spacing || dist_xz(crossing.entrance_a,entrance_b)<spacing ||
               dist_xz(crossing.entrance_b,entrance_a)<spacing || dist_xz(crossing.entrance_b,entrance_b)<spacing) return false;
        }
    }
    return true;
}

void append_unique_corridor_crossing(std::vector<WalkwayCorridorCrossing>& out,WalkwayCorridorCrossing crossing){
    for(const auto& existing:out)
        if(existing.corridor_index==crossing.corridor_index && dist_xz(existing.center,crossing.center)<0.70f) return;
    out.push_back(std::move(crossing));
}

void append_unique_supply_bridge(std::vector<WalkwaySupplyBridge>& out,WalkwaySupplyBridge bridge){
    for(const auto& existing:out)
        if(existing.utility_index==bridge.utility_index && dist_xz(existing.center,bridge.center)<0.70f) return;
    out.push_back(std::move(bridge));
}

bool analyze_network_crossings(const std::vector<Vec3>& path,const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                               const CorridorNetwork& corridors,const UtilityNetwork& utilities,
                               const std::vector<WalkwayConnection>& existing,
                               std::vector<WalkwayCorridorCrossing>& corridor_crossings,
                               std::vector<WalkwaySupplyBridge>& supply_bridges){
    if(path.size()<2) return false;
    const float total_length=path_len(path);

    // A corridor is solid/pressurized geometry. A walkway may only pass through
    // it as a deliberate transverse crossing with a generated access opening on
    // both sides. Grazing, parallel and overlapping routes are rejected.
    const auto& corridor_links=corridors.connections();
    for(std::size_t ci=0;ci<corridor_links.size();++ci){
        const auto& corridor=corridor_links[ci];
        if(corridor.path.size()<2) continue;
        std::vector<Vec3> accepted_centers;
        for(std::size_t i=0;i+1<path.size();++i){
            const Vec3 wt=flat_normal(path[i+1]-path[i]);
            for(std::size_t j=0;j+1<corridor.path.size();++j){
                const Vec3 ct=flat_normal(corridor.path[j+1]-corridor.path[j]);
                const float sin_angle=std::abs(wt.x*ct.z-wt.z*ct.x);
                Vec3 center{}; float t=0.0f;
                if(!segment_intersection_xz(path[i],path[i+1],corridor.path[j],corridor.path[j+1],center,&t)) continue;
                if(close_to_corridor_attachment(a,ci,center) || close_to_corridor_attachment(b,ci,center)) continue;
                if(sin_angle<CORRIDOR_CROSS_MIN_SIN) return false;
                if(dist_xz(center,corridor.path.front())<1.75f || dist_xz(center,corridor.path.back())<1.75f) return false;
                const float s=path_distance_at(path,i,t);
                if(s<1.80f || total_length-s<1.80f) return false;
                const Vec3 corridor_side=side_of(ct);
                const float normal_component=std::max(0.12f,std::abs(dot(wt,corridor_side)));
                const float shell_run=(CORRIDOR_HALF_WIDTH+0.10f)/normal_component;
                WalkwayCorridorCrossing crossing;
                crossing.corridor_index=ci;
                crossing.center=center;
                crossing.walkway_tangent=wt;
                crossing.corridor_tangent=ct;
                crossing.entrance_a=center-wt*shell_run;
                crossing.entrance_b=center+wt*shell_run;
                if(!corridor_access_spacing_clear(ci,crossing.entrance_a,crossing.entrance_b,existing)) return false;
                append_unique_corridor_crossing(corridor_crossings,crossing);
                accepted_centers.push_back(center);
            }
        }

        for(std::size_t i=0;i+1<path.size();++i){
            for(std::size_t j=0;j+1<corridor.path.size();++j){
                const auto closest=closest_segments_xz(path[i],path[i+1],corridor.path[j],corridor.path[j+1]);
                if(closest.distance>=CORRIDOR_OCCUPANCY) continue;
                if(close_to_corridor_attachment(a,ci,closest.a) || close_to_corridor_attachment(b,ci,closest.a)) continue;
                bool crossing_zone=false;
                for(const Vec3 center:accepted_centers){
                    if(dist_xz(closest.a,center)<CORRIDOR_CROSS_KEEP_CLEAR && dist_xz(closest.b,center)<CORRIDOR_CROSS_KEEP_CLEAR){
                        crossing_zone=true; break;
                    }
                }
                if(!crossing_zone) return false;
            }
        }
    }

    // Supply channels are low enough to pass under a walkway, but the walkway
    // must become a visible bridge at every true crossing. Running parallel on
    // top of a channel is not a bridge and remains invalid.
    const auto& supply_links=utilities.connections();
    for(std::size_t ui=0;ui<supply_links.size();++ui){
        const auto& supply=supply_links[ui];
        if(supply.path.size()<2) continue;
        std::vector<Vec3> accepted_centers;
        for(std::size_t i=0;i+1<path.size();++i){
            const Vec3 wt=flat_normal(path[i+1]-path[i]);
            for(std::size_t j=0;j+1<supply.path.size();++j){
                Vec3 sd=supply.path[j+1]-supply.path[j];
                const float horizontal=std::sqrt(sd.x*sd.x+sd.z*sd.z);
                if(horizontal<0.10f || std::abs(sd.y)>0.18f) continue;
                const Vec3 st=flat_normal(sd);
                const float sin_angle=std::abs(wt.x*st.z-wt.z*st.x);
                Vec3 center{}; float t=0.0f;
                if(!segment_intersection_xz(path[i],path[i+1],supply.path[j],supply.path[j+1],center,&t)) continue;
                if(sin_angle<SUPPLY_CROSS_MIN_SIN) return false;
                const float s=path_distance_at(path,i,t);
                if(s<1.35f || total_length-s<1.35f) return false;
                WalkwaySupplyBridge bridge;
                bridge.utility_index=ui;
                bridge.center=center;
                bridge.tangent=wt;
                bridge.path_distance=s;
                append_unique_supply_bridge(supply_bridges,bridge);
                accepted_centers.push_back(center);
            }
        }
        for(std::size_t i=0;i+1<path.size();++i){
            for(std::size_t j=0;j+1<supply.path.size();++j){
                Vec3 sd=supply.path[j+1]-supply.path[j];
                if(std::sqrt(sd.x*sd.x+sd.z*sd.z)<0.10f || std::abs(sd.y)>0.18f) continue;
                const auto closest=closest_segments_xz(path[i],path[i+1],supply.path[j],supply.path[j+1]);
                if(closest.distance>=SUPPLY_OCCUPANCY) continue;
                bool bridge_zone=false;
                for(const Vec3 center:accepted_centers){
                    if(dist_xz(closest.a,center)<2.20f && dist_xz(closest.b,center)<2.20f){ bridge_zone=true; break; }
                }
                if(!bridge_zone) return false;
            }
        }
    }

    for(const auto& bridge:supply_bridges)
        for(const auto& crossing:corridor_crossings)
            if(dist_xz(bridge.center,crossing.center)<BRIDGE_HALF_SPAN+1.0f) return false;
    return true;
}

std::vector<Vec3> densify_path(const std::vector<Vec3>& path,float max_step=0.38f){
    if(path.size()<2) return path;
    std::vector<Vec3> out;
    out.push_back(path.front());
    for(std::size_t i=0;i+1<path.size();++i){
        const float d=dist_xz(path[i],path[i+1]);
        const int pieces=std::max(1,(int)std::ceil(d/max_step));
        for(int k=1;k<=pieces;++k){
            const float t=(float)k/(float)pieces;
            out.push_back(path[i]+(path[i+1]-path[i])*t);
        }
    }
    return out;
}

std::vector<Vec3> apply_supply_bridge_elevation(std::vector<Vec3> path,const std::vector<WalkwaySupplyBridge>& bridges){
    if(path.size()<2 || bridges.empty()) return path;
    path=densify_path(path);
    std::vector<float> cumulative(path.size(),0.0f);
    for(std::size_t i=1;i<path.size();++i) cumulative[i]=cumulative[i-1]+dist_xz(path[i-1],path[i]);
    for(std::size_t i=0;i<path.size();++i){
        float rise=0.0f;
        for(const auto& bridge:bridges){
            const float d=std::abs(cumulative[i]-bridge.path_distance);
            if(d>=BRIDGE_HALF_SPAN) continue;
            const float phase=d/BRIDGE_HALF_SPAN;
            const float bump=0.5f*(1.0f+std::cos(PI*phase));
            rise=std::max(rise,BRIDGE_RISE*bump);
        }
        path[i].y=rise;
    }
    path.front().y=0.0f;
    path.back().y=0.0f;
    return path;
}

RouteResult route(const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                  const CorridorNetwork& corridors,const UtilityNetwork& utilities,
                  const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                  const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib,
                  const std::vector<WalkwayConnection>& existing){
    const float d=dist_xz(a.position,b.position);
    const float offset=std::clamp(d*0.28f,3.0f,9.0f);
    const std::array<float,5> offsets={0.0f,offset,-offset,offset*1.55f,-offset*1.55f};

    // Corridor placement is deterministic: it keeps the first valid route
    // family instead of choosing between nearly equal mirrored candidates on
    // every mouse update.  Do the same for walkways so a blueprint cannot hop
    // from +lateral to -lateral and appear to flip its individual sections.
    for(float lateral:offsets){
        auto candidate=smooth_candidate(a,b,lateral);
        if(!path_clear(candidate,a,b,habitats,hlib,resources,rlib)) continue;
        std::vector<WalkwayCorridorCrossing> crossings;
        std::vector<WalkwaySupplyBridge> bridges;
        if(!analyze_network_crossings(candidate,a,b,corridors,utilities,existing,crossings,bridges)) continue;
        RouteResult chosen;
        chosen.path=apply_supply_bridge_elevation(std::move(candidate),bridges);
        chosen.corridor_crossings=std::move(crossings);
        chosen.supply_bridges=std::move(bridges);
        return chosen;
    }
    return {};
}

void append_unique_direction(std::vector<Vec3>& arms,Vec3 d);
std::vector<Vec3> path_arms_at(const std::vector<Vec3>& path,Vec3 point);
float merge_radius_for_arms(const std::vector<Vec3>& arms);

bool walkway_path_clear(const std::vector<Vec3>& path,const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                        const std::vector<WalkwayConnection>& existing){
    constexpr float MIN_CENTERLINE_DISTANCE=HALF_WIDTH*2.0f+0.16f;
    auto merge_allow_radius=[&](const WalkwayEndpoint& endpoint,std::size_t host_index){
        if(endpoint.kind!=WalkwayEndpointKind::Walkway || endpoint.index!=host_index || host_index>=existing.size()) return 0.0f;
        std::vector<Vec3> arms;
        for(const Vec3& arm:path_arms_at(existing[host_index].path,endpoint.position)) append_unique_direction(arms,arm);
        for(const Vec3& arm:path_arms_at(path,endpoint.position)) append_unique_direction(arms,arm);
        return merge_radius_for_arms(arms)+0.30f;
    };
    for(std::size_t host_index=0;host_index<existing.size();++host_index){
        const auto& host=existing[host_index];
        if(host.path.size()<2) continue;
        const float allow_a=merge_allow_radius(a,host_index);
        const float allow_b=merge_allow_radius(b,host_index);
        for(std::size_t i=0;i+1<path.size();++i){
            for(std::size_t j=0;j+1<host.path.size();++j){
                const auto c=closest_segments_xz(path[i],path[i+1],host.path[j],host.path[j+1]);
                if(c.distance>=MIN_CENTERLINE_DISTANCE) continue;
                bool allowed=false;
                if(allow_a>0.0f && dist_xz(c.a,a.position)<=allow_a && dist_xz(c.b,a.position)<=allow_a) allowed=true;
                if(allow_b>0.0f && dist_xz(c.a,b.position)<=allow_b && dist_xz(c.b,b.position)<=allow_b) allowed=true;
                if(!allowed) return false;
            }
        }
    }
    return true;
}

void append_unique_direction(std::vector<Vec3>& arms,Vec3 d){
    d=flat_normal(d);
    constexpr float SAME_DIRECTION=0.985f;
    for(const Vec3& e:arms) if(dot(flat_normal(e),d)>SAME_DIRECTION) return;
    arms.push_back(d);
}

std::vector<Vec3> path_arms_at(const std::vector<Vec3>& path,Vec3 point){
    std::vector<Vec3> arms;
    if(path.size()<2) return arms;
    std::size_t best_i=0;
    float best_t=0.0f,best_d=std::numeric_limits<float>::max();
    Vec3 best=path.front();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 p0=path[i],p1=path[i+1];
        const float dx=p1.x-p0.x,dz=p1.z-p0.z,den=dx*dx+dz*dz;
        const float t=den>1.0e-6f?std::clamp(((point.x-p0.x)*dx+(point.z-p0.z)*dz)/den,0.0f,1.0f):0.0f;
        const Vec3 q{p0.x+dx*t,0.0f,p0.z+dz*t};
        const float d=dist_xz(point,q);
        if(d<best_d){ best_d=d; best_i=i; best_t=t; best=q; }
    }
    (void)best_t;
    if(dist_xz(best,path.front())>0.12f){
        Vec3 toward=path[best_i]-best;
        if(dist_xz(best,path[best_i])<0.08f && best_i>0) toward=path[best_i-1]-best;
        append_unique_direction(arms,toward);
    }
    if(dist_xz(best,path.back())>0.12f){
        Vec3 toward=path[best_i+1]-best;
        if(dist_xz(best,path[best_i+1])<0.08f && best_i+2<path.size()) toward=path[best_i+2]-best;
        append_unique_direction(arms,toward);
    }
    return arms;
}

float merge_radius_for_arms(const std::vector<Vec3>& arms){
    if(arms.size()<2) return 1.55f;
    std::vector<float> angles;
    angles.reserve(arms.size());
    for(const Vec3& a:arms){
        float angle=std::atan2(a.z,a.x);
        if(angle<0.0f) angle+=2.0f*PI;
        angles.push_back(angle);
    }
    std::sort(angles.begin(),angles.end());
    float min_gap=2.0f*PI;
    for(std::size_t i=0;i<angles.size();++i){
        const float next=i+1<angles.size()?angles[i+1]:angles.front()+2.0f*PI;
        min_gap=std::min(min_gap,next-angles[i]);
    }
    const float default_alpha=std::atan2(HALF_WIDTH,1.55f);
    const float safe_alpha=std::clamp(min_gap*0.42f,7.0f*PI/180.0f,default_alpha);
    return std::clamp(HALF_WIDTH/std::tan(safe_alpha),1.55f,5.0f);
}

void append_generated_entrance(Builder& b,const CorridorDoor& door){
    if(!door.generated) return;
    const Vec3 n=flat_normal(door.normal),side=side_of(n);
    constexpr float half=1.18f;
    std::vector<Vec3> outer;
    if(!door.wall_profile.empty()){
        outer.reserve(door.wall_profile.size());
        for(const Vec3& wp:door.wall_profile){
            const float lateral=dot(flat(wp-door.wall_position),side);
            outer.push_back({door.position.x+side.x*lateral,wp.y,door.position.z+side.z*lateral});
        }
        if(outer.size()==door.wall_profile.size()&&outer.size()>=3){
            for(std::size_t i=0;i<outer.size();++i){
                const std::size_t j=(i+1)%outer.size();
                Builder::tri(b.mesh.details,door.wall_profile[i],outer[i],outer[j]);
                Builder::tri(b.mesh.details,door.wall_profile[i],outer[j],door.wall_profile[j]);
                b.line(door.wall_profile[i],door.wall_profile[j]); b.line(outer[i],outer[j]);
            }
        }
    }
    const Vec3 base{door.position.x,0.0f,door.position.z};
    b.line(base-side*half,base+side*half);
    b.line(base-side*half,base-side*half+Vec3{0,2.65f,0});
    b.line(base+side*half,base+side*half+Vec3{0,2.65f,0});
    b.line(base-side*half+Vec3{0,2.65f,0},base+side*half+Vec3{0,2.65f,0});
}

void append_resource_access(Builder& b,const WalkwayEndpoint& ep){
    if(ep.kind!=WalkwayEndpointKind::Resource || !ep.generated_access || ep.landing_pad) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n),base=ep.position+Vec3{0,DECK_Y,0};
    constexpr float half=0.92f;
    // Open exterior access gate: clearly a crew access point, intentionally no
    // door leaf / pressure shell because walkways never provide atmosphere.
    b.line(base-side*half,base-side*half+Vec3{0,1.35f,0});
    b.line(base+side*half,base+side*half+Vec3{0,1.35f,0});
    b.line(base-side*half+Vec3{0,1.35f,0},base+side*half+Vec3{0,1.35f,0});
    b.line(base-side*half,base+side*half);
    b.cylinder(b.mesh.details,base-side*half-n*0.10f,base-side*half+n*0.10f,0.055f,6);
    b.cylinder(b.mesh.details,base+side*half-n*0.10f,base+side*half+n*0.10f,0.055f,6);
}

void append_corridor_access(Builder& b,const WalkwayEndpoint& ep){
    if(ep.kind!=WalkwayEndpointKind::Corridor || !ep.generated_access || !ep.door.generated) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n);
    constexpr float half=0.82f;
    const Vec3 inner=ep.door.wall_position+Vec3{0,DECK_Y,0};
    const Vec3 outer=ep.position+Vec3{0,DECK_Y,0};
    const Vec3 s=side*half;
    Builder::tri(b.mesh.deck,inner-s,outer-s,outer+s);
    Builder::tri(b.mesh.deck,inner-s,outer+s,inner+s);
    b.line(inner-s,outer-s); b.line(inner+s,outer+s);
    b.line(inner-s,inner+s); b.line(outer-s,outer+s);

    // Generated non-pressurized access frame at the corridor shell. The actual
    // corridor remains the pressurized interior; this frame marks the new door
    // location that the walkway is allowed to use.
    constexpr float h=2.35f;
    b.line(inner-s,inner-s+Vec3{0,h,0});
    b.line(inner+s,inner+s+Vec3{0,h,0});
    b.line(inner-s+Vec3{0,h,0},inner+s+Vec3{0,h,0});
}

void append_corridor_crossing_access(Builder& b,const WalkwayCorridorCrossing& crossing){
    // A through-crossing is not allowed to punch through a pressurized
    // corridor invisibly. Draw an exterior access frame on both shell faces so
    // the walkway clearly enters and exits through deliberate openings.
    const Vec3 wt=flat_normal(crossing.walkway_tangent);
    constexpr float half=0.84f;
    constexpr float height=2.35f;
    for(int side_index=0;side_index<2;++side_index){
        const Vec3 position=side_index==0?crossing.entrance_a:crossing.entrance_b;
        const Vec3 outward=side_index==0?wt*-1.0f:wt;
        const Vec3 side=side_of(outward);
        const Vec3 base=position+Vec3{0,DECK_Y,0};
        const Vec3 left=base-side*half;
        const Vec3 right=base+side*half;
        b.line(left,right);
        b.line(left,left+Vec3{0,height,0});
        b.line(right,right+Vec3{0,height,0});
        b.line(left+Vec3{0,height,0},right+Vec3{0,height,0});
        // Shallow threshold/collar overlaps both the walkway deck and the
        // corridor shell, avoiding a visible crack at the generated opening.
        const Vec3 inner=base-outward*0.16f;
        const Vec3 outer=base+outward*0.16f;
        const Vec3 hs=side*(half*0.94f);
        Builder::tri(b.mesh.deck,inner-hs,outer-hs,outer+hs);
        Builder::tri(b.mesh.deck,inner-hs,outer+hs,inner+hs);
    }
}

void append_supply_bridge_supports(Builder& b,const WalkwaySupplyBridge& bridge){
    const Vec3 tangent=flat_normal(bridge.tangent);
    const Vec3 side=side_of(tangent);
    constexpr float support_offset=1.55f;
    constexpr float lateral=0.54f;
    for(float d:{-support_offset,support_offset}){
        const float phase=std::abs(d)/BRIDGE_HALF_SPAN;
        const float rise=phase<1.0f?BRIDGE_RISE*0.5f*(1.0f+std::cos(PI*phase)):0.0f;
        const float top_y=DECK_Y+rise-0.05f;
        const Vec3 center=bridge.center+tangent*d;
        for(float s:{-lateral,lateral}){
            const Vec3 foot=center+side*s+Vec3{0,0.04f,0};
            const Vec3 top=center+side*s+Vec3{0,top_y,0};
            b.cylinder(b.mesh.details,foot,top,0.055f,6);
        }
        b.line(center-side*lateral+Vec3{0,top_y,0},center+side*lateral+Vec3{0,top_y,0});
    }
}

void append_habitat_steps(Builder& b,const WalkwayEndpoint& ep){
    if(ep.kind!=WalkwayEndpointKind::Habitat) return;
    const float target=std::max(0.24f,ep.door.wall_position.y-0.25f);
    const float rise=target-DECK_Y; if(rise<0.12f) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n);
    const int steps=std::clamp((int)std::ceil(rise/0.16f),2,7);
    const float run=1.18f,step_run=run/(float)steps,step_rise=rise/(float)steps;
    for(int i=0;i<steps;++i){
        const float d=step_run*((float)i+0.5f);
        const float y=target-step_rise*(float)i;
        const Vec3 c=ep.position+n*d+Vec3{0,y,0};
        const Vec3 s=side*(HALF_WIDTH*0.88f),f=n*(step_run*0.50f);
        Builder::tri(b.mesh.deck,c-f-s,c+f-s,c+f+s); Builder::tri(b.mesh.deck,c-f-s,c+f+s,c-f+s);
        b.line(c-f-s,c-f+s); b.line(c+f-s,c+f+s);
    }
}

void append_landing_terminal(Builder& b,const WalkwayEndpoint& ep){
    if(!ep.landing_pad) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n);
    const Vec3 edge=ep.position+Vec3{0,DECK_Y,0};
    const Vec3 outer=edge+n*1.15f;
    const Vec3 s0=side*(HALF_WIDTH*1.55f),s1=side*HALF_WIDTH;
    Builder::tri(b.mesh.deck,outer-s1,edge-s0,edge+s0);
    Builder::tri(b.mesh.deck,outer-s1,edge+s0,outer+s1);
    b.line(edge-s0,edge+s0); // clean, regular pad threshold
    b.line(outer-s1,edge-s0); b.line(outer+s1,edge+s0);
}

struct ClipInterval { float a{0.0f}; float b{1.0f}; };

std::vector<ClipInterval> outside_junctions(Vec3 p0,Vec3 p1,const std::vector<WalkwayJunction>& junctions){
    std::vector<ClipInterval> intervals{{0.0f,1.0f}};
    const float dx=p1.x-p0.x,dz=p1.z-p0.z;
    const float A=dx*dx+dz*dz;
    if(A<1.0e-8f) return {};
    for(const auto& j:junctions){
        const float ox=p0.x-j.position.x,oz=p0.z-j.position.z;
        const float B=2.0f*(ox*dx+oz*dz);
        const float C=ox*ox+oz*oz-j.radius*j.radius;
        const float disc=B*B-4.0f*A*C;
        float inside_lo=2.0f,inside_hi=-1.0f;
        if(disc<0.0f){
            const Vec3 mid=(p0+p1)*0.5f;
            if(dist_xz(mid,j.position)<j.radius){ inside_lo=0.0f; inside_hi=1.0f; }
        }else{
            const float root=std::sqrt(std::max(0.0f,disc));
            float t0=(-B-root)/(2.0f*A),t1=(-B+root)/(2.0f*A);
            if(t0>t1) std::swap(t0,t1);
            inside_lo=std::max(0.0f,t0);
            inside_hi=std::min(1.0f,t1);
            if(inside_hi<=inside_lo){
                const Vec3 mid=(p0+p1)*0.5f;
                if(dist_xz(mid,j.position)<j.radius){ inside_lo=0.0f; inside_hi=1.0f; }
            }
        }
        if(inside_hi<=inside_lo) continue;
        std::vector<ClipInterval> next;
        for(const auto& in:intervals){
            if(inside_hi<=in.a || inside_lo>=in.b){ next.push_back(in); continue; }
            if(inside_lo>in.a+1.0e-4f) next.push_back({in.a,std::min(in.b,inside_lo)});
            if(inside_hi<in.b-1.0e-4f) next.push_back({std::max(in.a,inside_hi),in.b});
        }
        intervals=std::move(next);
        if(intervals.empty()) break;
    }
    return intervals;
}

Vec3 path_vertex_tangent(const std::vector<Vec3>& path,std::size_t i){
    if(path.size()<2) return {1,0,0};
    const Vec3 forward=(i+1<path.size())?flat(path[i+1]-path[i]):flat(path[i]-path[i-1]);
    if(i==0) return flat_normal(forward);
    if(i+1>=path.size()) return flat_normal(forward);

    Vec3 tangent=flat(path[i+1]-path[i-1]);
    if(length(tangent)<0.001f) tangent=forward;
    tangent=flat_normal(tangent);
    // Keep every sampled frame facing in path order.  This is the important
    // invariant used by corridor strip generation and prevents left/right deck
    // edges from exchanging places in the live blueprint.
    if(dot(tangent,flat_normal(forward))<0.0f) tangent=tangent*-1.0f;
    return tangent;
}

void append_walkway_span(Builder& builder,Vec3 p0,Vec3 p1,Vec3 tangent0,Vec3 tangent1){
    p0.y+=DECK_Y; p1.y+=DECK_Y;
    tangent0=flat_normal(tangent0); tangent1=flat_normal(tangent1);
    if(dist_xz(p0,p1)<0.015f) return;
    const Vec3 side0=side_of(tangent0),side1=side_of(tangent1);
    const Vec3 s0=side0*HALF_WIDTH,s1=side1*HALF_WIDTH;
    Builder::tri(builder.mesh.deck,p0-s0,p1-s1,p1+s1);
    Builder::tri(builder.mesh.deck,p0-s0,p1+s1,p0+s0);
    // Open lunar walkway deck: only the two deck-edge seams are rendered.
    // Railings/posts are intentionally omitted.
    builder.line(p0-s0,p1-s1); builder.line(p0+s0,p1+s1);
    // Integrated services run below the open deck rather than along its edge,
    // keeping the silhouette completely railing-free.
    const Vec3 util0=side0*(HALF_WIDTH*0.58f),util1=side1*(HALF_WIDTH*0.58f);
    builder.cylinder(builder.mesh.details,p0+util0+Vec3{0,-0.24f,0},p1+util1+Vec3{0,-0.24f,0},0.10f,8);
    builder.cylinder(builder.mesh.details,p0-util0+Vec3{0,-0.18f,0},p1-util1+Vec3{0,-0.18f,0},0.035f,6);
    builder.cylinder(builder.mesh.details,p0-util0+Vec3{0,-0.30f,0},p1-util1+Vec3{0,-0.30f,0},0.035f,6);
}

void append_junction_patch(Builder& builder,const WalkwayJunction& junction){
    if(!junction.owns_patch || junction.arms.size()<2) return;

    // Intersections are deliberately built as a mitered merge of the incoming
    // walkway strips.  Do not add a circular/round junction plate: each arm is
    // clipped to the same trim distance and the remaining opening is filled by
    // one low-vertex polygon whose corners are the actual walkway edges.
    const Vec3 center=junction.position+Vec3{0,DECK_Y,0};
    std::vector<Vec3> points;
    points.reserve(junction.arms.size()*2+1);
    points.push_back(center);
    for(const Vec3& raw_arm:junction.arms){
        const Vec3 arm=flat_normal(raw_arm);
        const Vec3 side=side_of(arm);
        const Vec3 end=center+arm*junction.radius;
        points.push_back(end-side*HALF_WIDTH);
        points.push_back(end+side*HALF_WIDTH);
    }

    // 2-D monotonic-chain hull.  This gives T/Y/cross/end-splice junctions a
    // clean shared polygon without arcs, discs, duplicate overlapping decks or
    // a visible circular cap at the attachment point.
    std::sort(points.begin(),points.end(),[](const Vec3& a,const Vec3& b){
        if(std::abs(a.x-b.x)>1.0e-5f) return a.x<b.x;
        return a.z<b.z;
    });
    points.erase(std::unique(points.begin(),points.end(),[](const Vec3& a,const Vec3& b){
        return dist_xz(a,b)<0.01f;
    }),points.end());
    if(points.size()<3) return;

    auto cross_xz=[](const Vec3& o,const Vec3& a,const Vec3& b){
        return (a.x-o.x)*(b.z-o.z)-(a.z-o.z)*(b.x-o.x);
    };
    std::vector<Vec3> hull;
    hull.reserve(points.size()*2);
    for(const Vec3& p:points){
        while(hull.size()>=2 && cross_xz(hull[hull.size()-2],hull.back(),p)<=1.0e-5f) hull.pop_back();
        hull.push_back(p);
    }
    const std::size_t lower=hull.size();
    for(std::size_t i=points.size()-1;i-->0;){
        const Vec3& p=points[i];
        while(hull.size()>lower && cross_xz(hull[hull.size()-2],hull.back(),p)<=1.0e-5f) hull.pop_back();
        hull.push_back(p);
    }
    if(!hull.empty()) hull.pop_back();
    if(hull.size()<3) return;

    Vec3 centroid{};
    for(const Vec3& p:hull) centroid=centroid+p;
    centroid=centroid*(1.0f/(float)hull.size());
    centroid.y=DECK_Y;
    for(std::size_t i=0;i<hull.size();++i)
        Builder::tri(builder.mesh.deck,centroid,hull[i],hull[(i+1)%hull.size()]);

    // Intentionally no perimeter ring and no central service cylinder.  The
    // adjoining strips provide the visible edge seams, making the merge read as
    // one continuous bent walkway instead of a separate junction object.
}

WalkwayMesh make_mesh(const WalkwayEndpoint& a,const WalkwayEndpoint& b,const std::vector<Vec3>& path,
                      const std::vector<WalkwayJunction>& junctions={},
                      const std::vector<WalkwayCorridorCrossing>& corridor_crossings={},
                      const std::vector<WalkwaySupplyBridge>& supply_bridges={}){
    Builder builder;
    append_generated_entrance(builder,a.door); append_generated_entrance(builder,b.door);
    append_resource_access(builder,a); append_resource_access(builder,b);
    append_corridor_access(builder,a); append_corridor_access(builder,b);
    for(const auto& crossing:corridor_crossings) append_corridor_crossing_access(builder,crossing);
    for(const auto& bridge:supply_bridges) append_supply_bridge_supports(builder,bridge);
    append_habitat_steps(builder,a); append_habitat_steps(builder,b);
    append_landing_terminal(builder,a); append_landing_terminal(builder,b);

    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 p0=path[i],p1=path[i+1];
        const auto intervals=outside_junctions(p0,p1,junctions);
        const Vec3 t0=path_vertex_tangent(path,i),t1=path_vertex_tangent(path,i+1);
        for(const auto& interval:intervals){
            const Vec3 q0=p0+(p1-p0)*interval.a;
            const Vec3 q1=p0+(p1-p0)*interval.b;
            const Vec3 qa=flat_normal(t0*(1.0f-interval.a)+t1*interval.a);
            const Vec3 qb=flat_normal(t0*(1.0f-interval.b)+t1*interval.b);
            append_walkway_span(builder,q0,q1,qa,qb);
        }
    }
    for(const auto& junction:junctions) append_junction_patch(builder,junction);
    return std::move(builder.mesh);
}

std::optional<WalkwayEndpoint> resolve_anchor(const WalkwayBuildAnchor& anchor,Vec3 other,
                                              const CorridorNetwork& corridors,
                                              const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                                              const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib,
                                              const std::vector<WalkwayConnection>& existing){
    switch(anchor.kind){
        case WalkwayEndpointKind::Habitat:
            return resolve_habitat_endpoint(anchor,other,corridors,habitats,hlib,existing);
        case WalkwayEndpointKind::Resource:
            return resolve_resource_endpoint(anchor,resources,rlib);
        case WalkwayEndpointKind::Corridor:
            return resolve_corridor_endpoint(anchor,other,corridors,habitats,hlib,existing);
        case WalkwayEndpointKind::Walkway:
            return resolve_walkway_endpoint(anchor,other,existing);
        case WalkwayEndpointKind::Open:
            return resolve_open_endpoint(anchor,other);
    }
    return std::nullopt;
}
}

std::optional<WalkwayConnection> WalkwayNetwork::preview_connection(
    const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,const CorridorNetwork& corridors,const UtilityNetwork& utilities,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib) const {
    auto a=resolve_anchor(source,target.hit,corridors,habitats,hlib,resources,rlib,connections_);
    if(!a) return std::nullopt;
    auto b=resolve_anchor(target,a->position,corridors,habitats,hlib,resources,rlib,connections_);
    if(!b) return std::nullopt;

    if(a->kind!=WalkwayEndpointKind::Open && b->kind!=WalkwayEndpointKind::Open &&
       a->kind==b->kind && a->index==b->index) return std::nullopt;
    if(dist_xz(a->position,b->position)<0.90f) return std::nullopt;

    auto routed=route(*a,*b,corridors,utilities,habitats,hlib,resources,rlib,connections_);
    if(routed.path.size()<2 || !walkway_path_clear(routed.path,*a,*b,connections_)) return std::nullopt;

    WalkwayConnection out;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(routed.path);
    out.corridor_crossings=std::move(routed.corridor_crossings);
    out.supply_bridges=std::move(routed.supply_bridges);
    out.pressurized=false;

    // The blueprint already uses the same rounded transition as the committed
    // network. Only the new walkway owns these temporary patches; once placed,
    // rebuild_walkway_merges() moves ownership to the stable host connection.
    auto add_preview_merge=[&](const WalkwayEndpoint& endpoint){
        if(endpoint.kind!=WalkwayEndpointKind::Walkway || endpoint.index>=connections_.size()) return;
        WalkwayJunction junction;
        junction.position=endpoint.position;
        for(const Vec3& arm:path_arms_at(connections_[endpoint.index].path,junction.position)) append_unique_direction(junction.arms,arm);
        for(const Vec3& arm:path_arms_at(out.path,junction.position)) append_unique_direction(junction.arms,arm);
        if(junction.arms.size()<2) return;
        junction.radius=merge_radius_for_arms(junction.arms);
        junction.owns_patch=true;
        for(auto& existing:out.junctions){
            if(dist_xz(existing.position,junction.position)>1.05f) continue;
            for(const Vec3& arm:junction.arms) append_unique_direction(existing.arms,arm);
            existing.position=(existing.position+junction.position)*0.5f;
            existing.radius=merge_radius_for_arms(existing.arms);
            return;
        }
        out.junctions.push_back(std::move(junction));
    };
    add_preview_merge(out.endpoint_a);
    add_preview_merge(out.endpoint_b);

    out.mesh=make_mesh(out.endpoint_a,out.endpoint_b,out.path,out.junctions,out.corridor_crossings,out.supply_bridges);
    return out;
}

std::optional<WalkwayConnection> WalkwayNetwork::preview_invalid_connection(
    const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,const CorridorNetwork& corridors,const UtilityNetwork& utilities,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib) const {
    (void)utilities;
    // Keep any endpoint that can be resolved, but deliberately fall back to an
    // open point for an invalid structure/connection. This creates a complete
    // attempted walkway mesh that can be rendered red without ever becoming
    // buildable.
    auto a=resolve_anchor(source,target.hit,corridors,habitats,hlib,resources,rlib,connections_);
    if(!a) a=resolve_open_endpoint(source,target.hit);
    auto b=resolve_anchor(target,a->position,corridors,habitats,hlib,resources,rlib,connections_);
    if(!b) b=resolve_open_endpoint(target,a->position);
    if(dist_xz(a->position,b->position)<0.20f) return std::nullopt;

    auto path=smooth_candidate(*a,*b,0.0f);
    if(path.size()<2) return std::nullopt;
    WalkwayConnection out;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=make_mesh(out.endpoint_a,out.endpoint_b,out.path);
    out.pressurized=false;
    return out;
}

bool WalkwayNetwork::build_connection(const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,const CorridorNetwork& corridors,const UtilityNetwork& utilities,
                                      const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                                      const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    auto c=preview_connection(source,target,corridors,utilities,habitats,hlib,resources,rlib);
    if(!c) return false;

    // Use the same branch materialisation model as corridors.  A walkway-to-
    // walkway endpoint becomes a real vertex in the host centerline before the
    // branch is inserted.  This makes T/Y branches deterministic, gives every
    // arm the exact same junction coordinate and lets the merge rebuild create
    // a seamless shared deck without overlaps or floating near-matches.
    for(const WalkwayEndpoint* ep:{&c->endpoint_a,&c->endpoint_b}){
        if(ep->kind!=WalkwayEndpointKind::Walkway || ep->index>=connections_.size()) continue;
        insert_shared_path_vertex(connections_[ep->index],ep->position);
    }

    c->junctions.clear();
    c->resource_plan=construction_costs::walkway(path_len(c->path));
    connections_.push_back(std::move(*c));
    rebuild_walkway_merges();
    return true;
}

void WalkwayNetwork::rebuild_walkway_merges(){
    struct MergeSeed {
        Vec3 position{};
        std::size_t samples{0};
        std::vector<std::size_t> participants{};
    };
    auto add_participant=[](std::vector<std::size_t>& v,std::size_t index){
        if(std::find(v.begin(),v.end(),index)==v.end()) v.push_back(index);
    };

    for(auto& c:connections_) c.junctions.clear();
    std::vector<MergeSeed> seeds;
    for(std::size_t branch_index=0;branch_index<connections_.size();++branch_index){
        const auto& c=connections_[branch_index];
        for(const WalkwayEndpoint* endpoint:{&c.endpoint_a,&c.endpoint_b}){
            if(endpoint->kind!=WalkwayEndpointKind::Walkway || endpoint->index>=connections_.size() || endpoint->index==branch_index) continue;
            MergeSeed* seed=nullptr;
            for(auto& candidate:seeds){
                if(dist_xz(candidate.position,endpoint->position)<=1.05f){ seed=&candidate; break; }
            }
            if(!seed){
                seeds.push_back({endpoint->position,1,{}});
                seed=&seeds.back();
            }else{
                seed->position=(seed->position*(float)seed->samples+endpoint->position)*(1.0f/(float)(seed->samples+1));
                ++seed->samples;
            }
            add_participant(seed->participants,branch_index);
            add_participant(seed->participants,endpoint->index);
        }
    }

    for(const auto& seed:seeds){
        std::vector<Vec3> arms;
        for(std::size_t connection_index:seed.participants){
            if(connection_index>=connections_.size()) continue;
            for(const Vec3& arm:path_arms_at(connections_[connection_index].path,seed.position)) append_unique_direction(arms,arm);
        }
        if(arms.size()<2) continue;
        const float radius=merge_radius_for_arms(arms);
        const std::size_t owner=*std::min_element(seed.participants.begin(),seed.participants.end());
        for(std::size_t connection_index:seed.participants){
            if(connection_index>=connections_.size()) continue;
            WalkwayJunction junction;
            junction.position=seed.position;
            junction.arms=arms;
            junction.radius=radius;
            junction.owns_patch=connection_index==owner;
            connections_[connection_index].junctions.push_back(std::move(junction));
        }
    }

    for(auto& connection:connections_)
        connection.mesh=make_mesh(connection.endpoint_a,connection.endpoint_b,connection.path,connection.junctions,connection.corridor_crossings,connection.supply_bridges);
}

void WalkwayNetwork::update_construction(float dt,ResourceManagement& resources,bool allow_negative){
    for(auto& connection:connections_){
        connection.construction.update_gated(dt,[&](std::uint8_t next_step){
            return resources.consume(connection.resource_plan.next_step_cost((std::uint8_t)(next_step-1)),allow_negative);
        });
    }
}

float WalkwayNetwork::connection_length(std::size_t index) const {
    return index<connections_.size()?path_len(connections_[index].path):0.0f;
}

std::size_t WalkwayNetwork::endpoint_connection_count(WalkwayEndpointKind kind,std::size_t index) const {
    std::size_t n=0;
    for(const auto& c:connections_)
        for(const WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b})
            if(e->kind==kind&&e->index==index) ++n;
    return n;
}

void WalkwayNetwork::remove_connection(std::size_t index){
    if(index>=connections_.size()) return;
    connections_.erase(connections_.begin()+(std::ptrdiff_t)index);
    for(auto& c:connections_) for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
        if(e->kind!=WalkwayEndpointKind::Walkway) continue;
        if(e->index==index){ e->kind=WalkwayEndpointKind::Open; e->index=0; }
        else if(e->index>index) --e->index;
    }
    rebuild_walkway_merges();
}

void WalkwayNetwork::remove_habitat(std::size_t index){
    for(auto& c:connections_) for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
        if(e->kind==WalkwayEndpointKind::Habitat&&e->index==index){
            e->kind=WalkwayEndpointKind::Open;
            e->index=0;
            e->normal=e->door.normal;
            e->door={};
            e->generated_access=false;
        }else if(e->kind==WalkwayEndpointKind::Habitat&&e->index>index) --e->index;
    }
    rebuild_walkway_merges();
}

void WalkwayNetwork::remove_resource(std::size_t index){
    for(auto& c:connections_) for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
        if(e->kind==WalkwayEndpointKind::Resource&&e->index==index){
            e->kind=WalkwayEndpointKind::Open;
            e->index=0;
            e->generated_access=false;
            e->landing_pad=false;
        }else if(e->kind==WalkwayEndpointKind::Resource&&e->index>index) --e->index;
    }
    rebuild_walkway_merges();
}

void WalkwayNetwork::remove_corridor(std::size_t index){
    for(auto& c:connections_){
        for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
            if(e->kind==WalkwayEndpointKind::Corridor&&e->index==index){
                e->kind=WalkwayEndpointKind::Open;
                e->index=0;
                e->door={};
                e->generated_access=false;
            }else if(e->kind==WalkwayEndpointKind::Corridor&&e->index>index) --e->index;
        }
        c.corridor_crossings.erase(std::remove_if(c.corridor_crossings.begin(),c.corridor_crossings.end(),
            [&](const WalkwayCorridorCrossing& crossing){ return crossing.corridor_index==index; }),c.corridor_crossings.end());
        for(auto& crossing:c.corridor_crossings) if(crossing.corridor_index>index) --crossing.corridor_index;
    }
    rebuild_walkway_merges();
}

void WalkwayNetwork::remove_utility(std::size_t index){
    for(auto& c:connections_){
        c.supply_bridges.erase(std::remove_if(c.supply_bridges.begin(),c.supply_bridges.end(),
            [&](const WalkwaySupplyBridge& bridge){ return bridge.utility_index==index; }),c.supply_bridges.end());
        for(auto& bridge:c.supply_bridges) if(bridge.utility_index>index) --bridge.utility_index;
        for(auto& p:c.path) p.y=0.0f;
        c.path=apply_supply_bridge_elevation(std::move(c.path),c.supply_bridges);
    }
    rebuild_walkway_merges();
}

const char* walkway_endpoint_name(WalkwayEndpointKind kind){
    switch(kind){
        case WalkwayEndpointKind::Habitat:return "HABITAT";
        case WalkwayEndpointKind::Resource:return "STRUCTURE";
        case WalkwayEndpointKind::Corridor:return "CORRIDOR";
        case WalkwayEndpointKind::Walkway:return "WALKWAY";
        case WalkwayEndpointKind::Open:return "OPEN END";
    }
    return "END";
}

} // namespace game
