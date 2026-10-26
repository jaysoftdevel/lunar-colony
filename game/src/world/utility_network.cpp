#include "game/world/utility_network.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;

namespace {
constexpr float PI=3.14159265358979323846f;

struct Builder {
    UtilityMesh mesh;
    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a); mesh.edges.push_back(b); }

    void box(std::vector<Vec3>& dst,Vec3 c,Vec3 s,bool outline=true){
        const Vec3 h=s*0.5f;
        Vec3 p[8]={
            {c.x-h.x,c.y-h.y,c.z-h.z},{c.x+h.x,c.y-h.y,c.z-h.z},
            {c.x+h.x,c.y+h.y,c.z-h.z},{c.x-h.x,c.y+h.y,c.z-h.z},
            {c.x-h.x,c.y-h.y,c.z+h.z},{c.x+h.x,c.y-h.y,c.z+h.z},
            {c.x+h.x,c.y+h.y,c.z+h.z},{c.x-h.x,c.y+h.y,c.z+h.z}
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void oriented_box(std::vector<Vec3>& dst,Vec3 c,Vec3 axis_x,Vec3 axis_y,Vec3 axis_z,Vec3 size,bool outline=true){
        axis_x=normalize(axis_x); axis_y=normalize(axis_y); axis_z=normalize(axis_z);
        const Vec3 hx=axis_x*(size.x*0.5f), hy=axis_y*(size.y*0.5f), hz=axis_z*(size.z*0.5f);
        Vec3 p[8]={
            c-hx-hy-hz,c+hx-hy-hz,c+hx+hy-hz,c-hx+hy-hz,
            c-hx-hy+hz,c+hx-hy+hz,c+hx+hy+hz,c-hx+hy+hz
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void cylinder_between(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int segments=10,bool outline=true){
        const Vec3 axis=b-a;
        const float len=length(axis);
        if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len);
        const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        const Vec3 u=normalize(cross(n,ref));
        const Vec3 v=normalize(cross(n,u));
        const int seg=std::max(6,segments);
        for(int i=0;i<seg;++i){
            const float a0=2.0f*PI*(float)i/(float)seg;
            const float a1=2.0f*PI*(float)(i+1)/(float)seg;
            const Vec3 r0=u*std::cos(a0)*radius+v*std::sin(a0)*radius;
            const Vec3 r1=u*std::cos(a1)*radius+v*std::sin(a1)*radius;
            const Vec3 p0=a+r0,p1=a+r1,q0=b+r0,q1=b+r1;
            tri(dst,p0,q0,q1); tri(dst,p0,q1,p1);
            if(outline){ line(p0,p1); line(q0,q1); if(i%(std::max(1,seg/4))==0) line(p0,q0); }
        }
    }
};

float channel_y(UtilityKind){ return 0.28f; }

bool resource_supports(UtilityKind,ResourceKind){
    // A supply channel carries the common base services. Every external
    // resource structure can therefore join the colony supply backbone.
    return true;
}

Vec3 structure_center(const UtilityBuildAnchor& a,
                      const std::vector<PlacedHabitat>& habitats,
                      const std::vector<PlacedResource>& resources){
    if(a.kind==UtilityEndpointKind::Habitat && a.index<habitats.size()) return habitats[a.index].position;
    if(a.kind==UtilityEndpointKind::Resource && a.index<resources.size()) return resources[a.index].position;
    return a.hit;
}

bool ray_triangle(Vec3 origin,Vec3 direction,Vec3 a,Vec3 b,Vec3 c,float& t){
    constexpr float EPS=1.0e-6f;
    const Vec3 e1=b-a;
    const Vec3 e2=c-a;
    const Vec3 p=cross(direction,e2);
    const float det=dot(e1,p);
    if(std::abs(det)<EPS) return false;
    const float inv_det=1.0f/det;
    const Vec3 s=origin-a;
    const float u=dot(s,p)*inv_det;
    if(u<-EPS || u>1.0f+EPS) return false;
    const Vec3 q=cross(s,e1);
    const float v=dot(direction,q)*inv_det;
    if(v<-EPS || u+v>1.0f+EPS) return false;
    t=dot(e2,q)*inv_det;
    return t>EPS;
}

void nearest_mesh_hit(const std::vector<Vec3>& triangles,Vec3 origin,Vec3 direction,float& best_t,bool& found){
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        float t=0.0f;
        if(ray_triangle(origin,direction,triangles[i],triangles[i+1],triangles[i+2],t) && t<best_t){
            best_t=t;
            found=true;
        }
    }
}

Vec3 closest_point_triangle(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    const Vec3 ab=b-a,ac=c-a,ap=p-a;
    const float d1=dot(ab,ap),d2=dot(ac,ap);
    if(d1<=0.0f && d2<=0.0f) return a;
    const Vec3 bp=p-b;
    const float d3=dot(ab,bp),d4=dot(ac,bp);
    if(d3>=0.0f && d4<=d3) return b;
    const float vc=d1*d4-d3*d2;
    if(vc<=0.0f && d1>=0.0f && d3<=0.0f){ const float v=d1/(d1-d3); return a+ab*v; }
    const Vec3 cp=p-c;
    const float d5=dot(ab,cp),d6=dot(ac,cp);
    if(d6>=0.0f && d5<=d6) return c;
    const float vb=d5*d2-d1*d6;
    if(vb<=0.0f && d2>=0.0f && d6<=0.0f){ const float w=d2/(d2-d6); return a+ac*w; }
    const float va=d3*d6-d5*d4;
    if(va<=0.0f && (d4-d3)>=0.0f && (d5-d6)>=0.0f){
        const float w=(d4-d3)/((d4-d3)+(d5-d6));
        return b+(c-b)*w;
    }
    const float denom=1.0f/(va+vb+vc);
    const float v=vb*denom,w=vc*denom;
    return a+ab*v+ac*w;
}

void nearest_surface_point(const std::vector<Vec3>& triangles,Vec3 desired,float& best_d2,Vec3& best,bool& found){
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const Vec3 q=closest_point_triangle(desired,triangles[i],triangles[i+1],triangles[i+2]);
        const Vec3 d=q-desired;
        const float d2=dot(d,d);
        if(d2<best_d2){ best_d2=d2; best=q; found=true; }
    }
}

std::optional<UtilityEndpoint> surface_endpoint(const UtilityBuildAnchor& anchor,Vec3 toward,UtilityKind utility,
                                                const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                                                const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    Vec3 center{};
    float rotation=0.0f;
    float radius=2.0f;
    const HabitatMesh* habitat_mesh=nullptr;
    const ResourceMesh* resource_mesh=nullptr;
    if(anchor.kind==UtilityEndpointKind::Habitat){
        if(anchor.index>=habitats.size()) return std::nullopt;
        const auto& placed=habitats[anchor.index];
        habitat_mesh=&habitat_library.mesh(placed);
        center=placed.position;
        rotation=placed.rotation_degrees;
        radius=std::max(habitat_mesh->footprint_x,habitat_mesh->footprint_z)*0.75f+3.0f;
    }else{
        if(anchor.index>=resources.size()) return std::nullopt;
        const auto& placed=resources[anchor.index];
        resource_mesh=&resource_library.mesh(placed.kind);
        center=placed.position;
        rotation=placed.rotation_degrees;
        radius=std::max(resource_mesh->footprint_x,resource_mesh->footprint_z)*0.75f+3.0f;
    }

    // The cursor position controls the actual attachment location. This keeps
    // structure snapping convenient while still allowing fine adjustment along
    // the target hull before the second click. When the pointer is exactly at
    // the center, fall back to the opposite structure so a stable port exists.
    Vec3 outward=anchor.hit-center;
    outward.y=0.0f;
    if(length(outward)<0.08f){ outward=toward-center; outward.y=0.0f; }
    if(length(outward)<0.001f) outward={1,0,0};
    outward=normalize(outward);

    const Vec3 local_outward=normalize(rotate_y(outward,-rotation));
    const float local_y=channel_y(utility)-center.y;
    const Vec3 local_origin=local_outward*radius+Vec3{0,local_y,0};
    const Vec3 local_direction=local_outward*-1.0f;

    float best_t=std::numeric_limits<float>::max();
    bool found=false;
    if(habitat_mesh){
        // Pressure shell, glazing and foundation are real exterior surfaces.
        // Details are a fallback for industrial structures whose low utility
        // interface is itself an external equipment block.
        nearest_mesh_hit(habitat_mesh->shell,local_origin,local_direction,best_t,found);
        nearest_mesh_hit(habitat_mesh->glazing,local_origin,local_direction,best_t,found);
        nearest_mesh_hit(habitat_mesh->foundation,local_origin,local_direction,best_t,found);
        if(!found) nearest_mesh_hit(habitat_mesh->details,local_origin,local_direction,best_t,found);
    }else if(resource_mesh){
        nearest_mesh_hit(resource_mesh->body,local_origin,local_direction,best_t,found);
        nearest_mesh_hit(resource_mesh->foundation,local_origin,local_direction,best_t,found);
        if(!found) nearest_mesh_hit(resource_mesh->details,local_origin,local_direction,best_t,found);
    }
    Vec3 local_hit{};
    if(found){
        local_hit=local_origin+local_direction*best_t;
    }else{
        // Some resource models have open frames/legs at cable height. In that
        // case snap to the nearest real exterior triangle to the cursor probe
        // instead of falling back to a footprint rectangle (which created gaps).
        Vec3 desired=rotate_y(anchor.hit-center,-rotation);
        desired.y=local_y;
        float best_d2=std::numeric_limits<float>::max();
        if(habitat_mesh){
            nearest_surface_point(habitat_mesh->shell,desired,best_d2,local_hit,found);
            nearest_surface_point(habitat_mesh->glazing,desired,best_d2,local_hit,found);
            nearest_surface_point(habitat_mesh->foundation,desired,best_d2,local_hit,found);
            nearest_surface_point(habitat_mesh->details,desired,best_d2,local_hit,found);
        }else if(resource_mesh){
            nearest_surface_point(resource_mesh->body,desired,best_d2,local_hit,found);
            nearest_surface_point(resource_mesh->foundation,desired,best_d2,local_hit,found);
            nearest_surface_point(resource_mesh->details,desired,best_d2,local_hit,found);
        }
        if(!found) return std::nullopt;
    }

    UtilityEndpoint out;
    out.kind=anchor.kind;
    out.index=anchor.index;
    out.normal=outward;
    out.position=center+rotate_y(local_hit,rotation);
    // Keep route and port geometry on the exact triangle hit, including its Y.
    return out;
}

bool point_in_footprint(Vec3 p,Vec3 center,float rotation,float sx,float sz,float padding){
    const Vec3 local=rotate_y(p-center,-rotation);
    return std::abs(local.x)<sx*0.5f+padding && std::abs(local.z)<sz*0.5f+padding;
}

bool segment_hits_structure(Vec3 a,Vec3 b,UtilityEndpointKind skip_kind_a,std::size_t skip_a,
                            UtilityEndpointKind skip_kind_b,std::size_t skip_b,
                            const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                            const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    const float len=distance_xz(a,b);
    const int steps=std::max(2,(int)std::ceil(len/0.35f));
    for(int i=1;i<steps;++i){
        const float t=(float)i/(float)steps;
        const Vec3 p=a+(b-a)*t;
        for(std::size_t h=0;h<habitats.size();++h){
            if((skip_kind_a==UtilityEndpointKind::Habitat && h==skip_a) || (skip_kind_b==UtilityEndpointKind::Habitat && h==skip_b)) continue;
            const auto& placed=habitats[h];
            const auto& mesh=habitat_library.mesh(placed);
            if(point_in_footprint(p,placed.position,placed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,0.28f)) return true;
        }
        for(std::size_t r=0;r<resources.size();++r){
            if((skip_kind_a==UtilityEndpointKind::Resource && r==skip_a) || (skip_kind_b==UtilityEndpointKind::Resource && r==skip_b)) continue;
            const auto& placed=resources[r];
            const auto& mesh=resource_library.mesh(placed.kind);
            if(point_in_footprint(p,placed.position,placed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,0.28f)) return true;
        }
    }
    return false;
}

float cross2(Vec3 a,Vec3 b,Vec3 c){
    return (b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x);
}

bool on_segment_xz(Vec3 a,Vec3 b,Vec3 p){
    constexpr float EPS=1.0e-4f;
    return std::abs(cross2(a,b,p))<EPS &&
           p.x>=std::min(a.x,b.x)-EPS && p.x<=std::max(a.x,b.x)+EPS &&
           p.z>=std::min(a.z,b.z)-EPS && p.z<=std::max(a.z,b.z)+EPS;
}

bool segments_intersect_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    const float ab_c=cross2(a,b,c),ab_d=cross2(a,b,d);
    const float cd_a=cross2(c,d,a),cd_b=cross2(c,d,b);
    if(((ab_c>0.0f && ab_d<0.0f)||(ab_c<0.0f && ab_d>0.0f)) &&
       ((cd_a>0.0f && cd_b<0.0f)||(cd_a<0.0f && cd_b>0.0f))) return true;
    return on_segment_xz(a,b,c)||on_segment_xz(a,b,d)||on_segment_xz(c,d,a)||on_segment_xz(c,d,b);
}

float segment_distance_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    if(segments_intersect_xz(a,b,c,d)) return 0.0f;
    return std::min({point_segment_distance_xz(a,c,d),point_segment_distance_xz(b,c,d),
                     point_segment_distance_xz(c,a,b),point_segment_distance_xz(d,a,b)});
}

std::vector<Vec3> compact_path(std::vector<Vec3> path){
    std::vector<Vec3> out;
    out.reserve(path.size());
    for(const Vec3 p:path){
        if(out.empty() || distance_xz(out.back(),p)>0.02f || std::abs(out.back().y-p.y)>0.02f) out.push_back(p);
    }
    return out;
}

bool path_clear(const std::vector<Vec3>& path,const UtilityEndpoint& a,const UtilityEndpoint& b,
                const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    for(std::size_t i=0;i+1<path.size();++i){
        if(segment_hits_structure(path[i],path[i+1],a.kind,a.index,b.kind,b.index,habitats,habitat_library,resources,resource_library)) return false;
    }
    return true;
}

float projected_overlap_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    Vec3 axis=b-a; axis.y=0.0f;
    const float len=length(axis);
    if(len<0.0001f) return 0.0f;
    axis=axis*(1.0f/len);
    const float c0=dot(Vec3{c.x-a.x,0,c.z-a.z},axis);
    const float d0=dot(Vec3{d.x-a.x,0,d.z-a.z},axis);
    return std::max(0.0f,std::min(len,std::max(c0,d0))-std::max(0.0f,std::min(c0,d0)));
}

bool power_segments_overlap(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    Vec3 ab=b-a,cd=d-c; ab.y=0.0f; cd.y=0.0f;
    const float lab=length(ab),lcd=length(cd);
    if(lab<0.08f || lcd<0.08f) return false;
    const Vec3 na=ab*(1.0f/lab),nc=cd*(1.0f/lcd);
    // True crossings are valid and become junctions. Only parallel/collinear
    // runs that would physically occupy the same cable-tray width are rejected.
    if(std::abs(dot(na,nc))<0.965f) return false;
    if(projected_overlap_xz(a,b,c,d)<0.12f) return false;
    return segment_distance_xz(a,b,c,d)<0.52f;
}

bool power_path_compatible(const std::vector<Vec3>& path,const std::vector<UtilityConnection>& existing){
    for(std::size_t i=0;i+1<path.size();++i){
        for(const auto& cable:existing){
            
            for(std::size_t j=0;j+1<cable.path.size();++j){
                if(power_segments_overlap(path[i],path[i+1],cable.path[j],cable.path[j+1])) return false;
            }
        }
    }
    return true;
}

bool segment_intersection_point_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& out){
    // Power junctions are created only on horizontal tray runs; vertical mesh
    // drops/risers are endpoint adapters, not network intersections.
    if(std::abs(a.y-b.y)>0.06f || std::abs(c.y-d.y)>0.06f) return false;
    const float rx=b.x-a.x,rz=b.z-a.z;
    const float sx=d.x-c.x,sz=d.z-c.z;
    const float denom=rx*sz-rz*sx;
    if(std::abs(denom)<1.0e-5f) return false;
    const float qx=c.x-a.x,qz=c.z-a.z;
    const float t=(qx*sz-qz*sx)/denom;
    const float u=(qx*rz-qz*rx)/denom;
    if(t<-1.0e-4f || t>1.0001f || u<-1.0e-4f || u>1.0001f) return false;
    out={a.x+rx*t,(a.y+b.y+c.y+d.y)*0.25f,a.z+rz*t};
    return true;
}

void append_unique_point(std::vector<Vec3>& points,Vec3 p){
    for(const Vec3 q:points) if(distance_xz(p,q)<0.08f && std::abs(p.y-q.y)<0.08f) return;
    points.push_back(p);
}

std::vector<Vec3> power_intersections(const std::vector<Vec3>& path,const std::vector<UtilityConnection>& existing){
    std::vector<Vec3> points;
    for(std::size_t i=0;i+1<path.size();++i){
        for(const auto& cable:existing){
            
            for(std::size_t j=0;j+1<cable.path.size();++j){
                Vec3 p{};
                if(segment_intersection_point_xz(path[i],path[i+1],cable.path[j],cable.path[j+1],p)) append_unique_point(points,p);
            }
        }
    }
    return points;
}

std::vector<Vec3> power_intersections(const std::vector<Vec3>& a,const std::vector<Vec3>& b){
    std::vector<Vec3> points;
    for(std::size_t i=0;i+1<a.size();++i){
        for(std::size_t j=0;j+1<b.size();++j){
            Vec3 p{};
            if(segment_intersection_point_xz(a[i],a[i+1],b[j],b[j+1],p)) append_unique_point(points,p);
        }
    }
    return points;
}

float path_length(const std::vector<Vec3>& path){
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i) total+=distance_xz(path[i],path[i+1]);
    return total;
}

bool route_valid(UtilityKind /*kind*/,const std::vector<Vec3>& path,const UtilityEndpoint& a,const UtilityEndpoint& b,
                 const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                 const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                 const std::vector<UtilityConnection>& existing){
    if(path.size()<2) return false;
    if(!path_clear(path,a,b,habitats,habitat_library,resources,resource_library)) return false;
    return power_path_compatible(path,existing);
}

std::vector<Vec3> make_route(UtilityKind kind,const UtilityEndpoint& a,const UtilityEndpoint& b,
                             const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                             const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                             const std::vector<UtilityConnection>& existing){
    // Always leave a structure along the selected surface normal. This prevents
    // the first cable segment from cutting back through the source/target mesh.
    constexpr float LEAD=0.72f;
    const float y=channel_y(kind);
    const Vec3 start_drop{a.position.x,y,a.position.z};
    const Vec3 end_drop{b.position.x,y,b.position.z};
    Vec3 start_lead=start_drop+a.normal*LEAD;
    Vec3 end_lead=end_drop+b.normal*LEAD;

    std::vector<std::vector<Vec3>> candidates;
    candidates.push_back(compact_path({a.position,start_drop,start_lead,end_lead,end_drop,b.position}));
    candidates.push_back(compact_path({a.position,start_drop,start_lead,{end_lead.x,y,start_lead.z},end_lead,end_drop,b.position}));
    candidates.push_back(compact_path({a.position,start_drop,start_lead,{start_lead.x,y,end_lead.z},end_lead,end_drop,b.position}));

    // Parallel/coincident cable runs are still avoided, but true crossings are
    // valid and become shared junctions. Search deterministic offset lanes only
    // when an obvious route would physically overlap an existing tray.
    for(int step=1;step<=36;++step){
        const float offset=(float)step*0.9f;
        for(float lane_z:{std::min(start_lead.z,end_lead.z)-offset,std::max(start_lead.z,end_lead.z)+offset}){
            candidates.push_back(compact_path({a.position,start_drop,start_lead,{start_lead.x,y,lane_z},{end_lead.x,y,lane_z},end_lead,end_drop,b.position}));
        }
        for(float lane_x:{std::min(start_lead.x,end_lead.x)-offset,std::max(start_lead.x,end_lead.x)+offset}){
            candidates.push_back(compact_path({a.position,start_drop,start_lead,{lane_x,y,start_lead.z},{lane_x,y,end_lead.z},end_lead,end_drop,b.position}));
        }
    }

    std::vector<Vec3> best;
    float best_length=std::numeric_limits<float>::max();
    for(const auto& candidate:candidates){
        if(!route_valid(kind,candidate,a,b,habitats,habitat_library,resources,resource_library,existing)) continue;
        const float len=path_length(candidate);
        if(len<best_length){ best_length=len; best=candidate; }
    }
    return best;
}

void add_power_segment(Builder& b,Vec3 a,Vec3 c){
    const Vec3 d=c-a;
    const float len=length(d);
    if(len<0.001f) return;
    const Vec3 axis=d*(1.0f/len);
    Vec3 ref=std::abs(axis.y)<0.94f?Vec3{0,1,0}:Vec3{1,0,0};
    Vec3 side=normalize(cross(axis,ref));
    if(length(side)<0.001f) side={0,0,1};
    const Vec3 up=normalize(cross(side,axis));
    const Vec3 mid=(a+c)*0.5f;
    b.oriented_box(b.mesh.shell,mid-up*0.05f,axis,up,side,{len,0.16f,0.52f},false);
    for(float s:{-0.17f,0.17f}) b.cylinder_between(b.mesh.details,a+side*s+up*0.05f,c+side*s+up*0.05f,0.045f,8,true);
    const int supports=std::max(1,(int)std::floor(len/1.4f));
    for(int i=0;i<=supports;++i){
        const float t=(float)i/(float)std::max(1,supports);
        const Vec3 p=a+d*t;
        b.line(p-side*0.27f-up*0.09f,p+side*0.27f-up*0.09f);
    }
}

void add_water_segment(Builder& b,Vec3 a,Vec3 c){
    b.cylinder_between(b.mesh.shell,a,c,0.17f,10,true);
    Vec3 d=c-a; d.y=0;
    const float len=length(d);
    if(len<0.001f) return;
    const Vec3 n=d*(1.0f/len);
    const Vec3 side{-n.z,0,n.x};
    const int supports=std::max(1,(int)std::floor(len/1.6f));
    for(int i=0;i<=supports;++i){
        const float t=(float)i/(float)std::max(1,supports);
        const Vec3 p=a+(c-a)*t;
        b.line(p-side*0.26f+Vec3{0,-0.22f,0},p+side*0.26f+Vec3{0,-0.22f,0});
        b.line(p-side*0.18f+Vec3{0,-0.22f,0},p-side*0.18f+Vec3{0,-0.03f,0});
        b.line(p+side*0.18f+Vec3{0,-0.22f,0},p+side*0.18f+Vec3{0,-0.03f,0});
    }
}

void add_power_port(Builder& b,const UtilityEndpoint& ep){
    // Straddle the actual mesh surface so the junction box and the first cable
    // segment meet the structure without a visible air gap.
    const Vec3 n=normalize(Vec3{ep.normal.x,0.0f,ep.normal.z});
    const Vec3 side{-n.z,0,n.x};
    const Vec3 c=ep.position+n*0.08f+Vec3{0,0.08f,0};
    b.oriented_box(b.mesh.details,c,n,{0,1,0},side,{0.38f,0.46f,0.46f},true);
}

void add_water_port(Builder& b,const UtilityEndpoint& ep){
    const Vec3 inside=ep.position-ep.normal*0.20f;
    const Vec3 outside=ep.position+ep.normal*0.16f;
    b.cylinder_between(b.mesh.details,inside,outside,0.26f,12,true);
    b.cylinder_between(b.mesh.details,ep.position-ep.normal*0.03f,ep.position+ep.normal*0.03f,0.33f,12,true);
}

void add_power_junction(Builder& b,Vec3 p){
    // Compact conductive bus enclosure shared by every crossing cable. It sits
    // directly over the tray center so the intersecting cable meshes visibly
    // terminate into one common electrical junction rather than crossing loose.
    p.y=channel_y(UtilityKind::SupplyChannel);
    b.box(b.mesh.details,p+Vec3{0,0.08f,0},{0.72f,0.30f,0.72f},true);
    b.cylinder_between(b.mesh.details,p+Vec3{0,-0.03f,0},p+Vec3{0,0.34f,0},0.10f,8,true);
}

void add_open_supply_cap(Builder& b,const UtilityEndpoint& ep){
    const Vec3 n=normalize(Vec3{ep.normal.x,0.0f,ep.normal.z});
    b.oriented_box(b.mesh.details,ep.position+n*0.04f+Vec3{0,0.07f,0},n,{0,1,0},{-n.z,0,n.x},{0.34f,0.38f,0.34f},true);
    b.cylinder_between(b.mesh.details,ep.position-n*0.06f,ep.position+n*0.10f,0.25f,12,true);
}

UtilityMesh make_mesh(UtilityKind /*kind*/,const UtilityEndpoint& a,const UtilityEndpoint& b,const std::vector<Vec3>& path,const std::vector<Vec3>& junctions={}){
    Builder builder;
    // Attached endpoints use the combined structure port. Endpoints left behind
    // by structure deletion remain as capped, disconnected service stubs.
    if(a.kind==UtilityEndpointKind::Open) add_open_supply_cap(builder,a); else { add_power_port(builder,a); add_water_port(builder,a); }
    if(b.kind==UtilityEndpointKind::Open) add_open_supply_cap(builder,b); else { add_power_port(builder,b); add_water_port(builder,b); }

    for(std::size_t i=0;i+1<path.size();++i){
        // Shared service carrier: cable tray plus water line in the same run.
        add_power_segment(builder,path[i],path[i+1]);
        add_water_segment(builder,path[i],path[i+1]);
    }
    // Mark bends with a compact service joint so orthogonal routes read as one
    // continuous channel instead of two unrelated line segments.
    for(std::size_t i=1;i+1<path.size();++i){
        builder.box(builder.mesh.details,path[i]+Vec3{0,0.05f,0},{0.66f,0.34f,0.66f},true);
        builder.cylinder_between(builder.mesh.details,path[i]+Vec3{0,0.02f,0},path[i]+Vec3{0,0.30f,0},0.23f,12,true);
    }
    for(const Vec3 p:junctions) add_power_junction(builder,p);
    return builder.mesh;
}

bool same_anchor(const UtilityBuildAnchor& a,const UtilityBuildAnchor& b){ return a.kind==b.kind && a.index==b.index; }

} // namespace

bool UtilityNetwork::anchor_compatible(UtilityKind kind,const UtilityBuildAnchor& anchor,
                                       const std::vector<PlacedHabitat>& habitats,
                                       const std::vector<PlacedResource>& resources) const {
    if(anchor.kind==UtilityEndpointKind::Habitat) return anchor.index<habitats.size();
    if(anchor.index>=resources.size()) return false;
    return resource_supports(kind,resources[anchor.index].kind);
}

std::optional<UtilityEndpoint> UtilityNetwork::preview_anchor(
    UtilityKind kind,const UtilityBuildAnchor& anchor,Vec3 toward,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const {
    if(!anchor_compatible(kind,anchor,habitats,resources)) return std::nullopt;
    return surface_endpoint(anchor,toward,kind,habitats,habitat_library,resources,resource_library);
}

UtilityMesh UtilityNetwork::preview_endpoint_mesh(const UtilityEndpoint& endpoint) const {
    Builder builder;
    add_power_port(builder,endpoint);
    add_water_port(builder,endpoint);
    return builder.mesh;
}

std::optional<UtilityConnection> UtilityNetwork::preview_connection(
    UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const {
    if(same_anchor(a,b)) return std::nullopt;
    if(!anchor_compatible(kind,a,habitats,resources) || !anchor_compatible(kind,b,habitats,resources)) return std::nullopt;

    const Vec3 center_a=structure_center(a,habitats,resources);
    const Vec3 center_b=structure_center(b,habitats,resources);
    const auto ea=surface_endpoint(a,center_b,kind,habitats,habitat_library,resources,resource_library);
    const auto eb=surface_endpoint(b,center_a,kind,habitats,habitat_library,resources,resource_library);
    if(!ea || !eb) return std::nullopt;
    const std::vector<Vec3> path=make_route(kind,*ea,*eb,habitats,habitat_library,resources,resource_library,connections_);
    if(path.size()<2) return std::nullopt;

    UtilityConnection out;
    out.kind=kind;
    out.endpoint_a=*ea;
    out.endpoint_b=*eb;
    out.path=path;
    out.junctions=power_intersections(path,connections_);
    out.mesh=make_mesh(kind,*ea,*eb,path,out.junctions);
    return out;
}

bool UtilityNetwork::build_connection(
    UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    auto preview=preview_connection(kind,a,b,habitats,habitat_library,resources,resource_library);
    if(!preview) return false;
    for(const auto& existing:connections_){
        const bool same=(existing.endpoint_a.kind==preview->endpoint_a.kind && existing.endpoint_a.index==preview->endpoint_a.index &&
                         existing.endpoint_b.kind==preview->endpoint_b.kind && existing.endpoint_b.index==preview->endpoint_b.index) ||
                        (existing.endpoint_a.kind==preview->endpoint_b.kind && existing.endpoint_a.index==preview->endpoint_b.index &&
                         existing.endpoint_b.kind==preview->endpoint_a.kind && existing.endpoint_b.index==preview->endpoint_a.index);
        if(same) return false;
    }
    connections_.push_back(std::move(*preview));
    rebuild_supply_junctions();
    return true;
}

void UtilityNetwork::update_construction(float dt){
    for(auto& connection:connections_) connection.construction.update(dt);
}

float UtilityNetwork::connection_length(std::size_t index) const {
    if(index>=connections_.size()) return 0.0f;
    float total=0.0f;
    const auto& path=connections_[index].path;
    for(std::size_t i=0;i+1<path.size();++i) total+=distance_xz(path[i],path[i+1]);
    return total;
}

std::size_t UtilityNetwork::endpoint_connection_count(UtilityEndpointKind kind,std::size_t index) const {
    std::size_t count=0;
    for(const auto& c:connections_){
        if((c.endpoint_a.kind==kind && c.endpoint_a.index==index) || (c.endpoint_b.kind==kind && c.endpoint_b.index==index)) ++count;
    }
    return count;
}


void UtilityNetwork::remove_connection(std::size_t index){
    if(index>=connections_.size()) return;
    connections_.erase(connections_.begin()+(std::ptrdiff_t)index);
    rebuild_supply_junctions();
}

void UtilityNetwork::remove_habitat(std::size_t habitat_index){
    for(auto& connection:connections_){
        for(UtilityEndpoint* endpoint:{&connection.endpoint_a,&connection.endpoint_b}){
            if(endpoint->kind!=UtilityEndpointKind::Habitat) continue;
            if(endpoint->index==habitat_index){
                endpoint->kind=UtilityEndpointKind::Open;
                endpoint->index=0;
            }else if(endpoint->index>habitat_index){
                --endpoint->index;
            }
        }
    }
    rebuild_supply_junctions();
}

void UtilityNetwork::remove_resource(std::size_t resource_index){
    for(auto& connection:connections_){
        for(UtilityEndpoint* endpoint:{&connection.endpoint_a,&connection.endpoint_b}){
            if(endpoint->kind!=UtilityEndpointKind::Resource) continue;
            if(endpoint->index==resource_index){
                endpoint->kind=UtilityEndpointKind::Open;
                endpoint->index=0;
            }else if(endpoint->index>resource_index){
                --endpoint->index;
            }
        }
    }
    rebuild_supply_junctions();
}

void UtilityNetwork::rebuild_supply_junctions(){
    for(auto& c:connections_) c.junctions.clear();
    for(std::size_t i=0;i<connections_.size();++i){
        for(std::size_t j=i+1;j<connections_.size();++j){
            const auto points=power_intersections(connections_[i].path,connections_[j].path);
            for(const Vec3 p:points){
                append_unique_point(connections_[i].junctions,p);
                append_unique_point(connections_[j].junctions,p);
            }
        }
    }
    for(auto& c:connections_) c.mesh=make_mesh(c.kind,c.endpoint_a,c.endpoint_b,c.path,c.junctions);
}

const char* utility_kind_name(UtilityKind){ return "SUPPLY CHANNEL"; }
const char* utility_endpoint_name(UtilityEndpointKind kind){
    switch(kind){
        case UtilityEndpointKind::Habitat: return "HABITAT";
        case UtilityEndpointKind::Resource: return "RESOURCE";
        case UtilityEndpointKind::Open: return "OPEN END";
        default: return "ENDPOINT";
    }
}

} // namespace game
