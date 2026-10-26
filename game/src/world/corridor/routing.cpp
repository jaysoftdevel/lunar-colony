#include "game/world/corridor/detail.hpp"
#include <array>
#include <vector>

namespace game::corridor_detail {
bool segment_clear(Vec3 a,Vec3 b,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                   std::size_t ignore_a=NONE_INDEX,std::size_t ignore_b=NONE_INDEX){
    for(std::size_t i=0;i<habitats.size();++i){
        if(i==ignore_a || i==ignore_b) continue;
        const auto& hm=library.mesh(habitats[i].kind);
        const float radius=0.5f*std::sqrt(hm.footprint_x*hm.footprint_x+hm.footprint_z*hm.footprint_z)+CORRIDOR_HALF_WIDTH+ROUTE_CLEARANCE;
        if(point_segment_distance_xz(habitats[i].position,a,b)<radius) return false;
    }
    return true;
}

bool path_clear(const std::vector<Vec3>& path,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                std::size_t ignore_a=NONE_INDEX,std::size_t ignore_b=NONE_INDEX){
    for(std::size_t i=0;i+1<path.size();++i){
        if(!segment_clear(path[i],path[i+1],habitats,library,ignore_a,ignore_b)) return false;
    }
    return true;
}

float path_length(const std::vector<Vec3>& path){
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i) total+=distance_xz(path[i],path[i+1]);
    return total;
}

std::vector<Vec3> chaikin(std::vector<Vec3> pts,int iterations){
    if(pts.size()<3) return pts;
    for(int it=0;it<iterations;++it){
        std::vector<Vec3> out;
        out.reserve(pts.size()*2);
        out.push_back(pts.front());
        for(std::size_t i=0;i+1<pts.size();++i){
            const Vec3 a=pts[i],b=pts[i+1];
            out.push_back(a*0.75f+b*0.25f);
            out.push_back(a*0.25f+b*0.75f);
        }
        out.push_back(pts.back());
        pts=std::move(out);
    }
    std::vector<Vec3> compact;
    compact.reserve(pts.size());
    for(Vec3 p:pts){
        if(compact.empty() || distance_xz(compact.back(),p)>0.08f || std::abs(compact.back().y-p.y)>0.04f) compact.push_back(p);
    }
    return compact;
}

std::vector<Vec3> smooth_route(const CorridorEndpoint& a,const CorridorEndpoint& b,
                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    const float d=distance_xz(a.position,b.position);
    if(d<0.08f) return {};
    if(d<0.90f) return {a.position,b.position};
    const std::size_t ignore_a=a.kind==CorridorEndpointKind::Habitat?a.index:NONE_INDEX;
    const std::size_t ignore_b=b.kind==CorridorEndpointKind::Habitat?b.index:NONE_INDEX;
    if(a.kind==CorridorEndpointKind::Habitat && b.kind==CorridorEndpointKind::Habitat &&
       d<DIRECT_CORRIDOR_MAX_RASTERS && segment_clear(a.position,b.position,habitats,library,ignore_a,ignore_b)){
        return {a.position,b.position};
    }
    const Vec3 direct=flat_normal(b.position-a.position);
    const Vec3 lateral=perp_xz(direct);
    const float lead=std::clamp(d*0.22f,1.0f,3.2f);
    const float mid_scale=std::max(2.8f,std::min(9.0f,d*0.28f));
    const std::array<float,9> offsets={0.0f,1.0f,-1.0f,1.8f,-1.8f,2.6f,-2.6f,3.4f,-3.4f};

    for(float f:offsets){
        const Vec3 mid=(a.position+b.position)*0.5f+lateral*(f*mid_scale);
        std::vector<Vec3> controls={a.position,a.position+a.tangent*lead,mid,b.position+b.tangent*lead,b.position};
        auto path=chaikin(std::move(controls),3);
        if(path_clear(path,habitats,library,ignore_a,ignore_b)) return path;
    }
    return {};
}

// Build a short end-to-end splice whose terminal ring planes exactly match
// the two existing corridor ends.  The first derivative follows the outward
// tangent of corridor A; the final derivative follows the inward tangent of
// corridor B.  Because append_swept_shell() derives its end rings from these
// tangents, the new D-shaped pressure shell shares the exact same cross-section
// plane as each host corridor: no vestibule, no overlap sleeve and no air gap.
std::vector<Vec3> seamless_end_splice_route(Vec3 a,Vec3 outward_a,Vec3 b,Vec3 outward_b){
    const float d=distance_xz(a,b);
    if(d<0.02f) return {};
    outward_a=flat_normal(outward_a);
    outward_b=flat_normal(outward_b);
    if(length(outward_a)<0.001f) outward_a=flat_normal(b-a);
    if(length(outward_b)<0.001f) outward_b=flat_normal(a-b);

    const float lead=std::clamp(d*0.30f,0.12f,0.80f);
    const Vec3 p0=a;
    const Vec3 p1=a+outward_a*lead;
    // At the B end the route derivative must point opposite B's outward
    // direction so its terminal ring matches the host corridor's first ring.
    const Vec3 p2=b+outward_b*lead;
    const Vec3 p3=b;

    constexpr int SAMPLES=8;
    std::vector<Vec3> path;
    path.reserve(SAMPLES+1);
    for(int i=0;i<=SAMPLES;++i){
        const float t=(float)i/(float)SAMPLES;
        const float u=1.0f-t;
        Vec3 p=p0*(u*u*u)+p1*(3.0f*u*u*t)+p2*(3.0f*u*t*t)+p3*(t*t*t);
        // Keep the floor transition smooth when two corridor sections happen
        // to use different base heights.
        p.y=a.y+(b.y-a.y)*t;
        if(path.empty() || distance_xz(path.back(),p)>0.015f || std::abs(path.back().y-p.y)>0.01f)
            path.push_back(p);
    }
    if(path.size()<2) return {a,b};
    path.front()=a;
    path.back()=b;
    // append_swept_shell() derives each terminal ring from the first/last
    // path segment, not from the analytic Bezier derivative. Pin those two
    // short segments to the host directions so the ring planes are exactly
    // identical even with a coarse sampled curve.
    if(path.size()>=4){
        const float terminal=std::clamp(d*0.10f,0.04f,0.12f);
        path[1]=a+outward_a*terminal;
        path[path.size()-2]=b+outward_b*terminal;
    }
    return path;
}

Vec3 nearest_path_point(const std::vector<Vec3>& path,Vec3 p){
    Vec3 best=p;
    float best_d=std::numeric_limits<float>::max();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 q=closest_point_segment_xz(p,path[i],path[i+1]);
        const float d=distance_xz(p,q);
        if(d<best_d){ best_d=d; best=q; }
    }
    return best;
}


Vec3 nearest_path_tangent(const std::vector<Vec3>& path,Vec3 p){
    if(path.size()<2) return {1,0,0};
    float best=std::numeric_limits<float>::max();
    Vec3 tangent{1,0,0};
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 q=closest_point_segment_xz(p,path[i],path[i+1]);
        const float d=distance_xz(p,q);
        if(d<best){
            best=d;
            tangent=flat_normal(path[i+1]-path[i]);
        }
    }
    return length(tangent)>0.001f?tangent:Vec3{1,0,0};
}

std::pair<Vec3,Vec3> closest_pair(const std::vector<Vec3>& a,const std::vector<Vec3>& b){
    Vec3 best_a{},best_b{};
    float best=std::numeric_limits<float>::max();
    for(std::size_t i=0;i+1<a.size();++i){
        for(std::size_t j=0;j+1<b.size();++j){
            for(int s=0;s<=6;++s){
                const float t=(float)s/6.0f;
                const Vec3 p=a[i]+(a[i+1]-a[i])*t;
                const Vec3 q=closest_point_segment_xz(p,b[j],b[j+1]);
                const float d=distance_xz(p,q);
                if(d<best){ best=d;best_a=p;best_b=q; }
            }
        }
    }
    return {best_a,best_b};
}
} // namespace game::corridor_detail
