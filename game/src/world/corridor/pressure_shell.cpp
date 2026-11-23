#include "game/world/corridor/pressure_shell.hpp"
#include "game/world/corridor/detail.hpp"
#include <array>
#include <cmath>
#include <limits>

namespace game::corridor_detail {
namespace {
constexpr double epsilon=2.e-5;
struct Plane {Vec3 n;double d;};
using Polygon=std::vector<Vec3>;
struct Face {Polygon points;Plane plane;bool emit=true;};
struct Cell {std::vector<Face> faces;Vec3 lo{},hi{};std::size_t owner{};bool complete{};};
double product(Vec3 a,Vec3 b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
double distance(const Plane& p,Vec3 v){return product(p.n,v)-p.d;}
Vec3 interpolate(Vec3 a,Vec3 b,double t){return {float(a.x+(double(b.x)-a.x)*t),float(a.y+(double(b.y)-a.y)*t),float(a.z+(double(b.z)-a.z)*t)};}
Polygon clip(const Polygon& polygon,const Plane& plane,bool inside){
    Polygon out;if(polygon.empty())return out;
    Vec3 a=polygon.back();double da=distance(plane,a);
    for(Vec3 b:polygon){
        const double db=distance(plane,b);const bool ai=inside?da<=0:da>=0,bi=inside?db<=0:db>=0;
        if(ai!=bi)out.push_back(interpolate(a,b,da/(da-db)));
        if(bi)out.push_back(b);
        a=b;da=db;
    }
    Polygon clean;for(auto p:out)if(clean.empty() || length(p-clean.back())>1.e-6f)clean.push_back(p);
    if(clean.size()>1 && length(clean.front()-clean.back())<1.e-6f)clean.pop_back();
    return clean;
}
void add_face(Cell& cell,Polygon points,Vec3 center,bool emit=true){
    if(points.size()<3)return;
    Vec3 normal{};
    for(std::size_t j=1;j+1<points.size();++j){normal=cross(points[j]-points[0],points[j+1]-points[0]);if(length(normal)>1.e-7f)break;}
    if(length(normal)<1.e-7f)return;
    normal=normalize(normal);
    if(product(normal,points[0]-center)<0){std::reverse(points.begin(),points.end());normal=normal*-1.f;}
    cell.faces.push_back({std::move(points),{normal,0},emit});
    auto& face=cell.faces.back();face.plane.d=product(face.plane.n,face.points.front());
}
Cell loft(const Polygon& a,Polygon b,std::size_t owner,bool complete,bool cap_a=true,bool cap_b=true){
    Cell c;c.owner=owner;c.complete=complete;if(a.size()!=b.size() || a.size()<3)return c;
    // A hatch at the opposite end of a corridor has reversed lateral orientation.
    if(a.size()==13 && dot(a[1]-a[0],b[1]-b[0])<0){std::swap(b[0],b[1]);std::reverse(b.begin()+2,b.end());}
    Vec3 center{};for(auto p:a)center=center+p;for(auto p:b)center=center+p;center=center/float(2*a.size());
    add_face(c,a,center,cap_a);add_face(c,b,center,cap_b);
    for(std::size_t i=0;i<a.size();++i){const auto j=(i+1)%a.size();add_face(c,{a[i],b[i],b[j],a[j]},center);}
    c.lo=c.hi=a.front();for(const auto* ring:std::array<const Polygon*,2>{&a,&b})for(auto p:*ring){
        c.lo={std::min(c.lo.x,p.x),std::min(c.lo.y,p.y),std::min(c.lo.z,p.z)};
        c.hi={std::max(c.hi.x,p.x),std::max(c.hi.y,p.y),std::max(c.hi.z,p.z)};
    }
    Polygon points=a;points.insert(points.end(),b.begin(),b.end());
    bool convex=true;for(const auto& face:c.faces)for(auto p:points)if(distance(face.plane,p)>epsilon)convex=false;
    // Different collar shoulders or sloping bends need their convex envelope.
    if(!convex){
        const auto original=c.faces;std::vector<Plane> planes;
        for(std::size_t i=0;i<points.size();++i)for(std::size_t j=i+1;j<points.size();++j)for(std::size_t k=j+1;k<points.size();++k){
            Vec3 n=cross(points[j]-points[i],points[k]-points[i]);if(length(n)<1.e-6f)continue;n=normalize(n);
            double d=product(n,points[i]);bool positive=false,negative=false;
            for(auto p:points){const double q=product(n,p)-d;positive=positive || q>epsilon;negative=negative || q<-epsilon;}
            if(positive && negative)continue;
            if(positive){n=n*-1.f;d=-d;}
            bool duplicate=false;for(auto p:planes)if(dot(p.n,n)>.99999f && std::abs(p.d-d)<epsilon){duplicate=true;break;}
            if(!duplicate)planes.push_back({n,d});
        }
        c.faces.clear();
        for(auto plane:planes){
            Polygon face;Vec3 middle{};
            for(auto p:points)if(std::abs(distance(plane,p))<epsilon){bool duplicate=false;for(auto q:face)if(length(p-q)<1.e-5f)duplicate=true;if(!duplicate){face.push_back(p);middle=middle+p;}}
            if(face.size()<3)continue;
            middle=middle/float(face.size());
            const Vec3 u=normalize(face.front()-middle),v=cross(plane.n,u);
            std::sort(face.begin(),face.end(),[&](Vec3 p,Vec3 q){return std::atan2(dot(p-middle,v),dot(p-middle,u))<std::atan2(dot(q-middle,v),dot(q-middle,u));});
            bool emit=true;for(const auto& f:original)if(!f.emit && dot(f.plane.n,plane.n)>.99999f && std::abs(f.plane.d-plane.d)<epsilon)emit=false;
            add_face(c,std::move(face),center,emit);
        }
    }
    return c;
}
bool overlap(const Cell& a,const Cell& b){return a.lo.x<=b.hi.x+epsilon && a.hi.x>=b.lo.x-epsilon && a.lo.y<=b.hi.y+epsilon && a.hi.y>=b.lo.y-epsilon && a.lo.z<=b.hi.z+epsilon && a.hi.z>=b.lo.z-epsilon;}
Polygon ring(Vec3 p,Vec3 side,const std::vector<Vec2>& section){Polygon out;for(auto q:section)out.push_back(ring_vertex(p,side,q));return out;}
std::vector<Polygon> path_rings(const std::vector<Vec3>& path){
    std::vector<Polygon> rings;const auto section=corridor_cross_section();
    for(std::size_t i=0;i<path.size();++i){
        const Vec3 before=i?flat_normal(path[i]-path[i-1]):flat_normal(path[1]-path[0]);
        const Vec3 after=i+1<path.size()?flat_normal(path[i+1]-path[i]):before;
        Vec3 tangent=flat_normal(before+after);if(length(tangent)<.001f)tangent=after;
        const float scale=1.f/std::max(.20f,std::abs(dot(perp_xz(tangent),perp_xz(after))));
        rings.push_back(ring(path[i],perp_xz(tangent)*scale,section));
    }
    return rings;
}
void add_corridor_cells(std::vector<Cell>& cells,const CorridorConnection& c,std::size_t owner){
    if(c.path.size()<2)return;
    const auto rings=path_rings(c.path);
    for(std::size_t i=1;i<rings.size();++i)if(length(c.path[i]-c.path[i-1])>1.e-5f)cells.push_back(loft(rings[i-1],rings[i],owner,c.construction.complete()));
    for(int end=0;end<2;++end){
        const auto& ep=end?c.endpoint_b:c.endpoint_a;const auto& terminal=end?rings.back():rings.front();
        if(ep.kind==CorridorEndpointKind::Habitat || ep.kind==CorridorEndpointKind::Resource){
            Vec3 normal=flat_normal(ep.door.normal);if(length(normal)<.001f)continue;
            const Vec3 side=perp_xz(normal),base=ep.door.position-Vec3{0,geometry::kCorridorDeckY,0};
            float inner=dot(ep.door.wall_position-base,normal)-.20f;
            for(auto p:ep.door.wall_profile)inner=std::min(inner,dot(p-base,normal)-.20f);
            const auto profile=doorway_cross_section(GENERATED_ENTRANCE_HALF_WIDTH,GENERATED_ENTRANCE_HEIGHT);
            const auto inside=ring(base+normal*inner,side,profile),hatch=ring(base,side,profile);
            // The liner penetrates the building hull. Its interior mouth is
            // intentionally open to the building; every exterior face is closed.
            cells.push_back(loft(inside,hatch,owner,c.construction.complete(),false,true));
            cells.push_back(loft(hatch,terminal,owner,c.construction.complete()));
        }else if(ep.kind==CorridorEndpointKind::Corridor && ep.surface_matched_junction){
            const auto contact=ring(ep.host_position,perp_xz(ep.tangent),corridor_cross_section());
            cells.push_back(loft(contact,terminal,owner,c.construction.complete()));
        }
    }
}
std::vector<Polygon> outside(const Polygon& polygon,const Face& face,const Cell& other,bool owns_coplanar){
    for(const auto& plane:other.faces){
        bool coplanar=true;for(auto p:polygon)if(std::abs(distance(plane.plane,p))>epsilon){coplanar=false;break;}
        if(coplanar && dot(plane.plane.n,face.plane.n)>.9999f && owns_coplanar)return {polygon};
    }
    Polygon inside=polygon;std::vector<Polygon> result;
    for(const auto& boundary:other.faces){
        double min=std::numeric_limits<double>::max(),max=-min;
        for(auto p:inside){const double d=distance(boundary.plane,p);min=std::min(min,d);max=std::max(max,d);}
        if(min>epsilon){result.push_back(std::move(inside));return result;}
        if(max<=epsilon)continue;
        auto part=clip(inside,boundary.plane,false);if(part.size()>=3)result.push_back(std::move(part));
        inside=clip(inside,boundary.plane,true);if(inside.size()<3)return result;
    }
    return result;
}
}
void rebuild_pressure_shells(std::vector<CorridorConnection>& connections){
    std::vector<Cell> cells;
    for(std::size_t i=0;i<connections.size();++i){connections[i].mesh.shell.clear();add_corridor_cells(cells,connections[i],i);}
    for(const auto& chamber:pressure_chambers(connections)){
        Polygon a,b;for(auto p:chamber.footprint){a.push_back({p.x,chamber.position.y+CORRIDOR_FLOOR_Y,p.z});b.push_back({p.x,chamber.position.y+CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS,p.z});}
        cells.push_back(loft(a,b,chamber.owner,connections[chamber.owner].construction.complete()));
    }
    for(std::size_t i=0;i<cells.size();++i){
        const auto& cell=cells[i];std::vector<std::size_t> neighbors;
        for(std::size_t j=0;j<cells.size();++j)if(i!=j && cell.complete==cells[j].complete && overlap(cell,cells[j]))neighbors.push_back(j);
        for(const auto& face:cell.faces){
            if(!face.emit)continue;
            std::vector<Polygon> fragments{face.points};
            for(auto j:neighbors){
                std::vector<Polygon> next;
                for(const auto& p:fragments){auto parts=outside(p,face,cells[j],i<j);for(auto& part:parts)next.push_back(std::move(part));}
                fragments=std::move(next);if(fragments.empty())break;
            }
            auto& shell=connections[cell.owner].mesh.shell;
            for(const auto& p:fragments)for(std::size_t j=1;j+1<p.size();++j)if(length(cross(p[j]-p[0],p[j+1]-p[0]))>1.e-8f)tri(shell,p[0],p[j],p[j+1]);
        }
    }
}
} // namespace game::corridor_detail
