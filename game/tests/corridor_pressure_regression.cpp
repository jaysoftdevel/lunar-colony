#include "game/world/corridor/pressure_shell.hpp"
#include "game/world/corridor/detail.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace game;using namespace game::corridor_detail;
namespace {
void require(bool b,const std::string& m){if(!b)throw std::runtime_error(m);}
bool hit(Vec3 from,Vec3 to,const std::vector<CorridorConnection>& network){
    const Vec3 ray=to-from;
    for(const auto& c:network)if(c.construction.complete())for(std::size_t i=0;i+2<c.mesh.shell.size();i+=3){
        const auto a=c.mesh.shell[i],b=c.mesh.shell[i+1],d=c.mesh.shell[i+2];
        const auto e1=b-a,e2=d-a,p=cross(ray,e2);const double det=dot(e1,p);if(std::abs(det)<1.e-9)continue;
        const auto q=from-a;const double u=dot(q,p)/det;if(u<-.000001 || u>1.000001)continue;
        const auto r=cross(q,e1);const double v=dot(ray,r)/det;if(v<-.000001 || u+v>1.000001)continue;
        const double t=dot(e2,r)/det;if(t>1.e-6 && t<.999999)return true;
    }
    return false;
}
std::size_t probes=0;
void sealed(const std::vector<CorridorConnection>& network,Vec3 p,const std::string& label){
    for(int i=0;i<1024;++i){const float y=1.f-2.f*(i+.5f)/1024.f,a=i*2.39996323f,r=std::sqrt(1-y*y);const Vec3 ray{r*std::cos(a),y,r*std::sin(a)};
        require(hit(p,p+ray*200,network),label+" leaks at ray "+std::to_string(i));++probes;}
}
CorridorConnection tube(std::vector<Vec3> p){CorridorConnection c;c.path=std::move(p);c.construction.finish();c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.tangent=flat_normal(c.path[1]-c.path[0]);c.endpoint_b.tangent=flat_normal(c.path[c.path.size()-2]-c.path.back());return c;}
}
int main(){try{
 // Older saves have an airlock endpoint at zero and its next ring at .60 m.
 auto damaged=tube({{0,0,0},{.12f,.60f,0},{3.78f,.60f,0},{3.9f,.60f,0}});
 auto& da=damaged.endpoint_a;auto& db=damaged.endpoint_b;
 da.kind=db.kind=CorridorEndpointKind::Habitat;
 da.door.position={-1,.80f,0};db.door.position={4.9f,1.19f,0};
 da.door.normal={1,0,0};db.door.normal={-1,0,0};
 align_docking_path(da,db,damaged.path);
 for(const auto p:damaged.path)require(std::abs(p.y-(.80f-WALKWAY_Y))<.001f,"Airlock docking deck retained a drop");
 require(length(da.position-damaged.path.front())<.001f && length(db.position-damaged.path.back())<.001f,"Docking endpoints detached");
 const auto fixed=damaged.path;align_docking_path(da,db,damaged.path);
 for(std::size_t i=0;i<fixed.size();++i)require(length(fixed[i]-damaged.path[i])<.001f,"Docking repair is not idempotent");

 // The sleeve's frame must use the same rounded skin profile, not a freestanding
 // rectangular gate or oversized ring. Verify both endpoint orientations, a
 // raised sloping adapter, and the ground-to-flange load path.
 for(float yaw:{0.f,37.f,180.f})for(float rise:{0.f,.45f}){
    const Vec3 dir=rotate_y({1,0,0},yaw),side=perp_xz(dir);
    CorridorEndpoint ep;ep.kind=CorridorEndpointKind::Habitat;ep.tangent=dir*-1.f;
    ep.door.generated=true;ep.door.normal=dir;ep.door.wall_position={4,1.0f,-6};
    ep.door.position=ep.door.wall_position+dir*1.7f;
    ep.position=ep.door.position+dir*1.75f-Vec3{0,WALKWAY_Y-rise,0};
    const auto mesh=build_endpoint_fitting(ep);
    const auto door=doorway_cross_section(GENERATED_ENTRANCE_HALF_WIDTH,GENERATED_ENTRANCE_HEIGHT);
    const auto tube_profile=corridor_cross_section();
    require(door[7].y-door[2].y>.20f,"Sleeve still has a flat rectangular roof");
    const auto base=ep.door.position-Vec3{0,WALKWAY_Y,0};
    const auto crown=[&](float t){return base+(ep.position-base)*t+Vec3{0,std::lerp(door[7].y,tube_profile[7].y,t),0};};
    // The middle bay's flange begins at exactly this skin station. Its outer
    // crown must protrude, but the pressure-skin contact edge must still match.
    const auto skin=crown(.5f-.065f/1.75f);
    bool contact=false,outer=false,left_pad=false,right_pad=false;
    for(auto p:mesh.details){
        require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&p.y>=-.001f,"Invalid or buried connector member");
        contact|=length(p-skin)<.002f;
        outer|=std::abs(dot(p-skin,dir))<.003f&&std::abs(dot(p-skin,side))<.003f&&p.y>skin.y+.055f;
        const float along=dot(p-base,dir),lateral=dot(p-base,side);
        if(std::abs(p.y)<.001f&&std::abs(along-.875f)<.18f){left_pad|=lateral<-.85f;right_pad|=lateral>.85f;}
    }
    require(contact&&outer,"Rounded flange does not follow the skin crown");
    require(left_pad&&right_pad,"Rounded connector frame has no ground-bearing pair");
    for(std::size_t i=0;i+2<mesh.details.size();i+=3)
        require(length(cross(mesh.details[i+1]-mesh.details[i],mesh.details[i+2]-mesh.details[i]))>1e-9f,"Degenerate connector structural face");
    CorridorConnection detail_only;detail_only.construction.finish();detail_only.mesh.shell=mesh.details;
    require(!hit(ep.door.wall_position+dir*.15f+Vec3{0,1.0f,0},ep.position+Vec3{0,WALKWAY_Y+1.f,0},{detail_only}),"A new frame blocks the walking lane");
 }
 std::vector<CorridorConnection> n;
 for(auto p:{std::vector<Vec3>{{-20,0,0},{20,0,0}},std::vector<Vec3>{{-12,0,0},{0,0,0},{0,0,12}},std::vector<Vec3>{{-12,0,0},{-.12f,0,0},{0,0,.12f},{0,0,12}},std::vector<Vec3>{{-12,0,0},{0,.2f,0},{7,.4f,9}}}){
  n={tube(p)};rebuild_pressure_shells(n);sealed(n,p.front()+flat_normal(p[1]-p.front())*3+Vec3{0,1.1f,0},"Bend/slope");
  for(std::size_t i=1;i<p.size();++i)require(!hit((p[i-1]+p[i])*.5f+Vec3{0,1.1f,0},p[i]+Vec3{0,1.1f,0},n),"Bend retained an internal panel");
 }
 auto host=tube({{-25,0,0},{25,0,0}}),branch=tube({{0,0,1.87f},{0,0,16}});
 branch.endpoint_a.kind=CorridorEndpointKind::Corridor;branch.endpoint_a.index=0;branch.endpoint_a.surface_matched_junction=true;branch.endpoint_a.host_position={0,0,0};branch.endpoint_a.host_tangent={1,0,0};branch.endpoint_a.tangent={0,0,1};
 n={host,branch};rebuild_pressure_shells(n);for(auto p:{Vec3{-8,1.2f,0},Vec3{0,1.2f,0},Vec3{0,1.2f,5},Vec3{8,1.2f,0}})sealed(n,p,"T junction");
 require(!hit({0,1.2f,0},{0,1.2f,8},n),"T junction blocked doorway");
 n[1].construction.step=0;rebuild_pressure_shells(n);require(hit({0,1.2f,0},{0,1.2f,8},n),"Blueprint opened an active pressure wall");sealed(n,{-2,1.2f,0},"Planned branch");
 n[1].construction.finish();rebuild_pressure_shells(n);require(!hit({0,1.2f,0},{0,1.2f,8},n),"Commissioning kept the bulkhead");
 n.resize(1);rebuild_pressure_shells(n);require(hit({0,1.2f,0},{0,1.2f,8},n),"Demolition did not reseal host");
 for(float deg:{37.f,53.f,90.f,127.f,143.f}){
  const Vec3 d{std::cos(radians(deg)),0,std::sin(radians(deg))};n={host,tube({d*-18,d*18})};rebuild_pressure_shells(n);
  for(Vec3 p:{Vec3{0,1.1f,0},d*5+Vec3{0,1.1f,0},Vec3{-7,1.1f,0}})sealed(n,p,"Angled crossing "+std::to_string(deg));
  require(!hit({-8,1.1f,0},{8,1.1f,0},n),"Crossing blocked main walkway");require(!hit(d*-8+Vec3{0,1.1f,0},d*8+Vec3{0,1.1f,0},n),"Crossing blocked branch walkway");
 }
 for(float gap:{0.f,.08f,2.5f}){auto a=tube({{-18,0,0},{0,0,0}}),b=tube({{gap,0,0},{18,0,0}});a.endpoint_b.suppress_open_door=b.endpoint_a.suppress_open_door=true;n={a,b};if(gap>0)n.push_back(tube({{0,0,0},{gap,0,0}}));rebuild_pressure_shells(n);sealed(n,{-2,1.1f,0},"Splice");require(!hit({-5,1.1f,0},{5,1.1f,0},n),"Splice retained bulkhead");}
 // Repeated joins at rotated and translated map positions catch precision
 // errors that a single junction centered on the origin cannot expose.
 for(float yaw:{0.f,17.f,71.f})for(float offset:{0.f,100.f,-350.f}){
  const auto transform=[&](Vec3 p){return rotate_y(p,yaw)+Vec3{offset,0,offset*.37f};};
  auto a=host,b=branch;
  for(auto* c:{&a,&b}){
   for(auto& p:c->path)p=transform(p);
   for(auto* ep:{&c->endpoint_a,&c->endpoint_b}){ep->position=transform(ep->position);ep->host_position=transform(ep->host_position);ep->tangent=rotate_y(ep->tangent,yaw);ep->host_tangent=rotate_y(ep->host_tangent,yaw);}
  }
  n={a,b};rebuild_pressure_shells(n);sealed(n,transform({0,1.1f,0}),"Transformed junction");sealed(n,transform({0,1.1f,5}),"Transformed branch");
  require(!hit(transform({0,1.1f,0}),transform({0,1.1f,8}),n),"Transformed junction kept an internal wall");
 }
 std::cout<<"PASS "<<probes<<" outward rays: bends, slopes, T/X/angled joins, sparse panels, commissioning, demolition and splices\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
