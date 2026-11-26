#include "game/world/structural_model.hpp"
#include "game/world/structural_frame.hpp"
#include "game/world/robot_workshop.hpp"
#include <limits>

namespace game::structural {
namespace {
constexpr float bay_limit=1.50f;
int divisions(float span,float pitch=bay_limit){return std::max(1,int(std::ceil(span/pitch)));}
void skid(Builder& b,float hx,float hz,float y){
    ++b.metrics.frames;
    for(float z:{-hz,hz})b.beam({-hx,y,z},{hx,y,z},.18f,.24f,{0,1,0},true);
    for(float x:{-hx,0.f,hx})b.beam({x,y,-hz},{x,y,hz},.15f,.20f,{0,1,0},true);
    for(float x:{-hx,hx})for(float z:{-hz,hz})b.plate({x,y-.09f,z},{0,1,0},.38f,.38f);
}
void barrel(Builder& b,float x0,float x1,float cy,float ry,float rz,bool upper=false,bool cap_start=true,bool cap_end=true){
    const int n=divisions(x1-x0);const float depth=std::clamp(rz*.055f,.17f,.28f);
    const int half=upper?1:2;const int arc=std::max(12,divisions(pi*half*std::max(ry,rz)));
    const auto point=[&](float x,int j){const float a=pi*half*j/arc;return Vec3{x,cy+std::sin(a)*ry,std::cos(a)*rz};};
    for(int i=0;i<=n;++i){
        const float x=std::lerp(x0,x1,float(i)/n);++b.metrics.frames;
        for(int j=0;j<arc;++j){const auto a=point(x,j),c=point(x,j+1);b.beam(a,c,.14f,depth,{0,(a.y+c.y)*.5f-cy,(a.z+c.z)*.5f},true);}
        if(upper)b.beam(point(x,0)-Vec3{0,.22f,0},point(x,arc)-Vec3{0,.22f,0},.16f,.20f,{0,-1,0},true);
        if(i<n){
            const float next=std::lerp(x0,x1,float(i+1)/n);b.bay(next-x);
            for(int j=0;j<=(upper?arc:arc-1);++j){const int k=j;const auto a=point(x,k),c=point(next,k);
                b.beam(a,c,.085f,.12f,{0,a.y-cy,a.z});}
        }
        // Cradle feet end on the lower shell, rather than disappearing into the room.
        if(i%2==0||i==n)for(float sign:{-1.f,1.f}){
            const float z=sign*rz*.70f;const float y=upper?cy:cy-ry*std::sqrt(1-.70f*.70f);
            if(y>.35f){b.beam({x,.20f,z},{x,y,z},.16f,.18f,{1,0,0},true);b.plate({x,.20f,z},{0,1,0});}
        }
    }
    // A flat pressure bulkhead needs its own supported panel grid. Omit an
    // internal cap where a cylindrical bay opens into an adjoining hall.
    for(int end=0;end<2;++end)if(end?cap_end:cap_start){
        const float x=end?x1:x0;const Vec3 normal{end?1.f:-1.f,0,0};
        const int nz=divisions(2*rz),ny=divisions((upper?1.f:2.f)*ry);
        for(int j=1;j<nz;++j){const float z=std::lerp(-rz,rz,float(j)/nz),h=ry*std::sqrt(std::max(0.f,1-z*z/(rz*rz)));
            b.beam({x,upper?cy:cy-h,z},{x,cy+h,z},.085f,.13f,normal);}
        for(int j=1;j<ny;++j){const float y=std::lerp(upper?cy:cy-ry,cy+ry,float(j)/ny),dy=(y-cy)/ry;
            const float z=rz*std::sqrt(std::max(0.f,1-dy*dy));b.beam({x,y,-z},{x,y,z},.085f,.13f,normal);}
        ++b.metrics.frames;b.bay(2*rz/nz);b.bay((upper?1.f:2.f)*ry/ny);
    }
    b.bay(pi*half*std::max(ry,rz)/arc);
}
void hall(Builder& b,Vec3 low,Vec3 high,bool cap_start=true,bool cap_end=true){
    const int nx=divisions(high.x-low.x),nz=divisions(high.z-low.z);
    for(int i=0;i<=nx;++i){const float x=std::lerp(low.x,high.x,float(i)/nx);++b.metrics.frames;
        const Vec3 a{x,low.y,low.z},c{x,high.y,low.z},d{x,high.y,high.z},e{x,low.y,high.z};
        b.beam(a,c,.18f,.23f,{0,0,-1},true);b.beam(c,d,.18f,.25f,{0,1,0},true);b.beam(d,e,.18f,.23f,{0,0,1},true);b.beam(e,a,.17f,.20f,{0,-1,0},true);
        for(float z:{low.z,high.z}){
            b.plate({x,low.y,z},{0,1,0});
            // Knee braces transfer racking loads into the header without crossing an aisle.
            const float sign=z==low.z?1.f:-1.f;
            b.beam({x,high.y-.60f,z},{x,high.y,z+sign*.60f},.10f,.14f,{1,0,0});
        }
    }
    for(int j=0;j<=nz;++j){const float z=std::lerp(low.z,high.z,float(j)/nz);
        b.beam({low.x,high.y,z},{high.x,high.y,z},.09f,.13f);}
    const int ny=divisions(high.y-low.y);
    for(int k=0;k<=ny;++k){const float y=std::lerp(low.y,high.y,float(k)/ny);
        for(float z:{low.z,high.z})b.beam({low.x,y,z},{high.x,y,z},.10f,.14f,{0,0,1});
        for(int end=0;end<2;++end)if(end?cap_end:cap_start){const float x=end?high.x:low.x;b.beam({x,y,low.z},{x,y,high.z},.10f,.14f,{1,0,0});}
    }
    for(int end=0;end<2;++end)if(end?cap_end:cap_start)for(int j=1;j<nz;++j){const float x=end?high.x:low.x;const float z=std::lerp(low.z,high.z,float(j)/nz);b.beam({x,low.y,z},{x,high.y,z},.10f,.14f,{1,0,0});}
    b.bay((high.y-low.y)/ny);
    b.bay((high.x-low.x)/nx);b.bay((high.z-low.z)/nz);
}
void dome(Builder& b,Vec3 center,float rx,float ry,float rz){
    // Closed tension rings and meridional ribs share every node; all loads close
    // into the foundation ring. No diagonal is suspended in a glazing panel.
    const int meridians=std::clamp(4*int(std::ceil(2*pi*std::max(rx,rz)/bay_limit/4)),12,96);
    const int levels=divisions(pi*.5f*std::max({rx,ry,rz}));
    const float depth=std::clamp(std::max(rx,rz)*.045f,.17f,.30f);
    const auto point=[&](int j,int k){const float a=2*pi*(j+.5f)/meridians,t=pi*.5f*k/levels;
        return center+Vec3{rx*std::cos(t)*std::cos(a),ry*std::sin(t),rz*std::cos(t)*std::sin(a)};};
    for(int j=0;j<meridians;++j){
        ++b.metrics.frames;
        for(int k=0;k<levels;++k){const auto a=point(j,k),c=point(j,k+1);const auto radial=normalize((a+c)*.5f-center);
            b.beam(a,c,.14f,depth,radial,true);
            b.beam(a,point((j+1)%meridians,k),.095f,.14f,radial,k==0);
        }
        const auto foot=point(j,0);b.beam({foot.x,.22f,foot.z},foot,.15f,.18f,normalize(Vec3{foot.x,0,foot.z}),true);
        if(j%2==0)b.plate({foot.x,.22f,foot.z},{0,1,0});
    }
    // Crown compression/tension hub consolidates the converging ribs.
    b.box(center+Vec3{0,ry,0},{1,0,0},{0,1,0},{0,0,1},{.38f,.14f,.38f});
    b.bay(2*pi*std::max(rx,rz)/meridians);b.bay(pi*.5f*std::max({rx,ry,rz})/levels);
}
void panel_frames(Builder& b,const std::vector<Vec3>& body){
    // Only real rectangular facade quads receive stiffeners. Curved pressure
    // vessel triangles and circular endcaps must never acquire box-like cages.
    for(std::size_t i=0;i+5<body.size();i+=6){
        Vec3 lo=body[i],hi=lo;for(int k=1;k<6;++k){const auto p=body[i+k];lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};}
        const auto d=hi-lo;int axis=-1;
        if(d.x<.0001f&&d.y>.65f&&d.z>.65f)axis=0;
        if(d.y<.0001f&&d.x>.65f&&d.z>.65f)axis=1;
        if(d.z<.0001f&&d.x>.65f&&d.y>.65f)axis=2;
        if(axis<0)continue;
        const Vec3 u=axis==0?Vec3{0,0,1}:Vec3{1,0,0},v=axis==1?Vec3{0,0,1}:Vec3{0,1,0};
        const float width=dot(d,u),height=dot(d,v);
        bool rectangle=true;for(int k=0;k<6;++k){const auto p=body[i+k]-lo;const float x=dot(p,u),y=dot(p,v);
            if((std::abs(x)>.0001f&&std::abs(x-width)>.0001f)||(std::abs(y)>.0001f&&std::abs(y-height)>.0001f))rectangle=false;}
        if(!rectangle)continue;
        const Vec3 normal=normalize(cross(body[i+1]-body[i],body[i+2]-body[i]))*-1.f;const auto start=lo+normal*.035f;
        const int nx=divisions(width),ny=divisions(height);
        for(int j=0;j<=nx;++j){const auto a=start+u*(width*j/nx);b.beam(a,a+v*height,.07f,.10f,normal);}
        for(int j=0;j<=ny;++j){const auto a=start+v*(height*j/ny);b.beam(a,a+u*width,.07f,.10f,normal);}
        ++b.metrics.frames;b.bay(width/nx);b.bay(height/ny);
    }
}
void vessel_ring(Builder& b,Vec3 center,float radius){
    constexpr int count=32;
    for(int i=0;i<count;++i){const float a=2*pi*i/count,c=2*pi*(i+1)/count;
        b.beam(center+Vec3{radius*std::cos(a),0,radius*std::sin(a)},center+Vec3{radius*std::cos(c),0,radius*std::sin(c)},.10f,.15f,{0,1,0});}
    ++b.metrics.frames;
}
} // namespace

Metrics reinforce(HabitatMesh& m,HabitatKind kind){
    if(m.dome_radii.x>0){
        const auto start=m.hull_frames.size();Builder b{m.hull_frames,m.edges,{}};const auto r=m.dome_radii;
        dome(b,{0,m.dome_spring_y,0},r.x,r.y,r.z);
        const int segments=std::max(24,divisions(2*pi*std::max(r.x,r.z)));
        for(float y:{m.walkable_floor_y+.12f,m.walkable_floor_y+1.05f,m.walkable_floor_y+2.f})
            for(int j=0;j<segments;++j){const float a=2*pi*j/segments,c=2*pi*(j+1)/segments;
                b.beam({r.x*std::cos(a),y,r.z*std::sin(a)},{r.x*std::cos(c),y,r.z*std::sin(c)},.09f,.13f,{std::cos(a),0,std::sin(a)});
            }
        b.metrics.first_vertex=start;b.metrics.vertices=m.hull_frames.size()-start;return b.metrics;
    }
    Builder b{m.details,m.edges,{}};const auto before=m.details.size();
    const float base_x[]={10,10.5f,11,13.5f,14,17,18,21,13,15};
    const float s=m.footprint_x/base_x[int(kind)];
    switch(kind){
    case HabitatKind::AirLock:barrel(b,-3.6f*s,3.6f*s,2.55f,2.19f,2.19f*s,false,false,false);for(float x:{-4.05f,4.05f})hall(b,{(x-.725f)*s,.45f,-1.91f*s},{(x+.725f)*s,4.15f,1.91f*s},x<0,x>0);break;
    case HabitatKind::OxygenGeneratorDome:dome(b,{0,1,0},4.39f*s,6.56f,4.39f*s);break;
    case HabitatKind::CantineDome:dome(b,{0,1.05f,0},4.59f*s,4.04f,4.59f*s);break;
    case HabitatKind::BioDome:barrel(b,-5.25f*s,5.25f*s,1.20f,3.34f,3.34f*s,true);break;
    case HabitatKind::CrewQuarters:barrel(b,-5.4f*s,5.4f*s,2.65f,2.39f,2.39f*s);break;
    case HabitatKind::ProcessingFacility:barrel(b,-4.45f*s,5.25f*s,3.55f,2.79f,2.79f*s);break;
    case HabitatKind::Factory:hall(b,{-6.f*s,.70f,-3.74f*s},{1.f*s,6.04f,3.74f*s},true,false);barrel(b,1.05f*s,6.35f*s,3.40f,2.79f,2.79f*s,false,false,true);break;
    case HabitatKind::Manufactory:hall(b,{-7.7f*s,.80f,-4.29f*s},{.9f*s,7.04f,4.29f*s},true,false);barrel(b,.95f*s,8.6f*s,3.65f,3.04f,3.04f*s,false,false,true);break;
    case HabitatKind::SickBay:dome(b,{0,1,0},4.59f*s,3.59f,4.59f*s);break;
    case HabitatKind::Laboratory:barrel(b,-5.4f*s,5.4f*s,3.0f,2.59f,2.59f*s);break;
    default:break;
    }
    b.metrics.first_vertex=before;b.metrics.vertices=m.details.size()-before;return b.metrics;
}

Metrics reinforce(ResourceMesh& m,ResourceKind kind){
    Builder b{m.details,m.edges,{}};const auto before=m.details.size();
    const float hx=m.footprint_x*.38f,hz=m.footprint_z*.38f;
    if(kind==ResourceKind::StorageDome){const float s=m.footprint_x/13.f,sy=m.height/7.f;dome(b,{0,.95f*sy,0},5.25f*s,4.83f*sy,5.25f*s);}
    else if(kind==ResourceKind::RobotWorkshop){
        // Transverse barrel spans the rover aisle. All feet stay in the side walls.
        const int n=divisions(9.6f);
        for(int j=0;j<=n;++j){const float z=-4.8f+9.6f*j/n;++b.metrics.frames;
            for(int k=0;k<16;++k){const float a=pi*k/16,c=pi*(k+1)/16;
                const Vec3 p{5.30f*std::cos(a),workshop::roof_height(5.30f*std::cos(a),z),z},q{5.30f*std::cos(c),workshop::roof_height(5.30f*std::cos(c),z),z};
                b.beam(p,q,.15f,.19f,{(p.x+q.x)*.5f,(p.y+q.y)*.5f-workshop::kRoofSpring,0},true);}
            for(float x:{-5.30f,5.30f}){b.beam({x,.4f,z},{x,workshop::kRoofSpring,z},.17f,.18f,{1,0,0},true);b.plate({x,.46f,z},{0,1,0});}
        }
        b.bay(9.6f/n);
    }else if(kind==ResourceKind::StorageGate){
        const float x=m.footprint_x*.40f,z=m.footprint_z*.5f+.02f,y=m.height-.08f;
        for(float side:{-1.f,1.f}){
            b.beam({-x,.12f,side*z},{-x,y,side*z},.14f,.17f,{1,0,0},true);
            b.beam({x,.12f,side*z},{x,y,side*z},.14f,.17f,{1,0,0},true);
            b.beam({-x,y,side*z},{x,y,side*z},.14f,.17f);}
        for(float at:{-x,x})b.beam({at,y,-z},{at,y,z},.15f,.18f,{0,1,0},true);
        b.metrics.frames=2;
    }else if(kind==ResourceKind::SolarPanel){const float s=m.footprint_x/11.f;skid(b,2.9f*s,2.2f*s,.18f);
        for(float x:{-1.f,1.f})for(float z:{-1.f,1.f})b.beam({x*1.3f*s,.60f,z*s},{x*.35f*s,2.75f,z*.35f*s},.12f,.16f,{x,0,z});
    }else if(kind==ResourceKind::Antenna){const float s=m.footprint_x/8.f;
        for(int i=0;i<3;++i){++b.metrics.frames;const float a=2*pi*i/3;const Vec3 p{3.2f*s*std::cos(a),.20f,3.2f*s*std::sin(a)};b.beam(p,{0,2.4f,0},.15f,.20f,normalize(Vec3{p.x,0,p.z}),true);b.plate(p,{0,1,0});}
    }else if(kind==ResourceKind::HydrogenTank){const float s=m.height/9.1f;const float cy=4.05f*s,r=3.35f*s;
        for(float x:{-2.15f*s,2.15f*s})for(float z:{-1.5f*s,1.5f*s}){
            const float y=cy-std::sqrt(r*r-x*x-z*z);b.beam({x,.56f*s,z},{x,y,z},.16f,.21f,{1,0,0},true);b.plate({x,.61f*s,z},{0,1,0});}
        vessel_ring(b,{0,cy-2.1f*s,0},std::sqrt(r*r-2.1f*s*2.1f*s)+.035f);
    }else if(kind==ResourceKind::OxygenTank){const float s=m.height/4.6f;skid(b,3.1f*s,2.3f*s,.24f*s);
        for(float x:{-1.65f*s,1.65f*s})for(float y:{.74f*s,2.86f*s})vessel_ring(b,{x,y,0},1.32f*s);
        panel_frames(b,m.body);
    }else if(kind==ResourceKind::LandingPad){
        const float s=m.footprint_x/20.f;
        for(int j=0;j<16;++j){const float a=2*pi*j/16;const Vec3 n{std::cos(a),0,std::sin(a)};
            b.beam(n*(1.8f*s)+Vec3{0,.36f,0},n*(7.6f*s)+Vec3{0,.36f,0},.16f,.24f,{0,1,0},true);}
        vessel_ring(b,{0,.36f,0},7.6f*s);b.metrics.frames=16;
    }else{
        skid(b,hx,hz,std::min(.48f,m.height*.08f));panel_frames(b,m.body);
        if(kind==ResourceKind::WaterStorage){const float s=m.footprint_x/11.f;
            for(float x:{-2.65f*s,2.65f*s})for(float z:{-1.75f*s,1.75f*s})for(float y:{1.15f,5.70f})vessel_ring(b,{x,y,z},1.32f*s);}
    }
    b.metrics.first_vertex=before;b.metrics.vertices=m.details.size()-before;return b.metrics;
}
} // namespace game::structural
