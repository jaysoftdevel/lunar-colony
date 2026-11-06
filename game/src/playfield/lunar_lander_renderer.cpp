#include "game/game_app.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace game {
namespace {
using eng::Vec3;using eng::Color;
constexpr float pi=3.14159265359f;
struct Part {Color color;std::vector<Vec3> edges;};
struct Mesh {
    std::vector<Part> parts;
    // Only topology is submitted: no filled faces, materials or textures.
    void quad(Color c,Vec3 a,Vec3 b,Vec3 d,Vec3 e){
        auto it=std::find_if(parts.begin(),parts.end(),[&](const Part& p){return p.color.r==c.r&&p.color.g==c.g&&p.color.b==c.b&&p.color.a==c.a;});
        if(it==parts.end()){parts.push_back({c,{}});it=parts.end()-1;}
        it->edges.insert(it->edges.end(),{a,b,b,d,d,e,e,a});
    }
    void box(Vec3 p,Vec3 size,Color c){
        Vec3 h=size*0.5f;
        const Vec3 v[]={{p.x-h.x,p.y-h.y,p.z-h.z},{p.x+h.x,p.y-h.y,p.z-h.z},{p.x+h.x,p.y+h.y,p.z-h.z},{p.x-h.x,p.y+h.y,p.z-h.z},
            {p.x-h.x,p.y-h.y,p.z+h.z},{p.x+h.x,p.y-h.y,p.z+h.z},{p.x+h.x,p.y+h.y,p.z+h.z},{p.x-h.x,p.y+h.y,p.z+h.z}};
        for(const auto f:std::array<std::array<int,4>,6>{{{{0,1,2,3}},{{4,7,6,5}},{{0,4,5,1}},{{3,2,6,7}},{{1,5,6,2}},{{0,3,7,4}}}})quad(c,v[f[0]],v[f[1]],v[f[2]],v[f[3]]);
    }
    void beam(Vec3 a,Vec3 b,float width,Color c){
        Vec3 dir=eng::normalize(b-a),u=eng::normalize(eng::cross(dir,std::abs(dir.y)>0.9f?Vec3{1,0,0}:Vec3{0,1,0}))*(width*0.5f);
        Vec3 v=eng::normalize(eng::cross(dir,u))*(width*0.5f);
        quad(c,a-u-v,b-u-v,b+u-v,a+u-v);quad(c,a+u-v,b+u-v,b+u+v,a+u+v);
        quad(c,a+u+v,b+u+v,b-u+v,a-u+v);quad(c,a-u+v,b-u+v,b-u-v,a-u-v);
        quad(c,a-u-v,a+u-v,a+u+v,a-u+v);quad(c,b-u-v,b-u+v,b+u+v,b+u-v);
    }
    void cylinder(Vec3 p,float radius,float height,Color c,float top_radius=-1){
        const float r1=top_radius<0?radius:top_radius;
        for(int i=0;i<20;++i){float a=(float)i*pi*0.1f,b=(float)(i+1)*pi*0.1f;
            Vec3 v0=p+Vec3{radius*std::cos(a),0,radius*std::sin(a)},v1=p+Vec3{radius*std::cos(b),0,radius*std::sin(b)};
            Vec3 v2=p+Vec3{r1*std::cos(b),height,r1*std::sin(b)},v3=p+Vec3{r1*std::cos(a),height,r1*std::sin(a)};
            const float light=0.76f+0.24f*(0.5f+0.5f*std::cos(a-0.6f));Color shade{c.r*light,c.g*light,c.b*light,c.a};
            quad(shade,v0,v1,v2,v3);
        }
    }
    void draw(eng::Renderer& r,Vec3 offset,float roll=0,float opacity=1)const{
        const float a=roll*pi/180,c=std::cos(a),s=std::sin(a);
        for(const auto& part:parts){auto points=part.edges;for(auto& v:points){v={v.x*c-v.y*s,v.x*s+v.y*c,v.z};v+=offset;}r.draw_lines_3d(points,{std::max(.35f,part.color.r),std::max(.45f,part.color.g),std::max(.5f,part.color.b),part.color.a*opacity},1);}
    }
};
constexpr Color white{0.76f,0.79f,0.78f,1},gold{0.60f,0.39f,0.14f,1},dark{0.075f,0.095f,0.12f,1},metal{0.35f,0.40f,0.44f,1},blue{0.10f,0.26f,0.42f,1};
float smooth(float u){u=std::clamp(u,0.0f,1.0f);return u*u*(3-2*u);}

void crate(Mesh& m,Vec3 p,Color c){
    m.box(p+Vec3{0,.65f,0},{1.7f,1.2f,1.45f},c);
    for(float x:{-.65f,.65f})m.box(p+Vec3{x,.65f,0},{.10f,1.27f,1.53f},metal);
    m.box(p+Vec3{0,.65f,.74f},{.55f,.32f,.035f},white);
    for(float x:{-.6f,.6f})m.box(p+Vec3{x,.04f,0},{.22f,.12f,1.5f},dark);
}
}
void GameApp::render_lunar_lander(eng::Renderer& r){
    if(!lander_.active()||lander_.awaiting_site)return;
    const auto flight=lunar_lander_flight(lander_.in_flight()?lander_.elapsed:kLanderDescentSeconds);
    Vec3 origin=lander_.site+flight.offset;
    const float settle=lander_.phase==LanderPhase::Settling?std::exp(-lander_.elapsed*2.5f)*std::sin(lander_.elapsed*8)*.055f:0;
    origin.y+=settle;
    Mesh frame,top,tanks,gear;
    // A low, wide octagonal chassis with an open, beveled portal at the nose.
    // The ramp hinge stays aligned with the rover's deck and deployment path.
    const std::array<Vec3,8> chassis{{
        {-3.35f,2.36f,-4.15f},{3.35f,2.36f,-4.15f},{5.05f,2.36f,-2.85f},{5.05f,2.36f,2.0f},
        {3.25f,2.36f,3.60f},{-3.25f,2.36f,3.60f},{-5.05f,2.36f,2.0f},{-5.05f,2.36f,-2.85f}
    }};
    for(std::size_t i=0;i<chassis.size();++i){
        const auto a=chassis[i],b=chassis[(i+1)%chassis.size()];
        frame.beam(a,b,.23f,metal);
        frame.beam(a-Vec3{0,.40f,0},b-Vec3{0,.40f,0},.24f,metal);
        frame.beam(a,b-Vec3{0,.40f,0},.12f,white);
    }
    const std::array<Vec3,6> portal{{
        {-2.42f,2.42f,3.54f},{-2.42f,4.12f,3.54f},{-1.75f,4.88f,3.54f},
        {1.75f,4.88f,3.54f},{2.42f,4.12f,3.54f},{2.42f,2.42f,3.54f}
    }};
    for(float z:{3.54f,.10f,-3.10f}){
        for(std::size_t i=0;i+1<portal.size();++i){
            auto a=portal[i],b=portal[i+1];
            a.z=b.z=z;
            frame.beam(a,b,z>3?.29f:.18f,white);
        }
    }
    for(const auto& edge:portal) frame.beam(edge,{edge.x*.88f,edge.y,-3.10f},.15f,white);
    for(float x:{-1.85f,0.f,1.85f}) frame.beam({x,2.38f,-3.75f},{x,2.38f,3.55f},.10f,metal);
    for(float x:{-1.f,1.f}){
        frame.beam({x*2.39f,4.18f,3.5f},{x*4.9f,4.55f,2.02f},.22f,white);
        frame.beam({x*4.9f,4.55f,2.02f},{x*4.9f,4.55f,-2.85f},.20f,white);
        frame.beam({x*4.9f,4.55f,-2.85f},{x*3.4f,4.68f,-4.08f},.22f,white);
        for(float z:{-2.8f,2.05f}){
            frame.beam({x*4.92f,2.33f,z},{x*4.92f,4.55f,z},.17f,white);
            frame.beam({x*4.92f,2.33f,z},{x*2.55f,4.63f,z*.88f},.13f,metal);
        }
        frame.box({x*2.77f,4.45f,3.13f},{.52f,.34f,.37f},dark);
        for(float offset:{-.14f,.14f}) frame.box({x*2.77f+offset,4.46f,3.34f},{.19f,.18f,.055f},white);
        for(int i=0;i<3;++i){
            const float z=-2.15f+(float)i*1.73f;
            const Vec3 base{x*3.85f,2.52f,z};
            tanks.cylinder(base,.47f,.36f,gold,.70f);
            tanks.cylinder(base+Vec3{0,.36f,0},.70f,1.19f,gold);
            tanks.cylinder(base+Vec3{0,1.55f,0},.70f,.43f,gold,.29f);
            tanks.cylinder(base+Vec3{0,.85f,0},.74f,.09f,metal);
            tanks.beam(base+Vec3{0,1.98f,0},base+Vec3{0,2.16f,0},.12f,white);
        }
        tanks.beam({x*3.85f,4.68f,-2.2f},{x*3.85f,4.68f,1.38f},.11f,metal);
        for(int i=0;i<5;++i)
            tanks.beam({x*4.78f,3.0f+i*.20f,-3.64f},{x*4.78f,3.0f+i*.20f,-2.6f},.08f,blue);
    }

    const int aboard=lander_.phase<LanderPhase::DeployingRobots?lander_.builders+lander_.carriers:0;
    for(int i=0;i<aboard;++i){
        const auto berth=lunar_lander_robot_berth(i);
        const float x=berth.x,z=berth.z;
        frame.box({x,2.78f,z},{.75f,.37f,.65f},white);
        for(float side:{-.45f,.45f}) for(float dz:{-.23f,.23f})
            frame.cylinder({x+side,2.55f,z+dz},.13f,.25f,dark);
    }

    // Raised tapered command cabin, freight trays and a parabolic antenna.
    const Vec3 cabin_lower[]={{-1.65f,4.91f,.52f},{1.65f,4.91f,.52f},{1.65f,4.91f,-2.29f},{-1.65f,4.91f,-2.29f}};
    const Vec3 cabin_upper[]={{-1.16f,6.17f,-.13f},{1.16f,6.17f,-.13f},{1.16f,6.17f,-1.95f},{-1.16f,6.17f,-1.95f}};
    for(int i=0;i<4;++i){
        const int next=(i+1)%4;
        top.quad(white,cabin_lower[i],cabin_lower[next],cabin_upper[next],cabin_upper[i]);
    }
    for(float x:{-.72f,.72f}) top.quad(blue,{x-.48f,5.32f,.31f},{x+.48f,5.32f,.31f},{x+.42f,5.92f,-.01f},{x-.42f,5.92f,-.01f});
    for(float x:{-1.f,1.f}){
        const float rail=x*4.72f;
        for(float z:{-2.8f,-.8f,1.3f}) top.beam({rail,4.63f,z},{rail,5.25f,z},.05f,gold);
        top.beam({rail,5.25f,-2.8f},{rail,5.25f,1.3f},.07f,gold);
        top.beam({x*2.45f,4.86f,-2.8f},{x*4.72f,4.63f,-2.8f},.18f,metal);
        if(!lander_.remaining.empty()) for(float z:{-1.83f,.11f}) crate(top,{x*3.4f,4.83f,z},gold);
    }
    top.beam({.8f,6.18f,-1.28f},{.8f,7.42f,-1.28f},.085f,white);
    top.cylinder({.8f,7.35f,-1.28f},.26f,.41f,white,.94f);
    top.beam({.8f,7.76f,-1.28f},{.8f,8.15f,-1.28f},.045f,blue);
    top.beam({-1.07f,6.15f,-1.72f},{-1.07f,7.65f,-1.72f},.055f,white);

    // Engine cluster and six visibly articulated struts with wide pads.
    for(float x:{-1.48f,0.f,1.48f}){
        gear.cylinder({x,.40f,-1.48f},.59f,1.48f,metal,.28f);
        gear.cylinder({x,.49f,-1.48f},.62f,.14f,white);
    }
    for(float x:{-1.f,1.f}) for(float z:{-1.f,0.f,1.f}){
        const Vec3 root{x*4.19f,2.45f,z*2.53f-.33f};
        const Vec3 foot{x*(4.0f+.83f*flight.gear),1.3f*(1-flight.gear)+.16f,z*(3.18f+.90f*flight.gear)-.22f};
        const Vec3 knee=root+(foot-root)*.48f;
        gear.beam(root,knee,.33f,white);
        gear.beam(knee,foot,.19f,metal);
        gear.beam(root+Vec3{-x*.72f,-.38f,.25f},knee,.13f,white);
        gear.cylinder(foot-Vec3{0,.16f,0},.57f,.16f,gold,.45f);
        gear.cylinder(knee,.20f,.14f,gold);
    }
    for(std::size_t port=0;port<kLanderSalvagePortCount;++port){
        const Vec3 focus=lunar_lander_salvage_focus(lander_,port)-lander_.site;
        gear.box(focus,{.26f,.36f,.31f},gold);
        gear.beam(focus,{focus.x*.77f,1.96f,focus.z*.75f},.14f,metal);
    }

    Mesh& m=frame;
    // Two hinged ramp sections fold back onto each other when stowed.
    float ramp=0;
    if((int)lander_.phase>(int)LanderPhase::OpeningRamp)ramp=1;
    else if(lander_.phase==LanderPhase::OpeningRamp)ramp=smooth(lander_.elapsed/kLanderRampSeconds);
    constexpr float half_length=3.7f;
    const float a=(90.0f-108.925f*ramp)*pi/180,b=(-90.0f+71.075f*ramp)*pi/180;
    Vec3 hinge{0,2.4f,3.6f},mid=hinge+Vec3{0,half_length*std::sin(a),half_length*std::cos(a)},tip=mid+Vec3{0,half_length*std::sin(b),half_length*std::cos(b)};
    for(const auto seg:std::array<std::array<Vec3,2>,2>{{{{hinge,mid}},{{mid,tip}}}}){
        Vec3 side{2.0f,0,0};m.quad(metal,seg[0]-side,seg[1]-side,seg[1]+side,seg[0]+side);
        for(float x:{-2.02f,2.02f})m.beam(seg[0]+Vec3{x,.08f,0},seg[1]+Vec3{x,.08f,0},.09f,gold);
        for(int i=0;i<12;++i){Vec3 p=seg[0]+(seg[1]-seg[0])*((float)i/12);m.beam(p+Vec3{-1.92f,.028f,0},p+Vec3{1.92f,.028f,0},.04f,white);}
    }
    // Side crane with a pivot, telescoping boom and two drop lines.
    const Vec3 pivot{-4.64f,4.77f,-1.92f},elbow{-5.20f,7.00f,-1.98f},head{-8.35f,8.22f,-1.98f};
    top.cylinder({pivot.x,4.54f,pivot.z},.47f,.48f,gold);
    top.beam(pivot,elbow,.37f,white);
    top.beam(elbow,head,.32f,white);
    top.beam(elbow+Vec3{.15f,.18f,.17f},head+Vec3{0,.18f,.17f},.10f,gold);
    top.beam(pivot+Vec3{.28f,-.08f,.25f},elbow+Vec3{.35f,-.14f,.25f},.13f,metal);
    top.cylinder(head-Vec3{0,.16f,0},.20f,.32f,gold);
    for(float z:{-.12f,.12f}) top.beam(head+Vec3{0,0,z},head+Vec3{0,-1.7f,z},.025f,metal);
    top.box(head+Vec3{0,-1.74f,0},{.24f,.22f,.34f},gold);

    const float work=lander_.salvage_seconds;
    const auto draw_module=[&](const Mesh& module,float removal){
        const float lift=smooth((work-(removal-1.4f))/1.4f);
        if(lift<1) module.draw(r,origin+Vec3{0,.48f*lift,0},flight.roll_degrees,1-lift);
    };
    draw_module(top,4.5f);
    draw_module(tanks,9.0f);
    draw_module(frame,13.5f);
    // Keep the last recovery frame visible while waiting for a place for its parts.
    const float last_cut=smooth((work-16.0f)/2.0f);
    gear.draw(r,origin-Vec3{0,.24f*last_cut,0},flight.roll_degrees,1-.72f*last_cut);
    if(lander_.in_flight()){
        // Vacuum exhaust has no smoke column. Dust follows low lunar-gravity
        // ballistic arcs only when the plume reaches the regolith.
        Mesh exhaust;
        const float pulse=.92f+.08f*std::sin(lander_.elapsed*31);
        const float plume=std::min(origin.y+.55f,2.5f+1.8f*flight.thrust_acceleration)*pulse;
        exhaust.cylinder({0,.55f-plume,-.6f},1.5f*plume/5.0f,plume,{.18f,.44f,.75f,.27f},.55f);
        exhaust.cylinder({0,.55f-plume*.6f,-.6f},.45f,plume*.6f,{.55f,.80f,1,.60f},.36f);
        exhaust.draw(r,origin,flight.roll_degrees);
        if(origin.y<16){
            Mesh dust;
            for(int i=0;i<42;++i){
                const float angle=(float)i*2.399963f,age=std::fmod(lander_.elapsed*.85f+(float)i*.071f,1.1f);
                const float speed=3.0f+((float)(i%5))*.6f;
                const float radial=1+age*speed,height=std::max(.03f,1.1f*age-.81f*age*age);
                Vec3 p=lander_.site+Vec3{std::cos(angle)*radial,height,std::sin(angle)*radial};
                dust.box(p,{.065f,.045f,.13f},{.44f,.43f,.40f,.55f*(1-origin.y/16)});
            }
            dust.draw(r,{});
        }
    }
}
} // namespace game
