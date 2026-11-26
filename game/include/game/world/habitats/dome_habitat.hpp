#pragma once
#include "game/world/habitats/habitat_interior_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"
namespace game::habitats {
inline void ellipse_layer(std::vector<Vec3>& dst,const std::vector<Vec3>& src,float rx,float rz){for(auto p:src){p.x*=rx;p.z*=rz;dst.push_back(p);}}
inline void build_dome_habitat(Builder& b,HabitatKind kind,float floor){
    // Keep inherited placement bounds and docking elevations. The roof is a
    // doubly curved ellipsoid above a short pressure drum, never a barrel.
    constexpr float widths[]{10,10.5f,11,13.5f,14,17,18,21,13,15};
    constexpr float depths[]{5.8f,10,10.5f,9.5f,7,9,10,12,11,8};
    const int k=int(kind);auto& m=b.mesh;m.footprint_x=widths[k];m.footprint_z=depths[k];m.walkable_floor_y=floor;
    const float rx=kind==HabitatKind::AirLock?4.8f:widths[k]*.445f;
    const float rz=kind==HabitatKind::AirLock?2.5f:depths[k]*.445f;
    const float rise=kind==HabitatKind::AirLock?1.25f:kind==HabitatKind::OxygenGeneratorDome?2.3f:std::clamp(std::min(rx,rz)*.62f,1.8f,3.2f);
    const float spring=floor+2.90f;m.dome_radii={rx,rise,rz};m.dome_spring_y=spring;m.height=spring+rise+.20f;
    Builder skin;skin.cylinder_between(skin.mesh.shell,{0,floor-.10f,0},{0,spring,0},1,64,false,false);
    ellipse_layer(m.shell,skin.mesh.shell,rx,rz);
    Builder cap;cap.dome(cap.mesh.shell,{0,0,0},1,14,64,false,1);
    for(auto p:cap.mesh.shell)m.shell.push_back({p.x*rx,spring+p.y*rise,p.z*rz});
    Builder foundation;foundation.cylinder_y(foundation.mesh.foundation,{0,(floor-.10f)*.5f,0},floor-.10f,1,64,false);
    ellipse_layer(m.foundation,foundation.mesh.foundation,rx+.12f,rz+.12f);
    Builder deck;deck.cylinder_y(deck.mesh.interior,{0,floor-.08f,0},.16f,1,64,false);ellipse_layer(m.interior,deck.mesh.interior,rx-.12f,rz-.12f);
    if(kind==HabitatKind::AirLock){
        // The controller still owns four pressure seals and three cabins.
        // Only the outer envelope is replaced by a domed pressure volume.
        for(float x:{-1.6f,1.6f}){
            const float section=std::sqrt(1-x*x/(rx*rx));std::vector<Vec3> rim{{x,floor,rz*section}};
            for(int j=0;j<=32;++j){const float t=PI*j/32;rim.push_back({x,spring+rise*section*std::sin(t),rz*section*std::cos(t)});}
            rim.push_back({x,floor,-rz*section});
            for(float face:{-1.f,1.f}){
                const Vec3 center{x+face*.065f,(floor+spring)*.5f,0};
                for(std::size_t j=0;j<rim.size();++j){auto a=rim[j],c=rim[(j+1)%rim.size()];a.x+=face*.065f;c.x+=face*.065f;
                    m.hull_frames.insert(m.hull_frames.end(),{center,a,c});}
            }
        }
    }
    if(kind==HabitatKind::CrewQuarters){
        constexpr std::array<Vec2,8> bays{{{-1.08f,-2.10f},{1.08f,2.10f},{-1.08f,2.10f},{1.08f,-2.10f},{-3.30f,-2.10f},{3.30f,2.10f},{-3.30f,2.10f},{3.30f,-2.10f}}};
        for(const auto& bay:bays)m.crew_bunks.push_back({{bay.x,floor,std::copysign(.42f,bay.y)},{bay.x-.78f,1.59f,bay.y}});
    }
    if(kind==HabitatKind::AirLock)
        for(float sign:{-1.f,1.f})m.entrances.push_back({{sign*rx,floor,0},{sign,0,0}});
    if(kind!=HabitatKind::AirLock)interior_detail::console(b,{0,floor+.34f,rz*.85f},.90f,-1.f);
    if(kind==HabitatKind::BioDome)m.blocked_grow_bays=0xff;
    // Perimeter services and lights leave the central circulation core clear.
    const float service_height=kind==HabitatKind::OxygenGeneratorDome?3.15f:2.85f;
    for(int i=0;i<24;++i){const float a=2*PI*i/24,c=2*PI*(i+1)/24;
        const Vec3 p{rx*.91f*std::cos(a),floor+service_height,rz*.91f*std::sin(a)},q{rx*.91f*std::cos(c),floor+service_height,rz*.91f*std::sin(c)};
        b.cylinder_between(m.overhead_services,p,q,.038f,8,true,false);
        if(i%3==0)b.cylinder_between(m.overhead_lamps,p+Vec3{0,.12f,0},q+Vec3{0,.12f,0},.020f,8,true,false);
    }
    for(int i=0;i<48;++i){const float a=2*PI*i/48,c=2*PI*(i+1)/48;
        b.cylinder_between(m.markings,{1.30f*std::cos(a),floor+.025f,1.30f*std::sin(a)},{1.30f*std::cos(c),floor+.025f,1.30f*std::sin(c)},.010f,6,true,false);
    }
    // Paint and services follow the same conceptual zones as real habitat
    // studies: accessible circulation, perimeter equipment, overhead utilities.
    m.lining=m.shell;
    for(auto& p:m.lining){p.x*=.982f;p.z*=.982f;}
}
} // namespace game::habitats
