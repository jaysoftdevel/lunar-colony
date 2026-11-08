#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"
#include "game/world/geometry_standards.hpp"

namespace game::resources {
ResourceMesh make_storage_dome(StorageDomeSize size){
    Builder b;
    const float diameter=storage_dome_diameter_m(size);
    const float height=storage_dome_height_m(size);
    const float s=diameter/13.0f;
    const float sy=height/7.0f;
    b.mesh.footprint_x=diameter; b.mesh.footprint_z=diameter; b.mesh.height=height;
    b.mesh.walkable_floor_y=0.68f*sy;

    // Reinforced circular foundation and low warehouse dome.
    b.cylinder_y(b.mesh.foundation,{0,0.34f*sy,0},0.68f*sy,5.75f*s,32,true);
    b.cylinder_y(b.mesh.body,{0,0.78f*sy,0},0.48f*sy,5.35f*s,32,true);
    b.dome(b.mesh.body,{0,0.95f*sy,0},5.20f*s,9,32,true,0.92f*sy/s);

    // Follow the curved pressure skin, leaving the cardinal docking lanes open.
    for(int i=0;i<12;++i){
        const float angle=2.0f*PI*((float)i+0.5f)/12.0f;
        const Vec3 radial{std::cos(angle),0,std::sin(angle)};
        Vec3 previous=radial*(5.27f*s)+Vec3{0,.95f*sy,0};
        for(int segment=1;segment<=9;++segment){
            const float arc=PI*.5f*(float)segment/9.f;
            const Vec3 next=radial*(5.27f*s*std::cos(arc))+
                Vec3{0,(.95f+5.20f*.92f*std::sin(arc))*sy,0};
            b.cylinder_between(b.mesh.details,previous,next,.065f*s,6,false,true);
            previous=next;
        }
    }
    const float wall=5.35f*s;
    for(int side=0;side<4;++side){
        const float a=PI*0.5f*(float)side;
        const Vec3 n{std::sin(a),0.0f,-std::cos(a)};
        const float frame_h=geometry::kDoorFrameHeight;
        const float frame_w=geometry::kDoorClearWidth+0.70f;
        const float frame_d=0.55f;
        const Vec3 c=n*wall+Vec3{0,b.mesh.walkable_floor_y+frame_h*0.5f,0};
        // Docking ports stay human-sized on every dome tier. Increasing storage
        // capacity enlarges the warehouse, not the person-sized doors.
        b.box(b.mesh.details,c,{(side%2==0?frame_w:frame_d),frame_h,(side%2==0?frame_d:frame_w)});
        const Vec3 inner=n*(wall+0.30f)+Vec3{0,b.mesh.walkable_floor_y+geometry::kDoorClearHeight*0.5f,0};
        b.box(b.mesh.details,inner,{(side%2==0?geometry::kDoorClearWidth:0.24f),geometry::kDoorClearHeight,(side%2==0?0.24f:geometry::kDoorClearWidth)});
    }

    // Four docking lanes meet at an unobstructed handling intersection.
    const float deck=b.mesh.walkable_floor_y;
    const float lane=geometry::kDoorClearWidth*0.5f;
    const float lane_end=wall-0.75f;
    for(float side:{-1.0f,1.0f})for(float edge:{-lane,lane}){
        b.line({side*lane,deck+0.01f,edge},{side*lane_end,deck+0.01f,edge});
        b.line({edge,deck+0.01f,side*lane},{edge,deck+0.01f,side*lane_end});
    }

    // Fixed service cabinets stand outside the quadrant cargo grids.
    const float cabinet=0.236f*diameter;
    for(float x:{-cabinet,cabinet})for(float z:{-cabinet,cabinet}){
        b.box(b.mesh.details,{x,deck+0.57f,z},{0.52f,1.14f,0.48f});
        b.box(b.mesh.details,{x,deck+1.19f,z},{0.56f,0.10f,0.52f});
        b.box(b.mesh.details,{x,deck+0.81f,z-std::copysign(0.245f,z)},{0.27f,0.12f,0.025f});
    }
    return std::move(b.mesh);
}
}
