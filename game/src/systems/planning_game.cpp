#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/version.hpp"
#include "game/world/resource_builder.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace game {
using namespace eng;
using namespace layout;

namespace {
constexpr std::uint32_t bit(std::size_t i){ return std::uint32_t{1}<<i; }

enum class CampusStyle : std::uint8_t {
    Administration,
    Research,
    Assembly,
    Cargo,
    MissionControl,
    Communications,
    PropulsionTest,
    LaunchPad
};

struct CampusBuilding {
    const char* name;
    const char* subtitle;
    Vec3 center;
    Vec3 footprint; // x/z footprint; y is nominal building height
    MissionFacility facility;
    CampusStyle style;
};

// The Stage-1 company site is intentionally a compact fictional coastal launch
// campus, inspired by the functional zoning of real launch centers without
// reproducing any specific real facility.  Every structure is gameplay-linked.
constexpr std::array<CampusBuilding,8> kCampusBuildings{{
    {"PROGRAM & CONTRACTS OFFICE","Commercial programs / finance / mission authorization",{-40,0,-24},{18,5,13},MissionFacility::ProgramOffice,CampusStyle::Administration},
    {"RESEARCH & ENGINEERING","Technology research / vehicle design / refinement",{-40,0,  5},{23,6,16},MissionFacility::ResearchEngineering,CampusStyle::Research},
    {"VEHICLE ASSEMBLY BUILDING","Rocket and lander manufacturing / integration",{  5,0, -5},{18,22,22},MissionFacility::VehicleAssembly,CampusStyle::Assembly},
    {"CARGO INTEGRATION FACILITY","Lunar payload processing / manifest loading",{  7,0, 26},{24,7,15},MissionFacility::CargoIntegration,CampusStyle::Cargo},
    {"MISSION CONTROL CENTER","Mission planning / flight profile / launch operations",{-15,0,-27},{18,6,13},MissionFacility::MissionControl,CampusStyle::MissionControl},
    {"COMMUNICATIONS CENTER","Tracking / telemetry / final communications readiness",{-16,0, 22},{17,7,14},MissionFacility::Communications,CampusStyle::Communications},
    {"PROPULSION TEST CENTER","Engine and stage qualification / cryogenic testing",{ 38,0, 31},{20,8,15},MissionFacility::PropulsionTest,CampusStyle::PropulsionTest},
    {"LAUNCH COMPLEX","Pad / service tower / launch execution",{ 42,0, -8},{22,4,22},MissionFacility::LaunchComplex,CampusStyle::LaunchPad}
}};

bool campus_contains(const CampusBuilding& b,Vec3 p){
    return std::abs(p.x-b.center.x)<=b.footprint.x*0.5f &&
           std::abs(p.z-b.center.z)<=b.footprint.z*0.5f;
}


const char* mission_step_name(MissionPreparationStep step){
    switch(step){
        case MissionPreparationStep::Research:return "RESEARCH";
        case MissionPreparationStep::Design:return "DESIGN";
        case MissionPreparationStep::Test:return "TEST";
        case MissionPreparationStep::Refine:return "REFINE";
        case MissionPreparationStep::Plan:return "PLAN";
        case MissionPreparationStep::Confirm:return "CONFIRM";
        case MissionPreparationStep::Build:return "BUILD";
        case MissionPreparationStep::Integrate:return "INTEGRATE";
        case MissionPreparationStep::FinalConfirm:return "FINAL CONFIRM";
        case MissionPreparationStep::Launch:return "LAUNCH";
        case MissionPreparationStep::Complete:return "COMPLETE";
    }
    return "MISSION";
}

const char* mission_facility_name(MissionFacility facility){
    switch(facility){
        case MissionFacility::Overview:return "MISSION OVERVIEW";
        case MissionFacility::ProgramOffice:return "PROGRAM & CONTRACTS OFFICE";
        case MissionFacility::ResearchEngineering:return "RESEARCH & ENGINEERING";
        case MissionFacility::VehicleAssembly:return "VEHICLE ASSEMBLY BUILDING";
        case MissionFacility::CargoIntegration:return "CARGO INTEGRATION FACILITY";
        case MissionFacility::MissionControl:return "MISSION CONTROL CENTER";
        case MissionFacility::Communications:return "COMMUNICATIONS CENTER";
        case MissionFacility::PropulsionTest:return "PROPULSION TEST CENTER";
        case MissionFacility::LaunchComplex:return "LAUNCH COMPLEX";
    }
    return "MISSION FACILITY";
}

MissionFacility mission_step_facility(MissionPreparationStep step){
    switch(step){
        case MissionPreparationStep::Research:
        case MissionPreparationStep::Design:
        case MissionPreparationStep::Refine:return MissionFacility::ResearchEngineering;
        case MissionPreparationStep::Test:return MissionFacility::PropulsionTest;
        case MissionPreparationStep::Plan:return MissionFacility::MissionControl;
        case MissionPreparationStep::Confirm:return MissionFacility::ProgramOffice;
        case MissionPreparationStep::Build:return MissionFacility::VehicleAssembly;
        case MissionPreparationStep::Integrate:return MissionFacility::CargoIntegration;
        case MissionPreparationStep::FinalConfirm:return MissionFacility::Communications;
        case MissionPreparationStep::Launch:return MissionFacility::LaunchComplex;
        case MissionPreparationStep::Complete:return MissionFacility::Overview;
    }
    return MissionFacility::Overview;
}

void draw_campus_builder(Renderer& r,const resources::Builder& b,Color body,Color edge={0.08f,0.11f,0.13f,1.0f}){
    if(!b.mesh.foundation.empty()) r.draw_triangles_3d(b.mesh.foundation,{body.r*0.72f,body.g*0.72f,body.b*0.72f,body.a});
    if(!b.mesh.body.empty()) r.draw_triangles_3d(b.mesh.body,body);
    if(!b.mesh.details.empty()) r.draw_triangles_3d(b.mesh.details,{std::min(1.0f,body.r*1.13f),std::min(1.0f,body.g*1.13f),std::min(1.0f,body.b*1.13f),body.a});
    (void)edge; // Coplanar wire edges shimmer on rotation; use solid facets.
}

void campus_face(Renderer& r,Vec3 a,Vec3 b,Vec3 c,Vec3 d,Color color,int material=-1){
    const std::vector<Vec3> vertices{a,b,c,a,c,d};
    if(material<0) r.draw_triangles_3d(vertices,color);
    else r.draw_material_triangles_3d(vertices,material,color);
}

void campus_box(Renderer& r,Vec3 center,Vec3 size,Color color,int wall_material=6){
    // All six normals point outwards, important for stable directional lighting
    // and world-aligned 1024px material textures (no overlapping wire edges).
    const Vec3 lo=center-size*0.5f,hi=center+size*0.5f;
    auto tone=[&](float f){return Color{std::min(1.f,color.r*f),std::min(1.f,color.g*f),std::min(1.f,color.b*f),color.a};};
    campus_face(r,{lo.x,hi.y,lo.z},{lo.x,hi.y,hi.z},{hi.x,hi.y,hi.z},{hi.x,hi.y,lo.z},tone(1.08f),5);
    campus_face(r,{lo.x,lo.y,lo.z},{hi.x,lo.y,lo.z},{hi.x,lo.y,hi.z},{lo.x,lo.y,hi.z},tone(0.58f),wall_material);
    campus_face(r,{lo.x,lo.y,hi.z},{hi.x,lo.y,hi.z},{hi.x,hi.y,hi.z},{lo.x,hi.y,hi.z},tone(1.02f),wall_material);
    campus_face(r,{hi.x,lo.y,lo.z},{lo.x,lo.y,lo.z},{lo.x,hi.y,lo.z},{hi.x,hi.y,lo.z},tone(0.84f),wall_material);
    campus_face(r,{lo.x,lo.y,lo.z},{lo.x,lo.y,hi.z},{lo.x,hi.y,hi.z},{lo.x,hi.y,lo.z},tone(0.93f),wall_material);
    campus_face(r,{hi.x,lo.y,hi.z},{hi.x,lo.y,lo.z},{hi.x,hi.y,lo.z},{hi.x,hi.y,hi.z},tone(0.89f),wall_material);
}

void campus_material_box(Renderer& r,Vec3 center,Vec3 size,int material,Color tint){
    resources::Builder b;
    b.box(b.mesh.body,center,size,true);
    r.draw_material_triangles_3d(b.mesh.body,material,tint);
}

void campus_shadow(Renderer& r,Vec3 center,float sx,float sz,float height){
    // Projected geometric shadow. The apparent sun comes from upper-left;
    // shadows are grounded on the terrain, including tall launch towers.
    const float dx=height*0.44f, dz=height*0.28f;
    const float x0=center.x-sx*0.5f,z0=center.z-sz*0.5f;
    const float x1=center.x+sx*0.5f,z1=center.z+sz*0.5f;
    const Vec3 a{x0,0.091f,z0},b{x1,0.091f,z0},c{x1+dx,0.091f,z1+dz},d{x0+dx,0.091f,z1+dz};
    campus_face(r,a,b,c,d,{0.012f,0.023f,0.034f,0.42f});
}

void campus_cylinder(Renderer& r,Vec3 center,float height,float radius,Color color,int segments);

void campus_tree(Renderer& r,float x,float z,float scale){
    // Native coastal shrub/oak silhouette: thin trunk and two overlapping
    // crown tiers; its shadow shares the same global sunlight direction.
    campus_shadow(r,{x,0,z},1.7f*scale,1.6f*scale,3.8f*scale);
    campus_cylinder(r,{x,1.2f*scale,z},2.4f*scale,0.18f*scale,{0.28f,0.24f,0.18f,1},7);
    campus_cylinder(r,{x,3.0f*scale,z},2.4f*scale,1.12f*scale,{0.20f,0.37f,0.23f,1},9);
    campus_cylinder(r,{x,4.0f*scale,z},1.0f*scale,0.78f*scale,{0.27f,0.44f,0.27f,1},8);
}

void campus_daylight_sky(Renderer& r){
    // Full-screen atmosphere gradient before the perspective scene. This is a
    // color field, not another coplanar world polygon; hence no depth fights.
    const float w=(float)r.width(), h=(float)r.height();
    r.set_2d(1.0f);
    for(int band=0;band<28;++band){
        const float t=(band+0.5f)/28.0f;
        const float a=std::clamp(std::pow(t,0.88f),0.0f,1.0f);
        const Color zenith{0.25f,0.48f,0.70f,1.0f};
        const Color horizon{0.79f,0.85f,0.84f,1.0f};
        const Color c{zenith.r+(horizon.r-zenith.r)*a,
                      zenith.g+(horizon.g-zenith.g)*a,
                      zenith.b+(horizon.b-zenith.b)*a,1.0f};
        const float y=h*(float)band/28.0f;
        r.draw_rect({0,y,w,std::min(h-y,h/28.0f+0.5f)},c);
    }
}

void campus_detail_road_sign(Renderer& r,float x,float z){
    campus_box(r,{x,1.5f,z},{0.1f,2.9f,0.1f},{0.38f,0.41f,0.39f,1},8);
    campus_box(r,{x,2.85f,z},{0.15f,0.8f,1.35f},{0.27f,0.46f,0.46f,1},8);
    campus_box(r,{x+0.12f,2.8f,z},{0.05f,0.35f,1.08f},{0.88f,0.88f,0.83f,1},8);
}

void campus_detail_floodlight(Renderer& r,float x,float z,float height){
    campus_box(r,{x,height*0.50f,z},{0.16f,height,0.16f},{0.57f,0.60f,0.58f,1},8);
    campus_box(r,{x,height+0.14f,z},{1.5f,0.15f,0.24f},{0.60f,0.61f,0.60f,1},8);
    for(float dx:{-0.52f,0.52f})
        campus_box(r,{x+dx,height+0.08f,z-0.19f},{0.45f,0.30f,0.22f},{0.87f,0.91f,0.84f,1},8);
}

void campus_detail_service_vehicle(Renderer& r,float x,float z,bool cargo){
    const Color body=cargo?Color{0.64f,0.70f,0.70f,1}:Color{0.76f,0.74f,0.63f,1};
    campus_box(r,{x,0.80f,z},{cargo?4.0f:2.8f,1.2f,1.9f},body,8);
    campus_box(r,{x+(cargo?1.5f:0.84f),1.25f,z},{cargo?1.6f:1.1f,1.2f,1.75f},body,8);
    campus_box(r,{x+(cargo?2.34f:1.39f),1.47f,z},{0.08f,0.60f,1.3f},{0.10f,0.23f,0.32f,1},7);
    for(float sx:{-1.1f,1.05f})for(float sz:{-1.02f,1.02f})
        campus_cylinder(r,{x+sx,0.43f,z+sz},0.52f,0.39f,{0.095f,0.10f,0.11f,1},8);
}

void campus_detail_roof_equipment(Renderer& r,Vec3 p,float roof_y){
    const Color silver{0.66f,0.70f,0.69f,1};
    campus_box(r,{p.x-3,roof_y+0.68f,p.z-2.8f},{2.6f,1.2f,2.0f},silver,8);
    campus_box(r,{p.x+2.3f,roof_y+0.42f,p.z-2.5f},{3.0f,0.70f,1.2f},{0.42f,0.50f,0.51f,1},8);
    for(int i=0;i<6;++i) campus_box(r,{p.x-4.15f+0.42f*i,roof_y+0.69f,p.z-2.8f},
                                  {0.08f,0.68f,1.95f},{0.25f,0.33f,0.33f,1},8);
    campus_box(r,{p.x+0.15f,roof_y+0.68f,p.z+2.5f},{0.24f,1.4f,0.24f},silver,8);
}

void campus_dune_geometry(Renderer& r){
    // Sloped terrain mesh, not an overlapping box, along the mainland shore.
    for(int segment=0;segment<28;++segment){
        const float za=-110.0f+segment*8.0f,zb=za+8.0f;
        const float phase=0.35f*std::sin(za*0.075f)+0.14f*std::cos(za*0.19f);
        const float phase2=0.35f*std::sin(zb*0.075f)+0.14f*std::cos(zb*0.19f);
        campus_face(r,{55,0.14f+phase,za},{55,0.14f+phase2,zb},
                      {62,0.17f+phase2*0.4f,zb},{62,0.17f+phase*0.4f,za},
                      {0.77f,0.69f,0.48f,1},1);
        campus_face(r,{62,0.17f+phase*0.4f,za},{62,0.17f+phase2*0.4f,zb},
                      {68,0.078f,zb},{68,0.078f,za},{0.81f,0.73f,0.56f,1},1);
    }
}

void campus_service_details(Renderer& r){
    campus_detail_roof_equipment(r,{-40,0,5},4.45f);
    campus_detail_roof_equipment(r,{-15,0,-27},5.1f);
    campus_detail_roof_equipment(r,{7,0,26},6.2f);
    campus_detail_service_vehicle(r,-51.0f,-39.4f,false);
    campus_detail_service_vehicle(r,-44.0f,-39.4f,false);
    campus_detail_service_vehicle(r,7.0f,41.0f,true);
    campus_detail_service_vehicle(r,16.0f,41.0f,true);
    for(const auto& pos:std::array<Vec2,8>{{{-58,-9},{-58,36},{-30,39},{4,45},{34,-28},{55,-32},{55,33},{27,49}}})
        campus_detail_floodlight(r,pos.x,pos.y,7.0f);
    campus_detail_road_sign(r,-20.0f,12.0f);
    campus_detail_road_sign(r,32.0f,12.0f);
    // Additional umbilical conduits and weather-protected fueling lines.
    campus_box(r,{49.8f,1.4f,-8.0f},{0.55f,2.8f,15.5f},{0.48f,0.56f,0.54f,1},8);
    for(int i=0;i<6;++i){
        const float z=-15.0f+2.6f*i;
        campus_box(r,{49.8f,1.3f,z},{1.2f,0.09f,0.09f},{0.70f,0.71f,0.65f,1},8);
    }
    // Painted exclusion and safety stripes next to the test facility.
    for(int i=0;i<12;++i){
        float z=19.0f+2.2f*i;
        campus_material_box(r,{52.5f,0.090f,z},{0.28f,0.006f,1.2f},3,{0.82f,0.75f,0.32f,1});
    }
}

void render_campus_landscape(Renderer& r){
    // Fixed-size gameplay campus (128 x 128); the view's visual coastline
    // extends past the interactable parcel for a natural oblique horizon.
    campus_material_box(r,{0,-0.24f,0},{232,0.35f,224},1,{0.51f,0.43f,0.27f,1});
    campus_material_box(r,{-18,-0.065f,0},{156,0.16f,175},0,{0.21f,0.32f,0.17f,1});
    campus_material_box(r,{61,0.008f,0},{18,0.13f,224},1,{0.67f,0.60f,0.39f,1});
    campus_material_box(r,{103,-0.025f,0},{70,0.12f,230},4,{0.10f,0.34f,0.45f,1});
    campus_material_box(r,{76,0.078f,0},{3,0.008f,224},1,{0.74f,0.73f,0.52f,1.0f});
    campus_dune_geometry(r);
    // Roads, parking aprons and operational clear zones.
    constexpr Color asphalt{0.24f,0.27f,0.27f,1};
    constexpr Color concrete{0.44f,0.46f,0.43f,1};
    campus_material_box(r,{0,0.015f,10},{115,0.040f,5.0f},2,asphalt);
    campus_material_box(r,{-26,0.018f,-11},{5.3f,0.045f,73},2,asphalt);
    campus_material_box(r,{25,0.018f,-7},{57,0.045f,5.3f},2,asphalt);
    campus_material_box(r,{23,0.018f,29},{58,0.045f,5.0f},2,asphalt);
    campus_material_box(r,{44,0.018f,10},{4.5f,0.045f,43},2,asphalt);
    campus_material_box(r,{44,0.023f,-8},{30,0.052f,29},3,concrete);
    campus_material_box(r,{40,0.022f,31},{34,0.045f,23},3,concrete);
    campus_material_box(r,{-40,0.023f,-24},{28,0.045f,22},3,concrete);
    campus_material_box(r,{5,0.023f,-5},{26,0.045f,29},3,concrete);
    campus_material_box(r,{9,0.025f,26},{29,0.044f,21},3,concrete);
    // Lanes and crosswalks: physical scale gives the location a credible
    // spaceport/industrial-campus reading even at distant overview zoom.
    for(int i=0;i<20;++i){
        float x=-54.0f+(float)i*5.4f;
        campus_material_box(r,{x,0.048f,10},{2.25f,0.011f,0.11f},3,{0.83f,0.78f,0.53f,1});
    }
    for(int i=0;i<7;++i){
        float z=-45.0f+(float)i*7.7f;
        campus_material_box(r,{-26,0.047f,z},{0.10f,0.011f,3.0f},3,{0.86f,0.81f,0.56f,1});
    }
    // Parking stripes, fenced machinery staging and service fuel tanks.
    for(int row=0;row<2;++row)for(int col=0;col<5;++col){
        float x=-57.0f+(float)col*2.75f,z=-42.0f+(float)row*4.2f;
        campus_material_box(r,{x,0.045f,z},{0.12f,0.010f,3.4f},3,{0.67f,0.70f,0.60f,1});
    }
    for(int i=0;i<3;++i){
        const float z=49.0f+(float)i*5.5f;
        campus_cylinder(r,{30.0f,2.1f,z},4.2f,1.55f,{0.73f,0.72f,0.67f,1},16);
        campus_box(r,{30.0f,4.3f,z},{3.7f,0.32f,3.7f},{0.78f,0.78f,0.72f,1});
    }
    // Clustered coastal greenery without obstructing clickable footprints.
    for(int i=0;i<36;++i){
        const float x=-74.0f+float((i*37)%127), z=-78.0f+float((i*53)%147);
        bool clear=true;
        for(const auto& b:kCampusBuildings){
            if(std::abs(x-b.center.x)<b.footprint.x*0.5f+6.0f &&
               std::abs(z-b.center.z)<b.footprint.z*0.5f+6.0f)clear=false;
        }
        if(x>50.0f || (std::abs(z-10.0f)<4.0f && x>-59.0f) || std::abs(x+26.0f)<5.0f)clear=false;
        if(clear)campus_tree(r,x,z,0.68f+(float)(i%4)*0.13f);
    }
    // Security perimeter around hazardous launch complex, with spaced masts.
    for(int i=0;i<=12;++i){
        const float z=-34.0f+(float)i*4.5f;
        campus_box(r,{62,1.4f,z},{0.14f,2.7f,0.14f},{0.52f,0.58f,0.54f,1});
    }
    for(int i=0;i<10;++i){
        float z=-42.0f+(float)i*9.1f;
        campus_box(r,{-62.0f,3.25f,z},{0.18f,6.4f,0.18f},{0.43f,0.52f,0.53f,1});
        campus_box(r,{-61.4f,6.3f,z},{1.4f,0.16f,0.45f},{0.79f,0.78f,0.63f,1});
    }
}

void campus_cylinder(Renderer& r,Vec3 center,float height,float radius,Color color,int segments=16){
    resources::Builder b;
    b.cylinder_y(b.mesh.body,center,height,radius,segments,true);
    draw_campus_builder(r,b,color);
}

void render_campus_building(Renderer& r,const CampusBuilding& c,bool hovered,bool required){
    const Color neutral=hovered?Color{0.30f,0.63f,0.70f,1.0f}:required?Color{0.36f,0.57f,0.37f,1.0f}:Color{0.31f,0.39f,0.43f,1.0f};
    const Color light=hovered?Color{0.40f,0.73f,0.78f,1.0f}:required?Color{0.49f,0.68f,0.48f,1.0f}:Color{0.43f,0.48f,0.49f,1.0f};
    const Color dark=hovered?Color{0.22f,0.50f,0.58f,1.0f}:required?Color{0.25f,0.45f,0.27f,1.0f}:Color{0.22f,0.28f,0.31f,1.0f};
    const Vec3 p=c.center;
    switch(c.style){
        case CampusStyle::Administration:
            campus_box(r,{p.x,1.6f,p.z},{c.footprint.x,3.2f,c.footprint.z},neutral);
            campus_box(r,{p.x-4.8f,4.0f,p.z},{6.0f,1.7f,8.0f},light);
            campus_box(r,{p.x,1.6f,p.z+6.55f},{3.4f,2.5f,0.12f},{0.12f,0.24f,0.30f,1});
            for(int i=0;i<6;++i)campus_box(r,{p.x-7.0f+2.65f*i,2.45f,p.z+6.59f},{1.85f,0.95f,0.08f},{0.17f,0.36f,0.42f,1},7);
            break;
        case CampusStyle::Research:
            campus_box(r,{p.x,2.2f,p.z},{c.footprint.x,4.4f,c.footprint.z},neutral);
            campus_box(r,{p.x+5.7f,5.7f,p.z-1.0f},{7.5f,2.6f,8.0f},light);
            for(float x:{p.x-7.0f,p.x,p.x+7.0f}) campus_cylinder(r,{x,5.0f,p.z+5.2f},1.7f,1.25f,dark,12);
            for(int i=0;i<8;++i)campus_box(r,{p.x-9.7f+2.7f*i,2.5f,p.z+8.04f},{1.92f,1.65f,0.12f},{0.13f,0.31f,0.41f,1},7);
            campus_box(r,{p.x,5.12f,p.z-4.4f},{10,0.3f,4},{0.63f,0.62f,0.59f,1});
            break;
        case CampusStyle::Assembly:
            campus_box(r,{p.x,10.5f,p.z},{c.footprint.x,21.0f,c.footprint.z},neutral);
            campus_box(r,{p.x+6.2f,15.7f,p.z-5.2f},{4.2f,10.0f,8.0f},light);
            campus_box(r,{p.x,5.0f,p.z-11.8f},{11.0f,8.0f,3.0f},dark);
            // Vertical sliding assembly doors, repeated roof skylights.
            for(int i=0;i<8;++i){
                const float x=p.x-6.6f+1.90f*i;
                campus_box(r,{x,6.5f,p.z+11.09f},{1.32f,12.6f,0.15f},{0.16f,0.25f,0.29f,1},7);
            }
            for(int i=0;i<4;++i)campus_box(r,{p.x-5.0f+i*3.4f,21.18f,p.z+3.0f},{1.6f,0.10f,11.5f},{0.30f,0.48f,0.56f,1});
            break;
        case CampusStyle::Cargo:
            campus_box(r,{p.x,3.0f,p.z},{c.footprint.x,6.0f,c.footprint.z},neutral);
            for(float x:{p.x-7.0f,p.x,p.x+7.0f}) campus_box(r,{x,1.2f,p.z-8.0f},{5.0f,2.4f,3.0f},light);
            for(int i=0;i<5;++i)campus_box(r,{p.x-9.0f+i*4.3f,2.1f,p.z+7.56f},{3.1f,3.5f,0.18f},{0.15f,0.29f,0.35f,1},7);
            for(int i=0;i<4;++i)campus_box(r,{p.x-8.2f+5.3f*i,0.53f,p.z+12.0f},{2.3f,1.0f,2.3f},{0.53f,0.42f,0.24f,1});
            break;
        case CampusStyle::MissionControl:
            campus_box(r,{p.x,2.5f,p.z},{c.footprint.x,5.0f,c.footprint.z},neutral);
            campus_box(r,{p.x,5.2f,p.z+1.0f},{11.0f,1.0f,7.0f},light);
            for(int i=0;i<7;++i)campus_box(r,{p.x-7.3f+i*2.4f,3.45f,p.z+6.58f},{1.8f,1.5f,0.13f},{0.14f,0.36f,0.44f,1},7);
            break;
        case CampusStyle::Communications:{
            campus_box(r,{p.x,2.0f,p.z},{c.footprint.x,4.0f,c.footprint.z},neutral);
            resources::Builder dish;
            dish.cylinder_y(dish.mesh.body,{p.x,5.0f,p.z},3.0f,0.45f,10,true);
            dish.dome(dish.mesh.details,{p.x,6.3f,p.z},3.6f,5,18,true,0.32f);
            draw_campus_builder(r,dish,light);
            for(int i=0;i<5;++i)campus_box(r,{p.x-6.1f+i*3.0f,2.2f,p.z+7.06f},{1.9f,1.2f,0.11f},{0.13f,0.34f,0.42f,1},7);
            campus_box(r,{p.x+6.1f,6.4f,p.z-4.0f},{0.22f,5.0f,0.22f},{0.56f,0.59f,0.59f,1});
            campus_cylinder(r,{p.x+6.1f,9.0f,p.z-4.0f},0.25f,1.15f,{0.82f,0.83f,0.77f,1},16);
            break;
        }
        case CampusStyle::PropulsionTest:
            campus_box(r,{p.x,1.2f,p.z},{c.footprint.x,2.4f,c.footprint.z},dark);
            for(float x:{p.x-5.3f,p.x,p.x+5.3f}){
                campus_cylinder(r,{x,4.3f,p.z},5.8f,1.25f,neutral,14);
                campus_box(r,{x,7.8f,p.z},{3.6f,1.2f,3.6f},light);
                campus_box(r,{x,1.05f,p.z+5.6f},{3.1f,2.1f,1.0f},{0.46f,0.30f,0.20f,1});
            }
            break;
        case CampusStyle::LaunchPad:{
            campus_cylinder(r,{p.x,0.45f,p.z},0.9f,10.0f,dark,24);
            // Rocket stack and a simple service tower make the launch function
            // readable even at the default isometric camera distance.
            campus_cylinder(r,{p.x,8.0f,p.z},14.0f,1.35f,Color{0.67f,0.70f,0.69f,1.0f},18);
            campus_cylinder(r,{p.x,15.8f,p.z},2.0f,0.82f,Color{0.75f,0.77f,0.75f,1.0f},16);
            campus_box(r,{p.x+6.3f,8.2f,p.z},{2.2f,16.4f,2.2f},neutral);
            for(float y:{3.0f,7.0f,11.0f,15.0f}) campus_box(r,{p.x+3.8f,y,p.z},{5.0f,0.35f,0.5f},light);
            for(float y:{4.0f,8.0f,12.0f}){
                campus_box(r,{p.x+6.3f,y,p.z+0.8f},{2.6f,0.22f,0.22f},{0.72f,0.72f,0.70f,1});
                campus_box(r,{p.x+6.3f,y,p.z-0.8f},{2.6f,0.22f,0.22f},{0.72f,0.72f,0.70f,1});
            }
            campus_cylinder(r,{p.x-4.5f,2.5f,p.z-5.0f},5.0f,1.4f,{0.69f,0.70f,0.66f,1},16);
            campus_cylinder(r,{p.x-1.2f,2.5f,p.z-6.2f},5.0f,1.1f,{0.69f,0.70f,0.66f,1},16);
            campus_box(r,{p.x,0.25f,p.z},{11,0.45f,11},{0.24f,0.29f,0.29f,1});
            break;
        }
    }
}

bool project_campus_label(const Mat4& m,Vec3 p,int viewport_width,int viewport_height,float ui_scale,float& sx,float& sy){
    const float cx=m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12];
    const float cy=m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13];
    const float cw=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];
    if(cw<=0.001f) return false;
    const float nx=cx/cw,ny=cy/cw;
    sx=((nx*0.5f+0.5f)*(float)viewport_width)/std::max(0.001f,ui_scale);
    sy=((1.0f-(ny*0.5f+0.5f))*(float)viewport_height)/std::max(0.001f,ui_scale);
    return true;
}

struct TechDefinition {
    const char* name;
    const char* short_name;
    const char* unlock;
    std::int64_t cost_eur;
    std::uint32_t prerequisites;
    int level;
    int lane;
};

constexpr std::array<TechDefinition,kPlanningTechnologyCount> kTechs{{
    {"Mission Systems Engineering","SYSTEMS ENGINEERING","Program architecture, interfaces, verification and mission assurance",400'000,0,0,0},
    {"Space Structures & Materials","SPACE STRUCTURES","Vacuum/thermal structures, pressure vessels and lightweight module frames",550'000,0,0,1},
    {"Propulsion Fundamentals","PROPULSION","Liquid propulsion, feed systems, engines and cryogenic handling",750'000,0,0,2},
    {"Avionics, Guidance & Control","AVIONICS + GNC","Flight computers, navigation sensors, control laws and fault tolerance",650'000,bit(0),1,2},

    {"Autonomous Robotics","AUTONOMOUS ROBOTICS","Builder/carrier rover autonomy, safety coordination and remote operations",900'000,bit(0)|bit(3),2,0},
    {"Power Generation & Storage","POWER + STORAGE","Solar generation, batteries, power distribution and surface energy management",850'000,bit(0)|bit(1),1,1},
    {"Orbital Launch Vehicle","ORBITAL LAUNCH VEHICLE","A launch system able to place useful payload into Earth orbit",1'900'000,bit(1)|bit(2)|bit(3),2,2},
    {"Closed-Loop Life Support","LIFE SUPPORT","Atmosphere management, oxygen generation, water recovery and monitoring",1'150'000,bit(0)|bit(1),1,0},

    {"Lunar Transfer & Navigation","LUNAR TRANSFER","Trans-lunar injection, cruise navigation, lunar approach and payload delivery",1'700'000,bit(6)|bit(3),3,2},
    {"Lunar Surface Construction","SURFACE CONSTRUCTION","Deployable foundations, corridors, utility routing and robotic assembly",1'350'000,bit(4)|bit(1),3,0},
    {"ISRU & Water Processing","ISRU + WATER","Regolith prospecting, water extraction, purification and local resource handling",1'450'000,bit(5)|bit(9),4,1},
    {"Pressurized Habitats & Airlocks","HABITATS + AIRLOCKS","Pressure shells, hermetic doors, airlocks and crew-rated internal modules",1'650'000,bit(7)|bit(9),4,0},

    {"Controlled Environment Agriculture","BIO DOME SYSTEMS","Hydroponics, crop lighting, environmental control and food production",1'250'000,bit(5)|bit(7)|bit(11),5,0},
    {"Lunar Industry & Manufacturing","LUNAR INDUSTRY","Processing, factory, manufactory, storage and maintenance competence",1'600'000,bit(5)|bit(9)|bit(10),5,2},
    {"Stage-2 Module Competence","STAGE-2 MODULE COMPETENCE","Engineering competence for every structure and utility used by the robotic base stage",2'500'000,bit(10)|bit(11)|bit(12)|bit(13),6,1},
    {"Robotic Lunar Base Mission Architecture","ROBOTIC LUNAR BASE MISSION","Integrated launch, lunar cargo transfer, autonomous deployment and Stage-2 base preparation",3'200'000,bit(8)|bit(14)|bit(4),7,1}
}};

struct ContractDefinition {
    const char* name;
    const char* customer;
    const char* description;
    std::int64_t deposit_eur;
    std::int64_t finish_eur;
    double duration_days;
    std::uint32_t required_tech;
    int supported_tech; // completed technical evidence reduces the target research cost
    const char* deliverable;
};

// Existing commercial contract IDs 0..10 and their prices remain stable.
// New contracts are inserted before the lunar launch sponsorship, which moves
// from legacy ID 11 to the final slot. Save schema 30 explicitly remaps the
// old sponsorship on load, retaining all previous completion histories.
// All customer work is one-time; the lunar contract settles on launch.
constexpr std::array<ContractDefinition,kPlanningContractCount> kContracts{{
    {"Mission Architecture Study","Commercial Space Consortium","Develop the first real architecture for a reusable space-program baseline.",250'000,550'000,24.0,0,0,"Submit mission mass budget, design interfaces and an assurance report."},
    {"Flight Software & GNC Consulting","Small Satellite Operator","Prove flight navigation software in a customer application.",350'000,800'000,28.0,bit(0)|bit(3),4,"Deliver reviewed flight-control software and autonomy test results."},
    {"Space Materials Qualification","Launch Supplier","Test materials needed for pressure shells and structural modules.",450'000,1'050'000,30.0,bit(1),11,"Deliver thermal, vibration and pressure qualification records."},
    {"Autonomous Rover Prototype","Planetary Robotics Program","Demonstrate the robot logistics required for Stage 2 construction.",650'000,1'550'000,38.0,bit(4),9,"Deliver obstacle-avoidance trials and a deployable rover prototype."},
    {"Surface Energy Demonstrator","Exploration Infrastructure Agency","Prove a power system for the lunar outpost supply network.",750'000,1'850'000,42.0,bit(5),10,"Deliver solar/battery endurance results and distribution design."},
    {"Small Payload Launch Demo","Institutional Launch Customer","Qualify the launch systems needed before lunar transfer.",1'200'000,3'100'000,54.0,bit(6),8,"Deliver orbital insertion analysis and launch acceptance report."},
    {"Lunar Navigation Payload","Lunar Science Consortium","Validate navigation and the transfer trajectory to the Moon.",1'000'000,2'700'000,50.0,bit(8),15,"Deliver lunar approach navigation and cargo tracking dossier."},
    {"ISRU Prospecting Package","Resource Prospecting Partnership","Prove lunar water prospecting for later industrial facilities.",1'100'000,2'900'000,52.0,bit(10),13,"Deliver extraction trials and processing equipment specification."},
    {"Pressurized Module Test Article","Crew Systems Directorate","Demonstrate pressure-tight accommodation and life-support interfaces.",1'250'000,3'250'000,56.0,bit(11),14,"Deliver pressure integrity and airlock acceptance certificate."},
    {"Lunar Greenhouse Demonstrator","Closed Environment Research Group","Prove controlled crops for the future manned base.",900'000,2'500'000,47.0,bit(12),14,"Deliver hydroponic crop growth and environmental-control report."},
    {"Lunar Manufacturing Pilot","Industrial Space Partner","Demonstrate industrial equipment that will be deployed by robots.",1'400'000,3'900'000,62.0,bit(13),14,"Deliver integrated fabrication and maintenance test dossier."},
    // Additional one-time customers are arranged by research tier. Early
    // projects provide enough unrestricted cash to begin the technology tree;
    // late projects become the primary financing for launch hardware.
    {"Ground Segment Feasibility","Regional Ground Station Alliance","Survey a customer satellite communications ground network.",300'000,650'000,18.0,0,3,"Deliver station siting, uplink budgets and a coverage feasibility dossier."},
    {"Thermal Materials Survey","Advanced Materials Council","Evaluate thermal protection and light structures for small payloads.",350'000,750'000,22.0,0,1,"Submit sample coupons, thermal cycling records and a materials shortlist."},
    {"Propellant Safety Study","Civil Launch Safety Board","Analyze launch-pad fueling risks and emergency containment needs.",300'000,900'000,25.0,0,2,"Submit fueling hazard analysis and certified operating procedures."},
    {"Robotic Operations Assessment","Automation Research Network","Study rover logistics before ordering an autonomous field prototype.",450'000,950'000,29.0,bit(0),4,"Deliver robotic workload analysis and a fault-recovery concept."},
    {"Mission Verification Campaign","Orbital Systems Laboratory","Verify systems-engineering processes on a real spacecraft project.",450'000,1'100'000,27.0,bit(0),6,"Deliver interface verification matrices and acceptance evidence."},
    {"Composite Tank Development","Launch Structures Consortium","Build and qualify light propellant tanks for orbital launch stacks.",500'000,1'300'000,34.0,bit(1),6,"Deliver structural qualification and tank production specifications."},
    {"Engine Injector Test Article","Propulsion Technology Agency","Prototype liquid engine injector hardware for high-reliability stages.",550'000,1'600'000,38.0,bit(2),6,"Deliver thrust-chamber component test data and injector redesign."},
    {"Flight Computer Reliability","Commercial Avionics Group","Demonstrate fault-tolerant navigation software for a spacecraft bus.",700'000,1'700'000,32.0,bit(3),8,"Deliver hardware-in-the-loop reports and redundant controller software."},
    {"Solar Array Qualification","Energy Systems Manufacturer","Qualify deployable solar arrays and batteries for surface operations.",650'000,1'900'000,40.0,bit(5),14,"Deliver environmental test results and a power-conditioning prototype."},
    {"Rover Safety Qualification","Autonomous Mobility Authority","Certify robotic obstacle avoidance in a simulated surface worksite.",750'000,2'100'000,43.0,bit(4),14,"Deliver collision-avoidance trials and a rover safety case."},
    {"Orbital Fairing Certification","Payload Integration Agency","Validate payload separation and protection during ascent.",800'000,2'200'000,45.0,bit(6),8,"Deliver separation test results and fairing acceptance documentation."},
    {"Cabin Recycling Experiment","Environmental Control Institute","Demonstrate atmospheric recycling and water purification subsystems.",750'000,2'200'000,40.0,bit(7),11,"Deliver air revitalization and water reclamation endurance reports."},
    {"Cislunar Beacon Mission","Deep Space Tracking Consortium","Prove position-tracking and communications coverage for lunar cargo.",1'100'000,2'500'000,50.0,bit(8),15,"Deliver cislunar ranging, tracking and navigation measurements."},
    {"Automated Habitat Docking","Modular Habitat Partnership","Demonstrate robotic placement and safe connection of surface modules.",1'150'000,2'650'000,56.0,bit(9),14,"Deliver autonomous placement tests and habitat docking acceptance."},
    {"Ice Extraction Prototype","Space Resources Development Board","Demonstrate extraction of subsurface water into usable reserves.",1'200'000,2'900'000,55.0,bit(10),14,"Deliver frozen-regolith extraction trials and water-quality results."},
    {"Hermetic Hatch Endurance","Crew Safety Qualification Group","Qualify pressure doors and airlocks through repeated cycling.",1'150'000,2'800'000,54.0,bit(11),14,"Deliver sealed-door fatigue trials and leak-rate certificates."},
    {"Crop Lighting Automation","Agri-Space Research Cooperative","Optimize growth chamber lighting, water and hydroponics controls.",1'300'000,3'150'000,57.0,bit(12),14,"Deliver growing-cycle yields and crop system controls validation."},
    {"Lunar Fabrication Certificate","Advanced Manufacturing Alliance","Qualify additive manufacturing and repairs for lunar operations.",1'400'000,3'200'000,64.0,bit(13),15,"Deliver materials fabrication results and robotic maintenance trials."},
    {"Surface Logistics Integration","International Surface Operations","Integrate all construction utility and robot interfaces for a lunar base.",1'700'000,4'000'000,70.0,bit(14),15,"Deliver integrated base logistics and commissioning procedures."},
    {"Autonomous Deployment Review","Lunar Expedition Directorate","Verify end-to-end autonomous descent, unloading and surface deployment.",1'900'000,4'900'000,74.0,bit(15),-1,"Deliver rehearsal telemetry and a deployment readiness certificate."},
    {"Lunar Cargo Delivery Contract","International Lunar Program","Customer finances a real lunar shipment instead of a simulation-time study.",2'500'000,7'500'000,0.0,bit(14)|bit(8),-1,"Launch the funded lunar cargo mission after all readiness gates."}
}};


struct RocketDefinition {
    const char* name;
    const char* detail;
    std::int64_t lunar_transfer_payload_kg;
    std::int64_t cost_eur;
    std::uint32_t required_tech;
};

struct LanderDefinition {
    const char* name;
    const char* detail;
    std::int64_t surface_payload_kg;
    std::int64_t cost_eur;
    std::uint32_t required_tech;
};

constexpr std::array<RocketDefinition,3> kRockets{{
    {"ORBITAL HEAVY LIFTER","Conventional expendable cargo stack; lowest mission price.",120'000,8'500'000,bit(6)|bit(8)},
    {"LUNAR HEAVY LIFTER","High-energy cargo stack optimized for robotic lunar infrastructure.",220'000,13'500'000,bit(6)|bit(8)},
    {"REUSABLE SUPER-HEAVY","Reusable high-capacity launch stack for a large one-flight base package.",360'000,19'500'000,bit(15)}
}};

constexpr std::array<LanderDefinition,3> kLanders{{
    {"AUTONOMOUS CARGO LANDER","Compact robotic lander for a minimum viable surface package.",95'000,7'000'000,bit(8)|bit(4)},
    {"HEAVY CARGO LANDER","Wide-deck autonomous lander sized for a complete starter-base shipment.",180'000,11'500'000,bit(8)|bit(9)},
    {"BASE DEPLOYMENT LANDER","Integrated heavy lander with autonomous unloading and deployment support.",300'000,17'500'000,bit(15)}
}};

constexpr std::array<std::int64_t,kMaterialResourceCount> kResourcePurchaseEurPerKg{{
    18, 32, 44, 21, 4, 26, 140, 29
}};
constexpr std::int64_t kCargoIntegrationEurPerKg=85;

void add_bundle_local(ResourceBundle& dst,const ResourceBundle& src){
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=(MaterialResource)i;
        dst.amount(resource)+=src.amount(resource);
    }
}

std::string mass_text(std::int64_t kg){
    std::ostringstream ss;
    if(kg>=1000){ ss<<std::fixed<<std::setprecision(kg>=100'000?0:1)<<(double)kg/1000.0<<" T"; }
    else ss<<kg<<" KG";
    return ss.str();
}

std::string money(std::int64_t eur){
    std::ostringstream ss;
    const double m=(double)eur/1'000'000.0;
    if(std::abs(m)>=1.0){ ss<<"EUR "<<std::fixed<<std::setprecision(m>=10.0?1:2)<<m<<" M"; }
    else { ss<<"EUR "<<std::fixed<<std::setprecision(0)<<(double)eur/1000.0<<" K"; }
    return ss.str();
}

// Conservative, pre-launch funding audit. Do not include the lunar cargo
// contract's final payout (paid only after launch), research discounts,
// customer premiums or investor awards.  Even the most expensive rocket and
// lander plus the minimum viable manifest must fit inside total earnable
// commercial revenue (including the signing advances of signed contracts).
struct CampaignBudgetAudit {
    std::int64_t commercial_income{0};
    std::int64_t full_research_cost{0};
    std::int64_t maximum_vehicle_cost{0};
};
CampaignBudgetAudit campaign_budget_audit(){
    CampaignBudgetAudit audit;
    for(std::size_t i=0;i+1<kContracts.size();++i)
        audit.commercial_income+=kContracts[i].deposit_eur+kContracts[i].finish_eur;
    for(const auto& tech:kTechs) audit.full_research_cost+=tech.cost_eur;
    const auto rocket=std::max_element(kRockets.begin(),kRockets.end(),
        [](const auto& a,const auto& b){return a.cost_eur<b.cost_eur;});
    const auto lander=std::max_element(kLanders.begin(),kLanders.end(),
        [](const auto& a,const auto& b){return a.cost_eur<b.cost_eur;});
    audit.maximum_vehicle_cost=rocket->cost_eur+lander->cost_eur;
    return audit;
}

std::string days_text(double days){
    std::ostringstream ss; ss<<std::fixed<<std::setprecision(days<10.0?1:0)<<days<<" D"; return ss.str();
}

bool mask_unlocked(const PlanningGameState& state,std::uint32_t mask){
    for(std::size_t i=0;i<kPlanningTechnologyCount;++i)
        if((mask&bit(i)) && state.technologies[i]==0) return false;
    return true;
}

// Independent first-time customer deliveries provide technical evidence.
// Each distinct completed project reduces the relevant research purchase by
// 20%, capped at 35%. Completed contracts cannot be replayed.
std::int64_t effective_research_cost(const PlanningGameState& state,std::size_t technology){
    if(technology>=kTechs.size()) return 0;
    int discount=0;
    for(std::size_t i=0;i<kContracts.size();++i){
        if(kContracts[i].supported_tech==(int)technology && state.contracts[i].completed_count>0)
            discount=std::min(35,discount+20);
    }
    return kTechs[technology].cost_eur*(100-discount)/100;
}

// First delivery incentives reward different clients. Derived from
// completed_count, so older schema-29 saves gain
// correct progress automatically without changing serialized data.
std::int64_t first_delivery_bonus(const PlanningGameState& state,std::size_t index){
    if(index==0 || index>=kContracts.size()-1 || state.contracts[index].completed_count>0) return 0;
    return kContracts[index].finish_eur/5; // 20% first-time completion premium
}

int unique_deliveries(const PlanningGameState& state){
    int count=0;
    for(std::size_t i=0;i<kContracts.size()-1;++i) if(state.contracts[i].completed_count>0) ++count;
    return count;
}

std::int64_t reputation_award(int before,int after){
    std::int64_t reward=0;
    for(const auto& milestone:std::array<std::pair<int,std::int64_t>,3>{{{3,750'000},{6,1'500'000},{9,3'000'000}}})
        if(before<milestone.first && after>=milestone.first) reward+=milestone.second;
    return reward;
}

bool contract_ready_for_delivery(const PlanningGameState& state,std::size_t index){
    return index<kContracts.size()-1 && state.contracts[index].completed_count==0 &&
           state.contracts[index].status==PlanningContractStatus::Active &&
           state.contracts[index].progress_days+1e-9>=kContracts[index].duration_days;
}

int contract_priority(const PlanningGameState& state,std::size_t index){
    const auto& c=state.contracts[index];
    if(contract_ready_for_delivery(state,index)) return 0;
    if(c.status==PlanningContractStatus::Active)return 1;
    if(c.status==PlanningContractStatus::Completed || c.completed_count>0)return 4;
    if(mask_unlocked(state,kContracts[index].required_tech)){
        const int target=kContracts[index].supported_tech;
        if(c.completed_count==0 && target>=0 && !state.technologies[(std::size_t)target]) return 2;
        if(c.completed_count==0) return 3;
        return 4;
    }
    return 5;
}

std::string missing_contract_research(const PlanningGameState& state,std::size_t index){
    std::string missing;
    for(std::size_t t=0;t<kTechs.size();++t){
        if((kContracts[index].required_tech&bit(t)) && !state.technologies[t]){
            if(!missing.empty())missing+=" + ";
            missing+=kTechs[t].short_name;
        }
    }
    return missing;
}

// Show the entire technology dependency chain, not only the last direct
// requirements. The contract rules still use required_tech unchanged; this
// expansion is informational and cannot bypass research prerequisites.
struct ContractResearchRequirement {
    std::size_t tech{};
    bool direct{};
};
std::vector<ContractResearchRequirement> contract_research_requirements(std::size_t contract){
    std::vector<ContractResearchRequirement> items;
    if(contract>=kContracts.size()) return items;
    const auto direct=kContracts[contract].required_tech;
    auto closure=direct;
    for(std::size_t pass=0;pass<kTechs.size();++pass){
        const auto before=closure;
        for(std::size_t i=0;i<kTechs.size();++i)
            if(closure&bit(i)) closure|=kTechs[i].prerequisites;
        if(closure==before) break;
    }
    // Put direct contract requirements first, then their earlier supporting
    // research. The categories remain explicitly labeled in the UI.
    for(int category=0;category<2;++category)
        for(std::size_t i=0;i<kTechs.size();++i)
            if((closure&bit(i)) && ((direct&bit(i))!=0)==(category==0))
                items.push_back({i,category==0});
    return items;
}

std::size_t active_contract_count(const PlanningGameState& state){
    std::size_t count=0;
    for(const auto& c:state.contracts) if(c.status==PlanningContractStatus::Active) ++count;
    return count;
}

std::size_t unlocked_tech_count(const PlanningGameState& state){
    return (std::size_t)std::count_if(state.technologies.begin(),state.technologies.end(),[](std::uint8_t v){return v!=0;});
}

// The campaign coach is a context-sensitive destination, never an automatic
// research purchase, contract acceptance, or mission launch.
struct CampaignAdvice {
    PlanningView view{PlanningView::Contracts};
    MissionFacility facility{MissionFacility::ProgramOffice};
    int contract{-1};
    int technology{-1};
    const char* action{"OPEN CONTRACTS"};
    std::string reason;
};
CampaignAdvice campaign_advice(const PlanningGameState& state){
    CampaignAdvice hint;
    if(state.mission_step==MissionPreparationStep::Complete){
        hint.view=PlanningView::Mission;hint.facility=MissionFacility::Overview;
        hint.action="MISSION COMPLETE";hint.reason="Lunar shipment launched. Your Stage-1 campaign is complete.";
        return hint;
    }
    for(std::size_t i=0;i<kContracts.size()-1;++i){
        if(contract_ready_for_delivery(state,i)){
            hint.contract=(int)i;hint.action="DELIVER REPORT";
            hint.reason=std::string("Submit ")+kContracts[i].name+" and collect the customer payment.";
            return hint;
        }
    }
    const int delivered=unique_deliveries(state);
    if(delivered>0){
        // Respect technology dependencies; prioritize affordable nodes in
        // evolutionary order instead of recommending a locked end-game node.
        for(std::size_t i=0;i<kTechs.size();++i){
            if(state.technologies[i] || !mask_unlocked(state,kTechs[i].prerequisites))continue;
            if(state.funds_eur>=effective_research_cost(state,i)){
                hint.view=PlanningView::Technology;hint.facility=MissionFacility::ResearchEngineering;
                hint.technology=(int)i;hint.action="RESEARCH NEXT";
                hint.reason=std::string("Unlock ")+kTechs[i].short_name+" to open the next capability.";
                return hint;
            }
        }
    }
    if(state.technologies[14] && state.technologies[15] && state.mission_step!=MissionPreparationStep::Complete){
        // At the funding gate, recommend new revenue if the frozen plan is
        // unaffordable instead of sending the player to a disabled button.
        const auto ri=std::min<std::size_t>(state.mission_rocket,kRockets.size()-1);
        const auto li=std::min<std::size_t>(state.mission_lander,kLanders.size()-1);
        std::int64_t estimate=kRockets[ri].cost_eur+kLanders[li].cost_eur;
        for(std::size_t i=0;i<kMaterialResourceCount;++i)
            estimate+=std::max<std::int64_t>(0,state.mission_cargo.amount((MaterialResource)i))*(kResourcePurchaseEurPerKg[i]+kCargoIntegrationEurPerKg);
        if(state.mission_step!=MissionPreparationStep::Confirm || state.funds_eur>=estimate){
            hint.view=PlanningView::Mission;hint.facility=mission_step_facility(state.mission_step);
            hint.action="MISSION WORKFLOW";
            hint.reason=std::string("Next: ")+mission_step_name(state.mission_step)+" at "+mission_facility_name(hint.facility)+".";
            return hint;
        }
    }
    for(std::size_t i=0;i<kContracts.size();++i){
        if(state.contracts[i].status==PlanningContractStatus::Active && i<kContracts.size()-1){
            hint.contract=(int)i;hint.action="FINISH PROJECT";
            hint.reason=std::string("Advance to the next report for ")+kContracts[i].name+".";
            return hint;
        }
    }
    // Recommend only new customers; completed contracts are permanently closed.
    int best=-1,best_priority=999;
    for(std::size_t i=0;i<kContracts.size();++i){
        const auto& c=state.contracts[i];
        if(c.status==PlanningContractStatus::Active ||
           c.status==PlanningContractStatus::Completed || c.completed_count>0 ||
           (i==kContracts.size()-1&&state.mission_step==MissionPreparationStep::Complete) ||
           !mask_unlocked(state,kContracts[i].required_tech)) continue;
        const int rank=contract_priority(state,i);
        if(rank<best_priority){best_priority=rank;best=(int)i;}
    }
    hint.contract=best;
    hint.action="FIND CUSTOMER WORK";
    hint.reason=best>=0?std::string("Try ")+kContracts[(std::size_t)best].name+" to earn cash and new research evidence.":
                         "No new customer contracts available. Finish active deliveries or continue mission preparation.";
    return hint;
}

double speed_days_per_second(std::uint8_t s){
    switch(s){case 0:return 0.0;case 1:return 1.0;case 2:return 5.0;case 3:return 20.0;default:return 1.0;}
}

} // namespace

// Click target groups are separate so switching from a contract to research
// never accidentally commits an unrelated purchase. Spatial tolerance also
// avoids treating fast clicks while moving around the canvas as activation.
bool GameApp::planning_double_clicked(int group,int item){
    if(!ui_input_.left_pressed) return false;
    const auto now=std::chrono::steady_clock::now();
    const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(
        now-planning_last_click_time_).count();
    const int dx=ui_input_.mouse_x-planning_last_click_x_;
    const int dy=ui_input_.mouse_y-planning_last_click_y_;
    const bool activated=planning_last_click_group_==group &&
        planning_last_click_item_==item && ms>=0 && ms<=410 &&
        dx*dx+dy*dy<=22*22;
    planning_last_click_group_=activated?-1:group;
    planning_last_click_item_=activated?-1:item;
    planning_last_click_time_=now;
    planning_last_click_x_=ui_input_.mouse_x;
    planning_last_click_y_=ui_input_.mouse_y;
    return activated;
}

void GameApp::reset_planning_game(){
    planning_=default_planning_game_state();
    planning_structure_page_=-1;
    planning_contract_page_=0;
    planning_contract_selected_=0;
    planning_contract_research_page_=0;
    planning_tech_zoom_=1.02f;
    planning_tech_pan_x_=planning_tech_pan_y_=0;
    planning_tech_selected_=0;
    planning_last_click_group_=-1;
    planning_last_click_item_=-1;
    planning_status_message_="Welcome! Start with a customer contract, submit its deliverable, and invest the earnings in research. Then prepare the lunar mission.";
}

bool GameApp::planning_technology_available(std::size_t index) const {
    if(index>=kTechs.size() || planning_.technologies[index]) return false;
    return mask_unlocked(planning_,kTechs[index].prerequisites);
}

bool GameApp::planning_contract_available(std::size_t index) const {
    if(index>=kContracts.size()) return false;
    const auto& progress=planning_.contracts[index];
    if(progress.status==PlanningContractStatus::Active) return false;
    if(index==kContracts.size()-1 && planning_.mission_step==MissionPreparationStep::Complete) return false;
    // Historical repeatable projects may have status Available despite
    // completed_count > 0; completion history is authoritative.
    if(progress.status==PlanningContractStatus::Completed || progress.completed_count>0) return false;
    return mask_unlocked(planning_,kContracts[index].required_tech);
}

ResourceBundle GameApp::planning_reference_mission_cargo() const {
    // Minimum practical Stage-2 shipment: enough hardware/material stock to
    // establish power + water, a pressurized crew-ready core, rover service,
    // and the mandatory landing infrastructure. This deliberately follows the
    // same construction-cost model used by Stage 2 instead of inventing a
    // disconnected planning-only mass budget.
    ResourceBundle total{};
    auto add_habitat=[&](HabitatKind kind,HabitatSizeTier size){
        add_bundle_local(total,construction_costs::habitat(kind,size,habitat_models_.mesh(kind,size)).total());
    };
    auto add_structure=[&](ResourceKind kind,ResourceSizeTier size){
        add_bundle_local(total,construction_costs::structure(kind,resource_models_.mesh(kind,size)).total());
    };

    add_habitat(HabitatKind::AirLock,HabitatSizeTier::Small);
    add_habitat(HabitatKind::OxygenGeneratorDome,HabitatSizeTier::Small);
    add_habitat(HabitatKind::CantineDome,HabitatSizeTier::Small);
    add_habitat(HabitatKind::CrewQuarters,HabitatSizeTier::Mid); // six crew

    add_structure(ResourceKind::SolarPanel,ResourceSizeTier::Standard);
    add_structure(ResourceKind::SolarPanel,ResourceSizeTier::Standard); // redundancy
    add_structure(ResourceKind::PowerStorage,ResourceSizeTier::Standard);
    add_structure(ResourceKind::WaterDriller,ResourceSizeTier::Standard);
    add_structure(ResourceKind::WaterStorage,ResourceSizeTier::Standard);
    add_structure(ResourceKind::WaterPurifier,ResourceSizeTier::Small);
    add_structure(ResourceKind::LandingPad,ResourceSizeTier::Standard);
    add_structure(ResourceKind::RobotWorkshop,ResourceSizeTier::Small);

    add_bundle_local(total,construction_costs::corridor(70.0f).total());
    add_bundle_local(total,construction_costs::supply_channel(120.0f).total());
    add_bundle_local(total,construction_costs::walkway(30.0f).total());

    for(const auto material:kConstructionMaterialResources)
        total.amount(material)=(total.amount(material)*115+99)/100; // 15% routing/placement reserve

    total.water=8'000;
    total.food=2'000;
    total.medical=400;
    return total;
}

ResourceBundle GameApp::planning_structure_requirements() const {
    ResourceBundle required;
    const auto add=[&](const ResourceBundle& per_structure,std::uint8_t count){
        for(const auto material:kConstructionMaterialResources)
            required.amount(material)+=per_structure.amount(material)*count;
    };
    for(std::size_t i=0;i<planning_.mission_habitats.size();++i){
        const auto count=planning_.mission_habitats[i];
        if(!count) continue;
        const auto kind=(HabitatKind)i;
        constexpr auto size=HabitatSizeTier::Standard;
        add(construction_costs::habitat(kind,size,habitat_models_.mesh(kind,size)).total(),count);
    }
    for(std::size_t i=0;i<planning_.mission_structures.size();++i){
        const auto count=planning_.mission_structures[i];
        if(!count) continue;
        const auto kind=(ResourceKind)i;
        add(construction_costs::structure(kind,resource_models_.mesh(kind,ResourceSizeTier::Standard)).total(),count);
    }
    return required;
}

bool GameApp::planning_cargo_covers_structures() const {
    const auto required=planning_structure_requirements();
    for(const auto material:kConstructionMaterialResources)
        if(planning_.mission_cargo.amount(material)<required.amount(material)) return false;
    return true;
}

void GameApp::planning_add_required_cargo(){
    if(planning_.mission_step!=MissionPreparationStep::Plan) return;
    const auto required=planning_structure_requirements();
    for(const auto material:kConstructionMaterialResources){
        auto& amount=planning_.mission_cargo.amount(material);
        amount=std::max(amount,required.amount(material));
    }
    planning_status_message_="Payload updated for the selected structures.";
}

std::int64_t GameApp::planning_mission_payload_kg() const {
    return std::max<std::int64_t>(0,planning_.mission_cargo.total_amount())+
        planning_.mission_builders*kMissionBuilderMassKg+planning_.mission_carriers*kMissionCarrierMassKg;
}

std::int64_t GameApp::planning_mission_payload_capacity_kg() const {
    const auto ri=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1);
    const auto li=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
    return std::min(kRockets[ri].lunar_transfer_payload_kg,kLanders[li].surface_payload_kg);
}

std::int64_t GameApp::planning_mission_cost_eur() const {
    const auto ri=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1);
    const auto li=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
    std::int64_t total=kRockets[ri].cost_eur+kLanders[li].cost_eur+
        planning_.mission_builders*85'000+planning_.mission_carriers*110'000;
    for(std::size_t i=0;i<kMaterialResourceCount;++i){
        const auto resource=(MaterialResource)i;
        const auto kg=std::max<std::int64_t>(0,planning_.mission_cargo.amount(resource));
        total+=kg*(kResourcePurchaseEurPerKg[i]+kCargoIntegrationEurPerKg);
    }
    return total;
}

bool GameApp::planning_mission_requirements_met() const {
    // This function is the financial/program-authorization gate.  Earlier
    // facility work must already have reached CONFIRM before money can be
    // committed to flight hardware and cargo.
    if(planning_.mission_paid) return true;
    if(planning_.mission_step!=MissionPreparationStep::Confirm || planning_.mission_facility!=MissionFacility::ProgramOffice) return false;
    const auto ri=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1);
    const auto li=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
    if(!mask_unlocked(planning_,kRockets[ri].required_tech) || !mask_unlocked(planning_,kLanders[li].required_tech)) return false;
    if(!planning_.technologies[14] || !planning_.technologies[15]) return false;
    if(!planning_cargo_covers_structures()) return false;
    if(planning_mission_payload_kg()>planning_mission_payload_capacity_kg()) return false;
    return planning_.funds_eur>=planning_mission_cost_eur();
}

bool GameApp::purchase_planning_mission(){
    if(planning_.mission_paid || !planning_mission_requirements_met()) return false;
    const auto cost=planning_mission_cost_eur();
    planning_.funds_eur-=cost;
    planning_.mission_paid=true;
    planning_.mission_paid_cost_eur=cost;
    planning_.mission_step=MissionPreparationStep::Build;
    planning_.mission_facility=MissionFacility::VehicleAssembly;
    planning_status_message_="Program approval complete and mission funded. Proceed to the Vehicle Assembly Building.";
    return true;
}

void GameApp::deploy_lunar_base_from_mission(){
    // The lunar campaign is one-way. Preserve planning_ (the paid manifest,
    // research and contract history) but replace the Earth company campus
    // with a completely fresh robotic lunar-base simulation.
    const ResourceBundle shipped=planning_.mission_cargo;
    trader_={};trade_message_.clear();
    playfield_size_=kNewGamePlayfieldDefault;
    playfield_.rebuild(playfield_size_);
    camera_.reset(playfield_size_);
    first_person_mode_=FirstPersonMode::Off;
    app_.set_relative_mouse_mode(false);
    status_.reset();
    status_rate_display_={};
    battery_net_kw_=water_net_litres_per_day_=0;
    supply_power_delivered_kw_=supply_power_load_kw_=supply_water_load_litres_per_day_=0;
    resources_.reset();
    placed_habitats_.clear();
    placed_resources_.clear();
    ground_resource_piles_.clear();
    builder_robots_.clear();
    next_blueprint_placement_sequence_=1;
    construction_priority_.clear();
    debug_move_pick_robot_=-1;
    first_person_carried_resource_.reset();
    first_person_hover_ground_resource_=-1;
    first_person_hover_storage_resource_=-1;
    first_person_hover_storage_box_=-1;
    corridor_network_.clear();
    corridor_network_.set_playfield_half_extent((float)playfield_size_*0.5f);
    utility_network_.clear();
    utility_network_.set_playfield_half_extent((float)playfield_size_*0.5f);
    walkway_network_.clear();
    pressure_doors_.clear();
    airlock_cycles_.clear();
    storage_gate_open_.clear();
    production_fraction_.fill(0.0);
    life_support_fraction_seconds_=0.0;
    begin_lunar_landing(shipped,planning_.mission_builders,planning_.mission_carriers);
    build_catalog_.clear_selection();
    placement_rotation_degrees_=0.0f;
    placement_habitat_size_=HabitatSizeTier::Small;
    placement_resource_size_=ResourceSizeTier::Small;
    placement_storage_size_=StorageDomeSize::Small;
    storage_gate_placement_active_=false;
    storage_gate_placement_owner_=kNoRobotIndex;
    habitat_view_mode_=HabitatViewMode::Default;
    game_speed_=GameSpeed::Normal;
    clear_world_selection();
    selection_marquee_active_=false;
    selection_marquee_dragged_=false;
    corridor_link_source_=-1;
    corridor_build_start_.reset();
    utility_build_start_.reset();
    walkway_build_start_.reset();
    walkway_build_target_.reset();
    debug_start_template_enabled_=false;
    selection_kind_=SelectionKind::Lander;
    active_game_phase_=GamePhase::RobotPreparation;
    new_game_phase_selection_=GamePhase::RobotPreparation;
    new_game_mode_selection_=active_game_mode_;
    game_session_active_=true;
    screen_=Screen::Playfield;
    update_supply_simulation(0.0f);
}

bool GameApp::complete_planning_mission_step(){
    const auto step=planning_.mission_step;
    if(step!=MissionPreparationStep::Complete && planning_.mission_facility!=mission_step_facility(step)){
        planning_status_message_=std::string("This gate belongs to ")+mission_facility_name(mission_step_facility(step))+". Open that facility on the company site first.";
        return false;
    }
    const auto ri=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1);
    const auto li=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
    const auto cargo_ready=[&]{
        return planning_cargo_covers_structures()&&planning_mission_payload_kg()<=planning_mission_payload_capacity_kg();
    };

    switch(step){
        case MissionPreparationStep::Research:
            if(!planning_.technologies[14] || !planning_.technologies[15]){
                planning_status_message_="Research review blocked: unlock Stage-2 Module Competence and Robotic Lunar Base Mission Architecture in the Tech Tree.";
                return false;
            }
            planning_.elapsed_days+=10.0;
            planning_.mission_step=MissionPreparationStep::Design;
            planning_status_message_="Research baseline approved. Select and freeze the rocket/lander preliminary design.";
            return true;
        case MissionPreparationStep::Design:
            if(!mask_unlocked(planning_,kRockets[ri].required_tech) || !mask_unlocked(planning_,kLanders[li].required_tech)){
                planning_status_message_="Design blocked: selected rocket or lander requires technology that is not yet unlocked.";
                return false;
            }
            planning_.elapsed_days+=18.0;
            planning_.mission_step=MissionPreparationStep::Test;
            planning_.mission_facility=MissionFacility::PropulsionTest;
            planning_status_message_="Preliminary flight design frozen. Take the design to the Propulsion Test Center.";
            return true;
        case MissionPreparationStep::Test:
            planning_.elapsed_days+=24.0;
            planning_.mission_step=MissionPreparationStep::Refine;
            planning_.mission_facility=MissionFacility::ResearchEngineering;
            planning_status_message_="Qualification test campaign completed. Test findings returned to Research & Engineering for refinement.";
            return true;
        case MissionPreparationStep::Refine:
            planning_.elapsed_days+=12.0;
            planning_.mission_step=MissionPreparationStep::Plan;
            planning_.mission_facility=MissionFacility::MissionControl;
            planning_status_message_="Vehicle design refined and baselined. Mission Control can now finalize the lunar flight and cargo plan.";
            return true;
        case MissionPreparationStep::Plan:
            if(!cargo_ready()){
                planning_status_message_="Mission plan blocked: add cargo for your selected structures and check vehicle payload capacity.";
                return false;
            }
            planning_.elapsed_days+=7.0;
            planning_.mission_step=MissionPreparationStep::Confirm;
            planning_.mission_facility=MissionFacility::ProgramOffice;
            planning_status_message_="Mission plan frozen. Program & Contracts must authorize and fund the mission.";
            return true;
        case MissionPreparationStep::Confirm:
            return purchase_planning_mission();
        case MissionPreparationStep::Build:
            if(!planning_.mission_paid){
                planning_status_message_="Vehicle build blocked: the mission has not been funded.";
                return false;
            }
            planning_.elapsed_days+=45.0;
            planning_.mission_step=MissionPreparationStep::Integrate;
            planning_.mission_facility=MissionFacility::CargoIntegration;
            planning_status_message_="Rocket and lander flight hardware built. Move the lunar manifest to Cargo Integration.";
            return true;
        case MissionPreparationStep::Integrate:
            planning_.elapsed_days+=10.0;
            planning_.mission_step=MissionPreparationStep::FinalConfirm;
            planning_.mission_facility=MissionFacility::Communications;
            planning_status_message_="Cargo integrated and sealed. Communications Center must complete final readiness confirmation.";
            return true;
        case MissionPreparationStep::FinalConfirm:
            planning_.elapsed_days+=2.0;
            planning_.mission_step=MissionPreparationStep::Launch;
            planning_.mission_facility=MissionFacility::LaunchComplex;
            planning_status_message_="Final tracking, telemetry and communications readiness confirmed. Vehicle cleared for launch.";
            return true;
        case MissionPreparationStep::Launch:{
            // The launch gate is a one-off irreversible campaign transition.
            // No mission may be launched again, and the funded payload is
            // delivered to Stage 2 exactly once.
            if(!planning_.mission_paid || !cargo_ready()){
                planning_status_message_="Launch blocked: funded vehicle and valid sealed cargo are required.";
                return false;
            }
            planning_.elapsed_days+=1.0;
            planning_.mission_step=MissionPreparationStep::Complete;
            planning_.mission_facility=MissionFacility::Overview;
            // The final customer contract is a real delivery objective. It
            // cannot pay its completion fee merely because time passed.
            auto& customer=planning_.contracts.back();
            if(customer.status==PlanningContractStatus::Active){
                planning_.funds_eur+=kContracts.back().finish_eur;
                customer.status=PlanningContractStatus::Completed;
                customer.progress_days=0.0;
                if(customer.completed_count<std::numeric_limits<std::uint16_t>::max()) ++customer.completed_count;
                planning_status_message_="MISSION LAUNCHED. Lunar cargo customer delivery accepted: "+money(kContracts.back().finish_eur)+" final payout received.";
            }else{
                planning_status_message_="MISSION LAUNCHED. Lunar lander entering powered final approach.";
            }
            deploy_lunar_base_from_mission();
            return true;
        }
        case MissionPreparationStep::Complete:
            planning_status_message_="Mission already launched successfully.";
            return false;
    }
    return false;
}

bool GameApp::accept_planning_contract(std::size_t index){
    if(!planning_contract_available(index) || active_contract_count(planning_)>=kPlanningMaxActiveContracts) return false;
    auto& state=planning_.contracts[index];
    const auto& contract=kContracts[index];
    state.status=PlanningContractStatus::Active;
    state.progress_days=0.0;
    planning_.funds_eur+=contract.deposit_eur;
    planning_contract_selected_=(int)index;
    planning_status_message_=std::string("Project awarded: ")+contract.name+". "+money(contract.deposit_eur)+" advance received. Complete the work, then submit its report for final payment.";
    return true;
}

bool GameApp::deliver_planning_contract(std::size_t index){
    if(!contract_ready_for_delivery(planning_,index) || planning_.contracts[index].completed_count>0) return false;
    auto& progress=planning_.contracts[index];
    const auto& contract=kContracts[index];
    const int reputation_before=unique_deliveries(planning_);
    const auto premium=first_delivery_bonus(planning_,index);
    planning_.funds_eur+=contract.finish_eur+premium;
    if(progress.completed_count<std::numeric_limits<std::uint16_t>::max()) ++progress.completed_count;
    progress.progress_days=0.0;
    progress.status=PlanningContractStatus::Completed;
    const auto investor_award=reputation_award(reputation_before,unique_deliveries(planning_));
    planning_.funds_eur+=investor_award;
    const auto tech=contract.supported_tech;
    planning_status_message_=std::string("Delivered: ")+contract.name+". "+money(contract.finish_eur+premium)+
        " received"+(premium?" (new-customer premium)":"")+
        (investor_award?" + "+money(investor_award)+" investor milestone award":"")+
        ". "+(tech>=0?(std::string("Research evidence: ")+kTechs[(std::size_t)tech].short_name+" costs less."):"The program gains experience.");
    return true;
}

bool GameApp::unlock_planning_technology(std::size_t index){
    if(!planning_technology_available(index)) return false;
    const auto& tech=kTechs[index];
    const auto cost=effective_research_cost(planning_,index);
    if(planning_.funds_eur<cost){
        planning_status_message_=std::string("Insufficient funds for ")+tech.name+". Need "+money(cost)+"; customer contracts can help finance it.";
        return false;
    }
    planning_.funds_eur-=cost;
    planning_.technologies[index]=1;
    planning_status_message_=std::string("Technology unlocked: ")+tech.name+". Capability: "+tech.unlock+".";
    return true;
}

bool GameApp::advance_planning_to_next_report(){
    double wait=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<kContracts.size()-1;++i){
        const auto& state=planning_.contracts[i];
        if(state.status==PlanningContractStatus::Active && !contract_ready_for_delivery(planning_,i))
            wait=std::min(wait,std::max(0.0,kContracts[i].duration_days-state.progress_days));
    }
    if(!std::isfinite(wait) || wait<=0.0) return false;
    planning_.elapsed_days+=wait;
    for(std::size_t i=0;i<kContracts.size()-1;++i){
        auto& state=planning_.contracts[i];
        if(state.status==PlanningContractStatus::Active)
            state.progress_days=std::min(kContracts[i].duration_days,state.progress_days+wait);
    }
    planning_.time_speed=0; // prevent overshooting while reading the completed report
    for(std::size_t i=0;i<kContracts.size()-1;++i){
        if(contract_ready_for_delivery(planning_,i)){
            planning_contract_selected_=(int)i;
            planning_status_message_=std::string("Customer work finished: ")+kContracts[i].name+
                ". Select SUBMIT REPORT to claim its final payment.";
            break;
        }
    }
    planning_.view=PlanningView::Contracts;
    return true;
}

void GameApp::update_planning_game(float dt){
    const double days=std::max(0.0f,dt)*speed_days_per_second(planning_.time_speed);
    if(days<=0.0) return;
    planning_.elapsed_days+=days;
    for(std::size_t i=0;i<kContracts.size()-1;++i){
        auto& state=planning_.contracts[i];
        if(state.status!=PlanningContractStatus::Active || contract_ready_for_delivery(planning_,i)) continue;
        const auto& contract=kContracts[i];
        state.progress_days=std::min(contract.duration_days,state.progress_days+days);
        if(contract_ready_for_delivery(planning_,i)){
            planning_status_message_=std::string("Project ready: ")+contract.name+". Open Contracts and submit the deliverable to claim "+money(contract.finish_eur)+".";
        }
    }
    // The lunar cargo sponsorship stays active until the actual Launch gate.
}

void GameApp::render_planning_game(Renderer& r){
    app_.set_cursor_visible(true);
    const float aspect=(float)r.width()/std::max(1,r.height());
    const Mat4 planning_vp=camera_.campus_view_projection(aspect);

    int campus_hover=-1;
    Vec3 campus_hit{};
    if(planning_.view==PlanningView::Site){
        const float my=(float)input_.mouse_y/std::max(0.001f,ui_scale_);
        const float logical_h=(float)r.height()/std::max(0.001f,ui_scale_);
        if(my>=STATUS_BAR_H && my<logical_h-BUILD_BAR_H &&
           camera_.intersect_playfield(input_.mouse_x,input_.mouse_y,r.width(),r.height(),playfield_.size(),campus_hit)){
            for(std::size_t i=0;i<kCampusBuildings.size();++i){
                if(campus_contains(kCampusBuildings[i],campus_hit)){ campus_hover=(int)i; break; }
            }
        }
    }

    const bool first_customer=unlocked_tech_count(planning_)==0 &&
                  unique_deliveries(planning_)==0 && active_contract_count(planning_)==0;
    // In a new career, the first useful building is the Program Office.
    // Highlight urgent completed customer reports there as well instead
    // of showing Research forever as the only apparent destination.
    const auto next_facility=campaign_advice(planning_).facility;
    if(planning_.view==PlanningView::Site)campus_daylight_sky(r);
    r.set_3d(planning_vp);
    if(planning_.view==PlanningView::Site){
        render_campus_landscape(r);
        // Shadows are cast onto the fully developed terrain before drawing
        // solid buildings, so each structure appears grounded.
        for(const auto& site:kCampusBuildings)
            campus_shadow(r,site.center,site.footprint.x,site.footprint.z,site.footprint.y);
        for(std::size_t i=0;i<kCampusBuildings.size();++i)
            render_campus_building(r,kCampusBuildings[i],campus_hover==(int)i,kCampusBuildings[i].facility==next_facility);
        campus_service_details(r);
    }else{
        r.draw_triangles_3d(playfield_.floor_triangles(),{0.038f,0.050f,0.062f,1});
        r.draw_lines_3d(playfield_.grid_lines(),{0.13f,0.24f,0.29f,0.55f},1.0f);
    }

    r.set_2d(ui_scale_);
    const float logical_w=ui_width(r),logical_h=ui_height(r);
    const float top_h=STATUS_BAR_H;
    const float bottom_h=BUILD_BAR_H;
    const bool compact=logical_w<1250.0f || logical_h<720.0f;
    r.draw_rect({0,0,logical_w,top_h},{0.020f,0.030f,0.040f,0.98f});
    r.draw_rect({0,logical_h-bottom_h,logical_w,bottom_h},{0.020f,0.030f,0.040f,0.98f});

    if(compact){
        game_label(r,{12,11},"S1 PLANNING",0.47f,theme_);
        game_label_fitted(r,{165,10,300,20},"FUNDS  "+money(planning_.funds_eur),0.43f,theme_);
        game_label_fitted(r,{480,10,135,20},"DAY "+std::to_string((int)std::floor(planning_.elapsed_days+1.0)),0.39f,theme_,true);
        game_label_fitted(r,{620,10,145,20},"TECH "+std::to_string(unlocked_tech_count(planning_))+"/"+std::to_string(kPlanningTechnologyCount),0.39f,theme_,true);
        game_label_fitted(r,{770,10,std::max(100.0f,logical_w-782),20},"CONTRACTS "+std::to_string(active_contract_count(planning_))+"/"+std::to_string(kPlanningMaxActiveContracts),0.37f,theme_,true);
    }else{
        game_label(r,{18,11},"STAGE 1 - MISSION PLANNING",0.60f,theme_);
        game_label(r,{310,11},"FUNDS  "+money(planning_.funds_eur),0.58f,theme_);
        game_label(r,{600,11},"PROGRAM DAY  "+std::to_string((int)std::floor(planning_.elapsed_days+1.0)),0.54f,theme_,true);
        game_label(r,{820,11},"TECH  "+std::to_string(unlocked_tech_count(planning_))+" / "+std::to_string(kPlanningTechnologyCount),0.54f,theme_,true);
        game_label(r,{1015,11},"ACTIVE CONTRACTS  "+std::to_string(active_contract_count(planning_))+" / "+std::to_string(kPlanningMaxActiveContracts),0.54f,theme_,true);
        game_label(r,{logical_w-175,12},"VERSION "+std::string(version::current),0.43f,theme_,true);
    }

    const float content_y=top_h+12.0f;
    const float content_h=logical_h-top_h-bottom_h-24.0f;
    const float margin=16.0f;

    const float tab_w=compact?96.0f:166.0f;
    const float tab_gap=compact?6.0f:8.0f;
    // Selected navigation has its own visible tab marker. Do not force the
    // user to infer their location from the content alone.
    const std::array<PlanningView,4> sections{{PlanningView::Site,PlanningView::Contracts,
                                               PlanningView::Technology,PlanningView::Mission}};
    for(int t=0;t<4;++t){
        const float tx=margin+(tab_w+tab_gap)*(float)t;
        if(planning_.view==sections[(std::size_t)t]){
            r.draw_rect({tx-2,logical_h-bottom_h+11,tab_w+4,49},
                        {0.065f,0.155f,0.190f,1});
            r.draw_rect({tx,logical_h-bottom_h+12,tab_w,3},{0.24f,0.82f,0.95f,1});
        }
    }
    if(ui::button(r,ui_input_,{margin,logical_h-bottom_h+15,tab_w,42},"COMPANY SITE",theme_)) planning_.view=PlanningView::Site;
    if(ui::button(r,ui_input_,{margin+(tab_w+tab_gap),logical_h-bottom_h+15,tab_w,42},"CONTRACTS",theme_)) planning_.view=PlanningView::Contracts;
    if(ui::button(r,ui_input_,{margin+2*(tab_w+tab_gap),logical_h-bottom_h+15,tab_w,42},"TECH TREE",theme_)) planning_.view=PlanningView::Technology;
    if(ui::button(r,ui_input_,{margin+3*(tab_w+tab_gap),logical_h-bottom_h+15,tab_w,42},compact?"MISSION":"MISSION PREP",theme_)) planning_.view=PlanningView::Mission; // resume, never reset the mission station
    const auto guidance=campaign_advice(planning_);
    const float speed_x=logical_w-(compact?355.0f:410.0f);
    const float coach_x=margin+4.0f*(tab_w+tab_gap)+8.0f;
    const float coach_w=std::min(compact?158.0f:206.0f,speed_x-coach_x-12.0f);
    if(coach_w>=100.0f){
        r.draw_rect({coach_x-2,logical_h-bottom_h+12,coach_w+4,48},
                    {0.060f,0.215f,0.205f,1});
    }
    if(coach_w>=100.0f && ui::button(r,ui_input_,{coach_x,logical_h-bottom_h+15,coach_w,42},"NEXT STEP",theme_,
                                     planning_.mission_step!=MissionPreparationStep::Complete)){
        planning_.view=guidance.view;
        if(guidance.contract>=0) planning_contract_selected_=guidance.contract;
        if(guidance.technology>=0) planning_tech_selected_=guidance.technology;
        if(guidance.view==PlanningView::Mission) planning_.mission_facility=guidance.facility;
        planning_status_message_=guidance.reason;
    }
    game_label(r,{speed_x,logical_h-bottom_h+27},"TIME",0.44f,theme_,true);
    const std::array<const char*,4> speed_labels{{"PAUSE","1X","5X","20X"}};
    for(std::size_t i=0;i<speed_labels.size();++i){
        const float bw=compact?58.0f:68.0f;
        if(ui::button(r,ui_input_,{speed_x+54.0f+(bw+6.0f)*(float)i,logical_h-bottom_h+15,bw,42},speed_labels[i],theme_)) planning_.time_speed=(std::uint8_t)i;
    }

    if(planning_.view==PlanningView::Site){
        // The campus is the physical mission-workflow surface.  Every facility
        // opens its own preparation station; Contracts and the Tech Tree remain
        // available from the relevant facility as secondary planning tools.
        if(campus_hover>=0 && ui_input_.left_pressed){
            const auto& building=kCampusBuildings[(std::size_t)campus_hover];
            planning_.mission_facility=building.facility;
            planning_.view=PlanningView::Mission;
            planning_status_message_=std::string(building.name)+" opened for mission preparation.";
        }

        Rect site_info{margin,content_y,compact?330.0f:405.0f,compact?110.0f:129.0f};
        r.draw_rect(site_info,{0.020f,0.030f,0.040f,0.91f});
        game_label(r,{site_info.x+12,site_info.y+9},"SPACE COMPANY LAUNCH CAMPUS",compact?0.46f:0.58f,theme_);
        game_label_fitted(r,{site_info.x+12,site_info.y+34,site_info.w-24,17},
                          first_customer?"START HERE: ACCEPT YOUR FIRST CUSTOMER CONTRACT":
                          guidance.reason,
                          compact?0.30f:0.35f,theme_,true);
        game_label_fitted(r,{site_info.x+12,site_info.y+54,site_info.w-24,17},
                          compact?"WHEEL ZOOM / MMB ORBIT / RMB PAN":"WHEEL ZOOM  |  MMB ORBIT  |  RMB PAN  |  W/A/S/D MOVE",
                          compact?0.30f:0.35f,theme_,true);
        if(ui::button(r,ui_input_,{site_info.x+12,site_info.y+(compact?78.0f:88.0f),site_info.w-24,26},
                      guidance.action,theme_,planning_.mission_step!=MissionPreparationStep::Complete)){
            planning_.view=guidance.view;
            if(guidance.contract>=0)planning_contract_selected_=guidance.contract;
            if(guidance.technology>=0)planning_tech_selected_=guidance.technology;
            if(guidance.view==PlanningView::Mission)planning_.mission_facility=guidance.facility;
            planning_status_message_=guidance.reason;
        }

        // World-space labels stay attached to each facility as the player
        // orbits and pans around the 3D site.
        for(std::size_t i=0;i<kCampusBuildings.size();++i){
            const auto& b=kCampusBuildings[i];
            float sx=0.0f,sy=0.0f;
            const float label_height=std::max(5.0f,b.footprint.y+2.0f);
            if(!project_campus_label(planning_vp,{b.center.x,label_height,b.center.z},r.width(),r.height(),ui_scale_,sx,sy)) continue;
            if(sy<top_h+3.0f || sy>logical_h-bottom_h-22.0f) continue;
            const bool hovered=(int)i==campus_hover;
            const bool required=b.facility==next_facility;
            const float label_w=compact?132.0f:174.0f;
            const float label_h=(hovered||required)?(compact?38.0f:43.0f):(compact?21.0f:24.0f);
            Rect lr{sx-label_w*0.5f,sy-label_h*0.5f,label_w,label_h};
            lr.x=std::clamp(lr.x,2.0f,std::max(2.0f,logical_w-lr.w-2.0f));
            r.draw_rect(lr,hovered?Color{0.035f,0.105f,0.125f,0.95f}:required?Color{0.040f,0.090f,0.050f,0.94f}:Color{0.020f,0.030f,0.040f,0.84f});
            game_label_fitted(r,{lr.x+5,lr.y+4,lr.w-10,14},b.name,compact?0.25f:0.31f,theme_);
            if(hovered||required) game_label_fitted(r,{lr.x+5,lr.y+21,lr.w-10,14},
                                          hovered?"OPEN FACILITY":std::string("NEXT: ")+mission_step_name(planning_.mission_step),compact?0.23f:0.28f,theme_,true);
        }

        if(campus_hover>=0){
            const auto& b=kCampusBuildings[(std::size_t)campus_hover];
            const float w=compact?360.0f:500.0f;
            Rect hover_card{logical_w-w-margin,content_y,w,compact?74.0f:88.0f};
            r.draw_rect(hover_card,{0.020f,0.030f,0.040f,0.94f});
            game_label_fitted(r,{hover_card.x+12,hover_card.y+9,hover_card.w-24,18},b.name,compact?0.42f:0.52f,theme_);
            game_label_fitted(r,{hover_card.x+12,hover_card.y+31,hover_card.w-24,18},b.subtitle,compact?0.31f:0.37f,theme_,true);
            game_label_fitted(r,{hover_card.x+12,hover_card.y+52,hover_card.w-24,16},
                              std::string("LMB -> FACILITY WORKFLOW")+(b.facility==mission_step_facility(planning_.mission_step)?"  [NEXT REQUIRED]":""),compact?0.29f:0.34f,theme_,true);
        }
    }else if(planning_.view==PlanningView::Contracts){
        // Contracts are customer projects, not free money timers: a clear
        // award/deposit -> work -> delivery/review -> final payout sequence.
        // Completed technical evidence also makes the related research cheaper.
        const float gap=12.0f;
        const float list_w=std::floor((logical_w-2*margin-gap)*0.48f);
        Rect board{margin,content_y,list_w,content_h};
        Rect brief{board.x+board.w+gap,content_y,logical_w-margin-(board.x+board.w+gap),content_h};
        ui::panel(r,board,theme_);
        ui::panel(r,brief,theme_);
        game_label_fitted(r,{board.x+14,board.y+9,board.w-28,23},"CUSTOMER CONTRACTS",compact?0.58f:0.73f,theme_);
        const auto budget=campaign_budget_audit();
        const auto minimum_manifest=planning_reference_mission_cargo();
        std::int64_t minimum_delivery_cost=0;
        for(std::size_t ri=0;ri<kMaterialResourceCount;++ri)
            minimum_delivery_cost+=minimum_manifest.amount((MaterialResource)ri)*
                (kResourcePurchaseEurPerKg[ri]+kCargoIntegrationEurPerKg);
        const auto total_required=budget.full_research_cost+budget.maximum_vehicle_cost+minimum_delivery_cost;
        game_label_fitted(r,{board.x+14,board.y+36,board.w-28,17},
                          "REFERENCE PROGRAM COST " + money(total_required) + "  /  CONTRACT FUNDING " +
                          money(budget.commercial_income) + "  [EXCLUDES LAUNCH BONUS]",
                          0.35f,theme_,true);
        // Overview of the relationship to the rest of the first stage.
        const float guide_y=board.y+62;
        const float guide_gap=4.0f;
        const float guide_w=(board.w-28-guide_gap*2)/3.0f;
        for(int i=0;i<3;++i){
            const Rect badge{board.x+14+i*(guide_w+guide_gap),guide_y,guide_w,26.0f};
            r.draw_rect(badge,i==0?Color{0.055f,0.115f,0.128f,1}:Color{0.043f,0.066f,0.085f,1});
            game_label_fitted(r,{badge.x+4,badge.y+6,badge.w-8,15},
                              i==0?"1  CONTRACT":i==1?"2  RESEARCH":"3  LUNAR MISSION",0.35f,theme_);
        }
        const float header_y=board.y+96;
        const int visible_rows=compact?5:6;
        const int page_count=((int)kContracts.size()+visible_rows-1)/visible_rows;
        planning_contract_page_=std::clamp(planning_contract_page_,0,page_count-1);
        game_label_fitted(r,{board.x+14,header_y+7,board.w-190,19},
                          "PROJECTS  /  "+std::to_string(active_contract_count(planning_))+" OF 3 ACTIVE  /  "+std::to_string(unique_deliveries(planning_))+" CLIENTS",0.35f,theme_,true);
        if(ui::button(r,ui_input_,{board.x+board.w-164,header_y,70,30},"PREV",theme_,planning_contract_page_>0)) --planning_contract_page_;
        if(ui::button(r,ui_input_,{board.x+board.w-86,header_y,70,30},"NEXT",theme_,planning_contract_page_+1<page_count)) ++planning_contract_page_;
        // Keep stable save IDs; reorder only their screen presentation so
        // contracts ready for hand-in are never hidden behind locked ones.
        std::array<std::size_t,kPlanningContractCount> order{};
        for(std::size_t i=0;i<order.size();++i) order[i]=i;
        std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){
            return contract_priority(planning_,a)<contract_priority(planning_,b);
        });
        const float row_start=header_y+36;
        const float row_h=std::max(42.0f,(board.y+board.h-row_start-10.0f)/(float)visible_rows);
        for(int ri=0;ri<visible_rows;++ri){
            const int ordinal=planning_contract_page_*visible_rows+ri;
            if(ordinal>=(int)order.size())break;
            const std::size_t i=order[(std::size_t)ordinal];
            const auto& c=kContracts[i];
            const auto& ps=planning_.contracts[i];
            const bool selected=planning_contract_selected_==(int)i;
            const bool ready=contract_ready_for_delivery(planning_,i);
            const bool done=ps.status==PlanningContractStatus::Completed || ps.completed_count>0;
            const bool active=ps.status==PlanningContractStatus::Active && !done;
            const bool available=planning_contract_available(i);
            const Color state_color=done?Color{0.20f,0.78f,0.46f,1.0f}:
                                    ready?Color{0.99f,0.81f,0.32f,1.0f}:
                                    active?Color{0.33f,0.76f,0.98f,1.0f}:
                                    available?Color{0.50f,0.86f,0.98f,1.0f}:
                                    Color{0.45f,0.49f,0.55f,1.0f};
            Rect row{board.x+11,row_start+ri*row_h,board.w-22,row_h-4};
            r.draw_rect(row,selected?Color{0.060f,0.115f,0.143f,0.98f}:done?Color{0.030f,0.076f,0.055f,0.97f}:
                        ready?Color{0.086f,0.075f,0.038f,0.97f}:active?Color{0.035f,0.069f,0.093f,0.97f}:Color{0.040f,0.053f,0.068f,0.94f});
            // Always show a state-colored side rail and pill, even when selected.
            r.draw_rect({row.x,row.y,4,row.h},state_color);
            const std::string state=done?"COMPLETED - CLOSED":ready?"READY FOR DELIVERY":active?
                (i==kContracts.size()-1?"ACTIVE - AWAITING LAUNCH":"ACTIVE - IN PROGRESS"):
                available?"AVAILABLE":"LOCKED - RESEARCH";
            const float pill_w=std::min(170.0f,row.w*0.52f);
            r.draw_rect({row.x+8,row.y+row.h-23,pill_w,20},
                        Color{state_color.r*0.20f,state_color.g*0.20f,state_color.b*0.20f,1.0f});
            r.draw_rect({row.x+8,row.y+row.h-23,3,20},state_color);
            game_label_fitted(r,{row.x+10,row.y+5,row.w-20,18},c.name,compact?0.43f:0.50f,theme_);
            game_label_fitted(r,{row.x+14,row.y+row.h-19,pill_w-8,16},state,0.31f,theme_,true);
            game_label_fitted(r,{row.x+row.w*0.57f,row.y+row.h-19,row.w*0.40f,16},
                              "TOTAL "+money(c.deposit_eur+c.finish_eur),0.34f,theme_,true);
            if(active && i!=kContracts.size()-1){
                const float frac=(float)std::clamp(ps.progress_days/c.duration_days,0.0,1.0);
                r.draw_rect({row.x+10,row.y+row.h-3,row.w-20,2},{0.1f,0.14f,0.18f,1});
                r.draw_rect({row.x+10,row.y+row.h-3,(row.w-20)*frac,2},{0.25f,0.72f,0.82f,1});
            }
            if(ui_input_.left_pressed&&row.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y)){
                // Single click opens the brief; double-click performs the same
                // action as the primary button, always using its real guards.
                const bool activate=planning_double_clicked(1,(int)i);
                if(planning_contract_selected_!=(int)i) planning_contract_research_page_=0;
                planning_contract_selected_=(int)i;
                if(activate){
                    if(ready) deliver_planning_contract(i);
                    else if(available&&active_contract_count(planning_)<kPlanningMaxActiveContracts)
                        accept_planning_contract(i);
                    else if(active&&!done&&i!=kContracts.size()-1)
                        advance_planning_to_next_report();
                    else planning_status_message_=done?"Contract completed - this job cannot be repeated.":
                        "This contract is locked or all project slots are occupied.";
                }
            }
        }

        const std::size_t selected=(std::size_t)std::clamp(planning_contract_selected_,0,(int)kContracts.size()-1);
        const auto& c=kContracts[selected];
        const auto& ps=planning_.contracts[selected];
        const bool ready=contract_ready_for_delivery(planning_,selected);
        const bool completed=ps.status==PlanningContractStatus::Completed || ps.completed_count>0;
        const bool lunar_customer=selected==kContracts.size()-1;
        const float x=brief.x+15.0f, max_w=brief.w-30.0f;
        r.draw_rect({brief.x+3,brief.y+4,4,60},{0.24f,0.78f,0.93f,1});
        game_label_fitted(r,{x,brief.y+9,max_w,21},"SELECTED PROJECT  /  DETAILS",compact?0.55f:0.69f,theme_);
        game_label_fitted(r,{x,brief.y+35,max_w,21},c.name,compact?0.53f:0.69f,theme_);
        game_label_fitted(r,{x,brief.y+58,max_w,15},"CUSTOMER  "+std::string(c.customer),0.35f,theme_,true);
        game_label_fitted(r,{x,brief.y+77,max_w,16},c.description,0.39f,theme_,true);
        r.draw_rect({x,brief.y+99,max_w,1},{0.18f,0.36f,0.45f,0.8f});
        game_label_fitted(r,{x,brief.y+105,max_w,16},"CUSTOMER DELIVERABLE",0.38f,theme_);
        game_label_fitted(r,{x,brief.y+125,max_w,17},c.deliverable,0.37f,theme_,true);
        game_label_fitted(r,{x,brief.y+147,max_w,16},"HOW THIS ADVANCES YOUR MOON BASE",0.38f,theme_);
        if(c.supported_tech>=0){
            const auto ti=(std::size_t)c.supported_tech;
            game_label_fitted(r,{x,brief.y+166,max_w,16},
                std::string("Research evidence: ")+kTechs[ti].short_name+
                (ps.completed_count?"  /  DISCOUNT EARNED":"  /  20% SAVINGS ON DELIVERY"),
                0.34f,theme_,true);
        }else{
            game_label_fitted(r,{x,brief.y+166,max_w,16},
                "FINAL LUNAR DELIVERY / PAYMENT ON ACTUAL LAUNCH",0.36f,theme_,true);
        }
        // Mandatory research is visible for EVERY selected contract, including
        // completed and active ones. Show indirect prerequisites as well, so
        // the entire research route to each customer project is understandable.
        const auto requirements=contract_research_requirements(selected);
        const int total=(int)requirements.size();
        int researched=0;
        for(const auto& req:requirements) if(planning_.technologies[req.tech]) ++researched;
        const float req_title_y=brief.y+186.0f;
        game_label_fitted(r,{x,req_title_y,max_w-112,17},
                          "REQUIRED RESEARCH  "+std::to_string(researched)+" / "+std::to_string(total)+" COMPLETE",
                          0.39f,theme_);
        const float req_start_y=brief.y+208.0f;
        const float req_row_h=19.0f;
        const float pay_y=std::max(brief.y+225,brief.y+brief.h-141);
        const int req_rows=std::max(0,(int)std::floor((pay_y-req_start_y-3.0f)/req_row_h));
        const int page_size=std::max(1,req_rows);
        const int req_pages=std::max(1,(total+page_size-1)/page_size);
        planning_contract_research_page_=std::clamp(planning_contract_research_page_,0,req_pages-1);
        if(req_pages>1){
            if(ui::button(r,ui_input_,{x+max_w-109,req_title_y-2,48,21},"<",theme_,planning_contract_research_page_>0))
                --planning_contract_research_page_;
            if(ui::button(r,ui_input_,{x+max_w-55,req_title_y-2,48,21},">",theme_,planning_contract_research_page_+1<req_pages))
                ++planning_contract_research_page_;
        }
        if(total==0){
            if(req_rows>0){
                r.draw_rect({x,req_start_y,max_w,17},{0.045f,0.12f,0.10f,0.95f});
                const std::string label="NO RESEARCH REQUIRED - OPEN FROM DAY ONE";
                const float scale=ui::fit_text(label,ui::font_scale(0.40f*GAME_FONT_BOOST,theme_),{max_w-12,16});
                r.draw_text({x+7,req_start_y+1},label,scale,{0.35f,0.94f,0.62f,1});
            }
        }else if(req_rows>0){
            const int begin=planning_contract_research_page_*page_size;
            for(int i=0;i<req_rows && begin+i<total;++i){
                const auto& req=requirements[(std::size_t)(begin+i)];
                const bool unlocked=planning_.technologies[req.tech]!=0;
                const float y=req_start_y+(float)i*req_row_h;
                const Color tint=unlocked?Color{0.36f,0.97f,0.58f,1}:
                                           Color{1.0f,0.31f,0.32f,1};
                r.draw_rect({x,y,max_w,req_row_h-2},unlocked?Color{0.035f,0.105f,0.068f,1}:
                                                              Color{0.18f,0.043f,0.052f,1});
                r.draw_rect({x,y,4,req_row_h-2},tint);
                const std::string label=std::string(unlocked?"RESEARCHED  ":"MISSING  ")+
                    (req.direct?"[DIRECT]  ":"[PREREQUISITE]  ")+kTechs[req.tech].short_name;
                const float scale=ui::fit_text(label,ui::font_scale(0.38f*GAME_FONT_BOOST,theme_),
                                               {max_w-14,req_row_h-3});
                r.draw_text({x+10,y+2},label,scale,tint);
            }
        }
        // Anchor all payment and action controls relative to the panel bottom
        // so a 2x scaled 1280x720 layout remains fully clickable.
        r.draw_rect({x,pay_y-7,max_w,1},{0.18f,0.36f,0.45f,0.8f});
        const bool active=ps.status==PlanningContractStatus::Active && !completed;
        const auto premium=first_delivery_bonus(planning_,selected);
        game_label_fitted(r,{x,pay_y,max_w,18},
                          "ADVANCE  "+money(c.deposit_eur)+"   DELIVERY  "+money(c.finish_eur+premium),0.44f,theme_);
        std::string state;
        if(ready)state="READY: deliver the customer report for payment.";
        else if(active)state=lunar_customer?"DELIVERY IS VERIFIED AT THE LAUNCH COMPLEX.":
            "EXECUTION  "+days_text(ps.progress_days)+" / "+days_text(c.duration_days);
        else if(completed)state="COMPLETED - CLOSED PERMANENTLY / REWARDS ALREADY CLAIMED.";
        else if(planning_contract_available(selected))state=active_contract_count(planning_)>=kPlanningMaxActiveContracts?
             "3 PROJECT SLOTS FULL - SUBMIT ACTIVE WORK FIRST":"AVAILABLE - up to 3 contracts can run concurrently.";
        else if(lunar_customer&&planning_.mission_step==MissionPreparationStep::Complete)
            state="MISSION ALREADY LAUNCHED - SPONSORSHIP CLOSED.";
        else state="FIRST RESEARCH: "+missing_contract_research(planning_,selected);
        game_label_fitted(r,{x,pay_y+24,max_w,18},state,0.39f,theme_,true);
        if(!lunar_customer)
            game_label_fitted(r,{x,pay_y+44,max_w,15},completed?"ONE-TIME CONTRACT - RESEARCH BENEFIT RETAINED":
                                    premium?"INCLUDES +20% NEW CUSTOMER PREMIUM / FIRST DELIVERY":
                                    "ONE-TIME CONTRACT - SIGN ONCE, DELIVER ONCE",0.29f,theme_,true);
        const float button_y=brief.y+brief.h-75.0f;
        // Primary-action region always occupies the same location.
        r.draw_rect({x-3,button_y-3,max_w+6,37},
                    ready?Color{0.13f,0.19f,0.075f,1}:
                    completed?Color{0.05f,0.095f,0.075f,1}:
                    Color{0.055f,0.135f,0.18f,1});
        if(ready){
            if(ui::button(r,ui_input_,{x,button_y,max_w,31},"SUBMIT REPORT / CLAIM PAYMENT",theme_))
                deliver_planning_contract(selected);
        }else if(active){
            if(lunar_customer) ui::button(r,ui_input_,{x,button_y,max_w,31},"AWAITING MISSION LAUNCH",theme_,false);
            else{
                int finished=-1;
                for(std::size_t i=0;i<kContracts.size()-1;++i)
                    if(contract_ready_for_delivery(planning_,i)){finished=(int)i;break;}
                if(finished>=0){
                    if(ui::button(r,ui_input_,{x,button_y,max_w,31},"OPEN READY REPORT",theme_))planning_contract_selected_=finished;
                }else if(ui::button(r,ui_input_,{x,button_y,max_w,31},"ADVANCE TO NEXT REPORT",theme_)) advance_planning_to_next_report();
            }
        }else if(ps.status==PlanningContractStatus::Completed){
            ui::button(r,ui_input_,{x,button_y,max_w,31},"CUSTOMER DELIVERY COMPLETE",theme_,false);
        }else{
            if(ui::button(r,ui_input_,{x,button_y,max_w,31},"ACCEPT CONTRACT / RECEIVE ADVANCE",theme_,
                          planning_contract_available(selected)&&active_contract_count(planning_)<kPlanningMaxActiveContracts))
                accept_planning_contract(selected);
        }
        const float link_y=brief.y+brief.h-38.0f;
        const float link_w=(max_w-8)/2;
        if(ui::button(r,ui_input_,{x,link_y,link_w,30},"OPEN TECH TREE",theme_)){
            planning_.view=PlanningView::Technology;
            if(c.supported_tech>=0)planning_tech_selected_=c.supported_tech;
        }
        if(ui::button(r,ui_input_,{x+link_w+8,link_y,link_w,30},"RESUME MISSION",theme_)){
            planning_.view=PlanningView::Mission; // last station and manifest are preserved
        }
    }else if(planning_.view==PlanningView::Technology){
        render_planning_technology_tree(r,{margin,content_y,logical_w-margin*2,content_h},compact);
    }else{
        Rect mission{margin,content_y,logical_w-margin*2,content_h};
        ui::panel(r,mission,theme_);
        game_label(r,{mission.x+18,mission.y+10},"MISSION PREPARATION  /  "+std::string(mission_step_name(planning_.mission_step)),compact?0.55f:0.70f,theme_);
        game_label_fitted(r,{mission.x+18,mission.y+35,mission.w-36,17},
                          "Every flight advances through the facility chain. Complete the current gate before the next facility can act.",compact?0.32f:0.39f,theme_,true);

        constexpr int workflow_steps=10;
        const float flow_y=mission.y+58.0f;
        const int flow_cols=compact?5:10;
        const int flow_rows=compact?2:1;
        const float flow_gap=compact?4.0f:6.0f;
        const float flow_w=(mission.w-32.0f-flow_gap*(flow_cols-1))/(float)flow_cols;
        const float flow_h=compact?31.0f:38.0f;
        const std::array<MissionPreparationStep,workflow_steps> flow{{
            MissionPreparationStep::Research,MissionPreparationStep::Design,MissionPreparationStep::Test,
            MissionPreparationStep::Refine,MissionPreparationStep::Plan,MissionPreparationStep::Confirm,
            MissionPreparationStep::Build,MissionPreparationStep::Integrate,MissionPreparationStep::FinalConfirm,
            MissionPreparationStep::Launch
        }};
        for(int i=0;i<workflow_steps;++i){
            const int col=i%flow_cols,row=i/flow_cols;
            const Rect card{mission.x+16.0f+(flow_w+flow_gap)*(float)col,flow_y+(flow_h+4.0f)*(float)row,flow_w,flow_h};
            const auto step=flow[(std::size_t)i];
            const bool done=(std::uint8_t)planning_.mission_step>(std::uint8_t)step;
            const bool current=planning_.mission_step==step;
            r.draw_rect(card,current?Color{0.055f,0.115f,0.125f,0.98f}:done?Color{0.045f,0.082f,0.060f,0.96f}:Color{0.040f,0.047f,0.055f,0.96f});
            r.draw_rect({card.x,card.y,card.w,3},done?Color{0.20f,0.78f,0.46f,1.0f}:
                        current?Color{0.25f,0.81f,1.0f,1.0f}:Color{0.36f,0.40f,0.48f,1.0f});
            if(current && ui_input_.left_pressed && card.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y)){
                planning_.mission_facility=mission_step_facility(step);
                planning_status_message_=std::string("Current step: ")+mission_step_name(step)+
                    ". Complete its action in the facility panel.";
            }
            game_label_fitted(r,{card.x+4,card.y+5,card.w-8,13},mission_step_name(step),compact?0.22f:0.28f,theme_);
            if(!compact) game_label_fitted(r,{card.x+4,card.y+21,card.w-8,11},done?"DONE":current?"CURRENT":"LOCKED",0.20f,theme_,true);
        }

        const float body_y=flow_y+(flow_h+4.0f)*(float)flow_rows+8.0f;
        const float body_h=mission.y+mission.h-body_y-12.0f;
        const float facility_w=compact?230.0f:std::min(360.0f,mission.w*0.31f);
        const float facility_h=body_h;
        Rect facility{mission.x+16.0f,body_y,facility_w,facility_h};
        Rect work{facility.x+facility.w+10.0f,body_y,mission.x+mission.w-16.0f-(facility.x+facility.w+10.0f),body_h};
        r.draw_rect(facility,{0.030f,0.041f,0.052f,0.97f});
        r.draw_rect(work,{0.034f,0.046f,0.059f,0.97f});

        const auto required_facility=mission_step_facility(planning_.mission_step);
        const bool mission_complete=planning_.mission_step==MissionPreparationStep::Complete;
        const auto selected_facility=planning_.mission_facility;
        game_label_fitted(r,{facility.x+12,facility.y+10,facility.w-24,18},mission_facility_name(selected_facility),compact?0.40f:0.50f,theme_);

        auto facility_role=[&](MissionFacility f)->const char*{
            switch(f){
                case MissionFacility::Overview:return "Program overview and ordered preparation gates.";
                case MissionFacility::ProgramOffice:return "Contracts, budget authority and formal mission commitment.";
                case MissionFacility::ResearchEngineering:return "Research closure, preliminary design and post-test refinement.";
                case MissionFacility::VehicleAssembly:return "Manufacture and assemble the funded rocket and lunar lander.";
                case MissionFacility::CargoIntegration:return "Procure, inspect, load and seal the lunar resource manifest.";
                case MissionFacility::MissionControl:return "Freeze the cargo manifest and operational lunar flight plan.";
                case MissionFacility::Communications:return "Verify tracking, telemetry and deep-space communication readiness.";
                case MissionFacility::PropulsionTest:return "Qualify propulsion, stage interfaces and cryogenic systems.";
                case MissionFacility::LaunchComplex:return "Perform pad release and launch the completed mission.";
            }
            return "Mission preparation facility.";
        };
        game_label_fitted(r,{facility.x+12,facility.y+34,facility.w-24,compact?30.0f:38.0f},facility_role(selected_facility),compact?0.29f:0.34f,theme_,true);
        game_label_fitted(r,{facility.x+12,facility.y+(compact?68.0f:82.0f),facility.w-24,16},
                          mission_complete?"MISSION STATUS: LAUNCHED":std::string("CURRENT GATE: ")+mission_step_name(planning_.mission_step),compact?0.28f:0.34f,theme_,!mission_complete);
        if(!mission_complete) game_label_fitted(r,{facility.x+12,facility.y+(compact?88.0f:104.0f),facility.w-24,30},
                          std::string("REQUIRED FACILITY: ")+mission_facility_name(required_facility),compact?0.25f:0.30f,theme_,true);
        if(!mission_complete && facility.h>230.0f){
            const float next_y=facility.y+facility.h-118.0f;
            r.draw_rect({facility.x+9,next_y-3,facility.w-18,36},{0.066f,0.17f,0.20f,1});
            if(ui::button(r,ui_input_,{facility.x+12,next_y,facility.w-24,30},
                          "GO TO NEXT FACILITY",theme_)){
                planning_.mission_facility=required_facility;
                planning_status_message_=std::string("Ready to continue at ")+mission_facility_name(required_facility)+".";
            }
        }
        if(ui::button(r,ui_input_,{facility.x+12,facility.y+facility.h-78,facility.w-24,30},"MISSION OVERVIEW",theme_))
            planning_.mission_facility=MissionFacility::Overview;
        if(ui::button(r,ui_input_,{facility.x+12,facility.y+facility.h-42,facility.w-24,30},compact?"COMPANY SITE":"RETURN TO COMPANY SITE",theme_)) planning_.view=PlanningView::Site;

        const auto required_cargo=planning_structure_requirements();
        const auto rocket_index=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1);
        const auto lander_index=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
        const auto& rocket=kRockets[rocket_index];
        const auto& lander=kLanders[lander_index];
        const bool cargo_ready=planning_cargo_covers_structures();
        const bool capacity_ready=planning_mission_payload_kg()<=planning_mission_payload_capacity_kg();

        auto headline=[&](const std::string& title,const std::string& text){
            game_label_fitted(r,{work.x+14,work.y+10,work.w-28,18},title,compact?0.40f:0.52f,theme_);
            game_label_fitted(r,{work.x+14,work.y+34,work.w-28,compact?34.0f:42.0f},text,compact?0.28f:0.34f,theme_,true);
        };
        auto action_button=[&](const std::string& label,bool enabled,float y){
            return ui::button(r,ui_input_,{work.x+14,y,std::min(280.0f,work.w-28.0f),34},label,theme_,enabled);
        };
        auto draw_vehicle_selector=[&](float y,bool editable){
            const float row_h=compact?44.0f:58.0f;
            auto select=[&](const char* kind,const char* name,std::uint8_t& value,std::size_t count,float yy){
                game_label_fitted(r,{work.x+14,yy,100,16},kind,compact?0.30f:0.36f,theme_,true);
                if(ui::button(r,ui_input_,{work.x+116,yy-2,34,30},"<",theme_,editable&&value>0)) --value;
                game_label_fitted(r,{work.x+156,yy,work.w-220,18},name,compact?0.30f:0.38f,theme_);
                if(ui::button(r,ui_input_,{work.x+work.w-50,yy-2,34,30},">",theme_,editable&&(std::size_t)value+1<count)) ++value;
            };
            select("ROCKET",kRockets[std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1)].name,planning_.mission_rocket,kRockets.size(),y);
            select("LANDER",kLanders[std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1)].name,planning_.mission_lander,kLanders.size(),y+row_h);
        };
        auto draw_manifest=[&](float y,bool editable){
            const int cols=compact?4:2,rows=compact?2:4;
            const float gap=6.0f;
            const float col_w=(work.w-28.0f-gap*(float)(cols-1))/(float)cols;
            const float row_h=compact?34.0f:43.0f;
            if(editable){
                if(ui::button(r,ui_input_,{work.x+work.w-230,y-2,128,28},"AUTO-ADD CARGO",theme_,!required_cargo.empty()))
                    planning_add_required_cargo();
                if(ui::button(r,ui_input_,{work.x+work.w-94,y-2,78,28},"CLEAR",theme_)){ planning_.mission_cargo={}; planning_status_message_="Mission cargo cleared."; }
            }
            const float bot_y=y+30.0f;
            for(int role=0;role<2;++role){
                auto& count=role==0?planning_.mission_builders:planning_.mission_carriers;
                const float x=work.x+14+(work.w-28)*0.5f*(float)role;
                const float w=(work.w-28)*0.5f-6;
                game_label_fitted(r,{x,bot_y+6,w-72,14},std::string(role==0?"BUILDERS  ":"CARRIERS  ")+std::to_string(count),compact?0.25f:0.32f,theme_);
                if(ui::button(r,ui_input_,{x+w-68,bot_y,30,25},"-",theme_,editable&&count>1))--count;
                if(ui::button(r,ui_input_,{x+w-34,bot_y,30,25},"+",theme_,editable&&count<6))++count;
            }
            const float top=y+62.0f;
            for(std::size_t i=0;i<kMaterialResourceCount;++i){
                const int col=(int)i/rows,row=(int)i%rows;
                const auto m=(MaterialResource)i;
                const auto current=planning_.mission_cargo.amount(m),required=required_cargo.amount(m);
                const float x=work.x+14.0f+(col_w+gap)*(float)col;
                const float yy=top+row_h*(float)row;
                r.draw_rect({x,yy,col_w,row_h-4.0f},current>=required?Color{0.047f,0.071f,0.076f,0.96f}:Color{0.076f,0.050f,0.050f,0.96f});
                game_label_fitted(r,{x+6,yy+5,col_w-(editable?118.0f:12.0f),14},std::string(material_resource_name(m))+"  "+mass_text(current)+" / NEED "+mass_text(required),compact?0.23f:0.29f,theme_,true);
                if(editable){
                    const std::int64_t step=(m==MaterialResource::Food || m==MaterialResource::Medical)?100:1000;
                    if(ui::button(r,ui_input_,{x+col_w-108,yy+4,48,row_h-12},"-",theme_,current>0)) planning_.mission_cargo.amount(m)=std::max<std::int64_t>(0,current-step);
                    if(ui::button(r,ui_input_,{x+col_w-54,yy+4,48,row_h-12},"+",theme_)) planning_.mission_cargo.amount(m)=std::min<std::int64_t>(999'999,current+step);
                }
            }
            const float sy=top+row_h*(float)rows+2.0f;
            game_label_fitted(r,{work.x+14,sy,work.w-28,16},"PAYLOAD "+mass_text(planning_mission_payload_kg())+" / "+mass_text(planning_mission_payload_capacity_kg())+"   |   MISSION "+money(planning_mission_cost_eur()),compact?0.27f:0.33f,theme_,true);
            return sy+22.0f;
        };

        const auto draw_structure_plan=[&](float y){
            std::vector<const BuildObjectDefinition*> structures;
            for(const auto& item:build_catalog_.items())
                if(item.kind==BuildObjectKind::Habitat||item.kind==BuildObjectKind::Resource) structures.push_back(&item);

            constexpr float row_h=23;
            const int rows=std::clamp((int)((work.y+work.h-y-86)/row_h),1,10);
            const int last_page=std::max(0,((int)structures.size()-1)/rows);
            planning_structure_page_=std::clamp(planning_structure_page_,0,last_page);
            const int first=planning_structure_page_*rows;
            for(int i=0;i<rows&&first+i<(int)structures.size();++i){
                const auto& item=*structures[(std::size_t)(first+i)];
                auto& quantity=item.kind==BuildObjectKind::Habitat?planning_.mission_habitats[(std::size_t)item.habitat_kind]
                    :planning_.mission_structures[(std::size_t)item.resource_kind];
                const float yy=y+row_h*(float)i;
                const float right=work.x+work.w-14;
                r.draw_rect({work.x+14,yy,work.w-28,row_h-2},quantity?Color{.051f,.101f,.105f,1}:theme_.panel2);
                game_label_fitted(r,{work.x+20,yy+3,work.w-140,14},item.name,.35f,theme_);
                if(ui::button(r,ui_input_,{right-99,yy,28,21},"-",theme_,quantity>0)) --quantity;
                game_label_fitted(r,{right-65,yy+3,30,14},std::to_string(quantity),.35f,theme_);
                if(ui::button(r,ui_input_,{right-29,yy,28,21},"+",theme_,quantity<12)) ++quantity;
            }
            const float footer=y+rows*row_h+5;
            if(ui::button(r,ui_input_,{work.x+14,footer,30,22},"<",theme_,planning_structure_page_>0)) --planning_structure_page_;
            if(ui::button(r,ui_input_,{work.x+100,footer,30,22},">",theme_,planning_structure_page_<last_page)) ++planning_structure_page_;
            game_label_fitted(r,{work.x+48,footer+4,48,14},std::to_string(planning_structure_page_+1)+"/"+std::to_string(last_page+1),.30f,theme_);
            const auto needed=planning_structure_requirements();
            if(ui::button(r,ui_input_,{work.x+work.w-152,footer,138,22},"AUTO-ADD CARGO",theme_,!needed.empty()))
                planning_add_required_cargo();
            std::string line="NEEDED: ";
            for(const auto material:kConstructionMaterialResources)
                line+=std::string(material_resource_symbol(material))+" "+std::to_string(needed.amount(material))+" KG  ";
            game_label_fitted(r,{work.x+14,footer+30,work.w-28,16},line,.35f,theme_);
            game_label_fitted(r,{work.x+14,footer+50,work.w-28,16},"STANDARD SIZES / SMALL STORAGE DOME / ADD LINKS AND SUPPLIES IN PAYLOAD",.28f,theme_,true);
        };

        if(selected_facility==MissionFacility::Overview){
            headline(mission_complete?"MISSION COMPLETE":"ORDERED FACILITY WORKFLOW",
                     mission_complete?"The funded robotic lunar-base mission has launched. Stage 1 preparation is complete.":
                     std::string("Next required action: ")+mission_step_name(planning_.mission_step)+" at "+mission_facility_name(required_facility)+". Campus buildings remain the primary navigation path.");
            float y=work.y+88.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},std::string("FLIGHT HARDWARE: ")+rocket.name+" + "+lander.name,compact?0.30f:0.38f,theme_); y+=25.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"LUNAR PAYLOAD: "+mass_text(planning_mission_payload_kg())+" / "+mass_text(planning_mission_payload_capacity_kg()),compact?0.30f:0.38f,theme_,true); y+=25.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},std::string("PROGRAM FUNDING: ")+(planning_.mission_paid?"AUTHORIZED / PAID":"NOT YET COMMITTED"),compact?0.30f:0.38f,theme_,true); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},
                              planning_.contracts.back().status==PlanningContractStatus::Active?
                              "CUSTOMER SPONSORSHIP: SIGNED - FINAL PAYMENT ON LAUNCH":
                              planning_.contracts.back().status==PlanningContractStatus::Completed?
                              "CUSTOMER SPONSORSHIP: DELIVERED / PAID":
                              "OPTIONAL: SIGN LUNAR CARGO CONTRACT BEFORE LAUNCH",compact?0.29f:0.34f,theme_,true); y+=30.0f;
            if(!mission_complete && action_button("CONTINUE AT REQUIRED FACILITY",true,y)){
                planning_.mission_facility=required_facility;
                planning_status_message_=std::string("Resuming mission at ")+mission_facility_name(required_facility)+
                    ". Previous preparation and cargo selections are saved.";
            }
        }else if(selected_facility==MissionFacility::ResearchEngineering){
            headline("RESEARCH & ENGINEERING","Owns mission research closure, preliminary rocket/lander design, and design refinement after propulsion testing.");
            if(ui::button(r,ui_input_,{work.x+work.w-142,work.y+8,128,28},"TECH TREE",theme_)) planning_.view=PlanningView::Technology;
            float y=work.y+82.0f;
            if(planning_.mission_step==MissionPreparationStep::Research){
                game_label_fitted(r,{work.x+14,y,work.w-28,18},planning_.technologies[14]?"STAGE-2 MODULE COMPETENCE: READY":"STAGE-2 MODULE COMPETENCE: REQUIRED",0.34f,theme_,!planning_.technologies[14]); y+=22.0f;
                game_label_fitted(r,{work.x+14,y,work.w-28,18},planning_.technologies[15]?"ROBOTIC LUNAR MISSION ARCHITECTURE: READY":"ROBOTIC LUNAR MISSION ARCHITECTURE: REQUIRED",0.34f,theme_,!planning_.technologies[15]); y+=34.0f;
                if(action_button("APPROVE RESEARCH BASELINE",planning_.technologies[14]&&planning_.technologies[15],y)) complete_planning_mission_step();
            }else if(planning_.mission_step==MissionPreparationStep::Design){
                draw_vehicle_selector(y,true); y+=compact?100.0f:132.0f;
                const auto rr=std::min<std::size_t>(planning_.mission_rocket,kRockets.size()-1),ll=std::min<std::size_t>(planning_.mission_lander,kLanders.size()-1);
                const bool design_ready=mask_unlocked(planning_,kRockets[rr].required_tech)&&mask_unlocked(planning_,kLanders[ll].required_tech);
                if(action_button("FREEZE PRELIMINARY DESIGN",design_ready,y)) complete_planning_mission_step();
            }else if(planning_.mission_step==MissionPreparationStep::Refine){
                game_label_fitted(r,{work.x+14,y,work.w-28,18},"QUALIFICATION TESTS: COMPLETE",0.36f,theme_); y+=24.0f;
                game_label_fitted(r,{work.x+14,y,work.w-28,36},"Apply test findings to margins, interfaces and operating limits, then baseline the flight design for Mission Control.",0.33f,theme_,true); y+=48.0f;
                if(action_button("REFINE & BASELINE DESIGN",true,y)) complete_planning_mission_step();
            }else{
                game_label_fitted(r,{work.x+14,y,work.w-28,36},std::string("No current gate here. Continue at ")+mission_facility_name(required_facility)+".",0.34f,theme_,true);
            }
        }else if(selected_facility==MissionFacility::PropulsionTest){
            headline("PROPULSION QUALIFICATION","Run the frozen rocket/lander design through engine, stage-interface, cryogenic and abort-margin qualification before refinement.");
            float y=work.y+88.0f;
            draw_vehicle_selector(y,false); y+=compact?100.0f:132.0f;
            if(planning_.mission_step==MissionPreparationStep::Test){ if(action_button("RUN QUALIFICATION TESTS  (+24 D)",true,y)) complete_planning_mission_step(); }
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("TEST gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }else if(selected_facility==MissionFacility::MissionControl){
            headline("MISSION CONTROL - FLIGHT PLAN","Plan lunar structures and load the materials needed to build them.");
            const bool editable=planning_.mission_step==MissionPreparationStep::Plan;
            if(editable){
                if(ui::button(r,ui_input_,{work.x+14,work.y+57,112,24},"PAYLOAD",theme_,planning_structure_page_>=0))
                    planning_structure_page_=-1;
                if(ui::button(r,ui_input_,{work.x+132,work.y+57,112,24},"STRUCTURES",theme_,planning_structure_page_<0))
                    planning_structure_page_=0;
            }
            float y=work.y+(editable?88.0f:76.0f);
            if(editable&&planning_structure_page_>=0){
                draw_structure_plan(y);
            }else{
                y=draw_manifest(y,editable);
                if(editable && action_button("FREEZE MISSION PLAN  (+7 D)",cargo_ready&&capacity_ready,y)) complete_planning_mission_step();
                else if(!editable) game_label_fitted(r,{work.x+14,y,work.w-28,28},std::string("PLAN gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.31f,theme_,true);
            }
        }else if(selected_facility==MissionFacility::ProgramOffice){
            headline("PROGRAM AUTHORIZATION","Review contracts, company funds and the frozen mission plan. Formal authorization commits the mission budget and unlocks hardware production.");
            if(ui::button(r,ui_input_,{work.x+work.w-142,work.y+8,128,28},"CONTRACTS",theme_)) planning_.view=PlanningView::Contracts;
            float y=work.y+88.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"MISSION TOTAL: "+money(planning_.mission_paid?planning_.mission_paid_cost_eur:planning_mission_cost_eur()),compact?0.32f:0.40f,theme_); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"AVAILABLE FUNDS: "+money(planning_.funds_eur),compact?0.32f:0.40f,theme_,true); y+=34.0f;
            if(planning_.mission_step==MissionPreparationStep::Confirm){ if(action_button("CONFIRM & FUND MISSION",planning_mission_requirements_met(),y)) purchase_planning_mission(); }
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("CONFIRM gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }else if(selected_facility==MissionFacility::VehicleAssembly){
            headline("VEHICLE ASSEMBLY","Manufacture and integrate the funded rocket and lunar lander. Vehicle hardware cost was committed during Program Authorization.");
            float y=work.y+92.0f;
            draw_vehicle_selector(y,false); y+=compact?100.0f:132.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},planning_.mission_paid?"PROGRAM FUNDING: RELEASED":"PROGRAM FUNDING: NOT RELEASED",0.34f,theme_,!planning_.mission_paid); y+=30.0f;
            if(planning_.mission_step==MissionPreparationStep::Build){ if(action_button("BUILD FLIGHT HARDWARE  (+45 D)",planning_.mission_paid,y)) complete_planning_mission_step(); }
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("BUILD gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }else if(selected_facility==MissionFacility::CargoIntegration){
            headline("CARGO INTEGRATION","Procure the frozen manifest, inspect every shipment, load the lander and seal the payload for launch processing.");
            float y=work.y+78.0f;
            y=draw_manifest(y,false);
            if(planning_.mission_step==MissionPreparationStep::Integrate){ if(action_button("INTEGRATE & SEAL CARGO  (+10 D)",cargo_ready&&capacity_ready,y)) complete_planning_mission_step(); }
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("INTEGRATE gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }else if(selected_facility==MissionFacility::Communications){
            headline("FINAL READINESS REVIEW","Verify tracking, telemetry, command links, lunar relay coverage and mission-control handover before the launch complex receives flight release.");
            float y=work.y+92.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"TRACKING NETWORK: CONFIGURED",0.35f,theme_); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"DEEP-SPACE TELEMETRY: CONFIGURED",0.35f,theme_); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"MISSION CONTROL HANDOVER: READY",0.35f,theme_); y+=34.0f;
            if(planning_.mission_step==MissionPreparationStep::FinalConfirm){ if(action_button("FINAL CONFIRMATION  (+2 D)",true,y)) complete_planning_mission_step(); }
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("FINAL CONFIRM gate is not active. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }else if(selected_facility==MissionFacility::LaunchComplex){
            headline("LAUNCH COMPLEX","The pad is the final gate. Launch is only released after research, design, qualification, planning, funding, build, cargo integration and final readiness confirmation.");
            float y=work.y+96.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},std::string("VEHICLE: ")+rocket.name,0.35f,theme_); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"LANDER: "+std::string(lander.name),0.35f,theme_); y+=24.0f;
            game_label_fitted(r,{work.x+14,y,work.w-28,18},"PAYLOAD: "+mass_text(planning_mission_payload_kg()),0.35f,theme_,true); y+=38.0f;
            if(planning_.mission_step==MissionPreparationStep::Launch){ if(action_button("LAUNCH & DEPLOY TO MOON",planning_.mission_paid&&cargo_ready&&capacity_ready,y)) complete_planning_mission_step(); }
            else if(mission_complete) game_label_fitted(r,{work.x+14,y,work.w-28,24},"MISSION LAUNCHED SUCCESSFULLY",0.40f,theme_);
            else game_label_fitted(r,{work.x+14,y,work.w-28,32},std::string("LAUNCH gate locked. Current gate: ")+mission_step_name(planning_.mission_step)+".",0.33f,theme_,true);
        }
    }

    if(!planning_status_message_.empty()){
        const float w=std::min(logical_w-2*margin,900.0f);
        const float x=(logical_w-w)*0.5f;
        const float y=logical_h-bottom_h-30.0f;
        r.draw_rect({x,y,w,24},{0.028f,0.072f,0.083f,0.98f});
        r.draw_rect({x,y,3,24},{0.24f,0.80f,0.87f,1});
        game_label_fitted(r,{x+10,y+4,w-20,16},planning_status_message_,0.36f,theme_,true);
    }
}

// Stage-1 research navigation is a true zoomable DAG canvas, intentionally
// independent of 3D camera controls and all serialized simulation state.
// Geometry clipping is local to this view, so neither renderer backend needs
// an additional scissor API to keep dragged nodes inside the research panel.
void GameApp::render_planning_technology_tree(Renderer& r,Rect panel,bool compact){
    const Color navy{0.010f,0.026f,0.061f,1},navy2{0.017f,0.045f,0.088f,1};
    const Color cyan{0.22f,0.72f,0.99f,1},pale{0.80f,0.92f,1.0f,1};
    const Color soft{0.45f,0.61f,0.75f,1},green{0.32f,0.98f,0.70f,1};
    const float header_h=compact?69.0f:78.0f;
    const float details_h=compact?121.0f:142.0f;
    const Rect graph{panel.x+5.0f,panel.y+header_h,panel.w-10.0f,
                     std::max(85.0f,panel.h-header_h-details_h-9.0f)};
    const Rect detail{panel.x+5.0f,graph.y+graph.h+5.0f,panel.w-10.0f,
                      std::max(65.0f,panel.y+panel.h-graph.y-graph.h-10.0f)};
    r.draw_rect(panel,navy);
    r.draw_rect({panel.x+1,panel.y+1,panel.w-2,2},{0.09f,0.32f,0.51f,0.75f});
    r.draw_rect({panel.x+7,panel.y+7,4,32},{0.21f,0.76f,1.0f,0.92f});
    game_label_fitted(r,{panel.x+22,panel.y+9,std::min(490.0f,panel.w-350.0f),28},
                      "CHOOSE RESEARCH",compact?1.15f:1.50f,theme_);
    game_label_fitted(r,{panel.x+23,panel.y+38,std::min(600.0f,panel.w-355.0f),19},
                      "SELECT TO INSPECT  /  DOUBLE-CLICK TO UNLOCK",0.49f,theme_,true);
    const std::string progress=std::to_string(unlocked_tech_count(planning_))+" / 16 RESEARCHED";
    game_label_fitted(r,{panel.x+23,panel.y+header_h-17,std::min(270.0f,panel.w-350.0f),13},progress,0.50f,theme_,true);

    const float controls_y=panel.y+17.0f;
    const float controls_x=panel.x+panel.w-330.0f;
    if(ui::button(r,ui_input_,{controls_x,controls_y,64,33},"-",theme_)) planning_tech_zoom_=std::max(0.24f,planning_tech_zoom_/1.20f);
    if(ui::button(r,ui_input_,{controls_x+69,controls_y,64,33},"+",theme_)) planning_tech_zoom_=std::min(1.70f,planning_tech_zoom_*1.20f);
    constexpr float node_w=248.0f,node_h=96.0f,col_step=324.0f,row_step=149.0f;
    constexpr float world_w=8.0f*col_step+18.0f,world_h=3.0f*row_step+64.0f;
    if(ui::button(r,ui_input_,{controls_x+138,controls_y,88,33},"FIT ALL",theme_)){
        planning_tech_zoom_=std::clamp(std::min((graph.w-55.0f)/world_w,(graph.h-26.0f)/world_h),0.24f,1.20f);
        planning_tech_pan_x_=0;planning_tech_pan_y_=0;
    }
    if(ui::button(r,ui_input_,{controls_x+232,controls_y,87,33},"FOCUS",theme_)){
        const auto& t=kTechs[(std::size_t)std::clamp(planning_tech_selected_,0,15)];
        planning_tech_zoom_=std::max(planning_tech_zoom_,0.78f);
        planning_tech_pan_x_=(graph.w*0.5f-22.0f)/planning_tech_zoom_-(50.0f+t.level*col_step+node_w*0.5f);
        planning_tech_pan_y_=(graph.h*0.5f-10.0f)/planning_tech_zoom_-(60.0f+t.lane*row_step+node_h*0.5f);
    }
    game_label_fitted(r,{controls_x,controls_y+39,317,14},
                      "WHEEL ZOOM  |  DRAG BACKGROUND / RIGHT DRAG PAN",0.29f,theme_,true);

    // Graph surface: a very dark space chart with restrained navigation grid,
    // distant stars and illuminated lanes, taking inspiration from the visual
    // language in the supplied screenshot without copying any of its assets.
    r.draw_rect(graph,navy2);
    const std::array<Color,3> bands{{
        Color{0.015f,0.045f,0.086f,1},Color{0.020f,0.052f,0.089f,1},
        Color{0.015f,0.039f,0.078f,1}}};
    for(int i=0;i<3;++i){
        const float y=graph.y+graph.h*(float)i/3.0f;
        r.draw_rect({graph.x,y,graph.w,graph.h/3.0f},bands[(std::size_t)i]);
    }
    for(int i=0;i<9;++i){
        const float x=graph.x+graph.w*(float)i/8.0f;
        r.draw_line({x,graph.y},{x,graph.y+graph.h},Color{0.24f,0.52f,0.72f,0.10f},1.0f);
    }
    for(int i=0;i<6;++i){
        const float y=graph.y+graph.h*(float)i/5.0f;
        r.draw_line({graph.x,y},{graph.x+graph.w,y},Color{0.25f,0.54f,0.74f,0.08f},1.0f);
    }
    for(unsigned i=0;i<92;++i){
        const unsigned hx=(i*1664525u+1013904223u),hy=(i*22695477u+12345u);
        const float x=graph.x+6.0f+(hx%1000u)/1000.0f*(graph.w-12.0f);
        const float y=graph.y+6.0f+(hy%1000u)/1000.0f*(graph.h-12.0f);
        const float radius=(i%9==0)?2.0f:1.0f;
        r.draw_rect({x,y,radius,radius},Color{0.47f,0.69f,0.98f,i%7==0?0.42f:0.22f});
    }
    // 2D graph/world transforms. Pan is stored in world units, preventing
    // speed changes during zooming and preserving the point beneath the mouse.
    const float inset=24.0f;
    const bool pointer_in_graph=graph.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y);
    const float old_zoom=planning_tech_zoom_;
    if(pointer_in_graph && std::abs(ui_input_.wheel)>0.001f){
        const float px=(float)ui_input_.mouse_x-graph.x-inset;
        const float py=(float)ui_input_.mouse_y-graph.y-inset;
        const float wx=px/old_zoom-planning_tech_pan_x_;
        const float wy=py/old_zoom-planning_tech_pan_y_;
        planning_tech_zoom_=std::clamp(old_zoom*std::pow(1.14f,ui_input_.wheel),0.24f,1.70f);
        planning_tech_pan_x_=px/planning_tech_zoom_-wx;
        planning_tech_pan_y_=py/planning_tech_zoom_-wy;
    }
    const float z=planning_tech_zoom_;
    // LMB drags empty space, RMB/MMB drags anywhere. Nodes stay clickable.
    auto location=[&](float wx,float wy){return Vec2{graph.x+inset+(wx+planning_tech_pan_x_)*z,
                                                   graph.y+inset+(wy+planning_tech_pan_y_)*z};};
    auto node_world=[&](std::size_t i){return Rect{26.0f+kTechs[i].level*col_step,
                                     36.0f+kTechs[i].lane*row_step,node_w,node_h};};
    std::array<Rect,kPlanningTechnologyCount> nodes{};
    for(std::size_t i=0;i<nodes.size();++i){
        Rect w=node_world(i);Vec2 p=location(w.x,w.y);nodes[i]={p.x,p.y,w.w*z,w.h*z};
    }
    bool on_node=false;
    if(pointer_in_graph){
        for(const Rect& n:nodes)if(n.contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y))on_node=true;
    }
    if(pointer_in_graph && ((ui_input_.right_down||ui_input_.middle_down)||
           (ui_input_.left_down&&!ui_input_.left_pressed&&!on_node))){
        planning_tech_pan_x_+=(float)ui_input_.mouse_dx/z;
        planning_tech_pan_y_+=(float)ui_input_.mouse_dy/z;
    }
    // Clamps allow the first and last levels to be brought into view, but
    // prevent losing the entire network outside its viewport.
    auto clamp_pan=[&](float& pan,float world,float viewport){
        const float visible=(viewport-2.0f*inset)/z;
        if(visible>=world){pan=(visible-world)*0.5f;return;}
        pan=std::clamp(pan,visible-world-50.0f,45.0f);
    };
    clamp_pan(planning_tech_pan_x_,world_w,graph.w);
    clamp_pan(planning_tech_pan_y_,world_h,graph.h);
    for(std::size_t i=0;i<nodes.size();++i){
        Rect w=node_world(i);Vec2 p=location(w.x,w.y);nodes[i]={p.x,p.y,w.w*z,w.h*z};
    }

    // Clip both rectangles and polylines to the research viewport. The common
    // renderer interface lacks scissor rectangles, but both the GL and D3D
    // backends support these same native 2D primitives.
    auto clipped_rect=[&](Rect b,Color c){
        const float x0=std::max(graph.x,b.x),y0=std::max(graph.y,b.y);
        const float x1=std::min(graph.x+graph.w,b.x+b.w),y1=std::min(graph.y+graph.h,b.y+b.h);
        if(x1>x0 && y1>y0)r.draw_rect({x0,y0,x1-x0,y1-y0},c);
    };
    auto clipped_line=[&](Vec2 a,Vec2 b,Color c,float thickness){
        float u0=0,u1=1;
        const float dx=b.x-a.x,dy=b.y-a.y;
        const std::array<float,4> pp{{-dx,dx,-dy,dy}};
        const std::array<float,4> qq{{a.x-graph.x,graph.x+graph.w-a.x,a.y-graph.y,graph.y+graph.h-a.y}};
        for(int j=0;j<4;++j){
            if(std::abs(pp[(std::size_t)j])<0.000001f){if(qq[(std::size_t)j]<0)return;}
            else {const float t=qq[(std::size_t)j]/pp[(std::size_t)j];
                if(pp[(std::size_t)j]<0)u0=std::max(u0,t);else u1=std::min(u1,t);
                if(u1<u0)return;}
        }
        const Vec2 q0{a.x+u0*dx,a.y+u0*dy},q1{a.x+u1*dx,a.y+u1*dy};
        r.draw_line(q0,q1,c,thickness);
    };
    auto outline=[&](Rect q,Color c,float thick){
        clipped_line({q.x,q.y},{q.x+q.w,q.y},c,thick);
        clipped_line({q.x+q.w,q.y},{q.x+q.w,q.y+q.h},c,thick);
        clipped_line({q.x+q.w,q.y+q.h},{q.x,q.y+q.h},c,thick);
        clipped_line({q.x,q.y+q.h},{q.x,q.y},c,thick);
    };
    auto label=[&](Rect target,const std::string& value,float scale,Color color){
        // Omit the text until its bounds fit inside the scrollable viewport;
        // do not let glyphs render over the fixed controls or detail panel.
        if(target.x<graph.x+1 || target.y<graph.y+1 ||
           target.x+target.w>graph.x+graph.w-1 || target.y+target.h>graph.y+graph.h-1)return;
        const float actual=ui::fit_text(value,scale,{target.w,target.h});
        if(actual>0.0f)r.draw_text({target.x,target.y},value,actual,color);
    };
    const std::array<const char*,8> tiers{{"FOUNDATIONS","ENABLING SYSTEMS","LAUNCH SYSTEMS",
        "LUNAR OPERATIONS","SURFACE TECHNOLOGY","ADVANCED SYSTEMS","MODULE COMPETENCE","MISSION READY"}};
    for(int tier=0;tier<8;++tier){
        Vec2 p=location(26.0f+tier*col_step,7.0f);
        label({p.x,p.y,node_w*z,18.0f*z},tiers[(std::size_t)tier],1.20f*z,Color{0.44f,0.77f,0.98f,0.83f});
    }

    // Render prerequisites behind the technology cards. Multiple inbound
    // edges are intentionally visible for multi-discipline requirements.
    for(std::size_t i=0;i<nodes.size();++i){
        for(std::size_t p=0;p<nodes.size();++p){
            if(!(kTechs[i].prerequisites&bit(p)))continue;
            const Rect a=nodes[p],b=nodes[i];
            const bool both=planning_.technologies[p]&&planning_.technologies[i];
            const bool ready=planning_.technologies[p]!=0;
            const Color lit=both?green:ready?cyan:Color{0.27f,0.39f,0.54f,1};
            const float mid=a.x+a.w+(b.x-a.x-a.w)*0.48f;
            const std::array<std::pair<Vec2,Vec2>,3> parts{{
                {Vec2{a.x+a.w,a.y+a.h*0.5f},Vec2{mid,a.y+a.h*0.5f}},
                {Vec2{mid,a.y+a.h*0.5f},Vec2{mid,b.y+b.h*0.5f}},
                {Vec2{mid,b.y+b.h*0.5f},Vec2{b.x,b.y+b.h*0.5f}}}};
            for(const auto& segment:parts){
                clipped_line(segment.first,segment.second,{lit.r,lit.g,lit.b,0.12f},5.2f*z);
                clipped_line(segment.first,segment.second,{lit.r,lit.g,lit.b,0.72f},1.6f*z);
            }
            const Vec2 end{b.x,b.y+b.h*0.5f};
            clipped_line({end.x-6*z,end.y-4*z},end,{lit.r,lit.g,lit.b,0.77f},1.45f*z);
            clipped_line({end.x-6*z,end.y+4*z},end,{lit.r,lit.g,lit.b,0.77f},1.45f*z);
        }
    }

    if(pointer_in_graph && ui_input_.left_pressed){
        for(std::size_t i=0;i<nodes.size();++i){
            if(nodes[i].contains((float)ui_input_.mouse_x,(float)ui_input_.mouse_y)){
                const bool activate=planning_double_clicked(2,(int)i);
                planning_tech_selected_=(int)i;
                if(activate){
                    if(planning_.technologies[i])
                        planning_status_message_="Research already completed. This technology is permanently unlocked.";
                    else if(!planning_technology_available(i))
                        planning_status_message_="Research locked: unlock its prerequisite technologies first.";
                    else unlock_planning_technology(i); // validates funds and deducts once
                }
                break;
            }
        }
    }
    const std::array<const char*,16> sigils{{"SYS","MAT","ENG","GNC","BOT","PWR","LV","AIR",
                                             "NAV","BLD","H2O","HAB","BIO","IND","MOD","MOON"}};
    auto tech_color=[&](std::size_t i){
        if(i==2||i==6||i==8)return Color{1.0f,0.69f,0.38f,1};
        if(i==7||i==11||i==12)return Color{0.43f,0.95f,0.61f,1};
        if(i==14||i==15)return Color{0.83f,0.62f,1.0f,1};
        return cyan;
    };
    for(std::size_t i=0;i<nodes.size();++i){
        const Rect n=nodes[i];
        if(n.x+n.w<graph.x || n.x>graph.x+graph.w ||
           n.y+n.h<graph.y || n.y>graph.y+graph.h)continue;
        const bool researched=planning_.technologies[i]!=0;
        const bool available=planning_technology_available(i);
        const bool selected=planning_tech_selected_==(int)i;
        const Color state=researched?green:available?cyan:Color{0.30f,0.43f,0.56f,1};
        const Color accent=tech_color(i);
        if(selected){
            outline({n.x-4*z,n.y-4*z,n.w+8*z,n.h+8*z},{0.14f,0.70f,1.0f,0.22f},5*z);
            outline({n.x-2*z,n.y-2*z,n.w+4*z,n.h+4*z},{0.25f,0.81f,1.0f,0.53f},1.4f*z);
        }
        clipped_rect(n,researched?Color{0.027f,0.109f,0.101f,0.99f}:
                       available?Color{0.026f,0.077f,0.136f,0.99f}:Color{0.025f,0.046f,0.079f,0.98f});
        outline(n,{state.r,state.g,state.b,researched?0.72f:available?0.79f:0.38f},1.45f*z);
        // Top edge, icon badge and lower status separator.
        clipped_rect({n.x+1*z,n.y+1*z,n.w-2*z,2*z},{accent.r,accent.g,accent.b,researched?0.67f:0.54f});
        clipped_rect({n.x+5*z,n.y+7*z,49*z,51*z},{0.018f,0.032f,0.058f,0.94f});
        outline({n.x+5*z,n.y+7*z,49*z,51*z},{accent.r,accent.g,accent.b,0.47f},0.95f*z);
        label({n.x+8*z,n.y+23*z,42*z,20*z},sigils[i],1.80f*z,accent);
        clipped_line({n.x+61*z,n.y+70*z},{n.x+(node_w-8)*z,n.y+70*z},
                     {state.r,state.g,state.b,0.30f},0.9f*z);
        const std::string heading=kTechs[i].short_name;
        const std::size_t max_chars=20;
        if(heading.size()>max_chars){
            auto split=heading.rfind(' ',max_chars);
            if(split==std::string::npos||split<5)split=max_chars;
            label({n.x+63*z,n.y+14*z,178*z,19*z},heading.substr(0,split),1.65f*z,pale);
            label({n.x+63*z,n.y+34*z,178*z,18*z},heading.substr(split+(heading[split]==' '?1:0)),1.57f*z,pale);
        }else{
            label({n.x+63*z,n.y+22*z,178*z,21*z},heading,1.62f*z,pale);
        }
        label({n.x+63*z,n.y+54*z,178*z,16*z},money(effective_research_cost(planning_,i)),1.32f*z,soft);
        const std::string status=researched?"RESEARCHED":available?"READY TO FUND":"LOCKED";
        label({n.x+63*z,n.y+77*z,123*z,14*z},status,1.19f*z,state);
        if(researched){
            label({n.x+206*z,n.y+73*z,30*z,19*z},"OK",1.63f*z,green);
        }else if(!available){
            // Icon drawn with lines rather than a platform-specific font glyph.
            const float cx=n.x+223*z,cy=n.y+83*z;
            clipped_rect({cx-7*z,cy-2*z,13*z,10*z},Color{0.35f,0.48f,0.65f,0.80f});
            clipped_line({cx-5*z,cy-2*z},{cx-5*z,cy-7*z},soft,1.5f*z);
            clipped_line({cx+4*z,cy-2*z},{cx+4*z,cy-7*z},soft,1.5f*z);
            clipped_line({cx-5*z,cy-7*z},{cx+4*z,cy-7*z},soft,1.5f*z);
        }
    }
    // Viewport border is drawn last so clipped connections cannot bleed over it.
    outline(graph,{0.27f,0.61f,0.86f,0.80f},1.4f);

    // The selected technology is explained in a fixed, legible inspector.
    const std::size_t selected=(std::size_t)std::clamp(planning_tech_selected_,0,15);
    const TechDefinition& t=kTechs[selected];
    const bool researched=planning_.technologies[selected]!=0;
    const bool available=planning_technology_available(selected);
    const auto research_cost=effective_research_cost(planning_,selected);
    const bool affordable=planning_.funds_eur>=research_cost;
    r.draw_rect(detail,{0.024f,0.054f,0.096f,1});
    r.draw_rect({detail.x,detail.y,detail.w,3},
                researched?Color{0.25f,0.79f,0.47f,1}:
                available?Color{0.23f,0.77f,0.98f,1}:Color{0.31f,0.39f,0.50f,1});
    const float right_w=compact?190.0f:250.0f;
    const float left_w=std::max(170.0f,detail.w-right_w-40.0f);
    game_label_fitted(r,{detail.x+14,detail.y+10,left_w,24},t.name,compact?0.70f:0.84f,theme_);
    game_label_fitted(r,{detail.x+14,detail.y+34,left_w,22},t.unlock,compact?0.43f:0.53f,theme_,true);
    std::string prereq="PREREQUISITES: ";
    if(t.prerequisites==0)prereq+="NONE / FOUNDATIONAL RESEARCH";
    else for(std::size_t p=0;p<kTechs.size();++p)if(t.prerequisites&bit(p)){
        if(prereq.size()>16)prereq+="  |  ";
        prereq+=kTechs[p].short_name;
    }
    game_label_fitted(r,{detail.x+14,detail.y+60,left_w,19},prereq,0.44f,theme_,true);
    // Keep the overall goal visible even while the user inspects early nodes.
    game_label_fitted(r,{detail.x+14,detail.y+83,left_w,19},
                      "MISSION READY: STAGE-2 MODULE COMPETENCE -> ROBOTIC LUNAR BASE MISSION",0.41f,theme_,true);
    const float action_x=detail.x+detail.w-right_w+4.0f;
    game_label_fitted(r,{action_x,detail.y+12,right_w-20,22},
                      researched?"RESEARCH COMPLETE":available?"RESEARCH AVAILABLE":"PREREQUISITES LOCKED",
                      compact?0.48f:0.61f,theme_,researched||!available);
    game_label_fitted(r,{action_x,detail.y+37,right_w-20,18},
                      "COST: "+money(research_cost)+(research_cost<t.cost_eur?"  (CUSTOMER EVIDENCE BONUS)":""),0.50f,theme_,true);
    if(!researched){
        if(ui::button(r,ui_input_,{action_x,detail.y+65,right_w-21,35},
                      available&&!affordable?"EARN FUNDS - CONTRACTS":available?"UNLOCK TECHNOLOGY":"NEED PREREQUISITES",
                      theme_,available)){
            if(affordable)unlock_planning_technology(selected);
            else{
                const auto coach=campaign_advice(planning_);
                planning_.view=PlanningView::Contracts;
                if(coach.contract>=0)planning_contract_selected_=coach.contract;
                planning_status_message_="Research needs "+money(research_cost)+". Deliver customer work to raise funds.";
            }
        }
        if(available&&!affordable)
            game_label_fitted(r,{action_x,detail.y+105,right_w-21,13},"WORK WITH CUSTOMERS TO COVER THE GAP",0.26f,theme_,true);
    }else{
        game_label_fitted(r,{action_x,detail.y+73,right_w-20,20},
                          "CAPABILITY UNLOCKED",0.34f,theme_);
    }
}

} // namespace game
