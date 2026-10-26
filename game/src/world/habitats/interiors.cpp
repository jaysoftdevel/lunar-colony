#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats::interiors {
using namespace eng;

namespace {
void console(Builder& b,Vec3 c,float width=1.2f,float facing_z=-1.0f){
    b.box(b.mesh.interior,c,{width,0.68f,0.46f},false);
    b.box(b.mesh.interior,{c.x,c.y+0.48f,c.z+0.18f*facing_z},{width*0.82f,0.42f,0.10f},false);
}
void locker_bank(Builder& b,Vec3 c,int count,float spacing=0.62f){
    for(int i=0;i<count;++i){
        const float x=c.x+((float)i-(float)(count-1)*0.5f)*spacing;
        b.box(b.mesh.interior,{x,c.y,c.z},{0.52f,1.55f,0.50f},false);
        b.box(b.mesh.interior,{x,c.y+0.18f,c.z-0.27f},{0.36f,0.05f,0.04f},false);
    }
}
void ceiling_light(Builder& b,Vec3 a,Vec3 c){
    b.cylinder_between(b.mesh.interior,a,c,0.035f,6,true,false);
}
void stool(Builder& b,Vec3 p){
    b.cylinder_y(b.mesh.interior,{p.x,p.y+0.35f,p.z},0.70f,0.07f,6,false);
    b.cylinder_y(b.mesh.interior,{p.x,p.y+0.72f,p.z},0.12f,0.26f,10,false);
}
void tank(Builder& b,Vec3 p,float h=1.9f,float r=0.32f){
    b.cylinder_y(b.mesh.interior,{p.x,p.y+h*0.5f,p.z},h,r,12,false);
}
}

void add_airlock(Builder& b){
    // Suit prep, storage and pressure-control equipment along the cylindrical walls.
    locker_bank(b,{-1.7f,1.75f,-1.65f},4);
    locker_bank(b,{1.55f,1.75f,-1.65f},3);
    for(float x:{-2.4f,-0.8f,0.8f,2.4f}){
        b.cylinder_y(b.mesh.interior,{x,2.15f,1.60f},1.75f,0.16f,8,false);
        b.box(b.mesh.interior,{x,1.12f,1.60f},{0.55f,0.24f,0.55f},false);
    }
    console(b,{0.2f,1.38f,1.52f},1.45f,1.0f);
    b.box(b.mesh.interior,{0,1.02f,0},{1.9f,0.22f,0.75f},false);
    ceiling_light(b,{-2.7f,4.15f,0},{2.7f,4.15f,0});
    b.cylinder_between(b.mesh.interior,{-3.0f,3.82f,1.3f},{3.0f,3.82f,1.3f},0.06f,6,true,false);
}

void add_oxygen_dome(Builder& b){
    // Electrolyzer stacks, gas buffers, manifolds and operator stations.
    for(float z:{-2.1f,2.1f}){
        for(float x:{-2.2f,-0.75f,0.75f,2.2f})
            b.box(b.mesh.interior,{x,1.75f,z},{0.88f,1.65f,0.58f},false);
    }
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f})
        tank(b,{2.85f*std::cos(a),1.05f,2.85f*std::sin(a)},2.25f,0.36f);
    b.torus_x(b.mesh.interior,{0,3.55f,0},1.45f,0.055f,18,6);
    for(float z:{-1.25f,1.25f}) console(b,{0,1.35f,z},1.35f,z<0?-1.0f:1.0f);
    for(float x:{-2.5f,0.0f,2.5f}) ceiling_light(b,{x,4.25f,-1.6f},{x,4.25f,1.6f});
}

void add_cantine_dome(Builder& b){
    // Dining clusters, galley/service counter and food storage.
    for(float a:{0.0f,2.0944f,4.1888f}){
        const Vec3 c{2.45f*std::cos(a),1.38f,2.45f*std::sin(a)};
        b.cylinder_y(b.mesh.interior,c,0.16f,0.86f,16,false);
        for(float da:{-0.65f,0.0f,0.65f}){
            const float aa=a+da;
            stool(b,{c.x+1.15f*std::cos(aa),0.85f,c.z+1.15f*std::sin(aa)});
        }
    }
    b.box(b.mesh.interior,{-2.7f,1.35f,-2.55f},{3.2f,0.82f,0.68f},false);
    b.box(b.mesh.interior,{-3.55f,2.25f,-2.78f},{1.15f,1.35f,0.38f},false);
    b.box(b.mesh.interior,{-2.25f,2.25f,-2.78f},{1.15f,1.35f,0.38f},false);
    b.box(b.mesh.interior,{-3.2f,1.52f,2.7f},{2.1f,1.4f,0.45f},false);
    console(b,{2.7f,1.35f,-2.5f},1.25f,-1.0f);
    for(float x:{-2.4f,0.0f,2.4f}) ceiling_light(b,{x,4.55f,-1.4f},{x,4.55f,1.4f});
}

void add_bio_dome(Builder& b){
    // Denser hydroponics with irrigation plumbing, nutrient tanks and potting station.
    for(float z:{-2.35f,-0.8f,0.8f,2.35f}){
        for(float x:{-3.9f,-2.3f,-0.7f,0.9f,2.5f,4.1f}){
            b.box(b.mesh.interior,{x,1.52f,z},{1.15f,0.12f,0.55f},false);
            b.box(b.mesh.interior,{x,2.35f,z},{1.15f,0.12f,0.55f},false);
            for(float dx:{-0.35f,0.0f,0.35f}) b.cylinder_y(b.mesh.interior,{x+dx,1.90f,z},0.64f,0.055f,6,false);
        }
        b.cylinder_between(b.mesh.interior,{-4.6f,1.18f,z},{4.6f,1.18f,z},0.055f,6,true,false);
    }
    tank(b,{-4.55f,0.95f,2.35f},2.15f,0.42f);
    tank(b,{-4.55f,0.95f,-2.35f},2.15f,0.42f);
    b.box(b.mesh.interior,{4.25f,1.40f,0},{1.25f,0.75f,2.2f},false);
    console(b,{4.2f,2.15f,0},1.0f,1.0f);
    for(float x:{-3.7f,-1.8f,0.0f,1.8f,3.7f}) ceiling_light(b,{x,3.82f,-2.5f},{x,3.82f,2.5f});
}

void add_crew_quarters(Builder& b){
    // Personal lockers, fold-down desks, communal table and hygiene/service pods.
    locker_bank(b,{-2.9f,1.95f,-1.85f},4,0.58f);
    locker_bank(b,{2.2f,1.95f,-1.85f},4,0.58f);
    for(float x:{-3.4f,-1.15f,1.15f,3.4f}){
        b.box(b.mesh.interior,{x,1.15f,0.65f},{1.15f,0.10f,0.55f},false);
        stool(b,{x,0.75f,1.20f});
    }
    b.box(b.mesh.interior,{0,1.10f,0},{2.1f,0.16f,1.05f},false);
    for(Vec3 p:std::initializer_list<Vec3>{{-1.25f,0.75f,-0.6f},{1.25f,0.75f,-0.6f},{-1.25f,0.75f,0.6f},{1.25f,0.75f,0.6f}}) stool(b,p);
    b.box(b.mesh.interior,{-4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    b.box(b.mesh.interior,{4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    for(float x:{-3.2f,0.0f,3.2f}) ceiling_light(b,{x,4.25f,-0.8f},{x,4.25f,0.8f});
}

void add_processing_facility(Builder& b){
    // Interior refining line: furnaces, separation vessels, ingot molds and belt transfer.
    for(float x:{-4.8f,-2.9f,-1.0f}){
        b.box(b.mesh.interior,{x,2.0f,-1.8f},{1.25f,2.1f,1.15f},false);
        b.cylinder_y(b.mesh.interior,{x,3.35f,-1.8f},0.75f,0.42f,10,false);
    }
    b.box(b.mesh.interior,{-2.7f,1.25f,0.35f},{6.0f,0.22f,0.85f},false);
    for(int i=0;i<7;++i) b.cylinder_z(b.mesh.interior,{-5.1f+0.8f*i,1.42f,0.35f},0.95f,0.11f,8,false);
    for(float x:{0.8f,2.3f,3.8f}){
        b.box(b.mesh.interior,{x,1.28f,1.65f},{1.15f,0.34f,1.65f},false);
        for(int k=0;k<4;++k) b.box(b.mesh.interior,{x-0.33f+0.22f*k,1.54f,1.65f},{0.16f,0.12f,1.25f},false);
    }
    for(float z:{-1.8f,1.8f}) console(b,{4.65f,1.45f,z},1.2f,z<0?-1.0f:1.0f);
    for(float x:{-4.5f,-1.5f,1.5f,4.5f}) ceiling_light(b,{x,5.0f,-2.0f},{x,5.0f,2.0f});
}

void add_factory(Builder& b){
    // Additional assembly conveyor, component shelving, inspection station and tool cabinets.
    b.box(b.mesh.interior,{-2.6f,1.35f,0},{5.3f,0.20f,0.85f},false);
    for(int i=0;i<7;++i) b.cylinder_z(b.mesh.interior,{-4.9f+0.75f*i,1.52f,0},0.92f,0.10f,8,false);
    for(float x:{-4.5f,-2.8f,-1.1f}) for(float y:{1.65f,2.35f,3.05f})
        b.box(b.mesh.interior,{x,y,2.7f},{1.25f,0.18f,0.55f},false);
    b.box(b.mesh.interior,{0.15f,1.65f,-2.65f},{1.8f,1.25f,0.65f},false);
    console(b,{-0.1f,2.6f,-2.65f},1.25f,-1.0f);
    b.cylinder_y(b.mesh.interior,{-4.7f,2.25f,-2.45f},2.3f,0.35f,10,false);
    for(float x:{-4.4f,-2.2f,0.0f}) ceiling_light(b,{x,5.65f,-2.4f},{x,5.65f,2.4f});
}

void add_manufactory(Builder& b){
    // Heavy fabrication bed, additive/CNC cells, material pallets and operator island.
    b.box(b.mesh.interior,{-3.6f,1.55f,0},{5.8f,0.34f,2.6f},false);
    for(float x:{-5.6f,-3.6f,-1.6f}){
        b.box(b.mesh.interior,{x,2.25f,-2.55f},{1.45f,2.0f,1.1f},false);
        b.cylinder_y(b.mesh.interior,{x,3.55f,-2.55f},0.72f,0.35f,10,false);
    }
    for(int row=0;row<2;++row) for(int col=0;col<4;++col)
        b.box(b.mesh.interior,{-6.1f+1.45f*col,1.45f+0.72f*row,2.65f},{1.1f,0.55f,1.0f},false);
    console(b,{-0.1f,1.65f,2.65f},1.45f,1.0f);
    b.cylinder_between(b.mesh.interior,{-3.6f,5.95f,0},{-3.6f,3.35f,0},0.13f,8,true,false);
    b.box(b.mesh.interior,{-3.6f,3.15f,0},{0.75f,0.55f,1.2f},false);
    for(float x:{-5.8f,-3.4f,-1.0f}) ceiling_light(b,{x,6.05f,-3.2f},{x,6.05f,3.2f});
}

void add_sick_bay(Builder& b){
    // Medical storage, scanner arches, operating table, monitors and isolation cubicle.
    b.box(b.mesh.interior,{0,1.35f,0},{2.2f,0.32f,0.85f},false);
    b.torus_x(b.mesh.interior,{0,2.15f,0},0.85f,0.07f,16,6);
    for(float z:{-2.85f,2.85f}) locker_bank(b,{0,1.95f,z},5,0.6f);
    for(float x:{-2.0f,2.0f}){
        b.box(b.mesh.interior,{x,2.0f,0},{0.65f,1.2f,0.42f},false);
        b.box(b.mesh.interior,{x,2.55f,-0.25f},{0.48f,0.42f,0.08f},false);
    }
    b.box(b.mesh.interior,{-3.15f,1.85f,0},{0.9f,2.2f,2.0f},false);
    b.cylinder_y(b.mesh.interior,{-3.15f,3.15f,0},0.45f,0.28f,10,false);
    console(b,{3.15f,1.45f,0},1.15f,1.0f);
    for(float x:{-2.3f,0.0f,2.3f}) ceiling_light(b,{x,4.85f,-1.8f},{x,4.85f,1.8f});
}

void add_laboratory(Builder& b){
    // Dense research benches, glove boxes, sample carousel, servers and optical/instrument tables.
    for(float x:{-3.8f,-1.9f,0.0f,1.9f}){
        b.box(b.mesh.interior,{x,1.45f,0.2f},{1.45f,0.72f,0.72f},false);
        b.box(b.mesh.interior,{x,2.18f,0.2f},{1.15f,0.55f,0.12f},false);
    }
    b.box(b.mesh.interior,{-3.55f,2.0f,1.75f},{1.7f,1.8f,0.65f},false);
    b.box(b.mesh.interior,{2.65f,2.0f,1.75f},{1.7f,1.8f,0.65f},false);
    b.cylinder_y(b.mesh.interior,{0.2f,1.85f,-1.65f},1.55f,0.75f,18,false);
    for(int i=0;i<8;++i){
        const float a=2.0f*PI*(float)i/8.0f;
        b.cylinder_y(b.mesh.interior,{0.2f+0.52f*std::cos(a),2.55f,-1.65f+0.52f*std::sin(a)},0.35f,0.07f,6,false);
    }
    b.box(b.mesh.interior,{3.95f,1.75f,-1.55f},{0.85f,1.55f,0.65f},false);
    b.cylinder_between(b.mesh.interior,{-1.8f,2.0f,-1.6f},{-1.15f,2.75f,-1.1f},0.10f,8,true,false);
    b.cylinder_between(b.mesh.interior,{-1.15f,2.75f,-1.1f},{-0.6f,2.25f,-0.7f},0.08f,8,true,false);
    for(float x:{-3.6f,-1.2f,1.2f,3.6f}) ceiling_light(b,{x,4.85f,-1.7f},{x,4.85f,1.7f});
}

} // namespace game::habitats::interiors
