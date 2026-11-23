#include "game/game_app.hpp"
#include "game/world/structure_display.hpp"
#include "game/world/hull_surface.hpp"
#include "game/world/pressure_hardware.hpp"
#include "game/world/solar_collector.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/lunar_look.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/resource_builder.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/world/ore_stockyard.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/construction_sequence.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <span>
#include <sstream>

namespace game {
using namespace game::runtime;
using namespace layout;
using namespace world_math;
using namespace world_labels;

namespace {
const std::vector<Vec3> empty_mesh_layer;
LunarSurfaceMapping station_mapping(Vec3 position,float rotation,unsigned category){
    const auto x=static_cast<unsigned>(static_cast<int>(std::lround(position.x*10)));
    const auto z=static_cast<unsigned>(static_cast<int>(std::lround(position.z*10)));
    return {position,rotate_y({1,0,0},rotation),{0,1,0},rotate_y({0,0,1},rotation),
        (x*73856093u^z*19349663u^category*83492791u)&0x7fffffffu};
}

bool resource_has_hull(ResourceKind kind){
    return kind==ResourceKind::StorageDome || kind==ResourceKind::StorageGate || kind==ResourceKind::RobotWorkshop;
}

void draw_structure_ground_reservation(SceneRenderer& r,float physical_x,float physical_z,Vec3 position,float rotation_degrees,Color color,float width){
    const float hx=geometry::structure_ground_extent(physical_x)*0.5f;
    const float hz=geometry::structure_ground_extent(physical_z)*0.5f;
    // Lift guides above regolith/deck edges and add corner posts for FPV.
    const float y=.16f;
    std::vector<Vec3> marker{{-hx,y,-hz},{hx,y,-hz},{hx,y,-hz},{hx,y,hz},
        {hx,y,hz},{-hx,y,hz},{-hx,y,hz},{-hx,y,-hz}};
    for(float x:{-hx,hx})for(float z:{-hz,hz}){
        marker.push_back({x,y,z});marker.push_back({x,y+.65f,z});
    }
    const auto world=transformed_vertices(marker,position,rotation_degrees);
    r.draw_lines_3d(world,{.015f,.025f,.035f,.95f},width+3.f);
    color.a=1.f;
    r.draw_lines_3d(world,color,std::max(2.5f,width));
}

void draw_faded_hull(SceneRenderer& r,const std::vector<Vec3>& shell,const std::vector<Vec3>& glass,
                     const std::vector<Vec3>& edges,Vec3 position,float rotation,bool solid,
                     float installed_fraction=1.f){
    const Color tint{.15f,.78f,1.f,.12f};
    const auto visible=construction_sequence::reveal_triangles(shell,installed_fraction);
    if(solid){
        const auto mapping=station_mapping(position,rotation,99);
        r.draw_mapped_lunar_surface(transformed_vertices(visible,position,rotation),LunarMaterial::Hull,tint,mapping);
        r.draw_mapped_lunar_surface(transformed_vertices(construction_sequence::reveal_triangles(glass,installed_fraction),position,rotation),LunarMaterial::Glass,tint,mapping);
    }else mesh_wireframe::draw_layer(r,transformed_vertices(visible,position,rotation),{.15f,.78f,1.f,.60f},1.5f);
    // Cyan always means a physical shell is faded. Unbuilt parts remain drafts.
    const auto guides=transformed_vertices(edges,position,rotation);
    r.draw_lines_3d(guides,{.18f,.82f,1.f,installed_fraction<1.f?.40f:.72f},1.6f);
}

void draw_pad_markings(SceneRenderer& r,const ResourceMesh& mesh,Vec3 position,float rotation){
    const float scale=mesh.footprint_x/20.0f;
    const auto quad=[&](std::vector<Vec3>& out,float x,float z,float width,float depth,float y,float angle){
        const Vec3 center=rotate_y(Vec3{x*scale,y,z*scale},angle);
        const Vec3 x_axis=rotate_y(Vec3{width*scale*0.5f,0,0},angle);
        const Vec3 z_axis=rotate_y(Vec3{0,0,depth*scale*0.5f},angle);
        const Vec3 a=center-x_axis-z_axis,b=center+x_axis-z_axis,c=center+x_axis+z_axis,d=center-x_axis+z_axis;
        out.insert(out.end(),{a,b,c,a,c,d});
    };
    std::vector<Vec3> white,amber;
    for(float side:{-1.0f,1.0f}) quad(white,side*0.70f,0,0.29f,2.2f,0.761f,0);
    quad(white,0,0,1.40f,0.29f,0.761f,0);
    for(float angle:{0.f,90.f,180.f,270.f}){
        quad(amber,0,-7.08f,2.45f,.26f,.756f,angle);
        quad(white,0,-6.42f,.20f,.72f,.756f,angle);
    }
    for(int segment=0;segment<24;++segment)
        quad(white,0,-4.8f,1.02f,.16f,.756f,15.f*segment);
    r.draw_lunar_surface(transformed_vertices(white,position,rotation),LunarMaterial::Marking,{0.86f,0.83f,0.74f,1});
    r.draw_lunar_surface(transformed_vertices(amber,position,rotation),LunarMaterial::Marking,{0.83f,0.59f,0.22f,1});
}

std::vector<Vec3> construction_path_lines(const std::vector<Vec3>& path){
    std::vector<Vec3> lines;
    if(path.size()<2) return lines;
    lines.reserve((path.size()-1)*2);
    for(std::size_t i=0;i+1<path.size();++i){
        lines.push_back(path[i]);
        lines.push_back(path[i+1]);
    }
    return lines;
}

bool entrance_access_hits_corridor(const HabitatEntranceAccess& access,Vec3 structure_position,float rotation_degrees,
                                  const CorridorNetwork& network){
    // Authored exterior stairs/ramps are only surface-access geometry.  If a
    // pressurised corridor occupies that volume, hide the access set instead
    // of rendering stairs through the pressure tube.
    const float clearance=geometry::kCorridorHalfWidth+0.18f;
    auto vertex_hits=[&](Vec3 local){
        const Vec3 world=world_math::rotate_y(local,rotation_degrees)+structure_position;
        for(const auto& corridor:network.connections()){
            if(corridor.path.size()<2) continue;
            for(std::size_t i=0;i+1<corridor.path.size();++i){
                if(world_math::point_segment_distance_xz(world,corridor.path[i],corridor.path[i+1])<clearance) return true;
            }
        }
        return false;
    };
    for(const Vec3& v:access.details) if(vertex_hits(v)) return true;
    for(const Vec3& v:access.edges) if(vertex_hits(v)) return true;
    return false;
}

Color resource_box_color(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return {0.48f,0.52f,0.56f,1.0f};
        case MaterialResource::Copper: return {0.74f,0.42f,0.20f,1.0f};
        case MaterialResource::Silicon: return {0.28f,0.58f,0.76f,1.0f};
        case MaterialResource::Plastic: return {0.72f,0.70f,0.58f,1.0f};
        case MaterialResource::Water: return {0.22f,0.58f,0.86f,1.0f};
        case MaterialResource::Food: return {0.48f,0.72f,0.28f,1.0f};
        case MaterialResource::Medical: return {0.88f,0.42f,0.48f,1.0f};
        case MaterialResource::Aluminum: return {0.72f,0.78f,0.84f,1.0f};
        case MaterialResource::Count: break;
    }
    return {0.6f,0.6f,0.6f,1.0f};
}

struct ResourceBoxBatch {
    struct Faces {MaterialResource material;std::vector<Vec3> vertices;};
    std::vector<Faces> faces;
    std::vector<Vec3> edges;
    void append(std::span<const Vec3> triangles,std::span<const Vec3> lines,MaterialResource material){
        // Limit each batch's bounds to a small group of packages so that the
        // camera and sun passes can reject piles outside their own view.
        constexpr std::size_t batch_limit=16*36;
        if(faces.empty() || faces.back().material!=material || faces.back().vertices.size()>=batch_limit)
            faces.push_back({material,{}});
        auto& batch=faces.back().vertices;batch.insert(batch.end(),triangles.begin(),triangles.end());
        edges.insert(edges.end(),lines.begin(),lines.end());
    }
    void draw(SceneRenderer& r) const {
        // Inventory packages never intersect. Keep face order and consolidate
        // their common outline after the depth-tested faces, instead of two
        // GPU submissions for every individual 10 kg package.
        for(const auto& batch:faces){
            const auto c=resource_box_color(batch.material);
            r.draw_triangles_3d(batch.vertices,{c.r,c.g,c.b,r.lunar_scene_active()?1.0f:0.90f});
        }
        if(!edges.empty())r.draw_lines_3d(edges,{0.10f,0.12f,0.13f,1},1.25f);
    }
};

void draw_resource_box(SceneRenderer& r,Vec3 base,float half_width,float height,MaterialResource resource,bool selected=false,ResourceBoxBatch* batch=nullptr){
    const Vec3 c=base+Vec3{0,height*0.5f+0.025f,0};
    const float x0=c.x-half_width,x1=c.x+half_width,z0=c.z-half_width,z1=c.z+half_width,y0=c.y-height*0.5f,y1=c.y+height*0.5f;
    const std::array<Vec3,36> tris{{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z1},{x0,y0,z0},{x1,y0,z1},{x0,y0,z1},
        {x0,y1,z0},{x1,y1,z1},{x1,y1,z0},{x0,y1,z0},{x0,y1,z1},{x1,y1,z1},
        {x0,y0,z0},{x0,y1,z0},{x1,y1,z0},{x0,y0,z0},{x1,y1,z0},{x1,y0,z0},
        {x1,y0,z1},{x1,y1,z1},{x0,y1,z1},{x1,y0,z1},{x0,y1,z1},{x0,y0,z1},
        {x0,y0,z1},{x0,y1,z1},{x0,y1,z0},{x0,y0,z1},{x0,y1,z0},{x0,y0,z0},
        {x1,y0,z0},{x1,y1,z0},{x1,y1,z1},{x1,y0,z0},{x1,y1,z1},{x1,y0,z1}
    }};
    const std::array<Vec3,24> edges{{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z0},{x1,y0,z1},{x1,y0,z1},{x0,y0,z1},{x0,y0,z1},{x0,y0,z0},
        {x0,y1,z0},{x1,y1,z0},{x1,y1,z0},{x1,y1,z1},{x1,y1,z1},{x0,y1,z1},{x0,y1,z1},{x0,y1,z0},
        {x0,y0,z0},{x0,y1,z0},{x1,y0,z0},{x1,y1,z0},{x1,y0,z1},{x1,y1,z1},{x0,y0,z1},{x0,y1,z1}
    }};
    if(batch){batch->append(tris,edges,resource);return;}
    const Color base_color=resource_box_color(resource);
    r.draw_triangles_3d(std::vector<Vec3>(tris.begin(),tris.end()),{base_color.r,base_color.g,base_color.b,0.90f});
    r.draw_lines_3d(std::vector<Vec3>(edges.begin(),edges.end()),selected?Color{0.34f,1.00f,0.72f,1.0f}:Color{0.10f,0.12f,0.13f,1.0f},selected?2.8f:1.25f);
}


void draw_tracked_solar(SceneRenderer& r,const ResourceMesh& mesh,const PlacedResource& instance,Vec3 tracker_normal,bool selected,Color accent,bool solid){
    const float boost=selected?1.45f:1.0f;
    const auto foundation=transformed_vertices(mesh.foundation,instance.position,instance.rotation_degrees);
    const solar_collector::Pose pose(mesh.solar_head,instance.position,instance.rotation_degrees,tracker_normal);
    const auto body=solar_collector::vertices(mesh.body,mesh.solar_head.body,mesh.solar_head,pose);
    const auto details=solar_collector::vertices(mesh.details,mesh.solar_head.details,mesh.solar_head,pose);
    const auto edges=selected || !solid?
        solar_collector::vertices(mesh.edges,mesh.solar_head.edges,mesh.solar_head,pose):std::vector<Vec3>{};
    if(solid){
        std::vector<Vec3> panel,mast;
        for(std::size_t i=0;i+2<body.size();i+=3){
            const auto authored_center=(mesh.body[i]+mesh.body[i+1]+mesh.body[i+2])*(1.f/3);
            const auto face=normalize(cross(mesh.body[i+1]-mesh.body[i],mesh.body[i+2]-mesh.body[i]));
            const bool front=solar_collector::moving(i,mesh.solar_head.body)&&std::abs(dot(face,pose.authored_normal))>.9f&&dot(authored_center-mesh.solar_head.center,pose.authored_normal)>0;
            auto& layer=front?panel:mast;
            layer.insert(layer.end(),{body[i],body[i+1],body[i+2]});
        }
        r.draw_lunar_surface(foundation,LunarMaterial::Pad,{0.43f,0.44f,0.46f,1});
        r.draw_lunar_surface(mast,LunarMaterial::Paint,{0.80f,0.81f,0.78f,1});
        r.draw_mapped_lunar_surface(panel,LunarMaterial::Solar,{.10f,.20f,.34f,1},pose.mapping());
        r.draw_lunar_surface(details,LunarMaterial::Alloy,{0.69f,0.63f,0.49f,1});
        if(selected) r.draw_lines_3d(edges,accent,2.0f);
        return;
    }
    mesh_wireframe::draw_layer(r,foundation,{0.20f,0.84f,0.36f,1.0f},2.40f*boost);
    mesh_wireframe::draw_layer(r,body,{0.24f,0.96f,0.42f,1.0f},2.65f*boost);
    mesh_wireframe::draw_layer(r,details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
    r.draw_lines_3d(edges,selected?accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.65f:2.90f);
}


const std::vector<Vec3>& portal_layer(const std::vector<Vec3>& triangles,Vec3 position,float rotation,
                                       const std::vector<CorridorDoor>* doors,bool lines=false,bool foundation=false){
    if(triangles.empty() || !doors || doors->empty()) return triangles;
    struct Cache {
        const Vec3* source{};std::size_t size{};bool lines{};
        std::vector<mesh_portals::Aperture> openings;
        std::vector<Vec3> original;
        std::vector<Vec3> triangles;
    };
    static std::vector<Cache> cache;
    std::vector<mesh_portals::Aperture> openings;
    for(const auto& door:*doors){
        Vec3 center=rotate_y(door.wall_position-position,-rotation);
        const Vec3 normal=rotate_y(door.normal,-rotation);float low=-.90f,high=.90f;
        for(auto p:door.wall_profile){const float axial=dot(rotate_y(p-position,-rotation)-center,normal);low=std::min(low,axial-.25f);high=std::max(high,axial+.25f);}
        center=center+normal*((low+high)*.5f);
        openings.push_back({center,normal,(high-low)*.5f,
            geometry::kDoorClearWidth*0.5f,foundation?geometry::kCorridorDeckY-position.y-0.15f:door.position.y-position.y+0.015f,
            door.position.y-position.y+geometry::kDoorClearHeight});
    }
    for(const auto& entry:cache){
        if(entry.source!=triangles.data() || entry.size!=triangles.size() || entry.lines!=lines || entry.openings.size()!=openings.size()) continue;
        // A station reservation can replace geometry in-place without changing
        // its address or count. Never reuse a cut made from the old assembly.
        if(std::memcmp(entry.original.data(),triangles.data(),triangles.size()*sizeof(Vec3))!=0)continue;
        bool same=true;
        for(std::size_t i=0;i<openings.size();++i){
            const auto& a=openings[i];const auto& b=entry.openings[i];
            same=same && length(a.center-b.center)<0.0001f && length(a.normal-b.normal)<0.0001f &&
                std::abs(a.bottom-b.bottom)<0.0001f && std::abs(a.top-b.top)<0.0001f && std::abs(a.half_depth-b.half_depth)<.0001f;
        }
        if(same) return entry.triangles;
    }
    if(cache.size()>256) cache.clear();
    Cache entry{triangles.data(),triangles.size(),lines,openings,triangles,triangles};
    for(const auto& opening:openings) entry.triangles=lines?mesh_portals::subtract_lines(entry.triangles,opening):mesh_portals::subtract(entry.triangles,opening);
    cache.push_back(std::move(entry));
    return cache.back().triangles;
}

void draw_extension_mesh(SceneRenderer& r,const ResourceMesh& mesh,float progress,bool solid,bool preview){
    if(solid && !preview && progress>=0.9999f){
        r.draw_lunar_surface(mesh.foundation,LunarMaterial::Pad,{0.47f,0.50f,0.53f,1});
        r.draw_lunar_surface(mesh.body,LunarMaterial::Paint,{0.69f,0.73f,0.74f,1});
        r.draw_lunar_surface(mesh.details,LunarMaterial::Alloy,{0.52f,0.55f,0.57f,1});
        return;
    }
    const Color blueprint{0.10f,0.42f,0.96f,0.52f},installed{1,0.68f,0.12f,0.99f},complete{0.24f,0.96f,0.42f,1};
    for(const auto* layer:{&mesh.foundation,&mesh.body,&mesh.details}){
        if(preview || progress<0.9999f) mesh_wireframe::draw_layer(r,*layer,blueprint,1.50f);
        if(!preview && progress>0) mesh_wireframe::draw_layer(r,construction_sequence::reveal_triangles(*layer,progress),progress>=0.9999f?complete:installed,2.35f);
    }
    if(!mesh.edges.empty()) r.draw_lines_3d(mesh.edges,preview || progress<0.9999f?blueprint:complete,2.1f);
}
void draw_pressure_box(SceneRenderer& r,Vec3 center,Vec3 side,Vec3 normal,Vec3 size,Color color,bool solid){
    resources::Builder b;b.oriented_box(b.mesh.body,center,side,{0,1,0},normal,size);
    if(solid) r.draw_lunar_surface(b.mesh.body,LunarMaterial::Pad,{0.59f,0.62f,0.63f,1});
    else{
        mesh_wireframe::draw_layer(r,b.mesh.body,color,2.35f);
        r.draw_lines_3d(b.mesh.edges,color,2.1f);
    }
}
void draw_generated_airlock_access(SceneRenderer& r,const CorridorDoor& door,float deck_y,Color color,bool solid){
    if(door.entrance_index!=-4) return;
    Vec3 n=door.normal;n.y=0;n=normalize(n);if(length(n)<0.001f)return;
    const Vec3 side{-n.z,0,n.x};
    const int steps=std::max(1,(int)std::ceil(deck_y/0.16f));
    const float tread=0.32f,rise=deck_y/(float)steps;
    resources::Builder b;
    for(int i=0;i<steps;++i){
        const float top=deck_y-rise*(float)i;
        const Vec3 center=door.wall_position+n*(tread*((float)i+0.5f))+Vec3{0,top-0.05f-door.wall_position.y,0};
        b.oriented_box(b.mesh.body,center,side,{0,1,0},n,{1.76f,0.10f,tread});
    }
    if(solid) r.draw_lunar_surface(b.mesh.body,LunarMaterial::Pad,{0.54f,0.57f,0.59f,1});
    else{
        mesh_wireframe::draw_layer(r,b.mesh.body,color,2.35f);
        r.draw_lines_3d(b.mesh.edges,color,2.1f);
    }
}
const ResourceMesh& cargo_port_mesh(const ResourceMesh& source,const PlacedResource& owner){
    struct Entry {const ResourceMesh* source;std::array<int,3> faces;std::array<float,3> angles;std::uint64_t active;ResourceMesh mesh;};
    static std::vector<Entry> cache;
    const auto faces=storage_loading::faces(owner,true);
    const std::array<float,3> angles{{owner.storage_gate_angle_degrees,owner.storage_extra_angles[0],owner.storage_extra_angles[1]}};
    std::uint64_t active=0;for(const auto& station:source.stations)active=active*131u+std::uint64_t(station.active)+1u;
    for(const auto& entry:cache) if(entry.active==active && entry.source==&source && entry.faces==faces && entry.angles==angles) return entry.mesh;
    if(cache.size()>128) cache.clear();
    Entry entry{&source,faces,angles,active,source};
    for(int face:faces){
        if(face<0) continue;
        const auto normal=storage_loading::local_normal(owner,face);
        const auto center=normal*(storage_dome_diameter_m(owner.storage_size)*(5.35f/13.0f));
        const mesh_portals::Aperture opening{center,normal,0.95f,storage_loading::kHalfWidth+0.02f,0.05f,
            source.walkable_floor_y+geometry::kDoorFrameHeight+0.20f};
        // Replace the candidate crew-port furniture with the cargo module.
        // The pressure hull and the transfer buffer's rear seal remain intact.
        entry.mesh.details=mesh_portals::subtract(entry.mesh.details,opening);
        entry.mesh.edges=mesh_portals::subtract_lines(entry.mesh.edges,opening);
    }
    cache.push_back(std::move(entry));return cache.back().mesh;
}
void draw_sliding_door(SceneRenderer& r,const CorridorDoor& door,float open_fraction,bool solid,float progress=1){
    Vec3 n=door.normal;n.y=0;n=normalize(n);if(length(n)<0.001f)n={1,0,0};
    const Vec3 side{-n.z,0,n.x};
    const float w=geometry::kDoorClearWidth,h=geometry::kDoorClearHeight;
    const Vec3 base{door.wall_position.x,door.position.y,door.wall_position.z};
    const float slide=pressure_door::kSlide*std::clamp(open_fraction,0.0f,1.0f);
    resources::Builder b;
    auto box=[&](Vec3 center,Vec3 size){b.oriented_box(b.mesh.body,center,side,{0,1,0},n,size);};
    for(int sign:{-1,1}){
        box(base+side*((float)sign*(w*0.5f+0.08f))+Vec3{0,h*0.5f,0},{0.16f,h+0.16f,0.24f});
        const auto center=base+side*((float)sign*(w*.25f+slide))+Vec3{0,h*.5f,0};
        if(solid&&progress>=.9999f)pressure_hardware::leaf(r,center,side,n,2*pressure_door::kPanelHalfWidth,h+.10f,pressure_door::kThickness);
        else box(center,{2*pressure_door::kPanelHalfWidth,h+.10f,pressure_door::kThickness});
    }
    box(base+Vec3{0,h+0.08f,0},{w+0.50f,0.16f,0.28f});
    box(base+Vec3{0,0.005f,0},{w+0.25f,0.03f,0.24f});
    draw_extension_mesh(r,b.mesh,progress,solid,false);
    const Color light=pressure_door::sealed(open_fraction)?Color{1,0.57f,0.12f,1}:Color{0.24f,1,0.56f,1};
    r.draw_lines_3d({base+side*0.25f+Vec3{0,h+0.085f,0}+n*0.15f,base-side*0.25f+Vec3{0,h+0.085f,0}+n*0.15f},light,3.5f);
}
void draw_loading_gate(SceneRenderer& r,const PlacedResource& item,float opening,bool solid,bool show_hull,bool preview){
    const Vec3 n=storage_loading::normal(item),side{-n.z,0,n.x};
    const Vec3 wall=storage_loading::wall(item),mouth=storage_loading::mouth(item);
    const float depth=length(mouth-wall);
    const float h=storage_loading::kHeight,w=storage_loading::kHalfWidth;
    resources::Builder chamber;
    auto box=[&](Vec3 center,Vec3 size){chamber.oriented_box(chamber.mesh.body,center,side,{0,1,0},n,size);};
    const Vec3 mid=(wall+mouth)*0.5f;
    box(mid+Vec3{0,0.14f,0},{2*w+0.18f,0.18f,depth+0.20f});
    resources::Builder faded;
    auto wall_box=[&](Vec3 center,Vec3 size){
        if(show_hull || preview)box(center,size);else faded.oriented_box(faded.mesh.body,center,side,{0,1,0},n,size);
    };
    {
        wall_box(mid+Vec3{0,h+0.18f,0},{2*w+0.18f,0.16f,depth+0.20f});
        for(int sign:{-1,1}) wall_box(mid+side*((float)sign*(w+0.05f))+Vec3{0,h*0.5f+0.15f,0},{0.12f,h,depth+0.20f});
        wall_box(wall+Vec3{0,h*0.5f+0.15f,0},{2*w,h,0.12f});
    }
    const float progress=item.construction.progress();
    if(solid&&!preview&&show_hull&&progress>=.9999f){
        const auto& panels=hull_surface::split(chamber.mesh.body,mid.y+1.15f);
        r.draw_lunar_surface(panels.lower,LunarMaterial::Hull,{.74f,.77f,.75f,1});
        r.draw_lunar_surface(panels.upper,LunarMaterial::SolarShield,{.38f,.59f,.65f,.28f});
    }else draw_extension_mesh(r,chamber.mesh,progress,solid,preview);
    if(!show_hull&&!preview)draw_faded_hull(r,faded.mesh.body,empty_mesh_layer,faded.mesh.edges,{},0,solid,progress);
    resources::Builder leaves;
    for(int sign:{-1,1}){
        const Vec3 c=mouth+side*((float)sign*(w*0.5f+opening*(w+0.06f)))+Vec3{0,h*0.5f+0.15f,0}+n*0.08f;
        if(solid&&!preview&&progress>=.9999f)pressure_hardware::leaf(r,c,side,n,w+.04f,h,.10f);
        else leaves.oriented_box(leaves.mesh.body,c,side,{0,1,0},n,{w+.04f,h,.10f});
    }
    draw_extension_mesh(r,leaves.mesh,progress,solid,preview);
    const Vec3 service=storage_loading::service(item)+Vec3{0,0.04f,0};
    r.draw_lines_3d({service-side*1.25f-n*0.7f,service-side*1.25f+n*0.8f,
        service+side*1.25f-n*0.7f,service+side*1.25f+n*0.8f,
        service-side*1.25f+n*0.8f,service+side*1.25f+n*0.8f},{1,0.70f,0.16f,1},2.2f);
    const Vec3 tip=service-n*0.55f;
    r.draw_lines_3d({service+n*0.4f,tip,tip,tip+n*0.3f+side*0.28f,tip,tip+n*0.3f-side*0.28f},{1,0.70f,0.16f,1},2.5f);
}

}

void GameApp::render_build_footprints(SceneRenderer& r) const {
    for(const auto& item:placed_habitats_){
        const auto& mesh=habitat_models_.mesh(item);
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,item.position,item.rotation_degrees,
            item.construction.complete()?Color{1.f,.73f,.18f,1}:Color{.12f,.8f,1.f,1},2.6f);
    }
    for(const auto& item:placed_resources_){
        const auto& mesh=resource_models_.mesh(item);
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,item.position,item.rotation_degrees,
            item.construction.complete()?Color{1.f,.73f,.18f,1}:Color{.12f,.8f,1.f,1},2.6f);
    }
}

void GameApp::render_habitat_mesh(SceneRenderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,float construction_step,const std::vector<CorridorDoor>* fpv_doors,bool greenhouse_dark,HabitatKind kind,bool powered) const {
    if(preview){
        const auto shell=transformed_vertices(portal_layer(mesh.shell,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
        const auto interior=transformed_vertices(mesh.interior,position,rotation_degrees);
        const auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto glazing=transformed_vertices(portal_layer(mesh.glazing,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        const Color line=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        const Color faint=valid?Color{0.18f,0.78f,0.56f,0.78f}:Color{0.92f,0.25f,0.20f,0.78f};
        mesh_wireframe::draw_layer(r,foundation,faint,1.25f);
        mesh_wireframe::draw_layer(r,shell,line,1.35f);
        mesh_wireframe::draw_layer(r,interior,faint,1.05f);
        mesh_wireframe::draw_layer(r,details,faint,1.15f);
        mesh_wireframe::draw_layer(r,glazing,{line.r,line.g,line.b,0.62f},1.0f);
        r.draw_lines_3d(edges,line,1.8f);
        for(const auto& access:mesh.entrance_access){
            const auto access_details=transformed_vertices(access.details,position,rotation_degrees);
            const auto access_edges=transformed_vertices(access.edges,position,rotation_degrees);
            mesh_wireframe::draw_layer(r,access_details,faint,1.15f);
            r.draw_lines_3d(access_edges,line,1.6f);
        }
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,
                                          valid?Color{0.20f,0.80f,1.00f,0.62f}:Color{1.00f,0.30f,0.22f,0.70f},1.65f);
        return;
    }

    const bool show_hull=habitat_view_mode_==HabitatViewMode::FullHull;
    if(!show_hull){
        const float installed=construction_step<kConstructionSteps?construction_sequence::habitat_visibility(construction_step).shell:1.f;
        draw_faded_hull(r,portal_layer(mesh.shell,position,rotation_degrees,fpv_doors),
            portal_layer(mesh.glazing,position,rotation_degrees,fpv_doors),
            portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees,!mesh_layout_active(),installed);
    }
    const auto& shell_layer=show_hull?mesh.shell:empty_mesh_layer;
    const auto& glass_layer=show_hull?mesh.glazing:empty_mesh_layer;
    const auto& edge_layer=show_hull?mesh.edges:empty_mesh_layer;
    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::habitat_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto shell=transformed_vertices(construction_sequence::reveal_triangles(shell_layer,visibility.shell),position,rotation_degrees);
        const auto interior=transformed_vertices(construction_sequence::reveal_triangles(mesh.interior,visibility.interior),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto glazing=transformed_vertices(construction_sequence::reveal_triangles(glass_layer,visibility.glazing),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(edge_layer,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.28f:1.0f;
        // The complete target remains a blue engineering mesh. Every part
        // that has actually been installed replaces that draft with filled,
        // bold yellow construction geometry. The player can therefore read the
        // exact target shape and the completed fraction at the same time.
        const Color blueprint{0.08f,0.36f,0.96f,0.46f};
        const Color installed{1.00f,0.68f,0.12f,0.99f};
        const Color installed_dim{0.96f,0.56f,0.08f,0.96f};
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.foundation,position,rotation_degrees),blueprint,1.35f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(shell_layer,position,rotation_degrees),blueprint,1.55f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.interior,position,rotation_degrees),blueprint,1.25f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.details,position,rotation_degrees),blueprint,1.30f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(glass_layer,position,rotation_degrees),{0.10f,0.48f,1.00f,0.40f},1.20f*boost);
        const auto ghost_edges=transformed_vertices(portal_layer(edge_layer,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        if(!ghost_edges.empty()) r.draw_lines_3d(ghost_edges,{0.12f,0.50f,1.00f,0.62f},1.65f*boost);

        mesh_wireframe::draw_layer(r,foundation,installed_dim,1.95f*boost);
        mesh_wireframe::draw_layer(r,shell,installed,2.20f*boost);
        mesh_wireframe::draw_layer(r,interior,installed_dim,1.85f*boost);
        mesh_wireframe::draw_layer(r,details,installed,1.95f*boost);
        mesh_wireframe::draw_layer(r,glazing,{1.00f,0.76f,0.20f,0.94f},1.80f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,installed,selected?3.45f:2.45f);
        if(construction_step==0 || (selected && !show_hull)){
            draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,
                                              blueprint,selected?3.2f:2.4f);
        }
        return;
    }

    const bool solid=!mesh_layout_active();
    const auto shell=transformed_vertices(portal_layer(shell_layer,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
    const auto interior=transformed_vertices(portal_layer(mesh.interior,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto glazing=transformed_vertices(portal_layer(glass_layer,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto edges=selected || !solid?
        transformed_vertices(portal_layer(edge_layer,position,rotation_degrees,fpv_doors,true),position,rotation_degrees):std::vector<Vec3>{};
    if(selected && !show_hull)
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,theme_.accent,2.0f);
    if(solid){
        Color cladding{0.84f,0.83f,0.78f,1};
        Color fittings{0.64f,0.52f,0.32f,1};
        switch(kind){
            case HabitatKind::AirLock:
                cladding={0.76f,0.83f,0.85f,cladding.a};
                fittings={0.86f,0.57f,0.27f,1};
                break;
            case HabitatKind::OxygenGeneratorDome: fittings={0.38f,0.66f,0.73f,1};break;
            case HabitatKind::CantineDome: fittings={0.75f,0.62f,0.39f,1};break;
            case HabitatKind::BioDome: fittings={0.37f,0.63f,0.52f,1};break;
            case HabitatKind::CrewQuarters: fittings={0.40f,0.58f,0.74f,1};break;
            case HabitatKind::ProcessingFacility:
            case HabitatKind::Factory:
            case HabitatKind::Manufactory:
                cladding={0.55f,0.61f,0.63f,cladding.a};
                fittings={0.80f,0.56f,0.27f,1};
                break;
            case HabitatKind::SickBay: fittings={0.77f,0.33f,0.34f,1};break;
            case HabitatKind::Laboratory: fittings={0.48f,0.49f,0.68f,1};break;
            default: break;
        }
        const Color glass=greenhouse_dark?Color{0.14f,0.25f,0.27f,0.68f}:Color{0.27f,0.51f,0.63f,0.58f};
        const auto mapping=station_mapping(position,rotation_degrees,unsigned(kind)+1);
        r.draw_mapped_lunar_surface(foundation,LunarMaterial::Pad,{.48f,.49f,.50f,1},mapping);
        r.draw_mapped_lunar_surface(interior,LunarMaterial::Interior,{.73f,.76f,.77f,1},mapping);
        r.draw_mapped_lunar_surface(details,LunarMaterial::Alloy,fittings,mapping);
        const auto& panels=hull_surface::split(shell_layer,mesh.walkable_floor_y+1.0f);
        r.draw_mapped_lunar_surface(transformed_vertices(portal_layer(panels.lower,position,rotation_degrees,fpv_doors),position,rotation_degrees),LunarMaterial::Hull,cladding,mapping);
        r.draw_mapped_lunar_surface(transformed_vertices(portal_layer(panels.upper,position,rotation_degrees,fpv_doors),position,rotation_degrees),LunarMaterial::SolarShield,{.38f,.59f,.65f,.28f},mapping);
        r.draw_mapped_lunar_surface(glazing,LunarMaterial::SolarShield,{glass.r,glass.g,glass.b,.28f},mapping);
        const auto layer=[&](const std::vector<Vec3>& points,LunarMaterial material,Color tint,bool portal=true){
            const auto local=portal?portal_layer(points,position,rotation_degrees,fpv_doors):points;
            r.draw_mapped_lunar_surface(transformed_vertices(local,position,rotation_degrees),material,tint,mapping);
        };
        if(show_hull && first_person_mode_==FirstPersonMode::Active)layer(hull_surface::split(mesh.lining,mesh.walkable_floor_y+1.0f).lower,LunarMaterial::Interior,{.79f,.81f,.80f,1});
        layer(mesh.deck,LunarMaterial::Deck,{.27f,.31f,.34f,1});
        layer(mesh.service_metal,LunarMaterial::Alloy,{.43f,.48f,.51f,1});
        layer(mesh.fabric,LunarMaterial::Textile,{.25f,.39f,.46f,1},false);
        layer(mesh.displays,powered?LunarMaterial::Display:LunarMaterial::Glass,powered?Color{.20f,.67f,.80f,1}:Color{.025f,.04f,.05f,1});
        layer(mesh.lamps,powered?LunarMaterial::Emissive:LunarMaterial::Alloy,powered?Color{.77f,.88f,.96f,1}:Color{.09f,.10f,.11f,1});
        layer(mesh.markings,LunarMaterial::Marking,{.80f,.66f,.30f,1});
        if(selected) r.draw_lines_3d(edges,{0.98f,0.85f,0.46f,0.88f},2.0f);
        return;
    }
    const Color shell_color=greenhouse_dark?Color{0.15f,0.46f,0.24f,1.0f}:Color{0.24f,0.96f,0.42f,1.0f};
    const Color foundation_color=greenhouse_dark?Color{0.13f,0.38f,0.20f,1.0f}:Color{0.20f,0.84f,0.36f,1.0f};
    const Color interior_color=greenhouse_dark?Color{0.10f,0.27f,0.16f,0.78f}:Color{0.34f,1.00f,0.50f,0.98f};
    const Color detail_color=greenhouse_dark?Color{0.10f,0.23f,0.15f,0.80f}:Color{0.22f,0.92f,0.40f,0.99f};
    const Color glass_color=greenhouse_dark?Color{0.08f,0.18f,0.15f,0.46f}:Color{0.54f,1.00f,0.48f,0.90f};
    const Color edge_color=selected?Color{1.0f,0.94f,0.55f,1.0f}:Color{0.18f,1.00f,0.38f,1.0f};
    const float selected_boost=selected?1.45f:1.0f;

    mesh_wireframe::draw_layer(r,foundation,foundation_color,2.40f*selected_boost);
    mesh_wireframe::draw_layer(r,interior,interior_color,2.20f*selected_boost);
    mesh_wireframe::draw_layer(r,details,detail_color,2.30f*selected_boost);
    mesh_wireframe::draw_layer(r,glazing,glass_color,2.10f*selected_boost);
    mesh_wireframe::draw_layer(r,shell,shell_color,2.65f*selected_boost);
    r.draw_lines_3d(edges,edge_color,selected?3.65f:2.90f);
}

void GameApp::render_resource_mesh(SceneRenderer& r,const ResourceMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,float construction_step,const std::vector<CorridorDoor>* fpv_doors,ResourceKind kind) const {
    if(preview){
        const auto body=transformed_vertices(portal_layer(mesh.body,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
        const auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        const Color line=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        const Color faint=valid?Color{0.18f,0.78f,0.56f,0.78f}:Color{0.92f,0.25f,0.20f,0.78f};
        mesh_wireframe::draw_layer(r,foundation,faint,1.20f);
        mesh_wireframe::draw_layer(r,body,line,1.35f);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.glazing,position,rotation_degrees),faint,1.1f);
        mesh_wireframe::draw_layer(r,details,faint,1.10f);
        r.draw_lines_3d(edges,line,1.8f);
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,
                                          valid?Color{0.20f,0.80f,1.00f,0.62f}:Color{1.00f,0.30f,0.22f,0.70f},1.65f);
        return;
    }

    const bool show_hull=habitat_view_mode_==HabitatViewMode::FullHull || !resource_has_hull(kind);
    if(!show_hull){
        const float installed=construction_step<kConstructionSteps?construction_sequence::resource_visibility(construction_step).body:1.f;
        draw_faded_hull(r,portal_layer(mesh.body,position,rotation_degrees,fpv_doors),
            portal_layer(mesh.glazing,position,rotation_degrees,fpv_doors),
            portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees,!mesh_layout_active(),installed);
    }
    const auto& glass_layer=show_hull?mesh.glazing:empty_mesh_layer;
    const auto& body_layer=show_hull?mesh.body:empty_mesh_layer;
    const auto& edge_layer=show_hull?mesh.edges:empty_mesh_layer;
    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::resource_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto body=transformed_vertices(construction_sequence::reveal_triangles(body_layer,visibility.body),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(edge_layer,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.28f:1.0f;
        const Color blueprint{0.08f,0.36f,0.96f,0.46f};
        const Color installed{1.00f,0.68f,0.12f,0.99f};
        const Color installed_dim{0.96f,0.56f,0.08f,0.96f};
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.foundation,position,rotation_degrees),blueprint,1.35f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(body_layer,position,rotation_degrees),blueprint,1.55f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.details,position,rotation_degrees),blueprint,1.30f*boost);
        const auto ghost_edges=transformed_vertices(portal_layer(edge_layer,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        if(!ghost_edges.empty()) r.draw_lines_3d(ghost_edges,{0.12f,0.50f,1.00f,0.62f},1.65f*boost);

        mesh_wireframe::draw_layer(r,foundation,installed_dim,1.95f*boost);
        mesh_wireframe::draw_layer(r,body,installed,2.20f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(glass_layer,position,rotation_degrees),blueprint,1.1f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(construction_sequence::reveal_triangles(glass_layer,visibility.body),position,rotation_degrees),installed,1.7f*boost);
        mesh_wireframe::draw_layer(r,details,installed,1.95f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,installed,selected?3.45f:2.45f);
        if(construction_step==0 || (selected && !show_hull)){
            draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,
                                              blueprint,selected?3.2f:2.4f);
        }
        return;
    }

    const bool solid=!mesh_layout_active();
    const auto body=transformed_vertices(portal_layer(body_layer,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
    const auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto edges=selected || !solid?
        transformed_vertices(portal_layer(edge_layer,position,rotation_degrees,fpv_doors,true),position,rotation_degrees):std::vector<Vec3>{};
    if(selected && !show_hull)
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,rotation_degrees,theme_.accent,2.0f);
    if(solid){
        const bool pad=kind==ResourceKind::LandingPad;
        const Color paint=pad?Color{0.31f,0.34f,0.38f,1}:Color{0.80f,0.82f,0.78f,1};
        Color trim{0.66f,0.51f,0.28f,1};
        if(kind==ResourceKind::WaterStorage || kind==ResourceKind::WaterDriller) trim={0.35f,0.58f,0.71f,1};
        if(kind==ResourceKind::HydrogenTank || kind==ResourceKind::OxygenTank) trim={0.51f,0.65f,0.61f,1};
        if(pad) trim={0.80f,0.74f,0.57f,1};
        if(kind==ResourceKind::RobotWorkshop)trim={.40f,.45f,.49f,1};
        const auto mapping=station_mapping(position,rotation_degrees,unsigned(kind)+40);
        r.draw_mapped_lunar_surface(foundation,LunarMaterial::Pad,{.46f,.47f,.49f,1},mapping);
        const bool enclosure=kind!=ResourceKind::SolarPanel&&kind!=ResourceKind::Antenna&&kind!=ResourceKind::LandingPad;
        if(enclosure){
            const auto& panels=hull_surface::split(body_layer,mesh.walkable_floor_y+1.0f);
            r.draw_mapped_lunar_surface(transformed_vertices(portal_layer(panels.lower,position,rotation_degrees,fpv_doors),position,rotation_degrees),LunarMaterial::Hull,paint,mapping);
            r.draw_mapped_lunar_surface(transformed_vertices(portal_layer(panels.upper,position,rotation_degrees,fpv_doors),position,rotation_degrees),LunarMaterial::SolarShield,{.38f,.59f,.65f,.28f},mapping);
        }else r.draw_mapped_lunar_surface(body,pad?LunarMaterial::Deck:LunarMaterial::Hull,paint,mapping);
        r.draw_mapped_lunar_surface(details,pad?LunarMaterial::Hull:LunarMaterial::Alloy,trim,mapping);
        r.draw_mapped_lunar_surface(transformed_vertices(glass_layer,position,rotation_degrees),LunarMaterial::SolarShield,{.38f,.59f,.65f,.28f},mapping);
        if(pad) draw_pad_markings(r,mesh,position,rotation_degrees);
        if(selected) r.draw_lines_3d(edges,{0.98f,0.85f,0.46f,0.88f},2.0f);
        return;
    }
    const float boost=selected?1.45f:1.0f;
    mesh_wireframe::draw_layer(r,foundation,{0.20f,0.84f,0.36f,1.0f},2.40f*boost);
    mesh_wireframe::draw_layer(r,body,{0.24f,0.96f,0.42f,1.0f},2.65f*boost);
    mesh_wireframe::draw_layer(r,transformed_vertices(glass_layer,position,rotation_degrees),{.30f,.85f,.90f,.65f},1.5f*boost);
    mesh_wireframe::draw_layer(r,details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
    r.draw_lines_3d(edges,selected?Color{1.0f,0.94f,0.55f,1.0f}:Color{0.18f,1.00f,0.38f,1.0f},selected?3.65f:2.90f);
}


void GameApp::render_ground_resources(SceneRenderer& r){
    ResourceBoxBatch batch;
    for(std::size_t pile_index=0;pile_index<ground_resource_piles_.size();++pile_index){
        const auto& pile=ground_resource_piles_[pile_index];
        const bool selected=selection_kind_==SelectionKind::GroundResource &&
            (selected_ground_resource_==(int)pile_index ||
             std::find(selected_ground_resources_.begin(),selected_ground_resources_.end(),(int)pile_index)!=selected_ground_resources_.end());
        Vec3 base=pile.position;
        const auto capacity=pile.kind==GroundPileKind::Loose?pile.amount:resource_package_capacity_kg(pile.resource);
        for(auto remaining=pile.amount;remaining>0;remaining-=capacity){
            const auto amount=std::min(remaining,capacity);
            const float half=resource_package_half_width(pile.resource,amount);
            const float height=resource_package_height(pile.resource,amount);
            draw_resource_box(r,base,half,height,pile.resource,selected,selected?nullptr:&batch);
            base.y+=height+.02f;
        }
    }
    batch.draw(r);
}

void GameApp::render_storage_loading_gate(SceneRenderer& r,const PlacedResource& item,float opening,bool preview) const {
    draw_loading_gate(r,item,opening,!mesh_layout_active(),habitat_view_mode_==HabitatViewMode::FullHull,preview);
}

void GameApp::render_storage_gate_shutters(SceneRenderer& r,const PlacedResource& owner,int face,float opening,float progress,bool preview) const {
    const Vec3 n=storage_loading::normal(owner,face),side{-n.z,0,n.x};
    const Vec3 mouth=storage_loading::mouth(owner,face);const float w=storage_loading::kHalfWidth,h=storage_loading::kHeight;
    resources::Builder leaves;
    for(float sign:{-1.0f,1.0f}){
        const Vec3 center=mouth+side*(sign*(w*0.5f+opening*(w+0.06f)))+Vec3{0,h*0.5f+0.15f,0}+n*0.08f;
        if(!mesh_layout_active()&&!preview&&progress>=.9999f)pressure_hardware::leaf(r,center,side,n,w+.04f,h,.10f);
        else leaves.oriented_box(leaves.mesh.body,center,side,{0,1,0},n,{w+.04f,h,.10f});
    }
    draw_extension_mesh(r,leaves.mesh,progress,!mesh_layout_active(),preview);
}

void GameApp::render_workshop_gate(SceneRenderer& r,const PlacedResource& item,bool preview,bool valid) const {
    const Vec3 side=rotate_y({1,0,0},item.rotation_degrees),n=rotate_y({0,0,-1},item.rotation_degrees);
    const Vec3 base=workshop::world(item,{0,workshop::kDeck,-workshop::kHalfZ});
    const float open=item.construction.complete()?item.workshop_gate_open:0;
    const Color color=valid?(preview?Color{0.10f,0.42f,0.96f,0.65f}:Color{0.24f,0.96f,0.42f,1}):Color{1,0.2f,0.1f,1};
    for(float sign:{-1.0f,1.0f}){
        const auto center=base+side*(sign*(workshop::kGateHalf*0.5f+open*(workshop::kGateHalf+0.12f)))+Vec3{0,1.45f,0};
        if(preview){
            resources::Builder b;b.oriented_box(b.mesh.body,center,side,{0,1,0},n,{workshop::kGateHalf+0.06f,2.90f,0.18f});
            mesh_wireframe::draw_layer(r,b.mesh.body,color,1.8f);
        }else if(mesh_layout_active())draw_pressure_box(r,center,side,n,{workshop::kGateHalf+0.06f,2.90f,0.18f},color,false);
        else {
            resources::Builder rails,marks;
            for(int row=0;row<9;++row)
                rails.oriented_box(rails.mesh.body,center+Vec3{0,-1.32f+row*.33f,0}+n*.105f,side,{0,1,0},n,{workshop::kGateHalf-.08f,.045f,.035f},false);
            for(float edge:{-1.f,1.f})rails.oriented_box(rails.mesh.body,center+side*(edge*(workshop::kGateHalf*.5f-.065f))+n*.105f,side,{0,1,0},n,{.055f,2.85f,.035f},false);
            for(int stripe=0;stripe<6;++stripe)marks.oriented_box(marks.mesh.body,center+side*(-.94f+stripe*.37f)+Vec3{0,-1.27f,0}+n*.13f,side,{0,1,0},n,{.19f,.16f,.015f},false);
            LunarSurfaceMapping mapping;mapping.origin=center;mapping.x=side;mapping.y={0,1,0};mapping.z=n;mapping.seed=sign<0?3101:3102;
            pressure_hardware::leaf(r,center,side,n,workshop::kGateHalf+.06f,2.90f,.18f);
            r.draw_mapped_lunar_surface(rails.mesh.body,LunarMaterial::Alloy,{.20f,.25f,.28f,1},mapping);
            r.draw_mapped_lunar_surface(marks.mesh.body,LunarMaterial::Marking,{.95f,.65f,.12f,1},mapping);
        }
    }
    for(int slot=0;slot<workshop::kBays;++slot){
        const Vec3 p=workshop::bay(item,slot);
        bool docked=false;
        for(const auto& robot:builder_robots_) if(robot.workshop_parked && distance_xz(robot.position,p)<0.50f) docked=true;
        const Color lamp=docked?Color{0.25f,1,0.40f,1}:Color{0.22f,0.64f,1,1};
        const Vec3 bay=workshop::bay_local(slot);
        const Vec3 light=workshop::world(item,{bay.x<0?-4.70f:4.70f,1.90f,bay.z});
        r.draw_lines_3d({light+Vec3{0,-0.24f,0},light+Vec3{0,0.24f,0}},lamp,4);
    }
    const Color status=open>=0.999f?Color{0.25f,1,0.4f,1}:Color{1,0.64f,0.12f,1};
    r.draw_lines_3d({base+side*0.4f+Vec3{0,3.18f,0}+n*0.2f,base-side*0.4f+Vec3{0,3.18f,0}+n*0.2f},status,4);
    if(preview){
        const Vec3 a=workshop::world(item,{-workshop::kApproachHalfWidth,0.04f,workshop::kApproachFar});
        const Vec3 b=workshop::world(item,{workshop::kApproachHalfWidth,0.04f,workshop::kApproachFar});
        const Vec3 c=workshop::world(item,{workshop::kApproachHalfWidth,0.04f,-workshop::kHalfZ});
        const Vec3 d=workshop::world(item,{-workshop::kApproachHalfWidth,0.04f,-workshop::kHalfZ});
        r.draw_lines_3d({a,b,b,c,c,d,d,a},valid?Color{1,0.70f,0.18f,1}:Color{1,0.15f,0.10f,1},2.4f);
    }
}

void GameApp::render_build_queue_highlight(SceneRenderer& r,const ConstructionTargetRef& target) const {
    constexpr Color cyan{.31f,.94f,1.f,1.f};
    constexpr Vec3 lift{0,.08f,0};
    const bool show_hulls=habitat_view_mode_==HabitatViewMode::FullHull;
    const auto structure=[&](const auto& instance,const auto& mesh,bool show_edges){
        const Vec3 position=instance.position+lift;
        draw_structure_ground_reservation(r,mesh.footprint_x,mesh.footprint_z,position,
                                          instance.rotation_degrees,cyan,3.4f);
        if(show_edges) r.draw_lines_3d(transformed_vertices(mesh.edges,position,instance.rotation_degrees),cyan,3.4f);
    };
    const auto route=[&](const auto& link,bool show_edges){
        auto lines=show_edges && !link.mesh.edges.empty()?link.mesh.edges:construction_path_lines(link.path);
        for(auto& point:lines) point+=lift;
        r.draw_lines_3d(lines,cyan,3.4f);
    };
    const auto i=target.index;
    switch(target.kind){
        case ConstructionTargetKind::Habitat:
            structure(placed_habitats_[i],habitat_models_.mesh(placed_habitats_[i]),show_hulls);
            break;
        case ConstructionTargetKind::Resource:
            structure(placed_resources_[i],resource_models_.mesh(placed_resources_[i]),
                      show_hulls || !resource_has_hull(placed_resources_[i].kind));
            break;
        case ConstructionTargetKind::Corridor: route(corridor_network_.connections()[i],show_hulls);break;
        case ConstructionTargetKind::Utility: route(utility_network_.connections()[i],true);break;
        case ConstructionTargetKind::Walkway: route(walkway_network_.connections()[i],true);break;
        default: break;
    }
}

void GameApp::render_robot_blockers(SceneRenderer& r){
    constexpr Color red{1.0f,.08f,.10f,1.0f};
    const bool show_hulls=habitat_view_mode_==HabitatViewMode::FullHull;
    const auto outline=[&](Vec3 center,float rotation,float width,float depth){
        std::vector<Vec3> boundary;
        const float x=width*.5f+.22f,z=depth*.5f+.22f;
        const Vec3 a{-x,.07f,-z},b{x,.07f,-z},c{x,.07f,z},d{-x,.07f,z};
        for(const Vec3 point:{a,b,b,c,c,d,d,a}) boundary.push_back(center+rotate_y(point,rotation));
        r.draw_lines_3d(boundary,red,4.0f);
    };
    std::vector<std::uint64_t> drawn;
    for(const auto& rover:builder_robots_){
        const auto sequence=rover.blocking_structure;
        if(!sequence||rover.blocker_display_seconds<=0||
           std::find(drawn.begin(),drawn.end(),sequence)!=drawn.end()) continue;
        drawn.push_back(sequence);
        if(sequence==kLanderBlockerSequence&&lander_.active()){
            outline(lander_.site,0,10.7f,10.7f);
            outline(lander_.site+Vec3{0,0,7.15f},0,4.24f,7.1f);
            continue;
        }
        for(const auto& h:placed_habitats_) if(h.placement_sequence==sequence){
            const auto& mesh=habitat_models_.mesh(h);
            outline(h.position,h.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(show_hulls) r.draw_lines_3d(transformed_vertices(mesh.edges,h.position,h.rotation_degrees),red,3.3f);
        }
        for(const auto& item:placed_resources_) if(item.placement_sequence==sequence){
            const auto& mesh=resource_models_.mesh(item);
            outline(item.position,item.rotation_degrees,mesh.footprint_x,mesh.footprint_z);
            if(show_hulls || !resource_has_hull(item.kind))
                r.draw_lines_3d(transformed_vertices(mesh.edges,item.position,item.rotation_degrees),red,3.3f);
        }
        for(const auto& corridor:corridor_network_.connections()) if(corridor.placement_sequence==sequence)
            r.draw_lines_3d(show_hulls?corridor.mesh.edges:construction_path_lines(corridor.path),red,3.6f);
        for(const auto& walkway:walkway_network_.connections()) if(walkway.placement_sequence==sequence)
            r.draw_lines_3d(walkway.mesh.edges,red,3.6f);
    }
}

void GameApp::render_workshop_services(SceneRenderer& r,std::size_t index) const {
    const auto& home=placed_resources_[index];if(!home.construction.complete())return;
    const bool operational=workshop_operational(index);
    const auto mapping=station_mapping(home.position,home.rotation_degrees,173);
    if(operational && r.lunar_scene_active() && lunar_look::powered(home.supply,home.power_enabled))
        for(const auto& fixture:lunar_look::workshop_fixtures(home))lunar_look::draw_fixture(r,fixture,home.rotation_degrees);
    resources::Builder tools,hoses;
    for(int slot=0;slot<workshop::kBays;++slot){
        const Vec3 p=workshop::bay_local(slot);const float sign=p.x<0?-1.f:1.f;
        const auto robot=std::find_if(builder_robots_.begin(),builder_robots_.end(),[&](const auto& rover){
            return rover.workshop_index==index && rover.workshop_bay==slot && rover.workshop_parked;
        });
        const bool parked=operational && robot!=builder_robots_.end();
        const bool repair=parked && robot->condition<.999f;
        const bool charging=parked && robot->battery_charge_kwh<robot->battery_capacity_kwh*.995f;
        const structure_display::Mount screen{{sign*4.55f,2.40f,p.z+.72f},{0,0,sign},{0,1,0},.56f,.39f};
        std::vector<Vec3> case_mesh,glass;structure_display::casing(case_mesh,glass,screen);
        structure_display::box(case_mesh,{sign*4.65f,2.40f,p.z+.72f},screen.right,screen.up,screen.normal(),{.30f,.22f,.04f});
        structure_display::box(case_mesh,{sign*4.65f,2.20f,p.z+.72f},screen.right,screen.up,screen.normal(),{.10f,.42f,.06f});
        if(!mesh_layout_active()){
            r.draw_mapped_lunar_surface(transformed_vertices(case_mesh,home.position,home.rotation_degrees),LunarMaterial::Alloy,{.22f,.27f,.29f,1},mapping);
            r.draw_mapped_lunar_surface(transformed_vertices(glass,home.position,home.rotation_degrees),operational?LunarMaterial::Display:LunarMaterial::Rubber,{.02f,.04f,.05f,1},mapping);
            structure_display::Content info;
            info.title="SERVICE BAY "+std::to_string(slot+1);
            info.rows={parked?(repair?"REPAIRING":charging?"CHARGING":"READY"):"BAY EMPTY",
                "BAT "+structure_display::value(parked&&robot->battery_capacity_kwh>0?100*robot->battery_charge_kwh/robot->battery_capacity_kwh:0)+"%",
                "HULL "+(parked?structure_display::value(robot->condition*100)+"%":"--")};
            if(!parked)info.rows[1]="CHARGER STANDBY";
            const auto eye=first_person_mode_==FirstPersonMode::Active?first_person_.eye_position():camera_.eye_position();
            structure_display::draw(r,screen,home.position,home.rotation_degrees,info,operational,eye);
        }
        resources::Builder lamp;
        lamp.box(lamp.mesh.body,{sign*4.61f,2.77f,p.z+.72f},{.035f,.09f,.42f},false);
        const Color state=!operational?Color{.16f,.17f,.18f,1}:repair?Color{1.f,.51f,.08f,1}:
            charging?Color{.15f,.78f,1.f,1}:parked?Color{.22f,1.f,.44f,1}:Color{.37f,.50f,.55f,1};
        const auto lamp_vertices=transformed_vertices(lamp.mesh.body,home.position,home.rotation_degrees);
        if(mesh_layout_active())mesh_wireframe::draw_layer(r,lamp_vertices,state,1.3f);
        else r.draw_mapped_lunar_surface(lamp_vertices,operational?LunarMaterial::Emissive:LunarMaterial::Rubber,state,mapping);
        // The tool deploys only for a powered, occupied bay needing repair.
        // Motion follows real repair progress, and therefore freezes with simulation.
        const float cycle=repair?std::sin((1.f-robot->condition)*80.f):0.f;
        const Vec3 root{sign*3.95f,2.15f,p.z};
        const Vec3 tip=repair?Vec3{sign*3.74f,1.45f,p.z+.16f*cycle}:Vec3{sign*4.12f,2.06f,p.z};
        tools.cylinder_between(tools.mesh.body,root,tip,.046f,8);
        tools.box(tools.mesh.body,tip,{.12f,.16f,.15f},false);
        hoses.cylinder_between(hoses.mesh.body,{sign*4.68f,.93f,p.z+.66f},
            {sign*4.62f,.49f,p.z+.47f},.035f,6);
        if(charging)hoses.cylinder_between(hoses.mesh.body,{sign*4.62f,.49f,p.z+.47f},
            {sign*4.02f,.62f,p.z+.32f},.035f,6);
    }
    if(mesh_layout_active()){
        mesh_wireframe::draw_layer(r,transformed_vertices(tools.mesh.body,home.position,home.rotation_degrees),{.3f,.9f,.4f,1},1.4f);
        return;
    }
    r.draw_mapped_lunar_surface(transformed_vertices(tools.mesh.body,home.position,home.rotation_degrees),LunarMaterial::Alloy,{.65f,.69f,.70f,1},mapping);
    r.draw_mapped_lunar_surface(transformed_vertices(hoses.mesh.body,home.position,home.rotation_degrees),LunarMaterial::Rubber,{.055f,.065f,.075f,1},mapping);
}

void GameApp::render_resources(SceneRenderer& r){
    const bool solid=!mesh_layout_active();
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& instance=placed_resources_[i];
        const bool selected=selection_kind_==SelectionKind::Resource && selected_resource_==(int)i;
        const auto& base_mesh=resource_runtime_mesh(i);
        const auto& mesh=instance.kind==ResourceKind::StorageDome?cargo_port_mesh(base_mesh,instance):base_mesh;
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(instance.kind==ResourceKind::StorageDome && instance.construction.complete()){
            fpv_doors=storage_dome_doors(i);
            fpv_ptr=&fpv_doors;
        }
        if(instance.kind==ResourceKind::SolarPanel && instance.construction.complete()){
            const Vec3 visual_normal=instance.solar_visual_initialized?instance.solar_visual_normal:solar_tracking::fixed_panel_normal(instance.rotation_degrees);
            draw_tracked_solar(r,mesh,instance,visual_normal,selected,Color{1.0f,0.94f,0.55f,1.0f},solid);
        }
        else
            render_resource_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.progress()*(float)kConstructionSteps,fpv_ptr,instance.kind);
        if(r.lunar_scene_active() && instance.kind==ResourceKind::LandingPad && instance.construction.complete() &&
           lunar_look::powered(instance.supply,instance.power_enabled)){
            for(const auto& fixture:lunar_look::pad_fixtures(instance,mesh))
                lunar_look::draw_fixture(r,fixture,instance.rotation_degrees);
        }
        if(fpv_ptr) for(const auto& door:fpv_doors) draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position),solid);
        if(instance.kind==ResourceKind::RobotWorkshop){render_workshop_gate(r,instance,!instance.construction.complete());render_workshop_services(r,i);}
        if(instance.kind==ResourceKind::StorageGate && instance.storage_parent_index<placed_resources_.size()){
            const auto& owner=placed_resources_[instance.storage_parent_index];
            render_storage_gate_shutters(r,owner,instance.storage_loading_face,instance.storage_extension_open,instance.construction.progress());
        }

        if(instance.kind==ResourceKind::OreDriller && instance.construction.complete()){
            ResourceBoxBatch raw_ore;
            const auto pad=driller_stockyard(instance);
            for(std::size_t ore_kind=0;ore_kind<ore::materials.size();++ore_kind){
                const int count=ore::boxes(instance.mined_ore_kg[ore_kind]);
                for(int box=0;pad && box<count;++box){
                    const auto base=pad->box(ore_kind,box);
                    draw_resource_box(r,base,.20f,.31f,ore::materials[ore_kind],false,&raw_ore);
                }
            }
            raw_ore.draw(r);
        }

        if(instance.kind!=ResourceKind::StorageDome || !instance.construction.complete() ||
           (instance.stored_boxes.empty() && stored_ore_mass_kg(instance)==0)) continue;

        // Every stored resource remains a physical human-portable package. The
        // same deterministic pallet pose is used by FPV picking, so the box a
        // player sees is exactly the box that can be grabbed.
        ResourceBoxBatch boxes;
        for(std::size_t bi=0;bi<instance.stored_boxes.size();++bi){
            Vec3 base{}; float half=0.0f,box_h=0.0f;
            stored_resource_box_pose(instance,mesh,bi,base,half,box_h);
            draw_resource_box(r,base,half,box_h,instance.stored_boxes[bi].resource,false,&boxes);
        }
        std::size_t raw_slot=instance.stored_boxes.size();
        for(std::size_t kind=0;kind<ore::materials.size();++kind){
            // Show up to a rack of small crates per ore type. The inventory
            // panel and capacity accounting retain the exact stored mass.
            const auto visible=std::min(instance.stored_ore_boxes[kind],kVisibleOreBoxesPerType);
            for(std::uint32_t box=0;box<visible;++box){
                Vec3 base{};float half=0,box_h=0;
                stored_resource_box_pose(instance,mesh,raw_slot++,base,half,box_h);
                draw_resource_box(r,base,half,box_h,ore::materials[kind],false,&boxes);
            }
        }
        boxes.draw(r);
    }
}

void GameApp::render_utilities(SceneRenderer& r){
    const auto& links=utility_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& link=links[i];
        const bool selected=selection_kind_==SelectionKind::Utility && selected_utility_==(int)i;
        const float boost=selected?1.55f:1.0f;

        if(!link.construction.complete()){
            const float fraction=link.construction.progress();
            const Color build_main{1.00f,0.68f,0.12f,0.99f};
            const Color build_detail{0.96f,0.56f,0.08f,0.96f};
            const Color ghost{0.10f,0.40f,0.90f,0.46f};
            mesh_wireframe::draw_layer(r,link.mesh.shell,ghost,1.50f*boost);
            mesh_wireframe::draw_layer(r,link.mesh.details,ghost,1.25f*boost);
            if(!link.mesh.edges.empty()) r.draw_lines_3d(link.mesh.edges,{0.14f,0.54f,1.00f,0.58f},1.65f*boost);
            if(fraction<=0.0001f){
                r.draw_lines_3d(construction_path_lines(link.path),Color{0.12f,0.58f,1.00f,0.92f},selected?3.2f:2.3f);
            }else{
                const auto shell=construction_sequence::reveal_triangles(link.mesh.shell,fraction);
                const auto details=construction_sequence::reveal_triangles(link.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(link.mesh.edges,fraction);
                                mesh_wireframe::draw_layer(r,shell,build_main,2.05f*boost);
                mesh_wireframe::draw_layer(r,details,build_detail,1.80f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,build_main,selected?3.35f:2.30f);
            }
            continue;
        }

        const bool conveyor=link.kind==UtilityKind::ConveyorBelt;
        if(!mesh_layout_active()){
            r.draw_lunar_surface(link.mesh.shell,LunarMaterial::Paint,conveyor?Color{0.44f,0.47f,0.50f,1}:Color{0.63f,0.67f,0.67f,1});
            r.draw_lunar_surface(link.mesh.details,LunarMaterial::Alloy,{0.70f,0.55f,0.30f,1});
            if(selected) r.draw_lines_3d(link.mesh.edges,theme_.accent,2.1f);
        }else{
            const Color shell=conveyor?Color{.82f,.64f,.33f,1}:Color{0.24f,0.96f,0.42f,1};
            const Color details=conveyor?shell:Color{0.22f,0.92f,0.40f,0.99f};
            mesh_wireframe::draw_layer(r,link.mesh.shell,shell,2.60f*boost);
            mesh_wireframe::draw_layer(r,link.mesh.details,details,2.35f*boost);
            r.draw_lines_3d(link.mesh.edges,selected?theme_.accent:shell,selected?3.60f:2.85f);
        }
        if(conveyor && link.ore_cargo<ore::no_cargo)
            draw_resource_box(r,link.ore_box_position(),.23f,.34f,ore::materials[link.ore_cargo]);
    }
}


void GameApp::render_walkways(SceneRenderer& r){
    const auto& links=walkway_network_.connections();
    for(size_t i=0;i<links.size();++i){
        const auto& w=links[i];
        const bool selected=selection_kind_==SelectionKind::Walkway && selected_walkway_==(int)i;
        const float boost=selected?1.55f:1.0f;
        if(!w.construction.complete()){
            const float fraction=w.construction.progress();
            const Color build_main{1.00f,0.68f,0.12f,0.99f};
            const Color build_detail{0.96f,0.56f,0.08f,0.96f};
            const Color ghost{0.10f,0.40f,0.90f,0.46f};
            mesh_wireframe::draw_layer(r,w.mesh.deck,ghost,1.50f*boost);
            mesh_wireframe::draw_layer(r,w.mesh.details,ghost,1.25f*boost);
            if(!w.mesh.edges.empty()) r.draw_lines_3d(w.mesh.edges,{0.14f,0.54f,1.00f,0.58f},1.65f*boost);
            if(fraction<=0.0001f){
                r.draw_lines_3d(construction_path_lines(w.path),Color{0.12f,0.58f,1.00f,0.92f},selected?3.2f:2.3f);
            }else{
                const auto deck=construction_sequence::reveal_triangles(w.mesh.deck,fraction);
                const auto details=construction_sequence::reveal_triangles(w.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(w.mesh.edges,fraction);
                        mesh_wireframe::draw_layer(r,deck,build_main,2.05f*boost);
                mesh_wireframe::draw_layer(r,details,build_detail,1.80f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,build_main,selected?3.35f:2.30f);
            }
            continue;
        }
        if(!mesh_layout_active()){
            r.draw_lunar_surface(w.mesh.deck,LunarMaterial::Pad,{0.46f,0.49f,0.51f,1});
            r.draw_lunar_surface(w.mesh.details,LunarMaterial::Paint,{0.71f,0.66f,0.53f,1});
            if(selected) r.draw_lines_3d(w.mesh.edges,theme_.accent,2.1f);
            continue;
        }
        mesh_wireframe::draw_layer(r,w.mesh.deck,{0.24f,0.96f,0.42f,1.0f},2.60f*boost);
        mesh_wireframe::draw_layer(r,w.mesh.details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
        r.draw_lines_3d(w.mesh.edges,selected?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.60f:2.85f);
    }
}

void GameApp::render_corridors(SceneRenderer& r){
    corridor_network_.refresh_pressure_shells();
    const bool solid=!mesh_layout_active();
    const bool show_hull=habitat_view_mode_==HabitatViewMode::FullHull;
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& connection=corridors[i];
        const bool selected=selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)i;
        const bool link_source=corridor_link_source_==(int)i;
        const Color edge_color=(selected||link_source)?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f};
        const float boost=(selected||link_source)?1.55f:1.0f;
        if(!show_hull)draw_faded_hull(r,connection.mesh.shell,empty_mesh_layer,connection.mesh.edges,{},0,solid,connection.construction.progress());
        const auto& shell_layer=show_hull?connection.mesh.shell:empty_mesh_layer;
        const auto& edge_layer=show_hull?connection.mesh.edges:empty_mesh_layer;

        if(!connection.construction.complete()){
            for(const auto* endpoint:{&connection.endpoint_a,&connection.endpoint_b})
                if(endpoint->kind==CorridorEndpointKind::Habitat || endpoint->kind==CorridorEndpointKind::Resource)
                    draw_sliding_door(r,endpoint->door,0,solid,connection.construction.progress());
            const float fraction=connection.construction.progress();
            const Color build_main{1.00f,0.68f,0.12f,0.99f};
            const Color build_detail{0.96f,0.56f,0.08f,0.96f};
            const Color ghost{0.10f,0.40f,0.90f,0.46f};
            mesh_wireframe::draw_layer(r,shell_layer,ghost,1.55f*boost);
            mesh_wireframe::draw_layer(r,connection.mesh.details,ghost,1.30f*boost);
            if(!edge_layer.empty()) r.draw_lines_3d(edge_layer,{0.14f,0.54f,1.00f,0.58f},1.70f*boost);
            if(fraction<=0.0001f || (!show_hull && (selected || link_source))){
                r.draw_lines_3d(construction_path_lines(connection.path),Color{0.12f,0.58f,1.00f,0.92f},(selected||link_source)?3.3f:2.4f);
            }
            if(fraction>0.0001f){
                const auto shell=construction_sequence::reveal_triangles(shell_layer,fraction);
                const auto details=construction_sequence::reveal_triangles(connection.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(edge_layer,fraction);
                mesh_wireframe::draw_layer(r,details,build_detail,1.80f*boost);
                mesh_wireframe::draw_layer(r,shell,build_main,2.05f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,build_main,(selected||link_source)?3.45f:2.35f);
            }
            continue;
        }

        if(solid){
            r.draw_lunar_surface(connection.mesh.details,LunarMaterial::Alloy,{0.60f,0.63f,0.64f,1});
            const auto& panels=hull_surface::split(shell_layer,geometry::kCorridorDeckY+1.0f);
            r.draw_lunar_surface(panels.lower,LunarMaterial::Hull,{.76f,.78f,.73f,1});
            r.draw_lunar_surface(panels.upper,LunarMaterial::SolarShield,{.38f,.59f,.65f,.28f});
        }else{
            mesh_wireframe::draw_layer(r,connection.mesh.details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
            mesh_wireframe::draw_layer(r,shell_layer,{0.24f,0.96f,0.42f,1.0f},2.65f*boost);
        }
        if(selected || link_source)
            r.draw_lines_3d(show_hull?edge_layer:construction_path_lines(connection.path),edge_color,solid?2.2f:3.65f);
        else if(!solid && show_hull) r.draw_lines_3d(edge_layer,edge_color,2.90f);
    }
}


void GameApp::render_habitats(SceneRenderer& r){
    const bool solid=!mesh_layout_active();
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const auto& mesh=habitat_runtime_mesh(i);
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(instance.construction.complete()){
            fpv_doors=habitat_doors(i);
            fpv_ptr=&fpv_doors;
        }
        const bool greenhouse_dark=instance.kind==HabitatKind::BioDome && instance.construction.complete() && !hydroponics_lights_on(status_.hour());
        render_habitat_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.progress()*(float)kConstructionSteps,fpv_ptr,greenhouse_dark,instance.kind,lunar_look::powered(instance.supply,instance.power_enabled));
        render_habitat_screens(r,i);
        if(r.lunar_scene_active() && instance.construction.complete() && lunar_look::powered(instance.supply,instance.power_enabled))
            lunar_look::draw_fixture(r,lunar_look::habitat_fixture(instance,mesh),instance.rotation_degrees);
        if(fpv_ptr) for(const auto& door:fpv_doors){
            draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position),solid);
            if(instance.kind==HabitatKind::AirLock && door.entrance_index==-4)
                draw_generated_airlock_access(r,door,mesh.walkable_floor_y,
                    selected?Color{1.0f,0.94f,0.55f,1.0f}:Color{0.24f,0.96f,0.42f,1.0f},solid);
        }

        // Legacy authored access arrays are normally empty from v0.43.17 onward.
        // for an authored hatch that is currently the surface side of a
        // completed pressure network.  Ordinary structures never expose unused
        // entrances directly to the lunar surface.
        if(instance.construction.complete()) for(size_t entrance_index=0;entrance_index<mesh.entrance_access.size();++entrance_index){
            if(!habitat_exterior_access_enabled(i,entrance_index)) continue;
            const auto& access=mesh.entrance_access[entrance_index];
            if(entrance_access_hits_corridor(access,instance.position,instance.rotation_degrees,corridor_network_)) continue;
            const float access_fraction=instance.construction.complete()?1.0f:0.5f;
            const auto details=transformed_vertices(construction_sequence::reveal_triangles(access.details,access_fraction),instance.position,instance.rotation_degrees);
            const auto edges=transformed_vertices(construction_sequence::reveal_lines(access.edges,instance.construction.complete()?1.0f:0.0f),instance.position,instance.rotation_degrees);
            const Color access_color=instance.construction.complete()?Color{0.24f,0.96f,0.42f,1.0f}:Color{1.00f,0.68f,0.12f,0.99f};
            if(solid) r.draw_lunar_surface(details,LunarMaterial::Alloy,{0.54f,0.57f,0.59f,1});
            else mesh_wireframe::draw_layer(r,details,access_color,instance.construction.complete()?2.45f:2.0f);
            if(!edges.empty()){
                const Color ec=selected?Color{1.0f,0.94f,0.55f,1.0f}:Color{0.08f,0.11f,0.13f,1.0f};
                r.draw_lines_3d(edges,ec,selected?3.35f:(instance.construction.complete()?2.55f:2.25f));
            }
        }
    }
}

} // namespace game
