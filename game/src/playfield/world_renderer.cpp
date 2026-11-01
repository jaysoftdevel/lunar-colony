#include "game/game_app.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/resource_builder.hpp"
#include "game/world/robot_workshop.hpp"
#include "game/ui/layout.hpp"
#include "game/util/world_math.hpp"
#include "game/util/mesh_wireframe.hpp"
#include "game/world/construction_sequence.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/world/world_labels.hpp"
#include "game/version.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace game {
using namespace eng;
using namespace layout;
using namespace world_math;
using namespace world_labels;

namespace {
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
    void append(const std::vector<Vec3>& triangles,const std::vector<Vec3>& lines,MaterialResource material){
        if(faces.empty() || faces.back().material!=material)faces.push_back({material,{}});
        auto& batch=faces.back().vertices;batch.insert(batch.end(),triangles.begin(),triangles.end());
        edges.insert(edges.end(),lines.begin(),lines.end());
    }
    void draw(Renderer& r) const {
        // Inventory packages never intersect. Keep face order and consolidate
        // their common outline after the depth-tested faces, instead of two
        // GPU submissions for every individual 10 kg package.
        for(const auto& batch:faces){const auto c=resource_box_color(batch.material);r.draw_triangles_3d(batch.vertices,{c.r,c.g,c.b,0.90f});}
        if(!edges.empty())r.draw_lines_3d(edges,{0.10f,0.12f,0.13f,1},1.25f);
    }
};

void draw_resource_box(Renderer& r,Vec3 base,float half_width,float height,MaterialResource resource,bool selected=false,ResourceBoxBatch* batch=nullptr){
    const Vec3 c=base+Vec3{0,height*0.5f+0.025f,0};
    const float x0=c.x-half_width,x1=c.x+half_width,z0=c.z-half_width,z1=c.z+half_width,y0=c.y-height*0.5f,y1=c.y+height*0.5f;
    const std::vector<Vec3> tris{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z1},{x0,y0,z0},{x1,y0,z1},{x0,y0,z1},
        {x0,y1,z0},{x1,y1,z1},{x1,y1,z0},{x0,y1,z0},{x0,y1,z1},{x1,y1,z1},
        {x0,y0,z0},{x0,y1,z0},{x1,y1,z0},{x0,y0,z0},{x1,y1,z0},{x1,y0,z0},
        {x1,y0,z1},{x1,y1,z1},{x0,y1,z1},{x1,y0,z1},{x0,y1,z1},{x0,y0,z1},
        {x0,y0,z1},{x0,y1,z1},{x0,y1,z0},{x0,y0,z1},{x0,y1,z0},{x0,y0,z0},
        {x1,y0,z0},{x1,y1,z0},{x1,y1,z1},{x1,y0,z0},{x1,y1,z1},{x1,y0,z1}
    };
    const std::vector<Vec3> edges{
        {x0,y0,z0},{x1,y0,z0},{x1,y0,z0},{x1,y0,z1},{x1,y0,z1},{x0,y0,z1},{x0,y0,z1},{x0,y0,z0},
        {x0,y1,z0},{x1,y1,z0},{x1,y1,z0},{x1,y1,z1},{x1,y1,z1},{x0,y1,z1},{x0,y1,z1},{x0,y1,z0},
        {x0,y0,z0},{x0,y1,z0},{x1,y0,z0},{x1,y1,z0},{x1,y0,z1},{x1,y1,z1},{x0,y0,z1},{x0,y1,z1}
    };
    if(batch){batch->append(tris,edges,resource);return;}
    const Color base_color=resource_box_color(resource);
    r.draw_triangles_3d(tris,{base_color.r,base_color.g,base_color.b,0.90f});
    r.draw_lines_3d(edges,selected?Color{0.34f,1.00f,0.72f,1.0f}:Color{0.10f,0.12f,0.13f,1.0f},selected?2.8f:1.25f);
}


Vec3 rotate_from_to(Vec3 v,Vec3 from,Vec3 to){
    from=normalize(from);to=normalize(to);
    const float c=std::clamp(from.x*to.x+from.y*to.y+from.z*to.z,-1.0f,1.0f);
    Vec3 axis=cross(from,to);
    float sn=length(axis);
    if(sn<1.0e-5f){
        if(c>0.0f)return v;
        axis=normalize(std::abs(from.x)<0.8f?cross(from,{1,0,0}):cross(from,{0,1,0}));
        return axis*(2.0f*(axis.x*v.x+axis.y*v.y+axis.z*v.z))-v;
    }
    axis=axis*(1.0f/sn);
    const float d=axis.x*v.x+axis.y*v.y+axis.z*v.z;
    return v*c+cross(axis,v)*sn+axis*(d*(1.0f-c));
}

std::vector<Vec3> tracked_solar_vertices(const std::vector<Vec3>& src,Vec3 position,float base_rotation,Vec3 sun_direction){
    constexpr float pitch=24.0f*3.14159265358979323846f/180.0f;
    const Vec3 authored_normal{0,std::cos(pitch),-std::sin(pitch)};
    const Vec3 target_world=sun_direction.y>0.0f?normalize(sun_direction):rotate_y(authored_normal,base_rotation);
    const Vec3 target_local=rotate_y(target_world,-base_rotation);
    const Vec3 pivot{0,3.38f,0};
    std::vector<Vec3> local;local.reserve(src.size());
    for(Vec3 v:src){
        // The pedestal and inverter remain fixed. Everything above the azimuth
        // bearing follows the two-axis collector head, including cell-grid lines.
        if(v.y>3.12f)v=pivot+rotate_from_to(v-pivot,authored_normal,target_local);
        local.push_back(v);
    }
    return transformed_vertices(local,position,base_rotation);
}

void draw_tracked_solar(Renderer& r,const ResourceMesh& mesh,const PlacedResource& instance,Vec3 sun_direction,bool selected,Color accent){
    const float boost=selected?1.45f:1.0f;
    const auto foundation=transformed_vertices(mesh.foundation,instance.position,instance.rotation_degrees);
    const auto body=tracked_solar_vertices(mesh.body,instance.position,instance.rotation_degrees,sun_direction);
    const auto details=tracked_solar_vertices(mesh.details,instance.position,instance.rotation_degrees,sun_direction);
    const auto edges=tracked_solar_vertices(mesh.edges,instance.position,instance.rotation_degrees,sun_direction);
    mesh_wireframe::draw_layer(r,foundation,{0.20f,0.84f,0.36f,1.0f},2.40f*boost);
    mesh_wireframe::draw_layer(r,body,{0.24f,0.96f,0.42f,1.0f},2.65f*boost);
    mesh_wireframe::draw_layer(r,details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
    r.draw_lines_3d(edges,selected?accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.65f:2.90f);
}


const std::vector<Vec3>& portal_layer(const std::vector<Vec3>& triangles,Vec3 position,float rotation,
                                       const std::vector<CorridorDoor>* doors,bool lines=false,bool foundation=false){
    if(!doors || doors->empty()) return triangles;
    struct Cache {
        const Vec3* source{};std::size_t size{};bool lines{};
        std::vector<mesh_portals::Aperture> openings;
        std::vector<Vec3> triangles;
    };
    static std::vector<Cache> cache;
    std::vector<mesh_portals::Aperture> openings;
    for(const auto& door:*doors){
        const Vec3 center=rotate_y(door.wall_position-position,-rotation);
        openings.push_back({center,rotate_y(door.normal,-rotation),0.90f,
            geometry::kDoorClearWidth*0.5f,foundation?geometry::kCorridorDeckY-position.y-0.15f:door.position.y-position.y+0.015f,
            door.position.y-position.y+geometry::kDoorClearHeight});
    }
    for(const auto& entry:cache){
        if(entry.source!=triangles.data() || entry.size!=triangles.size() || entry.lines!=lines || entry.openings.size()!=openings.size()) continue;
        bool same=true;
        for(std::size_t i=0;i<openings.size();++i){
            const auto& a=openings[i];const auto& b=entry.openings[i];
            same=same && length(a.center-b.center)<0.0001f && length(a.normal-b.normal)<0.0001f &&
                std::abs(a.bottom-b.bottom)<0.0001f && std::abs(a.top-b.top)<0.0001f;
        }
        if(same) return entry.triangles;
    }
    if(cache.size()>256) cache.clear();
    Cache entry{triangles.data(),triangles.size(),lines,openings,triangles};
    for(const auto& opening:openings) entry.triangles=lines?mesh_portals::subtract_lines(entry.triangles,opening):mesh_portals::subtract(entry.triangles,opening);
    cache.push_back(std::move(entry));
    return cache.back().triangles;
}

void draw_extension_mesh(Renderer& r,const ResourceMesh& mesh,float progress,bool preview=false){
    const Color blueprint{0.10f,0.42f,0.96f,0.52f},installed{1,0.68f,0.12f,0.99f},complete{0.24f,0.96f,0.42f,1};
    for(const auto* layer:{&mesh.foundation,&mesh.body,&mesh.details}){
        if(preview || progress<0.9999f) mesh_wireframe::draw_layer(r,*layer,blueprint,1.50f);
        if(!preview && progress>0) mesh_wireframe::draw_layer(r,construction_sequence::reveal_triangles(*layer,progress),progress>=0.9999f?complete:installed,2.35f);
    }
    if(!mesh.edges.empty()) r.draw_lines_3d(mesh.edges,preview || progress<0.9999f?blueprint:complete,2.1f);
}
void draw_pressure_box(Renderer& r,Vec3 center,Vec3 side,Vec3 normal,Vec3 size,Color color){
    resources::Builder b;b.oriented_box(b.mesh.body,center,side,{0,1,0},normal,size);
    mesh_wireframe::draw_layer(r,b.mesh.body,color,2.35f);
    r.draw_lines_3d(b.mesh.edges,color,2.1f);
}
const ResourceMesh& cargo_port_mesh(const ResourceMesh& source,const PlacedResource& owner){
    struct Entry {const ResourceMesh* source;std::array<int,3> faces;std::array<float,3> angles;ResourceMesh mesh;};
    static std::vector<Entry> cache;
    const auto faces=storage_loading::faces(owner,true);
    const std::array<float,3> angles{{owner.storage_gate_angle_degrees,owner.storage_extra_angles[0],owner.storage_extra_angles[1]}};
    for(const auto& entry:cache) if(entry.source==&source && entry.faces==faces && entry.angles==angles) return entry.mesh;
    if(cache.size()>128) cache.clear();
    Entry entry{&source,faces,angles,source};
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
void draw_sliding_door(Renderer& r,const CorridorDoor& door,float open_fraction,float progress=1){
    Vec3 n=door.normal;n.y=0;n=normalize(n);if(length(n)<0.001f)n={1,0,0};
    const Vec3 side{-n.z,0,n.x};
    const float w=geometry::kDoorClearWidth,h=geometry::kDoorClearHeight;
    const Vec3 base{door.wall_position.x,door.position.y,door.wall_position.z};
    const float slide=pressure_door::kSlide*std::clamp(open_fraction,0.0f,1.0f);
    resources::Builder b;
    auto box=[&](Vec3 center,Vec3 size){b.oriented_box(b.mesh.body,center,side,{0,1,0},n,size);};
    for(int sign:{-1,1}){
        box(base+side*((float)sign*(w*0.5f+0.08f))+Vec3{0,h*0.5f,0},{0.16f,h+0.16f,0.24f});
        box(base+side*((float)sign*(w*0.25f+slide))+Vec3{0,h*0.5f,0},{2*pressure_door::kPanelHalfWidth,h+0.10f,pressure_door::kThickness});
    }
    box(base+Vec3{0,h+0.08f,0},{w+0.50f,0.16f,0.28f});
    box(base+Vec3{0,0.005f,0},{w+0.25f,0.03f,0.24f});
    draw_extension_mesh(r,b.mesh,progress);
    const Color light=pressure_door::sealed(open_fraction)?Color{1,0.57f,0.12f,1}:Color{0.24f,1,0.56f,1};
    r.draw_lines_3d({base+side*0.25f+Vec3{0,h+0.085f,0}+n*0.15f,base-side*0.25f+Vec3{0,h+0.085f,0}+n*0.15f},light,3.5f);
}
void draw_loading_gate(Renderer& r,const PlacedResource& item,float opening,bool preview=false){
    const Vec3 n=storage_loading::normal(item),side{-n.z,0,n.x};
    const Vec3 wall=storage_loading::wall(item),mouth=storage_loading::mouth(item);
    const float depth=length(mouth-wall);
    const float h=storage_loading::kHeight,w=storage_loading::kHalfWidth;
    resources::Builder chamber;
    auto box=[&](Vec3 center,Vec3 size){chamber.oriented_box(chamber.mesh.body,center,side,{0,1,0},n,size);};
    const Vec3 mid=(wall+mouth)*0.5f;
    box(mid+Vec3{0,0.14f,0},{2*w+0.18f,0.18f,depth+0.20f});
    box(mid+Vec3{0,h+0.18f,0},{2*w+0.18f,0.16f,depth+0.20f});
    for(int sign:{-1,1}) box(mid+side*((float)sign*(w+0.05f))+Vec3{0,h*0.5f+0.15f,0},{0.12f,h,depth+0.20f});
    // Permanent inner pressure bulkhead: the robot services a transfer buffer,
    // never opens a crew exit straight onto the lunar surface.
    box(wall+Vec3{0,h*0.5f+0.15f,0},{2*w,h,0.12f});
    const float progress=item.construction.progress();
    draw_extension_mesh(r,chamber.mesh,progress,preview);
    resources::Builder leaves;
    for(int sign:{-1,1}){
        const Vec3 c=mouth+side*((float)sign*(w*0.5f+opening*(w+0.06f)))+Vec3{0,h*0.5f+0.15f,0}+n*0.08f;
        leaves.oriented_box(leaves.mesh.body,c,side,{0,1,0},n,{w+0.04f,h,0.10f});
    }
    draw_extension_mesh(r,leaves.mesh,progress,preview);
    const Vec3 service=storage_loading::service(item)+Vec3{0,0.04f,0};
    r.draw_lines_3d({service-side*1.25f-n*0.7f,service-side*1.25f+n*0.8f,
        service+side*1.25f-n*0.7f,service+side*1.25f+n*0.8f,
        service-side*1.25f+n*0.8f,service+side*1.25f+n*0.8f},{1,0.70f,0.16f,1},2.2f);
    const Vec3 tip=service-n*0.55f;
    r.draw_lines_3d({service+n*0.4f,tip,tip,tip+n*0.3f+side*0.28f,tip,tip+n*0.3f-side*0.28f},{1,0.70f,0.16f,1},2.5f);
}

}

void GameApp::render_habitat_mesh(Renderer& r,const HabitatMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,float construction_step,const std::vector<CorridorDoor>* fpv_doors) const {
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
        return;
    }

    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::habitat_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto shell=transformed_vertices(construction_sequence::reveal_triangles(mesh.shell,visibility.shell),position,rotation_degrees);
        const auto interior=transformed_vertices(construction_sequence::reveal_triangles(mesh.interior,visibility.interior),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto glazing=transformed_vertices(construction_sequence::reveal_triangles(mesh.glazing,visibility.glazing),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(mesh.edges,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.28f:1.0f;
        // The complete target remains a blue engineering mesh. Every part
        // that has actually been installed replaces that draft with filled,
        // bold yellow construction geometry. The player can therefore read the
        // exact target shape and the completed fraction at the same time.
        const Color blueprint{0.08f,0.36f,0.96f,0.46f};
        const Color installed{1.00f,0.68f,0.12f,0.99f};
        const Color installed_dim{0.96f,0.56f,0.08f,0.96f};
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.foundation,position,rotation_degrees),blueprint,1.35f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.shell,position,rotation_degrees),blueprint,1.55f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.interior,position,rotation_degrees),blueprint,1.25f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.details,position,rotation_degrees),blueprint,1.30f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.glazing,position,rotation_degrees),{0.10f,0.48f,1.00f,0.40f},1.20f*boost);
        const auto ghost_edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        if(!ghost_edges.empty()) r.draw_lines_3d(ghost_edges,{0.12f,0.50f,1.00f,0.62f},1.65f*boost);

        mesh_wireframe::draw_layer(r,foundation,installed_dim,1.95f*boost);
        mesh_wireframe::draw_layer(r,shell,installed,2.20f*boost);
        mesh_wireframe::draw_layer(r,interior,installed_dim,1.85f*boost);
        mesh_wireframe::draw_layer(r,details,installed,1.95f*boost);
        mesh_wireframe::draw_layer(r,glazing,{1.00f,0.76f,0.20f,0.94f},1.80f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,installed,selected?3.45f:2.45f);
        if(construction_step==0){
            const float hx=mesh.footprint_x*0.5f, hz=mesh.footprint_z*0.5f;
            std::vector<Vec3> marker{{-hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,hz},{hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,-hz}};
            r.draw_lines_3d(transformed_vertices(marker,position,rotation_degrees),blueprint,selected?3.2f:2.4f);
        }
        return;
    }

    auto shell=transformed_vertices(portal_layer(mesh.shell,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
    const auto interior=transformed_vertices(portal_layer(mesh.interior,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    auto glazing=transformed_vertices(portal_layer(mesh.glazing,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
    const Color shell_color{0.24f,0.96f,0.42f,1.0f};
    const Color foundation_color{0.20f,0.84f,0.36f,1.0f};
    const Color interior_color{0.34f,1.00f,0.50f,0.98f};
    const Color detail_color{0.22f,0.92f,0.40f,0.99f};
    const Color glass_color{0.32f,1.00f,0.56f,0.82f};
    const Color edge_color=selected?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f};
    const float selected_boost=selected?1.45f:1.0f;

    // Structures intentionally remain mesh-only in this development phase.
    // Completion is communicated by green color and heavier engineering lines,
    // not by filled/material surfaces or textures.
    mesh_wireframe::draw_layer(r,foundation,foundation_color,2.40f*selected_boost);
    mesh_wireframe::draw_layer(r,interior,interior_color,2.20f*selected_boost);
    mesh_wireframe::draw_layer(r,details,detail_color,2.30f*selected_boost);
    mesh_wireframe::draw_layer(r,glazing,glass_color,2.10f*selected_boost);
    if(habitat_view_mode_==HabitatViewMode::Default){
        mesh_wireframe::draw_layer(r,shell,shell_color,2.65f*selected_boost);
    }else{
        mesh_wireframe::draw_layer(r,shell,{shell_color.r,shell_color.g,shell_color.b,0.24f},1.25f);
    }
    r.draw_lines_3d(edges,edge_color,selected?3.65f:2.90f);
}

void GameApp::render_resource_mesh(Renderer& r,const ResourceMesh& mesh,Vec3 position,float rotation_degrees,bool preview,bool valid,bool selected,float construction_step,const std::vector<CorridorDoor>* fpv_doors) const {
    if(preview){
        const auto body=transformed_vertices(portal_layer(mesh.body,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
        const auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
        const auto edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        const Color line=valid?Color{0.34f,1.00f,0.72f,0.96f}:Color{1.00f,0.38f,0.30f,0.96f};
        const Color faint=valid?Color{0.18f,0.78f,0.56f,0.78f}:Color{0.92f,0.25f,0.20f,0.78f};
        mesh_wireframe::draw_layer(r,foundation,faint,1.20f);
        mesh_wireframe::draw_layer(r,body,line,1.35f);
        mesh_wireframe::draw_layer(r,details,faint,1.10f);
        r.draw_lines_3d(edges,line,1.8f);
        return;
    }

    if(construction_step<kConstructionSteps){
        const auto visibility=construction_sequence::resource_visibility(construction_step);
        const auto foundation=transformed_vertices(construction_sequence::reveal_triangles(mesh.foundation,visibility.foundation),position,rotation_degrees);
        const auto body=transformed_vertices(construction_sequence::reveal_triangles(mesh.body,visibility.body),position,rotation_degrees);
        const auto details=transformed_vertices(construction_sequence::reveal_triangles(mesh.details,visibility.details),position,rotation_degrees);
        const auto edges=transformed_vertices(construction_sequence::reveal_lines(mesh.edges,visibility.edges),position,rotation_degrees);
        const float boost=selected?1.28f:1.0f;
        const Color blueprint{0.08f,0.36f,0.96f,0.46f};
        const Color installed{1.00f,0.68f,0.12f,0.99f};
        const Color installed_dim{0.96f,0.56f,0.08f,0.96f};
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.foundation,position,rotation_degrees),blueprint,1.35f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.body,position,rotation_degrees),blueprint,1.55f*boost);
        mesh_wireframe::draw_layer(r,transformed_vertices(mesh.details,position,rotation_degrees),blueprint,1.30f*boost);
        const auto ghost_edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
        if(!ghost_edges.empty()) r.draw_lines_3d(ghost_edges,{0.12f,0.50f,1.00f,0.62f},1.65f*boost);

        mesh_wireframe::draw_layer(r,foundation,installed_dim,1.95f*boost);
        mesh_wireframe::draw_layer(r,body,installed,2.20f*boost);
        mesh_wireframe::draw_layer(r,details,installed,1.95f*boost);
        if(!edges.empty()) r.draw_lines_3d(edges,installed,selected?3.45f:2.45f);
        if(construction_step==0){
            const float hx=mesh.footprint_x*0.5f, hz=mesh.footprint_z*0.5f;
            std::vector<Vec3> marker{{-hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,-hz},{hx,0.03f,hz},{hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,hz},{-hx,0.03f,-hz}};
            r.draw_lines_3d(transformed_vertices(marker,position,rotation_degrees),blueprint,selected?3.2f:2.4f);
        }
        return;
    }

    auto body=transformed_vertices(portal_layer(mesh.body,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto foundation=transformed_vertices(portal_layer(mesh.foundation,position,rotation_degrees,fpv_doors,false,true),position,rotation_degrees);
    auto details=transformed_vertices(portal_layer(mesh.details,position,rotation_degrees,fpv_doors),position,rotation_degrees);
    const auto edges=transformed_vertices(portal_layer(mesh.edges,position,rotation_degrees,fpv_doors,true),position,rotation_degrees);
    const float boost=selected?1.45f:1.0f;
    mesh_wireframe::draw_layer(r,foundation,{0.20f,0.84f,0.36f,1.0f},2.40f*boost);
    mesh_wireframe::draw_layer(r,body,{0.24f,0.96f,0.42f,1.0f},2.65f*boost);
    mesh_wireframe::draw_layer(r,details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
    r.draw_lines_3d(edges,selected?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.65f:2.90f);
}


void GameApp::render_ground_resources(Renderer& r){
    for(std::size_t pile_index=0;pile_index<ground_resource_piles_.size();++pile_index){
        const auto& pile=ground_resource_piles_[pile_index];
        const float half=resource_package_half_width(pile.resource,pile.amount);
        const float h=resource_package_height(pile.resource,pile.amount);
        const bool selected=selection_kind_==SelectionKind::GroundResource &&
            (selected_ground_resource_==(int)pile_index ||
             std::find(selected_ground_resources_.begin(),selected_ground_resources_.end(),(int)pile_index)!=selected_ground_resources_.end());
        draw_resource_box(r,pile.position,half,h,pile.resource,selected);
    }
}

void GameApp::render_storage_loading_gate(Renderer& r,const PlacedResource& item,float opening,bool preview) const {
    draw_loading_gate(r,item,opening,preview);
}

void GameApp::render_storage_gate_shutters(Renderer& r,const PlacedResource& owner,int face,float opening,float progress,bool preview) const {
    const Vec3 n=storage_loading::normal(owner,face),side{-n.z,0,n.x};
    const Vec3 mouth=storage_loading::mouth(owner,face);const float w=storage_loading::kHalfWidth,h=storage_loading::kHeight;
    resources::Builder leaves;
    for(float sign:{-1.0f,1.0f}){
        const Vec3 center=mouth+side*(sign*(w*0.5f+opening*(w+0.06f)))+Vec3{0,h*0.5f+0.15f,0}+n*0.08f;
        leaves.oriented_box(leaves.mesh.body,center,side,{0,1,0},n,{w+0.04f,h,0.10f});
    }
    draw_extension_mesh(r,leaves.mesh,progress,preview);
}

void GameApp::render_workshop_gate(Renderer& r,const PlacedResource& item,bool preview,bool valid) const {
    const Vec3 side=rotate_y({1,0,0},item.rotation_degrees),n=rotate_y({0,0,-1},item.rotation_degrees);
    const Vec3 base=workshop::world(item,{0,workshop::kDeck,-workshop::kHalfZ});
    const float open=item.construction.complete()?item.workshop_gate_open:0;
    const Color color=valid?(preview?Color{0.10f,0.42f,0.96f,0.65f}:Color{0.24f,0.96f,0.42f,1}):Color{1,0.2f,0.1f,1};
    for(float sign:{-1.0f,1.0f}){
        const auto center=base+side*(sign*(workshop::kGateHalf*0.5f+open*(workshop::kGateHalf+0.12f)))+Vec3{0,1.45f,0};
        if(preview){
            resources::Builder b;b.oriented_box(b.mesh.body,center,side,{0,1,0},n,{workshop::kGateHalf+0.06f,2.90f,0.18f});
            mesh_wireframe::draw_layer(r,b.mesh.body,color,1.8f);
        }else draw_pressure_box(r,center,side,n,{workshop::kGateHalf+0.06f,2.90f,0.18f},color);
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

void GameApp::render_resources(Renderer& r){
    for(size_t i=0;i<placed_resources_.size();++i){
        const auto& instance=placed_resources_[i];
        const bool selected=selection_kind_==SelectionKind::Resource && selected_resource_==(int)i;
        const auto& base_mesh=resource_models_.mesh(instance);
        const auto& mesh=instance.kind==ResourceKind::StorageDome?cargo_port_mesh(base_mesh,instance):base_mesh;
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(instance.kind==ResourceKind::StorageDome && instance.construction.complete()){
            fpv_doors=storage_dome_doors(i);
            fpv_ptr=&fpv_doors;
        }
        if(instance.kind==ResourceKind::SolarPanel && instance.construction.complete())
            draw_tracked_solar(r,mesh,instance,status_.sun_direction(),selected,theme_.accent);
        else
            render_resource_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.progress()*(float)kConstructionSteps,fpv_ptr);
        if(fpv_ptr) for(const auto& door:fpv_doors) draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position));
        if(instance.kind==ResourceKind::RobotWorkshop) render_workshop_gate(r,instance,!instance.construction.complete());
        if(instance.kind==ResourceKind::StorageGate && instance.storage_parent_index<placed_resources_.size()){
            const auto& owner=placed_resources_[instance.storage_parent_index];
            render_storage_gate_shutters(r,owner,instance.storage_loading_face,instance.storage_extension_open,instance.construction.progress());
        }

        if(instance.kind!=ResourceKind::StorageDome || !instance.construction.complete() || instance.stored_boxes.empty()) continue;

        // Every stored resource remains a physical human-portable package. The
        // same deterministic pallet pose is used by FPV picking, so the box a
        // player sees is exactly the box that can be grabbed.
        ResourceBoxBatch boxes;
        for(std::size_t bi=0;bi<instance.stored_boxes.size();++bi){
            Vec3 base{}; float half=0.0f,box_h=0.0f;
            stored_resource_box_pose(instance,mesh,bi,base,half,box_h);
            draw_resource_box(r,base,half,box_h,instance.stored_boxes[bi].resource,false,&boxes);
        }
        boxes.draw(r);
    }
}

void GameApp::render_utilities(Renderer& r){
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

        const Color shell{0.24f,0.96f,0.42f,1.0f};
        const Color details{0.22f,0.92f,0.40f,0.99f};
        mesh_wireframe::draw_layer(r,link.mesh.shell,shell,2.60f*boost);
        mesh_wireframe::draw_layer(r,link.mesh.details,details,2.35f*boost);
        r.draw_lines_3d(link.mesh.edges,selected?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.60f:2.85f);
    }
}


void GameApp::render_walkways(Renderer& r){
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
        mesh_wireframe::draw_layer(r,w.mesh.deck,{0.24f,0.96f,0.42f,1.0f},2.60f*boost);
        mesh_wireframe::draw_layer(r,w.mesh.details,{0.22f,0.92f,0.40f,0.99f},2.35f*boost);
        r.draw_lines_3d(w.mesh.edges,selected?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f},selected?3.60f:2.85f);
    }
}

void GameApp::render_corridors(Renderer& r){
    const auto& corridors=corridor_network_.connections();
    for(size_t i=0;i<corridors.size();++i){
        const auto& connection=corridors[i];
        const bool selected=selection_kind_==SelectionKind::Corridor && selected_corridor_==(int)i;
        const bool link_source=corridor_link_source_==(int)i;
        const Color edge_color=(selected||link_source)?theme_.accent:Color{0.18f,1.00f,0.38f,1.0f};
        const float boost=(selected||link_source)?1.55f:1.0f;

        if(!connection.construction.complete()){
            for(const auto* endpoint:{&connection.endpoint_a,&connection.endpoint_b})
                if(endpoint->kind==CorridorEndpointKind::Habitat || endpoint->kind==CorridorEndpointKind::Resource)
                    draw_sliding_door(r,endpoint->door,0,connection.construction.progress());
            const float fraction=connection.construction.progress();
            const Color build_main{1.00f,0.68f,0.12f,0.99f};
            const Color build_detail{0.96f,0.56f,0.08f,0.96f};
            const Color ghost{0.10f,0.40f,0.90f,0.46f};
            mesh_wireframe::draw_layer(r,connection.mesh.shell,ghost,1.55f*boost);
            mesh_wireframe::draw_layer(r,connection.mesh.details,ghost,1.30f*boost);
            if(!connection.mesh.edges.empty()) r.draw_lines_3d(connection.mesh.edges,{0.14f,0.54f,1.00f,0.58f},1.70f*boost);
            if(fraction<=0.0001f){
                r.draw_lines_3d(construction_path_lines(connection.path),Color{0.12f,0.58f,1.00f,0.92f},(selected||link_source)?3.3f:2.4f);
            }else{
                const auto shell=construction_sequence::reveal_triangles(connection.mesh.shell,fraction);
                const auto details=construction_sequence::reveal_triangles(connection.mesh.details,fraction);
                const auto edges=construction_sequence::reveal_lines(connection.mesh.edges,fraction);
                                mesh_wireframe::draw_layer(r,details,build_detail,1.80f*boost);
                mesh_wireframe::draw_layer(r,shell,build_main,2.05f*boost);
                if(!edges.empty()) r.draw_lines_3d(edges,build_main,(selected||link_source)?3.45f:2.35f);
            }
            continue;
        }

        const Color shell_color{0.24f,0.96f,0.42f,1.0f};
        const Color detail_color{0.22f,0.92f,0.40f,0.99f};
        // Open corridor endpoints are permanent sealed pressure bulkheads.
        // Exterior access is provided only by a connected Air Lock habitat, so
        // corridor meshes never carve/open a door at an exposed endpoint.
        mesh_wireframe::draw_layer(r,connection.mesh.details,detail_color,2.35f*boost);
        if(habitat_view_mode_==HabitatViewMode::Default || first_person_mode_==FirstPersonMode::Active){
            mesh_wireframe::draw_layer(r,connection.mesh.shell,shell_color,2.65f*boost);
        }else{
            mesh_wireframe::draw_layer(r,connection.mesh.shell,{shell_color.r,shell_color.g,shell_color.b,0.24f},1.25f);
        }
        r.draw_lines_3d(connection.mesh.edges,edge_color,(selected||link_source)?3.65f:2.90f);
    }
}


void GameApp::render_habitats(Renderer& r){
    for(size_t i=0;i<placed_habitats_.size();++i){
        const auto& instance=placed_habitats_[i];
        const auto& mesh=habitat_models_.mesh(instance);
        const bool selected=selection_kind_==SelectionKind::Habitat && selected_habitat_==(int)i;
        std::vector<CorridorDoor> fpv_doors;
        const std::vector<CorridorDoor>* fpv_ptr=nullptr;
        if(instance.construction.complete()){
            fpv_doors=habitat_doors(i);
            fpv_ptr=&fpv_doors;
        }
        render_habitat_mesh(r,mesh,instance.position,instance.rotation_degrees,false,true,selected,instance.construction.progress()*(float)kConstructionSteps,fpv_ptr);
        if(fpv_ptr) for(const auto& door:fpv_doors) draw_sliding_door(r,door,first_person_door_open_fraction(door.wall_position));

        // Exterior stairs/ramps exist only on the dedicated Air Lock and only
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
            mesh_wireframe::draw_layer(r,details,access_color,instance.construction.complete()?2.45f:2.0f);
            if(!edges.empty()){
                const Color ec=selected?theme_.accent:Color{0.08f,0.11f,0.13f,1.0f};
                r.draw_lines_3d(edges,ec,selected?3.35f:(instance.construction.complete()?2.55f:2.25f));
            }
        }
    }
}

} // namespace game
