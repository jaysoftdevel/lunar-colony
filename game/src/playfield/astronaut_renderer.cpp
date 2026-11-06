#include "game/game_app.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

namespace game {
using namespace eng;

namespace {
struct CrewBatch {
    std::size_t count;
    std::array<std::uint8_t,3> color;
};
struct CrewMesh {
    std::size_t vertex_count;
    std::span<const std::int16_t> frames;
    std::span<const std::uint16_t> triangles;
    std::span<const CrewBatch> batches;
};

#include "assets/astronaut_meshes.inc"

static_assert(kEVA.frames.size()==kEVA.vertex_count*3*(1+kCrewWalkFrames));
static_assert(kCoverall.frames.size()==kCoverall.vertex_count*3*(1+kCrewWalkFrames));

void draw_actor(Renderer& r,const CrewMesh& mesh,const AstronautTestActor& actor,const AstronautRole& role,const AstronautRestPose& pose){
    const float phase=actor.stride_phase*kCrewWalkFrames;
    const int frame=(int)phase;
    const float alpha=phase-frame;
    const auto stride=mesh.vertex_count*3;
    const auto* a=mesh.frames.data()+stride*(1+frame%kCrewWalkFrames);
    const auto* b=mesh.frames.data()+stride*(1+(frame+1)%kCrewWalkFrames);
    const auto* idle=mesh.frames.data();
    const float weight=std::clamp(actor.speed/astronaut_walk_speed(actor.suited),0.f,1.f);
    const float blend=weight*weight*(3.f-2.f*weight);
    const bool lying=astronaut_resting(actor);
    const float yaw=radians(pose.heading_degrees);
    const float c=std::cos(yaw),s=std::sin(yaw);
    std::vector<Vec3> vertices(mesh.vertex_count);
    for(std::size_t i=0;i<mesh.vertex_count;++i){
        const auto sample=[&](std::size_t axis){
            const std::size_t j=i*3+axis;
            return std::lerp((float)idle[j],std::lerp((float)a[j],(float)b[j],alpha),blend)
                   *(.0001f*geometry::kAstronautScale);
        };
        Vec3 p{sample(0),sample(1),sample(2)};
        if(lying)p={-p.x,p.z+.28f,p.y};
        vertices[i]=pose.feet+Vec3{p.x*c+p.z*s,p.y,-p.x*s+p.z*c};
    }
    std::size_t first=0;
    std::vector<Vec3> triangles;
    for(const auto& batch:mesh.batches){
        triangles.clear();
        triangles.reserve(batch.count);
        for(const auto index:mesh.triangles.subspan(first,batch.count))
            triangles.push_back(vertices[index]);
        first+=batch.count;
        const auto& rgb=batch.color;
        const float light=(.2126f*rgb[0]+.7152f*rgb[1]+.0722f*rgb[2])/255.f;
        // Preserve the visor, skin and small fittings. Tint the fabric and
        // its accent panels while retaining each batch's baked shading.
        const bool pale=rgb[0]>120 && std::abs((int)rgb[0]-rgb[2])<48;
        const bool teal=rgb[1]>rgb[0] && rgb[2]>rgb[0];
        const bool coverall=rgb[0]>rgb[1] && rgb[0]<145 && rgb[1]>40 && rgb[2]<100;
        const float tint=actor.suited?(pale?.38f:teal?.90f:0.f):coverall?.86f:0.f;
        r.draw_triangles_3d(triangles,{
            std::lerp(rgb[0]/255.f,light*role.color[0],tint),
            std::lerp(rgb[1]/255.f,light*role.color[1],tint),
            std::lerp(rgb[2]/255.f,light*role.color[2],tint),1.f});
    }
}

void draw_ring(Renderer& r,Vec3 center,float radius,Color color,float width){
    constexpr int segments=32;
    std::vector<Vec3> outline;
    outline.reserve(segments*2);
    const auto point=[&](int i){
        const float angle=6.2831853f*i/segments;
        return center+Vec3{radius*std::cos(angle),0,radius*std::sin(angle)};
    };
    for(int i=0;i<segments;++i){
        outline.push_back(point(i));
        outline.push_back(point(i+1));
    }
    r.draw_lines_3d(outline,color,width);
}
}

void GameApp::render_test_astronauts(Renderer& r) const {
    if(astronaut_cache_.oxygen_kg>0 || astronaut_cache_.food_kg>0 || astronaut_cache_.water_litres>0)
        draw_ring(r,{0,.018f,0},kCrewCacheRadius,{.30f,.68f,.58f,.55f},1.2f);
    for(std::size_t i=0;i<test_astronauts_.size();++i){
        const auto& actor=test_astronauts_[i];
        const auto& role=kAstronautRoles[i];
        const auto pose=astronaut_rest_pose(actor);
        draw_actor(r,actor.suited?kEVA:kCoverall,actor,role,pose);
        const bool selected=(int)i==selected_astronaut_;
        Color color{role.color[0],role.color[1],role.color[2],1.f};
        if(!selected)color.a=.66f;
        draw_ring(r,actor.position+Vec3{0,.022f,0},selected?.51f:.40f,color,selected?2.4f:1.35f);
        if(selected && actor.destination){
            const Vec3 target=*actor.destination+Vec3{0,.026f,0};
            draw_ring(r,target,.33f,color,1.8f);
            r.draw_lines_3d({
                target+Vec3{-.18f,0,0},target+Vec3{.18f,0,0},
                target+Vec3{0,0,-.18f},target+Vec3{0,0,.18f}
            },color,1.65f);
        }
    }
}

} // namespace game
