#include "game/game_app.hpp"
#include "game/world/resource_builder.hpp"
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

void draw_actor(Renderer& r,const CrewMesh& mesh,const AstronautTestActor& actor,const AstronautRole& role,const AstronautRestPose& pose,bool animated_cloth){
    const float phase=actor.stride_phase*kCrewWalkFrames;
    const int frame=(int)phase;
    const float alpha=phase-frame;
    const auto stride=mesh.vertex_count*3;
    const auto* a=mesh.frames.data()+stride*(1+frame%kCrewWalkFrames);
    const auto* b=mesh.frames.data()+stride*(1+(frame+1)%kCrewWalkFrames);
    const auto* idle=mesh.frames.data();
    const float weight=std::clamp(actor.speed/astronaut_walk_speed(actor.suited),0.f,1.f);
    const float blend=weight*weight*(3.f-2.f*weight);
    const float lift=actor.tool_carry_blend;
    const float carry=lift*lift*(3.f-2.f*lift);
    const bool lying=astronaut_resting(actor);
    const bool detailed=r.lunar_scene_active();
    const float yaw=radians(pose.heading_degrees);
    const float c=std::cos(yaw),s=std::sin(yaw);
    const auto world=[&](Vec3 p){return pose.feet+Vec3{p.x*c+p.z*s,p.y,-p.x*s+p.z*c};};
    constexpr float scale=.0001f*geometry::kAstronautScale;
    std::vector<Vec3> vertices(mesh.vertex_count);
    float sole=0.03f;
    for(std::size_t i=0;i<mesh.vertex_count;++i){
        const std::size_t j=i*3;
        const Vec3 rest{idle[j]*scale,idle[j+1]*scale,idle[j+2]*scale};
        const Vec3 gait{std::lerp((float)a[j],(float)b[j],alpha)*scale,
                        std::lerp((float)a[j+1],(float)b[j+1],alpha)*scale,
                        std::lerp((float)a[j+2],(float)b[j+2],alpha)*scale};
        Vec3 delta=(gait-rest)*blend;
        const float leg=std::clamp((.78f-rest.y)*2.8f,0.f,1.f);
        delta.z*=std::lerp(1.f,kAstronautStrideStretch,leg);
        delta.y*=std::lerp(.80f,1.f,leg);
        // Bias the tool hand toward its relaxed position while gripping the
        // case. The mask falls away at the elbow and the side of the torso.
        const float hand=std::clamp((rest.x-.148f)*30.f,0.f,1.f)*
                         std::clamp((.88f-rest.y)*6.f,0.f,1.f)*std::clamp((rest.y-.52f)*18.f,0.f,1.f);
        delta=delta*(1.f-hand*carry);
        Vec3 p=rest+delta+Vec3{.015f,.035f,.038f}*(hand*carry);
        if(detailed && animated_cloth && !actor.suited && !lying){
            const float cloth=std::clamp((p.y-0.38f)*3.0f,0.0f,1.0f)*std::clamp((1.48f-p.y)*4.0f,0.0f,1.0f);
            p.z+=cloth*blend*0.0065f*std::sin(p.y*32.0f+p.x*18.0f+actor.stride_phase*6.2831853f);
        }
        if(lying)p={-p.x,p.z+.28f,p.y};
        else sole=std::min(sole,p.y);
        vertices[i]=p;
    }
    // Keep the lower boot on the deck through interpolated contact frames.
    for(auto& vertex:vertices){
        if(!lying)vertex.y-=sole;
        vertex=world(vertex);
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
        const Color color{
            std::lerp(rgb[0]/255.f,light*role.color[0],tint),
            std::lerp(rgb[1]/255.f,light*role.color[1],tint),
            std::lerp(rgb[2]/255.f,light*role.color[2],tint),1.f};
        if(detailed){
            // The final five coverall material batches form the baked hair cap.
            const bool hair=!actor.suited && (&batch-mesh.batches.data())>=static_cast<std::ptrdiff_t>(mesh.batches.size()-5);
            const bool skin=!actor.suited && !coverall && rgb[0]>110 &&
                int(rgb[0])-rgb[1]>13 && int(rgb[1])-rgb[2]>10;
            const bool cloth=actor.suited?(pale || teal):coverall;
            const auto material=hair?LunarMaterial::Hair:skin?LunarMaterial::Skin:cloth?LunarMaterial::Fabric:LunarMaterial::Paint;
            r.draw_lunar_surface(triangles,material,color);
        }else r.draw_triangles_3d(triangles,color);
    }
    if(!lying && (actor.construction_job || lift>0.f)){
        resources::Builder box;
        const Vec3 side{c,0,-s},forward{s,0,c};
        const Vec3 center=world({.217f,std::lerp(.087f,.548f,carry),.077f});
        box.oriented_box(box.mesh.body,center,side,{0,1,0},forward,{.12f,.174f,.24f});
        box.oriented_box(box.mesh.details,center+Vec3{0,.097f,0},side,{0,1,0},forward,{.045f,.025f,.10f});
        r.draw_lunar_surface(box.mesh.body,LunarMaterial::Paint,{.69f,.44f,.18f,1.f});
        r.draw_lunar_surface(box.mesh.details,LunarMaterial::Fabric,{.16f,.17f,.18f,1.f});
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
        draw_actor(r,actor.suited?kEVA:kCoverall,actor,role,pose,graphics_.enabled(GraphicsFeature::Cloth));
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
