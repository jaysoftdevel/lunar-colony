#pragma once
#include "game/runtime/presentation.hpp"
#include <span>
namespace game::runtime::jsd_terrain {
using Int=int;
float smooth(float a,float b,float x){
    const float t=std::clamp((x-a)/(b-a),0.0f,1.0f);
    return t*t*(3.0f-2.0f*t);
}

std::uint32_t hash(Int x,Int y,Int seed){
    auto h=std::uint32_t(x)*0x9e3779b1u^std::uint32_t(y)*0x85ebca77u^std::uint32_t(seed)*0xc2b2ae3du;
    h^=h>>16;
    h*=0x7feb352du;
    h^=h>>15;
    h*=0x846ca68bu;
    return h^(h>>16);
}

float random(Int x,Int y,Int seed){ return float(hash(x,y,seed)&65535u)/65535.0f; }

float noise(float x,float y,Int seed,Int period=0,Int period_y=0){
    if(!period_y) period_y=period;
    const Int ix=Int(std::floor(x)),iy=Int(std::floor(y));
    const float u=smooth(0,1,x-ix),v=smooth(0,1,y-iy);
    const auto sample=[&](Int dx,Int dy){
        const Int a=ix+dx,b=iy+dy;
        return random(period?a%period:a,period_y?b%period_y:b,seed);
    };
    return std::lerp(std::lerp(sample(0,0),sample(1,0),u),std::lerp(sample(0,1),sample(1,1),u),v);
}

float terrain_height(float x,float z,float half){
    const float edge=std::max(std::abs(x),std::abs(z))-half;
    const float plains=(noise(x*0.013f,z*0.013f,80)-0.52f)*8.0f;
    const float undulation=(noise(x*0.004f,z*0.004f,61)-0.47f)*38.0f;
    const float massif=std::pow(noise(x*0.0016f,z*0.0016f,13),2.8f)*152.0f;
    const float ridges=(1.0f-std::abs(2.0f*noise(x*0.0062f,z*0.0062f,28)-1.0f))*13.0f;
    const float irregularity=(noise(x*0.029f,z*0.029f,81)-0.5f)*3.3f;
    float crater=0.0f;
    constexpr float spacing=125.0f;
    const Int ix=Int(std::floor(x/spacing)),iz=Int(std::floor(z/spacing));
    const float cx=(ix+0.35f+random(ix,iz,52)*0.30f)*spacing;
    const float cz=(iz+0.35f+random(ix,iz,69)*0.30f)*spacing;
    const float radius=16.0f+random(ix,iz,89)*23.0f;
    const float r=std::hypot(x-cx,z-cz)/radius;
    if(r<1.6f){
        const float rim=std::exp(-36.0f*(r-1.0f)*(r-1.0f));
        crater=radius*(0.065f*rim-0.19f*(1.0f-smooth(0.15f,0.96f,r)));
    }
    return (plains+crater+irregularity)*smooth(0,42,edge)
          +(undulation+massif+ridges-18.0f)*smooth(24,320,edge)
          -std::max(0.0f,edge-940.0f)*0.029f;
}

struct TerrainVertex { Vec3 position; Vec3 normal{0,1,0}; float stone{}; };

TerrainVertex terrain_vertex(float x,float z,float half){
    constexpr float step=0.8f;
    const float dx=terrain_height(x+step,z,half)-terrain_height(x-step,z,half);
    const float dz=terrain_height(x,z+step,half)-terrain_height(x,z-step,half);
    return {{x,terrain_height(x,z,half),z},normalize({-dx,2.0f*step,-dz}),0};
}

constexpr float boulder_spacing=7.2f;

void add_boulder_row(std::vector<TerrainVertex>& vertices,float half,int quality,float range,Int z,Int cells){
    for(Int x=-cells;x<=cells;++x){
        const float chance=random(x,z,312);
        if(chance<quality_budget(quality,0.93f,0.85f,0.78f)) continue;
        const float px=(x+0.14f+0.70f*random(x,z,314))*boulder_spacing;
        const float pz=(z+0.14f+0.70f*random(x,z,327))*boulder_spacing;
        const float edge=std::max(std::abs(px),std::abs(pz))-half;
        if(edge<3.4f || edge>range) continue;
        const float radius=0.28f+std::pow(random(x,z,345),3.0f)*2.0f;
        const float angle=random(x,z,353)*6.2831853f;
        const float y=terrain_height(px,pz,half);
        const float aspect=0.65f+0.65f*random(x,z,355);
        const Vec3 center{px,y,pz};
        const Vec3 peak=center+Vec3{radius*0.24f,radius*0.74f,-radius*0.12f};
        constexpr Int sides=6;
        std::array<Vec3,sides> foot{},shoulder{};
        for(Int j=0;j<sides;++j){
            const float a=angle+6.2831853f*j/sides;
            const float r=radius*(0.71f+0.48f*random(x+sides*j,z,366));
            const Vec3 lateral{std::cos(a)*r,0,std::sin(a)*r*aspect};
            foot[j]=center+lateral+Vec3{0,-0.16f*radius,0};
            shoulder[j]=center+lateral*0.78f+Vec3{0,radius*(0.26f+0.21f*random(x,z+j,369)),0};
        }
        const auto triangle=[&](Vec3 a,Vec3 b,Vec3 c){
            const Vec3 normal=normalize(cross(b-a,c-a));
            vertices.insert(vertices.end(),{{a,normal,1},{b,normal,1},{c,normal,1}});
        };
        for(Int j=0;j<sides;++j){
            const Int k=(j+1)%sides;
            triangle(foot[j],shoulder[j],shoulder[k]);
            triangle(foot[j],shoulder[k],foot[k]);
            triangle(shoulder[j],peak,shoulder[k]);
        }
    }
}

void add_boulders(std::vector<TerrainVertex>& vertices,float half,int quality,float range){
    const Int cells=static_cast<Int>(std::ceil((half+range)/boulder_spacing));
    auto& pool=GameJobs::shared();
    if(!pool.background_workers()){
        for(Int z=-cells;z<=cells;++z) add_boulder_row(vertices,half,quality,range,z,cells);
        return;
    }
    std::vector<std::vector<TerrainVertex>> rows(2*cells+1);
    pool.parallel_for_ranges(rows.size(),1,[&](std::size_t first,std::size_t last){
        for(std::size_t row=first;row<last;++row)
            add_boulder_row(rows[row],half,quality,range,static_cast<Int>(row)-cells,cells);
    });
    auto total=vertices.size();
    for(const auto& row:rows) total+=row.size();
    vertices.reserve(total);
    for(const auto& row:rows) vertices.insert(vertices.end(),row.begin(),row.end());
}

std::vector<TerrainVertex> terrain_mesh(float half,int quality,int clutter,int range){
    // The inner quad stays on the build/pick plane. The flat mode extends
    // the plane to the same horizon as the normal terrain.
    const float inner=quality?half:half+1700.0f;
    std::vector<TerrainVertex> vertices{
        {{-inner,0,-inner}},{{inner,0,inner}},{{inner,0,-inner}},
        {{-inner,0,-inner}},{{-inner,0,inner}},{{inner,0,inner}}
    };
    if(!quality) return vertices;
    const Int segments=quality_budget(quality,32,64,96);
    constexpr std::array<float,38> distances{
        0,4,8,12,16,21,26,32,38,45,53,62,72,83,95,108,122,137,153,
        171,191,213,237,263,293,325,361,401,445,494,550,614,686,770,872,1002,1190,1700
    };
    const std::size_t stride=segments+1;
    std::vector<TerrainVertex> samples(4*distances.size()*stride);
    auto& pool=GameJobs::shared();
    pool.parallel_for_ranges(4*distances.size(),1,[&](std::size_t first,std::size_t last){
        for(std::size_t row=first;row<last;++row){
            const auto side=row/distances.size();
            const float radius=half+distances[row%distances.size()];
            auto* current=samples.data()+row*stride;
            for(Int step=0;step<=segments;++step){
                const float p=radius*(2.0f*float(step)/segments-1.0f);
                const float x=side==0?p:side==1?radius:side==2?-p:-radius;
                const float z=side==0?-radius:side==1?p:side==2?radius:-p;
                current[step]=terrain_vertex(x,z,half);
            }
        }
    });
    constexpr std::size_t strips=distances.size()-1;
    const auto floor_size=vertices.size();
    vertices.resize(floor_size+4*strips*segments*6);
    pool.parallel_for_ranges(4*strips,1,[&](std::size_t first,std::size_t last){
        for(std::size_t strip=first;strip<last;++strip){
            const auto* previous=samples.data()+(strip+strip/strips)*stride;
            const auto* current=previous+stride;
            auto* triangle=vertices.data()+floor_size+strip*segments*6;
            for(Int step=0;step<segments;++step){
                const std::array quad{previous[step],current[step+1],current[step],
                                      previous[step],previous[step+1],current[step+1]};
                triangle=std::copy(quad.begin(),quad.end(),triangle);
            }
        }
    });
    if(clutter>0)add_boulders(vertices,half,clutter,170.0f*range/100.0f);
    return vertices;
}


}
