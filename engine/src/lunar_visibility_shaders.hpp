#pragma once

namespace eng::lunar_shaders {

inline constexpr const char* visibility_vertex=R"GLSL(#version 430 core
layout(location=0) in vec3 position;
uniform mat4 viewProjection;
flat out uint vertexIndex;
void main(){
    vertexIndex=uint(gl_VertexID);
    gl_Position=viewProjection*vec4(position,1.0);
}
)GLSL";

inline constexpr const char* visibility_fragment=R"GLSL(#version 430 core
flat in uint vertexIndex;
layout(location=0) out uint visibleTriangle;
uniform bool terrain;
void main(){
    // An unindexed triangle takes three consecutive vertices. The last
    // provoking vertex identifies its starting index in the bound stream.
    visibleTriangle=(vertexIndex-1u)|(terrain?0x80000000u:0u);
}
)GLSL";

inline constexpr const char* visibility_inputs=R"GLSL(#version 430 core
in vec2 uv;
uniform usampler2D visibilityMap;
uniform sampler2D depthMap;
uniform mat4 inverseViewProjection;
uniform vec3 eye;
uniform sampler2DArray surfaceMaps;
layout(std430,binding=3) readonly buffer TerrainVertices { float terrainStream[]; };
layout(std430,binding=4) readonly buffer ObjectVertices { float objectStream[]; };

vec3 vertexPosition(uint index,bool terrain){
    uint offset=index*(terrain?7u:8u);
    if(terrain) return vec3(terrainStream[offset],terrainStream[offset+1u],terrainStream[offset+2u]);
    return vec3(objectStream[offset],objectStream[offset+1u],objectStream[offset+2u]);
}
vec3 terrainNormal(uint index){
    uint o=index*7u+3u;
    return vec3(terrainStream[o],terrainStream[o+1u],terrainStream[o+2u]);
}
vec3 barycentric(vec3 p,vec3 a,vec3 b,vec3 c){
    vec3 ab=b-a,ac=c-a,ap=p-a;
    float aa=dot(ab,ab),bb=dot(ac,ac),abac=dot(ab,ac);
    float x=dot(ap,ab),y=dot(ap,ac);
    float inv=1.0/max(aa*bb-abac*abac,1e-11);
    float u=(bb*x-abac*y)*inv,v=(aa*y-abac*x)*inv;
    vec3 w=max(vec3(1.0-u-v,u,v),0.0);
    return w/max(dot(w,vec3(1.0)),1e-5);
}
)GLSL";

inline constexpr const char* visibility_resolve=R"GLSL(
void main(){
    uint packedId=texture(visibilityMap,uv).r;
    if(packedId==0u){
        baseColor=vec4(0.0);
        normalRoughness=vec4(0.5,1.0,0.5,1.0);
        materialFlags=vec4(0.0);
        return;
    }
    bool terrain=(packedId&0x80000000u)!=0u;
    uint i=(packedId&0x7fffffffu)-1u;
    vec3 a=vertexPosition(i,terrain),b=vertexPosition(i+1u,terrain),c=vertexPosition(i+2u,terrain);
    vec3 p=reconstruct(uv,texture(depthMap,uv).r,inverseViewProjection);
    vec3 n=normalize(cross(b-a,c-a));
    vec4 tint=vec4(1.0);
    float stone=0.0;
    int material=2;
    if(terrain){
        vec3 w=barycentric(p,a,b,c);
        n=normalize(terrainNormal(i)*w.x+terrainNormal(i+1u)*w.y+terrainNormal(i+2u)*w.z);
        stone=terrainStream[i*7u+6u];
    }else{
        uint offset=i*8u+3u;
        tint=vec4(objectStream[offset],objectStream[offset+1u],objectStream[offset+2u],objectStream[offset+3u]);
        material=int(objectStream[offset+4u]+0.5);
        if(dot(n,eye-p)<0.0) n=-n;
    }
    Surface surface=sampleSurface(p,n,stone,tint,material);
    baseColor=vec4(surface.albedo,surface.ao);
    normalRoughness=vec4(surface.n*0.5+0.5,surface.rough);
    materialFlags=vec4(surface.metal,surface.emission,surface.glass,surface.special);
}
)GLSL";

inline constexpr const char* cluster_culling=R"GLSL(
layout(local_size_x=64) in;
struct Cluster { vec4 sphere; uvec4 coarse,detail; };
layout(std430,binding=5) readonly buffer Clusters { Cluster clusters[]; };
layout(std430,binding=6) writeonly buffer DrawCommands { uvec4 commands[]; };
uniform mat4 viewProjection;
uniform vec3 eye;
uniform int clusterCount;

bool outside(vec4 plane,vec4 sphere){
    return dot(plane.xyz,sphere.xyz)+plane.w < -sphere.w*length(plane.xyz);
}
void main(){
    uint id=gl_GlobalInvocationID.x;
    if(id>=uint(clusterCount)) return;
    Cluster cluster=clusters[id];
    mat4 vp=transpose(viewProjection);
    vec4 s=cluster.sphere;
    bool visible=!(outside(vp[3]+vp[0],s) || outside(vp[3]-vp[0],s) ||
        outside(vp[3]+vp[1],s) || outside(vp[3]-vp[1],s) ||
        outside(vp[3]+vp[2],s) || outside(vp[3]-vp[2],s));
    bool detail=option(OPT_streamed_geometry) && cluster.detail.y>0u && distance(eye,s.xyz)<112.0+s.w;
    uvec4 draw=detail?cluster.detail:cluster.coarse;
    commands[id]=uvec4(visible?draw.y:0u,1u,draw.x,0u);
}
)GLSL";

} // namespace eng::lunar_shaders
