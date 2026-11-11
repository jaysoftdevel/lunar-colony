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
uniform vec2 pixelSize;
uniform sampler2DArray surfaceMaps;
layout(std430,binding=3) readonly buffer TerrainVertices { float terrainStream[]; };
layout(std430,binding=4) readonly buffer ObjectVertices { float objectStream[]; };

vec3 vertexPosition(uint index,bool terrain){
    uint offset=index*(terrain?7u:11u);
    if(terrain) return vec3(terrainStream[offset],terrainStream[offset+1u],terrainStream[offset+2u]);
    return vec3(objectStream[offset],objectStream[offset+1u],objectStream[offset+2u]);
}
vec3 terrainNormal(uint index){
    uint o=index*7u+3u;
    return vec3(terrainStream[o],terrainStream[o+1u],terrainStream[o+2u]);
}
vec3 objectCoordinate(uint index){
    uint o=index*11u+8u;
    return vec3(objectStream[o],objectStream[o+1u],objectStream[o+2u]);
}
vec2 triangleOffset(vec3 ap,vec3 ab,vec3 ac){
    float aa=dot(ab,ab),bb=dot(ac,ac),abac=dot(ab,ac);
    float x=dot(ap,ab),y=dot(ap,ac);
    float inv=1.0/max(aa*bb-abac*abac,1e-11);
    return vec2(bb*x-abac*y,aa*y-abac*x)*inv;
}
vec3 tangentStep(vec3 p,vec3 q,vec3 face){
    vec3 stepWorld=q-p,ray=q-eye;
    float denominator=dot(ray,face);
    if(abs(denominator)>0.001) return stepWorld-ray*(dot(stepWorld,face)/denominator);
    return stepWorld-face*dot(stepWorld,face);
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
    float depth=texture(depthMap,uv).r;
    vec3 p=reconstruct(uv,depth,inverseViewProjection);
    vec3 n=normalize(cross(b-a,c-a));
    // Recover gradients on this triangle, even beside a silhouette or depth break.
    vec3 dx=tangentStep(p,reconstruct(uv+vec2(pixelSize.x,0),depth,inverseViewProjection),n);
    vec3 dy=tangentStep(p,reconstruct(uv+vec2(0,pixelSize.y),depth,inverseViewProjection),n);
    vec3 coord=p,tx=dx,ty=dy;
    vec2 at=triangleOffset(p-a,b-a,c-a);
    vec4 tint=vec4(1.0);
    float stone=0.0;
    int material=2;
    if(terrain){
        vec3 w=vec3(1.0-at.x-at.y,at.x,at.y);
        n=normalize(terrainNormal(i)*w.x+terrainNormal(i+1u)*w.y+terrainNormal(i+2u)*w.z);
        stone=terrainStream[i*7u+6u];
    }else{
        uint offset=i*11u+3u;
        tint=vec4(objectStream[offset],objectStream[offset+1u],objectStream[offset+2u],objectStream[offset+3u]);
        material=int(objectStream[offset+4u]+0.5);
        if(dot(n,eye-p)<0.0) n=-n;
        vec3 qa=objectCoordinate(i),qb=objectCoordinate(i+1u)-qa,qc=objectCoordinate(i+2u)-qa;
        vec2 u=triangleOffset(dx,b-a,c-a),v=triangleOffset(dy,b-a,c-a);
        coord=qa+qb*at.x+qc*at.y;
        tx=qb*u.x+qc*u.y;
        ty=qb*v.x+qc*v.y;
    }
    Surface surface=sampleSurface(p,coord,n,dx,dy,tx,ty,stone,tint,material);
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
