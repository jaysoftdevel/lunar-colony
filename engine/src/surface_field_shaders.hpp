#pragma once

namespace eng::surface_shaders {

inline constexpr char hlsl_types[]=R"SHADER(
#define vec2 float2
#define vec3 float3
#define mix lerp
#define fract frac
)SHADER";

inline constexpr char fields[]=R"SHADER(
float surfaceHash(vec2 cell){
    vec3 p=fract(vec3(cell.x,cell.y,cell.x)*vec3(0.1031,0.1030,0.0973));
    p+=dot(p,p.yzx+33.33);
    return fract((p.x+p.y)*p.z);
}
float surfaceNoise(vec2 p){
    vec2 cell=floor(p),f=fract(p);
    vec2 w=f*f*(3.0-2.0*f);
    return mix(mix(surfaceHash(cell),surfaceHash(cell+vec2(1,0)),w.x),
               mix(surfaceHash(cell+vec2(0,1)),surfaceHash(cell+vec2(1,1)),w.x),w.y);
}
float filteredNoise(vec2 p,float footprint){
    return mix(surfaceNoise(p),0.5,smoothstep(0.28,0.95,footprint));
}
vec2 turnSurface(vec2 p){
    return vec2(0.83*p.x-0.56*p.y,0.56*p.x+0.83*p.y);
}
vec3 regolithField(vec2 p,float footprint){
    vec2 q=turnSurface(p);
    float basin=filteredNoise(q*0.023+vec2(41,-73),footprint*0.023);
    float dust=filteredNoise((p+vec2(basin*9.0,-basin*6.0))*0.091,footprint*0.091);
    float mineral=filteredNoise(q*0.43+vec2(-27,18),footprint*0.43);
    return vec3(basin,dust,mineral);
}
float regolithGrain(vec2 p,float footprint){
    vec2 q=turnSurface(p);
    float sand=filteredNoise(q*4.9,footprint*4.9);
    float fine=filteredNoise(q*14.0+vec2(37,61),footprint*14.0);
    float pebbles=smoothstep(0.68,0.91,sand)*smoothstep(0.33,0.73,fine);
    return 0.67*sand+0.33*fine+0.30*pebbles;
}
float coatingField(vec2 p,float footprint,float seed){
    vec2 q=turnSurface(p)+vec2(seed*0.17,seed*0.43);
    float oxidation=filteredNoise(q*0.48,footprint*0.48);
    float wear=filteredNoise((q+vec2(oxidation*1.24,-oxidation))*2.4,footprint*2.4);
    float speckle=filteredNoise(q*13.0+vec2(17,31),footprint*13.0);
    return 0.52*oxidation+0.31*wear+0.17*speckle;
}
)SHADER";

} // namespace eng::surface_shaders
