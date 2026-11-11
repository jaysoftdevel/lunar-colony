#pragma once

namespace eng::lunar_shaders {

inline constexpr const char* ray_inputs=R"GLSL(
layout(local_size_x=8,local_size_y=8) in;
uniform sampler2D depthMap;
uniform sampler2D baseMap;
uniform sampler2D normalMap;
uniform sampler2D materialMap;
uniform sampler2D historyDepth;
uniform mat4 inverseViewProjection;
uniform mat4 previousViewProjection;
uniform mat4 previousInverseViewProjection;
uniform vec3 eye;
uniform vec3 sun;
uniform float daylight;
uniform vec3 rayFocus;
uniform int rayNodeCount;
uniform int lightCount;
uniform int frameIndex;
uniform bool useHistory;

struct Fixture { vec4 positionRadius; vec4 colorPower; };
layout(std430,binding=2) readonly buffer LightBuffer { Fixture fixtures[]; };
const vec3 kLuma=vec3(0.2126,0.7152,0.0722);
const float kPi=3.14159265358979;

uint randomBits(inout uint seed){
    seed=seed*747796405u+2891336453u;
    uint word=((seed>>((seed>>28u)+4u))^seed)*277803737u;
    return (word>>22u)^word;
}
float rand(inout uint seed){ return float(randomBits(seed)>>8u)*(1.0/16777216.0); }
uint worldSeed(vec3 point){
    uvec3 cell=uvec3(ivec3(floor(point*13.0)));
    return cell.x*73856093u^cell.y*19349663u^cell.z*83492791u;
}

vec3 reconstruct(vec2 at,float depth,mat4 inverseProjection);
vec3 unitNormal(vec2 uv){ return normalize(texture(normalMap,uv).xyz*2.0-1.0); }

bool historyCoord(vec3 p,out vec2 at){
    vec4 q=previousViewProjection*vec4(p,1.0);
    if(q.w<0.04) return false;
    at=q.xy/q.w*0.5+0.5;
    if(min(at.x,at.y)<0.002 || max(at.x,at.y)>0.998) return false;
    float depth=texture(historyDepth,at).r;
    if(depth>=0.99997) return false;
    vec3 old=reconstruct(at,depth,previousInverseViewProjection);
    return distance(old,p)<0.16+0.006*distance(eye,p);
}

struct MaterialSample { vec3 p,n,albedo,view; float roughness,metal; };
MaterialSample sampleMaterial(vec2 at,float depth){
    vec4 nr=texture(normalMap,at);
    vec3 p=reconstruct(at,depth,inverseViewProjection);
    return MaterialSample(p,normalize(nr.rgb*2.0-1.0),texture(baseMap,at).rgb,
        normalize(eye-p),nr.a,texture(materialMap,at).r);
}

vec3 fixtureContribution(MaterialSample surface,uint index){
    Fixture light=fixtures[index];
    vec3 delta=light.positionRadius.xyz-surface.p;
    float r2=light.positionRadius.w*light.positionRadius.w;
    float d2=dot(delta,delta);
    if(d2>=r2 || d2<0.001) return vec3(0.0);
    vec3 l=delta*inversesqrt(d2);
    float nl=max(dot(surface.n,l),0.0);
    if(nl<0.0001) return vec3(0.0);
    float falloff=1.0-d2/r2;
    vec3 intensity=light.colorPower.rgb*light.colorPower.w*(falloff*falloff/max(0.7,d2));
    if(!option(OPT_pbr)) return surface.albedo*intensity*(nl/kPi);
    vec3 h=normalize(surface.view+l+vec3(0.00001));
    float nv=max(dot(surface.n,surface.view),0.001);
    float nh=max(dot(surface.n,h),0.0),vh=max(dot(surface.view,h),0.0);
    float a2=pow(surface.roughness,4.0);
    float d=nh*nh*(a2-1.0)+1.0;
    float k=pow(surface.roughness+1.0,2.0)*0.125;
    float g=nv/(nv*(1.0-k)+k)*nl/(nl*(1.0-k)+k);
    vec3 f0=mix(vec3(0.04),surface.albedo,surface.metal);
    vec3 f=f0+(1.0-f0)*pow(1.0-vh,5.0);
    vec3 spec=f*(a2/max(kPi*d*d,0.0001))*g/max(4.0*nv*nl,0.001);
    vec3 diffuse=(1.0-surface.metal)*(1.0-f)*surface.albedo/kPi;
    return (diffuse+spec)*intensity*nl;
}

// Packed reservoir: selected fixture index + 1, sum of RIS weights,
// effective candidate count, selected target luminance at this surface.
void resample(inout vec4 reservoir,uint index,float w,float count,float target,inout uint seed){
    reservoir.z+=count;
    reservoir.y+=w;
    if(w>0.0 && rand(seed)*reservoir.y<w){
        reservoir.x=float(index+1u);
        reservoir.w=target;
    }
}
void mergeReservoir(inout vec4 reservoir,vec4 other,MaterialSample surface,float maxCount,inout uint seed){
    if(other.x<1.0 || other.x>float(lightCount) || other.w<1e-7 || other.z<1.0) return;
    uint index=uint(other.x)-1u;
    float target=dot(fixtureContribution(surface,index),kLuma);
    float count=min(maxCount,other.z);
    float weight=target*(other.y/max(other.w,1e-7))*(count/other.z);
    resample(reservoir,index,weight,count,target,seed);
}
)GLSL";

inline constexpr const char* reservoir_compute=R"GLSL(
layout(rgba32f,binding=0) uniform writeonly image2D reservoirOut;
uniform sampler2D historyMap;
void main(){
    ivec2 pixel=ivec2(gl_GlobalInvocationID.xy);
    ivec2 size=imageSize(reservoirOut);
    if(any(greaterThanEqual(pixel,size))) return;
    vec2 uv=(vec2(pixel)+0.5)/vec2(size);
    float depth=texture(depthMap,uv).r;
    vec4 reservoir=vec4(0.0);
    if(lightCount>16 && depth<0.99997 && texture(materialMap,uv).g<0.5){
        MaterialSample surface=sampleMaterial(uv,depth);
        uint seed=worldSeed(surface.p)+uint(option(OPT_ray_denoise)?frameIndex:0)*524287u;
        for(int candidate=0;candidate<taps(OPT_reservoir_lights,2,4,6);++candidate){
            uint index=randomBits(seed)%uint(lightCount);
            float p=dot(fixtureContribution(surface,index),kLuma);
            resample(reservoir,index,p*float(lightCount),1.0,p,seed);
        }
        vec2 at;
        if(useHistory && historyCoord(surface.p,at))
            mergeReservoir(reservoir,texture(historyMap,at),surface,18.0,seed);
    }
    imageStore(reservoirOut,pixel,reservoir);
}
)GLSL";

inline constexpr const char* ray_queries=R"GLSL(
struct RayTriangle { vec4 a,b,c,tint; };
struct RayNode { vec4 minimum,maximum; uvec4 link; };
layout(std430,binding=0) readonly buffer Nodes { RayNode nodes[]; };
layout(std430,binding=1) readonly buffer Triangles { RayTriangle triangles[]; };
struct RayHit { float t; int index; };

bool intersectsBox(RayNode box,vec3 start,vec3 invDirection,float maxDistance){
    vec3 a=(box.minimum.xyz-start)*invDirection;
    vec3 b=(box.maximum.xyz-start)*invDirection;
    vec3 low=min(a,b),high=max(a,b);
    float near=max(0.0,max(low.x,max(low.y,low.z)));
    float far=min(high.x,min(high.y,high.z));
    return near<=min(far,maxDistance);
}

float intersectsTriangle(RayTriangle triangle,vec3 origin,vec3 direction){
    vec3 e1=triangle.b.xyz-triangle.a.xyz,e2=triangle.c.xyz-triangle.a.xyz;
    vec3 p=cross(direction,e2);
    float determinant=dot(e1,p);
    if(abs(determinant)<1e-8) return -1.0;
    float inverse=1.0/determinant;
    vec3 t=origin-triangle.a.xyz;
    float u=dot(t,p)*inverse;
    if(u<0.0 || u>1.0) return -1.0;
    vec3 q=cross(t,e1);
    float v=dot(direction,q)*inverse;
    if(v<0.0 || u+v>1.0) return -1.0;
    return dot(e2,q)*inverse;
}

RayHit trace(vec3 origin,vec3 direction,float maxDistance,bool anyHit){
    RayHit result=RayHit(maxDistance,-1);
    if(rayNodeCount<=0) return result;
    uint stack[32];
    int count=1;
    stack[0]=0u;
    vec3 safeDir=mix(vec3(-1.0),vec3(1.0),greaterThanEqual(direction,vec3(0.0)))*max(abs(direction),vec3(1e-6));
    vec3 invDirection=1.0/safeDir;
    while(count>0){
        uint index=stack[--count];
        RayNode node=nodes[index];
        if(!intersectsBox(node,origin,invDirection,result.t)) continue;
        if(node.link.z>0u){
            for(uint j=0u;j<node.link.z;++j){
                uint id=node.link.x+j;
                RayTriangle triangle=triangles[id];
                if(anyHit && (triangle.a.w==7.0 || triangle.a.w==9.0)) continue;
                float t=intersectsTriangle(triangle,origin,direction);
                if(t<=0.036 || t>=result.t) continue;
                result=RayHit(t,int(id));
                if(anyHit) return result;
            }
        }else if(count<30){
            bool forward=direction[int(node.link.w)]>0.0;
            stack[count++]=forward?node.link.y:node.link.x;
            stack[count++]=forward?node.link.x:node.link.y;
        }
    }
    return result;
}

vec3 hemisphere(vec3 n,inout uint seed){
    float a=2.0*kPi*rand(seed);
    float r=sqrt(rand(seed));
    vec3 tangent=normalize(cross(abs(n.y)<0.9?vec3(0,1,0):vec3(0,0,1),n));
    return tangent*(r*cos(a))+cross(n,tangent)*(r*sin(a))+n*sqrt(1.0-r*r);
}

vec3 hitRadiance(RayHit hit,vec3 origin,vec3 direction){
    if(hit.index<0){
        if(direction.y<-0.001 && origin.y/(-direction.y)<hit.t)
            return vec3(0.19,0.171,0.153)*(0.14+0.9*daylight*max(sun.y,0.0));
        return mix(vec3(0.017,0.026,0.045),vec3(0.050,0.058,0.077),daylight*0.7);
    }
    RayTriangle triangle=triangles[hit.index];
    vec3 base=pow(max(triangle.tint.rgb,vec3(0.0)),vec3(2.2));
    if(triangle.a.w==7.0) return base*4.6;
    vec3 n=normalize(cross(triangle.b.xyz-triangle.a.xyz,triangle.c.xyz-triangle.a.xyz));
    if(dot(n,direction)>0.0) n=-n;
    vec3 p=origin+direction*hit.t;
    float sunFactor=max(dot(n,sun),0.0)*daylight;
    if(sunFactor>0.02 && trace(p+n*0.085,sun,85.0,true).index>=0) sunFactor=0.0;
    vec3 color=base*(vec3(0.085,0.102,0.139)+vec3(1.54,1.48,1.36)*sunFactor);
    for(int i=0;i<min(lightCount,4);++i){
        vec3 l=fixtures[i].positionRadius.xyz-p;
        float d2=dot(l,l),radius=fixtures[i].positionRadius.w;
        float fade=max(0.0,1.0-d2/(radius*radius));
        if(fade>0.0)
            color+=base*fixtures[i].colorPower.rgb*fixtures[i].colorPower.w*fade*fade*
                max(dot(n,l)*inversesqrt(max(d2,0.01)),0.0)/(kPi*max(d2,0.65));
    }
    return color;
}

float visibleLight(uint index,vec3 origin){
    vec3 toLight=fixtures[index].positionRadius.xyz-origin;
    float range=length(toLight);
    if(range<0.15) return 1.0;
    return trace(origin,toLight/range,range-0.12,true).index<0?1.0:0.0;
}
)GLSL";

inline constexpr const char* ray_compute=R"GLSL(
layout(rgba16f,binding=0) uniform writeonly image2D specOut;
layout(rgba16f,binding=1) uniform writeonly image2D diffuseOut;
layout(rgba16f,binding=2) uniform writeonly image2D directOut;
uniform sampler2D reservoirMap;

void main(){
    ivec2 pixel=ivec2(gl_GlobalInvocationID.xy),size=imageSize(specOut);
    if(any(greaterThanEqual(pixel,size))) return;
    vec2 uv=(vec2(pixel)+0.5)/vec2(size);
    float depth=texture(depthMap,uv).r;
    vec4 spec=vec4(0.0),diffuse=vec4(0,0,0,1),direct=vec4(0,0,0,1);
    if(depth<0.99997 && texture(materialMap,uv).g<0.5){
        MaterialSample surface=sampleMaterial(uv,depth);
        float reach=distance(surface.p.xz,rayFocus.xz);
        uint seed=worldSeed(surface.p)+uint(option(OPT_ray_denoise)?frameIndex:0)*7819u;
        float offset=clamp(0.045+0.00045*distance(surface.p,eye),0.045,0.24);
        vec3 origin=surface.p+surface.n*offset;
        bool traced=option(OPT_ray_tracing);
        bool tracedShadows=traced && option(OPT_ray_shadows);
        if(traced && reach<182.0){
            vec3 randomDirection=hemisphere(surface.n,seed);
            bool indirect=option(OPT_ray_indirect),occlusion=option(OPT_ray_occlusion);
            if(indirect || occlusion){
                float aoReach=float(taps(OPT_ray_occlusion,2,3,4));
                float giReach=float(taps(OPT_ray_indirect,14,24,36));
                RayHit bounce=trace(origin,randomDirection,indirect?giReach:aoReach,false);
                if(indirect) diffuse.rgb=hitRadiance(bounce,origin,randomDirection)*0.54;
                if(occlusion && bounce.index>=0)
                    diffuse.a=mix(0.62,1.0,smoothstep(0.0,aoReach,bounce.t));
            }

            float strength=(1.0-smoothstep(0.20,0.63,surface.roughness))*(0.15+0.85*surface.metal);
            if(option(OPT_ray_reflections) && strength>0.025){
                vec3 reflection=reflect(-surface.view,surface.n);
                vec3 direction=normalize(mix(reflection,randomDirection,surface.roughness*surface.roughness*0.65));
                if(dot(direction,surface.n)>0.01){
                    RayHit reflected=trace(origin,direction,float(taps(OPT_ray_reflections,30,56,88)),false);
                    vec3 f0=mix(vec3(0.04),surface.albedo,surface.metal);
                    vec3 f=f0+(1.0-f0)*pow(1.0-max(dot(surface.n,surface.view),0.0),5.0);
                    spec=vec4(hitRadiance(reflected,origin,direction)*f*strength,strength);
                }
            }

            if(tracedShadows && daylight>0.001 && dot(surface.n,sun)>0.055){
                RayHit blocker=trace(origin,sun,float(taps(OPT_ray_shadows,45,90,150)),true);
                direct.a=blocker.index<0?1.0:smoothstep(140.0,182.0,reach);
            }
        }
        if(!option(OPT_reservoir_lights) || lightCount<=16){
            for(uint i=0u;i<uint(lightCount);++i){
                vec3 value=fixtureContribution(surface,i);
                if(dot(value,kLuma)>0.00004)
                    direct.rgb+=value*(tracedShadows?visibleLight(i,origin):1.0);
            }
        }else{
            vec4 reservoir=texelFetch(reservoirMap,pixel,0);
            const ivec2 offsets[4]=ivec2[4](ivec2(-2,0),ivec2(1,-1),ivec2(0,2),ivec2(2,1));
            for(int k=0;k<taps(OPT_reservoir_lights,1,2,4);++k){
                ivec2 nearPixel=clamp(pixel+offsets[k],ivec2(0),size-1);
                vec2 at=(vec2(nearPixel)+0.5)/vec2(size);
                float z=texture(depthMap,at).r;
                if(z>=0.99997 || dot(surface.n,unitNormal(at))<0.77) continue;
                vec3 q=reconstruct(at,z,inverseViewProjection);
                if(distance(surface.p,q)>0.45+0.014*distance(surface.p,eye)) continue;
                mergeReservoir(reservoir,texelFetch(reservoirMap,nearPixel,0),surface,8.0,seed);
            }
            if(reservoir.x>=1.0 && reservoir.w>1e-7){
                uint id=uint(reservoir.x)-1u;
                float weight=min(12.0,reservoir.y/max(reservoir.z*reservoir.w,1e-6));
                direct.rgb=fixtureContribution(surface,id)*weight*(tracedShadows?visibleLight(id,origin):1.0);
            }
        }
    }
    imageStore(specOut,pixel,clamp(spec,0.0,12.0));
    imageStore(diffuseOut,pixel,clamp(diffuse,0.0,8.0));
    imageStore(directOut,pixel,clamp(direct,0.0,12.0));
}
)GLSL";

inline constexpr const char* filter_ray_inputs=R"GLSL(#version 430 core
in vec2 uv;
layout(location=0) out vec4 specular;
layout(location=1) out vec4 indirect;
layout(location=2) out vec4 direct;
uniform sampler2D raySpecular;
uniform sampler2D rayIndirect;
uniform sampler2D rayDirect;
uniform sampler2D previousSpecular;
uniform sampler2D previousIndirect;
uniform sampler2D previousDirect;
uniform sampler2D depthMap;
uniform sampler2D normalMap;
uniform sampler2D historyDepth;
uniform mat4 inverseViewProjection;
uniform mat4 previousViewProjection;
uniform mat4 previousInverseViewProjection;
uniform vec3 eye;
uniform vec2 pixelSize;
uniform bool useHistory;
)GLSL";

inline constexpr const char* filter_ray_fragment=R"GLSL(
void main(){
    float depth=texture(depthMap,uv).r;
    specular=texture(raySpecular,uv);
    indirect=texture(rayIndirect,uv);
    direct=texture(rayDirect,uv);
    if(depth>=0.99997) return;
    vec3 p=reconstruct(uv,depth,inverseViewProjection);
    vec3 n=normalize(texture(normalMap,uv).xyz*2.0-1.0);
    float threshold=0.24+0.010*distance(p,eye);
    float count=1.0;
    float luminance=dot(indirect.rgb,vec3(0.2126,0.7152,0.0722));
    float variation=0.0;
    const vec2 offsets[6]=vec2[6](vec2(-1,-1),vec2(1,-1),vec2(-1,1),vec2(1,1),vec2(-2,0),vec2(2,0));
    for(int i=0;i<taps(OPT_ray_denoise,2,4,6);++i){
        vec2 at=clamp(uv+offsets[i]*pixelSize,0.0,1.0);
        float z=texture(depthMap,at).r;
        if(z>=0.99997) continue;
        vec3 q=reconstruct(at,z,inverseViewProjection);
        vec3 nn=normalize(texture(normalMap,at).xyz*2.0-1.0);
        float w=pow(max(dot(n,nn),0.0),12.0)*exp(-distance(p,q)/threshold)*0.68;
        specular+=texture(raySpecular,at)*w;
        vec4 bounced=texture(rayIndirect,at);
        indirect+=bounced*w;
        float change=dot(bounced.rgb,vec3(0.2126,0.7152,0.0722))-luminance;
        variation+=change*change*w;
        direct+=texture(rayDirect,at)*w;
        count+=w;
    }
    specular/=count;
    indirect/=count;
    indirect.rgb=neuralPrior(p,n,indirect.rgb);
    direct/=count;
    if(!useHistory) return;
    vec4 before=previousViewProjection*vec4(p,1.0);
    if(before.w<=0.05) return;
    vec2 at=before.xy/before.w*0.5+0.5;
    if(min(at.x,at.y)<0.002 || max(at.x,at.y)>0.998) return;
    float z=texture(historyDepth,at).r;
    if(z>=0.99997) return;
    float error=distance(reconstruct(at,z,previousInverseViewProjection),p);
    if(error>threshold) return;
    float history=0.78*(1.0-smoothstep(threshold*0.22,threshold,error));
    vec4 oldSpec=texture(previousSpecular,at);
    vec4 oldIndirect=texture(previousIndirect,at);
    vec4 oldDirect=texture(previousDirect,at);
    if(option(OPT_learned_denoise)){
        float sigma=sqrt(max(variation/count,0.00003));
        float difference=abs(dot(oldIndirect.rgb-indirect.rgb,vec3(0.2126,0.7152,0.0722)));
        vec4 features=vec4(error/threshold,min(difference/max(0.045,3.0*sigma),1.0),
                           min(sigma/0.29,1.0),1.0-min(count/4.0,1.0));
        float confidence=learnedConfidence(features)*(1.0-smoothstep(threshold*0.52,threshold,error));
        history=mix(history,clamp(confidence,0.03,0.88),0.66);
    }
    specular=mix(specular,clamp(oldSpec,specular-0.19,specular+0.19),history);
    indirect=mix(indirect,clamp(oldIndirect,indirect-0.21,indirect+0.21),history);
    direct=mix(direct,clamp(oldDirect,direct-0.26,direct+0.26),history*0.94);
}
)GLSL";

} // namespace eng::lunar_shaders
