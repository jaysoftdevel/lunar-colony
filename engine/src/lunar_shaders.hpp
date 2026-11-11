#pragma once

namespace eng::lunar_shaders {

inline constexpr const char* mesh_vertex=R"GLSL(#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec4 tint;
layout(location=2) in float material;
layout(location=3) in vec3 terrainNormal;
layout(location=4) in float rock;
layout(location=5) in vec3 surfaceCoordinate;
uniform mat4 viewProjection;
uniform bool terrain;
out vec3 world;
out vec3 coordinate;
out vec3 worldNormal;
flat out float stone;
flat out vec4 color;
flat out int kind;
void main(){
    world=position;
    coordinate=terrain?position:surfaceCoordinate;
    worldNormal=terrain?terrainNormal:vec3(0.0);
    stone=terrain?rock:0.0;
    color=terrain?vec4(1.0):tint;
    kind=terrain?2:int(material+0.5);
    gl_Position=viewProjection*vec4(position,1.0);
}
)GLSL";

inline constexpr const char* depth_vertex=R"GLSL(#version 330 core
layout(location=0) in vec3 position;
uniform mat4 lightProjection;
void main(){ gl_Position=lightProjection*vec4(position,1.0); }
)GLSL";

inline constexpr const char* depth_fragment=R"GLSL(#version 330 core
void main(){}
)GLSL";

inline constexpr const char* screen_vertex=R"GLSL(#version 330 core
out vec2 uv;
uniform float clipDepth;
void main(){
    uv=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));
    gl_Position=vec4(uv*2.0-1.0,clipDepth,1.0);
}
)GLSL";

inline constexpr const char* sky_fragment=R"GLSL(#version 330 core
in vec2 uv;
out vec4 pixel;
uniform mat4 inverseViewProjection;
uniform vec3 eye;
uniform vec3 sun;
uniform float daylight;
uniform sampler2D depthMap;

float hash(vec3 p){
    p=fract(p*0.1031);
    p+=dot(p,p.yxz+33.33);
    return fract((p.x+p.y)*p.z);
}
float noise(vec3 p){
    vec3 a=floor(p), f=fract(p);
    f=f*f*(3.0-2.0*f);
    vec4 low=vec4(hash(a),hash(a+vec3(1,0,0)),hash(a+vec3(0,1,0)),hash(a+vec3(1,1,0)));
    a.z+=1.0;
    vec4 high=vec4(hash(a),hash(a+vec3(1,0,0)),hash(a+vec3(0,1,0)),hash(a+vec3(1,1,0)));
    vec4 v=mix(low,high,f.z);
    return mix(mix(v.x,v.y,f.x),mix(v.z,v.w,f.x),f.y);
}
void main(){
    if(texture(depthMap,uv).r<1.0) discard;
    vec4 q=inverseViewProjection*vec4(uv*2.0-1.0,0.99,1.0);
    vec3 ray=normalize(q.xyz/q.w-eye);
    vec3 sky=vec3(0.00020,0.00029,0.00047);
    vec3 galactic=normalize(vec3(0.58,0.44,0.68));
    float galacticLatitude=dot(ray,galactic);
    float band=exp(-20.0*galacticLatitude*galacticLatitude);
    sky+=vec3(0.00052,0.00060,0.00083)*band*noise(ray*18.0);
    vec3 p=ray*225.0, c=floor(p);
    float seed=hash(c);
    vec3 starOffset=vec3(hash(c+7.0),hash(c+12.0),hash(c+15.0))*0.46+0.27;
    float starDistance=length(fract(p)-starOffset);
    float star=exp(-47.0*starDistance*starDistance)*step(0.953,seed);
    sky+=mix(vec3(0.71,0.82,1.0),vec3(1.0,0.80,0.58),hash(c+23.0))*star*(0.14-0.065*daylight);

    vec3 earthDirection=normalize(vec3(0.185,0.105,-1.0));
    float alignment=dot(ray,earthDirection);
    const float radius=0.036;
    float miss=length(cross(ray,earthDirection));
    if(alignment>0.0 && miss<radius+0.005){
        vec3 disk=(ray-alignment*earthDirection)/radius;
        float rr=length(disk);
        vec3 n=normalize(disk-earthDirection*sqrt(max(0.0,1.0-rr*rr)));
        vec3 map=n*5.2+vec3(4.9,3.8,2.1);
        float continent=noise(map)*0.69+noise(map*2.3)*0.22+noise(map*5.1)*0.09;
        float land=smoothstep(0.52,0.56,continent);
        float cloud=smoothstep(0.53,0.74,noise(map*2.4+vec3(0,8,0))*0.68+noise(map*6.1)*0.32);
        vec3 globe=mix(vec3(0.025,0.135,0.35),vec3(0.20,0.25,0.12),land);
        globe=mix(globe,vec3(0.73,0.84,0.93),cloud*0.83);
        globe*=0.055+1.30*max(dot(n,sun),0.0);
        globe+=vec3(0.04,0.20,0.39)*pow(min(rr,1.0),7.0)*(0.10+0.64*max(dot(n,sun),0.0));
        float aa=clamp(fwidth(miss),0.0001,0.004);
        sky=mix(sky,globe,1.0-smoothstep(radius-aa,radius+aa,miss));
        sky+=vec3(0.009,0.035,0.070)*exp(-1400.0*max(0.0,miss-radius))*smoothstep(0.85,1.0,rr);
    }
    if(daylight>0.0){
        float angle=length(cross(ray,sun));
        float toward=max(dot(ray,sun),0.0);
        float disc=(1.0-smoothstep(0.0042,0.0057,angle))*step(0.99,toward);
        sky+=vec3(1.0,0.83,0.64)*daylight*(disc*18.0+0.11*exp(-100.0*angle)*toward);
    }
    pixel=vec4(sky,1.0);
}
)GLSL";

inline constexpr const char* earth_sky_fragment=R"GLSL(
in vec2 uv;
out vec4 pixel;
uniform mat4 inverseViewProjection;
uniform vec3 eye;
uniform float time;
float hash(vec3 p){
    p=fract(p*0.1031);
    p+=dot(p,p.yxz+33.33);
    return fract((p.x+p.y)*p.z);
}
float noise(vec3 p){
    vec3 a=floor(p),f=fract(p);
    f=f*f*(3.0-2.0*f);
    float x0=mix(hash(a),hash(a+vec3(1,0,0)),f.x);
    float x1=mix(hash(a+vec3(0,1,0)),hash(a+vec3(1,1,0)),f.x);
    a.z+=1.0;
    float x2=mix(hash(a),hash(a+vec3(1,0,0)),f.x);
    float x3=mix(hash(a+vec3(0,1,0)),hash(a+vec3(1,1,0)),f.x);
    return mix(mix(x0,x1,f.y),mix(x2,x3,f.y),f.z);
}
float density(vec3 p){
    float layer=smoothstep(212.0,295.0,p.y)*(1.0-smoothstep(315.0,483.0,p.y));
    p.xz+=vec2(-time*4.1,time*1.7);
    vec3 q=p*vec3(0.0040,0.0072,0.0040);
    float mass=noise(q)*0.65+noise(q*2.12)*0.25+noise(q*4.1)*0.10;
    return max(0.0,mass-0.49)*2.9*layer;
}
void main(){
    vec4 q=inverseViewProjection*vec4(uv*2.0-1.0,0.995,1.0);
    vec3 ray=normalize(q.xyz/q.w-eye);
    vec3 sun=normalize(vec3(-0.42,0.65,-0.50));
    float horizon=exp(-max(ray.y,0.0)*5.0);
    float angle=dot(ray,sun);
    float rayleigh=0.75*(1.0+angle*angle);
    float mie=0.050/pow(max(0.08,1.0+0.74*0.74-2.0*0.74*angle),1.5);
    vec3 linearSky=mix(vec3(0.12,0.31,0.63)*rayleigh,vec3(0.52,0.71,0.80),horizon);
    linearSky+=mie*vec3(0.21,0.15,0.085);
    if(ray.y<0.0) linearSky=mix(vec3(0.26,0.43,0.57),linearSky,exp(ray.y*5.0));

    // Short front-to-back integration through a bounded cumulus layer.
    if(option(OPT_clouds) && ray.y>0.014){
        float start=max(0.0,(212.0-eye.y)/ray.y);
        float stop=min(9400.0,(483.0-eye.y)/ray.y);
        int steps=taps(OPT_clouds,4,8,12);
        float stepLength=max(0.0,stop-start)/float(steps);
        float transmittance=1.0;
        vec3 scattered=vec3(0.0);
        for(int step=0;step<steps;++step){
            vec3 p=eye+ray*(start+(float(step)+0.5)*stepLength);
            float d=density(p);
            if(d<=0.001) continue;
            float sunD=density(p+sun*33.0)+density(p+sun*70.0);
            float alpha=1.0-exp(-d*stepLength*0.016);
            vec3 light=mix(vec3(0.29,0.40,0.57),vec3(0.99,0.94,0.83),exp(-sunD*2.3));
            scattered+=transmittance*alpha*light;
            transmittance*=1.0-alpha;
            if(transmittance<0.025) break;
        }
        linearSky=linearSky*transmittance+scattered;
    }
    pixel=vec4(pow(clamp(linearSky,0.0,1.0),vec3(1.0/2.2)),1.0);
}
)GLSL";

inline constexpr const char* geometry_inputs=R"GLSL(#version 330 core
in vec3 world;
in vec3 coordinate;
in vec3 worldNormal;
flat in float stone;
flat in vec4 color;
flat in int kind;
uniform vec3 eye;
uniform sampler2DArray surfaceMaps;
)GLSL";

inline constexpr const char* scene_inputs=R"GLSL(#version 330 core
in vec2 uv;
out vec4 pixel;
uniform sampler2D depthMap;
uniform sampler2D baseMap;
uniform sampler2D normalMap;
uniform sampler2D materialMap;
uniform mat4 inverseViewProjection;
uniform vec3 eye;
uniform vec2 pixelSize;
uniform float projectionScale;
)GLSL";

inline constexpr const char* lighting_uniforms=R"GLSL(uniform vec3 sun;
uniform float daylight;
uniform mat4 lightCascades[3];
uniform sampler2DArrayShadow shadowMap;
uniform sampler2DArrayShadow shadowPageMap;
uniform sampler2D shadowTable;
uniform vec2 shadowOrigin;
uniform vec3 shadowX,shadowY,shadowSun;
uniform bool useShadowPages;
uniform float shadowPageBlend;
uniform bool useShadowMaps;
uniform int shadowQuality;
uniform int lightCount;
uniform vec4 localPositionRadius[16];
uniform vec4 localColorPower[16];

)GLSL";

inline constexpr const char* surface_common=R"GLSL(
uniform bool useGroundPages;
uniform sampler2DArray groundPageMap;
uniform sampler2D groundTable;
uniform vec2 groundOrigin;
vec4 groundDetail(vec2 world,vec2 worldDx,vec2 worldDy){
    if(!useGroundPages) return vec4(0.5,0.86,0.5,0.5);
    vec2 page=world/32.0;
    ivec2 at=ivec2(floor(page)-groundOrigin);
    if(any(lessThan(at,ivec2(0))) || any(greaterThanEqual(at,ivec2(5)))) return vec4(0.5,0.86,0.5,0.5);
    vec2 entry=texelFetch(groundTable,at,0).rg;
    int layer=int(entry.r*255.0+0.5)-1;
    if(layer<0) return vec4(0.5,0.86,0.5,0.5);
    vec2 uv=(fract(page)*96.0+2.0)/100.0;
    float lodScale=quality(OPT_virtual_textures)==1?1.92:quality(OPT_virtual_textures)==2?1.32:0.96;
    vec2 dx=worldDx*(lodScale*0.03),dy=worldDy*(lodScale*0.03);
    vec4 fine=textureGrad(groundPageMap,vec3(uv,float(layer)),dx,dy);
    float distanceFade=1.0-smoothstep(0.3,1.7,max(length(worldDx),length(worldDy)));
    return mix(vec4(0.5,0.86,0.5,0.5),fine,distanceFade*entry.g);
}
vec4 individualTile(vec2 uv,vec2 dx,vec2 dy,vec2 cell,float material){
    vec2 seed=cell+material*vec2(17.7,31.9);
    float key=surfaceHash(seed);
    vec2 phase=vec2(surfaceHash(seed+vec2(39,21)),surfaceHash(seed-vec2(13,57)));
    vec2 signUV=vec2(key<0.5?-1.0:1.0,phase.y<0.5?-1.0:1.0);
    float layer=material*4.0+floor(key*3.9999);
    vec4 tile=textureGrad(surfaceMaps,vec3((uv-cell)*signUV+phase,layer),dx*signUV,dy*signUV);
    tile.r*=0.90+0.20*phase.x;
    tile.ba=0.5+(tile.ba-0.5)*signUV;
    return tile;
}
vec4 tileField(vec2 uv,vec2 dx,vec2 dy,float material){
    // The same seeded tile contributes on both sides of a lattice edge.
    // Pass the continuous gradients explicitly so phase changes never affect mip selection.
    vec2 cell=floor(uv),w=smoothstep(vec2(0.0),vec2(1.0),fract(uv));
    vec4 a=individualTile(uv,dx,dy,cell,material);
    vec4 b=individualTile(uv,dx,dy,cell+vec2(1,0),material);
    vec4 c=individualTile(uv,dx,dy,cell+vec2(0,1),material);
    vec4 d=individualTile(uv,dx,dy,cell+vec2(1,1),material);
    return mix(mix(a,b,w.x),mix(c,d,w.x),w.y);
}
// xy stores the height gradient and z the dust accumulated in the depression.
vec3 impact(vec2 p,float spacing,float seed,float footprint){
    vec2 cell=floor(p/spacing);
    vec2 center=0.5+0.25*(vec2(surfaceHash(cell+seed),surfaceHash(cell-seed))-0.5);
    vec2 v=fract(p/spacing)-center;
    float yaw=6.2831853*surfaceHash(cell+seed*3.29);
    mat2 basis=mat2(cos(yaw),sin(yaw),-sin(yaw),cos(yaw));
    vec2 shape=vec2(1.0,mix(1.0,1.46,surfaceHash(cell+seed*0.24)));
    vec2 stretched=basis*v*shape;
    float radius=mix(0.077,0.243,surfaceHash(cell+2.73*seed));
    float strength=smoothstep(0.28,0.78,surfaceHash(cell+0.71*seed));
    float r=length(stretched)/radius;
    float bowl=1.0-smoothstep(0.10,1.0,r);
    float wall=exp(-38.0*(r-1.02)*(r-1.02));
    float shelf=exp(-11.0*(r-1.17)*(r-1.17));
    float t=clamp((r-0.10)/0.90,0.0,1.0);
    float slope=0.045*6.0*t*(1.0-t)/0.90
               -0.012*76.0*(r-1.02)*wall-0.005*22.0*(r-1.17)*shelf;
    float detailWeight=1.0-smoothstep(radius*spacing*0.10,radius*spacing*0.45,footprint);
    vec2 gradient=transpose(basis)*(stretched*shape/max(length(stretched),0.001));
    return vec3(gradient*slope/radius,0.28*bowl-0.12*wall)*strength*detailWeight;
}

struct Surface {
    vec3 albedo;
    vec3 n;
    float rough;
    float metal;
    float ao;
    float glass;
    float emission;
    float special;
};
Surface sampleSurface(vec3 world,vec3 tex,vec3 face,vec3 worldDx,vec3 worldDy,vec3 texDx,vec3 texDy,
                      float stone,vec4 color,int kind){
    vec3 tint=pow(max(color.rgb,vec3(0.0)),vec3(2.2));
    float footprint=max(length(worldDx),length(worldDy));
    int materialQuality=quality(OPT_material_detail);
    if(kind==7) return Surface(tint,vec3(0,1,0),1.0,0.0,1.0,0.0,1.0,0.0);
    if((kind==10 && !option(OPT_hair)) || (kind==5 && !option(OPT_cloth)) ||
       (kind==8 && !option(OPT_skin))) kind=0;
    if(kind==10){
        if(quality(OPT_hair)==1) return Surface(tint,face,0.47,0.0,1.0,0.0,0.0,0.0);
        if(quality(OPT_hair)==2) return Surface(tint,face,0.47,0.0,1.0,0.0,0.0,0.25);
        float thread=filteredNoise(vec2(tex.x*126.0+tex.z*141.0,tex.y*15.0),footprint*180.0);
        return Surface(tint*(0.89+0.22*thread),face,0.49,0.0,1.0,0.0,0.0,0.25);
    }
    if(kind==9){
        int detail=quality(OPT_decals);
        float wear=detail>1?filteredNoise(turnSurface(world.xz)*3.5+21.0,footprint*3.5):0.0;
        vec3 worn=mix(tint,vec3(0.16,0.149,0.131),0.53*(float(detail)/3.0)*smoothstep(0.54,0.85,wear));
        return Surface(worn,vec3(0,1,0),0.88,0.0,1.0,0.0,0.0,0.0);
    }
    vec3 n=face;
    vec3 albedo=tint;
    float roughness=0.65, metal=0.0, materialAO=1.0;
    if(kind==2){
        vec3 field=regolithField(world.xz,footprint);
        vec4 grain=materialQuality>0?tileField(world.xz*0.32,worldDx.xz*0.32,worldDy.xz*0.32,2.0):vec4(0.5);
        float grit=materialQuality>=2?regolithGrain(world.xz,footprint):0.5;
        int relief=quality(OPT_terrain_relief);
        vec3 dents=relief>0?impact(world.xz,31.0,14.17,footprint):vec3(0.0);
        if(relief>=3){
            vec3 small=impact(turnSurface(world.xz)+8.2,8.5,31.81,footprint);
            dents+=0.62*vec3(0.83*small.x+0.56*small.y,-0.56*small.x+0.83*small.y,small.z);
        }
        dents*=1.0-stone;
        vec4 streamed=groundDetail(world.xz,worldDx.xz,worldDy.xz);
        vec2 slope=dents.xy+(grain.ba-0.5)*0.18+(streamed.ba-0.5)*0.13;
        n=normalize(face+vec3(-slope.x,0.0,-slope.y));
        vec3 soil=mix(vec3(0.115,0.106,0.099),vec3(0.227,0.211,0.190),0.64*field.x+0.36*field.y);
        albedo=soil*(0.66+0.22*field.z+0.20*grit+0.35*grain.r)*(1.0-dents.z*0.74);
        albedo=mix(albedo,albedo*vec3(0.65,0.63,0.61)*(0.74+0.37*grain.r),stone);
        albedo*=0.92+0.16*streamed.r;
        roughness=clamp(0.82+0.12*field.y+(streamed.g-0.86)*0.22,0.80,0.96);
        materialAO=clamp(1.0-dents.z*0.68,0.65,1.0);
    }else if(materialQuality>0){
        if(kind==5) materialQuality=min(materialQuality,quality(OPT_cloth));
        float scale=kind==3?0.59:kind==5?3.2:kind==6?0.22:0.42;
        vec3 p=tex*scale,dx=texDx*scale,dy=texDy*scale;
        vec3 localFace=cross(texDx,texDy);
        localFace=dot(localFace,localFace)>1e-18?normalize(localFace):face;
        vec3 weights=pow(abs(localFace),vec3(4.0));
        weights/=max(0.0001,weights.x+weights.y+weights.z);
        float layer=kind==8?5.0:float(kind);
        vec4 map;
        if(materialQuality==1){
            if(weights.y>max(weights.x,weights.z)) map=tileField(p.xz,dx.xz,dy.xz,layer);
            else if(weights.x>weights.z) map=tileField(p.zy,dx.zy,dy.zy,layer);
            else map=tileField(p.xy,dx.xy,dy.xy,layer);
        }else{
            vec4 tx=weights.x>0.002?tileField(p.zy,dx.zy,dy.zy,layer):vec4(0.5);
            vec4 ty=weights.y>0.002?tileField(p.xz,dx.xz,dy.xz,layer):vec4(0.5);
            vec4 tz=weights.z>0.002?tileField(p.xy,dx.xy,dy.xy,layer):vec4(0.5);
            map=tx*weights.x+ty*weights.y+tz*weights.z;
            if(materialQuality>=3){
                vec3 slope=vec3(0,tx.a-0.5,tx.b-0.5)*weights.x
                          +vec3(ty.b-0.5,0,ty.a-0.5)*weights.y
                          +vec3(tz.b-0.5,tz.a-0.5,0)*weights.z;
                vec3 cotangentX=cross(worldDy,face),cotangentY=cross(face,worldDx);
                float area=dot(worldDx,cotangentX);
                if(abs(area)>1e-10){
                    vec3 gradient=(dot(slope,texDx)*cotangentX+dot(slope,texDy)*cotangentY)/area;
                    n=normalize(face-gradient*(kind==8?0.012:kind==3?0.042:0.12));
                }
            }
        }
        float age=coatingField(tex.xz+tex.y*vec2(0.42,0.73),footprint,float(kind)*41.0);
        albedo*=(0.73+0.48*map.r)*(0.87+0.24*age);
        roughness=clamp(map.g,0.18,0.97);
        if(kind==8){
            albedo=tint*(0.91+0.15*map.r);
            roughness=0.51;
        }
    }
    metal=kind==1?0.61:kind==3?0.31:0.0;
    if(kind==1 || kind==3 || kind==4){
        // Keep microscopic highlights inside a resolvable screen footprint.
        float minRoughness=kind==4?0.24:0.34;
        roughness=max(roughness,minRoughness+0.10*smoothstep(0.025,0.22,footprint));
    }
    if(quality(OPT_decals)>1 && (kind==0 || kind==6)){
        float dust=regolithField(world.xz,footprint).y;
        float deposit=(1.0-smoothstep(0.15,1.15,world.y))*smoothstep(0.42,0.75,dust)
                     *(quality(OPT_decals)==2?0.55:1.0);
        albedo=mix(albedo,vec3(0.19,0.173,0.155),0.30*deposit);
        roughness=mix(roughness,0.90,deposit*0.60);
    }

    float special=kind==5?0.5:kind==8?1.0:0.0;
    return Surface(albedo,n,roughness,metal,materialAO,kind==4?1.0:0.0,0.0,special);
}
)GLSL";

inline constexpr const char* geometry_sample=R"GLSL(
Surface getSurface(){
    bool authored=dot(worldNormal,worldNormal)>0.01;
    vec3 n=authored?normalize(worldNormal):normalize(cross(dFdx(world),dFdy(world)));
    if(!authored && dot(n,eye-world)<0.0) n=-n;
    return sampleSurface(world,coordinate,n,dFdx(world),dFdy(world),dFdx(coordinate),dFdy(coordinate),stone,color,kind);
}
)GLSL";

inline constexpr const char* lighting_common=R"GLSL(const float pi=3.14159265;
vec3 sunColor(){ return mix(vec3(1.0,0.79,0.60),vec3(1.0,0.96,0.89),smoothstep(0.02,0.5,sun.y)); }
vec3 environment(vec3 d){
    return mix(vec3(0.085,0.071,0.061),vec3(0.018,0.025,0.047),smoothstep(-0.45,0.55,d.y))
        *(0.4+0.6*daylight);
}
float filteredShadow(sampler2DArrayShadow map,vec3 uv,int layer,float filterRadius,float bias){
    vec2 texel=filterRadius/vec2(textureSize(map,0).xy);
    float lit=0.0,total=0.0;
    for(int y=-1;y<=1;++y) for(int x=-1;x<=1;++x){
        if(shadowQuality<3 && abs(x)+abs(y)>=shadowQuality) continue;
        float weight=float((2-abs(x))*(2-abs(y)));
        lit+=weight*texture(map,vec4(uv.xy+vec2(x,y)*texel,float(layer),uv.z-bias));
        total+=weight;
    }
    return lit/max(total,1.0);
}

vec2 pageVisibility(vec3 n,vec3 p){
    if(!useShadowPages || shadowPageBlend<0.001) return vec2(1,0);
    p+=n*0.057;
    vec2 cell=vec2(dot(p,shadowX),dot(p,shadowY))/19.0;
    ivec2 tile=ivec2(floor(cell)-shadowOrigin);
    if(any(lessThan(tile,ivec2(0))) || any(greaterThanEqual(tile,ivec2(5)))) return vec2(1,0);
    vec2 entry=texelFetch(shadowTable,tile,0).rg;
    int layer=int(entry.r*255.0+0.5)-1;
    if(layer<0) return vec2(1,0);
    vec2 uv=fract(cell);
    float z=(2047.0-dot(p,shadowSun))/4095.0;
    float edge=min(min(uv.x,uv.y),min(1.0-uv.x,1.0-uv.y));
    float weight=shadowPageBlend*entry.g*smoothstep(0.012,0.085,edge);
    return vec2(filteredShadow(shadowPageMap,vec3(uv,z),layer,1.85,0.000025),weight);
}
float visibility(vec3 n,vec3 p){
    if(!useShadowMaps || daylight<0.001) return 1.0;
    float facing=clamp(dot(n,sun),0.0,1.0);
    vec2 page=pageVisibility(n,p);
    float value=page.x*page.y,weight=1.0-page.y;
    if(weight<0.001) return value;
    for(int layer=0;layer<3;++layer){
        float bias=(0.040+float(layer)*0.060)*(2.0-facing);
        vec3 uv=(lightCascades[layer]*vec4(p+n*bias,1.0)).xyz*0.5+0.5;
        float edge=max(abs(uv.x-0.5),abs(uv.y-0.5));
        if(edge>=0.493 || uv.z<0.0 || uv.z>1.0) continue;
        float blend=1.0-smoothstep(0.417,0.493,edge);
        value+=weight*blend*filteredShadow(shadowMap,uv,layer,1.55,0.00013);
        weight*=1.0-blend;
        if(weight<0.001) return value;
    }
    return value+weight;
}
vec3 brdf(vec3 albedo,float roughness,float metal,vec3 n,vec3 v,vec3 l,vec3 radiance){
    if(!option(OPT_pbr)) return albedo*radiance*(max(dot(n,l),0.0)/pi);
    vec3 h=normalize(v+l+vec3(0.00001));
    float nv=max(dot(n,v),0.001), nl=max(dot(n,l),0.0);
    float nh=max(dot(n,h),0.0), vh=max(dot(v,h),0.0);
    float a=roughness*roughness, a2=a*a;
    float d=nh*nh*(a2-1.0)+1.0;
    float distribution=a2/max(pi*d*d,0.0001);
    float k=(roughness+1.0)*(roughness+1.0)*0.125;
    float g=nv/(nv*(1.0-k)+k)*nl/(nl*(1.0-k)+k);
    vec3 f0=mix(vec3(0.04),albedo,metal);
    vec3 fresnel=f0+(1.0-f0)*pow(1.0-vh,5.0);
    vec3 diffuse=(1.0-metal)*(1.0-fresnel)*albedo/pi;
    return (diffuse+distribution*g*fresnel/max(4.0*nv*nl,0.001))*radiance*nl;
}

vec3 shade(vec3 p,vec3 n,vec3 albedo,float rough,float metal,float ao,float glass,float special,float rayShadow,float fixtureWeight){
    vec3 view=normalize(eye-p);
    float shadow=min(visibility(n,p),mix(1.0,rayShadow,0.67));
    vec3 direct=brdf(albedo,rough,metal,n,view,sun,sunColor()*daylight*5.9)*shadow;
    vec3 bounce=mix(vec3(0.075,0.064,0.052),vec3(0.12,0.15,0.21),n.y*0.5+0.5);
    vec3 earthlight=vec3(0.10,0.15,0.24)*(1.0-daylight)*(0.42+0.58*max(n.y,0.0));
    vec3 diffuse=albedo*(bounce+earthlight)*ao;
    vec3 reflection=reflect(-view,n);
    float edge=pow(1.0-max(dot(n,view),0.0),4.0);
    vec3 reflected=environment(reflection)*(mix(vec3(0.045),albedo,metal)+edge*0.30);
    reflected+=vec3(0.075,0.145,0.225)*(0.25+edge*1.4)*glass;
    diffuse*=1.0-0.52*glass;
    // Fabric fuzz and a cheap diffuse wrap for exposed skin at small screen sizes.
    if(special>0.11 && special<0.36){
        vec3 strand=normalize(vec3(0,1,0)-n*n.y+vec3(0.0001,0,0));
        vec3 halfLight=normalize(sun+view+vec3(0.0001));
        float alignment=dot(strand,halfLight);
        float sheen=pow(max(0.0,1.0-alignment*alignment),18.0)*max(0.0,dot(n,sun));
        reflected+=mix(albedo,vec3(0.21,0.14,0.092),0.42)*sheen*0.41*daylight*shadow;
    }
    if(special>0.36 && special<0.75) reflected+=albedo*edge*(0.060+0.075*daylight);
    if(special>0.75){
        float back=pow(clamp(0.32+0.68*dot(n,sun),0.0,1.0),2.0);
        diffuse+=albedo*vec3(0.22,0.095,0.056)*back*daylight*shadow;
    }
    for(int i=0;i<(fixtureWeight>0.001?lightCount:0);++i){
        vec3 delta=localPositionRadius[i].xyz-p;
        float distance2=dot(delta,delta);
        float radius=localPositionRadius[i].w;
        if(distance2>=radius*radius) continue;
        vec3 l=delta*inversesqrt(max(distance2,0.0001));
        float falloff=1.0-distance2/(radius*radius);
        vec3 intensity=localColorPower[i].rgb*localColorPower[i].w*(falloff*falloff/max(0.7,distance2));
        direct+=brdf(albedo,rough,metal,n,view,l,intensity)*fixtureWeight;
    }
    return diffuse+direct+reflected;
}
)GLSL";

inline constexpr const char* reconstruct_common=R"GLSL(vec3 reconstruct(vec2 at,float depth,mat4 inverseProjection){
    vec4 p=inverseProjection*vec4(at*2.0-1.0,depth*2.0-1.0,1.0);
    return p.xyz/p.w;
}
)GLSL";

inline constexpr const char* gbuffer_outputs=R"GLSL(layout(location=0) out vec4 baseColor;
layout(location=1) out vec4 normalRoughness;
layout(location=2) out vec4 materialFlags;
)GLSL";

inline constexpr const char* gbuffer_fragment=R"GLSL(void main(){
    Surface m=getSurface();
    baseColor=vec4(m.albedo,m.ao);
    normalRoughness=vec4(m.n*0.5+0.5,m.rough);
    materialFlags=vec4(m.metal,m.emission,m.glass,m.special);
}
)GLSL";

inline constexpr const char* deferred_fragment=R"GLSL(
uniform bool useRays;
uniform float rayBlend;
uniform sampler2D rayIndirect;
uniform sampler2D rayDirect;
vec4 screenIndirect(vec3 p,vec3 n,bool contactAO,bool diffuseGI){
    float distanceToEye=length(p-eye);
    if(distanceToEye>265.0) return vec4(0,0,0,1);
    float radius=clamp(distanceToEye*0.025,0.55,2.8);
    float sampleRadius=clamp(projectionScale*radius/max(distanceToEye,0.2),2.0,23.0);
    float ao=0.0;
    vec3 bounce=vec3(0.0);
    int aoTaps=contactAO?taps(OPT_screen_occlusion,4,6,8):0;
    int giTaps=diffuseGI?taps(OPT_screen_indirect,3,5,8):0;
    int sampleCount=max(aoTaps,giTaps);
    for(int i=0;i<sampleCount;++i){
        float angle=float(i)*6.2831853/float(sampleCount)+0.3;
        vec2 offset=vec2(cos(angle),sin(angle))*pixelSize*sampleRadius*(0.59+0.39*float(i%2));
        vec2 at=uv+offset;
        float z=texture(depthMap,at).r;
        if(z>=0.99996) continue;
        vec3 delta=reconstruct(at,z,inverseViewProjection)-p;
        float dist=length(delta);
        vec3 direction=delta/max(dist,0.01);
        float falloff=1.0-smoothstep(0.20*radius,1.25*radius,dist);
        float receiving=max(0.0,dot(n,direction)-0.13);
        if((i+1)*aoTaps/sampleCount!=i*aoTaps/sampleCount) ao+=receiving*falloff;
        if((i+1)*giTaps/sampleCount!=i*giTaps/sampleCount && receiving*falloff>0.025){
            vec3 other=normalize(texture(normalMap,at).rgb*2.0-1.0);
            float emitting=max(dot(other,-direction),0.0);
            float sunLit=max(dot(other,sun),0.0)*daylight;
            vec3 color=texture(baseMap,at).rgb*(0.10+0.73*sunLit);
            bounce+=color*receiving*emitting*falloff;
        }
    }
    return vec4(bounce*(1.92/float(max(1,giTaps))),1.0-min(0.52,ao*(1.04/float(max(1,aoTaps)))));
}
void main(){
    float depth=texture(depthMap,uv).r;
    if(depth>=1.0) discard;
    vec4 base=texture(baseMap,uv);
    vec4 properties=texture(materialMap,uv);
    if(properties.g>0.5){ pixel=vec4(base.rgb*5.0,1.0); return; }
    vec4 nr=texture(normalMap,uv);
    vec3 n=normalize(nr.rgb*2.0-1.0);
    vec3 p=reconstruct(uv,depth,inverseViewProjection);
    bool trace=option(OPT_ray_tracing);
    bool rayGI=trace && option(OPT_ray_indirect);
    bool screenGI=(!rayGI || rayBlend<0.999) && option(OPT_screen_indirect);
    bool screenAO=option(OPT_screen_occlusion);
    vec4 screenBounce=(screenGI || screenAO)?screenIndirect(p,n,screenAO,screenGI):vec4(0,0,0,1);
    float ao=base.a*screenBounce.a;
    vec4 indirect=useRays?texture(rayIndirect,uv):vec4(0,0,0,1);
    indirect.rgb=rayGI?mix(screenBounce.rgb,indirect.rgb,rayBlend):screenBounce.rgb;
    vec4 direct=useRays?texture(rayDirect,uv):vec4(0,0,0,1);
    bool computeFixtures=option(OPT_reservoir_lights) || (trace && option(OPT_ray_shadows));
    float fixtureMix=computeFixtures?rayBlend:0.0;
    vec3 lit=shade(p,n,base.rgb,nr.w,properties.r,ao*mix(1.0,indirect.a,rayBlend),properties.b,properties.a,
        mix(1.0,direct.a,rayBlend),1.0-fixtureMix);
    lit+=direct.rgb*fixtureMix+base.rgb*indirect.rgb*(1.0-properties.r)*ao*0.47;
    pixel=vec4(lit,1.0);
}
)GLSL";

inline constexpr const char* transparency_outputs=R"GLSL(layout(location=0) out vec4 accum;
layout(location=1) out float opticalDepth;
)GLSL";

inline constexpr const char* transparency_fragment=R"GLSL(uniform bool weightedTransparency;
void main(){
    Surface m=getSurface();
    vec3 lit=m.emission>0.0?m.albedo*5.0:shade(world,m.n,m.albedo,m.rough,m.metal,m.ao,m.glass,m.special,1.0,1.0);
    float a=clamp(color.a,0.0,0.995);
    if(!weightedTransparency){
        accum=vec4(lit,a);
        opticalDepth=0.0;
        return;
    }
    float distanceToEye=length(eye-world);
    float weight=clamp((0.12+8.0*a)/(0.4+0.032*distanceToEye),0.05,24.0);
    accum=vec4(lit*a*weight,a*weight);
    opticalDepth=-log(max(1.0-a,0.0001));
}
)GLSL";

inline constexpr const char* line_fragment=R"GLSL(#version 330 core
flat in vec4 color;
out vec4 pixel;
void main(){ pixel=color; }
)GLSL";

inline constexpr const char* reflection_inputs=R"GLSL(uniform sampler2D source;
uniform mat4 viewProjection;
)GLSL";

inline constexpr const char* reflection_fragment=R"GLSL(vec2 project(vec3 p){
    vec4 q=viewProjection*vec4(p,1.0);
    if(q.w<=0.01) return vec2(-1.0);
    return q.xy/q.w*0.5+0.5;
}
void main(){
    pixel=vec4(0.0);
    float depth=texture(depthMap,uv).r;
    if(depth>=0.99995) return;
    vec4 material=texture(materialMap,uv);
    vec4 nr=texture(normalMap,uv);
    float reflectivity=(1.0-smoothstep(0.20,0.54,nr.w))*(0.07+0.34*material.r);
    if(reflectivity<0.028 || material.g>0.5) return;
    vec3 n=normalize(nr.xyz*2.0-1.0);
    vec3 origin=reconstruct(uv,depth,inverseViewProjection);
    vec3 toEye=normalize(eye-origin);
    vec3 ray=reflect(-toEye,n);
    if(dot(ray,n)<0.035) return;

    float before=0.0;
    for(int i=0;i<taps(OPT_screen_reflections,7,12,17);++i){
        float j=float(i)+1.0;
        float stepDistance=0.22+0.12*j*j;
        vec3 p=origin+n*0.095+ray*stepDistance;
        vec2 at=project(p);
        if(min(at.x,at.y)<0.015 || max(at.x,at.y)>0.985) break;
        float sampleDepth=texture(depthMap,at).r;
        if(sampleDepth>=0.99999){ before=stepDistance; continue; }
        vec3 surface=reconstruct(at,sampleDepth,inverseViewProjection);
        float gap=length(p-eye)-length(surface-eye);
        if(gap>0.0 && gap<0.28+0.23*j && distance(origin,surface)>0.55){
            float low=before,high=stepDistance;
            for(int refine=0;refine<taps(OPT_screen_reflections,1,2,3);++refine){
                float middle=(low+high)*0.5;
                vec3 candidate=origin+n*0.095+ray*middle;
                vec2 pos=project(candidate);
                float d=texture(depthMap,pos).r;
                vec3 target=reconstruct(pos,d,inverseViewProjection);
                if(length(candidate-eye)>length(target-eye)) high=middle;
                else low=middle;
            }
            at=project(origin+n*0.095+ray*high);
            float edge=1.0-smoothstep(0.37,0.49,max(abs(at.x-0.5),abs(at.y-0.5)));
            float f=reflectivity*(0.65+0.85*pow(1.0-max(dot(n,toEye),0.0),4.0));
            f*=edge*(1.0-smoothstep(7.0,35.0,high));
            pixel=vec4(texture(source,at).rgb*f,f);
            return;
        }
        before=stepDistance;
    }
}
)GLSL";

inline constexpr const char* temporal_inputs=R"GLSL(#version 330 core
in vec2 uv;
layout(location=0) out vec4 resolved;
layout(location=1) out float currentDepth;
uniform sampler2D source;
uniform sampler2D depthMap;
uniform sampler2D reflectedMap;
uniform sampler2D accumulatedMap;
uniform sampler2D densityMap;
uniform sampler2D historyMap;
uniform sampler2D historyDepth;
uniform mat4 inverseViewProjection;
uniform mat4 previousViewProjection;
uniform mat4 previousInverseViewProjection;
uniform vec3 eye;
uniform vec2 pixelSize;
uniform bool useHistory;
uniform bool useTransparency;
uniform bool weightedTransparency;
uniform bool useReflection;
uniform bool useVolumes;
uniform sampler2D volumeMap;
uniform sampler2D materialMap;
uniform sampler2D normalMap;
)GLSL";

inline constexpr const char* temporal_fragment=R"GLSL(vec3 currentColor(vec2 at,out float coverage){
    vec4 reflection=useReflection?texture(reflectedMap,at):vec4(0.0);
    vec3 c=texture(source,at).rgb*(1.0-reflection.a*0.25)+reflection.rgb;
    coverage=0.0;
    if(useVolumes){
        vec4 v=texture(volumeMap,at);
        c=c*(1.0-v.a)+v.rgb;
        coverage=v.a;
    }
    if(useTransparency){
        vec4 sum=texture(accumulatedMap,at);
        float opacity=weightedTransparency?1.0-exp(-texture(densityMap,at).r):sum.a;
        if(weightedTransparency){
            if(sum.a>0.0001) c=mix(c,sum.rgb/sum.a,opacity);
        }else c=c*(1.0-opacity)+sum.rgb;
        coverage=1.0-(1.0-coverage)*(1.0-opacity);
    }
    return c;
}

vec3 skinDiffusion(vec2 at,vec3 center,float z){
    if(!option(OPT_skin) || z>=0.99997 || texture(materialMap,at).a<0.77) return center;
    vec3 p=reconstruct(at,z,inverseViewProjection);
    vec3 n=normalize(texture(normalMap,at).xyz*2.0-1.0);
    float distanceToEye=distance(p,eye);
    float radius=clamp(9.0/max(distanceToEye,1.0),0.85,3.6);
    vec3 sum=center,weight=vec3(1.0);
    const vec2 taps[8]=vec2[8](vec2(1,0),vec2(-1,0),vec2(0,1),vec2(0,-1),
                               vec2(0.85,0.85),vec2(-0.85,0.85),vec2(0.85,-0.85),vec2(-0.85,-0.85));
    int count=quality(OPT_skin)==1?2:quality(OPT_skin)==2?4:8;
    for(int i=0;i<count;++i){
        vec2 pos=at+taps[i]*radius*pixelSize;
        float otherDepth=texture(depthMap,pos).r;
        if(otherDepth>=0.99997 || texture(materialMap,pos).a<0.77) continue;
        vec3 q=reconstruct(pos,otherDepth,inverseViewProjection);
        vec3 nn=normalize(texture(normalMap,pos).xyz*2.0-1.0);
        float edge=exp(-distance(p,q)*33.0)*smoothstep(0.42,0.90,dot(n,nn));
        vec3 w=edge*(i<4?vec3(0.25,0.13,0.08):vec3(0.13,0.032,0.012));
        sum+=texture(source,pos).rgb*w;
        weight+=w;
    }
    return sum/weight;
}

void main(){
    currentDepth=texture(depthMap,uv).r;
    float coverage;
    vec3 now=skinDiffusion(uv,currentColor(uv,coverage),currentDepth);
    resolved=vec4(now,1.0);
    if(!useHistory || currentDepth>=0.99998) return;

    vec3 world=reconstruct(uv,currentDepth,inverseViewProjection);
    vec4 reprojection=previousViewProjection*vec4(world,1.0);
    if(reprojection.w<=0.05) return;
    vec2 at=reprojection.xy/reprojection.w*0.5+0.5;
    if(min(at.x,at.y)<0.002 || max(at.x,at.y)>0.998) return;
    float depth=texture(historyDepth,at).r;
    if(depth>=0.99998) return;
    vec3 before=reconstruct(at,depth,previousInverseViewProjection);
    float change=distance(before,world);
    float tolerance=0.09+0.0065*distance(eye,world);
    if(change>tolerance) return;

    vec3 lo=now,hi=now;
    for(int i=0;i<4;++i){
        vec2 axis=i<2?vec2(1,0):vec2(0,1);
        vec2 offset=axis*pixelSize*(i%2==0?1.0:-1.0);
        float otherCoverage;
        vec3 nearColor=currentColor(uv+offset,otherCoverage);
        lo=min(lo,nearColor);
        hi=max(hi,nearColor);
    }
    vec3 historic=clamp(texture(historyMap,at).rgb,lo,hi);
    float travel=length((at-uv)/pixelSize);
    float lightingChange=length(historic-now)/(0.18+length(now));
    float reactive=clamp(coverage*1.25+change/tolerance*0.30+lightingChange*0.50,0.0,1.0);
    float blend=mix(0.87,0.35,smoothstep(1.25,20.0,travel))*(1.0-0.86*reactive);
    resolved=vec4(mix(now,historic,blend),1.0);
}
)GLSL";

inline constexpr const char* bloom_fragment=R"GLSL(#version 330 core
in vec2 uv;
out vec4 pixel;
uniform sampler2D source;
uniform vec2 stepUV;
uniform bool extract;
void main(){
    if(extract){
        vec3 light=(texture(source,uv+stepUV*vec2(-1,-1)).rgb+
                    texture(source,uv+stepUV*vec2(-1,1)).rgb+
                    texture(source,uv+stepUV*vec2(1,-1)).rgb+
                    texture(source,uv+stepUV*vec2(1,1)).rgb)*0.25;
        float peak=max(light.r,max(light.g,light.b));
        float knee=clamp((peak-0.70)*0.5,0.0,0.36);
        float weight=max(peak-1.15,3.0*knee*knee)/max(peak,0.0001);
        pixel=vec4(light*weight,1.0);
    }else{
        vec3 light=texture(source,uv).rgb*0.227027;
        light+=(texture(source,uv+stepUV*1.384615).rgb+texture(source,uv-stepUV*1.384615).rgb)*0.316216;
        light+=(texture(source,uv+stepUV*3.230769).rgb+texture(source,uv-stepUV*3.230769).rgb)*0.070270;
        pixel=vec4(light,1.0);
    }
}
)GLSL";

inline constexpr const char* composite_fragment=R"GLSL(
in vec2 uv;
out vec4 pixel;
uniform sampler2D source;
uniform sampler2D glow;
uniform float bloomStrength;
uniform float exposure;
void main(){
    vec3 linear=texture(source,uv).rgb;
    if(bloomStrength>0.0) linear+=texture(glow,uv).rgb*bloomStrength;
    if(option(OPT_vignette)){
        vec2 radial=(uv-0.5)*vec2(1.12,0.82);
        linear*=1.0-(0.20*float(quality(OPT_vignette))/3.0)*smoothstep(0.23,0.70,length(radial));
    }
    linear=max(vec3(0.0),linear*exposure);
    vec3 film=option(OPT_filmic)?clamp((linear*(2.51*linear+0.03))/(linear*(2.43*linear+0.59)+0.14),0.0,1.0)
                                :linear/(1.0+linear);
    pixel=vec4(pow(film,vec3(1.0/2.2)),1.0);
}
)GLSL";

inline constexpr const char* fxaa_fragment=R"GLSL(#version 330 core
in vec2 uv;
out vec4 pixel;
uniform sampler2D source;
uniform vec2 pixelSize;
uniform float spatialAA;
uniform int aaQuality;
uniform float sharpen;
uniform bool learnedUpscale;
uniform vec2 inputPixelSize;
uniform bool useTransparency;
uniform sampler2D overlayMap;
float luma(vec3 color){ return dot(color,vec3(0.299,0.587,0.114)); }
float upscaleResidual(float center,vec4 axial,vec4 diagonal);
vec4 present(vec3 color){
    if(learnedUpscale){
        vec2 dx=vec2(inputPixelSize.x,0),dy=vec2(0,inputPixelSize.y);
        float base=luma(texture(source,uv).rgb);
        vec4 axial=vec4(luma(texture(source,uv-dx).rgb),luma(texture(source,uv+dx).rgb),
                       luma(texture(source,uv-dy).rgb),luma(texture(source,uv+dy).rgb))-base;
        float edge=max(max(abs(axial.x),abs(axial.y)),max(abs(axial.z),abs(axial.w)));
        if(edge>0.021){
            vec4 diagonal=vec4(luma(texture(source,uv-dx-dy).rgb),luma(texture(source,uv+dx-dy).rgb),
                               luma(texture(source,uv-dx+dy).rgb),luma(texture(source,uv+dx+dy).rgb))-base;
            color=clamp(color+upscaleResidual(base,axial,diagonal)*smoothstep(0.021,0.075,edge),0.0,1.0);
        }
    }
    vec4 overlay=useTransparency?texture(overlayMap,uv):vec4(0.0);
    return vec4(color*(1.0-overlay.a)+overlay.rgb,1.0);
}
void main(){
    vec3 center=texture(source,uv).rgb;
    if(spatialAA<=0.0){ pixel=present(center); return; }
    vec3 left=texture(source,uv-vec2(pixelSize.x,0)).rgb;
    vec3 right=texture(source,uv+vec2(pixelSize.x,0)).rgb;
    vec3 down=texture(source,uv-vec2(0,pixelSize.y)).rgb;
    vec3 up=texture(source,uv+vec2(0,pixelSize.y)).rgb;
    vec3 lowColor=min(center,min(min(left,right),min(down,up)));
    vec3 highColor=max(center,max(max(left,right),max(down,up)));
    center=clamp(center+(center-(left+right+down+up)*0.25)*sharpen,lowColor,highColor);
    if(aaQuality==1){
        float contrast=luma(highColor)-luma(lowColor);
        float weight=smoothstep(0.04,0.18,contrast)*0.55*spatialAA;
        pixel=present(mix(center,(left+right+down+up)*0.25,weight));
        return;
    }
    float a=luma(texture(source,uv+vec2(-1,-1)*pixelSize).rgb);
    float b=luma(texture(source,uv+vec2(1,-1)*pixelSize).rgb);
    float c=luma(texture(source,uv+vec2(-1,1)*pixelSize).rgb);
    float d=luma(texture(source,uv+pixelSize).rgb);
    float low=min(luma(center),min(min(a,b),min(c,d)));
    float high=max(luma(center),max(max(a,b),max(c,d)));
    if(high-low<max(0.036,high*0.14)){ pixel=present(center); return; }
    vec2 dir=vec2(-((a+b)-(c+d)),(a+c)-(b+d));
    dir=clamp(dir/(min(abs(dir.x),abs(dir.y))+max(0.004,(a+b+c+d)*0.03125)),-7.0,7.0)*pixelSize;
    vec3 nearColor=(texture(source,uv-dir/6.0).rgb+texture(source,uv+dir/6.0).rgb)*0.5;
    if(aaQuality==2){ pixel=present(mix(center,nearColor,spatialAA)); return; }
    vec3 farColor=nearColor*0.5+(texture(source,uv-dir*0.5).rgb+texture(source,uv+dir*0.5).rgb)*0.25;
    float lum=luma(farColor);
    vec3 filtered=(lum<low || lum>high)?nearColor:farColor;
    pixel=present(mix(center,filtered,spatialAA));
}
)GLSL";

} // namespace eng::lunar_shaders
