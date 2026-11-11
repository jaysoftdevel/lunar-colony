#include "lunar_renderer.hpp"
#include "lunar_shaders.hpp"
#include "lunar_ray_shaders.hpp"
#include "lunar_bvh.hpp"
#include "lunar_geometry.hpp"
#include "lunar_visibility_shaders.hpp"
#include "lunar_volume_shaders.hpp"
#include "lunar_pages.hpp"
#include "lunar_light_fader.hpp"
#include "surface_field_shaders.hpp"
#include "lunar_neural_shaders.hpp"
#include "lunar_learned_filter.hpp"
#include "lunar_learned_upscale.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <iostream>
#include <initializer_list>
#include <string_view>
#include <vector>

namespace eng {
namespace {

using UInt=unsigned int;
using Int=int;
using Enum=unsigned int;
using Boolean=unsigned char;
using Size=std::ptrdiff_t;

constexpr Enum vertex_shader=0x8B31, fragment_shader=0x8B30, compute_shader=0x91B9;
constexpr Enum compile_status=0x8B81, link_status=0x8B82, major_version=0x821B, minor_version=0x821C;
constexpr Enum array_buffer=0x8892, stream_draw=0x88E0, static_draw=0x88E4;
constexpr Enum float_type=0x1406, byte_type=0x1401, int_type=0x1405;
constexpr Enum texture_2d=0x0DE1, texture_array=0x8C1A, texture0=0x84C0;
constexpr Enum min_filter=0x2801, mag_filter=0x2800, wrap_s=0x2802, wrap_t=0x2803;
constexpr Enum linear=0x2601, nearest=0x2600, trilinear=0x2703, repeat=0x2901, clamp_edge=0x812F;
constexpr Enum rgba=0x1908, rgba8=0x8058, rgba16f=0x881A, rgba32f=0x8814;
constexpr Enum depth_component=0x1902, depth24=0x81A6;
constexpr Enum storage_buffer=0x90D2, write_only=0x88B9, image_barrier=0x0020, fetch_barrier=0x0008;
constexpr Enum red=0x1903, r16f=0x822D, r32f=0x822E, r32ui=0x8236, red_integer=0x8D94;
constexpr Enum draw_indirect_buffer=0x8F3F, command_barrier=0x0040, storage_barrier=0x2000;
constexpr Enum color_buffer=0x1800, last_vertex_convention=0x8E4E;
constexpr Enum compare_mode=0x884C, compare_func=0x884D, compare_ref=0x884E;
constexpr Enum less=0x0201, less_equal=0x0203;
constexpr Enum framebuffer=0x8D40, color_attachment=0x8CE0, depth_attachment=0x8D00, complete=0x8CD5;
constexpr Enum color_bit=0x4000, depth_bit=0x0100, depth_test=0x0B71, blend=0x0BE2;
constexpr Enum offset_fill=0x8037, src_alpha=0x0302, inv_src_alpha=0x0303, one=1;
constexpr Enum triangles_mode=4, lines_mode=1, line_width_range=0x846E;
constexpr Enum mip_bilinear=0x2701,base_level=0x813C;
constexpr Enum extensions_enum=0x1F03,extension_count_enum=0x821D;
constexpr Enum anisotropy_enum=0x84FE,max_anisotropy_enum=0x84FF;
constexpr Int cascade_count=3,map_size=surface_resolution_pixels(3);
constexpr Int material_page_layers=25,shadow_page_layers=14,page_shadow_size=320;
constexpr Int material_page_texels=96,material_border=2,material_tile_size=material_page_texels+material_border*2;
constexpr float material_page_meters=32.0f,shadow_page_meters=19.0f;
constexpr Int material_count=static_cast<Int>(LunarMaterial::Emissive);
constexpr Int material_variants=4;
constexpr Int line_material=static_cast<Int>(LunarMaterial::Hair)+1;
using Feature=GraphicsFeature;
constexpr auto glsl330="#version 330 core\n";
constexpr auto glsl430="#version 430 core\n";

const char* graphics_shader_options(){
    static const std::string options=[]{
        std::string code=R"GLSL(uniform uint graphicsFlags,graphicsLow,graphicsMedium;
bool option(uint bit){return (graphicsFlags&bit)!=0u;}
int quality(uint bit){return !option(bit)?0:(graphicsLow&bit)!=0u?1:(graphicsMedium&bit)!=0u?2:3;}
int taps(uint bit,int low,int medium,int high){
    int level=quality(bit);
    return level<=1?low:level==2?medium:high;
}
)GLSL";
        for(std::size_t i=0;i<graphics_feature_keys.size();++i){
            code+="#define OPT_";
            code+=graphics_feature_keys[i];
            code+=" "+std::to_string(graphics_bit(static_cast<Feature>(i)))+"u\n";
        }
        return code;
    }();
    return options.c_str();
}

#define LUNAR_GL_FUNCTIONS(X) \
    X(void,GetFloatv,(Enum,float*)) X(void,GetIntegerv,(Enum,Int*)) \
    X(UInt,CreateShader,(Enum)) X(void,ShaderSource,(UInt,Int,const char* const*,const Int*)) \
    X(void,CompileShader,(UInt)) X(void,GetShaderiv,(UInt,Enum,Int*)) X(void,GetShaderInfoLog,(UInt,Int,Int*,char*)) \
    X(void,DeleteShader,(UInt)) X(UInt,CreateProgram,()) X(void,AttachShader,(UInt,UInt)) X(void,LinkProgram,(UInt)) \
    X(void,GetProgramiv,(UInt,Enum,Int*)) X(void,GetProgramInfoLog,(UInt,Int,Int*,char*)) \
    X(void,DeleteProgram,(UInt)) X(void,UseProgram,(UInt)) X(Int,GetUniformLocation,(UInt,const char*)) \
    X(void,UniformMatrix4fv,(Int,Int,Boolean,const float*)) X(void,Uniform4fv,(Int,Int,const float*)) \
    X(void,Uniform3f,(Int,float,float,float)) X(void,Uniform2f,(Int,float,float)) X(void,Uniform1f,(Int,float)) X(void,Uniform1i,(Int,Int)) X(void,Uniform1ui,(Int,UInt)) \
    X(void,GenVertexArrays,(Int,UInt*)) X(void,DeleteVertexArrays,(Int,const UInt*)) X(void,BindVertexArray,(UInt)) \
    X(void,GenBuffers,(Int,UInt*)) X(void,DeleteBuffers,(Int,const UInt*)) X(void,BindBuffer,(Enum,UInt)) \
    X(void,BufferData,(Enum,Size,const void*,Enum)) X(void,BufferSubData,(Enum,Size,Size,const void*)) \
    X(void,EnableVertexAttribArray,(UInt)) X(void,VertexAttribPointer,(UInt,Int,Enum,Boolean,Int,const void*)) \
    X(void,GenTextures,(Int,UInt*)) X(void,DeleteTextures,(Int,const UInt*)) X(void,BindTexture,(Enum,UInt)) \
    X(void,ActiveTexture,(Enum)) X(void,TexParameteri,(Enum,Enum,Int)) X(void,GenerateMipmap,(Enum)) \
    X(void,TexImage2D,(Enum,Int,Int,Int,Int,Int,Enum,Enum,const void*)) \
    X(void,TexImage3D,(Enum,Int,Int,Int,Int,Int,Int,Enum,Enum,const void*)) \
    X(void,TexSubImage2D,(Enum,Int,Int,Int,Int,Int,Enum,Enum,const void*)) \
    X(void,TexSubImage3D,(Enum,Int,Int,Int,Int,Int,Int,Int,Enum,Enum,const void*)) \
    X(void,GenFramebuffers,(Int,UInt*)) X(void,DeleteFramebuffers,(Int,const UInt*)) X(void,BindFramebuffer,(Enum,UInt)) \
    X(void,FramebufferTexture2D,(Enum,Enum,Enum,UInt,Int)) X(Enum,CheckFramebufferStatus,(Enum)) \
    X(void,FramebufferTextureLayer,(Enum,Enum,UInt,Int,Int)) X(void,DrawBuffers,(Int,const Enum*)) \
    X(void,DrawBuffer,(Enum)) X(void,ReadBuffer,(Enum)) \
    X(void,ClearBufferuiv,(Enum,Int,const UInt*)) X(void,ProvokingVertex,(Enum)) \
    X(void,Viewport,(Int,Int,Int,Int)) X(void,Enable,(Enum)) X(void,Disable,(Enum)) \
    X(void,BlendFunc,(Enum,Enum)) X(void,BlendFuncSeparate,(Enum,Enum,Enum,Enum)) X(void,DepthMask,(Boolean)) X(void,DepthFunc,(Enum)) X(void,PolygonOffset,(float,float)) \
    X(void,ClearColor,(float,float,float,float)) X(void,Clear,(Enum)) X(void,LineWidth,(float)) X(void,DrawArrays,(Enum,Int,Int))

struct Api {
#define DECLARE(result,name,args) result (*name) args{};
    LUNAR_GL_FUNCTIONS(DECLARE)
#undef DECLARE

    bool load(const LunarRenderer::Resolve& resolve){
#define LOAD(result,name,args) name=reinterpret_cast<decltype(name)>(resolve("gl" #name)); if(!name) return false;
        LUNAR_GL_FUNCTIONS(LOAD)
#undef LOAD
        return true;
    }
};
#undef LUNAR_GL_FUNCTIONS

struct ComputeApi {
    void (*dispatch)(UInt,UInt,UInt){};
    void (*barrier)(Enum){};
    void (*bind_image)(UInt,UInt,Int,Boolean,Int,Enum,Enum){};
    void (*bind_buffer)(Enum,UInt,UInt){};
    void (*multi_draw)(Enum,const void*,Int,Int){};

    bool load(const LunarRenderer::Resolve& resolve){
        dispatch=reinterpret_cast<decltype(dispatch)>(resolve("glDispatchCompute"));
        barrier=reinterpret_cast<decltype(barrier)>(resolve("glMemoryBarrier"));
        bind_image=reinterpret_cast<decltype(bind_image)>(resolve("glBindImageTexture"));
        bind_buffer=reinterpret_cast<decltype(bind_buffer)>(resolve("glBindBufferBase"));
        multi_draw=reinterpret_cast<decltype(multi_draw)>(resolve("glMultiDrawArraysIndirect"));
        return dispatch && barrier && bind_image && bind_buffer;
    }
};

enum Uniform {
    ViewProjection, Terrain, LightProjection, Eye, Sun, Daylight, ShadowMap, SurfaceMaps,
    LightCount, LightPositions, LightColors, InverseViewProjection, Source, StepUV, Extract,
    Glow, DepthMap, PixelSize, ProjectionScale, BaseMap, NormalMap, MaterialMap, LightCascades,
    ReflectedMap, AccumulatedMap, DensityMap, HistoryMap, HistoryDepth, PreviousViewProjection,
    PreviousInverseViewProjection, UseHistory, UseTransparency, SpatialAA, Time,
    RayFocus, RayNodeCount, FrameIndex, ReservoirMap, RaySpecular, RayIndirect, RayDirect,
    PreviousSpecular, PreviousIndirect, PreviousDirect, UseRays, VisibilityMap, ClusterCount, VolumeCount, VolumeOrigin, VolumeAxis, VolumeColor, VolumeParams,
    UseVolumes, VolumeMap, OverlayMap, Sharpen, GroundPageMap, GroundTable, GroundOrigin, UseGroundPages,
    ShadowPageMap, ShadowTable, ShadowOrigin, ShadowX, ShadowY, ShadowSun, UseShadowPages, ShadowPageBlend, RayBlend,
    NeuralFocus, UseNeural, LearnedUpscale, InputPixelSize, GraphicsFlags,GraphicsLow,GraphicsMedium,ShadowQuality,AAQuality, UseShadowMaps,
    UseReflection, WeightedTransparency, BloomStrength, Exposure, UniformCount
};
constexpr std::array<const char*,UniformCount> uniform_names{
    "viewProjection","terrain","lightProjection","eye","sun","daylight","shadowMap","surfaceMaps",
    "lightCount","localPositionRadius[0]","localColorPower[0]","inverseViewProjection","source","stepUV","extract",
    "glow","depthMap","pixelSize","projectionScale","baseMap","normalMap","materialMap","lightCascades[0]",
    "reflectedMap","accumulatedMap","densityMap","historyMap","historyDepth","previousViewProjection",
    "previousInverseViewProjection","useHistory","useTransparency","spatialAA","time",
    "rayFocus","rayNodeCount","frameIndex","reservoirMap","raySpecular","rayIndirect","rayDirect",
    "previousSpecular","previousIndirect","previousDirect","useRays","visibilityMap","clusterCount","volumeCount","volumeOriginLength[0]","volumeAxisRadius[0]",
    "volumeColorDensity[0]","volumeParameters[0]","useVolumes","volumeMap","overlayMap","sharpen",
    "groundPageMap","groundTable","groundOrigin","useGroundPages","shadowPageMap","shadowTable",
    "shadowOrigin","shadowX","shadowY","shadowSun","useShadowPages","shadowPageBlend","rayBlend","neuralFocus","useNeural","learnedUpscale","inputPixelSize","graphicsFlags","graphicsLow","graphicsMedium","shadowQuality","aaQuality","useShadowMaps",
    "useReflection","weightedTransparency","bloomStrength","exposure"
};

struct Program {
    UInt id{};
    std::array<Int,UniformCount> location{};
    Int operator[](Uniform uniform) const { return location[uniform]; }
};

struct Target {
    UInt fbo{},color{},depth{};
    std::array<UInt,2> aux{};
};

struct Attachment { Enum format; bool linear{}; };

struct Vertex {
    Vec3 position;
    Color tint;
    float material;
    Vec3 coordinate;
};
static_assert(sizeof(Vertex)==11*sizeof(float));

struct Draw {
    Int first{},count{};
    float width{1.0f};
    bool casts_shadow{};
    float depth{};
};

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

struct MaterialTexel { float shade,roughness,height; };

MaterialTexel texel(Int x,Int y,Int material,Int variant){
    const float u=float(x)/map_size,v=float(y)/map_size;
    const Int seed=material+variant*431;
    const float broad=noise(u*4,v*4,11+seed,4);
    const float cloud=noise(u*16,v*16,19+seed,16);
    const float fine=noise(u*96,v*96,53+seed,96);
    const float grit=random(x,y,37+seed);
    float shade=0.57f+(broad-0.5f)*0.16f+(cloud-0.5f)*0.17f+(fine-0.5f)*0.08f;
    float height=(fine-0.5f)*0.11f,roughness=0.51f+0.12f*cloud;
    switch(static_cast<LunarMaterial>(material)){
        case LunarMaterial::Regolith: {
            const float pebbles=smooth(0.70f,0.94f,fine);
            shade=0.34f+0.31f*broad+0.15f*cloud+0.17f*fine+(grit-0.5f)*0.10f+pebbles*0.22f;
            height=cloud*0.23f+fine*0.20f+pebbles*0.36f+grit*0.04f;
            roughness=0.91f;
            break;
        }
        case LunarMaterial::Solar: {
            shade=0.48f+0.16f*cloud+0.10f*broad;
            height=0.025f*fine;
            roughness=0.26f+0.15f*cloud;
            break;
        }
        case LunarMaterial::Paint:
        case LunarMaterial::Pad: {
            const bool pad=material==static_cast<Int>(LunarMaterial::Pad);
            const float mottling=noise(u*7,v*11,113+seed,7,11);
            const float worn=smooth(0.56f,0.82f,mottling)*smooth(0.50f,0.91f,fine);
            const float scratches=noise(u*24,v*144,131+seed,24,144);
            const float scuff=smooth(0.67f,0.93f,scratches)*smooth(0.49f,0.79f,cloud);
            shade+=0.15f*(mottling-0.5f)-worn*(pad?0.24f:0.18f)+0.12f*scuff+
                (grit-0.5f)*(pad?0.12f:0.025f);
            height-=worn*0.055f+scuff*0.023f;
            roughness+=pad?0.24f:0.09f*worn;
            break;
        }
        case LunarMaterial::Alloy: {
            const float brushing=noise(u*16,v*192,93+seed,16,192);
            const float scratches=smooth(0.74f,0.91f,brushing);
            shade+=0.25f*(brushing-0.5f)+0.16f*scratches;
            height+=0.09f*brushing-0.17f*scratches;
            roughness=0.32f+0.13f*cloud;
            break;
        }
        case LunarMaterial::Glass:
            shade=0.63f+0.16f*cloud;
            height*=0.05f;
            roughness=0.18f+0.14f*cloud;
            break;
        case LunarMaterial::Fabric: {
            const float weave=0.5f+0.5f*std::cos(201.0619f*u+0.42f*cloud)*std::cos(201.0619f*v+0.42f*broad);
            shade=0.51f+0.15f*cloud+(weave-0.5f)*0.13f;
            height+=0.15f*weave;
            roughness=0.77f+0.11f*cloud;
            break;
        }
        case LunarMaterial::Emissive:
        case LunarMaterial::Skin:
        case LunarMaterial::Marking:
        case LunarMaterial::Hair:
            break;
    }
    return {shade,roughness,height};
}

std::vector<std::uint8_t> material_maps(){
    constexpr std::size_t pixels=std::size_t(map_size)*map_size;
    std::vector<std::uint8_t> maps(pixels*material_count*material_variants*4);
    std::vector<MaterialTexel> layer(pixels);
    const auto byte=[](float x){ return std::uint8_t(std::clamp(x,0.0f,1.0f)*255.0f+0.5f); };
    for(Int index=0;index<material_count*material_variants;++index){
        const auto at=[&](Int x,Int y)->MaterialTexel&{
            return layer[std::size_t((y+map_size)%map_size)*map_size+(x+map_size)%map_size];
        };
        auto& pool=TaskPool::shared();
        pool.parallel_for(map_size,[&](std::size_t row){
            const Int y=static_cast<Int>(row);
            for(Int x=0;x<map_size;++x) at(x,y)=texel(x,y,index/material_variants,index%material_variants);
        });
        pool.parallel_for(map_size,[&](std::size_t row){
            const Int y=static_cast<Int>(row);
            for(Int x=0;x<map_size;++x){
                const auto i=(index*pixels+std::size_t(y)*map_size+x)*4;
                maps[i]=byte(at(x,y).shade);
                maps[i+1]=byte(at(x,y).roughness);
                maps[i+2]=byte(0.5f+(at(x+1,y).height-at(x-1,y).height)*1.55f);
                maps[i+3]=byte(0.5f+(at(x,y+1).height-at(x,y-1).height)*1.55f);
            }
        });
    }
    return maps;
}

std::vector<std::uint8_t> ground_page(lunar::PageCoord tile){
    std::vector<std::uint8_t> out(material_tile_size*material_tile_size*4);
    const auto density=[](float x,float z){
        const float drift=noise(x*0.13f,z*0.13f,691);
        const float mineral=noise((x+drift*2.2f)*0.56f,(z-drift)*0.56f,697);
        const float grains=noise((0.81f*x-0.59f*z)*1.30f,(0.59f*x+0.81f*z)*1.30f,701);
        return std::clamp(drift*0.33f+mineral*0.48f+grains*0.19f,0.0f,1.0f);
    };
    const auto byte=[](float x){ return static_cast<std::uint8_t>(std::clamp(x,0.0f,1.0f)*255.0f+0.5f); };
    constexpr float texel=material_page_meters/material_page_texels;
    for(int j=0;j<material_tile_size;++j) for(int i=0;i<material_tile_size;++i){
        const float x=tile.x*material_page_meters+(i-material_border+0.5f)*texel;
        const float z=tile.y*material_page_meters+(j-material_border+0.5f)*texel;
        const float n=density(x,z);
        const auto at=std::size_t(j*material_tile_size+i)*4;
        out[at]=byte(n);
        out[at+1]=byte(0.72f+0.24f*n);
        out[at+2]=byte(0.5f+(density(x+texel,z)-density(x-texel,z))*0.68f);
        out[at+3]=byte(0.5f+(density(x,z+texel)-density(x,z-texel))*0.68f);
    }
    return out;
}

lunar::PageCoord page_origin(float x,float y,float tile_size){
    return {int(std::floor(x/tile_size))-lunar::page_grid/2,int(std::floor(y/tile_size))-lunar::page_grid/2};
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

using lunar::TerrainVertex;

TerrainVertex terrain_vertex(float x,float z,float half){
    constexpr float step=0.8f;
    const float dx=terrain_height(x+step,z,half)-terrain_height(x-step,z,half);
    const float dz=terrain_height(x,z+step,half)-terrain_height(x,z-step,half);
    return {{x,terrain_height(x,z,half),z},normalize({-dx,2.0f*step,-dz}),0};
}

void add_boulders(std::vector<TerrainVertex>& vertices,float half,int quality,float range){
    constexpr float spacing=7.2f;
    const Int cells=static_cast<Int>(std::ceil((half+range)/spacing));
    for(Int z=-cells;z<=cells;++z) for(Int x=-cells;x<=cells;++x){
        const float chance=random(x,z,312);
        if(chance<quality_budget(quality,0.93f,0.85f,0.78f)) continue;
        const float px=(x+0.14f+0.70f*random(x,z,314))*spacing;
        const float pz=(z+0.14f+0.70f*random(x,z,327))*spacing;
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
    std::array<std::array<TerrainVertex,97>,2> row{};
    vertices.reserve(vertices.size()+(distances.size()-1)*4*segments*6);
    for(Int side=0;side<4;++side){
        for(std::size_t i=0;i<distances.size();++i){
            auto& current=row[i%2];
            const auto& previous=row[(i+1)%2];
            const float radius=half+distances[i];
            for(Int step=0;step<=segments;++step){
                const float p=radius*(2.0f*float(step)/segments-1.0f);
                const float x=side==0?p:side==1?radius:side==2?-p:-radius;
                const float z=side==0?-radius:side==1?p:side==2?radius:-p;
                current[step]=terrain_vertex(x,z,half);
            }
            if(i==0) continue;
            for(Int step=0;step<segments;++step){
                vertices.insert(vertices.end(),{previous[step],current[step+1],current[step],
                    previous[step],previous[step+1],current[step+1]});
            }
        }
    }
    if(clutter>0)add_boulders(vertices,half,clutter,170.0f*range/100.0f);
    return vertices;
}

bool invert(const Mat4& matrix,Mat4& inverse){
    // Pivoted Gauss-Jordan in double precision keeps the reconstruction
    // stable across the existing large orbit and FPV near/far ratios.
    double m[4][8]{};
    for(Int row=0;row<4;++row){
        for(Int col=0;col<4;++col) m[row][col]=matrix.m[col*4+row];
        m[row][row+4]=1.0;
    }
    for(Int col=0;col<4;++col){
        Int pivot=col;
        for(Int row=col+1;row<4;++row) if(std::abs(m[row][col])>std::abs(m[pivot][col])) pivot=row;
        if(std::abs(m[pivot][col])<1e-12) return false;
        for(Int i=0;i<8;++i) std::swap(m[pivot][i],m[col][i]);
        const double divisor=m[col][col];
        for(double& value:m[col]) value/=divisor;
        for(Int row=0;row<4;++row){
            if(row==col) continue;
            const double factor=m[row][col];
            for(Int i=0;i<8;++i) m[row][i]-=factor*m[col][i];
        }
    }
    for(Int row=0;row<4;++row) for(Int col=0;col<4;++col) inverse.m[col*4+row]=float(m[row][col+4]);
    return true;
}

Mat4 light_projection(const LunarScene& scene,float reach,int shadow_size){
    const Vec3 sun=normalize(scene.sun_direction);
    const Vec3 up=std::abs(sun.y)>0.92f?Vec3{0,0,1}:Vec3{0,1,0};
    const Vec3 x=normalize(cross(sun,up)),y=normalize(cross(x,sun));
    const float texel=reach*2.0f/shadow_size;
    const auto snap=[&](Vec3 axis){
        const float offset=dot(scene.focus,axis);
        return axis*(std::round(offset/texel)*texel-offset);
    };
    const Vec3 focus=scene.focus+snap(x)+snap(y);
    const float distance=reach+96.0f;
    return multiply(ortho(-reach,reach,-reach,reach,1.0f,distance*2.0f),
                    look_at(focus+sun*distance,focus,up));
}

float halton(unsigned index,unsigned base){
    float value=0.0f,factor=1.0f;
    for(;index;index/=base){
        factor/=float(base);
        value+=float(index%base)*factor;
    }
    return value;
}

Vec3 forward(const Mat4& projection){
    return normalize({projection.m[3],projection.m[7],projection.m[11]});
}

} // namespace

struct LunarRenderer::Impl {
    struct ScreenFeatures {
        bool reflections{},bloom{},volumes{},visibility{},rays{},ray_filter{},reservoirs{};
        bool operator==(const ScreenFeatures&) const = default;
    };

    Api gl;
    ComputeApi compute;
    bool visibility_path{},visibility_targets{},cluster_path{};
    UInt cluster_vao{},cluster_vertices{},cluster_data{},cluster_commands{};
    Target visibility;
    Program primitive_ids,material_resolve,cull_clusters;
    lunar::TerrainPages terrain_pages;
    bool advanced{},ray_targets{},ray_history{},ray_tree_ready{},reservoir_history{};
    Int ray_width{},ray_height{},ray_nodes{};
    UInt ray_frame{},ray_index{},reservoir_index{};
    std::array<UInt,3> ray_buffers{};
    Program reservoir_program,ray_program,ray_filter,neural_train_program;
    UInt neural_buffer{};
    Vec3 neural_focus{};
    float neural_daylight{};
    bool neural{},neural_initialized{};
    std::array<Target,2> reservoirs,ray_filtered;
    Target ray_raw;
    std::vector<lunar::RayTriangle> landscape_rays;
    std::array<lunar::Float4,64> fixture_positions{};
    std::size_t fixture_count{};
    lunar::LightFader raster_lights,ray_lights;
    std::span<const LunarLight> raster_fixtures,ray_fixtures;
    Vec3 ray_focus{};
    std::uint64_t ray_generation{1};
    std::future<lunar::RayTree> ray_job;
    std::chrono::steady_clock::time_point next_ray_build{};
    UInt ground_atlas{},ground_table{},shadow_atlas{},shadow_table{};
    lunar::PageCache ground_pages{material_page_layers},shadow_pages{shadow_page_layers};
    lunar::PageCoord ground_origin{},shadow_origin{};
    Vec3 page_sun{},page_x{},page_y{};
    float shadow_page_blend{},ray_blend{},frame_seconds{};
    bool page_support{},shadow_pages_active{};
    bool loaded{},ready{},in_scene{},targets_ready{},history_valid{},earth_sky_failed{},skin_in_scene{};
    Int width{},height{},output_width{},output_height{},history_width{},history_height{},terrain_vertices{},stone_start{};
    GraphicsSettings graphics=graphics_settings_from_environment();
    std::uint32_t frame_features{all_graphics_features};
    Program volume_program;
    Target volume_target,overlay;
    std::array<LunarVolume,8> volumes{};
    std::size_t volume_count{};
    unsigned jitter{},history_index{};
    float terrain_extent{},max_line_width{1.0f};
    LunarScene scene;
    Mat4 view_projection,inverse,previous_projection,previous_inverse;
    std::array<Mat4,cascade_count> cascades;
    Program geometry,shadow,sky,earth_sky,lighting,reflection,transparency,temporal,line,bloom,composite,fxaa;
    UInt mesh_vao{},terrain_vao{},screen_vao{},mesh_buffer{},terrain_buffer{},surface_maps{};
    UInt shadow_fbo{},shadow_layers{};
    Int shadow_size{},max_anisotropy{1};
    Size buffer_capacity{};
    Target gbuffer,hdr,reflections,oit,bloom_a,bloom_b,ldr;
    std::array<Target,2> history{};
    std::vector<Vertex> vertices;
    std::vector<Draw> opaque,translucent,lines;

    ~Impl(){
        if(!loaded) return;
        destroy_screen_targets();
        gl.DeleteBuffers(static_cast<Int>(ray_buffers.size()),ray_buffers.data());
        gl.DeleteFramebuffers(1,&shadow_fbo);
        const UInt textures[]{surface_maps,shadow_layers,ground_atlas,ground_table,shadow_atlas,shadow_table};
        const UInt buffers[]{mesh_buffer,terrain_buffer,cluster_vertices,cluster_data,cluster_commands,neural_buffer};
        const UInt arrays[]{mesh_vao,terrain_vao,screen_vao,cluster_vao};
        gl.DeleteTextures(6,textures);
        gl.DeleteBuffers(6,buffers);
        gl.DeleteVertexArrays(4,arrays);
        for(const Program* program:{&geometry,&shadow,&sky,&earth_sky,&lighting,&reflection,&transparency,
                                    &temporal,&line,&bloom,&composite,&fxaa,&reservoir_program,&ray_program,&ray_filter,
                                    &primitive_ids,&material_resolve,&cull_clusters,&volume_program,&neural_train_program})
            if(program->id) gl.DeleteProgram(program->id);
    }

    UInt compile(Enum stage,std::initializer_list<const char*> sources){
        const UInt shader=gl.CreateShader(stage);
        if(!shader) return 0;
        gl.ShaderSource(shader,static_cast<Int>(sources.size()),sources.begin(),nullptr);
        gl.CompileShader(shader);
        Int ok{};
        gl.GetShaderiv(shader,compile_status,&ok);
        if(ok) return shader;
        char message[2048]{};
        gl.GetShaderInfoLog(shader,sizeof(message),nullptr,message);
        std::cerr<<"Lunar shader: "<<message<<'\n';
        gl.DeleteShader(shader);
        return 0;
    }

    bool link(Program& program,const char* vertex,std::initializer_list<const char*> fragment){
        const UInt vs=compile(vertex_shader,{vertex}),fs=compile(fragment_shader,fragment);
        if(vs && fs){
            program.id=gl.CreateProgram();
            gl.AttachShader(program.id,vs);
            gl.AttachShader(program.id,fs);
            gl.LinkProgram(program.id);
        }
        if(vs) gl.DeleteShader(vs);
        if(fs) gl.DeleteShader(fs);
        if(!program.id) return false;
        Int ok{};
        gl.GetProgramiv(program.id,link_status,&ok);
        if(!ok){
            char message[2048]{};
            gl.GetProgramInfoLog(program.id,sizeof(message),nullptr,message);
            std::cerr<<"Lunar shader link: "<<message<<'\n';
            return false;
        }
        uniforms(program);
        return true;
    }

    void uniforms(Program& program){
        for(std::size_t i=0;i<uniform_names.size();++i)
            program.location[i]=gl.GetUniformLocation(program.id,uniform_names[i]);
    }

    bool link_compute(Program& program,std::initializer_list<const char*> source){
        const UInt shader=compile(compute_shader,source);
        if(!shader) return false;
        program.id=gl.CreateProgram();
        if(program.id){
            gl.AttachShader(program.id,shader);
            gl.LinkProgram(program.id);
        }
        gl.DeleteShader(shader);
        if(!program.id) return false;
        Int ok{};
        gl.GetProgramiv(program.id,link_status,&ok);
        if(!ok){
            char message[2048]{};
            gl.GetProgramInfoLog(program.id,sizeof(message),nullptr,message);
            std::cerr<<"Lunar compute link: "<<message<<'\n';
            return false;
        }
        uniforms(program);
        return true;
    }

    void bind_texture(Int unit,UInt tex,Enum target=texture_2d){
        gl.ActiveTexture(texture0+unit);
        gl.BindTexture(target,tex);
    }

    void sample(const Program& program,Uniform u,Int unit,UInt tex,Enum target=texture_2d){
        bind_texture(unit,tex,target);
        gl.Uniform1i(program[u],unit);
    }

    UInt texture(Int w,Int h,Attachment config){
        const auto format=config.format;
        const bool depth=format==depth24,single_channel=format==r16f || format==r32f,integer=format==r32ui;
        UInt id{};
        gl.GenTextures(1,&id);
        bind_texture(0,id);
        gl.TexImage2D(texture_2d,0,format,w,h,0,depth?depth_component:integer?red_integer:single_channel?red:rgba,
            depth || integer?int_type:float_type,nullptr);
        const auto filter=config.linear?linear:nearest;
        gl.TexParameteri(texture_2d,min_filter,filter);
        gl.TexParameteri(texture_2d,mag_filter,filter);
        gl.TexParameteri(texture_2d,wrap_s,clamp_edge);
        gl.TexParameteri(texture_2d,wrap_t,clamp_edge);
        return id;
    }

    bool make_target(Target& target,Int w,Int h,std::initializer_list<Attachment> colors,bool needs_depth=false){
        gl.GenFramebuffers(1,&target.fbo);
        gl.BindFramebuffer(framebuffer,target.fbo);
        const std::array<UInt*,3> ids{&target.color,&target.aux[0],&target.aux[1]};
        const Enum attachments[]{color_attachment,color_attachment+1,color_attachment+2};
        std::size_t i=0;
        for(const auto& config:colors){
            *ids[i]=texture(w,h,config);
            gl.FramebufferTexture2D(framebuffer,attachments[i],texture_2d,*ids[i],0);
            ++i;
        }
        gl.DrawBuffers(static_cast<Int>(colors.size()),attachments);
        if(needs_depth){
            target.depth=texture(w,h,{depth24});
            gl.FramebufferTexture2D(framebuffer,depth_attachment,texture_2d,target.depth,0);
        }
        return gl.CheckFramebufferStatus(framebuffer)==complete;
    }

    void attach_depth(UInt depth){
        gl.FramebufferTexture2D(framebuffer,depth_attachment,texture_2d,depth,0);
    }

    void destroy(Target& target){
        gl.DeleteFramebuffers(1,&target.fbo);
        const UInt textures[]{target.color,target.depth,target.aux[0],target.aux[1]};
        gl.DeleteTextures(4,textures);
        target={};
    }

    void destroy_screen_targets(){
        for(Target* target:{&gbuffer,&hdr,&reflections,&oit,&history[0],&history[1],&bloom_a,&bloom_b,&ldr,
                            &reservoirs[0],&reservoirs[1],&ray_raw,&ray_filtered[0],&ray_filtered[1],&visibility,&volume_target,&overlay})
            destroy(*target);
    }

    static bool trace_enabled(const GraphicsSettings& settings){
        constexpr auto effects=graphics_bit(Feature::RayShadows)|graphics_bit(Feature::RayReflections)|
            graphics_bit(Feature::RayOcclusion)|graphics_bit(Feature::RayIndirect);
        return settings.enabled(Feature::RayTracing) && (settings.features&effects)!=0;
    }

    ScreenFeatures screen_features(const GraphicsSettings& settings) const {
        const bool compute_available=settings.path==GraphicsPath::Auto && advanced;
        const bool reservoirs_needed=compute_available && settings.enabled(Feature::LocalLights) &&
            settings.enabled(Feature::ReservoirLights);
        const bool rays_needed=compute_available && (trace_enabled(settings) || reservoirs_needed);
        return {settings.enabled(Feature::ScreenReflections),settings.bloom>0,
                settings.enabled(Feature::VolumetricDust) || settings.enabled(Feature::VolumetricPlumes),
                settings.path==GraphicsPath::Auto && visibility_path && settings.enabled(Feature::VisibilityBuffer),
                rays_needed,rays_needed && settings.enabled(Feature::RayDenoise),reservoirs_needed};
    }

    void resize_rays(const ScreenFeatures& features){
        ray_targets=false;
        ray_history=reservoir_history=false;
        ray_index=reservoir_index=0;
        if(!features.rays) return;
        const float scale=graphics.ray_scale*0.01f;
        const float ratio=std::min(scale,1300.0f*scale/width);
        ray_width=std::max(1,Int(width*ratio));
        ray_height=std::max(1,Int(height*ratio));
        const auto triple=[&](Target& target){
            return make_target(target,ray_width,ray_height,{{rgba16f,true},{rgba16f,true},{rgba16f,true}});
        };
        ray_targets=triple(ray_raw)
            && (!features.ray_filter || (triple(ray_filtered[0]) && triple(ray_filtered[1])))
            && (!features.reservoirs || (make_target(reservoirs[0],ray_width,ray_height,{{rgba32f}})
                && make_target(reservoirs[1],ray_width,ray_height,{{rgba32f}})));
        if(!ray_targets){
            for(Target* target:{&ray_raw,&ray_filtered[0],&ray_filtered[1],&reservoirs[0],&reservoirs[1]})
                destroy(*target);
            std::cerr<<"Lunar compute targets unavailable; using the raster lighting path.\n";
        }
    }

    void resize(Int w,Int h){
        destroy_screen_targets();
        output_width=w;
        output_height=h;
        // Cap offscreen bandwidth at very high display resolutions. The HUD
        // still draws at full display resolution after the 3D image is scaled.
        const float pixels=float(w)*h;
        const float factor=std::min(graphics.render_scale*0.01f,std::sqrt(3500000.0f/pixels));
        const float history_scale=temporal_aa()?std::min(1.0f,std::sqrt(6200000.0f/pixels)):factor;
        width=std::max(1,Int(w*factor));
        height=std::max(1,Int(h*factor));
        history_width=std::max(1,Int(w*history_scale));
        history_height=std::max(1,Int(h*history_scale));
        history_valid=false;
        history_index=jitter=0;
        const auto features=screen_features(graphics);
        const bool history_needed=temporal_aa() || features.ray_filter || features.reservoirs;
        const bool resolve_ready=history_needed?
            make_target(history[0],history_width,history_height,{{rgba16f,true},{r32f}}) &&
            make_target(history[1],history_width,history_height,{{rgba16f,true},{r32f}}):
            make_target(history[0],history_width,history_height,{{rgba16f,true}});
        const Int bw=std::max(1,width/bloom_divisor()),bh=std::max(1,height/bloom_divisor());
        targets_ready=resolve_ready && make_target(gbuffer,width,height,{{rgba8},{rgba16f},{rgba8}},true)
            && make_target(hdr,width,height,{{rgba16f,true}})
            && (!features.reflections || make_target(reflections,std::max(1,width/2),std::max(1,height/2),{{rgba16f,true}}))
            && make_target(oit,width,height,{{rgba16f},{r16f}})
            && (!features.bloom || (make_target(bloom_a,bw,bh,{{rgba16f,true}}) && make_target(bloom_b,bw,bh,{{rgba16f,true}})))
            && (!features.volumes || make_target(volume_target,std::max(1,width/2),std::max(1,height/2),{{rgba16f,true}}))
            && make_target(overlay,width,height,{{rgba8,true}})
            && make_target(ldr,history_width,history_height,{{rgba8,true}});
        if(targets_ready){
            gl.BindFramebuffer(framebuffer,oit.fbo);
            attach_depth(gbuffer.depth);
            targets_ready=gl.CheckFramebufferStatus(framebuffer)==complete;
        }
        visibility_targets=false;
        if(targets_ready && features.visibility){
            visibility_targets=make_target(visibility,width,height,{{r32ui}});
            if(visibility_targets){
                attach_depth(gbuffer.depth);
                visibility_targets=gl.CheckFramebufferStatus(framebuffer)==complete;
            }
        }
        if(targets_ready) resize_rays(features);
        gl.BindFramebuffer(framebuffer,0);
        bind_texture(0,0);
        if(!targets_ready){
            destroy_screen_targets();
            std::cerr<<"Lunar rendering targets unavailable for "<<width<<'x'<<height<<"; using the standard view.\n";
        }
    }

    bool resize_shadow_maps(Int size){
        UInt candidate{};
        gl.GenTextures(1,&candidate);
        bind_texture(0,candidate,texture_array);
        gl.TexImage3D(texture_array,0,depth24,size,size,cascade_count,0,depth_component,int_type,nullptr);
        gl.TexParameteri(texture_array,compare_mode,compare_ref);
        gl.TexParameteri(texture_array,compare_func,less_equal);
        gl.TexParameteri(texture_array,min_filter,linear);
        gl.TexParameteri(texture_array,mag_filter,linear);
        gl.TexParameteri(texture_array,wrap_s,clamp_edge);
        gl.TexParameteri(texture_array,wrap_t,clamp_edge);
        bind_texture(0,0,texture_array);

        gl.BindFramebuffer(framebuffer,shadow_fbo);
        gl.FramebufferTextureLayer(framebuffer,depth_attachment,candidate,0,0);
        gl.DrawBuffer(0);
        gl.ReadBuffer(0);
        const bool ok=gl.CheckFramebufferStatus(framebuffer)==complete;
        if(ok){
            gl.DeleteTextures(1,&shadow_layers);
            shadow_layers=candidate;
            shadow_size=size;
        }else{
            gl.FramebufferTextureLayer(framebuffer,depth_attachment,shadow_layers,0,0);
            gl.DeleteTextures(1,&candidate);
        }
        gl.BindFramebuffer(framebuffer,0);
        return ok;
    }

    bool initialize_shadow(){
        gl.GenFramebuffers(1,&shadow_fbo);
        return resize_shadow_maps(cascade_resolution_pixels(graphics.shadow_resolution));
    }

    void initialize_texture_filtering(const Resolve& resolve){
        const auto get_extension=reinterpret_cast<const unsigned char*(*)(Enum,UInt)>(resolve("glGetStringi"));
        if(!get_extension)return;
        Int count{};
        gl.GetIntegerv(extension_count_enum,&count);
        for(Int i=0;i<count;++i){
            const auto* text=get_extension(extensions_enum,static_cast<UInt>(i));
            if(!text)continue;
            const std::string_view extension(reinterpret_cast<const char*>(text));
            if(extension=="GL_EXT_texture_filter_anisotropic" || extension=="GL_ARB_texture_filter_anisotropic"){
                float limit=1.0f;
                gl.GetFloatv(max_anisotropy_enum,&limit);
                max_anisotropy=static_cast<Int>(std::clamp(limit,1.f,16.f));
                break;
            }
        }
    }

    void configure_surface_sampling(){
        bind_texture(0,surface_maps,texture_array);
        gl.TexParameteri(texture_array,base_level,3-graphics.surface_resolution);
        gl.TexParameteri(texture_array,min_filter,graphics.texture_filter==0?mip_bilinear:trilinear);
        if(max_anisotropy>1)
            gl.TexParameteri(texture_array,anisotropy_enum,std::min(max_anisotropy,texture_anisotropy(graphics.texture_filter)));
        bind_texture(0,0,texture_array);
    }

    bool initialize_pages(){
        ground_table=texture(lunar::page_grid,lunar::page_grid,{rgba8});
        shadow_table=texture(lunar::page_grid,lunar::page_grid,{rgba8});
        gl.GenTextures(1,&ground_atlas);
        bind_texture(0,ground_atlas,texture_array);
        gl.TexImage3D(texture_array,0,rgba8,material_tile_size,material_tile_size,material_page_layers,0,rgba,byte_type,nullptr);
        gl.TexParameteri(texture_array,min_filter,trilinear);
        gl.TexParameteri(texture_array,mag_filter,linear);
        gl.TexParameteri(texture_array,wrap_s,clamp_edge);
        gl.TexParameteri(texture_array,wrap_t,clamp_edge);

        gl.GenTextures(1,&shadow_atlas);
        bind_texture(0,shadow_atlas,texture_array);
        gl.TexImage3D(texture_array,0,depth24,page_shadow_size,page_shadow_size,shadow_page_layers,0,depth_component,int_type,nullptr);
        gl.TexParameteri(texture_array,compare_mode,compare_ref);
        gl.TexParameteri(texture_array,compare_func,less_equal);
        gl.TexParameteri(texture_array,min_filter,linear);
        gl.TexParameteri(texture_array,mag_filter,linear);
        gl.TexParameteri(texture_array,wrap_s,clamp_edge);
        gl.TexParameteri(texture_array,wrap_t,clamp_edge);
        gl.BindFramebuffer(framebuffer,shadow_fbo);
        gl.FramebufferTextureLayer(framebuffer,depth_attachment,shadow_atlas,0,0);
        const bool ok=gl.CheckFramebufferStatus(framebuffer)==complete;
        gl.FramebufferTextureLayer(framebuffer,depth_attachment,shadow_layers,0,0);
        gl.BindFramebuffer(framebuffer,0);
        bind_texture(0,0,texture_array);
        bind_texture(0,0);
        return ok;
    }

    void initialize_advanced(const Resolve& resolve){
        Int major{},minor{};
        gl.GetIntegerv(major_version,&major);
        gl.GetIntegerv(minor_version,&minor);
        if(major<4 || (major==4 && minor<3) || !compute.load(resolve)) return;
        using namespace lunar_shaders;
        const char* options=graphics_shader_options();
        visibility_path=link(primitive_ids,visibility_vertex,{visibility_fragment})
            && link(material_resolve,screen_vertex,{visibility_inputs,options,gbuffer_outputs,
                surface_shaders::fields,surface_common,reconstruct_common,visibility_resolve});
        cluster_path=visibility_path && compute.multi_draw && link_compute(cull_clusters,{glsl430,options,cluster_culling});
        if(cluster_path){
            gl.GenVertexArrays(1,&cluster_vao);
            gl.GenBuffers(1,&cluster_vertices);
            gl.GenBuffers(1,&cluster_data);
            gl.GenBuffers(1,&cluster_commands);
            gl.BindVertexArray(cluster_vao);
            gl.BindBuffer(array_buffer,cluster_vertices);
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0,3,float_type,0,sizeof(TerrainVertex),nullptr);
            gl.BindVertexArray(0);
            gl.BindBuffer(array_buffer,0);
        }
        advanced=link_compute(reservoir_program,{glsl430,options,ray_inputs,reconstruct_common,reservoir_compute})
            && link_compute(ray_program,{glsl430,options,ray_inputs,reconstruct_common,ray_queries,ray_compute})
            && link(ray_filter,screen_vertex,{filter_ray_inputs,options,neural_cache,learned_filter,reconstruct_common,filter_ray_fragment});
        if(advanced){
            gl.GenBuffers(static_cast<Int>(ray_buffers.size()),ray_buffers.data());
            gl.GenBuffers(1,&neural_buffer);
            neural=link_compute(neural_train_program,{neural_train_inputs,neural_cache,reconstruct_common,neural_train});
        }
        std::cout<<(advanced?"Lunar graphics: OpenGL compute rays and reservoir lighting.\n":
                             "Lunar graphics: deferred raster lighting.\n");
        page_support=initialize_pages();
        if(visibility_path) std::cout<<"Lunar visibility buffer"<<(cluster_path?" with GPU cluster draws.\n":".\n");
        if(page_support) std::cout<<"Lunar material and near-shadow page caches.\n";
        if(neural) std::cout<<"Lunar online radiance model and learned ray filter.\n";
    }

    bool initialize(const Resolve& resolve){
        loaded=gl.load(resolve);
        if(!loaded) return false;
        using namespace lunar_shaders;
        const char* options=graphics_shader_options();
        if(!link(geometry,mesh_vertex,{geometry_inputs,options,gbuffer_outputs,surface_shaders::fields,surface_common,geometry_sample,gbuffer_fragment}) ||
           !link(shadow,depth_vertex,{depth_fragment}) || !link(sky,screen_vertex,{sky_fragment}) ||
           !link(lighting,screen_vertex,{scene_inputs,options,lighting_uniforms,lighting_common,reconstruct_common,deferred_fragment}) ||
           !link(reflection,screen_vertex,{scene_inputs,options,reflection_inputs,reconstruct_common,reflection_fragment}) ||
           !link(transparency,mesh_vertex,{geometry_inputs,options,lighting_uniforms,transparency_outputs,
                                          surface_shaders::fields,surface_common,geometry_sample,lighting_common,transparency_fragment}) ||
           !link(temporal,screen_vertex,{temporal_inputs,options,reconstruct_common,temporal_fragment}) ||
           !link(line,mesh_vertex,{line_fragment}) || !link(bloom,screen_vertex,{bloom_fragment}) ||
           !link(composite,screen_vertex,{glsl330,options,composite_fragment}) || !link(fxaa,screen_vertex,{fxaa_fragment,learned_upscale}) ||
           !link(volume_program,screen_vertex,{volume_inputs,options,reconstruct_common,volume_fragment}) ||
           !initialize_shadow()) return false;

        gl.GenTextures(1,&surface_maps);
        bind_texture(0,surface_maps,texture_array);
        const auto maps=material_maps();
        gl.TexImage3D(texture_array,0,rgba8,map_size,map_size,material_count*material_variants,0,rgba,byte_type,maps.data());
        gl.TexParameteri(texture_array,wrap_s,repeat);
        gl.TexParameteri(texture_array,wrap_t,repeat);
        gl.TexParameteri(texture_array,min_filter,trilinear);
        gl.TexParameteri(texture_array,mag_filter,linear);
        gl.GenerateMipmap(texture_array);
        initialize_texture_filtering(resolve);
        configure_surface_sampling();

        gl.GenVertexArrays(1,&mesh_vao);
        gl.GenVertexArrays(1,&terrain_vao);
        gl.GenVertexArrays(1,&screen_vao);
        gl.GenBuffers(1,&mesh_buffer);
        gl.GenBuffers(1,&terrain_buffer);
        gl.BindVertexArray(mesh_vao);
        gl.BindBuffer(array_buffer,mesh_buffer);
        buffer_capacity=8192;
        gl.BufferData(array_buffer,buffer_capacity,nullptr,stream_draw);
        gl.EnableVertexAttribArray(0);
        gl.EnableVertexAttribArray(1);
        gl.EnableVertexAttribArray(2);
        gl.VertexAttribPointer(0,3,float_type,0,sizeof(Vertex),nullptr);
        gl.VertexAttribPointer(1,4,float_type,0,sizeof(Vertex),reinterpret_cast<const void*>(offsetof(Vertex,tint)));
        gl.VertexAttribPointer(2,1,float_type,0,sizeof(Vertex),reinterpret_cast<const void*>(offsetof(Vertex,material)));
        gl.EnableVertexAttribArray(5);
        gl.VertexAttribPointer(5,3,float_type,0,sizeof(Vertex),reinterpret_cast<const void*>(offsetof(Vertex,coordinate)));
        gl.BindVertexArray(terrain_vao);
        gl.BindBuffer(array_buffer,terrain_buffer);
        gl.EnableVertexAttribArray(0);
        gl.VertexAttribPointer(0,3,float_type,0,sizeof(TerrainVertex),nullptr);
        gl.EnableVertexAttribArray(3);
        gl.VertexAttribPointer(3,3,float_type,0,sizeof(TerrainVertex),reinterpret_cast<const void*>(offsetof(TerrainVertex,normal)));
        gl.EnableVertexAttribArray(4);
        gl.VertexAttribPointer(4,1,float_type,0,sizeof(TerrainVertex),reinterpret_cast<const void*>(offsetof(TerrainVertex,stone)));
        gl.BindVertexArray(0);
        gl.BindBuffer(array_buffer,0);
        float widths[2]{1,1};
        gl.GetFloatv(line_width_range,widths);
        max_line_width=std::max(widths[1],1.0f);
        initialize_advanced(resolve);
        ready=true;
        return true;
    }

    Int bloom_divisor() const { return quality_budget(graphics.bloom_quality,8,6,4); }
    bool temporal_aa() const { return graphics.aa==GraphicsAA::Temporal; }
    bool auto_path() const { return graphics.path==GraphicsPath::Auto; }
    bool use_ground_pages() const { return auto_path() && page_support && graphics.enabled(Feature::VirtualTextures); }
    bool use_clusters() const {
        return cluster_path && graphics.enabled(Feature::GpuTerrain) && graphics.enabled(Feature::TerrainRelief);
    }
    bool weighted_transparency() const { return graphics.transparency==GraphicsTransparency::Weighted; }

    void set_graphics_settings(GraphicsSettings next){
        next.normalize();
        if(next==graphics) return;
        finish();
        const bool resolution_changed=(next.aa==GraphicsAA::Temporal)!=temporal_aa() ||
            next.render_scale!=graphics.render_scale || next.ray_scale!=graphics.ray_scale ||
            next.bloom_quality!=graphics.bloom_quality;
        if(next.path!=graphics.path || resolution_changed || screen_features(next)!=screen_features(graphics))
            output_width=0;
        if(loaded && next.path==GraphicsPath::Classic && graphics.path!=GraphicsPath::Classic){
            destroy_screen_targets();
            targets_ready=false;
        }
        if(next.quality(Feature::TerrainRelief)!=graphics.quality(Feature::TerrainRelief) ||
           next.ground_clutter!=graphics.ground_clutter || next.clutter_range!=graphics.clutter_range)
            terrain_extent=0.0f;
        if(!graphics.enabled(Feature::NeuralRadiance) && next.enabled(Feature::NeuralRadiance))
            neural_initialized=false;
        const bool texture_changed=next.surface_resolution!=graphics.surface_resolution ||
                                   next.texture_filter!=graphics.texture_filter;
        graphics=next;
        if(ready && texture_changed)configure_surface_sampling();
        if(ready && graphics.shadows!=GraphicsShadows::Off &&
           shadow_size!=cascade_resolution_pixels(graphics.shadow_resolution))
            resize_shadow_maps(cascade_resolution_pixels(graphics.shadow_resolution));
        reset_history(false);
    }

    bool begin(const LunarScene& next,Int w,Int h){
        if(!ready || graphics.path==GraphicsPath::Classic || w<1 || h<1 || next.half_extent<=0.0f) return false;
        if(output_width!=w || output_height!=h) resize(w,h);
        if(!targets_ready) return false;
        if(terrain_extent!=next.half_extent){
            const auto landscape=terrain_mesh(next.half_extent,graphics.quality(Feature::TerrainRelief),
                                              graphics.ground_clutter,graphics.clutter_range);
            terrain_vertices=static_cast<Int>(landscape.size());
            const auto rocks=std::find_if(landscape.begin(),landscape.end(),[](const TerrainVertex& v){ return v.stone>0.0f; });
            stone_start=static_cast<Int>(rocks-landscape.begin());
            gl.BindBuffer(array_buffer,terrain_buffer);
            gl.BufferData(array_buffer,Size(landscape.size()*sizeof(TerrainVertex)),landscape.data(),static_draw);
            gl.BindBuffer(array_buffer,0);
            if(cluster_path) initialize_terrain_pages(landscape);
            terrain_extent=next.half_extent;
            reset_history(true);
            landscape_rays.clear();
            if(advanced){
                landscape_rays.reserve((landscape.size()-6)/3);
                for(std::size_t i=6;i+2<landscape.size();i+=3){
                    const bool rock=landscape[i].stone>0.0f;
                    const Color tint=rock?Color{0.36f,0.33f,0.31f,1}:Color{0.49f,0.46f,0.43f,1};
                    landscape_rays.push_back(lunar::ray_triangle(landscape[i].position,landscape[i+1].position,
                        landscape[i+2].position,tint,static_cast<float>(LunarMaterial::Regolith)));
                }
            }
        }
        if(length(scene.eye-next.eye)>22.0f || dot(forward(scene.view_projection),forward(next.view_projection))<0.85f)
            reset_history(false);
        frame_seconds=std::clamp(next.seconds-scene.seconds,1.0f/240,0.1f);
        scene=next;
        scene.sun_direction=normalize(scene.sun_direction);
        scene.daylight=std::clamp(scene.daylight,0.0f,1.0f);
        if(graphics.shadows!=GraphicsShadows::Off && scene.daylight>0.001f){
            constexpr float reach[]{36.0f,106.0f,360.0f};
            for(Int i=0;i<cascade_count;++i) cascades[i]=light_projection(scene,reach[i],shadow_size);
        }else cascades.fill(Mat4::identity());
        const int light_quality=graphics.quality(Feature::LocalLights);
        const auto raster_budget=light_quality?quality_budget(light_quality,2u,4u,8u):0u;
        const auto ray_budget=graphics.enabled(Feature::ReservoirLights)?
            quality_budget(light_quality,8u,24u,64u):raster_budget;
        raster_fixtures=raster_lights.update(scene,raster_budget,16);
        ray_fixtures=ray_lights.update(scene,ray_targets && light_quality?ray_budget:0u);

        view_projection=scene.view_projection;
        if(temporal_aa()){
            const unsigned sample_index=jitter%8+1;
            ++jitter;
            const float dx=(halton(sample_index,2)-0.5f)*2.0f/width;
            const float dy=(halton(sample_index,3)-0.5f)*2.0f/height;
            for(Int column=0;column<4;++column){
                const float w_row=view_projection.m[column*4+3];
                view_projection.m[column*4]+=dx*w_row;
                view_projection.m[column*4+1]+=dy*w_row;
            }
        }
        if(!invert(view_projection,inverse)) return false;
        in_scene=true;
        return true;
    }

    void append(const std::vector<Vec3>& points,Int material,Color tint,bool is_line,float width_px,
                const LunarSurfaceMapping& mapping={}){
        const std::size_t arity=is_line?2u:3u;
        const std::size_t count=points.size()/arity*arity;
        if(!count || tint.a<=0.0f) return;
        const Int first=static_cast<Int>(vertices.size());
        for(std::size_t i=0;i<count;++i) vertices.push_back({points[i],tint,float(material),mapping.coordinate(points[i])});
        const bool transparent=tint.a<0.999f;
        if(!is_line && !transparent && material==static_cast<Int>(LunarMaterial::Skin)) skin_in_scene=true;
        const bool casts=!is_line && !transparent && material!=static_cast<Int>(LunarMaterial::Emissive) &&
            material!=static_cast<Int>(LunarMaterial::Marking);
        auto& draws=is_line?lines:transparent?translucent:opaque;
        const bool sort=!is_line && transparent && !weighted_transparency();
        if(!sort && !draws.empty()){
            auto& last=draws.back();
            if(last.first+last.count==first && last.width==width_px && last.casts_shadow==casts){
                last.count+=static_cast<Int>(count);
                return;
            }
        }
        float depth{};
        if(sort){
            const Vec3 axis=forward(scene.view_projection);
            for(std::size_t i=0;i<count;++i) depth+=dot(axis,points[i]-scene.eye);
            depth/=static_cast<float>(count);
        }
        draws.push_back({first,static_cast<Int>(count),width_px,casts,depth});
    }

    void upload(){
        if(vertices.empty()) return;
        gl.BindBuffer(array_buffer,mesh_buffer);
        const Size bytes=Size(vertices.size()*sizeof(Vertex));
        buffer_capacity=std::max<Size>(buffer_capacity,8192);
        while(buffer_capacity<bytes) buffer_capacity*=2;
        gl.BufferData(array_buffer,buffer_capacity,nullptr,stream_draw);
        gl.BufferSubData(array_buffer,0,bytes,vertices.data());
    }

    void matrix(const Program& program,Uniform u,const Mat4& m){ gl.UniformMatrix4fv(program[u],1,0,m.m.data()); }
    void vector(const Program& program,Uniform u,Vec3 v){ gl.Uniform3f(program[u],v.x,v.y,v.z); }

    void shader_options(const Program& program,std::uint32_t features){
        gl.Uniform1ui(program[GraphicsFlags],features);
        gl.Uniform1ui(program[GraphicsLow],graphics.low_quality);
        gl.Uniform1ui(program[GraphicsMedium],graphics.medium_quality);
        gl.Uniform1i(program[ShadowQuality],graphics.shadow_quality);
        gl.Uniform1i(program[AAQuality],graphics.aa_quality);
    }

    void render_shadows(){
        if(graphics.shadows==GraphicsShadows::Off || scene.daylight<=0.001f) return;
        gl.BindFramebuffer(framebuffer,shadow_fbo);
        gl.Viewport(0,0,shadow_size,shadow_size);
        gl.Enable(offset_fill);
        gl.PolygonOffset(1.2f,1.8f);
        gl.UseProgram(shadow.id);
        for(Int i=0;i<cascade_count;++i){
            gl.FramebufferTextureLayer(framebuffer,depth_attachment,shadow_layers,0,i);
            gl.Clear(depth_bit);
            matrix(shadow,LightProjection,cascades[i]);
            gl.BindVertexArray(mesh_vao);
            for(const auto& draw:opaque) if(draw.casts_shadow) gl.DrawArrays(triangles_mode,draw.first,draw.count);
            gl.BindVertexArray(terrain_vao);
            gl.DrawArrays(triangles_mode,stone_start,terrain_vertices-stone_start);
        }
        gl.Disable(offset_fill);
    }

    void upload_page_table(UInt texture_id,const lunar::PagePlan& plan){
        bind_texture(0,texture_id);
        gl.TexSubImage2D(texture_2d,0,0,0,lunar::page_grid,lunar::page_grid,rgba,byte_type,plan.table.data());
    }

    void update_ground_pages(){
        if(!use_ground_pages()) return;
        ground_origin=page_origin(scene.focus.x,scene.focus.z,material_page_meters);
        lunar::PageRevisions revisions;
        revisions.fill(1);
        const auto plan=ground_pages.plan(ground_origin,revisions,quality_budget(graphics.quality(Feature::VirtualTextures),1,2,4),
                                          std::max(1,int(frame_seconds*1600.0f)));
        if(!plan.uploads.empty()){
            std::vector<std::vector<std::uint8_t>> pixels(plan.uploads.size());
            TaskPool::shared().parallel_for(pixels.size(),[&](std::size_t index){
                pixels[index]=ground_page(plan.uploads[index].coordinate);
            });
            bind_texture(0,ground_atlas,texture_array);
            for(std::size_t index=0;index<pixels.size();++index){
                const auto& upload=plan.uploads[index];
                gl.TexSubImage3D(texture_array,0,0,0,upload.layer,material_tile_size,material_tile_size,1,
                    rgba,byte_type,pixels[index].data());
            }
            gl.GenerateMipmap(texture_array);
        }
        upload_page_table(ground_table,plan);
    }

    void ground_page_state(const Program& program){
        const bool use=use_ground_pages();
        shader_options(program,graphics.features);
        gl.Uniform1i(program[UseGroundPages],use);
        sample(program,GroundPageMap,9,use?ground_atlas:0,texture_array);
        sample(program,GroundTable,10,use?ground_table:0);
        gl.Uniform2f(program[GroundOrigin],float(ground_origin.x),float(ground_origin.y));
    }

    lunar::PageRevisions shadow_revisions() const {
        lunar::PageRevisions stamps{};
        const auto hash_coordinate=[](std::uint64_t stamp,float v){
            return (stamp^static_cast<std::uint32_t>(std::lround(v*64.0f)))*1099511628211ull;
        };
        for(const auto& draw:opaque){
            if(!draw.casts_shadow) continue;
            for(Int i=draw.first;i+2<draw.first+draw.count;i+=3){
                std::array<float,3> x{},y{};
                for(int v=0;v<3;++v){
                    const auto p=vertices[i+v].position;
                    x[v]=dot(p,page_x);
                    y[v]=dot(p,page_y);
                }
                const auto x0=std::max(0,int(std::floor(*std::min_element(x.begin(),x.end())/shadow_page_meters))-shadow_origin.x);
                const auto x1=std::min(lunar::page_grid-1,int(std::floor(*std::max_element(x.begin(),x.end())/shadow_page_meters))-shadow_origin.x);
                const auto y0=std::max(0,int(std::floor(*std::min_element(y.begin(),y.end())/shadow_page_meters))-shadow_origin.y);
                const auto y1=std::min(lunar::page_grid-1,int(std::floor(*std::max_element(y.begin(),y.end())/shadow_page_meters))-shadow_origin.y);
                if(x1<x0 || y1<y0) continue;
                std::uint64_t stamp=1469598103934665603ull;
                for(int v=0;v<3;++v){
                    const auto p=vertices[i+v].position;
                    stamp=hash_coordinate(hash_coordinate(hash_coordinate(stamp,p.x),p.y),p.z);
                }
                for(int y_page=y0;y_page<=y1;++y_page) for(int x_page=x0;x_page<=x1;++x_page){
                    auto& current=stamps[y_page*lunar::page_grid+x_page];
                    current=((current^stamp)*1099511628211ull)|1ull;
                }
            }
        }
        return stamps;
    }

    void render_shadow_pages(){
        shadow_pages_active=auto_path() && page_support && graphics.shadows==GraphicsShadows::Paged && scene.daylight>0.001f;
        if(!shadow_pages_active) return;
        constexpr float angular_step=0.0087266463f;
        const auto snap=[](float angle){ return std::round(angle/angular_step)*angular_step; };
        const float azimuth=std::atan2(scene.sun_direction.z,scene.sun_direction.x);
        const float elevation=std::asin(std::clamp(scene.sun_direction.y,-1.0f,1.0f));
        const float az=snap(azimuth),el=snap(elevation);
        const float angular_error=std::max(std::abs(azimuth-az),std::abs(elevation-el));
        shadow_page_blend=0.72f*(1.0f-smooth(angular_step*0.16f,angular_step*0.49f,angular_error));
        const Vec3 direction{std::cos(el)*std::cos(az),std::sin(el),std::cos(el)*std::sin(az)};
        if(dot(direction-page_sun,direction-page_sun)>0.000001f){
            page_sun=direction;
            shadow_pages.clear();
        }
        const Vec3 up=std::abs(page_sun.y)>0.92f?Vec3{0,0,1}:Vec3{0,1,0};
        page_x=normalize(cross(page_sun*-1.0f,up));
        page_y=normalize(cross(page_x,page_sun*-1.0f));
        shadow_origin=page_origin(dot(scene.focus,page_x),dot(scene.focus,page_y),shadow_page_meters);
        const auto plan=shadow_pages.plan(shadow_origin,shadow_revisions(),quality_budget(graphics.shadow_quality,1,2,4),
                                          std::max(1,int(frame_seconds*1500.0f)));
        upload_page_table(shadow_table,plan);
        if(plan.uploads.empty()) return;
        const Mat4 view=look_at(page_sun*2048.0f,Vec3{},up);
        gl.BindFramebuffer(framebuffer,shadow_fbo);
        gl.Viewport(0,0,page_shadow_size,page_shadow_size);
        gl.Enable(offset_fill);
        gl.PolygonOffset(1.0f,2.0f);
        gl.UseProgram(shadow.id);
        for(const auto& upload:plan.uploads){
            const float x=upload.coordinate.x*shadow_page_meters,y=upload.coordinate.y*shadow_page_meters;
            const Mat4 projection=multiply(ortho(x,x+shadow_page_meters,y,y+shadow_page_meters,1.0f,4096.0f),view);
            gl.FramebufferTextureLayer(framebuffer,depth_attachment,shadow_atlas,0,upload.layer);
            gl.Clear(depth_bit);
            matrix(shadow,LightProjection,projection);
            gl.BindVertexArray(mesh_vao);
            for(const auto& draw:opaque) if(draw.casts_shadow) gl.DrawArrays(triangles_mode,draw.first,draw.count);
            gl.BindVertexArray(terrain_vao);
            gl.DrawArrays(triangles_mode,stone_start,terrain_vertices-stone_start);
        }
        gl.Disable(offset_fill);
    }

    void lighting_state(const Program& program){
        vector(program,Eye,scene.eye);
        vector(program,Sun,scene.sun_direction);
        gl.Uniform1f(program[Daylight],scene.daylight);
        gl.UniformMatrix4fv(program[LightCascades],cascade_count,0,cascades.front().m.data());
        sample(program,ShadowMap,4,shadow_layers,texture_array);
        sample(program,ShadowPageMap,11,shadow_pages_active?shadow_atlas:0,texture_array);
        sample(program,ShadowTable,12,shadow_pages_active?shadow_table:0);
        gl.Uniform1i(program[UseShadowPages],shadow_pages_active);
        gl.Uniform1f(program[ShadowPageBlend],shadow_page_blend);
        gl.Uniform1i(program[UseShadowMaps],graphics.shadows!=GraphicsShadows::Off);
        gl.Uniform2f(program[ShadowOrigin],float(shadow_origin.x),float(shadow_origin.y));
        vector(program,ShadowX,page_x);
        vector(program,ShadowY,page_y);
        vector(program,ShadowSun,page_sun);
        std::array<float,64> position{},colors{};
        const auto count=raster_fixtures.size();
        for(std::size_t i=0;i<count;++i){
            const auto& light=raster_fixtures[i];
            const auto j=i*4;
            position[j]=light.position.x;
            position[j+1]=light.position.y;
            position[j+2]=light.position.z;
            position[j+3]=light.radius;
            colors[j]=light.color.r;
            colors[j+1]=light.color.g;
            colors[j+2]=light.color.b;
            colors[j+3]=light.power*std::lerp(1.0f,0.62f,scene.daylight);
        }
        gl.Uniform1i(program[LightCount],static_cast<Int>(count));
        gl.Uniform4fv(program[LightPositions],static_cast<Int>(count),position.data());
        gl.Uniform4fv(program[LightColors],static_cast<Int>(count),colors.data());
    }

    void screen(const Program& program){
        gl.UseProgram(program.id);
        gl.BindVertexArray(screen_vao);
    }

    void screen_size(const Program& program){
        gl.Uniform2f(program[PixelSize],1.0f/width,1.0f/height);
    }

    void gbuffer_state(const Program& program){
        sample(program,BaseMap,0,gbuffer.color);
        sample(program,NormalMap,1,gbuffer.aux[0]);
        sample(program,MaterialMap,2,gbuffer.aux[1]);
        sample(program,DepthMap,3,gbuffer.depth);
        matrix(program,InverseViewProjection,inverse);
        screen_size(program);
    }

    bool campus_atmosphere(const Mat4& projection,Vec3 eye,float seconds,Int w,Int h){
        Mat4 sky_inverse;
        if(!ready || graphics.path==GraphicsPath::Classic || !graphics.enabled(Feature::EarthAtmosphere) ||
           earth_sky_failed || !invert(projection,sky_inverse)) return false;
        if(!earth_sky.id && !link(earth_sky,lunar_shaders::screen_vertex,
                                  {glsl330,graphics_shader_options(),lunar_shaders::earth_sky_fragment})){
            earth_sky_failed=true;
            return false;
        }
        gl.BindFramebuffer(framebuffer,0);
        gl.Viewport(0,0,w,h);
        gl.Disable(depth_test);
        gl.Disable(blend);
        screen(earth_sky);
        shader_options(earth_sky,graphics.features);
        matrix(earth_sky,InverseViewProjection,sky_inverse);
        vector(earth_sky,Eye,eye);
        gl.Uniform1f(earth_sky[Time],seconds);
        gl.DrawArrays(triangles_mode,0,3);
        gl.BindVertexArray(0);
        gl.UseProgram(0);
        gl.Enable(blend);
        return true;
    }

    void initialize_terrain_pages(const std::vector<TerrainVertex>& landscape){
        terrain_pages=lunar::make_terrain_pages(landscape,static_cast<std::size_t>(stone_start));
        gl.BindBuffer(array_buffer,cluster_vertices);
        gl.BufferData(array_buffer,Size(terrain_pages.buffer_vertices()*sizeof(TerrainVertex)),nullptr,stream_draw);
        gl.BufferSubData(array_buffer,0,Size(terrain_pages.coarse.size()*sizeof(TerrainVertex)),terrain_pages.coarse.data());
        gl.BindBuffer(draw_indirect_buffer,cluster_commands);
        gl.BufferData(draw_indirect_buffer,Size(terrain_pages.clusters.size()*sizeof(lunar::UInt4)),nullptr,stream_draw);
        gl.BindBuffer(draw_indirect_buffer,0);
        gl.BindBuffer(array_buffer,0);
    }

    void prepare_cluster_draws(){
        if(graphics.enabled(Feature::StreamedGeometry)){
            const auto uploads=terrain_pages.update(scene.eye,scene.focus,
                quality_budget(graphics.quality(Feature::StreamedGeometry),16u,32u,64u));
            gl.BindBuffer(array_buffer,cluster_vertices);
            for(const auto& upload:uploads){
                const auto& data=terrain_pages.pages[upload.page].vertices;
                gl.BufferSubData(array_buffer,Size(upload.first_vertex*sizeof(TerrainVertex)),
                    Size(data.size()*sizeof(TerrainVertex)),data.data());
            }
        }
        upload_storage(5,cluster_data,terrain_pages.clusters.data(),terrain_pages.clusters.size());
        compute.bind_buffer(storage_buffer,6,cluster_commands);
        gl.UseProgram(cull_clusters.id);
        shader_options(cull_clusters,graphics.features);
        matrix(cull_clusters,ViewProjection,view_projection);
        vector(cull_clusters,Eye,scene.eye);
        gl.Uniform1i(cull_clusters[ClusterCount],static_cast<Int>(terrain_pages.clusters.size()));
        compute.dispatch(static_cast<UInt>((terrain_pages.clusters.size()+63)/64),1,1);
        compute.barrier(command_barrier|storage_barrier);
    }

    void render_visibility(){
        gl.BindFramebuffer(framebuffer,visibility.fbo);
        gl.Viewport(0,0,width,height);
        const UInt zero[4]{};
        gl.ClearBufferuiv(color_buffer,0,zero);
        gl.Clear(depth_bit);
        gl.ProvokingVertex(last_vertex_convention);
        gl.UseProgram(primitive_ids.id);
        matrix(primitive_ids,ViewProjection,view_projection);
        gl.Uniform1i(primitive_ids[Terrain],1);
        const bool clustered=use_clusters();
        if(clustered){
            gl.BindVertexArray(cluster_vao);
            gl.BindBuffer(draw_indirect_buffer,cluster_commands);
            compute.multi_draw(triangles_mode,nullptr,static_cast<Int>(terrain_pages.clusters.size()),0);
            gl.BindBuffer(draw_indirect_buffer,0);
        }else{
            gl.BindVertexArray(terrain_vao);
            gl.DrawArrays(triangles_mode,0,terrain_vertices);
        }
        gl.Uniform1i(primitive_ids[Terrain],0);
        gl.BindVertexArray(mesh_vao);
        for(const auto& draw:opaque) gl.DrawArrays(triangles_mode,draw.first,draw.count);

        gl.BindFramebuffer(framebuffer,gbuffer.fbo);
        attach_depth(0);
        gl.Disable(depth_test);
        screen(material_resolve);
        sample(material_resolve,VisibilityMap,0,visibility.color);
        sample(material_resolve,DepthMap,3,gbuffer.depth);
        sample(material_resolve,SurfaceMaps,5,surface_maps,texture_array);
        ground_page_state(material_resolve);
        screen_size(material_resolve);
        matrix(material_resolve,InverseViewProjection,inverse);
        vector(material_resolve,Eye,scene.eye);
        compute.bind_buffer(storage_buffer,3,clustered?cluster_vertices:terrain_buffer);
        compute.bind_buffer(storage_buffer,4,mesh_buffer);
        gl.DrawArrays(triangles_mode,0,3);
        attach_depth(gbuffer.depth);
    }

    void render_geometry(){
        gl.BindFramebuffer(framebuffer,gbuffer.fbo);
        gl.Viewport(0,0,width,height);
        gl.ClearColor(0,0,0,0);
        gl.Clear(color_bit|depth_bit);
        gl.UseProgram(geometry.id);
        matrix(geometry,ViewProjection,view_projection);
        vector(geometry,Eye,scene.eye);
        sample(geometry,SurfaceMaps,5,surface_maps,texture_array);
        ground_page_state(geometry);
        gl.Uniform1i(geometry[Terrain],1);
        gl.BindVertexArray(terrain_vao);
        gl.DrawArrays(triangles_mode,0,terrain_vertices);
        gl.Uniform1i(geometry[Terrain],0);
        gl.BindVertexArray(mesh_vao);
        for(const auto& draw:opaque) gl.DrawArrays(triangles_mode,draw.first,draw.count);
    }

    void reset_history(bool scene_reset){
        history_valid=ray_history=reservoir_history=false;
        if(scene_reset){
            raster_lights.reset();
            ray_lights.reset();
            ray_blend=0.0f;
            ++ray_generation;
            ray_tree_ready=false;
            neural_initialized=false;
            next_ray_build={};
            ground_pages.clear();
            shadow_pages.clear();
        }
    }

    template<class T>
    void upload_storage(UInt binding,UInt buffer,const T* data,std::size_t count){
        gl.BindBuffer(storage_buffer,buffer);
        gl.BufferData(storage_buffer,static_cast<Size>(std::max<std::size_t>(count,1)*sizeof(T)),
            count?data:nullptr,stream_draw);
        compute.bind_buffer(storage_buffer,binding,buffer);
    }

    void update_ray_tree(){
        using namespace std::chrono;
        if(ray_job.valid() && ray_job.wait_for(seconds(0))==std::future_status::ready){
            auto tree=ray_job.get();
            if(tree.generation==ray_generation){
                upload_storage(0,ray_buffers[0],tree.nodes.data(),tree.nodes.size());
                upload_storage(1,ray_buffers[1],tree.triangles.data(),tree.triangles.size());
                ray_nodes=static_cast<Int>(tree.nodes.size());
                ray_focus=tree.focus;
                ray_tree_ready=true;
            }
        }

        const auto now=steady_clock::now();
        if(ray_job.valid() || (now<next_ray_build && length(scene.focus-ray_focus)<22.0f)) return;
        constexpr std::size_t triangle_limit=126000;
        constexpr float radius=244.0f;
        std::vector<lunar::RayTriangle> triangles;
        triangles.reserve(std::min<std::size_t>(triangle_limit,vertices.size()/3+8192));
        const auto include=[&](const lunar::RayTriangle& t){
            const float x=std::clamp(scene.focus.x,std::min({t.a[0],t.b[0],t.c[0]}),std::max({t.a[0],t.b[0],t.c[0]}));
            const float z=std::clamp(scene.focus.z,std::min({t.a[2],t.b[2],t.c[2]}),std::max({t.a[2],t.b[2],t.c[2]}));
            const float dx=x-scene.focus.x,dz=z-scene.focus.z;
            if(dx*dx+dz*dz<radius*radius && triangles.size()<triangle_limit) triangles.push_back(t);
        };
        for(const auto& draw:opaque){
            for(Int i=draw.first;i+2<draw.first+draw.count;i+=3){
                const auto& v=vertices[static_cast<std::size_t>(i)];
                if(v.material!=static_cast<float>(LunarMaterial::Marking))
                    include(lunar::ray_triangle(v.position,vertices[i+1].position,vertices[i+2].position,v.tint,v.material));
            }
        }
        for(const auto& triangle:landscape_rays) include(triangle);
        auto job=std::make_shared<std::packaged_task<lunar::RayTree()>>(
            [input=std::move(triangles),focus=scene.focus,generation=ray_generation]() mutable {
                return lunar::build_ray_tree(std::move(input),focus,generation);
            });
        ray_job=job->get_future();
        TaskPool::shared().submit([job]{ (*job)(); });
        next_ray_build=now+milliseconds(92);
    }

    void upload_fixtures(bool active){
        struct Fixture { lunar::Float4 position,color; };
        std::array<Fixture,64> lights{};
        const auto count=active?ray_fixtures.size():0;
        bool same=count==fixture_count;
        for(std::size_t i=0;i<count;++i){
            const auto& light=ray_fixtures[i];
            lights[i]={{light.position.x,light.position.y,light.position.z,light.radius},
                       {light.color.r,light.color.g,light.color.b,light.power*std::lerp(1.0f,0.62f,scene.daylight)}};
            same=same && lights[i].position==fixture_positions[i];
            fixture_positions[i]=lights[i].position;
        }
        if(!same) reservoir_history=false;
        fixture_count=count;
        upload_storage(2,ray_buffers[2],lights.data(),count);
    }

    void dispatch_rays(){
        compute.dispatch((ray_width+7)/8,(ray_height+7)/8,1);
        compute.barrier(image_barrier|fetch_barrier);
    }

    void ray_inputs_for(const Program& program){
        gl.UseProgram(program.id);
        gbuffer_state(program);
        sample(program,HistoryDepth,5,history[history_index].aux[0]);
        matrix(program,PreviousViewProjection,previous_projection);
        matrix(program,PreviousInverseViewProjection,previous_inverse);
        vector(program,Eye,scene.eye);
        vector(program,Sun,scene.sun_direction);
        vector(program,RayFocus,ray_focus);
        gl.Uniform1f(program[Daylight],scene.daylight);
        shader_options(program,frame_features);
        gl.Uniform1i(program[RayNodeCount],(frame_features&graphics_bit(Feature::RayTracing))?ray_nodes:0);
        gl.Uniform1i(program[LightCount],static_cast<Int>(fixture_count));
        gl.Uniform1i(program[FrameIndex],static_cast<Int>(ray_frame&0x7fffffffu));
    }

    void update_neural(){
        if(!neural_initialized || length(neural_focus-scene.focus)>20.0f ||
           std::abs(neural_daylight-scene.daylight)>0.22f){
            std::array<lunar::Float4,38> weights{};
            for(int i=0;i<24;++i) for(int j=0;j<4;++j)
                weights[i][j]=float(hash(i,j,4201)&65535u)/65535.0f*0.72f-0.36f;
            upload_storage(7,neural_buffer,weights.data(),weights.size());
            neural_focus=scene.focus;
            neural_daylight=scene.daylight;
            neural_initialized=true;
        }else compute.bind_buffer(storage_buffer,7,neural_buffer);
        gl.UseProgram(neural_train_program.id);
        sample(neural_train_program,RayIndirect,0,ray_raw.aux[0]);
        sample(neural_train_program,DepthMap,1,gbuffer.depth);
        sample(neural_train_program,NormalMap,2,gbuffer.aux[0]);
        matrix(neural_train_program,InverseViewProjection,inverse);
        vector(neural_train_program,NeuralFocus,neural_focus);
        gl.Uniform1i(neural_train_program[UseNeural],1);
        gl.Uniform1i(neural_train_program[FrameIndex],static_cast<Int>(ray_frame&0x7fffffffu));
        compute.dispatch(1,1,1);
        compute.barrier(storage_barrier);
    }

    bool render_rays(){
        if(!auto_path() || !ray_targets) return false;
        const bool trace_requested=trace_enabled(graphics);
        if(trace_requested) update_ray_tree();
        const bool tracing=trace_requested && ray_tree_ready;
        const bool local_lights=graphics.enabled(Feature::LocalLights);
        const bool resample=local_lights && graphics.enabled(Feature::ReservoirLights) && !ray_fixtures.empty();
        if(!tracing && !resample) return false;
        if(tracing) frame_features|=graphics_bit(Feature::RayTracing);
        if(resample) frame_features|=graphics_bit(Feature::ReservoirLights);
        const bool trace_fixtures=local_lights && tracing && graphics.enabled(Feature::RayShadows);
        ++ray_frame;
        upload_fixtures(resample || trace_fixtures);
        compute.bind_buffer(storage_buffer,0,ray_buffers[0]);
        compute.bind_buffer(storage_buffer,1,ray_buffers[1]);
        // The compute shaders sample the geometry textures. Their source
        // framebuffer must not be active when a later draw writes the results.
        gl.BindFramebuffer(framebuffer,0);
        const bool use_reservoirs=resample && fixture_count>16;
        if(use_reservoirs){
            const UInt previous=reservoirs[reservoir_index].color;
            reservoir_index^=1;
            ray_inputs_for(reservoir_program);
            sample(reservoir_program,HistoryMap,4,previous);
            gl.Uniform1i(reservoir_program[UseHistory],history_valid && reservoir_history);
            compute.bind_image(0,reservoirs[reservoir_index].color,0,0,0,write_only,rgba32f);
            dispatch_rays();
        }
        reservoir_history=use_reservoirs;
        ray_inputs_for(ray_program);
        sample(ray_program,ReservoirMap,4,reservoirs[reservoir_index].color);
        compute.bind_image(0,ray_raw.color,0,0,0,write_only,rgba16f);
        compute.bind_image(1,ray_raw.aux[0],0,0,0,write_only,rgba16f);
        compute.bind_image(2,ray_raw.aux[1],0,0,0,write_only,rgba16f);
        dispatch_rays();
        for(UInt unit=0;unit<3;++unit) compute.bind_image(unit,0,0,0,0,write_only,rgba16f);
        if(!graphics.enabled(Feature::RayDenoise)){
            ray_history=false;
            return true;
        }
        const bool neural_prior=neural && tracing && graphics.enabled(Feature::RayIndirect) &&
            graphics.enabled(Feature::NeuralRadiance);
        if(neural_prior) update_neural();

        const auto& before=ray_filtered[ray_index];
        ray_index^=1;
        gl.BindFramebuffer(framebuffer,ray_filtered[ray_index].fbo);
        gl.Viewport(0,0,ray_width,ray_height);
        gl.Disable(depth_test);
        screen(ray_filter);
        shader_options(ray_filter,frame_features);
        sample(ray_filter,RaySpecular,0,ray_raw.color);
        sample(ray_filter,RayIndirect,1,ray_raw.aux[0]);
        sample(ray_filter,RayDirect,2,ray_raw.aux[1]);
        sample(ray_filter,PreviousSpecular,3,before.color);
        sample(ray_filter,PreviousIndirect,4,before.aux[0]);
        sample(ray_filter,PreviousDirect,5,before.aux[1]);
        sample(ray_filter,DepthMap,6,gbuffer.depth);
        sample(ray_filter,NormalMap,7,gbuffer.aux[0]);
        sample(ray_filter,HistoryDepth,8,history[history_index].aux[0]);
        matrix(ray_filter,InverseViewProjection,inverse);
        matrix(ray_filter,PreviousViewProjection,previous_projection);
        matrix(ray_filter,PreviousInverseViewProjection,previous_inverse);
        vector(ray_filter,Eye,scene.eye);
        vector(ray_filter,NeuralFocus,neural_focus);
        gl.Uniform1i(ray_filter[UseNeural],neural_prior);
        gl.Uniform2f(ray_filter[PixelSize],1.0f/ray_width,1.0f/ray_height);
        gl.Uniform1i(ray_filter[UseHistory],ray_history && history_valid);
        gl.DrawArrays(triangles_mode,0,3);
        ray_history=true;
        gl.Viewport(0,0,width,height);
        return true;
    }

    void render_lighting(const Target* computed){
        gl.Disable(depth_test);
        gl.BindFramebuffer(framebuffer,hdr.fbo);
        screen(sky);
        sample(sky,DepthMap,3,gbuffer.depth);
        matrix(sky,InverseViewProjection,inverse);
        vector(sky,Eye,scene.eye);
        vector(sky,Sun,scene.sun_direction);
        gl.Uniform1f(sky[Daylight],scene.daylight);
        gl.DrawArrays(triangles_mode,0,3);

        screen(lighting);
        gbuffer_state(lighting);
        lighting_state(lighting);
        gl.Uniform1f(lighting[RayBlend],ray_blend);
        shader_options(lighting,frame_features);
        gl.Uniform1i(lighting[UseRays],computed!=nullptr);
        sample(lighting,RayIndirect,5,computed?computed->aux[0]:0);
        sample(lighting,RayDirect,6,computed?computed->aux[1]:0);
        const auto& vp=scene.view_projection.m;
        const float scale=std::sqrt(vp[1]*vp[1]+vp[5]*vp[5]+vp[9]*vp[9]);
        gl.Uniform1f(lighting[ProjectionScale],0.5f*height*scale);
        gl.DrawArrays(triangles_mode,0,3);
    }

    void render_reflections(){
        gl.BindFramebuffer(framebuffer,reflections.fbo);
        gl.Viewport(0,0,std::max(1,width/2),std::max(1,height/2));
        screen(reflection);
        shader_options(reflection,graphics.features);
        gbuffer_state(reflection);
        sample(reflection,Source,4,hdr.color);
        matrix(reflection,ViewProjection,view_projection);
        vector(reflection,Eye,scene.eye);
        gl.DrawArrays(triangles_mode,0,3);
    }

    void render_transparency(){
        if(translucent.empty()) return;
        gl.BindFramebuffer(framebuffer,oit.fbo);
        gl.Viewport(0,0,width,height);
        gl.ClearColor(0,0,0,0);
        gl.Clear(color_bit);
        gl.Enable(depth_test);
        gl.DepthMask(0);
        // Weighted blending accumulates optical depth. Sorted blending
        // accumulates premultiplied color and alpha in the first attachment.
        gl.Enable(blend);
        const bool weighted=weighted_transparency();
        if(weighted) gl.BlendFunc(one,one);
        else{
            gl.BlendFuncSeparate(src_alpha,inv_src_alpha,one,inv_src_alpha);
            std::stable_sort(translucent.begin(),translucent.end(),
                [](const Draw& a,const Draw& b){ return a.depth>b.depth; });
        }
        gl.UseProgram(transparency.id);
        gl.Uniform1i(transparency[WeightedTransparency],weighted);
        gl.BindVertexArray(mesh_vao);
        matrix(transparency,ViewProjection,view_projection);
        gl.Uniform1i(transparency[Terrain],0);
        sample(transparency,SurfaceMaps,5,surface_maps,texture_array);
        ground_page_state(transparency);
        lighting_state(transparency);
        for(const auto& draw:translucent) gl.DrawArrays(triangles_mode,draw.first,draw.count);
        gl.Disable(blend);
        gl.Disable(depth_test);
        gl.DepthMask(1);
    }

    void render_volumes(){
        if(!volume_count) return;
        gl.BindFramebuffer(framebuffer,volume_target.fbo);
        gl.Viewport(0,0,std::max(1,width/2),std::max(1,height/2));
        screen(volume_program);
        shader_options(volume_program,graphics.features);
        sample(volume_program,DepthMap,0,gbuffer.depth);
        matrix(volume_program,InverseViewProjection,inverse);
        vector(volume_program,Eye,scene.eye);
        vector(volume_program,Sun,scene.sun_direction);
        gl.Uniform1f(volume_program[Daylight],scene.daylight);
        gl.Uniform1f(volume_program[Time],scene.seconds);
        gl.Uniform1i(volume_program[VolumeCount],static_cast<Int>(volume_count));
        std::array<lunar::Float4,8> origin{},axis{},color{},parameters{};
        for(std::size_t i=0;i<volume_count;++i){
            const auto& v=volumes[i];
            const Vec3 direction=normalize(v.axis);
            origin[i]={v.origin.x,v.origin.y,v.origin.z,v.length};
            axis[i]={direction.x,direction.y,direction.z,v.radius};
            color[i]={v.tint.r,v.tint.g,v.tint.b,v.density};
            parameters[i]={v.emission,v.cone?1.0f:0.0f,v.nozzle_radius,v.plume_length};
        }
        const auto count=static_cast<Int>(volume_count);
        gl.Uniform4fv(volume_program[VolumeOrigin],count,origin.front().data());
        gl.Uniform4fv(volume_program[VolumeAxis],count,axis.front().data());
        gl.Uniform4fv(volume_program[VolumeColor],count,color.front().data());
        gl.Uniform4fv(volume_program[VolumeParams],count,parameters.front().data());
        gl.DrawArrays(triangles_mode,0,3);
    }

    UInt resolve_world(UInt reflection_texture){
        if(!temporal_aa() && !ray_history && !reservoir_history && !reflection_texture &&
           translucent.empty() && !volume_count && !(skin_in_scene && graphics.enabled(Feature::Skin)))
            return hdr.color;

        const Target& previous=history[history_index];
        if(history[1].fbo) history_index^=1;
        const Target& current=history[history_index];
        gl.BindFramebuffer(framebuffer,current.fbo);
        gl.Viewport(0,0,history_width,history_height);
        screen(temporal);
        shader_options(temporal,graphics.features);
        gl.Uniform1i(temporal[UseReflection],reflection_texture!=0);
        gl.Uniform1i(temporal[WeightedTransparency],weighted_transparency());
        sample(temporal,Source,0,hdr.color);
        sample(temporal,DepthMap,1,gbuffer.depth);
        sample(temporal,ReflectedMap,2,reflection_texture);
        sample(temporal,AccumulatedMap,3,oit.color);
        sample(temporal,DensityMap,4,oit.aux[0]);
        const bool use_history=temporal_aa() && history_valid;
        sample(temporal,HistoryMap,5,use_history?previous.color:0);
        sample(temporal,HistoryDepth,6,use_history?previous.aux[0]:0);
        sample(temporal,MaterialMap,7,gbuffer.aux[1]);
        sample(temporal,NormalMap,8,gbuffer.aux[0]);
        sample(temporal,VolumeMap,9,volume_count?volume_target.color:0);
        gl.Uniform1i(temporal[UseVolumes],volume_count!=0);
        matrix(temporal,InverseViewProjection,inverse);
        matrix(temporal,PreviousViewProjection,previous_projection);
        matrix(temporal,PreviousInverseViewProjection,previous_inverse);
        vector(temporal,Eye,scene.eye);
        screen_size(temporal);
        gl.Uniform1i(temporal[UseHistory],use_history);
        gl.Uniform1i(temporal[UseTransparency],!translucent.empty());
        gl.DrawArrays(triangles_mode,0,3);
        previous_projection=view_projection;
        previous_inverse=inverse;
        return current.color;
    }

    void render_lines(){
        if(lines.empty()) return;
        gl.BindFramebuffer(framebuffer,overlay.fbo);
        gl.Viewport(0,0,width,height);
        gl.ClearColor(0,0,0,0);
        gl.Clear(color_bit);
        attach_depth(gbuffer.depth);
        gl.Enable(depth_test);
        gl.DepthMask(0);
        gl.Enable(blend);
        gl.BlendFuncSeparate(src_alpha,inv_src_alpha,one,inv_src_alpha);
        gl.UseProgram(line.id);
        gl.BindVertexArray(mesh_vao);
        matrix(line,ViewProjection,view_projection);
        gl.Uniform1i(line[Terrain],0);
        for(const auto& draw:lines){
            gl.LineWidth(std::clamp(draw.width,1.0f,max_line_width));
            gl.DrawArrays(lines_mode,draw.first,draw.count);
        }
        gl.LineWidth(1);
        gl.DepthMask(1);
        gl.Disable(blend);
        gl.Disable(depth_test);
        attach_depth(0);
    }

    void post_process(UInt resolved){
        if(graphics.bloom>0){
            const Int bw=std::max(1,width/bloom_divisor()),bh=std::max(1,height/bloom_divisor());
            gl.Viewport(0,0,bw,bh);
            screen(bloom);
            gl.BindFramebuffer(framebuffer,bloom_a.fbo);
            sample(bloom,Source,0,resolved);
            gl.Uniform1i(bloom[Extract],1);
            gl.Uniform2f(bloom[StepUV],0.90f/width,0.90f/height);
            gl.DrawArrays(triangles_mode,0,3);
            gl.Uniform1i(bloom[Extract],0);
            gl.BindFramebuffer(framebuffer,bloom_b.fbo);
            bind_texture(0,bloom_a.color);
            gl.Uniform2f(bloom[StepUV],1.0f/bw,0);
            gl.DrawArrays(triangles_mode,0,3);
            gl.BindFramebuffer(framebuffer,bloom_a.fbo);
            bind_texture(0,bloom_b.color);
            gl.Uniform2f(bloom[StepUV],0,1.0f/bh);
            gl.DrawArrays(triangles_mode,0,3);
        }

        gl.Viewport(0,0,history_width,history_height);
        gl.BindFramebuffer(framebuffer,ldr.fbo);
        screen(composite);
        sample(composite,Source,0,resolved);
        sample(composite,Glow,1,graphics.bloom>0?bloom_a.color:0);
        shader_options(composite,graphics.features);
        gl.Uniform1f(composite[BloomStrength],0.0043f*graphics.bloom);
        gl.Uniform1f(composite[Exposure],1.05f*std::exp2(0.1f*graphics.exposure_tenths));
        gl.DrawArrays(triangles_mode,0,3);
        render_lines();

        gl.BindFramebuffer(framebuffer,0);
        gl.Viewport(0,0,output_width,output_height);
        screen(fxaa);
        sample(fxaa,Source,0,ldr.color);
        sample(fxaa,OverlayMap,1,lines.empty()?0:overlay.color);
        gl.Uniform1i(fxaa[UseTransparency],!lines.empty());
        const float spatial=graphics.aa==GraphicsAA::Off?0.0f:temporal_aa() && history_valid?0.52f:1.0f;
        gl.Uniform1f(fxaa[SpatialAA],spatial);
        gl.Uniform1i(fxaa[AAQuality],graphics.aa_quality);
        gl.Uniform1f(fxaa[Sharpen],width<history_width?0.22f:0.05f);
        gl.Uniform1i(fxaa[LearnedUpscale],temporal_aa() && graphics.enabled(Feature::LearnedUpscale) && width*1.18f<history_width);
        gl.Uniform2f(fxaa[InputPixelSize],1.0f/width,1.0f/height);
        gl.Uniform2f(fxaa[PixelSize],1.0f/history_width,1.0f/history_height);
        gl.DrawArrays(triangles_mode,0,3);
    }

    void finish(){
        if(!in_scene) return;
        upload();
        gl.Enable(depth_test);
        gl.DepthFunc(less_equal);
        gl.DepthMask(1);
        gl.Disable(blend);
        render_shadows();
        render_shadow_pages();
        update_ground_pages();
        if(auto_path() && visibility_targets && graphics.enabled(Feature::VisibilityBuffer)){
            if(use_clusters()) prepare_cluster_draws();
            render_visibility();
        }else render_geometry();
        // Compute lighting can be unavailable while the asynchronous BVH is
        // building. Only advertise effects with valid output this frame.
        frame_features=graphics.features&~(graphics_bit(Feature::RayTracing)|graphics_bit(Feature::ReservoirLights));
        const Target* computed=nullptr;
        if(render_rays()){
            computed=graphics.enabled(Feature::RayDenoise)?&ray_filtered[ray_index]:&ray_raw;
            ray_blend=std::min(1.0f,ray_blend+frame_seconds*6.0f);
        }else{
            ray_history=reservoir_history=false;
            ray_blend=0.0f;
        }
        render_lighting(computed);
        UInt reflection_texture{};
        if(computed && (frame_features&graphics_bit(Feature::RayTracing)) && graphics.enabled(Feature::RayReflections))
            reflection_texture=computed->color;
        else if(graphics.enabled(Feature::ScreenReflections)){
            render_reflections();
            reflection_texture=reflections.color;
        }
        render_transparency();
        render_volumes();
        post_process(resolve_world(reflection_texture));
        history_valid=true;

        // The HUD and campus use the standard renderer and default VAO.
        gl.BindVertexArray(0);
        gl.BindBuffer(array_buffer,0);
        gl.UseProgram(0);
        for(Int unit:{11,9,5,4}) bind_texture(unit,0,texture_array);
        for(Int unit=12;unit>=0;--unit) bind_texture(unit,0);
        gl.DepthFunc(less);
        gl.Enable(depth_test);
        gl.Enable(blend);
        gl.BlendFunc(src_alpha,inv_src_alpha);
        vertices.clear();
        opaque.clear();
        translucent.clear();
        lines.clear();
        volume_count=0;
        skin_in_scene=false;
        in_scene=false;
    }
};

LunarRenderer::LunarRenderer():impl_(std::make_unique<Impl>()){}
LunarRenderer::~LunarRenderer()=default;
bool LunarRenderer::initialize(const Resolve& resolve){ return impl_->initialize(resolve); }
bool LunarRenderer::begin(const LunarScene& scene,int width,int height){ return impl_->begin(scene,width,height); }
void LunarRenderer::set_graphics_settings(const GraphicsSettings& settings){ impl_->set_graphics_settings(settings); }
GraphicsCapabilities LunarRenderer::graphics_capabilities() const {
    GraphicsCapabilities caps{impl_->ready,impl_->visibility_path,impl_->cluster_path,impl_->page_support,impl_->advanced,impl_->neural};
    caps.texture_anisotropy=impl_->max_anisotropy;
    return caps;
}
bool LunarRenderer::active() const { return impl_->in_scene; }
bool LunarRenderer::volume(const LunarVolume& volume){
    if(!impl_->in_scene || !impl_->graphics.enabled(volume.cone?Feature::VolumetricPlumes:Feature::VolumetricDust) ||
       impl_->volume_count>=impl_->volumes.size() ||
       volume.length<=0.0f || volume.radius<=0.0f || length(volume.axis)<0.001f) return false;
    impl_->volumes[impl_->volume_count++]=volume;
    return true;
}
void LunarRenderer::invalidate_history(){ impl_->reset_history(true); }
bool LunarRenderer::campus_atmosphere(const Mat4& projection,Vec3 eye,float seconds,int width,int height){
    return impl_->campus_atmosphere(projection,eye,seconds,width,height);
}
void LunarRenderer::triangles(const std::vector<Vec3>& vertices,LunarMaterial material,Color tint,
                              const LunarSurfaceMapping& mapping){
    if(material==LunarMaterial::Marking && !impl_->graphics.enabled(Feature::Decals)) return;
    impl_->append(vertices,static_cast<Int>(material),tint,false,1.0f,mapping);
}
void LunarRenderer::lines(const std::vector<Vec3>& vertices,Color tint,float width){
    impl_->append(vertices,line_material,tint,true,width);
}
void LunarRenderer::finish(){ impl_->finish(); }

} // namespace eng
