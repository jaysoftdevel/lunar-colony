#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/ui_font.hpp"
#include "engine/geometry_stream.hpp"
#include "lunar_renderer.hpp"
#include <algorithm>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <dlfcn.h>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <array>
#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>

namespace eng {
using GLenum=unsigned int; using GLboolean=unsigned char; using GLbitfield=unsigned int; using GLvoid=void; using GLbyte=signed char; using GLshort=short; using GLint=int; using GLsizei=int; using GLubyte=unsigned char; using GLushort=unsigned short; using GLuint=unsigned int; using GLfloat=float; using GLdouble=double; using GLchar=char; using GLsizeiptr=long;
using GLXContext=void*; using GLXDrawable=unsigned long;
constexpr GLenum GL_COLOR_BUFFER_BIT=0x00004000, GL_DEPTH_BUFFER_BIT=0x00000100, GL_DEPTH_TEST=0x0B71, GL_BLEND=0x0BE2, GL_SRC_ALPHA=0x0302, GL_ONE_MINUS_SRC_ALPHA=0x0303, GL_LINES=0x0001, GL_TRIANGLES=0x0004, GL_ARRAY_BUFFER=0x8892, GL_DYNAMIC_DRAW=0x88E8, GL_FLOAT=0x1406, GL_FALSE=0, GL_VERTEX_SHADER=0x8B31, GL_FRAGMENT_SHADER=0x8B30, GL_COMPILE_STATUS=0x8B81, GL_LINK_STATUS=0x8B82, GL_VENDOR=0x1F00, GL_RENDERER=0x1F01, GL_VERSION=0x1F02, GL_TEXTURE_2D=0x0DE1, GL_TEXTURE0=0x84C0, GL_RGB=0x1907, GL_UNSIGNED_BYTE=0x1401, GL_LINEAR=0x2601, GL_LINEAR_MIPMAP_LINEAR=0x2703, GL_REPEAT=0x2901, GL_TEXTURE_MIN_FILTER=0x2801, GL_TEXTURE_MAG_FILTER=0x2800, GL_TEXTURE_WRAP_S=0x2802, GL_TEXTURE_WRAP_T=0x2803, GL_UNPACK_ALIGNMENT=0x0CF5;

struct GLApi {
    void* lib=nullptr;
    GLXContext (*glXCreateContext)(Display*,XVisualInfo*,GLXContext,int)=nullptr;
    int (*glXMakeCurrent)(Display*,GLXDrawable,GLXContext)=nullptr;
    int (*glXIsDirect)(Display*,GLXContext)=nullptr;
    void (*glXSwapBuffers)(Display*,GLXDrawable)=nullptr;
    void (*glBindAttribLocation)(GLuint,GLuint,const GLchar*)=nullptr;
    void* (*glXGetProcAddressARB)(const GLubyte*)=nullptr;
    const char* (*glXQueryExtensionsString)(Display*,int)=nullptr;
    void (*glXSwapIntervalEXT)(Display*,GLXDrawable,int)=nullptr;
    int (*glXSwapIntervalMESA)(unsigned int)=nullptr;
    int (*glXSwapIntervalSGI)(int)=nullptr;
    const GLubyte* (*glGetString)(GLenum)=nullptr;
    void (*glClearColor)(GLfloat,GLfloat,GLfloat,GLfloat)=nullptr; void (*glClear)(GLbitfield)=nullptr; void (*glViewport)(GLint,GLint,GLsizei,GLsizei)=nullptr; void (*glEnable)(GLenum)=nullptr; void (*glDisable)(GLenum)=nullptr; void (*glBlendFunc)(GLenum,GLenum)=nullptr; void (*glLineWidth)(GLfloat)=nullptr; void (*glDepthMask)(GLboolean)=nullptr;
    GLuint (*glCreateShader)(GLenum)=nullptr; void (*glShaderSource)(GLuint,GLsizei,const GLchar* const*,const GLint*)=nullptr; void (*glCompileShader)(GLuint)=nullptr; void (*glGetShaderiv)(GLuint,GLenum,GLint*)=nullptr; void (*glGetShaderInfoLog)(GLuint,GLsizei,GLsizei*,GLchar*)=nullptr;
    GLuint (*glCreateProgram)()=nullptr; void (*glAttachShader)(GLuint,GLuint)=nullptr; void (*glLinkProgram)(GLuint)=nullptr; void (*glGetProgramiv)(GLuint,GLenum,GLint*)=nullptr; void (*glGetProgramInfoLog)(GLuint,GLsizei,GLsizei*,GLchar*)=nullptr; void (*glUseProgram)(GLuint)=nullptr;
    void (*glGenBuffers)(GLsizei,GLuint*)=nullptr; void (*glBindBuffer)(GLenum,GLuint)=nullptr; void (*glBufferData)(GLenum,GLsizeiptr,const void*,GLenum)=nullptr; void (*glBufferSubData)(GLenum,GLsizeiptr,GLsizeiptr,const void*)=nullptr; void (*glEnableVertexAttribArray)(GLuint)=nullptr; void (*glVertexAttribPointer)(GLuint,GLint,GLenum,GLboolean,GLsizei,const void*)=nullptr; void (*glDisableVertexAttribArray)(GLuint)=nullptr; void (*glDrawArrays)(GLenum,GLint,GLsizei)=nullptr;
    GLint (*glGetUniformLocation)(GLuint,const GLchar*)=nullptr; void (*glUniform1f)(GLint,GLfloat)=nullptr; void (*glUniformMatrix4fv)(GLint,GLsizei,GLboolean,const GLfloat*)=nullptr; void (*glUniform4f)(GLint,GLfloat,GLfloat,GLfloat,GLfloat)=nullptr;
    void (*glUniform1i)(GLint,GLint)=nullptr;
    void (*glGenTextures)(GLsizei,GLuint*)=nullptr;
    void (*glBindTexture)(GLenum,GLuint)=nullptr;
    void (*glTexImage2D)(GLenum,GLint,GLint,GLsizei,GLsizei,GLint,GLenum,GLenum,const void*)=nullptr;
    void (*glTexParameteri)(GLenum,GLenum,GLint)=nullptr;
    void (*glActiveTexture)(GLenum)=nullptr;
    void (*glPixelStorei)(GLenum,GLint)=nullptr;
    void (*glGenerateMipmap)(GLenum)=nullptr; // optional; fall back to bilinear without mipmaps
    void (*glFinish)()=nullptr;


    template<class T> bool load(T& out,const char* n){ out=(T)dlsym(lib,n); if(!out && glXGetProcAddressARB) out=(T)glXGetProcAddressARB((const GLubyte*)n); return out!=nullptr; }
    bool init(){
        lib=dlopen("libGL.so.1",RTLD_NOW|RTLD_GLOBAL); if(!lib)return false;
        load(glXCreateContext,"glXCreateContext"); load(glXMakeCurrent,"glXMakeCurrent"); load(glXIsDirect,"glXIsDirect"); load(glXSwapBuffers,"glXSwapBuffers"); load(glXGetProcAddressARB,"glXGetProcAddressARB");
        load(glXQueryExtensionsString,"glXQueryExtensionsString");
        load(glXSwapIntervalEXT,"glXSwapIntervalEXT");
        load(glXSwapIntervalMESA,"glXSwapIntervalMESA");
        load(glXSwapIntervalSGI,"glXSwapIntervalSGI");
        bool ok=true;
#define L(x) ok=load(x,#x)&&ok
        L(glGetString);L(glClearColor);L(glClear);L(glViewport);L(glEnable);L(glDisable);L(glBlendFunc);L(glLineWidth);L(glDepthMask);L(glBindAttribLocation);L(glCreateShader);L(glShaderSource);L(glCompileShader);L(glGetShaderiv);L(glGetShaderInfoLog);L(glCreateProgram);L(glAttachShader);L(glLinkProgram);L(glGetProgramiv);L(glGetProgramInfoLog);L(glUseProgram);L(glGenBuffers);L(glBindBuffer);L(glBufferData);L(glBufferSubData);L(glEnableVertexAttribArray);L(glVertexAttribPointer);L(glDisableVertexAttribArray);L(glDrawArrays);L(glGetUniformLocation);L(glUniform1f);L(glUniformMatrix4fv);L(glUniform4f);L(glUniform1i);L(glGenTextures);L(glBindTexture);L(glTexImage2D);L(glTexParameteri);L(glActiveTexture);L(glPixelStorei);
        load(glGenerateMipmap,"glGenerateMipmap");
        load(glFinish,"glFinish");
#undef L
        return ok&&glXCreateContext&&glXMakeCurrent&&glXIsDirect&&glXSwapBuffers;
    }
};

// Deterministic 1024x1024 tileable albedo textures. High-detail assets are
// generated at GPU initialization, so no image library or external download
// is needed and every installed build gets identical materials. The mip chain
// prevents the high-frequency texture shimmer formerly seen during orbit.
constexpr int kCampusTexSize=1024;
constexpr int kCampusTexCount=9;
static std::uint32_t tex_hash(int x,int y,int seed){
    std::uint32_t h=(std::uint32_t)x*0x9e3779b1u ^ (std::uint32_t)y*0x85ebca77u ^ (std::uint32_t)seed*0xc2b2ae3du;
    h^=h>>16;h*=0x7feb352du;h^=h>>15;h*=0x846ca68bu;h^=h>>16;
    return h;
}
static float tex_noise(int x,int y,int grid,int seed){
    const int ix=x*grid/kCampusTexSize, iy=y*grid/kCampusTexSize;
    float fx=(x*grid%kCampusTexSize)/(float)kCampusTexSize;
    float fy=(y*grid%kCampusTexSize)/(float)kCampusTexSize;
    fx=fx*fx*(3.0f-2.0f*fx);fy=fy*fy*(3.0f-2.0f*fy);
    auto h=[&](int a,int b){return (tex_hash((a+grid)%grid,(b+grid)%grid,seed)&65535u)/65535.0f;};
    return (h(ix,iy)*(1-fx)+h(ix+1,iy)*fx)*(1-fy)
          +(h(ix,iy+1)*(1-fx)+h(ix+1,iy+1)*fx)*fy;
}
static std::vector<std::uint8_t> make_campus_texture(int kind){
    std::vector<std::uint8_t> pix((std::size_t)kCampusTexSize*kCampusTexSize*3);
    static constexpr std::array<std::array<float,3>,kCampusTexCount> colors{{
        {{0.25f,0.40f,0.20f}}, // coastal vegetation
        {{0.76f,0.68f,0.47f}}, // dune sand
        {{0.27f,0.29f,0.30f}}, // tarmac
        {{0.57f,0.59f,0.58f}}, // weathered concrete
        {{0.11f,0.38f,0.48f}}, // deep coastal water
        {{0.42f,0.48f,0.53f}}, // corrugated roof
        {{0.58f,0.64f,0.68f}}, // industrial facade
        {{0.17f,0.36f,0.45f}}, // window glass
        {{0.58f,0.60f,0.58f}}, // hardware / coated metal
    }};
    for(int y=0;y<kCampusTexSize;++y)for(int x=0;x<kCampusTexSize;++x){
        float broad=tex_noise(x,y,4,kind+13);
        float mid=tex_noise(x,y,32,kind+47);
        float fine=tex_noise(x,y,256,kind+91);
        float point=(tex_hash(x,y,kind+123)&65535u)/65535.0f;
        float shade=(broad-0.5f)*0.34f+(mid-0.5f)*0.23f+(fine-0.5f)*0.17f+(point-0.5f)*0.10f;
        std::array<float,3> rgb=colors[(std::size_t)kind];
        if(kind==0){ // short grass and sparse sandy/earth patches
            if(broad<0.17f){rgb={{0.28f,0.38f,0.22f}};shade*=0.65f;}
            else if(point>0.965f) shade+=0.19f; // small bright stems
        }else if(kind==1){
            shade+=0.035f*std::sin((float)y*0.12f+6.0f*broad);
            if(point>0.992f)shade+=0.12f;
        }else if(kind==2){
            if(point>0.975f)shade+=0.20f; // embedded aggregate
            if(point<0.014f)shade-=0.22f;
        }else if(kind==3){
            const float seam=(x%256<3 || y%256<3)?-0.20f:0.0f;
            shade+=seam+(point>0.989f?0.09f:0.0f);
        }else if(kind==4){
            shade=0.08f*std::sin((float)y*0.056f+mid*5.0f)+0.045f*std::sin((float)x*0.086f+broad*5.0f);
            if(point>0.998f)shade+=0.10f;
        }else if(kind==5){
            shade+=(x%55<6?0.20f:-0.03f); // steel roof ribs
            if(y%485<3)shade-=0.12f;
        }else if(kind==6){
            if(x%171<3 || y%240<3)shade-=0.16f; // cladding seams
            if(point>0.996f)shade-=0.12f;
        }else if(kind==7){
            if(x%256<7 || y%256<7)shade-=0.18f; // mullions
            if((x+y/3)%291<24)shade+=0.09f; // diffuse sky reflections
        }else if(kind==8){
            if((x%152)<2)shade-=0.13f; // machined seams
            if(broad<0.17f){rgb={{0.52f,0.41f,0.32f}};} // weathering
        }
        const std::size_t index=((std::size_t)y*kCampusTexSize+x)*3;
        for(int ch=0;ch<3;++ch){
            float v=std::clamp(rgb[ch]*(1.0f+shade),0.0f,1.0f);
            pix[index+ch]=(std::uint8_t)(v*255.0f+0.5f);
        }
    }
    return pix;
}

class GLRenderer final: public Renderer {
    GLApi gl_; Display* d_{}; Window w_{}; GLXContext ctx_{}; int width_{},height_{}; GLuint prog_{},material_prog_{},vbo_{}; GLint u_mvp_{},u_color_{},material_mvp_{},material_color_{},material_kind_{},material_tex_{},material_tiling_{}; std::array<GLuint,kCampusTexCount> campus_textures_{}; Mat4 mvp_=Mat4::identity();
    GLsizeiptr vbo_capacity_bytes_{0};
    GeometryStream stream_;
    GraphicsSettings graphics_=graphics_settings_from_environment();
    std::unique_ptr<LunarRenderer> lunar_;
    bool lunar_attempted_{false};
    bool lunar_frame_{false};
    int campus_haze_level_{};
    bool benchmarking_{};
    double benchmark_ms_{};
    std::chrono::steady_clock::time_point benchmark_begin_{};
    Vec3 campus_eye_{};
    GLint campus_haze_{},campus_eye_uniform_{};
    struct MaterialCommand {GLint first{},count{};int kind{};Color tint{};};
    std::vector<float> material_vertices_; // interleaved position + flat face normal
    std::vector<MaterialCommand> material_commands_;

    bool hardware_accelerated_{false};
    std::string gpu_vendor_{};
    std::string gpu_renderer_{};
    std::string gpu_version_{};

    void configure_swap_interval(){
        int requested=0;
        if(const char* env=std::getenv("GAME_VSYNC")){
            if(std::strcmp(env,"1")==0 || std::strcmp(env,"true")==0 || std::strcmp(env,"on")==0) requested=1;
            else requested=0;
        }
        bool applied=false;
        if(gl_.glXSwapIntervalEXT){
            gl_.glXSwapIntervalEXT(d_,w_,requested);
            applied=true;
        }else if(gl_.glXSwapIntervalMESA){
            applied=gl_.glXSwapIntervalMESA((unsigned int)std::max(0,requested))==0;
        }else if(gl_.glXSwapIntervalSGI){
            applied=gl_.glXSwapIntervalSGI(std::max(0,requested))==0;
        }
        if(applied)
            std::cout<<"GPU present interval: "<<(requested==0?"unlimited (vsync off)":"vsync")
                     <<" (override with GAME_VSYNC=0|1)\n";
        else
            std::cout<<"GPU present interval control unavailable; driver default is active\n";
    }

    static bool software_renderer_name(std::string value){
        std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){ return (char)std::tolower(c); });
        static constexpr const char* needles[]={"llvmpipe","softpipe","swrast","software rasterizer","software renderer","mesa offscreen"};
        for(const char* needle:needles) if(value.find(needle)!=std::string::npos) return true;
        return false;
    }
    GLuint shader(GLenum type,const char* src){ GLuint s=gl_.glCreateShader(type); gl_.glShaderSource(s,1,&src,nullptr); gl_.glCompileShader(s); GLint ok=0; gl_.glGetShaderiv(s,GL_COMPILE_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetShaderInfoLog(s,2047,&n,log); std::cerr<<"Shader error: "<<log<<"\n"; } return s; }
    bool ensure_lunar(){
        if(lunar_attempted_) return lunar_!=nullptr;
        lunar_attempted_=true;
        auto candidate=std::make_unique<LunarRenderer>();
        if(!candidate->initialize([this](const char* name){
            void* address=nullptr;
            gl_.load(address,name);
            return address;
        })){
            std::cerr<<"Enhanced graphics require OpenGL 3.3 with framebuffer support; using the standard view.\n";
            return false;
        }
        candidate->set_graphics_settings(graphics_);
        lunar_=std::move(candidate);
        return true;
    }
    void flush_materials(){
        if(material_commands_.empty())return;
        gl_.glUseProgram(material_prog_);
        gl_.glUniformMatrix4fv(material_mvp_,1,GL_FALSE,mvp_.m.data());
        gl_.glUniform1i(material_tex_,0);
        gl_.glUniform1i(campus_haze_,campus_haze_level_);
        gl_.glUniform4f(campus_eye_uniform_,campus_eye_.x,campus_eye_.y,campus_eye_.z,0);
        gl_.glActiveTexture(GL_TEXTURE0);
        gl_.glBindBuffer(GL_ARRAY_BUFFER,vbo_);
        const auto bytes=(GLsizeiptr)(material_vertices_.size()*sizeof(float));
        if(bytes>vbo_capacity_bytes_){vbo_capacity_bytes_=4096;while(vbo_capacity_bytes_<bytes)vbo_capacity_bytes_*=2;}
        gl_.glBufferData(GL_ARRAY_BUFFER,vbo_capacity_bytes_,nullptr,GL_DYNAMIC_DRAW);
        gl_.glBufferSubData(GL_ARRAY_BUFFER,0,bytes,material_vertices_.data());
        gl_.glEnableVertexAttribArray(0);gl_.glEnableVertexAttribArray(1);
        gl_.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),nullptr);
        gl_.glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(const void*)(3*sizeof(float)));
        int last_kind=-1;
        for(const auto& cmd:material_commands_){
            if(last_kind!=cmd.kind){
                gl_.glBindTexture(GL_TEXTURE_2D,campus_textures_[(std::size_t)cmd.kind]);
                gl_.glUniform1f(material_kind_,(float)cmd.kind);
                static constexpr float tiling[]={0.028f,0.065f,0.40f,0.28f,0.085f,0.33f,0.28f,0.30f,0.45f};
                gl_.glUniform1f(material_tiling_,tiling[cmd.kind]);
                last_kind=cmd.kind;
            }
            const auto& c=cmd.tint;
            gl_.glUniform4f(material_color_,c.r,c.g,c.b,c.a);
            gl_.glDepthMask(c.a<0.999f ? GL_FALSE : (GLboolean)1);
            gl_.glDrawArrays(GL_TRIANGLES,cmd.first,cmd.count);
        }
        gl_.glDepthMask((GLboolean)1);
        gl_.glDisableVertexAttribArray(1);
        gl_.glBindTexture(GL_TEXTURE_2D,0);
        gl_.glUseProgram(prog_);
        material_vertices_.clear();material_commands_.clear();
    }
    void flush_geometry(){
        if(stream_.commands.empty()) return;
        gl_.glUseProgram(prog_);
        gl_.glUniformMatrix4fv(u_mvp_,1,GL_FALSE,mvp_.m.data());
        gl_.glBindBuffer(GL_ARRAY_BUFFER,vbo_);
        const GLsizeiptr bytes=(GLsizeiptr)(stream_.vertices.size()*sizeof(float));
        if(bytes>vbo_capacity_bytes_){
            vbo_capacity_bytes_=4096;
            while(vbo_capacity_bytes_<bytes) vbo_capacity_bytes_*=2;
        }
        // Orphan the previous backing once per pass. The driver can finish
        // reading that storage while this frame gets its own upload region.
        gl_.glBufferData(GL_ARRAY_BUFFER,vbo_capacity_bytes_,nullptr,GL_DYNAMIC_DRAW);
        gl_.glBufferSubData(GL_ARRAY_BUFFER,0,bytes,stream_.vertices.data());
        gl_.glEnableVertexAttribArray(0);
        gl_.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),nullptr);
        for(const auto& cmd:stream_.commands){
            const auto c=cmd.color;
            gl_.glUniform4f(u_color_,c.r,c.g,c.b,c.a);
            gl_.glDepthMask(c.a<0.999f ? GL_FALSE : (GLboolean)1);
            gl_.glLineWidth(cmd.width);
            gl_.glDrawArrays(cmd.primitive,(GLint)cmd.first,(GLsizei)cmd.count);
        }
        gl_.glDepthMask((GLboolean)1);
        stream_.clear();
    }
    void upload_and_draw(const float* data,size_t floats,GLenum prim,Color c,float width=1.0f){
        flush_materials();
        stream_.append(data,floats,prim,c,width);
        // Keep pathological worlds bounded without changing draw order.
        if(stream_.vertices.size()*sizeof(float)>=32*1024*1024) flush_geometry();
    }

public:
    bool initialize(void* native_display,std::uintptr_t native_window,int width,int height) override {
        d_=(Display*)native_display; w_=(Window)native_window; width_=width;height_=height; if(!gl_.init()) return false;
        XWindowAttributes wa{}; if(!XGetWindowAttributes(d_,w_,&wa)) return false;
        XVisualInfo templ{}; templ.visualid=XVisualIDFromVisual(wa.visual); int count=0;
        XVisualInfo* vi=XGetVisualInfo(d_,VisualIDMask,&templ,&count); if(!vi||count<1) return false;
        ctx_=gl_.glXCreateContext(d_,vi,nullptr,True); XFree(vi); if(!ctx_||!gl_.glXMakeCurrent(d_,w_,ctx_)) return false;

        const bool direct=gl_.glXIsDirect(d_,ctx_)!=0;
        auto gl_string=[&](GLenum name){
            const auto* value=gl_.glGetString(name);
            return value?std::string(reinterpret_cast<const char*>(value)):std::string{};
        };
        gpu_vendor_=gl_string(GL_VENDOR);
        gpu_renderer_=gl_string(GL_RENDERER);
        gpu_version_=gl_string(GL_VERSION);
        const bool forced_software=[](){
            const char* value=std::getenv("LIBGL_ALWAYS_SOFTWARE");
            return value && *value && std::strcmp(value,"0")!=0;
        }();
        hardware_accelerated_=direct && !forced_software && !software_renderer_name(gpu_renderer_);
        const bool diagnostic_software_allowed=[](){
            const char* value=std::getenv("GAME_ALLOW_SOFTWARE_GL");
            return value && std::strcmp(value,"1")==0;
        }();
        if(!hardware_accelerated_ && !diagnostic_software_allowed){
            std::cerr<<"Hardware OpenGL is required. GLX direct="<<(direct?"yes":"no")
                     <<", vendor='"<<gpu_vendor_<<"', renderer='"<<gpu_renderer_<<"', version='"<<gpu_version_<<"'.\n"
                     <<"Disable LIBGL_ALWAYS_SOFTWARE and install/enable the GPU driver.\n";
            return false;
        }
        std::cout<<"GPU renderer: "<<gpu_vendor_<<" | "<<gpu_renderer_<<" | OpenGL "<<gpu_version_
                 <<(hardware_accelerated_?" (hardware)":" (DIAGNOSTIC SOFTWARE MODE)")<<"\n";
        configure_swap_interval();

        const char* vs="#version 120\nattribute vec3 aPos; uniform mat4 uMVP; void main(){ gl_Position=uMVP*vec4(aPos,1.0); }";
        const char* fs="#version 120\nuniform vec4 uColor; void main(){ gl_FragColor=uColor; }";
        GLuint sv=shader(GL_VERTEX_SHADER,vs), sf=shader(GL_FRAGMENT_SHADER,fs); prog_=gl_.glCreateProgram(); gl_.glAttachShader(prog_,sv); gl_.glAttachShader(prog_,sf); gl_.glBindAttribLocation(prog_,0,"aPos"); gl_.glLinkProgram(prog_); GLint ok=0; gl_.glGetProgramiv(prog_,GL_LINK_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetProgramInfoLog(prog_,2047,&n,log); std::cerr<<"Program link error: "<<log<<"\n"; return false; }
        // Lit, tileable 1k GPU albedo textures replace the earlier single
        // world-space-noise shader. Per-face normals provide stable sun shading.
        const char* mvs=R"GLSL(#version 120
attribute vec3 aPos;attribute vec3 aNormal;
uniform mat4 uMVP;varying vec3 vWorld;varying vec3 vNormal;
void main(){vWorld=aPos;vNormal=aNormal;gl_Position=uMVP*vec4(aPos,1.0);}
)GLSL";
        const char* mfs=R"GLSL(#version 120
uniform vec4 uColor;uniform float uMaterial;uniform float uTiling;
uniform sampler2D uTexture;uniform int uHaze;uniform vec4 uEye;
varying vec3 vWorld;varying vec3 vNormal;
void main(){
    vec3 n=normalize(vNormal);
    if(n.y< -0.92)n.y=1.0; // two-sided terrain geometry
    vec2 uv=(abs(n.y)>0.66)?vWorld.xz:((abs(n.x)>abs(n.z))?vWorld.zy:vWorld.xy);
    uv*=uTiling;
    vec3 tex=texture2D(uTexture,uv).rgb;
    // A restrained albedo blend keeps mission-state highlight colors visible.
    vec3 base=mix(uColor.rgb,tex,uMaterial>5.5&&uMaterial<7.5?0.50:0.68);
    vec3 sun=normalize(vec3(-0.42,0.82,-0.39));
    float light=0.57+0.43*max(dot(n,sun),0.0);
    if(uMaterial>6.5&&uMaterial<7.5){
        light+=pow(max(dot(n,normalize(vec3(0.45,0.18,0.88))),0.0),14.0)*0.13;
    }
    if(uMaterial>3.5&&uMaterial<4.5)light=0.87;
    vec3 lit=clamp(base*light,0.0,1.0);
    if(uHaze!=0){
        float d=max(0.0,length(vWorld-uEye.xyz)-80.0);
        float haze=1.0-exp(-d*0.0013*(float(uHaze)/3.0));
        lit=mix(lit,vec3(0.65,0.76,0.82),haze*0.79);
    }
    gl_FragColor=vec4(lit,uColor.a);
}
)GLSL";
        GLuint mv=shader(GL_VERTEX_SHADER,mvs),mf=shader(GL_FRAGMENT_SHADER,mfs);
        material_prog_=gl_.glCreateProgram();gl_.glAttachShader(material_prog_,mv);gl_.glAttachShader(material_prog_,mf);
        gl_.glBindAttribLocation(material_prog_,0,"aPos");
        gl_.glBindAttribLocation(material_prog_,1,"aNormal");
        gl_.glLinkProgram(material_prog_);
        GLint material_ok=0;gl_.glGetProgramiv(material_prog_,GL_LINK_STATUS,&material_ok);
        if(!material_ok){ char log[2048]{};GLsizei n=0;gl_.glGetProgramInfoLog(material_prog_,2047,&n,log);std::cerr<<"Material shader: "<<log<<"\n";return false; }
        material_mvp_=gl_.glGetUniformLocation(material_prog_,"uMVP");
        material_color_=gl_.glGetUniformLocation(material_prog_,"uColor");
        material_kind_=gl_.glGetUniformLocation(material_prog_,"uMaterial");
        material_tex_=gl_.glGetUniformLocation(material_prog_,"uTexture");
        material_tiling_=gl_.glGetUniformLocation(material_prog_,"uTiling");
        campus_haze_=gl_.glGetUniformLocation(material_prog_,"uHaze");
        campus_eye_uniform_=gl_.glGetUniformLocation(material_prog_,"uEye");
        gl_.glPixelStorei(GL_UNPACK_ALIGNMENT,1);
        gl_.glActiveTexture(GL_TEXTURE0);
        gl_.glGenTextures(kCampusTexCount,campus_textures_.data());
        for(int kind=0;kind<kCampusTexCount;++kind){
            auto pixels=make_campus_texture(kind);
            gl_.glBindTexture(GL_TEXTURE_2D,campus_textures_[(std::size_t)kind]);
            gl_.glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,kCampusTexSize,kCampusTexSize,0,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
            gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
            gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
            gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,gl_.glGenerateMipmap?GL_LINEAR_MIPMAP_LINEAR:GL_LINEAR);
            if(gl_.glGenerateMipmap)gl_.glGenerateMipmap(GL_TEXTURE_2D);
        }
        gl_.glBindTexture(GL_TEXTURE_2D,0);
        std::cout<<"Campus materials: "<<kCampusTexCount<<" tileable 1024px textures"
                 <<(gl_.glGenerateMipmap?" (mipmapped)":" (bilinear fallback)")<<"\n";
        u_mvp_=gl_.glGetUniformLocation(prog_,"uMVP"); u_color_=gl_.glGetUniformLocation(prog_,"uColor"); gl_.glGenBuffers(1,&vbo_); resize(width,height); return true;
    }
    void resize(int w,int h) override { if(lunar_) lunar_->finish(); flush_geometry();flush_materials(); width_=w;height_=h; if(h<1)h=1; gl_.glViewport(0,0,w,h); }
    void begin_frame(Color c) override {
        if(benchmarking_) benchmark_begin_=std::chrono::steady_clock::now();
        if(lunar_ && !lunar_frame_) lunar_->invalidate_history();
        lunar_frame_=false;
        campus_haze_level_=0;
        stream_.clear();material_vertices_.clear();material_commands_.clear(); gl_.glClearColor(c.r,c.g,c.b,c.a); gl_.glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); gl_.glEnable(GL_DEPTH_TEST); gl_.glEnable(GL_BLEND); gl_.glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); }
    void end_frame() override {
        if(lunar_) lunar_->finish();
        flush_geometry();
        flush_materials();
        if(benchmarking_){
            gl_.glFinish();
            benchmark_ms_=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-benchmark_begin_).count();
        }
        gl_.glXSwapBuffers(d_,w_);
    }
    void set_3d(const Mat4& vp) override { if(lunar_) lunar_->finish(); flush_geometry();flush_materials(); mvp_=vp; gl_.glEnable(GL_DEPTH_TEST); }
    void set_2d(float scale=1.0f) override {
        if(lunar_) lunar_->finish();
        flush_geometry();flush_materials();
        scale=std::max(0.01f,scale);
        mvp_=ortho(0,(float)width_/scale,(float)height_/scale,0,-1,1);
        gl_.glDisable(GL_DEPTH_TEST);
    }
    void set_graphics_settings(const GraphicsSettings& settings) override {
        if(settings==graphics_) return;
        graphics_=settings;
        graphics_.normalize();
        if(lunar_) lunar_->set_graphics_settings(graphics_);
    }
    GraphicsCapabilities graphics_capabilities() override {
        return ensure_lunar()?lunar_->graphics_capabilities():GraphicsCapabilities{};
    }
    bool supports_graphics_benchmark() const override { return gl_.glFinish!=nullptr; }
    void set_graphics_benchmark(bool enabled) override { benchmarking_=enabled && supports_graphics_benchmark(); }
    double benchmark_frame_ms() const override { return benchmark_ms_; }
    bool draw_campus_atmosphere(const Mat4& projection,Vec3 eye,float seconds) override {
        if(graphics_.path==GraphicsPath::Classic || !ensure_lunar()) return false;
        flush_geometry();
        flush_materials();
        campus_eye_=eye;
        campus_haze_level_=graphics_.quality(GraphicsFeature::Haze);
        return lunar_->campus_atmosphere(projection,eye,seconds,width_,height_);
    }
    bool begin_lunar_scene(const LunarScene& scene) override {
        if(graphics_.path==GraphicsPath::Classic || !ensure_lunar()) return false;
        flush_geometry();
        flush_materials();
        lunar_frame_=lunar_->begin(scene,width_,height_);
        return lunar_frame_;
    }
    bool lunar_scene_active() const override { return lunar_ && lunar_->active(); }
    bool draw_lunar_volume(const LunarVolume& volume) override {
        return lunar_ && lunar_->volume(volume);
    }
    void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial material,Color c) override {
        if(lunar_scene_active()) lunar_->triangles(v,material,c);
        else draw_triangles_3d(v,c);
    }
    void draw_lines_3d(const std::vector<Vec3>& v,Color c,float width) override {
        if(v.empty()) return;
        if(lunar_scene_active()) lunar_->lines(v,c,width);
        else upload_and_draw((const float*)v.data(),v.size()*3,GL_LINES,c,width);
    }
    void draw_triangles_3d(const std::vector<Vec3>& v,Color c) override {
        if(v.empty()) return;
        if(lunar_scene_active()) lunar_->triangles(v,LunarMaterial::Paint,c);
        else upload_and_draw((const float*)v.data(),v.size()*3,GL_TRIANGLES,c,1);
    }
    void draw_material_triangles_3d(const std::vector<Vec3>& v,int kind,Color c) override {
        if(graphics_.path==GraphicsPath::Classic){
            draw_triangles_3d(v,c);
            return;
        }
        if(v.empty())return;
        flush_geometry();
        const int material=std::clamp(kind,0,kCampusTexCount-1);
        const auto first=(GLint)(material_vertices_.size()/6);
        material_vertices_.reserve(material_vertices_.size()+v.size()*6);
        for(std::size_t i=0;i+2<v.size();i+=3){
            const Vec3 n=normalize(cross(v[i+1]-v[i],v[i+2]-v[i]));
            for(std::size_t j=i;j<i+3;++j){
                const auto& p=v[j];
                material_vertices_.insert(material_vertices_.end(),{p.x,p.y,p.z,n.x,n.y,n.z});
            }
        }
        const GLint count=(GLint)(material_vertices_.size()/6)-first;
        if(count==0)return;
        if(!material_commands_.empty()){
            auto& last=material_commands_.back();
            if(last.kind==material && last.tint.r==c.r && last.tint.g==c.g &&
               last.tint.b==c.b && last.tint.a==c.a && last.first+last.count==first){
                last.count+=count;return;
            }
        }
        material_commands_.push_back({first,count,material,c});
        if(material_vertices_.size()>2*1024*1024)flush_materials();
    }
    void draw_rect(Rect r,Color c) override { float v[]={r.x,r.y,0,r.x+r.w,r.y,0,r.x+r.w,r.y+r.h,0, r.x,r.y,0,r.x+r.w,r.y+r.h,0,r.x,r.y+r.h,0}; upload_and_draw(v,18,GL_TRIANGLES,c); }
    void draw_line(Vec2 a,Vec2 b,Color c,float width) override { const auto v=ui::line_quad(a,b,width); upload_and_draw(v.data(),v.size(),GL_TRIANGLES,c); }
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override;
    int width() const override{return width_;} int height() const override{return height_;}
    bool hardware_accelerated() const override{return hardware_accelerated_;}
    std::string gpu_vendor() const override{return gpu_vendor_;}
    std::string gpu_renderer() const override{return gpu_renderer_;}
    std::string gpu_version() const override{return gpu_version_;}
};


void GLRenderer::draw_text(Vec2 p,const std::string& text,float scale,Color c){
    // Batch the complete bitmap string into one GPU submission. The old path
    // called draw_rect() once per lit 5x7 font pixel, causing thousands of
    // glBufferData + glDrawArrays calls in a normal HUD frame. Geometry stays
    // GPU-rendered; only the submission granularity changes.
    std::vector<float> vertices;
    vertices.reserve(text.size()*7u*5u*18u/2u);
    const float origin_x=p.x;
    auto quad=[&](float x,float y){
        const float x1=x+scale,y1=y+scale;
        const float q[]={x,y,0,x1,y,0,x1,y1,0, x,y,0,x1,y1,0,x,y1,0};
        vertices.insert(vertices.end(),q,q+18);
    };
    for(char raw:text){
        char ch=(raw>='a'&&raw<='z')?(char)(raw-'a'+'A'):raw;
        if(ch=='\n'){ p.y+=8*scale; p.x=origin_x; continue; }
        auto it=ui::kBitmapFont.find(ch);
        if(it==ui::kBitmapFont.end()){ p.x+=6*scale; continue; }
        const auto& rows=it->second;
        for(int y=0;y<7;++y) for(int x=0;x<5;++x)
            if(rows[y]&(1<<(4-x))) quad(p.x+x*scale,p.y+y*scale);
        p.x+=6*scale;
    }
    if(!vertices.empty()) upload_and_draw(vertices.data(),vertices.size(),GL_TRIANGLES,c,1.0f);
}
std::unique_ptr<Renderer> create_gl_renderer(){ return std::make_unique<GLRenderer>(); }
}
