#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/geometry_stream.hpp"
#include "lunar_renderer.hpp"
#include "surface_field_shaders.hpp"
#include "ui_batch.hpp"
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
constexpr GLenum GL_COLOR_BUFFER_BIT=0x00004000, GL_DEPTH_BUFFER_BIT=0x00000100, GL_DEPTH_TEST=0x0B71, GL_BLEND=0x0BE2, GL_SRC_ALPHA=0x0302, GL_ONE_MINUS_SRC_ALPHA=0x0303, GL_LINES=0x0001, GL_TRIANGLES=0x0004, GL_ARRAY_BUFFER=0x8892, GL_DYNAMIC_DRAW=0x88E8, GL_FLOAT=0x1406, GL_FALSE=0, GL_VERTEX_SHADER=0x8B31, GL_FRAGMENT_SHADER=0x8B30, GL_COMPILE_STATUS=0x8B81, GL_LINK_STATUS=0x8B82, GL_VENDOR=0x1F00, GL_RENDERER=0x1F01, GL_VERSION=0x1F02;
constexpr GLenum GL_UNSIGNED_BYTE=0x1401, GL_ALPHA=0x1906, GL_ALPHA8=0x803C, GL_TEXTURE0=0x84C0,
    GL_TEXTURE_2D=0x0DE1, GL_TEXTURE_MIN_FILTER=0x2801, GL_TEXTURE_MAG_FILTER=0x2800, GL_LINEAR=0x2601;

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
    void (*glFinish)()=nullptr;
    void (*glGenTextures)(GLsizei,GLuint*)=nullptr;
    void (*glActiveTexture)(GLenum)=nullptr;
    void (*glBindTexture)(GLenum,GLuint)=nullptr;
    void (*glTexImage2D)(GLenum,GLint,GLint,GLsizei,GLsizei,GLint,GLenum,GLenum,const void*)=nullptr;
    void (*glTexParameteri)(GLenum,GLenum,GLint)=nullptr;


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
        L(glGetString);L(glClearColor);L(glClear);L(glViewport);L(glEnable);L(glDisable);L(glBlendFunc);L(glLineWidth);L(glDepthMask);L(glBindAttribLocation);L(glCreateShader);L(glShaderSource);L(glCompileShader);L(glGetShaderiv);L(glGetShaderInfoLog);L(glCreateProgram);L(glAttachShader);L(glLinkProgram);L(glGetProgramiv);L(glGetProgramInfoLog);L(glUseProgram);L(glGenBuffers);L(glBindBuffer);L(glBufferData);L(glBufferSubData);L(glEnableVertexAttribArray);L(glVertexAttribPointer);L(glDisableVertexAttribArray);L(glDrawArrays);L(glGetUniformLocation);L(glUniform1f);L(glUniformMatrix4fv);L(glUniform4f);L(glUniform1i);
        load(glFinish,"glFinish");
        L(glGenTextures);L(glActiveTexture);L(glBindTexture);L(glTexImage2D);L(glTexParameteri);
#undef L
        return ok&&glXCreateContext&&glXMakeCurrent&&glXIsDirect&&glXSwapBuffers;
    }
};

constexpr int kCampusMaterialCount=9;

class GLRenderer final: public Renderer {
    GLApi gl_; Display* d_{}; Window w_{}; GLXContext ctx_{}; int width_{},height_{}; GLuint prog_{},material_prog_{},vbo_{}; GLint u_mvp_{},u_color_{},material_mvp_{},material_color_{},material_kind_{}; Mat4 mvp_=Mat4::identity();
    GLsizeiptr vbo_capacity_bytes_{0};
    GeometryStream stream_;
    ui::Batch ui_batch_;
    GLuint ui_program_{},ui_atlas_{},ui_buffer_{};
    GLint ui_mvp_{},ui_sampler_{};
    float ui_scale_{1.0f};
    bool ui_mode_{};
    GraphicsSettings graphics_=graphics_settings_from_environment();
    std::unique_ptr<LunarRenderer> lunar_;
    bool lunar_attempted_{false};
    bool lunar_frame_{false};
    int campus_haze_level_{};
    bool benchmarking_{},vsync_supported_{};
    double benchmark_ms_{};
    std::chrono::steady_clock::time_point benchmark_begin_{};
    Vec3 campus_eye_{};
    GLint campus_haze_{},campus_eye_uniform_{};
    enum class MaterialPass { Campus, Basic, Terrain };
    GLint material_pass_{};
    struct MaterialCommand {GLint first{},count{};int kind{};MaterialPass pass{};Color tint{};};
    std::vector<float> material_vertices_; // interleaved position + flat face normal
    std::vector<MaterialCommand> material_commands_;

    bool hardware_accelerated_{false};
    std::string gpu_vendor_{};
    std::string gpu_renderer_{};
    std::string gpu_version_{};

    void configure_swap_interval(){
        if(!vsync_supported_)return;
        const int interval=graphics_.vertical_sync && !benchmarking_?1:0;
        if(gl_.glXSwapIntervalEXT)gl_.glXSwapIntervalEXT(d_,w_,interval);
        else if(gl_.glXSwapIntervalMESA(static_cast<unsigned>(interval))!=0)vsync_supported_=false;
    }

    static bool software_renderer_name(std::string value){
        std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){ return (char)std::tolower(c); });
        static constexpr const char* needles[]={"llvmpipe","softpipe","swrast","software rasterizer","software renderer","mesa offscreen"};
        for(const char* needle:needles) if(value.find(needle)!=std::string::npos) return true;
        return false;
    }
    GLuint shader(GLenum type,const char* src){ GLuint s=gl_.glCreateShader(type); gl_.glShaderSource(s,1,&src,nullptr); gl_.glCompileShader(s); GLint ok=0; gl_.glGetShaderiv(s,GL_COMPILE_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetShaderInfoLog(s,2047,&n,log); std::cerr<<"Shader error: "<<log<<"\n"; } return s; }
    void initialize_ui(){
        constexpr char vertex[]=R"GLSL(#version 120
attribute vec2 position;
attribute vec2 uv;
attribute vec4 color;
uniform mat4 projection;
varying vec2 vUV;
varying vec4 vColor;
void main(){ vUV=uv; vColor=color; gl_Position=projection*vec4(position,0.0,1.0); }
)GLSL";
        constexpr char fragment[]=R"GLSL(#version 120
uniform sampler2D fontAtlas;
varying vec2 vUV;
varying vec4 vColor;
void main(){
    float coverage=1.0;
    if(vUV.x>=0.0){
        float distanceToEdge=texture2D(fontAtlas,vUV).a;
        float edge=clamp(fwidth(distanceToEdge)*0.55,0.017,0.28);
        coverage=smoothstep(0.49-edge,0.49+edge,distanceToEdge);
    }
    gl_FragColor=vec4(vColor.rgb,vColor.a*coverage);
}
)GLSL";
        const auto vs=shader(GL_VERTEX_SHADER,vertex),fs=shader(GL_FRAGMENT_SHADER,fragment);
        const auto program=gl_.glCreateProgram();
        gl_.glAttachShader(program,vs);
        gl_.glAttachShader(program,fs);
        gl_.glBindAttribLocation(program,0,"position");
        gl_.glBindAttribLocation(program,1,"uv");
        gl_.glBindAttribLocation(program,2,"color");
        gl_.glLinkProgram(program);
        GLint ok{};
        gl_.glGetProgramiv(program,GL_LINK_STATUS,&ok);
        if(!ok) return;

        gl_.glGenBuffers(1,&ui_buffer_);
        gl_.glGenTextures(1,&ui_atlas_);
        gl_.glActiveTexture(GL_TEXTURE0);
        gl_.glBindTexture(GL_TEXTURE_2D,ui_atlas_);
        gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        gl_.glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        gl_.glTexImage2D(GL_TEXTURE_2D,0,GL_ALPHA8,ui::atlas_width,ui::atlas_height,0,
                         GL_ALPHA,GL_UNSIGNED_BYTE,ui::font_atlas().data());
        gl_.glBindTexture(GL_TEXTURE_2D,0);
        ui_mvp_=gl_.glGetUniformLocation(program,"projection");
        ui_sampler_=gl_.glGetUniformLocation(program,"fontAtlas");
        ui_program_=program;
    }
    void flush_ui(){
        auto& vertices=ui_batch_.vertices;
        if(vertices.empty()) return;
        gl_.glUseProgram(ui_program_);
        gl_.glUniformMatrix4fv(ui_mvp_,1,GL_FALSE,mvp_.m.data());
        gl_.glUniform1i(ui_sampler_,0);
        gl_.glActiveTexture(GL_TEXTURE0);
        gl_.glBindTexture(GL_TEXTURE_2D,ui_atlas_);
        gl_.glBindBuffer(GL_ARRAY_BUFFER,ui_buffer_);
        gl_.glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(vertices.size()*sizeof(ui::Vertex)),vertices.data(),GL_DYNAMIC_DRAW);
        for(GLuint i:{0u,1u,2u}) gl_.glEnableVertexAttribArray(i);
        gl_.glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,sizeof(ui::Vertex),nullptr);
        gl_.glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,uv)));
        gl_.glVertexAttribPointer(2,4,GL_UNSIGNED_BYTE,1,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,color)));
        gl_.glDrawArrays(GL_TRIANGLES,0,static_cast<GLsizei>(vertices.size()));
        gl_.glDisableVertexAttribArray(2);
        gl_.glDisableVertexAttribArray(1);
        gl_.glBindTexture(GL_TEXTURE_2D,0);
        vertices.clear();
    }
    bool ensure_lunar(){
        if(lunar_attempted_) return lunar_!=nullptr;
        lunar_attempted_=true;
        auto candidate=std::make_unique<LunarRenderer>();
        candidate->set_graphics_settings(graphics_);
        if(!candidate->initialize([this](const char* name){
            void* address=nullptr;
            gl_.load(address,name);
            return address;
        })){
            std::cerr<<"Enhanced graphics require OpenGL 3.3 with framebuffer support; using the standard view.\n";
            return false;
        }
        lunar_=std::move(candidate);
        return true;
    }
    void flush_materials(){
        if(material_commands_.empty())return;
        gl_.glUseProgram(material_prog_);
        gl_.glUniformMatrix4fv(material_mvp_,1,GL_FALSE,mvp_.m.data());
        gl_.glUniform1i(campus_haze_,campus_haze_level_);
        gl_.glUniform4f(campus_eye_uniform_,campus_eye_.x,campus_eye_.y,campus_eye_.z,0);
        gl_.glBindBuffer(GL_ARRAY_BUFFER,vbo_);
        const auto bytes=(GLsizeiptr)(material_vertices_.size()*sizeof(float));
        if(bytes>vbo_capacity_bytes_){vbo_capacity_bytes_=4096;while(vbo_capacity_bytes_<bytes)vbo_capacity_bytes_*=2;}
        gl_.glBufferData(GL_ARRAY_BUFFER,vbo_capacity_bytes_,nullptr,GL_DYNAMIC_DRAW);
        gl_.glBufferSubData(GL_ARRAY_BUFFER,0,bytes,material_vertices_.data());
        gl_.glEnableVertexAttribArray(0);gl_.glEnableVertexAttribArray(1);
        gl_.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),nullptr);
        gl_.glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(const void*)(3*sizeof(float)));
        int last_kind=-1,last_pass=-1;
        for(const auto& cmd:material_commands_){
            const int pass=static_cast<int>(cmd.pass);
            if(last_pass!=pass){
                gl_.glUniform1i(material_pass_,pass);
                last_pass=pass;
            }
            if(last_kind!=cmd.kind){
                gl_.glUniform1f(material_kind_,(float)cmd.kind);
                last_kind=cmd.kind;
            }
            const auto& c=cmd.tint;
            gl_.glUniform4f(material_color_,c.r,c.g,c.b,c.a);
            gl_.glDepthMask(c.a<0.999f ? GL_FALSE : (GLboolean)1);
            gl_.glDrawArrays(GL_TRIANGLES,cmd.first,cmd.count);
        }
        gl_.glDepthMask((GLboolean)1);
        gl_.glDisableVertexAttribArray(1);
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
        flush_ui();
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
        const int screen=vi->screen;
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
        const char* extensions=gl_.glXQueryExtensionsString?gl_.glXQueryExtensionsString(d_,screen):nullptr;
        const std::string advertised=extensions?" "+std::string(extensions)+" ":std::string{};
        if(advertised.find(" GLX_EXT_swap_control ")==std::string::npos)gl_.glXSwapIntervalEXT=nullptr;
        if(advertised.find(" GLX_MESA_swap_control ")==std::string::npos)gl_.glXSwapIntervalMESA=nullptr;
        vsync_supported_=gl_.glXSwapIntervalEXT || gl_.glXSwapIntervalMESA;
        configure_swap_interval();
        // SGI can enable sync at startup but cannot switch it off reliably.
        if(!vsync_supported_ && graphics_.vertical_sync && gl_.glXSwapIntervalSGI &&
           advertised.find(" GLX_SGI_swap_control ")!=std::string::npos)
            gl_.glXSwapIntervalSGI(1);

        const char* vs="#version 120\nattribute vec3 aPos; uniform mat4 uMVP; void main(){ gl_Position=uMVP*vec4(aPos,1.0); }";
        const char* fs="#version 120\nuniform vec4 uColor; void main(){ gl_FragColor=uColor; }";
        GLuint sv=shader(GL_VERTEX_SHADER,vs), sf=shader(GL_FRAGMENT_SHADER,fs); prog_=gl_.glCreateProgram(); gl_.glAttachShader(prog_,sv); gl_.glAttachShader(prog_,sf); gl_.glBindAttribLocation(prog_,0,"aPos"); gl_.glLinkProgram(prog_); GLint ok=0; gl_.glGetProgramiv(prog_,GL_LINK_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetProgramInfoLog(prog_,2047,&n,log); std::cerr<<"Program link error: "<<log<<"\n"; return false; }
        // Campus and basic lunar materials share the world-space surface fields.
        const char* mvs=R"GLSL(#version 120
attribute vec3 aPos;attribute vec3 aNormal;
uniform mat4 uMVP;varying vec3 vWorld;varying vec3 vNormal;
void main(){vWorld=aPos;vNormal=aNormal;gl_Position=uMVP*vec4(aPos,1.0);}
)GLSL";
        const std::string mfs=std::string(R"GLSL(#version 120
uniform vec4 uColor;uniform float uMaterial;uniform int uPass;
uniform int uHaze;uniform vec4 uEye;
varying vec3 vWorld;varying vec3 vNormal;
)GLSL")+surface_shaders::fields+surface_shaders::basic_material+R"GLSL(
void main(){
    vec3 n=normalize(vNormal);
    float footprint=max(length(dFdx(vWorld)),length(dFdy(vWorld)));
    if(uPass!=0){
        float tone=basicSurfaceTone(vWorld,n,uMaterial,uPass==2,footprint);
        gl_FragColor=vec4(clamp(uColor.rgb*tone*basicSurfaceLight(n),0.0,1.0),uColor.a);
        return;
    }
    if(n.y< -0.92)n.y=1.0; // two-sided terrain geometry
    vec2 uv=(abs(n.y)>0.66)?vWorld.xz:((abs(n.x)>abs(n.z))?vWorld.zy:vWorld.xy);
    float macro=filteredNoise(turnSurface(uv)*0.13+uMaterial*13.7,footprint*0.13);
    float tone=coatingField(uv,uMaterial<2.0?footprint*0.5:footprint,uMaterial*31.3);
    if(uMaterial<2.0) tone=mix(tone,macro,0.68);
    if(uMaterial>3.5 && uMaterial<4.5){
        vec2 ripples=turnSurface(uv)*vec2(0.24,1.2);
        tone=mix(macro,filteredNoise(ripples,footprint*1.2),0.5);
    }
    vec3 base=uColor.rgb*(0.79+0.29*tone+0.13*macro);
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
        GLuint mv=shader(GL_VERTEX_SHADER,mvs),mf=shader(GL_FRAGMENT_SHADER,mfs.c_str());
        material_prog_=gl_.glCreateProgram();gl_.glAttachShader(material_prog_,mv);gl_.glAttachShader(material_prog_,mf);
        gl_.glBindAttribLocation(material_prog_,0,"aPos");
        gl_.glBindAttribLocation(material_prog_,1,"aNormal");
        gl_.glLinkProgram(material_prog_);
        GLint material_ok=0;gl_.glGetProgramiv(material_prog_,GL_LINK_STATUS,&material_ok);
        if(!material_ok){ char log[2048]{};GLsizei n=0;gl_.glGetProgramInfoLog(material_prog_,2047,&n,log);std::cerr<<"Material shader: "<<log<<"\n";return false; }
        material_mvp_=gl_.glGetUniformLocation(material_prog_,"uMVP");
        material_color_=gl_.glGetUniformLocation(material_prog_,"uColor");
        material_kind_=gl_.glGetUniformLocation(material_prog_,"uMaterial");
        material_pass_=gl_.glGetUniformLocation(material_prog_,"uPass");
        campus_haze_=gl_.glGetUniformLocation(material_prog_,"uHaze");
        campus_eye_uniform_=gl_.glGetUniformLocation(material_prog_,"uEye");
        u_mvp_=gl_.glGetUniformLocation(prog_,"uMVP"); u_color_=gl_.glGetUniformLocation(prog_,"uColor"); gl_.glGenBuffers(1,&vbo_);
        initialize_ui();
        resize(width,height);
        return true;
    }
    void resize(int w,int h) override { if(lunar_) lunar_->finish(); flush_geometry();flush_materials();flush_ui(); ui_mode_=false; width_=w;height_=h; if(h<1)h=1; gl_.glViewport(0,0,w,h); }
    void begin_frame(Color c) override {
        if(benchmarking_) benchmark_begin_=std::chrono::steady_clock::now();
        if(lunar_ && !lunar_frame_) lunar_->invalidate_history();
        lunar_frame_=false;
        campus_haze_level_=0;
        ui_mode_=false;
        ui_batch_.vertices.clear();
        stream_.clear();material_vertices_.clear();material_commands_.clear(); gl_.glClearColor(c.r,c.g,c.b,c.a); gl_.glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); gl_.glEnable(GL_DEPTH_TEST); gl_.glEnable(GL_BLEND); gl_.glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); }
    void end_frame() override {
        if(lunar_) lunar_->finish();
        flush_geometry();
        flush_materials();
        flush_ui();
        if(benchmarking_){
            gl_.glFinish();
            benchmark_ms_=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-benchmark_begin_).count();
        }
        gl_.glXSwapBuffers(d_,w_);
    }
    void set_3d(const Mat4& vp) override { if(lunar_) lunar_->finish(); flush_geometry();flush_materials();flush_ui(); ui_mode_=false; mvp_=vp; gl_.glEnable(GL_DEPTH_TEST); }
    void set_2d(float scale=1.0f) override {
        scale=std::max(0.01f,scale);
        if(ui_mode_ && scale==ui_scale_) return;
        if(lunar_) lunar_->finish();
        flush_geometry();flush_materials();flush_ui();
        ui_mode_=true;
        ui_scale_=scale;
        mvp_=ortho(0,(float)width_/scale,(float)height_/scale,0,-1,1);
        gl_.glDisable(GL_DEPTH_TEST);
    }
    void set_graphics_settings(const GraphicsSettings& settings) override {
        if(settings==graphics_) return;
        const bool sync_changed=settings.vertical_sync!=graphics_.vertical_sync;
        graphics_=settings;
        graphics_.normalize();
        if(sync_changed)configure_swap_interval();
        if(lunar_) lunar_->set_graphics_settings(graphics_);
    }
    GraphicsCapabilities graphics_capabilities() override {
        auto caps=ensure_lunar()?lunar_->graphics_capabilities():GraphicsCapabilities{};
        caps.vertical_sync=vsync_supported_;
        return caps;
    }
    bool supports_graphics_benchmark() const override { return gl_.glFinish!=nullptr; }
    void set_graphics_benchmark(bool enabled) override {
        enabled=enabled && supports_graphics_benchmark();
        if(benchmarking_==enabled)return;
        benchmarking_=enabled;
        configure_swap_interval();
    }
    double benchmark_frame_ms() const override { return benchmark_ms_; }
    bool draw_campus_atmosphere(const Mat4& projection,Vec3 eye,float seconds) override {
        if(graphics_.path==GraphicsPath::Classic || !ensure_lunar()) return false;
        flush_geometry();
        flush_materials();
        flush_ui();
        campus_eye_=eye;
        campus_haze_level_=graphics_.quality(GraphicsFeature::Haze);
        return lunar_->campus_atmosphere(projection,eye,seconds,width_,height_);
    }
    bool begin_lunar_scene(const LunarScene& scene) override {
        if(graphics_.path==GraphicsPath::Classic || !ensure_lunar()) return false;
        flush_geometry();
        flush_materials();
        flush_ui();
        lunar_frame_=lunar_->begin(scene,width_,height_);
        return lunar_frame_;
    }
    bool lunar_scene_active() const override { return lunar_ && lunar_->active(); }
    bool draw_lunar_volume(const LunarVolume& volume) override {
        return lunar_ && lunar_->volume(volume);
    }
    void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial material,Color c) override {
        if(lunar_scene_active()) lunar_->triangles(v,material,c);
        else if(material==LunarMaterial::Emissive || material==LunarMaterial::Marking) draw_triangles_3d(v,c);
        else queue_material(v,static_cast<int>(material),c,material==LunarMaterial::Regolith?MaterialPass::Terrain:MaterialPass::Basic);
    }
    void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial material,Color c,
                                  const LunarSurfaceMapping& mapping) override {
        if(lunar_scene_active()) lunar_->triangles(v,material,c,mapping);
        else draw_lunar_surface(v,material,c);
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
        const auto pass=graphics_.path!=GraphicsPath::Classic?MaterialPass::Campus:
            kind<2?MaterialPass::Terrain:MaterialPass::Basic;
        queue_material(v,std::clamp(kind,0,kCampusMaterialCount-1),c,pass);
    }
private:
    void queue_material(const std::vector<Vec3>& v,int material,Color c,MaterialPass pass){
        if(v.empty())return;
        flush_ui();
        flush_geometry();
        const auto first=(GLint)(material_vertices_.size()/6);
        const auto required=material_vertices_.size()+v.size()*6;
        if(required>material_vertices_.capacity())
            material_vertices_.reserve(std::max(required,2*material_vertices_.capacity()));
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
            if(last.kind==material && last.pass==pass && last.tint.r==c.r && last.tint.g==c.g &&
               last.tint.b==c.b && last.tint.a==c.a && last.first+last.count==first){
                last.count+=count;
            }else{
                material_commands_.push_back({first,count,material,pass,c});
            }
        }else material_commands_.push_back({first,count,material,pass,c});
        if(material_vertices_.size()>2*1024*1024)flush_materials();
    }
public:
    void draw_rect(Rect r,Color c) override { draw_gradient_rect(r,c,c); }
    void draw_gradient_rect(Rect r,Color top,Color bottom) override {
        if(ui_mode_ && ui_program_){
            flush_geometry();flush_materials();
            ui_batch_.quad(r,top,bottom);
        }else{
            const float v[]={r.x,r.y,0,r.x+r.w,r.y,0,r.x+r.w,r.y+r.h,0, r.x,r.y,0,r.x+r.w,r.y+r.h,0,r.x,r.y+r.h,0};
            upload_and_draw(v,18,GL_TRIANGLES,top);
        }
    }
    void draw_line(Vec2 a,Vec2 b,Color c,float width) override {
        if(ui_mode_ && ui_program_){
            flush_geometry();flush_materials();
            ui_batch_.stroke(a,b,c,width);
        }else{
            const auto v=ui::line_quad(a,b,width);
            upload_and_draw(v.data(),v.size(),GL_TRIANGLES,c);
        }
    }
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override;
    int width() const override{return width_;} int height() const override{return height_;}
    bool hardware_accelerated() const override{return hardware_accelerated_;}
    std::string gpu_vendor() const override{return gpu_vendor_;}
    std::string gpu_renderer() const override{return gpu_renderer_;}
    std::string gpu_version() const override{return gpu_version_;}
};


void GLRenderer::draw_text(Vec2 p,const std::string& text,float scale,Color c){
    if(ui_mode_ && ui_program_){
        flush_geometry();flush_materials();
        ui_batch_.text(p,text,scale,c);
    }else{
        const auto vertices=ui::bitmap_text(p,text,scale);
        upload_and_draw(vertices.data(),vertices.size(),GL_TRIANGLES,c);
    }
}
std::unique_ptr<Renderer> create_gl_renderer(){ return std::make_unique<GLRenderer>(); }
}
