#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/ui_font.hpp"
#include "engine/geometry_stream.hpp"
#include "lunar_renderer.hpp"
#include "surface_field_shaders.hpp"
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
#undef L
        return ok&&glXCreateContext&&glXMakeCurrent&&glXIsDirect&&glXSwapBuffers;
    }
};

constexpr int kCampusMaterialCount=9;

class GLRenderer final: public Renderer {
    GLApi gl_; Display* d_{}; Window w_{}; GLXContext ctx_{}; int width_{},height_{}; GLuint prog_{},material_prog_{},vbo_{}; GLint u_mvp_{},u_color_{},material_mvp_{},material_color_{},material_kind_{}; Mat4 mvp_=Mat4::identity();
    GLsizeiptr vbo_capacity_bytes_{0};
    GeometryStream stream_;
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
    struct MaterialCommand {GLint first{},count{};int kind{};Color tint{};};
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
        int last_kind=-1;
        for(const auto& cmd:material_commands_){
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
        // Campus materials share the filtered, world-anchored surface fields.
        const char* mvs=R"GLSL(#version 120
attribute vec3 aPos;attribute vec3 aNormal;
uniform mat4 uMVP;varying vec3 vWorld;varying vec3 vNormal;
void main(){vWorld=aPos;vNormal=aNormal;gl_Position=uMVP*vec4(aPos,1.0);}
)GLSL";
        const std::string mfs=std::string(R"GLSL(#version 120
uniform vec4 uColor;uniform float uMaterial;
uniform int uHaze;uniform vec4 uEye;
varying vec3 vWorld;varying vec3 vNormal;
)GLSL")+surface_shaders::fields+R"GLSL(
void main(){
    vec3 n=normalize(vNormal);
    if(n.y< -0.92)n.y=1.0; // two-sided terrain geometry
    vec2 uv=(abs(n.y)>0.66)?vWorld.xz:((abs(n.x)>abs(n.z))?vWorld.zy:vWorld.xy);
    float footprint=max(length(dFdx(vWorld)),length(dFdy(vWorld)));
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
        campus_haze_=gl_.glGetUniformLocation(material_prog_,"uHaze");
        campus_eye_uniform_=gl_.glGetUniformLocation(material_prog_,"uEye");
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
    void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial material,Color c,
                                  const LunarSurfaceMapping& mapping) override {
        if(lunar_scene_active()) lunar_->triangles(v,material,c,mapping);
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
        const int material=std::clamp(kind,0,kCampusMaterialCount-1);
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
