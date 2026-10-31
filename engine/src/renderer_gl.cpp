#include "engine/engine.hpp"
#include <algorithm>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <dlfcn.h>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
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
    const GLubyte* (*glGetString)(GLenum)=nullptr;
    void (*glClearColor)(GLfloat,GLfloat,GLfloat,GLfloat)=nullptr; void (*glClear)(GLbitfield)=nullptr; void (*glViewport)(GLint,GLint,GLsizei,GLsizei)=nullptr; void (*glEnable)(GLenum)=nullptr; void (*glDisable)(GLenum)=nullptr; void (*glBlendFunc)(GLenum,GLenum)=nullptr; void (*glLineWidth)(GLfloat)=nullptr;
    GLuint (*glCreateShader)(GLenum)=nullptr; void (*glShaderSource)(GLuint,GLsizei,const GLchar* const*,const GLint*)=nullptr; void (*glCompileShader)(GLuint)=nullptr; void (*glGetShaderiv)(GLuint,GLenum,GLint*)=nullptr; void (*glGetShaderInfoLog)(GLuint,GLsizei,GLsizei*,GLchar*)=nullptr;
    GLuint (*glCreateProgram)()=nullptr; void (*glAttachShader)(GLuint,GLuint)=nullptr; void (*glLinkProgram)(GLuint)=nullptr; void (*glGetProgramiv)(GLuint,GLenum,GLint*)=nullptr; void (*glGetProgramInfoLog)(GLuint,GLsizei,GLsizei*,GLchar*)=nullptr; void (*glUseProgram)(GLuint)=nullptr;
    void (*glGenBuffers)(GLsizei,GLuint*)=nullptr; void (*glBindBuffer)(GLenum,GLuint)=nullptr; void (*glBufferData)(GLenum,GLsizeiptr,const void*,GLenum)=nullptr; void (*glBufferSubData)(GLenum,GLsizeiptr,GLsizeiptr,const void*)=nullptr; void (*glEnableVertexAttribArray)(GLuint)=nullptr; void (*glVertexAttribPointer)(GLuint,GLint,GLenum,GLboolean,GLsizei,const void*)=nullptr; void (*glDrawArrays)(GLenum,GLint,GLsizei)=nullptr;
    GLint (*glGetUniformLocation)(GLuint,const GLchar*)=nullptr; void (*glUniformMatrix4fv)(GLint,GLsizei,GLboolean,const GLfloat*)=nullptr; void (*glUniform4f)(GLint,GLfloat,GLfloat,GLfloat,GLfloat)=nullptr;

    template<class T> bool load(T& out,const char* n){ out=(T)dlsym(lib,n); if(!out && glXGetProcAddressARB) out=(T)glXGetProcAddressARB((const GLubyte*)n); return out!=nullptr; }
    bool init(){
        lib=dlopen("libGL.so.1",RTLD_NOW|RTLD_GLOBAL); if(!lib)return false;
        load(glXCreateContext,"glXCreateContext"); load(glXMakeCurrent,"glXMakeCurrent"); load(glXIsDirect,"glXIsDirect"); load(glXSwapBuffers,"glXSwapBuffers"); load(glXGetProcAddressARB,"glXGetProcAddressARB");
        bool ok=true;
#define L(x) ok=load(x,#x)&&ok
        L(glGetString);L(glClearColor);L(glClear);L(glViewport);L(glEnable);L(glDisable);L(glBlendFunc);L(glLineWidth);L(glBindAttribLocation);L(glCreateShader);L(glShaderSource);L(glCompileShader);L(glGetShaderiv);L(glGetShaderInfoLog);L(glCreateProgram);L(glAttachShader);L(glLinkProgram);L(glGetProgramiv);L(glGetProgramInfoLog);L(glUseProgram);L(glGenBuffers);L(glBindBuffer);L(glBufferData);L(glBufferSubData);L(glEnableVertexAttribArray);L(glVertexAttribPointer);L(glDrawArrays);L(glGetUniformLocation);L(glUniformMatrix4fv);L(glUniform4f);
#undef L
        return ok&&glXCreateContext&&glXMakeCurrent&&glXIsDirect&&glXSwapBuffers;
    }
};

class GLRenderer final: public Renderer {
    GLApi gl_; Display* d_{}; Window w_{}; GLXContext ctx_{}; int width_{},height_{}; GLuint prog_{},vbo_{}; GLint u_mvp_{},u_color_{}; Mat4 mvp_=Mat4::identity();
    GLsizeiptr vbo_capacity_bytes_{0};
    bool hardware_accelerated_{false};
    std::string gpu_vendor_{};
    std::string gpu_renderer_{};
    std::string gpu_version_{};

    static bool software_renderer_name(std::string value){
        std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){ return (char)std::tolower(c); });
        static constexpr const char* needles[]={"llvmpipe","softpipe","swrast","software rasterizer","software renderer","mesa offscreen"};
        for(const char* needle:needles) if(value.find(needle)!=std::string::npos) return true;
        return false;
    }
    GLuint shader(GLenum type,const char* src){ GLuint s=gl_.glCreateShader(type); gl_.glShaderSource(s,1,&src,nullptr); gl_.glCompileShader(s); GLint ok=0; gl_.glGetShaderiv(s,GL_COMPILE_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetShaderInfoLog(s,2047,&n,log); std::cerr<<"Shader error: "<<log<<"\n"; } return s; }
    void upload_and_draw(const float* data,size_t floats,GLenum prim,Color c,float width=1.0f){
        if(!data || floats==0) return;
        gl_.glUseProgram(prog_); gl_.glUniformMatrix4fv(u_mvp_,1,GL_FALSE,mvp_.m.data()); gl_.glUniform4f(u_color_,c.r,c.g,c.b,c.a); gl_.glBindBuffer(GL_ARRAY_BUFFER,vbo_);
        const GLsizeiptr bytes=(GLsizeiptr)(floats*sizeof(float));
        if(bytes>vbo_capacity_bytes_){
            GLsizeiptr capacity=4096;
            while(capacity<bytes) capacity*=2;
            gl_.glBufferData(GL_ARRAY_BUFFER,capacity,nullptr,GL_DYNAMIC_DRAW);
            vbo_capacity_bytes_=capacity;
        }
        gl_.glBufferSubData(GL_ARRAY_BUFFER,0,bytes,data);
        gl_.glEnableVertexAttribArray(0); gl_.glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),nullptr); gl_.glLineWidth(width); gl_.glDrawArrays(prim,0,(GLsizei)(floats/3));
    }
public:
    bool initialize(void* native_display,unsigned long native_window,int width,int height) override {
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
        if(!hardware_accelerated_){
            std::cerr<<"Hardware OpenGL is required. GLX direct="<<(direct?"yes":"no")
                     <<", vendor='"<<gpu_vendor_<<"', renderer='"<<gpu_renderer_<<"', version='"<<gpu_version_<<"'.\n"
                     <<"Disable LIBGL_ALWAYS_SOFTWARE and install/enable the GPU driver.\n";
            return false;
        }
        std::cout<<"GPU renderer: "<<gpu_vendor_<<" | "<<gpu_renderer_<<" | OpenGL "<<gpu_version_<<" (direct GLX)\n";

        const char* vs="#version 120\nattribute vec3 aPos; uniform mat4 uMVP; void main(){ gl_Position=uMVP*vec4(aPos,1.0); }";
        const char* fs="#version 120\nuniform vec4 uColor; void main(){ gl_FragColor=uColor; }";
        GLuint sv=shader(GL_VERTEX_SHADER,vs), sf=shader(GL_FRAGMENT_SHADER,fs); prog_=gl_.glCreateProgram(); gl_.glAttachShader(prog_,sv); gl_.glAttachShader(prog_,sf); gl_.glBindAttribLocation(prog_,0,"aPos"); gl_.glLinkProgram(prog_); GLint ok=0; gl_.glGetProgramiv(prog_,GL_LINK_STATUS,&ok); if(!ok){ char log[2048]{}; GLsizei n=0; gl_.glGetProgramInfoLog(prog_,2047,&n,log); std::cerr<<"Program link error: "<<log<<"\n"; return false; }
        u_mvp_=gl_.glGetUniformLocation(prog_,"uMVP"); u_color_=gl_.glGetUniformLocation(prog_,"uColor"); gl_.glGenBuffers(1,&vbo_); resize(width,height); return true;
    }
    void resize(int w,int h) override { width_=w;height_=h; if(h<1)h=1; gl_.glViewport(0,0,w,h); }
    void begin_frame(Color c) override { gl_.glClearColor(c.r,c.g,c.b,c.a); gl_.glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); gl_.glEnable(GL_DEPTH_TEST); gl_.glEnable(GL_BLEND); gl_.glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); }
    void end_frame() override { gl_.glXSwapBuffers(d_,w_); }
    void set_3d(const Mat4& vp) override { mvp_=vp; gl_.glEnable(GL_DEPTH_TEST); }
    void set_2d(float scale=1.0f) override {
        scale=std::max(0.25f,scale);
        mvp_=ortho(0,(float)width_/scale,(float)height_/scale,0,-1,1);
        gl_.glDisable(GL_DEPTH_TEST);
    }
    void draw_lines_3d(const std::vector<Vec3>& v,Color c,float width) override { if(v.empty())return; upload_and_draw((const float*)v.data(),v.size()*3,GL_LINES,c,width); }
    void draw_triangles_3d(const std::vector<Vec3>& v,Color c) override { if(v.empty())return; upload_and_draw((const float*)v.data(),v.size()*3,GL_TRIANGLES,c,1); }
    void draw_rect(Rect r,Color c) override { float v[]={r.x,r.y,0,r.x+r.w,r.y,0,r.x+r.w,r.y+r.h,0, r.x,r.y,0,r.x+r.w,r.y+r.h,0,r.x,r.y+r.h,0}; upload_and_draw(v,18,GL_TRIANGLES,c); }
    void draw_line(Vec2 a,Vec2 b,Color c,float width) override { float v[]={a.x,a.y,0,b.x,b.y,0}; upload_and_draw(v,6,GL_LINES,c,width); }
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override;
    int width() const override{return width_;} int height() const override{return height_;}
    bool hardware_accelerated() const override{return hardware_accelerated_;}
    std::string gpu_vendor() const override{return gpu_vendor_;}
    std::string gpu_renderer() const override{return gpu_renderer_;}
    std::string gpu_version() const override{return gpu_version_;}
};

static const std::unordered_map<char,std::array<unsigned char,7>> FONT={
{'A',{14,17,17,31,17,17,17}},{'B',{30,17,17,30,17,17,30}},{'C',{14,17,16,16,16,17,14}},{'D',{30,17,17,17,17,17,30}},{'E',{31,16,16,30,16,16,31}},{'F',{31,16,16,30,16,16,16}},{'G',{14,17,16,23,17,17,14}},{'H',{17,17,17,31,17,17,17}},{'I',{31,4,4,4,4,4,31}},{'J',{7,2,2,2,18,18,12}},{'K',{17,18,20,24,20,18,17}},{'L',{16,16,16,16,16,16,31}},{'M',{17,27,21,21,17,17,17}},{'N',{17,25,21,19,17,17,17}},{'O',{14,17,17,17,17,17,14}},{'P',{30,17,17,30,16,16,16}},{'Q',{14,17,17,17,21,18,13}},{'R',{30,17,17,30,20,18,17}},{'S',{15,16,16,14,1,1,30}},{'T',{31,4,4,4,4,4,4}},{'U',{17,17,17,17,17,17,14}},{'V',{17,17,17,17,17,10,4}},{'W',{17,17,17,21,21,21,10}},{'X',{17,17,10,4,10,17,17}},{'Y',{17,17,10,4,4,4,4}},{'Z',{31,1,2,4,8,16,31}},
{'0',{14,17,19,21,25,17,14}},{'1',{4,12,4,4,4,4,14}},{'2',{14,17,1,2,4,8,31}},{'3',{30,1,1,14,1,1,30}},{'4',{2,6,10,18,31,2,2}},{'5',{31,16,16,30,1,1,30}},{'6',{14,16,16,30,17,17,14}},{'7',{31,1,2,4,8,8,8}},{'8',{14,17,17,14,17,17,14}},{'9',{14,17,17,15,1,1,14}},
{'%',{17,2,4,4,8,16,17}},{'+',{0,4,4,31,4,4,0}},{'-',{0,0,0,31,0,0,0}},{'.',{0,0,0,0,0,12,12}},{':',{0,12,12,0,12,12,0}},{'/',{1,2,4,8,16,0,0}},{'x',{0,0,17,10,4,10,17}},{'X',{17,17,10,4,10,17,17}},{' ',{0,0,0,0,0,0,0}},{'<',{2,4,8,16,8,4,2}},{'>',{8,4,2,1,2,4,8}},{'=',{0,0,31,0,31,0,0}},{'(',{2,4,8,8,8,4,2}},{')',{8,4,2,2,2,4,8}}
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
        auto it=FONT.find(ch);
        if(it==FONT.end()){ p.x+=6*scale; continue; }
        const auto& rows=it->second;
        for(int y=0;y<7;++y) for(int x=0;x<5;++x)
            if(rows[y]&(1<<(4-x))) quad(p.x+x*scale,p.y+y*scale);
        p.x+=6*scale;
    }
    if(!vertices.empty()) upload_and_draw(vertices.data(),vertices.size(),GL_TRIANGLES,c,1.0f);
}
std::unique_ptr<Renderer> create_gl_renderer(){ return std::make_unique<GLRenderer>(); }
}
