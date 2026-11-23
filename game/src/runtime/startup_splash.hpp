#pragma once
#include "ui_batch.hpp"
#include <frontier/platform.hpp>
#include <frontier/detail/gl.hpp>
#include <chrono>
#include <stdexcept>

namespace game::runtime {
struct StartupCancelled {};
class StartupSplash {
    rf::Platform& platform;
    vl::GL gl;
    ui::Batch batch;
    unsigned program{},vao{},vbo{},atlas{};
    std::string lastPhase;
    std::chrono::steady_clock::time_point began=std::chrono::steady_clock::now(),last{};
    unsigned shader(unsigned kind,const char* text){
        const auto id=gl.CreateShader(kind);gl.ShaderSource(id,1,&text,nullptr);gl.CompileShader(id);
        int ok=0;gl.GetShaderiv(id,vl::GL_COMPILE_STATUS,&ok);
        if(!ok){gl.DeleteShader(id);throw std::runtime_error("Cannot compile the startup splash");}return id;
    }
    void circle(float x,float y,float radius,Color color){
        const auto byte=[](float v){return static_cast<std::uint8_t>(std::clamp(v,0.f,1.f)*255);};
        const std::array rgba{byte(color.r),byte(color.g),byte(color.b),byte(color.a)};
        for(int i=0;i<64;++i){const float a=i*6.28318530718f/64,b=(i+1)*6.28318530718f/64;
            for(auto p:{Vec2{x,y},Vec2{x+std::cos(a)*radius,y+std::sin(a)*radius},Vec2{x+std::cos(b)*radius,y+std::sin(b)*radius}})
                batch.vertices.push_back({p,{-1,-1},rgba});}
    }
public:
    explicit StartupSplash(rf::Platform& p):platform(p){
        gl.load([&](const char* name){return platform.resolve(name);});
        platform.setVSync(false);platform.dimensions();
        gl.Viewport(0,0,platform.width,platform.height);gl.ClearColor(.015f,.024f,.038f,1);gl.Clear(vl::GL_COLOR_BUFFER_BIT);platform.present();
        const auto vs=shader(vl::GL_VERTEX_SHADER,R"GLSL(#version 330 core
layout(location=0)in vec2 position;layout(location=1)in vec2 uv;layout(location=2)in vec4 tint;
uniform vec2 viewport;out vec2 vUV;out vec4 vTint;
void main(){gl_Position=vec4(position.x/viewport.x*2.-1.,1.-position.y/viewport.y*2.,0,1);vUV=uv;vTint=tint;})GLSL");
        const auto fs=shader(vl::GL_FRAGMENT_SHADER,R"GLSL(#version 330 core
in vec2 vUV;in vec4 vTint;uniform sampler2D fontAtlas;out vec4 color;
void main(){float coverage=1.;if(vUV.x>=0.){float d=texture(fontAtlas,vUV).r;float edge=clamp(fwidth(d)*.55,.017,.28);coverage=smoothstep(.49-edge,.49+edge,d);}color=vec4(vTint.rgb,vTint.a*coverage);})GLSL");
        program=gl.CreateProgram();gl.AttachShader(program,vs);gl.AttachShader(program,fs);gl.LinkProgram(program);gl.DeleteShader(vs);gl.DeleteShader(fs);
        int linked=0;gl.GetProgramiv(program,vl::GL_LINK_STATUS,&linked);if(!linked)throw std::runtime_error("Cannot link startup splash");
        gl.GenVertexArrays(1,&vao);gl.GenBuffers(1,&vbo);gl.BindVertexArray(vao);gl.BindBuffer(vl::GL_ARRAY_BUFFER,vbo);
        for(unsigned i:{0u,1u,2u})gl.EnableVertexAttribArray(i);
        gl.VertexAttribPointer(0,2,vl::GL_FLOAT,0,sizeof(ui::Vertex),nullptr);
        gl.VertexAttribPointer(1,2,vl::GL_FLOAT,0,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,uv)));
        gl.VertexAttribPointer(2,4,vl::GL_UNSIGNED_BYTE,1,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,color)));
        gl.GenTextures(1,&atlas);gl.BindTexture(vl::GL_TEXTURE_2D,atlas);gl.TexParameteri(vl::GL_TEXTURE_2D,vl::GL_TEXTURE_MIN_FILTER,vl::GL_LINEAR);gl.TexParameteri(vl::GL_TEXTURE_2D,vl::GL_TEXTURE_MAG_FILTER,vl::GL_LINEAR);
        gl.TexImage2D(vl::GL_TEXTURE_2D,0,0x8229,ui::atlas_width,ui::atlas_height,0,0x1903,vl::GL_UNSIGNED_BYTE,ui::font_atlas().data());
        show("Starting Lunar Colony",{},true);
    }
    ~StartupSplash(){gl.DeleteProgram(program);gl.DeleteVertexArrays(1,&vao);gl.DeleteBuffers(1,&vbo);gl.DeleteTextures(1,&atlas);}
    void show(std::string_view phase,std::string_view detail={},bool force=false){
        if(platform.window){SDL_Event event{};while(SDL_PollEvent(&event))
            if(event.type==vl::sdl::quit||(event.type==vl::sdl::window&&event.window.event==14)||
               (event.type==vl::sdl::keyDown&&event.key.keysym.scancode==41))throw StartupCancelled{};}
        const auto now=std::chrono::steady_clock::now();if(!force&&phase==lastPhase&&now-last<std::chrono::milliseconds(33))return;last=now;lastPhase=phase;
        platform.dimensions();const float w=float(std::max(1,platform.width)),h=float(std::max(1,platform.height));
        const float scale=std::clamp(std::min(w/1280.f,h/720.f),.45f,3.f),cx=w*.5f,cy=h*.37f;
        batch.vertices.clear();
        batch.quad({0,0,w,h},{.015f,.024f,.038f,1},{.035f,.055f,.075f,1});
        for(int i=0;i<90;++i){const float x=float((i*193+31)%1279)/1280*w,y=float((i*317+47)%719)/720*h;const float size=(i%4==0?2.f:1.f)*scale;batch.quad({x,y,size,size},{.42f,.54f,.61f,.65f},{.42f,.54f,.61f,.65f});}
        circle(cx,cy,78*scale,{.12f,.22f,.27f,1});circle(cx-4*scale,cy-4*scale,72*scale,{.67f,.72f,.72f,1});
        for(auto crater:std::array<Vec3,5>{{{25,-23,13},{-24,-13,18},{8,29,16},{-33,26,8},{36,13,6}}})circle(cx+crater.x*scale,cy+crater.y*scale,crater.z*scale,{.49f,.56f,.57f,1});
        batch.stroke({cx-105*scale,cy+35*scale},{cx+105*scale,cy-35*scale},{.35f,.85f,.79f,.8f},2*scale);
        const auto centered=[&](float y,std::string_view text,float font,Color color){batch.text({cx-float(text.size())*ui::kGlyphAdvance*font*.5f,y},text,font,color);};
        centered(cy+115*scale,"LUNAR COLONY",4.2f*scale,{.92f,.97f,.95f,1});
        centered(cy+169*scale,phase,2.3f*scale,{.55f,.86f,.82f,1});
        centered(cy+201*scale,detail,1.65f*scale,{.57f,.65f,.71f,1});
        const float barW=380*scale,barY=cy+245*scale;
        batch.quad({cx-barW*.5f,barY,barW,3*scale},{.11f,.19f,.23f,1},{.11f,.19f,.23f,1});
        const float t=std::fmod(std::chrono::duration<float>(now-began).count()*.40f,1.f);
        batch.quad({cx-barW*.5f+t*(barW-70*scale),barY,70*scale,3*scale},{.38f,.88f,.78f,1},{.38f,.88f,.78f,1});
        // Progress callbacks can run in the middle of engine initialization.
        // Restore every state this independent splash pipeline touches.
        int viewport[4]{},oldProgram=0,oldVao=0,oldBuffer=0,oldDraw=0,oldRead=0,oldUnit=0,oldTexture=0,oldSampler=0;
        gl.GetIntegerv(0x0BA2,viewport);gl.GetIntegerv(0x8B8D,&oldProgram);gl.GetIntegerv(0x85B5,&oldVao);gl.GetIntegerv(0x8894,&oldBuffer);
        gl.GetIntegerv(0x8CA6,&oldDraw);gl.GetIntegerv(0x8CAA,&oldRead);gl.GetIntegerv(0x84E0,&oldUnit);
        gl.ActiveTexture(vl::GL_TEXTURE0);gl.GetIntegerv(0x8069,&oldTexture);gl.GetIntegerv(0x8919,&oldSampler);
        const unsigned capabilities[]{0x0B71,0x0B44,vl::GL_SCISSOR_TEST,0x8DB9,vl::GL_BLEND};int enabled[5]{};
        for(int i=0;i<5;++i)gl.GetIntegerv(capabilities[i],&enabled[i]);
        int srcRgb=0,dstRgb=0,srcAlpha=0,dstAlpha=0;
        gl.GetIntegerv(0x80C9,&srcRgb);gl.GetIntegerv(0x80C8,&dstRgb);gl.GetIntegerv(0x80CB,&srcAlpha);gl.GetIntegerv(0x80CA,&dstAlpha);
        gl.BindFramebuffer(vl::GL_FRAMEBUFFER,0);gl.Viewport(0,0,int(w),int(h));gl.Disable(0x0B71);gl.Disable(0x0B44);gl.Disable(vl::GL_SCISSOR_TEST);gl.Disable(0x8DB9);
        gl.Enable(vl::GL_BLEND);gl.BlendFunc(vl::GL_SRC_ALPHA,vl::GL_ONE_MINUS_SRC_ALPHA);gl.UseProgram(program);
        gl.Uniform2f(gl.GetUniformLocation(program,"viewport"),w,h);gl.Uniform1i(gl.GetUniformLocation(program,"fontAtlas"),0);
        gl.ActiveTexture(vl::GL_TEXTURE0);gl.BindSampler(0,0);gl.BindTexture(vl::GL_TEXTURE_2D,atlas);gl.BindVertexArray(vao);gl.BindBuffer(vl::GL_ARRAY_BUFFER,vbo);
        gl.BufferData(vl::GL_ARRAY_BUFFER,batch.vertices.size()*sizeof(ui::Vertex),batch.vertices.data(),vl::GL_STREAM_DRAW);
        gl.DrawArrays(vl::GL_TRIANGLES,0,int(batch.vertices.size()));platform.present();
        gl.BindFramebuffer(0x8CA9,oldDraw);gl.BindFramebuffer(0x8CA8,oldRead);gl.Viewport(viewport[0],viewport[1],viewport[2],viewport[3]);
        gl.UseProgram(oldProgram);gl.BindVertexArray(oldVao);gl.BindBuffer(vl::GL_ARRAY_BUFFER,oldBuffer);
        gl.BindSampler(0,oldSampler);gl.BindTexture(vl::GL_TEXTURE_2D,oldTexture);gl.ActiveTexture(oldUnit);
        gl.BlendFuncSeparate(srcRgb,dstRgb,srcAlpha,dstAlpha);
        for(int i=0;i<5;++i){if(enabled[i])gl.Enable(capabilities[i]);else gl.Disable(capabilities[i]);}
    }
};
}
