#include <jsd/game_shell.hpp>
#include <frontier/detail/gl.hpp>
#include <frontier/platform.hpp>
#include <frontier/renderer.hpp>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace rf;using namespace rf::gfx;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
namespace {
const unsigned char* (*originalString)(unsigned)=nullptr;
const unsigned char* changedVendor(unsigned name){return name==0x1F00?reinterpret_cast<const unsigned char*>("regression-other-driver"):originalString(name);}
}
int main(){try{
    namespace fs=std::filesystem;
    const auto root=fs::current_path()/"shader startup test cache";fs::remove_all(root);
    Platform platform(true,640,360);vl::GL gl;gl.load([&](const char* n){return platform.resolve(n);});
    const auto pixels=[&]{std::vector<unsigned char> image(640*360*4);gl.ReadPixels(0,0,640,360,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,image.data());return image;};
    {jsd::shell::Window splash(platform,"SHADER REGRESSION");
        gl.Viewport(13,17,600,300);gl.Enable(vl::GL_SCISSOR_TEST);gl.ColorMaski(1,0,1,1,1);
        unsigned framebuffer=0;gl.GenFramebuffers(1,&framebuffer);gl.BindFramebuffer(0x8CA9,framebuffer);
        splash.showStartup("Preparing graphics","First start / compiling shader cache",true);
        int viewport[4]{},drawFramebuffer=0,scissor=0;
        gl.GetIntegerv(0x0BA2,viewport);gl.GetIntegerv(0x8CA6,&drawFramebuffer);
        gl.GetIntegerv(vl::GL_SCISSOR_TEST,&scissor);
        const auto colorMask=reinterpret_cast<void (*)(unsigned,unsigned,unsigned char*)>(platform.resolve("glGetBooleani_v"));
        unsigned char mask[4]{};colorMask(0x0C23,1,mask);
        require(viewport[0]==13&&viewport[1]==17&&viewport[2]==600&&viewport[3]==300&&scissor&&
                unsigned(drawFramebuffer)==framebuffer&&mask[0]==0&&mask[1]==1,"splash changed renderer GL state");
        gl.Disable(vl::GL_SCISSOR_TEST);gl.ColorMaski(1,1,1,1,1);
        gl.BindFramebuffer(0x8CA9,0);gl.DeleteFramebuffers(1,&framebuffer);
        auto image=pixels();unsigned low=255,high=0;for(std::size_t i=0;i<image.size();i+=4){low=std::min(low,unsigned(image[i]));high=std::max(high,unsigned(image[i]));}
        require(high-low>100,"loading splash is blank");require(gl.GetError()==vl::GL_NO_ERROR,"splash generated a GL error");
        namespace shell=jsd::shell;
        shell::Menu menu;
        const shell::TitleItems title;
        const shell::Page page{"UNRELATED GAME",title.items()};
        shell::Input input;input.select=true;
        const auto first=splash.present(menu,page,input);
        require(first&&title.action(*first)==shell::TitleAction::NewGame,"Enter missed the default title action");
        input.up=true;
        const auto last=splash.present(menu,page,input);
        require(last&&title.action(*last)==shell::TitleAction::Exit,"keyboard menu navigation failed to wrap");
        input={};input.click=true;input.pointer={320,136};
        const auto click=splash.present(menu,page,input);
        require(click&&title.action(*click)==shell::TitleAction::NewGame,"drawable coordinates missed the menu button");
        require(pixels()!=image&&gl.GetError()==vl::GL_NO_ERROR,"title menu did not render a new frame");
        const shell::TitleItems withSave({false,true,true}), session({true,true,true});
        require(withSave.action(0)==shell::TitleAction::Continue&&session.action(0)==shell::TitleAction::Resume&&
                session.action(1)==shell::TitleAction::Save,"save and resume entries missing from title");
    }
    auto settings=GraphicsSettings::minimal();settings.viewDistance=100;
    unsigned callbacks=0;
    const auto render=[&](Renderer& renderer){Camera camera;camera.position={0,1,5};camera.direction=normalized(V3{0,-.1f,-1});
        renderer.begin(camera);renderer.primitive(Primitive::Box,{{0,0,0},{2,2,2},0});renderer.end();renderer.check();return pixels();};
    ShaderCacheStatistics cold,warm;std::vector<unsigned char> reference;
    {
        jsd::shell::Window splash(platform,"SHADER REGRESSION");
        auto startup=splash.rendererStartup();startup.cacheDirectory=root;
        const auto progress=startup.progress;
        startup.progress=[&](const ShaderBuildProgress& update){++callbacks;progress(update);};
        Renderer renderer(platform,settings,std::move(startup));
        cold=renderer.shaderCacheStatistics();reference=render(renderer);
        require(callbacks>0&&cold.compiled>0,"startup did not compile/report programs");
    }
    if(!cold.supported||!cold.stored){std::cout<<"PASS splash and source fallback; driver has no reusable program binaries\n";fs::remove_all(root);return 0;}
    {
        Renderer renderer(platform,settings,{true,root,{}});warm=renderer.shaderCacheStatistics();
        require(warm.loaded>0&&warm.compiled==0,"warm startup did not restore shader binaries");
        require(render(renderer)==reference,"cached shader output differs from source compilation");
    }
    fs::path entry;for(const auto& f:fs::recursive_directory_iterator(root))if(f.path().extension()==".bin"){entry=f.path();break;}
    require(!entry.empty(),"first launch did not persist shader cache");
    {std::fstream file(entry,std::ios::in|std::ios::out|std::ios::binary);file.seekg(-1,std::ios::end);char value=0;file.read(&value,1);value^=0x55;file.seekp(-1,std::ios::end);file.write(&value,1);}
    {
        Renderer renderer(platform,settings,{true,root,{}});const auto stats=renderer.shaderCacheStatistics();
        require(stats.rejected>=1&&stats.compiled>=1&&stats.loaded>0,"corrupt cache did not recompile safely");
        require(render(renderer)==reference,"corrupt cache recovery changed rendering");
    }
    {
        // Valid metadata/checksum with a rejected driver payload exercises the
        // actual glProgramBinary failure path, not just file validation.
        std::ifstream in(entry,std::ios::binary|std::ios::ate);const auto size=in.tellg();in.seekg(0);
        std::vector<unsigned char> bytes(static_cast<std::size_t>(size));in.read(reinterpret_cast<char*>(bytes.data()),bytes.size());in.close();
        require(bytes.size()>40,"cache entry has no payload");std::fill(bytes.begin()+40,bytes.end(),0);
        std::uint64_t checksum=1469598103934665603ull;for(std::size_t i=40;i<bytes.size();++i)checksum=(checksum^bytes[i])*1099511628211ull;
        std::memcpy(bytes.data()+32,&checksum,sizeof(checksum));
        {std::ofstream out(entry,std::ios::binary|std::ios::trunc);out.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());}
        Renderer renderer(platform,settings,{true,root,{}});
        require(renderer.shaderCacheStatistics().rejected>=1&&renderer.shaderCacheStatistics().compiled>=1,"driver-rejected binary prevented fallback");
        require(render(renderer)==reference,"driver rejection recovery changed rendering");
    }
    {
        const auto blocked=root/"not a directory";std::ofstream(blocked)<<"file";
        Renderer renderer(platform,settings,{true,blocked,{}});
        require(!renderer.shaderCacheStatistics().enabled&&renderer.shaderCacheStatistics().compiled>0,"unwritable cache prevented source fallback");render(renderer);
    }
    {
        Renderer renderer(platform,settings,{false,root,{}});
        require(!renderer.shaderCacheStatistics().enabled&&renderer.shaderCacheStatistics().loaded==0,"disabled shader cache still loaded binaries");
    }
    {
        Context context{[&](const char* name){return std::string_view(name)=="glProgramBinary"?nullptr:platform.resolve(name);},[]{return std::pair{640,360};},true,{}};
        Renderer renderer(context,settings,{true,root,{}});
        require(!renderer.shaderCacheStatistics().supported&&renderer.shaderCacheStatistics().compiled>0,"missing optional API broke GL 3.3 fallback");render(renderer);
    }
    {
        originalString=gl.GetString;
        Context context{[&](const char* name){return std::string_view(name)=="glGetString"?reinterpret_cast<void*>(&changedVendor):platform.resolve(name);},[]{return std::pair{640,360};},true,{}};
        Renderer renderer(context,settings,{true,root,{}});
        require(renderer.shaderCacheStatistics().loaded==cold.loaded&&renderer.shaderCacheStatistics().compiled==cold.compiled,"driver change reused incompatible cache namespace");
    }
    {
        unsigned progress=0;Renderer renderer(platform,settings,{true,root,[&](const ShaderBuildProgress&){++progress;}});
        const auto completed=progress;std::string error;
        const std::string vs=R"(#version 330 core
layout(location=0)in vec3 position;void main(){gl_Position=vec4(position,1);})";
        const std::string fs=R"(#version 330 core
out vec4 color;void main(){color=vec4(.2,.4,.6,1);})";
        auto before=renderer.shaderCacheStatistics();require(renderer.reloadMaterialShaders(vs,fs,error),error.c_str());
        require(renderer.shaderCacheStatistics().compiled==before.compiled+1,"new source did not miss cache");
        before=renderer.shaderCacheStatistics();require(renderer.reloadMaterialShaders(vs,fs,error),error.c_str());
        require(renderer.shaderCacheStatistics().loaded==before.loaded+1,"identical source did not reuse cache");
        before=renderer.shaderCacheStatistics();require(renderer.reloadMaterialShaders(vs,fs+"\n// source revision\n",error),error.c_str());
        require(renderer.shaderCacheStatistics().compiled==before.compiled+1,"source change did not invalidate cache");
        require(progress==completed,"startup callback remained attached after loading completed");
    }
    std::cout<<"PASS visible splash, cold/warm cache, identical pixels, corruption, source/driver invalidation, disabled/unwritable/unsupported fallback and callback lifetime\n";
    std::cout<<"cold compiled="<<cold.compiled<<" stored="<<cold.stored<<" compile_ms="<<cold.compileMilliseconds<<"; warm loaded="<<warm.loaded<<" load_ms="<<warm.loadMilliseconds<<'\n';
    fs::remove_all(root);return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
