#include <frontier/renderer.hpp>
#include <frontier/detail/gl.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace rf;using namespace rf::gfx;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
    Platform platform(true,320,200);auto settings=GraphicsSettings::minimal();settings.viewDistance=100;
    Renderer renderer(platform,settings);vl::GL gl;gl.load([&](const char* n){return platform.resolve(n);});
    Camera camera;camera.position={0,0,5};camera.direction={0,0,-1};
    const std::vector<LineSegment> guides{{{-2,0,1},{2,0,1},{0,1,0,1},5},{{-2,.4f,-1},{2,.4f,-1},{1,0,0,1},5}};
    const auto frame=[&](bool block){
        renderer.begin(camera);if(block)renderer.primitive(Primitive::Box,{{0,0,0},{2,2,.5f},0});
        renderer.lines(guides);renderer.end();renderer.check();
        std::vector<unsigned char> pixels(320*200*4);gl.ReadPixels(0,0,320,200,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());return pixels;
    };
    auto first=frame(true);require(renderer.statistics().lineSegments==2,"native guides were not drawn");
    require(renderer.statistics().lineUploadBytes==guides.size()*sizeof(LineSegment),"initial guides not uploaded");
    auto second=frame(true);require(renderer.statistics().lineUploadBytes==0,"identical guides uploaded every frame");
    require(first==second,"static guide pixels flicker");
    const auto index=[](int x,int y){return (y*320+x)*4;};
    auto middle=index(160,100);require(second[middle+1]>200&&second[middle]<30,"front guide missing");
    unsigned redBlocked=0,redOpen=0;
    for(int y=108;y<118;++y)for(int x=140;x<180;++x){auto n=index(x,y);if(second[n]>200&&second[n+1]<30)++redBlocked;}
    auto open=frame(false);
    for(int y=108;y<118;++y)for(int x=140;x<180;++x){auto n=index(x,y);if(open[n]>200&&open[n+1]<30)++redOpen;}
    require(redOpen>redBlocked+20,"guides ignored scene depth");
    camera.position.x=.3f;frame(true);require(renderer.statistics().lineUploadBytes==0,"camera movement rebuilt guide geometry");
    renderer.begin(camera);auto bad=guides;bad[0].width=std::numeric_limits<float>::quiet_NaN();bool rejected=false;
    try{renderer.lines(bad);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"nonfinite guides accepted");
    renderer.begin(camera);renderer.end();require(renderer.statistics().lineSegments==0,"stale guides leaked into next frame");
    const auto picture=[&](const std::vector<LineSegment>& segments){
        renderer.begin(camera);renderer.lines(segments);renderer.end();
        std::vector<unsigned char> pixels(320*200*4);
        gl.ReadPixels(0,0,320,200,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
        return pixels;
    };
    const LineSegment distant{{999,0,0},{1000,0,0},{1,0,0,.4f},3};
    const std::vector<LineSegment> visible{
        {{-1,0,0},{1,0,0},{0,1,0,.6f},12},
        {{0,.1f,6},{0,.5f,-4},{0,0,1,.7f},2},
        {{-1,3.2f,0},{1,3.2f,0},{1,1,0,.5f},100}
    };
    auto reference=picture(visible);
    std::vector<LineSegment> groups(8192,distant);
    groups[260]=visible[0];groups[3100]=visible[1];groups.back()=visible[2];
    require(picture(groups)==reference,"Guide culling changed coverage or blend order");
    require(renderer.statistics().lineSegments<groups.size(),"No offscreen guide blocks culled");
    const auto initialPosition=camera.position;
    camera.position.x+=1000;
    picture(groups);
    require(renderer.statistics().lineUploadBytes==0,"Camera motion reuploaded guide blocks");
    camera.position=initialPosition;
    require(picture(groups)==reference,"Returning camera lost retained guides");
    std::cout<<"PASS native guide rendering, depth occlusion, retained uploads, camera movement and validation\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
