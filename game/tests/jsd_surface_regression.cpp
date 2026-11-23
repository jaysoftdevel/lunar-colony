#include <frontier/renderer.hpp>
#include <frontier/detail/gl.hpp>
#include <iostream>
#include <stdexcept>
using namespace rf;
using namespace rf::gfx;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
    Platform platform(true,640,360);auto settings=GraphicsSettings::minimal();settings.viewDistance=100;settings.antiAliasing=AntiAliasing::Off;
    Renderer renderer(platform,settings);vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
    const auto capture=[&](SurfaceKind surface,float x,unsigned seed,bool uv,float time){
        Camera camera;camera.position={x,3.4f,5.5f};camera.direction=normalized(V3{0,-.5f,-1});
        Environment environment;environment.fogDensity=0;environment.atmosphere.time=time;
        Material material;material.surface=surface;material.surfaceSeed=seed;material.surfaceUV=uv;
        material.albedo={.46f,.50f,.52f,1};if(surface==SurfaceKind::Console)material.emissive={.1f,.8f,1,1};
        renderer.begin(camera,environment);renderer.primitive(Primitive::Box,{{x,0,0},{8,.5f,7},0},material);renderer.end();renderer.check();
        std::vector<unsigned char> image(640*360*4);gl.ReadPixels(0,0,640,360,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,image.data());return image;
    };
    const auto difference=[](const auto& a,const auto& b){double d=0;for(std::size_t i=0;i<a.size();++i)d+=std::abs(int(a[i])-int(b[i]));return d/a.size();};
    for(auto surface:{SurfaceKind::Regolith,SurfaceKind::HullPanels,SurfaceKind::InteriorPanels,SurfaceKind::RobotArmor,SurfaceKind::DeckPlating,SurfaceKind::FabricWeave,SurfaceKind::PressureDoor,SurfaceKind::CorridorDeck}){
        auto base=capture(surface,0,17,false,0),again=capture(surface,0,17,false,6),far=capture(surface,64,17,false,0),seed=capture(surface,0,381,false,0);
        require(base==again,"A static material flickers as time advances");
        require(difference(base,far)>.05 && difference(base,seed)>.05,"Material repeats at a distant position or ignores its seed");
    }
    auto attached=capture(SurfaceKind::RobotArmor,0,17,true,0),moved=capture(SurfaceKind::RobotArmor,64,17,true,0);
    require(difference(attached,moved)<.12,"UV surface detail slides while an object moves");
    auto cells=capture(SurfaceKind::PhotovoltaicCells,0,17,true,0),shifted_cells=capture(SurfaceKind::PhotovoltaicCells,64,17,true,0);
    require(difference(cells,shifted_cells)<.12,"Photovoltaic cells drift under a translated panel");
    auto panel=capture(SurfaceKind::Console,0,19,true,0),animated=capture(SurfaceKind::Console,0,19,true,5);
    require(panel==animated,"Display substrate fabricates changing telemetry");
    std::cout<<"PASS aperiodic world/seed variation, temporal stability, attached robot UVs and stable display substrates\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
