#include <frontier/renderer.hpp>
#include <frontier/detail/gl.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace rf;using namespace rf::gfx;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 Platform platform(true,640,360);auto settings=GraphicsSettings::minimal();settings.antiAliasing=AntiAliasing::Off;
 Renderer renderer(platform,settings);vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
 Camera camera;camera.position={0,2,0};camera.direction=normalized(V3{.1f,.4f,-1});camera.fovDegrees=68;
 Environment sky;sky.sunDirection={0,-1,0};sky.skyColor={0,0,0,1};sky.fogDensity=0;sky.stars.enabled=true;sky.stars.intensity=5;
 auto capture=[&]{renderer.begin(camera,sky);renderer.end();renderer.check();std::vector<unsigned char> pixels(640*360*4);gl.ReadPixels(0,0,640,360,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());return pixels;};
 auto difference=[](const auto& a,const auto& b){double total=0;for(std::size_t i=0;i<a.size();++i)if(i%4!=3)total+=std::abs(int(a[i])-int(b[i]));return total/(a.size()*.75);};
 auto base=capture();unsigned lit=0;for(std::size_t i=0;i<base.size();i+=4)if(std::max({base[i],base[i+1],base[i+2]})>15)++lit;
 require(lit>25 && lit<640*360/3,"Vacuum sky has no visible, sparse star catalog");
 camera.position={800,30,-1200};sky.atmosphere.time=100;auto translated=capture();require(difference(base,translated)<.03,"Infinite sky moves with camera translation or time");
 camera.direction=normalized(V3{.6f,.4f,-1});auto turned=capture();require(difference(base,turned)>.02,"Stars remain attached to screen during camera rotation");
 camera.direction=normalized(V3{.1f,.4f,-1});camera.fovDegrees=100;require(difference(base,capture())>.02,"Sky does not respect first-person FOV");
 sky.atmosphere.enabled=true;sky.sunDirection=normalized(V3{.2f,.8f,.1f});auto daytime=capture();sky.stars.enabled=false;require(difference(daytime,capture())<.03,"Earth daylight does not extinguish stars");
 // Recreate at a square aspect. A vertical-FOV camera must retain its central
 // sky directions rather than stretching a screen-space star texture.
 camera.position={0,2,0};camera.fovDegrees=68;sky.atmosphere.enabled=false;sky.sunDirection={0,-1,0};sky.stars.enabled=true;auto wide=capture();
 platform.width=360;platform.height=360;renderer.begin(camera,sky);renderer.end();renderer.check();std::vector<unsigned char> square(360*360*4);gl.ReadPixels(0,0,360,360,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,square.data());
 double error=0;for(int y=0;y<360;++y)for(int x=0;x<360;++x)for(int c=0;c<3;++c)error+=std::abs(int(wide[(y*640+x+140)*4+c])-int(square[(y*360+x)*4+c]));
 require(error/(360*360*3)<.15,"Viewport aspect stretches or shifts the spherical star layout");
 std::cout<<"PASS world-oriented sky sphere, no parallax/flicker, yaw/FOV/aspect and daylight extinction ("<<lit<<" star pixels)\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
