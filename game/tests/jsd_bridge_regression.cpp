#include "jsd_bridge.hpp"
#include <frontier/detail/gl.hpp>
#include <iostream>
#include <stdexcept>
#include <string_view>
using namespace game::runtime;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char** argv){
    try{
        for(float pitch:{-82.f,-35.f,0.f,55.f,89.5f,90.f})for(float yaw:{0.f,45.f,180.f,275.f}){
            const Vec3 eye{124,12,-88};
            const auto forward=normalize(Vec3{std::sin(radians(yaw))*std::cos(radians(pitch)),std::sin(radians(pitch)),-std::cos(radians(yaw))*std::cos(radians(pitch))});
            auto up=std::abs(forward.y)>.9999f?Vec3{0,0,-1}:Vec3{0,1,0};
            const auto projection=perspective(radians(68),16.f/9,.035f,5000);
            const auto vp=multiply(projection,look_at(eye,eye+forward,up));
            const auto decoded=jsd_bridge::decode_view(vp);
            require(rf::distance(decoded.camera.position,jsd_bridge::vector(eye))<.002f,"Camera eye reconstruction changed");
            require(std::abs(decoded.camera.nearPlane-.035f)<.001f,"First-person near plane changed");
            const auto view=rf::lookAt(decoded.camera.position,decoded.camera.position+decoded.camera.direction);
            rf::Mat4 p;p.m=projection.m;
            const auto rebuilt=p*view*decoded.correction;
            for(Vec3 point:{eye+forward*2,eye+forward*20+Vec3{2,1,0}}){
                rf::Mat4 original;original.m=vp.m;
                require(rf::distance(rf::transformPoint(original,jsd_bridge::vector(point)),rf::transformPoint(rebuilt,jsd_bridge::vector(point)))<.004f,"Camera projection differs at a pole");
            }
        }
        const auto settings=jsd_bridge::settings(balanced_graphics_settings());
        require(settings.depthOfField==rf::gfx::Quality::Off && settings.motionBlur==rf::gfx::Quality::Off,"First-person optics must stay sharp");
        require(settings.fogDensity==0 && settings.viewDistance>=5000,"Lunar vacuum and orbit range changed");
        if(argc>1 && std::string_view(argv[1])=="--render"){
            rf::Platform platform(true,640,360);
            auto renderer=jsd_bridge::create_renderer(platform);
            auto options=balanced_graphics_settings();options.set_quality(GraphicsFeature::TerrainRelief,1);options.ground_clutter=0;
            renderer->set_graphics_settings(options);
            vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
            for(int i=0;i<6;++i){
                renderer->begin_frame({.01f,.02f,.03f,1});
                const Vec3 eye=i<3?Vec3{0,1.7f,5}:Vec3{0,24,.01f};
                const auto vp=multiply(perspective(radians(68),16.f/9,.035f,5000),look_at(eye,{0,1,0},{0,1,0}));
                renderer->set_3d(vp);
                LunarScene scene{vp,eye,{0,1,0},{.4f,.8f,.3f},.8f,32};scene.first_person=i<3;scene.seconds=i/60.f;
                scene.lights[0]={{0,2,2},{.35f,.7f,1,1},12,40};scene.light_count=1;
                require(renderer->begin_lunar_scene(scene),"External engine scene did not start");
                renderer->draw_lunar_surface({{-1,0,0},{1,0,0},{1,3,0},{-1,0,0},{1,3,0},{-1,3,0}},LunarMaterial::Alloy,{.5f,.6f,.7f,1});
                renderer->set_2d(1.5f);renderer->draw_rect({8,8,40,20},{1,0,0,1});renderer->draw_text({8,40},"JSD FIRST PERSON",1.2f,{1,1,1,1});renderer->end_frame();
                std::array<unsigned char,4> pixel{};gl.ReadPixels(20,340,1,1,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixel.data());
                require(pixel[0]>240 && pixel[1]<10,"UI scaling or postprocessing order changed");
                require(gl.GetError()==0,"OpenGL rendering error");
            }
            // Exercise same-size edits within a single spatial/material batch.
            options.path=GraphicsPath::Classic;
            options.aa=GraphicsAA::Off;
            options.bloom=0;
            renderer->set_graphics_settings(options);
            const Vec3 eye{4,3,9};
            const auto panel_view=multiply(perspective(radians(62),16.f/9,.05f,500),
                                           look_at(eye,{4,3,4},{0,1,0}));
            std::vector<Vec3> panel{{2,1,4},{6,1,4},{6,5,4},{2,1,4},{6,5,4},{2,5,4}};
            LunarSurfaceMapping basis{};
            const auto panel_frame=[&](Color tint){
                renderer->begin_frame({0,0,0,1});
                renderer->set_3d(panel_view);
                renderer->draw_mapped_lunar_surface(panel,LunarMaterial::Hull,tint,basis);
                renderer->end_frame();
                std::vector<unsigned char> pixels(320*240*4);
                gl.ReadPixels(160,60,320,240,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
                return pixels;
            };
            const Color silver{.8f,.72f,.65f,1};
            const auto original=panel_frame(silver);
            require(panel_frame(silver)==original,"Repeated batch changed pixels");
            require(panel_frame({.15f,.7f,.2f,1})!=original,"Tint edit reused stale vertices");
            basis.origin.x=.37f;
            require(panel_frame(silver)!=original,"UV basis edit reused stale vertices");
            basis.origin.x=0;
            require(panel_frame(silver)==original,"Restored mapping did not restore the panel");
            const auto geometry=panel;
            std::fill(panel.begin(),panel.end(),panel.front());
            require(panel_frame(silver)!=original,"Collapsed triangles retained the old mesh");
            panel=geometry;
            require(panel_frame(silver)==original,"Restored vertices did not restore the panel");
            // Switching from a world into menus must clear stale world color/history.
            renderer->begin_frame({0,0,0,1});renderer->set_2d();renderer->draw_rect({0,0,640,360},{0,1,0,1});renderer->end_frame();
            std::array<unsigned char,4> pixel{};gl.ReadPixels(320,180,1,1,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixel.data());
            require(pixel[1]>240 && pixel[0]<10,"World-to-menu transition retained a stale frame");
        }
        std::cout<<"PASS JSD projection, optics and renderer integration\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
