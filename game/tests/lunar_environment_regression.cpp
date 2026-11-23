#include "game/game_app.hpp"
#include "jsd_bridge.hpp"
#include <frontier/detail/gl.hpp>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace game {
struct EngineIntegrationRegression {
    static int run(){
        using namespace runtime;
        constexpr int width=960,height=540;
        rf::Platform platform(true,width,height);
        auto renderer=jsd_bridge::create_renderer(platform);
        GameHost host;GameApp app(host);
        app.playfield_.rebuild(64);app.camera_.reset(64);app.status_.reset();
        app.screen_=GameApp::Screen::Playfield;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.game_speed_=GameApp::GameSpeed::Normal;
        app.graphics_.path=GraphicsPath::Classic;app.graphics_.features=0;
        app.graphics_.ground_clutter=0;app.graphics_.surface_resolution=1;
        app.graphics_.shadows=GraphicsShadows::Off;app.graphics_.aa=GraphicsAA::Off;app.graphics_.bloom=0;
        vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
        const auto capture=[&](const char* label){
            app.render(*renderer);
            std::vector<unsigned char> pixels(width*height*4);
            gl.ReadPixels(0,0,width,height,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
            if(const char* folder=std::getenv("GAME_RENDER_CAPTURE_DIR")){
                std::filesystem::create_directories(folder);
                std::ofstream out(std::filesystem::path(folder)/(std::string(label)+".ppm"),std::ios::binary);
                out<<"P6\n"<<width<<' '<<height<<"\n255\n";
                for(int y=height-1;y>=0;--y)for(int x=0;x<width;++x)
                    out.write(reinterpret_cast<const char*>(pixels.data()+(y*width+x)*4),3);
            }
            return pixels;
        };
        const auto bright=[](const auto& pixels,int x0,int x1,int y0,int y1){
            int count=0;
            for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){const auto i=(y*width+x)*4;
                if(std::max({pixels[i],pixels[i+1],pixels[i+2]})>20)++count;
            }
            return count;
        };
        for(const auto path:{GraphicsPath::Classic,GraphicsPath::Auto}){
            app.graphics_.path=path;
            const auto prefix=path==GraphicsPath::Classic?"classic-":"auto-";
            app.first_person_.spawn({0,0,0});InputState look;look.mouse_dy=-600;app.first_person_.update_look(look);
            const auto sky=capture((std::string(prefix)+"stars").c_str());
            const auto stars=bright(sky,420,700,140,400);
            if(stars<15 || stars>280*260/3)throw std::runtime_error(std::string(prefix)+"full graphics lost its sparse star sky");
            // Aim at ground 200 m outside the buildable square. The low-detail
            // global plane must cover this view even with every effect off.
            app.first_person_.spawn({232,20,0});look.mouse_dy=600;app.first_person_.update_look(look);
            const auto ground=capture((std::string(prefix)+"outside-surface").c_str());
            if(bright(ground,440,700,90,200)<260*110*9/10)
                throw std::runtime_error(std::string(prefix)+"full graphics lost the global surface beyond the playfield");
            std::cout<<"PASS "<<prefix<<"lowest settings: "<<stars<<" star pixels and continuous terrain outside the playfield\n";
        }
        return 0;
    }
};
}
int main(){try{return game::EngineIntegrationRegression::run();}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
