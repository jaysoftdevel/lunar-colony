#include "game/game_app.hpp"
#include "jsd_bridge.hpp"
#include <frontier/detail/gl.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace game {
struct EngineIntegrationRegression {
    static int run(){
        rf::Platform platform(true,960,540);
        auto renderer=game::runtime::jsd_bridge::create_renderer(platform);
        game::runtime::GameHost host;
        GameApp app(host);
        app.on_resize(960,540);app.game_speed_=GameApp::GameSpeed::Normal;
        app.graphics_=game::runtime::balanced_graphics_settings();app.graphics_.ground_clutter=1;
        if(const char* feature=std::getenv("GAME_RENDER_FEATURE_OFF")){
            if(std::string(feature)=="shadows")app.graphics_.shadows=game::runtime::GraphicsShadows::Off;
            if(std::string(feature)=="ao")app.graphics_.set(game::runtime::GraphicsFeature::ScreenOcclusion,false);
            if(std::string(feature)=="gi")app.graphics_.set(game::runtime::GraphicsFeature::ScreenIndirect,false);
        }
        app.playfield_.rebuild(64);app.camera_.reset(64);app.resources_.reset();app.status_.reset();
        app.game_session_active_=true;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.input_.mouse_x=app.input_.mouse_y=-100;
        PlacedHabitat room;room.kind=HabitatKind::CrewQuarters;room.position={0,0,0};room.construction.step=kConstructionSteps;
        room.supply.volume_m3=150;room.supply.power_fraction=1;room.power_enabled=true;
        app.placed_habitats_.push_back(room);
        PlacedResource panel;panel.kind=ResourceKind::SolarPanel;panel.position={-10,0,0};panel.construction.step=kConstructionSteps;
        app.placed_resources_.push_back(panel);
        app.builder_robots_.resize(1);app.builder_robots_[0].position={-6,0,3};
        app.ground_resource_piles_={{MaterialResource::Iron,500,{-3,0,4},.5f,1}};
        vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
        const std::array names{"menu","orbit","first-person-exterior","first-person-interior","campus","graphics-menu","lab-interior","airlock-interior","biodome-interior","robot-closeup","fov-options"};
        for(std::size_t scene=0;scene<names.size();++scene){
            app.screen_=scene==0?GameApp::Screen::MainMenu:GameApp::Screen::Playfield;
            app.first_person_mode_=GameApp::FirstPersonMode::Off;
            if(scene==2 || scene==3){
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                app.first_person_.spawn(scene==2?game::runtime::Vec3{0,0,14}:game::runtime::Vec3{0,app.habitat_models_.mesh(room).walkable_floor_y,0},scene==2?0:45);
            }
            if(scene==4){app.active_game_phase_=GameApp::GamePhase::Planning;app.planning_.view=PlanningView::Site;app.camera_.restore({95,47,0,{0,0,0}});}
            if(scene==5)app.screen_=GameApp::Screen::GraphicsSettings;
            if(scene>=6 && scene<=8){
                auto& h=app.placed_habitats_.front();h.kind=scene==6?HabitatKind::Laboratory:scene==7?HabitatKind::AirLock:HabitatKind::BioDome;
                app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                app.first_person_.spawn({0,app.habitat_models_.mesh(h).walkable_floor_y,0},45);
            }
            if(scene==9){
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                app.first_person_.spawn({-6,0,6},0);
            }
            if(scene==10)app.screen_=GameApp::Screen::Settings;
            for(int frame=0;frame<3;++frame){app.ui_time_seconds_+=1./60;app.render(*renderer);}
            std::vector<unsigned char> pixels(960*540*4);gl.ReadPixels(0,0,960,540,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
            unsigned low=255,high=0;
            for(int y=100;y<440;++y)for(int x=100;x<860;++x){const auto i=(y*960+x)*4;const unsigned value=(unsigned(pixels[i])+pixels[i+1]+pixels[i+2])/3;low=std::min(low,value);high=std::max(high,value);}
            if(high-low<12)throw std::runtime_error(std::string("Scene has no rendered contrast: ")+names[scene]);
            if(const char* folder=std::getenv("GAME_RENDER_CAPTURE_DIR")){
                std::filesystem::create_directories(folder);
                std::ofstream file(std::filesystem::path(folder)/(std::string(names[scene])+".ppm"),std::ios::binary);
                file<<"P6\n960 540\n255\n";
                for(int y=539;y>=0;--y)for(int x=0;x<960;++x)file.write(reinterpret_cast<const char*>(pixels.data()+(y*960+x)*4),3);
            }
            std::cout<<"PASS real game scene: "<<names[scene]<<'\n';
        }
        return 0;
    }
};
}
int main(){try{return game::EngineIntegrationRegression::run();}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
