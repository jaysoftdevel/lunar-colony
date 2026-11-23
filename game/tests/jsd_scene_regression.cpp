#include "game/game_app.hpp"
#include "game/world/corridor/detail.hpp"
#include "game/world/robot_workshop.hpp"
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
        if(const char* save=std::getenv("GAME_REPLAY_SAVE")){
            std::string error;
            if(!app.load_game_from_file(save,error))throw std::runtime_error(error);
            app.on_resize(960,540);app.screen_=GameApp::Screen::Playfield;
            std::cout<<"save graphics path="<<int(app.graphics_.path)<<" layout="<<int(app.world_layout_mode_)<<" camera distance="<<app.camera_.distance()<<std::endl;
            vl::GL replay_gl;replay_gl.load([&](const char* name){return platform.resolve(name);});
            for(int f=0;f<12;++f){
                app.input_.mouse_x=480+f;app.input_.mouse_y=270;
                auto start=std::chrono::steady_clock::now();
                app.render(*renderer);replay_gl.Finish();
                std::cout<<"save render "<<f<<" ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<std::endl;
            }
            return 0;
        }

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
        PlacedResource hangar;hangar.kind=ResourceKind::RobotWorkshop;hangar.position={-18,0,-16};hangar.construction.finish();app.placed_resources_.push_back(hangar);
        app.show_fps_=true;app.fps_=60;
        app.builder_robots_.resize(1);app.builder_robots_[0].position={-6,0,3};
        app.ground_resource_piles_={{MaterialResource::Iron,500,{-3,0,4},.5f,1}};
        vl::GL gl;gl.load([&](const char* name){return platform.resolve(name);});
        const std::array names{"menu","orbit","first-person-exterior","first-person-interior","campus","graphics-menu","lab-interior","airlock-interior","biodome-interior","robot-closeup","fov-options","sky-east","sky-west","workshop-front","workshop-rear","workshop-open","workshop-glass-roof","workshop-service-interior","workshop-grounded-frame","corridor-T-exterior","corridor-X-interior","corridor-dome-collar","faded-hulls","fpv-build-preview","solar-filter-hulls","oxygen-terminal","pressure-door-detail","workshop-terminal","gate-unused-red","gate-connected-green","solar-collector-morning","solar-collector-noon"};
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
            if(scene>=11){
                app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                if(scene<=12){app.first_person_.spawn({0,0,14},scene==11?0.f:125.f);game::runtime::InputState look;look.mouse_dy=-500;app.first_person_.update_look(look);}
                else if(scene<=18){
                    app.first_person_.spawn(scene==14?game::runtime::Vec3{-18,0,-3}:game::runtime::Vec3{-18,0,-30},scene==14?0.f:180.f);
                    app.placed_resources_[1].workshop_gate_open=scene==15?1.f:0.f;
                }
            }
            if(scene>=16 && scene<=18){
                auto& home=app.placed_resources_[1];home.supply.power_fraction=1;
                UtilityConnection feed;feed.kind=UtilityKind::SupplyChannel;feed.construction.finish();
                feed.endpoint_a.kind=feed.endpoint_b.kind=UtilityEndpointKind::Resource;feed.endpoint_a.index=0;feed.endpoint_b.index=1;
                feed.path={app.placed_resources_[0].position,home.position};app.utility_network_.restore_connections({feed});
                app.builder_robots_.resize(4);
                for(int slot=0;slot<4;++slot){auto& robot=app.builder_robots_[slot];robot.position=workshop::bay(home,slot);robot.workshop_index=1;robot.workshop_bay=slot;robot.workshop_parked=true;robot.condition=.55f;robot.battery_capacity_kwh=10;robot.battery_charge_kwh=3;}
                app.first_person_mode_=GameApp::FirstPersonMode::Off;
                app.camera_.restore({22,55,35,home.position});
                app.habitat_view_mode_=scene==18?GameApp::HabitatViewMode::None:GameApp::HabitatViewMode::FullHull;
                if(scene==17){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn(home.position+game::runtime::Vec3{0,workshop::kDeck,-3.7f},180);}
            }
            if(scene>=19){
                app.placed_habitats_.clear();app.placed_resources_.clear();app.builder_robots_.clear();app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;
                const auto tube=[](std::vector<game::runtime::Vec3> p){CorridorConnection c;c.path=std::move(p);c.construction.finish();c.supply.power_fraction=1;c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.tangent=game::runtime::normalize(c.path[1]-c.path[0]);c.endpoint_b.tangent=c.endpoint_a.tangent*-1.f;return c;};
                auto main=tube({{-16,0,0},{16,0,0}}),branch=tube({{0,0,scene==19?1.87f:-14.f},{0,0,14}});
                if(scene==19){branch.endpoint_a.kind=CorridorEndpointKind::Corridor;branch.endpoint_a.index=0;branch.endpoint_a.surface_matched_junction=true;branch.endpoint_a.host_position={0,0,0};branch.endpoint_a.host_tangent={1,0,0};branch.endpoint_a.tangent={0,0,1};}
                std::vector<CorridorConnection> links{main,branch};app.first_person_mode_=GameApp::FirstPersonMode::Off;app.camera_.restore({27,50,35,{0,0,0}});
                if(scene==20){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,geometry::kCorridorDeckY,-6},180);}
                if(scene==21){PlacedHabitat dome;dome.kind=HabitatKind::BioDome;dome.rotation_degrees=27;dome.construction.finish();dome.supply.power_fraction=1;app.placed_habitats_={dome};app.corridor_network_.clear();
                    const auto direction=corridor_detail::rotate_y({1,0,0},27);const CorridorBuildAnchor source{CorridorEndpointKind::Habitat,0,direction*30};
                    const auto ep=app.corridor_network_.preview_anchor(source,direction*40,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);if(!ep)throw std::runtime_error("Collar scene has no endpoint");
                    main=tube({ep->position,ep->position+ep->tangent*9});main.endpoint_a=*ep;links={main};app.camera_.restore({24,40,55,ep->door.wall_position});
                }
                corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(std::move(links));
            }
            if(scene>=22){
                app.placed_habitats_={room};app.placed_resources_={hangar};app.corridor_network_.clear();
                app.first_person_cursor_=false;app.habitat_view_mode_=GameApp::HabitatViewMode::None;
                app.first_person_mode_=GameApp::FirstPersonMode::Off;app.camera_.restore({40,55,30,{-5,0,-5}});
                if(scene==23){
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({18,0,22},0);
                    game::runtime::InputState look;look.mouse_dy=85;app.first_person_.update_look(look);
                    const auto& items=app.build_catalog_.items();
                    for(std::size_t i=0;i<items.size();++i)if(items[i].id=="solar_panel"){
                        app.build_catalog_.set_category(items[i].category);app.build_catalog_.select(int(i));break;
                    }
                    app.input_.mouse_x=480;app.input_.mouse_y=270;
                }
            }
            if(scene>=24){
                app.build_catalog_.clear_selection();app.first_person_cursor_=false;app.first_person_alt_cursor_=false;
                app.corridor_network_.clear();app.utility_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();
                app.placed_resources_.clear();app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;
                auto habitat=room;habitat.kind=scene==26?HabitatKind::AirLock:HabitatKind::OxygenGeneratorDome;
                habitat.supply.oxygen_produced_kg_per_day=12.3;habitat.supply.water_processed_litres_per_day=13.8;
                habitat.supply.oxygen_kg=habitat.supply.volume_m3*kOxygenKgPerM3*.86;
                app.placed_habitats_={habitat};app.first_person_mode_=GameApp::FirstPersonMode::Off;
                app.camera_.restore({24,45,40,{0,1,0}});
                if(scene==25){
                    const auto& model=app.habitat_models_.mesh(habitat);const auto& display=model.screens.back();
                    const auto normal=display.normal();const auto eye=display.center+normal*.60f;
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;
                    app.first_person_.spawn(eye-game::runtime::Vec3{0,geometry::kHumanEyeHeight,0},std::atan2(-normal.x,normal.z)*180/3.14159265f);
                }
                if(scene==26){
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;
                    app.first_person_.spawn({-.5f,app.habitat_models_.mesh(habitat).walkable_floor_y,0},90);
                    game::runtime::InputState look;look.mouse_dy=100;app.first_person_.update_look(look);
                }
                if(scene==27){
                    app.placed_habitats_.clear();auto home=hangar;home.position={0,0,0};home.supply.power_fraction=1;
                    auto solar=panel;solar.position={-20,0,0};app.placed_resources_={home,solar};
                    UtilityConnection feed;feed.kind=UtilityKind::SupplyChannel;feed.construction.finish();feed.endpoint_a.kind=feed.endpoint_b.kind=UtilityEndpointKind::Resource;
                    feed.endpoint_a.index=0;feed.endpoint_b.index=1;feed.path={{-5.5f,0,0},{-17,0,0}};app.utility_network_.restore_connections({feed});
                    app.builder_robots_.resize(1);auto& bot=app.builder_robots_[0];bot.workshop_index=0;bot.workshop_bay=0;bot.workshop_parked=true;bot.condition=.65f;bot.battery_capacity_kwh=10;bot.battery_charge_kwh=3;bot.position=workshop::bay(home,0);
                    const auto bay=workshop::bay_local(0);const float sign=bay.x<0?-1.f:1.f;
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;
                    app.first_person_.spawn({sign*3.25f,2.40f-geometry::kHumanEyeHeight,bay.z+.72f},sign<0?270:90);
                }
                app.input_.mouse_x=480;app.input_.mouse_y=270;
            }
            if(scene==28 || scene==29){
                app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
                PlacedResource warehouse;warehouse.kind=ResourceKind::StorageDome;warehouse.construction.finish();
                app.placed_resources_={warehouse};
                if(scene==29){
                    auto destination=room;destination.position={0,0,-25};app.placed_habitats_={destination};
                    const CorridorBuildAnchor a{CorridorEndpointKind::Resource,0,{0,0,-6}},b{CorridorEndpointKind::Habitat,0,{0,0,-19}};
                    auto link=app.corridor_network_.preview_built_corridor(a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
                    if(!link)throw std::runtime_error("Gate capture has no corridor route");
                    link->construction.finish();std::vector<CorridorConnection> links{*link};corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(std::move(links));
                }
                app.first_person_mode_=GameApp::FirstPersonMode::Active;
                app.first_person_.spawn({0,app.resource_models_.mesh(warehouse).walkable_floor_y,.2f},0);
            }
            if(scene==30 || scene==31){
                app.placed_habitats_.clear();app.corridor_network_.clear();
                auto collector=panel;collector.position={0,0,0};collector.rotation_degrees=37;collector.solar_visual_initialized=true;
                collector.solar_visual_normal=solar_tracking::pose(scene==30?9.f:12.f).direction;
                app.placed_resources_={collector};app.first_person_mode_=GameApp::FirstPersonMode::Off;
                app.camera_.restore({15,38,155,{0,3,0}});
            }
            for(int frame=0;frame<3;++frame){app.ui_time_seconds_+=1./60;app.render(*renderer);}
            std::vector<unsigned char> pixels(960*540*4);gl.ReadPixels(0,0,960,540,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
            unsigned low=255,high=0;
            for(int y=100;y<440;++y)for(int x=100;x<860;++x){const auto i=(y*960+x)*4;const unsigned value=(unsigned(pixels[i])+pixels[i+1]+pixels[i+2])/3;low=std::min(low,value);high=std::max(high,value);}
            if(high-low<12)throw std::runtime_error(std::string("Scene has no rendered contrast: ")+names[scene]);
            if(scene==28 || scene==29){
                int border_pixels=0;int low_y=540,high_y=0;
                for(int y=80;y<460;++y)for(int x=300;x<660;++x){const auto i=(y*960+x)*4;
                    const bool border=scene==28?pixels[i]>160&&pixels[i]>pixels[i+1]*3: pixels[i+1]>160&&pixels[i+1]>pixels[i]*3;
                    if(border){++border_pixels;low_y=std::min(low_y,y);high_y=std::max(high_y,y);}
                }
                if(border_pixels<400||high_y-low_y<150)throw std::runtime_error(std::string("Gate border is hidden by its frame: ")+names[scene]);
            }
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
