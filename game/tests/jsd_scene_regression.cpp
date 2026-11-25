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
        const std::array names{"menu","orbit","first-person-exterior","first-person-interior","campus","graphics-menu","lab-interior","airlock-interior","biodome-interior","robot-closeup","fov-options","sky-east","sky-west","workshop-front","workshop-rear","workshop-open","workshop-glass-roof","workshop-service-interior","workshop-grounded-frame","corridor-T-exterior","corridor-X-interior","corridor-dome-collar","faded-hulls","fpv-build-preview","solar-filter-hulls","oxygen-terminal","pressure-door-detail","workshop-terminal","gate-unused-no-frame","gate-connected-green","solar-collector-morning","solar-collector-noon","gate-unpowered-red","empty-factory","configured-factory","corridor-deck-detail","furnished-canteen-day","canteen-night-idle","canteen-night-occupied","placeable-oxygen-generator","storage-no-unused-doors","fpv-astronaut-vitals","classified-solid-packages","frame-airlock","frame-oxygen","frame-canteen","frame-biodome","frame-crew","frame-processing","frame-factory","frame-manufactory","frame-sickbay","frame-lab","inside-frame-airlock","inside-frame-oxygen","inside-frame-canteen","inside-frame-biodome","inside-frame-crew","inside-frame-processing","inside-frame-factory","inside-frame-manufactory","inside-frame-sickbay","inside-frame-lab","frame-storage","frame-workshop","frame-corridor","frame-junction","frame-exterior-equipment","airlock-seal-closed","airlock-seal-open","corridor-bend-grounded","corridor-cassette-close","oxygen-clear-service-layout","save-name-editor","save-existing-sidebar","airlock-open-transfer","deceased-player-recovery","fpv-oxygen-critical","fpv-multiple-critical","fpv-critical-dim","fpv-critical-max-zoom","hatch-powered-E","hatch-manual-progress","hatch-manual-interlock","hatch-common-layout","oxygen-generator-very-small","oxygen-generator-small","oxygen-generator-medium","saved-oxygen-clear-cutaway","factory-tangent-machines","crew-dome-habitat","laboratory-dome-habitat","cutaway-crew-low-walls","cutaway-biodome-low-walls","cutaway-storage-low-walls","cutaway-workshop-low-walls","fitted-airlock-cutaway","fitted-airlock-pressure-system","fixed-airlock-corridor","small-airlock-clear-transfer","rover-stowed-model","rover-unfold-start","rover-unfold-middle","rover-unfold-complete","rover-ramp-traffic","robot-independent-crew","selected-airlock-full-hull","airlock-built-free-stairs","airlock-fitted-end-collar","elevated-corridor-ground-supports","autosave-gameplay-options"};
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
            if(scene==28 || scene==29 || scene==32){
                app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();
                PlacedResource warehouse;warehouse.kind=ResourceKind::StorageDome;warehouse.construction.finish();warehouse.supply.power_fraction=scene==32?0:1;
                app.placed_resources_={warehouse};
                if(scene==29 || scene==32){
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
            if(scene==33 || scene==34){
                app.placed_resources_.clear();app.placed_habitats_.clear();app.corridor_network_.clear();
                auto hall=room;hall.kind=HabitatKind::Factory;app.placed_habitats_={hall};
                app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.first_person_mode_=GameApp::FirstPersonMode::Off;
                if(scene==34){
                    const float deck=app.habitat_models_.mesh(hall).walkable_floor_y;
                    InteriorUnit metal;metal.kind=InteriorUnitKind::MetalFabricator;metal.position={-3,deck,-2};metal.progress=.45;metal.batch_active=true;
                    InteriorUnit plastic;plastic.kind=InteriorUnitKind::PlasticFabricator;plastic.position={-3,deck,2};plastic.rotation_degrees=180;
                    app.placed_habitats_[0].units={metal,plastic};
                }
                app.focus_interior(0);
            }
            if(scene==35){
                app.placed_resources_.clear();app.placed_habitats_.clear();app.corridor_network_.clear();app.clear_world_selection();
                CorridorConnection run;run.path={{0,0,-15},{0,0,15}};run.construction.finish();run.supply.power_fraction=1;
                run.endpoint_a.kind=run.endpoint_b.kind=CorridorEndpointKind::OpenDoor;run.endpoint_a.position=run.path.front();run.endpoint_b.position=run.path.back();run.endpoint_a.tangent={0,0,-1};run.endpoint_b.tangent={0,0,1};
                std::vector<CorridorConnection> runs{run};corridor_detail::rebuild_all_meshes(runs);app.corridor_network_.restore_connections(std::move(runs));
                app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,geometry::kCorridorDeckY,4},0);
                game::runtime::InputState look;look.mouse_dy=210;app.first_person_.update_look(look);
            }
            if(scene>=36){
                app.placed_resources_.clear();app.placed_habitats_.clear();app.corridor_network_.clear();app.utility_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();app.clear_world_selection();
                app.first_person_mode_=GameApp::FirstPersonMode::Off;app.habitat_view_mode_=GameApp::HabitatViewMode::None;
                auto clock=app.status_.state();clock.hour=scene==36||scene==39?12:0;clock.crew_current=0;app.status_.restore(clock);
                auto hall=room;hall.kind=scene==39?HabitatKind::OxygenGeneratorDome:HabitatKind::CantineDome;hall.size=HabitatSizeTier::Mid;hall.supply.power_fraction=1;
                app.placed_habitats_={hall};const float deck=app.habitat_models_.mesh(hall).walkable_floor_y;
                if(scene<=38){
                    const auto unit=[&](InteriorUnitKind kind,game::runtime::Vec3 p){InteriorUnit u;u.kind=kind;u.position=p;return u;};
                    app.placed_habitats_[0].units={unit(InteriorUnitKind::FoodProcessor,{-2.5f,deck,-2.3f}),unit(InteriorUnitKind::DrinkingWell,{2.5f,deck,-2.3f}),unit(InteriorUnitKind::DiningTable,{2.2f,deck,2.8f}),unit(InteriorUnitKind::SeatingBench,{2.2f,deck,1.6f}),unit(InteriorUnitKind::SeatingBench,{2.2f,deck,4.0f})};
                    app.placed_habitats_[0].units[0].food_kg=12;app.placed_habitats_[0].units[1].water_litres=8;
                    if(scene==38){app.builder_robots_.resize(1);app.builder_robots_[0].position={-2,deck,2};}
                }else if(scene==39)app.fit_oxygen_units();
                if(scene==40){app.placed_habitats_.clear();auto store=panel;store.kind=ResourceKind::StorageDome;store.position={0,0,0};store.supply.power_fraction=1;store.construction.finish();app.placed_resources_={store};}
                if(!app.placed_habitats_.empty()){app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.focus_interior(0);}
                else app.camera_.restore({23,35,45,{0,1,0}});
            }

            if(scene>=41){
                app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.utility_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();app.clear_world_selection();app.reset_first_person_crew();
                auto clock=app.status_.state();clock.hour=10;app.status_.restore(clock);
                app.lander_.phase=LanderPhase::Ready;app.lander_.site={-8,0,-6};
                app.enter_first_person_mode();app.first_person_.spawn({0,0,4},0);app.first_person_.restore_view(0,-25);app.sync_controlled_astronaut(0);
                app.test_astronauts_[0].hunger=.34f;app.test_astronauts_[0].thirst=.22f;app.test_astronauts_[0].fatigue=.5f;app.test_astronauts_[0].suit_oxygen_kg=.21;
                for(int material=0;material<int(MaterialResource::Count);++material){
                    GroundResourcePile box;box.resource=MaterialResource(material);box.amount=resource_package_capacity_kg(box.resource);box.kind=GroundPileKind::Stack;
                    box.position={-2.1f+(material%4)*1.4f,0,-.3f-(material/4)*1.4f};app.ground_resource_piles_.push_back(box);
                }
                if(scene==42){app.first_person_.spawn({0,0,1.8f},0);app.first_person_.restore_view(0,-40);app.sync_controlled_astronaut(0);app.exit_first_person_mode();app.reset_first_person_crew();app.first_person_mode_=GameApp::FirstPersonMode::Active;}
                app.input_.mouse_x=480;app.input_.mouse_y=270;
            }

            if(scene>=43){
                app.screen_=GameApp::Screen::Playfield;app.input_={};
                app.reset_first_person_crew();app.lander_={};app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.utility_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();app.clear_world_selection();
                app.first_person_mode_=GameApp::FirstPersonMode::Off;app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;
                auto clock=app.status_.state();clock.hour=10;app.status_.restore(clock);
                app.camera_.restore({25,35,38,{0,2,0}});
                if(scene<=62){
                    auto h=room;h.kind=HabitatKind((scene-43)%10);h.size=HabitatSizeTier::Standard;app.placed_habitats_={h};
                    if(scene>=53){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,app.habitat_models_.mesh(h).walkable_floor_y,0},45);app.first_person_.restore_view(45,24);}
                }
                if(scene==63||scene==64){auto h=hangar;h.kind=scene==63?ResourceKind::StorageDome:ResourceKind::RobotWorkshop;h.position={0,0,0};app.placed_resources_={h};app.camera_.restore({23,35,38,{0,2,0}});}
                if(scene==65||scene==66){
                    const auto run=[](game::runtime::Vec3 a,game::runtime::Vec3 b){CorridorConnection c;c.path={a,b};c.construction.finish();c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=a;c.endpoint_b.position=b;c.endpoint_a.tangent=game::runtime::normalize(a-b);c.endpoint_b.tangent=game::runtime::normalize(b-a);return c;};
                    std::vector<CorridorConnection> links{run({0,0,-12},{0,0,12})};if(scene==66)links.push_back(run({-12,0,0},{12,0,0}));corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(links);
                    if(scene==65){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,geometry::kCorridorDeckY,7},0);app.first_person_.restore_view(0,12);}
                    else app.camera_.restore({32,42,38,{0,1,0}});
                }
                if(scene==67){
                    int j=0;for(auto kind:{ResourceKind::WaterDriller,ResourceKind::PowerStorage,ResourceKind::WaterStorage,ResourceKind::OreDriller,ResourceKind::WaterPurifier,ResourceKind::HydrogenGenerator,ResourceKind::HydrogenTank,ResourceKind::OxygenTank,ResourceKind::SolarPanel,ResourceKind::Antenna}){
                        auto h=panel;h.kind=kind;h.position={float(j%5-2)*17,0,float(j/5)*18-9};h.construction.finish();app.placed_resources_.push_back(h);++j;}
                    app.camera_.restore({76,50,32,{0,3,0}});
                }
                if(scene==68||scene==69){
                    auto h=room;h.kind=HabitatKind::AirLock;h.supply.power_fraction=1;app.placed_habitats_={h};app.pressure_doors_.clear();app.airlock_cycles_.clear();
                    const float y=app.habitat_models_.mesh(h).walkable_floor_y;
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-.55f,y,-.16f},90);app.first_person_.restore_view(90,-8);
                    app.airlock_cycles_.resize(1);app.airlock_cycles_[0].pressure=1;
                    for(const auto& door:app.habitat_doors(0)){PressureDoorState state;state.door=door;state.powered=true;state.open_fraction=scene==69&&door.entrance_index==-3?1.f:0.f;app.pressure_doors_.push_back(state);}
                }
                if(scene==70||scene==71){
                    CorridorConnection c;c.path={{-9,0,-8},{-3,0,-8},{0,0,-5},{0,0,12}};c.construction.finish();c.supply.power_fraction=1;
                    c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.tangent={-1,0,0};c.endpoint_b.tangent={0,0,1};
                    std::vector<CorridorConnection> links{c};corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(links);
                    app.camera_.restore({30,18,62,{-2,.6f,0}});
                    if(scene==71){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({.12f,geometry::kCorridorDeckY,6},0);app.first_person_.restore_view(0,-44);}
                }
                if(scene==72){
                    auto h=room;h.kind=HabitatKind::OxygenGeneratorDome;h.size=HabitatSizeTier::Small;h.supply.power_fraction=1;app.placed_habitats_={h};app.fit_oxygen_units();
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,app.habitat_models_.mesh(h).walkable_floor_y,-1},180);app.first_person_.restore_view(180,6);
                }
                if(scene==73||scene==74){
                    app.screen_=GameApp::Screen::SaveGame;app.game_session_active_=true;app.save_games_.clear();
                    for(int i=0;i<12;++i){GameApp::SaveGameEntry entry;entry.name="Lunar colony - Sol "+std::to_string(i+1);entry.version="0.75.0";app.save_games_.push_back(entry);}
                    app.save_game_name_=scene==73?"Lunar colony - Sol 13":"Research base - pressure test";app.save_name_editor_.reset(app.save_game_name_);
                    if(scene==74){app.save_name_editor_.anchor=0;app.save_name_editor_.caret=13;}
                    app.save_caret_epoch_=app.ui_time_seconds_;app.save_name_focus_=true;app.save_name_scroll_=0;
                }
                if(scene==75){
                    auto lock=room;lock.kind=HabitatKind::AirLock;lock.supply.power_fraction=1;auto oxygen=room;oxygen.kind=HabitatKind::OxygenGeneratorDome;oxygen.position={20,0,0};app.placed_habitats_={lock,oxygen};
                    const CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,{5,0,0}},b{CorridorEndpointKind::Habitat,1,{15,0,0}};
                    auto link=app.corridor_network_.preview_built_corridor(a,b,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);if(!link)throw std::runtime_error("Airlock transfer has no connection");
                    link->construction.finish();std::vector<CorridorConnection> links{*link};corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(links);
                    app.pressure_doors_.clear();for(const auto& door:app.habitat_doors(0)){PressureDoorState state;state.door=door;state.open_fraction=1;state.powered=true;app.pressure_doors_.push_back(state);}
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-3,app.habitat_models_.mesh(lock).walkable_floor_y,0},90);app.first_person_.restore_view(90,-4);
                }
                if(scene==76){
                    std::string error;if(!app.load_game_from_file(GAME_STUCK_FPV_FIXTURE,error))throw std::runtime_error(error);
                    app.screen_=GameApp::Screen::Playfield;app.enter_first_person_mode();app.input_={};
                }
                if(scene>=77&&scene<=80){
                    app.placed_habitats_={room};app.first_person_mode_=GameApp::FirstPersonMode::Active;
                    app.first_person_.spawn({0,0,14},0);app.controlled_astronaut_=0;app.astronaut_views_[0].present=true;
                    auto& actor=app.test_astronauts_[0];actor={};actor.position=app.first_person_.feet_position();
                    actor.suit_oxygen_kg=kSuitOxygenCapacityKg*.12;
                    if(scene>=78){actor.health=.15f;actor.hunger=.93f;actor.thirst=.91f;actor.fatigue=kCrewExhausted*.94f;}
                    app.show_fps_=true;app.ui_scale_index_=scene==80?4:0;app.ui_time_seconds_=scene==79?.70:.20;
                }
                if(scene>=81&&scene<=84){
                    auto cabin=room;cabin.kind=HabitatKind::CrewQuarters;cabin.supply.power_fraction=scene==82?0:1;app.placed_habitats_={cabin};
                    const float deck=app.habitat_models_.mesh(cabin).walkable_floor_y;
                    CorridorConnection c;c.construction.finish();c.endpoint_a.kind=CorridorEndpointKind::Habitat;c.endpoint_a.index=0;
                    c.endpoint_a.door.wall_position=c.endpoint_a.door.position={0,deck,-2.6f};c.endpoint_a.door.normal={0,0,-1};c.endpoint_a.position=c.endpoint_a.door.position;
                    c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_b.position={0,deck,-9};c.path={c.endpoint_a.position,c.endpoint_b.position};app.corridor_network_.restore_connections({c});
                    app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,deck,-.5f},0);app.ui_scale_index_=scene==84?4:0;app.door_hold_={};
                    app.update_pressure_doors(0);app.game_speed_=GameApp::GameSpeed::Normal;
                    if(scene==82){game::runtime::InputState held;held.key_e_down=true;for(int i=0;i<15;++i)app.update_door_interaction(.1f,held);}
                    if(scene==83){
                        cabin.kind=HabitatKind::AirLock;cabin.supply.power_fraction=0;app.placed_habitats_={cabin};app.corridor_network_.clear();app.pressure_doors_.clear();
                        const float floor=app.habitat_models_.mesh(cabin).walkable_floor_y;app.first_person_.spawn({.3f,floor,0},90);app.update_pressure_doors(0);
                        app.pressure_doors_[0].open_fraction=1;game::runtime::InputState held;held.key_e_down=true;app.update_door_interaction(.1f,held);
                    }
                    if(scene==84){app.first_person_.spawn({0,deck,-1.f},0);app.first_person_.restore_view(0,0);}
                }
                if(scene>=85){
                    app.ui_scale_index_=0;app.screen_=GameApp::Screen::Playfield;app.game_speed_=GameApp::GameSpeed::Pause;
                    auto h=room;h.kind=HabitatKind::OxygenGeneratorDome;h.supply.power_fraction=1;h.size=HabitatSizeTier::Small;
                    if(scene==89)h.kind=HabitatKind::Factory;if(scene==90)h.kind=HabitatKind::CrewQuarters;if(scene==91)h.kind=HabitatKind::Laboratory;
                    app.placed_habitats_={h};app.habitat_view_mode_=scene<=89?GameApp::HabitatViewMode::None:GameApp::HabitatViewMode::FullHull;
                    app.camera_.restore({22,38,35,{0,2,0}});app.first_person_mode_=GameApp::FirstPersonMode::Off;
                    if(scene==88){std::string error;if(!app.load_game_from_file(GAME_OXYGEN_FIT_FIXTURE,error))throw std::runtime_error(error);
                        app.screen_=GameApp::Screen::Playfield;app.habitat_view_mode_=GameApp::HabitatViewMode::None;app.camera_.restore({20,48,32,app.placed_habitats_[1].position+game::runtime::Vec3{0,1.7f,0}});
                    }
                    const std::size_t host=scene==88?1:0;
                    if(scene<=88){InteriorUnit u;u.kind=InteriorUnitKind::OxygenGenerator;u.size=scene==85?InteriorUnitSize::VerySmall:scene==86?InteriorUnitSize::Small:InteriorUnitSize::Medium;if(!app.find_interior_unit_bay(host,u))throw std::runtime_error("Dome render has no oxygen bay");app.placed_habitats_[host].units.push_back(u);}
                    if(scene==89||scene==91)for(int i=0;i<3;++i){InteriorUnit u;u.kind=scene==91?InteriorUnitKind::MedicalSynthesizer:i%2?InteriorUnitKind::PlasticFabricator:InteriorUnitKind::MetalFabricator;
                        if(app.find_interior_unit_bay(host,u)){u.progress=.43;u.batch_active=true;app.placed_habitats_[host].units.push_back(u);}
                    }
                }
                if(scene>=92&&scene<=95){
                    app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.habitat_view_mode_=GameApp::HabitatViewMode::None;
                    app.first_person_mode_=GameApp::FirstPersonMode::Off;app.camera_.restore({20,42,35,{0,1,0}});
                    if(scene<94){auto h=room;h.kind=scene==92?HabitatKind::CrewQuarters:HabitatKind::BioDome;h.size=HabitatSizeTier::Standard;h.units.clear();app.placed_habitats_={h};}
                    else {PlacedResource cabin;cabin.kind=scene==94?ResourceKind::StorageDome:ResourceKind::RobotWorkshop;cabin.construction.finish();app.placed_resources_={cabin};}
                }
                if(scene>=96&&scene<=99){
                    app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.pressure_doors_.clear();
                    auto h=room;h.kind=HabitatKind::AirLock;h.size=scene==99?HabitatSizeTier::Small:HabitatSizeTier::Standard;h.units.clear();h.supply.power_fraction=1;h.construction.finish();app.placed_habitats_={h};
                    app.habitat_view_mode_=scene==96?GameApp::HabitatViewMode::None:GameApp::HabitatViewMode::FullHull;
                    app.first_person_mode_=GameApp::FirstPersonMode::Off;app.camera_.restore({15,42,35,{0,1,0}});
                    const auto& m=app.habitat_models_.mesh(h);
                    if(scene==98){CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,{m.dome_radii.x,0,.65f}},b{CorridorEndpointKind::OpenDoor,0,{18,0,0}};
                        if(!app.corridor_network_.build_corridor(a,b,app.placed_habitats_,app.habitat_models_,{},app.resource_models_))throw std::runtime_error("Fixed airlock render corridor failed");
                        app.corridor_network_.mutable_connections().back().construction.finish();app.camera_.restore({23,39,35,{4,1,0}});}
                    if(scene==97||scene==99){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({scene==99?-2.2f:0,m.walkable_floor_y,0},scene==99?90:0);}
                    for(const auto& d:app.habitat_doors(0)){PressureDoorState state;state.door=d;state.open_fraction=scene>=98?1:0;app.pressure_doors_.push_back(state);}
                }
                if(scene>=100&&scene<=105){
                    app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();
                    app.first_person_mode_=GameApp::FirstPersonMode::Off;app.selection_kind_=GameApp::SelectionKind::None;
                    app.lander_={};app.lander_.phase=scene==100?LanderPhase::Landed:scene==104?LanderPhase::DeployingRobots:LanderPhase::OpeningRamp;
                    app.lander_.builders=app.lander_.carriers=3;app.lander_.boxed=true;
                    app.lander_.elapsed=scene==101?kLanderRampMotionSeconds:scene==102?7.75f:scene==103?kLanderRampSeconds:scene==104?12.f:0;
                    app.camera_.restore({15,22,0,{0,2.9f,0.5f}});
                    if(scene==105){app.lander_={};app.starting_robot_count_=app.starting_carrier_count_=2;app.reset_builder_robot_and_starter_stash();
                        app.ground_resource_piles_.clear();for(std::size_t i=0;i<app.builder_robots_.size();++i){auto& r=app.builder_robots_[i];r.position=r.visual_position={float(i%2)*5-2.5f,0,float(i/2)*5-2.5f};r.visual_initialized=true;r.heading_degrees=r.visual_heading_degrees=float(i)*85;}
                        app.camera_.restore({15,35,35,{0,.5f,0}});}
                }
                if(scene>=106){
                    app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.builder_robots_.clear();app.ground_resource_piles_.clear();app.pressure_doors_.clear();app.lander_={};
                    auto h=room;h.kind=HabitatKind::AirLock;h.construction.finish();h.size=HabitatSizeTier::Standard;h.supply.power_fraction=1;app.placed_habitats_={h};
                    app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;app.first_person_mode_=GameApp::FirstPersonMode::Off;
                    app.selection_kind_=scene==106?GameApp::SelectionKind::Habitat:GameApp::SelectionKind::None;app.selected_habitat_=0;
                    app.camera_.restore({15,38,40,{0,1.6f,0}});
                    if(scene==108)app.camera_.restore({7,12,-90,{-4.8f,1.6f,0}});
                    if(scene==109){app.placed_habitats_.clear();CorridorConnection c;c.path={{-10,.5f,0},{0,1,0},{8,1.7f,5}};c.construction.finish();c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.tangent=game::runtime::normalize(c.path[1]-c.path[0]);c.endpoint_b.tangent=game::runtime::normalize(c.path[c.path.size()-2]-c.path.back());app.corridor_network_.restore_connections({c});app.corridor_network_.synchronize_structure_interfaces({},app.habitat_models_,{},app.resource_models_);app.camera_.restore({23,18,35,{0,.4f,0}});}
                    if(scene==110){app.screen_=GameApp::Screen::Settings;app.autosave_minutes_=10;}
                }
                app.input_.mouse_x=app.input_.mouse_y=-100;
            }

            for(int frame=0;frame<3;++frame){app.ui_time_seconds_+=1./60;app.render(*renderer);}
            std::vector<unsigned char> pixels(960*540*4);gl.ReadPixels(0,0,960,540,vl::GL_RGBA,vl::GL_UNSIGNED_BYTE,pixels.data());
            unsigned low=255,high=0;
            for(int y=100;y<440;++y)for(int x=100;x<860;++x){const auto i=(y*960+x)*4;const unsigned value=(unsigned(pixels[i])+pixels[i+1]+pixels[i+2])/3;low=std::min(low,value);high=std::max(high,value);}
            if(high-low<12)throw std::runtime_error(std::string("Scene has no rendered contrast: ")+names[scene]);
            if(scene>=77&&scene<=80){
                for(const auto p:std::array<std::pair<int,int>,4>{{{0,270},{959,270},{480,0},{480,539}}}){
                    const auto at=(p.second*960+p.first)*4;
                    if(scene!=79&&(pixels[at]<180||pixels[at+1]>100))throw std::runtime_error("Critical FPV border is hidden at a screen edge");
                    // A translucent dim border can still have a bright red
                    // channel over sunlit ground: test red tint, not luminance.
                    if(scene==79&&int(pixels[at])-int(pixels[at+1])>130)throw std::runtime_error("Critical FPV border failed to dim");
                }
            }
            if(scene==28 || scene==29 || scene==32){
                int border_pixels=0;int low_y=540,high_y=0;
                for(int y=80;y<460;++y)for(int x=300;x<660;++x){const auto i=(y*960+x)*4;
                    const bool border=scene!=29?pixels[i]>160&&pixels[i]>pixels[i+1]*3: pixels[i+1]>160&&pixels[i+1]>pixels[i]*3;
                    if(border){++border_pixels;low_y=std::min(low_y,y);high_y=std::max(high_y,y);}
                }
                if(scene==28&&border_pixels>50)throw std::runtime_error("Unused powered gate still has red warning");
                if(scene!=28&&(border_pixels<400||high_y-low_y<150))throw std::runtime_error(std::string("Gate border is hidden by its frame: ")+names[scene]);
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
