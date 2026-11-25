#include "jsd/filesystem.hpp"
#include "game/game_app.hpp"
#include "game/config/preferences.hpp"
#include "game/ui/layout.hpp"
#include "game/runtime/ui_metrics.hpp"
#include "game/runtime/ui_font.hpp"
#include "game/systems/footstep_cadence.hpp"
#include "game/systems/driller_acoustics.hpp"
#include "game/systems/sound_assets.hpp"
#include "game/world/pressure_hardware.hpp"
#include "game/world/hull_surface.hpp"
#include "game/world/corridor/detail.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/solar_collector.hpp"
#include "game/world/interior_unit_mesh.hpp"
#include "game/world/interior_finish.hpp"
#include "game/world/habitats/screen_mounts.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool pass,const std::string& message){if(!pass)throw std::runtime_error(message);}
struct Draw {char type;std::array<float,6> v{};std::string text;game::runtime::Color color;};
struct Recorder:game::runtime::SceneRenderer {
    int w=1920,h=1080;float scale=1;bool hud=false;std::vector<Draw> draws;
    game::runtime::GraphicsSettings graphics;
    void set_graphics_settings(const game::runtime::GraphicsSettings& settings) override { graphics=settings; }
    std::vector<game::runtime::Vec3> belt_preview_lines,queue_hover_lines;
    std::size_t mesh_triangles=0;
    bool initialize(void*,std::uintptr_t,int,int)override{return true;}
    void resize(int width,int height)override{w=width;h=height;}
    void begin_frame(game::runtime::Color)override{draws.clear();belt_preview_lines.clear();queue_hover_lines.clear();mesh_triangles=0;hud=false;}void end_frame()override{}
    void set_3d(const game::runtime::Mat4&)override{hud=false;}void set_2d(float s)override{scale=s;hud=true;}
    void draw_lines_3d(const std::vector<game::runtime::Vec3>& points,game::runtime::Color color,float width)override{
        if(width==1.9f && color.r==.34f && color.g==.82f && color.b==1.0f)belt_preview_lines=points;
        if(color.r==.31f && color.g==.94f && color.b==1.f && color.a==1.f)
            queue_hover_lines.insert(queue_hover_lines.end(),points.begin(),points.end());
    }
    void draw_triangles_3d(const std::vector<game::runtime::Vec3>& points,game::runtime::Color)override{mesh_triangles+=points.size()/3;}
    void draw_rect(game::runtime::Rect r,game::runtime::Color c)override{if(hud)draws.push_back({'R',{r.x,r.y,r.w,r.h,scale,0},{},c});}
    void draw_line(game::runtime::Vec2 a,game::runtime::Vec2 b,game::runtime::Color c,float width)override{if(hud)draws.push_back({'L',{a.x,a.y,b.x,b.y,scale,width},{},c});}
    void draw_text(game::runtime::Vec2 p,const std::string& text,float s,game::runtime::Color c)override{if(hud){auto e=game::runtime::ui::text_extent(text,s);draws.push_back({'T',{p.x,p.y,e.x,e.y,scale,s},text,c});}}
    int width()const override{return w;}int height()const override{return h;}bool hardware_accelerated()const override{return false;}
    std::string gpu_vendor()const override{return "TEST";}std::string gpu_renderer()const override{return "SCALING RECORDER";}std::string gpu_version()const override{return "TEST";}
    game::runtime::Rect button(const std::string& caption)const{
        game::runtime::Rect last{};for(const auto& d:draws){if(d.type=='R')last={d.v[0],d.v[1],d.v[2],d.v[3]};if(d.type=='T' && d.text==caption)return last;}
        throw std::runtime_error("button not found: "+caption);
    }
    void check_bounds(const std::string& label)const{
        for(const auto& d:draws){if(d.type=='L')continue;const float s=d.v[4];
            if(d.v[0]<-0.1f || d.v[1]<-0.1f || (d.v[0]+d.v[2])*s>w+0.5f || (d.v[1]+d.v[3])*s>h+0.5f)
                throw std::runtime_error(label+": outside viewport: "+d.type+std::string(" ")+d.text+" at "+std::to_string(d.v[0])+","+std::to_string(d.v[1])+" size "+std::to_string(d.v[2])+","+std::to_string(d.v[3]));
        }
    }
    void write(const std::filesystem::path& path)const{
        std::ofstream f(path);f<<"{\"width\":"<<w<<",\"height\":"<<h<<",\"draws\":[";bool first=true;
        for(const auto& d:draws){if(!first)f<<",";first=false;f<<"{\"type\":\""<<d.type<<"\",\"v\":[";for(std::size_t i=0;i<d.v.size();++i){if(i)f<<",";f<<d.v[i];}f<<"],\"text\":"<<std::quoted(d.text)<<",\"color\":["<<d.color.r<<","<<d.color.g<<","<<d.color.b<<","<<d.color.a<<"]}";}
        f<<"]}";
    }
};
void compare(const Recorder& a,const Recorder& b,const std::string& name){
    require(a.draws.size()==b.draws.size(),name+": resolution changed layout contents");
    for(std::size_t i=0;i<a.draws.size();++i){const auto& x=a.draws[i];const auto& y=b.draws[i];require(x.type==y.type && x.text==y.text,name+": labels differ");
        for(int k=0;k<6;++k){if(k==4)continue;require(std::abs(x.v[k]-y.v[k])<0.03f,name+": logical geometry/font/stroke differs, item "+std::to_string(i));}
        require(std::abs(x.v[4]/a.h-y.v[4]/b.h)<1e-6f,name+": normalized scale differs");
    }
}
}
namespace game {
struct UiRegression {
    static void setup(GameApp& app,int scene,int zoom){
        app.ui_scale_index_=zoom;app.game_speed_=GameApp::GameSpeed::Pause;app.input_.mouse_x=-100;app.input_.mouse_y=-100;
        app.playfield_.rebuild(64);app.camera_.reset(64);app.resources_.reset();app.status_.reset();
        app.display_modes_={{1280,720,0},{1920,1080,0},{3840,2160,0}};app.display_mode_index_=1;
        using Screen=GameApp::Screen;
        switch(scene){
            case 0:app.screen_=Screen::MainMenu;break;
            case 1:app.screen_=Screen::MainMenu;app.game_session_active_=true;break;
            case 2:app.screen_=Screen::NewGame;break;
            case 3:app.screen_=Screen::Settings;break;
            case 4:app.screen_=Screen::SaveGame;app.game_session_active_=true;app.save_game_name_=std::string(64,'M');break;
            case 5:app.screen_=Screen::LoadGame;app.save_games_={{std::string(64,'M'),"test.gmsave","0.43.6"},{"SECOND COLONY","two.gmsave","0.43.6"}};break;
            case 6:app.screen_=Screen::Credits;break;
            default:app.screen_=Screen::Playfield;break;
        }
        if(scene>=7){
            PlacedHabitat habitat;habitat.kind=HabitatKind::CrewQuarters;habitat.position={0,0,0};habitat.construction.step=4;
            app.placed_habitats_={habitat};
            PlacedResource storage;storage.kind=ResourceKind::StorageDome;storage.position={10,0,0};storage.construction.step=kConstructionSteps;storage.supply.volume_m3=150;storage.supply.warnings=LowAir;
            app.placed_resources_={storage};app.builder_robots_.resize(1);app.builder_robots_[0].position={-10,0,-10};
            app.ground_resource_piles_={{MaterialResource::Iron,500,{0,0,-10},0.5f,1}};
            if(scene==8){app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;}
            if(scene==9){app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;}
            if(scene==10){app.selection_kind_=GameApp::SelectionKind::Robot;app.selected_robot_=0;}
            if(scene==11){app.selection_kind_=GameApp::SelectionKind::GroundResource;app.selected_ground_resource_=0;}
            if(scene==13 || scene==14){
                auto& h=app.placed_habitats_[0];h.kind=scene==13?HabitatKind::OxygenGeneratorDome:HabitatKind::CrewQuarters;h.construction.step=kConstructionSteps;
                h.supply.volume_m3=150;h.supply.oxygen_kg=150*kOxygenKgPerM3*0.75;h.supply.power_fraction=1;h.supply.water_to_fill_litres=150*kOxygenKgPerM3*0.25*kWaterPerOxygen;
                h.supply.air_change_percentage_points_per_day=scene==13?0.125:-0.125;h.supply.oxygen_net_kg_per_day=scene==13?0.5:-0.5;h.supply.oxygen_produced_kg_per_day=9;h.supply.water_processed_litres_per_day=10.125;h.supply.hydrogen_produced_kg_per_day=1.125;h.supply.hydrogen_kg=2.25;
                app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.game_speed_=GameApp::GameSpeed::Normal;
            }
            if(scene==15){CorridorConnection corridor;corridor.path={{-8,0,0},{8,0,0}};corridor.construction.step=corridor.construction.total_steps;corridor.supply.volume_m3=64;corridor.supply.oxygen_kg=64*kOxygenKgPerM3*0.5;
                app.corridor_network_.restore_connections({corridor});app.selection_kind_=GameApp::SelectionKind::Corridor;app.selected_corridor_=0;}
            if(scene==16 || scene==17){auto& item=app.placed_resources_[0];item.kind=scene==16?ResourceKind::StorageDome:ResourceKind::RobotWorkshop;
                item.supply.warnings=scene==16?(NoPower|LowAir):NoPower;app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;}
            if(scene==18){auto& item=app.placed_resources_[0];item.kind=ResourceKind::HydrogenGenerator;item.construction.step=kConstructionSteps;item.supply.hydrogen_power_kw=2.75;item.supply.hydrogen_consumed_kg_per_day=3.60;
                app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;app.game_speed_=GameApp::GameSpeed::Normal;}
            if(scene==19){auto& h=app.placed_habitats_[0];h.kind=HabitatKind::BioDome;h.size=HabitatSizeTier::Standard;h.construction.step=kConstructionSteps;h.supply.volume_m3=150;h.supply.oxygen_kg=150*kOxygenKgPerM3;h.supply.power_fraction=1;
                for(std::size_t bay=0;bay<hydroponics_bay_count(h.size);++bay){h.hydroponics[bay].plant=(HydroponicPlant)(1+(int)(bay%((std::size_t)HydroponicPlant::Count-1)));h.hydroponics[bay].growth_fraction=0.15+0.18*(double)bay;}
                app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.game_speed_=GameApp::GameSpeed::Normal;}
            if(scene==12){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-12,0,-12},45);}
        }
    }
    static void order_controls(){
        game::runtime::GameHost engine;GameApp defaults(engine);require(defaults.ui_scale_index_==0 && defaults.ui_scale_==1,"default UI scale is not 100%");require(defaults.playfield_size_==128,"default playfield is not 128 x 128");require(GameApp::kNewGamePlayfieldMin==32 && GameApp::kNewGamePlayfieldMax==1024 && GameApp::kNewGamePlayfieldStep==32,"new-game playfield range is not doubled");require(GameApp::kDebugTemplatePlayfieldSize==128,"debug template playfield is not doubled");Playfield maximum_field;maximum_field.rebuild(GameApp::kNewGamePlayfieldMax);require(maximum_field.size()==1024 && maximum_field.floor_triangles().size()==6,"1024 x 1024 playfield failed to build");int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<game::runtime::Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}})for(int domain=0;domain<5;++domain){
            GameApp app(engine);setup(app,7,zoom);app.placed_habitats_[0].placement_sequence=1;app.placed_resources_[0].placement_sequence=2;
            ConstructionTargetRef t;
            if(domain==0){t={ConstructionTargetKind::Habitat,0};app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;}
            if(domain==1){t={ConstructionTargetKind::Resource,0};app.placed_resources_[0].construction.step=0;app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;}
            if(domain==2){t={ConstructionTargetKind::Corridor,0};CorridorConnection c;c.placement_sequence=3;c.path={{0,0,0},{0,0,8}};app.corridor_network_.mutable_connections().push_back(c);app.selection_kind_=GameApp::SelectionKind::Corridor;app.selected_corridor_=0;}
            if(domain==3){t={ConstructionTargetKind::Utility,0};UtilityConnection c;c.placement_sequence=3;c.path={{0,0,0},{0,0,8}};app.utility_network_.mutable_connections().push_back(c);app.selection_kind_=GameApp::SelectionKind::Utility;app.selected_utility_=0;}
            if(domain==4){t={ConstructionTargetKind::Walkway,0};WalkwayConnection c;c.placement_sequence=3;c.path={{0,0,0},{0,0,8}};app.walkway_network_.mutable_connections().push_back(c);app.selection_kind_=GameApp::SelectionKind::Walkway;app.selected_walkway_=0;}
            Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("priority control");
            const auto panel=app.selection_panel_rect();require(panel.y+panel.h<=size.y/app.ui_scale_-layout::BUILD_BAR_H,"priority details overlap build catalog");
            const auto button=r.button("PRIORITIZE BUILD");require(!panel.contains(button.x+button.w/2,button.y+button.h/2),"priority button stayed in fixed details panel");app.input_.mouse_x=(int)((button.x+button.w/2)*r.scale);app.input_.mouse_y=(int)((button.y+button.h/2)*r.scale);app.input_.left_pressed=true;app.update(0,app.input_);require(!app.selection_marquee_active_,"floating priority click started a world marquee");app.render(r);
            require(app.construction_priority_rank(t)==0,"priority button hitbox missed");app.input_.left_pressed=false;app.render(r);r.button("REMOVE PRIORITY");
            bool marker=false;for(const auto& draw:r.draws)marker|=draw.text.find("PRIO 1")!=std::string::npos;require(marker,"promoted blueprint overhead marker missing");r.check_bounds("priority marker");
            app.input_.left_pressed=true;app.render(r);require(app.construction_priority_rank(t)==kNoRobotIndex,"remove priority button failed");++cases;
        }
        {
            GameApp debug(engine);setup(debug,7,2);debug.debug_frame_enabled_=true;debug.selection_kind_=GameApp::SelectionKind::None;
            Recorder debug_r;debug_r.resize(1920,1080);debug.render(debug_r);debug_r.check_bounds("debug layout switch");
            require(debug_r.graphics.path==game::runtime::GraphicsPath::Auto,"enhanced graphics are not the default");
            const auto layout_button=debug_r.button("FULL GRAPHICS");
            debug.input_.mouse_x=(int)((layout_button.x+layout_button.w/2)*debug_r.scale);
            debug.input_.mouse_y=(int)((layout_button.y+layout_button.h/2)*debug_r.scale);
            debug.input_.left_pressed=true;debug.render(debug_r);
            require(debug.world_layout_mode_==GameApp::WorldLayoutMode::Mesh,"debug layout switch did not select mesh mode");
            debug.input_.left_pressed=false;debug.render(debug_r);debug_r.button("MESHED");
            require(debug_r.graphics.path==game::runtime::GraphicsPath::Classic,"debug layout did not select the mesh renderer");
        }
        GameApp app(engine);setup(app,7,2);app.debug_frame_enabled_=true;app.selection_kind_=GameApp::SelectionKind::Robot;app.selected_robot_=0;
        Recorder r;app.render(r);r.check_bounds("debug robot inspector");const auto button=r.button("DEBUG MOVE / PICK DESTINATION");
        app.input_.mouse_x=(int)((button.x+button.w/2)*r.scale);app.input_.mouse_y=(int)((button.y+button.h/2)*r.scale);app.input_.left_pressed=true;app.render(r);require(app.debug_move_pick_robot_==0,"debug move button did not arm destination picker");
        game::runtime::InputState escape;escape.escape_pressed=true;app.update(0,escape);require(app.debug_move_pick_robot_==-1 && app.screen_==GameApp::Screen::Playfield,"escape from move picker left game");
        app.debug_move_pick_robot_=0;game::runtime::InputState click;click.mouse_x=r.w/2;click.mouse_y=r.h/2;click.left_pressed=true;click.left_down=true;
        app.update(0,click);require(app.debug_move_pick_robot_==-1 && app.builder_robots_[0].debug_move_active && app.selection_kind_==GameApp::SelectionKind::Robot,"destination click selected the world instead of issuing debug move");
        require(app.issue_debug_robot_move(0,{12,0,12}),"UI debug order rejected");app.input_={};app.render(r);r.button("CANCEL DEBUG MOVE");
        bool task=false;for(const auto& draw:r.draws)task|=draw.text=="TASK  DEBUG MOVE TO SELECTED PLACE";require(task,"debug destination missing from robot task panel");
        const auto cancel=r.button("CANCEL DEBUG MOVE");app.input_.mouse_x=(int)((cancel.x+cancel.w/2)*r.scale);app.input_.mouse_y=(int)((cancel.y+cancel.h/2)*r.scale);app.input_.left_pressed=true;app.render(r);require(!app.builder_robots_[0].debug_move_active,"cancel debug move failed");
        std::cout<<cases<<" priority control/marker/resolution cases plus debug move/cancel inspector controls passed\n";
    }
    static void airlock_status(){
        game::runtime::GameHost engine;int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<game::runtime::Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}}){
            GameApp app(engine);setup(app,7,zoom);auto& h=app.placed_habitats_[0];h.kind=HabitatKind::AirLock;h.construction.step=kConstructionSteps;
            app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.airlock_cycles_.resize(1);app.airlock_cycles_[0].pressure=0.4f;app.airlock_cycles_[0].target=1;app.airlock_cycles_[0].equalizing=true;
            Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("airlock pressure inspector");
            const auto has=[&](const std::string& text){for(const auto& d:r.draws)if(d.text.find(text)!=std::string::npos)return true;return false;};
            require(has("CENTER PRESSURE 40% OF INSIDE") && has("EQUALIZING TO INSIDE") && has("3 CHAMBERS / BIDIRECTIONAL"),"airlock details omit chamber pressure/cycle state");
            for(float x:{-3.f,0.f,3.f}){
                app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({x,app.habitat_models_.mesh(h).walkable_floor_y,0});app.render(r);r.check_bounds("airlock pressure HUD");
                require(has(x<0?"EXTERIOR ISOLATION CABIN":x>0?"INTERIOR ACCESS CABIN":"CENTER PRESSURE CHAMBER"),"airlock HUD labels wrong cabin");
                require(has("CENTER 40% OF INSIDE / EQUALIZING TO INSIDE"),"airlock HUD omits measured cycle");++cases;
            }
        }
        std::cout<<cases<<" airlock cabin/HUD/pressure/inspector resolution cases passed\n";
    }
    static void status_bars(const std::string& output){
        game::runtime::GameHost engine;int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<game::runtime::Vec2,5>{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}})for(int state=0;state<3;++state){
            GameApp app(engine);app.ui_scale_index_=zoom;app.on_resize((int)size.x,(int)size.y);app.ui_input_.mouse_x=app.ui_input_.mouse_y=-100;
            if(state){
                PlacedHabitat room;room.kind=HabitatKind::CrewQuarters;room.construction.step=kConstructionSteps;room.supply.volume_m3=100;room.supply.oxygen_kg=100*kOxygenKgPerM3;
                app.placed_habitats_.push_back(room);room.kind=HabitatKind::AirLock;room.supply.volume_m3=300;room.supply.oxygen_kg=state==2?300*kOxygenKgPerM3:0;app.placed_habitats_.push_back(room);
                room.kind=HabitatKind::BioDome;room.supply.volume_m3=100;room.supply.oxygen_kg=state==2?100*kOxygenKgPerM3:0;app.placed_habitats_.push_back(room); // disconnected rooms still count
                room.construction.step=0;room.supply.volume_m3=10000;room.supply.oxygen_kg=10000*kOxygenKgPerM3;app.placed_habitats_.push_back(room);
                PlacedResource battery;battery.kind=ResourceKind::PowerStorage;battery.construction.step=kConstructionSteps;battery.supply.energy_kwh=state==2?600:30;app.placed_resources_.push_back(battery);
                battery.size=ResourceSizeTier::Small;battery.supply.energy_kwh=state==2?300:15;app.placed_resources_.push_back(battery);battery.construction.step=0;app.placed_resources_.push_back(battery);
                PlacedResource dome;dome.kind=ResourceKind::StorageDome;dome.construction.step=kConstructionSteps;dome.supply.volume_m3=100;dome.supply.oxygen_kg=(state==2?100:50)*kOxygenKgPerM3;app.placed_resources_.push_back(dome);
                dome.kind=ResourceKind::RobotWorkshop;dome.supply.volume_m3=100000;dome.supply.oxygen_kg=100000*kOxygenKgPerM3;app.placed_resources_.push_back(dome);
                PlacedResource tank;tank.kind=ResourceKind::WaterStorage;tank.construction.step=kConstructionSteps;tank.supply.water_litres=state==2?20000:500;app.placed_resources_.push_back(tank);tank.size=ResourceSizeTier::Small;tank.supply.water_litres=state==2?10000:250;app.placed_resources_.push_back(tank);tank.construction.step=0;app.placed_resources_.push_back(tank);
                app.battery_net_kw_=state==2?-2:4.5;app.water_net_litres_per_day_=state==2?-34.1:123;
                CorridorConnection c;c.construction.step=c.construction.total_steps;c.supply.volume_m3=100000;c.supply.oxygen_kg=100000*kOxygenKgPerM3;app.corridor_network_.mutable_connections().push_back(c);
            }
            Recorder r;r.resize((int)size.x,(int)size.y);r.set_2d(app.ui_scale_);app.render_status_bar(r);r.check_bounds("status charge/air bars");
            std::vector<Draw> bars;std::string labels;for(const auto& d:r.draws){if(d.type=='R' && d.v[1]==33 && d.v[3]==6)bars.push_back(d);if(d.type=='T')labels+=d.text+"|";}
            require(bars.size()==8,"missing charge/air/water/storage indicator bars");const float charge=state==0?0:state==1?(45.0f/202.5f):1,air=state==0?0:state==1?0.25f:1;
            require(std::abs(bars[1].v[2]-bars[0].v[2]*charge)<0.001f,"charge bar is not actual completed-battery energy/capacity");
            require(std::abs(bars[3].v[2]-bars[2].v[2]*air)<0.001f,"air bar does not combine indoor volume correctly");
            require(std::abs(bars[5].v[2]-bars[4].v[2]*air)<0.001f,"water bar is not actual completed tank fill");
            if(state){require(labels.find(state==1?"BAT +4.5 KW":"BAT -2.0 KW")!=std::string::npos,"power charge/discharge rate missing");require(labels.find(state==1?"RATE +123.0 L/D":"RATE -34.1 L/D")!=std::string::npos,"water fill/drain rate missing");}
            require(labels.find(state==0?"AIR --":state==1?"AIR 25.0%":"AIR 100.0%")!=std::string::npos,"air label disagrees with indicator");
            require(labels.find(state==0?"POWER 0.0/0.0 KWH":state==1?"POWER 45.0/202.5 KWH":"POWER 202.5/202.5 KWH")!=std::string::npos,"power label uses stale counters or includes blueprints");
            if(!output.empty() && state==1 && zoom==2 && (size.x==1280 || size.x==3840))r.write(std::filesystem::path(output)/("status-bars-"+std::to_string((int)size.x)+".json"));
            ++cases;
        }
        std::cout<<cases<<" status-bar cases: battery capacity/charge, combined indoor air, disconnected rooms, excluded blueprints/outdoor machinery, empty/full/clamped values and resolution scaling passed\n";
    }
    static void storage_and_rate_display(){
        game::runtime::GameHost engine;
        GameApp app(engine);
        app.on_resize(1920,1080);
        Recorder r;
        r.set_2d(app.ui_scale_);
        const auto text=[&](const std::string& part){
            for(const auto& draw:r.draws)if(draw.type=='T'&&draw.text.find(part)!=std::string::npos)return true;
            return false;
        };
        const auto stock=[](PlacedResource& item,std::int64_t kg){
            item.stored_boxes={{MaterialResource::Iron,kg,1}};
            sync_stored_resource_totals(item);
        };

        PlacedResource small;small.kind=ResourceKind::StorageDome;
        small.construction.step=kConstructionSteps;
        const auto small_capacity=storage_dome_capacity_kg(small.storage_size);
        stock(small,small_capacity*85/100);
        PlacedResource large=small;
        large.storage_size=StorageDomeSize::Big;
        large.position={20,0,0};
        const auto large_capacity=storage_dome_capacity_kg(large.storage_size);
        stock(large,large_capacity/2);
        PlacedResource blueprint=large;
        blueprint.position={-20,0,0};
        blueprint.construction.step=0;
        stock(blueprint,large_capacity);
        app.placed_resources_={small,large,blueprint};
        app.ground_resource_piles_={{MaterialResource::Iron,100000,{0,0,0},1,1}};
        app.ui_input_.mouse_x=static_cast<int>(r.width()*.80f/app.ui_scale_);
        app.ui_input_.mouse_y=20;
        app.render_status_bar(r);
        const auto stored=small_capacity*85/100+large_capacity/2;
        const auto capacity=small_capacity+large_capacity;
        std::ostringstream percent;
        percent<<"STORAGE "<<std::fixed<<std::setprecision(1)<<100.0*stored/capacity<<"%";
        require(text(percent.str())&&text(std::to_string(stored)+" / "+std::to_string(capacity)+" KG USED"),
                "storage HUD ignored size, completion or physical stock");
        r.draws.clear();
        app.render_supply_indicators(r);
        const auto markers=[&]{
            int count=0;
            for(const auto& draw:r.draws)
                if(draw.type=='R' && draw.v[2]==20 && draw.v[3]==18)++count;
            return count;
        };
        require(markers()==0,"85 percent or unfinished dome warned");
        stock(app.placed_resources_[0],small_capacity*85/100+1);
        app.placed_resources_[0].supply.warnings=LowAir;
        r.draws.clear();
        app.render_supply_indicators(r);
        require(markers()==1,"nearly full dome has no separate storage marker");
        stock(app.placed_resources_[0],small_capacity/2);
        r.draws.clear();
        app.render_supply_indicators(r);
        require(markers()==0,"storage marker remained after unloading");

        PlacedHabitat generator;generator.kind=HabitatKind::OxygenGeneratorDome;
        generator.construction.step=kConstructionSteps;
        generator.supply.oxygen_produced_kg_per_day=1.25;
        app.placed_habitats_={generator,generator};
        app.placed_habitats_[1].supply.oxygen_produced_kg_per_day=7;
        app.selection_kind_=GameApp::SelectionKind::Habitat;
        app.selected_habitat_=0;
        app.battery_net_kw_=1.5;
        app.supply_power_load_kw_=18.5;
        app.supply_power_delivered_kw_=16.0;
        app.status_rate_display_={};
        const auto render_rates=[&]{
            r.draws.clear();
            app.render_status_bar(r);
            app.render_selection_panel(r);
        };
        render_rates();
        require(text("GRID 16.0/18.5 KW / BAT +1.5")&&text("O2 OUTPUT 1.250"),"initial rate display missing");
        app.battery_net_kw_=6.5;
        app.placed_habitats_[0].supply.oxygen_produced_kg_per_day=2.5;
        app.update(.25f,{});
        render_rates();
        require(text("BAT +1.5")&&text("O2 OUTPUT 1.250"),"rates changed before 500 ms");
        app.update(.25f,{});
        render_rates();
        require(text("BAT +6.5")&&text("O2 OUTPUT 2.500"),"rates did not refresh after 500 ms");
        app.selected_habitat_=1;
        render_rates();
        require(text("O2 OUTPUT 7.000"),"new selection displayed rates from the previous item");
        app.game_speed_=GameApp::GameSpeed::Pause;
        render_rates();
        require(text("RATE PAUSED")&&text("O2 OUTPUT 0.000"),"pause did not immediately reach rate displays");
        std::cout<<"Storage occupancy, per-dome 85 percent threshold and half-second rate displays passed\n";
    }
    static void grouped_build_menu(){
        game::runtime::GameHost engine;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom){
            GameApp app(engine);
            app.ui_scale_index_=zoom;
            app.on_resize(1280,720);
            Recorder r;
            r.resize(1280,720);
            r.set_2d(app.ui_scale_);
            for(const auto category:{BuildCategory::Habitats,BuildCategory::Structures}){
                app.build_catalog_.set_category(category);
                app.build_catalog_scroll_=0;
                const auto groups=category==BuildCategory::Habitats?
                    std::vector<std::string>{"ACCESS","AIR","LIVING","MEDICAL","INDUSTRY","STORAGE"}:
                    std::vector<std::string>{"POWER","WATER","HYDROGEN","AIR","MINING","SUPPLY","TRADE"};
                std::vector<std::string> seen;
                app.ui_input_={};
                for(int page=0;page<20;++page){
                    r.draws.clear();
                    app.render_build_bar(r);
                    r.check_bounds("grouped build tiles");
                    for(const auto& d:r.draws){
                        if(d.type=='T'&&std::find(groups.begin(),groups.end(),d.text)!=groups.end()&&
                           std::find(seen.begin(),seen.end(),d.text)==seen.end())seen.push_back(d.text);
                    }
                    app.ui_input_.mouse_x=160;
                    app.ui_input_.mouse_y=static_cast<int>(720/app.ui_scale_)-6;
                    app.ui_input_.wheel=-1;
                }
                require(seen==groups,"build group captions are missing or out of order at a supported zoom");
            }
        }
        std::cout<<"Both grouped build catalogs page through all captions at every UI scale\n";
    }

    static void resource_inventory(const std::string& output){
        game::runtime::GameHost engine;int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:layout::COMMON_RESOLUTIONS){
            GameApp app(engine);app.ui_scale_index_=zoom;app.on_resize(size.width,size.height);app.ui_input_.mouse_x=app.ui_input_.mouse_y=-100;
            ResourceBundle cargo;for(std::size_t i=0;i<kMaterialResourceCount;++i)cargo.amount((MaterialResource)i)=1234+(int)i;app.resources_.restore(cargo);
            PlacedResource tank;tank.kind=ResourceKind::HydrogenTank;tank.construction.step=kConstructionSteps;tank.supply.hydrogen_kg=50;app.placed_resources_.push_back(tank);tank.construction.step=0;tank.supply.hydrogen_kg=999;app.placed_resources_.push_back(tank);
            PlacedHabitat generator;generator.kind=HabitatKind::OxygenGeneratorDome;generator.construction.step=kConstructionSteps;generator.placement_sequence=10;generator.supply.hydrogen_kg=2.5;generator.supply.oxygen_kg=250;app.placed_habitats_.push_back(generator);generator.construction.step=0;generator.placement_sequence=11;generator.supply.hydrogen_kg=999;app.placed_habitats_.push_back(generator);
            PlacedResource pad;pad.kind=ResourceKind::LandingPad;pad.construction.step=kConstructionSteps;pad.rocket_fuel_kg=4200;pad.lander_fuel_kg=1150;app.placed_resources_.push_back(pad);
            app.trader_.oxygen={{10,21.5},{11,999},{99,999}};app.planning_.funds_eur=876543;
            Recorder r;r.resize(size.width,size.height);r.set_2d(app.ui_scale_);app.render_status_bar(r);r.check_bounds("all resource status bar");
            const auto has=[&](const std::string& text){for(const auto& d:r.draws)if(d.type=='T'&&d.text==text)return true;return false;};
            for(const char* name:{"IRON","COPPER","SILICON","PLASTIC","WATER","FOOD","MEDICAL","ALUMINUM","OXYGEN","HYDROGEN"})require(has(name),std::string("status bar omits ")+name);
            for(std::size_t i=0;i<kMaterialResourceCount;++i)require(has(std::to_string(1234+i)+" KG"),"status cargo amount missing");
            require(has("21.5 KG")&&has("52.5 KG"),"gas stocks include cabin air, blueprints or orphan buffers");
            require(has("CASH EUR 876543"),"cash omitted from status bar");
            if(!output.empty()&&zoom==0&&size.width==1920)r.write(std::filesystem::path(output)/"all-resources.json");
            // Exact stock tooltip belongs to the second row, including at zoom 200%.
            app.ui_input_.mouse_x=(int)(size.width*.95f/app.ui_scale_);app.ui_input_.mouse_y=70;r.draws.clear();app.render_status_bar(r);r.check_bounds("gas stock tooltip");bool tooltip=false;for(const auto& d:r.draws)if(d.type=='T')tooltip|=d.text.find("HYDROGEN / 52.5 KG STORED")!=std::string::npos;require(tooltip,"gas tooltip missing/wrong hitbox");
            ++cases;
        }
        std::cout<<cases<<" full-inventory status layouts: all ten resources, exact cargo/gas counts, exclusion of cabin air/blueprints/allocated fuel, cash and tooltip hitboxes passed\n";
    }
    static void progress_tree(){
        game::runtime::GameHost engine;int cases=0;
        constexpr std::array<game::runtime::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp app(engine);setup(app,7,zoom);app.debug_frame_enabled_=false;Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("progress tree");
            const auto has=[&](const std::string& text){
                return std::any_of(r.draws.begin(),r.draws.end(),[&](const auto& d){return d.type=='T' && d.text.find(text)!=std::string::npos;});
            };
            require(has("STAGE 1 / Build a solar collector") && !has("Build a battery"),
                    "collapsed progress did not show only the next target");
            const auto collapsed=r.button("+  COLONY PROGRESS");
            game::runtime::InputState input{};
            input.mouse_x=(int)std::lround((collapsed.x+collapsed.w*.5f)*r.scale);
            input.mouse_y=(int)std::lround((collapsed.y+collapsed.h*.5f)*r.scale);
            input.left_pressed=true;
            app.update_simulation(0.f,input);
            require(!app.selection_marquee_active_,"progress click reached the world");
            app.render(r);
            app.input_={};app.render(r);
            require(app.progress_tree_expanded_ && has("STAGE 1") && has("READY FOR HUMAN CREW"),
                    "expanding progress did not show the full checklist");
            r.check_bounds("expanded progress tree");
            const auto panel=app.progress_tree_panel_rect();const float logical_h=size.y/app.ui_scale_;
            require(std::any_of(r.draws.begin(),r.draws.end(),[&](const auto& d){
                return d.type=='R' && d.v[0]==panel.x && d.v[1]==panel.y && d.v[2]==panel.w &&
                    d.v[3]==panel.h && d.color.a==layout::kHudOpacity;
            }),"colony progress backdrop was not 60% transparent");
            require(panel.y>=layout::STATUS_BAR_H+32.0f,"progress panel overlaps status tooltip area");
            require(panel.y+panel.h<=logical_h-layout::BUILD_BAR_H-4.0f,"progress panel overlaps build catalog");
            ++cases;
        }
        GameApp debug(engine);setup(debug,7,2);debug.debug_frame_enabled_=true;Recorder r;r.resize(1920,1080);debug.render(r);for(const auto& d:r.draws)require(d.type!='T' || d.text.find("COLONY PROGRESS")==std::string::npos,"progress tree visible in debugging mode");
        std::cout<<cases<<" progression checklist resolution/border cases plus debug-mode suppression passed\n";
    }

    static void scrollable_selection_panel(){
        game::runtime::GameHost engine;GameApp app(engine);setup(app,16,4);
        app.placed_resources_[0].supply.warnings=255;
        Recorder r;r.resize(1280,720);app.render(r);
        const auto panel=app.selection_panel_rect();
        require(app.selection_panel_content_height()>panel.h+1.0f,"overflowing selection details were not capped to a viewport");
        require(app.selection_panel_max_scroll()>1.0f,"overflowing selection details have no scroll range");
        require(app.selection_panel_scroll_==0.0f,"new selection panel does not start at the top");
        bool bottom_before=false;
        for(const auto& d:r.draws)if(d.type=='T' && d.text.find("DEL REMOVE / LMB SELECT OTHER")!=std::string::npos)bottom_before=true;
        require(!bottom_before,"overflowing selection panel exposes bottom content before scrolling");

        const float distance=app.camera_.distance();
        game::runtime::InputState wheel{};
        wheel.mouse_x=(int)std::lround((panel.x+panel.w*0.5f)*app.ui_scale_);
        wheel.mouse_y=(int)std::lround((panel.y+panel.h*0.5f)*app.ui_scale_);
        wheel.wheel=-1.0f;
        app.update(0.0f,wheel);
        require(app.selection_panel_scroll_>0.0f,"mouse wheel over selection panel did not scroll details");
        require(app.camera_.distance()==distance,"selection-panel wheel also zoomed the world");
        for(int i=0;i<32 && app.selection_panel_scroll_<app.selection_panel_max_scroll()-0.01f;++i)app.update(0.0f,wheel);
        require(std::abs(app.selection_panel_scroll_-app.selection_panel_max_scroll())<0.01f,"selection panel did not reach its bottom scroll limit");

        app.input_=wheel;app.render(r);
        bool bottom_after=false,scrollbar=false;
        for(const auto& d:r.draws){
            if(d.type=='T' && d.text.find("DEL REMOVE / LMB SELECT OTHER")!=std::string::npos)bottom_after=true;
            if(d.type=='R' && d.v[0]>=panel.x+panel.w-9.0f && d.v[0]<=panel.x+panel.w-2.0f && d.v[1]>=panel.y+4.0f && d.v[1]+d.v[3]<=panel.y+panel.h-4.0f)scrollbar=true;
            if(d.type=='T' && d.v[0]>=panel.x && d.v[0]<panel.x+panel.w && d.v[1]>panel.y && d.v[1]<panel.y+panel.h)
                require(d.v[1]>=panel.y+4.0f-0.01f && d.v[1]+d.v[3]<=panel.y+panel.h-4.0f+0.01f,"scrolled info text escaped the selection viewport");
        }
        require(bottom_after,"bottom selection details are not reachable by scrolling");
        require(scrollbar,"overflowing selection panel does not render a scrollbar");
        std::cout<<"selection info overflow viewport, wheel scrolling, camera-wheel isolation and scrollbar passed\n";
    }

    static void hydroponics_panel(){
        game::runtime::GameHost engine;GameApp app(engine);setup(app,19,4);
        app.placed_habitats_[0].size=HabitatSizeTier::Big;
        for(std::size_t bay=0;bay<hydroponics_bay_count(HabitatSizeTier::Big);++bay){
            InteriorUnit rack;rack.kind=InteriorUnitKind::HydroponicRack;rack.grow_bay=static_cast<std::uint8_t>(bay);
            rack.position={((int(bay)%4)-1.5f)*2.f,app.habitat_models_.mesh(app.placed_habitats_[0]).walkable_floor_y,bay<4?-3.f:3.f};
            app.placed_habitats_[0].units.push_back(rack);
        }
        for(std::size_t bay=0;bay<hydroponics_bay_count(HabitatSizeTier::Big);++bay){app.placed_habitats_[0].hydroponics[bay].plant=(HydroponicPlant)(1+(int)(bay%((std::size_t)HydroponicPlant::Count-1)));app.placed_habitats_[0].hydroponics[bay].growth_fraction=0.1*(double)(bay+1);}
        Recorder r;r.resize(1280,720);app.on_resize(1280,720);app.render(r);
        bool heading=false,progress_bar=false,photoperiod=false;float bay1_y=-1,bay2_y=-2,bay1_x=0,bay2_x=0;
        for(const auto& d:r.draws){
            if(d.type=='T' && d.text.find("SEED PACKETS")!=std::string::npos)heading=true;
            if(d.type=='T' && d.text.find("PHOTOPERIOD")!=std::string::npos)photoperiod=true;
        }
        app.selection_panel_scroll_=120.0f;app.render(r);
        for(const auto& d:r.draws){
            if(d.type=='R' && d.v[3]>5.0f && d.v[3]<10.0f && d.v[2]>150.0f)progress_bar=true;
            if(d.type=='T' && d.text=="BAY 1"){bay1_x=d.v[0];bay1_y=d.v[1];}
            if(d.type=='T' && d.text=="BAY 2"){bay2_x=d.v[0];bay2_y=d.v[1];}
        }
        require(heading,"Bio Dome status frame has no two-column hydroponics table");
        require(photoperiod,"Bio Dome status frame has no light/dark photoperiod state");
        require(progress_bar,"Bio Dome bay growth progress bar missing");
        require(std::abs(bay1_y-bay2_y)<0.1f && bay2_x>bay1_x+100.0f,"Bio Dome bays are not rendered as a two-column table");
        require(app.selection_panel_content_height()>app.selection_panel_rect().h,"large Bio Dome table is not scrollable");

        auto& first=app.placed_habitats_[0].hydroponics[0];
        first.plant=HydroponicPlant::Potato;first.growth_fraction=0.72;
        app.render(r);
        const auto selector=r.button("POTATO  V");
        app.input_.mouse_x=(int)std::lround((selector.x+selector.w*0.5f)*r.scale);
        app.input_.mouse_y=(int)std::lround((selector.y+selector.h*0.5f)*r.scale);
        app.input_.left_pressed=true;
        app.render(r);
        app.input_.left_pressed=false;
        app.render(r);
        const auto remove=r.button("EMPTY / REMOVE CROP");
        app.input_.mouse_x=(int)std::lround((remove.x+remove.w*0.5f)*r.scale);
        app.input_.mouse_y=(int)std::lround((remove.y+remove.h*0.5f)*r.scale);
        app.input_.left_pressed=true;
        app.render(r);
        require(first.plant==HydroponicPlant::Empty,"Bio Dome dropdown could not remove a selected crop");
        require(first.growth_fraction==0.0,"removing a Bio Dome crop did not reset its growth cycle");

        const auto click_option=[&](const char* caption){
            app.input_={};app.render(r);
            const auto box=r.button(caption);
            app.input_.mouse_x=(int)std::lround((box.x+box.w*.5f)*r.scale);
            app.input_.mouse_y=(int)std::lround((box.y+box.h*.5f)*r.scale);
            app.input_.left_pressed=true;
            app.render(r);
            app.input_={};
        };
        app.colony_seeds_[0]=1;
        click_option("EMPTY / REMOVE  V");
        click_option("WHEAT / 0 SEEDS");
        require(first.plant==HydroponicPlant::Empty && app.colony_seeds_[0]==1,"planting ignored an empty seed inventory");
        click_option("POTATO / 1 SEED");
        require(first.plant==HydroponicPlant::Potato && app.colony_seeds_[0]==0,"new crop did not consume exactly one seed packet");
        require(!plant_hydroponic_crop(first,app.colony_seeds_,HydroponicPlant::Wheat) && first.plant==HydroponicPlant::Potato,
                "out-of-stock replacement destroyed an existing crop");

        app.input_.left_pressed=false;app.input_.mouse_x=-100;app.input_.mouse_y=-100;
        app.placed_habitats_[0].construction.step=kConstructionSteps-1;
        app.render(r);
        bool offline=false,crop_selector_visible=false;
        for(const auto& d:r.draws){
            if(d.type=='T' && d.text.find("OFFLINE UNTIL CONSTRUCTION IS COMPLETE")!=std::string::npos)offline=true;
            if(d.type=='T' && (d.text=="POTATO  V" || d.text=="EMPTY / REMOVE  V"))crop_selector_visible=true;
        }
        require(offline,"unfinished Bio Dome does not explain that hydroponics are offline");
        require(!crop_selector_visible,"unfinished Bio Dome exposes crop selection controls");
        std::cout<<"Bio Dome two-column dropdown table, crop removal, progress bars, photoperiod status and construction gating passed\n";
    }

    static void structure_power_controls(){
        game::runtime::GameHost engine;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom){
            GameApp app(engine);
            app.ui_scale_index_=zoom;
            app.on_resize(1920,1080);
            app.playfield_.rebuild(128);
            app.camera_.restore({30,65,0,{}});
            app.placed_resources_.resize(1);
            app.placed_habitats_.resize(1);
            auto& item=app.placed_resources_.front();
            auto& habitat=app.placed_habitats_.front();
            item.construction.step=habitat.construction.step=kConstructionSteps;
            Recorder r;
            r.resize(1920,1080);
            r.set_2d(app.ui_scale_);

            const auto select=[&](GameApp::SelectionKind kind){
                app.clear_world_selection();
                app.selection_kind_=kind;
                if(kind==GameApp::SelectionKind::Resource)app.selected_resource_=0;
                else app.selected_habitat_=0;
            };
            const auto click=[&](const char* caption){
                const auto button=r.button(caption);
                app.ui_input_.mouse_x=(int)(button.x+button.w*.5f);
                app.ui_input_.mouse_y=(int)(button.y+button.h*.5f);
                app.ui_input_.left_pressed=true;
                app.render_selection_panel(r);
                app.ui_input_={};
                r.draws.clear();
                app.render_selection_panel(r);
            };
            const auto find_row=[&](const char* prefix)->const Draw&{
                for(const auto& draw:r.draws)if(draw.type=='T' && draw.text.starts_with(prefix))return draw;
                throw std::runtime_error(std::string("missing structure inspector row: ")+prefix);
            };
            select(GameApp::SelectionKind::Resource);
            for(auto kind:{ResourceKind::SolarPanel,ResourceKind::WaterDriller}){
                item.kind=kind;
                item.supply.warnings=NoPower|NoWater;
                r.draws.clear();
                app.render_selection_panel(r);
                const auto& footer=find_row("DEL REMOVE");
                const auto& buffer=find_row("LOCAL BUFFER");
                require(buffer.v[1]+buffer.v[3]<=footer.v[1],"storage buffer and removal footer overlap");
                if(kind==ResourceKind::SolarPanel){
                    const auto& track=find_row("TRACKING ON");
                    const auto button=r.button("SUN TRACKING: ON");
                    require(track.v[1]+track.v[3]<=buffer.v[1] && buffer.v[1]+buffer.v[3]<=button.y &&
                        button.y+button.h<=footer.v[1],"solar rows or tracking button overlap");
                }
                click("POWER OFF");
                require(!item.power_enabled && item.supply.power_fraction==0,"resource power-off click failed");
                find_row("STATUS  POWERED OFF");
                r.draws.clear();
                app.render_supply_indicators(r);
                require(std::count_if(r.draws.begin(),r.draws.end(),[](const auto& draw){
                    return draw.type=='L' && draw.color.r==1.0f && draw.color.g==.16f && draw.color.b==.12f;
                })==21,"powered-off red power symbol missing");
                app.render_selection_panel(r);
                click("POWER ON");
                require(item.power_enabled,"resource power-on click failed");
            }
            select(GameApp::SelectionKind::Habitat);
            r.draws.clear();
            app.render_selection_panel(r);
            click("POWER OFF");
            require(!habitat.power_enabled && habitat.supply.power_fraction==0,"habitat power-off click failed");
            click("POWER ON");
            require(habitat.power_enabled,"habitat power-on click failed");
        }
        std::cout<<"Power switches, offline world icon and independent solar/driller rows passed\n";
    }

    static void presentation_controls(){
        using namespace game::runtime;
        GameHost engine;
        GameApp app(engine);
        setup(app,3,layout::UI_SCALE_COUNT-1);
        Recorder r;
        r.resize(1280,720);
        app.render(r);
        const auto click=[&](Rect button){
            app.input_.mouse_x=int(std::lround((button.x+button.w*.5f)*r.scale));
            app.input_.mouse_y=int(std::lround((button.y+button.h*.5f)*r.scale));
            app.input_.left_pressed=true;
            app.render(r);
            app.input_={};
            app.render(r);
            r.check_bounds("presentation settings");
        };
        app.graphics_.path=GraphicsPath::Deferred;
        click(r.button("FULL GRAPHICS"));
        require(r.graphics.path==GraphicsPath::Classic && app.graphics_.path==GraphicsPath::Deferred,
                "mesh override replaced the user's graphics path");
        click(r.button("MESHED"));
        require(r.graphics.path==GraphicsPath::Deferred,"full graphics did not restore the saved path");

        // The right column has two ON buttons. Use the one aligned with Music.
        const auto music=std::find_if(r.draws.begin(),r.draws.end(),[](const Draw& d){return d.type=='T' && d.text=="MUSIC";});
        require(music!=r.draws.end(),"Music row missing");
        const auto music_button=std::find_if(r.draws.begin(),r.draws.end(),[&](const Draw& d){
            return d.type=='R' && d.v[0]>music->v[0] && std::abs(d.v[1]+12-music->v[1])<.1f;
        });
        require(music_button!=r.draws.end(),"Music button missing");
        click({music_button->v[0],music_button->v[1],music_button->v[2],music_button->v[3]});
        require(!app.soundscape_.music_enabled() && !load_preferences().music_enabled,"Music Off did not persist");
        GameApp restarted(engine);
        restarted.on_start();
        require(!restarted.soundscape_.music_enabled(),"Music Off was lost on restart");

        setup(app,7,0);
        InputState key;
        for(std::uint8_t digit=1;digit<=3;++digit){
            key.key_number_pressed=digit;
            app.update_simulation(0,key);
            require(static_cast<int>(app.game_speed_)==digit-1,"number key selected the wrong lunar speed");
        }
        key={};
        key.key_space_pressed=true;
        const auto hour=app.status_.hour();
        app.update_simulation(.05f,key);
        require(app.simulation_paused() && app.status_.hour()==hour,"Space did not freeze lunar time");
        app.render(r);
        const auto border=r.draws.back();
        require(border.type=='R' && border.v[2]==3.5f && border.v[1]==3.5f,"pause border missing");
        app.update_simulation(.5f,{});
        app.render(r);
        require(r.draws.back().color.a!=border.color.a && app.status_.hour()==hour,
                "pause frame did not pulse while simulation time was frozen");
        app.update_simulation(0,key);
        require(app.game_speed_==GameApp::GameSpeed::Fast,"Space did not restore the prior speed");
        app.render(r);
        require(r.draws.back().color.a!=border.color.a || r.draws.back().v[2]!=border.v[2],
                "pause border remained visible after resuming");

        app.active_game_phase_=GameApp::GamePhase::Planning;
        const double day=app.planning_.elapsed_days;
        app.update_simulation(.05f,key);
        require(app.planning_.time_speed==0 && app.planning_.elapsed_days==day,
                "Space did not pause the planning clock");
        key={};
        key.key_number_pressed=2;
        app.update_simulation(.05f,key);
        require(app.planning_.time_speed==2 && std::abs(app.planning_.elapsed_days-day-.25)<.0001,
                "number key did not resume the planning clock at the selected speed");
        std::cout<<"Mesh override, Music persistence, stage speed shortcuts and pause frame passed\n";
    }

    static void playfield_surface(){
        using namespace game::runtime;
        struct SurfaceRecorder : Recorder {
            bool accept_enhanced{},saw_grid{};
            int scene_attempts{},surface_batches{};
            double surface_area{};
            float surface_tint{};
            void begin_frame(Color clear) override {
                Recorder::begin_frame(clear);
                saw_grid=false;
                scene_attempts=surface_batches=0;
                surface_area=0;
                surface_tint=0;
            }
            bool begin_lunar_scene(const LunarScene&) override {
                ++scene_attempts;
                return accept_enhanced;
            }
            void draw_lunar_surface(const std::vector<Vec3>& points,LunarMaterial material,Color color) override {
                if(material!=LunarMaterial::Regolith || points.empty())return;
                ++surface_batches;
                surface_tint=color.r;
                for(std::size_t i=0;i+2<points.size();i+=3){
                    const auto b=points[i+1]-points[i],c=points[i+2]-points[i];
                    const auto area=static_cast<double>(b.z)*c.x-static_cast<double>(b.x)*c.z;
                    require(area>0,"regolith triangle inverted");
                    surface_area+=area*.5;
                }
            }
            void draw_lines_3d(const std::vector<Vec3>& points,Color color,float width) override {
                Recorder::draw_lines_3d(points,color,width);
                saw_grid|=points.size()>32 && points.front().y==.01f;
            }
        };
        GameHost engine;
        GameApp app(engine);
        setup(app,7,0);
        app.placed_habitats_.clear();
        app.placed_resources_.clear();
        app.ground_resource_piles_.clear();
        app.builder_robots_.clear();
        app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.graphics_.features=0;
        app.graphics_.path=GraphicsPath::Classic;
        app.graphics_.surface_resolution=1;
        SurfaceRecorder r;
        for(int side:{32,1024}){
            app.playfield_.rebuild(side);
            app.render(r);
            require(std::abs(r.surface_area-side*side)<.5 && r.surface_batches==1 &&
                    r.surface_tint>.10f && !r.saw_grid && r.scene_attempts==1,
                    "Classic graphics did not draw the full playfield with the regolith material");
        }
        r.accept_enhanced=true;
        app.render(r);
        require(r.surface_batches==0 && r.scene_attempts==1,"Classic skipped the lunar sky/terrain or doubled the ground");
        r.accept_enhanced=false;
        app.graphics_.path=GraphicsPath::Auto;
        app.render(r);
        require(r.surface_batches>0 && r.scene_attempts==1,"enhanced initialization failure lost the surface");
        r.accept_enhanced=true;
        app.render(r);
        require(r.surface_batches==0 && r.scene_attempts==1,"fallback surface overlapped enhanced terrain");
        r.accept_enhanced=false;
        app.world_layout_mode_=GameApp::WorldLayoutMode::Mesh;
        app.render(r);
        if constexpr(build::debug_features){
            require(r.surface_batches==0 && r.scene_attempts==0 && r.saw_grid,"Debug Meshed showed a filled surface");
        }else{
            require(r.surface_batches>0 && r.scene_attempts==1 && !r.saw_grid,"Release hid the surface for a saved debug preference");
        }
        app.world_layout_mode_=GameApp::WorldLayoutMode::FullGraphics;
        app.first_person_mode_=GameApp::FirstPersonMode::Active;
        app.first_person_.spawn({0,0,0},0);
        app.render(r);
        require(r.surface_batches>0 && !r.saw_grid,"first-person graphics lost the playfield surface");
        std::cout<<"Classic, enhanced fallback, first-person and Debug Meshed terrain checks passed\n";
    }

    static void hull_views(){
        using namespace game::runtime;
        struct HullRecorder : Recorder {
            std::vector<Vec3> vertices;
            bool faded{};
            void draw_lunar_surface(const std::vector<Vec3>& points,LunarMaterial,Color c) override {
                if(!points.empty()&&c.a>=.10f&&c.a<=.15f&&c.b>.9f&&c.r<.3f)faded=true;
                vertices.insert(vertices.end(),points.begin(),points.end());
            }
            void draw_lines_3d(const std::vector<Vec3>& points,Color,float) override {
                vertices.insert(vertices.end(),points.begin(),points.end());
            }
            bool drew_at(float y) const {
                return std::any_of(vertices.begin(),vertices.end(),[&](Vec3 point){return point.y==y;});
            }
        };
        const auto layer=[](float y){return std::vector<Vec3>{{0,y,0},{0,y,1},{1,y,0}};};
        GameHost engine;
        GameApp app(engine);
        app.graphics_.path=GraphicsPath::Classic;
        require(app.habitat_view_mode_==GameApp::HabitatViewMode::FullHull,"new game did not start with Full Hull");
        HabitatMesh habitat;
        habitat.foundation=layer(1);habitat.interior=layer(2);
        habitat.shell=layer(3);habitat.glazing=layer(4);
        habitat.edges={{0,5,0},{1,5,0}};habitat.hull_frames=layer(6);
        ResourceMesh resource;resource.walkable_floor_y=2;
        resource.foundation=habitat.foundation;resource.details=habitat.interior;
        resource.body=habitat.shell;resource.edges=habitat.edges;
        CorridorConnection corridor;corridor.path={{0,1,0},{1,1,0}};
        corridor.mesh={habitat.shell,layer(0),habitat.edges};corridor.construction.finish();
        app.corridor_network_.restore_connections({corridor});
        HullRecorder r;
        const auto draw_room=[&](float step){r.vertices.clear();
            app.render_habitat_mesh(r,habitat,{},0,false,true,false,step);};
        // Mesh mode also displays outline layers; filled mode omits unselected edges.
        draw_room(kConstructionSteps);
        require(r.drew_at(1)&&r.drew_at(2)&&r.drew_at(3)&&r.drew_at(4)&&r.drew_at(6),
                "Full Hull lost filled exterior, frames or interior geometry");
        r.set_2d(1);app.render_view_selector(r,0,1280);
        const auto none=r.button("HIDE HULLS");require(r.button("FULL HULL").w>0,"Full Hull button missing");
        app.ui_input_.mouse_x=int(none.x+none.w*.5f);app.ui_input_.mouse_y=int(none.y+none.h*.5f);
        app.ui_input_.left_pressed=true;app.render_view_selector(r,0,1280);app.ui_input_={};
        require(app.habitat_view_mode_==GameApp::HabitatViewMode::None,"Hide hulls did not select the cutaway");
        for(const auto layout:{GameApp::WorldLayoutMode::FullGraphics,GameApp::WorldLayoutMode::Mesh}){
            app.world_layout_mode_=layout;
            for(const float step:{4.f,float(kConstructionSteps)}){
                draw_room(step);
                require(r.drew_at(1)&&r.drew_at(2)&&!r.drew_at(3)&&!r.drew_at(4)&&!r.drew_at(5)&&!r.drew_at(6),
                        "Habitat cutaway retained shell/frames or lost its interior");
            }
            for(const auto kind:{ResourceKind::StorageDome,ResourceKind::RobotWorkshop,ResourceKind::OreDriller}){
                r.vertices.clear();app.render_resource_mesh(r,resource,{},0,false,true,false,kConstructionSteps,nullptr,kind);
                require(r.drew_at(1)&&r.drew_at(2),"Resource cutaway lost deck or equipment");
                require(r.drew_at(3)==(kind==ResourceKind::OreDriller),"Cutaway hid exterior machinery or retained a habitat shell");
            }
            r.vertices.clear();app.render_corridors(r);
            require(!r.drew_at(3)&&!r.drew_at(5),"Corridor cutaway retained its pressure shell");
        }
        require(!r.faded,"Hide hulls submitted a ghost pressure shell");
        std::cout<<"Full Hull default and clear frame-free cutaways, with exterior equipment retained, passed\n";
    }

    static void structural_frames(){
        using namespace game::runtime;GameHost host;GameApp app(host);
        const auto check=[&](const auto& mesh,const std::string& name,bool crew){
            const auto& s=mesh.reinforcement;
            const auto& frame=[&]() -> const std::vector<Vec3>& {if constexpr(requires {mesh.hull_frames;})return mesh.hull_frames;else return mesh.details;}();
            require(s.members>0&&s.frames>0&&s.vertices>0&&s.vertices%3==0,name+" missing solid structural geometry");
            require(s.first_vertex+s.vertices<=frame.size()&&s.vertices<400000,name+" frame range or geometry budget");
            require(s.max_bay_m<=1.501f,name+" frame bays stretched beyond design spacing");
            const float bottom=mesh.walkable_floor_y+.045f,top=mesh.walkable_floor_y+geometry::kHumanHeight-.035f;
            for(std::size_t i=s.first_vertex;i<s.first_vertex+s.vertices;i+=3){
                const auto a=frame[i],b=frame[i+1],c=frame[i+2];
                for(auto p:{a,b,c})require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),name+" non-finite member");
                require(length(cross(b-a,c-a))>1e-9f,name+" degenerate structural face");
                if(crew&&std::max({a.y,b.y,c.y})>bottom&&std::min({a.y,b.y,c.y})<top){
                    const bool crosses_core=std::min({a.x,b.x,c.x})<.40f&&std::max({a.x,b.x,c.x})>-.40f&&std::min({a.z,b.z,c.z})<.40f&&std::max({a.z,b.z,c.z})>-.40f;
                    if(crosses_core){
                        std::vector<Vec3> polygon{a,b,c};
                        for(const auto& plane:std::array<std::pair<Vec3,float>,6>{{{{1,0,0},.40f},{{-1,0,0},.40f},{{0,0,1},.40f},{{0,0,-1},.40f},{{0,1,0},top},{{0,-1,0},-bottom}}})polygon=mesh_portals::clip(polygon,plane.first,plane.second,false);
                        require(polygon.size()<3,name+" structural member crosses the central crew clearance");
                    }
                }
            }
        };
        std::size_t vertices=0;
        for(int k=0;k<int(HabitatKind::Count);++k)for(int t=0;t<int(HabitatSizeTier::Count);++t){
            const auto& m=app.habitat_models_.mesh(HabitatKind(k),HabitatSizeTier(t));check(m,"Habitat "+std::to_string(k)+"/"+std::to_string(t),true);vertices+=m.reinforcement.vertices;
        }
        for(int k=0;k<int(ResourceKind::Count);++k)for(auto size:{ResourceSizeTier::Small,ResourceSizeTier::Standard}){
            const auto& m=app.resource_models_.mesh(ResourceKind(k),size);check(m,"Resource "+std::to_string(k),k==int(ResourceKind::StorageDome)||k==int(ResourceKind::RobotWorkshop));
        }
        for(int t=0;t<int(StorageDomeSize::Count);++t){const auto& m=app.resource_models_.mesh(ResourceKind::StorageDome,StorageDomeSize(t));check(m,"Storage "+std::to_string(t),true);}
        const auto& small=app.habitat_models_.mesh(HabitatKind::CantineDome,HabitatSizeTier::Small);
        const auto& big=app.habitat_models_.mesh(HabitatKind::CantineDome,HabitatSizeTier::Big);
        require(big.reinforcement.frames>small.reinforcement.frames,"Larger glazing bays merely stretch the same ribs");
        // Compound halls have open internal transfers, not a new solid bulkhead.
        app.playfield_.rebuild(128);
        for(auto kind:{HabitatKind::Factory,HabitatKind::Manufactory}){
            PlacedHabitat hall;hall.kind=kind;hall.construction.finish();app.placed_habitats_={hall};
            const float y=app.habitat_models_.mesh(hall).walkable_floor_y;
            for(int i=0;i<10;++i)require(app.first_person_move_allowed({i*.3f,y,0},{(i+1)*.3f,y,0}),"New frame closes an internal hall transfer");
        }
        app.placed_habitats_.clear();
        // Frames are physical opaque members even while the solar-filter skin is visible.
        struct Visual:Recorder{std::size_t metal=0,glass=0;bool faded=false;
            void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c,const LunarSurfaceMapping&)override{draw_lunar_surface(v,m,c);}
            void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c)override{
                if(m==LunarMaterial::Alloy){metal+=v.size();faded|=c.a<.99f;}if(m==LunarMaterial::SolarShield&&c.a>.10f&&c.a<.5f)glass+=v.size();}
        } visual;
        app.graphics_.path=GraphicsPath::Classic;app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;
        app.render_habitat_mesh(visual,big,{},0,false,true,false,kConstructionSteps,nullptr,false,HabitatKind::CantineDome,true);
        require(visual.metal>=big.reinforcement.vertices&&visual.glass>0&&!visual.faded,"New frame faded away with transparent upper hull");
        // Straight pressure passage: the solid hoops are outside the complete crew envelope.
        app.playfield_.rebuild(128);CorridorConnection c;c.path={{0,0,0},{0,0,20}};c.construction.finish();
        c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.tangent={0,0,-1};c.endpoint_b.tangent={0,0,1};
        std::vector<CorridorConnection> passages{c};corridor_detail::rebuild_all_meshes(passages);app.corridor_network_.restore_connections(passages);
        require(passages[0].mesh.details.size()>4000,"Corridor ribs are still line-only");
        for(int z=2;z<18;++z)require(app.first_person_move_allowed({0,geometry::kCorridorDeckY,float(z)},{0,geometry::kCorridorDeckY,float(z)+.5f}),"Solid corridor frame blocks FPV centerline");
        // New under-panel members are in the tracked range; base braces stay stationary.
        const auto& solar=app.resource_models_.mesh(ResourceKind::SolarPanel);
        solar_collector::Pose pose(solar.solar_head,{},37,normalize(Vec3{.6f,.7f,.2f}));
        bool moved=false,stationary=false;
        for(std::size_t i=0;i<solar.details.size();++i){const bool tracked=solar_collector::moving(i,solar.solar_head.details);const auto p=pose.point(solar.details[i],solar.solar_head,tracked);
            if(tracked)moved|=length(p-solar.details[i])>.1f;else if(i>=solar.reinforcement.first_vertex){stationary=true;require(length(p-world_math::rotate_y(solar.details[i],37))<1e-5f,"Static solar base rotates with its head");}}
        require(moved&&stationary,"Solar framing ranges not separated at the bearing");
        std::cout<<"All habitat/resource tiers: solid load frames, bounded bays, central clearances, opaque metal/UV glazing, corridor FPV and moving solar frames passed; added habitat vertices="<<vertices<<"\n";
    }
    static void corridor_airlock_fpv(){
        using namespace runtime;
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);app.viewport_width_=1920;app.viewport_height_=1080;
        app.screen_=GameApp::Screen::Playfield;app.game_session_active_=true;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.game_speed_=GameApp::GameSpeed::Pause;
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard,HabitatSizeTier::Mid,HabitatSizeTier::Big}){
            PlacedHabitat dome;dome.kind=HabitatKind::OxygenGeneratorDome;dome.size=tier;dome.rotation_degrees=37;dome.construction.finish();
            app.placed_habitats_={dome};app.fit_oxygen_units();auto& room=app.placed_habitats_[0];
            require(room.units.size()==1,"Oxygen fitting fails after complete solid collision check");
            const auto& u=room.units[0];const float floor=app.habitat_models_.mesh(room).walkable_floor_y;
            const auto world=[&](Vec3 p){return room.position+world_math::rotate_y(p,room.rotation_degrees);};
            // These upright tanks were rendered in service_metal and omitted by FPV.
            const auto tank=world(u.position+world_math::rotate_y({.92f,0,.20f},u.rotation_degrees));
            require(!app.first_person_move_allowed(tank,tank),"FPV can enter oxygen generator tank");
            app.astronaut_views_[0].present=app.astronaut_views_[0].visited=true;
            app.test_astronauts_[0].position=tank;app.selected_astronaut_=0;
            app.enter_first_person_mode();const auto recovered=app.first_person_.feet_position();
            require(app.first_person_mode_==GameApp::FirstPersonMode::Active&&app.first_person_move_allowed(recovered,recovered),"Old embedded oxygen pose does not recover to clear floor");
            require(length(recovered-tank)<2.f,"Oxygen recovery moved beyond nearby floor");
            app.exit_first_person_mode();app.reset_first_person_crew();
            // Traverse both axes around the installed machine; each
            // accepted movement must permit the exact reverse, including diagonals.
            const float reach=app.habitat_models_.mesh(room).footprint_x*.25f;
            for(int axis=0;axis<2;++axis)for(int i=-30;i<30;++i){
                Vec3 a{axis?0.f:reach*i/30,floor,axis?reach*i/30:0.f};
                Vec3 b{axis?0.f:reach*(i+1)/30,floor,axis?reach*(i+1)/30:0.f};
                a=world(a);b=world(b);
                if(app.first_person_move_allowed(a,a)&&app.first_person_move_allowed(a,b))
                    require(app.first_person_move_allowed(b,a),"Oxygen movement enters a one-way collision trap");
            }
            // A real rotated hatch, sleeve and ramp must permit egress and
            // re-entry in every oxygen-dome tier with the generator installed.
            const auto direction=world_math::rotate_y({1,0,0},room.rotation_degrees);
            const CorridorBuildAnchor anchor{CorridorEndpointKind::Habitat,0,direction*35};
            const auto endpoint=app.corridor_network_.preview_anchor(anchor,direction*45,app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
            require(bool(endpoint),"Oxygen dome has no usable hatch anchor");
            CorridorConnection connection;connection.endpoint_a=*endpoint;connection.endpoint_b.kind=CorridorEndpointKind::OpenDoor;
            connection.path={endpoint->position,endpoint->position+endpoint->tangent*9};connection.endpoint_b.position=connection.path.back();connection.endpoint_b.tangent=endpoint->tangent;connection.construction.finish();
            std::vector<CorridorConnection> links{connection};corridor_detail::rebuild_all_meshes(links);app.corridor_network_.restore_connections(links);
            app.pressure_doors_.clear();for(const auto& door:app.habitat_doors(0)){PressureDoorState state;state.door=door;state.open_fraction=1;app.pressure_doors_.push_back(state);}
            const auto destination=endpoint->position+endpoint->tangent*2;
            const int steps=int(std::ceil(length(destination)/.05f));Vec3 previous{0,floor,0};
            for(int step=1;step<=steps;++step){auto next=destination*(float(step)/steps);next.y=app.first_person_floor_height(next);
                require(app.first_person_move_allowed(previous,next)&&app.first_person_move_allowed(next,previous),"Oxygen hatch/ramp traps FPV in tier "+std::to_string(int(tier))+" step "+std::to_string(step));previous=next;}
            app.corridor_network_.clear();app.pressure_doors_.clear();
            // Returning from FPV must leave the structure inspector reachable.
            app.camera_.restore({24,35.26439f,45,{0,.8f,0}});
            const auto vp=app.camera_.view_projection(1920.f/1080);
            const Vec2 target{(vp.m[12]/vp.m[15]*.5f+.5f)*1920,(.5f-vp.m[13]/vp.m[15]*.5f)*1080};
            InputState click;click.left_pressed=true;click.mouse_x=int(target.x);click.mouse_y=int(target.y);
            app.astronaut_views_[0].present=true;app.test_astronauts_[0].position={30,0,30};app.selected_astronaut_=0;
            require(!app.handle_astronaut_click(click)&&app.selected_astronaut_==-1,"Selected astronaut still swallows structure selection");
            app.select_world_at(click.mouse_x,click.mouse_y);
            require(app.selection_kind_==GameApp::SelectionKind::Habitat&&app.selected_habitat_==0,"Oxygen inspector is not selectable after FPV");
            app.reset_first_person_crew();app.clear_world_selection();
        }
        // Landing on a low installed unit must not put the body inside it.
        PlacedHabitat canteen;canteen.kind=HabitatKind::CantineDome;canteen.construction.finish();
        InteriorUnit table;table.kind=InteriorUnitKind::DiningTable;table.position={0,app.habitat_models_.mesh(canteen).walkable_floor_y,2};canteen.units={table};app.placed_habitats_={canteen};
        const auto top=table.position+Vec3{0,interior_unit_profile(table.kind).size.y,0};
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn(top+Vec3{0,.15f,0},0);
        for(int tick=0;tick<120;++tick)app.update_first_person(1.f/120,{});
        require(std::abs(app.first_person_.feet_position().y-top.y)<.001f&&app.first_person_move_allowed(app.first_person_.feet_position(),app.first_person_.feet_position()),"FPV falls into a solid installed unit after a jump");
        app.first_person_mode_=GameApp::FirstPersonMode::Off;app.placed_habitats_.clear();
        const auto run=[](std::vector<Vec3> path){CorridorConnection c;c.path=std::move(path);c.construction.finish();c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();c.endpoint_a.tangent=normalize(c.path.front()-c.path[1]);c.endpoint_b.tangent=normalize(c.path.back()-c.path[c.path.size()-2]);return c;};
        for(auto c:{run({{0,0,-10},{0,0,10}}),run({{-8,0,0},{0,0,0},{5,0,5}}),run({{0,2,-10},{0,2,10}})}){
            std::vector<CorridorConnection> links{c};corridor_detail::rebuild_all_meshes(links);const auto& mesh=links[0].mesh;
            require(mesh.deck_surface.size()>6000&&mesh.metal_surface.size()>3000&&mesh.grip_surface.size()>100,"Detailed deck or frame render batch is absent");
            require(mesh.deck_surface.size()+mesh.metal_surface.size()+mesh.grip_surface.size()==mesh.details.size(),"Corridor render partition loses geometry");
            for(const auto p:mesh.details)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&p.y>=-.001f,"Corridor fitting penetrates the ground or contains invalid vertices");
        }
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard}){
            PlacedHabitat airlock;airlock.kind=HabitatKind::AirLock;airlock.size=tier;airlock.construction.finish();airlock.supply.power_fraction=1;app.placed_habitats_={airlock};
            const auto& m=app.habitat_runtime_mesh(0);require(m.stations.empty()&&m.screens.size()==2,"Fitted wall services missing or unnecessary airlock floor equipment remains");
            const float y=m.walkable_floor_y;
            for(float x:{-3.f,-.5f,.5f,3.f})require(app.first_person_move_allowed({x,y,0},{x+.1f,y,0}),"Airlock equipment obstructs transfer aisle");
            struct HatchRecorder:Recorder{std::size_t control=0,text=0;
                void draw_lunar_surface(const std::vector<Vec3>& vertices,LunarMaterial material,Color)override{
                    if(material==LunarMaterial::Display)control+=vertices.size();if(material==LunarMaterial::DisplayText)text+=vertices.size();}
            } controls;
            app.graphics_.path=GraphicsPath::Classic;app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,y,0},90);
            app.render_habitats(controls);
            require(controls.control>0&&controls.text>100,"Airlock hatch controls have no mounted display or readable status");
            app.first_person_mode_=GameApp::FirstPersonMode::Off;
        }
        std::cout<<"Oxygen tanks/embedded pose recovery/reversible movement, FPV-to-inspector selection, detailed corridor batches/grounding and uncluttered airlock aisle passed\n";
    }
    static void player_control(){
        using namespace game::runtime;
        GameHost host;GameApp app(host);Recorder r;
        app.playfield_.rebuild(128);app.camera_.reset(128);app.on_resize(r.w,r.h);
        app.screen_=GameApp::Screen::Playfield;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.game_session_active_=true;app.game_speed_=GameApp::GameSpeed::Normal;
        app.lander_.phase=LanderPhase::Ready;app.lander_.site={-8,0,-10};app.lander_.salvaged=true;
        app.builder_robots_.resize(2);app.builder_robots_[0].position={-35,0,-35};app.builder_robots_[1].position={-36,0,-35};app.builder_robots_[1].role=RobotRole::Carrier;
        InputState v;v.key_v_pressed=true;app.update_simulation(0,v);
        require(app.first_person_mode_==GameApp::FirstPersonMode::Active&&app.controlled_astronaut_==0,"V requires free placement instead of immediate character control");
        require(length(app.first_person_.feet_position()-app.crew_cache_position())<.01f,"First FPV did not spawn beside the lander");
        require(app.astronaut_present(0)&&!app.astronaut_present(1),"Initial FPV created invisible extra crew");
        InputState light;light.key_l_pressed=true;
        app.update_simulation(0,light);
        require(app.astronaut_views_[0].suit_light&&!app.astronaut_views_[1].suit_light,"L did not enable only the controlled astronaut's light");
        app.update_simulation(0,light);
        require(!app.astronaut_views_[0].suit_light,"L did not toggle the suit light off");
        const auto start=app.first_person_.feet_position();InputState walk;walk.key_w_down=true;
        for(int frame=0;frame<20;++frame)app.update_first_person(.016f,walk);
        require(length(app.first_person_.feet_position()-start)>.1f&&length(app.test_astronauts_[0].position-app.first_person_.feet_position())<1e-6,"FPV does not move the selected astronaut");
        app.first_person_.restore_view(133,24);const auto pose=app.first_person_.feet_position();
        app.exit_first_person_mode();
        require(length(app.test_astronauts_[0].position-pose)<.001f&&app.selected_astronaut_==0,"Leaving FPV lost the astronaut position/selection");
        app.update_astronaut_test(2);
        require(length(app.test_astronauts_[0].position-pose)<.001f,"Unassigned astronaut walks away after exiting FPV");
        app.enter_first_person_mode();
        require(length(app.first_person_.feet_position()-pose)<.001f&&std::abs(app.first_person_.yaw_degrees()-133)<.001f&&std::abs(app.first_person_.pitch_degrees()-24)<.001f,"FPV did not restore last position and orientation");
        // The selected astronaut takes precedence over the last controlled one.
        app.exit_first_person_mode();app.astronaut_views_[1].present=true;app.test_astronauts_[1].position={35,0,35};
        app.test_astronauts_[1].health=.63f;app.test_astronauts_[1].hunger=.41f;app.selected_astronaut_=1;
        app.enter_first_person_mode();
        require(app.controlled_astronaut_==1&&length(app.first_person_.feet_position()-Vec3{35,0,35})<.001f,"FPV ignored the selected astronaut");
        const auto other=app.test_astronauts_[0].position;app.update_astronaut_test(1);
        require(length(app.test_astronauts_[1].position-app.first_person_.feet_position())<.001f&&length(app.test_astronauts_[0].position-other)<.001f,"Autonomy moves a controlled or held astronaut");
        const double oxygen=app.test_astronauts_[1].suit_oxygen_kg;const float thirst=app.test_astronauts_[1].thirst;
        app.update_supply_simulation(4);
        require(app.test_astronauts_[1].suit_oxygen_kg<oxygen&&app.test_astronauts_[1].thirst>thirst&&app.test_astronauts_[1].fatigue>0,"FPV vitals are detached from resource simulation");
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const Vec2 size:std::array<Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}}){
            app.ui_scale_index_=zoom;app.ui_scale_=layout::UI_SCALES[zoom];app.on_resize(int(size.x),int(size.y));r.resize(int(size.x),int(size.y));r.begin_frame({});r.set_2d(app.ui_scale_);
            app.render_astronaut_status(r,Mat4{});r.check_bounds("astronaut vitals HUD");
            const auto panel=app.astronaut_status_rect();bool alpha=false,health=false,hunger=false,sleep=false;
            for(const auto& d:r.draws){if(d.type=='R'&&std::abs(d.v[0]-panel.x)<.01f&&std::abs(d.v[1]-panel.y)<.01f&&d.color.a==.75f)alpha=true;
                health|=d.text=="HEALTH 63%";hunger|=d.text=="HUNGER 42%";sleep|=d.text.find("SLEEP NEED")==0;}
            require(alpha&&health&&hunger&&sleep,"Vitals HUD lacks live values, sleep need or 25% transparency");
            require(panel.y+panel.h<=size.y/app.ui_scale_-layout::BUILD_BAR_H,"Vitals cover bottom config controls");
        }
        app.ui_scale_index_=0;app.ui_scale_=1;app.on_resize(1920,1080);r.resize(1920,1080);
        // A new obstruction triggers a nearby safe return, not arbitrary relocation.
        app.exit_first_person_mode();GroundResourcePile pile;pile.resource=MaterialResource::Iron;pile.amount=20;pile.kind=GroundPileKind::Stack;pile.position={35,0,35};
        app.ground_resource_piles_={pile};app.enter_first_person_mode();
        require(!app.resource_packages_block(app.first_person_.feet_position())&&length(app.first_person_.feet_position()-pile.position)<2,"Blocked FPV position did not resolve to nearest safe space");
        app.exit_first_person_mode();app.ground_resource_piles_.clear();
        // Selected interior crew must remain inside their cabin on first control.
        PlacedHabitat cabin;cabin.kind=HabitatKind::Factory;cabin.position={25,0,-25};cabin.construction.finish();app.placed_habitats_={cabin};
        const float deck=app.habitat_models_.mesh(cabin).walkable_floor_y;
        app.test_astronauts_[1].position={25,deck,-25};app.astronaut_views_[1].visited=false;app.selected_astronaut_=1;app.enter_first_person_mode();
        require(length(app.first_person_.feet_position()-Vec3{25,deck,-25})<.001f,"Selecting interior crew teleports them outside");
        app.exit_first_person_mode();app.placed_habitats_.clear();app.test_astronauts_[1].position={40,0,40};app.enter_first_person_mode();
        app.first_person_.restore_view(259,-18);app.test_astronauts_[1].health=.77f;app.test_astronauts_[1].fatigue=.64f;
        app.astronaut_cache_.water_litres=73;std::string error;
        require(app.save_game_to_name("fpv-crew-roundtrip",error),"FPV save failed: "+error);
        const auto save=std::filesystem::path(app.save_game_directory())/"fpv-crew-roundtrip.gmsave";GameApp restored(host);
        if(!restored.load_game_from_file(save.string(),error))throw std::runtime_error("FPV save rejected: "+error);
        require(restored.first_person_mode_==GameApp::FirstPersonMode::Off&&restored.last_fpv_astronaut_==1&&restored.astronaut_present(0)&&restored.astronaut_present(1)&&!restored.astronaut_present(2),"Crew roster or last control identity was lost");
        restored.enter_first_person_mode();
        require(restored.controlled_astronaut_==1&&length(restored.first_person_.feet_position()-Vec3{40,0,40})<.001f&&std::abs(restored.first_person_.pitch_degrees()+18)<.001f&&std::abs(restored.first_person_.yaw_degrees()-259)<.001f,"Saved FPV pose does not restore");
        require(restored.test_astronauts_[1].health==.77f&&restored.test_astronauts_[1].fatigue==.64f&&restored.astronaut_cache_.water_litres==73&&restored.test_astronauts_[0].health==1,"Individual vitals or finite reserves lost on save/load");
        restored.exit_first_person_mode();
        require(restored.load_game_from_file((std::filesystem::path(GAME_SOURCE_ROOT)/"game/tests/fixtures/schema48-lander-depot.gmsave").string(),error),"Old compact save rejected: "+error);
        require(!restored.astronauts_active()&&restored.last_fpv_astronaut_==-1&&restored.astronaut_cache_.water_litres==AstronautCache{}.water_litres,"Older save inherited a different world's FPV crew");
        restored.reset_first_person_crew();require(restored.last_fpv_astronaut_==-1&&!restored.astronauts_active(),"New world retains FPV state");std::filesystem::remove(save);
        // Shared geometry gives all stacked packages matching visible and physical bounds.
        app.exit_first_person_mode();app.ground_resource_piles_.clear();pile.position={12,0,12};pile.amount=3*resource_package_capacity_kg(pile.resource);app.ground_resource_piles_={pile};
        int count=0;float top=0;packages::each(pile,[&](const packages::Box& box){++count;top=box.top();return true;});
        require(count==3&&!app.first_person_move_allowed({10,0,12},{14,0,12}),"FPV passes through a solid resource stack");
        require(app.first_person_move_allowed({12,top,12},{12,top,12})&&std::abs(app.first_person_floor_height({12,top,12})-top)<.001f,"A package cannot support an astronaut above it");
        require(app.first_person_move_allowed({12,top+.1f,12},{14,top+.1f,12}),"Package blocks motion above its solid surface");
        for(int material=0;material<int(MaterialResource::Count);++material){
            const auto m=MaterialResource(material);std::vector<Vec3> backing,ink;packages::markings({{0,0,0},.3f,.4f,m},backing,ink);
            require(backing.size()==36&&!ink.empty(),"Classification missing from one of six package faces");
            for(int other=0;other<material;++other)require(packages::icon(m)!=packages::icon(MaterialResource(other)),"Two material classifications share the same icon");
        }
        struct OpaqueRecorder:Recorder{bool saw_box=false,translucent=false;
            void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial material,Color c)override{
                if(material==LunarMaterial::Paint&&!v.empty()){saw_box=true;translucent|=c.a!=1;}
            }
        } boxes;
        app.first_person_.spawn({12,0,14},0);app.first_person_mode_=GameApp::FirstPersonMode::Active;app.render_ground_resources(boxes);
        require(boxes.saw_box&&!boxes.translucent,"Resource package faces remain transparent");
        std::cout<<"Direct astronaut control, lander spawn, FPV continuity, nearest safe return, schema52 vitals, HUD scaling and solid classified packages passed\n";
    }
    static void fpv_building(){
        using namespace game::runtime;
        GameHost host;GameApp app(host);Recorder r;
        app.playfield_.rebuild(128);app.camera_.reset(128);
        app.screen_=GameApp::Screen::Playfield;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.game_session_active_=true;app.game_speed_=GameApp::GameSpeed::Pause;
        app.graphics_.path=GraphicsPath::Classic;app.first_person_mode_=GameApp::FirstPersonMode::Active;
        const auto select=[&](const char* id){
            const auto& items=app.build_catalog_.items();
            const auto it=std::find_if(items.begin(),items.end(),[&](const auto& item){return item.id==id;});
            require(it!=items.end(),std::string("Missing FPV build tool ")+id);
            app.build_catalog_.set_category(it->category);app.build_catalog_.select(int(it-items.begin()));
        };
        const auto aim=[&](Vec3 feet){
            app.first_person_.spawn(feet,0);InputState look;look.mouse_dy=82;app.first_person_.update_look(look);
            app.input_={};app.input_.mouse_x=r.w/2;app.input_.mouse_y=r.h/2;
        };
        aim({20,0,20});app.render(r);r.check_bounds("FPV bottom controls");
        require(r.button("FULL HULL").y>r.h/r.scale-layout::BUILD_BAR_H && r.button("HIDE HULLS").w>0,"FPV bottom configuration bar missing");
        Vec3 hit;require(app.pick_build_ground(hit)&&hit.z<12,"FPV center aim did not intersect the ground");
        select("crew_quarters");InputState rotation;rotation.key_m_pressed=true;
        require(app.update_placement_controls(rotation)&&app.placement_rotation_degrees_==15,"FPV rotation did not use structure controls");
        rotation={};rotation.wheel=1;const auto before=app.placement_habitat_size_;app.update_placement_controls(rotation);
        require(app.placement_habitat_size_!=before,"FPV wheel did not change structure size");
        app.input_.left_pressed=true;app.render(r);
        require(app.placed_habitats_.size()==1,"FPV center-aim structure placement failed: "+app.placement_tip_detail_);
        select("solar_panel");aim({-20,0,20});app.first_person_cursor_=true;
        app.input_.mouse_x=4;app.input_.mouse_y=r.h-10;app.input_.left_pressed=true;app.render(r);
        require(app.placed_resources_.empty()&&!app.pick_build_ground(hit),"HUD click leaked into FPV world placement");
        app.input_.mouse_x=r.w/2;app.input_.mouse_y=r.h/2;app.render(r);
        require(app.placed_resources_.size()==1,"FPV free-cursor resource placement failed: "+app.placement_tip_detail_);
        InputState tab;tab.key_tab_pressed=true;app.update_simulation(1.f/60,tab);
        require(!app.first_person_cursor_,"Tab did not return to FPV mouse look");
        app.update_simulation(1.f/60,tab);require(app.first_person_cursor_,"Tab did not release the FPV cursor");
        select("crew_quarters");InputState cancel;cancel.right_pressed=true;app.update_simulation(1.f/60,cancel);
        require(!app.build_catalog_.selected()&&app.first_person_mode_==GameApp::FirstPersonMode::Active,"RMB did not cancel the FPV tool");
        struct BorderRecorder:Recorder{
            bool bright{},backing{},posts{};
            void draw_lines_3d(const std::vector<Vec3>& v,Color c,float w) override {
                bright=bright||(c.a==1&&w>=2.5f&&c.b>.8f);backing=backing||(c.r<.05f&&w>=5.f);
                for(std::size_t i=1;i<v.size();i+=2)posts=posts||(std::abs(v[i].y-v[i-1].y)>.6f);
            }
        } borders;
        app.render_build_footprints(borders);require(borders.bright&&borders.backing&&borders.posts,"Build boundaries lack bright strokes, dark backing or FPV corner posts");
        const auto count=[](float speed,int fps){sound::FootstepCadence cadence;int n=0;for(int i=0;i<fps*10;++i)n+=cadence.advance(speed/fps,1.f/fps,true);return n;};
        require(count(2,60)>=19&&count(2,60)<=20&&count(4.2f,60)>=26&&count(4.2f,60)<=27,"Walking or sprint cadence is unrealistic");
        require(std::abs(count(2,30)-count(2,144))<=1,"Footstep cadence depends on render rate");
        sound::FootstepCadence cadence;
        for(int i=0;i<300;++i)require(!cadence.advance(0,1.f/60,true)&&!cadence.advance(.05f,1.f/60,false),"Footsteps play at rest or in the air");
        require(!cadence.advance(10,1.f/60,true),"Teleport emitted a footstep burst");
        for(auto cue:{sound::Cue::FootDust,sound::Cue::FootDeck}){
            std::vector<std::int16_t> previous;
            for(std::uint32_t variant=0;variant<4;++variant){const auto pcm=sound::synthesize(cue,variant);int peak=0;double energy=0;
                for(auto value:pcm){peak=std::max(peak,std::abs(int(value)));energy+=double(value)*value;}
                require(peak>1000&&peak<30000&&energy>1e6&&pcm!=previous,"Footstep clip is silent, clipped or lacks variants");previous=pcm;
            }
        }
        std::cout<<"FPV bottom controls, ground-ray placement, HUD isolation, borders and footstep cadence passed\n";
    }

    static void station_visuals(){
        using namespace game::runtime;
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        struct Materials:Recorder {
            std::vector<Vec3> glazing,paint,text,structure;
            bool bad_alpha{};
            void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c,const LunarSurfaceMapping&) override {draw_lunar_surface(v,m,c);}
            void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c) override {
                auto* out=m==LunarMaterial::SolarShield?&glazing:m==LunarMaterial::PressureDoor?&paint:m==LunarMaterial::DisplayText?&text:m==LunarMaterial::Alloy?&structure:nullptr;
                if(out)out->insert(out->end(),v.begin(),v.end());
                if(m==LunarMaterial::SolarShield)bad_alpha=bad_alpha||c.a<.10f||c.a>.5f;
            }
        };
        std::vector<Vec3> changed_skin{{0,0,0},{1,0,0},{0,2,0}};
        require(!hull_surface::split(changed_skin,1).upper.empty(),"Hull split omitted upper skin");
        changed_skin[2].y=.5f;
        require(hull_surface::split(changed_skin,1).upper.empty(),"Hull cache retained obsolete geometry after a same-size rebuild");
        int mounts=0;
        for(int kind=0;kind<int(HabitatKind::Count);++kind)for(int tier=0;tier<int(HabitatSizeTier::Count);++tier){
            const auto& mesh=app.habitat_models_.mesh(HabitatKind(kind),HabitatSizeTier(tier));
            require(mesh.screens.size()>=(kind==int(HabitatKind::AirLock)?0u:kind==int(HabitatKind::OxygenGeneratorDome)?2u:4u) && mesh.displays.size()==mesh.screens.size()*6,"A room variant has missing screen mounts or luminous box faces: "+std::to_string(kind)+"/"+std::to_string(tier)+" mounts="+std::to_string(mesh.screens.size())+" faces="+std::to_string(mesh.displays.size()));
            for(const auto& screen:mesh.screens){
                require(std::abs(length(screen.right)-1)<1e-5 && std::abs(dot(screen.right,screen.up))<1e-5,"Screen mounting basis is distorted");
                require(screen.width>.4f && screen.height>.25f,"Screen is too small to read");++mounts;
            }
            Materials visual;app.render_habitat_mesh(visual,mesh,{},0,false,true,false,kConstructionSteps,nullptr,false,HabitatKind(kind),true);
            require(!visual.glazing.empty()&&!visual.bad_alpha,"A room upper hull lost its transparent solar film");
            require(!visual.structure.empty(),"Transparent hull removed solid structural parts");
        }
        PlacedHabitat oxygen;oxygen.kind=HabitatKind::OxygenGeneratorDome;oxygen.construction.finish();oxygen.supply.power_fraction=.75;
        oxygen.supply.oxygen_produced_kg_per_day=12.3;oxygen.supply.water_processed_litres_per_day=13.8;
        app.placed_habitats_={oxygen};const auto content=app.habitat_screen_content(0);
        require(content.title=="OXYGEN SYSTEM"&&content.rows[0]=="O2 12.3 KG/D"&&content.rows[1]=="WATER 13.8 L/D","Oxygen screen invents telemetry or loses units");
        const structure_display::Mount screen{{0,0,0},{1,0,0},{0,1,0},.9f,.48f};Materials text;
        structure_display::draw(text,screen,{},0,content,true,{0,0,2});
        require(text.text.size()>300,"Screen did not emit readable glyph geometry");
        for(const auto v:text.text)require(std::abs(v.z-.004f)<1e-5,"Screen lettering floats away from its face");
        const auto first=text.text;text.text.clear();auto changed=content;changed.rows[0]="O2 24.6 KG/D";
        structure_display::draw(text,screen,{},0,changed,true,{0,0,2});
        require(first.size()!=text.text.size()||std::memcmp(first.data(),text.text.data(),first.size()*sizeof(Vec3))!=0,"Screen glyphs do not follow changed measurements");
        text.text.clear();structure_display::draw(text,screen,{},0,content,false,{0,0,2});require(text.text.empty(),"Unpowered terminal has emissive text");
        Materials door;pressure_hardware::leaf(door,{}, {1,0,0},{0,0,1},.8f,2.2f,.16f);
        require(!door.paint.empty()&&!door.structure.empty(),"Pressure door lacks coated skin and structural hardware");
        for(auto v:door.paint)require(std::abs(v.x)<=.401f&&std::abs(v.y)<=1.101f&&std::abs(v.z)<=.081f,"Door rework exceeds its physical envelope");
        app.screen_=GameApp::Screen::Playfield;app.game_speed_=GameApp::GameSpeed::Pause;
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({20,0,20});
        app.first_person_cursor_=false;InputState alt;alt.key_alt_down=true;alt.mouse_dx=300;
        const auto direction=app.first_person_.look_direction();app.update_simulation(1.f/60,alt);
        require(app.first_person_cursor_free()&&length(app.first_person_.look_direction()-direction)<1e-6,"Alt did not release the cursor without moving FPV");
        app.update_simulation(1.f/60,alt);require(length(app.first_person_.look_direction()-direction)<1e-6,"Holding Alt still steers FPV");
        alt.key_alt_down=false;app.update_simulation(1.f/60,alt);
        require(!app.first_person_cursor_free()&&length(app.first_person_.look_direction()-direction)<1e-6,"Alt release jumps the FPV camera");
        app.first_person_cursor_=true;alt.key_alt_down=true;app.update_simulation(1.f/60,alt);alt.key_alt_down=false;app.update_simulation(1.f/60,alt);
        require(app.first_person_cursor_free(),"Alt release lost the Tab cursor preference");
        std::cout<<mounts<<" screen mounts, live telemetry glyphs, transparent upper hulls, pressure hardware and Alt capture passed\n";
    }

    static void gates_layout_solar(){
        using namespace game::runtime;using namespace world_math;
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        PlacedHabitat grow;grow.kind=HabitatKind::BioDome;grow.construction.finish();grow.placement_sequence=1;
        PlacedHabitat destination;destination.kind=HabitatKind::CrewQuarters;destination.position={20,0,0};destination.construction.finish();destination.placement_sequence=2;
        app.placed_habitats_={grow,destination};
        const auto& source=app.habitat_models_.mesh(grow);
        require(std::none_of(source.stations.begin(),source.stations.end(),[](const auto& station){return station.kind==StationKind::Grow;}),"Dome contains obsolete template machines");
        InteriorUnit rack;rack.kind=InteriorUnitKind::HydroponicRack;
        require(app.find_interior_unit_bay(0,rack),"No legal grow rack bay");rack.grow_bay=0;
        app.placed_habitats_[0].units.push_back(rack);
        const auto radii=source.dome_radii;const auto normal=normalize(Vec3{rack.position.x/(radii.x*radii.x),0,rack.position.z/(radii.z*radii.z)});
        const float wall_radius=1/std::sqrt(normal.x*normal.x/(radii.x*radii.x)+normal.z*normal.z/(radii.z*radii.z));
        CorridorConnection corridor;corridor.endpoint_a.kind=CorridorEndpointKind::Habitat;corridor.endpoint_a.index=0;
        corridor.endpoint_a.door.wall_position=normal*wall_radius;corridor.endpoint_a.door.wall_position.y=source.walkable_floor_y;
        corridor.endpoint_a.door.position=corridor.endpoint_a.door.wall_position;corridor.endpoint_a.door.normal=normal;corridor.endpoint_a.position=corridor.endpoint_a.door.position;
        corridor.endpoint_b.kind=CorridorEndpointKind::Habitat;corridor.endpoint_b.index=1;corridor.endpoint_b.door.wall_position={15,source.walkable_floor_y,0};corridor.endpoint_b.door.position=corridor.endpoint_b.door.wall_position;corridor.endpoint_b.door.normal={-1,0,0};corridor.endpoint_b.position=corridor.endpoint_b.door.position;
        corridor.path={corridor.endpoint_a.position,corridor.endpoint_b.position};
        require(!app.interior_unit_reserved(0,rack),"Rack reserved without a gate");
        const auto before_vertices=app.habitat_runtime_mesh(0).service_metal.size();
        app.corridor_network_.restore_connections({corridor});
        const auto& filtered=app.habitat_runtime_mesh(0);
        require(app.interior_unit_reserved(0,rack)&&filtered.service_metal.size()<before_vertices,"Planned gate did not remove the complete rack assembly");
        require(filtered.shell.size()==source.shell.size()&&filtered.glazing.size()==source.glazing.size()&&std::memcmp(filtered.glazing.data(),source.glazing.data(),source.glazing.size()*sizeof(Vec3))==0,"Station reservation cut an unbuilt doorway through the pressure hull");
        std::string error;require(!app.interior_unit_position_clear(0,rack,error)&&error.find("GATE")!=std::string::npos,"Gate approach allows replacement equipment");
        const auto mask=filtered.blocked_grow_bays;app.placed_habitats_[0].hydroponics[0].plant=HydroponicPlant::Potato;
        app.update_supply_simulation(0);require(app.placed_habitats_[0].blocked_grow_bays==mask,"Supply simulation ignores reserved grow bays");
        const auto before=app.placed_habitats_[0].hydroponics[0].growth_fraction;app.update_supply_simulation(.1f);
        require(app.placed_habitats_[0].hydroponics[0].growth_fraction==before,"Reserved grow bay continues operating");
        app.corridor_network_.clear();require(!app.interior_unit_reserved(0,rack),"Cancelled gate kept a placeable rack reserved");
        struct Frames:Recorder {int red{},green{};void draw_lines_3d(const std::vector<Vec3>&,Color color,float) override {
            if(color.r==1.f&&color.g==.16f)++red;if(color.r==.18f&&color.g==.95f)++green;
        }};
        app.placed_habitats_[0].kind=HabitatKind::AirLock;PlacedResource store;store.kind=ResourceKind::StorageDome;store.position={45,0,0};store.construction.finish();app.placed_resources_={store};
        for(auto& h:app.placed_habitats_)h.supply.power_fraction=1;
        app.placed_resources_[0].supply.power_fraction=1;
        Frames disconnected;app.render_gate_connection_frames(disconnected);require(disconnected.red==0&&disconnected.green==0,"Unused powered frames still have red warnings");
        app.corridor_network_.restore_connections({corridor});Frames planned;app.render_gate_connection_frames(planned);require(planned.green==0,"Unfinished gate is marked accessible");
        corridor.construction.finish();app.corridor_network_.restore_connections({corridor});Frames connected;app.render_gate_connection_frames(connected);
        require(connected.green>=3&&connected.red==0,"Powered connected gates do not have green frames");
        corridor.endpoint_b.kind=CorridorEndpointKind::OpenDoor;app.corridor_network_.restore_connections({corridor});Frames capped;app.render_gate_connection_frames(capped);require(capped.green==0&&capped.red==0,"A powered capped corridor still has a connection warning");
        auto cross_a=corridor,cross_b=corridor;
        cross_a.path={{-10,0,0},{10,0,0}};cross_b.path={{0,0,-10},{0,0,10}};cross_b.endpoint_a.index=1;
        app.corridor_network_.restore_connections({cross_a,cross_b});Frames crossing;app.render_gate_connection_frames(crossing);
        require(crossing.green>=2,"An automatic pressure crossing does not connect the gate frames");
        cross_b.path={{-10,0,10},{10,0,10}};app.corridor_network_.restore_connections({cross_a,cross_b});Frames separate;app.render_gate_connection_frames(separate);
        require(separate.green==0,"Changed corridor paths reused obsolete gate connectivity");
        cross_b.endpoint_b.kind=CorridorEndpointKind::Corridor;cross_b.endpoint_b.index=0;
        app.corridor_network_.restore_connections({cross_a,cross_b});Frames branched;app.render_gate_connection_frames(branched);
        require(branched.green>=2,"An explicit corridor branch does not connect the gate frames");
        const auto& warehouse=app.resource_models_.mesh(store);
        require(warehouse.stations.size()==4,"Storage service cabinets lack whole-assembly metadata");
        const auto cabinet=warehouse.stations.front();const auto cabinet_center=(cabinet.low+cabinet.high)*.5f;
        const auto outward=normalize(Vec3{cabinet_center.x,0,cabinet_center.z});
        corridor.endpoint_a.kind=CorridorEndpointKind::Resource;corridor.endpoint_a.index=0;
        corridor.endpoint_a.door.wall_position=store.position+Vec3{outward.x*5.35f,warehouse.walkable_floor_y,outward.z*5.35f};corridor.endpoint_a.door.normal=outward;
        app.corridor_network_.restore_connections({corridor});const auto& cleared_store=app.resource_runtime_mesh(0);
        require(!cleared_store.stations.front().active&&cleared_store.details.size()<warehouse.details.size(),"Storage doorway leaves an obstructing service cabinet in place");
        require(cleared_store.body.size()==warehouse.body.size(),"Storage reservation damages the hull");
        // Change a reservation to the opposite, equally sized assembly. This
        // exercises cache invalidation when geometry counts remain identical.
        const auto first_removed=cleared_store.details;
        corridor.endpoint_a.door.wall_position=store.position+Vec3{-outward.x*5.35f,warehouse.walkable_floor_y,-outward.z*5.35f};corridor.endpoint_a.door.normal=outward*-1;
        app.corridor_network_.restore_connections({corridor});const auto& other_store=app.resource_runtime_mesh(0);
        require(other_store.stations.front().active&&!other_store.stations.back().active,"Storage station cache keeps an obsolete reservation");
        require(first_removed.size()==other_store.details.size()&&std::memcmp(first_removed.data(),other_store.details.data(),first_removed.size()*sizeof(Vec3))!=0,"Equal-size cabinet changes were not rebuilt");
        app.corridor_network_.clear();require(app.resource_runtime_mesh(0).details.size()==warehouse.details.size(),"Cancelled storage reservation kept a removed cabinet");
        std::size_t checked=0;
        for(const auto size:{ResourceSizeTier::Standard,ResourceSizeTier::Small}){
            const auto& solar=app.resource_models_.mesh(ResourceKind::SolarPanel,size);
            require(!solar.solar_head.body.empty()&&!solar.solar_head.details.empty(),"Collector head has no rigid part metadata");
            for(float yaw:{0.f,37.f,150.f,270.f})for(float hour:{6.1f,9.f,12.f,17.9f})for(Vec3 position:{Vec3{},Vec3{80,0,-90}}){
                const auto normal=solar_tracking::pose(hour).direction;const solar_collector::Pose pose(solar.solar_head,position,yaw,normal);const auto mapping=pose.mapping();
                for(const auto range:solar.solar_head.body)for(std::size_t i=range.first;i<range.last;++i){
                    const auto q=solar.body[i]-solar.solar_head.pivot;const auto point=pose.point(solar.body[i],solar.solar_head,true);const auto uv=mapping.coordinate(point);
                    const Vec3 expected{dot(q,pose.authored_x),dot(q,pose.authored_normal),dot(q,pose.authored_z)};
                    require(length(uv-expected)<.0001f,"Solar surface UVs drift while translating, yawing or tracking");++checked;
                }
                const auto first=solar.solar_head.body.front().first;
                const auto a=pose.point(solar.body[first],solar.solar_head,true),b=pose.point(solar.body[first+1],solar.solar_head,true);
                require(std::abs(length(a-b)-length(solar.body[first]-solar.body[first+1]))<.0001f,"Tracker stretches the collector surface");
                require(length(pose.point(solar.body[0],solar.solar_head,false)-(position+rotate_y(solar.body[0],yaw)))<.0001f,"Tracker rotates the fixed mast");
            }
        }
        std::cout<<"Gate colors, removed collision assemblies, reserved operating slots and "<<checked<<" locked solar UV samples passed\n";
    }

    static void interior_services(){
        using namespace game::runtime;
        GameHost engine;GameApp app(engine);app.playfield_.rebuild(128);app.on_resize(1920,1080);
        app.screen_=GameApp::Screen::Playfield;app.game_session_active_=true;
        auto clock=app.status_.state();clock.hour=12;clock.crew_current=0;app.status_.restore(clock);
        app.resources_.restore({500,500,500,500,500,500,500,500});
        PlacedHabitat room;room.kind=HabitatKind::CantineDome;room.construction.finish();room.placement_sequence=1;
        app.placed_habitats_={room};const float floor=app.habitat_models_.mesh(room).walkable_floor_y;
        for(auto kind:{InteriorUnitKind::FoodProcessor,InteriorUnitKind::DrinkingWell,InteriorUnitKind::DiningTable,InteriorUnitKind::SeatingBench}){
            require(interior_unit_allowed(HabitatKind::CantineDome,kind)&&!interior_unit_allowed(HabitatKind::Factory,kind),"Canteen catalog compatibility");
            InteriorUnit unit;unit.kind=kind;bool fit=false;std::string reason;
            for(float z=-3;z<=3&&!fit;z+=.25f)for(float x=-3;x<=3&&!fit;x+=.25f){
                unit.position={x,floor,z};if(app.interior_unit_position_clear(0,unit,reason))fit=true;
            }
            require(fit,"Canteen unit has no valid placement: "+std::string(interior_unit_profile(kind).name)+" / "+reason);
            const auto stock=app.resources_.amount(MaterialResource::Iron),cost=interior_unit_profile(kind).cost.iron;
            require(app.place_interior_unit(0,unit,reason),"Canteen unit cannot install: "+reason);
            require(app.resources_.amount(MaterialResource::Iron)==stock-cost,"Unit installation cost");
            if(kind==InteriorUnitKind::DiningTable||kind==InteriorUnitKind::SeatingBench){
                require(interior_unit_mesh(kind).screens.empty()&&interior_unit_profile(kind).power_kw==0,"Passive furniture draws power");
                Recorder screen;app.render_interior_unit_screens(screen,0);
            }
            app.placed_habitats_[0].units.clear();
        }
        InteriorUnit processor;processor.kind=InteriorUnitKind::FoodProcessor;processor.position={0,floor,2.25f};
        auto& host=app.placed_habitats_[0];host.units={processor};host.supply.power_fraction=1;host.supply.volume_m3=200;host.supply.oxygen_kg=200*kOxygenKgPerM3;
        const auto food=app.resources_.amount(MaterialResource::Food),water=app.resources_.amount(MaterialResource::Water);
        app.update_interior_units(1);require(host.units[0].batch_active&&app.resources_.amount(MaterialResource::Food)==food-2&&app.resources_.amount(MaterialResource::Water)==water-1,"Processor ingredient accounting");
        const auto progress=host.units[0].progress;host.units[0].enabled=false;app.update_interior_units(100);
        require(host.units[0].progress==progress,"Disabled processor continued");host.units[0].enabled=true;
        app.update_interior_units(48);require(host.units[0].food_kg==2&&app.resources_.amount(MaterialResource::Food)==food-2,"Prepared meals not buffered");
        host.units[0].food_kg=20;app.update_interior_units(48);require(!host.units[0].batch_active,"Full meal buffer consumed ingredients");
        InteriorUnit well;well.kind=InteriorUnitKind::DrinkingWell;well.position={-2,floor,2};host.units.push_back(well);
        std::string error;app.builder_robots_.resize(1);
        require(app.save_game_to_name("interior-services-roundtrip",error),error);GameApp loaded(engine);
        const auto path=std::filesystem::path(app.save_game_directory())/"interior-services-roundtrip.gmsave";
        require(loaded.load_game_from_file(path.string(),error),error);require(loaded.placed_habitats_[0].units[0].food_kg==20,"Meal save inventory changed");std::filesystem::remove(path);
        require(loaded.load_game_from_file((std::filesystem::path(GAME_SOURCE_ROOT)/"game/tests/fixtures/schema49-interior-batch.gmsave").string(),error),"Schema49 migration: "+error);
        require(loaded.placed_habitats_[0].units.size()==1&&loaded.placed_habitats_[0].units[0].batch_active&&loaded.placed_habitats_[0].units[0].water_litres==0,"Old unit batch changed");
        host.units={well};PlacedResource tank;tank.kind=ResourceKind::WaterStorage;tank.construction.finish();tank.position={20,0,0};tank.supply.water_litres=80;
        PlacedResource solar=tank;solar.kind=ResourceKind::SolarPanel;solar.position={30,0,0};solar.supply.water_litres=0;app.placed_resources_={tank,solar};
        UtilityConnection a;a.endpoint_a.kind=UtilityEndpointKind::Habitat;a.endpoint_a.index=0;a.endpoint_b.kind=UtilityEndpointKind::Resource;a.endpoint_b.index=0;a.path={{0,0,0},{20,0,0}};a.construction.finish();
        UtilityConnection b=a;b.endpoint_a.kind=UtilityEndpointKind::Resource;b.endpoint_a.index=0;b.endpoint_b.index=1;b.path={{20,0,0},{30,0,0}};
        app.utility_network_.restore_connections({a,b});app.update_supply_simulation(100);
        require(std::abs(host.units[0].water_litres-10)<1e-7&&std::abs(app.placed_resources_[0].supply.water_litres-70)<1e-6,"Well water conservation");
        host.units[0].water_litres=0;host.units[0].enabled=false;app.update_supply_simulation(10);require(host.units[0].water_litres==0,"Disabled well pumps water");
        host.units[0].enabled=true;host.units[0].water_litres=3.75;require(app.save_game_to_name("interior-services-roundtrip",error),error);
        require(loaded.load_game_from_file(path.string(),error),error);require(loaded.placed_habitats_[0].units[0].water_litres==3.75,"Well save inventory changed");std::filesystem::remove(path);
        app.builder_robots_.clear();host.supply.power_fraction=1;host.supply.lighting_fraction=-1;require(app.interior_light_level(0)==0,"Lights active by day");
        clock.hour=0;app.status_.restore(clock);require(app.interior_light_level(0)==.18f,"Empty room has no low night lights");
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,floor,0},0);require(app.interior_light_level(0)==1,"FPV entry does not brighten room");
        app.first_person_.spawn({30,floor,0},0);require(app.interior_light_level(0)==.18f,"Outside observer counts as occupant");app.first_person_mode_=GameApp::FirstPersonMode::Off;
        app.builder_robots_.resize(1);app.builder_robots_[0].position={0,floor,0};require(app.interior_light_level(0)==1,"Rover occupancy ignored");
        app.builder_robots_[0].position={0,floor-1,0};require(app.interior_light_level(0)==.18f,"Below-deck rover counts as occupant");
        host.supply.power_fraction=0;require(app.interior_light_level(0)==0,"Unpowered room lights remain on");
        // Ordinary colony residents also consume prepared food before raw stock.
        InteriorUnit ready_meals=processor;ready_meals.food_kg=2;host.units={ready_meals};host.power_enabled=false;
        clock.crew_current=4;app.status_.restore(clock);const auto raw_food=app.resources_.amount(MaterialResource::Food);
        app.update_resource_economy(1);
        require(std::abs(host.units[0].food_kg-(2.-4.*.85/kGameDaySeconds))<1e-9&&app.resources_.amount(MaterialResource::Food)==raw_food,"Normal colony ignores prepared meals");
        clock.crew_current=0;app.status_.restore(clock);host.power_enabled=true;
        if(build::debug_features){
            app.active_game_phase_=GameApp::GamePhase::Debug;app.stage3_mode_=GameApp::Stage3Mode::AstronautTest;
            for(auto& actor:app.test_astronauts_)actor.health=0;
            auto& actor=app.test_astronauts_[0];actor.health=1;actor.position={0,floor,0};actor.food_pack_kg=0;
            InteriorUnit meals=processor;meals.food_kg=2;host.units={well,meals};host.units[0].water_litres=5;host.supply.oxygen_kg=host.supply.volume_m3*kOxygenKgPerM3;host.power_enabled=false;
            app.update_supply_simulation(1);require(host.units[0].water_litres<5&&host.units[1].food_kg<2&&actor.food_pack_kg==0,"Crew ignores stored canteen supplies without electricity");
        }
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard,HabitatSizeTier::Mid,HabitatSizeTier::Big}){
            GameApp oxygen(engine);oxygen.playfield_.rebuild(128);room.kind=HabitatKind::OxygenGeneratorDome;room.size=tier;room.units.clear();oxygen.placed_habitats_={room};
            oxygen.fit_oxygen_units();const auto& fitted=oxygen.placed_habitats_[0];
            require(fitted.oxygen_unit_fitted&&fitted.units.size()==1,"Included oxygen generator does not fit tier "+std::to_string(int(tier)));
            require(interior_unit_profile(fitted.units[0].kind).size.y>=2&&oxygen.interior_unit_panel_height(0)>0,"Oxygen dimensions/catalog");
            oxygen.placed_habitats_[0].units.clear();oxygen.fit_oxygen_units();require(oxygen.placed_habitats_[0].units.empty(),"Removed generator respawned");
            if(tier==HabitatSizeTier::Standard){
                oxygen.builder_robots_.resize(1);require(oxygen.save_game_to_name("oxygen-unit-removed",error),error);
                const auto removed=std::filesystem::path(oxygen.save_game_directory())/"oxygen-unit-removed.gmsave";GameApp restored(engine);
                require(restored.load_game_from_file(removed.string(),error),error);require(restored.placed_habitats_[0].oxygen_unit_fitted&&restored.placed_habitats_[0].units.empty(),"Removed generator respawned after load");std::filesystem::remove(removed);
            }
        }
        const auto vp=Mat4::identity();require(sound::construction_presence({0,0,0},{0,0,1},vp)>0,"Visible construction muted");
        require(sound::construction_presence({2,0,0},{0,0,1},vp)==0&&sound::construction_presence({0,0,2},{0,0,1},vp)==0,"Offscreen construction audible");
        std::cout<<"Canteen recipes, water conservation, old/new saves, night occupancy, oxygen fit and construction acoustics passed\n";
    }


    static void interior_systems(){
        using namespace game::runtime;
        ColonyStatus clock;clock.reset();clock.update(240);
        require(clock.day()==1&&std::abs(clock.hour()-20)<.001f,"Day is not twice as long");clock.update(240);
        require(clock.day()==2&&std::abs(clock.hour()-8)<.001f,"480-second day does not wrap correctly");
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);app.on_resize(1920,1080);
        app.screen_=GameApp::Screen::Playfield;app.game_session_active_=true;app.active_game_phase_=GameApp::GamePhase::RobotPreparation;app.builder_robots_.resize(1);
        ResourceBundle rich{500,500,500,500,500,500,500,500};app.resources_.restore(rich);
        PlacedHabitat hall;hall.kind=HabitatKind::Factory;hall.construction.finish();hall.placement_sequence=1;hall.position={0,0,0};
        hall.supply.volume_m3=200;hall.supply.oxygen_kg=200*kOxygenKgPerM3;hall.supply.power_fraction=1;
        app.placed_habitats_={hall};
        for(int k=0;k<int(HabitatKind::Count);++k){app.placed_habitats_[0].kind=static_cast<HabitatKind>(k);const auto& cleared=app.habitat_runtime_mesh(0);
            for(const auto& station:cleared.stations)if(station.kind==StationKind::Equipment||station.kind==StationKind::Grow)
                require(!station.active,"Template operating machinery was not replaced by units");
        }
        app.placed_habitats_[0]=hall;app.placed_habitats_[0].kind=HabitatKind::BioDome;
        auto& garden=app.placed_habitats_[0];InteriorUnit rack;rack.kind=InteriorUnitKind::HydroponicRack;
        rack.position={0,app.habitat_models_.mesh(garden).walkable_floor_y,2};garden.units={rack};
        require((app.habitat_runtime_mesh(0).blocked_grow_bays&1u)==0,"Installed rack did not enable its crop bay");
        garden.units[0].enabled=false;require((app.habitat_runtime_mesh(0).blocked_grow_bays&1u)!=0,"Switched-off rack leaves its crop bay enabled");
        garden.units[0].enabled=true;garden.supply.oxygen_kg=garden.supply.volume_m3*kOxygenKgPerM3*.85;
        require(app.interior_unit_status(0,garden.units[0]).find("REDUCED GROWTH")!=std::string::npos,"Rack status disagrees with reduced-air crop growth");
        app.placed_habitats_[0]=hall;
        const auto empty_vertices=app.habitat_runtime_mesh(0).interior.size();
        const float floor=app.habitat_models_.mesh(hall).walkable_floor_y;InteriorUnit unit;unit.kind=InteriorUnitKind::PlasticFabricator;unit.position.y=floor;
        std::string reason;bool found=false;
        for(float x=-4;x<=4&&!found;x+=1)for(float z:{-2.f,2.f}){unit.position={x,floor,z};if(app.interior_unit_position_clear(0,unit,reason)){found=true;break;}}
        require(found,"No usable factory unit position: "+reason);
        require(app.place_interior_unit(0,unit,reason),"Cannot install factory unit: "+reason);
        require(app.placed_habitats_[0].units.size()==1&&app.resources_.amount(MaterialResource::Iron)==rich.iron-10,"Unit installation did not consume its cost");
        require(app.habitat_runtime_mesh(0).interior.size()>empty_vertices,"Installed unit has no solid geometry");
        CorridorConnection reserve;reserve.endpoint_a.kind=CorridorEndpointKind::Habitat;reserve.endpoint_a.index=0;
        reserve.endpoint_a.door.wall_position=unit.position+Vec3{0,0,-1.f};reserve.endpoint_a.door.normal={0,0,-1};
        app.corridor_network_.restore_connections({reserve});
        require(app.interior_unit_reserved(0,unit),"Connected doorway does not reserve custom unit space");
        require(!app.interior_unit_position_clear(0,unit,reason)&&reason=="GATE APPROACH RESERVED","New unit can occupy a doorway reserve");
        app.update_interior_units(10);require(!app.placed_habitats_[0].units[0].batch_active,"Reserved unit continues working");
        app.corridor_network_.clear();

        require(!app.interior_unit_position_clear(0,unit,reason),"Unit can overlap another machine");
        auto aisle=unit;aisle.position.x=aisle.position.z=0;require(!app.interior_unit_position_clear(0,aisle,reason),"Unit blocks the center aisle");
        auto outside=unit;outside.position.x=50;require(!app.interior_unit_position_clear(0,outside,reason),"Unit can be placed outside its host");
        app.focus_interior(0);require(std::abs(app.camera_.elevation_degrees()-35.26439f)<.001f&&std::abs(app.camera_.azimuth_degrees()-45.f)<.001f,"Selected room is not isometric");
        require(length(app.camera_.target()-(hall.position+Vec3{0,floor+.8f,0}))<.001f,"Selected room is not centered");
        app.placed_habitats_[0].unit_ore_kg={5,10,15,20};
        const auto silicon=app.resources_.amount(MaterialResource::Silicon),plastic=app.resources_.amount(MaterialResource::Plastic);
        app.update_interior_units(1);
        require(app.placed_habitats_[0].units[0].batch_active&&app.resources_.amount(MaterialResource::Silicon)==silicon-1,"Batch did not consume recipe inputs once");
        const auto progress=app.placed_habitats_[0].units[0].progress;
        app.placed_habitats_[0].power_enabled=false;app.update_interior_units(500);
        require(app.placed_habitats_[0].units[0].progress==progress,"Unpowered unit continues processing");app.placed_habitats_[0].power_enabled=true;
        std::string error;require(app.save_game_to_name("interior-unit-roundtrip",error),error);
        GameApp loaded(host);const auto save=std::filesystem::path(app.save_game_directory())/"interior-unit-roundtrip.gmsave";
        if(!loaded.load_game_from_file(save.string(),error))throw std::runtime_error("Unit save did not load: "+error);
        require(loaded.placed_habitats_[0].units.size()==1&&loaded.placed_habitats_[0].units[0].progress==progress&&loaded.placed_habitats_[0].units[0].batch_active,"Batch progress was lost on load");
        require(loaded.placed_habitats_[0].unit_ore_kg[3]==20,"Raw ore buffers were lost on load");
        app.update_interior_units(240);
        require(app.placed_habitats_[0].units[0].batches==1&&app.resources_.amount(MaterialResource::Plastic)==plastic+2,"Installed plastic unit did not produce its batch");
        require(app.resources_.amount(MaterialResource::Silicon)==silicon-1,"Batch charged input more than once");
        auto metal=unit;metal.kind=InteriorUnitKind::MetalFabricator;app.placed_habitats_[0].units={metal};app.placed_habitats_[0].unit_ore_kg={5,0,0,0};
        const auto iron=app.resources_.amount(MaterialResource::Iron);app.update_interior_units(1);
        require(app.placed_habitats_[0].unit_ore_kg[0]==0&&app.placed_habitats_[0].units[0].batch_active,"Metal unit does not consume actual raw ore");
        app.update_interior_units(120);require(app.resources_.amount(MaterialResource::Iron)==iron+3&&app.placed_habitats_[0].units[0].batches==1,"Raw ore was not converted by the installed fabricator");

        if(!loaded.load_game_from_file((std::filesystem::path(GAME_SOURCE_ROOT)/"game/tests/fixtures/schema48-lander-depot.gmsave").string(),error))throw std::runtime_error("Older compact save rejected: "+error);
        GameApp doors(host);doors.playfield_.rebuild(128);doors.screen_=GameApp::Screen::Playfield;doors.game_speed_=GameApp::GameSpeed::Normal;
        auto cabin=hall;cabin.kind=HabitatKind::CrewQuarters;doors.placed_habitats_={cabin};doors.placed_habitats_[0].supply.power_fraction=0;
        const float deck=doors.habitat_models_.mesh(cabin).walkable_floor_y;
        CorridorConnection c;c.construction.finish();c.endpoint_a.kind=CorridorEndpointKind::Habitat;c.endpoint_a.index=0;
        c.endpoint_a.door.wall_position={0,deck,-2.6f};c.endpoint_a.door.position=c.endpoint_a.door.wall_position;c.endpoint_a.door.normal={0,0,-1};c.endpoint_a.position=c.endpoint_a.door.position;c.endpoint_b.kind=CorridorEndpointKind::OpenDoor;c.endpoint_b.position={0,0,-10};c.path={c.endpoint_a.position,c.endpoint_b.position};
        doors.corridor_network_.restore_connections({c});doors.first_person_mode_=GameApp::FirstPersonMode::Active;doors.first_person_.spawn({0,deck,-.5f},0);
        for(int f=0;f<20;++f)doors.update_pressure_doors(.1f);
        require(doors.pressure_doors_.size()==1&&doors.pressure_doors_[0].open_fraction==0,"Unpowered pressure door auto-opens");
        require(!doors.manual_door_label().empty(),"FPV has no manual control for an unpowered door");
        InputState crank;crank.key_e_down=true;
        for(int f=0;f<32;++f){doors.update_first_person(.1f,crank);doors.update_pressure_doors(.1f);}
        require(doors.pressure_doors_[0].open_fraction>.99f&&!doors.pressure_doors_[0].powered,"Manual door crank does not open without electricity");
        require(doors.manual_door_interaction(true),"Manual close was unavailable");doors.first_person_.spawn({0,deck,0},0);
        for(int f=0;f<60;++f)doors.update_pressure_doors(.1f);
        require(doors.pressure_doors_[0].open_fraction<.001f,"Manual door does not close");
        doors.first_person_.spawn({0,deck,-.5f},0);doors.placed_habitats_[0].supply.power_fraction=1;
        for(int f=0;f<20;++f)doors.update_pressure_doors(.1f);
        require(doors.pressure_doors_[0].powered&&!doors.pressure_doors_[0].manual_override&&doors.pressure_doors_[0].open_fraction<.001f,"Power restoration auto-opens a player hatch");
        InputState tap;tap.key_e_pressed=true;doors.update_first_person(.016f,tap);for(int f=0;f<10;++f)doors.update_pressure_doors(.1f);
        require(doors.pressure_doors_[0].open_fraction>.99f,"Powered E command did not open the hatch");
        PlacedResource exterior;exterior.kind=ResourceKind::OreDriller;exterior.construction.finish();app.placed_resources_={exterior};
        Recorder solid,faded;app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;app.render_resources(solid);app.habitat_view_mode_=GameApp::HabitatViewMode::None;app.render_resources(faded);
        require(solid.mesh_triangles==faded.mesh_triangles,"Exterior structure changes with the interior cutaway option");
        std::cout<<"480-second day, empty interiors, validated units, recipe conservation, schema49/48, isometric focus, manual/powered doors and exterior hull independence passed\n";
    }

    static void runtime_paths(){
        using namespace game::runtime;
        namespace fs=std::filesystem;
        const auto folder=fs::temp_directory_path()/jsd::utf8_path("lunar-paths-ü-月-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
#ifdef _WIN32
        const std::wstring save_before=_wgetenv(L"GAME_SAVE_DIR")?_wgetenv(L"GAME_SAVE_DIR"):L"";
        const std::wstring config_before=_wgetenv(L"GAME_GRAPHICS_CONFIG")?_wgetenv(L"GAME_GRAPHICS_CONFIG"):L"";
        _wputenv_s(L"GAME_SAVE_DIR",L"");
#else
        const std::string save_before=std::getenv("GAME_SAVE_DIR")?std::getenv("GAME_SAVE_DIR"):"";
        const std::string config_before=std::getenv("GAME_GRAPHICS_CONFIG")?std::getenv("GAME_GRAPHICS_CONFIG"):"";
        unsetenv("GAME_SAVE_DIR");
#endif
        GameHost host;GameApp app(host);
        require(fs::weakly_canonical(app.save_game_directory())==fs::weakly_canonical(jsd::utf8_path(GAME_SOURCE_ROOT)/"savegames"),"External-engine workspace saves did not resolve to game root");
        fs::create_directories(folder);
#ifdef _WIN32
        _wputenv_s(L"GAME_SAVE_DIR",folder.c_str());
        const auto config=folder/L"graphics.cfg";_wputenv_s(L"GAME_GRAPHICS_CONFIG",config.c_str());
#else
        setenv("GAME_SAVE_DIR",folder.c_str(),1);
        const auto config=folder/"graphics.cfg";setenv("GAME_GRAPHICS_CONFIG",config.c_str(),1);
#endif
        auto preferences=load_preferences();preferences.first_person_fov=78;
        require(save_preferences(preferences)&&load_preferences().first_person_fov==78,"Unicode-path preferences did not roundtrip");
        require(app.save_game_directory()==folder,"Unicode save directory changed its encoding");
        std::string error;
        app.playfield_.rebuild(64);app.builder_robots_.resize(1);
        require(app.save_game_to_name("path-roundtrip",error),error);
        require(app.save_game_to_name("path-roundtrip",error),"Could not overwrite Unicode-path save: "+error);
        app.refresh_save_games();require(!app.save_games_.empty(),"Unicode-path save did not appear in list");
        GameApp loaded(host);require(loaded.load_game_from_file(app.save_games_.front().path,error),error);
#ifdef _WIN32
        _wputenv_s(L"GAME_SAVE_DIR",save_before.c_str());_wputenv_s(L"GAME_GRAPHICS_CONFIG",config_before.c_str());
#else
        if(save_before.empty())unsetenv("GAME_SAVE_DIR");else setenv("GAME_SAVE_DIR",save_before.c_str(),1);
        if(config_before.empty())unsetenv("GAME_GRAPHICS_CONFIG");else setenv("GAME_GRAPHICS_CONFIG",config_before.c_str(),1);
#endif
        fs::remove_all(folder);
        std::cout<<"Workspace-root save discovery and Unicode save/preferences roundtrip passed\n";
    }

    static void gameplay_options(){
        using namespace game::runtime;
        const auto previous=load_preferences();
        GameHost host;GameApp app(host);Recorder draw;
        InputState slow;slow.frame_seconds=.5f;app.update_simulation(.1f,slow);require(app.fps_==2.f,"FPS counter uses the clamped simulation step");
        app.screen_=GameApp::Screen::Settings;
        app.show_fps_=false;app.render(draw);
        float row=-1,left=0;Rect last{},button{};
        for(const auto& d:draw.draws)if(d.type=='T' && d.text=="FPS COUNTER"){row=d.v[1];left=d.v[0];}
        require(row>0,"FPS option missing in this build configuration");
        for(const auto& d:draw.draws){
            if(d.type=='R')last={d.v[0],d.v[1],d.v[2],d.v[3]};
            if(d.type=='T' && d.text=="OFF" && d.v[0]>left && std::abs(d.v[1]-row)<14)button=last;
        }
        require(button.w>0,"FPS option has no toggle hitbox");
        app.input_.mouse_x=int((button.x+button.w*.5f)*draw.scale);app.input_.mouse_y=int((button.y+button.h*.5f)*draw.scale);app.input_.left_pressed=true;app.render(draw);app.input_={};
        require(app.show_fps_ && load_preferences().show_fps,"FPS toggle did not apply/persist");
        app.render(draw);float auto_row=-1;for(const auto& d:draw.draws)if(d.type=='T'&&d.text=="AUTOSAVE")auto_row=d.v[1];
        Rect auto_plus{};for(const auto& d:draw.draws){if(d.type=='R')last={d.v[0],d.v[1],d.v[2],d.v[3]};if(d.type=='T'&&d.text=="+"&&std::abs(d.v[1]-auto_row)<14)auto_plus=last;}
        require(auto_row>0&&auto_plus.w>0,"Gameplay autosave interval control missing");
        app.input_.mouse_x=int((auto_plus.x+auto_plus.w*.5f)*draw.scale);app.input_.mouse_y=int((auto_plus.y+auto_plus.h*.5f)*draw.scale);app.input_.left_pressed=true;app.render(draw);app.input_={};
        require(app.autosave_minutes_==15&&load_preferences().autosave_minutes==15,"Autosave options control did not apply/persist");
        GameApp restarted(host);restarted.on_start();require(restarted.show_fps_&&restarted.autosave_minutes_==15,"FPS/autosave preference lost at restart");
        app.playfield_.rebuild(64);app.camera_.reset(64);app.screen_=GameApp::Screen::Playfield;app.fps_=60;
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,0,0});app.render(draw);
        unsigned gold=0,outline=0;bool fps=false;
        for(const auto& d:draw.draws){
            if(d.type=='T' && d.text=="FPS 60"){fps=true;require(d.v[0]<30 && d.v[1]>draw.h/draw.scale-layout::BUILD_BAR_H-40 && d.v[1]<draw.h/draw.scale-layout::BUILD_BAR_H,"FPS counter is not lower left");}
            if(d.type=='L' && d.color.r==.18f && d.color.g==1.f && d.color.b==.48f && d.v[5]>=3)++gold;
            if(d.type=='L' && d.color.r==.015f && d.v[5]>=4)++outline;
        }
        require(fps && gold>=4 && outline>=4,"FPS or outlined high-contrast crosshair was not drawn");
        app.show_fps_=false;app.render(draw);for(const auto& d:draw.draws)require(!(d.type=='T' && d.text=="FPS 60"),"Disabled FPS counter still draws");
        require(save_preferences(previous),"Could not restore preferences");
        std::cout<<"Release/Debug gameplay FPS UI, persistence, lower-left HUD and high-contrast crosshair passed\n";
    }

    static void fov_and_station_interiors(){
        using namespace game::runtime;
        const auto previous=load_preferences();
        GameHost host;GameApp app(host);Recorder recorder;
        app.screen_=GameApp::Screen::Settings;app.render(recorder);
        const auto click_fov=[&](const std::string& arrow){
            app.render(recorder);float row_y=-1000;Rect control{},last{};
            for(const auto& d:recorder.draws)if(d.type=='T' && d.text=="FIRST PERSON FOV")row_y=d.v[1];
            require(row_y>0,"First-person FOV option is missing");
            for(const auto& d:recorder.draws){
                if(d.type=='R')last={d.v[0],d.v[1],d.v[2],d.v[3]};
                if(d.type=='T' && d.text==arrow && std::abs(d.v[1]-row_y)<14)control=last;
            }
            require(control.w>0,"FOV arrow has no hitbox");
            app.input_.mouse_x=int((control.x+control.w*.5f)*recorder.scale);
            app.input_.mouse_y=int((control.y+control.h*.5f)*recorder.scale);
            app.input_.left_pressed=true;app.render(recorder);app.input_={};
        };
        click_fov(">");require(app.first_person_.fov()==70 && load_preferences().first_person_fov==70,"FOV did not apply and persist");
        GameApp restarted(host);restarted.on_start();require(restarted.first_person_.fov()==70,"FOV was lost at restart");
        click_fov("<");require(app.first_person_.fov()==68,"FOV decrement did not apply");
        const auto feet=app.first_person_.feet_position();const auto direction=app.first_person_.look_direction();
        app.first_person_.set_fov(50);const auto narrow=app.first_person_.view_projection(16.f/9);
        app.first_person_.set_fov(110);const auto wide=app.first_person_.view_projection(16.f/9);
        require(std::abs(narrow.m[0])>std::abs(wide.m[0]) && length(app.first_person_.feet_position()-feet)==0 && length(app.first_person_.look_direction()-direction)==0,"FOV changed physical camera state or failed to change projection");
        app.first_person_.set_fov(-99);require(app.first_person_.fov()==50,"FOV minimum was not enforced");
        app.first_person_.set_fov(500);require(app.first_person_.fov()==110,"FOV maximum was not enforced");
        for(int kind=0;kind<int(HabitatKind::Count);++kind)for(int tier=0;tier<int(HabitatSizeTier::Count);++tier){
            const auto& mesh=app.habitat_models_.mesh(HabitatKind(kind),HabitatSizeTier(tier));
            require(!mesh.lining.empty() && !mesh.deck.empty() && (!mesh.service_metal.empty()||!mesh.overhead_services.empty()) && (!mesh.lamps.empty()||!mesh.overhead_lamps.empty()) && (kind==int(HabitatKind::AirLock)||!mesh.displays.empty()),"A room variant has missing station detail layers: "+std::to_string(kind)+"/"+std::to_string(tier));
            for(const auto* layer:{&mesh.lining,&mesh.deck,&mesh.service_metal,&mesh.lamps,&mesh.displays,&mesh.overhead_services,&mesh.overhead_lamps}){
                require(layer->size()%3==0,"Station detail layer is not a triangle list");
                for(const auto& p:*layer)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"Station detail layer has invalid geometry");
            }
            if(HabitatKind(kind)==HabitatKind::CrewQuarters)require(!mesh.fabric.empty(),"Quarters have no fabric bedding");
        }
        require(save_preferences(previous),"Could not restore test preferences");
        std::cout<<"FOV controls, persistence, projection and 40 station interior variants passed\n";
    }

    static void graphics_quality(){
        playfield_surface();
        hull_views();
        using namespace game::runtime;
        struct BenchmarkRecorder : Recorder {
            bool benchmarking{};
            GraphicsCapabilities graphics_capabilities() override {
                GraphicsCapabilities caps{};
                caps.enhanced=true;caps.external_jsd=true;
                caps.vertical_sync=true;
                caps.texture_anisotropy=8;
                caps.ray_lighting=caps.visibility_buffer=caps.gpu_terrain=caps.pages=true;
                return caps;
            }
            bool supports_graphics_benchmark() const override { return true; }
            void set_graphics_benchmark(bool enabled) override { benchmarking=enabled; }
            double benchmark_frame_ms() const override {
                if(!benchmarking) return 0;
                if(graphics.path==GraphicsPath::Classic) return 6;
                return graphics.render_scale==100?24:graphics.render_scale==85?13:9;
            }
            bool begin_lunar_scene(const LunarScene&) override { return graphics.path!=GraphicsPath::Classic; }
            bool draw_campus_atmosphere(const Mat4&,Vec3,float) override { return graphics.path!=GraphicsPath::Classic; }
        };
        const auto saved=load_preferences();
        {
            std::ofstream legacy(std::getenv("GAME_GRAPHICS_CONFIG"));
            legacy<<"terrain_relief=1\nmaterial_detail=0\n";
        }
        using F=GraphicsFeature;
        const auto migrated=load_preferences().graphics;
        require(migrated.quality(F::TerrainRelief)==3 && migrated.quality(F::MaterialDetail)==0,
                "legacy on and off settings changed meaning");
        GameHost engine;
        GameApp app(engine);
        BenchmarkRecorder r;
        app.screen_=GameApp::Screen::GraphicsSettings;
        const auto press=[&](Rect box){
            app.input_.mouse_x=int((box.x+box.w*0.5f)*r.scale);
            app.input_.mouse_y=int((box.y+box.h*0.5f)*r.scale);
            app.input_.left_pressed=true;
            app.render(r);
            app.input_={};
        };
        const auto click=[&](const char* text){
            app.render(r);
            press(r.button(text));
        };
        const auto row=[&](const char* name){
            app.render(r);
            const auto title=std::find_if(r.draws.begin(),r.draws.end(),[&](const Draw& draw){
                return draw.type=='T' && draw.text==name;
            });
            require(title!=r.draws.end(),std::string("graphics row missing: ")+name);
            const auto button=std::find_if(r.draws.begin(),r.draws.end(),[&](const Draw& draw){
                return draw.type=='R' && draw.v[3]==31 && draw.v[0]>title->v[0] &&
                    std::abs(draw.v[1]-title->v[1])<16.f;
            });
            require(button!=r.draws.end(),std::string("graphics row has no selector: ")+name);
            press({button->v[0],button->v[1],button->v[2],button->v[3]});
        };
        const auto choose=[&](const char* name,const char* option){row(name);click(option);};
        click("BALANCED");
        require(app.graphics_.render_scale==85 && app.graphics_.particle_density==2,
                "quality preset did not apply the expected scale and particles");
        choose("VERTICAL SYNC","ON");
        choose("DISPLAY MODE","WINDOWED");
        require(app.graphics_.vertical_sync,"display mode changed vertical sync");
        click("ENVIRONMENT");
        choose("TERRAIN RELIEF","LOW");
        require(app.graphics_.quality(F::TerrainRelief)==1 && app.graphics_.quality(F::MaterialDetail)==2,
                "terrain and material quality settings interfered");
        click("TEXTURES");
        choose("MATERIAL DETAIL","BASIC");
        require(app.graphics_.quality(F::MaterialDetail)==0,"Basic material detail did not select the lowest setting");
        choose("MATERIAL DETAIL","MEDIUM");
#ifndef _WIN32
        click("ENVIRONMENT");
        choose("GROUND CLUTTER","OFF");
        require(app.graphics_.ground_clutter==0 && app.graphics_.quality(F::TerrainRelief)==1,
                "ground clutter changed terrain relief");
        click("TEXTURES");
        choose("SURFACE TEXTURE RESOLUTION","128 PX");
        row("TEXTURE FILTERING");
        r.check_bounds("texture filter dropdown");
        require(std::none_of(r.draws.begin(),r.draws.end(),[](const Draw& draw){return draw.text=="16X ANISOTROPIC";}),
                "texture filtering advertised a level above the GPU limit");
        click("8X ANISOTROPIC");
        click("LIGHTING");
        choose("SHADOW FILTER QUALITY","LOW");
        require(app.graphics_.shadow_quality==1,
                "external engine shadow quality did not apply");
        require(app.graphics_.surface_resolution==1 && app.graphics_.texture_filter==4,
                "surface resolution and filtering did not save independently");
#endif
        click("EFFECTS");
        row("PARTICLE DENSITY");
        click("PERFORMANCE");
        require(app.graphics_.render_scale==85 && app.graphics_dropdown_==-1,
                "dismissing a dropdown clicked through to another control");
        choose("PARTICLE DENSITY","OFF");
        require(app.graphics_.particle_density==0 && app.graphics_.bloom==75 &&
                load_preferences().graphics==app.graphics_,"individual graphics quality did not persist");
        app.ui_scale_index_=4;
        r.resize(1280,720);
        for(const auto* tab:{"DISPLAY","TEXTURES","ENVIRONMENT","LIGHTING","EFFECTS","ADVANCED","BENCHMARK"}){
            click(tab);
            app.render(r);
            r.check_bounds(tab);
        }
        app.ui_scale_index_=0;
        r.resize(1920,1080);

        const auto custom=app.graphics_;
        click("BENCHMARK");
        click("RUN BENCHMARK");
        require(app.graphics_benchmark_.running(),"benchmark did not start");
        for(int frame=0;frame<1000 && app.graphics_benchmark_.running();++frame) app.render(r);
        require(app.graphics_benchmark_.status()==BenchmarkStatus::Complete,"benchmark did not finish");
        require(app.graphics_==custom && load_preferences().graphics==custom,"benchmark changed settings before Apply");
        const auto* recommended=app.graphics_benchmark_.recommendation();
        require(recommended && recommended->profile==BenchmarkProfile::Balanced && recommended->worst_p90()==13,
                "benchmark did not choose highest quality within the frame budget");
        click("60 FPS TARGET");
        require(app.graphics_benchmark_.recommendation()->profile==BenchmarkProfile::Performance,
                "changing FPS target did not reuse the completed measurements");
        click("90 FPS TARGET");
        click("120 FPS TARGET");
        click("30 FPS TARGET");
        click("APPLY RECOMMENDATION");
        require(app.graphics_.render_scale==85 && app.graphics_.quality(F::MaterialDetail)==2 &&
                app.graphics_.vertical_sync && load_preferences().graphics==app.graphics_,
                "benchmark recommendation did not persist or overwrote vertical sync");

        click("RUN BENCHMARK");
        InputState escape{};
        escape.escape_pressed=true;
        app.update_simulation(0.01f,escape);
        app.render(r);
        require(app.graphics_benchmark_.status()==BenchmarkStatus::Cancelled && !r.benchmarking &&
                app.screen_==GameApp::Screen::GraphicsSettings,"cancelling benchmark left the graphics menu or timing active");
        click("RUN BENCHMARK");
        r.resize(1280,720);
        app.render(r);
        require(app.graphics_benchmark_.status()==BenchmarkStatus::Resized && !r.benchmarking,
                "benchmark mixed measurements across display sizes");
        r.check_bounds("benchmark display resize");
        require(save_preferences(saved),"could not restore isolated test preferences");
        std::cout<<"Graphics groups, dropdowns, persisted quality, VSync, and benchmark workflows passed\n";
    }

    static void build_mode(){
        using namespace game::runtime;
        GameHost engine;
        GameApp app(engine);
        require(app.debug_mode_active()==build::debug_features,"wrong default debug policy");
        auto preferences=load_preferences();
        preferences.mesh_layout=true;
        preferences.graphics.path=GraphicsPath::Deferred;
        require(save_preferences(preferences),"could not write isolated test preferences");
        app.on_start();
        Recorder r;
        app.screen_=GameApp::Screen::Settings;
        app.render(r);
        const auto has=[&](std::string_view text){
            return std::any_of(r.draws.begin(),r.draws.end(),[&](const Draw& d){
                return d.type=='T' && d.text.find(text)!=std::string::npos;
            });
        };
        require(has("GRAPHICS") && has("MUSIC") && has("SOUND VOLUME"),"player settings missing");
        require(has("DEBUG FRAME")==build::debug_features && has("DEBUG LAYOUT")==build::debug_features,
                "settings ignored the build policy");
        require(r.graphics.path==(build::debug_features?GraphicsPath::Classic:GraphicsPath::Deferred),
                "debug layout preference affected the wrong build");

        app.screen_=GameApp::Screen::NewGame;
        app.new_game_mode_selection_=GameApp::NewGameMode::FreePlay;
        app.render(r);
        require(has("1  PLANNING") && has("2  ROBOT PREPARATION"),"playable stages missing");
        require(has("3  DEBUG MODES")==build::debug_features && has("DEBUG START TEMPLATE")==build::debug_features,
                "new-game wizard ignored the build policy");

        app.new_game_phase_selection_=GameApp::GamePhase::Debug;
        app.start_new_game(GameApp::Stage3Mode::AstronautTest);
        require(app.astronaut_test_active()==build::debug_features,"incorrect Stage 3 access");
        app.active_game_phase_=GameApp::GamePhase::Debug;
        app.stage3_mode_=GameApp::Stage3Mode::Sandbox;
        app.debug_frame_enabled_=true;
        PlacedHabitat blueprint;
        app.finalize_placement(blueprint);
        require(blueprint.construction.complete()==build::debug_features,"instant construction escaped Debug");

        app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        app.stage3_mode_=GameApp::Stage3Mode::Sandbox;
        app.playfield_.rebuild(128);
        app.builder_robots_.resize(1);
        require(app.debug_robot_move_available(0)==build::debug_features,"robot command escaped Debug");

        const auto suffix=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        std::string error;
        for(const auto phase:{GameApp::GamePhase::RobotPreparation,GameApp::GamePhase::Debug}){
            app.active_game_phase_=phase;
            const std::string name="build-mode-"+suffix+"-"+std::to_string(static_cast<int>(phase));
            require(app.save_game_to_name(name,error),error);
            app.refresh_save_games();
            const auto item=std::find_if(app.save_games_.begin(),app.save_games_.end(),[&](const auto& save){return save.name==name;});
            require(item!=app.save_games_.end(),"could not find build-policy save");
            GameApp loaded(engine);
            const bool ok=loaded.load_game_from_file(item->path,error);
            std::filesystem::remove(item->path);
            require(ok==(phase!=GameApp::GamePhase::Debug || build::debug_features),"wrong save access for build mode");
            require(ok || error.find("Debug build")!=std::string::npos,"missing diagnostic for a Stage 3 save");
            if(ok) require(loaded.debug_mode_active()==build::debug_features,"save altered build policy");
        }
        std::cout<<(build::debug_features?"Debug":"Release")<<" menus, build actions, and save access passed\n";
    }

    static void build_order_panel(){
        game::runtime::GameHost engine;
        for(int zoom:{0,4}){
            GameApp app(engine);
            setup(app,7,zoom);
            app.debug_frame_enabled_=false;
            app.placed_habitats_.clear();
            app.placed_resources_.clear();
            app.builder_robots_.clear();
            for(int i=0;i<12;++i){
                PlacedResource panel;
                panel.kind=ResourceKind::SolarPanel;
                panel.placement_sequence=i+1;
                panel.instance_serial=i+1;
                panel.position={float(i-6)*4,0,18};
                app.placed_resources_.push_back(panel);
            }
            const ConstructionTargetRef promoted{ConstructionTargetKind::Resource,1};
            app.toggle_construction_priority(promoted);
            Recorder recorder;
            recorder.resize(1280,720);
            app.render(recorder);
            const auto click=[&](float x,float y){
                game::runtime::InputState input;
                input.mouse_x=int(std::lround(x*recorder.scale));
                input.mouse_y=int(std::lround(y*recorder.scale));
                input.left_pressed=true;
                app.update_simulation(0.f,input);
                require(!app.selection_marquee_active_,"queue click reached the world");
                app.render(recorder);
            };
            const auto header=recorder.button("+  BUILD ORDER  [12]");
            click(header.x+header.w/2,header.y+header.h/2);
            require(app.build_queue_expanded_,"queue header did not expand");
            app.input_={};
            app.render(recorder);
            recorder.check_bounds("expanded build order");
            recorder.button("NEXT");
            const auto expanded=app.build_queue_panel_rect();
            const float right=expanded.x+expanded.w-8;
            const float first_row_y=expanded.y+8;
            require(expanded.y+expanded.h<=720/recorder.scale-layout::BUILD_BAR_H &&
                    app.progress_tree_panel_rect().y+app.progress_tree_panel_rect().h<expanded.y,
                    "bottom build order overlaps the toolbar or progress panel");
            const auto hover=[&](float x,float y,float scroll=0.f){
                game::runtime::InputState input;
                input.mouse_x=int(std::lround(x*recorder.scale));
                input.mouse_y=int(std::lround(y*recorder.scale));
                input.wheel=scroll;
                app.update_simulation(0.f,input);
                app.render(recorder);
            };
            const auto outline_center_x=[&]{
                const auto& lines=recorder.queue_hover_lines;
                require(lines.size()>=8,"hovered queue row has no playfield outline");
                return (lines[0].x+lines[1].x)*.5f;
            };
            hover(right-90,first_row_y+17);
            require(outline_center_x()==app.placed_resources_[1].position.x &&
                    app.selection_kind_==GameApp::SelectionKind::None,
                    "queue hover did not outline the correct blueprint independently of selection");
            click(right-16,first_row_y+17);
            hover(right-90,first_row_y+17);
            require(outline_center_x()==app.placed_resources_[0].position.x,
                    "hover outline did not track a reordered row");
            require(app.construction_queue().front().sequence==1 &&
                    app.construction_priority_rank(promoted)==kNoRobotIndex,
                    "down arrow failed to demote the priority row");
            click(right-48,first_row_y+38+17);
            require(app.construction_queue().front().sequence==2 &&
                    app.construction_priority_rank(promoted)==kNoRobotIndex,
                    "up arrow failed to move a normal row");
            const auto distance=app.camera_.distance();
            hover(right-90,first_row_y+17,-1.f);
            require(app.build_queue_scroll_==1 && app.camera_.distance()==distance,
                    "scrolling the queue zoomed the playfield or left the rows unchanged");
            const auto visible=app.construction_queue()[1].target.index;
            require(outline_center_x()==app.placed_resources_[visible].position.x,
                    "queue scroll did not update the playfield hover in the same frame");
            hover(header.x+header.w/2,header.y+header.h/2);
            require(recorder.queue_hover_lines.empty(),"queue header left a playfield outline");
            app.input_={};
            app.selection_kind_=GameApp::SelectionKind::Resource;
            app.selected_resource_=11;
            app.render(recorder);
            require(app.build_queue_scroll_>1,"selecting a late blueprint did not reveal its queue row");
            const auto has_highlight=[&]{
                return std::any_of(recorder.draws.begin(),recorder.draws.end(),[](const Draw& draw){
                    return draw.type=='R' && draw.v[2]==3.f && draw.v[3]==35.f;
                });
            };
            require(has_highlight(),"selected blueprint did not highlight its queue row");
            app.clear_world_selection();
            app.render(recorder);
            require(!has_highlight(),"deselection left the queue highlight active");
            click(header.x+header.w/2,header.y+header.h/2);
            require(!app.build_queue_expanded_,"queue header did not collapse");
            hover(right-90,first_row_y+17);
            require(recorder.queue_hover_lines.empty(),"collapsed queue retained a playfield outline");
            app.progress_tree_expanded_=true;
            app.build_queue_expanded_=true;
            app.input_={};app.render(recorder);
            const auto progress=app.progress_tree_panel_rect(),both=app.build_queue_panel_rect();
            require(progress.y+progress.h<both.y && both.h>30.f,"expanded progress and queue cannot coexist");
            recorder.check_bounds("expanded colony progress and bottom build order");
            if constexpr(build::debug_features){
                app.debug_frame_enabled_=true;
                app.build_queue_expanded_=true;
                app.input_={};
                app.render(recorder);
                recorder.button("FULL GRAPHICS");
                const auto queue=app.build_queue_panel_rect(),debug=app.debug_frame_rect();
                require(debug.y+debug.h<queue.y,"expanded bottom queue overlaps debug frame");
                require(queue.h>30.f,"debug frame left no room for the expanded queue");
                recorder.check_bounds("combined queue and debug frame");
            }
        }
        std::cout<<"Build order mouse controls, priority demotion, playfield hover and scroll isolation passed\n";
    }

    static void activity_log_and_blueprint_progress(){
        game::runtime::GameHost engine;
        GameApp app(engine);
        setup(app,7,4);
        app.game_session_active_=true;
        app.reset_activity_log("Log regression");
        app.placed_resources_[0].placement_sequence=2;
        auto& rover=app.builder_robots_[0];
        rover.mode=BuilderRobotMode::Delivering;
        rover.safety.mode=robots::SafetyMode::Fault;
        rover.safety.block_reason=robots::BlockReason::NoRoute;
        app.ui_time_seconds_=1.25;
        app.update_activity_log();
        const auto first_count=app.activity_log_.entries().size();
        app.update_activity_log();
        require(app.activity_log_.entries().size()==first_count,"steady activities spam the log");
        rover.mode=BuilderRobotMode::Idle;
        rover.safety.mode=robots::SafetyMode::Idle;
        rover.safety.block_reason=robots::BlockReason::None;
        app.placed_resources_[0].supply.warnings=0;
        app.ui_time_seconds_=2.75;
        app.update_activity_log();
        const auto has=[&](LogEvent event,double time,const std::string& text){
            const auto& events=app.activity_log_.entries();
            return std::any_of(events.begin(),events.end(),[&](const auto& entry){
                return entry.event==event && entry.seconds==time && entry.message.find(text)!=std::string::npos;
            });
        };
        require(has(LogEvent::Started,1.25,"DELIVERING") && has(LogEvent::Stopped,2.75,"DELIVERING"),"rover start and stop missing");
        require(has(LogEvent::Warning,1.25,"NO SAFE ROUTE") && has(LogEvent::Resolved,2.75,"NO SAFE ROUTE"),"rover fault or resolution missing");
        require(has(LogEvent::Warning,1.25,"AIR BELOW 95%") && has(LogEvent::Resolved,2.75,"AIR BELOW 95%"),"structure warning or resolution missing");

        Recorder recorder;
        recorder.resize(1280,720);
        app.render(recorder);
        const auto click=[&](game::runtime::Rect button){
            game::runtime::InputState input;
            input.mouse_x=(int)std::lround((button.x+button.w*.5f)*recorder.scale);
            input.mouse_y=(int)std::lround((button.y+button.h*.5f)*recorder.scale);
            input.left_pressed=true;
            app.update_simulation(0.f,input);
            require(!app.selection_marquee_active_,"activity log click reached the world");
            app.render(recorder);
        };
        const auto closed_button=recorder.button("LOG +");
        click(closed_button);
        require(app.activity_log_expanded_,"activity log did not open");
        app.input_={};
        app.render(recorder);
        const auto open_button=recorder.button("LOG -");
        require(open_button.x==closed_button.x && open_button.y==closed_button.y,
                "opening the log moved its toolbar button");
        const auto dock=app.activity_log_panel_rect();
        require(dock.y+dock.h<recorder.h/recorder.scale-layout::BUILD_BAR_H,
                "activity log overlaps the build toolbar");
        click(recorder.button("OLDER"));
        require(app.activity_log_offset_>0,"older activity entries are inaccessible");
        app.input_={};
        app.render(recorder);
        const auto distance=app.camera_.distance();
        game::runtime::InputState wheel;
        const auto panel=app.activity_log_panel_rect();
        wheel.mouse_x=(int)std::lround((panel.x+30)*recorder.scale);
        wheel.mouse_y=(int)std::lround((panel.y+35)*recorder.scale);
        wheel.wheel=-1;
        const auto prior_offset=app.activity_log_offset_;
        app.update_simulation(0.f,wheel);
        app.render(recorder);
        require(app.activity_log_offset_+1==prior_offset && app.camera_.distance()==distance,
                "log wheel did not stay within the panel");
        click(recorder.button("EXPORT LOG"));
        require(app.activity_log_export_status_.starts_with("EXPORTED "),"log export failed");
        const auto path=std::filesystem::path(app.save_game_directory())/app.activity_log_export_status_.substr(9);
        std::ifstream log_file(path);
        std::string contents((std::istreambuf_iterator<char>(log_file)),{});
        log_file.close();
        std::filesystem::remove(path);
        require(contents.find("00:00:00.000  EVENT  Log regression")!=std::string::npos &&
                contents.find("00:00:01.250  START")!=std::string::npos &&
                contents.find("00:00:02.750  STOP")!=std::string::npos &&
                contents.find("00:00:02.750  RESOLVED")!=std::string::npos,"export omitted history or timestamps");
        recorder.check_bounds("expanded activity log");

        auto& blueprint=app.placed_habitats_[0];
        blueprint.placement_sequence=1;
        blueprint.construction.step=2;
        blueprint.resource_plan.total_steps=4;
        blueprint.resource_plan.steps[0].iron=20;
        blueprint.resource_plan.steps[1].iron=20;
        blueprint.resource_plan.steps[2].iron=30;
        blueprint.resource_plan.steps[3].iron=30;
        blueprint.site_buffer.iron=10;
        rover.mode=BuilderRobotMode::Delivering;
        rover.target={ConstructionTargetKind::Habitat,0};
        rover.cargo_committed_to_build=true;
        rover.logistics_reservation.iron=10;
        app.input_={};
        app.selection_kind_=GameApp::SelectionKind::Habitat;
        app.selected_habitat_=0;
        app.render(recorder);
        const auto label=[&](const std::string& value){
            return std::any_of(recorder.draws.begin(),recorder.draws.end(),[&](const Draw& draw){return draw.text==value;});
        };
        require(label("MATERIALS AT SITE  50%") && label("CONSTRUCTION  "+std::to_string((int)std::lround(200.f/kConstructionSteps))+"%"),
                "blueprint bars mixed materials in transit with on-site or assembly progress");
        require(app.selection_panel_rect().y+app.selection_panel_rect().h<app.activity_log_panel_rect().y,
                "activity log and selection inspector overlap");
        app.clear_world_selection();
        app.render(recorder);
        const auto bar=[&](float red,float width){
            return std::any_of(recorder.draws.begin(),recorder.draws.end(),[&](const Draw& draw){
                return draw.type=='R' && draw.color.r==red && draw.v[3]==7.f && std::abs(draw.v[2]-width)<.001f;
            });
        };
        require(bar(.21f,63.f) && bar(1.f,126.f*2.f/kConstructionSteps),"unselected blueprint is missing one of its progress bars");
        click(recorder.button("LOG -"));
        require(!app.activity_log_expanded_ && app.activity_log_panel_rect().h==0.f,
                "collapsed log still occupies playfield space");
        std::cout<<"Activity starts/stops, warning/resolution export, overlay input, and independent blueprint bars passed\n";
    }

    static void save_editor(){
        using namespace runtime;using text_edit::Editor;
        Editor edit;std::string value="Moon Base";edit.reset(value);InputState in;in.edit_home=true;edit.update(value,in,{});
        in={};in.text_input="New ";edit.update(value,in,{});require(value=="New Moon Base"&&edit.caret==4,"Text is still append-only");
        in={};in.edit_right=true;in.key_shift_down=true;for(int i=0;i<4;++i)edit.update(value,in,{});
        in={};in.edit_copy=true;const auto copied=edit.update(value,in,{});require(copied&&*copied=="Moon","Selection or clipboard copy failed");
        in={};in.text_input="Lunar";edit.update(value,in,{});require(value=="New Lunar Base","Selection replacement failed");
        in={};in.edit_undo=true;edit.update(value,in,{});require(value=="New Moon Base","Undo failed");in={};in.edit_redo=true;edit.update(value,in,{});require(value=="New Lunar Base","Redo failed");
        in={};in.edit_select_all=true;edit.update(value,in,{});in={};in.edit_paste=true;edit.update(value,in,"\xC3\xBC" "ber\nbase");require(value=="\xC3\xBC" "berbase","UTF-8 paste or line filtering failed");
        edit.move(2,false);in={};in.backspace_pressed=true;edit.update(value,in,{});require(value=="berbase","Backspace split a UTF-8 character");
        value="abcdef";edit.reset(value);edit.move(2,false);in={};in.delete_pressed=true;edit.update(value,in,{});require(value=="abdef","Forward delete failed");
        value=std::string(63,'a');edit.reset(value);in={};in.edit_paste=true;edit.update(value,in,"\xC3\xBC");require(value.size()==63,"Paste split a character at the 64-byte name limit");
        in={};in.text_input="x";edit.update(value,in,{});in.text_input="y";edit.update(value,in,{});require(value.size()==64,"Name limit failed");
        GameHost host;host.set_clipboard_text("test clipboard");require(host.clipboard_text()=="test clipboard","Headless clipboard contract failed");
        GameApp app(host);app.game_session_active_=true;app.screen_=GameApp::Screen::SaveGame;app.builder_robots_.resize(1);app.save_game_name_="EditableName";
        for(const auto size:std::array<Vec2,4>{{{1280,720},{1920,1080},{3440,1440},{3840,2160}}})for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom){
            app.ui_scale_index_=zoom;Recorder r;r.resize(int(size.x),int(size.y));app.render(r);r.check_bounds("save editor/list layout");
            r.button("SAVE");r.button("BACK");require(std::any_of(r.draws.begin(),r.draws.end(),[](const auto& d){return d.type=='T'&&d.text.find("EXISTING SAVES")!=std::string::npos;}),"Save dialog has no side list");
        }
        app.ui_scale_index_=0;Recorder r;r.resize(1920,1080);app.save_games_={{}};app.save_games_[0].name="Existing colony";app.save_games_[0].version="0.75.0";app.save_name_focus_=false;app.save_list_selected_=0;
        app.input_={};app.input_.enter_pressed=true;app.render(r);require(app.save_name_focus_&&app.save_status_message_.empty(),"Selecting a list row with Enter wrote a save unexpectedly");
        std::cout<<"Insert/caret/selection/copy-paste/undo-redo/UTF-8/name limits and 20 save-editor/sidebar scaling cases passed\n";
    }
    static void removal_controls(){
        using namespace runtime;namespace fs=std::filesystem;
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);app.builder_robots_.resize(1);
        app.active_game_phase_=GameApp::GamePhase::RobotPreparation;app.game_session_active_=true;
        app.screen_=GameApp::Screen::Playfield;app.game_speed_=GameApp::GameSpeed::Pause;
        for(int i=1;i<=3;++i){InputState in;in.key_number_pressed=i;app.update_simulation(0,in);require(int(app.game_speed_)==i-1,"Number key did not set speed");}
        InputState text;text.text_input="1";app.update_simulation(0,text);require(app.game_speed_==GameApp::GameSpeed::Normal,"Text-key fallback did not set speed");
        app.game_speed_=GameApp::GameSpeed::Pause;
        PlacedResource item;item.kind=ResourceKind::SolarPanel;item.position={20,0,20};item.placement_sequence=101;item.construction.finish();app.placed_resources_={item};
        app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;
        InputState del;del.delete_pressed=true;app.update_simulation(0,del);require(app.placed_resources_.size()==1&&app.deconstruction_jobs_.size()==1,"DEL did not queue deconstruction");
        app.deconstruction_jobs_.front().progress=.42f;
        const auto name="deconstruct-regression-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        std::string error;require(app.save_game_to_name(name,error),error);
        const auto found=std::find_if(app.save_games_.begin(),app.save_games_.end(),[&](const auto& e){return e.name==name;});require(found!=app.save_games_.end(),"Test save missing");
        const auto path=found->path;struct Cleanup{fs::path p;~Cleanup(){std::error_code e;fs::remove(p,e);}} cleanup{fs::u8path(path)};
        GameApp loaded(host);require(loaded.load_game_from_file(path,error),error);
        require(loaded.deconstruction_jobs_.size()==1&&loaded.deconstruction_jobs_.front().sequence==101&&std::abs(loaded.deconstruction_jobs_.front().progress-.42f)<.001f,"Save lost deconstruction progress");
        // Use the actual save-manager button hitboxes, not a direct unlink.
        app.screen_=GameApp::Screen::SaveGame;app.ui_scale_index_=0;app.save_list_scroll_=0;app.save_list_selected_=-1;
        const auto entry=*found;app.save_games_={entry};Recorder r;r.resize(1920,1080);
        const auto click=[&](const std::string& label){app.render(r);auto b=r.button(label);app.input_.mouse_x=int((b.x+b.w*.5f)*r.scale);app.input_.mouse_y=int((b.y+b.h*.5f)*r.scale);app.input_.left_pressed=true;app.render(r);app.input_={};};
        click("DELETE");require(fs::exists(cleanup.p),"Delete did not wait for confirmation");
        click("CONFIRM");require(!fs::exists(cleanup.p),"Confirmed save deletion failed");
        std::cout<<"Number shortcuts, DEL queue, in-progress save/load and confirmed save manager deletion passed\n";
    }
    static void airlock_transfers(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard})for(float rotation:{0.f,37.f,90.f}){
            PlacedHabitat lock;lock.kind=HabitatKind::AirLock;lock.size=tier;lock.rotation_degrees=rotation;lock.construction.finish();app.placed_habitats_={lock};
            const auto& m=app.habitat_models_.mesh(lock);const float scale=m.footprint_x/10.f,floor=m.walkable_floor_y;
            for(float sign:{-1.f,1.f})for(const auto* layer:{&m.shell,&m.lining,&m.interior,&m.details,&m.service_metal})for(std::size_t t=0;t+2<layer->size();t+=3){
                std::vector<Vec3> polygon{(*layer)[t],(*layer)[t+1],(*layer)[t+2]};
                const float x=sign*3.5f*scale;
                const std::array<std::pair<Vec3,float>,6> planes{{{{1,0,0},x+.4f*scale},{{-1,0,0},-x+.4f*scale},{{0,0,1},.5f},{{0,0,-1},.5f},{{0,1,0},floor+1.65f},{{0,-1,0},-floor-.06f}}};
                for(const auto& [n,d]:planes)polygon=mesh_portals::clip(polygon,n,d,false);
                float area=0;for(std::size_t k=1;k+1<polygon.size();++k)area+=length(cross(polygon[k]-polygon[0],polygon[k+1]-polygon[0]));
                require(area<1.e-5f,"An overlapping airlock cap/liner/frame still blocks the transfer cabin");
            }
            app.pressure_doors_.clear();for(const auto& d:app.habitat_doors(0)){PressureDoorState state;state.door=d;state.open_fraction=1;app.pressure_doors_.push_back(state);}
            const auto world=[&](float x){return world_math::rotate_y({x,floor,0},rotation);};
            for(float x=-4.1f*scale;x<4.1f*scale;x+=.05f)require(app.first_person_move_allowed(world(x),world(x+.05f))&&app.first_person_move_allowed(world(x+.05f),world(x)),"Open airlock aisle is not reversible tier="+std::to_string(int(tier))+" rot="+std::to_string(rotation)+" x="+std::to_string(x));
            for(auto& d:app.pressure_doors_)d.open_fraction=0;
            const float partition=airlock::partition_x(m);require(!app.first_person_move_allowed(world(partition-.4f),world(partition+.4f)),"Removing duplicate caps bypassed the real pressure seal");
        }
        std::cout<<"Both sizes / three rotations: visible and physical transfer clearance with functioning pressure seals passed\n";
    }
    static void hatch_interaction(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);app.screen_=GameApp::Screen::Playfield;app.game_speed_=GameApp::GameSpeed::Normal;
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({0,0,2.0f},0);
        PressureDoorState d;d.door.position=d.door.wall_position={0,0,0};d.door.normal={0,0,1};d.powered=true;app.pressure_doors_={d};
        const auto tick=[&](int frames,bool held){InputState in;in.key_e_down=held;for(int i=0;i<frames;++i)app.update_door_interaction(.1f,in);};
        require(app.manual_door_label().find("PRESS E")!=std::string::npos,"Powered hatch has no E prompt");
        InputState tap;tap.key_e_pressed=true;app.update_door_interaction(.016f,tap);require(app.pressure_doors_[0].player_request_seconds>0,"Powered E did not create an explicit request");
        app.pressure_doors_[0].powered=false;app.pressure_doors_[0].player_request_seconds=0;tick(10,true);const auto partial=app.pressure_doors_[0].open_fraction;
        require(partial>.30f&&partial<.36f,"Manual hold is not a three-second physical crank");tick(10,false);require(app.pressure_doors_[0].open_fraction==partial,"Released crank keeps opening");
        tick(22,true);require(app.pressure_doors_[0].open_fraction>.999f&&app.door_hold_.complete,"Resumed crank does not complete");tick(20,true);require(app.pressure_doors_[0].open_fraction>.999f,"Held E repeatedly toggles a completed hatch");
        tick(1,false);tick(10,true);require(app.pressure_doors_[0].open_fraction<.70f&&app.pressure_doors_[0].open_fraction>.63f,"Fresh hold does not crank closed");
        app.first_person_.spawn({0,0,6},0);tick(10,true);const auto stopped=app.pressure_doors_[0].open_fraction;require(!app.door_hold_.target,"Out-of-reach hold did not cancel");tick(10,true);require(app.pressure_doors_[0].open_fraction==stopped,"Crank continues outside reach");
        app.first_person_.spawn({0,0,2},0);app.game_speed_=GameApp::GameSpeed::Pause;tick(10,true);require(app.pressure_doors_[0].open_fraction==stopped,"Paused game advances the crank");app.game_speed_=GameApp::GameSpeed::Normal;
        app.pressure_doors_[0].open_fraction=.8f;app.door_hold_={};app.first_person_.spawn({0,0,.2f},0);app.first_person_.restore_view(0,-65);tick(10,true);require(app.pressure_doors_[0].open_fraction==.8f,"Manual close crushes the controlled body");
        app.first_person_.spawn({0,0,2},0);app.door_hold_={};app.pressure_doors_[0].open_fraction=0;tick(15,true);
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}}){
            Recorder r;r.resize(int(size.x),int(size.y));app.ui_scale_index_=zoom;app.on_resize(r.w,r.h);r.set_2d(app.ui_scale_);app.render_door_interaction(r);r.check_bounds("hatch crank ring");
            require(std::count_if(r.draws.begin(),r.draws.end(),[](const auto& x){return x.type=='L'&&x.color.r>.9f;})>=23,"Half crank lacks an opening progress ring");
        }
        // Both sides of a real airlock stay sealed until explicitly requested.
        PlacedHabitat lock;lock.kind=HabitatKind::AirLock;lock.construction.finish();lock.supply.power_fraction=1;app.placed_habitats_={lock};app.pressure_doors_.clear();
        const auto floor=app.habitat_models_.mesh(lock).walkable_floor_y;app.first_person_.spawn({-2.7f,floor,0},90);app.input_.key_w_down=true;
        for(int i=0;i<30;++i)app.update_pressure_doors(.1f);
        require(std::all_of(app.pressure_doors_.begin(),app.pressure_doors_.end(),[](const auto& s){return s.open_fraction==0;}),"Approaching an airlock automatically opens it");
        app.update_door_interaction(.016f,tap);
        for(int i=0;i<45;++i){app.update_pressure_doors(.1f);int open=0;for(const auto& s:app.pressure_doors_)open+=!pressure_door::sealed(s.open_fraction);require(open<=1,"Powered E bypasses the airlock interlock");}
        require(std::any_of(app.pressure_doors_.begin(),app.pressure_doors_.end(),[](const auto& s){return s.open_fraction>.99f;}),"Requested airlock hatch never opened after equalizing");
        lock.supply.power_fraction=0;app.placed_habitats_[0]=lock;app.update_pressure_doors(.1f);app.first_person_.spawn({.3f,floor,0},90);app.door_hold_={};tick(10,true);
        require(app.door_hold_.blocked&&app.manual_door_label().find("INTERLOCK")!=std::string::npos,"Unpowered opposing hatch ignores the interlock");
        std::cout<<"Explicit powered E / three-second crank / partial release / retarget / pause / safe close / 15 ring layouts / pressure interlock passed\n";
    }
    static void interior_surfaces(){
        using namespace runtime;HabitatMesh mesh;mesh.walkable_floor_y=0;
        mesh.interior={{0,0,0},{2,0,0},{2,0,2},{0,0,0},{2,0,2},{0,0,2},{1,0,1},{3,0,1},{3,0,3},{1,0,1},{3,0,3},{1,0,3}};
        mesh.deck=mesh.interior;interior_finish::finish(mesh);double area=0;
        for(std::size_t i=0;i<mesh.deck.size();i+=3)area+=length(cross(mesh.deck[i+1]-mesh.deck[i],mesh.deck[i+2]-mesh.deck[i]))*.5;
        require(std::abs(area-7)<1e-4&&mesh.interior.empty(),"Coplanar deck union duplicates area or leaves overlapping source faces");
        HabitatModelLibrary library;int cases=0;
        for(int kind=0;kind<int(HabitatKind::Count);++kind)for(int tier=0;tier<int(HabitatSizeTier::Count);++tier){
            const auto& m=library.mesh(HabitatKind(kind),HabitatSizeTier(tier));require(!m.deck.empty(),"Habitat has no finished deck");
            for(const auto& s:m.stations)require(s.first[0]+s.count[0]<=m.interior.size(),"Deck consolidation damaged station ranges");
            for(float x=-m.footprint_x*.4f;x<m.footprint_x*.4f;x+=.413f)for(float z=-m.footprint_z*.4f;z<m.footprint_z*.4f;z+=.397f){
                const Vec3 p{x,0,z};int hits=0;
                for(std::size_t i=0;i<m.deck.size();i+=3){const auto a=m.deck[i],b=m.deck[i+1],c=m.deck[i+2];
                    const auto ab=interior_finish::side(a,b,p),bc=interior_finish::side(b,c,p),ca=interior_finish::side(c,a,p);
                    if((ab>1e-5&&bc>1e-5&&ca>1e-5)||(ab<-1e-5&&bc<-1e-5&&ca<-1e-5))++hits;
                }require(hits<=1,"Habitat deck still contains coplanar overlapping finishes");
            }++cases;
        }
        std::cout<<cases<<" habitat size/type decks: unique planar coverage, preserved station ranges and partial-overlap union passed\n";
    }
    static void fpv_alerts(){
        using namespace runtime;GameHost host;GameApp app(host);
        app.screen_=GameApp::Screen::Playfield;app.first_person_mode_=GameApp::FirstPersonMode::Active;
        app.controlled_astronaut_=0;app.astronaut_views_[0].present=true;
        auto& a=app.test_astronauts_[0];
        const std::array<std::string,5> labels{"LOW HEALTH","LOW OXYGEN","LOW FOOD","LOW WATER","LOW REST"};
        const auto render=[&](Recorder& r){r.begin_frame({});app.on_resize(r.w,r.h);app.render_first_person_alerts(r);};
        const auto caption_count=[&](const Recorder& r,const std::string& text){return std::count_if(r.draws.begin(),r.draws.end(),[&](const auto& d){return d.type=='T'&&d.text==text;});};
        const auto reset=[&](){a={};app.placed_habitats_.clear();};
        Recorder r;
        reset();render(r);require(r.draws.empty(),"Healthy FPV shows critical warnings");
        a.health=.20f;a.suit_oxygen_kg=kSuitOxygenCapacityKg*.20;a.hunger=a.thirst=.80f;a.fatigue=kCrewExhausted*.80f;
        render(r);require(r.draws.empty(),"Exactly 20% reserves incorrectly trigger below-20% alerts");
        int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<Vec2,6>{{{1024,768},{1280,720},{1920,1080},{2560,1440},{3440,1440},{3840,2160}}})for(unsigned mask=0;mask<32;++mask){
            reset();app.ui_scale_index_=zoom;r.resize(int(size.x),int(size.y));app.show_fps_=true;
            a.health=mask&1?.199f:.201f;a.suit_oxygen_kg=kSuitOxygenCapacityKg*(mask&2?.199:.201);
            a.hunger=mask&4?.801f:.799f;a.thirst=mask&8?.801f:.799f;a.fatigue=kCrewExhausted*(mask&16?.801f:.799f);
            render(r);r.check_bounds("FPV critical alerts");
            for(unsigned i=0;i<labels.size();++i)require(caption_count(r,labels[i])==(mask&(1u<<i)?2:0),"A critical need is missing or a recovered need is still warned");
            if(mask){
                require(r.draws.size()>12&&r.draws[0].type=='R'&&r.draws[0].v[0]==0&&r.draws[0].v[1]==0,"Critical needs lack a screen border");
                const auto crew=app.astronaut_status_rect();const float bottom=r.h/app.ui_scale_-layout::BUILD_BAR_H;
                for(const auto& d:r.draws)if(d.type=='T'){
                    require(d.color.r>.9f&&d.color.g<.2f&&d.color.b<.2f&&d.color.a==1,"Critical caption is not persistent red");
                    require(d.v[0]+d.v[2]<crew.x&&d.v[1]+d.v[3]<bottom-29,"Warning covers crew controls or FPS/build bar");
                }
                require(std::count_if(r.draws.begin(),r.draws.end(),[](const auto& d){return d.type=='L';})>0||mask==1,"Critical warning is missing its need icon");
            }else require(r.draws.empty(),"Recovered reserves retained border");
            ++cases;
        }
        reset();a.health=.1f;app.ui_time_seconds_=.25;render(r);const float bright=r.draws[0].color.a;
        app.ui_time_seconds_=.75;render(r);require(bright-r.draws[0].color.a>.7f,"Border is not flashing on UI time");
        require(caption_count(r,"LOW HEALTH")==2&&r.draws.back().color.a==1,"Caption disappears in dim flash phase");
        app.controlled_astronaut_=1;app.astronaut_views_[1].present=true;app.test_astronauts_[1]={};render(r);require(r.draws.empty(),"Warnings come from previously controlled astronaut");app.controlled_astronaut_=0;
        app.first_person_mode_=GameApp::FirstPersonMode::Off;render(r);require(r.draws.empty(),"Alerts remain outside FPV");
        app.first_person_mode_=GameApp::FirstPersonMode::Active;app.screen_=GameApp::Screen::SaveGame;render(r);require(r.draws.empty(),"Alerts appear over menus");app.screen_=GameApp::Screen::Playfield;
        reset();a.suited=false;PlacedHabitat h;h.kind=HabitatKind::OxygenGeneratorDome;h.construction.finish();h.supply.volume_m3=100;
        h.supply.oxygen_kg=h.supply.volume_m3*kOxygenKgPerM3*.19;app.placed_habitats_={h};a.position={0,app.habitat_models_.mesh(h).walkable_floor_y,0};
        render(r);require(caption_count(r,"LOW OXYGEN")==2,"Unsuited cabin oxygen is not monitored");
        app.placed_habitats_[0].supply.oxygen_kg=100*kOxygenKgPerM3; a.suit_oxygen_kg=0;render(r);require(r.draws.empty(),"Unused empty suit falsely warns in healthy cabin air");
        app.placed_habitats_.clear();render(r);require(caption_count(r,"LOW OXYGEN")==2,"Unsuited vacuum fails to warn");
        a.suited=true;a.suit_oxygen_kg=kSuitOxygenCapacityKg;render(r);require(r.draws.empty(),"Full suit warns because exterior has no cabin air");
        std::cout<<cases<<" threshold combinations / resolution / zoom cases; flash phases, crew switch, menus and suit/cabin oxygen passed\n";
    }
    static void saved_fpv_recovery(const std::string& path){
        using namespace runtime;GameHost host;GameApp app(host);std::string error;require(app.load_game_from_file(path,error),error);
        require(app.last_fpv_astronaut_==0&&app.astronaut_present(0)&&app.test_astronauts_[0].health==0,"Provided save no longer reproduces deceased FPV");
        const auto original=app.test_astronauts_[0].position;const auto buildings=app.placed_habitats_.size();
        app.enter_first_person_mode();require(app.first_person_mode_==GameApp::FirstPersonMode::Off&&app.selected_astronaut_==0,"Deceased crew did not expose recovery status");
        require(app.selection_kind_==GameApp::SelectionKind::None,"Dead crew status retained a competing structure selection");
        for(const auto size:std::array<std::pair<int,int>,3>{{{640,360},{960,540},{1920,1080}}}){
            app.on_resize(size.first,size.second);const auto q=app.build_queue_panel_rect(),a=app.astronaut_status_rect();
            require(q.x+q.w<a.x&&q.x>=0,"Build queue covers the deceased crew recovery controls");
        }
        Recorder r;r.set_2d(1);app.on_resize(1920,1080);app.render_astronaut_status(r,{});const auto button=r.button("NEW EXPLORER");
        require(std::any_of(r.draws.begin(),r.draws.end(),[](const auto& d){return d.type=='T'&&d.text.find("DECEASED")!=std::string::npos;}),"Death is still presented as a silent input freeze");
        app.ui_input_.mouse_x=int(button.x+button.w*.5f);app.ui_input_.mouse_y=int(button.y+button.h*.5f);app.ui_input_.left_pressed=true;app.render_astronaut_status(r,{});app.ui_input_={};
        require(app.first_person_mode_==GameApp::FirstPersonMode::Active&&app.controlled_astronaut_==1&&app.test_astronauts_[0].health==0&&length(app.test_astronauts_[0].position-original)<.001f&&app.placed_habitats_.size()==buildings,"Explorer dispatch resurrected/moved the casualty or lost the colony");
        const auto start=app.first_person_.feet_position();require(world_math::distance_xz(start,app.crew_cache_position())<5&&app.first_person_move_allowed(start,start),"Replacement did not start at a clear lander-side position");
        float moved=0;for(int direction=0;direction<4;++direction){app.first_person_.spawn(start,90.f*direction);InputState input;input.key_w_down=true;
            for(int frame=0;frame<60;++frame)app.update_first_person(1.f/60,input);moved=std::max(moved,length(app.first_person_.feet_position()-start));}
        require(moved>1.f,"Replacement FPV remains immobilized");const int current=app.controlled_astronaut_;app.test_astronauts_[current].health=0;app.update_first_person(1.f/60,{});
        require(app.first_person_mode_==GameApp::FirstPersonMode::Off&&app.selected_astronaut_==current,"New death left the player trapped in FPV");
        std::cout<<"Provided 0.74 save: health=0 cause reproduced; explicit new explorer, living movement and safe FPV exit passed\n";
    }
    static void habitat_domes(const std::string& path){
        using namespace runtime;GameHost host;GameApp app(host);std::string error;require(app.load_game_from_file(path,error),error);
        require(app.placed_habitats_.size()==3,"Supplied colony did not load intact");
        const auto buildings=app.placed_habitats_.size();auto& oxygen=app.placed_habitats_[1];require(oxygen.kind==HabitatKind::OxygenGeneratorDome&&oxygen.units.empty(),"Fixture no longer reproduces empty oxygen dome");
        InteriorUnit generator;generator.kind=InteriorUnitKind::OxygenGenerator;
        require(app.find_interior_unit_bay(1,generator),"Supplied save still has no legal oxygen-generator bay");
        const auto iron=app.resources_.amount(MaterialResource::Iron);require(app.place_interior_unit(1,generator,error),error);
        require(app.placed_habitats_.size()==buildings&&oxygen.units.size()==1&&app.resources_.amount(MaterialResource::Iron)==iron-interior_unit_profile(generator.kind).cost.iron,"Generator placement lost buildings or charged incorrect materials");
        struct Capture:Recorder {std::size_t metal{},shield{};bool faded{};void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c,const LunarSurfaceMapping&) override{if(m==LunarMaterial::Alloy)metal+=v.size();if(m==LunarMaterial::SolarShield)shield+=v.size();faded|=!v.empty()&&c.a>0&&c.a<.99f;}};
        for(int k=0;k<int(HabitatKind::Count);++k){const auto& m=app.habitat_models_.mesh(HabitatKind(k));
            require(m.dome_radii.x>0&&m.dome_radii.z>0&&!m.hull_frames.empty()&&m.dome_radii.y==0,"Incorrect pressure-room profile");
            Capture full,cut;app.first_person_mode_=GameApp::FirstPersonMode::Off;app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;app.render_habitat_mesh(full,m,{},0,false,true,false,kConstructionSteps,nullptr,false,HabitatKind(k));
            app.habitat_view_mode_=GameApp::HabitatViewMode::None;app.render_habitat_mesh(cut,m,{},0,false,true,false,kConstructionSteps,nullptr,false,HabitatKind(k));
            require(full.metal-cut.metal==m.hull_frames.size()+m.overhead_services.size()+m.overhead_lamps.size()&&cut.shield==0&&!cut.faded,"Hide hulls retains structural frames or ghost hulls kind="+std::to_string(k));
        }
        app.placed_habitats_.clear();app.placed_resources_.clear();app.corridor_network_.clear();app.utility_network_.clear();
        float previous=0;
        for(const auto size:{InteriorUnitSize::VerySmall,InteriorUnitSize::Small,InteriorUnitSize::Medium}){
            PlacedHabitat h;h.kind=HabitatKind::OxygenGeneratorDome;h.size=HabitatSizeTier::Small;h.construction.finish();h.oxygen_unit_fitted=true;app.placed_habitats_={h};
            const auto& m=app.habitat_models_.mesh(h);generator={};generator.kind=InteriorUnitKind::OxygenGenerator;generator.size=size;
            require(app.find_interior_unit_bay(0,generator),"A generator size cannot fit the restored Small dome");
            const auto& profile=interior_unit_profile(generator);require(profile.size.x>previous,"Generator physical sizes do not increase");previous=profile.size.x;
            require(std::abs(interior_unit_mesh(generator).height-profile.size.y)<.001f,"Rendered generator and collision dimensions disagree");
            require(std::abs(interior_unit_wall_yaw(m,generator.position)-generator.rotation_degrees)<.01f,"Generator is not hull aligned");
            const auto iron=app.resources_.amount(MaterialResource::Iron);require(app.place_interior_unit(0,generator,error),error);
            require(app.resources_.amount(MaterialResource::Iron)==iron-profile.cost.iron,"Generator size charged the wrong cost");
            auto& installed=app.placed_habitats_[0].units[0];installed.water_litres=3.75;installed.progress=.37;installed.batch_active=true;installed.batches=12;
            GameApp roundtrip(host);roundtrip.playfield_.rebuild(128);roundtrip.placed_habitats_=app.placed_habitats_;roundtrip.builder_robots_.resize(1);
            require(roundtrip.save_game_to_name("generator-size-roundtrip",error),error);
            const auto path=std::filesystem::path(roundtrip.save_game_directory())/"generator-size-roundtrip.gmsave";GameApp loaded(host);
            require(loaded.load_game_from_file(path.string(),error),error);
            const auto& saved=loaded.placed_habitats_[0];require(saved.size==HabitatSizeTier::Small&&saved.units.size()==1&&saved.units[0].size==size&&saved.units[0].water_litres==3.75&&saved.units[0].progress==.37&&saved.units[0].batches==12&&saved.units[0].batch_active,"Schema55 lost unit size or inventory");
            if(size==InteriorUnitSize::Medium){
                std::ifstream input(path,std::ios::binary);std::string bytes{std::istreambuf_iterator<char>(input),{}};
                std::string record;const auto pod=[&](auto value){record.append(reinterpret_cast<const char*>(&value),sizeof(value));};
                const auto& u=roundtrip.placed_habitats_[0].units[0];pod(std::uint8_t(u.kind));pod(u.position.x);pod(u.position.y);pod(u.position.z);pod(u.rotation_degrees);pod(std::uint8_t(u.enabled));pod(std::uint8_t(u.batch_active));pod(u.progress);pod(u.ore_kind);pod(u.grow_bay);pod(u.batches);pod(u.water_litres);pod(u.food_kg);pod(std::uint8_t(u.size));
                const auto pos=bytes.find(record);require(pos!=std::string::npos&&bytes.find(record,pos+1)==std::string::npos,"Cannot locate synthetic legacy unit record");
                bytes.erase(pos+record.size()-1,1);const std::uint32_t schema=53;std::memcpy(bytes.data()+8,&schema,sizeof(schema));
                const auto version=bytes.find("0.80.0");if(version!=std::string::npos)bytes.replace(version,6,"0.79.0");
                const auto legacy=path.parent_path()/"generator-schema53.gmsave";{std::ofstream output(legacy,std::ios::binary);output.write(bytes.data(),bytes.size());}
                require(loaded.load_game_from_file(legacy.string(),error),error);std::filesystem::remove(legacy);
                const auto& old=loaded.placed_habitats_[0].units[0];require(old.size==InteriorUnitSize::Medium&&old.water_litres==3.75&&old.batches==12,"An old generator was downgraded or lost inventory");
            }
            std::filesystem::remove(path);
            // Exercise production, demand and water conservation through the real network.
            GameApp sim(host);sim.playfield_.rebuild(128);auto clock=sim.status_.state();clock.hour=12;clock.crew_current=0;sim.status_.restore(clock);
            h.size=HabitatSizeTier::Standard;h.units={generator};sim.placed_habitats_={h};
            PlacedResource tank;tank.kind=ResourceKind::WaterStorage;tank.position={20,0,0};tank.construction.finish();tank.supply.water_litres=80;
            auto solar=tank;solar.kind=ResourceKind::SolarPanel;solar.position={30,0,0};solar.supply.water_litres=0;sim.placed_resources_={tank,solar};
            UtilityConnection pipe;pipe.endpoint_a.kind=UtilityEndpointKind::Habitat;pipe.endpoint_a.index=0;pipe.endpoint_b.kind=UtilityEndpointKind::Resource;pipe.endpoint_b.index=0;pipe.path={{0,0,0},{20,0,0}};pipe.construction.finish();auto power=pipe;power.endpoint_a.kind=UtilityEndpointKind::Resource;power.endpoint_a.index=0;power.endpoint_b.index=1;power.path={{20,0,0},{30,0,0}};
            sim.utility_network_.restore_connections({pipe,power});sim.update_supply_simulation(20);
            const auto& supply=sim.placed_habitats_[0].supply;require(std::abs(supply.oxygen_produced_kg_per_day-oxygen_unit_capacity(generator))<1e-5,"Oxygen output ignores generator size");
            require(std::abs(supply.water_processed_litres_per_day-oxygen_unit_capacity(generator)*1.125)<1e-5,"Sized oxygen plant breaks water balance");
        }
        for(const auto kind:{HabitatKind::Factory,HabitatKind::Manufactory,HabitatKind::ProcessingFacility,HabitatKind::Laboratory,HabitatKind::CantineDome}){
            PlacedHabitat h;h.kind=kind;h.rotation_degrees=37;h.construction.finish();app.placed_habitats_={h};
            for(int n=0;n<int(InteriorUnitKind::Count);++n){const auto type=InteriorUnitKind(n);if(!interior_unit_allowed(kind,type)||!interior_unit_wall_aligned(type))continue;
                InteriorUnit u;u.kind=type;require(app.find_interior_unit_bay(0,u),"A processing unit has no service bay");
                require(app.interior_unit_position_clear(0,u,error),error);require(std::abs(u.position.y-app.habitat_models_.mesh(h).walkable_floor_y)<.001f,"Machine is not level with the deck");
            }
        }
        {
            PlacedHabitat small;small.kind=HabitatKind::ProcessingFacility;small.size=HabitatSizeTier::Small;small.construction.finish();app.placed_habitats_={small};
            InteriorUnit refinery;refinery.kind=InteriorUnitKind::OreRefinery;
            require(app.find_interior_unit_bay(0,refinery),"Small processing facility still has no legal ore-refinery bay");
            require(app.interior_unit_position_clear(0,refinery,error),error);
            InteriorUnit plastic;plastic.kind=InteriorUnitKind::PlasticFabricator;
            require(app.find_interior_unit_bay(0,plastic),"Processing facility has no legal plastic-processing bay");
            require(app.interior_unit_position_clear(0,plastic,error),error);
        }
        const auto& small=app.habitat_models_.mesh(HabitatKind::OxygenGeneratorDome,HabitatSizeTier::Small);const auto& standard=app.habitat_models_.mesh(HabitatKind::OxygenGeneratorDome,HabitatSizeTier::Standard);
        require(std::abs(small.footprint_x/standard.footprint_x-.76f)<.0001f,"Oxygen dome is still enlarged");
        require(std::string(habitat_size_name(HabitatKind::OxygenGeneratorDome,HabitatSizeTier::Standard))=="STANDARD","Dome label still describes the generator size");
        for(std::size_t i=0;i<app.build_catalog_.items().size();++i)if(app.build_catalog_.items()[i].kind==BuildObjectKind::Habitat&&app.build_catalog_.items()[i].habitat_kind==HabitatKind::OxygenGeneratorDome)app.build_catalog_.select(int(i));
        app.placement_habitat_size_=HabitatSizeTier::Small;InputState wheel;wheel.wheel=1;
        for(const auto expected:{HabitatSizeTier::Standard,HabitatSizeTier::Mid,HabitatSizeTier::Big,HabitatSizeTier::Small})require(app.update_placement_controls(wheel)&&app.placement_habitat_size_==expected,"Original dome size selection was not restored");
        PlacedHabitat h;h.kind=HabitatKind::OxygenGeneratorDome;h.size=HabitatSizeTier::Small;h.construction.finish();app.placed_habitats_={h};app.on_resize(1920,1080);app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;
        Recorder ui;ui.set_2d(1);app.render_interior_unit_panel(ui,0,20,100,350);const auto button=ui.button("VERY SMALL");
        app.ui_input_.mouse_x=int(button.x+button.w*.5f);app.ui_input_.mouse_y=int(button.y+button.h*.5f);app.ui_input_.left_pressed=true;app.render_interior_unit_panel(ui,0,20,100,350);app.ui_input_={};
        require(app.interior_unit_size_==InteriorUnitSize::VerySmall&&app.placed_habitats_[0].size==HabitatSizeTier::Small,"Generator selector changed the dome instead of the machine");
        std::cout<<"Restored dome sizing; three generator meshes/costs/output/water balances; schema55 roundtrip and schema53 medium migration; internal size UI passed\n";
    }



    static void autosave(){
        namespace fs=std::filesystem;using namespace runtime;
        const auto folder=fs::temp_directory_path()/fs::path(u8"lunar autosaves ü")/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        struct Scope {
            fs::path folder;
#ifdef _WIN32
            std::wstring previous=_wgetenv(L"GAME_SAVE_DIR")?_wgetenv(L"GAME_SAVE_DIR"):L"";
#else
            std::string previous=std::getenv("GAME_SAVE_DIR")?std::getenv("GAME_SAVE_DIR"):"";
#endif
            ~Scope(){
#ifdef _WIN32
                _wputenv_s(L"GAME_SAVE_DIR",previous.c_str());
#else
                if(previous.empty())unsetenv("GAME_SAVE_DIR");else setenv("GAME_SAVE_DIR",previous.c_str(),1);
#endif
                std::error_code ec;fs::remove_all(folder,ec);
            }
        } scope{folder};
#ifdef _WIN32
        _wputenv_s(L"GAME_SAVE_DIR",folder.c_str());
#else
        setenv("GAME_SAVE_DIR",folder.c_str(),1);
#endif
        GameHost host;GameApp app(host);app.playfield_size_=64;app.start_new_game();
        app.game_speed_=GameApp::GameSpeed::Normal;
        require(app.autosave_minutes_==10,"Autosave default is not ten minutes");
        std::string error;require(app.save_game_to_name("Autosave 1",error),error);
        const auto manual=app.save_games_.front().path;std::ifstream before_file(jsd::utf8_path(manual),std::ios::binary);
        const std::string before{std::istreambuf_iterator<char>(before_file),{}};before_file.close();
        app.update_autosave(599);require(!fs::exists(folder/"autosaves"),"Autosave ran before the real ten-minute deadline");
        app.game_speed_=GameApp::GameSpeed::Pause;app.update_autosave(900);require(app.autosave_elapsed_==599,"Paused play advances autosave");
        app.game_speed_=GameApp::GameSpeed::Fast;app.screen_=GameApp::Screen::Settings;app.update_autosave(900);require(app.autosave_elapsed_==599,"Menu time advances autosave");
        app.screen_=GameApp::Screen::Playfield;app.update_autosave(1);require(app.autosave_elapsed_==0,"Autosave did not run exactly at deadline");
        app.refresh_save_games();require(app.save_games_.size()==2,"Autosave missing from ordinary load list");
        for(int i=0;i<5;++i)app.update_autosave(600);
        int automatic=0;for(const auto& save:app.save_games_)if(jsd::utf8_path(save.path).parent_path()==folder/"autosaves")++automatic;
        require(automatic==3&&app.save_games_.size()==4,"Automatic rotation is not bounded to three separate slots");
        std::ifstream after_file(jsd::utf8_path(manual),std::ios::binary);require(std::string(std::istreambuf_iterator<char>(after_file),{})==before,"Autosave overwrote a manual save with the same name");after_file.close();
        const auto auto_entry=std::find_if(app.save_games_.begin(),app.save_games_.end(),[&](const auto& save){return jsd::utf8_path(save.path).parent_path()==folder/"autosaves";});
        GameApp restored(host);restored.autosave_minutes_=25;restored.autosave_elapsed_=99;
        require(restored.load_game_from_file(auto_entry->path,error),error);
        require(restored.autosave_minutes_==25&&restored.autosave_elapsed_==0,"Loading progress overwrites autosave settings or retains another session's timer");
        app.autosave_minutes_=0;app.update_autosave(900);require(app.autosave_elapsed_==0,"Disabled autosave ran");
        app.autosave_minutes_=1;InputState real;real.frame_seconds=60;app.update(0,real);
        require(app.autosave_elapsed_==0,"Autosave uses clamped simulation delta instead of the actual frame duration");
        fs::remove_all(folder/"autosaves");{std::ofstream blocked(folder/"autosaves");blocked<<"blocked";}
        app.update_autosave(60);require(app.autosave_notice_.find("FAILED")!=std::string::npos,"Failed autosave was silently reported as success");
        require(fs::exists(jsd::utf8_path(manual)),"Failed autosave removed manual progress");
        std::cout<<"Autosave default/deadline, real-time speed independence, pause/menus, three-slot rotation, manual-save isolation, load/reset, disable and failure reporting passed\n";
    }

    static void airlock_access(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        struct Capture:Recorder {
            std::size_t upper{},alloy{};
            void draw_lunar_surface(const std::vector<Vec3>& p,LunarMaterial m,Color)override{
                if(m==LunarMaterial::SolarShield)upper+=p.size();
                if(m==LunarMaterial::Alloy)alloy+=p.size();
            }
        };
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard})for(float rotation:{0.f,37.f,90.f}){
            app.corridor_network_.clear();PlacedHabitat h;h.kind=HabitatKind::AirLock;h.size=tier;h.rotation_degrees=rotation;h.construction.finish();app.placed_habitats_={h};
            const auto& m=app.habitat_models_.mesh(h);const auto doors=app.habitat_doors(0);
            app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;Capture plain,selected,cut;
            app.render_habitat_mesh(plain,m,{},rotation,false,true,false,kConstructionSteps,&doors,false,HabitatKind::AirLock);
            app.render_habitat_mesh(selected,m,{},rotation,false,true,true,kConstructionSteps,&doors,false,HabitatKind::AirLock);
            require(plain.upper>0&&selected.upper==plain.upper&&selected.alloy==plain.alloy,"Selecting airlock hides its full hull or load frames");
            app.habitat_view_mode_=GameApp::HabitatViewMode::None;app.render_habitat_mesh(cut,m,{},rotation,false,true,true,kConstructionSteps,&doors,false,HabitatKind::AirLock);
            require(cut.upper==0&&cut.alloy<plain.alloy,"Explicit airlock no-hull mode retains upper surface/frame");
            for(int sign:{-1,1}){
                const auto d=airlock::end_door(h,m,sign);require(app.airlock_access_enabled(0,d),"Isolated airlock end lacks built access");
                const auto collar=airlock::fitted_collar(d);require(collar.size()>100,"Fixed hatch has no skin-to-frame collar");
                bool reaches_skin=false,reaches_seat=false;for(const auto p:collar){const auto q=world_math::rotate_y(p-h.position,-rotation);
                    reaches_skin|=std::abs(q.x)<m.dome_radii.x-.03f;reaches_seat|=std::abs(q.x)>m.dome_radii.x;}
                require(reaches_skin&&reaches_seat,"Door collar does not bridge the pressure wall and jamb");
                const float run=.32f*std::ceil(m.walkable_floor_y/.16f);
                const auto top=d.wall_position+d.normal*.08f,bottom=d.wall_position+d.normal*(run-.08f);
                require(std::abs(app.first_person_floor_height(top)-m.walkable_floor_y)<.001f&&app.first_person_floor_height(bottom)<.17f,"Built airlock stair floor does not match its tread tops");
            }
            const auto d=airlock::end_door(h,m,1);CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,d.wall_position},b{CorridorEndpointKind::OpenDoor,0,d.wall_position+d.normal*15};
            require(app.corridor_network_.build_corridor(a,b,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"Airlock access fixture connection");
            require(app.airlock_access_enabled(0,d),"Blueprint corridor removed the stairs before being built");
            app.corridor_network_.mutable_connections().front().construction.finish();require(!app.airlock_access_enabled(0,d)&&app.airlock_access_enabled(0,airlock::end_door(h,m,-1)),"Connected sleeve duplicates stair or removes opposite access");
        }
        for(float y:{0.f,.6f,1.8f}){
            CorridorMesh m;corridor_detail::append_ring(m,{0,y,0},{1,0,0});
            bool ground=false,top=false;for(auto p:m.details){ground|=p.y<=.001f&&std::abs(p.z)>1.05f;top|=std::abs(p.y-y-.17f)<.1f&&std::abs(p.z)>1.05f;}
            require(ground&&top,"Elevated corridor support does not join ground and deck");
        }
        std::cout<<"Airlock full-hull selection, explicit cutaway, fitted pressure-wall collars, built stair floor, completed sleeve handoff and elevated ground-bearing corridor supports passed\n";
    }

    static void battery_doors(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        auto clock=app.status_.state();clock.hour=0;clock.crew_current=0;app.status_.restore(clock);
        PlacedHabitat h;h.kind=HabitatKind::AirLock;h.construction.finish();app.placed_habitats_={h};
        PlacedResource battery;battery.kind=ResourceKind::PowerStorage;battery.position={12,0,0};battery.construction.finish();battery.supply.energy_kwh=2;
        app.placed_resources_={battery};
        UtilityConnection feed;feed.kind=UtilityKind::SupplyChannel;feed.construction.finish();feed.endpoint_a.kind=UtilityEndpointKind::Habitat;feed.endpoint_a.index=0;
        feed.endpoint_b.kind=UtilityEndpointKind::Resource;feed.endpoint_b.index=0;feed.path={{0,0,0},{12,0,0}};app.utility_network_.restore_connections({feed});
        app.update_supply_simulation(0);require(app.airlock_powered(0),"Battery-fed airlock turns unpowered at night without charging");
        app.update_pressure_doors(.1f);require(app.pressure_doors_.size()==4,"Battery door fixture missing seals");
        for(const auto& door:app.pressure_doors_)require(door.powered,"Battery-fed hatch is marked manual/unpowered");
        // A high grid load reduces allocation, but low-power door control remains available.
        std::vector<UtilityConnection> feeds{feed};
        for(int i=0;i<4;++i){PlacedResource driller;driller.kind=ResourceKind::OreDriller;driller.position={float(20+i*6),0,0};driller.construction.finish();app.placed_resources_.push_back(driller);
            auto link=feed;link.endpoint_a.kind=UtilityEndpointKind::Resource;link.endpoint_a.index=0;link.endpoint_b.index=i+1;link.path={battery.position,driller.position};feeds.push_back(std::move(link));}
        app.utility_network_.restore_connections(std::move(feeds));
        app.update_supply_simulation(0);require(app.placed_habitats_[0].supply.power_fraction>0&&app.placed_habitats_[0].supply.power_fraction<.99,"Brownout fixture has no partial allocation");
        require(app.airlock_powered(0),"Partial grid load disables battery-backed door automation");
        require(app.interior_light_level(0)>.17f,"Night lighting lost under battery-only brownout");
        const auto before=app.placed_resources_[0].supply.energy_kwh;app.update_supply_simulation(1);
        require(app.placed_resources_[0].supply.energy_kwh<before,"Lighting does not draw stored energy");
        app.placed_habitats_[0].power_enabled=false;require(!app.airlock_powered(0),"Disabled airlock still powers doors");app.placed_habitats_[0].power_enabled=true;
        app.placed_resources_[0].supply.energy_kwh=0;app.update_supply_simulation(0);require(!app.airlock_powered(0),"Empty grid still powers doors");require(app.interior_light_level(0)==0,"Empty grid still powers lights");
        app.placed_resources_[0].supply.energy_kwh=2;app.utility_network_.clear();app.update_supply_simulation(0);require(!app.airlock_powered(0),"Disconnected battery remotely powers door");require(app.interior_light_level(0)==0,"Disconnected battery remotely powers light");
        std::cout<<"Battery-only night doors, partial grid allocation, disabled power, depleted battery and disconnected-grid behavior passed\n";
    }

    static void robot_crew(){
        using namespace runtime;GameHost host;
        const auto setup=[&](GameApp& app,int builders,int carriers){
            app.playfield_.rebuild(128);app.starting_robot_count_=std::max(1,builders);app.starting_carrier_count_=std::max(1,carriers);
            app.reset_builder_robot_and_starter_stash();app.ground_resource_piles_.clear();
            std::erase_if(app.builder_robots_,[&](const auto& r){return (r.role==RobotRole::Constructor&&!builders)||(r.role==RobotRole::Carrier&&!carriers);});
            for(std::size_t i=0;i<app.builder_robots_.size();++i){auto& r=app.builder_robots_[i];r.position={-28,0,-18+float(i)*5};r.visual_initialized=false;}
            app.ground_resource_piles_={{MaterialResource::Iron,300,{-18,0,-4},.4f,1},{MaterialResource::Copper,300,{-18,0,4},.4f,2}};
            for(int i=0;i<3;++i){PlacedResource job;job.kind=ResourceKind::SolarPanel;job.position={12+float(i)*14,0,18};job.placement_sequence=10+i;
                job.resource_plan.total_steps=kConstructionSteps;for(auto& step:job.resource_plan.steps)step.iron=10;app.placed_resources_.push_back(job);}
        };
        {
            GameApp app(host);setup(app,0,2);app.update_builder_robot(1.f/60);
            const auto& a=app.builder_robots_[0];const auto& b=app.builder_robots_[1];
            require(a.target.valid()&&b.target.valid()&&!a.target.same_as(b.target),"Carriers serialized behind the same reserved blueprint");
            require(a.logistics_reservation.iron>0&&b.logistics_reservation.iron>0,"Independent carriers did not publish physical material leases");
            std::int64_t leased=0;for(const auto& r:app.builder_robots_)leased+=r.logistics_reservation.iron;
            require(leased<=300,"Crew reserved more source iron than exists");
        }
        {
            GameApp app(host);setup(app,0,2);auto& active=app.builder_robots_[0];active.target={ConstructionTargetKind::Resource,0};
            active.mode=BuilderRobotMode::Fetching;active.logistics_reservation=app.placed_resources_[0].resource_plan.remaining(0);active.cargo_committed_to_build=true;
            active.route_resolution_elapsed=3;active.blocked_elapsed=3;app.update_builder_robot(1.f/60);
            require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,1}),"Slow path resolution silently released a peer's live material lease");
        }
        {
            GameApp app(host);setup(app,0,1);app.placed_resources_[0].kind=ResourceKind::StorageDome;
            app.placed_resources_[1].kind=ResourceKind::StorageGate;app.placed_resources_[1].storage_parent_index=0;
            app.toggle_construction_priority({ConstructionTargetKind::Resource,1});app.update_builder_robot(1.f/60);
            require(app.builder_robots_[0].target.same_as({ConstructionTargetKind::Resource,0}),"Prioritized dependent job starved its unfinished prerequisite");
        }
        {
            GameApp app(host);setup(app,0,1);for(auto& step:app.placed_resources_[0].resource_plan.steps){step.iron=0;step.plastic=10;}
            app.update_builder_robot(1.f/60);
            require(app.builder_robots_[0].target.same_as({ConstructionTargetKind::Resource,1}),"Unavailable first-job material blocked useful later work");
        }
        {
            GameApp app(host);setup(app,2,1);
            app.placed_resources_[0].site_buffer.iron=60;app.placed_resources_[1].site_buffer.iron=60;
            auto& carrier=app.builder_robots_[2];carrier.target={ConstructionTargetKind::Resource,0};carrier.mode=BuilderRobotMode::Fetching;
            carrier.logistics_reservation.iron=50;carrier.cargo_committed_to_build=true;
            app.update_builder_robot(1.f/60);
            require(app.builder_robots_[0].target.valid()&&app.builder_robots_[1].target.valid()&&
                !app.builder_robots_[0].target.same_as(app.builder_robots_[1].target),"Builders waited for a remote material trip or shared one job");
        }
        {
            GameApp app(host);setup(app,1,0);app.update_builder_robot(1.f/60);const auto& r=app.builder_robots_[0];
            require(r.target.valid()&&r.mode==BuilderRobotMode::Fetching&&r.logistics_reservation.iron>0,"Free constructor cannot self-haul without a carrier");
        }
        {
            GameApp app(host);setup(app,2,0);app.placed_resources_[0].site_buffer.iron=60;app.placed_resources_[1].site_buffer.iron=60;
            auto& blocked=app.builder_robots_[0];blocked.target={ConstructionTargetKind::Resource,0};blocked.mode=BuilderRobotMode::AwaitingMaterials;blocked.blocked_elapsed=5;blocked.safety.block_reason=robots::BlockReason::NoRoute;
            const auto start=blocked.position;app.update_builder_robot(1.f/60);
            require(blocked.crew_deferred_target.same_as({ConstructionTargetKind::Resource,0})&&blocked.crew_deferred_seconds>0,"Blocked claim was not offered back to the crew");
            require(app.builder_robots_[1].target.same_as({ConstructionTargetKind::Resource,0}),"Peer did not take the released work");
            require(length(blocked.position-start)<.3f,"Crew exchange teleported a robot");
        }
        {
            GameApp app(host);setup(app,0,2);app.update_builder_robot(.05f);
            for(int frame=0;frame<420;++frame){
                app.update_builder_robot(.05f);
                std::int64_t iron=0;for(const auto& pile:app.ground_resource_piles_)if(pile.resource==MaterialResource::Iron)iron+=pile.amount;
                for(const auto& r:app.builder_robots_){iron+=r.cargo.iron;require(r.safety.mode!=robots::SafetyMode::Fault,"Cooperative dispatch bypassed movement safety");}
                for(const auto& job:app.placed_resources_)iron+=job.site_buffer.iron;
                require(iron==300,"Concurrent crew pickup/delivery duplicated or lost physical iron");
            }
        }
        {
            GameApp app(host);setup(app,0,1);auto& r=app.builder_robots_[0];r.target={ConstructionTargetKind::Resource,0};
            r.mode=BuilderRobotMode::Delivering;r.cargo.iron=30;r.logistics_reservation.iron=30;r.cargo_committed_to_build=true;r.blocked_elapsed=5;r.safety.block_reason=robots::BlockReason::NoRoute;
            app.update_builder_robot(1.f/60);
            require(r.target.same_as({ConstructionTargetKind::Resource,1})&&r.cargo.iron==30&&r.logistics_reservation.iron==30,"Blocked loaded rover did not exchange its job with cargo intact");
        }
        {
            GameApp app(host);setup(app,2,2);app.ground_resource_piles_[0].amount=1000;
            for(std::size_t i=0;i<app.placed_resources_.size();++i){app.placed_resources_[i].kind=ResourceKind::PowerStorage;app.placed_resources_[i].position={float(i)*20,0,25};}
            bool complete=false;
            for(int frame=0;frame<6000&&!complete;++frame){
                app.update_builder_robot(.05f);std::int64_t iron=0,consumed=0;
                for(const auto& pile:app.ground_resource_piles_)if(pile.resource==MaterialResource::Iron)iron+=pile.amount;
                for(const auto& r:app.builder_robots_)iron+=r.cargo.iron;
                complete=true;for(const auto& job:app.placed_resources_){iron+=job.site_buffer.iron;consumed+=job.resource_plan.remaining(0).iron-job.resource_plan.remaining(job.construction.step).iron;complete&=job.construction.complete();}
                require(iron+consumed==1000,"Parallel construction lost or duplicated iron");
                std::int64_t in_stock=0,outstanding=0;for(const auto& pile:app.ground_resource_piles_)if(pile.resource==MaterialResource::Iron)in_stock+=pile.amount;
                for(const auto& r:app.builder_robots_)outstanding+=std::max<std::int64_t>(0,r.logistics_reservation.iron-r.cargo.iron);
                require(outstanding<=in_stock,"Crew overbooked physical source iron while searching routes");
            }
            require(complete,"Four-robot crew did not finish three independently supplied projects");
        }
        std::cout<<"Crew parallel claims, unavailable-head bypass, independent builders, constructor self-haul, blocked-job exchange and physical-stock conservation passed\n";
    }
    static void rover_unfold(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        app.lander_.phase=LanderPhase::Landed;app.lander_.builders=app.lander_.carriers=6;app.lander_.boxed=true;app.lander_.awaiting_site=false;
        for(int ordinal=0;ordinal<12;++ordinal){
            require(lunar_lander_robot_unfold(app.lander_,ordinal)==0,"Landed robot is not stowed");
            float previous=0;app.lander_.phase=LanderPhase::OpeningRamp;
            for(float t=0;t<=kLanderRampSeconds;t+=.017f){app.lander_.elapsed=t;const auto p=lunar_lander_robot_unfold(app.lander_,ordinal);
                require(p>=previous&&p>=0&&p<=1,"Unfold clock jumps or reverses");previous=p;}
            app.lander_.elapsed=kLanderRampSeconds;require(lunar_lander_robot_unfold(app.lander_,ordinal)==1,"Rover did not unfold before ramp exit");
            app.lander_.phase=LanderPhase::Landed;
        }
        struct Capture:Recorder {std::vector<Vec3> metal;void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color,const LunarSurfaceMapping&)override{
            if(m==LunarMaterial::RobotPaint||m==LunarMaterial::Rubber||m==LunarMaterial::Alloy)metal.insert(metal.end(),v.begin(),v.end());}};
        const auto capture=[&](LanderPhase phase,float time){app.lander_.phase=phase;app.lander_.elapsed=time;Capture r;app.render_builder_robot(r);return r.metal;};
        const auto folded=capture(LanderPhase::OpeningRamp,kLanderRampMotionSeconds),middle=capture(LanderPhase::OpeningRamp,7.75f),ready=capture(LanderPhase::OpeningRamp,kLanderRampSeconds),rolling=capture(LanderPhase::DeployingRobots,0);
        require(folded.size()>10000&&folded.size()==middle.size()&&middle.size()==ready.size()&&ready.size()==rolling.size(),"Rover switches from placeholder blocks to a different model");
        float changed=0,jump=0;for(std::size_t i=0;i<ready.size();++i){changed=std::max(changed,length(ready[i]-folded[i]));jump=std::max(jump,length(ready[i]-rolling[i]));}
        require(changed>.25f&&jump<.001f,"Mechanical unfolding missing or discontinuous at ramp deployment");
        app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        bool independent=false;const float duration=lunar_lander_deployment_seconds(app.lander_,0);
        for(int tick=0;tick<1500&&app.lander_.phase!=LanderPhase::Ready;++tick){app.update_lunar_lander(.05f);
            if(!app.builder_robots_.empty()&&app.builder_robots_.size()<12)independent=true;}
        require(independent&&app.builder_robots_.size()==12&&app.lander_.phase==LanderPhase::Ready,"Robots were held aboard until every rover finished unloading");
        require(duration<75,"Unfolding added an unbounded deployment delay");
        app.lander_.phase=LanderPhase::DeployingRobots;app.lander_.site={};
        require(!lunar_lander_segment_blocked(app.lander_,{6.65f,0,6.7f},{10,0,6.7f},.65f,1),"Released rover cannot leave its parked bay");
        require(lunar_lander_segment_blocked(app.lander_,{2,0,10},{6,0,10},.65f,4),"Active worker can cross an incoming rover's reserved bay route");

        std::cout<<"Twelve rover clock/geometry unfolding, continuous ramp transition and independent parking release passed\n";
    }

    static void airlock_factory_fit(){
        using namespace runtime;GameHost host;GameApp app(host);app.playfield_.rebuild(128);
        app.corridor_network_.set_playfield_half_extent(64);int cases=0;
        for(auto tier:{HabitatSizeTier::Small,HabitatSizeTier::Standard})for(float rotation:{0.f,37.f,90.f})for(int end:{-1,1}){
            app.corridor_network_.clear();app.walkway_network_.clear();app.pressure_doors_.clear();
            PlacedHabitat h;h.kind=HabitatKind::AirLock;h.size=tier;h.rotation_degrees=rotation;h.construction.finish();h.supply.power_fraction=1;app.placed_habitats_={h};
            const auto& mesh=app.habitat_models_.mesh(h);const auto& fitted=app.habitat_runtime_mesh(0);
            require(mesh.entrances.size()==2&&app.habitat_doors(0).size()==4,"Finished isolated airlock lacks its four seals");
            require(fitted.service_metal.size()>1000&&fitted.screens.size()==2&&!fitted.fabric.empty()&&!fitted.lamps.empty(),"Pressure recovery/filter/control/light equipment not fitted with construction");
            require(fitted.service_metal.size()==mesh.service_metal.size(),"Runtime optional-machine filtering removed integral airlock fit");
            InteriorUnit terminal;terminal.kind=InteriorUnitKind::ServiceTerminal;terminal.position={0,mesh.walkable_floor_y,0};std::string error;
            require(!app.place_interior_unit(0,terminal,error),"Fixed airlock accepted optional floor machinery");
            const auto world=[&](float x,float z=0.f){return world_math::rotate_y(Vec3{x,mesh.walkable_floor_y,z},rotation);};
            const auto original=app.habitat_doors(0);const auto n=world_math::rotate_y(Vec3{float(end),0,0},rotation);
            const auto at=world(end*mesh.dome_radii.x);
            require(!app.first_person_move_allowed(at-n*.5f,at+n*.5f),"Isolated airlock end seal is not physical");
            CorridorBuildAnchor a{CorridorEndpointKind::Habitat,0,world(end*mesh.footprint_x*.49f,.65f)},b{CorridorEndpointKind::OpenDoor,0,at+n*15};
            require(app.corridor_network_.build_corridor(a,b,app.placed_habitats_,app.habitat_models_,{},app.resource_models_),"Fixed off-center port did not snap");
            auto& c=app.corridor_network_.mutable_connections().front();
            require(length(c.endpoint_a.door.wall_position-at)<1e-4f&&dot(c.endpoint_a.door.normal,n)>.999f,"Corridor created an off-axis fifth airlock hatch");
            require(app.habitat_doors(0).size()==4,"Blueprint corridor changed fixed seal count");c.construction.finish();
            auto current=app.habitat_doors(0);require(current.size()==original.size(),"Completed corridor changed fixed seal count");
            for(const auto& d:original){bool found=false;for(const auto& q:current)found|=length(d.wall_position-q.wall_position)<1e-4f;require(found,"Connecting moved an airlock pressure door");}
            for(const auto& d:current){PressureDoorState state;state.door=d;state.open_fraction=1;app.pressure_doors_.push_back(state);}
            for(float lane:{-.25f,0.f,.25f})for(float x=-mesh.dome_radii.x+.18f;x<mesh.dome_radii.x-.20f;x+=.065f){
                const auto a=world(x,lane),b=world(x+.065f,lane);
                require(app.first_person_move_allowed(a,b)&&app.first_person_move_allowed(b,a),"Fixed airlock transfer lane blocked");
            }
            // Docking ramp and hull throat must be reversible at actual deck Y.
            const auto sample=[&](Vec3 start,Vec3 finish){
                const int count=std::max(1,int(std::ceil(length(finish-start)/.06f)));
                auto p=start;p.y=app.first_person_floor_height(p);
                for(int i=1;i<=count;++i){auto q=start+(finish-start)*(float(i)/count);q.y=app.first_person_floor_height(q);
                    require(app.first_person_move_allowed(p,q)&&app.first_person_move_allowed(q,p),"Fixed port/corridor floor has an invisible movement barrier");p=q;}
            };
            sample(at-n*.6f,c.endpoint_a.position+n*1.0f);
            // Saved oblique ports migrate to the fixed axis, then remain stable.
            c.endpoint_a.door.wall_position+=world_math::rotate_y(Vec3{0,0,.35f},rotation);
            app.corridor_network_.synchronize_structure_interfaces(app.placed_habitats_,app.habitat_models_,{},app.resource_models_);
            const auto repaired=app.corridor_network_.connections().front().endpoint_a;
            require(length(repaired.door.wall_position-at)<1e-4f,"Legacy airlock port was not aligned");
            app.corridor_network_.synchronize_structure_interfaces(app.placed_habitats_,app.habitat_models_,{},app.resource_models_);
            require(length(app.corridor_network_.connections().front().endpoint_a.position-repaired.position)<1e-4f,"Airlock save migration drifts on repeated load");
            ++cases;
        }
        std::cout<<cases<<" fixed airlock sizes/rotations/end orientations: four pre-fitted seals, pressure equipment, clear lanes, corridor snapping/ramp and stable old-port migration passed\n";
    }

    static void no_hull_boundaries(){
        using namespace runtime;GameHost host;GameApp app(host);
        struct Capture:Recorder {std::vector<Vec3> hull,metal,inside;void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color,const LunarSurfaceMapping&)override{
            auto* out=m==LunarMaterial::Hull?&hull:m==LunarMaterial::Alloy?&metal:m==LunarMaterial::Interior?&inside:nullptr;if(out)out->insert(out->end(),v.begin(),v.end());}};
        app.habitat_view_mode_=GameApp::HabitatViewMode::None;
        for(int k=0;k<int(HabitatKind::Count);++k){const auto kind=HabitatKind(k);const auto& model=app.habitat_models_.mesh(kind);
            Capture cut;app.render_habitat_mesh(cut,model,{},0,false,true,false,kConstructionSteps,nullptr,false,kind);
            require(!cut.hull.empty()&&!cut.inside.empty(),"Cutaway lost lower enclosure or interior");
            for(auto p:cut.hull)require(p.y<=model.walkable_floor_y+.721f,"Cutaway retained an upper wall");
            for(auto p:cut.metal)require(p.y<model.walkable_floor_y+2.81f,"Cutaway retained the overhead circular ring");
            require(!model.overhead_services.empty(),"Full hull lost its original overhead service assembly");
        }
        for(auto kind:{ResourceKind::StorageDome,ResourceKind::RobotWorkshop}){const auto& model=app.resource_models_.mesh(kind);Capture cut;app.render_resource_mesh(cut,model,{},0,false,true,false,kConstructionSteps,nullptr,kind);require(!cut.hull.empty(),"Enterable resource cutaway lost its lower perimeter");for(auto p:cut.hull)require(p.y<=model.walkable_floor_y+.721f,"Resource roof remains in cutaway");}
        const std::vector<Vec3> path{{0,2,0},{0,4,8}},wall{{-1,2,0},{-1,7,0},{-1,7,8},{-1,2,0},{-1,7,8},{-1,4,8}};
        const auto& low=hull_surface::lower_run(wall,path,.75f);require(!low.empty(),"Raised corridor lost its low wall");for(auto p:low)require(p.y<=2+p.z*.25f+.751f,"Low corridor wall ignores local deck slope");
        {
            CorridorEndpoint a,b;a.kind=CorridorEndpointKind::Habitat;b.kind=CorridorEndpointKind::Habitat;
            a.position={0,0,0};a.tangent={1,0,0};a.door.position={-1.6f,0,0};a.door.wall_position={-2.2f,0,0};a.door.normal={-1,0,0};
            b.position={8,0,0};b.tangent={-1,0,0};b.door.position={9.6f,0,0};b.door.wall_position={10.2f,0,0};b.door.normal={1,0,0};
            const auto mesh=corridor_detail::build_mesh(a,b,{{0,0,0},{8,0,0}},{},corridor_detail::NONE_INDEX);require(!mesh.details.empty(),"Synthetic corridor lost its detail mesh");
            const auto lower=[&]{std::vector<Vec3> out;for(std::size_t v=0;v+2<mesh.details.size();v+=3){const auto& p0=mesh.details[v];const auto& p1=mesh.details[v+1];const auto& p2=mesh.details[v+2];
                    if(std::max({p0.y,p1.y,p2.y})<=geometry::kCorridorDeckY+.20f)out.insert(out.end(),{p0,p1,p2});}return out;}();
            require(!lower.empty(),"No-hull corridor connection lost all lower deck/support geometry");
            bool reaches_left=false,reaches_right=false;for(const auto& p:lower){reaches_left|=p.x<-1.0f;reaches_right|=p.x>9.0f;}
            require(reaches_left&&reaches_right,"Corridor connector sleeves still miss walkway/support geometry near structures");
        }
        {
            // Synthetic, correctly oriented hatches at differing heights: the
            // visible deck must cover every centerline sample in BOTH hull modes.
            CorridorConnection c;c.construction.finish();c.path={{0,.60f,0},{8,.60f,0}};
            c.endpoint_a.kind=c.endpoint_b.kind=CorridorEndpointKind::Habitat;
            c.endpoint_a.position=c.path.front();c.endpoint_b.position=c.path.back();
            c.endpoint_a.tangent={1,0,0};c.endpoint_b.tangent={-1,0,0};
            c.endpoint_a.door.normal={1,0,0};c.endpoint_b.door.normal={-1,0,0};
            c.endpoint_a.door.position={-1.75f,1.4f,0};c.endpoint_a.door.wall_position={-3.45f,1.4f,0};
            c.endpoint_b.door.position={9.75f,.8f,0};c.endpoint_b.door.wall_position={11.45f,.8f,0};
            c.mesh=corridor_detail::build_mesh(c.endpoint_a,c.endpoint_b,c.path,{},corridor_detail::NONE_INDEX);
            app.corridor_network_.restore_connections({c});
            struct DeckCapture:Recorder{std::vector<Vec3> deck;void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color,const LunarSurfaceMapping&)override{if(m==LunarMaterial::CorridorDeck)deck.insert(deck.end(),v.begin(),v.end());}
                void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color)override{if(m==LunarMaterial::Alloy)deck.insert(deck.end(),v.begin(),v.end());}};
            for(auto mode:{GameApp::HabitatViewMode::FullHull,GameApp::HabitatViewMode::None}){
                app.habitat_view_mode_=mode;DeckCapture draw;app.render_corridors(draw);
                for(float x=-3.35f;x<11.35f;x+=.07f){const float y=app.first_person_floor_height({x,0,0});bool covered=false;
                    for(std::size_t j=0;j+2<draw.deck.size();j+=3){auto a=draw.deck[j],b=draw.deck[j+1],d=draw.deck[j+2];const auto n=cross(b-a,d-a);if(n.y<=.000001f)continue;
                        const Vec3 q{x,0,0};const float t0=cross(b-a,q-a).y,t1=cross(d-b,q-b).y,t2=cross(a-d,q-d).y;
                        if(std::min({t0,t1,t2})<-.00001f)continue;
                        const float top=a.y-(n.x*(x-a.x)-n.z*a.z)/n.y;if(std::abs(top-y)<.025f){covered=true;break;}}
                    require(covered,"Missing rendered walking surface at x="+std::to_string(x)+" y="+std::to_string(y));
                }
            }
            require(corridor_detail::docking_step_count(c.endpoint_a.door.position,c.path.front()+Vec3{0,geometry::kCorridorDeckY,0})>1,"Steep adapter has no stairs");
            app.corridor_network_.clear();
        }
        std::cout<<"Ten rectangular habitats, storage/workshop, cutaway boundaries and continuous rendered stair/deck surfaces passed\n";
    }
    static void suit_refill(){
        using namespace runtime;GameHost host;
        const auto run=[&](double level,bool power,bool outside,bool full){GameApp app(host);app.playfield_.rebuild(128);app.astronaut_views_[0].present=true;for(auto& a:app.test_astronauts_)a.health=0;
            PlacedHabitat room;room.kind=HabitatKind::CrewQuarters;room.construction.finish();room.power_enabled=power;app.placed_habitats_={room};
            app.update_supply_simulation(0);auto& air=app.placed_habitats_[0].supply;air.oxygen_kg=air.volume_m3*kOxygenKgPerM3*level;
            auto& actor=app.test_astronauts_[0];actor.position=outside?Vec3{45,0,45}:Vec3{0,app.habitat_models_.mesh(room).walkable_floor_y,0};actor.health=1;actor.suit_oxygen_kg=full?kSuitOxygenCapacityKg:.1;
            auto clock=app.status_.state();clock.crew_current=0;app.status_.restore(clock);const double before_air=air.oxygen_kg,before_suit=actor.suit_oxygen_kg;
            app.update_supply_simulation(1);const double loss=before_air-air.oxygen_kg,gain=actor.suit_oxygen_kg-before_suit,metabolic=kCrewOxygenKgPerDay/kGameDaySeconds;
            require(actor.suit_oxygen_kg>=0&&actor.suit_oxygen_kg<=kSuitOxygenCapacityKg,"Suit refill exceeded its finite tank");
            if(!outside)require(std::abs(loss-gain-metabolic)<1e-7,"Suit refill created cabin oxygen or lost mass");
            else require(loss==0&&gain<0,"Vacuum refilled the suit");
            if(level==0)require(loss==0&&gain<0,"Empty cabin created suit oxygen");
            return loss;
        };
        const auto low=run(.5,false,false,false),high=run(1,false,false,false);require(high>low*1.8,"Suit refill does not scale with cabin oxygen");
        run(0,false,false,false);run(1,true,true,false);run(1,false,false,true);
        std::cout<<"Oxygen-scaled suit refill in unpowered cabins, finite stock, no vacuum/empty-room refill, tank cap and oxygen conservation passed\n";
    }
    static void dome_save_probe(const std::string& path){
        using namespace runtime;GameHost host;GameApp app(host);std::string error;require(app.load_game_from_file(path,error),error);
        for(std::size_t i=0;i<app.placed_habitats_.size();++i){auto& h=app.placed_habitats_[i];
            std::cout<<"habitat "<<i<<" kind="<<int(h.kind)<<" size="<<int(h.size)<<" units="<<h.units.size()<<" fitted="<<h.oxygen_unit_fitted<<" pos="<<h.position.x<<","<<h.position.z<<"\n";
            if(h.kind!=HabitatKind::OxygenGeneratorDome)continue;
            const auto& m=app.habitat_models_.mesh(h);std::map<std::string,int> reasons;int valid=0;
            for(float z=-4;z<=4;z+=.5f)for(float x=-4;x<=4;x+=.5f){InteriorUnit u;u.kind=InteriorUnitKind::OxygenGenerator;u.position={x,m.walkable_floor_y,z};u.rotation_degrees=interior_unit_wall_yaw(m,u.position);
                if(app.interior_unit_position_clear(i,u,error)){++valid;if(valid==1)std::cout<<" first legal "<<x<<","<<z<<"\n";}else ++reasons[error];}
            std::cout<<" legal placements="<<valid<<"\n";for(const auto& [reason,n]:reasons)std::cout<<" "<<n<<" "<<reason<<"\n";
        }
    }
    static void fpv_save_probe(const std::string& path){
        using namespace runtime;GameHost host;GameApp app(host);std::string error;require(app.load_game_from_file(path,error),error);
        std::cout<<"last FPV "<<app.last_fpv_astronaut_<<" mode "<<int(app.first_person_mode_)<<"\n";
        for(std::size_t i=0;i<app.test_astronauts_.size();++i)if(app.astronaut_present(i)){const auto& a=app.test_astronauts_[i];
            app.controlled_astronaut_=int(i);std::cout<<"actor "<<i<<" feet "<<a.position.x<<","<<a.position.y<<","<<a.position.z<<" health "<<a.health<<" activity "<<int(a.activity)<<" clear "<<app.first_person_move_allowed(a.position,a.position)<<" floor "<<app.first_person_floor_height(a.position)<<" ceiling "<<app.first_person_ceiling_height(a.position)<<" yaw "<<app.astronaut_views_[i].yaw<<"\n";
            for(const auto delta:{Vec3{.08f,0,0},Vec3{-.08f,0,0},Vec3{0,0,.08f},Vec3{0,0,-.08f}})std::cout<<" direction "<<delta.x<<","<<delta.z<<" "<<app.first_person_move_allowed(a.position,a.position+delta)<<"\n";
        }
        app.controlled_astronaut_=-1;app.enter_first_person_mode();const auto before=app.first_person_.feet_position();
        std::cout<<"entered "<<before.x<<","<<before.y<<","<<before.z<<" clear "<<app.first_person_move_allowed(before,before)<<"\n";
        InputState input;input.key_w_down=true;
        for(int frame=0;frame<120;++frame){app.input_=input;app.update_pressure_doors(1.f/60);app.update_first_person(1.f/60,input);}
        const auto after=app.first_person_.feet_position();std::cout<<"after W "<<after.x<<","<<after.y<<","<<after.z<<" moved "<<length(after-before)<<"\n";
    }
    static void settings_boundary(){
        namespace fs=std::filesystem;using namespace runtime;
        const auto existing=load_preferences();
        struct Restore{Preferences value;~Restore(){save_preferences(value);}} restore{existing};
        GameHost host;GameApp app(host);app.playfield_.rebuild(128);app.builder_robots_.resize(1);
        auto clock=app.status_.state();clock.day=9;clock.hour=6.25f;app.status_.restore(clock);
        app.active_game_phase_=GameApp::GamePhase::RobotPreparation;
        PlacedHabitat building;building.kind=HabitatKind::OxygenGeneratorDome;building.position={12,0,14};building.construction.finish();app.placed_habitats_={building};
        app.astronaut_views_[0].present=app.astronaut_views_[0].visited=true;app.test_astronauts_[0].position={25,0,25};app.test_astronauts_[0].health=.73f;app.astronaut_views_[0].yaw=137;
        app.graphics_.render_scale=85;app.ui_scale_index_=1;app.habitat_view_mode_=GameApp::HabitatViewMode::FullHull;app.show_fps_=false;app.soundscape_.set_volume(.22f);app.soundscape_.set_music_enabled(false);app.first_person_.set_fov(62);
        const auto path=fs::path(app.save_game_directory())/"settings-boundary.gmsave";
        const auto legacy=fs::path(app.save_game_directory())/"settings-boundary-legacy.gmsave";
        struct Remove{fs::path a,b;~Remove(){std::error_code e;fs::remove(a,e);fs::remove(b,e);}} remove{path,legacy};
        const auto bytes=[](const fs::path& p){std::ifstream f(p,std::ios::binary);return std::vector<char>(std::istreambuf_iterator<char>(f),{});};
        std::string error;require(app.save_game_to_name("settings-boundary",error),error);const auto first=bytes(path);
        app.graphics_.render_scale=100;app.ui_scale_index_=4;app.habitat_view_mode_=GameApp::HabitatViewMode::None;app.show_fps_=true;app.soundscape_.set_volume(.83f);app.soundscape_.set_music_enabled(true);app.first_person_.set_fov(104);app.debug_frame_enabled_=false;app.world_layout_mode_=GameApp::WorldLayoutMode::Mesh;
        app.placement_rotation_degrees_=73;app.placement_habitat_size_=HabitatSizeTier::Big;app.placement_storage_size_=StorageDomeSize::Huge;app.build_catalog_.set_category(BuildCategory::Structures);
        require(app.save_game_to_name("settings-boundary",error),error);
        require(bytes(path)==first,"Settings or build tools are still serialized in savegame progress");
        app.persist_preferences();require(!app.preferences_save_failed_,"Game-wide settings were not saved");
        const auto preferences=load_preferences();
        require(preferences.ui_scale_index==4&&preferences.faded_hulls&&preferences.sound_volume_percent==83&&!preferences.debug_frame_enabled&&preferences.first_person_fov==104&&preferences.show_fps&&preferences.mesh_layout&&preferences.music_enabled,"Game-wide preference round trip omitted a setting");
        GameApp restarted(host);restarted.on_start();
        const auto unchanged=[&]{return restarted.ui_scale_index_==4&&restarted.habitat_view_mode_==GameApp::HabitatViewMode::None&&restarted.graphics_.render_scale==100&&restarted.first_person_.fov()==104&&restarted.show_fps_&&restarted.world_layout_mode_==GameApp::WorldLayoutMode::Mesh&&!restarted.debug_frame_enabled_&&std::abs(restarted.soundscape_.volume()-.83f)<.001f&&restarted.soundscape_.music_enabled();};
        require(unchanged(),"Restart did not restore game-wide settings");
        require(restarted.load_game_from_file(path.string(),error),error);require(unchanged(),"Schema52 load changed preferences");
        require(restarted.status_.state().day==9&&std::abs(restarted.status_.state().hour-6.25f)<.001f&&restarted.placed_habitats_.size()==1&&restarted.astronaut_present(0)&&restarted.test_astronauts_[0].health==.73f&&restarted.astronaut_views_[0].yaw==137,"Progress or astronaut view was lost while separating settings");
        // Reconstruct a schema51 header with deliberately conflicting display,
        // hull and tool values. Its body has the same colony schema as 52.
        std::size_t offset=12;const auto u32=[&](std::size_t i){std::uint32_t n;std::memcpy(&n,first.data()+i,4);return n;};
        require(u32(8)==55,"New save schema was not advanced");offset+=4+u32(offset);offset+=4+u32(offset);offset+=4;
        std::vector<char> older(first.begin(),first.begin()+offset);const std::uint32_t schema=51;std::memcpy(older.data()+8,&schema,4);
        const auto append=[&](auto value){const auto p=reinterpret_cast<const char*>(&value);older.insert(older.end(),p,p+sizeof(value));};
        append(std::int32_t{640});append(std::int32_t{480});append(std::uint8_t{0});append(std::int32_t{0});append(std::uint8_t{0});append(std::uint8_t{0});append(0.f);append(std::uint8_t{0});append(std::uint8_t{0});append(std::uint8_t{0});
        older.insert(older.end(),first.begin()+offset,first.end());{std::ofstream f(legacy,std::ios::binary);f.write(older.data(),older.size());}
        require(restarted.load_game_from_file(legacy.string(),error),error);require(unchanged(),"Legacy save reinstalled its display/UI preferences");
        require(restarted.load_game_from_file((fs::path(GAME_SOURCE_ROOT)/"game/tests/fixtures/schema48-lander-depot.gmsave").string(),error),error);require(unchanged(),"Schema48 load changed game-wide settings");
        require(restarted.load_game_from_file((fs::path(GAME_SOURCE_ROOT)/"game/tests/fixtures/schema29-stage1-contracts.gmsave").string(),error),error);require(unchanged(),"Legacy noncompact load changed settings");
        restarted.new_game_phase_selection_=GameApp::GamePhase::Planning;restarted.start_new_game();require(unchanged(),"New Game reset game-wide settings");
        GameApp controls(host);controls.on_start();controls.screen_=GameApp::Screen::Settings;controls.ui_scale_index_=1;Recorder widgets;controls.render(widgets);
        const auto click=[&](const Rect& rect){controls.input_.mouse_x=int((rect.x+rect.w*.5f)*widgets.scale);controls.input_.mouse_y=int((rect.y+rect.h*.5f)*widgets.scale);controls.input_.left_pressed=true;controls.render(widgets);controls.input_={};};
        click(widgets.button(">"));require(load_preferences().ui_scale_index==2,"UI scale control did not persist");
        controls.render(widgets);click(widgets.button("MUTE"));require(load_preferences().sound_volume_percent==0,"Mute control did not persist");
        controls.render(widgets);click(widgets.button("UNMUTE"));require(load_preferences().sound_volume_percent==72,"Unmute control did not persist");
        Recorder hull;hull.set_2d(1);controls.ui_input_={};controls.render_view_selector(hull,0,1280);const auto full=hull.button("FULL HULL");
        controls.ui_input_.mouse_x=int(full.x+full.w*.5f);controls.ui_input_.mouse_y=int(full.y+full.h*.5f);controls.ui_input_.left_pressed=true;controls.render_view_selector(hull,0,1280);
        require(!load_preferences().faded_hulls,"Hull view control did not persist");
        std::cout<<"Progress-only schema52 bytes, legacy schema51/48/29 preference isolation, game-wide UI/hull/audio/graphics/FOV settings and astronaut continuity passed\n";
    }

    static void recent_saves(){
        namespace fs=std::filesystem;
        game::runtime::GameHost engine;
        GameApp original(engine);
        original.builder_robots_.resize(1);
        const std::string prefix="resume-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        const std::string first=prefix+" A",second=prefix+"_A",debug=prefix+" Debug";
        std::vector<fs::path> paths;
        std::string error;
        const auto save=[&](const std::string& name,int size){
            original.playfield_.rebuild(size);
            require(original.save_game_to_name(name,error),error);
            const auto entry=std::find_if(original.save_games_.begin(),original.save_games_.end(),
                [&](const auto& candidate){return candidate.name==name;});
            require(entry!=original.save_games_.end(),"new save was omitted from the list");
            return fs::path(entry->path);
        };
        try{
            paths.push_back(save(first,64));
            original.habitat_view_mode_=GameApp::HabitatViewMode::None;
            paths.push_back(save(second,128));
            require(paths[0]!=paths[1],"distinct save names overwrote the same file");
            require(save(second,192)==paths[1],"same-name replacement created a duplicate save");

            original.active_game_phase_=GameApp::GamePhase::Debug;
            paths.push_back(save(debug,256));
            const auto now=fs::file_time_type::clock::now();
            fs::last_write_time(paths[0],now-std::chrono::hours(1));
            fs::last_write_time(paths[1],now);
            fs::last_write_time(paths[2],now+std::chrono::hours(1));

            GameApp restarted(engine);
            restarted.on_start();
            const auto global_view=restarted.habitat_view_mode_;
            require(!restarted.save_games_.empty() && restarted.save_games_.front().name==debug,"save list is out of date order");
            Recorder recorder;
            restarted.render(recorder);
            const auto button=recorder.button("CONTINUE");
            restarted.input_.mouse_x=static_cast<int>((button.x+button.w/2)*recorder.scale);
            restarted.input_.mouse_y=static_cast<int>((button.y+button.h/2)*recorder.scale);
            restarted.input_.left_pressed=true;
            restarted.render(recorder);
            require(restarted.screen_==GameApp::Screen::Playfield &&
                    restarted.save_game_name_==(build::debug_features?debug:second) &&
                    restarted.playfield_.size()==(build::debug_features?256:192),
                    "Continue did not load the newest compatible save");
            require(restarted.habitat_view_mode_==global_view,"Continue replaced the game-wide hull view");
            require(restarted.load_game_from_file(paths[0].string(),error) && restarted.playfield_.size()==64 &&
                    restarted.habitat_view_mode_==global_view,
                    "Loading an earlier colony changed its progress or game-wide view");
        }catch(...){
            for(const auto& path:paths) fs::remove(path);
            throw;
        }
        for(const auto& path:paths) fs::remove(path);
    }

    static void run(const std::string& output){
        order_controls();
        presentation_controls();
        airlock_status();
        status_bars(output);
        storage_and_rate_display();
        structure_power_controls();
        grouped_build_menu();
        resource_inventory(output);
        progress_tree();
        scrollable_selection_panel();
        hydroponics_panel();
        game::runtime::GameHost engine;
        constexpr std::array<game::runtime::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(int scene=0;scene<20;++scene){
            Recorder reference;
            for(std::size_t n=0;n<sizes.size();++n){
                GameApp app(engine);setup(app,scene,zoom);Recorder r;r.resize((int)sizes[n].x,(int)sizes[n].y);app.render(r);
                const std::string label="scene "+std::to_string(scene)+" zoom "+std::to_string(zoom)+" "+std::to_string(r.w);
                r.check_bounds(label);
                const auto has_text=[&](const std::string& part){for(const auto& d:r.draws)if(d.type=='T' && d.text.find(part)!=std::string::npos)return true;return false;};
                if(scene==8 || scene==9 || (scene>=13 && scene<=15))require(has_text("BREATHABLE AIR") && has_text("AIR CHANGE"),"indoor details omit the air percentage/change rate");
                if(scene==13)require(has_text("75.0%") && has_text("+0.125") && has_text("O2 OUTPUT 9.000 KG/D / WATER 10.125 L/D") && has_text("H2 OUTPUT 1.125 KG/D / BUFFER 2.250") && has_text("ELECTROLYSIS: O2 + H2"),"oxygen-generator details do not show measured oxygen/water/hydrogen output");
                if(scene==18)require(has_text("FUEL CELL 2.75 / 3.00 KW") && has_text("H2 USE 3.600 KG/D") && has_text("COMPLETED SUPPLY-CHANNEL NETWORK"),"hydrogen-generator details omit fuel-cell dispatch/transport telemetry");
                if(scene==19)require(has_text("HYDROPONICS") && has_text("PHOTOPERIOD"),"Bio Dome details omit hydroponics/photoperiod status");
                if(scene==9)require(has_text("WARNING:") && has_text("AIR BELOW 95%"),"storage warning icon reason missing");
                if(scene==16 || scene==17)require(has_text("WARNING:") && has_text("INSUFFICIENT POWER"),"storage/workshop early return hides warning reasons");
                if(scene==12)require(has_text("SPACE JUMP") && has_text("SHIFT SPRINT") && !has_text("SPACE PAUSE"),"POV help omits sprint/jump or still advertises Space pause");
                if(scene==14)require(has_text("-0.125"),"depleting interior details hide negative air change");
                if(n==0)reference=r;else compare(reference,r,label);
                if(!output.empty() && ((zoom==2 && (n==0 || n==4)) || (zoom==4 && n==0)))r.write(std::filesystem::path(output)/("ui-"+std::to_string(scene)+"-"+std::to_string(r.w)+"-zoom"+std::to_string(zoom)+".json"));
            }
        }
        const std::array<std::string,8> reasons{{"INSUFFICIENT POWER","NO TANK WATER","AIR BELOW 95%","RESERVE BELOW 10%","NO CONNECTED","LOW FOOD","LOW MEDICAL","NO CONNECTED H2"}};
        int structure_cases=0;
        for(int domain=0;domain<2;++domain)for(int kind=0;kind<(domain==0?(int)HabitatKind::Count:(int)ResourceKind::Count);++kind)for(int warning_state=0;warning_state<2;++warning_state){
            GameApp app(engine);setup(app,7,4);SupplyState* supply=nullptr;
            if(domain==0){auto& h=app.placed_habitats_[0];h.kind=(HabitatKind)kind;h.construction.step=kConstructionSteps;supply=&h.supply;app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;}
            else {auto& item=app.placed_resources_[0];item.kind=(ResourceKind)kind;item.construction.step=kConstructionSteps;supply=&item.supply;app.selection_kind_=GameApp::SelectionKind::Resource;app.selected_resource_=0;}
            supply->warnings=warning_state?255:0;
            Recorder detail;detail.resize(1280,720);app.render(detail);detail.check_bounds("all structure warnings");
            const auto panel=app.selection_panel_rect();require(panel.y+panel.h<=720/app.ui_scale_-layout::BUILD_BAR_H+0.01f,"warning details overlap build bar");
            std::string shown;for(const auto& d:detail.draws)if(d.type=='T' && d.v[0]>=panel.x && d.v[0]<panel.x+panel.w && d.v[1]>=panel.y+75 && d.v[1]<panel.y+110){shown+=d.text;shown+=' ';require(d.v[0]+d.v[2]<=panel.x+panel.w-10,"warning text escapes frame");}
            if(warning_state){require(shown.find("WARNING:")!=std::string::npos,"structure warning reasons missing");for(const auto& reason:reasons)require(shown.find(reason)!=std::string::npos,"omitted structure warning: "+reason);}
            else require(shown.find("WARNING:")==std::string::npos,"stale warning reason with no warning icon");
            ++structure_cases;
        }
        for(const auto size:sizes){
            GameApp app(engine);setup(app,0,2);Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);
            const auto start=r.button("NEW GAME");app.input_.mouse_x=(int)std::lround((start.x+start.w*0.5f)*r.scale);app.input_.mouse_y=(int)std::lround((start.y+start.h*0.5f)*r.scale);app.input_.left_pressed=true;app.render(r);
            require(app.screen_==GameApp::Screen::NewGame,"menu hitbox missed after resolution scaling");
            app.input_={};app.selection_kind_=GameApp::SelectionKind::Robot;app.on_resize(r.w,r.h);const auto pane=app.selection_panel_rect();game::runtime::InputState pointer;
            pointer.mouse_x=(int)std::lround((pane.x+pane.w/2)*app.ui_scale_);pointer.mouse_y=(int)std::lround((pane.y+pane.h/2)*app.ui_scale_);
            require(app.mouse_over_selection_panel(pointer),"selection hitbox differs from drawing");pointer.mouse_x=0;require(!app.mouse_over_selection_panel(pointer),"selection hitbox extends outside panel");
        }
        {
            GameApp stages(engine);setup(stages,2,2);Recorder stage_ui;stage_ui.resize(1920,1080);stages.render(stage_ui);
            bool planning=false,robot_stage=false,manned=false;for(const auto& d:stage_ui.draws)if(d.type=='T'){planning|=d.text.find("1  PLANNING")!=std::string::npos;robot_stage|=d.text.find("2  ROBOT PREPARATION")!=std::string::npos;manned|=d.text.find("3  DEBUG MODES")!=std::string::npos;}
            require(planning&&robot_stage&&manned,"new-game stage selector is incomplete");
            require(stages.new_game_mode_selection_==GameApp::NewGameMode::Career && stages.new_game_phase_selection_==GameApp::GamePhase::Planning,
                    "new game does not default to Career / Stage 1");
            require(stages.playfield_size_==GameApp::kNewGamePlayfieldDefault,"Stage 1 did not keep the fixed playfield");
            stages.new_game_phase_selection_=GameApp::GamePhase::Planning;stages.render(stage_ui);const auto create=stage_ui.button("START CAREER");
            stages.input_.mouse_x=(int)std::lround((create.x+create.w*0.5f)*stage_ui.scale);stages.input_.mouse_y=(int)std::lround((create.y+create.h*0.5f)*stage_ui.scale);stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.screen_==GameApp::Screen::Playfield && stages.active_game_phase_==GameApp::GamePhase::Planning,
                    "Career did not start in Stage 1");
            require(stages.active_game_mode_==GameApp::NewGameMode::Career && stages.playfield_size_==GameApp::kCompanySitePlayfieldSize,
                    "Career selection or fixed campus size did not apply");
            require(stages.builder_robots_.empty() && stages.placed_habitats_.empty() && stages.placed_resources_.empty(),"Stage 1 accidentally started Stage 2 gameplay content");
            stages.input_={};stages.render(stage_ui);bool campus=false,assembly=false,launch=false,site_button=false;for(const auto& d:stage_ui.draws)if(d.type=='T'){campus|=d.text.find("SPACE COMPANY LAUNCH CAMPUS")!=std::string::npos;assembly|=d.text.find("VEHICLE ASSEMBLY")!=std::string::npos;launch|=d.text.find("LAUNCH COMPLEX")!=std::string::npos;site_button|=d.text.find("COMPANY SITE")!=std::string::npos;}
            require(stages.planning_.view==PlanningView::Site && campus&&assembly&&launch&&site_button,"Stage 1 did not open on the interactive 3D company site");
            // The novice shortcut should lead to a useful action, not a
            // generic panel or a hidden game-state mutation.
            auto next_step=stage_ui.button("NEXT STEP");
            stages.input_.mouse_x=(int)std::lround((next_step.x+next_step.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((next_step.y+next_step.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.view==PlanningView::Contracts && stages.planning_contract_selected_==0 &&
                    stages.planning_.funds_eur==1'500'000,
                    "first campaign guidance did not open the tutorial contract safely");
            stages.planning_.view=PlanningView::Site;
            stages.input_={};stages.render(stage_ui);

            // Building clicks are the physical navigation layer. Point the
            // camera straight at the Program Office; it must open that
            // facility's mission-workflow station rather than bypassing gates.
            stages.camera_.restore({80.0f,90.0f,0.0f,{-40.0f,0.0f,-24.0f}});
            stages.input_={};stages.input_.mouse_x=stage_ui.w/2;stages.input_.mouse_y=stage_ui.h/2;stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.view==PlanningView::Mission && stages.planning_.mission_facility==MissionFacility::ProgramOffice,"clicking Program & Contracts did not open its mission-preparation station");
            stages.input_={};stages.render(stage_ui);const auto contracts_shortcut=stage_ui.button("CONTRACTS");
            stages.input_.mouse_x=(int)std::lround((contracts_shortcut.x+contracts_shortcut.w*0.5f)*stage_ui.scale);stages.input_.mouse_y=(int)std::lround((contracts_shortcut.y+contracts_shortcut.h*0.5f)*stage_ui.scale);stages.input_.left_pressed=true;stages.render(stage_ui);
            stages.input_={};stages.render(stage_ui);bool market=false,study=false,tech_button=false;for(const auto& d:stage_ui.draws)if(d.type=='T'){market|=d.text.find("CUSTOMER CONTRACTS")!=std::string::npos;study|=d.text.find("Mission Architecture Study")!=std::string::npos;tech_button|=d.text.find("TECH TREE")!=std::string::npos;}
            require(market&&study&&tech_button,"Stage 1 planning gameplay surface missing");
            require(stages.planning_.funds_eur==1'500'000,"Stage 1 seed funding changed unexpectedly");
            require(stages.accept_planning_contract(0),"initial planning contract was not selectable");
            require(stages.planning_.funds_eur==1'750'000 && stages.planning_.contracts[0].status==PlanningContractStatus::Active,"contract deposit was not paid immediately");
            stages.input_={};stages.render(stage_ui);
            const auto fast_report=stage_ui.button("ADVANCE TO NEXT REPORT");
            stages.input_.mouse_x=(int)std::lround((fast_report.x+fast_report.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((fast_report.y+fast_report.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.contracts[0].progress_days==24.0 && stages.planning_.time_speed==0 &&
                    stages.planning_.funds_eur==1'750'000,
                    "contract skip-to-report button must pause at the report without paying it");

            stages.planning_.time_speed=3;stages.update_planning_game(1.20f);
            require(stages.planning_.contracts[0].status==PlanningContractStatus::Active && stages.planning_.funds_eur==1'750'000,
                    "project was paid automatically before its customer deliverable was submitted");
            stages.input_={};stages.render(stage_ui);
            bool ready_for_delivery=false;for(const auto& d:stage_ui.draws)if(d.type=='T')ready_for_delivery|=d.text.find("READY FOR DELIVERY")!=std::string::npos;
            require(ready_for_delivery,"finished customer project was not highlighted for delivery");
            require(stages.deliver_planning_contract(0),"customer deliverable could not be submitted");
            require(!stages.deliver_planning_contract(0),"customer deliverable could be paid twice");
            require(stages.planning_.contracts[0].completed_count==1 && stages.planning_.contracts[0].status==PlanningContractStatus::Completed,"one-time planning contract did not close after delivery");
            require(stages.planning_.funds_eur==2'300'000,"contract final payout was not credited");
            require(stages.planning_technology_available(0) && stages.unlock_planning_technology(0),"foundation technology could not be funded");
            require(stages.planning_.funds_eur==1'980'000 && stages.planning_.technologies[0],"first customer report did not discount its related technology by 20%");
            // First-time income, one-click completion, and reputation must
            // be deterministic and derived from the existing save fields.
            require(stages.planning_contract_available(2)==false,"materials contract unlocked before structures research");

            stages.planning_.view=PlanningView::Technology;stages.input_={};stages.render(stage_ui);bool module_tech=false,mission_tech=false,mission_header=false;int tree_lines=0;for(const auto& d:stage_ui.draws){if(d.type=='T'){module_tech|=d.text.find("STAGE-2 MODULE COMPETENCE")!=std::string::npos;mission_tech|=d.text.find("ROBOTIC LUNAR BASE")!=std::string::npos;mission_header|=d.text.find("MISSION READY")!=std::string::npos;}if(d.type=='L')++tree_lines;}
            require(module_tech&&mission_tech,"Stage 1 tech tree does not expose its Stage-2 culmination");
            require(mission_header && tree_lines>=30,"Stage 1 technology display is not rendered as a connected dependency tree");

            // Node map is now navigable and keeps research controls out of the
            // camera/world simulation. The UI must allow zoom, overview and
            // inspection at the same scale as the other Stage-1 screens.
            stage_ui.button("FIT ALL");stage_ui.button("FOCUS");
            const float original_zoom=stages.planning_tech_zoom_;
            auto fit_button=stage_ui.button("FIT ALL");
            stages.input_={};stages.input_.mouse_x=(int)std::lround((fit_button.x+fit_button.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((fit_button.y+fit_button.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_tech_zoom_<original_zoom,"Fit All did not zoom research network to show all eight levels");
            stage_ui.check_bounds("Stage 1 research overview");
            stages.input_={};stages.planning_tech_selected_=1;stages.render(stage_ui);
            auto focus_button=stage_ui.button("FOCUS");
            stages.input_.mouse_x=(int)std::lround((focus_button.x+focus_button.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((focus_button.y+focus_button.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_tech_zoom_>=0.78f,"Focus did not restore a readable research-node size");
            stages.input_={};stages.render(stage_ui);
            auto unlock_button=stage_ui.button("UNLOCK TECHNOLOGY");
            stages.input_.mouse_x=(int)std::lround((unlock_button.x+unlock_button.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((unlock_button.y+unlock_button.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.technologies[1] && stages.planning_.funds_eur==1'430'000,
                    "Research inspector failed to fund and unlock the selected available technology");
            // First-time delivery premium +20%, and investor award for three
            // unique completed customers. No customer contract can replay.
            const auto cash_before_client2=stages.planning_.funds_eur;
            require(stages.accept_planning_contract(2),"material customer not available after structures research");
            const auto after_signing_client2=stages.planning_.funds_eur;
            require(after_signing_client2==cash_before_client2+450'000,"wrong material contract advance");
            require(stages.advance_planning_to_next_report(),"cannot advance to next report");
            require(stages.planning_.time_speed==0 && stages.planning_.contracts[2].progress_days==30.0,
                    "next report control did not stop exactly at the nearest deadline");
            require(stages.planning_.funds_eur==after_signing_client2,"time advance paid customer without submission");
            require(stages.deliver_planning_contract(2),"cannot deliver completed material job");
            require(stages.planning_.funds_eur==after_signing_client2+1'260'000,"first-time 20% customer premium not applied");
            stages.planning_.technologies[3]=1; // later contract's GNC prerequisite
            require(stages.accept_planning_contract(1),"second technical customer unavailable");
            const auto after_signing_client1=stages.planning_.funds_eur;
            require(stages.advance_planning_to_next_report() && stages.planning_.contracts[1].progress_days==28.0,
                    "new contract timing / one-click advance incorrect");
            require(stages.deliver_planning_contract(1),"second technical report could not be submitted");
            require(stages.planning_.funds_eur==after_signing_client1+800'000+160'000+750'000,
                    "third new customer did not receive one-time investor milestone and first-time premium");
            const auto funds_after_delivery=stages.planning_.funds_eur;
            require(stages.planning_.contracts[1].status==PlanningContractStatus::Completed &&
                    !stages.planning_contract_available(1) && !stages.accept_planning_contract(1) &&
                    !stages.deliver_planning_contract(1) && stages.planning_.funds_eur==funds_after_delivery,
                    "completed customer contract can be repeated or paid twice");
            // Legacy schema-29 saves could have already delivered a repeatable
            // contract but stored the status as Available. The completed_count
            // guard must reject it even before save/load migration.
            stages.planning_.contracts[1].status=PlanningContractStatus::Available;
            require(!stages.planning_contract_available(1) && !stages.accept_planning_contract(1),
                    "legacy delivered/repeatable contract reopened");
            stages.planning_.contracts[1].status=PlanningContractStatus::Completed;
            // Leaving Mission Prep and returning via the tab must keep the
            // last facility and full rocket/lander/cargo configuration.
            stages.planning_.view=PlanningView::Mission;
            stages.planning_.mission_step=MissionPreparationStep::Plan;
            stages.planning_.mission_facility=MissionFacility::MissionControl;
            stages.planning_.mission_rocket=1;
            stages.planning_.mission_lander=1;
            stages.planning_.mission_cargo.food+=600;
            const auto expected_cargo=stages.planning_.mission_cargo.total_amount();
            stages.input_={};stages.render(stage_ui);
            const auto site_tab=stage_ui.button("COMPANY SITE");
            stages.input_.mouse_x=(int)std::lround((site_tab.x+site_tab.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((site_tab.y+site_tab.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.view==PlanningView::Site,"mission site navigation failed");
            stages.input_={};stages.render(stage_ui);
            const auto mission_tab=stage_ui.button("MISSION PREP");
            stages.input_.mouse_x=(int)std::lround((mission_tab.x+mission_tab.w*0.5f)*stage_ui.scale);
            stages.input_.mouse_y=(int)std::lround((mission_tab.y+mission_tab.h*0.5f)*stage_ui.scale);
            stages.input_.left_pressed=true;stages.render(stage_ui);
            require(stages.planning_.view==PlanningView::Mission &&
                    stages.planning_.mission_facility==MissionFacility::MissionControl &&
                    stages.planning_.mission_step==MissionPreparationStep::Plan &&
                    stages.planning_.mission_rocket==1 && stages.planning_.mission_lander==1 &&
                    stages.planning_.mission_cargo.total_amount()==expected_cargo,
                    "returning from mission planning discarded its configuration / current facility");
            // Restore the untouched mission fixture for the ordered gate
            // regression below. The navigation case intentionally changed it.
            stages.planning_.mission_step=MissionPreparationStep::Research;
            stages.planning_.mission_facility=MissionFacility::Overview;
            stages.planning_.mission_rocket=0;
            stages.planning_.mission_lander=0;
            stages.planning_.mission_cargo={};
            stages.input_={};stages.render(stage_ui);

            stages.input_={};stages.render(stage_ui);stage_ui.check_bounds("Stage 1 research inspector");


            stages.planning_.view=PlanningView::Mission;stages.planning_.mission_facility=MissionFacility::Overview;stages.render(stage_ui);
            bool mission_title=false,workflow=false,research_gate=false;
            for(const auto& d:stage_ui.draws)if(d.type=='T'){
                mission_title|=d.text.find("MISSION PREPARATION")!=std::string::npos;
                workflow|=d.text.find("FINAL CONFIRM")!=std::string::npos;
                research_gate|=d.text.find("CURRENT GATE: RESEARCH")!=std::string::npos;
            }
            require(mission_title&&workflow&&research_gate,"Stage 1 facility workflow is incomplete");
            require(stages.planning_.mission_cargo.empty(),"new Stage 1 mission did not start with an empty payload");
            require(stages.planning_mission_payload_kg()<=stages.planning_mission_payload_capacity_kg(),"default rocket/lander cannot carry the mandatory minimum cargo");
            require(!stages.complete_planning_mission_step(),"mission gate advanced without opening the owning facility");
            stages.planning_.technologies.fill(1);stages.planning_.funds_eur=100'000'000;

            stages.planning_.mission_facility=MissionFacility::ResearchEngineering;
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Design,"research gate did not advance to design");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Test,"design gate did not advance to test");
            stages.planning_.mission_facility=MissionFacility::ResearchEngineering;
            require(!stages.complete_planning_mission_step(),"test gate advanced from the wrong facility");
            stages.planning_.mission_facility=MissionFacility::PropulsionTest;
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Refine,"test gate did not advance to refine");
            require(stages.planning_.mission_facility==MissionFacility::ResearchEngineering,"test completion did not route findings back to Research & Engineering");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Plan,"refine gate did not advance to plan");
            require(stages.planning_.mission_facility==MissionFacility::MissionControl,"refinement did not route to Mission Control");

            stages.input_={};stages.render(stage_ui);
            const auto press_mission=[&](const char* label){
                const auto box=stage_ui.button(label);
                stages.input_.mouse_x=(int)std::lround((box.x+box.w*.5f)*stage_ui.scale);
                stages.input_.mouse_y=(int)std::lround((box.y+box.h*.5f)*stage_ui.scale);
                stages.input_.left_pressed=true;
                stages.render(stage_ui);
                stages.input_={};stages.render(stage_ui);
            };
            press_mission("SEEDS");
            require(stages.planning_seed_page_ && stages.planning_.mission_seeds==kStarterCropSeeds,
                    "mission seed manifest is missing or has wrong starter quantities");
            press_mission("+");
            require(stages.planning_.mission_seeds[0]==4,"mission seed quantity could not be edited");
            const auto selected_seeds=stages.planning_.mission_seeds;
            press_mission("PAYLOAD");

            stages.planning_.mission_structures[(std::size_t)ResourceKind::SolarPanel]=2;
            stages.planning_.mission_structures[(std::size_t)ResourceKind::OxygenTank]=1;
            stages.planning_.mission_habitats[(std::size_t)HabitatKind::AirLock]=1;
            require(!stages.complete_planning_mission_step(),"mission plan accepted a shortfall for selected structures");
            stages.planning_.mission_cargo.water=98;
            stages.planning_add_required_cargo();
            const auto required=stages.planning_structure_requirements();
            require(!required.empty()&&stages.planning_cargo_covers_structures()&&stages.planning_.mission_cargo.water==98,
                    "auto-add did not cover the plan or overwrote manually added supplies");
            const auto added_mass=stages.planning_.mission_cargo.total_amount();
            stages.planning_add_required_cargo();
            require(stages.planning_.mission_cargo.total_amount()==added_mass,"auto-add duplicated the manifest");
            ResourceBundle selected_manifest=stages.planning_.mission_cargo;
            selected_manifest.iron+=321;
            selected_manifest.water+=98;
            selected_manifest.aluminum+=12;
            stages.planning_.mission_cargo=selected_manifest;
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Confirm,"valid mission plan did not advance to confirmation");
            require(stages.planning_.mission_facility==MissionFacility::ProgramOffice,"mission plan did not route to Program & Contracts");
            require(stages.planning_mission_requirements_met(),"confirmed fully researched/funded minimum mission is not fundable");
            // Save and reload *before* launch. Menu navigation and save/load
            // must preserve the already-approved plan, not silently start at
            // Research again or discard the chosen payload and vehicle pair.
            stages.planning_.mission_rocket=1;
            stages.planning_.mission_lander=1;
            stages.planning_.contracts[0].status=PlanningContractStatus::Available; // legacy delivered job
            stages.planning_.contracts.back().status=PlanningContractStatus::Active;
            std::string mid_save_error;
            require(stages.save_game_to_name("stage1-mid-mission",mid_save_error),
                    "mid-mission save failed: "+mid_save_error);
            const auto mid_save=std::filesystem::path(stages.save_game_directory())/"stage1-mid-mission.gmsave";
            GameApp mid_loaded(engine);
            require(mid_loaded.load_game_from_file(mid_save.string(),mid_save_error),
                    "mid-mission load failed: "+mid_save_error);
            require(mid_loaded.active_game_phase_==GameApp::GamePhase::Planning &&
                    mid_loaded.planning_.view==PlanningView::Mission &&
                    mid_loaded.planning_.mission_step==MissionPreparationStep::Confirm &&
                    mid_loaded.planning_.mission_facility==MissionFacility::ProgramOffice &&
                    mid_loaded.planning_.mission_rocket==1 && mid_loaded.planning_.mission_lander==1 &&
                    mid_loaded.planning_.mission_cargo.total_amount()==selected_manifest.total_amount() &&
                    mid_loaded.planning_.mission_seeds==selected_seeds &&
                    mid_loaded.planning_.mission_habitats==stages.planning_.mission_habitats &&
                    mid_loaded.planning_.mission_structures==stages.planning_.mission_structures &&
                    !mid_loaded.planning_.mission_paid,
                    "saved mission preparation/facility/vehicle/manifest was lost when loading");
            require(mid_loaded.planning_.contracts[0].status==PlanningContractStatus::Completed &&
                    !mid_loaded.planning_contract_available(0),
                    "legacy previously delivered contract was not migrated to permanent Completed");
            // The old catalog's lunar sponsorship was at legacy ID 11.
            GameApp migrated(engine);
            std::string legacy_error;
            require(migrated.load_game_from_file(GAME_LEGACY_PLANNING_FIXTURE,legacy_error),
                    "schema-29 contract migration failed: "+legacy_error);
            require(migrated.planning_.contracts[0].status==PlanningContractStatus::Completed &&
                    migrated.planning_.contracts[kPlanningContractCount-1].status==PlanningContractStatus::Active &&
                    migrated.planning_.contracts[11].status==PlanningContractStatus::Available &&
                    migrated.planning_.mission_seeds==kStarterCropSeeds,
                    "old lunar contract not remapped to last slot or old customer status not kept");
            std::error_code mid_remove_error;std::filesystem::remove(mid_save,mid_remove_error);
            stages.planning_.contracts.back().status=PlanningContractStatus::Available;
            stages.planning_.contracts[0].status=PlanningContractStatus::Completed;
            require(kPlanningContractCount==32,"new commercial project catalog size changed");
            GameApp added_customers(engine);
            added_customers.reset_planning_game();
            require(added_customers.accept_planning_contract(11) &&
                    added_customers.accept_planning_contract(12),
                    "new introductory contracts must be accessible without research");
            require(added_customers.advance_planning_to_next_report() &&
                    added_customers.deliver_planning_contract(11),
                    "new early contract cannot be delivered");
            require(!added_customers.accept_planning_contract(11),
                    "new commercial contract can be repeated after delivery");
            // Conservative prelaunch audit: total unconditional signing and
            // completion payouts across the catalog, excluding launch sponsor,
            // exceed all undiscounted research plus the most costly rocket,
            // lander and the actual minimum Stage-2 payload procurement.
            stages.planning_.mission_rocket=2;
            stages.planning_.mission_lander=2;
            const auto maximum_mission_cost=stages.planning_mission_cost_eur();
            constexpr std::int64_t all_research_cost=21'850'000;
            constexpr std::int64_t guaranteed_commercial_income=94'350'000;
            require(guaranteed_commercial_income>all_research_cost+maximum_mission_cost,
                    "one-time contract proceeds cannot finance full research and the most expensive minimum mission");
            stages.planning_.mission_rocket=1;
            stages.planning_.mission_lander=1;
            const auto mission_cost=stages.planning_mission_cost_eur();
            const auto funds_before_mission=stages.planning_.funds_eur;
            require(stages.purchase_planning_mission(),"valid lunar mission could not be authorized/paid");
            require(stages.planning_.mission_paid && stages.planning_.funds_eur==funds_before_mission-mission_cost,"mission authorization did not deduct the calculated price exactly once");
            require(stages.planning_.mission_step==MissionPreparationStep::Build && stages.planning_.mission_facility==MissionFacility::VehicleAssembly,"funding did not release the vehicle build gate");
            require(!stages.purchase_planning_mission(),"already-paid mission could be charged twice");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Integrate,"vehicle build did not advance to cargo integration");
            require(stages.planning_.mission_facility==MissionFacility::CargoIntegration,"vehicle build did not route to Cargo Integration");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::FinalConfirm,"cargo integration did not advance to final readiness");
            require(stages.planning_.mission_facility==MissionFacility::Communications,"cargo integration did not route to Communications");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Launch,"final readiness did not advance to launch");
            require(stages.planning_.mission_facility==MissionFacility::LaunchComplex,"final readiness did not route to Launch Complex");
            // Lunar customer sponsorship is tied to an actual launch, never
            // an artificial timer. Deposit on award; final fee on launch.
            require(stages.accept_planning_contract(kPlanningContractCount-1),"lunar launch customer could not sponsor the mission");
            const auto before_launch=stages.planning_.funds_eur;
            stages.update_planning_game(500.0f);
            require(stages.planning_.contracts[kPlanningContractCount-1].status==PlanningContractStatus::Active && stages.planning_.funds_eur==before_launch,
                    "lunar cargo delivery contract settled on time rather than actual launch");
            require(stages.complete_planning_mission_step() && stages.planning_.mission_step==MissionPreparationStep::Complete,"launch did not complete the Stage-1 mission workflow");
            require(stages.active_game_phase_==GameApp::GamePhase::RobotPreparation && stages.screen_==GameApp::Screen::Playfield,
                    "lunar launch did not start Stage-2 robotic preparation");
            require(stages.active_game_mode_==GameApp::NewGameMode::Career && stages.playfield_.size()==128,
                    "campaign launch did not retain Career mode and initialize a normal lunar playfield");
            require(stages.lander_.phase==LanderPhase::Approach && stages.builder_robots_.empty() && stages.ground_resource_piles_.empty() && stages.resources_.amounts().empty(),
                    "launch must keep the paid cargo and rovers aboard during descent");
            require(stages.lander_.seeds_aboard==selected_seeds && seed_packet_total(stages.colony_seeds_)==0,
                    "career lander did not load the Stage 1 seed manifest");
            require(stages.prepare_default_landing(),"default landing/stash do not fit");
            stages.update_lunar_lander(200);
            require(stages.lander_.phase==LanderPhase::Landed && stages.lander_.awaiting_stash &&
                    !stages.unload_lunar_lander(),"mission cargo could unload before post-touchdown stash placement");
            require(stages.prepare_default_landing() && stages.unload_lunar_lander(),
                    "lander did not unload after setting a stash");
            require(stages.colony_seeds_==selected_seeds && seed_packet_total(stages.lander_.seeds_aboard)==0,
                    "career landing did not deliver the selected seeds");
            stages.update_lunar_lander(10000);
            require(stages.lander_.phase==LanderPhase::Ready && stages.builder_robots_.size()==2 && stages.placed_habitats_.empty() && stages.placed_resources_.empty(),
                    "lander unloading did not commission the fresh robotic stage");
            require(stages.planning_.funds_eur==before_launch+7'500'000 && stages.planning_.contracts[kPlanningContractCount-1].status==PlanningContractStatus::Completed,
                    "actual launch did not settle the lunar customer contract exactly once");
            for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                const auto material=(MaterialResource)ri;
                require(stages.resources_.amount(material)==selected_manifest.amount(material),
                        "Stage-2 inventory does not equal the funded Stage-1 resource manifest");
                std::int64_t pallet_quantity=0;
                for(const auto& pallet:stages.ground_resource_piles_)if(pallet.resource==material)pallet_quantity+=pallet.amount;
                require(pallet_quantity==selected_manifest.amount(material),"Stage-2 pallet allocation lost or duplicated the shipped resource");
            }
            const auto cash_after_launch=stages.planning_.funds_eur;
            require(!stages.complete_planning_mission_step() && !stages.purchase_planning_mission() && !stages.planning_contract_available(kPlanningContractCount-1),
                    "completed lunar mission could be repeated or funded again");
            require(stages.planning_.funds_eur==cash_after_launch,"repeated launch generated duplicate funds");
            std::cout<<"One-time lunar campaign launch deployed "<<selected_manifest.total_amount()<<" kg of funded supplies into Stage 2; paid cost "<<mission_cost<<" EUR\n";

            std::string planning_save_error;require(stages.save_game_to_name("planning-ui-regression",planning_save_error),"Stage-2 campaign save failed: "+planning_save_error);
            const auto planning_save=std::filesystem::path(stages.save_game_directory())/"planning-ui-regression.gmsave";
            GameApp planning_loaded(engine);require(planning_loaded.load_game_from_file(planning_save.string(),planning_save_error),"Stage 1 load failed: "+planning_save_error);
            require(planning_loaded.active_game_phase_==GameApp::GamePhase::RobotPreparation && planning_loaded.planning_.funds_eur==stages.planning_.funds_eur,"Stage-2 campaign phase/funds did not survive save/load");
            require(planning_loaded.active_game_mode_==GameApp::NewGameMode::Career,"Career save/load did not preserve game mode");
            require(planning_loaded.planning_.technologies[0] && planning_loaded.planning_.contracts[0].completed_count==1,"Stage 1 technology/contract history did not survive save/load");
            require(planning_loaded.planning_.mission_paid && planning_loaded.planning_.mission_paid_cost_eur==mission_cost,"Stage 1 paid mission did not survive save/load");
            require(planning_loaded.planning_.mission_cargo.total_amount()==selected_manifest.total_amount(),"funded mission manifest did not survive save/load");
            for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                const auto material=(MaterialResource)ri;
                require(planning_loaded.resources_.amount(material)==selected_manifest.amount(material),
                        "Stage-2 saved inventory no longer matches shipped cargo");
            }
            require(planning_loaded.planning_.mission_step==MissionPreparationStep::Complete,"Stage 1 facility workflow state did not survive save/load");
            std::error_code planning_remove_error;std::filesystem::remove(planning_save,planning_remove_error);
            // A legacy completed Stage-1 save must finish deploying into Stage 2
            // when loaded rather than leaving the launched rocket on Earth.
            GameApp legacy_launched(engine);
            legacy_launched.playfield_.rebuild(128);
            legacy_launched.camera_.reset(128);
            legacy_launched.reset_planning_game();
            legacy_launched.planning_=stages.planning_;
            legacy_launched.active_game_phase_=GameApp::GamePhase::Planning;
            legacy_launched.active_game_mode_=GameApp::NewGameMode::Career;
            legacy_launched.game_session_active_=true;
            require(legacy_launched.save_game_to_name("legacy-stage1-launched",planning_save_error),"legacy launched save fixture failed");
            const auto legacy_save=std::filesystem::path(legacy_launched.save_game_directory())/"legacy-stage1-launched.gmsave";
            GameApp legacy_loaded(engine);
            require(legacy_loaded.load_game_from_file(legacy_save.string(),planning_save_error),"legacy launched save load failed: "+planning_save_error);
            require(legacy_loaded.active_game_phase_==GameApp::GamePhase::RobotPreparation &&
                    legacy_loaded.lander_.remaining.iron==selected_manifest.iron && legacy_loaded.resources_.amounts().empty() &&
                    legacy_loaded.planning_.mission_step==MissionPreparationStep::Complete,
                    "previously launched Stage-1 save was not migrated into lunar Stage 2");
            std::filesystem::remove(legacy_save,planning_remove_error);
            // Free Play is an actual UI choice, not just a hidden phase flag.
            GameApp sandbox_stage(engine);setup(sandbox_stage,2,2);
            sandbox_stage.render(stage_ui);
            const auto free_button=stage_ui.button("[ ] FREE PLAY");
            sandbox_stage.input_.mouse_x=(int)std::lround((free_button.x+free_button.w*0.5f)*stage_ui.scale);
            sandbox_stage.input_.mouse_y=(int)std::lround((free_button.y+free_button.h*0.5f)*stage_ui.scale);
            sandbox_stage.input_.left_pressed=true;sandbox_stage.render(stage_ui);
            require(sandbox_stage.new_game_mode_selection_==GameApp::NewGameMode::FreePlay,"Free Play selection missed its UI button");
            sandbox_stage.input_={};sandbox_stage.render(stage_ui);
            const auto robot_button=stage_ui.button("2  ROBOT PREPARATION");
            sandbox_stage.input_.mouse_x=(int)std::lround((robot_button.x+robot_button.w*0.5f)*stage_ui.scale);
            sandbox_stage.input_.mouse_y=(int)std::lround((robot_button.y+robot_button.h*0.5f)*stage_ui.scale);
            sandbox_stage.input_.left_pressed=true;sandbox_stage.render(stage_ui);
            require(sandbox_stage.new_game_phase_selection_==GameApp::GamePhase::RobotPreparation,"Free Play could not select Stage 2");
            sandbox_stage.input_={};sandbox_stage.render(stage_ui);
            const auto robot_create=stage_ui.button("CREATE");
            sandbox_stage.input_.mouse_x=(int)std::lround((robot_create.x+robot_create.w*0.5f)*stage_ui.scale);
            sandbox_stage.input_.mouse_y=(int)std::lround((robot_create.y+robot_create.h*0.5f)*stage_ui.scale);
            sandbox_stage.input_.left_pressed=true;sandbox_stage.render(stage_ui);
            require(sandbox_stage.active_game_mode_==GameApp::NewGameMode::FreePlay &&
                    sandbox_stage.active_game_phase_==GameApp::GamePhase::RobotPreparation,
                    "Stage-2 Free Play failed to start its independent robot simulation");
            require(sandbox_stage.lander_.manifest.iron==sandbox_stage.starter_resource_reserve().iron && sandbox_stage.resources_.amounts().empty(),
                    "Free Play Stage-2 stock was overwritten by Career mission cargo");
            GameApp sandbox(engine);
            setup(sandbox,2,2);
            sandbox.new_game_mode_selection_=GameApp::NewGameMode::FreePlay;
            sandbox.new_game_phase_selection_=GameApp::GamePhase::RobotPreparation;
            const auto click_menu=[&](const char* label){
                sandbox.input_={};
                sandbox.render(stage_ui);
                const auto box=stage_ui.button(label);
                sandbox.input_.mouse_x=(int)std::lround((box.x+box.w*.5f)*stage_ui.scale);
                sandbox.input_.mouse_y=(int)std::lround((box.y+box.h*.5f)*stage_ui.scale);
                sandbox.input_.left_pressed=true;
                sandbox.render(stage_ui);
                sandbox.input_={};
            };
            sandbox.debug_frame_enabled_=false;
            click_menu("3  DEBUG MODES");
            require(sandbox.new_game_phase_selection_==GameApp::GamePhase::RobotPreparation,
                    "Stage 3 was selectable with debug off");
            sandbox.debug_frame_enabled_=true;
            click_menu("3  DEBUG MODES");
            click_menu("CONTINUE");
            require(sandbox.screen_==GameApp::Screen::Stage3Modes && !sandbox.game_session_active_,
                    "Stage 3 continued directly into a session");
            click_menu("BACK");
            require(sandbox.screen_==GameApp::Screen::NewGame,"Stage 3 back did not return to stage selection");
            click_menu("CONTINUE");
            click_menu("SANDBOX MODE");
            require(sandbox.screen_==GameApp::Screen::Playfield &&
                    sandbox.active_game_phase_==GameApp::GamePhase::Debug && sandbox.stage3_mode_==GameApp::Stage3Mode::Sandbox &&
                    sandbox.active_game_mode_==GameApp::NewGameMode::FreePlay &&
                    sandbox.debug_mode_active() && sandbox.playfield_.size()==128,
                    "Stage 3 did not start an independent debug sandbox");
            require(sandbox.placed_habitats_.empty() && sandbox.placed_resources_.empty() &&
                    sandbox.corridor_network_.connections().empty() && sandbox.utility_network_.connections().empty() &&
                    sandbox.walkway_network_.connections().empty() && sandbox.builder_robots_.empty() &&
                    sandbox.ground_resource_piles_.empty() && sandbox.resources_.amounts().empty() && !sandbox.lander_.active(),
                    "Stage 3 started with a template or starter assets");

            sandbox.camera_.restore({105.0f,75.0f,0.0f,{}});
            const auto select_build=[&](const char* id){
                const auto& items=sandbox.build_catalog_.items();
                const auto item=std::find_if(items.begin(),items.end(),[&](const auto& candidate){return candidate.id==id;});
                require(item!=items.end(),std::string("missing sandbox tool ")+id);
                sandbox.clear_world_selection();
                sandbox.build_catalog_.set_category(item->category);
                sandbox.build_catalog_.select((int)(item-items.begin()));
            };
            const auto click_world=[&](float x,float z,bool shift=false){
                const auto vp=sandbox.camera_.view_projection((float)stage_ui.w/stage_ui.h);
                const auto& m=vp.m;
                const float w=m[3]*x+m[11]*z+m[15];
                sandbox.input_.mouse_x=(int)std::lround((.5f+(m[0]*x+m[8]*z+m[12])/(2*w))*stage_ui.w);
                sandbox.input_.mouse_y=(int)std::lround((.5f-(m[1]*x+m[9]*z+m[13])/(2*w))*stage_ui.h);
                sandbox.input_.left_pressed=true;
                sandbox.input_.key_shift_down=shift;
                sandbox.render(stage_ui);
                sandbox.input_={};
            };
            select_build("crew_quarters");
            sandbox.placement_rotation_degrees_=90.f;
            sandbox.placement_habitat_size_=HabitatSizeTier::Standard;
            click_world(25,-20,true);
            require(sandbox.placed_habitats_.size()==1 && sandbox.build_catalog_.selected() &&
                    sandbox.placement_rotation_degrees_==90.f && sandbox.placement_habitat_size_==HabitatSizeTier::Standard,
                    "Shift placement did not retain the blueprint, rotation and size");
            click_world(25,14);
            require(sandbox.placed_habitats_.size()==2 && !sandbox.build_catalog_.selected() &&
                    sandbox.placed_habitats_.back().rotation_degrees==90.f &&
                    sandbox.placed_habitats_.back().size==HabitatSizeTier::Standard,
                    "second placement did not use the retained settings or clear the tool");
            sandbox.placed_habitats_.pop_back();
            select_build("storage_dome");click_world(-25,22);
            select_build("corridor");click_world(10,8);click_world(32,8);
            select_build("supply_channel");click_world(-15,-17);click_world(5,-17);
            select_build("walkway");click_world(-6,30);click_world(12,30);
            require(sandbox.placed_habitats_.size()==1 && sandbox.placed_resources_.size()==1 &&
                    sandbox.corridor_network_.connections().size()==1 && sandbox.utility_network_.connections().size()==1 &&
                    sandbox.walkway_network_.connections().size()==1,"a Stage 3 placement tool did not commit");
            require(sandbox.add_storage_gate_at(0,180),"Stage 3 storage gate did not place");
            const auto built_free=[](const auto& item){return item.construction.complete() && item.resource_plan.total().empty();};
            require(built_free(sandbox.placed_habitats_.front()) && built_free(sandbox.placed_resources_.front()) &&
                    built_free(sandbox.placed_resources_.back()) && built_free(sandbox.corridor_network_.connections().front()) &&
                    built_free(sandbox.utility_network_.connections().front()) && built_free(sandbox.walkway_network_.connections().front()) &&
                    sandbox.resources_.amounts().empty(),"Stage 3 used construction sequences or charged for placement");

            // A queued blueprint in an older Stage 3 save retains its invested
            // materials when the sandbox completes it on load.
            auto legacy=sandbox.placed_habitats_.front();
            legacy.position={25,0,-43};
            legacy.placement_sequence=sandbox.next_blueprint_placement_sequence_++;
            legacy.construction.step=1;
            legacy.resource_plan.steps[0].iron=12;
            legacy.resource_plan.steps[1].iron=20;
            sandbox.placed_habitats_.push_back(legacy);
            std::string error;
            require(sandbox.save_game_to_name("stage3-sandbox-regression",error),"Stage 3 save failed: "+error);
            const auto save=std::filesystem::path(sandbox.save_game_directory())/"stage3-sandbox-regression.gmsave";
            GameApp loaded(engine);
            require(loaded.load_game_from_file(save.string(),error),"Stage 3 load failed: "+error);
            std::filesystem::remove(save);
            require(loaded.active_game_phase_==GameApp::GamePhase::Debug && loaded.stage3_mode_==GameApp::Stage3Mode::Sandbox && loaded.debug_mode_active() &&
                    loaded.placed_habitats_.size()==2 && built_free(loaded.placed_habitats_.front()) &&
                    loaded.placed_habitats_.back().construction.complete() &&
                    loaded.placed_habitats_.back().resource_plan.total().iron==12,
                    "Stage 3 save lost placements or migrated material costs incorrectly");
            // Return to the selector while a live sandbox exists. A missing
            // template must leave that session intact and allow another choice.
            sandbox.screen_=GameApp::Screen::Stage3Modes;
            const bool template_exists=std::filesystem::is_regular_file(sandbox.debug_blueprint_template_path());
#ifdef GAME_EXPECT_STAGED_DEBUG_TEMPLATE
            require(template_exists,"Debug scenario template is missing from the runtime directory");
#endif
            click_menu("TEMPLATE MODE");
            if(template_exists){
                GameApp blueprint_start(engine);
                blueprint_start.new_game_phase_selection_=GameApp::GamePhase::RobotPreparation;
                blueprint_start.debug_start_template_enabled_=true;
                blueprint_start.start_new_game();
                require(blueprint_start.screen_==GameApp::Screen::Playfield &&
                        blueprint_start.placed_resources_.size()==11 && blueprint_start.utility_network_.connections().size()==11,
                        "Stage 2 did not load the saved blueprint layout");
                for(const auto& link:blueprint_start.utility_network_.connections())
                    require(link.construction.step==0 && !link.mesh.shell.empty(),"template utility did not rebuild as a blueprint");

                require(sandbox.screen_==GameApp::Screen::Playfield &&
                        sandbox.stage3_mode_==GameApp::Stage3Mode::Template &&
                        sandbox.placed_resources_.size()==11 && sandbox.utility_network_.connections().size()==11 &&
                        !sandbox.builder_robots_.empty() && !sandbox.lander_.active(),
                        "Stage 3 template did not restore the prebuilt lunar base");
                const auto all_built=[](const auto& items){
                    return std::all_of(items.begin(),items.end(),[](const auto& item){return item.construction.complete();});
                };
                require(all_built(sandbox.placed_habitats_) && all_built(sandbox.placed_resources_) &&
                        all_built(sandbox.corridor_network_.connections()) && all_built(sandbox.utility_network_.connections()) &&
                        all_built(sandbox.walkway_network_.connections()),"template started with unfinished objects");
                PlacedHabitat pending;
                sandbox.finalize_placement(pending);
                require(!pending.construction.complete(),"template skipped normal construction for new placements");
                sandbox.placed_habitats_.push_back(pending);
                require(sandbox.save_game_to_name("stage3-sandbox-regression",error),"Stage 3 template save failed: "+error);
                require(loaded.load_game_from_file(save.string(),error),"Stage 3 template load failed: "+error);
                std::filesystem::remove(save);
                require(loaded.stage3_mode_==GameApp::Stage3Mode::Template &&
                        !loaded.placed_habitats_.back().construction.complete(),
                        "Stage 3 template mode or pending blueprint changed across save/load");
            }else{
                require(sandbox.screen_==GameApp::Screen::Stage3Modes && sandbox.game_session_active_ &&
                        sandbox.stage3_mode_==GameApp::Stage3Mode::Sandbox && sandbox.placed_habitats_.size()==2 &&
                        !sandbox.new_game_status_message_.empty(),"failed template load damaged the sandbox session");
            }
            sandbox.screen_=GameApp::Screen::Stage3Modes;
            click_menu("TEST ASTRONAUTS");
            require(sandbox.screen_==GameApp::Screen::Playfield && sandbox.astronaut_test_active() &&
                    sandbox.placed_habitats_.empty() && sandbox.placed_resources_.empty() &&
                    sandbox.builder_robots_.empty() && !sandbox.lander_.active(),
                    "astronaut test did not start on its own field");
            sandbox.render(stage_ui);
            bool test_title=false,build_menu=false;
            for(const auto& draw:stage_ui.draws)if(draw.type=='T'){
                test_title|=draw.text=="STAGE 3 - TEST ASTRONAUTS";
                build_menu|=draw.text=="GAME SPEED";
            }
            require(test_title && build_menu && stage_ui.mesh_triangles>80000,
                    "astronaut test did not render four meshes and the colony controls");
            const auto click_astronaut_scene=[&](GameApp& scene,game::runtime::Vec3 p){
                // Center the world target so the fixed vitals panel cannot intercept the click.
                auto camera=scene.camera_.state();camera.target=p;scene.camera_.restore(camera);
                const auto m=scene.camera_.view_projection((float)stage_ui.w/stage_ui.h).m;
                const float w=m[3]*p.x+m[7]*p.y+m[11]*p.z+m[15];
                game::runtime::InputState mouse;
                mouse.mouse_x=(int)std::lround((.5f+.5f*(m[0]*p.x+m[4]*p.y+m[8]*p.z+m[12])/w)*stage_ui.w);
                mouse.mouse_y=(int)std::lround((.5f-.5f*(m[1]*p.x+m[5]*p.y+m[9]*p.z+m[13])/w)*stage_ui.h);
                mouse.left_pressed=true;
                scene.update(0.f,mouse);
            };
            const auto advance_crew=[&](GameApp& scene,int frames){
                for(int i=0;i<frames;++i){
                    scene.update(.02f,{});
                    require(game::runtime::length(scene.test_astronauts_[0].position-scene.test_astronauts_[1].position)
                            >=kAstronautSeparation-.001f,"astronauts crossed their safety clearance");
                }
            };
            const auto eva_start=sandbox.test_astronauts_[0].position;
            const auto coverall_start=sandbox.test_astronauts_[1].position;
            click_astronaut_scene(sandbox,eva_start+game::runtime::Vec3{0,.95f,0});
            require(sandbox.selected_astronaut_==0,"mouse did not select the EVA astronaut");
            sandbox.render(stage_ui);
            const auto shows_text=[&](const char* caption){
                return std::any_of(stage_ui.draws.begin(),stage_ui.draws.end(),[&](const auto& draw){return draw.text==caption;});
            };
            require(shows_text("AMARA OKAFOR") && shows_text("WORKER / EVA SUIT") &&
                    shows_text("HEALTH 100%") && shows_text("HUNGER 0%") && shows_text("THIRST 0%"),"selected EVA astronaut status was missing");
            stage_ui.check_bounds("astronaut status");

            // This waypoint is on the other side of the stationary crew member.
            click_astronaut_scene(sandbox,{4.6f,0,-.55f});
            require(sandbox.test_astronauts_[0].destination.has_value(),"ground click did not give the EVA astronaut a waypoint");
            const auto eva_goal=*sandbox.test_astronauts_[0].destination;
            game::runtime::InputState right_click;
            right_click.right_pressed=true;
            sandbox.update(0.f,right_click);
            require(sandbox.selected_astronaut_==-1 && sandbox.test_astronauts_[0].destination &&
                    game::runtime::length(*sandbox.test_astronauts_[0].destination-eva_goal)<.001f,
                    "right click must deselect the astronaut and preserve its waypoint");
            advance_crew(sandbox,50);
            const auto eva_saved=sandbox.test_astronauts_[0].position;
            require(game::runtime::length(eva_saved-eva_start)>.2f && sandbox.test_astronauts_[0].stride_phase>.05f &&
                    game::runtime::length(sandbox.test_astronauts_[1].position-coverall_start)<.001f,
                    "walking did not advance the selected mesh animation independently");
            require(sandbox.save_game_to_name("stage3-sandbox-regression",error),"astronaut field save failed: "+error);
            require(loaded.load_game_from_file(save.string(),error) && loaded.astronaut_test_active(),
                    "astronaut test mode did not survive save/load");
            loaded.screen_=GameApp::Screen::Playfield;
            loaded.render(stage_ui);
            require(game::runtime::length(loaded.test_astronauts_[0].position-eva_saved)<.001f &&
                    loaded.test_astronauts_[0].destination.has_value(),
                    "save did not restore the astronaut position and pending waypoint");
            advance_crew(loaded,650);
            require(!loaded.test_astronauts_[0].destination && game::runtime::length(loaded.test_astronauts_[0].position-eva_goal)<.01f &&
                    game::runtime::length(loaded.test_astronauts_[1].position-coverall_start)<.001f,
                    "EVA astronaut did not resume and finish the saved detour");

            click_astronaut_scene(loaded,coverall_start+game::runtime::Vec3{0,.95f,0});
            require(loaded.selected_astronaut_==1,"mouse did not select the coverall astronaut");
            loaded.render(stage_ui);
            require(shows_text("LUCA REYES") && shows_text("ENGINEER / EVA SUIT"),
                    "selected coverall astronaut status was missing");
            click_astronaut_scene(loaded,{6.5f,0,-.55f});
            require(loaded.test_astronauts_[1].destination.has_value(),"ground click did not give the coverall astronaut a waypoint");
            const auto coverall_goal=*loaded.test_astronauts_[1].destination;
            click_astronaut_scene(loaded,loaded.test_astronauts_[0].position+game::runtime::Vec3{0,.95f,0});
            click_astronaut_scene(loaded,{-2.5f,0,-.18f});
            require(loaded.selected_astronaut_==0 && loaded.test_astronauts_[0].destination,
                    "another astronaut's pending walk prevented selecting and ordering the EVA astronaut");
            const auto return_goal=*loaded.test_astronauts_[0].destination;
            advance_crew(loaded,1150);
            require(!loaded.test_astronauts_[0].destination && !loaded.test_astronauts_[1].destination &&
                    game::runtime::length(loaded.test_astronauts_[0].position-return_goal)<.01f &&
                    game::runtime::length(loaded.test_astronauts_[1].position-coverall_goal)<.01f,
                    "astronauts did not finish both crossing waypoints");
            std::filesystem::remove(save);
            sandbox.camera_.restore({105.0f,75.0f,0.0f,{}});
            select_build("air_lock");
            click_world(18.f,0.f);
            require(sandbox.placed_habitats_.size()==1 &&
                    !sandbox.placed_habitats_.front().construction.complete(),
                    "astronaut test did not queue the placed Air Lock");
            advance_crew(sandbox,2400);
            require(sandbox.placed_habitats_.front().construction.complete(),
                    "astronauts did not finish the queued Air Lock");
            click_astronaut_scene(sandbox,sandbox.placed_habitats_.front().position);
            require(sandbox.selection_kind_==GameApp::SelectionKind::Habitat,"Air Lock click did not select the structure");
            const auto before_pan=sandbox.camera_.state().target;
            auto right_press=right_click;
            right_press.right_down=true;
            sandbox.update(0.f,right_press);
            game::runtime::InputState right_drag;
            right_drag.right_down=true;
            right_drag.mouse_dx=20;
            sandbox.update(0.f,right_drag);
            sandbox.update(0.f,{});
            require(sandbox.selection_kind_==GameApp::SelectionKind::Habitat &&
                    game::runtime::length(sandbox.camera_.state().target-before_pan)>.01f,
                    "right drag must pan with the structure still selected");
            sandbox.update(0.f,right_press);
            sandbox.update(0.f,{});
            require(sandbox.selection_kind_==GameApp::SelectionKind::None && sandbox.selected_habitat_==-1,
                    "right click did not clear the structure selection");
            select_build("air_lock");
            sandbox.update(0.f,right_press);
            sandbox.update(0.f,{});
            require(!sandbox.build_catalog_.selected() && sandbox.placed_habitats_.size()==1,
                    "right click did not cancel structure placement");
            std::cout<<"Stage 3 mode selection, crew status, queued building, crossing waypoints and save/load passed"
                     <<(template_exists?" with the restored template\n":" without a template file\n");

            sandbox.screen_=GameApp::Screen::Stage3Modes;
            click_menu("SANDBOX MODE");
            sandbox.camera_.restore({105.0f,75.0f,0.0f,{}});
            select_build("ore_driller");
            click_world(-23,0);
            select_build("processing_facility");
            click_world(22,0);
            select_build("conveyor_belt");
            click_world(22,0);
            require(!sandbox.utility_build_start_,"conveyor placement started at the receiver");
            click_world(-23,0);
            require(sandbox.utility_build_start_.has_value() && stage_ui.belt_preview_lines.size()>=4,
                    "first conveyor element did not render at the snapped driller");
            const auto start_anchor=*sandbox.utility_build_start_;
            const auto port=sandbox.utility_network_.preview_anchor(UtilityKind::ConveyorBelt,start_anchor,{22,0,0},
                sandbox.placed_habitats_,sandbox.habitat_models_,sandbox.placed_resources_,sandbox.resource_models_);
            require(port.has_value(),"first conveyor element has no hull port");
            const auto first_roller=port->position+port->normal*.25f;
            bool attached=false;
            for(std::size_t i=0;i+1<stage_ui.belt_preview_lines.size();i+=2){
                const auto midpoint=(stage_ui.belt_preview_lines[i]+stage_ui.belt_preview_lines[i+1])*.5f;
                attached|=game::runtime::length(midpoint-first_roller)<.01f;
            }
            require(attached,"first conveyor section floats away from the hull");
            click_world(22,0);
            require(sandbox.utility_network_.connections().size()==1 &&
                    sandbox.utility_network_.connections()[0].kind==UtilityKind::ConveyorBelt &&
                    sandbox.utility_network_.connections()[0].construction.complete(),
                    "conveyor build-menu placement did not create the route");
            require(game::runtime::length(sandbox.utility_network_.connections().front().endpoint_a.position-port->position)<.01f,
                    "completed conveyor moved away from the selected source port");
            sandbox.clear_world_selection();
            sandbox.selection_kind_=GameApp::SelectionKind::Resource;
            sandbox.selected_resource_=0;
            sandbox.placed_resources_[0].construction.step=0;
            sandbox.render(stage_ui);
            stage_ui.check_bounds("ore survey overlay");
            for(const auto* name:ore::names){
                bool concentration=false;
                for(const auto& draw:stage_ui.draws)
                    if(draw.type=='T' && draw.text.find(name)!=std::string::npos &&
                       draw.text.find('%')!=std::string::npos)concentration=true;
                require(concentration,"missing ore survey concentration");
            }

            sandbox.screen_=GameApp::Screen::Stage3Modes;
            click_menu("SANDBOX MODE");
            sandbox.camera_.restore({105.0f,75.0f,0.0f,{}});
            select_build("ore_driller");click_world(-38,0);
            select_build("storage_dome");click_world(0,0);
            select_build("landing_pad");click_world(36,0);
            require(sandbox.placed_resources_.size()==3,"conveyor chain structures could not be placed");
            select_build("conveyor_belt");click_world(-38,0);click_world(0,0);
            select_build("conveyor_belt");click_world(4,0);click_world(36,0);
            const auto& belts=sandbox.utility_network_.connections();
            require(belts.size()==2 &&
                    conveyor_route(belts[0].endpoint_a,belts[0].endpoint_b,sandbox.placed_habitats_,sandbox.placed_resources_)==ConveyorRoute::OreStorage &&
                    conveyor_route(belts[1].endpoint_a,belts[1].endpoint_b,sandbox.placed_habitats_,sandbox.placed_resources_)==ConveyorRoute::Trade,
                    "conveyor chain placement through the build menu failed");
            sandbox.render(stage_ui);
            stage_ui.check_bounds("trade conveyor inspector");
        }
        int site_layout_cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp site_ui(engine);setup(site_ui,7,zoom);site_ui.active_game_phase_=GameApp::GamePhase::Planning;site_ui.reset_planning_game();site_ui.planning_.view=PlanningView::Site;
            site_ui.camera_.restore({112.0f,52.0f,325.0f,{0.0f,0.0f,2.0f}});
            Recorder sr;sr.resize((int)size.x,(int)size.y);site_ui.on_resize(sr.w,sr.h);site_ui.render(sr);sr.check_bounds("company site");
            bool title=false,contracts=false,research=false,mission=false;for(const auto& d:sr.draws)if(d.type=='T'){title|=d.text.find("SPACE COMPANY LAUNCH CAMPUS")!=std::string::npos;contracts|=d.text.find("PROGRAM & CONTRACTS")!=std::string::npos;research|=d.text.find("RESEARCH & ENGINEERING")!=std::string::npos;mission|=d.text.find("LAUNCH COMPLEX")!=std::string::npos;}
            require(title&&contracts&&research&&mission,"company-site facilities vanished at supported scaling");++site_layout_cases;
        }
        std::cout<<site_layout_cases<<" company-site resolution/zoom cases passed\n";
        // The Stage-1 camera must remain interactive at the fixed site size.
        {
            CameraController scene_camera;
            scene_camera.restore({138.0f,55.0f,38.0f,{0,0,4}});
            const auto start=scene_camera.state();
            // The Stage-1 scene has a shallower, adaptive clip range. The
            // 0.05..5000 legacy projection caused campus roads/markings to
            // flicker as the camera changed view direction.
            const auto stable=scene_camera.campus_view_projection(16.0f/9.0f);
            const auto legacy=scene_camera.view_projection(16.0f/9.0f);
            float projection_delta=0.0f;
            for(std::size_t j=0;j<16;++j){
                require(std::isfinite(stable.m[j]),"Stage-1 projection contains invalid entries");
                projection_delta+=std::abs(stable.m[j]-legacy.m[j]);
            }
            require(projection_delta>0.01f,"Stage-1 camera still uses the precision-starved legacy frustum");
            for(float zoom_distance:{8.0f,138.0f,640.0f}){
                scene_camera.restore({zoom_distance,44.0f,38.0f,{0,0,4}});
                const auto m=scene_camera.campus_view_projection(16.0f/9.0f);
                for(float value:m.m)require(std::isfinite(value),"campus depth projection fails at extreme zoom");
            }
            scene_camera.restore(start);
            game::runtime::InputState zoom;zoom.wheel=2.0f;
            scene_camera.update(zoom,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(scene_camera.state().distance<start.distance,"Stage-1 mouse wheel zoom is not functional");
            game::runtime::InputState orbit;orbit.middle_down=true;orbit.mouse_dx=36;orbit.mouse_dy=20;
            scene_camera.update(orbit,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(scene_camera.state().azimuth_degrees!=start.azimuth_degrees &&
                    scene_camera.state().elevation_degrees!=start.elevation_degrees,"Stage-1 middle-button orbit failed");
            const auto old_target=scene_camera.state().target;
            game::runtime::InputState pan;pan.right_down=true;pan.mouse_dx=21;pan.mouse_dy=-8;
            scene_camera.update(pan,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(game::runtime::length(scene_camera.state().target-old_target)>0.01f,"Stage-1 right-button pan failed");
        }

        // Contract Office: every scaling mode must expose the actionable
        // customer report workflow, clear research benefit, and valid buttons.
        int contract_layout_cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp board_ui(engine);setup(board_ui,7,zoom);
            board_ui.active_game_phase_=GameApp::GamePhase::Planning;
            board_ui.reset_planning_game();board_ui.planning_.view=PlanningView::Contracts;
            Recorder cr;cr.resize((int)size.x,(int)size.y);board_ui.on_resize(cr.w,cr.h);
            board_ui.render(cr);cr.check_bounds("Stage 1 contract briefs");
            bool title=false,deliverable=false,benefit=false;
            for(const auto& d:cr.draws)if(d.type=='T'){
                title|=d.text.find("CUSTOMER CONTRACTS")!=std::string::npos;
                deliverable|=d.text.find("CUSTOMER DELIVERABLE")!=std::string::npos;
                benefit|=d.text.find("HOW THIS ADVANCES")!=std::string::npos;
            }
            require(title&&deliverable&&benefit,"customer contract does not explain how it supports the moon program");
            const auto accept=cr.button("ACCEPT CONTRACT / RECEIVE ADVANCE");
            board_ui.input_.mouse_x=(int)std::lround((accept.x+accept.w*0.5f)*cr.scale);
            board_ui.input_.mouse_y=(int)std::lround((accept.y+accept.h*0.5f)*cr.scale);
            board_ui.input_.left_pressed=true;board_ui.render(cr);
            require(board_ui.planning_.contracts[0].status==PlanningContractStatus::Active && board_ui.planning_.funds_eur==1'750'000,
                    "customer-project acceptance button did not deposit the advance");
            board_ui.input_={};board_ui.planning_.time_speed=3;board_ui.update_planning_game(1.3f);
            board_ui.render(cr);cr.check_bounds("Stage 1 ready-to-deliver brief");
            const auto deliver=cr.button("SUBMIT REPORT / CLAIM PAYMENT");
            board_ui.input_.mouse_x=(int)std::lround((deliver.x+deliver.w*0.5f)*cr.scale);
            board_ui.input_.mouse_y=(int)std::lround((deliver.y+deliver.h*0.5f)*cr.scale);
            board_ui.input_.left_pressed=true;board_ui.render(cr);
            require(board_ui.planning_.funds_eur==2'300'000 && board_ui.planning_.contracts[0].completed_count==1,
                    "customer report submission did not release the final payment");
            require(board_ui.planning_.contracts[0].status==PlanningContractStatus::Completed &&
                    !board_ui.planning_contract_available(0),
                    "delivered contract did not close permanently");
            board_ui.input_={};board_ui.render(cr);
            bool closed_label=false;
            for(const auto& draw:cr.draws) if(draw.type=='T')
                closed_label|=draw.text.find("COMPLETED - CLOSED")!=std::string::npos;
            require(closed_label,"contract UI did not show its permanent completion status");
            ++contract_layout_cases;
        }
        std::cout<<contract_layout_cases<<" contract-brief resolution/zoom and button delivery cases passed\n";
        // Contract inspector: include every direct research requirement and
        // recursively all upstream prerequisite technologies. Missing items
        // must be genuinely red; completed research genuinely green.
        {
            GameApp requirements(engine);setup(requirements,7,0);
            requirements.active_game_phase_=GameApp::GamePhase::Planning;
            requirements.reset_planning_game();requirements.planning_.view=PlanningView::Contracts;
            requirements.planning_contract_selected_=1;
            requirements.planning_.technologies[0]=1; // systems done, avionics missing
            Recorder sr;sr.resize(1920,1080);requirements.on_resize(sr.w,sr.h);requirements.render(sr);
            auto colored_requirement=[&](const std::string& text,bool red){
                for(const auto& d:sr.draws)if(d.type=='T'&&d.text.find(text)!=std::string::npos)
                    if(red?d.color.r>0.85f&&d.color.g<0.5f&&d.color.b<0.5f:
                           d.color.g>0.8f&&d.color.r<0.55f)return true;
                return false;
            };
            require(colored_requirement("RESEARCHED  [DIRECT]  SYSTEMS ENGINEERING",false),
                    "contract research did not mark completed technology green");
            require(colored_requirement("MISSING  [DIRECT]  AVIONICS + GNC",true),
                    "contract research did not mark missing direct requirement red");
            requirements.planning_.technologies[3]=1;
            requirements.input_={};requirements.render(sr);
            require(colored_requirement("RESEARCHED  [DIRECT]  AVIONICS + GNC",false),
                    "contract research status did not refresh when technology was unlocked");
            requirements.planning_contract_selected_=(int)kPlanningContractCount-1;
            requirements.planning_.technologies.fill(0);
            requirements.input_={};requirements.render(sr);
            bool shows_full_chain=false,shows_earlier_dependency=false;
            for(const auto& d:sr.draws)if(d.type=='T'){
                shows_full_chain|=d.text.find("REQUIRED RESEARCH  0 / 15 COMPLETE")!=std::string::npos;
                shows_earlier_dependency|=d.text.find("[PREREQUISITE]")!=std::string::npos;
            }
            require(shows_full_chain&&shows_earlier_dependency,
                    "lunar contract did not show complete recursive research dependency chain");
            requirements.ui_scale_index_=4;
            requirements.on_resize(sr.w,sr.h);
            requirements.input_={};requirements.render(sr);
            sr.check_bounds("contract research paginated at high UI zoom");
            const auto next=sr.button(">");
            requirements.input_.mouse_x=(int)std::lround((next.x+next.w*0.5f)*sr.scale);
            requirements.input_.mouse_y=(int)std::lround((next.y+next.h*0.5f)*sr.scale);
            requirements.input_.left_pressed=true;
            requirements.render(sr);
            require(requirements.planning_contract_research_page_==1,
                    "contract research pagination button is not interactive at high zoom");
            std::cout<<"contract research red/green and recursive prerequisites passed\n";
        }
        // Double click is an item-level primary action, not a shortcut around
        // contract/research guards. Single clicks must never spend money.
        {
            GameApp contracts(engine);setup(contracts,7,0);
            contracts.active_game_phase_=GameApp::GamePhase::Planning;
            contracts.reset_planning_game();contracts.planning_.view=PlanningView::Contracts;
            Recorder rr;rr.resize(1920,1080);contracts.on_resize(rr.w,rr.h);contracts.render(rr);
            auto click_label=[&](const std::string& label){
                for(const auto& d:rr.draws) if(d.type=='T' && d.text==label){
                    contracts.input_.mouse_x=(int)((d.v[0]+5)*rr.scale);
                    contracts.input_.mouse_y=(int)((d.v[1]+8)*rr.scale);
                    contracts.input_.left_pressed=true;
                    contracts.render(rr);
                    contracts.input_.left_pressed=false;
                    contracts.render(rr);
                    return;
                }
                throw std::runtime_error("contract row not found: "+label);
            };
            click_label("Mission Architecture Study");
            require(contracts.planning_.contracts[0].status==PlanningContractStatus::Available,
                    "single-click accepted a contract without a second click");
            click_label("Mission Architecture Study");
            require(contracts.planning_.contracts[0].status==PlanningContractStatus::Active &&
                    contracts.planning_.funds_eur==1'750'000,
                    "double-click did not sign contract and award advance");
            contracts.update_planning_game(24.0f);
            contracts.input_={};contracts.render(rr);
            click_label("Mission Architecture Study");
            click_label("Mission Architecture Study");
            require(contracts.planning_.contracts[0].completed_count==1 &&
                    contracts.planning_.funds_eur==2'300'000,
                    "double-click did not submit ready contract");
            contracts.planning_contract_page_=1;contracts.input_={};contracts.render(rr);
            click_label("Mission Architecture Study");
            click_label("Mission Architecture Study");
            require(contracts.planning_.contracts[0].completed_count==1 &&
                    contracts.planning_.funds_eur==2'300'000,
                    "double-click paid finished contract twice");

            GameApp research(engine);setup(research,7,0);
            research.active_game_phase_=GameApp::GamePhase::Planning;
            research.reset_planning_game();research.planning_.view=PlanningView::Technology;
            research.planning_tech_zoom_=0.78f;
            research.planning_tech_pan_x_=research.planning_tech_pan_y_=0.0f;
            Recorder tr;tr.resize(1920,1080);research.on_resize(tr.w,tr.h);research.render(tr);
            // First node (systems engineering) appears near the top-left of
            // the graph at this fixed zoom/pan configuration.
            const int tx=112,ty=350;
            auto click_tech=[&](){
                research.input_.mouse_x=tx;research.input_.mouse_y=ty;
                research.input_.left_pressed=true;research.render(tr);
                research.input_.left_pressed=false;research.render(tr);
            };
            const auto cash=research.planning_.funds_eur;
            click_tech();
            require(!research.planning_.technologies[0] && research.planning_.funds_eur==cash,
                    "research single click unexpectedly purchased technology");
            click_tech();
            require(research.planning_.technologies[0] && research.planning_.funds_eur<cash,
                    "research double click failed to unlock technology");
            const auto bought=research.planning_.funds_eur;
            click_tech();click_tech();
            require(research.planning_.funds_eur==bought,
                    "research double click charged completed technology again");
            tr.check_bounds("research double-click inspector");
            std::cout<<"contract/research double-click guards passed\n";
        }

        int mission_layout_cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp mission_ui(engine);setup(mission_ui,7,zoom);mission_ui.active_game_phase_=GameApp::GamePhase::Planning;mission_ui.reset_planning_game();mission_ui.planning_.view=PlanningView::Mission;mission_ui.planning_.mission_facility=MissionFacility::MissionControl;mission_ui.planning_.mission_step=MissionPreparationStep::Plan;
            Recorder mr;mr.resize((int)size.x,(int)size.y);mission_ui.on_resize(mr.w,mr.h);mission_ui.render(mr);mr.check_bounds("mission planning");
            bool title=false,workflow=false,manifest=false;for(const auto& d:mr.draws)if(d.type=='T'){title|=d.text.find("MISSION PREPARATION")!=std::string::npos;workflow|=d.text.find("FINAL CONFIRM")!=std::string::npos;manifest|=d.text.find("PAYLOAD")!=std::string::npos;}
            require(title&&workflow&&manifest,"mission workflow UI vanished at supported scaling");
            const auto click_plan=[&](const char* caption){
                const auto button=mr.button(caption);
                mission_ui.input_.mouse_x=(int)((button.x+button.w*.5f)*mr.scale);
                mission_ui.input_.mouse_y=(int)((button.y+button.h*.5f)*mr.scale);
                mission_ui.input_.left_pressed=true;
                mission_ui.render(mr);
                mission_ui.input_={};
                mission_ui.render(mr);
                mr.check_bounds("mission structure planning");
            };
            click_plan("STRUCTURES");
            click_plan("+");
            click_plan("AUTO-ADD CARGO");
            require(mission_ui.planning_.mission_habitats[(std::size_t)HabitatKind::AirLock]==1&&
                    !mission_ui.planning_.mission_cargo.empty()&&mission_ui.planning_cargo_covers_structures(),
                    "structure list or auto-add input failed");
            ++mission_layout_cases;
        }
        std::cout<<mission_layout_cases<<" mission-planning resolution/zoom cases passed\n";

        GameApp settings(engine);setup(settings,3,2);Recorder r;settings.render(r);const float before=settings.ui_scale_;const auto plus=r.button(">");settings.input_.mouse_x=(int)std::lround((plus.x+plus.w/2)*before);settings.input_.mouse_y=(int)std::lround((plus.y+plus.h/2)*before);settings.input_.left_pressed=true;settings.render(r);
        require(settings.ui_scale_index_==3 && settings.ui_scale_==before,"zoom changed input/drawing transform mid-frame");settings.input_={};settings.render(r);require(settings.ui_scale_>before,"zoom preference did not apply on next frame");
        // Top-down keyboard camera controls: W/S translate along the current
        // ground-forward axis, A/D strafe, and movement must be time based.
        {
            GameApp keyboard(engine);setup(keyboard,7,2);
            keyboard.screen_=GameApp::Screen::Playfield;
            const auto start=keyboard.camera_.target();
            game::runtime::InputState move_w;move_w.key_w_down=true;
            keyboard.update(0.5f,move_w);
            const auto after_w=keyboard.camera_.target();
            require(game::runtime::length(after_w-start)>0.1f,"W did not translate the top-down camera");
            game::runtime::InputState move_d;move_d.key_d_down=true;
            keyboard.update(0.5f,move_d);
            const auto after_d=keyboard.camera_.target();
            require(game::runtime::length(after_d-after_w)>0.1f,"D did not strafe the top-down camera");

            GameApp coarse(engine);setup(coarse,7,2);coarse.screen_=GameApp::Screen::Playfield;
            GameApp fine(engine);setup(fine,7,2);fine.screen_=GameApp::Screen::Playfield;
            for(int i=0;i<20;++i)coarse.update(0.05f,move_w);
            for(int i=0;i<100;++i)fine.update(0.01f,move_w);
            require(game::runtime::length(coarse.camera_.target()-fine.camera_.target())<0.05f,"WASD camera movement depends on frame rate");
        }
        // Invalid corridor placement renders a readable cursor tooltip above
        // the 3D preview. Keep this in the HUD bounds at common small screens.
        {
            GameApp tooltip(engine);setup(tooltip,7,2);
            tooltip.placement_tip_title_="NO CLEAR CORRIDOR ROUTE";
            tooltip.placement_tip_detail_="All tested routes are blocked by structure or corridor clearance. Move an endpoint or create more free space.";
            tooltip.input_.mouse_x=1210;tooltip.input_.mouse_y=650;
            Recorder tr;tr.resize(1280,720);tr.set_2d(tooltip.ui_scale_);
            tooltip.render_placement_tooltip(tr);
            tr.check_bounds("corridor rejection tooltip");
            bool title=false,body=false;
            for(const auto& d:tr.draws)if(d.type=='T'){title|=d.text.find("NO CLEAR CORRIDOR ROUTE")!=std::string::npos;body|=d.text.find("blocked")!=std::string::npos;}
            require(title && body,"corridor rejection tooltip omitted title or reason");

            tooltip.placement_tip_title_="AIR LOCK + CORRIDOR";
            tooltip.placement_tip_detail_.clear();
            tooltip.resources_.restore({});
            ConstructionResourcePlan habitat_cost;
            habitat_cost.steps[0].iron=25;
            GameApp::StructureAttachment attachment{};
            attachment.corridor.emplace();
            attachment.corridor->resource_plan.steps[0].iron=11;
            attachment.corridor->resource_plan.steps[0].aluminum=3;
            tooltip.set_placement_cost(habitat_cost,&attachment);
            tr.draws.clear();
            tooltip.render_placement_tooltip(tr);
            tr.check_bounds("combined placement cost");
            bool iron=false,aluminum=false,shortage=false;
            for(const auto& d:tr.draws)if(d.type=='T'){
                iron|=d.text.find("36 Fe")!=std::string::npos;
                aluminum|=d.text.find("3 Al")!=std::string::npos;
                shortage|=d.text.find("NEEDS MORE MATERIALS")!=std::string::npos;
            }
            require(iron && aluminum && shortage,"placement tooltip omits attached corridor cost or shortage");
        }

        // ESC from the in-game menu resumes the active session. ESC from a
        // sub-menu still returns to the game menu first.
        {
            GameApp menu(engine);setup(menu,0,2);
            menu.game_session_active_=true;
            menu.screen_=GameApp::Screen::MainMenu;
            game::runtime::InputState esc;esc.escape_pressed=true;
            menu.update(0.0f,esc);
            require(menu.screen_==GameApp::Screen::Playfield,"ESC did not resume from the in-game menu");
            menu.screen_=GameApp::Screen::Settings;
            menu.update(0.0f,esc);
            require(menu.screen_==GameApp::Screen::MainMenu,"ESC from an in-game sub-menu did not return to the game menu");
        }
        for(const auto size:std::array<game::runtime::Vec2,3>{{{1280,1024},{3440,1440},{5120,1440}}}){GameApp app(engine);setup(app,10,2);r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("non-16:9 HUD");}
        GameApp catalog(engine);setup(catalog,7,4);Recorder small;small.resize(1280,720);catalog.render(small);
        const auto next=small.button(">");catalog.input_.mouse_x=(int)std::lround((next.x+next.w/2)*small.scale);catalog.input_.mouse_y=(int)std::lround((next.y+next.h/2)*small.scale);catalog.input_.left_pressed=true;catalog.render(small);
        require(catalog.build_catalog_scroll_==1,"fixed-size icon catalog did not page");
        game::runtime::InputState wheel=catalog.input_;wheel.left_pressed=false;wheel.wheel=-1;const auto distance=catalog.camera_.distance();catalog.update(0,wheel);catalog.render(small);
        require(catalog.build_catalog_scroll_==2 && catalog.camera_.distance()==distance,"catalog wheel also zoomed the world");
        for(const auto& caption:{std::string("ON"),std::string(90,'W')}){Recorder button;button.set_2d(1);game::runtime::ui::button(button,{}, {0,0,82,34},caption,{});const auto& text=button.draws.back();require(text.v[0]>=game::runtime::ui::kControlPaddingX-0.01f && text.v[1]>=game::runtime::ui::kControlPaddingY-0.01f && text.v[0]+text.v[2]<=82-game::runtime::ui::kControlPaddingX+0.01f && text.v[1]+text.v[3]<=34-game::runtime::ui::kControlPaddingY+0.01f,"button caption escaped its padding");}
        const auto stroke=game::runtime::ui::line_quad({0,0},{40,0},2);require(std::abs(stroke[1]-stroke[7]-2)<1e-6f,"logical stroke thickness changed");
        const auto metric=game::runtime::ui::text_extent("WW\nW",2);require(metric.x==24 && metric.y==30,"multiline font metrics differ from glyph grid");
        for(char ch:std::string("[]_!?#|"))require(game::runtime::ui::kBitmapFont.count(ch)==1,"UI punctuation glyph missing");
        std::cout<<"475 resolution/zoom/screen cases and "+std::to_string(structure_cases)+" structure warning cases, HD-to-4K proportions, hitboxes, frame-boundary zoom, aspect ratios, button text and shared font/stroke metrics passed\n";
    }
};
}
int main(int argc,char** argv){
    const auto preferences=std::filesystem::temp_directory_path()/
        ("lunar-ui-regression-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".cfg");
#ifdef _WIN32
    _wputenv_s(L"GAME_SETTINGS_CONFIG",L"");
    if(_wputenv_s(L"GAME_GRAPHICS_CONFIG",preferences.c_str())!=0) return 1;
#else
    unsetenv("GAME_SETTINGS_CONFIG");
    if(setenv("GAME_GRAPHICS_CONFIG",preferences.c_str(),1)!=0) return 1;
#endif
    int result=0;
    try{
        const std::string out=argc>1?argv[1]:"";
        if(out=="build-mode"){
            game::UiRegression::build_mode();
            game::UiRegression::recent_saves();
        }
        else if(out=="gameplay-options") game::UiRegression::gameplay_options();
        else if(out=="airlock-ui") game::UiRegression::airlock_status();
        else if(out=="interior-services") game::UiRegression::interior_services();
        else if(out=="interior-systems") game::UiRegression::interior_systems();
        else if(out=="gates-layout-solar") game::UiRegression::gates_layout_solar();
        else if(out=="station-visuals") game::UiRegression::station_visuals();
        else if(out=="fpv-building") game::UiRegression::fpv_building();
        else if(out=="corridor-airlock-fpv") game::UiRegression::corridor_airlock_fpv();
        else if(out=="structural-frames") game::UiRegression::structural_frames();
        else if(out=="player-control") game::UiRegression::player_control();
        else if(out=="runtime-paths") game::UiRegression::runtime_paths();
        else if(out=="fov-interiors") game::UiRegression::fov_and_station_interiors();
        else if(out=="removal-controls") game::UiRegression::removal_controls();
        else if(out=="save-editor") game::UiRegression::save_editor();
        else if(out=="airlock-transfers") game::UiRegression::airlock_transfers();
        else if(out=="fpv-saved-recovery"&&argc>2) game::UiRegression::saved_fpv_recovery(argv[2]);
        else if(out=="fpv-alerts") game::UiRegression::fpv_alerts();
        else if(out=="hatch-interaction") game::UiRegression::hatch_interaction();
        else if(out=="interior-surfaces") game::UiRegression::interior_surfaces();
        else if(out=="fpv-save-probe"&&argc>2) game::UiRegression::fpv_save_probe(argv[2]);
        else if(out=="dome-save-probe"&&argc>2) game::UiRegression::dome_save_probe(argv[2]);
        else if(out=="autosave") game::UiRegression::autosave();
        else if(out=="airlock-access") game::UiRegression::airlock_access();
        else if(out=="battery-doors") game::UiRegression::battery_doors();
        else if(out=="robot-crew") game::UiRegression::robot_crew();
        else if(out=="rover-unfold") game::UiRegression::rover_unfold();
        else if(out=="airlock-factory-fit") game::UiRegression::airlock_factory_fit();
        else if(out=="no-hull-boundaries") game::UiRegression::no_hull_boundaries();
        else if(out=="suit-refill") game::UiRegression::suit_refill();
        else if(out=="habitat-domes"&&argc>2) game::UiRegression::habitat_domes(argv[2]);
        else if(out=="settings-boundary") game::UiRegression::settings_boundary();
        else if(out=="graphics-quality") game::UiRegression::graphics_quality();
        else if(out=="build-order") game::UiRegression::build_order_panel();
        else if(out=="activity-log") game::UiRegression::activity_log_and_blueprint_progress();
        else{
            if(!out.empty()) std::filesystem::create_directories(out);
            game::UiRegression::run(out);
        }
    }catch(const std::exception& e){
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        result=1;
    }
    std::error_code error;
    std::filesystem::remove(preferences,error);
    return result;
}
