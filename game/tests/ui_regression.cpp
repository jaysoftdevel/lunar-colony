#include "game/game_app.hpp"
#include "game/config/preferences.hpp"
#include "game/ui/layout.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/ui_font.hpp"
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
struct Draw {char type;std::array<float,6> v{};std::string text;eng::Color color;};
struct Recorder:eng::Renderer {
    int w=1920,h=1080;float scale=1;bool hud=false;std::vector<Draw> draws;
    eng::GraphicsSettings graphics;
    void set_graphics_settings(const eng::GraphicsSettings& settings) override { graphics=settings; }
    std::vector<eng::Vec3> belt_preview_lines,queue_hover_lines;
    std::size_t mesh_triangles=0;
    bool initialize(void*,std::uintptr_t,int,int)override{return true;}
    void resize(int width,int height)override{w=width;h=height;}
    void begin_frame(eng::Color)override{draws.clear();belt_preview_lines.clear();queue_hover_lines.clear();mesh_triangles=0;hud=false;}void end_frame()override{}
    void set_3d(const eng::Mat4&)override{hud=false;}void set_2d(float s)override{scale=s;hud=true;}
    void draw_lines_3d(const std::vector<eng::Vec3>& points,eng::Color color,float width)override{
        if(width==1.9f && color.r==.34f && color.g==.82f && color.b==1.0f)belt_preview_lines=points;
        if(color.r==.31f && color.g==.94f && color.b==1.f && color.a==1.f)
            queue_hover_lines.insert(queue_hover_lines.end(),points.begin(),points.end());
    }
    void draw_triangles_3d(const std::vector<eng::Vec3>& points,eng::Color)override{mesh_triangles+=points.size()/3;}
    void draw_rect(eng::Rect r,eng::Color c)override{if(hud)draws.push_back({'R',{r.x,r.y,r.w,r.h,scale,0},{},c});}
    void draw_line(eng::Vec2 a,eng::Vec2 b,eng::Color c,float width)override{if(hud)draws.push_back({'L',{a.x,a.y,b.x,b.y,scale,width},{},c});}
    void draw_text(eng::Vec2 p,const std::string& text,float s,eng::Color c)override{if(hud){auto e=eng::ui::text_extent(text,s);draws.push_back({'T',{p.x,p.y,e.x,e.y,scale,s},text,c});}}
    int width()const override{return w;}int height()const override{return h;}bool hardware_accelerated()const override{return false;}
    std::string gpu_vendor()const override{return "TEST";}std::string gpu_renderer()const override{return "SCALING RECORDER";}std::string gpu_version()const override{return "TEST";}
    eng::Rect button(const std::string& caption)const{
        eng::Rect last{};for(const auto& d:draws){if(d.type=='R')last={d.v[0],d.v[1],d.v[2],d.v[3]};if(d.type=='T' && d.text==caption)return last;}
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
        eng::Application engine;GameApp defaults(engine);require(defaults.ui_scale_index_==0 && defaults.ui_scale_==1,"default UI scale is not 100%");require(defaults.playfield_size_==128,"default playfield is not 128 x 128");require(GameApp::kNewGamePlayfieldMin==32 && GameApp::kNewGamePlayfieldMax==1024 && GameApp::kNewGamePlayfieldStep==32,"new-game playfield range is not doubled");require(GameApp::kDebugTemplatePlayfieldSize==128,"debug template playfield is not doubled");Playfield maximum_field;maximum_field.rebuild(GameApp::kNewGamePlayfieldMax);require(maximum_field.size()==1024 && maximum_field.floor_triangles().size()==6,"1024 x 1024 playfield failed to build");int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<eng::Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}})for(int domain=0;domain<5;++domain){
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
            require(debug_r.graphics.path==eng::GraphicsPath::Auto,"enhanced graphics are not the default");
            const auto layout_button=debug_r.button("FULL GRAPHICS");
            debug.input_.mouse_x=(int)((layout_button.x+layout_button.w/2)*debug_r.scale);
            debug.input_.mouse_y=(int)((layout_button.y+layout_button.h/2)*debug_r.scale);
            debug.input_.left_pressed=true;debug.render(debug_r);
            require(debug.world_layout_mode_==GameApp::WorldLayoutMode::Mesh,"debug layout switch did not select mesh mode");
            debug.input_.left_pressed=false;debug.render(debug_r);debug_r.button("MESHED");
            require(debug_r.graphics.path==eng::GraphicsPath::Classic,"debug layout did not select the mesh renderer");
        }
        GameApp app(engine);setup(app,7,2);app.debug_frame_enabled_=true;app.selection_kind_=GameApp::SelectionKind::Robot;app.selected_robot_=0;
        Recorder r;app.render(r);r.check_bounds("debug robot inspector");const auto button=r.button("DEBUG MOVE / PICK DESTINATION");
        app.input_.mouse_x=(int)((button.x+button.w/2)*r.scale);app.input_.mouse_y=(int)((button.y+button.h/2)*r.scale);app.input_.left_pressed=true;app.render(r);require(app.debug_move_pick_robot_==0,"debug move button did not arm destination picker");
        eng::InputState escape;escape.escape_pressed=true;app.update(0,escape);require(app.debug_move_pick_robot_==-1 && app.screen_==GameApp::Screen::Playfield,"escape from move picker left game");
        app.debug_move_pick_robot_=0;eng::InputState click;click.mouse_x=r.w/2;click.mouse_y=r.h/2;click.left_pressed=true;click.left_down=true;
        app.update(0,click);require(app.debug_move_pick_robot_==-1 && app.builder_robots_[0].debug_move_active && app.selection_kind_==GameApp::SelectionKind::Robot,"destination click selected the world instead of issuing debug move");
        require(app.issue_debug_robot_move(0,{12,0,12}),"UI debug order rejected");app.input_={};app.render(r);r.button("CANCEL DEBUG MOVE");
        bool task=false;for(const auto& draw:r.draws)task|=draw.text=="TASK  DEBUG MOVE TO SELECTED PLACE";require(task,"debug destination missing from robot task panel");
        const auto cancel=r.button("CANCEL DEBUG MOVE");app.input_.mouse_x=(int)((cancel.x+cancel.w/2)*r.scale);app.input_.mouse_y=(int)((cancel.y+cancel.h/2)*r.scale);app.input_.left_pressed=true;app.render(r);require(!app.builder_robots_[0].debug_move_active,"cancel debug move failed");
        std::cout<<cases<<" priority control/marker/resolution cases plus debug move/cancel inspector controls passed\n";
    }
    static void airlock_status(){
        eng::Application engine;int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<eng::Vec2,3>{{{1280,720},{1920,1080},{3840,2160}}}){
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
        eng::Application engine;int cases=0;
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:std::array<eng::Vec2,5>{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}})for(int state=0;state<3;++state){
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
        eng::Application engine;
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
        eng::Application engine;
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
        eng::Application engine;int cases=0;
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
        eng::Application engine;int cases=0;
        constexpr std::array<eng::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp app(engine);setup(app,7,zoom);app.debug_frame_enabled_=false;Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("progress tree");
            const auto has=[&](const std::string& text){
                return std::any_of(r.draws.begin(),r.draws.end(),[&](const auto& d){return d.type=='T' && d.text.find(text)!=std::string::npos;});
            };
            require(has("STAGE 1 / Build a solar collector") && !has("Build a battery"),
                    "collapsed progress did not show only the next target");
            const auto collapsed=r.button("+  COLONY PROGRESS");
            eng::InputState input{};
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
        eng::Application engine;GameApp app(engine);setup(app,16,4);
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
        eng::InputState wheel{};
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
        eng::Application engine;GameApp app(engine);setup(app,19,4);
        app.placed_habitats_[0].size=HabitatSizeTier::Big;
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
        eng::Application engine;
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
                    return draw.type=='L' && draw.color.r==.34f && draw.color.g==.84f && draw.color.b==1.0f;
                })==2,"powered-off info icon missing");
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
        using namespace eng;
        Application engine;
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
        using namespace eng;
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
        Application engine;
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
                    r.surface_tint>.10f && !r.saw_grid && r.scene_attempts==0,
                    "Classic graphics did not draw the full playfield with the regolith material");
        }
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
        using namespace eng;
        struct HullRecorder : Recorder {
            std::vector<Vec3> vertices;
            void draw_lunar_surface(const std::vector<Vec3>& points,LunarMaterial,Color) override {
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
        Application engine;
        GameApp app(engine);
        app.graphics_.path=GraphicsPath::Classic;
        require(app.habitat_view_mode_==GameApp::HabitatViewMode::FullHull,"new game did not start with Full Hull");
        HabitatMesh habitat;
        habitat.foundation=layer(1);
        habitat.interior=layer(2);
        habitat.shell=layer(3);
        habitat.glazing=layer(4);
        habitat.edges={{0,5,0},{1,5,0}};
        ResourceMesh resource;
        resource.foundation=habitat.foundation;
        resource.details=habitat.interior;
        resource.body=habitat.shell;
        resource.edges=habitat.edges;
        CorridorConnection corridor;
        corridor.path={{0,1,0},{1,1,0}};
        corridor.mesh={habitat.shell,habitat.interior,habitat.edges};
        corridor.construction.finish();
        app.corridor_network_.restore_connections({corridor});
        app.selection_kind_=GameApp::SelectionKind::Corridor;
        app.selected_corridor_=0;
        HullRecorder r;
        const auto draw_room=[&](float step){
            r.vertices.clear();
            app.render_habitat_mesh(r,habitat,{},0,false,true,true,step);
        };
        draw_room(kConstructionSteps);
        require(r.drew_at(1) && r.drew_at(2) && r.drew_at(3) && r.drew_at(4) && r.drew_at(5),
                "Classic Full Hull lost filled exterior or interior geometry");
        r.set_2d(1);
        app.render_view_selector(r,0,1280);
        const auto none=r.button("NONE");
        require(r.button("FULL HULL").w>0,"Full Hull button missing");
        app.ui_input_.mouse_x=static_cast<int>(none.x+none.w*.5f);
        app.ui_input_.mouse_y=static_cast<int>(none.y+none.h*.5f);
        app.ui_input_.left_pressed=true;
        app.render_view_selector(r,0,1280);
        require(app.habitat_view_mode_==GameApp::HabitatViewMode::None,"None button did not hide the hull");

        for(const auto layout:{GameApp::WorldLayoutMode::FullGraphics,GameApp::WorldLayoutMode::Mesh}){
            app.world_layout_mode_=layout;
            for(const float step:{4.f,static_cast<float>(kConstructionSteps)}){
                draw_room(step);
                require(r.drew_at(1) && r.drew_at(2) && !r.drew_at(3) && !r.drew_at(4) && !r.drew_at(5),
                        "None drew part of the habitat shell or lost the interior");
            }
            for(const auto kind:{ResourceKind::StorageDome,ResourceKind::StorageGate,ResourceKind::RobotWorkshop,ResourceKind::OreDriller}){
                r.vertices.clear();
                app.render_resource_mesh(r,resource,{},0,false,true,true,kConstructionSteps,nullptr,kind);
                const bool driller=kind==ResourceKind::OreDriller;
                require(r.drew_at(1) && r.drew_at(2) && r.drew_at(3)==driller && r.drew_at(5)==driller,
                        "None hid exterior equipment or drew a resource hull");
            }
            r.vertices.clear();
            app.render_corridors(r);
            require(r.drew_at(1) && r.drew_at(2) && !r.drew_at(3) && !r.drew_at(5),
                    "None drew the corridor shell or lost its deck and selection marker");
        }
        std::cout<<"Full Hull default, None selector and hull draw layers passed\n";
    }

    static void graphics_quality(){
        playfield_surface();
        hull_views();
        using namespace eng;
        struct BenchmarkRecorder : Recorder {
            bool benchmarking{};
            GraphicsCapabilities graphics_capabilities() override {
                GraphicsCapabilities caps{};
                caps.enhanced=true;
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
        Application engine;
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
        choose("CASCADE SHADOW RESOLUTION","768 PX");
        require(app.graphics_.shadow_resolution==1 && app.graphics_.shadow_quality==2,
                "shadow resolution changed the filter quality");
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
        using namespace eng;
        Application engine;
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
        eng::Application engine;
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
                eng::InputState input;
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
                eng::InputState input;
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
        eng::Application engine;
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
        const auto click=[&](eng::Rect button){
            eng::InputState input;
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
        eng::InputState wheel;
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

    static void recent_saves(){
        namespace fs=std::filesystem;
        eng::Application engine;
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
            require(restarted.habitat_view_mode_==GameApp::HabitatViewMode::None,"save did not restore the hidden hull view");
            require(restarted.load_game_from_file(paths[0].string(),error) && restarted.playfield_.size()==64 &&
                    restarted.habitat_view_mode_==GameApp::HabitatViewMode::FullHull,
                    "saving a colliding name changed the earlier colony");
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
        eng::Application engine;
        constexpr std::array<eng::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
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
                if(scene==12)require(has_text("J JUMP") && has_text("SPACE PAUSE"),"POV help omits the jump or pause shortcut");
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
            app.input_={};app.selection_kind_=GameApp::SelectionKind::Robot;app.on_resize(r.w,r.h);const auto pane=app.selection_panel_rect();eng::InputState pointer;
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
            const auto click_astronaut_scene=[&](GameApp& scene,eng::Vec3 p){
                const auto m=scene.camera_.view_projection((float)stage_ui.w/stage_ui.h).m;
                const float w=m[3]*p.x+m[7]*p.y+m[11]*p.z+m[15];
                eng::InputState mouse;
                mouse.mouse_x=(int)std::lround((.5f+.5f*(m[0]*p.x+m[4]*p.y+m[8]*p.z+m[12])/w)*stage_ui.w);
                mouse.mouse_y=(int)std::lround((.5f-.5f*(m[1]*p.x+m[5]*p.y+m[9]*p.z+m[13])/w)*stage_ui.h);
                mouse.left_pressed=true;
                scene.update(0.f,mouse);
            };
            const auto advance_crew=[&](GameApp& scene,int frames){
                for(int i=0;i<frames;++i){
                    scene.update(.02f,{});
                    require(eng::length(scene.test_astronauts_[0].position-scene.test_astronauts_[1].position)
                            >=kAstronautSeparation-.001f,"astronauts crossed their safety clearance");
                }
            };
            const auto eva_start=sandbox.test_astronauts_[0].position;
            const auto coverall_start=sandbox.test_astronauts_[1].position;
            click_astronaut_scene(sandbox,eva_start+eng::Vec3{0,.95f,0});
            require(sandbox.selected_astronaut_==0,"mouse did not select the EVA astronaut");
            sandbox.render(stage_ui);
            const auto shows_text=[&](const char* caption){
                return std::any_of(stage_ui.draws.begin(),stage_ui.draws.end(),[&](const auto& draw){return draw.text==caption;});
            };
            require(shows_text("AMARA OKAFOR") && shows_text("WORKER / EVA SUIT") &&
                    shows_text("HEALTH 100%") && shows_text("HUNGER 0%    THIRST 0%"),"selected EVA astronaut status was missing");
            stage_ui.check_bounds("astronaut status");

            // This waypoint is on the other side of the stationary crew member.
            click_astronaut_scene(sandbox,{4.6f,0,-.55f});
            require(sandbox.test_astronauts_[0].destination.has_value(),"ground click did not give the EVA astronaut a waypoint");
            const auto eva_goal=*sandbox.test_astronauts_[0].destination;
            eng::InputState right_click;
            right_click.right_pressed=true;
            sandbox.update(0.f,right_click);
            require(sandbox.selected_astronaut_==-1 && sandbox.test_astronauts_[0].destination &&
                    eng::length(*sandbox.test_astronauts_[0].destination-eva_goal)<.001f,
                    "right click must deselect the astronaut and preserve its waypoint");
            advance_crew(sandbox,50);
            const auto eva_saved=sandbox.test_astronauts_[0].position;
            require(eng::length(eva_saved-eva_start)>.2f && sandbox.test_astronauts_[0].stride_phase>.05f &&
                    eng::length(sandbox.test_astronauts_[1].position-coverall_start)<.001f,
                    "walking did not advance the selected mesh animation independently");
            require(sandbox.save_game_to_name("stage3-sandbox-regression",error),"astronaut field save failed: "+error);
            require(loaded.load_game_from_file(save.string(),error) && loaded.astronaut_test_active(),
                    "astronaut test mode did not survive save/load");
            loaded.screen_=GameApp::Screen::Playfield;
            loaded.render(stage_ui);
            require(eng::length(loaded.test_astronauts_[0].position-eva_saved)<.001f &&
                    loaded.test_astronauts_[0].destination.has_value(),
                    "save did not restore the astronaut position and pending waypoint");
            advance_crew(loaded,650);
            require(!loaded.test_astronauts_[0].destination && eng::length(loaded.test_astronauts_[0].position-eva_goal)<.01f &&
                    eng::length(loaded.test_astronauts_[1].position-coverall_start)<.001f,
                    "EVA astronaut did not resume and finish the saved detour");

            click_astronaut_scene(loaded,coverall_start+eng::Vec3{0,.95f,0});
            require(loaded.selected_astronaut_==1,"mouse did not select the coverall astronaut");
            loaded.render(stage_ui);
            require(shows_text("LUCA REYES") && shows_text("ENGINEER / EVA SUIT"),
                    "selected coverall astronaut status was missing");
            click_astronaut_scene(loaded,{6.5f,0,-.55f});
            require(loaded.test_astronauts_[1].destination.has_value(),"ground click did not give the coverall astronaut a waypoint");
            const auto coverall_goal=*loaded.test_astronauts_[1].destination;
            click_astronaut_scene(loaded,loaded.test_astronauts_[0].position+eng::Vec3{0,.95f,0});
            click_astronaut_scene(loaded,{-2.5f,0,-.18f});
            require(loaded.selected_astronaut_==0 && loaded.test_astronauts_[0].destination,
                    "another astronaut's pending walk prevented selecting and ordering the EVA astronaut");
            const auto return_goal=*loaded.test_astronauts_[0].destination;
            advance_crew(loaded,1150);
            require(!loaded.test_astronauts_[0].destination && !loaded.test_astronauts_[1].destination &&
                    eng::length(loaded.test_astronauts_[0].position-return_goal)<.01f &&
                    eng::length(loaded.test_astronauts_[1].position-coverall_goal)<.01f,
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
            eng::InputState right_drag;
            right_drag.right_down=true;
            right_drag.mouse_dx=20;
            sandbox.update(0.f,right_drag);
            sandbox.update(0.f,{});
            require(sandbox.selection_kind_==GameApp::SelectionKind::Habitat &&
                    eng::length(sandbox.camera_.state().target-before_pan)>.01f,
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
                attached|=eng::length(midpoint-first_roller)<.01f;
            }
            require(attached,"first conveyor section floats away from the hull");
            click_world(22,0);
            require(sandbox.utility_network_.connections().size()==1 &&
                    sandbox.utility_network_.connections()[0].kind==UtilityKind::ConveyorBelt &&
                    sandbox.utility_network_.connections()[0].construction.complete(),
                    "conveyor build-menu placement did not create the route");
            require(eng::length(sandbox.utility_network_.connections().front().endpoint_a.position-port->position)<.01f,
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
            eng::InputState zoom;zoom.wheel=2.0f;
            scene_camera.update(zoom,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(scene_camera.state().distance<start.distance,"Stage-1 mouse wheel zoom is not functional");
            eng::InputState orbit;orbit.middle_down=true;orbit.mouse_dx=36;orbit.mouse_dy=20;
            scene_camera.update(orbit,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(scene_camera.state().azimuth_degrees!=start.azimuth_degrees &&
                    scene_camera.state().elevation_degrees!=start.elevation_degrees,"Stage-1 middle-button orbit failed");
            const auto old_target=scene_camera.state().target;
            eng::InputState pan;pan.right_down=true;pan.mouse_dx=21;pan.mouse_dy=-8;
            scene_camera.update(pan,GameApp::kCompanySitePlayfieldSize,1080,0.016f);
            require(eng::length(scene_camera.state().target-old_target)>0.01f,"Stage-1 right-button pan failed");
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
            eng::InputState move_w;move_w.key_w_down=true;
            keyboard.update(0.5f,move_w);
            const auto after_w=keyboard.camera_.target();
            require(eng::length(after_w-start)>0.1f,"W did not translate the top-down camera");
            eng::InputState move_d;move_d.key_d_down=true;
            keyboard.update(0.5f,move_d);
            const auto after_d=keyboard.camera_.target();
            require(eng::length(after_d-after_w)>0.1f,"D did not strafe the top-down camera");

            GameApp coarse(engine);setup(coarse,7,2);coarse.screen_=GameApp::Screen::Playfield;
            GameApp fine(engine);setup(fine,7,2);fine.screen_=GameApp::Screen::Playfield;
            for(int i=0;i<20;++i)coarse.update(0.05f,move_w);
            for(int i=0;i<100;++i)fine.update(0.01f,move_w);
            require(eng::length(coarse.camera_.target()-fine.camera_.target())<0.05f,"WASD camera movement depends on frame rate");
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
            eng::InputState esc;esc.escape_pressed=true;
            menu.update(0.0f,esc);
            require(menu.screen_==GameApp::Screen::Playfield,"ESC did not resume from the in-game menu");
            menu.screen_=GameApp::Screen::Settings;
            menu.update(0.0f,esc);
            require(menu.screen_==GameApp::Screen::MainMenu,"ESC from an in-game sub-menu did not return to the game menu");
        }
        for(const auto size:std::array<eng::Vec2,3>{{{1280,1024},{3440,1440},{5120,1440}}}){GameApp app(engine);setup(app,10,2);r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("non-16:9 HUD");}
        GameApp catalog(engine);setup(catalog,7,4);Recorder small;small.resize(1280,720);catalog.render(small);
        const auto next=small.button(">");catalog.input_.mouse_x=(int)std::lround((next.x+next.w/2)*small.scale);catalog.input_.mouse_y=(int)std::lround((next.y+next.h/2)*small.scale);catalog.input_.left_pressed=true;catalog.render(small);
        require(catalog.build_catalog_scroll_==1,"fixed-size icon catalog did not page");
        eng::InputState wheel=catalog.input_;wheel.left_pressed=false;wheel.wheel=-1;const auto distance=catalog.camera_.distance();catalog.update(0,wheel);catalog.render(small);
        require(catalog.build_catalog_scroll_==2 && catalog.camera_.distance()==distance,"catalog wheel also zoomed the world");
        for(const auto& caption:{std::string("ON"),std::string(90,'W')}){Recorder button;button.set_2d(1);eng::ui::button(button,{}, {0,0,82,34},caption,{});const auto& text=button.draws.back();require(text.v[0]>=eng::ui::kControlPaddingX-0.01f && text.v[1]>=eng::ui::kControlPaddingY-0.01f && text.v[0]+text.v[2]<=82-eng::ui::kControlPaddingX+0.01f && text.v[1]+text.v[3]<=34-eng::ui::kControlPaddingY+0.01f,"button caption escaped its padding");}
        const auto stroke=eng::ui::line_quad({0,0},{40,0},2);require(std::abs(stroke[1]-stroke[7]-2)<1e-6f,"logical stroke thickness changed");
        const auto metric=eng::ui::text_extent("WW\nW",2);require(metric.x==24 && metric.y==30,"multiline font metrics differ from glyph grid");
        for(char ch:std::string("[]_!?#|"))require(eng::ui::kBitmapFont.count(ch)==1,"UI punctuation glyph missing");
        std::cout<<"475 resolution/zoom/screen cases and "+std::to_string(structure_cases)+" structure warning cases, HD-to-4K proportions, hitboxes, frame-boundary zoom, aspect ratios, button text and shared font/stroke metrics passed\n";
    }
};
}
int main(int argc,char** argv){
    const auto preferences=std::filesystem::temp_directory_path()/
        ("lunar-ui-regression-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".cfg");
#ifdef _WIN32
    if(_putenv_s("GAME_GRAPHICS_CONFIG",preferences.string().c_str())!=0) return 1;
#else
    if(setenv("GAME_GRAPHICS_CONFIG",preferences.c_str(),1)!=0) return 1;
#endif
    int result=0;
    try{
        const std::string out=argc>1?argv[1]:"";
        if(out=="build-mode"){
            game::UiRegression::build_mode();
            game::UiRegression::recent_saves();
        }
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
