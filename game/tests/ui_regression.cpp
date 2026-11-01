#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/ui_font.hpp"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool pass,const std::string& message){if(!pass)throw std::runtime_error(message);}
struct Draw {char type;std::array<float,6> v{};std::string text;eng::Color color;};
struct Recorder:eng::Renderer {
    int w=1920,h=1080;float scale=1;bool hud=false;std::vector<Draw> draws;
    bool initialize(void*,std::uintptr_t,int,int)override{return true;}
    void resize(int width,int height)override{w=width;h=height;}
    void begin_frame(eng::Color)override{draws.clear();hud=false;}void end_frame()override{}
    void set_3d(const eng::Mat4&)override{hud=false;}void set_2d(float s)override{scale=s;hud=true;}
    void draw_lines_3d(const std::vector<eng::Vec3>&,eng::Color,float)override{}
    void draw_triangles_3d(const std::vector<eng::Vec3>&,eng::Color)override{}
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
            if(scene==12){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-12,0,-12},45);}
        }
    }
    static void order_controls(){
        eng::Application engine;GameApp defaults(engine);require(defaults.ui_scale_index_==0 && defaults.ui_scale_==1,"default UI scale is not 100%");int cases=0;
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
            const auto layout_button=debug_r.button("MESH");
            debug.input_.mouse_x=(int)((layout_button.x+layout_button.w/2)*debug_r.scale);
            debug.input_.mouse_y=(int)((layout_button.y+layout_button.h/2)*debug_r.scale);
            debug.input_.left_pressed=true;debug.render(debug_r);
            require(debug.world_layout_mode_==GameApp::WorldLayoutMode::Textured,"debug layout switch did not select textured mode");
            debug.input_.left_pressed=false;debug.render(debug_r);debug_r.button("TEXTURED");
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
            require(bars.size()==6,"missing charge/air/water indicator bars");const float charge=state==0?0:state==1?0.5f:1,air=state==0?0:state==1?0.25f:1;
            require(std::abs(bars[1].v[2]-bars[0].v[2]*charge)<0.001f,"charge bar is not actual completed-battery energy/capacity");
            require(std::abs(bars[3].v[2]-bars[2].v[2]*air)<0.001f,"air bar does not combine indoor volume correctly");
            require(std::abs(bars[5].v[2]-bars[4].v[2]*air)<0.001f,"water bar is not actual completed tank fill");
            if(state){require(labels.find(state==1?"RATE +4.5 KW":"RATE -2.0 KW")!=std::string::npos,"power charge/discharge rate missing");require(labels.find(state==1?"RATE +123.0 L/D":"RATE -34.1 L/D")!=std::string::npos,"water fill/drain rate missing");}
            require(labels.find(state==0?"AIR --":state==1?"AIR 25.0%":"AIR 100.0%")!=std::string::npos,"air label disagrees with indicator");
            require(labels.find(state==0?"POWER 0.0/0.0 KWH":state==1?"POWER 45.0/90.0 KWH":"POWER 90.0/90.0 KWH")!=std::string::npos,"power label uses stale counters or includes blueprints");
            if(!output.empty() && state==1 && zoom==2 && (size.x==1280 || size.x==3840))r.write(std::filesystem::path(output)/("status-bars-"+std::to_string((int)size.x)+".json"));
            ++cases;
        }
        std::cout<<cases<<" status-bar cases: battery capacity/charge, combined indoor air, disconnected rooms, excluded blueprints/outdoor machinery, empty/full/clamped values and resolution scaling passed\n";
    }
    static void progress_tree(){
        eng::Application engine;int cases=0;
        constexpr std::array<eng::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(const auto size:sizes){
            GameApp app(engine);setup(app,7,zoom);app.debug_frame_enabled_=false;Recorder r;r.resize((int)size.x,(int)size.y);app.render(r);r.check_bounds("progress tree");
            bool title=false,stage=false,milestone=false;for(const auto& d:r.draws)if(d.type=='T'){title|=d.text.find("COLONY PROGRESS")!=std::string::npos;stage|=d.text.find("STAGE 1")!=std::string::npos;milestone|=d.text.find("READY FOR HUMAN CREW")!=std::string::npos;}
            require(title&&stage&&milestone,"normal gameplay progression checklist is missing");
            const auto panel=app.progress_tree_panel_rect();const float logical_h=size.y/app.ui_scale_;
            require(panel.y>=layout::STATUS_BAR_H+32.0f,"progress panel overlaps status tooltip area");
            require(panel.y+panel.h<=logical_h-layout::BUILD_BAR_H-4.0f,"progress panel overlaps build catalog");
            ++cases;
        }
        GameApp debug(engine);setup(debug,7,2);debug.debug_frame_enabled_=true;Recorder r;r.resize(1920,1080);debug.render(r);for(const auto& d:r.draws)require(d.type!='T' || d.text.find("COLONY PROGRESS")==std::string::npos,"progress tree visible in debugging mode");
        std::cout<<cases<<" progression checklist resolution/border cases plus debug-mode suppression passed\n";
    }

    static void run(const std::string& output){
        order_controls();
        airlock_status();
        status_bars(output);
        progress_tree();
        eng::Application engine;
        constexpr std::array<eng::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(int scene=0;scene<19;++scene){
            Recorder reference;
            for(std::size_t n=0;n<sizes.size();++n){
                GameApp app(engine);setup(app,scene,zoom);Recorder r;r.resize((int)sizes[n].x,(int)sizes[n].y);app.render(r);
                const std::string label="scene "+std::to_string(scene)+" zoom "+std::to_string(zoom)+" "+std::to_string(r.w);
                r.check_bounds(label);
                const auto has_text=[&](const std::string& part){for(const auto& d:r.draws)if(d.type=='T' && d.text.find(part)!=std::string::npos)return true;return false;};
                if(scene==8 || scene==9 || (scene>=13 && scene<=15))require(has_text("BREATHABLE AIR") && has_text("AIR CHANGE"),"indoor details omit the air percentage/change rate");
                if(scene==13)require(has_text("75.0%") && has_text("+0.125") && has_text("O2 OUTPUT 9.000 KG/D / WATER 10.125 L/D") && has_text("H2 OUTPUT 1.125 KG/D / BUFFER 2.250") && has_text("ELECTROLYSIS: O2 + H2"),"oxygen-generator details do not show measured oxygen/water/hydrogen output");
                if(scene==18)require(has_text("FUEL CELL 2.75 / 3.00 KW") && has_text("H2 USE 3.600 KG/D") && has_text("COMPLETED SUPPLY-CHANNEL NETWORK"),"hydrogen-generator details omit fuel-cell dispatch/transport telemetry");
                if(scene==9)require(has_text("WARNING:") && has_text("AIR BELOW 95%"),"storage warning icon reason missing");
                if(scene==16 || scene==17)require(has_text("WARNING:") && has_text("INSUFFICIENT POWER"),"storage/workshop early return hides warning reasons");
                if(scene==12)require(has_text("SPACE JUMP"),"POV help omits Space jump");
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
            const auto start=r.button("START NEW GAME");app.input_.mouse_x=(int)std::lround((start.x+start.w*0.5f)*r.scale);app.input_.mouse_y=(int)std::lround((start.y+start.h*0.5f)*r.scale);app.input_.left_pressed=true;app.render(r);
            require(app.screen_==GameApp::Screen::NewGame,"menu hitbox missed after resolution scaling");
            app.input_={};app.selection_kind_=GameApp::SelectionKind::Robot;app.on_resize(r.w,r.h);const auto pane=app.selection_panel_rect();eng::InputState pointer;
            pointer.mouse_x=(int)std::lround((pane.x+pane.w/2)*app.ui_scale_);pointer.mouse_y=(int)std::lround((pane.y+pane.h/2)*app.ui_scale_);
            require(app.mouse_over_selection_panel(pointer),"selection hitbox differs from drawing");pointer.mouse_x=0;require(!app.mouse_over_selection_panel(pointer),"selection hitbox extends outside panel");
        }
        GameApp settings(engine);setup(settings,3,2);Recorder r;settings.render(r);const float before=settings.ui_scale_;const auto plus=r.button(">");settings.input_.mouse_x=(int)std::lround((plus.x+plus.w/2)*before);settings.input_.mouse_y=(int)std::lround((plus.y+plus.h/2)*before);settings.input_.left_pressed=true;settings.render(r);
        require(settings.ui_scale_index_==3 && settings.ui_scale_==before,"zoom changed input/drawing transform mid-frame");settings.input_={};settings.render(r);require(settings.ui_scale_>before,"zoom preference did not apply on next frame");
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
int main(int argc,char** argv){try{const std::string out=argc>1?argv[1]:"";if(!out.empty())std::filesystem::create_directories(out);game::UiRegression::run(out);}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}
