#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "engine/ui_metrics.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok,const std::string& what){if(!ok)throw std::runtime_error(what);}
struct Capture:eng::Renderer {
    int w=1600,h=900;float scale=1;bool hud=false;eng::Mat4 matrix{};
    struct Line {eng::Vec3 a,b;eng::Color color;};std::vector<Line> lines;
    struct Tri {eng::Vec3 a,b,c;eng::Color color;};std::vector<Tri> triangles;
    struct Text {std::string text;eng::Rect rect;};std::vector<Text> texts;eng::Rect last_rect{};
    bool initialize(void*,std::uintptr_t,int,int)override{return true;}void resize(int x,int y)override{w=x;h=y;}
    void begin_frame(eng::Color)override{triangles.clear();lines.clear();texts.clear();hud=false;}void end_frame()override{}
    void set_3d(const eng::Mat4& m)override{matrix=m;hud=false;}void set_2d(float s)override{scale=s;hud=true;}
    void draw_lines_3d(const std::vector<eng::Vec3>& v,eng::Color color,float)override{for(std::size_t i=0;i+1<v.size();i+=2)lines.push_back({v[i],v[i+1],color});}
    void draw_triangles_3d(const std::vector<eng::Vec3>& v,eng::Color color)override{
        for(std::size_t i=0;i+2<v.size();i+=3)triangles.push_back({v[i],v[i+1],v[i+2],color});
    }
    void draw_rect(eng::Rect p,eng::Color)override{last_rect=p;}
    void draw_line(eng::Vec2,eng::Vec2,eng::Color,float)override{}
    void draw_text(eng::Vec2 p,const std::string& s,float size,eng::Color)override{
        auto extent=eng::ui::text_extent(s,size);
        require(p.x>=-.5f&&p.y>=-.5f&&(p.x+extent.x)*scale<=w+1&&(p.y+extent.y)*scale<=h+1,"lander UI text outside viewport: "+s);
        texts.push_back({s,last_rect});
    }
    int width()const override{return w;}int height()const override{return h;}bool hardware_accelerated()const override{return false;}
    std::string gpu_vendor()const override{return "TEST";}std::string gpu_renderer()const override{return "GEOMETRY CAPTURE";}std::string gpu_version()const override{return "TEST";}
    eng::Rect button(const std::string& s)const{for(const auto& t:texts)if(t.text==s)return t.rect;throw std::runtime_error("missing button "+s);}
    void write(const std::filesystem::path& p)const{
        std::ofstream f(p);f<<"{\"matrix\":[";for(int i=0;i<16;++i){if(i)f<<',';f<<matrix.m[i];}f<<"],\"triangles\":[";
        bool first=true;for(const auto& t:triangles){if(!first)f<<',';first=false;f<<'[';for(auto v:{t.a,t.b,t.c})f<<v.x<<','<<v.y<<','<<v.z<<',';f<<t.color.r<<','<<t.color.g<<','<<t.color.b<<','<<t.color.a<<']';}f<<"],\"lines\":[";first=true;for(const auto& l:lines){if(!first)f<<',';first=false;f<<'[';for(auto v:{l.a,l.b})f<<v.x<<','<<v.y<<','<<v.z<<',';f<<l.color.r<<','<<l.color.g<<','<<l.color.b<<','<<l.color.a<<']';}f<<"]}";
    }
};
}
namespace game {
using eng::Vec3;
struct TraderRegression {
    static void connect(GameApp& a,UtilityEndpointKind ka,std::size_t ia,UtilityEndpointKind kb,std::size_t ib){
        auto pa=ka==UtilityEndpointKind::Resource?a.placed_resources_[ia].position:a.placed_habitats_[ia].position;
        auto pb=kb==UtilityEndpointKind::Resource?a.placed_resources_[ib].position:a.placed_habitats_[ib].position;
        require(a.utility_network_.build_connection(UtilityKind::SupplyChannel,{ka,ia,pa},{kb,ib,pb},a.placed_habitats_,a.habitat_models_,a.placed_resources_,a.resource_models_),"trade power network failed");
        for(auto& u:a.utility_network_.mutable_connections())u.construction.step=u.construction.total_steps;
    }
    static void setup(GameApp& a){
        a.playfield_size_=128;a.playfield_.rebuild(128);a.camera_.restore({105,45,20,{0,0,0}});a.status_.reset();auto state=a.status_.state();state.hour=12;a.status_.restore(state);
        a.active_game_phase_=GameApp::GamePhase::RobotPreparation;a.screen_=GameApp::Screen::Playfield;a.game_session_active_=true;a.debug_frame_enabled_=false;
        a.starting_robot_count_=1;a.starting_carrier_count_=1;a.reset_builder_robot_and_starter_stash();a.resources_.reset();a.ground_resource_piles_.clear();a.utility_network_.set_playfield_half_extent(64);
        a.builder_robots_[0].position={-25,0,-18};a.builder_robots_[1].position={-25,0,-22};for(auto& r:a.builder_robots_){r.visual_position=r.position;r.safety={};}
        const auto add=[&](ResourceKind k,Vec3 pos,std::uint64_t seq){PlacedResource p;p.kind=k;p.position=pos;p.placement_sequence=seq;p.construction.step=kConstructionSteps;a.placed_resources_.push_back(p);};
        add(ResourceKind::LandingPad,{16,0,0},10);add(ResourceKind::SolarPanel,{16,0,26},11);add(ResourceKind::PowerStorage,{-4,0,30},12);add(ResourceKind::WaterStorage,{-20,0,38},13);add(ResourceKind::HydrogenTank,{-38,0,18},14);add(ResourceKind::Antenna,{38,0,30},15);
        a.placed_resources_[2].supply.energy_kwh=power_storage_capacity_kwh(ResourceSizeTier::Standard);a.placed_resources_[3].supply.water_litres=1800;a.placed_resources_[4].supply.hydrogen_kg=100;
        PlacedHabitat h;h.kind=HabitatKind::OxygenGeneratorDome;h.position={-20,0,18};h.placement_sequence=20;h.construction.step=kConstructionSteps;a.placed_habitats_.push_back(h);
        for(std::size_t i=0;i<4;++i)connect(a,UtilityEndpointKind::Resource,i,UtilityEndpointKind::Resource,i+1);
        connect(a,UtilityEndpointKind::Resource,3,UtilityEndpointKind::Habitat,0);
        a.update_supply_simulation(0);a.placed_habitats_[0].supply.oxygen_kg=a.placed_habitats_[0].supply.volume_m3*kOxygenKgPerM3;
        a.next_blueprint_placement_sequence_=30;a.selection_kind_=GameApp::SelectionKind::Resource;a.selected_resource_=0;
        ResourceBundle b;b.iron=6000;b.food=1000;a.resources_.restore(b);a.ground_resource_piles_={{MaterialResource::Iron,6000,{-18,0,-6},.36f,1},{MaterialResource::Food,1000,{-18,0,-11},.36f,1}};
        require(a.placed_resources_[0].supply.power_fraction>=.5,"test pad has no real network power");
    }
    static std::unique_ptr<GameApp> resume(GameApp& a,const std::string& name){
        std::string error;require(a.save_game_to_name(name,error),"trade save failed: "+error);
        auto path=std::filesystem::path(a.save_game_directory())/(name+".gmsave");auto b=std::make_unique<GameApp>(a.app_);
        require(b->load_game_from_file(path.string(),error),"trade load failed: "+error);std::filesystem::remove(path);
        require(b->trader_.phase==a.trader_.phase&&b->trader_.loaded==a.trader_.loaded&&b->trader_.requested==a.trader_.requested&&b->planning_.funds_eur==a.planning_.funds_eur,"resume changed trade/cash state");
        require(b->trader_.hauls.size()==a.trader_.hauls.size(),"resume dropped cargo carrier");return b;
    }
    static void oxygen_storage(eng::Application& engine){
        GameApp app(engine);
        setup(app);
        PlacedResource tank;
        tank.kind=ResourceKind::OxygenTank;
        tank.position={-39,0,41};
        tank.placement_sequence=28;
        tank.size=ResourceSizeTier::Small;
        tank.construction.step=kConstructionSteps;
        app.placed_resources_.push_back(tank);
        const auto index=app.placed_resources_.size()-1;
        app.update_supply_simulation(240);
        require(app.placed_resources_[index].stored_oxygen_kg==0,"unconnected oxygen tank filled");
        connect(app,UtilityEndpointKind::Resource,3,UtilityEndpointKind::Resource,index);
        app.placed_resources_[2].supply.energy_kwh=135;
        app.placed_habitats_[0].supply.oxygen_kg=app.placed_habitats_[0].supply.volume_m3*kOxygenKgPerM3;
        app.update_supply_simulation(240);
        const auto stored=app.placed_resources_[index].stored_oxygen_kg;
        require(stored>3&&stored<=250&&app.trader_available()[8]==(std::int64_t)stored,
                "oxygen storage did not collect/export connected surplus");
        auto loaded=resume(app,"oxygen-storage");
        require(std::abs(loaded->placed_resources_[index].stored_oxygen_kg-stored)<1e-9,"oxygen tank lost stock on load");
        app.placed_habitats_[0].supply.oxygen_kg=0;
        app.update_supply_simulation(.1f);
        require(app.placed_resources_[index].stored_oxygen_kg<stored&&app.placed_habitats_[0].supply.oxygen_kg>0,
                "reserve did not supply a connected low-air habitat");
        require(loaded->call_trader(0),"oxygen tank sale could not call trader");
        loaded->update_trader(30);
        loaded->trader_.requested[8]=3;
        require(loaded->start_trader_sale()&&std::abs(loaded->placed_resources_[index].stored_oxygen_kg-(stored-3))<1e-9,
                "oxygen tank sale did not reserve exact stock");
    }
    static void conveyor_trade(eng::Application& engine){
        GameApp app(engine);
        setup(app);
        PlacedResource drill;
        drill.kind=ResourceKind::OreDriller;
        drill.position={-52,0,-29};
        app.finalize_placement(drill);
        drill.construction.finish();
        drill.supply.power_fraction=1;
        app.placed_resources_.push_back(drill);
        const auto di=app.placed_resources_.size()-1;
        PlacedResource dome;
        dome.kind=ResourceKind::StorageDome;
        dome.position={-29,0,-28};
        app.finalize_placement(dome);
        dome.construction.finish();
        app.placed_resources_.push_back(dome);
        const auto si=app.placed_resources_.size()-1;

        const auto build=[&](std::size_t from,std::size_t to){
            const auto start=app.placed_resources_[from].position;
            const auto finish=app.placed_resources_[to].position;
            return app.utility_network_.build_connection(UtilityKind::ConveyorBelt,
                {UtilityEndpointKind::Resource,from,start},{UtilityEndpointKind::Resource,to,finish},
                app.placed_habitats_,app.habitat_models_,app.placed_resources_,app.resource_models_);
        };
        require(!build(di,0) && !build(0,si) && !build(si,di),"invalid conveyor direction accepted");
        require(build(di,si),"ore driller could not connect to storage");
        auto& belts=app.utility_network_.mutable_connections();
        const auto intake=belts.size()-1;
        app.finalize_placement(belts[intake]);
        belts[intake].construction.finish();
        require(build(si,0),"storage could not connect to the landing pad");
        const auto export_belt=belts.size()-1;
        app.finalize_placement(belts[export_belt]);
        belts[export_belt].construction.finish();
        require(!build(si,0) && app.conveyor_route_clear(belts[intake].path) &&
                app.conveyor_route_clear(belts[export_belt].path),"conveyor path or source occupancy invalid");
        require(!app.utility_network_.evaluate_services(app.corridor_network_,app.placed_habitats_,
                app.placed_resources_).resources[si].connected,"cargo belts became utility links");

        auto& stock=app.placed_resources_[si];
        stock.stored_ore_boxes[0]=storage_dome_capacity_kg(stock.storage_size)/ore::box_kg;
        app.placed_resources_[di].mined_ore_kg[0]=ore::box_kg;
        app.update_ore_logistics(app.utility_network_.connection_length(intake)/ore::belt_speed_mps+1);
        require(belts[intake].ore_cargo==0,"full dome did not hold the arriving ore crate");
        stock.stored_ore_boxes[0]=1;
        app.update_ore_logistics(.1f);
        require(stock.stored_ore_boxes[0]==2 && belts[intake].ore_cargo==ore::no_cargo &&
                app.resources_.amount(MaterialResource::Iron)==6000,
                "dome conveyor did not accept raw ore without crediting refined stock");

        stock.stored_boxes={{MaterialResource::Iron,180,1.f}};
        sync_stored_resource_totals(stock);
        ResourceBundle added;
        added.iron=180;
        app.resources_.add(added);
        require(!storage_loading::has_ready_gate(stock) && app.call_trader(0),
                "conveyor-fed storage fixture has a gate or could not call a trader");
        app.update_trader(24);
        const auto raw=(std::size_t)TradeGood::IronOre;
        require(app.trader_available()[raw]==10 && app.trader_available()[0]==6180,"belt stock missing from sale manifest");
        app.trader_.requested[raw]=1;
        require(!app.start_trader_sale() && stock.stored_ore_boxes[0]==2,"partial raw ore box was reserved");
        app.trader_.requested[raw]=10;
        app.trader_.requested[0]=180;
        require(app.start_trader_sale() && app.trader_.lots.size()==2 &&
                stock.stored_boxes.empty() && !stock.stored_ore_boxes[0] &&
                app.resources_.amount(MaterialResource::Iron)==6000,"reservation did not prefer the belt-fed stock");

        const auto belt_sequence=belts[export_belt].placement_sequence;
        app.update_trade_hauling(.1f,[](auto,auto){return true;},[](auto,auto){},[](auto){});
        require(app.trader_.hauls.empty(),"a rover was dispatched for belt cargo");
        app.update_trader(.5f);
        require(app.trader_.lots.front().conveyor_sequence==belt_sequence &&
                app.trader_.lots.front().conveyor_distance_m>0 && app.trader_.loaded[0]==0,
                "cargo was credited before reaching the pad");
        auto loaded=resume(app,"conveyor-trader");
        const auto distance=loaded->trader_.lots.front().conveyor_distance_m;
        require(distance==app.trader_.lots.front().conveyor_distance_m &&
                loaded->trader_.lots.front().conveyor_sequence==belt_sequence,
                "in-flight trader cargo did not survive save/load");
        loaded->placed_resources_[0].supply.power_fraction=0;
        loaded->update_trader(600);
        require(loaded->trader_.loaded[0]==0 && loaded->trader_.lots.front().conveyor_distance_m==distance,
                "belt loaded a trader without pad power");
        loaded->update_supply_simulation(0);
        loaded->update_trader(600);
        require(loaded->trader_.phase==TradePhase::Departing && loaded->trader_.loaded==loaded->trader_.requested,
                "belt-only shipment did not finish with the exact manifest");
        const auto cash=loaded->planning_.funds_eur;
        loaded->update_trader(18);
        require(loaded->planning_.funds_eur==cash+180*trade_price_eur(TradeGood::Iron)+10*trade_price_eur(TradeGood::IronOre),
                "belt cargo was not paid once at departure");

        app.remove_utility_route(export_belt);
        require(app.trader_.lots.front().conveyor_sequence==0 && app.trader_.lots.front().conveyor_distance_m==0 &&
                app.trader_.lots.front().remaining==180 && app.trader_.lots.back().remaining==10,
                "removing an active belt lost or duplicated reserved goods");
        app.update_trade_hauling(.1f,[](auto,auto){return true;},[](auto,auto){},[](auto){});
        require(!app.trader_.hauls.empty(),"removed belt cargo was not offered to rovers");
        for(int i=0;i<600*90 && app.trader_.phase==TradePhase::Loading;++i){
            app.update_trader(1.f/90);
            app.update_construction(1.f/90);
        }
        require(app.trader_.phase==TradePhase::Departing && app.trader_.loaded==app.trader_.requested,
                "rover fallback did not finish both lots after removing the export belt");
        std::cout<<"Conveyor trade PASS: raw storage, capacity backpressure, separate ore sale, cargo custody, power and belt removal\n";
    }
    static void run(){
        eng::Application engine;oxygen_storage(engine);conveyor_trade(engine);auto a=std::make_unique<GameApp>(engine);setup(*a);
        a->placed_resources_[0].construction.step=0;require(!a->call_trader(0),"unfinished pad called trader");a->placed_resources_[0].construction.step=kConstructionSteps;
        a->placed_resources_[0].supply.power_fraction=0;require(!a->call_trader(0),"unpowered pad called trader");a->update_supply_simulation(0);
        a->placed_resources_.pop_back();
        require(!a->call_trader(0),"powered pad called trader without an antenna");
        Capture ui;
        a->render(ui);a->selection_panel_scroll_=a->selection_panel_max_scroll();a->render(ui);
        ui.button("BUILD AN ANTENNA TO CALL A TRADER");
        for(const auto& label:ui.texts)require(label.text!="TRADE","trade button exposed without antenna");
        PlacedResource antenna;antenna.kind=ResourceKind::Antenna;antenna.position={38,0,30};antenna.placement_sequence=15;
        a->placed_resources_.push_back(antenna);
        require(!a->call_trader(0),"antenna blueprint satisfied trade prerequisite");
        a->placed_resources_.back().construction.step=kConstructionSteps;
        a->render(ui);const auto call=ui.button("TRADE");
        a->input_.mouse_x=(int)((call.x+call.w/2)*ui.scale);a->input_.mouse_y=(int)((call.y+call.h/2)*ui.scale);a->input_.left_pressed=true;a->render(ui);a->input_={};
        require(a->trader_.phase==TradePhase::Approach&&!a->call_trader(0),"TRADE button not latched / duplicate trader called");
        a->placed_resources_.pop_back();
        a->update_trader(10);a=resume(*a,"trader-approach");
        const float held=a->trader_.elapsed;a->placed_resources_[0].supply.power_fraction=0;a->update_trader(100);require(a->trader_.elapsed==held,"approach ignored lost pad power");a->update_supply_simulation(0);a->update_trader(30);
        require(a->trader_.phase==TradePhase::Docked&&!a->start_trader_sale(),"empty sale accepted");
        const auto air=a->placed_habitats_[0].supply.oxygen_kg,water=a->placed_resources_[3].supply.water_litres;
        a->update_supply_simulation(240);const double bottled=a->trader_oxygen_stock_amount(20);
        require(bottled>10&&a->placed_resources_[3].supply.water_litres<water-10,"bottled oxygen appeared without electrolysis/water");
        require(a->placed_habitats_[0].supply.oxygen_kg>=air*.98,"export production drained breathable air");
        const auto available=a->trader_available();require(available[8]>=10&&available[9]>=100,"oxygen/hydrogen missing from sale catalog");
        a->trader_.requested[0]=20001;require(!a->start_trader_sale()&&a->resources_.amount(MaterialResource::Iron)==6000,"invalid sale changed stock");
        a->trader_.requested={};a->trader_.requested[0]=5000;a->trader_.requested[5]=700;a->trader_.requested[8]=3;a->trader_.requested[9]=6;
        const auto cash=a->planning_.funds_eur;const double air_at_sale=a->placed_habitats_[0].supply.oxygen_kg;const double hydrogen=a->placed_resources_[4].supply.hydrogen_kg;
        require(a->start_trader_sale()&&!a->start_trader_sale(),"sale not atomic / repeatable");const auto value=5000*trade_price_eur(TradeGood::Iron)+700*trade_price_eur(TradeGood::Food)+3*trade_price_eur(TradeGood::Oxygen)+6*trade_price_eur(TradeGood::Hydrogen);
        require(a->planning_.funds_eur==cash&&a->trader_.loaded[0]==0,"cash/cargo credited at reservation");
        require(a->resources_.amount(MaterialResource::Iron)==1000&&a->resources_.amount(MaterialResource::Food)==300,"sale goods not reserved from normal inventory");
        require(std::abs(a->placed_habitats_[0].supply.oxygen_kg-air_at_sale)<1e-8&&std::abs(a->placed_resources_[4].supply.hydrogen_kg-(hydrogen-6))<1e-8,"gas sale drained atmosphere or wrong H2 mass");
        a->screen_=GameApp::Screen::Playfield;
        a->selection_kind_=GameApp::SelectionKind::Resource;
        a->selected_resource_=0;
        a->render(ui);
        a->selection_panel_scroll_=a->selection_panel_max_scroll();
        a->render(ui);
        bool remaining=false;
        for(const auto& text:ui.texts){
            remaining|=text.text.find("5000 KG REMAINING")!=std::string::npos;
            require(text.text!="ALL"&&text.text!="START LOADING"&&text.text.find("EUR/KG")==std::string::npos,
                    "loading view still contains the sale palette");
        }
        require(remaining,"loading view did not show outstanding cargo");
        a=resume(*a,"trader-reserved");
        bool carried=false,saved_loaded=false;
        for(int i=0;i<600*90&&a->trader_.phase==TradePhase::Loading;++i){
            a->update_trader(1.f/90);a->update_construction(1.f/90);
            for(const auto& h:a->trader_.hauls)if(h.cargo>0)carried=true;
            if(carried&&!saved_loaded){a=resume(*a,"trader-on-rover");saved_loaded=true;}
            for(const auto& r:a->builder_robots_)require(r.safety.mode!=robots::SafetyMode::Fault,"trade rover safety fault");
            require(a->planning_.funds_eur==cash,"paid before loaded ship departed");
        }
        if(a->trader_.phase!=TradePhase::Departing){
            std::cerr<<"trade timeout, hauls "<<a->trader_.hauls.size()<<'\n';for(const auto& h:a->trader_.hauls){const auto& r=a->builder_robots_[h.robot];std::cerr<<"bot "<<(int)h.robot<<" phase "<<(int)h.phase<<" cargo "<<h.cargo<<" pos "<<r.position.x<<","<<r.position.z<<" goal "<<r.safety.goal.x<<","<<r.safety.goal.z<<" block "<<(int)r.safety.block_reason<<'\n';}
        }
        require(carried&&saved_loaded&&a->trader_.phase==TradePhase::Departing,"robots did not physically load all goods and clear the ship");
        require(a->trader_.loaded==a->trader_.requested&&a->trader_.hauls.empty(),"departed with missing goods or rover still on dock");
        a->update_trader(17);a=resume(*a,"trader-departing");a->update_trader(1);require(a->trader_.phase==TradePhase::Complete&&a->planning_.funds_eur==cash+value,"wrong or missing departure payment");
        a=resume(*a,"trader-paid");a->update_trader(100);require(a->planning_.funds_eur==cash+value,"cash paid twice after resume");
        require(!a->call_trader(0),"new call accepted after antenna removal");
        antenna.construction.step=kConstructionSteps;a->placed_resources_.push_back(antenna);
        require(a->call_trader(0)&&a->planning_.funds_eur==cash+value,"second trade visit failed or duplicated payment");
        // A real completed Storage Gate exports its dome inventory through
        // physical source lots, using the same robot/coordinator pipeline.
        GameApp storage(engine);setup(storage);storage.ground_resource_piles_.clear();
        PlacedResource dome;dome.kind=ResourceKind::StorageDome;dome.position={-36,0,-28};dome.placement_sequence=40;dome.construction.step=kConstructionSteps;
        storage.placed_resources_.push_back(dome);const auto di=storage.placed_resources_.size()-1;
        require(storage.add_storage_gate(di),"storage trade fixture could not place gate");
        for(auto& r:storage.placed_resources_)if(r.kind==ResourceKind::StorageGate)r.construction.step=kConstructionSteps;
        storage.synchronize_storage_gates();require(storage_loading::has_ready_gate(storage.placed_resources_[di]),"storage trade gate not ready");
        storage.placed_resources_[di].stored_boxes={{MaterialResource::Iron,1300,1},{MaterialResource::Silicon,900,1}};sync_stored_resource_totals(storage.placed_resources_[di]);
        ResourceBundle st;st.iron=1300;st.silicon=900;storage.resources_.restore(st);
        require(storage.call_trader(0),"storage shipment trader call failed");storage.update_trader(24);storage.trader_.requested[0]=1300;storage.trader_.requested[2]=900;
        require(storage.start_trader_sale(),"storage inventory could not be sealed for export: "+storage.trade_message_);
        require(storage.placed_resources_[di].stored_boxes.empty()&&storage.resources_.amounts().empty(),"storage inventory counted twice after reservation");
        for(int i=0;i<600*90&&storage.trader_.phase==TradePhase::Loading;++i){storage.update_trader(1.f/90);storage.update_construction(1.f/90);}
        require(storage.trader_.phase==TradePhase::Departing&&storage.trader_.loaded[0]==1300&&storage.trader_.loaded[2]==900,"storage robots did not physically load gate exports");
        // All supported zoom levels retain accessible trade controls after scrolling.
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom){GameApp p(engine);setup(p);p.ui_scale_index_=zoom;Capture c;c.resize(1280,720);p.render(c);p.selection_panel_scroll_=p.selection_panel_max_scroll();p.render(c);c.button("TRADE");p.call_trader(0);p.update_trader(24);p.render(c);p.selection_panel_scroll_=p.selection_panel_max_scroll();p.render(c);c.button("START LOADING");}
        for(auto phase:{TradePhase::Approach,TradePhase::Docked,TradePhase::Loading,TradePhase::Departing}){
            GameApp wire(engine);setup(wire);require(wire.call_trader(0),"wire ship fixture call");wire.trader_.phase=phase;wire.trader_.elapsed=phase==TradePhase::Approach?15:phase==TradePhase::Departing?5:0;
            Capture c;wire.render_trader(c);require(c.triangles.empty()&&!c.lines.empty(),"trader submits filled surfaces / missing wire mesh");
        }
        if(const char* output=std::getenv("TRADER_CAPTURE_DIR")){
            std::filesystem::create_directories(output);
            for(int frame=0;frame<3;++frame){GameApp picture(engine);setup(picture);picture.call_trader(0);picture.trader_.phase=frame==0?TradePhase::Approach:frame==1?TradePhase::Docked:TradePhase::Departing;picture.trader_.elapsed=frame==0?15:frame==2?5:0;picture.trader_.loaded[0]=5000;
                Capture c;picture.camera_.restore({42,36,35,{16,3,0}});c.set_3d(picture.camera_.view_projection(1600.f/900));picture.render_trader(c);c.write(std::filesystem::path(output)/("trader-"+std::to_string(frame)+".json"));}
        }
        std::cout<<"Trader PASS: powered pad latch, oxygen mass balance, physical robot loading, reservations, live save/resume, post-departure cash exactly once, repeat visits and UI scaling\n";
    }
};
}
int main(){try{game::TraderRegression::run();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
