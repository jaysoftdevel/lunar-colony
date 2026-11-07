#include "game/game_app.hpp"
#include "game/ui/layout.hpp"
#include "game/systems/construction_costs.hpp"
#include "game/util/world_math.hpp"
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
#ifndef GAME_LEGACY_LANDER_FIXTURES
#define GAME_LEGACY_LANDER_FIXTURES "game/tests/fixtures"
#endif

namespace game {
struct LanderRegression {
    static ResourceBundle cargo(){ResourceBundle b;b.iron=1000;b.food=600;b.water=700;b.silicon=333;return b;}
    static void setup(GameApp& a){a.playfield_size_=128;a.playfield_.rebuild(128);a.on_resize(1600,900);a.screen_=GameApp::Screen::Playfield;a.active_game_phase_=GameApp::GamePhase::RobotPreparation;a.game_session_active_=true;a.status_.reset();a.begin_lunar_landing(cargo(),1,1);}
    static eng::Vec2 screen(GameApp& a,eng::Vec3 p){auto m=a.camera_.view_projection(1600.f/900);float w=m.m[3]*p.x+m.m[7]*p.y+m.m[11]*p.z+m.m[15];return {(m.m[0]*p.x+m.m[4]*p.y+m.m[8]*p.z+m.m[12])/w*800+800,450-(m.m[1]*p.x+m.m[5]*p.y+m.m[9]*p.z+m.m[13])/w*450};}
    static void click(GameApp& a,eng::Vec3 p){auto s=screen(a,p);eng::InputState in;in.mouse_x=(int)s.x;in.mouse_y=(int)s.y;in.left_pressed=true;a.update(0,in);}
    static void roundtrip(GameApp& a,const std::string& name){std::string error;require(a.save_game_to_name(name,error),error);auto path=std::filesystem::path(a.save_game_directory())/(name+".gmsave");GameApp b(a.app_);require(b.load_game_from_file(path.string(),error),error);require(b.lander_.awaiting_site==a.lander_.awaiting_site&&b.lander_.awaiting_stash==a.lander_.awaiting_stash&&b.lander_.salvaged==a.lander_.salvaged&&b.lander_.boxed==a.lander_.boxed,"placement/salvage flags lost on load");require(eng::length(b.lander_.stash_center-a.lander_.stash_center)<.001f&&b.ground_resource_piles_.size()==a.ground_resource_piles_.size(),"stash/boxes lost on load");require(b.resources_.amounts().total_amount()==a.resources_.amounts().total_amount(),"inventory changed on resume");std::filesystem::remove(path);}
    static void unload_queue(eng::Application& engine){
        using namespace world_math;
        // A full 12-rover cabin is the tightest case. Also sample each legacy
        // parked prefix against the new queue and the remaining parking bays.
        LunarLanderState lander;
        lander.builders=lander.carriers=6;
        for(int first=0;first<12;++first){
            const float duration=lunar_lander_deployment_seconds(lander,first);
            bool simultaneous=false;
            for(lander.elapsed=0;lander.elapsed<=duration;lander.elapsed+=.02f){
                std::array<LanderRoverPose,12> poses{};
                int on_ramp=0;
                for(int i=0;i<12;++i){
                    auto& pose=poses[(std::size_t)i];
                    pose=i<first?LanderRoverPose{lunar_lander_robot_bay(lander,i),180,0,true}
                        :lunar_lander_robot_pose(lander,i,first);
                    on_ramp+=pose.position.z>3.7f&&pose.position.z<10.5f&&pose.position.y>0;
                    for(int j=0;j<i;++j){
                        const auto& other=poses[(std::size_t)j];
                        const auto a=footprint(pose.position,pose.heading,1.24f,1.54f);
                        const auto b=footprint(other.position,other.heading,1.24f,1.54f);
                        require(!overlaps(a,b,.06f),"unload queue caused rover hull overlap");
                    }
                }
                simultaneous|=on_ramp>=2;
            }
            if(first<11) require(simultaneous,"rovers did not follow one another down the ramp");
        }

        GameApp app(engine);setup(app);
        app.begin_lunar_landing(cargo(),6,6);
        require(app.prepare_default_landing(),"fleet deployment site");
        app.update_lunar_lander(1);
        const auto view=app.camera_.state();
        eng::InputState orbit;
        orbit.middle_down=true;
        orbit.mouse_dx=20;
        orbit.mouse_dy=12;
        orbit.wheel=1;
        orbit.mouse_x=900;
        orbit.mouse_y=420;
        app.update(.05f,orbit);
        const auto controlled=app.camera_.state();
        app.update(.05f,{});
        require(app.lander_.in_flight()&&controlled.azimuth_degrees!=view.azimuth_degrees&&
                controlled.elevation_degrees!=view.elevation_degrees&&controlled.distance<view.distance&&
                app.camera_.state().azimuth_degrees==controlled.azimuth_degrees,
                "approach camera ignored or reset player input");
        app.update_lunar_lander(200);
        require(app.unload_lunar_lander(),"fleet unload");
        app.update_lunar_lander(kLanderRampSeconds+7);
        require(app.lander_.phase==LanderPhase::DeployingRobots,"fleet queue not active");
        const auto expected=lunar_lander_robot_pose(app.lander_,2,0);
        std::string error;
        require(app.save_game_to_name("rover-queue-mid-ramp",error),error);
        const auto path=std::filesystem::path(app.save_game_directory())/"rover-queue-mid-ramp.gmsave";
        GameApp resumed(engine);
        require(resumed.load_game_from_file(path.string(),error),error);
        require(eng::length(lunar_lander_robot_pose(resumed.lander_,2,0).position-expected.position)<.001f,
                "unload queue moved on save/load");
        resumed.update_lunar_lander(100);
        require(resumed.lander_.phase==LanderPhase::Ready&&resumed.builder_robots_.size()==12,
                "full fleet did not finish parking");
        for(int i=0;i<12;++i)
            require(eng::length(resumed.builder_robots_[(std::size_t)i].position-lunar_lander_robot_bay(resumed.lander_,i))<.001f,
                    "rover did not reach its assigned bay");
        // This file was produced by the schema-35 game while its second
        // rover was on the ramp. The parked prefix must survive migration.
        const auto old_path=std::filesystem::path(GAME_LEGACY_LANDER_FIXTURES)/"schema35-mid-ramp.gmsave";
        GameApp legacy(engine);
        require(legacy.load_game_from_file(old_path.string(),error),error);
        const eng::Vec3 parked{11.6f,0.0f,-6.4f};
        require(legacy.builder_robots_.size()==1&&legacy.lander_.legacy_parking&&legacy.lander_.elapsed==0&&
                eng::length(legacy.builder_robots_[0].position-parked)<.001f,
                "legacy unload did not preserve its parked rover");
        legacy.update_lunar_lander(100);
        require(legacy.builder_robots_.size()==12,"legacy unload lost or duplicated a rover");
        std::filesystem::remove(path);
    }
    static void run(const char* attached_save){
        eng::Application engine;
        unload_queue(engine);
        GameApp a(engine);setup(a);
        if(attached_save){
            GameApp saved(engine);
            std::string error;
            require(saved.load_game_from_file(attached_save,error),error);
            std::cout<<"Attached save: "<<saved.playfield_.size()<<" m field, "
                     <<saved.lander_.manifest.total_amount()<<" kg cargo, phase "<<(int)saved.lander_.phase<<"\n";
            require(saved.prepare_default_landing(),"attached save could not place compact marker");
            saved.update_lunar_lander(200);
            if(saved.lander_.phase==LanderPhase::Landed) require(saved.unload_lunar_lander(),"attached save could not place stacks");
            std::cout<<"Attached save: "<<saved.ground_resource_piles_.size()<<" stacks around the 2.5 m marker\n";
        }
        require(a.lander_.awaiting_site,"new deployment did not ask for landing position");a.update_lunar_lander(200);require(a.lander_.elapsed==0&&a.builder_robots_.empty(),"lander descended before mouse selection");roundtrip(a,"site-choice");
        require(!a.confirm_lander_site({1000,0,0}),"off-map landing accepted");click(a,{0,0,0});require(!a.lander_.awaiting_site&&a.lander_.awaiting_stash,"mouse touchdown selection failed");
        require(!a.confirm_stash_site(a.lander_.site),"stash overlaps ship");require(a.prepare_default_landing(),"no default clear stash location");const auto location=a.lander_.stash_center;a.lander_.awaiting_stash=true;click(a,location);require(!a.lander_.awaiting_stash,"mouse stash choice failed");
        a.update_lunar_lander(200);require(a.lander_.phase==LanderPhase::Landed,"powered touchdown failed");require(!a.salvage_lander(),"salvage accepted with robots aboard");
        require(a.unload_lunar_lander(),"unload failed");require(a.lander_.remaining.empty()&&a.resources_.amounts().total_amount()==cargo().total_amount(),"cargo not dropped immediately");
        std::int64_t physical=0;
        bool stacked=false;
        for(const auto& b:a.ground_resource_piles_){
            const auto capacity=resource_package_capacity_kg(b.resource);
            require(b.kind==GroundPileKind::Stash&&b.amount>0&&b.amount<=capacity*kGroundStackLayers,"invalid stash stack");
            physical+=b.amount;
            stacked|=b.amount>capacity;
        }
        require(physical==cargo().total_amount()&&stacked,"stacking changed box mass or did not stack");
        for(std::size_t i=0;i<a.ground_resource_piles_.size();++i)for(std::size_t j=0;j<i;++j)require(eng::length(a.ground_resource_piles_[i].position-a.ground_resource_piles_[j].position)>a.ground_resource_piles_[i].radius+a.ground_resource_piles_[j].radius+.05f,"resource stacks overlap");
        a.update_lunar_lander(400);require(a.lander_.phase==LanderPhase::Ready&&a.builder_robots_.size()==2,"ramp robot deployment failed");
        const auto box=a.ground_resource_piles_.front();auto s=screen(a,box.position);a.select_world_at((int)s.x,(int)s.y);require(a.selection_kind_==GameApp::SelectionKind::GroundResource,"individual box not mouse selectable");
        s=screen(a,a.lander_.stash_center);a.select_world_at((int)s.x,(int)s.y);
        require(a.selection_kind_==GameApp::SelectionKind::Stash,"stash marker not mouse selectable");
        for(auto phase:{LanderPhase::Approach,LanderPhase::Landed,LanderPhase::OpeningRamp,LanderPhase::DeployingRobots,LanderPhase::Ready}){
            const auto saved=a.lander_;a.lander_.phase=phase;a.lander_.elapsed=phase==LanderPhase::Approach?70:2;
            Capture c;a.render_lunar_lander(c);require(c.triangles.empty()&&!c.lines.empty(),"lander submits filled surfaces / missing wire mesh");a.lander_=saved;
        }
        if(const char* output=std::getenv("LANDER_CAPTURE_DIR")){
            std::filesystem::create_directories(output);Capture c;
            a.camera_.restore({25,25,29,{0,2,0}});c.set_3d(a.camera_.view_projection(1600.f/900));a.render_lunar_lander(c);c.write(std::filesystem::path(output)/"lander.json");
        }
        // The schema-33 game wrote one aggregate pile per material.
        {
            const auto old_path=std::filesystem::path(GAME_LEGACY_LANDER_FIXTURES)/"schema33-aggregate-depot.gmsave";
            GameApp resumed(engine);
            std::string error;
            require(resumed.load_game_from_file(old_path.string(),error),error);
            require(resumed.lander_.awaiting_stash&&!resumed.lander_.boxed,"legacy depot not awaiting mouse area");
            require(resumed.prepare_default_landing(),"legacy depot repacking failed");
            std::int64_t amount=0;
            for(const auto& pile:resumed.ground_resource_piles_){
                amount+=pile.amount;
                require(pile.kind==GroundPileKind::Stash&&pile.amount<=kGroundStackLayers*resource_package_capacity_kg(pile.resource),
                        "legacy aggregate remains");
            }
            require(amount==cargo().total_amount()&&resumed.resources_.amounts().total_amount()==amount,
                    "legacy repack changed inventory");
        }
        roundtrip(a,"stacked-depot");
        const auto before=a.resources_.amounts().total_amount();
        Capture intact;a.render_lunar_lander(intact);
        require(a.salvage_lander(),"ready lander not salvageable");
        require(a.resources_.amounts().total_amount()==before&&a.lander_.active(),"salvage awarded before builder work");
        bool cutting=false;
        for(int i=0;i<120*90&&a.lander_.salvage_seconds<6;++i){
            a.update_construction(1.f/90);
            cutting|=a.builder_robots_[0].action==BuilderRobotAction::Dismantle;
        }
        require(cutting&&a.lander_.salvage_seconds>=6,"builder did not reach the ship and begin cutting");
        require(a.lander_.active()&&a.resources_.amounts().total_amount()==before,"salvage paid during cutting");
        Capture cut;a.render_lunar_lander(cut);
        require(cut.triangles.empty()&&!cut.lines.empty()&&cut.lines.size()<intact.lines.size(),"cutting did not remove a lander module");
        std::string error;
        require(a.save_game_to_name("salvage-in-progress",error),error);
        const auto pending_save=std::filesystem::path(a.save_game_directory())/"salvage-in-progress.gmsave";
        GameApp resumed(engine);
        require(resumed.load_game_from_file(pending_save.string(),error),error);
        require(resumed.lander_.salvage_seconds==a.lander_.salvage_seconds&&resumed.lander_.salvage_worker==0,
                "save lost builder salvage assignment/progress");
        for(int i=0;i<60*90&&resumed.lander_.active();++i)resumed.update_construction(1.f/90);
        require(resumed.resources_.amounts().total_amount()==before+5400&&!resumed.lander_.active(),"resumed salvage failed");
        std::filesystem::remove(pending_save);
        for(int i=0;i<60*90&&a.lander_.active();++i)a.update_construction(1.f/90);
        require(a.resources_.amounts().total_amount()==before+5400&&!a.lander_.active(),"salvage did not return resources/remove hull");
        require(!a.salvage_lander()&&a.resources_.amounts().total_amount()==before+5400,"salvage duplicated resources");
        roundtrip(a,"salvaged-depot");
        require(!lunar_lander_segment_blocked(a.lander_,{0,0,0},{0,0,0},1),"salvaged hull still blocks navigation");
        // Ordinary logistics can collect real boxes and complete construction.
        PlacedResource solar;solar.kind=ResourceKind::SolarPanel;solar.size=ResourceSizeTier::Small;solar.position={25,0,-20};solar.placement_sequence=1;solar.resource_plan=construction_costs::structure(solar.kind,a.resource_models_.mesh(solar));a.placed_resources_.push_back(solar);
        for(int i=0;i<450*90&&!a.placed_resources_[0].construction.complete();++i)a.update_construction(1.f/90);
        if(!a.placed_resources_[0].construction.complete()){
            std::cerr<<"build stage "<<(int)a.placed_resources_[0].construction.step<<" stash "<<a.lander_.stash_center.x<<","<<a.lander_.stash_center.z<<" stock "<<a.resources_.amounts().total_amount()<<"\n";
            for(const auto& r:a.builder_robots_)std::cerr<<"robot "<<(int)r.role<<" mode "<<(int)r.mode<<" pos "<<r.position.x<<","<<r.position.z<<" station "<<r.interaction_position.x<<","<<r.interaction_position.z<<" cargo "<<r.cargo.total_amount()<<" block "<<(int)r.safety.block_reason<<"\n";
        }
        require(a.placed_resources_[0].construction.complete(),"robots could not build using the boxed stash");
        for(float t=0;t<79;t+=.1f){auto f=lunar_lander_flight(t);require(f.offset.y>=0&&f.velocity.y<=.01f&&f.thrust_acceleration<2.5f,"descent profile regressed");}
        GameApp free(engine);free.playfield_size_=64;free.playfield_.rebuild(64);free.begin_lunar_landing(free.starter_resource_reserve(),6,6);require(free.prepare_default_landing(),"default free-play manifest cannot fit its physical stash");free.update_lunar_lander(200);require(free.unload_lunar_lander(),"full default manifest boxes do not fit");roundtrip(free,"full-cargo-boxes");
        bool paired_stacks=false;
        for(const auto& a:free.ground_resource_piles_) for(const auto& b:free.ground_resource_piles_){
            const auto delta=a.position-b.position;
            paired_stacks|=std::abs(delta.z)<.001f&&std::abs(std::abs(delta.x)-.60f)<.001f;
        }
        require(paired_stacks,"full cargo did not produce closely paired stacks");
        std::cout<<"Lander PASS: landing/marker picks, cargo stacks, 12-rover unload queue and mid-ramp save/resume, builder travel/cutting, module removal, salvage save/resume and one-time yield, crate-fed construction and full cargo on 64 m field\n";
    }
};
}
int main(int argc,char** argv){try{game::LanderRegression::run(argc>1?argv[1]:nullptr);return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
