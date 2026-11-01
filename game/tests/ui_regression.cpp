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
            case 5:app.screen_=Screen::LoadGame;app.save_games_={{std::string(64,'M'),"test.gmsave","0.43.5"},{"SECOND COLONY","two.gmsave","0.43.5"}};break;
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
                h.supply.air_change_percentage_points_per_day=scene==13?0.125:-0.125;h.supply.oxygen_net_kg_per_day=scene==13?0.5:-0.5;h.supply.oxygen_produced_kg_per_day=9;h.supply.water_processed_litres_per_day=10.125;
                app.selection_kind_=GameApp::SelectionKind::Habitat;app.selected_habitat_=0;app.game_speed_=GameApp::GameSpeed::Normal;
            }
            if(scene==15){CorridorConnection corridor;corridor.path={{-8,0,0},{8,0,0}};corridor.construction.step=corridor.construction.total_steps;corridor.supply.volume_m3=64;corridor.supply.oxygen_kg=64*kOxygenKgPerM3*0.5;
                app.corridor_network_.restore_connections({corridor});app.selection_kind_=GameApp::SelectionKind::Corridor;app.selected_corridor_=0;}
            if(scene==12){app.first_person_mode_=GameApp::FirstPersonMode::Active;app.first_person_.spawn({-12,0,-12},45);}
        }
    }
    static void run(const std::string& output){
        eng::Application engine;
        constexpr std::array<eng::Vec2,5> sizes{{{1280,720},{1600,900},{1920,1080},{2560,1440},{3840,2160}}};
        for(int zoom=0;zoom<layout::UI_SCALE_COUNT;++zoom)for(int scene=0;scene<16;++scene){
            Recorder reference;
            for(std::size_t n=0;n<sizes.size();++n){
                GameApp app(engine);setup(app,scene,zoom);Recorder r;r.resize((int)sizes[n].x,(int)sizes[n].y);app.render(r);
                const std::string label="scene "+std::to_string(scene)+" zoom "+std::to_string(zoom)+" "+std::to_string(r.w);
                r.check_bounds(label);
                const auto has_text=[&](const std::string& part){for(const auto& d:r.draws)if(d.type=='T' && d.text.find(part)!=std::string::npos)return true;return false;};
                if(scene==8 || scene==9 || scene>=13)require(has_text("BREATHABLE AIR") && has_text("AIR CHANGE"),"indoor details omit the air percentage/change rate");
                if(scene==13)require(has_text("75.0%") && has_text("+0.125") && has_text("9.000 KG/DAY / WATER 10.125 L/DAY") && has_text("CONVERTING WATER TO OXYGEN"),"generator details do not show measured intake/output");
                if(scene==14)require(has_text("-0.125"),"depleting interior details hide negative air change");
                if(n==0)reference=r;else compare(reference,r,label);
                if(!output.empty() && ((zoom==2 && (n==0 || n==4)) || (zoom==4 && n==0)))r.write(std::filesystem::path(output)/("ui-"+std::to_string(scene)+"-"+std::to_string(r.w)+"-zoom"+std::to_string(zoom)+".json"));
            }
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
        std::cout<<"400 resolution/zoom/screen cases, HD-to-4K proportions, hitboxes, frame-boundary zoom, aspect ratios, button text and shared font/stroke metrics passed\n";
    }
};
}
int main(int argc,char** argv){try{const std::string out=argc>1?argv[1]:"";if(!out.empty())std::filesystem::create_directories(out);game::UiRegression::run(out);}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}
