#include "game/runtime/presentation.hpp"
#include "game/game_app.hpp"
#include <iostream>
#include <stdexcept>
namespace game {
struct StartupRegression {
    static bool load(GameApp& app,std::string& error){return app.load_game_from_file(std::string(GAME_SOURCE_ROOT)+"/game/tests/fixtures/schema48-lander-depot.gmsave",error);}
};
}
using namespace game::runtime;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Probe final:GameLoop {
    GameHost& host;bool& started;bool& shown;
    std::unique_ptr<game::GameApp> actual;
    Probe(GameHost& h,bool& s,bool& v):host(h),started(s),shown(v),actual(std::make_unique<game::GameApp>(h)){}
    void on_start() override{require(host.fullscreen(),"fullscreen was lost before game initialization");host.set_fullscreen(false);actual->on_start();require(host.fullscreen(),"preferences left the ready game windowed");started=true;}
    void on_resize(int w,int h) override{actual->on_resize(w,h);}
    void update(float dt,const InputState& in) override{actual->update(dt,in);}
    void render(SceneRenderer& r) override{
        require(started,"menu rendered before game readiness");actual->render(r);shown=true;
        std::string error;
        require(game::StartupRegression::load(*actual,error),error.c_str());require(host.fullscreen(),"legacy save disabled fullscreen");
        const auto display=host.current_display_mode();
        if(host.set_display_mode(display.width,display.height))require(host.fullscreen(),"exclusive fullscreen was misdetected as windowed");
        host.set_fullscreen(false);require(!host.fullscreen(),"explicit windowed toggle failed");
        require(game::StartupRegression::load(*actual,error),error.c_str());require(!host.fullscreen(),"save load changed this session's explicit window mode");
        host.set_fullscreen(true);require(host.fullscreen(),"fullscreen could not be restored");host.request_quit();
    }
};
int main(){try{
    GameHost host;WindowConfig config;require(config.fullscreen,"game startup default is windowed");config.title="Lunar Colony startup regression";
    bool factory=false,started=false,shown=false;
    const int result=host.run(config,[&]()->std::unique_ptr<GameLoop>{
        require(host.fullscreen(),"asset factory ran before fullscreen window creation");factory=true;return std::make_unique<Probe>(host,started,shown);
    });
    require(result==0&&factory&&started&&shown,"startup did not reach a ready fullscreen frame");
    std::cout<<"PASS default fullscreen before assets and readiness; legacy loads preserve mode; exclusive and windowed toggles; clean exit\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
