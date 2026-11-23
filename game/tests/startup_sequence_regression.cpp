#include "game/runtime/presentation.hpp"
#include "game/game_app.hpp"
#include <iostream>
#include <stdexcept>
using namespace game::runtime;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Probe final:GameLoop {
    GameHost& host;bool& started;bool& shown;
    std::unique_ptr<game::GameApp> actual;
    Probe(GameHost& h,bool& s,bool& v):host(h),started(s),shown(v),actual(std::make_unique<game::GameApp>(h)){}
    void on_start() override{require(host.fullscreen(),"fullscreen was lost before game initialization");actual->on_start();started=true;}
    void on_resize(int w,int h) override{actual->on_resize(w,h);}
    void update(float dt,const InputState& in) override{actual->update(dt,in);}
    void render(SceneRenderer& r) override{
        require(started,"menu rendered before game readiness");actual->render(r);shown=true;host.request_quit();
    }
};
int main(){try{
    GameHost host;WindowConfig config;config.fullscreen=true;config.title="Lunar Colony startup regression";
    bool factory=false,started=false,shown=false;
    const int result=host.run(config,[&]()->std::unique_ptr<GameLoop>{
        require(host.fullscreen(),"asset factory ran before fullscreen window creation");factory=true;return std::make_unique<Probe>(host,started,shown);
    });
    require(result==0&&factory&&started&&shown,"startup did not reach a ready fullscreen frame");
    std::cout<<"PASS fullscreen precedes asset creation; first presentation follows on_start; clean exit\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
