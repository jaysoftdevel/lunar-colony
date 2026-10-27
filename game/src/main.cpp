#include "engine/engine.hpp"
#include "game/game_app.hpp"
#include "game/version.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

int main(int argc,char** argv){
    if(argc>1 && (std::string_view(argv[1])=="--version" || std::string_view(argv[1])=="-v")){
        std::cout<<"game "<<game::version::current<<"\n";
        return 0;
    }
    eng::Application app;
    eng::AppConfig cfg;
    cfg.width=1600;
    cfg.height=900;
    cfg.title="Game v"+std::string(game::version::current);
    cfg.fullscreen=true;
    return app.run(cfg,std::make_unique<game::GameApp>(app));
}
