#include "engine/engine.hpp"
#include "game/game_app.hpp"
#include "game/version.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

#if defined(_WIN32) && defined(GAME_D3D12_AGILITY_VERSION)
extern "C" {
__declspec(dllexport) extern const unsigned int D3D12SDKVersion=GAME_D3D12_AGILITY_VERSION;
__declspec(dllexport) const char* D3D12SDKPath=".\\D3D12\\";
}
#endif

int main(int argc,char** argv){
    if(argc>1 && (std::string_view(argv[1])=="--version" || std::string_view(argv[1])=="-v")){
        std::cout<<"game "<<game::version::current<<"\n";
        return 0;
    }
    if(argc>1 && std::string_view(argv[1])=="--build-mode"){
        std::cout<<(game::build::debug_features?"Debug":"Release")<<"\n";
        return 0;
    }
    eng::Application app;
    eng::AppConfig cfg;
    cfg.width=1600;
    cfg.height=900;
    cfg.title="Game v"+std::string(game::version::current);
    if constexpr(game::build::debug_features) cfg.title+=" [Debug]";
    cfg.fullscreen=true;
    return app.run(cfg,std::make_unique<game::GameApp>(app));
}
