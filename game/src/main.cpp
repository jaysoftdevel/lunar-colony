#include "engine/engine.hpp"
#include "game/game_app.hpp"
#include <memory>

int main(){
    eng::Application app;
    eng::AppConfig cfg;
    cfg.width=1600;
    cfg.height=900;
    cfg.title="Game - Playfield Prototype";
    cfg.fullscreen=true;
    return app.run(cfg,std::make_unique<game::GameApp>(app));
}
