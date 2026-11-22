#include "game/runtime/jobs.hpp"
#include <cstdlib>
#include <sstream>
namespace game::runtime {
GameJobs::GameJobs(){
    std::size_t lanes=available_;
    if(const char* setting=std::getenv("GAME_CPU_LANES")){
        char* end=nullptr;const auto requested=std::strtoul(setting,&end,10);
        if(end!=setting && !*end && requested>0)lanes=std::clamp<std::size_t>(requested,1,available_);
    }else if(const char* setting=std::getenv("JSD_WORKERS")){
        char* end=nullptr;const auto requested=std::strtoul(setting,&end,10);
        if(end!=setting && !*end && requested>0)lanes=std::min<std::size_t>(requested+1,available_);
    }
    if(lanes>1)jobs_=std::make_unique<rf::Jobs>(static_cast<unsigned>(lanes-1));
}
std::size_t cpu_parallel_lanes(){return GameJobs::shared().parallel_lanes();}
std::string cpu_worker_summary(){
    const auto info=GameJobs::shared().cpu_info();std::ostringstream out;
    out<<"JSD jobs: "<<info.lanes<<" lanes / "<<info.available<<" available CPUs; "<<info.lanes-1<<" engine workers";
    return out.str();
}
}
