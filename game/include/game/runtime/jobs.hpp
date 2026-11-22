#pragma once
#include <frontier/jobs.hpp>
#include <algorithm>
#include <future>
#include <memory>
#include <string>
namespace game::runtime {
struct CpuWorkerInfo {std::size_t available{},lanes{},assigned_workers{};bool affinity_enabled{};};
// Game call sites delegate scheduling and nested work to the external JSD pool.
class GameJobs {
    std::unique_ptr<rf::Jobs> jobs_;
    std::size_t available_=rf::availableHardwareThreads();
public:
    GameJobs();
    static GameJobs& shared(){static GameJobs instance;return instance;}
    std::size_t background_workers()const{return jobs_?jobs_->count():0;}
    std::size_t parallel_lanes()const{return background_workers()+1;}
    CpuWorkerInfo cpu_info()const{return {available_,parallel_lanes(),0,false};}
    template<class Fn> auto submit(Fn&& fn){
        using Result=std::invoke_result_t<std::decay_t<Fn>&>;
        if(jobs_)return jobs_->submit(std::forward<Fn>(fn));
        std::packaged_task<Result()> task(std::forward<Fn>(fn));auto future=task.get_future();task();return future;
    }
    template<class Fn>void parallel_for(std::size_t count,Fn&& fn){
        if(!count)return;
        if(!jobs_){for(std::size_t i=0;i<count;++i)fn(i);return;}
        jobs_->parallelFor(count,1,[&](std::size_t first,std::size_t last){for(auto i=first;i<last;++i)fn(i);});
    }
    template<class Fn>void parallel_for_ranges(std::size_t count,std::size_t grain,Fn&& fn){
        if(!count)return;
        if(!jobs_){fn(0,count);return;}
        jobs_->parallelFor(count,std::max<std::size_t>(1,grain),std::forward<Fn>(fn));
    }
};
std::size_t cpu_parallel_lanes();
std::string cpu_worker_summary();
}
