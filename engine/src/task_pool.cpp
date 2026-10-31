#include "engine/engine.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#ifdef __linux__
#include <sched.h>
#endif

namespace eng {

struct TaskPool::Impl {
    std::mutex mutex{};
    std::condition_variable cv{};
    std::deque<std::packaged_task<void()>> queue{};
    std::vector<std::thread> workers{};
    bool stop{false};
    std::size_t lanes{1};

    Impl(){
        unsigned hw=std::thread::hardware_concurrency();
        if(hw==0) hw=2;
#ifdef __linux__
        cpu_set_t allowed{};
        if(sched_getaffinity(0,sizeof(allowed),&allowed)==0){
            const int affinity_cpus=CPU_COUNT(&allowed);
            if(affinity_cpus>0) hw=(unsigned)affinity_cpus;
        }
#endif
        // Runtime simulation is allowed to occupy at most half of the visible
        // logical CPUs. One lane is the game thread itself; the rest are
        // persistent helpers, so route preparation can scale without spawning
        // transient threads every planner tick.
        lanes=std::max<std::size_t>(1,(std::size_t)hw/2u);
        if(const char* env=std::getenv("GAME_CPU_LANES")){
            char* end=nullptr;
            const long requested=std::strtol(env,&end,10);
            if(end!=env && requested>0) lanes=std::clamp<std::size_t>((std::size_t)requested,1,(std::size_t)hw);
        }
        const std::size_t helpers=lanes>1?lanes-1:0;
        workers.reserve(helpers);
        for(std::size_t i=0;i<helpers;++i){
            workers.emplace_back([this](){
                for(;;){
                    std::packaged_task<void()> task;
                    {
                        std::unique_lock lock(mutex);
                        cv.wait(lock,[&](){ return stop || !queue.empty(); });
                        if(stop && queue.empty()) return;
                        task=std::move(queue.front());
                        queue.pop_front();
                    }
                    task();
                }
            });
        }
    }

    ~Impl(){
        {
            std::lock_guard lock(mutex);
            stop=true;
        }
        cv.notify_all();
        for(auto& worker:workers) if(worker.joinable()) worker.join();
    }
};

TaskPool::TaskPool():impl_(std::make_unique<Impl>()){}
TaskPool::~TaskPool()=default;
TaskPool& TaskPool::shared(){ static TaskPool pool; return pool; }
std::size_t TaskPool::parallel_lanes() const { return impl_?impl_->lanes:1; }
std::size_t TaskPool::background_workers() const { return impl_?impl_->workers.size():0; }

std::future<void> TaskPool::submit(std::function<void()> task){
    std::packaged_task<void()> packaged(std::move(task));
    auto future=packaged.get_future();
    if(!impl_ || impl_->workers.empty()){
        packaged();
        return future;
    }
    {
        std::lock_guard lock(impl_->mutex);
        impl_->queue.emplace_back(std::move(packaged));
    }
    impl_->cv.notify_one();
    return future;
}

std::size_t cpu_parallel_lanes(){ return TaskPool::shared().parallel_lanes(); }

} // namespace eng
