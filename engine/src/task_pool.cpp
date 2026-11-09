#include "engine/engine.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#include <system_error>
#ifdef __linux__
#include <sched.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace eng {
namespace {
thread_local bool background_context=false;
}

struct TaskPool::Impl {
    std::mutex mutex{};
    std::condition_variable cv{};
    struct Entry {
        std::shared_ptr<std::function<void()>> task;
        std::size_t copies;
        bool background;
    };
    std::deque<Entry> queue{};
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
#elif defined(_WIN32)
        // Windows 11 can expose multiple processor groups on very large hosts.
        // Count all active logical processors instead of assuming one 64-CPU
        // group. The Windows scheduler can distribute these unpinned workers.
        const DWORD active=GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
        if(active>0) hw=(unsigned)active;
#endif
        // Use every logical CPU that the host/launcher makes available to this
        // process. One lane is the game thread itself; every remaining lane is
        // backed by one persistent helper. The old ~50% cap kept the runtime
        // artificially CPU-bound on dense multi-rover saves even though the
        // expensive navigation jobs are independent and already synchronized
        // before authoritative world mutation.
        lanes=std::max<std::size_t>(1,(std::size_t)hw);
        if(const char* env=std::getenv("GAME_CPU_LANES")){
            char* end=nullptr;
            const long requested=std::strtol(env,&end,10);
            if(end!=env && requested>0) lanes=std::clamp<std::size_t>((std::size_t)requested,1,(std::size_t)hw);
        }
        const std::size_t helpers=lanes>1?lanes-1:0;
        workers.reserve(helpers);
        for(std::size_t i=0;i<helpers;++i){
            try {
                workers.emplace_back([this](){ worker_loop(); });
            } catch(const std::system_error&) {
                // A host thread limit must not abort startup after some workers
                // have already been launched.
                break;
            }
        }
        lanes=workers.size()+1;
    }

    void worker_loop(){
        for(;;){
            std::shared_ptr<std::function<void()>> task;
            {
                std::unique_lock lock(mutex);
                cv.wait(lock,[&](){ return stop || !queue.empty(); });
                if(stop && queue.empty()) return;
                auto& entry=queue.front();
                task=entry.task;
                background_context=entry.background;
                if(--entry.copies==0) queue.pop_front();
            }
            (*task)();
            background_context=false;
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
    auto packaged=std::make_shared<std::packaged_task<void()>>(std::move(task));
    auto future=packaged->get_future();
    if(!impl_ || impl_->workers.empty()){
        (*packaged)();
        return future;
    }
    post_shared(std::make_shared<std::function<void()>>([packaged](){ (*packaged)(); }),1);
    return future;
}

void TaskPool::post_shared(const std::shared_ptr<std::function<void()>>& task,std::size_t copies,bool joined){
    if(copies==0) return;
    {
        std::lock_guard lock(impl_->mutex);
        // Joined work can hold up the current frame. Background submissions
        // stay FIFO while new synchronous batches can recruit idle workers.
        if(joined && !background_context) impl_->queue.push_front({task,copies,false});
        else impl_->queue.push_back({task,copies,true});
    }
    if(copies==impl_->workers.size()) impl_->cv.notify_all();
    else for(std::size_t i=0;i<copies;++i) impl_->cv.notify_one();
}

void TaskPool::discard_shared(const std::shared_ptr<std::function<void()>>& task){
    std::lock_guard lock(impl_->mutex);
    // Once the caller has exhausted the index range, outstanding invitations
    // would only wake workers to discover there is nothing left to claim.
    std::erase_if(impl_->queue,[&](const auto& entry){ return entry.task==task; });
}

std::size_t cpu_parallel_lanes(){ return TaskPool::shared().parallel_lanes(); }

} // namespace eng
