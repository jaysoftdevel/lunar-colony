#include "engine/engine.hpp"
#include "cpu_affinity.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <new>
#include <string_view>
#include <thread>
#include <system_error>

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
    cpu::Affinity affinity=cpu::discover();
    std::size_t available=affinity.processors.empty()?
        std::max(1u,std::thread::hardware_concurrency()):affinity.processors.size();
    std::size_t lanes{1};
    bool affinity_enabled=true;
    std::atomic<std::size_t> initialized{},assigned{};

    Impl(){
        lanes=available;
        if(const char* env=std::getenv("GAME_CPU_LANES")){
            char* end=nullptr;
            const long requested=std::strtol(env,&end,10);
            if(end!=env && requested>0) lanes=std::min(available,static_cast<std::size_t>(requested));
        }
        if(const char* env=std::getenv("GAME_CPU_PIN")) affinity_enabled=std::string_view(env)!="0";
        workers.reserve(lanes-1);
        // Leave the application thread and driver-created threads free to
        // use their original masks. A worker selects its own CPU on entry.
        for(std::size_t index=1;index<lanes;++index){
            try {
                workers.emplace_back([this,index](){
                    if(affinity_enabled && cpu::select_current_thread(affinity,index))
                        assigned.fetch_add(1,std::memory_order_relaxed);
                    initialized.fetch_add(1,std::memory_order_acq_rel);
                    initialized.notify_one();
                    worker_loop();
                });
            } catch(const std::system_error&) {
                break;
            } catch(const std::bad_alloc&) {
                break;
            }
        }
        lanes=workers.size()+1;
        for(auto ready=initialized.load(std::memory_order_acquire);ready<workers.size();
            ready=initialized.load(std::memory_order_acquire))
            initialized.wait(ready);
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
CpuWorkerInfo TaskPool::cpu_info() const {
    return {impl_->available,impl_->lanes,impl_->assigned.load(std::memory_order_relaxed),impl_->affinity_enabled};
}

void TaskPool::enqueue(std::function<void()> task){
    if(background_workers()==0){
        task();
        return;
    }
    post_shared(std::make_shared<std::function<void()>>(std::move(task)),1);
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

std::string cpu_worker_summary(){
    const auto info=TaskPool::shared().cpu_info();
    auto text=std::string("CPU lanes ")+std::to_string(info.lanes)+"/"+std::to_string(info.available)+
        " available ("+std::to_string(info.lanes-1)+" workers + application thread";
    if(info.affinity_enabled) text+=", "+std::to_string(info.assigned_workers)+" workers assigned to CPU sets";
    else text+=", worker placement off";
    return text+")";
}

} // namespace eng
