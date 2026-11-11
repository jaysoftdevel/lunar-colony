#include "engine/engine.hpp"
#if defined(__linux__)
#include "../src/cpu_affinity.hpp"
#endif

#include <array>
#include <atomic>
#include <barrier>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(){
    try {
#if defined(__linux__)
        auto allowed=eng::cpu::discover().processors;
        std::sort(allowed.begin(),allowed.end());
        const auto caller=std::this_thread::get_id();
#endif
        auto& pool=eng::TaskPool::shared();
        const auto info=pool.cpu_info();
        const auto lanes=info.lanes;
        std::barrier occupied(static_cast<std::ptrdiff_t>(lanes));
        std::atomic<std::size_t> participants{};
#if defined(__linux__)
        std::vector<std::vector<unsigned>> worker_masks(lanes);
#endif
        // Each callback holds its lane until every other lane has claimed
        // work, proving the pool recruits all available helpers under load.
        pool.parallel_for(lanes,[&]([[maybe_unused]] std::size_t slot){
#if defined(__linux__)
            if(std::this_thread::get_id()!=caller) worker_masks[slot]=eng::cpu::discover().processors;
#endif
            participants.fetch_add(1,std::memory_order_relaxed);
            occupied.arrive_and_wait();
        });
        if(participants.load()!=lanes || pool.background_workers()+1!=lanes)
            throw std::runtime_error("CPU lane count does not match participants");
        if(lanes>info.available || (!info.affinity_enabled && info.assigned_workers!=0))
            throw std::runtime_error("worker placement summary is inconsistent");
#if defined(__linux__)
        auto remaining=eng::cpu::discover().processors;
        std::sort(remaining.begin(),remaining.end());
        if(allowed!=remaining || (!allowed.empty() && info.available!=allowed.size()))
            throw std::runtime_error("the caller's permitted CPU set changed");
        if(info.affinity_enabled && lanes>1 && info.assigned_workers==lanes-1){
            std::vector<unsigned> assigned;
            for(const auto& mask:worker_masks){
                if(mask.empty()) continue;
                if(mask.size()!=1 || !std::binary_search(allowed.begin(),allowed.end(),mask.front()))
                    throw std::runtime_error("a worker is outside the permitted CPU set");
                assigned.push_back(mask.front());
            }
            std::sort(assigned.begin(),assigned.end());
            if(assigned.size()!=info.assigned_workers || std::adjacent_find(assigned.begin(),assigned.end())!=assigned.end())
                throw std::runtime_error("workers were assigned overlapping CPU sets");
        }
#endif

        std::array<std::atomic<unsigned>,1024> visits{};
        auto background=pool.submit([&]{
            pool.parallel_for(16,[&](std::size_t section){
                pool.parallel_for(64,[&](std::size_t index){
                    ++visits[section*64+index];
                });
            });
        });
        background.get();
        for(const auto& count:visits)
            if(count.load()!=1) throw std::runtime_error("nested job lost or duplicated an index");

        std::array<std::atomic<unsigned>,1021> ranged_visits{};
        pool.parallel_for_ranges(ranged_visits.size(),19,[&](std::size_t first,std::size_t last){
            if(first>=last || last>ranged_visits.size()) throw std::runtime_error("invalid work range");
            for(auto i=first;i<last;++i) ++ranged_visits[i];
        });
        for(const auto& count:ranged_visits)
            if(count.load()!=1) throw std::runtime_error("work ranges overlap or leave a gap");
        if(pool.submit([]{ return 54; }).get()!=54) throw std::runtime_error("background result was lost");

        bool caught=false;
        try {
            pool.parallel_for(32,[](std::size_t i){
                if(i==5) throw std::logic_error("job failure");
            });
        } catch(const std::logic_error&) {
            caught=true;
        }
        if(!caught) throw std::runtime_error("job exception did not reach the caller");
        pool.submit([]{}).get();
        std::cout<<eng::cpu_worker_summary()<<". Range coverage, nested batches and exception recovery passed.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        return 1;
    }
}
