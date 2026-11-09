#include "engine/engine.hpp"

#include <array>
#include <atomic>
#include <barrier>
#include <iostream>
#include <stdexcept>

int main(){
    try {
        auto& pool=eng::TaskPool::shared();
        const auto lanes=pool.parallel_lanes();
        std::barrier occupied(static_cast<std::ptrdiff_t>(lanes));
        std::atomic<std::size_t> participants{};
        // Each callback holds its lane until every other lane has claimed
        // work, proving the pool recruits all available helpers under load.
        pool.parallel_for(lanes,[&](std::size_t){
            participants.fetch_add(1,std::memory_order_relaxed);
            occupied.arrive_and_wait();
        });
        if(participants.load()!=lanes || pool.background_workers()+1!=lanes)
            throw std::runtime_error("CPU lane count does not match participants");

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
        std::cout<<"Task pool used "<<lanes<<" CPU lanes. Nested batches and exception recovery passed.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        return 1;
    }
}
