#pragma once
#include "engine/engine.hpp"
#include <cmath>
#include <span>

namespace eng::lunar {

class LightFader {
public:
    std::span<const LunarLight> update(const LunarScene& scene,std::size_t budget,std::size_t limit=64){
        budget=std::min(budget,slots_.size());
        if(budget==0){
            reset();
            return {};
        }
        const float elapsed=scene.seconds-previous_seconds_;
        const bool fresh=!initialized_ || elapsed<0.0f || elapsed>0.8f || previous_budget_!=budget;
        if(fresh) slots_.fill({});
        initialized_=true;
        previous_seconds_=scene.seconds;
        previous_budget_=budget;
        const float blend=fresh?1.0f:1.0f-std::exp(-std::clamp(elapsed,1.0f/240,0.1f)/0.085f);

        const std::size_t count=std::min({scene.light_count,budget,scene.lights.size()});
        // Spare slots let outgoing and incoming fixtures crossfade in place.
        std::size_t capacity=std::min({budget*2,limit,slots_.size()});
        std::array<bool,64> matched{};
        std::array<const LunarLight*,64> targets{};
        for(std::size_t i=0;i<count;++i){
            const auto& candidate=scene.lights[i];
            if(candidate.power<=0.0f || candidate.radius<=0.0f) continue;
            for(std::size_t slot=0;slot<capacity;++slot){
                const auto& old=slots_[slot];
                if(targets[slot] || old.radius!=candidate.radius ||
                   dot(old.position-candidate.position,old.position-candidate.position)>1e-6f) continue;
                targets[slot]=&candidate;
                matched[i]=true;
                break;
            }
        }
        for(std::size_t slot=0;slot<capacity;++slot){
            auto& old=slots_[slot];
            const auto* target=targets[slot];
            if(target){
                const float power=std::lerp(old.power,target->power,blend);
                old=*target;
                old.power=power;
            }else{
                old.power*=1.0f-blend;
                if(old.power<0.012f) old={};
            }
        }
        std::size_t free=0;
        for(std::size_t i=0;i<count;++i){
            const auto& candidate=scene.lights[i];
            if(matched[i] || candidate.power<=0.0f || candidate.radius<=0.0f) continue;
            while(free<capacity && slots_[free].radius>0.0f) ++free;
            if(free==capacity) break;
            slots_[free]=candidate;
            slots_[free++].power*=blend;
        }
        while(capacity && slots_[capacity-1].radius<=0.0f) --capacity;
        return {slots_.data(),capacity};
    }
    void reset(){
        slots_.fill({});
        initialized_=false;
    }
private:
    std::array<LunarLight,64> slots_{};
    float previous_seconds_{};
    std::size_t previous_budget_{};
    bool initialized_{};
};

} // namespace eng::lunar
