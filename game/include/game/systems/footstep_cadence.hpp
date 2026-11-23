#pragma once
#include <algorithm>
#include <cmath>

namespace game::sound {
// Advances from actual travel, so pushing against a wall never plays footsteps.
// One phase is one boot contact, independent of render rate or colony speed.
class FootstepCadence {
public:
    void reset(){phase_=.5f;}
    bool advance(float distance,float seconds,bool grounded){
        if(!grounded || seconds<=0 || seconds>.25f || distance>2.f || distance/seconds<.12f){reset();return false;}
        const float speed=distance/seconds;
        const float contacts_per_second=std::clamp(1.3f+.32f*speed,1.5f,2.9f);
        phase_+=seconds*contacts_per_second;
        if(phase_<1.f)return false;
        phase_=std::fmod(phase_,1.f);
        return true;
    }
private:
    float phase_{.5f};
};
} // namespace game::sound
