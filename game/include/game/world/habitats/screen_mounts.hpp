#pragma once
#include "game/world/structure_display.hpp"
#include "game/world/supply_state.hpp"

namespace game::structure_display {
Content environment(const char* title,const SupplyState& supply);
inline Mount jamb_mount(Vec3 base,Vec3 normal,float half_width,float depth,float face){
    const Vec3 side{-normal.z,0,normal.x};
    return {base+side*(half_width+.48f)+Vec3{0,1.42f,0}+normal*(face*depth),
            side*(-face),{0,1,0},.78f,.44f};
}
void draw_door_pair(runtime::SceneRenderer&,Vec3 base,Vec3 normal,float half_width,float depth,
                    const Content&,bool powered,Vec3 eye);
}
