#pragma once
#include "game/util/world_math.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_builder.hpp"
#include <array>

namespace game::lunar_look {

struct Fixture {
    eng::Vec3 position;
    eng::Vec3 size;
    eng::Color color;
    float reach;
    float power;

    eng::LunarLight light() const {
        return {position+eng::Vec3{0,size.y*0.5f+0.15f,0},color,reach,power};
    }
};

inline bool powered(const SupplyState& supply,bool enabled){
    return enabled && supply.power_fraction>=0.5;
}

inline Fixture habitat_fixture(const PlacedHabitat& habitat,const HabitatMesh& mesh){
    const bool airlock=habitat.kind==HabitatKind::AirLock;
    const eng::Vec3 offset{0,mesh.walkable_floor_y+0.14f,mesh.footprint_z*0.48f};
    return {habitat.position+world_math::rotate_y(offset,habitat.rotation_degrees),
        {0.86f,0.13f,0.18f},airlock?eng::Color{0.32f,0.83f,1.0f,1}:eng::Color{1.0f,0.69f,0.36f,1},
        12.0f,46.0f};
}

inline std::array<Fixture,4> pad_fixtures(const PlacedResource& pad,const ResourceMesh& mesh){
    std::array<Fixture,4> out{};
    std::size_t i=0;
    for(float x:{-1.0f,1.0f}) for(float z:{-1.0f,1.0f}){
        const eng::Vec3 offset{x*mesh.footprint_x*0.2881f,1.99f,z*mesh.footprint_z*0.2881f};
        out[i++]={pad.position+world_math::rotate_y(offset,pad.rotation_degrees),
            {0.45f,0.12f,0.45f},{0.30f,0.87f,1.0f,1},15.0f,76.0f};
    }
    return out;
}

inline void draw_fixture(eng::Renderer& r,const Fixture& fixture,float rotation){
    resources::Builder model;
    const eng::Vec3 side=world_math::rotate_y({1,0,0},rotation);
    const eng::Vec3 forward=world_math::rotate_y({0,0,1},rotation);
    model.oriented_box(model.mesh.body,fixture.position,side,{0,1,0},forward,fixture.size);
    r.draw_lunar_surface(model.mesh.body,eng::LunarMaterial::Emissive,fixture.color);
}

} // namespace game::lunar_look
