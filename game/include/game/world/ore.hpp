#pragma once
#include "engine/engine.hpp"
#include "game/systems/resource_management.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace game::ore {

inline constexpr std::array materials{MaterialResource::Iron,MaterialResource::Copper,
    MaterialResource::Silicon,MaterialResource::Aluminum};
inline constexpr std::array<const char*,materials.size()> names{"IRON ORE","COPPER ORE","SILICON ORE","ALUMINUM ORE"};
inline constexpr std::array<eng::Color,materials.size()> colors{{
    {0.93f,0.32f,0.27f,1},{0.97f,0.62f,0.22f,1},{0.42f,0.78f,1,1},{0.60f,0.88f,0.37f,1}}};
inline constexpr auto no_cargo=static_cast<std::uint8_t>(materials.size());
inline constexpr std::int64_t box_kg=5;
inline constexpr std::int64_t refined_box_kg=3;
inline constexpr double stock_capacity_kg=60; // per ore
inline constexpr double extraction_kg_per_day=120;
inline constexpr float belt_speed_mps=2;
inline constexpr float belt_half_width=.48f;
inline constexpr float belt_height=.38f;

struct Haul {
    std::uint64_t source{0}, destination{0};
    std::uint8_t kind{no_cargo};
    bool loaded{false};
    eng::Vec3 goal{};
    bool goal_valid{false};
    bool active() const { return kind<no_cargo; }
};

using Mix=std::array<double,materials.size()>;
inline Mix deposit(eng::Vec3 p){
    // Continuous deposits sampled in world space. The same location yields
    // the same mix regardless of playfield size, orientation or save/load.
    Mix result{};
    double total=0;
    for(std::size_t i=0;i<result.size();++i){
        const double phase=static_cast<double>(i);
        const double vein=.5+.5*std::sin(p.x*(.059+.011*phase)+p.z*(.093-.009*phase)+phase*2.15);
        const double pocket=.5+.5*std::cos(p.z*.067-p.x*.042+phase*1.57);
        result[i]=.035+vein*vein*pocket;
        total+=result[i];
    }
    for(auto& share:result)share/=total;
    return result;
}

inline int boxes(double kg){ return static_cast<int>(kg/box_kg); }

} // namespace game::ore
