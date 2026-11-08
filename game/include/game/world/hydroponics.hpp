#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace game {

enum class HydroponicPlant : std::uint8_t {
    Empty = 0,
    Potato,
    Wheat,
    Soybean,
    Pea,
    Lettuce,
    Tomato,
    Strawberry,
    Count
};

struct HydroponicPlantProfile {
    const char* name;
    const char* role;
    double growth_days;
    std::int64_t harvest_food_kg;
    double water_litres_per_day;
    double power_kw;
};

struct HydroponicsBay {
    HydroponicPlant plant{HydroponicPlant::Empty};
    double growth_fraction{0.0};
    std::uint32_t harvests{0};
};

inline constexpr std::size_t kMaxHydroponicsBays=8;
inline constexpr auto kCropKindCount=static_cast<std::size_t>(HydroponicPlant::Count)-1;
inline constexpr std::uint8_t kMaxSeedPacketsPerCrop=99;
using CropSeeds=std::array<std::uint8_t,kCropKindCount>;
inline constexpr CropSeeds kStarterCropSeeds=[]{
    CropSeeds seeds{};
    seeds.fill(3);
    return seeds;
}();

constexpr int seed_packet_total(const CropSeeds& seeds){
    int total=0;
    for(const auto count:seeds)total+=count;
    return total;
}

inline bool plant_hydroponic_crop(HydroponicsBay& bay,CropSeeds& seeds,HydroponicPlant plant){
    if(plant>=HydroponicPlant::Count)return false;
    if(plant==bay.plant)return true;
    if(plant!=HydroponicPlant::Empty){
        auto& packets=seeds[static_cast<std::size_t>(plant)-1];
        if(packets==0)return false;
        --packets;
    }
    bay.plant=plant;
    bay.growth_fraction=0.0;
    return true;
}

inline const HydroponicPlantProfile& hydroponic_profile(HydroponicPlant plant){
    static constexpr std::array<HydroponicPlantProfile,(std::size_t)HydroponicPlant::Count> profiles{{
        {"EMPTY",      "unused bay",                 1.0,  0, 0.0, 0.00},
        {"POTATO",     "calorie staple",             3.0, 18, 3.2, 0.30},
        {"WHEAT",      "carbohydrate / grain",       4.5, 16, 2.6, 0.34},
        {"SOYBEAN",    "protein / oil",              4.0, 13, 2.8, 0.32},
        {"PEA",        "protein / crop diversity",   3.0, 11, 2.4, 0.26},
        {"LETTUCE",    "fast fresh greens",          1.5,  7, 1.8, 0.22},
        {"TOMATO",     "vitamins / fresh produce",   3.5, 10, 2.7, 0.30},
        {"STRAWBERRY", "vitamin C / crew morale",    4.0,  8, 2.3, 0.28},
    }};
    const auto i=(std::size_t)plant;
    return profiles[i<profiles.size()?i:0];
}

inline HydroponicPlant next_hydroponic_plant(HydroponicPlant plant,int direction=1){
    const int count=(int)HydroponicPlant::Count;
    int value=(int)plant;
    value=(value+(direction>=0?1:-1)+count)%count;
    return (HydroponicPlant)value;
}

// Controlled greenhouse photoperiod. The Bio Dome uses artificial grow lights
// for a 16 h light / 8 h dark cycle, independent of the exterior lunar sky.
// Climate-control/base electrical load remains active through both phases.
inline constexpr float kHydroponicsLightStartHour=6.0f;
inline constexpr float kHydroponicsLightEndHour=22.0f;
inline constexpr float kHydroponicsLightHours=16.0f;
inline constexpr float kHydroponicsDarkHours=8.0f;

inline float normalize_hydroponics_hour(float hour){
    while(hour<0.0f) hour+=24.0f;
    while(hour>=24.0f) hour-=24.0f;
    return hour;
}

inline bool hydroponics_lights_on(float hour){
    const float h=normalize_hydroponics_hour(hour);
    return h>=kHydroponicsLightStartHour && h<kHydroponicsLightEndHour;
}

inline float hydroponics_hours_until_phase_change(float hour){
    const float h=normalize_hydroponics_hour(hour);
    if(hydroponics_lights_on(h)) return kHydroponicsLightEndHour-h;
    if(h<kHydroponicsLightStartHour) return kHydroponicsLightStartHour-h;
    return 24.0f-h+kHydroponicsLightStartHour;
}

} // namespace game
