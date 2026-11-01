#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace game {
// Units: metres, kg O2, litres water, kWh. One game day = 240 simulation seconds.
// Cabin target is 70 kPa, 293 K, 30% O2; inert buffer gas is an assumed reserve.
// Electrolysis supplies oxygen only, with hydrogen vented: 9/8 kg H2O per kg O2.
inline constexpr double kOxygenKgPerM3=70000.0*0.30*0.032/(8.314462618*293.0);
inline constexpr double kWaterPerOxygen=1.125;
enum SupplyWarning : std::uint32_t { NoPower=1, NoWater=2, LowAir=4, LowStock=8, NoStorage=16, NoFood=32, NoMedical=64 };
struct SupplyState {
    double oxygen_kg{0}, energy_kwh{0}, water_litres{0};
    // Derived each simulation tick; persisted quantities above survive save/load.
    double volume_m3{0}, water_to_fill_litres{0}, power_fraction{0}, network_water_to_fill_litres{0};
    // Measured rates from the last simulation update, per in-game day.
    // These are derived, never serialized or used as an inventory source.
    double oxygen_produced_kg_per_day{0}, water_processed_litres_per_day{0};
    double oxygen_net_kg_per_day{0}, air_change_percentage_points_per_day{0};
    std::uint32_t warnings{0};
    double breathability() const {return volume_m3>0?std::clamp(100.0*oxygen_kg/(volume_m3*kOxygenKgPerM3),0.0,100.0):0;}
};
}
