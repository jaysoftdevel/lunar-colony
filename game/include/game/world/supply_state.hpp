#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace game {
// Units: metres, kg O2, litres water, kWh. One game day = 240 simulation seconds.
// Cabin target is 70 kPa, 293 K, 30% O2; inert buffer gas is an assumed reserve.
// Electrolysis mass balance: 9/8 kg H2O per kg O2 and 1/8 kg H2 per kg O2.
// Hydrogen is buffered at the oxygen generator and can be sent through a
// connected supply-channel component to an outdoor fuel-cell generator.
inline constexpr double kOxygenKgPerM3=70000.0*0.30*0.032/(8.314462618*293.0);
inline constexpr double kWaterPerOxygen=1.125;
inline constexpr double kHydrogenPerOxygen=0.125;
inline constexpr double kHydrogenLowerHeatingValueKwhPerKg=33.3;
inline constexpr double kFuelCellElectricalEfficiency=0.55;
inline constexpr double kHydrogenElectricalKwhPerKg=kHydrogenLowerHeatingValueKwhPerKg*kFuelCellElectricalEfficiency;
inline constexpr double kHydrogenGeneratorPeakKw=3.0;
inline constexpr double kOxygenGeneratorHydrogenBufferKg=4.5;
inline constexpr double kSolarPeakKw=16.0;
inline constexpr double kCrewOxygenKgPerDay=0.84;
inline constexpr double kCrewWaterLitresPerDay=4.0;
// Conservative sealed-habitat oxygen loss. This represents normal hatch/seal
// leakage and small maintenance losses, not a catastrophic pressure breach.
inline constexpr double kPressurizedOxygenLeakFractionPerDay=0.001;
enum SupplyWarning : std::uint32_t { NoPower=1, NoWater=2, LowAir=4, LowStock=8, NoStorage=16, NoFood=32, NoMedical=64, NoHydrogen=128 };
struct SupplyState {
    double oxygen_kg{0}, energy_kwh{0}, water_litres{0}, hydrogen_kg{0};
    // Derived each simulation tick; persisted quantities above survive save/load.
    double volume_m3{0}, water_to_fill_litres{0}, power_fraction{0}, network_water_to_fill_litres{0};
    // Measured rates from the last simulation update, per in-game day.
    // These are derived, never serialized or used as an inventory source.
    double oxygen_produced_kg_per_day{0}, water_processed_litres_per_day{0};
    double hydrogen_produced_kg_per_day{0}, hydrogen_vented_kg_per_day{0};
    double hydrogen_consumed_kg_per_day{0}, hydrogen_power_kw{0};
    double oxygen_net_kg_per_day{0}, air_change_percentage_points_per_day{0};
    // Derived demand telemetry for UI/debugging. These never create or store
    // resources and are recalculated every simulation update.
    double power_demand_kw{0}, water_demand_litres_per_day{0}, oxygen_demand_kg_per_day{0};
    std::uint32_t warnings{0};
    double breathability() const {return volume_m3>0?std::clamp(100.0*oxygen_kg/(volume_m3*kOxygenKgPerM3),0.0,100.0):0;}
};
}
