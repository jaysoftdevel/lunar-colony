#pragma once
#include "game/runtime/presentation.hpp"
#include "game/systems/resource_management.hpp"
#include <cstdint>
namespace game {
enum class HabitatKind;
enum class InteriorUnitKind : std::uint8_t { MetalFabricator, PlasticFabricator, OreRefinery, MedicalSynthesizer, HydroponicRack, ServiceTerminal, FoodProcessor, DrinkingWell, DiningTable, SeatingBench, OxygenGenerator, Count };
struct InteriorUnit {
    InteriorUnitKind kind{InteriorUnitKind::MetalFabricator};
    runtime::Vec3 position{}; // Local feet position, on the host deck.
    float rotation_degrees{};
    bool enabled{true},batch_active{false};
    bool gate_reserved{false}; // Derived, not serialized.
    double progress{};
    std::uint8_t ore_kind{},grow_bay{};
    std::uint32_t batches{};
    double water_litres{},food_kg{}; // Finite local service inventories.
};
struct InteriorUnitProfile {
    const char* name;runtime::Vec3 size;ResourceBundle cost;float power_kw;double cycle_days;
    ResourceBundle input;MaterialResource output;std::int64_t output_kg;
};
const InteriorUnitProfile& interior_unit_profile(InteriorUnitKind);
const char* interior_unit_recipe(InteriorUnitKind);
bool interior_unit_allowed(HabitatKind,InteriorUnitKind);
}
