#pragma once
#include "game/runtime/presentation.hpp"
#include "game/systems/resource_management.hpp"
#include <cstdint>
namespace game {
enum class HabitatKind;
enum class InteriorUnitKind : std::uint8_t { MetalFabricator, PlasticFabricator, OreRefinery, MedicalSynthesizer, HydroponicRack, ServiceTerminal, FoodProcessor, DrinkingWell, DiningTable, SeatingBench, OxygenGenerator, Count };
enum class InteriorUnitSize : std::uint8_t { VerySmall, Small, Medium, Count };
struct InteriorUnit {
    InteriorUnitKind kind{InteriorUnitKind::MetalFabricator};
    InteriorUnitSize size{InteriorUnitSize::Medium}; // Earlier saves retain the original full-size plant.
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
const InteriorUnitProfile& interior_unit_profile(InteriorUnitKind,InteriorUnitSize);
inline const InteriorUnitProfile& interior_unit_profile(const InteriorUnit& unit){return interior_unit_profile(unit.kind,unit.size);}
inline float oxygen_unit_factor(InteriorUnitSize size){return size==InteriorUnitSize::VerySmall?.25f:size==InteriorUnitSize::Small?.5f:1.f;}
inline double oxygen_unit_capacity(const InteriorUnit& unit){return unit.kind==InteriorUnitKind::OxygenGenerator?18.*oxygen_unit_factor(unit.size):0.;}
inline const char* interior_unit_size_name(InteriorUnitSize size){return size==InteriorUnitSize::VerySmall?"VERY SMALL":size==InteriorUnitSize::Small?"SMALL":"MEDIUM";}
const char* interior_unit_recipe(InteriorUnitKind);
const char* interior_unit_recipe(InteriorUnitKind,InteriorUnitSize);
inline const char* interior_unit_recipe(const InteriorUnit& unit){return interior_unit_recipe(unit.kind,unit.size);}
bool interior_unit_allowed(HabitatKind,InteriorUnitKind);
}
